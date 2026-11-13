// ============================================================================
// axi_slv_bridge : requester.  AXI4 slave port -> MWr / MRd / Cfg TLPs,
//                  completions -> R / B channels.
//   slv_wr_path, wr_coalescer, slv_rd_path (incl. rd_return), rd_tag_table
//   (cpl_buf inside).  Rules AX-1..AX-9, AX-14, TL-12..TL-16, WC-*, OP4-*.
// ============================================================================
module axi_slv_bridge
  import pcie_pkg::*;
#(
  parameter int OPT_EN   = 1,
  parameter int MAX_TAGS = 16,
  parameter logic [15:0] DEVICE_ID  = 16'h0100,
  parameter logic [63:0] ECAM_BASE  = 64'h0000_0000_F000_0000
) (
  input  logic         clk,
  input  logic         rst_n,
  input  logic         dl_up,
  // knobs
  input  logic [2:0]   mps_code,
  input  logic [2:0]   mrrs_code,
  input  logic         wr_coalesce_en,
  input  logic         rd_ooo_en,
  input  logic [15:0]  coal_timeout,
  input  logic [7:0]   max_outstanding_rd,
  input  logic [31:0]  cpl_timeout,
  input  logic         coal_flush_csr,
  // AXI slave port
  input  logic [3:0]   s_awid,   input logic [63:0] s_awaddr, input logic [7:0] s_awlen,
  input  logic [2:0]   s_awsize, input logic [1:0]  s_awburst, input logic s_awvalid, output logic s_awready,
  input  logic [127:0] s_wdata,  input logic [15:0] s_wstrb, input logic s_wlast, input logic s_wvalid, output logic s_wready,
  output logic [3:0]   s_bid,    output logic [1:0] s_bresp, output logic s_bvalid, input logic s_bready,
  input  logic [3:0]   s_arid,   input logic [63:0] s_araddr, input logic [7:0] s_arlen,
  input  logic [2:0]   s_arsize, input logic [1:0]  s_arburst, input logic s_arvalid, output logic s_arready,
  output logic [3:0]   s_rid,    output logic [127:0] s_rdata, output logic [1:0] s_rresp, output logic s_rlast,
  output logic         s_rvalid, input logic s_rready,
  // to tl_tx
  output logic         p_valid, input logic p_ready, output logic [127:0] p_data, output logic [3:0] p_keep,
  output logic         p_sop,   output logic p_eop, output logic [15:0] p_age,
  input  logic [15:0]  age_now, output logic age_take, output logic open_valid, output logic [15:0] open_age,
  input  logic         coal_flush_tl,
  output logic         np_valid, input logic np_ready, output logic [127:0] np_data, output logic [3:0] np_keep,
  // completions from tl_rx
  input  logic         cp_valid, input logic [127:0] cp_data, input logic [3:0] cp_keep,
  input  logic         cp_sop,   input logic cp_eop, input logic cp_bad_v, input logic [7:0] cp_bad_tag,
  // events / status
  output logic         ev_axi_unsup,
  output logic         ev_unexpected_cpl,
  output logic         ev_cpl_err_rx,
  output logic         ev_cpl_timeout,
  output logic         ev_rd_issued,
  output logic         ev_lat_valid,
  output logic [31:0]  ev_lat_val,
  output logic         ev_hol_stall,
  output logic         ev_merged,
  output logic         ev_flush_timeout,
  output logic         txn_pending,
  output logic         idle
);
  localparam int TW = $clog2(MAX_TAGS);

  // ---- tag table ---------------------------------------------------------------
  logic [TW-1:0] al_tag; logic al_free, al_free_mem, al_go;
  logic [1:0] al_kind; logic [3:0] al_burst; logic [5:0] al_sub; logic [11:0] al_addr; logic [9:0] al_len;
  logic fr_en, fr_en2; logic [TW-1:0] fr_tag, fr_tag2;
  logic [MAX_TAGS-1:0] t_valid, t_done, t_err;
  logic [MAX_TAGS-1:0][3:0] t_burst; logic [MAX_TAGS-1:0][5:0] t_sub; logic [MAX_TAGS-1:0][1:0] t_kind;
  logic [MAX_TAGS-1:0][15:0] t_done_cyc;
  logic cb_re; logic [TW+4:0] cb_raddr; logic [127:0] cb_rdata;
  logic [15:0] now; logic [4:0] out_cnt;
  logic [1:0] tt_kind; logic [3:0] tt_burst; logic [5:0] tt_sub; logic [11:0] tt_addr; logic [9:0] tt_len;

  rd_tag_table #(.MAX_TAGS(MAX_TAGS), .DEVICE_ID(DEVICE_ID)) u_tags (
    .clk, .rst_n, .dl_up, .cpl_timeout, .max_outstanding_rd,
    .al_tag, .al_free, .al_free_mem, .al_go, .al_kind(tt_kind), .al_burst(tt_burst), .al_sub(tt_sub),
    .al_addr(tt_addr), .al_len(tt_len),
    .fr_en, .fr_tag, .fr_en2, .fr_tag2,
    .t_valid, .t_done, .t_err, .t_burst, .t_sub, .t_kind, .t_done_cyc,
    .cp_valid, .cp_data, .cp_keep, .cp_sop, .cp_eop, .cp_bad_v, .cp_bad_tag,
    .cb_re, .cb_raddr, .cb_rdata,
    .now_o(now), .ev_unexpected_cpl, .ev_cpl_err_rx, .ev_cpl_timeout, .pending(txn_pending), .out_cnt);

  // ---- write path ----------------------------------------------------------------
  logic ck_valid, ck_ready, ck_hdr4; logic [63:0] ck_addr; logic [6:0] ck_len; logic [3:0] ck_fbe, ck_lbe;
  logic [63:0][31:0] ck_ring; logic [63:0] ck_mask;
  logic cfgw_req, cfgw_gnt; logic [25:0] cfgw_addr; logic [31:0] cfgw_data; logic [3:0] cfgw_be;
  logic [7:0] aw_acc_cnt, aw_done_cnt;
  logic ev_unsup_w, ev_unsup_r, wr_busy;

  slv_wr_path #(.ECAM_BASE(ECAM_BASE), .MAX_TAGS(MAX_TAGS)) u_wr (
    .clk, .rst_n, .dl_up, .mps_code,
    .s_awid, .s_awaddr, .s_awlen, .s_awsize, .s_awburst, .s_awvalid, .s_awready,
    .s_wdata, .s_wstrb, .s_wlast, .s_wvalid, .s_wready,
    .s_bid, .s_bresp, .s_bvalid, .s_bready,
    .ck_valid, .ck_ready, .ck_hdr4, .ck_addr, .ck_len, .ck_fbe, .ck_lbe, .ck_ring, .ck_mask,
    .cfgw_req, .cfgw_gnt, .cfgw_tag(al_tag), .cfgw_addr, .cfgw_data, .cfgw_be,
    .t_done, .t_err, .fr_en(fr_en2), .fr_tag(fr_tag2),
    .aw_acc_cnt, .aw_done_cnt, .ev_axi_unsup(ev_unsup_w), .busy(wr_busy));

  logic co_idle;
  wr_coalescer #(.OPT_EN(OPT_EN), .DEVICE_ID(DEVICE_ID)) u_co (
    .clk, .rst_n, .dl_up, .wc_en(wr_coalesce_en), .coal_timeout, .mps_code,
    .coal_flush_tl, .coal_flush_csr,
    .ck_valid, .ck_ready, .ck_hdr4, .ck_addr, .ck_len, .ck_fbe, .ck_lbe, .ck_ring, .ck_mask,
    .age_now, .age_take, .open_valid, .open_age,
    .p_valid, .p_ready, .p_data, .p_keep, .p_sop, .p_eop, .p_age,
    .ev_merged, .ev_flush_timeout, .idle(co_idle));

  // ---- read path -------------------------------------------------------------------
  logic rq_valid, rq_is_cfg, rq_gnt; logic [63:0] rq_addr; logic [7:0] rq_len_dw;
  logic rd_busy;
  slv_rd_path #(.ECAM_BASE(ECAM_BASE), .MAX_TAGS(MAX_TAGS)) u_rd (
    .clk, .rst_n, .dl_up, .mrrs_code, .rd_ooo_en(rd_ooo_en && (OPT_EN != 0)),
    .s_arid, .s_araddr, .s_arlen, .s_arsize, .s_arburst, .s_arvalid, .s_arready,
    .s_rid, .s_rdata, .s_rresp, .s_rlast, .s_rvalid, .s_rready,
    .rq_valid, .rq_is_cfg, .rq_addr, .rq_len_dw, .rq_gnt,
    .al_free, .al_free_mem, .al_kind, .al_burst, .al_sub, .al_addr, .al_len,
    .al_tag, .t_valid, .t_done, .t_err, .t_burst, .t_sub, .now,
    .fr_en, .fr_tag, .cb_re, .cb_raddr, .cb_rdata,
    .aw_done_cnt, .aw_acc_cnt,
    .ev_axi_unsup(ev_unsup_r), .ev_rd_issued, .ev_lat_valid, .ev_lat_val, .ev_hol_stall, .busy(rd_busy));
  assign ev_axi_unsup = ev_unsup_w | ev_unsup_r;

  // ---- NP request mux + TLP builder ---------------------------------------------------
  wire sel_cfgw = cfgw_req && al_free;
  wire sel_rd   = !sel_cfgw && rq_valid;
  assign np_valid = sel_cfgw || sel_rd;
  wire  push_np = np_valid && np_ready;
  assign cfgw_gnt = push_np && sel_cfgw;
  assign rq_gnt   = push_np && sel_rd;
  assign al_go    = push_np;
  assign tt_kind  = sel_cfgw ? 2'd2 : al_kind;
  assign tt_burst = sel_cfgw ? 4'd0 : al_burst;
  assign tt_sub   = sel_cfgw ? 6'd0 : al_sub;
  assign tt_addr  = sel_cfgw ? 12'd0 : al_addr;
  assign tt_len   = sel_cfgw ? 10'd4 : al_len;

  wire        rq_hdr4 = (rq_addr[63:32] != 32'b0);
  wire [7:0]  tagb    = {{(8-TW){1'b0}}, al_tag};
  wire [25:0] cadr    = sel_cfgw ? cfgw_addr : rq_addr[27:2];
  wire [31:0] cdw2    = {cadr[25:18], cadr[17:13], cadr[12:10], 4'b0, cadr[9:6], cadr[5:0], 2'b00};
  logic [31:0] n0, n1, n2, n3;
  always_comb begin
    n3 = 32'h0;
    if (sel_cfgw) begin
      n0 = {FT_CFGWR0, 24'b0}; n0[9:0] = 10'd1;
      n1 = {DEVICE_ID, tagb, 4'b0000, cfgw_be};
      n2 = cdw2; n3 = cfgw_data; np_keep = 4'b1111;
    end else if (rq_is_cfg) begin
      n0 = {FT_CFGRD0, 24'b0}; n0[9:0] = 10'd1;
      n1 = {DEVICE_ID, tagb, 4'b0000, 4'b1111};
      n2 = cdw2; np_keep = 4'b0111;
    end else begin
      n0 = {(rq_hdr4 ? FT_MRD64 : FT_MRD32), 24'b0}; n0[9:0] = {2'b0, rq_len_dw};
      n1 = {DEVICE_ID, tagb, (rq_len_dw == 8'd1) ? 4'b0000 : 4'b1111, 4'b1111};
      if (rq_hdr4) begin n2 = rq_addr[63:32]; n3 = {rq_addr[31:2], 2'b00}; np_keep = 4'b1111; end
      else         begin n2 = {rq_addr[31:2], 2'b00}; np_keep = 4'b0111; end
    end
  end
  assign np_data = {bsw32(n3) & {32{!sel_cfgw && rq_hdr4 && !rq_is_cfg}} | (n3 & {32{sel_cfgw}}),
                    bsw32(n2), bsw32(n1), bsw32(n0)};

  assign idle = co_idle && !wr_busy && !rd_busy && !txn_pending;
endmodule
