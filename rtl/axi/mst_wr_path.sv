// ============================================================================
// mst_wr_path : inbound MWr -> AXI4 master write burst (AX-10)
//   payload re-packed to destination lanes, held in a P data RAM, one AXI
//   burst per TLP.  Credits are released after the last W beat (TL-19).
// ============================================================================
module mst_wr_path
  import pcie_pkg::*;
#(
  parameter int PBUF_WORDS = 256
) (
  input  logic         clk,
  input  logic         rst_n,
  input  logic [63:0]  axi_base,
  // stream from tl_rx
  input  logic         dl_up,           // flush of the RX P buffer on link-down (6.1 DL_Inactive)
  input  logic         mw_valid,
  input  logic [127:0] mw_data,
  input  logic [3:0]   mw_keep,
  input  logic         mw_sop,
  input  logic         mw_eop,
  input  logic [27:0]  mw_off,
  input  logic [3:0]   mw_fbe,
  input  logic [3:0]   mw_lbe,
  input  logic [9:0]   mw_len,
  input  logic         mw_hdr4,
  input  logic         mw_res_v,
  input  logic         mw_res_ok,
  output logic         p_rel,
  output logic [11:0]  p_rel_data,
  // AXI master write channels
  output logic [3:0]   m_awid,
  output logic [63:0]  m_awaddr,
  output logic [7:0]   m_awlen,
  output logic [2:0]   m_awsize,
  output logic [1:0]   m_awburst,
  output logic         m_awvalid,
  input  logic         m_awready,
  output logic [127:0] m_wdata,
  output logic [15:0]  m_wstrb,
  output logic         m_wlast,
  output logic         m_wvalid,
  input  logic         m_wready,
  input  logic [3:0]   m_bid,
  input  logic [1:0]   m_bresp,
  input  logic         m_bvalid,
  output logic         m_bready,
  // status
  output logic [15:0]  b_cnt,           // B responses received
  output logic         ev_axi_mst_err,
  output logic         busy
);
  localparam int PAW = $clog2(PBUF_WORDS);
  localparam int RD  = 64;               // descriptor depth (>= P_HDR_CR + in-flight)
  localparam int RDW = $clog2(RD);

  // ---- ingest : repack engine -------------------------------------------------
  logic        e_ov; logic [127:0] e_od; logic [3:0] e_oen; logic e_of, e_ol; logic [2:0] e_occ; logic e_busy;
  wire  [2:0]  in_n = pop4(mw_keep);
  dw_repack u_ep (.clk, .rst_n, .abort(!dl_up),
    .in_valid(mw_valid), .in_first(mw_sop), .in_last(mw_eop), .in_data(mw_data), .in_n(in_n),
    .skip_i(mw_hdr4 ? 3'd4 : 3'd3), .off_i(mw_off[3:2]),
    .out_valid(e_ov), .out_data(e_od), .out_en(e_oen), .out_first(e_of), .out_last(e_ol), .occ(e_occ), .busy(e_busy));

  // RAM
  logic [15:0] wcnt;                       // words written (cumulative)
  logic            rd_re; logic [PAW-1:0] rd_addr; logic [131:0] rd_data;
  ram_sdp #(.W(132), .DEPTH(PBUF_WORDS), .LANES(1)) u_pram (
    .clk(clk), .we(e_ov), .waddr(wcnt[PAW-1:0]), .wdata({e_oen, e_od}),
    .re(rd_re), .raddr(rd_addr), .rdata(rd_data));

  // TLP bookkeeping at sop / eop
  logic [27:0] t_off; logic [3:0] t_fbe, t_lbe; logic [9:0] t_len; logic t_hdr4;
  logic [10:0] dw_acc;
  logic [15:0] exp_total;
  wire  [27:0] c_off  = mw_sop ? mw_off : t_off;
  wire  [3:0]  c_fbe  = mw_sop ? mw_fbe : t_fbe;
  wire  [3:0]  c_lbe  = mw_sop ? mw_lbe : t_lbe;
  wire  [9:0]  c_len  = mw_sop ? mw_len : t_len;
  wire         c_hdr4 = mw_sop ? mw_hdr4 : t_hdr4;
  wire  [10:0] tot_dws = (mw_sop ? 11'd0 : dw_acc) + {8'b0, in_n};
  wire  [10:0] pl_dws  = (tot_dws > (c_hdr4 ? 11'd4 : 11'd3)) ? (tot_dws - (c_hdr4 ? 11'd4 : 11'd3)) : 11'd0;
  wire  [10:0] nw_calc = (pl_dws == 0) ? 11'd0 : ((pl_dws + {9'b0, c_off[3:2]} + 11'd3) >> 2);

  // descriptor ring : written at eop, resolved by mw_res_v, consumed in order
  logic [15:0] d_start [0:RD-1];
  logic [6:0]  d_nw    [0:RD-1];
  logic [27:0] d_off   [0:RD-1];
  logic [3:0]  d_fbe   [0:RD-1];
  logic [3:0]  d_lbe   [0:RD-1];
  logic [9:0]  d_len   [0:RD-1];
  logic        d_ok    [0:RD-1];
  logic [RDW:0] d_wp, d_res, d_rp;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      t_off <= '0; t_fbe <= '0; t_lbe <= '0; t_len <= '0; t_hdr4 <= 1'b0; dw_acc <= '0; exp_total <= '0;
      d_wp <= '0; d_res <= '0; wcnt <= '0;
      for (int i = 0; i < RD; i++) begin d_start[i] <= '0; d_nw[i] <= '0; d_off[i] <= '0; d_fbe[i] <= '0; d_lbe[i] <= '0; d_len[i] <= '0; d_ok[i] <= 1'b0; end
    end else if (!dl_up) begin
      // bug B-W1: DL_Inactive flushes the P buffer. Descriptors not yet started are dropped; a burst already on
      // m_axi finishes from its own copies (c_*, iss_addr) and the DLCMSM waits for it (busy -> dll_fc rx_quiet)
      // before advertising the buffer again, so its late p_rel can no longer release credits a second time.
      wcnt <= '0; exp_total <= '0; dw_acc <= '0; d_wp <= '0; d_res <= '0;
    end else begin
      if (e_ov) wcnt <= wcnt + 16'd1;
      if (mw_valid) begin
        if (mw_sop) begin t_off <= mw_off; t_fbe <= mw_fbe; t_lbe <= mw_lbe; t_len <= mw_len; t_hdr4 <= mw_hdr4; end
        dw_acc <= mw_eop ? 11'd0 : tot_dws;
        if (mw_eop) begin
          d_start[d_wp[RDW-1:0]] <= exp_total;
          d_nw   [d_wp[RDW-1:0]] <= nw_calc[6:0];
          d_off  [d_wp[RDW-1:0]] <= c_off;
          d_fbe  [d_wp[RDW-1:0]] <= c_fbe;
          d_lbe  [d_wp[RDW-1:0]] <= c_lbe;
          d_len  [d_wp[RDW-1:0]] <= c_len;
          d_ok   [d_wp[RDW-1:0]] <= 1'b0;
          d_wp <= d_wp + 1'b1;
          exp_total <= exp_total + {5'b0, nw_calc};
        end
      end
      if (mw_res_v) begin
        d_ok[d_res[RDW-1:0]] <= mw_res_ok;
        d_res <= d_res + 1'b1;
      end
    end
  end

  // ---- AXI side FSM -----------------------------------------------------------------
  typedef enum logic [1:0] {M_IDLE, M_AW, M_W} m_st_t;
  m_st_t m_st;
  logic [27:0] c_o; logic [3:0] c_fb, c_lb; logic [9:0] c_l; logic [6:0] c_nw; logic [15:0] c_start;
  logic [6:0]  iss_left;      // beats still to be read from RAM
  logic [PAW-1:0] iss_addr;
  logic        wv;            // W beat held in RAM output register
  logic [6:0]  w_idx;         // index of the beat in the RAM output register
  logic [6:0]  iss_idx;
  logic [6:0]  last_idx;
  logic [15:0] pend_ok;

  wire head_resolved = (d_rp != d_res);
  wire [15:0] words_have = wcnt - d_start[d_rp[RDW-1:0]];
  wire head_ready = head_resolved && d_ok[d_rp[RDW-1:0]] && (words_have >= {9'b0, d_nw[d_rp[RDW-1:0]]});
  wire head_skip  = head_resolved && !d_ok[d_rp[RDW-1:0]];

  assign m_awid = 4'd0; assign m_awsize = 3'd4; assign m_awburst = 2'b01;
  assign m_awvalid = (m_st == M_AW);
  assign m_awaddr  = axi_base + {36'b0, c_o[27:4], 4'b0};
  assign m_awlen   = {1'b0, c_nw} - 8'd1;

  wire w_hs = wv && m_wready;
  assign m_wvalid = wv && (m_st == M_W);
  assign m_wdata  = rd_data[127:0];
  assign m_wlast  = (w_idx == last_idx);
  assign rd_re    = (m_st == M_W) && (iss_left != 0) && (!wv || m_wready);
  assign rd_addr  = iss_addr;

  // strobes
  logic [15:0] strb;
  always_comb begin
    strb = '0;
    for (int l = 0; l < 4; l++) begin
      logic signed [9:0] di;
      logic [3:0] bm;
      di = $signed({3'b0, w_idx}) * 4 + l - $signed({8'b0, c_o[3:2]});
      bm = 4'b0000;
      if (di >= 0 && di < $signed({1'b0, c_l}) && rd_data[128 + l]) begin
        if (c_l == 10'd1)        bm = c_fb;
        else if (di == 0)        bm = c_fb;
        else if (di == $signed({1'b0, c_l}) - 1) bm = c_lb;
        else                     bm = 4'b1111;
      end
      strb[4*l +: 4] = bm;
    end
  end
  assign m_wstrb = strb;

  assign p_rel      = w_hs && m_wlast;
  assign p_rel_data = 12'((c_l + 10'd3) >> 2);
  assign m_bready   = 1'b1;
  assign ev_axi_mst_err = m_bvalid && (m_bresp != 2'b00);
  assign busy = (m_st != M_IDLE) || (d_rp != d_wp);

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      m_st <= M_IDLE; d_rp <= '0; wv <= 1'b0; w_idx <= '0; iss_left <= '0; iss_addr <= '0; iss_idx <= '0;
      c_o <= '0; c_fb <= '0; c_lb <= '0; c_l <= '0; c_nw <= '0; c_start <= '0; last_idx <= '0; b_cnt <= '0; pend_ok <= '0;
    end else begin
      // pend_ok : accepted writes (counted in tl_rx p_acc_cnt) whose AXI burst has not started yet. A flush drops
      // them, so they are added to b_cnt at once: reads ordered behind them (AX-12, nd_snap) must not wait forever.
      if (!dl_up) begin
        b_cnt <= b_cnt + 16'(m_bvalid) + pend_ok + 16'(mw_res_v && mw_res_ok);   // a result pulse in the flush cycle counts too
        pend_ok <= '0;
        d_rp <= '0;
      end else begin
        if (m_bvalid) b_cnt <= b_cnt + 16'd1;
        pend_ok <= pend_ok + 16'(mw_res_v && mw_res_ok) - 16'(m_st == M_IDLE && !head_skip && head_ready);
      end
      case (m_st)
        M_IDLE: begin
          if (!dl_up) ;                                    // flushed: nothing to start
          else if (head_skip) d_rp <= d_rp + 1'b1;
          else if (head_ready) begin
            c_o <= d_off[d_rp[RDW-1:0]]; c_fb <= d_fbe[d_rp[RDW-1:0]]; c_lb <= d_lbe[d_rp[RDW-1:0]];
            c_l <= d_len[d_rp[RDW-1:0]]; c_nw <= d_nw[d_rp[RDW-1:0]]; c_start <= d_start[d_rp[RDW-1:0]];
            last_idx <= d_nw[d_rp[RDW-1:0]] - 7'd1;
            d_rp <= d_rp + 1'b1;
            m_st <= M_AW;
          end
        end
        M_AW: if (m_awready) begin
          m_st <= M_W;
          iss_left <= c_nw; iss_addr <= c_start[PAW-1:0]; iss_idx <= '0; wv <= 1'b0;
        end
        M_W: begin
          if (rd_re) begin
            iss_addr <= iss_addr + 1'b1; iss_left <= iss_left - 7'd1;
            wv <= 1'b1; w_idx <= iss_idx; iss_idx <= iss_idx + 7'd1;
          end else if (w_hs) wv <= 1'b0;
          if (w_hs && m_wlast) begin m_st <= M_IDLE; wv <= 1'b0; end
        end
        default: m_st <= M_IDLE;
      endcase
    end
  end
endmodule
