// ============================================================================
// pcie_ep_top : PCIe-style TL + DLL endpoint node with AXI4 bridges
//   Owns no logic except reset synchronisation, and the wiring of
//   axi_slv_bridge / axi_mst_bridge / tl_tx / tl_rx / cfg_space / dll_tx /
//   dll_rx / dll_fc / csr_regs (spec section 2.1).
//   OPT_EN = 0 removes the four optimization blocks (synthesis comparison).
// ============================================================================
module pcie_ep_top
  import pcie_pkg::*;
#(
  parameter int MPS_MAX_BYTES    = 256,
  parameter int MAX_TAGS         = 16,
  parameter int MAX_CPLR_REQS    = 8,
  parameter int REPLAY_BUF_BEATS = 256,
  parameter int MAX_REPLAY_TLPS  = 64,
  parameter int P_HDR_CR         = 32,
  parameter int P_DATA_CR        = 128,
  parameter int NP_HDR_CR        = 32,
  parameter int NP_DATA_CR       = 32,
  parameter int CPL_HDR_CR       = 0,
  parameter int CPL_DATA_CR      = 0,
  parameter int BAR0_SIZE_LOG2   = 20,
  parameter logic [15:0] DEVICE_ID = 16'h0100,
  parameter logic [63:0] ECAM_BASE = 64'h0000_0000_F000_0000,
  parameter int OPT_EN           = 1
) (
  input  logic         clk,
  input  logic         rst_n,
  input  logic [63:0]  axi_base,

  // AXI4 slave port (requester)
  input  logic [3:0]   s_axi_awid,   input logic [63:0] s_axi_awaddr, input logic [7:0] s_axi_awlen,
  input  logic [2:0]   s_axi_awsize, input logic [1:0]  s_axi_awburst, input logic s_axi_awvalid, output logic s_axi_awready,
  input  logic [127:0] s_axi_wdata,  input logic [15:0] s_axi_wstrb, input logic s_axi_wlast,
  input  logic         s_axi_wvalid, output logic s_axi_wready,
  output logic [3:0]   s_axi_bid,    output logic [1:0] s_axi_bresp, output logic s_axi_bvalid, input logic s_axi_bready,
  input  logic [3:0]   s_axi_arid,   input logic [63:0] s_axi_araddr, input logic [7:0] s_axi_arlen,
  input  logic [2:0]   s_axi_arsize, input logic [1:0]  s_axi_arburst, input logic s_axi_arvalid, output logic s_axi_arready,
  output logic [3:0]   s_axi_rid,    output logic [127:0] s_axi_rdata, output logic [1:0] s_axi_rresp,
  output logic         s_axi_rlast,  output logic s_axi_rvalid, input logic s_axi_rready,

  // AXI4 master port (completer)
  output logic [3:0]   m_axi_awid,   output logic [63:0] m_axi_awaddr, output logic [7:0] m_axi_awlen,
  output logic [2:0]   m_axi_awsize, output logic [1:0]  m_axi_awburst, output logic m_axi_awvalid, input logic m_axi_awready,
  output logic [127:0] m_axi_wdata,  output logic [15:0] m_axi_wstrb, output logic m_axi_wlast,
  output logic         m_axi_wvalid, input logic m_axi_wready,
  input  logic [3:0]   m_axi_bid,    input logic [1:0] m_axi_bresp, input logic m_axi_bvalid, output logic m_axi_bready,
  output logic [3:0]   m_axi_arid,   output logic [63:0] m_axi_araddr, output logic [7:0] m_axi_arlen,
  output logic [2:0]   m_axi_arsize, output logic [1:0]  m_axi_arburst, output logic m_axi_arvalid, input logic m_axi_arready,
  input  logic [3:0]   m_axi_rid,    input logic [127:0] m_axi_rdata, input logic [1:0] m_axi_rresp,
  input  logic         m_axi_rlast,  input logic m_axi_rvalid, output logic m_axi_rready,

  // link
  output logic         lnk_tx_valid, output logic [127:0] lnk_tx_data, output logic [15:0] lnk_tx_keep,
  output logic         lnk_tx_sop,   output logic lnk_tx_eop, output logic lnk_tx_dllp,
  input  logic         lnk_rx_valid, input logic [127:0] lnk_rx_data, input logic [15:0] lnk_rx_keep,
  input  logic         lnk_rx_sop,   input logic lnk_rx_eop, input logic lnk_rx_dllp,
  input  logic         phy_link_up,

  // CSR bus
  input  logic         csr_req, input logic csr_we, input logic [11:0] csr_addr, input logic [31:0] csr_wdata,
  output logic         csr_ack, output logic [31:0] csr_rdata,

  // status
  output logic         dl_up,
  output logic         fatal_err
);
  // ---------------------------------------------------------------------------
  // reset : asserted asynchronously, released synchronously
  // ---------------------------------------------------------------------------
  logic [1:0] rst_sync;
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) rst_sync <= 2'b00;
    else        rst_sync <= {rst_sync[0], 1'b1};
  end
  wire rstn = rst_sync[1];

  localparam int MAX_FRAME_BEATS = (MPS_MAX_BYTES + 16 + 2 + 4 + 15) / 16;

  // ---------------------------------------------------------------------------
  // CSR
  // ---------------------------------------------------------------------------
  logic link_enable, soft_reset, coal_flush_csr;
  logic opt_ack_coal, opt_ack_adapt, opt_replay_adapt, opt_fc_early, opt_wr_coal, opt_rd_ooo;
  logic [7:0] ack_coal_n; logic [15:0] ack_timer_cyc; logic [7:0] adapt_hold;
  logic [7:0] replay_win; logic [15:0] replay_timeout;
  logic [15:0] updatefc_timer; logic [7:0] fc_thr_h, fc_thr_d; logic [15:0] fc_max_delay;
  logic [15:0] coal_timeout; logic [7:0] max_out_rd; logic [31:0] cpl_timeout;
  logic [15:0] err_set, err_status;
  logic [25:0][11:0] cnt_inc;
  logic rd_lat_valid; logic [31:0] rd_lat_val;
  logic st_fc1, st_fc2, st_replay, st_txidle, st_rxidle;

  csr_regs #(.MAX_REPLAY_TLPS(MAX_REPLAY_TLPS), .MAX_TAGS(MAX_TAGS)) u_csr (
    .clk, .rst_n(rstn), .csr_req, .csr_we, .csr_addr, .csr_wdata, .csr_ack, .csr_rdata,
    .link_enable, .soft_reset, .coal_flush(coal_flush_csr),
    .opt_ack_coalesce_en(opt_ack_coal), .opt_ack_adapt_en(opt_ack_adapt), .opt_replay_adapt_en(opt_replay_adapt),
    .opt_fc_early_en(opt_fc_early), .opt_wr_coalesce_en(opt_wr_coal), .opt_rd_ooo_en(opt_rd_ooo),
    .ack_coalesce_n(ack_coal_n), .ack_timer_cyc, .adapt_hold, .replay_win, .replay_timeout,
    .updatefc_timer, .fc_early_thresh_hdr(fc_thr_h), .fc_early_thresh_data(fc_thr_d), .fc_early_max_delay(fc_max_delay),
    .coal_timeout, .max_outstanding_rd(max_out_rd), .cpl_timeout,
    .phy_link_up, .dl_up, .fc_init1_done(st_fc1), .fc_init2_done(st_fc2),
    .replay_active(st_replay), .tx_idle(st_txidle), .rx_idle(st_rxidle),
    .err_set, .err_status, .fatal_err,
    .cnt_inc, .rd_lat_valid, .rd_lat_val);

  // ---------------------------------------------------------------------------
  // DLL
  // ---------------------------------------------------------------------------
  logic dll_en; logic [1:0] dl_state;
  logic tl_tx_valid, tl_tx_ready, tl_tx_sop, tl_tx_eop; logic [127:0] tl_tx_data; logic [3:0] tl_tx_keep;
  logic dl_rx_valid, dl_rx_sop, dl_rx_eop; logic [127:0] dl_rx_data; logic [3:0] dl_rx_keep;
  logic nak_req, nak_grant, ack_req, ack_grant, fc_req, fc_grant; logic [11:0] rx_seq_m1; logic [31:0] fc_dllp;
  logic rx_ack_valid, rx_nak_valid; logic [11:0] rx_ack_seq;
  logic fc_rx_valid; logic [7:0] fc_rx_type, fc_rx_hdr; logic [11:0] fc_rx_data;
  logic fc_valid;
  logic [7:0] fc_lim_p_hdr, fc_lim_np_hdr, fc_lim_cpl_hdr; logic [11:0] fc_lim_p_data, fc_lim_np_data, fc_lim_cpl_data;
  logic fc_inf_p_hdr, fc_inf_p_data, fc_inf_np_hdr, fc_inf_np_data, fc_inf_cpl_hdr, fc_inf_cpl_data;
  logic [7:0] fc_ca_p_hdr, fc_ca_np_hdr, fc_ca_cpl_hdr; logic [11:0] fc_ca_p_data, fc_ca_np_data, fc_ca_cpl_data;
  logic ev_rollover, ev_dllp_unexp, ev_replay_to, ev_tx_tlp, ev_beat_tlp, ev_beat_dllp, ev_beat_replay;
  logic ev_stall_replay, ev_nak_rx, ev_ack_rx, ev_replay_start, ev_nak_tx, ev_ack_tx; logic [10:0] ev_tx_pl;
  logic dtx_idle, dtx_replay;
  logic ev_lcrc, ev_seq, ev_dllp_crc, ev_rx_tlp, ev_rx_dup, drx_idle; logic [11:0] ev_rx_pl;
  logic ev_updatefc;

  logic mw_busy;                         // P buffer still draining to m_axi (holds the DLCMSM in DL_Inactive after a flush)
  dll_fc #(.P_HDR_CR(P_HDR_CR), .P_DATA_CR(P_DATA_CR), .NP_HDR_CR(NP_HDR_CR), .NP_DATA_CR(NP_DATA_CR),
           .CPL_HDR_CR(CPL_HDR_CR), .CPL_DATA_CR(CPL_DATA_CR), .OPT_EN(OPT_EN)) u_fc (
    .clk, .rst_n(rstn), .phy_link_up, .link_enable, .rx_quiet(!mw_busy), .soft_reset, .rollover(ev_rollover),
    .dl_up, .dll_en, .dl_state, .fc_init1_done(st_fc1), .fc_init2_done(st_fc2),
    .fc_rx_valid, .fc_rx_type, .fc_rx_hdr, .fc_rx_data, .fc_req, .fc_dllp, .fc_grant,
    .fc_valid, .fc_lim_p_hdr, .fc_lim_p_data, .fc_lim_np_hdr, .fc_lim_np_data, .fc_lim_cpl_hdr, .fc_lim_cpl_data,
    .fc_inf_p_hdr, .fc_inf_p_data, .fc_inf_np_hdr, .fc_inf_np_data, .fc_inf_cpl_hdr, .fc_inf_cpl_data,
    .fc_ca_p_hdr, .fc_ca_p_data, .fc_ca_np_hdr, .fc_ca_np_data, .fc_ca_cpl_hdr, .fc_ca_cpl_data,
    .updatefc_timer_cyc(updatefc_timer), .early_en(opt_fc_early), .early_thresh_hdr(fc_thr_h),
    .early_thresh_data(fc_thr_d), .early_max_delay(fc_max_delay), .ev_updatefc_tx(ev_updatefc));

  dll_tx #(.REPLAY_BUF_BEATS(REPLAY_BUF_BEATS), .MAX_REPLAY_TLPS(MAX_REPLAY_TLPS), .OPT_EN(OPT_EN)) u_dtx (
    .clk, .rst_n(rstn), .dl_active(dl_up), .dll_en,
    .tl_tx_valid, .tl_tx_ready, .tl_tx_data, .tl_tx_keep, .tl_tx_sop, .tl_tx_eop,
    .nak_req, .nak_grant, .ack_req, .ack_grant, .rx_seq_m1, .fc_req, .fc_dllp, .fc_grant,
    .rx_ack_valid, .rx_nak_valid, .rx_ack_seq,
    .cfg_win(replay_win), .cfg_timeout(replay_timeout), .adapt_en(opt_replay_adapt),
    .lnk_tx_valid, .lnk_tx_data, .lnk_tx_keep, .lnk_tx_sop, .lnk_tx_eop, .lnk_tx_dllp,
    .replay_active(dtx_replay), .tx_idle(dtx_idle), .ev_rollover, .ev_dllp_unexpected(ev_dllp_unexp),
    .ev_replay_timeout(ev_replay_to), .ev_tx_tlp, .ev_tx_payload_bytes(ev_tx_pl), .ev_beat_tlp, .ev_beat_dllp,
    .ev_beat_replay, .ev_stall_replay, .ev_nak_rx, .ev_ack_rx, .ev_replay_start, .ev_nak_tx, .ev_ack_tx,
    .dbg_outstanding(), .dbg_next_tx_seq(), .dbg_ackd_seq(), .dbg_replay_num(), .dbg_replay_timer(), .dbg_timeout());

  dll_rx #(.MAX_FRAME_BEATS(MAX_FRAME_BEATS), .OPT_EN(OPT_EN)) u_drx (
    .clk, .rst_n(rstn), .dl_active(dl_up), .dll_en, .phy_link_up,
    .lnk_rx_valid, .lnk_rx_data, .lnk_rx_keep, .lnk_rx_sop, .lnk_rx_eop, .lnk_rx_dllp,
    .dl_rx_valid, .dl_rx_data, .dl_rx_keep, .dl_rx_sop, .dl_rx_eop,
    .nak_req, .nak_grant, .ack_req, .ack_grant, .rx_seq_m1, .rx_ack_valid, .rx_nak_valid, .rx_ack_seq,
    .fc_rx_valid, .fc_rx_type, .fc_rx_hdr, .fc_rx_data,
    .ack_coal_en(opt_ack_coal), .ack_adapt_en(opt_ack_adapt), .ack_coal_n(ack_coal_n),
    .ack_timer_cyc, .adapt_hold_cfg(adapt_hold),
    .ev_lcrc_err(ev_lcrc), .ev_seq_err(ev_seq), .ev_dllp_crc_err(ev_dllp_crc), .ev_rx_tlp,
    .ev_rx_payload_bytes(ev_rx_pl), .ev_rx_dup, .rx_idle(drx_idle),
    .dbg_pending_cnt(), .dbg_ack_timer(), .dbg_neff(), .dbg_adapt_active());

  // ---------------------------------------------------------------------------
  // TL
  // ---------------------------------------------------------------------------
  logic mse, bme; logic [31:0] bar0; logic [2:0] mps_code, mrrs_code;
  logic cfg_acc_valid, cfg_acc_we; logic [9:0] cfg_acc_dw; logic [31:0] cfg_acc_wdata, cfg_acc_rdata; logic [3:0] cfg_acc_be;
  logic txn_pending;

  cfg_space #(.MPS_MAX_BYTES(MPS_MAX_BYTES), .BAR0_SIZE_LOG2(BAR0_SIZE_LOG2)) u_cfg (
    .clk, .rst_n(rstn), .acc_valid(cfg_acc_valid), .acc_we(cfg_acc_we), .acc_dw(cfg_acc_dw),
    .acc_wdata(cfg_acc_wdata), .acc_be(cfg_acc_be), .acc_rdata(cfg_acc_rdata),
    .mse, .bme, .bar0, .mps_code, .mrrs_code, .err_set, .txn_pending);

  // tl_tx
  logic p_valid, p_ready, p_sop, p_eop; logic [127:0] p_data; logic [3:0] p_keep; logic [15:0] p_age, age_now, open_age;
  logic age_take, open_valid, coal_flush_tl;
  logic np_valid, np_ready; logic [127:0] np_data; logic [3:0] np_keep;
  logic cpl0_valid, cpl0_ready, cpl0_sop, cpl0_eop; logic [127:0] cpl0_data; logic [3:0] cpl0_keep;
  logic cpl1_valid, cpl1_ready, cpl1_sop, cpl1_eop; logic [127:0] cpl1_data; logic [3:0] cpl1_keep;
  logic ev_stall_credit, ttx_idle, np_pending;

  tl_tx u_ttx (
    .clk, .rst_n(rstn), .dl_up,
    .p_valid, .p_ready, .p_data, .p_keep, .p_sop, .p_eop, .p_age, .age_now, .age_take,
    .open_valid, .open_age, .coal_flush(coal_flush_tl),
    .np_valid, .np_ready, .np_data, .np_keep,
    .cpl0_valid, .cpl0_ready, .cpl0_data, .cpl0_keep, .cpl0_sop, .cpl0_eop,
    .cpl1_valid, .cpl1_ready, .cpl1_data, .cpl1_keep, .cpl1_sop, .cpl1_eop,
    .tl_tx_valid, .tl_tx_ready, .tl_tx_data, .tl_tx_keep, .tl_tx_sop, .tl_tx_eop,
    .fc_valid, .fc_lim_p_hdr, .fc_lim_p_data, .fc_lim_np_hdr, .fc_lim_np_data, .fc_lim_cpl_hdr, .fc_lim_cpl_data,
    .fc_inf_p_hdr, .fc_inf_p_data, .fc_inf_np_hdr, .fc_inf_np_data, .fc_inf_cpl_hdr, .fc_inf_cpl_data,
    .ev_stall_credit, .tx_idle(ttx_idle), .np_pending, .dbg_start(), .dbg_cc_p_hdr(), .dbg_cc_p_data());

  // tl_rx
  logic mw_valid, mw_sop, mw_eop, mw_hdr4, mw_res_v, mw_res_ok; logic [127:0] mw_data; logic [3:0] mw_keep;
  logic [27:0] mw_off; logic [3:0] mw_fbe, mw_lbe; logic [9:0] mw_len;
  logic p_rel; logic [11:0] p_rel_data;
  logic cp_valid, cp_sop, cp_eop, cp_bad_v; logic [127:0] cp_data; logic [3:0] cp_keep; logic [7:0] cp_bad_tag;
  logic nd_valid, nd_ready, np_rel; logic [27:0] nd_off; logic [9:0] nd_len; logic [3:0] nd_fbe, nd_lbe;
  logic [15:0] nd_reqid, nd_snap, p_acc_cnt; logic [7:0] nd_tag;
  logic ev_malformed, ev_poisoned, ev_ur_posted, ev_ur_sent, ev_fc_viol, trx_idle;
  logic rel_p, rel_np, rel_cpl;

  tl_rx #(.MPS_MAX_BYTES(MPS_MAX_BYTES), .BAR0_SIZE_LOG2(BAR0_SIZE_LOG2), .DEVICE_ID(DEVICE_ID),
          .P_HDR_CR(P_HDR_CR), .P_DATA_CR(P_DATA_CR), .NP_HDR_CR(NP_HDR_CR), .NP_DATA_CR(NP_DATA_CR),
          .CPL_HDR_CR(CPL_HDR_CR), .CPL_DATA_CR(CPL_DATA_CR)) u_trx (
    .clk, .rst_n(rstn), .dl_up,
    .dl_rx_valid, .dl_rx_data, .dl_rx_keep, .dl_rx_sop, .dl_rx_eop,
    .mse, .bar0, .mps_code,
    .mw_valid, .mw_data, .mw_keep, .mw_sop, .mw_eop, .mw_off, .mw_fbe, .mw_lbe, .mw_len, .mw_hdr4,
    .mw_res_v, .mw_res_ok, .p_rel, .p_rel_data,
    .cp_valid, .cp_data, .cp_keep, .cp_sop, .cp_eop, .cp_bad_v, .cp_bad_tag,
    .nd_valid, .nd_ready, .nd_off, .nd_len, .nd_fbe, .nd_lbe, .nd_reqid, .nd_tag, .nd_snap, .np_rel,
    .cpl1_valid, .cpl1_ready, .cpl1_data, .cpl1_keep, .cpl1_sop, .cpl1_eop,
    .cfg_acc_valid, .cfg_acc_we, .cfg_acc_dw, .cfg_acc_wdata, .cfg_acc_be, .cfg_acc_rdata,
    .fc_ca_p_hdr, .fc_ca_p_data, .fc_ca_np_hdr, .fc_ca_np_data, .fc_ca_cpl_hdr, .fc_ca_cpl_data,
    .fc_rel_pulse_p(rel_p), .fc_rel_pulse_np(rel_np), .fc_rel_pulse_cpl(rel_cpl),
    .ev_malformed, .ev_poisoned, .ev_ur_posted, .ev_ur_sent, .ev_fc_violation(ev_fc_viol),
    .p_acc_cnt, .rx_idle(trx_idle));

  // ---------------------------------------------------------------------------
  // AXI bridges
  // ---------------------------------------------------------------------------
  logic ev_axi_unsup, ev_unexp_cpl, ev_cpl_err, ev_cpl_to, ev_rd_issued, ev_lat_valid, ev_hol, ev_merged, ev_flush_to;
  logic [31:0] ev_lat_val; logic sb_idle;

  axi_slv_bridge #(.OPT_EN(OPT_EN), .MAX_TAGS(MAX_TAGS), .DEVICE_ID(DEVICE_ID), .ECAM_BASE(ECAM_BASE)) u_slv (
    .clk, .rst_n(rstn), .dl_up, .mps_code, .mrrs_code, .wr_coalesce_en(opt_wr_coal), .rd_ooo_en(opt_rd_ooo),
    .coal_timeout, .max_outstanding_rd(max_out_rd), .cpl_timeout, .coal_flush_csr,
    .s_awid(s_axi_awid), .s_awaddr(s_axi_awaddr), .s_awlen(s_axi_awlen), .s_awsize(s_axi_awsize), .s_awburst(s_axi_awburst),
    .s_awvalid(s_axi_awvalid), .s_awready(s_axi_awready),
    .s_wdata(s_axi_wdata), .s_wstrb(s_axi_wstrb), .s_wlast(s_axi_wlast), .s_wvalid(s_axi_wvalid), .s_wready(s_axi_wready),
    .s_bid(s_axi_bid), .s_bresp(s_axi_bresp), .s_bvalid(s_axi_bvalid), .s_bready(s_axi_bready),
    .s_arid(s_axi_arid), .s_araddr(s_axi_araddr), .s_arlen(s_axi_arlen), .s_arsize(s_axi_arsize), .s_arburst(s_axi_arburst),
    .s_arvalid(s_axi_arvalid), .s_arready(s_axi_arready),
    .s_rid(s_axi_rid), .s_rdata(s_axi_rdata), .s_rresp(s_axi_rresp), .s_rlast(s_axi_rlast), .s_rvalid(s_axi_rvalid), .s_rready(s_axi_rready),
    .p_valid, .p_ready, .p_data, .p_keep, .p_sop, .p_eop, .p_age, .age_now, .age_take, .open_valid, .open_age,
    .coal_flush_tl, .np_valid, .np_ready, .np_data, .np_keep,
    .cp_valid, .cp_data, .cp_keep, .cp_sop, .cp_eop, .cp_bad_v, .cp_bad_tag,
    .ev_axi_unsup, .ev_unexpected_cpl(ev_unexp_cpl), .ev_cpl_err_rx(ev_cpl_err), .ev_cpl_timeout(ev_cpl_to),
    .ev_rd_issued, .ev_lat_valid, .ev_lat_val, .ev_hol_stall(ev_hol), .ev_merged, .ev_flush_timeout(ev_flush_to),
    .txn_pending, .idle(sb_idle));

  logic [15:0] wr_b_cnt; logic ev_mst_err_w, ev_mst_err_r, mr_busy;
  mst_wr_path u_mw (
    .clk, .rst_n(rstn), .axi_base, .dl_up,
    .mw_valid, .mw_data, .mw_keep, .mw_sop, .mw_eop, .mw_off, .mw_fbe, .mw_lbe, .mw_len, .mw_hdr4,
    .mw_res_v, .mw_res_ok, .p_rel, .p_rel_data,
    .m_awid(m_axi_awid), .m_awaddr(m_axi_awaddr), .m_awlen(m_axi_awlen), .m_awsize(m_axi_awsize), .m_awburst(m_axi_awburst),
    .m_awvalid(m_axi_awvalid), .m_awready(m_axi_awready),
    .m_wdata(m_axi_wdata), .m_wstrb(m_axi_wstrb), .m_wlast(m_axi_wlast), .m_wvalid(m_axi_wvalid), .m_wready(m_axi_wready),
    .m_bid(m_axi_bid), .m_bresp(m_axi_bresp), .m_bvalid(m_axi_bvalid), .m_bready(m_axi_bready),
    .b_cnt(wr_b_cnt), .ev_axi_mst_err(ev_mst_err_w), .busy(mw_busy));

  mst_rd_path #(.MAX_CPLR_REQS(MAX_CPLR_REQS), .DEVICE_ID(DEVICE_ID)) u_mr (
    .clk, .rst_n(rstn), .dl_up, .axi_base, .mps_code,
    .nd_valid, .nd_ready, .nd_off, .nd_len, .nd_fbe, .nd_lbe, .nd_reqid, .nd_tag, .nd_snap, .np_rel, .wr_b_cnt,
    .m_arid(m_axi_arid), .m_araddr(m_axi_araddr), .m_arlen(m_axi_arlen), .m_arsize(m_axi_arsize), .m_arburst(m_axi_arburst),
    .m_arvalid(m_axi_arvalid), .m_arready(m_axi_arready),
    .m_rid(m_axi_rid), .m_rdata(m_axi_rdata), .m_rresp(m_axi_rresp), .m_rlast(m_axi_rlast), .m_rvalid(m_axi_rvalid),
    .m_rready(m_axi_rready),
    .cpl0_valid, .cpl0_ready, .cpl0_data, .cpl0_keep, .cpl0_sop, .cpl0_eop,
    .ev_axi_mst_err(ev_mst_err_r), .busy(mr_busy));

  // ---------------------------------------------------------------------------
  // events -> CSR
  // ---------------------------------------------------------------------------
  always_comb begin
    err_set = '0;
    err_set[ERR_LCRC_ERR_RX]         = ev_lcrc;
    err_set[ERR_SEQ_ERR_RX]          = ev_seq;
    err_set[ERR_DLLP_CRC_ERR]        = ev_dllp_crc;
    err_set[ERR_REPLAY_NUM_ROLLOVER] = ev_rollover;
    err_set[ERR_REPLAY_TIMEOUT]      = ev_replay_to;
    err_set[ERR_MALFORMED]           = ev_malformed;
    err_set[ERR_UR_SENT]             = ev_ur_sent;
    err_set[ERR_POISONED]            = ev_poisoned;
    err_set[ERR_CPL_TIMEOUT]         = ev_cpl_to;
    err_set[ERR_UNEXPECTED_CPL]      = ev_unexp_cpl;
    err_set[ERR_CPL_ERR_RX]          = ev_cpl_err;
    err_set[ERR_AXI_UNSUP]           = ev_axi_unsup;
    err_set[ERR_FC_VIOLATION]        = ev_fc_viol;
    err_set[ERR_DLLP_UNEXPECTED]     = ev_dllp_unexp;
    err_set[ERR_UR_POSTED]           = ev_ur_posted;
    err_set[ERR_AXI_MST_ERR]         = ev_mst_err_w | ev_mst_err_r;
  end

  always_comb begin
    cnt_inc = '0;
    cnt_inc[1]  = {11'b0, ev_tx_tlp};
    cnt_inc[2]  = ev_tx_tlp ? {1'b0, ev_tx_pl} : 12'd0;
    cnt_inc[3]  = {11'b0, ev_beat_tlp};
    cnt_inc[4]  = {11'b0, ev_beat_dllp};
    cnt_inc[5]  = {11'b0, ev_beat_replay};
    cnt_inc[6]  = {11'b0, ev_stall_credit};
    cnt_inc[7]  = {11'b0, ev_stall_replay};
    cnt_inc[8]  = {11'b0, ev_rx_tlp};
    cnt_inc[9]  = ev_rx_tlp ? ev_rx_pl : 12'd0;
    cnt_inc[10] = {11'b0, ev_lcrc};
    cnt_inc[11] = {11'b0, ev_rx_dup};
    cnt_inc[12] = {11'b0, ev_nak_tx};
    cnt_inc[13] = {11'b0, ev_nak_rx};
    cnt_inc[14] = {11'b0, ev_ack_tx};
    cnt_inc[15] = {11'b0, ev_ack_rx};
    cnt_inc[16] = {11'b0, ev_replay_start};
    cnt_inc[17] = {11'b0, ev_replay_to};
    cnt_inc[18] = {11'b0, ev_updatefc};
    cnt_inc[19] = {11'b0, ev_merged};
    cnt_inc[20] = {11'b0, ev_rd_issued};
    cnt_inc[24] = {11'b0, ev_hol};
    cnt_inc[25] = {11'b0, ev_flush_to};
  end
  assign rd_lat_valid = ev_lat_valid;
  assign rd_lat_val   = ev_lat_val;

  assign st_replay  = dtx_replay;
  assign st_txidle  = ttx_idle && dtx_idle && sb_idle && !mw_busy && !mr_busy;
  assign st_rxidle  = drx_idle && trx_idle;
endmodule
