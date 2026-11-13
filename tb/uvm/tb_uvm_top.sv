// ============================================================================
// tb_uvm_top : two pcie_ep_top nodes + link_model + interfaces, then run_test()
//   node 0 = A, node 1 = B.   direction 0 = A->B, direction 1 = B->A
// ============================================================================
`timescale 1ns/1ps
`include "uvm_macros.svh"
module tb_uvm_top;
  import uvm_pkg::*;
  import pcie_uvm_pkg::*;
  parameter int LAT = 8;
  parameter int MAX_TAGS_P = 16;

  logic clk = 0, rst_n = 0;
  always #2 clk = ~clk;

  axi_if s_axi0 (clk, rst_n), s_axi1 (clk, rst_n);       // application side, TB drives
  axi_if m_axi0 (clk, rst_n), m_axi1 (clk, rst_n);       // memory side, TB responds
  csr_if csr0 (clk, rst_n), csr1 (clk, rst_n);
  lnk_if tap_a2b_in (clk, rst_n), tap_a2b_out (clk, rst_n), tap_b2a_in (clk, rst_n), tap_b2a_out (clk, rst_n);
  fi_if  fi_a2b (clk, rst_n), fi_b2a (clk, rst_n);

  logic tx_v [2], tx_s [2], tx_e [2], tx_d [2], rx_v [2], rx_s [2], rx_e [2], rx_d [2];
  logic [127:0] tx_dat [2], rx_dat [2]; logic [15:0] tx_k [2], rx_k [2];
  logic phy [2];

  // ---- node A ----
`ifdef FINITE_CPL
  // T26 build variant: finite completion credits (FC-3, A-FC-7 for the Cpl class)
  localparam int CPL_H = 8, CPL_D = 64;
`else
  localparam int CPL_H = 0, CPL_D = 0;
`endif
  pcie_ep_top #(.DEVICE_ID(16'h0100), .CPL_HDR_CR(CPL_H), .CPL_DATA_CR(CPL_D)) dutA (
    .clk, .rst_n, .axi_base(64'h0),
    .s_axi_awid(s_axi0.awid), .s_axi_awaddr(s_axi0.awaddr), .s_axi_awlen(s_axi0.awlen), .s_axi_awsize(s_axi0.awsize),
    .s_axi_awburst(s_axi0.awburst), .s_axi_awvalid(s_axi0.awvalid), .s_axi_awready(s_axi0.awready),
    .s_axi_wdata(s_axi0.wdata), .s_axi_wstrb(s_axi0.wstrb), .s_axi_wlast(s_axi0.wlast), .s_axi_wvalid(s_axi0.wvalid), .s_axi_wready(s_axi0.wready),
    .s_axi_bid(s_axi0.bid), .s_axi_bresp(s_axi0.bresp), .s_axi_bvalid(s_axi0.bvalid), .s_axi_bready(s_axi0.bready),
    .s_axi_arid(s_axi0.arid), .s_axi_araddr(s_axi0.araddr), .s_axi_arlen(s_axi0.arlen), .s_axi_arsize(s_axi0.arsize),
    .s_axi_arburst(s_axi0.arburst), .s_axi_arvalid(s_axi0.arvalid), .s_axi_arready(s_axi0.arready),
    .s_axi_rid(s_axi0.rid), .s_axi_rdata(s_axi0.rdata), .s_axi_rresp(s_axi0.rresp), .s_axi_rlast(s_axi0.rlast),
    .s_axi_rvalid(s_axi0.rvalid), .s_axi_rready(s_axi0.rready),
    .m_axi_awid(m_axi0.awid), .m_axi_awaddr(m_axi0.awaddr), .m_axi_awlen(m_axi0.awlen), .m_axi_awsize(m_axi0.awsize),
    .m_axi_awburst(m_axi0.awburst), .m_axi_awvalid(m_axi0.awvalid), .m_axi_awready(m_axi0.awready),
    .m_axi_wdata(m_axi0.wdata), .m_axi_wstrb(m_axi0.wstrb), .m_axi_wlast(m_axi0.wlast), .m_axi_wvalid(m_axi0.wvalid), .m_axi_wready(m_axi0.wready),
    .m_axi_bid(m_axi0.bid), .m_axi_bresp(m_axi0.bresp), .m_axi_bvalid(m_axi0.bvalid), .m_axi_bready(m_axi0.bready),
    .m_axi_arid(m_axi0.arid), .m_axi_araddr(m_axi0.araddr), .m_axi_arlen(m_axi0.arlen), .m_axi_arsize(m_axi0.arsize),
    .m_axi_arburst(m_axi0.arburst), .m_axi_arvalid(m_axi0.arvalid), .m_axi_arready(m_axi0.arready),
    .m_axi_rid(m_axi0.rid), .m_axi_rdata(m_axi0.rdata), .m_axi_rresp(m_axi0.rresp), .m_axi_rlast(m_axi0.rlast),
    .m_axi_rvalid(m_axi0.rvalid), .m_axi_rready(m_axi0.rready),
    .lnk_tx_valid(tx_v[0]), .lnk_tx_data(tx_dat[0]), .lnk_tx_keep(tx_k[0]), .lnk_tx_sop(tx_s[0]), .lnk_tx_eop(tx_e[0]), .lnk_tx_dllp(tx_d[0]),
    .lnk_rx_valid(rx_v[0]), .lnk_rx_data(rx_dat[0]), .lnk_rx_keep(rx_k[0]), .lnk_rx_sop(rx_s[0]), .lnk_rx_eop(rx_e[0]), .lnk_rx_dllp(rx_d[0]),
    .phy_link_up(phy[0]),
    .csr_req(csr0.req), .csr_we(csr0.we), .csr_addr(csr0.addr), .csr_wdata(csr0.wdata), .csr_ack(csr0.ack), .csr_rdata(csr0.rdata),
    .dl_up(csr0.dl_up), .fatal_err(csr0.fatal_err));

  // ---- node B ----
  pcie_ep_top #(.DEVICE_ID(16'h0200), .CPL_HDR_CR(CPL_H), .CPL_DATA_CR(CPL_D)) dutB (
    .clk, .rst_n, .axi_base(64'h0),
    .s_axi_awid(s_axi1.awid), .s_axi_awaddr(s_axi1.awaddr), .s_axi_awlen(s_axi1.awlen), .s_axi_awsize(s_axi1.awsize),
    .s_axi_awburst(s_axi1.awburst), .s_axi_awvalid(s_axi1.awvalid), .s_axi_awready(s_axi1.awready),
    .s_axi_wdata(s_axi1.wdata), .s_axi_wstrb(s_axi1.wstrb), .s_axi_wlast(s_axi1.wlast), .s_axi_wvalid(s_axi1.wvalid), .s_axi_wready(s_axi1.wready),
    .s_axi_bid(s_axi1.bid), .s_axi_bresp(s_axi1.bresp), .s_axi_bvalid(s_axi1.bvalid), .s_axi_bready(s_axi1.bready),
    .s_axi_arid(s_axi1.arid), .s_axi_araddr(s_axi1.araddr), .s_axi_arlen(s_axi1.arlen), .s_axi_arsize(s_axi1.arsize),
    .s_axi_arburst(s_axi1.arburst), .s_axi_arvalid(s_axi1.arvalid), .s_axi_arready(s_axi1.arready),
    .s_axi_rid(s_axi1.rid), .s_axi_rdata(s_axi1.rdata), .s_axi_rresp(s_axi1.rresp), .s_axi_rlast(s_axi1.rlast),
    .s_axi_rvalid(s_axi1.rvalid), .s_axi_rready(s_axi1.rready),
    .m_axi_awid(m_axi1.awid), .m_axi_awaddr(m_axi1.awaddr), .m_axi_awlen(m_axi1.awlen), .m_axi_awsize(m_axi1.awsize),
    .m_axi_awburst(m_axi1.awburst), .m_axi_awvalid(m_axi1.awvalid), .m_axi_awready(m_axi1.awready),
    .m_axi_wdata(m_axi1.wdata), .m_axi_wstrb(m_axi1.wstrb), .m_axi_wlast(m_axi1.wlast), .m_axi_wvalid(m_axi1.wvalid), .m_axi_wready(m_axi1.wready),
    .m_axi_bid(m_axi1.bid), .m_axi_bresp(m_axi1.bresp), .m_axi_bvalid(m_axi1.bvalid), .m_axi_bready(m_axi1.bready),
    .m_axi_arid(m_axi1.arid), .m_axi_araddr(m_axi1.araddr), .m_axi_arlen(m_axi1.arlen), .m_axi_arsize(m_axi1.arsize),
    .m_axi_arburst(m_axi1.arburst), .m_axi_arvalid(m_axi1.arvalid), .m_axi_arready(m_axi1.arready),
    .m_axi_rid(m_axi1.rid), .m_axi_rdata(m_axi1.rdata), .m_axi_rresp(m_axi1.rresp), .m_axi_rlast(m_axi1.rlast),
    .m_axi_rvalid(m_axi1.rvalid), .m_axi_rready(m_axi1.rready),
    .lnk_tx_valid(tx_v[1]), .lnk_tx_data(tx_dat[1]), .lnk_tx_keep(tx_k[1]), .lnk_tx_sop(tx_s[1]), .lnk_tx_eop(tx_e[1]), .lnk_tx_dllp(tx_d[1]),
    .lnk_rx_valid(rx_v[1]), .lnk_rx_data(rx_dat[1]), .lnk_rx_keep(rx_k[1]), .lnk_rx_sop(rx_s[1]), .lnk_rx_eop(rx_e[1]), .lnk_rx_dllp(rx_d[1]),
    .phy_link_up(phy[1]),
    .csr_req(csr1.req), .csr_we(csr1.we), .csr_addr(csr1.addr), .csr_wdata(csr1.wdata), .csr_ack(csr1.ack), .csr_rdata(csr1.rdata),
    .dl_up(csr1.dl_up), .fatal_err(csr1.fatal_err));

  link_model #(.LINK_LATENCY(LAT), .LINK_UP_DELAY(16)) LM (.clk, .rst_n,
    .a_tx_valid(tx_v[0]), .a_tx_data(tx_dat[0]), .a_tx_keep(tx_k[0]), .a_tx_sop(tx_s[0]), .a_tx_eop(tx_e[0]), .a_tx_dllp(tx_d[0]),
    .b_rx_valid(rx_v[1]), .b_rx_data(rx_dat[1]), .b_rx_keep(rx_k[1]), .b_rx_sop(rx_s[1]), .b_rx_eop(rx_e[1]), .b_rx_dllp(rx_d[1]),
    .b_tx_valid(tx_v[1]), .b_tx_data(tx_dat[1]), .b_tx_keep(tx_k[1]), .b_tx_sop(tx_s[1]), .b_tx_eop(tx_e[1]), .b_tx_dllp(tx_d[1]),
    .a_rx_valid(rx_v[0]), .a_rx_data(rx_dat[0]), .a_rx_keep(rx_k[0]), .a_rx_sop(rx_s[0]), .a_rx_eop(rx_e[0]), .a_rx_dllp(rx_d[0]),
    .phy_link_up_a(phy[0]), .phy_link_up_b(phy[1]), .fi_a2b(fi_a2b.fi), .fi_b2a(fi_b2a.fi),
    .evt_valid_a2b(fi_a2b.evt_valid), .evt_action_a2b(fi_a2b.evt_action), .evt_seq_a2b(fi_a2b.evt_seq), .evt_dllp_a2b(fi_a2b.evt_dllp), .evt_idx_a2b(fi_a2b.evt_idx),
    .evt_valid_b2a(fi_b2a.evt_valid), .evt_action_b2a(fi_b2a.evt_action), .evt_seq_b2a(fi_b2a.evt_seq), .evt_dllp_b2a(fi_b2a.evt_dllp), .evt_idx_b2a(fi_b2a.evt_idx));

  // ---- link taps (passive) ----
  assign tap_a2b_in.valid  = tx_v[0]; assign tap_a2b_in.data  = tx_dat[0]; assign tap_a2b_in.keep  = tx_k[0];
  assign tap_a2b_in.sop    = tx_s[0]; assign tap_a2b_in.eop   = tx_e[0];   assign tap_a2b_in.dllp  = tx_d[0];
  assign tap_a2b_out.valid = rx_v[1]; assign tap_a2b_out.data = rx_dat[1]; assign tap_a2b_out.keep = rx_k[1];
  assign tap_a2b_out.sop   = rx_s[1]; assign tap_a2b_out.eop  = rx_e[1];   assign tap_a2b_out.dllp = rx_d[1];
  assign tap_b2a_in.valid  = tx_v[1]; assign tap_b2a_in.data  = tx_dat[1]; assign tap_b2a_in.keep  = tx_k[1];
  assign tap_b2a_in.sop    = tx_s[1]; assign tap_b2a_in.eop   = tx_e[1];   assign tap_b2a_in.dllp  = tx_d[1];
  assign tap_b2a_out.valid = rx_v[0]; assign tap_b2a_out.data = rx_dat[0]; assign tap_b2a_out.keep = rx_k[0];
  assign tap_b2a_out.sop   = rx_s[0]; assign tap_b2a_out.eop  = rx_e[0];   assign tap_b2a_out.dllp = rx_d[0];

  initial begin
    rst_n = 0;
    repeat (10) @(posedge clk);
    rst_n <= 1;
  end

  initial begin
    uvm_config_db#(virtual axi_if)::set(null, "*", "s_axi_vif0", s_axi0);
    uvm_config_db#(virtual axi_if)::set(null, "*", "s_axi_vif1", s_axi1);
    uvm_config_db#(virtual axi_if)::set(null, "*", "m_axi_vif0", m_axi0);
    uvm_config_db#(virtual axi_if)::set(null, "*", "m_axi_vif1", m_axi1);
    uvm_config_db#(virtual csr_if)::set(null, "*", "csr_vif0", csr0);
    uvm_config_db#(virtual csr_if)::set(null, "*", "csr_vif1", csr1);
    uvm_config_db#(virtual lnk_if)::set(null, "*", "tap_a2b_in",  tap_a2b_in);
    uvm_config_db#(virtual lnk_if)::set(null, "*", "tap_a2b_out", tap_a2b_out);
    uvm_config_db#(virtual lnk_if)::set(null, "*", "tap_b2a_in",  tap_b2a_in);
    uvm_config_db#(virtual lnk_if)::set(null, "*", "tap_b2a_out", tap_b2a_out);
    uvm_config_db#(virtual fi_if)::set(null, "*", "fi_vif0", fi_a2b);
    uvm_config_db#(virtual fi_if)::set(null, "*", "fi_vif1", fi_b2a);
    run_test();
  end

  // ---- switching activity for PrimeTime PX (spec 14.1): VCD of node A over the test's measurement window
  //   +PWR_VCD=<file> : the perf test triggers the global uvm_events "saif_start" / "saif_stop".
  //   (RTL SAIF via $toggle_* produces an empty file with this VCS install, so PrimeTime reads the VCD directly)
  initial begin
    string f;
    if ($value$plusargs("PWR_VCD=%s", f)) begin
      uvm_event es = uvm_event_pool::get_global("saif_start"), ep = uvm_event_pool::get_global("saif_stop");
      es.wait_trigger();
      $dumpfile(f);
      $dumpvars(0, tb_uvm_top.dutA);
      ep.wait_trigger();
      $dumpoff;
      $dumpflush;
      $display("power VCD written: %s", f);
    end
  end
  // ---- waveforms for DVE: +VPD=<file> [+WAVE_START=cycle] records both nodes (ports, DLL TX/RX, TL TX/RX)
  initial begin
    string f; int t0;
    if ($value$plusargs("VPD=%s", f)) begin
      if (!$value$plusargs("WAVE_START=%d", t0)) t0 = 0;
      #(t0 * 4ns);
      $vcdplusfile(f);
      $vcdpluson(1, tb_uvm_top);
      $vcdpluson(1, tb_uvm_top.dutA); $vcdpluson(1, tb_uvm_top.dutB);
      $vcdpluson(1, tb_uvm_top.dutA.u_dtx); $vcdpluson(1, tb_uvm_top.dutB.u_drx);
      $vcdpluson(1, tb_uvm_top.dutA.u_drx); $vcdpluson(1, tb_uvm_top.dutB.u_dtx);
    end
  end
  // ---- waveform for the README snapshot: +WAVE=<vcd> dumps the link and AXI ports of node A
  initial begin
    string f; int t0;
    if ($value$plusargs("WAVE=%s", f)) begin
      if (!$value$plusargs("WAVE_START=%d", t0)) t0 = 0;
      #(t0 * 4ns);
      $dumpfile(f);
      $dumpvars(1, tb_uvm_top.dutA);
      $dumpvars(1, tb_uvm_top.dutA.u_dtx);
      $dumpvars(1, tb_uvm_top.dutB.u_drx);
    end
  end

  // watchdog
  initial begin
    #(200ms);
    `uvm_fatal("TOP", "global watchdog expired")
  end
endmodule
