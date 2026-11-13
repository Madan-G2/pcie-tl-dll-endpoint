// debug probes (compile with +define+DBG_PROBE, plusarg +PROBE to print)
module dbg_probe;
  bit en; initial en = $test$plusargs("PROBE");

  always @(posedge tb_sys.clk) if (en && tb_sys.rst_n) begin
    if (tb_sys.N[0].dut.cfg_acc_valid)
      $display("[%0t] node0 cfg access we=%b dw=%h wdata=%h be=%b rdata=%h", $time, tb_sys.N[0].dut.cfg_acc_we, tb_sys.N[0].dut.cfg_acc_dw, tb_sys.N[0].dut.cfg_acc_wdata, tb_sys.N[0].dut.cfg_acc_be, tb_sys.N[0].dut.cfg_acc_rdata);
    if (tb_sys.N[0].dut.err_set != 0) $display("[%0t] node0 err_set=%h", $time, tb_sys.N[0].dut.err_set);
    if (tb_sys.N[0].dut.cp_valid && tb_sys.N[0].dut.cp_sop)
      $display("[%0t] node0 cp_sop data=%h", $time, tb_sys.N[0].dut.cp_data);
  end

  always @(posedge tb_sys.clk) if (en && tb_sys.rst_n) begin
    if (tb_sys.N[1].dut.cfg_acc_valid)
      $display("[%0t] node1 cfg access we=%b dw=%h wdata=%h be=%b rdata=%h", $time, tb_sys.N[1].dut.cfg_acc_we, tb_sys.N[1].dut.cfg_acc_dw, tb_sys.N[1].dut.cfg_acc_wdata, tb_sys.N[1].dut.cfg_acc_be, tb_sys.N[1].dut.cfg_acc_rdata);
    if (tb_sys.N[1].dut.err_set != 0) $display("[%0t] node1 err_set=%h", $time, tb_sys.N[1].dut.err_set);
    if (tb_sys.N[1].dut.cp_valid && tb_sys.N[1].dut.cp_sop)
      $display("[%0t] node1 cp_sop data=%h", $time, tb_sys.N[1].dut.cp_data);
  end

  always @(posedge tb_sys.clk) if (en && tb_sys.rst_n) begin
    if (tb_sys.N[1].dut.u_slv.u_tags.al_go)
      $display("[%0t] n1 ALLOC tag=%0d kind=%0d burst=%0d sub=%0d addr=%h len=%0d", $time, tb_sys.N[1].dut.u_slv.u_tags.al_tag, tb_sys.N[1].dut.u_slv.u_tags.al_kind, tb_sys.N[1].dut.u_slv.u_tags.al_burst, tb_sys.N[1].dut.u_slv.u_tags.al_sub, tb_sys.N[1].dut.u_slv.u_tags.al_addr, tb_sys.N[1].dut.u_slv.u_tags.al_len);
    if (tb_sys.N[1].dut.u_slv.u_tags.fr_en)  $display("[%0t] n1 FREE tag=%0d", $time, tb_sys.N[1].dut.u_slv.u_tags.fr_tag);
    if (tb_sys.N[1].dut.u_slv.u_tags.m_bad)
      $display("[%0t] n1 BAD cpl tag=%0d v=%b d=%b reqid=%h status=%0d bc=%0d lower=%h len=%0d hasdata=%b rcvd=%0d tklen=%0d", $time, tb_sys.N[1].dut.u_slv.u_tags.c_tag, tb_sys.N[1].dut.u_slv.u_tags.v[tb_sys.N[1].dut.u_slv.u_tags.ti], tb_sys.N[1].dut.u_slv.u_tags.d[tb_sys.N[1].dut.u_slv.u_tags.ti], tb_sys.N[1].dut.u_slv.u_tags.c_reqid, tb_sys.N[1].dut.u_slv.u_tags.c_status, tb_sys.N[1].dut.u_slv.u_tags.c_bc, tb_sys.N[1].dut.u_slv.u_tags.c_lower, tb_sys.N[1].dut.u_slv.u_tags.c_len, tb_sys.N[1].dut.u_slv.u_tags.c_hasdata, tb_sys.N[1].dut.u_slv.u_tags.tk_rcvd[tb_sys.N[1].dut.u_slv.u_tags.ti], tb_sys.N[1].dut.u_slv.u_tags.tk_len[tb_sys.N[1].dut.u_slv.u_tags.ti]);
  end
  // node0 completer view of inbound NP requests
  always @(posedge tb_sys.clk) if (en && tb_sys.rst_n) begin
    if (tb_sys.N[0].dut.dl_rx_valid && tb_sys.N[0].dut.dl_rx_sop && tb_sys.N[0].dut.dl_rx_data[7:0] == 8'h00)
      $display("[%0t] n0 dl_rx MRd data=%h", $time, tb_sys.N[0].dut.dl_rx_data);
    if (tb_sys.N[0].dut.nd_valid && tb_sys.N[0].dut.nd_ready)
      $display("[%0t] n0 nd handshake off=%h len=%0d tag=%0d reqid=%h", $time, tb_sys.N[0].dut.nd_off, tb_sys.N[0].dut.nd_len, tb_sys.N[0].dut.nd_tag, tb_sys.N[0].dut.nd_reqid);
  end
  always @(posedge tb_sys.clk) if (en && tb_sys.rst_n) begin
    if (tb_sys.N[0].dut.u_mr.np_rel)
      $display("[%0t] n0 AR slot=%0d off=%h len=%0d tag=%0d arlen=%0d", $time, tb_sys.N[0].dut.u_mr.free_slot, tb_sys.N[0].dut.u_mr.pq_off, tb_sys.N[0].dut.u_mr.pq_len, tb_sys.N[0].dut.u_mr.pq_tag, tb_sys.N[0].dut.u_mr.m_arlen);
    if (tb_sys.N[0].dut.m_axi_rvalid && tb_sys.N[0].dut.m_axi_rlast)
      $display("[%0t] n0 R last rid=%0d", $time, tb_sys.N[0].dut.m_axi_rid);
    if (tb_sys.N[0].dut.u_mr.g_st == 3'd1)
      $display("[%0t] n0 G_INIT slot=%0d a=%h len=%0d tag=%0d", $time, tb_sys.N[0].dut.u_mr.g_slot, tb_sys.N[0].dut.u_mr.s_a[tb_sys.N[0].dut.u_mr.g_slot], tb_sys.N[0].dut.u_mr.s_len[tb_sys.N[0].dut.u_mr.g_slot], tb_sys.N[0].dut.u_mr.s_tag[tb_sys.N[0].dut.u_mr.g_slot]);
    if (tb_sys.N[0].dut.cpl0_valid && tb_sys.N[0].dut.cpl0_ready && tb_sys.N[0].dut.cpl0_sop)
      $display("[%0t] n0 cpl0 push sop data=%h", $time, tb_sys.N[0].dut.cpl0_data);
  end
endmodule
