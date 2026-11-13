// ============================================================================
// CSR helper (register model light), virtual sequencer, environment (spec 10)
// ============================================================================
class csr_op_seq extends uvm_sequence #(csr_item);
  `uvm_object_utils(csr_op_seq)
  bit we; bit [11:0] addr; bit [31:0] wdata, rdata;
  function new(string name = "csr_op_seq"); super.new(name); endfunction
  task body();
    csr_item it = csr_item::type_id::create("it");
    start_item(it); it.we = we; it.addr = addr; it.wdata = wdata; finish_item(it);
    rdata = it.rdata;
  endtask
endclass

class axi_one_seq extends uvm_sequence #(axi_txn);
  `uvm_object_utils(axi_one_seq)
  axi_txn t;
  function new(string name = "axi_one_seq"); super.new(name); endfunction
  task body();
    start_item(t); finish_item(t);
  endtask
endclass

class fi_one_seq extends uvm_sequence #(fi_cmd);
  `uvm_object_utils(fi_one_seq)
  link_pkg::link_fi_t cmd;
  function new(string name = "fi_one_seq"); super.new(name); endfunction
  task body();
    fi_cmd c = fi_cmd::type_id::create("c");
    start_item(c); c.cmd = cmd; finish_item(c);
  endtask
endclass

// lightweight register access layer for sections 9.2 and 9.4 (read, write, snapshot, presets)
class csr_api extends uvm_object;
  `uvm_object_utils(csr_api)
  csr_sequencer sqr;
  pcie_env_cfg  cfg;
  int node;
  function new(string name = "csr_api"); super.new(name); endfunction
  task write(bit [11:0] a, bit [31:0] d);
    csr_op_seq s = csr_op_seq::type_id::create("w");
    s.we = 1; s.addr = a; s.wdata = d; s.start(sqr);
  endtask
  task read(bit [11:0] a, output bit [31:0] d);
    csr_op_seq s = csr_op_seq::type_id::create("r");
    s.we = 0; s.addr = a; s.start(sqr); d = s.rdata;
  endtask
  task snapshot();
    write(CSR_CTRL, 32'h0000_0009);            // LINK_ENABLE | SNAPSHOT
  endtask
  task set_baseline();
    write(CSR_OPT_CTRL, 32'h0);
  endtask
  task set_optimized();
    write(CSR_OPT_CTRL, 32'(cfg.opt_mask));
  endtask
  task status(output bit [31:0] s);
    read(CSR_STATUS, s);
  endtask
  task read_counters(output bit [31:0] c[bit [11:0]]);
    bit [31:0] d;
    for (bit [11:0] a = 12'h040; a <= 12'h0A4; a += 4) begin read(a, d); c[a] = d; end
  endtask
endclass

class pcie_vseqr extends uvm_sequencer;
  `uvm_component_utils(pcie_vseqr)
  axi_sequencer m_sqr [2];
  csr_sequencer c_sqr [2];
  fi_sequencer  f_sqr [2];
  csr_api       csr [2];
  virtual csr_if csr_vif [2];
  pcie_env_cfg  cfg;
  function new(string n, uvm_component p); super.new(n, p); endfunction
endclass

class pcie_env_c extends uvm_env;
  `uvm_component_utils(pcie_env_c)
  pcie_env_cfg cfg;
  axi_master_agent m_agt [2];
  axi_slave_agent  s_agt [2];
  csr_agent        c_agt [2];
  link_monitor     lmon  [4];           // 0 A->B in, 1 A->B out, 2 B->A in, 3 B->A out
  fi_agent         f_agt [2];
  status_monitor   stat;
  axi_data_sb      axi_sb;
  dll_proto_sb     dll_sb [2];
  tl_proto_sb      tl_sb  [2];
  fc_sb            fcs    [2];
  order_sb         ord    [2];
  pcie_csr_block   regmodel [2];          // RAL (spec 10.1 csr_agent + RAL)
  csr_reg_adapter  reg_adp;
  perf_mon         perf;
  pcie_cov         cov;
  pcie_vseqr       vsqr;
  virtual axi_if   s_vif [2], m_vif [2];
  virtual csr_if   c_vif [2];
  virtual lnk_if   t_vif [4];
  virtual fi_if    f_vif [2];

  function new(string n, uvm_component p); super.new(n, p); endfunction

  function void build_phase(uvm_phase phase);
    string tn[4] = '{"tap_a2b_in", "tap_a2b_out", "tap_b2a_in", "tap_b2a_out"};
    if (!uvm_config_db#(pcie_env_cfg)::get(this, "", "cfg", cfg)) `uvm_fatal("NOCFG", "pcie_env: cfg not set")
    uvm_config_db#(pcie_env_cfg)::set(this, "*", "cfg", cfg);
    for (int n = 0; n < 2; n++) begin
      if (!uvm_config_db#(virtual axi_if)::get(this, "", $sformatf("s_axi_vif%0d", n), s_vif[n])) `uvm_fatal("NOVIF", "s_axi vif")
      if (!uvm_config_db#(virtual axi_if)::get(this, "", $sformatf("m_axi_vif%0d", n), m_vif[n])) `uvm_fatal("NOVIF", "m_axi vif")
      if (!uvm_config_db#(virtual csr_if)::get(this, "", $sformatf("csr_vif%0d", n), c_vif[n])) `uvm_fatal("NOVIF", "csr vif")
      if (!uvm_config_db#(virtual fi_if)::get(this, "", $sformatf("fi_vif%0d", n), f_vif[n])) `uvm_fatal("NOVIF", "fi vif")
      uvm_config_db#(virtual axi_if)::set(this, $sformatf("m_agt%0d*", n), "vif", s_vif[n]);
      uvm_config_db#(virtual axi_if)::set(this, $sformatf("s_agt%0d*", n), "vif", m_vif[n]);
      uvm_config_db#(virtual csr_if)::set(this, $sformatf("c_agt%0d*", n), "vif", c_vif[n]);
      uvm_config_db#(virtual fi_if)::set(this, $sformatf("f_agt%0d*", n), "vif", f_vif[n]);
      uvm_config_db#(int)::set(this, $sformatf("f_agt%0d*", n), "dir", n);
      uvm_config_db#(int)::set(this, $sformatf("s_agt%0d*", n), "node", n);
      m_agt[n] = axi_master_agent::type_id::create($sformatf("m_agt%0d", n), this);
      s_agt[n] = axi_slave_agent ::type_id::create($sformatf("s_agt%0d", n), this);
      c_agt[n] = csr_agent       ::type_id::create($sformatf("c_agt%0d", n), this);
      f_agt[n] = fi_agent        ::type_id::create($sformatf("f_agt%0d", n), this);
      dll_sb[n] = dll_proto_sb::type_id::create($sformatf("dll_sb%0d", n), this);
      tl_sb[n]  = tl_proto_sb ::type_id::create($sformatf("tl_sb%0d", n), this);
      fcs[n]    = fc_sb       ::type_id::create($sformatf("fc_sb%0d", n), this);
      uvm_config_db#(int)::set(this, $sformatf("dll_sb%0d", n), "dir", n);
      uvm_config_db#(int)::set(this, $sformatf("tl_sb%0d", n), "dir", n);
      uvm_config_db#(int)::set(this, $sformatf("fc_sb%0d", n), "dir", n);
      ord[n]    = order_sb    ::type_id::create($sformatf("ord%0d", n), this);
      uvm_config_db#(int)::set(this, $sformatf("ord%0d", n), "node", n);
    end
    for (int i = 0; i < 4; i++) begin
      if (!uvm_config_db#(virtual lnk_if)::get(this, "", tn[i], t_vif[i])) `uvm_fatal("NOVIF", tn[i])
      uvm_config_db#(virtual lnk_if)::set(this, $sformatf("lmon%0d", i), "vif", t_vif[i]);
      uvm_config_db#(int)::set(this, $sformatf("lmon%0d", i), "dir", i / 2);
      uvm_config_db#(int)::set(this, $sformatf("lmon%0d", i), "is_in", (i % 2) == 0);
      uvm_config_db#(virtual csr_if)::set(this, $sformatf("lmon%0d", i), "dl_vif", c_vif[i / 2]);
      lmon[i] = link_monitor::type_id::create($sformatf("lmon%0d", i), this);
    end
    uvm_config_db#(virtual csr_if)::set(this, "stat", "vif0", c_vif[0]);
    uvm_config_db#(virtual csr_if)::set(this, "stat", "vif1", c_vif[1]);
    stat   = status_monitor::type_id::create("stat", this);
    axi_sb = axi_data_sb::type_id::create("axi_sb", this);
    perf   = perf_mon::type_id::create("perf", this);
    cov    = pcie_cov::type_id::create("cov", this);
    vsqr   = pcie_vseqr::type_id::create("vsqr", this);
    reg_adp = csr_reg_adapter::type_id::create("reg_adp");
    for (int n = 0; n < 2; n++) begin
      regmodel[n] = pcie_csr_block::type_id::create($sformatf("regmodel%0d", n));
      regmodel[n].max_tags = cfg.max_tags;
      regmodel[n].configure(null, "");
      regmodel[n].build();
    end
  endfunction

  function void connect_phase(uvm_phase phase);
    for (int d = 0; d < 2; d++) begin
      int r = 1 - d;
      lmon[2*d].ap.connect(dll_sb[d].in_imp);       lmon[2*d+1].ap.connect(dll_sb[d].out_imp);
      lmon[2*r].ap.connect(dll_sb[d].rin_imp);      lmon[2*r+1].ap.connect(dll_sb[d].rout_imp);
      stat.ap.connect(dll_sb[d].dl_imp);
      lmon[2*d].ap.connect(tl_sb[d].in_imp);        lmon[2*r].ap.connect(tl_sb[d].rin_imp);
      stat.ap.connect(tl_sb[d].dl_imp);
      lmon[2*d].ap.connect(fcs[d].in_imp);          lmon[2*r+1].ap.connect(fcs[d].rout_imp);
      stat.ap.connect(fcs[d].dl_imp);
      m_agt[d].mon.ap_wr.connect(axi_sb.s_wr[d].analysis_export);
      m_agt[d].mon.ap_rd.connect(axi_sb.s_rd[d].analysis_export);
      s_agt[d].mon.ap_wr.connect(axi_sb.m_wr[d].analysis_export);
      s_agt[d].mon.ap_rd.connect(axi_sb.m_rd[d].analysis_export);
      m_agt[d].mon.ap_wr.connect(cov.axi_imp);
      m_agt[d].mon.ap_rd.connect(cov.axi_imp);
      f_agt[d].mon.ap.connect(cov.fi_imp);
      lmon[2*d].ap.connect(cov.frame_imp);
      m_agt[d].mon.ap_wacc.connect(ord[d].wacc_imp);   m_agt[d].mon.ap_ar.connect(ord[d].ar_imp);
      m_agt[d].mon.ap_wr.connect(ord[d].b_imp);        m_agt[d].mon.ap_rd.connect(ord[d].r_imp);
      lmon[2*d].ap.connect(ord[d].tap_imp);            stat.ap.connect(ord[d].dl_imp);
    end
    stat.ap.connect(perf.dl_imp);
    lmon[0].ap.connect(perf.in0_imp);  lmon[1].ap.connect(perf.out0_imp);
    lmon[2].ap.connect(perf.in1_imp);  lmon[3].ap.connect(perf.out1_imp);
    for (int n = 0; n < 2; n++) begin
      vsqr.m_sqr[n] = m_agt[n].sqr; vsqr.c_sqr[n] = c_agt[n].sqr; vsqr.f_sqr[n] = f_agt[n].sqr;
      vsqr.csr[n] = csr_api::type_id::create($sformatf("csr%0d", n));
      vsqr.csr[n].sqr = c_agt[n].sqr; vsqr.csr[n].cfg = cfg; vsqr.csr[n].node = n;
      vsqr.csr_vif[n] = c_vif[n];
    end
    vsqr.cfg = cfg;
    for (int n = 0; n < 2; n++) begin
      regmodel[n].map.set_sequencer(c_agt[n].sqr, reg_adp);
      regmodel[n].map.set_auto_predict(1);
    end
  endfunction
endclass
