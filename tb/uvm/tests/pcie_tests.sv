// ============================================================================
// Tests (spec 13.1 subset).  Plusargs: +OPT=0/1  +PPM=n  +NOPS=n  +MEMPOL=0/1/2  +MEMDLY=n  +OOO=0/1  +BP=pct
// ============================================================================
class pcie_base_test extends uvm_test;
  `uvm_component_utils(pcie_base_test)
  pcie_env_c env;
  pcie_env_cfg cfg;
  function new(string n, uvm_component p); super.new(n, p); endfunction

  virtual function void configure();      // overridden / extended by tests
    int v;
    cfg.test_name = get_type_name();
    if ($value$plusargs("OPT=%d", v))    begin cfg.opt_mode[0] = v[0]; cfg.opt_mode[1] = v[0]; end
    if ($value$plusargs("OPTMASK=%h", v)) begin cfg.opt_mask = v[5:0]; cfg.opt_mode[0] = 1; cfg.opt_mode[1] = 1; end
    if ($value$plusargs("NOPS=%d", v))   cfg.n_transactions = v;
    if ($value$plusargs("MEMPOL=%d", v)) begin cfg.mem_policy[0] = v; cfg.mem_policy[1] = v; end
    if ($value$plusargs("MEMDLY=%d", v)) begin cfg.mem_delay[0] = v; cfg.mem_delay[1] = v; end
    if ($value$plusargs("OOO=%d", v))    begin cfg.mem_ooo[0] = v[0]; cfg.mem_ooo[1] = v[0]; end
    if ($value$plusargs("BP=%d", v))     cfg.axi_bp_pct = v;
    if ($value$plusargs("MPS=%d", v))    cfg.mps_code = v;
    if ($value$plusargs("MRRS=%d", v))   cfg.mrrs_code = v;
  endfunction

  function void build_phase(uvm_phase phase);
    cfg = pcie_env_cfg::type_id::create("cfg");
    configure();
    uvm_config_db#(pcie_env_cfg)::set(this, "env", "cfg", cfg);
    env = pcie_env_c::type_id::create("env", this);
  endfunction

  // body of the main phase : override
  virtual task main_body(uvm_phase phase);
    directed_vseq d = directed_vseq::type_id::create("d");
    d.start(env.vsqr);
  endtask

  task run_phase(uvm_phase phase);
    bringup_vseq bu = bringup_vseq::type_id::create("bu");
    drain_vseq   dr = drain_vseq::type_id::create("dr");
    endcheck_vseq ec = endcheck_vseq::type_id::create("ec");
    phase.raise_objection(this);
    bu.start(env.vsqr);
    main_body(phase);
    dr.start(env.vsqr);
    ec.env = env; ec.start(env.vsqr);
    phase.drop_objection(this);
  endtask

  function void report_phase(uvm_phase phase);
    uvm_report_server rs = uvm_report_server::get_server();
    int e = rs.get_severity_count(UVM_ERROR) + rs.get_severity_count(UVM_FATAL);
    $display("RESULT: %s (errors=%0d) test=%s", e == 0 ? "PASS" : "FAIL", e, get_type_name());
  endfunction
endclass

class pcie_smoke_test extends pcie_base_test;
  `uvm_component_utils(pcie_smoke_test)
  function new(string n, uvm_component p); super.new(n, p); endfunction
  function void configure();
    super.configure();
    // node A: AXI_UNSUP + CPL_ERR_RX (UR completion received); node B: UR_SENT
    cfg.err_allowed[0] = 16'h0C00;  cfg.err_allowed[1] = 16'h0040;
    cfg.err_required[0] = 16'h0C00; cfg.err_required[1] = 16'h0040;
  endfunction
endclass

class pcie_rand_test extends pcie_base_test;
  `uvm_component_utils(pcie_rand_test)
  function new(string n, uvm_component p); super.new(n, p); endfunction
  function void configure();
    super.configure();
    cfg.err_allowed[0] = 16'h0800; cfg.err_allowed[1] = 16'h0800;
  endfunction
  task main_body(uvm_phase phase);
    rand_traffic_vseq r = rand_traffic_vseq::type_id::create("r");
    r.nops = cfg.n_transactions;
    r.start(env.vsqr);
  endtask
endclass

class pcie_rand_opt_test extends pcie_rand_test;
  `uvm_component_utils(pcie_rand_opt_test)
  function new(string n, uvm_component p); super.new(n, p); endfunction
  function void configure();
    super.configure();
    cfg.opt_mode[0] = 1; cfg.opt_mode[1] = 1; cfg.mem_ooo[0] = 1; cfg.mem_ooo[1] = 1; cfg.mem_policy[0] = 2; cfg.mem_policy[1] = 2;
  endfunction
endclass

// random bit errors on the link (W3-style, section 8.1)
class pcie_fi_rate_test extends pcie_rand_test;
  `uvm_component_utils(pcie_fi_rate_test)
  int ppm = 20000;
  function new(string n, uvm_component p); super.new(n, p); endfunction
  function void configure();
    super.configure();
    void'($value$plusargs("PPM=%d", ppm));
    // LCRC_ERR_RX, SEQ_ERR_RX, DLLP_CRC_ERR, REPLAY_TIMEOUT, DLLP_UNEXPECTED, AXI_UNSUP
    cfg.err_allowed[0] = 16'h2817; cfg.err_allowed[1] = 16'h2817;
  endfunction
  task main_body(uvm_phase phase);
    pcie_base_vseq b = pcie_base_vseq::type_id::create("b");
    b.cfg = cfg;
    b.set_sequencer(env.vsqr);
    b.start(env.vsqr);
    b.apply_fi(0, b.mk_fi(1, 0, 0, ppm, 0, 0, 32'h1111 + cfg.seed));
    b.apply_fi(1, b.mk_fi(1, 0, 0, ppm, 0, 0, 32'h2222 + cfg.seed));
    super.main_body(phase);
    b.clear_fi(0); b.clear_fi(1);
  endtask
endclass

// one-shot faults of every kind on frames of each class (13.1 DL tests)
class pcie_fi_directed_test extends pcie_base_test;
  `uvm_component_utils(pcie_fi_directed_test)
  function new(string n, uvm_component p); super.new(n, p); endfunction
  function void configure();
    super.configure();
    cfg.err_allowed[0] = 16'h2817; cfg.err_allowed[1] = 16'h2817;
  endfunction
  task main_body(uvm_phase phase);
    pcie_base_vseq b = pcie_base_vseq::type_id::create("b");
    rand_traffic_vseq r;
    // {action, target} : corrupt TLP, drop TLP, duplicate TLP, seq offset, drop Ack, corrupt Ack, drop Nak path, drop UpdateFC, corrupt UpdateFC, delay TLP
    int act[] = '{1, 2, 3, 4, 2, 1, 1, 2, 1, 7};
    int tgt[] = '{1, 1, 1, 1, 3, 3, 4, 5, 5, 1};
    b.cfg = cfg; b.set_sequencer(env.vsqr);
    foreach (act[i]) begin
      r = rand_traffic_vseq::type_id::create($sformatf("r%0d", i));
      r.nops = 8;
      b.apply_fi(0, b.mk_fi(act[i], tgt[i], 2, 0, 0, 2, 100 + i));       // mode 2 : the next 2 candidate frames
      b.apply_fi(1, b.mk_fi(act[i], tgt[i], 2, 0, 0, 2, 200 + i));
      r.start(env.vsqr);
      b.clear_fi(0); b.clear_fi(1);
      b.wait_cycles(200);
    end
  endtask
endclass

// TL-17 / UR paths : headers modified after the DLL (REPLACE_HDR), AX-1 and BAR violations
class pcie_err_test extends pcie_base_test;
  `uvm_component_utils(pcie_err_test)
  function new(string n, uvm_component p); super.new(n, p); endfunction
  function void configure();
    super.configure();
    cfg.err_allowed[0]  = 16'h4FEF; cfg.err_allowed[1]  = 16'h4FEF;
  endfunction
  task main_body(uvm_phase phase);
    pcie_base_vseq b = pcie_base_vseq::type_id::create("b");
    bit [1:0] r; bytes_t w, rb;
    bit [63:0] B = cfg.bar0_base[1];
    b.cfg = cfg; b.set_sequencer(env.vsqr);
    // writes outside BAR0 are dropped by the completer (UR_POSTED); reads get UR
    w = b.rnd_bytes(16); b.do_write(0, 64'h0000_0000_5000_0000, w, 4, r);
    b.do_read(0, 64'h0000_0000_5000_0000, 16, 4, rb, r);
    if (r != 2'b10) `uvm_error("ERR", "outside-BAR read must give SLVERR")
    // unsupported AXI forms (AX-1): wrong size, 4K crossing, sparse strobes
    begin
      axi_txn t = b.mk(0, B + 64'h10, 4, 0); b.send(0, t); wait (t.done);
      if (t.resp != 2'b10) `uvm_error("ERR", "size-0 read must give SLVERR")
      t = b.mk(1, B + 64'hFF8, 32, 4); b.send(0, t); wait (t.done);
      if (t.resp != 2'b10) `uvm_error("ERR", "4K crossing write must give SLVERR")
      begin
        bit sb[$] = '{1,1,1,1, 0,0,0,0, 1,1,1,1, 1,1,1,1};
        t = b.mk(1, B + 64'h100, 16, 4, b.rnd_bytes(16), sb); b.send(0, t); wait (t.done);
        if (t.resp != 2'b10) `uvm_error("ERR", "sparse-strobe write must give SLVERR")
      end
    end
    // REPLACE_HDR : flip TC on the next MWr TLP so the completer sees a malformed header (TL-17)
    begin
      link_pkg::link_fi_t c = b.mk_fi(5, 7, 1, 0, 1, 0, 7);
      c.match_fmt_type = 8'h40;
      c.hdr_xor = 128'h0000_0000_0000_0000_0000_0000_0000_1000;     // TC[2] : wire byte 1, bit 4
      b.apply_fi(0, c);
      w = b.rnd_bytes(32); b.do_write(0, B + 64'h200, w, 4, r);
      b.wait_cycles(300);
      b.clear_fi(0);
      env.axi_sb.forget_pending(1);                                  // the malformed write is dropped on purpose
      cfg.err_required[1] = 16'h0020;                                // MALFORMED must be flagged at node B
    end
  endtask
endclass

// credit / coalescer pressure
class pcie_small_wr_test extends pcie_base_test;
  `uvm_component_utils(pcie_small_wr_test)
  function new(string n, uvm_component p); super.new(n, p); endfunction
  task main_body(uvm_phase phase);
    small_write_vseq s = small_write_vseq::type_id::create("s");
    s.n = cfg.n_transactions * 5;
    s.start(env.vsqr);
  endtask
endclass

// link goes down in the middle of traffic and comes back
class pcie_linkdown_test extends pcie_base_test;
  `uvm_component_utils(pcie_linkdown_test)
  function new(string n, uvm_component p); super.new(n, p); endfunction
  function void configure();
    super.configure();
    cfg.err_allowed[0] = 16'hFFFF; cfg.err_allowed[1] = 16'hFFFF;
    cfg.check_counters = 0; cfg.sb_en_perf = 0;
  endfunction
  task main_body(uvm_phase phase);
    pcie_base_vseq b = pcie_base_vseq::type_id::create("b");
    bit [1:0] r; bytes_t w, rb;
    b.cfg = cfg; b.set_sequencer(env.vsqr);
    w = b.rnd_bytes(64); b.do_write(0, cfg.bar0_base[1] + 64'h100, w, 4, r);
    b.apply_fi(0, b.mk_fi(6, 0, 1, 0, 3, 0, 5));       // link down at the 3rd candidate frame
    b.wait_cycles(1500);
    b.clear_fi(0);
    wait (env.c_vif[0].dl_up && env.c_vif[1].dl_up);
    b.wait_cycles(100);
    `uvm_info("LD", "link recovered", UVM_LOW)
  endtask
endclass

// many reads in flight on both nodes, optionally out of order
class pcie_rd_pipe_test extends pcie_base_test;
  `uvm_component_utils(pcie_rd_pipe_test)
  function new(string n, uvm_component p); super.new(n, p); endfunction
  task main_body(uvm_phase phase);
    rd_pipe_vseq v = rd_pipe_vseq::type_id::create("v");
    v.start(env.vsqr);
  endtask
endclass

class pcie_rd_pipe_opt_test extends pcie_rd_pipe_test;
  `uvm_component_utils(pcie_rd_pipe_opt_test)
  function new(string n, uvm_component p); super.new(n, p); endfunction
  function void configure();
    super.configure();
    cfg.opt_mode[0] = 1; cfg.opt_mode[1] = 1; cfg.mem_ooo[0] = 1; cfg.mem_ooo[1] = 1; cfg.mem_policy[0] = 2; cfg.mem_policy[1] = 2;
  endfunction
endclass

// credit starvation (section 8.2): UpdateFC period of 2000 cycles so the 32 header credits run out
class pcie_credit_starve_test extends pcie_base_test;
  `uvm_component_utils(pcie_credit_starve_test)
  function new(string n, uvm_component p); super.new(n, p); endfunction
  function void configure();
    super.configure();
    cfg.extra_csr_addr.push_back(32'(CSR_FC_CFG)); cfg.extra_csr_data.push_back(32'h2008_07D0);
    cfg.ack_bound_cyc = 8000;
  endfunction
  task main_body(uvm_phase phase);
    small_write_vseq s = small_write_vseq::type_id::create("s");
    s.n = 150;
    s.start(env.vsqr);
  endtask
endclass

// ---------------------------------------------------------------------------------
// Mutation-closure tests (spec 13.3 step 5).  Each one was added because a mutant survived.
// ---------------------------------------------------------------------------------
// LCRC: flip every region of the 4 LCRC bytes, one TLP at a time (M01)
class pcie_lcrc_bits_test extends pcie_base_test;
  `uvm_component_utils(pcie_lcrc_bits_test)
  function new(string n, uvm_component p); super.new(n, p); endfunction
  function void configure();
    super.configure();
    cfg.err_allowed[0] = 16'h2817; cfg.err_allowed[1] = 16'h2817;
  endfunction
  task main_body(uvm_phase phase);
    pcie_base_vseq b = pcie_base_vseq::type_id::create("b");
    bit [1:0] r; bytes_t w, rb;
    bit [63:0] B = cfg.bar0_base[1];
    int bits[] = '{0, 7, 8, 15, 16, 23, 24, 30, 31};
    bit [31:0] syn[32] = '{32'hdb710641, 32'h6d930ac3, 32'hdb261586, 32'h6d3d2d4d, 32'hda7a5a9a, 32'h6f85b375, 32'hdf0b66ea, 32'h6567cb95, 32'hcacf972a, 32'h4eee2815, 32'h9ddc502a, 32'he0c9a615, 32'h1ae24a6b, 32'h35c494d6, 32'h6b8929ac, 32'hd7125358, 32'h7555a0f1, 32'heaab41e2, 32'h0e278585, 32'h1c4f0b0a, 32'h389e1614, 32'h713c2c28, 32'he2785850, 32'h1f81b6e1, 32'h3f036dc2, 32'h7e06db84, 32'hfc0db708, 32'h236a6851, 32'h46d4d0a2, 32'h8da9a144, 32'hc02244c9, 32'h5b358fd3};
    b.cfg = cfg; b.set_sequencer(env.vsqr);
    foreach (bits[i]) begin
      link_pkg::link_fi_t c = b.mk_fi(8, 1, 2, 0, 0, 1, 300 + i);        // action 8 : flip LCRC bits, TLPs only, next 1 frame
      c.hdr_xor = 128'd1 << bits[i];
      b.apply_fi(0, c);
      w = b.rnd_bytes(32); b.do_write(0, B + 64'h400 + i * 64'h40, w, 4, r);
      b.do_read(0, B + 64'h400 + i * 64'h40, 32, 4, rb, r); b.chk(rb, w, $sformatf("LCRC bit %0d", bits[i]));
      b.clear_fi(0);
      b.wait_cycles(100);
    end
    // LCRC errors whose CRC-32 residue differs from the good value in exactly one bit (a compare that skips that bit accepts them).
    // syn[k] = error pattern on L0..L3 that makes the receiver's residue differ from CRC32_RESIDUE by 1<<k (solved over GF(2)).
    foreach (syn[k]) begin
      link_pkg::link_fi_t c = b.mk_fi(8, 1, 2, 0, 0, 1, 350 + k);
      c.hdr_xor = 128'(syn[k]);
      b.apply_fi(0, c);
      w = b.rnd_bytes(32); b.do_write(0, B + 64'h1000 + k * 64'h40, w, 4, r);
      b.do_read(0, B + 64'h1000 + k * 64'h40, 32, 4, rb, r); b.chk(rb, w, $sformatf("LCRC residue bit %0d", k));
      b.clear_fi(0);
      b.wait_cycles(100);
    end
  endtask
endclass

// sequence numbers wrap: more than 4096 TLPs in one run (M03)
class pcie_seq_wrap_test extends pcie_base_test;
  `uvm_component_utils(pcie_seq_wrap_test)
  function new(string n, uvm_component p); super.new(n, p); endfunction
  task main_body(uvm_phase phase);
    small_write_vseq s = small_write_vseq::type_id::create("s");
    s.n = 4400;
    s.start(env.vsqr);
  endtask
endclass

// one frame corrupted on every try but the fourth: REPLAY_NUM reaches 3 without rolling over (M10)
class pcie_replay_num_test extends pcie_base_test;
  `uvm_component_utils(pcie_replay_num_test)
  function new(string n, uvm_component p); super.new(n, p); endfunction
  function void configure();
    super.configure();
    cfg.err_allowed[0] = 16'h2817; cfg.err_allowed[1] = 16'h2817;
  endfunction
  task main_body(uvm_phase phase);
    pcie_base_vseq b = pcie_base_vseq::type_id::create("b");
    bit [1:0] r; bytes_t w, rb;
    bit [63:0] B = cfg.bar0_base[1];
    b.cfg = cfg; b.set_sequencer(env.vsqr);
    for (int k = 1; k <= 3; k++) begin
      b.apply_fi(0, b.mk_fi(1, 1, 2, 0, 0, k, 400 + k));              // corrupt the next k TLP frames (first try and k-1 replays)
      w = b.rnd_bytes(64); b.do_write(0, B + 64'h800 + k * 64'h80, w, 4, r);
      b.wait_cycles(2000);
      b.do_read(0, B + 64'h800 + k * 64'h80, 64, 4, rb, r); b.chk(rb, w, $sformatf("after %0d corrupted tries", k));
      b.clear_fi(0);
    end
  endtask
endclass

// coalescer edge cases, optimisations on (M25, M26)
class pcie_coalesce_edge_test extends pcie_base_test;
  `uvm_component_utils(pcie_coalesce_edge_test)
  function new(string n, uvm_component p); super.new(n, p); endfunction
  function void configure();
    super.configure();
    cfg.opt_mode[0] = 1; cfg.opt_mode[1] = 1;
  endfunction
  task main_body(uvm_phase phase);
    coalesce_edge_vseq s = coalesce_edge_vseq::type_id::create("s");
    s.start(env.vsqr);
  endtask
endclass

// BAR window edges and Memory Space Enable (M30, M31)
class pcie_bar_edge_test extends pcie_base_test;
  `uvm_component_utils(pcie_bar_edge_test)
  function new(string n, uvm_component p); super.new(n, p); endfunction
  function void configure();
    super.configure();
    cfg.err_allowed[0] = 16'h4FEF; cfg.err_allowed[1] = 16'h4FEF;
  endfunction
  task main_body(uvm_phase phase);
    bar_edge_vseq s = bar_edge_vseq::type_id::create("s");
    s.sb_h = env.axi_sb;
    s.start(env.vsqr);
  endtask
endclass

// MRd with partial First / Last BE (REPLACE_HDR on the wire): completion Byte Count and Lower Address (M21)
class pcie_partial_be_test extends pcie_base_test;
  `uvm_component_utils(pcie_partial_be_test)
  function new(string n, uvm_component p); super.new(n, p); endfunction
  function void configure();
    super.configure();
    cfg.err_allowed[0] = 16'hFFFF; cfg.err_allowed[1] = 16'hFFFF;
  endfunction
  task main_body(uvm_phase phase);
    pcie_base_vseq b = pcie_base_vseq::type_id::create("b");
    bit [1:0] r; bytes_t w, rb;
    bit [63:0] B = cfg.bar0_base[1];
    bit [7:0] xs[] = '{8'h01, 8'h03, 8'h07, 8'h80, 8'hC0, 8'hE0, 8'h81, 8'hE7};   // byte 7 of the MRd header = {Last BE, First BE}
    b.cfg = cfg; b.set_sequencer(env.vsqr);
    w = b.rnd_bytes(256); b.do_write(0, B + 64'h1000, w, 4, r);
    b.wait_cycles(200);
    foreach (xs[i]) begin
      link_pkg::link_fi_t c = b.mk_fi(5, 7, 2, 0, 0, 1, 500 + i);
      c.match_fmt_type = 8'h00;                                       // MRd, 3DW
      c.hdr_xor = 128'(xs[i]) << 56;
      cfg.be_ovr_fbe = xs[i][3:0]; cfg.be_ovr_lbe = xs[i][7:4]; cfg.rd_slverr_ok = 1;
      b.apply_fi(0, c);
      b.do_read(0, B + 64'h1000 + i * 64'h40, 64, 4, rb, r);
      b.clear_fi(0);
      cfg.be_ovr_fbe = 0; cfg.be_ovr_lbe = 0; cfg.rd_slverr_ok = 0;
      b.wait_cycles(500);
    end
  endtask
endclass

// ---------------------------------------------------------------------------------
// RAL test (spec 10.1 / 9.2): uvm_reg built-in reset and bit-bash sequences on both
// nodes, then the registers the generic sequences cannot check (clamps, W1C,
// write-only self-clearing bits, unmapped addresses CS-2, counter snapshot CS-3),
// then the reset values are restored and the normal bring-up and traffic run.
// ---------------------------------------------------------------------------------
class pcie_ral_test extends pcie_base_test;
  `uvm_component_utils(pcie_ral_test)
  function new(string n, uvm_component p); super.new(n, p); endfunction
  function void configure();
    super.configure();
    cfg.err_allowed[0] = 16'h0800; cfg.err_allowed[1] = 16'h0800;
  endfunction

  task chk(pcie_csr_reg r, bit [31:0] wv, bit [31:0] exp, string what);
    uvm_status_e st; uvm_reg_data_t d;
    r.write(st, wv);
    r.read(st, d);
    if (d[31:0] !== exp) `uvm_error("RAL", $sformatf("%s: wrote %h read %h expected %h", what, wv, d[31:0], exp))
  endtask

  task run_phase(uvm_phase phase);
    uvm_reg_hw_reset_seq rst_s; uvm_reg_bit_bash_seq bb_s;
    bringup_vseq bu = bringup_vseq::type_id::create("bu");
    rand_traffic_vseq r = rand_traffic_vseq::type_id::create("r");
    drain_vseq   dr = drain_vseq::type_id::create("dr");
    endcheck_vseq ec = endcheck_vseq::type_id::create("ec");
    uvm_status_e st; uvm_reg_data_t d;
    phase.raise_objection(this);
    wait (env.c_vif[0].dl_up === 1'b1 && env.c_vif[1].dl_up === 1'b1);
    repeat (40) @(posedge env.c_vif[0].clk);
    for (int n = 0; n < 2; n++) begin
      rst_s = uvm_reg_hw_reset_seq::type_id::create($sformatf("rst%0d", n));
      rst_s.model = env.regmodel[n]; rst_s.start(null);
      bb_s = uvm_reg_bit_bash_seq::type_id::create($sformatf("bb%0d", n));
      bb_s.model = env.regmodel[n]; bb_s.start(null);
      // clamps (9.2): REPLAY_WIN_TLPS in 1..MAX_REPLAY_TLPS, MAX_OUTSTANDING_RD in 1..MAX_TAGS
      chk(env.regmodel[n].replay_cfg, 32'h0400_0000, 32'h0400_0001, "REPLAY_CFG win=0 clamps to 1");
      chk(env.regmodel[n].replay_cfg, 32'h0400_00FF, 32'h0400_0040, "REPLAY_CFG win=255 clamps to 64");
      chk(env.regmodel[n].replay_cfg, 32'h0400_0008, 32'h0400_0008, "REPLAY_CFG win=8");
      chk(env.regmodel[n].rd_cfg, 32'h0, 32'h1, "RD_CFG 0 clamps to 1");
      chk(env.regmodel[n].rd_cfg, 32'hFF, 32'(cfg.max_tags), "RD_CFG 255 clamps to MAX_TAGS");
      // CTRL: self-clearing command bits read back 0, LINK_ENABLE stays 1
      chk(env.regmodel[n].ctrl, 32'h0000_0009, 32'h0000_0001, "CTRL SNAPSHOT self-clears");
      // ERR_STATUS W1C with nothing set reads 0
      chk(env.regmodel[n].err_status, 32'h0000_FFFF, 32'h0, "ERR_STATUS W1C");
      // counters: snapshot then read through the model (values are volatile, only the access is checked)
      foreach (env.regmodel[n].cnt[i]) env.regmodel[n].cnt[i].read(st, d);
      env.regmodel[n].cnt[0].read(st, d);
      if (d == 0) `uvm_error("RAL", "CNT_CYCLES is 0 after SNAPSHOT with the link up")
      // CS-2: unmapped addresses read 0 and ignore writes
      begin
        bit [31:0] v;
        env.vsqr.csr[n].write(12'h0C0, 32'hDEAD_BEEF); env.vsqr.csr[n].read(12'h0C0, v);
        if (v !== 0) `uvm_error("RAL", $sformatf("unmapped 0x0C0 reads %h", v))
        env.vsqr.csr[n].read(12'h034, v);
        if (v !== 0) `uvm_error("RAL", $sformatf("unmapped 0x034 reads %h", v))
      end
      // restore every RW register to its reset value
      env.regmodel[n].replay_cfg.write(st, 32'h0400_0008);
      env.regmodel[n].rd_cfg.write(st, cfg.max_tags);
      env.regmodel[n].opt_ctrl.write(st, 0);
      env.regmodel[n].ack_cfg.write(st, env.regmodel[n].ack_cfg.get_reset());
      env.regmodel[n].fc_cfg.write(st, env.regmodel[n].fc_cfg.get_reset());
      env.regmodel[n].wr_coal_cfg.write(st, env.regmodel[n].wr_coal_cfg.get_reset());
      env.regmodel[n].cpl_timeout.write(st, env.regmodel[n].cpl_timeout.get_reset());
      env.regmodel[n].err_fatal_mask.write(st, env.regmodel[n].err_fatal_mask.get_reset());
      env.regmodel[n].fc_cfg2.write(st, env.regmodel[n].fc_cfg2.get_reset());
      `uvm_info("RAL", $sformatf("node%0d: reset + bit-bash + directed register checks done", n), UVM_LOW)
    end
    // the link must still work: normal flow
    bu.start(env.vsqr);
    r.nops = 20; r.start(env.vsqr);
    dr.start(env.vsqr);
    ec.env = env; ec.start(env.vsqr);
    phase.drop_objection(this);
  endtask
endclass
