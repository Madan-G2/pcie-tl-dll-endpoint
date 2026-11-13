// ============================================================================
// pcie_cov : functional coverage (spec 12, subset)
// ============================================================================
`uvm_analysis_imp_decl(_c_frame)
`uvm_analysis_imp_decl(_c_axi)
`uvm_analysis_imp_decl(_c_fi)

class pcie_cov extends uvm_component;
  `uvm_component_utils(pcie_cov)
  uvm_analysis_imp_c_frame #(link_frame, pcie_cov) frame_imp;
  uvm_analysis_imp_c_axi   #(axi_txn,    pcie_cov) axi_imp;
  uvm_analysis_imp_c_fi    #(fi_evt,     pcie_cov) fi_imp;
  pcie_env_cfg cfg;

  covergroup cg_tlp with function sample(bit [7:0] ft, int len_dw, bit is_replay_seq);
    option.per_instance = 1;
    cp_type: coverpoint ft { bins mrd32 = {8'h00}; bins mrd64 = {8'h20}; bins mwr32 = {8'h40}; bins mwr64 = {8'h60};
                             bins cfgrd = {8'h04}; bins cfgwr = {8'h44}; bins cpl = {8'h0A}; bins cpld = {8'h4A}; }
    cp_len: coverpoint len_dw { bins one = {1}; bins two_4 = {[2:4]}; bins five_16 = {[5:16]}; bins sixteen_32 = {[17:32]}; bins big = {[33:64]}; bins huge = {[65:128]}; }
    x_type_len: cross cp_type, cp_len;
  endgroup
  covergroup cg_dllp with function sample(bit [7:0] t, bit crc_ok);
    option.per_instance = 1;
    cp_type: coverpoint t { bins ack = {8'h00}; bins nak = {8'h10}; bins ifc1[] = {8'h40, 8'h50, 8'h60}; bins ifc2[] = {8'hC0, 8'hD0, 8'hE0};
                            bins ufc[] = {8'h80, 8'h90, 8'hA0}; }
    cp_crc: coverpoint crc_ok;
  endgroup
  covergroup cg_axi with function sample(bit rw, int size, int len, bit partial, bit unsup, bit [1:0] resp);
    option.per_instance = 1;
    cp_rw: coverpoint rw;
    cp_size: coverpoint size { bins s4 = {2}; bins s8 = {3}; bins s16 = {4}; bins other = default; }
    cp_len: coverpoint len { bins one = {1}; bins few = {[2:4]}; bins mid = {[5:16]}; bins big = {[17:64]}; bins max = {[65:256]}; }
    cp_partial: coverpoint partial;
    cp_unsup: coverpoint unsup;
    cp_resp: coverpoint resp { bins ok = {0}; bins slverr = {2}; bins other = default; }
    x_rw_size: cross cp_rw, cp_size;
    x_rw_len:  cross cp_rw, cp_len;
  endgroup
  covergroup cg_fi with function sample(bit [3:0] action, bit dllp, int dir);
    option.per_instance = 1;
    cp_action: coverpoint action { bins corrupt = {1}; bins drop = {2}; bins dup = {3}; bins seqoff = {4}; bins replhdr = {5}; bins linkdown = {6}; bins delay = {7}; }
    cp_dllp: coverpoint dllp;
    cp_dir: coverpoint dir;
    x_act_kind: cross cp_action, cp_dllp { ignore_bins tlp_only_dllp = binsof(cp_action) intersect {4, 5} && binsof(cp_dllp) intersect {1}; }
  endgroup
  covergroup cg_err with function sample(int bit_idx);
    option.per_instance = 1;
    cp_bit: coverpoint bit_idx { bins b[] = {[0:15]}; }
  endgroup

  function new(string n, uvm_component p);
    super.new(n, p);
    cg_tlp = new(); cg_dllp = new(); cg_axi = new(); cg_fi = new(); cg_err = new();
  endfunction
  function void build_phase(uvm_phase phase);
    frame_imp = new("frame_imp", this); axi_imp = new("axi_imp", this); fi_imp = new("fi_imp", this);
    if (!uvm_config_db#(pcie_env_cfg)::get(this, "", "cfg", cfg)) `uvm_fatal("NOCFG", "pcie_cov cfg")
  endfunction
  function void write_c_frame(link_frame f);
    if (!f.is_in) return;
    if (f.is_dllp) cg_dllp.sample(f.dtype, f.crc16_ok);
    else if (f.tlp != null && f.tlp.supported) cg_tlp.sample(f.tlp.ft(), f.tlp.len, 0);
  endfunction
  function void write_c_axi(axi_txn t);
    bit partial = 0; bit unsup = 0;
    foreach (t.strb[i]) if (!t.strb[i]) partial = 1;
    unsup = (t.resp == 2'b10);
    cg_axi.sample(t.rw, t.size, int'(t.len) + 1, partial, unsup, t.resp);
  endfunction
  function void write_c_fi(fi_evt e);
    cg_fi.sample(e.action, e.dllp, e.dir);
  endfunction
  function void sample_err(bit [15:0] es);
    for (int i = 0; i < 16; i++) if (es[i]) cg_err.sample(i);
  endfunction
  function void report_phase(uvm_phase phase);
    `uvm_info("COV", $sformatf("tlp=%0.1f%% dllp=%0.1f%% axi=%0.1f%% fi=%0.1f%% err=%0.1f%%",
              cg_tlp.get_inst_coverage(), cg_dllp.get_inst_coverage(), cg_axi.get_inst_coverage(), cg_fi.get_inst_coverage(), cg_err.get_inst_coverage()), UVM_LOW)
  endfunction
endclass
