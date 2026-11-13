// ============================================================================
// perf_mon : independent counting from the taps (SB-14), CSV output (section 8)
//   node n transmits on direction n and receives on direction 1-n.
// ============================================================================
`uvm_analysis_imp_decl(_p_in0)
`uvm_analysis_imp_decl(_p_in1)
`uvm_analysis_imp_decl(_p_out0)
`uvm_analysis_imp_decl(_p_out1)

class perf_mon extends uvm_component;
  `uvm_component_utils(perf_mon)
  uvm_analysis_imp_p_in0  #(link_frame, perf_mon) in0_imp;
  uvm_analysis_imp_p_in1  #(link_frame, perf_mon) in1_imp;
  uvm_analysis_imp_p_out0 #(link_frame, perf_mon) out0_imp;
  uvm_analysis_imp_p_out1 #(link_frame, perf_mon) out1_imp;
  uvm_analysis_imp_dl     #(dl_event,   perf_mon) dl_imp;
  pcie_env_cfg cfg;
  typedef longint unsigned cnt_t;
  cnt_t live [2][bit [11:0]];
  cnt_t snap [2][bit [11:0]];
  bit   snap_valid [2];
  int   next_seq [2];       // new-TLP detection on the tx side of node n
  int   nrs [2];            // receive-side model of node n
  longint first_cycle [2], last_cycle [2];
  bit   dl_ok;
  function new(string n, uvm_component p); super.new(n, p); endfunction
  function void build_phase(uvm_phase phase);
    in0_imp = new("in0_imp", this); in1_imp = new("in1_imp", this); out0_imp = new("out0_imp", this); out1_imp = new("out1_imp", this); dl_imp = new("dl_imp", this);
    if (!uvm_config_db#(pcie_env_cfg)::get(this, "", "cfg", cfg)) `uvm_fatal("NOCFG", "perf_mon cfg")
  endfunction
  function void inc(int n, bit [11:0] a, cnt_t v = 1);
    if (!live[n].exists(a)) live[n][a] = 0;
    live[n][a] += v;
  endfunction

  function void tx_frame(int n, link_frame f);
    if (first_cycle[n] == 0) first_cycle[n] = f.start_cycle;
    last_cycle[n] = f.end_cycle;
    if (f.is_dllp) begin
      inc(n, CNT_TX_BEATS_DLLP);
      if (f.crc16_ok) begin
        if (f.is_ack()) inc(n, CNT_ACK_TX);
        if (f.is_nak()) inc(n, CNT_NAK_TX);
        if (f.fc_kind() == 3) inc(n, CNT_UPDFC_TX);
      end
    end else begin
      inc(n, CNT_TX_BEATS_TLP, f.beats);
      if (f.seq == next_seq[n]) begin
        next_seq[n] = (next_seq[n] + 1) & 12'hFFF;
        inc(n, CNT_TX_TLP);
        if (f.tlp != null && f.tlp.has_data) inc(n, CNT_TX_PAY, f.tlp.len * 4);
      end else inc(n, CNT_TX_BEATS_REPLAY, f.beats);
    end
  endfunction
  function void rx_frame(int n, link_frame f);
    if (f.is_dllp) return;
    if (!f.lcrc_ok || f.bytes.size() < 18) inc(n, CNT_RX_LCRC);
    else if (f.seq == nrs[n]) begin
      nrs[n] = (nrs[n] + 1) & 12'hFFF;
      inc(n, CNT_RX_TLP);
      if (f.tlp != null && f.tlp.has_data) inc(n, CNT_RX_PAY, f.tlp.payload.size());     // bytes received (a malformed Length must not count)
    end else if (((nrs[n] - f.seq) & 12'hFFF) <= 2048) inc(n, CNT_RX_DUP);
  endfunction
  function void write_p_in0(link_frame f);  tx_frame(0, f); endfunction
  function void write_p_in1(link_frame f);  tx_frame(1, f); endfunction
  function void write_p_out0(link_frame f); rx_frame(1, f); endfunction     // direction 0 is received by node 1
  function void write_p_out1(link_frame f); rx_frame(0, f); endfunction

  function void write_dl(dl_event e); reset_models(e.node); endfunction
  function void reset_models(int n);
    next_seq[n] = 0; nrs[n] = 0;
  endfunction

  // the test calls this at the instant it writes ctrl.SNAPSHOT
  function void take_snapshot(int n);
    snap[n] = live[n]; snap_valid[n] = 1;
  endfunction

  function void check_phase(uvm_phase phase);
    bit [11:0] chk [$] = '{CNT_TX_TLP, CNT_TX_PAY, CNT_TX_BEATS_TLP, CNT_TX_BEATS_REPLAY, CNT_RX_TLP, CNT_RX_PAY,
                           CNT_RX_LCRC, CNT_RX_DUP, CNT_ACK_TX, CNT_NAK_TX, CNT_UPDFC_TX, CNT_TX_BEATS_DLLP};
    if (!cfg.sb_en_perf || !cfg.cnt_valid) return;
    for (int n = 0; n < 2; n++) begin
      if (!snap_valid[n]) continue;
      foreach (chk[i]) begin
        bit [11:0] a = chk[i];
        cnt_t exp = snap[n].exists(a) ? snap[n][a] : 0;
        cnt_t got = cfg.cnt[n].exists(a) ? cfg.cnt[n][a] : 0;
        // DLLP counters may be off by an in-flight periodic UpdateFC at the snapshot instant
        int tol = (a == CNT_UPDFC_TX || a == CNT_TX_BEATS_DLLP) ? 2 : 0;
        longint diff = longint'(got) - longint'(exp);
        if (diff > tol || diff < -tol)
          `uvm_error("SB-14", $sformatf("node%0d CSR counter 0x%03h = %0d but the taps counted %0d", n, a, got, exp))
      end
    end
  endfunction

  function void report_phase(uvm_phase phase);
    string tag; int unsigned seed;
    int fd;
    if (!$value$plusargs("PERF_CSV=%s", tag)) return;
    fd = $fopen(tag, "a");
    if (fd == 0) return;
    for (int n = 0; n < 2; n++) begin
      cnt_t cyc = (last_cycle[n] > first_cycle[n]) ? (last_cycle[n] - first_cycle[n]) : 1;
      cnt_t pay = live[n].exists(CNT_TX_PAY) ? live[n][CNT_TX_PAY] : 0;
      cnt_t bt = live[n].exists(CNT_TX_BEATS_TLP) ? live[n][CNT_TX_BEATS_TLP] : 0;
      cnt_t bd = live[n].exists(CNT_TX_BEATS_DLLP) ? live[n][CNT_TX_BEATS_DLLP] : 0;
      cnt_t tlp = live[n].exists(CNT_TX_TLP) ? live[n][CNT_TX_TLP] : 0;
      cnt_t rp = live[n].exists(CNT_TX_BEATS_REPLAY) ? live[n][CNT_TX_BEATS_REPLAY] : 0;
      $fdisplay(fd, "%s,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%f", cfg.test_name, n, cyc, tlp, pay, bt, bd, rp, opt_flag(), real'(pay) / real'(cyc));
    end
    $fclose(fd);
  endfunction
  function int opt_flag(); return int'(cfg.opt_mode[0]); endfunction
endclass
