// ============================================================================
// order_sb : SB-12 (TL-8) ordering at the TX tap of ONE node n
//   * P in order : MWr TLPs carry the bytes of the AXI writes in the order the
//     writes were accepted at s_axi (a write is accepted when its AW and its
//     last W beat have both been seen).
//   * no NP overtakes an older P : an MRd must not reach the wire while an
//     AXI write that was accepted at least ORD_GUARD cycles before the MRd's
//     AR handshake still has bytes that have not been sent.
//   Writes answered with SLVERR (AX-1, never become a TLP) are dropped from
//   the model when their B response arrives, so a violation is only reported
//   when the overtaken write is really sent later (lazy confirmation).
//   Config accesses (ECAM window) are CfgWr / CfgRd (NP) and not tracked.
//   Cpl vs P ordering is checked inside tl_tx by SVA a_tl_5a / a_tl_5b.
// ============================================================================
`uvm_analysis_imp_decl(_o_wacc)
`uvm_analysis_imp_decl(_o_ar)
`uvm_analysis_imp_decl(_o_b)
`uvm_analysis_imp_decl(_o_r)
`uvm_analysis_imp_decl(_o_tap)
`uvm_analysis_imp_decl(_o_dl)

class order_sb extends uvm_component;
  `uvm_component_utils(order_sb)
  uvm_analysis_imp_o_wacc #(axi_txn,    order_sb) wacc_imp;
  uvm_analysis_imp_o_ar   #(axi_txn,    order_sb) ar_imp;
  uvm_analysis_imp_o_b    #(axi_txn,    order_sb) b_imp;
  uvm_analysis_imp_o_r    #(axi_txn,    order_sb) r_imp;
  uvm_analysis_imp_o_tap  #(link_frame, order_sb) tap_imp;
  uvm_analysis_imp_o_dl   #(dl_event,   order_sb) dl_imp;
  pcie_env_cfg cfg;
  int node;

  typedef struct {
    int unsigned uid; longint t; bit [3:0] id; bit [63:0] lo;
    int pend [bit [63:0]];           // byte address -> 1 while not yet sent
    bit got_b; bit sent_some; string overtaken;
  } wrec_t;
  wrec_t   wq [$];
  int      early [bit [63:0]];        // bytes sent before their write was fully accepted (cut-through of long bursts)
  axi_txn  arq [$];
  int unsigned uid_n;
  bit          up [2];
  int          next_seq;              // next new sequence number at this tap
  // stats
  int unsigned n_p, n_np, n_chk_np, n_viol;

  localparam longint ECAM_LO = 64'h0000_0000_F000_0000, ECAM_HI = 64'h0000_0001_0000_0000;

  function new(string n, uvm_component p); super.new(n, p); endfunction
  function void build_phase(uvm_phase phase);
    wacc_imp = new("wacc_imp", this); ar_imp = new("ar_imp", this); b_imp = new("b_imp", this);
    r_imp = new("r_imp", this); tap_imp = new("tap_imp", this); dl_imp = new("dl_imp", this);
    if (!uvm_config_db#(pcie_env_cfg)::get(this, "", "cfg", cfg)) `uvm_fatal("NOCFG", "order_sb cfg")
    if (!uvm_config_db#(int)::get(this, "", "node", node)) node = 0;
  endfunction

  function bit is_cfg_addr(bit [63:0] a); return a >= ECAM_LO && a < ECAM_HI; endfunction

  // ---- AXI side -------------------------------------------------------------------
  function void write_o_wacc(axi_txn t);
    wrec_t r;
    if (!cfg.sb_en_order || is_cfg_addr(t.addr)) return;
    r.uid = uid_n++; r.t = t.t_acc; r.id = t.id; r.lo = t.addr;
    begin
      bit [63:0] base = t.addr & ~((64'd1 << t.size) - 1);
      foreach (t.strb[i]) if (t.strb[i]) begin
        if (early.exists(base + i)) begin early[base + i]--; if (early[base + i] == 0) early.delete(base + i); r.sent_some = 1; end
        else r.pend[base + i] = 1;
      end
    end
    if (r.pend.size() == 0 && r.sent_some) begin r.got_b = 0; wq.push_back(r); return; end
    if (r.pend.size() == 0) return;                       // no enabled byte : nothing to send
    if ($test$plusargs("ORD_TRACE")) `uvm_info("ORDT", $sformatf("node%0d WACC id=%0d addr=%h len=%0d size=%0d bytes=%0d t=%0d", node, t.id, t.addr, t.len, t.size, r.pend.size(), r.t), UVM_NONE)
    wq.push_back(r);
  endfunction

  function void write_o_b(axi_txn t);
    if (!cfg.sb_en_order) return;
    foreach (wq[i]) if (!wq[i].got_b && wq[i].id == t.id && wq[i].lo == t.addr) begin
      wq[i].got_b = 1;
      if (t.resp != 2'b00 && !wq[i].sent_some) wq.delete(i);   // SLVERR : the write never becomes a TLP
      break;
    end
    purge();
  endfunction

  function void write_o_ar(axi_txn t);
    if (!cfg.sb_en_order || is_cfg_addr(t.addr)) return;
    arq.push_back(t);
  endfunction

  function void write_o_r(axi_txn t);
    foreach (arq[i]) if (arq[i].id == t.id && arq[i].addr == t.addr) begin arq.delete(i); break; end
  endfunction

  function void write_o_dl(dl_event e);
    up[e.node] = e.up;
    if (!e.up) begin next_seq = 0; wq.delete(); arq.delete(); early.delete(); end   // link down: queues flushed, writes lost (LM-8)
    else if (up[0] && up[1]) next_seq = 0;
  endfunction

  function void purge();
    while (wq.size() > 0 && wq[0].pend.size() == 0 && wq[0].got_b) void'(wq.pop_front());
  endfunction

  // ---- TX tap ---------------------------------------------------------------------
  function void write_o_tap(link_frame f);
    tlp_item t;
    if (!cfg.sb_en_order || f.is_dllp || !f.lcrc_ok || f.tlp == null) return;
    if (f.seq != next_seq) return;                        // replayed frame : order was checked the first time
    next_seq = (next_seq + 1) & 12'hFFF;
    t = f.tlp;
    if ($test$plusargs("ORD_TRACE")) `uvm_info("ORDT", $sformatf("node%0d seq=%0d %s", node, f.seq, t.sprint_short()), UVM_NONE)
    if (t.is_wr && t.is_mem) check_p(t, f);
    else if (t.is_rd && t.is_mem) check_np(t, f);
  endfunction

  // byte addresses enabled by a memory request header
  function void tlp_bytes(tlp_item t, ref bit [63:0] q[$]);
    int n = (t.len == 0) ? 1024 : int'(t.len);
    for (int dw = 0; dw < n; dw++)
      for (int b = 0; b < 4; b++) begin
        bit en = (dw == 0) ? t.fbe[b] : ((dw == n - 1) ? t.lbe[b] : 1'b1);
        if (n == 1) en = t.fbe[b];
        if (en) q.push_back(t.addr + dw * 4 + b);
      end
  endfunction

  function void check_p(tlp_item t, link_frame f);
    bit [63:0] q[$];
    n_p++;
    tlp_bytes(t, q);
    foreach (q[k]) begin
      bit hit = 0;
      foreach (wq[i]) if (wq[i].pend.exists(q[k])) begin
        hit = 1;
        wq[i].pend.delete(q[k]);
        if (!wq[i].sent_some && wq[i].overtaken != "") begin
          n_viol++;
          `uvm_error("SB-12", $sformatf("node%0d: AXI write @%h (accepted cycle %0d) %s", node, wq[i].lo, wq[i].t, wq[i].overtaken))
        end
        wq[i].sent_some = 1;
        break;
      end
      if (!hit) early[q[k]]++;                           // the write is still receiving W beats
    end
    // P in order : every write older than the youngest one this TLP carried must be finished
    begin
      int last_touched = -1;
      foreach (wq[i]) if (wq[i].sent_some) last_touched = i;
      for (int i = 0; i < last_touched; i++)
        if (wq[i].pend.size() > 0 && wq[i].overtaken == "")
          wq[i].overtaken = $sformatf("was overtaken by the younger MWr seq=%0d addr=%h at cycle %0d (P order)", f.seq, t.addr, f.start_cycle);
    end
    purge();
  endfunction

  function void check_np(tlp_item t, link_frame f);
    longint t_ar = -1;
    n_np++;
    foreach (arq[i]) if (t.addr >= arq[i].addr && t.addr < arq[i].addr + arq[i].nbytes()) begin t_ar = arq[i].t_start; break; end
    if (t_ar < 0) return;                                  // not from this node's s_axi (should not happen) : nothing to compare
    n_chk_np++;
    foreach (wq[i]) if (wq[i].pend.size() > 0) begin
      longint margin = t_ar - wq[i].t;
      if (margin >= longint'(cfg.ord_guard) && wq[i].overtaken == "")
        wq[i].overtaken = $sformatf("was overtaken by MRd seq=%0d addr=%h (AR accepted %0d cycles after the write) at cycle %0d (NP passed P)",
                                    f.seq, t.addr, margin, f.start_cycle);
    end
  endfunction

  function void check_phase(uvm_phase phase);
    if (!cfg.sb_en_order) return;
    foreach (wq[i]) if (wq[i].pend.size() > 0 && wq[i].got_b && cfg.check_counters)
    begin
      bit [63:0] fa; void'(wq[i].pend.first(fa));
      `uvm_error("SB-12", $sformatf("node%0d: AXI write @%h (accepted cycle %0d, OKAY) : %0d bytes never sent as MWr (first %h)", node, wq[i].lo, wq[i].t, wq[i].pend.size(), fa))
    end
  endfunction

  function void report_phase(uvm_phase phase);
    `uvm_info("ORDER_SB", $sformatf("node%0d: MWr checked=%0d MRd seen=%0d MRd ordered against writes=%0d violations=%0d",
              node, n_p, n_np, n_chk_np, n_viol), UVM_LOW)
  endfunction
endclass
