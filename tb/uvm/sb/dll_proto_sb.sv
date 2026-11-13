// ============================================================================
// dll_proto_sb : link-layer protocol checks for ONE direction d (SB-4 .. SB-8)
//   _in   : frames at the input tap of d  (what the transmitter put on the wire)
//   _out  : frames at the output tap of d (what the receiver got, after fault injection)
//   _rin  : reverse-direction input tap   (Ack/Nak DLLPs the receiver sent)
//   _rout : reverse-direction output tap  (Ack/Nak DLLPs the transmitter received)
// ============================================================================
`uvm_analysis_imp_decl(_in)
`uvm_analysis_imp_decl(_out)
`uvm_analysis_imp_decl(_rin)
`uvm_analysis_imp_decl(_rout)
`uvm_analysis_imp_decl(_dl)

class dll_proto_sb extends uvm_component;
  `uvm_component_utils(dll_proto_sb)
  uvm_analysis_imp_in   #(link_frame, dll_proto_sb) in_imp;
  uvm_analysis_imp_out  #(link_frame, dll_proto_sb) out_imp;
  uvm_analysis_imp_rin  #(link_frame, dll_proto_sb) rin_imp;
  uvm_analysis_imp_rout #(link_frame, dll_proto_sb) rout_imp;
  uvm_analysis_imp_dl   #(dl_event,   dll_proto_sb) dl_imp;
  pcie_env_cfg cfg;
  int dir;

  bit up [2];
  bit armed;
  // transmitter view
  int next_seq, acked_seq;
  bytes_t store [int];
  bytes_t grave [int]; longint grave_t [int];       // recently acked frames : a replay may still be in the DUT pipeline
  bit in_replay; int last_replay_seq;
  bit nak_pending; longint nak_t; bit nak_sweep_seen;
  typedef struct { longint t; int oldest; } oh_t;
  oh_t ohist [$];
  // receiver view
  int nrs; bit nak_sched;
  typedef struct { longint t; int nrs; } nh_t;
  nh_t nhist [$];
  bit exp_nak; longint exp_nak_t;
  typedef struct { int seq; longint t; } pa_t;
  pa_t pend_ack [$];
  // stats
  int unsigned n_new, n_replay, n_ack, n_nak, n_dup_ack, n_bad_rx, n_seqahead, n_dllp_bad;
  // DL-18 replay latency (wire level) and DL-15 Ack latency
  bit     rp_wait; longint rp_trig; int rp_dllp_beats; longint prev_in_end = -1;
  int unsigned n_rp_lat; longint max_rp_lat = -1000, max_ack_lat;

  function new(string n, uvm_component p); super.new(n, p); endfunction
  function void build_phase(uvm_phase phase);
    in_imp = new("in_imp", this); out_imp = new("out_imp", this); rin_imp = new("rin_imp", this);
    rout_imp = new("rout_imp", this); dl_imp = new("dl_imp", this);
    if (!uvm_config_db#(pcie_env_cfg)::get(this, "", "cfg", cfg)) `uvm_fatal("NOCFG", "dll_proto_sb cfg")
    if (!uvm_config_db#(int)::get(this, "", "dir", dir)) dir = 0;
  endfunction

  function int md(int a, int b); return (a - b) & 12'hFFF; endfunction
  function int outstanding(); return md(next_seq - 1, acked_seq); endfunction

  // transmitter view (node dir) and receiver view (node 1-dir) are armed and reset separately: after a reset one node
  // can be DL_Active a few cycles before the other and already send (the peer discards those frames, DL-17)
  bit tx_armed, rx_armed;
  function void reset_tx();
    next_seq = 0; acked_seq = 4095; store.delete(); grave.delete(); grave_t.delete(); in_replay = 0; nak_pending = 0; nak_sweep_seen = 0; ohist.delete();
    rp_wait = 0;
    ohist.push_back('{cyc_now(), 0});
  endfunction
  function void reset_rx();
    nrs = 0; nak_sched = 0; nhist.delete(); exp_nak = 0; pend_ack.delete();
    nhist.push_back('{cyc_now(), 0});
  endfunction
  function void reset_state(); reset_tx(); reset_rx(); endfunction

  function void write_dl(dl_event e);
    up[e.node] = e.up;
    if (e.node == dir)     begin tx_armed = e.up; reset_tx(); end
    else                   begin rx_armed = e.up; reset_rx(); end
    armed = up[0] && up[1];
  endfunction

  function bit oldest_recent(int seq, longint now);
    if (seq == ((acked_seq + 1) & 12'hFFF)) return 1;
    foreach (ohist[i]) if (now - ohist[i].t <= 24 && ohist[i].oldest == seq) return 1;
    return 0;
  endfunction
  function bit nrs_recent(int v, longint now);       // v == NEXT_RCV_SEQ-1 at some time within the last 24 cycles
    int start = 0;
    if (md(nrs - 1, 0) == v) return 1;
    // the entry that was in force at (now-24), and every later one
    foreach (nhist[i]) if (nhist[i].t <= now - 24) start = i;
    for (int i = start; i < nhist.size(); i++) if (md(nhist[i].nrs - 1, 0) == v) return 1;
    return 0;
  endfunction

  // ---- input tap : what the transmitter does -------------------------------------
  bit trace;
  longint rp_base_end;
  function void write_in(link_frame f);
    if ($test$plusargs("DLL_TRACE")) `uvm_info("TRC", $sformatf("dir%0d IN   %s", dir, f.sprint_short()), UVM_NONE)
    if (!cfg.sb_en_dll || !tx_armed) begin prev_in_end = f.end_cycle; return; end
    if (rp_wait) begin
      // a frame already on the line when the replay was triggered is the "frame in flight" of DL-6
      if (f.start_cycle <= rp_trig && !f.is_dllp && f.seq != next_seq) rp_wait = 0;   // the replay was already under way
      else if (f.start_cycle <= rp_trig && f.end_cycle >= rp_trig && f.end_cycle > rp_base_end) rp_base_end = f.end_cycle;
      if (!rp_wait) begin end
      else if (f.start_cycle > rp_trig && f.is_dllp) begin
        rp_dllp_beats += f.beats;                    // DLLPs sent before the replay (DL-5) add one cycle each
      end else if (f.start_cycle > rp_trig && f.seq != next_seq) begin           // first replayed TLP frame after the trigger
        // the replay starts at the end of the frame in flight (or at the trigger on an idle line)
        longint base = rp_trig;
        longint lat;
        if (rp_base_end >= rp_trig) base = rp_base_end + 1;
        lat = f.start_cycle - base;
        n_rp_lat++; if (lat - rp_dllp_beats > max_rp_lat) max_rp_lat = lat - rp_dllp_beats;
        if (cfg.replay_lat_en && lat > 8 + rp_dllp_beats)
          `uvm_error("SB-5", $sformatf("DL-18: first replayed sop (seq=%0d) %0d cycles after the replay start (limit 8 + %0d DLLP beats)",
                     f.seq, lat, rp_dllp_beats))
        rp_wait = 0;
      end
    end
    prev_in_end = f.end_cycle;
    if (f.is_dllp) begin
      if (!f.crc16_ok)     `uvm_error("SB-8", $sformatf("DLLP with wrong CRC-16 at the input tap: %s", f.sprint_short()))
      if (!f.legal_type)   `uvm_error("SB-8", $sformatf("DLLP with illegal type at the input tap: %s", f.sprint_short()))
      return;
    end
    if (!f.lcrc_ok) `uvm_error("SB-4", $sformatf("TLP frame with wrong LCRC at the input tap: %s", f.sprint_short()))
    if (f.bytes[0][7:4] != 0) `uvm_error("SB-4", $sformatf("sequence prefix reserved bits set: %02h", f.bytes[0]))
    if (f.seq == next_seq) begin
      if (outstanding() >= int'(cfg.replay_win))
        `uvm_error("SB-7", $sformatf("%0d TLPs outstanding exceeds REPLAY_WIN_TLPS=%0d when sending seq %0d", outstanding() + 1, cfg.replay_win, f.seq))
      if (nak_pending && f.start_cycle > nak_t + 24 && !nak_sweep_seen && outstanding() > 0)
        `uvm_error("SB-5", $sformatf("new TLP seq=%0d sent %0d cycles after a Nak instead of replaying the oldest unacked (%0d)",
                   f.seq, f.start_cycle - nak_t, (acked_seq + 1) & 12'hFFF))
      store[f.seq] = f.bytes; next_seq = (next_seq + 1) & 12'hFFF; in_replay = 0; n_new++;
    end else begin
      n_replay++;
      if (!store.exists(f.seq) && grave.exists(f.seq) && (f.start_cycle - grave_t[f.seq]) <= 24) begin
        if (grave[f.seq] != f.bytes) `uvm_error("SB-5", $sformatf("replayed frame seq=%0d is not byte-identical to the original", f.seq))
        in_replay = 1; last_replay_seq = f.seq;          // raced with the Ack that freed it
      end else if (!store.exists(f.seq)) begin
        `uvm_error("SB-5", $sformatf("replayed/unknown TLP seq=%0d (next new=%0d, acked=%0d)", f.seq, next_seq, acked_seq))
      end else begin
        if (store[f.seq] != f.bytes) `uvm_error("SB-5", $sformatf("replayed frame seq=%0d is not byte-identical to the original", f.seq))
        if (!in_replay || f.seq != ((last_replay_seq + 1) & 12'hFFF)) begin
          if (!oldest_recent(f.seq, f.start_cycle))
            `uvm_error("SB-5", $sformatf("replay sweep starts at seq=%0d, oldest unacked is %0d", f.seq, (acked_seq + 1) & 12'hFFF))
        end
        in_replay = 1; last_replay_seq = f.seq; nak_sweep_seen = 1;
      end
    end
  endfunction

  // ---- reverse output tap : Ack/Nak as received by the transmitter ----------------
  function void write_rout(link_frame f);
    if (!cfg.sb_en_dll || !tx_armed) return;
    if (!f.is_dllp || !f.crc16_ok || !(f.is_ack() || f.is_nak())) return;
    begin
      int s = f.dseq; int d = md(s, acked_seq);
      if (d <= outstanding()) begin
        if (f.is_nak()) begin
          if (md(next_seq - 1, s) != 0) begin
            nak_pending = 1; nak_t = f.start_cycle; nak_sweep_seen = 0;
            rp_wait = 1; rp_trig = f.end_cycle + cfg.nak_rx_lat; rp_dllp_beats = 0;
            rp_base_end = (prev_in_end >= rp_trig) ? prev_in_end : -1;
          end
        end
        if (d > 0) begin
          for (int k = 1; k <= d; k++) begin
            int sq = (acked_seq + k) & 12'hFFF;
            if (store.exists(sq)) begin grave[sq] = store[sq]; grave_t[sq] = f.start_cycle; end
            store.delete(sq);
          end
          acked_seq = s;
          ohist.push_back('{f.start_cycle, (acked_seq + 1) & 12'hFFF});
          if (ohist.size() > 64) void'(ohist.pop_front());
        end
      end
    end
  endfunction

  // ---- output tap : what the receiver sees -----------------------------------------
  function void write_out(link_frame f);
    longint now;
    if ($test$plusargs("DLL_TRACE")) `uvm_info("TRC", $sformatf("dir%0d OUT  %s end=%0d", dir, f.sprint_short(), f.end_cycle), UVM_NONE)
    if (!cfg.sb_en_dll || !rx_armed) return;
    now = f.end_cycle;
    if (f.is_dllp) return;
    if (!f.lcrc_ok || f.bytes.size() < 18) begin
      n_bad_rx++;
      if (!nak_sched) begin nak_sched = 1; exp_nak = 1; exp_nak_t = now; end
    end else if (f.seq == nrs) begin
      nrs = (nrs + 1) & 12'hFFF; nak_sched = 0; exp_nak = 0;
      nhist.push_back('{now, nrs}); if (nhist.size() > 64) void'(nhist.pop_front());
      pend_ack.push_back('{f.seq, now});
    end else if (md(nrs, f.seq) > 0 && md(nrs, f.seq) <= 2048) begin
      n_dup_ack++;                                    // duplicate : immediate Ack, no Nak
    end else begin
      n_seqahead++;
      if (!nak_sched) begin nak_sched = 1; exp_nak = 1; exp_nak_t = now; end
    end
  endfunction

  // ---- reverse input tap : Ack/Nak as sent by the receiver ---------------------------
  function void write_rin(link_frame f);
    if ($test$plusargs("DLL_TRACE")) `uvm_info("TRC", $sformatf("dir%0d RIN  %s", dir, f.sprint_short()), UVM_NONE)
    if (!cfg.sb_en_dll || !rx_armed) return;
    if (!f.is_dllp || !f.crc16_ok) return;
    if (!(f.is_ack() || f.is_nak())) return;
    begin
      int s = f.dseq; longint now = f.start_cycle;
      if (!nrs_recent(s, now))
        `uvm_error("SB-6", $sformatf("%s carries %0d but NEXT_RCV_SEQ-1 was %0d (recent history)", f.is_nak() ? "Nak" : "Ack", s, md(nrs - 1, 0)))
      if (f.is_nak()) begin
        n_nak++;
        if (!exp_nak) `uvm_error("SB-6", $sformatf("Nak(%0d) sent without an error episode (or a second Nak in the same episode)", s))
        exp_nak = 0;
      end else n_ack++;
      // everything up to s is acknowledged
      while (pend_ack.size() > 0 && md(s, pend_ack[0].seq) < 2048) begin
        longint lat = now - pend_ack[0].t;
        if (lat > max_ack_lat) max_ack_lat = lat;
        if (cfg.ack_tight_en && lat > longint'(cfg.ack_timer_cyc + cfg.max_frame_beats + 4 + cfg.ack_wire_lat))
          `uvm_error("SB-8", $sformatf("DL-15: TLP seq=%0d acknowledged %0d cycles after it was accepted (limit ACK_TIMER_CYC %0d + MAX_FRAME_BEATS %0d + 4 + %0d)",
                     pend_ack[0].seq, lat, cfg.ack_timer_cyc, cfg.max_frame_beats, cfg.ack_wire_lat))
        void'(pend_ack.pop_front());
      end
    end
  endfunction

  task run_phase(uvm_phase phase);
    forever begin
      repeat (16) #(4ns);
      if (armed && cfg.sb_en_dll) begin
        longint now = cyc_now();
        if (pend_ack.size() > 0 && now - pend_ack[0].t > cfg.ack_bound_cyc)
          `uvm_error("SB-8", $sformatf("TLP seq=%0d accepted %0d cycles ago has not been acknowledged (bound %0d)", pend_ack[0].seq, now - pend_ack[0].t, cfg.ack_bound_cyc))
        if (exp_nak && now - exp_nak_t > cfg.ack_bound_cyc)
          `uvm_error("SB-6", $sformatf("no Nak %0d cycles after an error episode started (bound %0d)", now - exp_nak_t, cfg.ack_bound_cyc))
      end
    end
  endtask

  function void report_phase(uvm_phase phase);
    `uvm_info("DLL_SB", $sformatf("dir%0d: new TLPs=%0d replayed=%0d acks=%0d naks=%0d dup-acks=%0d bad rx frames=%0d seq-ahead=%0d",
              dir, n_new, n_replay, n_ack, n_nak, n_dup_ack, n_bad_rx, n_seqahead), UVM_LOW)
    `uvm_info("DLL_SB", $sformatf("dir%0d: max Ack latency=%0d cycles (limit %0d); Nak-triggered replays timed=%0d, max replay latency (minus DLLP beats)=%0d (limit 8)",
              dir, max_ack_lat, cfg.ack_timer_cyc + cfg.max_frame_beats + 4 + cfg.ack_wire_lat, n_rp_lat, max_rp_lat), UVM_LOW)
  endfunction
endclass
