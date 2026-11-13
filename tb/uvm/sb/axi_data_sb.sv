// ============================================================================
// axi_data_sb : end-to-end data integrity (SB-1, SB-2, SB-3), spec 10.4
//   node n's s_axi traffic is served by node (1-n)'s memory on m_axi.
// ============================================================================
typedef class axi_data_sb;

class axi_fwd extends uvm_subscriber #(axi_txn);
  axi_data_sb sb; int kind; int node;
  function new(string name, uvm_component parent); super.new(name, parent); endfunction
  function void write(axi_txn t);
    sb.handle(kind, node, t);
  endfunction
endclass

class axi_data_sb extends uvm_component;
  `uvm_component_utils(axi_data_sb)
  pcie_env_cfg cfg;
  // kind: 0 s_wr, 1 s_rd, 2 m_wr, 3 m_rd
  axi_fwd s_wr[2], s_rd[2], m_wr[2], m_rd[2];
  // per-target-node, per-offset byte queues
  byte unsigned exp_q [2][longint unsigned][$];    // expected write bytes not yet seen on m_axi
  byte unsigned got_q [2][longint unsigned][$];    // bytes seen on m_axi before the expectation arrived
  byte unsigned rdo_q [2][longint unsigned][$];    // bytes returned by memory reads (m_axi) not yet consumed
  int unsigned n_bytes_ok, n_rd_ok, n_unsup, n_ur, n_mismatch;
  function new(string n, uvm_component p); super.new(n, p); endfunction
  function void build_phase(uvm_phase phase);
    if (!uvm_config_db#(pcie_env_cfg)::get(this, "", "cfg", cfg)) `uvm_fatal("NOCFG", "axi_data_sb cfg")
    for (int n = 0; n < 2; n++) begin
      s_wr[n] = new($sformatf("s_wr%0d", n), this); s_wr[n].sb = this; s_wr[n].kind = 0; s_wr[n].node = n;
      s_rd[n] = new($sformatf("s_rd%0d", n), this); s_rd[n].sb = this; s_rd[n].kind = 1; s_rd[n].node = n;
      m_wr[n] = new($sformatf("m_wr%0d", n), this); m_wr[n].sb = this; m_wr[n].kind = 2; m_wr[n].node = n;
      m_rd[n] = new($sformatf("m_rd%0d", n), this); m_rd[n].sb = this; m_rd[n].kind = 3; m_rd[n].node = n;
    end
  endfunction

  // ---- predictors for AX-1 / AX-2 / CF-4 ---------------------------------------
  function bit is_ecam(bit [63:0] a);
    return (a >= ECAM_BASE) && (a < ECAM_BASE + 64'h1000_0000);
  endfunction
  function bit axi_supported(axi_txn t);
    int sz = 1 << t.size;
    if (t.burst != 2'b01) return 0;
    if (!(t.size inside {2, 3, 4})) return 0;
    if ((t.addr & (sz - 1)) != 0) return 0;
    if (((t.addr & 64'hFFF) + (int'(t.len) + 1) * sz) > 4096) return 0;
    return 1;
  endfunction
  function bit strobes_ok(axi_txn t);          // AX-2 : a single contiguous run of enabled bytes
    int f = -1, l = -1;
    foreach (t.strb[i]) if (t.strb[i]) begin if (f < 0) f = i; l = i; end
    if (f < 0) return 0;
    for (int i = f; i <= l; i++) if (!t.strb[i]) return 0;
    return 1;
  endfunction
  function bit cfg_ok(axi_txn t);
    return axi_supported(t) && t.size == 2 && t.len == 0;
  endfunction

  function void handle(int kind, int node, axi_txn t);
    int tgt = 1 - node;
    if (!cfg.sb_en_axi) return;                    // tests that lose traffic on purpose (T38) switch the data checks off
    case (kind)
      0: begin   // s_axi write at node
        if (is_ecam(t.addr)) begin
          if (!cfg_ok(t) && t.resp != 2'b10) `uvm_error("SB-1", $sformatf("unsupported ECAM write must give SLVERR: %s", t.convert2string()))
          return;
        end
        if (!axi_supported(t) || !strobes_ok(t)) begin
          n_unsup++;
          if (t.resp != 2'b10) `uvm_error("SB-1", $sformatf("AX-1/AX-2 unsupported write must return SLVERR, got %0d: %s", t.resp, t.convert2string()))
          return;
        end
        if (t.resp != 2'b00) begin `uvm_error("SB-1", $sformatf("legal write returned resp=%0d: %s", t.resp, t.convert2string())) return; end
        if (!cfg.in_bar0(tgt, t.addr)) begin n_ur++; return; end            // posted write dropped by the completer (UR_POSTED)
        foreach (t.data[i]) if (t.strb[i]) begin
          longint unsigned off = t.addr + i - cfg.bar0_base[tgt];
          exp_q[tgt][off].push_back(t.data[i]);
          match_w(tgt, off);
        end
      end
      2: begin   // m_axi write at node
        foreach (t.data[i]) if (t.strb[i]) begin
          longint unsigned a = t.addr + i;
          got_q[node][a].push_back(t.data[i]);
          rdo_q[node].delete(a);                                           // older read observations of this byte are stale
          match_w(node, a);
        end
      end
      3: begin   // m_axi read at node : remember what the memory returned
        if (t.resp != 2'b00) return;
        foreach (t.data[i]) begin
          longint unsigned a = t.addr + i;
          rdo_q[node][a].push_back(t.data[i]);
          if (rdo_q[node][a].size() > 32) void'(rdo_q[node][a].pop_front());
        end
      end
      1: begin   // s_axi read at node
        if (is_ecam(t.addr)) begin
          if (!cfg_ok(t) && t.resp != 2'b10) `uvm_error("SB-2", $sformatf("unsupported ECAM read must give SLVERR: %s", t.convert2string()))
          return;
        end
        if (!axi_supported(t)) begin
          n_unsup++;
          if (t.resp != 2'b10) `uvm_error("SB-2", $sformatf("unsupported read must return SLVERR: %s", t.convert2string()))
          foreach (t.data[i]) if (t.data[i] != 0) begin `uvm_error("SB-2", "AX-1 read error beats must carry zero data") break; end
          return;
        end
        if (!cfg.in_bar0(tgt, t.addr)) begin
          n_ur++;
          if (t.resp != 2'b10) `uvm_error("SB-2", $sformatf("read outside BAR0 must return SLVERR (UR), got %0d: %s", t.resp, t.convert2string()))
          return;
        end
        if (cfg.in_mem_err(tgt, t.addr - cfg.bar0_base[tgt])) begin
          if (t.resp != 2'b10) `uvm_error("SB-2", $sformatf("read of a failing memory range must return SLVERR (CA): %s", t.convert2string()))
          return;
        end
        if (t.resp == 2'b10 && cfg.rd_slverr_ok) return;
        if (t.resp != 2'b00) begin `uvm_error("SB-2", $sformatf("legal read returned resp=%0d: %s", t.resp, t.convert2string())) return; end
        foreach (t.data[i]) begin
          longint unsigned off = t.addr + i - cfg.bar0_base[tgt];
          int hit = -1;
          if (rdo_q[tgt].exists(off)) foreach (rdo_q[tgt][off][k]) if (rdo_q[tgt][off][k] == t.data[i]) begin hit = k; break; end
          if (hit < 0) begin
            n_mismatch++;
            `uvm_error("SB-2/3", $sformatf("read data mismatch at node%0d memory offset %h: returned %02h, memory never produced that value (%s)",
                       tgt, off, t.data[i], t.convert2string()))
            return;
          end
          rdo_q[tgt][off].delete(hit);
        end
        n_rd_ok++;
      end
    endcase
  endfunction

  function void match_w(int n, longint unsigned a);
    while (exp_q[n].exists(a) && exp_q[n][a].size() > 0 && got_q[n].exists(a) && got_q[n][a].size() > 0) begin
      byte unsigned e = exp_q[n][a].pop_front();
      byte unsigned g = got_q[n][a].pop_front();
      if (e !== g) begin
        n_mismatch++;
        `uvm_error("SB-1", $sformatf("node%0d memory write byte at %h: expected %02h (from the remote s_axi write), got %02h", n, a, e, g))
      end else n_bytes_ok++;
    end
  endfunction

  // tests that deliberately make the completer drop writes (e.g. malformed TLP) forget the expectation
  function void forget_pending(int tgt);
    exp_q[tgt].delete();
  endfunction

  function void check_phase(uvm_phase phase);
    if (!cfg.sb_en_axi) return;
    for (int n = 0; n < 2; n++) begin
      foreach (exp_q[n][a]) if (exp_q[n][a].size() > 0) begin
        `uvm_error("SB-1", $sformatf("node%0d: %0d accepted write byte(s) at %h never appeared on m_axi (first=%02h)", n, exp_q[n][a].size(), a, exp_q[n][a][0])) break;
      end
      foreach (got_q[n][a]) if (got_q[n][a].size() > 0) begin
        `uvm_error("SB-1", $sformatf("node%0d: m_axi wrote byte(s) at %h that no s_axi write accounts for (first=%02h)", n, a, got_q[n][a][0])) break;
      end
    end
  endfunction

  function void report_phase(uvm_phase phase);
    `uvm_info("AXI_SB", $sformatf("bytes matched=%0d  read txns checked=%0d  unsupported=%0d  UR/outside-BAR=%0d  mismatches=%0d",
              n_bytes_ok, n_rd_ok, n_unsup, n_ur, n_mismatch), UVM_LOW)
  endfunction
endclass
