// ============================================================================
// pcie_env_cfg (spec 10.2)
// ============================================================================
class pcie_env_cfg extends uvm_object;
  `uvm_object_utils(pcie_env_cfg)
  // protocol configuration programmed by bring-up
  int unsigned mps_code  = 1;                    // 0:128 1:256
  int unsigned mrrs_code = 2;                    // 0:128 1:256 2:512
  int unsigned max_tags  = 16;
  bit [63:0]   bar0_base [2] = '{64'h1000_0000, 64'h1000_0000};
  int unsigned bar0_size_log2 = 20;
  bit [15:0]   dev_id [2] = '{16'h0100, 16'h0200};
  // credits advertised (RTL defaults)
  int unsigned p_hdr_cr = 32, p_data_cr = 128, np_hdr_cr = 32, np_data_cr = 32, cpl_hdr_cr = 0, cpl_data_cr = 0;
  int unsigned replay_win = 8;
  int unsigned ack_timer_cyc = 32;
  int unsigned ack_bound_cyc = 3000;            // SB-8 : liveness bound (link busy / replays) for "an accepted TLP gets acknowledged"
  // SB-8 tight bound (DL-15 / A-DL-18): Ack at the receiver's TX tap no later than
  //   ACK_TIMER_CYC + MAX_FRAME_BEATS + 4 + ack_wire_lat after the TLP's eop at its RX tap
  int unsigned max_frame_beats = 18;             // (MPS_MAX 256 + 16 + 2 + 4 + 15) / 16
  int unsigned ack_wire_lat = 8;                 // dll_rx accept pipeline (CRC stages) + TX output register, cycles
  bit          ack_tight_en = 1;
  // SB-5 / DL-18 : first replayed sop <= 8 + DLLP beats sent first, after the Nak is decoded
  int unsigned nak_rx_lat = 4;                   // Nak eop at the transmitter's RX tap -> replay trigger inside dll_tx (measured minimum 5 to sop)
  bit          replay_lat_en = 1;
  // SB-12
  bit          sb_en_order = 1;
  int unsigned ord_guard = 4;                    // a write is "older" than a read if accepted >= ord_guard cycles earlier
  // CSR presets
  bit          opt_mode [2] = '{0, 0};           // 0 baseline, 1 optimized
  bit [5:0]    opt_mask = 6'h3F;                  // OPT_CTRL value written in optimized mode (+OPTMASK=hex: single-switch runs, T37)
  bit [31:0]   extra_csr_addr[$];
  bit [31:0]   extra_csr_data[$];
  int          ack_n = 4;
  // memory (m_axi slave agent) policy per node: 0 fixed, 1 random, 2 bimodal
  int          mem_policy [2] = '{0, 0};
  int          mem_delay  [2] = '{10, 10};
  bit          mem_ooo    [2] = '{0, 0};
  bit [63:0]   mem_err_lo [2] = '{64'h0, 64'h0};
  bit [63:0]   mem_err_hi [2] = '{64'h0, 64'h0};   // lo==hi : no error range
  int          slow_pct = 20;                     // bimodal policy: percentage of slow accesses (W4)
  int          axi_bp_pct = 0;                    // random bready/rready de-assertion on the master agent
  // link
  link_pkg::link_fi_t fi_cfg [2];
  // checking
  bit          sb_en_axi = 1, sb_en_dll = 1, sb_en_tl = 1, sb_en_fc = 1, sb_en_perf = 1;
  bit [15:0]   err_allowed  [2] = '{16'h0000, 16'h0000};   // err_status bits that are acceptable
  bit [15:0]   err_required [2] = '{16'h0000, 16'h0000};   // err_status bits that must be set (SB-15)
  bit          bringup_done = 0;
  bit [3:0]    be_ovr_fbe = 0, be_ovr_lbe = 0;     // a test rewrites MRd byte enables on the wire (REPLACE_HDR) : XOR masks the scoreboards apply
  bit          rd_slverr_ok = 0;                   // such a test lets the requester's read end with SLVERR (completion mismatch -> timeout)
  bit          check_counters = 1;
  int unsigned n_transactions = 40;
  int unsigned seed = 1;
  string       test_name = "test";
  // end-of-test counter snapshots filled in by the test: cnt[node][csr addr]
  bit [31:0]   cnt [2][bit [11:0]];
  bit [31:0]   err_status [2];
  bit          cnt_valid = 0;

  function new(string name = "pcie_env_cfg");
    super.new(name);
    foreach (fi_cfg[i]) fi_cfg[i] = '0;
  endfunction

  // is [addr, addr+n) inside BAR0 of the node that receives (completer)?
  function bit in_bar0(int completer, bit [63:0] a);
    return (a >= bar0_base[completer]) && (a < bar0_base[completer] + (64'd1 << bar0_size_log2));
  endfunction
  function bit in_mem_err(int node, bit [63:0] off);
    return (mem_err_hi[node] != mem_err_lo[node]) && (off >= mem_err_lo[node]) && (off < mem_err_hi[node]);
  endfunction
  function int mps_bytes(); return 128 << mps_code; endfunction
  function int mrrs_bytes(); return 128 << mrrs_code; endfunction
endclass
