// ============================================================================
// tl_rx : TLP parse / check (TL-17, 4.6), routing, RX credit accounting
//         (TL-18, TL-19, FC-4), NP buffer + dispatch (completer reads, config,
//         UR completions).  Payload of MWr / Cpl TLPs is forwarded as raw
//         streams to the bridges; this block never back-pressures (LK-4).
// ============================================================================
module tl_rx
  import pcie_pkg::*;
#(
  parameter int MPS_MAX_BYTES  = 256,
  parameter int BAR0_SIZE_LOG2 = 20,
  parameter logic [15:0] DEVICE_ID = 16'h0100,
  parameter int P_HDR_CR   = 32,  parameter int P_DATA_CR  = 128,
  parameter int NP_HDR_CR  = 32,  parameter int NP_DATA_CR = 32,
  parameter int CPL_HDR_CR = 0,   parameter int CPL_DATA_CR = 0
) (
  input  logic         clk,
  input  logic         rst_n,
  input  logic         dl_up,

  // from DLL
  input  logic         dl_rx_valid,
  input  logic [127:0] dl_rx_data,
  input  logic [3:0]   dl_rx_keep,
  input  logic         dl_rx_sop,
  input  logic         dl_rx_eop,

  // config state
  input  logic         mse,
  input  logic [31:0]  bar0,
  input  logic [2:0]   mps_code,

  // inbound MWr stream -> mst_wr_path
  output logic         mw_valid,
  output logic [127:0] mw_data,
  output logic [3:0]   mw_keep,
  output logic         mw_sop,
  output logic         mw_eop,
  output logic [27:0]  mw_off,        // BAR0 offset of first byte (DW aligned), valid with sop
  output logic [3:0]   mw_fbe,
  output logic [3:0]   mw_lbe,
  output logic [9:0]   mw_len,
  output logic         mw_hdr4,
  output logic         mw_res_v,      // result pulse for the TLP that just ended (E+1)
  output logic         mw_res_ok,
  // credit releases from mst_wr_path
  input  logic         p_rel,
  input  logic [11:0]  p_rel_data,

  // inbound Cpl stream -> axi_slv_bridge
  output logic         cp_valid,
  output logic [127:0] cp_data,
  output logic [3:0]   cp_keep,
  output logic         cp_sop,
  output logic         cp_eop,
  output logic         cp_bad_v,      // eop-time failure of an already forwarded completion
  output logic [7:0]   cp_bad_tag,

  // NP reads -> mst_rd_path
  output logic         nd_valid,
  input  logic         nd_ready,
  output logic [27:0]  nd_off,
  output logic [9:0]   nd_len,
  output logic [3:0]   nd_fbe,
  output logic [3:0]   nd_lbe,
  output logic [15:0]  nd_reqid,
  output logic [7:0]   nd_tag,
  output logic [15:0]  nd_snap,
  input  logic         np_rel,        // AR issued

  // simple completions (UR / config) -> tl_tx cpl1
  output logic         cpl1_valid,
  input  logic         cpl1_ready,
  output logic [127:0] cpl1_data,
  output logic [3:0]   cpl1_keep,
  output logic         cpl1_sop,
  output logic         cpl1_eop,

  // cfg_space access
  output logic         cfg_acc_valid,
  output logic         cfg_acc_we,
  output logic [9:0]   cfg_acc_dw,
  output logic [31:0]  cfg_acc_wdata,
  output logic [3:0]   cfg_acc_be,
  input  logic [31:0]  cfg_acc_rdata,

  // FC allocation (to dll_fc)
  output logic [7:0]   fc_ca_p_hdr,   output logic [11:0] fc_ca_p_data,
  output logic [7:0]   fc_ca_np_hdr,  output logic [11:0] fc_ca_np_data,
  output logic [7:0]   fc_ca_cpl_hdr, output logic [11:0] fc_ca_cpl_data,
  output logic         fc_rel_pulse_p, fc_rel_pulse_np, fc_rel_pulse_cpl,

  // events
  output logic         ev_malformed,
  output logic         ev_poisoned,
  output logic         ev_ur_posted,
  output logic         ev_ur_sent,
  output logic         ev_fc_violation,
  output logic [15:0]  p_acc_cnt,
  output logic         rx_idle
);
  localparam int MPS_MAX_DW = MPS_MAX_BYTES / 4;

  // ---------------------------------------------------------------------------
  // beat-0 capture and decode
  // ---------------------------------------------------------------------------
  logic [127:0] b0_r;
  logic [10:0]  dw_acc;           // DWs seen before this beat
  logic         in_tlp;           // between sop and eop
  wire  [127:0] b0 = dl_rx_sop ? dl_rx_data : b0_r;

  wire [31:0] h0 = bsw32(b0[31:0]);
  wire [31:0] h1 = bsw32(b0[63:32]);
  wire [31:0] h2 = bsw32(b0[95:64]);
  wire [31:0] h3 = bsw32(b0[127:96]);
  wire [2:0]  fmt = h0[31:29];
  wire [4:0]  typ = h0[28:24];
  wire [7:0]  ft  = {fmt, typ};
  wire [2:0]  tc  = h0[22:20];
  wire        th  = h0[16], td = h0[15], ep = h0[14];
  wire [1:0]  at  = h0[11:10];
  wire [9:0]  len = h0[9:0];
  wire [3:0]  fbe = h1[3:0], lbe = h1[7:4];
  wire [7:0]  tag = h1[15:8];
  wire [15:0] reqid = h1[31:16];

  wire is_mrd  = (ft == FT_MRD32)  || (ft == FT_MRD64);
  wire is_mwr  = (ft == FT_MWR32)  || (ft == FT_MWR64);
  wire is_cfgrd = (ft == FT_CFGRD0), is_cfgwr = (ft == FT_CFGWR0);
  wire is_cfg  = is_cfgrd || is_cfgwr;
  wire is_cpl  = (ft == FT_CPL), is_cpld = (ft == FT_CPLD);
  wire is_cplx = is_cpl || is_cpld;
  wire sup     = is_mrd || is_mwr || is_cfg || is_cplx;
  wire hdr4    = fmt[0];
  wire [2:0] hdr_dws = hdr4 ? 3'd4 : 3'd3;
  wire [63:0] addr = hdr4 ? {h2, h3[31:2], 2'b00} : {32'b0, h2[31:2], 2'b00};
  wire [11:0] len_bytes = {len[9:0], 2'b00} == 12'd0 ? 12'd0 : {len[9:0], 2'b00};
  wire [12:0] end_off = {1'b0, addr[11:0]} + {1'b0, len[9:0], 2'b00};

  wire [10:0] mps_bytes = (mps_code == 3'd0) ? 11'd128 : 11'd256;

  // header-time malformed conditions (TL-17, TL-2..TL-5)
  wire be_bad  = (is_mrd || is_mwr || is_cfg) &&
                 ((len == 10'd1) ? ((lbe != 4'b0) || !be_contig(fbe))
                                 : (!fbe_ok_multi(fbe) || !lbe_ok_multi(lbe)));
  wire hdr_mal = !sup || td || (tc != 3'b0) || (at != 2'b0) || th ||
                 ((is_mrd || is_mwr || is_cfg || is_cpld) && (len == 10'd0)) ||
                 ((is_mwr || is_cpld) && ({1'b0, len, 2'b00} > {2'b0, mps_bytes})) ||
                 (is_mrd && (len > 10'd128)) ||
                 ((is_mrd || is_mwr) && (end_off > 13'd4096)) ||
                 be_bad ||
                 (is_cfg && (len != 10'd1));

  // address / BAR
`ifdef MUT_30
  wire [31:0] bar_mask = ~((32'd1 << (BAR0_SIZE_LOG2 + 1)) - 32'd1);
`else
  wire [31:0] bar_mask = ~((32'd1 << BAR0_SIZE_LOG2) - 32'd1);
`endif
`ifdef MUT_31
  wire        bar_hit  = 1'b1 && (addr[63:32] == 32'b0) && ((addr[31:0] & bar_mask) == (bar0 & bar_mask));
`else
  wire        bar_hit  = mse && (addr[63:32] == 32'b0) && ((addr[31:0] & bar_mask) == (bar0 & bar_mask));
`endif
  wire [27:0] bar_off  = addr[27:0] & ((28'd1 << BAR0_SIZE_LOG2) - 28'd1);
  wire [2:0]  cfg_func = h2[18:16];

  // credit class (0 P, 1 NP, 2 Cpl) even for unsupported types
  logic [1:0] cls;
  always_comb begin
    if (typ[4:3] == 2'b10) cls = 2'd0;                         // messages are posted
    else if (typ[4:1] == 4'b0101) cls = 2'd2;                  // Cpl / CplLk
    else if (typ == 5'b00000 && fmt[1]) cls = 2'd0;            // MWr
    else cls = 2'd1;
  end

  // streams
  wire in_tlp_or_sop = dl_rx_sop || in_tlp;
  wire mw_feed = dl_rx_valid && in_tlp_or_sop && is_mwr && !hdr_mal && !ep && bar_hit;
  wire cp_feed = dl_rx_valid && in_tlp_or_sop && is_cplx && !hdr_mal;

  assign mw_valid = mw_feed; assign mw_data = dl_rx_data; assign mw_keep = dl_rx_keep;
  assign mw_sop = dl_rx_sop; assign mw_eop = dl_rx_eop;
  assign mw_off = bar_off; assign mw_fbe = fbe; assign mw_lbe = lbe; assign mw_len = len; assign mw_hdr4 = hdr4;
  assign cp_valid = cp_feed; assign cp_data = dl_rx_data; assign cp_keep = dl_rx_keep;
  assign cp_sop = dl_rx_sop; assign cp_eop = dl_rx_eop;

  // ---------------------------------------------------------------------------
  // eop-time decisions
  // ---------------------------------------------------------------------------
  wire [2:0]  beat_dws  = pop4(dl_rx_keep);
  wire [10:0] total_dws = (dl_rx_sop ? 11'd0 : dw_acc) + {8'b0, beat_dws};
  wire [10:0] exp_dws   = {8'b0, hdr_dws} + ((fmt[1] && !is_cpl) ? {1'b0, len} : 11'd0);
  wire        cnt_bad   = (total_dws != exp_dws);
  wire        eop_ev    = dl_rx_valid && in_tlp_or_sop && dl_rx_eop && dl_up;

  // credits the sender consumed for this TLP, from what was actually received
  wire [10:0] pl_dws_act = (total_dws > {8'b0, hdr_dws}) ? (total_dws - {8'b0, hdr_dws}) : 11'd0;
  wire [11:0] need_d_act = {1'b0, (pl_dws_act + 11'd3) >> 2};
  wire [11:0] need_d_hdr = (fmt[1] && !is_cpl) ? {2'b0, (len + 10'd3) >> 2} : 12'd0;   // header based

  wire mal_eop   = eop_ev && (hdr_mal || cnt_bad);
  wire ok_eop    = eop_ev && !hdr_mal && !cnt_bad;

  wire poisoned  = ok_eop && is_mwr && ep;
  wire ur_posted = ok_eop && is_mwr && !ep && !bar_hit;
  wire mwr_acc   = ok_eop && is_mwr && !ep && bar_hit;
  wire np_mem    = ok_eop && is_mrd;
  wire np_cfg    = ok_eop && is_cfg;
  wire cpl_acc   = ok_eop && is_cplx;

  // ---------------------------------------------------------------------------
  // credit accounting
  // ---------------------------------------------------------------------------
  function automatic logic [7:0]  ADV_H(input int c);
    return (c == 0) ? 8'(P_HDR_CR)  : (c == 1) ? 8'(NP_HDR_CR)  : 8'(CPL_HDR_CR);
  endfunction
  function automatic logic [11:0] ADV_D(input int c);
    return (c == 0) ? 12'(P_DATA_CR) : (c == 1) ? 12'(NP_DATA_CR) : 12'(CPL_DATA_CR);
  endfunction

  logic [8:0]  used_h [0:2];
  logic [12:0] used_d [0:2];
  logic [7:0]  ca_h [0:2];
  logic [11:0] ca_d [0:2];

  // per-class need and violation (only for accepted P / NP)
  wire [11:0] need_d_acc = need_d_hdr;
  wire viol_p  = mwr_acc && (((P_HDR_CR  != 0) && (used_h[0] + 9'd1 > 9'(P_HDR_CR))) ||
                             ((P_DATA_CR != 0) && (used_d[0] + {1'b0, need_d_acc} > 13'(P_DATA_CR))));
  wire viol_np = (np_mem || np_cfg) && (((NP_HDR_CR != 0) && (used_h[1] + 9'd1 > 9'(NP_HDR_CR))) ||
                             ((NP_DATA_CR != 0) && (used_d[1] + {1'b0, need_d_acc} > 13'(NP_DATA_CR))));
  wire viol    = viol_p || viol_np;
  assign ev_fc_violation = viol;

  // release events (hdr, data) per class this cycle
  logic [7:0]  rel_h [0:2];
  logic [11:0] rel_d [0:2];
  logic [8:0]  cons_h [0:2];        // consumer releases (decrement used)
  logic [12:0] cons_d [0:2];
  logic        local_rel; logic local_rel_d;
  always_comb begin
    for (int c = 0; c < 3; c++) begin rel_h[c] = 8'd0; rel_d[c] = 12'd0; cons_h[c] = 9'd0; cons_d[c] = 13'd0; end
    // immediate releases for dropped TLPs and Cpl (class of the packet)
`ifdef MUT_18
    if (1'b0 && mal_eop)            begin rel_h[cls] = rel_h[cls] + 8'd1; rel_d[cls] = rel_d[cls] + need_d_act; end
`else
    if (mal_eop)            begin rel_h[cls] = rel_h[cls] + 8'd1; rel_d[cls] = rel_d[cls] + need_d_act; end
`endif
    if (poisoned || ur_posted) begin rel_h[0] = rel_h[0] + 8'd1; rel_d[0] = rel_d[0] + need_d_hdr; end
    if (cpl_acc)            begin rel_h[2] = rel_h[2] + 8'd1; rel_d[2] = rel_d[2] + need_d_hdr; end
    // consumer releases
    if (p_rel)    begin rel_h[0] = rel_h[0] + 8'd1; rel_d[0] = rel_d[0] + p_rel_data; cons_h[0] = 9'd1; cons_d[0] = {1'b0, p_rel_data}; end
    if (np_rel)   begin rel_h[1] = rel_h[1] + 8'd1;
                        cons_h[1] = cons_h[1] + 9'd1; end
    if (local_rel) begin rel_h[1] = rel_h[1] + 8'd1; rel_d[1] = rel_d[1] + {11'b0, local_rel_d};
                         cons_h[1] = cons_h[1] + 9'd1; cons_d[1] = cons_d[1] + {12'b0, local_rel_d}; end
  end
  // NP data credit of the CfgWr/MRd released via np_rel (MRd: none) -> no data

  wire [8:0]  chg_h [0:2];
  wire [12:0] chg_d [0:2];
  assign chg_h[0] = (mwr_acc && !viol_p) ? 9'd1 : 9'd0;
  assign chg_d[0] = (mwr_acc && !viol_p) ? {1'b0, need_d_acc} : 13'd0;
  assign chg_h[1] = ((np_mem || np_cfg) && !viol_np) ? 9'd1 : 9'd0;
  assign chg_d[1] = ((np_mem || np_cfg) && !viol_np) ? {1'b0, need_d_acc} : 13'd0;
  assign chg_h[2] = 9'd0; assign chg_d[2] = 13'd0;

  logic [2:0] pulse;
  always_comb begin
    for (int c = 0; c < 3; c++)
      pulse[c] = ((rel_h[c] != 0) && (ADV_H(c) != 0)) || ((rel_d[c] != 0) && (ADV_D(c) != 0));
  end
  assign {fc_rel_pulse_cpl, fc_rel_pulse_np, fc_rel_pulse_p} = pulse;
  assign fc_ca_p_hdr = ca_h[0];   assign fc_ca_p_data = ca_d[0];
  assign fc_ca_np_hdr = ca_h[1];  assign fc_ca_np_data = ca_d[1];
  assign fc_ca_cpl_hdr = ca_h[2]; assign fc_ca_cpl_data = ca_d[2];

  // ---------------------------------------------------------------------------
  // NP buffer and dispatch
  // ---------------------------------------------------------------------------
  localparam int NPD  = NP_HDR_CR;
  localparam int NPAW = $clog2(NPD);
  logic [127:0] npb_d [0:NPD-1];
  logic [1:0]   npb_k [0:NPD-1];       // 0 RD_HIT, 1 RD_UR, 2 CFG, 3 CFG_UR
  logic [27:0]  npb_o [0:NPD-1];
  logic [15:0]  npb_s [0:NPD-1];
  logic [NPAW:0] npb_wp, npb_rp;
  wire npb_empty = (npb_wp == npb_rp);
  logic [1:0] push_kind;
  always_comb begin
    if (np_mem) push_kind = bar_hit ? 2'd0 : 2'd1;
    else        push_kind = (cfg_func == 3'd0) ? 2'd2 : 2'd3;
  end
  wire npb_push = (np_mem || np_cfg) && !viol_np;

  wire [127:0] hd_d = npb_d[npb_rp[NPAW-1:0]];
  wire [1:0]   hd_k = npb_k[npb_rp[NPAW-1:0]];
  wire [27:0]  hd_o = npb_o[npb_rp[NPAW-1:0]];
  wire [15:0]  hd_s = npb_s[npb_rp[NPAW-1:0]];
  wire [31:0]  g0 = bsw32(hd_d[31:0]);
  wire [31:0]  g1 = bsw32(hd_d[63:32]);
  wire [31:0]  g2 = bsw32(hd_d[95:64]);

  logic c1_valid; logic [127:0] c1_data; logic [3:0] c1_keep;
  assign cpl1_valid = c1_valid; assign cpl1_data = c1_data; assign cpl1_keep = c1_keep;
  assign cpl1_sop = 1'b1; assign cpl1_eop = 1'b1;

  wire hd_rd_hit = !npb_empty && (hd_k == 2'd0);
  wire hd_local  = !npb_empty && (hd_k != 2'd0);
  assign nd_valid = hd_rd_hit;
  assign nd_off   = hd_o;  assign nd_len = g0[9:0];
  assign nd_fbe   = g1[3:0]; assign nd_lbe = g1[7:4];
  assign nd_reqid = g1[31:16]; assign nd_tag = g1[15:8]; assign nd_snap = hd_s;

  wire gen_local = hd_local && (!c1_valid || cpl1_ready);
  wire hd_is_cfgwr = (g0[31:24] == FT_CFGWR0);
  assign cfg_acc_valid = gen_local && (hd_k == 2'd2);
  assign cfg_acc_we    = hd_is_cfgwr;
  assign cfg_acc_dw    = {g2[11:8], g2[7:2]};
  assign cfg_acc_wdata = hd_d[127:96];
  assign cfg_acc_be    = g1[3:0];
  assign local_rel     = gen_local;
  assign local_rel_d   = (hd_k == 2'd2) && hd_is_cfgwr;
  assign ev_ur_sent    = gen_local && (hd_k == 2'd1 || hd_k == 2'd3);

  // build completion
  logic [31:0] c_dw0, c_dw1, c_dw2, c_dw3;
  logic [3:0]  c_keep;
  always_comb begin
`ifdef MUT_32
    c_dw1 = {DEVICE_ID, ((hd_k == 2'd1 || hd_k == 2'd3) ? CPL_UR : CPL_SC), 1'b0, ((hd_k == 2'd1 || hd_k == 2'd3) ? 12'd0 : 12'd4)};
`else
    c_dw1 = {DEVICE_ID, ((hd_k == 2'd1 || hd_k == 2'd3) ? CPL_UR : CPL_SC), 1'b0, 12'd4};
`endif
    c_dw2 = {g1[31:16], g1[15:8], 1'b0, 7'd0};
    c_dw3 = 32'h0;
    if (hd_k == 2'd2 && !hd_is_cfgwr) begin
      c_dw0 = {FT_CPLD, 24'b0}; c_dw0[9:0] = 10'd1; c_dw3 = cfg_acc_rdata; c_keep = 4'b1111;
    end else begin
      c_dw0 = {FT_CPL, 24'b0}; c_keep = 4'b0111;
    end
  end

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      b0_r <= '0; dw_acc <= '0; in_tlp <= 1'b0;
      npb_wp <= '0; npb_rp <= '0; c1_valid <= 1'b0; c1_data <= '0; c1_keep <= '0;
      for (int c = 0; c < 3; c++) begin
        used_h[c] <= '0; used_d[c] <= '0; ca_h[c] <= (ADV_H(c) != 0) ? ADV_H(c) : 8'd0; ca_d[c] <= (ADV_D(c) != 0) ? ADV_D(c) : 12'd0;
      end
      mw_res_v <= 1'b0; mw_res_ok <= 1'b0; cp_bad_v <= 1'b0; cp_bad_tag <= '0;
      ev_malformed <= 1'b0; ev_poisoned <= 1'b0; ev_ur_posted <= 1'b0;
      p_acc_cnt <= '0;
      for (int i = 0; i < NPD; i++) begin npb_d[i] <= '0; npb_k[i] <= '0; npb_o[i] <= '0; npb_s[i] <= '0; end
    end else begin
      ev_malformed <= mal_eop; ev_poisoned <= poisoned; ev_ur_posted <= ur_posted;
      mw_res_v <= 1'b0; cp_bad_v <= 1'b0;

      if (!dl_up) begin
        in_tlp <= 1'b0; npb_wp <= '0; npb_rp <= '0; c1_valid <= 1'b0;
        for (int c = 0; c < 3; c++) begin
          used_h[c] <= '0; used_d[c] <= '0;
          ca_h[c] <= (ADV_H(c) != 0) ? ADV_H(c) : 8'd0; ca_d[c] <= (ADV_D(c) != 0) ? ADV_D(c) : 12'd0;
        end
      end else begin
        // ---- stream tracking ----
        if (dl_rx_valid) begin
          if (dl_rx_sop) b0_r <= dl_rx_data;
          dw_acc <= dl_rx_eop ? 11'd0 : total_dws;
          in_tlp <= !dl_rx_eop;
        end

        // ---- result pulses to the bridges ----
        if (eop_ev && is_mwr && !hdr_mal && !ep && bar_hit) begin
          mw_res_v <= 1'b1; mw_res_ok <= ok_eop && !viol_p;
          if (ok_eop && !viol_p) p_acc_cnt <= p_acc_cnt + 16'd1;
        end
        if (eop_ev && is_cplx && !hdr_mal && cnt_bad) begin cp_bad_v <= 1'b1; cp_bad_tag <= tag; end

        // ---- NP buffer push / pop ----
        if (npb_push) begin
          npb_d[npb_wp[NPAW-1:0]] <= b0;
          npb_k[npb_wp[NPAW-1:0]] <= push_kind;
          npb_o[npb_wp[NPAW-1:0]] <= bar_off;
          npb_s[npb_wp[NPAW-1:0]] <= p_acc_cnt;
          npb_wp <= npb_wp + 1'b1;
        end
        if ((hd_rd_hit && nd_ready) || gen_local) npb_rp <= npb_rp + 1'b1;

        // ---- local completion register ----
        if (cpl1_ready && c1_valid) c1_valid <= 1'b0;
        if (gen_local) begin
          c1_valid <= 1'b1;
          c1_data  <= {32'h0, bsw32(c_dw2), bsw32(c_dw1), bsw32(c_dw0)} | {c_dw3, 96'h0};
          c1_keep  <= c_keep;
        end

        // ---- credit counters ----
        for (int c = 0; c < 3; c++) begin
          used_h[c] <= used_h[c] + chg_h[c] - cons_h[c];
          used_d[c] <= used_d[c] + chg_d[c] - cons_d[c];
`ifdef MUT_17
          if (ADV_H(c) != 0) ca_h[c] <= ca_h[c];
`else
          if (ADV_H(c) != 0) ca_h[c] <= ca_h[c] + rel_h[c];
`endif
`ifdef MUT_17
          if (ADV_D(c) != 0) ca_d[c] <= ca_d[c];
`else
          if (ADV_D(c) != 0) ca_d[c] <= ca_d[c] + rel_d[c];
`endif
        end
      end
    end
  end

  assign rx_idle = !dl_rx_valid && !in_tlp && npb_empty && !c1_valid;
endmodule
