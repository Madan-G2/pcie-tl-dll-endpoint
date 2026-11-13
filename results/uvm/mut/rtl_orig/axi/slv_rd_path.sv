// ============================================================================
// slv_rd_path : AXI4 slave read channels -> MRd / CfgRd requests (AX-1, AX-4,
//               AX-5, AX-7, AX-8, AX-9, AX-14) and rd_return (R channel,
//               OP4-1..OP4-4).
//   burst table (16 entries) in acceptance order, tags from rd_tag_table.
// ============================================================================
module slv_rd_path
  import pcie_pkg::*;
#(
  parameter logic [63:0] ECAM_BASE = 64'h0000_0000_F000_0000,
  parameter int MAX_TAGS = 16
) (
  input  logic         clk,
  input  logic         rst_n,
  input  logic         dl_up,
  input  logic [2:0]   mrrs_code,
  input  logic         rd_ooo_en,
  // AXI slave read channels
  input  logic [3:0]   s_arid,
  input  logic [63:0]  s_araddr,
  input  logic [7:0]   s_arlen,
  input  logic [2:0]   s_arsize,
  input  logic [1:0]   s_arburst,
  input  logic         s_arvalid,
  output logic         s_arready,
  output logic [3:0]   s_rid,
  output logic [127:0] s_rdata,
  output logic [1:0]   s_rresp,
  output logic         s_rlast,
  output logic         s_rvalid,
  input  logic         s_rready,
  // request to the NP mux
  output logic         rq_valid,
  output logic         rq_is_cfg,
  output logic [63:0]  rq_addr,
  output logic [7:0]   rq_len_dw,
  input  logic         rq_gnt,             // TLP pushed and tag allocated
  // tag table
  input  logic         al_free,
  input  logic         al_free_mem,
  output logic [1:0]   al_kind,
  output logic [3:0]   al_burst,
  output logic [5:0]   al_sub,
  output logic [11:0]  al_addr,
  output logic [9:0]   al_len,
  input  logic [$clog2(MAX_TAGS)-1:0] al_tag,
  input  logic [MAX_TAGS-1:0] t_valid, t_done, t_err,
  input  logic [MAX_TAGS-1:0][3:0] t_burst,
  input  logic [MAX_TAGS-1:0][5:0] t_sub,
  input  logic [15:0]  now,
  output logic         fr_en,
  output logic [$clog2(MAX_TAGS)-1:0] fr_tag,
  output logic         cb_re,
  output logic [$clog2(MAX_TAGS)+4:0] cb_raddr,
  input  logic [127:0] cb_rdata,
  // write ordering barrier
  input  logic [7:0]   aw_done_cnt,
  input  logic [7:0]   aw_acc_cnt,
  // events
  output logic         ev_axi_unsup,
  output logic         ev_rd_issued,
  output logic         ev_lat_valid,
  output logic [31:0]  ev_lat_val,
  output logic         ev_hol_stall,
  output logic         busy
);
  localparam int BN = 16;
  localparam int TW = $clog2(MAX_TAGS);

  // ---------------------------------------------------------------------------
  // burst table
  // ---------------------------------------------------------------------------
  logic [BN-1:0] b_valid, b_abort, b_seen;
  logic [3:0]    b_id    [0:BN-1];
  logic [63:0]   b_addr  [0:BN-1];
  logic [2:0]    b_size  [0:BN-1];
  logic [8:0]    b_beats [0:BN-1];
  logic [1:0]    b_kind  [0:BN-1];      // 0 mem, 1 cfg, 2 unsupported
  logic [1:0]    b_mrrs  [0:BN-1];
  logic [6:0]    b_nmrd  [0:BN-1];
  logic [6:0]    b_iss   [0:BN-1];
  logic [7:0]    b_snap  [0:BN-1];
  logic [15:0]   b_acc   [0:BN-1];
  logic [15:0]   b_dcyc  [0:BN-1];
  logic [7:0]    b_seq   [0:BN-1];
  logic [7:0]    seq_ctr;
  logic [3:0]    b_wp, b_ip;            // write pointer, issue pointer

  // ---- AR intake (AX-1, AX-14) ------------------------------------------------
  assign s_arready = dl_up && !b_valid[b_wp];
  wire        a_incr  = (s_arburst == 2'b01);
  wire        a_szok  = (s_arsize == 3'd2) || (s_arsize == 3'd3) || (s_arsize == 3'd4);
  wire        a_algn  = ((s_araddr & ((64'd1 << s_arsize) - 64'd1)) == 64'd0);
  wire [12:0] a_bytes = ({5'b0, s_arlen} + 13'd1) << s_arsize;
  wire        a_noxng = ({1'b0, s_araddr[11:0]} + a_bytes) <= 13'd4096;
  wire        a_ecam  = (s_araddr >= ECAM_BASE) && (s_araddr < ECAM_BASE + 64'h1000_0000);
  logic [1:0] a_kind;
  always_comb begin
    if (a_ecam) a_kind = (a_incr && a_algn && s_arlen == 8'd0 && s_arsize == 3'd2) ? 2'd1 : 2'd2;
    else        a_kind = (a_incr && a_szok && a_algn && a_noxng) ? 2'd0 : 2'd2;
  end
  wire ar_hs = s_arvalid && s_arready;
  assign ev_axi_unsup = ar_hs && (a_kind == 2'd2);

  // number of MRd TLPs for the new burst
  logic [11:0] a_end12;
  logic [11:0] a_blk0, a_blk1;
  logic [6:0]  a_nmrd;
  always_comb begin
    a_end12 = s_araddr[11:0] + a_bytes[11:0] - 12'd1;                 // last byte
    a_blk0  = s_araddr[11:0] >> (7 + mrrs_code);
    a_blk1  = a_end12 >> (7 + mrrs_code);
    a_nmrd  = (a_kind == 2'd0) ? (a_blk1[6:0] - a_blk0[6:0] + 7'd1) : (a_kind == 2'd1 ? 7'd1 : 7'd0);
  end

  // ---------------------------------------------------------------------------
  // issuer : MRd / CfgRd generation
  // ---------------------------------------------------------------------------
  wire        ip_valid = b_valid[b_ip] && (b_kind[b_ip] != 2'd2) && !b_abort[b_ip] && (b_iss[b_ip] < b_nmrd[b_ip]);
  wire [7:0]  snap_d   = aw_done_cnt - b_snap[b_ip];
  wire        wr_ok    = !snap_d[7];
  wire [1:0]  ip_mrrs  = b_mrrs[b_ip];
  wire [11:0] ip_a12   = b_addr[b_ip][11:0];
  wire [11:0] ip_bytes = {3'b0, b_beats[b_ip]} << b_size[b_ip];
  wire [11:0] ip_end   = ip_a12 + ip_bytes;                         // exclusive, <= 4096 (fits in 13 bits; 12-bit wrap ok if 0)
  wire [12:0] ip_end13 = {1'b0, ip_a12} + {1'b0, ip_bytes};
  wire [11:0] blk_base = (ip_a12 >> (7 + ip_mrrs)) << (7 + ip_mrrs);
  wire [12:0] blk_sub  = {1'b0, blk_base} + (13'(b_iss[b_ip]) << (7 + ip_mrrs));
  wire [12:0] blk_next = blk_sub + (13'd128 << ip_mrrs);
  wire [12:0] ck_s13   = (b_iss[b_ip] == 7'd0) ? {1'b0, ip_a12} : blk_sub;
  wire [12:0] ck_e13   = (ip_end13 < blk_next) ? ip_end13 : blk_next;
  wire [11:0] ck_bytes = ck_e13[11:0] - ck_s13[11:0];
  wire        tag_ok   = (b_kind[b_ip] == 2'd0) ? al_free_mem : al_free;

  assign rq_valid  = dl_up && ip_valid && wr_ok && tag_ok;
  assign rq_is_cfg = (b_kind[b_ip] == 2'd1);
  assign rq_addr   = {b_addr[b_ip][63:12], ck_s13[11:0]};
  assign rq_len_dw = (b_kind[b_ip] == 2'd1) ? 8'd1 : 8'(ck_bytes >> 2);
  assign al_kind   = rq_is_cfg ? 2'd1 : 2'd0;
  assign al_burst  = b_ip;
  assign al_sub    = b_iss[b_ip][5:0];
  assign al_addr   = ck_s13[11:0];
  assign al_len    = rq_is_cfg ? 10'd4 : ck_bytes[9:0];
  assign ev_rd_issued = rq_gnt && !rq_is_cfg;

  // ---------------------------------------------------------------------------
  // rd_return : candidate selection
  // ---------------------------------------------------------------------------
  logic [BN-1:0] rdy_c, rdy;           // rdy_c : combinational, rdy : registered copy (timing)
  logic [3:0]    n_done [0:BN-1];
  logic [BN-1:0] big;
  always_comb begin
    for (int b = 0; b < BN; b++) begin
      logic [6:0] nd;
      nd = '0;
      for (int t = 0; t < MAX_TAGS; t++)
        if (t_valid[t] && t_done[t] && t_burst[t] == b[3:0]) nd = nd + 7'd1;
      big[b] = (b_nmrd[b] > MAX_TAGS[6:0]);
      rdy_c[b] = 1'b0;
      if (b_valid[b]) begin
        if (b_kind[b] == 2'd2 || b_abort[b]) rdy_c[b] = 1'b1;
        else if (big[b]) begin
          for (int t = 0; t < MAX_TAGS; t++)
            if (t_valid[t] && t_done[t] && t_burst[t] == b[3:0] && t_sub[t] == 6'd0) rdy_c[b] = 1'b1;
        end
        else rdy_c[b] = (b_iss[b] == b_nmrd[b]) && (nd == b_nmrd[b]);
      end
      n_done[b] = nd[3:0];
    end
  end

  logic [BN-1:0] elig_c, elig, blk_inc;
  always_comb begin
    for (int b = 0; b < BN; b++) begin
      logic older_same, older_any, older_inc_same, older_inc_any;
      older_same = 1'b0; older_any = 1'b0; older_inc_same = 1'b0; older_inc_any = 1'b0;
      for (int o = 0; o < BN; o++) begin
        logic [7:0] dsq;
        dsq = b_seq[b] - b_seq[o];                    // o is older than b iff 1..127
        if (b_valid[o] && o != b && dsq != 8'd0 && !dsq[7]) begin
          older_any = 1'b1;
          if (!rdy[o]) older_inc_any = 1'b1;
          if (b_id[o] == b_id[b]) begin
            older_same = 1'b1;
            if (!rdy[o]) older_inc_same = 1'b1;
          end
        end
      end
      elig_c[b]  = b_valid[b] && rdy[b] && (rd_ooo_en ? !older_same : !older_any);
      blk_inc[b] = b_valid[b] && rdy[b] && !elig_c[b] && (rd_ooo_en ? older_inc_same : older_inc_any);
    end
  end
  assign ev_hol_stall = |blk_inc;

  // pipeline: rdy_c -> rdy (1) ; rdy -> elig (2) ; elig,age -> sel (3).  A selection
  // is re-validated at load time (b_valid && rdy), which covers slot reuse after a burst
  // has been served (rdy never falls for a live burst).
  logic [15:0] age_q [0:BN-1];
  logic [3:0]  sel_q; logic selv_q;
  logic [3:0]  sel;   logic sel_v;
  always_comb begin
    logic        tv [0:2*BN-2];
    logic [15:0] ta [0:2*BN-2];
    logic [3:0]  ti [0:2*BN-2];
    for (int b = 0; b < BN; b++) begin tv[b] = elig[b]; ta[b] = age_q[b]; ti[b] = b[3:0]; end
    for (int lvl = 0; lvl < 4; lvl++) begin
      for (int k = 0; k < (BN >> (lvl+1)); k++) begin
        int l, r, d;
        logic [3:0] il, ir;
        l = (2*BN - (2*BN >> lvl)) + 2*k;          // level base + 2k
        r = l + 1;
        d = (2*BN - (2*BN >> (lvl+1))) + k;        // next level base + k
        il = ti[l]; ir = ti[r];
        if (!tv[l] || (tv[r] && (ta[r] > ta[l] || (ta[r] == ta[l] && b_id[ir] < b_id[il])))) begin
          tv[d] = tv[r]; ta[d] = ta[r]; ti[d] = ir;
        end else begin
          tv[d] = tv[l]; ta[d] = ta[l]; ti[d] = il;
        end
      end
    end
    sel_v = tv[2*BN-2]; sel = ti[2*BN-2];
  end
  wire sel_ok = selv_q && b_valid[sel_q] && rdy[sel_q];

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      rdy <= '0; elig <= '0; sel_q <= '0; selv_q <= 1'b0;
      for (int b = 0; b < BN; b++) age_q[b] <= '0;
    end else begin
      rdy  <= rdy_c;
      elig <= elig_c;
      for (int b = 0; b < BN; b++) age_q[b] <= now - b_dcyc[b];
      sel_q <= sel; selv_q <= sel_v;
    end
  end

  // ---------------------------------------------------------------------------
  // rd_return : streaming FSM
  // ---------------------------------------------------------------------------
  logic        rs_busy;
  logic [3:0]  rs_b;
  logic [8:0]  rs_i;                  // next beat to issue
  logic [11:0] rs_a12;                // address of next beat to issue
  logic [11:0] rs_abase;              // burst start (low 12)
  logic [2:0]  rs_size;
  logic [8:0]  rs_beats;
  logic [1:0]  rs_kind, rs_mrrs;
  logic [3:0]  rs_id;
  logic [15:0] rs_acc;
  logic        rs_abort;

  // output register
  logic        r_v, r_last, r_cfg;
  logic [1:0]  r_resp;
  logic [3:0]  r_id;
  logic [15:0] r_mask;
  logic [15:0] r_acc;
  logic        r_lat_en;

  // per-beat sub-read index and tag lookup
  wire [11:0] nx_a12   = rs_a12 + (12'd1 << rs_size);
  wire [4:0]  blk_cur  = 5'(rs_a12 >> (7 + rs_mrrs)) - 5'(rs_abase >> (7 + rs_mrrs));
  wire [4:0]  blk_nxt  = 5'(nx_a12 >> (7 + rs_mrrs)) - 5'(rs_abase >> (7 + rs_mrrs));
  logic       f_found; logic [TW-1:0] f_tag;
  always_comb begin
    f_found = 1'b0; f_tag = '0;
    for (int t = 0; t < MAX_TAGS; t++)
      if (t_valid[t] && t_burst[t] == rs_b && t_sub[t] == {1'b0, blk_cur}) begin f_found = 1'b1; f_tag = t[TW-1:0]; end
  end
  wire last_beat   = (rs_i == rs_beats - 9'd1);
  wire chunk_last  = last_beat || (blk_nxt != blk_cur);
  wire is_err_beat = (rs_kind == 2'd2) || rs_abort || (!f_found && b_abort[rs_b]) || (f_found && t_err[f_tag]);
  wire tag_ready  = (rs_kind == 2'd2) || rs_abort || b_abort[rs_b] || (f_found && t_done[f_tag]);
  wire can_issue  = rs_busy && tag_ready && (!r_v || s_rready);
  wire issue_read = can_issue && !is_err_beat;

  assign cb_re    = issue_read;
  assign cb_raddr = {f_tag, rs_a12[8:4]};
  assign fr_en    = can_issue && chunk_last && f_found;
  assign fr_tag   = f_tag;

  // R channel outputs
  logic [15:0] win_m;
  always_comb win_m = ((16'd1 << (16'd1 << rs_size)) - 16'd1) << rs_a12[3:0];
  assign s_rvalid = r_v;
  assign s_rid    = r_id;
  assign s_rlast  = r_last;
  assign s_rresp  = r_resp;
  logic [127:0] rd_sel;
  always_comb begin
    rd_sel = cb_rdata;                       // payload is stored at lane addr[3:2] (AX-8)
    for (int b = 0; b < 16; b++) s_rdata[8*b +: 8] = r_mask[b] ? rd_sel[8*b +: 8] : 8'h00;
  end

  assign ev_lat_valid = r_v && s_rready && r_last;
  assign ev_lat_val   = {16'b0, now - r_acc};

  assign busy = (|b_valid) || r_v;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      b_valid <= '0; b_abort <= '0; b_seen <= '0; b_wp <= '0; b_ip <= '0; seq_ctr <= '0;
      for (int i = 0; i < BN; i++) begin
        b_id[i] <= '0; b_addr[i] <= '0; b_size[i] <= '0; b_beats[i] <= '0; b_kind[i] <= '0; b_mrrs[i] <= '0;
        b_nmrd[i] <= '0; b_iss[i] <= '0; b_snap[i] <= '0; b_acc[i] <= '0; b_dcyc[i] <= '0; b_seq[i] <= '0;
      end
      rs_busy <= 1'b0; rs_b <= '0; rs_i <= '0; rs_a12 <= '0; rs_abase <= '0; rs_size <= '0; rs_beats <= '0;
      rs_kind <= '0; rs_mrrs <= '0; rs_id <= '0; rs_acc <= '0; rs_abort <= 1'b0;
      r_v <= 1'b0; r_last <= 1'b0; r_cfg <= 1'b0; r_resp <= '0; r_id <= '0; r_mask <= '0; r_acc <= '0; r_lat_en <= 1'b0;
    end else begin
      // ---- AR accept ----
      if (ar_hs) begin
        b_valid[b_wp] <= 1'b1; b_abort[b_wp] <= 1'b0; b_seen[b_wp] <= 1'b0;
        b_id[b_wp] <= s_arid; b_addr[b_wp] <= s_araddr; b_size[b_wp] <= s_arsize; b_beats[b_wp] <= {1'b0, s_arlen} + 9'd1;
        b_kind[b_wp] <= a_kind; b_mrrs[b_wp] <= mrrs_code[1:0]; b_nmrd[b_wp] <= a_nmrd; b_iss[b_wp] <= '0;
        b_snap[b_wp] <= aw_acc_cnt; b_acc[b_wp] <= now;
        b_seq[b_wp] <= seq_ctr; seq_ctr <= seq_ctr + 8'd1;
        b_wp <= b_wp + 1'b1;
      end
      // ---- MRd issue ----
      if (rq_gnt) begin
        b_iss[b_ip] <= b_iss[b_ip] + 7'd1;
      end
      // advance the issue pointer past finished / unissuable bursts
      if (b_valid[b_ip] && ((b_iss[b_ip] >= b_nmrd[b_ip]) || b_abort[b_ip] || b_kind[b_ip] == 2'd2) && b_ip != b_wp)
        b_ip <= b_ip + 1'b1;
      else if (!b_valid[b_ip] && b_ip != b_wp) b_ip <= b_ip + 1'b1;

      // ---- completion time stamp (OP4-2) ----
      for (int b = 0; b < BN; b++)
        if (b_valid[b] && rdy[b] && !b_seen[b]) begin b_seen[b] <= 1'b1; b_dcyc[b] <= now; end

      // ---- link down : abort all unfinished bursts ----
      if (!dl_up) for (int b = 0; b < BN; b++) if (b_valid[b]) b_abort[b] <= 1'b1;

      // ---- R output register ----
      if (r_v && s_rready) r_v <= 1'b0;
      if (can_issue) begin
        r_v    <= 1'b1;
        r_last <= last_beat;
        r_id   <= rs_id;
        r_cfg  <= (rs_kind == 2'd1);
        r_resp <= is_err_beat ? 2'b10 : 2'b00;
        r_mask <= is_err_beat ? 16'h0000 : win_m;
        r_acc  <= rs_acc;
        rs_i   <= rs_i + 9'd1;
        rs_a12 <= nx_a12;
        if (last_beat) begin
          rs_busy <= 1'b0;
          b_valid[rs_b] <= 1'b0;
        end
      end
      // zero data mask for error beats is applied through r_mask
      if (!rs_busy && sel_ok) begin
        rs_busy <= 1'b1; rs_b <= sel_q; rs_i <= '0;
        rs_a12 <= b_addr[sel_q][11:0]; rs_abase <= b_addr[sel_q][11:0];
        rs_size <= b_size[sel_q]; rs_beats <= b_beats[sel_q]; rs_kind <= b_kind[sel_q]; rs_mrrs <= b_mrrs[sel_q];
        rs_id <= b_id[sel_q]; rs_acc <= b_acc[sel_q]; rs_abort <= b_abort[sel_q];
      end
    end
  end
endmodule
