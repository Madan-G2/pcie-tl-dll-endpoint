// ============================================================================
// tl_tx : three TX queues (P, NP, Cpl), ordering + credit arbiter, TLP stream
//         to the DLL (4.4: TL-7 .. TL-11).  Class index: 0 = P, 1 = NP, 2 = Cpl.
//
//  * Producers push whole beats; a TLP becomes eligible only when its eop
//    beat is queued (IF-1).
//  * Age = value of a shared 16-bit enqueue counter.  P TLPs get their age
//    from the write coalescer (age_now / age_take handshake, taken when the
//    TLP is "opened"); NP / Cpl TLPs take it when their first beat is pushed.
//  * Two Cpl producers (cpl0 = read completions, cpl1 = UR / config
//    completions) are merged into the Cpl queue at TLP boundaries.
// ============================================================================
module tl_tx
  import pcie_pkg::*;
#(
  parameter int P_DEPTH   = 64,    // beats (>= 17 for a 256 B MWr)
  parameter int NP_DEPTH  = 16,
  parameter int CPL_DEPTH = 64
) (
  input  logic          clk,
  input  logic          rst_n,
  input  logic          dl_up,
  // posted producer (wr_coalescer)
  input  logic          p_valid,
  output logic          p_ready,
  input  logic [127:0]  p_data,
  input  logic [3:0]    p_keep,
  input  logic          p_sop,
  input  logic          p_eop,
  input  logic [15:0]   p_age,
  output logic [15:0]   age_now,
  input  logic          age_take,
  input  logic          open_valid,
  input  logic [15:0]   open_age,
  output logic          coal_flush,
  // non-posted producer (single-beat TLPs)
  input  logic          np_valid,
  output logic          np_ready,
  input  logic [127:0]  np_data,
  input  logic [3:0]    np_keep,
  // completion producers
  input  logic          cpl0_valid,
  output logic          cpl0_ready,
  input  logic [127:0]  cpl0_data,
  input  logic [3:0]    cpl0_keep,
  input  logic          cpl0_sop,
  input  logic          cpl0_eop,
  input  logic          cpl1_valid,
  output logic          cpl1_ready,
  input  logic [127:0]  cpl1_data,
  input  logic [3:0]    cpl1_keep,
  input  logic          cpl1_sop,
  input  logic          cpl1_eop,
  // to DLL
  output logic          tl_tx_valid,
  input  logic          tl_tx_ready,
  output logic [127:0]  tl_tx_data,
  output logic [3:0]    tl_tx_keep,
  output logic          tl_tx_sop,
  output logic          tl_tx_eop,
  // flow-control limits from the DLL
  input  logic          fc_valid,
  input  logic [7:0]    fc_lim_p_hdr,  input logic [11:0] fc_lim_p_data,
  input  logic [7:0]    fc_lim_np_hdr, input logic [11:0] fc_lim_np_data,
  input  logic [7:0]    fc_lim_cpl_hdr, input logic [11:0] fc_lim_cpl_data,
  input  logic          fc_inf_p_hdr,  input logic fc_inf_p_data,
  input  logic          fc_inf_np_hdr, input logic fc_inf_np_data,
  input  logic          fc_inf_cpl_hdr, input logic fc_inf_cpl_data,
  // status
  output logic          ev_stall_credit,
  output logic          tx_idle,
  output logic          np_pending,
  output logic          dbg_start,
  output logic [7:0]    dbg_cc_p_hdr,
  output logic [11:0]   dbg_cc_p_data
);
  wire flush = !dl_up;

  // ---------------------------------------------------------------------------
  // enqueue counter / age allocation
  // ---------------------------------------------------------------------------
  logic [15:0] enq_ctr;
  logic        np_push, cpl_sop_push;
  logic [15:0] np_age, cpl_age_new, cpl_age_r;
  logic        cpl_in_pkt;
  assign age_now = enq_ctr;
  assign np_age  = enq_ctr + 16'(age_take);

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n)     enq_ctr <= '0;
    else if (flush) enq_ctr <= '0;
    else enq_ctr <= enq_ctr + 16'(age_take) + 16'(np_push) + 16'(cpl_sop_push);
  end

  // ---------------------------------------------------------------------------
  // queues
  // ---------------------------------------------------------------------------
  logic [2:0]        q_space, q_empty, q_eopf, q_sopf, q_pop;
  logic [2:0][127:0] q_data;
  logic [2:0][3:0]   q_keep;
  logic [2:0][15:0]  q_age;
  logic [2:0]        q_pkt;
  logic [$clog2(P_DEPTH):0]   p_cnt;
  logic [$clog2(NP_DEPTH):0]  np_cnt;
  logic [$clog2(CPL_DEPTH):0] cpl_cnt;

  // completion producer merge
  logic       cpl_sel_lock, cpl_sel_r;
  logic       cpl_sel;                 // 0 = cpl0, 1 = cpl1
  logic       cq_push;
  logic [127:0] cq_data; logic [3:0] cq_keep; logic cq_sop, cq_eop;
  always_comb begin
    cpl_sel = cpl_sel_lock ? cpl_sel_r : (cpl1_valid ? 1'b1 : 1'b0);
    cq_push = 1'b0; cq_data = cpl0_data; cq_keep = cpl0_keep; cq_sop = cpl0_sop; cq_eop = cpl0_eop;
    cpl0_ready = 1'b0; cpl1_ready = 1'b0;
    if (cpl_sel) begin
      cq_data = cpl1_data; cq_keep = cpl1_keep; cq_sop = cpl1_sop; cq_eop = cpl1_eop;
      cq_push = cpl1_valid && q_space[2]; cpl1_ready = q_space[2];
    end else begin
      cq_push = cpl0_valid && q_space[2]; cpl0_ready = q_space[2];
    end
  end
  assign cpl_sop_push = cq_push && cq_sop;
  assign cpl_age_new  = enq_ctr + 16'(age_take) + 16'(np_push);
  wire [15:0] cq_age  = cq_sop ? cpl_age_new : cpl_age_r;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      cpl_sel_lock <= 1'b0; cpl_sel_r <= 1'b0; cpl_age_r <= '0;
    end else if (flush) begin
      cpl_sel_lock <= 1'b0; cpl_sel_r <= 1'b0; cpl_age_r <= '0;
    end else if (cq_push) begin
      cpl_age_r <= cq_age;
      if (cq_eop) cpl_sel_lock <= 1'b0;
      else begin cpl_sel_lock <= 1'b1; cpl_sel_r <= cpl_sel; end
    end
  end

  assign p_ready  = q_space[0];
  assign np_ready = q_space[1];
  assign np_push  = np_valid && q_space[1];

  tl_queue #(.DEPTH(P_DEPTH)) u_qp (
    .clk, .rst_n, .flush, .push(p_valid && q_space[0]), .push_data(p_data), .push_keep(p_keep),
    .push_sop(p_sop), .push_eop(p_eop), .push_age(p_age), .space(q_space[0]),
    .head_data(q_data[0]), .head_keep(q_keep[0]), .head_sop(q_sopf[0]), .head_eop(q_eopf[0]),
    .head_age(q_age[0]), .pop(q_pop[0]), .tlp_cnt(p_cnt), .empty(q_empty[0]));
  tl_queue #(.DEPTH(NP_DEPTH)) u_qnp (
    .clk, .rst_n, .flush, .push(np_push), .push_data(np_data), .push_keep(np_keep),
    .push_sop(1'b1), .push_eop(1'b1), .push_age(np_age), .space(q_space[1]),
    .head_data(q_data[1]), .head_keep(q_keep[1]), .head_sop(q_sopf[1]), .head_eop(q_eopf[1]),
    .head_age(q_age[1]), .pop(q_pop[1]), .tlp_cnt(np_cnt), .empty(q_empty[1]));
  tl_queue #(.DEPTH(CPL_DEPTH)) u_qcpl (
    .clk, .rst_n, .flush, .push(cq_push), .push_data(cq_data), .push_keep(cq_keep),
    .push_sop(cq_sop), .push_eop(cq_eop), .push_age(cq_age), .space(q_space[2]),
    .head_data(q_data[2]), .head_keep(q_keep[2]), .head_sop(q_sopf[2]), .head_eop(q_eopf[2]),
    .head_age(q_age[2]), .pop(q_pop[2]), .tlp_cnt(cpl_cnt), .empty(q_empty[2]));

  assign q_pkt = {(cpl_cnt != '0), (np_cnt != '0), (p_cnt != '0)};
  assign np_pending = !q_empty[1];

  // ---------------------------------------------------------------------------
  // credit state (TL-10)
  // ---------------------------------------------------------------------------
  logic [2:0][7:0]  lim_h;
  logic [2:0][11:0] lim_d;
  logic [2:0]       inf_h, inf_d;
  assign lim_h = {fc_lim_cpl_hdr, fc_lim_np_hdr, fc_lim_p_hdr};
  assign lim_d = {fc_lim_cpl_data, fc_lim_np_data, fc_lim_p_data};
  assign inf_h = {fc_inf_cpl_hdr, fc_inf_np_hdr, fc_inf_p_hdr};
  assign inf_d = {fc_inf_cpl_data, fc_inf_np_data, fc_inf_p_data};

  logic [2:0][7:0]  cc_h;
  logic [2:0][11:0] cc_d;
  logic [2:0][11:0] need_d;
  logic [2:0]       cr_ok, ord_ok, elig, blk_cr;
  logic             p_exists;
  logic [15:0]      p_min_age;

  always_comb begin
    p_exists  = q_pkt[0] || open_valid;
    p_min_age = q_age[0];
    if (!q_pkt[0]) p_min_age = open_age;
    else if (open_valid && age_lt(open_age, q_age[0])) p_min_age = open_age;
    for (int c = 0; c < 3; c++) begin
      logic [9:0] len;
      len = {q_data[c][17:16], q_data[c][31:24]};
`ifdef MUT_19
      need_d[c] = q_data[c][6] ? 12'(len >> 2) : 12'd0;     // Fmt[1] = has data
`else
      need_d[c] = q_data[c][6] ? 12'((len + 10'd3) >> 2) : 12'd0;     // Fmt[1] = has data
`endif
`ifdef MUT_16
      cr_ok[c]  = (inf_h[c] || (((9'(lim_h[c]) - (9'(cc_h[c]) + 9'd1)) & 9'h1FF) <= 9'd128)) &&
`else
      cr_ok[c]  = (inf_h[c] || cr_ok8 (lim_h[c], cc_h[c], 8'd1)) &&
`endif
                  (inf_d[c] || cr_ok12(lim_d[c], cc_d[c], need_d[c]));
`ifdef MUT_24
      ord_ok[c] = 1'b1;
`else
      ord_ok[c] = (c == 0) ? 1'b1 : !(p_exists && age_lt(p_min_age, q_age[c]));
`endif
      elig[c]   = q_pkt[c] && fc_valid && dl_up && cr_ok[c] && ord_ok[c];
      blk_cr[c] = q_pkt[c] && fc_valid && dl_up && !cr_ok[c] && ord_ok[c];
    end
  end

  // WC-3 : an NP / Cpl TLP is queued behind the open posted TLP
  assign coal_flush = open_valid && ((q_pkt[1] && age_lt(open_age, q_age[1])) ||
                                     (q_pkt[2] && age_lt(open_age, q_age[2])));

  // ---------------------------------------------------------------------------
  // arbiter (TL-9): smallest age among eligible heads
  // ---------------------------------------------------------------------------
  logic [1:0] pick;
  logic       pick_v;
  always_comb begin
    pick_v = 1'b0; pick = 2'd0;
    for (int c = 0; c < 3; c++) begin
      if (elig[c] && (!pick_v || age_lt(q_age[c], q_age[pick]))) begin
        pick = 2'(c); pick_v = 1'b1;
      end
    end
  end

  logic       streaming, locked;
  logic [1:0] str_sel, lock_sel, cur_sel;
  logic       cur_v;
  always_comb begin
    if (streaming)   begin cur_sel = str_sel;  cur_v = 1'b1; end
    else if (locked) begin cur_sel = lock_sel; cur_v = 1'b1; end
    else             begin cur_sel = pick;     cur_v = pick_v; end
  end

  assign tl_tx_valid = cur_v && dl_up;
  assign tl_tx_data  = q_data[cur_sel];
  assign tl_tx_keep  = q_keep[cur_sel];
  assign tl_tx_eop   = q_eopf[cur_sel];
  assign tl_tx_sop   = !streaming;

  wire acc = tl_tx_valid && tl_tx_ready;
  always_comb begin
    q_pop = '0;
    if (acc) q_pop[cur_sel] = 1'b1;
  end

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      streaming <= 1'b0; locked <= 1'b0; str_sel <= '0; lock_sel <= '0;
    end else if (flush) begin
      streaming <= 1'b0; locked <= 1'b0; str_sel <= '0; lock_sel <= '0;
    end else begin
      if (tl_tx_valid && !tl_tx_ready && !streaming) begin locked <= 1'b1; lock_sel <= cur_sel; end
      if (acc) begin
        locked <= 1'b0;
        if (!streaming && !tl_tx_eop) begin streaming <= 1'b1; str_sel <= cur_sel; end
        if (streaming && tl_tx_eop)   streaming <= 1'b0;
      end
    end
  end

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      cc_h <= '0; cc_d <= '0;
    end else if (flush) begin
      cc_h <= '0; cc_d <= '0;
    end else if (acc && !streaming) begin
      cc_h[cur_sel] <= cc_h[cur_sel] + 8'd1;
      cc_d[cur_sel] <= cc_d[cur_sel] + need_d[cur_sel];
    end
  end

  assign ev_stall_credit = !streaming && !(|elig) && (|blk_cr);     // TL-11
  assign tx_idle         = (&q_empty) && !streaming;
  assign dbg_start       = acc && !streaming;
  assign dbg_cc_p_hdr    = cc_h[0];
  assign dbg_cc_p_data   = cc_d[0];
endmodule
