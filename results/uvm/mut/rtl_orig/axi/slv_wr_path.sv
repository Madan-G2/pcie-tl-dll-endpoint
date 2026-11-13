// ============================================================================
// slv_wr_path : AXI4 slave write channels -> MWr chunks (AX-1..AX-3, AX-6,
//               AX-9, AX-14 config writes)
//   * AW FIFO with legality check (unsupported -> SLVERR, no TLP)
//   * W beats are collected into a 256-byte chunk buffer addressed by
//     address[7:2]; a chunk ends at an MPS boundary or at wlast
//   * strobe analysis (first/last DW BE, sparse -> unsupported)
//   * B responses are queued in order; config writes wait for their Cpl
// ============================================================================
module slv_wr_path
  import pcie_pkg::*;
#(
  parameter logic [63:0] ECAM_BASE = 64'h0000_0000_F000_0000,
  parameter int MAX_TAGS = 16
) (
  input  logic         clk,
  input  logic         rst_n,
  input  logic         dl_up,
  input  logic [2:0]   mps_code,
  // AXI slave write channels
  input  logic [3:0]   s_awid,
  input  logic [63:0]  s_awaddr,
  input  logic [7:0]   s_awlen,
  input  logic [2:0]   s_awsize,
  input  logic [1:0]   s_awburst,
  input  logic         s_awvalid,
  output logic         s_awready,
  input  logic [127:0] s_wdata,
  input  logic [15:0]  s_wstrb,
  input  logic         s_wlast,
  input  logic         s_wvalid,
  output logic         s_wready,
  output logic [3:0]   s_bid,
  output logic [1:0]   s_bresp,
  output logic         s_bvalid,
  input  logic         s_bready,
  // chunk -> coalescer
  output logic         ck_valid,
  input  logic         ck_ready,
  output logic         ck_hdr4,
  output logic [63:0]  ck_addr,
  output logic [6:0]   ck_len,
  output logic [3:0]   ck_fbe,
  output logic [3:0]   ck_lbe,
  output logic [63:0][31:0] ck_ring,
  output logic [63:0]  ck_mask,
  // config write request (NP) -> bridge NP mux
  output logic         cfgw_req,
  input  logic         cfgw_gnt,
  input  logic [$clog2(MAX_TAGS)-1:0] cfgw_tag,
  output logic [25:0]  cfgw_addr,
  output logic [31:0]  cfgw_data,
  output logic [3:0]   cfgw_be,
  // tag table (config write completion)
  input  logic [MAX_TAGS-1:0] t_done,
  input  logic [MAX_TAGS-1:0] t_err,
  output logic         fr_en,
  output logic [$clog2(MAX_TAGS)-1:0] fr_tag,
  // ordering barrier against reads
  output logic [7:0]   aw_acc_cnt,
  output logic [7:0]   aw_done_cnt,
  output logic         ev_axi_unsup,
  output logic         busy
);
  localparam int TW = $clog2(MAX_TAGS);

  // ---------------------------------------------------------------------------
  // AW intake + legality (AX-1, AX-14)
  // ---------------------------------------------------------------------------
  typedef struct packed {
    logic [3:0]  id;
    logic [63:0] addr;
    logic [7:0]  len;
    logic [2:0]  size;
    logic [1:0]  kind;           // 0 mem, 1 cfg, 2 unsupported
  } aw_t;
  aw_t aw_q [0:3];
  logic [2:0] aw_wp, aw_rp;
  wire aw_full  = ((aw_wp - aw_rp) >= 3'd4);
  wire aw_empty = (aw_wp == aw_rp);
  assign s_awready = dl_up && !aw_full;

  wire        b_incr  = (s_awburst == 2'b01);
  wire        sz_ok   = (s_awsize == 3'd2) || (s_awsize == 3'd3) || (s_awsize == 3'd4);
  wire        aligned = ((s_awaddr & ((64'd1 << s_awsize) - 64'd1)) == 64'd0);
  wire [12:0] bytes_aw = ({5'b0, s_awlen} + 13'd1) << s_awsize;
  wire        nocross = ({1'b0, s_awaddr[11:0]} + bytes_aw) <= 13'd4096;
  wire        is_ecam = (s_awaddr >= ECAM_BASE) && (s_awaddr < ECAM_BASE + 64'h1000_0000);
  logic [1:0] aw_kind;
  always_comb begin
    if (is_ecam) aw_kind = (b_incr && aligned && s_awlen == 8'd0 && s_awsize == 3'd2) ? 2'd1 : 2'd2;
    else         aw_kind = (b_incr && sz_ok && aligned && nocross) ? 2'd0 : 2'd2;
  end
  wire aw_hs = s_awvalid && s_awready;
  logic ev_sparse;
  assign ev_axi_unsup = (aw_hs && (aw_kind == 2'd2)) || ev_sparse;

  // ---------------------------------------------------------------------------
  // B queue
  // ---------------------------------------------------------------------------
  logic [3:0]    bq_id   [0:7];
  logic [1:0]    bq_resp [0:7];
  logic          bq_cfg  [0:7];
  logic [TW-1:0] bq_tag  [0:7];
  logic [3:0]    bq_wp, bq_rp;
  wire bq_full  = ((bq_wp - bq_rp) >= 4'd8);
  wire bq_empty = (bq_wp == bq_rp);
  wire [2:0] bh = bq_rp[2:0];
  wire bh_ready = !bq_empty && (!bq_cfg[bh] || t_done[bq_tag[bh]]);
  assign s_bid    = bq_id[bh];
  assign s_bresp  = bq_cfg[bh] ? (t_err[bq_tag[bh]] ? 2'b10 : 2'b00) : bq_resp[bh];
  assign s_bvalid = bh_ready;
  assign fr_en    = s_bvalid && s_bready && bq_cfg[bh];
  assign fr_tag   = bq_tag[bh];

  logic [TW-1:0] cfgw_tag_r;
  // ---------------------------------------------------------------------------
  // W FSM
  // ---------------------------------------------------------------------------
  typedef enum logic [2:0] {W_IDLE, W_BEATS, W_EVAL, W_SEND, W_CFG, W_FIN} w_st_t;
  w_st_t w_st;
  aw_t   cur;
  logic [63:0] cur_addr;
  logic        burst_bad, last_chunk_flag, wlast_seen;
  logic [127:0] c_data [0:15];
  logic [15:0]  c_be   [0:15];
  logic [63:0]  blk_addr;               // address of the first beat of the chunk
  logic [31:0]  cfg_d; logic [3:0] cfg_b;

  wire [8:0]  mps_b = (mps_code == 3'd0) ? 9'd128 : 9'd256;
  wire [63:0] nxt_addr = cur_addr + (64'd1 << cur.size);
  wire        blk_cross = (mps_code == 3'd0) ? (nxt_addr[11:7] != cur_addr[11:7]) : (nxt_addr[11:8] != cur_addr[11:8]);
  wire        beat_hs = s_wvalid && s_wready;
  assign s_wready = dl_up && (w_st == W_BEATS) && !bq_full;

  // byte mask of the active window in the 16 B word
  logic [15:0] win_mask;
  always_comb begin
    win_mask = ((16'd1 << (16'd1 << cur.size)) - 16'd1) << cur_addr[3:0];
  end
  wire [15:0] bmask = s_wstrb & win_mask;

  // strobe analysis (registered in W_EVAL)
  logic [63:0] dwen, dwfull;
  logic [3:0]  dwbe [0:63];
  always_comb begin
    for (int w = 0; w < 16; w++)
      for (int l = 0; l < 4; l++) begin
        dwbe[4*w+l]   = c_be[w][4*l +: 4];
        dwen[4*w+l]   = |c_be[w][4*l +: 4];
        dwfull[4*w+l] = &c_be[w][4*l +: 4];
      end
  end
  logic        an_none, an_bad;
  logic [5:0]  an_f, an_l;
  logic [3:0]  an_fbe, an_lbe;
  always_comb begin
    logic found;
    an_f = '0; an_l = '0; found = 1'b0; an_bad = 1'b0;
    an_none = (dwen == 64'd0);
    for (int d = 0; d < 64; d++) begin
      if (dwen[d] && !found) begin an_f = d[5:0]; found = 1'b1; end
      if (dwen[d]) an_l = d[5:0];
    end
    an_fbe = dwbe[an_f]; an_lbe = (an_l == an_f) ? 4'b0000 : dwbe[an_l];
    for (int d = 0; d < 64; d++)
      if (!an_none && d > an_f && d < an_l && !dwfull[d]) an_bad = 1'b1;
    if (!an_none) begin
      if (an_f == an_l) begin if (!be_contig(an_fbe)) an_bad = 1'b1; end
      else begin
        if (!fbe_ok_multi(an_fbe)) an_bad = 1'b1;
        if (!lbe_ok_multi(an_lbe)) an_bad = 1'b1;
      end
    end
  end
  logic        e_none, e_bad; logic [5:0] e_f, e_l; logic [3:0] e_fbe, e_lbe;
  assign ev_sparse = (w_st == W_EVAL) && an_bad && !burst_bad;

  // chunk outputs
  logic [63:0][31:0] ring_v;
  logic [63:0]       mask_v;
  always_comb begin
    for (int w = 0; w < 16; w++)
      for (int l = 0; l < 4; l++) begin
        ring_v[4*w+l] = c_data[w][32*l +: 32];
        mask_v[4*w+l] = (4*w+l >= e_f) && (4*w+l <= e_l);
      end
  end
  assign ck_ring = ring_v;
  assign ck_mask = mask_v;
  assign ck_addr = {blk_addr[63:8], e_f, 2'b00};
  assign ck_len  = {1'b0, e_l} - {1'b0, e_f} + 7'd1;
  assign ck_fbe  = e_fbe;
  assign ck_lbe  = (e_l == e_f) ? 4'b0000 : e_lbe;
  assign ck_hdr4 = (ck_addr[63:32] != 32'b0);
  assign ck_valid = (w_st == W_SEND) && !burst_bad && !e_none && dl_up;

  assign cfgw_req  = (w_st == W_CFG);
  assign cfgw_addr = cur.addr[27:2];
  assign cfgw_data = cfg_d;
  assign cfgw_be   = cfg_b;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      aw_wp <= '0; aw_rp <= '0; bq_wp <= '0; bq_rp <= '0;
      w_st <= W_IDLE; cur <= '0; cur_addr <= '0; burst_bad <= 1'b0; blk_addr <= '0; cfg_d <= '0; cfg_b <= '0;
      aw_acc_cnt <= '0; aw_done_cnt <= '0;
      e_none <= 1'b1; e_bad <= 1'b0; e_f <= '0; e_l <= '0; e_fbe <= '0; e_lbe <= '0; last_chunk_flag <= 1'b0;
      for (int i = 0; i < 4; i++) aw_q[i] <= '0;
      for (int i = 0; i < 8; i++) begin bq_id[i] <= '0; bq_resp[i] <= '0; bq_cfg[i] <= 1'b0; bq_tag[i] <= '0; end
      for (int i = 0; i < 16; i++) begin c_data[i] <= '0; c_be[i] <= '0; end
    end else begin
      // AW intake
      if (aw_hs) begin
        aw_q[aw_wp[1:0]] <= '{id: s_awid, addr: s_awaddr, len: s_awlen, size: s_awsize, kind: aw_kind};
        aw_wp <= aw_wp + 1'b1;
        aw_acc_cnt <= aw_acc_cnt + 8'd1;
      end
      // B pop
      if (s_bvalid && s_bready) bq_rp <= bq_rp + 1'b1;

      case (w_st)
        W_IDLE: if (!aw_empty && !bq_full) begin
          cur <= aw_q[aw_rp[1:0]]; aw_rp <= aw_rp + 1'b1;
          cur_addr <= aw_q[aw_rp[1:0]].addr;
          blk_addr <= aw_q[aw_rp[1:0]].addr;
          burst_bad <= (aw_q[aw_rp[1:0]].kind == 2'd2);
          for (int i = 0; i < 16; i++) c_be[i] <= '0;
          w_st <= W_BEATS;
        end
        W_BEATS: if (beat_hs) begin
          cur_addr <= nxt_addr;
          if (cur.kind == 2'd0 && !burst_bad) begin
            for (int b = 0; b < 16; b++)
              if (bmask[b]) c_data[cur_addr[7:4]][8*b +: 8] <= s_wdata[8*b +: 8];
            c_be[cur_addr[7:4]] <= c_be[cur_addr[7:4]] | bmask;
          end
          if (cur.kind == 2'd1) begin
            cfg_d <= s_wdata[32*cur_addr[3:2] +: 32];
            cfg_b <= s_wstrb[4*cur_addr[3:2] +: 4];
          end
          last_chunk_flag <= s_wlast;
          if (cur.kind == 2'd0 && !burst_bad && (s_wlast || blk_cross)) w_st <= W_EVAL;
          else if (s_wlast) w_st <= (cur.kind == 2'd1) ? W_CFG : W_FIN;
        end
        W_EVAL: begin
          e_none <= an_none; e_bad <= an_bad; e_f <= an_f; e_l <= an_l; e_fbe <= an_fbe; e_lbe <= an_lbe;
          w_st <= W_SEND;
          if (an_bad && !burst_bad) begin burst_bad <= 1'b1; end
        end
        W_SEND: begin
          if (e_none || burst_bad || ck_ready) begin
            // chunk consumed or dropped : clear and carry on
            for (int i = 0; i < 16; i++) c_be[i] <= '0;
            blk_addr <= cur_addr;
            w_st <= last_chunk_flag ? W_FIN : W_BEATS;
          end
        end
        W_CFG: if (cfgw_gnt) begin
          // B entry for the config write is pushed below
          w_st <= W_FIN;
        end
        W_FIN: begin
          aw_done_cnt <= aw_done_cnt + 8'd1;
          w_st <= W_IDLE;
        end
        default: w_st <= W_IDLE;
      endcase

      // B queue push
      if (w_st == W_FIN) begin
        bq_id[bq_wp[2:0]] <= cur.id;
        bq_cfg[bq_wp[2:0]] <= (cur.kind == 2'd1) && !burst_bad;
        bq_tag[bq_wp[2:0]] <= cfgw_tag_r;
        bq_resp[bq_wp[2:0]] <= burst_bad ? 2'b10 : 2'b00;
        bq_wp <= bq_wp + 1'b1;
      end
    end
  end

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) cfgw_tag_r <= '0;
    else if (cfgw_gnt) cfgw_tag_r <= cfgw_tag;
  end
  assign busy = !aw_empty || (w_st != W_IDLE) || !bq_empty;
endmodule
