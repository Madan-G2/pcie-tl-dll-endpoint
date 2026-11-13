// ============================================================================
// mst_rd_path : inbound MRd -> AXI4 master read burst (AX-11, AX-12),
//               cplr_tracker (slot state + read data RAM),
//               cpl_gen (completion splitting, TL-21 .. TL-24).
// ============================================================================
module mst_rd_path
  import pcie_pkg::*;
#(
  parameter int MAX_CPLR_REQS = 8,
  parameter logic [15:0] DEVICE_ID = 16'h0100
) (
  input  logic         clk,
  input  logic         rst_n,
  input  logic         dl_up,
  input  logic [63:0]  axi_base,
  input  logic [2:0]   mps_code,
  // request from tl_rx (NP dispatch)
  input  logic         nd_valid,
  output logic         nd_ready,
  input  logic [27:0]  nd_off,
  input  logic [9:0]   nd_len,
  input  logic [3:0]   nd_fbe,
  input  logic [3:0]   nd_lbe,
  input  logic [15:0]  nd_reqid,
  input  logic [7:0]   nd_tag,
  input  logic [15:0]  nd_snap,
  output logic         np_rel,
  input  logic [15:0]  wr_b_cnt,
  // AXI master read channels
  output logic [3:0]   m_arid,
  output logic [63:0]  m_araddr,
  output logic [7:0]   m_arlen,
  output logic [2:0]   m_arsize,
  output logic [1:0]   m_arburst,
  output logic         m_arvalid,
  input  logic         m_arready,
  input  logic [3:0]   m_rid,
  input  logic [127:0] m_rdata,
  input  logic [1:0]   m_rresp,
  input  logic         m_rlast,
  input  logic         m_rvalid,
  output logic         m_rready,
  // completions -> tl_tx cpl0
  output logic         cpl0_valid,
  input  logic         cpl0_ready,
  output logic [127:0] cpl0_data,
  output logic [3:0]   cpl0_keep,
  output logic         cpl0_sop,
  output logic         cpl0_eop,
  output logic         ev_axi_mst_err,
  output logic         busy
);
  localparam int SW  = (MAX_CPLR_REQS <= 2) ? 1 : $clog2(MAX_CPLR_REQS);
  localparam int RBW = 64;                       // words per slot (>= 33)

  // ---------------------------------------------------------------------------
  // request intake : one pending request register
  // ---------------------------------------------------------------------------
  logic        pq_v;
  logic [27:0] pq_off;
  logic [9:0]  pq_len;
  logic [3:0]  pq_fbe, pq_lbe;
  logic [15:0] pq_reqid, pq_snap;
  logic [7:0]  pq_tag;
  assign nd_ready = !pq_v;

  // slot state
  logic [MAX_CPLR_REQS-1:0] s_busy, s_done, s_err, s_abort;
  logic [11:0] s_a    [0:MAX_CPLR_REQS-1];
  logic [9:0]  s_len  [0:MAX_CPLR_REQS-1];
  logic [3:0]  s_fbe  [0:MAX_CPLR_REQS-1];
  logic [3:0]  s_lbe  [0:MAX_CPLR_REQS-1];
  logic [15:0] s_rid  [0:MAX_CPLR_REQS-1];
  logic [7:0]  s_tag  [0:MAX_CPLR_REQS-1];
  logic [5:0]  s_beat [0:MAX_CPLR_REQS-1];

  logic [SW-1:0] free_slot;
  logic          have_free;
  always_comb begin
    have_free = 1'b0; free_slot = '0;
    for (int i = MAX_CPLR_REQS - 1; i >= 0; i--)
      if (!s_busy[i]) begin have_free = 1'b1; free_slot = i[SW-1:0]; end
  end

  wire [15:0] bdiff  = wr_b_cnt - pq_snap;
  wire        wr_ok  = !bdiff[15];                  // every earlier inbound MWr has its B (AX-12)
  wire [10:0] tot_dw = {1'b0, pq_len} + {9'b0, pq_off[3:2]};
  wire [8:0]  nwrd   = 9'((tot_dw + 11'd3) >> 2);

  // the ARID must stay stable while ARVALID waits for ARREADY (A-AX-1): the chosen
  // slot is latched when ARVALID is first presented and not ready.
  logic          ar_hold;
  logic [SW-1:0] ar_slot_h;
  wire  [SW-1:0] ar_slot = ar_hold ? ar_slot_h : free_slot;
  assign m_arid    = 4'(ar_slot);
  assign m_araddr  = axi_base + {36'b0, pq_off[27:4], 4'b0};
  assign m_arlen   = 8'(nwrd) - 8'd1;
  assign m_arsize  = 3'd4;
  assign m_arburst = 2'b01;
  assign m_arvalid = pq_v && have_free && wr_ok && dl_up;
  assign np_rel    = m_arvalid && m_arready;

  // completion-order FIFO of finished slots
  logic [SW-1:0] dq [0:MAX_CPLR_REQS-1];
  logic [SW:0]   dq_wp, dq_rp;

  // read data RAM : {slot, beat}
  logic [SW+5:0] rb_wa, rb_ra;
  logic          rb_re;
  logic [127:0]  rb_rd;
  wire  [SW-1:0] r_slot = m_rid[SW-1:0];
  assign rb_wa = {r_slot, s_beat[r_slot]};
  ram_sdp #(.W(128), .DEPTH(MAX_CPLR_REQS*RBW)) u_rbuf (
    .clk(clk), .we(m_rvalid), .waddr(rb_wa), .wdata(m_rdata), .re(rb_re), .raddr(rb_ra), .rdata(rb_rd));
  assign m_rready = 1'b1;
  assign ev_axi_mst_err = m_rvalid && (m_rresp != 2'b00);

  // ---------------------------------------------------------------------------
  // cpl_gen state
  // ---------------------------------------------------------------------------
  typedef enum logic [2:0] {G_IDLE, G_INIT, G_CHUNK, G_FEED, G_DRAIN, G_ERR} g_st_t;
  g_st_t g_st;
  logic [SW-1:0] g_slot;
  logic [12:0]   g_a, g_end, g_cur, ck_end;
  logic [15:0]   g_rid;
  logic [7:0]    g_tag;
  logic [9:0]    ck_e0;
  logic [9:0]    in_left;
  logic [5:0]    rd_word;
  logic          rd_v, rd_v_first, rd_v_last, iss_first;
  logic [2:0]    rd_v_n;
  logic [31:0]   hd0, hd1, hd2;                 // logical header DWs

  // output beat FIFO (8 deep)
  logic [127:0] of_d [0:7];
  logic [3:0]   of_k [0:7];
  logic         of_s [0:7];
  logic         of_e [0:7];
  logic [2:0]   of_wp, of_rp;
  logic [3:0]   of_cnt;
  assign cpl0_valid = (of_cnt != 0);
  assign cpl0_data  = of_d[of_rp];
  assign cpl0_keep  = of_k[of_rp];
  assign cpl0_sop   = of_s[of_rp];
  assign cpl0_eop   = of_e[of_rp];
  wire of_pop = cpl0_valid && cpl0_ready;

  // repack engine : skip the leading DWs of the first word, data starts at lane 3
  logic         e_ov, e_of, e_ol, e_busy;
  logic [127:0] e_od;
  logic [3:0]   e_oen;
  logic [2:0]   e_occ;
  dw_repack u_cp (.clk, .rst_n, .abort(!dl_up),
    .in_valid(rd_v), .in_first(rd_v_first), .in_last(rd_v_last), .in_data(rb_rd), .in_n(rd_v_n),
    .skip_i({1'b0, ck_e0[1:0]}), .off_i(2'd3),
    .out_valid(e_ov), .out_data(e_od), .out_en(e_oen), .out_first(e_of), .out_last(e_ol),
    .occ(e_occ), .busy(e_busy));

  wire single_push = (g_st == G_ERR) && (of_cnt <= 4'd5);
  logic         ob_push, ob_s, ob_e;
  logic [127:0] ob_d;
  logic [3:0]   ob_k;
  always_comb begin
    ob_push = 1'b0; ob_d = '0; ob_k = '0; ob_s = 1'b0; ob_e = 1'b0;
    if (e_ov) begin
      ob_push = 1'b1; ob_d = e_od; ob_s = e_of; ob_e = e_ol; ob_k = e_oen;
      if (e_of) begin
        ob_d[31:0] = bsw32(hd0); ob_d[63:32] = bsw32(hd1); ob_d[95:64] = bsw32(hd2);
        ob_k = e_oen | 4'b0111;
      end
    end else if (single_push) begin
      ob_push = 1'b1; ob_d = {32'h0, bsw32(hd2), bsw32(hd1), bsw32(hd0)}; ob_k = 4'b0111; ob_s = 1'b1; ob_e = 1'b1;
    end
  end

  // ---- chunk arithmetic (TL-21 / TL-22) ----------------------------------------------
  wire [12:0] mps_b    = (mps_code == 3'd0) ? 13'd128 : 13'd256;
  wire [12:0] cur_pm   = g_cur + mps_b;
  wire [12:0] cap_end  = {cur_pm[12:6], 6'b0};                  // align_down(cur + MPS, RCB)
  wire [12:0] ck_end_c = (g_end < cap_end) ? g_end : cap_end;
  wire [12:0] cur_f4   = {g_cur[12:2], 2'b00};
  wire [12:0] end_c4   = (ck_end_c + 13'd3) & 13'h1FFC;
  wire [12:0] ck_len_b = end_c4 - cur_f4;                       // bytes, multiple of 4
  wire [12:0] d0_c     = (cur_f4 - {g_a[12:2], 2'b00}) >> 2;
  wire [12:0] e0_c     = d0_c + {11'b0, g_a[3:2]};              // burst DW index of first payload DW
  wire [11:0] bc_c     = g_end[11:0] - g_cur[11:0];

  function automatic logic [4:0] lead_f(input logic [3:0] be);
    return be[0] ? 5'd0 : be[1] ? 5'd1 : be[2] ? 5'd2 : 5'd3;
  endfunction
  function automatic logic [4:0] trail_f(input logic [3:0] be);
    return be[3] ? 5'd0 : be[2] ? 5'd1 : be[1] ? 5'd2 : 5'd3;
  endfunction

  wire space_ok = ((of_cnt + {1'b0, e_occ} + 4'd2) <= 4'd8);
  assign rb_re  = (g_st == G_FEED) && space_ok;
  assign rb_ra  = {g_slot, rd_word};

  wire [SW-1:0] sl = g_slot;
  wire [12:0] sl_a    = {1'b0, s_a[sl]};
  wire [12:0] sl_start = sl_a + {8'b0, lead_f(s_fbe[sl])};
  wire [12:0] sl_end   = (s_len[sl] == 10'd1) ? (sl_a + 13'd4 - {8'b0, trail_f(s_fbe[sl])})
                                              : (sl_a + {1'b0, s_len[sl], 2'b00} - {8'b0, trail_f(s_lbe[sl])});

  // ---------------------------------------------------------------------------
  // sequential
  // ---------------------------------------------------------------------------
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      ar_hold <= 1'b0; ar_slot_h <= '0;
      pq_v <= 1'b0; pq_off <= '0; pq_len <= '0; pq_fbe <= '0; pq_lbe <= '0; pq_reqid <= '0; pq_tag <= '0; pq_snap <= '0;
      s_busy <= '0; s_done <= '0; s_err <= '0; s_abort <= '0; dq_wp <= '0; dq_rp <= '0;
      for (int i = 0; i < MAX_CPLR_REQS; i++) begin
        s_a[i] <= '0; s_len[i] <= '0; s_fbe[i] <= '0; s_lbe[i] <= '0; s_rid[i] <= '0; s_tag[i] <= '0; s_beat[i] <= '0; dq[i] <= '0;
      end
      g_st <= G_IDLE; g_slot <= '0; g_a <= '0; g_end <= '0; g_cur <= '0; ck_end <= '0; g_rid <= '0; g_tag <= '0;
      ck_e0 <= '0; in_left <= '0; rd_word <= '0; rd_v <= 1'b0; rd_v_first <= 1'b0; rd_v_last <= 1'b0; rd_v_n <= '0; iss_first <= 1'b0;
      hd0 <= '0; hd1 <= '0; hd2 <= '0;
      of_wp <= '0; of_rp <= '0; of_cnt <= '0;
      for (int i = 0; i < 8; i++) begin of_d[i] <= '0; of_k[i] <= '0; of_s[i] <= 1'b0; of_e[i] <= 1'b0; end
    end else begin
      // ---- intake / AR ----
      if (m_arvalid && !m_arready) begin ar_hold <= 1'b1; ar_slot_h <= ar_slot; end
      else                         ar_hold <= 1'b0;
      if (nd_valid && nd_ready) begin
        pq_v <= 1'b1; pq_off <= nd_off; pq_len <= nd_len; pq_fbe <= nd_fbe; pq_lbe <= nd_lbe;
        pq_reqid <= nd_reqid; pq_tag <= nd_tag; pq_snap <= nd_snap;
      end
      if (np_rel) begin
        pq_v <= 1'b0;
        s_busy[ar_slot] <= 1'b1; s_done[ar_slot] <= 1'b0; s_err[ar_slot] <= 1'b0; s_abort[ar_slot] <= 1'b0;
        s_a[ar_slot] <= pq_off[11:0]; s_len[ar_slot] <= pq_len; s_fbe[ar_slot] <= pq_fbe; s_lbe[ar_slot] <= pq_lbe;
        s_rid[ar_slot] <= pq_reqid; s_tag[ar_slot] <= pq_tag; s_beat[ar_slot] <= '0;
      end
      if (!dl_up) begin
        pq_v <= 1'b0;
        for (int i = 0; i < MAX_CPLR_REQS; i++) if (s_busy[i] && !s_done[i]) s_abort[i] <= 1'b1;
      end

      // ---- R channel ----
      if (m_rvalid) begin
        s_beat[r_slot] <= s_beat[r_slot] + 6'd1;
        if (m_rresp != 2'b00) s_err[r_slot] <= 1'b1;
        if (m_rlast) begin
          if (s_abort[r_slot] || !dl_up) s_busy[r_slot] <= 1'b0;
          else begin
            s_done[r_slot] <= 1'b1;
            dq[dq_wp[SW-1:0]] <= r_slot; dq_wp <= dq_wp + 1'b1;
          end
        end
      end

      // ---- output FIFO ----
      if (!dl_up) begin
        of_wp <= '0; of_rp <= '0; of_cnt <= '0;
      end else begin
        if (ob_push) begin
          of_d[of_wp] <= ob_d; of_k[of_wp] <= ob_k; of_s[of_wp] <= ob_s; of_e[of_wp] <= ob_e; of_wp <= of_wp + 3'd1;
        end
        if (of_pop) of_rp <= of_rp + 3'd1;
        of_cnt <= of_cnt + {3'b0, ob_push} - {3'b0, of_pop};
      end

      // ---- completion generation ----
      rd_v <= 1'b0;
      case (g_st)
        G_IDLE: if (dq_wp != dq_rp && dl_up) begin
          g_slot <= dq[dq_rp[SW-1:0]];
          dq_rp  <= dq_rp + 1'b1;
          g_st   <= G_INIT;
        end
        G_INIT: begin
          g_a <= sl_a; g_end <= sl_end; g_cur <= sl_start;
          g_rid <= s_rid[sl]; g_tag <= s_tag[sl];
          if (s_err[sl] || s_abort[sl]) begin     // AXI error : CA completion (TL-24)
            hd0 <= {FT_CPL, 24'b0};
            hd1 <= {DEVICE_ID, CPL_CA, 1'b0, 12'd4};
            hd2 <= {s_rid[sl], s_tag[sl], 1'b0, 7'd0};
            g_st <= G_ERR;
          end else g_st <= G_CHUNK;
        end
        G_CHUNK: if (space_ok && !e_busy && !rd_v) begin
          ck_end  <= ck_end_c;
          ck_e0   <= e0_c[9:0];
          hd0 <= {FT_CPLD, 24'b0};
          hd0[9:0] <= ck_len_b[11:2];
          hd1 <= {DEVICE_ID, CPL_SC, 1'b0, bc_c};
          hd2 <= {g_rid, g_tag, 1'b0, g_cur[6:0]};
          in_left   <= {1'b0, ck_len_b[11:2]} + {8'b0, e0_c[1:0]};
          rd_word   <= e0_c[7:2];
          iss_first <= 1'b1;
          g_st <= G_FEED;
        end
        G_FEED: if (space_ok) begin
          rd_v       <= 1'b1;
          rd_v_first <= iss_first;
          rd_v_last  <= (in_left <= 10'd4);
          rd_v_n     <= (in_left >= 10'd4) ? 3'd4 : in_left[2:0];
          iss_first  <= 1'b0;
          rd_word    <= rd_word + 6'd1;
          in_left    <= in_left - 10'd4;
          if (in_left <= 10'd4) g_st <= G_DRAIN;
        end
        G_DRAIN: if (!rd_v && !e_busy && !e_ov) begin
          if (ck_end >= g_end) begin
            s_busy[g_slot] <= 1'b0; s_done[g_slot] <= 1'b0;
            g_st <= G_IDLE;
          end else begin
            g_cur <= ck_end;
            g_st  <= G_CHUNK;
          end
        end
        G_ERR: if (of_cnt <= 4'd5) begin
          s_busy[g_slot] <= 1'b0; s_done[g_slot] <= 1'b0;
          g_st <= G_IDLE;
        end
        default: g_st <= G_IDLE;
      endcase
      if (!dl_up && g_st != G_IDLE) begin
        s_busy[g_slot] <= 1'b0; g_st <= G_IDLE;
      end
    end
  end

  assign busy = pq_v || (|s_busy) || (g_st != G_IDLE) || (of_cnt != 0);
endmodule
