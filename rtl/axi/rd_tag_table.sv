// ============================================================================
// rd_tag_table : requester tag table (TL-12..TL-16) + completion receive
//                (match, TL-14 checks, payload re-pack) + cpl_buf RAM.
//   cpl_buf layout: word = {tag, addr[8:4]}, lane = addr[3:2]  (16 B words,
//   32 words = 512 B per tag, one slot per tag).
// ============================================================================
module rd_tag_table
  import pcie_pkg::*;
#(
  parameter int MAX_TAGS = 16,
  parameter logic [15:0] DEVICE_ID = 16'h0100
) (
  input  logic         clk,
  input  logic         rst_n,
  input  logic         dl_up,
  input  logic [31:0]  cpl_timeout,
  input  logic [7:0]   max_outstanding_rd,

  // allocation
  output logic [$clog2(MAX_TAGS)-1:0] al_tag,
  output logic         al_free,           // a tag is free
  output logic         al_free_mem,       // a tag is free and the outstanding-read limit allows a mem read
  input  logic         al_go,
  input  logic [1:0]   al_kind,           // 0 mem read, 1 cfg read, 2 cfg write
  input  logic [3:0]   al_burst,
  input  logic [5:0]   al_sub,
  input  logic [11:0]  al_addr,
  input  logic [9:0]   al_len,            // bytes
  // release
  input  logic         fr_en,
  input  logic [$clog2(MAX_TAGS)-1:0] fr_tag,
  input  logic         fr_en2,
  input  logic [$clog2(MAX_TAGS)-1:0] fr_tag2,

  // state exported
  output logic [MAX_TAGS-1:0] t_valid, t_done, t_err,
  output logic [MAX_TAGS-1:0][3:0] t_burst,
  output logic [MAX_TAGS-1:0][5:0] t_sub,
  output logic [MAX_TAGS-1:0][1:0] t_kind,
  output logic [MAX_TAGS-1:0][15:0] t_done_cyc,

  // completion stream from tl_rx
  input  logic         cp_valid,
  input  logic [127:0] cp_data,
  input  logic [3:0]   cp_keep,
  input  logic         cp_sop,
  input  logic         cp_eop,
  input  logic         cp_bad_v,
  input  logic [7:0]   cp_bad_tag,

  // cpl_buf read port
  input  logic         cb_re,
  input  logic [$clog2(MAX_TAGS)+4:0] cb_raddr,
  output logic [127:0] cb_rdata,

  output logic [15:0]  now_o,
  output logic         ev_unexpected_cpl,
  output logic         ev_cpl_err_rx,
  output logic         ev_cpl_timeout,
  output logic         pending,
  output logic [4:0]   out_cnt
);
  localparam int TW = $clog2(MAX_TAGS);

  logic [15:0] now;
  assign now_o = now;
  logic [1:0]  tk_kind [0:MAX_TAGS-1];
  logic [3:0]  tk_burst[0:MAX_TAGS-1];
  logic [5:0]  tk_sub  [0:MAX_TAGS-1];
  logic [11:0] tk_addr [0:MAX_TAGS-1];
  logic [9:0]  tk_len  [0:MAX_TAGS-1];
  logic [9:0]  tk_rcvd [0:MAX_TAGS-1];
  logic [31:0] tk_timer[0:MAX_TAGS-1];
  logic [15:0] tk_dcyc [0:MAX_TAGS-1];
  logic [MAX_TAGS-1:0] v, d, e;
  always_comb begin
    for (int i = 0; i < MAX_TAGS; i++) begin
      t_burst[i] = tk_burst[i]; t_sub[i] = tk_sub[i]; t_kind[i] = tk_kind[i]; t_done_cyc[i] = tk_dcyc[i];
    end
    t_valid = v; t_done = d; t_err = e;
  end

  // allocation
  logic [4:0] cnt_valid;
  always_comb begin
    al_free = 1'b0; al_tag = '0; cnt_valid = '0;
    for (int i = MAX_TAGS - 1; i >= 0; i--) if (!v[i]) begin al_free = 1'b1; al_tag = i[TW-1:0]; end
    for (int i = 0; i < MAX_TAGS; i++) cnt_valid = cnt_valid + {4'b0, v[i]};
  end
  assign out_cnt     = cnt_valid;
  assign al_free_mem = al_free && (cnt_valid < max_outstanding_rd[4:0]);
  assign pending     = |v;

  // ---------------------------------------------------------------------------
  // completion header decode (beat 0)
  // ---------------------------------------------------------------------------
  logic [127:0] cb0_r;
  logic         feed_r;
  logic         in_pkt;
  wire  [127:0] cb0 = cp_sop ? cp_data : cb0_r;
  wire  [31:0]  h0 = bsw32(cb0[31:0]);
  wire  [31:0]  h1 = bsw32(cb0[63:32]);
  wire  [31:0]  h2 = bsw32(cb0[95:64]);
  wire  [2:0]   c_status = h1[15:13];
  wire  [11:0]  c_bc     = h1[11:0];
  wire  [15:0]  c_reqid  = h2[31:16];
  wire  [7:0]   c_tag    = h2[15:8];
  wire  [6:0]   c_lower  = h2[6:0];
  wire  [9:0]   c_len    = h0[9:0];
  wire          c_hasdata = h0[30];                 // Fmt[1] = payload present
  wire          c_ep     = h0[14];
  wire  [TW-1:0] ti      = c_tag[TW-1:0];

  wire        tag_ok   = (c_reqid == DEVICE_ID) && (c_tag < MAX_TAGS) && v[ti] && !d[ti];
  wire [11:0] nxt_b    = tk_addr[ti] + {2'b0, tk_rcvd[ti]};
  wire [9:0]  remain   = tk_len[ti] - tk_rcvd[ti];
  wire [11:0] chunk_b  = {c_len, 2'b00};
  wire        sc       = (c_status == CPL_SC);
  wire        cfgr     = (tk_kind[ti] == 2'd1);          // config completions carry Lower Address = 0
  wire        data_ok  = c_hasdata && (c_bc == {2'b0, remain}) && (cfgr || (c_lower == nxt_b[6:0])) &&
                         (chunk_b <= {2'b0, remain}) && (c_len != 10'd0);
  wire        cfgw     = (tk_kind[ti] == 2'd2);
  wire        sop_ev   = cp_valid && cp_sop && dl_up;
  wire        m_status = sop_ev && tag_ok && !sc;                                  // UR / CA : close with error
  wire        m_cfgw   = sop_ev && tag_ok && sc && cfgw && !c_hasdata;             // config write completion
  wire        m_data   = sop_ev && tag_ok && sc && !cfgw && data_ok;               // good data chunk
  wire        m_bad    = sop_ev && !(m_status || m_cfgw || m_data);                // unexpected (TL-13/14)
  wire        last_chunk = (chunk_b == {2'b0, remain});

  // context FIFO for payload words
  logic [TW-1:0] cx_tag  [0:3];
  logic [4:0]    cx_base [0:3];
  logic          cx_last [0:3];
  logic [1:0]    cx_wp, cx_rp; logic [2:0] cx_cnt;
  logic [4:0]    wk;                                  // word counter within the TLP

  logic         e_ov, e_of, e_ol, e_busy;
  logic [127:0] e_od; logic [3:0] e_oen; logic [2:0] e_occ;
  wire          feed_now = sop_ev ? m_data : feed_r;
  wire          eng_in   = cp_valid && dl_up && (cp_sop || in_pkt) && feed_now;
  dw_repack u_rp (.clk, .rst_n, .abort(!dl_up),
    .in_valid(eng_in), .in_first(cp_sop), .in_last(cp_eop), .in_data(cp_data), .in_n(pop4(cp_keep)),
    .skip_i(3'd3), .off_i(nxt_b[3:2]),
    .out_valid(e_ov), .out_data(e_od), .out_en(e_oen), .out_first(e_of), .out_last(e_ol), .occ(e_occ), .busy(e_busy));

  wire [TW-1:0] w_tag  = cx_tag[cx_rp];
  wire [4:0]    w_base = cx_base[cx_rp];
  wire [4:0]    w_idx  = e_of ? w_base : (w_base + wk);
  wire [TW+4:0] w_addr = {w_tag, w_idx};
  ram_sdp #(.W(128), .DEPTH(MAX_TAGS*32), .LANES(4)) u_cbuf (
    .clk(clk), .we(e_ov ? e_oen : 4'b0), .waddr(w_addr), .wdata(e_od), .re(cb_re), .raddr(cb_raddr), .rdata(cb_rdata));

  // ---------------------------------------------------------------------------
  // sequential
  // ---------------------------------------------------------------------------
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      now <= '0; v <= '0; d <= '0; e <= '0; cb0_r <= '0; feed_r <= 1'b0; in_pkt <= 1'b0;
      cx_wp <= '0; cx_rp <= '0; cx_cnt <= '0; wk <= '0;
      ev_unexpected_cpl <= 1'b0; ev_cpl_err_rx <= 1'b0; ev_cpl_timeout <= 1'b0;
      for (int i = 0; i < MAX_TAGS; i++) begin
        tk_kind[i] <= '0; tk_burst[i] <= '0; tk_sub[i] <= '0; tk_addr[i] <= '0; tk_len[i] <= '0;
        tk_rcvd[i] <= '0; tk_timer[i] <= '0; tk_dcyc[i] <= '0;
      end
      for (int i = 0; i < 4; i++) begin cx_tag[i] <= '0; cx_base[i] <= '0; cx_last[i] <= 1'b0; end
    end else begin
      now <= now + 16'd1;
      ev_unexpected_cpl <= m_bad;
      ev_cpl_err_rx     <= m_status || (m_data && c_ep);
      ev_cpl_timeout    <= 1'b0;

      // ---- completion stream ----
      if (cp_valid && dl_up) begin
        if (cp_sop) begin cb0_r <= cp_data; feed_r <= m_data; end
        in_pkt <= !cp_eop;
      end
      if (m_status) begin e[ti] <= 1'b1; d[ti] <= 1'b1; tk_dcyc[ti] <= now; end
      if (m_cfgw)   begin d[ti] <= 1'b1; tk_dcyc[ti] <= now; end
      if (m_data) begin
`ifdef MUT_23
        tk_rcvd[ti] <= tk_rcvd[ti] + chunk_b[9:0]; v[ti] <= 1'b0;
`else
        tk_rcvd[ti] <= tk_rcvd[ti] + chunk_b[9:0];
`endif
        if (c_ep) e[ti] <= 1'b1;
        cx_tag[cx_wp] <= ti; cx_base[cx_wp] <= nxt_b[8:4]; cx_last[cx_wp] <= last_chunk;
        cx_wp <= cx_wp + 2'd1;
      end
      // payload words written : context bookkeeping
      if (e_ov) begin
        wk <= e_ol ? 5'd0 : (e_of ? 5'd1 : wk + 5'd1);
        if (e_ol) begin
          if (cx_last[cx_rp]) begin d[w_tag] <= 1'b1; tk_dcyc[w_tag] <= now; end
          cx_rp <= cx_rp + 2'd1;
        end
      end
      cx_cnt <= cx_cnt + {2'b0, m_data} - {2'b0, (e_ov && e_ol)};

      // eop-time failure of a forwarded completion
      if (cp_bad_v && cp_bad_tag < MAX_TAGS && v[cp_bad_tag[TW-1:0]]) begin
        e[cp_bad_tag[TW-1:0]] <= 1'b1; d[cp_bad_tag[TW-1:0]] <= 1'b1; tk_dcyc[cp_bad_tag[TW-1:0]] <= now;
      end

      // ---- allocation / release ----
      if (fr_en)  begin v[fr_tag]  <= 1'b0; d[fr_tag]  <= 1'b0; e[fr_tag]  <= 1'b0; end
      if (fr_en2) begin v[fr_tag2] <= 1'b0; d[fr_tag2] <= 1'b0; e[fr_tag2] <= 1'b0; end
      if (al_go) begin
        v[al_tag] <= 1'b1; d[al_tag] <= 1'b0; e[al_tag] <= 1'b0;
        tk_kind[al_tag] <= al_kind; tk_burst[al_tag] <= al_burst; tk_sub[al_tag] <= al_sub;
        tk_addr[al_tag] <= al_addr; tk_len[al_tag] <= al_len; tk_rcvd[al_tag] <= '0; tk_timer[al_tag] <= '0;
      end

      // ---- timers (TL-16) ----
      for (int i = 0; i < MAX_TAGS; i++) begin
        if (v[i] && !d[i]) begin
          tk_timer[i] <= tk_timer[i] + 32'd1;
          if (cpl_timeout != 32'd0 && tk_timer[i] >= cpl_timeout) begin
            e[i] <= 1'b1; d[i] <= 1'b1; tk_dcyc[i] <= now; ev_cpl_timeout <= 1'b1;
          end
        end
      end

      // ---- link down : fail everything outstanding ----
      if (!dl_up) begin
        for (int i = 0; i < MAX_TAGS; i++) if (v[i] && !d[i]) begin e[i] <= 1'b1; d[i] <= 1'b1; tk_dcyc[i] <= now; end
        cx_wp <= '0; cx_rp <= '0; cx_cnt <= '0; wk <= '0; in_pkt <= 1'b0;
      end
    end
  end
endmodule
