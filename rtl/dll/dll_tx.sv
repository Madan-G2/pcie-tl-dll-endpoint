// ============================================================================
// dll_tx : sequence numbers, LCRC, replay buffer + descriptors, replay engine,
//          DLLP/TLP/replay transmit multiplexer (5.3, DL-1..DL-8, 8.1)
//
//  Output pipeline: every lnk_tx_* signal is a register.
//  TL stream (tl_*) is wire-order bytes of the TLP, DW granular.
//  Frame = seq(2B) + TLP bytes + LCRC(4B): frame beat j = {tl_beat[j][111:0],
//  prev16}, so the tail of the TLP may spill into one extra beat.
// ============================================================================
module dll_tx
  import pcie_pkg::*;
#(
  parameter int REPLAY_BUF_BEATS = 256,
  parameter int MAX_REPLAY_TLPS  = 64,
  parameter int OPT_EN           = 1
) (
  input  logic          clk,
  input  logic          rst_n,
  input  logic          dl_active,
  input  logic          dll_en,
  // TL -> DLL stream
  input  logic          tl_tx_valid,
  output logic          tl_tx_ready,
  input  logic [127:0]  tl_tx_data,
  input  logic [3:0]    tl_tx_keep,
  input  logic          tl_tx_sop,
  input  logic          tl_tx_eop,
  // Ack/Nak requests from dll_rx
  input  logic          nak_req,
  output logic          nak_grant,
  input  logic          ack_req,
  output logic          ack_grant,
  input  logic [11:0]   rx_seq_m1,
  // FC DLLP request from dll_fc: fc_dllp = {p3, p2, p1, type}
  input  logic          fc_req,
  input  logic [31:0]   fc_dllp,
  output logic          fc_grant,
  // received Ack / Nak
  input  logic          rx_ack_valid,
  input  logic          rx_nak_valid,
  input  logic [11:0]   rx_ack_seq,
  // knobs
  input  logic [7:0]    cfg_win,
  input  logic [15:0]   cfg_timeout,
  input  logic          adapt_en,
  // link transmit
  output logic          lnk_tx_valid,
  output logic [127:0]  lnk_tx_data,
  output logic [15:0]   lnk_tx_keep,
  output logic          lnk_tx_sop,
  output logic          lnk_tx_eop,
  output logic          lnk_tx_dllp,
  // status / events
  output logic          replay_active,
  output logic          tx_idle,
  output logic          ev_rollover,
  output logic          ev_dllp_unexpected,
  output logic          ev_replay_timeout,
  output logic          ev_tx_tlp,
  output logic [10:0]   ev_tx_payload_bytes,
  output logic          ev_beat_tlp,
  output logic          ev_beat_dllp,
  output logic          ev_beat_replay,
  output logic          ev_stall_replay,
  output logic          ev_nak_rx,
  output logic          ev_ack_rx,
  output logic          ev_replay_start,
  output logic          ev_nak_tx,
  output logic          ev_ack_tx,
  output logic [6:0]    dbg_outstanding,
  output logic [11:0]   dbg_next_tx_seq,
  output logic [11:0]   dbg_ackd_seq,
  output logic [1:0]    dbg_replay_num,
  output logic [15:0]   dbg_replay_timer,
  output logic [15:0]   dbg_timeout
);
  // ---- aliases to the internal names ---------------------------------------------
  logic        dl_up, tl_valid, tl_ready, tl_sop, tl_eop;
  logic [127:0] tl_data; logic [3:0] tl_keep;
  logic        rxa_valid, rxa_nak, ack_gnt, nak_gnt, fcd_req, fcd_gnt;
  logic [11:0] rxa_seq, nrs;
  logic [7:0]  fcd_type, fcd_hdr; logic [11:0] fcd_data;
  logic        lnk_valid, lnk_sop, lnk_eop, lnk_dllp;
  logic [127:0] lnk_data; logic [15:0] lnk_keep;
  logic        cfg_adapt_en;
  logic [9:0]  ev_tx_pay_bytes;
  logic        ev_replay_to_start;
  assign dl_up = dl_active;
  assign tl_valid = tl_tx_valid; assign tl_tx_ready = tl_ready; assign tl_data = tl_tx_data;
  assign tl_keep = tl_tx_keep; assign tl_sop = tl_tx_sop; assign tl_eop = tl_tx_eop;
  assign rxa_valid = (rx_ack_valid | rx_nak_valid) & dll_en; assign rxa_nak = rx_nak_valid; assign rxa_seq = rx_ack_seq;
  assign nrs = rx_seq_m1 + 12'd1;
  assign nak_grant = nak_gnt; assign ack_grant = ack_gnt; assign fc_grant = fcd_gnt;
  assign fcd_req = fc_req & dll_en;
  assign fcd_type = fc_dllp[7:0];
  assign fcd_hdr  = {fc_dllp[13:8], fc_dllp[23:22]};
  assign fcd_data = {fc_dllp[19:16], fc_dllp[31:24]};
  assign lnk_tx_valid = lnk_valid; assign lnk_tx_data = lnk_data; assign lnk_tx_keep = lnk_keep;
  assign lnk_tx_sop = lnk_sop; assign lnk_tx_eop = lnk_eop; assign lnk_tx_dllp = lnk_dllp;
  assign cfg_adapt_en = adapt_en && (OPT_EN != 0);
  assign ev_tx_payload_bytes = {1'b0, ev_tx_pay_bytes};
  localparam int AW  = $clog2(REPLAY_BUF_BEATS);
  localparam int NDW = $clog2(MAX_REPLAY_TLPS);

  typedef enum logic [1:0] {S_IDLE, S_TLP, S_TAIL, S_RPL} st_t;
  st_t st;

  // ---------------------------------------------------------------------------
  // replay RAM
  // ---------------------------------------------------------------------------
  logic          ram_we, ram_re;
  logic [AW-1:0] ram_waddr, ram_raddr;
  logic [127:0]  ram_wdata, ram_rdata;
  ram_sdp #(.W(128), .DEPTH(REPLAY_BUF_BEATS), .LANES(1)) u_replay_ram (
    .clk(clk), .we(ram_we), .waddr(ram_waddr), .wdata(ram_wdata),
    .re(ram_re), .raddr(ram_raddr), .rdata(ram_rdata));

  // ---------------------------------------------------------------------------
  // state variables (5.1)
  // ---------------------------------------------------------------------------
  logic [11:0] nts, ackd;
  logic [1:0]  rnum;
  logic [6:0]  n_out;                 // logical outstanding TLPs
  logic [AW:0] used_beats;
  logic [AW-1:0] wptr;
  logic [15:0] now;
  logic        dead;

  // descriptor FIFO
  logic [11:0]   d_seq   [MAX_REPLAY_TLPS];
  logic [AW-1:0] d_start [MAX_REPLAY_TLPS];
  logic [4:0]    d_nb    [MAX_REPLAY_TLPS];
  logic [15:0]   d_keep  [MAX_REPLAY_TLPS];
  logic [15:0]   d_ts    [MAX_REPLAY_TLPS];
  logic          d_rep   [MAX_REPLAY_TLPS];
  logic [NDW-1:0] d_head, d_tail;
  logic [NDW:0]   n_desc;
  logic [11:0]   pop_cnt;
  logic [6:0]    rp_off;

  // ---------------------------------------------------------------------------
  // TL stream processing (frame building)
  // ---------------------------------------------------------------------------
  logic [31:0]   crc_r;
  logic [15:0]   prev_r;
  logic [4:0]    fr_nb;               // beats emitted so far in current frame
  logic [AW-1:0] fr_start;
  logic [11:0]   fr_seq;
  logic [15:0]   tail_keep;
  logic [127:0]  tail_data;
  logic          tail_pend;

  // DLLP select
  logic sel_nak, sel_ack, sel_fc, sel_dllp;
  // replay control
  logic rpl_new, rpl_active_r;
  assign replay_active = rpl_active_r;

  wire dllp_pend = nak_req | ack_req | fcd_req;
  assign sel_nak  = (st == S_IDLE) && nak_req;
  assign sel_ack  = (st == S_IDLE) && !nak_req && ack_req;
  assign sel_fc   = (st == S_IDLE) && !nak_req && !ack_req && fcd_req;
  assign sel_dllp = sel_nak | sel_ack | sel_fc;

  // window / space check from first TL beat
  logic [4:0] hdr_fb;
  logic [2:0] hdr_fmt;
  logic [9:0] hdr_len;
  assign hdr_fmt = tl_data[7:5];
  assign hdr_len = {tl_data[17:16], tl_data[31:24]};
  assign hdr_fb  = frame_beats_from_hdr(hdr_fmt, hdr_len);
  wire [6:0] win_eff = (cfg_win == 8'd0) ? 7'd1 :
                       (cfg_win > 8'(MAX_REPLAY_TLPS) ? 7'(MAX_REPLAY_TLPS) : cfg_win[6:0]);
  wire win_ok   = (n_out < win_eff);
  wire space_ok = ((used_beats + (AW+1)'(hdr_fb)) <= (AW+1)'(REPLAY_BUF_BEATS));
  wire idle_can = dl_up && !dllp_pend && !rpl_active_r && !dead;
  wire ready_idle = idle_can && win_ok && space_ok;

  assign tl_ready = (st == S_IDLE) ? ready_idle : (st == S_TLP);
  wire tl_acc = tl_valid && tl_ready;
  wire first_beat = (st == S_IDLE);

  assign ev_stall_replay = (st == S_IDLE) && tl_valid && idle_can && !(win_ok && space_ok);

  // frame beat construction
  logic [11:0]  seq_eff;
  logic [15:0]  prev_eff;
  logic [31:0]  crc_eff;
  logic [127:0] fbeat;
  logic [2:0]   dw_n;
  logic [31:0]  crc1, crc_f, crc2;
  logic [127:0] o_data;
  logic [15:0]  o_keep;
  logic         o_eop_f, o_tail;
  logic [127:0] t_data;
  logic [15:0]  t_keep;
  logic [4:0]   nbf;
  logic [127:0] mask;

  always_comb begin
    seq_eff  = first_beat ? nts : fr_seq;
    prev_eff = first_beat ? {seq_eff[7:0], 4'b0000, seq_eff[11:8]} : prev_r;
`ifdef MUT_02
    crc_eff  = first_beat ? 32'h00000000 : crc_r;
`else
    crc_eff  = first_beat ? 32'hFFFFFFFF : crc_r;
`endif
    fbeat    = {tl_data[111:0], prev_eff};
    dw_n     = tl_keep[3] ? 3'd4 : tl_keep[2] ? 3'd3 : tl_keep[1] ? 3'd2 : 3'd1;
    nbf      = 5'd2 + {dw_n, 2'b00};
    crc1 = '0; crc_f = '0; crc2 = '0; mask = '0;
    o_data = fbeat; o_keep = 16'hFFFF; o_eop_f = 1'b0; o_tail = 1'b0;
    t_data = '0; t_keep = '0;
    if (!tl_eop) begin
      crc1 = crc32_nb(crc_eff, fbeat, 16);
    end else begin
      case (dw_n)
        3'd1, 3'd2: begin
          crc1   = crc32_nb(crc_eff, fbeat, int'(nbf));
          crc_f  = ~crc1;
          mask   = (128'd1 << (8 * int'(nbf))) - 128'd1;
          o_data = (fbeat & mask) | ({96'b0, crc_f} << (8 * int'(nbf)));
          o_keep = (16'd1 << (nbf + 5'd4)) - 16'd1;
          o_eop_f = 1'b1;
        end
        3'd3: begin
          crc1   = crc32_nb(crc_eff, fbeat, 14);
          crc_f  = ~crc1;
          o_data = {crc_f[15:0], fbeat[111:0]};
          o_keep = 16'hFFFF;
          o_tail = 1'b1;
          t_data = {112'b0, crc_f[31:16]};
          t_keep = 16'h0003;
        end
        default: begin
          crc1   = crc32_nb(crc_eff, fbeat, 16);
          crc2   = crc32_nb(crc1, {112'b0, tl_data[127:112]}, 2);
          crc_f  = ~crc2;
          o_data = fbeat;
          o_keep = 16'hFFFF;
          o_tail = 1'b1;
          t_data = {80'b0, crc_f, tl_data[127:112]};
          t_keep = 16'h003F;
        end
      endcase
    end
  end

  // ---------------------------------------------------------------------------
  // DLLP build
  // ---------------------------------------------------------------------------
  logic [7:0]  dl_type, dl_p1, dl_p2, dl_p3;
  logic [11:0] dl_seq;
  logic [15:0] dl_crc;
  logic [127:0] dl_beat;
  always_comb begin
`ifdef MUT_06
    dl_seq  = sel_nak ? nrs : nrs - 12'd1;
`else
    dl_seq  = nrs - 12'd1;
`endif
    dl_type = DLLP_ACK; dl_p1 = 8'h00; dl_p2 = {4'b0, dl_seq[11:8]}; dl_p3 = dl_seq[7:0];
    if (sel_nak) dl_type = DLLP_NAK;
    else if (sel_fc) begin
      dl_type = fcd_type;
      dl_p1   = {2'b00, fcd_hdr[7:2]};
      dl_p2   = {fcd_hdr[1:0], 2'b00, fcd_data[11:8]};
      dl_p3   = fcd_data[7:0];
    end
`ifdef MUT_14
    dl_crc  = crc16_dllp({dl_p3, dl_p2, dl_p1, dl_type}) ^ 16'h8000;
`else
    dl_crc  = crc16_dllp({dl_p3, dl_p2, dl_p1, dl_type});
`endif
    dl_beat = {80'b0, dl_crc, dl_p3, dl_p2, dl_p1, dl_type};
  end
  assign nak_gnt = sel_nak;
  assign ack_gnt = sel_ack;
  assign fcd_gnt = sel_fc;
  assign ev_nak_tx = sel_nak;
  assign ev_ack_tx = sel_ack;

  // ---------------------------------------------------------------------------
  // Ack / Nak reception (DL-3, DL-4)
  // ---------------------------------------------------------------------------
  logic [11:0] ak_d;
  logic        ak_act, ak_valid, ak_inval;
  assign ak_d     = rxa_seq - ackd;
  assign ak_act   = rxa_valid && dl_up && !dead;
  assign ak_valid = ak_act && ((ak_d == 12'd0) || (ak_d <= {5'b0, n_out}));
  assign ak_inval = ak_act && !ak_valid;
  assign ev_dllp_unexpected = ak_inval;
  assign ev_nak_rx = ak_valid && rxa_nak;
  assign ev_ack_rx = ak_valid && !rxa_nak;

  // ---------------------------------------------------------------------------
  // replay engine
  // ---------------------------------------------------------------------------
  logic [AW-1:0] rp_start;
  logic [4:0]    rp_nb;
  logic [15:0]   rp_lastkeep;
  logic [4:0]    rp_iss, rp_out;
  logic [NDW-1:0] rp_idx;
  assign rp_idx = d_head + NDW'(rp_off);

`ifdef MUT_35
  // M35 (checker validation for DL-18 / A-DL-6 / SB-5): the replay engine waits 12 extra cycles before starting
  logic [3:0] m35_cnt; logic m35_prev;
  always_ff @(posedge clk or negedge rst_n)
    if (!rst_n) begin m35_cnt <= '0; m35_prev <= 1'b0; end
    else begin
      m35_prev <= rpl_active_r;
      if (rpl_active_r && !m35_prev) m35_cnt <= 4'd12; else if (m35_cnt != 4'd0) m35_cnt <= m35_cnt - 4'd1;
    end
  wire rpl_ctl     = (st == S_IDLE) && !sel_dllp && dl_up && rpl_active_r && (pop_cnt == 12'd0) && !dead &&
                     (m35_cnt == 4'd0) && !(rpl_active_r && !m35_prev);
`else
  wire rpl_ctl     = (st == S_IDLE) && !sel_dllp && dl_up && rpl_active_r && (pop_cnt == 12'd0) && !dead;
`endif
  wire rpl_do_new  = rpl_ctl && rpl_new;
`ifdef MUT_08
  wire rpl_do_sweep_end = rpl_ctl && !rpl_new && ((rp_off + 7'd1) >= n_desc[6:0]);
`else
  wire rpl_do_sweep_end = rpl_ctl && !rpl_new && (rp_off >= n_desc[6:0]);
`endif
`ifdef MUT_08
  wire rpl_do_frame = rpl_ctl && !rpl_new && ((rp_off + 7'd1) < n_desc[6:0]);
`else
  wire rpl_do_frame = rpl_ctl && !rpl_new && (rp_off < n_desc[6:0]);
`endif

  // replay timer
  logic        rt_run;
  logic [15:0] rt_cnt;
  logic [15:0] rtt;
  logic [15:0] to_adapt;
  logic [15:0] to_used;
  logic        rt_expire;
  always_comb begin
    logic [16:0] t17;
    t17 = {rtt, 1'b0} + 17'd32;
    if (t17 < 17'd64) to_adapt = 16'd64;
    else if (t17 > 17'd4095) to_adapt = 16'd4095;
    else to_adapt = t17[15:0];
    to_used = cfg_adapt_en ? to_adapt : cfg_timeout;
    rt_expire = rt_run && (rt_cnt >= to_used);
  end

  // frame completion
  wire frame_done_tlp = (tl_acc && tl_eop && !o_tail) || (st == S_TAIL);
  wire frame_start    = tl_acc && first_beat;
  logic [4:0] fr_nb_last;
  logic [15:0] fr_keep_last;
  logic [4:0] fr_nb_now;
  logic [AW-1:0] fr_start_now;
  logic [11:0] fr_seq_now;
  always_comb begin
    fr_nb_now    = (st == S_TAIL) ? fr_nb + 5'd1 : (first_beat ? 5'd1 : fr_nb + 5'd1);
    fr_start_now = first_beat ? wptr : fr_start;
    fr_seq_now   = first_beat ? nts : fr_seq;
    fr_keep_last = (st == S_TAIL) ? tail_keep : o_keep;
  end

  // ---------------------------------------------------------------------------
  // RAM write / read controls
  // ---------------------------------------------------------------------------
  always_comb begin
    ram_we    = 1'b0; ram_wdata = o_data; ram_waddr = wptr;
    ram_re    = 1'b0; ram_raddr = d_start[rp_idx];
    if (tl_acc) begin
      ram_we = 1'b1; ram_wdata = o_data;
    end else if (st == S_TAIL) begin
      ram_we = 1'b1; ram_wdata = tail_data;
    end
    if (rpl_do_frame) begin
      ram_re = 1'b1; ram_raddr = d_start[rp_idx];
    end else if (st == S_RPL && rp_iss < rp_nb) begin
      ram_re = 1'b1; ram_raddr = rp_start + AW'(rp_iss);
    end
  end

  // ---------------------------------------------------------------------------
  // sequential
  // ---------------------------------------------------------------------------
  logic [6:0]  n_out_nxt;
  logic [11:0] ak_free;
`ifdef MUT_12
  assign ak_free = (ak_valid && ak_d != 12'd0) ? ak_d + 12'd1 : 12'd0;
`else
  assign ak_free = (ak_valid && ak_d != 12'd0) ? ak_d : 12'd0;
`endif
  assign n_out_nxt = n_out + 7'(frame_done_tlp) - ak_free[6:0];

  logic pop_en;
  assign pop_en = (pop_cnt != 12'd0) && (n_desc != 0);

`ifdef MUT_10
  assign ev_rollover = rpl_do_new && (rnum == 2'd2) && (n_desc != 0);
`else
  assign ev_rollover = rpl_do_new && (rnum == 2'd3) && (n_desc != 0);
`endif
`ifdef MUT_10
  assign ev_replay_start = rpl_do_new && (rnum != 2'd2) && (n_desc != 0);
`else
  assign ev_replay_start = rpl_do_new && (rnum != 2'd3) && (n_desc != 0);
`endif
  logic to_pending;
  logic to_fire;
  assign to_fire = rt_expire && !rpl_active_r && !dead && dl_up && (n_out_nxt != 7'd0) && !(ak_valid && ak_d != 12'd0);
  assign ev_replay_timeout = to_fire;
  assign ev_replay_to_start = ev_replay_start && to_pending;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      lnk_valid <= 1'b0; lnk_data <= '0; lnk_keep <= '0; lnk_sop <= 1'b0; lnk_eop <= 1'b0; lnk_dllp <= 1'b0;
      st <= S_IDLE; nts <= 12'd0; ackd <= 12'hFFF; rnum <= 2'd0; n_out <= '0; used_beats <= '0;
      wptr <= '0; now <= '0; dead <= 1'b0;
      d_head <= '0; d_tail <= '0; n_desc <= '0; pop_cnt <= '0; rp_off <= '0;
      crc_r <= 32'hFFFFFFFF; prev_r <= '0; fr_nb <= '0; fr_start <= '0; fr_seq <= '0;
      tail_keep <= '0; tail_data <= '0; tail_pend <= 1'b0;
      rpl_new <= 1'b0; rpl_active_r <= 1'b0; to_pending <= 1'b0;
      rp_start <= '0; rp_nb <= '0; rp_lastkeep <= '0; rp_iss <= '0; rp_out <= '0;
      rt_run <= 1'b0; rt_cnt <= '0; rtt <= 16'd64;
      ev_tx_tlp <= 1'b0; ev_tx_pay_bytes <= '0; ev_beat_tlp <= 1'b0; ev_beat_dllp <= 1'b0; ev_beat_replay <= 1'b0;
    end else begin
      now <= now + 16'd1;
      ev_tx_tlp <= 1'b0; ev_tx_pay_bytes <= '0; ev_beat_tlp <= 1'b0; ev_beat_dllp <= 1'b0; ev_beat_replay <= 1'b0;
      lnk_valid <= 1'b0; lnk_sop <= 1'b0; lnk_eop <= 1'b0; lnk_dllp <= 1'b0;

      if (!dl_up) begin
        // DL_Inactive / DL_Init : TLP state takes entry values, DLLPs may still flow
        nts <= 12'd0; ackd <= 12'hFFF; rnum <= 2'd0; n_out <= '0; used_beats <= '0; wptr <= '0;
        d_head <= '0; d_tail <= '0; n_desc <= '0; pop_cnt <= '0; rp_off <= '0;
        rpl_new <= 1'b0; rpl_active_r <= 1'b0; to_pending <= 1'b0; dead <= 1'b0;
        rt_run <= 1'b0; rt_cnt <= '0; tail_pend <= 1'b0;
        if (st != S_IDLE) st <= S_IDLE;
        if (sel_fc || sel_ack || sel_nak) begin
          lnk_valid <= 1'b1; lnk_data <= dl_beat; lnk_keep <= 16'h003F;
          lnk_sop <= 1'b1; lnk_eop <= 1'b1; lnk_dllp <= 1'b1; ev_beat_dllp <= 1'b1;
        end
      end else begin
        // ------------------------------------------------------------------
        // output stage / state machine
        // ------------------------------------------------------------------
        case (st)
          S_IDLE: begin
            if (sel_dllp) begin
              lnk_valid <= 1'b1; lnk_data <= dl_beat; lnk_keep <= 16'h003F;
              lnk_sop <= 1'b1; lnk_eop <= 1'b1; lnk_dllp <= 1'b1; ev_beat_dllp <= 1'b1;
            end else if (tl_acc) begin
              lnk_valid <= 1'b1; lnk_data <= o_data; lnk_keep <= o_keep;
              lnk_sop <= 1'b1; lnk_eop <= o_eop_f; lnk_dllp <= 1'b0; ev_beat_tlp <= 1'b1;
              ev_tx_tlp <= 1'b1; ev_tx_pay_bytes <= hdr_fmt[1] ? {hdr_len[7:0], 2'b00} : 10'd0;
              nts <= nts + 12'd1;
              fr_seq <= nts; fr_start <= wptr; fr_nb <= 5'd1; wptr <= wptr + 1'b1;
              if (!tl_eop) begin st <= S_TLP; crc_r <= crc1; prev_r <= tl_data[127:112]; end
              else if (o_tail) begin st <= S_TAIL; tail_data <= t_data; tail_keep <= t_keep; end
            end else if (rpl_do_frame) begin
              st <= S_RPL;
              rp_start <= d_start[rp_idx]; rp_nb <= d_nb[rp_idx]; rp_lastkeep <= d_keep[rp_idx];
              rp_iss <= 5'd1; rp_out <= 5'd0;
              d_rep[rp_idx] <= 1'b1;
              rp_off <= rp_off + 7'd1;
            end
          end
          S_TLP: begin
            if (tl_acc) begin
              lnk_valid <= 1'b1; lnk_data <= o_data; lnk_keep <= o_keep;
              lnk_sop <= 1'b0; lnk_eop <= o_eop_f; lnk_dllp <= 1'b0; ev_beat_tlp <= 1'b1;
              fr_nb <= fr_nb + 5'd1; wptr <= wptr + 1'b1;
              if (!tl_eop) begin crc_r <= crc1; prev_r <= tl_data[127:112]; end
              else begin
                if (o_tail) begin st <= S_TAIL; tail_data <= t_data; tail_keep <= t_keep; end
                else st <= S_IDLE;
              end
            end
          end
          S_TAIL: begin
            lnk_valid <= 1'b1; lnk_data <= tail_data; lnk_keep <= tail_keep;
            lnk_sop <= 1'b0; lnk_eop <= 1'b1; lnk_dllp <= 1'b0; ev_beat_tlp <= 1'b1;
            fr_nb <= fr_nb + 5'd1; wptr <= wptr + 1'b1;
            st <= S_IDLE;
          end
          S_RPL: begin
            lnk_valid <= 1'b1; lnk_data <= ram_rdata;
            lnk_keep <= (rp_out == rp_nb - 5'd1) ? rp_lastkeep : 16'hFFFF;
            lnk_sop <= (rp_out == 5'd0); lnk_eop <= (rp_out == rp_nb - 5'd1); lnk_dllp <= 1'b0;
            ev_beat_tlp <= 1'b1; ev_beat_replay <= 1'b1;
            if (rp_iss < rp_nb) rp_iss <= rp_iss + 5'd1;
            rp_out <= rp_out + 5'd1;
            if (rp_out == rp_nb - 5'd1) st <= S_IDLE;
          end
        endcase

        // ------------------------------------------------------------------
`ifdef MUT_13
        if (wptr == AW'(REPLAY_BUF_BEATS-1) && (tl_acc || st == S_TAIL)) wptr <= wptr; // descriptor push at frame end
`else
        // descriptor push at frame end
`endif
        // ------------------------------------------------------------------
        if (frame_done_tlp) begin
          d_seq[d_tail]   <= fr_seq_now;
          d_start[d_tail] <= fr_start_now;
          d_nb[d_tail]    <= fr_nb_now;
          d_keep[d_tail]  <= fr_keep_last;
          d_ts[d_tail]    <= now;
          d_rep[d_tail]   <= 1'b0;
          d_tail          <= d_tail + 1'b1;
        end

        // ------------------------------------------------------------------
        // counts: n_out, n_desc, used_beats, pops
        // ------------------------------------------------------------------
        n_out <= n_out_nxt;
        pop_cnt <= pop_cnt + ak_free - (pop_en ? 12'd1 : 12'd0);
        if (pop_en) begin
          d_head  <= d_head + 1'b1;
          if (rp_off != 7'd0) rp_off <= rp_off - 7'd1;
          // RTT sample on the last frame of an Ack
          if (cfg_adapt_en && pop_cnt == 12'd1 && !d_rep[d_head]) begin
            logic [15:0] smp;
            logic signed [16:0] dlt;
            smp = now - d_ts[d_head];
            if (smp > 16'd2048) smp = 16'd2048;
            dlt = $signed({1'b0, smp}) - $signed({1'b0, rtt});
            rtt <= rtt + 16'(dlt >>> 3);
          end
        end
        n_desc <= n_desc + (NDW+1)'(frame_done_tlp) - (NDW+1)'(pop_en);
        used_beats <= used_beats + (frame_done_tlp ? (AW+1)'(fr_nb_now) : '0)
                                 - (pop_en ? (AW+1)'(d_nb[d_head]) : '0);

        // ------------------------------------------------------------------
        // Ack / Nak
        // ------------------------------------------------------------------
        if (ak_valid) begin
          ackd <= rxa_seq;
`ifdef MUT_09
          ;
`else
          if (ak_d != 12'd0) rnum <= 2'd0;
`endif
          if (rxa_nak && (n_out_nxt != 7'd0)) begin
            rpl_new <= 1'b1; rpl_active_r <= 1'b1; to_pending <= 1'b0;
          end
        end

        // ------------------------------------------------------------------
        // replay sweep control
        // ------------------------------------------------------------------
        if (rpl_do_new) begin
          rpl_new <= 1'b0;
          if (n_desc == 0) begin
            rpl_active_r <= 1'b0;
`ifdef MUT_10
          end else if (rnum == 2'd2) begin
`else
          end else if (rnum == 2'd3) begin
`endif
            dead <= 1'b1; rpl_active_r <= 1'b0;
          end else begin
            rnum <= rnum + 2'd1;
`ifdef MUT_07
            rp_off <= n_desc[6:0] - 7'd1;
`else
            rp_off <= 7'd0;
`endif
          end
        end
        if (rpl_do_sweep_end) begin
          rpl_active_r <= 1'b0;
        end

        // ------------------------------------------------------------------
        // replay timer (DL-7)
        // ------------------------------------------------------------------
        if (rpl_active_r || dead) begin
          rt_run <= 1'b0; rt_cnt <= '0;
        end else if (n_out_nxt == 7'd0) begin
          rt_run <= 1'b0; rt_cnt <= '0;
        end else if (ak_valid && ak_d != 12'd0) begin
`ifdef MUT_11
          rt_run <= 1'b1;
`else
          rt_run <= 1'b1; rt_cnt <= '0;
`endif
        end else if (to_fire) begin
          rt_run <= 1'b0; rt_cnt <= '0;
          rpl_new <= 1'b1; rpl_active_r <= 1'b1; to_pending <= 1'b1;
        end else if (!rt_run && frame_done_tlp) begin
          rt_run <= 1'b1; rt_cnt <= '0;
        end else if (rt_run) begin
          rt_cnt <= rt_cnt + 16'd1;
        end
        if (rpl_do_sweep_end && n_out_nxt != 7'd0) begin
          rt_run <= 1'b1; rt_cnt <= '0;
        end
      end
    end
  end

  assign tx_idle = (n_out == 7'd0) && (st == S_IDLE) && !dllp_pend && !rpl_active_r;
  assign dbg_outstanding = n_out; assign dbg_next_tx_seq = nts; assign dbg_ackd_seq = ackd; assign dbg_replay_num = rnum;
  assign dbg_replay_timer = rt_cnt; assign dbg_timeout = to_used;
endmodule
