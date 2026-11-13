// ============================================================================
// dw_repack : DW stream re-packer (4 x 32-bit lanes per word)
//
//  Takes a stream of input words (in_n valid DWs, lanes 0..in_n-1), drops the
//  first `skip` DWs of the packet, and re-emits the remaining DWs starting at
//  output lane `off`.  One word in per cycle; output words come out of a small
//  FIFO (a flush word may follow the last input word, so two words can be
//  produced in one cycle).  The consumer cannot stall the engine: a source
//  that can stall must reserve 3 output slots (use `occ`) per word pushed.
//
//  Parameters skip_i/off_i are sampled on the in_first word.
//  Output: out_valid/out_data/out_en/out_first/out_last (comb from FIFO head,
//  popped every cycle).
//
//  Used for: payload extraction in tl_rx (MWr -> P buffer, CplD -> cpl_buf),
//  header merge for the write TLP serializer and cpl_gen.
// ============================================================================
module dw_repack (
  input  logic         clk,
  input  logic         rst_n,
  input  logic         abort,
  input  logic         in_valid,
  input  logic         in_first,
  input  logic         in_last,
  input  logic [127:0] in_data,
  input  logic [2:0]   in_n,
  input  logic [2:0]   skip_i,
  input  logic [1:0]   off_i,
  output logic         out_valid,
  output logic [127:0] out_data,
  output logic [3:0]   out_en,
  output logic         out_first,
  output logic         out_last,
  output logic [2:0]   occ,           // words held in the output FIFO
  output logic         busy
);
  logic [2:0]   skip_left;
  logic [2:0]   f;
  logic [3:0]   en_acc;
  logic [127:0] acc;
  logic         first_pend;       // next emitted word is the first of the packet
  logic         in_pkt;
  logic         flush_pend;

  // --- effective state for this cycle --------------------------------------
  logic [2:0]   sk_cur, f_cur;
  logic [3:0]   en_cur;
  logic [127:0] acc_cur;
  logic         first_cur, inpkt_cur;
  always_comb begin
    if (in_valid && in_first) begin
      sk_cur = skip_i; f_cur = {1'b0, off_i}; en_cur = 4'b0; acc_cur = '0;
      first_cur = 1'b1; inpkt_cur = 1'b1;
    end else begin
      sk_cur = skip_left; f_cur = f; en_cur = en_acc; acc_cur = acc;
      first_cur = first_pend; inpkt_cur = in_pkt;
    end
  end

  logic [2:0]   sk_take, n_eff;
  logic [127:0] shifted;
  always_comb begin
    sk_take = (sk_cur > in_n) ? in_n : sk_cur;
    n_eff   = in_n - sk_take;
    shifted = in_data >> (32 * sk_take);
  end

  logic [255:0] comb;
  logic [7:0]   comb_en;
  logic [3:0]   tot;
  always_comb begin
    comb    = {128'b0, acc_cur};
    comb_en = {4'b0, en_cur};
    for (int i = 0; i < 4; i++)
      if (i < n_eff) begin
        comb[32*(f_cur+i) +: 32] = shifted[32*i +: 32];
        comb_en[f_cur+i]         = 1'b1;
      end
    tot = {1'b0, f_cur} + {1'b0, n_eff};
  end

  wire act      = in_valid && inpkt_cur;
  wire new_word = act && (tot >= 4'd4);

  // --- output FIFO (depth 4, up to two pushes per cycle) -------------------
  logic [127:0] q_data [0:3];
  logic [3:0]   q_en   [0:3];
  logic         q_first[0:3];
  logic         q_last [0:3];
  logic [1:0]   wp, rp;
  logic [2:0]   cnt;

  wire push0 = flush_pend && !abort;
  wire push1 = new_word   && !abort;
  wire [1:0] npush = {1'b0, push0} + {1'b0, push1};
  wire pop = (cnt != 0);

  assign out_valid = pop;
  assign out_data  = q_data[rp];
  assign out_en    = q_en[rp];
  assign out_first = q_first[rp];
  assign out_last  = q_last[rp];
  assign occ       = cnt;

  wire new_is_last = in_last && (tot == 4'd4);

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      skip_left <= '0; f <= '0; en_acc <= '0; acc <= '0; first_pend <= 1'b0;
      in_pkt <= 1'b0; flush_pend <= 1'b0; wp <= '0; rp <= '0; cnt <= '0;
      for (int i = 0; i < 4; i++) begin q_data[i] <= '0; q_en[i] <= '0; q_first[i] <= 1'b0; q_last[i] <= 1'b0; end
    end else if (abort) begin
      in_pkt <= 1'b0; flush_pend <= 1'b0; wp <= '0; rp <= '0; cnt <= '0;
    end else begin
      // pushes
      if (push0) begin
        q_data[wp]  <= acc;
        q_en[wp]    <= en_acc;
        q_first[wp] <= first_pend;
        q_last[wp]  <= 1'b1;
      end
      if (push1) begin
        q_data[wp + {1'b0, push0}]  <= comb[127:0];
        q_en[wp + {1'b0, push0}]    <= comb_en[3:0];
        q_first[wp + {1'b0, push0}] <= first_cur;
        q_last[wp + {1'b0, push0}]  <= new_is_last;
      end
      wp  <= wp + {1'b0, npush};
      if (pop) rp <= rp + 2'd1;
      cnt <= cnt + {1'b0, npush} - {2'b0, pop};

      flush_pend <= 1'b0;
      if (in_valid) begin
        skip_left <= sk_cur - sk_take;
        in_pkt    <= !in_last;
        if (inpkt_cur) begin
          if (tot >= 4'd4) begin
            f          <= tot[2:0] - 3'd4;
            en_acc     <= comb_en[7:4];
            acc        <= comb[255:128];
            first_pend <= 1'b0;
            if (in_last && tot > 4'd4) flush_pend <= 1'b1;
          end else begin
            f          <= tot[2:0];
            en_acc     <= comb_en[3:0];
            acc        <= comb[127:0];
            first_pend <= first_cur;
            if (in_last && (comb_en[3:0] != 4'b0)) flush_pend <= 1'b1;
          end
        end
      end
    end
  end
  assign busy = flush_pend | in_pkt | (cnt != 0);
endmodule
