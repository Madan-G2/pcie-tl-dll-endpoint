// beat FIFO used by the three TX queues (entries carry sop/eop/keep/age)
module tl_queue #(parameter int DEPTH = 64) (
  input  logic         clk,
  input  logic         rst_n,
  input  logic         flush,
  // push
  input  logic         push,
  input  logic [127:0] push_data,
  input  logic [3:0]   push_keep,
  input  logic         push_sop,
  input  logic         push_eop,
  input  logic [15:0]  push_age,
  output logic         space,            // at least one free entry
  // head (async read)
  output logic [127:0] head_data,
  output logic [3:0]   head_keep,
  output logic         head_sop,
  output logic         head_eop,
  output logic [15:0]  head_age,
  input  logic         pop,
  output logic [$clog2(DEPTH):0] tlp_cnt,   // complete TLPs queued
  output logic         empty
);
  localparam int AW = $clog2(DEPTH);
  logic [127:0] m_data [0:DEPTH-1];
  logic [3:0]   m_keep [0:DEPTH-1];
  logic         m_sop  [0:DEPTH-1];
  logic         m_eop  [0:DEPTH-1];
  logic [15:0]  m_age  [0:DEPTH-1];
  logic [AW:0]  wp, rp;
  wire  [AW:0]  cnt = wp - rp;
  assign space     = (cnt < (AW+1)'(DEPTH));
  assign empty     = (cnt == 0);
  assign head_data = m_data[rp[AW-1:0]];
  assign head_keep = m_keep[rp[AW-1:0]];
  assign head_sop  = m_sop[rp[AW-1:0]];
  assign head_eop  = m_eop[rp[AW-1:0]];
  assign head_age  = m_age[rp[AW-1:0]];

  // storage array: no reset (a RAM has none); contents are qualified by wp/rp
  // synopsys translate_off
  initial for (int i = 0; i < DEPTH; i++) begin m_data[i] = '0; m_keep[i] = '0; m_sop[i] = 1'b0; m_eop[i] = 1'b0; m_age[i] = '0; end
  // synopsys translate_on
  always @(posedge clk) begin
    if (push) begin
      m_data[wp[AW-1:0]] <= push_data;
      m_keep[wp[AW-1:0]] <= push_keep;
      m_sop [wp[AW-1:0]] <= push_sop;
      m_eop [wp[AW-1:0]] <= push_eop;
      m_age [wp[AW-1:0]] <= push_age;
    end
  end

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      wp <= '0; rp <= '0; tlp_cnt <= '0;
    end else if (flush) begin
      wp <= '0; rp <= '0; tlp_cnt <= '0;
    end else begin
      if (push) wp <= wp + 1'b1;
      if (pop) rp <= rp + 1'b1;
      tlp_cnt <= tlp_cnt + ((push && push_eop) ? 1'b1 : 1'b0) - ((pop && head_eop) ? 1'b1 : 1'b0);
    end
  end
endmodule
