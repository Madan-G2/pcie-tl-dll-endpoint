// ============================================================================
// ram_sdp : simple dual-port synchronous RAM, registered read data.
//  - one write port (per-lane write enables), one read port
//  - rdata holds its value while re=0 (so a consumer can stall)
//  - a read of the address written in the same cycle returns the OLD data
// For the synthesis flow this module is the "RAM macro" boundary: it is
// black-boxed in the "without memories" run (see syn/dc_run.tcl).
// ============================================================================
module ram_sdp #(
  parameter int W      = 128,
  parameter int DEPTH  = 256,
  parameter int LANES  = 1,
  parameter int AW     = (DEPTH <= 2) ? 1 : $clog2(DEPTH)
) (
  input  logic              clk,
  input  logic [LANES-1:0]  we,
  input  logic [AW-1:0]     waddr,
  input  logic [W-1:0]      wdata,
  input  logic              re,
  input  logic [AW-1:0]     raddr,
  output logic [W-1:0]      rdata
);
  localparam int LW = W / LANES;
  logic [W-1:0] mem [0:DEPTH-1];

  // simulation-only zero init (no reset in a real RAM)
  // synopsys translate_off
  initial begin
    for (int i = 0; i < DEPTH; i++) mem[i] = '0;
    rdata = '0;
  end
  // synopsys translate_on

  always @(posedge clk) begin
    for (int l = 0; l < LANES; l++)
      if (we[l]) mem[waddr][l*LW +: LW] <= wdata[l*LW +: LW];
    if (re) rdata <= mem[raddr];
  end
endmodule
