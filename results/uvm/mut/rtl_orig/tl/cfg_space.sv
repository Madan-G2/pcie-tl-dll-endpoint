// ============================================================================
// cfg_space : Type 0 configuration space (9.1, CF-1..CF-3)
// ============================================================================
module cfg_space
  import pcie_pkg::*;
#(
  parameter int MPS_MAX_BYTES  = 256,
  parameter int BAR0_SIZE_LOG2 = 20
) (
  input  logic         clk,
  input  logic         rst_n,
  // access port (from tl_rx / completer)
  input  logic         acc_valid,         // one-cycle strobe
  input  logic         acc_we,
  input  logic [9:0]   acc_dw,            // {ExtReg[3:0], Reg[5:0]}
  input  logic [31:0]  acc_wdata,
  input  logic [3:0]   acc_be,
  output logic [31:0]  acc_rdata,         // combinational read of acc_dw
  // state outputs
  output logic         mse,
  output logic         bme,
  output logic [31:0]  bar0,
  output logic [2:0]   mps_code,
  output logic [2:0]   mrrs_code,
  // error events -> Device Status
  input  logic [15:0]  err_set,
  input  logic         txn_pending
);
  localparam logic [2:0] MPS_MAX_CODE = (MPS_MAX_BYTES >= 256) ? 3'd1 : 3'd0;

  logic [15:0] cmd_r;             // only bits 1,2 implemented
  logic [31:0] bar0_r;
  logic [2:0]  mps_r, mrrs_r;
  logic [3:0]  dstat;             // [0] correctable [1] non-fatal [2] fatal [3] UR detected

  assign mse  = cmd_r[1];
  assign bme  = cmd_r[2];
  assign bar0 = bar0_r;
  assign mps_code  = mps_r;
  assign mrrs_code = mrrs_r;

  wire [31:0] bar_mask = ~((32'd1 << BAR0_SIZE_LOG2) - 32'd1);

  logic [31:0] rd;
  always_comb begin
    rd = 32'h0;
    case (acc_dw)
      10'h000: rd = 32'h0001ABCD;
      10'h001: rd = {12'h001, 4'b0, 16'b0} | {16'b0, 16'b0} | {16'h0010, cmd_r};      // status bit4 (cap list)
      10'h002: rd = 32'h05800001;
      10'h003: rd = 32'h00000000;
      10'h004: rd = bar0_r;
      10'h00D: rd = 32'h00000040;
      10'h010: rd = 32'h00020010;
      10'h011: rd = {29'b0, MPS_MAX_CODE};
      10'h012: rd = {10'b0, txn_pending, 1'b0, dstat[3], dstat[2], dstat[1], dstat[0], 1'b0, mrrs_r, 4'b0, mps_r, 5'b0} & 32'hFFFF_FFFF;
      default: rd = 32'h0;
    endcase
  end
  // Status[31:16] : bit4 of status = capability list ; fix layout explicitly
  logic [31:0] rd_cmd;
  assign rd_cmd = {16'h0010, cmd_r};
  assign acc_rdata = (acc_dw == 10'h001) ? rd_cmd :
                     (acc_dw == 10'h012) ? {10'b0, txn_pending, 1'b0, dstat[3], dstat[2], dstat[1], dstat[0],
                                            1'b0, mrrs_r, 4'b0, mps_r, 5'b0} : rd;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      cmd_r <= 16'h0; bar0_r <= 32'h0; mps_r <= 3'd0; mrrs_r <= 3'd2; dstat <= '0;
    end else begin
      // Device Status set events (CF-3)
      if (|(err_set[ERR_LCRC_ERR_RX] | err_set[ERR_SEQ_ERR_RX] | err_set[ERR_DLLP_CRC_ERR] | err_set[ERR_REPLAY_TIMEOUT]))
        dstat[0] <= 1'b1;
      if (err_set[ERR_UR_SENT] | err_set[ERR_UR_POSTED] | err_set[ERR_POISONED] | err_set[ERR_CPL_TIMEOUT] |
          err_set[ERR_UNEXPECTED_CPL] | err_set[ERR_CPL_ERR_RX] | err_set[ERR_DLLP_UNEXPECTED] |
          err_set[ERR_AXI_UNSUP] | err_set[ERR_AXI_MST_ERR])
        dstat[1] <= 1'b1;
      if (err_set[ERR_REPLAY_NUM_ROLLOVER] | err_set[ERR_MALFORMED] | err_set[ERR_FC_VIOLATION])
        dstat[2] <= 1'b1;
      if (err_set[ERR_UR_SENT] | err_set[ERR_UR_POSTED]) dstat[3] <= 1'b1;

      if (acc_valid && acc_we) begin
        case (acc_dw)
          10'h001: begin
            if (acc_be[0]) cmd_r[7:0] <= {5'b0, acc_wdata[2:1], 1'b0};
          end
          10'h004: begin
            for (int i = 0; i < 4; i++)
              if (acc_be[i]) bar0_r[8*i +: 8] <= acc_wdata[8*i +: 8] & bar_mask[8*i +: 8];
          end
          10'h012: begin
            // Device Control [15:0] : MPS [7:5], MRRS [14:12]
            if (acc_be[0]) mps_r <= (acc_wdata[7:5] > MPS_MAX_CODE) ? MPS_MAX_CODE : acc_wdata[7:5];
            if (acc_be[1]) mrrs_r <= (acc_wdata[14:12] > 3'd2) ? 3'd2 : acc_wdata[14:12];
            // Device Status [31:16] W1C : [16]..[19]
            if (acc_be[2]) begin
              if (acc_wdata[16]) dstat[0] <= 1'b0;
              if (acc_wdata[17]) dstat[1] <= 1'b0;
              if (acc_wdata[18]) dstat[2] <= 1'b0;
              if (acc_wdata[19]) dstat[3] <= 1'b0;
            end
          end
          default: ;
        endcase
      end
    end
  end
endmodule
