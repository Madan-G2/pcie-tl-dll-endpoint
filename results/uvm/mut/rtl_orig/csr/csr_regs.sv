// ============================================================================
// csr_regs : CSR bus (CS-1..CS-3), tuning knobs, error status, counters
// ============================================================================
module csr_regs
  import pcie_pkg::*;
#(
  parameter int MAX_REPLAY_TLPS = 64,
  parameter int MAX_TAGS        = 16
) (
  input  logic         clk,
  input  logic         rst_n,
  // bus
  input  logic         csr_req,
  input  logic         csr_we,
  input  logic [11:0]  csr_addr,
  input  logic [31:0]  csr_wdata,
  output logic         csr_ack,
  output logic [31:0]  csr_rdata,
  // knobs
  output logic         link_enable,
  output logic         soft_reset,
  output logic         coal_flush,
  output logic         opt_ack_coalesce_en, opt_ack_adapt_en, opt_replay_adapt_en,
  output logic         opt_fc_early_en, opt_wr_coalesce_en, opt_rd_ooo_en,
  output logic [7:0]   ack_coalesce_n,
  output logic [15:0]  ack_timer_cyc,
  output logic [7:0]   adapt_hold,
  output logic [7:0]   replay_win,
  output logic [15:0]  replay_timeout,
  output logic [15:0]  updatefc_timer,
  output logic [7:0]   fc_early_thresh_hdr,
  output logic [7:0]   fc_early_thresh_data,
  output logic [15:0]  fc_early_max_delay,
  output logic [15:0]  coal_timeout,
  output logic [7:0]   max_outstanding_rd,
  output logic [31:0]  cpl_timeout,
  // status
  input  logic         phy_link_up, dl_up, fc_init1_done, fc_init2_done,
  input  logic         replay_active, tx_idle, rx_idle,
  // errors
  input  logic [15:0]  err_set,
  output logic [15:0]  err_status,
  output logic         fatal_err,
  // counters
  input  logic [25:0][11:0] cnt_inc,
  input  logic         rd_lat_valid,
  input  logic [31:0]  rd_lat_val
);
  localparam int NC = 26;

  logic [4:0]  ctrl_cmd;           // registered self-clearing pulses
  logic        link_en_r;
  logic [5:0]  opt_r;
  logic [31:0] ack_cfg, replay_cfg, fc_cfg, wr_coal_cfg, rd_cfg, cpl_to, fc_cfg2, fatal_mask;
  logic [15:0] err_r;

  assign link_enable = link_en_r;
  assign soft_reset  = ctrl_cmd[1];
  assign coal_flush  = ctrl_cmd[4];
  assign {opt_rd_ooo_en, opt_wr_coalesce_en, opt_fc_early_en, opt_replay_adapt_en, opt_ack_adapt_en, opt_ack_coalesce_en} = opt_r;
  assign ack_coalesce_n       = ack_cfg[7:0];
  assign ack_timer_cyc        = ack_cfg[23:8];
  assign adapt_hold           = ack_cfg[31:24];
  assign replay_win           = replay_cfg[7:0];
  assign replay_timeout       = replay_cfg[31:16];
  assign updatefc_timer       = fc_cfg[15:0];
  assign fc_early_thresh_hdr  = fc_cfg[23:16];
  assign fc_early_thresh_data = fc_cfg[31:24];
  assign fc_early_max_delay   = fc_cfg2[15:0];
  assign coal_timeout         = wr_coal_cfg[15:0];
  assign max_outstanding_rd   = rd_cfg[7:0];
  assign cpl_timeout          = cpl_to;
  assign err_status           = err_r;
  assign fatal_err            = |(err_r & fatal_mask[15:0]);

  // counters
  logic [31:0] cnt_live   [0:NC-1];
  logic [31:0] cnt_shadow [0:NC-1];

  // read mux
  logic [31:0] rd_mux;
  always_comb begin
    rd_mux = 32'h0;
    case (csr_addr)
      12'h000: rd_mux = 32'h50434501;
      12'h004: rd_mux = {31'b0, link_en_r};
      12'h008: rd_mux = {24'b0, fatal_err, rx_idle, tx_idle, replay_active, fc_init2_done, fc_init1_done, dl_up, phy_link_up};
      12'h00C: rd_mux = {26'b0, opt_r};
      12'h010: rd_mux = ack_cfg;
      12'h014: rd_mux = replay_cfg;
      12'h018: rd_mux = fc_cfg;
      12'h01C: rd_mux = wr_coal_cfg;
      12'h020: rd_mux = rd_cfg;
      12'h024: rd_mux = cpl_to;
      12'h028: rd_mux = {16'b0, err_r};
      12'h02C: rd_mux = fatal_mask;
      12'h030: rd_mux = fc_cfg2;
      default: begin
        if (csr_addr >= 12'h040 && csr_addr <= 12'h0A4) rd_mux = cnt_shadow[(csr_addr - 12'h040) >> 2];
      end
    endcase
  end

  wire wr = csr_req && csr_we;
  wire [11:0] wa = csr_addr;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      csr_ack <= 1'b0; csr_rdata <= '0; ctrl_cmd <= '0; link_en_r <= 1'b1; opt_r <= '0;
      ack_cfg     <= {8'd16, 16'd32, 8'd4};
      replay_cfg  <= {16'd1024, 8'd0, 8'd8};
      fc_cfg      <= {8'd32, 8'd8, 16'd128};
      wr_coal_cfg <= 32'd16;
      rd_cfg      <= 32'(MAX_TAGS);
      cpl_to      <= 32'd100000;
      fc_cfg2     <= 32'd32;
      fatal_mask  <= 32'h0000_1028;
      err_r       <= '0;
      for (int i = 0; i < NC; i++) begin cnt_live[i] <= '0; cnt_shadow[i] <= '0; end
    end else begin
      csr_ack   <= csr_req;
      if (csr_req && !csr_we) csr_rdata <= rd_mux;
      ctrl_cmd  <= '0;

      // ---- writes ----
      if (wr) begin
        case (wa)
          12'h004: begin
            link_en_r <= csr_wdata[0];
            ctrl_cmd  <= csr_wdata[4:0] & 5'b11110;
          end
          12'h00C: opt_r <= csr_wdata[5:0];
          12'h010: ack_cfg <= csr_wdata;
          12'h014: begin
            replay_cfg[31:16] <= csr_wdata[31:16];
            replay_cfg[15:8]  <= 8'd0;
            replay_cfg[7:0]   <= (csr_wdata[7:0] == 8'd0) ? 8'd1 :
                                 (csr_wdata[7:0] > 8'(MAX_REPLAY_TLPS)) ? 8'(MAX_REPLAY_TLPS) : csr_wdata[7:0];
          end
          12'h018: fc_cfg <= csr_wdata;
          12'h01C: wr_coal_cfg <= {16'b0, csr_wdata[15:0]};
          12'h020: rd_cfg <= {24'b0, (csr_wdata[7:0] == 8'd0) ? 8'd1 :
                                      (csr_wdata[7:0] > 8'(MAX_TAGS)) ? 8'(MAX_TAGS) : csr_wdata[7:0]};
          12'h024: cpl_to <= csr_wdata;
          12'h02C: fatal_mask <= csr_wdata;
          12'h030: fc_cfg2 <= {16'b0, csr_wdata[15:0]};
          default: ;
        endcase
      end

      // ---- error status : W1C, set wins ----
      begin
        logic [15:0] nxt;
        nxt = err_r;
        if (wr && wa == 12'h028) nxt = nxt & ~csr_wdata[15:0];
        err_r <= nxt | err_set;
      end

      // ---- counters ----
      for (int i = 0; i < NC; i++) begin
        if (ctrl_cmd[2]) begin cnt_live[i] <= '0; cnt_shadow[i] <= '0; end
        else begin
          case (i)
            0:  cnt_live[i] <= cnt_live[i] + (dl_up ? 32'd1 : 32'd0);
            21: cnt_live[i] <= cnt_live[i] + (rd_lat_valid ? 32'd1 : 32'd0);
            22: cnt_live[i] <= cnt_live[i] + (rd_lat_valid ? rd_lat_val : 32'd0);
            23: if (rd_lat_valid && rd_lat_val > cnt_live[i]) cnt_live[i] <= rd_lat_val;
            default: cnt_live[i] <= cnt_live[i] + {20'b0, cnt_inc[i]};
          endcase
          if (ctrl_cmd[3]) begin
            // snapshot : live (before this cycle's increment) -> shadow
            cnt_shadow[i] <= cnt_live[i];
          end
        end
      end
    end
  end
endmodule
