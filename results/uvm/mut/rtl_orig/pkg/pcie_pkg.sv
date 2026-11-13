// ============================================================================
// pcie_pkg : constants, CRC functions (single golden definition), helpers
// Spec refs: 1.4 (conventions), 5.2 (CRC-32 LCRC), 5.4 (CRC-16 DLLP), TL-7
// ============================================================================
package pcie_pkg;

  // ---- fixed constants ------------------------------------------------------
  localparam int DATA_W        = 128;
  localparam int RCB_BYTES     = 64;
  localparam int FC_INIT_PERIOD = 32;      // 6.1
  localparam logic [31:0] CRC32_RESIDUE = 32'hDEBB20E3; // crc state after a frame incl. its own LCRC

  // ---- TLP Fmt/Type ---------------------------------------------------------
  localparam logic [7:0] FT_MRD32  = {3'b000, 5'b00000};
  localparam logic [7:0] FT_MRD64  = {3'b001, 5'b00000};
  localparam logic [7:0] FT_MWR32  = {3'b010, 5'b00000};
  localparam logic [7:0] FT_MWR64  = {3'b011, 5'b00000};
  localparam logic [7:0] FT_CFGRD0 = {3'b000, 5'b00100};
  localparam logic [7:0] FT_CFGWR0 = {3'b010, 5'b00100};
  localparam logic [7:0] FT_CPL    = {3'b000, 5'b01010};
  localparam logic [7:0] FT_CPLD   = {3'b010, 5'b01010};

  // ---- completion status ----------------------------------------------------
  localparam logic [2:0] CPL_SC = 3'b000;
  localparam logic [2:0] CPL_UR = 3'b001;
  localparam logic [2:0] CPL_CA = 3'b100;

  // ---- DLLP types (5.4) -----------------------------------------------------
  localparam logic [7:0] DLLP_ACK      = 8'h00;
  localparam logic [7:0] DLLP_NAK      = 8'h10;
  localparam logic [7:0] DLLP_INITFC1_P   = 8'h40;
  localparam logic [7:0] DLLP_INITFC1_NP  = 8'h50;
  localparam logic [7:0] DLLP_INITFC1_CPL = 8'h60;
  localparam logic [7:0] DLLP_INITFC2_P   = 8'hC0;
  localparam logic [7:0] DLLP_INITFC2_NP  = 8'hD0;
  localparam logic [7:0] DLLP_INITFC2_CPL = 8'hE0;
  localparam logic [7:0] DLLP_UPDFC_P     = 8'h80;
  localparam logic [7:0] DLLP_UPDFC_NP    = 8'h90;
  localparam logic [7:0] DLLP_UPDFC_CPL   = 8'hA0;

  // ---- err_status bit positions (9.3) --------------------------------------
  localparam int ERR_LCRC_ERR_RX       = 0;
  localparam int ERR_SEQ_ERR_RX        = 1;
  localparam int ERR_DLLP_CRC_ERR      = 2;
  localparam int ERR_REPLAY_NUM_ROLLOVER = 3;
  localparam int ERR_REPLAY_TIMEOUT    = 4;
  localparam int ERR_MALFORMED         = 5;
  localparam int ERR_UR_SENT           = 6;
  localparam int ERR_POISONED          = 7;
  localparam int ERR_CPL_TIMEOUT       = 8;
  localparam int ERR_UNEXPECTED_CPL    = 9;
  localparam int ERR_CPL_ERR_RX        = 10;
  localparam int ERR_AXI_UNSUP         = 11;
  localparam int ERR_FC_VIOLATION      = 12;
  localparam int ERR_DLLP_UNEXPECTED   = 13;
  localparam int ERR_UR_POSTED         = 14;
  localparam int ERR_AXI_MST_ERR       = 15;

  // ---- helpers --------------------------------------------------------------
  // logical DW <-> bus DW (1.4)
  function automatic logic [31:0] bsw32(input logic [31:0] x);
    return {x[7:0], x[15:8], x[23:16], x[31:24]};
  endfunction

  // CRC-32 (IEEE 802.3, reflected), one byte (spec 5.2, identical to zlib.crc32 core)
  function automatic logic [31:0] crc32_byte(input logic [31:0] c_in, input logic [7:0] b);
    logic [31:0] c;
    c = c_in ^ {24'b0, b};
    for (int i = 0; i < 8; i++)
      c = c[0] ? ((c >> 1) ^ 32'hEDB88320) : (c >> 1);
    return c;
  endfunction

  // CRC-32 over the first nb bytes (wire order, byte 0 first) of a 128-bit beat
  function automatic logic [31:0] crc32_nb(input logic [31:0] c_in, input logic [127:0] d, input int nb);
    logic [31:0] c;
    c = c_in;
    for (int i = 0; i < 16; i++)
      if (i < nb) c = crc32_byte(c, d[8*i +: 8]);
    return c;
  endfunction

  // CRC-16 of a DLLP (spec 5.4); d = {p3,p2,p1,type}
  function automatic logic [15:0] crc16_dllp(input logic [31:0] d);
    logic [15:0] c;
    logic fb;
    c = 16'hFFFF;
    for (int i = 0; i < 4; i++)
      for (int b = 0; b < 8; b++) begin
        fb = c[15] ^ d[8*i + b];
        c  = {c[14:0], 1'b0};
        if (fb) c = c ^ 16'h100B;
      end
    return ~c;
  endfunction

  // 16-bit wrap-around "a is older than b" (a enqueued before b)
  function automatic logic age_lt(input logic [15:0] a, input logic [15:0] b);
    logic [15:0] d;
    d = a - b;
    return d[15];
  endfunction

  // TL-7 credit check, N=8 (header) or N=12 (data). returns 1 if allowed
  function automatic logic cr_ok8(input logic [7:0] lim, input logic [7:0] cc, input logic [7:0] need);
    logic [7:0] d;
    d = lim - (cc + need);
    return (d <= 8'd128);
  endfunction
  function automatic logic cr_ok12(input logic [11:0] lim, input logic [11:0] cc, input logic [11:0] need);
    logic [11:0] d;
    d = lim - (cc + need);
    return (d <= 12'd2048);
  endfunction

  // number of set bits in a 4-bit vector
  function automatic logic [2:0] pop4(input logic [3:0] v);
    return {2'b0, v[0]} + {2'b0, v[1]} + {2'b0, v[2]} + {2'b0, v[3]};
  endfunction

  // contiguous-ones checks for TL-5
  function automatic logic fbe_ok_multi(input logic [3:0] b);
    return (b == 4'b1111) || (b == 4'b1110) || (b == 4'b1100) || (b == 4'b1000);
  endfunction
  function automatic logic lbe_ok_multi(input logic [3:0] b);
    return (b == 4'b1111) || (b == 4'b0111) || (b == 4'b0011) || (b == 4'b0001);
  endfunction
  function automatic logic be_contig(input logic [3:0] b);
    return (b == 4'b0001) || (b == 4'b0010) || (b == 4'b0100) || (b == 4'b1000) ||
           (b == 4'b0011) || (b == 4'b0110) || (b == 4'b1100) ||
           (b == 4'b0111) || (b == 4'b1110) || (b == 4'b1111);
  endfunction

  // number of set bits in a 16-bit keep (contiguous from 0): number of valid bytes
  function automatic logic [4:0] keep_bytes(input logic [15:0] k);
    logic [4:0] n;
    n = 0;
    for (int i = 0; i < 16; i++) n = n + {4'b0, k[i]};
    return n;
  endfunction

  // frame beats from TLP header DW0 (logical, i.e. bswapped) : 5.2
  function automatic logic [4:0] frame_beats_from_hdr(input logic [2:0] fmt, input logic [9:0] len);
    logic [12:0] bytes;
    logic [10:0] pl;
    pl    = fmt[1] ? {1'b0, len} : 11'd0;
    bytes = 13'd2 + (fmt[0] ? 13'd16 : 13'd12) + {pl, 2'b00} + 13'd4;
    return (bytes + 13'd15) >> 4;
  endfunction

endpackage
