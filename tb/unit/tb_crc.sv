// ============================================================================
// tb_crc : CRC-32 (LCRC) and CRC-16 (DLLP) of pcie_pkg against golden values
//   * crc32_byte over a byte stream  == Python zlib.crc32   (crc_vectors.svh)
//   * crc32_nb (16-byte beat, nb valid bytes) == crc32_byte byte by byte, every nb 1..16
//   * residue: CRC state after a frame followed by its own LCRC == CRC32_RESIDUE
//   * single-bit errors anywhere in the frame or in the LCRC are always detected
//   * crc16_dllp == bit-serial reference (spec 5.4)
// ============================================================================
module tb_crc;
  import pcie_pkg::*;
  `include "crc_vectors.svh"
  int errors = 0, checks = 0;

  function automatic bit [31:0] crc_stream(int off, int n);
    bit [31:0] c = 32'hFFFF_FFFF;
    for (int i = 0; i < n; i++) c = crc32_byte(c, v_bytes[off + i]);
    return ~c;
  endfunction

  initial begin
    automatic int off = 0;
    // 1. zlib golden values
    for (int v = 0; v < NV; v++) begin
      automatic bit [31:0] got = crc_stream(off, v_len[v]);
      checks++;
      if (got !== v_crc[v]) begin errors++; $display("ERROR crc32 vector %0d (len %0d): got %08h exp %08h", v, v_len[v], got, v_crc[v]); end
      off += v_len[v];
    end
    // 2. beat-wide CRC == byte-serial CRC for every byte count
    for (int t = 0; t < 2000; t++) begin
      automatic logic [127:0] d = {$urandom, $urandom, $urandom, $urandom};
      automatic bit [31:0] c0 = $urandom;
      automatic int nb = 1 + t % 16;
      automatic bit [31:0] ref_c = c0;
      for (int i = 0; i < nb; i++) ref_c = crc32_byte(ref_c, d[8*i +: 8]);
      checks++;
      if (crc32_nb(c0, d, nb) !== ref_c) begin errors++; $display("ERROR crc32_nb nb=%0d", nb); end
    end
    // 3. residue and 4. single-bit error detection on random frames
    for (int t = 0; t < 300; t++) begin
      automatic byte unsigned fr [$];
      automatic bit [31:0] l, c;
      automatic int n = 6 + $urandom_range(0, 270);
      for (int i = 0; i < n; i++) fr.push_back($urandom);
      c = 32'hFFFF_FFFF; foreach (fr[i]) c = crc32_byte(c, fr[i]); l = ~c;
      for (int k = 0; k < 4; k++) fr.push_back(l[8*k +: 8]);              // LCRC sent LSB byte first (5.2)
      c = 32'hFFFF_FFFF; foreach (fr[i]) c = crc32_byte(c, fr[i]);
      checks++;
      if (c !== CRC32_RESIDUE) begin errors++; $display("ERROR residue %08h exp %08h (n=%0d)", c, CRC32_RESIDUE, n); end
      begin
        automatic int bi = $urandom_range(0, 8 * fr.size() - 1);
        fr[bi / 8] ^= (8'h1 << (bi % 8));
        c = 32'hFFFF_FFFF; foreach (fr[i]) c = crc32_byte(c, fr[i]);
        checks++;
        if (c === CRC32_RESIDUE) begin errors++; $display("ERROR single-bit error at bit %0d not detected", bi); end
      end
    end
    // 5. CRC-16
    for (int v = 0; v < N16; v++) begin
      checks++;
      if (crc16_dllp(v16_d[v]) !== v16_c[v]) begin errors++; $display("ERROR crc16 %08h: got %04h exp %04h", v16_d[v], crc16_dllp(v16_d[v]), v16_c[v]); end
    end
    $display("tb_crc: %0d checks, %0d errors -> %s", checks, errors, errors == 0 ? "PASS" : "FAIL");
    $finish;
  end
endmodule
