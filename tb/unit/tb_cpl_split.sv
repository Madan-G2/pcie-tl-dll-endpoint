// ============================================================================
// tb_cpl_split : completion generation of mst_rd_path (cpl_gen, TL-21 .. TL-23)
//   against the reference cpl_split() (same algorithm as the UVM golden library):
//   random address / length / byte enables, MPS 128 and 256, every completion's
//   Length, Byte Count, Lower Address, Completer ID, Requester ID, Tag, status and
//   payload bytes (memory model returns a known pattern), plus RCB alignment of
//   every completion after the first, and a CA completion for an AXI SLVERR (TL-24).
// ============================================================================
module tb_cpl_split;
  import pcie_pkg::*;
  logic clk = 0, rst_n = 0;
  always #2 clk = ~clk;
  logic [2:0] mps_code;
  logic nd_valid, nd_ready, np_rel; logic [27:0] nd_off; logic [9:0] nd_len; logic [3:0] nd_fbe, nd_lbe;
  logic [15:0] nd_reqid; logic [7:0] nd_tag;
  logic [3:0] arid; logic [63:0] araddr; logic [7:0] arlen; logic [2:0] arsize; logic [1:0] arburst; logic arvalid, arready;
  logic [3:0] rid; logic [127:0] rdata; logic [1:0] rresp; logic rlast, rvalid, rready;
  logic cv, cr, cs, ce; logic [127:0] cd; logic [3:0] ck; logic ev_err, busy;

  mst_rd_path #(.MAX_CPLR_REQS(8), .DEVICE_ID(16'h0200)) dut (.clk, .rst_n, .dl_up(1'b1), .axi_base(64'h0), .mps_code,
    .nd_valid, .nd_ready, .nd_off, .nd_len, .nd_fbe, .nd_lbe, .nd_reqid, .nd_tag, .nd_snap(16'h0), .np_rel, .wr_b_cnt(16'h0),
    .m_arid(arid), .m_araddr(araddr), .m_arlen(arlen), .m_arsize(arsize), .m_arburst(arburst), .m_arvalid(arvalid), .m_arready(arready),
    .m_rid(rid), .m_rdata(rdata), .m_rresp(rresp), .m_rlast(rlast), .m_rvalid(rvalid), .m_rready(rready),
    .cpl0_valid(cv), .cpl0_ready(cr), .cpl0_data(cd), .cpl0_keep(ck), .cpl0_sop(cs), .cpl0_eop(ce), .ev_axi_mst_err(ev_err), .busy);

  function automatic byte unsigned mem(bit [63:0] a); return 8'(a[7:0] ^ a[15:8] ^ 8'h5A ^ (a[11:4] * 8'd3)); endfunction
  bit [63:0] err_lo = 64'h0F00_0000, err_hi = 64'h0F00_1000;     // reads in this window get SLVERR

  // ---- AXI slave (in order, random latency) -----------------------------------------
  typedef struct { bit [3:0] id; bit [63:0] a; int beats; } ar_t;
  ar_t arq [$];
  always @(posedge clk) if (rst_n && arvalid && arready) arq.push_back('{arid, araddr, int'(arlen) + 1});
  initial begin
    rvalid = 0; rlast = 0; rid = 0; rdata = 0; rresp = 0;
    forever begin
      @(negedge clk);
      if (arq.size() > 0) begin
        automatic ar_t r = arq.pop_front();
        repeat ($urandom_range(0, 6)) @(negedge clk);
        for (int b = 0; b < r.beats; b++) begin
          for (int i = 0; i < 16; i++) rdata[8*i +: 8] = mem(r.a + 16 * b + i);
          rid = r.id; rresp = (r.a >= err_lo && r.a < err_hi) ? 2'b10 : 2'b00; rlast = (b == r.beats - 1); rvalid = 1;
          @(negedge clk);
          rvalid = 0; rlast = 0;
          if ($urandom_range(0, 3) == 0) @(negedge clk);
        end
      end
    end
  end
  always @(negedge clk) arready = ($urandom_range(0, 3) != 0);
  // completion-queue back-pressure: short random gaps plus long stalls (finite Cpl credits waiting for UpdateFC)
  initial begin
    cr = 1;
    forever begin
      @(negedge clk);
      if ($urandom_range(0, 99) < 3) begin cr = 0; repeat ($urandom_range(20, 120)) @(negedge clk); end
      cr = ($urandom_range(0, 4) != 0);
    end
  end

  // ---- completion decoder ---------------------------------------------------------------
  typedef struct { bit [7:0] ft; int len; bit [15:0] cplid; bit [2:0] st; int bc; bit [15:0] rqid; bit [7:0] tag; int la; bit [31:0] dw[$]; } cpl_t;
  cpl_t got [$];
  bit [31:0] cur [$];
  always @(posedge clk) if (rst_n && cv && cr) begin
    if (cs) cur.delete();
    for (int d = 0; d < 4; d++) if (ck[d]) cur.push_back(cd[32*d +: 32]);
    if (ce) begin
      automatic cpl_t c;
      automatic bit [31:0] h0 = bsw32(cur[0]), h1 = bsw32(cur[1]), h2 = bsw32(cur[2]);
      c.ft = h0[31:24]; c.len = (h0[9:0] == 0) ? 1024 : h0[9:0]; c.cplid = h1[31:16]; c.st = h1[15:13]; c.bc = (h1[11:0] == 0) ? 4096 : h1[11:0];
      c.rqid = h2[31:16]; c.tag = h2[15:8]; c.la = h2[6:0];
      for (int i = 3; i < cur.size(); i++) c.dw.push_back(cur[i]);
      got.push_back(c);
    end
  end

  // ---- reference (TL-21, TL-22) : identical to cpl_split of the UVM library ----------
  typedef struct { int len_dw; int bytecnt; int lowaddr; longint cur; } chunk_t;
  function automatic void ref_split(bit [63:0] a, int len_dw, bit [3:0] fbe, bit [3:0] lbe, int mps, ref chunk_t q[$]);
    longint rs, re, c, ce, lim; int lo, trail; bit [3:0] l = (len_dw == 1) ? fbe : lbe;
    lo = fbe[0] ? 0 : fbe[1] ? 1 : fbe[2] ? 2 : 3;
    trail = l[3] ? 0 : l[2] ? 1 : l[1] ? 2 : 3;
    rs = longint'(a) + lo; re = longint'(a) + 4 * len_dw - trail;
    q.delete(); c = rs;
    while (c < re) begin
      chunk_t k;
      lim = ((c + mps) / RCB_BYTES) * RCB_BYTES;
      ce = (re < lim) ? re : lim;
      k.len_dw = int'(((ce + 3) / 4) - (c / 4)); k.bytecnt = int'(re - c); k.lowaddr = int'(c & 7'h7F); k.cur = c;
      q.push_back(k); c = ce;
    end
  endfunction

  int errors = 0, checks = 0, n_req = 0, n_cpl = 0, n_ca = 0;
  typedef struct { bit [63:0] a; int len; bit [3:0] fbe, lbe; bit [7:0] tag; bit err; int mps; } req_t;
  req_t reqs [$];

  task automatic issue(req_t r);
    @(negedge clk);
    nd_valid = 1; nd_off = 28'(r.a); nd_len = 10'(r.len); nd_fbe = r.fbe; nd_lbe = r.lbe; nd_reqid = 16'h0100; nd_tag = r.tag;
    do @(posedge clk); while (!nd_ready);
    #0.1 nd_valid = 0;
    reqs.push_back(r); n_req++;
  endtask

  task automatic check_all();
    int gi = 0;
    foreach (reqs[r]) begin
      chunk_t q[$];
      if (reqs[r].err) begin
        checks++;
        if (gi >= got.size() || got[gi].ft != FT_CPL || got[gi].st != CPL_CA || got[gi].tag != reqs[r].tag) begin
          errors++; $display("ERROR req %0d (SLVERR): expected one CA completion", r);
        end else n_ca++;
        gi++; continue;
      end
      ref_split(reqs[r].a, reqs[r].len, reqs[r].fbe, reqs[r].lbe, reqs[r].mps, q);
      foreach (q[k]) begin
        cpl_t c;
        checks++;
        if (gi >= got.size()) begin errors++; $display("ERROR req %0d: completion %0d missing", r, k); break; end
        c = got[gi++]; n_cpl++;
        if (c.ft != FT_CPLD || c.st != CPL_SC || c.cplid != 16'h0200 || c.rqid != 16'h0100 || c.tag != reqs[r].tag ||
            c.len != q[k].len_dw || c.bc != q[k].bytecnt || c.la != q[k].lowaddr) begin
          errors++;
          if (errors < 20) $display("ERROR req %0d a=%h len=%0d fbe=%h lbe=%h mps=%0d cpl %0d: len=%0d bc=%0d la=%h tag=%0d, exp len=%0d bc=%0d la=%h",
                   r, reqs[r].a, reqs[r].len, reqs[r].fbe, reqs[r].lbe, reqs[r].mps, k, c.len, c.bc, c.la, c.tag, q[k].len_dw, q[k].bytecnt, q[k].lowaddr);
        end
        if (k > 0 && (q[k].cur % RCB_BYTES) != 0) begin errors++; $display("ERROR reference chunk %0d not RCB aligned", k); end
        // payload: DWs from the DW that holds the first byte of this completion
        for (int d = 0; d < c.dw.size(); d++) begin
          bit [63:0] ba = ((q[k].cur / 4) * 4) + 4 * d;
          bit [31:0] e = {mem(ba + 3), mem(ba + 2), mem(ba + 1), mem(ba)};
          if (c.dw[d] !== e) begin errors++; if (errors < 20) $display("ERROR req %0d cpl %0d DW %0d: %h exp %h", r, k, d, c.dw[d], e); break; end
        end
        if (c.dw.size() != q[k].len_dw) begin errors++; $display("ERROR req %0d cpl %0d: %0d payload DWs, header says %0d", r, k, c.dw.size(), q[k].len_dw); end
      end
    end
    if (gi != got.size()) begin errors++; $display("ERROR %0d unexpected completions", got.size() - gi); end
    reqs.delete(); got.delete();
  endtask

  initial begin
    bit [3:0] fbes[] = '{4'hF, 4'hE, 4'hC, 4'h8}; bit [3:0] lbes[] = '{4'hF, 4'h7, 4'h3, 4'h1};
    bit [3:0] be1[]  = '{4'h1, 4'h2, 4'h4, 4'h8, 4'h3, 4'h6, 4'hC, 4'h7, 4'hE, 4'hF};
    nd_valid = 0; nd_off = 0; nd_len = 0; nd_fbe = 0; nd_lbe = 0; nd_reqid = 0; nd_tag = 0;
    repeat (4) @(posedge clk); rst_n = 1; repeat (2) @(posedge clk);
    for (int m = 0; m < 2; m++) begin
      mps_code = 3'(m);
      for (int i = 0; i < 400; i++) begin
        automatic req_t r;
        r.len = ($urandom_range(0, 3) == 0) ? $urandom_range(1, 4) : $urandom_range(1, 128);
        r.a = 64'h0010_0000 + (64'($urandom_range(0, 255)) << 12);
        // keep the request inside one 4 KiB page (TL-4)
        r.a[11:2] = 10'($urandom_range(0, 1024 - r.len));
        r.fbe = (r.len == 1) ? be1[$urandom_range(0, be1.size() - 1)] : fbes[$urandom_range(0, 3)];
        r.lbe = (r.len == 1) ? 4'h0 : lbes[$urandom_range(0, 3)];
        r.tag = 8'(i); r.mps = m ? 256 : 128; r.err = 0;
        if (i % 50 == 49) begin r.a = err_lo + 64'h40; r.len = 8; r.fbe = 4'hF; r.lbe = 4'hF; r.err = 1; end
        issue(r);
        if (i % 8 == 7) begin wait (!busy); repeat (3) @(posedge clk); end
      end
      wait (!busy); repeat (10) @(posedge clk);
      check_all();
    end
    $display("tb_cpl_split: %0d requests, %0d completions checked, %0d CA, %0d checks, %0d errors -> %s",
             n_req, n_cpl, n_ca, checks, errors, (errors == 0 && n_cpl > n_req) ? "PASS" : "FAIL");
    $finish;
  end
  initial begin #(40ms); $display("tb_cpl_split: TIMEOUT -> FAIL"); $finish; end
  // the completion FIFO must never be written when full (bug B-S5)
  always @(posedge clk) if (rst_n && dut.ob_push && !(dut.cpl0_valid && cr) && dut.of_cnt >= 8) begin
    errors++; $display("ERROR completion output FIFO written while full at %0t", $time);
  end
endmodule
