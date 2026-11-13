# Results (spec 14.1, 8.5, 12, 13)

All numbers are from the final RTL (v5: v4 plus the fixes B-F1, B-S5, B-R1, B-W1 of `bug_log.md`) and were regenerated on
2026-11-12. Every table names the report or CSV it comes from.

## 1. Synthesis and PPA (spec 14.1)

Flow: Synopsys Design Compiler V-2023.12-SP5, `compile_ultra -no_autoungroup`, FreePDK45 `gscl45nm` (OSU standard cells,
typical corner). Clock 4.0 ns (250 MHz), setup uncertainty 0.20 ns, clock transition 0.10 ns, I/O delays 20 % of the period on
every AXI, link and CSR port, BUFX2 input drive, 0.05 pF output load (`syn/constraints.sdc`, `syn/dc_run.tcl`).
Gate equivalents (GE) use NAND2X1 = 1.8772 um^2.

**Without memories** (the five `ram_sdp` RAMs are black boxes; the TL TX queues are flip-flops and are counted):

| Build | WNS (setup) | TNS | Violating paths | Area (um^2) | Area (GE) | Report |
|---|---|---|---|---|---|---|
| OPT_EN=1 (all four optimizations) | **0.00 ns (met)** | **0.00** | **0** | 957,449 | 510 k | `syn/reports/qor_v5_opt1_bb.rpt` |
| OPT_EN=0 (optimization blocks removed) | 0.00 ns (met) | 0.00 | 0 | 923,424 | 492 k | `syn_opt0/reports/qor_v5_opt0_bb.rpt` |

* **Optimization cost:** +34,025 um^2 (+3.7 %, 18.1 k GE) for OP1-OP4 at the same 250 MHz; timing is met in both builds.
* **Cost of this round's bug fixes:** +959 um^2 against v4 for OPT_EN=1 (+241 um^2 for OPT_EN=0).
* **Critical path (OPT_EN=1):** 3.70 ns, from `u_ttx/u_qp/rp_reg[2]` (TL P-queue read pointer) through the TL-queue read mux
  and the DLL TX framing logic to `u_dtx/lnk_data_reg[85]` (`syn/reports/timing_v5_opt1_bb.rpt`).
* **Area by block (OPT_EN=1)**, from `syn/reports/area_hier_nosplit_v5_opt1_bb.rpt`: tl_tx 33.9 % (the TX queues are flops),
  axi_slv_bridge 26.7 %, mst_wr_path 10.0 %, dll_tx 8.1 %, tl_rx 7.9 %, mst_rd_path 5.3 %, csr_regs 3.8 %, dll_rx 3.7 %,
  dll_fc 0.5 %, cfg_space 0.1 %.

**Hold (gap closed in this round).** The setup runs apply the 0.20 ns uncertainty to hold as well and do not fix hold, so their
QoR lists 49,116 hold "violations" (WHS -0.14 ns) on flop-to-flop paths without logic. `syn/dc_hold.tcl` reads the netlist
back, sets the hold uncertainty to the skew budget (0.05 ns), reports hold, then runs `set_fix_hold` +
`compile -incremental_mapping -only_hold_time`:

| Build | Hold, 0.05 ns uncertainty, before fix | After set_fix_hold | Setup after fix | Area after fix | Report |
|---|---|---|---|---|---|
| OPT_EN=1 | 0 violations | 0 violations, worst hold slack +0.01 ns | met (0.00) | 962,811 um^2 (+0.6 %) | `syn/reports/qor_holdfixed_v5_opt1_bb.rpt` |
| OPT_EN=0 | 0 violations | 0 violations | met (0.00) | 930,195 um^2 (+0.7 %) | `syn_opt0/reports/qor_holdfixed_v5_opt0_bb.rpt` |

The clock is ideal (no clock tree); a real hold sign-off needs the post-CTS clock from place and route, which is not available
here (no Innovus / IC Compiler II license on this host).

**With memories** (`MEMS=full`, the five RAMs synthesized to flip-flops): **timing is not met.** Full numbers are in
section 4.

### Power (spec 14.1)
Activity: node A over the W1a measurement window (4,000 x 128-byte writes A->B; first AW at A to last W at B), recorded as a VCD
of `tb_uvm_top.dutA` (`+PWR_VCD`), converted with `vcd2saif` (PrimeTime K-2015.12) and annotated on the OPT_EN=1 netlist.
Power engine: Design Compiler `report_power` (Power Compiler, averaged). PrimeTime PX was the planned tool; both installed
versions (Q-2019.12, K-2015.12) crash in `link_design` on this host for the full netlist (they link every block on its own);
the attempt is kept in `power/pt_power.tcl` and the workarounds in `power/` (`bb_stubs.py`, `ptlibs/`).

| Mode (OPT_CTRL) | CORRUPT_BIT rate | Total | Dynamic | Leakage | Clock network | Window | Energy / delivered byte |
|---|---|---|---|---|---|---|---|
| baseline (0x00) | 0 ppm | 560.1 mW | 554.5 mW | 5.6 mW | 530.6 mW | 56,064 cycles | 245 pJ/B |
| baseline (0x00) | 1,000 ppm | 560.1 mW | 554.6 mW | 5.6 mW | 530.6 mW | 56,077 cycles | 245 pJ/B |
| all on (0x3F) | 0 ppm | 556.2 mW | 550.6 mW | 5.6 mW | 530.6 mW | 68,096 cycles | 296 pJ/B |
| all on (0x3F) | 1,000 ppm | 556.2 mW | 550.6 mW | 5.6 mW | 530.6 mW | 68,096 cycles | 296 pJ/B |

(`results/csv/power.csv`, `power/reports/power_w1a_*.rpt`, flow `power/run_power.sh`)

* 95 % of the power is the clock pins of the ~50 k flip-flops: the design has no clock gating, so power is almost independent of
  traffic and energy per byte follows throughput. With all optimizations on W1a runs slower (limitation L2, section 3), so it
  costs more energy per byte. Clock gating of the TX queues and the replay / RX buffers is the obvious power fix.
* Annotation: 100 % of ports and 4.6 % of nets are annotated by name (register and port nets); the rest is propagated by the
  tool. Unpacked arrays (queue storage) are not in a VCD. Treat the dynamic part as an estimate; the clock part does not depend on it.

## 2. Verification (spec 10 - 13)

| Item | Result | Where |
|---|---|---|
| UVM regression | **166 / 166 pass** (33 tests x 5 seeds + seq_wrap), 0 UVM errors, 0 assertion failures | `scripts/run_uvm_regr.sh 5 60`, `results/uvm/regr_uvm.log` |
| Unit benches (10.6) | **5 / 5 pass**: tb_crc (2,716 checks vs Python zlib), tb_credit_gate (287,432), tb_replay_buf (19,569, 75 pointer wraps), tb_wr_coalescer (2,215 TLPs), tb_cpl_split (800 requests, 1,432 completions) | `scripts/run_unit.sh`, `results/unit/` |
| Mutation (13.3) | **35 / 35 killed** on the final RTL (the 34 of the spec + M35, a DL-18 checker-validation mutant); unmutated design passes all 31 tests of the list | `results/mut_v5_summary.txt`, `results/mutation_report.md` |
| Performance (perf level) | 351 / 351 runs pass, 3 seeds per point | `scripts/run_perf.sh`, `results/csv/perf.csv` |

**Coverage**, merged over the final regression (urg, `results/coverage/urg/`). Code coverage is now scoped to the RTL of both
nodes (`tb/uvm/cov/cm_hier.cfg`); the "before" column was measured on the whole testbench hierarchy and is only indicative.

| Metric | Before this round | Final | Spec 12.2 target |
|---|---|---|---|
| Line | 95.1 % | **98.5 %** | >= 95 % - met |
| Branch | 86.0 % | **92.0 %** | >= 95 % |
| Condition | 81.0 % | **88.6 %** | >= 90 % on arbiter / credit gate / ack scheduler / replay engine |
| FSM (transitions) | 65.6 % | **83.3 %** | 100 % or excluded |
| Toggle | 76.8 % | **81.8 %** | >= 90 % datapath, 100 % control registers |
| Assertions (attempted / covered) | 84.5 % | **94.6 %** | all hit |
| Functional covergroups | 75.8 % | **83.0 %** | 100 % or excluded |

Remaining holes and their reasons are listed in `verif_plan.md` ("Coverage: holes left, with reasons"). The targets that are
not met are stated as such; they are not claimed.

## 3. Optimizations OP1 - OP4 (spec 8.5)

Each workload is run in three modes: baseline (OPT_CTRL = 0x00), the optimization's own switches only, and everything on (0x3F).
Mean over seeds 1-3 (min / max in the plots, `results/plots/`). Full tables: `results/csv/summary.md`.

| Opt | Workload | Result (own switches only) | With everything on |
|---|---|---|---|
| OP1 (0x07) | W1b 16 B writes, 50,000 ppm, lat 8 | goodput 1.40 -> 2.25 B/cycle (**+61 %**) | 2.61 B/cycle (+86 %) |
| OP1 | W1c tail loss, lat 8 / 32 | recovery 1,116 -> **184** cycles / 1,140 -> **302** cycles | 210 / 322 |
| OP1 | W1a/W1b, lat 32, <= 10,000 ppm | **-3 % to -12 %** (see L3) | W1b +54 %, W1a -16 % (see L2) |
| OP2 (0x08) | W2, UpdateFC period 2,000 cycles | link utilization 0.050 -> **0.430**, credit stalls 86,294 -> **0** cycles | 0.274 (L2) |
| OP3 (0x10) | W3a 4 B sequential writes | link efficiency 12.4 % -> **80.7 %** (baseline matches the analytic 12.5 / 25 / 33.3 / 50 / 66.7 / 80 % table within 0.3 %) | same |
| OP4 (0x20) | W4, 20 % slow reads, 8 IDs | read latency p50 380 -> **135** cycles, p99 427 -> 422; single-ID control run unchanged (380 / 380) | same |

Findings (also in `bug_log.md`):
* **L2** - the write coalescer has a single open-TLP buffer: a chunk that cannot be merged waits for the previous TLP to be
  serialized. Writes that are already large (W1a, 128 B) or not contiguous (W2) get slower with WR_COALESCE on (W1a -17 %),
  which fails the spec's W3b "no slower than baseline" criterion. A second open-TLP buffer would remove it.
* **L3** - Ack coalescing with an 8-TLP replay window at LINK_LATENCY 32: fewer Acks make the window cover a longer round trip,
  so with few errors OP1 alone costs 3-12 %; it pays off under heavy errors and for tail losses.

## 4. Synthesis with memories (`syn_mem/`)
`MEMS=full` turns the five RAMs (replay buffer 4 KiB, RX ring 1 KiB, completion buffer 8 KiB, read buffer 8 KiB, P buffer 4.1 KiB)
into flip-flops, because the library has no RAM macros. Same constraints as section 1, OPT_EN=1, final RTL (v5). The run
finished on 2026-11-12 after 2.5 hours (`syn_mem/dc_v5_opt1_mem.log`).

| | Without memories (section 1) | With memories | Report |
|---|---|---|---|
| WNS (setup) | 0.00 ns (met) | **-0.32 ns (violated)** | `syn_mem/reports/qor_v5_opt1_mem.rpt` |
| TNS | 0.00 | -9,716.68 ns | same |
| Violating paths | 0 | 44,682 | same |
| Area | 957,449 um^2 | 4,139,758 um^2 (2.2 M GE) | same |
| Sequential cells | about 50 k | 256,277 | same |
| Leaf cells | | 1,072,639 | same |

* **Timing is not met at 250 MHz with the memories as flip-flops.** The critical path starts at the same register as in the
  black-box build (`u_ttx/u_qp/rp_reg[5]`, TL P-queue read pointer) and now ends in the write-data input of a replay-buffer
  storage flop (`u_dtx/u_replay_ram/mem_reg[1][98]`): arrival 2.86 ns against 2.54 ns required
  (`syn_mem/reports/timing_v5_opt1_mem.rpt`). The violating paths are the write paths into the flop arrays.
* The 250 MHz result of section 1 therefore holds for the logic with the RAMs as black boxes, as the spec allows when the
  memories are stated. A real implementation would use compiled SRAM macros, which this flow does not have.
* Hold: the QoR lists 255,769 hold violations (WHS -0.14 ns) with the 0.20 ns setup uncertainty applied to hold, the same
  artefact as in section 1. `dc_hold.tcl` was not run on this netlist.
* Power: `report_power` on this netlist gives 1.24 W (leakage 20.4 mW) with the tool's default switching activity, not the
  simulation SAIF of section 1, so it is not comparable with the 560 mW figure
  (`syn_mem/reports/power_v5_opt1_mem.rpt`).
* The replay buffer alone is 536,452 um^2 (13.0 % of the total) and the RX ring 126,293 um^2 (3.1 %)
  (`syn_mem/reports/area_hier_nosplit_v5_opt1_mem.rpt`).
