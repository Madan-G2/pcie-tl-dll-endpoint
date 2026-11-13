# Mutant list (spec 13.3)

Source of truth: `mutation/mutants.py` (table and one-time instrumentation of the RTL).  Each mutant is enabled with `+define+MUT_nn`.

| ID | Block | Mutation | Note |
|---|---|---|---|
| M01 | dll_rx | LCRC compare ignores bit 31 |  |
| M02 | dll_tx | LCRC init value 0 instead of all ones |  |
| M03 | dll_rx | NEXT_RCV_SEQ wraps at 2048 |  |
| M04 | dll_rx | Duplicate frame causes Nak instead of Ack |  |
| M05 | dll_rx | Frame with sequence ahead is accepted |  |
| M06 | dll_tx | Nak carries NEXT_RCV_SEQ instead of minus 1 |  |
| M07 | dll_tx | Replay starts at the newest frame |  |
| M08 | dll_tx | Replay drops the last outstanding frame |  |
| M09 | dll_tx | REPLAY_NUM not reset on Ack progress |  |
| M10 | dll_tx | Rollover check one replay early (2-bit counter: at 2 instead of 3) | adapted: the replay counter is 2 bits, so "4 instead of 3" is built as "rollover at the 3rd replay instead of the 4th" |
| M11 | dll_tx | Replay timer not restarted on Ack progress |  |
| M12 | dll_tx | Ack frees one frame too many |  |
| M13 | replay_buf (dll_tx wptr) | Write pointer does not wrap at the end | the replay RAM pointer is in dll_tx; built as "pointer sticks at the last beat instead of wrapping" |
| M14 | dll_tx | DLLP CRC wrong on TX only | built as an XOR of the TX CRC-16 (same effect as a wrong polynomial) |
| M15 | credit_gate | Check uses < instead of <= at the limit (last credit can never be consumed) | the literal `d < 128` boundary is unreachable (credits <= 127 / 2047); built as "the last credit can never be consumed" |
| M16 | credit_gate | Header consumed counter is 9 bits (compare done in 9 bits) | built as a 9-bit subtraction against the 8-bit limit (breaks at counter wrap) |
| M17 | fc_rx_alloc (tl_rx) | Released credits not added to allocated | fc_rx_alloc is inside tl_rx |
| M18 | tl_rx | Credits of a malformed TLP not released |  |
| M19 | tl_tx | Data credits use floor(Length/4) |  |
| M20 | cpl_gen (mst_rd_path) | Completion split ignores RCB alignment | cpl_gen is inside mst_rd_path / tl_rx |
| M21 | cpl_gen | Byte Count ignores first-DW byte enables |  |
| M22 | cpl_gen | Lower Address uses bits [5:0] |  |
| M23 | rd_tag_table | Tag freed on the first completion |  |
| M24 | tx_arb (tl_tx) | NP allowed to pass an older P |  |
| M25 | wr_coalescer | Merges across a 4 KiB boundary |  |
| M26 | wr_coalescer | Merges when the new First BE is partial |  |
| M27 | wr_coalescer | Timeout flush never fires for a single chunk |  |
| M28 | rd_return (slv_rd_path) | OOO mode breaks same-ID order |  |
| M29 | mst_rd_path | AR issued before an earlier write's B |  |
| M30 | cfg_space (BAR decode in tl_rx) | BAR window twice as large: address one past the end hits | BAR decode lives in tl_rx here, not cfg_space; built as a BAR window twice as large |
| M31 | cfg_space (MSE in tl_rx) | MSE ignored | MSE is checked in tl_rx here |
| M32 | cpl_gen (UR in tl_rx) | UR completion Byte Count 0 |  |
| M33 | ack_sched (dll_rx) | Timer never forces an Ack in coalescing mode |  |
| M34 | dlcmsm (dll_fc) | dl_up rises after InitFC1 only |  |
| M35 | dll_tx (replay engine) | Replay starts 12 cycles late (checker validation for DL-18) | built as an extra 12-cycle wait before the first replayed frame |
