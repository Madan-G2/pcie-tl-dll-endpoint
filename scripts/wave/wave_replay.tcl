# DVE session for the README waveform: corrupted TLP -> Nak -> replay (pcie_replay_num_test, seed 1)
gui_open_db -design V1 -file wave_replay.vpd
set w [gui_open_window Wave]
set tl [gui_get_window_ids -type TopLevel]
foreach t $tl { catch {gui_show_window -window $t -show_state maximized} }
gui_sg_create AXI_A
gui_sg_addsignal -group AXI_A { tb_uvm_top.clk tb_uvm_top.dutA.s_axi_awvalid tb_uvm_top.dutA.s_axi_awready tb_uvm_top.dutA.s_axi_wvalid tb_uvm_top.dutA.s_axi_wlast }
gui_sg_create Link_A_TX
gui_sg_addsignal -group Link_A_TX { tb_uvm_top.dutA.lnk_tx_valid tb_uvm_top.dutA.lnk_tx_sop tb_uvm_top.dutA.lnk_tx_eop tb_uvm_top.dutA.lnk_tx_dllp tb_uvm_top.dutA.u_dtx.dbg_next_tx_seq }
gui_sg_create DLL_RX_B
gui_sg_addsignal -group DLL_RX_B { tb_uvm_top.dutB.lnk_rx_valid tb_uvm_top.dutB.lnk_rx_sop tb_uvm_top.dutB.lnk_rx_eop tb_uvm_top.dutB.u_drx.ev_lcrc_err tb_uvm_top.dutB.u_drx.nak_req tb_uvm_top.dutB.u_drx.ack_req tb_uvm_top.dutB.u_drx.rx_seq_m1 }
gui_sg_create Replay_A
gui_sg_addsignal -group Replay_A { tb_uvm_top.dutA.u_dtx.ev_nak_rx tb_uvm_top.dutA.u_dtx.ev_replay_start tb_uvm_top.dutA.u_dtx.replay_active tb_uvm_top.dutA.u_dtx.ev_beat_replay tb_uvm_top.dutA.u_dtx.dbg_ackd_seq tb_uvm_top.dutA.u_dtx.dbg_replay_num }
gui_list_add_group -id Wave.1 {Replay_A DLL_RX_B Link_A_TX AXI_A}
gui_wv_zoom_timerange -id Wave.1 3440000 3820000
set f [open dve_ids.txt w]; puts $f "wave=$w toplevels=$tl"; close $f
