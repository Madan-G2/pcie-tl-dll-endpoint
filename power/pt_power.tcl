# ============================================================================
# PrimeTime PX (PrimePower) averaged power of node A (spec 14.1)
#   env NETLIST : gate netlist from DC (syn/reports/pcie_ep_top_<tag>.v)
#   env VCD     : RTL VCD of tb_uvm_top.dutA over the W1a window (tb/uvm/tb_uvm_top.sv +PWR_VCD=)
#   env TAG     : report suffix
# The RTL VCD is mapped by name (read_vcd -rtl): register outputs and hierarchical port nets keep their names in the netlist
# (compile_ultra -no_autoungroup); everything not annotated is propagated by PrimeTime from the annotated nets.
# ============================================================================
set LIB     /apps/cadence/gpdk/FreePDK45/osu_soc/lib/files/gscl45nm.db
set_app_var search_path   [list . [file dirname $LIB]]
set_app_var link_path     [list * $LIB]
set_app_var power_enable_analysis true
set_app_var power_analysis_mode averaged
set_app_var link_create_black_boxes true          ;# ram_sdp left as black boxes ("without memories")
file mkdir reports

read_verilog [list $::env(NETLIST)]   ;# list: the path may contain a space
current_design [expr {[info exists ::env(TOP)] ? $::env(TOP) : "pcie_ep_top_OPT_EN1"}]   ;# DC names the elaborated top after its parameter
link_design
puts "PT linked [get_object_name [current_design]] : [sizeof_collection [get_cells -hier * -filter is_hierarchical==false]] leaf cells"
source ../syn/constraints.sdc
read_vcd -rtl -strip_path tb_uvm_top/dutA $::env(VCD)
report_switching_activity -list_not_annotated > reports/annot_$::env(TAG).rpt
report_switching_activity                     > reports/activity_$::env(TAG).rpt
update_power
report_power                                  > reports/power_$::env(TAG).rpt
report_power -hierarchy -levels 2             > reports/power_hier_$::env(TAG).rpt
exit
