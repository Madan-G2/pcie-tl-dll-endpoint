# ============================================================================
# Averaged power of node A with Design Compiler (Power Compiler engine), used because PrimeTime (Q-2019.12 and
# K-2015.12) crashes in link_design on this host for the full netlist (see README of power/).
#   env NETLIST : gate netlist (syn/reports/pcie_ep_top_<tag>.v)    env SAIF : backward SAIF of tb_uvm_top/dutA
#   env TAG     : report suffix
# Same library, same SDC (250 MHz) and the same activity file PrimeTime PX would read.
# ============================================================================
set LIBDB /apps/cadence/gpdk/FreePDK45/osu_soc/lib/files/gscl45nm.db
set_app_var target_library $LIBDB
set_app_var link_library   [list * $LIBDB]
file mkdir reports
read_verilog [list $::env(NETLIST)]   ;# list: the path may contain a space
current_design [lindex [get_object_name [get_designs pcie_ep_top_OPT_EN*]] 0]
link
source ../syn/constraints.sdc
read_saif -auto_map_names -input $::env(SAIF) -instance_name tb_uvm_top/dutA -verbose > reports/saif_$::env(TAG).log
report_saif -hier -missing                > reports/saif_annot_$::env(TAG).rpt
report_power                              > reports/power_$::env(TAG).rpt
report_power -hier -hier_level 1          > reports/power_hier_$::env(TAG).rpt
puts "DC POWER DONE $::env(TAG)"
exit
