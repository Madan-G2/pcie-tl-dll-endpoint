#!/bin/bash
# power/run_power.sh : W1a activity (baseline / optimized, 0 / 1000 ppm) -> averaged power of node A
#   1. simulation with +PWR_VCD : VCD of tb_uvm_top.dutA over the W1a measurement window (first AW .. last W at B)
#   2. vcd2saif (PrimeTime K-2015.12 utility) -> backward SAIF
#   3. Design Compiler report_power on the synthesized netlist with that SAIF (dc_power.tcl)
#      PrimeTime PX itself (pt_power.tcl) crashes in link_design on this host for the full design - see README.md
#   -> power/reports/power_<tag>.rpt and results/csv/power.csv
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
N=${W1N:-4000}      # W1a is 20,000 writes; 4,000 keeps the VCDs small and is already in steady state
source /apps/set_license.sh >/dev/null 2>&1; export SNPSLMD_LICENSE_FILE=license-server
DC=/apps/synopsys/dc_compiler_2023_V/syn/V-2023.12-SP5/bin/dc_shell
V2S=/apps/synopsys/PRIME_TIME/bin/vcd2saif
NET=${NETLIST:-../syn/reports/pcie_ep_top_v5_opt1_bb.v}    # relative to power/: DC cannot read a path with a space
mkdir -p "$ROOT/power/saif" "$ROOT/results/csv"; cd "$ROOT/power"
CSV="$ROOT/results/csv/power.csv"; echo "tag,opt_ctrl,ppm,window_cycles,delivered_bytes,dynamic_mW,leakage_mW,total_mW,clock_network_mW,pJ_per_byte" > "$CSV"
for o in 0 3F; do for ppm in 0 1000; do
  tag=w1a_opt${o}_ppm${ppm}
  M="+OPT=0"; [ $o != 0 ] && M="+OPTMASK=$o"
  [ -s saif/$tag.saif ] || ( cd "$ROOT/results/uvm" && ./simv_uvm -no_save +UVM_TESTNAME=pcie_w1_test +W1N=$N +W1SIZE=128 +PPM=$ppm $M +ntb_random_seed=1 \
      +PWR_VCD="$ROOT/power/saif/$tag.vcd" +PERF_OUT="$ROOT/power/saif/$tag.csv" > "$ROOT/power/saif/$tag.log" 2>&1 )
  [ -s saif/$tag.saif ] || LD_LIBRARY_PATH="$ROOT/power/ptlibs" $V2S -input saif/$tag.vcd -output saif/$tag.saif -instance tb_uvm_top/dutA > saif/$tag.v2s.log 2>&1
  rm -f saif/$tag.vcd
  NETLIST="$NET" SAIF="saif/$tag.saif" TAG=$tag $DC -f dc_power.tcl > dc_$tag.log 2>&1
  row=$(tail -1 saif/$tag.csv)                                  # W1a,opt,ppm,lat,seed,n,size,win,bytes,...
  win=$(echo $row | cut -d, -f8); bytes=$(echo $row | cut -d, -f9)
  read dyn leak tot clk <<< $(awk '
     /^Total Dynamic Power/ {d=$5; du=$6}
     /^Cell Leakage Power/  {l=$5; lu=$6}
     /^clock_network/       {c=$5}
     /^Total  /             {t=$(NF-1); tu=$NF}
     END { f["W"]=1e3; f["mW"]=1; f["uW"]=1e-3; f["nW"]=1e-6; print d*f[du], l*f[lu], t*f[tu], c }' reports/power_$tag.rpt)
  epb=$(python3 -c "print('%.1f' % ($tot*1e-3 * $win*4e-9 / $bytes * 1e12))")
  echo "$tag,$o,$ppm,$win,$bytes,$dyn,$leak,$tot,$clk,$epb" | tee -a "$CSV"
done; done
