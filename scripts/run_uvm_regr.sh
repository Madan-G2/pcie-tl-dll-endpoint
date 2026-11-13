#!/bin/bash
# UVM regression (spec 13.2): scripts/run_uvm_regr.sh [nseeds] [nops] [jobs]
#   builds both variants once, runs every test x seed in parallel, merges coverage (results/uvm/cov.vdb, urg report in
#   results/coverage/), log per run in results/uvm/regr/, summary in results/uvm/regr_uvm.log
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
NS=${1:-5}; NOPS=${2:-60}; JOBS=${3:-6}
source /apps/set_license.sh >/dev/null 2>&1
cd "$ROOT/results/uvm" || exit 1
[ -z "$NOBUILD" ] && { rm -rf cov.vdb; "$ROOT/tb/uvm/run_uvm.sh" build; "$ROOT/tb/uvm/run_uvm.sh" build_cpl; }
rm -rf regr; mkdir -p regr
L=regr/list.txt; : > $L
add() { echo "$*" >> $L; }      # name | runner | test | plusargs
for sd in $(seq 1 $NS); do
  S="+ntb_random_seed=$sd +NOPS=$NOPS"
  add "smoke_s$sd|run|pcie_smoke_test|$S"
  add "rand_base_s$sd|run|pcie_rand_test|$S"
  add "rand_bimodal_bp_s$sd|run|pcie_rand_test|$S +MEMPOL=2 +BP=20"
  add "rand_opt_s$sd|run|pcie_rand_opt_test|$S"
  add "small_wr_base_s$sd|run|pcie_small_wr_test|$S"
  add "small_wr_opt_s$sd|run|pcie_small_wr_test|$S +OPT=1"
  add "fi_1000ppm_s$sd|run|pcie_fi_rate_test|$S +PPM=1000"
  add "fi_20000ppm_s$sd|run|pcie_fi_rate_test|$S +PPM=20000"
  add "fi_opt_s$sd|run|pcie_fi_rate_test|$S +PPM=20000 +OPT=1"
  add "fi_directed_s$sd|run|pcie_fi_directed_test|$S"
  add "rd_pipe_base_s$sd|run|pcie_rd_pipe_test|$S"
  add "rd_pipe_opt_s$sd|run|pcie_rd_pipe_opt_test|$S"
  add "credit_starve_s$sd|run|pcie_credit_starve_test|$S"
  add "credit_exhaust_s$sd|run|pcie_credit_exhaust_test|$S"
  add "err_s$sd|run|pcie_err_test|$S"
  add "axi_unsup_b_s$sd|run|pcie_axi_unsup_b_test|$S"
  add "linkdown_s$sd|run|pcie_linkdown_test|$S"
  add "lcrc_bits_s$sd|run|pcie_lcrc_bits_test|$S"
  add "replay_num_s$sd|run|pcie_replay_num_test|$S"
  add "rollover_s$sd|run|pcie_rollover_test|$S"
  add "replay_full_s$sd|run|pcie_replay_full_test|$S"
  add "nak_ack_s$sd|run|pcie_nak_ack_test|$S"
  add "coalesce_edge_s$sd|run|pcie_coalesce_edge_test|$S"
  add "coal_flush_s$sd|run|pcie_coal_flush_test|$S"
  add "bar_edge_s$sd|run|pcie_bar_edge_test|$S"
  add "partial_be_s$sd|run|pcie_partial_be_test|$S"
  add "unexpected_cpl_s$sd|run|pcie_unexpected_cpl_test|$S"
  add "poisoned_s$sd|run|pcie_poisoned_test|$S"
  add "malformed_all_s$sd|run|pcie_malformed_all_test|$S"
  add "ral_s$sd|run|pcie_ral_test|$S"
  add "axi_mst_err_s$sd|run|pcie_axi_mst_err_test|$S"
  add "reset_events_s$sd|run|pcie_reset_events_test|$S"
  add "cpl_finite_s$sd|run_cpl|pcie_cpl_finite_test|$S"
done
add "seq_wrap|run|pcie_seq_wrap_test|+ntb_random_seed=1"
one() {
  IFS='|' read -r name runner test args <<< "$1"
  timeout -s KILL 900 "$ROOT/tb/uvm/run_uvm.sh" $runner $test $args > regr/$name.log 2>&1
  out=regr/$name.log
  if grep -q "RESULT: PASS" $out && ! grep -qE '^Error: "|failed at [0-9]+ps' $out; then echo "PASS $name"
  else echo "FAIL $name : $(grep -hE 'UVM_ERROR|UVM_FATAL|^Error: "|failed at [0-9]+ps' $out | head -2 | sed 's#.*/tb/uvm/##' | cut -c1-200 | tr '\n' ' ')"; fi
}
export -f one; export ROOT
T0=$(date +%s)
cat $L | xargs -P $JOBS -I{} bash -c 'one "$@"' _ {} | sort > regr_uvm.log
pass=$(grep -c "^PASS" regr_uvm.log); fail=$(grep -c "^FAIL" regr_uvm.log)
echo "UVM REGRESSION: pass=$pass fail=$fail  ($(wc -l < $L) runs, $NS seeds, $(( $(date +%s) - T0 )) s)" | tee -a regr_uvm.log
mkdir -p ../coverage
urg -dir cov.vdb -report ../coverage/urg -format both > ../coverage/urg.log 2>&1     # relative: urg -report cannot take a path with a space
sed -n '/Total Coverage Summary/,/^$/p;/Total Groups Coverage Summary/,/^$/p' ../coverage/urg/dashboard.txt | tee -a regr_uvm.log
