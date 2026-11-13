#!/bin/bash
# usage: mutation/run_mutation.sh <id|0> [tests-file]     (id 0 = unmutated baseline)
# Builds simv with +define+MUT_nn in results/mut/Mnn, runs the test list in order, stops at the first failure
# and writes results/mut/Mnn/verdict.txt : KILLED|SURVIVED|BUILD_FAIL, first failing test, first detector.
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
source /apps/set_license.sh >/dev/null 2>&1
ID=$(printf "%02d" $1)
D="$ROOT/results/mut/M$ID"; rm -rf "$D"; mkdir -p "$D"; cd "$D" || exit 1
DEF=""; [ "$1" != "0" ] && DEF="+define+MUT_$ID"
FILES=("$ROOT/rtl/pkg/pcie_pkg.sv")
for d in csr dll tl axi top; do for f in "$ROOT"/rtl/$d/*.sv; do FILES+=("$f"); done; done
vcs -sverilog -full64 -timescale=1ns/1ps -ntb_opts uvm-1.2 -debug_access+r $DEF +incdir+"$ROOT/tb/uvm" "${FILES[@]}" \
    "$ROOT/tb/link_model/link_pkg.sv" "$ROOT/tb/link_model/link_model.sv" "$ROOT/tb/uvm/pcie_ifs.sv" "$ROOT/tb/uvm/pcie_uvm_pkg.sv" \
    "$ROOT/tb/uvm/sva/pcie_checker.sv" "$ROOT/tb/uvm/sva/dll_checkers.sv" "$ROOT/tb/uvm/sva/tl_checkers.sv" "$ROOT/tb/uvm/tb_uvm_top.sv" \
    -top tb_uvm_top -o simv -l build.log > /dev/null 2>&1
if [ ! -x ./simv ]; then echo "BUILD_FAIL" > verdict.txt; exit 0; fi
TESTS=${2:-$ROOT/mutation/tests.txt}
n=0
while read -r name test args; do
  [ -z "$name" ] && continue; n=$((n+1))
  timeout -s KILL 300 ./simv -no_save +UVM_TESTNAME=$test $args 2>&1 < /dev/null | head -c 30000000 > run.log; rc=${PIPESTATUS[0]}; out=$(cat run.log)
  if [ $rc -eq 124 -o $rc -eq 137 ]; then echo "KILLED|$name|timeout (hang)" > verdict.txt; exit 0; fi
  if ! echo "$out" | grep -q "RESULT: PASS" || echo "$out" | grep -qE '^Error: "|failed at [0-9]+ps'; then
    det=$(echo "$out" | grep -E "failed at [0-9]+ps|^Error: \"|UVM_ERROR .*\[|UVM_FATAL .*\[" | head -1 | sed -E 's#.*(sva/|checkers\.sv", [0-9]+: )##; s#^.*tb_uvm_top\.dut[AB]\.##' | cut -c1-110)
    [ -z "$det" ] && det="no RESULT line (simulation error)"
    echo "KILLED|$name|$det" > verdict.txt; exit 0
  fi
done < "$TESTS"
echo "SURVIVED|all $n tests passed|" > verdict.txt
