#!/bin/bash
# Unit benches (spec 10.6): scripts/run_unit.sh [bench ...]   -> results/unit/<bench>.log, summary on stdout
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
source /apps/set_license.sh >/dev/null 2>&1
OUT="$ROOT/results/unit"; mkdir -p "$OUT"; cd "$OUT" || exit 1
BENCHES=${@:-"tb_crc tb_credit_gate tb_replay_buf tb_wr_coalescer tb_cpl_split"}
RTL=("$ROOT/rtl/pkg/pcie_pkg.sv"); for d in csr dll tl axi; do for f in "$ROOT"/rtl/$d/*.sv; do RTL+=("$f"); done; done
pass=0; fail=0
for b in $BENCHES; do
  rm -rf "$b.simv" "$b.simv.daidir"
  vcs -sverilog -full64 -timescale=1ns/1ps ${DEFS} +incdir+"$ROOT/tb/unit" "${RTL[@]}" "$ROOT/tb/unit/$b.sv" -top $b -o $b.simv -l $b.build.log > /dev/null 2>&1
  if [ ! -x $b.simv ]; then echo "BUILD_FAIL $b (see results/unit/$b.build.log)"; fail=$((fail+1)); continue; fi
  ./$b.simv -no_save > $b.log 2>&1
  line=$(grep -E "^$b: " $b.log | tail -1)
  if echo "$line" | grep -q "PASS"; then pass=$((pass+1)); else fail=$((fail+1)); fi
  echo "${line:-$b: no result line -> FAIL}"
done
echo "UNIT: pass=$pass fail=$fail"
