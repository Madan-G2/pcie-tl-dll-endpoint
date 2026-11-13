#!/bin/bash
# system regression sweep over seeds / modes.  usage: scripts/run_regression.sh [nseeds] [nops] [nstress]
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
NS=${1:-20}; NOPS=${2:-80}; NST=${3:-300}
source /apps/set_license.sh
cd "$ROOT/results" || exit 1
pass=0; fail=0
for sd in $(seq 1 $NS); do
  for cfg in "0 0" "1 0" "0 30000" "1 60000"; do
    set -- $cfg
    out=$(timeout 600 ./simv_sys -no_save +SEED=$sd +OPT=$1 +PPM=$2 +NOPS=$NOPS +STRESS=$NST 2>&1)
    if echo "$out" | grep -q "RESULT: PASS"; then pass=$((pass+1)); else fail=$((fail+1)); echo "FAIL seed=$sd opt=$1 ppm=$2: $(echo "$out" | grep -E 'ERR |PROTOCOL|RESULT' | head -3 | tr '\n' ' ')"; fi
  done
done
echo "REGRESSION: pass=$pass fail=$fail"
