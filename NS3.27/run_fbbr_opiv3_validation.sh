#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MODE="${1:-smoke}"
OUT="${2:-$ROOT/docs/fbbr_opiv3_validation/repro}"
FOUR_FLOW_BUFFER_BDP="${FBBR_FOUR_FLOW_BUFFER_BDP:-8}"
CONFIG="$ROOT/examples/CCconfig/fbbr_opiv2_validation.conf"
export LD_LIBRARY_PATH="$ROOT/build${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"

mkdir -p "$OUT"
cd "$ROOT"
./waf build -j2
./build/scratch/fbbr_4flow --fbbrOpiv2SelfTest=true \
  > "$OUT/deterministic_selftest.log" 2>&1

run_case() {
  local flows="$1"
  local seconds="$2"
  local seed="$3"
  local buffer_bdp="$4"
  local directory="$5"
  mkdir -p "$directory"
  ./build/scratch/generic_p2p_switch_flows \
    --nFlows="$flows" --algos=F-BBR --simTime="$seconds" \
    --serviceRate=100Mbps --accessRate=1Gbps \
    --accessDelayMs=1 --serviceDelayMs=23 --switchBufferBdp="$buffer_bdp" \
    --flowStartTimes=0 --flowStopTimes="$seconds" \
    --perFlowAppRateLimits=0 --processIntervalUs=100 \
    --goodputIntervalMs=100 --dataGeneratorBatch=2 --useEngineTimer=true \
    --enableTrace=true --enableHeavyTrace=false \
    --enableQueueTrace=false --enableConvergenceGateTrace=false \
    --emitRunMeta=true --emitBottleneckQueueTrace=true \
    --queueSampleIntervalUs=1000 --fbbrConfig="$CONFIG" \
    --tracePath="$directory/" --traceName=fbbr_opiv3_repro \
    --seed="$seed" --runId="$seed" \
    > "$directory/stdout.log" 2> "$directory/stderr.log"
}

run_case 1 45 1 2 "$OUT/phase_a_n1_bdp2"
run_case 4 20 4 "$FOUR_FLOW_BUFFER_BDP" \
  "$OUT/phase_a_n4_bdp${FOUR_FLOW_BUFFER_BDP}"
python3 examples/ConcurrentFlow/plot_fbbr_opiv2.py \
  --run-dir "$OUT/phase_a_n4_bdp${FOUR_FLOW_BUFFER_BDP}" \
  --output-dir "$OUT/phase_a_n4_bdp${FOUR_FLOW_BUFFER_BDP}/plots"

set +e
python3 examples/ConcurrentFlow/analyze_fbbr_opiv3_input.py \
  "$OUT/phase_a_n1_bdp2" \
  "$OUT/phase_a_n4_bdp${FOUR_FLOW_BUFFER_BDP}" \
  --output-dir "$OUT/phase_a_report"
phase_a_status=$?
set -e
if [[ "$phase_a_status" -ne 0 ]]; then
  echo "Phase A acceptance is not met; preserving artifacts and continuing." >&2
fi

if [[ "$MODE" == "full" ]]; then
  python3 examples/ConcurrentFlow/run_fbbr_opiv2_validation_matrix.py \
    --scenario all --seeds 1-10 --jobs 2 \
    --buffer-bdp "$FOUR_FLOW_BUFFER_BDP" \
    --output-root "$OUT/full_matrix"
  python3 examples/ConcurrentFlow/analyze_fbbr_opiv2_validation.py \
    "$OUT/full_matrix" --output-dir "$OUT/full_analysis"
  python3 examples/ConcurrentFlow/plot_fbbr_opiv2_validation.py \
    "$OUT/full_analysis" --output-dir "$OUT/full_analysis/plots"
fi

exit "$phase_a_status"
