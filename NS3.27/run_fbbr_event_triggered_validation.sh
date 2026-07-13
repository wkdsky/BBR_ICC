#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
OUT="${1:-$ROOT/docs/fbbr_event_triggered/latest}"
CONFIG="$ROOT/examples/CCconfig/fbbr_default.conf"
export LD_LIBRARY_PATH="$ROOT/build${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"

mkdir -p "$OUT"
cd "$ROOT"
./waf build -j2
./build/scratch/fbbr_4flow --fbbrFrequencySearchSelfTest=true \
  > "$OUT/deterministic_selftest.log" 2>&1

run_case() {
  local mode="$1" flows="$2" seconds="$3" seed="$4" bdp="$5" directory="$6"
  local case_config="$CONFIG"
  mkdir -p "$directory"
  if [[ "$mode" == "fixed" ]]; then
    case_config="$directory/fbbr_fixed_window.conf"
    sed 's/f_bbr.frequency_search.event_triggered_windows_enabled = true/f_bbr.frequency_search.event_triggered_windows_enabled = false/' \
      "$CONFIG" > "$case_config"
  fi
  ./build/scratch/generic_p2p_switch_flows \
    --nFlows="$flows" --algos=F-BBR --simTime="$seconds" \
    --serviceRate=100Mbps --accessRate=1Gbps \
    --accessDelayMs=1 --serviceDelayMs=23 --switchBufferBdp="$bdp" \
    --flowStartTimes=0 --flowStopTimes="$seconds" --perFlowAppRateLimits=0 \
    --processIntervalUs=1000 --goodputIntervalMs=500 --dataGeneratorBatch=20 \
    --useEngineTimer=true --enableTrace=true --enableHeavyTrace=false \
    --enableQueueTrace=false --enableConvergenceGateTrace=false \
    --enableEquivalenceAudit=false --emitRunMeta=true \
    --emitBottleneckQueueTrace=false --fbbrConfig="$case_config" \
    --tracePath="$directory/" --traceName="fbbr_${mode}" \
    --seed="$seed" --runId="$seed" \
    > "$directory/stdout.log" 2> "$directory/stderr.log"
}

# Same-topology fixed-window/event-window A/B.
run_case event 4 45 2 2 "$OUT/ab_event"
run_case fixed 4 45 2 2 "$OUT/ab_fixed"

# Buffer-depth and multi-seed event matrix.
IFS=',' read -r -a seeds <<< "${SEEDS:-1,2,3,4,5,6,7,8,9,10}"
for bdp in 2 4 8; do
  for seed in "${seeds[@]}"; do
    run_case event 4 "${MATRIX_SECONDS:-30}" "$seed" "$bdp" \
      "$OUT/matrix_bdp${bdp}_seed${seed}"
  done
done

for flows in 1 2 4 8; do
  for seed in "${seeds[@]}"; do
    run_case event "$flows" "${CONVERGENCE_SECONDS:-45}" "$seed" 2 \
      "$OUT/convergence_flows${flows}_seed${seed}"
  done
done

mkdir -p "$OUT/reserve_configs"
for reserve_pct in 0 1 2 5 10; do
  reserve=$(awk -v pct="$reserve_pct" 'BEGIN { printf "%.2f", pct / 100.0 }')
  reserve_config="$OUT/reserve_configs/reserve_${reserve_pct}.conf"
  sed -e "s/f_bbr.frequency_search.q_reserve_low_bdp = .*/f_bbr.frequency_search.q_reserve_low_bdp = $reserve/" \
      -e "s/f_bbr.frequency_search.q_reserve_high_bdp = .*/f_bbr.frequency_search.q_reserve_high_bdp = $reserve/" \
      "$CONFIG" > "$reserve_config"
  original_config="$CONFIG"
  CONFIG="$reserve_config"
  run_case event 4 "${RESERVE_SECONDS:-30}" 1 2 \
    "$OUT/reserve_${reserve_pct}"
  CONFIG="$original_config"
done

dynamic_dir="$OUT/dynamic_capacity"
mkdir -p "$dynamic_dir"
./build/scratch/generic_p2p_switch_flows \
  --nFlows=4 --algos=F-BBR --simTime=50 \
  --serviceRate=100Mbps --accessRate=1Gbps \
  --accessDelayMs=1 --serviceDelayMs=23 --switchBufferBdp=2 \
  --flowStartTimes=0 --flowStopTimes=50 --perFlowAppRateLimits=0 \
  --capacitySchedule=0:100Mbps,15:70Mbps,30:100Mbps,40:70Mbps \
  --processIntervalUs=1000 --goodputIntervalMs=500 --dataGeneratorBatch=20 \
  --useEngineTimer=true --enableTrace=true --enableHeavyTrace=false \
  --enableQueueTrace=false --enableConvergenceGateTrace=false \
  --enableEquivalenceAudit=false --emitRunMeta=true \
  --emitBottleneckQueueTrace=false --fbbrConfig="$CONFIG" \
  --tracePath="$dynamic_dir/" --traceName=fbbr_dynamic \
  --seed=6 --runId=6 > "$dynamic_dir/stdout.log" 2> "$dynamic_dir/stderr.log"

mapfile -t run_dirs < <(find "$OUT" -name run_meta.json -printf '%h\n' | sort)
python3 examples/ConcurrentFlow/analyze_fbbr_event_triggered.py \
  "${run_dirs[@]}" --output-dir "$OUT/analysis"

echo "F-BBR event-triggered validation artifacts: $OUT"
