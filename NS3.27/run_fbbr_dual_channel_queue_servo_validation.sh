#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
OUT="${1:-$ROOT/docs/fbbr_dual_channel_queue_servo/latest}"
PROFILE="${PROFILE:-QUICK}"
CONFIG="$ROOT/examples/CCconfig/fbbr_default.conf"
export LD_LIBRARY_PATH="$ROOT/build${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"

if [[ "$PROFILE" == "FULL" ]]; then
  SEEDS_VALUE="${SEEDS:-1,2,3,4,5,6,7,8,9,10}"
  SINGLE_SECONDS="${SINGLE_SECONDS:-45}"
  FOUR_SECONDS="${FOUR_SECONDS:-45}"
  RESERVE_SECONDS="${RESERVE_SECONDS:-30}"
  DYNAMIC_SECONDS="${DYNAMIC_SECONDS:-50}"
else
  SEEDS_VALUE="${SEEDS:-1,2}"
  SINGLE_SECONDS="${SINGLE_SECONDS:-15}"
  FOUR_SECONDS="${FOUR_SECONDS:-15}"
  RESERVE_SECONDS="${RESERVE_SECONDS:-8}"
  DYNAMIC_SECONDS="${DYNAMIC_SECONDS:-20}"
fi

rm -rf "$OUT"
mkdir -p "$OUT/logs" "$OUT/configs"
cd "$ROOT"

./waf build -j2 >"$OUT/logs/build.log" 2>&1
./build/scratch/fbbr_4flow --fbbrFrequencySearchSelfTest=true \
  >"$OUT/logs/fbbr_deterministic_selftest.log" 2>&1
./build/scratch/freqccv4_4flow --gateStateMachineSelfTest=true \
  >"$OUT/logs/freqccv4_gate_selftest.log" 2>&1
./build/scratch/freqccv4_4flow --trustedBwSelectionSelfTest=true \
  >"$OUT/logs/freqccv4_trusted_selftest.log" 2>&1
./build/scratch/freqccv4_4flow --trustedBwPacingSelfTest=true \
  >"$OUT/logs/freqccv4_pacing_selftest.log" 2>&1

set +e
rg -n "F-BBR|FBBR|fbbr|kFBBR" \
  src/dqc/model/thirdparty/congestion/freqccv4_sender.h \
  src/dqc/model/thirdparty/congestion/freqccv4_sender.cc \
  scratch/freqccv4_4flow.cc \
  examples/CCconfig/freqccv4_default.conf \
  >"$OUT/logs/source_isolation.log" 2>&1
SOURCE_ISOLATION_STATUS=$?
set -e
if [[ $SOURCE_ISOLATION_STATUS -eq 0 ]]; then
  echo "FAIL_FBBR_FREQCCV4_ISOLATION" >"$OUT/logs/source_isolation.result"
else
  echo "PASS" >"$OUT/logs/source_isolation.result"
fi

flow_starts() {
  local flows="$1" seed="$2" out="" index jitter
  for ((index=0; index<flows; ++index)); do
    jitter=$(awk -v seed="$seed" -v i="$index" \
      'BEGIN { printf "%.3f", ((seed * 37 + i * 19) % 51) / 1000.0 }')
    out+="${out:+,}${jitter}"
  done
  printf '%s' "$out"
}

capacity_micro_schedule() {
  local seconds="$1" seed="$2" time=0 out="" rate sign
  while (( time < seconds )); do
    sign=$(( (seed + time / 5) % 3 - 1 ))
    rate=$((100 + sign))
    out+="${out:+,}${time}:${rate}Mbps"
    time=$((time + 5))
  done
  printf '%s' "$out"
}

run_case() {
  local name="$1" flows="$2" seconds="$3" seed="$4" bdp="$5"
  local config="$6" capacity="$7" background="$8"
  local directory="$OUT/$name"
  local starts stops="" index
  starts="$(flow_starts "$flows" "$seed")"
  for ((index=0; index<flows; ++index)); do
    stops+="${stops:+,}${seconds}"
  done
  mkdir -p "$directory"
  ./build/scratch/generic_p2p_switch_flows \
    --nFlows="$flows" --algos=F-BBR --simTime="$seconds" \
    --serviceRate=100Mbps --accessRate=1Gbps \
    --accessDelayMs=1 --serviceDelayMs=23 --switchBufferBdp="$bdp" \
    --flowStartTimes="$starts" --flowStopTimes="$stops" \
    --perFlowAppRateLimits=0 --backgroundRateSchedule="$background" \
    --capacitySchedule="$capacity" --ackTimingJitterUs=100 \
    --ackJitterIntervalMs=10 --packetSizeVariationBytes=100 \
    --processIntervalUs=1000 --goodputIntervalMs=500 --dataGeneratorBatch=50 \
    --useEngineTimer=true --enableTrace=true --enableHeavyTrace=false \
    --enableQueueTrace=false --enableConvergenceGateTrace=false \
    --enableEquivalenceAudit=false --emitRunMeta=true \
    --emitBottleneckQueueTrace=false --fbbrConfig="$config" \
    --tracePath="$directory/" --traceName="$name" \
    --seed="$seed" --runId="$seed" \
    >"$directory/stdout.log" 2>"$directory/stderr.log"
}

cp "$CONFIG" "$OUT/configs/new.conf"
sed -e 's/f_bbr.queue_servo.enabled = true/f_bbr.queue_servo.enabled = false/' \
    -e 's/f_bbr.trigger.queue.prominence_start = .*/f_bbr.trigger.queue.prominence_start = 1000000000/' \
    -e 's/f_bbr.trigger.queue.prominence_continue = .*/f_bbr.trigger.queue.prominence_continue = 1000000000/' \
    "$CONFIG" >"$OUT/configs/current_drate_only.conf"

IFS=',' read -r -a seeds <<<"$SEEDS_VALUE"
for seed in "${seeds[@]}"; do
  capacity="$(capacity_micro_schedule "$SINGLE_SECONDS" "$seed")"
  background="0:$((1 + seed % 2))Mbps"
  run_case "single_seed${seed}" 1 "$SINGLE_SECONDS" "$seed" 2 \
    "$OUT/configs/new.conf" "$capacity" "$background"

  capacity="$(capacity_micro_schedule "$FOUR_SECONDS" "$seed")"
  run_case "four_seed${seed}" 4 "$FOUR_SECONDS" "$seed" 2 \
    "$OUT/configs/new.conf" "$capacity" "$background"
done

run_case ab_current 4 "$FOUR_SECONDS" 101 2 \
  "$OUT/configs/current_drate_only.conf" "0:100Mbps" "0:0Mbps"
run_case ab_new 4 "$FOUR_SECONDS" 101 2 \
  "$OUT/configs/new.conf" "0:100Mbps" "0:0Mbps"

for reserve_pct in 0 1 2 5 10; do
  reserve=$(awk -v pct="$reserve_pct" 'BEGIN { printf "%.2f", pct / 100.0 }')
  reserve_config="$OUT/configs/reserve_${reserve_pct}.conf"
  sed -e "s/f_bbr.queue_reserve.low_bdp = .*/f_bbr.queue_reserve.low_bdp = $reserve/" \
      -e "s/f_bbr.queue_reserve.high_bdp = .*/f_bbr.queue_reserve.high_bdp = $reserve/" \
      "$CONFIG" >"$reserve_config"
  run_case "reserve_${reserve_pct}" 4 "$RESERVE_SECONDS" 201 2 \
    "$reserve_config" "0:100Mbps" "0:0Mbps"
done

run_case dynamic_capacity 4 "$DYNAMIC_SECONDS" 301 2 \
  "$OUT/configs/new.conf" \
  "0:100Mbps,5:70Mbps,10:100Mbps,15:70Mbps" \
  "0:0Mbps,5:10Mbps,10:20Mbps,15:0Mbps"

mixed="$OUT/mixed_isolation"
mkdir -p "$mixed"
./build/scratch/generic_p2p_switch_flows \
  --nFlows=2 --algos=F-BBR,FreqCCv4 --simTime=8 \
  --serviceRate=100Mbps --accessRate=1Gbps --accessDelayMs=1 \
  --serviceDelayMs=23 --switchBufferBdp=2 --flowStartTimes=0,0.02 \
  --flowStopTimes=8,8 --perFlowAppRateLimits=0 --processIntervalUs=1000 \
  --goodputIntervalMs=500 --dataGeneratorBatch=50 --useEngineTimer=true \
  --enableTrace=true --enableHeavyTrace=false --enableQueueTrace=false \
  --emitRunMeta=true --emitBottleneckQueueTrace=false \
  --fbbrConfig="$OUT/configs/new.conf" \
  --freqccv4Config="$ROOT/examples/CCconfig/freqccv4_default.conf" \
  --tracePath="$mixed/" --traceName=mixed --seed=401 --runId=401 \
  >"$mixed/stdout.log" 2>"$mixed/stderr.log"

cross_count=$(find "$mixed" -maxdepth 1 -type f \( \
  -name 'flow1_freq*.csv' -o -name 'flow2_fbbr*.csv' \) | wc -l)
if [[ "$cross_count" -eq 0 ]]; then
  echo PASS >"$OUT/logs/mixed_trace_isolation.result"
else
  echo FAIL_FBBR_FREQCCV4_ISOLATION >"$OUT/logs/mixed_trace_isolation.result"
fi

mapfile -t run_dirs < <(find "$OUT" -name run_meta.json -printf '%h\n' | \
  rg -v '/mixed_isolation$' | sort)
python3 examples/ConcurrentFlow/analyze_fbbr_dual_channel_queue_servo.py \
  "${run_dirs[@]}" --output-dir "$OUT/analysis"

echo "F-BBR dual-channel queue-servo validation artifacts: $OUT"
