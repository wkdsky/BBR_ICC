#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RESULT_ROOT="${RESULT_ROOT:-${ROOT_DIR}/results/fbbr_4flow_gate_packet_instability}"
SIM_TIME="${SIM_TIME:-30}"
FLOW_SIZE_BYTES="${FLOW_SIZE_BYTES:-0}"
PROCESS_INTERVAL_US="${PROCESS_INTERVAL_US:-1000}"
GATE_TRACE_SAMPLE_INTERVAL_US="${GATE_TRACE_SAMPLE_INTERVAL_US:-10000}"
USE_ENGINE_TIMER="${USE_ENGINE_TIMER:-true}"
DYNAMIC_DELAY_ENABLE="${DYNAMIC_DELAY_ENABLE:-true}"
ENABLE_HEAVY_TRACE="${ENABLE_HEAVY_TRACE:-true}"

declare -a SEEDS=(2001 2002 2003)
declare -a GROUP_IDS=(
  "group_A_bbrv2"
  "group_B_fbbr_open"
  "group_C_fbbr_gated"
)

declare -A GROUP_ALGO=(
  ["group_A_bbrv2"]="BBRv2"
  ["group_B_fbbr_open"]="FBBR"
  ["group_C_fbbr_gated"]="FBBR"
)

declare -A GROUP_TRACE=(
  ["group_A_bbrv2"]="false"
  ["group_B_fbbr_open"]="false"
  ["group_C_fbbr_gated"]="true"
)

declare -A GROUP_CONTROL=(
  ["group_A_bbrv2"]="false"
  ["group_B_fbbr_open"]="false"
  ["group_C_fbbr_gated"]="true"
)


cd "${ROOT_DIR}"
mkdir -p "${RESULT_ROOT}/multiseed"

echo "[build] ./waf build"
./waf build

for seed in "${SEEDS[@]}"; do
  for group in "${GROUP_IDS[@]}"; do
    out_dir="${RESULT_ROOT}/multiseed/${group}/same_start/seed_${seed}"
    mkdir -p "${out_dir}"
    run_id="${seed}"
    run_arg="scratch/fbbr_4flow --algo=${GROUP_ALGO[$group]} --sim_time=${SIM_TIME} --flowSizeBytes=${FLOW_SIZE_BYTES} --processIntervalUs=${PROCESS_INTERVAL_US} --flowStartMode=same_start --runId=${run_id} --seed=${seed} --outputDir=${out_dir}/ --enableConvergenceGateTrace=${GROUP_TRACE[$group]} --enableConvergenceGateControl=${GROUP_CONTROL[$group]} --dynamic_delay_enable=${DYNAMIC_DELAY_ENABLE} --enableHeavyTrace=${ENABLE_HEAVY_TRACE} --gateTraceMode=sampled_pacing --gateTraceSampleIntervalUs=${GATE_TRACE_SAMPLE_INTERVAL_US} --useEngineTimer=${USE_ENGINE_TIMER}"
    printf '%s\n' "./waf --run \"${run_arg}\"" > "${out_dir}/command.txt"
    echo "[run] ${group}/same_start/seed_${seed}"
    ./waf --run "${run_arg}" > "${out_dir}/run.log" 2>&1
  done
done

echo "[analyze] ${RESULT_ROOT}"
python3 "${ROOT_DIR}/analyze_fbbr_packet_instability.py" \
  --results-dir "${RESULT_ROOT}" \
  --sim-time "${SIM_TIME}" \
  --flow-size-bytes "${FLOW_SIZE_BYTES}"
