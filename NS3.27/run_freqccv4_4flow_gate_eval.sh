#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RESULT_ROOT="${RESULT_ROOT:-${ROOT_DIR}/results/freqccv4_4flow_gate_eval}"
RUNS="${RUNS:-5}"
START_RUN="${START_RUN:-1}"
SIM_TIME="${SIM_TIME:-30}"
FLOW_SIZE_BYTES="${FLOW_SIZE_BYTES:-15000000}"
PROCESS_INTERVAL_US="${PROCESS_INTERVAL_US:-1000}"
SEED_BASE="${SEED_BASE:-1000}"
DYNAMIC_DELAY_ENABLE="${DYNAMIC_DELAY_ENABLE:-true}"
SMOKE_MODE="${SMOKE_MODE:-false}"
ENABLE_HEAVY_TRACE="${ENABLE_HEAVY_TRACE:-false}"
GATE_TRACE_MODE="${GATE_TRACE_MODE:-round_only}"
GATE_TRACE_SAMPLE_INTERVAL_US="${GATE_TRACE_SAMPLE_INTERVAL_US:-1000}"
USE_ENGINE_TIMER="${USE_ENGINE_TIMER:-true}"

declare -a GROUP_IDS=(
  "group_A_bbrv2"
  "group_B_freqccv4_old"
  "group_C_trace_only"
  "group_D_gate_mod_only"
  "group_E_gate_plus_fref"
)

declare -A GROUP_ALGO=(
  ["group_A_bbrv2"]="bbrv2"
  ["group_B_freqccv4_old"]="freqccv4"
  ["group_C_trace_only"]="freqccv4"
  ["group_D_gate_mod_only"]="freqccv4"
  ["group_E_gate_plus_fref"]="freqccv4"
)

declare -A GROUP_TRACE=(
  ["group_A_bbrv2"]="false"
  ["group_B_freqccv4_old"]="false"
  ["group_C_trace_only"]="true"
  ["group_D_gate_mod_only"]="true"
  ["group_E_gate_plus_fref"]="true"
)

declare -A GROUP_CONTROL=(
  ["group_A_bbrv2"]="false"
  ["group_B_freqccv4_old"]="false"
  ["group_C_trace_only"]="false"
  ["group_D_gate_mod_only"]="true"
  ["group_E_gate_plus_fref"]="true"
)

declare -A GROUP_FREF=(
  ["group_A_bbrv2"]="false"
  ["group_B_freqccv4_old"]="false"
  ["group_C_trace_only"]="false"
  ["group_D_gate_mod_only"]="false"
  ["group_E_gate_plus_fref"]="true"
)

declare -a PATTERNS=("same_start" "staggered_start")

cd "${ROOT_DIR}"
mkdir -p "${RESULT_ROOT}"

echo "[build] ./waf build"
./waf build

end_run=$((START_RUN + RUNS - 1))
for group in "${GROUP_IDS[@]}"; do
  for pattern in "${PATTERNS[@]}"; do
    for run_id in $(seq "${START_RUN}" "${end_run}"); do
      out_dir="${RESULT_ROOT}/${group}/${pattern}/run_${run_id}"
      mkdir -p "${out_dir}"
      seed=$((SEED_BASE + run_id))
      cmd="./waf --run \"scratch/freqccv4_4flow --algo=${GROUP_ALGO[$group]} --sim_time=${SIM_TIME} --flowSizeBytes=${FLOW_SIZE_BYTES} --processIntervalUs=${PROCESS_INTERVAL_US} --flowStartMode=${pattern} --runId=${run_id} --seed=${seed} --outputDir=${out_dir}/ --enableConvergenceGateTrace=${GROUP_TRACE[$group]} --enableConvergenceGateControl=${GROUP_CONTROL[$group]} --enableFreqRefPacingControl=${GROUP_FREF[$group]} --dynamic_delay_enable=${DYNAMIC_DELAY_ENABLE} --smokeMode=${SMOKE_MODE} --enableHeavyTrace=${ENABLE_HEAVY_TRACE} --gateTraceMode=${GATE_TRACE_MODE} --gateTraceSampleIntervalUs=${GATE_TRACE_SAMPLE_INTERVAL_US} --useEngineTimer=${USE_ENGINE_TIMER}\""
      echo "${cmd}" > "${out_dir}/command.txt"
      echo "[run] ${group}/${pattern}/run_${run_id}"
      eval "${cmd}" > "${out_dir}/run.log" 2>&1
    done
  done
done

echo "[analyze] ${RESULT_ROOT}"
python3 "${ROOT_DIR}/analyze_freqccv4_4flow_gate_eval.py" \
  --results-dir "${RESULT_ROOT}" \
  --flow-size-bytes "${FLOW_SIZE_BYTES}" \
  --sim-time "${SIM_TIME}"
