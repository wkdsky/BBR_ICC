#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RESULT_ROOT="${RESULT_ROOT:-${ROOT_DIR}/results/fbbr_formal_matrix}"
MATRIX_DIR="${RESULT_ROOT}/finite_flow_fct"
REPORT_DIR="${RESULT_ROOT}/reports"
FORMAL_MODE="${FORMAL_MODE:-smoke}"
SIM_TIME="${SIM_TIME:-30}"
PROCESS_INTERVAL_US="${PROCESS_INTERVAL_US:-5000}"
USE_ENGINE_TIMER="${USE_ENGINE_TIMER:-true}"
DYNAMIC_DELAY_ENABLE="${DYNAMIC_DELAY_ENABLE:-false}"
ENABLE_HEAVY_TRACE="${ENABLE_HEAVY_TRACE:-true}"
GATE_TRACE_MODE_DEFAULT="${GATE_TRACE_MODE_DEFAULT:-round_only}"
GATE_TRACE_MODE_E="${GATE_TRACE_MODE_E:-sampled_pacing}"
GATE_TRACE_SAMPLE_INTERVAL_US="${GATE_TRACE_SAMPLE_INTERVAL_US:-10000}"
SKIP_EXISTING="${SKIP_EXISTING:-true}"

if [[ -n "${SEEDS:-}" ]]; then
  read -r -a SEED_LIST <<< "${SEEDS}"
elif [[ "${FORMAL_MODE}" == "full" ]]; then
  SEED_LIST=(3001 3002 3003 3004 3005 3006 3007 3008 3009 3010)
else
  SEED_LIST=(3001 3002 3003 3004 3005)
fi

if [[ -n "${SIZE_LABELS:-}" && -n "${SIZE_BYTES:-}" ]]; then
  read -r -a SIZE_LABEL_LIST <<< "${SIZE_LABELS}"
  read -r -a SIZE_BYTE_LIST <<< "${SIZE_BYTES}"
elif [[ "${FORMAL_MODE}" == "full" ]]; then
  SIZE_LABEL_LIST=("2MB" "6MB" "15MB")
  SIZE_BYTE_LIST=(2000000 6000000 15000000)
else
  SIZE_LABEL_LIST=("6MB")
  SIZE_BYTE_LIST=(6000000)
fi

if [[ -n "${START_MODES:-}" ]]; then
  read -r -a START_MODE_LIST <<< "${START_MODES}"
else
  START_MODE_LIST=("same_start" "staggered_start")
fi

GROUP_IDS=(
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


mkdir -p "${MATRIX_DIR}" "${REPORT_DIR}"
cd "${ROOT_DIR}"

{
  echo "scenario=finite_flow_fct"
  echo "formal_mode=${FORMAL_MODE}"
  echo "seeds=${SEED_LIST[*]}"
  echo "size_labels=${SIZE_LABEL_LIST[*]}"
  echo "size_bytes=${SIZE_BYTE_LIST[*]}"
  echo "start_modes=${START_MODE_LIST[*]}"
  echo "sim_time=${SIM_TIME}"
  echo "processIntervalUs=${PROCESS_INTERVAL_US}"
  echo "dynamic_delay_enable=${DYNAMIC_DELAY_ENABLE}"
  echo "useEngineTimer=${USE_ENGINE_TIMER}"
  echo "enableHeavyTrace=${ENABLE_HEAVY_TRACE}"
  echo "gateTraceModeDefault=${GATE_TRACE_MODE_DEFAULT}"
  echo "gateTraceModeE=${GATE_TRACE_MODE_E}"
  echo "gateTraceSampleIntervalUs=${GATE_TRACE_SAMPLE_INTERVAL_US}"
  echo "flowStartGapMs=not_available_in_fbbr_4flow_cc; staggered_start uses built-in 20/40/60ms offsets"
} > "${MATRIX_DIR}/scenario_config.txt"

echo "[build] ./waf build"
if ./waf build > "${REPORT_DIR}/build_fct.log" 2>&1; then
  echo "PASS" > "${REPORT_DIR}/build_status_fct.txt"
else
  echo "FAIL" > "${REPORT_DIR}/build_status_fct.txt"
  tail -80 "${REPORT_DIR}/build_fct.log" >&2 || true
  exit 1
fi

for size_idx in "${!SIZE_LABEL_LIST[@]}"; do
  size_label="${SIZE_LABEL_LIST[$size_idx]}"
  flow_size_bytes="${SIZE_BYTE_LIST[$size_idx]}"
  size_dir="size_${size_label}"

  for start_mode in "${START_MODE_LIST[@]}"; do
    for seed in "${SEED_LIST[@]}"; do
      for group in "${GROUP_IDS[@]}"; do
        out_dir="${MATRIX_DIR}/${size_dir}/${start_mode}/${group}/seed_${seed}"
        done_marker="${out_dir}/.complete"
        if [[ "${SKIP_EXISTING}" == "true" && -f "${done_marker}" ]]; then
          echo "[skip] finite_flow_fct/${size_dir}/${start_mode}/${group}/seed_${seed}"
          continue
        fi

        mkdir -p "${out_dir}"
        gate_mode="${GATE_TRACE_MODE_DEFAULT}"
        if [[ "${group}" == "group_C_fbbr_gated" ]]; then
          gate_mode="${GATE_TRACE_MODE_E}"
        fi

        run_arg="scratch/fbbr_4flow --algo=${GROUP_ALGO[$group]} --sim_time=${SIM_TIME} --flowSizeBytes=${flow_size_bytes} --processIntervalUs=${PROCESS_INTERVAL_US} --flowStartMode=${start_mode} --runId=${seed} --seed=${seed} --outputDir=${out_dir}/ --enableConvergenceGateTrace=${GROUP_TRACE[$group]} --enableConvergenceGateControl=${GROUP_CONTROL[$group]} --dynamic_delay_enable=${DYNAMIC_DELAY_ENABLE} --enableHeavyTrace=${ENABLE_HEAVY_TRACE} --gateTraceMode=${gate_mode} --gateTraceSampleIntervalUs=${GATE_TRACE_SAMPLE_INTERVAL_US} --useEngineTimer=${USE_ENGINE_TIMER}"
        printf '%s\n' "./waf --run \"${run_arg}\"" > "${out_dir}/command.txt"
        echo "[run] finite_flow_fct/${size_dir}/${start_mode}/${group}/seed_${seed}"
        rm -f "${done_marker}"
        ./waf --run "${run_arg}" > "${out_dir}/run.log" 2>&1
        date -u +"%Y-%m-%dT%H:%M:%SZ" > "${done_marker}"
      done
    done
  done
done

echo "[analyze] ${RESULT_ROOT}"
python3 "${ROOT_DIR}/analyze_fbbr_formal_matrix.py" --results-dir "${RESULT_ROOT}"
