#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RESULT_ROOT="${RESULT_ROOT:-${ROOT_DIR}/results/freqccv4_formal_matrix}"
MATRIX_DIR="${RESULT_ROOT}/long_lived_dynamic_delay"
REPORT_DIR="${RESULT_ROOT}/reports"
FORMAL_MODE="${FORMAL_MODE:-smoke}"
SIM_TIME="${SIM_TIME:-30}"
FLOW_SIZE_BYTES="${FLOW_SIZE_BYTES:-0}"
PROCESS_INTERVAL_US="${PROCESS_INTERVAL_US:-1000}"
USE_ENGINE_TIMER="${USE_ENGINE_TIMER:-true}"
DYNAMIC_DELAY_ENABLE="${DYNAMIC_DELAY_ENABLE:-true}"
ENABLE_HEAVY_TRACE="${ENABLE_HEAVY_TRACE:-true}"
GATE_TRACE_MODE_DEFAULT="${GATE_TRACE_MODE_DEFAULT:-round_only}"
GATE_TRACE_MODE_E="${GATE_TRACE_MODE_E:-sampled_pacing}"
GATE_TRACE_SAMPLE_INTERVAL_US="${GATE_TRACE_SAMPLE_INTERVAL_US:-10000}"
SKIP_EXISTING="${SKIP_EXISTING:-true}"

if [[ -n "${SEEDS:-}" ]]; then
  read -r -a SEED_LIST <<< "${SEEDS}"
elif [[ "${FORMAL_MODE}" == "full" ]]; then
  SEED_LIST=(2001 2002 2003 2004 2005 2006 2007 2008 2009 2010)
else
  SEED_LIST=(2001 2002 2003 2004 2005)
fi

GROUP_IDS=(
  "group_A_bbrv2"
  "group_B_old_freqccv4"
  "group_C_trace_only"
  "group_D_gate_mod_only"
  "group_E_gate_plus_fref"
)

declare -A GROUP_ALGO=(
  ["group_A_bbrv2"]="bbrv2"
  ["group_B_old_freqccv4"]="freqccv4"
  ["group_C_trace_only"]="freqccv4"
  ["group_D_gate_mod_only"]="freqccv4"
  ["group_E_gate_plus_fref"]="freqccv4"
)

declare -A GROUP_TRACE=(
  ["group_A_bbrv2"]="false"
  ["group_B_old_freqccv4"]="false"
  ["group_C_trace_only"]="true"
  ["group_D_gate_mod_only"]="true"
  ["group_E_gate_plus_fref"]="true"
)

declare -A GROUP_CONTROL=(
  ["group_A_bbrv2"]="false"
  ["group_B_old_freqccv4"]="false"
  ["group_C_trace_only"]="false"
  ["group_D_gate_mod_only"]="true"
  ["group_E_gate_plus_fref"]="true"
)

declare -A GROUP_FREF=(
  ["group_A_bbrv2"]="false"
  ["group_B_old_freqccv4"]="false"
  ["group_C_trace_only"]="false"
  ["group_D_gate_mod_only"]="false"
  ["group_E_gate_plus_fref"]="true"
)

mkdir -p "${MATRIX_DIR}" "${REPORT_DIR}"
cd "${ROOT_DIR}"

{
  echo "scenario=long_lived_dynamic_delay"
  echo "formal_mode=${FORMAL_MODE}"
  echo "seeds=${SEED_LIST[*]}"
  echo "sim_time=${SIM_TIME}"
  echo "flowSizeBytes=${FLOW_SIZE_BYTES}"
  echo "processIntervalUs=${PROCESS_INTERVAL_US}"
  echo "dynamic_delay_enable=${DYNAMIC_DELAY_ENABLE}"
  echo "useEngineTimer=${USE_ENGINE_TIMER}"
  echo "enableHeavyTrace=${ENABLE_HEAVY_TRACE}"
  echo "gateTraceModeDefault=${GATE_TRACE_MODE_DEFAULT}"
  echo "gateTraceModeE=${GATE_TRACE_MODE_E}"
  echo "gateTraceSampleIntervalUs=${GATE_TRACE_SAMPLE_INTERVAL_US}"
} > "${MATRIX_DIR}/scenario_config.txt"

echo "[build] ./waf build"
if ./waf build > "${REPORT_DIR}/build_long_lived.log" 2>&1; then
  echo "PASS" > "${REPORT_DIR}/build_status_long_lived.txt"
else
  echo "FAIL" > "${REPORT_DIR}/build_status_long_lived.txt"
  tail -80 "${REPORT_DIR}/build_long_lived.log" >&2 || true
  exit 1
fi

echo "[self-test] gateStateMachineSelfTest"
if ./waf --run "scratch/freqccv4_4flow --gateStateMachineSelfTest=true" \
    > "${REPORT_DIR}/gate_state_machine_self_test_long_lived.log" 2>&1; then
  echo "PASS" > "${REPORT_DIR}/gate_state_machine_self_test_long_lived_status.txt"
else
  echo "FAIL" > "${REPORT_DIR}/gate_state_machine_self_test_long_lived_status.txt"
  tail -80 "${REPORT_DIR}/gate_state_machine_self_test_long_lived.log" >&2 || true
  exit 1
fi

for seed in "${SEED_LIST[@]}"; do
  for group in "${GROUP_IDS[@]}"; do
    out_dir="${MATRIX_DIR}/${group}/seed_${seed}"
    done_marker="${out_dir}/.complete"
    if [[ "${SKIP_EXISTING}" == "true" && -f "${done_marker}" ]]; then
      echo "[skip] long_lived_dynamic_delay/${group}/seed_${seed}"
      continue
    fi

    mkdir -p "${out_dir}"
    gate_mode="${GATE_TRACE_MODE_DEFAULT}"
    if [[ "${group}" == "group_E_gate_plus_fref" ]]; then
      gate_mode="${GATE_TRACE_MODE_E}"
    fi

    run_arg="scratch/freqccv4_4flow --algo=${GROUP_ALGO[$group]} --sim_time=${SIM_TIME} --flowSizeBytes=${FLOW_SIZE_BYTES} --processIntervalUs=${PROCESS_INTERVAL_US} --flowStartMode=same_start --runId=${seed} --seed=${seed} --outputDir=${out_dir}/ --enableConvergenceGateTrace=${GROUP_TRACE[$group]} --enableConvergenceGateControl=${GROUP_CONTROL[$group]} --enableFreqRefPacingControl=${GROUP_FREF[$group]} --dynamic_delay_enable=${DYNAMIC_DELAY_ENABLE} --enableHeavyTrace=${ENABLE_HEAVY_TRACE} --gateTraceMode=${gate_mode} --gateTraceSampleIntervalUs=${GATE_TRACE_SAMPLE_INTERVAL_US} --useEngineTimer=${USE_ENGINE_TIMER}"
    printf '%s\n' "./waf --run \"${run_arg}\"" > "${out_dir}/command.txt"
    echo "[run] long_lived_dynamic_delay/${group}/seed_${seed}"
    rm -f "${done_marker}"
    ./waf --run "${run_arg}" > "${out_dir}/run.log" 2>&1
    date -u +"%Y-%m-%dT%H:%M:%SZ" > "${done_marker}"
  done
done

echo "[analyze] ${RESULT_ROOT}"
python3 "${ROOT_DIR}/analyze_freqccv4_formal_matrix.py" --results-dir "${RESULT_ROOT}"
