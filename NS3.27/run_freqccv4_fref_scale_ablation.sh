#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RESULT_ROOT="${RESULT_ROOT:-${ROOT_DIR}/results/freqccv4_fref_scale_ablation}"
REPORT_DIR="${RESULT_ROOT}/reports"
SKIP_EXISTING="${SKIP_EXISTING:-true}"

LONG_SIM_TIME="${LONG_SIM_TIME:-30}"
LONG_PROCESS_INTERVAL_US="${LONG_PROCESS_INTERVAL_US:-1000}"
FCT_SIM_TIME="${FCT_SIM_TIME:-30}"
FCT_PROCESS_INTERVAL_US="${FCT_PROCESS_INTERVAL_US:-5000}"
GATE_TRACE_SAMPLE_INTERVAL_US="${GATE_TRACE_SAMPLE_INTERVAL_US:-10000}"
USE_ENGINE_TIMER="${USE_ENGINE_TIMER:-true}"
ENABLE_HEAVY_TRACE="${ENABLE_HEAVY_TRACE:-true}"

if [[ -n "${LONG_SEEDS:-}" ]]; then
  read -r -a LONG_SEED_LIST <<< "${LONG_SEEDS}"
else
  LONG_SEED_LIST=(2001 2002 2003 2004 2005 2006 2007 2008 2009 2010)
fi

if [[ -n "${FCT_SEEDS:-}" ]]; then
  read -r -a FCT_SEED_LIST <<< "${FCT_SEEDS}"
else
  FCT_SEED_LIST=(3001 3002 3003 3004 3005 3006 3007 3008 3009 3010)
fi

if [[ -n "${FCT_SIZE_LABELS:-}" && -n "${FCT_SIZE_BYTES:-}" ]]; then
  read -r -a FCT_SIZE_LABEL_LIST <<< "${FCT_SIZE_LABELS}"
  read -r -a FCT_SIZE_BYTE_LIST <<< "${FCT_SIZE_BYTES}"
else
  FCT_SIZE_LABEL_LIST=("6MB")
  FCT_SIZE_BYTE_LIST=(6000000)
fi

if [[ -n "${FCT_START_MODES:-}" ]]; then
  read -r -a FCT_START_MODE_LIST <<< "${FCT_START_MODES}"
else
  FCT_START_MODE_LIST=("same_start" "staggered_start")
fi

GROUP_IDS=(
  "group_B_old_freqccv4"
  "group_D_gate_only"
  "group_E_current"
  "group_E_downward_only"
  "group_E_asymmetric"
  "group_E_high_conf_only"
  "group_E_early_episode_only"
)

declare -A GROUP_TRACE=(
  ["group_B_old_freqccv4"]="false"
  ["group_D_gate_only"]="true"
  ["group_E_current"]="true"
  ["group_E_downward_only"]="true"
  ["group_E_asymmetric"]="true"
  ["group_E_high_conf_only"]="true"
  ["group_E_early_episode_only"]="true"
)

declare -A GROUP_CONTROL=(
  ["group_B_old_freqccv4"]="false"
  ["group_D_gate_only"]="true"
  ["group_E_current"]="true"
  ["group_E_downward_only"]="true"
  ["group_E_asymmetric"]="true"
  ["group_E_high_conf_only"]="true"
  ["group_E_early_episode_only"]="true"
)

declare -A GROUP_FREF=(
  ["group_B_old_freqccv4"]="false"
  ["group_D_gate_only"]="false"
  ["group_E_current"]="true"
  ["group_E_downward_only"]="true"
  ["group_E_asymmetric"]="true"
  ["group_E_high_conf_only"]="true"
  ["group_E_early_episode_only"]="true"
)

declare -A GROUP_SCALE_MODE=(
  ["group_B_old_freqccv4"]="current"
  ["group_D_gate_only"]="current"
  ["group_E_current"]="current"
  ["group_E_downward_only"]="downward_only"
  ["group_E_asymmetric"]="asymmetric"
  ["group_E_high_conf_only"]="high_conf_only"
  ["group_E_early_episode_only"]="early_episode_only"
)

mkdir -p "${RESULT_ROOT}" "${REPORT_DIR}"
cd "${ROOT_DIR}"

{
  echo "long_seeds=${LONG_SEED_LIST[*]}"
  echo "fct_seeds=${FCT_SEED_LIST[*]}"
  echo "fct_size_labels=${FCT_SIZE_LABEL_LIST[*]}"
  echo "fct_size_bytes=${FCT_SIZE_BYTE_LIST[*]}"
  echo "fct_start_modes=${FCT_START_MODE_LIST[*]}"
  echo "long_sim_time=${LONG_SIM_TIME}"
  echo "fct_sim_time=${FCT_SIM_TIME}"
  echo "long_processIntervalUs=${LONG_PROCESS_INTERVAL_US}"
  echo "fct_processIntervalUs=${FCT_PROCESS_INTERVAL_US}"
  echo "dynamic_delay_long=true"
  echo "dynamic_delay_fct=false"
  echo "freqRefHighConfThreshold=0.8"
  echo "freqRefUpBeta=0.25"
} > "${RESULT_ROOT}/scenario_config.txt"

echo "[build] ./waf build"
if ./waf build > "${REPORT_DIR}/build.log" 2>&1; then
  echo "PASS" > "${REPORT_DIR}/build_status.txt"
else
  echo "FAIL" > "${REPORT_DIR}/build_status.txt"
  tail -80 "${REPORT_DIR}/build.log" >&2 || true
  exit 1
fi

echo "[self-test] gateStateMachineSelfTest"
if ./waf --run "scratch/freqccv4_4flow --gateStateMachineSelfTest=true" \
    > "${REPORT_DIR}/gate_state_machine_self_test.log" 2>&1; then
  echo "PASS" > "${REPORT_DIR}/gate_state_machine_self_test_status.txt"
else
  echo "FAIL" > "${REPORT_DIR}/gate_state_machine_self_test_status.txt"
  tail -80 "${REPORT_DIR}/gate_state_machine_self_test.log" >&2 || true
  exit 1
fi

run_one() {
  local out_dir="$1"
  local sim_time="$2"
  local flow_size="$3"
  local process_interval="$4"
  local flow_start="$5"
  local seed="$6"
  local group="$7"
  local dynamic_delay="$8"
  local audit="$9"
  local gate_mode="${10}"

  local done_marker="${out_dir}/.complete"
  if [[ "${SKIP_EXISTING}" == "true" && -f "${done_marker}" ]]; then
    echo "[skip] ${out_dir#${RESULT_ROOT}/}"
    return
  fi
  mkdir -p "${out_dir}"
  local run_arg="scratch/freqccv4_4flow --algo=freqccv4 --sim_time=${sim_time} --flowSizeBytes=${flow_size} --processIntervalUs=${process_interval} --flowStartMode=${flow_start} --runId=${seed} --seed=${seed} --outputDir=${out_dir}/ --enableConvergenceGateTrace=${GROUP_TRACE[$group]} --enableConvergenceGateControl=${GROUP_CONTROL[$group]} --enableFreqRefPacingControl=${GROUP_FREF[$group]} --freqRefScaleMode=${GROUP_SCALE_MODE[$group]} --freqRefHighConfThreshold=0.8 --freqRefUpBeta=0.25 --dynamic_delay_enable=${dynamic_delay} --enableHeavyTrace=${ENABLE_HEAVY_TRACE} --gateTraceMode=${gate_mode} --gateTraceSampleIntervalUs=${GATE_TRACE_SAMPLE_INTERVAL_US} --useEngineTimer=${USE_ENGINE_TIMER} --enableEquivalenceAuditTrace=${audit}"
  printf '%s\n' "./waf --run \"${run_arg}\"" > "${out_dir}/command.txt"
  echo "[run] ${out_dir#${RESULT_ROOT}/}"
  rm -f "${done_marker}"
  ./waf --run "${run_arg}" > "${out_dir}/run.log" 2>&1
  date -u +"%Y-%m-%dT%H:%M:%SZ" > "${done_marker}"
}

echo "[audit] trace-only B/C guard"
AUDIT_ROOT="${RESULT_ROOT}/trace_only_guard"
run_one "${AUDIT_ROOT}/B_reference" 3 0 1000 same_start 5151 group_B_old_freqccv4 true true round_only
run_one "${AUDIT_ROOT}/B_repeat" 3 0 1000 same_start 5151 group_B_old_freqccv4 true true round_only

if [[ ! -f "${AUDIT_ROOT}/C_trace_only/.complete_c_actual" ]]; then
  rm -f "${AUDIT_ROOT}/C_trace_only/.complete"
  mkdir -p "${AUDIT_ROOT}/C_trace_only"
  run_arg="scratch/freqccv4_4flow --algo=freqccv4 --sim_time=3 --flowSizeBytes=0 --processIntervalUs=1000 --flowStartMode=same_start --runId=5151 --seed=5151 --outputDir=${AUDIT_ROOT}/C_trace_only/ --enableConvergenceGateTrace=true --enableConvergenceGateControl=false --enableFreqRefPacingControl=false --freqRefScaleMode=current --freqRefHighConfThreshold=0.8 --freqRefUpBeta=0.25 --dynamic_delay_enable=true --enableHeavyTrace=${ENABLE_HEAVY_TRACE} --gateTraceMode=round_only --gateTraceSampleIntervalUs=${GATE_TRACE_SAMPLE_INTERVAL_US} --useEngineTimer=${USE_ENGINE_TIMER} --enableEquivalenceAuditTrace=true"
  printf '%s\n' "./waf --run \"${run_arg}\"" > "${AUDIT_ROOT}/C_trace_only/command.txt"
  echo "[run] trace_only_guard/C_trace_only"
  ./waf --run "${run_arg}" > "${AUDIT_ROOT}/C_trace_only/run.log" 2>&1
  date -u +"%Y-%m-%dT%H:%M:%SZ" > "${AUDIT_ROOT}/C_trace_only/.complete"
  date -u +"%Y-%m-%dT%H:%M:%SZ" > "${AUDIT_ROOT}/C_trace_only/.complete_c_actual"
fi

echo "[matrix] long-lived scale modes"
for seed in "${LONG_SEED_LIST[@]}"; do
  for group in "${GROUP_IDS[@]}"; do
    gate_mode="round_only"
    if [[ "${group}" == group_E_* ]]; then
      gate_mode="sampled_pacing"
    fi
    run_one "${RESULT_ROOT}/long_lived_dynamic_delay/${group}/seed_${seed}" \
      "${LONG_SIM_TIME}" 0 "${LONG_PROCESS_INTERVAL_US}" same_start "${seed}" \
      "${group}" true false "${gate_mode}"
  done
done

echo "[matrix] finite-flow scale modes"
for size_idx in "${!FCT_SIZE_LABEL_LIST[@]}"; do
  size_label="${FCT_SIZE_LABEL_LIST[$size_idx]}"
  flow_size="${FCT_SIZE_BYTE_LIST[$size_idx]}"
  for start_mode in "${FCT_START_MODE_LIST[@]}"; do
    for seed in "${FCT_SEED_LIST[@]}"; do
      for group in "${GROUP_IDS[@]}"; do
        gate_mode="round_only"
        if [[ "${group}" == group_E_* ]]; then
          gate_mode="sampled_pacing"
        fi
        run_one "${RESULT_ROOT}/finite_flow_fct/size_${size_label}/${start_mode}/${group}/seed_${seed}" \
          "${FCT_SIM_TIME}" "${flow_size}" "${FCT_PROCESS_INTERVAL_US}" "${start_mode}" "${seed}" \
          "${group}" false false "${gate_mode}"
      done
    done
  done
done

echo "[analyze] ${RESULT_ROOT}"
python3 "${ROOT_DIR}/analyze_freqccv4_fref_scale_ablation.py" --results-dir "${RESULT_ROOT}"
