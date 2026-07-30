#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
NS3_DIR="$(cd "${SCRIPT_DIR}/../../.." && pwd)"
SCENARIOS_FILE="${SCRIPT_DIR}/scenarios.csv"
RESULTS_REL="results/test2"
RAW_REL="${RESULTS_REL}/raw"
LOGS_REL="${RESULTS_REL}/logs"
MANIFEST_REL="${RAW_REL}/manifest.csv"
MANIFEST_ENTRIES_REL="${RAW_REL}/manifest_entries"
TARGET="test2-fixed4"
BINARY_REL="build/examples/paper-test/test2/ns3.27-test2-fixed4-optimized"
FBBR_CONFIG_REL="examples/CCconfig/fbbr_default.conf"

SMOKE=0
SKIP_BUILD=0
JOBS=1
SCENARIO_FILTER=""
for argument in "$@"; do
  case "${argument}" in
    --smoke) SMOKE=1 ;;
    --skip-build) SKIP_BUILD=1 ;;
    --jobs=*) JOBS="${argument#--jobs=}" ;;
    --scenario=*) SCENARIO_FILTER="${argument#--scenario=}" ;;
    *)
      echo "Usage: $0 [--smoke] [--skip-build] [--jobs=N] [--scenario=ID[,ID...]]" >&2
      exit 2
      ;;
  esac
done

if [[ ! "${JOBS}" =~ ^[1-9][0-9]*$ ]]; then
  echo "--jobs must be a positive integer" >&2
  exit 2
fi
if [[ ! -s "${SCENARIOS_FILE}" ]]; then
  echo "Missing scenario file: ${SCENARIOS_FILE}" >&2
  exit 1
fi

scenario_is_selected() {
  local scenario_id="$1"
  [[ -z "${SCENARIO_FILTER}" || ",${SCENARIO_FILTER}," == *",${scenario_id},"* ]]
}

cd "${NS3_DIR}"
mkdir -p "${RAW_REL}" "${LOGS_REL}" "${RESULTS_REL}/summary" \
  "${MANIFEST_ENTRIES_REL}"
if [[ ! -r "${FBBR_CONFIG_REL}" ]]; then
  echo "Missing FBBR configuration: ${NS3_DIR}/${FBBR_CONFIG_REL}" >&2
  exit 1
fi

sync_exported_header() {
  local source="$1"
  local target="build/ns3/$(basename "${source}")"
  if [[ -e "${target}" ]]; then
    chmod u+w "${target}"
  fi
  cp "${source}" "${target}"
  chmod a-w "${target}"
}

if [[ "${SKIP_BUILD}" -eq 0 ]]; then
  ./waf configure --enable-examples --build-profile=optimized >/dev/null
  sync_exported_header src/dqc/model/dqc_sender.h
  sync_exported_header src/dqc/model/dqc_receiver.h
  sync_exported_header src/dqc/model/thirdparty/congestion/quic_bbr2_sender.h
  sync_exported_header src/dqc/model/thirdparty/congestion/fbbr_config_loader.h
  sync_exported_header src/dqc/model/thirdparty/congestion/fbbr_sender.h
  sync_exported_header src/dqc/model/thirdparty/include/proto_types.h
  ./waf build --targets="${TARGET}"
fi

if [[ ! -x "${BINARY_REL}" ]]; then
  echo "Missing experiment binary: ${BINARY_REL}" >&2
  exit 1
fi

SIMULATION_TIME=300
if [[ "${SMOKE}" -eq 1 ]]; then
  SIMULATION_TIME=45
fi

export LD_LIBRARY_PATH="${NS3_DIR}/build${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
printf '%s\n' \
  'scenario_id,capacity_bps,capacity_mbps,base_rtt_s,base_rtt_ms,buffer_bdp,bottleneck_delay_s,access_bps,purpose,algorithm,mode,seed,run_id,run_summary_path,stage_metrics_path,flow_metrics_path,events_path,minute_metrics_path,minute_flow_metrics_path,metadata_path' \
  > "${MANIFEST_REL}"

run_case() {
  local scenario_id="$1"
  local capacity_bps="$2"
  local capacity_mbps="$3"
  local base_rtt_s="$4"
  local base_rtt_ms="$5"
  local buffer_bdp="$6"
  local bottleneck_delay_s="$7"
  local access_bps="$8"
  local purpose="$9"
  local algorithm="${10}"
  local seed="${11}"
  local run_id="${12}"
  local manifest_entry="${13}"
  local mode="original"
  local algorithm_token
  local scenario_raw_rel="${RAW_REL}/${scenario_id}"
  local prefix
  local log_path
  local -a command_args

  if [[ "${algorithm}" == "BBRv2-formal" ]]; then
    mode="ideal"
  fi
  algorithm_token="${algorithm//+/_}"
  algorithm_token="${algorithm_token//-/_}"
  prefix="${scenario_raw_rel}/${algorithm_token}_seed${seed}_run${run_id}"
  log_path="${LOGS_REL}/${scenario_id}/${algorithm_token}_seed${seed}_run${run_id}.log"
  mkdir -p "${scenario_raw_rel}" "${LOGS_REL}/${scenario_id}"
  command_args=(
    "--algorithm=${algorithm}"
    "--seed=${seed}"
    "--runId=${run_id}"
    "--outputDir=${scenario_raw_rel}"
    "--simulationTime=${SIMULATION_TIME}"
    "--bottleneckBps=${capacity_bps}"
    "--accessBps=${access_bps}"
    "--baseRtt=${base_rtt_s}"
    "--bottleneckDelay=${bottleneck_delay_s}"
    "--queueBdp=${buffer_bdp}"
  )
  if [[ "${algorithm}" == "FBBR" ]]; then
    command_args+=("--fbbrConfig=${FBBR_CONFIG_REL}")
  fi

  echo "[run ${run_id}] scenario=${scenario_id} algorithm=${algorithm} seed=${seed}"
  if ! "${BINARY_REL}" "${command_args[@]}" >"${log_path}" 2>&1; then
    tail -n 160 "${log_path}" >&2
    exit 1
  fi
  for suffix in run_summary stage_metrics flow_metrics events minute_metrics minute_flow_metrics; do
    if [[ ! -s "${prefix}_${suffix}.csv" ]]; then
      echo "Missing expected result file for ${prefix}: ${suffix}" >&2
      exit 1
    fi
  done
  if [[ ! -s "${prefix}_metadata.json" ]]; then
    echo "Missing expected metadata for ${prefix}" >&2
    exit 1
  fi
  printf '%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s\n' \
    "${scenario_id}" "${capacity_bps}" "${capacity_mbps}" \
    "${base_rtt_s}" "${base_rtt_ms}" "${buffer_bdp}" \
    "${bottleneck_delay_s}" "${access_bps}" "${purpose}" \
    "${algorithm}" "${mode}" "${seed}" "${run_id}" \
    "${prefix}_run_summary.csv" "${prefix}_stage_metrics.csv" \
    "${prefix}_flow_metrics.csv" "${prefix}_events.csv" \
    "${prefix}_minute_metrics.csv" "${prefix}_minute_flow_metrics.csv" \
    "${prefix}_metadata.json" > "${manifest_entry}"
}

algorithms=(
  BBR-R
  oBBR
  BBRv2+
  CUBIC
  BBRv2-formal
  BBRv2
  FBBR
)

declare -a active_pids=()
declare -a manifest_entries=()
selected_scenarios=0
while IFS=, read -r scenario_id capacity_bps capacity_mbps base_rtt_s base_rtt_ms buffer_bdp bottleneck_delay_s access_bps purpose; do
  if [[ "${scenario_id}" == "scenario_id" || -z "${scenario_id}" ]]; then
    continue
  fi
  if ! scenario_is_selected "${scenario_id}"; then
    continue
  fi
  selected_scenarios=$((selected_scenarios + 1))
  for algorithm_index in "${!algorithms[@]}"; do
    algorithm="${algorithms[algorithm_index]}"
    run_id=$((algorithm_index + 1))
    manifest_entry="${MANIFEST_ENTRIES_REL}/${scenario_id}_$(printf '%02d' "${run_id}").csv"
    manifest_entries+=("${manifest_entry}")
    run_case \
      "${scenario_id}" "${capacity_bps}" "${capacity_mbps}" \
      "${base_rtt_s}" "${base_rtt_ms}" "${buffer_bdp}" \
      "${bottleneck_delay_s}" "${access_bps}" "${purpose}" \
      "${algorithm}" 1 "${run_id}" "${manifest_entry}" &
    active_pids+=("$!")
    if [[ "${#active_pids[@]}" -ge "${JOBS}" ]]; then
      for pid in "${active_pids[@]}"; do
        wait "${pid}"
      done
      active_pids=()
    fi
  done
done < "${SCENARIOS_FILE}"

if [[ "${selected_scenarios}" -eq 0 ]]; then
  echo "No scenarios selected by --scenario=${SCENARIO_FILTER}" >&2
  exit 2
fi
for pid in "${active_pids[@]}"; do
  wait "${pid}"
done
for manifest_entry in "${manifest_entries[@]}"; do
  cat "${manifest_entry}" >> "${MANIFEST_REL}"
done

analysis_args=(
  --results-dir "${RESULTS_REL}"
  --manifest "${MANIFEST_REL}"
)
if [[ "${SMOKE}" -eq 0 && -z "${SCENARIO_FILTER}" ]]; then
  analysis_args+=(--expect-full)
fi
python3 "${SCRIPT_DIR}/analyze_test2.py" "${analysis_args[@]}"
echo "Results: ${NS3_DIR}/${RESULTS_REL}"
