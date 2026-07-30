#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
NS3_DIR="$(cd "${SCRIPT_DIR}/../../.." && pwd)"
SCENARIOS_FILE="${SCRIPT_DIR}/scenarios.csv"
RESULTS_REL="results/test3"
RAW_REL="${RESULTS_REL}/raw"
LOGS_REL="${RESULTS_REL}/logs"
MANIFEST_REL="${RAW_REL}/manifest.csv"
MANIFEST_ENTRIES_REL="${RAW_REL}/manifest_entries"
TARGET="test3-dynamic-rtt"
BINARY_REL="build/examples/paper-test/test3/ns3.27-test3-dynamic-rtt-optimized"
FBBR_CONFIG_REL="examples/CCconfig/fbbr_default.conf"

SKIP_BUILD=0
JOBS=1
ALGORITHM_FILTER=""
for argument in "$@"; do
  case "${argument}" in
    --skip-build) SKIP_BUILD=1 ;;
    --jobs=*) JOBS="${argument#--jobs=}" ;;
    --algorithm=*) ALGORITHM_FILTER="${argument#--algorithm=}" ;;
    *)
      echo "Usage: $0 [--skip-build] [--jobs=N] [--algorithm=NAME[,NAME...]]" >&2
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

algorithm_is_selected() {
  local algorithm="$1"
  [[ -z "${ALGORITHM_FILTER}" || ",${ALGORITHM_FILTER}," == *",${algorithm},"* ]]
}

cd "${NS3_DIR}"
mkdir -p "${RAW_REL}" "${LOGS_REL}" "${RESULTS_REL}/summary" \
  "${RESULTS_REL}/figures" "${MANIFEST_ENTRIES_REL}"
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

sha256sum "${FBBR_CONFIG_REL}" > "${RESULTS_REL}/fbbr_default.conf.sha256"
export LD_LIBRARY_PATH="${NS3_DIR}/build${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"

printf '%s\n' \
  'scenario_id,capacity_bps,capacity_mbps,initial_queue_bdp,bottleneck_delay_s,access_bps,simulation_time_s,sample_interval_s,settle_guard_s,propagation_rtt_profile,purpose,algorithm,mode,seed,run_id,run_summary_path,rtt_profile_path,rtt_timeseries_path,metadata_path' \
  > "${MANIFEST_REL}"

run_case() {
  local scenario_id="$1"
  local capacity_bps="$2"
  local capacity_mbps="$3"
  local initial_queue_bdp="$4"
  local bottleneck_delay_s="$5"
  local access_bps="$6"
  local simulation_time_s="$7"
  local sample_interval_s="$8"
  local settle_guard_s="$9"
  local profile_pipe="${10}"
  local purpose="${11}"
  local algorithm="${12}"
  local run_id="${13}"
  local manifest_entry="${14}"
  local profile="${profile_pipe//|/,}"
  local mode="original"
  local algorithm_token="${algorithm//+/_}"
  algorithm_token="${algorithm_token//-/_}"
  if [[ "${algorithm}" == "BBRv2-formal" ]]; then
    mode="ideal"
  fi
  local scenario_raw_rel="${RAW_REL}/${scenario_id}"
  local prefix="${scenario_raw_rel}/${algorithm_token}_seed1_run${run_id}"
  local log_path="${LOGS_REL}/${scenario_id}/${algorithm_token}_seed1_run${run_id}.log"
  mkdir -p "${scenario_raw_rel}" "${LOGS_REL}/${scenario_id}"

  local -a command_args=(
    "--algorithm=${algorithm}"
    "--seed=1"
    "--runId=${run_id}"
    "--outputDir=${scenario_raw_rel}"
    "--bottleneckBps=${capacity_bps}"
    "--accessBps=${access_bps}"
    "--bottleneckDelay=${bottleneck_delay_s}"
    "--initialQueueBdp=${initial_queue_bdp}"
    "--simulationTime=${simulation_time_s}"
    "--sampleInterval=${sample_interval_s}"
    "--propagationRttProfile=${profile}"
  )
  if [[ "${algorithm}" == "FBBR" ]]; then
    command_args+=("--fbbrConfig=${FBBR_CONFIG_REL}")
  fi

  echo "[run ${run_id}] scenario=${scenario_id} algorithm=${algorithm}"
  if ! "${BINARY_REL}" "${command_args[@]}" >"${log_path}" 2>&1; then
    tail -n 160 "${log_path}" >&2
    exit 1
  fi
  for suffix in run_summary rtt_profile rtt_timeseries; do
    if [[ ! -s "${prefix}_${suffix}.csv" ]]; then
      echo "Missing expected output: ${prefix}_${suffix}.csv" >&2
      exit 1
    fi
  done
  if [[ ! -s "${prefix}_metadata.json" ]]; then
    echo "Missing expected output: ${prefix}_metadata.json" >&2
    exit 1
  fi
  printf '%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s\n' \
    "${scenario_id}" "${capacity_bps}" "${capacity_mbps}" \
    "${initial_queue_bdp}" "${bottleneck_delay_s}" "${access_bps}" \
    "${simulation_time_s}" "${sample_interval_s}" "${settle_guard_s}" \
    "${profile_pipe}" "${purpose}" "${algorithm}" "${mode}" "1" "${run_id}" \
    "${prefix}_run_summary.csv" "${prefix}_rtt_profile.csv" \
    "${prefix}_rtt_timeseries.csv" "${prefix}_metadata.json" \
    > "${manifest_entry}"
}

algorithms=(BBR-R oBBR BBRv2+ CUBIC BBRv2-formal BBRv2 FBBR)
declare -a active_pids=()
declare -a manifest_entries=()
while IFS=, read -r scenario_id capacity_bps capacity_mbps initial_queue_bdp \
    bottleneck_delay_s access_bps simulation_time_s sample_interval_s \
    settle_guard_s propagation_rtt_profile purpose; do
  if [[ "${scenario_id}" == "scenario_id" || -z "${scenario_id}" ]]; then
    continue
  fi
  for algorithm_index in "${!algorithms[@]}"; do
    algorithm="${algorithms[algorithm_index]}"
    if ! algorithm_is_selected "${algorithm}"; then
      continue
    fi
    run_id=$((algorithm_index + 1))
    manifest_entry="${MANIFEST_ENTRIES_REL}/${scenario_id}_$(printf '%02d' "${run_id}").csv"
    manifest_entries+=("${manifest_entry}")
    run_case \
      "${scenario_id}" "${capacity_bps}" "${capacity_mbps}" \
      "${initial_queue_bdp}" "${bottleneck_delay_s}" "${access_bps}" \
      "${simulation_time_s}" "${sample_interval_s}" "${settle_guard_s}" \
      "${propagation_rtt_profile}" "${purpose}" "${algorithm}" \
      "${run_id}" "${manifest_entry}" &
    active_pids+=("$!")
    if [[ "${#active_pids[@]}" -ge "${JOBS}" ]]; then
      for pid in "${active_pids[@]}"; do
        wait "${pid}"
      done
      active_pids=()
    fi
  done
done < "${SCENARIOS_FILE}"

if [[ "${#manifest_entries[@]}" -eq 0 ]]; then
  echo "No algorithms selected by --algorithm=${ALGORITHM_FILTER}" >&2
  exit 2
fi
for pid in "${active_pids[@]}"; do
  wait "${pid}"
done
for manifest_entry in "${manifest_entries[@]}"; do
  while IFS= read -r manifest_line || [[ -n "${manifest_line}" ]]; do
    printf '%s\n' "${manifest_line}" >> "${MANIFEST_REL}"
  done < "${manifest_entry}"
done

python3 "${SCRIPT_DIR}/analyze_test3.py" \
  --results-dir "${RESULTS_REL}" \
  --manifest "${MANIFEST_REL}"
echo "Results: ${NS3_DIR}/${RESULTS_REL}"
