#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
NS3_DIR="$(cd "${SCRIPT_DIR}/../../.." && pwd)"
RESULTS_REL="results/test1"
RAW_REL="${RESULTS_REL}/raw"
LOGS_REL="${RESULTS_REL}/logs"
MANIFEST_REL="${RAW_REL}/manifest.csv"
MANIFEST_ENTRIES_REL="${RAW_REL}/manifest_entries"
TARGET="test1-probe-order"
BINARY_REL="build/examples/paper-test/test1/ns3.27-test1-probe-order-optimized"
FBBR_CONFIG_REL="examples/CCconfig/fbbr_default.conf"

SMOKE=0
SKIP_BUILD=0
JOBS=1
for argument in "$@"; do
  case "${argument}" in
    --smoke) SMOKE=1 ;;
    --skip-build) SKIP_BUILD=1 ;;
    --jobs=*) JOBS="${argument#--jobs=}" ;;
    *)
      echo "Usage: $0 [--smoke] [--skip-build] [--jobs=N]" >&2
      exit 2
      ;;
  esac
done

if [[ ! "${JOBS}" =~ ^[1-9][0-9]*$ ]]; then
  echo "--jobs must be a positive integer" >&2
  exit 2
fi

cd "${NS3_DIR}"
mkdir -p "${RAW_REL}" "${LOGS_REL}" "${RESULTS_REL}/summary" \
  "${RESULTS_REL}/figures" "${MANIFEST_ENTRIES_REL}"
if [[ ! -r "${FBBR_CONFIG_REL}" ]]; then
  echo "Missing FBBR configuration: ${NS3_DIR}/${FBBR_CONFIG_REL}" >&2
  exit 1
fi

# Waf exports public DQC headers as read-only copies. Keep them aligned with
# the experiment's source tree before compiling the selected example target.
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

export LD_LIBRARY_PATH="${NS3_DIR}/build${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
printf '%s\n' \
  'algorithm,mode,seed,run_id,run_summary_path,stage_metrics_path,flow_metrics_path,events_path,minute_metrics_path,minute_flow_metrics_path,metadata_path' \
  > "${MANIFEST_REL}"

run_case() {
  local algorithm="$1"
  local seed="$2"
  local run_id="$3"
  local mode="original"
  local algorithm_token
  local prefix
  local log_path
  local extension
  local -a command_args

  if [[ "${algorithm}" == "BBRv2-ideal" ]]; then
    mode="ideal"
  fi
  algorithm_token="${algorithm//+/_}"
  algorithm_token="${algorithm_token//-/_}"
  prefix="${RAW_REL}/${algorithm_token}_seed${seed}_run${run_id}"
  log_path="${LOGS_REL}/${algorithm_token}_seed${seed}_run${run_id}.log"
  command_args=(
    "--algorithm=${algorithm}"
    "--seed=${seed}"
    "--runId=${run_id}"
    "--outputDir=${RAW_REL}"
    --queueBdp=40
  )
  if [[ "${algorithm}" == "FBBR" ]]; then
    command_args+=("--fbbrConfig=${FBBR_CONFIG_REL}")
  fi
  if [[ "${SMOKE}" -eq 1 ]]; then
    command_args+=(
      --simulationTime=210
      --idealSettle=3
      --strictGap=0.10
      --strictMinUp=0.03
      --strictMaxUp=0.05
    )
  fi

  echo "[run ${run_id}] algorithm=${algorithm} mode=${mode} seed=${seed}"
  if ! "${BINARY_REL}" "${command_args[@]}" >"${log_path}" 2>&1; then
    tail -n 160 "${log_path}" >&2
    exit 1
  fi
  for suffix in run_summary stage_metrics flow_metrics events minute_metrics minute_flow_metrics metadata; do
    extension="csv"
    if [[ "${suffix}" == "metadata" ]]; then
      extension="json"
    fi
    if [[ ! -s "${prefix}_${suffix}.${extension}" ]]; then
      echo "Missing expected result file for ${prefix}: ${suffix}" >&2
      exit 1
    fi
  done
  printf '%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s\n' \
    "${algorithm}" "${mode}" "${seed}" "${run_id}" \
    "${prefix}_run_summary.csv" "${prefix}_stage_metrics.csv" \
    "${prefix}_flow_metrics.csv" "${prefix}_events.csv" \
    "${prefix}_minute_metrics.csv" \
    "${prefix}_minute_flow_metrics.csv" \
    "${prefix}_metadata.json" > "${MANIFEST_ENTRIES_REL}/${run_id}.csv"
}

algorithms=(
  BBR-R
  oBBR
  BBRv2+
  CUBIC
  BBRv2-ideal
  BBRv2
  FBBR
)

active_pids=()
for index in "${!algorithms[@]}"; do
  run_case "${algorithms[index]}" 1 "$((index + 1))" &
  active_pids+=("$!")
  if [[ "${#active_pids[@]}" -ge "${JOBS}" ]]; then
    for pid in "${active_pids[@]}"; do
      wait "${pid}"
    done
    active_pids=()
  fi
done
for pid in "${active_pids[@]}"; do
  wait "${pid}"
done
for ((run_id = 1; run_id <= ${#algorithms[@]}; ++run_id)); do
  cat "${MANIFEST_ENTRIES_REL}/${run_id}.csv" >> "${MANIFEST_REL}"
done

analysis_args=(
  --results-dir "${RESULTS_REL}"
  --manifest "${MANIFEST_REL}"
)
if [[ "${SMOKE}" -eq 0 ]]; then
  analysis_args+=(--expect-full)
fi
python3 "${SCRIPT_DIR}/analyze_test1.py" "${analysis_args[@]}"
echo "Results: ${NS3_DIR}/${RESULTS_REL}"
