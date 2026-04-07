#!/usr/bin/env bash

set -euo pipefail

usage() {
    cat <<'EOF'
Usage: scripts/showcase/run_showcase_scenario.sh [OPTIONS]

Run one ns-3 showcase scenario and optionally render PNG figures.

Options:
  --algo <bbrv2|bbrv2plus|bbrv2plus_ecn|bbrv2_noprobe_rtt|obbr|freqccv3>
  --flows <4|8|16|32>
  --sim-time <seconds>
  --trace-dir <path>
  --fig-dir <path>
  --title <text>
  --skip-plot
  --rebuild
  --help

Notes:
  - bbrv2 maps to scratch/bbrv2_<flows>flow.cc.
  - bbrv2plus maps to scratch/bbrv2plus_<flows>flow.cc.
  - obbr maps to scratch/obbr_<flows>flow.cc.
  - freqccv3 maps to scratch/freqccv3_<flows>flow.cc.
EOF
}

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
NS3_DIR="$(cd "${SCRIPT_DIR}/../.." && pwd)"

ALGO="bbrv2plus"
FLOWS="4"
SIM_TIME="10"
TRACE_DIR=""
FIG_DIR=""
TITLE=""
SKIP_PLOT=0
REBUILD=0

while [[ $# -gt 0 ]]; do
    case "$1" in
        --algo)
            ALGO="$2"
            shift 2
            ;;
        --flows)
            FLOWS="$2"
            shift 2
            ;;
        --sim-time)
            SIM_TIME="$2"
            shift 2
            ;;
        --trace-dir)
            TRACE_DIR="$2"
            shift 2
            ;;
        --fig-dir)
            FIG_DIR="$2"
            shift 2
            ;;
        --title)
            TITLE="$2"
            shift 2
            ;;
        --skip-plot)
            SKIP_PLOT=1
            shift
            ;;
        --rebuild)
            REBUILD=1
            shift
            ;;
        --help)
            usage
            exit 0
            ;;
        *)
            echo "Unknown option: $1" >&2
            usage >&2
            exit 1
            ;;
    esac
done

case "$ALGO" in
    obbr)
        SCENARIO="obbr_${FLOWS}flow"
        RESULT_GROUP="oBBR"
        ALGO_TAG="obbr"
        RUN_ARGS="--sim_time=${SIM_TIME}"
        ;;
    bbrv2)
        SCENARIO="bbrv2_${FLOWS}flow"
        RESULT_GROUP="BBRv2"
        ALGO_TAG="bbrv2"
        RUN_ARGS="--sim_time=${SIM_TIME} --cc=${ALGO}"
        ;;
    bbrv2_noprobe_rtt)
        SCENARIO="bbrv2_${FLOWS}flow"
        RESULT_GROUP="BBRv2"
        ALGO_TAG="bbrv2_noprobe_rtt"
        RUN_ARGS="--sim_time=${SIM_TIME} --cc=${ALGO}"
        ;;
    bbrv2plus|bbrv2plus_ecn)
        SCENARIO="bbrv2plus_${FLOWS}flow"
        RESULT_GROUP="BBRv2plus"
        ALGO_TAG="${ALGO}"
        RUN_ARGS="--sim_time=${SIM_TIME} --cc=${ALGO}"
        ;;
    freqccv3)
        SCENARIO="freqccv3_${FLOWS}flow"
        RESULT_GROUP="FreqCCv3"
        ALGO_TAG="freqccv3"
        RUN_ARGS="--sim_time=${SIM_TIME}"
        ;;
    *)
        echo "Unsupported --algo value: ${ALGO}" >&2
        exit 1
        ;;
esac

SCENARIO_FILE="${NS3_DIR}/scratch/${SCENARIO}.cc"
if [[ ! -f "${SCENARIO_FILE}" ]]; then
    echo "Scenario source not found: ${SCENARIO_FILE}" >&2
    exit 1
fi

if [[ -z "${TRACE_DIR}" ]]; then
    TRACE_DIR="${NS3_DIR}/showcase_results/${RESULT_GROUP}/traces/${ALGO_TAG}_${FLOWS}flow_demo"
fi
if [[ -z "${FIG_DIR}" ]]; then
    FIG_DIR="${NS3_DIR}/showcase_results/${RESULT_GROUP}/figs/${ALGO_TAG}_${FLOWS}flow_demo"
fi
if [[ -z "${TITLE}" ]]; then
    TITLE="${ALGO_TAG} ${FLOWS}-flow"
fi

mkdir -p "${TRACE_DIR}"
mkdir -p "${FIG_DIR}"

cd "${NS3_DIR}"

export MPLCONFIGDIR="${TMPDIR:-/tmp}/mplconfig_${USER:-user}"
mkdir -p "${MPLCONFIGDIR}"

if [[ "${REBUILD}" -eq 1 ]]; then
    ./waf build --targets="${SCENARIO}"
fi

./waf --run "${SCENARIO} ${RUN_ARGS} --trace_path=${TRACE_DIR}/"

if [[ "${SKIP_PLOT}" -eq 0 ]]; then
    python3 "${NS3_DIR}/scripts/showcase/plot_trace_bundle.py" \
        --trace-dir "${TRACE_DIR}" \
        --out-dir "${FIG_DIR}" \
        --title "${TITLE}"
fi

echo "Trace directory: ${TRACE_DIR}"
if [[ "${SKIP_PLOT}" -eq 0 ]]; then
    echo "Figure directory: ${FIG_DIR}"
fi
