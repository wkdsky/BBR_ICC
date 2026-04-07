#!/bin/bash

#################################################################################
# FreqCCv3 Batch Runner Script
#
# Usage: ./run_freqccv3_batch.sh [OPTIONS]
#
# Options:
#   --flows 4,8,16,32           Comma-separated list of flow counts to run
#   --sender-bw <Mbps>          Edge link bandwidth (flow-specific defaults if not set)
#   --bottle-bw <Mbps>          Bottleneck bandwidth (flow-specific defaults if not set)
#   --bottle-delay <ms>         Bottleneck delay (default: flow-specific)
#   --queue-bdp <factor>        Queue size in BDP units (default: 1)
#   --sim-time <seconds>        Simulation duration (default: 30 s)
#   --freqall <Hz>              Apply one frequency to all active flows
#   --ampall <mode>             Apply one amplitude mode to all active flows
#   --fixedall <Mbps>           Apply one fixed amplitude to all active flows
#   --freqN <Hz>                Flow N oscillation frequency, N=1..32
#   --ampN <mode>               Flow N amplitude mode, N=1..32
#   --fixedN <Mbps>             Flow N fixed amplitude, N=1..32
#   --interval-win-rtt-mult <x> Interval-phase STFT window multiplier on min_rtt
#   --instance <id>             Instance number (default: 1)
#   --rebuild                   Force rebuild even if no changes
#   --help                      Show this help message
#
# Examples:
#   ./run_freqccv3_batch.sh
#   ./run_freqccv3_batch.sh --flows 8 --sender-bw 10 --bottle-bw 16 --sim-time 60
#   ./run_freqccv3_batch.sh --flows 4,8 --bottle-delay 40 --queue-bdp 2
#   ./run_freqccv3_batch.sh --flows 8 --freqall 60 --ampall miu2 --fixedall 0
#   ./run_freqccv3_batch.sh --flows 16 --freqall 60 --ampall miu2 --interval-win-rtt-mult 1.5
#   ./run_freqccv3_batch.sh --flows 4 --freq1 80 --amp1 miu2 --fixed1 2.5

#  cd /home/wkd/BBR_ICC/NS3.27
#  ./run_freqccv3_batch.sh --flows 4 --sender-bw 8 --bottle-bw 20 --bottle-delay 18 --queue-bdp 1 --sim-time 120 --freqall 60 --ampall miu2
#################################################################################

set -euo pipefail

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

FLOWS="4,8,16,32"
SIM_TIME=30
INSTANCE=1
REBUILD=false
CUSTOM_SENDER_BW=""
CUSTOM_BOTTLE_BW=""
CUSTOM_BOTTLE_DELAY=""
QUEUE_BDP=1
RUN_TAG="$(date +%Y%m%d_%H%M%S)"
TRACE_ROOT=""
ACTIVE_SCRIPT_FILE=""

INTERVAL_WIN_RTT_MULT=""

declare -A DEFAULT_SENDER_BW
DEFAULT_SENDER_BW[4]=8
DEFAULT_SENDER_BW[8]=10
DEFAULT_SENDER_BW[16]=10
DEFAULT_SENDER_BW[32]=10

declare -A DEFAULT_BOTTLE_BW
DEFAULT_BOTTLE_BW[4]=20
DEFAULT_BOTTLE_BW[8]=16
DEFAULT_BOTTLE_BW[16]=32
DEFAULT_BOTTLE_BW[32]=64

declare -A DEFAULT_BOTTLE_DELAY
DEFAULT_BOTTLE_DELAY[4]=18
DEFAULT_BOTTLE_DELAY[8]=28
DEFAULT_BOTTLE_DELAY[16]=18
DEFAULT_BOTTLE_DELAY[32]=28

declare -A FLOW_FREQ_OVERRIDE
declare -A FLOW_AMP_OVERRIDE
declare -A FLOW_FIXED_OVERRIDE
FLOW_FREQ_ALL=""
FLOW_AMP_ALL=""
FLOW_FIXED_ALL=""

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
NS3_DIR="$SCRIPT_DIR"
SCRATCH_DIR="$NS3_DIR/scratch"

while [[ $# -gt 0 ]]; do
    case $1 in
        --flows)
            FLOWS="$2"
            shift 2
            ;;
        --sender-bw)
            CUSTOM_SENDER_BW="$2"
            shift 2
            ;;
        --bottle-bw)
            CUSTOM_BOTTLE_BW="$2"
            shift 2
            ;;
        --bottle-delay)
            CUSTOM_BOTTLE_DELAY="$2"
            shift 2
            ;;
        --queue-bdp)
            QUEUE_BDP="$2"
            shift 2
            ;;
        --sim-time)
            SIM_TIME="$2"
            shift 2
            ;;
        --freqall)
            FLOW_FREQ_ALL="$2"
            shift 2
            ;;
        --ampall)
            FLOW_AMP_ALL="$2"
            shift 2
            ;;
        --fixedall)
            FLOW_FIXED_ALL="$2"
            shift 2
            ;;
        --interval-win-rtt-mult)
            INTERVAL_WIN_RTT_MULT="$2"
            shift 2
            ;;
        --instance)
            INSTANCE="$2"
            shift 2
            ;;
        --rebuild)
            REBUILD=true
            shift
            ;;
        --help)
            sed -n '3,31p' "$0" | sed 's/^# \{0,1\}//'
            exit 0
            ;;
        *)
            if [[ "$1" =~ ^--(freq|amp|fixed)([0-9]+)$ ]]; then
                kind="${BASH_REMATCH[1]}"
                idx="${BASH_REMATCH[2]}"
                if (( idx < 1 || idx > 32 )); then
                    echo -e "${RED}Error: ${kind}${idx} is out of supported range 1..32${NC}"
                    exit 1
                fi
                case "$kind" in
                    freq)
                        FLOW_FREQ_OVERRIDE["$idx"]="$2"
                        ;;
                    amp)
                        FLOW_AMP_OVERRIDE["$idx"]="$2"
                        ;;
                    fixed)
                        FLOW_FIXED_OVERRIDE["$idx"]="$2"
                        ;;
                esac
                shift 2
            else
                echo -e "${RED}Unknown option: $1${NC}"
                exit 1
            fi
            ;;
    esac
done

print_header() {
    echo ""
    echo -e "${BLUE}╔════════════════════════════════════════════════════════════════╗${NC}"
    echo -e "${BLUE}║${NC} $1"
    echo -e "${BLUE}╚════════════════════════════════════════════════════════════════╝${NC}"
    echo ""
}

update_script_params() {
    local script_file=$1
    local sender_bw=$2
    local bottle_bw=$3
    local bottle_delay=$4
    local queue_bdp=$5

    if [ ! -f "$script_file" ]; then
        echo -e "${RED}Error: Script not found: $script_file${NC}"
        return 1
    fi

    cp "$script_file" "$script_file.bak"

    local sender_bps
    local bottle_bps
    local rtt_ms
    local qdelay_ms
    sender_bps=$(awk -v bw="$sender_bw" 'BEGIN { printf "%.0f", bw * 1000000 }')
    bottle_bps=$(awk -v bw="$bottle_bw" 'BEGIN { printf "%.0f", bw * 1000000 }')
    rtt_ms=$(((2 * 1 + bottle_delay) * 2))
    qdelay_ms=$(awk -v q="$queue_bdp" -v rtt="$rtt_ms" 'BEGIN { printf "%.0f", q * rtt }')

    sed -i "s/const uint64_t TOPO_SENDER_BW.*=.*/const uint64_t TOPO_SENDER_BW       =   $sender_bps;    \\/\\/ in bps/" "$script_file"
    sed -i "s/const uint64_t TOPO_BOTTLE_BW.*=.*/const uint64_t TOPO_BOTTLE_BW       =   $bottle_bps;    \\/\\/ in bps/" "$script_file"
    sed -i "s/const uint64_t TOPO_BOTTLE_PDELAY.*=.*/const uint64_t TOPO_BOTTLE_PDELAY   =   $bottle_delay;    \\/\\/ in ms/" "$script_file"
    sed -i "s/const uint64_t TOPO_DEFAULT_QDELAY.*=.*/const uint64_t TOPO_DEFAULT_QDELAY  =   $qdelay_ms;    \\/\\/ in ms/" "$script_file"

    echo -e "${GREEN}✓${NC} Updated: $(basename "$script_file")"
    echo "  TOPO_SENDER_BW=$sender_bw Mbps, TOPO_BOTTLE_BW=$bottle_bw Mbps, TOPO_BOTTLE_PDELAY=$bottle_delay ms, TOPO_DEFAULT_QDELAY=$qdelay_ms ms (${queue_bdp}BDP)"
}

restore_script() {
    local script_file=$1
    if [ -f "$script_file.bak" ]; then
        mv "$script_file.bak" "$script_file"
    fi
}

build_flow_args() {
    local num_flows=$1
    FLOW_ARGS=()

    for idx in "${!FLOW_FREQ_OVERRIDE[@]}"; do
        if (( idx > num_flows )); then
            echo -e "${RED}Error: --freq${idx} exceeds flow count ${num_flows}${NC}"
            return 1
        fi
    done

    for idx in "${!FLOW_AMP_OVERRIDE[@]}"; do
        if (( idx > num_flows )); then
            echo -e "${RED}Error: --amp${idx} exceeds flow count ${num_flows}${NC}"
            return 1
        fi
    done

    for idx in "${!FLOW_FIXED_OVERRIDE[@]}"; do
        if (( idx > num_flows )); then
            echo -e "${RED}Error: --fixed${idx} exceeds flow count ${num_flows}${NC}"
            return 1
        fi
    done

    local i
    for ((i = 1; i <= num_flows; i++)); do
        if [[ -n "$FLOW_FREQ_ALL" ]]; then
            FLOW_ARGS+=("--freq${i}=${FLOW_FREQ_ALL}")
        fi
        if [[ -n "$FLOW_AMP_ALL" ]]; then
            FLOW_ARGS+=("--amp${i}=${FLOW_AMP_ALL}")
        fi
        if [[ -n "$FLOW_FIXED_ALL" ]]; then
            FLOW_ARGS+=("--fixed${i}=${FLOW_FIXED_ALL}")
        fi
        if [[ -n "${FLOW_FREQ_OVERRIDE[$i]+x}" ]]; then
            FLOW_ARGS+=("--freq${i}=${FLOW_FREQ_OVERRIDE[$i]}")
        fi
        if [[ -n "${FLOW_AMP_OVERRIDE[$i]+x}" ]]; then
            FLOW_ARGS+=("--amp${i}=${FLOW_AMP_OVERRIDE[$i]}")
        fi
        if [[ -n "${FLOW_FIXED_OVERRIDE[$i]+x}" ]]; then
            FLOW_ARGS+=("--fixed${i}=${FLOW_FIXED_OVERRIDE[$i]}")
        fi
    done
}

print_flow_overrides() {
    local num_flows=$1
    local has_all_defaults=false
    local has_single_overrides=false
    local i

    if [[ -n "$FLOW_FREQ_ALL" || -n "$FLOW_AMP_ALL" || -n "$FLOW_FIXED_ALL" ]]; then
        echo "  All-flow defaults: freq=${FLOW_FREQ_ALL:-default}, amp=${FLOW_AMP_ALL:-default}, fixed=${FLOW_FIXED_ALL:-default}"
        has_all_defaults=true
    fi

    for ((i = 1; i <= num_flows; i++)); do
        if [[ -n "${FLOW_FREQ_OVERRIDE[$i]+x}" || -n "${FLOW_AMP_OVERRIDE[$i]+x}" || -n "${FLOW_FIXED_OVERRIDE[$i]+x}" ]]; then
            if [[ "$has_single_overrides" == false ]]; then
                echo "  Flow overrides:"
                has_single_overrides=true
            fi
            echo "    flow${i}: freq=${FLOW_FREQ_OVERRIDE[$i]:-default}, amp=${FLOW_AMP_OVERRIDE[$i]:-default}, fixed=${FLOW_FIXED_OVERRIDE[$i]:-default}"
        fi
    done
}

cleanup() {
    if [ -n "$ACTIVE_SCRIPT_FILE" ]; then
        restore_script "$ACTIVE_SCRIPT_FILE"
        ACTIVE_SCRIPT_FILE=""
    fi
}

trap cleanup EXIT INT TERM

run_test() {
    local num_flows=$1
    local sender_bw=$2
    local bottle_bw=$3
    local bottle_delay=$4
    local queue_bdp=$5
    local sim_time=$6

    local script="freqccv3_${num_flows}flow"
    local script_file="$SCRATCH_DIR/${script}.cc"
    local sender_bw_tag="${sender_bw//./p}"
    local bottle_bw_tag="${bottle_bw//./p}"
    local queue_bdp_tag="${queue_bdp//./p}"
    local scenario_dir="${TRACE_ROOT}/${num_flows}f_sender${sender_bw_tag}M_bottle${bottle_bw_tag}M_delay${bottle_delay}ms_q${queue_bdp_tag}bdp_time${sim_time}s"
    local trace_path="${scenario_dir}/"
    local run_log="${scenario_dir}/run.log"
    local bin_path="build/scratch/${script}"
    local run_extra_args=""

    if [ ! -f "$script_file" ]; then
        echo -e "${RED}✗ Script not found: $script_file${NC}"
        return 1
    fi

    if ! build_flow_args "$num_flows"; then
        return 1
    fi

    for arg in "${FLOW_ARGS[@]}"; do
        run_extra_args+=" ${arg}"
    done
    if [[ -n "$INTERVAL_WIN_RTT_MULT" ]]; then
        run_extra_args+=" --interval_win_rtt_mult=${INTERVAL_WIN_RTT_MULT}"
    fi

    print_header "Running $num_flows flows: SENDER_BW=${sender_bw}Mbps, BOTTLE_BW=${bottle_bw}Mbps, DELAY=${bottle_delay}ms, QUEUE=${queue_bdp}BDP, SIM_TIME=${sim_time}s"
    print_flow_overrides "$num_flows"

    mkdir -p "$scenario_dir"

    update_script_params "$script_file" "$sender_bw" "$bottle_bw" "$bottle_delay" "$queue_bdp"
    ACTIVE_SCRIPT_FILE="$script_file"

    echo -e "${BLUE}Building...${NC}"
    cd "$NS3_DIR"
    if $REBUILD || [ ! -f "$bin_path" ] || [ "$script_file" -nt "$bin_path" ]; then
        if ! ./waf build --targets="${script}" > /dev/null 2>&1; then
            echo -e "${RED}✗ Build failed for ${script}${NC}"
            restore_script "$script_file"
            ACTIVE_SCRIPT_FILE=""
            return 1
        fi
    fi

    echo -e "${BLUE}Running simulation...${NC}"
    local start_time
    local end_time
    local duration
    start_time=$(date +%s)

    if ! ./waf --run "${script} --sim_time=${sim_time} --trace_path=${trace_path}${run_extra_args}" 2>&1 | tee "$run_log"; then
        restore_script "$script_file"
        ACTIVE_SCRIPT_FILE=""
        return 1
    fi

    end_time=$(date +%s)
    duration=$((end_time - start_time))

    echo -e "${GREEN}✓ Completed in ${duration}s${NC}"
    echo ""

    restore_script "$script_file"
    ACTIVE_SCRIPT_FILE=""
}

print_header "FreqCCv3 Batch Runner"

TRACE_ROOT="traces/freqccv3_batch_ins${INSTANCE}_${RUN_TAG}"
mkdir -p "$TRACE_ROOT"

echo -e "${BLUE}Configuration:${NC}"
echo "  Flows: $FLOWS"
echo "  Sender BW: ${CUSTOM_SENDER_BW:-auto} Mbps"
echo "  Bottleneck BW: ${CUSTOM_BOTTLE_BW:-auto} Mbps"
echo "  Bottleneck Delay: ${CUSTOM_BOTTLE_DELAY:-auto} ms"
echo "  Queue: $QUEUE_BDP BDP"
echo "  Simulation Time: $SIM_TIME seconds"
echo "  Instance: $INSTANCE"
echo "  Trace Root: $TRACE_ROOT"
echo "  Interval Window RTT Multiplier: ${INTERVAL_WIN_RTT_MULT:-default}"
if [[ -n "$FLOW_FREQ_ALL" || -n "$FLOW_AMP_ALL" || -n "$FLOW_FIXED_ALL" || ${#FLOW_FREQ_OVERRIDE[@]} -gt 0 || ${#FLOW_AMP_OVERRIDE[@]} -gt 0 || ${#FLOW_FIXED_OVERRIDE[@]} -gt 0 ]]; then
    echo "  Per-flow overrides: enabled"
fi
echo ""

IFS=',' read -ra FLOW_ARRAY <<< "$FLOWS"
total_tests=${#FLOW_ARRAY[@]}
current_test=0

for num_flows in "${FLOW_ARRAY[@]}"; do
    num_flows=$(echo "$num_flows" | xargs)
    current_test=$((current_test + 1))

    if [ -n "$CUSTOM_SENDER_BW" ]; then
        sender_bw=$CUSTOM_SENDER_BW
    else
        sender_bw=${DEFAULT_SENDER_BW[$num_flows]:-10}
    fi

    if [ -n "$CUSTOM_BOTTLE_BW" ]; then
        bottle_bw=$CUSTOM_BOTTLE_BW
    else
        bottle_bw=${DEFAULT_BOTTLE_BW[$num_flows]:-$((num_flows * 2))}
    fi

    if [ -n "$CUSTOM_BOTTLE_DELAY" ]; then
        bottle_delay=$CUSTOM_BOTTLE_DELAY
    else
        bottle_delay=${DEFAULT_BOTTLE_DELAY[$num_flows]:-28}
    fi

    echo -e "${YELLOW}[${current_test}/${total_tests}]${NC} Testing with $num_flows flows..."

    if run_test "$num_flows" "$sender_bw" "$bottle_bw" "$bottle_delay" "$QUEUE_BDP" "$SIM_TIME"; then
        echo -e "${GREEN}✓ Test completed successfully${NC}"
    else
        echo -e "${RED}✗ Test failed${NC}"
    fi

    echo ""
done

print_header "All Tests Completed"
echo -e "${GREEN}Trace files saved to: ${TRACE_ROOT}/${NC}"
echo ""
ls -lh "$TRACE_ROOT" | tail -n +2 | head -20
echo ""

echo -e "${BLUE}Summary:${NC}"
echo "  Total tests: $total_tests"
echo "  Results saved under: $TRACE_ROOT"
echo ""
