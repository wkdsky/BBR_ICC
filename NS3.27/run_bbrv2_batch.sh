#!/bin/bash

#################################################################################
# BBRv2 Batch Runner Script
#
# Usage: ./run_bbrv2_batch.sh [OPTIONS]
#
# Options:
#   --flows 4,8,16,32           Comma-separated list of flow counts to run
#   --sender-bw <Mbps>          Edge link bandwidth (default: 10 Mbps)
#   --bottle-bw <Mbps>          Bottleneck bandwidth (auto if not set)
#   --bottle-delay <ms>         Bottleneck delay (default: 28 ms)
#   --sim-time <seconds>        Simulation duration (default: 30 s)
#   --instance <id>             Instance number (default: 1)
#   --rebuild                   Force rebuild even if no changes
#   --help                       Show this help message
#
# Examples:
#   # Run all (4,8,16,32) with defaults
#   ./run_bbrv2_batch.sh
#
#   # Run only 8 flows with custom parameters
#   ./run_bbrv2_batch.sh --flows 8 --sender-bw 10 --bottle-bw 16 --sim-time 60
#
#   # Run 4,16 flows with custom bottleneck delay
#   ./run_bbrv2_batch.sh --flows 4,16 --bottle-delay 50 --sim-time 120
#################################################################################

set -e

# Color output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Default values
FLOWS="4,8,16,32"
SENDER_BW=10
BOTTLE_DELAY=28
SIM_TIME=30
INSTANCE=1
REBUILD=false

# Flow-specific defaults
declare -A DEFAULT_BOTTLE_BW
DEFAULT_BOTTLE_BW[4]=8
DEFAULT_BOTTLE_BW[8]=16
DEFAULT_BOTTLE_BW[16]=32
DEFAULT_BOTTLE_BW[32]=64

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
NS3_DIR="$SCRIPT_DIR"
SCRATCH_DIR="$NS3_DIR/scratch"

# Parse arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --flows)
            FLOWS="$2"
            shift 2
            ;;
        --sender-bw)
            SENDER_BW="$2"
            shift 2
            ;;
        --bottle-bw)
            CUSTOM_BOTTLE_BW="$2"
            shift 2
            ;;
        --bottle-delay)
            BOTTLE_DELAY="$2"
            shift 2
            ;;
        --sim-time)
            SIM_TIME="$2"
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
            head -n 30 "$0" | tail -n +2 | sed 's/^# //'
            exit 0
            ;;
        *)
            echo -e "${RED}Unknown option: $1${NC}"
            exit 1
            ;;
    esac
done

# Function to print section header
print_header() {
    echo ""
    echo -e "${BLUE}╔════════════════════════════════════════════════════════════════╗${NC}"
    echo -e "${BLUE}║${NC} $1"
    echo -e "${BLUE}╚════════════════════════════════════════════════════════════════╝${NC}"
    echo ""
}

# Function to update C++ constants in a script
update_script_params() {
    local script_file=$1
    local sender_bw=$2
    local bottle_bw=$3
    local bottle_delay=$4

    if [ ! -f "$script_file" ]; then
        echo -e "${RED}Error: Script not found: $script_file${NC}"
        return 1
    fi

    # Create backup
    cp "$script_file" "$script_file.bak"

    # Convert Mbps to bps
    local sender_bps=$((sender_bw * 1000000))
    local bottle_bps=$((bottle_bw * 1000000))

    # Update TOPO_SENDER_BW
    sed -i "s/const uint64_t TOPO_SENDER_BW.*=.*/const uint64_t TOPO_SENDER_BW       =   $sender_bps;    \/\/ in bps/" "$script_file"

    # Update TOPO_BOTTLE_BW
    sed -i "s/const uint64_t TOPO_BOTTLE_BW.*=.*/const uint64_t TOPO_BOTTLE_BW       =   $bottle_bps;    \/\/ in bps/" "$script_file"

    # Update TOPO_BOTTLE_PDELAY
    sed -i "s/const uint64_t TOPO_BOTTLE_PDELAY.*=.*/const uint64_t TOPO_BOTTLE_PDELAY   =   $bottle_delay;    \/\/ in ms/" "$script_file"

    echo -e "${GREEN}✓${NC} Updated: $(basename $script_file)"
    echo "  TOPO_SENDER_BW=$sender_bw Mbps, TOPO_BOTTLE_BW=$bottle_bw Mbps, TOPO_BOTTLE_PDELAY=$bottle_delay ms"
}

# Function to run a single test
run_test() {
    local num_flows=$1
    local sender_bw=$2
    local bottle_bw=$3
    local bottle_delay=$4
    local sim_time=$5

    local script="${num_flows}_bbrv2"
    local script_file="$SCRATCH_DIR/${script}.cc"

    # Check if script exists
    if [ ! -f "$script_file" ]; then
        echo -e "${RED}✗ Script not found: $script_file${NC}"
        return 1
    fi

    print_header "Running $num_flows flows: SENDER_BW=${sender_bw}Mbps, BOTTLE_BW=${bottle_bw}Mbps, DELAY=${bottle_delay}ms, SIM_TIME=${sim_time}s"

    # Update script parameters
    update_script_params "$script_file" "$sender_bw" "$bottle_bw" "$bottle_delay"

    # Build if needed
    echo -e "${BLUE}Building...${NC}"
    cd "$NS3_DIR"
    if $REBUILD || [ ! -f "build/scratch/${script}" ]; then
        ./waf build --targets=scratch/${script} > /dev/null 2>&1 || {
            echo -e "${RED}✗ Build failed for ${script}${NC}"
            return 1
        }
    fi

    # Run the test
    echo -e "${BLUE}Running simulation...${NC}"
    start_time=$(date +%s)

    ./waf --run "scratch/${script} --sim_time=${sim_time}" 2>&1 | tee "test_${script}_${sender_bw}mbps_${bottle_bw}mbps.log"

    end_time=$(date +%s)
    duration=$((end_time - start_time))

    echo -e "${GREEN}✓ Completed in ${duration}s${NC}"
    echo ""

    # Restore original script
    if [ -f "$script_file.bak" ]; then
        mv "$script_file.bak" "$script_file"
    fi
}

# Main execution
print_header "BBRv2 Batch Runner"

echo -e "${BLUE}Configuration:${NC}"
echo "  Flows: $FLOWS"
echo "  Sender BW: $SENDER_BW Mbps"
echo "  Bottleneck Delay: $BOTTLE_DELAY ms"
echo "  Simulation Time: $SIM_TIME seconds"
echo "  Instance: $INSTANCE"
echo ""

# Convert flow list to array
IFS=',' read -ra FLOW_ARRAY <<< "$FLOWS"

# Run tests for each flow count
total_tests=${#FLOW_ARRAY[@]}
current_test=0

for num_flows in "${FLOW_ARRAY[@]}"; do
    num_flows=$(echo $num_flows | xargs)  # Trim whitespace
    current_test=$((current_test + 1))

    # Determine bottleneck bandwidth
    if [ -n "$CUSTOM_BOTTLE_BW" ]; then
        bottle_bw=$CUSTOM_BOTTLE_BW
    else
        bottle_bw=${DEFAULT_BOTTLE_BW[$num_flows]:-$(($num_flows * 2))}
    fi

    echo -e "${YELLOW}[${current_test}/${total_tests}]${NC} Testing with $num_flows flows..."

    if run_test "$num_flows" "$SENDER_BW" "$bottle_bw" "$BOTTLE_DELAY" "$SIM_TIME"; then
        echo -e "${GREEN}✓ Test completed successfully${NC}"
    else
        echo -e "${RED}✗ Test failed${NC}"
    fi

    echo ""
done

print_header "All Tests Completed"
echo -e "${GREEN}Trace files saved to: traces/${NC}"
echo ""
ls -lh traces/ | tail -n +2 | head -20
echo ""

echo -e "${BLUE}Summary:${NC}"
echo "  Total tests: $total_tests"
echo "  Results saved to: test_*.log"
echo ""
