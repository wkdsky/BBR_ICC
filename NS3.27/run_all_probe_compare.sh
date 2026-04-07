#!/bin/bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BATCH_SCRIPT="$SCRIPT_DIR/run_bbrv2_batch.sh"

FLOWS="4,8,16,32"
SENDER_BW=50
SIM_TIME=65
BOTTLE_BWS=(100)
BOTTLE_DELAYS=(48)
QUEUE_BDPS=(10)

if [[ ! -f "$BATCH_SCRIPT" ]]; then
    echo "Error: $BATCH_SCRIPT not found"
    exit 1
fi

run_group() {
    local probe_rtt=$1
    local label
    if [[ "$probe_rtt" == "off" ]]; then
        label="bbrv2_noprobe_rtt"
    else
        label="bbrv2"
    fi

    echo ""
    echo "============================================================"
    echo "Running group: $label (probe-rtt=$probe_rtt)"
    echo "============================================================"
    echo ""

    for bw in "${BOTTLE_BWS[@]}"; do
        for delay in "${BOTTLE_DELAYS[@]}"; do
            for q in "${QUEUE_BDPS[@]}"; do
                echo ">>> flows=$FLOWS sender=$SENDER_BW bottle_bw=$bw delay=$delay queue=$q sim_time=$SIM_TIME probe_rtt=$probe_rtt"
                bash "$BATCH_SCRIPT" \
                    --flows "$FLOWS" \
                    --sender-bw "$SENDER_BW" \
                    --bottle-bw "$bw" \
                    --bottle-delay "$delay" \
                    --queue-bdp "$q" \
                    --sim-time "$SIM_TIME" \
                    --probe-rtt "$probe_rtt"
            done
        done
    done
}

run_group off
run_group on

