#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
OUT="${1:-$ROOT/docs/fbbr_persistent_search/latest}"
CONFIG="$ROOT/examples/CCconfig/fbbr_default.conf"
export LD_LIBRARY_PATH="$ROOT/build${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"

mkdir -p "$OUT"
cd "$ROOT"
./waf build -j2
./build/scratch/fbbr_4flow --fbbrFrequencySearchSelfTest=true \
  > "$OUT/deterministic_selftest.log" 2>&1

run_case() {
  local flows="$1" seconds="$2" seed="$3" buffer_bdp="$4" directory="$5"
  local equivalence_audit="${6:-true}"
  mkdir -p "$directory"
  ./build/scratch/generic_p2p_switch_flows \
    --nFlows="$flows" --algos=F-BBR --simTime="$seconds" \
    --serviceRate=100Mbps --accessRate=1Gbps \
    --accessDelayMs=1 --serviceDelayMs=23 --switchBufferBdp="$buffer_bdp" \
    --flowStartTimes=0 --flowStopTimes="$seconds" --perFlowAppRateLimits=0 \
    --processIntervalUs=1000 --goodputIntervalMs=500 \
    --dataGeneratorBatch=20 --useEngineTimer=true \
    --enableTrace=true --enableHeavyTrace=false --enableQueueTrace=false \
    --enableConvergenceGateTrace=false \
    --enableEquivalenceAudit="$equivalence_audit" \
    --emitRunMeta=true --emitBottleneckQueueTrace=false \
    --fbbrConfig="$CONFIG" --tracePath="$directory/" \
    --traceName=fbbr_persistent --seed="$seed" --runId="$seed" \
    > "$directory/stdout.log" 2> "$directory/stderr.log"
}

run_case 1 12 1 2 "$OUT/phase_a_n1_bdp2"
run_case 4 20 2 4 "$OUT/phase_a_n4_bdp4"
run_case 4 20 3 8 "$OUT/phase_a_n4_bdp8"
run_case 4 20 4 16 "$OUT/phase_a_n4_bdp16"
run_case 4 45 5 8 "$OUT/phase_d_persistent_bdp8"
run_case 4 180 7 8 "$OUT/phase_d_long_bdp8" false

run_dynamic_case() {
  local directory="$OUT/phase_e_capacity_bdp8"
  mkdir -p "$directory"
  ./build/scratch/generic_p2p_switch_flows \
    --nFlows=4 --algos=F-BBR --simTime=50 \
    --serviceRate=100Mbps --accessRate=1Gbps \
    --accessDelayMs=1 --serviceDelayMs=23 --switchBufferBdp=8 \
    --flowStartTimes=0 --flowStopTimes=50 --perFlowAppRateLimits=0 \
    --capacitySchedule=0:100Mbps,15:70Mbps,30:100Mbps,40:70Mbps \
    --processIntervalUs=1000 --goodputIntervalMs=500 \
    --dataGeneratorBatch=20 --useEngineTimer=true \
    --enableTrace=true --enableHeavyTrace=false --enableQueueTrace=false \
    --enableConvergenceGateTrace=false --enableEquivalenceAudit=true \
    --emitRunMeta=true --emitBottleneckQueueTrace=false \
    --fbbrConfig="$CONFIG" --tracePath="$directory/" \
    --traceName=fbbr_dynamic --seed=6 --runId=6 \
    > "$directory/stdout.log" 2> "$directory/stderr.log"
}

run_dynamic_case

python3 examples/ConcurrentFlow/analyze_fbbr_pacer_audit.py \
  "$OUT/phase_a_n1_bdp2" "$OUT/phase_a_n4_bdp4" \
  "$OUT/phase_a_n4_bdp8" "$OUT/phase_a_n4_bdp16" \
  --output-dir "$OUT/actuator_summary"

python3 examples/ConcurrentFlow/analyze_fbbr_window_convergence.py \
  "$OUT/phase_a_n4_bdp4" "$OUT/phase_a_n4_bdp8" \
  "$OUT/phase_a_n4_bdp16" "$OUT/phase_d_persistent_bdp8" \
  "$OUT/phase_d_long_bdp8" "$OUT/phase_e_capacity_bdp8" \
  --output-dir "$OUT/window_convergence"

python3 - "$OUT" <<'PY'
import csv
import glob
import os
import sys

root = sys.argv[1]
rows = []
for path in glob.glob(os.path.join(root, "phase_d_persistent_bdp8",
                                   "flow*_fbbr_cruises.csv")):
    with open(path, newline="") as handle:
        rows.extend(csv.DictReader(handle))
eligible = len(rows)
active = sum(row.get("search_active") == "true" for row in rows)
enabled = sum(row.get("probe_enabled") == "true" for row in rows)
with open(os.path.join(root, "persistent_search_summary.csv"), "w", newline="") as handle:
    writer = csv.writer(handle)
    writer.writerow(("eligible_cruises", "search_active_cruises",
                     "probe_enabled_cruises", "search_active_ratio"))
    writer.writerow((eligible, active, enabled, active / eligible if eligible else 0.0))
if eligible == 0 or active != eligible:
    raise SystemExit("FAIL_PERSISTENT_SEARCH_DISABLED")
PY

python3 - "$OUT" <<'PY'
import csv
import glob
import os
import sys

root = sys.argv[1]
with open(os.path.join(root, "large_bdp_matrix_summary.csv"), "w", newline="") as output:
    writer = csv.writer(output)
    writer.writerow(("buffer_bdp", "cruises", "search_active_ratio",
                     "pulser_decisions", "measurable_pulser_ratio", "collisions"))
    for bdp in (4, 8, 16):
        cruises = []
        cruise_pattern = os.path.join(root, f"phase_a_n4_bdp{bdp}",
                                      "flow*_fbbr_cruises.csv")
        for path in glob.glob(cruise_pattern):
            with open(path, newline="") as handle:
                cruises.extend(csv.DictReader(handle))
        blocks = []
        block_pattern = os.path.join(root, f"phase_a_n4_bdp{bdp}",
                                     "flow*_fbbr_event_windows.csv")
        for path in glob.glob(block_pattern):
            with open(path, newline="") as handle:
                blocks.extend(csv.DictReader(handle))
        active = sum(row.get("search_active") == "true" for row in cruises)
        pulser = [row for row in blocks if row.get("is_pulser") == "true"]
        measurable = sum(row.get("invalid_reason") == "none" for row in pulser)
        collisions = sum(row.get("decision_collision_suspected") == "true"
                         for row in pulser)
        writer.writerow((bdp, len(cruises),
                         active / len(cruises) if cruises else 0.0,
                         len(pulser), measurable / len(pulser) if pulser else 0.0,
                         collisions))

dynamic_rows = []
for path in glob.glob(os.path.join(root, "phase_e_capacity_bdp8",
                                   "flow*_fbbr_cruises.csv")):
    with open(path, newline="") as handle:
        dynamic_rows.extend(csv.DictReader(handle))
dynamic_blocks = []
for path in glob.glob(os.path.join(root, "phase_e_capacity_bdp8",
                                   "flow*_fbbr_event_windows.csv")):
    with open(path, newline="") as handle:
        dynamic_blocks.extend(csv.DictReader(handle))
with open(os.path.join(root, "dynamic_search_summary.csv"), "w", newline="") as output:
    writer = csv.writer(output)
    writer.writerow(("cruises", "search_active_cruises", "search_active_ratio",
                     "dynamic_reacquire_decisions", "trusted_publications",
                     "collisions"))
    active = sum(row.get("search_active") == "true" for row in dynamic_rows)
    dynamic = sum(
        row.get("search_state_before") == "DYNAMIC_REACQUIRE" or
        row.get("search_state_after") == "DYNAMIC_REACQUIRE"
        for row in dynamic_blocks
    )
    publications = sum(row.get("publication_valid") == "true"
                       for row in dynamic_rows)
    collisions = sum(row.get("decision_collision_suspected") == "true"
                     for row in dynamic_blocks)
    writer.writerow((len(dynamic_rows), active,
                     active / len(dynamic_rows) if dynamic_rows else 0.0,
                     dynamic, publications, collisions))
if not dynamic_rows or active != len(dynamic_rows):
    raise SystemExit("FAIL_DYNAMIC_SEARCH_DISABLED")
PY

echo "F-BBR persistent-search validation artifacts: $OUT"
