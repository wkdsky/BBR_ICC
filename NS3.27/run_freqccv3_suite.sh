#!/bin/bash

################################################################################
# FreqCCv3 Suite Runner (plan file driven)
#
# Usage:
#   ./run_freqccv3_suite.sh --plan-file <plan.txt> [OPTIONS]
#
# Plan file format:
#   - empty lines / lines starting with # are ignored
#   - default: each line is appended to ./run_freqccv3_batch.sh
#       e.g. --flows 4,8 --sender-bw 10 --bottle-bw 16 --queue-bdp 2
#   - raw command mode: prefix line with CMD:
#       e.g. CMD: ./run_freqccv3_batch.sh --flows 8 --bottle-delay 40 --sim-time 60
#
# Options:
#   --plan-file <path>        Plan file path (required)
#   --suite-name <name>       Suite folder prefix (default: freqccv3_suite)
#   --instance-base <id>      Auto instance starting value (default: 1)
#   --stop-on-fail            Stop immediately if one case fails
#   --keep-batch-root         Keep per-run traces/freqccv3_batch_ins* roots
#   --dry-run                 Print commands only, do not execute
#   --help                    Show help
################################################################################

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
NS3_DIR="$SCRIPT_DIR"
BATCH_SCRIPT="./run_freqccv3_batch.sh"

PLAN_FILE=""
SUITE_NAME="freqccv3_suite"
INSTANCE_BASE=1
STOP_ON_FAIL=false
KEEP_BATCH_ROOT=false
DRY_RUN=false

usage() {
  sed -n '3,23p' "$0" | sed 's/^# \{0,1\}//'
}

trim() {
  local value="$1"
  value="${value#"${value%%[![:space:]]*}"}"
  value="${value%"${value##*[![:space:]]}"}"
  printf '%s' "$value"
}

strip_ansi() {
  sed -r 's/\x1B\[[0-9;]*[A-Za-z]//g'
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --plan-file)
      PLAN_FILE="$2"
      shift 2
      ;;
    --suite-name)
      SUITE_NAME="$2"
      shift 2
      ;;
    --instance-base)
      INSTANCE_BASE="$2"
      shift 2
      ;;
    --stop-on-fail)
      STOP_ON_FAIL=true
      shift
      ;;
    --keep-batch-root)
      KEEP_BATCH_ROOT=true
      shift
      ;;
    --dry-run)
      DRY_RUN=true
      shift
      ;;
    --help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown option: $1"
      usage
      exit 1
      ;;
  esac
done

if [[ -z "$PLAN_FILE" ]]; then
  echo "Error: --plan-file is required"
  usage
  exit 1
fi

if [[ ! -f "$PLAN_FILE" ]]; then
  echo "Error: plan file not found: $PLAN_FILE"
  exit 1
fi

if [[ ! -f "$NS3_DIR/run_freqccv3_batch.sh" ]]; then
  echo "Error: $NS3_DIR/run_freqccv3_batch.sh not found"
  exit 1
fi

RUN_TAG="$(date +%Y%m%d_%H%M%S)"
SUITE_ROOT="$NS3_DIR/traces/${SUITE_NAME}_${RUN_TAG}"
META_DIR="$SUITE_ROOT/_meta"
SUMMARY_TSV="$META_DIR/summary.tsv"

mkdir -p "$META_DIR"

echo -e "case_id\tline_no\tstatus\ttrace_root\tcommand" > "$SUMMARY_TSV"

echo "Suite root: $SUITE_ROOT"
echo "Plan file : $PLAN_FILE"
echo "Dry run   : $DRY_RUN"
echo ""

case_no=0
line_no=0
fail_count=0

while IFS= read -r raw_line || [[ -n "$raw_line" ]]; do
  line_no=$((line_no + 1))
  line="$(trim "$raw_line")"

  if [[ -z "$line" || "${line:0:1}" == "#" ]]; then
    continue
  fi

  case_no=$((case_no + 1))
  case_id="case$(printf '%03d' "$case_no")"
  case_log="$META_DIR/${case_id}.log"

  if [[ "$line" == CMD:* ]]; then
    cmd_line="$(trim "${line#CMD:}")"
  else
    cmd_line="$BATCH_SCRIPT $line"
  fi

  if [[ "$cmd_line" == *"run_freqccv3_batch.sh"* && "$cmd_line" != *"--instance"* ]]; then
    cmd_line="$cmd_line --instance $((INSTANCE_BASE + case_no - 1))"
  fi

  echo "[$case_id] line $line_no"
  echo "  $cmd_line"

  if $DRY_RUN; then
    echo -e "${case_id}\t${line_no}\tDRY_RUN\t-\t${cmd_line}" >> "$SUMMARY_TSV"
    echo ""
    continue
  fi

  set +e
  (
    cd "$NS3_DIR"
    bash -lc "$cmd_line"
  ) 2>&1 | tee "$case_log"
  cmd_status=${PIPESTATUS[0]}
  set -e

  trace_root_rel="$(strip_ansi < "$case_log" | awk -F'Trace Root: ' '/Trace Root:/ {print $2}' | tail -n 1 | sed 's/[[:space:]]*$//')"
  trace_root_abs=""

  if [[ -n "$trace_root_rel" ]]; then
    if [[ "$trace_root_rel" == /* ]]; then
      trace_root_abs="$trace_root_rel"
    else
      trace_root_abs="$NS3_DIR/$trace_root_rel"
    fi
  fi

  if [[ $cmd_status -eq 0 && -n "$trace_root_abs" && -d "$trace_root_abs" && "$KEEP_BATCH_ROOT" == false ]]; then
    shopt -s nullglob
    for scenario_dir in "$trace_root_abs"/*; do
      [[ -d "$scenario_dir" ]] || continue
      scenario_base="$(basename "$scenario_dir")"
      target_dir="$SUITE_ROOT/${case_id}__${scenario_base}"
      if [[ -e "$target_dir" ]]; then
        target_dir="${target_dir}__$(date +%H%M%S)"
      fi
      mv "$scenario_dir" "$target_dir"
    done
    shopt -u nullglob
    rmdir "$trace_root_abs" 2>/dev/null || true
  fi

  if [[ $cmd_status -eq 0 ]]; then
    status="OK"
  else
    status="FAIL"
    fail_count=$((fail_count + 1))
  fi

  echo -e "${case_id}\t${line_no}\t${status}\t${trace_root_rel:-N/A}\t${cmd_line}" >> "$SUMMARY_TSV"
  echo "  => status: $status"
  echo ""

  if [[ $cmd_status -ne 0 && "$STOP_ON_FAIL" == true ]]; then
    echo "Stop on fail enabled, aborting."
    break
  fi
done < "$PLAN_FILE"

echo "Done."
echo "- Suite root : $SUITE_ROOT"
echo "- Summary    : $SUMMARY_TSV"
echo "- Cases      : $case_no"
echo "- Failures   : $fail_count"

if [[ $case_no -eq 0 ]]; then
  echo "Warning: no runnable lines found in plan file."
fi
