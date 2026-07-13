#!/usr/bin/env python3
"""Analyze F-BBR event-triggered frequency-search traces."""

from __future__ import annotations

import argparse
import csv
import json
import math
import statistics
from collections import Counter
from pathlib import Path


def number(row: dict[str, str], key: str, default: float = math.nan) -> float:
    try:
        value = float(row.get(key, ""))
        return value if math.isfinite(value) else default
    except (TypeError, ValueError):
        return default


def boolean(row: dict[str, str], key: str) -> bool:
    return row.get(key, "").strip().lower() == "true"


def median(values: list[float]) -> float:
    finite = [value for value in values if math.isfinite(value)]
    return statistics.median(finite) if finite else math.nan


def ratio(numerator: int, denominator: int) -> float:
    return numerator / denominator if denominator else math.nan


def read_csvs(run_dir: Path, suffix: str) -> list[dict[str, str]]:
    rows: list[dict[str, str]] = []
    for path in sorted(run_dir.glob(f"flow*_{suffix}.csv")):
        with path.open(newline="") as handle:
            rows.extend(csv.DictReader(handle))
    return rows


def read_meta(run_dir: Path) -> dict[str, object]:
    path = run_dir / "run_meta.json"
    if not path.exists():
        return {}
    with path.open() as handle:
        return json.load(handle)


def read_goodput(run_dir: Path) -> dict[str, list[tuple[float, float]]]:
    samples: dict[str, list[tuple[float, float]]] = {}
    for path in sorted(run_dir.glob("*_flow*_F-BBR_good.txt")):
        flow = path.name.split("_flow", 1)[1].split("_", 1)[0]
        values: list[tuple[float, float]] = []
        with path.open() as handle:
            for line in handle:
                if not line or line.startswith("#"):
                    continue
                fields = line.split()
                if len(fields) < 2:
                    continue
                try:
                    values.append((float(fields[0]), float(fields[1]) * 1000.0))
                except ValueError:
                    pass
        samples[flow] = values
    return samples


def goodput_metrics(run_dir: Path, capacity_bps: float) -> tuple[float, float]:
    samples = read_goodput(run_dir)
    means: list[float] = []
    for values in samples.values():
        if not values:
            continue
        start = values[0][0] + 0.5 * (values[-1][0] - values[0][0])
        tail = [rate for time, rate in values if time >= start]
        means.append(statistics.mean(tail or [rate for _, rate in values]))
    total = sum(means)
    utilization = total / capacity_bps if capacity_bps > 0.0 else math.nan
    fairness = ((total * total) / (len(means) * sum(rate * rate for rate in means))
                if means and sum(rate * rate for rate in means) > 0.0 else math.nan)
    return utilization, fairness


def analyze(run_dir: Path) -> tuple[dict[str, object], list[dict[str, str]]]:
    meta = read_meta(run_dir)
    triggers = read_csvs(run_dir, "fbbr_trigger_cycles")
    windows = read_csvs(run_dir, "fbbr_event_windows")
    diagnostics = read_csvs(run_dir, "fbbr_diagnostic_windows")
    cruises = read_csvs(run_dir, "fbbr_cruises")

    capacity = float(meta.get("capacity_bps", 0.0) or 0.0)
    n_flows = int(meta.get("n_flows", 0) or 0)
    fair_share = capacity / n_flows if capacity > 0.0 and n_flows else math.nan
    rtprop_us = float(meta.get("measured_rtprop_s", meta.get("rtprop_s", 0.0)) or 0.0) * 1e6

    safety = [row for row in windows
              if number(row, "window_end_s", 0.0) <= number(row, "window_start_s", 0.0)]
    production = [row for row in windows
                  if boolean(row, "is_pulser") and
                  number(row, "window_end_s", 0.0) > number(row, "window_start_s", 0.0)]
    event_windows = [row for row in production if number(row, "window_length_cycles", 0.0) > 0.0]
    fixed_windows = [row for row in production if number(row, "window_length_cycles", 0.0) <= 0.0]
    mode = "event" if triggers or event_windows else "fixed"
    scored = event_windows if mode == "event" else fixed_windows
    independent = [row for row in scored if boolean(row, "independent_for_control")]
    overlapping = [row for row in scored if not boolean(row, "independent_for_control")]
    measurable = [row for row in independent
                  if row.get("invalid_reason") == "none" and
                  row.get("classification") not in ("", "INVALID", "DYNAMIC")]
    trusted = [row for row in independent if boolean(row, "trusted_bw_published")]
    updates = [row for row in independent
               if abs(number(row, "applied_next_baseline_bps", 0.0) -
                      number(row, "baseline_before_bps", 0.0)) >
               max(1.0, 1e-6 * number(row, "baseline_before_bps", 0.0))]
    regular_drains = [row for row in updates
                      if not boolean(row, "hard_loss_abort") and
                      number(row, "applied_next_baseline_bps", 0.0) <
                      number(row, "baseline_before_bps", 0.0)]

    trigger_rows = [row for row in triggers if boolean(row, "trigger_pass")]
    pulser_cycles = [row for row in triggers if row.get("pulser_role") == "PULSER"]
    continue_rows = [row for row in triggers if boolean(row, "continue_pass")]
    decision_latencies = []
    for row in event_windows:
        capture_start = number(row, "capture_start_s")
        candidates = [number(trigger, "cycle_end_s") for trigger in trigger_rows
                      if trigger.get("flow_id") == row.get("flow_id") and
                      trigger.get("cruise_id") == row.get("cruise_id") and
                      trigger.get("cycle_id") == row.get("trigger_cycle_id") and
                      number(trigger, "cycle_end_s") <= capture_start + 1e-9]
        start = max(candidates, default=capture_start)
        decision_latencies.append(number(row, "window_end_s") - start)

    direction_correct = 0
    for row in measurable:
        baseline = number(row, "baseline_before_bps")
        classification = row.get("classification")
        queue_overload = (number(row, "q_min_us") > number(row, "q_H_us") or
                          number(row, "q95_event_us") > number(row, "q_peak_cap_us"))
        if not math.isfinite(fair_share) or not math.isfinite(baseline):
            continue
        if classification == "QUEUED_OVERLOAD" and queue_overload:
            direction_correct += 1
        elif classification in ("UNDERLOAD", "UNDERLOADED") and baseline < 0.95 * fair_share and not queue_overload:
            direction_correct += 1
        elif classification == "NEAR_OPTIMAL" and abs(baseline - fair_share) <= 0.10 * fair_share:
            direction_correct += 1

    drain_steps = [1.0 - number(row, "applied_next_baseline_bps") /
                   number(row, "baseline_before_bps")
                   for row in regular_drains
                   if number(row, "baseline_before_bps", 0.0) > 0.0]
    utilization, fairness = goodput_metrics(run_dir, capacity)
    failures: list[str] = []
    if mode == "event":
        if not trigger_rows:
            failures.append("FAIL_TRIGGER_LOW_RECALL")
        if trigger_rows and ratio(len(event_windows), len(trigger_rows)) < 0.70:
            failures.append("FAIL_EVENT_WINDOW_NOT_MEASURABLE")
        if any(not boolean(row, "trigger_cycle_excluded_from_score") for row in event_windows):
            failures.append("FAIL_EVENT_WINDOW_NOT_MEASURABLE")
        if pulser_cycles and not event_windows:
            failures.append("FAIL_EVENT_WINDOW_NOT_MEASURABLE")
    if scored and ratio(len(measurable), len(independent)) < 0.60:
        failures.append("FAIL_EVENT_WINDOW_NOT_MEASURABLE")
    reached_track = any(row.get("search_state_after") in
                        ("TRACK", "LOCK_CANDIDATE", "LOCKED")
                        for row in independent)
    if scored and not reached_track:
        failures.append("FAIL_SEARCH_NOT_CONVERGED")
    if mode == "event" and pulser_cycles and not reached_track:
        failures.append("FAIL_SEARCH_NOT_CONVERGED")
    if mode == "event" and pulser_cycles and not trusted:
        failures.append("FAIL_TRUSTED_BW_INACCURATE")
    if drain_steps and max(drain_steps) > 0.050001:
        failures.append("FAIL_BASELINE_OVERSHOOT")
    q95_bdp = [number(row, "q95_event_us") / rtprop_us for row in event_windows
               if rtprop_us > 0.0 and math.isfinite(number(row, "q95_event_us"))]
    if q95_bdp and median(q95_bdp) > 0.10:
        failures.append("FAIL_QUEUE_RESERVE_TARGET")
    if cruises and any(not boolean(row, "search_active") for row in cruises):
        failures.append("FAIL_PERSISTENT_SEARCH_DISABLED")
    failures = list(dict.fromkeys(failures))

    summary: dict[str, object] = {
        "run": run_dir.name,
        "mode": mode,
        "flows": n_flows,
        "capacity_mbps": capacity / 1e6,
        "buffer_bdp": meta.get("buffer_bdp", math.nan),
        "seed": meta.get("seed", math.nan),
        "trigger_cycles": len(triggers),
        "pulser_trigger_cycles": len(pulser_cycles),
        "trigger_passes": len(trigger_rows),
        "continue_passes": len(continue_rows),
        "completed_event_windows": len(event_windows),
        "capture_completion_ratio": ratio(len(event_windows), len(trigger_rows)),
        "independent_scored_windows": len(independent),
        "overlap_diagnostic_windows": len(overlapping) + len(diagnostics),
        "measurable_windows": len(measurable),
        "measurable_ratio": ratio(len(measurable), len(independent)),
        "median_window_cycles": median([number(row, "window_length_cycles") for row in event_windows]),
        "median_decision_latency_s": median(decision_latencies),
        "baseline_updates": len(updates),
        "regular_drain_updates": len(regular_drains),
        "max_regular_drain_step": max(drain_steps, default=math.nan),
        "trusted_publications": len(trusted),
        "direction_accuracy": ratio(direction_correct, len(measurable)),
        "median_target_score": median([number(row, "S_target") for row in independent]),
        "median_q_min_bdp": median([number(row, "q_min_us") / rtprop_us for row in event_windows]) if rtprop_us > 0 else math.nan,
        "median_q95_bdp": median(q95_bdp),
        "tail_utilization": utilization,
        "tail_jain_fairness": fairness,
        "hard_congestion_safety_aborts": len(safety),
        "trigger_reasons": ";".join(f"{key}:{value}" for key, value in sorted(Counter(
            row.get("trigger_reason", "") for row in pulser_cycles).items())),
        "failure_labels": ";".join(failures) if failures else "PASS",
    }
    return summary, scored


def format_value(value: object) -> str:
    if isinstance(value, float):
        return "nan" if not math.isfinite(value) else f"{value:.6g}"
    return str(value)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("run_dirs", nargs="+", type=Path)
    parser.add_argument("--output-dir", required=True, type=Path)
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)

    summaries: list[dict[str, object]] = []
    windows: list[dict[str, str]] = []
    for run_dir in args.run_dirs:
        summary, run_windows = analyze(run_dir.resolve())
        summaries.append(summary)
        for row in run_windows:
            windows.append({"run": run_dir.name, **row})

    with (args.output_dir / "event_validation_summary.csv").open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(summaries[0]))
        writer.writeheader()
        writer.writerows(summaries)
    if windows:
        with (args.output_dir / "event_validation_windows.csv").open("w", newline="") as handle:
            writer = csv.DictWriter(handle, fieldnames=list(windows[0]))
            writer.writeheader()
            writer.writerows(windows)
    with (args.output_dir / "event_validation_summary.md").open("w") as handle:
        handle.write("# F-BBR Event-Triggered Validation Summary\n\n")
        handle.write("| Run | Mode | Triggers | Completed | Independent | Measurable | Updates | Trusted | Utilization | Jain | Result |\n")
        handle.write("|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---|\n")
        for row in summaries:
            handle.write(
                f"| {row['run']} | {row['mode']} | {row['trigger_passes']} | "
                f"{row['completed_event_windows']} | {row['independent_scored_windows']} | "
                f"{row['measurable_windows']} | {row['baseline_updates']} | "
                f"{row['trusted_publications']} | {format_value(row['tail_utilization'])} | "
                f"{format_value(row['tail_jain_fairness'])} | {row['failure_labels']} |\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
