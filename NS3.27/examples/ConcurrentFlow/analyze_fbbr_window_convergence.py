#!/usr/bin/env python3
"""Measure F-BBR window-level convergence toward the per-flow fair share."""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path


def read_rows(pattern: str) -> list[dict[str, str]]:
    rows: list[dict[str, str]] = []
    for path in sorted(Path().glob(pattern)):
        with path.open(newline="") as handle:
            rows.extend(csv.DictReader(handle))
    return rows


def as_float(row: dict[str, str], key: str) -> float:
    try:
        return float(row.get(key, "nan"))
    except ValueError:
        return math.nan


def is_true(row: dict[str, str], key: str) -> bool:
    return row.get(key, "").lower() == "true"


def analyze_run(run_dir: Path) -> tuple[list[dict[str, object]], dict[str, object]]:
    cruises: list[dict[str, str]] = []
    blocks: list[dict[str, str]] = []
    for path in sorted(run_dir.glob("flow*_fbbr_cruises.csv")):
        with path.open(newline="") as handle:
            cruises.extend(csv.DictReader(handle))
    for path in sorted(run_dir.glob("flow*_fbbr_event_windows.csv")):
        with path.open(newline="") as handle:
            blocks.extend(csv.DictReader(handle))

    cruise_by_key = {
        (row["flow_id"], row["cruise_id"]): row for row in cruises
    }
    output: list[dict[str, object]] = []
    publication_times: list[float] = []
    publication_latencies: list[float] = []
    first_within_5_by_flow: dict[str, float] = {}
    first_window_by_flow: dict[str, float] = {}

    for row in blocks:
        if not is_true(row, "is_pulser"):
            continue
        cruise = cruise_by_key.get((row["flow_id"], row["cruise_id"]))
        if cruise is None:
            continue
        fair = as_float(cruise, "fair_share_bps")
        before = as_float(row, "baseline_before_bps")
        after = as_float(row, "applied_next_baseline_bps")
        window_end = as_float(row, "window_end_s")
        cruise_start = as_float(cruise, "start_time_s")
        if not all(math.isfinite(x) and x > 0.0 for x in (fair, before, after)):
            continue
        error_before = abs(before - fair) / fair
        error_after = abs(after - fair) / fair
        delta = after - before
        gap = fair - before
        updated = abs(delta) > max(1.0, 1e-6 * before)
        toward = updated and delta * gap > 0.0
        error_reduced = error_after + 1e-12 < error_before
        within_5_after = error_after <= 0.05
        flow_id = row["flow_id"]
        first_window_by_flow[flow_id] = min(
            first_window_by_flow.get(flow_id, window_end), window_end
        )
        if within_5_after:
            first_within_5_by_flow[flow_id] = min(
                first_within_5_by_flow.get(flow_id, window_end), window_end
            )
        if is_true(row, "trusted_bw_published"):
            publication_times.append(window_end)
            publication_latencies.append(window_end - cruise_start)
        output.append({
            "run": run_dir.name,
            "flow_id": flow_id,
            "cruise_id": row["cruise_id"],
            "block_id": row["block_id"],
            "window_end_s": window_end,
            "seconds_from_cruise_start": window_end - cruise_start,
            "fair_share_bps": fair,
            "baseline_before_bps": before,
            "baseline_after_bps": after,
            "relative_error_before": error_before,
            "relative_error_after": error_after,
            "baseline_updated": updated,
            "action_toward_fair_share": toward,
            "fair_share_error_reduced": error_reduced,
            "within_5pct_after": within_5_after,
            "classification": row.get("classification", ""),
            "search_state_after": row.get("search_state_after", ""),
            "measurable": row.get("invalid_reason") == "none",
            "trusted_bw_published": is_true(row, "trusted_bw_published"),
            "trusted_bw_bps": as_float(row, "trusted_bw_control_bps"),
        })

    updated_rows = [row for row in output if row["baseline_updated"]]
    reduced_rows = [row for row in updated_rows if row["fair_share_error_reduced"]]
    toward_rows = [row for row in updated_rows if row["action_toward_fair_share"]]
    reductions = [
        float(row["relative_error_before"]) - float(row["relative_error_after"])
        for row in updated_rows
    ]
    publication_times.sort()
    publication_intervals = [
        right - left for left, right in zip(publication_times, publication_times[1:])
    ]
    convergence_latencies = [
        first_within_5_by_flow[flow] - first_window_by_flow[flow]
        for flow in first_within_5_by_flow
    ]

    def median(values: list[float]) -> float:
        values = sorted(values)
        if not values:
            return math.nan
        middle = len(values) // 2
        if len(values) % 2:
            return values[middle]
        return 0.5 * (values[middle - 1] + values[middle])

    summary: dict[str, object] = {
        "run": run_dir.name,
        "pulser_windows": len(output),
        "measurable_pulser_windows": sum(bool(row["measurable"]) for row in output),
        "baseline_update_windows": len(updated_rows),
        "action_toward_fair_share_ratio": (
            len(toward_rows) / len(updated_rows) if updated_rows else math.nan
        ),
        "fair_share_error_reduced_ratio": (
            len(reduced_rows) / len(updated_rows) if updated_rows else math.nan
        ),
        "median_relative_error_reduction_per_update": median(reductions),
        "median_window_decision_latency_s": median([
            float(row["seconds_from_cruise_start"]) for row in output
        ]),
        "flows_within_5pct": len(first_within_5_by_flow),
        "flows_with_pulser_window": len(first_window_by_flow),
        "median_time_to_within_5pct_s": median(convergence_latencies),
        "median_within_5pct_window_latency_s": median([
            float(row["seconds_from_cruise_start"])
            for row in output if row["within_5pct_after"]
        ]),
        "trusted_bw_publications": len(publication_times),
        "trusted_publication_per_pulser_window_ratio": (
            len(publication_times) / len(output) if output else math.nan
        ),
        "median_trusted_publication_interval_s": median(publication_intervals),
        "median_trusted_publication_latency_s": median(publication_latencies),
    }
    return output, summary


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("run_dirs", nargs="+", type=Path)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)

    windows: list[dict[str, object]] = []
    summaries: list[dict[str, object]] = []
    for run_dir in args.run_dirs:
        run_windows, run_summary = analyze_run(run_dir.resolve())
        windows.extend(run_windows)
        summaries.append(run_summary)

    if windows:
        with (args.output_dir / "window_convergence.csv").open("w", newline="") as handle:
            writer = csv.DictWriter(handle, fieldnames=list(windows[0]))
            writer.writeheader()
            writer.writerows(windows)
    if summaries:
        with (args.output_dir / "window_convergence_summary.csv").open(
            "w", newline=""
        ) as handle:
            writer = csv.DictWriter(handle, fieldnames=list(summaries[0]))
            writer.writeheader()
            writer.writerows(summaries)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
