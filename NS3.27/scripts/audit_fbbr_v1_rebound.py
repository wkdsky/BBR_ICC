#!/usr/bin/env python3
"""Audit V1 baseline rebound and pacing actuation from existing traces."""

from __future__ import annotations

import argparse
import bisect
import csv
import json
import math
import re
from collections import Counter
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence, Tuple


DECREASE_ACTION = "HYBRID_REGIME_III_GRADIENT_MATCHED_DECREASE"
SCENARIOS = {
    "fixed_4": "fixed_4flows_100M_5BDP_240s",
    "fixed_32": "fixed_32flows_100M_5BDP_240s",
    "cell_4": (
        "cellular_taxi_128M_4flow_8flow_5BDP_180s/"
        "cellular_taxi_128M_4flows_5BDP_180s"
    ),
    "cell_8": (
        "cellular_taxi_128M_4flow_8flow_5BDP_180s/"
        "cellular_taxi_128M_8flows_5BDP_180s"
    ),
}
SOURCES = ("UNDERLOAD", "FULL_LOAD", "cruise/native reset", "other")


def percentile(values: Iterable[float], quantile: float) -> float:
    ordered = sorted(value for value in values if math.isfinite(value))
    if not ordered:
        return math.nan
    position = quantile * (len(ordered) - 1)
    lower = int(math.floor(position))
    upper = int(math.ceil(position))
    if lower == upper:
        return ordered[lower]
    fraction = position - lower
    return ordered[lower] + fraction * (ordered[upper] - ordered[lower])


def flow_id(path: Path) -> int:
    match = re.search(r"(?:^|_)flow(\d+)(?:_|$)", path.name)
    if not match:
        raise ValueError(f"cannot extract flow id from {path}")
    return int(match.group(1))


def finite_float(row: Dict[str, str], key: str) -> float:
    try:
        value = float(row[key])
    except (KeyError, TypeError, ValueError):
        return math.nan
    return value if math.isfinite(value) else math.nan


def read_waveform(path: Path) -> List[Dict[str, str]]:
    with path.open("r", encoding="utf-8", errors="replace") as handle:
        rows = list(csv.DictReader(handle))
    return sorted(rows, key=lambda row: finite_float(row, "time_s"))


def valid_classification_row(row: Dict[str, str]) -> bool:
    return (
        row.get("classification") in {"UNDERLOAD", "FULL_LOAD", "OVERLOAD"}
        and row.get("delivery_rate_stats_valid") == "true"
        and math.isfinite(finite_float(row, "baseline_before_bps"))
        and math.isfinite(finite_float(row, "baseline_after_bps"))
    )


def read_numeric_trace(path: Path) -> Tuple[List[float], List[float]]:
    samples: List[Tuple[float, float]] = []
    with path.open("r", encoding="utf-8", errors="replace") as handle:
        for line in handle:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            fields = re.split(r"\s+", line)
            if len(fields) < 2:
                continue
            try:
                time_s = float(fields[0])
                value = float(fields[1])
            except ValueError:
                continue
            if math.isfinite(time_s) and math.isfinite(value):
                samples.append((time_s, value))
    samples.sort()
    # PacingRate can be queried many times at one timestamp. Keep the last
    # value so a timestamp cannot receive packet-count weighting.
    unique: List[Tuple[float, float]] = []
    for sample in samples:
        if unique and sample[0] == unique[-1][0]:
            unique[-1] = sample
        else:
            unique.append(sample)
    return [sample[0] for sample in unique], [sample[1] for sample in unique]


def previous_sample_mean(
    times: Sequence[float],
    values: Sequence[float],
    start_s: float,
    end_s: float,
) -> float:
    """Time-weighted previous-sample mean on [start_s, end_s)."""
    if not times or end_s <= start_s:
        return math.nan
    index = max(0, bisect.bisect_right(times, start_s) - 1)
    current_time = start_s
    current_value = values[index]
    total = 0.0
    while index + 1 < len(times) and times[index + 1] < end_s:
        next_time = max(current_time, times[index + 1])
        total += (next_time - current_time) * current_value
        current_time = next_time
        index += 1
        current_value = values[index]
    total += (end_s - current_time) * current_value
    return total / (end_s - start_s)


def pacing_mean_bps(
    row: Dict[str, str], times: Sequence[float], values_kbps: Sequence[float]
) -> float:
    start_s = finite_float(row, "response_window_start_s")
    end_s = finite_float(row, "response_window_end_s")
    feedback_s = finite_float(row, "probe_epoch_rtt_s")
    if not all(math.isfinite(value) for value in (start_s, end_s, feedback_s)):
        return math.nan
    return 1000.0 * previous_sample_mean(
        times, values_kbps, start_s - feedback_s, end_s - feedback_s
    )


def transition_source(previous: Dict[str, str], current: Dict[str, str]) -> str:
    previous_after = finite_float(previous, "baseline_after_bps")
    current_before = finite_float(current, "baseline_before_bps")
    current_after = finite_float(current, "baseline_after_bps")
    tolerance = max(1.0, 1e-9 * max(previous_after, current_before, current_after))
    reset_growth = max(0.0, current_before - previous_after)
    action_growth = max(0.0, current_after - current_before)
    if reset_growth > tolerance and reset_growth >= action_growth:
        return "cruise/native reset"
    if action_growth > tolerance:
        classification = current.get("classification", "")
        if classification == "UNDERLOAD":
            return "UNDERLOAD"
        if classification == "FULL_LOAD":
            return "FULL_LOAD"
        return "other"
    if current_after > previous_after + tolerance:
        return "other"
    return "other"


def audit_flow(
    rows: Sequence[Dict[str, str]],
    pacing_times: Sequence[float],
    pacing_values_kbps: Sequence[float],
) -> Tuple[List[dict], Counter]:
    windows = [row for row in rows if valid_classification_row(row)]
    events: List[dict] = []
    sources: Counter = Counter()
    for index, row in enumerate(windows):
        if row.get("action") != DECREASE_ACTION:
            continue
        before = finite_float(row, "baseline_before_bps")
        after = finite_float(row, "baseline_after_bps")
        decrease = before - after
        if not math.isfinite(decrease) or decrease <= 0.0:
            continue
        next_one = windows[index + 1] if index + 1 < len(windows) else None
        next_two = windows[index + 2] if index + 2 < len(windows) else None
        rebound_one = (
            next_one is not None
            and finite_float(next_one, "baseline_after_bps") > after + 1.0
        )
        rebound_two = rebound_one or (
            next_two is not None
            and finite_float(next_two, "baseline_after_bps") > after + 1.0
        )
        source = ""
        previous = row
        for candidate in (next_one, next_two):
            if candidate is None:
                continue
            if finite_float(candidate, "baseline_after_bps") > after + 1.0:
                source = transition_source(previous, candidate)
                sources[source] += 1
                break
            previous = candidate
        u_value = (
            max(0.0, finite_float(next_one, "baseline_after_bps") - after)
            / (decrease + 1e-12)
            if next_one is not None
            else math.nan
        )
        pre_pacing = pacing_mean_bps(row, pacing_times, pacing_values_kbps)
        post_pacing = (
            pacing_mean_bps(next_one, pacing_times, pacing_values_kbps)
            if next_one is not None
            else math.nan
        )
        actuation = (
            (pre_pacing - post_pacing) / (decrease + 1e-12)
            if math.isfinite(pre_pacing) and math.isfinite(post_pacing)
            else math.nan
        )
        events.append(
            {
                "time_s": finite_float(row, "time_s"),
                "baseline_before_bps": before,
                "baseline_after_bps": after,
                "decrease_bps": decrease,
                "increase_within_1_window": rebound_one,
                "increase_within_2_windows": rebound_two,
                "U": u_value,
                "rebound_source": source,
                "pre_actual_pacing_mean_bps": pre_pacing,
                "post_actual_pacing_mean_bps": post_pacing,
                "actuation_ratio_A": actuation,
            }
        )
    return events, sources


def write_csv(path: Path, rows: Sequence[dict]) -> None:
    if not rows:
        return
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--result-root", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path)
    args = parser.parse_args()
    root = args.result_root.resolve()
    output = (args.output_dir or root / "v1_closed_loop_audit").resolve()
    output.mkdir(parents=True, exist_ok=True)

    summaries: List[dict] = []
    detailed: List[dict] = []
    for scenario, relative in SCENARIOS.items():
        run_dir = root / relative / "FBBR-hybrid"
        scenario_events: List[dict] = []
        source_counts: Counter = Counter()
        for waveform_path in sorted(
            run_dir.glob("flow*_cruise_waveform_search.csv"), key=flow_id
        ):
            flow = flow_id(waveform_path)
            sendrate_path = next(
                iter(
                    run_dir.glob(
                        f"FBBR-hybrid_flow{flow}_FBBR-hybrid_sendrate.txt"
                    )
                ),
                None,
            )
            if sendrate_path is None:
                pacing_times, pacing_values = [], []
            else:
                pacing_times, pacing_values = read_numeric_trace(sendrate_path)
            events, flow_sources = audit_flow(
                read_waveform(waveform_path), pacing_times, pacing_values
            )
            source_counts.update(flow_sources)
            for event in events:
                event = {"scenario": scenario, "flow": flow, **event}
                scenario_events.append(event)
                detailed.append(event)
        u_values = [event["U"] for event in scenario_events]
        a_values = [
            event["actuation_ratio_A"]
            for event in scenario_events
            if math.isfinite(event["actuation_ratio_A"])
        ]
        count = len(scenario_events)
        summaries.append(
            {
                "scenario": scenario,
                "decrease_count": count,
                "increase_within_1_window_ratio": (
                    sum(event["increase_within_1_window"] for event in scenario_events)
                    / count
                    if count
                    else math.nan
                ),
                "increase_within_2_windows_ratio": (
                    sum(event["increase_within_2_windows"] for event in scenario_events)
                    / count
                    if count
                    else math.nan
                ),
                "median_U": percentile(u_values, 0.50),
                "p95_U": percentile(u_values, 0.95),
                "UNDERLOAD": source_counts["UNDERLOAD"],
                "FULL_LOAD": source_counts["FULL_LOAD"],
                "cruise/native reset": source_counts["cruise/native reset"],
                "other": source_counts["other"],
                "actuation_sample_count": len(a_values),
                "actuation_ratio_p50": percentile(a_values, 0.50),
                "actuation_ratio_p10": percentile(a_values, 0.10),
                "actuation_ratio_lt_0p5": (
                    sum(value < 0.5 for value in a_values) / len(a_values)
                    if a_values
                    else math.nan
                ),
            }
        )

    write_csv(output / "v1_rebound_actuation_events.csv", detailed)
    write_csv(output / "v1_rebound_actuation_summary.csv", summaries)
    with (output / "v1_rebound_actuation_summary.json").open(
        "w", encoding="utf-8"
    ) as handle:
        json.dump(summaries, handle, indent=2, allow_nan=True)
        handle.write("\n")
    for summary in summaries:
        print(json.dumps(summary, ensure_ascii=False, allow_nan=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
