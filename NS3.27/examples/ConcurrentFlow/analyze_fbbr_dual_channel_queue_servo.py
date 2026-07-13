#!/usr/bin/env python3
"""Analyze F-BBR dual-channel trigger and queue-servo validation runs."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import statistics
from collections import Counter
from pathlib import Path


ALLOWED_FAILURES = [
    "FAIL_FBBR_FREQCCV4_ISOLATION",
    "FAIL_DELIVERY_TRIGGER",
    "FAIL_QUEUE_TRIGGER",
    "FAIL_COMBINED_TRIGGER",
    "FAIL_QUEUE_SERVO",
    "FAIL_EVENT_WINDOW_NOT_MEASURABLE",
    "FAIL_SCORE_NOT_CORRELATED",
    "FAIL_SEARCH_NOT_CONVERGED",
    "FAIL_BASELINE_OVERSHOOT",
    "FAIL_QUEUE_RESERVE_TARGET",
    "FAIL_TRUSTED_BW_INACCURATE",
    "FAIL_DYNAMIC_TRACKING",
    "FAIL_PERSISTENT_SEARCH_DISABLED",
    "FAIL_EXPERIMENT_NOT_IDENTIFIABLE",
]


def number(row: dict[str, str], key: str, default: float = math.nan) -> float:
    try:
        return float(row.get(key, ""))
    except (TypeError, ValueError):
        return default


def boolean(row: dict[str, str], key: str) -> bool:
    return row.get(key, "").strip().lower() in {"1", "true", "yes"}


def median(values: list[float]) -> float:
    finite = [value for value in values if math.isfinite(value)]
    return statistics.median(finite) if finite else math.nan


def percentile(values: list[float], fraction: float) -> float:
    finite = sorted(value for value in values if math.isfinite(value))
    if not finite:
        return math.nan
    position = max(0.0, min(1.0, fraction)) * (len(finite) - 1)
    low = int(math.floor(position))
    high = int(math.ceil(position))
    if low == high:
        return finite[low]
    return finite[low] + (finite[high] - finite[low]) * (position - low)


def ratio(numerator: int | float, denominator: int | float) -> float:
    return numerator / denominator if denominator else math.nan


def read_rows(run_dir: Path, suffix: str) -> list[dict[str, str]]:
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
                fields = line.split()
                if len(fields) < 2 or line.startswith("#"):
                    continue
                try:
                    values.append((float(fields[0]), float(fields[1]) * 1000.0))
                except ValueError:
                    pass
        samples[flow] = values
    return samples


def goodput_metrics(run_dir: Path, capacity_bps: float) -> tuple[float, float]:
    means: list[float] = []
    for values in read_goodput(run_dir).values():
        if not values:
            continue
        tail_start = values[0][0] + 0.5 * (values[-1][0] - values[0][0])
        tail = [rate for time_s, rate in values if time_s >= tail_start]
        means.append(statistics.mean(tail or [rate for _, rate in values]))
    total = sum(means)
    utilization = total / capacity_bps if capacity_bps > 0.0 else math.nan
    square_sum = sum(value * value for value in means)
    fairness = total * total / (len(means) * square_sum) if square_sum else math.nan
    return utilization, fairness


def rank(values: list[float]) -> list[float]:
    order = sorted(range(len(values)), key=values.__getitem__)
    ranks = [0.0] * len(values)
    index = 0
    while index < len(order):
        end = index + 1
        while end < len(order) and values[order[end]] == values[order[index]]:
            end += 1
        average = 0.5 * (index + end - 1) + 1.0
        for position in range(index, end):
            ranks[order[position]] = average
        index = end
    return ranks


def spearman(xs: list[float], ys: list[float]) -> float:
    pairs = [(x, y) for x, y in zip(xs, ys)
             if math.isfinite(x) and math.isfinite(y)]
    if len(pairs) < 3:
        return math.nan
    rx = rank([pair[0] for pair in pairs])
    ry = rank([pair[1] for pair in pairs])
    mx = statistics.mean(rx)
    my = statistics.mean(ry)
    numerator = sum((x - mx) * (y - my) for x, y in zip(rx, ry))
    denominator = math.sqrt(sum((x - mx) ** 2 for x in rx) *
                            sum((y - my) ** 2 for y in ry))
    return numerator / denominator if denominator else math.nan


def trace_hash(run_dir: Path) -> str:
    digest = hashlib.sha256()
    paths = sorted(run_dir.glob("flow*_fbbr_trigger_cycles.csv"))
    paths += sorted(run_dir.glob("flow*_fbbr_queue_servo.csv"))
    paths += sorted(run_dir.glob("*_flow*_F-BBR_good.txt"))
    for path in paths:
        digest.update(path.name.encode())
        digest.update(path.read_bytes())
    return digest.hexdigest()


def analyze_run(run_dir: Path) -> tuple[dict[str, object], list[tuple[float, float]]]:
    meta = read_meta(run_dir)
    triggers = read_rows(run_dir, "fbbr_trigger_cycles")
    servo = read_rows(run_dir, "fbbr_queue_servo")
    windows = read_rows(run_dir, "fbbr_event_windows")
    cruises = read_rows(run_dir, "fbbr_cruises")
    capacity = float(meta.get("capacity_bps", 0.0) or 0.0)
    flows = int(meta.get("n_flows", 0) or 0)
    background_schedule = meta.get("background_schedule", [])
    background_bps = 0.0
    if isinstance(background_schedule, list) and background_schedule:
        background_bps = float(background_schedule[-1].get("rate_bps", 0.0))
    fair_share = ((capacity - background_bps) / flows
                  if flows and capacity > background_bps else math.nan)
    rtprop_s = float(meta.get("measured_rtprop_s", meta.get("rtprop_s", 0.0)) or 0.0)
    rtprop_us = rtprop_s * 1e6
    utilization, fairness = goodput_metrics(run_dir, capacity)

    pulser = [row for row in triggers if row.get("pulser_role") == "PULSER"]
    passed = [row for row in pulser if boolean(row, "trigger_pass")]
    sources = Counter(row.get("combined_trigger_source", "NONE") for row in pulser)
    production = [row for row in windows
                  if boolean(row, "is_pulser") and
                  number(row, "window_length_cycles", 0.0) > 0.0]
    independent = [row for row in production if boolean(row, "independent_for_control")]
    measurable = [row for row in independent
                  if row.get("invalid_reason") == "none" and
                  row.get("classification") not in {"", "INVALID", "DYNAMIC"}]
    trusted = [row for row in independent if boolean(row, "trusted_bw_published")]
    trigger_ids = {(row.get("flow_id"), row.get("cruise_id"),
                    row.get("trigger_cycle_id")) for row in production}

    q_peak_bdp = [number(row, "q_peak_fast_us") / rtprop_us for row in servo
                  if rtprop_us > 0.0]
    q_floor_bdp = [number(row, "q_floor_fast_us") / rtprop_us for row in servo
                   if rtprop_us > 0.0]
    factors = [number(row, "servo_factor") for row in servo]
    tail_index = len(servo) // 2
    tail_q_peak = q_peak_bdp[tail_index:] if len(q_peak_bdp) == len(servo) else q_peak_bdp
    tail_factors = factors[tail_index:]
    first_high_time = math.nan
    band_time = math.nan
    for row in servo:
        q_floor = number(row, "q_floor_fast_us")
        q_high = number(row, "q_high_us")
        time_s = number(row, "time_s")
        if not math.isfinite(first_high_time) and q_floor > q_high:
            first_high_time = time_s
        if math.isfinite(first_high_time) and not math.isfinite(band_time) and \
                q_floor <= q_high:
            band_time = time_s
    drain_rtts = ((band_time - first_high_time) / rtprop_s
                  if math.isfinite(band_time) and rtprop_s > 0.0 else math.nan)

    before_errors: list[float] = []
    after_errors: list[float] = []
    score_gt: list[tuple[float, float]] = []
    for row in independent:
        before = number(row, "baseline_before_bps")
        after = number(row, "applied_next_baseline_bps")
        if fair_share > 0.0:
            before_errors.append(abs(before - fair_share) / fair_share)
            after_errors.append(abs(after - fair_share) / fair_share)
        target = (
            number(row, "q_L_us") <= number(row, "q_min_us") <= number(row, "q_H_us") and
            number(row, "q95_event_us") <= number(row, "q_peak_cap_us") and
            abs(number(row, "queue_trend_event")) <= 0.10 and
            abs(number(row, "queue_servo_factor_mean", 1.0) - 1.0) <= 0.02 and
            number(row, "loss_ratio", 0.0) < 0.005 and
            number(row, "ecn_ratio", 0.0) < 0.02
        )
        score_gt.append((number(row, "S_target"), 1.0 if target else 0.0))

    failures: list[str] = []
    if pulser and not any(source in sources for source in ("DELIVERY_ONLY", "BOTH")):
        failures.append("FAIL_DELIVERY_TRIGGER")
    if flows == 1 and pulser and not any(source in sources for source in ("QUEUE_ONLY", "BOTH")):
        failures.append("FAIL_QUEUE_TRIGGER")
    if pulser and not passed:
        failures.append("FAIL_COMBINED_TRIGGER")
    if servo and percentile(tail_q_peak, 0.95) > 0.10:
        failures.append("FAIL_QUEUE_SERVO")
    if production and ratio(len(measurable), len(independent)) < 0.60:
        failures.append("FAIL_EVENT_WINDOW_NOT_MEASURABLE")
    if independent and median(after_errors) > median(before_errors) + 1e-9:
        failures.append("FAIL_SEARCH_NOT_CONVERGED")
    if servo and median(tail_factors) < 0.98:
        failures.append("FAIL_QUEUE_RESERVE_TARGET")
    if cruises and any(not boolean(row, "search_active") for row in cruises):
        failures.append("FAIL_PERSISTENT_SEARCH_DISABLED")
    if independent and not trusted:
        failures.append("FAIL_TRUSTED_BW_INACCURATE")

    row: dict[str, object] = {
        "run": run_dir.name,
        "flows": flows,
        "seed": meta.get("seed", math.nan),
        "buffer_bdp": meta.get("buffer_bdp", math.nan),
        "ack_jitter_us": meta.get("ack_timing_jitter_us", 0.0),
        "trigger_attempts": len(pulser),
        "trigger_passes": len(passed),
        "delivery_only": sources["DELIVERY_ONLY"],
        "queue_only": sources["QUEUE_ONLY"],
        "both": sources["BOTH"],
        "hard_safety": sources["HARD_SAFETY_ONLY"],
        "completed_windows": len(production),
        "unique_trigger_cycles": len(trigger_ids),
        "measurable_windows": len(measurable),
        "measurable_ratio": ratio(len(measurable), len(independent)),
        "median_first_direction_cycles": median([
            number(row, "window_length_cycles") for row in independent]),
        "trusted_publications": len(trusted),
        "tail_utilization": utilization,
        "tail_jain": fairness,
        "q_floor_median_bdp": median(q_floor_bdp),
        "q_peak_p95_bdp": percentile(q_peak_bdp, 0.95),
        "tail_q_peak_p95_bdp": percentile(tail_q_peak, 0.95),
        "drain_to_band_rtts": drain_rtts,
        "servo_factor_min": min(factors, default=math.nan),
        "tail_servo_factor_median": median(tail_factors),
        "baseline_error_before_median": median(before_errors),
        "baseline_error_after_median": median(after_errors),
        "score_gt_spearman": spearman(
            [pair[0] for pair in score_gt], [pair[1] for pair in score_gt]),
        "trace_hash": trace_hash(run_dir),
        "failure_labels": ";".join(dict.fromkeys(failures)) if failures else "PASS",
    }
    return row, score_gt


def fmt(value: object) -> str:
    if isinstance(value, float):
        return "nan" if not math.isfinite(value) else f"{value:.6g}"
    return str(value)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("run_dirs", nargs="+", type=Path)
    parser.add_argument("--output-dir", required=True, type=Path)
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)

    rows: list[dict[str, object]] = []
    all_score_gt: list[tuple[float, float]] = []
    for run_dir in args.run_dirs:
        row, score_gt = analyze_run(run_dir.resolve())
        rows.append(row)
        all_score_gt.extend(score_gt)

    csv_path = args.output_dir / "dual_channel_queue_servo_summary.csv"
    with csv_path.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)

    hashes = [str(row["trace_hash"]) for row in rows]
    unique_hashes = len(set(hashes))
    aggregate_failures: list[str] = []
    for row in rows:
        if row["failure_labels"] != "PASS":
            aggregate_failures.extend(str(row["failure_labels"]).split(";"))
    randomized = [row for row in rows if float(row["ack_jitter_us"]) > 0.0]
    if len(randomized) > 1 and len({row["trace_hash"] for row in randomized}) <= 1:
        aggregate_failures.append("FAIL_EXPERIMENT_NOT_IDENTIFIABLE")
    aggregate_failures = list(dict.fromkeys(
        failure for failure in aggregate_failures if failure in ALLOWED_FAILURES))
    aggregate_score_rho = spearman(
        [pair[0] for pair in all_score_gt], [pair[1] for pair in all_score_gt])

    aggregate = {
        "run_count": len(rows),
        "unique_trace_hashes": unique_hashes,
        "aggregate_score_gt_spearman": aggregate_score_rho,
        "median_tail_utilization": median([
            float(row["tail_utilization"]) for row in rows]),
        "median_tail_q_peak_p95_bdp": median([
            float(row["tail_q_peak_p95_bdp"]) for row in rows]),
        "median_drain_to_band_rtts": median([
            float(row["drain_to_band_rtts"]) for row in rows]),
        "total_delivery_only": sum(int(row["delivery_only"]) for row in rows),
        "total_queue_only": sum(int(row["queue_only"]) for row in rows),
        "total_both": sum(int(row["both"]) for row in rows),
        "total_trusted_publications": sum(
            int(row["trusted_publications"]) for row in rows),
        "failure_labels": aggregate_failures or ["PASS"],
    }
    with (args.output_dir / "dual_channel_queue_servo_aggregate.json").open("w") as handle:
        json.dump(aggregate, handle, indent=2, sort_keys=True)

    with (args.output_dir / "dual_channel_queue_servo_summary.md").open("w") as handle:
        handle.write("# F-BBR Dual-Channel Queue-Servo Validation Summary\n\n")
        handle.write("| Run | Flows | Seed | D/Q/B | Windows | Measurable | Trusted | Util | Tail q95 BDP | Drain RTTs | Factor | Result |\n")
        handle.write("|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---|\n")
        for row in rows:
            handle.write(
                f"| {row['run']} | {row['flows']} | {row['seed']} | "
                f"{row['delivery_only']}/{row['queue_only']}/{row['both']} | "
                f"{row['completed_windows']} | {fmt(row['measurable_ratio'])} | "
                f"{row['trusted_publications']} | {fmt(row['tail_utilization'])} | "
                f"{fmt(row['tail_q_peak_p95_bdp'])} | "
                f"{fmt(row['drain_to_band_rtts'])} | "
                f"{fmt(row['tail_servo_factor_median'])} | "
                f"{row['failure_labels']} |\n")
        handle.write("\nAggregate: `" + json.dumps(aggregate, sort_keys=True) + "`\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
