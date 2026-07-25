#!/usr/bin/env python3
"""Summarize Gradient-Matched FBBR-hybrid experiment traces."""

from __future__ import annotations

import argparse
import csv
import json
import math
import re
from pathlib import Path
from typing import Dict, Iterable, List, Sequence, Tuple


ACTUATOR_ACTIONS = {
    "HYBRID_REGIME_III_GRADIENT_MATCHED_HOLD",
    "HYBRID_REGIME_III_GRADIENT_MATCHED_DECREASE",
}
OLD_OVERLOAD_ACTIONS = {
    "HYBRID_REGIME_III_USE_ADAPTIVE_QUARTER_GAP",
    "HYBRID_REGIME_III_USE_RTPROP_DRATE_MIDPOINT",
    "HYBRID_REGIME_III_USE_MINIMUM",
}
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
BASELINES = {
    "fixed_4": {
        "throughput": 90.405,
        "queue_mean": 7.246,
        "queue_p95": 27.711,
        "jain": 0.916419,
    },
    "fixed_32": {
        "throughput": 97.886,
        "queue_mean": 20.629,
        "queue_p95": 34.128,
        "jain": 0.926612,
    },
    "cell_4": {
        "throughput": 40.008,
        "queue_mean": 129.357,
        "queue_p95": 760.317,
        "jain": 0.847437,
    },
    "cell_8": {
        "throughput": 42.055,
        "queue_mean": 128.422,
        "queue_p95": 730.911,
        "jain": 0.863960,
    },
}


def percentile(values: Iterable[float], quantile: float) -> float:
    ordered = sorted(value for value in values if math.isfinite(value))
    if not ordered:
        return math.nan
    if len(ordered) == 1:
        return ordered[0]
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


def read_waveform(path: Path) -> List[Dict[str, str]]:
    with path.open("r", encoding="utf-8", errors="replace") as handle:
        return list(csv.DictReader(handle))


def beta_from_row(row: Dict[str, str]) -> float:
    before = float(row["baseline_before_bps"])
    after = float(row["baseline_after_bps"])
    if not math.isfinite(before) or not math.isfinite(after) or before <= 0.0:
        return 0.0
    # The actuator itself clamps to 0.10.  Trace baselines are formatted with
    # limited decimal precision, so their reconstructed ratio can exceed the
    # exact cap by a few ppm.
    return max(0.0, min(0.10, (before - after) / before))


def summarize_overload_rows(rows: Sequence[Dict[str, str]]) -> Dict[str, float]:
    actuator = [row for row in rows if row["action"] in ACTUATOR_ACTIONS]
    betas = [beta_from_row(row) for row in actuator]
    guarded: List[Tuple[Dict[str, str], float, float]] = []
    for row in actuator:
        if (
            row.get("hybrid_srtt_low_rtprop_valid") != "true"
            or row.get("hybrid_max_srtt_valid") != "true"
        ):
            continue
        rtprop_ms = float(row["hybrid_srtt_low_rtprop_ms"])
        max_srtt_ms = float(row["hybrid_max_srtt_ms"])
        srtt_max_ms = float(row["srtt_window_max_ms"])
        if (
            not math.isfinite(rtprop_ms)
            or not math.isfinite(max_srtt_ms)
            or not math.isfinite(srtt_max_ms)
            or rtprop_ms <= 0.0
            or max_srtt_ms < rtprop_ms
        ):
            continue
        guard_ms = max(
            0.1 * rtprop_ms, (max_srtt_ms - rtprop_ms) / 3.0
        )
        guarded.append((row, guard_ms, srtt_max_ms - rtprop_ms))
    conservative_low = [
        row for row, guard_ms, queue_max_ms in guarded
        if queue_max_ms <= guard_ms + 1e-9
    ]
    return {
        "total_overload_windows": len(actuator),
        "overload_hold_windows": sum(
            row["action"].endswith("_HOLD") for row in actuator
        ),
        "overload_decrease_windows": sum(
            row["action"].endswith("_DECREASE") for row in actuator
        ),
        "mean_beta": sum(betas) / len(betas) if betas else 0.0,
        "p50_beta": percentile(betas, 0.50) if betas else 0.0,
        "p95_beta": percentile(betas, 0.95) if betas else 0.0,
        "max_beta": max(betas, default=0.0),
        "beta_at_cap_windows": sum(beta >= 0.099 for beta in betas),
        "conservative_low_queue_windows": len(conservative_low),
        "conservative_low_queue_hold_windows": sum(
            row["action"].endswith("_HOLD") for row in conservative_low
        ),
        "conservative_low_queue_decrease_windows": sum(
            row["action"].endswith("_DECREASE") for row in conservative_low
        ),
        "median_queue_guard_ms": percentile(
            (guard_ms for _, guard_ms, _ in guarded), 0.50
        ),
        "median_window_mean_queue_ms": percentile(
            (
                float(row["srtt_window_mean_ms"])
                - float(row["hybrid_srtt_low_rtprop_ms"])
                for row, _, _ in guarded
            ),
            0.50,
        ),
        "lower_bound_search_windows": sum(
            row["action"] == "HYBRID_LOWER_BOUND_SEARCH_REDUCE_0P8"
            for row in rows
        ),
        "old_discrete_overload_actions": sum(
            row["action"] in OLD_OVERLOAD_ACTIONS for row in rows
        ),
    }


def read_numeric_trace(path: Path) -> List[Tuple[float, float]]:
    rows: List[Tuple[float, float]] = []
    with path.open("r", encoding="utf-8", errors="replace") as handle:
        for line in handle:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            fields = re.split(r"\s+", line)
            if len(fields) < 2:
                continue
            try:
                rows.append((float(fields[0]), float(fields[1])))
            except ValueError:
                continue
    return sorted(rows)


def sample_previous(
    rows: Sequence[Tuple[float, float]], time_s: float
) -> float:
    if not rows:
        return math.nan
    lower = 0
    upper = len(rows)
    while lower < upper:
        middle = (lower + upper) // 2
        if rows[middle][0] <= time_s:
            lower = middle + 1
        else:
            upper = middle
    return rows[max(0, lower - 1)][1]


def time_sampled_values(
    rows: Sequence[Tuple[float, float]],
    warmup_s: float,
    end_s: float,
    step_s: float,
) -> List[float]:
    count = int(math.ceil(end_s / step_s))
    return [
        sample_previous(rows, index * step_s)
        for index in range(count)
        if warmup_s <= index * step_s < end_s
    ]


def per_flow_throughput_mbps(
    goodput_path: Path,
    warmup_s: float,
    end_s: float,
    step_s: float,
) -> float:
    values_kbps = time_sampled_values(
        read_numeric_trace(goodput_path), warmup_s, end_s, step_s
    )
    finite = [value for value in values_kbps if math.isfinite(value)]
    return sum(finite) / len(finite) / 1000.0 if finite else math.nan


def baseline_samples_bps(
    waveform_rows: Sequence[Dict[str, str]],
    warmup_s: float,
    end_s: float,
    step_s: float,
) -> List[float]:
    baseline_rows = sorted(
        (
            float(row["time_s"]),
            float(row["baseline_after_bps"]),
        )
        for row in waveform_rows
        if math.isfinite(float(row["time_s"]))
        and math.isfinite(float(row["baseline_after_bps"]))
    )
    return time_sampled_values(baseline_rows, warmup_s, end_s, step_s)


def waveform_files(run_dir: Path) -> List[Path]:
    return sorted(
        run_dir.glob("flow*_cruise_waveform_search.csv"),
        key=flow_id,
    )


def goodput_files(run_dir: Path) -> Dict[int, Path]:
    return {
        flow_id(path): path
        for path in run_dir.glob("FBBR-hybrid_flow*_FBBR-hybrid_good.txt")
    }


def summarize_scenario(run_dir: Path) -> Tuple[Dict[str, float], Dict[int, dict]]:
    per_flow: Dict[int, dict] = {}
    all_rows: List[Dict[str, str]] = []
    for path in waveform_files(run_dir):
        rows = read_waveform(path)
        all_rows.extend(rows)
        per_flow[flow_id(path)] = {
            **summarize_overload_rows(rows),
            "_rows": rows,
        }
    return summarize_overload_rows(all_rows), per_flow


def read_summary(path: Path) -> Dict[str, str]:
    with path.open("r", encoding="utf-8") as handle:
        rows = list(csv.DictReader(handle))
    if len(rows) != 1:
        raise ValueError(f"expected one summary row in {path}, got {len(rows)}")
    return rows[0]


def write_csv(path: Path, fieldnames: Sequence[str], rows: Sequence[dict]) -> None:
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--result-root", type=Path, required=True)
    parser.add_argument("--baseline-root", type=Path, required=True)
    parser.add_argument("--warmup-s", type=float, default=5.0)
    parser.add_argument("--step-s", type=float, default=0.1)
    args = parser.parse_args()
    result_root = args.result_root.resolve()
    baseline_root = args.baseline_root.resolve()

    overload_rows: List[dict] = []
    metric_rows: List[dict] = []
    scenario_per_flow: Dict[str, Dict[int, dict]] = {}
    for scenario, relative in SCENARIOS.items():
        scenario_dir = result_root / relative
        run_dir = scenario_dir / "FBBR-hybrid"
        overload, per_flow = summarize_scenario(run_dir)
        scenario_per_flow[scenario] = per_flow
        overload_rows.append({"scenario": scenario, **overload})

        summary = read_summary(scenario_dir / "compare" / "summary_metrics.csv")
        actual = {
            "throughput": float(
                summary["avg_aggregate_throughput_mbps_after_warmup"]
            ),
            "capacity": float(
                summary["avg_bottleneck_capacity_mbps_after_warmup"]
            ),
            "utilization": float(
                summary["avg_bottleneck_utilization_after_warmup"]
            ),
            "queue_mean": float(summary["avg_queue_delay_ms_after_warmup"]),
            "queue_p95": float(summary["p95_queue_delay_ms_after_warmup"]),
            "jain": float(summary["avg_jain_fairness_after_warmup"]),
            "loss_pct": float(summary["aggregate_loss_rate_pct_whole_run"]),
            "owd_ms": float(summary["avg_one_way_delay_ms_whole_run"]),
            "total_gib": float(summary["total_received_bytes_whole_run"])
            / (1024.0**3),
        }
        baseline = BASELINES[scenario]
        metric_rows.append(
            {
                "scenario": scenario,
                **actual,
                "throughput_delta": actual["throughput"]
                - baseline["throughput"],
                "throughput_delta_pct": 100.0
                * (actual["throughput"] / baseline["throughput"] - 1.0),
                "queue_mean_delta": actual["queue_mean"]
                - baseline["queue_mean"],
                "queue_mean_delta_pct": 100.0
                * (actual["queue_mean"] / baseline["queue_mean"] - 1.0),
                "queue_p95_delta": actual["queue_p95"]
                - baseline["queue_p95"],
                "queue_p95_delta_pct": 100.0
                * (actual["queue_p95"] / baseline["queue_p95"] - 1.0),
                "jain_delta": actual["jain"] - baseline["jain"],
            }
        )

    overload_fields = [
        "scenario",
        "total_overload_windows",
        "overload_hold_windows",
        "overload_decrease_windows",
        "mean_beta",
        "p50_beta",
        "p95_beta",
        "max_beta",
        "beta_at_cap_windows",
        "conservative_low_queue_windows",
        "conservative_low_queue_hold_windows",
        "conservative_low_queue_decrease_windows",
        "median_queue_guard_ms",
        "median_window_mean_queue_ms",
        "lower_bound_search_windows",
        "old_discrete_overload_actions",
    ]
    write_csv(result_root / "overload_summary.csv", overload_fields, overload_rows)
    metric_fields = list(metric_rows[0].keys())
    write_csv(result_root / "scenario_metrics.csv", metric_fields, metric_rows)

    fixed4_dir = result_root / SCENARIOS["fixed_4"] / "FBBR-hybrid"
    fixed4_old_dir = baseline_root / "fixed_4flows_FBBR-hybrid"
    new_goodput = goodput_files(fixed4_dir)
    old_goodput = goodput_files(fixed4_old_dir)
    old_waveforms = {
        flow_id(path): read_waveform(path)
        for path in waveform_files(fixed4_old_dir)
    }
    fixed4_rows: List[dict] = []
    for flow, values in sorted(scenario_per_flow["fixed_4"].items()):
        rows = values.pop("_rows")
        new_baselines = baseline_samples_bps(
            rows, args.warmup_s, 240.0, args.step_s
        )
        old_rows = old_waveforms[flow]
        old_baselines = baseline_samples_bps(
            old_rows, args.warmup_s, 240.0, args.step_s
        )
        old_overload = summarize_overload_rows(old_rows)
        new_throughput = per_flow_throughput_mbps(
            new_goodput[flow], args.warmup_s, 240.0, args.step_s
        )
        old_throughput = per_flow_throughput_mbps(
            old_goodput[flow], args.warmup_s, 240.0, args.step_s
        )
        fixed4_rows.append(
            {
                "flow": flow,
                "throughput_mbps": new_throughput,
                "old_throughput_mbps": old_throughput,
                "throughput_delta_mbps": new_throughput - old_throughput,
                "avg_baseline_mbps": sum(new_baselines)
                / len(new_baselines)
                / 1e6,
                "old_avg_baseline_mbps": sum(old_baselines)
                / len(old_baselines)
                / 1e6,
                "avg_baseline_delta_mbps": (
                    sum(new_baselines) / len(new_baselines)
                    - sum(old_baselines) / len(old_baselines)
                )
                / 1e6,
                "p10_baseline_mbps": percentile(new_baselines, 0.10) / 1e6,
                "old_p10_baseline_mbps": percentile(old_baselines, 0.10)
                / 1e6,
                "overload_count": values["total_overload_windows"],
                "hold_count": values["overload_hold_windows"],
                "decrease_count": values["overload_decrease_windows"],
                "mean_beta": values["mean_beta"],
                "old_discrete_overload_count": old_overload[
                    "old_discrete_overload_actions"
                ],
                "lower_bound_search_windows": values[
                    "lower_bound_search_windows"
                ],
            }
        )
    write_csv(
        result_root / "fixed4_per_flow.csv",
        list(fixed4_rows[0].keys()),
        fixed4_rows,
    )

    checks = {
        "all_scenario_loss_zero": all(
            row["loss_pct"] == 0.0 for row in metric_rows
        ),
        "all_actuator_beta_at_most_0p10": all(
            row["max_beta"] <= 0.100001 for row in overload_rows
        ),
        "no_old_discrete_overload_actions": all(
            row["old_discrete_overload_actions"] == 0
            for row in overload_rows
        ),
        "all_scenarios_have_hold_and_decrease": all(
            row["overload_hold_windows"] > 0
            and row["overload_decrease_windows"] > 0
            for row in overload_rows
        ),
    }
    with (result_root / "checks.json").open("w", encoding="utf-8") as handle:
        json.dump(checks, handle, indent=2, sort_keys=True)
        handle.write("\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
