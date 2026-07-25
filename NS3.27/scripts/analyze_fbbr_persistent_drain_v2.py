#!/usr/bin/env python3
"""Build the reproducible result tables for FBBR Persistent Drain V2."""

from __future__ import annotations

import csv
import json
import math
import re
from pathlib import Path
from typing import Iterable, Sequence


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

ORIGINAL_DIV3 = {
    "fixed_4": {
        "throughput_mbps": 90.405,
        "avg_queue_ms": 7.246,
        "p95_queue_ms": 27.711,
        "jain": 0.916419,
        "loss_pct": 0.0,
    },
    "fixed_32": {
        "throughput_mbps": 97.886,
        "avg_queue_ms": 20.629,
        "p95_queue_ms": 34.128,
        "jain": 0.926612,
        "loss_pct": 0.0,
    },
    "cell_4": {
        "throughput_mbps": 40.008,
        "avg_queue_ms": 129.357,
        "p95_queue_ms": 760.317,
        "jain": 0.847437,
        "loss_pct": 0.0,
    },
    "cell_8": {
        "throughput_mbps": 42.055,
        "avg_queue_ms": 128.422,
        "p95_queue_ms": 730.911,
        "jain": 0.863960,
        "loss_pct": 0.0,
    },
}

GRADIENT_V1 = {
    "fixed_4": {
        "throughput_mbps": 97.386,
        "avg_queue_ms": 11.805,
        "p95_queue_ms": 21.427,
        "jain": 0.953042,
        "loss_pct": 0.0,
    },
    "fixed_32": {
        "throughput_mbps": 97.993,
        "avg_queue_ms": 85.639,
        "p95_queue_ms": 129.574,
        "jain": 0.836427,
        "loss_pct": 0.0,
    },
    "cell_4": {
        "throughput_mbps": 42.341,
        "avg_queue_ms": 70.573,
        "p95_queue_ms": 372.294,
        "jain": 0.864459,
        "loss_pct": 0.0,
    },
    "cell_8": {
        "throughput_mbps": 42.789,
        "avg_queue_ms": 126.014,
        "p95_queue_ms": 716.208,
        "jain": 0.776970,
        "loss_pct": 0.0,
    },
}

SUMMARY_RE = re.compile(
    r"flow_id=(?P<flow>\d+) "
    r"drain_entry_count=(?P<entries>\d+) "
    r"drain_window_count=(?P<windows>\d+) "
    r"drain_exit_count=(?P<exits>\d+) "
    r"mean_drain_duration_windows=(?P<mean_duration>[0-9.eE+-]+) "
    r"baseline_blocked_increase_count=(?P<blocked>\d+) "
    r"mean_beta_queue=(?P<mean_beta>[0-9.eE+-]+) "
    r"p95_beta_queue=(?P<p95_beta>[0-9.eE+-]+)"
)


def percentile(values: Iterable[float], q: float) -> float:
    ordered = sorted(value for value in values if math.isfinite(value))
    if not ordered:
        return math.nan
    if len(ordered) == 1:
        return ordered[0]
    position = q * (len(ordered) - 1)
    lo = int(math.floor(position))
    hi = int(math.ceil(position))
    if lo == hi:
        return ordered[lo]
    return ordered[lo] + (position - lo) * (ordered[hi] - ordered[lo])


def flow_id(path: Path) -> int:
    match = re.search(r"(?:^|_)flow(\d+)(?:_|$)", path.name)
    if not match:
        match = re.search(r"^flow(\d+)_", path.name)
    if not match:
        raise ValueError(f"cannot extract flow id from {path}")
    return int(match.group(1))


def read_csv(path: Path) -> list[dict[str, str]]:
    with path.open("r", encoding="utf-8", errors="replace") as handle:
        return list(csv.DictReader(handle))


def write_csv(path: Path, rows: Sequence[dict], fields: Sequence[str]) -> None:
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def read_numeric_trace(path: Path) -> list[tuple[float, float]]:
    rows: list[tuple[float, float]] = []
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
                pass
    return sorted(rows)


def sample_previous(rows: Sequence[tuple[float, float]], at_s: float) -> float:
    lo = 0
    hi = len(rows)
    while lo < hi:
        mid = (lo + hi) // 2
        if rows[mid][0] <= at_s:
            lo = mid + 1
        else:
            hi = mid
    if not rows:
        return math.nan
    return rows[max(0, lo - 1)][1]


def time_samples(
    rows: Sequence[tuple[float, float]],
    start_s: float,
    end_s: float,
    step_s: float = 0.1,
) -> list[float]:
    return [
        sample_previous(rows, index * step_s)
        for index in range(int(math.ceil(end_s / step_s)))
        if start_s <= index * step_s < end_s
    ]


def waveform_rows(run_dir: Path) -> dict[int, list[dict[str, str]]]:
    return {
        flow_id(path): read_csv(path)
        for path in sorted(
            run_dir.glob("flow*_cruise_waveform_search.csv"), key=flow_id
        )
    }


def parse_runtime_summaries(run_dir: Path, scenario: str) -> list[dict]:
    rows: list[dict] = []
    log = (run_dir / "run.log").read_text(
        encoding="utf-8", errors="replace"
    )
    for match in SUMMARY_RE.finditer(log):
        group = match.groupdict()
        rows.append(
            {
                "scenario": scenario,
                "flow": int(group["flow"]),
                "drain_entry_count": int(group["entries"]),
                "drain_window_count": int(group["windows"]),
                "drain_exit_count": int(group["exits"]),
                "mean_drain_duration_windows": float(
                    group["mean_duration"]
                ),
                "baseline_blocked_increase_count": int(group["blocked"]),
                "mean_beta_queue": float(group["mean_beta"]),
                "p95_beta_queue": float(group["p95_beta"]),
            }
        )
    return sorted(rows, key=lambda row: row["flow"])


def read_scenario_metric(scenario_root: Path) -> dict[str, float]:
    row = read_csv(scenario_root / "compare" / "summary_metrics.csv")[0]
    return {
        "throughput_mbps": float(
            row["avg_aggregate_throughput_mbps_after_warmup"]
        ),
        "avg_queue_ms": float(row["avg_queue_delay_ms_after_warmup"]),
        "p95_queue_ms": float(row["p95_queue_delay_ms_after_warmup"]),
        "jain": float(row["avg_jain_fairness_after_warmup"]),
        "loss_pct": float(row["aggregate_loss_rate_pct_whole_run"]),
        "avg_one_way_delay_ms": float(
            row["avg_one_way_delay_ms_whole_run"]
        ),
        "total_received_gib": float(row["total_received_bytes_whole_run"])
        / (1024.0**3),
        "capacity_mbps": float(
            row["avg_bottleneck_capacity_mbps_after_warmup"]
        ),
        "utilization": float(
            row["avg_bottleneck_utilization_after_warmup"]
        ),
    }


def fixed4_per_flow(run_dir: Path) -> list[dict]:
    waveforms = waveform_rows(run_dir)
    goodput = {
        flow_id(path): path
        for path in run_dir.glob(
            "FBBR-hybrid_flow*_FBBR-hybrid_good.txt"
        )
    }
    output = []
    for flow in sorted(waveforms):
        throughput_values = time_samples(
            read_numeric_trace(goodput[flow]), 5.0, 240.0
        )
        baseline_trace = sorted(
            (
                float(row["time_s"]),
                float(row["baseline_after_bps"]),
            )
            for row in waveforms[flow]
        )
        baseline_values = time_samples(baseline_trace, 5.0, 240.0)
        output.append(
            {
                "flow": flow,
                "throughput_mbps": sum(throughput_values)
                / len(throughput_values)
                / 1000.0,
                "avg_baseline_mbps": sum(baseline_values)
                / len(baseline_values)
                / 1e6,
                "p10_baseline_mbps": percentile(baseline_values, 0.10)
                / 1e6,
                "final_baseline_mbps": baseline_trace[-1][1] / 1e6,
            }
        )
    return output


def valid_classification_rows(
    rows: Sequence[dict[str, str]],
) -> list[dict[str, str]]:
    return [
        row
        for row in rows
        if row.get("classification")
        in {"UNDERLOAD", "FULL_LOAD", "OVERLOAD"}
        and float(row.get("q_guard", "nan")) > 0.0
    ]


def fixed32_diagnostics(
    per_flow: dict[int, list[dict[str, str]]],
    runtime: Sequence[dict],
) -> dict:
    all_valid = [
        row
        for rows in per_flow.values()
        for row in valid_classification_rows(rows)
    ]
    decreases: list[tuple[list[dict[str, str]], int]] = []
    rebound_one = 0
    rebound_two = 0
    u_values: list[float] = []
    actuation_values: list[float] = []
    for rows in per_flow.values():
        valid = valid_classification_rows(rows)
        for index, row in enumerate(valid):
            before = float(row["baseline_before_bps"])
            after = float(row["baseline_after_bps"])
            delta = before - after
            if (
                row["classification"] != "OVERLOAD"
                or delta <= 0.5
                or "DECREASE" not in row["action"]
            ):
                continue
            decreases.append((valid, index))
            later = valid[index + 1 : index + 3]
            if later and float(later[0]["baseline_after_bps"]) > after + 0.5:
                rebound_one += 1
            if any(
                float(item["baseline_after_bps"]) > after + 0.5
                for item in later
            ):
                rebound_two += 1
            if later:
                u_values.append(
                    max(
                        0.0,
                        float(later[0]["baseline_after_bps"]) - after,
                    )
                    / (delta + 1e-9)
                )
                pacing_pre = float(row["actual_pacing_mean"])
                pacing_post = float(later[0]["actual_pacing_mean"])
                if math.isfinite(pacing_pre) and math.isfinite(pacing_post):
                    actuation_values.append(
                        (pacing_pre - pacing_post) / (delta + 1e-9)
                    )

    durations = [
        int(row["drain_window_count"] / row["drain_entry_count"])
        for row in runtime
        if row["drain_entry_count"] == 1
    ]
    duration_histogram = {
        str(value): durations.count(value) for value in sorted(set(durations))
    }
    q50_ms = [float(row["q50"]) * 1000.0 for row in all_valid]
    q90_ms = [float(row["q90"]) * 1000.0 for row in all_valid]
    qguard_ms = [float(row["q_guard"]) * 1000.0 for row in all_valid]
    count = len(decreases)
    return {
        "valid_classification_windows": len(all_valid),
        "q50_p50_ms": percentile(q50_ms, 0.50),
        "q50_p95_ms": percentile(q50_ms, 0.95),
        "q90_p50_ms": percentile(q90_ms, 0.50),
        "q90_p95_ms": percentile(q90_ms, 0.95),
        "q_guard_p50_ms": percentile(qguard_ms, 0.50),
        "q_guard_p95_ms": percentile(qguard_ms, 0.95),
        "drain_duration_histogram_windows": duration_histogram,
        "drain_duration_p50_windows": percentile(durations, 0.50),
        "drain_duration_p95_windows": percentile(durations, 0.95),
        "drain_duration_min_windows": min(durations, default=math.nan),
        "drain_duration_max_windows": max(durations, default=math.nan),
        "completed_drains": sum(row["drain_exit_count"] for row in runtime),
        "right_censored_drains": sum(
            row["drain_entry_count"] - row["drain_exit_count"]
            for row in runtime
        ),
        "blocked_increases": sum(
            row["baseline_blocked_increase_count"] for row in runtime
        ),
        "overload_decrease_count": count,
        "increase_within_1_window_ratio": rebound_one / count
        if count
        else math.nan,
        "increase_within_2_windows_ratio": rebound_two / count
        if count
        else math.nan,
        "median_U": percentile(u_values, 0.50),
        "p95_U": percentile(u_values, 0.95),
        "actuation_sample_count": len(actuation_values),
        "actuation_A_p50": percentile(actuation_values, 0.50),
        "actuation_A_p10": percentile(actuation_values, 0.10),
        "actuation_A_lt_0_5_ratio": (
            sum(value < 0.5 for value in actuation_values)
            / len(actuation_values)
            if actuation_values
            else math.nan
        ),
    }


def aggregate_drain_summary(
    scenario: str,
    runtime: Sequence[dict],
    per_flow: dict[int, list[dict[str, str]]],
) -> dict:
    entries = sum(row["drain_entry_count"] for row in runtime)
    windows = sum(row["drain_window_count"] for row in runtime)
    exits = sum(row["drain_exit_count"] for row in runtime)
    blocked = sum(row["baseline_blocked_increase_count"] for row in runtime)
    mean_beta = (
        sum(
            row["mean_beta_queue"] * row["drain_window_count"]
            for row in runtime
        )
        / windows
        if windows
        else 0.0
    )
    beta_samples: list[float] = []
    for rows in per_flow.values():
        was_active = False
        for row in rows:
            is_valid = row.get("classification") in {
                "UNDERLOAD",
                "FULL_LOAD",
                "OVERLOAD",
            }
            active = row.get("drain_active") == "true"
            if is_valid and (active or was_active):
                beta_samples.append(float(row["beta_queue"]))
            was_active = active
    return {
        "scenario": scenario,
        "drain_entry_count": entries,
        "drain_window_count": windows,
        "drain_exit_count": exits,
        "mean_drain_duration_windows": windows / entries if entries else 0.0,
        "baseline_blocked_increase_count": blocked,
        "mean_beta_queue": mean_beta,
        "p95_beta_queue": percentile(beta_samples, 0.95),
    }


def trace_checks(all_waveforms: dict[str, dict[int, list[dict]]]) -> dict:
    required = {
        "drain_active",
        "q50",
        "q90",
        "q_guard",
        "beta_v1",
        "beta_queue",
        "beta_final",
        "drain_ceiling",
        "actual_pacing_mean",
    }
    missing: list[str] = []
    ceiling_violations = 0
    minimum_violations = 0
    beta_cap_violations = 0
    nonfinite_actual_pacing = 0
    max_beta = 0.0
    min_baseline = math.inf
    for scenario, flows in all_waveforms.items():
        for flow, rows in flows.items():
            if rows:
                for field in sorted(required - set(rows[0])):
                    missing.append(f"{scenario}:flow{flow}:{field}")
            for row in rows:
                baseline = float(row["baseline_after_bps"])
                beta = float(row["beta_final"])
                min_baseline = min(min_baseline, baseline)
                max_beta = max(max_beta, beta)
                if baseline < 1_000_000.0 - 0.5:
                    minimum_violations += 1
                if beta > 0.100001:
                    beta_cap_violations += 1
                if row["drain_active"] == "true":
                    ceiling = float(row["drain_ceiling"])
                    if baseline > ceiling + 1.0:
                        ceiling_violations += 1
                if not math.isfinite(float(row["actual_pacing_mean"])):
                    nonfinite_actual_pacing += 1
    return {
        "missing_trace_fields": missing,
        "ceiling_violations": ceiling_violations,
        "minimum_rate_violations": minimum_violations,
        "beta_cap_violations": beta_cap_violations,
        "nonfinite_actual_pacing_rows": nonfinite_actual_pacing,
        "minimum_observed_baseline_bps": min_baseline,
        "maximum_observed_beta_final": max_beta,
    }


def acceptance(v2: dict[str, dict[str, float]]) -> dict:
    checks = {
        "fixed_4_throughput_ge_95": v2["fixed_4"]["throughput_mbps"]
        >= 95.0,
        "fixed_4_avg_queue_le_8": v2["fixed_4"]["avg_queue_ms"] <= 8.0,
        "fixed_4_p95_queue_le_27_711": v2["fixed_4"]["p95_queue_ms"]
        <= 27.711,
        "fixed_32_throughput_ge_97": v2["fixed_32"]["throughput_mbps"]
        >= 97.0,
        "fixed_32_avg_queue_le_22_7": v2["fixed_32"]["avg_queue_ms"]
        <= 22.7,
        "fixed_32_p95_queue_le_37_5": v2["fixed_32"]["p95_queue_ms"]
        <= 37.5,
    }
    for scenario in ("cell_4", "cell_8"):
        checks[f"{scenario}_throughput_within_2pct_of_div3"] = (
            v2[scenario]["throughput_mbps"]
            >= 0.98 * ORIGINAL_DIV3[scenario]["throughput_mbps"]
        )
        checks[f"{scenario}_avg_queue_not_above_div3"] = (
            v2[scenario]["avg_queue_ms"]
            <= ORIGINAL_DIV3[scenario]["avg_queue_ms"]
        )
        checks[f"{scenario}_p95_queue_not_above_div3"] = (
            v2[scenario]["p95_queue_ms"]
            <= ORIGINAL_DIV3[scenario]["p95_queue_ms"]
        )
    checks["all_loss_zero"] = all(
        metric["loss_pct"] == 0.0 for metric in v2.values()
    )
    for scenario in SCENARIOS:
        checks[f"{scenario}_jain_within_0_02_of_div3"] = (
            v2[scenario]["jain"]
            >= ORIGINAL_DIV3[scenario]["jain"] - 0.02
        )
    return {**checks, "overall": all(checks.values())}


def main() -> int:
    ns3 = Path(__file__).resolve().parents[1]
    root = ns3 / "results" / "fbbr_persistent_queue_drain_v2"
    all_metrics: dict[str, dict[str, float]] = {}
    all_waveforms: dict[str, dict[int, list[dict]]] = {}
    runtime_rows: list[dict] = []
    aggregate_rows: list[dict] = []

    for scenario, relative in SCENARIOS.items():
        scenario_root = root / relative
        run_dir = scenario_root / "FBBR-hybrid"
        all_metrics[scenario] = read_scenario_metric(scenario_root)
        all_waveforms[scenario] = waveform_rows(run_dir)
        runtime = parse_runtime_summaries(run_dir, scenario)
        runtime_rows.extend(runtime)
        aggregate_rows.append(
            aggregate_drain_summary(
                scenario, runtime, all_waveforms[scenario]
            )
        )

    metric_rows = [
        {"scenario": scenario, **all_metrics[scenario]}
        for scenario in SCENARIOS
    ]
    write_csv(
        root / "v2_scenario_metrics.csv",
        metric_rows,
        list(metric_rows[0]),
    )

    abc_rows: list[dict] = []
    for scenario in SCENARIOS:
        for version, metrics in (
            ("A_original_div3", ORIGINAL_DIV3[scenario]),
            ("B_gradient_v1", GRADIENT_V1[scenario]),
            ("C_persistent_drain_v2", all_metrics[scenario]),
        ):
            abc_rows.append(
                {
                    "scenario": scenario,
                    "version": version,
                    **{
                        key: metrics[key]
                        for key in (
                            "throughput_mbps",
                            "avg_queue_ms",
                            "p95_queue_ms",
                            "jain",
                            "loss_pct",
                        )
                    },
                }
            )
        abc_rows.append(
            {
                "scenario": scenario,
                "version": "C_minus_A",
                **{
                    key: all_metrics[scenario][key]
                    - ORIGINAL_DIV3[scenario][key]
                    for key in (
                        "throughput_mbps",
                        "avg_queue_ms",
                        "p95_queue_ms",
                        "jain",
                        "loss_pct",
                    )
                },
            }
        )
        abc_rows.append(
            {
                "scenario": scenario,
                "version": "C_minus_B",
                **{
                    key: all_metrics[scenario][key]
                    - GRADIENT_V1[scenario][key]
                    for key in (
                        "throughput_mbps",
                        "avg_queue_ms",
                        "p95_queue_ms",
                        "jain",
                        "loss_pct",
                    )
                },
            }
        )
    write_csv(root / "abc_comparison.csv", abc_rows, list(abc_rows[0]))
    write_csv(
        root / "drain_per_flow.csv",
        runtime_rows,
        list(runtime_rows[0]),
    )
    write_csv(
        root / "drain_scenario_summary.csv",
        aggregate_rows,
        list(aggregate_rows[0]),
    )

    fixed4 = fixed4_per_flow(
        root / SCENARIOS["fixed_4"] / "FBBR-hybrid"
    )
    write_csv(root / "fixed4_per_flow.csv", fixed4, list(fixed4[0]))

    fixed32_runtime = [
        row for row in runtime_rows if row["scenario"] == "fixed_32"
    ]
    fixed32 = fixed32_diagnostics(
        all_waveforms["fixed_32"], fixed32_runtime
    )
    with (root / "fixed32_diagnostics.json").open(
        "w", encoding="utf-8"
    ) as handle:
        json.dump(fixed32, handle, indent=2, sort_keys=True)
        handle.write("\n")

    checks = {
        "trace": trace_checks(all_waveforms),
        "acceptance": acceptance(all_metrics),
    }
    with (root / "checks.json").open("w", encoding="utf-8") as handle:
        json.dump(checks, handle, indent=2, sort_keys=True)
        handle.write("\n")

    print(json.dumps(
        {
            "v2_metrics": all_metrics,
            "drain_summary": aggregate_rows,
            "fixed32": fixed32,
            "checks": checks,
        },
        indent=2,
        sort_keys=True,
    ))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
