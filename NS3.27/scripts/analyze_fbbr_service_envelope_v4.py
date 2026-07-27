#!/usr/bin/env python3
"""Analyze FBBR service-consistent inflight-envelope runs."""

from __future__ import annotations

import argparse
import bisect
import csv
import json
import math
from collections import defaultdict
from pathlib import Path
from typing import Dict, Iterable, List, Sequence, Tuple

import analyze_fbbr_model_projection_v3 as common


SCENARIOS = common.SCENARIOS
ALGORITHMS = ("FBBR-hybrid", "FBBR-hybridv3", "FBBR")
BASELINES = ("BBR-R", "BBRv2")
BASELINE_SUMMARIES = {
    "fixed_4": (
        "fbbr_hybrid_vs_bbrr_4flows_shared_100M_5BDP_240s_20260722"
        "/compare/summary_metrics.csv"
    ),
    "fixed_32": (
        "fbbr_hybrid_vs_bbrr_vs_bbrv2_32flows_shared_100M_5BDP_240s_"
        "20260722/compare/summary_metrics.csv"
    ),
    "cell_4": (
        "cellular_links_taxi_128M_4flow_8flow_5BDP_180s_3cc_20260722"
        "/cellular_taxi_128M_4flows_5BDP_180s/compare/"
        "summary_metrics.csv"
    ),
    "cell_8": (
        "cellular_links_taxi_128M_4flow_8flow_5BDP_180s_3cc_20260722"
        "/cellular_taxi_128M_8flows_5BDP_180s/compare/"
        "summary_metrics.csv"
    ),
}


def f(value: object, default: float = math.nan) -> float:
    return common.finite_float(value, default)


def write_csv(path: Path, rows: Sequence[Dict[str, object]]) -> None:
    if not rows:
        common.write_csv(path, rows)
        return
    fieldnames: List[str] = []
    seen = set()
    for row in rows:
        for field in row:
            if field not in seen:
                seen.add(field)
                fieldnames.append(field)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def load_metrics(path: Path) -> Dict[str, Dict[str, str]]:
    return common.read_metrics(path)


def summarize_v4(run_dir: Path, scenario: str) -> Dict[str, object]:
    rows: List[Dict[str, str]] = []
    for path in sorted(
        run_dir.glob("flow*_v4_service_envelope_summary.csv"),
        key=common.flow_id,
    ):
        rows.extend(common.read_csv(path))
    fields = [
        "reference_trusted_time_ratio",
        "reference_guard_time_ratio",
        "reference_last_valid_time_ratio",
        "reference_invalid_time_ratio",
        "projection_active_time_ratio",
        "service_history_valid_time_ratio",
        "app_limited_fallback_time_ratio",
        "plan_only_fallback_time_ratio",
        "service_limited_time_ratio",
        "cap_binding_time_ratio",
        "mean_plan_inflight",
        "p95_plan_inflight",
        "mean_service_inflight",
        "p95_service_inflight",
        "mean_probe_credit",
        "p95_probe_credit",
        "mean_extra_acked",
        "p95_extra_acked",
        "mean_service_restriction",
        "p95_service_restriction",
        "mean_enforced_excess",
        "p95_enforced_excess",
    ]
    result: Dict[str, object] = {
        "scenario": scenario,
        "flow_count": len(rows),
    }
    for field in fields:
        result[field] = common.mean(f(row.get(field)) for row in rows)
    return result


def load_schedule(config_path: Path) -> List[Tuple[float, float]]:
    config = json.loads(config_path.read_text(encoding="utf-8"))
    result: List[Tuple[float, float]] = []
    for item in str(config.get("capacity_schedule", "")).split(","):
        if not item or ":" not in item:
            continue
        time_text, rate_text = item.split(":", 1)
        text = rate_text.strip().lower()
        scale = 1.0
        for suffix, multiplier in (
            ("gbps", 1e9),
            ("mbps", 1e6),
            ("kbps", 1e3),
            ("bps", 1.0),
        ):
            if text.endswith(suffix):
                text = text[: -len(suffix)]
                scale = multiplier
                break
        result.append((float(time_text), float(text) * scale))
    return sorted(result)


def load_queue(run_dir: Path) -> Tuple[List[float], List[float]]:
    times: List[float] = []
    delays: List[float] = []
    path = run_dir / "bottleneck_queue.csv"
    if not path.exists():
        return times, delays
    for row in common.read_csv(path):
        time_s = f(row.get("time_s"))
        queue_bytes = f(row.get("queue_bytes"))
        capacity = f(row.get("capacity_bps"))
        if (
            math.isfinite(time_s)
            and math.isfinite(queue_bytes)
            and capacity > 0.0
        ):
            times.append(time_s)
            delays.append(queue_bytes * 8.0e3 / capacity)
    return times, delays


def nearest_queue_delay(
    time_s: float, times: Sequence[float], delays: Sequence[float]
) -> float:
    if not times:
        return math.nan
    index = bisect.bisect_left(times, time_s)
    candidates = [max(0, min(index, len(times) - 1))]
    if index > 0:
        candidates.append(index - 1)
    best = min(candidates, key=lambda item: abs(times[item] - time_s))
    return delays[best]


def transition_kind(before: float, after: float) -> str:
    if before <= 0.0:
        return "stable"
    ratio = after / before
    if ratio < 0.95:
        return "capacity_down"
    if ratio > 1.05:
        return "capacity_up"
    return "capacity_stable"


def transition_aligned_rows(
    run_dir: Path, scenario: str
) -> List[Dict[str, object]]:
    schedule = load_schedule(run_dir / "config.json")
    if not schedule:
        return []
    schedule_times = [item[0] for item in schedule]
    queue_times, queue_delays = load_queue(run_dir)
    result: List[Dict[str, object]] = []
    for path in sorted(
        run_dir.glob("flow*_cruise_waveform_search.csv"),
        key=common.flow_id,
    ):
        fid = common.flow_id(path)
        for row in common.read_csv(path):
            time_s = f(row.get("time_s"))
            if not math.isfinite(time_s):
                continue
            index = max(0, bisect.bisect_right(schedule_times, time_s) - 1)
            after = schedule[index][1]
            before = schedule[index - 1][1] if index > 0 else after
            envelope = f(row.get("v4_envelope"), 0.0)
            cap = f(row.get("v4_inflight_cap"), 0.0)
            extra = f(row.get("v4_extra_acked"), 0.0)
            result.append(
                {
                    "scenario": scenario,
                    "flow_id": fid,
                    "time_s": time_s,
                    "transition_time_s": schedule[index][0],
                    "seconds_after_transition": time_s - schedule[index][0],
                    "window_class": transition_kind(before, after),
                    "capacity_before_bps": before,
                    "capacity_after_bps": after,
                    "plan_inflight": f(row.get("v4_plan_inflight"), 0.0),
                    "service_inflight": f(
                        row.get("v4_service_inflight"), 0.0
                    ),
                    "positive_probe_credit": f(
                        row.get("v4_positive_probe_credit"), 0.0
                    ),
                    "service_budget": f(
                        row.get("v4_service_budget"), 0.0
                    ),
                    "final_envelope": envelope,
                    "actual_inflight": f(
                        row.get("v4_actual_inflight"), 0.0
                    ),
                    "queue_delay_ms": nearest_queue_delay(
                        time_s, queue_times, queue_delays
                    ),
                    "reference_bw_bps": f(
                        row.get("v4_reference_bw"), 0.0
                    ),
                    "cap_binding_fraction": f(
                        row.get("v4_cap_binding_fraction"), 0.0
                    ),
                    "service_history_valid": (
                        row.get("v4_service_history_valid") == "true"
                    ),
                    "app_limited_contaminated": (
                        row.get("v4_app_limited_contaminated") == "true"
                    ),
                    "projection_active": (
                        row.get("v4_projection_active") == "true"
                    ),
                    "service_restriction": f(
                        row.get("v4_service_restriction"), 0.0
                    ),
                    "extra_acked": extra,
                    "inflight_cap": cap,
                    "extra_acked_cap_ratio": (
                        extra / cap if cap > 0.0 else math.nan
                    ),
                }
            )
    return result


def aggregate_transition_classes(
    rows: Sequence[Dict[str, object]]
) -> List[Dict[str, object]]:
    grouped: Dict[Tuple[str, str], List[Dict[str, object]]] = defaultdict(list)
    for row in rows:
        grouped[(str(row["scenario"]), str(row["window_class"]))].append(row)
    result: List[Dict[str, object]] = []
    numeric = [
        "seconds_after_transition",
        "capacity_before_bps",
        "capacity_after_bps",
        "plan_inflight",
        "service_inflight",
        "positive_probe_credit",
        "service_budget",
        "final_envelope",
        "actual_inflight",
        "queue_delay_ms",
        "reference_bw_bps",
        "cap_binding_fraction",
        "service_restriction",
        "extra_acked",
        "extra_acked_cap_ratio",
    ]
    for (scenario, kind), selected in sorted(grouped.items()):
        output: Dict[str, object] = {
            "scenario": scenario,
            "window_class": kind,
            "sample_count": len(selected),
            "service_history_valid_ratio": common.mean(
                1.0 if row["service_history_valid"] else 0.0
                for row in selected
            ),
            "projection_active_ratio": common.mean(
                1.0 if row["projection_active"] else 0.0
                for row in selected
            ),
            "app_limited_contaminated_ratio": common.mean(
                1.0 if row["app_limited_contaminated"] else 0.0
                for row in selected
            ),
        }
        for field in numeric:
            values = [f(row[field]) for row in selected]
            output[f"mean_{field}"] = common.mean(values)
            output[f"p95_{field}"] = common.percentile(values, 0.95)
        result.append(output)
    return result


def queue_transition_summary(
    run_dir: Path, scenario: str, algorithm: str
) -> List[Dict[str, object]]:
    schedule = load_schedule(run_dir / "config.json")
    queue_times, queue_delays = load_queue(run_dir)
    if len(schedule) < 2 or not queue_times:
        return []
    phases = (
        ("pre_20ms", -0.020, 0.0),
        ("feedback_0_20ms", 0.0, 0.020),
        ("feedback_20_50ms", 0.020, 0.050),
        ("settled_50_90ms", 0.050, 0.090),
    )
    grouped: Dict[Tuple[str, str], List[float]] = defaultdict(list)
    transition_counts: Dict[str, int] = defaultdict(int)
    capacity_before: Dict[str, List[float]] = defaultdict(list)
    capacity_after: Dict[str, List[float]] = defaultdict(list)
    for index in range(1, len(schedule)):
        time_s, after = schedule[index]
        before = schedule[index - 1][1]
        kind = transition_kind(before, after)
        transition_counts[kind] += 1
        capacity_before[kind].append(before)
        capacity_after[kind].append(after)
        for phase, start_offset, end_offset in phases:
            left = bisect.bisect_left(queue_times, time_s + start_offset)
            right = bisect.bisect_left(queue_times, time_s + end_offset)
            grouped[(kind, phase)].extend(queue_delays[left:right])
    result: List[Dict[str, object]] = []
    for kind in ("capacity_down", "capacity_stable", "capacity_up"):
        for phase, _, _ in phases:
            values = grouped[(kind, phase)]
            result.append(
                {
                    "scenario": scenario,
                    "algorithm": algorithm,
                    "window_class": kind,
                    "relative_phase": phase,
                    "transition_count": transition_counts[kind],
                    "queue_sample_count": len(values),
                    "mean_capacity_before_bps": common.mean(
                        capacity_before[kind]
                    ),
                    "mean_capacity_after_bps": common.mean(
                        capacity_after[kind]
                    ),
                    "mean_queue_delay_ms": common.mean(values),
                    "p95_queue_delay_ms": common.percentile(values, 0.95),
                    "max_queue_delay_ms": max(values, default=math.nan),
                }
            )
    return result


def synthetic_dynamic_summary(root: Path) -> List[Dict[str, object]]:
    scenarios = (
        "dynamic_1flow_100_40_100M_60s",
        "dynamic_4flows_100_40_100M_60s",
    )
    phases = (
        ("initial_100M_steady", 5.0, 20.0),
        ("40M", 20.0, 40.0),
        ("100M_recovery_early", 40.0, 45.0),
        ("100M_recovered", 45.0, 60.0),
    )
    result: List[Dict[str, object]] = []
    for scenario in scenarios:
        compare = root / scenario / "compare"
        throughput = common.read_csv(
            compare / "timeseries_throughput_aggregate_mbps.csv"
        )
        queue = common.read_csv(
            compare / "timeseries_queue_delay_mean_ms.csv"
        )
        for algorithm in ("FBBR-hybridv3", "FBBR"):
            for phase, start_s, end_s in phases:
                throughput_values = [
                    f(row.get(algorithm))
                    for row in throughput
                    if start_s <= f(row.get("time_s")) < end_s
                ]
                queue_values = [
                    f(row.get(algorithm))
                    for row in queue
                    if start_s <= f(row.get("time_s")) < end_s
                ]
                result.append(
                    {
                        "scenario": scenario,
                        "algorithm": algorithm,
                        "phase": phase,
                        "start_s": start_s,
                        "end_s": end_s,
                        "mean_throughput_mbps": common.mean(
                            throughput_values
                        ),
                        "p95_throughput_mbps": common.percentile(
                            throughput_values, 0.95
                        ),
                        "mean_queue_delay_ms": common.mean(queue_values),
                        "p95_queue_delay_ms": common.percentile(
                            queue_values, 0.95
                        ),
                        "max_queue_delay_ms": max(
                            queue_values, default=math.nan
                        ),
                    }
                )
    return result


def pearson(rows: Sequence[Dict[str, object]], lhs: str, rhs: str) -> float:
    pairs = [
        (f(row.get(lhs)), f(row.get(rhs)))
        for row in rows
        if math.isfinite(f(row.get(lhs))) and math.isfinite(f(row.get(rhs)))
    ]
    if len(pairs) < 2:
        return math.nan
    mean_lhs = common.mean(pair[0] for pair in pairs)
    mean_rhs = common.mean(pair[1] for pair in pairs)
    numerator = sum(
        (left - mean_lhs) * (right - mean_rhs)
        for left, right in pairs
    )
    lhs_energy = sum((left - mean_lhs) ** 2 for left, _ in pairs)
    rhs_energy = sum((right - mean_rhs) ** 2 for _, right in pairs)
    denominator = math.sqrt(lhs_energy * rhs_energy)
    return numerator / denominator if denominator > 0.0 else math.nan


def frequency_service_split(
    run_dir: Path, scenario: str
) -> List[Dict[str, object]]:
    grouped: Dict[str, List[Dict[str, float]]] = defaultdict(list)
    for waveform_path in sorted(
        run_dir.glob("flow*_cruise_waveform_search.csv"),
        key=common.flow_id,
    ):
        fid = common.flow_id(waveform_path)
        gate_path = run_dir / f"flow{fid}_freq_gate_trace.csv"
        pacing = common.load_gate_pacing(gate_path) if gate_path.exists() else []
        times = [float(sample["time"]) for sample in pacing]
        for row in common.read_csv(waveform_path):
            start_s = f(row.get("analysis_cycle_start_s"))
            end_s = f(row.get("analysis_cycle_end_s"))
            if not math.isfinite(start_s) or not math.isfinite(end_s):
                continue
            left = bisect.bisect_left(times, start_s)
            right = bisect.bisect_right(times, end_s)
            selected = pacing[left:right]
            target = common.harmonic_amplitude(selected, "target")
            actual = common.harmonic_amplitude(selected, "actual")
            key = (
                "service_limited"
                if f(row.get("v4_service_restriction"), 0.0) > 0.0
                else "not_service_limited"
            )
            grouped[key].append(
                {
                    "target": target,
                    "actual": actual,
                    "ratio": actual / target if target > 0.0 else math.nan,
                    "drate_score": max(
                        0.0,
                        1.0 - f(
                            row.get("drate_srate_period_error_ratio"),
                            math.inf,
                        ),
                    ),
                    "srtt_score": max(
                        0.0,
                        1.0 - f(
                            row.get("srtt_srate_period_error_ratio"),
                            math.inf,
                        ),
                    ),
                }
            )
    result: List[Dict[str, object]] = []
    for key in ("not_service_limited", "service_limited"):
        rows = grouped[key]
        result.append(
            {
                "scenario": scenario,
                "window_group": key,
                "window_count": len(rows),
                "target_amplitude_p50_bps": common.percentile(
                    (row["target"] for row in rows), 0.5
                ),
                "actual_amplitude_p50_bps": common.percentile(
                    (row["actual"] for row in rows), 0.5
                ),
                "actual_target_amplitude_ratio_p50": common.percentile(
                    (row["ratio"] for row in rows), 0.5
                ),
                "drate_frequency_score_p50": common.percentile(
                    (row["drate_score"] for row in rows), 0.5
                ),
                "srtt_frequency_score_p50": common.percentile(
                    (row["srtt_score"] for row in rows), 0.5
                ),
            }
        )
    return result


def acceptance(
    scenario: str,
    metrics: Dict[Tuple[str, str], Dict[str, str]],
) -> Dict[str, object]:
    current = metrics[(scenario, "FBBR")]
    v3 = metrics[(scenario, "FBBR-hybridv3")]
    throughput = f(current["avg_aggregate_throughput_mbps_after_warmup"])
    v3_throughput = f(v3["avg_aggregate_throughput_mbps_after_warmup"])
    queue_mean = f(current["avg_queue_delay_ms_after_warmup"])
    queue_p95 = f(current["p95_queue_delay_ms_after_warmup"])
    jain = f(current["avg_jain_fairness_after_warmup"])
    loss = f(current["aggregate_loss_rate_pct_whole_run"])
    if scenario == "fixed_4":
        limits = (0.995 * v3_throughput, 13.269, 13.363 * 1.05, 0.985)
    elif scenario == "fixed_32":
        limits = (0.995 * v3_throughput, 20.0, 20.0, 0.956)
    elif scenario == "cell_4":
        limits = (0.985 * v3_throughput, 73.393, 416.988, 0.828943)
    else:
        limits = (0.985 * v3_throughput, 154.985, 1007.408, 0.798307)
    checks = {
        "throughput_pass": throughput >= limits[0],
        "queue_mean_pass": queue_mean <= limits[1],
        "queue_p95_pass": queue_p95 <= limits[2],
        "jain_pass": jain >= limits[3],
        "loss_pass": loss == 0.0,
    }
    return {
        "scenario": scenario,
        "throughput_mbps": throughput,
        "v3_throughput_mbps": v3_throughput,
        "throughput_min": limits[0],
        "queue_mean_ms": queue_mean,
        "queue_mean_max": limits[1],
        "queue_p95_ms": queue_p95,
        "queue_p95_max": limits[2],
        "jain": jain,
        "jain_min": limits[3],
        "loss_pct": loss,
        **checks,
        "scenario_pass": all(checks.values()),
    }


def return_code(run_dir: Path) -> int:
    path = run_dir / "return_code.txt"
    return int(path.read_text().strip()) if path.exists() else -999


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--result-root",
        type=Path,
        default=Path(
            "results/fbbr_service_consistent_inflight_envelope_v4"
        ),
    )
    parser.add_argument(
        "--v3-result-root",
        type=Path,
        default=Path(
            "results/fbbr_model_consistent_inflight_projection_v3"
        ),
    )
    args = parser.parse_args()
    root = args.result_root.resolve()
    v3_root = args.v3_result_root.resolve()
    root.mkdir(parents=True, exist_ok=True)

    metric_rows: List[Dict[str, object]] = []
    metrics: Dict[Tuple[str, str], Dict[str, str]] = {}
    return_codes: Dict[str, int] = {}
    v4_rows: List[Dict[str, object]] = []
    frequency_rows: List[Dict[str, object]] = []
    frequency_split_rows: List[Dict[str, object]] = []
    transition_rows: List[Dict[str, object]] = []
    queue_transition_rows: List[Dict[str, object]] = []

    for scenario, spec in SCENARIOS.items():
        scenario_dir = root / str(spec["path"])
        current = load_metrics(
            scenario_dir / "compare" / "summary_metrics.csv"
        )
        reused_baselines = load_metrics(
            root.parent / BASELINE_SUMMARIES[scenario]
        )
        for algorithm in ALGORITHMS:
            row = current[algorithm]
            metrics[(scenario, algorithm)] = row
            output: Dict[str, object] = {"scenario": scenario}
            output.update(row)
            output["result_provenance"] = "formal_v4_comparison"
            metric_rows.append(output)
            return_codes[f"{scenario}:{algorithm}"] = return_code(
                scenario_dir / algorithm
            )
        for algorithm in BASELINES:
            row = reused_baselines[algorithm]
            output = {"scenario": scenario}
            output.update(row)
            output["result_provenance"] = "reused_v3_same_config_rc0"
            metric_rows.append(output)
            return_codes[f"{scenario}:{algorithm}"] = return_code(
                Path(row["source_run_dir"])
            )

        v4_dir = scenario_dir / "FBBR"
        v4_rows.append(summarize_v4(v4_dir, scenario))
        for algorithm in ALGORITHMS:
            frequency, _ = common.summarize_frequency_run(
                scenario_dir / algorithm, algorithm
            )
            frequency["scenario"] = scenario
            if algorithm == "FBBR":
                references: List[float] = []
                for gate in v4_dir.glob("flow*_freq_gate_trace.csv"):
                    references.extend(
                        f(sample["base"])
                        for sample in common.load_gate_pacing(gate)
                        if str(sample["base_source"]).startswith("V4_")
                        and f(sample["base"]) > 0.0
                    )
                frequency["v4_reference_sample_mean_bps"] = common.mean(
                    references
                )
                frequency["v4_reference_sample_p50_bps"] = common.percentile(
                    references, 0.5
                )
            frequency_rows.append(frequency)
        frequency_split_rows.extend(
            frequency_service_split(v4_dir, scenario)
        )
        if not bool(spec["fixed"]):
            transition_rows.extend(
                transition_aligned_rows(v4_dir, scenario)
            )
            for algorithm in ALGORITHMS:
                queue_transition_rows.extend(
                    queue_transition_summary(
                        scenario_dir / algorithm, scenario, algorithm
                    )
                )

    fixed4_rows: List[Dict[str, object]] = []
    fixed4_dir = (
        root
        / str(SCENARIOS["fixed_4"]["path"])
        / "FBBR"
    )
    global_jain = f(
        metrics[("fixed_4", "FBBR")][
            "avg_jain_fairness_after_warmup"
        ]
    )
    for path in sorted(
        fixed4_dir.glob(
            "FBBR_flow*_FBBR_good.txt"
        ),
        key=common.flow_id,
    ):
        fid = common.flow_id(path)
        gate = fixed4_dir / f"flow{fid}_freq_gate_trace.csv"
        references = [
            f(sample["base"])
            for sample in common.load_gate_pacing(gate)
            if str(sample["base_source"]).startswith("V4_")
            and f(sample["base"]) > 0.0
        ]
        fixed4_rows.append(
            {
                "flow_id": fid,
                "throughput_mbps_after_warmup": (
                    common.read_goodput_mean_mbps(path)
                ),
                "reference_bw_mean_mbps": common.mean(references) / 1e6,
                "reference_bw_p50_mbps": (
                    common.percentile(references, 0.5) / 1e6
                ),
                "global_jain": global_jain,
            }
        )

    fixed32_rows: List[Dict[str, object]] = []
    fixed32_dir = (
        root
        / str(SCENARIOS["fixed_32"]["path"])
        / "FBBR"
    )
    for summary_path in sorted(
        fixed32_dir.glob("flow*_v4_service_envelope_summary.csv"),
        key=common.flow_id,
    ):
        fid = common.flow_id(summary_path)
        summary_row = common.read_csv(summary_path)[0]
        goodput_path = next(
            fixed32_dir.glob(
                f"FBBR_flow{fid}_FBBR_good.txt"
            )
        )
        fixed32_rows.append(
            {
                "flow_id": fid,
                "throughput_mbps_after_warmup": (
                    common.read_goodput_mean_mbps(goodput_path)
                ),
                "service_limited_time_ratio": f(
                    summary_row["service_limited_time_ratio"]
                ),
                "mean_service_inflight": f(
                    summary_row["mean_service_inflight"]
                ),
                "mean_probe_credit": f(
                    summary_row["mean_probe_credit"]
                ),
                "mean_extra_acked": f(
                    summary_row["mean_extra_acked"]
                ),
                "mean_service_restriction": f(
                    summary_row["mean_service_restriction"]
                ),
            }
        )
    fixed32_correlations = [
        {
            "dependent": "throughput_mbps_after_warmup",
            "independent": field,
            "pearson_correlation": pearson(
                fixed32_rows,
                "throughput_mbps_after_warmup",
                field,
            ),
        }
        for field in (
            "service_limited_time_ratio",
            "mean_service_inflight",
            "mean_probe_credit",
            "mean_extra_acked",
            "mean_service_restriction",
        )
    ]

    acceptance_rows = [
        acceptance(scenario, metrics) for scenario in SCENARIOS
    ]
    transition_summary = aggregate_transition_classes(transition_rows)
    dynamic_rows = synthetic_dynamic_summary(root)
    write_csv(root / "formal_metrics.csv", metric_rows)
    write_csv(root / "v4_envelope_summary.csv", v4_rows)
    write_csv(root / "frequency_integrity.csv", frequency_rows)
    write_csv(
        root / "frequency_service_limited_split.csv",
        frequency_split_rows,
    )
    write_csv(root / "fixed4_per_flow.csv", fixed4_rows)
    write_csv(root / "fixed32_per_flow.csv", fixed32_rows)
    write_csv(
        root / "fixed32_share_correlations.csv",
        fixed32_correlations,
    )
    write_csv(root / "capacity_transition_aligned.csv", transition_rows)
    write_csv(root / "capacity_transition_summary.csv", transition_summary)
    write_csv(
        root / "capacity_transition_queue_summary.csv",
        queue_transition_rows,
    )
    write_csv(root / "synthetic_dynamic_summary.csv", dynamic_rows)
    write_csv(root / "acceptance.csv", acceptance_rows)
    summary = {
        "return_codes": return_codes,
        "all_required_return_codes_zero": all(
            code == 0 for code in return_codes.values()
        ),
        "acceptance_all_pass": all(
            bool(row["scenario_pass"]) for row in acceptance_rows
        ),
        "acceptance": acceptance_rows,
        "v4_envelope": v4_rows,
        "frequency": frequency_rows,
        "frequency_service_split": frequency_split_rows,
        "fixed32_share_correlations": fixed32_correlations,
        "capacity_transition_summary": transition_summary,
        "capacity_transition_queue_summary": queue_transition_rows,
        "synthetic_dynamic_summary": dynamic_rows,
    }
    (root / "analysis_summary.json").write_text(
        json.dumps(summary, indent=2, allow_nan=True) + "\n",
        encoding="utf-8",
    )
    print(json.dumps(summary, indent=2, allow_nan=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
