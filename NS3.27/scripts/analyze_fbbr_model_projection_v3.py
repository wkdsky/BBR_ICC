#!/usr/bin/env python3
"""Analyze FBBR-hybridv3 model-consistent inflight experiments."""

from __future__ import annotations

import argparse
import bisect
import csv
import json
import math
import re
from collections import Counter
from pathlib import Path
from typing import Dict, Iterable, List, Sequence, Tuple


SCENARIOS = {
    "fixed_4": {
        "path": "fixed_4flows_100M_5BDP_240s",
        "flows": 4,
        "duration": 240.0,
        "fixed": True,
    },
    "fixed_32": {
        "path": "fixed_32flows_100M_5BDP_240s",
        "flows": 32,
        "duration": 240.0,
        "fixed": True,
    },
    "cell_4": {
        "path": "cellular_taxi_4flows_128M_5BDP_180s",
        "flows": 4,
        "duration": 180.0,
        "fixed": False,
    },
    "cell_8": {
        "path": "cellular_taxi_8flows_128M_5BDP_180s",
        "flows": 8,
        "duration": 180.0,
        "fixed": False,
    },
}

REUSED_METRICS = {
    "fixed_4": (
        "fbbr_hybrid_vs_bbrr_4flows_shared_100M_5BDP_240s_20260722/"
        "compare/summary_metrics.csv"
    ),
    "fixed_32": (
        "fbbr_hybrid_vs_bbrr_vs_bbrv2_32flows_shared_100M_5BDP_240s_"
        "20260722/compare/summary_metrics.csv"
    ),
    "cell_4": (
        "cellular_links_taxi_128M_4flow_8flow_5BDP_180s_3cc_20260722/"
        "cellular_taxi_128M_4flows_5BDP_180s/compare/summary_metrics.csv"
    ),
    "cell_8": (
        "cellular_links_taxi_128M_4flow_8flow_5BDP_180s_3cc_20260722/"
        "cellular_taxi_128M_8flows_5BDP_180s/compare/summary_metrics.csv"
    ),
}

ACCEPTANCE = {
    "fixed_4": {
        "throughput_min": 95.0,
        "queue_mean_max": 7.246,
        "queue_p95_max": 27.711,
        "jain_min": 0.916,
    },
    "fixed_32": {
        "throughput_min": 97.0,
        "queue_mean_max": 20.629,
        "queue_p95_max": 34.128,
        "jain_min": 0.906,
    },
    "cell_4": {
        "throughput_min": 0.98 * 40.008032,
        "queue_mean_max": 129.357,
        "queue_p95_max": 760.317,
        "jain_min": 0.847437 - 0.02,
    },
    "cell_8": {
        "throughput_min": 0.98 * 42.055086,
        "queue_mean_max": 128.422,
        "queue_p95_max": 730.911,
        "jain_min": 0.863960 - 0.02,
    },
}


def finite_float(value: object, default: float = math.nan) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError):
        return default
    return result if math.isfinite(result) else default


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


def mean(values: Iterable[float]) -> float:
    finite = [value for value in values if math.isfinite(value)]
    return sum(finite) / len(finite) if finite else math.nan


def flow_id(path: Path) -> int:
    match = re.search(r"(?:^|_)flow(\d+)(?:_|$)", path.name)
    if not match:
        raise ValueError(f"cannot extract flow id from {path}")
    return int(match.group(1))


def read_csv(path: Path) -> List[Dict[str, str]]:
    with path.open("r", encoding="utf-8", errors="replace") as handle:
        return list(csv.DictReader(handle))


def write_csv(path: Path, rows: Sequence[Dict[str, object]]) -> None:
    if not rows:
        return
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def read_metrics(path: Path) -> Dict[str, Dict[str, str]]:
    return {row["cc"]: row for row in read_csv(path)}


def read_goodput_mean_mbps(path: Path, warmup_s: float = 5.0) -> float:
    values: List[float] = []
    with path.open("r", encoding="utf-8", errors="replace") as handle:
        for line in handle:
            if not line.strip() or line.startswith("#"):
                continue
            fields = line.split()
            if len(fields) < 2:
                continue
            time_s = finite_float(fields[0])
            kbps = finite_float(fields[1])
            if time_s >= warmup_s and math.isfinite(kbps):
                values.append(kbps)
    return mean(values) / 1000.0


def load_gate_pacing(path: Path) -> List[Dict[str, object]]:
    # PacingRate can be queried repeatedly at one timestamp.  Keep the last
    # value so harmonic fits weight simulation time, not query count.
    by_time: Dict[float, Dict[str, object]] = {}
    with path.open("r", encoding="utf-8", errors="replace") as handle:
        for row in csv.DictReader(handle):
            if row.get("row_type") != "pacing":
                continue
            time_s = finite_float(row.get("time"))
            if not math.isfinite(time_s):
                continue
            by_time[time_s] = {
                "time": time_s,
                "target": finite_float(row.get("final_pacing_rate_bps")),
                # Existing delivery-rate telemetry is the closest available
                # no-new-log proxy for cwnd-limited actual paced output.
                "actual": finite_float(row.get("current_delivery_rate")),
                "base": finite_float(row.get("pacing_base_bw_bps")),
                "amplitude": finite_float(row.get("amplitude_bps_eff")),
                "base_source": row.get("pacing_base_source", ""),
                "trusted_valid": row.get("trusted_bw_valid") == "true",
                "trusted_source": row.get("trusted_bw_source", ""),
            }
    return [by_time[key] for key in sorted(by_time)]


def harmonic_amplitude(
    samples: Sequence[Dict[str, object]],
    key: str,
    frequency_hz: float = 5.0,
) -> float:
    points = [
        (float(sample["time"]), finite_float(sample.get(key)))
        for sample in samples
        if math.isfinite(finite_float(sample.get(key)))
    ]
    if len(points) < 8:
        return math.nan
    center = mean(value for _, value in points)
    cos_sum = 0.0
    sin_sum = 0.0
    for time_s, value in points:
        phase = 2.0 * math.pi * frequency_hz * time_s
        centered = value - center
        cos_sum += centered * math.cos(phase)
        sin_sum += centered * math.sin(phase)
    return 2.0 * math.hypot(cos_sum, sin_sum) / len(points)


def summarize_frequency_run(
    run_dir: Path,
    algorithm: str,
) -> Tuple[Dict[str, object], List[Dict[str, object]]]:
    waveform_files = sorted(
        run_dir.glob("flow*_cruise_waveform_search.csv"), key=flow_id
    )
    all_rows: List[Dict[str, str]] = []
    target_amplitudes: List[float] = []
    actual_amplitudes: List[float] = []
    amplitude_ratios: List[float] = []
    target_construction_ratios: List[float] = []
    split_rows: List[Dict[str, object]] = []
    trusted_update_times: List[float] = []
    reference_samples: List[float] = []

    for waveform_path in waveform_files:
        fid = flow_id(waveform_path)
        waveform_rows = read_csv(waveform_path)
        all_rows.extend(waveform_rows)
        gate_path = run_dir / f"flow{fid}_freq_gate_trace.csv"
        if not gate_path.exists():
            continue
        pacing = load_gate_pacing(gate_path)
        times = [float(sample["time"]) for sample in pacing]
        for sample in pacing:
            if (
                sample["trusted_valid"]
                and sample["trusted_source"]
                not in {"NONE", "NATIVE_BW_FALLBACK"}
            ):
                trusted_update_times.append(float(sample["time"]))
                break
        reference_samples.extend(
            finite_float(sample["base"])
            for sample in pacing
            if str(sample["base_source"]).startswith("V3_")
            and finite_float(sample["base"]) > 0.0
        )

        for window in waveform_rows:
            start_s = finite_float(window.get("analysis_cycle_start_s"))
            end_s = finite_float(window.get("analysis_cycle_end_s"))
            if not math.isfinite(start_s) or not math.isfinite(end_s):
                continue
            left = bisect.bisect_left(times, start_s)
            right = bisect.bisect_right(times, end_s)
            selected = pacing[left:right]
            target = harmonic_amplitude(selected, "target")
            actual = harmonic_amplitude(selected, "actual")
            ratio = actual / target if target > 0.0 else math.nan
            commanded_amplitude = percentile(
                (
                    finite_float(sample.get("amplitude"))
                    for sample in selected
                    if finite_float(sample.get("amplitude")) > 0.0
                ),
                0.5,
            )
            construction_ratio = (
                target / commanded_amplitude
                if commanded_amplitude > 0.0 else math.nan
            )
            if math.isfinite(target):
                target_amplitudes.append(target)
            if math.isfinite(actual):
                actual_amplitudes.append(actual)
            if math.isfinite(ratio):
                amplitude_ratios.append(ratio)
            if math.isfinite(construction_ratio):
                target_construction_ratios.append(construction_ratio)
            if algorithm == "FBBR-hybridv3":
                binding = finite_float(
                    window.get("v3_cap_binding_fraction"), 0.0
                ) > 0.0
                split_rows.append(
                    {
                        "binding": binding,
                        "target_amplitude_bps": target,
                        "actual_amplitude_bps": actual,
                        "actual_target_amplitude_ratio": ratio,
                    }
                )

    valid_windows = sum(
        row.get("sender_waveform_valid") == "true"
        and row.get("drate_periodic_input_valid") == "true"
        and row.get("srtt_periodic_input_valid") == "true"
        for row in all_rows
    )
    regimes = Counter(row.get("unsuppressed_regime", "") for row in all_rows)
    # The current V1 detector mode is time_waveform, where the legacy
    # Goertzel integrity columns remain zero.  Use the detector's existing
    # sender/response period error as a bounded frequency-match score.
    drate_scores = [
        max(
            0.0,
            1.0 - finite_float(
                row.get("drate_srate_period_error_ratio")
            ),
        )
        for row in all_rows
        if math.isfinite(
            finite_float(row.get("drate_srate_period_error_ratio"))
        )
    ]
    srtt_scores = [
        max(
            0.0,
            1.0 - finite_float(
                row.get("srtt_srate_period_error_ratio")
            ),
        )
        for row in all_rows
        if math.isfinite(
            finite_float(row.get("srtt_srate_period_error_ratio"))
        )
    ]
    summary: Dict[str, object] = {
        "algorithm": algorithm,
        "flow_count": len(waveform_files),
        "analysis_window_count": len(all_rows),
        "valid_frequency_window_count": valid_windows,
        "regime_i_underload_count": regimes["UNDERLOAD"],
        "regime_ii_full_load_count": regimes["FULL_LOAD"],
        "regime_iii_overload_count": regimes["OVERLOAD"],
        "inconclusive_count": regimes["INCONCLUSIVE"],
        "trusted_first_update_min_s": min(trusted_update_times, default=math.nan),
        "trusted_first_update_median_s": percentile(
            trusted_update_times, 0.5
        ),
        "drate_frequency_score_p50": percentile(drate_scores, 0.5),
        "drate_frequency_score_p95": percentile(drate_scores, 0.95),
        "srtt_frequency_score_p50": percentile(srtt_scores, 0.5),
        "srtt_frequency_score_p95": percentile(srtt_scores, 0.95),
        "pacing_target_main_frequency_amplitude_p50_bps": percentile(
            target_amplitudes, 0.5
        ),
        "actual_delivery_main_frequency_amplitude_p50_bps": percentile(
            actual_amplitudes, 0.5
        ),
        "actual_target_amplitude_ratio_p50": percentile(
            amplitude_ratios, 0.5
        ),
        "target_triangle_fundamental_to_commanded_amplitude_p50": percentile(
            target_construction_ratios, 0.5
        ),
        "v3_reference_sample_mean_bps": mean(reference_samples),
        "v3_reference_sample_p50_bps": percentile(reference_samples, 0.5),
    }
    return summary, split_rows


def aggregate_binding_split(
    scenario: str,
    rows: Sequence[Dict[str, object]],
) -> List[Dict[str, object]]:
    result: List[Dict[str, object]] = []
    for binding in (False, True):
        selected = [row for row in rows if row["binding"] == binding]
        result.append(
            {
                "scenario": scenario,
                "window_group": "binding" if binding else "non_binding",
                "window_count": len(selected),
                "target_amplitude_p50_bps": percentile(
                    (
                        finite_float(row["target_amplitude_bps"])
                        for row in selected
                    ),
                    0.5,
                ),
                "actual_amplitude_p50_bps": percentile(
                    (
                        finite_float(row["actual_amplitude_bps"])
                        for row in selected
                    ),
                    0.5,
                ),
                "actual_target_amplitude_ratio_p50": percentile(
                    (
                        finite_float(row["actual_target_amplitude_ratio"])
                        for row in selected
                    ),
                    0.5,
                ),
            }
        )
    return result


def summarize_projection(run_dir: Path, scenario: str) -> Dict[str, object]:
    rows: List[Dict[str, str]] = []
    for path in sorted(run_dir.glob("flow*_v3_projection_summary.csv"), key=flow_id):
        rows.extend(read_csv(path))
    numeric_fields = [
        "reference_trusted_time_ratio",
        "reference_guard_time_ratio",
        "reference_last_valid_time_ratio",
        "reference_invalid_time_ratio",
        "projection_active_time_ratio",
        "history_invalid_time_ratio",
        "cap_binding_time_ratio",
        "mean_model_inflight",
        "mean_raw_queue_debt",
        "p95_raw_queue_debt",
        "mean_enforced_excess",
        "p95_enforced_excess",
    ]
    result: Dict[str, object] = {
        "scenario": scenario,
        "flow_count": len(rows),
    }
    for field in numeric_fields:
        result[field] = mean(finite_float(row.get(field)) for row in rows)
    return result


def cellular_window_response(
    run_dir: Path,
    scenario: str,
    flow_count: int,
) -> Dict[str, object]:
    config = json.loads((run_dir / "config.json").read_text())
    schedule: List[Tuple[float, float]] = []
    for item in config.get("capacity_schedule", "").split(","):
        if not item or ":" not in item:
            continue
        time_text, rate_text = item.split(":", 1)
        multiplier = 1e6 if rate_text.endswith("Mbps") else 1.0
        rate_text = rate_text.removesuffix("Mbps")
        schedule.append((float(time_text), float(rate_text) * multiplier))
    schedule.sort()
    schedule_times = [item[0] for item in schedule]

    reference_to_fair_share: List[float] = []
    binding_fractions: List[float] = []
    for path in run_dir.glob("flow*_cruise_waveform_search.csv"):
        for row in read_csv(path):
            time_s = finite_float(row.get("time_s"))
            reference = finite_float(row.get("v3_reference_bw"))
            if not math.isfinite(time_s) or reference <= 0.0 or not schedule:
                continue
            index = max(0, bisect.bisect_right(schedule_times, time_s) - 1)
            fair_share = schedule[index][1] / flow_count
            if fair_share > 0.0:
                reference_to_fair_share.append(reference / fair_share)
            binding_fractions.append(
                finite_float(row.get("v3_cap_binding_fraction"), 0.0)
            )
    return {
        "scenario": scenario,
        "window_reference_sample_count": len(reference_to_fair_share),
        "reference_to_instant_fair_capacity_p50": percentile(
            reference_to_fair_share, 0.5
        ),
        "reference_to_instant_fair_capacity_p95": percentile(
            reference_to_fair_share, 0.95
        ),
        "window_cap_binding_fraction_mean": mean(binding_fractions),
    }


def return_code(run_dir: Path) -> int:
    path = run_dir / "return_code.txt"
    return int(path.read_text().strip()) if path.exists() else -999


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--result-root",
        type=Path,
        default=Path("results/fbbr_model_consistent_inflight_projection_v3"),
    )
    parser.add_argument("--all-results-root", type=Path, default=Path("results"))
    args = parser.parse_args()
    root = args.result_root.resolve()
    all_results = args.all_results_root.resolve()
    root.mkdir(parents=True, exist_ok=True)

    metric_rows: List[Dict[str, object]] = []
    metric_lookup: Dict[Tuple[str, str], Dict[str, str]] = {}
    run_codes: Dict[str, int] = {}
    projection_rows: List[Dict[str, object]] = []
    frequency_rows: List[Dict[str, object]] = []
    binding_rows: List[Dict[str, object]] = []
    cellular_rows: List[Dict[str, object]] = []

    for scenario, spec in SCENARIOS.items():
        scenario_dir = root / str(spec["path"])
        current_metrics = read_metrics(
            scenario_dir / "compare" / "summary_metrics.csv"
        )
        reused_metrics = read_metrics(
            all_results / REUSED_METRICS[scenario]
        )
        for algorithm in ("FBBR-hybrid", "FBBR-hybridv3"):
            metric = current_metrics[algorithm]
            metric_lookup[(scenario, algorithm)] = metric
            row: Dict[str, object] = {"scenario": scenario}
            row.update(metric)
            row["result_provenance"] = "formal_v3_run"
            metric_rows.append(row)
            code = return_code(scenario_dir / algorithm)
            run_codes[f"{scenario}:{algorithm}"] = code
        for algorithm in ("BBR-R", "BBRv2"):
            metric = reused_metrics[algorithm]
            row = {"scenario": scenario}
            row.update(metric)
            row["result_provenance"] = "reused_same_config_return_code_0"
            metric_rows.append(row)
            code = return_code(Path(metric["source_run_dir"]))
            run_codes[f"{scenario}:{algorithm}"] = code

        v3_dir = scenario_dir / "FBBR-hybridv3"
        projection_rows.append(summarize_projection(v3_dir, scenario))
        if not bool(spec["fixed"]):
            cellular_rows.append(
                cellular_window_response(
                    v3_dir, scenario, int(spec["flows"])
                )
            )

        if bool(spec["fixed"]):
            for algorithm in ("FBBR-hybrid", "FBBR-hybridv3"):
                frequency, split = summarize_frequency_run(
                    scenario_dir / algorithm, algorithm
                )
                frequency["scenario"] = scenario
                frequency_rows.append(frequency)
                if algorithm == "FBBR-hybridv3":
                    binding_rows.extend(
                        aggregate_binding_split(scenario, split)
                    )

    fixed4_flow_rows: List[Dict[str, object]] = []
    fixed4_dir = root / str(SCENARIOS["fixed_4"]["path"])
    global_jain = finite_float(
        metric_lookup[("fixed_4", "FBBR-hybridv3")][
            "avg_jain_fairness_after_warmup"
        ]
    )
    for goodput_path in sorted(
        (fixed4_dir / "FBBR-hybridv3").glob(
            "FBBR-hybridv3_flow*_FBBR-hybridv3_good.txt"
        ),
        key=flow_id,
    ):
        fid = flow_id(goodput_path)
        gate_path = (
            fixed4_dir
            / "FBBR-hybridv3"
            / f"flow{fid}_freq_gate_trace.csv"
        )
        pacing = load_gate_pacing(gate_path)
        references = [
            finite_float(sample["base"])
            for sample in pacing
            if str(sample["base_source"]).startswith("V3_")
            and finite_float(sample["base"]) > 0.0
        ]
        projection_path = (
            fixed4_dir
            / "FBBR-hybridv3"
            / f"flow{fid}_v3_projection_summary.csv"
        )
        projection = read_csv(projection_path)[0]
        fixed4_flow_rows.append(
            {
                "flow_id": fid,
                "throughput_mbps_after_warmup": read_goodput_mean_mbps(
                    goodput_path
                ),
                "reference_bw_mean_mbps": mean(references) / 1e6,
                "reference_bw_p50_mbps": percentile(references, 0.5) / 1e6,
                "reference_trusted_time_ratio": projection[
                    "reference_trusted_time_ratio"
                ],
                "reference_guard_time_ratio": projection[
                    "reference_guard_time_ratio"
                ],
                "global_jain": global_jain,
            }
        )

    acceptance_rows: List[Dict[str, object]] = []
    for scenario, limits in ACCEPTANCE.items():
        metric = metric_lookup[(scenario, "FBBR-hybridv3")]
        throughput = finite_float(
            metric["avg_aggregate_throughput_mbps_after_warmup"]
        )
        queue_mean = finite_float(
            metric["avg_queue_delay_ms_after_warmup"]
        )
        queue_p95 = finite_float(
            metric["p95_queue_delay_ms_after_warmup"]
        )
        jain = finite_float(metric["avg_jain_fairness_after_warmup"])
        loss = finite_float(metric["aggregate_loss_rate_pct_whole_run"])
        checks = {
            "throughput_pass": throughput >= limits["throughput_min"],
            "queue_mean_pass": queue_mean <= limits["queue_mean_max"],
            "queue_p95_pass": queue_p95 <= limits["queue_p95_max"],
            "jain_pass": jain >= limits["jain_min"],
            "loss_pass": loss == 0.0,
        }
        acceptance_rows.append(
            {
                "scenario": scenario,
                "throughput_mbps": throughput,
                "throughput_min": limits["throughput_min"],
                "queue_mean_ms": queue_mean,
                "queue_mean_max": limits["queue_mean_max"],
                "queue_p95_ms": queue_p95,
                "queue_p95_max": limits["queue_p95_max"],
                "jain": jain,
                "jain_min": limits["jain_min"],
                "loss_pct": loss,
                **checks,
                "scenario_pass": all(checks.values()),
            }
        )

    write_csv(root / "formal_metrics.csv", metric_rows)
    write_csv(root / "v3_projection_summary.csv", projection_rows)
    write_csv(root / "frequency_integrity.csv", frequency_rows)
    write_csv(root / "frequency_binding_split.csv", binding_rows)
    write_csv(root / "fixed4_per_flow.csv", fixed4_flow_rows)
    write_csv(root / "cellular_response.csv", cellular_rows)
    write_csv(root / "acceptance.csv", acceptance_rows)

    summary = {
        "return_codes": run_codes,
        "all_required_return_codes_zero": all(
            code == 0 for code in run_codes.values()
        ),
        "acceptance_all_pass": all(
            bool(row["scenario_pass"]) for row in acceptance_rows
        ),
        "acceptance": acceptance_rows,
        "projection": projection_rows,
        "frequency": frequency_rows,
        "binding_split": binding_rows,
        "cellular_response": cellular_rows,
    }
    (root / "analysis_summary.json").write_text(
        json.dumps(summary, indent=2, allow_nan=True) + "\n",
        encoding="utf-8",
    )
    print(json.dumps(summary, indent=2, allow_nan=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
