#!/usr/bin/env python3
"""Analyze the fixed-population dynamic-propagation-RTT Test 3 experiment."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


ALGORITHMS = [
    "BBR-R",
    "oBBR",
    "BBRv2+",
    "CUBIC",
    "BBRv2-formal",
    "BBRv2",
    "FBBR",
]
FLOW_COLUMNS = [f"flow{index}_received_bytes" for index in range(1, 5)]
COLORS = {
    "BBR-R": "#0077BB",
    "oBBR": "#EE7733",
    "BBRv2+": "#009988",
    "CUBIC": "#999933",
    "BBRv2-formal": "#000000",
    "BBRv2": "#CC3311",
    "FBBR": "#EE3377",
}


def resolve_path(manifest_path: Path, value: object) -> Path:
    path = Path(str(value))
    if path.is_file():
        return path
    candidate = manifest_path.parents[3] / path
    if candidate.is_file():
        return candidate
    raise FileNotFoundError(f"Missing manifest output: {value}")


def require_columns(frame: pd.DataFrame, columns: list[str], name: str) -> None:
    missing = [column for column in columns if column not in frame.columns]
    if missing:
        raise ValueError(f"{name} is missing columns: {', '.join(missing)}")


def jain(values: np.ndarray) -> float:
    values = np.asarray(values, dtype=float)
    total = float(values.sum())
    squares = float(np.square(values).sum())
    if squares <= 0.0:
        return float("nan")
    return total * total / (len(values) * squares)


def percentile(values: pd.Series, fraction: float) -> float:
    values = pd.to_numeric(values, errors="coerce").dropna()
    return float(values.quantile(fraction)) if not values.empty else float("nan")


def format_metric(value: object, precision: int) -> str:
    if pd.isna(value):
        return "-"
    return f"{float(value):.{precision}f}"


def values_at(frame: pd.DataFrame, time_s: float, columns: list[str]) -> np.ndarray:
    times = frame.time_s.to_numpy(dtype=float)
    return np.array(
        [np.interp(time_s, times, frame[column].to_numpy(dtype=float)) for column in columns],
        dtype=float,
    )


def stage_window(profile: pd.DataFrame, stage_index: int, settle_guard_s: float) -> tuple[float, float, float, float]:
    stage = profile.iloc[stage_index]
    start_s = float(stage.stage_start_s)
    end_s = float(stage.stage_end_s)
    steady_start_s = min(end_s, start_s + settle_guard_s)
    return start_s, end_s, steady_start_s, max(0.0, end_s - steady_start_s)


def phase_metrics(
    frame: pd.DataFrame,
    profile: pd.DataFrame,
    manifest_row: pd.Series,
) -> list[dict[str, object]]:
    records: list[dict[str, object]] = []
    capacity_bps = float(manifest_row.capacity_bps)
    settle_guard_s = float(manifest_row.settle_guard_s)
    for stage_index in range(len(profile)):
        start_s, end_s, steady_start_s, duration_s = stage_window(
            profile, stage_index, settle_guard_s
        )
        window = frame[(frame.time_s >= steady_start_s) & (frame.time_s < end_s)].copy()
        begin = values_at(frame, steady_start_s, FLOW_COLUMNS)
        end = values_at(frame, end_s, FLOW_COLUMNS)
        flow_goodput_bps = (end - begin) * 8.0 / max(duration_s, 1e-9)
        aggregate_goodput_bps = float(flow_goodput_bps.sum())
        snapshot_window = window[window.snapshot_flow_count > 0]
        record = {
            "scenario_id": manifest_row.scenario_id,
            "algorithm": manifest_row.algorithm,
            "mode": manifest_row["mode"],
            "seed": int(manifest_row.seed),
            "run_id": int(manifest_row.run_id),
            "stage_index": stage_index,
            "stage_start_s": start_s,
            "stage_end_s": end_s,
            "steady_start_s": steady_start_s,
            "steady_duration_s": duration_s,
            "configured_base_rtt_ms": float(profile.iloc[stage_index].configured_base_rtt_ms),
            "expected_bdp_bytes": int(profile.iloc[stage_index].expected_bdp_bytes),
            "sample_count": len(window),
            "aggregate_goodput_bps": aggregate_goodput_bps,
            "utilization_pct": 100.0 * aggregate_goodput_bps / capacity_bps,
            "jain_fairness": jain(flow_goodput_bps),
            "min_flow_goodput_bps": float(flow_goodput_bps.min()),
            "max_flow_goodput_bps": float(flow_goodput_bps.max()),
            "mean_queue_delay_ms": float(window.queue_delay_ms.mean()),
            "p95_queue_delay_ms": percentile(window.queue_delay_ms, 0.95),
            "max_queue_delay_ms": float(window.queue_delay_ms.max()),
            "mean_inflight_bdp": float(
                (window.aggregate_inflight_bytes / window.expected_bdp_bytes).mean()
            ),
            "mean_srtt_ms": (
                float(snapshot_window.mean_srtt_us.mean()) / 1000.0
                if not snapshot_window.empty
                else float("nan")
            ),
            "mean_min_rtt_ms": (
                float(snapshot_window.mean_min_rtt_us.mean()) / 1000.0
                if not snapshot_window.empty
                else float("nan")
            ),
        }
        for index, value in enumerate(flow_goodput_bps, start=1):
            record[f"flow{index}_goodput_bps"] = float(value)
        records.append(record)
    return records


def transition_recovery_time(
    frame: pd.DataFrame, start_s: float, end_s: float, capacity_bps: float
) -> float:
    times = frame.time_s.to_numpy(dtype=float)
    total = frame[FLOW_COLUMNS].sum(axis=1).to_numpy(dtype=float)
    candidates = frame[(frame.time_s >= start_s + 5.0) & (frame.time_s < end_s)]
    for time_s in candidates.time_s.to_numpy(dtype=float):
        delivered = np.interp(time_s, times, total) - np.interp(time_s - 5.0, times, total)
        if delivered * 8.0 / 5.0 >= 0.90 * capacity_bps:
            return float(time_s - start_s)
    return float("nan")


def transition_metrics(
    frame: pd.DataFrame,
    profile: pd.DataFrame,
    manifest_row: pd.Series,
) -> list[dict[str, object]]:
    records: list[dict[str, object]] = []
    capacity_bps = float(manifest_row.capacity_bps)
    settle_guard_s = float(manifest_row.settle_guard_s)
    for stage_index in range(1, len(profile)):
        stage = profile.iloc[stage_index]
        start_s = float(stage.stage_start_s)
        end_s = min(float(stage.stage_end_s), start_s + settle_guard_s)
        duration_s = max(0.0, end_s - start_s)
        window = frame[(frame.time_s >= start_s) & (frame.time_s < end_s)].copy()
        begin = values_at(frame, start_s, FLOW_COLUMNS)
        end = values_at(frame, end_s, FLOW_COLUMNS)
        aggregate_goodput_bps = float(((end - begin) * 8.0 / max(duration_s, 1e-9)).sum())
        records.append(
            {
                "scenario_id": manifest_row.scenario_id,
                "algorithm": manifest_row.algorithm,
                "mode": manifest_row["mode"],
                "seed": int(manifest_row.seed),
                "run_id": int(manifest_row.run_id),
                "transition_index": stage_index,
                "transition_start_s": start_s,
                "from_base_rtt_ms": float(profile.iloc[stage_index - 1].configured_base_rtt_ms),
                "to_base_rtt_ms": float(stage.configured_base_rtt_ms),
                "window_end_s": end_s,
                "window_duration_s": duration_s,
                "aggregate_goodput_bps": aggregate_goodput_bps,
                "utilization_pct": 100.0 * aggregate_goodput_bps / capacity_bps,
                "p95_queue_delay_ms": percentile(window.queue_delay_ms, 0.95),
                "max_queue_delay_ms": float(window.queue_delay_ms.max()),
                "recovery_to_90pct_s": transition_recovery_time(
                    frame, start_s, float(stage.stage_end_s), capacity_bps
                ),
            }
        )
    return records


def rolling_metrics(frame: pd.DataFrame, window_s: float) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    times = frame.time_s.to_numpy(dtype=float)
    received = frame[FLOW_COLUMNS].to_numpy(dtype=float)
    total = received.sum(axis=1)
    goodput = np.full(len(frame), np.nan)
    fairness = np.full(len(frame), np.nan)
    for index, time_s in enumerate(times):
        if time_s < times[0] + window_s:
            continue
        before = np.array(
            [np.interp(time_s - window_s, times, received[:, flow]) for flow in range(received.shape[1])]
        )
        rates = (received[index] - before) * 8.0 / window_s
        goodput[index] = rates.sum() / 1e6
        fairness[index] = jain(rates)
    return times, goodput, fairness


def render_figure(
    all_series: dict[str, pd.DataFrame],
    profile: pd.DataFrame,
    output_path: Path,
) -> None:
    plt.rcParams.update(
        {
            "font.family": "DejaVu Sans",
            "font.size": 9,
            "axes.titlesize": 10,
            "axes.labelsize": 9,
            "legend.fontsize": 8,
        }
    )
    figure, axes = plt.subplots(3, 1, figsize=(10.6, 7.2), sharex=True)
    for algorithm in ALGORITHMS:
        frame = all_series.get(algorithm)
        if frame is None or frame.empty:
            continue
        times, goodput_mbps, fairness = rolling_metrics(frame, 5.0)
        axes[0].plot(times, goodput_mbps, color=COLORS[algorithm], linewidth=1.25, label=algorithm)
        queue_smoothed = frame.queue_delay_ms.rolling(10, min_periods=1).mean()
        axes[1].plot(frame.time_s, queue_smoothed, color=COLORS[algorithm], linewidth=1.15)
        axes[2].plot(times, fairness, color=COLORS[algorithm], linewidth=1.15)
    stage_starts = profile.stage_start_s.to_numpy(dtype=float)
    stage_ends = profile.stage_end_s.to_numpy(dtype=float)
    for axis in axes:
        for start_s in stage_starts[1:]:
            axis.axvline(start_s, color="#777777", linestyle=(0, (2, 2)), linewidth=0.8, zorder=0)
        axis.grid(axis="y", alpha=0.2, linewidth=0.5)
    for start_s, end_s, rtt_ms in zip(
        stage_starts, stage_ends, profile.configured_base_rtt_ms.to_numpy(dtype=float)
    ):
        axes[0].annotate(
            f"{rtt_ms:g} ms",
            xy=((start_s + end_s) / 2.0, 1.0),
            xycoords=("data", "axes fraction"),
            xytext=(0, -3),
            textcoords="offset points",
            ha="center",
            va="top",
            fontsize=8,
        )
    axes[0].set_title("(a) Aggregate goodput: 5 s rolling window", loc="left")
    axes[0].set_ylabel("Goodput (Mbit/s)")
    axes[0].set_ylim(bottom=0.0)
    axes[1].set_title("(b) Queue delay: 1 s rolling mean", loc="left")
    axes[1].set_ylabel("Delay (ms)")
    axes[1].set_ylim(bottom=0.0)
    axes[2].set_title("(c) Flow fairness: 5 s rolling window", loc="left")
    axes[2].set_ylabel("Jain index")
    axes[2].set_xlabel("Time (s)")
    axes[2].set_ylim(0.0, 1.05)
    axes[0].legend(loc="lower left", ncol=4, frameon=False, columnspacing=0.8)
    figure.tight_layout()
    figure.savefig(output_path, dpi=220, bbox_inches="tight", pad_inches=0.03)
    plt.close(figure)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--results-dir", required=True, type=Path)
    parser.add_argument("--manifest", required=True, type=Path)
    args = parser.parse_args()

    results_dir = args.results_dir.resolve()
    manifest_path = args.manifest.resolve()
    summary_dir = results_dir / "summary"
    figure_dir = results_dir / "figures"
    summary_dir.mkdir(parents=True, exist_ok=True)
    figure_dir.mkdir(parents=True, exist_ok=True)

    manifest = pd.read_csv(manifest_path)
    require_columns(
        manifest,
        [
            "scenario_id", "capacity_bps", "settle_guard_s", "algorithm", "mode",
            "seed", "run_id", "run_summary_path", "rtt_profile_path",
            "rtt_timeseries_path", "metadata_path",
        ],
        "manifest",
    )
    if manifest.empty:
        raise ValueError("manifest contains no Test 3 runs")

    all_phase_records: list[dict[str, object]] = []
    all_transition_records: list[dict[str, object]] = []
    all_series: dict[str, pd.DataFrame] = {}
    validation: list[dict[str, object]] = []
    canonical_profile: pd.DataFrame | None = None
    for _, row in manifest.iterrows():
        profile = pd.read_csv(resolve_path(manifest_path, row.rtt_profile_path))
        series = pd.read_csv(resolve_path(manifest_path, row.rtt_timeseries_path))
        summary = pd.read_csv(resolve_path(manifest_path, row.run_summary_path))
        metadata = json.loads(resolve_path(manifest_path, row.metadata_path).read_text(encoding="utf-8"))
        require_columns(profile, ["stage_index", "stage_start_s", "stage_end_s", "configured_base_rtt_ms", "expected_bdp_bytes"], "rtt profile")
        require_columns(series, ["time_s", "stage_index", "queue_delay_ms", "aggregate_inflight_bytes", "expected_bdp_bytes", "snapshot_flow_count", "mean_srtt_us", "mean_min_rtt_us", *FLOW_COLUMNS], "rtt timeseries")
        require_columns(summary, ["algorithm", "mode", "validation_pass"], "run summary")
        valid = bool(int(summary.iloc[0].validation_pass)) and bool(metadata.get("validation_pass"))
        validation.append(
            {
                "scenario_id": row.scenario_id,
                "algorithm": row.algorithm,
                "mode": row["mode"],
                "run_id": int(row.run_id),
                "passed": valid,
                "detail": f"samples={len(series)}, stages={len(profile)}",
            }
        )
        if not valid:
            raise ValueError(f"raw validation failed for {row.algorithm}")
        profile = profile.sort_values("stage_index").reset_index(drop=True)
        series = series.sort_values("time_s").reset_index(drop=True)
        if canonical_profile is None:
            canonical_profile = profile[["stage_index", "stage_start_s", "stage_end_s", "configured_base_rtt_ms"]].copy()
        elif not canonical_profile.equals(profile[["stage_index", "stage_start_s", "stage_end_s", "configured_base_rtt_ms"]].copy()):
            raise ValueError("controller runs used different RTT profiles")
        all_series[str(row.algorithm)] = series
        all_phase_records.extend(phase_metrics(series, profile, row))
        all_transition_records.extend(transition_metrics(series, profile, row))

    phase = pd.DataFrame(all_phase_records)
    transitions = pd.DataFrame(all_transition_records)
    phase.to_csv(summary_dir / "phase_metrics.csv", index=False)
    transitions.to_csv(summary_dir / "transition_metrics.csv", index=False)
    validation_frame = pd.DataFrame(validation)
    validation_frame.to_csv(summary_dir / "validation.csv", index=False)

    overall = (
        phase.groupby(["scenario_id", "algorithm", "mode", "seed", "run_id"], as_index=False)
        .agg(
            mean_steady_utilization_pct=("utilization_pct", "mean"),
            min_steady_utilization_pct=("utilization_pct", "min"),
            mean_steady_jain=("jain_fairness", "mean"),
            min_steady_jain=("jain_fairness", "min"),
            mean_p95_queue_delay_ms=("p95_queue_delay_ms", "mean"),
            max_queue_delay_ms=("max_queue_delay_ms", "max"),
        )
    )
    if not transitions.empty:
        recovery = transitions.groupby(["scenario_id", "algorithm", "mode", "seed", "run_id"], as_index=False).agg(
            transition_count=("transition_index", "size"),
            transitions_reaching_90pct=("recovery_to_90pct_s", "count"),
            mean_transition_utilization_pct=("utilization_pct", "mean"),
            max_transition_queue_delay_ms=("max_queue_delay_ms", "max"),
            mean_recovery_to_90pct_s=("recovery_to_90pct_s", "mean"),
        )
        overall = overall.merge(recovery, on=["scenario_id", "algorithm", "mode", "seed", "run_id"], how="left")
    overall.to_csv(summary_dir / "overall_metrics.csv", index=False)

    if canonical_profile is not None:
        render_figure(all_series, canonical_profile, figure_dir / "dynamic_rtt_response.png")

    rows = overall.sort_values("algorithm").copy()
    validation_rows = validation_frame.sort_values("algorithm")
    stage_rows = phase.sort_values(["stage_index", "algorithm"])
    transition_rows = transitions.sort_values(["transition_index", "algorithm"])
    profile_schedule = "; ".join(
        f"{stage.stage_start_s:g}-{stage.stage_end_s:g} s: "
        f"{stage.configured_base_rtt_ms:g} ms"
        for stage in canonical_profile.itertuples()
    )
    lines = [
        "# Test 3: Dynamic Propagation RTT",
        "",
        "## Scenario",
        "",
        "Four long-lived flows share a fixed 100 Mbit/s bottleneck, a fixed 1,000,000-byte DropTail bottleneck queue, and fixed 1 Gbit/s access-link rates. Only the symmetric access-link propagation delay changes.",
        "",
        f"- Propagation RTT schedule: {profile_schedule}.",
        "- The bottleneck one-way propagation delay remains 10 ms; capacity, queue bytes, and flow population do not change.",
        "- Settled metrics exclude the first 15 seconds of each RTT stage. Transition metrics cover those excluded 15 seconds; recovery is the first full 5-second interval reaching 90% bottleneck utilization.",
        "",
        "## Raw Validation",
        "",
        "| Algorithm | Mode | Samples | RTT stages | Passed |",
        "| --- | --- | ---: | ---: | --- |",
    ]
    for row in validation_rows.itertuples():
        detail = str(row.detail).replace("samples=", "").replace(", stages=", " | ")
        lines.append(
            f"| {row.algorithm} | {row.mode} | {detail} | {'yes' if row.passed else 'no'} |"
        )
    lines.extend(
        [
            "",
            "## Overall",
            "",
            "| Algorithm | Mean util. (%) | Min util. (%) | Mean Jain | Min Jain | Mean p95 queue (ms) | Max queue (ms) | Reached 90% (steps) | Mean recovery to 90% (s) |",
            "| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |",
        ]
    )
    for row in rows.itertuples():
        recovery = getattr(row, "mean_recovery_to_90pct_s", float("nan"))
        recovered_steps = getattr(row, "transitions_reaching_90pct", 0)
        transition_count = getattr(row, "transition_count", 0)
        lines.append(
            f"| {row.algorithm} | {format_metric(row.mean_steady_utilization_pct, 2)} | "
            f"{format_metric(row.min_steady_utilization_pct, 2)} | "
            f"{format_metric(row.mean_steady_jain, 3)} | "
            f"{format_metric(row.min_steady_jain, 3)} | "
            f"{format_metric(row.mean_p95_queue_delay_ms, 2)} | "
            f"{format_metric(row.max_queue_delay_ms, 2)} | "
            f"{int(recovered_steps)}/{int(transition_count)} | "
            f"{format_metric(recovery, 2)} |"
        )
    lines.extend(
        [
            "",
            "## Settled Stages",
            "",
            "| Stage | RTT (ms) | Algorithm | Util. (%) | Jain | p95 queue (ms) | Max queue (ms) | Min-flow goodput (Mbit/s) |",
            "| ---: | ---: | --- | ---: | ---: | ---: | ---: | ---: |",
        ]
    )
    for row in stage_rows.itertuples():
        lines.append(
            f"| {row.stage_index} | {format_metric(row.configured_base_rtt_ms, 0)} | "
            f"{row.algorithm} | {format_metric(row.utilization_pct, 2)} | "
            f"{format_metric(row.jain_fairness, 3)} | "
            f"{format_metric(row.p95_queue_delay_ms, 2)} | "
            f"{format_metric(row.max_queue_delay_ms, 2)} | "
            f"{format_metric(row.min_flow_goodput_bps / 1e6, 2)} |"
        )
    lines.extend(
        [
            "",
            "## RTT Transitions",
            "",
            "| Time (s) | RTT change (ms) | Algorithm | Util. in first 15 s (%) | p95 queue (ms) | Recovery to 90% (s) |",
            "| ---: | --- | --- | ---: | ---: | ---: |",
        ]
    )
    for row in transition_rows.itertuples():
        lines.append(
            f"| {format_metric(row.transition_start_s, 0)} | "
            f"{format_metric(row.from_base_rtt_ms, 0)} -> "
            f"{format_metric(row.to_base_rtt_ms, 0)} | {row.algorithm} | "
            f"{format_metric(row.utilization_pct, 2)} | "
            f"{format_metric(row.p95_queue_delay_ms, 2)} | "
            f"{format_metric(row.recovery_to_90pct_s, 2)} |"
        )

    fbbr_phase = phase[phase["algorithm"] == "FBBR"]
    if not fbbr_phase.empty:
        lowest_fbbr = fbbr_phase.loc[fbbr_phase["utilization_pct"].idxmin()]
        lines.extend(
            [
                "",
                "## FBBR Observation",
                "",
                f"With the configured FBBR defaults, the lowest settled FBBR utilization is {lowest_fbbr.utilization_pct:.2f}% in the {lowest_fbbr.configured_base_rtt_ms:.0f} ms RTT stage. FBBR reaches 90% utilization in {int(rows.loc[rows['algorithm'] == 'FBBR', 'transitions_reaching_90pct'].iloc[0])}/4 transitions.",
                "This is a single-seed controller comparison, so it identifies a reproducible behavior to investigate rather than a confidence interval.",
            ]
        )
    lines.extend(
        [
            "",
            "## Outputs",
            "",
            "- `summary/phase_metrics.csv`: settled metrics, including per-flow goodput and BBR-style SRTT/MinRTT snapshots where available.",
            "- `summary/transition_metrics.csv`: first 15 seconds after each RTT step.",
            "- `summary/overall_metrics.csv`: compact controller comparison.",
            "- `figures/dynamic_rtt_response.png`: goodput, queue delay, and fairness over time.",
            "- `raw/DYN-RTT/`: raw samples, per-run profiles, metadata, and controller logs.",
        ]
    )
    (results_dir / "RESULTS.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
