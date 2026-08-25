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
from matplotlib import font_manager
from matplotlib.lines import Line2D


def register_times_new_roman() -> None:
    for font_path in font_manager.findSystemFonts(fontext="ttf"):
        if Path(font_path).stem.lower() == "times_new_roman":
            font_manager.fontManager.addfont(font_path)
            return


register_times_new_roman()


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
MARKERS = {
    "BBR-R": "o",
    "oBBR": "s",
    "BBRv2+": "^",
    "CUBIC": "D",
    "BBRv2-formal": "P",
    "BBRv2": "v",
    "FBBR": "X",
}
LINESTYLES = {
    "BBR-R": "-",
    "oBBR": (0, (6, 2)),
    "BBRv2+": (0, (4, 1.5, 1, 1.5)),
    "CUBIC": (0, (1, 1.5)),
    "BBRv2-formal": (0, (7, 1.5, 1.5, 1.5)),
    "BBRv2": (0, (4, 1.5, 1, 1.5, 1, 1.5)),
    "FBBR": (0, (1, 1.5, 1, 3)),
}
HATCHES = {
    "BBR-R": "",
    "oBBR": "//",
    "BBRv2+": "..",
    "CUBIC": "xx",
    "BBRv2": "\\\\",
    "FBBR": "++",
}
MARKER_TIME_OFFSETS_S = {
    label: offset
    for label, offset in zip(
        ALGORITHMS,
        (-0.75, -0.5, -0.25, 0.0, 0.25, 0.5, 0.75),
    )
}
PAPER_STYLE = {
    "font.family": "Times New Roman",
    "font.size": 8.0,
    "axes.titlesize": 8.5,
    "axes.labelsize": 8.0,
    "xtick.labelsize": 8.0,
    "ytick.labelsize": 8.0,
    "legend.fontsize": 8.0,
    "pdf.fonttype": 42,
    "ps.fonttype": 42,
}
MARKER_INTERVAL_S = 10.0


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


def plot_series_with_markers(
    axis: plt.Axes,
    times: np.ndarray,
    values: pd.Series | np.ndarray,
    algorithm: str,
    show_markers: bool = True,
    drawstyle: str = "default",
) -> None:
    numeric_values = np.asarray(values, dtype=float)
    axis.plot(
        times,
        numeric_values,
        color=COLORS[algorithm],
        linestyle=LINESTYLES[algorithm],
        linewidth=1.25,
        drawstyle=drawstyle,
        zorder=3,
    )
    if not show_markers:
        return
    marker_step = max(1, int(round(MARKER_INTERVAL_S / np.median(np.diff(times)))))
    marker_indices = np.arange(0, len(times), marker_step)
    axis.plot(
        times[marker_indices] + MARKER_TIME_OFFSETS_S[algorithm],
        numeric_values[marker_indices],
        color=COLORS[algorithm],
        linestyle="None",
        marker=MARKERS[algorithm],
        markerfacecolor=COLORS[algorithm],
        markeredgecolor="none",
        markeredgewidth=0.0,
        markersize=3.6,
        zorder=4,
    )


def queue_stage_statistics(
    frame: pd.DataFrame,
    profile: pd.DataFrame,
    settle_guard_s: float,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    means: list[float] = []
    minima: list[float] = []
    p95_values: list[float] = []
    for stage in profile.itertuples():
        steady_start_s = min(float(stage.stage_end_s), float(stage.stage_start_s) + settle_guard_s)
        values = frame.loc[
            (frame.time_s >= steady_start_s) & (frame.time_s < float(stage.stage_end_s)),
            "queue_delay_ms",
        ].astype(float)
        if values.empty:
            means.append(float("nan"))
            minima.append(float("nan"))
            p95_values.append(float("nan"))
            continue
        means.append(float(values.mean()))
        minima.append(float(values.min()))
        p95_values.append(float(values.quantile(0.95)))
    return np.asarray(means), np.asarray(minima), np.asarray(p95_values)


def rtprop_gap_statistics(
    frame: pd.DataFrame,
    profile: pd.DataFrame,
    settle_guard_s: float,
) -> np.ndarray:
    times = frame.time_s.to_numpy(dtype=float)
    sample_interval_s = float(np.median(np.diff(times)))
    filter_samples = max(1, int(round(10.0 / sample_interval_s)))
    latest_rtt_ms = frame.mean_latest_rtt_us.where(
        frame.mean_latest_rtt_us > 0
    ) / 1000.0
    estimated_rtprop_ms = latest_rtt_ms.rolling(
        filter_samples, min_periods=1
    ).min()
    true_rtprop_ms = frame.configured_base_rtt_ms.astype(float)
    relative_gap_pct = (
        (estimated_rtprop_ms - true_rtprop_ms).abs() / true_rtprop_ms * 100.0
    )
    standard_deviations: list[float] = []
    for stage in profile.itertuples():
        steady_start_s = min(float(stage.stage_end_s), float(stage.stage_start_s) + settle_guard_s)
        values = relative_gap_pct[
            (frame.time_s >= steady_start_s) & (frame.time_s < float(stage.stage_end_s))
        ].dropna()
        if values.empty:
            standard_deviations.append(float("nan"))
            continue
        standard_deviations.append(float(values.std(ddof=0)))
    return np.asarray(standard_deviations)


def render_figure(
    all_series: dict[str, pd.DataFrame],
    profile: pd.DataFrame,
    settle_guard_s: float,
    output_path: Path,
) -> None:
    plt.rcParams.update(PAPER_STYLE)
    figure, axes = plt.subplots(
        2,
        1,
        figsize=(3.5, 4.2),
        sharex=False,
        gridspec_kw={"height_ratios": [4.0, 6.0]},
    )
    present_algorithms = [
        algorithm
        for algorithm in ALGORITHMS
        if algorithm in all_series and not all_series[algorithm].empty
    ]
    for algorithm in present_algorithms:
        frame = all_series[algorithm]
        times, goodput_mbps, _ = rolling_metrics(frame, 5.0)
        plot_series_with_markers(axes[0], times, goodput_mbps, algorithm)

    stage_starts = profile.stage_start_s.to_numpy(dtype=float)
    stage_ends = profile.stage_end_s.to_numpy(dtype=float)
    stage_boundaries = np.concatenate(
        (stage_starts, np.asarray([stage_ends[-1]], dtype=float))
    )
    for start_s in stage_starts[1:]:
        axes[0].axvline(
            start_s,
            color="#777777",
            linestyle=(0, (2, 2)),
            linewidth=0.8,
            zorder=0,
        )
    axes[0].grid(axis="y", alpha=0.13, linewidth=0.45)
    axes[0].set_title("(a) Aggregate goodput", loc="left")
    axes[0].set_ylabel("Rate (Mbit/s)")
    axes[0].set_xlabel("Time (s)", labelpad=1.0)
    axes[0].set_ylim(0.0, 100.0)
    axes[0].set_xlim(stage_boundaries[0], stage_boundaries[-1])
    axes[0].set_xticks(stage_boundaries)
    axes[0].yaxis.labelpad = 1.0
    axes[0].tick_params(axis="both", which="major", length=2.2, width=0.6)

    stage_positions = np.arange(len(profile), dtype=float)
    bar_width = 0.12
    algorithm_offsets = (
        np.arange(len(present_algorithms), dtype=float)
        - (len(present_algorithms) - 1.0) / 2.0
    ) * bar_width
    maximum_queue = 0.0
    maximum_error = 0.0
    stage_statistics = {}
    for algorithm in present_algorithms:
        queue_means, queue_minima, queue_p95_values = queue_stage_statistics(
            all_series[algorithm], profile, settle_guard_s
        )
        rtprop_standard_deviations = rtprop_gap_statistics(
            all_series[algorithm], profile, settle_guard_s
        )
        stage_statistics[algorithm] = {
            "queue_means": queue_means,
            "queue_minima": queue_minima,
            "queue_p95_values": queue_p95_values,
            "rtprop_standard_deviations": rtprop_standard_deviations,
        }
        finite_queue = queue_p95_values
        finite_queue = finite_queue[np.isfinite(finite_queue)]
        if finite_queue.size > 0:
            maximum_queue = max(maximum_queue, float(finite_queue.max()))
        finite_error = rtprop_standard_deviations
        finite_error = finite_error[np.isfinite(finite_error)]
        if finite_error.size > 0:
            maximum_error = max(maximum_error, float(finite_error.max()))

    queue_limit = max(10.0, np.ceil(maximum_queue * 1.15 / 10.0) * 10.0)
    error_limit = max(50.0, np.ceil(maximum_error * 1.10 / 50.0) * 50.0)
    queue_scale = 100.0 / queue_limit
    error_scale = 100.0 / error_limit
    for algorithm, offset in zip(present_algorithms, algorithm_offsets):
        statistics = stage_statistics[algorithm]
        queue_means = statistics["queue_means"]
        queue_lower_errors = queue_means - statistics["queue_minima"]
        queue_upper_errors = statistics["queue_p95_values"] - queue_means
        axes[1].bar(
            stage_positions + offset,
            queue_means * queue_scale,
            width=bar_width * 0.88,
            color=COLORS[algorithm],
            alpha=0.82,
            edgecolor="none",
            zorder=3,
        )
        axes[1].errorbar(
            stage_positions + offset,
            queue_means * queue_scale,
            yerr=np.vstack((queue_lower_errors, queue_upper_errors)) * queue_scale,
            fmt="none",
            ecolor="#777777",
            elinewidth=0.9,
            capsize=2.2,
            capthick=0.8,
            zorder=4,
        )
        rtprop_standard_deviations = statistics["rtprop_standard_deviations"]
        axes[1].bar(
            stage_positions + offset,
            -rtprop_standard_deviations * error_scale,
            width=bar_width * 0.88,
            color="#8a8a8a",
            alpha=0.88,
            edgecolor="#555555",
            linewidth=0.35,
            hatch=HATCHES[algorithm],
            zorder=3,
        )

    axes[1].axhline(0.0, color="#222222", linewidth=0.9, zorder=2)
    for boundary in stage_positions[:-1] + 0.5:
        axes[1].axvline(
            boundary,
            color="#777777",
            linestyle=(0, (2, 2)),
            linewidth=0.8,
            zorder=0,
        )
    axes[1].set_title("(b) Queue delay / RTprop error", loc="left", pad=2.0)
    axes[1].set_ylabel("")
    axes[1].set_ylim(-100.0, 100.0)
    axes[1].set_xlim(-0.55, len(profile) - 0.45)
    queue_ticks = np.arange(0.0, queue_limit + 0.1, 20.0)
    error_ticks = np.arange(10.0, error_limit + 0.1, 10.0)
    tick_positions = np.concatenate(
        (
            -error_ticks[::-1] * error_scale,
            np.asarray([0.0]),
            queue_ticks[1:] * queue_scale,
        )
    )
    tick_labels = [f"{value:g}" for value in error_ticks[::-1]]
    tick_labels += ["0"]
    tick_labels += [f"{value:g}" for value in queue_ticks[1:]]
    axes[1].set_yticks(tick_positions)
    axes[1].set_yticklabels(tick_labels)
    axes[1].text(
        -0.10,
        0.75,
        "Delay (ms)",
        transform=axes[1].transAxes,
        rotation=90,
        ha="center",
        va="center",
        fontsize=8.0,
    )
    axes[1].text(
        -0.10,
        0.25,
        "Error (%)",
        transform=axes[1].transAxes,
        rotation=90,
        ha="center",
        va="center",
        fontsize=8.0,
    )
    axes[1].set_xlabel("Propagation RTT stage")
    axes[1].set_xticks(stage_positions)
    axes[1].set_xticklabels(
        [
            f"{rtt:g} ms"
            for rtt in profile.configured_base_rtt_ms.to_numpy(dtype=float)
        ],
    )
    axes[1].grid(axis="y", alpha=0.13, linewidth=0.45)
    axes[1].tick_params(axis="both", which="major", length=2.2, width=0.6)

    legend_handles = [
        Line2D(
            [0],
            [0],
            color=COLORS[algorithm],
            linestyle=LINESTYLES[algorithm],
            linewidth=1.25,
            marker=MARKERS[algorithm],
            markerfacecolor=COLORS[algorithm],
            markeredgecolor="none",
            markeredgewidth=0.0,
            markersize=3.6,
            label=algorithm,
        )
        for algorithm in present_algorithms
    ]
    figure.subplots_adjust(left=0.15, right=0.995, top=0.97, bottom=0.18, hspace=0.34)
    figure.legend(
        handles=legend_handles,
        loc="lower center",
        bbox_to_anchor=(0.5, 0.035),
        ncol=6,
        frameon=False,
        handlelength=1.0,
        handletextpad=0.20,
        columnspacing=0.65,
        labelspacing=0.18,
        borderaxespad=0.0,
    )
    figure.savefig(output_path, dpi=600, bbox_inches="tight", pad_inches=0.01)
    figure.savefig(output_path.with_suffix(".pdf"), bbox_inches="tight", pad_inches=0.01)
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
        require_columns(series, ["time_s", "stage_index", "queue_delay_ms", "aggregate_inflight_bytes", "expected_bdp_bytes", "snapshot_flow_count", "mean_srtt_us", "mean_min_rtt_us", "mean_latest_rtt_us", *FLOW_COLUMNS], "rtt timeseries")
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
        render_figure(
            all_series,
            canonical_profile,
            float(manifest.iloc[0].settle_guard_s),
            figure_dir / "dynamic_rtt_response.png",
        )

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
        "- Settled metrics exclude the first 15 seconds of each propagation RTT stage. Transition metrics cover those excluded 15 seconds; recovery is the first full 5-second interval reaching 90% bottleneck utilization.",
        "",
        "## Raw Validation",
        "",
        "| Algorithm | Mode | Samples | Propagation RTT stages | Passed |",
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
                f"With the configured FBBR defaults, the lowest settled FBBR utilization is {lowest_fbbr.utilization_pct:.2f}% in the {lowest_fbbr.configured_base_rtt_ms:.0f} ms propagation RTT stage. FBBR reaches 90% utilization in {int(rows.loc[rows['algorithm'] == 'FBBR', 'transitions_reaching_90pct'].iloc[0])}/4 transitions.",
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
            "- `figures/dynamic_rtt_response.png`: one left-right Fig.1-style figure with queue-delay stage means and downward relative-RTprop-error standard deviations.",
            "- `raw/DYN-RTT/`: raw samples, per-run profiles, metadata, and controller logs.",
        ]
    )
    (results_dir / "RESULTS.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
