#!/usr/bin/env python3
"""Render the original four-panel Test 1 figure for one selected FBBR run.

The baseline controllers are read from an existing Test 1 raw directory.  The
FBBR input is supplied separately so a freshly selected seed can replace only
the FBBR curve without changing the already completed controller runs.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import font_manager
from matplotlib.lines import Line2D
import pandas as pd


def register_times_new_roman() -> None:
    for font_path in font_manager.findSystemFonts(fontext="ttf"):
        if Path(font_path).stem.lower() == "times_new_roman":
            font_manager.fontManager.addfont(font_path)
            return


register_times_new_roman()


CAPACITY_BPS = 100_000_000.0
SIMULATION_TIME_S = 1800.0
STAGE_FLOWS = [2, 4, 8, 16, 8, 4, 2]
STAGE_EDGES_S = [SIMULATION_TIME_S * index / len(STAGE_FLOWS)
                 for index in range(len(STAGE_FLOWS) + 1)]
STAGE_EDGES_MIN = [value / 60.0 for value in STAGE_EDGES_S]
BASELINE_ALGORITHMS = [
    "BBR-R",
    "oBBR",
    "BBRv2+",
    "CUBIC",
    "BBRv2-formal",
    "BBRv2",
]
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
MARKER_TIME_OFFSETS_MIN = {
    label: offset
    for label, offset in zip(
        [*BASELINE_ALGORITHMS, "FBBR"],
        (-0.18, -0.12, -0.06, 0.0, 0.06, 0.12, 0.18),
    )
}
PAPER_STYLE = {
    "font.family": "Times New Roman",
    "font.size": 11.5,
    "axes.titlesize": 13.0,
    "axes.labelsize": 12.0,
    "xtick.labelsize": 10.5,
    "ytick.labelsize": 10.5,
    "legend.fontsize": 10.5,
    "pdf.fonttype": 42,
    "ps.fonttype": 42,
}


def as_bool(value: object) -> bool:
    return str(value).strip().lower() in {"1", "true", "yes"}


def parse_float(value: object) -> float | None:
    try:
        parsed = float(value)
    except (TypeError, ValueError):
        return None
    return parsed if math.isfinite(parsed) else None


def stage_index_for_time(time_s: float) -> int:
    index = int(time_s / (SIMULATION_TIME_S / len(STAGE_FLOWS)))
    return max(0, min(index, len(STAGE_FLOWS) - 1))


def prefix_for_summary(path: Path) -> Path:
    suffix = "_run_summary.csv"
    if not path.name.endswith(suffix):
        raise ValueError(f"Not a Test 1 run summary: {path}")
    return path.with_name(path.name[: -len(suffix)])


def find_run(raw_dir: Path, algorithm: str, seed: int | None = None) -> Path:
    candidates: list[Path] = []
    for summary_path in sorted(raw_dir.glob("*_run_summary.csv")):
        try:
            summary = pd.read_csv(summary_path)
        except (OSError, pd.errors.EmptyDataError):
            continue
        if summary.empty or "algorithm" not in summary or "seed" not in summary:
            continue
        row = summary.iloc[0]
        if str(row.algorithm) != algorithm:
            continue
        if seed is not None and int(row.seed) != seed:
            continue
        candidates.append(prefix_for_summary(summary_path))
    if len(candidates) != 1:
        qualifier = f" seed={seed}" if seed is not None else ""
        raise ValueError(
            f"Expected exactly one {algorithm}{qualifier} run in {raw_dir}; "
            f"found {len(candidates)}"
        )
    return candidates[0]


def require_file(prefix: Path, suffix: str) -> Path:
    path = prefix.with_name(prefix.name + suffix)
    if not path.is_file() or path.stat().st_size == 0:
        raise FileNotFoundError(f"Missing required data file: {path}")
    return path


def load_minute_metrics(prefix: Path, label: str) -> pd.DataFrame:
    data = pd.read_csv(require_file(prefix, "_minute_metrics.csv"))
    required = {
        "window_end_s",
        "mean_queue_delay_ms",
        "aggregate_goodput_bps",
        "jain_fairness",
    }
    missing = sorted(required - set(data.columns))
    if missing:
        raise ValueError(f"{prefix} minute metrics missing: {', '.join(missing)}")
    data = data.copy()
    data["plot_label"] = label
    data["time_min"] = data.window_end_s.astype(float) / 60.0
    data["aggregate_goodput_mbps"] = data.aggregate_goodput_bps.astype(float) / 1e6
    return data.sort_values("time_min")


def load_estimator_trace(prefix: Path) -> pd.DataFrame:
    data = pd.read_csv(require_file(prefix, "_diagnostic_flow_trace.csv"))
    required = {"time_s", "active_flows", "max_bw_bps"}
    missing = sorted(required - set(data.columns))
    if missing:
        raise ValueError(f"{prefix} diagnostic flow trace missing: {', '.join(missing)}")
    data = data.copy()
    data = data[(data.active_flows > 0) & (data.max_bw_bps > 0)]
    data = data[(data.time_s >= 0.0) & (data.time_s < SIMULATION_TIME_S)]
    data["max_bw_bps"] = data.max_bw_bps.astype(float)
    data["time_bin_s"] = data.time_s.astype(float).floordiv(1).astype(int)
    data["ratio"] = (
        data.max_bw_bps.astype(float)
        / (CAPACITY_BPS / data.active_flows.astype(float))
    )
    result = (
        data.groupby("time_bin_s", as_index=False)
        .agg(
            mean_ratio=("ratio", "mean"),
            std_ratio=("ratio", "std"),
            mean_bps=("max_bw_bps", "mean"),
            std_bps=("max_bw_bps", "std"),
            sample_count=("ratio", "size"),
        )
        .sort_values("time_bin_s")
    )
    result["std_ratio"] = result.std_ratio.fillna(0.0)
    result["std_bps"] = result.std_bps.fillna(0.0)
    result["lower_ratio"] = (result.mean_ratio - result.std_ratio).clip(lower=0.0)
    result["upper_ratio"] = result.mean_ratio + result.std_ratio
    result["lower_bps"] = (result.mean_bps - result.std_bps).clip(lower=0.0)
    result["upper_bps"] = result.mean_bps + result.std_bps
    result["mean_mbps"] = result.mean_bps / 1e6
    result["lower_mbps"] = result.lower_bps / 1e6
    result["upper_mbps"] = result.upper_bps / 1e6
    result["time_min"] = (result.time_bin_s.astype(float) + 0.5) / 60.0
    return result


def load_pacing_beq_samples(prefix: Path) -> pd.DataFrame:
    """Return the latest sampled pacing Beq record for every flow and second."""
    path = require_file(prefix, "_diagnostic_fbbr_trace.csv")
    latest_by_second_and_flow: dict[tuple[int, int], dict[str, object]] = {}
    with path.open(encoding="utf-8", newline="") as handle:
        for outer in csv.DictReader(handle):
            if outer.get("label") != "FREQ_GATE_TRACE":
                continue
            values = next(csv.reader([outer.get("diagnostics", "")]), [])
            if len(values) < 46 or values[2] != "pacing":
                continue
            time_s = parse_float(values[0])
            flow_id = parse_float(values[1])
            beq_bps = parse_float(values[24])
            pacing_base_bps = parse_float(values[41])
            if (
                time_s is None
                or flow_id is None
                or not 0.0 <= time_s < SIMULATION_TIME_S
            ):
                continue
            second = int(time_s)
            latest_by_second_and_flow[(second, int(flow_id))] = {
                "time_s": second + 0.5,
                "time_min": (second + 0.5) / 60.0,
                "time_bin_s": second,
                "flow_id": int(flow_id),
                "beq_bps": beq_bps or 0.0,
                "beq_valid": as_bool(values[25]),
                "beq_application_valid": as_bool(values[30]),
                "pacing_base_bps": pacing_base_bps or 0.0,
            }

    result = pd.DataFrame(latest_by_second_and_flow.values())
    if result.empty:
        raise ValueError(f"No sampled pacing Beq records found in {path}")
    return result.sort_values(["time_s", "flow_id"])


def summarize_realtime_beq_trace(pacing_samples: pd.DataFrame) -> pd.DataFrame:
    """Aggregate the currently valid Beq samples across active flows."""
    samples_by_second = {
        int(second): group.to_dict("records")
        for second, group in pacing_samples.groupby("time_bin_s", sort=True)
    }

    records: list[dict[str, object]] = []
    for second in range(int(SIMULATION_TIME_S)):
        samples = samples_by_second.get(second, [])
        if not samples:
            continue
        active_flow_count = len(samples)
        valid_samples = [
            sample
            for sample in samples
            if bool(sample["beq_valid"]) and float(sample["beq_bps"]) > 0.0
        ]
        valid_flow_count = len(valid_samples)
        stage_index = stage_index_for_time(second + 0.5)
        records.append(
            {
                "time_s": second + 0.5,
                "time_min": (second + 0.5) / 60.0,
                "stage_index": stage_index,
                "stage_label": f"N{STAGE_FLOWS[stage_index]}",
                "active_flow_count": active_flow_count,
                "valid_flow_count": valid_flow_count,
                "valid_flow_fraction": valid_flow_count / active_flow_count,
                "application_valid_flow_count": sum(
                    bool(sample["beq_application_valid"]) for sample in samples
                ),
                "mean_active_beq_bps": sum(
                    float(sample["beq_bps"])
                    if bool(sample["beq_valid"])
                    else 0.0
                    for sample in samples
                )
                / active_flow_count,
                "mean_valid_beq_bps": (
                    sum(float(sample["beq_bps"]) for sample in valid_samples)
                    / valid_flow_count
                    if valid_flow_count
                    else float("nan")
                ),
                "mean_pacing_base_bps": sum(
                    float(sample["pacing_base_bps"]) for sample in samples
                )
                / active_flow_count,
            }
        )
    result = pd.DataFrame(records)
    if result.empty:
        raise ValueError("No sampled pacing Beq records available for aggregation")
    return result.sort_values("time_s")


def summarize_held_beq_trace(pacing_samples: pd.DataFrame) -> pd.DataFrame:
    """Forward-fill each active flow's last valid Beq, then take its mean.

    A Beq publication is per sender.  The trace's current validity can later
    be cleared by the convergence gate, but that is not a new Beq output.
    This view therefore keeps a flow's latest valid value until it publishes a
    replacement or leaves the active set.
    """
    samples_by_second = {
        int(second): group.to_dict("records")
        for second, group in pacing_samples.groupby("time_bin_s", sort=True)
    }
    held_by_flow: dict[int, float] = {}
    records: list[dict[str, object]] = []
    for second in range(int(SIMULATION_TIME_S)):
        samples = samples_by_second.get(second, [])
        if not samples:
            continue
        for sample in samples:
            if bool(sample["beq_valid"]) and float(sample["beq_bps"]) > 0.0:
                held_by_flow[int(sample["flow_id"])] = float(sample["beq_bps"])
        held_values = [
            held_by_flow[int(sample["flow_id"])]
            for sample in samples
            if int(sample["flow_id"]) in held_by_flow
        ]
        stage_index = stage_index_for_time(second + 0.5)
        records.append(
            {
                "time_s": second + 0.5,
                "time_min": (second + 0.5) / 60.0,
                "stage_index": stage_index,
                "stage_label": f"N{STAGE_FLOWS[stage_index]}",
                "active_flow_count": len(samples),
                "held_beq_flow_count": len(held_values),
                "held_beq_coverage": len(held_values) / len(samples),
                "mean_held_beq_bps": (
                    sum(held_values) / len(held_values)
                    if held_values
                    else float("nan")
                ),
            }
        )
    result = pd.DataFrame(records)
    if result.empty:
        raise ValueError("No sampled pacing Beq records available for hold")
    return result.sort_values("time_s")


def load_cruise_beq_updates(prefix: Path) -> pd.DataFrame:
    """Read every valid CRUISE-exit Beq or GuardBw publication.

    FBBR's CRUISE selector publishes ``guard_filter_stage2_`` through the
    normal Beq field whenever GuardBw was updated.  The source is then
    ``GUARD_FILTER``.  Consequently the common Beq field contains both the
    direct Beq and GuardBw outcomes needed by the figure.
    """
    path = require_file(prefix, "_diagnostic_fbbr_trace.csv")
    records: list[dict[str, object]] = []
    with path.open(encoding="utf-8", newline="") as handle:
        for outer in csv.DictReader(handle):
            if outer.get("label") != "CRUISE_SUMMARY":
                continue
            values = next(csv.reader([outer.get("diagnostics", "")]), [])
            if len(values) < 46:
                continue
            time_s = parse_float(outer.get("callback_time_s"))
            trace_flow_id = parse_float(outer.get("flow_id"))
            cruise_start_s = parse_float(values[1])
            beq_bps = parse_float(values[16])
            if (
                time_s is None
                or beq_bps is None
                or not 0.0 <= time_s < SIMULATION_TIME_S
                or beq_bps <= 0.0
                or not as_bool(values[29])
            ):
                continue
            records.append(
                {
                    "time_s": time_s,
                    "time_min": time_s / 60.0,
                    "trace_flow_id": trace_flow_id,
                    "cruise_start_s": cruise_start_s,
                    "beq_bps": beq_bps,
                    "beq_source": values[17],
                    "beq_application_valid": as_bool(values[32]),
                }
            )
    result = pd.DataFrame(records)
    if result.empty:
        raise ValueError(f"No valid CRUISE Beq or GuardBw records found in {path}")
    return result.sort_values("time_s")


def format_time_tick(time_min: float) -> str:
    rounded = round(time_min)
    if abs(time_min - rounded) < 1e-9:
        return str(rounded)
    return f"{time_min:.1f}"


def configure_flow_stage_axis(
    axis: plt.Axes,
    *,
    show_xlabel: bool = True,
    show_ticklabels: bool = True,
    show_stage_lines: bool = True,
) -> None:
    """Use the archived figure's transition ticks and centered flow labels."""
    starts_min = STAGE_EDGES_MIN[:-1]
    ends_min = STAGE_EDGES_MIN[1:]
    centers_min = [(start + end) / 2.0 for start, end in zip(starts_min, ends_min)]
    marker_margin_min = max(abs(offset) for offset in MARKER_TIME_OFFSETS_MIN.values())
    axis.set_xlim(
        starts_min[0] - marker_margin_min - 0.03,
        ends_min[-1] + marker_margin_min + 0.03,
    )
    if show_stage_lines:
        for boundary_min in starts_min[1:]:
            axis.axvline(
                boundary_min,
                color="#555555",
                linestyle=(0, (2, 2)),
                linewidth=0.55,
                alpha=0.48,
                zorder=0,
            )
    if show_ticklabels:
        transition_ticks = [*starts_min, ends_min[-1]]
        axis.set_xticks(transition_ticks)
        axis.set_xticklabels([format_time_tick(value) for value in transition_ticks])
        axis.tick_params(axis="x", which="major", length=3.2, width=0.7, pad=21)
        for center_min, flow_count in zip(centers_min, STAGE_FLOWS):
            axis.annotate(
                f"{flow_count}\nflows",
                xy=(center_min, 0.0),
                xycoords=axis.get_xaxis_transform(),
                xytext=(0.0, -1.0),
                textcoords="offset points",
                ha="center",
                va="top",
                fontsize=PAPER_STYLE["xtick.labelsize"],
                linespacing=0.9,
                color="#333333",
                annotation_clip=False,
            )
    else:
        axis.set_xticks([])
    if show_xlabel:
        axis.set_xlabel("Time (min)", labelpad=0)


def plot_minute_lines(axis: plt.Axes, series: dict[str, pd.DataFrame], column: str) -> None:
    for label, data in series.items():
        axis.plot(
            data.time_min,
            data[column].astype(float),
            color=COLORS[label],
            linestyle=LINESTYLES[label],
            linewidth=1.25,
            zorder=3,
        )
        axis.plot(
            data.time_min + MARKER_TIME_OFFSETS_MIN[label],
            data[column].astype(float),
            color=COLORS[label],
            linestyle="None",
            marker=MARKERS[label],
            markerfacecolor=COLORS[label],
            markeredgecolor="none",
            markeredgewidth=0.0,
            markersize=5.3,
            zorder=4,
        )


def add_break_marks(upper: plt.Axes, lower: plt.Axes) -> None:
    upper.spines.bottom.set_visible(False)
    lower.spines.top.set_visible(False)
    upper.tick_params(labelbottom=False, bottom=False)
    lower.tick_params(top=False)
    size = 0.014
    kwargs = dict(color="black", clip_on=False, linewidth=0.9)
    upper.plot((-size, size), (-size, size), transform=upper.transAxes, **kwargs)
    upper.plot((1 - size, 1 + size), (-size, size), transform=upper.transAxes, **kwargs)
    lower.plot((-size, size), (1 - size, 1 + size), transform=lower.transAxes, **kwargs)
    lower.plot((1 - size, 1 + size), (1 - size, 1 + size), transform=lower.transAxes, **kwargs)


def plot_broken_metric(
    upper: plt.Axes,
    lower: plt.Axes,
    series: dict[str, pd.DataFrame],
    column: str,
    upper_limits: tuple[float, float],
    lower_limits: tuple[float, float],
    ylabel: str,
    title: str,
) -> None:
    plot_minute_lines(upper, series, column)
    plot_minute_lines(lower, series, column)
    upper.set_ylim(*upper_limits)
    lower.set_ylim(*lower_limits)
    upper.set_title(title, loc="left", pad=5)
    upper.set_ylabel("")
    configure_flow_stage_axis(upper, show_xlabel=False, show_ticklabels=False)
    configure_flow_stage_axis(lower)
    upper.grid(axis="y", alpha=0.13, linewidth=0.45)
    lower.grid(axis="y", alpha=0.13, linewidth=0.45)
    upper.tick_params(axis="y", which="major", length=3, width=0.7)
    lower.tick_params(axis="y", which="major", length=3, width=0.7)
    add_break_marks(upper, lower)


def plot_estimator_panel(
    axis: plt.Axes,
    fbbr_native: pd.DataFrame,
    held_beq: pd.DataFrame,
) -> None:
    plotted_beq = held_beq.dropna(subset=["mean_held_beq_bps"]).reset_index(drop=True)
    if not plotted_beq.empty:
        # Keep the final per-flow-held mean through the simulation endpoint.
        plotted_beq.loc[len(plotted_beq)] = {
            "time_s": SIMULATION_TIME_S,
            "time_min": SIMULATION_TIME_S / 60.0,
            "mean_held_beq_bps": float(plotted_beq.iloc[-1].mean_held_beq_bps),
        }
        axis.step(
            plotted_beq.time_min,
            plotted_beq.mean_held_beq_bps / 1e6,
            where="post",
            color=COLORS["FBBR"],
            linewidth=1.4,
            label="FBBR mean Beq",
            zorder=5,
        )
    axis.plot(
        fbbr_native.time_min,
        fbbr_native.mean_mbps,
        color="#222222",
        linestyle="--",
        linewidth=1.1,
        label="FBBR native MaxBw",
        zorder=3,
    )
    fair_share_mbps = [CAPACITY_BPS / count / 1e6 for count in STAGE_FLOWS]
    axis.step(
        STAGE_EDGES_MIN,
        fair_share_mbps + [fair_share_mbps[-1]],
        where="post",
        color="#777777",
        linestyle="-.",
        linewidth=1.2,
        label="Fair share",
        zorder=2,
    )
    axis.set_title("(a) Bandwidth estimates", loc="left", pad=5)
    axis.set_ylabel("Estimate (Mbit/s)")
    beq_max_mbps = (
        float(plotted_beq.mean_held_beq_bps.max()) / 1e6
        if not plotted_beq.empty
        else 0.0
    )
    y_max = max(
        55.0,
        max(fair_share_mbps) * 1.08,
        float(fbbr_native.upper_mbps.max()) * 1.05,
        beq_max_mbps * 1.05,
    )
    axis.set_ylim(0.0, y_max)
    configure_flow_stage_axis(axis)
    axis.grid(axis="y", alpha=0.13, linewidth=0.45)
    axis.tick_params(axis="y", which="major", length=3, width=0.7)
    axis.legend(
        loc="upper center",
        bbox_to_anchor=(0.52, 1.0),
        ncol=1,
        frameon=False,
        fontsize=10.5,
        handlelength=1.8,
        handletextpad=0.35,
        labelspacing=0.25,
        borderaxespad=0.25,
    )


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Render the original four-panel Fig. 1 with one selected FBBR seed."
    )
    parser.add_argument("--baseline-raw", required=True, type=Path)
    parser.add_argument("--fbbr-raw", required=True, type=Path)
    parser.add_argument("--seed", required=True, type=int)
    parser.add_argument("--output-dir", required=True, type=Path)
    args = parser.parse_args()

    baseline_raw = args.baseline_raw.resolve()
    fbbr_raw = args.fbbr_raw.resolve()
    output_dir = args.output_dir.resolve()
    figure_dir = output_dir / "figures"
    summary_dir = output_dir / "summary"
    figure_dir.mkdir(parents=True, exist_ok=True)
    summary_dir.mkdir(parents=True, exist_ok=True)

    series: dict[str, pd.DataFrame] = {}
    prefixes: dict[str, Path] = {}
    formal_label = "BBRv2-formal"
    for algorithm in BASELINE_ALGORITHMS:
        if algorithm == formal_label:
            try:
                prefix = find_run(baseline_raw, algorithm)
            except ValueError:
                formal_label = "BBRv2-ideal"
                prefix = find_run(baseline_raw, formal_label)
        else:
            prefix = find_run(baseline_raw, algorithm)
        prefixes[algorithm] = prefix
        series[algorithm] = load_minute_metrics(prefix, algorithm)
    fbbr_prefix = find_run(fbbr_raw, "FBBR", args.seed)
    prefixes["FBBR"] = fbbr_prefix
    series["FBBR"] = load_minute_metrics(fbbr_prefix, "FBBR")

    estimator_bbrv2 = load_estimator_trace(prefixes["BBRv2"])
    estimator_formal = load_estimator_trace(prefixes["BBRv2-formal"])
    estimator_fbbr = load_estimator_trace(fbbr_prefix)
    pacing_beq_samples = load_pacing_beq_samples(fbbr_prefix)
    beq_trace = summarize_realtime_beq_trace(pacing_beq_samples)
    held_beq_trace = summarize_held_beq_trace(pacing_beq_samples)
    cruise_beq = load_cruise_beq_updates(fbbr_prefix)

    minute_data = pd.concat(series.values(), ignore_index=True)
    minute_data.to_csv(summary_dir / "fig1b_minute_data.csv", index=False)
    estimator_data = pd.concat(
        [
            estimator_bbrv2.assign(estimator="BBRv2 MaxBw"),
            estimator_formal.assign(estimator="BBRv2-formal MaxBw"),
            estimator_fbbr.assign(estimator="FBBR native MaxBw"),
        ],
        ignore_index=True,
    )
    estimator_data.to_csv(summary_dir / "fig1b_estimator_trace.csv", index=False)
    beq_trace.to_csv(summary_dir / "fig1b_beq_trace.csv", index=False)
    held_beq_trace.to_csv(summary_dir / "fig1b_held_beq_trace.csv", index=False)
    cruise_beq.to_csv(summary_dir / "fig1b_cruise_beq_updates.csv", index=False)
    selection = {
        "selected_fbbr_seed": args.seed,
        "baseline_raw": str(baseline_raw),
        "selected_fbbr_raw": str(fbbr_raw),
        "run_prefixes": {label: str(prefix) for label, prefix in prefixes.items()},
    }
    (output_dir / "selection.json").write_text(
        json.dumps(selection, indent=2) + "\n", encoding="utf-8"
    )

    plt.rcParams.update(PAPER_STYLE)
    figure = plt.figure(figsize=(15.25, 3.95))
    panel_grid = figure.add_gridspec(1, 4, wspace=0.28)
    estimate_axis = figure.add_subplot(panel_grid[0])
    goodput_grid = panel_grid[1].subgridspec(
        2, 1, height_ratios=(1.75, 0.8), hspace=0.055
    )
    queue_grid = panel_grid[2].subgridspec(
        2, 1, height_ratios=(1.45, 1.0), hspace=0.055
    )
    goodput_upper = figure.add_subplot(goodput_grid[0])
    goodput_lower = figure.add_subplot(goodput_grid[1], sharex=goodput_upper)
    queue_upper = figure.add_subplot(queue_grid[0])
    queue_lower = figure.add_subplot(queue_grid[1], sharex=queue_upper)
    fairness_axis = figure.add_subplot(panel_grid[3])

    plot_broken_metric(
        queue_upper,
        queue_lower,
        series,
        "mean_queue_delay_ms",
        (240.0, 1650.0),
        (0.0, 220.0),
        "Delay (ms)",
        "(c) Queue dynamics",
    )
    plot_broken_metric(
        goodput_upper,
        goodput_lower,
        series,
        "aggregate_goodput_mbps",
        (94.0, 98.0),
        (80.0, 83.0),
        "Goodput (Mbit/s)",
        "(b) Aggregate goodput",
    )
    plot_minute_lines(fairness_axis, series, "jain_fairness")
    fairness_axis.set_ylim(0.4, 1.05)
    fairness_axis.set_yticks([0.6, 0.8, 1.0])
    fairness_axis.set_ylabel("Jain index")
    fairness_axis.set_title("(d) Flow fairness", loc="left", pad=5)
    configure_flow_stage_axis(fairness_axis)
    fairness_axis.grid(axis="y", alpha=0.13, linewidth=0.45)
    fairness_axis.tick_params(axis="y", which="major", length=3, width=0.7)
    plot_estimator_panel(
        estimate_axis,
        estimator_fbbr,
        held_beq_trace,
    )

    figure.subplots_adjust(left=0.051, right=0.995, top=0.9, bottom=0.34)
    for upper_axis, lower_axis, ylabel in (
        (queue_upper, queue_lower, "Delay (ms)"),
        (goodput_upper, goodput_lower, "Goodput (Mbit/s)"),
    ):
        upper_box = upper_axis.get_position()
        lower_box = lower_axis.get_position()
        figure.text(
            upper_box.x0 - 0.024,
            (upper_box.y1 + lower_box.y0) / 2.0,
            ylabel,
            rotation="vertical",
            ha="center",
            va="center",
            fontsize=12.0,
        )

    legend_handles = [
        Line2D(
            [0],
            [0],
            color=COLORS[label],
            linestyle=LINESTYLES[label],
            linewidth=1.25,
            marker=MARKERS[label],
            markerfacecolor=COLORS[label],
            markeredgecolor="none",
            markeredgewidth=0.0,
            markersize=4.9,
            label=label,
        )
        for label in [*BASELINE_ALGORITHMS, "FBBR"]
    ]
    figure.legend(
        handles=legend_handles,
        loc="lower center",
        bbox_to_anchor=(0.5, 0.12),
        ncol=7,
        frameon=False,
        handlelength=1.2,
        handletextpad=0.25,
        columnspacing=0.42,
        borderaxespad=0.0,
    )
    png_path = figure_dir / "fig1b_excess_inflight_and_queue_delay.png"
    pdf_path = figure_dir / "fig1b_excess_inflight_and_queue_delay.pdf"
    figure.savefig(png_path, dpi=300, bbox_inches="tight", pad_inches=0.02)
    figure.savefig(pdf_path, bbox_inches="tight", pad_inches=0.02)
    plt.close(figure)
    print(png_path)
    print(pdf_path)


if __name__ == "__main__":
    main()
