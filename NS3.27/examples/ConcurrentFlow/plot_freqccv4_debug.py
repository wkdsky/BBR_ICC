#!/usr/bin/env python3
"""
Plot single-run FreqCCv4 debug figures.

The script is intentionally trace-format driven. It works with the current
FreqCCv4 summary traces and automatically uses richer pacing traces if a run was
captured with flow*_freq_gate_trace.csv or *_sendrate.txt enabled.
"""

import argparse
import csv
import json
import math
import re
import shlex
import sys
from bisect import bisect_right
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.path import Path as MplPath
from matplotlib.patches import Patch, PathPatch, Rectangle


# Publication-style typography. Times New Roman is preferred; the remaining
# serif fonts are fallbacks for systems where it is unavailable.
plt.rcParams.update(
    {
        "font.family": "serif",
        "font.serif": ["Times New Roman", "Times", "Liberation Serif", "DejaVu Serif"],
        "mathtext.fontset": "stix",
        "font.size": 10.5,
        "axes.labelsize": 12,
        "axes.titlesize": 13,
        "legend.fontsize": 10.5,
        "xtick.labelsize": 10,
        "ytick.labelsize": 10,
        "axes.linewidth": 0.8,
        "pdf.fonttype": 42,
        "ps.fonttype": 42,
    }
)


FLOW_RE = re.compile(r"flow(\d+)", re.IGNORECASE)

LINE_COLORS = {
    "trusted": "#1f77b4",
    "effective": "#d62728",
    "maxbw": "#2ca02c",
    "native": "#9467bd",
    "fair": "#555555",
    "final_pacing": "#d62728",
    "native_pacing": "#1f77b4",
    "aggregate": "#111111",
    "latest_rtt": "#ff7f0e",
    "smoothed_rtt": "#1f77b4",
}

# Delivery-comparison overlays use a separate, color-accessible pair so they
# remain distinguishable from the blue/orange raw-sample series. State is also
# encoded redundantly by step geometry, marker shape, and line style.
MAXBW_STATE_COLOR = "#7A5195"      # muted purple
TRUSTEDBW_STATE_COLOR = "#008B8B"  # dark teal
WINDOW_EDGE_COLOR = "#B7791F"      # amber sampling-window boundary


MODE_COLORS = {
    "start": "#7f7f7f",
    "drain": "#ff7f0e",
    "probeBW_cruise": "#2ca02c",
    "probeBW_refill": "#1f77b4",
    "probeBW_up": "#d62728",
    "probeBW_down": "#9467bd",
    "probeBW_down_slightly": "#8c564b",
}

PHASE_BG_ALPHA = 0.085
PHASE_BG_COLORS = {
    "probeBW_cruise": "#2ca02c",
    "probeBW_refill": "#1f77b4",
    "probeBW_up": "#d62728",
    "probeBW_down": "#9467bd",
    "probeRTT": "#7f7f7f",
}
PHASE_LABELS = {
    "probeBW_cruise": "PROBE_BW cruise",
    "probeBW_refill": "PROBE_BW refill",
    "probeBW_up": "PROBE_BW up",
    "probeBW_down": "PROBE_BW down",
    "probeRTT": "PROBE_RTT",
}
PHASE_SHORT_LABELS = {
    "start": "START",
    "drain": "DRAIN",
    "probeBW_cruise": "CRUISE",
    "probeBW_refill": "REFILL",
    "probeBW_up": "UP",
    "probeBW_down": "DOWN",
    "probeRTT": "PROBE_RTT",
}


def parse_args(argv: Optional[List[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Create FreqCCv4 per-run debug plots from an FreqCCv4 trace directory."
    )
    parser.add_argument(
        "--run-dir",
        required=True,
        help="FreqCCv4 run directory, e.g. .../four_cc.../FreqCCv4.",
    )
    parser.add_argument(
        "--output-dir",
        default="",
        help="Output directory. Defaults to RUN_DIR/debug_plots.",
    )
    parser.add_argument(
        "--service-rate",
        default="",
        help="Bottleneck service rate. Defaults to run config/comparison_config, then 100Mbps.",
    )
    parser.add_argument(
        "--sample-step-s",
        type=float,
        default=0.1,
        help="Sampling/binning step for aggregate time series. Default: 0.1s.",
    )
    parser.add_argument(
        "--warmup-s",
        type=float,
        default=5.0,
        help="Warmup skipped by the summary CSV. Default: 5s.",
    )
    return parser.parse_args(argv)


def parse_rate_bps(text: str) -> float:
    value = str(text).strip()
    match = re.fullmatch(r"([0-9]+(?:\.[0-9]+)?)\s*([kKmMgG]?)\s*(?:b(?:it)?s?|bps)?", value)
    if not match:
        raise ValueError(f"Cannot parse rate: {text}")
    number = float(match.group(1))
    unit = match.group(2).lower()
    scale = {"": 1.0, "k": 1e3, "m": 1e6, "g": 1e9}[unit]
    return number * scale


def infer_service_rate(run_dir: Path, user_value: str) -> float:
    if user_value:
        return parse_rate_bps(user_value)

    candidates: List[str] = []
    for path, key in (
        (run_dir / "config.json", "service_rate"),
        (run_dir.parent / "comparison_config.json", "shared_bottleneck_rate"),
    ):
        if not path.exists():
            continue
        try:
            data = json.loads(path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError):
            continue
        value = data.get(key)
        if value:
            candidates.append(str(value))

    for value in candidates:
        try:
            return parse_rate_bps(value)
        except ValueError:
            pass
    return parse_rate_bps("100Mbps")


def flow_id_from_path(path: Path) -> int:
    match = FLOW_RE.search(path.name)
    if not match:
        return 0
    return int(match.group(1))


def to_float(value: object, default: float = math.nan) -> float:
    if value is None:
        return default
    text = str(value).strip()
    if not text or text.lower() in {"nan", "none", "null"}:
        return default
    try:
        return float(text)
    except ValueError:
        return default


def to_bool(value: object) -> bool:
    return str(value).strip().lower() in {"1", "true", "yes", "y"}


def finite_values(values: Iterable[float]) -> List[float]:
    return [value for value in values if math.isfinite(value)]


def percentile(values: Iterable[float], pct: float) -> float:
    valid = sorted(finite_values(values))
    if not valid:
        return math.nan
    if len(valid) == 1:
        return valid[0]
    pos = (len(valid) - 1) * pct / 100.0
    lo = int(math.floor(pos))
    hi = int(math.ceil(pos))
    if lo == hi:
        return valid[lo]
    return valid[lo] + (valid[hi] - valid[lo]) * (pos - lo)


def build_grid(max_time_s: float, step_s: float) -> List[float]:
    if max_time_s <= 0.0:
        return [0.0]
    count = int(math.ceil(max_time_s / step_s)) + 1
    return [round(idx * step_s, 10) for idx in range(count)]


def sample_previous(
    series: Sequence[Tuple[float, float]],
    grid: Sequence[float],
    fill: float = math.nan,
) -> List[float]:
    if not series:
        return [fill for _ in grid]
    ordered = sorted(series, key=lambda item: item[0])
    times = [item[0] for item in ordered]
    values = [item[1] for item in ordered]
    sampled: List[float] = []
    for time_s in grid:
        idx = bisect_right(times, time_s) - 1
        sampled.append(values[idx] if idx >= 0 else fill)
    return sampled


def write_csv(path: Path, header: Sequence[str], rows: Iterable[Sequence[object]]) -> None:
    with path.open("w", newline="", encoding="utf-8") as fh:
        writer = csv.writer(fh)
        writer.writerow(header)
        writer.writerows(rows)


def clean_output_dir(output_dir: Path) -> None:
    patterns = (
        "*.png",
        "*_timeseries.csv",
        "aggregate_round_bandwidth.csv",
        "delivery_rate_first10_timeseries.csv",
        "delivery_rate_bbrv2_vs_freqccv4_*.csv",
        "delivery_rate_trace_missing.txt",
        "delivery_rate_bbrv2_vs_freqccv4_missing.txt",
        "debug_summary.csv",
        "manifest.txt",
        "pacing_trace_missing.txt",
        "plot_command.txt",
        "round_bandwidth_selection.csv",
        "srtt_trace_missing.txt",
    )
    for pattern in patterns:
        for path in output_dir.glob(pattern):
            if path.is_file():
                path.unlink()


def phase_key(mode: str) -> Optional[str]:
    normalized = mode.strip().lower().replace("_", "").replace("-", "")
    if "probertt" in normalized:
        return "probeRTT"
    if normalized in {"probebwcruise", "probecruise"}:
        return "probeBW_cruise"
    if normalized in {"probebwrefill", "proberefill"}:
        return "probeBW_refill"
    if normalized in {"probebwup", "probeup"}:
        return "probeBW_up"
    if normalized in {"probebwdown", "probedown", "probebwdownslightly", "probedownslightly"}:
        return "probeBW_down"
    return None


def mode_segments_for_flow(
    run_dir: Path,
    flow_id: int,
    max_time_s: Optional[float] = None,
) -> List[Tuple[float, float, str]]:
    mode_file = None
    for path in find_mode_files(run_dir):
        if flow_id_from_path(path) == flow_id:
            mode_file = path
            break
    if mode_file is None:
        return []
    rows = load_modes(mode_file)
    if not rows:
        return []
    if max_time_s is None or max_time_s <= 0.0:
        max_time_s = rows[-1][0]
    segments: List[Tuple[float, float, str]] = []
    for idx, (start, mode) in enumerate(rows):
        end = rows[idx + 1][0] if idx + 1 < len(rows) else max_time_s
        if end > start:
            segments.append((start, end, mode))
    return segments


def first_flow_id_with_modes(run_dir: Path) -> int:
    flow_ids = [flow_id_from_path(path) for path in find_mode_files(run_dir)]
    flow_ids = [flow_id for flow_id in flow_ids if flow_id > 0]
    return min(flow_ids) if flow_ids else 0


def add_phase_background(
    ax,
    run_dir: Path,
    flow_id: int,
    max_time_s: Optional[float] = None,
) -> None:
    if flow_id <= 0:
        return
    for start, end, mode in mode_segments_for_flow(run_dir, flow_id, max_time_s):
        key = phase_key(mode)
        if key is None:
            continue
        ax.axvspan(
            start,
            end,
            color=PHASE_BG_COLORS[key],
            alpha=PHASE_BG_ALPHA,
            linewidth=0,
            zorder=0,
        )


def add_phase_legend(ax) -> None:
    handles = [
        Patch(
            facecolor=PHASE_BG_COLORS[key],
            edgecolor="none",
            alpha=PHASE_BG_ALPHA * 2.0,
            label=PHASE_LABELS[key],
        )
        for key in ("probeBW_cruise", "probeBW_refill", "probeBW_up", "probeBW_down", "probeRTT")
    ]
    legend = ax.legend(
        handles=handles,
        ncol=3,
        frameon=True,
        fontsize=7,
        loc="upper left",
        framealpha=0.70,
        borderpad=0.3,
        handlelength=1.6,
    )
    ax.add_artist(legend)


def short_phase_label(mode: str) -> str:
    key = phase_key(mode)
    if key:
        return PHASE_SHORT_LABELS[key]
    normalized = mode.strip()
    return PHASE_SHORT_LABELS.get(normalized, normalized)


def compact_phase_label(mode: str, segment_width: float) -> str:
    """Keep complete phase names even for short lower-timeline segments."""
    return short_phase_label(mode)


def add_phase_separators_with_labels(
    ax,
    run_dir: Path,
    flow_id: int,
    start_s: float,
    end_s: float,
    alternating_background: bool = False,
) -> None:
    segments = mode_segments_for_flow(run_dir, flow_id, end_s)
    if not segments:
        return
    boundaries = {start_s, end_s}
    clipped: List[Tuple[float, float, str]] = []
    for start, end, mode in segments:
        left = max(start_s, start)
        right = min(end_s, end)
        if right <= left:
            continue
        boundaries.add(left)
        boundaries.add(right)
        clipped.append((left, right, mode))
    if alternating_background:
        for index, (left, right, _) in enumerate(clipped):
            ax.axvspan(
                left,
                right,
                facecolor="#F1F2F3" if index % 2 == 0 else "#FFFFFF",
                edgecolor="none",
                zorder=0,
            )
    for boundary in sorted(boundaries):
        if start_s < boundary < end_s:
            ax.axvline(
                boundary,
                color="#8A8A8A" if alternating_background else "#555555",
                linestyle=(0, (2, 3)) if alternating_background else (0, (3, 3)),
                linewidth=0.70 if alternating_background else 0.75,
                alpha=0.72 if alternating_background else 0.65,
                zorder=0.8 if alternating_background else 1,
            )
    transform = ax.get_xaxis_transform()
    for left, right, mode in clipped:
        segment_width = right - left
        if segment_width < 0.035 and not alternating_background:
            continue
        center = (left + right) / 2.0
        is_short_segment = segment_width < 0.18
        label_text = compact_phase_label(mode, segment_width) if alternating_background else short_phase_label(mode)
        if alternating_background and is_short_segment:
            bracket_y = 0.010
            leader_y = -0.012
            ax.plot(
                [left, right],
                [bracket_y, bracket_y],
                transform=transform,
                color="#666666",
                linestyle=(0, (2, 2)),
                linewidth=0.68,
                clip_on=False,
            )
            ax.plot(
                [center, center],
                [bracket_y, leader_y],
                transform=transform,
                color="#666666",
                linestyle=(0, (2, 2)),
                linewidth=0.68,
                clip_on=False,
            )
        ax.text(
            center,
            (-0.074 if is_short_segment else -0.118)
            if alternating_background
            else (-0.10 if is_short_segment else -0.18),
            label_text,
            transform=transform,
            ha="center",
            va="top",
            fontsize=8.2 if is_short_segment else (9.0 if alternating_background else 8.0),
            fontweight="medium" if alternating_background else "normal",
            color="#2E2E2E" if alternating_background else "#333333",
            rotation=52 if is_short_segment else 0,
            clip_on=False,
        )


def add_probe_bw_underbrace(
    ax,
    start_s: float,
    end_s: float,
    label: str,
    y: float = -0.205,
) -> None:
    """Draw a horizontal underbrace in data-x / axes-y coordinates."""
    width = end_s - start_s
    if width <= 0.0:
        return
    center = (start_s + end_s) / 2.0
    vertices = [
        (start_s, y),
        (start_s + 0.02 * width, y),
        (start_s + 0.02 * width, y - 0.045),
        (start_s + 0.07 * width, y - 0.045),
        (center - 0.07 * width, y - 0.045),
        (center - 0.02 * width, y - 0.045),
        (center - 0.02 * width, y - 0.10),
        (center, y - 0.10),
        (center + 0.02 * width, y - 0.10),
        (center + 0.02 * width, y - 0.045),
        (center + 0.07 * width, y - 0.045),
        (end_s - 0.07 * width, y - 0.045),
        (end_s - 0.02 * width, y - 0.045),
        (end_s - 0.02 * width, y),
        (end_s, y),
    ]
    codes = [
        MplPath.MOVETO,
        MplPath.CURVE4,
        MplPath.CURVE4,
        MplPath.CURVE4,
        MplPath.LINETO,
        MplPath.CURVE4,
        MplPath.CURVE4,
        MplPath.CURVE4,
        MplPath.CURVE4,
        MplPath.CURVE4,
        MplPath.CURVE4,
        MplPath.LINETO,
        MplPath.CURVE4,
        MplPath.CURVE4,
        MplPath.CURVE4,
    ]
    transform = ax.get_xaxis_transform()
    patch = PathPatch(
        MplPath(vertices, codes),
        transform=transform,
        fill=False,
        edgecolor="#333333",
        linewidth=1.15,
        capstyle="round",
        joinstyle="round",
        clip_on=False,
        zorder=7,
    )
    ax.add_patch(patch)
    ax.text(
        center,
        y - 0.13,
        label,
        transform=transform,
        ha="center",
        va="top",
        fontsize=10.5,
        fontweight="semibold",
        color="#222222",
        clip_on=False,
    )


def add_partial_probe_bw_underbrace(
    ax,
    start_s: float,
    end_s: float,
    label: str,
    y: float = -0.205,
) -> None:
    """Draw an open-ended underbrace for a ProbeBW cycle cut by the plot end."""
    width = end_s - start_s
    if width <= 0.0:
        return
    vertices = [
        (start_s, y),
        (start_s + 0.04 * width, y),
        (start_s + 0.04 * width, y - 0.045),
        (start_s + 0.14 * width, y - 0.045),
        (end_s - 0.06 * width, y - 0.045),
        (end_s, y - 0.045),
    ]
    codes = [
        MplPath.MOVETO,
        MplPath.CURVE4,
        MplPath.CURVE4,
        MplPath.CURVE4,
        MplPath.LINETO,
        MplPath.LINETO,
    ]
    transform = ax.get_xaxis_transform()
    ax.add_patch(
        PathPatch(
            MplPath(vertices, codes),
            transform=transform,
            fill=False,
            edgecolor="#333333",
            linewidth=1.15,
            capstyle="round",
            joinstyle="round",
            clip_on=False,
            zorder=7,
        )
    )
    center = (start_s + end_s) / 2.0
    ax.text(
        center,
        y - 0.13,
        label,
        transform=transform,
        ha="center",
        va="top",
        fontsize=10.5,
        fontweight="semibold",
        color="#222222",
        clip_on=False,
    )
    ax.text(
        end_s - 0.015 * width,
        y - 0.045,
        "...",
        transform=transform,
        ha="right",
        va="center",
        fontsize=10.0,
        color="#333333",
        clip_on=False,
    )


def add_congestion_signal_bracket(
    ax,
    start_s: float,
    end_s: float,
    label: str,
) -> None:
    if end_s <= start_s:
        return
    transform = ax.get_xaxis_transform()
    bracket_y = 0.82
    ax.annotate(
        "",
        xy=(end_s, bracket_y),
        xytext=(start_s, bracket_y),
        xycoords=transform,
        textcoords=transform,
        arrowprops={
            "arrowstyle": "|-|",
            "color": "#7A3300",
            "linewidth": 1.15,
            "shrinkA": 0,
            "shrinkB": 0,
        },
        annotation_clip=True,
        zorder=9,
    )
    ax.text(
        (start_s + end_s) / 2.0,
        bracket_y + 0.025,
        label,
        transform=transform,
        ha="center",
        va="bottom",
        fontsize=8.8,
        fontweight="semibold",
        color="#7A3300",
        zorder=9,
    )


def read_two_column_trace(path: Path, value_scale: float = 1.0) -> List[Tuple[float, float]]:
    rows: List[Tuple[float, float]] = []
    with path.open("r", encoding="utf-8", errors="replace") as fh:
        for line in fh:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            fields = re.split(r"\s+", line)
            if len(fields) < 2:
                continue
            try:
                rows.append((float(fields[0]), float(fields[1]) * value_scale))
            except ValueError:
                continue
    rows.sort(key=lambda item: item[0])
    return rows


def find_round_files(run_dir: Path) -> List[Path]:
    return sorted(run_dir.glob("*_flow*_*_cruise_best_full_load_window.csv"))


def load_round_bandwidth(run_dir: Path) -> Dict[int, List[Dict[str, object]]]:
    by_flow: Dict[int, List[Dict[str, object]]] = {}
    for path in find_round_files(run_dir):
        flow_id = flow_id_from_path(path)
        if flow_id <= 0:
            continue
        rows: List[Dict[str, object]] = []
        with path.open("r", encoding="utf-8", errors="replace", newline="") as fh:
            reader = csv.DictReader(fh)
            for raw in reader:
                selection_native_bps = to_float(raw.get("selection_native_bw_bps"))
                trusted_bps = to_float(raw.get("best_trusted_bw"))
                native_end_kbps = to_float(raw.get("cruise_end_native_bw_kbps"))
                fair_kbps = to_float(raw.get("fair_share_bandwidth_kbps"))
                rows.append(
                    {
                        "flow_id": flow_id,
                        "cruise_id": int(to_float(raw.get("cruise_id"), 0.0)),
                        "cruise_start_time": to_float(raw.get("cruise_start_time")),
                        "cruise_end_time": to_float(raw.get("cruise_end_time")),
                        "candidate_count": int(to_float(raw.get("candidate_count"), 0.0)),
                        "best_window_start_time": to_float(raw.get("best_window_start_time")),
                        "best_window_end_time": to_float(raw.get("best_window_end_time")),
                        "best_full_load_quality": to_float(raw.get("best_full_load_quality")),
                        "best_drate_mean_mbps": to_float(raw.get("best_drate_mean_kbps")) / 1000.0,
                        "maxbw_mbps": selection_native_bps / 1e6,
                        "native_end_mbps": native_end_kbps / 1000.0,
                        "trusted_bw_mbps": trusted_bps / 1e6,
                        "fair_share_mbps": fair_kbps / 1000.0,
                        "trusted_source": raw.get("best_trusted_bw_source", ""),
                        "spectral_pass": to_bool(raw.get("dual_signal_spectral_gate_pass")),
                    }
                )
        rows.sort(key=lambda item: (float(item["cruise_end_time"]), int(item["cruise_id"])))
        by_flow[flow_id] = rows
    return by_flow


def write_round_csv(output_dir: Path, by_flow: Dict[int, List[Dict[str, object]]]) -> Path:
    path = output_dir / "round_bandwidth_selection.csv"
    header = [
        "flow_id",
        "cruise_id",
        "cruise_start_time",
        "cruise_end_time",
        "candidate_count",
        "best_window_start_time",
        "best_window_end_time",
        "best_full_load_quality",
        "best_drate_mean_mbps",
        "maxbw_mbps",
        "native_end_mbps",
        "trusted_bw_mbps",
        "fair_share_mbps",
        "trusted_source",
        "spectral_pass",
    ]
    rows = []
    for flow_id in sorted(by_flow):
        for item in by_flow[flow_id]:
            rows.append([item.get(key, "") for key in header])
    write_csv(path, header, rows)
    return path


def plot_round_bandwidth_per_flow(
    output_dir: Path,
    run_dir: Path,
    by_flow: Dict[int, List[Dict[str, object]]],
) -> Optional[Path]:
    if not by_flow:
        return None
    fig, axes = plt.subplots(
        len(by_flow),
        1,
        figsize=(12.0, max(3.0, 2.55 * len(by_flow))),
        sharex=True,
        squeeze=False,
    )
    for ax, flow_id in zip(axes[:, 0], sorted(by_flow)):
        rows = by_flow[flow_id]
        times = [float(row["cruise_end_time"]) for row in rows]
        add_phase_background(ax, run_dir, flow_id, max(times) if times else None)
        ax.plot(times, [row["maxbw_mbps"] for row in rows], label="maxbw / native selection", color=LINE_COLORS["maxbw"], linewidth=1.5)
        ax.plot(times, [row["trusted_bw_mbps"] for row in rows], label="trustedBw", color=LINE_COLORS["trusted"], linewidth=1.5)
        ax.plot(times, [row["native_end_mbps"] for row in rows], label="cruise-end native", color=LINE_COLORS["native"], linewidth=1.0, linestyle=":")
        fair = finite_values(float(row["fair_share_mbps"]) for row in rows)
        if fair:
            ax.axhline(fair[0], color=LINE_COLORS["fair"], linestyle="--", linewidth=0.9, label="fair share")
        ax.set_ylabel(f"Flow {flow_id}\nMbps")
        ax.grid(True, alpha=0.25)
    add_phase_legend(axes[0, 0])
    axes[0, 0].legend(ncol=5, frameon=False, fontsize=8, loc="upper right")
    axes[-1, 0].set_xlabel("Time at cruise end (s)")
    fig.suptitle("FreqCCv4 per-cruise bandwidth actually selected", y=0.995)
    fig.tight_layout()
    path = output_dir / "round_bandwidth_per_flow.png"
    fig.savefig(path, dpi=180)
    plt.close(fig)
    return path


def plot_round_quality(output_dir: Path, by_flow: Dict[int, List[Dict[str, object]]]) -> Optional[Path]:
    if not by_flow:
        return None
    fig, axes = plt.subplots(2, 1, figsize=(12.0, 6.4), sharex=True)
    for flow_id in sorted(by_flow):
        rows = by_flow[flow_id]
        times = [float(row["cruise_end_time"]) for row in rows]
        axes[0].plot(times, [row["best_full_load_quality"] for row in rows], linewidth=1.2, label=f"flow{flow_id}")
        axes[1].plot(times, [row["candidate_count"] for row in rows], linewidth=1.2, label=f"flow{flow_id}")
    axes[0].set_ylabel("quality")
    axes[0].set_title("Best full-load window quality")
    axes[1].set_ylabel("windows")
    axes[1].set_title("Candidate window count")
    axes[1].set_xlabel("Time at cruise end (s)")
    for ax in axes:
        ax.grid(True, alpha=0.25)
        ax.legend(ncol=4, frameon=False, fontsize=8)
    fig.tight_layout()
    path = output_dir / "round_selection_quality.png"
    fig.savefig(path, dpi=180)
    plt.close(fig)
    return path


def plot_round_aggregate(
    output_dir: Path,
    run_dir: Path,
    by_flow: Dict[int, List[Dict[str, object]]],
    service_rate_bps: float,
    sample_step_s: float,
) -> Optional[Path]:
    if not by_flow:
        return None
    max_time = max(
        [0.0]
        + [
            float(row["cruise_end_time"])
            for rows in by_flow.values()
            for row in rows
            if math.isfinite(float(row["cruise_end_time"]))
        ]
    )
    grid = build_grid(max_time, sample_step_s)
    metric_map = {
        "sum_maxbw_mbps": "maxbw_mbps",
        "sum_trusted_bw_mbps": "trusted_bw_mbps",
        "sum_native_end_mbps": "native_end_mbps",
    }
    sampled_by_metric: Dict[str, List[float]] = {}
    active_counts: List[int] = [0 for _ in grid]
    for out_name, key in metric_map.items():
        total = [0.0 for _ in grid]
        counts = [0 for _ in grid]
        for rows in by_flow.values():
            series = [
                (float(row["cruise_end_time"]), float(row[key]))
                for row in rows
                if math.isfinite(float(row["cruise_end_time"])) and math.isfinite(float(row[key]))
            ]
            sampled = sample_previous(series, grid)
            for idx, value in enumerate(sampled):
                if math.isfinite(value):
                    total[idx] += value
                    counts[idx] += 1
        sampled_by_metric[out_name] = [total[idx] if counts[idx] else math.nan for idx in range(len(grid))]
        active_counts = [max(active_counts[idx], counts[idx]) for idx in range(len(grid))]

    csv_path = output_dir / "aggregate_round_bandwidth.csv"
    rows = []
    for idx, time_s in enumerate(grid):
        rows.append(
            [
                time_s,
                active_counts[idx],
                sampled_by_metric["sum_maxbw_mbps"][idx],
                sampled_by_metric["sum_trusted_bw_mbps"][idx],
                sampled_by_metric["sum_native_end_mbps"][idx],
            ]
        )
    write_csv(
        csv_path,
        [
            "time_s",
            "active_selection_count",
            "sum_maxbw_mbps",
            "sum_trusted_bw_mbps",
            "sum_native_end_mbps",
        ],
        rows,
    )

    fig, ax = plt.subplots(figsize=(12.0, 5.8))
    add_phase_background(ax, run_dir, first_flow_id_with_modes(run_dir), max_time)
    ax.plot(grid, sampled_by_metric["sum_maxbw_mbps"], label="sum maxbw/native selection", color=LINE_COLORS["maxbw"], linewidth=1.6)
    ax.plot(grid, sampled_by_metric["sum_trusted_bw_mbps"], label="sum trustedBw", color=LINE_COLORS["trusted"], linewidth=1.6)
    ax.plot(grid, sampled_by_metric["sum_native_end_mbps"], label="sum cruise-end native", color=LINE_COLORS["native"], linewidth=1.0, linestyle=":")
    ax.axhline(service_rate_bps / 1e6, color="#333333", linestyle="--", linewidth=1.0, label="bottleneck capacity")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Mbps")
    ax.set_title("FreqCCv4 aggregate bandwidth selection across flows")
    ax.grid(True, alpha=0.25)
    add_phase_legend(ax)
    ax.legend(ncol=2, frameon=False)
    fig.tight_layout()
    path = output_dir / "aggregate_round_bandwidth.png"
    fig.savefig(path, dpi=180)
    plt.close(fig)
    return path


def find_goodput_files(run_dir: Path) -> List[Path]:
    return sorted(run_dir.glob("*_flow*_*_good.txt"))


def plot_goodput(
    output_dir: Path,
    run_dir: Path,
    service_rate_bps: float,
    sample_step_s: float,
) -> Optional[Path]:
    files = find_goodput_files(run_dir)
    if not files:
        return None
    by_flow = {flow_id_from_path(path): read_two_column_trace(path, value_scale=1.0 / 1000.0) for path in files}
    by_flow = {flow_id: rows for flow_id, rows in by_flow.items() if flow_id > 0 and rows}
    if not by_flow:
        return None
    max_time = max(rows[-1][0] for rows in by_flow.values())
    grid = build_grid(max_time, sample_step_s)
    sampled = {flow_id: sample_previous(rows, grid, fill=0.0) for flow_id, rows in by_flow.items()}
    aggregate = [sum(values[idx] for values in sampled.values()) for idx in range(len(grid))]

    csv_rows = []
    flow_ids = sorted(sampled)
    for idx, time_s in enumerate(grid):
        csv_rows.append([time_s, aggregate[idx], *[sampled[flow_id][idx] for flow_id in flow_ids]])
    write_csv(
        output_dir / "goodput_timeseries.csv",
        ["time_s", "aggregate_goodput_mbps", *[f"flow{flow_id}_goodput_mbps" for flow_id in flow_ids]],
        csv_rows,
    )

    fig, axes = plt.subplots(2, 1, figsize=(12.0, 7.0), sharex=True)
    add_phase_background(axes[0], run_dir, first_flow_id_with_modes(run_dir), max_time)
    for flow_id in flow_ids:
        axes[0].plot(grid, sampled[flow_id], linewidth=1.0, label=f"flow{flow_id}")
    axes[0].set_ylabel("Mbps")
    axes[0].set_title("Per-flow goodput")
    add_phase_legend(axes[0])
    axes[0].legend(ncol=4, frameon=False, fontsize=8)
    add_phase_background(axes[1], run_dir, first_flow_id_with_modes(run_dir), max_time)
    axes[1].plot(grid, aggregate, color=LINE_COLORS["aggregate"], linewidth=1.4, label="aggregate")
    axes[1].axhline(service_rate_bps / 1e6, color="#555555", linestyle="--", linewidth=1.0, label="bottleneck capacity")
    axes[1].set_ylabel("Mbps")
    axes[1].set_xlabel("Time (s)")
    axes[1].set_title("Aggregate goodput")
    axes[1].legend(frameon=False)
    for ax in axes:
        ax.grid(True, alpha=0.25)
    fig.tight_layout()
    path = output_dir / "goodput_per_flow_and_aggregate.png"
    fig.savefig(path, dpi=180)
    plt.close(fig)
    return path


def find_queue_file(run_dir: Path) -> Optional[Path]:
    files = sorted(run_dir.glob("*_shared_bottleneck_queue.txt"))
    if files:
        return files[0]
    files = sorted(run_dir.glob("*_switch_egress_flow*_bottleneck_queue.txt"))
    return files[0] if files else None


def load_queue_bins(
    path: Path,
    service_rate_bps: float,
    sample_step_s: float,
) -> Tuple[List[float], List[float], List[float], List[float]]:
    bins: List[List[float]] = []
    with path.open("r", encoding="utf-8", errors="replace") as fh:
        for line in fh:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            fields = re.split(r"\s+", line)
            if len(fields) < 2:
                continue
            try:
                time_s = float(fields[0])
                queue_bytes = float(fields[1])
            except ValueError:
                continue
            idx = int(max(0.0, time_s) / sample_step_s)
            while len(bins) <= idx:
                bins.append([])
            bins[idx].append(queue_bytes * 8.0 / service_rate_bps * 1000.0)

    times: List[float] = []
    mean_values: List[float] = []
    p95_values: List[float] = []
    max_values: List[float] = []
    for idx, values in enumerate(bins):
        times.append(round((idx + 0.5) * sample_step_s, 10))
        if values:
            mean_values.append(sum(values) / len(values))
            p95_values.append(percentile(values, 95.0))
            max_values.append(max(values))
        else:
            mean_values.append(math.nan)
            p95_values.append(math.nan)
            max_values.append(math.nan)
    return times, mean_values, p95_values, max_values


def plot_queue_delay(
    output_dir: Path,
    run_dir: Path,
    service_rate_bps: float,
    sample_step_s: float,
) -> Optional[Path]:
    queue_file = find_queue_file(run_dir)
    if queue_file is None:
        return None
    times, mean_values, p95_values, max_values = load_queue_bins(queue_file, service_rate_bps, sample_step_s)
    write_csv(
        output_dir / "queue_delay_timeseries.csv",
        ["time_s", "mean_queue_delay_ms", "p95_queue_delay_ms", "max_queue_delay_ms"],
        zip(times, mean_values, p95_values, max_values),
    )
    fig, ax = plt.subplots(figsize=(12.0, 5.8))
    add_phase_background(ax, run_dir, first_flow_id_with_modes(run_dir), times[-1] if times else None)
    ax.plot(times, mean_values, label="mean", linewidth=1.3, color="#1f77b4")
    ax.plot(times, p95_values, label="p95", linewidth=1.2, color="#ff7f0e")
    ax.plot(times, max_values, label="max", linewidth=0.9, color="#d62728", alpha=0.85)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Queueing delay (ms)")
    ax.set_title("Shared bottleneck total queueing delay")
    ax.grid(True, alpha=0.25)
    add_phase_legend(ax)
    ax.legend(frameon=False)
    fig.tight_layout()
    path = output_dir / "queue_delay_total.png"
    fig.savefig(path, dpi=180)
    plt.close(fig)
    return path


def plot_queue_delay_per_flow(
    output_dir: Path,
    run_dir: Path,
    sample_step_s: float,
) -> Tuple[Optional[Path], Optional[Path]]:
    files = sorted(run_dir.glob("*_flow*_*_qdelay.txt"))
    by_flow = {
        flow_id_from_path(path): read_two_column_trace(path)
        for path in files
    }
    by_flow = {
        flow_id: rows
        for flow_id, rows in by_flow.items()
        if flow_id > 0 and rows
    }
    if not by_flow:
        return None, None

    max_time = max(rows[-1][0] for rows in by_flow.values())
    grid = build_grid(max_time, sample_step_s)
    sampled = {
        flow_id: sample_previous(rows, grid, fill=math.nan)
        for flow_id, rows in by_flow.items()
    }
    flow_ids = sorted(sampled)
    aggregate_mean: List[float] = []
    aggregate_p95: List[float] = []
    aggregate_max: List[float] = []
    csv_rows = []
    for idx, time_s in enumerate(grid):
        values = finite_values([sampled[flow_id][idx] for flow_id in flow_ids])
        mean_value = sum(values) / len(values) if values else math.nan
        p95_value = percentile(values, 95.0) if values else math.nan
        max_value = max(values) if values else math.nan
        aggregate_mean.append(mean_value)
        aggregate_p95.append(p95_value)
        aggregate_max.append(max_value)
        csv_rows.append(
            [time_s, mean_value, p95_value, max_value,
             *[sampled[flow_id][idx] for flow_id in flow_ids]]
        )
    write_csv(
        output_dir / "queue_delay_per_flow_timeseries.csv",
        ["time_s", "mean_across_flows_ms", "p95_across_flows_ms",
         "max_across_flows_ms",
         *[f"flow{flow_id}_queue_delay_ms" for flow_id in flow_ids]],
        csv_rows,
    )

    fig, axes = plt.subplots(
        len(flow_ids), 1,
        figsize=(12.0, max(4.0, 2.5 * len(flow_ids))),
        sharex=True,
        squeeze=False,
    )
    for ax, flow_id in zip(axes[:, 0], flow_ids):
        add_phase_background(ax, run_dir, flow_id, max_time)
        ax.plot(grid, sampled[flow_id], linewidth=1.0, color="#1f77b4")
        ax.set_ylabel(f"Flow {flow_id}\nms")
        ax.grid(True, alpha=0.25)
    axes[-1, 0].set_xlabel("Time (s)")
    fig.suptitle("FreqCCv4 queueing delay per flow", y=0.995)
    fig.tight_layout()
    per_flow_path = output_dir / "queue_delay_per_flow.png"
    fig.savefig(per_flow_path, dpi=180)
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(12.0, 5.8))
    add_phase_background(ax, run_dir, first_flow_id_with_modes(run_dir), max_time)
    ax.plot(grid, aggregate_mean, label="mean across flows", linewidth=1.3)
    ax.plot(grid, aggregate_p95, label="p95 across flows", linewidth=1.1)
    ax.plot(grid, aggregate_max, label="max across flows", linewidth=0.9)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Queueing delay (ms)")
    ax.set_title("Aggregate FreqCCv4 per-flow queueing delay")
    ax.grid(True, alpha=0.25)
    ax.legend(frameon=False)
    fig.tight_layout()
    aggregate_path = output_dir / "queue_delay_aggregate_flows.png"
    fig.savefig(aggregate_path, dpi=180)
    plt.close(fig)
    return per_flow_path, aggregate_path


def find_mode_files(run_dir: Path) -> List[Path]:
    return sorted(run_dir.glob("*_flow*_*_bbrmode.txt"))


def load_modes(path: Path) -> List[Tuple[float, str]]:
    rows: List[Tuple[float, str]] = []
    with path.open("r", encoding="utf-8", errors="replace") as fh:
        for line in fh:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            fields = re.split(r"\s+", line)
            if len(fields) < 2:
                continue
            try:
                rows.append((float(fields[0]), fields[1]))
            except ValueError:
                continue
    rows.sort(key=lambda item: item[0])
    return rows


def plot_mode_timeline(output_dir: Path, run_dir: Path, max_time_s: float) -> Optional[Path]:
    files = find_mode_files(run_dir)
    by_flow = {flow_id_from_path(path): load_modes(path) for path in files}
    by_flow = {flow_id: rows for flow_id, rows in by_flow.items() if flow_id > 0 and rows}
    if not by_flow:
        return None
    if max_time_s <= 0.0:
        max_time_s = max(rows[-1][0] for rows in by_flow.values())
    seen_modes = []
    fig, ax = plt.subplots(figsize=(12.0, 1.2 + 0.65 * len(by_flow)))
    for lane_idx, flow_id in enumerate(sorted(by_flow), start=1):
        rows = by_flow[flow_id]
        for idx, (start, mode) in enumerate(rows):
            end = rows[idx + 1][0] if idx + 1 < len(rows) else max_time_s
            if end <= start:
                continue
            ax.broken_barh(
                [(start, end - start)],
                (lane_idx - 0.35, 0.7),
                facecolors=MODE_COLORS.get(mode, "#bbbbbb"),
                edgecolors="none",
            )
            if mode not in seen_modes:
                seen_modes.append(mode)
    handles = [
        plt.Line2D([0], [0], color=MODE_COLORS.get(mode, "#bbbbbb"), linewidth=6, label=mode)
        for mode in seen_modes
    ]
    ax.set_yticks(list(range(1, len(by_flow) + 1)))
    ax.set_yticklabels([f"flow{flow_id}" for flow_id in sorted(by_flow)])
    ax.set_xlim(0.0, max_time_s)
    ax.set_xlabel("Time (s)")
    ax.set_title("FreqCCv4 mode timeline")
    ax.grid(True, axis="x", alpha=0.25)
    ax.legend(handles=handles, ncol=min(4, max(1, len(handles))), frameon=False, fontsize=8, loc="upper right")
    fig.tight_layout()
    path = output_dir / "bbr_mode_timeline.png"
    fig.savefig(path, dpi=180)
    plt.close(fig)
    return path


def find_gate_files(run_dir: Path) -> List[Path]:
    return sorted(run_dir.glob("flow*_freq_gate_trace.csv"))


def load_gate_pacing(path: Path, sample_step_s: float) -> List[Dict[str, float]]:
    rows: List[Dict[str, float]] = []
    last_emit = -math.inf
    with path.open("r", encoding="utf-8", errors="replace", newline="") as fh:
        reader = csv.DictReader(fh)
        for raw in reader:
            if raw.get("row_type") != "pacing":
                continue
            time_s = to_float(raw.get("time"))
            if not math.isfinite(time_s):
                continue
            if time_s - last_emit < sample_step_s:
                continue
            last_emit = time_s
            rows.append(
                {
                    "time_s": time_s,
                    "native_pacing_mbps": to_float(raw.get("native_pacing_bps")) / 1e6,
                    "final_pacing_mbps": to_float(raw.get("final_pacing_rate_bps")) / 1e6,
                    "b_native_mbps": to_float(raw.get("current_native_bw_bps")) / 1e6,
                    "pacing_base_bw_mbps": to_float(raw.get("pacing_base_bw_bps")) / 1e6,
                    "trusted_bw_mbps": to_float(raw.get("trusted_bw_bps")) / 1e6,
                }
            )
    return rows


def find_sendrate_files(run_dir: Path) -> List[Path]:
    return sorted(run_dir.glob("*_flow*_*_sendrate.txt"))


def plot_pacing_from_gate(
    output_dir: Path,
    run_dir: Path,
    service_rate_bps: float,
    sample_step_s: float,
) -> Tuple[Optional[Path], Optional[Path]]:
    gate_files = find_gate_files(run_dir)
    by_flow: Dict[int, List[Dict[str, float]]] = {}
    for path in gate_files:
        flow_id = flow_id_from_path(path)
        rows = load_gate_pacing(path, sample_step_s)
        if flow_id > 0 and rows:
            by_flow[flow_id] = rows
    if not by_flow:
        return None, None

    csv_rows = []
    for flow_id in sorted(by_flow):
        for row in by_flow[flow_id]:
            csv_rows.append(
                [
                    flow_id,
                    row["time_s"],
                    row["native_pacing_mbps"],
                    row["final_pacing_mbps"],
                    row["b_native_mbps"],
                    row["pacing_base_bw_mbps"],
                    row["trusted_bw_mbps"],
                ]
            )
    write_csv(
        output_dir / "pacing_rate_timeseries.csv",
        [
            "flow_id",
            "time_s",
            "native_pacing_mbps",
            "final_pacing_mbps",
            "b_native_mbps",
            "pacing_base_bw_mbps",
            "trusted_bw_mbps",
        ],
        csv_rows,
    )

    fig, axes = plt.subplots(
        len(by_flow),
        1,
        figsize=(12.0, max(3.0, 2.35 * len(by_flow))),
        sharex=True,
        squeeze=False,
    )
    for ax, flow_id in zip(axes[:, 0], sorted(by_flow)):
        rows = by_flow[flow_id]
        times = [row["time_s"] for row in rows]
        add_phase_background(ax, run_dir, flow_id, max(times) if times else None)
        ax.plot(times, [row["final_pacing_mbps"] for row in rows], label="final pacing", color=LINE_COLORS["final_pacing"], linewidth=1.0)
        ax.plot(times, [row["native_pacing_mbps"] for row in rows], label="native pacing", color=LINE_COLORS["native_pacing"], linewidth=0.9, linestyle="--")
        ax.set_ylabel(f"Flow {flow_id}\nMbps")
        ax.grid(True, alpha=0.25)
    add_phase_legend(axes[0, 0])
    axes[0, 0].legend(ncol=2, frameon=False, fontsize=8, loc="upper right")
    axes[-1, 0].set_xlabel("Time (s)")
    fig.suptitle("FreqCCv4 pacing rate from freq gate trace", y=0.995)
    fig.tight_layout()
    per_flow_path = output_dir / "pacing_rate_per_flow.png"
    fig.savefig(per_flow_path, dpi=180)
    plt.close(fig)

    max_time = max(row["time_s"] for rows in by_flow.values() for row in rows)
    grid = build_grid(max_time, sample_step_s)
    aggregate = [0.0 for _ in grid]
    for rows in by_flow.values():
        series = [(row["time_s"], row["final_pacing_mbps"]) for row in rows]
        sampled = sample_previous(series, grid, fill=0.0)
        aggregate = [aggregate[idx] + sampled[idx] for idx in range(len(grid))]
    fig, ax = plt.subplots(figsize=(12.0, 5.6))
    add_phase_background(ax, run_dir, first_flow_id_with_modes(run_dir), max_time)
    ax.plot(grid, aggregate, color=LINE_COLORS["aggregate"], linewidth=1.2, label="sum final pacing")
    ax.axhline(service_rate_bps / 1e6, color="#555555", linestyle="--", linewidth=1.0, label="bottleneck capacity")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Mbps")
    ax.set_title("Aggregate FreqCCv4 pacing rate")
    ax.grid(True, alpha=0.25)
    add_phase_legend(ax)
    ax.legend(frameon=False)
    fig.tight_layout()
    aggregate_path = output_dir / "pacing_rate_aggregate.png"
    fig.savefig(aggregate_path, dpi=180)
    plt.close(fig)
    return per_flow_path, aggregate_path


def load_gate_trusted_bw(path: Path) -> List[Dict[str, float]]:
    """Read trusted_bw updates and instantaneous pacing-base from gate trace.

    Returns one row per emitted round/pacing sample, with the following fields:

        time_s            : event time (s)
        trusted_bw_mbps   : last selected trusted_bw (0 if invalid)
        trusted_valid     : bool, whether trusted_bw was valid at this moment
        trusted_source    : str, "NORMAL" / "MERGED" / "NONE" / other
        trusted_conf      : float in [0,1], selection confidence
        trusted_app_valid : bool, whether trusted_bw is allowed to drive pacing
        trusted_app_phase : str, e.g. "CRUISE" / "POST_CRUISE" / "NONE"
        trusted_cruise_id : int, last cruise the trusted_bw was selected on
        pacing_base_mbps  : the bw actually used as pacing base this instant
        native_bw_mbps    : current Native BBR detected bw
        sample_type       : "round" or "pacing"
    """
    rows: List[Dict[str, object]] = []
    with path.open("r", encoding="utf-8", errors="replace", newline="") as fh:
        reader = csv.DictReader(fh)
        for raw in reader:
            time_s = to_float(raw.get("time"))
            if not math.isfinite(time_s):
                continue
            rows.append(
                {
                    "time_s": time_s,
                    "trusted_bw_mbps": to_float(raw.get("trusted_bw_bps")) / 1e6,
                    "trusted_valid": to_bool(raw.get("trusted_bw_valid")),
                    "trusted_source": str(raw.get("trusted_bw_source") or "NONE").strip(),
                    "trusted_conf": to_float(raw.get("trusted_bw_conf")),
                    "trusted_app_valid": to_bool(raw.get("trusted_bw_application_valid")),
                    "trusted_app_phase": str(raw.get("trusted_bw_application_phase") or "NONE").strip(),
                    "trusted_cruise_id": int(to_float(raw.get("trusted_bw_cruise_id"), 0.0)),
                    "pacing_base_mbps": to_float(raw.get("pacing_base_bw_bps")) / 1e6,
                    "native_bw_mbps": to_float(raw.get("current_native_bw_bps")) / 1e6,
                    "sample_type": str(raw.get("row_type") or "").strip(),
                }
            )
    rows.sort(key=lambda r: float(r["time_s"]))  # type: ignore[arg-type]
    return rows  # type: ignore[return-value]


def _trusted_source_color(source: str) -> str:
    s = source.upper()
    if s == "TIME_WAVEFORM_SRTT_SEARCH":
        return TRUSTEDBW_STATE_COLOR
    if s == "NORMAL":
        return "#1f77b4"
    if s == "MERGED":
        return "#ff7f0e"
    if s == "FALLBACK":
        return "#7f7f7f"
    return "#bbbbbb"


def plot_trusted_bw_per_flow(
    output_dir: Path,
    run_dir: Path,
    service_rate_bps: float,
) -> Tuple[Optional[Path], Optional[Path]]:
    """Per-flow trusted_bw selection timeline + aggregate trusted_bw.

    Top panel series (per flow):
        * trusted_bw (NORMAL)       : solid line, blue
        * trusted_bw (MERGED)       : solid line, orange
        * trusted_bw (other source) : solid line, gray
        * trusted_bw_valid window   : pale green shading where trusted_bw is valid
        * pacing_base_bw            : thin black line (what is actually used)
        * current_native_bw         : thin dashed purple line (BBR2 detected)
    Bottom aggregate panel: sum of trusted_bw across all flows, with bottleneck
    capacity reference line and shaded post-cruise application windows.

    The figure makes it obvious when a cruise successfully produced a trusted_bw
    estimate, whether the rescue (MERGED) path was used, and how the trusted_bw
    tracks the native detected bw during cruise vs. pacing-base after cruise.
    """
    gate_files = find_gate_files(run_dir)
    by_flow: Dict[int, List[Dict[str, float]]] = {}
    for path in gate_files:
        flow_id = flow_id_from_path(path)
        rows = load_gate_trusted_bw(path)
        if flow_id > 0 and rows:
            by_flow[flow_id] = rows
    if not by_flow:
        return None, None

    # Round-event rows drive the trusted_bw step; pacing-event rows only
    # contribute pacing_base_mbps (which can change intra-round when
    # pacing_base_source switches back to NATIVE_BBR).
    def split(rows: List[Dict[str, float]]):
        rounds = [r for r in rows if r["sample_type"] == "round"]
        pacings = [r for r in rows if r["sample_type"] == "pacing"]
        return rounds, pacings

    csv_rows = []
    for flow_id in sorted(by_flow):
        for row in by_flow[flow_id]:
            csv_rows.append(
                [
                    flow_id,
                    row["time_s"],
                    row["sample_type"],
                    row["trusted_bw_mbps"],
                    row["trusted_valid"],
                    row["trusted_source"],
                    row["trusted_conf"],
                    row["trusted_app_valid"],
                    row["trusted_app_phase"],
                    row["trusted_cruise_id"],
                    row["pacing_base_mbps"],
                    row["native_bw_mbps"],
                ]
            )
    write_csv(
        output_dir / "trusted_bw_timeseries.csv",
        [
            "flow_id",
            "time_s",
            "sample_type",
            "trusted_bw_mbps",
            "trusted_valid",
            "trusted_source",
            "trusted_conf",
            "trusted_app_valid",
            "trusted_app_phase",
            "trusted_cruise_id",
            "pacing_base_mbps",
            "native_bw_mbps",
        ],
        csv_rows,
    )

    fig, axes = plt.subplots(
        len(by_flow),
        1,
        figsize=(12.0, max(3.0, 2.4 * len(by_flow))),
        sharex=True,
        squeeze=False,
    )
    for ax, flow_id in zip(axes[:, 0], sorted(by_flow)):
        rows = by_flow[flow_id]
        rounds, pacings = split(rows)
        max_time = max(row["time_s"] for row in rows)

        # Shaded band where trusted_bw is valid.
        in_band = False
        band_start = 0.0
        for row in rounds + pacings:
            t = row["time_s"]
            v = bool(row["trusted_valid"])
            if v and not in_band:
                in_band = True
                band_start = t
            elif not v and in_band:
                ax.axvspan(
                    band_start,
                    t,
                    color="#2ca02c",
                    alpha=0.07,
                    linewidth=0,
                    zorder=0,
                )
                in_band = False
        if in_band:
            ax.axvspan(
                band_start,
                max_time,
                color="#2ca02c",
                alpha=0.07,
                linewidth=0,
                zorder=0,
            )

        # Step-trace the trusted_bw by source on round events.
        if rounds:
            t_round = [r["time_s"] for r in rounds]
            t_round_step: List[float] = []
            v_round_step: List[float] = []
            for r in rounds:
                if r["trusted_valid"]:
                    t_round_step.append(r["time_s"])
                    v_round_step.append(r["trusted_bw_mbps"])
            ax.plot(
                t_round_step,
                v_round_step,
                color="#1f77b4",
                linewidth=0.0,
                marker="o",
                markersize=2.2,
                alpha=0.55,
                label="_nolegend_",
            )
            # Per-source segment lines: connect two adjacent valid points only
            # when they share the same source, so the colour encodes the
            # source without fabricating cross-source segments.
            sources = sorted({
                str(r["trusted_source"]).upper()
                for r in rounds
                if r["trusted_valid"] and str(r["trusted_source"]).upper() != "NONE"
            })
            for source in sources:
                seg_t: List[float] = []
                seg_v: List[float] = []
                for r in rounds:
                    if not r["trusted_valid"]:
                        continue
                    if str(r["trusted_source"]).upper() != source:
                        continue
                    if seg_t and seg_t[-1] == r["time_s"]:
                        seg_v[-1] = r["trusted_bw_mbps"]
                    else:
                        seg_t.append(r["time_s"])
                        seg_v.append(r["trusted_bw_mbps"])
                if seg_t:
                    ax.plot(
                        seg_t,
                        seg_v,
                        color=_trusted_source_color(source),
                        linewidth=1.4,
                        marker="o",
                        markersize=2.5,
                        label=f"trusted_bw [{source}]",
                    )

        # pacing_base line (the bw actually driving pacing).
        if pacings:
            t_p = [r["time_s"] for r in pacings]
            v_p = [r["pacing_base_mbps"] for r in pacings]
            ax.plot(
                t_p,
                v_p,
                color="#111111",
                linewidth=0.7,
                alpha=0.85,
                label="pacing_base",
            )

        # current_native_bw line (BBR2 detected max bw).
        if rounds:
            t_n = [r["time_s"] for r in rounds]
            v_n = [r["native_bw_mbps"] for r in rounds]
            ax.plot(
                t_n,
                v_n,
                color=LINE_COLORS["native"],
                linewidth=0.7,
                linestyle="--",
                alpha=0.55,
                label="native bw",
            )

        ax.set_ylabel(f"Flow {flow_id}\nMbps")
        ax.grid(True, alpha=0.25)
        if service_rate_bps > 0:
            ax.axhline(
                service_rate_bps / 1e6 / max(1, len(by_flow)),
                color="#888888",
                linewidth=0.6,
                linestyle=":",
                alpha=0.6,
            )

    handles, labels = axes[0, 0].get_legend_handles_labels()
    handles.append(Patch(facecolor="#2ca02c", alpha=0.30, label="trusted_bw valid"))
    handles.append(Patch(facecolor="#888888", alpha=0.6, label="fair share"))
    axes[0, 0].legend(handles, labels + ["trusted_bw valid", "fair share"], ncol=3, frameon=False, fontsize=8, loc="upper right")
    axes[-1, 0].set_xlabel("Time (s)")
    fig.suptitle("FreqCCv4 trusted_bw per flow", y=0.995)
    fig.tight_layout()
    per_flow_path = output_dir / "trusted_bw_per_flow.png"
    fig.savefig(per_flow_path, dpi=180)
    plt.close(fig)

    # Aggregate trusted_bw across flows (sample-previous on grid).
    max_time = max(row["time_s"] for rows in by_flow.values() for row in rows)
    grid = build_grid(max_time, 0.1)
    aggregate = [0.0 for _ in grid]
    aggregate_valid_frac = [0.0 for _ in grid]
    for rows in by_flow.values():
        rounds = [r for r in rows if r["sample_type"] == "round"]
        series = [
            (r["time_s"], r["trusted_bw_mbps"])
            for r in rounds
            if r["trusted_valid"]
        ]
        sampled = sample_previous(series, grid, fill=0.0)
        aggregate = [aggregate[idx] + sampled[idx] for idx in range(len(grid))]

        frac_series = [
            (r["time_s"], 1.0) for r in rounds if r["trusted_valid"]
        ]
        sampled_frac = sample_previous(frac_series, grid, fill=0.0)
        aggregate_valid_frac = [
            aggregate_valid_frac[idx] + sampled_frac[idx]
            for idx in range(len(grid))
        ]
    aggregate_valid_frac = [
        v / max(1, len(by_flow)) for v in aggregate_valid_frac
    ]

    fig, host = plt.subplots(figsize=(12.0, 5.6))
    add_phase_background(host, run_dir, first_flow_id_with_modes(run_dir), max_time)
    host.plot(grid, aggregate, color=LINE_COLORS["trusted"], linewidth=1.4, label="sum trusted_bw (valid only)")
    host.axhline(service_rate_bps / 1e6, color="#555555", linestyle="--", linewidth=1.0, label="bottleneck capacity")
    host.set_xlabel("Time (s)")
    host.set_ylabel("Mbps")
    host.set_title("Aggregate FreqCCv4 trusted_bw")
    host.grid(True, alpha=0.25)
    host.legend(loc="upper left", frameon=False)

    frac_ax = host.twinx()
    frac_ax.plot(
        grid,
        aggregate_valid_frac,
        color="#2ca02c",
        linewidth=0.9,
        linestyle=":",
        alpha=0.7,
        label="fraction of flows with valid trusted_bw",
    )
    frac_ax.set_ylabel("valid-fraction", color="#2ca02c")
    frac_ax.set_ylim(0.0, 1.05)
    frac_ax.tick_params(axis="y", labelcolor="#2ca02c")
    h2, l2 = frac_ax.get_legend_handles_labels()
    host.legend(h2 + host.get_legend_handles_labels()[0], l2 + host.get_legend_handles_labels()[1], loc="upper right", frameon=False)

    fig.tight_layout()
    aggregate_path = output_dir / "trusted_bw_aggregate.png"
    fig.savefig(aggregate_path, dpi=180)
    plt.close(fig)
    return per_flow_path, aggregate_path


def plot_pacing_from_sendrate(
    output_dir: Path,
    run_dir: Path,
    service_rate_bps: float,
    sample_step_s: float,
) -> Tuple[Optional[Path], Optional[Path]]:
    files = find_sendrate_files(run_dir)
    by_flow = {flow_id_from_path(path): read_two_column_trace(path, value_scale=1.0 / 1000.0) for path in files}
    by_flow = {flow_id: rows for flow_id, rows in by_flow.items() if flow_id > 0 and rows}
    if not by_flow:
        return None, None

    csv_rows = []
    for flow_id in sorted(by_flow):
        for time_s, value in by_flow[flow_id]:
            csv_rows.append([flow_id, time_s, value])
    write_csv(output_dir / "pacing_rate_timeseries.csv", ["flow_id", "time_s", "pacing_rate_mbps"], csv_rows)

    fig, axes = plt.subplots(
        len(by_flow),
        1,
        figsize=(12.0, max(3.0, 2.35 * len(by_flow))),
        sharex=True,
        squeeze=False,
    )
    for ax, flow_id in zip(axes[:, 0], sorted(by_flow)):
        rows = by_flow[flow_id]
        add_phase_background(ax, run_dir, flow_id, rows[-1][0] if rows else None)
        ax.plot([time_s for time_s, _ in rows], [value for _, value in rows], linewidth=1.0, color=LINE_COLORS["final_pacing"], label="pacing")
        ax.set_ylabel(f"Flow {flow_id}\nMbps")
        ax.grid(True, alpha=0.25)
    add_phase_legend(axes[0, 0])
    axes[0, 0].legend(frameon=False, fontsize=8, loc="upper right")
    axes[-1, 0].set_xlabel("Time (s)")
    fig.suptitle("FreqCCv4 pacing rate from sendrate trace", y=0.995)
    fig.tight_layout()
    per_flow_path = output_dir / "pacing_rate_per_flow.png"
    fig.savefig(per_flow_path, dpi=180)
    plt.close(fig)

    max_time = max(rows[-1][0] for rows in by_flow.values())
    grid = build_grid(max_time, sample_step_s)
    aggregate = [0.0 for _ in grid]
    for rows in by_flow.values():
        sampled = sample_previous(rows, grid, fill=0.0)
        aggregate = [aggregate[idx] + sampled[idx] for idx in range(len(grid))]
    fig, ax = plt.subplots(figsize=(12.0, 5.6))
    add_phase_background(ax, run_dir, first_flow_id_with_modes(run_dir), max_time)
    ax.plot(grid, aggregate, color=LINE_COLORS["aggregate"], linewidth=1.2, label="sum pacing")
    ax.axhline(service_rate_bps / 1e6, color="#555555", linestyle="--", linewidth=1.0, label="bottleneck capacity")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Mbps")
    ax.set_title("Aggregate FreqCCv4 pacing rate")
    ax.grid(True, alpha=0.25)
    add_phase_legend(ax)
    ax.legend(frameon=False)
    fig.tight_layout()
    aggregate_path = output_dir / "pacing_rate_aggregate.png"
    fig.savefig(aggregate_path, dpi=180)
    plt.close(fig)
    return per_flow_path, aggregate_path


def plot_pacing_proxy(
    output_dir: Path,
    run_dir: Path,
    by_flow: Dict[int, List[Dict[str, object]]],
) -> Optional[Path]:
    note = (
        "No true pacing-rate trace was found in this run.\n\n"
        "Expected files:\n"
        "  - flow*_freq_gate_trace.csv with row_type=pacing, or\n"
        "  - *_flow*_*_sendrate.txt\n\n"
        "For the same experiment behavior plus pacing debug trace, rerun with:\n"
        "  --enableConvergenceGateTrace=true --gateTraceMode=sampled_pacing "
        "--gateTraceSampleIntervalUs=10000\n\n"
        "This script will automatically plot native_pacing/final_pacing when those files exist.\n"
    )
    (output_dir / "pacing_trace_missing.txt").write_text(note, encoding="utf-8")
    if not by_flow:
        return None

    fig, axes = plt.subplots(
        len(by_flow),
        1,
        figsize=(12.0, max(3.0, 2.35 * len(by_flow))),
        sharex=True,
        squeeze=False,
    )
    for ax, flow_id in zip(axes[:, 0], sorted(by_flow)):
        rows = by_flow[flow_id]
        times = [float(row["cruise_end_time"]) for row in rows]
        add_phase_background(ax, run_dir, flow_id, max(times) if times else None)
        ax.plot(times, [row["trusted_bw_mbps"] for row in rows], label="TrustedBw", color=LINE_COLORS["effective"], linewidth=1.4)
        ax.plot(times, [row["maxbw_mbps"] for row in rows], label="maxbw/native selection", color=LINE_COLORS["maxbw"], linewidth=1.1, linestyle="--")
        ax.set_ylabel(f"Flow {flow_id}\nMbps")
        ax.grid(True, alpha=0.25)
    add_phase_legend(axes[0, 0])
    axes[0, 0].legend(ncol=2, frameon=False, fontsize=8, loc="upper right")
    axes[-1, 0].set_xlabel("Time at cruise end (s)")
    fig.suptitle("Pacing trace missing: showing per-cruise TrustedBw", y=0.995)
    fig.tight_layout()
    path = output_dir / "trusted_bw_proxy_per_flow.png"
    fig.savefig(path, dpi=180)
    plt.close(fig)
    return path


def load_gate_delivery_rate(path: Path, end_s: float) -> List[Tuple[float, float]]:
    rows: List[Tuple[float, float]] = []
    with path.open("r", encoding="utf-8", errors="replace", newline="") as fh:
        reader = csv.DictReader(fh)
        for raw in reader:
            if raw.get("row_type") != "pacing":
                continue
            time_s = to_float(raw.get("time"))
            if not math.isfinite(time_s) or time_s > end_s:
                continue
            delivery_rate_bps = to_float(raw.get("current_delivery_rate"))
            if not math.isfinite(delivery_rate_bps):
                continue
            rows.append((time_s, delivery_rate_bps / 1e6))
    rows.sort(key=lambda item: item[0])
    return rows


def plot_delivery_rate_first_window(
    output_dir: Path,
    run_dir: Path,
    end_s: float = 10.0,
) -> Optional[Path]:
    gate_files = find_gate_files(run_dir)
    by_flow: Dict[int, List[Tuple[float, float]]] = {}
    for path in gate_files:
        flow_id = flow_id_from_path(path)
        rows = load_gate_delivery_rate(path, end_s)
        if flow_id > 0 and rows:
            by_flow[flow_id] = rows
    if not by_flow:
        (output_dir / "delivery_rate_trace_missing.txt").write_text(
            "No delivery-rate trace was found in this run.\n\n"
            "Expected files:\n"
            "  - flow*_freq_gate_trace.csv with row_type=pacing and current_delivery_rate\n\n"
            "Rerun with --enableConvergenceGateTrace=true --gateTraceMode=sampled_pacing.\n",
            encoding="utf-8",
        )
        return None

    csv_rows = []
    for flow_id in sorted(by_flow):
        for time_s, value in by_flow[flow_id]:
            csv_rows.append([flow_id, time_s, value])
    write_csv(
        output_dir / "delivery_rate_first10_timeseries.csv",
        ["flow_id", "time_s", "delivery_rate_mbps"],
        csv_rows,
    )

    fig, axes = plt.subplots(
        len(by_flow),
        1,
        figsize=(12.0, max(4.0, 2.75 * len(by_flow))),
        sharex=True,
        squeeze=False,
    )
    for ax, flow_id in zip(axes[:, 0], sorted(by_flow)):
        rows = by_flow[flow_id]
        add_phase_separators_with_labels(ax, run_dir, flow_id, 0.0, end_s)
        ax.scatter(
            [time_s for time_s, _ in rows],
            [value for _, value in rows],
            color="#1f77b4",
            s=7,
            alpha=0.58,
            linewidths=0,
            label="delivery rate",
            zorder=3,
            rasterized=True,
        )
        ax.set_xlim(0.0, end_s)
        ax.set_ylabel(f"Flow {flow_id}\nMbps")
        ax.grid(True, axis="y", alpha=0.25)
        ax.grid(True, axis="x", alpha=0.08)
    axes[0, 0].legend(frameon=False, fontsize=8, loc="upper right")
    axes[-1, 0].set_xlabel("Time (s)")
    fig.suptitle("FreqCCv4 delivery rate in the first 10 seconds", y=0.995)
    fig.subplots_adjust(left=0.08, right=0.99, top=0.96, bottom=0.08, hspace=0.42)
    path = output_dir / "delivery_rate_first10_by_flow.png"
    fig.savefig(path, dpi=180)
    plt.close(fig)
    return path


def plot_delivery_rate_per_flow_and_aggregate(
    output_dir: Path,
    run_dir: Path,
    service_rate_bps: float,
    sample_step_s: float,
) -> Tuple[Optional[Path], Optional[Path]]:
    by_flow: Dict[int, List[Tuple[float, float]]] = {}
    for path in find_gate_files(run_dir):
        flow_id = flow_id_from_path(path)
        rows = load_gate_delivery_rate(path, math.inf)
        if flow_id > 0 and rows:
            by_flow[flow_id] = rows
    if not by_flow:
        return None, None

    max_time = max(rows[-1][0] for rows in by_flow.values())
    grid = build_grid(max_time, sample_step_s)
    sampled = {
        flow_id: sample_previous(rows, grid, fill=0.0)
        for flow_id, rows in by_flow.items()
    }
    flow_ids = sorted(sampled)
    aggregate = [
        sum(sampled[flow_id][idx] for flow_id in flow_ids)
        for idx in range(len(grid))
    ]
    write_csv(
        output_dir / "delivery_rate_timeseries.csv",
        ["time_s", "aggregate_delivery_rate_mbps",
         *[f"flow{flow_id}_delivery_rate_mbps" for flow_id in flow_ids]],
        ([time_s, aggregate[idx],
          *[sampled[flow_id][idx] for flow_id in flow_ids]]
         for idx, time_s in enumerate(grid)),
    )

    fig, axes = plt.subplots(
        len(flow_ids), 1,
        figsize=(12.0, max(4.0, 2.5 * len(flow_ids))),
        sharex=True,
        squeeze=False,
    )
    for ax, flow_id in zip(axes[:, 0], flow_ids):
        add_phase_background(ax, run_dir, flow_id, max_time)
        ax.plot(grid, sampled[flow_id], linewidth=1.0, color="#1f77b4")
        ax.axhline(service_rate_bps / 1e6 / len(flow_ids),
                   color="#777777", linestyle="--", linewidth=0.8)
        ax.set_ylabel(f"Flow {flow_id}\nMbps")
        ax.grid(True, alpha=0.25)
    axes[-1, 0].set_xlabel("Time (s)")
    fig.suptitle("FreqCCv4 Delivery Rate per flow", y=0.995)
    fig.tight_layout()
    per_flow_path = output_dir / "delivery_rate_per_flow.png"
    fig.savefig(per_flow_path, dpi=180)
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(12.0, 5.8))
    add_phase_background(ax, run_dir, first_flow_id_with_modes(run_dir), max_time)
    ax.plot(grid, aggregate, color=LINE_COLORS["aggregate"], linewidth=1.3,
            label="sum Delivery Rate")
    ax.axhline(service_rate_bps / 1e6, color="#555555", linestyle="--",
               linewidth=1.0, label="bottleneck capacity")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Mbps")
    ax.set_title("Aggregate FreqCCv4 Delivery Rate")
    ax.grid(True, alpha=0.25)
    ax.legend(frameon=False)
    fig.tight_layout()
    aggregate_path = output_dir / "delivery_rate_aggregate.png"
    fig.savefig(aggregate_path, dpi=180)
    plt.close(fig)
    return per_flow_path, aggregate_path


def find_raw_delivery_files(run_dir: Path) -> List[Path]:
    return sorted(run_dir.glob("*_flow*_*_recvrate_raw.txt"))


def time_token(value: float) -> str:
    if abs(value - round(value)) < 1e-9:
        return str(int(round(value)))
    return f"{value:g}".replace(".", "p")


def delivery_window_slug(start_s: float, end_s: float) -> str:
    if start_s <= 0.0:
        return f"first{time_token(end_s)}"
    return f"{time_token(start_s)}to{time_token(end_s)}"


def time_label(value: float) -> str:
    if abs(value - round(value)) < 1e-9:
        return str(int(round(value)))
    return f"{value:g}"


def delivery_window_title(start_s: float, end_s: float) -> str:
    if start_s <= 0.0:
        return f"in the first {time_label(end_s)} seconds"
    return f"from {time_label(start_s)} to {time_label(end_s)} seconds"


def delivery_compare_file_stem(
    start_s: float,
    end_s: float,
    selected_flow_ids: Optional[Sequence[int]] = None,
    highlight_start_s: Optional[float] = None,
    highlight_end_s: Optional[float] = None,
    highlight_windows: Optional[Sequence[Tuple[float, float]]] = None,
) -> str:
    stem = f"delivery_rate_bbrv2_vs_freqccv4_{delivery_window_slug(start_s, end_s)}"
    if selected_flow_ids:
        suffix = "_".join(f"flow{flow_id}" for flow_id in selected_flow_ids)
        stem = f"{stem}_{suffix}"
    if highlight_start_s is not None and highlight_end_s is not None:
        stem = f"{stem}_highlight{delivery_window_slug(highlight_start_s, highlight_end_s)}"
    if highlight_windows:
        windows_slug = "_".join(
            delivery_window_slug(window_start, window_end)
            for window_start, window_end in highlight_windows
        )
        stem = f"{stem}_highlights{windows_slug}"
    return stem


def load_raw_delivery_rate(path: Path, start_s: float, end_s: float) -> List[Tuple[float, float]]:
    rows: List[Tuple[float, float]] = []
    with path.open("r", encoding="utf-8", errors="replace") as fh:
        for line in fh:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            fields = re.split(r"\s+", line)
            if len(fields) < 2:
                continue
            try:
                time_s = float(fields[0])
                value_mbps = float(fields[1]) / 1000.0
            except ValueError:
                continue
            if start_s <= time_s <= end_s:
                rows.append((time_s, value_mbps))
    rows.sort(key=lambda item: item[0])
    return rows


def add_delivery_rate_highlight(
    ax,
    rows: Sequence[Tuple[float, float]],
    start_s: float,
    end_s: float,
    label_prefix: str,
    selected_value_mbps: Optional[float] = None,
    compact_label: bool = False,
) -> Optional[float]:
    """Emphasize one sampling window and return its selected TrustedBw value."""
    if end_s <= start_s:
        return None
    window_rows = [
        (time_s, value)
        for time_s, value in rows
        if start_s <= time_s <= end_s and math.isfinite(value)
    ]
    if not window_rows:
        return None

    values = [value for _, value in window_rows]
    mean_value = (
        selected_value_mbps
        if selected_value_mbps is not None and math.isfinite(selected_value_mbps)
        else sum(values) / len(values)
    )
    y_min = min(values)
    y_max = max(values)
    y_span = max(0.5, y_max - y_min)
    y_pad = max(0.45, y_span * 0.16)
    y_bottom = y_min - y_pad
    y_top = y_max + y_pad

    highlight = Rectangle(
        (start_s, y_bottom),
        end_s - start_s,
        y_top - y_bottom,
        facecolor=(0.902, 0.624, 0.0, 0.10),
        edgecolor=WINDOW_EDGE_COLOR,
        linewidth=1.15,
        zorder=1.2,
        label="Frequency-domain sampling window",
    )
    ax.add_patch(highlight)

    # The amber rectangle denotes the observation interval; teal denotes the
    # value selected from that interval and later committed at CRUISE exit.
    ax.hlines(
        mean_value,
        start_s,
        end_s,
        colors=TRUSTEDBW_STATE_COLOR,
        linestyles=(0, (4.0, 2.4)),
        linewidth=1.65,
        zorder=5,
        label="_nolegend_",
    )

    label_x = start_s + (end_s - start_s) * 0.04
    label_y = y_bottom - max(0.28, y_span * 0.06)
    if compact_label:
        annotation_text = f"{label_prefix} = {mean_value:.2f} Mbps"
    else:
        annotation_detail = (
            "selected from the frequency-domain\nsampling window"
            if selected_value_mbps is not None and math.isfinite(selected_value_mbps)
            else "mean delivery rate within the\nfrequency-domain sampling window"
        )
        annotation_text = f"{label_prefix} = {mean_value:.2f} Mbps\n{annotation_detail}"
    ax.annotate(
        annotation_text,
        xy=(start_s + (end_s - start_s) * 0.92, mean_value),
        xytext=(label_x, label_y),
        textcoords="data",
        ha="left",
        va="top",
        fontsize=9.0,
        fontweight="semibold",
        linespacing=0.95,
        color="#222222",
        zorder=6,
        arrowprops={
            "arrowstyle": "-",
            "color": TRUSTEDBW_STATE_COLOR,
            "linewidth": 1.00,
            "shrinkA": 4,
            "shrinkB": 3,
        },
        bbox={
            "boxstyle": "square,pad=0.20",
            "facecolor": "white",
            "edgecolor": TRUSTEDBW_STATE_COLOR,
            "linewidth": 0.80,
            "alpha": 0.97,
        },
    )
    return mean_value

def phase_starts_for_flow(run_dir: Path, flow_id: int, target_phase: str) -> List[float]:
    """Return transition times at which a flow enters the requested phase."""
    mode_file = next(
        (path for path in find_mode_files(run_dir) if flow_id_from_path(path) == flow_id),
        None,
    )
    if mode_file is None:
        return []

    starts: List[float] = []
    previous_key: Optional[str] = None
    for time_s, mode in load_modes(mode_file):
        key = phase_key(mode)
        if key == target_phase and previous_key != target_phase:
            starts.append(time_s)
        previous_key = key
    return starts


def complete_probe_bw_cycles_in_window(
    run_dir: Path,
    flow_id: int,
    start_s: float,
    end_s: float,
    cycle_phase: str,
    limit: int,
) -> List[Tuple[float, float]]:
    """Find complete phase-to-next-same-phase ProbeBW cycles in the plot."""
    phase_starts = phase_starts_for_flow(run_dir, flow_id, cycle_phase)
    tolerance = 1e-9
    cycles: List[Tuple[float, float]] = []
    for cycle_start, cycle_end in zip(phase_starts, phase_starts[1:]):
        if cycle_start >= start_s - tolerance and cycle_end <= end_s + tolerance:
            cycles.append((cycle_start, cycle_end))
            if len(cycles) >= limit:
                break
    return cycles


def bbrv2_cycle_max_point(
    rows: Sequence[Tuple[float, float]],
    cycle_start_s: float,
    cycle_end_s: float,
) -> Optional[Tuple[float, float]]:
    """Return the time/value of the maximum delivery-rate sample in one cycle."""
    cycle_rows = [
        (time_s, value)
        for time_s, value in rows
        if cycle_start_s <= time_s < cycle_end_s and math.isfinite(value)
    ]
    return max(cycle_rows, key=lambda item: item[1]) if cycle_rows else None


def cruise_end_for_sampling_window(
    run_dir: Path,
    flow_id: int,
    window_start_s: float,
    window_end_s: float,
    plot_end_s: float,
) -> Optional[float]:
    """Map a sampling window to the CRUISE exit at which its value is committed."""
    cruise_segments = [
        (start, end)
        for start, end, mode in mode_segments_for_flow(run_dir, flow_id, plot_end_s)
        if phase_key(mode) == "probeBW_cruise"
    ]
    if not cruise_segments:
        return None

    midpoint = (window_start_s + window_end_s) / 2.0
    containing = [
        (start, end)
        for start, end in cruise_segments
        if start - 1e-9 <= midpoint <= end + 1e-9
    ]
    if containing:
        return min(containing, key=lambda item: item[1] - item[0])[1]

    overlaps = []
    for start, end in cruise_segments:
        overlap = max(0.0, min(end, window_end_s) - max(start, window_start_s))
        if overlap > 0.0:
            overlaps.append((overlap, end))
    if not overlaps:
        return None
    return max(overlaps, key=lambda item: item[0])[1]


def add_bandwidth_state_trajectories(
    ax,
    start_s: float,
    initial_state_value: float,
    maxbw_points: Sequence[Tuple[float, float]],
    trusted_updates: Sequence[Tuple[float, float, float, float]],
    end_s: float,
) -> None:
    """Draw event-driven maxBw and CRUISE-exit TrustedBw state trajectories.

    ``initial_state_value`` is the pre-existing state already in force at the
    left plot boundary (conceptually the previous round's maxBw).  Both the
    maxBw and TrustedBw trajectories start from this common value on the y-axis.

    maxbw_points contains ``(peak_time, peak_value)`` pairs. trusted_updates
    contains ``(window_start, window_end, cruise_end, trusted_value)`` tuples.
    """
    if not math.isfinite(initial_state_value):
        return

    ordered_max = sorted(
        {(float(time_s), float(value)) for time_s, value in maxbw_points if math.isfinite(time_s) and math.isfinite(value)},
        key=lambda item: item[0],
    )

    visible_max = [
        (time_s, value)
        for time_s, value in ordered_max
        if start_s - 1e-9 <= time_s <= end_s + 1e-9
    ]
    x_gap = max(0.018, (end_s - start_s) * 0.006)
    y_gap = 0.38
    current_time = start_s
    current_value = initial_state_value
    legend_pending = True
    for time_s, value in visible_max:
        if time_s > current_time:
            ax.plot(
                [current_time, time_s],
                [current_value, current_value],
                color=MAXBW_STATE_COLOR,
                linewidth=2.55,
                solid_capstyle="butt",
                label="maxBw" if legend_pending else "_nolegend_",
                zorder=4.10,
            )
            legend_pending = False
        delta = value - current_value
        if abs(delta) > y_gap:
            vertical_end = value - math.copysign(y_gap, delta)
            ax.plot(
                [time_s, time_s],
                [current_value, vertical_end],
                color=MAXBW_STATE_COLOR,
                linewidth=2.55,
                solid_capstyle="butt",
                label="_nolegend_",
                zorder=4.10,
            )
        current_time = min(end_s, time_s + x_gap)
        current_value = value
    if end_s > current_time:
        ax.plot(
            [current_time, end_s],
            [current_value, current_value],
            color=MAXBW_STATE_COLOR,
            linewidth=2.55,
            solid_capstyle="butt",
            label="maxBw" if legend_pending else "_nolegend_",
            zorder=4.10,
        )

    ordered_updates = sorted(
        [
            (window_start, window_end, update_time, value)
            for window_start, window_end, update_time, value in trusted_updates
            if math.isfinite(update_time)
            and math.isfinite(value)
            and update_time >= start_s - 1e-9
            and update_time <= end_s + 1e-9
        ],
        key=lambda item: item[2],
    )

    trusted_times = [start_s]
    trusted_values = [initial_state_value]
    for _, _, update_time, value in ordered_updates:
        if abs(update_time - trusted_times[-1]) <= 1e-9:
            trusted_values[-1] = value
        else:
            trusted_times.append(update_time)
            trusted_values.append(value)
    if end_s > trusted_times[-1]:
        trusted_times.append(end_s)
        trusted_values.append(trusted_values[-1])

    ax.step(
        trusted_times,
        trusted_values,
        where="post",
        color=TRUSTEDBW_STATE_COLOR,
        linewidth=1.45,
        solid_capstyle="butt",
        solid_joinstyle="miter",
        label="TrustedBw",
        zorder=4.25,
    )

    for window_start, window_end, update_time, value in ordered_updates:
        guide_start = min(max(window_end, window_start), update_time)
        if update_time - guide_start > 1e-9:
            ax.plot(
                [guide_start, update_time],
                [value, value],
                color=TRUSTEDBW_STATE_COLOR,
                linewidth=1.25,
                linestyle=(0, (2.4, 2.2)),
                alpha=0.82,
                label="_nolegend_",
                zorder=4.18,
            )
        ax.scatter(
            [update_time],
            [value],
            s=38,
            marker="o",
            facecolor="white",
            edgecolor=TRUSTEDBW_STATE_COLOR,
            linewidth=1.25,
            zorder=7.2,
            label="_nolegend_",
        )


def add_bbrv2_maxbw_marker(
    ax,
    rows: Sequence[Tuple[float, float]],
    cycle_start_s: float,
    cycle_end_s: float,
    label_prefix: str = "maxBw",
    label_suffix: str = "",
) -> Optional[Tuple[float, float]]:
    """Mark and label the peak sample that triggers a maxBw update."""
    point = bbrv2_cycle_max_point(rows, cycle_start_s, cycle_end_s)
    if point is None:
        return None
    max_time_s, max_value = point

    ax.scatter(
        [max_time_s],
        [max_value],
        s=118,
        marker="D",
        facecolor="none",
        edgecolor=MAXBW_STATE_COLOR,
        linewidth=1.45,
        zorder=7.8,
        label="_nolegend_",
    )

    all_values = [value for _, value in rows if math.isfinite(value)]
    value_floor = min(all_values) if all_values else max_value
    value_ceiling = max(all_values) if all_values else max_value
    all_times = [time_s for time_s, value in rows if math.isfinite(value)]
    time_span = (
        max(1e-12, max(all_times) - min(all_times))
        if all_times
        else cycle_end_s - cycle_start_s
    )
    value_span = max(0.5, value_ceiling - value_floor) if all_values else 1.0
    place_right = (
        max_time_s <= (min(all_times) + max(all_times)) / 2.0 if all_times else True
    )
    label_x = max_time_s + (0.07 * time_span if place_right else -0.07 * time_span)
    label_y = max_value + max(0.65, 0.07 * value_span)

    ax.annotate(
        f"{label_prefix} {max_value:.2f} Mbps{label_suffix}",
        xy=(max_time_s, max_value),
        xytext=(label_x, label_y),
        textcoords="data",
        ha="left" if place_right else "right",
        va="bottom",
        fontsize=9.8,
        fontweight="semibold",
        color="#222222",
        zorder=9,
        arrowprops={
            "arrowstyle": "-",
            "color": MAXBW_STATE_COLOR,
            "linewidth": 1.00,
            "shrinkA": 3,
            "shrinkB": 10,
        },
        bbox={
            "boxstyle": "square,pad=0.20",
            "facecolor": "white",
            "edgecolor": MAXBW_STATE_COLOR,
            "linewidth": 0.80,
            "alpha": 0.97,
        },
        annotation_clip=True,
    )
    return point

def _deduplicated_legend_entries(ax) -> Tuple[List[object], List[str]]:
    handles, labels = ax.get_legend_handles_labels()
    unique: Dict[str, object] = {}
    for handle, label in zip(handles, labels):
        if label and not label.startswith("_") and label not in unique:
            unique[label] = handle
    return list(unique.values()), list(unique.keys())


def plot_bbrv2_vs_freqccv4_delivery_rate(
    output_dir: Path,
    freqccv4_run_dir: Path,
    service_rate_bps: float,
    end_s: float = 5.0,
    start_s: float = 0.0,
    selected_flow_ids: Optional[Sequence[int]] = None,
    highlight_start_s: Optional[float] = None,
    highlight_end_s: Optional[float] = None,
    highlight_cc: str = "FreqCCv4",
    highlight_label_prefix: str = "TrustedBw",
    highlight_windows: Optional[Sequence[Tuple[float, float]]] = None,
    highlight_values_mbps: Optional[Sequence[float]] = None,
    show_bbrv2_maxbw: bool = True,
    comparison_cc_label: str = "FreqCCv4",
    maxbw_cycle_phase: str = "probeBW_refill",
    maxbw_cycle_count: int = 1,
    initial_state_value_mbps: Optional[float] = None,
    relative_time_axis: bool = False,
    show_probe_cycle_braces: bool = False,
    show_partial_probe_cycle_brace: bool = False,
    include_partial_maxbw_cycle: bool = False,
    congestion_signal_window: Optional[Tuple[float, float]] = None,
    congestion_signal_label: str = "Packet-loss signal",
    comparison_raw_file: Optional[Path] = None,
    scenario_label: str = "",
    synthetic_start_s: Optional[float] = None,
) -> Optional[Path]:
    # scenario_label and synthetic_start_s are retained for CLI backward
    # compatibility. This figure intentionally renders neither a title/subtitle
    # nor a synthetic-region overlay.
    requested_flow_ids = sorted(set(selected_flow_ids or []))
    all_highlight_windows: List[Tuple[float, float]] = []
    all_highlight_values: List[Optional[float]] = []
    if highlight_start_s is not None and highlight_end_s is not None:
        all_highlight_windows.append((highlight_start_s, highlight_end_s))
        all_highlight_values.append(None)
    for window_idx, window in enumerate(highlight_windows or []):
        all_highlight_windows.append(window)
        value = (
            highlight_values_mbps[window_idx]
            if highlight_values_mbps is not None
            and window_idx < len(highlight_values_mbps)
            else None
        )
        all_highlight_values.append(value)
    stem = delivery_compare_file_stem(
        start_s,
        end_s,
        requested_flow_ids,
        highlight_windows=all_highlight_windows,
    )
    if comparison_raw_file is not None:
        stem = f"{stem}_synthetic"
    root = freqccv4_run_dir.parent
    runs = {
        "BBRv2": root / "BBRv2",
        comparison_cc_label: freqccv4_run_dir,
    }
    by_cc_flow: Dict[str, Dict[int, List[Tuple[float, float]]]] = {}
    for cc, run_dir in runs.items():
        by_flow: Dict[int, List[Tuple[float, float]]] = {}
        paths = (
            [comparison_raw_file]
            if cc == comparison_cc_label and comparison_raw_file is not None
            else find_raw_delivery_files(run_dir)
        )
        for path in paths:
            flow_id = flow_id_from_path(path)
            rows = load_raw_delivery_rate(path, start_s, end_s)
            if flow_id > 0 and rows:
                by_flow[flow_id] = rows
        by_cc_flow[cc] = by_flow
    all_flow_ids = sorted(set(by_cc_flow["BBRv2"]) | set(by_cc_flow[comparison_cc_label]))
    flow_ids = [flow_id for flow_id in all_flow_ids if not requested_flow_ids or flow_id in requested_flow_ids]
    if not flow_ids or not by_cc_flow["BBRv2"] or not by_cc_flow[comparison_cc_label]:
        (output_dir / "delivery_rate_bbrv2_vs_freqccv4_missing.txt").write_text(
            "Cannot plot BBRv2 vs FreqCCv4 delivery-rate comparison.\n\n"
            "Expected files in both BBRv2 and FreqCCv4 run directories:\n"
            "  - *_flow*_*_recvrate_raw.txt\n\n"
            "Rerun missing CCs with --enableHeavyTrace=true.\n",
            encoding="utf-8",
        )
        return None

    csv_rows = []
    for cc in ("BBRv2", comparison_cc_label):
        for flow_id in flow_ids:
            for time_s, value in by_cc_flow[cc].get(flow_id, []):
                csv_rows.append([cc, flow_id, time_s, value])
    write_csv(
        output_dir / f"{stem}.csv",
        ["cc", "flow_id", "time_s", "delivery_rate_mbps"],
        csv_rows,
    )

    fair_share_mbps = service_rate_bps / 1e6 / max(1, len(all_flow_ids))
    fig, axes = plt.subplots(
        len(flow_ids),
        1,
        figsize=(12.4, max(5.1, 3.05 * len(flow_ids))),
        sharex=True,
        squeeze=False,
    )
    sample_styles = {
        "BBRv2": {"color": "#0072B2", "marker": "o", "size": 8.5, "alpha": 0.70},
        comparison_cc_label: {"color": "#D55E00", "marker": "s", "size": 7.0, "alpha": 0.62},
    }
    for ax, flow_id in zip(axes[:, 0], flow_ids):
        add_phase_separators_with_labels(
            ax,
            freqccv4_run_dir,
            flow_id,
            start_s,
            end_s,
            alternating_background=True,
        )

        trusted_window_results: List[Tuple[float, float, float]] = []
        if all_highlight_windows:
            highlight_cc_key = (
                comparison_cc_label if highlight_cc == "FreqCCv4" else highlight_cc
            )
            for window_idx, ((window_start, window_end), selected_value_mbps) in enumerate(
                zip(all_highlight_windows, all_highlight_values), start=1
            ):
                clipped_start = max(start_s, window_start)
                clipped_end = min(end_s, window_end)
                label_prefix = (
                    highlight_label_prefix
                    if len(all_highlight_windows) == 1
                    else f"{highlight_label_prefix} {window_idx}"
                )
                selected_value = add_delivery_rate_highlight(
                    ax,
                    by_cc_flow.get(highlight_cc_key, {}).get(flow_id, []),
                    clipped_start,
                    clipped_end,
                    label_prefix,
                    selected_value_mbps,
                    relative_time_axis,
                )
                if selected_value is not None and math.isfinite(selected_value):
                    trusted_window_results.append(
                        (clipped_start, clipped_end, selected_value)
                    )

        for cc in ("BBRv2", comparison_cc_label):
            rows = by_cc_flow[cc].get(flow_id, [])
            if not rows:
                continue
            style = sample_styles[cc]
            ax.scatter(
                [time_s for time_s, _ in rows],
                [value for _, value in rows],
                color=style["color"],
                marker=style["marker"],
                s=style["size"],
                alpha=style["alpha"],
                linewidths=0,
                label=f"{cc} delivery-rate",
                zorder=3 if cc == comparison_cc_label else 2,
                rasterized=True,
            )

        if show_bbrv2_maxbw:
            complete_cycles = complete_probe_bw_cycles_in_window(
                runs["BBRv2"],
                flow_id,
                start_s,
                end_s,
                maxbw_cycle_phase,
                maxbw_cycle_count,
            )
            cycle_intervals: List[Tuple[float, float]] = list(complete_cycles)

            if include_partial_maxbw_cycle:
                bbrv2_phase_starts = phase_starts_for_flow(
                    runs["BBRv2"], flow_id, maxbw_cycle_phase
                )
                partial_start = next(
                    (
                        cycle_start
                        for cycle_start, cycle_end in zip(
                            bbrv2_phase_starts, bbrv2_phase_starts[1:]
                        )
                        if cycle_start >= start_s
                        and cycle_start < end_s
                        and cycle_end > end_s
                    ),
                    None,
                )
                if partial_start is not None:
                    cycle_intervals.append((partial_start, end_s))

            maxbw_points = [
                point
                for cycle_start, cycle_end in cycle_intervals
                for point in [
                    bbrv2_cycle_max_point(
                        by_cc_flow["BBRv2"].get(flow_id, []),
                        cycle_start,
                        cycle_end,
                    )
                ]
                if point is not None
            ]

            trusted_updates: List[Tuple[float, float, float, float]] = []
            for window_start, window_end, trusted_value in trusted_window_results:
                update_time = cruise_end_for_sampling_window(
                    freqccv4_run_dir,
                    flow_id,
                    window_start,
                    window_end,
                    end_s,
                )
                if update_time is not None:
                    trusted_updates.append(
                        (window_start, window_end, update_time, trusted_value)
                    )

            initial_state_value = (
                initial_state_value_mbps
                if initial_state_value_mbps is not None
                and math.isfinite(initial_state_value_mbps)
                else next(
                    (
                        value
                        for time_s, value in by_cc_flow["BBRv2"].get(flow_id, [])
                        if math.isfinite(time_s)
                        and math.isfinite(value)
                        and time_s >= start_s - 1e-9
                    ),
                    maxbw_points[0][1] if maxbw_points else math.nan,
                )
            )

            add_bandwidth_state_trajectories(
                ax,
                start_s,
                initial_state_value,
                maxbw_points,
                trusted_updates,
                end_s,
            )

            signal_label_suffix = (
                f" ({congestion_signal_label.strip().lower()})"
                if congestion_signal_window is not None
                and congestion_signal_label.strip()
                else ""
            )
            total_cycles = len(cycle_intervals)
            for cycle_idx, (cycle_start, cycle_end) in enumerate(
                cycle_intervals, start=1
            ):
                label_prefix = (
                    "maxBw"
                    if total_cycles == 1
                    else f"Cycle {cycle_idx} maxBw"
                )
                add_bbrv2_maxbw_marker(
                    ax,
                    by_cc_flow["BBRv2"].get(flow_id, []),
                    cycle_start,
                    cycle_end,
                    label_prefix,
                    signal_label_suffix if cycle_idx == 3 else "",
                )

        ax.axhline(
            fair_share_mbps,
            color="#4A4A4A",
            linestyle=(0, (5, 3.5)),
            linewidth=1.10,
            alpha=0.88,
            label=f"Fair share: {fair_share_mbps:.1f} Mbps",
            zorder=1.8,
        )
        ax.set_xlim(start_s, end_s)
        ax.set_ylabel("Rate (Mbps)")
        if relative_time_axis:
            relative_tick_count = int(math.floor(end_s - start_s))
            relative_ticks = [start_s + offset for offset in range(relative_tick_count + 1)]
            relative_labels = [
                r"$t$" if offset == 0 else rf"$t+{offset}$"
                for offset in range(relative_tick_count + 1)
            ]
            ax.set_xticks(relative_ticks, relative_labels)
        if show_probe_cycle_braces:
            probe_cycles = complete_probe_bw_cycles_in_window(
                freqccv4_run_dir,
                flow_id,
                start_s,
                end_s,
                "probeBW_up",
                2,
            )
            for cycle_idx, (cycle_start, cycle_end) in enumerate(probe_cycles, start=1):
                add_probe_bw_underbrace(
                    ax,
                    cycle_start,
                    cycle_end,
                    f"ProbeBW {cycle_idx}",
                )
        if show_partial_probe_cycle_brace:
            freqcc_up_starts = phase_starts_for_flow(
                freqccv4_run_dir, flow_id, "probeBW_up"
            )
            partial_start = next(
                (
                    cycle_start
                    for cycle_start, cycle_end in zip(
                        freqcc_up_starts, freqcc_up_starts[1:]
                    )
                    if cycle_start >= start_s
                    and cycle_start < end_s
                    and cycle_end > end_s
                ),
                None,
            )
            if partial_start is not None:
                add_partial_probe_bw_underbrace(
                    ax,
                    partial_start,
                    end_s,
                    "ProbeBW 3",
                )
        ax.grid(False)
        ax.tick_params(direction="out", length=4.0, width=0.80, color="#333333")
        ax.spines["left"].set_color("#333333")
        ax.spines["bottom"].set_color("#333333")
        ax.spines["top"].set_visible(False)
        ax.spines["right"].set_visible(False)

    handles, labels = _deduplicated_legend_entries(axes[0, 0])
    legend_columns = max(1, len(labels))
    fig.legend(
        handles,
        labels,
        ncol=legend_columns,
        loc="upper center",
        bbox_to_anchor=(0.5, 0.985),
        frameon=False,
        columnspacing=1.15,
        handlelength=2.2,
        handletextpad=0.55,
        markerscale=1.35,
        fontsize=9.6,
    )

    single_flow = len(flow_ids) == 1
    fig.supxlabel("Time (s)", y=0.018, fontsize=12)
    fig.subplots_adjust(
        left=0.095,
        right=0.985,
        top=0.86 if single_flow else 0.90,
        bottom=0.23 if single_flow and show_probe_cycle_braces else (0.21 if single_flow else 0.14),
        hspace=0.66,
    )
    path = output_dir / f"{stem}.png"
    fig.savefig(path, dpi=300, bbox_inches="tight")
    plt.close(fig)
    return path

def find_rtt_files(run_dir: Path) -> List[Path]:
    return sorted(run_dir.glob("*_flow*_*_rtt.txt"))


def load_rtt_trace(path: Path, sample_step_s: float) -> List[Dict[str, float]]:
    rows: List[Dict[str, float]] = []
    last_emit = -math.inf
    with path.open("r", encoding="utf-8", errors="replace") as fh:
        for line in fh:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            fields = re.split(r"\s+", line)
            if len(fields) < 4:
                continue
            try:
                time_s = float(fields[0])
                latest_rtt_ms = float(fields[2])
                smoothed_rtt_ms = float(fields[3])
            except ValueError:
                continue
            if time_s - last_emit < sample_step_s:
                continue
            last_emit = time_s
            rows.append(
                {
                    "time_s": time_s,
                    "latest_rtt_ms": latest_rtt_ms,
                    "smoothed_rtt_ms": smoothed_rtt_ms,
                }
            )
    return rows


def plot_rtt(
    output_dir: Path,
    run_dir: Path,
    sample_step_s: float,
) -> Tuple[Optional[Path], Optional[Path]]:
    files = find_rtt_files(run_dir)
    by_flow: Dict[int, List[Dict[str, float]]] = {}
    for path in files:
        flow_id = flow_id_from_path(path)
        rows = load_rtt_trace(path, sample_step_s)
        if flow_id > 0 and rows:
            by_flow[flow_id] = rows
    if not by_flow:
        (output_dir / "srtt_trace_missing.txt").write_text(
            "No RTT/SRTT trace was found in this run.\n\n"
            "Expected files:\n"
            "  - *_flow*_*_rtt.txt\n\n"
            "Rerun with --enableHeavyTrace=true to emit RTT/SRTT traces.\n",
            encoding="utf-8",
        )
        return None, None

    csv_rows = []
    for flow_id in sorted(by_flow):
        for row in by_flow[flow_id]:
            csv_rows.append(
                [
                    flow_id,
                    row["time_s"],
                    row["latest_rtt_ms"],
                    row["smoothed_rtt_ms"],
                ]
            )
    write_csv(
        output_dir / "srtt_timeseries.csv",
        ["flow_id", "time_s", "latest_rtt_ms", "smoothed_rtt_ms"],
        csv_rows,
    )

    fig, axes = plt.subplots(
        len(by_flow),
        1,
        figsize=(12.0, max(3.0, 2.35 * len(by_flow))),
        sharex=True,
        squeeze=False,
    )
    for ax, flow_id in zip(axes[:, 0], sorted(by_flow)):
        rows = by_flow[flow_id]
        times = [row["time_s"] for row in rows]
        add_phase_background(ax, run_dir, flow_id, max(times) if times else None)
        ax.plot(
            times,
            [row["smoothed_rtt_ms"] for row in rows],
            label="smoothed RTT",
            color=LINE_COLORS["smoothed_rtt"],
            linewidth=1.1,
        )
        ax.plot(
            times,
            [row["latest_rtt_ms"] for row in rows],
            label="latest RTT",
            color=LINE_COLORS["latest_rtt"],
            linewidth=0.7,
            alpha=0.5,
        )
        ax.set_ylabel(f"Flow {flow_id}\nms")
        ax.grid(True, alpha=0.25)
    add_phase_legend(axes[0, 0])
    axes[0, 0].legend(ncol=2, frameon=False, fontsize=8, loc="upper right")
    axes[-1, 0].set_xlabel("Time (s)")
    fig.suptitle("FreqCCv4 latest RTT and smoothed RTT", y=0.995)
    fig.tight_layout()
    per_flow_path = output_dir / "srtt_per_flow.png"
    fig.savefig(per_flow_path, dpi=180)
    plt.close(fig)

    max_time = max(row["time_s"] for rows in by_flow.values() for row in rows)
    grid = build_grid(max_time, sample_step_s)
    mean_srtt: List[float] = []
    p95_latest: List[float] = []
    sampled_srtt = {
        flow_id: sample_previous(
            [(row["time_s"], row["smoothed_rtt_ms"]) for row in rows],
            grid,
        )
        for flow_id, rows in by_flow.items()
    }
    sampled_latest = {
        flow_id: sample_previous(
            [(row["time_s"], row["latest_rtt_ms"]) for row in rows],
            grid,
        )
        for flow_id, rows in by_flow.items()
    }
    for idx in range(len(grid)):
        srtt_values = finite_values(values[idx] for values in sampled_srtt.values())
        latest_values = finite_values(values[idx] for values in sampled_latest.values())
        mean_srtt.append(sum(srtt_values) / len(srtt_values) if srtt_values else math.nan)
        p95_latest.append(percentile(latest_values, 95.0))
    write_csv(
        output_dir / "srtt_aggregate_timeseries.csv",
        ["time_s", "mean_smoothed_rtt_ms", "p95_latest_rtt_ms"],
        zip(grid, mean_srtt, p95_latest),
    )

    fig, ax = plt.subplots(figsize=(12.0, 5.6))
    add_phase_background(ax, run_dir, first_flow_id_with_modes(run_dir), max_time)
    ax.plot(grid, mean_srtt, color=LINE_COLORS["smoothed_rtt"], linewidth=1.3, label="mean smoothed RTT")
    ax.plot(grid, p95_latest, color=LINE_COLORS["latest_rtt"], linewidth=1.0, alpha=0.75, label="p95 latest RTT")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("RTT (ms)")
    ax.set_title("Aggregate FreqCCv4 RTT view")
    ax.grid(True, alpha=0.25)
    add_phase_legend(ax)
    ax.legend(frameon=False)
    fig.tight_layout()
    aggregate_path = output_dir / "srtt_aggregate.png"
    fig.savefig(aggregate_path, dpi=180)
    plt.close(fig)
    return per_flow_path, aggregate_path


def summarize(
    output_dir: Path,
    service_rate_bps: float,
    warmup_s: float,
) -> Optional[Path]:
    queue_csv = output_dir / "queue_delay_timeseries.csv"
    goodput_csv = output_dir / "goodput_timeseries.csv"
    srtt_csv = output_dir / "srtt_aggregate_timeseries.csv"
    rows: List[List[object]] = [["metric", "value"]]
    rows.append(["service_rate_mbps", service_rate_bps / 1e6])
    rows.append(["warmup_s", warmup_s])
    if queue_csv.exists():
        mean_vals: List[float] = []
        p95_vals: List[float] = []
        max_vals: List[float] = []
        with queue_csv.open("r", encoding="utf-8", newline="") as fh:
            reader = csv.DictReader(fh)
            for row in reader:
                if to_float(row.get("time_s")) < warmup_s:
                    continue
                mean_vals.append(to_float(row.get("mean_queue_delay_ms")))
                p95_vals.append(to_float(row.get("p95_queue_delay_ms")))
                max_vals.append(to_float(row.get("max_queue_delay_ms")))
        rows.append(["avg_mean_queue_delay_ms_after_warmup", sum(finite_values(mean_vals)) / max(1, len(finite_values(mean_vals)))])
        rows.append(["avg_p95_queue_delay_ms_after_warmup", sum(finite_values(p95_vals)) / max(1, len(finite_values(p95_vals)))])
        vals = finite_values(max_vals)
        rows.append(["max_queue_delay_ms_after_warmup", max(vals) if vals else math.nan])
    if goodput_csv.exists():
        aggregate_vals: List[float] = []
        with goodput_csv.open("r", encoding="utf-8", newline="") as fh:
            reader = csv.DictReader(fh)
            for row in reader:
                if to_float(row.get("time_s")) < warmup_s:
                    continue
                aggregate_vals.append(to_float(row.get("aggregate_goodput_mbps")))
        vals = finite_values(aggregate_vals)
        rows.append(["avg_aggregate_goodput_mbps_after_warmup", sum(vals) / max(1, len(vals))])
    if srtt_csv.exists():
        mean_srtt_values: List[float] = []
        p95_latest_values: List[float] = []
        with srtt_csv.open("r", encoding="utf-8", newline="") as fh:
            reader = csv.DictReader(fh)
            for row in reader:
                if to_float(row.get("time_s")) < warmup_s:
                    continue
                mean_srtt_values.append(to_float(row.get("mean_smoothed_rtt_ms")))
                p95_latest_values.append(to_float(row.get("p95_latest_rtt_ms")))
        vals = finite_values(mean_srtt_values)
        rows.append(["avg_mean_smoothed_rtt_ms_after_warmup", sum(vals) / max(1, len(vals))])
        vals = finite_values(p95_latest_values)
        rows.append(["avg_p95_latest_rtt_ms_after_warmup", sum(vals) / max(1, len(vals))])
    path = output_dir / "debug_summary.csv"
    write_csv(path, rows[0], rows[1:])
    return path


def record_command(output_dir: Path, argv: Sequence[str]) -> None:
    command = " ".join(shlex.quote(part) for part in [sys.executable, *argv])
    (output_dir / "plot_command.txt").write_text(command + "\n", encoding="utf-8")


def main(argv: Optional[List[str]] = None) -> int:
    args = parse_args(argv)
    run_dir = Path(args.run_dir).expanduser().resolve()
    if not run_dir.exists():
        raise SystemExit(f"Run directory does not exist: {run_dir}")
    output_dir = Path(args.output_dir).expanduser().resolve() if args.output_dir else run_dir / "debug_plots"
    output_dir.mkdir(parents=True, exist_ok=True)
    clean_output_dir(output_dir)
    record_command(output_dir, sys.argv)
    service_rate_bps = infer_service_rate(run_dir, args.service_rate)

    generated: List[Path] = []
    by_flow = load_round_bandwidth(run_dir)
    if by_flow:
        generated.append(write_round_csv(output_dir, by_flow))
    for path in (
        plot_round_bandwidth_per_flow(output_dir, run_dir, by_flow),
        plot_round_aggregate(output_dir, run_dir, by_flow, service_rate_bps, args.sample_step_s),
        plot_round_quality(output_dir, by_flow),
        plot_queue_delay(output_dir, run_dir, service_rate_bps, args.sample_step_s),
        plot_goodput(output_dir, run_dir, service_rate_bps, args.sample_step_s),
    ):
        if path:
            generated.append(path)

    queue_per_flow, queue_aggregate = plot_queue_delay_per_flow(
        output_dir, run_dir, args.sample_step_s
    )
    if queue_per_flow:
        generated.append(queue_per_flow)
    if queue_aggregate:
        generated.append(queue_aggregate)

    pacing_per_flow, pacing_aggregate = plot_pacing_from_gate(
        output_dir, run_dir, service_rate_bps, args.sample_step_s
    )
    if pacing_per_flow is None:
        pacing_per_flow, pacing_aggregate = plot_pacing_from_sendrate(
            output_dir, run_dir, service_rate_bps, args.sample_step_s
        )
    if pacing_per_flow:
        generated.append(pacing_per_flow)
    if pacing_aggregate:
        generated.append(pacing_aggregate)
    if pacing_per_flow is None:
        proxy = plot_pacing_proxy(output_dir, run_dir, by_flow)
        if proxy:
            generated.append(proxy)

    trusted_per_flow, trusted_aggregate = plot_trusted_bw_per_flow(
        output_dir, run_dir, service_rate_bps
    )
    if trusted_per_flow:
        generated.append(trusted_per_flow)
    if trusted_aggregate:
        generated.append(trusted_aggregate)

    delivery_path = plot_delivery_rate_first_window(output_dir, run_dir, 10.0)
    if delivery_path:
        generated.append(delivery_path)
    delivery_per_flow, delivery_aggregate = plot_delivery_rate_per_flow_and_aggregate(
        output_dir, run_dir, service_rate_bps, args.sample_step_s
    )
    if delivery_per_flow:
        generated.append(delivery_per_flow)
    if delivery_aggregate:
        generated.append(delivery_aggregate)

    srtt_per_flow, srtt_aggregate = plot_rtt(output_dir, run_dir, args.sample_step_s)
    if srtt_per_flow:
        generated.append(srtt_per_flow)
    if srtt_aggregate:
        generated.append(srtt_aggregate)

    max_time = 0.0
    goodput_files = find_goodput_files(run_dir)
    for path in goodput_files:
        rows = read_two_column_trace(path)
        if rows:
            max_time = max(max_time, rows[-1][0])
    mode_path = plot_mode_timeline(output_dir, run_dir, max_time)
    if mode_path:
        generated.append(mode_path)

    summary_path = summarize(output_dir, service_rate_bps, args.warmup_s)
    if summary_path:
        generated.append(summary_path)

    manifest = output_dir / "manifest.txt"
    all_outputs = sorted(
        path for path in output_dir.iterdir() if path.is_file() and path.name != manifest.name
    )
    manifest.write_text(
        "\n".join(
            [
                f"run_dir={run_dir}",
                f"service_rate_mbps={service_rate_bps / 1e6:.6f}",
                f"sample_step_s={args.sample_step_s}",
                "generated:",
                *[f"  {path.name}" for path in all_outputs],
            ]
        )
        + "\n",
        encoding="utf-8",
    )
    print(f"FreqCCv4 debug plots written to: {output_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
