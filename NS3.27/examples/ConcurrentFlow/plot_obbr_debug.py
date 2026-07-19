#!/usr/bin/env python3
"""Plot delivery-rate figures from raw receiver traces.

This script is intentionally trace-format driven. It consumes
``*_recvrate_raw.txt`` traces, so it works for BBRv2plus, FBBR, oBBR,
and other single-CC run directories that emit raw delivery-rate samples.
"""

import argparse
import json
import math
import shlex
import sys
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

from plot_fbbr_debug import (
    add_phase_background,
    add_phase_separators_with_labels,
    build_grid,
    first_flow_id_with_modes,
    flow_id_from_path,
    infer_service_rate as fbbr_infer_service_rate,
    read_two_column_trace,
    sample_previous,
    write_csv,
)


def parse_args(argv: Optional[List[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Create delivery-rate plots from raw receiver traces."
    )
    parser.add_argument(
        "--run-dir",
        "--fbbr-run-dir",
        "--obbr-run-dir",
        "--bbrv2plus-run-dir",
        dest="run_dir",
        required=True,
        help="Run directory that contains *_recvrate_raw.txt traces.",
    )
    parser.add_argument(
        "--output-dir",
        default="",
        help="Output directory. Defaults to RUN_DIR/debug_plots.",
    )
    parser.add_argument(
        "--service-rate",
        default="",
        help=(
            "Bottleneck service rate. Defaults to run_meta/config/"
            "comparison_config, then 100Mbps."
        ),
    )
    parser.add_argument(
        "--cc-label",
        default="",
        help="Label used in plot titles. Defaults to run_meta algorithm or run directory name.",
    )
    parser.add_argument(
        "--sample-step-s",
        type=float,
        default=0.1,
        help="Sampling/binning step for aggregate time series. Default: 0.1s.",
    )
    parser.add_argument(
        "--first-window-s",
        type=float,
        default=10.0,
        help="Delivery-rate window end time for the first-window figure. Default: 10s.",
    )
    return parser.parse_args(argv)


def rate_value_bps(run_dir: Path, value: object) -> Optional[float]:
    if isinstance(value, (int, float)):
        rate_bps = float(value)
    else:
        text = str(value).strip() if value is not None else ""
        if not text:
            return None
        try:
            rate_bps = fbbr_infer_service_rate(run_dir, text)
        except ValueError:
            return None
    return rate_bps if math.isfinite(rate_bps) and rate_bps > 0.0 else None


def infer_service_rate(run_dir: Path, user_value: str) -> float:
    if user_value:
        return fbbr_infer_service_rate(run_dir, user_value)

    meta_path = run_dir / "run_meta.json"
    if meta_path.exists():
        try:
            data = json.loads(meta_path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError):
            data = {}
        for key in ("capacity_bps", "configured_capacity_bps", "service_rate_bps"):
            rate_bps = rate_value_bps(run_dir, data.get(key))
            if rate_bps is not None:
                return rate_bps
    return fbbr_infer_service_rate(run_dir, "")


def infer_cc_label(run_dir: Path) -> str:
    meta_path = run_dir / "run_meta.json"
    if meta_path.exists():
        try:
            data = json.loads(meta_path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError):
            data = {}
        for key in ("algorithm", "cc", "congestion_control"):
            value = str(data.get(key, "")).strip()
            if value:
                return value
        algorithms = data.get("algorithms")
        if isinstance(algorithms, list):
            labels = [str(item).strip() for item in algorithms if str(item).strip()]
            if labels and len(set(labels)) == 1:
                return labels[0]
    return run_dir.name or "CC"


def format_seconds(value: float) -> str:
    if abs(value - round(value)) < 1e-9:
        return str(int(round(value)))
    return f"{value:g}"


def find_delivery_rate_files(run_dir: Path) -> List[Path]:
    return sorted(run_dir.glob("*_flow*_*_recvrate_raw.txt"))


def cleanup_legacy_delivery_notes(output_dir: Path) -> None:
    for name in (
        "delivery_rate_trace_missing.txt",
        "delivery_rate_bbrv2_vs_fbbr_missing.txt",
        "obbr_delivery_rate_trace_missing.txt",
    ):
        (output_dir / name).unlink(missing_ok=True)


def record_delivery_rate_command(output_dir: Path, argv: List[str]) -> None:
    command = " ".join(shlex.quote(part) for part in [sys.executable, *argv])
    (output_dir / "delivery_rate_plot_command.txt").write_text(
        command + "\n", encoding="utf-8"
    )


def write_manifest(
    output_dir: Path,
    run_dir: Path,
    cc_label: str,
    service_rate_bps: float,
    sample_step_s: float,
) -> None:
    manifest = output_dir / "manifest.txt"
    all_outputs = sorted(
        path for path in output_dir.iterdir() if path.is_file() and path.name != manifest.name
    )
    manifest.write_text(
        "\n".join(
            [
                f"run_dir={run_dir}",
                f"cc_label={cc_label}",
                f"service_rate_mbps={service_rate_bps / 1e6:.6f}",
                f"sample_step_s={sample_step_s}",
                f"delivery_trace_source={cc_label}_recvrate_raw",
                "generated:",
                *[f"  {path.name}" for path in all_outputs],
            ]
        )
        + "\n",
        encoding="utf-8",
    )


def load_delivery_rate(path: Path, end_s: float) -> List[Tuple[float, float]]:
    return [
        (time_s, value_mbps)
        for time_s, value_mbps in read_two_column_trace(path, value_scale=1.0 / 1000.0)
        if time_s <= end_s
    ]


def plot_delivery_rate_first_window(
    output_dir: Path,
    run_dir: Path,
    cc_label: str,
    end_s: float,
) -> Optional[Path]:
    by_flow: Dict[int, List[Tuple[float, float]]] = {}
    for path in find_delivery_rate_files(run_dir):
        flow_id = flow_id_from_path(path)
        rows = load_delivery_rate(path, end_s)
        if flow_id > 0 and rows:
            by_flow[flow_id] = rows
    if not by_flow:
        (output_dir / "delivery_rate_trace_missing.txt").write_text(
            f"No {cc_label} delivery-rate trace was found in this run.\n\n"
            "Expected files:\n"
            "  - *_flow*_*_recvrate_raw.txt with delivery_rate_sample(kbps)\n\n"
            "Rerun with heavy trace enabled so the raw delivery-rate stream is emitted.\n",
            encoding="utf-8",
        )
        return None

    cleanup_legacy_delivery_notes(output_dir)

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
    fig.suptitle(
        f"{cc_label} delivery rate in the first {format_seconds(end_s)} seconds",
        y=0.995,
    )
    fig.subplots_adjust(left=0.08, right=0.99, top=0.96, bottom=0.08, hspace=0.42)
    path = output_dir / "delivery_rate_first10_by_flow.png"
    fig.savefig(path, dpi=180)
    plt.close(fig)
    return path


def plot_delivery_rate_first_window_combined(
    output_dir: Path,
    run_dir: Path,
    cc_label: str,
    service_rate_bps: float,
    end_s: float,
) -> Optional[Path]:
    by_flow: Dict[int, List[Tuple[float, float]]] = {}
    for path in find_delivery_rate_files(run_dir):
        flow_id = flow_id_from_path(path)
        rows = load_delivery_rate(path, end_s)
        if flow_id > 0 and rows:
            by_flow[flow_id] = rows
    if not by_flow:
        return None

    flow_ids = sorted(by_flow)
    fair_share_mbps = service_rate_bps / 1e6 / max(1, len(flow_ids))

    fig, ax = plt.subplots(figsize=(12.0, 5.8))
    add_phase_separators_with_labels(
        ax,
        run_dir,
        flow_ids[0],
        0.0,
        end_s,
        alternating_background=True,
    )
    for flow_id in flow_ids:
        rows = by_flow[flow_id]
        ax.plot(
            [time_s for time_s, _ in rows],
            [value for _, value in rows],
            linewidth=1.0,
            alpha=0.86,
            label=f"flow{flow_id}",
        )
    ax.axhline(
        fair_share_mbps,
        color="#333333",
        linestyle="--",
        linewidth=1.1,
        label=f"fair share ({fair_share_mbps:.2f} Mbps)",
        zorder=4,
    )
    ax.set_xlim(0.0, end_s)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Delivery rate (Mbps)")
    ax.set_title(
        f"{cc_label} delivery rate in the first {format_seconds(end_s)} seconds"
    )
    ax.grid(True, alpha=0.25)
    ax.legend(frameon=False, ncol=3, loc="upper right")
    fig.tight_layout()
    path = output_dir / "delivery_rate_first10_combined.png"
    fig.savefig(path, dpi=180)
    plt.close(fig)
    return path


def plot_delivery_rate_per_flow_and_aggregate(
    output_dir: Path,
    run_dir: Path,
    cc_label: str,
    service_rate_bps: float,
    sample_step_s: float,
) -> Tuple[Optional[Path], Optional[Path]]:
    by_flow: Dict[int, List[Tuple[float, float]]] = {}
    for path in find_delivery_rate_files(run_dir):
        flow_id = flow_id_from_path(path)
        rows = load_delivery_rate(path, math.inf)
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
    fig.suptitle(f"{cc_label} delivery rate per flow", y=0.995)
    fig.tight_layout()
    per_flow_path = output_dir / "delivery_rate_per_flow.png"
    fig.savefig(per_flow_path, dpi=180)
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(12.0, 5.8))
    add_phase_background(ax, run_dir, first_flow_id_with_modes(run_dir), max_time)
    ax.plot(grid, aggregate, color="#111111", linewidth=1.3, label="aggregate")
    ax.axhline(service_rate_bps / 1e6, color="#555555", linestyle="--",
               linewidth=1.0, label="bottleneck capacity")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Mbps")
    ax.set_title(f"Aggregate {cc_label} delivery rate")
    ax.grid(True, alpha=0.25)
    ax.legend(frameon=False)
    fig.tight_layout()
    aggregate_path = output_dir / "delivery_rate_aggregate.png"
    fig.savefig(aggregate_path, dpi=180)
    plt.close(fig)
    cleanup_legacy_delivery_notes(output_dir)
    return per_flow_path, aggregate_path


def main(argv: Optional[List[str]] = None) -> int:
    args = parse_args(argv)
    run_dir = Path(args.run_dir).expanduser().resolve()
    if not run_dir.exists():
        raise SystemExit(f"Run directory does not exist: {run_dir}")
    if args.first_window_s <= 0.0:
        raise SystemExit("--first-window-s must be > 0")
    if args.sample_step_s <= 0.0:
        raise SystemExit("--sample-step-s must be > 0")
    cc_label = args.cc_label.strip() or infer_cc_label(run_dir)

    output_dir = (
        Path(args.output_dir).expanduser().resolve()
        if args.output_dir
        else run_dir / "debug_plots"
    )
    output_dir.mkdir(parents=True, exist_ok=True)
    record_delivery_rate_command(output_dir, sys.argv)

    service_rate_bps = infer_service_rate(run_dir, args.service_rate)
    first_window_path = plot_delivery_rate_first_window(
        output_dir,
        run_dir,
        cc_label,
        args.first_window_s,
    )
    first_window_combined_path = plot_delivery_rate_first_window_combined(
        output_dir,
        run_dir,
        cc_label,
        service_rate_bps,
        args.first_window_s,
    )
    per_flow_path, aggregate_path = plot_delivery_rate_per_flow_and_aggregate(
        output_dir,
        run_dir,
        cc_label,
        service_rate_bps,
        args.sample_step_s,
    )
    generated = [
        path
        for path in (
            first_window_path,
            first_window_combined_path,
            per_flow_path,
            aggregate_path,
        )
        if path
    ]
    if not generated:
        return 1
    write_manifest(output_dir, run_dir, cc_label, service_rate_bps, args.sample_step_s)
    print(f"{cc_label} delivery-rate plots written to: {output_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
