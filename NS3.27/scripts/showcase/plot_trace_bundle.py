#!/usr/bin/env python3
"""Render unified showcase figures from ns-3 DQC trace exports."""

from __future__ import annotations

import argparse
import re
from pathlib import Path

import matplotlib.pyplot as plt
import pandas as pd


FLOW_FILE_RE = re.compile(r"^(?P<base>.+)_(?P<flow>\d+)_(?P<metric>[a-z]+)\.txt$")
FLOW_COLORS = ["#d62828", "#1d3557", "#2a9d8f", "#f4a261", "#6a4c93", "#4d908e"]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Plot receiver rate, RTT, queue delay, loss, and queue occupancy "
        "from a single ns-3 trace directory."
    )
    parser.add_argument("--trace-dir", required=True, help="Input directory containing *.txt traces")
    parser.add_argument("--out-dir", required=True, help="Output directory for PNG figures")
    parser.add_argument("--title", default="", help="Scenario title prefix used in figure titles")
    parser.add_argument("--prefix", default="", help="Optional filename prefix for generated figures")
    return parser.parse_args()


def read_trace(path: Path, names: list[str]) -> pd.DataFrame:
    return pd.read_csv(path, sep=r"\s+", comment="#", header=None, names=names, engine="python")


def read_queue_trace(path: Path) -> pd.DataFrame:
    records = []
    with path.open("r", encoding="utf-8") as handle:
        for raw_line in handle:
            line = raw_line.strip()
            if not line or line.startswith("#"):
                continue
            fields = line.split("\t")
            row = {
                "time_s": float(fields[0]),
                "total_bytes": float(fields[1]),
            }
            for index, field in enumerate(fields[2:], start=1):
                byte_count = field.split("(", 1)[0]
                row[f"flow{index}_bytes"] = float(byte_count)
            records.append(row)
    return pd.DataFrame(records)


def discover_bundle(trace_dir: Path) -> tuple[str, list[int]]:
    base_names: dict[str, set[int]] = {}
    for path in trace_dir.glob("*_recvrate.txt"):
        match = FLOW_FILE_RE.match(path.name)
        if not match:
            continue
        base = match.group("base")
        flow = int(match.group("flow"))
        base_names.setdefault(base, set()).add(flow)
    if not base_names:
        raise FileNotFoundError(f"No flow trace files found in {trace_dir}")
    base_name, flows = max(base_names.items(), key=lambda item: len(item[1]))
    return base_name, sorted(flows)


def metric_path(trace_dir: Path, base_name: str, flow: int, metric: str) -> Path:
    return trace_dir / f"{base_name}_{flow}_{metric}.txt"


def make_figure(figsize: tuple[float, float] = (10, 5)):
    plt.rc("font", size=16)
    plt.rc("axes", grid=False)
    plt.rc("axes", facecolor="white")
    fig, ax = plt.subplots(figsize=figsize)
    return fig, ax


def save_figure(fig, out_dir: Path, filename: str) -> None:
    out_dir.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_dir / filename, bbox_inches="tight", dpi=300)
    plt.close(fig)


def build_filename(prefix: str, stem: str) -> str:
    return f"{prefix}_{stem}.png" if prefix else f"{stem}.png"


def metric_title(title: str, suffix: str) -> str:
    return f"{title}: {suffix}" if title else suffix


def plot_receiver_rate(trace_dir: Path, out_dir: Path, base_name: str, flows: list[int], title: str, prefix: str) -> None:
    fig, ax = make_figure()
    for idx, flow in enumerate(flows):
        data = read_trace(metric_path(trace_dir, base_name, flow, "recvrate"), ["time_s", "rate_kbps"])
        ax.plot(
            data["time_s"],
            data["rate_kbps"] / 1000.0,
            label=f"Flow {flow}",
            color=FLOW_COLORS[idx % len(FLOW_COLORS)],
            linewidth=1.8,
        )
    ax.set_title(metric_title(title, "Receiver Rate"))
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Rate (Mbps)")
    ax.legend(ncol=min(4, len(flows)), fontsize=12)
    save_figure(fig, out_dir, build_filename(prefix, "recv_rate"))


def plot_smoothed_rtt(trace_dir: Path, out_dir: Path, base_name: str, title: str, prefix: str) -> None:
    fig, ax = make_figure()
    data = read_trace(metric_path(trace_dir, base_name, 1, "rtt"), ["time_s", "seq", "rtt_ms", "srtt_ms"])
    ax.plot(data["time_s"], data["srtt_ms"], color="#1d3557", linewidth=1.8)
    ax.set_title(metric_title(title, "Flow 1 Smoothed RTT"))
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("RTT (ms)")
    save_figure(fig, out_dir, build_filename(prefix, "flow1_srtt"))


def plot_queue_delay(trace_dir: Path, out_dir: Path, base_name: str, title: str, prefix: str) -> None:
    fig, ax = make_figure()
    data = read_trace(
        metric_path(trace_dir, base_name, 1, "qdelay"),
        ["time_s", "queue_delay_ms", "latest_rtt_ms", "min_rtt_ms"],
    )
    ax.plot(data["time_s"], data["queue_delay_ms"], color="#2a9d8f", linewidth=1.8)
    ax.set_title(metric_title(title, "Flow 1 Queue Delay"))
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Queue Delay (ms)")
    save_figure(fig, out_dir, build_filename(prefix, "flow1_qdelay"))


def plot_loss(trace_dir: Path, out_dir: Path, base_name: str, title: str, prefix: str) -> None:
    fig, ax = make_figure()
    data = read_trace(
        metric_path(trace_dir, base_name, 1, "lossrate"),
        ["time_s", "loss_rate_pct", "cum_loss_rate_pct"],
    )
    ax.plot(data["time_s"], data["cum_loss_rate_pct"], color="#d62828", linewidth=1.8)
    ax.set_title(metric_title(title, "Flow 1 Cumulative Loss Rate"))
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Loss Rate (%)")
    save_figure(fig, out_dir, build_filename(prefix, "flow1_cum_loss"))


def plot_queue_occupancy(trace_dir: Path, out_dir: Path, base_name: str, flows: list[int], title: str, prefix: str) -> None:
    fig, ax = make_figure(figsize=(10, 5.5))
    data = read_queue_trace(trace_dir / f"{base_name}_bottleneck_queue.txt")
    ax.plot(data["time_s"], data["total_bytes"], color="black", linewidth=2.0, label="Total")
    for idx, flow in enumerate(flows):
        column = f"flow{flow}_bytes"
        if column not in data.columns:
            continue
        ax.plot(
            data["time_s"],
            data[column],
            label=f"Flow {flow}",
            color=FLOW_COLORS[idx % len(FLOW_COLORS)],
            linewidth=1.4,
            alpha=0.9,
        )
    ax.set_title(metric_title(title, "Bottleneck Queue Occupancy"))
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Bytes")
    ax.legend(ncol=min(3, len(flows) + 1), fontsize=11)
    save_figure(fig, out_dir, build_filename(prefix, "bottleneck_queue"))


def main() -> None:
    args = parse_args()
    trace_dir = Path(args.trace_dir).resolve()
    out_dir = Path(args.out_dir).resolve()
    base_name, flows = discover_bundle(trace_dir)

    plot_receiver_rate(trace_dir, out_dir, base_name, flows, args.title, args.prefix)
    plot_smoothed_rtt(trace_dir, out_dir, base_name, args.title, args.prefix)
    plot_queue_delay(trace_dir, out_dir, base_name, args.title, args.prefix)
    plot_loss(trace_dir, out_dir, base_name, args.title, args.prefix)
    plot_queue_occupancy(trace_dir, out_dir, base_name, flows, args.title, args.prefix)

    print(f"Generated 5 figures in {out_dir}")


if __name__ == "__main__":
    main()
