#!/usr/bin/env python3
"""Plot and validate a two-state FBBR delivery-rate experiment."""

import argparse
import csv
import json
import math
from pathlib import Path
from statistics import mean
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt


Point = Tuple[float, float]


def read_whitespace_points(path: Path, value_column: int) -> List[Point]:
    rows: List[Point] = []
    with path.open("r", encoding="utf-8") as stream:
        for line in stream:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split()
            if len(parts) <= value_column:
                continue
            try:
                time_s = float(parts[0])
                value = float(parts[value_column])
            except ValueError:
                continue
            if math.isfinite(time_s) and math.isfinite(value):
                rows.append((time_s, value))
    return rows


def read_queue(path: Path) -> List[Dict[str, float]]:
    rows: List[Dict[str, float]] = []
    with path.open("r", encoding="utf-8", newline="") as stream:
        for raw in csv.DictReader(stream):
            try:
                rows.append({key: float(value) for key, value in raw.items()})
            except (TypeError, ValueError):
                continue
    return rows


def percentile(values: Sequence[float], quantile: float) -> float:
    clean = sorted(value for value in values if math.isfinite(value))
    if not clean:
        return math.nan
    position = quantile * (len(clean) - 1)
    low = int(math.floor(position))
    high = int(math.ceil(position))
    if low == high:
        return clean[low]
    fraction = position - low
    return clean[low] * (1.0 - fraction) + clean[high] * fraction


def select_points(rows: Iterable[Point], start_s: float, end_s: float) -> List[Point]:
    return [(time_s, value) for time_s, value in rows if start_s <= time_s <= end_s]


def select_queue(
    rows: Iterable[Dict[str, float]], start_s: float, end_s: float
) -> List[Dict[str, float]]:
    return [row for row in rows if start_s <= row["time_s"] <= end_s]


def bin_points(rows: Sequence[Point], width_s: float) -> List[Point]:
    bins: Dict[int, List[float]] = {}
    for time_s, value in rows:
        index = int(math.floor(time_s / width_s + 1e-12))
        bins.setdefault(index, []).append(value)
    return [
        ((index + 0.5) * width_s, mean(values))
        for index, values in sorted(bins.items())
    ]


def delivery_metrics(rows_mbps: Sequence[Point]) -> Dict[str, float]:
    values = [value for _, value in rows_mbps]
    return {
        "samples": len(values),
        "mean_mbps": mean(values) if values else math.nan,
        "p05_mbps": percentile(values, 0.05),
        "p50_mbps": percentile(values, 0.50),
        "p95_mbps": percentile(values, 0.95),
    }


def queue_metrics(
    rows: Sequence[Dict[str, float]], buffer_bytes: float, bdp_bytes: float
) -> Dict[str, float]:
    queue_bytes = [row["queue_bytes"] for row in rows]
    drops = sum(row.get("drop_bytes_delta", 0.0) for row in rows)
    nonzero = sum(value > 0.0 for value in queue_bytes)
    maximum = max(queue_bytes) if queue_bytes else math.nan
    return {
        "samples": len(queue_bytes),
        "nonzero_fraction": nonzero / len(queue_bytes) if queue_bytes else math.nan,
        "mean_bytes": mean(queue_bytes) if queue_bytes else math.nan,
        "p50_bytes": percentile(queue_bytes, 0.50),
        "p95_bytes": percentile(queue_bytes, 0.95),
        "max_bytes": maximum,
        "max_bdp": maximum / bdp_bytes if bdp_bytes > 0.0 else math.nan,
        "max_buffer_fraction": maximum / buffer_bytes if buffer_bytes > 0.0 else math.nan,
        "drop_bytes": drops,
    }


def rtt_metrics(rows: Sequence[Tuple[float, float, float]], rtprop_ms: float) -> Dict[str, float]:
    latest = [row[1] for row in rows]
    smoothed = [row[2] for row in rows]
    maximum = max(latest) if latest else math.nan
    return {
        "samples": len(latest),
        "latest_p50_ms": percentile(latest, 0.50),
        "latest_p95_ms": percentile(latest, 0.95),
        "latest_max_ms": maximum,
        "smoothed_p50_ms": percentile(smoothed, 0.50),
        "max_queueing_rtt_ms": maximum - rtprop_ms if latest else math.nan,
    }


def read_rtt(path: Path) -> List[Tuple[float, float, float]]:
    rows: List[Tuple[float, float, float]] = []
    with path.open("r", encoding="utf-8") as stream:
        for line in stream:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split()
            if len(parts) < 4:
                continue
            try:
                rows.append((float(parts[0]), float(parts[2]), float(parts[3])))
            except ValueError:
                continue
    return rows


def find_flow_file(run_dir: Path, flow_id: int, suffix: str) -> Path:
    matches = sorted(run_dir.glob(f"*_flow{flow_id}_*_{suffix}"))
    if not matches:
        raise FileNotFoundError(f"missing flow {flow_id} trace ending in {suffix}")
    return matches[0]


def rate_at(schedule: Sequence[Dict[str, float]], time_s: float) -> float:
    rate_bps = 0.0
    for step in schedule:
        if float(step["time_s"]) > time_s + 1e-12:
            break
        rate_bps = float(step["rate_bps"])
    return rate_bps


def write_csv(path: Path, header: Sequence[str], rows: Iterable[Sequence[object]]) -> None:
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(header)
        writer.writerows(rows)


def shade_states(ax, underload: Tuple[float, float], loaded: Tuple[float, float]) -> None:
    ax.axvspan(*underload, color="#2A9D8F", alpha=0.10, linewidth=0)
    ax.axvspan(*loaded, color="#E76F51", alpha=0.10, linewidth=0)


def parse_args(argv: Optional[List[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--fbbr-run-dir", dest="run_dir", required=True)
    parser.add_argument("--output-dir", default="")
    parser.add_argument("--flow-id", type=int, default=1)
    parser.add_argument("--underload-window", type=float, nargs=2, required=True)
    parser.add_argument("--loaded-window", type=float, nargs=2, required=True)
    parser.add_argument("--bin-ms", type=float, default=20.0)
    return parser.parse_args(argv)


def main(argv: Optional[List[str]] = None) -> int:
    args = parse_args(argv)
    run_dir = Path(args.run_dir).resolve()
    output_dir = (
        Path(args.output_dir).resolve()
        if args.output_dir
        else run_dir / "two_state_analysis"
    )
    output_dir.mkdir(parents=True, exist_ok=True)

    underload = tuple(args.underload_window)
    loaded = tuple(args.loaded_window)
    if underload[1] <= underload[0] or loaded[1] <= loaded[0]:
        raise SystemExit("analysis windows must have positive duration")

    meta = json.loads((run_dir / "run_meta.json").read_text(encoding="utf-8"))
    queue_rows = read_queue(run_dir / "bottleneck_queue.csv")
    service_rate_bps = float(meta["capacity_bps"])
    bdp_bytes = float(meta["bdp_bytes"])
    buffer_bytes = float(meta["buffer_bytes"])
    rtprop_ms = 1000.0 * float(meta["rtprop_s"])
    buffer_delay_ms = 1000.0 * 8.0 * buffer_bytes / service_rate_bps

    delivery_by_flow: Dict[int, List[Point]] = {}
    for flow_id in range(1, int(meta["n_flows"]) + 1):
        raw_path = find_flow_file(run_dir, flow_id, "recvrate_raw.txt")
        delivery_by_flow[flow_id] = [
            (time_s, value_kbps / 1000.0)
            for time_s, value_kbps in read_whitespace_points(raw_path, 1)
        ]
    target_delivery = delivery_by_flow[args.flow_id]
    rtt_path = find_flow_file(run_dir, args.flow_id, "rtt.txt")
    rtt_rows = read_rtt(rtt_path)

    states: Dict[str, Dict[str, object]] = {}
    for name, window in (("underload", underload), ("loaded", loaded)):
        state_delivery = select_points(target_delivery, *window)
        state_queue = select_queue(queue_rows, *window)
        state_rtt = [row for row in rtt_rows if window[0] <= row[0] <= window[1]]
        states[name] = {
            "window_s": list(window),
            "delivery": delivery_metrics(state_delivery),
            "queue": queue_metrics(state_queue, buffer_bytes, bdp_bytes),
            "rtt": rtt_metrics(state_rtt, rtprop_ms),
        }

    under_q = states["underload"]["queue"]
    loaded_q = states["loaded"]["queue"]
    loaded_rtt = states["loaded"]["rtt"]
    checks = {
        "underload_has_no_standing_queue": (
            float(under_q["p95_bytes"]) <= 1500.0
            and float(under_q["drop_bytes"]) == 0.0
        ),
        "loaded_has_persistent_queue": (
            float(loaded_q["nonzero_fraction"]) >= 0.90
            and float(loaded_q["p50_bytes"]) > 1500.0
        ),
        "loaded_queue_does_not_fill_buffer": (
            float(loaded_q["max_buffer_fraction"]) < 0.80
            and float(loaded_q["drop_bytes"]) == 0.0
        ),
        "loaded_rtt_excursion_fits_buffer": (
            float(loaded_rtt["max_queueing_rtt_ms"]) < buffer_delay_ms
        ),
    }
    report = {
        "run_dir": str(run_dir),
        "target_flow_id": args.flow_id,
        "service_rate_bps": service_rate_bps,
        "bdp_bytes": bdp_bytes,
        "buffer_bytes": buffer_bytes,
        "buffer_bdp": buffer_bytes / bdp_bytes,
        "rtprop_ms": rtprop_ms,
        "buffer_delay_ms": buffer_delay_ms,
        "states": states,
        "checks": checks,
        "all_checks_pass": all(checks.values()),
    }
    (output_dir / "two_state_metrics.json").write_text(
        json.dumps(report, indent=2) + "\n", encoding="utf-8"
    )

    bin_width_s = args.bin_ms / 1000.0
    binned_by_flow = {
        flow_id: bin_points(rows, bin_width_s)
        for flow_id, rows in delivery_by_flow.items()
    }
    write_csv(
        output_dir / "delivery_timeseries.csv",
        ("flow_id", "time_s", "delivery_rate_mbps"),
        (
            (flow_id, time_s, value)
            for flow_id, rows in binned_by_flow.items()
            for time_s, value in rows
        ),
    )

    schedules = meta.get("per_flow_rate_schedules", [])
    target_schedule = schedules[args.flow_id - 1] if len(schedules) >= args.flow_id else []
    cruise_baseline_caps = meta.get("fbbr_cruise_baseline_caps_bps", [])
    target_cruise_baseline_cap_bps = (
        float(cruise_baseline_caps[args.flow_id - 1])
        if len(cruise_baseline_caps) >= args.flow_id
        else 0.0
    )
    plot_start = min(underload[0], loaded[0])
    plot_end = max(underload[1], loaded[1])

    fig, axes = plt.subplots(3, 1, figsize=(13.0, 9.0), sharex=True)
    colors = ("#006D77", "#E76F51", "#6A4C93", "#457B9D")
    for flow_id, rows in binned_by_flow.items():
        visible = select_points(rows, plot_start, plot_end)
        axes[0].plot(
            [row[0] for row in visible],
            [row[1] for row in visible],
            linewidth=1.0 if flow_id != args.flow_id else 1.5,
            alpha=0.65 if flow_id != args.flow_id else 0.95,
            color=colors[(flow_id - 1) % len(colors)],
            label=f"flow {flow_id}" + (" (target)" if flow_id == args.flow_id else ""),
        )
    if target_schedule and any(float(step["rate_bps"]) > 0.0 for step in target_schedule):
        step_times = [plot_start]
        step_rates = [rate_at(target_schedule, plot_start) / 1e6]
        for step in target_schedule:
            time_s = float(step["time_s"])
            if plot_start < time_s < plot_end:
                step_times.append(time_s)
                step_rates.append(float(step["rate_bps"]) / 1e6)
        step_times.append(plot_end)
        step_rates.append(step_rates[-1])
        axes[0].step(
            step_times,
            step_rates,
            where="post",
            color="#222222",
            linewidth=1.0,
            linestyle="--",
            label="target pacing cap",
        )
    if target_cruise_baseline_cap_bps > 0.0:
        axes[0].axhline(
            target_cruise_baseline_cap_bps / 1e6,
            color="#222222",
            linewidth=1.0,
            linestyle="--",
            label="target CRUISE baseline cap",
        )
    axes[0].axhline(service_rate_bps / 1e6, color="#777777", linewidth=0.8, linestyle=":")
    axes[0].set_ylabel("Delivery rate (Mbps)")
    axes[0].legend(
        loc="upper left",
        bbox_to_anchor=(1.005, 1.0),
        ncol=1,
        fontsize=8,
        borderaxespad=0.0,
    )
    axes[0].grid(alpha=0.20)

    visible_queue = select_queue(queue_rows, plot_start, plot_end)
    axes[1].plot(
        [row["time_s"] for row in visible_queue],
        [row["queue_bdp"] for row in visible_queue],
        color="#264653",
        linewidth=0.9,
    )
    axes[1].axhline(buffer_bytes / bdp_bytes, color="#C1121F", linestyle="--", linewidth=1.0)
    axes[1].set_ylabel("Queue (BDP)")
    axes[1].grid(alpha=0.20)

    visible_rtt = [row for row in rtt_rows if plot_start <= row[0] <= plot_end]
    axes[2].plot(
        [row[0] for row in visible_rtt],
        [row[1] for row in visible_rtt],
        color="#6A4C93",
        linewidth=0.7,
        alpha=0.50,
        label="latest RTT",
    )
    axes[2].plot(
        [row[0] for row in visible_rtt],
        [row[2] for row in visible_rtt],
        color="#2A9D8F",
        linewidth=1.2,
        label="smoothed RTT",
    )
    axes[2].axhline(rtprop_ms, color="#777777", linewidth=0.8, linestyle=":", label="RTprop")
    axes[2].axhline(
        rtprop_ms + buffer_delay_ms,
        color="#C1121F",
        linewidth=1.0,
        linestyle="--",
        label="10BDP RTT ceiling",
    )
    axes[2].set_ylabel("RTT (ms)")
    axes[2].set_xlabel("Simulation time (s)")
    axes[2].legend(loc="upper right", ncol=2, fontsize=8)
    axes[2].grid(alpha=0.20)

    for ax in axes:
        shade_states(ax, underload, loaded)
        ax.set_xlim(plot_start, plot_end)
    axes[0].text(
        underload[0] + 0.15,
        0.90,
        "UNDERLOAD",
        transform=axes[0].get_xaxis_transform(),
        ha="left",
        va="top",
        color="#147D73",
        fontsize=9,
        fontweight="bold",
    )
    axes[0].text(
        loaded[0] + 0.15,
        0.90,
        "FULL / LIGHT OVERLOAD",
        transform=axes[0].get_xaxis_transform(),
        ha="left",
        va="top",
        color="#B64932",
        fontsize=9,
        fontweight="bold",
    )
    fig.suptitle("FBBR delivery response across two offered-load states")
    fig.tight_layout()
    fig.savefig(
        output_dir / "delivery_two_state_overview.png",
        dpi=180,
        bbox_inches="tight",
    )
    plt.close(fig)

    comparison_duration_s = min(
        underload[1] - underload[0], loaded[1] - loaded[0]
    )
    tick_step_s = (
        0.2 if comparison_duration_s <= 2.0
        else 0.5 if comparison_duration_s <= 4.0
        else 1.0
    )
    tick_count = int(math.floor(comparison_duration_s / tick_step_s + 1e-9))
    common_ticks = [index * tick_step_s for index in range(tick_count + 1)]
    if not common_ticks or common_ticks[-1] < comparison_duration_s - 1e-9:
        common_ticks.append(comparison_duration_s)

    fig, axes = plt.subplots(
        1, 2, figsize=(13.0, 4.2), sharex=True, sharey=True
    )
    for ax, (name, window), color in zip(
        axes,
        (("Underload", underload), ("Full / light overload", loaded)),
        ("#006D77", "#E76F51"),
    ):
        effective_end_s = window[0] + comparison_duration_s
        rows = select_points(
            binned_by_flow[args.flow_id], window[0], effective_end_s
        )
        ax.plot(
            [time_s - window[0] for time_s, _ in rows],
            [value for _, value in rows],
            color=color,
            linewidth=1.3,
        )
        ax.set_title(name)
        ax.set_xlabel("Time within state (s)")
        ax.set_xlim(0.0, comparison_duration_s)
        ax.set_xticks(common_ticks)
        ax.grid(alpha=0.22)
    axes[0].set_ylabel("Target-flow delivery rate (Mbps)")
    fig.suptitle("Target-flow delivery time series in the two states")
    fig.tight_layout()
    fig.savefig(output_dir / "delivery_underload_vs_loaded.png", dpi=180)
    plt.close(fig)

    report_lines = [
        "# FBBR two-state validation",
        "",
        f"- Target flow: {args.flow_id}",
        f"- Bottleneck: {service_rate_bps / 1e6:.3f} Mbps",
        f"- Buffer: {buffer_bytes:.0f} bytes ({buffer_bytes / bdp_bytes:.3f} BDP)",
        f"- RTprop: {rtprop_ms:.3f} ms",
        f"- Buffer RTT budget: {buffer_delay_ms:.3f} ms",
        "",
    ]
    for name in ("underload", "loaded"):
        state = states[name]
        delivery = state["delivery"]
        queue = state["queue"]
        rtt = state["rtt"]
        report_lines.extend(
            [
                f"## {name}",
                "",
                f"- Window: {state['window_s'][0]:.3f}-{state['window_s'][1]:.3f} s",
                f"- Delivery mean/p05/p95: {delivery['mean_mbps']:.3f} / {delivery['p05_mbps']:.3f} / {delivery['p95_mbps']:.3f} Mbps",
                f"- Queue nonzero fraction: {queue['nonzero_fraction']:.6f}",
                f"- Queue p50/p95/max: {queue['p50_bytes']:.1f} / {queue['p95_bytes']:.1f} / {queue['max_bytes']:.1f} bytes",
                f"- Queue max buffer fraction: {queue['max_buffer_fraction']:.6f}",
                f"- Drop bytes: {queue['drop_bytes']:.0f}",
                f"- Latest RTT p95/max: {rtt['latest_p95_ms']:.3f} / {rtt['latest_max_ms']:.3f} ms",
                "",
            ]
        )
    report_lines.extend(
        ["## Checks", ""]
        + [f"- {'PASS' if passed else 'FAIL'}: {name}" for name, passed in checks.items()]
        + ["", f"Overall: {'PASS' if report['all_checks_pass'] else 'FAIL'}", ""]
    )
    (output_dir / "README.md").write_text("\n".join(report_lines), encoding="utf-8")

    print(f"two-state plots and metrics written to: {output_dir}")
    print(json.dumps(checks, indent=2))
    return 0 if report["all_checks_pass"] else 3


if __name__ == "__main__":
    raise SystemExit(main())
