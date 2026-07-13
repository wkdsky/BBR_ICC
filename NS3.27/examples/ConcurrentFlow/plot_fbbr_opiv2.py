#!/usr/bin/env python3
"""Generate explainable F-BBR OPIv2 diagnostics from an ns-3 trace directory."""

import argparse
import csv
import math
import re
from collections import Counter
from pathlib import Path
from typing import Dict, Iterable, List, Optional

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt


FLOW_RE = re.compile(r"flow(\d+)", re.IGNORECASE)
CLASS_COLORS = {
    "UNDERLOAD": "#d9edf7",
    "NEAR_OPTIMAL": "#dff0d8",
    "OVERLOAD": "#f2dede",
    "QUEUED_OVERLOAD": "#f2dede",
    "BUFFER_SATURATED": "#d9534f",
    "DYNAMIC": "#fcf8e3",
    "INVALID": "#eeeeee",
}


def parse_args(argv: Optional[List[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Plot F-BBR coded-sine OPIv2 block, bin, and publication diagnostics."
    )
    parser.add_argument("--run-dir", required=True, help="Directory containing F-BBR trace CSVs")
    parser.add_argument("--output-dir", default="", help="Defaults to RUN_DIR/fbbr_opiv2_plots")
    parser.add_argument("--fair-share-bps", type=float, default=0.0)
    return parser.parse_args(argv)


def number(value: object, default: float = math.nan) -> float:
    try:
        parsed = float(str(value).strip())
        return parsed if math.isfinite(parsed) else default
    except (TypeError, ValueError):
        return default


def truth(value: object) -> bool:
    return str(value).strip().lower() in {"1", "true", "yes", "on"}


def value(row: Dict[str, str], *names: str) -> object:
    for name in names:
        if name in row and str(row[name]).strip() != "":
            return row[name]
    return None


def read_csv(path: Path) -> List[Dict[str, str]]:
    with path.open("r", encoding="utf-8", errors="replace", newline="") as handle:
        return list(csv.DictReader(handle))


def flow_id(path: Path) -> int:
    match = FLOW_RE.search(path.name)
    return int(match.group(1)) if match else 0


def shade_classes(axes: Iterable, blocks: List[Dict[str, str]]) -> None:
    for row in blocks:
        start = number(value(row, "window_start_s", "start_time_s"))
        end = number(value(row, "window_end_s", "end_time_s"))
        classification = str(row.get("classification", "INVALID"))
        if not math.isfinite(start) or not math.isfinite(end) or end <= start:
            continue
        for axis in axes:
            axis.axvspan(start, end, color=CLASS_COLORS.get(classification, "#eeeeee"),
                         alpha=0.28, linewidth=0)


def plot_flow(output_dir: Path, fid: int, bins: List[Dict[str, str]],
              blocks: List[Dict[str, str]], cruises: List[Dict[str, str]],
              fair_share_bps: float) -> Path:
    figure, axes = plt.subplots(7, 1, figsize=(14, 20), sharex=True)
    shade_classes(axes, blocks)

    bin_time = [0.5 * (number(row.get("time_start_s"), 0.0) +
                       number(row.get("time_end_s"), 0.0)) for row in bins]
    if bins:
        axes[0].plot(bin_time, [number(row.get("native_pacing_bps")) / 1e6 for row in bins],
                     label="native pacing", linewidth=0.9)
        axes[0].plot(bin_time, [number(row.get("commanded_pacing_bps")) / 1e6 for row in bins],
                     label="commanded pacing", linewidth=0.9)
        axes[0].plot(bin_time, [number(row.get("delivery_rate_bps")) / 1e6 for row in bins],
                     label="fixed-bin delivery", linewidth=0.8, alpha=0.8)
        axes[1].plot(bin_time, [number(row.get("qdelay_us")) / 1000.0 for row in bins],
                     label="qdelay", linewidth=0.9)
    block_time = [0.5 * (number(value(row, "window_start_s", "start_time_s"), 0.0) +
                         number(value(row, "window_end_s", "end_time_s"), 0.0))
                  for row in blocks]
    cruise_time = [number(row.get("end_time_s")) for row in cruises]
    if cruises:
        axes[0].step(cruise_time, [number(row.get("trusted_bw_bps"), 0.0) / 1e6 for row in cruises],
                     where="post", label="published trusted_bw", linewidth=1.5)
    if blocks and "baseline_before_bps" in blocks[0]:
        axes[0].step(block_time,
                     [number(row.get("baseline_before_bps"), 0.0) / 1e6
                      for row in blocks], where="mid", linewidth=1.4,
                     label="search baseline")
    if fair_share_bps > 0:
        axes[0].axhline(fair_share_bps / 1e6, color="#333", linestyle="--",
                        linewidth=1.0, label="fair share")
    axes[0].set_ylabel("Rate (Mbps)")
    axes[0].set_title(f"F-BBR flow {fid}: pacing, service, and trusted bandwidth")
    axes[0].legend(ncol=4, frameon=False)

    if blocks:
        axes[1].plot(block_time, [number(row.get("q_zero_us")) / 1000.0 for row in blocks],
                     label="q_zero", linestyle="--")
        axes[1].plot(block_time, [number(row.get("q_probe_max_us")) / 1000.0 for row in blocks],
                     label="q_probe_max", linestyle=":")
    axes[1].set_ylabel("Queue delay (ms)")
    axes[1].set_title("Raw queue response and low-queue bounds")
    if axes[1].get_legend_handles_labels()[0]:
        axes[1].legend(frameon=False)

    score_fields = [("C_meas", "C_meas"), ("S_raw", "S_raw"),
                    ("S_opt", "S_opt"), ("E_under", "E_under"),
                    ("E_over", "E_over"), ("direction_score", "D")]
    for field, label in score_fields:
        axes[2].plot(block_time, [number(row.get(field)) for row in blocks], marker="o",
                     markersize=2.5, label=label)
    axes[2].set_ylim(-0.05, 1.05)
    axes[2].set_ylabel("Score")
    axes[2].set_title("Measurement confidence and separable operating-point evidence")
    axes[2].legend(ncol=6, frameon=False)

    axes[3].plot(block_time, [number(row.get("G_d_pos")) for row in blocks], label="G_d_pos")
    axes[3].plot(block_time, [number(row.get("G_q_pos")) for row in blocks], label="G_q_pos")
    axes[3].plot(block_time, [number(row.get("drain_ratio")) for row in blocks], label="drain ratio")
    axes[3].plot(block_time, [number(row.get("q_floor_us")) /
                              max(number(row.get("rtprop_frozen_us")), 1.0)
                              for row in blocks], label="q_floor/RTprop")
    axes[3].set_ylabel("Gain / ratio")
    axes[3].set_title("Saturation, queue build, floor, and periodic drain")
    axes[3].legend(ncol=4, frameon=False)

    for field in ("g_lockin", "g_fd", "g_fused", "h_fd"):
        axes[4].plot(block_time, [number(row.get(field)) for row in blocks],
                     marker="o", markersize=2.5, label=field)
    axes[4].axhline(0.0, color="#333", linewidth=0.7)
    axes[4].set_ylabel("Utility derivative")
    axes[4].set_title("Lock-in gradient, finite-difference cross-check, and curvature")
    axes[4].legend(ncol=4, frameon=False)

    axes[5].plot(block_time, [number(row.get("baseline_before_bps"), 0.0) / 1e6
                              for row in blocks], marker="o", label="search baseline")
    axes[5].plot(block_time, [number(row.get("applied_next_baseline_bps"), 0.0) / 1e6
                              for row in blocks], marker=".", label="next baseline")
    axes[5].plot(block_time, [number(row.get("underload_bound_bps"), 0.0) / 1e6
                              for row in blocks], linestyle="--", label="bracket low")
    axes[5].plot(block_time, [number(row.get("overload_bound_bps"), 0.0) / 1e6
                              for row in blocks], linestyle="--", label="bracket high")
    axes[5].plot(block_time, [number(row.get("window_candidate_bps"), 0.0) / 1e6
                              for row in blocks], marker="x", label="lock candidate")
    if cruises:
        axes[5].step(cruise_time, [number(row.get("raw_candidate_bps"), 0.0) / 1e6 for row in cruises],
                     where="post", label="cruise consensus")
        axes[5].step(cruise_time, [number(row.get("published_bps"), 0.0) / 1e6 for row in cruises],
                     where="post", label="history-stabilized publication")
    if fair_share_bps > 0:
        axes[5].axhline(fair_share_bps / 1e6, color="#333", linestyle="--", label="fair share")
    axes[5].set_ylabel("Bandwidth (Mbps)")
    axes[5].set_title("Adaptive baseline, bracket, lock candidate, and publication")
    axes[5].legend(ncol=4, frameon=False)

    labels = [str(row.get("classification", "INVALID")) for row in blocks]
    class_index = {name: index for index, name in enumerate(CLASS_COLORS)}
    axes[6].scatter(block_time, [class_index.get(label, 4) for label in labels],
                    c=[CLASS_COLORS.get(label, "#eee") for label in labels], edgecolors="#444")
    axes[6].set_yticks(list(class_index.values()))
    axes[6].set_yticklabels(list(class_index.keys()))
    axes[6].set_ylabel("Classification")
    axes[6].set_xlabel("Simulation time (s)")
    axes[6].set_title("Explainable operating-point classification background")

    for axis in axes:
        axis.grid(True, alpha=0.22)
    figure.suptitle("F-BBR Adaptive CRUISE (OPI v3)", fontsize=16)
    figure.tight_layout(rect=(0, 0, 1, 0.985))
    output = output_dir / f"fbbr_opiv2_flow{fid}_diagnostics.png"
    figure.savefig(output, dpi=180)
    plt.close(figure)
    return output


def plot_invalid_reasons(output_dir: Path, blocks_by_flow: Dict[int, List[Dict[str, str]]]) -> Path:
    reasons = Counter()
    for blocks in blocks_by_flow.values():
        for row in blocks:
            reason = str(row.get("invalid_reason") or row.get("candidate_invalid_reason") or "none")
            if reason != "none":
                for part in reason.split("|"):
                    reasons[part] += 1
    labels, counts = zip(*reasons.most_common()) if reasons else (("none",), (0,))
    figure, axis = plt.subplots(figsize=(12, max(3.5, 0.42 * len(labels) + 1.5)))
    axis.barh(list(reversed(labels)), list(reversed(counts)), color="#607d8b")
    axis.set_xlabel("Block count")
    axis.set_title("F-BBR OPI v3 invalid/candidate-rejection reasons")
    axis.grid(True, axis="x", alpha=0.25)
    figure.tight_layout()
    output = output_dir / "fbbr_opiv2_invalid_reasons.png"
    figure.savefig(output, dpi=180)
    plt.close(figure)
    return output


def main(argv: Optional[List[str]] = None) -> int:
    args = parse_args(argv)
    run_dir = Path(args.run_dir).expanduser().resolve()
    output_dir = Path(args.output_dir).expanduser().resolve() if args.output_dir else run_dir / "fbbr_opiv2_plots"
    output_dir.mkdir(parents=True, exist_ok=True)
    bins_by_flow = {flow_id(path): read_csv(path) for path in run_dir.glob("flow*_fbbr_opiv2_bins.csv")}
    blocks_by_flow = {flow_id(path): read_csv(path) for path in run_dir.glob("flow*_fbbr_opiv2_blocks.csv")}
    cruises_by_flow = {flow_id(path): read_csv(path) for path in run_dir.glob("flow*_fbbr_opiv2_cruises.csv")}
    flow_ids = sorted(set(bins_by_flow) | set(blocks_by_flow) | set(cruises_by_flow))
    if not flow_ids:
        raise SystemExit(f"No F-BBR OPIv2 CSV traces found in {run_dir}")
    outputs = []
    for fid in flow_ids:
        outputs.append(plot_flow(output_dir, fid, bins_by_flow.get(fid, []),
                                 blocks_by_flow.get(fid, []), cruises_by_flow.get(fid, []),
                                 args.fair_share_bps))
    outputs.append(plot_invalid_reasons(output_dir, blocks_by_flow))
    (output_dir / "manifest.txt").write_text("\n".join(str(path.name) for path in outputs) + "\n",
                                             encoding="utf-8")
    print(f"F-BBR OPI v3 diagnostics written to: {output_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
