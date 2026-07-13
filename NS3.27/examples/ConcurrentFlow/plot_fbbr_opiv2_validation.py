#!/usr/bin/env python3
"""Generate the required F-BBR OPIv2 validation figures."""

from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


LABELS = ["IDEAL", "UNDERLOAD", "OVERLOAD", "DYNAMIC", "TRANSITION"]
COLORS = {
    "IDEAL": "#2ca02c", "UNDERLOAD": "#1f77b4", "OVERLOAD": "#d62728",
    "DYNAMIC": "#ff7f0e", "TRANSITION": "#7f7f7f",
}


def save(fig: plt.Figure, path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.tight_layout()
    fig.savefig(path, dpi=160)
    plt.close(fig)


def aggregate_figures(window: pd.DataFrame, cruise: pd.DataFrame, output: Path) -> None:
    fig, ax = plt.subplots(figsize=(7, 6))
    for label in LABELS:
        g = window[window.gt_label.eq(label)]
        if len(g):
            ax.scatter(g.gt_queue_optimality_score, g.optimality_score, s=9,
                       alpha=.35, label=label, color=COLORS[label])
    ax.set(xlabel="Independent queue optimality", ylabel="OPIv2 optimality score",
           xlim=(-.02, 1.02), ylim=(-.02, 1.02))
    ax.legend(markerscale=2)
    save(fig, output / "score_vs_ground_truth.png")

    identifiable = cruise[cruise.within_cruise_status.eq("identifiable")]
    fig, ax = plt.subplots(figsize=(7, 4))
    ax.hist(identifiable.within_cruise_spearman_queue.dropna(), bins=24,
            color="#4c78a8", edgecolor="white")
    ax.axvline(0, color="black", lw=1)
    ax.set(xlabel="Within-CRUISE Spearman rho", ylabel="CRUISE count")
    save(fig, output / "within_cruise_correlation_histogram.png")

    values = [window.loc[window.gt_label.eq(x), "optimality_score"].dropna() for x in LABELS]
    fig, ax = plt.subplots(figsize=(8, 4.5))
    ax.boxplot(values, tick_labels=LABELS, showfliers=False)
    ax.set(ylabel="OPIv2 optimality score")
    save(fig, output / "score_by_external_label.png")

    bins = pd.qcut(window.optimality_score.rank(method="first"), 10, labels=False)
    cal = window.assign(decile=bins).groupby("decile").agg(
        score=("optimality_score", "mean"),
        gt=("gt_queue_optimality_score", "mean"),
        ideal=("gt_label", lambda x: np.mean(x == "IDEAL")),
        n=("gt_label", "size"),
    )
    cal.to_csv(output / "calibration.csv")
    fig, ax = plt.subplots(figsize=(7, 4.5))
    ax.plot(cal["score"], cal["gt"], "o-", label="mean independent queue score")
    ax.plot(cal.score, cal.ideal, "s-", label="IDEAL fraction")
    ax.plot([0, 1], [0, 1], "k--", lw=1)
    ax.set(xlabel="Mean OPIv2 score by decile", ylabel="Observed value", ylim=(-.02, 1.02))
    ax.legend()
    save(fig, output / "score_calibration.png")

    published = cruise[cruise.published_valid]
    fig, ax = plt.subplots(figsize=(6, 6))
    for scenario, g in published.groupby("scenario"):
        ax.scatter(g.fair_share_bps / 1e6, g.published_bw_bps / 1e6,
                   s=16, alpha=.5, label=scenario)
    plotted_values = np.r_[published.fair_share_bps.to_numpy(float) / 1e6,
                           published.published_bw_bps.to_numpy(float) / 1e6]
    upper = max(1.0, float(np.max(plotted_values))) if len(plotted_values) else 1.0
    ax.plot([0, upper], [0, upper], "k--", lw=1)
    ax.set(xlabel="Theoretical fair share (Mbps)", ylabel="Published trusted_bw (Mbps)")
    if len(published):
        ax.legend(ncol=2)
    save(fig, output / "trusted_bw_vs_fair_share.png")

    errors = np.sort(published.trusted_bw_error_ratio.dropna().to_numpy(float))
    fig, ax = plt.subplots(figsize=(7, 4.5))
    if len(errors):
        ax.plot(errors, np.arange(1, len(errors) + 1) / len(errors))
    ax.axvline(.10, color="red", ls="--", label="10%")
    ax.set(xlabel="trusted_bw absolute percentage error", ylabel="CDF", ylim=(0, 1.02))
    ax.legend()
    save(fig, output / "trusted_bw_ape_cdf.png")


def per_flow_figures(window: pd.DataFrame, output: Path) -> None:
    time_dir = output / "per_flow_timeseries"
    scatter_dir = output / "per_flow_scatter"
    for key, group in window.groupby(["scenario", "seed", "flow_id"]):
        scenario, seed, flow = key
        g = group.sort_values("window_start_s")
        t = (g.window_start_s + g.window_end_s) / 2
        fig, axes = plt.subplots(4, 1, figsize=(11, 9), sharex=True)
        axes[0].plot(t, g.gt_q_p50_bdp, label="queue p50 / BDP")
        axes[0].plot(t, g.gt_q_p95_bdp, label="queue p95 / BDP", alpha=.7)
        axes[0].legend(loc="upper right")
        axes[1].plot(t, g.gt_utilization, label="utilization")
        axes[1].axhline(.95, color="black", ls="--", lw=.8)
        axes[1].legend(loc="upper right")
        axes[2].plot(t, g.optimality_score, label="OPIv2 score")
        axes[2].plot(t, g.gt_queue_optimality_score, label="independent score", alpha=.8)
        axes[2].legend(loc="upper right")
        axes[3].plot(t, g.gt_flow_goodput_bps / 1e6, label="receiver goodput")
        axes[3].plot(t, g.theoretical_fair_share_bps / 1e6, label="fair share")
        pub = g[g.published_valid]
        axes[3].scatter((pub.window_start_s + pub.window_end_s) / 2,
                        pub.published_bw_bps / 1e6, marker="x", color="red",
                        label="published trusted_bw")
        axes[3].set_xlabel("Simulation time (s)")
        axes[3].legend(loc="upper right")
        fig.suptitle(f"{scenario} seed {seed} flow {flow}")
        save(fig, time_dir / f"{scenario}_seed{int(seed):03d}_flow{int(flow)}.png")

        fig, ax = plt.subplots(figsize=(5.5, 5))
        for label in LABELS:
            x = g[g.gt_label.eq(label)]
            if len(x):
                ax.scatter(x.gt_queue_optimality_score, x.optimality_score,
                           s=15, alpha=.55, color=COLORS[label], label=label)
        ax.set(xlabel="Independent queue optimality", ylabel="OPIv2 optimality",
               xlim=(-.02, 1.02), ylim=(-.02, 1.02))
        ax.legend(fontsize=7)
        save(fig, scatter_dir / f"{scenario}_seed{int(seed):03d}_flow{int(flow)}.png")


def failure_case_figures(analysis: Path, output: Path) -> None:
    path = analysis / "failure_cases.csv"
    if not path.exists() or path.stat().st_size <= 1:
        return
    try:
        cases = pd.read_csv(path).head(100)
    except pd.errors.EmptyDataError:
        return
    matrix = analysis.parent
    for index, row in cases.iterrows():
        if not np.isfinite(pd.to_numeric(row.get("window_start_s"), errors="coerce")):
            continue
        candidates = list(matrix.glob(f"{row.scenario}_*/seed{int(row.seed):03d}"))
        if not candidates:
            continue
        queue_path = candidates[0] / "bottleneck_queue.csv"
        if not queue_path.exists():
            continue
        start, end = float(row.window_start_s), float(row.window_end_s)
        frequency = float(row.get("frequency_hz", 1.0))
        margin = 2 / frequency if frequency > 0 else max(end - start, 1)
        q = pd.read_csv(queue_path)
        q = q[(q.time_s >= start - margin) & (q.time_s <= end + margin)]
        if len(q) > 4000:
            q = q.iloc[:: max(1, len(q) // 4000)]
        fig, axes = plt.subplots(2, 1, figsize=(9, 5), sharex=True)
        axes[0].plot(q.time_s, q.queue_bdp)
        axes[0].set_ylabel("queue / BDP")
        axes[1].plot(q.time_s, q.egress_rate_bps / 1e6)
        axes[1].set(ylabel="egress Mbps", xlabel="Simulation time (s)")
        for ax in axes:
            ax.axvspan(start, end, color="red", alpha=.15)
        fig.suptitle(
            f"{row.failure_type}: {row.scenario} seed {int(row.seed)} "
            f"flow {int(row.flow_id)} cruise {int(row.cruise_id)}"
        )
        save(fig, output / "failure_cases" / f"case_{index:03d}.png")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("analysis_dir", type=Path)
    parser.add_argument("--output-dir", type=Path)
    args = parser.parse_args()
    analysis = args.analysis_dir.resolve()
    output = (args.output_dir or analysis / "plots").resolve()
    output.mkdir(parents=True, exist_ok=True)
    window = pd.read_csv(analysis / "window_validation.csv")
    cruise = pd.read_csv(analysis / "cruise_validation.csv")
    for name in ["shadow_window", "published_valid"]:
        if name in window:
            window[name] = window[name].astype(str).str.lower().eq("true")
    cruise["published_valid"] = cruise.published_valid.astype(str).str.lower().eq("true")
    aggregate_figures(window, cruise, output)
    per_flow_figures(window, output)
    failure_case_figures(analysis, output)
    print(output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
