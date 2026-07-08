#!/usr/bin/env python3
import argparse
import math
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import pandas as pd


GROUP_ORDER = [
    "group_A_bbrv2",
    "group_B_freqccv4_old",
    "group_C_trace_only",
    "group_D_gate_mod_only",
    "group_E_gate_plus_fref",
]
PATTERNS = ["same_start", "staggered_start"]
FLOW_IDS = [1, 2, 3, 4]


def read_ws(path: Path, columns):
    if not path.exists() or path.stat().st_size == 0:
        return pd.DataFrame(columns=columns)
    return pd.read_csv(path, sep=r"\s+", comment="#", names=columns, engine="python")


def read_gate(path: Path):
    if not path.exists() or path.stat().st_size == 0:
        return pd.DataFrame()
    return pd.read_csv(path)


def cv(series):
    s = pd.to_numeric(series, errors="coerce").dropna()
    if len(s) < 2:
        return math.nan
    mean = s.mean()
    if abs(mean) < 1e-12:
        return math.nan
    return s.std(ddof=0) / mean


def pctl(values, q):
    vals = pd.Series(values).dropna()
    if vals.empty:
        return math.nan
    return float(vals.quantile(q / 100.0))


def bool_series(s):
    return s.astype(str).str.lower().isin(["true", "1", "yes"])


def active_share(df, col, sim_time):
    if df.empty or col not in df.columns:
        return math.nan
    tmp = df[["time", col]].copy()
    tmp["time"] = pd.to_numeric(tmp["time"], errors="coerce")
    tmp = tmp.dropna(subset=["time"]).sort_values("time")
    if tmp.empty:
        return math.nan
    values = bool_series(tmp[col])
    next_t = tmp["time"].shift(-1)
    median_dt = tmp["time"].diff().dropna().median()
    if pd.isna(median_dt) or median_dt <= 0:
        median_dt = 0.001
    dt = (next_t - tmp["time"]).fillna(median_dt).clip(lower=0)
    denom = min(float(sim_time), float(tmp["time"].max() + median_dt))
    if denom <= 0:
        return math.nan
    return float(dt[values].sum() / denom)


def count_switches(df, col):
    if df.empty or col not in df.columns:
        return math.nan
    vals = bool_series(df[col]).reset_index(drop=True)
    if len(vals) < 2:
        return 0
    return int((vals != vals.shift(1)).sum() - 1)


def summarize_run(run_dir: Path, group: str, pattern: str, run: str, flow_size_bytes: int, sim_time: float):
    row = {"group": group, "pattern": pattern, "run": run}
    fcts = []
    throughputs = []
    pacing_cvs = []
    pacing_stds = []
    delivery_cvs = []
    delivery_stds = []
    bw_cvs = []
    stable_switches = []
    freq_tool_shares = []
    fref_valid_shares = []
    scale_abs_means = []
    scale_abs_maxes = []
    loss_cum_rates = []

    for fid in FLOW_IDS:
        good = read_ws(run_dir / f"flow{fid}_good.txt", ["time", "goodput_kbps"])
        if not good.empty:
            active = good[pd.to_numeric(good["goodput_kbps"], errors="coerce") > 0]
            fct = float(active["time"].max()) if not active.empty else math.nan
            avg_goodput = float(pd.to_numeric(good["goodput_kbps"], errors="coerce").mean())
        else:
            fct = math.nan
            avg_goodput = math.nan
        if math.isnan(fct):
            fct = sim_time
        fcts.append(fct)
        if flow_size_bytes > 0 and fct > 0:
            throughputs.append(flow_size_bytes * 8.0 / fct / 1000.0)
        else:
            throughputs.append(avg_goodput)
        row[f"flow{fid}_fct_s"] = fct
        row[f"flow{fid}_throughput_kbps"] = throughputs[-1]

        pacing = read_ws(run_dir / f"flow{fid}_sendrate.txt", ["time", "pacing_rate_kbps"])
        pacing_values = pd.to_numeric(pacing.get("pacing_rate_kbps", pd.Series(dtype=float)), errors="coerce").dropna()
        pacing_stds.append(float(pacing_values.std(ddof=0)) if len(pacing_values) else math.nan)
        pacing_cvs.append(cv(pacing_values))

        delivery = read_ws(run_dir / f"flow{fid}_recvrate_raw.txt", ["time", "delivery_rate_kbps"])
        delivery_values = pd.to_numeric(delivery.get("delivery_rate_kbps", pd.Series(dtype=float)), errors="coerce").dropna()
        delivery_stds.append(float(delivery_values.std(ddof=0)) if len(delivery_values) else math.nan)
        delivery_cvs.append(cv(delivery_values))

        bw = read_ws(run_dir / f"flow{fid}_bw.txt", ["time", "bandwidth_kbps"])
        bw_cvs.append(cv(bw.get("bandwidth_kbps", pd.Series(dtype=float))))

        loss = read_ws(run_dir / f"flow{fid}_lossrate.txt", ["time", "loss_rate_pct", "cum_loss_rate_pct"])
        if not loss.empty:
            loss_cum_rates.append(float(pd.to_numeric(loss["cum_loss_rate_pct"], errors="coerce").max()))

        gate = read_gate(run_dir / f"flow{fid}_freq_gate_trace.csv")
        if not gate.empty:
            round_rows = gate[gate["row_type"] == "round"] if "row_type" in gate.columns else gate
            pacing_rows = gate[gate["row_type"] == "pacing"] if "row_type" in gate.columns else gate
            stable_switches.append(count_switches(round_rows, "bbr_stable"))
            freq_tool_shares.append(active_share(pacing_rows, "freq_tool_on", sim_time))
            fref_valid_shares.append(active_share(pacing_rows, "f_ref_valid", sim_time))
            if "scale" in pacing_rows.columns:
                scale_dev = (pd.to_numeric(pacing_rows["scale"], errors="coerce") - 1.0).abs().dropna()
                scale_abs_means.append(float(scale_dev.mean()) if len(scale_dev) else math.nan)
                scale_abs_maxes.append(float(scale_dev.max()) if len(scale_dev) else math.nan)

    row["avg_fct_s"] = float(pd.Series(fcts).mean())
    row["max_fct_s"] = float(pd.Series(fcts).max())
    row["p95_fct_s"] = pctl(fcts, 95)
    row["avg_throughput_kbps"] = float(pd.Series(throughputs).mean())
    valid_thr = pd.Series(throughputs).dropna()
    if len(valid_thr) > 0 and (valid_thr ** 2).sum() > 0:
        row["jain_fairness"] = float((valid_thr.sum() ** 2) / (len(valid_thr) * (valid_thr ** 2).sum()))
    else:
        row["jain_fairness"] = math.nan
    row["pacing_rate_std_kbps_mean"] = float(pd.Series(pacing_stds).mean())
    row["pacing_rate_cv_mean"] = float(pd.Series(pacing_cvs).mean())
    row["delivery_rate_std_kbps_mean"] = float(pd.Series(delivery_stds).mean())
    row["delivery_rate_cv_mean"] = float(pd.Series(delivery_cvs).mean())
    row["maxbw_cv_mean"] = float(pd.Series(bw_cvs).mean())
    row["stable_switches_mean"] = float(pd.Series(stable_switches).mean()) if stable_switches else math.nan
    row["freq_tool_on_share_mean"] = float(pd.Series(freq_tool_shares).mean()) if freq_tool_shares else math.nan
    row["f_ref_valid_share_mean"] = float(pd.Series(fref_valid_shares).mean()) if fref_valid_shares else math.nan
    row["scale_abs_dev_mean"] = float(pd.Series(scale_abs_means).mean()) if scale_abs_means else math.nan
    row["scale_abs_dev_max"] = float(pd.Series(scale_abs_maxes).max()) if scale_abs_maxes else math.nan
    row["loss_cum_rate_pct_max"] = float(pd.Series(loss_cum_rates).max()) if loss_cum_rates else 0.0
    row["ecn_mark_count"] = math.nan

    queue = read_ws(run_dir / "freqccv4_4flow_bottleneck_queue.txt",
                    ["time", "queue_bytes", "flow1_share", "flow2_share", "flow3_share", "flow4_share"])
    if not queue.empty:
        q = pd.to_numeric(queue["queue_bytes"], errors="coerce").dropna()
        row["queue_mean_bytes"] = float(q.mean())
        row["queue_p95_bytes"] = float(q.quantile(0.95))
        row["queue_p99_bytes"] = float(q.quantile(0.99))
        row["queue_max_bytes"] = float(q.max())
    else:
        row["queue_mean_bytes"] = row["queue_p95_bytes"] = row["queue_p99_bytes"] = row["queue_max_bytes"] = math.nan
    return row


def discover_runs(results_dir: Path):
    for group in GROUP_ORDER:
        for pattern in PATTERNS:
            base = results_dir / group / pattern
            if not base.exists():
                continue
            for run_dir in sorted(base.glob("run_*")):
                if run_dir.is_dir():
                    yield group, pattern, run_dir.name, run_dir


def write_summaries(results_dir: Path, flow_size_bytes: int, sim_time: float):
    rows = [summarize_run(run_dir, group, pattern, run, flow_size_bytes, sim_time)
            for group, pattern, run, run_dir in discover_runs(results_dir)]
    summary = pd.DataFrame(rows)
    if summary.empty:
        return summary, pd.DataFrame()
    summary.to_csv(results_dir / "summary.csv", index=False)
    metric_cols = [c for c in summary.columns if c not in {"group", "pattern", "run"}]
    by_group = summary.groupby(["group", "pattern"], as_index=False)[metric_cols].mean(numeric_only=True)
    by_group.to_csv(results_dir / "summary_by_group.csv", index=False)
    return summary, by_group


def save_fig(fig, base: Path):
    fig.tight_layout()
    fig.savefig(base.with_suffix(".png"), dpi=160)
    fig.savefig(base.with_suffix(".pdf"))
    plt.close(fig)


def plot_fct(by_group: pd.DataFrame, plots_dir: Path):
    if by_group.empty:
        return
    for pattern in PATTERNS:
        data = by_group[by_group["pattern"] == pattern].set_index("group").reindex(GROUP_ORDER)
        fig, ax = plt.subplots(figsize=(10, 4))
        x = range(len(data))
        ax.bar([i - 0.18 for i in x], data["avg_fct_s"], width=0.36, label="avg FCT")
        ax.bar([i + 0.18 for i in x], data["max_fct_s"], width=0.36, label="makespan")
        ax.set_xticks(list(x))
        ax.set_xticklabels([g.replace("group_", "") for g in data.index], rotation=20, ha="right")
        ax.set_ylabel("seconds")
        ax.set_title(f"FCT / makespan - {pattern}")
        ax.legend()
        save_fig(fig, plots_dir / f"fct_bar_{pattern}")


def plot_queue(results_dir: Path, plots_dir: Path):
    for pattern in PATTERNS:
        fig, ax = plt.subplots(figsize=(10, 4))
        for group in ["group_B_freqccv4_old", "group_D_gate_mod_only", "group_E_gate_plus_fref"]:
            q = read_ws(results_dir / group / pattern / "run_1" / "freqccv4_4flow_bottleneck_queue.txt",
                        ["time", "queue_bytes", "flow1_share", "flow2_share", "flow3_share", "flow4_share"])
            if not q.empty:
                ax.plot(q["time"], q["queue_bytes"], label=group.replace("group_", ""))
        ax.set_title(f"Bottleneck queue - {pattern} run_1")
        ax.set_xlabel("time (s)")
        ax.set_ylabel("bytes")
        ax.legend()
        save_fig(fig, plots_dir / f"queue_timeseries_{pattern}")


def plot_rate_family(results_dir: Path, plots_dir: Path, kind: str):
    filename = "flow{fid}_sendrate.txt" if kind == "pacing" else "flow{fid}_recvrate_raw.txt"
    col = "pacing_rate_kbps" if kind == "pacing" else "delivery_rate_kbps"
    ylabel = "kbps"
    for pattern in PATTERNS:
        for group in ["group_B_freqccv4_old", "group_D_gate_mod_only", "group_E_gate_plus_fref"]:
            fig, ax = plt.subplots(figsize=(10, 4))
            for fid in FLOW_IDS:
                df = read_ws(results_dir / group / pattern / "run_1" / filename.format(fid=fid), ["time", col])
                if not df.empty:
                    ax.plot(df["time"], df[col], label=f"flow{fid}", linewidth=1.0)
            ax.set_title(f"{kind} rate - {group.replace('group_', '')} {pattern} run_1")
            ax.set_xlabel("time (s)")
            ax.set_ylabel(ylabel)
            ax.legend(ncol=4, fontsize=8)
            save_fig(fig, plots_dir / f"{kind}_rate_{pattern}_{group}")


def plot_gate_state(results_dir: Path, plots_dir: Path):
    for pattern in PATTERNS:
        gate = read_gate(results_dir / "group_E_gate_plus_fref" / pattern / "run_1" / "flow1_freq_gate_trace.csv")
        if gate.empty:
            continue
        round_rows = gate[gate["row_type"] == "round"].copy()
        pacing_rows = gate[gate["row_type"] == "pacing"].copy()
        fig, axes = plt.subplots(4, 1, figsize=(10, 8), sharex=True)
        if not round_rows.empty:
            axes[0].step(round_rows["time"], round_rows["stable_cnt"], where="post")
            axes[1].step(round_rows["time"], bool_series(round_rows["bbr_stable"]).astype(int), where="post")
        if not pacing_rows.empty:
            axes[2].step(pacing_rows["time"], bool_series(pacing_rows["freq_tool_on"]).astype(int), where="post")
            axes[3].plot(pacing_rows["time"], pacing_rows["w_freq"])
        axes[0].set_ylabel("stable_cnt")
        axes[1].set_ylabel("bbr_stable")
        axes[2].set_ylabel("tool_on")
        axes[3].set_ylabel("w_freq")
        axes[3].set_xlabel("time (s)")
        fig.suptitle(f"Group E gate state - {pattern} run_1 flow1")
        save_fig(fig, plots_dir / f"gate_state_group_E_{pattern}")

        fig, ax = plt.subplots(figsize=(10, 4))
        if not pacing_rows.empty:
            ax.plot(pacing_rows["time"], pacing_rows["f_ref"], label="f_ref")
            ax.plot(pacing_rows["time"], pacing_rows["b_native"], label="b_native")
            ax.plot(pacing_rows["time"], pacing_rows["b_target"], label="b_target")
        ax.set_title(f"F_ref / B_native / B_target - {pattern} run_1 flow1")
        ax.set_xlabel("time (s)")
        ax.set_ylabel("bps")
        ax.legend()
        save_fig(fig, plots_dir / f"fref_btarget_group_E_{pattern}")


def generate_plots(results_dir: Path, by_group: pd.DataFrame):
    plots_dir = results_dir / "plots"
    plots_dir.mkdir(parents=True, exist_ok=True)
    plot_fct(by_group, plots_dir)
    plot_queue(results_dir, plots_dir)
    plot_rate_family(results_dir, plots_dir, "delivery")
    plot_rate_family(results_dir, plots_dir, "pacing")
    plot_gate_state(results_dir, plots_dir)


def sanity_checks(results_dir: Path, summary: pd.DataFrame):
    lines = ["# FreqCCv4 4-flow convergence gate sanity report", ""]
    violations = []

    for group in ["group_C_trace_only", "group_D_gate_mod_only", "group_E_gate_plus_fref"]:
        for pattern in PATTERNS:
            for run_dir in sorted((results_dir / group / pattern).glob("run_*")):
                for fid in FLOW_IDS:
                    gate = read_gate(run_dir / f"flow{fid}_freq_gate_trace.csv")
                    if gate.empty:
                        violations.append(f"{group}/{pattern}/{run_dir.name}/flow{fid}: missing gate trace")
                        continue
                    round_rows = gate[gate["row_type"] == "round"]
                    pacing_rows = gate[gate["row_type"] == "pacing"]
                    just = round_rows[bool_series(round_rows["just_exited"])] if not round_rows.empty else pd.DataFrame()
                    bad_just = just[(pd.to_numeric(just["stable_cnt"], errors="coerce") != 0) |
                                    ((pd.to_numeric(just["w_freq"], errors="coerce") - 1.0).abs() > 1e-6)]
                    if not bad_just.empty:
                        violations.append(f"{group}/{pattern}/{run_dir.name}/flow{fid}: just_exited stable_cnt/w_freq violation")
                    stable3 = round_rows[pd.to_numeric(round_rows["stable_cnt"], errors="coerce") >= 3]
                    if not stable3.empty:
                        bad_stable3 = stable3[(~bool_series(stable3["bbr_stable"])) |
                                              (bool_series(stable3["freq_tool_on"])) |
                                              ((pd.to_numeric(stable3["w_freq"], errors="coerce")).abs() > 1e-6)]
                        if not bad_stable3.empty:
                            violations.append(f"{group}/{pattern}/{run_dir.name}/flow{fid}: stable_cnt==3 closure violation")
                    if group == "group_D_gate_mod_only" and not pacing_rows.empty:
                        stable_pacing = pacing_rows[bool_series(pacing_rows["bbr_stable"])]
                        bad_amp = stable_pacing[pd.to_numeric(stable_pacing["amplitude_bps_eff"], errors="coerce").abs() > 0]
                        if not bad_amp.empty:
                            violations.append(f"{group}/{pattern}/{run_dir.name}/flow{fid}: stable pacing still modulated")
                    if group == "group_E_gate_plus_fref" and not pacing_rows.empty:
                        scale_dev = (pd.to_numeric(pacing_rows["scale"], errors="coerce") - 1.0).abs()
                        active_scale = pacing_rows[scale_dev > 1e-6]
                        bad_scale = active_scale[(bool_series(active_scale["bbr_stable"])) |
                                                 (~bool_series(active_scale["f_ref_valid"]))]
                        if not bad_scale.empty:
                            violations.append(f"{group}/{pattern}/{run_dir.name}/flow{fid}: scale deviates outside !stable && f_ref_valid")

    lines.append("## Group C vs Group B Trace-Only Check")
    if not summary.empty:
        for pattern in PATTERNS:
            b = summary[(summary.group == "group_B_freqccv4_old") & (summary.pattern == pattern)]
            c = summary[(summary.group == "group_C_trace_only") & (summary.pattern == pattern)]
            merged = b.merge(c, on="run", suffixes=("_B", "_C"))
            if merged.empty:
                lines.append(f"- {pattern}: insufficient paired runs.")
                continue
            for metric in ["avg_fct_s", "queue_mean_bytes", "pacing_rate_cv_mean", "delivery_rate_cv_mean"]:
                diff = (merged[f"{metric}_B"] - merged[f"{metric}_C"]).abs()
                denom = merged[f"{metric}_B"].abs().replace(0, math.nan)
                rel = (diff / denom).max()
                lines.append(f"- {pattern} {metric}: max relative diff = {rel:.6g}")

    lines.append("")
    lines.append("## Gate State Checks")
    if violations:
        lines.append(f"- FAIL: {len(violations)} violation(s) found.")
        lines.extend([f"  - {v}" for v in violations[:50]])
    else:
        lines.append("- PASS: no just_exited, stable_cnt==3, modulation-off, or F_ref scale gating violations found.")

    lines.append("")
    lines.append("## Observed Stable Count / Weight")
    for pattern in PATTERNS:
        gate = read_gate(results_dir / "group_E_gate_plus_fref" / pattern / "run_1" / "flow1_freq_gate_trace.csv")
        if gate.empty:
            lines.append(f"- {pattern}: no Group E representative gate trace.")
            continue
        round_rows = gate[gate["row_type"] == "round"]
        observed = sorted(pd.to_numeric(round_rows["stable_cnt"], errors="coerce").dropna().unique().tolist())
        weights = sorted(round(pd.to_numeric(round_rows["w_freq"], errors="coerce").dropna().iloc[i], 4)
                         for i in range(len(round_rows))) if not round_rows.empty else []
        lines.append(f"- {pattern}: stable_cnt values observed in run_1 flow1 = {observed}; w_freq sample set = {sorted(set(weights))[:10]}")

    return "\n".join(lines) + "\n"


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--results-dir", required=True)
    parser.add_argument("--flow-size-bytes", type=int, default=15000000)
    parser.add_argument("--sim-time", type=float, default=30.0)
    args = parser.parse_args()

    results_dir = Path(args.results_dir)
    results_dir.mkdir(parents=True, exist_ok=True)
    summary, by_group = write_summaries(results_dir, args.flow_size_bytes, args.sim_time)
    generate_plots(results_dir, by_group)
    report = sanity_checks(results_dir, summary)
    (results_dir / "sanity_report.md").write_text(report)


if __name__ == "__main__":
    main()
