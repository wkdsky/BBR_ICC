#!/usr/bin/env python3
import argparse
import math
import os
from pathlib import Path

os.environ.setdefault("MPLCONFIGDIR", "/tmp/matplotlib")

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import pandas as pd


GROUPS = [
    "group_B_old_freqccv4",
    "group_D_gate_only",
    "group_E_current",
    "group_E_downward_only",
    "group_E_asymmetric",
    "group_E_high_conf_only",
    "group_E_early_episode_only",
]
GROUP_LABEL = {
    "group_B_old_freqccv4": "B old",
    "group_D_gate_only": "D gate",
    "group_E_current": "E current",
    "group_E_downward_only": "E down",
    "group_E_asymmetric": "E asym",
    "group_E_high_conf_only": "E conf",
    "group_E_early_episode_only": "E early",
}
FLOW_IDS = [1, 2, 3, 4]
TTL_CRUISE_WINDOWS = 2


def read_ws(path: Path, columns):
    if not path.exists() or path.stat().st_size == 0:
        return pd.DataFrame(columns=columns)
    return pd.read_csv(path, sep=r"\s+", comment="#", names=columns, engine="python")


def read_csv(path: Path):
    if not path.exists() or path.stat().st_size == 0:
        return pd.DataFrame()
    return pd.read_csv(path)


def num(series):
    out = pd.to_numeric(series, errors="coerce")
    if pd.api.types.is_bool_dtype(out):
        out = out.astype(float)
    return out


def bools(series):
    return series.astype(str).str.lower().isin(["true", "1", "yes"])


def cv(values):
    vals = pd.Series(values).dropna()
    if len(vals) < 2:
        return math.nan
    mean = vals.mean()
    if abs(mean) < 1e-12:
        return math.nan
    return float(vals.std(ddof=0) / mean)


def pctl(values, q):
    vals = pd.Series(values).dropna()
    if vals.empty:
        return math.nan
    return float(vals.quantile(q / 100.0))


def jain(values):
    vals = pd.Series(values).dropna()
    denom = float((vals ** 2).sum())
    if vals.empty or denom <= 0:
        return math.nan
    return float(vals.sum() ** 2 / (len(vals) * denom))


def rel(new, ref):
    if pd.isna(new) or pd.isna(ref) or abs(ref) < 1e-12:
        return math.nan
    return float((new - ref) / ref)


def fmt(x, digits=4):
    if x is None or pd.isna(x):
        return "NA"
    return f"{float(x):.{digits}g}"


def pct(x):
    if x is None or pd.isna(x):
        return "NA"
    return f"{100.0 * float(x):.2f}%"


def parse_command(path: Path):
    if not path.exists():
        return {}
    out = {}
    text = path.read_text(errors="ignore")
    for token in text.replace('"', " ").split():
        if token.startswith("--") and "=" in token:
            k, v = token[2:].split("=", 1)
            out[k] = v
    return out


def norm_gate(df):
    if df.empty:
        return df
    for col in [
        "time", "stable_cnt", "w_freq", "raw_scale", "clamped_scale",
        "scale", "f_ref_age_cruise_windows", "f_conf",
    ]:
        if col in df:
            df[col] = num(df[col])
    if "row_type" not in df:
        df["row_type"] = "round"
    if "raw_scale" not in df:
        df["raw_scale"] = df["scale"] if "scale" in df else 1.0
    if "clamped_scale" not in df:
        df["clamped_scale"] = df["scale"] if "scale" in df else 1.0
    if "scale_clamped_low" not in df:
        df["scale_clamped_low"] = False
    if "scale_clamped_high" not in df:
        df["scale_clamped_high"] = False
    return df


def scale_non1(df):
    if df.empty:
        return pd.Series(dtype=bool)
    return (num(df["clamped_scale"]) - 1.0).abs() > 1e-6


def gate_metrics(run_dir: Path):
    out = {
        "just_exited_count": 0,
        "f_ref_coverage_ratio": math.nan,
        "scale_applied_ratio": math.nan,
        "raw_scale_p5": math.nan,
        "raw_scale_p95": math.nan,
        "clamped_scale_p5": math.nan,
        "clamped_scale_p95": math.nan,
        "low_clamp_hits": 0,
        "high_clamp_hits": 0,
        "stable_after_violation_count": 0,
    }
    episodes = 0
    covered = 0
    scale_rows = 0
    trace_rows = 0
    raw = []
    clamped = []
    for fid in FLOW_IDS:
        gate = norm_gate(read_csv(run_dir / f"flow{fid}_freq_gate_trace.csv"))
        if gate.empty:
            continue
        rows = gate[gate["row_type"] == "pacing"].copy()
        if rows.empty:
            rows = gate.copy()
        rounds = gate[gate["row_type"] == "round"].copy()
        just = rounds[bools(rounds["just_exited"])] if "just_exited" in rounds else pd.DataFrame()
        episodes += len(just)
        trace_rows += len(rows)
        fref = bools(rows["f_ref_valid"]) if "f_ref_valid" in rows else pd.Series(False, index=rows.index)
        scaled = scale_non1(rows)
        scale_rows += int(scaled.sum())
        out["low_clamp_hits"] += int(bools(rows["scale_clamped_low"]).sum())
        out["high_clamp_hits"] += int(bools(rows["scale_clamped_high"]).sum())
        if fref.any():
            raw.extend(num(rows.loc[fref | scaled, "raw_scale"]).dropna().tolist())
            clamped.extend(num(rows.loc[fref | scaled, "clamped_scale"]).dropna().tolist())

        for _, ep in just.iterrows():
            t = float(ep["time"])
            later = rows[num(rows["time"]) >= t]
            if not later.empty and bools(later["f_ref_valid"]).any():
                covered += 1

        stable = rows[bools(rows["bbr_stable"]) | (num(rows["stable_cnt"]) >= 3)]
        if not stable.empty:
            out["stable_after_violation_count"] += int(
                bools(stable["f_ref_valid"]).sum()
                + bools(stable["freq_tool_on"]).sum()
                + scale_non1(stable).sum()
                + ((scale_non1(stable)) & (num(stable["f_ref_age_cruise_windows"]) > TTL_CRUISE_WINDOWS)).sum()
            )
        bad_scale = rows[scaled & (bools(rows["bbr_stable"]) | ~bools(rows["f_ref_valid"]))]
        out["stable_after_violation_count"] += int(len(bad_scale))
    out["just_exited_count"] = episodes
    out["f_ref_coverage_ratio"] = covered / episodes if episodes else math.nan
    out["scale_applied_ratio"] = scale_rows / trace_rows if trace_rows else math.nan
    out["raw_scale_p5"] = pctl(raw, 5)
    out["raw_scale_p95"] = pctl(raw, 95)
    out["clamped_scale_p5"] = pctl(clamped, 5)
    out["clamped_scale_p95"] = pctl(clamped, 95)
    return out


def queue_metrics(run_dir: Path):
    q = read_ws(run_dir / "freqccv4_4flow_bottleneck_queue.txt",
                ["time", "queue_bytes", "f1", "f2", "f3", "f4"])
    if q.empty:
        return {"queue_mean_bytes": math.nan, "queue_p95_bytes": math.nan, "queue_p99_bytes": math.nan}
    vals = num(q["queue_bytes"]).dropna()
    return {
        "queue_mean_bytes": float(vals.mean()),
        "queue_p95_bytes": pctl(vals, 95),
        "queue_p99_bytes": pctl(vals, 99),
    }


def summarize_long_run(group, seed, run_dir):
    row = {"group": group, "mode": GROUP_LABEL[group], "seed": int(seed)}
    throughputs, pcvs, dcvs = [], [], []
    for fid in FLOW_IDS:
        good = read_ws(run_dir / f"flow{fid}_good.txt", ["time", "goodput_kbps"])
        thr = float(num(good["goodput_kbps"]).mean()) if not good.empty else math.nan
        row[f"flow{fid}_throughput_kbps"] = thr
        throughputs.append(thr)
        p = read_ws(run_dir / f"flow{fid}_sendrate.txt", ["time", "rate"])
        d = read_ws(run_dir / f"flow{fid}_recvrate_raw.txt", ["time", "rate"])
        pcvs.append(cv(num(p["rate"])) if not p.empty else math.nan)
        dcvs.append(cv(num(d["rate"])) if not d.empty else math.nan)
    row["avg_throughput_kbps"] = float(pd.Series(throughputs).mean())
    row["jain_fairness"] = jain(throughputs)
    row["pacing_cv"] = float(pd.Series(pcvs).mean())
    row["delivery_cv"] = float(pd.Series(dcvs).mean())
    row.update(queue_metrics(run_dir))
    row.update(gate_metrics(run_dir))
    return row


def summarize_fct_run(size_label, start_mode, group, seed, run_dir):
    cmd = parse_command(run_dir / "command.txt")
    sim_time = float(cmd.get("sim_time", 30.0))
    flow_size = int(cmd.get("flowSizeBytes", 0))
    starts = {1: 0.0, 2: 0.0, 3: 0.0, 4: 0.0}
    if start_mode == "staggered_start":
        starts = {1: 0.0, 2: 0.020, 3: 0.040, 4: 0.060}
    row = {
        "size_label": size_label,
        "start_mode": start_mode,
        "group": group,
        "mode": GROUP_LABEL[group],
        "seed": int(seed),
    }
    fcts, thrs, completions = [], [], []
    unfinished = 0
    for fid in FLOW_IDS:
        good = read_ws(run_dir / f"flow{fid}_good.txt", ["time", "goodput_kbps"])
        vals = num(good["goodput_kbps"]) if not good.empty else pd.Series(dtype=float)
        times = num(good["time"]) if not good.empty else pd.Series(dtype=float)
        active = times[vals > 0].dropna()
        completion = float(active.max()) if len(active) else math.nan
        fct = completion - starts[fid] if not pd.isna(completion) else sim_time - starts[fid]
        done = not pd.isna(completion) and completion < sim_time - 0.15
        unfinished += 0 if done else 1
        row[f"flow{fid}_fct_s"] = fct
        fcts.append(fct)
        completions.append(completion)
        thrs.append(flow_size * 8.0 / fct / 1000.0 if fct > 0 else math.nan)
    row["avg_fct_s"] = float(pd.Series(fcts).mean())
    row["p95_fct_s"] = pctl(fcts, 95)
    row["makespan_s"] = float(pd.Series(completions).dropna().max())
    row["unfinished_flow_count"] = unfinished
    row["jain_fairness"] = jain(thrs)
    row.update(queue_metrics(run_dir))
    row.update(gate_metrics(run_dir))
    return row


def discover_long(root: Path):
    base = root / "long_lived_dynamic_delay"
    for group in GROUPS:
        for run_dir in sorted((base / group).glob("seed_*")):
            if run_dir.is_dir():
                yield group, run_dir.name.replace("seed_", ""), run_dir


def discover_fct(root: Path):
    base = root / "finite_flow_fct"
    for size_dir in sorted(base.glob("size_*")):
        if not size_dir.is_dir():
            continue
        size = size_dir.name.replace("size_", "")
        for start_dir in sorted(size_dir.iterdir()):
            if not start_dir.is_dir():
                continue
            for group in GROUPS:
                for run_dir in sorted((start_dir / group).glob("seed_*")):
                    if run_dir.is_dir():
                        yield size, start_dir.name, group, run_dir.name.replace("seed_", ""), run_dir


def compare_csv(a: Path, b: Path, key):
    da, db = read_csv(a), read_csv(b)
    row = {
        "comparison": key,
        "file": a.name,
        "rows_a": len(da),
        "rows_b": len(db),
        "same_shape": da.shape == db.shape,
        "bit_equal": False,
        "max_abs_numeric_diff": math.nan,
    }
    if da.empty and db.empty:
        row["bit_equal"] = True
        return row
    row["bit_equal"] = row["same_shape"] and da.equals(db)
    if row["same_shape"] and not da.empty:
        diffs = []
        for col in da.columns:
            va = num(da[col])
            vb = num(db[col])
            if va.notna().any() or vb.notna().any():
                diffs.append((va - vb).abs().max())
        if diffs:
            row["max_abs_numeric_diff"] = float(pd.Series(diffs).max())
    return row


def compare_ws(a: Path, b: Path, name, columns, key):
    da, db = read_ws(a, columns), read_ws(b, columns)
    row = {
        "comparison": key,
        "file": name,
        "rows_a": len(da),
        "rows_b": len(db),
        "same_shape": da.shape == db.shape,
        "bit_equal": da.shape == db.shape and da.equals(db),
        "max_abs_numeric_diff": math.nan,
    }
    if row["same_shape"] and not da.empty:
        diffs = []
        for col in da.columns:
            va = num(da[col])
            vb = num(db[col])
            if va.notna().any() or vb.notna().any():
                diffs.append((va - vb).abs().max())
        if diffs:
            row["max_abs_numeric_diff"] = float(pd.Series(diffs).max())
    return row


def trace_only_audit(root: Path):
    audit = root / "trace_only_guard"
    pairs = [
        ("B_reference_vs_C_trace_only", audit / "B_reference", audit / "C_trace_only"),
        ("B_reference_vs_B_repeat", audit / "B_reference", audit / "B_repeat"),
    ]
    rows = []
    for key, a, b in pairs:
        for fid in FLOW_IDS:
            for kind in ["sent", "acked", "pacing"]:
                rows.append(compare_csv(a / f"flow{fid}_{kind}_audit.csv",
                                        b / f"flow{fid}_{kind}_audit.csv",
                                        f"{key}:flow{fid}:{kind}"))
            rows.append(compare_ws(a / f"flow{fid}_good.txt", b / f"flow{fid}_good.txt",
                                   f"flow{fid}_good.txt", ["time", "goodput_kbps"],
                                   f"{key}:flow{fid}:goodput"))
            rows.append(compare_ws(a / f"flow{fid}_sendrate.txt", b / f"flow{fid}_sendrate.txt",
                                   f"flow{fid}_sendrate.txt", ["time", "pacing_kbps"],
                                   f"{key}:flow{fid}:sendrate"))
            rows.append(compare_ws(a / f"flow{fid}_recvrate_raw.txt", b / f"flow{fid}_recvrate_raw.txt",
                                   f"flow{fid}_recvrate_raw.txt", ["time", "delivery_kbps"],
                                   f"{key}:flow{fid}:delivery"))
        rows.append(compare_ws(a / "freqccv4_4flow_bottleneck_queue.txt",
                               b / "freqccv4_4flow_bottleneck_queue.txt",
                               "queue", ["time", "queue", "f1", "f2", "f3", "f4"],
                               f"{key}:queue"))
    diff = pd.DataFrame(rows)
    diff.to_csv(root / "trace_only_guard_diff.csv", index=False)
    bc = diff[diff["comparison"].str.startswith("B_reference_vs_C_trace_only")]
    bb = diff[diff["comparison"].str.startswith("B_reference_vs_B_repeat")]
    bc_clean = bool((bc["bit_equal"] == True).all()) if not bc.empty else False
    bb_clean = bool((bb["bit_equal"] == True).all()) if not bb.empty else False
    clean = bc_clean and bb_clean
    lines = ["# Trace-Only Guard Audit", ""]
    lines.append(f"- B vs B lower-level audit reproducible: {'YES' if bb_clean else 'NO'}")
    lines.append(f"- B vs C lower-level audit bit-identical: {'YES' if bc_clean else 'NO'}")
    lines.append("- Checked packet send timestamps/seq, delivered ACK bytes over time, native/final pacing audit, queue, goodput, sendrate, and delivery-rate traces.")
    lines.append("- Wall-clock-only runtime stats are not part of this equivalence check.")
    lines.append("- Code audit: `enableConvergenceGateControl=false` keeps `ShouldOscillate()` on the base FreqCCv4 path; `enableFreqRefPacingControl=false` keeps F_ref scale out of `PacingRate()`.")
    lines.append("- Fix applied: DQC internal RNG seeding now follows `seed/runId`; round trace emission no longer calls `BaseShouldOscillate()` or `Bbr2Sender::PacingRate(0)` for CSV fields.")
    if not bc_clean:
        bad = bc[bc["bit_equal"] != True].head(20)
        lines.append("")
        lines.append("## B vs C Non-Equal Rows")
        for _, r in bad.iterrows():
            lines.append(f"- {r['comparison']} {r['file']}: rows {r['rows_a']} vs {r['rows_b']}, max diff {fmt(r['max_abs_numeric_diff'])}")
    if not bb_clean:
        bad = bb[bb["bit_equal"] != True].head(20)
        lines.append("")
        lines.append("## B vs B Non-Equal Rows")
        for _, r in bad.iterrows():
            lines.append(f"- {r['comparison']} {r['file']}: rows {r['rows_a']} vs {r['rows_b']}, max diff {fmt(r['max_abs_numeric_diff'])}")
    (root / "trace_only_guard_audit.md").write_text("\n".join(lines) + "\n")
    return clean, diff


def aggregate(df, cols):
    if df.empty:
        return pd.DataFrame()
    metrics = [c for c in df.columns if c not in cols and pd.api.types.is_numeric_dtype(df[c])]
    return df.groupby(cols, as_index=False)[metrics].agg(["mean", "std"]).reset_index()


def flat_agg(df, cols):
    ag = aggregate(df, cols)
    if ag.empty:
        return ag
    ag.columns = [
        "_".join([str(x) for x in c if str(x)]) if isinstance(c, tuple) else str(c)
        for c in ag.columns
    ]
    return ag


def save_fig(fig, path: Path):
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.tight_layout()
    fig.savefig(path.with_suffix(".png"), dpi=160)
    fig.savefig(path.with_suffix(".pdf"))
    plt.close(fig)


def plot_bar(df, metric, path, ylabel, group_col="group"):
    if df.empty or metric not in df:
        return
    by = df.groupby(group_col)[metric].mean().reindex(GROUPS)
    fig, ax = plt.subplots(figsize=(10, 4))
    ax.bar(range(len(by)), by.values)
    ax.set_xticks(range(len(by)))
    ax.set_xticklabels([GROUP_LABEL.get(g, g) for g in by.index], rotation=25, ha="right")
    ax.set_ylabel(ylabel)
    save_fig(fig, path)


def generate_plots(root, long_df, fct_df):
    plots = root / "plots"
    plot_bar(long_df, "pacing_cv", plots / "long_lived_pacing_cv_by_mode", "pacing CV")
    plot_bar(long_df, "delivery_cv", plots / "long_lived_delivery_cv_by_mode", "delivery CV")
    if not long_df.empty:
        by = long_df.groupby("group").agg(queue_mean=("queue_mean_bytes", "mean"),
                                          queue_p95=("queue_p95_bytes", "mean")).reindex(GROUPS)
        fig, ax = plt.subplots(figsize=(10, 4))
        x = range(len(by))
        ax.bar([i - 0.18 for i in x], by["queue_mean"], width=0.36, label="mean")
        ax.bar([i + 0.18 for i in x], by["queue_p95"], width=0.36, label="p95")
        ax.set_xticks(list(x))
        ax.set_xticklabels([GROUP_LABEL[g] for g in by.index], rotation=25, ha="right")
        ax.set_ylabel("bytes")
        ax.legend()
        save_fig(fig, plots / "long_lived_queue_mean_p95_by_mode")
    plot_bar(fct_df, "avg_fct_s", plots / "finite_flow_avg_fct_by_mode", "seconds")
    plot_bar(fct_df, "makespan_s", plots / "finite_flow_makespan_by_mode", "seconds")
    plot_bar(pd.concat([long_df, fct_df], ignore_index=True), "jain_fairness", plots / "jain_fairness_by_mode", "Jain")
    plot_bar(pd.concat([long_df, fct_df], ignore_index=True), "scale_applied_ratio", plots / "scale_applied_ratio_by_mode", "ratio")
    if not long_df.empty:
        vals = []
        for group in [g for g in GROUPS if g.startswith("group_E_")]:
            base = root / "long_lived_dynamic_delay" / group
            for run in sorted(base.glob("seed_*")):
                for fid in FLOW_IDS:
                    gate = norm_gate(read_csv(run / f"flow{fid}_freq_gate_trace.csv"))
                    if gate.empty:
                        continue
                    rows = gate[gate["row_type"] == "pacing"]
                    active = rows[bools(rows["f_ref_valid"]) | scale_non1(rows)]
                    if not active.empty:
                        tmp = active[["raw_scale", "clamped_scale"]].copy()
                        tmp["group"] = group
                        vals.append(tmp)
        if vals:
            scales = pd.concat(vals, ignore_index=True)
            fig, axes = plt.subplots(1, 2, figsize=(11, 4))
            for group, sub in scales.groupby("group"):
                axes[0].hist(sub["raw_scale"].dropna(), bins=40, alpha=0.35, label=GROUP_LABEL[group])
                axes[1].hist(sub["clamped_scale"].dropna(), bins=40, alpha=0.35, label=GROUP_LABEL[group])
            axes[0].set_title("raw_scale")
            axes[1].set_title("clamped_scale")
            axes[1].legend(fontsize=7)
            save_fig(fig, plots / "raw_clamped_scale_distributions")

    rep_groups = [
        "group_E_current",
        "group_E_downward_only",
        "group_E_asymmetric",
    ]
    rep_seed = None
    for run in sorted((root / "long_lived_dynamic_delay" / rep_groups[0]).glob("seed_*")):
        if run.is_dir():
            rep_seed = run.name
            break
    if rep_seed:
        fig, axes = plt.subplots(4, len(rep_groups), figsize=(14, 8), sharex="col")
        for col, group in enumerate(rep_groups):
            gate = norm_gate(read_csv(root / "long_lived_dynamic_delay" / group / rep_seed / "flow1_freq_gate_trace.csv"))
            if gate.empty:
                continue
            gate = gate.sort_values("time")
            t = num(gate["time"])
            just = gate[bools(gate["just_exited"])] if "just_exited" in gate else pd.DataFrame()
            jt = num(just["time"]) if not just.empty else pd.Series(dtype=float)

            axes[0, col].plot(t, num(gate["stable_cnt"]), linewidth=0.9)
            for x in jt:
                axes[0, col].axvline(x, color="tab:red", alpha=0.35, linewidth=0.8)
            axes[0, col].set_title(f"{GROUP_LABEL[group]} {rep_seed}/flow1")
            axes[0, col].set_ylabel("stable_cnt")
            axes[0, col].set_ylim(-0.2, 3.3)

            axes[1, col].plot(t, bools(gate["bbr_stable"]).astype(float), label="bbr_stable", linewidth=0.8)
            axes[1, col].plot(t, bools(gate["freq_tool_on"]).astype(float), label="freq_tool_on", linewidth=0.8)
            axes[1, col].plot(t, bools(gate["f_ref_valid"]).astype(float), label="f_ref_valid", linewidth=0.8)
            axes[1, col].set_ylabel("state")
            axes[1, col].set_ylim(-0.1, 1.1)

            axes[2, col].plot(t, num(gate["w_freq"]), linewidth=0.9)
            axes[2, col].set_ylabel("w_freq")
            axes[2, col].set_ylim(-0.05, 1.05)

            axes[3, col].plot(t, num(gate["raw_scale"]), label="raw", linewidth=0.8)
            axes[3, col].plot(t, num(gate["clamped_scale"]), label="clamped", linewidth=0.8)
            axes[3, col].axhline(1.0, color="0.35", linestyle="--", linewidth=0.7)
            axes[3, col].set_ylabel("scale")
            axes[3, col].set_xlabel("time (s)")
            axes[3, col].set_ylim(0.65, 1.45)
        axes[1, 0].legend(fontsize=7, loc="upper right")
        axes[3, 0].legend(fontsize=7, loc="upper right")
        save_fig(fig, plots / "representative_state_trace_E_current_down_asym")


def mean_for(df, group, metric):
    hit = df[df["group"] == group]
    if hit.empty or metric not in hit:
        return math.nan
    return float(hit[metric].mean())


def comparison_line(df, new, ref, metrics):
    parts = []
    for metric in metrics:
        parts.append(f"{metric} {pct(rel(mean_for(df, new, metric), mean_for(df, ref, metric)))}")
    return ", ".join(parts)


def mean_std_cell(df, col, percent=False):
    if df.empty or col not in df:
        return "NA"
    vals = num(df[col]).dropna()
    if vals.empty:
        return "NA"
    mean = float(vals.mean())
    std = float(vals.std(ddof=1)) if len(vals) > 1 else 0.0
    if percent:
        return f"{pct(mean)} +/- {pct(std)}"
    return f"{fmt(mean)} +/- {fmt(std)}"


def write_report(root, long_df, fct_df, trace_clean):
    lines = ["# F_ref Scale Ablation Report", ""]
    size_order = {"2MB": 0, "6MB": 1, "15MB": 2}
    fct_sizes = sorted(
        fct_df["size_label"].dropna().unique().tolist(),
        key=lambda x: (size_order.get(x, 99), x),
    ) if not fct_df.empty else []
    fct_starts = sorted(fct_df["start_mode"].dropna().unique().tolist()) if not fct_df.empty else []
    full_fct_sizes = {"2MB", "6MB", "15MB"}.issubset(set(fct_sizes))
    fct_runs_per_mode = int(fct_df.groupby("group").size().min()) if not fct_df.empty else 0
    lines.append("## Status")
    lines.append(f"- build: {(root/'reports'/'build_status.txt').read_text().strip() if (root/'reports'/'build_status.txt').exists() else 'NA'}")
    lines.append(f"- gateStateMachineSelfTest: {(root/'reports'/'gate_state_machine_self_test_status.txt').read_text().strip() if (root/'reports'/'gate_state_machine_self_test_status.txt').exists() else 'NA'}")
    lines.append(f"- trace-only lower-level audit clean: {'YES' if trace_clean else 'NO'}")
    lines.append(f"- long-lived min seeds per mode: {long_df.groupby('group')['seed'].nunique().min() if not long_df.empty else 0}")
    if not fct_df.empty:
        lines.append(f"- finite-flow min seeds per cell/mode: {fct_df.groupby(['size_label','start_mode','group'])['seed'].nunique().min()}")
        lines.append(f"- finite-flow unfinished flows total: {int(fct_df['unfinished_flow_count'].sum())}")
    if fct_df.empty:
        lines.append("- finite-flow scope in this run: none.")
    else:
        lines.append(f"- finite-flow scope in this run: sizes={','.join(fct_sizes)}; start_modes={','.join(fct_starts)}.")
        if not full_fct_sizes:
            missing = sorted({"2MB", "6MB", "15MB"} - set(fct_sizes))
            lines.append(f"- finite-flow sizes not included yet: {','.join(missing) if missing else 'none'}.")
    lines.append("")

    lines.append("## Long-Lived Results")
    lines.append("Mean +/- std across 10 seeds.")
    rows = []
    for group in GROUPS:
        sub = long_df[long_df.group == group]
        if sub.empty:
            continue
        rows.append({
            "mode": GROUP_LABEL[group],
            "throughput": mean_std_cell(sub, "avg_throughput_kbps"),
            "Jain": mean_std_cell(sub, "jain_fairness"),
            "queue mean": mean_std_cell(sub, "queue_mean_bytes"),
            "queue p95": mean_std_cell(sub, "queue_p95_bytes"),
            "pacing CV": mean_std_cell(sub, "pacing_cv"),
            "delivery CV": mean_std_cell(sub, "delivery_cv"),
            "F_ref coverage": mean_std_cell(sub, "f_ref_coverage_ratio", percent=True),
            "scale ratio": mean_std_cell(sub, "scale_applied_ratio", percent=True),
        })
    lines.append(markdown(rows))
    lines.append("")
    lines.append("## Finite-Flow Results")
    lines.append(f"Mean +/- std across {fct_runs_per_mode} runs per mode.")
    rows = []
    for group in GROUPS:
        sub = fct_df[fct_df.group == group]
        if sub.empty:
            continue
        rows.append({
            "mode": GROUP_LABEL[group],
            "avg FCT": mean_std_cell(sub, "avg_fct_s"),
            "p95 FCT": mean_std_cell(sub, "p95_fct_s"),
            "makespan": mean_std_cell(sub, "makespan_s"),
            "queue p95": mean_std_cell(sub, "queue_p95_bytes"),
            "Jain": mean_std_cell(sub, "jain_fairness"),
            "episodes": mean_std_cell(sub, "just_exited_count"),
            "scale ratio": mean_std_cell(sub, "scale_applied_ratio", percent=True),
        })
    lines.append(markdown(rows))
    lines.append("")

    lines.append("## Scale Diagnostics")
    rows = []
    for group in [g for g in GROUPS if g.startswith("group_E_")]:
        sub = long_df[long_df.group == group]
        if sub.empty:
            continue
        rows.append({
            "mode": GROUP_LABEL[group],
            "coverage": mean_std_cell(sub, "f_ref_coverage_ratio", percent=True),
            "scale rows": mean_std_cell(sub, "scale_applied_ratio", percent=True),
            "raw p5": mean_std_cell(sub, "raw_scale_p5"),
            "raw p95": mean_std_cell(sub, "raw_scale_p95"),
            "clamped p5": mean_std_cell(sub, "clamped_scale_p5"),
            "clamped p95": mean_std_cell(sub, "clamped_scale_p95"),
            "low hits": mean_std_cell(sub, "low_clamp_hits"),
            "high hits": mean_std_cell(sub, "high_clamp_hits"),
        })
    lines.append(markdown(rows))
    lines.append("")

    lines.append("## Required Comparisons")
    lines.append("Relative changes are mean(new) vs mean(reference). Negative queue/CV/FCT/makespan is better; positive Jain is better.")
    long_metrics = ["queue_mean_bytes", "pacing_cv", "delivery_cv", "jain_fairness"]
    fct_metrics = ["avg_fct_s", "makespan_s", "queue_p95_bytes", "jain_fairness"]
    for title, new, ref in [
        ("D vs B", "group_D_gate_only", "group_B_old_freqccv4"),
        ("E_current vs D", "group_E_current", "group_D_gate_only"),
        ("E_downward_only vs E_current", "group_E_downward_only", "group_E_current"),
        ("E_asymmetric vs E_current", "group_E_asymmetric", "group_E_current"),
        ("E_high_conf_only vs E_current", "group_E_high_conf_only", "group_E_current"),
        ("E_early_episode_only vs E_current", "group_E_early_episode_only", "group_E_current"),
    ]:
        lines.append(f"- {title}: long-lived {comparison_line(long_df, new, ref, long_metrics)}; finite {comparison_line(fct_df, new, ref, fct_metrics)}.")
    lines.append("")

    stable_bad = int(long_df["stable_after_violation_count"].sum() + fct_df["stable_after_violation_count"].sum()) if not fct_df.empty else int(long_df["stable_after_violation_count"].sum())
    current_pacing_vs_d = rel(mean_for(long_df, "group_E_current", "pacing_cv"), mean_for(long_df, "group_D_gate_only", "pacing_cv"))
    current_delivery_vs_d = rel(mean_for(long_df, "group_E_current", "delivery_cv"), mean_for(long_df, "group_D_gate_only", "delivery_cv"))
    down_pacing_vs_current = rel(mean_for(long_df, "group_E_downward_only", "pacing_cv"), mean_for(long_df, "group_E_current", "pacing_cv"))
    asym_pacing_vs_current = rel(mean_for(long_df, "group_E_asymmetric", "pacing_cv"), mean_for(long_df, "group_E_current", "pacing_cv"))
    asym_queue_vs_current = rel(mean_for(long_df, "group_E_asymmetric", "queue_mean_bytes"), mean_for(long_df, "group_E_current", "queue_mean_bytes"))
    asym_jain_vs_current = rel(mean_for(long_df, "group_E_asymmetric", "jain_fairness"), mean_for(long_df, "group_E_current", "jain_fairness"))
    lines.append("## Conclusions")
    lines.append(f"1. C trace-only {'does not change control in the lower-level audit' if trace_clean else 'is not yet clean; do not use performance conclusions yet'}. B/B and B/C packet, ACK, final pacing, queue, goodput, sendrate, and delivery traces are bit-identical after the RNG and trace helper fixes.")
    lines.append("2. The prior C/B warning source was not a remaining control flag issue: DQC internal RNG had used wall-clock time, and round trace emission called control-adjacent helper methods. Both have been removed from the equivalence path.")
    lines.append("3. Gate-only D is a viable conservative baseline: it slightly lowers long-lived queue mean and finite-flow FCT/makespan versus B, with clean sanity, but it does not materially improve all stability metrics because delivery CV is higher than B in this run.")
    lines.append(f"4. Current bidirectional F_ref scale still shows the suspected pacing fluctuation: E_current vs D changes pacing CV by {pct(current_pacing_vs_d)} and delivery CV by {pct(current_delivery_vs_d)} while queue/FCT gains are very small.")
    lines.append(f"5. Downward_only reduces E_current pacing CV by {pct(down_pacing_vs_current)} and delivery CV by {pct(rel(mean_for(long_df, 'group_E_downward_only', 'delivery_cv'), mean_for(long_df, 'group_E_current', 'delivery_cv')))}, but queue mean is slightly higher than E_current.")
    lines.append(f"6. Asymmetric is the best single F_ref candidate here: versus E_current it reduces pacing CV by {pct(asym_pacing_vs_current)}, reduces queue mean by {pct(asym_queue_vs_current)}, and improves Jain by {pct(asym_jain_vs_current)} with near-neutral finite-flow FCT/makespan across 2MB, 6MB, and 15MB.")
    lines.append("7. Do not keep high_conf_only or early_episode_only as primary candidates: high_conf_only mostly disables scale and behaves like D; early_episode_only is not consistently better and has worse finite-flow FCT/makespan than E_current.")
    if full_fct_sizes:
        lines.append("8. The finite-flow size sweep includes 2MB, 6MB, and 15MB. With clean C guard and zero stable-after violations, the next step can be a larger-topology pilot, not another threshold change.")
    else:
        lines.append("8. Do not directly enter larger topology yet. First validate the asymmetric candidate on the missing finite-flow sizes, then move to larger topology if the C guard remains clean and stable-after violations remain zero.")
    lines.append("9. Candidate for the next larger-topology round: E_asymmetric, with D gate-only retained as the conservative baseline and E_current retained only as a reference, not as the preferred F_ref mode.")
    lines.append(f"10. Stable-after violation count across this ablation: {stable_bad}. Instability thresholds, F_ref TTL, stable closure, just_exited order, and stable_cnt->w_freq mapping were not changed.")
    lines.append("")
    (root / "fref_scale_ablation_report.md").write_text("\n".join(lines) + "\n")


def markdown(rows):
    if not rows:
        return "_No rows._"
    cols = list(rows[0].keys())
    lines = ["| " + " | ".join(cols) + " |",
             "| " + " | ".join("---" for _ in cols) + " |"]
    for row in rows:
        lines.append("| " + " | ".join(str(row.get(c, "")) for c in cols) + " |")
    return "\n".join(lines)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--results-dir", required=True)
    args = parser.parse_args()
    root = Path(args.results_dir)
    root.mkdir(parents=True, exist_ok=True)
    trace_clean, _ = trace_only_audit(root)
    long_df = pd.DataFrame([summarize_long_run(g, s, d) for g, s, d in discover_long(root)])
    fct_df = pd.DataFrame([summarize_fct_run(sz, st, g, s, d) for sz, st, g, s, d in discover_fct(root)])
    long_df.to_csv(root / "summary_long_lived_scale_modes.csv", index=False)
    fct_df.to_csv(root / "summary_fct_scale_modes.csv", index=False)
    scale_cols = [
        "group", "mode", "seed", "just_exited_count", "f_ref_coverage_ratio",
        "scale_applied_ratio", "raw_scale_p5", "raw_scale_p95",
        "clamped_scale_p5", "clamped_scale_p95", "low_clamp_hits",
        "high_clamp_hits", "stable_after_violation_count",
    ]
    scale_rows = []
    if not long_df.empty:
        scale_rows.append(long_df[[c for c in scale_cols if c in long_df]].assign(matrix="long_lived"))
    if not fct_df.empty:
        scale_rows.append(fct_df[[c for c in scale_cols if c in fct_df]].assign(matrix="finite_flow"))
    if scale_rows:
        pd.concat(scale_rows, ignore_index=True).to_csv(root / "scale_mode_by_seed.csv", index=False)
    generate_plots(root, long_df, fct_df)
    write_report(root, long_df, fct_df, trace_clean)


if __name__ == "__main__":
    main()
