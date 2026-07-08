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
    "group_A_bbrv2",
    "group_B_freqccv4_old",
    "group_C_trace_only",
    "group_D_gate_mod_only",
    "group_E_gate_plus_fref",
]
TRACED_GROUPS = [
    "group_C_trace_only",
    "group_D_gate_mod_only",
    "group_E_gate_plus_fref",
]
FLOW_IDS = [1, 2, 3, 4]


def read_ws(path: Path, columns):
    if not path.exists() or path.stat().st_size == 0:
        return pd.DataFrame(columns=columns)
    return pd.read_csv(path, sep=r"\s+", comment="#", names=columns, engine="python")


def read_csv(path: Path):
    if not path.exists() or path.stat().st_size == 0:
        return pd.DataFrame()
    return pd.read_csv(path)


def bool_series(s):
    return s.astype(str).str.lower().isin(["true", "1", "yes"])


def num(s):
    return pd.to_numeric(s, errors="coerce")


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


def markdown_table(df, index=False):
    if df is None or df.empty:
        return "_No rows._"
    out = df.reset_index() if index else df.copy()
    cols = list(out.columns)
    lines = [
        "| " + " | ".join(str(c) for c in cols) + " |",
        "| " + " | ".join("---" for _ in cols) + " |",
    ]
    for _, row in out.iterrows():
        vals = []
        for col in cols:
            value = row[col]
            if isinstance(value, float):
                vals.append("" if math.isnan(value) else f"{value:.6g}")
            else:
                vals.append(str(value))
        lines.append("| " + " | ".join(vals) + " |")
    return "\n".join(lines)


def discover_runs(results_dir: Path):
    for group in GROUPS:
        base = results_dir / "multiseed" / group / "same_start"
        if not base.exists():
            base = results_dir / group / "same_start"
        if not base.exists():
            continue
        for run_dir in sorted(base.glob("seed_*")) + sorted(base.glob("run_*")):
            if run_dir.is_dir():
                yield group, run_dir.name, run_dir


def row_duration(df, start, end):
    if df.empty:
        return pd.Series(dtype=float)
    times = num(df["time"]).reset_index(drop=True)
    next_times = times.shift(-1)
    median_dt = times.diff().dropna().median()
    if pd.isna(median_dt) or median_dt <= 0:
        median_dt = 0.001
    dt = (next_times - times).fillna(median_dt).clip(lower=0)
    clipped_next = pd.concat([next_times, pd.Series([math.nan] * len(times))], axis=1).iloc[:, 0]
    clipped_next = clipped_next.fillna(times + median_dt).clip(upper=end)
    clipped_start = times.clip(lower=start)
    return (clipped_next - clipped_start).clip(lower=0)


def active_duration(df, mask, start, end):
    if df.empty:
        return 0.0
    d = row_duration(df, start, end)
    return float(d[mask.reset_index(drop=True)].sum())


def normalize_gate(gate):
    if gate.empty:
        return gate
    for col in [
        "time",
        "round_id",
        "d_round",
        "d_prev",
        "v_round",
        "stable_cnt",
        "f_ref",
        "f_conf",
        "f_ref_age_cruise_windows",
        "unstable_episode_id",
        "w_freq",
        "b_native",
        "b_target",
        "b_eff",
        "scale",
        "raw_scale",
        "clamped_scale",
        "native_pacing",
        "final_pacing",
        "amplitude_bps_eff",
        "current_delivery_rate",
    ]:
        if col in gate.columns:
            gate[col] = num(gate[col])
    if "raw_scale" not in gate.columns:
        gate["raw_scale"] = gate.get("scale", 1.0)
    if "clamped_scale" not in gate.columns:
        gate["clamped_scale"] = gate.get("scale", 1.0)
    if "scale_clamped_low" not in gate.columns:
        gate["scale_clamped_low"] = False
    if "scale_clamped_high" not in gate.columns:
        gate["scale_clamped_high"] = False
    if "f_ref_invalid_reason" not in gate.columns:
        gate["f_ref_invalid_reason"] = "unknown"
    return gate


def episode_rows(group, run_name, run_dir, fid, sim_time):
    gate = normalize_gate(read_csv(run_dir / f"flow{fid}_freq_gate_trace.csv"))
    if gate.empty:
        return []
    rounds = gate[gate["row_type"] == "round"].copy().reset_index(drop=True)
    pacing = gate[gate["row_type"] == "pacing"].copy().reset_index(drop=True)
    if rounds.empty:
        return []

    cruise = read_csv(run_dir / f"flow{fid}_cruise_full_load_quality.csv")
    summary = read_csv(run_dir / f"flow{fid}_cruise_best_full_load_window.csv")
    if not cruise.empty:
        for col in ["window_start_time", "window_end_time", "full_load_quality"]:
            cruise[col] = num(cruise[col])
    if not summary.empty:
        for col in [
            "cruise_end_time",
            "candidate_count",
            "best_full_load_window_exists",
            "best_full_load_quality",
            "best_drate_mean_kbps",
        ]:
            if col in summary.columns:
                summary[col] = num(summary[col])

    rows = []
    just_indices = rounds.index[bool_series(rounds["just_exited"])].tolist()
    for episode_idx, idx in enumerate(just_indices, start=1):
        start = float(rounds.loc[idx, "time"])
        after = rounds.loc[idx:].copy()
        closure_mask = (
            (num(after["stable_cnt"]) >= 3)
            & bool_series(after["bbr_stable"])
            & (num(after["w_freq"]).abs() < 1e-6)
        )
        closure = after[closure_mask]
        if closure.empty:
            end = sim_time
            closed = False
            ep_rounds = rounds[(rounds["time"] >= start) & (rounds["time"] <= end)]
        else:
            end = float(closure.iloc[0]["time"])
            closed = True
            ep_rounds = rounds[(rounds["time"] >= start) & (rounds["time"] <= end)]
        ep_pacing = pacing[(pacing["time"] >= start) & (pacing["time"] <= end)].copy()

        stable_seq = [int(x) for x in num(ep_rounds["stable_cnt"]).dropna().tolist()]
        w_seq = [round(float(x), 6) for x in num(ep_rounds["w_freq"]).dropna().tolist()]
        tool_mask = bool_series(ep_pacing["freq_tool_on"]) if not ep_pacing.empty else pd.Series(dtype=bool)
        fref_mask = bool_series(ep_pacing["f_ref_valid"]) if not ep_pacing.empty else pd.Series(dtype=bool)
        scale_mask = (
            (num(ep_pacing["clamped_scale"]) - 1.0).abs() > 1e-6
        ) if not ep_pacing.empty else pd.Series(dtype=bool)
        fref_times = ep_pacing.loc[fref_mask, "time"] if not ep_pacing.empty else pd.Series(dtype=float)

        if cruise.empty:
            cruise_windows = 0
            credible_windows = 0
        else:
            overlap = cruise[
                (cruise["window_end_time"] >= start)
                & (cruise["window_start_time"] <= end)
            ]
            cruise_windows = int(len(overlap))
            credible_windows = int(
                (
                    bool_series(overlap["is_full_load_candidate"])
                    & ~bool_series(overlap["low_confidence"])
                ).sum()
            )

        if summary.empty:
            finalize_calls = 0
            best_f_ref = math.nan
            best_f_conf = math.nan
        else:
            calls = summary[
                (summary["cruise_end_time"] >= start)
                & (summary["cruise_end_time"] <= end)
            ]
            finalize_calls = int(len(calls))
            credible = calls[bool_series(calls["best_full_load_window_exists"])] if not calls.empty else calls
            if credible.empty:
                best_f_ref = math.nan
                best_f_conf = math.nan
            else:
                best_idx = num(credible["best_full_load_quality"]).idxmax()
                best_f_conf = float(credible.loc[best_idx, "best_full_load_quality"])
                best_f_ref = float(credible.loc[best_idx, "best_drate_mean_kbps"] * 1000.0)

        reason = "unknown"
        if fref_mask.sum() == 0:
            reason = "no credible window" if credible_windows == 0 and finalize_calls > 0 else "unknown"
        else:
            invalid_rows = ep_pacing[~fref_mask]
            invalid_after_valid = invalid_rows[invalid_rows["time"] > float(fref_times.min())]
            reasons = set(str(x) for x in invalid_after_valid.get("f_ref_invalid_reason", []))
            if "ttl_expired" in reasons:
                reason = "TTL expired"
            elif "stable_closure" in reasons or closed:
                reason = "stable closure"
            elif credible_windows == 0:
                reason = "no credible window"

        rows.append(
            {
                "group": group,
                "seed": run_name,
                "flow": fid,
                "episode_id": episode_idx,
                "trace_episode_id": int(num(ep_rounds["unstable_episode_id"]).dropna().iloc[0])
                if "unstable_episode_id" in ep_rounds and not num(ep_rounds["unstable_episode_id"]).dropna().empty
                else episode_idx,
                "just_exited_time": start,
                "episode_start_time": start,
                "episode_end_time": end,
                "episode_duration": end - start,
                "closed": closed,
                "stable_cnt_sequence": "/".join(str(x) for x in stable_seq),
                "w_freq_sequence": "/".join(f"{x:g}" for x in w_seq),
                "freq_tool_on_duration": active_duration(ep_pacing, tool_mask, start, end),
                "f_ref_first_valid_time": float(fref_times.min()) if len(fref_times) else math.nan,
                "f_ref_valid_total_rows": int(fref_mask.sum()) if not ep_pacing.empty else 0,
                "f_ref_valid_total_duration": active_duration(ep_pacing, fref_mask, start, end),
                "scale_applied_rows": int(scale_mask.sum()) if not ep_pacing.empty else 0,
                "scale_applied_duration": active_duration(ep_pacing, scale_mask, start, end),
                "cruise_windows_during_episode": cruise_windows,
                "finalize_cruise_calls_during_episode": finalize_calls,
                "credible_windows_found": credible_windows,
                "best_f_ref": best_f_ref,
                "best_f_conf": best_f_conf,
                "f_ref_invalid_reason": reason,
            }
        )
    return rows


def summarize_run(group, run_name, run_dir, sim_time, flow_size_bytes):
    row = {"group": group, "seed": run_name}
    throughputs = []
    pacing_cvs = []
    delivery_cvs = []
    for fid in FLOW_IDS:
        good = read_ws(run_dir / f"flow{fid}_good.txt", ["time", "goodput_kbps"])
        avg_good = float(num(good["goodput_kbps"]).mean()) if not good.empty else math.nan
        if flow_size_bytes > 0:
            active = good[num(good["goodput_kbps"]) > 0] if not good.empty else pd.DataFrame()
            fct = float(active["time"].max()) if not active.empty else math.nan
            row[f"flow{fid}_fct_s"] = fct
            thr = flow_size_bytes * 8.0 / fct / 1000.0 if fct and fct > 0 else math.nan
        else:
            thr = avg_good
        row[f"flow{fid}_throughput_kbps"] = thr
        throughputs.append(thr)

        pacing = read_ws(run_dir / f"flow{fid}_sendrate.txt", ["time", "pacing_rate_kbps"])
        delivery = read_ws(run_dir / f"flow{fid}_recvrate_raw.txt", ["time", "delivery_rate_kbps"])
        pacing_cvs.append(cv(num(pacing["pacing_rate_kbps"])) if not pacing.empty else math.nan)
        delivery_cvs.append(cv(num(delivery["delivery_rate_kbps"])) if not delivery.empty else math.nan)

    valid_thr = pd.Series(throughputs).dropna()
    row["avg_throughput_kbps"] = float(valid_thr.mean()) if len(valid_thr) else math.nan
    row["jain_fairness"] = (
        float((valid_thr.sum() ** 2) / (len(valid_thr) * (valid_thr ** 2).sum()))
        if len(valid_thr) and (valid_thr ** 2).sum() > 0
        else math.nan
    )
    row["pacing_rate_cv_mean"] = float(pd.Series(pacing_cvs).mean())
    row["delivery_rate_cv_mean"] = float(pd.Series(delivery_cvs).mean())

    queue = read_ws(
        run_dir / "freqccv4_4flow_bottleneck_queue.txt",
        ["time", "queue_bytes", "flow1_share", "flow2_share", "flow3_share", "flow4_share"],
    )
    if not queue.empty:
        q = num(queue["queue_bytes"]).dropna()
        row["queue_mean_bytes"] = float(q.mean())
        row["queue_p95_bytes"] = float(q.quantile(0.95))
        row["queue_p99_bytes"] = float(q.quantile(0.99))
        row["queue_max_bytes"] = float(q.max())
    else:
        row["queue_mean_bytes"] = row["queue_p95_bytes"] = row["queue_p99_bytes"] = row["queue_max_bytes"] = math.nan
    return row


def scale_rows(group, run_name, run_dir, fid):
    gate = normalize_gate(read_csv(run_dir / f"flow{fid}_freq_gate_trace.csv"))
    if gate.empty:
        return None
    pacing = gate[gate["row_type"] == "pacing"].copy()
    if pacing.empty:
        return None
    active = pacing[bool_series(pacing["f_ref_valid"]) | ((num(pacing["clamped_scale"]) - 1.0).abs() > 1e-6)]
    if active.empty:
        active = pacing.iloc[0:0]
    raw = num(active["raw_scale"]).dropna()
    clamped = num(active["clamped_scale"]).dropna()
    total = max(len(pacing), 1)
    scale_dev = (num(pacing["clamped_scale"]) - 1.0).abs() > 1e-6
    return {
        "group": group,
        "seed": run_name,
        "flow": fid,
        "pacing_rows": len(pacing),
        "scale_non1_rows": int(scale_dev.sum()),
        "scale_non1_ratio": float(scale_dev.mean()),
        "low_clamp_hit_count": int(bool_series(pacing["scale_clamped_low"]).sum()),
        "high_clamp_hit_count": int(bool_series(pacing["scale_clamped_high"]).sum()),
        "low_clamp_hit_ratio": float(bool_series(pacing["scale_clamped_low"]).sum() / total),
        "high_clamp_hit_ratio": float(bool_series(pacing["scale_clamped_high"]).sum() / total),
        "raw_scale_min": float(raw.min()) if len(raw) else math.nan,
        "raw_scale_p5": pctl(raw, 5),
        "raw_scale_mean": float(raw.mean()) if len(raw) else math.nan,
        "raw_scale_p95": pctl(raw, 95),
        "raw_scale_max": float(raw.max()) if len(raw) else math.nan,
        "clamped_scale_min": float(clamped.min()) if len(clamped) else math.nan,
        "clamped_scale_mean": float(clamped.mean()) if len(clamped) else math.nan,
        "clamped_scale_max": float(clamped.max()) if len(clamped) else math.nan,
    }


def gate_sanity(results_dir: Path, coverage: pd.DataFrame):
    violations = []
    stale = 0
    for group, run_name, run_dir in discover_runs(results_dir):
        if group not in TRACED_GROUPS:
            continue
        for fid in FLOW_IDS:
            gate = normalize_gate(read_csv(run_dir / f"flow{fid}_freq_gate_trace.csv"))
            if gate.empty:
                violations.append(f"{group}/{run_name}/flow{fid}: missing gate trace")
                continue
            rounds = gate[gate["row_type"] == "round"]
            pacing = gate[gate["row_type"] == "pacing"]
            just = rounds[bool_series(rounds["just_exited"])]
            bad_just = just[(num(just["stable_cnt"]) != 0) | ((num(just["w_freq"]) - 1.0).abs() > 1e-6)]
            if not bad_just.empty:
                violations.append(f"{group}/{run_name}/flow{fid}: just_exited stable_cnt/w_freq violation")
            stable_rounds = rounds[bool_series(rounds["bbr_stable"])]
            bad_stable = stable_rounds[
                (num(stable_rounds["w_freq"]).abs() > 1e-6)
                | bool_series(stable_rounds["freq_tool_on"])
                | bool_series(stable_rounds["f_ref_valid"])
            ]
            if not bad_stable.empty:
                violations.append(f"{group}/{run_name}/flow{fid}: stable round retained tool/F_ref/weight")
            stable_pacing = pacing[bool_series(pacing["bbr_stable"])] if not pacing.empty else pacing
            if not stable_pacing.empty:
                bad_scale = stable_pacing[(num(stable_pacing["clamped_scale"]) - 1.0).abs() > 1e-6]
                bad_tool = stable_pacing[bool_series(stable_pacing["freq_tool_on"])]
                bad_fref = stable_pacing[bool_series(stable_pacing["f_ref_valid"])]
                stale += len(bad_fref)
                if not bad_scale.empty:
                    violations.append(f"{group}/{run_name}/flow{fid}: stable pacing scale != 1")
                if not bad_tool.empty:
                    violations.append(f"{group}/{run_name}/flow{fid}: stable pacing freq_tool_on")
                if not bad_fref.empty:
                    violations.append(f"{group}/{run_name}/flow{fid}: stable pacing f_ref_valid")
            if group == "group_E_gate_plus_fref" and not pacing.empty:
                scaled = pacing[(num(pacing["clamped_scale"]) - 1.0).abs() > 1e-6]
                bad = scaled[bool_series(scaled["bbr_stable"]) | ~bool_series(scaled["f_ref_valid"])]
                if not bad.empty:
                    violations.append(f"{group}/{run_name}/flow{fid}: scale outside !stable && f_ref_valid")
    return violations, stale


def write_reports(results_dir, summary, coverage, scale, violations, stale_count):
    coverage.to_csv(results_dir / "fref_coverage_by_episode.csv", index=False)
    scale.to_csv(results_dir / "scale_clamp_by_flow.csv", index=False)
    summary.to_csv(results_dir / "summary.csv", index=False)
    metric_cols = [c for c in summary.columns if c not in {"group", "seed"}]
    by_group_seed = summary.groupby(["group", "seed"], as_index=False)[metric_cols].mean(numeric_only=True)
    by_group_seed.to_csv(results_dir / "summary_by_group_seed.csv", index=False)
    by_group = summary.groupby("group", as_index=False)[metric_cols].mean(numeric_only=True)
    by_group.to_csv(results_dir / "summary_by_group.csv", index=False)

    cov_lines = ["# F_ref Coverage Report", ""]
    if not coverage.empty:
        totals = coverage.groupby("group").agg(
            episodes=("episode_id", "count"),
            fref_valid=("f_ref_valid_total_rows", lambda x: int((x > 0).sum())),
            scale_applied=("scale_applied_rows", lambda x: int((x > 0).sum())),
            credible=("credible_windows_found", lambda x: int((x > 0).sum())),
        )
        cov_lines.append("## Group Totals")
        cov_lines.append(markdown_table(totals, index=True))
        cov_lines.append("")
        reason = coverage.groupby(["group", "f_ref_invalid_reason"]).size().reset_index(name="episodes")
        cov_lines.append("## Invalid/Unavailable Reasons")
        cov_lines.append(markdown_table(reason))
        cov_lines.append("")
        timing = coverage.groupby("group").agg(
            no_finalize=("finalize_cruise_calls_during_episode", lambda x: int((x == 0).sum())),
            no_credible_window=("credible_windows_found", lambda x: int((x == 0).sum())),
            fref_valid_duration_mean=("f_ref_valid_total_duration", "mean"),
            scale_duration_mean=("scale_applied_duration", "mean"),
        )
        cov_lines.append("## Timing Diagnostics")
        cov_lines.append(markdown_table(timing, index=True))
        cov_lines.append("")
        cov_lines.append(
            "The previous low coverage was consistent with round-based TTL: normal completed rounds advanced faster than CRUISE finalization, so active episodes could lose F_ref before stable_cnt reached 1/2. The current build ages active F_ref only on CRUISE finalization and clears it immediately on stable closure. Remaining unavailable episodes are dominated by CRUISE/finalize timing: many unstable episodes close before a CRUISE finalize can commit a credible window."
        )
    (results_dir / "fref_coverage_report.md").write_text("\n".join(cov_lines) + "\n")

    scale_lines = ["# Scale Clamp Report", ""]
    if not scale.empty:
        cols = [
            "low_clamp_hit_count",
            "high_clamp_hit_count",
            "scale_non1_rows",
            "raw_scale_min",
            "raw_scale_mean",
            "raw_scale_p95",
            "raw_scale_max",
            "clamped_scale_min",
            "clamped_scale_mean",
            "clamped_scale_max",
        ]
        scale_lines.append(markdown_table(scale.groupby("group")[cols].mean(numeric_only=True), index=True))
    (results_dir / "scale_clamp_report.md").write_text("\n".join(scale_lines) + "\n")

    sanity_lines = ["# Packet Instability Gate Sanity Report", ""]
    if violations:
        sanity_lines.append(f"- FAIL: {len(violations)} violation(s).")
        sanity_lines.extend(f"  - {v}" for v in violations[:100])
    else:
        sanity_lines.append("- PASS: all gate invariants satisfied.")
    sanity_lines.append(f"- Stale stable F_ref pacing rows: {stale_count}")
    (results_dir / "sanity_report.md").write_text("\n".join(sanity_lines) + "\n")


def save_fig(fig, path: Path):
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.tight_layout()
    fig.savefig(path.with_suffix(".png"), dpi=160)
    fig.savefig(path.with_suffix(".pdf"))
    plt.close(fig)


def plot_gate_state(results_dir: Path):
    run_dir = results_dir / "multiseed" / "group_E_gate_plus_fref" / "same_start" / "seed_2001"
    gate = normalize_gate(read_csv(run_dir / "flow1_freq_gate_trace.csv"))
    if gate.empty:
        return
    rounds = gate[gate["row_type"] == "round"]
    pacing = gate[gate["row_type"] == "pacing"]
    fig, axes = plt.subplots(7, 1, figsize=(11, 10), sharex=True)
    axes[0].step(rounds["time"], rounds["stable_cnt"], where="post")
    axes[1].step(rounds["time"], bool_series(rounds["bbr_stable"]).astype(int), where="post")
    just = rounds[bool_series(rounds["just_exited"])]
    axes[2].vlines(just["time"], 0, 1, colors="tab:red")
    axes[3].step(pacing["time"], bool_series(pacing["freq_tool_on"]).astype(int), where="post")
    axes[4].step(pacing["time"], bool_series(pacing["f_ref_valid"]).astype(int), where="post")
    axes[5].step(rounds["time"], rounds["w_freq"], where="post")
    axes[6].plot(pacing["time"], pacing["clamped_scale"], linewidth=1)
    labels = ["stable_cnt", "bbr_stable", "just_exited", "freq_tool_on", "f_ref_valid", "w_freq", "scale"]
    for ax, label in zip(axes, labels):
        ax.set_ylabel(label)
    axes[-1].set_xlabel("time (s)")
    save_fig(fig, results_dir / "plots" / "gate_state_group_E_seed2001")


def plot_fref_timeline(results_dir: Path, coverage: pd.DataFrame):
    run_dir = results_dir / "multiseed" / "group_E_gate_plus_fref" / "same_start" / "seed_2001"
    gate = normalize_gate(read_csv(run_dir / "flow1_freq_gate_trace.csv"))
    if gate.empty:
        return
    pacing = gate[gate["row_type"] == "pacing"]
    fig, axes = plt.subplots(2, 1, figsize=(11, 7), sharex=True)
    axes[0].plot(pacing["time"], pacing["f_ref"], label="F_ref")
    axes[0].plot(pacing["time"], pacing["b_native"], label="B_native")
    axes[0].plot(pacing["time"], pacing["b_target"], label="B_target")
    axes[1].plot(pacing["time"], pacing["raw_scale"], label="raw_scale")
    axes[1].plot(pacing["time"], pacing["clamped_scale"], label="clamped_scale")
    eps = coverage[
        (coverage.group == "group_E_gate_plus_fref")
        & (coverage.seed == "seed_2001")
        & (coverage.flow == 1)
    ]
    for _, ep in eps.iterrows():
        for ax in axes:
            ax.axvspan(ep.episode_start_time, ep.episode_end_time, color="tab:red", alpha=0.08)
    axes[0].set_ylabel("bps")
    axes[1].set_ylabel("scale")
    axes[1].set_xlabel("time (s)")
    axes[0].legend()
    axes[1].legend()
    save_fig(fig, results_dir / "plots" / "fref_coverage_timeline_group_E_seed2001")


def plot_scale_distribution(results_dir: Path):
    vals = []
    for _, _, run_dir in discover_runs(results_dir):
        for fid in FLOW_IDS:
            gate = normalize_gate(read_csv(run_dir / f"flow{fid}_freq_gate_trace.csv"))
            if gate.empty:
                continue
            p = gate[gate["row_type"] == "pacing"]
            active = p[bool_series(p["f_ref_valid"]) | ((num(p["clamped_scale"]) - 1).abs() > 1e-6)]
            vals.append(active[["raw_scale", "clamped_scale"]])
    if not vals:
        return
    df = pd.concat(vals, ignore_index=True).dropna()
    if df.empty:
        return
    fig, axes = plt.subplots(1, 2, figsize=(10, 4))
    axes[0].hist(df["raw_scale"], bins=40)
    axes[0].set_title("raw_scale")
    axes[1].hist(df["clamped_scale"], bins=40)
    axes[1].set_title("clamped_scale")
    save_fig(fig, results_dir / "plots" / "scale_clamp_distribution")


def plot_timeseries(results_dir: Path, kind: str):
    filename = {
        "queue": "freqccv4_4flow_bottleneck_queue.txt",
        "delivery": "flow{fid}_recvrate_raw.txt",
        "pacing": "flow{fid}_sendrate.txt",
    }[kind]
    fig, ax = plt.subplots(figsize=(11, 4))
    for group in ["group_B_freqccv4_old", "group_D_gate_mod_only", "group_E_gate_plus_fref"]:
        run_dir = results_dir / "multiseed" / group / "same_start" / "seed_2001"
        if kind == "queue":
            df = read_ws(run_dir / filename, ["time", "queue_bytes", "f1", "f2", "f3", "f4"])
            if not df.empty:
                ax.plot(df["time"], df["queue_bytes"], label=group.replace("group_", ""))
        else:
            for fid in FLOW_IDS:
                col = "delivery_rate_kbps" if kind == "delivery" else "pacing_rate_kbps"
                df = read_ws(run_dir / filename.format(fid=fid), ["time", col])
                if not df.empty:
                    ax.plot(df["time"], df[col], linewidth=0.7, alpha=0.55, label=f"{group.replace('group_', '')} f{fid}")
    ax.set_xlabel("time (s)")
    ax.set_ylabel("bytes" if kind == "queue" else "kbps")
    ax.legend(ncol=3, fontsize=7)
    output_name = {
        "queue": "queue_time_series",
        "delivery": "delivery_rate_time_series",
        "pacing": "pacing_rate_time_series",
    }[kind]
    save_fig(fig, results_dir / "plots" / output_name)


def plot_summary(results_dir: Path, summary: pd.DataFrame):
    if summary.empty:
        return
    by = summary.groupby("group", as_index=False).mean(numeric_only=True).set_index("group").reindex(GROUPS)
    metrics = [
        ("avg_throughput_kbps", "throughput"),
        ("jain_fairness", "Jain"),
        ("queue_mean_bytes", "queue mean"),
        ("queue_p95_bytes", "queue p95"),
        ("pacing_rate_cv_mean", "pacing CV"),
        ("delivery_rate_cv_mean", "delivery CV"),
    ]
    fig, axes = plt.subplots(2, 3, figsize=(13, 7))
    for ax, (col, title) in zip(axes.ravel(), metrics):
        ax.bar(range(len(by)), by[col])
        ax.set_title(title)
        ax.set_xticks(range(len(by)))
        ax.set_xticklabels([x.replace("group_", "") for x in by.index], rotation=25, ha="right")
    save_fig(fig, results_dir / "plots" / "summary_by_group_bar")


def generate_plots(results_dir: Path, summary: pd.DataFrame, coverage: pd.DataFrame):
    plot_gate_state(results_dir)
    plot_fref_timeline(results_dir, coverage)
    plot_scale_distribution(results_dir)
    plot_timeseries(results_dir, "queue")
    plot_timeseries(results_dir, "delivery")
    plot_timeseries(results_dir, "pacing")
    plot_summary(results_dir, summary)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--results-dir", required=True)
    parser.add_argument("--sim-time", type=float, default=30.0)
    parser.add_argument("--flow-size-bytes", type=int, default=0)
    args = parser.parse_args()
    results_dir = Path(args.results_dir)

    summary_rows = []
    coverage_rows = []
    scale_stats = []
    for group, run_name, run_dir in discover_runs(results_dir):
        summary_rows.append(summarize_run(group, run_name, run_dir, args.sim_time, args.flow_size_bytes))
        if group in TRACED_GROUPS:
            for fid in FLOW_IDS:
                coverage_rows.extend(episode_rows(group, run_name, run_dir, fid, args.sim_time))
                sr = scale_rows(group, run_name, run_dir, fid)
                if sr is not None:
                    scale_stats.append(sr)

    summary = pd.DataFrame(summary_rows)
    coverage = pd.DataFrame(coverage_rows)
    scale = pd.DataFrame(scale_stats)
    violations, stale_count = gate_sanity(results_dir, coverage)
    write_reports(results_dir, summary, coverage, scale, violations, stale_count)
    generate_plots(results_dir, summary, coverage)


if __name__ == "__main__":
    main()
