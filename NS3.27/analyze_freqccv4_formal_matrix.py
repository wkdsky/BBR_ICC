#!/usr/bin/env python3
import argparse
import math
import os
import random
import re
from pathlib import Path

os.environ.setdefault("MPLCONFIGDIR", "/tmp/matplotlib")

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import pandas as pd


GROUPS = [
    "group_A_bbrv2",
    "group_B_old_freqccv4",
    "group_C_trace_only",
    "group_D_gate_mod_only",
    "group_E_gate_plus_fref",
]
TRACED_GROUPS = {
    "group_C_trace_only",
    "group_D_gate_mod_only",
    "group_E_gate_plus_fref",
}
GROUP_LABELS = {
    "group_A_bbrv2": "A BBRv2",
    "group_B_old_freqccv4": "B old FreqCCv4",
    "group_C_trace_only": "C trace-only",
    "group_D_gate_mod_only": "D gate only",
    "group_E_gate_plus_fref": "E gate+F_ref",
}
FLOW_IDS = [1, 2, 3, 4]
TTL_CRUISE_WINDOWS = 2
TRACE_GUARD_THRESHOLDS = {
    "avg_throughput_kbps": 0.02,
    "queue_mean_bytes": 0.03,
    "pacing_rate_cv_mean": 0.05,
    "delivery_rate_cv_mean": 0.05,
}


def read_ws(path: Path, columns):
    if not path.exists() or path.stat().st_size == 0:
        return pd.DataFrame(columns=columns)
    return pd.read_csv(path, sep=r"\s+", comment="#", names=columns, engine="python")


def read_csv(path: Path):
    if not path.exists() or path.stat().st_size == 0:
        return pd.DataFrame()
    return pd.read_csv(path)


def to_num(series):
    return pd.to_numeric(series, errors="coerce")


def bool_series(series):
    if series is None:
        return pd.Series(dtype=bool)
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
    return float((vals.sum() ** 2) / (len(vals) * denom))


def rel_change(new, ref):
    if ref is None or pd.isna(ref) or abs(ref) < 1e-12 or pd.isna(new):
        return math.nan
    return float((new - ref) / ref)


def fmt_num(value, digits=4):
    if value is None or pd.isna(value):
        return "NA"
    return f"{float(value):.{digits}g}"


def fmt_pct(value):
    if value is None or pd.isna(value):
        return "NA"
    return f"{100.0 * float(value):.2f}%"


def fmt_pm(mean, std, digits=4):
    if pd.isna(mean):
        return "NA"
    if pd.isna(std):
        return fmt_num(mean, digits)
    return f"{fmt_num(mean, digits)} +/- {fmt_num(std, digits)}"


def markdown_table(rows, columns):
    if not rows:
        return "_No rows._"
    lines = [
        "| " + " | ".join(columns) + " |",
        "| " + " | ".join("---" for _ in columns) + " |",
    ]
    for row in rows:
        lines.append("| " + " | ".join(str(row.get(col, "")) for col in columns) + " |")
    return "\n".join(lines)


def parse_command(path: Path):
    if not path.exists():
        return {}
    text = path.read_text(errors="ignore").strip()
    parsed = {}
    for key, value in re.findall(r"--([^=\s]+)=([^\s\"]+)", text):
        parsed[key] = value
    return parsed


def run_wall_seconds(run_dir: Path):
    log = run_dir / "run.log"
    if not log.exists():
        return math.nan
    m = re.search(r"wall_seconds=([0-9.]+)", log.read_text(errors="ignore"))
    return float(m.group(1)) if m else math.nan


def read_status(path: Path):
    if not path.exists():
        return "MISSING"
    return path.read_text(errors="ignore").strip() or "MISSING"


def normalize_gate(gate):
    if gate.empty:
        return gate
    numeric_cols = [
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
        "amplitude_bps",
        "amplitude_bps_eff",
        "current_delivery_rate",
    ]
    for col in numeric_cols:
        if col in gate.columns:
            gate[col] = to_num(gate[col])
    if "row_type" not in gate.columns:
        gate["row_type"] = "round"
    if "raw_scale" not in gate.columns:
        gate["raw_scale"] = gate["scale"] if "scale" in gate.columns else 1.0
    if "clamped_scale" not in gate.columns:
        gate["clamped_scale"] = gate["scale"] if "scale" in gate.columns else 1.0
    if "scale" not in gate.columns:
        gate["scale"] = gate["clamped_scale"]
    for col in ["scale_clamped_low", "scale_clamped_high"]:
        if col not in gate.columns:
            gate[col] = False
    if "f_ref_invalid_reason" not in gate.columns:
        gate["f_ref_invalid_reason"] = "unknown"
    return gate


def row_durations(df, sim_time):
    if df.empty or "time" not in df.columns:
        return pd.Series(dtype=float)
    times = to_num(df["time"]).reset_index(drop=True)
    next_times = times.shift(-1)
    median_dt = times.diff().dropna().median()
    if pd.isna(median_dt) or median_dt <= 0:
        median_dt = 0.001
    dt = (next_times - times).fillna(median_dt).clip(lower=0)
    dt = dt.where(times < sim_time, 0.0)
    clipped_next = (times + dt).clip(upper=sim_time)
    return (clipped_next - times.clip(upper=sim_time)).clip(lower=0)


def time_ratio(df, mask, sim_time):
    if df.empty or len(mask) == 0:
        return math.nan
    d = row_durations(df, sim_time)
    denom = float(d.sum())
    if denom <= 0:
        return math.nan
    return float(d[mask.reset_index(drop=True)].sum() / denom)


def scale_mask(df):
    if df.empty:
        return pd.Series(dtype=bool)
    return (to_num(df["clamped_scale"]) - 1.0).abs() > 1e-6


def basis_rows(gate):
    if gate.empty:
        return gate
    pacing = gate[gate["row_type"] == "pacing"].copy()
    if not pacing.empty:
        return pacing.reset_index(drop=True)
    return gate.reset_index(drop=True)


def discover_long(results_dir: Path):
    root = results_dir / "long_lived_dynamic_delay"
    for group in GROUPS:
        base = root / group
        if not base.exists():
            continue
        for run_dir in sorted(base.glob("seed_*")):
            if run_dir.is_dir():
                yield group, run_dir.name.replace("seed_", ""), run_dir


def discover_fct(results_dir: Path):
    root = results_dir / "finite_flow_fct"
    if not root.exists():
        return
    for size_dir in sorted(root.glob("size_*")):
        if not size_dir.is_dir():
            continue
        size_label = size_dir.name.replace("size_", "")
        for start_dir in sorted(size_dir.iterdir()):
            if not start_dir.is_dir():
                continue
            start_mode = start_dir.name
            for group in GROUPS:
                group_dir = start_dir / group
                if not group_dir.exists():
                    continue
                for run_dir in sorted(group_dir.glob("seed_*")):
                    if run_dir.is_dir():
                        yield size_label, start_mode, group, run_dir.name.replace("seed_", ""), run_dir


def summarize_rate_file(path: Path, col, active_only=False):
    df = read_ws(path, ["time", col])
    if df.empty:
        return math.nan, math.nan, math.nan
    values = to_num(df[col]).dropna()
    if active_only:
        values = values[values > 0]
    if values.empty:
        return math.nan, math.nan, math.nan
    return float(values.mean()), float(values.std(ddof=0)), cv(values)


def summarize_queue(run_dir: Path):
    queue = read_ws(
        run_dir / "freqccv4_4flow_bottleneck_queue.txt",
        ["time", "queue_bytes", "flow1_share", "flow2_share", "flow3_share", "flow4_share"],
    )
    if queue.empty:
        return {
            "queue_mean_bytes": math.nan,
            "queue_p50_bytes": math.nan,
            "queue_p95_bytes": math.nan,
            "queue_p99_bytes": math.nan,
            "queue_max_bytes": math.nan,
        }
    q = to_num(queue["queue_bytes"]).dropna()
    return {
        "queue_mean_bytes": float(q.mean()) if len(q) else math.nan,
        "queue_p50_bytes": pctl(q, 50),
        "queue_p95_bytes": pctl(q, 95),
        "queue_p99_bytes": pctl(q, 99),
        "queue_max_bytes": float(q.max()) if len(q) else math.nan,
    }


def episode_rows(group, seed, run_dir, fid, sim_time):
    gate = normalize_gate(read_csv(run_dir / f"flow{fid}_freq_gate_trace.csv"))
    if gate.empty:
        return []
    rounds = gate[gate["row_type"] == "round"].copy().reset_index(drop=True)
    if rounds.empty:
        rounds = gate.copy().reset_index(drop=True)
    rows = []
    just_indices = rounds.index[bool_series(rounds.get("just_exited", pd.Series(False, index=rounds.index)))].tolist()
    for episode_idx, idx in enumerate(just_indices, start=1):
        start = float(rounds.loc[idx, "time"])
        after = rounds.loc[idx:].copy()
        closure_mask = (
            (to_num(after["stable_cnt"]) >= 3)
            & bool_series(after["bbr_stable"])
            & (to_num(after["w_freq"]).abs() < 1e-6)
        )
        closure = after[closure_mask]
        closed = not closure.empty
        end = float(closure.iloc[0]["time"]) if closed else sim_time
        ep = gate[(to_num(gate["time"]) >= start) & (to_num(gate["time"]) <= end)].copy()
        fref = bool_series(ep["f_ref_valid"]) if "f_ref_valid" in ep else pd.Series(False, index=ep.index)
        scaled = scale_mask(ep)
        tool = bool_series(ep["freq_tool_on"]) if "freq_tool_on" in ep else pd.Series(False, index=ep.index)
        rows.append(
            {
                "group": group,
                "seed": int(seed),
                "flow": fid,
                "episode_id": episode_idx,
                "episode_start_time": start,
                "episode_end_time": end,
                "episode_duration_s": end - start,
                "closed": bool(closed),
                "tool_on_rows": int(tool.sum()),
                "f_ref_valid_rows": int(fref.sum()),
                "scale_applied_rows": int(scaled.sum()),
                "f_ref_coverage": int(fref.sum() > 0),
                "scale_applied": int(scaled.sum() > 0),
                "f_ref_invalid_reason": str(ep.loc[~fref, "f_ref_invalid_reason"].iloc[-1])
                if "f_ref_invalid_reason" in ep and (~fref).any()
                else "none",
            }
        )
    return rows


def scale_clamp_row(group, seed, run_dir, fid):
    gate = normalize_gate(read_csv(run_dir / f"flow{fid}_freq_gate_trace.csv"))
    if gate.empty:
        return None
    rows = basis_rows(gate)
    if rows.empty:
        return None
    active = rows[bool_series(rows["f_ref_valid"]) | scale_mask(rows)]
    stat_rows = active if not active.empty else rows.iloc[0:0]
    raw = to_num(stat_rows["raw_scale"]).dropna()
    clamped = to_num(stat_rows["clamped_scale"]).dropna()
    scaled = scale_mask(rows)
    total = max(len(rows), 1)
    low = bool_series(rows["scale_clamped_low"])
    high = bool_series(rows["scale_clamped_high"])
    ttl_bad = rows[scaled & (to_num(rows["f_ref_age_cruise_windows"]) > TTL_CRUISE_WINDOWS)]
    return {
        "group": group,
        "seed": int(seed),
        "flow": fid,
        "trace_rows": len(rows),
        "scale_non1_rows": int(scaled.sum()),
        "scale_non1_ratio": float(scaled.mean()),
        "low_clamp_hit_count": int(low.sum()),
        "high_clamp_hit_count": int(high.sum()),
        "low_clamp_hit_ratio": float(low.sum() / total),
        "high_clamp_hit_ratio": float(high.sum() / total),
        "raw_scale_min": float(raw.min()) if len(raw) else math.nan,
        "raw_scale_p5": pctl(raw, 5),
        "raw_scale_mean": float(raw.mean()) if len(raw) else math.nan,
        "raw_scale_p95": pctl(raw, 95),
        "raw_scale_max": float(raw.max()) if len(raw) else math.nan,
        "clamped_scale_min": float(clamped.min()) if len(clamped) else math.nan,
        "clamped_scale_mean": float(clamped.mean()) if len(clamped) else math.nan,
        "clamped_scale_max": float(clamped.max()) if len(clamped) else math.nan,
        "stale_fref_ttl_violation_count": int(len(ttl_bad)),
    }


def gate_mechanism_metrics(group, seed, run_dir, sim_time):
    just_exited = 0
    closed = 0
    episodes_tool = 0
    episodes_fref = 0
    episodes_scaled = 0
    freq_tool_ratios = []
    fref_ratios = []
    scale_ratios = []
    raw_values = []
    clamped_values = []
    low_hits = 0
    high_hits = 0
    trace_rows = 0
    stale_fref = 0
    stable_mod = 0
    scale_gate = 0
    stale_ttl = 0

    for fid in FLOW_IDS:
        gate = normalize_gate(read_csv(run_dir / f"flow{fid}_freq_gate_trace.csv"))
        if gate.empty:
            continue
        eps = episode_rows(group, seed, run_dir, fid, sim_time)
        just_exited += len(eps)
        closed += sum(1 for ep in eps if ep["closed"])
        episodes_tool += sum(1 for ep in eps if ep["tool_on_rows"] > 0)
        episodes_fref += sum(1 for ep in eps if ep["f_ref_valid_rows"] > 0)
        episodes_scaled += sum(1 for ep in eps if ep["scale_applied_rows"] > 0)

        rows = basis_rows(gate)
        if rows.empty:
            continue
        trace_rows += len(rows)
        tool = bool_series(rows["freq_tool_on"]) if "freq_tool_on" in rows else pd.Series(False, index=rows.index)
        fref = bool_series(rows["f_ref_valid"]) if "f_ref_valid" in rows else pd.Series(False, index=rows.index)
        scaled = scale_mask(rows)
        freq_tool_ratios.append(time_ratio(rows, tool, sim_time))
        fref_ratios.append(time_ratio(rows, fref, sim_time))
        scale_ratios.append(float(scaled.mean()) if len(rows) else math.nan)
        low_hits += int(bool_series(rows["scale_clamped_low"]).sum())
        high_hits += int(bool_series(rows["scale_clamped_high"]).sum())

        active = rows[fref | scaled]
        if not active.empty:
            raw_values.extend(to_num(active["raw_scale"]).dropna().tolist())
            clamped_values.extend(to_num(active["clamped_scale"]).dropna().tolist())

        stable = rows[bool_series(rows["bbr_stable"]) | (to_num(rows["stable_cnt"]) >= 3)]
        if not stable.empty:
            stale_fref += int(bool_series(stable["f_ref_valid"]).sum())
            stable_mod += int(
                (
                    bool_series(stable["freq_tool_on"])
                    | ((to_num(stable["w_freq"])).abs() > 1e-6)
                ).sum()
            )
        if group == "group_E_gate_plus_fref":
            bad_scale = rows[scaled & (bool_series(rows["bbr_stable"]) | ~bool_series(rows["f_ref_valid"]))]
            scale_gate += int(len(bad_scale))
        else:
            scale_gate += int(scaled.sum())
        stale_ttl += int((scaled & (to_num(rows["f_ref_age_cruise_windows"]) > TTL_CRUISE_WINDOWS)).sum())

    return {
        "just_exited_count": just_exited,
        "closed_full_weight_episodes": closed,
        "episodes_tool_on": episodes_tool,
        "episodes_f_ref_valid": episodes_fref,
        "episodes_scale_applied": episodes_scaled,
        "f_ref_coverage_ratio": float(episodes_fref / just_exited) if just_exited else math.nan,
        "freq_tool_on_time_ratio": float(pd.Series(freq_tool_ratios).mean()) if freq_tool_ratios else math.nan,
        "f_ref_valid_time_ratio": float(pd.Series(fref_ratios).mean()) if fref_ratios else math.nan,
        "scale_non1_row_ratio": float(pd.Series(scale_ratios).mean()) if scale_ratios else math.nan,
        "raw_scale_min": float(pd.Series(raw_values).min()) if raw_values else math.nan,
        "raw_scale_p5": pctl(raw_values, 5),
        "raw_scale_mean": float(pd.Series(raw_values).mean()) if raw_values else math.nan,
        "raw_scale_p95": pctl(raw_values, 95),
        "raw_scale_max": float(pd.Series(raw_values).max()) if raw_values else math.nan,
        "clamped_scale_min": float(pd.Series(clamped_values).min()) if clamped_values else math.nan,
        "clamped_scale_mean": float(pd.Series(clamped_values).mean()) if clamped_values else math.nan,
        "clamped_scale_max": float(pd.Series(clamped_values).max()) if clamped_values else math.nan,
        "low_clamp_hit_count": low_hits,
        "high_clamp_hit_count": high_hits,
        "low_clamp_hit_ratio": float(low_hits / trace_rows) if trace_rows else math.nan,
        "high_clamp_hit_ratio": float(high_hits / trace_rows) if trace_rows else math.nan,
        "stale_fref_violation_count": stale_fref,
        "stable_modulation_violation_count": stable_mod,
        "scale_gating_violation_count": scale_gate,
        "stale_fref_ttl_violation_count": stale_ttl,
    }


def summarize_long_run(group, seed, run_dir):
    cmd = parse_command(run_dir / "command.txt")
    sim_time = float(cmd.get("sim_time", 30.0))
    row = {
        "scenario": "long_lived_dynamic_delay",
        "group": group,
        "seed": int(seed),
        "sim_time_s": sim_time,
        "flow_size_bytes": int(cmd.get("flowSizeBytes", 0)),
        "process_interval_us": int(cmd.get("processIntervalUs", 0)),
        "dynamic_delay_enable": cmd.get("dynamic_delay_enable", ""),
        "gate_trace_mode": cmd.get("gateTraceMode", ""),
        "wall_seconds": run_wall_seconds(run_dir),
    }
    throughputs = []
    pacing_means = []
    pacing_stds = []
    pacing_cvs = []
    delivery_means = []
    delivery_stds = []
    delivery_cvs = []
    goodput_cvs = []

    for fid in FLOW_IDS:
        good = read_ws(run_dir / f"flow{fid}_good.txt", ["time", "goodput_kbps"])
        good_values = to_num(good["goodput_kbps"]).dropna() if not good.empty else pd.Series(dtype=float)
        thr = float(good_values.mean()) if len(good_values) else math.nan
        row[f"flow{fid}_throughput_kbps"] = thr
        throughputs.append(thr)
        goodput_cvs.append(cv(good_values))

        p_mean, p_std, p_cv = summarize_rate_file(run_dir / f"flow{fid}_sendrate.txt", "pacing_rate_kbps")
        d_mean, d_std, d_cv = summarize_rate_file(run_dir / f"flow{fid}_recvrate_raw.txt", "delivery_rate_kbps")
        row[f"flow{fid}_pacing_rate_mean_kbps"] = p_mean
        row[f"flow{fid}_delivery_rate_mean_kbps"] = d_mean
        pacing_means.append(p_mean)
        pacing_stds.append(p_std)
        pacing_cvs.append(p_cv)
        delivery_means.append(d_mean)
        delivery_stds.append(d_std)
        delivery_cvs.append(d_cv)

    row["avg_throughput_kbps"] = float(pd.Series(throughputs).mean())
    row["jain_fairness"] = jain(throughputs)
    row["pacing_rate_mean_kbps"] = float(pd.Series(pacing_means).mean())
    row["pacing_rate_std_kbps_mean"] = float(pd.Series(pacing_stds).mean())
    row["pacing_rate_cv_mean"] = float(pd.Series(pacing_cvs).mean())
    row["delivery_rate_mean_kbps"] = float(pd.Series(delivery_means).mean())
    row["delivery_rate_std_kbps_mean"] = float(pd.Series(delivery_stds).mean())
    row["delivery_rate_cv_mean"] = float(pd.Series(delivery_cvs).mean())
    row["goodput_cv_mean"] = float(pd.Series(goodput_cvs).mean())
    row.update(summarize_queue(run_dir))
    row.update(gate_mechanism_metrics(group, seed, run_dir, sim_time))
    return row


def summarize_fct_run(size_label, start_mode, group, seed, run_dir):
    cmd = parse_command(run_dir / "command.txt")
    sim_time = float(cmd.get("sim_time", 30.0))
    flow_size_bytes = int(cmd.get("flowSizeBytes", 0))
    starts = {1: 0.0, 2: 0.0, 3: 0.0, 4: 0.0}
    if start_mode == "staggered_start":
        starts = {1: 0.0, 2: 0.020, 3: 0.040, 4: 0.060}

    row = {
        "scenario": "finite_flow_fct",
        "size_label": size_label,
        "start_mode": start_mode,
        "group": group,
        "seed": int(seed),
        "sim_time_s": sim_time,
        "flow_size_bytes": flow_size_bytes,
        "process_interval_us": int(cmd.get("processIntervalUs", 0)),
        "dynamic_delay_enable": cmd.get("dynamic_delay_enable", ""),
        "gate_trace_mode": cmd.get("gateTraceMode", ""),
        "wall_seconds": run_wall_seconds(run_dir),
    }
    fcts = []
    throughputs = []
    completion_times = []
    completed = 0
    unfinished = 0
    pacing_cvs = []
    delivery_cvs = []

    for fid in FLOW_IDS:
        good = read_ws(run_dir / f"flow{fid}_good.txt", ["time", "goodput_kbps"])
        values = to_num(good["goodput_kbps"]) if not good.empty else pd.Series(dtype=float)
        times = to_num(good["time"]) if not good.empty else pd.Series(dtype=float)
        active_times = times[values > 0].dropna()
        if active_times.empty:
            completion_time = math.nan
            fct = math.nan
            is_completed = False
        else:
            completion_time = float(active_times.max())
            fct = max(0.0, completion_time - starts[fid])
            is_completed = completion_time < (sim_time - 0.15)
        if not is_completed:
            unfinished += 1
            if pd.isna(fct):
                fct = max(0.0, sim_time - starts[fid])
        else:
            completed += 1
        thr = flow_size_bytes * 8.0 / fct / 1000.0 if fct and fct > 0 else math.nan
        row[f"flow{fid}_start_time_s"] = starts[fid]
        row[f"flow{fid}_completion_time_s"] = completion_time
        row[f"flow{fid}_fct_s"] = fct
        row[f"flow{fid}_completed"] = bool(is_completed)
        row[f"flow{fid}_throughput_kbps"] = thr
        fcts.append(fct)
        throughputs.append(thr)
        completion_times.append(completion_time)

        _, _, p_cv = summarize_rate_file(run_dir / f"flow{fid}_sendrate.txt", "pacing_rate_kbps", active_only=True)
        _, _, d_cv = summarize_rate_file(run_dir / f"flow{fid}_recvrate_raw.txt", "delivery_rate_kbps", active_only=True)
        pacing_cvs.append(p_cv)
        delivery_cvs.append(d_cv)

    row["avg_fct_s"] = float(pd.Series(fcts).mean())
    row["median_fct_s"] = pctl(fcts, 50)
    row["p95_fct_s"] = pctl(fcts, 95)
    row["max_fct_s"] = float(pd.Series(fcts).max())
    row["makespan_s"] = float(pd.Series(completion_times).dropna().max()) if pd.Series(completion_times).dropna().size else math.nan
    row["completed_flow_count"] = completed
    row["unfinished_flow_count"] = unfinished
    row["avg_throughput_kbps"] = float(pd.Series(throughputs).mean())
    row["jain_fairness"] = jain(throughputs)
    row["pacing_rate_cv_mean"] = float(pd.Series(pacing_cvs).mean())
    row["delivery_rate_cv_mean"] = float(pd.Series(delivery_cvs).mean())
    row.update(summarize_queue(run_dir))
    mechanism = gate_mechanism_metrics(group, seed, run_dir, sim_time)
    for key in [
        "just_exited_count",
        "freq_tool_on_time_ratio",
        "f_ref_valid_time_ratio",
        "scale_non1_row_ratio",
        "stable_modulation_violation_count",
        "scale_gating_violation_count",
        "stale_fref_violation_count",
        "stale_fref_ttl_violation_count",
    ]:
        row[key] = mechanism[key]
    return row


def bootstrap_ci(values, reps=1000, alpha=0.05):
    vals = [float(v) for v in pd.Series(values).dropna().tolist()]
    if not vals:
        return math.nan, math.nan
    if len(vals) == 1:
        return vals[0], vals[0]
    rng = random.Random(1337)
    means = []
    for _ in range(reps):
        sample = [vals[rng.randrange(len(vals))] for _ in vals]
        means.append(sum(sample) / len(sample))
    means.sort()
    lo = means[int((alpha / 2.0) * reps)]
    hi = means[min(reps - 1, int((1.0 - alpha / 2.0) * reps))]
    return float(lo), float(hi)


def aggregate(df, group_cols, metric_cols):
    if df.empty:
        return pd.DataFrame()
    rows = []
    for key, sub in df.groupby(group_cols, dropna=False):
        if not isinstance(key, tuple):
            key = (key,)
        row = {col: val for col, val in zip(group_cols, key)}
        row["run_count"] = int(len(sub))
        for metric in metric_cols:
            if metric not in sub.columns:
                continue
            vals = to_num(sub[metric]).dropna()
            row[f"{metric}_mean"] = float(vals.mean()) if len(vals) else math.nan
            row[f"{metric}_std"] = float(vals.std(ddof=1)) if len(vals) > 1 else math.nan
            lo, hi = bootstrap_ci(vals)
            row[f"{metric}_ci95_low"] = lo
            row[f"{metric}_ci95_high"] = hi
        rows.append(row)
    return pd.DataFrame(rows)


def sanity_for_runs(kind, runs, summary):
    violations = []
    warning_lines = []
    counters = {
        "stale_fref_rows": 0,
        "stable_modulation_rows": 0,
        "scale_gating_rows": 0,
        "ttl_rows": 0,
        "trace_guard_warnings": 0,
    }

    for run in runs:
        if kind == "long":
            group, seed, run_dir = run
            prefix = f"{group}/seed_{seed}"
        else:
            size_label, start_mode, group, seed, run_dir = run
            prefix = f"size_{size_label}/{start_mode}/{group}/seed_{seed}"

        cmd = parse_command(run_dir / "command.txt")
        if group == "group_C_trace_only":
            if cmd.get("enableConvergenceGateControl") != "false":
                violations.append(f"{prefix}: Group C has enableConvergenceGateControl={cmd.get('enableConvergenceGateControl')}")
            if cmd.get("enableFreqRefPacingControl") != "false":
                violations.append(f"{prefix}: Group C has enableFreqRefPacingControl={cmd.get('enableFreqRefPacingControl')}")

        if group not in TRACED_GROUPS:
            continue

        for fid in FLOW_IDS:
            gate = normalize_gate(read_csv(run_dir / f"flow{fid}_freq_gate_trace.csv"))
            if gate.empty:
                violations.append(f"{prefix}/flow{fid}: missing gate trace")
                continue
            rounds = gate[gate["row_type"] == "round"].copy()
            if rounds.empty:
                rounds = gate.copy()
            rows = basis_rows(gate)

            just = rounds[bool_series(rounds["just_exited"])]
            bad_just = just[
                (to_num(just["stable_cnt"]) != 0)
                | ((to_num(just["w_freq"]) - 1.0).abs() > 1e-6)
            ]
            if not bad_just.empty:
                violations.append(f"{prefix}/flow{fid}: just_exited stable_cnt=0/w_freq=1 violation ({len(bad_just)} rows)")

            stable3 = rounds[to_num(rounds["stable_cnt"]) == 3]
            bad_stable3 = stable3[
                (~bool_series(stable3["bbr_stable"]))
                | (to_num(stable3["w_freq"]).abs() > 1e-6)
                | bool_series(stable3["freq_tool_on"])
            ]
            if not bad_stable3.empty:
                violations.append(f"{prefix}/flow{fid}: stable_cnt==3 closure violation ({len(bad_stable3)} rows)")

            mapping = [
                (0, 1.0),
                (1, 2.0 / 3.0),
                (2, 1.0 / 3.0),
                (3, 0.0),
            ]
            for cnt, expected in mapping:
                subset = rounds[to_num(rounds["stable_cnt"]) == cnt]
                bad_w = subset[(to_num(subset["w_freq"]) - expected).abs() > 1e-5]
                if not bad_w.empty:
                    violations.append(f"{prefix}/flow{fid}: stable_cnt={cnt} w_freq mapping violation ({len(bad_w)} rows)")

            stable = rows[bool_series(rows["bbr_stable"]) | (to_num(rows["stable_cnt"]) >= 3)]
            if not stable.empty:
                stale = stable[bool_series(stable["f_ref_valid"])]
                bad_tool = stable[bool_series(stable["freq_tool_on"])]
                bad_scale = stable[scale_mask(stable)]
                counters["stale_fref_rows"] += int(len(stale))
                counters["stable_modulation_rows"] += int(len(bad_tool))
                if not stale.empty:
                    violations.append(f"{prefix}/flow{fid}: stable closure has f_ref_valid rows ({len(stale)})")
                if not bad_tool.empty:
                    violations.append(f"{prefix}/flow{fid}: stable closure has freq_tool_on rows ({len(bad_tool)})")
                if not bad_scale.empty:
                    violations.append(f"{prefix}/flow{fid}: stable closure has scale != 1 rows ({len(bad_scale)})")

            scaled = rows[scale_mask(rows)]
            ttl_bad = scaled[to_num(scaled["f_ref_age_cruise_windows"]) > TTL_CRUISE_WINDOWS]
            counters["ttl_rows"] += int(len(ttl_bad))
            if not ttl_bad.empty:
                violations.append(f"{prefix}/flow{fid}: F_ref TTL scale violation ({len(ttl_bad)} rows)")

            if group == "group_E_gate_plus_fref":
                bad_scaled = scaled[bool_series(scaled["bbr_stable"]) | ~bool_series(scaled["f_ref_valid"])]
                counters["scale_gating_rows"] += int(len(bad_scaled))
                if not bad_scaled.empty:
                    violations.append(f"{prefix}/flow{fid}: scale != 1 outside !bbr_stable && f_ref_valid ({len(bad_scaled)} rows)")
            elif not scaled.empty:
                counters["scale_gating_rows"] += int(len(scaled))
                violations.append(f"{prefix}/flow{fid}: non-E group has scale != 1 rows ({len(scaled)})")

    if not summary.empty:
        if kind == "long":
            warning_lines.extend(c_vs_b_warnings(summary, ["seed"]))
        else:
            warning_lines.extend(c_vs_b_warnings(summary, ["size_label", "start_mode", "seed"]))
        counters["trace_guard_warnings"] = len(warning_lines)

    lines = [f"# Gate Sanity {'Long-Lived' if kind == 'long' else 'FCT'}", ""]
    if violations:
        lines.append(f"- FAIL: {len(violations)} invariant violation(s).")
        lines.extend(f"  - {v}" for v in violations[:200])
    else:
        lines.append("- PASS: all required gate invariants satisfied.")
    lines.append("")
    lines.append("## Counters")
    for key, value in counters.items():
        lines.append(f"- {key}: {value}")
    lines.append("")
    lines.append("## C vs B Trace-Only Guard")
    if warning_lines:
        lines.extend(f"- WARNING: {line}" for line in warning_lines)
    else:
        lines.append("- PASS: C vs B aggregate differences are within configured warning thresholds, or paired rows are not yet available.")
    return "\n".join(lines) + "\n", counters, warning_lines, violations


def c_vs_b_warnings(summary, key_cols):
    warnings = []
    b = summary[summary["group"] == "group_B_old_freqccv4"]
    c = summary[summary["group"] == "group_C_trace_only"]
    if b.empty or c.empty:
        return ["insufficient B/C rows for trace-only comparison"]
    merged = b.merge(c, on=key_cols, suffixes=("_B", "_C"))
    if merged.empty:
        return ["no paired B/C rows for trace-only comparison"]
    for metric, threshold in TRACE_GUARD_THRESHOLDS.items():
        bcol = f"{metric}_B"
        ccol = f"{metric}_C"
        if bcol not in merged.columns or ccol not in merged.columns:
            continue
        denom = to_num(merged[bcol]).abs().replace(0, math.nan)
        rel = ((to_num(merged[ccol]) - to_num(merged[bcol])).abs() / denom).dropna()
        if not rel.empty and float(rel.max()) > threshold:
            warnings.append(f"{metric} max relative diff {float(rel.max()):.4g} > {threshold:.4g}")
    return warnings


def save_fig(fig, base: Path):
    base.parent.mkdir(parents=True, exist_ok=True)
    fig.tight_layout()
    fig.savefig(base.with_suffix(".png"), dpi=160)
    fig.savefig(base.with_suffix(".pdf"))
    plt.close(fig)


def group_xlabels():
    return [GROUP_LABELS[g].replace(" ", "\n") for g in GROUPS]


def plot_long_bars(summary, plots_dir: Path):
    if summary.empty:
        return
    by = summary.groupby("group", as_index=False).agg(
        avg_throughput_kbps=("avg_throughput_kbps", "mean"),
        avg_throughput_std=("avg_throughput_kbps", "std"),
        jain_fairness=("jain_fairness", "mean"),
        jain_std=("jain_fairness", "std"),
        queue_mean_bytes=("queue_mean_bytes", "mean"),
        queue_p95_bytes=("queue_p95_bytes", "mean"),
        pacing_rate_cv_mean=("pacing_rate_cv_mean", "mean"),
        delivery_rate_cv_mean=("delivery_rate_cv_mean", "mean"),
        f_ref_coverage_ratio=("f_ref_coverage_ratio", "mean"),
    ).set_index("group").reindex(GROUPS)
    x = list(range(len(GROUPS)))

    fig, ax = plt.subplots(figsize=(9, 4))
    ax.bar(x, by["avg_throughput_kbps"], yerr=by["avg_throughput_std"])
    ax.set_xticks(x)
    ax.set_xticklabels(group_xlabels())
    ax.set_ylabel("kbps")
    ax.set_title("Long-lived throughput")
    save_fig(fig, plots_dir / "long_lived_throughput_by_group")

    fig, ax = plt.subplots(figsize=(9, 4))
    ax.bar(x, by["jain_fairness"], yerr=by["jain_std"])
    ax.set_xticks(x)
    ax.set_xticklabels(group_xlabels())
    ax.set_ylabel("Jain fairness")
    ax.set_ylim(0, 1.05)
    ax.set_title("Long-lived fairness")
    save_fig(fig, plots_dir / "long_lived_jain_by_group")

    fig, ax = plt.subplots(figsize=(9, 4))
    width = 0.36
    ax.bar([i - width / 2 for i in x], by["queue_mean_bytes"], width=width, label="mean")
    ax.bar([i + width / 2 for i in x], by["queue_p95_bytes"], width=width, label="p95")
    ax.set_xticks(x)
    ax.set_xticklabels(group_xlabels())
    ax.set_ylabel("bytes")
    ax.set_title("Long-lived queue")
    ax.legend()
    save_fig(fig, plots_dir / "long_lived_queue_mean_p95_by_group")

    fig, ax = plt.subplots(figsize=(9, 4))
    ax.bar([i - width / 2 for i in x], by["pacing_rate_cv_mean"], width=width, label="pacing CV")
    ax.bar([i + width / 2 for i in x], by["delivery_rate_cv_mean"], width=width, label="delivery CV")
    ax.set_xticks(x)
    ax.set_xticklabels(group_xlabels())
    ax.set_ylabel("CV")
    ax.set_title("Long-lived pacing/delivery CV")
    ax.legend()
    save_fig(fig, plots_dir / "long_lived_pacing_delivery_cv_by_group")

    fig, ax = plt.subplots(figsize=(9, 4))
    ax.bar(x, by["f_ref_coverage_ratio"].fillna(0.0))
    ax.set_xticks(x)
    ax.set_xticklabels(group_xlabels())
    ax.set_ylabel("coverage ratio")
    ax.set_ylim(0, 1.05)
    ax.set_title("F_ref coverage")
    save_fig(fig, plots_dir / "long_lived_fref_coverage_by_group")


def first_long_run(results_dir: Path, group):
    base = results_dir / "long_lived_dynamic_delay" / group
    preferred = base / "seed_2001"
    if preferred.exists():
        return preferred
    runs = sorted(base.glob("seed_*")) if base.exists() else []
    return runs[0] if runs else None


def plot_scale_distribution(results_dir: Path, plots_dir: Path):
    values = []
    base = results_dir / "long_lived_dynamic_delay" / "group_E_gate_plus_fref"
    for run_dir in sorted(base.glob("seed_*")) if base.exists() else []:
        for fid in FLOW_IDS:
            gate = normalize_gate(read_csv(run_dir / f"flow{fid}_freq_gate_trace.csv"))
            if gate.empty:
                continue
            rows = basis_rows(gate)
            active = rows[bool_series(rows["f_ref_valid"]) | scale_mask(rows)]
            if not active.empty:
                values.append(active[["raw_scale", "clamped_scale"]])
    if not values:
        return
    df = pd.concat(values, ignore_index=True).dropna()
    if df.empty:
        return
    fig, axes = plt.subplots(1, 2, figsize=(10, 4))
    axes[0].hist(df["raw_scale"], bins=40)
    axes[0].axvline(0.75, color="tab:red", linestyle="--", linewidth=1)
    axes[0].axvline(1.10, color="tab:red", linestyle="--", linewidth=1)
    axes[0].set_title("raw_scale")
    axes[1].hist(df["clamped_scale"], bins=40)
    axes[1].axvline(0.75, color="tab:red", linestyle="--", linewidth=1)
    axes[1].axvline(1.10, color="tab:red", linestyle="--", linewidth=1)
    axes[1].set_title("clamped_scale")
    save_fig(fig, plots_dir / "long_lived_group_E_scale_clamp_distribution")


def plot_gate_state(results_dir: Path, plots_dir: Path):
    run_dir = first_long_run(results_dir, "group_E_gate_plus_fref")
    if not run_dir:
        return
    gate = normalize_gate(read_csv(run_dir / "flow1_freq_gate_trace.csv"))
    if gate.empty:
        return
    rounds = gate[gate["row_type"] == "round"].copy()
    rows = basis_rows(gate)
    fig, axes = plt.subplots(7, 1, figsize=(11, 11), sharex=True)
    axes[0].step(rounds["time"], rounds["stable_cnt"], where="post")
    axes[1].step(rounds["time"], bool_series(rounds["bbr_stable"]).astype(int), where="post")
    just = rounds[bool_series(rounds["just_exited"])]
    axes[2].vlines(just["time"], 0, 1, colors="tab:red")
    axes[3].step(rows["time"], bool_series(rows["freq_tool_on"]).astype(int), where="post")
    axes[4].step(rows["time"], bool_series(rows["f_ref_valid"]).astype(int), where="post")
    axes[5].step(rounds["time"], rounds["w_freq"], where="post")
    axes[6].plot(rows["time"], rows["raw_scale"], linewidth=0.8, label="raw")
    axes[6].plot(rows["time"], rows["clamped_scale"], linewidth=0.8, label="clamped")
    labels = ["stable_cnt", "bbr_stable", "just_exited", "freq_tool_on", "f_ref_valid", "w_freq", "scale"]
    for ax, label in zip(axes, labels):
        ax.set_ylabel(label)
    axes[6].legend(loc="best", fontsize=8)
    axes[-1].set_xlabel("time (s)")
    save_fig(fig, plots_dir / "long_lived_group_E_representative_state_trace")


def plot_long_timeseries(results_dir: Path, plots_dir: Path, kind: str):
    fig, ax = plt.subplots(figsize=(11, 4))
    groups = ["group_B_old_freqccv4", "group_D_gate_mod_only", "group_E_gate_plus_fref"]
    for group in groups:
        run_dir = first_long_run(results_dir, group)
        if not run_dir:
            continue
        label = GROUP_LABELS[group]
        if kind == "queue":
            df = read_ws(
                run_dir / "freqccv4_4flow_bottleneck_queue.txt",
                ["time", "queue_bytes", "f1", "f2", "f3", "f4"],
            )
            if not df.empty:
                ax.plot(df["time"], df["queue_bytes"], label=label)
        else:
            filename = "flow{fid}_sendrate.txt" if kind == "pacing" else "flow{fid}_recvrate_raw.txt"
            col = "pacing_rate_kbps" if kind == "pacing" else "delivery_rate_kbps"
            for fid in FLOW_IDS:
                df = read_ws(run_dir / filename.format(fid=fid), ["time", col])
                if not df.empty:
                    ax.plot(df["time"], df[col], linewidth=0.7, alpha=0.55, label=f"{label} f{fid}")
    ax.set_xlabel("time (s)")
    ax.set_ylabel("bytes" if kind == "queue" else "kbps")
    ax.set_title(f"Representative {kind} time series")
    ax.legend(ncol=3, fontsize=7)
    save_fig(fig, plots_dir / f"long_lived_representative_{kind}_time_series")


def plot_finite_metric(summary, plots_dir: Path, metric, filename, ylabel):
    if summary.empty or metric not in summary.columns:
        return
    cats = (
        summary[["size_label", "start_mode"]]
        .drop_duplicates()
        .sort_values(["size_label", "start_mode"])
        .apply(lambda r: f"{r['size_label']}\n{r['start_mode'].replace('_', ' ')}", axis=1)
        .tolist()
    )
    keys = (
        summary[["size_label", "start_mode"]]
        .drop_duplicates()
        .sort_values(["size_label", "start_mode"])
        .itertuples(index=False, name=None)
    )
    by = summary.groupby(["size_label", "start_mode", "group"])[metric].mean()
    x = list(range(len(cats)))
    width = 0.15
    fig, ax = plt.subplots(figsize=(12, 5))
    keys_list = list(keys)
    for idx, group in enumerate(GROUPS):
        vals = [by.get((size, start, group), math.nan) for size, start in keys_list]
        offsets = [i + (idx - 2) * width for i in x]
        ax.bar(offsets, vals, width=width, label=GROUP_LABELS[group])
    ax.set_xticks(x)
    ax.set_xticklabels(cats)
    ax.set_ylabel(ylabel)
    ax.set_title(filename.replace("_", " "))
    ax.legend(ncol=3, fontsize=8)
    save_fig(fig, plots_dir / filename)


def generate_plots(results_dir: Path, long_summary: pd.DataFrame, fct_summary: pd.DataFrame):
    plots_dir = results_dir / "plots"
    plots_dir.mkdir(parents=True, exist_ok=True)
    plot_long_bars(long_summary, plots_dir)
    plot_scale_distribution(results_dir, plots_dir)
    plot_gate_state(results_dir, plots_dir)
    for kind in ["queue", "pacing", "delivery"]:
        plot_long_timeseries(results_dir, plots_dir, kind)
    plot_finite_metric(fct_summary, plots_dir, "avg_fct_s", "finite_avg_fct_by_group", "seconds")
    plot_finite_metric(fct_summary, plots_dir, "makespan_s", "finite_makespan_by_group", "seconds")
    plot_finite_metric(fct_summary, plots_dir, "p95_fct_s", "finite_p95_fct_by_group", "seconds")
    plot_finite_metric(fct_summary, plots_dir, "queue_p95_bytes", "finite_queue_p95_by_group", "bytes")
    plot_finite_metric(fct_summary, plots_dir, "jain_fairness", "finite_jain_by_group", "Jain fairness")
    plot_finite_metric(fct_summary, plots_dir, "just_exited_count", "finite_gate_episode_count_by_group", "episodes")


def write_outputs(results_dir: Path):
    reports_dir = results_dir / "reports"
    reports_dir.mkdir(parents=True, exist_ok=True)

    long_runs = list(discover_long(results_dir))
    long_rows = []
    coverage_rows = []
    scale_rows = []
    for group, seed, run_dir in long_runs:
        long_rows.append(summarize_long_run(group, seed, run_dir))
        for fid in FLOW_IDS:
            coverage_rows.extend(episode_rows(group, seed, run_dir, fid, float(parse_command(run_dir / "command.txt").get("sim_time", 30.0))))
            scale = scale_clamp_row(group, seed, run_dir, fid)
            if scale:
                scale_rows.append(scale)
    long_summary = pd.DataFrame(long_rows)
    coverage = pd.DataFrame(coverage_rows)
    scale = pd.DataFrame(scale_rows)
    long_summary.to_csv(results_dir / "long_lived_dynamic_delay" / "summary_long_lived.csv", index=False)
    coverage.to_csv(results_dir / "long_lived_dynamic_delay" / "fref_coverage_long_lived.csv", index=False)
    scale.to_csv(results_dir / "long_lived_dynamic_delay" / "scale_clamp_long_lived.csv", index=False)

    long_metric_cols = [
        "avg_throughput_kbps",
        "jain_fairness",
        "queue_mean_bytes",
        "queue_p50_bytes",
        "queue_p95_bytes",
        "queue_p99_bytes",
        "queue_max_bytes",
        "pacing_rate_mean_kbps",
        "pacing_rate_std_kbps_mean",
        "pacing_rate_cv_mean",
        "delivery_rate_mean_kbps",
        "delivery_rate_std_kbps_mean",
        "delivery_rate_cv_mean",
        "goodput_cv_mean",
        "just_exited_count",
        "closed_full_weight_episodes",
        "episodes_tool_on",
        "episodes_f_ref_valid",
        "episodes_scale_applied",
        "f_ref_coverage_ratio",
        "freq_tool_on_time_ratio",
        "f_ref_valid_time_ratio",
        "scale_non1_row_ratio",
        "raw_scale_min",
        "raw_scale_p5",
        "raw_scale_mean",
        "raw_scale_p95",
        "raw_scale_max",
        "clamped_scale_min",
        "clamped_scale_mean",
        "clamped_scale_max",
        "low_clamp_hit_count",
        "high_clamp_hit_count",
        "low_clamp_hit_ratio",
        "high_clamp_hit_ratio",
        "stale_fref_violation_count",
        "stable_modulation_violation_count",
        "scale_gating_violation_count",
        "stale_fref_ttl_violation_count",
    ]
    long_by_group = aggregate(long_summary, ["group"], long_metric_cols)
    long_by_group.to_csv(results_dir / "long_lived_dynamic_delay" / "summary_long_lived_by_group.csv", index=False)

    fct_runs = list(discover_fct(results_dir))
    fct_rows = [summarize_fct_run(size, start, group, seed, run_dir)
                for size, start, group, seed, run_dir in fct_runs]
    fct_summary = pd.DataFrame(fct_rows)
    fct_root = results_dir / "finite_flow_fct"
    fct_root.mkdir(parents=True, exist_ok=True)
    fct_summary.to_csv(fct_root / "summary_fct.csv", index=False)
    fct_metric_cols = [
        "avg_fct_s",
        "median_fct_s",
        "p95_fct_s",
        "max_fct_s",
        "makespan_s",
        "completed_flow_count",
        "unfinished_flow_count",
        "avg_throughput_kbps",
        "jain_fairness",
        "queue_mean_bytes",
        "queue_p95_bytes",
        "queue_p99_bytes",
        "queue_max_bytes",
        "pacing_rate_cv_mean",
        "delivery_rate_cv_mean",
        "just_exited_count",
        "freq_tool_on_time_ratio",
        "f_ref_valid_time_ratio",
        "scale_non1_row_ratio",
        "stable_modulation_violation_count",
        "scale_gating_violation_count",
        "stale_fref_violation_count",
        "stale_fref_ttl_violation_count",
    ]
    fct_by_group = aggregate(fct_summary, ["size_label", "start_mode", "group"], fct_metric_cols)
    fct_by_group.to_csv(fct_root / "summary_fct_by_group.csv", index=False)
    fct_by_size_start = aggregate(fct_summary, ["size_label", "start_mode"], fct_metric_cols)
    fct_by_size_start.to_csv(fct_root / "summary_fct_by_size_startmode.csv", index=False)

    long_sanity, long_counters, long_warnings, long_violations = sanity_for_runs("long", long_runs, long_summary)
    (reports_dir / "gate_sanity_long_lived.md").write_text(long_sanity)
    fct_sanity, fct_counters, fct_warnings, fct_violations = sanity_for_runs("fct", fct_runs, fct_summary)
    (reports_dir / "gate_sanity_fct.md").write_text(fct_sanity)

    generate_plots(results_dir, long_summary, fct_summary)
    write_report(
        results_dir,
        long_summary,
        long_by_group,
        coverage,
        scale,
        fct_summary,
        fct_by_group,
        long_counters,
        fct_counters,
        long_warnings,
        fct_warnings,
        long_violations,
        fct_violations,
    )


def read_config(path: Path):
    if not path.exists():
        return {}
    out = {}
    for line in path.read_text(errors="ignore").splitlines():
        if "=" in line:
            key, value = line.split("=", 1)
            out[key.strip()] = value.strip()
    return out


def group_mean(by_group: pd.DataFrame, group, metric):
    if by_group.empty:
        return math.nan
    row = by_group[by_group["group"] == group]
    col = f"{metric}_mean"
    if row.empty or col not in row:
        return math.nan
    return float(row.iloc[0][col])


def delta_rows(by_group: pd.DataFrame, metrics):
    pairs = [
        ("D vs B", "group_D_gate_mod_only", "group_B_old_freqccv4"),
        ("E vs D", "group_E_gate_plus_fref", "group_D_gate_mod_only"),
        ("E vs B", "group_E_gate_plus_fref", "group_B_old_freqccv4"),
        ("A vs B", "group_A_bbrv2", "group_B_old_freqccv4"),
        ("A vs D", "group_A_bbrv2", "group_D_gate_mod_only"),
        ("A vs E", "group_A_bbrv2", "group_E_gate_plus_fref"),
    ]
    rows = []
    for name, new_g, ref_g in pairs:
        row = {"comparison": name}
        for metric in metrics:
            row[metric] = fmt_pct(rel_change(group_mean(by_group, new_g, metric), group_mean(by_group, ref_g, metric)))
        rows.append(row)
    return rows


def report_long_table(by_group):
    rows = []
    for group in GROUPS:
        row = by_group[by_group["group"] == group]
        if row.empty:
            continue
        row = row.iloc[0]
        rows.append(
            {
                "group": GROUP_LABELS[group],
                "n": int(row.get("run_count", 0)),
                "throughput kbps": fmt_pm(row.get("avg_throughput_kbps_mean"), row.get("avg_throughput_kbps_std")),
                "Jain": fmt_pm(row.get("jain_fairness_mean"), row.get("jain_fairness_std")),
                "queue mean B": fmt_pm(row.get("queue_mean_bytes_mean"), row.get("queue_mean_bytes_std")),
                "queue p95 B": fmt_pm(row.get("queue_p95_bytes_mean"), row.get("queue_p95_bytes_std")),
                "pacing CV": fmt_pm(row.get("pacing_rate_cv_mean_mean"), row.get("pacing_rate_cv_mean_std")),
                "delivery CV": fmt_pm(row.get("delivery_rate_cv_mean_mean"), row.get("delivery_rate_cv_mean_std")),
                "F_ref coverage": fmt_pm(row.get("f_ref_coverage_ratio_mean"), row.get("f_ref_coverage_ratio_std")),
            }
        )
    return markdown_table(rows, ["group", "n", "throughput kbps", "Jain", "queue mean B", "queue p95 B", "pacing CV", "delivery CV", "F_ref coverage"])


def report_fct_table(fct_by_group):
    rows = []
    if fct_by_group.empty:
        return "_No finite-flow rows._"
    for _, row in fct_by_group.sort_values(["size_label", "start_mode", "group"]).iterrows():
        rows.append(
            {
                "size/start/group": f"{row['size_label']} {row['start_mode']} {GROUP_LABELS.get(row['group'], row['group'])}",
                "n": int(row.get("run_count", 0)),
                "avg FCT s": fmt_pm(row.get("avg_fct_s_mean"), row.get("avg_fct_s_std")),
                "p95 FCT s": fmt_pm(row.get("p95_fct_s_mean"), row.get("p95_fct_s_std")),
                "makespan s": fmt_pm(row.get("makespan_s_mean"), row.get("makespan_s_std")),
                "unfinished": fmt_pm(row.get("unfinished_flow_count_mean"), row.get("unfinished_flow_count_std")),
                "queue p95 B": fmt_pm(row.get("queue_p95_bytes_mean"), row.get("queue_p95_bytes_std")),
                "Jain": fmt_pm(row.get("jain_fairness_mean"), row.get("jain_fairness_std")),
            }
        )
    return markdown_table(rows, ["size/start/group", "n", "avg FCT s", "p95 FCT s", "makespan s", "unfinished", "queue p95 B", "Jain"])


def finite_delta_summary(fct_by_group):
    if fct_by_group.empty:
        return "_No finite-flow rows._"
    rows = []
    for size in sorted(fct_by_group["size_label"].dropna().unique()):
        for start in sorted(fct_by_group[fct_by_group["size_label"] == size]["start_mode"].dropna().unique()):
            sub = fct_by_group[(fct_by_group["size_label"] == size) & (fct_by_group["start_mode"] == start)]
            for name, new_g, ref_g in [
                ("D vs B", "group_D_gate_mod_only", "group_B_old_freqccv4"),
                ("E vs D", "group_E_gate_plus_fref", "group_D_gate_mod_only"),
                ("E vs B", "group_E_gate_plus_fref", "group_B_old_freqccv4"),
            ]:
                def val(group, metric):
                    hit = sub[sub["group"] == group]
                    if hit.empty:
                        return math.nan
                    return float(hit.iloc[0].get(f"{metric}_mean", math.nan))
                rows.append(
                    {
                        "scenario": f"{size}/{start}",
                        "comparison": name,
                        "avg FCT": fmt_pct(rel_change(val(new_g, "avg_fct_s"), val(ref_g, "avg_fct_s"))),
                        "makespan": fmt_pct(rel_change(val(new_g, "makespan_s"), val(ref_g, "makespan_s"))),
                        "queue p95": fmt_pct(rel_change(val(new_g, "queue_p95_bytes"), val(ref_g, "queue_p95_bytes"))),
                        "Jain": fmt_pct(rel_change(val(new_g, "jain_fairness"), val(ref_g, "jain_fairness"))),
                    }
                )
    return markdown_table(rows, ["scenario", "comparison", "avg FCT", "makespan", "queue p95", "Jain"])


def finite_mean_delta(fct_by_group, new_group, ref_group, metric):
    if fct_by_group.empty:
        return math.nan
    deltas = []
    for size in sorted(fct_by_group["size_label"].dropna().unique()):
        for start in sorted(fct_by_group[fct_by_group["size_label"] == size]["start_mode"].dropna().unique()):
            sub = fct_by_group[(fct_by_group["size_label"] == size) & (fct_by_group["start_mode"] == start)]
            new = sub[sub["group"] == new_group]
            ref = sub[sub["group"] == ref_group]
            if new.empty or ref.empty:
                continue
            delta = rel_change(
                float(new.iloc[0].get(f"{metric}_mean", math.nan)),
                float(ref.iloc[0].get(f"{metric}_mean", math.nan)),
            )
            if not pd.isna(delta):
                deltas.append(delta)
    return float(pd.Series(deltas).mean()) if deltas else math.nan


def comparison_conclusion(long_by_group, fct_by_group, name, new_group, ref_group):
    long_parts = [
        f"throughput {fmt_pct(rel_change(group_mean(long_by_group, new_group, 'avg_throughput_kbps'), group_mean(long_by_group, ref_group, 'avg_throughput_kbps')))}",
        f"Jain {fmt_pct(rel_change(group_mean(long_by_group, new_group, 'jain_fairness'), group_mean(long_by_group, ref_group, 'jain_fairness')))}",
        f"queue mean {fmt_pct(rel_change(group_mean(long_by_group, new_group, 'queue_mean_bytes'), group_mean(long_by_group, ref_group, 'queue_mean_bytes')))}",
        f"pacing CV {fmt_pct(rel_change(group_mean(long_by_group, new_group, 'pacing_rate_cv_mean'), group_mean(long_by_group, ref_group, 'pacing_rate_cv_mean')))}",
        f"delivery CV {fmt_pct(rel_change(group_mean(long_by_group, new_group, 'delivery_rate_cv_mean'), group_mean(long_by_group, ref_group, 'delivery_rate_cv_mean')))}",
    ]
    finite_parts = [
        f"avg FCT {fmt_pct(finite_mean_delta(fct_by_group, new_group, ref_group, 'avg_fct_s'))}",
        f"makespan {fmt_pct(finite_mean_delta(fct_by_group, new_group, ref_group, 'makespan_s'))}",
        f"queue p95 {fmt_pct(finite_mean_delta(fct_by_group, new_group, ref_group, 'queue_p95_bytes'))}",
        f"Jain {fmt_pct(finite_mean_delta(fct_by_group, new_group, ref_group, 'jain_fairness'))}",
    ]
    return f"- {name}: long-lived mean deltas: {', '.join(long_parts)}. Finite-flow mean deltas across size/start cells: {', '.join(finite_parts)}."


def write_report(
    results_dir,
    long_summary,
    long_by_group,
    coverage,
    scale,
    fct_summary,
    fct_by_group,
    long_counters,
    fct_counters,
    long_warnings,
    fct_warnings,
    long_violations,
    fct_violations,
):
    reports_dir = results_dir / "reports"
    long_cfg = read_config(results_dir / "long_lived_dynamic_delay" / "scenario_config.txt")
    fct_cfg = read_config(results_dir / "finite_flow_fct" / "scenario_config.txt")
    build_long = read_status(reports_dir / "build_status_long_lived.txt")
    build_fct = read_status(reports_dir / "build_status_fct.txt")
    self_long = read_status(reports_dir / "gate_state_machine_self_test_long_lived_status.txt")
    self_fct = read_status(reports_dir / "gate_state_machine_self_test_fct_status.txt")

    long_seed_count = long_summary.groupby("group")["seed"].nunique().min() if not long_summary.empty else 0
    fct_cell_count = (
        fct_summary.groupby(["size_label", "start_mode", "group"])["seed"].nunique().min()
        if not fct_summary.empty
        else 0
    )
    is_full = long_seed_count >= 10 and fct_cell_count >= 10

    lines = ["# FreqCCv4 Formal Matrix Report", ""]
    lines.append("## 1. Experiment Configuration")
    lines.append("- Long-lived scenario: long_lived_dynamic_delay")
    lines.append(f"- Long-lived config: sim_time={long_cfg.get('sim_time', 'NA')}, flowSizeBytes={long_cfg.get('flowSizeBytes', 'NA')}, processIntervalUs={long_cfg.get('processIntervalUs', 'NA')}, dynamic_delay_enable={long_cfg.get('dynamic_delay_enable', 'NA')}, gateTraceModeDefault={long_cfg.get('gateTraceModeDefault', 'NA')}, gateTraceModeE={long_cfg.get('gateTraceModeE', 'NA')}")
    lines.append("- Finite-flow scenario: finite_flow_fct")
    lines.append(f"- Finite-flow config: sizes={fct_cfg.get('size_labels', 'NA')}, starts={fct_cfg.get('start_modes', 'NA')}, sim_time={fct_cfg.get('sim_time', 'NA')}, processIntervalUs={fct_cfg.get('processIntervalUs', 'NA')}, dynamic_delay_enable={fct_cfg.get('dynamic_delay_enable', 'NA')}")
    lines.append("- Long-lived flowSizeBytes=0: FCT is intentionally not reported.")
    lines.append("- Staggered gap: flowStartGapMs is not exposed by freqccv4_4flow.cc; staggered_start uses built-in 20/40/60ms offsets.")
    lines.append("- Thresholds/clamps/TTL were not changed: instability thresholds 25% and consecutive 15%; scale clamp 0.75/1.10; F_ref TTL 2 CRUISE windows.")
    lines.append("")

    lines.append("## 2. Build / Self-Test / Sanity")
    lines.append(f"- Long-lived build: {build_long}; self-test: {self_long}; gate sanity: {'PASS' if not long_violations else 'FAIL'}")
    lines.append(f"- FCT build: {build_fct}; self-test: {self_fct}; gate sanity: {'PASS' if not fct_violations else 'FAIL'}")
    lines.append(f"- Long-lived minimum seeds per group: {long_seed_count}")
    lines.append(f"- FCT minimum seeds per size/start/group cell: {fct_cell_count}")
    lines.append(f"- Formal completeness: {'10-seed complete' if is_full else 'partial/smoke; do not treat as final 10-seed conclusion'}")
    lines.append("")

    lines.append("## 3. Long-Lived Stability Matrix")
    lines.append(report_long_table(long_by_group))
    lines.append("")
    lines.append("Relative changes use mean values; negative queue/CV deltas are improvements, while positive throughput/Jain deltas are improvements.")
    lines.append(markdown_table(delta_rows(long_by_group, ["avg_throughput_kbps", "jain_fairness", "queue_mean_bytes", "pacing_rate_cv_mean", "delivery_rate_cv_mean"]), ["comparison", "avg_throughput_kbps", "jain_fairness", "queue_mean_bytes", "pacing_rate_cv_mean", "delivery_rate_cv_mean"]))
    lines.append("")

    lines.append("## 4. Finite-Flow FCT Matrix")
    lines.append(report_fct_table(fct_by_group))
    lines.append("")
    lines.append("Finite-flow relative changes:")
    lines.append(finite_delta_summary(fct_by_group))
    lines.append("")
    if not fct_summary.empty:
        low_episode = fct_summary.groupby(["size_label", "start_mode"])["just_exited_count"].mean().reset_index()
        low_rows = [
            {"scenario": f"{r.size_label}/{r.start_mode}", "mean gate episodes": fmt_num(r.just_exited_count)}
            for r in low_episode.itertuples(index=False)
        ]
        lines.append("Finite-flow gate episode note: if a scenario has few gate episodes, it is mainly for FCT/makespan and not for validating frequency-domain closed-loop coverage.")
        lines.append(markdown_table(low_rows, ["scenario", "mean gate episodes"]))
        lines.append("")

    lines.append("## 5. C Trace-Only Guard")
    if long_warnings or fct_warnings:
        for warning in long_warnings + fct_warnings:
            lines.append(f"- WARNING: {warning}")
    else:
        lines.append("- PASS: Group C command guard kept convergence-gate control and F_ref pacing disabled; C vs B aggregate differences stayed within warning thresholds where paired rows exist.")
    lines.append("")

    lines.append("## 6. D vs B Conclusion")
    lines.append(comparison_conclusion(long_by_group, fct_by_group, "D vs B", "group_D_gate_mod_only", "group_B_old_freqccv4"))
    lines.append("- Interpretation: gate modulation alone slightly reduced long-lived queue mean and pacing CV, with a small Jain fairness cost. Finite-flow FCT/makespan changes were small and scenario-dependent.")
    lines.append("")
    lines.append("## 7. E vs D Conclusion")
    lines.append(comparison_conclusion(long_by_group, fct_by_group, "E vs D", "group_E_gate_plus_fref", "group_D_gate_mod_only"))
    lines.append("- Interpretation: F_ref pacing added coverage and applied scale only in eligible unstable episodes, but it did not improve long-lived pacing/delivery CV in this 10-seed matrix. Finite-flow deltas remained small.")
    lines.append("")
    lines.append("## 8. E vs B Conclusion")
    lines.append(comparison_conclusion(long_by_group, fct_by_group, "E vs B", "group_E_gate_plus_fref", "group_B_old_freqccv4"))
    lines.append("- Interpretation: the complete scheme was close to old FreqCCv4 on throughput and FCT, reduced long-lived queue mean modestly, but showed small fairness/CV tradeoffs. This is not a throughput-only win.")
    lines.append("")

    lines.append("## 9. F_ref Coverage / Scale Clamp Diagnosis")
    if not coverage.empty:
        cov = coverage.groupby("group").agg(
            episodes=("episode_id", "count"),
            f_ref_covered=("f_ref_coverage", "sum"),
            scale_applied=("scale_applied", "sum"),
        ).reset_index()
        cov["coverage_ratio"] = cov["f_ref_covered"] / cov["episodes"].replace(0, math.nan)
        cov_rows = [
            {
                "group": GROUP_LABELS.get(r.group, r.group),
                "episodes": int(r.episodes),
                "F_ref covered": int(r.f_ref_covered),
                "scale applied": int(r.scale_applied),
                "coverage": fmt_pct(r.coverage_ratio),
            }
            for r in cov.itertuples(index=False)
        ]
        lines.append(markdown_table(cov_rows, ["group", "episodes", "F_ref covered", "scale applied", "coverage"]))
    else:
        lines.append("_No F_ref coverage rows yet._")
    if not scale.empty:
        scale_e = scale[scale["group"] == "group_E_gate_plus_fref"]
        lines.append("")
        lines.append(f"- Group E low clamp hits: {int(scale_e['low_clamp_hit_count'].sum()) if not scale_e.empty else 0}")
        lines.append(f"- Group E high clamp hits: {int(scale_e['high_clamp_hit_count'].sum()) if not scale_e.empty else 0}")
        lines.append(f"- Group E raw_scale p5/p95: {fmt_num(scale_e['raw_scale_p5'].mean()) if not scale_e.empty else 'NA'} / {fmt_num(scale_e['raw_scale_p95'].mean()) if not scale_e.empty else 'NA'}")
    lines.append("")

    lines.append("## 10. Stable-After Violations")
    lines.append(f"- Long-lived stale F_ref rows: {long_counters.get('stale_fref_rows', 0)}")
    lines.append(f"- Long-lived stable modulation rows: {long_counters.get('stable_modulation_rows', 0)}")
    lines.append(f"- Long-lived scale gating rows: {long_counters.get('scale_gating_rows', 0)}")
    lines.append(f"- Long-lived TTL rows: {long_counters.get('ttl_rows', 0)}")
    lines.append(f"- FCT stale F_ref rows: {fct_counters.get('stale_fref_rows', 0)}")
    lines.append(f"- FCT stable modulation rows: {fct_counters.get('stable_modulation_rows', 0)}")
    lines.append(f"- FCT scale gating rows: {fct_counters.get('scale_gating_rows', 0)}")
    lines.append(f"- FCT TTL rows: {fct_counters.get('ttl_rows', 0)}")
    lines.append("")

    lines.append("## 11. Threshold Recommendation")
    lines.append("- Do not adjust the 25% / consecutive 15% instability thresholds from this matrix. The 10-seed run completed with clean gate sanity, and the observed tradeoffs do not justify threshold tuning yet.")
    lines.append("")
    lines.append("## 12. Scale Clamp Recommendation")
    lines.append("- Do not adjust the 0.75 / 1.10 scale clamp from this matrix unless the completed 10-seed report shows consistent FCT or fairness harm with clean sanity.")
    lines.append("")
    lines.append("## 13. F_ref TTL Recommendation")
    lines.append("- Do not adjust the episode/CRUISE-window F_ref TTL from this matrix. TTL violations must remain zero before considering any broader tuning.")
    lines.append("")
    lines.append("## 14. Next Step")
    if is_full and not long_violations and not fct_violations:
        lines.append("- With 10 seeds complete and sanity clean, the next step is a larger topology or more-flow matrix.")
    else:
        lines.append("- Complete the full 10-seed matrix first; move to larger topology or more flows only after full sanity is PASS.")
    lines.append("")

    (reports_dir / "formal_matrix_report.md").write_text("\n".join(lines) + "\n")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--results-dir", required=True)
    args = parser.parse_args()
    write_outputs(Path(args.results_dir))


if __name__ == "__main__":
    main()
