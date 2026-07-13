#!/usr/bin/env python3
"""Align F-BBR OPIv2 windows with independent bottleneck ground truth."""

from __future__ import annotations

import argparse
import json
import math
import re
from pathlib import Path

import numpy as np
import pandas as pd
from scipy.stats import mannwhitneyu, rankdata, spearmanr, theilslopes


BOOL = {"true": True, "false": False, "1": True, "0": False}
CORE = {"E1", "E2", "E3", "E4"}
NEGATIVE = {"E7", "E8", "E9"}


def as_bool(value: object) -> bool:
    if isinstance(value, bool):
        return value
    return BOOL.get(str(value).strip().lower(), False)


def rho(x: pd.Series | np.ndarray, y: pd.Series | np.ndarray) -> float:
    a = np.asarray(x, float)
    b = np.asarray(y, float)
    mask = np.isfinite(a) & np.isfinite(b)
    if mask.sum() < 3 or np.ptp(a[mask]) == 0 or np.ptp(b[mask]) == 0:
        return math.nan
    return float(spearmanr(a[mask], b[mask]).statistic)


def auc_score(y: np.ndarray, score: np.ndarray) -> float:
    y = np.asarray(y, bool)
    n1, n0 = int(y.sum()), int((~y).sum())
    if not n1 or not n0:
        return math.nan
    ranks = rankdata(score)
    return float((ranks[y].sum() - n1 * (n1 + 1) / 2) / (n1 * n0))


def average_precision(y: np.ndarray, score: np.ndarray) -> float:
    y = np.asarray(y, bool)
    if not y.any():
        return math.nan
    order = np.argsort(-np.asarray(score, float), kind="stable")
    sorted_y = y[order]
    precision = np.cumsum(sorted_y) / np.arange(1, len(y) + 1)
    return float(precision[sorted_y].mean())


def cliff_delta(a: np.ndarray, b: np.ndarray) -> float:
    a, b = np.asarray(a, float), np.asarray(b, float)
    if not len(a) or not len(b):
        return math.nan
    # Rank formulation includes ties and avoids an O(n*m) comparison matrix.
    ranks = rankdata(np.r_[a, b])
    u = ranks[: len(a)].sum() - len(a) * (len(a) + 1) / 2
    return float((2 * u) / (len(a) * len(b)) - 1)


def read_goodput(path: Path) -> tuple[np.ndarray, np.ndarray]:
    frame = pd.read_csv(path, sep="\t", comment="#", names=["time", "kbps"])
    return frame.time.to_numpy(float), frame.kbps.to_numpy(float) * 1000.0


def schedule_change(meta: dict, start: float, end: float) -> bool:
    changes = list(meta.get("capacity_schedule", [])) + list(
        meta.get("background_schedule", [])
    )
    changes += [{"time_s": x} for x in meta.get("flow_start_times_s", [])]
    changes += [{"time_s": x} for x in meta.get("flow_stop_times_s", [])]
    return any(start < float(x.get("time_s", -1)) < end for x in changes)


def robust_trend(time: np.ndarray, q: np.ndarray, duration: float) -> float:
    if len(time) < 3:
        return math.nan
    if len(time) > 1200:
        idx = np.linspace(0, len(time) - 1, 1200).astype(int)
        time, q = time[idx], q[idx]
    try:
        return float(theilslopes(q, time).slope * duration)
    except Exception:
        return math.nan


def cycle_drain(
    queue_t: np.ndarray,
    queue_q: np.ndarray,
    bins: pd.DataFrame,
    start: float,
    end: float,
) -> float:
    if bins.empty:
        return math.nan
    selected = bins[
        (bins.time_end_s > start)
        & (bins.time_start_s < end)
        & (pd.to_numeric(bins.coded_excitation, errors="coerce") < 0)
    ]
    results: list[bool] = []
    for _, cycle in selected.groupby("cycle_id"):
        drained = False
        for row in cycle.itertuples():
            lo = max(start, float(row.time_start_s))
            hi = min(end, float(row.time_end_s))
            a, b = np.searchsorted(queue_t, [lo, hi])
            if b > a and np.nanmin(queue_q[a:b]) <= 0.05:
                drained = True
                break
        results.append(drained)
    return float(np.mean(results)) if results else math.nan


def align_window(
    row: pd.Series,
    queue: pd.DataFrame,
    bins: pd.DataFrame,
    goodput: tuple[np.ndarray, np.ndarray],
    meta: dict,
) -> dict[str, object]:
    start, end = float(row.start_time_s), float(row.end_time_s)
    duration = end - start
    qt = queue.time_s.to_numpy(float)
    a, b = np.searchsorted(qt, [start, end])
    q = queue.iloc[a:b]
    if q.empty or duration <= 0:
        raise ValueError(f"queue trace does not cover window {start}-{end}")
    qbdp = q.queue_bdp.to_numpy(float)
    capacity = float(np.average(q.capacity_bps))
    dequeue = float(q.dequeue_bytes_delta.sum())
    enqueue = float(q.enqueue_bytes_delta.sum())
    drop = float(q.drop_bytes_delta.sum())
    ecn = float(q.ecn_marked_bytes_delta.sum())
    util = 8.0 * dequeue / (capacity * duration) if capacity > 0 else math.nan
    quantiles = np.quantile(qbdp, [0.10, 0.50, 0.95])
    drain_fraction = float(np.mean(qbdp <= 0.05))
    cycle_ratio = cycle_drain(qt, queue.queue_bdp.to_numpy(float), bins, start, end)
    if not np.isfinite(cycle_ratio):
        cycle_ratio = drain_fraction
    trend = robust_trend(q.time_s.to_numpy(float), qbdp, duration)
    loss = drop / max(enqueue, 1.0)
    ecn_ratio = ecn / max(dequeue, 1.0)
    dynamic = schedule_change(meta, start, end)
    fair_values = q.theoretical_fair_share_bps.to_numpy(float)
    positive_fair = fair_values[fair_values > 0]
    fair = float(np.median(positive_fair)) if len(positive_fair) else 0.0
    if len(positive_fair) and np.ptp(positive_fair) > 0.05 * np.mean(positive_fair):
        dynamic = True

    ideal = (
        util >= 0.95
        and quantiles[0] <= 0.05
        and quantiles[1] <= 0.10
        and quantiles[2] <= 0.30
        and cycle_ratio >= 0.70
        and abs(trend) <= 0.10
        and loss <= 0.001
        and ecn_ratio <= 0.02
    )
    underload = util <= 0.90 and quantiles[2] <= 0.10
    overload = (
        quantiles[1] >= 0.25
        or quantiles[2] >= 0.75
        or trend >= 0.25
        or loss > 0.005
        or ecn_ratio > 0.05
    )
    dynamic = dynamic or (abs(trend) > 0.10 and not overload)
    if dynamic:
        label = "DYNAMIC"
    elif ideal:
        label = "IDEAL"
    elif underload:
        label = "UNDERLOAD"
    elif overload:
        label = "OVERLOAD"
    else:
        label = "TRANSITION"

    s_util = np.clip((util - 0.85) / 0.10, 0, 1)
    s_floor = math.exp(-quantiles[0] / 0.05)
    s_tail = math.exp(-quantiles[2] / 0.30)
    s_drain = np.clip((cycle_ratio - 0.30) / 0.40, 0, 1)
    s_stationary = math.exp(-abs(trend) / 0.10)
    s_safe = math.exp(-100 * loss) * math.exp(-10 * ecn_ratio)
    queue_score = float(
        max(0.0, s_util * s_floor * s_tail * s_drain * s_stationary * s_safe)
        ** (1 / 6)
    )
    gt, gv = goodput
    ga, gb = np.searchsorted(gt, [start, end])
    goodput_bps = float(np.mean(gv[ga:gb])) if gb > ga else math.nan
    share_error = abs(goodput_bps - fair) / fair if fair > 0 else math.nan
    share_score = math.exp(-share_error / 0.10) if np.isfinite(share_error) else math.nan
    flow_score = math.sqrt(queue_score * share_score) if np.isfinite(share_score) else math.nan
    return {
        "gt_utilization": util,
        "gt_q_p10_bdp": quantiles[0],
        "gt_q_p50_bdp": quantiles[1],
        "gt_q_p95_bdp": quantiles[2],
        "gt_q_mean_bdp": float(np.mean(qbdp)),
        "gt_q_max_bdp": float(np.max(qbdp)),
        "gt_drain_fraction": drain_fraction,
        "gt_cycle_drain_ratio": cycle_ratio,
        "gt_queue_change_per_window_bdp": trend,
        "gt_loss_ratio": loss,
        "gt_ecn_ratio": ecn_ratio,
        "gt_label": label,
        "gt_queue_optimality_score": queue_score,
        "gt_flow_goodput_bps": goodput_bps,
        "theoretical_fair_share_bps": fair,
        "gt_active_fbbr_flows": int(round(float(q.active_fbbr_flows.median()))),
        "gt_share_error_ratio": share_error,
        "gt_share_score": share_score,
        "gt_flow_optimality_score": flow_score,
    }


def normalize_window(frame: pd.DataFrame, shadow: bool) -> pd.DataFrame:
    rename = {
        "block_id": "window_id", "C_meas": "measurement_confidence",
        "window_start_s": "start_time_s", "window_end_s": "end_time_s",
        "S_opt": "optimality_score", "S_full": "full_score",
        "S_lowq": "low_queue_score", "S_stationary": "stationary_score",
        "S_safe": "safe_score", "candidate_bps": "candidate_bw_bps",
    }
    frame = frame.rename(columns=rename)
    frame["shadow_window"] = shadow
    frame["decision_eligible"] = not shadow
    for name in ["candidate_valid", "shadow_window", "decision_eligible"]:
        if name in frame:
            frame[name] = frame[name].map(as_bool)
    return frame


def scenario_key(path: Path) -> str:
    return path.parent.name.split("_", 1)[0]


def process_run(run_dir: Path) -> tuple[list[dict], list[dict]]:
    meta = json.loads((run_dir / "run_meta.json").read_text())
    queue = pd.read_csv(run_dir / "bottleneck_queue.csv")
    scenario = scenario_key(run_dir)
    seed = int(meta["seed"])
    windows: list[dict] = []
    cruises: list[dict] = []
    for block_path in sorted(run_dir.glob("flow*_fbbr_opiv2_blocks.csv")):
        flow = int(re.search(r"flow(\d+)", block_path.name).group(1))
        shadow_path = run_dir / f"flow{flow}_fbbr_opiv2_shadow_windows.csv"
        bin_path = run_dir / f"flow{flow}_fbbr_opiv2_bins.csv"
        cruise_path = run_dir / f"flow{flow}_fbbr_opiv2_cruises.csv"
        goodput_path = run_dir / f"fbbr_validation_flow{flow}_F-BBR_good.txt"
        blocks = normalize_window(pd.read_csv(block_path), False)
        shadows = normalize_window(pd.read_csv(shadow_path), True)
        bins = pd.read_csv(bin_path) if bin_path.exists() else pd.DataFrame()
        cruise = pd.read_csv(cruise_path)
        goodput = read_goodput(goodput_path)
        cruise_by_id = cruise.set_index("cruise_id", drop=False)
        combined_windows = pd.concat([blocks, shadows], ignore_index=True)
        window_cruise_ids = set(combined_windows.cruise_id) if len(combined_windows) else set()
        for _, row in combined_windows.iterrows():
            aligned = align_window(row, queue, bins, goodput, meta)
            c = cruise_by_id.loc[row.cruise_id] if row.cruise_id in cruise_by_id.index else None
            record = row.to_dict()
            record.update(aligned)
            record.update(
                scenario=scenario, seed=seed, run_id=int(meta["run_id"]), flow_id=flow,
                window_start_s=float(row.start_time_s), window_end_s=float(row.end_time_s),
                published_valid=as_bool(c.publication_valid) if c is not None else False,
                published_bw_bps=float(c.published_bps) if c is not None else 0.0,
                published_reason=str(c.publication_invalid_reason) if c is not None else "",
            )
            windows.append(record)
        for _, c in cruise.iterrows():
            cruise_gt = align_window(
                pd.Series({"start_time_s": c.start_time_s, "end_time_s": c.end_time_s}),
                queue, bins, goodput, meta,
            ) if c.cruise_id not in window_cruise_ids and float(c.end_time_s) > float(c.start_time_s) else {}
            cruises.append(
                {
                    **c.to_dict(), "scenario": scenario, "seed": seed,
                    "run_id": int(meta["run_id"]), "flow_id": flow,
                    "published_valid": as_bool(c.publication_valid),
                    "published_bw_bps": float(c.published_bps),
                    "fair_share_bps": float(c.fair_share_bps),
                    **{f"cruise_{k}": v for k, v in cruise_gt.items()},
                }
            )
    return windows, cruises


def cluster_bootstrap(frame: pd.DataFrame, iterations: int, rng: np.random.Generator) -> tuple[float, float]:
    groups = [g for _, g in frame.groupby(["scenario", "seed"])]
    values = []
    for _ in range(iterations):
        sampled = [groups[i] for i in rng.integers(0, len(groups), len(groups))]
        data = pd.concat(sampled, ignore_index=True)
        values.append(rho(data.optimality_score, data.gt_queue_optimality_score))
    finite = np.asarray(values)[np.isfinite(values)]
    return tuple(np.quantile(finite, [0.025, 0.975])) if len(finite) else (math.nan, math.nan)


def block_permutation(frame: pd.DataFrame, iterations: int, rng: np.random.Generator) -> float:
    observed = rho(frame.optimality_score, frame.gt_queue_optimality_score)
    if not np.isfinite(observed):
        return math.nan
    groups = [g.copy() for _, g in frame.groupby(["scenario", "seed", "flow_id"])]
    exceed = 0
    for _ in range(iterations):
        shuffled = []
        for group in groups:
            cruise_ids = group.cruise_id.unique().copy()
            permuted = rng.permutation(cruise_ids)
            score_by_cruise = {
                target: group.loc[group.cruise_id == source, "optimality_score"].to_numpy()
                for target, source in zip(cruise_ids, permuted)
            }
            copy = group.copy()
            for target, scores in score_by_cruise.items():
                idx = copy.index[copy.cruise_id == target]
                copy.loc[idx, "optimality_score"] = np.resize(scores, len(idx))
            shuffled.append(copy)
        perm = pd.concat(shuffled, ignore_index=True)
        exceed += abs(rho(perm.optimality_score, perm.gt_queue_optimality_score)) >= abs(observed)
    return (exceed + 1) / (iterations + 1)


def add_cruise_metrics(windows: pd.DataFrame, cruises: pd.DataFrame) -> pd.DataFrame:
    rows = []
    seen = set()
    window_groups = windows.groupby(["scenario", "seed", "flow_id", "cruise_id"]) if len(windows) else []
    for key, group in window_groups:
        shadows = group[group.shadow_window]
        identifiable = (
            len(shadows) >= 4
            and shadows.gt_queue_optimality_score.quantile(0.75)
            - shadows.gt_queue_optimality_score.quantile(0.25) >= 0.10
            and shadows.optimality_score.quantile(0.75)
            - shadows.optimality_score.quantile(0.25) >= 0.05
        )
        if len(shadows) < 4:
            within_status = "insufficient_within_cruise_windows"
        elif identifiable:
            within_status = "identifiable"
        else:
            within_status = "not_identifiable_within_cruise"
        c = cruises[
            (cruises.scenario == key[0]) & (cruises.seed == key[1])
            & (cruises.flow_id == key[2]) & (cruises.cruise_id == key[3])
        ]
        c = c.iloc[-1] if len(c) else None
        pub = bool(c.published_valid) if c is not None else False
        bw = float(c.published_bw_bps) if c is not None else 0.0
        fair = float(group.theoretical_fair_share_bps.median())
        rows.append(
            {
                "scenario": key[0], "seed": key[1], "flow_id": key[2], "cruise_id": key[3],
                "n_windows": len(group), "n_shadow_windows": len(shadows),
                "within_cruise_status": within_status,
                "within_cruise_spearman_queue": rho(shadows.optimality_score, shadows.gt_queue_optimality_score) if identifiable else math.nan,
                "within_cruise_spearman_flow": rho(shadows.optimality_score, shadows.gt_flow_optimality_score) if identifiable else math.nan,
                "score_mean": group.optimality_score.mean(), "score_max": group.optimality_score.max(),
                "gt_queue_score_mean": group.gt_queue_optimality_score.mean(),
                "gt_queue_score_max": group.gt_queue_optimality_score.max(),
                "gt_label": group.gt_label.mode().iloc[0],
                "published_valid": pub, "published_bw_bps": bw, "fair_share_bps": fair,
                "cruise_start_s": float(c.start_time_s) if c is not None else float(group.window_start_s.min()),
                "cruise_end_s": float(c.end_time_s) if c is not None else float(group.window_end_s.max()),
                "trusted_bw_bps": float(c.trusted_bw_bps) if c is not None else 0.0,
                "trusted_bw_error_ratio": abs(bw - fair) / fair if pub and fair > 0 else math.nan,
                "trusted_bw_signed_error": (bw - fair) / fair if pub and fair > 0 else math.nan,
                "trusted_bw_history_action": str(c.history_update_action) if c is not None else "",
                "trusted_bw_reason": str(c.publication_invalid_reason) if c is not None else "",
            }
        )
        seen.add(key)
    for c in cruises.itertuples():
        key = (c.scenario, c.seed, c.flow_id, c.cruise_id)
        if key in seen:
            continue
        pub = bool(c.published_valid)
        bw = float(c.published_bw_bps)
        fair = float(getattr(c, "cruise_theoretical_fair_share_bps", c.fair_share_bps))
        rows.append(
            {
                "scenario": c.scenario, "seed": c.seed, "flow_id": c.flow_id,
                "cruise_id": c.cruise_id, "n_windows": 0, "n_shadow_windows": 0,
                "within_cruise_status": "insufficient_within_cruise_windows",
                "within_cruise_spearman_queue": math.nan,
                "within_cruise_spearman_flow": math.nan,
                "score_mean": math.nan, "score_max": math.nan,
                "gt_queue_score_mean": getattr(c, "cruise_gt_queue_optimality_score", math.nan),
                "gt_queue_score_max": getattr(c, "cruise_gt_queue_optimality_score", math.nan),
                "gt_label": getattr(c, "cruise_gt_label", ""),
                "published_valid": pub, "published_bw_bps": bw, "fair_share_bps": fair,
                "cruise_start_s": float(c.start_time_s), "cruise_end_s": float(c.end_time_s),
                "trusted_bw_bps": float(c.trusted_bw_bps),
                "trusted_bw_error_ratio": abs(bw - fair) / fair if pub and fair > 0 else math.nan,
                "trusted_bw_signed_error": (bw - fair) / fair if pub and fair > 0 else math.nan,
                "trusted_bw_history_action": str(c.history_update_action),
                "trusted_bw_reason": str(c.publication_invalid_reason),
            }
        )
    return pd.DataFrame(rows)


def tracking_metrics(cruise: pd.DataFrame) -> pd.DataFrame:
    schedules = {
        "E5": [(0, 100e6), (40, 50e6), (80, 25e6), (140, 50e6), (180, 100e6)],
        "E6": [(0, 50e6), (40, 40e6), (80, 30e6), (120, 20e6), (160, 50e6)],
    }
    rows = []
    for scenario, schedule in schedules.items():
        data = cruise[cruise.scenario.eq(scenario)]
        for (seed, flow), group in data.groupby(["seed", "flow_id"]):
            group = group.sort_values("cruise_end_s")
            flow_start = 0 if flow == 1 else (40 if flow == 2 else 80)
            flow_stop = math.inf if flow == 1 else (180 if flow == 2 else 140)
            for index, (change, target) in enumerate(schedule):
                next_change = schedule[index + 1][0] if index + 1 < len(schedule) else math.inf
                if not (flow_start <= change < flow_stop):
                    continue
                epoch = group[(group.cruise_end_s >= change) & (group.cruise_end_s < min(next_change, flow_stop))]
                eligible = epoch[
                    epoch.published_valid
                    & ((epoch.published_bw_bps - target).abs() / target <= 0.10)
                ]
                if len(eligible):
                    first = eligible.iloc[0]
                    before = epoch[epoch.cruise_end_s <= first.cruise_end_s]
                    elapsed = max(0.0, float(first.cruise_end_s) - change)
                    cruise_count = len(before)
                else:
                    elapsed, cruise_count = math.nan, math.nan
                old_target = schedule[index - 1][1] if index else target
                stale = epoch[
                    (epoch.trusted_bw_bps - old_target).abs() / old_target <= 0.10
                ] if index else epoch.iloc[0:0]
                stale_duration = (
                    max(0.0, float(stale.cruise_end_s.max()) - change) if len(stale) else 0.0
                )
                rows.append(
                    {
                        "scenario": scenario, "seed": seed, "flow_id": flow,
                        "change_time_s": change, "target_fair_share_bps": target,
                        "tracked": bool(len(eligible)), "tracking_time_s": elapsed,
                        "tracking_cruise_count": cruise_count,
                        "old_trusted_bw_stale_duration_s": stale_duration,
                        "within_30s": bool(len(eligible) and elapsed <= 30),
                        "within_3_cruises": bool(len(eligible) and cruise_count <= 3),
                    }
                )
    return pd.DataFrame(rows)


def run_metrics(windows: pd.DataFrame, cruise: pd.DataFrame) -> pd.DataFrame:
    rows = []
    for (scenario, seed), group in windows.groupby(["scenario", "seed"]):
        y = group.gt_label.eq("IDEAL").to_numpy()
        score = group.optimality_score.to_numpy(float)
        high = score >= 0.70
        stable_pub = cruise[(cruise.scenario == scenario) & (cruise.seed == seed) & cruise.published_valid]
        errors = stable_pub.trusted_bw_error_ratio.dropna().to_numpy(float)
        false = cruise[(cruise.scenario == scenario) & (cruise.seed == seed)]
        false_count = 0
        for r in false.itertuples():
            false_count += bool(r.published_valid and r.gt_label in {"UNDERLOAD", "OVERLOAD", "DYNAMIC"})
        published = false[false.published_valid]
        cvs = []
        for _, g in published.groupby("cruise_id"):
            if len(g) > 1 and g.published_bw_bps.mean() > 0:
                cvs.append(g.published_bw_bps.std(ddof=0) / g.published_bw_bps.mean())
        rows.append(
            {
                "scenario": scenario, "seed": seed, "n_flows": group.flow_id.nunique(),
                "pooled_spearman_queue": rho(score, group.gt_queue_optimality_score),
                "pooled_spearman_flow": rho(score, group.gt_flow_optimality_score),
                "ideal_auc": auc_score(y, score), "ideal_average_precision": average_precision(y, score),
                "high_score_precision": float(y[high].mean()) if high.any() else math.nan,
                "high_score_recall": float((y & high).sum() / max(y.sum(), 1)),
                "false_valid_rate": false_count / max(len(false), 1),
                "trusted_bw_mape": float(np.median(errors)) if len(errors) else math.nan,
                "trusted_bw_p90_ape": float(np.quantile(errors, 0.9)) if len(errors) else math.nan,
                "trusted_bw_signed_bias": float(stable_pub.trusted_bw_signed_error.median()) if len(stable_pub) else math.nan,
                "cross_flow_cv": float(np.nanmedian(cvs)) if cvs else math.nan,
                "publication_coverage": float(false.published_valid.mean()) if len(false) else 0.0,
            }
        )
    return pd.DataFrame(rows)


def ideal_epoch_coverage(windows: pd.DataFrame, cruise: pd.DataFrame) -> float:
    covered, total = 0, 0
    for key, group in cruise.sort_values("cruise_id").groupby(["scenario", "seed", "flow_id"]):
        group = group.copy()
        group["ideal"] = group.gt_label.eq("IDEAL")
        prior = False
        for pos, ideal in enumerate(group.ideal.to_numpy(bool)):
            if ideal and not prior:
                epoch = group.iloc[pos : pos + 3]
                epoch = epoch[epoch.ideal]
                total += 1
                covered += bool(epoch.published_valid.any())
            prior = ideal
    return covered / total if total else math.nan


def failures(windows: pd.DataFrame, cruise: pd.DataFrame, limit: int = 50) -> pd.DataFrame:
    cases = []
    high = windows[(windows.optimality_score >= 0.70) & windows.gt_label.ne("IDEAL")].copy()
    high["failure_type"] = "high_score_nonideal"
    high["severity"] = high.optimality_score
    low = windows[(windows.optimality_score < 0.50) & windows.gt_label.eq("IDEAL")].copy()
    low["failure_type"] = "low_score_ideal"
    low["severity"] = 1 - low.optimality_score
    bad_pub = cruise[cruise.published_valid & (cruise.trusted_bw_error_ratio > 0.10)].copy()
    bad_pub["failure_type"] = "published_bw_large_error"
    bad_pub["severity"] = bad_pub.trusted_bw_error_ratio
    missed_rows = []
    for key, group in cruise.sort_values("cruise_id").groupby(["scenario", "seed", "flow_id"]):
        consecutive = 0
        for row in group.itertuples():
            ideal = row.gt_label == "IDEAL"
            consecutive = consecutive + 1 if ideal and not row.published_valid else 0
            if consecutive >= 3:
                missed_rows.append(
                    {"scenario": key[0], "seed": key[1], "flow_id": key[2],
                     "cruise_id": row.cruise_id, "failure_type": "missed_publication",
                     "severity": consecutive, "trusted_bw_reason": row.trusted_bw_reason,
                     "fair_share_bps": row.fair_share_bps}
                )
    missed = pd.DataFrame(missed_rows)
    for frame in [high, low, bad_pub, missed]:
        if len(frame):
            cases.append(frame.nlargest(limit, "severity"))
    return pd.concat(cases, ignore_index=True, sort=False) if cases else pd.DataFrame(
        columns=["failure_type", "severity", "scenario", "seed", "flow_id",
                 "cruise_id", "window_id", "window_start_s", "window_end_s"]
    )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("validation_root", type=Path)
    parser.add_argument("--output-dir", type=Path)
    parser.add_argument("--bootstrap-iterations", type=int, default=2000)
    parser.add_argument("--permutation-iterations", type=int, default=2000)
    parser.add_argument("--seed", type=int, default=20260711)
    args = parser.parse_args()
    root = args.validation_root.resolve()
    output = (args.output_dir or root / "analysis").resolve()
    output.mkdir(parents=True, exist_ok=True)
    manifest = pd.read_csv(root / "matrix_manifest.csv")
    passed = manifest[manifest.status.isin(["PASS", "SKIPPED_COMPLETE"])]
    all_windows, all_cruises = [], []
    for run in passed.run_dir:
        windows, cruises = process_run(Path(run))
        all_windows.extend(windows)
        all_cruises.extend(cruises)
    if not all_windows:
        raise SystemExit("no complete runs to analyze")
    windows = pd.DataFrame(all_windows)
    cruises_raw = pd.DataFrame(all_cruises)
    for name in ["optimality_score", "measurement_confidence", "candidate_bw_bps"]:
        windows[name] = pd.to_numeric(windows[name], errors="coerce")
    cruise = add_cruise_metrics(windows, cruises_raw)
    tracking = tracking_metrics(cruise)
    runs = run_metrics(windows, cruise)
    expected_runs = passed[["scenario", "seed"]].drop_duplicates().copy()
    expected_runs["n_flows_expected"] = expected_runs.scenario.map(
        {"E1": 1, "E2": 2, "E3": 4, "E4": 8, "E5": 4,
         "E6": 2, "E7": 4, "E8": 4, "E9": 4}
    )
    runs = expected_runs.merge(runs, on=["scenario", "seed"], how="left")
    runs["n_flows"] = runs.n_flows.fillna(runs.n_flows_expected).astype(int)
    runs = runs.drop(columns="n_flows_expected")
    windows.to_csv(output / "window_validation.csv", index=False)
    cruise.to_csv(output / "cruise_validation.csv", index=False)
    runs.to_csv(output / "run_validation.csv", index=False)
    tracking.to_csv(output / "dynamic_tracking.csv", index=False)

    rng = np.random.default_rng(args.seed)
    core = windows[windows.scenario.isin(CORE)]
    observed = rho(core.optimality_score, core.gt_queue_optimality_score)
    ci_low, ci_high = cluster_bootstrap(core, args.bootstrap_iterations, rng)
    perm_p = block_permutation(core, args.permutation_iterations, rng)
    y = core.gt_label.eq("IDEAL").to_numpy()
    score = core.optimality_score.to_numpy(float)
    high = score >= 0.70
    identifiable = cruise[cruise.within_cruise_status.eq("identifiable")]
    published = cruise[cruise.published_valid & cruise.scenario.isin(CORE)]
    errors = published.trusted_bw_error_ratio.dropna().to_numpy(float)
    signed = published.trusted_bw_signed_error.dropna().to_numpy(float)
    negative = windows[windows.scenario.isin(NEGATIVE)]
    neg_cruise = cruise[cruise.scenario.isin(NEGATIVE)]
    correlation_rows = [
        {"level": "core_pooled", "scenario": "E1-E4", "seed": math.nan,
         "flow_id": math.nan, "n": len(core), "rho_queue": observed,
         "rho_flow": rho(core.optimality_score, core.gt_flow_optimality_score)}
    ]
    for scenario, group in windows.groupby("scenario"):
        correlation_rows.append(
            {"level": "scenario", "scenario": scenario, "seed": math.nan,
             "flow_id": math.nan, "n": len(group),
             "rho_queue": rho(group.optimality_score, group.gt_queue_optimality_score),
             "rho_flow": rho(group.optimality_score, group.gt_flow_optimality_score)}
        )
    flow_rhos = []
    for key, group in core.groupby(["scenario", "seed", "flow_id"]):
        value = rho(group.optimality_score, group.gt_queue_optimality_score)
        flow_rhos.append(value)
        correlation_rows.append(
            {"level": "run_flow", "scenario": key[0], "seed": key[1],
             "flow_id": key[2], "n": len(group), "rho_queue": value,
             "rho_flow": rho(group.optimality_score, group.gt_flow_optimality_score)}
        )
    pd.DataFrame(correlation_rows).to_csv(output / "correlation_summary.csv", index=False)
    flow_rhos_finite = np.asarray(flow_rhos)[np.isfinite(flow_rhos)]
    core_cvs = []
    for _, group in published.groupby(["scenario", "seed", "cruise_id"]):
        if len(group) > 1 and group.published_bw_bps.mean() > 0:
            core_cvs.append(group.published_bw_bps.std(ddof=0) / group.published_bw_bps.mean())
    coverage = ideal_epoch_coverage(core, cruise[cruise.scenario.isin(CORE)])
    metrics = {
        "matrix_runs_expected": 90,
        "matrix_runs_completed": int(len(passed)),
        "matrix_success_rate": float(len(passed) / max(len(manifest), 1)),
        "core_pooled_spearman_rho": observed,
        "core_bootstrap_ci_low": ci_low,
        "core_bootstrap_ci_high": ci_high,
        "core_block_permutation_p": perm_p,
        "core_ideal_auc": auc_score(y, score),
        "core_ideal_average_precision": average_precision(y, score),
        "core_high_score_precision": float(y[high].mean()) if high.any() else math.nan,
        "core_high_score_recall": float((y & high).sum() / max(y.sum(), 1)),
        "within_cruise_positive_fraction": float((identifiable.within_cruise_spearman_queue > 0).mean()) if len(identifiable) else math.nan,
        "within_cruise_median_rho": float(identifiable.within_cruise_spearman_queue.median()) if len(identifiable) else math.nan,
        "per_flow_median_rho": float(np.median(flow_rhos_finite)) if len(flow_rhos_finite) else math.nan,
        "per_flow_positive_fraction": float(np.mean(flow_rhos_finite > 0)) if len(flow_rhos_finite) else math.nan,
        "trusted_bw_median_mape": float(np.median(errors)) if len(errors) else math.nan,
        "trusted_bw_p90_ape": float(np.quantile(errors, 0.9)) if len(errors) else math.nan,
        "trusted_bw_median_signed_bias": float(np.median(signed)) if len(signed) else math.nan,
        "trusted_bw_valid_precision": float((errors <= 0.10).mean()) if len(errors) else math.nan,
        "trusted_bw_cross_flow_cv_p90": float(np.quantile(core_cvs, .9)) if core_cvs else math.nan,
        "publication_coverage": coverage,
        "negative_high_score_false_positive_rate": float((negative.optimality_score >= 0.70).mean()) if len(negative) else math.nan,
        "negative_false_publication_rate": float(neg_cruise.published_valid.mean()) if len(neg_cruise) else math.nan,
        "gradient_disagreement_rate": float((~windows.gradient_agreement.map(as_bool)).mean()),
        "measurement_confidence_gt_queue_rho": rho(
            windows.measurement_confidence, windows.gt_queue_optimality_score
        ),
        "measurement_confidence_optimality_rho": rho(
            windows.measurement_confidence, windows.optimality_score
        ),
        "dynamic_tracking_success_rate": float(
            (tracking.within_30s & tracking.within_3_cruises).mean()
        ) if len(tracking) else math.nan,
        "dynamic_tracking_p90_time_s": float(
            tracking.loc[tracking.tracked, "tracking_time_s"].quantile(.9)
        ) if len(tracking) and tracking.tracked.any() else math.nan,
    }
    dist_rows = []
    ideal_score = core.loc[core.gt_label.eq("IDEAL"), "optimality_score"].to_numpy(float)
    for label in ["UNDERLOAD", "OVERLOAD", "DYNAMIC"]:
        other = core.loc[core.gt_label.eq(label), "optimality_score"].to_numpy(float)
        dist_rows.append(
            {
                "comparison": f"IDEAL_vs_{label}",
                "median_difference": float(np.median(ideal_score) - np.median(other)) if len(ideal_score) and len(other) else math.nan,
                "mann_whitney_p": float(mannwhitneyu(ideal_score, other).pvalue) if len(ideal_score) and len(other) else math.nan,
                "cliffs_delta": cliff_delta(ideal_score, other),
            }
        )
    pd.DataFrame(dist_rows).to_csv(output / "distribution_tests.csv", index=False)
    dist_by_name = {x["comparison"]: x for x in dist_rows}
    nonideal_score = core.loc[core.gt_label.ne("IDEAL"), "optimality_score"].to_numpy(float)
    metrics.update(
        ideal_nonideal_median_difference=(
            float(np.median(ideal_score) - np.median(nonideal_score))
            if len(ideal_score) and len(nonideal_score) else math.nan
        ),
        cliffs_delta_ideal_underload=dist_by_name["IDEAL_vs_UNDERLOAD"]["cliffs_delta"],
        cliffs_delta_ideal_overload=dist_by_name["IDEAL_vs_OVERLOAD"]["cliffs_delta"],
        confusion_tp=int((y & high).sum()), confusion_fp=int((~y & high).sum()),
        confusion_tn=int((~y & ~high).sum()), confusion_fn=int((y & ~high).sum()),
    )
    failure = failures(windows, cruise)
    failure.to_csv(output / "failure_cases.csv", index=False)

    grad = windows.assign(
        gradient_disagreement=~windows.gradient_agreement.map(as_bool),
        snr_bin=pd.cut(pd.to_numeric(windows.snr_delivery, errors="coerce"), [-np.inf, 3, 6, 10, np.inf]),
        delay_bin=pd.cut(pd.to_numeric(windows.delay_ratio, errors="coerce"), [-np.inf, .85, 1.15, np.inf]),
    )
    grad.groupby(["gt_label", "gt_active_fbbr_flows", "snr_bin", "delay_bin"], observed=True).gradient_disagreement.agg(
        ["count", "mean"]
    ).reset_index().to_csv(output / "gradient_disagreement.csv", index=False)

    checks = {
        "score_rho": observed >= 0.55,
        "score_ci_low": ci_low > 0.35,
        "score_permutation": perm_p < 0.01,
        "per_flow_median_rho": metrics["per_flow_median_rho"] >= 0.45,
        "per_flow_positive_fraction": metrics["per_flow_positive_fraction"] >= 0.80,
        "within_cruise_positive_fraction": metrics["within_cruise_positive_fraction"] >= 0.70,
        "within_cruise_median_rho": metrics["within_cruise_median_rho"] >= 0.35,
        "ideal_auc": metrics["core_ideal_auc"] >= 0.85,
        "ideal_ap": metrics["core_ideal_average_precision"] >= 0.70,
        "high_score_precision": metrics["core_high_score_precision"] >= 0.80,
        "ideal_nonideal_median_difference": metrics["ideal_nonideal_median_difference"] >= 0.15,
        "cliffs_delta_ideal_underload": metrics["cliffs_delta_ideal_underload"] >= 0.47,
        "cliffs_delta_ideal_overload": metrics["cliffs_delta_ideal_overload"] >= 0.47,
        "negative_high_score_fpr": metrics["negative_high_score_false_positive_rate"] <= 0.05,
        "negative_false_publication": metrics["negative_false_publication_rate"] <= 0.05,
        "trusted_mape": metrics["trusted_bw_median_mape"] <= 0.05,
        "trusted_p90": metrics["trusted_bw_p90_ape"] <= 0.10,
        "trusted_bias": abs(metrics["trusted_bw_median_signed_bias"]) <= 0.03,
        "trusted_valid_precision": metrics["trusted_bw_valid_precision"] >= 0.90,
        "trusted_cross_flow_cv": metrics["trusted_bw_cross_flow_cv_p90"] <= 0.08,
        "publication_coverage": metrics["publication_coverage"] >= 0.60,
        "dynamic_tracking": metrics["dynamic_tracking_success_rate"] >= 1.0,
    }
    metrics["acceptance_checks"] = checks
    metrics["all_static_acceptance_pass"] = all(checks.values())
    if all(checks.values()):
        conclusion = "PASS"
    elif all(checks[x] for x in ["score_rho", "score_ci_low", "score_permutation",
                                  "trusted_mape", "trusted_p90", "trusted_bias"]) and not checks["publication_coverage"]:
        conclusion = "PASS_BUT_CONSERVATIVE"
    elif not all(checks[x] for x in ["score_rho", "score_ci_low", "score_permutation"]):
        conclusion = "FAIL_SCORE_NOT_CORRELATED"
    elif not checks["negative_false_publication"]:
        conclusion = "FAIL_FALSE_VALID"
    elif not all(checks[x] for x in ["trusted_mape", "trusted_p90", "trusted_bias"]):
        conclusion = "FAIL_TRUSTED_BW_BIASED"
    elif not checks["dynamic_tracking"]:
        conclusion = "FAIL_DYNAMIC_TRACKING"
    else:
        conclusion = "FAIL_TRACE_OR_EXPERIMENT"
    metrics["conclusion"] = conclusion
    (output / "fbbr_opiv2_validation_summary.json").write_text(
        json.dumps(metrics, indent=2, allow_nan=True) + "\n"
    )
    pd.DataFrame(
        [{"metric": k, "value": v} for k, v in metrics.items() if not isinstance(v, dict)]
    ).to_csv(output / "fbbr_opiv2_validation_metrics.csv", index=False)
    report = [
        "# F-BBR OPIv2 Validation Report", "",
        f"Runs completed: {len(passed)}/{len(manifest)}", "",
        "## Primary score validation", "",
        f"- Spearman rho: {observed:.6g}",
        f"- Cluster-bootstrap 95% CI: [{ci_low:.6g}, {ci_high:.6g}]",
        f"- Block-permutation p: {perm_p:.6g}",
        f"- ROC-AUC / AP: {metrics['core_ideal_auc']:.6g} / {metrics['core_ideal_average_precision']:.6g}",
        "", "## trusted_bw", "",
        f"- Median MAPE / P90 APE: {metrics['trusted_bw_median_mape']:.6g} / {metrics['trusted_bw_p90_ape']:.6g}",
        f"- Median signed bias: {metrics['trusted_bw_median_signed_bias']:.6g}",
        "", "## Acceptance", "",
    ]
    report += [f"- {'PASS' if value else 'FAIL'}: {name}" for name, value in checks.items()]
    report += ["", "## Conclusion", "", conclusion]
    (output / "fbbr_opiv2_validation_report.md").write_text("\n".join(report) + "\n")
    print(json.dumps(metrics, indent=2, allow_nan=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
