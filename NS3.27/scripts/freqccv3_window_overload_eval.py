#!/usr/bin/env python3
"""
Evaluate the user-specified window-level FreqCCv3 overload rule.

Rule per STFT window:
  non-overload (full-or-below) iff
    1) dominant recvrate frequency is close to a target ratio center, and
    2) at least rtt_below_ratio of RTT samples in the window are below maxRTT,
       where maxRTT is the maximum RTT observed in the previous two INT stages.
  otherwise overload.

Offline oracle:
  a window is labeled overload when the time-weighted mean queue occupancy
  exceeds oracle_qfrac_threshold * BDP.
"""

from __future__ import annotations

import argparse
import json
import math
import os
from dataclasses import asdict, dataclass

import numpy as np

NUMERIC_CACHE: dict[tuple[str, tuple[int, ...]], np.ndarray] = {}
UPPHASE_CACHE: dict[str, list[dict[str, float]]] = {}
QUEUE_CACHE: dict[str, np.ndarray] = {}


@dataclass
class Dataset:
    name: str
    kind: str
    trace_dir: str
    prefix: str
    num_flows: int
    queue_file: str
    bottleneck_mbps: float


@dataclass
class Metrics:
    windows: int
    accuracy: float
    balanced_accuracy: float
    precision_nonover: float
    recall_nonover: float
    precision_over: float
    recall_over: float
    tp_nonover: int
    fp_nonover: int
    fn_nonover: int
    tn_nonover: int


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--dataset",
        action="append",
        required=True,
        help="Dataset spec: name|kind|trace_dir|prefix|num_flows|queue_file|bottleneck_mbps",
    )
    parser.add_argument("--rate-window-mult", type=float, default=0.75)
    parser.add_argument(
        "--recv-trace-name",
        choices=["recvrate", "recvrate_raw"],
        default="recvrate",
        help="Which receive-rate trace to use as the recv spectral input.",
    )
    parser.add_argument("--overlap", type=float, default=0.9)
    parser.add_argument("--nfft-mult", type=int, default=4)
    parser.add_argument("--search-gate-low", type=float, default=0.4)
    parser.add_argument("--search-gate-high", type=float, default=1.3)
    parser.add_argument("--close-tols", default="0.30,0.25,0.20,0.15,0.10")
    parser.add_argument("--rtt-below-ratio", type=float, default=0.9)
    parser.add_argument("--oracle-qfrac-threshold", type=float, default=0.9)
    parser.add_argument("--center-mode", choices=["ref", "empirical-nonover"], default="ref")
    parser.add_argument("--center-value", type=float, default=1.0)
    parser.add_argument(
        "--causal-mode",
        choices=["observe", "delivery", "arrival", "band"],
        default="band",
        help="How to back-project an observed recv window to the queue-label window.",
    )
    parser.add_argument(
        "--pre-bottleneck-prop-ms",
        type=float,
        default=1.0,
        help="Propagation from sender to bottleneck queue entrance in ms.",
    )
    parser.add_argument(
        "--causal-quantile-low",
        type=float,
        default=0.10,
        help="Low quantile used when robustifying causal back-projection.",
    )
    parser.add_argument(
        "--causal-quantile-high",
        type=float,
        default=0.90,
        help="High quantile used when robustifying causal back-projection.",
    )
    parser.add_argument(
        "--causal-pad-ms",
        type=float,
        default=0.0,
        help="Extra padding added on both sides of the causal queue window in ms.",
    )
    parser.add_argument(
        "--causal-pad-window-frac",
        type=float,
        default=0.25,
        help="Extra padding as a fraction of the observed window length.",
    )
    parser.add_argument(
        "--tolerant-match-rtt-frac",
        type=float,
        default=0.0,
        help="If > 0, expand NONOVER truth/pred labels by this RTT fraction before scoring.",
    )
    parser.add_argument("--output", default="")
    return parser.parse_args()


def parse_float_list(text: str) -> list[float]:
    return [float(item) for item in text.split(",") if item.strip()]


def parse_dataset(spec: str) -> Dataset:
    parts = spec.split("|")
    if len(parts) != 7:
        raise ValueError(f"Bad dataset spec: {spec}")
    name, kind, trace_dir, prefix, num_flows, queue_file, bottleneck = parts
    queue_path = queue_file if os.path.isabs(queue_file) else os.path.join(trace_dir, queue_file)
    return Dataset(
        name=name,
        kind=kind,
        trace_dir=trace_dir,
        prefix=prefix,
        num_flows=int(num_flows),
        queue_file=queue_path,
        bottleneck_mbps=float(bottleneck),
    )


def load_numeric_file(path: str, cols: tuple[int, ...]) -> np.ndarray:
    key = (path, cols)
    if key in NUMERIC_CACHE:
        return NUMERIC_CACHE[key]
    rows = []
    with open(path, "r", encoding="utf-8") as handle:
        for line in handle:
            if not line.strip() or line.startswith("#"):
                continue
            parts = line.split()
            if len(parts) <= max(cols):
                continue
            try:
                rows.append([float(parts[idx]) for idx in cols])
            except ValueError:
                continue
    data = np.asarray(rows, dtype=float)
    NUMERIC_CACHE[key] = data
    return data


def load_upphase_file(path: str):
    if path in UPPHASE_CACHE:
        return UPPHASE_CACHE[path]
    rows = []
    with open(path, "r", encoding="utf-8") as handle:
        for line in handle:
            if not line.strip() or line.startswith("#"):
                continue
            parts = line.split()
            if len(parts) < 3:
                continue
            try:
                start = float(parts[0])
                duration_ms = float(parts[1])
                freq_hz = float(parts[2])
            except ValueError:
                continue
            rows.append({"start": start, "end": start + duration_ms / 1000.0, "freq_hz": freq_hz})
    UPPHASE_CACHE[path] = rows
    return rows


def load_queue_trace(path: str) -> np.ndarray:
    if path in QUEUE_CACHE:
        return QUEUE_CACHE[path]
    rows = []
    if not os.path.exists(path):
        data = np.empty((0, 2), dtype=float)
        QUEUE_CACHE[path] = data
        return data
    with open(path, "r", encoding="utf-8") as handle:
        for line in handle:
            if not line.strip() or line.startswith("#"):
                continue
            parts = line.split()
            if len(parts) < 2:
                continue
            try:
                rows.append([float(parts[0]), float(parts[1])])
            except ValueError:
                continue
    data = np.asarray(rows, dtype=float)
    QUEUE_CACHE[path] = data
    return data


def flow_paths(dataset: Dataset, flow_index: int) -> dict[str, str]:
    if dataset.num_flows == 1:
        base = os.path.join(dataset.trace_dir, dataset.prefix)
    else:
        base = os.path.join(dataset.trace_dir, f"{dataset.prefix}_{flow_index}")
    return {
        "recvrate": base + "_recvrate.txt",
        "recvrate_raw": base + "_recvrate_raw.txt",
        "rtt": base + "_rtt.txt",
        "qdelay": base + "_qdelay.txt",
        "upphase": base + "_upphase.txt",
    }


def interval_slice(data: np.ndarray, start: float, end: float) -> np.ndarray:
    if data.size == 0:
        return np.empty((0, data.shape[1] if data.ndim == 2 else 0))
    mask = (data[:, 0] >= start) & (data[:, 0] <= end)
    return data[mask]


def build_intervals(up_phases):
    return [
        {"ref_freq_hz": cur["freq_hz"], "int_start": cur["end"], "int_end": nxt["start"]}
        for cur, nxt in zip(up_phases[:-1], up_phases[1:])
        if nxt["start"] > cur["end"]
    ]


def compute_min_rtt_s(qdelay: np.ndarray, rtt: np.ndarray) -> float:
    if qdelay.size > 0 and qdelay.shape[1] >= 4:
        min_rtt_ms = np.min(qdelay[:, 3])
        if min_rtt_ms > 0:
            return float(min_rtt_ms / 1000.0)
    if rtt.size > 0 and rtt.shape[1] >= 2:
        min_rtt_ms = np.min(rtt[:, 1])
        if min_rtt_ms > 0:
            return float(min_rtt_ms / 1000.0)
    raise RuntimeError("Cannot infer min RTT")


def normalize_signal(values: np.ndarray, eps: float = 1e-12) -> np.ndarray:
    return (values - np.mean(values)) / (np.std(values) + eps)


def dominant_freq_in_band(
    times: np.ndarray,
    values: np.ndarray,
    ref_freq_hz: float,
    window_s: float,
    overlap: float,
    search_gate_low: float,
    search_gate_high: float,
    nfft_mult: int,
):
    if len(times) < 4:
        return []

    dt = float(np.mean(np.diff(times)))
    if not math.isfinite(dt) or dt <= 0:
        return []
    win_len = max(4, int(window_s / dt))
    if len(values) < win_len:
        return []
    hop_len = max(1, int(win_len * (1.0 - overlap)))
    nfft = max(win_len, int(win_len * nfft_mult))
    freqs = np.fft.rfftfreq(nfft, d=dt)
    mask = (freqs >= search_gate_low * ref_freq_hz) & (freqs <= search_gate_high * ref_freq_hz)
    if not np.any(mask):
        return []

    vals = normalize_signal(values)
    hann = np.hanning(win_len)
    out = []
    for start in range(0, len(vals) - win_len + 1, hop_len):
        segment = (vals[start : start + win_len] - np.mean(vals[start : start + win_len])) * hann
        spectrum = np.abs(np.fft.rfft(segment, n=nfft))
        band = spectrum[mask]
        if band.size == 0:
            continue
        idx = int(np.argmax(band))
        win_t0 = float(times[start])
        win_t1 = float(times[start + win_len - 1])
        out.append(
            {
                "win_t0": win_t0,
                "win_t1": win_t1,
                "dom_freq_hz": float(freqs[mask][idx]),
                "start_idx": int(start),
                "end_idx": int(start + win_len - 1),
            }
        )
    return out


def extend_queue_window(queue_data: np.ndarray, start: float, end: float) -> np.ndarray:
    if queue_data.size == 0:
        return np.empty((0, 2), dtype=float)

    mask = (queue_data[:, 0] >= start) & (queue_data[:, 0] <= end)
    interval = queue_data[mask]
    before = queue_data[queue_data[:, 0] < start]
    if len(before):
        first = before[-1].copy()
        first[0] = start
        interval = np.vstack([first, interval]) if len(interval) else np.asarray([first])
    if len(interval) and interval[-1, 0] < end:
        last = interval[-1].copy()
        last[0] = end
        interval = np.vstack([interval, last])
    return interval


def weighted_average(times: np.ndarray, values: np.ndarray) -> float | None:
    if len(times) == 0:
        return None
    if len(times) == 1:
        return float(values[0])
    deltas = np.diff(times)
    total = np.sum(deltas)
    if total <= 0.0:
        return float(np.mean(values))
    return float(np.sum(deltas * values[:-1]) / total)


def interpolate_values(sample_times: np.ndarray,
                       sample_values: np.ndarray,
                       query_times: np.ndarray) -> np.ndarray:
    if len(sample_times) == 0 or len(sample_values) == 0 or len(query_times) == 0:
        return np.asarray([], dtype=float)
    return np.interp(
        query_times,
        sample_times,
        sample_values,
        left=float(sample_values[0]),
        right=float(sample_values[-1]),
    )


def causal_queue_window(observed_times: np.ndarray,
                        observed_rtts: np.ndarray,
                        min_rtt_s: float,
                        observed_window_s: float,
                        args) -> tuple[float, float] | None:
    if len(observed_times) == 0 or len(observed_rtts) == 0:
        return None

    valid = np.isfinite(observed_times) & np.isfinite(observed_rtts)
    observed_times = observed_times[valid]
    observed_rtts = observed_rtts[valid]
    if len(observed_times) == 0:
        return None

    low_q = min(max(args.causal_quantile_low, 0.0), 1.0)
    high_q = min(max(args.causal_quantile_high, 0.0), 1.0)
    if low_q > high_q:
        low_q, high_q = high_q, low_q

    reverse_prop_s = min_rtt_s / 2.0
    pre_bottleneck_prop_s = max(args.pre_bottleneck_prop_ms, 0.0) / 1000.0
    arrival_times = observed_times - observed_rtts + pre_bottleneck_prop_s
    delivery_times = observed_times - reverse_prop_s

    if args.causal_mode == "observe":
        start = float(np.quantile(observed_times, low_q))
        end = float(np.quantile(observed_times, high_q))
    elif args.causal_mode == "delivery":
        start = float(np.quantile(delivery_times, low_q))
        end = float(np.quantile(delivery_times, high_q))
    elif args.causal_mode == "arrival":
        start = float(np.quantile(arrival_times, low_q))
        end = float(np.quantile(arrival_times, high_q))
    else:
        start = float(np.quantile(arrival_times, low_q))
        end = float(np.quantile(delivery_times, high_q))

    pad_s = max(
        max(args.causal_pad_ms, 0.0) / 1000.0,
        max(args.causal_pad_window_frac, 0.0) * max(observed_window_s, 0.0),
    )
    start -= pad_s
    end += pad_s
    if end <= start:
        center = 0.5 * (start + end)
        half = max(0.5 * observed_window_s, 1e-6)
        start = center - half
        end = center + half
    return max(0.0, start), max(0.0, end)


def compute_metrics(rows):
    tp_nonover = sum(1 for row in rows if row["pred_nonover"] and row["true_nonover"])
    fp_nonover = sum(1 for row in rows if row["pred_nonover"] and not row["true_nonover"])
    fn_nonover = sum(1 for row in rows if (not row["pred_nonover"]) and row["true_nonover"])
    tn_nonover = sum(1 for row in rows if (not row["pred_nonover"]) and (not row["true_nonover"]))
    total = len(rows)
    accuracy = (tp_nonover + tn_nonover) / total if total else 0.0
    precision_nonover = tp_nonover / (tp_nonover + fp_nonover) if (tp_nonover + fp_nonover) else 0.0
    recall_nonover = tp_nonover / (tp_nonover + fn_nonover) if (tp_nonover + fn_nonover) else 0.0
    tp_over = tn_nonover
    fp_over = fn_nonover
    fn_over = fp_nonover
    precision_over = tp_over / (tp_over + fp_over) if (tp_over + fp_over) else 0.0
    recall_over = tp_over / (tp_over + fn_over) if (tp_over + fn_over) else 0.0
    balanced_accuracy = 0.5 * (recall_nonover + recall_over)
    return Metrics(
        windows=total,
        accuracy=accuracy,
        balanced_accuracy=balanced_accuracy,
        precision_nonover=precision_nonover,
        recall_nonover=recall_nonover,
        precision_over=precision_over,
        recall_over=recall_over,
        tp_nonover=tp_nonover,
        fp_nonover=fp_nonover,
        fn_nonover=fn_nonover,
        tn_nonover=tn_nonover,
    )


def dilate_boolean_by_time(times: np.ndarray, labels: np.ndarray, radius_s: float) -> np.ndarray:
    if len(times) == 0:
        return np.asarray([], dtype=bool)
    if radius_s <= 0.0:
        return labels.astype(bool, copy=True)
    positive_times = np.asarray(times[labels], dtype=float)
    if len(positive_times) == 0:
        return np.zeros(len(times), dtype=bool)
    left = np.searchsorted(positive_times, times - radius_s, side="left")
    right = np.searchsorted(positive_times, times + radius_s, side="right")
    return right > left


def tolerant_rows(rows: list[dict], tolerance_rtt_frac: float) -> list[dict]:
    if tolerance_rtt_frac <= 0.0 or not rows:
        return [{**row} for row in rows]

    grouped: dict[tuple[str, int], list[dict]] = {}
    for row in rows:
        grouped.setdefault((row["dataset"], row["flow"]), []).append(row)

    out = []
    for group_rows in grouped.values():
        group_rows = sorted(group_rows, key=lambda item: 0.5 * (item["win_t0"] + item["win_t1"]))
        times = np.asarray([0.5 * (row["win_t0"] + row["win_t1"]) for row in group_rows], dtype=float)
        true_nonover = np.asarray([bool(row["true_nonover"]) for row in group_rows], dtype=bool)
        pred_nonover = np.asarray([bool(row["pred_nonover"]) for row in group_rows], dtype=bool)
        min_rtt_s = float(group_rows[0]["min_rtt_s"])
        radius_s = max(tolerance_rtt_frac, 0.0) * max(min_rtt_s, 0.0)
        true_dilated = dilate_boolean_by_time(times, true_nonover, radius_s)
        pred_dilated = dilate_boolean_by_time(times, pred_nonover, radius_s)
        for row, true_flag, pred_flag in zip(group_rows, true_dilated, pred_dilated):
            out.append({**row, "true_nonover": bool(true_flag), "pred_nonover": bool(pred_flag)})
    return out


def neighborhood_metrics(rows: list[dict], tolerance_rtt_frac: float) -> dict[str, float | int | None]:
    if not rows:
        return {
            "tolerance_rtt_frac": tolerance_rtt_frac,
            "neighbor_balanced_accuracy": 0.0,
            "hit_nonover_rate": 0.0,
            "clean_over_rate": 0.0,
            "supported_pred_nonover_rate": 0.0,
            "true_nonover_total": 0,
            "true_over_total": 0,
            "pred_nonover_total": 0,
        }

    grouped: dict[tuple[str, int], list[dict]] = {}
    for row in rows:
        grouped.setdefault((row["dataset"], row["flow"]), []).append(row)

    true_nonover_hits = 0
    true_nonover_total = 0
    true_over_clean = 0
    true_over_total = 0
    pred_nonover_supported = 0
    pred_nonover_total = 0

    for group_rows in grouped.values():
        group_rows = sorted(group_rows, key=lambda item: 0.5 * (item["win_t0"] + item["win_t1"]))
        times = np.asarray([0.5 * (row["win_t0"] + row["win_t1"]) for row in group_rows], dtype=float)
        true_nonover = np.asarray([bool(row["true_nonover"]) for row in group_rows], dtype=bool)
        pred_nonover = np.asarray([bool(row["pred_nonover"]) for row in group_rows], dtype=bool)
        min_rtt_s = float(group_rows[0]["min_rtt_s"])
        radius_s = max(tolerance_rtt_frac, 0.0) * max(min_rtt_s, 0.0)

        pred_dilated = dilate_boolean_by_time(times, pred_nonover, radius_s)
        true_dilated = dilate_boolean_by_time(times, true_nonover, radius_s)

        true_nonover_hits += int(np.sum(pred_dilated & true_nonover))
        true_nonover_total += int(np.sum(true_nonover))
        true_over_clean += int(np.sum((~pred_dilated) & (~true_nonover)))
        true_over_total += int(np.sum(~true_nonover))
        pred_nonover_supported += int(np.sum(true_dilated & pred_nonover))
        pred_nonover_total += int(np.sum(pred_nonover))

    hit_nonover_rate = true_nonover_hits / true_nonover_total if true_nonover_total else 0.0
    clean_over_rate = true_over_clean / true_over_total if true_over_total else 0.0
    supported_pred_nonover_rate = (
        pred_nonover_supported / pred_nonover_total if pred_nonover_total else 0.0
    )
    return {
        "tolerance_rtt_frac": tolerance_rtt_frac,
        "neighbor_balanced_accuracy": 0.5 * (hit_nonover_rate + clean_over_rate),
        "hit_nonover_rate": hit_nonover_rate,
        "clean_over_rate": clean_over_rate,
        "supported_pred_nonover_rate": supported_pred_nonover_rate,
        "true_nonover_total": true_nonover_total,
        "true_over_total": true_over_total,
        "pred_nonover_total": pred_nonover_total,
    }


def summarize_ratios(rows):
    if not rows:
        return {
            "count": 0,
            "ratio_median": None,
            "ratio_p10": None,
            "ratio_p90": None,
        }
    ratios = np.asarray([row["ratio"] for row in rows], dtype=float)
    ratios = np.sort(ratios)
    return {
        "count": int(len(ratios)),
        "ratio_median": float(np.median(ratios)),
        "ratio_p10": float(np.percentile(ratios, 10)),
        "ratio_p90": float(np.percentile(ratios, 90)),
    }


def collect_rows(dataset: Dataset, args) -> list[dict]:
    queue_data = load_queue_trace(dataset.queue_file)
    rows = []

    for flow_index in range(1, dataset.num_flows + 1):
        paths = flow_paths(dataset, flow_index)
        recvrate = load_numeric_file(paths[args.recv_trace_name], (0, 1))
        rtt = load_numeric_file(paths["rtt"], (0, 2))  # raw RTT
        qdelay = load_numeric_file(paths["qdelay"], (0, 1, 2, 3))
        upphase = load_upphase_file(paths["upphase"])
        min_rtt_s = compute_min_rtt_s(qdelay, rtt)
        bdp_bytes = dataset.bottleneck_mbps * 1_000_000.0 * min_rtt_s / 8.0
        int_intervals = build_intervals(upphase)
        interval_rtt_max = []
        for interval in int_intervals:
            rtt_slice = interval_slice(rtt, interval["int_start"], interval["int_end"])
            interval_rtt_max.append(float(np.max(rtt_slice[:, 1])) if len(rtt_slice) else None)

        for idx, interval in enumerate(int_intervals):
            prev_candidates = [item for item in interval_rtt_max[max(0, idx - 2) : idx] if item is not None]
            if not prev_candidates:
                continue
            hist_max_rtt = float(max(prev_candidates))
            recv_slice = interval_slice(recvrate, interval["int_start"], interval["int_end"])
            if len(recv_slice) < 4:
                continue
            window_s = args.rate_window_mult * min_rtt_s
            windows = dominant_freq_in_band(
                recv_slice[:, 0],
                recv_slice[:, 1],
                interval["ref_freq_hz"],
                window_s,
                args.overlap,
                args.search_gate_low,
                args.search_gate_high,
                args.nfft_mult,
            )
            for window in windows:
                win_t0 = float(window["win_t0"])
                win_t1 = float(window["win_t1"])
                dom_freq = float(window["dom_freq_hz"])
                rtt_win = interval_slice(rtt, win_t0, win_t1)
                if len(rtt_win) == 0:
                    continue
                below_ratio = float(np.mean(rtt_win[:, 1] < hist_max_rtt))
                obs_times = recv_slice[
                    window["start_idx"] : window["end_idx"] + 1, 0
                ]
                obs_rtts = interpolate_values(rtt[:, 0], rtt[:, 1], obs_times)
                observed_window_s = max(win_t1 - win_t0, 0.0)
                cause_window = causal_queue_window(
                    obs_times,
                    obs_rtts,
                    min_rtt_s,
                    observed_window_s,
                    args,
                )
                if cause_window is None:
                    continue
                cause_t0, cause_t1 = cause_window
                queue_win = extend_queue_window(queue_data, cause_t0, cause_t1)
                if len(queue_win) == 0 or bdp_bytes <= 0:
                    continue
                qfrac = queue_win[:, 1] / bdp_bytes
                avg_qfrac = weighted_average(queue_win[:, 0], qfrac)
                if avg_qfrac is None:
                    continue
                rows.append(
                    {
                        "dataset": dataset.name,
                        "kind": dataset.kind,
                        "flow": flow_index,
                        "interval_idx": idx,
                        "win_t0": win_t0,
                        "win_t1": win_t1,
                        "cause_t0": cause_t0,
                        "cause_t1": cause_t1,
                        "ref_freq_hz": interval["ref_freq_hz"],
                        "dom_freq_hz": dom_freq,
                        "ratio": dom_freq / interval["ref_freq_hz"],
                        "min_rtt_s": min_rtt_s,
                        "hist_max_rtt": hist_max_rtt,
                        "below_ratio": below_ratio,
                        "avg_qfrac": float(avg_qfrac),
                    }
                )
    return rows


def main():
    args = parse_args()
    datasets = [parse_dataset(spec) for spec in args.dataset]
    all_rows = []
    for dataset in datasets:
        all_rows.extend(collect_rows(dataset, args))

    if args.center_mode == "empirical-nonover":
        center_rows = [row for row in all_rows if row["avg_qfrac"] <= args.oracle_qfrac_threshold]
        center_summary = summarize_ratios(center_rows)
        if center_summary["ratio_median"] is None:
            raise RuntimeError("Cannot estimate empirical center from non-overload oracle windows")
        center_ratio = center_summary["ratio_median"]
    else:
        center_rows = []
        center_summary = summarize_ratios(center_rows)
        center_ratio = args.center_value

    results = []
    for close_tol in parse_float_list(args.close_tols):
        rows = []
        for row in all_rows:
            close_ok = abs(row["ratio"] - center_ratio) <= close_tol
            pred_nonover = close_ok and row["below_ratio"] >= args.rtt_below_ratio
            true_nonover = row["avg_qfrac"] <= args.oracle_qfrac_threshold
            rows.append({**row, "pred_nonover": pred_nonover, "true_nonover": true_nonover})

        overall = compute_metrics(rows)
        tolerant = compute_metrics(tolerant_rows(rows, args.tolerant_match_rtt_frac))
        neighbor = neighborhood_metrics(rows, args.tolerant_match_rtt_frac)
        per_dataset = {}
        tolerant_per_dataset = {}
        neighbor_per_dataset = {}
        for dataset in datasets:
            ds_rows = [row for row in rows if row["dataset"] == dataset.name]
            per_dataset[dataset.name] = asdict(compute_metrics(ds_rows))
            tol_ds_rows = tolerant_rows(ds_rows, args.tolerant_match_rtt_frac)
            tolerant_per_dataset[dataset.name] = asdict(compute_metrics(tol_ds_rows))
            neighbor_per_dataset[dataset.name] = neighborhood_metrics(
                ds_rows,
                args.tolerant_match_rtt_frac,
            )

        result = {
            "close_tol": close_tol,
            "overall": asdict(overall),
            "tolerant_overall": asdict(tolerant),
            "neighbor_overall": neighbor,
            "per_dataset": per_dataset,
            "tolerant_per_dataset": tolerant_per_dataset,
            "neighbor_per_dataset": neighbor_per_dataset,
        }
        results.append(result)

    results.sort(
        key=lambda item: (
            item["neighbor_overall"]["neighbor_balanced_accuracy"],
            item["neighbor_overall"]["hit_nonover_rate"],
            item["tolerant_overall"]["balanced_accuracy"],
            item["overall"]["balanced_accuracy"],
        ),
        reverse=True,
    )

    print("Window-level overload evaluation:")
    print(
        f"center_mode={args.center_mode} "
        f"recv_trace={args.recv_trace_name} "
        f"center_ratio={center_ratio:.4f}"
    )
    if center_summary["count"]:
        print(
            "empirical_nonover_ratio "
            f"count={center_summary['count']} "
            f"median={center_summary['ratio_median']:.4f} "
            f"p10={center_summary['ratio_p10']:.4f} "
            f"p90={center_summary['ratio_p90']:.4f}"
        )
    for item in results:
        metrics = item["overall"]
        tol_metrics = item["tolerant_overall"]
        neighbor = item["neighbor_overall"]
        print(
            f"tol={item['close_tol']:.2f} "
            f"windows={metrics['windows']} "
            f"nbr_bal_acc={neighbor['neighbor_balanced_accuracy']:.4f} "
            f"hit_nonover={neighbor['hit_nonover_rate']:.4f} "
            f"clean_over={neighbor['clean_over_rate']:.4f} "
            f"tol_bal_acc={tol_metrics['balanced_accuracy']:.4f} "
            f"tol_acc={tol_metrics['accuracy']:.4f} "
            f"bal_acc={metrics['balanced_accuracy']:.4f} "
            f"acc={metrics['accuracy']:.4f} "
            f"prec_nonover={metrics['precision_nonover']:.4f} "
            f"rec_nonover={metrics['recall_nonover']:.4f} "
            f"prec_over={metrics['precision_over']:.4f} "
            f"rec_over={metrics['recall_over']:.4f}"
        )

    if args.output:
        payload = {
            "config": {
                "rate_window_mult": args.rate_window_mult,
                "recv_trace_name": args.recv_trace_name,
                "overlap": args.overlap,
                "nfft_mult": args.nfft_mult,
                "search_gate_low": args.search_gate_low,
                "search_gate_high": args.search_gate_high,
                "rtt_below_ratio": args.rtt_below_ratio,
                "oracle_qfrac_threshold": args.oracle_qfrac_threshold,
                "center_mode": args.center_mode,
                "center_value": args.center_value,
                "center_ratio_used": center_ratio,
                "causal_mode": args.causal_mode,
                "pre_bottleneck_prop_ms": args.pre_bottleneck_prop_ms,
                "causal_quantile_low": args.causal_quantile_low,
                "causal_quantile_high": args.causal_quantile_high,
                "causal_pad_ms": args.causal_pad_ms,
                "causal_pad_window_frac": args.causal_pad_window_frac,
                "tolerant_match_rtt_frac": args.tolerant_match_rtt_frac,
            },
            "center_summary": center_summary,
            "results": results,
        }
        with open(args.output, "w", encoding="utf-8") as handle:
            json.dump(payload, handle, indent=2)
        print(f"\nSaved summary to {args.output}")


if __name__ == "__main__":
    main()
