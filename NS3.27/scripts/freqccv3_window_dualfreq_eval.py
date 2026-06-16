#!/usr/bin/env python3
"""
Evaluate joint recvrate + RTT spectral paradigms for window-level load judgment.

Per recv-anchored window, this script extracts:
  - sender-UP band template from the immediately preceding UP phase
  - recv window spectral profile
  - RTT window spectral profile
  - queue-oracle NONOVER/OVER label via causal back-projection

It then searches multiple interpretable fusion paradigms:
  - strict_dual_peak
  - strict_dual_template
  - recv_template_rtt_peak
  - recv_peak_rtt_template
  - soft_score
  - soft_score_with_alignment
  - logreg

Dataset spec format:
  name|kind|trace_dir|prefix|num_flows|queue_file|bottleneck_mbps
"""

from __future__ import annotations

import argparse
import itertools
import json
import math
import os
from dataclasses import asdict, dataclass

import numpy as np
from scipy import optimize


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
    parser.add_argument(
        "--rtt-signal",
        choices=["rtt", "srtt"],
        default="rtt",
        help="Which RTT column from *_rtt.txt to use for RTT-side spectral input.",
    )
    parser.add_argument("--rtt-window-mult", type=float, default=1.5)
    parser.add_argument("--overlap", type=float, default=0.9)
    parser.add_argument("--search-gate-low", type=float, default=0.5)
    parser.add_argument("--search-gate-high", type=float, default=1.3)
    parser.add_argument("--nfft-mult", type=int, default=4)
    parser.add_argument("--uniform-step-ms", type=float, default=1.0)
    parser.add_argument("--shape-bins", type=int, default=24)
    parser.add_argument("--template-local-peak-frac", type=float, default=0.7)
    parser.add_argument("--oracle-qfrac-threshold", type=float, default=0.9)
    parser.add_argument(
        "--causal-mode",
        choices=["observe", "delivery", "arrival", "band"],
        default="delivery",
    )
    parser.add_argument("--pre-bottleneck-prop-ms", type=float, default=1.0)
    parser.add_argument("--causal-quantile-low", type=float, default=0.10)
    parser.add_argument("--causal-quantile-high", type=float, default=0.90)
    parser.add_argument("--causal-pad-ms", type=float, default=0.0)
    parser.add_argument("--causal-pad-window-frac", type=float, default=0.25)
    parser.add_argument("--tolerant-match-rtt-frac", type=float, default=0.5)
    parser.add_argument("--peak-close-tols", default="0.10,0.16,0.22,0.30")
    parser.add_argument("--shape-dist-thrs", default="0.35,0.45,0.60,0.80")
    parser.add_argument("--energy-thrs", default="0.08,0.12,0.16,0.22")
    parser.add_argument("--peakrel-thrs", default="0.10,0.14,0.20,0.28")
    parser.add_argument("--joint-align-thrs", default="0.08,0.12,0.16,0.22")
    parser.add_argument("--score-thrs", default="0.45,0.55,0.65,0.75")
    parser.add_argument("--soft-weights", default="0.55:0.45,0.65:0.35,0.75:0.25")
    parser.add_argument("--rtt-upper-thrs", default="0.60,0.70,0.80,0.95,1.10")
    parser.add_argument(
        "--paradigms",
        default="strict_dual_peak,strict_dual_template,recv_template_rtt_peak,recv_peak_rtt_template,recv_template_align_rttupper,soft_score,soft_score_with_alignment,soft_score_align_rttupper,logreg",
    )
    parser.add_argument("--top-k", type=int, default=20)
    parser.add_argument("--output", default="")
    return parser.parse_args()


def parse_float_list(text: str) -> list[float]:
    return [float(item) for item in text.split(",") if item.strip()]


def parse_weight_pairs(text: str) -> list[tuple[float, float]]:
    pairs = []
    for item in text.split(","):
        token = item.strip()
        if not token:
            continue
        left, right = token.split(":")
        pairs.append((float(left), float(right)))
    return pairs


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
    if rows:
        data = np.asarray(rows, dtype=float)
        if len(data) > 1:
            data = data[np.argsort(data[:, 0], kind="stable")]
    else:
        data = np.empty((0, len(cols)), dtype=float)
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
    if rows:
        data = np.asarray(rows, dtype=float)
        if len(data) > 1:
            data = data[np.argsort(data[:, 0], kind="stable")]
    else:
        data = np.empty((0, 2), dtype=float)
    QUEUE_CACHE[path] = data
    return data


def flow_paths(dataset: Dataset, flow_index: int) -> dict[str, str]:
    if dataset.num_flows == 1:
        base = os.path.join(dataset.trace_dir, dataset.prefix)
    else:
        base = os.path.join(dataset.trace_dir, f"{dataset.prefix}_{flow_index}")
    return {
        "sendrate": base + "_sendrate.txt",
        "recvrate": base + "_recvrate.txt",
        "recvrate_raw": base + "_recvrate_raw.txt",
        "rtt": base + "_rtt.txt",
        "qdelay": base + "_qdelay.txt",
        "upphase": base + "_upphase.txt",
    }


def interval_slice(data: np.ndarray, start: float, end: float) -> np.ndarray:
    if data.size == 0:
        cols = data.shape[1] if data.ndim == 2 else 0
        return np.empty((0, cols), dtype=float)
    left = int(np.searchsorted(data[:, 0], start, side="left"))
    right = int(np.searchsorted(data[:, 0], end, side="right"))
    return data[left:right]


def build_intervals(up_phases):
    return [
        {
            "up_start": cur["start"],
            "up_end": cur["end"],
            "ref_freq_hz": cur["freq_hz"],
            "int_start": cur["end"],
            "int_end": nxt["start"],
        }
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


def extend_queue_window(queue_data: np.ndarray, start: float, end: float) -> np.ndarray:
    if queue_data.size == 0:
        return np.empty((0, 2), dtype=float)
    left = int(np.searchsorted(queue_data[:, 0], start, side="left"))
    right = int(np.searchsorted(queue_data[:, 0], end, side="right"))
    interval = queue_data[left:right]
    if left > 0:
        first = queue_data[left - 1].copy()
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


def interpolate_values(sample_times: np.ndarray, sample_values: np.ndarray, query_times: np.ndarray) -> np.ndarray:
    if len(sample_times) == 0 or len(sample_values) == 0 or len(query_times) == 0:
        return np.asarray([], dtype=float)
    order = np.argsort(sample_times, kind="stable")
    sample_times = sample_times[order]
    sample_values = sample_values[order]
    uniq_times, uniq_index = np.unique(sample_times, return_index=True)
    uniq_values = sample_values[uniq_index]
    if len(uniq_times) == 1:
        return np.full(len(query_times), float(uniq_values[0]), dtype=float)
    return np.interp(
        query_times,
        uniq_times,
        uniq_values,
        left=float(uniq_values[0]),
        right=float(uniq_values[-1]),
    )


def causal_queue_window(
    observed_times: np.ndarray,
    observed_rtts: np.ndarray,
    min_rtt_s: float,
    observed_window_s: float,
    args,
) -> tuple[float, float] | None:
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


def neighborhood_metrics(rows: list[dict], tolerance_rtt_frac: float) -> dict[str, float | int]:
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


def normalize_signal(values: np.ndarray, eps: float = 1e-12) -> np.ndarray:
    return (values - np.mean(values)) / (np.std(values) + eps)


def dedupe_sorted_trace(data: np.ndarray) -> np.ndarray:
    if len(data) == 0:
        return data
    order = np.argsort(data[:, 0], kind="stable")
    data = data[order]
    uniq_times, uniq_index = np.unique(data[:, 0], return_index=True)
    dedup = np.column_stack([uniq_times, data[uniq_index, 1]])
    return dedup


def resample_uniform(trace: np.ndarray, start: float, end: float, step_s: float) -> tuple[np.ndarray, np.ndarray]:
    if trace.size == 0 or end <= start or step_s <= 0.0:
        return np.asarray([], dtype=float), np.asarray([], dtype=float)
    clipped = interval_slice(trace, start, end)
    if len(clipped) == 0:
        trace = dedupe_sorted_trace(trace)
        if len(trace) == 0:
            return np.asarray([], dtype=float), np.asarray([], dtype=float)
        clipped = trace
    else:
        clipped = dedupe_sorted_trace(clipped)
    times = np.arange(start, end + 0.5 * step_s, step_s, dtype=float)
    values = interpolate_values(clipped[:, 0], clipped[:, 1], times)
    return times, values


def local_peak_indices(values: np.ndarray) -> list[int]:
    peaks: list[int] = []
    if len(values) < 3:
        return peaks
    for idx in range(1, len(values) - 1):
        if values[idx] >= values[idx - 1] and values[idx] >= values[idx + 1]:
            peaks.append(idx)
    return peaks


def choose_band_peak(
    freqs: np.ndarray,
    band_mag: np.ndarray,
    ref_freq_hz: float,
    local_peak_frac: float,
) -> tuple[int, float]:
    if len(band_mag) == 0:
        return -1, 0.0
    max_mag = float(np.max(band_mag))
    peak_candidates = []
    for idx in local_peak_indices(band_mag):
        if band_mag[idx] >= local_peak_frac * max_mag:
            peak_candidates.append(idx)
    if not peak_candidates:
        best = int(np.argmax(band_mag))
        return best, float(band_mag[best])
    best = min(peak_candidates, key=lambda idx: abs(freqs[idx] - ref_freq_hz))
    return int(best), float(band_mag[best])


def build_spectrum_profile(
    values: np.ndarray,
    sample_step_s: float,
    ref_freq_hz: float,
    gate_low: float,
    gate_high: float,
    nfft_mult: int,
    shape_bins: int,
    local_peak_frac: float,
) -> dict:
    profile = {
        "valid": False,
        "peak_freq_hz": 0.0,
        "peak_ratio": 0.0,
        "band_energy_ratio": 0.0,
        "band_peak_rel_total": 0.0,
        "band_peak_rel_global": 0.0,
        "shape_distance": math.inf,
        "band_shape": [],
        "total_energy": 0.0,
    }
    if len(values) < 8 or sample_step_s <= 0.0 or ref_freq_hz <= 0.0:
        return profile

    signal = normalize_signal(values)
    signal_len = len(signal)
    nfft = max(signal_len, int(signal_len * nfft_mult))
    hann = np.hanning(signal_len)
    spectrum = np.abs(np.fft.rfft((signal - np.mean(signal)) * hann, n=nfft))
    freqs = np.fft.rfftfreq(nfft, d=sample_step_s)
    if len(freqs) <= 1:
        return profile

    total_energy = float(np.sum(spectrum[1:]))
    global_peak = float(np.max(spectrum[1:])) if len(spectrum) > 1 else 0.0
    if total_energy <= 0.0 or global_peak <= 0.0:
        return profile

    band_mask = (freqs >= gate_low * ref_freq_hz) & (freqs <= gate_high * ref_freq_hz)
    if not np.any(band_mask):
        return profile

    band_freqs = freqs[band_mask]
    band_mag = spectrum[band_mask]
    if len(band_mag) == 0:
        return profile

    peak_idx, peak_mag = choose_band_peak(band_freqs, band_mag, ref_freq_hz, local_peak_frac)
    if peak_idx < 0 or peak_mag <= 0.0:
        return profile

    band_energy = float(np.sum(band_mag))
    band_shape = []
    if shape_bins <= 1:
        band_shape = [float(peak_mag)]
    else:
        target_freqs = np.linspace(gate_low * ref_freq_hz, gate_high * ref_freq_hz, shape_bins)
        band_shape = np.interp(target_freqs, freqs, spectrum, left=0.0, right=0.0).tolist()
    shape_sum = float(np.sum(band_shape))
    if shape_sum > 0.0:
        band_shape = [value / shape_sum for value in band_shape]
    else:
        band_shape = []

    peak_freq_hz = float(band_freqs[peak_idx])
    profile.update(
        {
            "valid": bool(band_shape),
            "peak_freq_hz": peak_freq_hz,
            "peak_ratio": peak_freq_hz / ref_freq_hz,
            "band_energy_ratio": band_energy / total_energy,
            "band_peak_rel_total": peak_mag / total_energy,
            "band_peak_rel_global": peak_mag / global_peak,
            "band_shape": band_shape,
            "total_energy": total_energy,
        }
    )
    return profile


def shape_distance(lhs: list[float], rhs: list[float]) -> float:
    if not lhs or not rhs or len(lhs) != len(rhs):
        return math.inf
    return float(np.sum(np.abs(np.asarray(lhs, dtype=float) - np.asarray(rhs, dtype=float))))


def aligned_window(trace: np.ndarray, center: float, window_s: float) -> np.ndarray:
    if trace.size == 0 or window_s <= 0.0:
        cols = trace.shape[1] if trace.ndim == 2 else 0
        return np.empty((0, cols), dtype=float)
    start = center - 0.5 * window_s
    end = center + 0.5 * window_s
    return interval_slice(trace, start, end)


def collect_rows(dataset: Dataset, args) -> list[dict]:
    queue_data = load_queue_trace(dataset.queue_file)
    rows = []
    step_s = max(args.uniform_step_ms, 0.1) / 1000.0
    rtt_col = 3 if args.rtt_signal == "srtt" else 2

    for flow_index in range(1, dataset.num_flows + 1):
        paths = flow_paths(dataset, flow_index)
        sendrate = load_numeric_file(paths["sendrate"], (0, 1))
        recvrate = load_numeric_file(paths[args.recv_trace_name], (0, 1))
        rtt = load_numeric_file(paths["rtt"], (0, rtt_col))
        qdelay = load_numeric_file(paths["qdelay"], (0, 1, 2, 3))
        upphase = load_upphase_file(paths["upphase"])
        min_rtt_s = compute_min_rtt_s(qdelay, rtt)
        bdp_bytes = dataset.bottleneck_mbps * 1_000_000.0 * min_rtt_s / 8.0
        int_intervals = build_intervals(upphase)
        interval_rtt_max = []
        for interval in int_intervals:
            rtt_slice = interval_slice(rtt, interval["int_start"], interval["int_end"])
            interval_rtt_max.append(float(np.max(rtt_slice[:, 1])) if len(rtt_slice) else None)

        recv_window_s = max(args.rate_window_mult, 0.1) * min_rtt_s
        rtt_window_s = max(args.rtt_window_mult, 0.1) * min_rtt_s

        for idx, interval in enumerate(int_intervals):
            prev_candidates = [item for item in interval_rtt_max[max(0, idx - 2) : idx] if item is not None]
            if not prev_candidates:
                continue
            recv_slice = interval_slice(recvrate, interval["int_start"], interval["int_end"])
            if len(recv_slice) < 8:
                continue

            up_send = interval_slice(sendrate, interval["up_start"], interval["up_end"])
            if len(up_send) < 4:
                continue
            up_times, up_values = resample_uniform(up_send, interval["up_start"], interval["up_end"], step_s)
            sender_profile = build_spectrum_profile(
                up_values,
                step_s,
                interval["ref_freq_hz"],
                args.search_gate_low,
                args.search_gate_high,
                args.nfft_mult,
                args.shape_bins,
                args.template_local_peak_frac,
            )
            if not sender_profile["valid"]:
                continue

            dt_recv = float(np.mean(np.diff(recv_slice[:, 0])))
            if not math.isfinite(dt_recv) or dt_recv <= 0.0:
                continue
            recv_win_len = max(8, int(recv_window_s / dt_recv))
            if len(recv_slice) < recv_win_len:
                continue
            hop_len = max(1, int(recv_win_len * (1.0 - args.overlap)))

            for start_idx in range(0, len(recv_slice) - recv_win_len + 1, hop_len):
                end_idx = start_idx + recv_win_len - 1
                win_t0 = float(recv_slice[start_idx, 0])
                win_t1 = float(recv_slice[end_idx, 0])
                win_center = 0.5 * (win_t0 + win_t1)

                recv_obs = recv_slice[start_idx : end_idx + 1]
                recv_times_uniform, recv_values_uniform = resample_uniform(
                    recv_obs,
                    win_t0,
                    win_t1,
                    step_s,
                )
                recv_profile = build_spectrum_profile(
                    recv_values_uniform,
                    step_s,
                    interval["ref_freq_hz"],
                    args.search_gate_low,
                    args.search_gate_high,
                    args.nfft_mult,
                    args.shape_bins,
                    args.template_local_peak_frac,
                )

                rtt_obs = aligned_window(rtt, win_center, rtt_window_s)
                if len(rtt_obs) < 6:
                    continue
                rtt_t0 = max(interval["int_start"], win_center - 0.5 * rtt_window_s)
                rtt_t1 = min(interval["int_end"], win_center + 0.5 * rtt_window_s)
                rtt_times_uniform, rtt_values_uniform = resample_uniform(
                    rtt_obs,
                    rtt_t0,
                    rtt_t1,
                    step_s,
                )
                rtt_profile = build_spectrum_profile(
                    rtt_values_uniform,
                    step_s,
                    interval["ref_freq_hz"],
                    args.search_gate_low,
                    args.search_gate_high,
                    args.nfft_mult,
                    args.shape_bins,
                    args.template_local_peak_frac,
                )

                recv_shape_dist = shape_distance(recv_profile["band_shape"], sender_profile["band_shape"])
                rtt_shape_dist = shape_distance(rtt_profile["band_shape"], sender_profile["band_shape"])

                obs_times = recv_obs[:, 0]
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
                        "min_rtt_s": min_rtt_s,
                        "ref_freq_hz": interval["ref_freq_hz"],
                        "true_nonover": bool(avg_qfrac <= args.oracle_qfrac_threshold),
                        "avg_qfrac": float(avg_qfrac),
                        "sender_peak_ratio": float(sender_profile["peak_ratio"]),
                        "recv_valid": bool(recv_profile["valid"]),
                        "recv_peak_ratio": float(recv_profile["peak_ratio"]),
                        "recv_peak_close_ref": abs(float(recv_profile["peak_ratio"]) - 1.0),
                        "recv_peak_close_sender": abs(float(recv_profile["peak_ratio"]) - float(sender_profile["peak_ratio"])),
                        "recv_band_energy": float(recv_profile["band_energy_ratio"]),
                        "recv_band_peak_rel_total": float(recv_profile["band_peak_rel_total"]),
                        "recv_band_peak_rel_global": float(recv_profile["band_peak_rel_global"]),
                        "recv_shape_dist": float(recv_shape_dist),
                        "rtt_valid": bool(rtt_profile["valid"]),
                        "rtt_peak_ratio": float(rtt_profile["peak_ratio"]),
                        "rtt_peak_close_ref": abs(float(rtt_profile["peak_ratio"]) - 1.0),
                        "rtt_peak_close_sender": abs(float(rtt_profile["peak_ratio"]) - float(sender_profile["peak_ratio"])),
                        "rtt_band_energy": float(rtt_profile["band_energy_ratio"]),
                        "rtt_band_peak_rel_total": float(rtt_profile["band_peak_rel_total"]),
                        "rtt_band_peak_rel_global": float(rtt_profile["band_peak_rel_global"]),
                        "rtt_shape_dist": float(rtt_shape_dist),
                        "peak_alignment": abs(float(recv_profile["peak_ratio"]) - float(rtt_profile["peak_ratio"])),
                    }
                )
    return rows


def compute_metrics(rows: list[dict]) -> Metrics:
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


def score_from_feature(value: float, threshold: float, higher_is_better: bool) -> float:
    if higher_is_better:
        if threshold <= 0.0:
            return 1.0 if value > 0.0 else 0.0
        return float(np.clip(value / threshold, 0.0, 1.5))
    if threshold <= 0.0:
        return 1.0
    return float(np.clip(threshold / max(value, 1e-12), 0.0, 1.5))


def evaluate_rule(rows: list[dict], pred_fn) -> dict:
    pred_rows = [{**row, "pred_nonover": bool(pred_fn(row))} for row in rows]
    overall = compute_metrics(pred_rows)
    tolerant = compute_metrics(tolerant_rows(pred_rows, TOLERANCE_RTT_FRAC))
    neighbor = neighborhood_metrics(pred_rows, TOLERANCE_RTT_FRAC)
    return {
        "rows": pred_rows,
        "overall": asdict(overall),
        "tolerant_overall": asdict(tolerant),
        "neighbor_overall": neighbor,
    }


def sigmoid(values):
    return 1.0 / (1.0 + np.exp(-np.clip(values, -40.0, 40.0)))


def fit_logreg(train_x: np.ndarray, train_y: np.ndarray, l2: float = 1e-2):
    def objective(theta):
        logits = train_x @ theta[:-1] + theta[-1]
        prob = sigmoid(logits)
        eps = 1e-12
        loss = -np.mean(train_y * np.log(prob + eps) + (1.0 - train_y) * np.log(1.0 - prob + eps))
        reg = 0.5 * l2 * np.sum(theta[:-1] ** 2)
        return loss + reg

    theta0 = np.zeros(train_x.shape[1] + 1, dtype=float)
    res = optimize.minimize(objective, theta0, method="L-BFGS-B")
    if not res.success:
        return theta0
    return res.x


def sanitize_matrix(matrix: np.ndarray) -> np.ndarray:
    matrix = np.asarray(matrix, dtype=float).copy()
    if matrix.ndim != 2:
        return matrix
    for col in range(matrix.shape[1]):
        column = matrix[:, col]
        finite = column[np.isfinite(column)]
        if len(finite) == 0:
            matrix[:, col] = 0.0
            continue
        fill_hi = float(np.max(finite) + max(np.std(finite), 1.0))
        fill_lo = float(np.min(finite) - max(np.std(finite), 1.0))
        column = np.nan_to_num(column, nan=np.median(finite), posinf=fill_hi, neginf=fill_lo)
        matrix[:, col] = column
    return matrix


def evaluate_logreg(rows: list[dict]) -> list[dict]:
    dataset_names = sorted({row["dataset"] for row in rows})
    feature_keys = [
        "recv_peak_close_sender",
        "recv_band_energy",
        "recv_band_peak_rel_total",
        "recv_shape_dist",
        "rtt_peak_close_sender",
        "rtt_band_energy",
        "rtt_band_peak_rel_total",
        "rtt_shape_dist",
        "peak_alignment",
    ]
    pred_rows = []
    for held_out in dataset_names:
        train = [row for row in rows if row["dataset"] != held_out]
        test = [row for row in rows if row["dataset"] == held_out]
        if not train or not test:
            continue
        train_x = sanitize_matrix(np.asarray([[row[key] for key in feature_keys] for row in train], dtype=float))
        test_x = sanitize_matrix(np.asarray([[row[key] for key in feature_keys] for row in test], dtype=float))
        train_y = np.asarray([1.0 if row["true_nonover"] else 0.0 for row in train], dtype=float)
        mean = np.mean(train_x, axis=0)
        std = np.std(train_x, axis=0)
        std = np.where(std > 1e-12, std, 1.0)
        train_x = (train_x - mean) / std
        test_x = (test_x - mean) / std
        theta = fit_logreg(train_x, train_y)
        probs = sigmoid(test_x @ theta[:-1] + theta[-1])
        for row, prob in zip(test, probs):
            pred_rows.append({**row, "pred_nonover": bool(prob >= 0.5), "prob_nonover": float(prob)})
    return pred_rows


def percentile_summary(rows: list[dict], key: str) -> dict[str, float | int | None]:
    values = np.asarray([row[key] for row in rows if math.isfinite(float(row[key]))], dtype=float)
    if len(values) == 0:
        return {"count": 0, "median": None, "p10": None, "p90": None}
    return {
        "count": int(len(values)),
        "median": float(np.median(values)),
        "p10": float(np.percentile(values, 10)),
        "p90": float(np.percentile(values, 90)),
    }


def main():
    global TOLERANCE_RTT_FRAC
    args = parse_args()
    TOLERANCE_RTT_FRAC = args.tolerant_match_rtt_frac
    paradigms = [item.strip() for item in args.paradigms.split(",") if item.strip()]
    datasets = [parse_dataset(spec) for spec in args.dataset]
    rows = []
    for dataset in datasets:
        rows.extend(collect_rows(dataset, args))
    if not rows:
        raise RuntimeError("No valid windows extracted")

    results = []
    peak_close_tols = parse_float_list(args.peak_close_tols)
    shape_dist_thrs = parse_float_list(args.shape_dist_thrs)
    energy_thrs = parse_float_list(args.energy_thrs)
    peakrel_thrs = parse_float_list(args.peakrel_thrs)
    joint_align_thrs = parse_float_list(args.joint_align_thrs)
    score_thrs = parse_float_list(args.score_thrs)
    soft_weights = parse_weight_pairs(args.soft_weights)
    rtt_upper_thrs = parse_float_list(args.rtt_upper_thrs)

    if "strict_dual_peak" in paradigms:
        for recv_close, rtt_close, recv_energy, rtt_energy in itertools.product(
            peak_close_tols,
            peak_close_tols,
            energy_thrs,
            energy_thrs,
        ):
            evaluated = evaluate_rule(
                rows,
                lambda row, rc=recv_close, tc=rtt_close, re=recv_energy, te=rtt_energy: (
                    row["recv_valid"]
                    and row["rtt_valid"]
                    and row["recv_peak_close_sender"] <= rc
                    and row["rtt_peak_close_sender"] <= tc
                    and row["recv_band_energy"] >= re
                    and row["rtt_band_energy"] >= te
                ),
            )
            results.append(
                {
                    "paradigm": "strict_dual_peak",
                    "params": {
                        "recv_peak_close": recv_close,
                        "rtt_peak_close": rtt_close,
                        "recv_energy": recv_energy,
                        "rtt_energy": rtt_energy,
                    },
                    **{k: v for k, v in evaluated.items() if k != "rows"},
                }
            )

    if "strict_dual_template" in paradigms:
        for recv_shape, rtt_shape, recv_peakrel, rtt_peakrel in itertools.product(
            shape_dist_thrs,
            shape_dist_thrs,
            peakrel_thrs,
            peakrel_thrs,
        ):
            evaluated = evaluate_rule(
                rows,
                lambda row, rs=recv_shape, ts=rtt_shape, rp=recv_peakrel, tp=rtt_peakrel: (
                    row["recv_valid"]
                    and row["rtt_valid"]
                    and row["recv_shape_dist"] <= rs
                    and row["rtt_shape_dist"] <= ts
                    and row["recv_band_peak_rel_total"] >= rp
                    and row["rtt_band_peak_rel_total"] >= tp
                ),
            )
            results.append(
                {
                    "paradigm": "strict_dual_template",
                    "params": {
                        "recv_shape": recv_shape,
                        "rtt_shape": rtt_shape,
                        "recv_peakrel": recv_peakrel,
                        "rtt_peakrel": rtt_peakrel,
                    },
                    **{k: v for k, v in evaluated.items() if k != "rows"},
                }
            )

    if "recv_template_rtt_peak" in paradigms:
        for recv_shape, recv_peakrel, rtt_close, rtt_energy in itertools.product(
            shape_dist_thrs,
            peakrel_thrs,
            peak_close_tols,
            energy_thrs,
        ):
            evaluated = evaluate_rule(
                rows,
                lambda row, rs=recv_shape, rp=recv_peakrel, tc=rtt_close, te=rtt_energy: (
                    row["recv_valid"]
                    and row["rtt_valid"]
                    and row["recv_shape_dist"] <= rs
                    and row["recv_band_peak_rel_total"] >= rp
                    and row["rtt_peak_close_sender"] <= tc
                    and row["rtt_band_energy"] >= te
                ),
            )
            results.append(
                {
                    "paradigm": "recv_template_rtt_peak",
                    "params": {
                        "recv_shape": recv_shape,
                        "recv_peakrel": recv_peakrel,
                        "rtt_peak_close": rtt_close,
                        "rtt_energy": rtt_energy,
                    },
                    **{k: v for k, v in evaluated.items() if k != "rows"},
                }
            )

    if "recv_peak_rtt_template" in paradigms:
        for recv_close, recv_energy, rtt_shape, rtt_peakrel in itertools.product(
            peak_close_tols,
            energy_thrs,
            shape_dist_thrs,
            peakrel_thrs,
        ):
            evaluated = evaluate_rule(
                rows,
                lambda row, rc=recv_close, re=recv_energy, ts=rtt_shape, tp=rtt_peakrel: (
                    row["recv_valid"]
                    and row["rtt_valid"]
                    and row["recv_peak_close_sender"] <= rc
                    and row["recv_band_energy"] >= re
                    and row["rtt_shape_dist"] <= ts
                    and row["rtt_band_peak_rel_total"] >= tp
                ),
            )
            results.append(
                {
                    "paradigm": "recv_peak_rtt_template",
                    "params": {
                        "recv_peak_close": recv_close,
                        "recv_energy": recv_energy,
                        "rtt_shape": rtt_shape,
                        "rtt_peakrel": rtt_peakrel,
                    },
                    **{k: v for k, v in evaluated.items() if k != "rows"},
                }
            )

    if "recv_template_align_rttupper" in paradigms:
        for recv_shape, recv_energy, align_thr, rtt_upper in itertools.product(
            shape_dist_thrs,
            energy_thrs,
            joint_align_thrs,
            rtt_upper_thrs,
        ):
            evaluated = evaluate_rule(
                rows,
                lambda row, rs=recv_shape, re=recv_energy, at=align_thr, ru=rtt_upper: (
                    row["recv_valid"]
                    and row["recv_shape_dist"] <= rs
                    and row["recv_band_energy"] >= re
                    and row["peak_alignment"] <= at
                    and row["rtt_peak_ratio"] <= ru
                ),
            )
            results.append(
                {
                    "paradigm": "recv_template_align_rttupper",
                    "params": {
                        "recv_shape": recv_shape,
                        "recv_energy": recv_energy,
                        "align_thr": align_thr,
                        "rtt_upper": rtt_upper,
                    },
                    **{k: v for k, v in evaluated.items() if k != "rows"},
                }
            )

    if "soft_score" in paradigms:
        for recv_w, rtt_w in soft_weights:
            for recv_close, rtt_close, recv_shape, rtt_shape, align_thr, score_thr in itertools.product(
                peak_close_tols,
                peak_close_tols,
                shape_dist_thrs,
                shape_dist_thrs,
                joint_align_thrs,
                score_thrs,
            ):
                evaluated = evaluate_rule(
                    rows,
                    lambda row,
                    rw=recv_w,
                    tw=rtt_w,
                    rc=recv_close,
                    tc=rtt_close,
                    rs=recv_shape,
                    ts=rtt_shape,
                    at=align_thr,
                    st=score_thr: (
                        (
                            rw
                            * 0.5
                            * (
                                score_from_feature(row["recv_peak_close_sender"], rc, False)
                                + score_from_feature(row["recv_shape_dist"], rs, False)
                            )
                            + tw
                            * 0.5
                            * (
                                score_from_feature(row["rtt_peak_close_sender"], tc, False)
                                + score_from_feature(row["rtt_shape_dist"], ts, False)
                            )
                            + 0.15 * score_from_feature(row["peak_alignment"], at, False)
                        )
                        >= st
                    ),
                )
                results.append(
                    {
                        "paradigm": "soft_score",
                        "params": {
                            "recv_weight": recv_w,
                            "rtt_weight": rtt_w,
                            "recv_peak_close": recv_close,
                            "rtt_peak_close": rtt_close,
                            "recv_shape": recv_shape,
                            "rtt_shape": rtt_shape,
                            "align_thr": align_thr,
                            "score_thr": score_thr,
                        },
                        **{k: v for k, v in evaluated.items() if k != "rows"},
                    }
                )

    if "soft_score_align_rttupper" in paradigms:
        for recv_w, rtt_w in soft_weights:
            for recv_shape, recv_energy, align_thr, rtt_upper, score_thr in itertools.product(
                shape_dist_thrs,
                energy_thrs,
                joint_align_thrs,
                rtt_upper_thrs,
                score_thrs,
            ):
                evaluated = evaluate_rule(
                    rows,
                    lambda row,
                    rw=recv_w,
                    tw=rtt_w,
                    rs=recv_shape,
                    re=recv_energy,
                    at=align_thr,
                    ru=rtt_upper,
                    st=score_thr: (
                        (
                            rw
                            * 0.5
                            * (
                                score_from_feature(row["recv_shape_dist"], rs, False)
                                + score_from_feature(row["recv_band_energy"], re, True)
                            )
                            + tw
                            * 0.5
                            * (
                                score_from_feature(row["peak_alignment"], at, False)
                                + score_from_feature(row["rtt_peak_ratio"], ru, False)
                            )
                        )
                        >= st
                    ),
                )
                results.append(
                    {
                        "paradigm": "soft_score_align_rttupper",
                        "params": {
                            "recv_weight": recv_w,
                            "rtt_weight": rtt_w,
                            "recv_shape": recv_shape,
                            "recv_energy": recv_energy,
                            "align_thr": align_thr,
                            "rtt_upper": rtt_upper,
                            "score_thr": score_thr,
                        },
                        **{k: v for k, v in evaluated.items() if k != "rows"},
                    }
                )

    if "soft_score_with_alignment" in paradigms:
        for recv_w, rtt_w in soft_weights:
            for recv_energy, rtt_energy, recv_peakrel, rtt_peakrel, align_thr, score_thr in itertools.product(
                energy_thrs,
                energy_thrs,
                peakrel_thrs,
                peakrel_thrs,
                joint_align_thrs,
                score_thrs,
            ):
                evaluated = evaluate_rule(
                    rows,
                    lambda row,
                    rw=recv_w,
                    tw=rtt_w,
                    re=recv_energy,
                    te=rtt_energy,
                    rp=recv_peakrel,
                    tp=rtt_peakrel,
                    at=align_thr,
                    st=score_thr: (
                        (
                            rw
                            * 0.5
                            * (
                                score_from_feature(row["recv_band_energy"], re, True)
                                + score_from_feature(row["recv_band_peak_rel_total"], rp, True)
                            )
                            + tw
                            * 0.5
                            * (
                                score_from_feature(row["rtt_band_energy"], te, True)
                                + score_from_feature(row["rtt_band_peak_rel_total"], tp, True)
                            )
                            + 0.20 * score_from_feature(row["peak_alignment"], at, False)
                        )
                        >= st
                    ),
                )
                results.append(
                    {
                        "paradigm": "soft_score_with_alignment",
                        "params": {
                            "recv_weight": recv_w,
                            "rtt_weight": rtt_w,
                            "recv_energy": recv_energy,
                            "rtt_energy": rtt_energy,
                            "recv_peakrel": recv_peakrel,
                            "rtt_peakrel": rtt_peakrel,
                            "align_thr": align_thr,
                            "score_thr": score_thr,
                        },
                        **{k: v for k, v in evaluated.items() if k != "rows"},
                    }
                )

    if "logreg" in paradigms:
        pred_rows = evaluate_logreg(rows)
        overall = compute_metrics(pred_rows)
        tolerant = compute_metrics(tolerant_rows(pred_rows, TOLERANCE_RTT_FRAC))
        neighbor = neighborhood_metrics(pred_rows, TOLERANCE_RTT_FRAC)
        results.append(
            {
                "paradigm": "logreg",
                "params": {"leave_one_dataset_out": True},
                "overall": asdict(overall),
                "tolerant_overall": asdict(tolerant),
                "neighbor_overall": neighbor,
            }
        )

    results.sort(
        key=lambda item: (
            item["neighbor_overall"]["neighbor_balanced_accuracy"],
            item["neighbor_overall"]["hit_nonover_rate"],
            item["tolerant_overall"]["balanced_accuracy"],
            item["overall"]["balanced_accuracy"],
        ),
        reverse=True,
    )

    top = results[: args.top_k]
    nonover_rows = [row for row in rows if row["true_nonover"]]
    over_rows = [row for row in rows if not row["true_nonover"]]

    print("Joint dual-frequency evaluation:")
    print(
        f"windows={len(rows)} nonover={len(nonover_rows)} over={len(over_rows)} "
        f"recv_trace={args.recv_trace_name} "
        f"rtt_signal={args.rtt_signal} "
        f"rate_window_mult={args.rate_window_mult:.3f} "
        f"rtt_window_mult={args.rtt_window_mult:.3f} "
        f"overlap={args.overlap:.2f} "
        f"gate=[{args.search_gate_low:.2f},{args.search_gate_high:.2f}] "
        f"nfft_mult={args.nfft_mult}"
    )
    print(
        "nonover summaries: "
        f"recv_peak_close_sender={percentile_summary(nonover_rows, 'recv_peak_close_sender')} "
        f"rtt_peak_close_sender={percentile_summary(nonover_rows, 'rtt_peak_close_sender')} "
        f"recv_shape_dist={percentile_summary(nonover_rows, 'recv_shape_dist')} "
        f"rtt_shape_dist={percentile_summary(nonover_rows, 'rtt_shape_dist')}"
    )
    for idx, item in enumerate(top, start=1):
        met = item["overall"]
        tol_met = item["tolerant_overall"]
        neighbor = item["neighbor_overall"]
        print(
            f"{idx:02d}. paradigm={item['paradigm']} "
            f"params={json.dumps(item['params'], sort_keys=True)} "
            f"nbr_bal_acc={neighbor['neighbor_balanced_accuracy']:.4f} "
            f"hit_nonover={neighbor['hit_nonover_rate']:.4f} "
            f"clean_over={neighbor['clean_over_rate']:.4f} "
            f"tol_bal_acc={tol_met['balanced_accuracy']:.4f} "
            f"bal_acc={met['balanced_accuracy']:.4f} "
            f"acc={met['accuracy']:.4f}"
        )

    if args.output:
        payload = {
            "config": {
                "rate_window_mult": args.rate_window_mult,
                "recv_trace_name": args.recv_trace_name,
                "rtt_signal": args.rtt_signal,
                "rtt_window_mult": args.rtt_window_mult,
                "overlap": args.overlap,
                "search_gate_low": args.search_gate_low,
                "search_gate_high": args.search_gate_high,
                "nfft_mult": args.nfft_mult,
                "uniform_step_ms": args.uniform_step_ms,
                "shape_bins": args.shape_bins,
                "template_local_peak_frac": args.template_local_peak_frac,
                "oracle_qfrac_threshold": args.oracle_qfrac_threshold,
                "causal_mode": args.causal_mode,
                "pre_bottleneck_prop_ms": args.pre_bottleneck_prop_ms,
                "causal_quantile_low": args.causal_quantile_low,
                "causal_quantile_high": args.causal_quantile_high,
                "causal_pad_ms": args.causal_pad_ms,
                "causal_pad_window_frac": args.causal_pad_window_frac,
                "tolerant_match_rtt_frac": args.tolerant_match_rtt_frac,
                "paradigms": paradigms,
            },
            "window_count": len(rows),
            "nonover_count": len(nonover_rows),
            "over_count": len(over_rows),
            "nonover_summaries": {
                "recv_peak_close_sender": percentile_summary(nonover_rows, "recv_peak_close_sender"),
                "rtt_peak_close_sender": percentile_summary(nonover_rows, "rtt_peak_close_sender"),
                "recv_shape_dist": percentile_summary(nonover_rows, "recv_shape_dist"),
                "rtt_shape_dist": percentile_summary(nonover_rows, "rtt_shape_dist"),
                "peak_alignment": percentile_summary(nonover_rows, "peak_alignment"),
            },
            "top": top,
        }
        with open(args.output, "w", encoding="utf-8") as handle:
            json.dump(payload, handle, indent=2)
        print(f"\nSaved summary to {args.output}")


if __name__ == "__main__":
    main()
