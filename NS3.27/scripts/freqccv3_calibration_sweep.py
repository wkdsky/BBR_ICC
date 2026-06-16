#!/usr/bin/env python3
"""
Sweep frequency-detection parameters for FreqCCv3 traces.

The script treats each ProbeBW interval as:
  previous UP phase -> reference frequency source
  following INT = CRUISE + REFILL -> main detection window

For each interval it:
  1. extracts the sender reference frequency from the previous UP trace
  2. performs open-band STFT on recvrate and smoothed RTT
  3. keeps peaks only inside a gated frequency band
  4. selects the peak nearest to the sender reference frequency
  5. compares recvrate / RTT detection against queue-derived state labels

Dataset spec format:
  name|trace_dir|prefix|queue_file|bottleneck_mbps

Example:
  --dataset "two_under|/tmp/cal/two_under|freqccv3_2flow_calibration_under_probe|freqccv3_2flow_calibration_bottleneck_queue.txt|20"
"""

from __future__ import annotations

import argparse
import itertools
import json
import math
import os
from dataclasses import dataclass

import numpy as np
from scipy import signal


DT = 0.001


@dataclass
class Dataset:
    name: str
    trace_dir: str
    prefix: str
    queue_file: str
    bottleneck_mbps: float


@dataclass
class IntervalResult:
    observed_state: str
    predicted_state: str
    recv_present: bool
    rtt_present: bool
    recv_error: float | None
    rtt_error: float | None
    queue_avg_frac: float
    queue_p95_frac: float


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--dataset",
        action="append",
        required=True,
        help="Dataset spec: name|trace_dir|prefix|queue_file|bottleneck_mbps",
    )
    parser.add_argument("--recv-window-mults", default="0.5,1.0,1.5,2.0")
    parser.add_argument("--rtt-window-mults", default="1.0,2.0,3.0,4.0")
    parser.add_argument("--overlaps", default="0.80,0.90")
    parser.add_argument("--gate-lows", default="0.5")
    parser.add_argument("--gate-highs", default="1.25,1.5,1.75")
    parser.add_argument("--energy-thresholds", default="0.10,0.15,0.20")
    parser.add_argument("--presence-threshold", type=float, default=0.20)
    parser.add_argument("--max-rel-error", type=float, default=0.50)
    parser.add_argument("--output", default="")
    return parser.parse_args()


def parse_float_list(text: str) -> list[float]:
    return [float(item) for item in text.split(",") if item.strip()]


def parse_dataset(spec: str) -> Dataset:
    parts = spec.split("|")
    if len(parts) != 5:
        raise ValueError(f"Bad dataset spec: {spec}")
    name, trace_dir, prefix, queue_file, bottleneck = parts
    queue_path = queue_file
    if not os.path.isabs(queue_path):
        queue_path = os.path.join(trace_dir, queue_file)
    return Dataset(
        name=name,
        trace_dir=trace_dir,
        prefix=prefix,
        queue_file=queue_path,
        bottleneck_mbps=float(bottleneck),
    )


def load_numeric_file(path: str, cols: tuple[int, ...]) -> np.ndarray:
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
    return np.asarray(rows, dtype=float)


def load_upphase_file(path: str):
    rows = []
    with open(path, "r", encoding="utf-8") as handle:
        for line in handle:
            if not line.strip() or line.startswith("#"):
                continue
            parts = line.split()
            if len(parts) < 7:
                continue
            try:
                start = float(parts[0])
                duration_ms = float(parts[1])
                freq_hz = float(parts[2])
                rows.append(
                    {
                        "start": start,
                        "end": start + duration_ms / 1000.0,
                        "duration": duration_ms / 1000.0,
                        "freq_hz": freq_hz,
                    }
                )
            except ValueError:
                continue
    return rows


def load_queue_trace(path: str) -> np.ndarray:
    rows = []
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
    return np.asarray(rows, dtype=float)


def resample_uniform(times: np.ndarray, values: np.ndarray, dt: float = DT):
    if len(times) < 4:
        return None, None
    t0 = times[0]
    t1 = times[-1]
    if t1 - t0 < 4 * dt:
        return None, None
    uniform_t = np.arange(t0, t1, dt)
    if len(uniform_t) < 8:
        return None, None
    uniform_v = np.interp(uniform_t, times, values)
    uniform_v = uniform_v - np.mean(uniform_v)
    return uniform_t, uniform_v


def compute_min_rtt_s(qdelay: np.ndarray, rtt: np.ndarray) -> float:
    if qdelay.size > 0 and qdelay.shape[1] >= 4:
        min_rtt_ms = np.min(qdelay[:, 3])
        if min_rtt_ms > 0:
            return min_rtt_ms / 1000.0
    if rtt.size > 0:
        min_rtt_ms = np.min(rtt[:, 2])
        if min_rtt_ms > 0:
            return min_rtt_ms / 1000.0
    raise RuntimeError("Cannot infer min RTT from traces")


def make_intervals(up_phases):
    intervals = []
    for idx in range(len(up_phases) - 1):
        cur = up_phases[idx]
        nxt = up_phases[idx + 1]
        if nxt["start"] <= cur["end"]:
            continue
        intervals.append(
            {
                "ref_freq_hz": cur["freq_hz"],
                "up_start": cur["start"],
                "up_end": cur["end"],
                "int_start": cur["end"],
                "int_end": nxt["start"],
            }
        )
    return intervals


def stft_nearest_peak(
    times: np.ndarray,
    values: np.ndarray,
    ref_freq_hz: float,
    min_rtt_s: float,
    window_mult: float,
    overlap: float,
    gate_low: float,
    gate_high: float,
    energy_threshold: float,
):
    uniform_t, uniform_v = resample_uniform(times, values)
    if uniform_t is None:
        return {
            "present": False,
            "present_ratio": 0.0,
            "median_error": None,
        }

    window_s = max(window_mult * min_rtt_s, 4.0 / max(ref_freq_hz, 1.0))
    nperseg = max(16, int(round(window_s / DT)))
    if nperseg >= len(uniform_v):
        return {
            "present": False,
            "present_ratio": 0.0,
            "median_error": None,
        }

    noverlap = int(nperseg * overlap)
    if noverlap >= nperseg:
        noverlap = nperseg - 1

    freqs, _, zxx = signal.stft(
        uniform_v,
        fs=1.0 / DT,
        window="hann",
        nperseg=nperseg,
        noverlap=noverlap,
        boundary=None,
        padded=False,
    )
    mags = np.abs(zxx)
    if mags.size == 0:
        return {
            "present": False,
            "present_ratio": 0.0,
            "median_error": None,
        }

    band_min = gate_low * ref_freq_hz
    band_max = gate_high * ref_freq_hz
    band_mask = (freqs >= band_min) & (freqs <= band_max)
    if not np.any(band_mask):
        return {
            "present": False,
            "present_ratio": 0.0,
            "median_error": None,
        }

    selected_errors = []
    present_columns = 0

    for col_idx in range(mags.shape[1]):
        spectrum = mags[:, col_idx]
        total_energy = np.sum(spectrum[1:])
        if total_energy <= 0.0:
            continue

        band_freqs = freqs[band_mask]
        band_spec = spectrum[band_mask]
        if len(band_spec) < 3:
            continue

        peak_indices, _ = signal.find_peaks(band_spec)
        if len(peak_indices) == 0:
            peak_indices = np.array([int(np.argmax(band_spec))])

        candidates = []
        for peak_idx in peak_indices:
            peak_mag = band_spec[peak_idx]
            peak_freq = band_freqs[peak_idx]
            energy_ratio = peak_mag / total_energy
            if energy_ratio < energy_threshold:
                continue
            rel_error = abs(peak_freq - ref_freq_hz) / max(ref_freq_hz, 1e-6)
            candidates.append((rel_error, -peak_mag, peak_freq))

        if not candidates:
            continue

        candidates.sort()
        rel_error, _, _ = candidates[0]
        selected_errors.append(rel_error)
        present_columns += 1

    present_ratio = present_columns / mags.shape[1] if mags.shape[1] > 0 else 0.0
    if not selected_errors:
        return {
            "present": False,
            "present_ratio": present_ratio,
            "median_error": None,
        }

    return {
        "present": True,
        "present_ratio": present_ratio,
        "median_error": float(np.median(selected_errors)),
    }


def interval_slice(data: np.ndarray, start: float, end: float) -> np.ndarray:
    if data.size == 0:
        return np.empty((0, data.shape[1] if data.ndim == 2 else 0))
    mask = (data[:, 0] >= start) & (data[:, 0] <= end)
    return data[mask]


def queue_state_from_interval(
    queue_data: np.ndarray,
    start: float,
    end: float,
    bdp_bytes: float,
) -> tuple[str, float, float]:
    interval = interval_slice(queue_data, start, end)
    if interval.size == 0 or bdp_bytes <= 0:
        return "unknown", 0.0, 0.0
    q_frac = interval[:, 1] / bdp_bytes
    avg_frac = float(np.mean(q_frac))
    p95_frac = float(np.percentile(q_frac, 95))

    if avg_frac < 0.05 and p95_frac < 0.15:
        return "UNDER", avg_frac, p95_frac
    if avg_frac > 0.45 or p95_frac > 0.90:
        return "OVER", avg_frac, p95_frac
    return "FULL", avg_frac, p95_frac


def predict_state(recv_present: bool, rtt_present: bool) -> str:
    if recv_present and not rtt_present:
        return "UNDER"
    if recv_present and rtt_present:
        return "FULL"
    if not recv_present and not rtt_present:
        return "OVER"
    return "UNCERTAIN"


def analyze_dataset(dataset: Dataset, config: dict) -> list[IntervalResult]:
    prefix = os.path.join(dataset.trace_dir, dataset.prefix)
    sendrate = load_numeric_file(prefix + "_sendrate.txt", (0, 1))
    recvrate = load_numeric_file(prefix + "_recvrate.txt", (0, 1))
    rtt = load_numeric_file(prefix + "_rtt.txt", (0, 3))
    qdelay = load_numeric_file(prefix + "_qdelay.txt", (0, 1, 2, 3))
    upphase = load_upphase_file(prefix + "_upphase.txt")
    queue_data = load_queue_trace(dataset.queue_file)

    min_rtt_s = compute_min_rtt_s(qdelay, rtt)
    bdp_bytes = dataset.bottleneck_mbps * 1000000.0 * min_rtt_s / 8.0
    intervals = make_intervals(upphase)
    results = []

    for interval in intervals:
        int_start = interval["int_start"]
        int_end = interval["int_end"]
        ref_freq = interval["ref_freq_hz"]
        if int_end - int_start < min_rtt_s:
            continue

        recv_slice = interval_slice(recvrate, int_start, int_end)
        rtt_slice = interval_slice(rtt, int_start, int_end)
        if recv_slice.size == 0 or rtt_slice.size == 0:
            continue

        recv_detection = stft_nearest_peak(
            recv_slice[:, 0],
            recv_slice[:, 1],
            ref_freq,
            min_rtt_s,
            config["recv_window_mult"],
            config["overlap"],
            config["gate_low"],
            config["gate_high"],
            config["energy_threshold"],
        )
        rtt_detection = stft_nearest_peak(
            rtt_slice[:, 0],
            rtt_slice[:, 1],
            ref_freq,
            min_rtt_s,
            config["rtt_window_mult"],
            config["overlap"],
            config["gate_low"],
            config["gate_high"],
            config["energy_threshold"],
        )

        recv_present = (
            recv_detection["present"]
            and recv_detection["present_ratio"] >= config["presence_threshold"]
            and (
                recv_detection["median_error"] is not None
                and recv_detection["median_error"] <= config["max_rel_error"]
            )
        )
        rtt_present = (
            rtt_detection["present"]
            and rtt_detection["present_ratio"] >= config["presence_threshold"]
            and (
                rtt_detection["median_error"] is not None
                and rtt_detection["median_error"] <= config["max_rel_error"]
            )
        )

        observed_state, avg_frac, p95_frac = queue_state_from_interval(
            queue_data, int_start, int_end, bdp_bytes
        )
        predicted_state = predict_state(recv_present, rtt_present)
        results.append(
            IntervalResult(
                observed_state=observed_state,
                predicted_state=predicted_state,
                recv_present=recv_present,
                rtt_present=rtt_present,
                recv_error=recv_detection["median_error"],
                rtt_error=rtt_detection["median_error"],
                queue_avg_frac=avg_frac,
                queue_p95_frac=p95_frac,
            )
        )

    return results


def score_results(all_results: dict[str, list[IntervalResult]]) -> dict:
    total = 0
    correct = 0
    uncertain = 0
    recv_present = 0
    rtt_present = 0
    recv_error = []
    rtt_error = []
    per_dataset = {}

    for name, results in all_results.items():
        ds_total = 0
        ds_correct = 0
        for item in results:
            if item.observed_state == "unknown":
                continue
            ds_total += 1
            total += 1
            if item.predicted_state == item.observed_state:
                ds_correct += 1
                correct += 1
            if item.predicted_state == "UNCERTAIN":
                uncertain += 1
            if item.recv_present:
                recv_present += 1
            if item.rtt_present:
                rtt_present += 1
            if item.recv_error is not None:
                recv_error.append(item.recv_error)
            if item.rtt_error is not None:
                rtt_error.append(item.rtt_error)

        per_dataset[name] = {
            "intervals": ds_total,
            "accuracy": (ds_correct / ds_total) if ds_total else 0.0,
        }

    accuracy = correct / total if total else 0.0
    recv_cov = recv_present / total if total else 0.0
    rtt_cov = rtt_present / total if total else 0.0
    recv_med_err = float(np.median(recv_error)) if recv_error else 1.0
    rtt_med_err = float(np.median(rtt_error)) if rtt_error else 1.0
    uncertainty = uncertain / total if total else 1.0

    score = (
        4.0 * accuracy
        + 0.6 * recv_cov
        + 0.4 * rtt_cov
        - 0.5 * recv_med_err
        - 0.3 * rtt_med_err
        - 0.4 * uncertainty
    )

    return {
        "score": score,
        "accuracy": accuracy,
        "recv_coverage": recv_cov,
        "rtt_coverage": rtt_cov,
        "recv_median_error": recv_med_err,
        "rtt_median_error": rtt_med_err,
        "uncertainty": uncertainty,
        "intervals": total,
        "per_dataset": per_dataset,
    }


def main():
    args = parse_args()
    datasets = [parse_dataset(spec) for spec in args.dataset]

    recv_window_mults = parse_float_list(args.recv_window_mults)
    rtt_window_mults = parse_float_list(args.rtt_window_mults)
    overlaps = parse_float_list(args.overlaps)
    gate_lows = parse_float_list(args.gate_lows)
    gate_highs = parse_float_list(args.gate_highs)
    energy_thresholds = parse_float_list(args.energy_thresholds)

    configs = []
    for recv_w, rtt_w, overlap, gate_low, gate_high, energy in itertools.product(
        recv_window_mults,
        rtt_window_mults,
        overlaps,
        gate_lows,
        gate_highs,
        energy_thresholds,
    ):
        if gate_high <= gate_low:
            continue
        config = {
            "recv_window_mult": recv_w,
            "rtt_window_mult": rtt_w,
            "overlap": overlap,
            "gate_low": gate_low,
            "gate_high": gate_high,
            "energy_threshold": energy,
            "presence_threshold": args.presence_threshold,
            "max_rel_error": args.max_rel_error,
        }
        all_results = {dataset.name: analyze_dataset(dataset, config) for dataset in datasets}
        summary = score_results(all_results)
        configs.append({"config": config, "summary": summary})

    configs.sort(key=lambda item: item["summary"]["score"], reverse=True)
    top = configs[:10]

    print("Top parameter configurations:")
    for idx, item in enumerate(top, start=1):
        print(
            f"{idx:02d}. score={item['summary']['score']:.4f} "
            f"acc={item['summary']['accuracy']:.3f} "
            f"recv_cov={item['summary']['recv_coverage']:.3f} "
            f"rtt_cov={item['summary']['rtt_coverage']:.3f} "
            f"recv_w={item['config']['recv_window_mult']} "
            f"rtt_w={item['config']['rtt_window_mult']} "
            f"overlap={item['config']['overlap']} "
            f"gate=[{item['config']['gate_low']},{item['config']['gate_high']}] "
            f"energy={item['config']['energy_threshold']}"
        )

    if args.output:
        payload = {
            "datasets": [dataset.__dict__ for dataset in datasets],
            "top": top,
        }
        with open(args.output, "w", encoding="utf-8") as handle:
            json.dump(payload, handle, indent=2)
        print(f"\nSaved summary to {args.output}")


if __name__ == "__main__":
    main()
