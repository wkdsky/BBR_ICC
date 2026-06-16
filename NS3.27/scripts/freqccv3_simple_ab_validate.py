#!/usr/bin/env python3
"""
Compare a simple FreqCCv3 A/B experiment:
  - run A: oscillation enabled in UP
  - run B: oscillation disabled

The goal is not to classify FULL/OVER online, but to verify causality:
  1. UP really injects a sender-side spectral component near f_ref.
  2. The following INT = CRUISE + REFILL interval shows stronger recv/srtt
     spectral response when the UP excitation exists.

This script intentionally follows the user's offline style more closely than
the current in-algorithm analyzer:
  - native ACK-driven samples
  - STFT on the measured sequence
  - Hann window
  - configurable zero-padding
  - gated-band energy and dominant frequency statistics
"""

from __future__ import annotations

import argparse
import json
import math
import os
from dataclasses import asdict, dataclass

import numpy as np


@dataclass
class SegmentSummary:
    dominant_ratio_median: float | None
    band_energy_median: float | None
    band_peak_rel_median: float | None
    sample_count_median: float | None
    segment_count: int
    nonempty_segment_count: int


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--on-dir", required=True)
    parser.add_argument("--off-dir", required=True)
    parser.add_argument("--prefix", required=True)
    parser.add_argument("--ref-freq-hz", type=float, default=60.0)
    parser.add_argument("--rate-window-mult", type=float, default=0.75)
    parser.add_argument("--rtt-window-mult", type=float, default=2.0)
    parser.add_argument("--overlap", type=float, default=0.9)
    parser.add_argument("--nfft-mult", type=int, default=4)
    parser.add_argument("--narrow-low", type=float, default=0.7)
    parser.add_argument("--narrow-high", type=float, default=1.3)
    parser.add_argument("--wide-low", type=float, default=0.4)
    parser.add_argument("--wide-high", type=float, default=1.3)
    parser.add_argument("--head-seconds", type=float, default=0.4)
    parser.add_argument("--output", default="")
    return parser.parse_args()


def load_numeric_file(path: str, cols: tuple[int, ...]) -> np.ndarray:
    if not os.path.exists(path):
        return np.empty((0, len(cols)), dtype=float)
    rows = []
    with open(path, "r", encoding="utf-8") as handle:
        for line in handle:
            if not line.strip() or line.startswith("#"):
                continue
            parts = line.split()
            if len(parts) <= max(cols):
                continue
            try:
                rows.append([float(parts[index]) for index in cols])
            except ValueError:
                continue
    return np.asarray(rows, dtype=float)


def load_up_phases(path: str):
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
            rows.append(
                {
                    "start": start,
                    "end": start + duration_ms / 1000.0,
                    "freq_hz": freq_hz,
                }
            )
    return rows


def build_int_intervals(up_phases):
    intervals = []
    for current, nxt in zip(up_phases[:-1], up_phases[1:]):
        if nxt["start"] <= current["end"]:
            continue
        intervals.append(
            {
                "start": current["end"],
                "end": nxt["start"],
                "ref_freq_hz": current["freq_hz"],
            }
        )
    return intervals


def slice_time_range(data: np.ndarray, start: float, end: float) -> np.ndarray:
    if data.size == 0:
        return np.empty((0, data.shape[1] if data.ndim == 2 else 0), dtype=float)
    mask = (data[:, 0] >= start) & (data[:, 0] <= end)
    return data[mask]


def normalize_signal(values: np.ndarray, eps: float = 1e-12) -> np.ndarray:
    return (values - np.mean(values)) / (np.std(values) + eps)


def fft_band_features(
    times: np.ndarray,
    values: np.ndarray,
    ref_freq_hz: float,
    gate_low: float,
    gate_high: float,
    nfft_mult: int,
):
    if len(times) < 4:
        return None

    dt = float(np.mean(np.diff(times)))
    if not math.isfinite(dt) or dt <= 0.0:
        return None

    normalized = normalize_signal(values)
    window = np.hanning(len(normalized))
    segment = (normalized - np.mean(normalized)) * window
    nfft = max(len(segment), int(len(segment) * nfft_mult))
    spectrum = np.abs(np.fft.rfft(segment, n=nfft))
    freqs = np.fft.rfftfreq(nfft, d=dt)

    total = float(np.sum(spectrum[1:]))
    if total <= 0.0:
        return None

    band_mask = (freqs >= gate_low * ref_freq_hz) & (freqs <= gate_high * ref_freq_hz)
    if not np.any(band_mask):
        return None

    band = spectrum[band_mask]
    if band.size == 0:
        return None

    dominant_freq = float(freqs[band_mask][int(np.argmax(band))])
    return {
        "dominant_ratio": dominant_freq / ref_freq_hz if ref_freq_hz > 0 else None,
        "band_energy": float(np.sum(band) / total),
        "band_peak_rel": float(np.max(band) / total),
        "sample_count": int(len(values)),
    }


def stft_band_features(
    times: np.ndarray,
    values: np.ndarray,
    ref_freq_hz: float,
    window_mult: float,
    overlap: float,
    gate_low: float,
    gate_high: float,
    nfft_mult: int,
    min_rtt_s: float,
):
    if len(times) < 4:
        return []

    dt = float(np.mean(np.diff(times)))
    if not math.isfinite(dt) or dt <= 0.0:
        return []

    win_s = window_mult * min_rtt_s
    win_len = max(4, int(win_s / dt))
    if len(values) < win_len:
        return []

    hop_len = max(1, int(win_len * (1.0 - overlap)))
    nfft = max(win_len, int(win_len * nfft_mult))
    freqs = np.fft.rfftfreq(nfft, d=dt)
    band_mask = (freqs >= gate_low * ref_freq_hz) & (freqs <= gate_high * ref_freq_hz)
    if not np.any(band_mask):
        return []

    normalized = normalize_signal(values)
    window = np.hanning(win_len)
    features = []

    for start in range(0, len(normalized) - win_len + 1, hop_len):
        segment = normalized[start : start + win_len]
        segment = (segment - np.mean(segment)) * window
        spectrum = np.abs(np.fft.rfft(segment, n=nfft))
        total = float(np.sum(spectrum[1:]))
        if total <= 0.0:
            continue
        band = spectrum[band_mask]
        if band.size == 0:
            continue
        dominant_freq = float(freqs[band_mask][int(np.argmax(band))])
        features.append(
            {
                "dominant_ratio": dominant_freq / ref_freq_hz if ref_freq_hz > 0 else None,
                "band_energy": float(np.sum(band) / total),
                "band_peak_rel": float(np.max(band) / total),
                "sample_count": int(win_len),
            }
        )
    return features


def summarize_features(feature_list) -> SegmentSummary:
    if not feature_list:
        return SegmentSummary(None, None, None, None, 0, 0)

    dominant = [item["dominant_ratio"] for item in feature_list if item["dominant_ratio"] is not None]
    band_energy = [item["band_energy"] for item in feature_list]
    band_peak_rel = [item["band_peak_rel"] for item in feature_list]
    sample_counts = [item["sample_count"] for item in feature_list]

    return SegmentSummary(
        dominant_ratio_median=float(np.median(dominant)) if dominant else None,
        band_energy_median=float(np.median(band_energy)) if band_energy else None,
        band_peak_rel_median=float(np.median(band_peak_rel)) if band_peak_rel else None,
        sample_count_median=float(np.median(sample_counts)) if sample_counts else None,
        segment_count=len(feature_list),
        nonempty_segment_count=len(feature_list),
    )


def summarize_per_segment(segment_feature_lists) -> SegmentSummary:
    flat = []
    nonempty = 0
    for features in segment_feature_lists:
        if features:
            nonempty += 1
            flat.extend(features)
    summary = summarize_features(flat)
    summary.segment_count = len(segment_feature_lists)
    summary.nonempty_segment_count = nonempty
    return summary


def summarize_internal_freq(path: str, peak_col: int):
    data = load_numeric_file(path, (0, 2, peak_col))
    if data.size == 0:
        return {
            "count": 0,
            "sender_peak_median_hz": None,
            "peak_median_hz": None,
        }
    return {
        "count": int(len(data)),
        "sender_peak_median_hz": float(np.median(data[:, 1])),
        "peak_median_hz": float(np.median(data[:, 2])),
    }


def analyze_run(run_dir: str, prefix: str, args):
    sendrate = load_numeric_file(os.path.join(run_dir, prefix + "_sendrate.txt"), (0, 1))
    recvrate = load_numeric_file(os.path.join(run_dir, prefix + "_recvrate.txt"), (0, 1))
    rtt = load_numeric_file(os.path.join(run_dir, prefix + "_rtt.txt"), (0, 3))
    up_phases = load_up_phases(os.path.join(run_dir, prefix + "_upphase.txt"))
    intervals = build_int_intervals(up_phases)

    min_rtt_s = None
    if rtt.size:
        min_rtt_s = float(np.min(rtt[:, 1])) / 1000.0
    if not min_rtt_s or min_rtt_s <= 0.0:
        min_rtt_s = 0.04

    up_narrow = []
    up_wide = []
    for up in up_phases:
        up_send = slice_time_range(sendrate, up["start"], up["end"])
        if len(up_send) < 4:
            continue
        narrow = fft_band_features(
            up_send[:, 0],
            up_send[:, 1],
            up["freq_hz"],
            args.narrow_low,
            args.narrow_high,
            args.nfft_mult,
        )
        wide = fft_band_features(
            up_send[:, 0],
            up_send[:, 1],
            up["freq_hz"],
            args.wide_low,
            args.wide_high,
            args.nfft_mult,
        )
        if narrow is not None:
            up_narrow.append(narrow)
        if wide is not None:
            up_wide.append(wide)

    recv_int_narrow = []
    recv_int_wide = []
    rtt_int_narrow = []
    rtt_int_wide = []
    recv_head_narrow = []
    recv_head_wide = []
    rtt_head_narrow = []
    rtt_head_wide = []
    for interval in intervals:
        interval_recv = slice_time_range(recvrate, interval["start"], interval["end"])
        interval_rtt = slice_time_range(rtt, interval["start"], interval["end"])
        head_end = min(interval["end"], interval["start"] + args.head_seconds)
        head_recv = slice_time_range(recvrate, interval["start"], head_end)
        head_rtt = slice_time_range(rtt, interval["start"], head_end)

        recv_int_narrow.append(
            stft_band_features(
                interval_recv[:, 0],
                interval_recv[:, 1],
                interval["ref_freq_hz"],
                args.rate_window_mult,
                args.overlap,
                args.narrow_low,
                args.narrow_high,
                args.nfft_mult,
                min_rtt_s,
            )
        )
        recv_head_narrow.append(
            stft_band_features(
                head_recv[:, 0],
                head_recv[:, 1],
                interval["ref_freq_hz"],
                args.rate_window_mult,
                args.overlap,
                args.narrow_low,
                args.narrow_high,
                args.nfft_mult,
                min_rtt_s,
            )
        )
        recv_int_wide.append(
            stft_band_features(
                interval_recv[:, 0],
                interval_recv[:, 1],
                interval["ref_freq_hz"],
                args.rate_window_mult,
                args.overlap,
                args.wide_low,
                args.wide_high,
                args.nfft_mult,
                min_rtt_s,
            )
        )
        recv_head_wide.append(
            stft_band_features(
                head_recv[:, 0],
                head_recv[:, 1],
                interval["ref_freq_hz"],
                args.rate_window_mult,
                args.overlap,
                args.wide_low,
                args.wide_high,
                args.nfft_mult,
                min_rtt_s,
            )
        )
        rtt_int_narrow.append(
            stft_band_features(
                interval_rtt[:, 0],
                interval_rtt[:, 1],
                interval["ref_freq_hz"],
                args.rtt_window_mult,
                args.overlap,
                args.narrow_low,
                args.narrow_high,
                args.nfft_mult,
                min_rtt_s,
            )
        )
        rtt_head_narrow.append(
            stft_band_features(
                head_rtt[:, 0],
                head_rtt[:, 1],
                interval["ref_freq_hz"],
                args.rtt_window_mult,
                args.overlap,
                args.narrow_low,
                args.narrow_high,
                args.nfft_mult,
                min_rtt_s,
            )
        )
        rtt_int_wide.append(
            stft_band_features(
                interval_rtt[:, 0],
                interval_rtt[:, 1],
                interval["ref_freq_hz"],
                args.rtt_window_mult,
                args.overlap,
                args.wide_low,
                args.wide_high,
                args.nfft_mult,
                min_rtt_s,
            )
        )
        rtt_head_wide.append(
            stft_band_features(
                head_rtt[:, 0],
                head_rtt[:, 1],
                interval["ref_freq_hz"],
                args.rtt_window_mult,
                args.overlap,
                args.wide_low,
                args.wide_high,
                args.nfft_mult,
                min_rtt_s,
            )
        )

    return {
        "up_count": len(up_phases),
        "interval_count": len(intervals),
        "min_rtt_ms": min_rtt_s * 1000.0,
        "up_send_narrow": asdict(summarize_features(up_narrow)),
        "up_send_wide": asdict(summarize_features(up_wide)),
        "int_recv_narrow": asdict(summarize_per_segment(recv_int_narrow)),
        "int_recv_wide": asdict(summarize_per_segment(recv_int_wide)),
        "int_rtt_narrow": asdict(summarize_per_segment(rtt_int_narrow)),
        "int_rtt_wide": asdict(summarize_per_segment(rtt_int_wide)),
        "head_recv_narrow": asdict(summarize_per_segment(recv_head_narrow)),
        "head_recv_wide": asdict(summarize_per_segment(recv_head_wide)),
        "head_rtt_narrow": asdict(summarize_per_segment(rtt_head_narrow)),
        "head_rtt_wide": asdict(summarize_per_segment(rtt_head_wide)),
        "internal_recvfreq": summarize_internal_freq(
            os.path.join(run_dir, prefix + "_recvfreq.txt"), 3
        ),
        "internal_rttfreq": summarize_internal_freq(
            os.path.join(run_dir, prefix + "_rttfreq.txt"), 3
        ),
    }


def ratio(on_value, off_value):
    if on_value is None or off_value is None:
        return None
    if abs(off_value) < 1e-12:
        return None
    return on_value / off_value


def build_contrast(on_data, off_data):
    return {
        "up_send_narrow_band_energy_ratio": ratio(
            on_data["up_send_narrow"]["band_energy_median"],
            off_data["up_send_narrow"]["band_energy_median"],
        ),
        "up_send_wide_band_energy_ratio": ratio(
            on_data["up_send_wide"]["band_energy_median"],
            off_data["up_send_wide"]["band_energy_median"],
        ),
        "int_recv_narrow_band_energy_ratio": ratio(
            on_data["int_recv_narrow"]["band_energy_median"],
            off_data["int_recv_narrow"]["band_energy_median"],
        ),
        "int_recv_wide_band_energy_ratio": ratio(
            on_data["int_recv_wide"]["band_energy_median"],
            off_data["int_recv_wide"]["band_energy_median"],
        ),
        "head_recv_narrow_band_energy_ratio": ratio(
            on_data["head_recv_narrow"]["band_energy_median"],
            off_data["head_recv_narrow"]["band_energy_median"],
        ),
        "head_recv_wide_band_energy_ratio": ratio(
            on_data["head_recv_wide"]["band_energy_median"],
            off_data["head_recv_wide"]["band_energy_median"],
        ),
        "int_rtt_narrow_band_energy_ratio": ratio(
            on_data["int_rtt_narrow"]["band_energy_median"],
            off_data["int_rtt_narrow"]["band_energy_median"],
        ),
        "int_rtt_wide_band_energy_ratio": ratio(
            on_data["int_rtt_wide"]["band_energy_median"],
            off_data["int_rtt_wide"]["band_energy_median"],
        ),
        "head_rtt_narrow_band_energy_ratio": ratio(
            on_data["head_rtt_narrow"]["band_energy_median"],
            off_data["head_rtt_narrow"]["band_energy_median"],
        ),
        "head_rtt_wide_band_energy_ratio": ratio(
            on_data["head_rtt_wide"]["band_energy_median"],
            off_data["head_rtt_wide"]["band_energy_median"],
        ),
        "internal_recvfreq_count_diff": (
            on_data["internal_recvfreq"]["count"] - off_data["internal_recvfreq"]["count"]
        ),
        "internal_rttfreq_count_diff": (
            on_data["internal_rttfreq"]["count"] - off_data["internal_rttfreq"]["count"]
        ),
    }


def main():
    args = parse_args()
    on_data = analyze_run(args.on_dir, args.prefix, args)
    off_data = analyze_run(args.off_dir, args.prefix, args)
    result = {
        "config": {
            "ref_freq_hz": args.ref_freq_hz,
            "rate_window_mult": args.rate_window_mult,
            "rtt_window_mult": args.rtt_window_mult,
            "overlap": args.overlap,
            "nfft_mult": args.nfft_mult,
            "narrow_gate": [args.narrow_low, args.narrow_high],
            "wide_gate": [args.wide_low, args.wide_high],
            "head_seconds": args.head_seconds,
        },
        "on": on_data,
        "off": off_data,
        "contrast": build_contrast(on_data, off_data),
    }

    text = json.dumps(result, indent=2, sort_keys=True)
    if args.output:
        with open(args.output, "w", encoding="utf-8") as handle:
            handle.write(text)
            handle.write("\n")
    print(text)


if __name__ == "__main__":
    main()
