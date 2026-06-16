#!/usr/bin/env python3
"""
Diagnose why FreqCCv3 recvrate dominant frequency is often lower than the
sender-side probe frequency.

The script combines three checks:
  1. trace-pipeline inspection on a concrete dataset
  2. whole-interval and short-window dominant-frequency measurements
  3. a synthetic RTT-round peak-hold experiment that mimics BandwidthLatest()

This is meant to answer a specific question:
  is the low dominant frequency mainly caused by the network response itself,
  by the ACK-driven BandwidthLatest() sampling pipeline, or by short-window STFT?
"""

from __future__ import annotations

import argparse
import json
import math
import os

import numpy as np


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--trace-dir", required=True)
    parser.add_argument("--prefix", required=True)
    parser.add_argument("--output", default="")
    return parser.parse_args()


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


def load_upphase(path: str) -> list[dict[str, float]]:
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
                    "duration_ms": duration_ms,
                    "freq_hz": freq_hz,
                }
            )
    return rows


def build_int_intervals(up_phases: list[dict[str, float]]) -> list[dict[str, float]]:
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


def slice_range(data: np.ndarray, start: float, end: float) -> np.ndarray:
    if data.size == 0:
        return np.empty((0, data.shape[1] if data.ndim == 2 else 0), dtype=float)
    mask = (data[:, 0] >= start) & (data[:, 0] <= end)
    return data[mask]


def normalize(values: np.ndarray, eps: float = 1e-12) -> np.ndarray:
    return (values - np.mean(values)) / (np.std(values) + eps)


def cadence_stats(times: np.ndarray) -> dict[str, float | int | None]:
    if len(times) < 2:
        return {"count": int(len(times)), "dt_ms_mean": None, "dt_ms_median": None}
    deltas = np.diff(times) * 1000.0
    return {
        "count": int(len(times)),
        "dt_ms_mean": float(np.mean(deltas)),
        "dt_ms_median": float(np.median(deltas)),
        "dt_ms_p10": float(np.percentile(deltas, 10)),
        "dt_ms_p90": float(np.percentile(deltas, 90)),
    }


def whole_interval_dominant_ratio(
    times: np.ndarray,
    values: np.ndarray,
    ref_freq_hz: float,
    gate_low: float = 0.7,
    gate_high: float = 1.3,
) -> float | None:
    if len(times) < 8:
        return None
    dt = float(np.mean(np.diff(times)))
    if not math.isfinite(dt) or dt <= 0.0:
        return None
    vals = normalize(values)
    spectrum = np.abs(np.fft.rfft(vals * np.hanning(len(vals))))
    freqs = np.fft.rfftfreq(len(vals), d=dt)
    mask = (freqs >= gate_low * ref_freq_hz) & (freqs <= gate_high * ref_freq_hz)
    if not np.any(mask):
        return None
    return float(freqs[mask][int(np.argmax(spectrum[mask]))] / ref_freq_hz)


def short_window_dominant_ratios(
    times: np.ndarray,
    values: np.ndarray,
    ref_freq_hz: float,
    min_rtt_s: float,
    window_mult: float,
    nfft_mult: int,
    overlap: float = 0.9,
    gate_low: float = 0.7,
    gate_high: float = 1.3,
) -> list[float]:
    if len(times) < 8:
        return []
    dt = float(np.mean(np.diff(times)))
    if not math.isfinite(dt) or dt <= 0.0:
        return []
    win_len = max(4, int((window_mult * min_rtt_s) / dt))
    if len(values) < win_len:
        return []
    hop_len = max(1, int(win_len * (1.0 - overlap)))
    nfft = max(win_len, int(win_len * nfft_mult))
    freqs = np.fft.rfftfreq(nfft, d=dt)
    mask = (freqs >= gate_low * ref_freq_hz) & (freqs <= gate_high * ref_freq_hz)
    if not np.any(mask):
        return []
    vals = normalize(values)
    out = []
    for start in range(0, len(vals) - win_len + 1, hop_len):
        seg = vals[start : start + win_len]
        seg = (seg - np.mean(seg)) * np.hanning(win_len)
        spectrum = np.abs(np.fft.rfft(seg, n=nfft))[mask]
        out.append(float(freqs[mask][int(np.argmax(spectrum))] / ref_freq_hz))
    return out


def top_peaks(
    times: np.ndarray,
    values: np.ndarray,
    low_hz: float,
    high_hz: float,
    topk: int = 8,
) -> list[dict[str, float]]:
    if len(times) < 8:
        return []
    dt = float(np.mean(np.diff(times)))
    vals = normalize(values)
    spectrum = np.abs(np.fft.rfft(vals * np.hanning(len(vals))))
    freqs = np.fft.rfftfreq(len(vals), d=dt)
    mask = (freqs >= low_hz) & (freqs <= high_hz)
    masked_freqs = freqs[mask]
    masked_spectrum = spectrum[mask]
    if masked_spectrum.size == 0:
        return []
    order = np.argsort(masked_spectrum)[::-1][:topk]
    return [
        {
            "freq_hz": float(masked_freqs[idx]),
            "amplitude": float(masked_spectrum[idx]),
        }
        for idx in order
    ]


def synthetic_peakhold_diagnosis(
    dt_s: float,
    min_rtt_s: float,
    ref_freq_hz: float,
    duration_s: float,
) -> dict[str, object]:
    t = np.arange(0.0, duration_s, dt_s)
    pure = np.sin(2.0 * np.pi * ref_freq_hz * t)
    round_index = (t / min_rtt_s).astype(int)
    peakhold = np.empty_like(pure)
    for rid in np.unique(round_index):
        mask = round_index == rid
        peakhold[mask] = np.maximum.accumulate(pure[mask])

    def dominant_ratio(values: np.ndarray) -> float:
        vals = normalize(values)
        spectrum = np.abs(np.fft.rfft(vals * np.hanning(len(vals))))
        freqs = np.fft.rfftfreq(len(vals), d=dt_s)
        mask = (freqs >= 0.5 * ref_freq_hz) & (freqs <= 1.5 * ref_freq_hz)
        return float(freqs[mask][int(np.argmax(spectrum[mask]))] / ref_freq_hz)

    def short_summary(values: np.ndarray) -> dict[str, dict[str, float]]:
        out = {}
        for window_mult, nfft_mult in [(0.75, 4), (1.5, 16), (2.5, 16)]:
            ratios = short_window_dominant_ratios(
                t,
                values,
                ref_freq_hz,
                min_rtt_s,
                window_mult,
                nfft_mult,
            )
            out[f"w{window_mult}_n{nfft_mult}"] = {
                "median": float(np.median(ratios)),
                "p10": float(np.percentile(ratios, 10)),
                "p90": float(np.percentile(ratios, 90)),
            }
        return out

    return {
        "pure_whole_interval_ratio": dominant_ratio(pure),
        "peakhold_whole_interval_ratio": dominant_ratio(peakhold),
        "pure_top_peaks": top_peaks(t, pure, 30.0, 90.0),
        "peakhold_top_peaks": top_peaks(t, peakhold, 30.0, 90.0),
        "pure_short_windows": short_summary(pure),
        "peakhold_short_windows": short_summary(peakhold),
    }


def summarize_ratio_list(values: list[float]) -> dict[str, float | int | None]:
    if not values:
        return {"count": 0, "median": None, "p10": None, "p90": None}
    arr = np.asarray(values, dtype=float)
    return {
        "count": int(len(arr)),
        "median": float(np.median(arr)),
        "p10": float(np.percentile(arr, 10)),
        "p90": float(np.percentile(arr, 90)),
    }


def main():
    args = parse_args()
    base = os.path.join(args.trace_dir, args.prefix)
    sendrate = load_numeric_file(base + "_sendrate.txt", (0, 1))
    recvrate = load_numeric_file(base + "_recvrate.txt", (0, 1))
    bw = load_numeric_file(base + "_bw.txt", (0, 1))
    qdelay = load_numeric_file(base + "_qdelay.txt", (0, 1, 2, 3))
    upphase = load_upphase(base + "_upphase.txt")
    int_intervals = build_int_intervals(upphase)
    min_rtt_s = float(np.min(qdelay[:, 3]) / 1000.0)

    whole_int_ratios = []
    first_interval_summary = None
    for interval_index, interval in enumerate(int_intervals):
        recv_slice = slice_range(recvrate, interval["start"], interval["end"])
        if len(recv_slice) < 8:
            continue
        ratio = whole_interval_dominant_ratio(
            recv_slice[:, 0],
            recv_slice[:, 1],
            interval["ref_freq_hz"],
        )
        if ratio is not None:
            whole_int_ratios.append(ratio)
        if first_interval_summary is None:
            bw_slice = slice_range(bw, interval["start"], interval["end"])
            first_interval_summary = {
                "interval_index": interval_index,
                "duration_s": float(interval["end"] - interval["start"]),
                "ref_freq_hz": float(interval["ref_freq_hz"]),
                "recv_top_peaks": top_peaks(recv_slice[:, 0], recv_slice[:, 1], 30.0, 90.0),
                "bw_top_peaks": top_peaks(bw_slice[:, 0], bw_slice[:, 1], 30.0, 90.0),
                "recv_short_windows": {
                    "w0.75_n4": summarize_ratio_list(
                        short_window_dominant_ratios(
                            recv_slice[:, 0], recv_slice[:, 1], interval["ref_freq_hz"], min_rtt_s, 0.75, 4
                        )
                    ),
                    "w1.5_n16": summarize_ratio_list(
                        short_window_dominant_ratios(
                            recv_slice[:, 0], recv_slice[:, 1], interval["ref_freq_hz"], min_rtt_s, 1.5, 16
                        )
                    ),
                    "w2.5_n16": summarize_ratio_list(
                        short_window_dominant_ratios(
                            recv_slice[:, 0], recv_slice[:, 1], interval["ref_freq_hz"], min_rtt_s, 2.5, 16
                        )
                    ),
                },
            }

    first_up = upphase[0]
    send_up_slice = slice_range(sendrate, first_up["start"], first_up["end"])
    sender_summary = {
        "first_up_duration_ms": float(first_up["duration_ms"]),
        "first_up_ref_freq_hz": float(first_up["freq_hz"]),
        "send_top_peaks": top_peaks(send_up_slice[:, 0], send_up_slice[:, 1], 30.0, 90.0),
        "whole_up_ratio": whole_interval_dominant_ratio(
            send_up_slice[:, 0],
            send_up_slice[:, 1],
            first_up["freq_hz"],
        ),
    }

    dt_recv_s = float(np.mean(np.diff(recvrate[:, 0])))
    dt_send_s = float(np.mean(np.diff(sendrate[:, 0])))
    synthetic = synthetic_peakhold_diagnosis(
        dt_s=dt_recv_s,
        min_rtt_s=min_rtt_s,
        ref_freq_hz=float(first_up["freq_hz"]),
        duration_s=float(first_interval_summary["duration_s"]) if first_interval_summary else 2.5,
    )

    payload = {
        "trace_dir": args.trace_dir,
        "prefix": args.prefix,
        "min_rtt_s": min_rtt_s,
        "cadence": {
            "sendrate": cadence_stats(sendrate[:, 0]),
            "recvrate": cadence_stats(recvrate[:, 0]),
            "bw": cadence_stats(bw[:, 0]),
        },
        "sender_summary": sender_summary,
        "whole_int_recv_ratio": summarize_ratio_list(whole_int_ratios),
        "first_interval_summary": first_interval_summary,
        "synthetic_peakhold": synthetic,
        "conclusion": {
            "trace_signal_definition": "recvrate trace is ACK-driven BandwidthLatest(), not raw receiver goodput",
            "bbr2_update_shape": "BandwidthLatest only increases within a round and is reset from sample_max_bandwidth at round end",
            "expected_effect": "RTT-round peak-hold of a 60Hz probe creates strong sub-reference peaks near 0.83x and below",
            "stft_effect": "short windows below about 2 cycles worsen the bias and often collapse to the band low edge",
        },
    }

    print("Freq-bias diagnosis:")
    print(
        f"min_rtt={min_rtt_s:.6f}s "
        f"send_dt_mean_ms={payload['cadence']['sendrate']['dt_ms_mean']:.4f} "
        f"recv_dt_mean_ms={payload['cadence']['recvrate']['dt_ms_mean']:.4f}"
    )
    print(
        f"sender_up_ratio={sender_summary['whole_up_ratio']:.4f} "
        f"recv_int_ratio_med={payload['whole_int_recv_ratio']['median']:.4f}"
    )
    print(
        f"synthetic_pure_ratio={synthetic['pure_whole_interval_ratio']:.4f} "
        f"synthetic_peakhold_ratio={synthetic['peakhold_whole_interval_ratio']:.4f}"
    )
    print("first_int_recv_peaks:", ", ".join(f"{item['freq_hz']:.2f}" for item in first_interval_summary["recv_top_peaks"][:5]))
    print("synthetic_peakhold_peaks:", ", ".join(f"{item['freq_hz']:.2f}" for item in synthetic["peakhold_top_peaks"][:5]))

    if args.output:
        with open(args.output, "w", encoding="utf-8") as handle:
            json.dump(payload, handle, indent=2)
        print(f"saved_json={args.output}")


if __name__ == "__main__":
    main()
