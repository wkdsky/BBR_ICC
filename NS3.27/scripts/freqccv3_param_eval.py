#!/usr/bin/env python3
"""
Evaluate STFT detector parameters for FreqCCv3 on two classes of datasets:

1. oracle datasets:
   2-flow runs with offline queue-state labels used only for validation.
2. multi datasets:
   4/8/16-flow runs used to pick a configuration that stays stable across scales.

Dataset spec format:
  name|kind|trace_dir|prefix|num_flows|queue_file|bottleneck_mbps

kind:
  oracle-under | oracle-full | oracle-over | multi
"""

from __future__ import annotations

import argparse
import itertools
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
class DatasetStats:
    recv_ratio_med: float | None
    rtt_ratio_med: float | None
    recv_ratio_std: float | None
    rtt_ratio_std: float | None
    recv_cov: float
    rtt_cov: float
    recv_band_med: float | None
    rtt_band_med: float | None
    avg_qfrac_bdp: float | None
    p95_qfrac_bdp: float | None
    interval_count: int


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--dataset",
        action="append",
        required=True,
        help="Dataset spec: name|kind|trace_dir|prefix|num_flows|queue_file|bottleneck_mbps",
    )
    parser.add_argument("--rate-window-mults", default="0.5,0.75,1.0,1.25")
    parser.add_argument("--rtt-window-mults", default="1.0,1.5,2.0,2.5,3.0")
    parser.add_argument("--overlaps", default="0.8,0.9")
    parser.add_argument("--gate-lows", default="0.3,0.5,0.7")
    parser.add_argument("--gate-highs", default="1.1,1.2,1.3,1.5")
    parser.add_argument("--nfft-mults", default="2,4,8")
    parser.add_argument("--top-k", type=int, default=10)
    parser.add_argument("--quality-tol", type=float, default=0.35)
    parser.add_argument("--output", default="")
    return parser.parse_args()


def parse_float_list(text: str) -> list[float]:
    return [float(item) for item in text.split(",") if item.strip()]


def parse_int_list(text: str) -> list[int]:
    return [int(item) for item in text.split(",") if item.strip()]


def parse_dataset(spec: str) -> Dataset:
    parts = spec.split("|")
    if len(parts) != 7:
        raise ValueError(f"Bad dataset spec: {spec}")
    name, kind, trace_dir, prefix, num_flows, queue_file, bottleneck = parts
    queue_path = queue_file
    if not os.path.isabs(queue_path):
        queue_path = os.path.join(trace_dir, queue_file)
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
            rows.append(
                {
                    "start": start,
                    "end": start + duration_ms / 1000.0,
                    "freq_hz": freq_hz,
                }
            )
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
        {
            "ref_freq_hz": cur["freq_hz"],
            "int_start": cur["end"],
            "int_end": nxt["start"],
        }
        for cur, nxt in zip(up_phases[:-1], up_phases[1:])
        if nxt["start"] > cur["end"]
    ]


def normalize_signal(values: np.ndarray, eps: float = 1e-12) -> np.ndarray:
    return (values - np.mean(values)) / (np.std(values) + eps)


def dominant_and_band_stats(
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
        return np.asarray([], dtype=float), np.asarray([], dtype=float)

    dt = float(np.mean(np.diff(times)))
    if not math.isfinite(dt) or dt <= 0.0:
        return np.asarray([], dtype=float), np.asarray([], dtype=float)

    window_s = window_mult * min_rtt_s
    win_len = max(4, int(window_s / dt))
    if len(values) < win_len:
        return np.asarray([], dtype=float), np.asarray([], dtype=float)

    hop_len = max(1, int(win_len * (1.0 - overlap)))
    nfft = max(win_len, int(win_len * nfft_mult))
    freqs = np.fft.rfftfreq(nfft, d=dt)
    band_mask = (freqs >= gate_low * ref_freq_hz) & (freqs <= gate_high * ref_freq_hz)
    if not np.any(band_mask):
        return np.asarray([], dtype=float), np.asarray([], dtype=float)

    normalized = normalize_signal(values)
    window = np.hanning(win_len)
    dominant = []
    band_ratios = []

    for start in range(0, len(normalized) - win_len + 1, hop_len):
        segment = normalized[start : start + win_len]
        segment = (segment - np.mean(segment)) * window
        spectrum = np.abs(np.fft.rfft(segment, n=nfft))
        band = spectrum[band_mask]
        total = float(np.sum(spectrum[1:]))
        if band.size == 0 or total <= 0.0:
            continue
        dominant.append(float(freqs[band_mask][int(np.argmax(band))]))
        band_ratios.append(float(np.sum(band) / total))

    return np.asarray(dominant, dtype=float), np.asarray(band_ratios, dtype=float)


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


def median_or_none(values: list[float]) -> float | None:
    return float(np.median(values)) if values else None


def std_or_none(values: list[float]) -> float | None:
    return float(np.std(values)) if values else None


def mean_or_none(values: list[float]) -> float | None:
    return float(np.mean(values)) if values else None


def analyze_dataset(dataset: Dataset, config: dict) -> DatasetStats:
    queue_data = load_queue_trace(dataset.queue_file)

    recv_ratios = []
    rtt_ratios = []
    recv_bands = []
    rtt_bands = []
    avg_qfrac = []
    p95_qfrac = []
    recv_ok = 0
    rtt_ok = 0
    interval_count = 0

    for flow_index in range(1, dataset.num_flows + 1):
        paths = flow_paths(dataset, flow_index)
        recvrate = load_numeric_file(paths["recvrate"], (0, 1))
        rtt = load_numeric_file(paths["rtt"], (0, 3))
        qdelay = load_numeric_file(paths["qdelay"], (0, 1, 2, 3))
        upphase = load_upphase_file(paths["upphase"])
        min_rtt_s = compute_min_rtt_s(qdelay, rtt)
        bdp_bytes = dataset.bottleneck_mbps * 1_000_000.0 * min_rtt_s / 8.0

        for interval in build_intervals(upphase):
            int_start = interval["int_start"]
            int_end = interval["int_end"]
            ref_freq = interval["ref_freq_hz"]

            recv_slice = interval_slice(recvrate, int_start, int_end)
            rtt_slice = interval_slice(rtt, int_start, int_end)
            if recv_slice.size == 0 or rtt_slice.size == 0:
                continue

            interval_count += 1

            if queue_data.size > 0 and bdp_bytes > 0:
                queue_slice = extend_queue_window(queue_data, int_start, int_end)
                if len(queue_slice):
                    qfrac = queue_slice[:, 1] / bdp_bytes
                    avg = weighted_average(queue_slice[:, 0], qfrac)
                    if avg is not None:
                        avg_qfrac.append(avg)
                    p95_qfrac.append(float(np.percentile(qfrac, 95)))

            recv_dom, recv_band = dominant_and_band_stats(
                recv_slice[:, 0],
                recv_slice[:, 1],
                ref_freq,
                config["rate_window_mult"],
                config["overlap"],
                config["gate_low"],
                config["gate_high"],
                config["nfft_mult"],
                min_rtt_s,
            )
            rtt_dom, rtt_band = dominant_and_band_stats(
                rtt_slice[:, 0],
                rtt_slice[:, 1],
                ref_freq,
                config["rtt_window_mult"],
                config["overlap"],
                config["gate_low"],
                config["gate_high"],
                config["nfft_mult"],
                min_rtt_s,
            )

            if len(recv_dom):
                recv_ok += 1
                recv_ratios.append(float(np.median(recv_dom / ref_freq)))
                recv_bands.append(float(np.median(recv_band)))
            if len(rtt_dom):
                rtt_ok += 1
                rtt_ratios.append(float(np.median(rtt_dom / ref_freq)))
                rtt_bands.append(float(np.median(rtt_band)))

    recv_cov = recv_ok / interval_count if interval_count else 0.0
    rtt_cov = rtt_ok / interval_count if interval_count else 0.0

    return DatasetStats(
        recv_ratio_med=median_or_none(recv_ratios),
        rtt_ratio_med=median_or_none(rtt_ratios),
        recv_ratio_std=std_or_none(recv_ratios),
        rtt_ratio_std=std_or_none(rtt_ratios),
        recv_cov=recv_cov,
        rtt_cov=rtt_cov,
        recv_band_med=median_or_none(recv_bands),
        rtt_band_med=median_or_none(rtt_bands),
        avg_qfrac_bdp=mean_or_none(avg_qfrac),
        p95_qfrac_bdp=mean_or_none(p95_qfrac),
        interval_count=interval_count,
    )


def signal_quality(ratio_med: float | None, cov: float, tol: float) -> float:
    if ratio_med is None:
        return 0.0
    align = max(0.0, 1.0 - abs(ratio_med - 1.0) / tol)
    return cov * align


def summarize_config(per_dataset: dict[str, DatasetStats], datasets: list[Dataset], tol: float) -> dict[str, float]:
    under = [d for d in datasets if d.kind == "oracle-under"]
    full = [d for d in datasets if d.kind == "oracle-full"]
    over = [d for d in datasets if d.kind == "oracle-over"]
    multi = [d for d in datasets if d.kind == "multi"]

    def avg_quality(items, attr):
        values = []
        for item in items:
            stats = per_dataset[item.name]
            values.append(
                signal_quality(
                    getattr(stats, f"{attr}_ratio_med"),
                    getattr(stats, f"{attr}_cov"),
                    tol,
                )
            )
        return float(np.mean(values)) if values else 0.0

    under_recv = avg_quality(under, "recv")
    under_rtt = avg_quality(under, "rtt")
    full_recv = avg_quality(full, "recv")
    full_rtt = avg_quality(full, "rtt")
    over_recv = avg_quality(over, "recv")
    over_rtt = avg_quality(over, "rtt")

    multi_recv_vals = []
    multi_rtt_vals = []
    multi_recv_meds = []
    multi_rtt_meds = []
    multi_recv_covs = []
    multi_rtt_covs = []
    for item in multi:
        stats = per_dataset[item.name]
        multi_recv_vals.append(signal_quality(stats.recv_ratio_med, stats.recv_cov, tol))
        multi_rtt_vals.append(signal_quality(stats.rtt_ratio_med, stats.rtt_cov, tol))
        if stats.recv_ratio_med is not None:
            multi_recv_meds.append(stats.recv_ratio_med)
        if stats.rtt_ratio_med is not None:
            multi_rtt_meds.append(stats.rtt_ratio_med)
        multi_recv_covs.append(stats.recv_cov)
        multi_rtt_covs.append(stats.rtt_cov)

    multi_recv = float(np.mean(multi_recv_vals)) if multi_recv_vals else 0.0
    multi_rtt = float(np.mean(multi_rtt_vals)) if multi_rtt_vals else 0.0
    multi_recv_std = float(np.std(multi_recv_meds)) if multi_recv_meds else 1.0
    multi_rtt_std = float(np.std(multi_rtt_meds)) if multi_rtt_meds else 1.0
    multi_recv_cov = float(np.mean(multi_recv_covs)) if multi_recv_covs else 0.0
    multi_rtt_cov = float(np.mean(multi_rtt_covs)) if multi_rtt_covs else 0.0

    oracle_selectivity = (
        0.7 * full_recv
        + 1.0 * full_rtt
        + 0.5 * under_recv
        - 0.6 * under_rtt
        - 0.8 * over_recv
        - 1.0 * over_rtt
    )
    multi_consistency = (
        0.6 * multi_recv
        + 0.6 * multi_rtt
        + 0.4 * multi_recv_cov
        + 0.4 * multi_rtt_cov
        - 0.8 * multi_recv_std
        - 0.8 * multi_rtt_std
    )

    score = oracle_selectivity + multi_consistency
    return {
        "score": score,
        "oracle_selectivity": oracle_selectivity,
        "multi_consistency": multi_consistency,
        "under_recv_quality": under_recv,
        "under_rtt_quality": under_rtt,
        "full_recv_quality": full_recv,
        "full_rtt_quality": full_rtt,
        "over_recv_quality": over_recv,
        "over_rtt_quality": over_rtt,
        "multi_recv_quality": multi_recv,
        "multi_rtt_quality": multi_rtt,
        "multi_recv_cov": multi_recv_cov,
        "multi_rtt_cov": multi_rtt_cov,
        "multi_recv_std": multi_recv_std,
        "multi_rtt_std": multi_rtt_std,
    }


def main():
    args = parse_args()
    datasets = [parse_dataset(spec) for spec in args.dataset]

    configs = []
    for rate_window_mult, rtt_window_mult, overlap, gate_low, gate_high, nfft_mult in itertools.product(
        parse_float_list(args.rate_window_mults),
        parse_float_list(args.rtt_window_mults),
        parse_float_list(args.overlaps),
        parse_float_list(args.gate_lows),
        parse_float_list(args.gate_highs),
        parse_int_list(args.nfft_mults),
    ):
        if gate_high <= gate_low:
            continue
        config = {
            "rate_window_mult": rate_window_mult,
            "rtt_window_mult": rtt_window_mult,
            "overlap": overlap,
            "gate_low": gate_low,
            "gate_high": gate_high,
            "nfft_mult": nfft_mult,
        }
        per_dataset = {dataset.name: analyze_dataset(dataset, config) for dataset in datasets}
        summary = summarize_config(per_dataset, datasets, args.quality_tol)
        configs.append(
            {
                "config": config,
                "summary": summary,
                "per_dataset": {name: asdict(stats) for name, stats in per_dataset.items()},
            }
        )

    configs.sort(
        key=lambda item: (
            item["summary"]["score"],
            item["summary"]["oracle_selectivity"],
            item["summary"]["multi_consistency"],
        ),
        reverse=True,
    )
    top = configs[: args.top_k]

    print("Top parameter configurations:")
    for idx, item in enumerate(top, start=1):
        cfg = item["config"]
        summ = item["summary"]
        print(
            f"{idx:02d}. score={summ['score']:.4f} "
            f"oracle={summ['oracle_selectivity']:.4f} "
            f"multi={summ['multi_consistency']:.4f} "
            f"full=({summ['full_recv_quality']:.3f},{summ['full_rtt_quality']:.3f}) "
            f"under=({summ['under_recv_quality']:.3f},{summ['under_rtt_quality']:.3f}) "
            f"over=({summ['over_recv_quality']:.3f},{summ['over_rtt_quality']:.3f}) "
            f"multi_cov=({summ['multi_recv_cov']:.3f},{summ['multi_rtt_cov']:.3f}) "
            f"multi_std=({summ['multi_recv_std']:.4f},{summ['multi_rtt_std']:.4f}) "
            f"rate_w={cfg['rate_window_mult']} "
            f"rtt_w={cfg['rtt_window_mult']} "
            f"overlap={cfg['overlap']} "
            f"gate=[{cfg['gate_low']},{cfg['gate_high']}] "
            f"nfft={cfg['nfft_mult']}x"
        )

    if args.output:
        payload = {
            "datasets": [asdict(dataset) for dataset in datasets],
            "top": top,
        }
        with open(args.output, "w", encoding="utf-8") as handle:
            json.dump(payload, handle, indent=2)
        print(f"\nSaved summary to {args.output}")


if __name__ == "__main__":
    main()
