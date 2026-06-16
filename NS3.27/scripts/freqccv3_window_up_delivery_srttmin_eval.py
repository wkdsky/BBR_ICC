#!/usr/bin/env python3
"""
Evaluate a fixed UP-phase modulation rule with:

  1. delivery-rate frequency gate
  2. minimum-SRTT threshold gate

Window rule:
  predict NONOVER iff
    min(srtt in current window) <
      min_rtt + srtt_threshold_frac * (recent2_max_rtt - min_rtt)
    and
      freq_ratio_low < f_delivery / f_send < freq_ratio_high

Otherwise the window is left undecided and is not counted as a NONOVER
prediction.

Primary metric:
  positive_precision = TP / predicted_nonover_total

This matches the user's requested accuracy notion: only windows predicted as
NONOVER can hurt accuracy, and they hurt it only when the matched oracle label
is actually OVER.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
from dataclasses import asdict, dataclass

import numpy as np


SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

import freqccv3_window_dualfreq_eval as dual  # noqa: E402


@dataclass
class PositiveOnlyMetrics:
    windows: int
    pred_nonover_total: int
    abstain_total: int
    true_nonover_total: int
    true_over_total: int
    tp_nonover: int
    fp_over: int
    positive_precision: float
    false_positive_rate: float
    prediction_rate: float
    overload_leak_rate: float


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--dataset",
        action="append",
        required=True,
        help="Dataset spec: name|kind|trace_dir|prefix|num_flows|queue_file|bottleneck_mbps",
    )
    parser.add_argument(
        "--recv-trace-name",
        choices=["recvrate", "recvrate_raw"],
        default="recvrate_raw",
        help="Delivery-rate-side trace. recvrate_raw is the literal delivery-rate interpretation.",
    )
    parser.add_argument(
        "--rtt-signal",
        choices=["rtt", "srtt"],
        default="srtt",
    )
    parser.add_argument("--rate-window-mult", type=float, default=0.75)
    parser.add_argument("--rtt-window-mult", type=float, default=1.5)
    parser.add_argument("--overlap", type=float, default=0.9)
    parser.add_argument("--search-gate-low", type=float, default=0.50)
    parser.add_argument("--search-gate-high", type=float, default=1.80)
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

    parser.add_argument("--freq-ratio-low", type=float, default=0.75)
    parser.add_argument("--freq-ratio-high", type=float, default=1.50)
    parser.add_argument("--srtt-threshold-frac", type=float, default=0.50)
    parser.add_argument("--output", default="")
    return parser.parse_args()


def collect_rows(dataset: dual.Dataset, args) -> list[dict]:
    queue_data = dual.load_queue_trace(dataset.queue_file)
    rows = []
    step_s = max(args.uniform_step_ms, 0.1) / 1000.0
    rtt_col = 3 if args.rtt_signal == "srtt" else 2

    for flow_index in range(1, dataset.num_flows + 1):
        paths = dual.flow_paths(dataset, flow_index)
        sendrate = dual.load_numeric_file(paths["sendrate"], (0, 1))
        delivery = dual.load_numeric_file(paths[args.recv_trace_name], (0, 1))
        rtt = dual.load_numeric_file(paths["rtt"], (0, rtt_col))
        qdelay = dual.load_numeric_file(paths["qdelay"], (0, 1, 2, 3))
        upphase = dual.load_upphase_file(paths["upphase"])
        min_rtt_s = dual.compute_min_rtt_s(qdelay, rtt)
        bdp_bytes = dataset.bottleneck_mbps * 1_000_000.0 * min_rtt_s / 8.0
        int_intervals = dual.build_intervals(upphase)

        interval_rtt_max: list[float | None] = []
        for interval in int_intervals:
            rtt_slice = dual.interval_slice(rtt, interval["int_start"], interval["int_end"])
            interval_rtt_max.append(float(np.max(rtt_slice[:, 1])) if len(rtt_slice) else None)

        delivery_window_s = max(args.rate_window_mult, 0.1) * min_rtt_s
        srtt_window_s = max(args.rtt_window_mult, 0.1) * min_rtt_s

        for idx, interval in enumerate(int_intervals):
            prev_candidates = [item for item in interval_rtt_max[max(0, idx - 2):idx] if item is not None]
            if not prev_candidates:
                continue

            recent2_max_rtt_s = max(prev_candidates)
            srtt_threshold_s = min_rtt_s + args.srtt_threshold_frac * (recent2_max_rtt_s - min_rtt_s)

            up_send = dual.interval_slice(sendrate, interval["up_start"], interval["up_end"])
            if len(up_send) < 4:
                continue
            _, up_values = dual.resample_uniform(up_send, interval["up_start"], interval["up_end"], step_s)
            sender_profile = dual.build_spectrum_profile(
                up_values,
                step_s,
                interval["ref_freq_hz"],
                args.search_gate_low,
                args.search_gate_high,
                args.nfft_mult,
                args.shape_bins,
                args.template_local_peak_frac,
            )
            if not sender_profile["valid"] or sender_profile["peak_freq_hz"] <= 0.0:
                continue

            send_peak_freq_hz = float(sender_profile["peak_freq_hz"])
            delivery_slice = dual.interval_slice(delivery, interval["int_start"], interval["int_end"])
            if len(delivery_slice) < 8:
                continue

            dt_delivery = float(np.mean(np.diff(delivery_slice[:, 0])))
            if not math.isfinite(dt_delivery) or dt_delivery <= 0.0:
                continue
            delivery_win_len = max(8, int(delivery_window_s / dt_delivery))
            if len(delivery_slice) < delivery_win_len:
                continue
            hop_len = max(1, int(delivery_win_len * (1.0 - args.overlap)))

            for start_idx in range(0, len(delivery_slice) - delivery_win_len + 1, hop_len):
                end_idx = start_idx + delivery_win_len - 1
                win_t0 = float(delivery_slice[start_idx, 0])
                win_t1 = float(delivery_slice[end_idx, 0])
                win_center = 0.5 * (win_t0 + win_t1)

                delivery_obs = delivery_slice[start_idx:end_idx + 1]
                _, delivery_values_uniform = dual.resample_uniform(
                    delivery_obs,
                    win_t0,
                    win_t1,
                    step_s,
                )
                delivery_profile = dual.build_spectrum_profile(
                    delivery_values_uniform,
                    step_s,
                    send_peak_freq_hz,
                    args.search_gate_low,
                    args.search_gate_high,
                    args.nfft_mult,
                    args.shape_bins,
                    args.template_local_peak_frac,
                )

                srtt_obs = dual.aligned_window(rtt, win_center, srtt_window_s)
                if len(srtt_obs) < 1:
                    continue
                window_min_srtt_s = float(np.min(srtt_obs[:, 1]))

                obs_times = delivery_obs[:, 0]
                obs_rtts = dual.interpolate_values(rtt[:, 0], rtt[:, 1], obs_times)
                observed_window_s = max(win_t1 - win_t0, 0.0)
                cause_window = dual.causal_queue_window(
                    obs_times,
                    obs_rtts,
                    min_rtt_s,
                    observed_window_s,
                    args,
                )
                if cause_window is None:
                    continue
                cause_t0, cause_t1 = cause_window
                queue_win = dual.extend_queue_window(queue_data, cause_t0, cause_t1)
                if len(queue_win) == 0 or bdp_bytes <= 0.0:
                    continue
                qfrac = queue_win[:, 1] / bdp_bytes
                avg_qfrac = dual.weighted_average(queue_win[:, 0], qfrac)
                if avg_qfrac is None:
                    continue

                delivery_peak_freq_hz = float(delivery_profile["peak_freq_hz"]) if delivery_profile["valid"] else 0.0
                delivery_freq_ratio = (
                    delivery_peak_freq_hz / send_peak_freq_hz if send_peak_freq_hz > 0.0 and delivery_peak_freq_hz > 0.0 else 0.0
                )

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
                        "recent2_max_rtt_s": float(recent2_max_rtt_s),
                        "srtt_threshold_s": float(srtt_threshold_s),
                        "window_min_srtt_s": float(window_min_srtt_s),
                        "send_peak_freq_hz": float(send_peak_freq_hz),
                        "delivery_valid": bool(delivery_profile["valid"]),
                        "delivery_peak_freq_hz": float(delivery_peak_freq_hz),
                        "delivery_freq_ratio": float(delivery_freq_ratio),
                        "true_nonover": bool(avg_qfrac <= args.oracle_qfrac_threshold),
                        "avg_qfrac": float(avg_qfrac),
                    }
                )
    return rows


def base_predicate(row: dict, args) -> bool:
    return (
        row["delivery_valid"]
        and row["window_min_srtt_s"] < row["srtt_threshold_s"]
        and args.freq_ratio_low < row["delivery_freq_ratio"] < args.freq_ratio_high
    )


def positive_only_metrics(rows: list[dict]) -> PositiveOnlyMetrics:
    windows = len(rows)
    pred_rows = [row for row in rows if row["pred_nonover"]]
    pred_nonover_total = len(pred_rows)
    true_nonover_total = sum(1 for row in rows if row["true_nonover"])
    true_over_total = windows - true_nonover_total
    tp_nonover = sum(1 for row in pred_rows if row["true_nonover"])
    fp_over = pred_nonover_total - tp_nonover
    abstain_total = windows - pred_nonover_total
    positive_precision = tp_nonover / pred_nonover_total if pred_nonover_total else 0.0
    false_positive_rate = fp_over / pred_nonover_total if pred_nonover_total else 0.0
    prediction_rate = pred_nonover_total / windows if windows else 0.0
    overload_leak_rate = fp_over / true_over_total if true_over_total else 0.0
    return PositiveOnlyMetrics(
        windows=windows,
        pred_nonover_total=pred_nonover_total,
        abstain_total=abstain_total,
        true_nonover_total=true_nonover_total,
        true_over_total=true_over_total,
        tp_nonover=tp_nonover,
        fp_over=fp_over,
        positive_precision=positive_precision,
        false_positive_rate=false_positive_rate,
        prediction_rate=prediction_rate,
        overload_leak_rate=overload_leak_rate,
    )


def main():
    args = parse_args()
    datasets = [dual.parse_dataset(spec) for spec in args.dataset]

    rows = []
    for dataset in datasets:
        rows.extend(collect_rows(dataset, args))
    if not rows:
        raise RuntimeError("No valid windows extracted")

    pred_rows = [{**row, "pred_nonover": bool(base_predicate(row, args))} for row in rows]
    overall = positive_only_metrics(pred_rows)

    per_dataset = {}
    for dataset_name in sorted({row["dataset"] for row in pred_rows}):
        ds_rows = [row for row in pred_rows if row["dataset"] == dataset_name]
        per_dataset[dataset_name] = {
            "metrics": asdict(positive_only_metrics(ds_rows)),
        }

    payload = {
        "config": {
            "recv_trace_name": args.recv_trace_name,
            "rtt_signal": args.rtt_signal,
            "rate_window_mult": args.rate_window_mult,
            "rtt_window_mult": args.rtt_window_mult,
            "overlap": args.overlap,
            "search_gate_low": args.search_gate_low,
            "search_gate_high": args.search_gate_high,
            "nfft_mult": args.nfft_mult,
            "oracle_qfrac_threshold": args.oracle_qfrac_threshold,
            "causal_mode": args.causal_mode,
            "pre_bottleneck_prop_ms": args.pre_bottleneck_prop_ms,
            "causal_quantile_low": args.causal_quantile_low,
            "causal_quantile_high": args.causal_quantile_high,
            "causal_pad_ms": args.causal_pad_ms,
            "causal_pad_window_frac": args.causal_pad_window_frac,
            "freq_ratio_low": args.freq_ratio_low,
            "freq_ratio_high": args.freq_ratio_high,
            "srtt_threshold_frac": args.srtt_threshold_frac,
        },
        "window_count": len(pred_rows),
        "overall": asdict(overall),
        "per_dataset": per_dataset,
    }

    print("UP delivery+minSRTT evaluation:")
    print(
        f"windows={overall.windows} recv_trace={args.recv_trace_name} "
        f"rtt_signal={args.rtt_signal} "
        f"freq_gate=({args.freq_ratio_low:.2f},{args.freq_ratio_high:.2f}) "
        f"srtt_threshold_frac={args.srtt_threshold_frac:.2f}"
    )
    print(
        f"positive_precision={overall.positive_precision:.4f} "
        f"false_positive_rate={overall.false_positive_rate:.4f} "
        f"prediction_rate={overall.prediction_rate:.4f} "
        f"overload_leak_rate={overall.overload_leak_rate:.4f} "
        f"pred_nonover_total={overall.pred_nonover_total}"
    )

    if args.output:
        with open(args.output, "w", encoding="utf-8") as handle:
            json.dump(payload, handle, indent=2, sort_keys=True)
        print(f"Saved summary to {args.output}")


if __name__ == "__main__":
    main()
