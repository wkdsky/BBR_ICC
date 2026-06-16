#!/usr/bin/env python3
"""
Evaluate the user's NONOVER rule at a short-stage granularity.

Window rule:
  predict NONOVER iff
    min(srtt in window) <
      current_bbr_min_rtt + srtt_threshold_frac *
      (recent2_probe_bw_max_rtt - current_bbr_min_rtt)
    and
      freq_ratio_low < f_delivery / f_send < freq_ratio_high

Stage rule:
  - only runs of at least stage_min_positive_windows consecutive positive
    windows count as predicted NONOVER stages
  - a predicted stage is correct iff its causal queue interval contains at
    least one sample with qfrac <= oracle_qfrac_threshold
  - a predicted stage is wrong iff that causal queue interval stays above the
    threshold for the whole stage
  - stages without a NONOVER prediction are ignored
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
class StageMetrics:
    predicted_stages: int
    correct_stages: int
    wrong_all_over_stages: int
    stage_positive_precision: float
    mean_stage_duration_s: float
    mean_stage_windows: float


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
    parser.add_argument("--search-gate-high", type=float, default=1.50)
    parser.add_argument("--nfft-mult", type=int, default=4)
    parser.add_argument("--uniform-step-ms", type=float, default=1.0)
    parser.add_argument("--shape-bins", type=int, default=24)
    parser.add_argument("--template-local-peak-frac", type=float, default=0.7)
    parser.add_argument("--oracle-qfrac-threshold", type=float, default=1.0)
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
    parser.add_argument("--srtt-threshold-frac", type=float, default=0.60)
    parser.add_argument("--stage-min-positive-windows", type=int, default=2)
    parser.add_argument("--output", default="")
    return parser.parse_args()


def build_probe_bw_cycles(up_phases: list[dict]) -> list[dict]:
    cycles = []
    for cur, nxt in zip(up_phases[:-1], up_phases[1:]):
        if nxt["start"] <= cur["end"]:
            continue
        cycles.append(
            {
                "cycle_start": cur["start"],
                "cycle_end": nxt["start"],
                "up_start": cur["start"],
                "up_end": cur["end"],
                "int_start": cur["end"],
                "int_end": nxt["start"],
                "ref_freq_hz": cur["freq_hz"],
            }
        )
    return cycles


def latest_step_value(sample_times: np.ndarray, sample_values: np.ndarray, query_time: float) -> float:
    if len(sample_times) == 0 or len(sample_values) == 0:
        return float("nan")
    index = int(np.searchsorted(sample_times, query_time, side="right")) - 1
    index = max(0, min(index, len(sample_values) - 1))
    return float(sample_values[index])


def finalize_stage(
    stage_rows: list[dict],
    queue_data: np.ndarray,
    queue_prop_times: np.ndarray,
    queue_prop_rtt_s: np.ndarray,
    dataset: dual.Dataset,
    args,
    stage_records: list[dict],
):
    if len(stage_rows) < args.stage_min_positive_windows:
        return

    cause_t0 = min(row["cause_t0"] for row in stage_rows)
    cause_t1 = max(row["cause_t1"] for row in stage_rows)
    queue_win = dual.extend_queue_window(queue_data, cause_t0, cause_t1)
    if len(queue_win) == 0:
        return

    prop_rtt_s = dual.interpolate_values(queue_prop_times, queue_prop_rtt_s, queue_win[:, 0])
    if len(prop_rtt_s) == 0:
        return

    valid = np.isfinite(prop_rtt_s) & (prop_rtt_s > 0.0)
    if not np.any(valid):
        return

    queue_bytes = queue_win[valid, 1]
    bdp_bytes = dataset.bottleneck_mbps * 1_000_000.0 * prop_rtt_s[valid] / 8.0
    qfrac = queue_bytes / np.maximum(bdp_bytes, 1.0)
    has_nonover = bool(np.any(qfrac <= args.oracle_qfrac_threshold))

    stage_records.append(
        {
            "stage_start": float(stage_rows[0]["win_t0"]),
            "stage_end": float(stage_rows[-1]["win_t1"]),
            "stage_duration_s": float(stage_rows[-1]["win_t1"] - stage_rows[0]["win_t0"]),
            "window_count": len(stage_rows),
            "has_nonover": has_nonover,
            "qfrac_min": float(np.min(qfrac)),
            "qfrac_max": float(np.max(qfrac)),
        }
    )


def compute_stage_metrics(stage_records: list[dict]) -> StageMetrics:
    predicted = len(stage_records)
    correct = sum(1 for item in stage_records if item["has_nonover"])
    wrong = predicted - correct
    mean_duration = float(np.mean([item["stage_duration_s"] for item in stage_records])) if stage_records else 0.0
    mean_windows = float(np.mean([item["window_count"] for item in stage_records])) if stage_records else 0.0
    precision = correct / predicted if predicted else 0.0
    return StageMetrics(
        predicted_stages=predicted,
        correct_stages=correct,
        wrong_all_over_stages=wrong,
        stage_positive_precision=precision,
        mean_stage_duration_s=mean_duration,
        mean_stage_windows=mean_windows,
    )


def evaluate_dataset(dataset: dual.Dataset, args) -> dict:
    queue_data = dual.load_queue_trace(dataset.queue_file)
    rtt_col = 3 if args.rtt_signal == "srtt" else 2
    step_s = max(args.uniform_step_ms, 0.1) / 1000.0
    stage_records = []

    for flow_index in range(1, dataset.num_flows + 1):
        paths = dual.flow_paths(dataset, flow_index)
        sendrate = dual.load_numeric_file(paths["sendrate"], (0, 1))
        delivery = dual.load_numeric_file(paths[args.recv_trace_name], (0, 1))
        rtt = dual.load_numeric_file(paths["rtt"], (0, rtt_col))
        qdelay = dual.load_numeric_file(paths["qdelay"], (0, 1, 2, 3))
        upphase = dual.load_upphase_file(paths["upphase"])
        cycles = build_probe_bw_cycles(upphase)

        if len(sendrate) == 0 or len(delivery) == 0 or len(rtt) == 0 or len(qdelay) == 0 or len(cycles) < 3:
            continue

        qdelay_times = qdelay[:, 0]
        qdelay_min_rtt_s = qdelay[:, 3] / 1000.0
        prop_rtt_s = np.maximum((qdelay[:, 2] - qdelay[:, 1]) / 1000.0, 1e-9)

        cycle_rtt_max_s: list[float | None] = []
        for cycle in cycles:
            cycle_rtt = dual.interval_slice(rtt, cycle["cycle_start"], cycle["cycle_end"])
            cycle_rtt_max_s.append(float(np.max(cycle_rtt[:, 1])) if len(cycle_rtt) else None)

        for cycle_idx, cycle in enumerate(cycles):
            prev_cycle_max = [item for item in cycle_rtt_max_s[max(0, cycle_idx - 2):cycle_idx] if item is not None]
            if len(prev_cycle_max) < 2:
                continue

            recent2_max_rtt_s = max(prev_cycle_max)

            up_send = dual.interval_slice(sendrate, cycle["up_start"], cycle["up_end"])
            if len(up_send) < 4:
                continue
            _, up_values = dual.resample_uniform(up_send, cycle["up_start"], cycle["up_end"], step_s)
            sender_profile = dual.build_spectrum_profile(
                up_values,
                step_s,
                cycle["ref_freq_hz"],
                args.search_gate_low,
                args.search_gate_high,
                args.nfft_mult,
                args.shape_bins,
                args.template_local_peak_frac,
            )
            if not sender_profile["valid"] or sender_profile["peak_freq_hz"] <= 0.0:
                continue
            send_peak_freq_hz = float(sender_profile["peak_freq_hz"])

            delivery_slice = dual.interval_slice(delivery, cycle["int_start"], cycle["int_end"])
            if len(delivery_slice) < 8:
                continue
            dt_delivery = float(np.mean(np.diff(delivery_slice[:, 0])))
            if not math.isfinite(dt_delivery) or dt_delivery <= 0.0:
                continue
            delivery_window_s = max(args.rate_window_mult, 0.1) * max(latest_step_value(qdelay_times, qdelay_min_rtt_s, cycle["int_start"]), 1e-9)
            delivery_win_len = max(8, int(delivery_window_s / dt_delivery))
            if len(delivery_slice) < delivery_win_len:
                continue
            hop_len = max(1, int(delivery_win_len * (1.0 - args.overlap)))

            cycle_rows = []
            for start_idx in range(0, len(delivery_slice) - delivery_win_len + 1, hop_len):
                end_idx = start_idx + delivery_win_len - 1
                win_t0 = float(delivery_slice[start_idx, 0])
                win_t1 = float(delivery_slice[end_idx, 0])
                win_center = 0.5 * (win_t0 + win_t1)

                current_min_rtt_s = latest_step_value(qdelay_times, qdelay_min_rtt_s, win_center)
                if not math.isfinite(current_min_rtt_s) or current_min_rtt_s <= 0.0:
                    continue

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

                srtt_window_s = max(args.rtt_window_mult, 0.1) * current_min_rtt_s
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
                    current_min_rtt_s,
                    observed_window_s,
                    args,
                )
                if cause_window is None:
                    continue
                cause_t0, cause_t1 = cause_window

                delivery_peak_freq_hz = float(delivery_profile["peak_freq_hz"]) if delivery_profile["valid"] else 0.0
                delivery_freq_ratio = (
                    delivery_peak_freq_hz / send_peak_freq_hz
                    if send_peak_freq_hz > 0.0 and delivery_peak_freq_hz > 0.0
                    else 0.0
                )
                srtt_threshold_s = current_min_rtt_s + args.srtt_threshold_frac * (recent2_max_rtt_s - current_min_rtt_s)
                pred_nonover = (
                    delivery_profile["valid"]
                    and window_min_srtt_s < srtt_threshold_s
                    and args.freq_ratio_low < delivery_freq_ratio < args.freq_ratio_high
                )

                cycle_rows.append(
                    {
                        "cycle_idx": cycle_idx,
                        "flow": flow_index,
                        "win_t0": win_t0,
                        "win_t1": win_t1,
                        "cause_t0": float(cause_t0),
                        "cause_t1": float(cause_t1),
                        "pred_nonover": bool(pred_nonover),
                    }
                )

            if not cycle_rows:
                continue

            run = []
            for row in cycle_rows:
                if row["pred_nonover"]:
                    run.append(row)
                else:
                    finalize_stage(
                        run,
                        queue_data,
                        qdelay_times,
                        prop_rtt_s,
                        dataset,
                        args,
                        stage_records,
                    )
                    run = []
            finalize_stage(
                run,
                queue_data,
                qdelay_times,
                prop_rtt_s,
                dataset,
                args,
                stage_records,
            )

    metrics = compute_stage_metrics(stage_records)
    return {
        "metrics": asdict(metrics),
        "stage_records": stage_records,
    }


def main():
    args = parse_args()
    datasets = [dual.parse_dataset(spec) for spec in args.dataset]

    per_dataset = {}
    all_stage_records = []
    for dataset in datasets:
        result = evaluate_dataset(dataset, args)
        per_dataset[dataset.name] = result
        all_stage_records.extend(result["stage_records"])

    overall = compute_stage_metrics(all_stage_records)
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
            "stage_min_positive_windows": args.stage_min_positive_windows,
        },
        "overall": asdict(overall),
        "per_dataset": per_dataset,
    }

    print("Stage-level NONOVER evaluation:")
    print(
        f"recv_trace={args.recv_trace_name} rtt_signal={args.rtt_signal} "
        f"freq_gate=({args.freq_ratio_low:.2f},{args.freq_ratio_high:.2f}) "
        f"srtt_threshold_frac={args.srtt_threshold_frac:.2f} "
        f"stage_min_positive_windows={args.stage_min_positive_windows} "
        f"oracle_qfrac_threshold={args.oracle_qfrac_threshold:.2f}"
    )
    print(
        f"overall predicted_stages={overall.predicted_stages} "
        f"correct_stages={overall.correct_stages} "
        f"wrong_all_over_stages={overall.wrong_all_over_stages} "
        f"stage_positive_precision={overall.stage_positive_precision:.4f}"
    )
    for dataset in datasets:
        metrics = per_dataset[dataset.name]["metrics"]
        print(
            f"{dataset.name} predicted_stages={metrics['predicted_stages']} "
            f"correct_stages={metrics['correct_stages']} "
            f"wrong_all_over_stages={metrics['wrong_all_over_stages']} "
            f"stage_positive_precision={metrics['stage_positive_precision']:.4f} "
            f"mean_stage_duration_s={metrics['mean_stage_duration_s']:.4f} "
            f"mean_stage_windows={metrics['mean_stage_windows']:.2f}"
        )

    if args.output:
        with open(args.output, "w", encoding="utf-8") as handle:
            json.dump(payload, handle, indent=2, sort_keys=True)
        print(f"Saved summary to {args.output}")


if __name__ == "__main__":
    main()
