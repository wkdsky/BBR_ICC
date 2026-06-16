#!/usr/bin/env python3
"""
Evaluate a causal temporal confirmation rule on top of the fixed dual-frequency
 window classifier.

Base window rule:
  recv_template_align_rttupper
  D_recv <= recv_shape_thr
  E_recv >= recv_energy_thr
  A <= align_thr
  rho_rtt <= rtt_upper_thr

Temporal confirmation:
  For the current window, look back over H RTTs and partition that history into
  bins of width B RTT. If at least K bins contain at least one base-positive
  window, confirm the current window as NONOVER.

This is overlap-aware: heavily overlapped STFT windows inside the same short
time region count as one temporal support hit.
"""

from __future__ import annotations

import argparse
import itertools
import json
import math
import os
import sys
from dataclasses import asdict


SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

import freqccv3_window_dualfreq_eval as dual  # noqa: E402


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
        default="recvrate",
    )
    parser.add_argument(
        "--rtt-signal",
        choices=["rtt", "srtt"],
        default="rtt",
    )
    parser.add_argument("--rate-window-mult", type=float, default=0.75)
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

    parser.add_argument("--recv-shape-thr", type=float, default=0.45)
    parser.add_argument("--recv-energy-thr", type=float, default=0.08)
    parser.add_argument("--align-thr", type=float, default=0.45)
    parser.add_argument("--rtt-upper-thr", type=float, default=0.60)

    parser.add_argument("--horizon-rtt-mults", default="2,3,4,5,6,7")
    parser.add_argument("--support-bin-rtt-mults", default="0.5,1.0,1.5")
    parser.add_argument("--min-hit-bins", default="1,2,3,4,5,6")
    parser.add_argument("--require-current-positive", action="store_true", default=True)
    parser.add_argument("--top-k", type=int, default=20)
    parser.add_argument("--output", default="")
    return parser.parse_args()


def parse_float_list(text: str) -> list[float]:
    return [float(item.strip()) for item in text.split(",") if item.strip()]


def parse_int_list(text: str) -> list[int]:
    return [int(item.strip()) for item in text.split(",") if item.strip()]


def base_predicate(row: dict, args) -> bool:
    return (
        row["recv_valid"]
        and row["recv_shape_dist"] <= args.recv_shape_thr
        and row["recv_band_energy"] >= args.recv_energy_thr
        and row["peak_alignment"] <= args.align_thr
        and row["rtt_peak_ratio"] <= args.rtt_upper_thr
    )


def group_rows(rows: list[dict]) -> dict[tuple[str, int], list[dict]]:
    grouped: dict[tuple[str, int], list[dict]] = {}
    for row in rows:
        grouped.setdefault((row["dataset"], row["flow"]), []).append(row)
    for key in grouped:
        grouped[key] = sorted(grouped[key], key=lambda item: 0.5 * (item["win_t0"] + item["win_t1"]))
    return grouped


def apply_temporal_confirmation(
    rows: list[dict],
    horizon_rtt_mult: float,
    support_bin_rtt_mult: float,
    min_hit_bins: int,
    require_current_positive: bool,
) -> list[dict]:
    grouped = group_rows(rows)
    out = []

    for group_rows_sorted in grouped.values():
        centers = [0.5 * (row["win_t0"] + row["win_t1"]) for row in group_rows_sorted]
        for idx, row in enumerate(group_rows_sorted):
            min_rtt_s = max(float(row["min_rtt_s"]), 1e-9)
            horizon_s = max(horizon_rtt_mult, 1e-9) * min_rtt_s
            bin_s = max(support_bin_rtt_mult, 1e-9) * min_rtt_s
            current_center = centers[idx]
            cutoff = current_center - horizon_s

            confirmed = False
            if row["base_pred_nonover"] or not require_current_positive:
                hit_bins: set[int] = set()
                scan_idx = idx
                while scan_idx >= 0 and centers[scan_idx] >= cutoff:
                    other = group_rows_sorted[scan_idx]
                    if other["base_pred_nonover"]:
                        delta_s = current_center - centers[scan_idx]
                        bin_idx = int(math.floor(delta_s / bin_s))
                        hit_bins.add(bin_idx)
                        if len(hit_bins) >= min_hit_bins:
                            confirmed = True
                            break
                    scan_idx -= 1

            out.append({**row, "pred_nonover": confirmed})

    return out


def support_stats(base_rows: list[dict], confirmed_rows: list[dict]) -> dict[str, float | int]:
    base_positive = sum(1 for row in base_rows if row["base_pred_nonover"])
    confirmed = sum(1 for row in confirmed_rows if row["pred_nonover"])
    base_true = sum(1 for row in base_rows if row["base_pred_nonover"] and row["true_nonover"])
    confirmed_true = sum(1 for row in confirmed_rows if row["pred_nonover"] and row["true_nonover"])
    return {
        "base_positive_windows": base_positive,
        "confirmed_windows": confirmed,
        "base_positive_precision": base_true / base_positive if base_positive else 0.0,
        "confirmed_precision": confirmed_true / confirmed if confirmed else 0.0,
        "confirmed_over_base_ratio": confirmed / base_positive if base_positive else 0.0,
    }


def evaluate_temporal_rule(base_rows: list[dict], args, horizon_rtt_mult: float, support_bin_rtt_mult: float, min_hit_bins: int) -> dict:
    confirmed_rows = apply_temporal_confirmation(
        base_rows,
        horizon_rtt_mult=horizon_rtt_mult,
        support_bin_rtt_mult=support_bin_rtt_mult,
        min_hit_bins=min_hit_bins,
        require_current_positive=args.require_current_positive,
    )
    overall = dual.compute_metrics(confirmed_rows)
    tolerant = dual.compute_metrics(dual.tolerant_rows(confirmed_rows, args.tolerant_match_rtt_frac))
    neighbor = dual.neighborhood_metrics(confirmed_rows, args.tolerant_match_rtt_frac)
    per_dataset = {}
    for dataset_name in sorted({row["dataset"] for row in confirmed_rows}):
        ds_rows = [row for row in confirmed_rows if row["dataset"] == dataset_name]
        per_dataset[dataset_name] = {
            "overall": asdict(dual.compute_metrics(ds_rows)),
            "neighbor": dual.neighborhood_metrics(ds_rows, args.tolerant_match_rtt_frac),
        }
    return {
        "params": {
            "horizon_rtt_mult": horizon_rtt_mult,
            "support_bin_rtt_mult": support_bin_rtt_mult,
            "min_hit_bins": min_hit_bins,
            "require_current_positive": args.require_current_positive,
        },
        "overall": asdict(overall),
        "tolerant_overall": asdict(tolerant),
        "neighbor_overall": neighbor,
        "support_stats": support_stats(base_rows, confirmed_rows),
        "per_dataset": per_dataset,
    }


def main():
    args = parse_args()
    dual.TOLERANCE_RTT_FRAC = args.tolerant_match_rtt_frac
    datasets = [dual.parse_dataset(spec) for spec in args.dataset]

    rows = []
    for dataset in datasets:
        rows.extend(dual.collect_rows(dataset, args))
    if not rows:
        raise RuntimeError("No valid windows extracted")

    base_rows = [{**row, "base_pred_nonover": bool(base_predicate(row, args))} for row in rows]
    base_eval_rows = [{**row, "pred_nonover": bool(row["base_pred_nonover"])} for row in base_rows]
    base_overall = dual.compute_metrics(base_eval_rows)
    base_tolerant = dual.compute_metrics(dual.tolerant_rows(base_eval_rows, args.tolerant_match_rtt_frac))
    base_neighbor = dual.neighborhood_metrics(base_eval_rows, args.tolerant_match_rtt_frac)

    results = []
    for horizon_rtt_mult, support_bin_rtt_mult, min_hit_bins in itertools.product(
        parse_float_list(args.horizon_rtt_mults),
        parse_float_list(args.support_bin_rtt_mults),
        parse_int_list(args.min_hit_bins),
    ):
        if support_bin_rtt_mult > horizon_rtt_mult + 1e-12:
            continue
        max_bins = int(math.floor(horizon_rtt_mult / support_bin_rtt_mult + 1e-9)) + 1
        if min_hit_bins > max_bins:
            continue
        results.append(
            evaluate_temporal_rule(
                base_rows,
                args,
                horizon_rtt_mult=horizon_rtt_mult,
                support_bin_rtt_mult=support_bin_rtt_mult,
                min_hit_bins=min_hit_bins,
            )
        )

    results.sort(
        key=lambda item: (
            item["neighbor_overall"]["neighbor_balanced_accuracy"],
            item["neighbor_overall"]["hit_nonover_rate"],
            item["neighbor_overall"]["clean_over_rate"],
            item["support_stats"]["confirmed_precision"],
        ),
        reverse=True,
    )

    top = results[: args.top_k]
    print("Temporal confirmation evaluation:")
    print(
        f"windows={len(rows)} recv_trace={args.recv_trace_name} "
        f"rtt_signal={args.rtt_signal} "
        f"base_rule=recv_template_align_rttupper "
        f"shape={args.recv_shape_thr:.2f} energy={args.recv_energy_thr:.2f} "
        f"align={args.align_thr:.2f} rtt_upper={args.rtt_upper_thr:.2f}"
    )
    print(
        "base "
        f"nbr_bal_acc={base_neighbor['neighbor_balanced_accuracy']:.4f} "
        f"hit_nonover={base_neighbor['hit_nonover_rate']:.4f} "
        f"clean_over={base_neighbor['clean_over_rate']:.4f} "
        f"confirmed_precision={support_stats(base_rows, base_eval_rows)['confirmed_precision']:.4f}"
    )
    for idx, item in enumerate(top, start=1):
        neighbor = item["neighbor_overall"]
        print(
            f"{idx:02d}. params={json.dumps(item['params'], sort_keys=True)} "
            f"nbr_bal_acc={neighbor['neighbor_balanced_accuracy']:.4f} "
            f"hit_nonover={neighbor['hit_nonover_rate']:.4f} "
            f"clean_over={neighbor['clean_over_rate']:.4f} "
            f"confirmed_precision={item['support_stats']['confirmed_precision']:.4f} "
            f"confirmed_windows={item['support_stats']['confirmed_windows']}"
        )

    if args.output:
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
                "tolerant_match_rtt_frac": args.tolerant_match_rtt_frac,
                "recv_shape_thr": args.recv_shape_thr,
                "recv_energy_thr": args.recv_energy_thr,
                "align_thr": args.align_thr,
                "rtt_upper_thr": args.rtt_upper_thr,
            },
            "window_count": len(rows),
            "base": {
                "overall": asdict(base_overall),
                "tolerant_overall": asdict(base_tolerant),
                "neighbor_overall": base_neighbor,
                "support_stats": support_stats(base_rows, base_eval_rows),
            },
            "top": top,
        }
        with open(args.output, "w", encoding="utf-8") as handle:
            json.dump(payload, handle, indent=2)
        print(f"\nSaved summary to {args.output}")


if __name__ == "__main__":
    main()
