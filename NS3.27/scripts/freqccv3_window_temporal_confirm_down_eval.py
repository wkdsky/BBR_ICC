#!/usr/bin/env python3
"""
Evaluate the fixed dual-frequency window rule plus causal temporal confirmation
when the sender-side modulation phase is PROBE_DOWN instead of PROBE_UP.

This script keeps the same window rule and the same temporal confirmer as the
UP-based evaluator, but changes the reference phase:

  current rule:
    recv_template_align_rttupper
    D_recv <= recv_shape_thr
    E_recv >= recv_energy_thr
    A <= align_thr
    rho_rtt <= rtt_upper_thr

  reference phase:
    previous PROBE_DOWN sendrate segment

  judged interval:
    previous PROBE_DOWN end  ->  next PROBE_DOWN start
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
        default="srtt",
    )
    parser.add_argument("--nominal-probe-freq-hz", type=float, default=60.0)
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


def load_mode_trace(path: str) -> list[tuple[float, str]]:
    rows: list[tuple[float, str]] = []
    with open(path, "r", encoding="utf-8") as handle:
        for line in handle:
            if not line.strip() or line.startswith("#"):
                continue
            parts = line.split()
            if len(parts) < 2:
                continue
            try:
                rows.append((float(parts[0]), str(parts[1]).strip()))
            except ValueError:
                continue
    rows.sort(key=lambda item: item[0])
    return rows


def extract_phase_intervals(mode_rows: list[tuple[float, str]], target: str) -> list[tuple[float, float]]:
    intervals: list[tuple[float, float]] = []
    in_seg = False
    t_start = 0.0
    for idx, (time_s, mode) in enumerate(mode_rows):
        if mode == target and not in_seg:
            in_seg = True
            t_start = time_s
        elif mode != target and in_seg:
            intervals.append((t_start, time_s))
            in_seg = False
    if in_seg and mode_rows:
        intervals.append((t_start, mode_rows[-1][0]))
    return intervals


def down_phase_intervals(dataset: dual.Dataset, flow_index: int) -> list[dict]:
    paths = dual.flow_paths(dataset, flow_index)
    base = paths["sendrate"][: -len("_sendrate.txt")]
    mode_path = base + "_bbrmode.txt"
    mode_rows = load_mode_trace(mode_path)
    down_segments = extract_phase_intervals(mode_rows, "probeBW_down")
    out = []
    for cur, nxt in zip(down_segments[:-1], down_segments[1:]):
        cur_start, cur_end = cur
        nxt_start, _ = nxt
        if nxt_start <= cur_end:
            continue
        out.append(
            {
                "mod_start": cur_start,
                "mod_end": cur_end,
                "int_start": cur_end,
                "int_end": nxt_start,
            }
        )
    return out


def collect_rows(dataset: dual.Dataset, args) -> list[dict]:
    queue_data = dual.load_queue_trace(dataset.queue_file)
    rows = []
    step_s = max(args.uniform_step_ms, 0.1) / 1000.0
    rtt_col = 3 if args.rtt_signal == "srtt" else 2

    for flow_index in range(1, dataset.num_flows + 1):
        paths = dual.flow_paths(dataset, flow_index)
        sendrate = dual.load_numeric_file(paths["sendrate"], (0, 1))
        recvrate = dual.load_numeric_file(paths[args.recv_trace_name], (0, 1))
        rtt = dual.load_numeric_file(paths["rtt"], (0, rtt_col))
        qdelay = dual.load_numeric_file(paths["qdelay"], (0, 1, 2, 3))
        min_rtt_s = dual.compute_min_rtt_s(qdelay, rtt)
        bdp_bytes = dataset.bottleneck_mbps * 1_000_000.0 * min_rtt_s / 8.0
        intervals = down_phase_intervals(dataset, flow_index)
        interval_rtt_max = []
        for interval in intervals:
            rtt_slice = dual.interval_slice(rtt, interval["int_start"], interval["int_end"])
            interval_rtt_max.append(float(rtt_slice[:, 1].max()) if len(rtt_slice) else None)

        recv_window_s = max(args.rate_window_mult, 0.1) * min_rtt_s
        rtt_window_s = max(args.rtt_window_mult, 0.1) * min_rtt_s

        for idx, interval in enumerate(intervals):
            prev_candidates = [item for item in interval_rtt_max[max(0, idx - 2):idx] if item is not None]
            if not prev_candidates:
                continue

            mod_send = dual.interval_slice(sendrate, interval["mod_start"], interval["mod_end"])
            if len(mod_send) < 4:
                continue
            _, mod_values = dual.resample_uniform(
                mod_send,
                interval["mod_start"],
                interval["mod_end"],
                step_s,
            )
            sender_profile = dual.build_spectrum_profile(
                mod_values,
                step_s,
                args.nominal_probe_freq_hz,
                args.search_gate_low,
                args.search_gate_high,
                args.nfft_mult,
                args.shape_bins,
                args.template_local_peak_frac,
            )
            if not sender_profile["valid"]:
                continue
            ref_freq_hz = float(sender_profile["peak_freq_hz"]) if sender_profile["peak_freq_hz"] > 0 else args.nominal_probe_freq_hz
            sender_profile = dual.build_spectrum_profile(
                mod_values,
                step_s,
                ref_freq_hz,
                args.search_gate_low,
                args.search_gate_high,
                args.nfft_mult,
                args.shape_bins,
                args.template_local_peak_frac,
            )
            if not sender_profile["valid"]:
                continue

            recv_slice = dual.interval_slice(recvrate, interval["int_start"], interval["int_end"])
            if len(recv_slice) < 8:
                continue

            dt_recv = float((recv_slice[1:, 0] - recv_slice[:-1, 0]).mean()) if len(recv_slice) >= 2 else math.nan
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

                recv_obs = recv_slice[start_idx:end_idx + 1]
                _, recv_values_uniform = dual.resample_uniform(recv_obs, win_t0, win_t1, step_s)
                recv_profile = dual.build_spectrum_profile(
                    recv_values_uniform,
                    step_s,
                    ref_freq_hz,
                    args.search_gate_low,
                    args.search_gate_high,
                    args.nfft_mult,
                    args.shape_bins,
                    args.template_local_peak_frac,
                )

                rtt_obs = dual.aligned_window(rtt, win_center, rtt_window_s)
                if len(rtt_obs) < 6:
                    continue
                rtt_t0 = max(interval["int_start"], win_center - 0.5 * rtt_window_s)
                rtt_t1 = min(interval["int_end"], win_center + 0.5 * rtt_window_s)
                _, rtt_values_uniform = dual.resample_uniform(rtt_obs, rtt_t0, rtt_t1, step_s)
                rtt_profile = dual.build_spectrum_profile(
                    rtt_values_uniform,
                    step_s,
                    ref_freq_hz,
                    args.search_gate_low,
                    args.search_gate_high,
                    args.nfft_mult,
                    args.shape_bins,
                    args.template_local_peak_frac,
                )

                recv_shape_dist = dual.shape_distance(recv_profile["band_shape"], sender_profile["band_shape"])
                rtt_shape_dist = dual.shape_distance(rtt_profile["band_shape"], sender_profile["band_shape"])

                obs_times = recv_obs[:, 0]
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
                if len(queue_win) == 0 or bdp_bytes <= 0:
                    continue
                qfrac = queue_win[:, 1] / bdp_bytes
                avg_qfrac = dual.weighted_average(queue_win[:, 0], qfrac)
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
                        "ref_freq_hz": ref_freq_hz,
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
                        hit_bins.add(int(math.floor(delta_s / bin_s)))
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
    }


def main():
    args = parse_args()
    dual.TOLERANCE_RTT_FRAC = args.tolerant_match_rtt_frac
    datasets = [dual.parse_dataset(spec) for spec in args.dataset]

    rows = []
    for dataset in datasets:
        rows.extend(collect_rows(dataset, args))
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

    top = results[:args.top_k]
    print("Temporal confirmation evaluation (PROBE_DOWN reference):")
    print(
        f"windows={len(rows)} recv_trace={args.recv_trace_name} "
        f"rtt_signal={args.rtt_signal} nominal_probe_freq_hz={args.nominal_probe_freq_hz:.1f} "
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
                "reference_phase": "probe_down",
                "recv_trace_name": args.recv_trace_name,
                "rtt_signal": args.rtt_signal,
                "nominal_probe_freq_hz": args.nominal_probe_freq_hz,
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
