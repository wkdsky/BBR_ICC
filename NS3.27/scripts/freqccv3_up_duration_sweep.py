#!/usr/bin/env python3
"""
Sweep the experimental minimum PROBE_UP duration gate and measure whether
window-level NONOVER/OVER detection becomes more reliable.

Workflow per duration target:
  1. run the 2-flow calibration scenario for multiple fixed background rates
  2. parse actual achieved UP durations from *_upphase.txt
  3. call freqccv3_window_overload_eval.py on the emitted trace directories
  4. aggregate neighbor-style accuracy metrics across all datasets

This script intentionally keeps the online rule unchanged. It only varies
the minimum allowed PROBE_UP residence time before queue-triggered exit.
"""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
from dataclasses import dataclass

import numpy as np


@dataclass
class RunSpec:
    duration_mult: float
    bg_rate_mbps: float
    trace_dir: str
    dataset_name: str
    prefix: str


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--ns3-dir", default="NS3.27")
    parser.add_argument("--output-root", required=True)
    parser.add_argument("--durations", default="0,0.5,1.0,1.5,2.0,2.5,3.0,4.0")
    parser.add_argument("--bg-rates", default="0,4,8,12")
    parser.add_argument("--sim-time", type=float, default=20.0)
    parser.add_argument("--probe-freq-hz", type=float, default=60.0)
    parser.add_argument("--probe-amp-mode", default="miu2")
    parser.add_argument("--queue-state", default="manual")
    parser.add_argument("--bottleneck-mbps", type=float, default=20.0)
    parser.add_argument("--bottleneck-delay-ms", type=float, default=18.0)
    parser.add_argument("--rate-window-mult", type=float, default=0.75)
    parser.add_argument("--overlap", type=float, default=0.9)
    parser.add_argument("--nfft-mult", type=int, default=4)
    parser.add_argument("--search-gate-low", type=float, default=0.7)
    parser.add_argument("--search-gate-high", type=float, default=1.3)
    parser.add_argument("--close-tols", default="0.30,0.25,0.20,0.15,0.10")
    parser.add_argument("--rtt-below-ratio", type=float, default=0.9)
    parser.add_argument("--oracle-qfrac-threshold", type=float, default=0.9)
    parser.add_argument("--center-mode", choices=["ref", "empirical-nonover"], default="empirical-nonover")
    parser.add_argument("--center-value", type=float, default=1.0)
    parser.add_argument("--causal-mode", choices=["observe", "delivery", "arrival", "band"], default="delivery")
    parser.add_argument("--causal-quantile-low", type=float, default=0.10)
    parser.add_argument("--causal-quantile-high", type=float, default=0.90)
    parser.add_argument("--causal-pad-ms", type=float, default=0.0)
    parser.add_argument("--causal-pad-window-frac", type=float, default=0.25)
    parser.add_argument("--tolerant-match-rtt-frac", type=float, default=0.5)
    parser.add_argument("--pre-bottleneck-prop-ms", type=float, default=1.0)
    parser.add_argument("--oracle-flow-count", type=int, default=1)
    parser.add_argument("--skip-runs", action="store_true")
    parser.add_argument("--keep-going", action="store_true")
    return parser.parse_args()


def parse_csv_floats(text: str) -> list[float]:
    return [float(item.strip()) for item in text.split(",") if item.strip()]


def safe_tag(value: float) -> str:
    text = f"{value:.3f}".rstrip("0").rstrip(".")
    return text.replace("-", "m").replace(".", "p")


def ensure_dir(path: str):
    os.makedirs(path, exist_ok=True)


def run_command(cmd: list[str], cwd: str):
    return subprocess.run(cmd, cwd=cwd, text=True, capture_output=True)


def load_upphase_durations_rtt(path: str) -> list[float]:
    rows = []
    if not os.path.exists(path):
        return rows
    with open(path, "r", encoding="utf-8") as handle:
        for line in handle:
            if not line.strip() or line.startswith("#"):
                continue
            parts = line.split()
            if len(parts) < 2:
                continue
            try:
                duration_ms = float(parts[1])
            except ValueError:
                continue
            duration_s = duration_ms / 1000.0
            rows.append(duration_s)
    return rows


def infer_min_rtt_s_from_qdelay(path: str) -> float | None:
    if not os.path.exists(path):
        return None
    mins = []
    with open(path, "r", encoding="utf-8") as handle:
        for line in handle:
            if not line.strip() or line.startswith("#"):
                continue
            parts = line.split()
            if len(parts) < 4:
                continue
            try:
                mins.append(float(parts[3]) / 1000.0)
            except ValueError:
                continue
    if not mins:
        return None
    return min(mins)


def summarize(values: list[float]) -> dict[str, float | int | None]:
    if not values:
        return {
            "count": 0,
            "mean": None,
            "median": None,
            "p10": None,
            "p90": None,
            "stdev": None,
        }
    arr = np.asarray(values, dtype=float)
    stdev = float(np.std(arr)) if len(arr) > 1 else 0.0
    return {
        "count": int(len(arr)),
        "mean": float(np.mean(arr)),
        "median": float(np.median(arr)),
        "p10": float(np.percentile(arr, 10)),
        "p90": float(np.percentile(arr, 90)),
        "stdev": stdev,
    }


def make_run_specs(args) -> list[RunSpec]:
    specs = []
    for duration_mult in parse_csv_floats(args.durations):
        dur_tag = safe_tag(duration_mult)
        for bg_rate in parse_csv_floats(args.bg_rates):
            bg_tag = safe_tag(bg_rate)
            trace_dir = os.path.join(args.output_root, f"dur{dur_tag}_bg{bg_tag}")
            specs.append(
                RunSpec(
                    duration_mult=duration_mult,
                    bg_rate_mbps=bg_rate,
                    trace_dir=trace_dir,
                    dataset_name=f"dur{dur_tag}_bg{bg_tag}",
                    prefix="freqccv3_2flow_calibration_manual_probe",
                )
            )
    return specs


def scenario_cmd(args, spec: RunSpec) -> list[str]:
    run_string = (
        "scratch/freqccv3_2flow_calibration "
        f"--trace_path={spec.trace_dir} "
        f"--queue_state={args.queue_state} "
        f"--bg_rate_mbps={spec.bg_rate_mbps} "
        f"--probe_freq_hz={args.probe_freq_hz} "
        f"--probe_amp_mode={args.probe_amp_mode} "
        f"--probe_up_min_rtt_mult={spec.duration_mult} "
        f"--sim_time={args.sim_time} "
        f"--bottleneck_bw_mbps={args.bottleneck_mbps} "
        f"--bottleneck_delay_ms={args.bottleneck_delay_ms}"
    )
    return ["./waf", "--run", run_string]


def eval_cmd(args, datasets: list[RunSpec], output_json: str) -> list[str]:
    cmd = [
        sys.executable,
        os.path.join(args.ns3_dir, "scripts", "freqccv3_window_overload_eval.py"),
    ]
    for spec in datasets:
        cmd.extend(
            [
                "--dataset",
                (
                    f"{spec.dataset_name}|manual|{spec.trace_dir}|{spec.prefix}|"
                    f"{args.oracle_flow_count}|freqccv3_2flow_calibration_bottleneck_queue.txt|"
                    f"{args.bottleneck_mbps}"
                ),
            ]
        )
    cmd.extend(
        [
            "--rate-window-mult",
            str(args.rate_window_mult),
            "--overlap",
            str(args.overlap),
            "--nfft-mult",
            str(args.nfft_mult),
            "--search-gate-low",
            str(args.search_gate_low),
            "--search-gate-high",
            str(args.search_gate_high),
            "--close-tols",
            args.close_tols,
            "--rtt-below-ratio",
            str(args.rtt_below_ratio),
            "--oracle-qfrac-threshold",
            str(args.oracle_qfrac_threshold),
            "--center-mode",
            args.center_mode,
            "--center-value",
            str(args.center_value),
            "--causal-mode",
            args.causal_mode,
            "--pre-bottleneck-prop-ms",
            str(args.pre_bottleneck_prop_ms),
            "--causal-quantile-low",
            str(args.causal_quantile_low),
            "--causal-quantile-high",
            str(args.causal_quantile_high),
            "--causal-pad-ms",
            str(args.causal_pad_ms),
            "--causal-pad-window-frac",
            str(args.causal_pad_window_frac),
            "--tolerant-match-rtt-frac",
            str(args.tolerant_match_rtt_frac),
            "--output",
            output_json,
        ]
    )
    return cmd


def best_eval_result(eval_payload: dict) -> dict:
    results = eval_payload.get("results", [])
    if not results:
        return {}
    return max(
        results,
        key=lambda item: (
            item["neighbor_overall"]["neighbor_balanced_accuracy"],
            item["neighbor_overall"]["hit_nonover_rate"],
            item["tolerant_overall"]["balanced_accuracy"],
            item["overall"]["balanced_accuracy"],
            -item["close_tol"],
        ),
    )


def main():
    args = parse_args()
    ensure_dir(args.output_root)
    run_specs = make_run_specs(args)

    failures = []
    if not args.skip_runs:
        for spec in run_specs:
            ensure_dir(spec.trace_dir)
            cmd = scenario_cmd(args, spec)
            result = run_command(cmd, cwd=args.ns3_dir)
            stdout_path = os.path.join(spec.trace_dir, "run.stdout.txt")
            stderr_path = os.path.join(spec.trace_dir, "run.stderr.txt")
            with open(stdout_path, "w", encoding="utf-8") as handle:
                handle.write(result.stdout)
            with open(stderr_path, "w", encoding="utf-8") as handle:
                handle.write(result.stderr)
            if result.returncode != 0:
                failures.append(
                    {
                        "duration_mult": spec.duration_mult,
                        "bg_rate_mbps": spec.bg_rate_mbps,
                        "returncode": result.returncode,
                        "stdout": stdout_path,
                        "stderr": stderr_path,
                    }
                )
                if not args.keep-going:
                    break

    if failures and not args.keep-going:
        summary_path = os.path.join(args.output_root, "failures.json")
        with open(summary_path, "w", encoding="utf-8") as handle:
            json.dump(failures, handle, indent=2)
        print(f"Scenario run failed; details saved to {summary_path}", file=sys.stderr)
        return 1

    duration_groups: dict[float, list[RunSpec]] = {}
    for spec in run_specs:
        duration_groups.setdefault(spec.duration_mult, []).append(spec)

    duration_summaries = []
    for duration_mult, specs in sorted(duration_groups.items(), key=lambda item: item[0]):
        duration_values_rtt = []
        per_dataset_duration = {}
        usable_specs = []
        for spec in specs:
            qdelay_path = os.path.join(spec.trace_dir, spec.prefix + "_qdelay.txt")
            upphase_path = os.path.join(spec.trace_dir, spec.prefix + "_upphase.txt")
            min_rtt_s = infer_min_rtt_s_from_qdelay(qdelay_path)
            up_durations_s = load_upphase_durations_rtt(upphase_path)
            if min_rtt_s is None or min_rtt_s <= 0.0 or not up_durations_s:
                per_dataset_duration[spec.dataset_name] = {
                    "bg_rate_mbps": spec.bg_rate_mbps,
                    "min_rtt_s": min_rtt_s,
                    "up_duration_rtt": summarize([]),
                }
                continue
            up_duration_rtt = [value / min_rtt_s for value in up_durations_s]
            duration_values_rtt.extend(up_duration_rtt)
            per_dataset_duration[spec.dataset_name] = {
                "bg_rate_mbps": spec.bg_rate_mbps,
                "min_rtt_s": min_rtt_s,
                "up_duration_rtt": summarize(up_duration_rtt),
            }
            usable_specs.append(spec)

        eval_output = os.path.join(args.output_root, f"dur_{safe_tag(duration_mult)}_eval.json")
        eval_payload = {}
        if usable_specs:
            cmd = eval_cmd(args, usable_specs, eval_output)
            result = run_command(cmd, cwd=os.getcwd())
            if result.returncode != 0:
                eval_payload = {
                    "error": {
                        "returncode": result.returncode,
                        "stdout": result.stdout,
                        "stderr": result.stderr,
                    }
                }
            else:
                with open(eval_output, "r", encoding="utf-8") as handle:
                    eval_payload = json.load(handle)
        duration_summaries.append(
            {
                "duration_mult_target": duration_mult,
                "up_duration_rtt_overall": summarize(duration_values_rtt),
                "up_duration_rtt_per_dataset": per_dataset_duration,
                "eval_output": eval_output if os.path.exists(eval_output) else None,
                "eval_best": best_eval_result(eval_payload) if eval_payload.get("results") else None,
                "eval_payload": eval_payload if "error" in eval_payload else None,
            }
        )

    summary = {
        "config": {
            "durations": parse_csv_floats(args.durations),
            "bg_rates": parse_csv_floats(args.bg_rates),
            "sim_time": args.sim_time,
            "probe_freq_hz": args.probe_freq_hz,
            "probe_amp_mode": args.probe_amp_mode,
            "queue_state": args.queue_state,
            "bottleneck_mbps": args.bottleneck_mbps,
            "bottleneck_delay_ms": args.bottleneck_delay_ms,
            "rate_window_mult": args.rate_window_mult,
            "overlap": args.overlap,
            "nfft_mult": args.nfft_mult,
            "search_gate_low": args.search_gate_low,
            "search_gate_high": args.search_gate_high,
            "close_tols": args.close_tols,
            "rtt_below_ratio": args.rtt_below_ratio,
            "oracle_qfrac_threshold": args.oracle_qfrac_threshold,
            "center_mode": args.center_mode,
            "center_value": args.center_value,
            "causal_mode": args.causal_mode,
            "causal_quantile_low": args.causal_quantile_low,
            "causal_quantile_high": args.causal_quantile_high,
            "causal_pad_ms": args.causal_pad_ms,
            "causal_pad_window_frac": args.causal_pad_window_frac,
            "tolerant_match_rtt_frac": args.tolerant_match_rtt_frac,
            "pre_bottleneck_prop_ms": args.pre_bottleneck_prop_ms,
            "oracle_flow_count": args.oracle_flow_count,
        },
        "failures": failures,
        "durations": duration_summaries,
    }
    summary_path = os.path.join(args.output_root, "duration_sweep_summary.json")
    with open(summary_path, "w", encoding="utf-8") as handle:
        json.dump(summary, handle, indent=2)

    for item in duration_summaries:
        best = item["eval_best"]
        up_summary = item["up_duration_rtt_overall"]
        if best is None:
            print(
                f"target={item['duration_mult_target']:.2f} "
                f"up_med_rtt={up_summary['median']} "
                "eval=unavailable"
            )
            continue
        neighbor = best["neighbor_overall"]
        print(
            f"target={item['duration_mult_target']:.2f} "
            f"up_med_rtt={up_summary['median']:.3f} "
            f"up_p10={up_summary['p10']:.3f} "
            f"up_p90={up_summary['p90']:.3f} "
            f"tol={best['close_tol']:.2f} "
            f"nbr_bal_acc={neighbor['neighbor_balanced_accuracy']:.4f} "
            f"hit_nonover={neighbor['hit_nonover_rate']:.4f} "
            f"clean_over={neighbor['clean_over_rate']:.4f} "
            f"support_pred={neighbor['supported_pred_nonover_rate']:.4f}"
        )

    print(f"\nSaved duration sweep summary to {summary_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
