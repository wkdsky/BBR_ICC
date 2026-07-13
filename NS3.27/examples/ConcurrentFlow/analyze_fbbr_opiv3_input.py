#!/usr/bin/env python3
"""Audit F-BBR OPI v3 input realization without weakening production gates."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np
import pandas as pd


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("run_dirs", nargs="+", type=Path)
    parser.add_argument("--output-dir", required=True, type=Path)
    args = parser.parse_args()
    output = args.output_dir.resolve()
    output.mkdir(parents=True, exist_ok=True)

    frames: list[pd.DataFrame] = []
    for run_dir in args.run_dirs:
        for path in sorted(run_dir.resolve().glob("flow*_fbbr_opiv2_blocks.csv")):
            frame = pd.read_csv(path)
            frame["run_dir"] = str(run_dir)
            frame["flow_file"] = path.name
            frames.append(frame)
    if not frames:
        parser.error("no production block CSVs found")
    windows = pd.concat(frames, ignore_index=True)
    # An asynchronous hard-loss abort has no complete excitation window and is
    # safety evidence, not an eligible input-realization denominator.
    eligible = windows[
        ~windows.invalid_reason.astype(str).str.contains(
            "hard_congestion_window_abort", na=False
        )
    ].copy()
    ratio = pd.to_numeric(eligible.realized_amplitude_ratio, errors="coerce")
    coherence = pd.to_numeric(eligible.input_cycle_coherence, errors="coerce")
    error = (ratio - 1.0).abs()
    passed = ratio.between(0.75, 1.25) & coherence.ge(0.85)
    eligible["input_realization_pass"] = passed
    eligible["realized_amplitude_error"] = error
    eligible.to_csv(output / "input_window_audit.csv", index=False)

    metrics = {
        "eligible_windows": int(len(eligible)),
        "input_realization_pass_rate": float(passed.mean()),
        "median_realized_amplitude_error": float(error.median()),
        "p95_realized_amplitude_error": float(error.quantile(0.95)),
        "median_input_coherence": float(coherence.median()),
    }
    checks = {
        "pass_rate_ge_90pct": metrics["input_realization_pass_rate"] >= 0.90,
        "median_error_le_10pct": metrics["median_realized_amplitude_error"] <= 0.10,
        "p95_error_le_25pct": metrics["p95_realized_amplitude_error"] <= 0.25,
    }
    metrics["acceptance_checks"] = checks
    metrics["conclusion"] = (
        "PASS" if all(checks.values()) else "FAIL_INPUT_NOT_REALIZED"
    )
    (output / "input_realization_summary.json").write_text(
        json.dumps(metrics, indent=2) + "\n"
    )
    lines = [
        "# F-BBR OPI v3 Phase A Input Realization",
        "",
        f"Eligible windows: {metrics['eligible_windows']}",
        f"Pass rate: {metrics['input_realization_pass_rate']:.6f}",
        f"Median amplitude error: {metrics['median_realized_amplitude_error']:.6f}",
        f"P95 amplitude error: {metrics['p95_realized_amplitude_error']:.6f}",
        f"Median input coherence: {metrics['median_input_coherence']:.6f}",
        "",
        "## Acceptance",
        "",
        *[f"- {'PASS' if ok else 'FAIL'}: {name}" for name, ok in checks.items()],
        "",
        "## Conclusion",
        "",
        metrics["conclusion"],
    ]
    (output / "PHASE_A_INPUT_REALIZATION_REPORT.md").write_text(
        "\n".join(lines) + "\n"
    )
    print(json.dumps(metrics, indent=2))
    return 0 if all(checks.values()) else 2


if __name__ == "__main__":
    raise SystemExit(main())
