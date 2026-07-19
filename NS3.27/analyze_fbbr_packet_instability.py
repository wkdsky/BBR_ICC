#!/usr/bin/env python3
"""Validate FBBR TrustedBw selection and pacing invariants in trace trees."""

import argparse
import csv
import math
from pathlib import Path


REQUIRED_GATE_COLUMNS = {
    "current_native_bw_bps",
    "trusted_bw_bps",
    "trusted_bw_valid",
    "trusted_bw_source",
    "trusted_bw_cruise_id",
    "trusted_bw_fresh",
    "trusted_bw_application_valid",
    "drate_spectral_integrity_score",
    "srtt_spectral_integrity_score",
    "joint_spectral_integrity_score",
    "drate_spectral_gate_pass",
    "srtt_spectral_gate_pass",
    "dual_signal_spectral_gate_pass",
    "limiting_spectral_signal",
    "pacing_base_bw_bps",
    "pacing_base_source",
    "phase_pacing_gain",
    "final_pacing_rate_bps",
}


def as_bool(value):
    return str(value).strip().lower() in {"1", "true", "yes"}


def as_float(value):
    try:
        return float(value)
    except (TypeError, ValueError):
        return math.nan


def close(lhs, rhs):
    return math.isfinite(lhs) and math.isfinite(rhs) and abs(lhs - rhs) <= max(
        2.0, 1e-8 * max(abs(lhs), abs(rhs), 1.0)
    )


def read_rows(path):
    with path.open(encoding="utf-8", errors="replace", newline="") as handle:
        reader = csv.DictReader(handle)
        rows = list(reader)
        return set(reader.fieldnames or []), rows


def check_scores(path, line_no, row, violations):
    drate = as_float(row.get("drate_spectral_integrity_score"))
    srtt = as_float(row.get("srtt_spectral_integrity_score"))
    joint = as_float(row.get("joint_spectral_integrity_score"))
    if not close(joint, min(drate, srtt)):
        violations.append(f"{path}:{line_no}: joint spectral score is not min(drate, srtt)")
    drate_gate = as_bool(row.get("drate_spectral_gate_pass"))
    srtt_gate = as_bool(row.get("srtt_spectral_gate_pass"))
    dual_gate = as_bool(row.get("dual_signal_spectral_gate_pass"))
    if dual_gate != (drate_gate and srtt_gate):
        violations.append(f"{path}:{line_no}: dual gate differs from drate_gate AND srtt_gate")
    expected_limit = "EQUAL"
    if drate < srtt:
        expected_limit = "DRATE"
    elif srtt < drate:
        expected_limit = "SRTT"
    if row.get("limiting_spectral_signal") != expected_limit:
        violations.append(f"{path}:{line_no}: incorrect limiting spectral signal")


def validate_trace_tree(results_dir):
    violations = []
    checked = {"gate": 0, "window": 0, "summary": 0}

    for path in results_dir.rglob("*_freq_gate_trace.csv"):
        fields, rows = read_rows(path)
        missing = REQUIRED_GATE_COLUMNS - fields
        if missing:
            violations.append(f"{path}: missing columns: {', '.join(sorted(missing))}")
            continue
        checked["gate"] += 1
        for line_no, row in enumerate(rows, 2):
            check_scores(path, line_no, row, violations)
            trusted_valid = as_bool(row.get("trusted_bw_valid"))
            trusted_bps = as_float(row.get("trusted_bw_bps"))
            source = row.get("trusted_bw_source")
            if not trusted_valid and (trusted_bps != 0.0 or source != "NONE"):
                violations.append(f"{path}:{line_no}: invalid TrustedBw is not zero/NONE")
            if row.get("row_type") != "pacing":
                continue
            base_source = row.get("pacing_base_source")
            base_bps = as_float(row.get("pacing_base_bw_bps"))
            native_bps = as_float(row.get("current_native_bw_bps"))
            gain = as_float(row.get("phase_pacing_gain"))
            final_bps = as_float(row.get("final_pacing_rate_bps"))
            phase = row.get("trusted_bw_application_phase")
            if as_bool(row.get("is_cruise")) and base_source != "NATIVE_BBR":
                violations.append(f"{path}:{line_no}: CRUISE did not use Native BBR baseline")
            if base_source == "TRUSTED_BW":
                if phase not in {"REFILL", "UP", "DOWN"}:
                    violations.append(f"{path}:{line_no}: TrustedBw used outside REFILL/UP/DOWN")
                if not (trusted_valid and as_bool(row.get("trusted_bw_fresh")) and
                        as_bool(row.get("trusted_bw_application_valid"))):
                    violations.append(f"{path}:{line_no}: TrustedBw baseline lacks fresh valid application")
                if not close(base_bps, trusted_bps):
                    violations.append(f"{path}:{line_no}: pacing baseline differs from TrustedBw")
                if not close(final_bps, gain * base_bps):
                    violations.append(f"{path}:{line_no}: TrustedBw pacing is not gain times baseline")
            elif base_source == "NATIVE_BBR":
                if math.isfinite(native_bps) and native_bps > 0 and not close(base_bps, native_bps):
                    violations.append(f"{path}:{line_no}: native pacing baseline differs from NativeBw")
            else:
                violations.append(f"{path}:{line_no}: unknown pacing baseline source")

    for path in results_dir.rglob("*_cruise_full_load_quality.csv"):
        fields, rows = read_rows(path)
        required = {
            "drate_spectral_integrity_score",
            "srtt_spectral_integrity_score",
            "joint_spectral_integrity_score",
            "drate_spectral_gate_pass",
            "srtt_spectral_gate_pass",
            "dual_signal_spectral_gate_pass",
            "limiting_spectral_signal",
        }
        missing = required - fields
        if missing:
            violations.append(f"{path}: missing columns: {', '.join(sorted(missing))}")
            continue
        checked["window"] += 1
        for line_no, row in enumerate(rows, 2):
            check_scores(path, line_no, row, violations)
            if as_bool(row.get("is_full_load_candidate")) and not as_bool(
                row.get("dual_signal_spectral_gate_pass")
            ):
                violations.append(f"{path}:{line_no}: candidate bypassed dual-signal gate")

    for path in results_dir.rglob("*_cruise_best_full_load_window.csv"):
        fields, rows = read_rows(path)
        required = {
            "best_trusted_bw",
            "best_trusted_bw_source",
            "trusted_bw_valid",
            "trusted_bw_fresh",
            "trusted_bw_application_valid",
            "drate_spectral_integrity_score",
            "srtt_spectral_integrity_score",
            "joint_spectral_integrity_score",
            "drate_spectral_gate_pass",
            "srtt_spectral_gate_pass",
            "dual_signal_spectral_gate_pass",
            "limiting_spectral_signal",
        }
        missing = required - fields
        if missing:
            violations.append(f"{path}: missing columns: {', '.join(sorted(missing))}")
            continue
        checked["summary"] += 1
        for line_no, row in enumerate(rows, 2):
            check_scores(path, line_no, row, violations)
            valid = as_bool(row.get("trusted_bw_valid"))
            if valid and not as_bool(row.get("dual_signal_spectral_gate_pass")):
                violations.append(f"{path}:{line_no}: published TrustedBw bypassed dual gate")
            if valid and as_float(row.get("best_trusted_bw")) <= 0:
                violations.append(f"{path}:{line_no}: valid TrustedBw is not positive")
            if not valid and (
                as_float(row.get("best_trusted_bw")) != 0.0 or
                row.get("best_trusted_bw_source") != "NONE"
            ):
                violations.append(f"{path}:{line_no}: invalid TrustedBw is not zero/NONE")

    return checked, violations


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--results-dir", type=Path, required=True)
    args, _ = parser.parse_known_args()
    checked, violations = validate_trace_tree(args.results_dir)
    report = [
        "# TrustedBw Architecture Validation",
        "",
        f"- Gate files checked: {checked['gate']}",
        f"- Window files checked: {checked['window']}",
        f"- Summary files checked: {checked['summary']}",
        f"- Violations: {len(violations)}",
        "",
    ]
    report.extend(f"- {item}" for item in violations)
    args.results_dir.mkdir(parents=True, exist_ok=True)
    (args.results_dir / "trusted_bw_architecture_validation.md").write_text(
        "\n".join(report) + "\n", encoding="utf-8"
    )
    print("\n".join(report))
    return 1 if violations else 0


if __name__ == "__main__":
    raise SystemExit(main())
