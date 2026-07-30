#!/usr/bin/env python3
"""Generate a clearly labeled educational counterfactual delivery-rate trace."""

import argparse
from bisect import bisect_left, bisect_right
import json
import math
import re
from pathlib import Path
from typing import Dict, List, Tuple


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", required=True, help="Original raw receive-rate trace.")
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--flow-id", type=int, default=1)
    parser.add_argument("--synthetic-start-s", type=float, required=True)
    parser.add_argument("--up-end-s", type=float, required=True)
    parser.add_argument("--down-end-s", type=float, required=True)
    parser.add_argument("--end-s", type=float, required=True)
    parser.add_argument("--baseline-mbps", type=float, required=True)
    parser.add_argument("--reference-up-start-s", type=float, required=True)
    parser.add_argument("--reference-up-end-s", type=float, required=True)
    parser.add_argument("--reference-down-end-s", type=float, required=True)
    parser.add_argument("--reference-cruise-end-s", type=float, required=True)
    return parser.parse_args()


def load_trace(path: Path) -> List[Tuple[float, float]]:
    rows: List[Tuple[float, float]] = []
    with path.open("r", encoding="utf-8", errors="replace") as fh:
        for line in fh:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            fields = re.split(r"\s+", line)
            if len(fields) < 2:
                continue
            rows.append((float(fields[0]), float(fields[1]) / 1000.0))
    return rows


def percentile(values: List[float], q: float) -> float:
    ordered = sorted(values)
    if not ordered:
        return math.nan
    position = (len(ordered) - 1) * q / 100.0
    lower = int(math.floor(position))
    upper = int(math.ceil(position))
    if lower == upper:
        return ordered[lower]
    fraction = position - lower
    return ordered[lower] * (1.0 - fraction) + ordered[upper] * fraction


def values_in_window(
    rows: List[Tuple[float, float]], start_s: float, end_s: float
) -> List[float]:
    return [value for time_s, value in rows if start_s <= time_s < end_s]


def quantile_map(
    value: float,
    source_values: List[float],
    target_values: List[float],
    target_shift_mbps: float,
) -> float:
    source_low = percentile(source_values, 5.0)
    source_high = percentile(source_values, 95.0)
    target_low = percentile(target_values, 5.0) + target_shift_mbps
    target_high = percentile(target_values, 95.0) + target_shift_mbps
    scale = (target_high - target_low) / max(source_high - source_low, 1e-9)
    return target_low + (value - source_low) * scale


def smoothstep(value: float) -> float:
    clipped = min(1.0, max(0.0, value))
    return clipped * clipped * (3.0 - 2.0 * clipped)


def blend(left: float, right: float, weight: float) -> float:
    return left * (1.0 - weight) + right * weight


def interpolate_series(rows: List[Tuple[float, float]], time_s: float) -> float:
    if not rows:
        return math.nan
    times = [item[0] for item in rows]
    index = bisect_right(times, time_s)
    if index <= 0:
        return rows[0][1]
    if index >= len(rows):
        return rows[-1][1]
    left_time, left_value = rows[index - 1]
    right_time, right_value = rows[index]
    weight = (time_s - left_time) / max(right_time - left_time, 1e-12)
    return blend(left_value, right_value, weight)


def detrended_residuals(
    rows: List[Tuple[float, float]],
    start_s: float,
    end_s: float,
    half_width_s: float,
) -> Dict[float, float]:
    selected = [(time_s, value) for time_s, value in rows if start_s <= time_s < end_s]
    times = [time_s for time_s, _ in selected]
    values = [value for _, value in selected]
    prefix = [0.0]
    for value in values:
        prefix.append(prefix[-1] + value)
    residuals: Dict[float, float] = {}
    for time_s, value in selected:
        left = bisect_left(times, time_s - half_width_s)
        right = bisect_right(times, time_s + half_width_s)
        local_mean = (prefix[right] - prefix[left]) / max(1, right - left)
        residuals[time_s] = value - local_mean
    return residuals


def main() -> int:
    args = parse_args()
    input_path = Path(args.input).resolve()
    output_dir = Path(args.output_dir).resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    rows = load_trace(input_path)

    reference_up = values_in_window(
        rows, args.reference_up_start_s, args.reference_up_end_s
    )
    reference_down = values_in_window(
        rows, args.reference_up_end_s, args.reference_down_end_s
    )
    reference_down_rows = [
        (time_s, value)
        for time_s, value in rows
        if args.reference_up_end_s <= time_s < args.reference_down_end_s
    ]
    reference_cruise = values_in_window(
        rows, args.reference_down_end_s, args.reference_cruise_end_s
    )
    target_up = values_in_window(rows, args.synthetic_start_s, args.up_end_s)
    target_down = values_in_window(rows, args.up_end_s, args.down_end_s)
    target_cruise = values_in_window(rows, args.down_end_s, args.end_s)

    reference_cruise_center = percentile(reference_cruise, 50.0)
    reference_shift = args.baseline_mbps - reference_cruise_center
    residual_half_width_s = 0.10
    reference_residuals = detrended_residuals(
        rows,
        args.reference_down_end_s,
        args.reference_cruise_end_s,
        residual_half_width_s,
    )
    target_residuals = detrended_residuals(
        rows,
        args.down_end_s,
        args.end_s,
        residual_half_width_s,
    )
    reference_steady_residuals = [
        value
        for time_s, value in reference_residuals.items()
        if time_s >= args.reference_down_end_s + 0.15
    ]
    target_steady_residuals = [
        value
        for time_s, value in target_residuals.items()
        if time_s >= args.down_end_s + 0.15
    ]
    reference_residual_span = percentile(
        reference_steady_residuals, 95.0
    ) - percentile(reference_steady_residuals, 5.0)
    target_residual_span = percentile(target_steady_residuals, 95.0) - percentile(
        target_steady_residuals, 5.0
    )
    calculated_cruise_residual_scale = reference_residual_span / max(
        target_residual_span, 1e-9
    )
    cruise_residual_scale = min(calculated_cruise_residual_scale, 0.85)
    cruise_envelope_hold_s = 0.015
    cruise_envelope_tau_s = 0.04
    cruise_residual_ramp_s = 0.18

    def mapped_up(value: float) -> float:
        return quantile_map(value, target_up, reference_up, reference_shift)

    def mapped_down(time_s: float) -> float:
        progress = (time_s - args.up_end_s) / max(
            args.down_end_s - args.up_end_s, 1e-12
        )
        progress = min(1.0, max(0.0, progress))
        warped_progress = progress + 0.012 * math.sin(math.pi * progress)
        reference_time = args.reference_up_end_s + warped_progress * (
            args.reference_down_end_s - args.reference_up_end_s
        )
        reference_value = interpolate_series(reference_down_rows, reference_time)
        shape_adjustment = 0.045 * math.sin(2.0 * math.pi * progress) * math.sin(
            math.pi * progress
        )
        return reference_value + reference_shift + shape_adjustment

    down_exit_level = mapped_down(args.down_end_s)

    def mapped_cruise_base(time_s: float) -> float:
        elapsed = max(0.0, time_s - args.down_end_s)
        decay_elapsed = max(0.0, elapsed - cruise_envelope_hold_s)
        envelope = args.baseline_mbps + (
            down_exit_level - args.baseline_mbps
        ) * math.exp(-decay_elapsed / cruise_envelope_tau_s)
        residual_weight = smoothstep(elapsed / cruise_residual_ramp_s)
        residual = target_residuals.get(time_s, 0.0) * cruise_residual_scale
        return envelope + residual * residual_weight

    lowest_cruise_time = min(
        (
            (time_s, mapped_cruise_base(time_s))
            for time_s, _ in rows
            if args.down_end_s <= time_s <= args.end_s
        ),
        key=lambda item: item[1],
    )[0]
    low_cluster_lift_mbps = 0.75
    low_cluster_width_s = 0.05

    def mapped_cruise(time_s: float) -> float:
        base_value = mapped_cruise_base(time_s)
        distance = (time_s - lowest_cruise_time) / low_cluster_width_s
        lift = low_cluster_lift_mbps * math.exp(-0.5 * distance * distance)
        return base_value + lift

    blend_width_s = 0.015
    generated: List[Dict[str, object]] = []
    for time_s, original_mbps in rows:
        if time_s > args.end_s:
            break
        synthetic_mbps = original_mbps
        phase = "original"
        modified = False
        if time_s >= args.synthetic_start_s:
            modified = True
            if time_s < args.up_end_s:
                phase = "UP"
                mapped = mapped_up(original_mbps)
                start_weight = smoothstep(
                    (time_s - args.synthetic_start_s) / blend_width_s
                )
                synthetic_mbps = blend(original_mbps, mapped, start_weight)
            elif time_s < args.down_end_s:
                phase = "DOWN"
                previous = mapped_up(original_mbps)
                current = mapped_down(time_s)
                weight = smoothstep(
                    (time_s - args.up_end_s) / blend_width_s
                )
                synthetic_mbps = blend(previous, current, weight)
            else:
                phase = "CRUISE"
                previous = mapped_down(time_s)
                current = mapped_cruise(time_s)
                weight = smoothstep(
                    (time_s - args.down_end_s) / blend_width_s
                )
                synthetic_mbps = blend(previous, current, weight)
        generated.append(
            {
                "time_s": time_s,
                "original_delivery_rate_mbps": original_mbps,
                "synthetic_delivery_rate_mbps": synthetic_mbps,
                "phase": phase,
                "synthetic_modified": modified,
            }
        )

    stem = (
        f"flow{args.flow_id}_synthetic_counterfactual_"
        f"{args.synthetic_start_s:g}to{args.end_s:g}"
    ).replace(".", "p")
    csv_path = output_dir / f"{stem}.csv"
    raw_path = output_dir / f"synthetic_flow{args.flow_id}_counterfactual_recvrate_raw.txt"
    metadata_path = output_dir / f"{stem}.json"

    with csv_path.open("w", encoding="utf-8", newline="") as fh:
        fh.write(
            "time_s,original_delivery_rate_mbps,synthetic_delivery_rate_mbps,"
            "phase,synthetic_modified\n"
        )
        for row in generated:
            fh.write(
                f"{row['time_s']:.7f},{row['original_delivery_rate_mbps']:.6f},"
                f"{row['synthetic_delivery_rate_mbps']:.6f},{row['phase']},"
                f"{str(row['synthetic_modified']).lower()}\n"
            )

    with raw_path.open("w", encoding="utf-8") as fh:
        fh.write("# SYNTHETIC EDUCATIONAL COUNTERFACTUAL - NOT AN ORIGINAL NS-3 TRACE\n")
        fh.write("#time(s)\tdelivery_rate_sample(kbps)\n")
        for row in generated:
            fh.write(
                f"{row['time_s']:.7f}\t"
                f"{float(row['synthetic_delivery_rate_mbps']) * 1000.0:.6f}\n"
            )

    metadata = {
        "provenance": "synthetic educational counterfactual; not original simulation output",
        "input_trace": str(input_path),
        "output_csv": str(csv_path),
        "output_raw_trace": str(raw_path),
        "synthetic_start_s": args.synthetic_start_s,
        "end_s": args.end_s,
        "target_baseline_mbps": args.baseline_mbps,
        "method": (
            "UP/DOWN phasewise 5th-95th percentile affine matching; CRUISE "
            "decaying center envelope plus scaled detrended periodic residual"
        ),
        "reference_probe_bw": {
            "up": [args.reference_up_start_s, args.reference_up_end_s],
            "down": [args.reference_up_end_s, args.reference_down_end_s],
            "cruise": [args.reference_down_end_s, args.reference_cruise_end_s],
        },
        "synthetic_probe_bw": {
            "up": [args.synthetic_start_s, args.up_end_s],
            "down": [args.up_end_s, args.down_end_s],
            "cruise": [args.down_end_s, args.end_s],
        },
        "reference_cruise_center_mbps": reference_cruise_center,
        "reference_shift_mbps": reference_shift,
        "cruise_down_exit_level_mbps": down_exit_level,
        "down_model": "time-normalized ProbeBW 2 DOWN trace with small smooth warp",
        "calculated_cruise_residual_scale": calculated_cruise_residual_scale,
        "cruise_residual_scale": cruise_residual_scale,
        "cruise_residual_half_width_s": residual_half_width_s,
        "cruise_envelope_hold_s": cruise_envelope_hold_s,
        "cruise_envelope_tau_s": cruise_envelope_tau_s,
        "cruise_residual_ramp_s": cruise_residual_ramp_s,
        "low_cluster_center_s": lowest_cruise_time,
        "low_cluster_lift_mbps": low_cluster_lift_mbps,
        "low_cluster_width_s": low_cluster_width_s,
        "boundary_blend_width_s": blend_width_s,
    }
    metadata_path.write_text(json.dumps(metadata, indent=2) + "\n", encoding="utf-8")

    print(f"CSV: {csv_path}")
    print(f"RAW: {raw_path}")
    print(f"METADATA: {metadata_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
