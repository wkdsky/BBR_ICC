#!/usr/bin/env python3
"""
Plot BBRv2 vs FreqCCv4 raw delivery-rate samples, sampling windows, and
peak-triggered maxBw / CRUISE-exit TrustedBw update trajectories.
"""

import argparse
import sys
from pathlib import Path
from typing import List, Optional, Tuple

from plot_freqccv4_debug import (
    delivery_compare_file_stem,
    infer_service_rate,
    plot_bbrv2_vs_freqccv4_delivery_rate,
)


def parse_args(argv: Optional[List[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Create a BBRv2 vs FreqCCv4 delivery-rate comparison with "
            "maxBw and TrustedBw update trajectories."
        )
    )
    parser.add_argument(
        "--freqccv4-run-dir",
        "--run-dir",
        dest="freqccv4_run_dir",
        required=True,
        help="FreqCCv4 run directory. Its parent is expected to contain the BBRv2 directory.",
    )
    parser.add_argument(
        "--start-s",
        type=float,
        required=True,
        help="Window start time in seconds.",
    )
    parser.add_argument(
        "--end-s",
        type=float,
        required=True,
        help="Window end time in seconds.",
    )
    parser.add_argument(
        "--output-dir",
        default="",
        help="Output directory. Defaults to FREQCCV4_RUN_DIR/debug_plots/delivery_rate_windows.",
    )
    parser.add_argument(
        "--service-rate",
        default="",
        help="Bottleneck service rate. Defaults to run config/comparison_config, then 100Mbps.",
    )
    parser.add_argument(
        "--flow-id",
        type=int,
        action="append",
        default=[],
        help="Only plot one flow id. Can be repeated. Defaults to all flows.",
    )
    parser.add_argument(
        "--highlight-start-s",
        type=float,
        default=None,
        help="Optional frequency-domain sampling window start time.",
    )
    parser.add_argument(
        "--highlight-end-s",
        type=float,
        default=None,
        help="Optional frequency-domain sampling window end time.",
    )
    parser.add_argument(
        "--highlight-window",
        type=float,
        nargs=2,
        action="append",
        default=[],
        metavar=("START_S", "END_S"),
        help="Frequency-domain sampling window. Can be repeated.",
    )
    parser.add_argument(
        "--highlight-value-mbps",
        type=float,
        action="append",
        default=[],
        help=(
            "TrustedBw selected from the corresponding --highlight-window; "
            "it is shown as being committed at that CRUISE exit."
        ),
    )
    parser.add_argument(
        "--highlight-cc",
        default="FreqCCv4",
        help="CC name to average in the highlight window. Default: FreqCCv4.",
    )
    parser.add_argument(
        "--highlight-label",
        default="TrustedBw",
        help=(
            "Text used for the highlighted-window mean annotation. "
            "Default: TrustedBw."
        ),
    )
    parser.add_argument(
        "--hide-maxbw",
        action="store_true",
        help="Do not annotate the BBRv2 maximum delivery-rate sample.",
    )
    parser.add_argument(
        "--maxbw-cycle-phase",
        choices=("refill", "up"),
        default="refill",
        help="ProbeBW phase whose consecutive starts delimit maxBw cycles.",
    )
    parser.add_argument(
        "--maxbw-cycle-count",
        type=int,
        default=1,
        help="Maximum number of complete ProbeBW cycles to annotate. Default: 1.",
    )
    parser.add_argument(
        "--initial-state-value-mbps",
        type=float,
        default=None,
        help="Initial maxBw/TrustedBw state value at the left plot boundary.",
    )
    parser.add_argument(
        "--relative-time-axis",
        action="store_true",
        help="Label x-axis ticks relative to the plot start as t0+n.",
    )
    parser.add_argument(
        "--probe-cycle-braces",
        action="store_true",
        help="Draw braces for complete UP-to-UP ProbeBW cycles below the x-axis.",
    )
    parser.add_argument(
        "--partial-probe-cycle-brace",
        action="store_true",
        help="Draw an open brace for the final incomplete ProbeBW cycle.",
    )
    parser.add_argument(
        "--include-partial-maxbw-cycle",
        action="store_true",
        help="Annotate maxBw in the final visible incomplete ProbeBW cycle.",
    )
    parser.add_argument(
        "--congestion-signal-window",
        type=float,
        nargs=2,
        default=None,
        metavar=("START_S", "END_S"),
        help=(
            "Optional congestion-signal interval. The interval is not drawn; "
            "when supplied, the third maxBw annotation receives the suffix "
            "'(packet-loss signal)'."
        ),
    )
    parser.add_argument(
        "--congestion-signal-label",
        default="packet-loss signal",
        help=(
            "Text appended in parentheses to the third maxBw annotation when "
            "--congestion-signal-window is supplied."
        ),
    )
    parser.add_argument(
        "--comparison-raw-file",
        default="",
        help="Optional replacement raw delivery-rate trace for the comparison CC.",
    )
    parser.add_argument(
        "--scenario-label",
        default="",
        help="Deprecated compatibility option; figure titles/subtitles are not rendered.",
    )
    parser.add_argument(
        "--synthetic-start-s",
        type=float,
        default=None,
        help="Deprecated compatibility option; no synthetic region is rendered.",
    )
    return parser.parse_args(argv)


def main(argv: Optional[List[str]] = None) -> int:
    args = parse_args(argv)
    if args.start_s < 0.0:
        print("--start-s must be >= 0", file=sys.stderr)
        return 2
    if args.end_s <= args.start_s:
        print("--end-s must be greater than --start-s", file=sys.stderr)
        return 2
    if args.maxbw_cycle_count < 1:
        print("--maxbw-cycle-count must be >= 1", file=sys.stderr)
        return 2
    if (args.highlight_start_s is None) != (args.highlight_end_s is None):
        print("--highlight-start-s and --highlight-end-s must be provided together", file=sys.stderr)
        return 2
    if (
        args.highlight_start_s is not None
        and args.highlight_end_s is not None
        and args.highlight_end_s <= args.highlight_start_s
    ):
        print("--highlight-end-s must be greater than --highlight-start-s", file=sys.stderr)
        return 2
    highlight_windows: List[Tuple[float, float]] = [
        (start_s, end_s) for start_s, end_s in args.highlight_window
    ]
    for start_s, end_s in highlight_windows:
        if end_s <= start_s:
            print("Each --highlight-window END_S must be greater than START_S", file=sys.stderr)
            return 2
    if args.highlight_value_mbps and len(args.highlight_value_mbps) != len(highlight_windows):
        print(
            "The number of --highlight-value-mbps arguments must match --highlight-window",
            file=sys.stderr,
        )
        return 2

    freqccv4_run_dir = Path(args.freqccv4_run_dir).resolve()
    if not freqccv4_run_dir.is_dir():
        print(f"FreqCCv4 run directory does not exist: {freqccv4_run_dir}", file=sys.stderr)
        return 2

    output_dir = (
        Path(args.output_dir).resolve()
        if args.output_dir
        else freqccv4_run_dir / "debug_plots" / "delivery_rate_windows"
    )
    output_dir.mkdir(parents=True, exist_ok=True)

    service_rate_bps = infer_service_rate(freqccv4_run_dir, args.service_rate)
    png_path = plot_bbrv2_vs_freqccv4_delivery_rate(
        output_dir,
        freqccv4_run_dir,
        service_rate_bps,
        args.end_s,
        start_s=args.start_s,
        selected_flow_ids=args.flow_id,
        highlight_start_s=args.highlight_start_s,
        highlight_end_s=args.highlight_end_s,
        highlight_cc=args.highlight_cc,
        highlight_label_prefix=args.highlight_label,
        highlight_windows=highlight_windows,
        highlight_values_mbps=args.highlight_value_mbps,
        show_bbrv2_maxbw=not args.hide_maxbw,
        comparison_cc_label=freqccv4_run_dir.name,
        maxbw_cycle_phase=f"probeBW_{args.maxbw_cycle_phase}",
        maxbw_cycle_count=args.maxbw_cycle_count,
        initial_state_value_mbps=args.initial_state_value_mbps,
        relative_time_axis=args.relative_time_axis,
        show_probe_cycle_braces=args.probe_cycle_braces,
        show_partial_probe_cycle_brace=args.partial_probe_cycle_brace,
        include_partial_maxbw_cycle=args.include_partial_maxbw_cycle,
        congestion_signal_window=(
            tuple(args.congestion_signal_window)
            if args.congestion_signal_window is not None
            else None
        ),
        congestion_signal_label=args.congestion_signal_label,
        comparison_raw_file=(
            Path(args.comparison_raw_file).resolve() if args.comparison_raw_file else None
        ),
        scenario_label=args.scenario_label,
        synthetic_start_s=args.synthetic_start_s,
    )
    if png_path is None:
        print(f"Failed to create delivery-rate plot. See {output_dir}", file=sys.stderr)
        return 1

    stem = delivery_compare_file_stem(
        args.start_s,
        args.end_s,
        args.flow_id,
        args.highlight_start_s,
        args.highlight_end_s,
        highlight_windows,
    )
    if args.comparison_raw_file:
        stem = f"{stem}_synthetic"
    csv_path = output_dir / f"{stem}.csv"
    print(f"PNG: {png_path}")
    print(f"CSV: {csv_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
