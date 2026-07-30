#!/usr/bin/env python3
"""Aggregate the dynamic Test 1 outputs and render Fig. 1(a)/(b)."""

from __future__ import annotations

import argparse
import math
from pathlib import Path
from typing import Iterable

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.ticker import MultipleLocator
import pandas as pd


ALGORITHMS = [
    "BBR-R",
    "oBBR",
    "BBRv2+",
    "CUBIC",
    "BBRv2-ideal",
    "BBRv2",
    "FBBR",
]
STAGE_COUNTS = [2, 4, 8, 16, 8, 4, 2]
STAGE_LABELS = [
    "N2_rise",
    "N4_rise",
    "N8_rise",
    "N16_peak",
    "N8_fall",
    "N4_fall",
    "N2_fall",
]
EXPECTED_IDEAL_EVENTS = sum(STAGE_COUNTS)
MAX_FLOWS = max(STAGE_COUNTS)
CAPACITY_BPS = 100_000_000.0
MINUTE_WINDOW_S = 60.0
COLORS = {
    "BBR-R": "#0072B2",
    "oBBR": "#D55E00",
    "BBRv2+": "#009E73",
    "CUBIC": "#CC79A7",
    "BBRv2-ideal": "#000000",
    "BBRv2": "#E69F00",
    "FBBR": "#56B4E9",
}
MARKERS = {
    "BBR-R": "o",
    "oBBR": "s",
    "BBRv2+": "^",
    "CUBIC": "D",
    "BBRv2-ideal": "P",
    "BBRv2": "v",
    "FBBR": "X",
}


def read_paths(manifest: Path, column: str) -> list[Path]:
    table = pd.read_csv(manifest)
    if column not in table.columns:
        raise ValueError(f"Manifest has no {column} column: {manifest}")
    paths: list[Path] = []
    for value in table[column].dropna():
        path = Path(str(value))
        if not path.exists():
            raise FileNotFoundError(f"Manifest entry does not exist: {path}")
        paths.append(path)
    return paths


def concat_csv(paths: Iterable[Path]) -> pd.DataFrame:
    tables = [pd.read_csv(path) for path in paths]
    if not tables:
        raise ValueError("No result CSV files were found")
    nonempty_tables = [table for table in tables if not table.empty]
    return pd.concat(nonempty_tables or tables[:1], ignore_index=True)


def truth(values: pd.Series) -> pd.Series:
    return values.astype(str).str.strip().str.lower().isin({"1", "true", "yes"})


def require_columns(frame: pd.DataFrame, columns: list[str], name: str) -> None:
    missing = [column for column in columns if column not in frame.columns]
    if missing:
        raise ValueError(f"{name} is missing columns: {', '.join(missing)}")


def add_derived_stage_metrics(stages: pd.DataFrame) -> pd.DataFrame:
    """Backfill metrics that are a linear transform of existing raw samples."""
    stages = stages.copy()
    if "mean_queue_delay_ms" not in stages.columns:
        require_columns(stages, ["mean_queue_bytes"], "stage metrics")
        stages["mean_queue_delay_ms"] = (
            stages["mean_queue_bytes"].astype(float) * 8.0 / CAPACITY_BPS * 1000.0
        )
    return stages


def configure_time_axis(axis: plt.Axes, simulation_time_s: float) -> None:
    """Use the shared minute axis: labeled every two minutes, ticked every minute."""
    duration_min = simulation_time_s / 60.0
    axis.set_xlim(0.0, duration_min)
    major_ticks = list(range(2, int(math.floor(duration_min)) + 1, 2))
    axis.set_xticks(major_ticks)
    axis.xaxis.set_minor_locator(MultipleLocator(1.0))
    axis.tick_params(axis="x", which="major", length=5)
    axis.tick_params(axis="x", which="minor", length=3, labelbottom=False)
    axis.set_xlabel("Simulation time (min)")


def validate_results(
    summary: pd.DataFrame,
    stages: pd.DataFrame,
    flows: pd.DataFrame,
    events: pd.DataFrame,
    minutes: pd.DataFrame,
    minute_flows: pd.DataFrame,
    expect_full: bool,
) -> pd.DataFrame:
    require_columns(
        summary,
        [
            "algorithm",
            "mode",
            "seed",
            "run_id",
            "simulation_time_s",
            "expected_ideal_up_events",
            "observed_ideal_up_events",
            "max_concurrent_up",
            "ideal_sequence_validation",
            "validation_pass",
        ],
        "run summary",
    )
    require_columns(
        stages,
        [
            "algorithm",
            "mode",
            "seed",
            "run_id",
            "stage_index",
            "stage_label",
            "active_flows",
        ],
        "stage metrics",
    )
    require_columns(
        flows,
        [
            "algorithm",
            "mode",
            "seed",
            "run_id",
            "stage_index",
            "flow_id",
        ],
        "flow metrics",
    )
    require_columns(
        events,
        [
            "algorithm",
            "mode",
            "seed",
            "run_id",
            "stage_index",
            "flow_id",
            "start_time_s",
            "end_time_s",
            "pre_other_up_count",
            "max_concurrent_up",
        ],
        "UP events",
    )
    require_columns(
        minutes,
        [
            "algorithm",
            "mode",
            "seed",
            "run_id",
            "minute_index",
            "window_start_s",
            "window_end_s",
            "sample_count",
            "mean_excess_inflight_bdp",
            "mean_queue_delay_ms",
            "aggregate_goodput_bps",
            "mean_flow_goodput_bps",
            "jain_fairness",
            "throughput_recorded",
        ],
        "minute metrics",
    )
    require_columns(
        minute_flows,
        [
            "algorithm",
            "mode",
            "seed",
            "run_id",
            "minute_index",
            "window_start_s",
            "window_end_s",
            "flow_id",
            "active_in_window",
            "active_duration_s",
            "received_bytes",
            "goodput_bps",
        ],
        "minute flow metrics",
    )

    checks: list[dict[str, object]] = []

    def add_check(name: str, passed: bool, detail: str) -> None:
        checks.append({"check": name, "passed": int(passed), "detail": detail})
        if not passed:
            raise ValueError(f"Validation failed: {name}: {detail}")

    add_check(
        "all_run_summaries_pass",
        truth(summary.validation_pass).all(),
        "Every run summary must report validation_pass=1.",
    )
    expected_stage = pd.DataFrame(
        {
            "stage_index": list(range(len(STAGE_COUNTS))),
            "stage_label": STAGE_LABELS,
            "active_flows": STAGE_COUNTS,
        }
    )
    run_keys = ["algorithm", "mode", "seed", "run_id"]
    for key, group in stages.groupby(run_keys, sort=False):
        observed = group.sort_values("stage_index")[
            ["stage_index", "stage_label", "active_flows"]
        ].reset_index(drop=True)
        add_check(
            f"dynamic_stage_sequence_{key}",
            observed.equals(expected_stage),
            "Expected 2,4,8,16,8,4,2 with the canonical stage labels.",
        )
        for _, stage in group.iterrows():
            stage_flows = flows[
                (flows.algorithm == stage.algorithm)
                & (flows["mode"] == stage["mode"])
                & (flows.seed == stage.seed)
                & (flows.run_id == stage.run_id)
                & (flows.stage_index == stage.stage_index)
            ]
            add_check(
                f"flow_rows_{key}_stage{int(stage.stage_index)}",
                len(stage_flows) == int(stage.active_flows),
                "Each stage must expose one flow-metrics row per active flow.",
            )

    for _, run in summary.iterrows():
        run_minutes = minutes[
            (minutes["algorithm"] == run["algorithm"])
            & (minutes["mode"] == run["mode"])
            & (minutes["seed"] == run["seed"])
            & (minutes["run_id"] == run["run_id"])
        ].sort_values("minute_index")
        expected_minutes = int(
            math.ceil(float(run["simulation_time_s"]) / MINUTE_WINDOW_S)
        )
        expected_indices = list(range(expected_minutes))
        add_check(
            f"minute_windows_{run['algorithm']}_{int(run['run_id'])}",
            len(run_minutes) == expected_minutes
            and run_minutes.minute_index.astype(int).tolist() == expected_indices
            and (run_minutes.sample_count.astype(int) > 0).all(),
            "Each run needs one populated row for every 60-second window.",
        )
        run_minute_flows = minute_flows[
            (minute_flows["algorithm"] == run["algorithm"])
            & (minute_flows["mode"] == run["mode"])
            & (minute_flows["seed"] == run["seed"])
            & (minute_flows["run_id"] == run["run_id"])
        ]
        expected_flow_rows = expected_minutes * MAX_FLOWS
        add_check(
            f"minute_flow_rows_{run['algorithm']}_{int(run['run_id'])}",
            len(run_minute_flows) == expected_flow_rows,
            "Each one-minute window must retain one trace row for every flow.",
        )
        expected_flow_ids = list(range(1, MAX_FLOWS + 1))
        for minute_index in expected_indices:
            minute_flow_rows = run_minute_flows[
                run_minute_flows.minute_index.astype(int) == minute_index
            ].sort_values("flow_id")
            add_check(
                f"minute_flow_ids_{run['algorithm']}_{int(run['run_id'])}_{minute_index}",
                minute_flow_rows.flow_id.astype(int).tolist() == expected_flow_ids,
                "Every minute trace must enumerate flow IDs 1..16 exactly once.",
            )
        add_check(
            f"minute_performance_values_{run['algorithm']}_{int(run['run_id'])}",
            run_minutes.aggregate_goodput_bps.astype(float).ge(0.0).all()
            and run_minutes.mean_flow_goodput_bps.astype(float).ge(0.0).all()
            and run_minutes.jain_fairness.astype(float).between(0.0, 1.0).all()
            and truth(run_minutes.throughput_recorded).all(),
            "Minute aggregate/per-flow goodput must be non-negative, Jain must be in [0, 1], and each boundary snapshot must execute.",
        )

    ideal_summary = summary[summary.algorithm == "BBRv2-ideal"]
    add_check(
        "ideal_run_present_once",
        len(ideal_summary) == 1,
        "Exactly one BBRv2-ideal run is required.",
    )
    if len(ideal_summary) == 1:
        row = ideal_summary.iloc[0]
        add_check(
            "ideal_mode",
            row["mode"] == "ideal",
            "BBRv2-ideal must be marked ideal.",
        )
        add_check(
            "ideal_event_count_summary",
            int(row.expected_ideal_up_events) == EXPECTED_IDEAL_EVENTS
            and int(row.observed_ideal_up_events) == EXPECTED_IDEAL_EVENTS,
            f"Expected {EXPECTED_IDEAL_EVENTS} sequential UP events.",
        )
        add_check(
            "ideal_summary_max_concurrent_up",
            int(row.max_concurrent_up) == 1,
            "The run summary must report one concurrent UP at most.",
        )
        add_check(
            "ideal_sequence_validation",
            bool(truth(pd.Series([row.ideal_sequence_validation])).iloc[0]),
            "The scenario must validate the ideal flow order.",
        )

    ideal_events = events[events.algorithm == "BBRv2-ideal"].copy()
    add_check(
        "ideal_event_rows",
        len(ideal_events) == EXPECTED_IDEAL_EVENTS,
        f"Expected {EXPECTED_IDEAL_EVENTS} BBRv2-ideal event rows.",
    )
    if not ideal_events.empty:
        add_check(
            "ideal_no_overlapping_up",
            (ideal_events.max_concurrent_up.astype(int) <= 1).all()
            and (ideal_events.pre_other_up_count.astype(int) == 0).all(),
            "Every ideal UP must begin with no other UP and remain non-overlapping.",
        )
        add_check(
            "ideal_event_duration",
            (ideal_events.end_time_s >= ideal_events.start_time_s).all(),
            "Every ideal UP event must have a non-negative duration.",
        )
        for index, count in enumerate(STAGE_COUNTS):
            stage_events = ideal_events[ideal_events.stage_index == index].sort_values(
                "start_time_s"
            )
            expected_flow_ids = list(range(1, count + 1))
            add_check(
                f"ideal_stage_{index}_count_and_order",
                len(stage_events) == count
                and stage_events.flow_id.astype(int).tolist() == expected_flow_ids,
                f"Stage {index} must admit flows 1..{count} in order.",
            )

    nonideal_events = events[events.algorithm != "BBRv2-ideal"]
    add_check(
        "nonideal_event_trace_empty",
        nonideal_events.empty,
        "Only BBRv2-ideal emits the experiment-specific sequential-UP trace.",
    )

    if expect_full:
        add_check(
            "full_algorithm_set",
            summary.algorithm.tolist() == ALGORITHMS,
            "The full matrix must retain the requested algorithm order.",
        )
        add_check(
            "full_run_count",
            len(summary) == len(ALGORITHMS),
            "The full matrix has one deterministic run per algorithm.",
        )
        add_check(
            "full_duration",
            (summary.simulation_time_s.astype(float) == 1800.0).all(),
            "Every full run must last 1800 simulated seconds.",
        )
        add_check(
            "full_minute_rows",
            len(minutes) == len(ALGORITHMS) * int(1800.0 / MINUTE_WINDOW_S),
            "The full matrix needs 30 one-minute rows per controller.",
        )
        add_check(
            "full_minute_flow_rows",
            len(minute_flows)
            == len(ALGORITHMS) * int(1800.0 / MINUTE_WINDOW_S) * MAX_FLOWS,
            "The full matrix needs 16 per-flow trace rows for every controller-minute.",
        )
        original = summary[summary.algorithm != "BBRv2-ideal"]
        add_check(
            "original_modes",
            (original["mode"] == "original").all(),
            "All controllers other than BBRv2-ideal must retain original mode.",
        )
    return pd.DataFrame(checks)


def write_aggregate_data(
    summary: pd.DataFrame,
    stages: pd.DataFrame,
    flows: pd.DataFrame,
    events: pd.DataFrame,
    minutes: pd.DataFrame,
    minute_flows: pd.DataFrame,
    summary_dir: Path,
) -> pd.DataFrame:
    summary.to_csv(summary_dir / "all_runs.csv", index=False)
    stages.to_csv(summary_dir / "all_stage_metrics.csv", index=False)
    flows.to_csv(summary_dir / "all_flow_metrics.csv", index=False)
    events.to_csv(summary_dir / "all_ideal_up_events.csv", index=False)
    minutes.to_csv(summary_dir / "all_minute_metrics.csv", index=False)
    minute_flows.to_csv(summary_dir / "all_minute_flow_metrics.csv", index=False)
    stages.sort_values(["algorithm", "stage_index"]).to_csv(
        summary_dir / "comparison_metrics.csv", index=False
    )

    aggregations = {
        "aggregate_goodput_bps": "mean",
        "utilization_pct": "mean",
        "jain_fairness": "mean",
        "mean_excess_inflight_bdp": "mean",
        "p95_excess_inflight_bdp": "mean",
        "max_excess_inflight_bdp": "max",
        "mean_aggregate_inflight_bytes": "mean",
        "p95_aggregate_inflight_bytes": "mean",
        "mean_queue_delay_ms": "mean",
        "p95_queue_delay_ms": "mean",
        "p99_queue_delay_ms": "mean",
        "max_queue_delay_ms": "max",
        "queue_drop_packets": "sum",
        "queue_drop_bytes": "sum",
        "mean_sum_pacing_bps": "mean",
        "mean_bandwidth_estimate_bps": "mean",
        "ideal_up_events": "sum",
        "theory_applicable_up_events": "sum",
        "mean_theory_error_pct": "mean",
        "max_abs_theory_error_pct": "max",
    }
    overall = (
        stages.groupby(["algorithm", "mode"], as_index=False)
        .agg(aggregations)
        .set_index("algorithm")
        .reindex(ALGORITHMS)
        .reset_index()
    )
    overall.to_csv(summary_dir / "overall_comparison_metrics.csv", index=False)
    return overall


def plot_fig1a(
    events: pd.DataFrame,
    figures_dir: Path,
    summary_dir: Path,
    simulation_time_s: float,
) -> None:
    data = events[events.algorithm == "BBRv2-ideal"].copy()
    data = data.sort_values("start_time_s").reset_index(drop=True)
    if len(data) != EXPECTED_IDEAL_EVENTS:
        raise ValueError("Fig. 1(a) requires all BBRv2-ideal UP events")
    data["time_min"] = data.start_time_s.astype(float) / 60.0
    data["measured_max_bw_mbps"] = data.max_bw_after_bps / 1e6
    data["theory_max_bw_mbps"] = data.theory_max_bw_bps / 1e6
    data["theory_error_mbps"] = data.theory_error_bps / 1e6
    data.to_csv(summary_dir / "fig1a_data.csv", index=False)

    fig, axis = plt.subplots(figsize=(10.6, 5.2), constrained_layout=True)
    eligible = truth(data.theory_applicable)
    axis.plot(
        data.time_min,
        data.measured_max_bw_mbps,
        color=COLORS["BBRv2-ideal"],
        marker="s",
        markersize=3.4,
        linewidth=1.5,
        label="Measured MaxBw",
    )
    axis.plot(
        data.loc[eligible, "time_min"],
        data.loc[eligible, "theory_max_bw_mbps"],
        color="#D55E00",
        marker="o",
        markersize=3.2,
        linewidth=1.4,
        label="Theory: sequential fluid recurrence",
    )
    ymax = max(
        1.0,
        float(data.measured_max_bw_mbps.max()),
        float(data.theory_max_bw_mbps.max()),
    )
    configure_time_axis(axis, simulation_time_s)
    axis.set_ylim(bottom=0.0, top=ymax * 1.06)
    axis.set_ylabel("MaxBw (Mbit/s)")
    axis.set_title("Fig. 1(a): BBRv2-ideal theory and measured MaxBw")
    axis.grid(axis="y", alpha=0.25)
    axis.legend(frameon=False, loc="upper right")
    fig.savefig(figures_dir / "fig1a_maxbw_theory_vs_measured.png", dpi=220)
    fig.savefig(figures_dir / "fig1a_maxbw_theory_vs_measured.pdf")
    plt.close(fig)


def plot_fig1b(
    stages: pd.DataFrame,
    minutes: pd.DataFrame,
    minute_flows: pd.DataFrame,
    figures_dir: Path,
    summary_dir: Path,
    simulation_time_s: float,
) -> None:
    stages.sort_values(["algorithm", "stage_index"]).to_csv(
        summary_dir / "fig1b_stage_data.csv", index=False
    )
    raw_minute_data = minutes[
        [
            "algorithm",
            "mode",
            "seed",
            "run_id",
            "minute_index",
            "window_start_s",
            "window_end_s",
            "sample_count",
            "mean_active_flows",
            "mean_queue_delay_ms",
            "aggregate_goodput_bps",
            "mean_flow_goodput_bps",
            "jain_fairness",
        ]
    ].sort_values(["algorithm", "run_id", "minute_index"])
    raw_minute_data.to_csv(summary_dir / "fig1b_minute_data.csv", index=False)
    minute_flows.sort_values(
        ["algorithm", "run_id", "minute_index", "flow_id"]
    ).to_csv(summary_dir / "fig1b_minute_flow_data.csv", index=False)
    figure_data = (
        raw_minute_data.groupby(
            ["algorithm", "mode", "minute_index", "window_start_s", "window_end_s"],
            as_index=False,
        )
        .agg(
            mean_active_flows=("mean_active_flows", "mean"),
            mean_queue_delay_ms=("mean_queue_delay_ms", "mean"),
            aggregate_goodput_bps=("aggregate_goodput_bps", "mean"),
            mean_flow_goodput_bps=("mean_flow_goodput_bps", "mean"),
            jain_fairness=("jain_fairness", "mean"),
        )
        .sort_values(["algorithm", "minute_index"])
    )
    figure_data["time_min"] = figure_data.window_end_s.astype(float) / 60.0
    figure_data["aggregate_goodput_mbps"] = (
        figure_data.aggregate_goodput_bps.astype(float) / 1e6
    )
    figure_data["mean_flow_goodput_mbps"] = (
        figure_data.mean_flow_goodput_bps.astype(float) / 1e6
    )
    figure_data.to_csv(summary_dir / "fig1b_data.csv", index=False)

    fig, axes = plt.subplots(2, 2, figsize=(12.4, 8.6))
    panels = [
        (
            axes[0, 0],
            "(a) One-minute mean queue delay",
            "mean_queue_delay_ms",
            "Mean queue delay (ms)",
        ),
        (
            axes[0, 1],
            "(b) One-minute aggregate goodput",
            "aggregate_goodput_mbps",
            "Aggregate goodput (Mbit/s)",
        ),
        (
            axes[1, 0],
            "(c) One-minute mean active-flow goodput",
            "mean_flow_goodput_mbps",
            "Mean active-flow goodput (Mbit/s)",
        ),
        (
            axes[1, 1],
            "(d) One-minute Jain fairness",
            "jain_fairness",
            "Jain fairness index",
        ),
    ]
    for axis, title, metric, ylabel in panels:
        for algorithm in ALGORITHMS:
            subset = figure_data[figure_data.algorithm == algorithm].sort_values(
                "minute_index"
            )
            if subset.empty:
                continue
            axis.plot(
                subset.time_min,
                subset[metric],
                color=COLORS[algorithm],
                marker=MARKERS[algorithm],
                markersize=4.5,
                linewidth=1.5,
                label=algorithm,
            )
        configure_time_axis(axis, simulation_time_s)
        axis.set_ylabel(ylabel)
        axis.set_title(title)
        axis.grid(axis="y", alpha=0.25)
        if metric == "jain_fairness":
            axis.set_ylim(0.0, 1.05)
    handles, labels = axes[0, 0].get_legend_handles_labels()
    fig.legend(handles, labels, loc="lower center", ncol=4, frameon=False)
    fig.subplots_adjust(
        left=0.08, right=0.99, top=0.92, bottom=0.16, hspace=0.42, wspace=0.30
    )
    fig.savefig(figures_dir / "fig1b_excess_inflight_and_queue_delay.png", dpi=220)
    fig.savefig(figures_dir / "fig1b_excess_inflight_and_queue_delay.pdf")
    plt.close(fig)


def metric_definition(column: str) -> str:
    definitions = {
        "algorithm": "Controller label passed to the scenario.",
        "mode": "original for native controller behavior; ideal only for BBRv2-ideal.",
        "seed": "ns-3 and DQC deterministic random seed.",
        "run_id": "Run identifier from the runner manifest.",
        "simulation_time_s": "Total simulated duration in seconds.",
        "stages": "Number of dynamic population stages.",
        "expected_ideal_up_events": "Sum of requested ideal UP events across all stages.",
        "observed_ideal_up_events": "Recorded BBRv2-ideal UP events.",
        "max_concurrent_up": "Maximum simultaneously active ideal UP events.",
        "probe_rtt_seen": "Whether a sampled BBR-family controller entered ProbeRTT.",
        "ideal_sequence_validation": "Sequence/order/non-overlap validation for BBRv2-ideal.",
        "validation_pass": "Scenario validation result.",
        "stage_index": "Zero-based index in 2,4,8,16,8,4,2.",
        "stage_label": "Human-readable dynamic stage name.",
        "active_flows": "Configured active flow count in the stage.",
        "stage_start_s": "Nominal stage start time.",
        "stage_end_s": "Nominal stage end time.",
        "measurement_start_s": "Actual metrics-window start after the edge guard.",
        "measurement_end_s": "Actual metrics-window end before the edge guard.",
        "duration_s": "Metrics-window duration.",
        "sample_count": "Number of periodic aggregate samples.",
        "minute_index": "Zero-based 60-second aggregation-window index.",
        "window_start_s": "Start of the one-minute aggregation window.",
        "window_end_s": "End of the one-minute aggregation window.",
        "mean_active_flows": "Mean configured active-flow count in the one-minute window.",
        "throughput_recorded": "Whether the receiver-byte snapshot at this minute boundary executed.",
        "aggregate_goodput_bps": "Aggregate receiver goodput over the measurement window.",
        "utilization_pct": "Aggregate goodput divided by 100 Mbit/s capacity.",
        "jain_fairness": "Jain fairness over active-flow receiver goodputs.",
        "mean_flow_goodput_bps": "Mean receiver goodput across flows active in the measurement window.",
        "min_flow_goodput_bps": "Minimum active-flow receiver goodput.",
        "max_flow_goodput_bps": "Maximum active-flow receiver goodput.",
        "mean_excess_inflight_bytes": "Mean max(0, aggregate inflight minus one base BDP).",
        "mean_excess_inflight_bdp": "Mean excess inflight normalized by one base BDP.",
        "p95_excess_inflight_bytes": "95th percentile excess inflight in bytes.",
        "p95_excess_inflight_bdp": "95th percentile excess inflight in BDP.",
        "max_excess_inflight_bdp": "Maximum excess inflight in BDP.",
        "mean_aggregate_inflight_bytes": "Mean sum of active-flow inflight bytes.",
        "p95_aggregate_inflight_bytes": "95th percentile aggregate inflight bytes.",
        "mean_queue_bytes": "Mean bottleneck DropTail queue occupancy.",
        "mean_queue_delay_ms": "Mean queue serialization delay, mean_queue_bytes * 8 / C.",
        "p50_queue_delay_ms": "Median queue serialization delay, queue_bytes * 8 / C.",
        "p95_queue_delay_ms": "95th percentile queue serialization delay.",
        "p99_queue_delay_ms": "99th percentile queue serialization delay.",
        "max_queue_delay_ms": "Maximum queue serialization delay.",
        "queue_drop_packets": "Bottleneck DropTail packet drops during the stage.",
        "queue_drop_bytes": "Bottleneck DropTail dropped bytes during the stage.",
        "mean_sum_pacing_bps": "Mean sum of active-flow controller pacing rates.",
        "mean_bandwidth_estimate_bps": "Mean sum of controller bandwidth estimates.",
        "ideal_up_events": "Ideal UP events recorded in the stage.",
        "theory_applicable_up_events": "Ideal UP events with no concurrent other UP.",
        "mean_theory_error_pct": "Mean (measured MaxBw after UP minus prediction) / prediction.",
        "max_abs_theory_error_pct": "Largest absolute theory prediction error in the stage.",
        "flow_id": "One-based flow identifier.",
        "active_in_window": "Whether this flow was active for any portion of the one-minute window.",
        "active_duration_s": "Seconds the flow was configured active within the one-minute window.",
        "received_bytes": "Receiver payload bytes during the associated measurement window.",
        "goodput_bps": "Per-flow receiver goodput over its active duration in the associated window.",
        "goodput_share_pct": "Per-flow share of stage aggregate goodput.",
        "event_id": "One-based BBRv2-ideal UP event identifier.",
        "probe_order": "Configured ideal UP order; equals flow_id.",
        "theory_applicable": "No other flow was in ProbeBW-UP at this UP entry.",
        "strict_controlled": "Event entered through the BBRv2-ideal admission gate.",
        "pre_all_other_cruise": "Diagnostic: all non-owner BBRv2 flows were in PROBE_CRUISE at entry.",
        "pre_other_up_count": "Number of non-owner ProbeBW-UP flows at entry.",
        "start_time_s": "ProbeBW-UP entry time.",
        "end_time_s": "ProbeBW-UP exit time.",
        "max_bw_before_bps": "Owner BBRv2 MaxBw before UP.",
        "max_bw_peak_bps": "Largest owner MaxBw sampled during UP.",
        "max_bw_after_bps": "Owner BBRv2 MaxBw after UP.",
        "delivery_rate_peak_bps": "Largest owner delivery-rate sample during UP.",
        "up_pacing_rate_bps": "Owner pacing rate at UP entry.",
        "sum_pacing_start_bps": "All active-flow pacing-rate sum at UP entry.",
        "theory_service_bps": "Fluid-model service allocated to the UP owner.",
        "theory_max_bw_bps": "Fluid-model predicted post-UP MaxBw.",
        "effective_service_bps": "Capacity split implied by sampled pacing rates.",
        "theory_error_bps": "Measured post-UP MaxBw minus theory_max_bw_bps.",
        "theory_error_pct": "theory_error_bps divided by theory_max_bw_bps.",
        "max_queue_bytes": "Largest bottleneck queue occupancy during UP.",
        "max_aggregate_inflight_bytes": "Largest aggregate inflight during UP.",
    }
    return definitions.get(column, "Recorded field; see the scenario source for its exact type.")


def write_metric_catalog(
    summary: pd.DataFrame,
    stages: pd.DataFrame,
    flows: pd.DataFrame,
    events: pd.DataFrame,
    minutes: pd.DataFrame,
    minute_flows: pd.DataFrame,
    output: Path,
) -> None:
    datasets = [
        ("all_runs.csv", summary),
        ("all_stage_metrics.csv / comparison_metrics.csv", stages),
        ("all_flow_metrics.csv", flows),
        ("all_ideal_up_events.csv", events),
        ("all_minute_metrics.csv", minutes),
        ("all_minute_flow_metrics.csv", minute_flows),
    ]
    lines = [
        "# Test 1 Metric Catalog",
        "",
        "All data below are generated from the files listed in `raw/manifest.csv`.",
        "The bottleneck is 100 Mbit/s and one base BDP is C * 40 ms / 8.",
        "",
    ]
    for dataset, frame in datasets:
        lines.extend([f"## `{dataset}`", "", "| Field | Definition |", "| --- | --- |"])
        for column in frame.columns:
            lines.append(f"| `{column}` | {metric_definition(column)} |")
        lines.append("")
    lines.extend(
        [
            "## Comparison Scope",
            "",
            "All eight algorithms appear in the stage-level comparison and the one-minute Fig. 1(b) time series.",
            "Fig. 1(b) reports queue delay, aggregate goodput, mean active-flow goodput, and Jain fairness from receiver byte snapshots at every 60-second boundary.",
            "Fig. 1(a) is intentionally limited to BBRv2-ideal: the sequential",
            "ProbeBW-UP fluid recurrence is defined only for that experiment-specific",
            "BBRv2 admission path. CUBIC has no ProbeBW-UP state and the other",
            "controllers retain their original implementations.",
            "",
        ]
    )
    output.write_text("\n".join(lines), encoding="utf-8")


def format_peak_table(stages: pd.DataFrame) -> list[str]:
    peak = stages[stages.stage_label == "N16_peak"].set_index("algorithm")
    lines = [
        "| Algorithm | Goodput (Mbit/s) | Utilization (%) | Jain | Mean excess (BDP) | Mean queue delay (ms) | p95 queue delay (ms) | Drops |",
        "| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |",
    ]
    for algorithm in ALGORITHMS:
        if algorithm not in peak.index:
            continue
        row = peak.loc[algorithm]
        lines.append(
            "| {algorithm} | {goodput:.3f} | {utilization:.3f} | {jain:.5f} | "
            "{excess:.4f} | {mean_delay:.4f} | {p95_delay:.4f} | {drops} |".format(
                algorithm=algorithm,
                goodput=float(row.aggregate_goodput_bps) / 1e6,
                utilization=float(row.utilization_pct),
                jain=float(row.jain_fairness),
                excess=float(row.mean_excess_inflight_bdp),
                mean_delay=float(row.mean_queue_delay_ms),
                p95_delay=float(row.p95_queue_delay_ms),
                drops=int(row.queue_drop_packets),
            )
        )
    return lines


def write_report(
    summary: pd.DataFrame,
    stages: pd.DataFrame,
    events: pd.DataFrame,
    output: Path,
) -> None:
    ideal_events = events[events.algorithm == "BBRv2-ideal"]
    non_cruise = int((~truth(ideal_events.pre_all_other_cruise)).sum())
    probe_rtt_runs = int(truth(summary.probe_rtt_seen).sum())
    durations = sorted(set(summary.simulation_time_s.astype(float).tolist()))
    if durations == [1800.0]:
        duration_line = "- Duration: 1800 simulated seconds for each full run, split evenly across seven stages."
    else:
        duration_line = "- Duration: smoke run with simulated duration(s) " + ", ".join(
            f"{duration:g}" for duration in durations
        ) + " seconds."
    lines = [
        "# Test 1 Results",
        "",
        "This report is generated from the paths in `raw/manifest.csv`.",
        "",
        "## Scenario",
        "",
        "- Bottleneck: C=100 Mbit/s, base RTT=40 ms, DropTail buffer=40 BDP.",
        "- Dynamic population: 2 -> 4 -> 8 -> 16 -> 8 -> 4 -> 2.",
        duration_line,
        "- Algorithms: BBR-R, oBBR, BBRv2+, CUBIC, BBRv2-ideal, BBRv2, FBBR.",
        "- BBRv2-ideal alone changes ProbeBW-UP admission: flow IDs enter in order and no two are in UP concurrently.",
        "- All other controllers, including BBRv2 without a suffix, retain their original control paths.",
        "- ProbeRTT is observed as a metric; no cross-algorithm ProbeRTT alias or control-path change is used.",
        "",
        "## Validation",
        "",
        f"- Runs: {len(summary)}",
        f"- Expected / observed BBRv2-ideal UP events: {EXPECTED_IDEAL_EVENTS} / {len(ideal_events)}",
        f"- Largest simultaneous ideal UP count: {int(ideal_events.max_concurrent_up.max()) if not ideal_events.empty else 0}",
        f"- Ideal entries without the all-other-CRUISE diagnostic: {non_cruise}",
        f"- Runs where ProbeRTT was observed: {probe_rtt_runs}",
        "",
        "The non-owner CRUISE field is diagnostic only. The enforced condition is the requested one: no other flow may be in ProbeBW-UP during an ideal UP.",
        "",
        "## N=16 Peak Comparison",
        "",
        *format_peak_table(stages),
        "",
        "## Outputs",
        "",
        "- `summary/comparison_metrics.csv`: all stage-level comparison metrics for all algorithms.",
        "- `summary/overall_comparison_metrics.csv`: equal-stage aggregate comparison.",
        "- `summary/METRICS.md`: complete field catalog and definitions.",
        "- `summary/all_minute_metrics.csv`: one-minute aggregate measurements, including receiver aggregate goodput, mean active-flow goodput, and Jain fairness.",
        "- `summary/all_minute_flow_metrics.csv`: one-minute per-flow receiver byte and goodput trace.",
        "- `summary/fig1a_data.csv`: time-indexed per-UP theory and measured MaxBw records for BBRv2-ideal.",
        "- `summary/fig1b_data.csv`: all-algorithm one-minute plot data.",
        "- `figures/fig1a_maxbw_theory_vs_measured.png`.",
        "- `figures/fig1b_excess_inflight_and_queue_delay.png`.",
        "",
        "Both figures use simulation time in minutes, with labels every two minutes and unlabeled one-minute minor ticks. Fig. 1(a) stays an event-level theory check for BBRv2-ideal. Fig. 1(b) contains four one-minute panels: queue delay, aggregate goodput, mean active-flow goodput, and Jain fairness.",
        "",
    ]
    output.write_text("\n".join(lines), encoding="utf-8")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--results-dir", required=True, type=Path)
    parser.add_argument("--manifest", required=True, type=Path)
    parser.add_argument("--expect-full", action="store_true")
    args = parser.parse_args()

    results_dir: Path = args.results_dir
    summary_dir = results_dir / "summary"
    figures_dir = results_dir / "figures"
    summary_dir.mkdir(parents=True, exist_ok=True)
    figures_dir.mkdir(parents=True, exist_ok=True)

    summary = concat_csv(read_paths(args.manifest, "run_summary_path"))
    stages = concat_csv(read_paths(args.manifest, "stage_metrics_path"))
    stages = add_derived_stage_metrics(stages)
    flows = concat_csv(read_paths(args.manifest, "flow_metrics_path"))
    events = concat_csv(read_paths(args.manifest, "events_path"))
    minutes = concat_csv(read_paths(args.manifest, "minute_metrics_path"))
    minute_flows = concat_csv(read_paths(args.manifest, "minute_flow_metrics_path"))

    validation = validate_results(
        summary, stages, flows, events, minutes, minute_flows, args.expect_full
    )
    validation.to_csv(summary_dir / "validation.csv", index=False)
    write_aggregate_data(
        summary, stages, flows, events, minutes, minute_flows, summary_dir
    )
    simulation_time_s = float(summary.simulation_time_s.astype(float).max())
    plot_fig1a(events, figures_dir, summary_dir, simulation_time_s)
    plot_fig1b(
        stages, minutes, minute_flows, figures_dir, summary_dir, simulation_time_s
    )
    write_metric_catalog(
        summary,
        stages,
        flows,
        events,
        minutes,
        minute_flows,
        summary_dir / "METRICS.md",
    )
    write_report(summary, stages, events, results_dir / "RESULTS.md")


if __name__ == "__main__":
    main()
