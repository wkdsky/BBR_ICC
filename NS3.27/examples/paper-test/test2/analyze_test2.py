#!/usr/bin/env python3
"""Aggregate fixed-four-flow Test 2 outputs without generating figures."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Any

import pandas as pd


ALGORITHMS = [
    "BBR-R",
    "oBBR",
    "BBRv2+",
    "CUBIC",
    "BBRv2-formal",
    "BBRv2",
    "FBBR",
]
SCENARIOS = ["BASE", "BUF-S", "BW-L", "BW-H", "RTT-S", "RTT-L"]
EXPECTED_SCENARIOS = {
    "BASE": (100_000_000, 0.040, 2.0),
    "BUF-S": (100_000_000, 0.040, 0.5),
    "BW-L": (10_000_000, 0.040, 2.0),
    "BW-H": (1_000_000_000, 0.040, 2.0),
    "RTT-S": (100_000_000, 0.010, 2.0),
    "RTT-L": (100_000_000, 0.200, 2.0),
}
MANIFEST_COLUMNS = [
    "scenario_id",
    "capacity_bps",
    "capacity_mbps",
    "base_rtt_s",
    "base_rtt_ms",
    "buffer_bdp",
    "bottleneck_delay_s",
    "access_bps",
    "purpose",
    "algorithm",
    "mode",
    "seed",
    "run_id",
    "run_summary_path",
    "stage_metrics_path",
    "flow_metrics_path",
    "events_path",
    "minute_metrics_path",
    "minute_flow_metrics_path",
    "metadata_path",
]
CONFIG_COLUMNS = [
    "scenario_id",
    "capacity_bps",
    "capacity_mbps",
    "base_rtt_s",
    "base_rtt_ms",
    "buffer_bdp",
    "bottleneck_delay_s",
    "access_bps",
    "purpose",
]
RUN_KEY = ["scenario_id", "algorithm", "mode", "seed", "run_id"]


def truth(values: pd.Series) -> pd.Series:
    return values.astype(str).str.strip().str.lower().isin({"1", "true", "yes"})


def require_columns(frame: pd.DataFrame, columns: list[str], name: str) -> None:
    missing = [column for column in columns if column not in frame.columns]
    if missing:
        raise ValueError(f"{name} is missing columns: {', '.join(missing)}")


def resolve_path(manifest: Path, value: object) -> Path:
    path = Path(str(value))
    if path.exists():
        return path
    if path.is_absolute():
        raise FileNotFoundError(f"Manifest entry does not exist: {path}")
    ns3_dir = manifest.parents[3]
    alternate = ns3_dir / path
    if alternate.exists():
        return alternate
    raise FileNotFoundError(f"Manifest entry does not exist: {path}")


def read_manifest(path: Path) -> pd.DataFrame:
    manifest = pd.read_csv(path)
    require_columns(manifest, MANIFEST_COLUMNS, "manifest")
    if manifest.empty:
        raise ValueError("Manifest contains no Test 2 runs")
    return manifest


def load_csv_dataset(
    manifest: pd.DataFrame, manifest_path: Path, path_column: str, name: str
) -> pd.DataFrame:
    tables: list[pd.DataFrame] = []
    for entry in manifest.to_dict("records"):
        table = pd.read_csv(resolve_path(manifest_path, entry[path_column]))
        require_columns(table, ["algorithm", "mode", "seed", "run_id"], name)
        for column in CONFIG_COLUMNS:
            table[column] = entry[column]
        table["_manifest_algorithm"] = entry["algorithm"]
        table["_manifest_mode"] = entry["mode"]
        table["_manifest_seed"] = entry["seed"]
        table["_manifest_run_id"] = entry["run_id"]
        tables.append(table)
    nonempty_tables = [table for table in tables if not table.empty]
    return pd.concat(nonempty_tables or tables[:1], ignore_index=True)


def load_metadata(manifest: pd.DataFrame, manifest_path: Path) -> pd.DataFrame:
    records: list[dict[str, Any]] = []
    for entry in manifest.to_dict("records"):
        metadata_path = resolve_path(manifest_path, entry["metadata_path"])
        payload = json.loads(metadata_path.read_text(encoding="utf-8"))
        record: dict[str, Any] = dict(payload)
        for column in CONFIG_COLUMNS:
            if column in record:
                record[f"metadata_{column}"] = record[column]
            record[column] = entry[column]
        record["_manifest_algorithm"] = entry["algorithm"]
        record["_manifest_mode"] = entry["mode"]
        record["_manifest_seed"] = entry["seed"]
        record["_manifest_run_id"] = entry["run_id"]
        records.append(record)
    return pd.DataFrame(records)


def numeric_matches(value: object, expected: float, tolerance: float = 1e-9) -> bool:
    try:
        return abs(float(value) - expected) <= tolerance
    except (TypeError, ValueError):
        return False


def validate_identity(name: str, frame: pd.DataFrame) -> tuple[bool, str]:
    fields = [
        ("algorithm", "_manifest_algorithm"),
        ("mode", "_manifest_mode"),
        ("seed", "_manifest_seed"),
        ("run_id", "_manifest_run_id"),
    ]
    invalid = 0
    for raw_column, manifest_column in fields:
        if raw_column not in frame.columns:
            return False, f"Missing raw identity field {raw_column}"
        invalid += int((frame[raw_column].astype(str) != frame[manifest_column].astype(str)).sum())
    return invalid == 0, f"{invalid} raw rows disagree with their manifest entry"


def validate_results(
    manifest: pd.DataFrame,
    summary: pd.DataFrame,
    stages: pd.DataFrame,
    flows: pd.DataFrame,
    events: pd.DataFrame,
    minutes: pd.DataFrame,
    minute_flows: pd.DataFrame,
    metadata: pd.DataFrame,
    expect_full: bool,
) -> pd.DataFrame:
    checks: list[dict[str, object]] = []

    def add(check: str, passed: bool, detail: str) -> None:
        checks.append({"check": check, "passed": bool(passed), "detail": detail})

    duplicate_manifest = manifest.duplicated(
        ["scenario_id", "algorithm", "seed", "run_id"]
    ).sum()
    add(
        "manifest_unique_runs",
        duplicate_manifest == 0,
        f"{duplicate_manifest} duplicate scenario/controller/seed/run rows",
    )
    for name, frame in [
        ("run_summary", summary),
        ("stage_metrics", stages),
        ("flow_metrics", flows),
        ("events", events),
        ("minute_metrics", minutes),
        ("minute_flow_metrics", minute_flows),
        ("metadata", metadata),
    ]:
        passed, detail = validate_identity(name, frame)
        add(f"{name}_identity", passed, detail)

    required_summary = [
        "simulation_time_s",
        "stages",
        "active_flows",
        "validation_pass",
    ]
    required_stage = [
        "stage_index",
        "stage_label",
        "active_flows",
        "sample_count",
        "aggregate_goodput_bps",
    ]
    require_columns(summary, required_summary, "run summary")
    require_columns(stages, required_stage, "stage metrics")
    require_columns(flows, ["flow_id", "goodput_bps"], "flow metrics")
    require_columns(minutes, ["minute_index", "mean_active_flows"], "minute metrics")
    require_columns(
        minute_flows,
        ["minute_index", "flow_id", "active_in_window"],
        "minute flow metrics",
    )

    add(
        "one_summary_per_manifest_run",
        len(summary) == len(manifest),
        f"Expected {len(manifest)} summaries, found {len(summary)}",
    )
    add(
        "all_run_validations_pass",
        truth(summary.validation_pass).all(),
        f"{int((~truth(summary.validation_pass)).sum())} run validations failed",
    )

    for entry in manifest.to_dict("records"):
        scenario_id = entry["scenario_id"]
        algorithm = entry["algorithm"]
        seed = str(entry["seed"])
        run_id = str(entry["run_id"])
        mask = (
            (summary.scenario_id == scenario_id)
            & (summary.algorithm == algorithm)
            & (summary.seed.astype(str) == seed)
            & (summary.run_id.astype(str) == run_id)
        )
        run_summary = summary.loc[mask]
        label = f"{scenario_id}:{algorithm}"
        add(
            f"{label}:summary_count",
            len(run_summary) == 1,
            f"Expected one summary row, found {len(run_summary)}",
        )
        if len(run_summary) != 1:
            continue
        run = run_summary.iloc[0]
        stage_mask = (
            (stages.scenario_id == scenario_id)
            & (stages.algorithm == algorithm)
            & (stages.seed.astype(str) == seed)
            & (stages.run_id.astype(str) == run_id)
        )
        run_stages = stages.loc[stage_mask]
        add(
            f"{label}:fixed_four_stage",
            len(run_stages) == 1
            and int(run_stages.iloc[0].stage_index) == 0
            and str(run_stages.iloc[0].stage_label) == "N4_steady"
            and int(run_stages.iloc[0].active_flows) == 4
            and int(run_stages.iloc[0].sample_count) > 0,
            "One populated N4_steady stage is required.",
        )
        flow_mask = (
            (flows.scenario_id == scenario_id)
            & (flows.algorithm == algorithm)
            & (flows.seed.astype(str) == seed)
            & (flows.run_id.astype(str) == run_id)
        )
        run_flows = flows.loc[flow_mask]
        add(
            f"{label}:four_flow_rows",
            len(run_flows) == 4 and sorted(run_flows.flow_id.astype(int)) == [1, 2, 3, 4],
            f"Expected four flow rows, found {len(run_flows)}",
        )
        expected_minutes = int(math.ceil(float(run.simulation_time_s) / 60.0))
        minute_mask = (
            (minutes.scenario_id == scenario_id)
            & (minutes.algorithm == algorithm)
            & (minutes.seed.astype(str) == seed)
            & (minutes.run_id.astype(str) == run_id)
        )
        run_minutes = minutes.loc[minute_mask]
        add(
            f"{label}:minute_rows",
            len(run_minutes) == expected_minutes
            and (pd.to_numeric(run_minutes.mean_active_flows) == 4.0).all(),
            f"Expected {expected_minutes} fixed-four-flow minute rows, found {len(run_minutes)}",
        )
        minute_flow_mask = (
            (minute_flows.scenario_id == scenario_id)
            & (minute_flows.algorithm == algorithm)
            & (minute_flows.seed.astype(str) == seed)
            & (minute_flows.run_id.astype(str) == run_id)
        )
        run_minute_flows = minute_flows.loc[minute_flow_mask]
        add(
            f"{label}:minute_flow_rows",
            len(run_minute_flows) == expected_minutes * 4,
            f"Expected {expected_minutes * 4} minute-flow rows, found {len(run_minute_flows)}",
        )

    for scenario_id, (capacity_bps, base_rtt_s, buffer_bdp) in EXPECTED_SCENARIOS.items():
        configured = manifest[manifest.scenario_id == scenario_id]
        if configured.empty:
            continue
        values_match = (
            configured.capacity_bps.map(lambda value: numeric_matches(value, capacity_bps)).all()
            and configured.base_rtt_s.map(lambda value: numeric_matches(value, base_rtt_s)).all()
            and configured.buffer_bdp.map(lambda value: numeric_matches(value, buffer_bdp)).all()
        )
        add(
            f"{scenario_id}:configured_parameters",
            values_match,
            f"C={capacity_bps} bit/s, RTT={base_rtt_s * 1000:g} ms, buffer={buffer_bdp:g} BDP",
        )

    expected_metadata_bdp = (
        metadata.metadata_capacity_bps.astype(float)
        * metadata.metadata_base_rtt_s.astype(float)
        / 8.0
    )
    metadata_matches = (
        (metadata.algorithm.astype(str) == metadata._manifest_algorithm.astype(str)).all()
        and (metadata.active_flows.astype(int) == 4).all()
        and (
            metadata.metadata_capacity_bps.astype(float)
            == metadata.capacity_bps.astype(float)
        ).all()
        and (
            metadata.metadata_base_rtt_s.astype(float)
            == metadata.base_rtt_s.astype(float)
        ).all()
        and (
            metadata.queue_bdp.astype(float)
            == metadata.buffer_bdp.astype(float)
        ).all()
        and (metadata.base_bdp_bytes.astype(float) - expected_metadata_bdp).abs().le(1.0).all()
    )
    add(
        "metadata_matches_scenario",
        metadata_matches,
        "Metadata records the selected capacity, RTT-derived BDP, and four active flows.",
    )

    if expect_full:
        add(
            "full_scenario_set",
            set(manifest.scenario_id) == set(SCENARIOS),
            "The complete matrix contains BASE, BUF-S, BW-L, BW-H, RTT-S, and RTT-L.",
        )
        add(
            "full_run_count",
            len(manifest) == len(SCENARIOS) * len(ALGORITHMS),
            f"Expected {len(SCENARIOS) * len(ALGORITHMS)} manifest rows, found {len(manifest)}",
        )
        add(
            "full_duration",
            (summary.simulation_time_s.astype(float) == 300.0).all(),
            "Every full matrix run lasts 300 simulated seconds.",
        )
        for scenario_id in SCENARIOS:
            algorithms = manifest.loc[
                manifest.scenario_id == scenario_id, "algorithm"
            ].tolist()
            add(
                f"{scenario_id}:controller_set",
                algorithms == ALGORITHMS,
                "Controller order is BBR-R, oBBR, BBRv2+, CUBIC, BBRv2-formal, BBRv2, FBBR.",
            )
    return pd.DataFrame(checks)


def sort_experiment_rows(frame: pd.DataFrame, extra_columns: list[str]) -> pd.DataFrame:
    if frame.empty:
        return frame.copy()
    sorted_frame = frame.copy()
    sorted_frame["_scenario_order"] = sorted_frame.scenario_id.map(
        {scenario: index for index, scenario in enumerate(SCENARIOS)}
    ).fillna(len(SCENARIOS))
    sorted_frame["_algorithm_order"] = sorted_frame.algorithm.map(
        {algorithm: index for index, algorithm in enumerate(ALGORITHMS)}
    ).fillna(len(ALGORITHMS))
    return (
        sorted_frame.sort_values(["_scenario_order", "_algorithm_order", *extra_columns])
        .drop(columns=["_scenario_order", "_algorithm_order"])
        .reset_index(drop=True)
    )


def public_frame(frame: pd.DataFrame) -> pd.DataFrame:
    return frame.drop(
        columns=[column for column in frame.columns if column.startswith("_manifest_")]
    )


def build_key_metrics(stages: pd.DataFrame, summary: pd.DataFrame) -> pd.DataFrame:
    summary_columns = RUN_KEY + [
        "simulation_time_s",
        "observed_ideal_up_events",
        "max_concurrent_up",
        "ideal_sequence_validation",
        "validation_pass",
    ]
    key_metrics = stages.merge(
        summary[summary_columns], on=RUN_KEY, how="left", validate="one_to_one"
    ).copy()
    key_metrics["path_bdp_bytes"] = (
        key_metrics.capacity_bps.astype(float) * key_metrics.base_rtt_s.astype(float) / 8.0
    ).round().astype("int64")
    key_metrics["buffer_bytes"] = (
        key_metrics.path_bdp_bytes.astype(float) * key_metrics.buffer_bdp.astype(float)
    ).round().astype("int64")
    key_metrics["aggregate_goodput_mbps"] = (
        key_metrics.aggregate_goodput_bps.astype(float) / 1e6
    )
    key_metrics["mean_flow_goodput_mbps"] = (
        key_metrics.mean_flow_goodput_bps.astype(float) / 1e6
    )
    key_metrics["min_flow_goodput_mbps"] = (
        key_metrics.min_flow_goodput_bps.astype(float) / 1e6
    )
    key_metrics["max_flow_goodput_mbps"] = (
        key_metrics.max_flow_goodput_bps.astype(float) / 1e6
    )
    key_metrics["mean_sum_pacing_mbps"] = (
        key_metrics.mean_sum_pacing_bps.astype(float) / 1e6
    )
    key_metrics["mean_bandwidth_estimate_mbps"] = (
        key_metrics.mean_bandwidth_estimate_bps.astype(float) / 1e6
    )
    columns = [
        "scenario_id",
        "purpose",
        "capacity_mbps",
        "base_rtt_ms",
        "buffer_bdp",
        "path_bdp_bytes",
        "buffer_bytes",
        "algorithm",
        "mode",
        "seed",
        "run_id",
        "simulation_time_s",
        "measurement_start_s",
        "measurement_end_s",
        "duration_s",
        "sample_count",
        "aggregate_goodput_mbps",
        "utilization_pct",
        "jain_fairness",
        "mean_flow_goodput_mbps",
        "min_flow_goodput_mbps",
        "max_flow_goodput_mbps",
        "mean_excess_inflight_bdp",
        "p95_excess_inflight_bdp",
        "max_excess_inflight_bdp",
        "mean_queue_delay_ms",
        "p50_queue_delay_ms",
        "p95_queue_delay_ms",
        "p99_queue_delay_ms",
        "max_queue_delay_ms",
        "queue_drop_packets",
        "queue_drop_bytes",
        "mean_sum_pacing_mbps",
        "mean_bandwidth_estimate_mbps",
        "probe_rtt_seen",
        "observed_ideal_up_events",
        "max_concurrent_up",
        "ideal_sequence_validation",
        "validation_pass",
    ]
    return sort_experiment_rows(key_metrics[columns], [])


def metric_definition(column: str) -> str:
    definitions = {
        "scenario_id": "Scenario identifier from scenarios.csv.",
        "purpose": "Controlled network condition represented by the scenario.",
        "capacity_mbps": "Bottleneck capacity in Mbit/s.",
        "base_rtt_ms": "End-to-end propagation RTT in milliseconds.",
        "buffer_bdp": "DropTail bottleneck queue capacity in path-BDP multiples.",
        "path_bdp_bytes": "Computed as capacity_bps * base_rtt_s / 8.",
        "buffer_bytes": "Configured bottleneck queue capacity in bytes.",
        "algorithm": "Controller selected for the run.",
        "mode": "original for native behavior; ideal for BBRv2-formal.",
        "seed": "Deterministic ns-3 and DQC random seed.",
        "run_id": "Deterministic DQC/ns-3 run identifier.",
        "simulation_time_s": "Total simulated duration; 300 seconds in a full run.",
        "measurement_start_s": "First point included in stage metrics after the guard.",
        "measurement_end_s": "Last point included in stage metrics before the guard.",
        "duration_s": "Stage measurement window duration.",
        "sample_count": "Periodic aggregate samples in the measurement window.",
        "aggregate_goodput_mbps": "Aggregate receiver goodput across the four flows.",
        "utilization_pct": "Aggregate goodput as a percentage of bottleneck capacity.",
        "jain_fairness": "Jain fairness over four per-flow receiver goodputs.",
        "mean_flow_goodput_mbps": "Mean receiver goodput across the four flows.",
        "min_flow_goodput_mbps": "Lowest four-flow receiver goodput.",
        "max_flow_goodput_mbps": "Highest four-flow receiver goodput.",
        "mean_excess_inflight_bdp": "Mean max(0, aggregate inflight minus one path BDP), normalized by BDP.",
        "p95_excess_inflight_bdp": "95th percentile excess inflight normalized by BDP.",
        "max_excess_inflight_bdp": "Largest excess inflight normalized by BDP.",
        "mean_queue_delay_ms": "Mean bottleneck queue serialization delay.",
        "p50_queue_delay_ms": "Median bottleneck queue serialization delay.",
        "p95_queue_delay_ms": "95th percentile bottleneck queue serialization delay.",
        "p99_queue_delay_ms": "99th percentile bottleneck queue serialization delay.",
        "max_queue_delay_ms": "Largest bottleneck queue serialization delay.",
        "queue_drop_packets": "Bottleneck DropTail packet drops in the measurement window.",
        "queue_drop_bytes": "Bottleneck DropTail dropped bytes in the measurement window.",
        "mean_sum_pacing_mbps": "Mean sum of controller pacing rates.",
        "mean_bandwidth_estimate_mbps": "Mean sum of controller bandwidth estimates.",
        "probe_rtt_seen": "Whether sampled BBR-family state entered ProbeRTT.",
        "observed_ideal_up_events": "Recorded BBRv2-formal ProbeBW-UP events.",
        "max_concurrent_up": "Largest count of simultaneous BBRv2-formal ProbeBW-UP events.",
        "ideal_sequence_validation": "Formal BBRv2 entry order/non-overlap validation.",
        "validation_pass": "Whether the run completed all required validation checks.",
    }
    return definitions.get(column, "Recorded Test 2 field.")


def write_metric_catalog(key_metrics: pd.DataFrame, output: Path) -> None:
    lines = [
        "# Test 2 Metric Catalog",
        "",
        "`scenario_key_metrics.csv` is the primary one-row-per-scenario-controller table.",
        "Every row measures a fixed set of four long-lived flows; no figures are generated by this experiment.",
        "",
        "| Field | Definition |",
        "| --- | --- |",
    ]
    for column in key_metrics.columns:
        lines.append(f"| `{column}` | {metric_definition(column)} |")
    lines.append("")
    output.write_text("\n".join(lines), encoding="utf-8")


def write_report(
    manifest: pd.DataFrame,
    key_metrics: pd.DataFrame,
    validation: pd.DataFrame,
    output: Path,
) -> None:
    scenario_rows = (
        manifest[CONFIG_COLUMNS]
        .drop_duplicates("scenario_id")
        .set_index("scenario_id")
        .reindex(SCENARIOS)
        .reset_index()
    )
    lines = [
        "# Test 2 Results",
        "",
        "Each scenario uses four long-lived DQC flows for 300 simulated seconds.",
        "The controller set is BBR-R, oBBR, BBRv2+, CUBIC, BBRv2-formal, BBRv2, and FBBR.",
        "No figures are generated yet.",
        "",
        "## Scenarios",
        "",
        "| ID | Capacity (Mbit/s) | RTT (ms) | Buffer (BDP) | Purpose |",
        "| --- | ---: | ---: | ---: | --- |",
    ]
    for row in scenario_rows.itertuples():
        lines.append(
            f"| {row.scenario_id} | {float(row.capacity_mbps):g} | "
            f"{float(row.base_rtt_ms):g} | {float(row.buffer_bdp):g} | {row.purpose} |"
        )
    failed_checks = int((~validation.passed.astype(bool)).sum())
    lines.extend(
        [
            "",
            "## Validation",
            "",
            f"- Runs: {len(key_metrics)}",
            f"- Failed checks: {failed_checks}",
            "- Primary table: `summary/scenario_key_metrics.csv`.",
            "",
        ]
    )
    for scenario_id in SCENARIOS:
        subset = key_metrics[key_metrics.scenario_id == scenario_id]
        if subset.empty:
            continue
        lines.extend(
            [
                f"## {scenario_id}",
                "",
                "| Algorithm | Goodput (Mbit/s) | Utilization (%) | Jain | Mean queue delay (ms) | p95 queue delay (ms) | Drops |",
                "| --- | ---: | ---: | ---: | ---: | ---: | ---: |",
            ]
        )
        for row in subset.itertuples():
            lines.append(
                f"| {row.algorithm} | {float(row.aggregate_goodput_mbps):.3f} | "
                f"{float(row.utilization_pct):.3f} | {float(row.jain_fairness):.5f} | "
                f"{float(row.mean_queue_delay_ms):.4f} | "
                f"{float(row.p95_queue_delay_ms):.4f} | "
                f"{int(row.queue_drop_packets)} |"
            )
        lines.append("")
    output.write_text("\n".join(lines), encoding="utf-8")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--results-dir", required=True, type=Path)
    parser.add_argument("--manifest", required=True, type=Path)
    parser.add_argument("--expect-full", action="store_true")
    args = parser.parse_args()

    results_dir = args.results_dir
    summary_dir = results_dir / "summary"
    summary_dir.mkdir(parents=True, exist_ok=True)
    manifest_path = args.manifest
    manifest = read_manifest(manifest_path)
    summary = load_csv_dataset(manifest, manifest_path, "run_summary_path", "run summary")
    stages = load_csv_dataset(manifest, manifest_path, "stage_metrics_path", "stage metrics")
    flows = load_csv_dataset(manifest, manifest_path, "flow_metrics_path", "flow metrics")
    events = load_csv_dataset(manifest, manifest_path, "events_path", "events")
    minutes = load_csv_dataset(manifest, manifest_path, "minute_metrics_path", "minute metrics")
    minute_flows = load_csv_dataset(
        manifest, manifest_path, "minute_flow_metrics_path", "minute flow metrics"
    )
    metadata = load_metadata(manifest, manifest_path)

    validation = validate_results(
        manifest,
        summary,
        stages,
        flows,
        events,
        minutes,
        minute_flows,
        metadata,
        args.expect_full,
    )
    key_metrics = build_key_metrics(stages, summary)

    manifest.to_csv(summary_dir / "manifest.csv", index=False)
    public_frame(sort_experiment_rows(summary, [])).to_csv(
        summary_dir / "all_runs.csv", index=False
    )
    public_frame(sort_experiment_rows(stages, ["stage_index"])).to_csv(
        summary_dir / "all_stage_metrics.csv", index=False
    )
    public_frame(sort_experiment_rows(flows, ["stage_index", "flow_id"])).to_csv(
        summary_dir / "all_flow_metrics.csv", index=False
    )
    public_frame(sort_experiment_rows(events, ["event_id"])).to_csv(
        summary_dir / "all_ideal_up_events.csv", index=False
    )
    public_frame(sort_experiment_rows(minutes, ["minute_index"])).to_csv(
        summary_dir / "all_minute_metrics.csv", index=False
    )
    public_frame(sort_experiment_rows(minute_flows, ["minute_index", "flow_id"])).to_csv(
        summary_dir / "all_minute_flow_metrics.csv", index=False
    )
    public_frame(sort_experiment_rows(metadata, [])).to_csv(
        summary_dir / "all_metadata.csv", index=False
    )
    public_frame(sort_experiment_rows(stages, ["stage_index"])).to_csv(
        summary_dir / "scenario_stage_metrics.csv", index=False
    )
    key_metrics.to_csv(summary_dir / "scenario_key_metrics.csv", index=False)
    validation.to_csv(summary_dir / "validation.csv", index=False)
    write_metric_catalog(key_metrics, summary_dir / "METRICS.md")
    write_report(manifest, key_metrics, validation, results_dir / "RESULTS.md")

    if not validation.passed.astype(bool).all():
        failures = validation.loc[~validation.passed.astype(bool), "check"].tolist()
        raise SystemExit("Test 2 validation failed: " + ", ".join(failures))


if __name__ == "__main__":
    main()
