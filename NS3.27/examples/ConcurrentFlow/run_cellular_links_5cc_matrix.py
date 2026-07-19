#!/usr/bin/env python3
"""Run the five-CC cellular-links matrix using the project Mahimahi trace."""

import argparse
import csv
import json
import shlex
import shutil
import subprocess
import sys
import urllib.request
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence, Tuple


SCRIPT_DIR = Path(__file__).resolve().parent
RUNNER = SCRIPT_DIR / "run_4cc_comparison.py"
CONFIG_PATH = SCRIPT_DIR / "cellular_links_5cc_matrix.json"
NS3_ROOT = SCRIPT_DIR.parents[1]
REPO_ROOT = NS3_ROOT.parent
DEFAULT_LOG_ROOT = Path("/mnt/nasDisk_ds3617/wkd/1FreqBBR")


@dataclass(frozen=True)
class Scenario:
    name: str
    n_flows: int
    switch_buffer_bdp: float
    seed: int
    service_rate: str
    capacity_scale: float


def unique_dir(path: Path) -> Path:
    if not path.exists():
        return path
    for index in range(2, 1000):
        candidate = path.with_name(f"{path.name}_{index:02d}")
        if not candidate.exists():
            return candidate
    raise RuntimeError(f"too many existing experiment directories for {path}")


def load_config(path: Path) -> Dict[str, object]:
    with path.open("r", encoding="utf-8") as handle:
        return json.load(handle)


def scenarios_from_config(config: Dict[str, object]) -> List[Scenario]:
    return [
        Scenario(
            name=str(item["name"]),
            n_flows=int(item["n_flows"]),
            switch_buffer_bdp=float(item["switch_buffer_bdp"]),
            seed=int(item["seed"]),
            service_rate=str(item.get("service_rate", "")),
            capacity_scale=float(item.get("capacity_scale", 1.0)),
        )
        for item in config["scenarios"]  # type: ignore[index]
    ]


def split_csv(text: str) -> List[str]:
    return [part.strip() for part in str(text).split(",") if part.strip()]


def resolve_trace(
    config: Dict[str, object],
    inputs_dir: Path,
    override: str,
    download: bool,
) -> Path:
    trace_cfg = config["trace"]  # type: ignore[index]
    if override:
        trace_path = Path(override).expanduser().resolve()
        if not trace_path.exists():
            raise FileNotFoundError(f"trace path does not exist: {trace_path}")
        return trace_path

    legacy_path = Path(str(trace_cfg["legacy_project_path"])).expanduser()
    if legacy_path.exists():
        return legacy_path.resolve()

    output = inputs_dir / str(trace_cfg["upstream_file"])
    if output.exists():
        return output
    if not download:
        raise FileNotFoundError(
            f"missing legacy trace {legacy_path}; re-run without --no-download-trace"
        )

    url = (
        f"https://raw.githubusercontent.com/Soheil-ab/Cellular-Traces-NYC/"
        f"{trace_cfg['upstream_commit']}/{trace_cfg['upstream_file']}"
    )
    print(f"Downloading cellular trace: {url}")
    with urllib.request.urlopen(url, timeout=60) as response:
        output.write_bytes(response.read())
    return output


def read_trace_duration_s(path: Path) -> float:
    first: Optional[float] = None
    last: Optional[float] = None
    with path.open("r", encoding="utf-8", errors="replace") as handle:
        for line in handle:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            value = float(line.split()[0])
            if first is None:
                first = value
            last = value
    if last is None:
        raise ValueError(f"trace has no Mahimahi timestamps: {path}")
    return last / 1000.0


def run_capacity_converter(converter: Path, trace_path: Path, output: Path) -> None:
    with output.open("w", encoding="utf-8") as handle:
        subprocess.run(
            ["bash", str(converter), str(trace_path)],
            cwd=str(converter.parent),
            text=True,
            stdout=handle,
            stderr=subprocess.PIPE,
            check=True,
        )


def load_capacity_rows(path: Path, sim_time_s: float) -> List[Tuple[float, float]]:
    rows: List[Tuple[float, float]] = []
    with path.open("r", encoding="utf-8", errors="replace") as handle:
        for line in handle:
            parts = line.split()
            if len(parts) < 2:
                continue
            time_s = float(parts[0]) / 1000.0
            rate_mbps = float(parts[1])
            if time_s <= sim_time_s + 1e-12:
                rows.append((time_s, rate_mbps))
    if not rows:
        raise ValueError(f"capacity converter produced no rows for {path}")
    return rows


def scale_capacity_rows(
    rows: Sequence[Tuple[float, float]],
    scale: float,
) -> List[Tuple[float, float]]:
    if scale <= 0.0:
        raise ValueError(f"capacity_scale must be positive, got {scale}")
    return [(time_s, rate_mbps * scale) for time_s, rate_mbps in rows]


def write_capacity_schedule_files(
    rows: Sequence[Tuple[float, float]],
    output_csv: Path,
    output_ns3: Path,
) -> str:
    entries: List[str] = [f"0:{rows[0][1]:.9g}Mbps"]
    for time_s, rate_mbps in rows:
        if time_s <= 0.0:
            continue
        entries.append(f"{time_s:.6f}:{rate_mbps:.9g}Mbps")
    schedule = ",".join(entries)

    with output_csv.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(["time_s", "rate_mbps"])
        writer.writerow(["0.000000", f"{rows[0][1]:.9g}"])
        for time_s, rate_mbps in rows:
            if time_s > 0.0:
                writer.writerow([f"{time_s:.6f}", f"{rate_mbps:.9g}"])
    output_ns3.write_text(schedule + "\n", encoding="utf-8")
    return schedule


def shell_join(parts: Iterable[str]) -> str:
    return " ".join(shlex.quote(str(part)) for part in parts)


def build_common_args(
    args: argparse.Namespace,
    config: Dict[str, object],
    scenario: Scenario,
    capacity_schedule: str,
    ccs: Sequence[str],
) -> List[str]:
    link = config["link"]  # type: ignore[index]
    defaults = config["runner_defaults"]  # type: ignore[index]
    service_rate = scenario.service_rate or str(link["nominal_service_rate"])
    command = [
        "--n-flows",
        str(scenario.n_flows),
        "--sim-time",
        str(args.sim_time),
        "--seed",
        str(scenario.seed),
        "--run-id",
        str(args.run_id),
        "--start-times",
        "0",
        "--stop-times",
        str(args.sim_time),
        "--access-rate",
        str(link["access_rate"]),
        "--service-rate",
        service_rate,
        "--capacity-schedule",
        capacity_schedule,
        "--access-delay-ms",
        str(link["access_delay_ms"]),
        "--service-delay-ms",
        str(link["service_delay_ms"]),
        "--switch-buffer-bdp",
        str(scenario.switch_buffer_bdp),
        "--data-generator-batch",
        str(args.data_generator_batch or defaults["data_generator_batch"]),
        "--stream-buffer-bytes",
        str(args.stream_buffer_bytes if args.stream_buffer_bytes is not None else defaults["stream_buffer_bytes"]),
        "--queue-sample-interval-us",
        str(args.queue_sample_interval_us or defaults["queue_sample_interval_us"]),
        "--warmup-s",
        str(args.warmup_s if args.warmup_s is not None else defaults["warmup_s"]),
        "--only-cc",
        ",".join(ccs),
    ]
    if args.no_enable_queue_trace:
        command.append("--no-enable-queue-trace")
    else:
        command.append("--enable-queue-trace")
    if args.direct_binary:
        command.append("--direct-binary")
    return command


def scenario_complete(path: Path, ccs: Sequence[str]) -> bool:
    if not (path / "compare" / "phase_summary_metrics.csv").exists():
        return False
    for cc in ccs:
        rc = path / cc / "return_code.txt"
        if not rc.exists() or rc.read_text(encoding="utf-8").strip() != "0":
            return False
    return True


def run_build(root: Path, dry_run: bool) -> int:
    if dry_run:
        return 0
    build_log = root / "build.log"
    with build_log.open("w", encoding="utf-8") as handle:
        proc = subprocess.run(
            ["./waf", "build"],
            cwd=str(NS3_ROOT),
            text=True,
            stdout=handle,
            stderr=subprocess.STDOUT,
            check=False,
        )
    return proc.returncode


def run_scenario(
    root: Path,
    config: Dict[str, object],
    scenario: Scenario,
    args: argparse.Namespace,
    capacity_schedule: str,
    ccs: Sequence[str],
    batch_log,
) -> int:
    scenario_dir = root / scenario.name
    if args.skip_completed and scenario_complete(scenario_dir, ccs):
        message = f"[skip] completed {scenario.name}"
        print(message)
        batch_log.write(message + "\n")
        return 0

    scenario_dir.mkdir(parents=True, exist_ok=True)
    common_args = build_common_args(args, config, scenario, capacity_schedule, ccs)

    scenario_meta = {
        "scenario": scenario.__dict__,
        "sim_time_s": args.sim_time,
        "ccs": list(ccs),
        "common_runner_args_without_only_cc_override": common_args,
    }
    (scenario_dir / "scenario_config.json").write_text(
        json.dumps(scenario_meta, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )

    for cc in ccs:
        cmd = [
            sys.executable,
            "-B",
            str(RUNNER),
            "--experiment-dir",
            str(scenario_dir),
            "--only-cc",
            cc,
            "--skip-completed",
            "--no-plot",
            *[part for part in common_args if part != "--only-cc" and part != ",".join(ccs)],
        ]
        command_text = shell_join(cmd)
        batch_log.write(f"[{scenario.name}][{cc}] {command_text}\n")
        batch_log.flush()
        if args.dry_run:
            print(command_text)
            continue

        driver_log_path = scenario_dir / f"{cc}_driver.log"
        with driver_log_path.open("a", encoding="utf-8") as driver_log:
            driver_log.write("\n=== command ===\n" + command_text + "\n")
            driver_log.flush()
            proc = subprocess.run(
                cmd,
                cwd=str(SCRIPT_DIR),
                text=True,
                stdout=driver_log,
                stderr=subprocess.STDOUT,
                check=False,
            )
            driver_log.write(f"\n=== return_code: {proc.returncode} ===\n")
        message = f"[{scenario.name}] {cc} return_code={proc.returncode}"
        print(message)
        batch_log.write(message + "\n")
        if proc.returncode != 0 and not args.continue_on_error:
            return proc.returncode

    if args.dry_run or args.no_plot:
        return 0

    plot_cmd = [
        sys.executable,
        "-B",
        str(RUNNER),
        "--experiment-dir",
        str(scenario_dir),
        "--skip-run",
        *common_args,
    ]
    plot_text = shell_join(plot_cmd)
    with (scenario_dir / "plot_driver.log").open("a", encoding="utf-8") as plot_log:
        plot_log.write("\n=== command ===\n" + plot_text + "\n")
        plot_log.flush()
        proc = subprocess.run(
            plot_cmd,
            cwd=str(SCRIPT_DIR),
            text=True,
            stdout=plot_log,
            stderr=subprocess.STDOUT,
            check=False,
        )
        plot_log.write(f"\n=== return_code: {proc.returncode} ===\n")
    batch_log.write(f"[{scenario.name}][plot] return_code={proc.returncode}\n")
    batch_log.flush()
    return proc.returncode


def write_root_metadata(
    root: Path,
    config: Dict[str, object],
    args: argparse.Namespace,
    trace_path: Path,
    capacity_rows: Sequence[Tuple[float, float]],
    ccs: Sequence[str],
    scenarios: Sequence[Scenario],
) -> None:
    scripts_dir = root / "scripts"
    scripts_dir.mkdir(parents=True, exist_ok=True)
    for path in [
        CONFIG_PATH,
        Path(__file__).resolve(),
        SCRIPT_DIR / "run_cellular_links_5cc_matrix.sh",
        RUNNER,
        SCRIPT_DIR / "plot_4cc_comparison.py",
    ]:
        shutil.copy2(path, scripts_dir / path.name)

    metadata = {
        "created_at": datetime.now().isoformat(timespec="seconds"),
        "root": str(root),
        "config": config,
        "selected_ccs": list(ccs),
        "selected_scenarios": [scenario.__dict__ for scenario in scenarios],
        "sim_time_s": args.sim_time,
        "trace_path_used": str(trace_path),
        "base_capacity_schedule_points": len(capacity_rows) + 1,
        "notes": [
            "The raw Mahimahi trace is the experiment source of truth.",
            "The ns-3 capacitySchedule is generated from the raw trace with the project catCapacityTrace.sh converter.",
            "Scenario capacity_scale preserves the trace timing/shape while scaling the configured cellular service rate.",
        ],
    }
    (root / "batch_config.json").write_text(
        json.dumps(metadata, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )


def make_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default=str(CONFIG_PATH))
    parser.add_argument("--log-root", default=str(DEFAULT_LOG_ROOT))
    parser.add_argument("--experiment-dir", default="")
    parser.add_argument("--trace-path", default="", help="Override raw Mahimahi uplink trace path.")
    parser.add_argument("--no-download-trace", action="store_true")
    parser.add_argument("--sim-time", type=float, default=0.0, help="Seconds. 0 uses config default, --full-trace uses the trace end time.")
    parser.add_argument("--full-trace", action="store_true", help="Run until the raw trace ends.")
    parser.add_argument("--allow-tail-hold", action="store_true", help="Allow sim-time beyond trace end; ns-3 will hold the last capacity.")
    parser.add_argument("--only-cc", default="", help="Comma-separated subset of configured CCs.")
    parser.add_argument("--start-scenario", type=int, default=1)
    parser.add_argument("--only-scenario", default="", help="Comma-separated scenario names.")
    parser.add_argument("--run-id", type=int, default=1)
    parser.add_argument("--data-generator-batch", type=int, default=0)
    parser.add_argument("--stream-buffer-bytes", type=int, default=None)
    parser.add_argument("--queue-sample-interval-us", type=int, default=0)
    parser.add_argument("--warmup-s", type=float, default=None)
    parser.add_argument("--enable-queue-trace", dest="no_enable_queue_trace", action="store_false")
    parser.add_argument("--no-enable-queue-trace", dest="no_enable_queue_trace", action="store_true")
    parser.set_defaults(no_enable_queue_trace=True)
    parser.add_argument("--direct-binary", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--skip-completed", action="store_true")
    parser.add_argument("--continue-on-error", action="store_true")
    parser.add_argument("--no-plot", action="store_true")
    parser.add_argument("--dry-run", action="store_true")
    return parser


def main(argv: Optional[List[str]] = None) -> int:
    args = make_parser().parse_args(argv)
    config = load_config(Path(args.config).expanduser().resolve())
    trace_cfg = config["trace"]  # type: ignore[index]

    if args.experiment_dir:
        root = Path(args.experiment_dir).expanduser().resolve()
    else:
        name = datetime.now().strftime("cellular_links_5cc_matrix_%Y%m%d_%H%M%S")
        root = unique_dir(Path(args.log_root).expanduser().resolve() / name)
    root.mkdir(parents=True, exist_ok=True)
    inputs_dir = root / "inputs"
    inputs_dir.mkdir(parents=True, exist_ok=True)

    trace_path = resolve_trace(
        config,
        inputs_dir,
        args.trace_path,
        download=not args.no_download_trace,
    )
    archived_trace_raw = inputs_dir / str(trace_cfg["upstream_file"])
    if trace_path.resolve() != archived_trace_raw.resolve():
        shutil.copy2(trace_path, archived_trace_raw)
    archived_trace = inputs_dir / "trace_taxi"
    if archived_trace_raw.resolve() != archived_trace.resolve():
        shutil.copy2(archived_trace_raw, archived_trace)

    converter = Path(str(trace_cfg["project_capacity_converter"]))
    capacity_text = inputs_dir / "cap_cellular_taxi.txt"
    run_capacity_converter(converter, archived_trace, capacity_text)
    shutil.copy2(converter, inputs_dir / converter.name)

    downlink = Path(str(trace_cfg["constant_downlink_trace"]))
    if downlink.exists():
        shutil.copy2(downlink, inputs_dir / downlink.name)

    trace_duration_s = read_trace_duration_s(archived_trace)
    if args.full_trace:
        args.sim_time = trace_duration_s
    elif args.sim_time <= 0.0:
        args.sim_time = float(trace_cfg["default_sim_time_s"])
    if args.sim_time > trace_duration_s and not args.allow_tail_hold:
        print(
            f"config error: sim-time {args.sim_time:.3f}s exceeds raw trace "
            f"{trace_duration_s:.3f}s; use --allow-tail-hold to override",
            file=sys.stderr,
        )
        return 2

    capacity_rows = load_capacity_rows(capacity_text, args.sim_time)
    write_capacity_schedule_files(
        capacity_rows,
        inputs_dir / "capacity_schedule_base_unscaled.csv",
        inputs_dir / "capacity_schedule_base_unscaled.ns3.txt",
    )

    configured_ccs = [str(item) for item in config["ccs"]]  # type: ignore[index]
    ccs = split_csv(args.only_cc) if args.only_cc else configured_ccs
    invalid_ccs = [cc for cc in ccs if cc not in configured_ccs]
    if invalid_ccs:
        print(f"config error: unsupported CCs: {', '.join(invalid_ccs)}", file=sys.stderr)
        return 2

    scenarios = scenarios_from_config(config)
    if args.only_scenario:
        wanted = set(split_csv(args.only_scenario))
        scenarios = [scenario for scenario in scenarios if scenario.name in wanted]
    scenarios = [
        scenario
        for index, scenario in enumerate(scenarios, start=1)
        if index >= args.start_scenario
    ]
    if not scenarios:
        print("config error: no scenarios selected", file=sys.stderr)
        return 2

    scenario_capacity_schedules: Dict[str, str] = {}
    for scenario in scenarios:
        scaled_rows = scale_capacity_rows(capacity_rows, scenario.capacity_scale)
        scenario_capacity_schedules[scenario.name] = write_capacity_schedule_files(
            scaled_rows,
            inputs_dir / f"{scenario.name}_capacity_schedule.csv",
            inputs_dir / f"{scenario.name}_capacity_schedule.ns3.txt",
        )

    write_root_metadata(root, config, args, archived_trace, capacity_rows, ccs, scenarios)
    print(f"Experiment root: {root}")
    print(f"Raw Mahimahi trace: {archived_trace}")
    print(f"Trace duration: {trace_duration_s:.3f}s; sim-time: {args.sim_time:.3f}s")
    print(f"CCs: {', '.join(ccs)}")
    print(f"Scenarios: {', '.join(scenario.name for scenario in scenarios)}")

    build_rc = run_build(root, args.dry_run)
    if build_rc != 0:
        print(f"ns-3 build failed: {build_rc}", file=sys.stderr)
        return build_rc

    with (root / "batch.log").open("a", encoding="utf-8") as batch_log:
        for scenario in scenarios:
            rc = run_scenario(
                root,
                config,
                scenario,
                args,
                scenario_capacity_schedules[scenario.name],
                ccs,
                batch_log,
            )
            if rc != 0:
                print(f"scenario failed: {scenario.name}, return_code={rc}", file=sys.stderr)
                return rc

    print(f"Cellular-links matrix complete: {root}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
