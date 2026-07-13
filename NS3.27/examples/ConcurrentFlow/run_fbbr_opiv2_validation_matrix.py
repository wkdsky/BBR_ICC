#!/usr/bin/env python3
"""Run the frozen F-BBR OPIv2 E1--E9 validation matrix."""

from __future__ import annotations

import argparse
import csv
import json
import os
import shlex
import subprocess
import time
from concurrent.futures import ThreadPoolExecutor, as_completed
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import pandas as pd


NS3_ROOT = Path(__file__).resolve().parents[2]
BINARY = NS3_ROOT / "build/scratch/generic_p2p_switch_flows"
CONFIG = NS3_ROOT / "examples/CCconfig/fbbr_opiv2_validation.conf"
DEFAULT_ROOT = Path(
    "/mnt/nasDisk_ds3617/wkd/1FreqBBR/"
    "fbbr_opiv2_validation_20260711/matrix"
)


@dataclass(frozen=True)
class Scenario:
    key: str
    directory: str
    n_flows: int
    sim_time: int
    starts: str = "0"
    stops: str = ""
    limits: str = "0"
    background: str = ""
    capacity: str = ""


SCENARIOS = {
    "E1": Scenario("E1", "E1_n1_steady", 1, 180),
    "E2": Scenario("E2", "E2_n2_steady", 2, 180),
    "E3": Scenario("E3", "E3_n4_steady", 4, 180),
    "E4": Scenario("E4", "E4_n8_steady", 8, 180),
    "E5": Scenario(
        "E5", "E5_active_set_step", 4, 220,
        starts="0,40,80,80", stops="220,180,140,140"
    ),
    "E6": Scenario(
        "E6", "E6_background_step", 2, 200,
        background="0:0Mbps,40:20Mbps,80:40Mbps,120:60Mbps,160:0Mbps"
    ),
    "E7": Scenario(
        "E7", "E7_underload_control", 4, 180,
        limits="15Mbps,15Mbps,15Mbps,15Mbps"
    ),
    "E8": Scenario(
        "E8", "E8_persistent_queue_control", 4, 180,
        limits="24Mbps,24Mbps,24Mbps,24Mbps", background="0:4Mbps"
    ),
    "E9": Scenario(
        "E9", "E9_dynamic_capacity_control", 4, 180,
        capacity=(
            "0:100Mbps,20:70Mbps,40:100Mbps,60:70Mbps,80:100Mbps,"
            "100:70Mbps,120:100Mbps,140:70Mbps,160:100Mbps"
        ),
    ),
}


def parse_seeds(text: str) -> list[int]:
    values: set[int] = set()
    for part in text.split(","):
        part = part.strip()
        if not part:
            continue
        if "-" in part:
            lo, hi = (int(x) for x in part.split("-", 1))
            values.update(range(lo, hi + 1))
        else:
            values.add(int(part))
    if not values or min(values) <= 0:
        raise argparse.ArgumentTypeError("seeds must be positive")
    return sorted(values)


def select_scenarios(text: str) -> list[Scenario]:
    if text.lower() == "all":
        return list(SCENARIOS.values())
    keys = [x.strip().upper() for x in text.split(",") if x.strip()]
    missing = [x for x in keys if x not in SCENARIOS]
    if missing:
        raise ValueError(f"unknown scenarios: {','.join(missing)}")
    return [SCENARIOS[x] for x in keys]


def build_command(
    s: Scenario, seed: int, run_dir: Path, buffer_bdp: float
) -> list[str]:
    stops = s.stops or str(s.sim_time)
    args = [
        str(BINARY),
        f"--nFlows={s.n_flows}",
        "--algos=F-BBR",
        f"--simTime={s.sim_time}",
        "--serviceRate=100Mbps",
        "--accessRate=1Gbps",
        "--accessDelayMs=1",
        "--serviceDelayMs=23",
        f"--switchBufferBdp={buffer_bdp:g}",
        f"--flowStartTimes={s.starts}",
        f"--flowStopTimes={stops}",
        f"--perFlowAppRateLimits={s.limits}",
        "--processIntervalUs=100",
        "--goodputIntervalMs=100",
        "--dataGeneratorBatch=2",
        "--useEngineTimer=true",
        "--enableTrace=true",
        "--enableHeavyTrace=false",
        "--enableQueueTrace=false",
        "--enableConvergenceGateTrace=false",
        "--emitRunMeta=true",
        "--emitBottleneckQueueTrace=true",
        "--queueSampleIntervalUs=200",
        f"--fbbrConfig={CONFIG}",
        f"--tracePath={run_dir}/",
        "--traceName=fbbr_validation",
        f"--seed={seed}",
        f"--runId={seed}",
    ]
    if s.background:
        args.append(f"--backgroundRateSchedule={s.background}")
    if s.capacity:
        args.append(f"--capacitySchedule={s.capacity}")
    return args


def csv_width_error(path: Path) -> str | None:
    try:
        with path.open(newline="") as handle:
            rows = csv.reader(handle)
            header = next(rows, None)
            if not header:
                return "empty_csv"
            width = len(header)
            for line_no, row in enumerate(rows, 2):
                if len(row) != width:
                    return f"csv_width_line_{line_no}_{len(row)}_expected_{width}"
    except Exception as exc:  # pragma: no cover - retained in manifest
        return f"csv_read_error:{exc}"
    return None


def check_run(s: Scenario, run_dir: Path) -> tuple[bool, str]:
    required = [run_dir / "run_meta.json", run_dir / "bottleneck_queue.csv"]
    for flow in range(1, s.n_flows + 1):
        required.extend(
            [
                run_dir / f"flow{flow}_fbbr_opiv2_blocks.csv",
                run_dir / f"flow{flow}_fbbr_opiv2_shadow_windows.csv",
                run_dir / f"flow{flow}_fbbr_opiv2_cruises.csv",
                run_dir / f"fbbr_validation_flow{flow}_F-BBR_good.txt",
            ]
        )
    missing = [p.name for p in required if not p.exists() or p.stat().st_size == 0]
    if missing:
        return False, "missing:" + ";".join(missing)
    try:
        json.loads((run_dir / "run_meta.json").read_text())
    except Exception as exc:
        return False, f"invalid_run_meta:{exc}"
    for path in run_dir.glob("*.csv"):
        error = csv_width_error(path)
        if error:
            return False, f"{path.name}:{error}"
    return True, "ok"


def augment_run_meta(run_dir: Path) -> None:
    """Add sender-observed frozen RTprop after the simulation has completed."""
    path = run_dir / "run_meta.json"
    if not path.exists():
        return
    meta = json.loads(path.read_text())
    values: list[float] = []
    for block in run_dir.glob("flow*_fbbr_opiv2_blocks.csv"):
        try:
            frame = pd.read_csv(block, usecols=["rtprop_frozen_us"])
            values.extend(pd.to_numeric(frame.rtprop_frozen_us, errors="coerce").dropna() / 1e6)
        except (ValueError, pd.errors.EmptyDataError):
            continue
    if values:
        frozen = float(np.median(values))
        empty = float(meta.get("empty_path_rtt_s", meta.get("rtprop_s", frozen)))
        bdp = float(meta["capacity_bps"]) * empty / 8.0
        meta["sender_frozen_rtprop_s"] = frozen
        meta["measured_rtprop_s"] = empty
        meta["bdp_bytes"] = bdp
        meta["configured_buffer_ratio_measured"] = float(meta["buffer_bytes"]) / bdp
        meta["sender_rtprop_difference_ratio"] = (frozen - empty) / empty if empty else 0.0
    path.write_text(json.dumps(meta, indent=2) + "\n")


def completed(s: Scenario, run_dir: Path) -> bool:
    status = run_dir / "status.json"
    if not status.exists():
        return False
    try:
        saved = json.loads(status.read_text())
    except Exception:
        return False
    augment_run_meta(run_dir)
    valid, _ = check_run(s, run_dir)
    return saved.get("status") == "PASS" and valid


def run_one(
    s: Scenario,
    seed: int,
    output_root: Path,
    force: bool,
    buffer_bdp: float,
) -> dict[str, object]:
    run_dir = output_root / s.directory / f"seed{seed:03d}"
    run_dir.mkdir(parents=True, exist_ok=True)
    if not force and completed(s, run_dir):
        return {
            "scenario": s.key, "scenario_dir": s.directory, "seed": seed,
            "run_dir": str(run_dir), "status": "SKIPPED_COMPLETE",
            "exit_code": 0, "wall_seconds": 0.0, "integrity": "ok",
        }
    command = build_command(s, seed, run_dir, buffer_bdp)
    (run_dir / "command.txt").write_text(shlex.join(command) + "\n")
    started = time.time()
    with (run_dir / "stdout.log").open("w") as stdout, \
            (run_dir / "stderr.log").open("w") as stderr:
        env = os.environ.copy()
        env["LD_LIBRARY_PATH"] = str(NS3_ROOT / "build") + os.pathsep + env.get(
            "LD_LIBRARY_PATH", ""
        )
        proc = subprocess.run(
            command, cwd=NS3_ROOT, stdout=stdout, stderr=stderr, env=env
        )
    wall = time.time() - started
    if proc.returncode == 0:
        augment_run_meta(run_dir)
    valid, integrity = check_run(s, run_dir) if proc.returncode == 0 else (False, "not_checked")
    status = "PASS" if proc.returncode == 0 and valid else "FAIL"
    record: dict[str, object] = {
        "scenario": s.key, "scenario_dir": s.directory, "seed": seed,
        "run_dir": str(run_dir), "status": status,
        "exit_code": proc.returncode, "wall_seconds": round(wall, 6),
        "integrity": integrity,
    }
    (run_dir / "status.json").write_text(json.dumps(record, indent=2) + "\n")
    return record


def write_manifest(output_root: Path, records: list[dict[str, object]]) -> None:
    fields = [
        "scenario", "scenario_dir", "seed", "run_dir", "status",
        "exit_code", "wall_seconds", "integrity",
    ]
    with (output_root / "matrix_manifest.csv").open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        writer.writerows(sorted(records, key=lambda x: (str(x["scenario"]), int(x["seed"]))))


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--scenario", default="all", help="E1, E3 or all")
    parser.add_argument("--seeds", default="1-10")
    parser.add_argument("--jobs", type=int, default=2)
    parser.add_argument(
        "--buffer-bdp", type=float, default=8.0,
        help="Shared bottleneck buffer in BDP units (default: 8 for large-BDP validation)",
    )
    parser.add_argument("--resume", action="store_true", help="Retained for explicit CLI compatibility")
    parser.add_argument("--force", action="store_true")
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--output-root", type=Path, default=DEFAULT_ROOT)
    args = parser.parse_args()
    if args.buffer_bdp <= 0:
        parser.error("--buffer-bdp must be positive")

    scenarios = select_scenarios(args.scenario)
    seeds = parse_seeds(args.seeds)
    output_root = args.output_root.expanduser().resolve()
    output_root.mkdir(parents=True, exist_ok=True)
    if not BINARY.exists():
        parser.error(f"binary not found: {BINARY}; run ./waf build")

    jobs = [(s, seed) for s in scenarios for seed in seeds]
    if args.dry_run:
        for s, seed in jobs:
            run_dir = output_root / s.directory / f"seed{seed:03d}"
            print(shlex.join(build_command(s, seed, run_dir, args.buffer_bdp)))
        return 0

    records: list[dict[str, object]] = []
    with ThreadPoolExecutor(max_workers=max(1, args.jobs)) as pool:
        futures = {
            pool.submit(
                run_one, s, seed, output_root, args.force, args.buffer_bdp
            ): (s, seed)
            for s, seed in jobs
        }
        for future in as_completed(futures):
            record = future.result()
            records.append(record)
            write_manifest(output_root, records)
            print(
                f"[{record['status']}] {record['scenario']} seed={record['seed']} "
                f"wall={record['wall_seconds']}s {record['integrity']}",
                flush=True,
            )
    return 0 if all(r["status"] in {"PASS", "SKIPPED_COMPLETE"} for r in records) else 1


if __name__ == "__main__":
    raise SystemExit(main())
