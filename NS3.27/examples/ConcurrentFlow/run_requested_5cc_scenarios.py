#!/usr/bin/env python3
"""Run the requested five-CC, six-scenario comparison matrix."""

import argparse
import csv
import json
import shlex
import subprocess
import sys
from dataclasses import asdict, dataclass
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Optional


SCRIPT_DIR = Path(__file__).resolve().parent
RUNNER = SCRIPT_DIR / "run_4cc_comparison.py"
NS3_ROOT = SCRIPT_DIR.parents[1]
DEFAULT_LOG_ROOT = Path("/mnt/nasDisk_ds3617/wkd/1FreqBBR")
DEFAULT_FBBR_CONFIG = NS3_ROOT / "examples" / "CCconfig" / "fbbr_default.conf"
CCS = (
    "FBBR-adaptive",
    "FBBR",
    "BBRv2",
    "BBRv2plus",
    "oBBR",
)


@dataclass(frozen=True)
class Scenario:
    directory: str
    description: str
    sim_time_s: int
    seed: int
    n_flows: int
    service_rate: str
    start_times: str
    stop_times: str


SCENARIOS = (
    Scenario(
        "scenario1_4flows_100M_5BDP",
        "4条流从0s同步启动并持续到300s；共享瓶颈100Mbps；buffer=5BDP",
        300,
        1,
        4,
        "100Mbps",
        "0",
        "300",
    ),
    Scenario(
        "scenario2_8flows_500M_5BDP_300s_seed2",
        "8条流从0s同步启动并持续到300s；共享瓶颈500Mbps；buffer=5BDP；seed=2补跑",
        300,
        2,
        8,
        "500Mbps",
        "0",
        "300",
    ),
    Scenario(
        "scenario3_16flows_100M_5BDP",
        "16条流从0s同步启动并持续到480s；共享瓶颈100Mbps；buffer=5BDP；两个FBBR算法幅度=2sr",
        480,
        3,
        16,
        "100Mbps",
        "0",
        "480",
    ),
    Scenario(
        "scenario4_32flows_500M_5BDP",
        "32条流从0s同步启动并持续到480s；共享瓶颈500Mbps；buffer=5BDP；两个FBBR算法幅度=2sr",
        480,
        4,
        32,
        "500Mbps",
        "0",
        "480",
    ),
    Scenario(
        "scenario5_4plus4_join_100M_5BDP_600s",
        "共8条流：前4条0s启动，后4条300s加入，全部600s停止；共享瓶颈100Mbps；buffer=5BDP",
        600,
        5,
        8,
        "100Mbps",
        "0,0,0,0,300,300,300,300",
        "600",
    ),
    Scenario(
        "scenario6_8minus4_exit_500M_5BDP_600s_u64goodput",
        "共8条流：全部0s启动，后4条300s退出，前4条持续到600s；共享瓶颈500Mbps；buffer=5BDP；64位goodput累计基线修复",
        600,
        6,
        8,
        "500Mbps",
        "0",
        "600,600,600,600,300,300,300,300",
    ),
)


def unique_dir(path: Path) -> Path:
    if not path.exists():
        return path
    for index in range(2, 1000):
        candidate = path.with_name(f"{path.name}_{index:02d}")
        if not candidate.exists():
            return candidate
    raise RuntimeError(f"无法创建唯一目录：{path}")


def scenario_complete(path: Path) -> bool:
    return all(
        (path / cc / "return_code.txt").exists()
        and (path / cc / "return_code.txt").read_text(encoding="utf-8").strip() == "0"
        for cc in CCS
    ) and (path / "compare" / "phase_summary_metrics.csv").exists()


def run_scenario(root: Path, scenario: Scenario, batch_log) -> int:
    scenario_dir = root / scenario.directory
    if scenario_complete(scenario_dir):
        message = f"[skip] 已完成：{scenario.directory}\n"
        print(message, end="")
        batch_log.write(message)
        batch_log.flush()
        return 0

    scenario_dir.mkdir(parents=True, exist_ok=True)
    common_args = [
        "--n-flows",
        str(scenario.n_flows),
        "--sim-time",
        str(scenario.sim_time_s),
        "--seed",
        str(scenario.seed),
        "--start-times",
        scenario.start_times,
        "--stop-times",
        scenario.stop_times,
        "--service-rate",
        scenario.service_rate,
        "--switch-buffer-bdp",
        "5",
        "--data-generator-batch",
        "64",
        "--no-enable-queue-trace",
        "--queue-sample-interval-us",
        "1000",
        "--warmup-s",
        "5",
    ]
    batch_log.write(f"\n=== {scenario.directory} ===\n")
    batch_log.flush()
    print(f"\n开始场景：{scenario.description}")
    use_2sr = scenario.n_flows in {16, 32}
    freq_config_args = (
        ["--fbbr-config", str(root / "fbbr_2sr.conf")]
        if use_2sr
        else []
    )

    processes = []
    for cc in CCS:
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
            "--direct-binary",
            *(freq_config_args if cc.startswith("FBBR") else []),
            *common_args,
        ]
        driver_log_path = scenario_dir / f"{cc}_driver.log"
        driver_log = driver_log_path.open("a", encoding="utf-8")
        command_text = " ".join(shlex.quote(part) for part in cmd)
        driver_log.write(f"\n=== command ===\n{command_text}\n")
        driver_log.flush()
        batch_log.write(f"[{cc}] {command_text}\n")
        proc = subprocess.Popen(
            cmd,
            cwd=str(SCRIPT_DIR),
            text=True,
            stdout=driver_log,
            stderr=subprocess.STDOUT,
        )
        processes.append((cc, proc, driver_log))

    return_codes: Dict[str, int] = {}
    for cc, proc, driver_log in processes:
        return_code = proc.wait()
        driver_log.write(f"\n=== return_code: {return_code} ===\n")
        driver_log.close()
        return_codes[cc] = return_code
        message = f"[{scenario.directory}] {cc} 完成，返回码 {return_code}"
        print(message)
        batch_log.write(message + "\n")

    failed = {cc: code for cc, code in return_codes.items() if code != 0}
    if failed:
        batch_log.write(f"=== failed: {failed} ===\n")
        batch_log.flush()
        return next(iter(failed.values()))

    plot_cmd = [
        sys.executable,
        "-B",
        str(RUNNER),
        "--experiment-dir",
        str(scenario_dir),
        "--only-cc",
        ",".join(CCS),
        "--skip-run",
        *freq_config_args,
        *common_args,
    ]
    plot_log_path = scenario_dir / "plot_driver.log"
    with plot_log_path.open("a", encoding="utf-8") as plot_log:
        plot_log.write(
            "\n=== command ===\n"
            + " ".join(shlex.quote(part) for part in plot_cmd)
            + "\n"
        )
        plot_log.flush()
        plot_proc = subprocess.run(
            plot_cmd,
            cwd=str(SCRIPT_DIR),
            text=True,
            stdout=plot_log,
            stderr=subprocess.STDOUT,
            check=False,
        )
        plot_log.write(f"\n=== return_code: {plot_proc.returncode} ===\n")
    batch_log.write(f"=== plot_return_code: {plot_proc.returncode} ===\n")
    batch_log.flush()
    return plot_proc.returncode


def build_ns3(root: Path) -> int:
    build_log_path = root / "build.log"
    with build_log_path.open("w", encoding="utf-8") as build_log:
        proc = subprocess.run(
            ["./waf", "build"],
            cwd=str(NS3_ROOT),
            text=True,
            stdout=build_log,
            stderr=subprocess.STDOUT,
            check=False,
        )
    return proc.returncode


def write_2sr_config(root: Path) -> Path:
    source = DEFAULT_FBBR_CONFIG.read_text(encoding="utf-8")
    old = "default_amplitude_mode = 4sr"
    if old not in source:
        raise RuntimeError(f"默认配置中找不到待替换项：{old}")
    output = root / "fbbr_2sr.conf"
    output.write_text(
        source.replace(old, "default_amplitude_mode = 2sr", 1),
        encoding="utf-8",
    )
    return output


def read_phase_results(path: Path) -> Dict[str, List[Dict[str, str]]]:
    result: Dict[str, List[Dict[str, str]]] = {cc: [] for cc in CCS}
    with path.open("r", newline="", encoding="utf-8") as fh:
        for row in csv.DictReader(fh):
            cc = row["cc"]
            if cc in result:
                result[cc].append(row)
    return result


def compact_result(rows: List[Dict[str, str]]) -> str:
    parts = []
    for row in rows:
        phase = int(row["phase_index"])
        active = int(row["active_flows"])
        throughput = float(row["avg_aggregate_throughput_mbps"])
        utilization = float(row["avg_bottleneck_utilization"]) * 100.0
        avg_queue = float(row["avg_queue_delay_ms"])
        p95_queue = float(row["p95_queue_delay_ms"])
        fairness = float(row["avg_jain_fairness"])
        parts.append(
            f"阶段{phase}({active}流): 吞吐{throughput:.2f}Mbps, "
            f"利用率{utilization:.1f}%, 队列{avg_queue:.2f}/{p95_queue:.2f}ms(均值/P95), "
            f"Jain={fairness:.3f}"
        )
    return " | ".join(parts)


def write_summary(root: Path) -> Path:
    phase_results: Dict[str, Optional[Dict[str, List[Dict[str, str]]]]] = {}
    for scenario in SCENARIOS:
        phase_path = (
            root / scenario.directory / "compare" / "phase_summary_metrics.csv"
        )
        phase_results[scenario.directory] = (
            read_phase_results(phase_path) if phase_path.exists() else None
        )
    output = root / "all_scenarios_summary.csv"
    base = "场景1/2为300s，场景3/4为480s，场景5/6为600s；动态流在300s切换；RTT约42ms；access=1Gbps；长流；buffer=5BDP；队列1ms采样；场景3/4的FBBR幅度=2sr；seed按场景为1,2,3,4,5,6；runId=1"
    headers = ["基础设置", "对比CC"]
    for index in range(1, len(SCENARIOS) + 1):
        headers.extend([f"场景{index}", f"结果{index}"])
    with output.open("w", newline="", encoding="utf-8-sig") as fh:
        writer = csv.writer(fh)
        writer.writerow(headers)
        for cc in CCS:
            row = [base, cc]
            for scenario in SCENARIOS:
                scenario_results = phase_results[scenario.directory]
                row.extend([
                    scenario.description,
                    (
                        compact_result(scenario_results[cc])
                        if scenario_results is not None
                        else "沿用此前已完成结果；本批次按要求未重跑，未复制数值"
                    ),
                ])
            writer.writerow(row)
    return output


def make_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--log-root", default=str(DEFAULT_LOG_ROOT))
    parser.add_argument("--experiment-dir", default="")
    parser.add_argument("--start-scenario", type=int, choices=range(1, 7), default=1)
    return parser


def main(argv: Optional[List[str]] = None) -> int:
    args = make_parser().parse_args(argv)
    if args.experiment_dir:
        root = Path(args.experiment_dir).expanduser().resolve()
    else:
        name = datetime.now().strftime("five_cc_six_scenarios_%Y%m%d_%H%M%S")
        root = unique_dir(Path(args.log_root).expanduser().resolve() / name)
    root.mkdir(parents=True, exist_ok=True)
    print(f"实验总目录：{root}")
    two_sr_config = write_2sr_config(root)

    (root / "batch_config.json").write_text(
        json.dumps(
            {
                "ccs": list(CCS),
                "scenarios": [asdict(scenario) for scenario in SCENARIOS],
                "fbbr_2sr_config": str(two_sr_config),
                "fbbr_2sr_scenarios": [
                    scenario.directory
                    for scenario in SCENARIOS
                    if scenario.n_flows in {16, 32}
                ],
            },
            ensure_ascii=False,
            indent=2,
        )
        + "\n",
        encoding="utf-8",
    )

    build_return_code = build_ns3(root)
    if build_return_code != 0:
        print(f"ns-3 编译失败，返回码 {build_return_code}", file=sys.stderr)
        return build_return_code

    with (root / "batch.log").open("a", encoding="utf-8") as batch_log:
        for scenario_index, scenario in enumerate(SCENARIOS, start=1):
            if scenario_index < args.start_scenario:
                message = f"[forced-skip] 场景 {scenario_index}：{scenario.directory}"
                print(message)
                batch_log.write(message + "\n")
                continue
            return_code = run_scenario(root, scenario, batch_log)
            if return_code != 0:
                print(f"场景失败：{scenario.directory}，返回码 {return_code}", file=sys.stderr)
                return return_code

    summary = write_summary(root)
    print(f"\n全部实验完成：{root}")
    print(f"总汇总 CSV：{summary}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
