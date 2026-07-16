#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Run a composite 4-flow / 4-CC comparison experiment.

Directory layout:
  /mnt/nasDisk_ds3617/wkd/1FreqBBR/<experiment>/
    BBRv2/
    oBBR/
    BBRv2plus/
    F-BBR/
    compare/
"""

import argparse
import json
import os
import re
import shlex
import subprocess
import sys
from dataclasses import asdict, dataclass
from datetime import datetime
from pathlib import Path
from typing import Dict, Iterable, List, Optional


NS3_ROOT = Path(__file__).resolve().parents[2]
SCENARIO = "generic_p2p_switch_flows"
DEFAULT_FREQCCV4_CONFIG = NS3_ROOT / "examples" / "CCconfig" / "freqccv4_default.conf"
DEFAULT_LOG_ROOT = Path("/mnt/nasDisk_ds3617/wkd/1FreqBBR")

CCS = ("BBRv2", "oBBR", "BBRv2plus", "F-BBR")
SELECTABLE_CCS = (
    *CCS,
    "FreqCCv4",
    "FreqCCv4-adaptive",
    "FreqCCv4-hybrid",
)
PLOT_SCRIPT = Path(__file__).with_name("plot_4cc_comparison.py")


@dataclass
class RunnerConfig:
    n_flows: int = 4
    sim_time: float = 30.0
    algos: str = "BBRv2"
    start_times: str = "0"
    stop_times: str = ""
    initial_rates: str = "0"
    rate_schedules: str = ""
    freqccv4_cruise_base_caps: str = "0"
    access_rate: str = "1Gbps"
    service_rate: str = "20Mbps"
    access_delay_ms: float = 1.0
    service_delay_ms: float = 10.0
    prop_delay_ms: float = -1.0
    switch_buffer_bytes: int = 0
    switch_buffer_bdp: float = 1.0
    endpoint_queue_bytes: int = 1073741824
    flow_size_bytes: int = 0
    process_interval_us: int = 100
    goodput_interval_ms: int = 100
    use_engine_timer: bool = True
    enable_trace: bool = True
    enable_heavy_trace: bool = False
    enable_queue_trace: bool = True
    emit_bottleneck_queue_trace: bool = True
    queue_sample_interval_us: int = 200
    emulated_connections: int = 1
    data_generator_batch: int = 2
    stream_buffer_bytes: int = 0
    enable_convergence_gate_trace: bool = False
    enable_convergence_gate_control: bool = False
    gate_trace_mode: str = "round_only"
    gate_trace_sample_interval_us: int = 10000
    interval_win_rtt_mult: float = 1.0
    config_paths: str = ""
    freqccv4_config: str = str(DEFAULT_FREQCCV4_CONFIG)
    obbr_config: str = ""
    bbrv2plus_config: str = ""
    bbrv2_config: str = ""
    log_root: str = str(DEFAULT_LOG_ROOT)
    experiment_name: str = ""
    trace_path: str = ""
    trace_name: str = "generic_p2p_switch"
    seed: int = 1
    run_id: int = 1
    direct_binary: bool = False


def bool_to_ns3(value: bool) -> str:
    return "true" if value else "false"


def sanitize_path_part(value: str, fallback: str = "experiment") -> str:
    text = re.sub(r"[^0-9A-Za-z._-]+", "_", str(value).strip())
    text = text.strip("._-")
    return text or fallback


def unique_dir(path: Path) -> Path:
    if not path.exists():
        return path
    for idx in range(2, 1000):
        candidate = path.with_name(f"{path.name}_{idx:02d}")
        if not candidate.exists():
            return candidate
    raise RuntimeError(f"无法创建唯一实验目录，已有过多重名目录: {path}")


def prepare_experiment_dir(cfg: RunnerConfig, create: bool) -> Path:
    explicit_trace_path = bool(cfg.trace_path)
    if cfg.trace_path:
        experiment_dir = Path(cfg.trace_path).expanduser()
    else:
        root = Path(cfg.log_root).expanduser()
        name = sanitize_path_part(cfg.experiment_name) if cfg.experiment_name else sanitize_path_part(cfg.trace_name)
        experiment_dir = unique_dir(root / name)

    experiment_dir = experiment_dir.resolve()
    cfg.trace_path = str(experiment_dir)
    if create:
        try:
            experiment_dir.mkdir(parents=True, exist_ok=explicit_trace_path)
        except PermissionError as exc:
            raise RuntimeError(
                f"无法在 {experiment_dir.parent} 下创建实验目录，当前用户没有写权限。\n"
                f"请先调整 NAS 目录权限，例如：sudo chown -R $USER:$USER {cfg.log_root}\n"
                f"或：sudo chmod -R u+w {cfg.log_root}\n"
                f"原始错误：{exc}"
            ) from exc
    return experiment_dir


def build_ns3_args(cfg: RunnerConfig) -> List[str]:
    mapping = {
        "nFlows": cfg.n_flows,
        "simTime": cfg.sim_time,
        "algos": cfg.algos,
        "startTimes": cfg.start_times,
        "flowStopTimes": cfg.stop_times,
        "initialRates": cfg.initial_rates,
        "rateSchedules": cfg.rate_schedules,
        "freqccv4CruiseBaseCaps": cfg.freqccv4_cruise_base_caps,
        "accessRate": cfg.access_rate,
        "serviceRate": cfg.service_rate,
        "accessDelayMs": cfg.access_delay_ms,
        "serviceDelayMs": cfg.service_delay_ms,
        "propDelayMs": cfg.prop_delay_ms,
        "switchBufferBytes": cfg.switch_buffer_bytes,
        "switchBufferBdp": cfg.switch_buffer_bdp,
        "endpointQueueBytes": cfg.endpoint_queue_bytes,
        "flowSizeBytes": cfg.flow_size_bytes,
        "processIntervalUs": cfg.process_interval_us,
        "goodputIntervalMs": cfg.goodput_interval_ms,
        "useEngineTimer": bool_to_ns3(cfg.use_engine_timer),
        "enableTrace": bool_to_ns3(cfg.enable_trace),
        "enableHeavyTrace": bool_to_ns3(cfg.enable_heavy_trace),
        "enableQueueTrace": bool_to_ns3(cfg.enable_queue_trace),
        "emitBottleneckQueueTrace": bool_to_ns3(cfg.emit_bottleneck_queue_trace),
        "queueSampleIntervalUs": cfg.queue_sample_interval_us,
        "emulatedConnections": cfg.emulated_connections,
        "dataGeneratorBatch": cfg.data_generator_batch,
        "streamBufferBytes": cfg.stream_buffer_bytes,
        "enableConvergenceGateTrace": bool_to_ns3(cfg.enable_convergence_gate_trace),
        "enableConvergenceGateControl": bool_to_ns3(cfg.enable_convergence_gate_control),
        "gateTraceMode": cfg.gate_trace_mode,
        "gateTraceSampleIntervalUs": cfg.gate_trace_sample_interval_us,
        "intervalWinRttMult": cfg.interval_win_rtt_mult,
        "configPaths": cfg.config_paths,
        "freqccv4Config": cfg.freqccv4_config,
        "obbrConfig": cfg.obbr_config,
        "bbrv2plusConfig": cfg.bbrv2plus_config,
        "bbrv2Config": cfg.bbrv2_config,
        "tracePath": cfg.trace_path,
        "traceName": cfg.trace_name,
        "seed": cfg.seed,
        "runId": cfg.run_id,
    }
    args = []
    for key, value in mapping.items():
        if value is None:
            continue
        if isinstance(value, str) and value == "":
            continue
        args.append(f"--{key}={value}")
    return args


def print_summary(cfg: RunnerConfig, ns3_args: Iterable[str]) -> None:
    print("\n即将运行的并发流场景配置：")
    print("=" * 88)
    print(f"ns-3 根目录：{NS3_ROOT}")
    print(f"场景脚本：scratch/{SCENARIO}.cc")
    print(f"流数量：{cfg.n_flows}")
    print(f"算法：{cfg.algos}")
    print(f"注入时刻：{cfg.start_times} s")
    print(f"停止时刻：{cfg.stop_times or cfg.sim_time} s")
    print(f"发送速率上限 initialRates：{cfg.initial_rates}")
    if cfg.rate_schedules:
        print(f"分阶段发送速率上限 rateSchedules：{cfg.rate_schedules}")
    if cfg.freqccv4_cruise_base_caps != "0":
        print(f"FreqCCv4 CRUISE 基线上限：{cfg.freqccv4_cruise_base_caps}")
    print(f"sender/receiver access 链路：{cfg.access_rate}, {cfg.access_delay_ms} ms")
    print(f"共享 bottleneck 链路：{cfg.service_rate}, {cfg.service_delay_ms} ms")
    print(f"共享 bottleneck 出口 buffer：{cfg.switch_buffer_bytes} bytes；0 表示按 {cfg.switch_buffer_bdp} BDP 自动计算")
    print(f"端侧/access 队列：{cfg.endpoint_queue_bytes} bytes")
    print(f"仿真时长：{cfg.sim_time} s")
    print(f"heavy trace：{cfg.enable_heavy_trace}")
    print(
        "队列 trace："
        f"逐事件={cfg.enable_queue_trace}, 固定采样={cfg.emit_bottleneck_queue_trace}, "
        f"间隔={cfg.queue_sample_interval_us} us"
    )
    print(f"FreqCCv4 gate trace：{cfg.enable_convergence_gate_trace}, mode={cfg.gate_trace_mode}")
    print(f"log 根目录：{cfg.log_root}")
    print(f"本次实验目录：{cfg.trace_path}")
    print("本次实验目录内会保存：run.log、command.txt、config.json、ns-3 trace 文件")
    print(f"freqccv4 config：{cfg.freqccv4_config}")
    print("-" * 88)
    if cfg.direct_binary:
        binary = NS3_ROOT / "build" / "scratch" / SCENARIO
        print("完整二进制命令（批量脚本已预先编译）：")
        print(
            f"LD_LIBRARY_PATH={shlex.quote(str(NS3_ROOT / 'build'))}:$LD_LIBRARY_PATH "
            + " ".join(shlex.quote(part) for part in [str(binary), *ns3_args])
        )
    else:
        run_part = " ".join([SCENARIO, *ns3_args])
        print("完整 waf 命令：")
        print(f"cd {shlex.quote(str(NS3_ROOT))}")
        print(f"./waf --run {shlex.quote(run_part)}")
    print("=" * 88)
    sys.stdout.flush()


def run_command(cfg: RunnerConfig, dry_run: bool) -> int:
    try:
        experiment_dir = prepare_experiment_dir(cfg, create=not dry_run)
    except RuntimeError as exc:
        print(f"配置错误：{exc}", file=sys.stderr)
        return 2
    ns3_args = build_ns3_args(cfg)
    print_summary(cfg, ns3_args)
    if dry_run:
        print("\n--dry-run 已启用：只打印命令，不执行。")
        return 0

    if cfg.direct_binary:
        binary = NS3_ROOT / "build" / "scratch" / SCENARIO
        if not binary.exists():
            print(f"配置错误：直接运行的二进制不存在：{binary}", file=sys.stderr)
            return 2
        cmd = [str(binary), *ns3_args]
        command_env = os.environ.copy()
        old_library_path = command_env.get("LD_LIBRARY_PATH", "")
        command_env["LD_LIBRARY_PATH"] = str(NS3_ROOT / "build") + (
            f":{old_library_path}" if old_library_path else ""
        )
        command_text = (
            f"LD_LIBRARY_PATH={shlex.quote(command_env['LD_LIBRARY_PATH'])} "
            + " ".join(shlex.quote(part) for part in cmd)
            + "\n"
        )
    else:
        run_part = " ".join([SCENARIO, *ns3_args])
        cmd = ["./waf", "--run", run_part]
        command_text = (
            f"cd {shlex.quote(str(NS3_ROOT))}\n"
            f"./waf --run {shlex.quote(run_part)}\n"
        )
        command_env = None
    (experiment_dir / "command.txt").write_text(command_text, encoding="utf-8")
    (experiment_dir / "config.json").write_text(
        json.dumps(asdict(cfg), ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )

    log_file = experiment_dir / "run.log"
    print(f"\n运行输出会同时写入：{log_file}")
    with log_file.open("w", encoding="utf-8") as log:
        log.write("=== command ===\n")
        log.write(command_text)
        log.write("\n=== output ===\n")
        log.flush()
        proc = subprocess.Popen(
            cmd,
            cwd=str(NS3_ROOT),
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            bufsize=1,
            env=command_env,
        )
        assert proc.stdout is not None
        for line in proc.stdout:
            print(line, end="")
            log.write(line)
        return_code = proc.wait()
        log.write(f"\n=== return_code: {return_code} ===\n")
    (experiment_dir / "return_code.txt").write_text(f"{return_code}\n", encoding="utf-8")
    return return_code

PAPER_REFERENCES = [
    {
        "name": "BBR: Congestion-Based Congestion Control",
        "url": "https://queue.acm.org/detail.cfm?id=3022184",
        "note": "BBR uses bottleneck bandwidth, RTprop and BDP; queueing begins when inflight exceeds BDP.",
    },
    {
        "name": "An Emulation-based Evaluation of TCP BBRv2 Alpha for Wired Broadband",
        "url": "https://research.cec.sc.edu/files/cyberinfra/files/bbrv2_final_comcom.pdf",
        "note": "Uses Mininet, buffer sizes in BDP units, variable RTTs and flow counts, and reports throughput/fairness/utilization.",
    },
    {
        "name": "BBRv2+: Towards balancing aggressiveness and fairness with delay-based bandwidth probing",
        "url": "https://arxiv.org/pdf/2107.03057",
        "note": "Uses Mininet/Mahimahi, records throughput and bottleneck queue sojourn time, and exposes BBRv2+ parameters.",
    },
    {
        "name": "oBBR: Optimize Retransmissions of BBR Flows on the Internet",
        "url": "https://www.usenix.org/system/files/atc23-bi.pdf",
        "note": "Uses shallow-buffer settings such as R=0.5 BDP, 100Mbps and 40ms RTT, and compares oBBR-0.5/0.75/1.",
    },
]

ALGORITHM_PARAMS = {
    "BBRv2": {
        "source": "src/dqc/model/thirdparty/congestion/quic_bbr2_sender.cc",
        "config_file": "",
        "notes": "原始 DQC BBRv2 实现；本实验不额外传入配置文件。",
    },
    "oBBR": {
        "source": "src/dqc/model/thirdparty/congestion/obbr_sender.cc",
        "paper_variant": "oBBR-0.5",
        "config_file": "",
        "parameters": {
            "u_mu": 0.5,
            "startup_high_gain": 2.885,
            "drain_gain": "1/2.885",
            "probe_bw_pacing_gain_cycle": [1.25, 0.75, 1, 1, 1, 1, 1, 1],
            "probe_bw_cwnd_gain": 2.0,
            "max_cwnd_gain": 2.05,
            "probe_up_min_gain": 1.25,
            "bandwidth_drop_threshold": 0.75,
            "queueing_rtt_threshold": 2.5,
            "signal_window": 30,
            "loss_window_ms": 3000,
            "score_window_ms": 200,
        },
        "notes": "本地代码常量 kObbrU=0.5，对应 oBBR 论文中重传更稳的 oBBR-0.5 方案。",
    },
    "BBRv2plus": {
        "source": "src/dqc/model/thirdparty/congestion/quic_bbr2plus_sender.cc",
        "config_file": "",
        "parameters": {
            "rtt_comp_startup_gain": 2.885,
            "rtt_comp_gain": 2.0,
            "rtt_comp_rttvar_threshold": 0.4,
            "rc_min_rtt_window_rounds": 4,
            "jitter_window_rounds": 4,
            "fast_convergence_rtt_threshold": 1.1,
            "fast_convergence_preup_threshold": 0.02,
            "fast_convergence_rtt_error_us": 2000,
            "fast_convergence_probe_again_threshold": 0.02,
            "fast_convergence_probe_cycle_base_rounds": 8,
            "fast_convergence_probe_cycle_random_rounds": 4,
            "rounds_to_advance_bw_filter": 25,
            "max_probe_again_per_cycle": 1,
            "pre_up_pacing_gain": 1.10,
            "down_slightly_pacing_gain": 0.90,
        },
        "notes": "采用本地 BBRv2plus 代码内置参数，贴近论文的 delay-guided probing 与 jitter BDP compensation。",
    },
    "F-BBR": {
        "source": "src/dqc/model/thirdparty/congestion/freqccv4_sender.cc",
        "config_file": str(DEFAULT_FREQCCV4_CONFIG),
        "algorithm": "F-BBR",
        "operating_point_identifier": "OPIv2",
        "notes": "F-BBR 的兼容源文件名仍为 freqccv4_sender.cc；默认使用 coded-sine OPIv2。",
    },
    "FreqCCv4": {
        "source": "src/dqc/model/thirdparty/congestion/freqccv4_sender.cc",
        "config_file": str(DEFAULT_FREQCCV4_CONFIG),
        "algorithm": "FreqCCv4",
        "notes": "FreqCCv4 原版使用固定 Δ 上升/下降基线。",
    },
    "FreqCCv4-adaptive": {
        "source": "src/dqc/model/thirdparty/congestion/freqccv4_sender.cc",
        "config_file": str(DEFAULT_FREQCCV4_CONFIG),
        "algorithm": "FreqCCv4-adaptive",
        "notes": "Adaptive Guard 分支使用平滑 Δ、窗口边界、过载确认和队列守卫。",
    },
    "FreqCCv4-hybrid": {
        "source": "src/dqc/model/thirdparty/congestion/freqccv4_sender.cc",
        "config_file": str(DEFAULT_FREQCCV4_CONFIG),
        "algorithm": "FreqCCv4-hybrid",
        "notes": "Hybrid 分支使用窗口极值调整基线，并平滑 FULL_LOAD 窗口均值。",
    },
}


def split_csv(text: str) -> List[str]:
    return [part.strip() for part in str(text).split(",") if part.strip()]


def default_experiment_name(seed: int, run_id: int) -> str:
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    return f"four_cc_4flows_bbr_compare_seed{seed}_run{run_id}_{timestamp}"


def prepare_root(args: argparse.Namespace, create: bool) -> Path:
    if args.experiment_dir:
        root = Path(args.experiment_dir).expanduser().resolve()
    else:
        name = sanitize_path_part(args.experiment_name) if args.experiment_name else default_experiment_name(args.seed, args.run_id)
        root = unique_dir(Path(args.log_root).expanduser() / name).resolve()
    if create:
        try:
            root.mkdir(parents=True, exist_ok=bool(args.experiment_dir))
        except PermissionError as exc:
            raise RuntimeError(
                f"无法创建复合实验目录：{root}\n"
                f"请先调整 NAS 目录权限，例如：sudo chown -R $USER:$USER {args.log_root}\n"
                f"原始错误：{exc}"
            ) from exc
    return root


def build_sub_config(args: argparse.Namespace, root: Path, cc: str) -> RunnerConfig:
    cfg = RunnerConfig()
    cfg.n_flows = args.n_flows
    cfg.sim_time = args.sim_time
    cfg.algos = cc
    cfg.start_times = args.start_times
    cfg.stop_times = args.stop_times
    cfg.initial_rates = args.initial_rates
    cfg.rate_schedules = args.rate_schedules
    cfg.freqccv4_cruise_base_caps = args.freqccv4_cruise_base_caps
    cfg.access_rate = args.access_rate
    cfg.service_rate = args.service_rate
    cfg.access_delay_ms = args.access_delay_ms
    cfg.service_delay_ms = args.service_delay_ms
    cfg.prop_delay_ms = -1.0
    cfg.switch_buffer_bytes = args.switch_buffer_bytes
    cfg.switch_buffer_bdp = args.switch_buffer_bdp
    cfg.endpoint_queue_bytes = args.endpoint_queue_bytes
    cfg.flow_size_bytes = args.flow_size_bytes
    cfg.process_interval_us = args.process_interval_us
    cfg.goodput_interval_ms = args.goodput_interval_ms
    cfg.use_engine_timer = args.use_engine_timer
    cfg.enable_trace = True
    cfg.enable_heavy_trace = args.enable_heavy_trace
    cfg.enable_queue_trace = args.enable_queue_trace
    cfg.emit_bottleneck_queue_trace = True
    cfg.queue_sample_interval_us = args.queue_sample_interval_us
    cfg.enable_convergence_gate_trace = args.enable_convergence_gate_trace
    cfg.gate_trace_mode = args.gate_trace_mode
    cfg.data_generator_batch = args.data_generator_batch
    cfg.stream_buffer_bytes = args.stream_buffer_bytes
    cfg.freqccv4_config = args.freqccv4_config
    cfg.log_root = str(root)
    cfg.trace_path = str(root / cc)
    cfg.trace_name = cc
    cfg.seed = args.seed
    cfg.run_id = args.run_id
    cfg.direct_binary = args.direct_binary
    return cfg


def scenario_metadata(args: argparse.Namespace, ccs: Iterable[str]) -> Dict[str, object]:
    return {
        "topology": "n sender/receiver pairs in a dumbbell topology; every flow shares one left-switch -> right-switch bottleneck link.",
        "ccs": list(ccs),
        "n_flows_per_subexperiment": args.n_flows,
        "sim_time_s": args.sim_time,
        "start_times_s": args.start_times,
        "stop_times_s": args.stop_times or str(args.sim_time),
        "initial_rates": args.initial_rates,
        "rate_schedules": args.rate_schedules,
        "freqccv4_cruise_base_caps": args.freqccv4_cruise_base_caps,
        "data_generator_batch": args.data_generator_batch,
        "stream_buffer_bytes": args.stream_buffer_bytes,
        "enable_convergence_gate_trace": args.enable_convergence_gate_trace,
        "gate_trace_mode": args.gate_trace_mode,
        "enable_queue_trace": args.enable_queue_trace,
        "queue_sample_interval_us": args.queue_sample_interval_us,
        "seed": args.seed,
        "run_id": args.run_id,
        "access_rate": args.access_rate,
        "shared_bottleneck_rate": args.service_rate,
        "rtt_design": f"RTT ~= 2 * (2 * {args.access_delay_ms}ms + {args.service_delay_ms}ms)",
        "switch_buffer": {
            "bytes": args.switch_buffer_bytes,
            "bdp_multiplier_when_bytes_is_zero": args.switch_buffer_bdp,
            "note": f"Switch buffer is configured as {args.switch_buffer_bdp} BDP; endpoint queues remain large.",
        },
        "metrics": [
            "aggregate throughput over time from *_good.txt",
            "mean per-flow throughput over time from *_good.txt",
            "shared bottleneck queueing delay over time from queue bytes / bottleneck service rate",
            "P95 queueing delay after warmup",
        ],
        "notes": [
            "当前 n 对 n 拓扑是 dumbbell；所有 flow 共享同一条 left-switch -> right-switch 瓶颈链路。",
            "service-rate 表示共享瓶颈总容量；access 链路保持高速以避免成为每流独立瓶颈。",
        ],
        "paper_references": PAPER_REFERENCES,
    }


def write_metadata(root: Path, args: argparse.Namespace, ccs: Iterable[str]) -> None:
    selected = list(ccs)
    params = {
        cc: dict(ALGORITHM_PARAMS.get(cc, {"algorithm": cc}))
        for cc in selected
    }
    for cc in (
        "F-BBR",
        "FreqCCv4",
        "FreqCCv4-adaptive",
        "FreqCCv4-hybrid",
    ):
        if cc in params:
            params[cc]["config_file"] = args.freqccv4_config

    (root / "comparison_config.json").write_text(
        json.dumps(scenario_metadata(args, selected), ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    (root / "algorithm_params.json").write_text(
        json.dumps(params, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    lines = [
        "# 4CC 对比实验方案",
        "",
        "## 目录结构",
        "",
        *[f"- `{cc}/`：{cc} 四流子实验" for cc in selected],
        "- `compare/`：汇总 CSV 和对比图",
        "",
        "## 默认场景",
        "",
        f"- 流数量：{args.n_flows}",
        f"- 启动时刻：{args.start_times} s",
        f"- 停止时刻：{args.stop_times or args.sim_time} s",
        f"- 共享 bottleneck 链路：`left switch -> right switch` 为 {args.service_rate}",
        f"- access 链路：`sender[i] -> left switch` 和 `right switch -> receiver[i]` 为 {args.access_rate}",
        f"- RTT 设计：约 `2 * (2 * {args.access_delay_ms}ms + {args.service_delay_ms}ms)`",
        f"- 共享 bottleneck 出口 buffer：{args.switch_buffer_bdp} BDP（若 `switch_buffer_bytes=0`）",
        f"- 发送侧填充批量：每次可写填充 {args.data_generator_batch} 个 1500B 包",
        f"- 仿真时长：{args.sim_time}s",
        "",
        "## 参考依据",
        "",
    ]
    for ref in PAPER_REFERENCES:
        lines.append(f"- {ref['name']}: {ref['url']}")
    (root / "README_4cc_comparison.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


def run_plot(root: Path, args: argparse.Namespace, ccs: Iterable[str]) -> int:
    compare_dir = root / "compare"
    compare_dir.mkdir(parents=True, exist_ok=True)
    plot_env = os.environ.copy()
    plot_env["MPLCONFIGDIR"] = str(compare_dir / ".matplotlib")
    plot_env["XDG_CACHE_HOME"] = str(compare_dir / ".cache")
    selected = list(ccs)
    cmd = [
        sys.executable,
        str(PLOT_SCRIPT),
        "--experiment-dir",
        str(root),
        "--service-rate",
        args.service_rate,
        "--n-flows",
        str(args.n_flows),
        "--ccs",
        ",".join(selected),
        "--trace-names",
        ",".join(selected),
        "--sample-step-s",
        str(args.sample_step_s),
        "--warmup-s",
        str(args.warmup_s),
        "--flow-start-times",
        args.start_times,
        "--flow-stop-times",
        args.stop_times or str(args.sim_time),
        "--sim-time",
        str(args.sim_time),
    ]
    (compare_dir / "plot_command.txt").write_text(
        " ".join(shlex.quote(part) for part in cmd) + "\n", encoding="utf-8"
    )
    with (compare_dir / "plot.log").open("w", encoding="utf-8") as log:
        proc = subprocess.Popen(
            cmd,
            cwd=str(NS3_ROOT),
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            env=plot_env,
        )
        assert proc.stdout is not None
        for line in proc.stdout:
            print(line, end="")
            log.write(line)
        return proc.wait()


def selected_ccs(args: argparse.Namespace) -> List[str]:
    if not args.only_cc:
        return list(CCS)
    requested = split_csv(args.only_cc)
    invalid = [cc for cc in requested if cc not in SELECTABLE_CCS]
    if invalid:
        raise ValueError(
            f"--only-cc 只支持 {', '.join(SELECTABLE_CCS)}，收到：{', '.join(invalid)}"
        )
    return requested


def load_return_codes(root: Path) -> Dict[str, int]:
    results: Dict[str, int] = {}
    existing = root / "return_codes.json"
    if existing.exists():
        try:
            raw = json.loads(existing.read_text(encoding="utf-8"))
            for key, value in raw.items():
                if isinstance(value, int):
                    results[key] = value
        except Exception:
            pass
    for cc in SELECTABLE_CCS:
        rc_file = root / cc / "return_code.txt"
        if not rc_file.exists():
            continue
        try:
            results[cc] = int(rc_file.read_text(encoding="utf-8").strip())
        except ValueError:
            pass
    return results


def make_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="运行 4 条流、4 个 CC 的复合对比实验，并自动画吞吐/排队延迟对比图。",
        formatter_class=argparse.RawTextHelpFormatter,
        epilog=(
            "默认方案：\n"
            "  - 四个子实验：BBRv2、oBBR、BBRv2plus、F-BBR\n"
            "  - 每个子实验 4 条同算法长流，n 个发送端对 n 个接收端，构成 dumbbell\n"
            "  - 所有流共享一条 left-switch -> right-switch 瓶颈链路，默认 100Mbps\n"
            "  - 共享瓶颈出口 buffer 为 0.5BDP，端侧/access 队列为 1GiB\n"
            "  - 输出根目录默认在 /mnt/nasDisk_ds3617/wkd/1FreqBBR/ 下新建一个文件夹\n\n"
            "常用命令：\n"
            "  python3 run_4cc_comparison.py\n"
            "  python3 run_4cc_comparison.py --dry-run\n"
            "  python3 run_4cc_comparison.py --experiment-dir /path/to/old_exp --skip-run\n"
        ),
    )
    parser.add_argument("--log-root", default=str(DEFAULT_LOG_ROOT), help="复合实验根目录的父目录。")
    parser.add_argument("--experiment-name", default="", help="复合实验目录名；默认按时间自动生成。")
    parser.add_argument("--experiment-dir", default="", help="指定完整复合实验目录。配合 --skip-run 可只重画图。")
    parser.add_argument("--dry-run", action="store_true", help="只打印四个子实验命令，不运行、不画图。")
    parser.add_argument("--skip-run", action="store_true", help="跳过 ns-3 子实验，只根据已有 trace 画图。")
    parser.add_argument("--skip-completed", action="store_true", help="已有 return_code=0 的 CC 子实验不再重复运行。")
    parser.add_argument("--no-plot", action="store_true", help="只运行四个子实验，不执行画图脚本。")
    parser.add_argument("--continue-on-error", action="store_true", help="某个 CC 子实验失败后继续尝试后续子实验。")
    parser.add_argument("--direct-binary", action="store_true", help="直接运行已编译的 build/scratch 二进制；批量并行实验使用。")
    parser.add_argument(
        "--only-cc",
        default="",
        help=(
            "只运行指定 CC，可用逗号分隔。额外支持 FreqCCv4、"
            "FreqCCv4-adaptive 和 FreqCCv4-hybrid。"
        ),
    )

    parser.add_argument("--n-flows", type=int, default=4, help="每个子实验流数量。该复合方案默认 4。")
    parser.add_argument("--sim-time", type=float, default=60.0, help="仿真时长秒。默认 60。论文常用更长，调试可降低。")
    parser.add_argument("--start-times", default="0,0,0,0", help="四条流注入时刻。默认同步启动，贴近多数多流实验。")
    parser.add_argument("--stop-times", default="", help="每条流停止时刻；给 1 个值或每流 1 个值。默认都在仿真结束时停止。")
    parser.add_argument("--initial-rates", default="0", help="发送端速率上限。0 表示不限制，让 CC 自行决定 pacing。")
    parser.add_argument(
        "--rate-schedules",
        default="",
        help=(
            "每流运行时 pacing 上限，time:rate 用逗号分隔，流之间用 @ 分隔；"
            "例如 0:20Mbps,6:55Mbps@0:20Mbps,6:55Mbps。"
        ),
    )
    parser.add_argument(
        "--freqccv4-cruise-base-caps",
        default="0",
        help=(
            "每流 FreqCCv4 CRUISE 原生 pacing 基线上限；0 关闭。"
            "该上限在加扰动前应用，用于给完整波形留出余量。"
        ),
    )
    parser.add_argument("--access-rate", default="1Gbps", help="sender/receiver access 链路速率。默认 1Gbps。")
    parser.add_argument("--service-rate", default="100Mbps", help="共享 bottleneck 链路总速率。默认 100Mbps。")
    parser.add_argument("--access-delay-ms", type=float, default=1.0, help="sender/receiver access 单向时延。默认 1ms。")
    parser.add_argument("--service-delay-ms", type=float, default=19.0, help="共享 bottleneck 单向时延。默认 19ms。")
    parser.add_argument("--switch-buffer-bytes", type=int, default=0, help="共享 bottleneck 出口 buffer 字节；0 表示按 BDP 自动计算。")
    parser.add_argument("--switch-buffer-bdp", type=float, default=0.5, help="自动 buffer 的 BDP 倍数。默认 0.5BDP。")
    parser.add_argument("--endpoint-queue-bytes", type=int, default=1073741824, help="端侧/access 队列，默认 1GiB。")
    parser.add_argument("--flow-size-bytes", type=int, default=0, help="每流发送字节上限；0 表示长流。")
    parser.add_argument("--process-interval-us", type=int, default=100, help="DQC 轮询间隔 us。默认 100。")
    parser.add_argument("--goodput-interval-ms", type=int, default=100, help="goodput 统计粒度 ms。默认 100。")
    parser.add_argument("--data-generator-batch", type=int, default=2, help="发送侧每次可写时填充的 1500B 包数。默认 2；饱和负载建议 64。")
    parser.add_argument("--stream-buffer-bytes", type=int, default=0, help="DQC stream send buffer；0 保持核心默认值。")
    parser.add_argument("--use-engine-timer", action=argparse.BooleanOptionalAction, default=True, help="是否使用 DQC engine timer。默认启用。")
    parser.add_argument("--enable-heavy-trace", action=argparse.BooleanOptionalAction, default=False, help="是否开启 RTT/BW/sendrate 等重 trace。默认关闭。")
    parser.add_argument("--enable-queue-trace", action=argparse.BooleanOptionalAction, default=True, help="是否输出逐事件共享队列 trace；长实验可关闭并保留固定间隔 bottleneck_queue.csv。")
    parser.add_argument("--queue-sample-interval-us", type=int, default=200, help="bottleneck_queue.csv 固定采样间隔 us。默认 200。")
    parser.add_argument("--enable-convergence-gate-trace", action=argparse.BooleanOptionalAction, default=False, help="是否输出 FreqCCv4 gate trace；绘制 Delivery Rate/TrustedBw 细节图时应开启。")
    parser.add_argument("--gate-trace-mode", choices=("off", "round_only", "sampled_pacing", "full"), default="round_only", help="FreqCCv4 gate trace 粒度。细节图建议 sampled_pacing。")
    parser.add_argument("--freqccv4-config", default=str(DEFAULT_FREQCCV4_CONFIG), help="freqccv4 配置文件路径；默认保持现有 freqccv4_default.conf。")
    parser.add_argument("--seed", type=int, default=1, help="ns-3 RNG seed。默认 1。")
    parser.add_argument("--run-id", type=int, default=1, help="ns-3 RNG run id。默认 1。")
    parser.add_argument("--sample-step-s", type=float, default=0.1, help="画图重采样时间步长。默认 0.1s。")
    parser.add_argument("--warmup-s", type=float, default=5.0, help="汇总平均值跳过的启动阶段。默认 5s。")
    return parser


def main(argv: Optional[List[str]] = None) -> int:
    parser = make_parser()
    args = parser.parse_args(argv)

    if args.n_flows != 4:
        print("提示：这是 4 条流对比方案；你覆盖了 --n-flows，脚本会照常运行。")
    if len(split_csv(args.start_times)) not in {1, args.n_flows}:
        print(f"配置错误：--start-times 必须给 1 个值或 {args.n_flows} 个值。", file=sys.stderr)
        return 2
    if args.stop_times and len(split_csv(args.stop_times)) not in {1, args.n_flows}:
        print(f"配置错误：--stop-times 必须给 1 个值或 {args.n_flows} 个值。", file=sys.stderr)
        return 2
    try:
        ccs_to_run = selected_ccs(args)
    except ValueError as exc:
        print(f"配置错误：{exc}", file=sys.stderr)
        return 2

    try:
        root = prepare_root(args, create=not args.dry_run)
    except RuntimeError as exc:
        print(f"配置错误：{exc}", file=sys.stderr)
        return 2

    print("\n4CC 复合实验目录：", root)
    print("子目录：" + " / ".join([*ccs_to_run, "compare"]))
    print("默认指标：aggregate throughput、mean per-flow throughput、mean/P95 queueing delay")
    if args.only_cc:
        print("本次只运行：", ", ".join(ccs_to_run))

    if not args.dry_run:
        write_metadata(root, args, ccs_to_run)

    results: Dict[str, int] = {} if args.dry_run else load_return_codes(root)
    if not args.skip_run:
        for cc in ccs_to_run:
            if args.skip_completed and results.get(cc) == 0:
                print(f"跳过已完成子实验：{cc}")
                continue
            print("\n" + "=" * 88)
            print(f"开始子实验：{cc}")
            cfg = build_sub_config(args, root, cc)
            if not args.dry_run:
                (root / f"{cc}_runner_config.json").write_text(
                    json.dumps(asdict(cfg), ensure_ascii=False, indent=2) + "\n",
                    encoding="utf-8",
                )
            rc = run_command(cfg, args.dry_run)
            results[cc] = rc
            if rc != 0 and not args.continue_on_error:
                print(f"子实验 {cc} 失败，返回码 {rc}。")
                return rc

    if args.dry_run:
        print("\n--dry-run 已启用：未创建实验目录、未运行 ns-3、未画图。")
        return 0

    if not args.no_plot:
        print("\n" + "=" * 88)
        print("开始生成 compare 对比图")
        plot_rc = run_plot(root, args, ccs_to_run)
        results["plot"] = plot_rc
        if plot_rc != 0:
            return plot_rc

    (root / "return_codes.json").write_text(
        json.dumps(results, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    print("\n复合实验完成：", root)
    if not args.no_plot:
        print("对比图目录：", root / "compare")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
