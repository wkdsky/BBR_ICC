# ns-3 迁移展示：BBRv2、BBRv2plus、oBBR 与 FreqCCv3

## 1. 项目概述

本工作将多个具有代表性的拥塞控制算法迁移到 `ns-3.27` 的 `DQC` 主机框架中：

- `BBRv2`
- `BBRv2plus`
- `oBBR`
- `FreqCCv3`

迁移后的目标不是把 Linux 内核代码逐行照搬到仿真器里，而是在 `ns-3` 现有的 `DQC` 主机实现上，按各算法原始宿主分别保留语义、控制逻辑和可复现实验路径，使其能够：

- 在统一的仿真环境中与 `BBRv2`、`CUBIC`、`oBBR` 等算法公平对比
- 复用 `ns-3` 的拓扑、链路参数、批量实验和 trace 能力
- 降低复现实验门槛，避免依赖内核补丁、真实 QUIC 服务端和复杂外部网络环境

## 2. 这项工作的意义

### 2.1 研究意义

- `BBRv2plus` 和 `oBBR` 都属于对 BBR 系列算法行为进行增强的方案，但它们原本的宿主环境并不一致：`BBRv2plus` 对应 `BBRv2` 宿主，`oBBR` 对应 `BBRv1` 宿主。
- 将它们统一迁移到 `ns-3/DQC` 后，可以在同一 host model、同一链路模型、同一 trace 口径下做 apples-to-apples 对比。
- 这使得论文复现、教学展示、参数敏感性分析和后续算法迭代都更容易。

### 2.2 工程意义

- 原始 `oBBR` 工程主要面向 `nginx-quic` 实验环境，核心逻辑挂在 `ngx_bbr.c` 的 `BBRv1` 路径上，运行门槛较高。
- `BBRv2plus` 在本仓库中的迁移实现则基于 `DQC BBRv2` 做 host-native 集成，避免了直接维护一套“内核式”宿主。
- 迁移后，`BBRv2`、`BBRv2plus`、`oBBR` 和 `FreqCCv3` 四个并列算法都可以直接通过 `./waf --run` 进入统一实验链路。

## 3. 工作量说明

下面的统计以当前仓库中的迁移实现文件和主要迁移提交为依据。

### 3.1 当前仓库中可直接归属的核心代码规模

- `BBRv2plus` 核心 sender 文件：
  [`src/dqc/model/thirdparty/congestion/quic_bbr2plus_sender.h`](/home/wkd/BBR_ICC/NS3.27/src/dqc/model/thirdparty/congestion/quic_bbr2plus_sender.h) + [`src/dqc/model/thirdparty/congestion/quic_bbr2plus_sender.cc`](/home/wkd/BBR_ICC/NS3.27/src/dqc/model/thirdparty/congestion/quic_bbr2plus_sender.cc)
  共约 `483` 行
- `oBBR` 核心 sender 文件：
  [`src/dqc/model/thirdparty/congestion/obbr_sender.h`](/home/wkd/BBR_ICC/NS3.27/src/dqc/model/thirdparty/congestion/obbr_sender.h) + [`src/dqc/model/thirdparty/congestion/obbr_sender.cc`](/home/wkd/BBR_ICC/NS3.27/src/dqc/model/thirdparty/congestion/obbr_sender.cc)
  共约 `1515` 行
- 对应 ns-3 实验驱动场景：
  [`scratch/bbrv2_4flow.cc`](/home/wkd/BBR_ICC/NS3.27/scratch/bbrv2_4flow.cc) + [`scratch/obbr_4flow.cc`](/home/wkd/BBR_ICC/NS3.27/scratch/obbr_4flow.cc)
  共约 `791` 行

仅这几类最直接的迁移与展示文件合计约 `2789` 行。

### 3.2 按主要迁移提交统计的代码改动规模

主要迁移提交包括：

- `90c3b70`：将 `oBBR` 和 `BBRv2plus` 主体迁移进入 `NS3.27`
- `9604145`：补充并收敛 `BBRv2plus` 的 DQC-hosted 迁移实现与说明文档

按 `git show --stat` 统计：

- 累计新增约 `3370` 行
- 累计调整/删除约 `510` 行

这说明该工作并不是“加一个枚举值”级别的接线，而是涉及：

- 新算法 sender 的完整实现
- `DQC` 宿主接入
- trace/export 接口补齐
- 仿真入口与实验场景构建
- 批量运行与文档说明

## 4. 主要代码位置

### 4.1 BBRv2plus

- [`src/dqc/model/thirdparty/congestion/quic_bbr2plus_sender.h`](/home/wkd/BBR_ICC/NS3.27/src/dqc/model/thirdparty/congestion/quic_bbr2plus_sender.h)
- [`src/dqc/model/thirdparty/congestion/quic_bbr2plus_sender.cc`](/home/wkd/BBR_ICC/NS3.27/src/dqc/model/thirdparty/congestion/quic_bbr2plus_sender.cc)
- [`src/dqc/model/thirdparty/congestion/quic_bbr2_probe_bw.cc`](/home/wkd/BBR_ICC/NS3.27/src/dqc/model/thirdparty/congestion/quic_bbr2_probe_bw.cc)
- [`src/dqc/model/dqc_sender.cc`](/home/wkd/BBR_ICC/NS3.27/src/dqc/model/dqc_sender.cc)
- [`scratch/bbrv2_4flow.cc`](/home/wkd/BBR_ICC/NS3.27/scratch/bbrv2_4flow.cc)
- [`run_bbrv2_batch.sh`](/home/wkd/BBR_ICC/NS3.27/run_bbrv2_batch.sh)

### 4.2 oBBR

- [`src/dqc/model/thirdparty/congestion/obbr_sender.h`](/home/wkd/BBR_ICC/NS3.27/src/dqc/model/thirdparty/congestion/obbr_sender.h)
- [`src/dqc/model/thirdparty/congestion/obbr_sender.cc`](/home/wkd/BBR_ICC/NS3.27/src/dqc/model/thirdparty/congestion/obbr_sender.cc)
- [`src/dqc/model/thirdparty/congestion/proto_send_algorithm_interface.cc`](/home/wkd/BBR_ICC/NS3.27/src/dqc/model/thirdparty/congestion/proto_send_algorithm_interface.cc)
- [`src/dqc/model/dqc_sender.cc`](/home/wkd/BBR_ICC/NS3.27/src/dqc/model/dqc_sender.cc)
- [`scratch/obbr_4flow.cc`](/home/wkd/BBR_ICC/NS3.27/scratch/obbr_4flow.cc)

## 5. 如何执行

### 5.1 编译

```bash
cd NS3.27
./waf build
```

### 5.2 统一脚本入口

新增统一脚本：

- [`scripts/showcase/run_showcase_scenario.sh`](/home/wkd/BBR_ICC/NS3.27/scripts/showcase/run_showcase_scenario.sh)
- [`scripts/showcase/plot_trace_bundle.py`](/home/wkd/BBR_ICC/NS3.27/scripts/showcase/plot_trace_bundle.py)

其中：

- `run_showcase_scenario.sh` 用于执行一个 `ns-3` 场景，并按统一目录落 trace/fig
- `plot_trace_bundle.py` 用于把一个 trace 目录渲染成展示图片

这个脚本对应的实验场景是：

- `bbrv2_Nflow.cc`：`N` 条长期竞争流共享同一个 dumbbell 拓扑瓶颈链路，固定用于 `BBRv2`
- `bbrv2plus_Nflow.cc`：同样的 `N` 条长期竞争流场景，但固定用于 `BBRv2plus`
- `obbr_Nflow.cc`：同样是 `N` 条长期竞争流共享一个瓶颈链路，默认使用 `oBBR`
- `freqccv3_Nflow.cc`：同样的长期竞争流场景，固定用于 `FreqCCv3`

这里要区分“场景驱动文件”和“具体算法”：

- `bbrv2_4flow.cc`、`bbrv2plus_4flow.cc`、`obbr_4flow.cc`、`freqccv3_4flow.cc` 是四个并列算法各自的 4-flow 场景驱动
- 目录名、trace 前缀和场景文件名现在按算法一一对应，不再共用一个驱动再靠参数区分

### 5.3 已实际执行的展示场景

本仓库已经实际跑通并整理了 4-flow demo，当前可直接查看的归档结果包括：

- `BBRv2`：
  [`showcase_results/BBRv2/traces/bbrv2_4flow_demo`](/home/wkd/BBR_ICC/NS3.27/showcase_results/BBRv2/traces/bbrv2_4flow_demo)
  和
  [`showcase_results/BBRv2/figs/bbrv2_4flow_demo`](/home/wkd/BBR_ICC/NS3.27/showcase_results/BBRv2/figs/bbrv2_4flow_demo)
- `BBRv2plus`：
  [`showcase_results/BBRv2plus/traces/bbrv2plus_4flow_demo`](/home/wkd/BBR_ICC/NS3.27/showcase_results/BBRv2plus/traces/bbrv2plus_4flow_demo)
  和
  [`showcase_results/BBRv2plus/figs/bbrv2plus_4flow_demo`](/home/wkd/BBR_ICC/NS3.27/showcase_results/BBRv2plus/figs/bbrv2plus_4flow_demo)
- `oBBR`：
  [`showcase_results/oBBR/traces/obbr_4flow_demo`](/home/wkd/BBR_ICC/NS3.27/showcase_results/oBBR/traces/obbr_4flow_demo)
  和
  [`showcase_results/oBBR/figs/obbr_4flow_demo`](/home/wkd/BBR_ICC/NS3.27/showcase_results/oBBR/figs/obbr_4flow_demo)
- `FreqCCv3`：
  [`showcase_results/FreqCCv3/traces/freqccv3_4flow_demo`](/home/wkd/BBR_ICC/NS3.27/showcase_results/FreqCCv3/traces/freqccv3_4flow_demo)
  和
  [`showcase_results/FreqCCv3/figs/freqccv3_4flow_demo`](/home/wkd/BBR_ICC/NS3.27/showcase_results/FreqCCv3/figs/freqccv3_4flow_demo)

对应执行命令分别是：

```bash
cd NS3.27
bash scripts/showcase/run_showcase_scenario.sh \
  --algo bbrv2plus \
  --flows 4 \
  --sim-time 10 \
  --trace-dir showcase_results/BBRv2plus/traces/bbrv2plus_4flow_demo \
  --fig-dir showcase_results/BBRv2plus/figs/bbrv2plus_4flow_demo \
  --title "4-flow BBRv2plus demo"
```

```bash
cd NS3.27
bash scripts/showcase/run_showcase_scenario.sh \
  --algo bbrv2 \
  --flows 4 \
  --sim-time 10 \
  --trace-dir showcase_results/BBRv2/traces/bbrv2_4flow_demo \
  --fig-dir showcase_results/BBRv2/figs/bbrv2_4flow_demo \
  --title "4-flow BBRv2 demo"
```

```bash
cd NS3.27
bash scripts/showcase/run_showcase_scenario.sh \
  --algo obbr \
  --flows 4 \
  --sim-time 10 \
  --trace-dir showcase_results/oBBR/traces/obbr_4flow_demo \
  --fig-dir showcase_results/oBBR/figs/obbr_4flow_demo \
  --title "4-flow oBBR demo"
```

```bash
cd NS3.27
bash scripts/showcase/run_showcase_scenario.sh \
  --algo freqccv3 \
  --flows 4 \
  --sim-time 1 \
  --trace-dir showcase_results/FreqCCv3/traces/freqccv3_4flow_demo \
  --fig-dir showcase_results/FreqCCv3/figs/freqccv3_4flow_demo \
  --title "4-flow FreqCCv3 demo"
```

### 5.4 运行 BBRv2plus 单场景

4 流示例：

```bash
cd NS3.27
./waf --run "bbrv2plus_4flow --sim_time=30 --trace_path=showcase_results/BBRv2plus/traces/bbrv2plus_4flow_demo/"
```

如果需要 ECN 版本：

```bash
cd NS3.27
./waf --run "bbrv2plus_4flow --sim_time=30 --cc=bbrv2plus_ecn --trace_path=showcase_results/BBRv2plus/traces/bbrv2plus_4flow_demo_ecn/"
```

### 5.5 运行 BBRv2plus 批量实验

```bash
cd NS3.27
bash ./run_bbrv2_batch.sh \
  --flows 4,8,16,32 \
  --sender-bw 10 \
  --bottle-bw 16 \
  --bottle-delay 28 \
  --queue-bdp 1 \
  --sim-time 30
```

批量脚本会默认输出到：

- `NS3.27/traces/bbrv2_batch_ins*`

建议将最终对外展示所需结果整理归档到：

- `NS3.27/showcase_results/BBRv2/traces/`
- `NS3.27/showcase_results/BBRv2/figs/`
- `NS3.27/showcase_results/BBRv2plus/traces/`
- `NS3.27/showcase_results/BBRv2plus/figs/`
- `NS3.27/showcase_results/oBBR/traces/`
- `NS3.27/showcase_results/oBBR/figs/`
- `NS3.27/showcase_results/FreqCCv3/traces/`
- `NS3.27/showcase_results/FreqCCv3/figs/`

### 5.6 运行 oBBR 单场景

```bash
cd NS3.27
./waf --run "obbr_4flow --sim_time=30 --trace_path=showcase_results/oBBR/traces/obbr_4flow_demo/"
```

## 6. 以后建议怎么跑

建议统一按 `4 / 8 / 16 / 32` 流组织实验。

- `BBRv2` 侧已经有：
  [`scratch/bbrv2_4flow.cc`](/home/wkd/BBR_ICC/NS3.27/scratch/bbrv2_4flow.cc)
  [`scratch/bbrv2_8flow.cc`](/home/wkd/BBR_ICC/NS3.27/scratch/bbrv2_8flow.cc)
  [`scratch/bbrv2_16flow.cc`](/home/wkd/BBR_ICC/NS3.27/scratch/bbrv2_16flow.cc)
  [`scratch/bbrv2_32flow.cc`](/home/wkd/BBR_ICC/NS3.27/scratch/bbrv2_32flow.cc)
- `BBRv2plus` 侧已经有：
  [`scratch/bbrv2plus_4flow.cc`](/home/wkd/BBR_ICC/NS3.27/scratch/bbrv2plus_4flow.cc)
  [`scratch/bbrv2plus_8flow.cc`](/home/wkd/BBR_ICC/NS3.27/scratch/bbrv2plus_8flow.cc)
  [`scratch/bbrv2plus_16flow.cc`](/home/wkd/BBR_ICC/NS3.27/scratch/bbrv2plus_16flow.cc)
  [`scratch/bbrv2plus_32flow.cc`](/home/wkd/BBR_ICC/NS3.27/scratch/bbrv2plus_32flow.cc)
- `oBBR` 侧也已补齐：
  [`scratch/obbr_4flow.cc`](/home/wkd/BBR_ICC/NS3.27/scratch/obbr_4flow.cc)
  [`scratch/obbr_8flow.cc`](/home/wkd/BBR_ICC/NS3.27/scratch/obbr_8flow.cc)
  [`scratch/obbr_16flow.cc`](/home/wkd/BBR_ICC/NS3.27/scratch/obbr_16flow.cc)
  [`scratch/obbr_32flow.cc`](/home/wkd/BBR_ICC/NS3.27/scratch/obbr_32flow.cc)
- `FreqCCv3` 侧也已补齐：
  [`scratch/freqccv3_4flow.cc`](/home/wkd/BBR_ICC/NS3.27/scratch/freqccv3_4flow.cc)
  [`scratch/freqccv3_8flow.cc`](/home/wkd/BBR_ICC/NS3.27/scratch/freqccv3_8flow.cc)
  [`scratch/freqccv3_16flow.cc`](/home/wkd/BBR_ICC/NS3.27/scratch/freqccv3_16flow.cc)
  [`scratch/freqccv3_32flow.cc`](/home/wkd/BBR_ICC/NS3.27/scratch/freqccv3_32flow.cc)

这样四个算法都可以在同一组并发流规模下对齐展示，便于做：

- 竞争公平性对比
- 队列占用和 RTT 扩张对比
- 不同并发规模下的稳定性与扩展性对比

## 7. 脚本参数、可传配置与当前限制

### 7.1 `run_showcase_scenario.sh`

支持参数：

- `--algo`：`bbrv2` / `bbrv2plus` / `bbrv2plus_ecn` / `bbrv2_noprobe_rtt` / `obbr` / `freqccv3`
- `--flows`：选择 `4 / 8 / 16 / 32`
- `--sim-time`：仿真时长，透传到 `--sim_time`
- `--trace-dir`：trace 输出目录
- `--fig-dir`：图片输出目录
- `--title`：图片标题前缀
- `--skip-plot`：只跑仿真，不出图
- `--rebuild`：先编译目标场景再运行

脚本行为：

- 当 `--algo=bbrv2` 或 `--algo=bbrv2_noprobe_rtt` 时，脚本会调用 `scratch/bbrv2_<flows>flow.cc`，结果落到 `showcase_results/BBRv2/`
- 当 `--algo=bbrv2plus` 或 `--algo=bbrv2plus_ecn` 时，脚本会调用 `scratch/bbrv2plus_<flows>flow.cc`，结果落到 `showcase_results/BBRv2plus/`
- 当 `--algo=obbr` 时，脚本会调用 `scratch/obbr_<flows>flow.cc`
- 当 `--algo=freqccv3` 时，脚本会调用 `scratch/freqccv3_<flows>flow.cc`
- 跑完后自动调用 `plot_trace_bundle.py`，生成以下 5 类展示图：
  - 多流接收速率
  - `flow1` 平滑 RTT
  - `flow1` queue delay
  - `flow1` cumulative loss
  - bottleneck queue occupancy

当前状态：

- `BBRv2`、`BBRv2plus`、`oBBR` 和 `FreqCCv3` 四个并列算法都已经补齐 `4 / 8 / 16 / 32` 流场景。

### 7.2 `bbrv2_4flow.cc`

支持参数：

- `--sim_time`
- `--trace_path`
- `--cc`

其中 `--cc` 当前支持：

- `bbrv2`
- `bbrv2_noprobe_rtt`

### 7.3 `bbrv2plus_4flow.cc`

支持参数：

- `--sim_time`
- `--trace_path`
- `--cc`

其中 `--cc` 当前支持：

- `bbrv2plus`
- `bbrv2plus_ecn`

### 7.4 `obbr_4flow.cc`

支持参数：

- `--sim_time`
- `--trace_path`

### 7.5 `freqccv3_4flow.cc` / `freqccv3_8flow.cc` / `freqccv3_16flow.cc` / `freqccv3_32flow.cc`

支持参数：

- `--sim_time`
- `--trace_path`
- `--freq1..N`
- `--amp1..N`
- `--fixed1..N`
- `--interval_win_rtt_mult`

### 7.6 `run_bbrv2_batch.sh`

支持参数：

- `--flows`
- `--sender-bw`
- `--bottle-bw`
- `--bottle-delay`
- `--queue-bdp`
- `--sim-time`
- `--probe-rtt`
- `--instance`
- `--rebuild`

这个脚本的含义不是“单次展示跑一个 demo”，而是批量扫描 `4 / 8 / 16 / 32` 流场景，快速得到一整组 BBRv2 家族结果。

它目前会在运行前直接修改目标 `scratch/bbrv2_Nflow.cc` 中的 4 个拓扑常量，然后构建并执行：

- `TOPO_SENDER_BW`
- `TOPO_BOTTLE_BW`
- `TOPO_BOTTLE_PDELAY`
- `TOPO_DEFAULT_QDELAY`

也就是说，当前批量脚本可改的核心配置包括：

- 边缘链路带宽
- 瓶颈带宽
- 瓶颈传播时延
- 队列大小（按 `BDP` 因子换算为毫秒）
- 仿真时长
- 是否打开 `ProbeRTT`

如果后续要让 `oBBR` 也支持同样的批量跑法，最直接的方式就是参照这个脚本补一个 `run_obbr_batch.sh`，并让它调用 `obbr_4flow.cc / obbr_8flow.cc / obbr_16flow.cc / obbr_32flow.cc`。

### 7.7 `run_freqccv3_batch.sh`

支持参数：

- `--flows`
- `--sender-bw`
- `--bottle-bw`
- `--bottle-delay`
- `--queue-bdp`
- `--sim-time`
- `--freqall`
- `--ampall`
- `--fixedall`
- `--freq1..32`
- `--amp1..32`
- `--fixed1..32`
- `--interval-win-rtt-mult`
- `--instance`
- `--rebuild`

这个脚本的含义是批量扫描 `FreqCCv3` 的 `4 / 8 / 16 / 32` 流场景，并把每条流的频率、幅度模式和固定振幅参数一起透传给对应的 `scratch/freqccv3_Nflow.cc`。

它目前会在运行前直接修改目标 `scratch/freqccv3_Nflow.cc` 中的 4 个拓扑常量，然后构建并执行：

- `TOPO_SENDER_BW`
- `TOPO_BOTTLE_BW`
- `TOPO_BOTTLE_PDELAY`
- `TOPO_DEFAULT_QDELAY`

也就是说，当前 `FreqCCv3` 批量脚本既能改链路模板，也能改算法特有参数：

- 边缘链路带宽
- 瓶颈带宽
- 瓶颈传播时延
- 队列大小（按 `BDP` 因子换算为毫秒）
- 仿真时长
- 全流统一的 `freq / amp / fixed`
- 指定单流的 `freqN / ampN / fixedN`
- `interval_win_rtt_mult`

### 7.8 场景源码里还能直接修改什么

无论是 [`scratch/bbrv2_4flow.cc`](/home/wkd/BBR_ICC/NS3.27/scratch/bbrv2_4flow.cc)、[`scratch/bbrv2plus_4flow.cc`](/home/wkd/BBR_ICC/NS3.27/scratch/bbrv2plus_4flow.cc)、[`scratch/obbr_4flow.cc`](/home/wkd/BBR_ICC/NS3.27/scratch/obbr_4flow.cc) 还是 [`scratch/freqccv3_4flow.cc`](/home/wkd/BBR_ICC/NS3.27/scratch/freqccv3_4flow.cc)，当前都保留了几类最直接的源码级可调项：

- `NUM_FLOWS`：并发流数量
- `TOPO_SENDER_BW`：发送侧边缘链路带宽
- `TOPO_BOTTLE_BW`：瓶颈带宽
- `TOPO_BOTTLE_PDELAY`：瓶颈传播时延
- `TOPO_DEFAULT_QDELAY`：队列时延预算，对应队列深度

除此之外，源码里还可以继续细调：

- 每条流的启动时间偏移
- 仿真停止时间
- trace 输出路径
- `BBRv2` 家族使用的具体 `cc` 名称

因此现在的推荐分工是：

- 展示型单次运行：优先用 `run_showcase_scenario.sh`
- BBRv2 家族批量扫参：优先用 `run_bbrv2_batch.sh`
- 改拓扑模板本身：直接改 `scratch/*.cc`

## 8. 实验结果如何组织

为便于对外展示，本仓库统一采用如下结果目录：

```text
NS3.27/showcase_results/
├── BBRv2/
│   ├── traces/
│   └── figs/
├── BBRv2plus/
│   ├── traces/
│   └── figs/
├── FreqCCv3/
│   ├── traces/
│   └── figs/
└── oBBR/
    ├── traces/
    └── figs/
```

推荐约定如下：

- `traces/`：保存仿真原始输出，如 `bw`、`rtt`、`goodput`、`queue_delay`、`loss_rate`、`bbr_mode` 等文本数据
- `figs/`：保存对外展示图片，如 `png`、`pdf`

这样做的好处是：

- 四个算法的结果路径完全对齐
- 汇报材料和论文插图不再散落在各脚本目录
- 后续增加 `BBRv2`、`CUBIC`、`FreqCCv3` 等算法时也能按同一结构扩展

## 9. 如何看实验图片

### 7.1 oBBR 原始 artifact 的画图方式

`oBBR` 原始 artifact 已经提供了成熟的画图流水线：

- 原始数据目录：
  [`oBBR/nginx-quic/data`](/home/wkd/BBR_ICC/oBBR/nginx-quic/data)
- 画图脚本目录：
  [`oBBR/nginx-quic/scripts/plots`](/home/wkd/BBR_ICC/oBBR/nginx-quic/scripts/plots)
- 图片输出目录：
  [`oBBR/nginx-quic/figs`](/home/wkd/BBR_ICC/oBBR/nginx-quic/figs)

其基本模式是：

```text
data/ -> plots/*.py -> figs/*.png
```

例如：

- [`oBBR/nginx-quic/scripts/plots/fig7.py`](/home/wkd/BBR_ICC/oBBR/nginx-quic/scripts/plots/fig7.py)
- [`oBBR/nginx-quic/scripts/plots/fig8.py`](/home/wkd/BBR_ICC/oBBR/nginx-quic/scripts/plots/fig8.py)

### 7.2 本仓库推荐的统一展示方式

对外展示时，建议沿用 `oBBR` 的“原始数据与图片分离”思路，但统一收口到：

- `NS3.27/showcase_results/BBRv2/figs/`
- `NS3.27/showcase_results/BBRv2plus/figs/`
- `NS3.27/showcase_results/FreqCCv3/figs/`
- `NS3.27/showcase_results/oBBR/figs/`

也就是说：

- `oBBR` 的图片可以继续参考原有 Python 画图脚本风格生成
- `BBRv2` 与 `BBRv2plus` 的图片都应采用同样的 Matplotlib 风格，但结果目录必须分开

这样在汇报时，只需要打开一个总目录即可找到四个算法的图片成果。

目前已经统一生成的展示图位于：

- [`showcase_results/BBRv2/figs/bbrv2_4flow_demo`](/home/wkd/BBR_ICC/NS3.27/showcase_results/BBRv2/figs/bbrv2_4flow_demo)
- [`showcase_results/BBRv2plus/figs/bbrv2plus_4flow_demo`](/home/wkd/BBR_ICC/NS3.27/showcase_results/BBRv2plus/figs/bbrv2plus_4flow_demo)
- [`showcase_results/FreqCCv3/figs/freqccv3_4flow_demo`](/home/wkd/BBR_ICC/NS3.27/showcase_results/FreqCCv3/figs/freqccv3_4flow_demo)
- [`showcase_results/oBBR/figs/obbr_4flow_demo`](/home/wkd/BBR_ICC/NS3.27/showcase_results/oBBR/figs/obbr_4flow_demo)

每个目录下都包含：

- `recv_rate.png`
- `flow1_srtt.png`
- `flow1_qdelay.png`
- `flow1_cum_loss.png`
- `bottleneck_queue.png`

## 10. 对外展示时建议突出什么

建议汇报时重点强调四点：

- 这不是简单“接线”，而是将两个不同来源的控制算法统一迁移到 `ns-3/DQC` 宿主中，形成可比较、可复现的实验平台。
- 迁移工作量明确，核心 sender、实验场景、DQC 接入、trace/export、批量实验链路都已补齐。
- `BBRv2plus` 适合强调其对 `DQC BBRv2` 的增量机制迁移；`oBBR` 适合强调其针对重传优化与带宽下降场景的响应能力。
- 结果目录已经统一，后续无论是论文图、答辩图还是项目汇报图，都可以从 `showcase_results/` 直接取用。

## 11. 相关补充文档

- [`README_DQC_BBRV2PLUS.md`](/home/wkd/BBR_ICC/NS3.27/README_DQC_BBRV2PLUS.md)
- [`oBBR/README.md`](/home/wkd/BBR_ICC/oBBR/README.md)
