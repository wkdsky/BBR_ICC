# FBBR：基于固定频率 CRUISE 调制的 BBRv2 拥塞控制实现

本仓库的当前研究主线是 FBBR。FBBR 是一个基于 BBRv2 的 ns-3.27/DQC 拥塞控制实现：它在 `PROBE_BW` 的 `PROBE_CRUISE` 阶段注入已知频率的三角 pacing 波，观测发送速率、交付速率和 SRTT 对该激励的响应，判断路径处于欠载、满载还是过载状态，并据此更新后续 pacing 使用的带宽基线。

FBBR 的目标是在保持较高链路利用率的同时减少持续排队和丢包。它不读取显式公平份额，不修改原生 BBRv2 的拥塞窗口上限，也不维护独立的 FBBR RTprop；当前实现直接复用 BBRv2 的 `MinRtt`、`cwnd_`、ProbeRTT、丢包/ECN 响应和主状态机。

本文只描述当前工作树中的 FBBR。旧的 ICC、FreqCC、FreqCCv2、FreqCCv3 代码仍保留在仓库中用于历史追踪或对比，但不是当前 FBBR 算法的实现依据。

## 核心文件位置

| 内容 | 文件 |
| --- | --- |
| FBBR 控制器接口与状态 | [`fbbr_sender.h`](NS3.27/src/dqc/model/thirdparty/congestion/fbbr_sender.h) |
| FBBR 核心控制逻辑 | [`fbbr_sender.cc`](NS3.27/src/dqc/model/thirdparty/congestion/fbbr_sender.cc) |
| FBBR 配置结构与解析声明 | [`fbbr_config_loader.h`](NS3.27/src/dqc/model/thirdparty/congestion/fbbr_config_loader.h) |
| FBBR 严格配置解析器 | [`fbbr_config_loader.cc`](NS3.27/src/dqc/model/thirdparty/congestion/fbbr_config_loader.cc) |
| DQC 发送端接入与配置下发 | [`dqc_sender.cc`](NS3.27/src/dqc/model/dqc_sender.cc) |
| 拥塞控制器工厂注册 | [`proto_send_algorithm_interface.cc`](NS3.27/src/dqc/model/thirdparty/congestion/proto_send_algorithm_interface.cc) |
| 当前统一实验配置 | [`fbbr_default.conf`](NS3.27/examples/CCconfig/fbbr_default.conf) |
| 完整算法说明 | [`FBBR_ALGORITHM.md`](FBBR_ALGORITHM.md) |
| 三组实验操作手册 | [`实验运行操作手册.md`](NS3.27/examples/paper-test/实验运行操作手册.md) |

## FBBR 与原生 BBRv2 的关系

FBBR 继承 `Bbr2Sender`，默认通过 DQC 的 `kFBBR` 类型创建。当前控制边界如下：

| 模块 | 当前 FBBR 行为 |
| --- | --- |
| STARTUP / DRAIN | 使用原生 BBRv2 行为。 |
| PROBE_BW 状态转换 | 保留原生 DOWN、CRUISE、REFILL、UP 转换；活动波形期间可把正常 CRUISE 退出延迟到完整周期边界。 |
| ProbeRTT | `kFBBR` 默认保留；工厂另有 `kFBBRNoProbeRtt` 供专门实验使用。 |
| 带宽模型 | 复用 BBRv2 的 `MinBandwidth()`、`MaxBandwidth()` 和 `BandwidthEstimate()`。 |
| RTprop | 直接使用 BBRv2 `model_.MinRtt()`，没有独立 FBBR RTprop。 |
| 拥塞窗口 | `GetCongestionWindow()` 直接返回原生 `cwnd_`。 |
| 丢包与 ECN | 使用原生 BBRv2 的处理路径。 |
| FBBR 扩展 | CRUISE 固定频率激励、响应采集、负载分类、基线调整、BEQ 发布与 pacing 基线选择。 |

当前版本已经移除旧的 inflight 服务包络、`plan_inflight`、`service_inflight`、正向探测 credit 和额外 inflight cap。因此，FBBR 的低排队效果来自 pacing 基线和波形控制，而不是额外收紧拥塞窗口。

## 总体执行链

```text
BBRv2 STARTUP / DRAIN / PROBE_BW
                 |
                 v
          进入 PROBE_CRUISE
                 |
                 +-- 选择注入基线 B：上一轮有效 BEQ 或原生 MaxBw
                 +-- 计算幅度 A 与有效幅度 Aeff
                 +-- 注入 0 -> -1 -> 0 -> +1 -> 0 三角波
                 |
                 v
          等待 1 个 SRTT 完成 settle
                 |
                 v
          收集 sender-rate、delivery-rate、SRTT
                 |
                 +-- 重采样、覆盖率和 app-limited 检查
                 +-- 在已知频率上执行 Goertzel 检测
                 +-- 检查 SRTT 活动、肩部、水平段和重复裁剪
                 |
                 v
       Regime I / II / III / INCONCLUSIVE
                 |
                 +-- I：提高基线 B
                 +-- II：保持基线 B
                 +-- III：降低基线 B
                 +-- 不确定：扩窗或放大激励后重试
                 |
                 v
          离开 CRUISE 时发布 BEQ
                 |
                 v
       REFILL / UP / DOWN 使用新鲜 BEQ
```

发生丢包、ECN、recovery 或其他安全退出条件时，FBBR 不会为了等待波形周期完整而阻止原生安全路径。

## CRUISE 三角波注入

### 关键记号

| 记号 | 当前实现含义 |
| --- | --- |
| `B` | 当前 CRUISE 注入基线 `current_injection_baseline_bw_`。 |
| `A` | 请求的单边三角波幅度。 |
| `Aeff` | 经过最低 pacing 速率保护后的实际幅度。 |
| `f` | 固定调制频率，当前默认 5 Hz。 |
| `F` | 最低 pacing 速率，当前默认 0.2 Mbit/s。 |
| `R` | BBRv2 的 `MinRtt()`。 |
| `M` | MaxBw 更新时刻附近观测到的最大 SRTT。 |
| `BEQ` | 当前 CRUISE 结束时发布、供后续 pacing 使用的带宽基线。 |

### 波形公式

CRUISE 活动窗口中的 pacing 为：

```text
Pacing(t) = max(F, B + Aeff * triangle(t))
```

设一个周期内的归一化相位为 `q`：

```text
0.00 <= q < 0.25: triangle(q) = -4q
0.25 <= q < 0.75: triangle(q) = 4q - 2
0.75 <= q < 1.00: triangle(q) = 4 - 4q
```

因此相位顺序是：

```text
0 -> -1 -> 0 -> +1 -> 0
```

负半波先出现，用于先观察降低发送速率时交付速率和 RTT 是否同步下降，再观察正半波是否触发交付增长或排队响应。

当前默认幅度模式为 `4sr`：

```text
A = 当前原生发送 pacing rate / 4
Aeff = min(A, B - F)
```

如果 `B <= F`，则 `Aeff=0`，避免负半波把发送速率压到最低速率以下。

### 支持的幅度模式

| 模式 | 含义 |
| --- | --- |
| `Nsr` | 当前发送速率除以 `N`，`N` 支持 1～20；例如 `4sr=SR/4`。 |
| `fixed_mbps` | 使用配置项 `default_fixed_amplitude_mbps`。 |
| 正数 Mbps | 直接作为固定幅度。 |
| `miu2/miu3/miu4/miu8` | 为兼容旧实验保留的带宽估计比例模式。 |

每流覆盖参数使用从 0 开始的流 ID，例如 `flow.0.modulation_freq_hz`。

## 信号采集与频率检测

FBBR 在 CRUISE 中维护三类历史：

| 信号 | 采集时机 | 用途 |
| --- | --- | --- |
| sender-rate | 每次发送 | 确认实际命令 pacing 中存在注入频率。 |
| delivery-rate | 有效 ACK 到达 | 观察路径交付速率是否响应注入。 |
| SRTT | 有效 ACK 到达 | 观察排队和传播时延响应。 |

默认先等待 1 个 SRTT，再采集 2 个完整调制周期。结果不确定时，窗口可扩展到 3 个周期。重采样步长根据周期计算，并限制在 1～5 ms；超过允许插值空洞、覆盖率不足或 app-limited 样本过多时，本次输入会被判为无效。

### Goertzel 目标频率检测

FBBR 已知注入频率 `f`，因此不需要在整个频谱中搜索主频。它直接在 sender-rate 和 delivery-rate 上计算 generalized Goertzel 分量：

```text
omega = 2*pi*f*sample_step
s[n] = (x[n] - mean) + 2*cos(omega)*s[n-1] - s[n-2]

coherent_power_ratio = |X(f)|^2 / (N * sum((x[n]-mean)^2))
```

当前配置要求：

```text
coherent_power_ratio >= 0.10
```

sender-rate 和 delivery-rate 都存在目标频率分量时，才把交付速率视为对注入波形产生了同频响应。

### SRTT 时域证据

SRTT 不只检查目标频率，还分析波形形状和裁剪特征，包括：

- p95-p05 活动幅度、显著变化步数和斜率反转；
- 顶部或底部连续水平段；
- 正肩部和负肩部；
- 两周期重复顶部或底部裁剪；
- 中部序列扰动和掩码后的周期一致性；
- `SRTTmax` 与 MaxBw 周围最大 SRTT `M` 的关系；
- `SRTTmin` 与 BBRv2 `MinRtt` `R` 的关系。

完整的 N01～N16 判定树见 [`FBBR_ALGORITHM.md`](FBBR_ALGORITHM.md)。

## Regime 判定与基线调整

FBBR 将有效窗口分为四类：

| 分类 | Regime | 解释 | 基线动作 |
| --- | --- | --- | --- |
| `UNDERLOAD` | I | 路径仍有可利用带宽，或交付速率能较好跟随激励且 RTT 未表现出持续过载。 | 提高 `B`。 |
| `FULL_LOAD` | II | 当前基线接近可用服务能力。 | 保持 `B`。 |
| `OVERLOAD` | III | 出现 RTT 上涨、顶部裁剪或交付响应受限等过载证据。 | 降低 `B`。 |
| `INCONCLUSIVE` | 无 | 样本不足、证据冲突或波形质量不够。 | 保持 `B`，扩窗或放大信号重试。 |

### Regime I：提高基线

优先级为：

1. 若 `Dmax` 位于当前 `B` 与 BBRv2 `MaxBw` 之间，使用 `Dmax`；
2. 否则尝试使用 `(Dmax + MaxBw) / 2`；
3. 仍无合法候选时使用 `1.02 * B`；
4. 最终结果不得低于最低 pacing 速率 `F`。

### Regime III：降低基线

优先级为：

1. 若 `Dmin` 位于 BBRv2 `MinBw` 与当前 `B` 之间，使用 `Dmin`；
2. 否则尝试使用 `MinBw + (Dmin - MinBw) / 2`；
3. 仍无合法候选时使用 `0.98 * B`；
4. 最终结果不得低于 `F`。

### Regime II：保持基线

当前版本已经移除 inflight 服务包络和对应的区间交付率计算，因此 Regime II 不再生成新的区间速率候选，直接保持当前基线并重新进入 settle。

### 不确定窗口

首次 `INCONCLUSIVE` 会延长观察窗口。持续不确定时，FBBR 可依据交付速率响应幅度提高激励；若响应幅度不可用，则以当前 `B` 的一定比例作为放大基数。放大仍受初始幅度倍数上限和最低 pacing 速率保护。

## BEQ 选择与应用

BEQ 是 FBBR 在一轮 CRUISE 结束时发布的服务基线估计。它不是显式公平份额，控制器也不会读取实验中的理想公平份额作为输入。

当前 BEQ 选择优先级为：

1. 使用本轮 CRUISE 产生的合法波形 BEQ；
2. 否则，对 SRTT 位于 `[1.05R, 1.10R]` 的 delivery-rate 样本做时间加权平均；
3. 若没有符合该 SRTT 区间的样本，对整个 CRUISE 的有效 delivery-rate 做时间加权平均；
4. 若仍无有效样本，依次回退到原生 MaxBw、初始 CRUISE 基线、BandwidthEstimate 和最低 pacing 速率。

时间加权平均定义为：

```text
BEQ = sum(delivery_rate_i * duration_i) / sum(duration_i)
```

新鲜且有效的 BEQ 可在 `PROBE_REFILL`、`PROBE_UP`、`PROBE_DOWN` 等非 CRUISE 阶段参与 pacing。活动 CRUISE 仍使用当前注入基线 `B`；上一轮有效 BEQ 可以作为下一轮 CRUISE 的初始基线。

## MaxBw-flat 安全试验

当原生 `MaxBw` 高于当前 CRUISE 初始基线时，FBBR 可以暂时停止三角波，以 MaxBw 平速发送并观察若干 RTT：

- 出现丢包、ECN、recovery 或 app-limited 时拒绝；
- SRTT 明显高于 RTprop 或进入试验时的 SRTT 时拒绝；
- 交付速率无法接近试验速率时拒绝；
- 连续 3 个合格 RTT 后接受，把 CRUISE 基线提升到该 MaxBw。

试验期间的 ACK 不参与普通波形分类、收敛轮次或 CRUISE 时间加权 BEQ，避免把安全验证过程污染为常规响应样本。

## 当前默认配置

权威配置文件为 [`NS3.27/examples/CCconfig/fbbr_default.conf`](NS3.27/examples/CCconfig/fbbr_default.conf)。配置加载器采用严格模式：未知 key、非法幅度模式、格式错误或越界值会导致加载失败，而不是静默忽略。

### 主要生效参数

| 参数 | 当前值 | 作用 |
| --- | ---: | --- |
| `default_modulation_freq_hz` | 5.0 | CRUISE 三角波频率，周期 0.2 s。 |
| `default_amplitude_mode` | `4sr` | 幅度为当前原生发送速率的 1/4。 |
| `default_fixed_amplitude_mbps` | 10.0 | 仅 `fixed_mbps` 模式使用。 |
| `pacing.minimum_rate_mbps` | 0.2 | 最低 pacing 速率和负半波保护下限。 |
| `waveform.initial_window_periods` | 2.0 | 初始观察窗口。 |
| `waveform.extended_window_periods` | 3.0 | 不确定结果的扩展窗口。 |
| `waveform.max_window_periods` | 3.0 | 最大观察窗口。 |
| `waveform.min_cycle_coverage_ratio` | 0.85 | 最低重采样覆盖率。 |
| `waveform.max_app_limited_sample_ratio` | 0.25 | delivery-rate 窗口允许的最大 app-limited 比例。 |
| `goertzel.min_coherent_power_ratio` | 0.10 | 目标频率分量存在阈值。 |
| `stability.single_round_exit_threshold` | 0.25 | 单轮交付率变化退出稳定状态的阈值。 |
| `stability.consecutive_exit_threshold` | 0.15 | 连续两轮变化退出稳定状态的阈值。 |
| `stability.stable_rounds` | 3 | 回到稳定状态所需轮次。 |

配置文件还包含活动检测、水平段、肩部、重复裁剪、中部扰动和 Regime 周期性判断参数。完整解释见 [`FBBR_ALGORITHM.md`](FBBR_ALGORITHM.md)。

### 收敛门控说明

FBBR 会持续计算 BBR 稳定/非稳定状态并输出诊断信息。当前 paper-test runner 默认没有开启 `enable_convergence_gate_control`，因此稳定状态不会自动关闭 CRUISE 波形；门控状态默认只用于观测。

配置中的部分 `trace.*` 字段由解析器接受，但 paper-test 的 `ConfigureFBBR()` 路径并未应用全部 trace 开关。需要精确诊断采样粒度时，应同时检查 runner 是否调用了 `ConfigureFBBRConvergenceGate()`；详见算法文档的 trace 说明。

记录配置版本：

```bash
sha256sum NS3.27/examples/CCconfig/fbbr_default.conf
```

## 编译与运行

### 环境

当前实现运行在 Linux、ns-3.27 和 DQC 上。分析脚本使用 Python 3，并依赖：

```text
pandas
matplotlib
numpy
```

检查 Python 依赖：

```bash
python3 -c 'import pandas, matplotlib, numpy; print("analysis dependencies: OK")'
```

仓库根目录的 [`install_deps.sh`](install_deps.sh) 和 [`NS3.27/README.md`](NS3.27/README.md) 保留了 ns-3.27 的依赖说明，其中部分包名面向较老 Linux 发行版；新系统应安装对应的等价软件包。

### 只运行 FBBR

Test 3 支持算法筛选，是最直接的 FBBR 单算法入口。运行脚本会自动配置、编译、执行仿真并生成分析结果：

```bash
cd NS3.27
examples/paper-test/test3/run_test3.sh --algorithm=FBBR --jobs=1
```

结果写入：

```text
NS3.27/results/test3/
```

### 运行完整论文实验

从 `NS3.27` 目录依次执行：

```bash
examples/paper-test/test1/run_test1.sh --jobs=4
examples/paper-test/test2/run_test2.sh --jobs=4
examples/paper-test/test3/run_test3.sh --jobs=4
```

详细的环境检查、结果备份、筛选参数、重分析命令和故障排查见 [`实验运行操作手册.md`](NS3.27/examples/paper-test/实验运行操作手册.md)。

## 三组 paper-test 实验

| 实验 | 场景 | 主要目的 | 说明 |
| --- | --- | --- | --- |
| Test 1 | 100 Mbit/s、40 ms RTT、40 BDP，流数 `2→4→8→16→8→4→2` | 观察动态增减流下的吞吐、排队、公平性和带宽估计 | [`test1/README.md`](NS3.27/examples/paper-test/test1/README.md) |
| Test 2 | 固定 4 流，组合 10/100/1000 Mbit/s、10/40/200 ms RTT 和 0.5/2 BDP | 比较固定场景中的容量、RTT 和缓冲敏感性 | [`test2/README.md`](NS3.27/examples/paper-test/test2/README.md) |
| Test 3 | 固定 4 流和 100 Mbit/s，RTT 按 `40→120→30→80→40 ms` 变化 | 隔离动态传播 RTT 对控制器的影响 | [`test3/README.md`](NS3.27/examples/paper-test/test3/README.md) |

三组实验的主要对比算法为：

```text
BBR-R, oBBR, BBRv2+, CUBIC, BBRv2-formal/BBRv2-ideal, BBRv2, FBBR
```

Test 1 的形式化 BBRv2 参数名为 `BBRv2-ideal`，Test 2 和 Test 3 使用 `BBRv2-formal`。

### 当前结果应如何解读

现有实验总体显示，FBBR 在多组场景中能够维持接近瓶颈容量的 goodput，并明显降低原始 BBRv2 的排队和丢包；其主要代价是部分多流、低带宽、长 RTT 或动态阶段中的 Jain 公平性和收敛速度。

这些结果具有以下边界：

- paper-test 主要是 `seed=1` 的确定性运行，不代表跨 seed 统计显著性；
- Test 1 的最终展示数据可能使用单独重跑选择，应以该目录的 `selection.json` 和 README 为准；
- 深缓冲、浅缓冲、固定 RTT 和动态 RTT 的结果不能相互替代；
- 公平性、排队、吞吐和丢包必须一起解读，不能只用单一指标判断算法优劣。

## 输出与调试

标准实验输出位于 `NS3.27/results/test1`、`test2` 和 `test3`：

```text
raw/                 原始 CSV/JSON 和本次运行 manifest
logs/                每个场景、算法的模拟器日志
summary/             聚合指标、校验结果和绘图中间数据
figures/             Test 1/Test 3 的 PNG/PDF 图
RESULTS.md            自动生成的结果报告
```

诊断 FBBR 时重点查看：

| 问题 | 关键字段或事件 |
| --- | --- |
| 为什么进入 I/II/III | `decision_rule`、分类结果、SRTT/DRate 波形有效性、裁剪类型、`M`、`R`、`Dmin`、`Dmax`。 |
| 为什么调整基线 | `waveform_last_action`、`delta_source`、原始/应用后的 baseline delta。 |
| 为什么发布该 BEQ | `beq_source`、`beq_valid`、`beq_fresh`、`beq_application_valid`。 |
| 为什么没有分类 | `invalid_reason`、coverage、app-limited ratio、Goertzel reason 和 retry 状态。 |
| pacing 最终采用什么基线 | `pacing_base_source`、`native_pacing_bps`、`final_pacing_rate_bps`、`triangle_wave`。 |
| 稳定门控是否影响控制 | `bbr_stable`、`freq_tool_needed`、`freq_tool_on` 和门控控制开关。 |

Test 3 的每流 `controller_trace.csv` 会记录 BBR 状态、pacing、RTT、BEQ 和 FBBR 动作，适合检查动态 RTT 下的控制过程。

## 对比实现

| 算法 | 本仓库入口或说明 |
| --- | --- |
| BBR-R | [`README_BBR_R_MIGRATION.md`](NS3.27/README_BBR_R_MIGRATION.md) |
| oBBR | [`README_OBBR_MIGRATION.md`](NS3.27/README_OBBR_MIGRATION.md) |
| BBRv2+ | [`README_DQC_BBRV2PLUS.md`](NS3.27/README_DQC_BBRV2PLUS.md) |
| CUBIC | [`README_CUBIC_NS3_47_MIGRATION.md`](NS3.27/README_CUBIC_NS3_47_MIGRATION.md) |
| BBRv2 | DQC 原生 `Bbr2Sender` 控制路径 |
| FBBR | `FBBRSender`，DQC 类型 `kFBBR` |

## 文档索引

- [`FBBR_ALGORITHM.md`](FBBR_ALGORITHM.md)：当前 FBBR 的完整状态、判定树、执行器、BEQ 和配置说明。
- [`实验运行操作手册.md`](NS3.27/examples/paper-test/实验运行操作手册.md)：Test 1～3 的完整运行流程。
- [`Test 1 README`](NS3.27/examples/paper-test/test1/README.md)：动态流数实验及当前选定结果。
- [`Test 2 README`](NS3.27/examples/paper-test/test2/README.md)：固定四流场景矩阵。
- [`Test 3 README`](NS3.27/examples/paper-test/test3/README.md)：动态传播 RTT 实验。
- [`FBBR-internet测试.md`](FBBR-internet测试.md)：已有互联网路径实验记录。

若源码、配置和文档出现不一致，优先级依次为：当前 `fbbr_sender.*` 源码、`fbbr_config_loader.*`、实际运行使用的 `fbbr_default.conf`、本 README 和其他实验记录。
