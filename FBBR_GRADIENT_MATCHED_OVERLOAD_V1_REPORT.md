# FBBR Gradient-Matched Overload V1 实验报告

日期：2026-07-25  
工作区：`/home/wkd/FreqBBR`  
实验结果根目录：`NS3.27/results/fbbr_gradient_matched_overload_v1`

## 1. 结论摘要

第一阶段修改完成，编译、自测、smoke test 和四个完整 FBBR-hybrid
实验均返回 0。BBR-R、BBRv2 源码未修改，参考结果复用了相同拓扑、
seed=1、runId=1 且 `return_code=0` 的既有实验。

结论分为两部分：

1. **离散 OVERLOAD 执行器的过度降速是固定 4-flow 吞吐不足的主要原因。**
   固定 4-flow 聚合吞吐从 90.405 提升到 97.386 Mbps，Jain 从
   0.916419 提升到 0.953042；原弱流 flow2/flow4 的吞吐分别提升
   8.066/2.368 Mbps，平均 baseline 分别提升 12.350/5.765 Mbps。
2. **V1 尚未满足整体队列安全门槛。** 固定 4-flow 的 P95 队列下降
   22.68%，但平均队列上升 62.92%；固定 32-flow 的平均/P95 队列分别
   上升 315.14%/279.67%，Jain 也下降。因此当前结果不支持直接进入
   queue-neutral baseline recovery：固定 4-flow 吞吐问题已经基本消失，
   此时主动恢复更可能加重队列。后续若继续研究，应先解决当前执行器在
   高并发下的聚合队列控制偏弱问题，而不是实现第二阶段恢复。

本报告没有实现 baseline 主动恢复、QNBA、周期响应增益估计、
tentative commit 或其他第二阶段机制。

## 2. 修改范围

### 2.1 修改文件与函数

- `NS3.27/src/dqc/model/thirdparty/congestion/fbbr_sender.cc`
  - `ComputeRobustQueueGradient()`：从当前分类窗口的原始 SRTT 历史样本
    计算 P90 队列、winsorized OLS 梯度和 MAD 噪声区。
  - `ComputeGradientMatchedDecrease()`：计算 `q_guard`、`g_desired` 和
    限幅后的 `beta`。
  - `FBBRSender::ComputeFbbrHybridInjectionBaseline()`：仅替换
    FBBR-hybrid 的 OVERLOAD 分支；FULL_LOAD、UNDERLOAD 原分支不变。
  - `FBBRSender::AnalyzeFbbrHybridWindow()`：为当前实际分类窗口生成队列
    梯度输入。
  - `FBBRSender::ApplyFbbrHybridClassification()`：把连续 β 交给执行器，
    并输出既有 waveform trace 中的新 hold/decrease action 名称。
  - `FBBRSender::RunFbbrHybridSelfTest()`：增加秒单位、P90、重复时间戳、
    hold/decrease 和 10% cap 的边界测试。
- `NS3.27/src/dqc/model/thirdparty/congestion/fbbr_sender.h`
  - 增加小型内部结果字段和执行器参数；没有修改其他 CC 接口。
- `NS3.27/scripts/analyze_fbbr_gradient_matched_v1.py`
  - 从既有 classification/baseline trace 离线恢复 β，生成场景、OVERLOAD
    和固定 4-flow 逐流汇总。没有增加逐 ACK、逐包或新的运行时日志。
- `NS3.27/examples/ConcurrentFlow/cellular_gradient_matched_v1.json`
  - 固化与原 128M/5BDP/180s Cellular Taxi 实验相同的输入参数。

`dqc_trace.cc`、BBR-R、BBRv2 均未修改。现有 trace 已包含
`classification/action/baseline_before/baseline_after`，足以离线计算 β，
因此没有扩展 trace schema。

### 2.2 分类器与其他动作保持情况

- Regime III fallback 仍为：

  \[
  SRTT_{\max} >
  \max\left(1.1RTprop,\ RTprop+\frac{MaxSRTT-RTprop}{3}\right)
  \]

- inflight fallback、波形分类、Goertzel、横切判断、RTprop/MaxSRTT 和
  `baseline_low` 更新未改。
- FULL_LOAD 仍只用 `meandrate` 更新 TrustedBw，不更新 baseline。
- UNDERLOAD 仍是原 `bracket_target / midpoint / maxdrate` 执行链。
- 没有启用可选的 3% loss floor。当前 loss 事件语义不能可靠区分蜂窝随机
  丢包与拥塞丢包，直接启用会违反“非拥塞错误不视作拥塞”的约束。

## 3. 旧执行器与新执行器

旧 OVERLOAD 分支在每个日常判定窗口选择以下离散目标之一：

- `baseline_low + (MaxBw - baseline_low) / 4`
- RTpropDRate midpoint
- 窗口 `mindrate`

新分支完全不读取这些值作为 OVERLOAD 目标。它只对当前 baseline 施加连续
乘法降幅：

\[
baseline_{next}
=
\max\left(minimum\_rate,\ baseline_{current}(1-\beta)\right)
\]

如果控制输入无效，安全行为是 `beta=0`、hold 当前 baseline，不回退到离散
强降速。

## 4. 实际实现公式与参数

原始 SRTT 存储单位为毫秒、时间戳为 `QuicTime` 微秒。helper 内分别除以
1000 和 1,000,000，统一转换到秒：

\[
q_k=\max(0,SRTT_k-RTprop), \qquad q_{90}=P90(q_k)
\]

\[
q_{\mathrm{guard}}
=
\max\left(0.1RTprop,\frac{MaxSRTT-RTprop}{3}\right)
\]

实现先把 \(q_k\) winsorize 到 P5–P95，然后以窗口首样本时间为数值原点做
普通最小二乘回归：

\[
q_k=a+g_{\mathrm{raw}}t_k
\]

相邻 winsorized 样本给出：

\[
v_k=\frac{q_k-q_{k-1}}{t_k-t_{k-1}}, \qquad
g_{\mathrm{noise}}=1.4826\,MAD(v_k)
\]

\[
g=
\begin{cases}
0,& |g_{\mathrm{raw}}|\le 2g_{\mathrm{noise}}\\
g_{\mathrm{raw}},& \text{otherwise}
\end{cases},
\qquad
g=\operatorname{clip}(g,-0.5,0.5)
\]

少于 4 个有效样本、重复/逆序时间戳、非有限值、零时间方差或回归失败时，
\(g=0\)，不会触发旧的强降速 fallback。

对当前实际参与分类的完整窗口：

\[
q_{\mathrm{excess}}=\max(0,q_{90}-q_{\mathrm{guard}})
\]

\[
T_d=2T_{\mathrm{window}}, \qquad
g_{\mathrm{desired}}=-\frac{q_{\mathrm{excess}}}{T_d}
\]

\[
denominator=\max(0.5,1+g)
\]

\[
\beta^*
=
1-\frac{1+g_{\mathrm{desired}}}{denominator},
\qquad
\beta=\operatorname{clip}(\beta^*,0,0.10)
\]

没有新增 Mbps floor、flow-count 分支或可调参数。

## 5. 编译、自测与实验返回码

| 项目 | 命令/场景 | 仿真/测试 rc | 绘图 rc |
|---|---|---:|---:|
| 编译 | `./waf build` | 0 | — |
| 内置边界自测 | `fbbr_4flow --fbbrHybridSelfTest=true` | 0 | — |
| Smoke | 固定 4-flow / 100M / 5BDP / 60s | 0 | — |
| 完整实验 1 | 固定 4-flow / 100M / 5BDP / 240s | 0 | 0 |
| 完整实验 2 | 固定 32-flow / 100M / 5BDP / 240s | 0 | 0 |
| 完整实验 3 | Cellular Taxi 4-flow / 128M / 5BDP / 180s | 0 | 0 |
| 完整实验 4 | Cellular Taxi 8-flow / 128M / 5BDP / 180s | 0 | 0 |

编译只有既有 `scratch/dqc-test.cc` 未使用函数警告，没有新增编译警告或错误。
四场景 BBR-R/BBRv2 的复用结果各自 `return_code.txt=0`。

Smoke trace 检查结果：

- 无崩溃、无 baseline 字段 NaN；
- 四条流都同时出现 hold 和连续小幅 decrease；
- 日常 OVERLOAD 的单窗口最大 β 为 1.87%–2.75%；
- 没有 quarter-gap/midpoint/mindrate OVERLOAD action；
- smoke 中 baseline 最低 10.279 Mbps，高于 1 Mbps minimum。

## 6. 四场景完整指标

统计口径：吞吐、容量、利用率、队列和 Jain 排除前 5s；Loss、OWD、
Total GiB 使用 whole run。Cellular 容量为 warmup 后 Taxi 动态容量的
时间平均值。标注“复用”的 BBR-R/BBRv2 是相同配置、seed/runId 的既有
有效结果。

| 场景 | 算法 | 吞吐 Mbps | 容量 Mbps | 利用率 | Avg Q ms | P95 Q ms | Jain | Loss % | Avg OWD ms | Total GiB |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 固定 4 | FBBR-hybrid V1 | 97.386 | 100.000 | 97.386% | 11.805 | 21.427 | 0.953042 | 0 | 33.031 | 2.719 |
| 固定 4 | BBR-R（复用） | 97.719 | 100.000 | 97.719% | 5.063 | 14.630 | 0.986803 | 0 | 26.461 | 2.728 |
| 固定 4 | BBRv2（复用） | 98.052 | 100.000 | 98.052% | 181.835 | 209.770 | 0.943277 | 1.727570 | 200.016 | 2.734 |
| 固定 32 | FBBR-hybrid V1 | 97.993 | 100.000 | 97.993% | 85.639 | 129.574 | 0.836427 | 0 | 105.491 | 2.735 |
| 固定 32 | BBR-R（复用） | 97.900 | 100.000 | 97.900% | 15.503 | 31.314 | 0.760189 | 0 | 36.832 | 2.735 |
| 固定 32 | BBRv2（复用） | 98.503 | 100.000 | 98.503% | 199.574 | 209.898 | 0.850406 | 3.796570 | 217.967 | 2.735 |
| Cell 4 | FBBR-hybrid V1 | 42.341 | 43.849 | 96.561% | 70.573 | 372.294 | 0.864459 | 0 | 26.648 | 0.877 |
| Cell 4 | BBR-R（复用） | 41.805 | 43.849 | 95.340% | 64.575 | 231.549 | 0.925854 | 0 | 30.136 | 0.866 |
| Cell 4 | BBRv2（复用） | 43.121 | 43.849 | 98.340% | 741.386 | 4792.732 | 0.842796 | 0.793566 | 174.166 | 0.891 |
| Cell 8 | FBBR-hybrid V1 | 42.789 | 43.849 | 97.584% | 126.014 | 716.208 | 0.776970 | 0 | 32.289 | 0.885 |
| Cell 8 | BBR-R（复用） | 41.955 | 43.849 | 95.681% | 85.242 | 364.943 | 0.844523 | 0 | 31.858 | 0.869 |
| Cell 8 | BBRv2（复用） | 43.190 | 43.849 | 98.498% | 731.845 | 4192.689 | 0.684773 | 0.524525 | 182.410 | 0.891 |

## 7. 相对当前 `/3` FBBR-hybrid 基线

| 场景 | 吞吐 Δ Mbps | 吞吐 Δ | Avg Q Δ ms | Avg Q Δ | P95 Q Δ ms | P95 Q Δ | Jain Δ |
|---|---:|---:|---:|---:|---:|---:|---:|
| 固定 4 | +6.981 | +7.722% | +4.559 | +62.917% | -6.284 | -22.676% | +0.036623 |
| 固定 32 | +0.107 | +0.109% | +65.010 | +315.137% | +95.446 | +279.669% | -0.090185 |
| Cell 4 | +2.333 | +5.830% | -58.784 | -45.443% | -388.023 | -51.034% | +0.017022 |
| Cell 8 | +0.734 | +1.746% | -2.408 | -1.875% | -14.703 | -2.012% | -0.086990 |

四个 FBBR-hybrid V1 场景 Loss 均为 0。

## 8. OVERLOAD hold/decrease/β 汇总

β 由既有 trace 的
`(baseline_before-baseline_after)/baseline_before` 离线恢复；有限精度 trace
重建值按实现中的精确 10% cap 截断。只统计真正进入新日常执行器的 action；
既有 lower-bound acquisition 的 `HYBRID_LOWER_BOUND_SEARCH_REDUCE_0P8`
单独列出。

| 场景 | OVERLOAD | Hold | Decrease | Mean β | P50 β | P95 β | Max β | β≈10% | 既有 0.8 搜索 |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 固定 4 | 1330 | 30 | 1300 | 1.166% | 1.103% | 2.193% | 3.252% | 0 | 57 |
| 固定 32 | 6553 | 21 | 6532 | 5.935% | 5.837% | 10.000% | 10.000% | 616 | 141 |
| Cell 4 | 577 | 16 | 561 | 2.935% | 1.475% | 10.000% | 10.000% | 65 | 44 |
| Cell 8 | 516 | 6 | 510 | 4.069% | 2.712% | 10.000% | 10.000% | 64 | 11 |

四个场景的旧 quarter-gap/midpoint/mindrate OVERLOAD action 计数均为 0。

为了核对“低队列且不增长”行为，另取一个比 `q90<=q_guard` 更严格、可由
现有窗口 trace 直接判定的保守子集：

\[
SRTT_{\max,window}-RTprop \le q_{\mathrm{guard}}
\]

| 场景 | 保守低队列窗口 | Hold | Decrease |
|---|---:|---:|---:|
| 固定 4 | 11 | 11 | 0 |
| 固定 32 | 5 | 5 | 0 |
| Cell 4 | 9 | 9 | 0 |
| Cell 8 | 4 | 4 | 0 |

所有保守低队列窗口都 hold。其余低队列但正增长的窗口按公式允许小幅
decrease，因而不能用“全部 OVERLOAD 的 hold 占比”代替关键条件判断。

## 9. 固定 4-flow 逐流结果

吞吐与 baseline 均排除前 5s；baseline 使用 0.1s previous-sample
时间加权。括号内是当前 `/3` 基线值。

| Flow | 吞吐 Mbps | 吞吐 Δ | Avg baseline Mbps | Avg baseline Δ | P10 baseline Mbps | OVERLOAD | Hold | Decrease | Mean β | 0.8 搜索 |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 1 | 25.861 (27.128) | -1.267 | 28.153 (25.754) | +2.400 | 24.222 (22.329) | 351 | 5 | 346 | 1.189% | 7 |
| 2 | 27.958 (19.891) | +8.066 | 30.425 (18.075) | +12.350 | 23.677 (14.548) | 326 | 7 | 319 | 1.150% | 6 |
| 3 | 23.023 (25.209) | -2.186 | 25.056 (23.599) | +1.457 | 19.231 (19.989) | 336 | 4 | 332 | 1.221% | 17 |
| 4 | 20.544 (18.176) | +2.368 | 21.948 (16.184) | +5.765 | 14.166 (13.144) | 317 | 14 | 303 | 1.097% | 27 |

原弱流 flow2、flow4 的平均 baseline、P10 baseline 和吞吐全部上升；流间
重新分配使 flow1/flow3 吞吐下降，但整体 Jain 反而提高。

## 10. 目标判断

| 判断项 | 结果 | 证据 |
|---|---|---|
| 固定 4 吞吐 >90.405 | 通过 | 97.386 Mbps |
| 固定 4 Jain 不下降 | 通过 | 0.953042 > 0.916419 |
| flow2/flow4 baseline 与吞吐上升 | 通过 | 见逐流表 |
| 固定 4 Avg/P95 队列不明显增加 | **部分失败** | P95 -22.68%，但 Avg +62.92% |
| 固定 32 吞吐保持 97%–98% | 通过 | 97.993 Mbps |
| 固定 32 Avg/P95 队列增加 ≤3% | **失败** | +315.14% / +279.67% |
| Cell 4/8 吞吐下降 ≤2% | 通过 | 两者均上升 |
| Cell 4/8 Avg/P95 队列增加 ≤3% | 通过 | 两者均下降 |
| Cell 4/8 Loss=0 | 通过 | 均为 0 |
| 低队列 OVERLOAD 主要 hold | 通过 | 保守低队列子集 29/29 hold |
| 无 quarter-gap 级日常跳变 | 通过 | 旧 action=0，β≤10% |
| 高/增长队列连续降速 | 通过 | 四场景都有连续 decrease，固定 32 的 P95 β 达 10% |

因此，V1 验证了执行器过度降速假设，但没有通过整体场景安全验收。

## 11. 异常结果的 trace 证据

### 11.1 固定 4-flow：平均队列上升而 P95 下降

旧 `/3` 固定 4-flow trace 中有 919 次离散 OVERLOAD 动作：

- quarter-gap：488 次；
- midpoint：393 次；
- mindrate：38 次；
- 平均单次降幅 15.445%，P50 6.713%，P95 48.568%，最大 66.513%；
- 408 次降幅超过 10%。

V1 固定 4-flow 的日常 OVERLOAD 平均 β 只有 1.166%，最大 3.252%。
瓶颈队列分布从“频繁排空、偶尔高尖峰”变为“持续但较低的非零队列”：

| 版本 | 队列为 0 的采样占比 | P50 Q ms | P95 Q ms |
|---|---:|---:|---:|
| 旧 `/3` | 15.87% | 1.382 | 27.711 |
| V1 | 0.94% | 11.866 | 21.427 |

这解释了平均值上升、P95 反而下降：旧执行器强降后产生较多空队列和吞吐
空洞，V1 消除了多数空洞，但维持了约 12ms 的中位队列。

### 11.2 固定 32-flow：执行器在工作，但聚合队列控制不足

固定 32-flow 有 6553 个日常 OVERLOAD 窗口，其中 6532 个 decrease；
平均 β 5.935%、P95 β 10%，616 个窗口接近 cap。执行器窗口的中位
`q_guard` 为 23.317ms，而中位窗口平均队列为 67.590ms。也就是说异常并非
“全部误 hold”或控制器没有看到高队列。

实际结果表明

\[
y=C(1+g)
\]

以及“本流 baseline 乘法变化能同比映射到聚合输入变化”的近似，在 32 条
独立、异步 FBBR-hybrid 流共同竞争时不够准确。每条流在自己的分类窗口内
响应，pacing baseline 还受到既有探测波形、TrustedBw/native pacing 路径和
相位错位影响；即使多数窗口执行 5%–10% 连续降速，聚合队列仍长期维持高位。
这是 V1 最主要的失败证据。按本阶段约束，没有进一步调 cap、按 flow 数选参
或加入聚合恢复/估计机制。

### 11.3 Cellular

Cell 4 的吞吐、Jain 和两项队列指标同时改善；Cell 8 的吞吐和队列改善，
但 Jain 从 0.863960 降到 0.776970。两场景仍为零丢包。说明 V1 对动态容量
本身没有造成吞吐/队列回归，但公平性仍需在后续设计中关注。

### 11.4 既有 lower-bound 0.8 搜索

trace 中仍能看到既有的
`HYBRID_LOWER_BOUND_SEARCH_REDUCE_0P8`。它是进入日常分类执行器之前的
lower-bound acquisition 路径，不是 quarter-gap/midpoint/mindrate
OVERLOAD actuator。本阶段按“只替换日常 OVERLOAD 执行器、保持
baseline_low 语义”约束没有修改它，统计也没有把 20% 搜索步骤计入 β。

## 12. 是否进入第二阶段

- **执行器过度降速是否是固定 4 吞吐不足的主要原因：是。** 吞吐、Jain、
  flow2/flow4 baseline 与吞吐的同步改善，以及旧离散动作的 48.6% P95
  降幅，共同构成直接证据。
- **现在是否值得进入 queue-neutral baseline recovery：不建议直接进入。**
  固定 4 已达到 97.4% 利用率，主动恢复的收益空间很小；同时固定 4 平均队列
  和固定 32 全部队列指标已经超标。应先理解并修正 V1 在高并发下的聚合
  队列控制偏差，再决定是否需要第二阶段恢复。

本阶段到此停止，没有实现第二阶段。

## 13. 可复现产物

- `NS3.27/results/fbbr_gradient_matched_overload_v1/scenario_metrics.csv`
- `NS3.27/results/fbbr_gradient_matched_overload_v1/overload_summary.csv`
- `NS3.27/results/fbbr_gradient_matched_overload_v1/fixed4_per_flow.csv`
- `NS3.27/results/fbbr_gradient_matched_overload_v1/checks.json`
- 各场景目录内的 `command.txt`、`config.json`、`return_code.txt`、
  `run_meta.json`、waveform classification trace、baseline/throughput/SRTT
  trace和 `compare/summary_metrics.csv`

