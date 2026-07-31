# 当前 FBBR 算法详细说明

## 1. 文档范围

本文说明当前工作树中的 FBBR 实现，而不是论文中的抽象版本或旧的 FreqCC 实现。结论以以下文件为准：

- 核心控制器：[fbbr_sender.h](/home/wkd/FreqBBR/NS3.27/src/dqc/model/thirdparty/congestion/fbbr_sender.h:1) 与 [fbbr_sender.cc](/home/wkd/FreqBBR/NS3.27/src/dqc/model/thirdparty/congestion/fbbr_sender.cc:1)。
- 严格配置解析器：[fbbr_config_loader.cc](/home/wkd/FreqBBR/NS3.27/src/dqc/model/thirdparty/congestion/fbbr_config_loader.cc:1)。
- 当前实验配置：[fbbr_default.conf](/home/wkd/FreqBBR/NS3.27/examples/CCconfig/fbbr_default.conf:1)。
- Test2 的接入点：[test2-fixed4.cc](/home/wkd/FreqBBR/NS3.27/examples/paper-test/test2/test2-fixed4.cc:887)。

FBBR 继承 BBRv2 的主状态机、带宽模型、ProbeRTT、丢包/ECN 响应和原生 PROBE_BW 转换。FBBR 的波形激励、采集、判别和 Regime 更新只在 PROBE_BW 的 PROBE_CRUISE 中运行；其结果通过 CRUISE 基线和离开 CRUISE 时发布的 BEQ 影响后续 pacing。FBBR 不再维护额外的 inflight 服务包络，`GetCongestionWindow()` 直接使用原生 BBRv2 的 `cwnd_`。

当前 Test2 会对每条 FBBR 流调用 `ConfigureFBBR()`，但没有调用 `SetConvergenceGateControlEnabled(true)`。因此本次实验中收敛门控控制默认关闭：稳定性模块仍计算和记录状态，却不会因稳定而停掉 CRUISE 波动。

## 2. 记号与关键状态

| 符号/状态 | 当前实现中的来源 | 含义 |
| --- | --- | --- |
| B | current_injection_baseline_bw_ | 当前 CRUISE 注入基线。进入 CRUISE 时优先取上一轮有效 BEQ，否则取 BBRv2 的 MaxBw。 |
| A | current_probe_amplitude_bps_ | 请求的单边三角波幅度。当前配置为 4sr，即原生发送 pacing rate 的 1/4。 |
| Aeff | effective_probe_amplitude_bps_ | 受最小 pacing 速率保护后的有效幅度。 |
| f | cruise_modulation_freq_hz_ | 注入频率；当前为 5 Hz，周期 T=0.2 s。 |
| F | minimum_pacing_rate_bps_ | pacing 下限；当前 0.2 Mbit/s。 |
| R | model_.MinRtt() | FBBR 的 RTprop 参考值。 |
| M | fbbr_max_srtt_ms_ | 围绕 MaxBw 更新时刻采集的最大 SRTT。每次 MaxBw 更新记录以该时刻为中心、半窗为 3 个 RTT 的观测，成熟后写入 M。 |
| Dmin/Dmax | 窗口中重采样 delivery-rate 的最小/最大值 | Regime III/I 执行器的首选基线候选。 |
| MinBw/MaxBw | BBRv2 model_.MinBandwidth()/MaxBandwidth() | BBRv2 带宽模型给出的边界，用于 Regime I/III 中点候选和原生回退。 |
| BEQ | beq_ | FBBR 在一轮 CRUISE 结束时发布的带宽基线；不是显式的公平份额。 |
| CRUISE 时间加权 BEQ | delivery-rate/SRTT 历史 | 波形 BEQ 不可用时，优先使用 SRTT 位于 `[1.05R, 1.10R]` 的 delivery-rate 样本；没有符合样本时使用全 CRUISE 样本。 |
| v_round | abs(D_round - D_prev) / D_prev | 连续两个非 app-limited RTT 轮次的最大 delivery-rate 相对变化。 |

重要约束：fair_share_bandwidth_bps_ 只被写入 trace 汇总，不参与 Regime 判别、BEQ、pacing 或 inflight 决策。因此 FBBR 不依赖也不假设网络提供显式公平份额。

## 3. 总体执行链

~~~text
BBRv2 STARTUP / DRAIN / PROBE_BW
                 |
                 v
          进入 PROBE_CRUISE
                 |
                 +-- 选 B：上一轮 BEQ 或原生 MaxBw
                 +-- 计算 A 与 Aeff，建立 0 -> -1 -> 0 -> +1 -> 0 三角波
                 +-- 可选 MaxBw-flat 安全试验；若执行则暂时禁用波形
                 |
                 v
          等待 1 个 SRTT settle
                 |
                 v
          收集 2 个周期的 sender-rate、delivery-rate、SRTT
                 |
                 +-- 重采样、插值、有效性检查、Goertzel、水平段/肩部/重复裁剪检测
                 |
                 v
          Regime 判定：I(UNDERLOAD) / II(FULL_LOAD) / III(OVERLOAD)
                 |
                 +-- I：提高 B
                 +-- II：首次保持；本轮已见 I/III 时区间交付率无效，仍保持 B
                 +-- III：降低 B
                 |
                 v
          不确定：一次延长观察；之后可放大信号并重试
                 |
                 v
          离开 CRUISE：波形 BEQ -> SRTT 区间时间加权 BEQ
                         -> 全 CRUISE 时间加权 BEQ -> 原生带宽回退
                 |
                 v
          REFILL / UP / DOWN 使用新鲜 BEQ；GetCongestionWindow 直接返回原生 cwnd
~~~

CRUISE 的原生退出在活动波形周期中会延迟到下一个周期边界，以避免截断 0 -> -1 -> 0 -> +1 -> 0 的激励。发生丢包或 recovery 时，安全退出不等待该边界。

## 4. CRUISE 注入、pacing 与收敛门控

### 4.1 三角波

在 CRUISE 活动窗口内：

~~~text
Pacing(t) = max(F, B + Aeff * triangle(t))

q = ((t - probe_epoch_start) mod T) / T

0 <= q < 0.25:  triangle(q) = -4q
0.25 <= q < 0.75: triangle(q) = 4q - 2
0.75 <= q < 1:    triangle(q) = 4 - 4q
~~~

因此一个周期的相位顺序为 0 -> -1 -> 0 -> +1 -> 0，而不是从正半波开始。波形公式见 [TriangleWave](/home/wkd/FreqBBR/NS3.27/src/dqc/model/thirdparty/congestion/fbbr_sender.cc:2034)。

当前 A 的语义：

- default_amplitude_mode=4sr：A = Bbr2Sender::PacingRate(0) / 4。
- Nsr 支持 N=1..20，20sr 的含义是 SR/20，不是 20*SR。
- fixed_mbps、数值 Mbps 和旧 Miu 模式仍可解析，但当前 4sr 下不生效。
- 若 B <= F，Aeff=0；否则 Aeff=min(A, B-F)。这保证负半波不会把 pacing 压到 F 以下。
- PROBE_CRUISE 的 cwnd gain 固定为 1.1；由上一轮 RTprop 刷新触发的 PROBE_DOWN pacing gain 固定为 0.75。二者不是配置文件参数。

### 4.2 收敛门控

每个 RTT 轮次从非 app-limited 的 sample_max_bandwidth 取最大值得到 D_round。若有连续两轮：

~~~text
v_round = abs(D_round - D_prev) / D_prev
~~~

稳定态退出条件：

~~~text
v_round > single_round_exit_threshold
or
(v_round > consecutive_exit_threshold and prev_v_round > consecutive_exit_threshold)
~~~

退出稳定后，full_drate_ref 设为当前 D_round。之后若完成轮次达到 full_drate_ref * full_pipe_growth_threshold，则更新参考并重新计数；否则累积 stable_rounds 次轮次后回到稳定态。

当前 Test2 不开启 enable_convergence_gate_control，所以 stable/unstable 不会决定是否产生三角波。若外部显式打开控制，stable 时会关闭调制并清除 BEQ 应用；unstable 时才允许调制。这套逻辑位于 [FinalizeCompletedRound](/home/wkd/FreqBBR/NS3.27/src/dqc/model/thirdparty/congestion/fbbr_sender.cc:2442) 和 [UpdateFreqWeightAndToolState](/home/wkd/FreqBBR/NS3.27/src/dqc/model/thirdparty/congestion/fbbr_sender.cc:2553)。

## 5. 信号采集与波形特征模块

### 5.1 输入、时间对齐和有效性

FBBR 保存三条历史：

| 历史 | 采集时机 | 用途 |
| --- | --- | --- |
| sender_rate_history_ | 每次发送 | 实际命令 pacing rate。 |
| delivery_rate_history_ | 有效 ACK 后 | 当前 Test2 选择 DeliveryRateLatest；记录 ACK 字节数和 app-limited 标记。 |
| srtt_history_ | 有效 ACK 后 | smoothed RTT，单位 ms。 |

窗口长度由当前频率决定。初始窗口为 2 个周期，先等待一个 SRTT；FBBR 重采样步长为 clamp(T/40, 1 ms, 5 ms)。sender-rate 窗口向前偏移一个 probe epoch RTT，以与接收侧 delivery-rate/SRTT 对齐。

delivery-rate 输入有效的最低条件：

- 至少 4 个 delivery-rate 样本，确认字节数大于 0。
- 重采样覆盖率至少为 waveform.min_cycle_coverage_ratio。
- app-limited 样本比例不超过 waveform.max_app_limited_sample_ratio。

SRTT 输入有效的最低条件：

- 至少 4 个 SRTT 样本。
- 重采样覆盖率至少为 waveform.min_cycle_coverage_ratio。

最大插值空洞为 waveform.max_interpolation_gap_period_ratio*T；超过该空洞的点无效。FBBR 对窗口 SRTT 均值另做时间加权计算，避免 ACK 密度改变均值。

### 5.2 Goertzel 目标频率检测

FBBR 已知自己注入的频率 f，不通过 FFT 搜索主频。它在 sender-rate 和 delivery-rate 的重采样序列上直接运行 generalized Goertzel：

~~~text
omega = 2*pi*f*sample_step
s[n] = (x[n] - mean) + 2*cos(omega)*s[n-1] - s[n-2]

coherent_power_ratio = |X(f)|^2 / (N * sum((x[n]-mean)^2))
component_present = coherent_power_ratio >= goertzel.min_coherent_power_ratio
~~~

在 FBBR 路径中，sender-rate 与 delivery-rate 的目标频率分量直接用 Goertzel 判定，不再走旧的 sender/delivery-rate 周期相似度分支。delivery-rate 波还要求 delivery-rate 输入有效，且 sender 与 delivery-rate 的目标频率分量都存在。SRTT 的波存在条件由时域活动检测得到，不以 Goertzel 相似度替代。实现见 [AnalyzeGoertzelComponent](/home/wkd/FreqBBR/NS3.27/src/dqc/model/thirdparty/congestion/fbbr_sender.cc:5056)。

### 5.3 时域活动、裁剪和掩码

SRTT 以及 delivery-rate 的活动/裁剪预处理仍可使用以下时域模块；但 FBBR 对 sender-rate/delivery-rate 的最终目标频率匹配使用 Goertzel，SRTT 仍使用这些时域证据：

- 活动检测：p95-p05 幅度需同时超过噪声倍数和相对电平阈值；还检查活动步数、活动比例、上下方向变化、有效路径比例和斜率反转。
- 连续水平段：检测顶部/底部水平段、平坦比例、局部斜率、两侧斜率、边界折点、电平跨度、总漂移和极值距离。
- 肩部：以水平段两侧极值和周期残差验证正/负肩部。
- 重复裁剪：要求两个周期内都有接触点，验证接触样本数、周期一致性、平台平坦度、边界、外侧偏离和缺失间隔。
- 中部序列扰动：识别不符合两侧趋势的中间段，将其从周期性检验掩掉；每周期掩码比例受限。
- 周期性：SRTT 使用实际周期估计、相关系数和周期误差；验证过的上裁剪会触发硬否决，不把它误判为同频响应。

三个 SRTT 裁剪方向证据的优先级为：

| 代码 | 证据 | 含义 |
| --- | --- | --- |
| U1 | positive shoulder | 顶部正肩部。 |
| U2 | long top line | 顶部持续水平线。 |
| U3 | repeated top clip | 两周期重复顶部裁剪。 |
| L1 | negative shoulder | 底部负肩部。 |
| L2 | long bottom line | 底部持续水平线。 |
| L3 | repeated bottom clip | 两周期重复底部裁剪。 |

只有 SRTT 本身存在普通波活动时，这些裁剪证据才会进入 Regime 分支；否则会退回无裁剪决策树。这样孤立的水平线不会被误作为裁剪。

## 6. Regime I/II/III 的完整判定树

分类映射固定如下：

| 分类 | FBBR Regime | 后续行为 |
| --- | --- | --- |
| UNDERLOAD | I | 提高 B。 |
| FULL_LOAD | II | 首次仅保持；同一 CRUISE 已见 I/III 后用区间交付率更新 B 与 BEQ。 |
| OVERLOAD | III | 降低 B。 |
| INCONCLUSIVE | 无 | 保持 B，扩窗或重试。 |

判别函数为 [ClassifyFbbrRegime](/home/wkd/FreqBBR/NS3.27/src/dqc/model/thirdparty/congestion/fbbr_sender.cc:3541)。以下表中，D 匹配/不匹配指 delivery-rate 对注入频率的 Goertzel 结果；SRTT 波指时域活动结果。

### 6.1 进入判定树前的条件

1. 若 SRTT 无普通波活动，任何 U1..L3 裁剪证据都不作为裁剪分支输入。
2. 若某分支要求 D 的周期性但 D 输入无效，结果为 INCONCLUSIVE。
3. M 为 MaxBw 更新附近的最大 SRTT；R 为 FBBR RTprop。
4. MaxExceeded 的条件是 SRTTmax > M；MinBelowRtprop 的条件是 SRTTmin < R。

### 6.2 裁剪优先分支 N01--N09

| 规则 | 前提 | 分类 | 刷新 R |
| --- | --- | --- | --- |
| N01 | U1 且 D 匹配 | FULL_LOAD / II | 否 |
| N02 | U1 且 D 不匹配 | OVERLOAD / III | 否 |
| N03 | U2 且 D 匹配 | FULL_LOAD / II | 否 |
| N04 | U2 且 D 不匹配 | OVERLOAD / III | 否 |
| N05 | U3 | OVERLOAD / III | 否 |
| N06 | L1 | FULL_LOAD / II | 否 |
| N07 | L2 且 D 有波 | UNDERLOAD / I | 是 |
| N08 | L2 且 D 无波 | FULL_LOAD / II | 否 |
| N09 | L3 | UNDERLOAD / I | 否 |

U1/U2 在 D 输入无效时不会猜测分类，而是 INCONCLUSIVE。U3、L1、L3 不需要 D 的周期性结果。

### 6.3 无裁剪分支 N10--N16

若没有有效裁剪分支，先检查 SRTT 输入：

| 规则 | 前提 | 分类 | 刷新 R |
| --- | --- | --- | --- |
| 无规则 | SRTT 输入无效 | INCONCLUSIVE | 否 |
| N10 | SRTT 有波且 SRTTmax > M | OVERLOAD / III | 否 |
| N11 | SRTT 有波且 SRTTmin < R | UNDERLOAD / I | 是 |
| N12 | SRTT 有波，且未命中 N10/N11，使用回退阈值 | 见下 | 否 |
| 无规则 | SRTT 无波且 D 输入无效 | INCONCLUSIVE | 否 |
| N13 | SRTT 无波且 D 匹配 | UNDERLOAD / I | 否 |
| N14 | SRTT 无波、D 不匹配且 SRTTmax > M | OVERLOAD / III | 否 |
| N15 | SRTT 无波、D 不匹配且 SRTTmin < R | UNDERLOAD / I | 是 |
| N16 | SRTT 无波、D 不匹配，且未命中 N14/N15，使用回退阈值 | 见下 | 否 |

N12/N16 的回退阈值只在 SRTT 统计、M 和 R 都有效，且 M>=R>0 时成立：

~~~text
overload_threshold = max(1.10*R, R + (M-R)/3)

SRTTmax > overload_threshold  => OVERLOAD
SRTTmax < 1.05*R              => UNDERLOAD
otherwise                      => FULL_LOAD
~~~

N07、N11、N15 才把当前窗口的 SRTTmin 发布为新的 FBBR RTprop。L3/N09 虽为 UNDERLOAD，但不刷新 RTprop。

## 7. 三个 Regime 的执行器

执行器为 [ComputeFbbrInjectionBaseline](/home/wkd/FreqBBR/NS3.27/src/dqc/model/thirdparty/congestion/fbbr_sender.cc:3691)。分类有效且 Dmin、Dmax、最小速率均有效后，规则如下。

### 7.1 Regime I：UNDERLOAD

令当前基线为 B。

1. 若 MaxBw 有效且 Dmax > B 且 Dmax < MaxBw，设置 Bnext=Dmax。
2. 否则，若 MaxBw 有效、Dmax < MaxBw，且 midpoint=(Dmax+MaxBw)/2 大于 B，设置 Bnext=midpoint。
3. 否则，设置 Bnext=1.02*B。
4. 最终 Bnext=max(F, Bnext)。

这三个分支在 trace 中分别标为：

~~~text
FBBR_REGIME_I_USE_MAX_DRATE
FBBR_REGIME_I_USE_MAXBW_MAXDRATE_MIDPOINT
FBBR_REGIME_I_GROW_BASELINE_1P02
~~~

### 7.2 Regime III：OVERLOAD

1. 若 MinBw 有效且 Dmin < B 且 Dmin > MinBw，设置 Bnext=Dmin。
2. 否则，若 MinBw 有效、Dmin > MinBw，且 midpoint=MinBw+(Dmin-MinBw)/2 小于 B，设置 Bnext=midpoint。
3. 否则，设置 Bnext=0.98*B。
4. 最终 Bnext=max(F, Bnext)。

对应 trace：

~~~text
FBBR_REGIME_III_USE_MIN_DRATE
FBBR_REGIME_III_USE_MINBW_MINDRATE_MIDPOINT
FBBR_REGIME_III_DECREASE_BASELINE_0P98
~~~

### 7.3 Regime II：FULL_LOAD

- 若本轮 CRUISE 尚未出现 Regime I 或 III：保持 B，不发布新的 BEQ。
- 若本轮已出现 I 或 III：当前实现将 `interval_delivery_rate_bps` 固定为 `0.0`；由于 inflight 包络已移除，无法计算区间交付率，直接执行 `FBBR_REGIME_II_INVALID_DELIVERED_INTERVAL_HOLD`。
- 因此当前 FBBR 的 Regime II 不更新 B 或 BEQ；`FBBR_WINDOW_MEAN`/`FBBR_WINDOW_DELIVERED_RATE` 候选路径不可达。

FullLoad 不使用区间交付率的估计值替代；无论是首次 FullLoad 还是区间速率无效，都会保持基线并重新 settle。

## 8. 不确定结果、扩窗和幅度重试

若分类为 INCONCLUSIVE：

1. 首次不确定可扩展观察窗口；当前 FBBR 的滚动重试最大使用 3 个周期，并在扩展后分析较后的 2 周期窗口。
2. 若连续两个窗口中 SRTT 或 delivery-rate 无波，启动 wave-fidelity enhancement，抑制本次分类与状态更新。
3. 第二次不确定后可以提高激励幅度：优先使用 waveform.delta_drate_amplitude_ratio*观测 delivery-rate 响应幅度；无可用响应时使用 waveform.delta_fallback_baseline_ratio*B。
4. 放大倍数为 waveform.inconclusive_signal_amplification_factor，且不超过初始幅度的 waveform.inconclusive_signal_amplification_max_ratio，同时仍经过 Aeff 和 F 的保护。
5. 任意分类/执行器/统计输入无效时，不改变 B，重置 settle 后继续采集。

## 9. BEQ 与 CRUISE 时间加权回退

### 9.1 BEQ 选择优先级

离开 CRUISE 时，当前 FBBR 按以下顺序发布 BEQ：

1. 若当前 CRUISE 产生了合法的波形 BEQ：使用该 BEQ。
2. 否则，选取当前 CRUISE 中对应 SRTT 位于 `[1.05R, 1.10R]` 的有效 delivery-rate 样本，按样本保持时间计算时间加权平均值，来源为 `CRUISE_SRTT_1P05_1P10_TIME_WEIGHTED`。
3. 若没有符合 SRTT 区间的样本，使用当前 CRUISE 全部有效 delivery-rate 样本的时间加权平均值，来源为 `CRUISE_ALL_TIME_WEIGHTED`。
4. 若整个 CRUISE 没有有效 delivery-rate 样本：依次回退到 BBRv2 MaxBw、初始 CRUISE 基线、BandwidthEstimate、最小 pacing rate，来源为 `NATIVE_FALLBACK`。

上一轮有效 BEQ 仍可作为下一轮 CRUISE 的初始注入基线，但不会在当前 CRUISE 的最终 BEQ 发布中替代当前 CRUISE 样本。

选择出的 BEQ 只在 PROBE_REFILL、PROBE_UP、PROBE_DOWN/PROBE_DOWN_SLIGHTLY 中以新鲜且有效的结果参与 pacing。正常活动波形的 CRUISE 使用当前注入基线 B；若波形被禁用，则继续使用原生或当前轮已发布的 BEQ。

### 9.2 时间加权回退的精确定义

时间加权回退只使用当前 FBBR CRUISE 内的 delivery-rate 历史样本。每个有效样本从其时间戳保持到下一个 delivery-rate 样本或 CRUISE 结束：

~~~text
Beq = sum(delivery_rate_i * duration_i) / sum(duration_i)
~~~

样本要求：

- delivery-rate 样本自身有效且带宽为正。
- MaxBw-flat trial 的 ACK 被显式排除。
- SRTT 区间优先路径要求对应时刻的 SRTT 满足 `1.05R <= SRTT <= 1.10R`，两端点均包含。

delivery-rate 与 SRTT 按 ACK 事件时间关联；SRTT 使用不晚于该 delivery-rate 样本时间的最新有效值。优先路径没有可积分的符合样本时，改用同一 CRUISE 内全部有效 delivery-rate 样本。

## 10. Inflight 服务包络已移除

旧的 inflight 服务包络（plan_inflight / service_inflight / positive_probe_credit / envelope / inflight_cap）及其专属历史、快照和 telemetry 控制已从 FBBR 中移除。`GetCongestionWindow()` 直接返回原生 BBRv2 的 `cwnd_`，不再施加额外的 in-flight 上限；旧的 `FBBR_FLOW_SUMMARY` 诊断路径也不再产生。`FinalizeFbbrTrace()` 仍保留为 no-op，以兼容外部调用。

这不表示所有 `ComputeFbbr*` 方法都已删除：当前仍保留 `ComputeFbbrMaxSrttAround`、`ComputeFbbrTimeWeightedSrttMeanMs` 和 `ComputeFbbrInjectionBaseline`，分别服务于 MaxBw 附近 SRTT 观测、SRTT 时间加权均值和 Regime 执行器。区间交付率 helper 已移除，`ApplyFbbrClassification()` 将 Regime II 的 interval 速率固定为 `0.0`，因此 FullLoad 在需要区间速率时只能保持基线。

## 11. MaxBw-flat 安全试验

FBBR 还实现了非配置化的 MaxBw-flat 试验。当当前是 FBBR 的 PROBE_CRUISE、非 recovery，且：

~~~text
MaxBw > initial_cruise_baseline
MaxBw 和 initial_cruise_baseline 均为有效正带宽
~~~

它暂停三角波、以 MaxBw 平速发送。app-limited 和 RTT 条件在试验开始后检查；下列任一条件会拒绝试验：

- 丢包、ECN、recovery 或 app-limited。
- SRTT>1.05*RTprop，或 SRTT>entry_SRTT+0.02*RTprop。
- 某 RTT 轮次 delivery rate<0.97*trial_rate。
- 某轮最大 SRTT>1.03*RTprop，或 >entry_SRTT+0.01*RTprop。

连续 3 个合格 RTT 轮次后接受，将 B 提升到试验 MaxBw；否则恢复原 B。试验 ACK 不进入 CRUISE 时间加权回退、收敛轮次、波形分类或 Regime 更新。实现见 [MaxBw-flat trial](/home/wkd/FreqBBR/NS3.27/src/dqc/model/thirdparty/congestion/fbbr_sender.cc:1308)。

## 12. 当前配置参数

配置加载器是严格模式：未知 key、格式错误和非法 amplitude mode 都会导致加载失败。数值进入控制器后会按各参数的范围再做 clamp 或回退。所有参数的完整原文在附录 A。

### 12.1 注入、pacing 与运行时控制

| 参数 | 当前值 | 作用与当前有效性 |
| --- | --- | --- |
| default_modulation_freq_hz | 5.0 | 默认 CRUISE 注入频率；当前周期 0.2 s。 |
| default_amplitude_mode | 4sr | 当前有效。A=原生发送 pacing rate/4。支持 1sr..20sr；也接受旧 srN 别名。 |
| default_fixed_amplitude_mbps | 10.0 | 仅 fixed_mbps 模式使用；当前不生效。 |
| flow.0..3.modulation_freq_hz | 均为 5.0 | 每流覆盖默认频率；当前四流均为 5 Hz。 |
| flow.0..3.fixed_amplitude_mbps | 均为 50.0 | 每流 fixed 幅度；当前 4sr 模式下不生效。 |
| pacing.minimum_rate_mbps | 0.2 | 负半波 pacing 下限 F，并决定 Aeff 的最大可用幅度。 |

### 12.2 稳定性、BEQ 与窗口收集

| 参数 | 当前值 | 作用 |
| --- | --- | --- |
| stability.single_round_exit_threshold | 0.25 | 单轮 v_round 超过此值时退出稳定态。门控控制关闭时只影响状态/trace。 |
| stability.consecutive_exit_threshold | 0.15 | 连续两轮 v_round 均超过此值时退出稳定态。 |
| stability.stable_rounds | 3 | 非稳定态下重新稳定所需合格 RTT 轮次数。 |
| stability.full_pipe_growth_threshold | 1.25 | 完成轮次达到参考 delivery rate 的此倍数时更新 full_drate_ref 并重新计数。 |
| beq.clear_on_cruise_start | true | 当前 true。源码中 false 会被提示为 overridden，CRUISE 开始仍清除 BEQ 应用状态；不要把 false 当作保留应用的开关。 |
| waveform.initial_window_periods | 2.0 | 初始采集周期数。 |
| waveform.extended_window_periods | 3.0 | 配置时为扩展/最大窗口的下限和默认值；当前 FBBR 滚动重试实际以 min(3.0,max_window_periods) 扩至 3 周期。 |
| waveform.max_window_periods | 3.0 | 窗口长度上限，也决定 FBBR 历史保留的波形时间尺度。 |
| waveform.min_cycle_coverage_ratio | 0.85 | 原始重采样序列最小覆盖率。 |
| waveform.masked_min_cycle_coverage_ratio | 0.50 | 经过水平段/中部扰动掩码后的最小覆盖率。 |
| waveform.max_app_limited_sample_ratio | 0.25 | delivery-rate 窗口允许的最大 app-limited 样本比例。 |
| waveform.max_interpolation_gap_period_ratio | 0.10 | 允许插值的最大连续空洞，占一个注入周期的比例。 |

### 12.3 周期性、裁剪与重试

| 参数 | 当前值 | 作用 |
| --- | --- | --- |
| goertzel.min_coherent_power_ratio | 0.10 | sender 与 delivery-rate 在 f 处判定目标分量存在的下限。 |
| waveform.period_tolerance_ratio | 0.15 | 通用实际周期估计允许的相对误差。 |
| waveform.min_periodicity_correlation | 0.50 | 通用周期相关性下限。 |
| waveform.local_slope_window_period_ratio | 0.05 | 局部斜率窗口的周期比例。 |
| waveform.min_local_slope_window_ms | 5.0 | 局部斜率窗口的最小绝对时长。 |
| waveform.clip_min_duration_ratio | 0.15 | 水平/裁剪候选的最短持续比例。 |
| waveform.clip_min_half_overlap_ratio | 0.75 | 裁剪候选与相应半周期的最小覆盖。 |
| waveform.clip_max_slope_ratio | 0.10 | 裁剪内部允许的最大归一化斜率。 |
| waveform.plateau_max_level_span_ratio | 0.15 | 平台内部允许的最大电平跨度比例。 |
| waveform.plateau_extreme_distance_ratio | 0.15 | 平台到两侧极值需要满足的距离比例。 |
| waveform.delta_drate_amplitude_ratio | 0.50 | 不确定重试时，根据交付率响应幅度生成激励基数的比例。 |
| waveform.delta_fallback_baseline_ratio | 0.25 | 无交付率响应时，不确定重试激励基数占 B 的比例。 |
| waveform.max_baseline_adjustments | 8 | 可进行的基线调整次数上限。 |
| waveform.inconclusive_signal_amplification_factor | 1.25 | 重试时每次放大激励的倍数。 |
| waveform.inconclusive_signal_amplification_max_ratio | 2.0 | 相对初始幅度的最大放大倍数。 |

### 12.4 Regime 专用参数

| 参数 | 当前值 | 作用 |
| --- | --- | --- |
| fbbr.regime.long_top_horizontal_duration_ratio | 0.20 | U2 顶部长期水平线的最小周期占比。 |
| fbbr.regime.long_bottom_horizontal_duration_ratio | 0.30 | L2 底部长期水平线的最小周期占比。 |
| fbbr.regime.period_tolerance_ratio | 0.20 | FBBR SRTT 周期性匹配允许的相对周期误差。 |
| fbbr.regime.min_periodicity_correlation | 0.50 | FBBR SRTT 周期性匹配所需的最低相关性。 |

上裁剪的周期性硬否决、无波连续窗口阈值 2、滚动重试前移周期数 1，以及 Regime I/III 的 1.02/0.98 步长均为当前源码硬编码值，不在配置文件中。

### 12.5 波形活动检测

| 参数 | 当前值 | 作用 |
| --- | --- | --- |
| waveform.activity.amplitude_noise_multiplier | 6.0 | p95-p05 幅度相对噪声 sigma 的最低倍数。 |
| waveform.activity.min_level_ratio | 0.02 | 幅度相对信号电平的最低比例。 |
| waveform.activity.step_noise_multiplier | 3.0 | 单步显著变化相对噪声的门限倍数。 |
| waveform.activity.min_normalized_step_slope | 3.5 | 归一化显著步斜率门限。 |
| waveform.activity.min_active_steps | 4 | 最少活动步数。 |
| waveform.activity.min_active_step_ratio | 0.10 | 最少活动步比例。 |
| waveform.activity.min_directional_change_ratio | 0.20 | 上/下方向变化所需比例。 |
| waveform.activity.min_significant_path_ratio | 0.80 | 显著变化路径最小比例。 |
| waveform.activity.min_slope_reversals | 1 | 最少斜率反转次数。 |

### 12.6 连续水平段检测

| 参数 | 当前值 | 作用 |
| --- | --- | --- |
| waveform.horizontal.continuous_min_duration_ratio | 0.15 | 连续水平段最短周期比例。 |
| waveform.horizontal.min_valid_coverage_ratio | 0.85 | 水平段检测所需的有效样本覆盖率。 |
| waveform.horizontal.min_flat_fraction | 0.90 | 水平段内部的最小平坦步比例。 |
| waveform.horizontal.max_local_slope_ratio | 0.10 | 平坦段最大局部斜率比例。 |
| waveform.horizontal.min_side_slope_ratio | 0.25 | 两侧变化所需最小斜率比例。 |
| waveform.horizontal.min_boundary_kink_ratio | 0.25 | 平台边界折点所需的最小斜率差比例。 |
| waveform.horizontal.max_level_span_ratio | 0.10 | 水平段内允许的最大电平跨度。 |
| waveform.horizontal.max_total_drift_ratio | 0.05 | 水平段总漂移上限。 |
| waveform.horizontal.min_side_change_ratio | 0.10 | 平台外侧最小电平变化比例。 |
| waveform.horizontal.amplitude_noise_multiplier | 6.0 | 候选幅度相对噪声的最低倍数。 |
| waveform.horizontal.level_span_noise_multiplier | 4.0 | 电平跨度容差的噪声倍数。 |
| waveform.horizontal.slope_noise_multiplier | 3.0 | 斜率相关门限的噪声倍数。 |
| waveform.horizontal.extreme_distance_ratio | 0.10 | 平台到极值的最小距离比例。 |

### 12.7 重复裁剪检测

| 参数 | 当前值 | 作用 |
| --- | --- | --- |
| waveform.repeated_clip_max_period_error_ratio | 0.15 | 两周期裁剪接触位置允许的周期误差。 |
| waveform.repeated_clip_max_level_delta_ratio | 0.05 | 两周期接触电平允许的最大差异比例。 |
| waveform.repeated_clip_contact_level_tolerance_ratio | 0.05 | 判为同一裁剪接触电平的容差。 |
| waveform.repeated_clip_min_contact_samples_per_cycle | 2 | 每周期最少接触样本。 |
| waveform.repeated_clip_min_total_contact_samples | 4 | 全窗口最少接触样本。 |
| waveform.repeated_clip_min_contact_sample_ratio | 0.05 | 接触样本相对有效样本的最低比例。 |
| waveform.repeated_clip_min_pooled_flat_fraction | 0.90 | 汇总接触段内最小平坦比例。 |
| waveform.repeated_clip_min_verified_boundary_fraction | 0.75 | 可观察边界中必须验证为裁剪边界的比例。 |
| waveform.repeated_clip_min_outside_excursion_ratio | 0.10 | 平台外侧必须出现的最小越界比例。 |
| waveform.repeated_clip_min_extrapolated_overshoot_ratio | 0.05 | 外推越界的最低比例。 |
| waveform.repeated_clip_merge_gap_ratio | 0.025 | 合并相邻接触段允许的间隔比例。 |
| waveform.repeated_clip_max_missing_gap_ratio | 0.05 | 接触段中允许的最大连续缺样比例。 |

### 12.8 肩部与中部序列扰动

| 参数 | 当前值 | 作用 |
| --- | --- | --- |
| waveform.shoulder.min_half_overlap_ratio | 0.75 | 肩部覆盖相应半周期的最低比例。 |
| waveform.shoulder.min_side_change_ratio | 0.15 | 肩部两侧最小电平变化比例。 |
| waveform.shoulder.max_residual_cycle_period_error_ratio | 0.20 | 肩部两侧极值周期残差上限。 |
| waveform.shoulder.min_residual_cycle_leg_duration_ratio | 0.15 | 肩部残余周期腿的最短比例。 |
| waveform.middle.min_duration_ratio | 0.05 | 中部扰动最短持续比例。 |
| waveform.middle.max_duration_ratio | 0.35 | 中部扰动最长持续比例。 |
| waveform.middle.context_duration_ratio | 0.10 | 用于比较两侧趋势的上下文比例。 |
| waveform.middle.min_trend_slope_ratio | 0.20 | 扰动段内部趋势的最小斜率比例。 |
| waveform.middle.max_context_slope_delta_ratio | 0.75 | 两侧上下文斜率的最大允许差异。 |
| waveform.middle.min_slope_mismatch_ratio | 0.50 | 扰动段与上下文的最小斜率不匹配比例。 |
| waveform.middle.min_mismatching_sample_ratio | 0.25 | 至少需不匹配的样本比例。 |
| waveform.middle.min_mismatching_samples | 2 | 最少不匹配样本数。 |
| waveform.middle.min_consecutive_mismatching_samples | 2 | 最少连续不匹配样本数。 |
| waveform.middle.min_bridge_deviation_ratio | 0.05 | 相对两端连线的最低偏离比例。 |
| waveform.middle.noise_multiplier | 3.0 | 中部扰动阈值的噪声倍数。 |
| waveform.middle.max_mask_ratio_per_cycle | 0.35 | 每周期允许被中部掩码覆盖的最大比例。 |

### 12.9 trace 参数及其 Test2 例外

| 参数 | 配置文件值 | 当前 Test2 接线 |
| --- | --- | --- |
| trace.gate_trace_mode | round_only | `ConfigureFBBR()` 不读取；Test2 未调用 `SetGateTraceMode()`，实际保留构造默认 `round_only`。 |
| trace.gate_trace_sample_interval_us | 10000 | `ConfigureFBBR()` 不读取；实际保留构造默认 1 ms，而不是配置文件的 10 ms。 |
| trace.enable_cruise_window_trace | true | 配置解析器接受，但 `ConfigureFBBR()` 不应用该字段。 |
| trace.enable_beq_selection_trace | true | 配置解析器接受，但 `ConfigureFBBR()` 不应用该字段。 |

这些 trace 字段由配置解析器接受，但 `ConfigureFBBR()` 没有读取它们。当前 Test2 只调用 `ConfigureFBBR()`，没有调用 `SetConvergenceGateTraceEnabled()`、`SetConvergenceGateControlEnabled()` 或 `SetGateTraceMode()`；因此实际 FBBR 默认值为 gate trace enabled=true、control=false、mode=round_only、sample interval=1 ms。两个 `trace.enable_*` 配置项本身不会改变运行时状态。

## 13. 观测与调试

建议从以下信息诊断算法行为：

| 现象 | 重点字段/事件 |
| --- | --- |
| 某轮为何进入 I/II/III | Waveform trace 中 decision_rule=N01..N16、selected clip case、SRTT/DRate 有波、M、R、Dmin/Dmax。 |
| 基线为何变化 | waveform_last_action、delta_source、raw/applied baseline delta、I/III 子分支标记。 |
| BEQ 为何不同 | `beq_source` 表示波形 BEQ、`CRUISE_SRTT_1P05_1P10_TIME_WEIGHTED`、`CRUISE_ALL_TIME_WEIGHTED` 或 `NATIVE_FALLBACK`。 |
| 波形为何没有分类 | invalid_reason、coverage、app_limited_ratio、Goertzel component reason、无波 retry 状态。 |
| 低队列是否由 cwnd 包络造成 | cwnd 包络已移除；当前直接反映 BBRv2 原生 cwnd。 |
| MaxBw 是否被验证 | MaxBw-flat trial 的启动、拒绝原因、good_rounds、three_good_rounds。 |

## 附录 A：当前生效配置全文

以下是本说明对应的 [fbbr_default.conf](/home/wkd/FreqBBR/NS3.27/examples/CCconfig/fbbr_default.conf:1) 原文。除上节明确说明的 runner 接线例外外，参数值以此文件为准。

~~~ini
# FBBR active configuration (key = value)
#
# This file contains only knobs that participate in the current FBBR control
# path. Retired or unknown keys are rejected by the loader.

# CRUISE triangular modulation.
default_modulation_freq_hz = 5.0

# Nsr means current sending rate divided by N. N must be an integer in [1, 20]:
# 1sr = SR / 1, 4sr = SR / 4, and 20sr = SR / 20.
# fixed_mbps instead uses default_fixed_amplitude_mbps below.
default_amplitude_mode = 4sr
default_fixed_amplitude_mbps = 10.0

# Per-flow overrides use zero-based IDs consistently across all experiments.
# The fixed-amplitude values apply only when default_amplitude_mode=fixed_mbps;
# Nsr modes use SR / N instead.
flow.0.modulation_freq_hz = 5.0
flow.1.modulation_freq_hz = 5.0
flow.2.modulation_freq_hz = 5.0
flow.3.modulation_freq_hz = 5.0
flow.0.fixed_amplitude_mbps = 50.0
flow.1.fixed_amplitude_mbps = 50.0
flow.2.fixed_amplitude_mbps = 50.0
flow.3.fixed_amplitude_mbps = 50.0

# The single-sided probe amplitude is capped so pacing cannot fall below F.
pacing.minimum_rate_mbps = 0.2

# Convergence gate.
stability.single_round_exit_threshold = 0.25
stability.consecutive_exit_threshold = 0.15
stability.stable_rounds = 3
stability.full_pipe_growth_threshold = 1.25
beq.clear_on_cruise_start = true

# Time-waveform collection and periodicity tests.
waveform.initial_window_periods = 2.0
waveform.extended_window_periods = 3.0
waveform.max_window_periods = 3.0
waveform.period_tolerance_ratio = 0.15
waveform.min_periodicity_correlation = 0.50
waveform.min_cycle_coverage_ratio = 0.85
waveform.masked_min_cycle_coverage_ratio = 0.50
goertzel.min_coherent_power_ratio = 0.10

# Local slope, clipping, and baseline evidence.
waveform.local_slope_window_period_ratio = 0.05
waveform.min_local_slope_window_ms = 5.0
waveform.clip_min_duration_ratio = 0.15
waveform.clip_min_half_overlap_ratio = 0.75
waveform.clip_max_slope_ratio = 0.10
waveform.plateau_max_level_span_ratio = 0.15
waveform.plateau_extreme_distance_ratio = 0.15
waveform.delta_drate_amplitude_ratio = 0.50
waveform.delta_fallback_baseline_ratio = 0.25
waveform.max_baseline_adjustments = 8
waveform.inconclusive_signal_amplification_factor = 1.25
waveform.inconclusive_signal_amplification_max_ratio = 2.0
waveform.max_app_limited_sample_ratio = 0.25
waveform.max_interpolation_gap_period_ratio = 0.10

# FBBR Regime I/II/III classifier.
fbbr.regime.long_top_horizontal_duration_ratio = 0.20
fbbr.regime.long_bottom_horizontal_duration_ratio = 0.30
fbbr.regime.period_tolerance_ratio = 0.20
fbbr.regime.min_periodicity_correlation = 0.50

# Delivery-rate waveform activity.
waveform.activity.amplitude_noise_multiplier = 6.0
waveform.activity.min_level_ratio = 0.02
waveform.activity.step_noise_multiplier = 3.0
waveform.activity.min_normalized_step_slope = 3.5
waveform.activity.min_active_steps = 4
waveform.activity.min_active_step_ratio = 0.10
waveform.activity.min_directional_change_ratio = 0.20
waveform.activity.min_significant_path_ratio = 0.80
waveform.activity.min_slope_reversals = 1

# Horizontal-segment and clipping checks.
waveform.horizontal.continuous_min_duration_ratio = 0.15
waveform.horizontal.min_valid_coverage_ratio = 0.85
waveform.horizontal.min_flat_fraction = 0.90
waveform.horizontal.max_local_slope_ratio = 0.10
waveform.horizontal.min_side_slope_ratio = 0.25
waveform.horizontal.min_boundary_kink_ratio = 0.25
waveform.horizontal.max_level_span_ratio = 0.10
waveform.horizontal.max_total_drift_ratio = 0.05
waveform.horizontal.min_side_change_ratio = 0.10
waveform.horizontal.amplitude_noise_multiplier = 6.0
waveform.horizontal.level_span_noise_multiplier = 4.0
waveform.horizontal.slope_noise_multiplier = 3.0
waveform.horizontal.extreme_distance_ratio = 0.10

# Repeated clipping checks.
waveform.repeated_clip_max_period_error_ratio = 0.15
waveform.repeated_clip_max_level_delta_ratio = 0.05
waveform.repeated_clip_contact_level_tolerance_ratio = 0.05
waveform.repeated_clip_min_contact_samples_per_cycle = 2
waveform.repeated_clip_min_total_contact_samples = 4
waveform.repeated_clip_min_contact_sample_ratio = 0.05
waveform.repeated_clip_min_pooled_flat_fraction = 0.90
waveform.repeated_clip_min_verified_boundary_fraction = 0.75
waveform.repeated_clip_min_outside_excursion_ratio = 0.10
waveform.repeated_clip_min_extrapolated_overshoot_ratio = 0.05
waveform.repeated_clip_merge_gap_ratio = 0.025
waveform.repeated_clip_max_missing_gap_ratio = 0.05

# Shoulder and middle-plateau checks.
waveform.shoulder.min_half_overlap_ratio = 0.75
waveform.shoulder.min_side_change_ratio = 0.15
waveform.shoulder.max_residual_cycle_period_error_ratio = 0.20
waveform.shoulder.min_residual_cycle_leg_duration_ratio = 0.15

waveform.middle.min_duration_ratio = 0.05
waveform.middle.max_duration_ratio = 0.35
waveform.middle.context_duration_ratio = 0.10
waveform.middle.min_trend_slope_ratio = 0.20
waveform.middle.max_context_slope_delta_ratio = 0.75
waveform.middle.min_slope_mismatch_ratio = 0.50
waveform.middle.min_mismatching_sample_ratio = 0.25
waveform.middle.min_mismatching_samples = 2
waveform.middle.min_consecutive_mismatching_samples = 2
waveform.middle.min_bridge_deviation_ratio = 0.05
waveform.middle.noise_multiplier = 3.0
waveform.middle.max_mask_ratio_per_cycle = 0.35

# Optional FBBR diagnostic traces.
trace.gate_trace_mode = round_only
trace.gate_trace_sample_interval_us = 10000
trace.enable_cruise_window_trace = true
trace.enable_beq_selection_trace = true
~~~
