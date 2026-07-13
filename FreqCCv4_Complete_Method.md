# FreqCCv4 当前完整做法

> 依据：2026-07-13 工作区现行代码，而不是历史设计稿。
>
> 主实现：`NS3.27/src/dqc/model/thirdparty/congestion/freqccv4_sender.{h,cc}`
>
> 当前配置类型和入口仍名为 `FreqBbrConfig`、`ConfigureFreqBbr()`；本文保留代码中的真实命名。

## 1. 一句话定义

FreqCCv4 以当前仓库的 Native BBRv2 为主体，在 `PROBE_BW::PROBE_CRUISE` 阶段给原生 pacing 叠加固定频率三角波，观察发送速率、Delivery Rate 和 SRTT 对该频率的响应，通过 Delivery Rate/SRTT 双信号频谱完整性门控选出一个可信窗口，将窗口内的平均 Delivery Rate 发布为 `TrustedBw`，并且只在紧随其后的 `REFILL`、`UP`、`DOWN` 阶段用它替换 pacing 的带宽基线。

核心边界如下：

- Native BBRv2 继续维护 MaxBw、BandwidthEstimate、min RTT、BDP、cwnd、`inflight_hi` 和 `inflight_lo`。
- `TrustedBw` 是 pacing-only 辅助量，不写回 Native BBRv2 带宽模型。
- CRUISE 的调制基线始终是 Native BBRv2 pacing，不使用旧 `TrustedBw`。
- `TrustedBw` 只允许由本轮刚结束的 CRUISE 产生，并在下一轮 CRUISE 开始时关闭应用窗口。
- Delivery Rate 和 SRTT 必须分别通过自己的硬门限；联合分数严格取两者最小值，不存在单信号候选路径。
- 当前活跃评分不使用 fair-share、队列长度、ECN 或 loss 对 `TrustedBw` 做裁剪。

## 2. 代码组成

| 层次 | 当前文件 | 作用 |
|---|---|---|
| 算法主体 | `NS3.27/src/dqc/model/thirdparty/congestion/freqccv4_sender.{h,cc}` | 调制、采样、FFT、窗口评分、双信号门控、TrustedBw 生命周期和 pacing |
| BBRv2 父类 | `quic_bbr2_sender.{h,cc}`、`quic_bbr2_probe_bw.{h,cc}` | Native BBRv2 模型、PROBE_BW 子状态、phase gain、cwnd/inflight |
| 算法注册 | `proto_send_algorithm_interface.cc`、`proto_types.h` | `kFreqCCv4` 到 `FreqCCv4Sender` 的构造映射 |
| DQC 接入 | `NS3.27/src/dqc/model/dqc_sender.{h,cc}` | 配置转换、trace callback、运行时参数下发 |
| Trace 落盘 | `NS3.27/src/dqc/model/dqc_trace.{h,cc}` | CRUISE 窗口、CRUISE 汇总和 gate/pacing CSV |
| 默认配置 | `NS3.27/examples/CCconfig/freqccv4_default.conf` | 4 流默认频率、幅度、门限和 trace 设置 |
| 主场景 | `NS3.27/scratch/freqccv4_4flow.cc` | 四流 dumbbell、动态 RTT、CLI、自测试入口 |
| 通用场景 | `NS3.27/scratch/generic_p2p_switch_flows.cc` | 多算法/多流通用实验，额外支持 CRUISE baseline cap |
| 构建 | `NS3.27/src/dqc/wscript`、`NS3.27/wscript` | 编译 FreqCCv4 并链接 FFTW3 |

### 2.1 关键函数索引

| 环节 | 函数 |
|---|---|
| phase 切换 | `OnProbeBwPhaseEntered()` |
| CRUISE 生命周期 | `EnterCruise()`、`LeaveCruise()`、`FinalizeCruise()` |
| 每包 pacing | `PacingRate()`、`TriangleWave()`、`GetCurrentAmplitudeBps()` |
| 发送/接收采样 | `OnPacketSent()`、`OnCongestionEvent()` |
| RTT 收敛观测 | `OnCongestionEventStarted()`、`FinalizeCompletedRound()` |
| 稳定态更新 | `CheckExitStable()`、`UpdateReconvergenceEvidence()`、`UpdateFreqWeightAndToolState()` |
| 滑动窗口 | `RunDueCruiseWindowAnalysis()`、`BuildCruiseWindowResult()` |
| 重采样和 FFT | `ResampleRateSeries()`、`ResampleRttSeries()`、`BuildSpectrumProfile()` |
| 相位/周期质量 | `ComputePhaseCoherence()`、`AnalyzeCycleQuality()` |
| TrustedBw 选择 | `RunTrustedBwSelection()`、`PublishTrustedBwSelection()` |
| TrustedBw 失效 | `ClearTrustedBw()`、`ClearTrustedBwApplication()` |
| Trace | `EmitCruiseWindowTrace()`、`EmitCruiseSummaryTrace()`、`EmitFreqGateCsvRow()` |

## 3. 总体执行链路

```text
Native BBRv2 进入 PROBE_CRUISE
        |
        v
EnterCruise()
  - cruise_id++
  - 关闭上一轮 TrustedBw 的应用/新鲜状态
  - 锁定本轮调制频率
  - 清空本轮窗口结果
  - 第一个窗口从 cruise_start + min_rtt 开始
        |
        v
每次发包: PacingRate()
  - 取 Native BBR pacing
  - 满足开关条件时叠加固定频率三角波
  - 记录实际发送 pacing 到 sender_rate_history
        |
        v
每次 ACK: OnCongestionEvent()
  - 记录 Delivery Rate、SRTT、ACK/loss 历史
  - 完成到期的滑动 CRUISE 窗口分析
        |
        v
每个 RTT 结束: FinalizeCompletedRound()
  - 计算 D_round 和 v_round
  - 更新可选的收敛门控状态
        |
        v
离开 PROBE_CRUISE: FinalizeCruise()
  - 补齐到期 NORMAL 窗口
  - NORMAL 选择失败时可做一次 MERGED rescue
  - 双信号门控通过后发布 TrustedBw
  - 输出窗口和本轮汇总 trace
        |
        v
REFILL / UP / DOWN
  - 新鲜 TrustedBw 有效: phase_gain * TrustedBw
  - 否则: Native BBR pacing
        |
        v
进入下一轮 CRUISE
  - 旧 TrustedBw 不再可用于控制
```

## 4. BBRv2 状态与 FreqCCv4 行为

FreqCCv4 继承 `Bbr2Sender`，不另造一套主状态机。与本算法直接相关的 `PROBE_BW` 循环是：

```text
DOWN -> CRUISE -> REFILL -> UP -> DOWN -> ...
```

代码还支持 `PROBE_DOWN_SLIGHTLY`，FreqCCv4 将它按 `DOWN` 处理。

| BBR 阶段 | 调制 | TrustedBw pacing 基线 | 其他控制 |
|---|---:|---:|---|
| STARTUP | 否 | 否 | 完全使用 Native BBRv2 |
| DRAIN | 否 | 否 | 完全使用 Native BBRv2 |
| PROBE_BW / CRUISE | 是，满足开关条件时 | 否 | Native pacing 加三角波 |
| PROBE_BW / REFILL | 否 | 是，新鲜且有效时 | phase gain 仍来自 Native BBRv2 |
| PROBE_BW / UP | 否 | 是，新鲜且有效时 | phase gain、退出条件、inflight 仍来自 Native BBRv2 |
| PROBE_BW / DOWN | 否 | 是，新鲜且有效时 | phase gain、排空逻辑仍来自 Native BBRv2 |
| PROBE_BW / DOWN_SLIGHTLY | 否 | 是，新鲜且有效时 | 按 DOWN 应用 |
| PROBE_RTT | 否 | 否 | 清空 TrustedBw，完全使用 Native BBRv2 |

只要离开 `PROBE_BW`，`OnCongestionEvent()` 就会执行 `ClearTrustedBw("non_probe_bw")`。

## 5. CRUISE 固定频率三角调制

### 5.1 开启条件

基础开启条件 `BaseShouldOscillate()` 同时要求：

1. 当前幅度大于 0；
2. 配置频率大于 0；
3. 已完成 DRAIN 并进入 `PROBE_BW`；
4. 当前 phase 为 `PROBE_CRUISE`；
5. `in_cruise_` 为真且 `cruise_start_time_` 有效。

若 `enableConvergenceGateControl=false`，满足上述条件就调制。若该开关为真，还要求 `bbr_stable_ == false`。

当前默认配置下：

- `enableConvergenceGateControl` 默认是 `false`，所以收敛状态机只观测，不关闭调制；
- 频率为 5 Hz；
- 幅度模式为 `4miu`，即 Native `BandwidthEstimate()/4`。

### 5.2 幅度模式

设 Native BBRv2 带宽估计为 `B_native`，Native BBRv2 当前 pacing 为 `P_native`，调制幅度为 `A`：

| 配置字符串 | 枚举 | 幅度 |
|---|---|---:|
| `2miu` / `miu2` | `kMiu2` | `B_native / 2` |
| `3miu` / `miu3` | `kMiu3` | `B_native / 3` |
| `4miu` / `miu4` | `kMiu4` | `B_native / 4` |
| `8miu` / `miu8` | `kMiu8` | `B_native / 8` |
| `2sr` / `sr2` | `kSR2` | `P_native / 2` |
| `3sr` / `sr3` | `kSR3` | `P_native / 3` |
| `4sr` / `sr4` | `kSR4` | `P_native / 4` |
| `8sr` / `sr8` | `kSR8` | `P_native / 8` |
| `12sr` / `sr12` | `kSR12` | `P_native / 12` |
| `16sr` / `sr16` | `kSR16` | `P_native / 16` |
| `fixed_mbps` 或无法识别的非数字字符串 | `kFixed` | `fixed_amplitude_mbps` |
| 可解析的正数字符串 | `kFixed` | 该数字对应的 Mbps |

### 5.3 三角波

对本轮 CRUISE 开始后的时间 `t`，频率为 `f`，归一化相位为：

```text
q = (t mod (1/f)) / (1/f),  q in [0, 1)
```

三角波 `T(q)` 为：

```text
0.00 <= q < 0.25: T(q) = 4q
0.25 <= q < 0.75: T(q) = 2 - 4q
0.75 <= q < 1.00: T(q) = 4q - 4
```

所以一个周期依次为 `0 -> +1 -> 0 -> -1 -> 0`，调制偏移是：

```text
offset_bps = A * T(q)
```

### 5.4 Pacing 公式

CRUISE 中：

```text
final_pacing = max(1000 bps, NativeBbrPacing + A * T(q))
```

`TrustedBw` 可应用的 REFILL/UP/DOWN 中：

```text
final_pacing = phase_pacing_gain * TrustedBw
```

如果 `TrustedBw` 不满足新鲜度、有效位、phase 或收敛门控条件：

```text
final_pacing = NativeBbrPacing
```

通用场景可以额外设置 `cruise_baseline_cap_bps_`，它只在 CRUISE 调制路径中限制调制前 baseline；四流主场景当前没有设置该 cap。

## 6. 信号采集

### 6.1 四条历史

| 历史 | 采集点 | 当前内容 | 活跃用途 |
|---|---|---|---|
| `sender_rate_history_` | `OnPacketSent()` | `PacingRate()` 实际返回值 | 与 drate 做频率和谱形对照 |
| `delivery_rate_history_` | 有新 ACK 的 `OnCongestionEvent()` | 默认 `BandwidthLatest()`；可切换到 `DeliveryRateLatest()` | TrustedBw 数值和 Delivery Rate 频谱 |
| `srtt_history_` | 有新 ACK 的 `OnCongestionEvent()` | smoothed RTT，单位 ms | SRTT 频谱、周期波形和相位一致性 |
| `ack_window_history_` | 有新 ACK 的 `OnCongestionEvent()` | acked bytes 和是否有 loss | 当前选择链路未使用，仅供尚未接入的拥塞分数函数 |

每个 deque 最多保存 20000 条，超出后删除最早样本。

### 6.2 Delivery Rate 的默认口径

`ConfigureFreqCC()` 的 `recv_signal_mode` 默认是 `bandwidth_latest`，所以主场景默认把 `BandwidthLatest()` 写入 `delivery_rate_history_`。只有显式选择下列别名时才使用 `DeliveryRateLatest()`：

```text
delivery_rate_latest / delivery_latest / raw_delivery / recvrate_raw
```

四流主场景当前没有暴露 `recv_signal_mode` CLI，因此默认使用 `BandwidthLatest()`。

### 6.3 RTT 级收敛样本

每个 RTT 内，算法从有效、非 app-limited、非零且非 recovery 的 `sample_max_bandwidth` 中取最大值：

```text
D_round = max(valid sample_max_bandwidth in this RTT)
```

当前仓库的 `InRecovery()` 仍是 BBRv2 stub，但 FreqCCv4 已保留 recovery 过滤条件。

## 7. CRUISE 滑动窗口

### 7.1 起点、长度和步进

进入 CRUISE 后，第一个窗口不是立刻开始，而是从：

```text
window_start_0 = cruise_start + min_rtt
```

这样会跳过 CRUISE 刚切入时至少一个 min RTT 的过渡段。

设载波频率为 `f_ref`，默认最少周期数为 4，窗口长度为：

```text
window_duration = max(4 / f_ref, 2 * min_rtt)
```

窗口步进为：

```text
window_step = max(1 ms, 0.25 * window_duration)
```

默认 5 Hz 时，频率项给出 0.8 s 窗口，默认步进是 0.2 s。

### 7.2 sender/receiver 时间对齐

对接收侧窗口 `[window_start, window_end]`：

- Delivery Rate 使用 `[window_start, window_end]`；
- SRTT 使用 `[window_start, window_end]`；
- sender rate 使用 `[window_start-min_rtt, window_end-min_rtt]`。

即发送侧探针向前平移一个 min RTT，近似对齐它在接收侧造成的响应。

### 7.3 输入覆盖硬条件

Delivery Rate 和 SRTT 分别检查：

- 至少 4 个原始样本；
- 时间戳严格递增；
- rate 必须非零，RTT 必须为有限正数；
- 完整窗口至少覆盖 2 个载波周期；
- 首末原始样本之间也至少覆盖 2 个周期；
- 原始样本覆盖时长 / 窗口时长至少为 0.75。

任一信号覆盖失败，该信号无效，双信号门控必然失败。

### 7.4 重采样

三条信号都线性插值到固定 1 ms 步长，即采样率 1000 Hz。重采样结果少于 8 点时无效。

- sender rate：不做线性 detrend；
- Delivery Rate：不做线性 detrend；
- SRTT：先减去首尾点连成的线性趋势；
- 三条信号进入 FFT 前都会再次减均值并乘 Hann 窗。

因此载波必须低于 500 Hz，且当前实现主要面向远低于 Nyquist 的低频探针。

## 8. FFT 与频谱特征

当前实现使用 FFTW3 的实数到复数 FFT：

1. 信号减均值；
2. 乘 Hann 窗；
3. 末尾补零到原长度的 4 倍；
4. 计算单边幅度谱；
5. 只在 `[0.7*f_ref, 1.3*f_ref]` 内找载波峰值。

每条信号得到 `SpectrumProfile`：

| 字段 | 含义 |
|---|---|
| `peak_freq_hz` | 目标频带最大幅度所在频率，带三点抛物线插值 |
| `target_amp` | 目标频带内最大幅度 |
| `noise_floor` | 目标频带之外所有幅度的中位数 |
| `band_energy_rel` | 目标频带幅度和 / 全部非 DC 幅度和 |
| `band_peak_rel` | 目标频带最大幅度 / 全部非 DC 幅度和 |
| `peak_width_hz` | 半峰宽，并至少取 `max(freq_step, 1/window_duration)` |
| `freq_step_hz` | 补零后 FFT bin 间隔 |
| `band_shape` | 目标频带内 16 点插值、归一化后的谱形 |

这里的“能量”实现上使用幅度和，不是幅度平方和。

## 9. 双信号频谱完整性

Delivery Rate 和 SRTT 各自计算五个质量因子。

### 9.1 五个质量因子

频率尺度：

```text
sigma_f = max(freq_sigma_ratio * f_ref, 0.5 / window_duration)
```

频偏质量：

```text
q_freq = exp(-0.5 * (abs(peak_freq-f_ref)/sigma_f)^2)
```

SNR 质量：

```text
q_snr = logistic(snr, min_snr, snr_slope)
snr = target_amp / max(noise_floor, epsilon)
```

目标频带集中度：

```text
q_energy = logistic(band_energy_rel, energy_threshold, energy_slope)
```

峰宽质量：

```text
width_ratio = peak_width_hz / max(freq_step_hz, 1/window_duration)
q_width = exp(-0.5 * (max(0, width_ratio-r0)/width_sigma)^2)
```

相位一致性：

- 按 `f_ref` 把重采样信号切成完整周期；
- 每周期减去周期均值并投影到载波复相量；
- 用所有周期复向量和的模 / 各周期模之和作为 `q_phase`；
- 少于 2 个有效周期时相位无效，质量记为 0。

### 9.2 单信号完整性分数

五个质量因子取几何平均：

```text
Q_drate = (q_freq * q_snr * q_energy * q_width * q_phase)^(1/5)
Q_srtt  = (q_freq * q_snr * q_energy * q_width * q_phase)^(1/5)
```

联合分数严格取较小者：

```text
Q_joint = min(Q_drate, Q_srtt)
```

`limiting_spectral_signal` 记录当前瓶颈是 `DRATE`、`SRTT` 还是 `EQUAL`。

### 9.3 双信号硬门控

Delivery Rate 必须同时满足：

```text
输入覆盖有效
频谱有效且噪底有效
相位计算有效
drate_snr >= spectral.min_drate_snr
drate_width_ratio <= spectral.max_drate_width_ratio
drate_phase_coherence >= spectral.min_drate_phase_coherence
Q_drate >= spectral.drate_integrity_threshold
```

SRTT 使用同构的独立门限。最终：

```text
dual_signal_spectral_gate_pass =
    drate_spectral_gate_pass && srtt_spectral_gate_pass
```

以下任一情况都不能产生候选：缺样本、覆盖不足、时间戳异常、数值异常、噪底失败、SNR 太低、峰太宽、相位不一致、完整性分数不足或任一信号缺失。

## 10. 满载候选和窗口质量

### 10.1 载波频率匹配

候选频率容差为：

```text
freq_tolerance = max(0.20 * f_ref, 2 / window_duration)
freq_score = clamp01(1 - abs(peak_freq-f_ref)/freq_tolerance)
freq_quality = min(drate_freq_score, srtt_freq_score)
```

满载候选要求：

```text
dual_signal_spectral_gate_pass
drate_freq_score >= 0.60
srtt_freq_score >= 0.60
drate_mean_kbps 为有限正数
```

这里的“满载”含义是 Delivery Rate 和 SRTT 都对发送端载波表现出可信的同频响应，不是直接读取队列、loss 或 ECN 得出的标签。

### 10.2 幅度质量

Delivery Rate 增益：

```text
drate_gain = drate_target_amp / max(srate_target_amp, epsilon)
drate_amplitude_score = clamp01(drate_gain / 0.30)
```

SRTT 幅度质量把 SRTT SNR 从最小门限线性映射到目标 SNR=3.0；噪底估计失败时旧的幅度子分会回退到 0.5，但双信号硬门控仍会失败，所以该窗口不会成为 TrustedBw 候选。

```text
amplitude_quality =
    0.4 * drate_amplitude_score +
    0.6 * srtt_amplitude_score
```

### 10.3 波形质量

Delivery Rate 波形质量比较 sender rate 与 Delivery Rate 的 16 点归一化频带谱形：

```text
shape_distance = sum(abs(drate_band_shape - srate_band_shape))
drate_waveform_quality = clamp01(1 - shape_distance/0.40)
```

SRTT 波形质量按周期统计：顶部削顶、底部削顶、周期是否同时包含上升和下降、前后半周期变化量是否不对称，以及峰值相位是否稳定。

```text
distortion =
    0.25 * top_clip_ratio +
    0.25 * bottom_clip_ratio +
    0.20 * cycle_incompleteness +
    0.15 * cycle_asymmetry +
    0.15 * phase_instability

srtt_waveform_quality = 1 - distortion
waveform_quality =
    0.4 * drate_waveform_quality +
    0.6 * srtt_waveform_quality
```

### 10.4 周期一致性与综合分

```text
consistency_quality =
    0.5 * cycle_frequency_stability +
    0.5 * cycle_phase_stability

full_load_quality_v1 = base_quality =
    0.35 * freq_quality +
    0.25 * waveform_quality +
    0.20 * amplitude_quality +
    0.20 * consistency_quality

full_load_quality_v2 =
    dual_signal_gate_pass ? Q_joint * base_quality : 0
```

### 10.5 low confidence 与可靠窗口

窗口被标记 `low_confidence` 的条件包括：双信号门控失败、sender rate 无效或频率分数小于 0.60、Delivery Rate/SRTT 原始样本少于 4、预期或有效周期少于 2，以及 `base_quality < 0.50`。

真正可用于选择的可靠窗口还必须满足：

```text
is_full_load_candidate == true
low_confidence == false
full_load_quality_v2 >= 0.50
drate_mean_kbps > 0
```

`0.50` 这一可靠窗口门限当前是类内固定值，不在配置文件中。

## 11. TrustedBw 选择

### 11.1 NORMAL 优先

离开 CRUISE 时，算法先在所有 `window_source=NORMAL` 的可靠窗口中，按 `full_load_quality_v2` 降序选择最佳窗口。

如果本轮 CRUISE 实际没有开启频率工具，或者开启 control 后当前已经稳定，则直接返回空选择。

### 11.2 MERGED rescue

当没有 NORMAL 可靠窗口，且 rescue 开启时：

1. 以每个 NORMAL 窗口的起点为起点；
2. 把窗口时长乘 `merged_rescue.window_multiplier`，默认 2.0；
3. 末端不能超过当前 CRUISE 结束时间；
4. 重新执行完整的采样、FFT、双信号门控和质量评分；
5. 比较前半窗/后半窗 Delivery Rate 中位数差：

```text
trend_ratio = abs(median(first_half)-median(second_half)) /
              median(all_samples)
```

6. `trend_ratio` 必须不超过 0.20；
7. 再从可靠 MERGED 窗口中按 V2 质量选最佳。

MERGED 不会绕过双信号门控。当前 `max_merged_passes_` 在配置应用时被强制限制到最多 1 次。

### 11.3 数值、来源与置信度

选中窗口后：

```text
TrustedBw = best_window.drate_mean_kbps * 1000
```

即 TrustedBw 是窗口内 1 ms 重采样 Delivery Rate 的平均值，不是 Native MaxBw、fair-share 或频率分数本身。

来源：

```text
NORMAL_SPECTRAL
MERGED_SPECTRAL
NONE
```

置信度：

```text
NORMAL: trusted_bw_conf = full_load_quality_v2
MERGED: trusted_bw_conf = 0.8 * full_load_quality_v2
```

当前 pacing 只检查 TrustedBw 的有效位和新鲜度，不进一步按 `trusted_bw_conf` 缩放或加权。

## 12. TrustedBw 生命周期

### 12.1 发布

`PublishTrustedBwSelection()` 只有在 selection 标记有效、双信号门控通过、TrustedBw 非零且有限时才发布。发布后：

```text
trusted_bw_valid = true
trusted_bw_fresh = true
trusted_bw_application_valid = true
trusted_bw_ready_for_post_cruise = true
trusted_bw_cruise_id = current cruise_id
application_phase = POST_CRUISE_READY
```

### 12.2 使用

使用 TrustedBw 还要求：

```text
phase in {REFILL, UP, DOWN, DOWN_SLIGHTLY}
trusted_bw_application_valid
trusted_bw_ready_for_post_cruise
trusted_bw_fresh
trusted_bw_cruise_id == current cruise_id
trusted_bw_valid && trusted_bw > 0
control 未开启，或当前 bbr_stable == false
```

### 12.3 失效

以下事件会完整清零 TrustedBw 及其应用状态：

- 配置发生应用：`configuration_changed`；
- 离开 `PROBE_BW`：`non_probe_bw`；
- control 开启并重新收敛：`stable_closure`；
- 本轮没有得到双信号可信选择：`no_dual_signal_trusted_bw`。

进入新 CRUISE 时执行的是 `ClearTrustedBwApplication("cruise_start")`：底层数值可能暂时保留用于 trace，但 `fresh/application/ready` 都会变为 false，因此绝不会在新 CRUISE 或其后误用旧结果。

配置项 `trusted_bw.clear_on_cruise_start=false` 当前会被明确覆盖；实现始终关闭旧应用窗口，以保证 fresh-only 语义。

## 13. 收敛门控

### 13.1 稳定态退出

每轮计算：

```text
v_round = abs(D_round-D_prev) / D_prev
```

初始 `bbr_stable_ = true`。出现任一情况时退出稳定态：

```text
v_round > 0.25
或
当前 v_round > 0.15 且上一轮 v_round > 0.15
```

退出后：

```text
bbr_stable = false
stable_cnt = 0
full_drate_ref = D_round
freq_tool_needed = true
w_freq = 1
unstable_episode_id++
```

### 13.2 重新收敛

不稳定态下：

- 如果 `D_round >= 1.25 * full_drate_ref`，更新参考带宽并把 `stable_cnt` 清零；
- 否则每个有效 RTT 把 `stable_cnt` 加一；
- 连续 3 个有效 RTT 没有出现上述大增长，就重新进入稳定态并清空 TrustedBw。

观测权重：

```text
w_freq = 1 - stable_cnt / stable_rounds
```

当前 `w_freq_` 只进入 trace，不参与调制幅度、TrustedBw 或 pacing 的连续加权；控制效果是全开/全关。

### 13.3 trace-only 与 control

这两个开关必须区分：

| 开关 | 作用 |
|---|---|
| `enableConvergenceGateTrace` | 打开 gate CSV 和相关诊断 |
| `enableConvergenceGateControl` | 稳定时关闭 CRUISE 调制并禁止 TrustedBw 应用 |

默认 control 关闭。因此默认算法行为是持续在每个 CRUISE 进行测量；收敛状态只记录，不改变控制。

## 14. 不会被 TrustedBw 改写的 Native 状态

当前实现没有把 TrustedBw 写入以下任一状态：

- BBRv2 MaxBw filter；
- `BandwidthEstimate()`；
- `BandwidthLatest()` / `DeliveryRateLatest()`；
- BDP 计算；
- cwnd；
- `inflight_hi` / `inflight_lo`；
- STARTUP full-pipe 检测；
- PROBE_UP/PROBE_DOWN 的退出条件；
- ACK 聚合、loss 或 ECN 模型。

因此 FreqCCv4 的控制面改动可以概括为：

```text
CRUISE: Native pacing 上加测量载波
POST-CRUISE: 在指定 phase 临时替换 pacing bandwidth baseline
其他所有模型状态: Native BBRv2
```

## 15. 当前默认配置

文件：`NS3.27/examples/CCconfig/freqccv4_default.conf`

当前存在三层默认值，不能混为一谈：

| 层次 | 频率 | 幅度模式/幅度 |
|---|---:|---|
| 裸 `FreqCCv4Sender` 构造器 | 5 Hz | fixed，0 bps |
| `FreqBbrConfig` 和四流场景内置值 | 5 Hz | `fixed_mbps`，50 Mbps |
| 检入的默认配置文件 | 5 Hz | `4miu` |

正常运行四流场景时会成功加载检入配置文件，所以实际默认是第三行；只有配置文件缺失或不加载时才退回第二行。

```ini
default_modulation_freq_hz = 5.0
default_amplitude_mode = 4miu
default_fixed_amplitude_mbps = 10.0

flow.0.modulation_freq_hz = 5.0
flow.1.modulation_freq_hz = 5.0
flow.2.modulation_freq_hz = 5.0
flow.3.modulation_freq_hz = 5.0
flow.0.fixed_amplitude_mbps = 50.0
flow.1.fixed_amplitude_mbps = 50.0
flow.2.fixed_amplitude_mbps = 50.0
flow.3.fixed_amplitude_mbps = 50.0

stability.single_round_exit_threshold = 0.25
stability.consecutive_exit_threshold = 0.15
stability.stable_rounds = 3
stability.full_pipe_growth_threshold = 1.25

spectral.drate_integrity_threshold = 0.25
spectral.srtt_integrity_threshold = 0.25
spectral.min_drate_snr = 1.5
spectral.min_srtt_snr = 1.5
spectral.max_drate_width_ratio = 2.0
spectral.max_srtt_width_ratio = 2.5
spectral.min_drate_phase_coherence = 0.5
spectral.min_srtt_phase_coherence = 0.5
spectral.freq_sigma_ratio = 0.08
spectral.snr_slope = 2.0
spectral.energy_threshold = 0.10
spectral.energy_slope = 20.0
spectral.width_r0_drate = 1.5
spectral.width_r0_srtt = 2.0
spectral.width_sigma = 0.8

merged_rescue.enable = true
merged_rescue.window_multiplier = 2.0
merged_rescue.max_passes = 1
merged_rescue.max_trend_ratio = 0.20
merged_rescue.confidence_discount = 0.8

trusted_bw.clear_on_cruise_start = true

trace.gate_trace_mode = round_only
trace.gate_trace_sample_interval_us = 10000
trace.enable_cruise_window_trace = true
trace.enable_trusted_bw_selection_trace = true
```

因为默认幅度模式是 `4miu`，四个 `fixed_amplitude_mbps=50` 当前不会决定实际幅度；只有切换到 fixed 模式时才生效。

## 16. 配置加载与覆盖顺序

四流场景的顺序是：

1. 预扫描 `--freqBbrConfig`；
2. 加载 `key=value` 文件；
3. 把配置文件默认值复制到场景全局变量；
4. 解析全部 CLI；
5. 再把 CLI 后的全局值写回 `FreqBbrConfig`；
6. 每条流依次调用 `ConfigureFreqBbr(config, flow_id)` 和 `ConfigureFreqCC(...)`。

所以 CLI 优先级高于配置文件。默认路径当前是绝对路径：

```text
/home/wkd/BBR_ICC/NS3.27/examples/CCconfig/freqccv4_default.conf
```

迁移仓库位置时应显式传入：

```bash
--freqBbrConfig=/absolute/path/to/freqccv4_default.conf
```

解析失败或未知 key 会打印 warning；配置文件无法打开时使用代码内置默认值继续运行。

## 17. 可配置参数与当前固定参数

### 17.1 配置文件可调

- 默认和逐流频率；
- 默认幅度模式和 fixed Mbps；
- 稳定态退出/重新收敛门限；
- Delivery Rate/SRTT 完整性、SNR、峰宽、相位门限；
- 频偏、SNR、能量、峰宽软评分参数；
- MERGED rescue 参数；
- trace 模式和采样间隔。

### 17.2 当前类内固定

| 参数 | 当前值 |
|---|---:|
| 重采样步长 | 1 ms |
| 窗口最少载波周期 | 4 |
| 窗口步进比例 | 0.25 |
| 候选 drate 频率分数 | 0.60 |
| 候选 SRTT 频率分数 | 0.60 |
| 可靠窗口 V2 门限 | 0.50 |
| 频率搜索带 | `[0.7*f_ref, 1.3*f_ref]` |
| 频带谱形点数 | 16 |
| FFT 补零倍数 | 4 |
| 目标 Delivery Rate gain | 0.30 |
| 目标 SRTT SNR | 3.0 |
| 最大谱形距离标尺 | 0.40 |
| 历史最大样本数 | 20000 |

`SetCruiseWindowConfig()` 虽然存在，但当前场景没有调用它。

## 18. 当前接口中的实际注意事项

这些是理解“现在做法”时必须保留的代码事实：

1. 配置类型仍叫 `FreqBbrConfig`，不是 `FreqCCv4Config`。
2. `ConfigureFreqBbr()` 每次应用配置都会清空旧 TrustedBw。
3. `trusted_bw.clear_on_cruise_start=false` 当前不会改变 fresh-only 行为。
4. `merged_rescue.max_passes` 即使配置大于 1，也会被截断为 1。
5. `interval_win_rtt_mult` CLI 会调用 `SetFreqCCIntervalWindowMultiplier()`，但该 DQC 接口当前只处理 `kFreqCCv3`，对 FreqCCv4 无效。
6. `fair_share_bandwidth_bps_` 当前只写入 CRUISE summary trace，不参与选择、裁剪或 pacing。
7. `w_freq_` 当前只用于 gate trace，不缩放三角波幅度。
8. `ComputeCongestionScore()` 和 `ack_window_history_` 已存在，但当前窗口质量和 TrustedBw 选择没有调用该分数。
9. `trusted_bw_conf_` 用于 trace，不参与 pacing 的连续权重。
10. 四流场景里的 `trace.enable_trusted_bw_selection_trace` 实际映射到 `g_enable_convergence_gate_trace`。
11. CRUISE 窗口 trace 和 gate trace 是两个独立开关；`enableHeavyTrace` 又是第三组高频 trace。
12. 主场景默认用 `BandwidthLatest()` 作为文中 Delivery Rate 信号历史。

## 19. Trace 输出

### 19.1 CRUISE 每窗口

文件：

```text
flow<N>_cruise_full_load_quality.csv
```

关键字段：

- 时间和 `cruise_id`；
- 配置频率及 srate/drate/SRTT 峰值频率；
- drate/SRTT 频率分数；
- SNR、频带集中度、峰宽、相位一致性；
- `Q_drate`、`Q_srtt`、`Q_joint`；
- 三个 gate pass 位和 limiting signal；
- `spectral_invalid_reason`；
- V1/V2 质量、candidate、low confidence、rank；
- `window_source=NORMAL|MERGED`。

### 19.2 CRUISE 汇总

文件：

```text
flow<N>_cruise_best_full_load_window.csv
```

每轮 CRUISE 一行，记录候选数量、最佳窗口、TrustedBw、source、选择时的 NativeBw、新鲜度和应用有效位。

### 19.3 收敛与 pacing gate

文件：

```text
flow<N>_freq_gate_trace.csv
```

`gateTraceMode`：

| 模式 | CSV 行 |
|---|---|
| `off` | 不写 gate CSV |
| `round_only` | 每个 RTT 一行 |
| `sampled_pacing` | RTT 行 + 按最小间隔抽样的 pacing 行 |
| `full` | RTT 行 + 每次 pacing 查询行，数据量最大 |

关键字段包括 `D_round`、`v_round`、stable 状态、TrustedBw 生命周期、双信号分数、pacing base source、phase gain、Native/final pacing、MERGED rescue 结果和选择耗时。

### 19.4 其他

主场景还可以输出：

- goodput；
- BBR mode；
- loss rate；
- `enableHeavyTrace=true` 时的 bw/send rate/recv rate/raw recv rate/queue delay/inflight；
- bottleneck queue occupancy；
- `enableEquivalenceAuditTrace=true` 时的 packet/ACK/pacing audit。

## 20. 四流主场景的当前参数

文件头注释写着“8 Mbps、60 ms”，但现行常量已经不同。实际静态拓扑是：

| 项目 | 当前代码值 |
|---|---:|
| 流数 | 4 |
| 每侧 edge link | 8 Mbps，1 ms |
| 共享 bottleneck | 20 Mbps，18 ms |
| 基础端到端 RTT | `2*(1+18+1)=40 ms` |
| 队列 | 按 40 ms 计算的 1 BDP DropTail |
| 默认每流数据量 | 15,000,000 bytes |
| 默认仿真时长 | 30 s |
| 默认 flow start | 同时开始 |
| staggered start | 0/20/40/60 ms |
| 默认动态 delay | 开启 |
| trace 中 fair-share | `20 Mbps / 4 = 5 Mbps`，仅诊断 |

动态传播时延在 4、7、12、18、22、27 秒切换，代码打印的目标 path RTT 依次为 20、130、28、118、20、144 ms。

每条流自己的 8 Mbps edge link 也会形成上限；因此“共享瓶颈 20 Mbps”和“单流最大可见速率”不能混为一谈。

## 21. 构建与运行

### 21.1 依赖和构建

FreqCCv4 的 FFT 实现依赖 FFTW3，DQC 模块通过 `FFTW3` 链接。

```bash
cd /home/wkd/BBR_ICC/NS3.27
./waf build
```

### 21.2 最小运行

```bash
cd /home/wkd/BBR_ICC/NS3.27
./waf --run "scratch/freqccv4_4flow \
  --algo=freqccv4 \
  --sim_time=30 \
  --freqBbrConfig=/home/wkd/BBR_ICC/NS3.27/examples/CCconfig/freqccv4_default.conf \
  --outputDir=/tmp/freqccv4_run/"
```

### 21.3 快速 smoke

```bash
./waf --run "scratch/freqccv4_4flow \
  --algo=freqccv4 \
  --smokeMode=true \
  --outputDir=/tmp/freqccv4_smoke/"
```

`smokeMode` 会把仿真压到最多 0.5 s、每流最多 20000 bytes、关闭动态 delay 和 heavy trace，并把高频 gate trace 降为 `round_only`。它只验证运行链路，不足以产生完整的 5 Hz/4-cycle CRUISE 频谱窗口。

### 21.4 关键 CLI

```text
--algo=freqccv4|bbrv2
--freqBbrConfig=<path>
--freq1..4=<Hz>
--amp1..4=<mode>
--fixed1..4=<Mbps>
--enableConvergenceGateTrace=true|false
--enableConvergenceGateControl=true|false
--enableCruiseWindowTrace=true|false
--gateTraceMode=off|round_only|sampled_pacing|full
--gateTraceSampleIntervalUs=<us>
--enableHeavyTrace=true|false
--flowStartMode=same_start|staggered_start
--flowSizeBytes=<bytes>
--dynamic_delay_enable=true|false
--processIntervalUs=<us>
--useEngineTimer=true|false
--seed=<seed>
--runId=<run>
--outputDir=<dir>
```

## 22. 自测试和验证

### 22.1 三个内置自测试

```bash
cd /home/wkd/BBR_ICC/NS3.27

./waf --run "scratch/freqccv4_4flow --gateStateMachineSelfTest=true"
./waf --run "scratch/freqccv4_4flow --trustedBwSelectionSelfTest=true"
./waf --run "scratch/freqccv4_4flow --trustedBwPacingSelfTest=true"
```

分别验证：

- 单轮 25% / 连续两轮 15% 的稳定态退出和三轮重新收敛；
- 联合分数严格取最小值，任一信号无效或低于门限都拒绝；
- REFILL/UP/DOWN 的 baseline 选择和 phase gain 公式；生产路径无 TrustedBw 时精确回退 `Bbr2Sender::PacingRate()`。

快捷脚本：

```bash
./run_freqccv4_trusted_bw_validation.sh
```

该脚本当前只运行 selection 和 pacing 两项，不包含 gate state-machine 自测试。

### 22.2 A/B/C gate 评估

```bash
./run_freqccv4_4flow_gate_eval.sh
```

分组：

```text
A: Native BBRv2
B: FreqCCv4，gate trace/control 都关闭
C: FreqCCv4，gate trace/control 都开启
```

同时覆盖 `same_start` 和 `staggered_start`，最后调用 `analyze_freqccv4_4flow_gate_eval.py`。

### 22.3 正式矩阵

长流动态 RTT：

```bash
./run_freqccv4_formal_long_lived.sh
```

有限流 FCT：

```bash
./run_freqccv4_formal_fct.sh
```

二者默认使用 5 个 seed；设置 `FORMAL_MODE=full` 时使用 10 个 seed。分析入口是：

```bash
python3 analyze_freqccv4_formal_matrix.py \
  --results-dir results/freqccv4_formal_matrix
```

另有 packet instability 多 seed 脚本：

```bash
./run_freqccv4_packet_instability_multiseed.sh
```

### 22.4 本文档生成时的验证结果

在 2026-07-13 当前工作区执行结果：

```text
./waf build                                      PASS
gateStateMachineSelfTest                         PASS
trustedBwSelectionSelfTest                       PASS
trustedBwPacingSelfTest                          PASS
freqccv4_4flow --smokeMode=true                  PASS
```

## 23. 关键不变式

实现或修改 FreqCCv4 时应保持：

1. CRUISE 中 `pacing_base_source` 必须是 `NATIVE_BBR`。
2. `TrustedBw` 只能来自双信号都通过的 NORMAL/MERGED 可靠窗口。
3. `Q_joint` 必须严格等于 `min(Q_drate, Q_srtt)`。
4. MERGED rescue 必须重新跑完整双信号门控，不能复用失败 NORMAL 的单信号结果。
5. TrustedBw 只能在本轮 CRUISE 后的 REFILL/UP/DOWN 使用。
6. 新 CRUISE 开始后旧结果必须失去 fresh/application/ready 状态。
7. 无有效 TrustedBw 时 pacing 必须精确回退 `Bbr2Sender::PacingRate()`。
8. TrustedBw 不得写入 Native MaxBw、BandwidthEstimate、BDP、cwnd 或 inflight 上下界。
9. gate control 关闭时，收敛状态机不得改变默认调制控制。
10. trace 中必须能区分 Native/Trusted pacing base、NORMAL/MERGED 来源和无效原因。

## 24. 伪代码汇总

```text
on enter CRUISE(now):
    cruise_id += 1
    clear_trusted_application("cruise_start")
    cruise_start = now
    fixed_freq = configured_freq
    windows.clear()
    next_window_start = now + min_rtt

on pacing query(now):
    native_pacing = BBRv2.PacingRate()
    phase = current_probe_bw_phase

    use_trusted =
        phase in {REFILL, UP, DOWN, DOWN_SLIGHTLY} and
        trusted valid/fresh/ready/application-valid and
        trusted.cruise_id == current cruise_id and
        (control disabled or not stable)

    if use_trusted:
        baseline = phase_gain * trusted_bw
    else:
        baseline = native_pacing

    oscillate = in CRUISE and amplitude>0 and freq>0 and
                (control disabled or not stable)

    if oscillate:
        return max(1000, baseline + amplitude * triangle(now))
    if use_trusted:
        return baseline
    return native_pacing

on ACK(event):
    BBRv2 handles event
    collect drate, SRTT, ack/loss history
    if in CRUISE:
        while a complete sliding window is due:
            windows.push(build_window(NORMAL))

on RTT end:
    D_round = max valid non-app-limited delivery sample in RTT
    v_round = abs(D_round-D_prev)/D_prev
    update stable/unstable state

on leave CRUISE(now):
    finish all due NORMAL windows
    best = highest V2 reliable NORMAL window
    if best missing and merged rescue enabled:
        build extended MERGED windows
        reject high-trend windows
        best = highest V2 reliable MERGED window

    if best exists and both signal gates pass:
        trusted_bw = mean_resampled_drate(best)
        publish fresh TrustedBw for this cruise_id
    else:
        clear TrustedBw("no_dual_signal_trusted_bw")

    emit per-window and per-cruise traces
```

## 25. 最终归纳

FreqCCv4 当前不是“用频域估计全面替代 BBRv2”的算法，而是一个严格限定作用面的闭环：

```text
CRUISE 主动注入已知载波
    -> Delivery Rate 与 SRTT 双信号验证载波响应
    -> 从可靠窗口取平均 Delivery Rate 作为 TrustedBw
    -> 仅在下一组 REFILL/UP/DOWN 替换 pacing baseline
    -> 下一次 CRUISE 重新测量，旧结果失效
```

它的核心安全性来自三点：双信号硬门控、fresh-only 生命周期、pacing-only 应用边界。当前实现中的 fair-share、`w_freq`、拥塞分数和 confidence 主要仍是诊断信息，不能把它们描述成已经参与控制的机制。
