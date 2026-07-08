# FreqCCv4 频域工作流与 trustedBw 输出逻辑

本文按当前仓库代码整理 FreqCCv4 在频域中的完整实现链路：它如何在 `PROBE_CRUISE` 注入固定频率扰动，如何采集 `drate` 和 `srtt`，如何把时域样本转成频域特征，如何给每个观测窗口打分，最后如何把可信窗口的交付速率转换成控制用的 `trustedBw`。

注意：当前代码里没有名为 `trustedBw` 的变量。本文把实现中的可信带宽参考 `f_ref_` 视为 `trustedBw` 的代码对应物：

```text
trustedBw_bps ~= f_ref_.ToBitsPerSecond()
              = best可信full-load窗口的 drate_mean_kbps * 1000
```

相关源码主要在：

- `NS3.27/src/dqc/model/thirdparty/congestion/freqccv4_sender.h`
- `NS3.27/src/dqc/model/thirdparty/congestion/freqccv4_sender.cc`
- `NS3.27/src/dqc/model/dqc_sender.cc`
- `NS3.27/src/dqc/model/dqc_trace.cc`
- `NS3.27/scratch/freqccv4_4flow.cc`

## 1. 总体思路

FreqCCv4 的频域逻辑本质上是一个主动探测闭环：

```text
1. 只在 BBRv2 PROBE_BW::PROBE_CRUISE 阶段注入固定频率 f0 的三角波 pacing 扰动。
2. 发送端保存被扰动后的 srate 序列。
3. ACK 到达时保存接收/交付速率 drate 序列和 SRTT 序列。
4. 在 cruise 内按 RTT 对齐后的滑动窗口切片。
5. 对 srate / drate / srtt 做 1ms 重采样、去均值、Hann 加窗、零填充 FFT。
6. 在 [0.7f0, 1.3f0] 频带内找峰值、幅值、噪声底和频谱形状。
7. 用 drate 和 srtt 是否同时响应 f0 来判断这个窗口是否像 full-load。
8. 离开 cruise 时选出质量最高且非 low-confidence 的 full-load 窗口。
9. 用该窗口的平均 drate 生成 f_ref_，也就是本文所称 trustedBw。
10. pacing 层把原生 BBRv2 带宽参考和 f_ref_ 按权重混合，再输出最终 pacing rate。
```

它不是被动估计频率，而是先由发送端主动注入已知频率 `f0`，再看交付速率和 RTT 是否在同一频率附近产生响应。这个响应越稳定，窗口越可信。

## 2. 代码入口与配置

场景脚本 `freqccv4_4flow.cc` 在创建发送应用时调用：

```text
sendApp->ConfigureFreqCC(freq_hz, amp_mode, fixed_mbps)
sendApp->ConfigureFreqCCv4ConvergenceGate(...)
sendApp->SetFreqCCIntervalWindowMultiplier(...)
sendApp->SetFreqCCFairShareBandwidth(...)
```

对应代码在 `NS3.27/scratch/freqccv4_4flow.cc:282-294`。

`ConfigureFreqCC()` 对 FreqCCv4 做三件事：

- 设置扰动频率：`SetOscillationFrequency(freq_hz)`。
- 设置扰动幅度模式：`SetOscillationAmplitude(amp_mode, fixed_bps)`。
- 设置接收侧信号来源：默认用 `BandwidthLatest()`，传入 `delivery_rate_latest` / `recvrate_raw` 等模式时用 `DeliveryRateLatest()`。

对应代码在 `NS3.27/src/dqc/model/dqc_sender.cc:1088-1133`。

`ConfigureFreqCCv4ConvergenceGate()` 设置三个重要开关：

- `enable_convergence_gate_trace_`：是否输出 gate / cruise trace。
- `enable_convergence_gate_control_`：是否在 BBR 稳定时关闭 CRUISE 扰动。
- `enable_freq_ref_pacing_control_`：是否把 `f_ref_` 用到 pacing 缩放。

对应代码在 `NS3.27/src/dqc/model/dqc_sender.cc:908-933`。

## 3. 频域扰动如何产生

FreqCCv4 只在满足 `BaseShouldOscillate()` 时注入扰动：

```text
amplitude > 0
configured_modulation_freq_hz_ > 0
drain_completed_ == true
mode_ == PROBE_BW
probe_bw_phase == PROBE_CRUISE
in_cruise_ == true
cruise_start_time_ 有效
```

对应代码在 `freqccv4_sender.cc:440-448`。

当 `enable_convergence_gate_control_` 打开时，`ShouldOscillate()` 还要求 `!bbr_stable_`。也就是说，如果 BBRv2 已经重新稳定，频域扰动会被 gate 关闭。对应代码在 `freqccv4_sender.cc:451-457`。

扰动波形是三角波，范围约为 `[-1, +1]`：

```text
period = 1 / f0
q = (now - cruise_start_time) mod period / period

q < 0.25:  tri = 4q
q < 0.75:  tri = 2 - 4q
otherwise: tri = 4q - 4
```

对应代码在 `freqccv4_sender.cc:491-510`。

最终 pacing rate 计算为：

```text
native_pacing = Bbr2Sender::PacingRate(bytes_in_flight)
effective_base_pacing = native_pacing * scale
amplitude_eff = amplitude_bps * scale
offset = amplitude_eff * triangle_wave(now)
final_pacing = max(effective_base_pacing + offset, 1000 bps)
```

其中 `scale` 来自 `f_ref_` 控制面；如果没有启用 `F_ref` pacing 控制，`scale = 1`。对应代码在 `freqccv4_sender.cc:1214-1302`。

## 4. drate 与 srtt 如何采集

### 4.1 srate：发送端速率样本

每次发包时，FreqCCv4 先调用自己的 `PacingRate()` 得到当前实际 pacing rate，并写入：

```text
sender_rate_history_.push_back({sent_time, sender_rate})
```

这意味着 `srate` 记录的是已经叠加 FreqCCv4 扰动后的发送速率。对应代码在 `freqccv4_sender.cc:1135-1145`。

### 4.2 drate：接收/交付速率样本

每次 ACK/loss 事件中，FreqCCv4 先让 BBRv2 更新模型：

```text
Bbr2Sender::OnCongestionEvent(...)
```

然后如果本次事件有 ACK 字节且时间前进，就记录接收侧速率信号：

```text
recv_signal = use_delivery_rate_latest_for_signal_history_
              ? DeliveryRateLatest()
              : BandwidthLatest()

delivery_rate_history_.push_back({event_time, recv_signal})
```

对应代码在 `freqccv4_sender.cc:1154-1181`。

因此本文中的窗口级 `drate` 指 `delivery_rate_history_`，它是频域窗口分析使用的交付/接收速率序列。它不是 round 级 `d_round_`，后者只服务于收敛门控。

### 4.3 srtt：平滑 RTT 样本

同一个 ACK 事件中，代码读取：

```text
smoothed_rtt = rtt_stats_->smoothed_rtt()
if smoothed_rtt == 0:
    smoothed_rtt = rtt_stats_->SmoothedOrInitialRtt()

srtt_history_.push_back({event_time, smoothed_rtt_ms})
```

单位转换为毫秒。对应代码在 `freqccv4_sender.cc:1182-1191`。

### 4.4 ACK/loss 窗口样本

代码还保存：

```text
ack_window_history_.push_back({event_time, acked_bytes, has_loss})
```

`ComputeCongestionScore()` 可用它计算 ECN/loss 拥塞分数，但当前 `AnalyzeCruiseWindow()` 没有把这个分数纳入窗口评分。对应代码在 `freqccv4_sender.cc:1193-1194` 和 `freqccv4_sender.cc:2211-2231`。

## 5. round 级 D_round 与频域窗口 drate 的区别

当前 FreqCCv4 有两套 delivery-rate 概念：

```text
窗口级 drate:
    delivery_rate_history_
    用于 FFT、频域响应检测、窗口 full-load 评分、生成 f_ref_。

round级 D_round:
    d_round_
    每个 packet-timed round 内最大 fresh non-app-limited delivery-rate sample。
    用于判断 BBR 是否失稳、是否重新收敛、是否需要开启频域工具。
```

`D_round` 只接受满足以下条件的 sample：

```text
sample_valid == true
sample_max_bandwidth > 0
sample_is_app_limited == false
InRecovery() == false
```

并在 round 内取最大值。对应代码在 `freqccv4_sender.cc:570-585`。

每个 round 结束时，代码计算：

```text
v_round = abs(D_round - D_prev) / D_prev
```

如果 BBR 已稳定，但：

```text
v_round > 0.25
or (v_round > 0.15 and previous_v_round > 0.15)
```

就退出稳定状态，开启 unstable episode，`w_freq_ = 1.0`，并准备使用频域工具。对应代码在 `freqccv4_sender.cc:587-658`。

重新收敛时，如果 `D_round` 没有比 `full_drate_ref_` 增长 25%，`stable_cnt_` 递增；连续 `kStableRounds = 3` 轮后认为 BBR 重新稳定，清空 active `f_ref_` 并关闭频域工具。对应代码在 `freqccv4_sender.cc:661-722`。

## 6. 观测窗口如何切分

窗口分析只在 `PROBE_CRUISE` 内运行。进入 cruise 时：

```text
cruise_start_time_ = now
cruise_modulation_freq_hz_ = configured_modulation_freq_hz_
next_cruise_window_start_ = cruise_start_time_ + min_rtt
```

对应代码在 `freqccv4_sender.cc:524-560`。

窗口从 `cruise_start_time_ + min_rtt` 开始，是为了让发送端扰动经过约一个 RTT 后再在接收侧 `drate/srtt` 中体现。

窗口长度：

```text
window_duration_s = max(
    min_cruise_cycles_per_window_ / f0,
    2 * min_rtt_s
)
```

默认：

```text
min_cruise_cycles_per_window_ = 4.0
cruise_window_step_ratio_ = 0.25
```

也就是每个窗口至少覆盖 4 个扰动周期，并且步长默认为窗口长度的 1/4。对应代码在 `freqccv4_sender.h:370-371` 和 `freqccv4_sender.cc:1309-1355`。

## 7. 时域样本如何进入频域

`AnalyzeCruiseWindow()` 对一个窗口做三组信号分析：

```text
srate_samples = sender_rate_history_[window_start - min_rtt, window_end - min_rtt]
drate_samples = delivery_rate_history_[window_start, window_end]
srtt_samples  = srtt_history_[window_start, window_end]

srate = AnalyzeRateSeries(..., detrend=false)
drate = AnalyzeRateSeries(..., detrend=false)
srtt  = AnalyzeRttSeries(..., detrend=true)
```

发送端 `srate` 使用 `min_rtt` 前移后的窗口，是为了和接收端观测对齐。对应代码在 `freqccv4_sender.cc:1358-1384`。

### 7.1 选择窗口内样本

`SelectRateSamples()` 和 `SelectRttSamples()` 只保留时间戳落在 `[start, end]` 内的样本。对应代码在 `freqccv4_sender.cc:1694-1718`。

### 7.2 1ms 等间隔重采样

FFT 需要等间隔序列，因此 rate 和 RTT 都被线性插值到固定步长：

```text
kSampleStepSec = 0.001
fs = 1 / kSampleStepSec = 1000 Hz
```

rate 重采样输出单位为 `Kbit/s`，RTT 重采样输出单位为 `ms`。如果原始样本少于 2 个，或者重采样后少于 8 个点，窗口无效。对应代码在 `freqccv4_sender.cc:1720-1821`。

### 7.3 均值与 detrend

`AnalyzeRateSeries()` 和 `AnalyzeRttSeries()` 都会先计算重采样序列均值：

```text
mean_value = average(values)
```

对 `srtt` 额外做线性 detrend：

```text
values[i] -= first + frac * (last - first)
```

当前 `srate/drate` 调用时 `detrend=false`，`srtt` 调用时 `detrend=true`。对应代码在 `freqccv4_sender.cc:1823-1883`。

## 8. FFT 频谱特征如何构造

`BuildSpectrumProfile()` 是频域特征核心。输入是重采样后的时域序列，输出 `SpectrumProfile`：

```text
peak_freq_hz
band_peak_rel
band_energy_rel
target_amp
noise_floor
noise_floor_valid
band_shape
valid
```

对应结构定义在 `freqccv4_sender.h:134-144`，实现对应 `freqccv4_sender.cc:1885-2018`。

### 8.1 去均值、Hann 加窗、4倍零填充

FFT 前先做：

```text
signal_len = values.size()
nfft = signal_len * kFftZeroPadMultiplier
kFftZeroPadMultiplier = 4

in[i] = (values[i] - mean(values)) * hann(i)
in[signal_len:nfft] = 0
```

然后调用：

```text
fftw_plan_dft_r2c_1d(nfft, in, out, FFTW_ESTIMATE)
```

对应代码在 `freqccv4_sender.cc:1894-1920`。

### 8.2 频率分辨率

```text
fs = 1 / sample_step_s = 1000 Hz
freq_step = fs / nfft
```

只分析正频率 bin，跳过 DC：

```text
k_min = 1
k_max = nfft / 2
magnitude[k] = sqrt(real^2 + imag^2)
```

对应代码在 `freqccv4_sender.cc:1922-1933`。

### 8.3 目标频带

FreqCCv4 不只看单个频率点，而是在配置频率 `f0` 附近找峰：

```text
band_low_hz  = 0.70 * f0
band_high_hz = 1.30 * f0
```

对应常量在 `freqccv4_sender.h:434-435`，实现对应 `freqccv4_sender.cc:1935-1953`。

窗口频谱输出的关键含义：

- `target_amp`：目标频带内最大幅值。
- `peak_freq_hz`：目标频带内最大幅值对应频率，并用相邻 bin 做抛物线插值修正。
- `noise_floor`：目标频带外幅值的中位数。
- `band_energy_rel`：目标频带幅值和 / 全频带幅值和。
- `band_peak_rel`：目标频带峰值 / 全频带幅值和。
- `band_shape`：把 `[0.7f0, 1.3f0]` 插值成 16 个点并归一化，用于比较 drate 频谱形状是否像 srate。

对应代码在 `freqccv4_sender.cc:1954-2012`。

## 9. 每个窗口如何打分

一个 `CruiseWindowResult` 会记录窗口的频率、幅值、波形、一致性、候选状态和最终质量。结构定义在 `freqccv4_sender.h:170-214`。

### 9.1 频率分数

每个信号都计算峰值频率和配置频率 `f0` 的接近程度：

```text
freq_tolerance = max(0.20 * f0, 2 / window_duration_s)
score = clamp01(1 - abs(peak_freq_hz - f0) / freq_tolerance)
```

对应代码在 `freqccv4_sender.cc:1362-1365` 和 `freqccv4_sender.cc:2200-2208`。

窗口频率质量只使用 `drate` 和 `srtt`：

```text
freq_quality = 0.5 * drate_freq_score + 0.5 * srtt_freq_score
```

对应代码在 `freqccv4_sender.cc:1401-1417`。

### 9.2 幅值响应分数

drate 幅值响应看的是接收侧扰动幅值相对发送侧扰动幅值的比例：

```text
drate_gain = drate_target_amp / max(srate_target_amp, epsilon)
drate_amplitude_score = clamp01(drate_gain / kTargetDrateGain)
kTargetDrateGain = 0.30
```

对应代码在 `freqccv4_sender.cc:1419-1425`。

srtt 幅值响应看的是目标频带幅值和噪声底的信噪比：

```text
srtt_snr = srtt_target_amp / noise_floor
srtt_amplitude_score = ScoreThreshold(srtt_snr, 1.50, 3.00)
```

其中：

```text
ScoreThreshold(value, min, target) = clamp01((value - min) / (target - min))
```

如果噪声底不可用但 `srtt` 有效，`srtt_amplitude_score` 默认 0.5。对应代码在 `freqccv4_sender.cc:1427-1443` 和 `freqccv4_sender.cc:2240-2247`。

综合幅值质量：

```text
amplitude_quality =
    0.4 * drate_amplitude_score
  + 0.6 * srtt_amplitude_score
```

### 9.3 波形质量

drate 波形质量比较 `drate.band_shape` 和 `srate.band_shape`：

```text
shape_distance = sum(abs(drate_shape[i] - srate_shape[i]))
drate_waveform_quality = clamp01(1 - shape_distance / 0.40)
```

如果缺少可比频谱形状，默认 0.5。对应代码在 `freqccv4_sender.cc:1445-1455` 和 `freqccv4_sender.cc:2020-2031`。

srtt 波形质量由 `AnalyzeCycleQuality()` 对每个周期的形状做时域检查，统计：

- `top_clip_ratio`：周期顶部是否出现削顶/平台。
- `bottom_clip_ratio`：周期底部是否出现削底/平台。
- `cycle_incompleteness`：是否缺少明显上升或下降段。
- `cycle_asymmetry`：前后半周期变化量是否不对称。
- `phase_instability`：各周期峰值相位是否飘移。

失真分数：

```text
distortion_score =
    0.25 * top_clip_ratio
  + 0.25 * bottom_clip_ratio
  + 0.20 * cycle_incompleteness
  + 0.15 * cycle_asymmetry
  + 0.15 * phase_instability

srtt_waveform_quality = clamp01(1 - distortion_score)
```

对应代码在 `freqccv4_sender.cc:2033-2198`。

综合波形质量：

```text
waveform_quality =
    0.4 * drate_waveform_quality
  + 0.6 * srtt_waveform_quality
```

对应代码在 `freqccv4_sender.cc:1457-1478`。

### 9.4 周期一致性质量

`AnalyzeCycleQuality()` 还输出：

```text
cycle_frequency_stability
cycle_phase_stability
```

频率稳定性来自相邻周期峰值间隔估算出的频率是否接近 `f0`。相位稳定性来自各周期峰值相位的标准差。最终：

```text
consistency_quality =
    0.5 * cycle_frequency_stability
  + 0.5 * cycle_phase_stability
```

对应代码在 `freqccv4_sender.cc:1479-1481` 和 `freqccv4_sender.cc:2141-2182`。

### 9.5 full-load 候选判定

窗口成为 full-load candidate 的硬条件是：

```text
drate_valid == true
srtt_valid == true
drate_freq_score >= 0.60
srtt_freq_score >= 0.60
```

对应常量在 `freqccv4_sender.h:427-428`，判定代码在 `freqccv4_sender.cc:1483-1486`。

注意：`srate_freq_score` 不参与候选硬判定，但会影响 `low_confidence`，因为如果发送侧扰动本身在窗口内不稳定，这个窗口不能强信任。

### 9.6 最终窗口得分

窗口最终质量：

```text
full_load_quality =
    clamp01(
        0.35 * freq_quality
      + 0.25 * waveform_quality
      + 0.20 * amplitude_quality
      + 0.20 * consistency_quality
    )
```

对应代码在 `freqccv4_sender.cc:1487-1491`。

窗口标签：

```text
is_full_load_candidate ? "FULL_LOAD_CANDIDATE" : "NOT_FULL_LOAD_CANDIDATE"
```

对应代码在 `freqccv4_sender.cc:1492-1494`。

### 9.7 low_confidence 判定

即使 `drate/srtt` 频率匹配，窗口也可能被标成低可信：

```text
low_confidence =
    frequency_match
    and (
        srate_unstable
        or drate_sample_count < 4
        or srtt_sample_count < 4
        or expected_cycles < 2
        or valid_cycle_count < 2
        or srtt_noise_floor_invalid
        or full_load_quality < 0.50
    )
```

对应代码在 `freqccv4_sender.cc:1496-1512`。`min_full_load_quality_for_reliable_window_` 默认是 0.50，定义在 `freqccv4_sender.cc` 构造函数初始化处，字段在 `freqccv4_sender.h:373`。

## 10. cruise 结束后如何选 trustedBw

离开 `PROBE_CRUISE` 时执行 `FinalizeCruise()`：

```text
1. 找出所有 is_full_load_candidate 窗口。
2. 按 full_load_quality 从高到低排序。
3. 标记 rank，rank=1 的窗口为 is_best_full_load_window。
4. 调用 RefreshFreqRefFromBestCruiseWindow() 尝试刷新 f_ref_。
5. 输出窗口 trace 和 cruise summary trace。
```

对应代码在 `freqccv4_sender.cc:1530-1561`。

真正刷新 `f_ref_` 时，比 summary 更严格：`RefreshFreqRefFromBestCruiseWindow()` 会跳过：

```text
!is_full_load_candidate
low_confidence == true
```

然后在剩余可信候选中选择 `full_load_quality` 最高的窗口。对应代码在 `freqccv4_sender.cc:774-784`。

如果找不到可信窗口，或 `drate_mean_kbps <= 0`，则不刷新 `f_ref_`。对应代码在 `freqccv4_sender.cc:786-792`。

如果找到可信窗口：

```text
refreshed_ref = QuicBandwidth::FromBitsPerSecond(
    round(best->drate_mean_kbps * 1000)
)
```

这就是本文所称的 `trustedBw`。对应代码在 `freqccv4_sender.cc:794-795`。

根据当前是否处于 unstable episode，写入位置不同：

```text
if !bbr_stable_ and unstable_episode_active_:
    f_ref_ = refreshed_ref
    f_conf_ = best->full_load_quality
    f_ref_valid_ = true
else:
    staged_f_ref_ = refreshed_ref
    staged_f_conf_ = best->full_load_quality
    staged_f_ref_valid_ = true
```

对应代码在 `freqccv4_sender.cc:796-807`。

含义是：

- 如果当前 BBR 已经失稳且频域工具正在参与控制，可信窗口直接更新 active `f_ref_`。
- 如果当前 BBR 稳定，可信窗口先暂存为 `staged_f_ref_`，等下一次检测到失稳时再激活。

`f_ref_` 和 `staged_f_ref_` 都有 cruise-window 级 TTL，最大年龄为 `kMaxFreqRefAgeCruiseWindows = 2`。对应代码在 `freqccv4_sender.h:424` 和 `freqccv4_sender.cc:751-772`。

## 11. trustedBw 如何进入最终 pacing 输出

`PacingRate()` 里先拿到 BBRv2 原生 pacing 和原生带宽参考：

```text
native_pacing = Bbr2Sender::PacingRate(bytes_in_flight)
b_native = BandwidthEstimate()
```

当以下条件同时满足：

```text
enable_freq_ref_pacing_control_ == true
bbr_stable_ == false
f_ref_valid_ == true
b_native > 0
```

代码构造目标带宽：

```text
b_target = (1 - w_freq_) * b_native + w_freq_ * f_ref_
raw_scale = b_target / b_native
scale = ApplyFreqRefScaleMode(raw_scale)
```

对应代码在 `freqccv4_sender.cc:1214-1238`。

`ApplyFreqRefScaleMode()` 默认把 scale 限制在：

```text
low = 0.75
high = 1.10
```

不同模式会进一步约束：

- `current`：双向缩放，使用默认 `[0.75, 1.10]`。
- `downward_only`：只允许降低，不允许上调。
- `asymmetric`：上调部分乘 `freq_ref_up_beta_`，并把上界改成 1.05。
- `high_conf_only`：只有 `f_conf_ >= freq_ref_high_conf_threshold_` 时才使用。
- `early_episode_only`：只在 `stable_cnt_ <= 1` 的早期失稳阶段使用。

对应代码在 `freqccv4_sender.cc:817-864`。

最终：

```text
effective_base_pacing = native_pacing * scale
final_pacing = effective_base_pacing + amplitude_bps * scale * triangle_wave
returned_pacing =
    if no oscillation and no scale applied:
        native_pacing
    else:
        final_pacing
```

对应代码在 `freqccv4_sender.cc:1240-1302`。

因此完整控制链路可以写成：

```text
best window
  -> best.drate_mean_kbps
  -> trustedBw_bps / f_ref_
  -> b_target = blend(BandwidthEstimate(), f_ref_, w_freq_)
  -> scale = clamp(b_target / BandwidthEstimate())
  -> native_pacing * scale
  -> 再叠加 CRUISE 三角波扰动
  -> returned_pacing
```

## 12. trace 输出中如何看这些结果

窗口级 trace 文件：

```text
*_cruise_full_load_quality.csv
```

字段包括：

```text
srate_peak_freq_hz
drate_peak_freq_hz
srtt_peak_freq_hz
drate_freq_score
srtt_freq_score
freq_quality
drate_target_amp
srate_target_amp
drate_gain
drate_amplitude_score
srtt_target_amp
srtt_noise_floor
srtt_snr
srtt_amplitude_score
drate_waveform_quality
srtt_waveform_quality
waveform_quality
cycle_frequency_stability
cycle_phase_stability
consistency_quality
is_full_load_candidate
full_load_quality
full_load_rank_in_cruise
is_best_full_load_window
low_confidence
label
```

对应 header 在 `NS3.27/src/dqc/model/dqc_trace.cc:567-590`。

cruise 汇总文件：

```text
*_cruise_best_full_load_window.csv
```

字段包括：

```text
best_full_load_quality
best_drate_freq_score
best_srtt_freq_score
best_srtt_waveform_quality
best_drate_amplitude_score
best_srtt_amplitude_score
best_drate_mean_kbps
```

对应 header 在 `NS3.27/src/dqc/model/dqc_trace.cc:592-611`。

注意：summary 中的 best window 是按 `is_full_load_candidate` 和 `full_load_quality` 选出的 best candidate；而控制面的 `f_ref_` 更新还会排除 `low_confidence` 窗口。因此做控制面分析时，以 gate trace 的 `f_ref` 和 `f_ref_valid` 为准。

gate trace 文件：

```text
flowX_freq_gate_trace.csv
```

关键字段：

```text
d_round
d_prev
v_round
full_drate_ref
stable_cnt
bbr_stable
freq_tool_needed
freq_tool_on
f_ref
f_ref_valid
f_conf
w_freq
b_native
b_target
b_eff
scale
raw_scale
native_pacing
final_pacing
current_delivery_rate
```

对应 header 在 `NS3.27/src/dqc/model/dqc_trace.cc:614-638`。

## 13. 一句话代码映射

```text
drate 提取:
    OnCongestionEvent()
    -> BandwidthLatest() 或 DeliveryRateLatest()
    -> delivery_rate_history_

srtt 提取:
    OnCongestionEvent()
    -> rtt_stats_->smoothed_rtt()
    -> srtt_history_

频域特征:
    Select samples
    -> 1ms resample
    -> optional detrend
    -> mean removal + Hann
    -> 4x zero-padding FFTW
    -> peak_freq / target_amp / noise_floor / band_shape

窗口评分:
    freq_quality
    amplitude_quality
    waveform_quality
    consistency_quality
    -> full_load_quality
    -> is_full_load_candidate / low_confidence

trustedBw:
    FinalizeCruise()
    -> RefreshFreqRefFromBestCruiseWindow()
    -> best non-low-confidence candidate
    -> trustedBw_bps = best.drate_mean_kbps * 1000
    -> f_ref_

最终输出:
    PacingRate()
    -> b_target = (1 - w_freq) * BandwidthEstimate() + w_freq * f_ref_
    -> scale = clamp(b_target / BandwidthEstimate())
    -> final_pacing = Bbr2Sender::PacingRate() * scale + triangle_wave_offset
```

## 14. 当前实现的几个重要边界

- `trustedBw` 不是独立变量名；代码控制面使用的是 `f_ref_`，trace 里字段名也是 `f_ref`。
- `best_drate_mean_kbps` 是窗口内重采样 drate 序列的平均值，不是瞬时最大 delivery rate。
- `D_round` 是收敛门控用的 round 级最大 fresh non-app-limited delivery rate，不直接用于窗口频域 FFT。
- `ComputeCongestionScore()` 已实现 ECN/loss 拥塞分数，但当前窗口 full-load 评分没有使用它。
- summary trace 的 best candidate 不等于一定被控制面信任；`f_ref_` 刷新会排除 `low_confidence`。
- `enableFreqRefPacingControl=false` 时，频域逻辑仍可分析和输出 trace，但不会用 `f_ref_` 改 pacing。
- `enableConvergenceGateControl=false` 时，扰动不会被稳定状态 gate 关闭，仍按基础 CRUISE 条件运行。
