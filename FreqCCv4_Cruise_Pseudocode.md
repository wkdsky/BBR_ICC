# FreqCCv4 Cruise 阶段逻辑伪代码说明

本文档整理 `FreqCCv4Sender` 在 `PROBE_BW::PROBE_CRUISE` 阶段的核心逻辑，来源于：

- `NS3.27/src/dqc/model/thirdparty/congestion/freqccv4_sender.cc`
- `NS3.27/src/dqc/model/thirdparty/congestion/freqccv4_sender.h`
- `NS3.27/src/dqc/model/dqc_trace.cc`

FreqCCv4 的 cruise 阶段做两件事：

1. 只在 BBRv2 的 `PROBE_CRUISE` 阶段对 pacing rate 叠加固定频率三角波扰动。
2. 在 cruise 内用滑动窗口分析发送速率、接收/交付速率、SRTT 的频谱响应，筛选可能处于 full-load 的窗口，并在离开 cruise 时输出窗口级和 cruise 汇总结果。

## 阶段入口与出口

源码入口是 `OnProbeBwPhaseEntered(phase, now)`。FreqCCv4 不改变 BBRv2 的阶段机，只监听 BBRv2 何时进入/离开 `PROBE_CRUISE`。

```text
on_probe_bw_phase_entered(phase, now):
    if phase == PROBE_CRUISE:
        enter_cruise(now)
        return

    if currently_in_cruise:
        leave_cruise(now)
```

### enter_cruise(now)

```text
enter_cruise(now):
    in_cruise = true

    # 每次进入 cruise 都分配一个新的 cruise_id，
    # 后续所有窗口输出都带这个 id，便于按一次 cruise 周期聚合。
    cruise_id += 1

    cruise_start_time = now

    # cruise 内使用进入时的配置频率作为本轮固定扰动频率。
    cruise_modulation_freq_hz = configured_modulation_freq_hz

    # 允许本轮重新打印 minRTT 缺失告警。
    min_rtt_warning_logged = false

    # 清空上一轮 cruise 的窗口分析缓存。
    current_cruise_windows.clear()

    # 初始窗口从 cruise_start_time + minRTT 开始，
    # 因为发送端速率扰动需要大约一个 minRTT 后才会体现在接收端/RTT 观测中。
    reset_cruise_window_state()
```

### leave_cruise(now)

```text
leave_cruise(now):
    # 离开 cruise 前先统一结算本轮所有已分析窗口：
    # 1. 找出候选窗口
    # 2. 按质量排序
    # 3. 写窗口 CSV
    # 4. 写 cruise 汇总 CSV
    finalize_cruise(now)

    in_cruise = false
    cruise_start_time = ZERO

    # 恢复为当前配置频率。下一次进入 cruise 时会重新固定。
    cruise_modulation_freq_hz = configured_modulation_freq_hz

    reset_cruise_window_state()
    current_cruise_windows.clear()
```

## Cruise 内的速率扰动

FreqCCv4 的扰动只会在 `ShouldOscillate()` 为真时启用。

```text
should_oscillate():
    if current_amplitude_bps == 0:
        return false

    if configured_modulation_freq_hz <= 0:
        return false

    if drain_has_not_completed:
        return false

    if BBR_mode != PROBE_BW:
        return false

    return current_probe_bw_phase == PROBE_CRUISE
           and in_cruise == true
           and cruise_start_time != ZERO
```

### 扰动幅度

```text
get_current_amplitude_bps():
    max_bw = BandwidthEstimate()
    base_rate = Bbr2Sender.PacingRate(0)

    switch amplitude_mode:
        kFixed: return fixed_amplitude_bps
        kMiu2:  return max_bw / 2
        kMiu3:  return max_bw / 3
        kMiu4:  return max_bw / 4
        kMiu8:  return max_bw / 8
        kSR2:   return base_rate / 2
        kSR3:   return base_rate / 3
        kSR4:   return base_rate / 4
        kSR8:   return base_rate / 8
        kSR12:  return base_rate / 12
        kSR16:  return base_rate / 16
        default: return 0
```

其中 `miu` 模式基于 BBR 的最大带宽估计，`sr` 模式基于 BBRv2 原始 pacing rate，`fixed` 模式使用外部传入的固定 bit/s 幅度。

### 三角波

```text
triangle_wave(now):
    if cruise_modulation_freq_hz <= 0 or cruise_start_time == ZERO:
        return 0

    elapsed_s = now - cruise_start_time
    period_s = 1 / cruise_modulation_freq_hz
    q = (elapsed_s mod period_s) / period_s

    # 输出范围约为 [-1, +1]。
    if q < 0.25:
        return 4 * q              # 0 -> +1
    if q < 0.75:
        return 2 - 4 * q          # +1 -> -1
    return 4 * q - 4              # -1 -> 0
```

### PacingRate()

```text
pacing_rate(bytes_in_flight):
    base_rate = Bbr2Sender.PacingRate(bytes_in_flight)

    if not should_oscillate():
        return base_rate

    amplitude_bps = get_current_amplitude_bps()
    offset_bps = amplitude_bps * triangle_wave(current_time)
    final_bps = base_rate + offset_bps

    # 避免扰动把发送速率压成 0 或负数。
    final_bps = max(final_bps, 1000)

    return Bandwidth(final_bps)
```

## 样本采集

FreqCCv4 分别维护发送端速率、接收/交付速率、SRTT、ACK 窗口四类历史。

### 发送样本

每次发包时记录当前 `PacingRate()`，所以 `sender_rate_history` 是带扰动后的发送速率序列。

```text
on_packet_sent(sent_time, bytes_in_flight, packet_number, bytes):
    current_time = sent_time

    sender_rate = pacing_rate(bytes_in_flight)
    sender_rate_history.push({time: sent_time, rate: sender_rate})
    trim_history_to_max_samples(sender_rate_history)

    Bbr2Sender.OnPacketSent(...)
```

### ACK / RTT / 接收速率样本

每次 ACK/loss 事件先交给 BBRv2 更新模型，再采集接收侧信号。

```text
on_congestion_event(event_time, acked_packets, lost_packets):
    current_time = event_time
    acked_bytes = sum(bytes_acked in acked_packets)

    Bbr2Sender.OnCongestionEvent(...)

    if first_time_entered_PROBE_BW:
        drain_completed = true

    if last_ack_time exists and event_time > last_ack_time and acked_bytes > 0:
        if use_delivery_rate_latest_for_signal_history:
            recv_signal = DeliveryRateLatest()
        else:
            recv_signal = BandwidthLatest()

        delivery_rate_history.push({time: event_time, rate: recv_signal})

        smoothed_rtt = rtt_stats.smoothed_rtt()
        if smoothed_rtt == 0:
            smoothed_rtt = rtt_stats.SmoothedOrInitialRtt()

        if smoothed_rtt != 0:
            srtt_history.push({time: event_time, rtt_ms: smoothed_rtt_ms})

        ack_window_history.push({
            time: event_time,
            acked_bytes: acked_bytes,
            has_loss: lost_packets is not empty
        })

        trim all histories to kMaxHistorySamples

    last_ack_time = event_time

    if in_cruise and BBR_mode == PROBE_BW and phase == PROBE_CRUISE:
        run_due_cruise_window_analysis(event_time)
```

注意：`ComputeCongestionScore()` 会基于 ACK 字节数、ECN 和 loss 计算拥塞分数，但当前 `AnalyzeCruiseWindow()` 没有使用这个分数参与候选判定或输出。

## Cruise 窗口调度

cruise 窗口不是从 `cruise_start_time` 立即开始，而是从 `cruise_start_time + minRTT` 开始，这相当于把发送端扰动和接收端观测做 RTT 对齐。

```text
reset_cruise_window_state():
    min_rtt = model.MinRtt()
    if min_rtt == 0:
        min_rtt = rtt_stats.MinOrInitialRtt()

    if min_rtt != 0 and cruise_start_time != ZERO:
        next_cruise_window_start = cruise_start_time + min_rtt
    else:
        next_cruise_window_start = ZERO
```

```text
run_due_cruise_window_analysis(now):
    if configured_modulation_freq_hz <= 0 or cruise_start_time == ZERO:
        return

    min_rtt = model.MinRtt()
    if min_rtt == 0:
        min_rtt = rtt_stats.MinOrInitialRtt()

    if min_rtt == 0:
        log warning once
        return

    if next_cruise_window_start == ZERO:
        next_cruise_window_start = cruise_start_time + min_rtt

    min_rtt_s = min_rtt in seconds

    # 窗口长度取两者最大值：
    # 1. 至少覆盖 min_cruise_cycles_per_window 个扰动周期
    # 2. 至少覆盖 2 个 minRTT
    window_duration_s = max(
        min_cruise_cycles_per_window / configured_modulation_freq_hz,
        2 * min_rtt_s
    )

    window_duration = TimeDelta(window_duration_s)

    # 滑动步长为窗口长度的一定比例，默认 0.25。
    # 至少 1ms，避免 step 过小或为 0。
    step = cruise_window_step_ratio * window_duration
    step = max(step, 1ms)

    while next_cruise_window_start + window_duration <= now
          and in_cruise:
        analyze_cruise_window(
            window_start = next_cruise_window_start,
            window_end = next_cruise_window_start + window_duration,
            min_rtt = min_rtt,
            window_duration_s = window_duration_s
        )

        next_cruise_window_start += step
```

默认配置：

- `min_cruise_cycles_per_window = 4.0`
- `cruise_window_step_ratio = 0.25`
- `kDefaultOscillationFreqHz = 1.0 Hz`
- `kSampleStepSec = 0.001 s`
- `kMaxHistorySamples = 20000`

## 单个 Cruise 窗口分析

### 选择样本并做 RTT 对齐

```text
analyze_cruise_window(window_start, window_end, min_rtt, window_duration_s):
    reference_freq = configured_modulation_freq_hz

    # 频率容忍度取两者最大值：
    # 1. reference_freq 的固定比例，默认 20%
    # 2. 由窗口长度决定的最低频率分辨能力 2 / window_duration_s
    freq_tolerance = max(
        freq_tolerance_ratio * reference_freq,
        2 / window_duration_s
    )

    # 发送端速率窗口向前平移 minRTT，
    # 用发送扰动 [window_start - minRTT, window_end - minRTT]
    # 对齐接收/RTT 响应 [window_start, window_end]。
    sender_start = window_start - min_rtt
    sender_end = window_end - min_rtt

    srate_samples = select(sender_rate_history, sender_start, sender_end)
    drate_samples = select(delivery_rate_history, window_start, window_end)
    srtt_samples = select(srtt_history, window_start, window_end)

    srate = analyze_rate_series(srate_samples, sender_start, sender_end,
                                reference_freq, detrend=false)
    drate = analyze_rate_series(drate_samples, window_start, window_end,
                                reference_freq, detrend=false)
    srtt = analyze_rtt_series(srtt_samples, window_start, window_end,
                              reference_freq, detrend=true)
```

变量含义：

- `srate`：sender rate，发送端实际 pacing rate 序列，包含三角波扰动。
- `drate`：delivery/receive rate，ACK 事件处记录的接收/交付速率信号。
- `srtt`：smoothed RTT，平滑 RTT 信号。分析前会做线性去趋势，降低慢变化 RTT 基线对频谱的影响。

### 信号重采样与频谱

`AnalyzeRateSeries()` / `AnalyzeRttSeries()` 的内部流程一致：

```text
analyze_series(samples, start, end, reference_freq, detrend):
    values = resample_to_fixed_step(samples, start, end, step = 1ms)

    if values is empty:
        return invalid result

    mean_value = average(values)

    if detrend:
        # 减去首尾连线形成的线性趋势。
        for each point i:
            values[i] -= linear_interpolation(values.first, values.last, i)

    profile = build_spectrum_profile(values, sample_step = 1ms,
                                     ref_freq = reference_freq)

    return {profile, mean_value, values, valid = profile.valid}
```

`BuildSpectrumProfile()` 主要做：

```text
build_spectrum_profile(values, sample_step, ref_freq):
    if values too short or ref_freq <= 0:
        return invalid profile

    # 1. 去均值 + Hann 窗
    # 2. 以 kFftZeroPadMultiplier=4 做 zero-padding FFT
    # 3. 计算每个频点 magnitude
    # 4. 只关注 [0.70 * ref_freq, 1.30 * ref_freq] 频带
    # 5. 在该频带内找最大峰值，并用邻近频点做抛物线插值细化 peak_freq_hz
    # 6. 用频带外 magnitude 的中位数估计 noise_floor
    # 7. 把目标频带重采样成 16 个归一化 shape bin，用于形状距离比较

    return SpectrumProfile:
        peak_freq_hz       # 目标频带内的峰值频率
        target_amp         # 目标频带内峰值 magnitude
        noise_floor        # 频带外噪声底，中位数
        noise_floor_valid  # noise_floor 是否有效
        band_energy_rel    # 目标频带能量 / 全频能量
        band_peak_rel      # 目标频带峰值 / 全频能量
        band_shape         # 目标频带形状，长度 16，归一化
        valid
```

### 频率评分

```text
freq_score = clamp01(
    1 - abs(peak_freq_hz - reference_freq) / freq_tolerance
)

srate_freq_score = score(srate_peak_freq_hz)
drate_freq_score = score(drate_peak_freq_hz)
srtt_freq_score = score(srtt_peak_freq_hz)

# full-load 判断真正使用 drate 和 srtt 的频率响应。
freq_quality = 0.5 * drate_freq_score + 0.5 * srtt_freq_score
```

相关阈值：

- `kMinDrateFreqScoreForCandidate = 0.60`
- `kMinSrttFreqScoreForCandidate = 0.60`
- `freq_tolerance_ratio = 0.20`

### 幅度评分

```text
srate_target_amp = srate.profile.target_amp if valid else 0
drate_target_amp = drate.profile.target_amp if valid else 0

# 接收/交付速率响应相对发送速率扰动的增益。
drate_gain = drate_target_amp / max(srate_target_amp, epsilon)

# drate_gain 达到 kTargetDrateGain=0.30 时得满分。
drate_amplitude_score = clamp01(drate_gain / 0.30)

srtt_target_amp = srtt.profile.target_amp if valid else 0
srtt_noise_floor = srtt.profile.noise_floor

if srtt noise floor valid:
    srtt_snr = srtt_target_amp / max(srtt_noise_floor, epsilon)

    # SRTT SNR <= 1.5 记 0 分，>= 3.0 记满分，中间线性映射。
    srtt_amplitude_score = score_threshold(srtt_snr,
                                           min = 1.50,
                                           target = 3.00)
else:
    srtt_snr = 0
    srtt_amplitude_score = 0.5

amplitude_quality = 0.4 * drate_amplitude_score
                  + 0.6 * srtt_amplitude_score
```

### 波形评分

drate 波形质量来自 `drate` 与 `srate` 在目标频带内的归一化频谱形状距离。

```text
if drate.valid and srate.valid:
    drate_shape_distance = sum(abs(drate.band_shape[i] - srate.band_shape[i]))

    # shape distance 为 0 表示形状完全一致；
    # 达到 kMaxDrateShapeDistance=0.40 时降为 0 分。
    drate_waveform_quality = clamp01(
        1 - drate_shape_distance / 0.40
    )
else:
    drate_waveform_quality = 0.5
```

SRTT 波形质量来自 `AnalyzeCycleQuality()`，它按参考频率切分周期，检查每个周期的峰值位置、削顶、底部削平、不完整性和非对称性。

```text
analyze_cycle_quality(srtt_values, sample_step = 1ms, ref_freq):
    samples_per_cycle = round((1 / ref_freq) / sample_step)
    cycle_count = len(values) / samples_per_cycle

    for each complete cycle:
        cycle_min, cycle_max = minmax(cycle)
        amplitude = cycle_max - cycle_min

        if amplitude too small:
            mark cycle incomplete
            continue

        valid_cycles += 1

        peak_index = index of cycle_max within this cycle
        phase_offset = peak_index / samples_per_cycle

        top_clip_ratio += fraction of samples near top with near-zero slope
        bottom_clip_ratio += fraction of samples near bottom with near-zero slope
        incompleteness += whether both positive and negative slope exist
        asymmetry += difference between first-half and second-half absolute movement

    if enough phase offsets:
        cycle_phase_stability = 1 - phase_std / kMaxPhaseStdCycles
    else:
        cycle_phase_stability = 0.5

    if enough peak indices:
        estimate frequency from peak-to-peak distance
        cycle_frequency_stability = average freq_score(estimated_freq)
    else if valid_cycles >= 2:
        cycle_frequency_stability = 1.0
    else:
        cycle_frequency_stability = 0.5

    distortion_score = clamp01(
        0.25 * top_clip_ratio
      + 0.25 * bottom_clip_ratio
      + 0.20 * cycle_incompleteness
      + 0.15 * cycle_asymmetry
      + 0.15 * phase_instability
    )

    waveform_quality = 1 - distortion_score
```

窗口最终波形质量：

```text
waveform_quality = 0.4 * drate_waveform_quality
                 + 0.6 * srtt_waveform_quality
```

### 一致性评分

```text
consistency_quality = 0.5 * cycle_frequency_stability
                    + 0.5 * cycle_phase_stability
```

这里的一致性只来自 SRTT 周期分析。若 SRTT 相位不可靠，`cycle_phase_stability` 会回退为 `0.5`。

### Full-load 候选判定

```text
is_full_load_candidate =
    drate_valid
    and srtt_valid
    and drate_freq_score >= 0.60
    and srtt_freq_score >= 0.60

full_load_quality = clamp01(
    0.35 * freq_quality
  + 0.25 * waveform_quality
  + 0.20 * amplitude_quality
  + 0.20 * consistency_quality
)

label = "FULL_LOAD_CANDIDATE" if is_full_load_candidate
        else "NOT_FULL_LOAD_CANDIDATE"
```

解释：

- 是否成为候选窗口，只看 `drate` 和 `srtt` 是否都有效且频率命中配置扰动频率。
- `full_load_quality` 是候选窗口排序和可信度判断用的综合质量分；即使不是候选窗口，也会计算这个分数并输出。

### 低可信度标记

```text
frequency_match =
    drate_freq_score >= 0.60
    and srtt_freq_score >= 0.60

srate_unstable =
    not srate_valid
    or srate_freq_score < 0.60

samples_insufficient =
    drate_sample_count < 4
    or srtt_sample_count < 4

expected_cycles = floor(window_duration_s * reference_freq)

cycles_insufficient =
    expected_cycles < 2
    or valid_cycle_count < 2

low_confidence =
    frequency_match
    and (
        srate_unstable
        or samples_insufficient
        or cycles_insufficient
        or srtt_noise_floor_invalid
        or full_load_quality < min_full_load_quality_for_reliable_window
    )
```

默认 `min_full_load_quality_for_reliable_window = 0.50`。

这里有一个细节：`low_confidence` 只有在 `frequency_match` 为真时才可能为真。如果频率都没有匹配，则窗口本来就不是可靠候选，标签会是 `NOT_FULL_LOAD_CANDIDATE`，但 `low_confidence` 不一定置真。

## Cruise 结束时排序与输出

窗口分析结果先缓存在 `current_cruise_windows`，不会立即写 CSV。只有 `LeaveCruise()` 调用 `FinalizeCruise()` 时才统一输出。

```text
finalize_cruise(now):
    candidate_indices = []

    for each window in current_cruise_windows:
        if window.is_full_load_candidate:
            candidate_indices.push(window.index)

    sort candidate_indices by full_load_quality descending

    for rank, index in sorted candidate_indices:
        window.full_load_rank_in_cruise = rank + 1
        window.is_best_full_load_window = (rank == 0)

    for each window in current_cruise_windows:
        emit_cruise_window_trace(window)

    emit_cruise_summary_trace(now)
```

## 输出说明

FreqCCv4 的 cruise 输出通过 `SetCruiseLoadTraceCallback()` 传到 `DqcTrace::OnFreqCCv4Load()`，最终按 label 分到两个 CSV。

### 窗口级输出

文件名格式：

```text
<trace_name>_cruise_full_load_quality.csv
```

每一行对应一个已分析的滑动窗口。

| 字段 | 含义 |
| --- | --- |
| `cruise_id` | 第几次进入 cruise，从 1 递增。 |
| `window_start_time` | 窗口开始时间，单位秒。 |
| `window_end_time` | 窗口结束时间，单位秒。 |
| `configured_modulation_freq_hz` | 本窗口使用的配置扰动频率。 |
| `srate_peak_freq_hz` | 发送速率在目标频带内的峰值频率。 |
| `drate_peak_freq_hz` | 接收/交付速率在目标频带内的峰值频率。 |
| `srtt_peak_freq_hz` | SRTT 在目标频带内的峰值频率。 |
| `drate_freq_score` | drate 峰值频率与配置频率的匹配分。 |
| `srtt_freq_score` | SRTT 峰值频率与配置频率的匹配分。 |
| `freq_quality` | `0.5 * drate_freq_score + 0.5 * srtt_freq_score`。 |
| `drate_target_amp` | drate 在目标频带内的峰值幅度。 |
| `srate_target_amp` | srate 在目标频带内的峰值幅度。 |
| `drate_gain` | `drate_target_amp / srate_target_amp`，反映发送扰动传到接收速率的强度。 |
| `drate_amplitude_score` | drate 幅度分，`drate_gain >= 0.30` 时满分。 |
| `srtt_target_amp` | SRTT 在目标频带内的峰值幅度。 |
| `srtt_noise_floor` | SRTT 目标频带外的噪声底估计。 |
| `srtt_snr` | `srtt_target_amp / srtt_noise_floor`。 |
| `srtt_amplitude_score` | SRTT 幅度分，SNR 从 1.5 到 3.0 线性映射到 0 到 1。 |
| `drate_waveform_quality` | drate 与 srate 的目标频带形状相似度。 |
| `srtt_waveform_quality` | SRTT 周期波形质量，削顶/不完整/非对称越少越高。 |
| `waveform_quality` | `0.4 * drate_waveform_quality + 0.6 * srtt_waveform_quality`。 |
| `cycle_frequency_stability` | SRTT 周期峰间距推算出的频率稳定性。 |
| `cycle_phase_stability` | SRTT 周期峰值相位稳定性。 |
| `consistency_quality` | `0.5 * cycle_frequency_stability + 0.5 * cycle_phase_stability`。 |
| `srtt_top_clip_ratio` | SRTT 周期顶部削平比例。 |
| `srtt_bottom_clip_ratio` | SRTT 周期底部削平比例。 |
| `srtt_distortion_score` | SRTT 波形失真分，越高表示失真越大。 |
| `is_full_load_candidate` | 是否为 full-load 候选窗口。 |
| `full_load_quality` | 综合质量分，用于候选排序。 |
| `full_load_rank_in_cruise` | 候选窗口在本轮 cruise 内按质量排序的名次；非候选为 `-1`。 |
| `is_best_full_load_window` | 是否为本轮 cruise 内质量最高的候选窗口。 |
| `low_confidence` | 频率匹配但样本、周期、发送扰动、噪声底或综合质量存在可信度问题。 |
| `label` | `FULL_LOAD_CANDIDATE` 或 `NOT_FULL_LOAD_CANDIDATE`。 |

### Cruise 汇总输出

文件名格式：

```text
<trace_name>_cruise_best_full_load_window.csv
```

每一行对应一次 cruise。

| 字段 | 含义 |
| --- | --- |
| `cruise_id` | 第几次进入 cruise。 |
| `cruise_start_time` | 本轮 cruise 开始时间，单位秒。 |
| `cruise_end_time` | 本轮 cruise 结束时间，单位秒。 |
| `candidate_count` | 本轮 cruise 中 full-load 候选窗口数量。 |
| `best_full_load_window_exists` | 是否存在至少一个候选窗口。 |
| `best_window_start_time` | 最佳候选窗口开始时间；不存在时为 `-1`。 |
| `best_window_end_time` | 最佳候选窗口结束时间；不存在时为 `-1`。 |
| `best_full_load_quality` | 最佳候选窗口的综合质量分；不存在时为 `0`。 |
| `best_drate_freq_score` | 最佳候选窗口的 drate 频率匹配分。 |
| `best_srtt_freq_score` | 最佳候选窗口的 SRTT 频率匹配分。 |
| `best_srtt_waveform_quality` | 最佳候选窗口的 SRTT 波形质量。 |
| `best_drate_amplitude_score` | 最佳候选窗口的 drate 幅度分。 |
| `best_srtt_amplitude_score` | 最佳候选窗口的 SRTT 幅度分。 |
| `best_drate_mean_kbps` | 最佳候选窗口内 delivery rate 的平均值，单位 Kbit/s；不存在候选窗口时为 `0`。 |
| `cruise_end_max_bandwidth_kbps` | 当前 cruise 结束时该流的 BBR/FreqCCv4 最大带宽估计，单位 Kbit/s。 |
| `fair_share_bandwidth_kbps` | 该实验注入的每流公平带宽份额，单位 Kbit/s；在 `freqccv4_4flow.cc` 中为 `TOPO_BOTTLE_BW / NUM_FLOWS`。 |

## 关键变量解释

| 变量 | 含义 |
| --- | --- |
| `configured_modulation_freq_hz_` | 外部配置的扰动频率，默认 `1.0 Hz`。 |
| `cruise_modulation_freq_hz_` | 当前 cruise 固定使用的扰动频率，进入 cruise 时从配置值复制。 |
| `amplitude_mode_` | 扰动幅度模式，支持固定值、按 `BandwidthEstimate()` 比例、按 BBRv2 base pacing rate 比例。 |
| `fixed_amplitude_bps_` | 固定幅度模式下的 bit/s 幅度。 |
| `drain_completed_` | 是否已经从启动/排空进入过 `PROBE_BW`，未完成前不启用扰动。 |
| `in_cruise_` | FreqCCv4 自己记录的当前是否处于 cruise。 |
| `cruise_start_time_` | 当前 cruise 的开始时间。 |
| `next_cruise_window_start_` | 下一个待分析滑动窗口的开始时间。 |
| `current_time_` | 最近一次发包或 ACK/loss 事件时间，`PacingRate()` 用它计算三角波相位。 |
| `last_ack_time_` | 上一个 ACK/loss 事件时间，用于过滤无效或非递增 ACK 样本。 |
| `use_delivery_rate_latest_for_signal_history_` | 选择接收速率信号来源：`DeliveryRateLatest()` 或 `BandwidthLatest()`。 |
| `min_rtt_warning_logged_` | 本轮 cruise 是否已经打印过 minRTT 缺失告警。 |
| `cruise_id_` | cruise 轮次编号。 |
| `min_cruise_cycles_per_window_` | 一个分析窗口至少包含多少个扰动周期，默认 `4.0`。 |
| `cruise_window_step_ratio_` | 滑动步长占窗口长度的比例，默认 `0.25`。 |
| `freq_tolerance_ratio_` | 峰值频率匹配容忍比例，默认 `0.20`。 |
| `min_full_load_quality_for_reliable_window_` | 可信窗口最低综合质量，默认 `0.50`。 |
| `sender_rate_history_` | 发包时记录的 pacing rate 历史。 |
| `delivery_rate_history_` | ACK 事件时记录的接收/交付速率历史。 |
| `srtt_history_` | ACK 事件时记录的 smoothed RTT 历史。 |
| `ack_window_history_` | ACK 字节数和 loss 标记历史；当前 cruise 判定未使用。 |
| `current_cruise_windows_` | 当前 cruise 中已经分析出的窗口结果缓存。 |
| `cruise_load_trace_cb_` | cruise 窗口和汇总结果的 trace 回调。 |

## 读结果时的建议

1. 先看 `<trace_name>_cruise_best_full_load_window.csv`，判断每次 cruise 是否找到候选窗口，以及最佳窗口的时间段。
2. 再到 `<trace_name>_cruise_full_load_quality.csv` 按 `cruise_id` 和 `full_load_rank_in_cruise` 查看候选窗口排序。
3. 如果 `is_full_load_candidate=true` 但 `low_confidence=true`，优先检查 `srate_freq_score`、样本数量、`valid_cycle_count`、`srtt_noise_floor` 和 `full_load_quality`。
4. 如果没有候选窗口，重点检查 `drate_freq_score` 与 `srtt_freq_score` 是否低于 `0.60`，这说明接收速率或 SRTT 没有稳定跟随配置扰动频率。
