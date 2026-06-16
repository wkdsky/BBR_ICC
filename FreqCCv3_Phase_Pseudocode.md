# FreqCCv3 阶段处理伪代码

本文只整理当前 `FreqCCv3Sender` 在线实现的行为，不包含离线评估脚本。源码入口主要是：

- `NS3.27/src/dqc/model/thirdparty/congestion/freqccv3_sender.h`
- `NS3.27/src/dqc/model/thirdparty/congestion/freqccv3_sender.cc`

`FreqCCv3Sender` 继承 `Bbr2Sender`。大部分 BBRv2 状态机仍由父类完成，`FreqCCv3` 主要在三个位置追加逻辑：

1. `OnPacketSent()`：记录发送端实际 pacing rate 序列。
2. `OnCongestionEvent()`：先让父类推进 BBRv2 状态机，再根据阶段切换做 `NEW_REFILL`、`PROBE_UP` 振荡状态、频域分析和自适应参数更新。
3. `PacingRate()` / `GetCongestionWindow()`：在 `PROBE_UP` 加三角波速率扰动，在 `NEW_REFILL` 限制 cwnd。

## 阶段编号

`GetCurrentBbrModeIndex()` 把 BBRv2 模式映射为 trace 编号：

```text
0 = STARTUP
1 = DRAIN
2 = PROBE_BW_DOWN
3 = PROBE_BW_CRUISE
4 = PROBE_BW_REFILL, FreqCCv3 中作为 NEW_REFILL 处理
5 = PROBE_BW_UP
6 = PROBE_RTT
```

## 关键状态

```text
oscillation_freq_hz              当前准备给下一次 PROBE_UP 使用的振荡频率
current_oscillation_freq_hz      当前或刚结束的 PROBE_UP 实际使用的频率
amplitude_mode                   振幅模式，fixed / miuN / srN
fixed_amplitude_bps              fixed 模式下的振幅

drain_completed                  是否已经进入过 PROBE_BW
oscillation_start_time           当前 PROBE_UP 振荡起点
up_phase_start_time              当前 PROBE_UP 起点
last_probe_bw_phase              上一次 ACK 后记录的 ProbeBW 子阶段

in_new_refill                    当前是否处于 FreqCCv3 的 NEW_REFILL 包装状态
new_refill_state                 kDraining / kFilling / kDone / kNotInNewRefill
interval_window_multiplier       INT 阶段频域窗口 = multiplier * min_rtt
min_probe_up_duration_rtt_multiplier
                                  可选实验门控，限制 PROBE_UP 至少停留若干 min_rtt

up_pacing_gain                   下一次 PROBE_UP 使用的 pacing gain
current_up_pacing_gain           当前或刚结束的 PROBE_UP 实际使用的 gain

signal_history                   ACK 驱动的接收速率序列，BandwidthLatest 或 DeliveryRateLatest
sender_rate_history              发包时记录的实际发送速率序列，包含振荡
rtt_signal_history               ACK 时刻采样的 smoothed RTT 序列

last_down_phase_end_time         上一次 PROBE_DOWN 结束时间，后续 INT = CRUISE + REFILL 起点
last_up_phase_valid              上一次 PROBE_UP 是否至少包含 kMinCyclesPerUp 个振荡周期
last_up_sender_band_template     上一次 PROBE_UP 的发送端带内频谱模板
```

## 总体事件流

```text
OnPacketSent(sent_time, bytes_in_flight, packet):
    current_time = sent_time

    sender_rate = PacingRate(bytes_in_flight)
    sender_rate_history.push(sent_time, sender_rate)
    trim sender_rate_history to at most 20000 samples

    last_packet_sent_time = sent_time

    call Bbr2Sender::OnPacketSent(...)
```

```text
OnCongestionEvent(event_time, acked_packets, lost_packets):
    current_time = event_time
    acked_bytes = sum(acked_packets.bytes_acked)

    phase_before = GetCurrentProbeBwPhase()
    in_probe_up_before = phase_before == PROBE_UP
    has_loss_or_ecn = lost_packets not empty OR GetBytesEcnInRounds() > 0

    call Bbr2Sender::OnCongestionEvent(...)
        父类在这里更新 BBRv2 模式、ProbeBW 子阶段、带宽模型、RTT、loss/ECN 状态等

    if this ACK can form a receive-rate sample:
        recv_signal = DeliveryRateLatest() if configured else BandwidthLatest()
        signal_history.push(event_time, recv_signal)
        rtt_signal_history.push(event_time, smoothed_rtt_ms)
        trim both histories to at most 20000 samples

    current_mode = mode_
    current_phase = GetCurrentProbeBwPhase()

    if first time reaching PROBE_BW:
        drain_completed = true

    if current_mode == PROBE_BW:
        handle ProbeBW phase hooks:
            record DOWN end
            handle NEW_REFILL
            handle entering PROBE_UP
            handle leaving PROBE_UP

    last_mode = current_mode
    last_probe_bw_phase = current_phase
```

## STARTUP

```text
阶段推进:
    由 Bbr2Sender 父类处理

FreqCCv3 追加逻辑:
    不振荡
    不进入 NEW_REFILL
    不做 UP/INT 频域配对
    OnPacketSent 仍会记录 sender_rate_history
    OnCongestionEvent 仍可能记录 signal_history / rtt_signal_history

PacingRate:
    return Bbr2Sender::PacingRate()
```

## DRAIN

```text
阶段推进:
    由 Bbr2Sender 父类处理

FreqCCv3 追加逻辑:
    drain_completed 仍为 false
    不振荡
    不进入 NEW_REFILL
    不做 UP/INT 频域配对

当父类从 DRAIN 进入 PROBE_BW:
    drain_completed = true
```

`drain_completed` 是振荡门控之一。即使当前阶段名满足 `PROBE_UP`，也必须已经进入过 `PROBE_BW` 后才允许振荡。

## PROBE_BW_DOWN

```text
阶段推进:
    由 Bbr2Sender 父类处理

FreqCCv3 追加逻辑:
    if phase_before == PROBE_DOWN and current_phase != PROBE_DOWN:
        last_down_phase_end_time = event_time

含义:
    后续 INT 分析窗口从 DOWN 结束开始
    INT = CRUISE + REFILL, 直到下一次进入 PROBE_UP

PacingRate:
    不振荡
    return Bbr2Sender::PacingRate()
```

## PROBE_BW_CRUISE

```text
阶段推进:
    由 Bbr2Sender 父类处理

FreqCCv3 追加逻辑:
    持续积累 signal_history 和 rtt_signal_history
    不振荡
    等待父类从 CRUISE 进入 REFILL

当 current_phase == PROBE_REFILL 且 phase_before == PROBE_CRUISE:
    进入 FreqCCv3 NEW_REFILL 包装逻辑
```

## PROBE_BW_REFILL, 即 NEW_REFILL

`FreqCCv3` 没有新增父类状态枚举，而是在父类 `PROBE_REFILL` 内部维护 `in_new_refill` 和 `new_refill_state`。

```text
Enter NEW_REFILL:
    in_new_refill = true
    UpdateNewRefillState(prior_in_flight)
    model.set_pacing_gain(GetNewRefillPacingGain())
```

```text
UpdateNewRefillState(bytes_in_flight):
    high = 0.75 * BDP + 2 * MSS + MaxAckHeight
    low  = 0.72 * BDP + 2 * MSS + MaxAckHeight

    if bytes_in_flight > high:
        new_refill_state = kDraining
    else if bytes_in_flight < low:
        new_refill_state = kFilling
    else:
        new_refill_state = kDone
```

```text
ShouldExitNewRefill(bytes_in_flight):
    if not in_new_refill:
        return false

    high = 0.75 * BDP + 2 * MSS + MaxAckHeight

    switch new_refill_state:
        kDraining:
            return bytes_in_flight <= high
        kFilling:
            return bytes_in_flight >= low
        kDone:
            return true
        kNotInNewRefill:
            return true
```

```text
While in NEW_REFILL and parent phase is PROBE_REFILL:
    current_inflight = prior_in_flight

    if ShouldExitNewRefill(current_inflight):
        in_new_refill = false
        new_refill_state = kNotInNewRefill
        父类 BBRv2 后续在 refill round 结束时进入 PROBE_UP
    else:
        model.set_pacing_gain(GetNewRefillPacingGain())
```

当前代码里有一个实现细节需要注意：

```text
GetNewRefillPacingGain():
    return 1.0
```

也就是说，虽然头文件注释和 `NewRefillState::kDraining` 注释提到 draining gain 0.75，但当前实际代码总是给 NEW_REFILL 返回 `1.0`。实际 drain/fill 控制主要依赖 `GetCongestionWindow()` 的 cap：

```text
GetCongestionWindow():
    cwnd = cwnd_

    if in_new_refill:
        cap = 0.75 * BDP + 2 * MSS + MaxAckHeight
        cwnd = min(cwnd, cap)

    return cwnd
```

## 进入 PROBE_BW_UP

进入 `PROBE_UP` 的检测不是看 `phase_before`，而是比较本次阶段和 `last_probe_bw_phase`：

```text
in_probe_up = current_phase == PROBE_UP
was_in_probe_up_before = last_probe_bw_phase == PROBE_UP
entering_probe_up = in_probe_up and not was_in_probe_up_before
```

进入时先处理上一轮 INT 窗口：

```text
if entering_probe_up:
    if last_up_phase_valid and last_down_phase_end_time exists:
        interval_start = last_down_phase_end_time
        interval_end = event_time

        if last_up_sender_template_valid:
            interval_ref_freq = last_up_sender_template_freq_hz
            interval_threshold = 0
        else if last_up_phase_peak_freq > 0:
            interval_ref_freq = last_up_phase_peak_freq
            interval_threshold = last_up_phase_peak_freq * 2 / 3
        else:
            interval_ref_freq = 0
            interval_threshold = 0

        PerformFreqAnalysis(interval_start, interval_end,
                            interval_threshold, interval_ref_freq)

        if last_up_sender_template_valid:
            rtt_ref_freq = last_up_sender_template_freq_hz
            rtt_threshold = 0
        else if last_up_rtt_peak_freq > 0:
            rtt_ref_freq = last_up_rtt_peak_freq
            rtt_threshold = last_up_rtt_peak_freq * 2 / 3
        else:
            rtt_ref_freq = 0
            rtt_threshold = 0

        PerformRttFreqAnalysis(interval_start, interval_end,
                               rtt_threshold, rtt_ref_freq)

    clear signal_history and rtt_signal_history older than event_time
```

然后初始化当前 UP：

```text
    oscillation_start_time = event_time
    up_phase_start_time = event_time
    last_up_phase_start_time = event_time

    last_up_phase_peak_freq = 0
    last_up_rtt_peak_freq = 0
    last_up_phase_valid = false
    sender_max_peak_freq_hz = 0
    last_up_sender_template = invalid / empty

    bandwidth_before_up = model.MaxBandwidth()
    up_phase_exited_early = false

    current_oscillation_freq_hz = oscillation_freq_hz
    current_up_pacing_gain = up_pacing_gain

    model.set_pacing_gain(up_pacing_gain)

    in_new_refill = false
    new_refill_state = kNotInNewRefill
```

## PROBE_BW_UP 内部发包速率

`FreqCCv3` 只在 `PROBE_UP` 中注入振荡。

```text
ShouldOscillate():
    if GetCurrentAmplitudeBps() == 0:
        return false
    if not drain_completed:
        return false
    if mode_ != PROBE_BW:
        return false
    return GetCurrentProbeBwPhase() == PROBE_UP
```

振幅计算：

```text
GetCurrentAmplitudeBps():
    max_bw = BandwidthEstimate()
    base_rate = Bbr2Sender::PacingRate(0)

    fixed: return fixed_amplitude_bps
    miu2:  return max_bw / 2
    miu3:  return max_bw / 3
    miu4:  return max_bw / 4
    miu8:  return max_bw / 8
    sr2:   return base_rate / 2
    sr3:   return base_rate / 3
    sr4:   return base_rate / 4
    sr8:   return base_rate / 8
```

实际 pacing rate：

```text
PacingRate(bytes_in_flight):
    base_rate = Bbr2Sender::PacingRate(bytes_in_flight)

    if not ShouldOscillate():
        return base_rate

    if oscillation_start_time is zero:
        return base_rate

    offset = CalculateOscillationOffset(current_time)
    final_bps = base_rate.bps + offset
    final_bps = max(final_bps, 1000)
    return final_bps
```

三角波 offset：

```text
CalculateOscillationOffset(now):
    elapsed = now - oscillation_start_time
    period = 1 / oscillation_freq_hz
    phase = (elapsed mod period) / period

    if phase < 0.25:
        triangle = phase * 4              # 0 -> 1
    else if phase < 0.75:
        triangle = 2 - phase * 4          # 1 -> -1
    else:
        triangle = phase * 4 - 4          # -1 -> 0

    return triangle * GetCurrentAmplitudeBps()
```

## 离开 PROBE_BW_UP

离开检测：

```text
leaving_probe_up = not in_probe_up and was_in_probe_up_before
need_cleanup = not in_probe_up and up_phase_start_time exists

if leaving_probe_up or need_cleanup:
    if in_probe_up_before and has_loss_or_ecn and not in_probe_up:
        up_phase_exited_early = true

    up_duration = event_time - up_phase_start_time
    last_up_duration_sec = up_duration.seconds
    last_up_phase_end_time = event_time
    cycles_in_up = last_up_duration_sec * current_oscillation_freq_hz
```

如果 UP 足够长：

```text
if cycles_in_up >= kMinCyclesPerUp:
    last_up_phase_valid = true

    sender_samples = sender_rate_history samples in [up_start, up_end]

    if sender_samples not empty:
        sender_window = CalculateSTFTWindowSize(current_oscillation_freq_hz)
        sender_window = min(sender_window, up_duration)
        step = max(sender_window * 0.1, 1ms)

        for each sliding sender window:
            result = AnalyzeWindow(window_samples,
                                   sender_window,
                                   current_oscillation_freq_hz)
            if result.valid:
                sender_peak_freqs.append(result.peak_freq_hz)

        sender_max_peak_freq_hz = max(sender_peak_freqs) or 0
        sender_ref_freq = sender_max_peak_freq_hz or current_oscillation_freq_hz

        CaptureSenderSpectrumTemplate(sender_samples,
                                      up_duration,
                                      sender_ref_freq)

        if captured template has better peak:
            sender_ref_freq = last_up_sender_template_freq_hz
            sender_max_peak_freq_hz = sender_ref_freq if previously zero

        temporarily disable template matching for current UP analysis
        PerformFreqAnalysis(up_start, up_end, 0, sender_ref_freq)
        PerformRttFreqAnalysis(up_start, up_end, 0, sender_ref_freq)
        restore template validity

    else:
        sender_max_peak_freq_hz = 0
        PerformFreqAnalysis(up_start, up_end, 0, current_oscillation_freq_hz)
        PerformRttFreqAnalysis(up_start, up_end, 0, current_oscillation_freq_hz)

    clear sender_rate_history and rtt_signal_history older than event_time
```

如果 UP 太短：

```text
else:
    last_up_phase_valid = false
    sender_max_peak_freq_hz = 0
    clear signal_history, rtt_signal_history, sender_rate_history older than event_time
```

之后更新下一轮参数：

```text
if last_up_duration_sec > 1ms:
    min_rtt_sec = model.MinRtt()
    dynamic_min_freq = max(50Hz, 4 / min_rtt_sec) if min_rtt known else 50Hz

    if cycles_in_up > kMaxCyclesPerUp:
        new_freq = kTargetCyclesPerUp / last_up_duration_sec
        oscillation_freq_hz = clamp(new_freq, dynamic_min_freq, 100Hz)

    up_duration_rtt_multiple = last_up_duration_sec / min_rtt_sec

    if up_duration_rtt_multiple > 2.5:
        up_pacing_gain = 1.25
    else if 2.0 <= up_duration_rtt_multiple <= 2.5:
        calculated = up_duration_rtt_multiple * 1.25 / 2.5
        incremented = up_pacing_gain + 0.01
        up_pacing_gain = min(1.25, max(incremented, calculated))
    else if 1.5 < up_duration_rtt_multiple < 2.0:
        calculated = up_duration_rtt_multiple * 1.00 / 1.5
        decremented = up_pacing_gain - 0.01
        up_pacing_gain = max(1.00, min(decremented, calculated))
    else:
        up_pacing_gain = 1.00

    emit up_phase_trace if callback exists

up_phase_count += 1
oscillation_start_time = zero
up_phase_start_time = zero
```

## PROBE_RTT

```text
阶段推进:
    由 Bbr2Sender 父类处理

FreqCCv3 追加逻辑:
    不振荡，因为 mode_ != PROBE_BW
    不进入 NEW_REFILL
    OnCongestionEvent 仍会记录 ACK 驱动信号样本

PacingRate:
    return Bbr2Sender::PacingRate()
```

## 可选的 PROBE_UP 最小时长门控

`ShouldDelayProbeUpExit()` 是父类可调用的 override，用于实验性地延迟 `PROBE_UP` 退出。

```text
ShouldDelayProbeUpExit(now):
    if min_probe_up_duration_rtt_multiplier <= 0:
        return false
    if up_phase_start_time is zero:
        return false
    if min_rtt is zero:
        return false

    elapsed = now - up_phase_start_time
    required = min_probe_up_duration_rtt_multiplier * min_rtt
    return elapsed < required
```

## 频域分析伪代码

接收速率和 RTT 的分析流程相同，区别是输入序列与窗口长度。

### 频谱 profile

```text
BuildSpectrumProfile(values, sample_step_s, ref_freq_hz):
    if len(values) < 8 or sample_step_s <= 0 or ref_freq_hz <= 0:
        return invalid

    nfft = len(values) * 4
    x = (values - mean(values)) * HannWindow
    spectrum = abs(RFFT(x, nfft))
    freqs = rfftfreq(nfft, sample_step_s)

    band = [0.70 * ref_freq_hz, 1.30 * ref_freq_hz]
    peak = max spectrum bin inside band

    optionally parabolic-interpolate peak frequency

    band_energy_ratio = sum(band magnitudes) / sum(all nonzero magnitudes)
    band_peak_rel = peak_magnitude / sum(all nonzero magnitudes)

    band_shape = interpolate spectrum into 16 bins over band
    normalize band_shape by sum

    valid = band_peak_rel >= 0.2 and band_shape not empty
    return peak_freq, band_energy_ratio, band_peak_rel, band_shape, valid
```

### 单窗口分析

```text
AnalyzeWindow(samples, window_duration, expected_freq_hz):
    avg_rate = mean(samples.rate)
    ref_freq = expected_freq_hz

    if ref_freq <= 0 and last_up_sender_template_valid:
        ref_freq = last_up_sender_template_freq_hz
    if ref_freq <= 0:
        return invalid

    resample samples to 1ms uniform grid over window_duration
    profile = BuildSpectrumProfile(uniform_values, 1ms, ref_freq)
    if profile invalid:
        return invalid

    valid = true
    shape_distance = 0
    if last_up_sender_template_valid:
        shape_distance = L1(profile.band_shape, last_up_sender_band_template)
        if shape_distance > 0.40:
            valid = false

    return profile.peak_freq_hz, avg_rate, shape_distance, valid
```

`AnalyzeRttWindow()` 同理，只是 `avg_rate` 换成 `avg_rtt_ms`，输入值换成 RTT。

### 接收速率段分析

```text
PerformFreqAnalysis(start_time, end_time, threshold_freq_hz, expected_freq_hz):
    range_samples = signal_history in [start_time, end_time]
    if empty:
        return

    if expected_freq_hz > 0:
        window_size = CalculateSTFTWindowSize(expected_freq_hz)
    else:
        window_size = interval_window_multiplier * min_rtt
        fallback to last_up_window_size or whole duration

    window_size = min(window_size, duration)

    smooth samples using 10% of window_size
    step = max(window_size * 0.1, 1ms)

    for each sliding window:
        result = AnalyzeWindow(window_samples, window_size, expected_freq_hz)
        accept = result.valid
        if threshold_freq_hz > 0 and result.peak_freq_hz < threshold_freq_hz:
            accept = false

        if accept:
            record window start, peak frequency, avg rate

    group consecutive accepted windows into peaks:
        split when frequency jump > 10Hz
        for each group:
            peak_duration = group_end - group_start
            min_duration = 0.75 / expected_freq_hz if expected_freq_hz > 0 else 20ms
            if peak_duration >= min_duration:
                emit freq_analysis_trace(start, duration,
                                         sender_max_peak_freq_hz,
                                         max_receiver_freq,
                                         avg_rate)

    if threshold_freq_hz <= 0:
        update last_up_phase_peak_freq with median detected frequency
```

### RTT 段分析

```text
PerformRttFreqAnalysis(start_time, end_time, threshold_freq_hz, expected_freq_hz):
    range_samples = rtt_signal_history in [start_time, end_time]
    if empty:
        return

    if expected_freq_hz > 0:
        window_size = CalculateRttSTFTWindowSize(expected_freq_hz)
    else:
        window_size = last_up_rtt_window_size
        fallback to CalculateRttSTFTWindowSize(last_up_rtt_peak_freq)
        fallback to 3 * min_rtt or whole duration

    window_size = min(window_size, duration)
    step = max(window_size * 0.1, 5ms)

    for each sliding window:
        result = AnalyzeRttWindow(window_samples, window_size, expected_freq_hz)
        accept = result.valid
        if threshold_freq_hz > 0 and result.peak_freq_hz < threshold_freq_hz:
            accept = false

        if accept:
            record window start, peak frequency, avg rtt

    group consecutive accepted windows into peaks:
        split when frequency jump > 10Hz
        require duration >= 0.75 cycle or 20ms
        emit rtt_freq_analysis_trace(start, duration,
                                     sender_max_peak_freq_hz,
                                     max_rtt_freq,
                                     avg_rtt_ms)

    if threshold_freq_hz <= 0:
        update last_up_rtt_peak_freq with median detected frequency
```

## 一轮 ProbeBW 周期的简化图

```text
父类 BBRv2:
    PROBE_DOWN -> PROBE_CRUISE -> PROBE_REFILL -> PROBE_UP -> PROBE_DOWN ...

FreqCCv3 叠加:
    PROBE_DOWN exits:
        last_down_phase_end_time = now

    PROBE_CRUISE:
        collect recv/rtt signal for future INT

    PROBE_REFILL:
        wrap as NEW_REFILL
        classify inflight as draining/filling/done
        cap cwnd at 0.75 BDP threshold while in_new_refill

    enter PROBE_UP:
        analyze previous INT = CRUISE + REFILL using previous UP template
        reset current UP state
        set pacing_gain = up_pacing_gain
        start triangle-wave oscillation

    during PROBE_UP:
        PacingRate = BBRv2 base pacing rate + triangle_wave_offset
        sender_rate_history records actual oscillating rate

    leave PROBE_UP:
        if enough cycles:
            analyze current UP sender/recv/rtt spectra
            cache sender band-shape template for next INT
        adapt next oscillation frequency if too many cycles occurred
        adapt next UP pacing gain from UP duration in min_rtt units
        stop oscillation
```
