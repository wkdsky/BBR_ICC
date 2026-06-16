# FreqCCv4 Hybrid BtlBw 设计方案

本文档描述一个不依赖真实公平份额的 FreqCCv4 混合瓶颈带宽估计方案。在线控制只能使用发送端可观测信号；`fair_share_bandwidth_kbps` 只能用于离线评估，不参与状态转移或 pacing 控制。

## 1. Motivation and Design Goal

FreqCCv4 当前可以同时观测两类带宽信号：

```text
raw_max_bw_kbps:
    BandwidthEstimate() 的当前输出，也就是 BBR/FreqCCv4 原始最大带宽估计。

best_drate_mean_kbps:
    当前 cruise 中 best full-load window 内 delivery rate 的平均值。
```

实验现象：

- 前几轮 `raw_max_bw_kbps` 可能因为 max filter 保留早期偏高样本而偏高。
- 前几轮可信的 `best_drate_mean_kbps` 往往更接近实际可持续份额。
- 后期 `raw_max_bw_kbps` 经过多轮更新后可能逐渐稳定。
- 后期 `best_drate_mean_kbps` 可能因为 cruise 太短、best window 缺失、动态 delay、频谱误判等原因变得不稳定。

设计目标是构造一个控制用带宽：

```text
effective_btlbw_kbps
```

让它在前期借助可信 `best_drate_mean_kbps` 抑制偏高的 `raw_max_bw_kbps`，在 `raw_max_bw_kbps` 自身稳定且与 `drate_ref_kbps` 收敛后，平滑切回基于 `raw_max_bw_kbps` 的控制。

## 2. Online Signals and Offline-Only Signals

在线算法只能使用本连接可观测信号：

| 信号 | 在线可用 | 说明 |
| --- | --- | --- |
| `raw_max_bw_kbps` | 是 | `BandwidthEstimate()` 的原始输出。 |
| `best_drate_mean_kbps` | 是 | best full-load window 内 delivery rate 均值。 |
| `best_full_load_quality` | 是 | best window 综合质量。 |
| `best_drate_freq_score` | 是 | delivery rate 是否跟随扰动频率。 |
| `best_srtt_freq_score` | 是 | SRTT 是否跟随扰动频率。 |
| `best_window_duration_s` | 是 | best window 覆盖时长。 |
| `best_window_sample_count` | 是 | best window 中 drate/SRTT 样本充分性。 |
| `best_low_confidence` | 是 | 低可信标记，可作为软权重。 |
| `queue_delay_ms` | 是 | 可由 RTT - minRTT 或已有 queue-delay trace 得到。 |
| `loss_rate` | 是 | 本连接 ACK/loss 统计可得。 |
| `ecn_rate` | 是 | ECN 开启时本连接可得。 |
| `fair_share_bandwidth_kbps` | 否 | 真实公平份额不可在线假设，只能离线评估。 |

在线状态机不再判断：

```text
raw_max_bw_kbps 是否接近 fair_share_bandwidth_kbps
```

改为判断：

```text
raw_max_bw_kbps 是否相对自身稳定
drate_ref_kbps 是否由近期可信 best window 支撑
drate_ref_kbps 是否相对自身稳定
maxbw_ref_kbps 是否与 drate_ref_kbps 收敛
当前是否存在排队、loss、ECN 等拥塞保护信号
```

## 3. Definition of best_drate_mean_kbps

`best_drate_mean_kbps` 的来源链路如下：

```text
OnCongestionEvent()
    -> ACK 事件时记录 delivery/receive rate 到 delivery_rate_history_
       默认使用 BandwidthLatest()
       如果 use_delivery_rate_latest_for_signal_history_ 为 true，则使用 DeliveryRateLatest()

AnalyzeCruiseWindow(window_start, window_end)
    -> 选择 delivery_rate_history_ 中 [window_start, window_end] 的 drate samples
    -> ResampleRateSeries(..., sample_step = 1ms)
    -> AnalyzeRateSeries() 计算重采样序列 mean_value
    -> result.drate_mean_kbps = drate.mean_value

FinalizeCruise()
    -> 在本轮 cruise 的 full-load candidates 中选择 full_load_quality 最高的窗口
    -> best_drate_mean_kbps = best->drate_mean_kbps
```

因此它不是瞬时接收速率，也不是最大 delivery rate，而是“本轮 cruise 中最佳 full-load 窗口内，delivery rate 重采样序列的平均值”，单位 Kbit/s。

## 4. State Variables and Parameters

建议在 `FreqCCv4Sender` 中新增：

```cpp
enum class HybridBtlBwSource {
  kDrateBootstrap,
  kBlendToMaxBw,
  kMaxBwSteady,
};

double effective_btlbw_kbps_;
double previous_effective_btlbw_kbps_;
double drate_ref_kbps_;
double maxbw_ref_kbps_;
double previous_raw_max_bw_kbps_;
double previous_drate_ref_kbps_;
int stable_cruise_count_;
int blend_cruise_count_;
int no_best_window_count_;
int maxbw_over_drate_count_;
HybridBtlBwSource hybrid_btlbw_source_;
```

| 变量 | 含义 |
| --- | --- |
| `effective_btlbw_kbps_` | 最终用于 pacing scaling 的控制带宽。 |
| `previous_effective_btlbw_kbps_` | 上一轮 cruise 的控制带宽，用于限制无可信 drate 时的增长。 |
| `drate_ref_kbps_` | 可信 `best_drate_mean_kbps` 的 EWMA。 |
| `maxbw_ref_kbps_` | `raw_max_bw_kbps` 的 EWMA。 |
| `previous_raw_max_bw_kbps_` | 上一轮 cruise 的原始 maxbw。 |
| `previous_drate_ref_kbps_` | 上一轮 cruise 的 drate 参考值。 |
| `stable_cruise_count_` | 连续满足 `ready_to_blend` 的 cruise 数。 |
| `blend_cruise_count_` | 从 drate 向 maxbw 平滑切换的进度。 |
| `no_best_window_count_` | 连续没有可信 best window 的 cruise 数。 |
| `maxbw_over_drate_count_` | `maxbw_ref_kbps` 明显高于 `drate_ref_kbps` 且伴随可疑信号的连续计数。 |
| `hybrid_btlbw_source_` | 当前控制来源。 |

推荐初始参数：

| 参数 | 建议值 | 含义 |
| --- | ---: | --- |
| `quality_min` | `0.55` | best window 最低综合质量。 |
| `freq_score_min` | `0.60` | drate/SRTT 频率分数最低要求。 |
| `min_valid_window_duration_s` | `0.30` | best window 最短有效覆盖时长，需随频率调整。 |
| `min_valid_sample_count` | `4` | drate 和 SRTT 的最低原始样本数。 |
| `drate_ewma_gain` | `0.30` | drate 参考值 EWMA 新样本权重。 |
| `low_confidence_drate_gain` | `0.10` | low-confidence best window 的 EWMA 新样本权重。 |
| `maxbw_ewma_gain` | `0.25` | maxbw 参考值 EWMA 新样本权重。 |
| `maxbw_stable_ratio` | `0.08` | raw maxbw 相对上一轮变化小于 8% 视为自身稳定。 |
| `drate_stable_ratio` | `0.10` | drate_ref 相对上一轮变化小于 10% 视为稳定。 |
| `maxbw_drate_close_ratio` | `0.15` | maxbw_ref 与 drate_ref 差距小于 15% 视为二者收敛。 |
| `stable_required_cruises` | `3` | 连续 3 轮满足后开始切换。 |
| `transition_len_cruises` | `3` | 用 3 轮从 drate 平滑切到 maxbw。 |
| `maxbw_over_drate_ratio` | `0.25` | steady 后 maxbw_ref 高于 drate_ref 25% 视为可疑。 |
| `fallback_required_cruises` | `2` | 连续 2 轮可疑后回退。 |
| `no_best_window_hold_limit` | `3` | 连续无可信 best window 时最多持有旧 drate_ref 的轮数。 |
| `queue_delay_threshold_ms` | 待扫参 | 排队延迟保护阈值。 |
| `loss_rate_threshold` | 待扫参 | loss 保护阈值。 |
| `ecn_rate_threshold` | 待扫参 | ECN 保护阈值。 |
| `min_scale` | `0.50` | 第一版 pacing scale 下界保护。 |
| `max_scale` | `1.10` | 第一版 pacing scale 上界保护。 |

`min_scale` 和 `max_scale` 是第一版工程安全保护，不是理论最优参数，应做敏感性实验：

```text
min_scale in {0.4, 0.5, 0.6}
max_scale in {1.0, 1.1, 1.2}
```

## 5. Trusted Best-Window Update

只用可信 best window 更新 `drate_ref_kbps_`：

```text
best_window_trusted =
    best_full_load_window_exists
    and best_full_load_quality >= quality_min
    and best_drate_freq_score >= freq_score_min
    and best_srtt_freq_score >= freq_score_min
    and best_drate_mean_kbps > 0
    and best_window_duration_s >= min_valid_window_duration_s
    and best_drate_sample_count >= min_valid_sample_count
    and best_srtt_sample_count >= min_valid_sample_count
```

加入窗口覆盖和样本充分性是必要的：如果 cruise 太短或样本太少，频率分数可能偶然过线，不应强烈更新 `drate_ref_kbps_`。

第一版可以用二值权重：

```text
if best_low_confidence:
    drate_gain = low_confidence_drate_gain   # e.g. 0.10
else:
    drate_gain = drate_ewma_gain             # e.g. 0.30

drate_ref_kbps = EWMA(drate_ref_kbps, best_drate_mean_kbps, drate_gain)
```

后续可改成连续质量加权 EWMA：

```text
drate_gain = drate_ewma_gain * best_full_load_quality
```

或：

```text
drate_gain =
    drate_ewma_gain * min(best_drate_freq_score, best_srtt_freq_score)
```

这样可信度连续影响更新幅度，而不是只依赖 `best_low_confidence` 的二值判断。

## 6. Stability and Congestion Guards

### raw_max_bw_kbps 与 maxbw_ref_kbps 的职责

文档中严格区分以下变量：

```text
raw_max_bw_kbps:
    BandwidthEstimate() 的单轮原始输出。
    用于检测原始 max-filter 估计是否发生突变。

maxbw_ref_kbps:
    raw_max_bw_kbps 的 EWMA。
    用作 BLEND_TO_MAXBW 和 MAXBW_STEADY 阶段的平滑控制参考。

drate_ref_kbps:
    可信 best_drate_mean_kbps 的 EWMA。
    用作早期校准参考。

effective_btlbw_kbps:
    最终用于 pacing scaling 的控制带宽。
```

因此：

- `maxbw_self_stable` 用 `raw_max_bw_kbps` 判断是合理的，因为它要捕捉原始估计跳变。
- `maxbw_close_to_drate` 用 `maxbw_ref_kbps` 和 `drate_ref_kbps` 判断是合理的，因为它看的是平滑参考是否收敛。
- `MAXBW_STEADY` 中使用 `maxbw_ref_kbps` 而不是单轮 `raw_max_bw_kbps` 是合理的，因为 steady 阶段也不应直接使用单轮 spike。

### drate_recent

如果连续几轮没有可信 best window，`drate_ref_kbps_` 没有更新，它会天然表现为稳定。这个“稳定”只是旧值没变，不是可信信号稳定。

因此定义：

```text
drate_recent =
    no_best_window_count <= no_best_window_hold_limit
```

并规定：

```text
drate_self_stable is considered valid only when drate_recent is true.
```

### maxbw_self_stable

```text
if previous_raw_max_bw_kbps > 0:
    maxbw_delta_ratio =
        abs(raw_max_bw_kbps - previous_raw_max_bw_kbps)
        / previous_raw_max_bw_kbps
else:
    maxbw_delta_ratio = infinity

maxbw_self_stable =
    maxbw_delta_ratio <= maxbw_stable_ratio
```

### drate_self_stable

```text
if drate_recent and previous_drate_ref_kbps > 0:
    drate_delta_ratio =
        abs(drate_ref_kbps - previous_drate_ref_kbps)
        / previous_drate_ref_kbps
else:
    drate_delta_ratio = infinity

drate_self_stable =
    drate_recent
    and drate_delta_ratio <= drate_stable_ratio
```

### maxbw_close_to_drate

```text
if drate_ref_kbps > 0:
    maxbw_drate_gap_ratio =
        abs(maxbw_ref_kbps - drate_ref_kbps) / drate_ref_kbps
else:
    maxbw_drate_gap_ratio = infinity

maxbw_close_to_drate =
    maxbw_drate_gap_ratio <= maxbw_drate_close_ratio
```

### congestion_guard

queue delay、loss、ECN 不能只做 trace，它们应作为过估计保护信号：

```text
congestion_guard =
    queue_delay_ms > queue_delay_threshold_ms
    or loss_rate > loss_rate_threshold
    or ecn_rate > ecn_rate_threshold
```

如果某些信号不可用，则该项视为 false。例如未启用 ECN 时：

```text
ecn_rate > ecn_rate_threshold == false
```

### ready_to_blend

切换到 maxbw 前必须满足：

```text
ready_to_blend =
    drate_recent
    and maxbw_self_stable
    and drate_self_stable
    and maxbw_close_to_drate
    and not congestion_guard
```

这里 `drate_recent` 很关键，避免状态机把陈旧 `drate_ref_kbps_` 的不变误判为稳定，从而过早从 `DRATE_BOOTSTRAP` 切到 `BLEND_TO_MAXBW`。

## 7. Hybrid BtlBw State Machine

### DRATE_BOOTSTRAP

目标：前期抑制偏高 `raw_max_bw_kbps`。

```text
on cruise end:
    raw_max_bw_kbps = BandwidthEstimate()

    if best_window_trusted:
        gain = best_low_confidence ? low_confidence_drate_gain : drate_ewma_gain
        drate_ref_kbps = EWMA(drate_ref_kbps, best_drate_mean_kbps, gain)
        no_best_window_count = 0
    else:
        no_best_window_count += 1

    maxbw_ref_kbps = EWMA(maxbw_ref_kbps, raw_max_bw_kbps, maxbw_ewma_gain)

    if drate_ref_kbps is valid:
        if no_best_window_count <= no_best_window_hold_limit:
            effective_btlbw_kbps = drate_ref_kbps
        else:
            # 长期没有可信 drate 时，不能永远卡在旧值。
            # 逐步放松到 maxbw_ref，但限制每轮上升幅度。
            effective_btlbw_kbps =
                min(maxbw_ref_kbps, previous_effective_btlbw_kbps * 1.10)
    else:
        effective_btlbw_kbps = raw_max_bw_kbps

    if ready_to_blend:
        stable_cruise_count += 1
    else:
        stable_cruise_count = 0

    if stable_cruise_count >= stable_required_cruises:
        state = BLEND_TO_MAXBW
        blend_cruise_count = 0
```

### BLEND_TO_MAXBW

目标：避免从 `drate_ref_kbps` 突然跳到 `maxbw_ref_kbps`。

```text
on cruise end:
    update drate_ref_kbps if best_window_trusted
    update maxbw_ref_kbps

    blend_cruise_count += 1
    alpha = min(1.0, blend_cruise_count / transition_len_cruises)

    effective_btlbw_kbps =
        (1 - alpha) * drate_ref_kbps
      + alpha       * maxbw_ref_kbps

    if alpha >= 1.0:
        state = MAXBW_STEADY
        maxbw_over_drate_count = 0

    if maxbw_ref_kbps > drate_ref_kbps * (1 + maxbw_over_drate_ratio)
       and (not maxbw_close_to_drate or congestion_guard):
        state = DRATE_BOOTSTRAP
        stable_cruise_count = 0
```

### MAXBW_STEADY

目标：回到基于 `maxbw_ref_kbps` 的控制，同时保留回退保护。

```text
on cruise end:
    update drate_ref_kbps if best_window_trusted
    update maxbw_ref_kbps

    effective_btlbw_kbps = maxbw_ref_kbps

    if drate_ref_kbps is valid
       and maxbw_ref_kbps > drate_ref_kbps * (1 + maxbw_over_drate_ratio)
       and (not maxbw_self_stable or congestion_guard):
        maxbw_over_drate_count += 1
    else:
        maxbw_over_drate_count = 0

    if maxbw_over_drate_count >= fallback_required_cruises:
        state = DRATE_BOOTSTRAP
        stable_cruise_count = 0
```

解释：

```text
If maxbw_ref remains higher than drate_ref but does not cause queueing,
loss, or ECN, it may indicate a real capacity increase. Otherwise, it is
treated as a possible overestimation.
```

### 真实容量上升的处理

如果 `maxbw_ref_kbps` 持续高于 `drate_ref_kbps`，但没有 `congestion_guard`，且 `raw_max_bw_kbps` 自身稳定，应把它视为可能的真实容量上升，而不是立即回退。

建议策略：

```text
if maxbw_ref_kbps > drate_ref_kbps * (1 + maxbw_over_drate_ratio)
   and maxbw_self_stable
   and not congestion_guard:
    # 暂时接纳更高的 maxbw_ref。
    # 同时放慢 drate_ref 更新或等待新的可信 best window。
    do not increment maxbw_over_drate_count
    effective_btlbw_kbps = maxbw_ref_kbps
```

如果随后出现 queue delay、loss 或 ECN，则说明该高估计可能不可持续，再触发回退。

## 8. Pacing-Rate Scaling Interface

第一版建议不要直接改 BBRv2 内部 bandwidth filter，而是在 `FreqCCv4Sender::PacingRate()` 层做缩放。

当前逻辑：

```text
base_rate = Bbr2Sender.PacingRate(bytes_in_flight)
amplitude = GetCurrentAmplitudeBps()
final_rate = base_rate + amplitude * TriangleWave(...)
```

建议改为：

```text
raw_base_rate = Bbr2Sender.PacingRate(bytes_in_flight)
raw_bw = BandwidthEstimate()

if hybrid enabled and effective_btlbw_kbps valid and raw_bw > 0:
    scale = effective_btlbw_kbps / raw_bw
    scale = clamp(scale, min_scale, max_scale)
    corrected_base_rate = raw_base_rate * scale
else:
    scale = 1.0
    corrected_base_rate = raw_base_rate

amplitude = GetAmplitude(corrected_base_rate, effective_btlbw_kbps)
final_rate = corrected_base_rate + amplitude * TriangleWave(...)
```

关键点：调制幅度也必须基于修正后的 `corrected_base_rate` 或 `effective_btlbw_kbps`，不能继续基于未修正的 `raw_max_bw_kbps` 或未缩放的 `raw_base_rate`。否则基础速率已经被压低，但扰动幅度仍然偏大，可能重新造成 overshoot。

建议幅度接口：

```text
GetAmplitude(mode, corrected_base_rate, effective_btlbw):
    fixed:
        return fixed_amplitude_bps
    miuN:
        return effective_btlbw / N
    srN:
        return corrected_base_rate / N
```

`min_scale` 和 `max_scale` 是第一版安全保护，应通过敏感性实验确定。

## 9. Trace Fields

为了定位状态机和 pacing 行为，需要 trace：

```text
raw_max_bandwidth_kbps
best_drate_mean_kbps
effective_btlbw_kbps
hybrid_btlbw_source
drate_ref_kbps
maxbw_ref_kbps
scale
best_window_trusted
best_low_confidence
drate_recent
maxbw_self_stable
drate_self_stable
maxbw_close_to_drate
congestion_guard
queue_delay_ms
loss_rate
ecn_rate
maxbw_delta_ratio
drate_delta_ratio
maxbw_drate_gap_ratio
stable_cruise_count
blend_cruise_count
no_best_window_count
maxbw_over_drate_count
```

`scale` 必须 trace，否则无法判断 pacing 改动到底来自 hybrid BtlBw，还是来自原始 BBR phase gain 或 FreqCCv4 的三角波调制。

`fair_share_bandwidth_kbps` 可以继续输出，但只能用于离线画图和误差评估：

```text
abs(effective_btlbw_kbps - fair_share_bandwidth_kbps) / fair_share_bandwidth_kbps
abs(raw_max_bw_kbps - fair_share_bandwidth_kbps) / fair_share_bandwidth_kbps
abs(best_drate_mean_kbps - fair_share_bandwidth_kbps) / fair_share_bandwidth_kbps
```

它不能参与在线状态转移。

## 10. Validation Plan

建议先做“只 trace，不控制”的版本：

1. 每个 cruise 离线计算状态机结果和 `effective_btlbw_kbps`。
2. 不改变实际 pacing。
3. 检查状态机是否符合预期。

基础验证：

1. 前几轮 `raw_max_bw_kbps` 明显高于 `drate_ref_kbps` 时，状态是否停留在 `DRATE_BOOTSTRAP`。
2. `raw_max_bw_kbps` 自身稳定、`drate_ref_kbps` 近期有效且二者接近时，是否进入 `BLEND_TO_MAXBW`。
3. 后期 best window 缺失或抖动时，`effective_btlbw_kbps` 是否不会被单轮 drate 大幅拉动。
4. `fair_share_bandwidth_kbps` 是否只用于离线误差评估，不影响状态机。

反例场景：

1. No trusted best window for multiple cruises:
   验证系统是否逐步放松到 `maxbw_ref_kbps`，而不是卡死在旧 `drate_ref_kbps`。

2. Sudden capacity increase:
   验证 `raw_max_bw_kbps` 高于 `drate_ref_kbps` 但无拥塞时，系统不会错误回退。

3. Persistent overestimation with queue/loss/ECN:
   验证 `maxbw_ref_kbps` 高于 `drate_ref_kbps` 且有拥塞信号时，系统能回退到 `DRATE_BOOTSTRAP`。

参数敏感性实验：

```text
min_scale in {0.4, 0.5, 0.6}
max_scale in {1.0, 1.1, 1.2}
maxbw_drate_close_ratio in {0.10, 0.15, 0.20}
maxbw_over_drate_ratio in {0.20, 0.25, 0.30}
stable_required_cruises in {2, 3, 4}
```

## 11. Risks and Fallback Policies

1. `best_drate_mean_kbps` 来自频谱识别出的 best window，误判时会污染控制环，所以必须有质量、频率、窗口覆盖、样本数量门槛。
2. `drate_self_stable` 只有在 `drate_recent` 为 true 时才有意义，否则旧值不变会被误判为稳定。
3. queue delay、loss、ECN 应参与切换和回退逻辑，不能只做 trace。
4. 如果长期没有可信 best window，不能无限期使用旧 `drate_ref_kbps`，需要逐步放松到 `maxbw_ref_kbps`。
5. 如果路径真实容量上升，`maxbw_ref_kbps > drate_ref_kbps` 不一定是坏事；没有 `congestion_guard` 且 `raw_max_bw_kbps` 自身稳定时，应允许系统接纳新容量。
6. 第一版只建议控制 pacing rate，不建议同步修改 cwnd 或 BBRv2 内部 max filter。
7. `min_scale` / `max_scale` 是工程保护参数，需要扫参，不应作为理论常数写死。

## 12. Recommended Implementation Order

1. 实现状态机 trace，不改变控制。
2. 用 `freqccv4_4flow` 多轮实验验证状态转移是否符合直觉。
3. 加命令行开关启用 hybrid pacing scaling，默认关闭。
4. 比较原始 FreqCCv4 与 hybrid FreqCCv4 的 goodput、公平性、queue delay、loss。
5. 如果 pacing scaling 有收益，再考虑是否把 `effective_btlbw_kbps` 接入 cwnd/BDP 相关路径。
