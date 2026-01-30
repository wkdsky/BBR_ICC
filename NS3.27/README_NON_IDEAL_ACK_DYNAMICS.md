# 非理性 ACK 动态（NIAD）Trace 计划

我们采用如下定义：
“Non-Ideal ACK Dynamics 包括 ACK compression 和 ACK aggregation，
它们会扭曲 ACK 到达过程并偏置 delivery-rate sampling。”

本计划按“每次 ACK 到达即检查”的方式识别 NIAD，不使用固定时间窗口。

## 目标

- 识别 ACK compression（ACK 到达过于密集）和 ACK aggregation（ACK 到达过于稀疏、聚合）。
- 量化每次事件的开始时刻、持续时间、影响字节数以及对采样速率的偏置。

## 采集位置（Sender 侧）

建议在发送端获取以下信息：
- ACK 到达事件：`ack_receive_time`、`acked_bytes`、`acked_packets`、
  `largest_acked`、`ack_delay_ms`、`smoothed_rtt_ms`。
- 发送/节奏速率（pacing rate）若可获得，用于偏置计算。

可用钩子建议：
- `SendPacketManager::OnAckEnd`：ACK 到达与 `acked_bytes`。
- 发送侧 `sendrate` trace：pacing rate。

## Trace 文件

### 1) 每次 ACK 事件 Trace
文件名：
`<flow>_ackevent.txt`

字段：
```
#time(s)  acked_bytes  acked_pkts  largest_acked  ack_delay(ms)  rtt(ms)  ack_interval(ms)  ack_rate(kbps)  pacing_rate(kbps)  sample_bias
```

说明：
- `ack_interval(ms)`：距离上一次 ACK 到达的时间。
- `ack_rate(kbps)`：`8 * acked_bytes / ack_interval(ms)`。
- `sample_bias`：`ack_rate / pacing_rate`（无 pacing rate 则置 0）。

### 2) NIAD 事件 Trace（非固定窗口）
文件名：
`<flow>_ackepisode.txt`

字段：
```
#type  start(s)  end(s)  duration(ms)  ack_events  acked_bytes  iat_min(ms)  iat_max(ms)  ack_rate_peak(kbps)  pacing_rate_mean(kbps)  bias_peak
```

说明：
- `type`：`compress` 或 `aggregate`。
- `duration(ms)`：`end - start`。
- `ack_events` / `acked_bytes`：事件期间 ACK 次数/字节数。
- `iat_min/ms`、`iat_max/ms`：事件内 ACK 间隔极值。
- `bias_peak`：事件内 `ack_rate / pacing_rate` 的最大值（无 pacing rate 则用 0）。

## NIAD 识别规则（事件级）

核心思想：每次 ACK 到达时比较“当前 ACK 间隔/字节”与“基线”。

### 基线定义（推荐）
- `baseline_iat`：ACK 间隔的 EWMA。
- `baseline_ack_bytes`：每次 ACK 字节数的 EWMA。
- `baseline_ack_rate`：ACK 速率的 EWMA（可选）。

EWMA 只需在“非异常状态”更新，避免被异常污染。

### 事件触发条件（示例，可调）

**ACK compression（到达过于密集）**
- `ack_interval <= k_c * baseline_iat`
- 且 `ack_rate >= k_r * baseline_ack_rate`（有 pacing 时优先）

推荐：`k_c = 0.5`，`k_r = 1.5`。

**ACK aggregation（到达过于稀疏、聚合）**
- `ack_interval >= k_a * baseline_iat`
- 且 `acked_bytes >= k_b * baseline_ack_bytes`

推荐：`k_a = 2.0`，`k_b = 2.0`。

若同时满足 compression 与 aggregation，选择“偏离度更大”的类型：
```
compression_score = baseline_iat / max(ack_interval, eps)
aggregation_score = ack_interval / max(baseline_iat, eps)
```
取较大者为事件类型。

### 事件结束条件

当连续 `M` 次 ACK 不满足该事件的触发条件即结束（建议 `M = 2`）。

## 事件量化与输出

事件结束时输出一行：
- `start`：事件首次触发的 ACK 到达时刻（或上一个 ACK 时刻）。
- `end`：事件最后一次触发的 ACK 到达时刻。
- `duration`：`end - start`。
- `acked_bytes`：事件期间累计 ACK 字节数。
- `iat_min/max`：事件期间的 ACK 间隔极值。
- `bias_peak`：事件期间最大采样偏置（有 pacing rate 时）。

仿真结束时，若事件未结束，直接以 `sim_end` 作为 `end` 输出。

## 备注

- 本方案不需要固定时间窗口；每次 ACK 都会检查并动态生成事件。
- 事件 trace 与 per-ACK trace 可同时保留，便于复现与后处理。 
