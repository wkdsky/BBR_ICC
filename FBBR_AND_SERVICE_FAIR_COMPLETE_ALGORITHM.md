# FBBR 与 FBBR-ServiceFair 完整算法说明

> 本文统一描述当前可用的 `FBBR`（`kFBBR`）和
> `FBBR-ServiceFair`（`kFBBRServiceFair`）。二者共享 service-consistent
> inflight envelope；ServiceFair 仅在明确的 Cruise、Regime 和
> TrustedBw publication 边界增加服务公平控制。

> 术语说明：源码中的 `FbbrV4*` 类型、函数、trace 字段和部分历史注释保留了
> `V4` 这个实现代号；它不再是可选算法名。本文中未特别说明的 `V4` 均指这套
> 由 `FBBR` 与 `FBBR-ServiceFair` 共同使用的内部 service-envelope 实现。

## 1. 文档范围

这是一份单文件、可独立阅读的 FBBR / ServiceFair 算法规范，统一说明：

- 独立算法入口和 V1/V3/FBBR/ServiceFair 隔离；
- FBBR/ServiceFair 继承的 BBRv2 phase、MaxBw、RTprop、TrustedBw、GuardBw；
- 频率激励、三角波、Goertzel/time-waveform 观察器；
- DRate/SRTT 特征和 Regime I/II/III 判定；
- ReferenceBw 和最终 pacing target 的选择；
- `PacingBaseTarget` 的构造；
- target/base rate segment 历史；
- bandwidth sampler 累计 delivered 历史；
- planned inflight、positive probe credit、service inflight；
- app-limited、counter reset、历史覆盖和 fallback；
- service-consistent envelope、native headroom 和最终 cwnd；
- 共享 service-envelope 的私有状态变量、控制变量和纯观测变量；
- ServiceFair 的 service/qdelay 信号、每 Cruise AIMD、Regime III 保护和
  TrustedBw publication 修正；
- 数值安全、边界行为、运行时伪代码、trace 和 self-test。

实验结果和验收结论仍保存在
`FBBR_SERVICE_CONSISTENT_INFLIGHT_ENVELOPE_V4_REPORT.md`。本文的重点是
“算法如何运行”，不是重复实验报告。

## 2. 一句话定义

FBBR 保持 V3 的长期 ReferenceBw 和最终 pacing target，不根据短期 RTT 或
队列直接降速；它把 V3 的无队列发送计划：

\[
I_{plan}(t)=
\int_{t-RTprop}^{t}\frac{PacingTarget(\tau)}{8}\,d\tau
\]

扩展为同时受近期真实服务量约束的 envelope：

\[
I_{env}(t)=
\begin{cases}
\min\left(I_{plan}(t), I_{service}(t)+I_{probe}^{+}(t)\right),
& service\ history\ valid\\
I_{plan}(t),& otherwise
\end{cases}
\]

最后只在 cwnd getter 的出口执行：

\[
Cwnd_{final}=\min(Cwnd_{native},I_{cap})
\]

\[
I_{cap}=
AlignMssUp\left(
\max\left(MinPipeCwnd,\,
I_{env}+ExtraAcked+OffloadBudget
\right)\right)
\]

当前实现：

\[
OffloadBudget=0
\]

因此 FBBR shared service envelope 的控制分工是：

| 时间尺度 | 决定量 | 职责 |
|---|---|---|
| 长期 | TrustedBw / GuardBw / PreviousTrusted / native fallback | 决定 pacing 的参考速率 |
| 一个 RTprop | 已确认 delivered bytes | 在容量下降后收缩可用 inflight |
| 一个 RTprop | 已经存在的正向 phase/waveform probing | 为容量增长提供扩张信用 |
| ACK 聚合时间尺度 | BBRv2 `MaxAckHeight()` | 保留 native ACK aggregation headroom |
| 最终发送接口 | `min(native_cwnd, inflight_cap)` | 唯一新增执行器 |

## 3. 算法身份和隔离

### 3.1 算法身份

| 外部名称 | 内部枚举 | 独立控制行为 |
|---|---|---|
| `FBBR-hybrid` | `kFBBRHybrid` | Gradient-Matched V1 |
| `FBBR-hybridv3` | `kFBBRHybridV3` | Model-Consistent Inflight Projection |
| `FBBR` | `kFBBR` | Service-Consistent Inflight Envelope |
| `FBBR-ServiceFair` | `kFBBRServiceFair` | FBBR envelope + Cruise service fairness |

`kFBBR` 复用原 V4 的数值槽位；旧裸 FBBR 的数值槽位不再映射到任何算法，
因此新 FBBR 和 ServiceFair 的既有数值保持稳定。

### 3.2 身份 helper

```cpp
bool IsFbbrHybrid() const;             // 只匹配 V1
bool IsFbbrHybridV3() const;           // 只匹配 V3
bool IsFbbr() const;                   // 只匹配 FBBR
bool IsFbbrServiceFair() const;        // 只匹配 ServiceFair
bool UsesFbbrServiceEnvelope() const;  // FBBR || ServiceFair
bool IsFbbrProjectionObserver() const; // V3 || service envelope
bool IsFbbrHybridObserver() const;     // V1 || V3 || service envelope
```

职责：

- `IsFbbr()` 只识别裸 `FBBR`；
- `IsFbbrServiceFair()` 只打开公平控制；它不创建第二套 envelope；
- `UsesFbbrServiceEnvelope()` 确保 FBBR 与 ServiceFair 使用相同的
  target/base history、delivered history、telemetry 和 final cwnd cap；
- `IsFbbrProjectionObserver()` 共享 V3/FBBR/ServiceFair 的信息层和
  Reference 相关观测；
- `IsFbbrHybridObserver()` 共享频率观察、Guard、Trusted 和部分 RTprop
  观察逻辑；
- 没有全局开关把 V3 替换成 FBBR。

### 3.3 V1、V3 不受 FBBR shared state 影响

- V3 继续使用自己的 `FbbrV3PacingTargetSegment`；
- V3 只记录 `target_rate`，不记录 shared envelope 的 `base_target_rate`；
- V3 的 projection 仍要求 `ReferenceBw valid`；
- FBBR/ServiceFair 使用独立 `fbbr_v4_rate_history_`；
- FBBR/ServiceFair 使用独立 `fbbr_v4_delivered_history_`；
- 共享记录、计算和 summary helper 都先检查 `UsesFbbrServiceEnvelope()`；
- V1 仍进入 `ApplyFbbrHybridClassification()`；
- V3/FBBR/ServiceFair 进入 `ApplyFbbrHybridV3Classification()`；
- FBBR/ServiceFair 不调用 V1 的 Gradient-Matched baseline actuator。

## 4. FBBR shared service-envelope 端到端控制路径

```text
Native BBRv2
  ├─ MaxBw / MinRtt / ProbeBW phase / inflight_hi/lo / ProbeRTT
  ├─ native pacing rate
  ├─ native cwnd
  ├─ cumulative total_bytes_acked
  ├─ current app-limited state
  └─ MaxAckHeight
          │
          ├──────────────┐
          │              │
Frequency/time-waveform  │
observer                 │
  ├─ DRate/SRTT features │
  ├─ Regime I/II/III     │
  ├─ TrustedBw/GuardBw   │
  └─ RTprop observations │
          │              │
          ▼              │
Reference selection      │
          │              │
          ▼              │
PacingTarget ────────────┼──► target history ─► I_plan
PacingBaseTarget ────────┼──► base history ───► I_probe+
                         │
cumulative delivered ────┴──► delivered history ─► I_service
                                                    │
                          service valid ─────────────┤
                                                    ▼
                         I_env = min(plan, service + probe)
                                                    │
MaxAckHeight ───────────────────────────────────────┤
                                                    ▼
                                              I_cap
                                                    │
native cwnd ────────────────────────────────────────┤
                                                    ▼
                                  final cwnd = min(native, cap)
```

控制路径中没有从 `I_env`、`I_cap` 或 `Cwnd_final` 返回修改：

- `PacingTarget`；
- ReferenceBw；
- TrustedBw；
- GuardBw；
- MaxBw；
- `inflight_hi`；
- `inflight_lo`；
- ProbeBW phase；
- ProbeRTT 状态。

## 5. 单位和符号

| 符号/类型 | 单位或含义 |
|---|---|
| `QuicBandwidth` | bit/s |
| `QuicTime` | 单调时间戳 |
| `TimeDelta` | 时间间隔，积分内部使用 µs |
| `QuicByteCount` / `uint64_t` | bytes |
| `PacingTarget` | 实际返回给 pacer 的目标速率 |
| `PacingBaseTarget` | 移除所有正向 probing 后的同路径目标速率 |
| `D(t)` | sampler 权威累计 acked/delivered payload bytes |
| `RTprop` | `fbbr_hybrid_srtt_low_` 有效时优先，否则 native `MinRtt()` |
| `Cwnd_native` | BBRv2 已完成原生计算后的 `cwnd_` |
| `ActualInflight` | 最近 congestion event 的 bytes in flight |

bit/s、µs 到 byte 的换算为：

\[
Bytes=\frac{Rate_{bps}\times Duration_{\mu s}}{8\,000\,000}
\]

## 6. 运行时生命周期

### 6.1 初始化和算法创建

命令行中的 `FBBR`、`FBBR-ServiceFair` 分别解析为 `kFBBR`、
`kFBBRServiceFair`。工厂为它们创建独立 `FBBRSender`，但底层仍是同一个
BBRv2 sender/model。

两种算法共享的 service-envelope history 初始为空：

- rate history integrity 为 true；
- delivered history integrity 为 true；
- last target/base 为 zero；
- counter reset time 为 zero；
- telemetry counters 为 zero。

### 6.2 每次发送

`OnPacketSent()` 的顺序是：

1. 更新 `current_time_ = sent_time`；
2. 调用 `PacingRate(bytes_in_flight)`；
3. 在 `PacingRate()` 内同时形成最终 target 和 base target；
4. FBBR/ServiceFair 记录或更新 rate segment；
5. sender-rate observer history 记录实际返回的 target；
6. 调用 native `Bbr2Sender::OnPacketSent()`。

shared service-envelope 的 cwnd cap 不参与 `PacingRate()`，所以 cap binding 不会改变本次或后续
`PacingTarget` 的计算公式。

### 6.3 每次 ACK/congestion event

`OnCongestionEvent()` 的关键顺序是：

1. 记录 event time 和本批 ACK bytes；
2. 使用既有逻辑衰减 waveform 对 native MaxBw sample 的正向污染；
3. 先调用 `Bbr2Sender::OnCongestionEvent()`；
4. native model 更新 MaxBw、MinRtt、phase、cwnd、累计 acked 和
   app-limited；
5. 更新共享 DRate、SRTT、ACK-window、Guard 和 waveform observer；
6. 在 ProbeBW Cruise 中推进 frequency/time-waveform 状态机；
7. 仅当 `acked_packets` 非空时，FBBR/ServiceFair 记录更新后的
   `model_.total_bytes_acked()` 和 `model_.is_app_limited()`；
8. 更新 shared service-envelope telemetry snapshot；ServiceFair 随后更新
   qdelay/service 信号。

这样 delivered point 一定来自 native sampler 已完成 ACK 更新后的累计值。

### 6.4 每次读取 cwnd

```cpp
QuicByteCount FBBRSender::GetCongestionWindow() const {
    if (UsesFbbrServiceEnvelope()) {
        return ApplyFbbrV4InflightEnvelope(cwnd_);
    }
    return ApplyFbbrV3InflightProjection(cwnd_);
}
```

FBBR/ServiceFair 先构造一次完整 snapshot。若 projection 不活跃，原样返回 native cwnd；
若活跃，返回：

\[
\min(Cwnd_{native},I_{cap})
\]

### 6.5 flow 结束

FBBR/ServiceFair 各自只汇总一次 flow summary。它不会输出逐 ACK 或逐 packet 的新文件日志。

## 7. 保持不变的 native BBRv2 层

FBBR shared service envelope 不替换 BBRv2 的以下机制：

- Startup；
- 初始 Drain/Startup 到 ProbeBW 的转换；
- ProbeBW Cruise/Refill/Up/Down progression；
- ProbeRTT；
- bandwidth sampler；
- delivery-rate sample；
- MaxBw filter；
- MinRtt/RTprop 基础测量；
- native cwnd；
- native pacing limits；
- `inflight_hi` / `inflight_lo`；
- ACK aggregation 的 `MaxAckHeight()`；
- native loss/recovery 行为。

V4 只是最终 send allowance 的额外上限。它不是新的 BBR mode，也没有 V4
专属 Drain、Recovery、进入、退出或恢复状态机。

## 8. 频率激励和 convergence gate

### 8.1 激励位置

频率激励只在以下条件满足时形成：

- 已完成初始 drain；
- 当前为 `PROBE_BW`；
- 当前 phase 为 `PROBE_CRUISE`；
- 已进入 Cruise；
- modulation frequency 大于 0；
- amplitude 大于 0；
- time-waveform 模式下 baseline、probe epoch 和状态机均有效；
- convergence gate 启用时，当前不处于 `bbr_stable_`。

### 8.2 每轮 delivery-rate 稳定性

每一 round 取非 app-limited、非 recovery 的最大 delivery-rate sample：

\[
D_{round}=\max(sample\_max\_bandwidth)
\]

相邻 round 的相对变化：

\[
v_{round}=
\frac{|D_{round}-D_{prev}|}{D_{prev}}
\]

默认稳定门：

- 单轮退出稳定：`v_round > 0.25`；
- 连续退出稳定：本轮和上一轮都 `> 0.15`；
- 未稳定时累计 3 个没有 full-pipe growth 的 round；
- 若当前 round 相对 reference 增长到 `1.25×`，更新 reference 并重置
  stable count；
- 达到 3 个稳定 round 后关闭 frequency tool；
- `w_freq = clamp(1 - stable_cnt / stable_rounds, 0, 1)`。

这些是继承的 frequency-observer gate，不是 V4 service envelope 参数。

### 8.3 amplitude

默认配置是 `4sr`，即：

\[
A=\frac{NativePacingRate}{4}
\]

工程同时保留 fixed、MaxBw 的 1/2、1/3、1/4、1/8，以及 sender rate 的
1/2、1/3、1/4、1/8、1/12、1/16 模式。V4 不增加 amplitude 模式。

每个 flow 的配置可以覆盖默认 amplitude；当前默认配置中 flow 0–3 使用
50 Mbps fixed amplitude。

### 8.4 time-waveform 三角波

令：

\[
q=\frac{(t-t_{epoch})\bmod T}{T},\qquad T=\frac{1}{f}
\]

time-waveform 模式的负半周期先行：

\[
Wave(q)=
\begin{cases}
-4q,&0\le q<0.25\\
4q-2,&0.25\le q<0.75\\
4-4q,&0.75\le q<1
\end{cases}
\]

因此一个周期依次经过：

```text
0 → -1 → 0 → +1 → 0
```

整数 pacing offset：

\[
Offset_{bps}=int64(A_{bps}\times Wave(q))
\]

最终 target 使用已有 `AddPacingOffsetWithFloor()`，下限为
`minimum_pacing_rate_bps_`。默认 pacing floor 为 1 Mbps。

legacy spectral 模式沿用原正半周期先行的三角波定义；V4 不修改两种模式。

## 9. DRate/SRTT 观察和窗口处理

V4 使用与 V1/V3 相同的观察器，不另建 Reference estimator。

### 9.1 原始历史

共享 observer 在内存维护：

- `sender_rate_history_`：发送端最终 pacing target；
- `delivery_rate_history_`：最新 delivery rate、有效性、app-limited、
  acked bytes；
- `srtt_history_`：SRTT；
- `ack_window_history_`：ACK bytes 和 loss 标志；
- `pending_hybrid_max_srtt_observations_`：MaxBw 更新附近的 MaxSRTT
  观察窗口。

这些 observer history 与 V4 service history 是不同数据结构。

### 9.2 time-waveform 窗口

默认频率 5 Hz，周期 200 ms。状态机先等待一个 probe-epoch RTT，再采集：

- 初始 2 个周期；
- 不确定时最多扩展到 3 个周期；
- rolling retry 使用后三个周期中的后两个周期重新分析，同时保留前一周期
  的判定信息；
- 连续两个有效窗口无 waveform 时进入 wave-fidelity retry；
- 第二次仍 inconclusive 时，使用既有 1.25 倍 amplitude 放大，最大为
  初始 amplitude 的 2 倍；
- 发现任一 DRate/SRTT waveform 后退出 no-wave enhancement。

该 amplitude retry 是原共享观察器逻辑，不是 V4 envelope 新增参数。

### 9.3 重采样

Hybrid observer 的采样步长为：

\[
\Delta t=
clamp\left(\frac{T}{40},1ms,5ms\right)
\]

最大允许插值 gap：

\[
Gap_{max}=0.10T
\]

sender signal 使用一个 `probe_epoch_rtt_` 的 feedback lag：

```text
sender samples: [window_start - lag, window_end - lag]
DRate samples:  [window_start,       window_end]
SRTT samples:   [window_start,       window_end]
```

DRate input valid 要求：

- 至少 4 个 sample；
- acked bytes 大于 0；
- coverage 不低于 0.85；
- app-limited sample ratio 不高于 0.25。

SRTT input valid 要求：

- 至少 4 个 sample；
- coverage 不低于 0.85。

### 9.4 特征提取

对 DRate 和 SRTT 分别提取：

- min、max、mean；
- ordinary waveform activity；
- 实际周期和周期相关性；
- continuous horizontal segment；
- top/bottom repeated clip；
- positive/negative shoulder；
- middle sequential disturbance；
- edge line mask；
- masked periodic similarity；
- sender target 与 receiver response 的周期一致性；
- current inflight 和 native BDP；
- SRTT 相对 RTprop/MaxSRTT 的位置。

DRate ordinary-wave 判断保留 raw valid view；horizontal/middle disturbance
mask 用于周期和 clipping 判定，避免把断边或平台误当成完整波形。

### 9.5 Goertzel/legacy spectral 路径

legacy spectral 模式继续：

- 对 sender rate、DRate、SRTT 计算目标频率和邻域频谱；
- 计算 target amplitude、noise floor、SNR、peak width、phase coherence；
- DRate 和 SRTT 分别形成 spectral integrity score；
- 联合 score 取二者较小者；
- 两个信号必须分别过门才可发布 spectral TrustedBw；
- normal window 无可靠结果时可执行一次 2× merged rescue；
- merged window 仍必须通过双信号 gate；
- merged rate trend ratio 不得超过 0.20；
- merged confidence 乘 0.8。

V4 不绕过该路径，也不修改其 score、阈值或 TrustedBw 发布函数。

## 10. Regime I/II/III 判定树

### 10.1 clip case 优先级

SRTT clip case 只有在 `SRTT input valid && SRTT has_wave` 时生效。优先级：

```text
U1 positive shoulder
U2 long top line
U3 repeated top clip
L1 negative shoulder
L2 long bottom line
L3 repeated bottom clip
None/fallback
```

如果只是检测到水平线、但 SRTT 没有可观察 waveform，则不直接采用 clip
case，而是进入 no-cut fallback/retry。

### 10.2 N01–N16

| Rule | 前提 | 判定 | 信息层副作用 |
|---|---|---|---|
| N01 | U1 且 DRate periodic match | Regime II / FULL_LOAD | 无 lower reference 更新 |
| N02 | U1 且 DRate periodic mismatch | Regime III / OVERLOAD | 无 lower reference 更新 |
| N03 | U2 且 DRate periodic match | Regime II | 无 lower reference更新 |
| N04 | U2 且 DRate periodic mismatch | Regime III | 无 lower reference 更新 |
| N05 | U3 repeated top clip | Regime III | 无 lower reference 更新 |
| N06 | L1 negative shoulder | Regime II | 无 lower reference 更新 |
| N07 | L2 且 DRate ordinary wave | Regime I / UNDERLOAD | refresh RTprop；更新 RTpropDRate/baseline-low |
| N08 | L2，DRate input valid 但无 ordinary wave | Regime II | 无 lower reference 更新 |
| N09 | L3 repeated bottom clip | Regime I | 更新 RTpropDRate/baseline-low，不 refresh RTprop |
| N10 | 无 clip、SRTT 有 wave、`SRTTmax > MaxSRTT` | Regime III | 无 lower reference 更新 |
| N11 | 无 clip、SRTT 有 wave、`SRTTmin < RTprop` | Regime I | refresh RTprop；更新 RTpropDRate/baseline-low |
| N12 | 无 clip、SRTT 有 wave，进入 fallback threshold | Regime I / II / III | 无 lower reference 更新 |
| N13 | 无 clip、SRTT 无 wave、DRate periodic match | Regime I | 无 lower reference 更新 |
| N14 | 无 clip、SRTT 无 wave、`SRTTmax > MaxSRTT` | Regime III | 无 lower reference 更新 |
| N15 | 无 clip、SRTT 无 wave、`SRTTmin < RTprop` | Regime I | refresh RTprop；更新 RTpropDRate/baseline-low |
| N16 | 无 clip、SRTT 无 wave，进入 fallback threshold | Regime I / II / III | 无 lower reference 更新 |

需要 DRate periodic 的规则在 periodic input invalid 时返回
`INCONCLUSIVE`。

### 10.3 N12/N16 fallback

N12/N16 只在 SRTT stats、`RTprop` 和 `MaxSRTT` 均有效，且
`MaxSRTT >= RTprop > 0` 时可判定。当前源码定义：

\[
T_{overload}=\max\left(
1.10RTprop,
RTprop+\frac{MaxSRTT-RTprop}{3}
\right)
\]

然后按 `SRTTmax` 三段分类：

| 条件 | 结果 |
|---|---|
| `SRTTmax > T_overload` | Regime III / OVERLOAD |
| `SRTTmax < 1.05 * RTprop` | Regime I / UNDERLOAD |
| 其余 | Regime II / FULL_LOAD |

它不是 inflight/BDP 判据，也不读取 `SRTTmean`。前提无效时返回
`INCONCLUSIVE`。

### 10.4 `/3` 与边界值的源码核对

N12/N16 当前编译代码使用上节的 `/3` 公式，并同时带有 `1.10 * RTprop`
overload floor 和 `1.05 * RTprop` underload floor；不存在 `/4` 或
`Inflight >= 1.1 * BDP` 的 fallback 分支。

V1 Gradient-Matched queue guard 仍有独立的 `/3` 公式：

\[
Q_{guard}=
\max\left(0.1RTprop,\frac{MaxSRTT-RTprop}{3}\right)
\]

FBBR 与 ServiceFair 共用 N12/N16 的三段 SRTT fallback；二者不调用 V1 的
Gradient-Matched baseline decrease。V1 的 queue guard 即使被共享分析代码
计算，也不会成为 FBBR/ServiceFair actuator。

## 11. Regime 对 FBBR 的实际作用

FBBR/ServiceFair 共用 N01-N16 分类和 `ComputeFbbrV4InjectionBaseline()`。
分类的信息层副作用先由 `ApplyFbbrHybridRegimeStateUpdates()` 执行；随后
才执行下面的 FBBR baseline/TrustedBw 动作。ServiceFair 的额外修正在第 38
节以后说明。

### 11.1 Inconclusive 或无效 DRate

`INCONCLUSIVE`、无效 DRate stats、或 Regime II 所需的累计 delivered
区间无效时都不改变 baseline 或 TrustedBw，保留错误原因并在 settle 后继续
采集。若 Regime II 前本 Cruise 已出现 Regime I/III，缺少合法 delivered
区间同样只能 hold，不能回退为 ACK-sample mean。

### 11.2 Regime II / FULL_LOAD

若本 Cruise 尚未出现 Regime I 或 III，Regime II 验证窗口但保持当前
baseline。若已经出现 I/III，必须使用窗口边界之间的累计 delivered rate：

```text
baseline  = Delivered(window_start, window_end) / window_duration
TrustedBw = 同一个 cumulative-delivery rate
```

这条合法 Regime II measurement 是 Cruise publication 的最高优先级。

### 11.3 Regime I / UNDERLOAD

记当前 baseline 为 \(B\)、窗口最大 delivery rate 为 \(R_{max}\)、native
MaxBw 为 \(M\)。FBBR 按顺序选择：

\[
B_{next}=\begin{cases}
R_{max},& B<R_{max}<M\\
R_{max}+\frac{M-R_{max}}{2},& R_{max}<M\ \land\ B<\frac{M+R_{max}}2\\
1.02B,& otherwise
\end{cases}
\]

结果受 minimum pacing rate 下限保护。N07/N09/N11/N15 仍按第 10 节更新
RTprop / RTpropDRate / lower observation；该信息更新不是新的 fairness
动作。

### 11.4 Regime III / OVERLOAD

记窗口最小 delivery rate 为 \(R_{min}\)、native MinBw 为 \(m\)。FBBR
按顺序选择：

\[
B_{next}=\begin{cases}
R_{min},& m<R_{min}<B\\
m+\frac{R_{min}-m}{2},& R_{min}>m\ \land\ m+\frac{R_{min}-m}{2}<B\\
0.98B,& otherwise
\end{cases}
\]

这不是 V1 Gradient-Matched decrease，也不写 `inflight_hi/lo`。ServiceFair
会在这条 raw candidate 之上再施加第 41 节的 service floor/yield cap。

### 11.5 Regime 的边界

Regime 可以改变当前 Cruise baseline、在满足条件时发布 TrustedBw，并更新
共享的 RTprop/lower observation；它不能直接改变 `I_service`、`I_probe+`、
`I_env`、native BBR mode、`inflight_hi/lo` 或创建 recovery 状态。

## 12. GuardBw

Guard 是共享的 ACK-clock 低通 fallback，不是 FBBR/ServiceFair 新 estimator。

### 12.1 原始 Guard sample

只在 ProbeBW Cruise、存在 ACK、send state 有效时维护一个 measurement
window。window 至少持续：

\[
\max(CurrentRTT,50ms)
\]

令：

```text
delta_delivered = cumulative_delivered_now - delivered_at_window_start
ack_elapsed     = ack_time_now - ack_time_start
send_elapsed    = send_time_now - send_time_start
delivery_elapsed = max(ack_elapsed, send_elapsed)
```

sample 只有在以下条件同时满足时有效：

- `delta_delivered > 0`；
- ACK elapsed 和 send elapsed 都大于 0；
- delivery elapsed 有效；
- 若 min RTT 有效，则 `delivery_elapsed >= min_rtt`；
- window 内没有 app-limited。

原始速率：

\[
GuardSample=
\frac{8\times\Delta DeliveredBytes}{DeliveryElapsed}
\]

### 12.2 两极低通

每一级使用：

\[
Y_n=\frac{7Y_{n-1}+X_n}{8}
\]

```text
stage1 = LPF(stage1, raw_sample)
stage2 = LPF(stage2, stage1)
```

首次 sample 直接初始化两个 stage。可信 TrustedBw 发布时，两个 stage
都锚定到该 TrustedBw。

### 12.3 Cruise 结束的 TrustedBw 候选优先级

```text
1. time-waveform Regime II candidate
2. spectral dual-signal candidate
3. 本 Cruise 更新过的 Guard stage2
4. FBBR 的 PreviousTrusted
5. native fallback
```

`FBBR-ServiceFair` 进入 Cruise 时可用 PreviousTrusted 初始化 baseline，
但 Cruise 结束的 publication 不把它作为独立候选来源：在 Guard 也不可用时
先选择 native fallback，再由第 42 节修正。native fallback 仅用于 pacing
bootstrap，不会伪装成 V3 的可信 `ReferenceBw`。

## 13. FBBR/ServiceFair 的 TrustedBw 使用

FBBR/ServiceFair 共用第 12.3 节的 TrustedBw publication，但 `PacingRate()`
不调用仅供 V3 使用的 `SelectFbbrV3ReferenceBw()`。对 shared service envelope
而言，pacing baseline 的优先级是：

1. time-waveform Cruise 使用 `current_injection_baseline_bw_`，phase gain
   固定为 1；
2. 在 ProbeBW 中，若已有 finite、positive 的已发布 `trusted_bw_`，并且不在
   已出现 Regime I/III 的 Cruise 内，则使用 PreviousTrusted；
3. 其余允许应用 TrustedBw 的 phase，使用 fresh、同 Cruise、已通过 native
   publication 校验的 `trusted_bw_`；
4. 以上均不成立时直接使用 native BBR pacing。

已发布的 `trusted_bw_` 可以来自 Regime II cumulative delivery、spectral、
Guard、FBBR PreviousTrusted 或 native fallback。FBBR/ServiceFair 不增加
TTL、置信度阈值或另一套 reference estimator。

重要区别：V3 projection 要求 `SelectFbbrV3ReferenceBw()` 返回有效
`ReferenceBw`；FBBR/ServiceFair 的 envelope 不要求它有效。即使没有可用
TrustedBw，仍会记录 native/time-waveform bootstrap target；target history 覆盖
完整 RTprop 后，planned projection 仍可启用。

## 14. PacingTarget

### 14.1 原始来源优先级

`PacingRate()` 首先取得：

```text
native_pacing = Bbr2Sender::PacingRate(bytes_in_flight)
native_bw     = BandwidthEstimate()
phase         = current ProbeBW phase
phase_gain    = PacingGain()
```

baseline/reference 路径：

1. 默认 native bandwidth/native pacing；
2. time-waveform Cruise 中使用 `current_injection_baseline_bw_`；
3. V1 lower-bound search 只对 V1 生效，FBBR/ServiceFair 不进入；
4. FBBR/ServiceFair 在非 Cruise ProbeBW phase 可使用 usable
   PreviousTrusted 或 fresh TrustedBw；
5. 仅 V3 在 `SelectFbbrV3ReferenceBw()` 有效时以该 reference 覆盖前述
   pacing source。

完整的 pacing-base source 枚举为：

```text
NATIVE_BBR
WAVEFORM_CRUISE_BASELINE
TRUSTED_BW
V3_TRUSTED_REFERENCE
V3_GUARD_REFERENCE
V3_LAST_VALID_REFERENCE
```

后三个 `V3_*_REFERENCE` source 仅属于 V3；FBBR/ServiceFair 实际只使用前
三个 source。

### 14.2 phase gain

对 FBBR/ServiceFair 的 usable TrustedBw：

\[
BaselinePacing=PhaseGain\times TrustedBw
\]

time-waveform Cruise baseline 的 phase gain 固定为 1。V3 valid Reference
path 也使用同一 phase-gain 规则；native path 保留 native BBRv2 的 pacing
limit 和 phase 结果。

### 14.3 waveform offset

\[
PacingTarget=
FloorAdd(BaselinePacing,Offset_{wave},MinimumRate)
\]

当没有 waveform、Trusted、Reference、V1 search 或 waveform Cruise
baseline 时，直接返回 `native_pacing`，避免重新构造 native 数值。

legacy spectral Cruise 的既有 `cruise_baseline_cap_bps_` 若启用，会同时
clamp target/base 的 wave 前 baseline。V4 没有新增该 cap。

### 14.4 不受 envelope 反馈

以下量不会进入 `PacingTarget`：

- `I_service`；
- `I_probe+`；
- `I_env`；
- `I_cap`；
- `service_limited`；
- `cap_binding`；
- `service_restriction`；
- `actual_inflight - cap`。

## 15. PacingBaseTarget

`PacingBaseTarget` 与 `PacingTarget` 在同一次 `PacingRate()` 调用中生成。
它使用相同：

- baseline/reference；
- native bandwidth；
- native pacing result；
- phase；
- native clamp；
- legacy Cruise cap；
- rate unit；
- minimum pacing floor；
- integer conversion；
- waveform offset；
- 最终 rounding。

唯一变化是移除所有正向 probing。

### 15.1 phase base

\[
Gain_{phase,base}=\min(Gain_{phase},1)
\]

- gain 大于 1：移除超过 1 的正向 phase 增量；
- gain 等于 1：不变；
- gain 小于 1：保留完整负向 gain。

Reference path：

\[
BaseBeforeWave=
Gain_{phase,base}\times ReferenceBw
\]

native 正向 phase path：

\[
BaseBeforeWave=
\min(NativePacing,Gain_{phase,base}\times NativeBw)
\]

这保留 native clamp，不用固定比例推测 base。

### 15.2 waveform base

\[
Offset_{base}=\min(Offset_{wave},0)
\]

\[
PacingBaseTarget=
FloorAdd(BaseBeforeWave,Offset_{base},MinimumRate)
\]

- 正半周期的 offset 被移除；
- 负半周期完整保留；
- phase 和 waveform 都正向时，不分开估计两份 credit，而是使用两个最终
  rate 的真实差值。

### 15.3 最终不变量

native clamp 或 rounding 后执行：

\[
PacingBaseTarget=
\min(PacingBaseTarget,PacingTarget)
\]

因此始终保证：

\[
0<PacingBaseTarget\le PacingTarget
\]

## 16. Rate target history

### 16.1 数据结构

```cpp
struct FbbrRateSegment {
    QuicTime start;
    QuicTime end; // zero 表示当前 open segment
    QuicBandwidth target_rate;
    QuicBandwidth base_target_rate;
};
```

V4 私有容器：

```cpp
mutable std::deque<FbbrRateSegment> fbbr_v4_rate_history_;
```

### 16.2 记录规则

`RecordFbbrV4RateTargets(now, target, base)`：

1. 非 V4 直接返回；
2. target/base 必须 finite 且大于 0；
3. 先执行 `base = min(base, target)`；
4. history 为空时创建 open segment；
5. `now < open.start`：标记 rate history integrity invalid；
6. target 和 base 都未变化：不创建 segment；
7. 同一时间戳且当前为 open segment：就地覆盖 target/base；
8. 正常变化：把旧 segment 的 `end` 设为 now，再创建新 open segment；
9. 更新 last target/base。

同一时刻的多次 pacing 更新不会产生零时长段。

### 16.3 清理

`fbbr_v4_max_rtprop_seen_` 保存运行中见过的最大有效 RTprop。令：

\[
cutoff=now-MaxRtpropSeen
\]

清理时只要第二个 segment 的 start 仍不晚于 cutoff，就弹出第一个
segment。这样始终保留覆盖积分左边界的 anchor segment。

历史可以保留多于一个 RTprop，但不能少于完整积分所需的覆盖。

### 16.4 完整覆盖判定

`HasFullFbbrV4TargetHistory(now, rtprop)` 要求：

- 当前算法是 V4；
- rate history integrity valid；
- RTprop finite 且大于 0；
- history 非空；
- `now >= rtprop`；
- 每段 target/base finite、positive 且 `base <= target`；
- 从 `now-rtprop` 到 now 不存在时间 gap。

Reference validity 不参与该判定。

## 17. Planned inflight

窗口：

\[
W=[t-RTprop,t]
\]

对每个 segment：

\[
\Delta t_j=
\max\left(
0,
\min(t,end_j)-\max(t-RTprop,start_j)
\right)
\]

open segment 的 end 视为当前 t。

\[
\boxed{
I_{plan}(t)=
\sum_j\frac{R_{target,j}\Delta t_j}{8}
}
\]

实现使用：

\[
\frac{bps\times\mu s}{8\,000\,000}
\]

并用 `long double` 累加，最后 `llround` 到 bytes。非 finite、非正结果返回
0；超过 `QuicByteCount` 最大值时饱和。

V4 不使用：

```text
current target × RTprop
ReferenceBw × RTprop
MaxBw × RTprop
delivery-rate sample × RTprop
```

对相同 target history，V4 的 `I_plan` 与 V3 的积分定义完全相同。

## 18. Positive probe credit

每段的正向 probe rate：

\[
P_{probe,j}^{+}=
\max(0,R_{target,j}-R_{base,j})
\]

过去一个 RTprop：

\[
\boxed{
I_{probe}^{+}(t)=
\sum_j
\frac{
\max(0,R_{target,j}-R_{base,j})
}{8}\Delta t_j
}
\]

它包括：

- ProbeBW Refill/Up 等 `phase_gain > 1` 的最终增量；
- frequency waveform 正半周期；
- 两者同时存在时最终 target 相对最终 base 的完整差值；
- native clamp 和 pacing floor 后仍实际存在的正差。

它不包括：

- DOWN 或其他 `phase_gain < 1`；
- waveform 负半周期；
- 固定百分比 headroom；
- 额外 gain；
- 尚未实际进入 target 的“潜在 probing”。

示例：

\[
R_{target}=125Mbps,\quad
R_{base}=100Mbps,\quad
\Delta t=20ms
\]

\[
I_{probe}^{+}=
\frac{25\times10^6\times0.020}{8}
=62500\ bytes
\]

## 19. Delivered service history

### 19.1 数据源

V4 使用：

```cpp
model_.total_bytes_acked()
```

这是当前 BBR bandwidth sampler/model 已确认的累计 delivered/acked payload
counter。记录发生在 native ACK 更新之后。

不使用：

- application bytes；
- total sent bytes；
- retransmission duplicate bytes；
- 单个 delivery-rate sample；
- DRate max；
- TrustedBw；
- GuardBw。

### 19.2 数据结构

```cpp
struct FbbrDeliveredPoint {
    QuicTime timestamp;
    uint64_t cumulative_delivered_bytes;
    bool app_limited;
};
```

V4 私有容器：

```cpp
std::deque<FbbrDeliveredPoint> fbbr_v4_delivered_history_;
```

### 19.3 记录规则

`RecordFbbrV4DeliveredPoint(now, cumulative, app_limited)`：

1. 非 V4 直接返回；
2. history 为空时直接加入；
3. timestamp 逆序：永久标记 delivered history integrity invalid；
4. 同 timestamp：
   - cumulative 取最大值；
   - app-limited 做逻辑 OR；
5. 新 timestamp 且 cumulative 下降：
   - 视为 counter reset/新 generation；
   - 清空旧 history；
   - 记录 `last_counter_reset_time = now`；
6. 加入新点；
7. 按最大 RTprop 清理历史。

空 ACK vector 不记录点。重复 ACK 不会使 sampler counter 增长；同时间戳
merge 也不会重复累计。

### 19.4 step-function 语义

每个 delivered point 表示 sender 到该 ACK 时刻已经确认的累计 delivered。
两个 ACK 点之间：

```text
D(t) 保持上一个已知 counter
```

不在线性插值中间值，也不虚构尚未确认的 service。

### 19.5 清理和 anchor

令：

\[
cutoff=now-MaxRtpropSeen
\]

当 history 的第二个点仍 `<= cutoff` 时弹出第一个点。最终保留不晚于
边界的最新 step anchor，以及边界后的全部点。

## 20. Service history validity

`HasValidFbbrV4ServiceHistory()` 的结构条件：

- 当前算法为 V4；
- delivered history integrity valid；
- RTprop finite 且大于 0；
- history 非空；
- `now >= rtprop`；
- 存在 timestamp `<= now-rtprop` 的最近 anchor；
- `[now-rtprop,now]` 内没有 counter reset；
- 从 anchor 到 now counter 单调不减。

app-limited 条件：

- 当前 `model_.is_app_limited()` 必须为 false；
- anchor point 的 `app_limited` 必须为 false；
- `[now-rtprop,now]` 内所有 point 的 `app_limited` 必须为 false。

源码把 anchor 的 flag 也纳入污染判断，这是保守的 step-function 处理：
只有一个完整、可锚定的 clean RTprop 覆盖形成后才恢复 valid。

状态：

```text
service_history_valid = structural_valid && !app_limited_contaminated
```

没有 app-limited 时间阈值、宽限期或恢复 gain。

## 21. Service inflight

令：

\[
t_0=t-RTprop
\]

选择：

- `D(t0)`：timestamp 不晚于 \(t_0\) 的最新 point；
- `D(t)`：timestamp 不晚于当前 t 的最新 point。

然后：

\[
\boxed{
I_{service}(t)=
\max(0,D(t)-D(t_0))
}
\]

若缺少 anchor/current，或 current counter 小于 anchor，函数返回 0。

`I_service` 即使在 service history invalid 时仍可为 trace 计算，但不会
限制 envelope。

## 22. Service-consistent envelope

### 22.1 service budget

service history valid：

\[
\boxed{
I_{service\_budget}=
SatAdd(I_{service},I_{probe}^{+})
}
\]

加法溢出时饱和到 `QuicByteCount::max()`。

### 22.2 envelope

\[
\boxed{
I_{env}=
\begin{cases}
\min(I_{plan},I_{service\_budget}),
& service\_history\_valid\\
I_{plan},& otherwise
\end{cases}
}
\]

### 22.3 动态含义

稳定链路：

```text
recent service + positive probing ≈ or > plan
I_env = I_plan
V4 退化为 V3 planned projection
```

容量下降：

```text
ACK 反馈后的 delivered/RTprop 下降
I_service_budget < I_plan
I_env 自动收缩
ReferenceBw 不被直接修改
```

容量上升：

```text
已有 phase/waveform positive excess 形成 I_probe+
允许 envelope 先扩张
成功 probe 的 delivered 随后进入新的 I_service
```

service invalid：

```text
I_env = I_plan
不把应用空闲或损坏 history 当成容量下降
```

## 23. Native headroom 和 inflight cap

### 23.1 ACK aggregation

\[
ExtraAcked=model_.MaxAckHeight()
\]

这是 BBRv2 原生 ACK aggregation headroom。

### 23.2 offload

当前 DQC BBRv2 没有独立 TSO/offload byte budget：

\[
OffloadBudget=0
\]

不使用固定比例替代。

### 23.3 饱和加法

```text
cap = SatAdd(I_env, ExtraAcked)
cap = SatAdd(cap, OffloadBudget)
cap = max(cap, cwnd_limits().Min())
```

### 23.4 MSS 对齐

当前 packet quantum 为 `kDefaultTCPMSS`。若 cap 不是 MSS 的整数倍，则：

\[
I_{cap}=SatAdd(cap,MSS-(cap\bmod MSS))
\]

即向上对齐，不向下截断。

### 23.5 最终公式

\[
\boxed{
Cwnd_{final}=
\min(Cwnd_{native},I_{cap})
}
\]

在 native BBRv2 自身遵守 `cwnd_limits().Min()` 的前提下，最终 cwnd 也不
低于 MinPipeCwnd。

## 24. 激活、fallback 和 phase 行为

### 24.1 projection active

源码条件：

```cpp
projection_active =
    UsesFbbrServiceEnvelope() &&
    RTpropValid &&
    target_history_covers_full_rtprop &&
    drain_completed_ &&
    mode_ == Bbr2Mode::PROBE_BW;
```

明确不要求：

```text
ReferenceBw valid
service_history_valid
```

### 24.2 phase 表

| 状态/phase | shared history | shared cap | 行为 |
|---|---|---|---|
| Startup | 持续维护 target/delivered | 不启用 | native BBR |
| initial Drain/transition | 持续维护 | 不启用 | native BBR |
| ProbeBW Cruise | 持续维护 | history 满后启用 | envelope |
| ProbeBW Refill | 持续维护 | 启用 | 正 phase 可形成 probe credit |
| ProbeBW Up | 持续维护 | 启用 | 正 phase 可形成 probe credit |
| ProbeBW Down | 持续维护 | 启用 | 负 phase 不形成 credit |
| ProbeRTT | 持续维护 | 不启用 | native ProbeRTT |
| TrustedBw/reference unavailable | 持续维护 | 可启用 | 使用实际 bootstrap target |
| service invalid | 持续维护 | planned cap | `I_env=I_plan` |
| service valid | 持续维护 | service envelope | `min(plan,service+probe)` |

### 24.3 cap 解除

V4 不把 cap 写入持久模型。只要当前 snapshot 不再 active，或者 cap 大于
native cwnd，getter 立即恢复 native send allowance。

没有 hysteresis、恢复计时器或逐步放开。

## 25. 当前 inflight 高于 cap

如果：

\[
BytesInFlight>I_{cap}
\]

V4：

- 不丢弃已发送 packet；
- 不把 cap 抬到当前 inflight；
- 不执行 `max(final_cwnd,bytes_in_flight)`；
- 不创建 recovery state；
- 通过较小 cwnd 停止新增发送；
- 等待 ACK 使现有 inflight 自然排空。

## 26. 观测量和控制量隔离

### 26.1 纯观测公式

\[
PlanExcess=\max(0,I_{plan}-I_{service})
\]

\[
ServiceRestriction=\max(0,I_{plan}-I_{env})
\]

\[
RawQueueDebt=\max(0,BytesInFlight-I_{plan})
\]

\[
EnvelopeDebt=\max(0,BytesInFlight-I_{env})
\]

\[
EnforcedExcess=\max(0,BytesInFlight-I_{cap})
\]

这些量只进入 snapshot、trace、mean/P95，不进入：

- pacing；
- Reference；
- service budget；
- headroom；
- cwnd cap。

### 26.2 状态布尔量

```text
target_history_valid
service_history_valid
app_limited_contaminated
projection_active
service_limited
cap_binding
```

定义：

```text
service_limited =
    projection_active &&
    service_history_valid &&
    I_env < I_plan

cap_binding =
    projection_active &&
    I_cap < native_cwnd &&
    actual_inflight > I_cap
```

`cap_binding` 是严格的观测定义：不仅 cap 小于 native cwnd，还要求当前
inflight 已经高于 cap。

## 27. V4 snapshot 的完整计算顺序

`BuildFbbrV4EnvelopeSnapshot(native_cwnd, actual_inflight)`：

1. 保存 native cwnd 和 actual inflight；
2. 仅用于观察地选择当前 Reference/source；
3. 读取 last target/base；
4. 取得 RTprop；
5. 判断 target history 是否完整；
6. 完整时计算 `I_plan` 和 `I_probe+`；
7. 有 RTprop 时计算 `I_service`；
8. 检查 service history 和当前 app-limited；
9. 饱和计算 service budget；
10. valid 时取 `min(plan,budget)`，invalid 时用 plan；
11. 读取 `MaxAckHeight()`；
12. target history 完整时计算 cap；
13. 计算五个纯观测 debt/restriction；
14. 判断 projection active；
15. 判断 service limited；
16. 判断 cap binding。

Reference 在第 2 步只供 pacing source/trace；第 14 步不读取
`reference.valid`。

## 28. 所有 V4 私有变量

### 28.1 rate history/control

| 变量 | 类型 | 作用 | 是否进控制 |
|---|---|---|---|
| `fbbr_v4_rate_history_` | deque of `FbbrRateSegment` | target/base 分段历史 | 是 |
| `fbbr_v4_max_rtprop_seen_` | `TimeDelta` | 保证 history 至少覆盖见过的最大 RTprop | 是，影响历史保留 |
| `fbbr_v4_rate_history_integrity_valid_` | bool | timestamp/segment 完整性 | 是，控制 projection validity |
| `fbbr_v4_last_target_rate_` | bandwidth | 最新 target，snapshot/trace | 间接；积分用 history |
| `fbbr_v4_last_base_target_rate_` | bandwidth | 最新 base，snapshot/trace | 间接；积分用 history |

### 28.2 delivered history/control

| 变量 | 类型 | 作用 | 是否进控制 |
|---|---|---|---|
| `fbbr_v4_delivered_history_` | deque of `FbbrDeliveredPoint` | cumulative delivered step history | 是 |
| `fbbr_v4_delivered_history_integrity_valid_` | bool | timestamp/单调性完整性 | 是 |
| `fbbr_v4_last_counter_reset_time_` | `QuicTime` | reset 是否污染最近 RTprop | 是 |

### 28.3 snapshot 变量

| 变量 | 含义 | 是否进最终 cap |
|---|---|---|
| `reference` | 当前 Reference 值和 source | 通过 pacing 间接进入 plan；不作为 active 条件 |
| `pacing_target` | 最新 target | history 的观测摘要 |
| `pacing_base_target` | 最新 base | history 的观测摘要 |
| `plan_inflight` | target 精确积分 | 是 |
| `service_inflight` | cumulative delivered 差 | service valid 时是 |
| `positive_probe_credit` | 正向 target-base 精确积分 | service valid 时是 |
| `service_budget` | 饱和 service+probe | 是 |
| `envelope` | plan 和 service budget 的较小者 | 是 |
| `extra_acked` | native `MaxAckHeight()` | 是 |
| `inflight_cap` | MinPipe/MSS 后最终 cap | 是 |
| `native_cwnd` | native BBRv2 cwnd | 是，最终 min |
| `actual_inflight` | 最近实际 inflight | 否，只用于 debt/binding |
| `plan_excess` | plan-service | 否 |
| `service_restriction` | plan-envelope | 否 |
| `raw_queue_debt` | actual-plan | 否 |
| `envelope_debt` | actual-envelope | 否 |
| `enforced_excess` | actual-cap | 否 |
| 六个 bool | validity/active/limited/binding | validity/active 三项控制 fallback；其余观测 |

### 28.4 telemetry 当前状态

以下字段只做时间积分或 sample 汇总，不反向控制算法：

```text
fbbr_v4_telemetry_last_time_
fbbr_v4_telemetry_initialized_
fbbr_v4_telemetry_reference_source_
fbbr_v4_telemetry_projection_active_
fbbr_v4_telemetry_service_history_valid_
fbbr_v4_telemetry_app_limited_fallback_
fbbr_v4_telemetry_plan_only_fallback_
fbbr_v4_telemetry_service_limited_
fbbr_v4_telemetry_cap_binding_
fbbr_v4_telemetry_plan_inflight_
fbbr_v4_telemetry_service_inflight_
fbbr_v4_telemetry_probe_credit_
fbbr_v4_telemetry_extra_acked_
fbbr_v4_telemetry_service_restriction_
fbbr_v4_telemetry_enforced_excess_
```

### 28.5 telemetry 累计状态

```text
fbbr_v4_telemetry_total_us_
fbbr_v4_reference_trusted_us_
fbbr_v4_reference_guard_us_
fbbr_v4_reference_last_valid_us_
fbbr_v4_reference_invalid_us_
fbbr_v4_projection_active_us_
fbbr_v4_service_history_valid_us_
fbbr_v4_app_limited_fallback_us_
fbbr_v4_plan_only_fallback_us_
fbbr_v4_service_limited_us_
fbbr_v4_cap_binding_us_
fbbr_v4_plan_inflight_byte_us_
fbbr_v4_service_inflight_byte_us_
fbbr_v4_probe_credit_byte_us_
fbbr_v4_extra_acked_byte_us_
fbbr_v4_service_restriction_byte_us_
fbbr_v4_enforced_excess_byte_us_
```

### 28.6 P95 sample 和窗口计数

```text
fbbr_v4_plan_inflight_samples_
fbbr_v4_service_inflight_samples_
fbbr_v4_probe_credit_samples_
fbbr_v4_extra_acked_samples_
fbbr_v4_service_restriction_samples_
fbbr_v4_enforced_excess_samples_
fbbr_v4_window_ack_events_
fbbr_v4_window_cap_binding_events_
fbbr_v4_flow_summary_emitted_
```

全部为观测/输出用途。

## 29. 共享但会影响 V4 的变量

| 共享状态 | 来源 | 对 V4 的作用 |
|---|---|---|
| `trusted_bw_`, validity/source/fresh fields | frequency/time-waveform publication | 选择 Reference/pacing baseline |
| `guard_filter_stage1_/stage2_` | ACK clock Guard | Cruise 结束时的 fallback Reference |
| `fbbr_hybrid_srtt_low_` | inherited RTprop observation | V4 积分窗口长度 |
| `model_.MinRtt()` | native BBRv2 | RTprop fallback |
| `model_.MaxBandwidth()` | native BBRv2 | bootstrap/observer upper reference |
| `model_.total_bytes_acked()` | bandwidth sampler | service history counter |
| `model_.is_app_limited()` | bandwidth sampler | service validity |
| `model_.MaxAckHeight()` | BBRv2 ACK aggregation | cap headroom |
| `cwnd_` | native BBRv2 | final min 的 native cwnd |
| `cwnd_limits().Min()` | native sender | MinPipeCwnd |
| `mode_`, current ProbeBW phase | native BBRv2 | target gain和 cap activation |
| `drain_completed_` | BBRv2/FBBR lifecycle | 避免 Startup/initial transition 被覆盖 |
| `current_injection_baseline_bw_` | Cruise/reference mirror | time-waveform pacing baseline |
| `current_probe_amplitude_bps_` | existing waveform logic | target 和 probe credit |

## 30. 数值和异常处理

### 30.1 bandwidth

- target/base 必须 finite 且 positive；
- base 始终 clamp 到不高于 target；
- minimum pacing floor 防止负 offset 把 rate 降到 0；
- bandwidth 到 integer 的转换使用已有 QuicBandwidth API。

### 30.2 时间

- RTprop zero/infinite/non-positive：history invalid，projection inactive；
- timestamp 逆序：对应 history integrity invalid；
- overlap 非正：该 segment 跳过；
- open segment 的结束时间是 snapshot 当前时刻。

### 30.3 bytes

- 积分用 `long double`；
- 非 finite/非正积分结果返回 0；
- planned/probe 积分饱和到 `QuicByteCount::max()`；
- service+probe 使用显式饱和加法；
- envelope+headroom 使用显式饱和加法；
- MSS alignment 使用显式饱和加法；
- delivered counter 下降视为 reset，而不是 unsigned wrap 后的巨大 service。

### 30.4 app-limited

- 当前 app-limited 或窗口污染都使 service invalid；
- fallback 是 `I_env=I_plan`；
- 不把低 service 固化成 cap；
- 不新增延时阈值；
- 一个完整 clean RTprop 后自动重新 valid。

### 30.5 ACK silence/compression

算法不增加额外平滑：

- ACK silence 会让 step-function `D(t)` 暂时不增长；
- ACK compression 可能让最近 RTprop 的 confirmed service 偏高；
- `MaxAckHeight()` 保留 native ACK aggregation headroom；
- 这些影响由 trace 分析，不引入 V4 gain 或修正规则。

## 31. 完整伪代码

### 31.1 pacing 和 rate history

```text
function PacingRate(bytes_in_flight):
    native_target = NativeBbrPacingRate(bytes_in_flight)
    native_bw     = NativeBandwidthEstimate()
    phase_gain    = NativePacingGain()
    reference     = SelectExistingV3Reference()

    target_before_wave =
        ExistingV3PacingBaseline(
            native_target,
            native_bw,
            reference,
            phase_gain,
            current_waveform_baseline,
            existing_native_limits)

    base_phase_gain = min(phase_gain, 1)
    base_before_wave =
        SameBaselineAndLimitsButRemovePositivePhase(
            native_target,
            native_bw,
            reference,
            base_phase_gain)

    wave = ExistingTriangleWave(now)
    offset = ExistingAmplitude() * wave

    target = AddWithSameFloorAndRounding(
        target_before_wave, offset)

    base = AddWithSameFloorAndRounding(
        base_before_wave, min(offset, 0))

    base = min(base, target)

    if algorithm in {FBBR, FBBR-ServiceFair}:
        RecordRateTargets(now, target, base)

    return target
```

### 31.2 ACK 和 delivered history

```text
function OnCongestionEvent(event):
    NativeBbrOnCongestionEvent(event)
    RunExistingFrequencyAndWaveformObserver(event)

    if algorithm in {FBBR, FBBR-ServiceFair} and event.acked_packets not empty:
        RecordDeliveredPoint(
            event.time,
            model.total_bytes_acked,
            model.is_app_limited)

        UpdateFBBRServiceEnvelopeTelemetry(event.time, actual_inflight)
```

### 31.3 snapshot

```text
function BuildFBBRServiceEnvelopeSnapshot(native_cwnd, actual_inflight):
    rtprop = HybridSrttLow if valid else NativeMinRtt

    target_valid = HasFullTargetHistory(now, rtprop)
    if target_valid:
        plan  = Integral(target_rate, now-rtprop, now)
        probe = Integral(max(target-base, 0), now-rtprop, now)
    else:
        plan = 0
        probe = 0

    service = DeliveredAtOrBefore(now)
              - DeliveredAtOrBefore(now-rtprop)

    service_valid =
        ValidRtprop &&
        HasAnchor &&
        MonotonicCounter &&
        NoResetInWindow &&
        NoAppLimitedInWindow &&
        !CurrentAppLimited

    service_budget = SaturatingAdd(service, probe)

    if service_valid:
        envelope = min(plan, service_budget)
    else:
        envelope = plan

    extra_acked = NativeMaxAckHeight
    cap = AlignMssUp(
        max(MinPipeCwnd,
            SaturatingAdd(envelope, extra_acked)))

    active =
        algorithm in {FBBR, FBBR-ServiceFair} &&
        rtprop valid &&
        target_valid &&
        drain_completed &&
        mode == ProbeBW

    return all control and observation fields
```

### 31.4 final cwnd

```text
function GetCongestionWindow():
    native = native_cwnd
    snapshot = BuildFBBRServiceEnvelopeSnapshot(native, latest_actual_inflight)

    if !snapshot.projection_active:
        return native

    return min(native, snapshot.inflight_cap)
```

## 32. Trace

### 32.1 每个既有分类窗口追加字段

```text
v4_reference_bw
v4_reference_source
v4_plan_inflight
v4_service_inflight
v4_positive_probe_credit
v4_service_budget
v4_envelope
v4_extra_acked
v4_inflight_cap
v4_native_cwnd
v4_actual_inflight
v4_service_history_valid
v4_app_limited_contaminated
v4_projection_active
v4_service_restriction
v4_enforced_excess
v4_cap_binding_fraction
```

窗口 `cap_binding_fraction`：

\[
\frac{cap\ binding\ ACK\ snapshots}
{all\ ACK\ snapshots\ since\ previous\ window}
\]

输出窗口 trace 后清零窗口 ACK/binding counters。

### 32.2 每 flow 一次 summary

时间比例：

```text
reference trusted/guard/last-valid/invalid time ratio
projection_active_time_ratio
service_history_valid_time_ratio
app_limited_fallback_time_ratio
plan_only_fallback_time_ratio
service_limited_time_ratio
cap_binding_time_ratio
```

分布：

```text
mean/p95 plan_inflight
mean/p95 service_inflight
mean/p95 probe_credit
mean/p95 extra_acked
mean/p95 service_restriction
mean/p95 enforced_excess
```

mean 使用 byte×time / total time；P95 使用 ACK snapshot sample。

## 33. 默认配置

FBBR/ServiceFair 没有专属 capacity、flow-count、Cellular、fixed、queue、gain 或 recovery
参数。它们继承 `NS3.27/examples/CCconfig/fbbr_default.conf`。与 shared information layer 和
pacing 相关的当前默认值如下：

```text
default_modulation_freq_hz = 5.0
default_amplitude_mode = 4sr
default_fixed_amplitude_mbps = 10.0
pacing.minimum_rate_mbps = 1.0

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
cruise_detector.mode = time_waveform
waveform.recv_signal_mode = delivery_rate_latest

waveform.initial_settle_rtt_mult = 1.0
waveform.post_adjust_settle_rtt_mult = 1.0
waveform.negative_half_first = true
waveform.initial_window_periods = 2.0
waveform.extended_window_periods = 3.0
waveform.max_window_periods = 3.0

waveform.period_tolerance_ratio = 0.15
waveform.min_periodicity_correlation = 0.50
waveform.min_cycle_coverage_ratio = 0.85
waveform.masked_min_cycle_coverage_ratio = 0.50
waveform.local_slope_window_period_ratio = 0.05
waveform.min_local_slope_window_ms = 5.0
waveform.clip_min_duration_ratio = 0.15
waveform.clip_min_half_overlap_ratio = 0.75
waveform.clip_max_slope_ratio = 0.10
waveform.plateau_max_level_span_ratio = 0.15
waveform.plateau_extreme_distance_ratio = 0.15
waveform.delta_drate_amplitude_ratio = 0.50
waveform.delta_fallback_baseline_ratio = 0.25
waveform.adaptive_delta_fallback_baseline_ratio = 0.10

waveform.max_inconclusive_extensions = 1
waveform.inconclusive_signal_amplification_factor = 1.25
waveform.inconclusive_signal_amplification_max_ratio = 2.0
waveform.max_app_limited_sample_ratio = 0.25
waveform.max_interpolation_gap_period_ratio = 0.10

fbbr.regime.long_top_horizontal_duration_ratio = 0.20
fbbr.regime.long_bottom_horizontal_duration_ratio = 0.30
fbbr.regime.actuator.midpoint_trigger_ratio = 0.50
fbbr.wave_fidelity.no_wave_trigger_windows = 2
fbbr.wave_fidelity.retry_window_advance_periods = 1

waveform.activity.amplitude_noise_multiplier = 6.0
waveform.activity.min_level_ratio = 0.02
waveform.activity.step_noise_multiplier = 3.0
waveform.activity.min_normalized_step_slope = 3.5
waveform.activity.min_active_steps = 4
waveform.activity.min_active_step_ratio = 0.10
waveform.activity.min_directional_change_ratio = 0.20
waveform.activity.min_significant_path_ratio = 0.80
waveform.activity.min_slope_reversals = 1

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

waveform.repeated_clip_max_period_error_ratio = 0.15
waveform.repeated_clip_max_level_delta_ratio = 0.05
waveform.repeated_clip_contact_level_tolerance_ratio = 0.05
waveform.repeated_clip_min_contact_samples_per_cycle = 2
waveform.repeated_clip_min_total_contact_samples = 4
waveform.repeated_clip_min_contact_sample_ratio = 0.05
waveform.repeated_clip_min_contact_span_ratio_of_window = 0.50
waveform.repeated_clip_min_pooled_flat_fraction = 0.90
waveform.repeated_clip_min_verified_boundary_fraction = 0.75
waveform.repeated_clip_min_outside_excursion_ratio = 0.10
waveform.repeated_clip_min_extrapolated_overshoot_ratio = 0.05
waveform.repeated_clip_merge_gap_ratio = 0.025
waveform.repeated_clip_max_missing_gap_ratio = 0.05

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

fbbr.regime.period_tolerance_ratio = 0.20
fbbr.regime.min_periodicity_correlation = 0.50
fbbr.regime.periodic_upper_clip_is_hard_veto = true

trace.gate_trace_mode = round_only
trace.gate_trace_sample_interval_us = 10000
trace.enable_cruise_window_trace = true
trace.enable_trusted_bw_selection_trace = true
```

配置文件中还保留若干旧 Adaptive delta/queue-guard 和旧 waveform
compatibility key。它们不进入 FBBR shared service envelope，也不控制第 11
节 FBBR 固定定义的 Regime I/III baseline 执行器；不能据此推断存在 queue
target 或另一套 legacy baseline actuator。

## 34. Self-test 覆盖

当前 FBBR self-test 覆盖：

### 34.1 identity/isolation

- FBBR enum 与 V1/V3 不同；
- FBBR identity helper；
- FBBR-only history；
- V3/FBBR planned integral 一致性。

### 34.2 planned inflight

- 恒定速率；
- 分段速率；
- partial overlap；
- 完整 history 覆盖；
- zero/infinite RTprop。

### 34.3 probe credit

- 100/100 Mbps 得 0；
- 125/100 Mbps 持续 20 ms 得 62500 bytes；
- target 低于 base 时为 0；
- 正负半周期只累计正差；
- phase 与 waveform 叠加不重复计数。

### 34.4 service history

- cumulative counter 单调；
- 左边界 anchor；
- 同时间戳 merge；
- duplicate ACK；
- history 不足；
- counter reset；
- timestamp 逆序；
- interval/current app-limited；
- 一个完整 clean RTprop 后恢复。

### 34.5 envelope/cwnd

- service budget 大于 plan；
- service budget 小于 plan；
- service invalid 回退 plan；
- Reference invalid 仍可 active；
- final cwnd 不高于 native；
- MinPipeCwnd；
- inflight 高于 cap 不抬 cap；
- Startup/ProbeRTT native；
- 饱和 arithmetic。

V1 和 V3 self-test 的归一化输出 hash 与 FBBR 迁移前相同，证明 FBBR 没有
替换它们的算法入口。

## 35. 源码索引

| 内容 | 文件/函数 |
|---|---|
| 枚举 | `proto_types.h` / `kFBBR` |
| 工厂 | `proto_send_algorithm_interface.cc` |
| parser | `fbbr_4flow.cc`, `generic_p2p_switch_flows.cc` |
| FBBR/ServiceFair shared structs/state | `fbbr_sender.h` |
| identity | `IsFbbr()`、`IsFbbrServiceFair()`、`UsesFbbrServiceEnvelope()` |
| Reference | `SelectFbbrV3ReferenceBw()` |
| pacing target/base | `PacingRate()` |
| rate history | `RecordFbbrV4RateTargets()` |
| target coverage | `HasFullFbbrV4TargetHistory()` |
| plan | `ComputeFbbrV4PlannedInflightBytes()` |
| probe credit | `ComputeFbbrV4PositiveProbeCreditBytes()` |
| delivered record | `RecordFbbrV4DeliveredPoint()` |
| service validity | `HasValidFbbrV4ServiceHistory()` |
| service bytes | `ComputeFbbrV4ServiceInflightBytes()` |
| envelope | `ComputeFbbrV4EnvelopeBytes()` |
| headroom/cap | `ComputeFbbrV4InflightCapBytes()` |
| snapshot | `BuildFbbrV4EnvelopeSnapshot()` |
| final cwnd | `ApplyFbbrV4InflightEnvelope()`, `GetCongestionWindow()` |
| observer | `AnalyzeFbbrHybridWindow()` |
| classifier | `ClassifyFbbrHybridRegime()` |
| V3/FBBR info actuator | `ApplyFbbrHybridV3Classification()` |
| Guard | `UpdateGuardEstimatorFromCongestionEvent()` |
| Trusted publish | `PublishFbbrHybridCruiseTrustedBw()` |
| telemetry | `UpdateFbbrV4Telemetry()`, `EmitFbbrV4FlowSummary()` |
| self-test | `RunFbbrSelfTest()` |

## 36. FBBR service envelope 明确不包含的机制

第 11 节的 Regime I/III baseline 动作属于 FBBR 的 load executor；下列项目
则不是 FBBR service envelope 新增的控制机制：

- queue target；
- RTT/queue gradient pacing gain；
- V4 Drain；
- V4 recovery；
- capacity-specific 参数；
- flow-count 分支；
- Cellular 特判；
- fixed-link 特判；
- 新 TrustedBw estimator；
- 新 GuardBw estimator；
- 新 app-limited timeout；
- service smoothing gain；
- probe-credit gain；
- fixed inflight headroom；
- cap-to-Reference feedback；
- cap-to-MaxBw feedback；
- cap-to-inflight_hi/lo feedback；
- `max(final_cwnd,bytes_in_flight)`；
- V5。

## 37. 最终控制不变量

只要 FBBR 或 ServiceFair 的 shared service-envelope projection active：

\[
PacingBaseTarget\le PacingTarget
\]

\[
I_{probe}^{+}\ge0
\]

\[
I_{env}\le I_{plan}
\]

\[
I_{cap}\ge MinPipeCwnd
\]

\[
Cwnd_{final}\le Cwnd_{native}
\]

service history invalid：

\[
I_{env}=I_{plan}
\]

service history valid：

\[
I_{env}=
\min(I_{plan},I_{service}+I_{probe}^{+})
\]

最终唯一新增控制式始终是：

\[
\boxed{
Cwnd_{final}=
\min\left(
Cwnd_{native},
AlignMssUp\left[
\max\left(
MinPipeCwnd,
I_{env}+MaxAckHeight
\right)
\right]
\right)
}
\]

长期 ReferenceBw 决定目标发送速率，近期已确认 service 决定短期 envelope，
已有正向 probing 决定容量回升时的扩张信用。ServiceFair 只在第 38-45 节
所列的 baseline、Regime III 和 TrustedBw 边界加入控制，不回写 envelope。

## 38. FBBR-ServiceFair 的定位和不变量

`FBBR-ServiceFair` 不是第二套 BBR，也不替换 FBBR 的 target/base history、
delivered history、service envelope 或最终 cwnd cap。它复用第 4-37 节的
全部 FBBR 路径，并且只增加三类控制点：

1. 每个 Cruise 至多一次的 service/qdelay AIMD，用来修改
   `current_injection_baseline_bw_`；
2. Regime III 的 yield cap 与 service floor，用来限制 FBBR 的 raw
   overload candidate；
3. Cruise 结束时的 TrustedBw publication correction，用来让最终发布值与
   已完成的公平动作和最后一个有效 Regime 一致。

因此以下公式对两个算法完全一致：

\[
I_{plan}=\int PacingTarget
\]

\[
I_{probe}^{+}=\int\max(PacingTarget-PacingBaseTarget,0)
\]

\[
I_{env}=\min(I_{plan},I_{service}+I_{probe}^{+})
\]

\[
Cwnd_{final}=\min(Cwnd_{native},I_{cap})
\]

ServiceFair 不能直接修改 `I_service`、`I_probe+`、`I_env`、`I_cap`、
`MaxAckHeight`、native `MaxBw`、`inflight_hi/lo`、ProbeBW phase 或 ProbeRTT。
它也没有跨 Cruise 的独立公平速率；跨 Cruise 的唯一带宽状态仍是已发布的
`TrustedBw`。

## 39. ServiceFair 状态、采样和生命周期

### 39.1 私有状态

ServiceFair 只在 `kFBBRServiceFair` 下读取或更新以下状态：

| 状态 | 含义 |
|---|---|
| `service_fair_service_rate_` | 最近一个有效 RTprop 窗口的累计 delivered service rate |
| `service_fair_previous_service_rate_` | 上一次有效 service rate，仅用于变化率 trace |
| `service_fair_qdelay_ewma_` | 当前排队延迟 EWMA |
| `service_fair_previous_qdelay_ewma_` | 上一 Cruise 的 EWMA，用于上升趋势判断 |
| `service_fair_last_update_cruise_id_` | 保证每个 Cruise 最多一次 AIMD 更新 |
| `service_fair_last_valid_regime_this_cruise_` | 最后一个有效 I/II/III Regime，用于 publication correction |
| `service_fair_raw_regime_candidate_bps_` | FBBR Regime executor 给出的原始候选 |
| `service_fair_final_regime_candidate_bps_` | ServiceFair 约束后的最终候选 |

连接迁移时执行 `ResetFbbrServiceFairState()` 并清除 TrustedBw；native BBR
从非 Startup 重新进入 Startup 时也重置这些公平信号。重置时刻会阻止任何
跨越该时刻的 delivered-service 窗口被采用。

### 39.2 ACK 时更新的信号

只有 ServiceFair、ProbeBW 且存在 ACK 时，`OnCongestionEvent()` 在记录
FBBR delivered point 后调用 `UpdateFbbrServiceFairSignals()`。

排队延迟样本：

\[
q_{sample}=\max(SRTT-RTprop,0)
\]

首次样本直接初始化；之后使用：

\[
q_{ewma}=\frac{7q_{ewma,old}+q_{sample}}8
\]

service rate 使用同一个 RTprop 时间窗内的累计 delivered bytes：

\[
ServiceRate=\frac{8\,[Delivered(t)-Delivered(t-RTprop)]}{RTprop}
\]

它要求第 20 节已有的全部 service-history 条件：左边界 anchor、单调
counter、无 reset、窗口和当前均非 app-limited、以及完整 RTprop 覆盖。任一
条件不满足时 `service_fair_service_history_valid_` 为 false，控制器不能使用
该 service rate。

### 39.3 Cruise 入口

`EnterCruise()` 先按 FBBR 规则用 PreviousTrusted 或 native baseline 初始化
`current_injection_baseline_bw_`，然后调用 `RunFbbrServiceFairCycleUpdate()`。
同一 `cruise_id_` 的后续入口不会重复更新。

## 40. ServiceFair 每 Cruise AIMD 负载执行器

定义：\(R=RTprop\)，\(B\) 为当前 injection baseline，\(q=q_{ewma}\)。

加性步长为：

\[
\alpha=\frac{0.5\times8\times MSS}{R}\quad bits/s
\]

当前实现的硬阈值是：

\[
q_{low}=\max(1ms,0.03R)
\]

\[
q_{high}=\max(2ms,0.10R)
\]

\[
q_{trend}=q_{ewma}-q_{ewma,previous},\quad
T_{trend}=q_{low}
\]

乘性系数和绝对下限为：

```text
beta = 0.995
minimum_service_fair_rate = 1 Mbps
```

进入函数即写入 `service_fair_last_update_cruise_id_`，因此包括 skip 在内，
同一 Cruise 都不会再次执行 AIMD。判断优先级如下：

1. baseline 无效：`SKIP_INVALID_HISTORY`；
2. 当前 app-limited 且 service history 无效：`SKIP_APP_LIMITED`；
3. RTprop、qdelay EWMA、service history、service rate 或 `alpha` 任一无效：
   `SKIP_INVALID_HISTORY`；
4. 否则进入下表的 AIMD 判断。

两种 skip 都保持 baseline 不变。

满足前提后按以下顺序执行：

| 条件 | 动作 | 新 baseline |
|---|---|---|
| `q > q_high`，或已有上轮 EWMA 且 `q_trend > T_trend` | `MULTIPLICATIVE_DECREASE` | `min(B, max(1 Mbps, 0.995B))` |
| `q < q_low` | `ADDITIVE_INCREASE` | `B + alpha`，若 MaxBw 有效则不高于 `max(B, MaxBw)` |
| 其他 | `HOLD` | `B` |

更新后，控制器保存当前 qdelay/service rate 为下一 Cruise 的 previous 值，
增加 `service_fair_cycle_count_`，并记录 service-rate change：

\[
\frac{ServiceRate_{current}}{ServiceRate_{previous}}-1
\]

该变化率是 trace，不是另一个控制分支。

## 41. Regime I/II/III 在 ServiceFair 下的执行

ServiceFair 的负载判定树与 FBBR 完全相同：仍使用第 10 节 N01-N16，仍要求
相同的 DRate/SRTT 有效性和 N12/N16 fallback。差异只出现在分类已经给出
I/II/III 后。

### 41.1 Inconclusive、无 DRate 或无合法 Regime II delivered interval

与 FBBR 相同：baseline hold、TrustedBw hold、settle 后重采。公平控制不把
无效样本伪装成负载结论。

### 41.2 Regime I / UNDERLOAD

先使用第 11.3 节的 FBBR raw candidate（`maxdrate`、MaxBw midpoint 或
`1.02B`）。ServiceFair 不对 Regime I 再做额外削减；它记录 raw/final
candidate 相同，并把该 Regime 作为本 Cruise 最后有效 Regime。

### 41.3 Regime II / FULL_LOAD

FBBR 的 Regime II 语义不变。特别是已经经历 I/III 后，唯一可信的候选是
窗口边界累计 delivered rate。该候选在 TrustedBw publication 中优先级最高，
ServiceFair 不得用公平 AIMD、MaxBw 或 service floor 改写它。

### 41.4 Regime III / OVERLOAD

先计算第 11.4 节的 FBBR raw candidate \(C_{raw}\)。ServiceFair 再应用：

\[
C_{yield}=\beta B
\]

\[
C_0=\min(C_{raw},C_{yield})
\]

当 service history 有效且 \(S=ServiceRate>0\) 时：

\[
C_{floor}=\min(C_{yield},0.98S)
\]

\[
C_{final}=\min(C_{yield},\max(C_0,C_{floor}))
\]

没有有效 service history 时不计算 \(C_{floor}\)，但 \(C_{yield}\) 仍然
约束 candidate。这样 Regime III 一定不会高于 `0.995 * baseline`，同时在
已确认 service 合理时避免把 baseline 压到远低于近期实际服务量。

## 42. ServiceFair 的 TrustedBw publication 修正

FBBR 的正常 Cruise 候选优先级仍是 Regime II cumulative delivery、spectral
candidate、Guard、PreviousTrusted、native fallback。`ApplyFbbrServiceFairTrustedBwCorrection()`
在候选选出后执行以下有限修正：

| 条件 | 最终 TrustedBw |
|---|---|
| 合法 Regime II cumulative-delivery candidate | 原样保留，绝不修正 |
| 最后有效 Regime 是 I | `max(candidate, final_baseline)`，再不高于有效 MaxBw |
| 最后有效 Regime 是 III | `min(candidate, final_baseline)`；若 service 有效，可再抬至不超过 baseline 的 `0.98 * service_rate` |
| 未出现有效 Regime，但本 Cruise 完成 AI 或 MD | `final_baseline` |
| HOLD、skip 或无有效历史 | 原候选不变 |

所有被修正的结果仍受 minimum pacing rate 下限保护。该 publication correction
只修改将要发布的 `TrustedBw`；不回写历史 target/base、delivered history 或
已生成的 envelope snapshot。

## 43. ServiceFair trace、可观测性和失败路径

每次 ServiceFair 事件以 `SERVICE_FAIRNESS` 写入既有 load trace。事件为：

```text
CYCLE_UPDATE       每 Cruise AIMD/hold/skip
REGIME_EXECUTOR     I/II/III 或 invalid 分类执行后
TRUSTED_PUBLISH     Cruise 最终候选发布时
```

每行字段顺序为：

```text
time_s
flow_id
cruise_id
event
classification
baseline_bps
baseline_valid
cycle_count
action
qdelay_ewma_ms
qdelay_trend_ms
service_rate_bps
service_rate_change
alpha_bps
beta
raw_regime_candidate_bps
final_regime_candidate_bps
```

`action` 只可能为：

```text
NOT_RUN
ADDITIVE_INCREASE
MULTIPLICATIVE_DECREASE
HOLD
SKIP_APP_LIMITED
SKIP_INVALID_HISTORY
```

排障时必须优先区分：

- `SKIP_APP_LIMITED`：采样本身不具备容量含义，不应解读为公平退让；
- `SKIP_INVALID_HISTORY`：RTprop、qdelay、anchor、reset 或 delivered
  history 的任一条件不足；
- `HOLD`：输入有效，但 qdelay 位于允许带内且没有过快增长；
- Regime II candidate：即便此前发生过 AIMD，也必须以合法 delivered
  measurement 为准；
- Regime III：检查 `raw_regime_candidate_bps`、`final_regime_candidate_bps`
  和 service rate，确认 yield cap/floor 是否生效。

## 44. FBBR 与 ServiceFair 的联合伪代码

```text
function OnCongestionEvent(event):
    NativeBbrOnCongestionEvent(event)
    UpdateFBBRObserverAndWaveformHistory(event)

    if algorithm in {FBBR, FBBR-ServiceFair} and event.has_ack:
        RecordTargetAndDeliveredHistoryForServiceEnvelope(event)

    if algorithm == FBBR-ServiceFair and event.has_ack:
        q_sample = max(SRTT - RTprop, 0)
        qdelay_ewma = EWMA_7_8(qdelay_ewma, q_sample)
        service_history_valid, service_rate =
            ValidateAndMeasureDeliveredOverRtprop()

    if in ProbeBW Cruise:
        analysis = RunTimeWaveformStateMachine()
        classification = ClassifyN01ToN16(analysis)
        raw = ExecuteFBBRRegime(classification)

        if algorithm == FBBR-ServiceFair and classification == OVERLOAD:
            baseline = ApplyYieldCapAndServiceFloor(
                raw, baseline_before, service_rate, service_history_valid)
        else:
            baseline = raw

function EnterCruise():
    baseline = PreviousTrustedOrNativeFBBRBaseline()

    if algorithm == FBBR-ServiceFair and not updated_this_cruise:
        if valid_rtprop_and_qdelay_and_service:
            if excessive_or_rising_qdelay:
                baseline = max(1Mbps, 0.995 * baseline)
            else if low_qdelay:
                baseline = min(baseline + alpha, max(baseline, MaxBw))
            else:
                baseline = baseline
        mark_updated_this_cruise()

function FinalizeCruise():
    candidate = SelectFBBRTrustedBwCandidate()
    if algorithm == FBBR-ServiceFair:
        candidate = CorrectCandidateForLastRegimeAndFairness(candidate)
    PublishTrustedBw(candidate)

function GetCongestionWindow():
    return ApplySharedFBBRServiceEnvelope(native_cwnd)
```

## 45. 配置、自测和源码索引

FBBR 与 ServiceFair 共用 `examples/CCconfig/fbbr_default.conf`。当前没有
ServiceFair 专属的外部配置键；其 `beta=0.995`、minimum rate `1 Mbps`、
qdelay 比例、EWMA 系数和 \(\alpha\) 公式是源码中的固定算法语义。

运行入口：

```bash
./waf --run "fbbr_4flow --algo=FBBR"
./waf --run "fbbr_4flow --algo=FBBR-ServiceFair"
./waf --run "fbbr_4flow --fbbrSelfTest=true"
./waf --run "fbbr_4flow --fbbrServiceFairSelfTest=true"
```

`RunFbbrSelfTest()` 覆盖 FBBR identity、planned inflight、probe credit、
service history、envelope、Regime 和 cwnd 不变量。`RunFbbrServiceFairSelfTest()`
覆盖：

- ServiceFair 只继承 FBBR envelope；
- `alpha`、Regime III beta/yield、service floor 的精确数值；
- 每 Cruise 一次的 AI/MD 与 qdelay/service 信号；
- inconclusive window 不覆盖已完成的公平更新；
- ProbeRTT / connection migration 不保留陈旧公平信号；
- Regime I/III 的 TrustedBw publication correction。

主要源码对应关系：

| 内容 | 函数或状态 |
|---|---|
| FBBR/ServiceFair identity | `IsFbbr()`、`IsFbbrServiceFair()`、`UsesFbbrServiceEnvelope()` |
| 共享 load 判定 | `ClassifyFbbrHybridRegime()`、`AnalyzeFbbrHybridWindow()` |
| FBBR Regime executor | `ComputeFbbrV4InjectionBaseline()`、`ApplyFbbrHybridV4Classification()` |
| FBBR envelope | `BuildFbbrV4EnvelopeSnapshot()`、`ApplyFbbrV4InflightEnvelope()` |
| qdelay/service signal | `UpdateFbbrServiceFairSignals()` |
| Cruise AIMD | `RunFbbrServiceFairCycleUpdate()` |
| Regime III 保护 | `ApplyFbbrServiceFairRegimeIIIControlLimit()` |
| TrustedBw 修正 | `ApplyFbbrServiceFairTrustedBwCorrection()` |
| reset/trace | `ResetFbbrServiceFairState()`、`EmitFbbrServiceFairTrace()` |
