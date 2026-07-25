# FBBR-hybirdv4 Service-Consistent Inflight Envelope 报告

## 1. 结论摘要

本轮在不创建 Git 分支、不修改 V1/V3 既有控制语义、不增加任何
gain、阈值、平滑、恢复状态机或场景特判的前提下，新增了独立算法
`FBBR-hybirdv4`，内部枚举为 `kFBBRHybridV4`。

实现、编译、自测试、隔离 hash、sanity、smoke、人工动态容量、四组正式
实验和离线分析均已完成。本轮没有实现 V5，也没有在实验失败后继续调参。

最终结论不是“全部验收通过”：

- Fixed 4 通过全部目标，V4 吞吐与 V3 基本相同，Avg/P95 queue 从
  13.269/13.363 ms 降到 9.724/9.900 ms。
- 人工 100→40→100 Mbps 场景证明 delivered service 会收缩，已有
  positive probing 能恢复吞吐，没有低速自锁。
- Fixed 32 未通过 queue 和 Jain 目标。
- Cellular Taxi 4/8 的吞吐、Jain、Loss 均通过，但 queue 目标未通过。
- Taxi 中 service envelope 确实显著低于 plan；失败的主要证据不是
  delivered history 没有收缩，而是 `ExtraAcked` 占最终 cap 的比例过高，
  大量抵消了 service restriction。
- Fixed 32 中各流近期 service、probe credit、ExtraAcked 与既得吞吐份额
  高度相关，出现 per-flow ACK-clock 份额强化，符合失败类型 9。
- V4 没有在代码路径上绕过 Goertzel/time-waveform/Reference 发布，但正式
  运行的有效窗口数和 Reference 有效率显著下降。因此只能证明“观察器仍被
  调用且 waveform 构造保持”，不能声称“观察器动态行为与 V3 等价”。

对题目最后问题的明确回答是：

> **不能完整做到。** 当前 Service-consistent inflight envelope 能保持
> Fixed 4 静态性能，能在人工容量下降时限制 plan，并能依靠已有 probing
> 在容量回升后恢复吞吐；但是它没有保持 Fixed 32 的公平性/队列，也没有把
> Taxi 4/8 的队列压到目标以内。它没有直接修改长期 ReferenceBw，但
> ExtraAcked headroom 和 per-flow ACK-clock 份额强化使完整目标未实现。

## 2. V1 / V3 / V4 算法隔离

三个入口同时存在：

| 外部名称 | 内部枚举 | 控制含义 |
|---|---|---|
| `FBBR-hybrid` | `kFBBRHybrid` | 原 Gradient-Matched V1 |
| `FBBR-hybridv3` | `kFBBRHybridV3` | 原 Model-Consistent Inflight Projection |
| `FBBR-hybirdv4` | `kFBBRHybridV4` | Service-Consistent Inflight Envelope |

隔离方式：

- `IsFbbrHybridV4()` 只匹配 `kFBBRHybridV4`。
- `IsFbbrProjectionObserver()` 只用于共享 V3/V4 信息层入口。
- V3 的 target-only history、积分函数、cap 函数和激活条件保持原实现。
- V4 使用独立 `fbbr_v4_rate_history_` 和
  `fbbr_v4_delivered_history_`。
- `RecordFbbrV4RateTargets()`、`RecordFbbrV4DeliveredPoint()`、
  `UpdateFbbrV4Telemetry()` 均先检查 `IsFbbrHybridV4()`。
- 正式目录中 V1/V3 的 V4 summary 文件数均为 0；V4 分别产生
  4、32、4、8 个逐流 summary。
- 没有全局开关把 V3 替换成 V4。

修改前、修改后使用同一归一化方法删除 Waf 噪声行，hash 完全相同：

| 算法 | 修改前 SHA-256 | 修改后 SHA-256 | 结果 |
|---|---|---|---|
| V1 | `cc5643738e2167c08f03fe9c97425ade508b9a9f4b80361f6e503c28d6f3c36b` | 同左 | PASS |
| V3 | `47884c51abeb21e3ccfa50d70ca259b6263e33efcb48a320da4a9a0d0fafff34` | 同左 | PASS |

归一化命令仅删除：

```text
^Waf:
^'build' finished
^Build commands will be stored
```

因此 hash 证明的是同一 self-test 输入下 V1/V3 的文本输出逐字节不变。

## 3. 修改文件与主要函数

### 核心枚举、工厂和发送路径

- `NS3.27/src/dqc/model/thirdparty/include/proto_types.h`
  - 追加 `kFBBRHybridV4`，没有重排旧枚举。
- `NS3.27/src/dqc/model/thirdparty/congestion/proto_send_algorithm_interface.cc`
  - 增加 V4 独立工厂 case。
- `NS3.27/src/dqc/model/dqc_sender.cc`
  - 把 V4 识别为 FBBR/BBRv2 sender。
  - flow 结束时只对 V4 调用 `FinalizeFbbrV4Trace()`。
- `NS3.27/src/dqc/model/thirdparty/congestion/quic_bbr2_misc.h`
  - 暴露 sampler 当前 `is_app_limited()` 状态。

### V4 控制实现

- `NS3.27/src/dqc/model/thirdparty/congestion/fbbr_sender.h`
  - `FbbrRateSegment`
  - `FbbrDeliveredPoint`
  - `FbbrV4EnvelopeSnapshot`
  - V4 history、telemetry 和 helper 声明。
- `NS3.27/src/dqc/model/thirdparty/congestion/fbbr_sender.cc`
  - `IsFbbrHybridV4()`
  - `IsFbbrProjectionObserver()`
  - `RecordFbbrV4RateTargets()`
  - `RecordFbbrV4DeliveredPoint()`
  - `HasFullFbbrV4TargetHistory()`
  - `HasValidFbbrV4ServiceHistory()`
  - `ComputeFbbrV4PlannedInflightBytes()`
  - `ComputeFbbrV4PositiveProbeCreditBytes()`
  - `ComputeFbbrV4ServiceInflightBytes()`
  - `ComputeFbbrV4EnvelopeBytes()`
  - `ComputeFbbrV4InflightCapBytes()`
  - `BuildFbbrV4EnvelopeSnapshot()`
  - `ApplyFbbrV4InflightEnvelope()`
  - `UpdateFbbrV4Telemetry()`
  - `EmitFbbrV4FlowSummary()`
  - `RunFbbrHybridV4SelfTest()`

### trace、命令行与分析

- `NS3.27/src/dqc/model/dqc_trace.h/.cc`
  - 在既有分类窗口末尾追加 V4 字段。
  - 增加每 flow 一次的 V4 summary。
- `NS3.27/scratch/fbbr_4flow.cc`
  - V4 parser 和 `--fbbrHybridV4SelfTest`。
- `NS3.27/scratch/generic_p2p_switch_flows.cc`
  - 外部名称 `FBBR-hybirdv4`。
- `NS3.27/examples/ConcurrentFlow/run_4cc_comparison.py`
  - V4 选择、参数、目录和 config。
- `NS3.27/examples/ConcurrentFlow/plot_4cc_comparison.py`
  - V4 图例。
- `NS3.27/scripts/analyze_fbbr_service_envelope_v4.py`
  - 正式指标、V4 分布、频域完整性、Taxi transition、人工动态容量、
    acceptance 和失败归因所需离线数据。

## 4. Reference 和 pacing target

V4 复用 V3 的 Reference 选择代码：

\[
ReferenceBw =
\begin{cases}
TrustedBw,& valid\\
GuardBw,& valid\\
PreviousTrusted,& 现有有效期内\\
invalid,& 否则
\end{cases}
\]

当 Reference invalid 时，`PacingRate()` 原有逻辑仍会产生 native/V1
bootstrap pacing target。V4 的 projection 激活不再依赖
`snapshot.reference.valid`，而 V3 的
`BuildFbbrV3ProjectionSnapshot()` 仍明确要求：

```cpp
snapshot.reference.valid &&
HasFullFbbrV3RateHistory(...) &&
drain_completed_ &&
mode_ == Bbr2Mode::PROBE_BW
```

V4 没有改写 TrustedBw、GuardBw、PreviousTrusted、MaxBw、RTprop、
inflight_hi/lo 或 phase progression。Envelope 路径也没有向 Reference
发布函数回写任何值。

## 5. PacingBaseTarget 构造

`PacingTarget` 继续走 V3 的同一最终路径。V4 在同一调用中同时构造
`PacingBaseTarget`：

1. baseline/reference、rate unit、phase、native pacing limit 与 target
   相同；
2. phase gain 使用 `min(phase_gain, 1)`；
3. waveform 使用同一个最终整数 `offset_bps`，base 只保留
   `min(0, offset_bps)`；
4. DOWN 和负半周期保持，不抬回 1；
5. 调用同一个 `AddPacingOffsetWithFloor()`；
6. 最后执行 `min(base_target, target)`。

因此：

\[
PacingBaseTarget(t) \le PacingTarget(t)
\]

实现没有固定比例近似，也没有额外 probe gain。phase 和 waveform 同时
为正时，credit 只从最终 target/base 差值积分一次。

## 6. target、probe 和 delivered histories

### Rate segment

```cpp
struct FbbrRateSegment {
    QuicTime start;
    QuicTime end;
    QuicBandwidth target_rate;
    QuicBandwidth base_target_rate;
};
```

- target 或 base target 变化时关闭旧段、开启新段。
- 同一时间戳更新 open segment，不产生零时长段。
- 清理时保留覆盖积分边界的 anchor segment。
- V3 的 `FbbrV3PacingTargetSegment` 和 target-only history 未改。

### Delivered point

```cpp
struct FbbrDeliveredPoint {
    QuicTime timestamp;
    uint64_t cumulative_delivered_bytes;
    bool app_limited;
};
```

记录值来自 BBR model 在 ACK 更新后的 `total_bytes_acked()`，不是发送字节、
应用字节、retransmission 字节或 rate sample 推算值。

- 空 ACK/重复 ACK 不增加累计 delivered。
- 同时间戳保留最大 counter，并对 app-limited 做 OR。
- timestamp 逆序会使 history invalid。
- counter reset 清空旧 generation，并在 reset 仍处于最近 RTprop 时
  保持 invalid。
- 清理时保留 `t-RTprop` 之前最近的 step anchor。

## 7. 三个核心积分/差分

### Planned inflight

\[
I_{plan}(t)=
\sum_j \frac{R_{target,j}}{8}\Delta t_j
\]

每个 \(\Delta t_j\) 是 segment 与 \([t-RTprop,t]\) 的精确 overlap，
没有使用 `current_rate × RTprop`。

### Positive probe credit

\[
I_{probe}^{+}(t)=
\sum_j
\frac{\max(0,R_{target,j}-R_{base,j})}{8}\Delta t_j
\]

它包含 phase UP/Refill 和 waveform 正半周期叠加后的真实 excess；负 phase
和负半周期不产生 credit。

### Delivered service

令 \(t_0=t-RTprop\)，取不晚于 \(t_0\) 的最近 step anchor：

\[
I_{service}(t)=\max(0,D(t)-D(t_0))
\]

没有在两个 ACK 点之间做线性插值。

## 8. App-limited 处理

`HasValidFbbrV4ServiceHistory()` 同时检查：

- RTprop 有效；
- 存在 `timestamp <= t-RTprop` 的 anchor；
- 累计 delivered 单调；
- 最近 RTprop 内没有 counter reset；
- 区间内没有 app-limited point；
- 当前 sampler 不处于 app-limited。

任一条件不满足：

\[
I_{env}=I_{plan}
\]

没有增加 app-limited 时间阈值。四组正式实验的
`app_limited_fallback_time_ratio` 均为 0，未发现误触发。

## 9. Envelope、native headroom 和最终 cwnd

service history valid 时：

\[
I_{service\_budget}=I_{service}+I_{probe}^{+}
\]

\[
I_{env}=\min(I_{plan},I_{service\_budget})
\]

invalid 时：

\[
I_{env}=I_{plan}
\]

Native headroom：

\[
NativeHeadroom=model.MaxAckHeight()+0
\]

当前 DQC BBRv2 没有独立 offload budget，所以 `OffloadBudget=0`。V4 直接
复用 V3 已验证的 saturating add、`cwnd_limits().Min()` 和 MSS 向上对齐：

\[
I_{cap}=\max(MinPipeCwnd,I_{env}+NativeHeadroom)
\]

最终唯一控制式：

\[
Cwnd_{final}=\min(Cwnd_{native},I_{cap})
\]

`GetCongestionWindow()` 对 V4 只调用
`ApplyFbbrV4InflightEnvelope(cwnd_)`。它不使用
`max(final_cwnd, bytes_in_flight)`；current inflight 高于 cap 时只停止新增
发送，不丢弃 packet，也不把 cap 写回 inflight_hi/lo。

Startup、initial Drain/Startup transition 和 ProbeRTT 都不满足 V4
`mode == PROBE_BW && drain_completed`，因此保持 native 行为。cap 解除后
getter 立即返回 native allowance。

## 10. trace 和 flow summary

只在既有有效频域/分类窗口末尾追加：

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

每 flow 结束一次 summary 包含四类 Reference ratio、projection/service/
fallback/service-limited/binding ratio，以及 plan/service/probe/ExtraAcked/
restriction/excess 的 mean/P95。

没有新增逐 packet、逐 ACK 或独立高频文件日志。

## 11. 编译、自测试和隔离结果

| 项目 | 返回码 | PASS | FAIL |
|---|---:|---:|---:|
| `./waf build` | 0 | build success | 0 |
| V1 self-test | 0 | 70 | 0 |
| V3 self-test | 0 | 24 | 0 |
| V4 self-test | 0 | 30 | 0 |
| V1 normalized hash | 0 | 与修改前相同 | 0 |
| V3 normalized hash | 0 | 与修改前相同 | 0 |

V4 的 30 个测试覆盖：

- 独立枚举和算法 identity；
- constant、segmented、partial overlap planned inflight；
- V3/V4 相同 history 的 plan 相等；
- 100/100 Mbps credit=0；
- 125/100 Mbps、20 ms credit=62500 B；
- target<base、正负半周期、phase+waveform 不重复计数；
- delivered 单调、边界 anchor、同时间戳 merge、duplicate ACK；
- history 不足、counter reset、timestamp 逆序；
- app-limited interval/current 和一个完整 clean RTprop 后恢复；
- envelope 不限制、限制、invalid fallback；
- Reference invalid 仍激活；
- final cwnd 上下界、inflight 高于 cap 不抬 cap；
- Startup、ProbeRTT；
- zero RTprop、saturating arithmetic。

`python3 -m py_compile` 和 `git diff --check` 均返回 0。

## 12. Sanity、人工动态容量和 smoke

### 12.1 单流固定 100 Mbps / 5 BDP / 120 s

| 算法 | Throughput Mbps | Avg Q ms | P95 Q ms | Jain | Loss % |
|---|---:|---:|---:|---:|---:|
| V4 | 97.116 | 8.651 | 8.736 | 1.000000 | 0 |

相对 V3 报告的同场景约 97.916 Mbps，吞吐下降约 0.82%，满足不超过 1%。
V4 flow summary：

- projection active 99.455%；
- service history valid 99.917%；
- service-limited 1.268%；
- mean plan/service/probe = 617.5/510.7/125.6 KB；
- `service + probe` 均值约 636.3 KB，高于 plan；
- mean enforced excess 245 B，没有长期 minimum-cwnd 自锁。

### 12.2 人工 0–20 s 100M、20–40 s 40M、40–60 s 100M

总体：

| 场景 | 算法 | Throughput Mbps | Avg/P95 Q ms | Jain | Loss % |
|---|---|---:|---:|---:|---:|
| 1 flow | V3 | 57.479 | 26.444 / 228.322 | 1.000000 | 0 |
| 1 flow | V4 | 75.894 | 15.887 / 26.832 | 1.000000 | 0 |
| 4 flows | V3 | 75.630 | 65.302 / 237.154 | 0.829051 | 0 |
| 4 flows | V4 | 76.579 | 50.367 / 113.594 | 0.954234 | 0 |

分阶段：

| 场景 | 算法 | 5–20 s 100M throughput/Q | 20–40 s 40M throughput/Q | 40–45 s recovery throughput/Q | 45–60 s throughput/Q |
|---|---|---:|---:|---:|---:|
| 1 flow | V3 | 97.116 / 9.768 | 39.044 / 65.302 | 45.966 / 0.133 | 46.259 / 0.080 |
| 1 flow | V4 | 97.116 / 8.655 | 39.359 / 26.608 | 94.703 / 10.429 | 97.115 / 10.643 |
| 4 flows | V3 | 97.114 / 13.096 | 39.549 / 156.497 | 91.084 / 11.188 | 97.101 / 13.951 |
| 4 flows | V4 | 97.111 / 8.860 | 39.320 / 109.133 | 93.976 / 18.725 | 99.926 / 24.064 |

结论：

- 100→40 后 V4 的低容量队列显著低于 V3。
- 40→100 后 V4 单流在前 5 s 已恢复 94.7 Mbps，之后 97.1 Mbps；
  V3 单流仍约 46.3 Mbps。
- 4-flow V4 也恢复到约 100 Mbps。
- positive probing 足以恢复，没有永久低速自锁。
- Reference 没有被 envelope 代码直接写入；但是不同 inflight/ACK 反馈会
  改变后续观察和发布，所以不能把两算法的 Reference 数值假定为相同。

受“只在既有分类窗口追加字段”的日志约束，人工 transition 时刻 20/40 s
没有分类窗口行，无法从该实验给出每 ACK 的精确 cap 收缩时刻。本报告没有
用插值伪造该时间。

### 12.3 固定 smoke

| 场景 | Throughput Mbps | Avg/P95 Q ms | Jain | Loss % | RC |
|---|---:|---:|---:|---:|---:|
| 4 flows / 60 s | 97.114 | 8.802 / 8.873 | 0.996246 | 0 | 0 |
| 32 flows / 60 s | 98.467 | 22.609 / 24.296 | 0.778354 | 0 | 0 |

4-flow smoke 正常；32-flow smoke 已暴露公平性和 queue 回归。按要求没有
因此添加补丁或参数。

## 13. 正式实验方法和返回码

所有新 V1/V3/V4 正式实验：

- seed=1，runId=1；
- 同一 `fbbr_default.conf`；
- buffer=5 BDP；
- `dataGeneratorBatch=2048`；
- `streamBufferBytes=67108864`；
- process interval 100 µs；
- goodput interval 100 ms；
- bottleneck queue sample 200 µs；
- fixed：100 Mbps、4/32 flows、240 s、access/service 单向
  1/19 ms；
- Taxi：同一 1650 点 schedule、标称 128 Mbps、4/8 flows、180 s、
  access/service 单向 0/10 ms；
- throughput/capacity/util/queue/Jain 跳过前 5 s；
- Loss/OWD/Total GiB 使用 whole run。

一次早期诊断误用了 runner 新默认的 batch=2、stream buffer=0；发现与旧
`config.json` 不一致后，正式目录已用 2048/64 MiB 完整覆盖重跑。最终
V1/V3 Fixed 指标与旧报告逐位一致，证明正式配置已恢复。报告和
`formal_metrics.csv` 只使用重跑后的结果。

新 V1/V3/V4 共 12 个正式返回码均为 0。BBR-R/BBRv2 复用 2026-07-22
同配置结果，8 个返回码均为 0。因此正式对比的 20 个返回码全为 0。

## 14. 四场景完整指标

| 场景 | 算法 | Throughput Mbps | Avg Q ms | P95 Q ms | Jain | Loss % | OWD ms | Total GiB |
|---|---|---:|---:|---:|---:|---:|---:|---:|
| Fixed 4 | V1 | 97.641 | 12.946 | 24.422 | 0.925240 | 0 | 34.112 | 2.726 |
| Fixed 4 | V3 | 97.914 | 13.269 | 13.363 | 0.995225 | 0 | 34.430 | 2.734 |
| Fixed 4 | V4 | 97.914 | 9.724 | 9.900 | 0.998959 | 0 | 30.832 | 2.733 |
| Fixed 4 | BBR-R | 97.719 | 5.063 | 14.630 | 0.986803 | 0 | 26.461 | 2.728 |
| Fixed 4 | BBRv2 | 98.052 | 181.835 | 209.770 | 0.943277 | 1.727570 | 200.016 | 2.734 |
| Fixed 32 | V1 | 98.014 | 82.985 | 178.344 | 0.853085 | 0.003501 | 102.887 | 2.735 |
| Fixed 32 | V3 | 97.898 | 17.523 | 17.652 | 0.966755 | 0 | 38.908 | 2.735 |
| Fixed 32 | V4 | 99.505 | 25.411 | 26.826 | 0.790149 | 0 | 46.591 | 2.735 |
| Fixed 32 | BBR-R | 97.900 | 15.503 | 31.314 | 0.760189 | 0 | 36.832 | 2.735 |
| Fixed 32 | BBRv2 | 98.503 | 199.574 | 209.898 | 0.850406 | 3.796570 | 217.967 | 2.735 |
| Cell 4 | V1 | 42.303 | 73.393 | 416.988 | 0.848943 | 0 | 26.624 | 0.875 |
| Cell 4 | V3 | 42.561 | 121.320 | 528.901 | 0.784271 | 0 | 33.424 | 0.880 |
| Cell 4 | V4 | 44.243 | 131.213 | 445.490 | 0.925604 | 0 | 49.822 | 0.892 |
| Cell 4 | BBR-R | 41.805 | 64.575 | 231.549 | 0.925854 | 0 | 30.136 | 0.866 |
| Cell 4 | BBRv2 | 43.121 | 741.386 | 4792.732 | 0.842796 | 0.793566 | 174.166 | 0.891 |
| Cell 8 | V1 | 42.607 | 154.985 | 1007.408 | 0.818307 | 0 | 33.810 | 0.880 |
| Cell 8 | V3 | 43.124 | 188.427 | 1224.354 | 0.810170 | 0 | 45.756 | 0.887 |
| Cell 8 | V4 | 45.172 | 267.801 | 1394.640 | 0.911946 | 0 | 62.373 | 0.892 |
| Cell 8 | BBR-R | 41.955 | 85.242 | 364.943 | 0.844523 | 0 | 31.858 | 0.869 |
| Cell 8 | BBRv2 | 43.190 | 731.845 | 4192.689 | 0.684773 | 0.524525 | 182.410 | 0.891 |

V4 相对 V3 throughput：

- Fixed 4：−0.0002%；
- Fixed 32：+1.641%；
- Cell 4：+3.950%；
- Cell 8：+4.749%。

## 15. V4 projection、service 和分布

以下为逐流 summary 的跨流算术平均，P95 列是“各流 P95 的平均”。

| 场景 | Ref invalid | Projection | Service valid | Plan fallback | Service-limited | Cap binding |
|---|---:|---:|---:|---:|---:|---:|
| Fixed 4 | 1.253% | 99.760% | 99.958% | 0 | 1.536% | 0.099% |
| Fixed 32 | 47.173% | 99.766% | 99.955% | 0 | 31.920% | 14.654% |
| Cell 4 | 99.459% | 99.375% | 99.904% | 0 | 56.829% | 17.542% |
| Cell 8 | 99.334% | 99.314% | 99.813% | 0 | 53.040% | 18.882% |

| 场景 | Plan mean/P95 B | Service mean/P95 B | Probe mean/P95 B | ExtraAcked mean/P95 B |
|---|---:|---:|---:|---:|
| Fixed 4 | 151766 / 151146 | 130289 / 137123 | 30649 / 30229 | 6630 / 6809 |
| Fixed 32 | 26082 / 26042 | 20640 / 26390 | 5294 / 5208 | 5505 / 6070 |
| Cell 4 | 79521 / 109423 | 47963 / 87386 | 15981 / 21885 | 28273 / 52137 |
| Cell 8 | 38972 / 54400 | 25706 / 46522 | 7844 / 10880 | 21129 / 34577 |

| 场景 | Restriction mean/P95 B | Enforced excess mean/P95 B |
|---|---:|---:|
| Fixed 4 | 48 / 62 | 4 / 0 |
| Fixed 32 | 3144 / 12019 | 986 / 7815 |
| Cell 4 | 21538 / 59221 | 2953 / 35615 |
| Cell 8 | 9056 / 27954 | 1515 / 17395 |

V3/V4 projection active 对照：

| 场景 | V3 | V4 |
|---|---:|---:|
| Fixed 4 | 98.025% | 99.760% |
| Fixed 32 | 98.938% | 99.766% |
| Cell 4 | 47.944% | 99.375% |
| Cell 8 | 46.677% | 99.314% |

这直接验证 V4 不再要求 Reference valid；Taxi 中即使 Reference 约 99.4%
无效，plan projection 仍约 99.3% 激活。

## 16. Fixed 4 逐流与 Fixed 32 静态回归

### Fixed 4

| Flow | Throughput Mbps | Reference mean Mbps | Reference P50 Mbps |
|---:|---:|---:|---:|
| 1 | 25.250 | 23.906 | 23.906 |
| 2 | 25.034 | 23.869 | 23.869 |
| 3 | 24.365 | 22.044 | 22.044 |
| 4 | 23.265 | 21.997 | 21.997 |

全局 Jain=0.998959，Loss=0。V4 保持 V3 throughput，并降低稳定 queue。

### Fixed 32

V4 未通过：

- Avg Q 25.411 ms，高于 20 ms 目标，且比 V3 高 45.0%；
- P95 Q 26.826 ms，高于 20 ms 目标，且比 V3 高 52.0%；
- Jain 0.790149，低于 0.956；
- throughput 和 Loss 仍通过。

逐流 5 s 后平均吞吐范围为 1.630–9.139 Mbps，中位数 2.499 Mbps。吞吐
与各 envelope 量的 Pearson correlation：

| 量 | correlation |
|---|---:|
| service-limited time ratio | 0.313 |
| mean service inflight | 0.998 |
| mean probe credit | 0.993 |
| mean ExtraAcked | 0.953 |
| mean service restriction | 0.899 |

这说明已有高 ACK-clock 份额的流同时获得更多 delivered service、probe
credit 和 ExtraAcked；envelope 没有创造新份额，反而强化既得 per-flow
份额。该证据对应失败类型 9。

## 17. Cellular Taxi transition 对齐分析

### 17.1 分类窗口中的 envelope

每行是现有分类窗口的跨 flow/sample 均值；`Reference=0` 表示该窗口
Reference invalid，而不是 0 bps 控制 reference。

| 场景/窗口 | Capacity before→after Mbps | Plan B | Service B | Probe B | Budget B | Envelope B | Actual B | Queue ms | Ref Bps | Binding | Extra/cap |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| Cell4 down | 45.995→28.843 | 43178 | 14086 | 385 | 14471 | 14471 | 56554 | 123.328 | 0 | 25.13% | 71.04% |
| Cell4 stable | 52.119→52.358 | 43012 | 5992 | 0 | 5992 | 5992 | 63156 | 34.812 | 0 | 30.19% | 83.94% |
| Cell4 up | 27.938→40.057 | 43443 | 15490 | 426 | 15916 | 15916 | 51306 | 34.966 | 0 | 24.13% | 72.92% |
| Cell8 down | 27.899→20.232 | 26662 | 2699 | 912 | 3611 | 3611 | 37325 | 509.030 | 0 | 27.39% | 85.57% |
| Cell8 stable | 48.629→48.979 | 32346 | 7188 | 371 | 7559 | 7192 | 42287 | 107.020 | 0 | 21.81% | 77.63% |
| Cell8 up | 27.561→36.243 | 23666 | 9521 | 347 | 9868 | 9486 | 32877 | 58.877 | 0 | 18.05% | 66.74% |

下降窗口的第一条“观察到 restriction>0”的分类记录：

- Cell 4：transition 后 10.2 ms；分类记录均值 40.6 ms；
- Cell 8：transition 后 3.7 ms；分类记录均值 60.9 ms。

在这些记录中 envelope 已明显低于 plan；V3 plan cap 本身没有 service
收缩项。因此可以说 V4 在第一条可观察窗口已提前收缩。但是 Taxi 每约
100 ms 又发生一次容量变化，且 V4 字段只允许写到既有分类窗口，故这些是
“first observed latency”，不是精确的 per-ACK 因果延迟。精确提前多少
毫秒在当前最小 trace 下不可辨识，本报告不做线性插值。

### 17.2 容量下降后的密集 queue trace

用既有 200 µs bottleneck queue trace 对全部 520 个下降 transition 对齐。
transition 后 50–90 ms：

| 场景 | 算法 | Mean Q ms | P95 Q ms | Max Q ms |
|---|---|---:|---:|---:|
| Cell 4 | V1 | 108.142 | 417.309 | 3720.954 |
| Cell 4 | V3 | 192.607 | 906.780 | 6217.910 |
| Cell 4 | V4 | 205.937 | 782.469 | 6731.989 |
| Cell 8 | V1 | 180.933 | 743.785 | 5760.992 |
| Cell 8 | V3 | 223.182 | 892.677 | 5960.005 |
| Cell 8 | V4 | 358.302 | 1542.107 | 10867.674 |

回答动态问题：

1. **service envelope 是否先收缩：**是，分类窗口中 service 明显低于
   plan，first observed 为 10.2/3.7 ms；精确控制事件延迟不可由最小 trace
   识别。
2. **queue peak 是否下降：**Cell 4 的下降窗口 P95 比 V3 低，但 mean/max
   没有下降；Cell 8 的 mean/P95/max 全部更差。不能宣称整体 queue peak
   已解决。
3. **probe credit 是否允许恢复：**是。人工单流回升后恢复到 97.1 Mbps；
   Taxi 4/8 throughput 也分别比 V3 高 3.95%/4.75%。
4. **是否低速自锁：**人工单流、4-flow 和 Taxi throughput 均未出现；
   但 Fixed 32 出现的是 per-flow 公平份额强化，而非总吞吐自锁。
5. **app-limited fallback 是否误触发：**没有，四组正式 ratio 都为 0。
6. **ExtraAcked 占 cap 比例：**分类窗口 mean 为 Cell 4
   71%–84%、Cell 8 67%–86%；P95 为约 96%–99%。它几乎填满 cap，是
   restriction 没有转化为 queue 降幅的直接证据。

## 18. 频域完整性

### 18.1 构造路径

代码层面，V4 没有绕过：

- `RunWaveformCruiseStateMachine()`；
- Goertzel/time-waveform 分析；
- DRate/SRTT 特征；
- Regime I/II/III 和原 `/3` 分类阈值；
- Trusted/Guard 发布函数。

cap 只在 `GetCongestionWindow()` 最终 getter 生效，`PacingRate()` 返回的
target 不因 cap binding 被改写。Fixed harmonic 中
triangle fundamental/commanded amplitude：

- Fixed 4：V3 0.81114，V4 0.81300；
- Fixed 32：V3 0.80807，V4 0.81716。

这支持 waveform 数学构造保持。绝对 target amplitude 可因不同 Reference、
phase 和闭环行为而不同，不能要求逐样本相等。

### 18.2 窗口、Regime 和 score

| 场景/算法 | Windows | Valid | Regime I/II/III | Inconclusive | DRate P50/P95 | SRTT P50/P95 |
|---|---:|---:|---:|---:|---:|---:|
| Fixed4 V3 | 36 | 36 | 11/1/24 | 0 | .950/.994 | .950/.992 |
| Fixed4 V4 | 24 | 24 | 18/6/0 | 0 | .975/1.000 | .975/1.000 |
| Fixed32 V3 | 101 | 96 | 30/0/66 | 5 | .888/.986 | .775/.975 |
| Fixed32 V4 | 36 | 36 | 17/0/19 | 0 | .875/.975 | .750/.750 |
| Cell4 V3 | 608 | 471 | 75/7/406 | 120 | .900/1.000 | .875/1.000 |
| Cell4 V4 | 25 | 6 | 2/2/5 | 16 | .913/.925 | .825/.935 |
| Cell8 V3 | 1133 | 799 | 151/2/689 | 291 | .925/1.000 | .850/.975 |
| Cell8 V4 | 43 | 10 | 3/3/9 | 28 | .950/1.000 | .800/.935 |

Fixed target/actual harmonic：

| 场景 | 算法 | Target P50 bps | Actual P50 bps | Actual/target |
|---|---|---:|---:|---:|
| Fixed4 | V3 | 1,850,400 | 687,713 | 0.329 |
| Fixed4 | V4 | 1,756,881 | 967,042 | 0.530 |
| Fixed32 | V3 | 311,215 | 110,362 | 0.336 |
| Fixed32 | V4 | 262,124 | 103,091 | 0.463 |

V4 service-limited / non-service-limited：

| 场景 | Group | Windows | Target P50 bps | Actual/target P50 |
|---|---|---:|---:|---:|
| Fixed4 | non-limited | 14 | 1,747,741 | 0.511 |
| Fixed4 | limited | 10 | 1,791,378 | 0.571 |
| Fixed32 | non-limited | 7 | 213,572 | 0.511 |
| Fixed32 | limited | 29 | 281,708 | 0.378 |

Taxi 正式配置按 V3 报告关闭 sampled-pacing gate trace，所以 target 主频
幅度在 Taxi 中不可计算；只报告 waveform 分类 window 和 score，不填造
harmonic 数值。

Fixed gate 中首次实际 pacing base source 发布时间：

- Fixed4 V3：Guard 2.692 s，Trusted 3.341 s；
- Fixed4 V4：Guard 2.681 s，Trusted 3.226 s；
- Fixed32 V3：Guard 1.631 s，无 Trusted；
- Fixed32 V4：Guard 1.192 s，无 Trusted。

Taxi 没有 gate trace，因此没有可审计的精确 publish timestamp；flow
summary 仍给出 source time ratio。

### 18.3 完整性判定

观察器在代码和 trace 中没有被 cap bypass，且 V4 在 service-limited 与
non-limited 窗口都执行了分析。但是 V4 的窗口数、有效窗口数、Regime
分布和 Reference 有效率与 V3 差异很大，尤其：

- Cell 4 valid windows 471→6；
- Cell 8 valid windows 799→10；
- Cell 4 Reference invalid 52.06%→99.46%；
- Cell 8 Reference invalid 53.32%→99.33%。

因此“目标 waveform 构造未破坏”成立；“频域观察器的闭环动态完整性与 V3
等价”不成立。差异来自 V4 cwnd cap 改变后的 ACK/DRate/SRTT 输入和状态
演进，而不是代码直接跳过观察器，但它仍是正式结果中的失败证据。

## 19. 验收结果

| 场景 | Throughput | Avg Q | P95 Q | Jain | Loss | 总结果 |
|---|---|---|---|---|---|---|
| Fixed 4 | PASS | PASS | PASS | PASS | PASS | PASS |
| Fixed 32 | PASS | FAIL | FAIL | FAIL | PASS | FAIL |
| Cell 4 | PASS | FAIL | FAIL | PASS | PASS | FAIL |
| Cell 8 | PASS | FAIL | FAIL | PASS | PASS | FAIL |

具体偏差：

- Fixed 32 Avg/P95 queue 比 V3 高 45.0%/52.0%，Jain 低 0.1766。
- Cell 4 Avg queue 比 V1 高 78.8%，P95 高 6.84%。
- Cell 8 Avg queue 比 V1 高 72.8%，P95 高 38.4%。

没有 loss 回归，所有场景 throughput 均满足“相对 V3 不下降超过阈值”。

## 20. 十二类失败归因

| 编号 | 候选原因 | 证据判定 |
|---:|---|---|
| 1 | delivered history 不能及时反映下降 | 不支持。下降窗口 Cell4 service/plan=14.1/43.2 KB，Cell8=2.7/26.7 KB；history 明显收缩。 |
| 2 | ACK compression 使 service 偏高 | 不是主要证据。service 大幅低于 plan；真正大的 ACK 相关项是 ExtraAcked headroom。 |
| 3 | ACK silence 使 service 偏低 | 无充分证据。service valid 99.8%–99.9%，总吞吐没有损失或低速自锁。 |
| 4 | ExtraAcked 过大，抵消 restriction | **主要支持。** Taxi ExtraAcked/cap mean 67%–86%、P95 约 96%–99%。 |
| 5 | ExtraAcked 不足，导致吞吐损失 | 不支持。Taxi throughput 比 V3 高 3.95%/4.75%。 |
| 6 | positive probe credit 不足 | 不支持。人工回升后单流恢复 97.1 Mbps，4-flow 约 100 Mbps。 |
| 7 | positive probe credit 过大 | 非主要原因。Taxi mean probe 7.8–16.0 KB，小于 mean ExtraAcked 21.1–28.3 KB 和 restriction。 |
| 8 | app-limited 误判 | 不支持。四个正式场景 app-limited fallback ratio=0。 |
| 9 | per-flow service envelope 公平份额自锁 | **Fixed32 支持。** Jain=0.790；throughput 与 service/probe/ExtraAcked correlation=.998/.993/.953。 |
| 10 | target/base 构造或积分错误 | 不支持。30 个 V4 self-test 全过，62500 B、partial overlap、V3/V4 plan equality 均精确。 |
| 11 | final cwnd 被后续路径覆盖 | 不支持。最终 getter 只返回 `min(native,cap)`；inflight-above-cap self-test 通过，Loss=0。 |
| 12 | Reference 更新长期错误 | **次要支持。** Taxi Reference invalid 从 V3 的约 52%/53% 升到 V4 的约 99.4%/99.3%，valid windows 同时骤减。Estimator 代码未改，但闭环输入已改变。 |

最符合数据的组合是：

1. Taxi：类型 4 为主，类型 12 为次；
2. Fixed 32：类型 9 为主；
3. 不是 delivered history、app-limited、probe recovery 或积分实现错误。

## 21. 结果文件

主要离线产物位于：

```text
NS3.27/results/fbbr_service_consistent_inflight_envelope_v4/
```

包括：

```text
analysis_summary.json
formal_metrics.csv
acceptance.csv
v4_envelope_summary.csv
fixed4_per_flow.csv
fixed32_per_flow.csv
fixed32_share_correlations.csv
frequency_integrity.csv
frequency_service_limited_split.csv
capacity_transition_aligned.csv
capacity_transition_summary.csv
capacity_transition_queue_summary.csv
synthetic_dynamic_summary.csv
```

四个正式场景、sanity、smoke 和人工动态容量的原始 trace、config、command、
run log、return code 和 compare 图均保留在各自子目录。

## 22. 最终回答

Service-consistent inflight envelope 的核心机制按要求实现且运行：

- 不修改长期 ReferenceBw estimator；
- Reference invalid 时仍构造并激活 plan；
- delivered service 能在下降后收缩；
- probe credit 能在上升后恢复吞吐；
- final cwnd 只有 `min(native, cap)` 一个 actuator。

但正式结果表明，这些条件还不足以同时实现“保持 V3 全部静态性能”和
“Taxi queue 达到 V1 目标”。Native `ExtraAcked` 在 Taxi 中接近占满 cap，
而 32-flow 的 per-flow service/probe/headroom 又强化既得 ACK-clock 份额。

所以本轮答案是：

```text
机制实现：是
Fixed 4 保持静态性能：是
动态下降时 service envelope 收缩：是
已有 probing 恢复吞吐：是
无需直接修改长期 ReferenceBw：是
四场景整体验收：否
```

按任务约束，本轮到此停止，不添加 gain、阈值、平滑、恢复规则、场景特判
或 V5。
