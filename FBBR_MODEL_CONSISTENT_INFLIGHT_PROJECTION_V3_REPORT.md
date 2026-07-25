# FBBR Model-Consistent Inflight Projection V3 报告

日期：2026-07-25  
代码工作区：`/home/wkd/FreqBBR`  
实验根目录：`NS3.27/results/fbbr_model_consistent_inflight_projection_v3`

## 1. 结论摘要

本轮已完成独立算法入口 `FBBR-hybridv3`、编译、24 项 V3
self-test、V1 隔离回归、1-flow sanity、4/32-flow smoke、四组
V1/V3 正式实验及频域/投影分析。没有创建 Git 分支，也没有实现 V4、
恢复状态机、Drain、delay gain 或额外 baseline actuator。

V3 的唯一控制式是：

\[
Cwnd_{final}=\min(Cwnd_{native}, I_{cap})
\]

\[
I_{cap}=\max(MinPipeCwnd,I_{model}+ExtraAcked)
\]

当前 DQC BBRv2 实现没有独立的 TSO/offload 字节预算，因此
`OffloadBudget=0`，没有用固定比例替代。

实验结论不是“全部通过”：

- 固定 32-flow：吞吐、队列、公平性和 Loss 全部通过，队列相对当前
  V1 显著下降。
- 固定 4-flow：吞吐、公平性、P95 和 Loss 通过，但 Avg Q=13.269 ms，
  高于 7.246 ms 目标。
- Cellular 4-flow：吞吐、Avg/P95 Q 和 Loss 通过，但 Jain=0.784271，
  低于 0.827437 目标。
- Cellular 8-flow：吞吐和 Loss 通过；Avg/P95 Q 与 Jain 未通过。

所以对最终问题的回答是：

> Model-consistent inflight projection 能在固定链路，尤其 32-flow
> 场景中，在不持续修改 ReferenceBw 的情况下保持吞吐、公平性并明显
> 降低队列；但本次实现不能在四个场景中同时做到这一点。动态 Taxi
> 容量下 ReferenceBw 有效/更新速度不足，导致投影约一半时间未激活，
> 且有效 Reference 在容量骤降时可能高于即时公平 BDP，因此 cellular
> 队列和公平性未达到目标。

## 2. 算法入口与 V1/V3 隔离

新增枚举 `kFBBRHybridV3`，位于原有枚举末尾，避免改变已有枚举值。
工厂中新增独立 `case kFBBRHybridV3`，构造同一个 `FBBRSender` 类但传入
独立拥塞控制类型。

隔离方式如下：

- `IsFbbrHybrid()` 只对 `kFBBRHybrid` 返回 true，继续拥有 V1
  Gradient-Matched actuator、lower-bound search 和原有兼容行为。
- `IsFbbrHybridV3()` 只对 `kFBBRHybridV3` 返回 true。
- `IsFbbrHybridObserver()` 只用于共享频域观察、TrustedBw/GuardBw、
  RTprop 和 MaxBw 信息层。
- V3 分类执行器走独立的 `ApplyFbbrHybridV3Classification()`；它不会
  调用 V1 的 gradient decrease、quarter-gap、midpoint、mindrate、
  lower-bound drain 或 baseline recovery。
- 没有用全局 bool 把 `FBBR-hybrid` 偷换为 V3。

命令行和 Python 批量框架均可用同一套脚本选择：

```text
--only-cc FBBR-hybrid,FBBR-hybridv3
```

trace、命令和结果目录均显示完整算法名 `FBBR-hybridv3`。

## 3. 修改文件与主要函数

| 文件 | 修改内容 |
|---|---|
| `src/dqc/model/thirdparty/include/proto_types.h` | 新增 `kFBBRHybridV3` |
| `src/dqc/model/thirdparty/congestion/proto_send_algorithm_interface.cc` | 独立工厂 case |
| `src/dqc/model/thirdparty/congestion/fbbr_sender.h/.cc` | Reference 选择、target 历史、精确积分、cap、最终 cwnd、分类、日志、自测 |
| `src/dqc/model/dqc_sender.h/.cc` | V3 算法识别及 flow-end 汇总的安全落盘 |
| `src/dqc/model/dqc_trace.h/.cc` | 窗口 V3 字段和 flow summary CSV |
| `scratch/generic_p2p_switch_flows.cc` | 算法解析、结果名、销毁前汇总 |
| `scratch/fbbr_4flow.cc` | V3 解析及 `fbbrHybridV3SelfTest` |
| `examples/ConcurrentFlow/run_4cc_comparison.py` | V3 批量实验入口 |
| `examples/ConcurrentFlow/plot_4cc_comparison.py` | V1/V3 绘图标识 |
| `scripts/analyze_fbbr_model_projection_v3.py` | 正式指标、频域、binding 和验收分析 |

核心函数：

```cpp
bool IsFbbrHybridV3() const;
FbbrV3ReferenceResult SelectFbbrV3ReferenceBw() const;
void RecordFbbrV3PacingTarget(...);
bool HasFullFbbrV3RateHistory(...) const;
QuicByteCount ComputeFbbrV3ModelInflightBytes(...) const;
QuicByteCount ComputeFbbrV3InflightCapBytes(...) const;
QuicByteCount ApplyFbbrV3InflightProjection(...) const;
FbbrV3ProjectionSnapshot BuildFbbrV3ProjectionSnapshot(...) const;
QuicByteCount GetCongestionWindow() const override;
void ApplyFbbrHybridV3Classification(...);
```

## 4. ReferenceBw 选择

V3 按现有发布/有效语义选择：

1. 当前有效且来源为现有频域/窗口结果的 `TrustedBw`；
2. 当前已由原 Cruise 完成路径发布的 `GUARD_FILTER`；
3. 现有 `PREVIOUS_TRUSTED` 有效状态；
4. 否则 invalid。

原两级 ACK 低通滤波器的第一个内部样本不直接成为 Reference。它仍先走
原有 Cruise 完成发布路径，避免在早期低速 bootstrap 样本上立即闭合
`pacing -> delivery -> guard -> pacing`。这没有增加阈值或 TTL，只复用
现有发布有效性。

`NATIVE_BW_FALLBACK`/永久 MaxBw 不被视为有效 Reference。Reference
invalid 时：

- pacing 继续走 native/V1 bootstrap；
- target 历史仍继续记录；
- inflight projection 不激活；
- 不用 invalid Reference 构造 cap。

Reference valid 后，它成为 V3 pacing baseline。`current_injection_baseline`
仅作兼容镜像；Cruise transition 不再用 MaxBw 覆盖有效 Reference。

## 5. Regime 职责

- Regime II/FULL_LOAD：保留现有 MeanDRate 候选、TrustedBw 置信度与发布
  路径，不直接改 pacing baseline。
- Regime I/UNDERLOAD：保留 probing、波形、Guard/Trusted 观察、MaxBw、
  delivery-rate 模型和 phase progression；V3 action 只记录
  `HOLD_REFERENCE_UNDERLOAD`。
- Regime III/OVERLOAD：保留污染样本拒绝/冻结、分类和 RTprop 观察；
  V3 action 只记录 `HOLD_REFERENCE_OVERLOAD`。
- INCONCLUSIVE：保持 Reference。

V3 代码注释明确禁止 queue target/gain、gradient gain、baseline step、
drain/recovery、flow-count/scenario 参数。V3 没有新增配置键。

## 6. 最终 pacing target 与历史

`FBBRSender::PacingRate()` 仍先构造原 BBR phase gain 和原频率三角波：

\[
PacingTarget(t)=F(ReferenceBw, BBRPhaseGain,
                  FBBRFrequencyWaveform, NativeLimits)
\]

V3 仅在 Reference valid 时替换 baseline；频率、相位、波形幅度和
ProbeBW phase gain 均未改变。cap binding 不会降低这个 target。
`RecordFbbrV3PacingTarget()` 记录的是函数最终返回的期望 pacing target，
不是受 cwnd 限制后的实际发送速率。

历史是连接级分段常数 deque。target 变化时关闭上一段并开始新段；相同
时间戳合并，不制造零时长积分。维护：

```text
fbbr_v3_max_rtprop_seen_
fbbr_v3_history_start_time_
fbbr_v3_history_valid_
```

清理只删除早于已见最大 RTprop、今后不可能参与积分的闭合 segment。
RTprop 增长且现有历史不足时，history invalid，等真实覆盖新 RTprop
后自动恢复；不以 `current_rate * RTprop` 补齐。

## 7. 精确 ModelInflight 积分

对每个 segment 与窗口 \([t-RTprop,t]\) 做精确 overlap：

\[
I_{model}(t)=
\sum_j \frac{R_j}{8}
\max(0,\min(t,t_j^{end})-\max(t-RTprop,t_j^{start}))
\]

实现单位：

- `QuicBandwidth`：bit/s；
- segment 时间和 RTprop：microseconds；
- 积分除数：\(8\times10^6\)；
- 输出：bytes；
- 中间使用 `long double` 并在转换前做上界钳制。

因此周期正半波、负半波、phase gain 切换和 Reference 变化都按真实 target
历史进入 BDP，而不是当前速率近似。

## 8. Native headroom 与 cap

实际复用字段：

- ACK aggregation：`model_.MaxAckHeight()`；
- MinPipeCwnd：`cwnd_limits().Min()`；
- 对齐：`kDefaultTCPMSS` 向上取整。

当前实现没有可复用的 TSO/offload/send-quantum 字节预算，所以该项为 0；
没有创造固定百分比替代。

\[
I_{cap}=\max(cwnd\_limits().Min(),
             I_{model}+model_.MaxAckHeight())
\]

所有 byte 加法和 MSS 对齐都做饱和检查。

## 9. 最终 cwnd/send-allowance 路径

原 BBRv2 先照常更新 `cwnd_`、MaxBw、inflight_hi/lo、ProbeRTT 等模型状态。
FBBR 最终覆写的虚函数：

```cpp
QuicByteCount FBBRSender::GetCongestionWindow() const {
  return ApplyFbbrV3InflightProjection(cwnd_);
}
```

`Bbr2Sender::CanSend()` 通过虚函数读取该值，之后没有代码再次抬高 V3
结果。实现没有：

- 修改 MaxBw、TrustedBw、GuardBw；
- 回写 inflight_hi/inflight_lo；
- 把 cap 变成持久模型上界；
- 使用 `max(cwnd_final, bytes_in_flight)`；
- 丢弃在途包。

当前 inflight 大于 cap 时只停止新发送，ACK 自然排空；cap 解除后立即使用
原 Reference pacing，无恢复状态机。严格统计语义中，只有
`actual_inflight > cap` 才记为 binding；等于 cap 不计 excess binding。

projection 激活条件：

```text
algorithm == FBBR-hybridv3
ReferenceBw valid
RTprop valid
history covers full RTprop
drain_completed
mode == PROBE_BW
```

Startup、初始 Drain 和 ProbeRTT 不受 V3 cap 覆写。历史在这些阶段仍维护。

## 10. Queue debt

\[
RawQueueDebt=\max(0,BytesInFlight-I_{model})
\]

\[
EnforcedExcess=\max(0,BytesInFlight-I_{cap})
\]

二者只进入 telemetry，不进入 pacing、Reference、cap 或 cwnd 公式。

## 11. 日志

每个现有有效分类窗口末尾追加：

```text
v3_reference_bw
v3_reference_source
v3_model_inflight
v3_inflight_cap
v3_native_cwnd
v3_actual_inflight
v3_raw_queue_debt
v3_enforced_excess
v3_cap_binding_fraction
v3_history_valid
```

每个 flow 结束写一次 `flowN_v3_projection_summary.csv`。汇总在
`Simulator::Destroy()` 之前显式且幂等落盘，避免 sender 析构时 trace
对象已销毁。没有新增逐 ACK、逐包独立日志；ACK 事件只更新内存计数。

## 12. 编译、自测和机制 sanity

| 检查 | 返回码/结果 |
|---|---:|
| `./waf build` | 0 |
| V3 self-test | 0，24 PASS |
| V1 self-test | 0，70 PASS |
| V1 前后归一化输出 SHA256 | 均为 `7202b980...2bf71` |
| 1-flow / 100M / 5BDP / 120s | 0 |
| 4-flow / 100M / 5BDP / 60s smoke | 0 |
| 32-flow / 100M / 5BDP / 60s smoke | 0 |

V1 输出比较只去掉 Waf 自身构建耗时行；其余输出逐字节一致。

V3 self-test 覆盖：

- V1/V3 算法隔离和 V1 key action；
- 100 Mbps × 20 ms = 250000 bytes；
- 50/100 Mbps 两段精确积分 = 187500 bytes；
- 中间变化、partial overlap、重复时间戳；
- RTprop 增长暂停和真实历史恢复；
- native headroom、MinPipe、MSS 对齐、饱和运算；
- `final <= native`、inflight 高于 cap 不抬 cap；
- 等于 cap 不计 binding，超过 1 byte 才计 binding；
- Startup/ProbeRTT 不覆写；
- cap 解除无状态机；
- Trusted/Guard/PREVIOUS_TRUSTED/native fallback 选择。

sanity/smoke 指标：

| 场景 | Throughput Mbps | Util | Avg/P95 Q ms | Jain | Loss % |
|---|---:|---:|---:|---:|---:|
| 1-flow 120s | 97.916 | 97.916% | 9.932 / 10.022 | 1.000000 | 0 |
| 4-flow smoke | 97.914 | 97.914% | 13.290 / 13.363 | 0.995094 | 0 |
| 32-flow smoke | 97.895 | 97.895% | 17.508 / 17.647 | 0.966661 | 0 |

单流 Reference 发布后 projection 正常激活；无死锁、长期停发或 minimum
cwnd 卡死。target waveform 不因 cap binding 被改写。

## 13. 正式实验方法

所有新 V1/V3 实验：

- seed=1，runId=1；
- 同一 `fbbr_default.conf`；
- 同一拓扑、流开始/结束、buffer=5BDP、业务批次、timer；
- fixed：100 Mbps，4/32 flows，240 s，1/19 ms 单向分段；
- cellular：同一 Taxi 1650 点容量序列，标称 128 Mbps，4/8 flows，
  180 s，0/10 ms 单向分段；
- throughput/capacity/utilization/queue/Jain 跳过前 5 s；
- Loss/OWD/Total GiB 使用 whole run。

BBR-R、BBRv2 复用 2026-07-22 相同配置且 `return_code.txt=0` 的结果。
四场景 V1/V3 和复用结果共 16 个 return code 均为 0。

## 14. 四场景完整指标

| 场景 | 算法 | Throughput Mbps | Capacity Mbps | Util | Avg Q ms | P95 Q ms | Jain | Loss % | OWD ms | Total GiB |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| Fixed 4 | V1 | 97.641 | 100.000 | 97.641% | 12.946 | 24.422 | 0.925240 | 0 | 34.112 | 2.726 |
| Fixed 4 | V3 | 97.914 | 100.000 | 97.914% | 13.269 | 13.363 | 0.995225 | 0 | 34.430 | 2.734 |
| Fixed 4 | BBR-R | 97.719 | 100.000 | 97.719% | 5.063 | 14.630 | 0.986803 | 0 | 26.461 | 2.728 |
| Fixed 4 | BBRv2 | 98.052 | 100.000 | 98.052% | 181.835 | 209.770 | 0.943277 | 1.727570 | 200.016 | 2.734 |
| Fixed 32 | V1 | 98.014 | 100.000 | 98.014% | 82.985 | 178.344 | 0.853085 | 0.003501 | 102.887 | 2.735 |
| Fixed 32 | V3 | 97.898 | 100.000 | 97.898% | 17.523 | 17.652 | 0.966755 | 0 | 38.908 | 2.735 |
| Fixed 32 | BBR-R | 97.900 | 100.000 | 97.900% | 15.503 | 31.314 | 0.760189 | 0 | 36.832 | 2.735 |
| Fixed 32 | BBRv2 | 98.503 | 100.000 | 98.503% | 199.574 | 209.898 | 0.850406 | 3.796570 | 217.967 | 2.735 |
| Cell 4 | V1 | 42.303 | 43.849 | 96.476% | 73.393 | 416.988 | 0.848943 | 0 | 26.624 | 0.875 |
| Cell 4 | V3 | 42.561 | 43.849 | 97.064% | 121.320 | 528.901 | 0.784271 | 0 | 33.424 | 0.880 |
| Cell 4 | BBR-R | 41.805 | 43.849 | 95.340% | 64.575 | 231.549 | 0.925854 | 0 | 30.136 | 0.866 |
| Cell 4 | BBRv2 | 43.121 | 43.849 | 98.340% | 741.386 | 4792.732 | 0.842796 | 0.793566 | 174.166 | 0.891 |
| Cell 8 | V1 | 42.607 | 43.849 | 97.168% | 154.985 | 1007.408 | 0.818307 | 0 | 33.810 | 0.880 |
| Cell 8 | V3 | 43.124 | 43.849 | 98.346% | 188.427 | 1224.354 | 0.810170 | 0 | 45.756 | 0.887 |
| Cell 8 | BBR-R | 41.955 | 43.849 | 95.681% | 85.242 | 364.943 | 0.844523 | 0 | 31.858 | 0.869 |
| Cell 8 | BBRv2 | 43.190 | 43.849 | 98.498% | 731.845 | 4192.689 | 0.684773 | 0.524525 | 182.410 | 0.891 |

## 15. Reference 来源、projection 和 debt

以下为 flow summary 的跨流算术平均；P95 debt 列是“各流 P95 的平均”。
所有字节量均为每流 bytes。

| 场景 | Trusted | Guard | Last | Invalid | Active | History invalid | Binding | Mean model | Mean raw debt | Mean flow P95 raw | Mean excess | Mean flow P95 excess |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| Fixed 4 | 24.657% | 73.367% | 0 | 1.975% | 98.025% | 1.975% | 0.0157% | 154230 | 10639 | 10927 | 1.03 | 0 |
| Fixed 32 | 0 | 98.938% | 0 | 1.062% | 98.938% | 1.062% | 0.0252% | 19309 | 1871 | 1956 | 0.34 | 0 |
| Cell 4 | 0.848% | 47.096% | 0 | 52.056% | 47.944% | 52.056% | 1.420% | 18032 | 2547 | 6355 | 342.24 | 0 |
| Cell 8 | 0.012% | 46.665% | 0 | 53.323% | 46.677% | 53.323% | 1.910% | 9186 | 2948 | 5968 | 312.54 | 0 |

固定场景 Reference 几乎全程有效，projection 约 98–99% 时间激活；
cellular 中 Reference invalid 约 52–53%，这是动态场景的核心差异。

## 16. Fixed 4 逐流

Reference 均值来自 sampled-pacing trace 中实际选用的 V3 Reference。

| Flow | Throughput Mbps | Mean Reference Mbps | P50 Reference Mbps | Trusted ratio | Guard ratio |
|---:|---:|---:|---:|---:|---:|
| 1 | 26.263 | 25.705 | 25.705 | 0 | 97.617% |
| 2 | 25.645 | 24.629 | 24.629 | 0 | 98.783% |
| 3 | 24.022 | 23.773 | 23.773 | 98.629% | 0 |
| 4 | 21.984 | 21.461 | 21.459 | 0 | 97.070% |

聚合 Jain=0.995225。V3 明显改善了当前 V1 的公平性
（0.925240 -> 0.995225）和 P95 Q（24.422 -> 13.363 ms），但 Avg Q
略高于 V1 且没有达到 7.246 ms 目标。

## 17. Fixed 32 聚合队列和 cap

V3 相对当前 V1：

- Throughput：98.014 -> 97.898 Mbps，下降 0.116 Mbps；
- Avg Q：82.985 -> 17.523 ms，下降 78.9%；
- P95 Q：178.344 -> 17.652 ms，下降 90.1%；
- Jain：0.853085 -> 0.966755；
- Loss：0.003501% -> 0。

projection active=98.938%，但严格 cap-binding 时间仅 0.0252%。这说明
最终 cap 很少处于“actual inflight 已超过 cap”的排空状态；大多数时间
native cwnd/ACK 自然演化已不高于 cap。V3 没有把 cap 回写为持久
inflight_hi/lo。

## 18. Cellular Reference/cap 响应

Taxi 容量以约 100 ms 粒度剧烈变化，而 V3 Reference 仍只由原有
Trusted/Guard 发布语义更新。

- Cell 4 projection active 47.944%，binding 1.420%；
- Cell 8 projection active 46.677%，binding 1.910%；
- 正 Reference 的分类窗口样本很少（Cell 4 为 2 个，Cell 8 为 4 个）；
- 在这些有限窗口上，`Reference / instant fair capacity`：
  - Cell 4 P50=6.67；
  - Cell 8 P50=2.22、P95=9.82。

该比值会被 Taxi 的瞬时低容量谷值放大，样本数不足以作为总体均值，但它
清楚证明容量骤降时存在“旧 Reference 高于即时公平份额”的窗口。V3
不会用 RTT 或 Regime III 直接降低 Reference，且 Reference invalid 时
不启用 projection，因此动态容量下出现更高队列和公平性下降。

## 19. 频域特征完整性

正式配置使用现有 `cruise_detector.mode=time_waveform`。因此 legacy
Goertzel spectral-integrity 数值列在 V1/V3 都保持 0；V3 没有改变或
绕过配置选择，Goertzel 代码路径仍被保留。下表的 DRate/SRTT
“frequency score”按现有 time-waveform
`1 - sender/response period_error_ratio` 统计，不是新增控制分数。

| 场景/算法 | 分析窗 | 有效频域窗 | I/II/III | Inconclusive | Trusted 首次 min/median s | DRate score P50/P95 | SRTT score P50/P95 |
|---|---:|---:|---:|---:|---:|---:|---:|
| Fixed 4 V1 | 1966 | 1964 | 711/24/1229 | 2 | 2.684/3.100 | 0.950/1.000 | 0.925/1.000 |
| Fixed 4 V3 | 36 | 36 | 11/1/24 | 0 | 2.692/3.099 | 0.950/0.994 | 0.950/0.992 |
| Fixed 32 V1 | 12323 | 12106 | 7924/8/4177 | 214 | 1.619/1.736 | 0.950/1.000 | 0.872/0.975 |
| Fixed 32 V3 | 101 | 96 | 30/0/66 | 5 | 1.631/1.744 | 0.888/0.986 | 0.775/0.975 |

V3 的观察窗数量显著少于 V1，是因为 Reference 发布后稳定闭合，V3 不再
用 baseline actuator 持续制造新不稳定 episode；不是绕过
`AnalyzeWaveformWindow()`。所有 V3 窗口仍输出 Regime、DRate/SRTT
结构和 Trusted 更新，首次更新时间与 V1 基本一致。

### 19.1 target/actual 主频幅度

“actual”使用现有 `current_delivery_rate` 作为 cwnd-limited 实际输出
代理，不新增高频发送日志。

| 场景/算法 | Target P50 Mbps | Actual P50 Mbps | Actual/Target P50 | Target fundamental / commanded amplitude |
|---|---:|---:|---:|---:|
| Fixed 4 V1 | 5.285 | 1.983 | 0.381 | 0.81115 |
| Fixed 4 V3 | 1.850 | 0.688 | 0.329 | 0.81114 |
| Fixed 32 V1 | 0.641 | 0.155 | 0.247 | 0.80491 |
| Fixed 32 V3 | 0.311 | 0.110 | 0.336 | 0.80807 |

三角波理论基频/峰值幅度为 \(8/\pi^2=0.81057\)。V1/V3 的 target
归一化结果几乎相同，证明 V3 没有修改 target 波形构造，也没有在 cap
binding 时降低 target。

Fixed 4 非 binding 的 actual/target 由 0.381 降到 0.309，约 18.9%；
Fixed 32 则由 0.247 升到 0.336。因此不能声称所有场景的实际响应幅度都
与 V1 相同，但 target waveform 本身完整。

### 19.2 binding 与非 binding 窗口

| 场景 | 窗口组 | 窗口数 | Target P50 Mbps | Actual P50 Mbps | Actual/Target |
|---|---|---:|---:|---:|---:|
| Fixed 4 | non-binding | 33 | 1.819 | 0.629 | 0.309 |
| Fixed 4 | binding | 3 | 4.540 | 1.977 | 0.435 |
| Fixed 32 | non-binding | 75 | 0.291 | 0.101 | 0.336 |
| Fixed 32 | binding | 26 | 0.744 | 0.211 | 0.345 |

binding 窗口的 target 未被截断，actual/target 也没有比非 binding
窗口更低。严格 binding 只在 actual inflight 超过 cap 时累计；同时
flow summary 的 P95 enforced excess 为 0，说明超过 cap 的事件稀少且
通过 ACK 自然排空。没有证据表明 cap 过度破坏了设计波形。

## 20. 验收

| 场景 | Throughput | Avg Q | P95 Q | Jain | Loss | 场景结论 |
|---|---|---|---|---|---|---|
| Fixed 4 | PASS | **FAIL** | PASS | PASS | PASS | FAIL |
| Fixed 32 | PASS | PASS | PASS | PASS | PASS | PASS |
| Cell 4 | PASS | PASS | PASS | **FAIL** | PASS | FAIL |
| Cell 8 | PASS | **FAIL** | **FAIL** | **FAIL** | PASS | FAIL |

全部验收：**FAIL**。V3 四场景 Loss 均为 0。

## 21. 九类失败归因

| 类别 | 判断 | 证据 |
|---|---|---|
| 1. Reference 长期偏低 | 不是主要失败原因 | 四场景 throughput 均通过；fixed 利用率约 97.9%，cell 为 97.1/98.3% |
| 2. Reference 偏高、model 高于即时公平 BDP | cellular 有证据 | 容量低谷窗口 Reference/instant fair capacity P50=6.67（Cell 4）、2.22（Cell 8）；样本少但方向明确 |
| 3. ACK/offload headroom 过大 | Fixed 4 最一致的次要归因 | Fixed 4 raw debt 均值 10.6 KB/flow，但 binding 仅 0.0157%、enforced excess P95=0；队列大多位于 model+native ACK headroom 内。没有 offload budget。未单独 trace `MaxAckHeight`，所以不能进一步定量拆分 |
| 4. headroom 不足、cap 过紧 | 不支持 | throughput 全通过，fixed binding 极低，无 Loss；binding 窗口 actual/target 未下降 |
| 5. pacing history 积分错误 | 排除 | 常速、两段、partial overlap、重复时间戳、RTprop 增长/恢复及溢出测试全部 PASS |
| 6. cap 被后续 cwnd/send path 覆盖 | 排除 | 最终虚函数 `GetCongestionWindow()` 返回 min；无后续抬高；Fixed 32 队列大降是运行证据 |
| 7. cap 正常但 waveform 被过度截断 | 不支持 | target 归一化基频约 0.811，与 V1/理论相同；binding actual/target 不低于 non-binding |
| 8. Trusted/Guard 更新未运行 | 部分、不属于完全失效 | Fixed 首次更新时间与 V1 一致且 Guard 占 73–99%；cell Trusted 几乎没有发布，但 Guard 仍占约 47% |
| 9. 动态容量快于 Reference 更新 | cellular 的主归因 | cell Reference invalid/history invalid 52–53%，projection 只激活约 47%；有效旧 Reference 又会跨越容量急降 |

本轮不根据上述失败再增加机制或参数。

## 22. 产物

- `NS3.27/results/fbbr_model_consistent_inflight_projection_v3/formal_metrics.csv`
- `NS3.27/results/fbbr_model_consistent_inflight_projection_v3/v3_projection_summary.csv`
- `NS3.27/results/fbbr_model_consistent_inflight_projection_v3/frequency_integrity.csv`
- `NS3.27/results/fbbr_model_consistent_inflight_projection_v3/frequency_binding_split.csv`
- `NS3.27/results/fbbr_model_consistent_inflight_projection_v3/fixed4_per_flow.csv`
- `NS3.27/results/fbbr_model_consistent_inflight_projection_v3/cellular_response.csv`
- `NS3.27/results/fbbr_model_consistent_inflight_projection_v3/acceptance.csv`
- `NS3.27/results/fbbr_model_consistent_inflight_projection_v3/analysis_summary.json`
- 各场景目录内 `command.txt`、`config.json`、`return_code.txt`、trace 和
  `compare/summary_metrics.csv`

本报告到此停止；未实现其他方案。
