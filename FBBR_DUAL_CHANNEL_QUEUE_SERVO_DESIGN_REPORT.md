# F-BBR Dual-Channel Trigger and RTT Queue-Servo Implementation Report

生成时间：2026-07-13  
目标仓库：`/home/wkd/BBR_ICC`  
执行规范：`FBBR_DUAL_CHANNEL_QUEUE_SERVO_CODEX_PROMPT.md`

## 1. 最终结论

本轮已完成独立 F-BBR 路径的双通道事件触发、RTT 级 queue reserve servo、
slow/fast loop 职责隔离、trace、随机化 runner、分析器、确定性测试和真实矩阵。

已经确认通过：

- `PASS`：完整构建。
- `PASS`：F-BBR 确定性自测，包含 delivery-only、queue-only、BOTH、hard safety、
  trigger selection bias、重叠独立性、连续排队排空、reserve recovery、baseline
  commit、hard loss 和 persistent search。
- `PASS`：FreqCCv4 gate、trusted selection、trusted pacing 三项回归。
- `PASS`：F-BBR/FreqCCv4 源码隔离。
- `PASS`：F-BBR/FreqCCv4 混合运行 trace 隔离。
- `PASS`：trigger/servo/event CSV 分别为 54/27/188 列，数据行无列宽错误。
- `PASS`：随机矩阵 28 个 run 得到 28 个唯一关键 trace hash。
- `PASS`：hard safety 后 listener/search 自动继续，persistent search 未被关闭。
- `PASS`：servo correction 不再被 slow loop 重复吸收；临时 `g_q` 不进入 trusted。
- `PASS`：最终 slow baseline 不再追逐 `g_q<1` 的临时 underload。

仍未达到规范验收：

- `FAIL_QUEUE_TRIGGER`：最终真实单流 10-seed 中 queue-only/BOTH run coverage 为 0%。
  生产状态下 servo 很快把系统送到 delivery 可辨识区，queue response 仍低于严格
  `3 x 1.4826 MAD` 门限；没有降低门限伪造通过。
- `FAIL_QUEUE_SERVO`：默认 2%-5% BDP reserve 在四流和动态场景中 q95 仍超
  `0.10 BDP`，或以明显利用率损失换取浅队列。
- `FAIL_QUEUE_RESERVE_TARGET`：0/1/2/5/10% BDP sweep 无一同时满足
  utilization `>=99%` 与 q95 `<=0.10 BDP`。
- `FAIL_EVENT_WINDOW_NOT_MEASURABLE`：单流总体可测性明显改善，但部分四流 seed
  仍无完成窗口或窗口不确定。
- `FAIL_SCORE_NOT_CORRELATED`：真实 target-region 样本不足，aggregate score/ground
  truth Spearman 为 `NaN`，不能声明 score calibration 成功。
- `FAIL_SEARCH_NOT_CONVERGED`：真实矩阵未达到 TRACK/LOCK 验收。
- `FAIL_TRUSTED_BW_INACCURATE`：所有真实矩阵 trusted publication 都为 0；按规范，
  不能把“未发布”解释为安全成功。
- `FAIL_DYNAMIC_TRACKING`：50 秒动态场景 utilization `0.6853`，tail q95
  `2.8544 BDP`，未在要求时间内恢复。

因此，本轮结果是：基础架构和测量入口已经完成并验证，真实事件窗从长期 0 trigger
提升为高覆盖且多数可测；但默认浅队列控制与 BBR ProbeBW 相位机之间仍有结构性吞吐
代价，最终 TRACK/LOCK/trusted 和动态跟踪尚未通过。

## 2. 输入与基线事实

规范要求预读的以下历史文件在执行时不在工作区：

```text
FBBR_EVENT_TRIGGERED_DYNAMIC_WINDOW_FINAL_REPORT.md
FBBR_EVENT_WINDOW_CURRENT_CODE_MAP.md
CURRENT_FBBR_CODE_MAP.md
```

因此，先基于实际源码、配置和已有实验产物生成了：

```text
FBBR_DUAL_CHANNEL_CURRENT_CODE_MAP.md
```

该文件记录了改造前 CRUISE 调用链、actual emission、ACK/delivery、RTT、loss、ECN、
pulser/watcher、动态窗口、RTprop、queue score、search baseline、trusted 和 trace 映射。

## 3. 算法隔离

F-BBR 保持以下独立路径：

```text
CC type       kFBBR
sender        FBBRSender
config        FBBRConfig / FBBRFrequencySearchConfig
config file   examples/CCconfig/fbbr_default.conf
CLI           --fbbrConfig
scenario      fbbr_4flow / generic runner algo F-BBR
trace prefix  fbbr
```

没有在 `freqccv4_sender.{h,cc}` 中加入 F-BBR 分支，没有让 `FBBRSender` 继承或持有
`FreqCCv4Sender`，也没有让 F-BBR trace 写入 `freq_*` 文件。

最终源码扫描命令：

```bash
rg -n "F-BBR|FBBR|fbbr|kFBBR" \
  src/dqc/model/thirdparty/congestion/freqccv4_sender.h \
  src/dqc/model/thirdparty/congestion/freqccv4_sender.cc \
  scratch/freqccv4_4flow.cc \
  examples/CCconfig/freqccv4_default.conf
```

结果无匹配，`source_isolation.result = PASS`。

## 4. 实现文件

核心算法：

- `NS3.27/src/dqc/model/thirdparty/congestion/fbbr_frequency_search.h`
- `NS3.27/src/dqc/model/thirdparty/congestion/fbbr_frequency_search.cc`
- `NS3.27/src/dqc/model/thirdparty/congestion/fbbr_sender.h`
- `NS3.27/src/dqc/model/thirdparty/congestion/fbbr_sender.cc`

公共注册、trace 和 runner：

- `NS3.27/src/dqc/model/dqc_trace.h`
- `NS3.27/src/dqc/model/dqc_trace.cc`
- `NS3.27/src/dqc/model/dqc_sender.h`
- `NS3.27/src/dqc/model/dqc_sender.cc`
- `NS3.27/scratch/generic_p2p_switch_flows.cc`

配置和验证：

- `NS3.27/examples/CCconfig/fbbr_default.conf`
- `NS3.27/examples/ConcurrentFlow/analyze_fbbr_dual_channel_queue_servo.py`
- `NS3.27/run_fbbr_dual_channel_queue_servo_validation.sh`
- `FBBR_DUAL_CHANNEL_CURRENT_CODE_MAP.md`
- `FBBR_DUAL_CHANNEL_QUEUE_SERVO_DESIGN_REPORT.md`

## 5. 双通道触发

实现入口：

```text
FBBRFrequencySearch::AnalyzeTriggerCycle()
  fbbr_frequency_search.cc:1136
```

### 5.1 Delivery channel

输入为实际发包与 ACK bytes 重建的 delivery rate，要求：

```text
actual input measurable
period error <= configured tolerance
response >= 4 MSS per carrier period
spectral prominence >= 2.0
normalized match >= 0.50
coverage/app-limited/recovery/stationarity eligible
```

### 5.2 Queue-derivative channel

输入为 `latest RTT - frozen RTprop anchor`，先做 robust linear detrend，再使用五点
Savitzky-Golay 平滑，计算目标 carrier 的 queue amplitude，并转换为：

```text
A_qdot = 2*pi*f0*|Q(f0)|
```

噪声门限保持规范值：

```text
max(3 * 1.4826 * MAD(queue residual derivative),
    2 * timestamp quantum derivative)
```

没有为真实结果降低 prominence、match 或 MAD 倍数。

生产探针为非对称零均值波形，存在已知整数谐波。最终 queue 邻频参考使用
`1.75f0/2.5f0`，避免把合法的 `2f0` 输入谐波当噪声，同时保持确定性纯正弦
queue-only prominence `3.02677 >= 2.0`。

### 5.3 周期与来源

短窗自由频率扫描容易在 Hann 主瓣边界得到 20%-25% 偏差，因此增加相邻两个载波
周期的同相重复相关校验。只有相关系数 `>=0.50` 才把 period estimate 锚定到已知实际
发包 carrier；同频反相噪声、错频和容量阶跃负对照仍被拒绝。

组合来源：

```text
DELIVERY_ONLY
QUEUE_ONLY
BOTH
HARD_SAFETY_ONLY
NONE
```

组合置信度：

```text
C_trigger = 1 - (1 - C_delivery) * (1 - C_queue)
```

## 6. 选择偏差与动态窗口

保持以下不变量：

- trigger cycle 不参与 score、direction、candidate 或 trusted。
- capture 从 trigger 后的独立周期开始。
- 4 个独立周期后允许第一次方向判断。
- 6 个有效周期后才允许 lockable score。
- diagnostic stride 为 0.25 cycle。
- control stride 为 0.5/1 cycle，trusted independence 为 4 cycles。
- servo step `>2%` 的 carrier cycle 权重置零。
- overlap window 只做诊断，不更新 slow baseline 或 trusted。

最终真实单流 15 秒 3-seed复测：

| Seed | Trigger pass | Event windows | Independent | Measurable |
|---:|---:|---:|---:|---:|
| 1 | 22 | 16 | 9 | 8 |
| 2 | 24 | 18 | 10 | 10 |
| 3 | 19 | 8 | 5 | 5 |

输入侧 servo factor 已按 delay 对齐插值。修复前错误地用“输入时刻 native pacing”除以
“输出时刻 servo factor”，导致虚假的 `baseline_changed_inside_window`；修复后相关窗口
的 `native_baseline_drift` 降到约 `1e-14`。

## 7. RTT Queue Reserve Servo

实现入口：

```text
FBBRQueueReserveServo::Update()
  fbbr_frequency_search.cc:2605

FBBRSender::FBBRQueueServoUpdate()
  fbbr_sender.cc:1541
```

每个 RTT 统计：

```text
P20 queue delay
median queue delay
P90 queue delay
two-RTT queue trend
delivery median
loss ratio
ECN ratio
```

最终 CRUISE pacing：

```text
R_final = B_s * g_q * (1 + amplitude * waveform)
```

### 7.1 修正的控制语义

普通 queue down correction 现在是相对 `B_s` 的目标，而不是对旧 factor 连续相减：

```text
target_factor = min(1 - delta_down, 0.99 * D_median / B_s)
g_q = min(old_g_q, target_factor)
```

该修复消除了 `1.00 -> 0.95 -> 0.90 -> 0.85` 的错误累计降速。

低储备恢复：

- emergency/saturated recovery 最多 `2%/RTT`；
- 普通 correction 在 factor `<0.98` 时至少 `1%/RTT`；
- HOLD 分支真正保持 factor，不在 queue gate 中间自动抖动；
- 所有上调均不超过配置的 `2%/RTT`。

hard safety：

```text
loss >= 2%
ECN >= hard threshold
q_floor >= 4*q_H
q_peak >= 4*q_peak_cap
```

进入 `EMERGENCY_DRAIN`，factor 为 `0.70`。

### 7.2 Slow/fast loop 边界

已消除两类重复修正：

1. hard/soft congestion 不再让 slow controller 永久乘 `0.70-0.90`；即时保护只由
   queue servo 执行。
2. 当 window mean factor 不在 `1 +/- 2%` 或含 servo transition cycle 时，slow loop
   返回 `QUEUE_SERVO_TRANSIENT_BASELINE_HOLD`，不得把临时 underload 写入 `B_s`。

只有满足：

```text
saturated down >= 4 RTT
queue remains high
sustainable frequency direction <= -0.20
valid delivery median
```

才允许 `QUEUE_SAFETY_BASELINE_COMMIT`，单次不超过 2%，且不低于
`0.99 * delivery median`。

## 8. ACK 随机化修复

初版 runner 每 10 ms 直接修改 receiver access channel delay。时延降低时，后发 ACK
会超过已在途 ACK，造成非拥塞 packet-threshold loss。隔离实验结果：

| 模式 | hard-loss RTT 比例 | tail goodput |
|---|---:|---:|
| 无 ACK runtime jitter | 0% | 约 92.7 Mbps |
| 100 us runtime jitter | 约 39.1% | 约 64.0 Mbps |
| 仅 packet-size variation | 0% | 约 93.1 Mbps |
| 两者同时 | 约 37.7% | 约 65.0 Mbps |

最终实现改为仿真开始前为每条 receiver access path 设置有种子的固定 timing offset，
保留跨 seed/flow ACK phase 随机性，但不在有包在途时改变传播时延。

随机源均可关闭并写入 `run_meta.json`：

```text
flow start jitter
seeded ACK path timing offset
small seeded background traffic
packet-size variation
capacity micro-jitter
seed / run_id
```

## 9. 配置

新增或正式启用的配置包括：

```text
f_bbr.trigger.delivery.prominence_start = 2.0
f_bbr.trigger.delivery.prominence_continue = 1.5
f_bbr.trigger.delivery.match_start = 0.50
f_bbr.trigger.delivery.match_continue = 0.35
f_bbr.trigger.delivery.min_response_mss = 4

f_bbr.trigger.queue.prominence_start = 2.0
f_bbr.trigger.queue.prominence_continue = 1.5
f_bbr.trigger.queue.match_start = 0.50
f_bbr.trigger.queue.match_continue = 0.35
f_bbr.trigger.queue.noise_mad_multiplier = 3.0
f_bbr.trigger.queue.min_timestamp_quanta = 2

f_bbr.queue_reserve.low_bdp = 0.02
f_bbr.queue_reserve.high_bdp = 0.05
f_bbr.queue_reserve.peak_cap_bdp = 0.10

f_bbr.queue_servo.enabled = true
f_bbr.queue_servo.update_rtts = 1.0
f_bbr.queue_servo.high_gain = 0.50
f_bbr.queue_servo.trend_gain = 0.10
f_bbr.queue_servo.low_gain = 0.25
f_bbr.queue_servo.down_step_max = 0.05
f_bbr.queue_servo.up_step_max = 0.02
f_bbr.queue_servo.recovery_step_max = 0.02
f_bbr.queue_servo.delivery_drain_factor = 0.99
f_bbr.queue_servo.hard_queue_multiple = 4.0
f_bbr.queue_servo.emergency_factor = 0.70
f_bbr.queue_servo.commit_min_rtts = 4
f_bbr.queue_servo.commit_step_max = 0.02

f_bbr.frequency_search.trusted_independent_stride_cycles = 4
f_bbr.direction.channel_split_weight = 0.60
f_bbr.direction.utility_gradient_weight = 0.40
```

## 10. Trace

新增：

```text
flowN_fbbr_queue_servo.csv
```

扩展：

```text
flowN_fbbr_trigger_cycles.csv
flowN_fbbr_bins.csv
flowN_fbbr_event_windows.csv
flowN_fbbr_cruises.csv
```

最终列宽检查：

| Trace | Columns | Bad rows |
|---|---:|---:|
| trigger cycles | 54 | 0 |
| queue servo | 27 | 0 |
| event windows | 188 | 0 |

关键新增字段包括 delivery/queue period、prominence、match、queue noise floor、combined
source/confidence、servo state/factor、fast queue statistics、transition cycles、baseline commit、
双通道计数、hard safety、event windows、slow updates 和 trusted publications。

## 11. 确定性测试

最终命令：

```bash
cd /home/wkd/BBR_ICC/NS3.27
LD_LIBRARY_PATH=build ./build/scratch/fbbr_4flow \
  --fbbrFrequencySearchSelfTest=true
```

最终摘要：

```text
trigger_correct: PASS
trigger_queue_only: QUEUE_ONLY, eta_q=3.02677, rho_q=1
trigger suite: TP=3 FN=0 TN=5 FP=0
precision=1 recall=1 false_rate=0 median_latency_cycles=1
queue servo deterministic suite: PASS
four-flow pulser election rotation: PASS
OVERALL RESULT: PASS
```

负对照覆盖：

```text
actual input absent
weak one-MSS response
wrong period
same-frequency phase-inconsistent noise
capacity step without causal carrier
different-frequency shared fluctuation
trigger-only selection bias
overlap diagnostic update isolation
```

## 12. FreqCCv4 回归与混合隔离

最终结果：

| Check | Result |
|---|---|
| FreqCCv4 gate state machine | PASS |
| FreqCCv4 trusted selection | PASS |
| FreqCCv4 trusted pacing | PASS |
| source isolation | PASS |
| mixed trace isolation | PASS |

混合运行中：

```text
F-BBR flow only generated fbbr traces
FreqCCv4 flow only generated freq traces
cross_count = 0
```

## 13. 最终代码 Smoke

产物：

```text
NS3.27/docs/fbbr_dual_channel_queue_servo/
  execution_20260713_final_code_smoke/
```

该矩阵使用缩短时长，只验证最终二进制、self-test、FreqCCv4 回归、source isolation、
mixed isolation、trace schema 和 analyzer 端到端执行，不用于性能验收。

结果：

```text
build PASS
F-BBR self-test PASS
FreqCCv4 regressions PASS
source isolation PASS
mixed trace isolation PASS
10/10 run trace hashes unique
```

## 14. FULL 单种子代表矩阵

产物：

```text
NS3.27/docs/fbbr_dual_channel_queue_servo/
  execution_20260713_final_full_seed1/
```

时长：single/four/A-B 为 45 秒，reserve sweep 为 30 秒，dynamic 为 50 秒。

| Run | D/Q/B | Windows | Measurable | Trusted | Tail util | Tail q95 BDP | Tail factor | Result |
|---|---:|---:|---:|---:|---:|---:|---:|---|
| single seed1 | 26/0/0 | 22 | 0.9167 | 0 | 0.9156 | 0.0212 | 0.9600 | FAIL_QUEUE_TRIGGER; FAIL_QUEUE_RESERVE_TARGET; FAIL_TRUSTED_BW_INACCURATE |
| four seed1 | 3/0/0 | 0 | n/a | 0 | 0.9188 | 0.1508 | 0.9362 | FAIL_QUEUE_SERVO; FAIL_QUEUE_RESERVE_TARGET |
| A/B current | 3/0/0 | 0 | n/a | 0 | 0.9789 | n/a | n/a | PASS |
| A/B new | 21/1/0 | 5 | 0.8000 | 0 | 0.9289 | 0.1414 | 0.9557 | FAIL_QUEUE_SERVO; FAIL_QUEUE_RESERVE_TARGET; FAIL_TRUSTED_BW_INACCURATE |
| dynamic | 1/0/0 | 0 | n/a | 0 | 0.6853 | 2.8544 | 0.7000 | FAIL_DYNAMIC_TRACKING; FAIL_QUEUE_SERVO |

FULL 结果说明：

- 单流不再出现“全部 pulser cycles 0 trigger”；event windows 多数可测。
- 四流 pulser/window 覆盖仍不足。
- 新 queue servo 相对旧 DRate-only control 付出约 5 个百分点 utilization 代价。
- 延长到 45 秒仍无 trusted publication，不能归因于短时长。

## 15. 10-Seed 随机矩阵

产物：

```text
NS3.27/docs/fbbr_dual_channel_queue_servo/
  execution_20260713_final_random10/
```

随机主场景为 10 个 single + 10 个 four，每个 15 秒；另含 A/B、reserve、dynamic
和 mixed isolation，共 28 runs。

### 15.1 可识别性

```text
run_count = 28
unique_trace_hashes = 28
experiment identifiability = PASS
```

### 15.2 Single-flow 聚合

```text
runs                         10
delivery-or-BOTH run coverage 100%
queue-only-or-BOTH coverage    0%
capture run coverage           100%
total completed windows        157
median measurable ratio        0.6818
median tail utilization        0.9187
median tail q95 BDP            0.0571
median tail servo factor       0.9525
trusted run coverage           0%
unique hashes                  10/10
```

最终邻频修正后的独立 3-seed 单流复测仍为全部 delivery-only，说明
`FAIL_QUEUE_TRIGGER` 不是旧 `2f0` 邻频引用单独造成，而是 queue response 本身低于
严格 MAD 门限。

### 15.3 Four-flow 聚合

```text
runs                         10
delivery-or-BOTH run coverage 90%
queue-only-or-BOTH coverage   10%
capture run coverage          50%
total completed windows       18
median measurable ratio       0.6667 (有独立窗口的 runs)
median tail utilization       0.9367
median tail q95 BDP           0.1281
median tail servo factor      0.9466
trusted run coverage          0%
unique hashes                 10/10
```

## 16. Reserve Sweep

FULL 30 秒，四流，buffer 2 BDP：

| Reserve | Tail util | Tail q95 BDP | Tail factor | Measurable ratio | Result |
|---:|---:|---:|---:|---:|---|
| 0% | 0.7559 | 0.1100 | 0.7000 | 0.4667 | FAIL_QUEUE_SERVO; FAIL_QUEUE_RESERVE_TARGET |
| 1% | 0.8737 | 0.0193 | 0.8400 | 0.8462 | FAIL_QUEUE_RESERVE_TARGET |
| 2% | 0.9323 | 0.0633 | 0.9500 | 0.8889 | FAIL_QUEUE_RESERVE_TARGET |
| 5% | 0.9405 | 0.0824 | 0.9600 | 0.6429 | FAIL_QUEUE_RESERVE_TARGET |
| 10% | 0.9504 | 0.1284 | 0.9607 | 0.0000 | FAIL_QUEUE_SERVO; FAIL_QUEUE_RESERVE_TARGET |

没有档位同时满足：

```text
utilization >= 0.99
q95 <= 0.10 BDP
```

因此没有把任何 sweep 档位宣称为“验证通过的默认 reserve”。当前默认 2%-5% BDP
仍保留为开发初值，而不是普适参数。

## 17. A/B 结果

45 秒四流固定 seed 101：

```text
current DRate-only tail utilization = 0.9789
new dual-channel+servo utilization = 0.9289
delta = -0.0500

new completed windows = 5
new measurable ratio = 0.80
new tail q95 = 0.1414 BDP
new trusted publications = 0
```

新算法解决了 control 侧 event-window 缺失的一部分问题，但默认 servo 未形成可接受的
吞吐/队列 Pareto 改善。

## 18. Dynamic 结果

50 秒四流，容量 100/70 Mbps 与背景流变化：

```text
tail utilization = 0.6853
tail q95 = 2.8544 BDP
tail factor = 0.70
completed event windows = 0
trusted publications = 0
```

状态变化后没有在 3 个 eligible CRUISE 内恢复 direction/bracket，判定：

```text
FAIL_DYNAMIC_TRACKING
FAIL_QUEUE_SERVO
FAIL_QUEUE_RESERVE_TARGET
```

## 19. 验收矩阵

| 规范项 | 结果 | 证据 |
|---|---|---|
| 独立 F-BBR 路径 | PASS | source + mixed isolation |
| Build | PASS | waf build |
| Deterministic dual trigger | PASS | precision/recall 1.0, FP 0 |
| Hard safety | PASS | 3% loss -> factor 0.70 |
| Continuous 1-BDP drain | PASS | deterministic <=10 RTT |
| Reserve recovery <=2%/RTT | PASS | deterministic |
| No duplicate slow correction | PASS | deterministic + baseline error hold |
| Baseline commit <=2% | PASS | deterministic |
| Persistent listener/search | PASS | cruise trace + self-test |
| Random experiment identifiable | PASS | 28/28 unique hashes |
| Real single queue trigger >=80% runs | FAIL_QUEUE_TRIGGER | 0/10 |
| Capture ratio >=70% | FAIL_EVENT_WINDOW_NOT_MEASURABLE | single 10/10, four 5/10 |
| Independent measurable >=60% | FAIL_EVENT_WINDOW_NOT_MEASURABLE | medians 0.6818/0.6667，但部分 runs 为 0 |
| Queue q95 <=0.10 BDP | FAIL_QUEUE_SERVO | four/default/dynamic fail |
| Utilization >=99% | FAIL_QUEUE_RESERVE_TARGET | no reserve passes |
| Score correlation | FAIL_SCORE_NOT_CORRELATED | Spearman NaN |
| TRACK/LOCK convergence | FAIL_SEARCH_NOT_CONVERGED | no real TRACK/LOCK acceptance |
| trusted MAPE/P90/bias/CV | FAIL_TRUSTED_BW_INACCURATE | publication count 0 |
| Dynamic recovery | FAIL_DYNAMIC_TRACKING | util/q95 fail |

## 20. 失败根因

### 20.1 Queue servo 与 ProbeBW phase interaction

CRUISE 入口常承接 BBR ProbeBW UP 形成的 queue。默认 `q_H=0.05 BDP` 时，
`q_floor >= 4*q_H` 经常立即进入 `0.70` emergency drain。即使 1-3 RTT 内回到 band，
后续从 0.70 恢复到接近 1 的面积损失仍显著降低平均利用率。

### 20.2 Single-flow queue trigger

servo 在 trigger listener 完成两个周期前已经把深队列送回浅队列/underload 区，真实
delivery carrier 变强，而 queue carrier amplitude 低于严格 residual MAD。当前架构中
trigger 与 servo 并行，但 hard safety 比频域 listener 更快，造成单流最终主要由 delivery
通道开窗。

### 20.3 Trusted publication

trusted 要求 factor 接近 1、无 transition、near-optimal queue band、独立 lockable score
和候选共识。真实窗口虽然可测，但 factor 常为 0.94-0.96，或 q95/score/dynamic gate
不满足，因此没有候选进入最终 publication。该行为符合安全门，但未完成论文目标。

### 20.4 Dynamic capacity

容量下降与背景流变化使 absolute queue 长期触发 emergency factor，而 slow loop 又按职责
隔离不能在 servo transient 时快速改写 `B_s`。缺少跨 phase 的容量变化快速重锚机制，
导致 queue 长期高、event listener 不可用。

## 21. 可复现命令

最终代码 smoke：

```bash
cd /home/wkd/BBR_ICC/NS3.27
PROFILE=QUICK SEEDS=1 SINGLE_SECONDS=6 FOUR_SECONDS=6 \
  RESERVE_SECONDS=2 DYNAMIC_SECONDS=6 \
  ./run_fbbr_dual_channel_queue_servo_validation.sh \
  docs/fbbr_dual_channel_queue_servo/execution_20260713_final_code_smoke
```

规范时长代表矩阵：

```bash
PROFILE=FULL SEEDS=1 \
  ./run_fbbr_dual_channel_queue_servo_validation.sh \
  docs/fbbr_dual_channel_queue_servo/execution_20260713_final_full_seed1
```

10-seed 随机矩阵：

```bash
PROFILE=QUICK SEEDS=1,2,3,4,5,6,7,8,9,10 \
  RESERVE_SECONDS=4 DYNAMIC_SECONDS=12 \
  ./run_fbbr_dual_channel_queue_servo_validation.sh \
  docs/fbbr_dual_channel_queue_servo/execution_20260713_final_random10
```

## 22. 产物索引

```text
/home/wkd/BBR_ICC/FBBR_DUAL_CHANNEL_CURRENT_CODE_MAP.md
/home/wkd/BBR_ICC/FBBR_DUAL_CHANNEL_QUEUE_SERVO_DESIGN_REPORT.md

/home/wkd/BBR_ICC/NS3.27/docs/fbbr_dual_channel_queue_servo/
  execution_20260713_final_code_smoke/
  execution_20260713_final_full_seed1/
  execution_20260713_final_random10/
  final_neighbor_seed1/
  final_neighbor_seed2/
  final_neighbor_seed3/
  loss_diag/
```

每个 execution 目录包含：

```text
logs/build.log
logs/fbbr_deterministic_selftest.log
logs/freqccv4_*selftest.log
logs/source_isolation.result
logs/mixed_trace_isolation.result
configs/*.conf
per-run run_meta.json
flowN_fbbr_*.csv
*_F-BBR_good.txt
analysis/dual_channel_queue_servo_summary.csv
analysis/dual_channel_queue_servo_summary.md
analysis/dual_channel_queue_servo_aggregate.json
```

## 23. 后续工程方向

本轮没有通过降阈值、伪造 oracle、把 servo pacing 当 trusted 或修改 FreqCCv4 来隐藏
失败。下一步若继续，应针对结构问题，而不是继续调 score：

1. 让 queue servo 在 ProbeBW 非 CRUISE phase 也能预防 CRUISE 入口深队列，或增加
   phase-aware queue handoff，减少每次入口的 0.70 emergency 面积。
2. 为 listener 保留 hard-safety 前的短 queue carrier ring-buffer，避免 servo 先把
   queue-only 证据擦除；仍需保持 trigger cycle 不进入 score。
3. 增加 capacity-change fast anchor，使动态场景能安全地把持续 servo correction 小步
   提交到 `B_s`，而不是永久停在 transient hold。
4. 先产生真实 near-optimal target-region samples，再校准 score 和 trusted gate；在
   Spearman 为 NaN 时不应继续放宽 LOCK 阈值。
