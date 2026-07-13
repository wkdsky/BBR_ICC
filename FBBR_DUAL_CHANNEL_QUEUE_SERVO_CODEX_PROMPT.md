# F-BBR：双通道事件触发、RTT 级浅队列伺服与持续频域搜索完整改造规范（Codex 执行稿）

> 目标仓库：`/home/wkd/BBR_ICC`  
> 核心模块：`fbbr_sender.{h,cc}`、`fbbr_opiv2.{h,cc}`（建议重命名为 `fbbr_frequency_search.{h,cc}`）、F-BBR pacer 接口、F-BBR trace、runner、analyzer、plotter  
> 本轮目标：解决固定起点/固定长度窗口在真实多流环境中可测率低、决策延迟长、DRAIN 越过公平份额、SEEK 恢复过慢、trusted_bw 长期不发布的问题。  
> 算法正式名称保持 **F-BBR**。内部模块可命名为 `event_triggered_frequency_search`，不要再更改算法对外名称。

## F-BBR-only 硬边界

本任务只研究和改造 F-BBR。FreqCCv4 已经是另一篇论文、另一种算法和另一套代码路径。Codex 必须遵守：

```text
F-BBR CC type:        kFBBR
F-BBR sender:         FBBRSender
F-BBR config:         FBBRConfig
F-BBR config file:    fbbr_default.conf
F-BBR CLI:            --fbbrConfig
F-BBR scenario:       fbbr_4flow
F-BBR trace prefix:   fbbr
```

禁止：

1. 在 `freqccv4_sender.*` 中增加任何 F-BBR 分支；
2. 让 `FBBRSender` 继承、持有或转换为 `FreqCCv4Sender`；
3. 使用 `FreqBbrConfig` 或 `ConfigureFreqBbr()` 配置 F-BBR；
4. 让 CLI 名称 `F-BBR` 映射到 `kFreqCCv4`；
5. 将 F-BBR trace 写入 `freq_*` 文件；
6. 为了复用旧代码而改变 FreqCCv4 的行为。

允许修改的公共文件仅限注册和分发职责，例如 enum、factory、`DqcSender`、trace manager、runner 和 build script；所有公共修改都必须保留两个算法的独立构造、独立配置和独立 trace。

优先将内部文件 `fbbr_opiv2.{h,cc}` 重命名为：

```text
fbbr_frequency_search.{h,cc}
```

并将新类使用纯 F-BBR 名称，例如：

```text
FBBRFrequencySearch
FBBREventWindowAnalyzer
FBBRSearchController
```

如暂时不能安全重命名物理文件，必须至少保证新类、配置、trace、报告和外部接口不再出现 `OPIv2` 或 `FreqCCv4`。

修改公共注册代码后，只运行 FreqCCv4 回归测试确认隔离；不得把任何新控制逻辑写入 FreqCCv4。

---

# 0. 必须先承认并保留的最新实验事实

开始编码前必须完整阅读：

```text
FBBR_EVENT_TRIGGERED_DYNAMIC_WINDOW_FINAL_REPORT.md
FBBR_EVENT_WINDOW_CURRENT_CODE_MAP.md（若存在）
CURRENT_FBBR_CODE_MAP.md（若存在）
fbbr_sender.h/.cc
fbbr_frequency_search.h/.cc
fbbr_default.conf
fbbr_4flow.cc
analyze_fbbr_event_triggered.py
run_fbbr_event_triggered_validation.sh
```

首先生成：

```text
FBBR_DUAL_CHANNEL_CURRENT_CODE_MAP.md
```

必须映射：

- F-BBR 进入和离开 CRUISE 的准确调用链；
- actual packet emission、delivery rate、latest RTT、loss、ECN 的采集路径；
- 当前 pulser/watcher、carrier、事件窗口和 dense sliding 实现；
- 当前 RTprop anchor、queue estimate、queue score；
- 当前 search baseline、DRAIN、TRACK、LOCK、trusted publication；
- 当前所有 `f_bbr.*` 配置与 `flowN_fbbr_*` trace；
- 本文每项改造对应的代码位置。

当前独立 F-BBR 已经完成动态窗口和 mild DRAIN，但真实结果仍为：

```text
4 flows / 100 Mbps / 2 BDP:
tail utilization ≈ 97.54%–97.86%
q_min / q95 ≈ 1.09 / 1.10 BDP
TRACK = 0
LOCK = 0
trusted_bw publication = 0
```

真实事件窗口一旦完成：

```text
independent measurable ratio = 100%
direction accuracy = 100%
Spearman point estimate ≈ 0.685
```

但触发覆盖和闭环失败：

```text
1-flow 45 s:
36 pulser cycles
0 trigger

8-flow:
trigger/completion coverage very low

dynamic capacity:
后两个稳态段 utilization 约 47.4% 和 72.9%
```

当前普通 queue excess 只在稀疏事件窗口后执行一次约 3.7% mild DRAIN，不能把约 1.1 BDP standing queue 持续拉回 0.02–0.05 BDP 目标带。

因此本轮不得通过以下方式“修结果”：

1. 只放宽 score 或 LOCK 阈值；
2. 只提高 probe amplitude；
3. 只延长窗口或 CRUISE；
4. 继续把 DRate 同频响应作为唯一开窗条件；
5. 继续把 queue DRAIN 绑定在稀疏事件窗口上；
6. 把 queue-servo 临时 pacing 当作 trusted_bw；
7. 用理论公平份额或瓶颈 queue oracle 输入生产控制器；
8. 用无 publication 解释为安全成功。
# 1. 新方案的核心判断与最终架构

当前失败的核心不是事件窗口本身，而是两个结构性错误：

```text
错误 1：只有 DRate 出现目标周期才开窗。
错误 2：只有事件窗口完成后才进行一次 queue DRAIN。
```

在瓶颈未饱和时，实际发送扰动主要透传到 delivery rate；但在瓶颈已饱和、队列非空时，delivery rate 会被服务速率削平，扰动主要进入 queue：

$$
\delta d(t)\approx\delta s(t),\qquad \text{UNDERLOAD}
$$

$$
\frac{dq(t)}{dt}\propto R_{\Sigma}(t)-C,\qquad \text{SATURATED}
$$

因此，单流满载时 DRate 周期很弱可能是正常的饱和现象，不能继续解释为“没有可测周期”。

F-BBR 必须改为三个并行、严格分工的回路：

```text
Actual packet emission
        │
        ├── Delivery-rate carrier detector
        │
        ├── Queue-derivative carrier detector
        │
        └── Absolute queue / loss / ECN safety detector
                        │
                        ▼
             Dual-channel event trigger
                        │
                        ▼
            Frequency slow-loop estimator
                        │
                        ▼
               Search baseline B_s
                        │
            ┌───────────┴───────────┐
            ▼                       ▼
RTT-level queue reserve servo   Periodic probe generator
       factor g_q(t)             carrier 1+aφ(t)
            └───────────┬───────────┘
                        ▼
       final CRUISE pacing = B_s × g_q × (1+aφ)
```

职责：

### 频域慢环

- 判断 sustainable bandwidth 应升、降或保持；
- 更新 `search_baseline`；
- 建立 bracket；
- 生成 candidate；
- 发布 `trusted_bw`。

### 队列快环

- 每 RTT 持续把 queue 拉回浅队列储备带；
- 深队列时连续排空，而不是只做一次 DRAIN；
- 队列过低时提供小幅临时恢复；
- 不写入 `trusted_bw` 或 BBRv2 MaxBw；
- 进入目标带后回到 unity gain。

### 周期探针

- 提供已知频率的因果激励；
- 支持 DRate 和 queue-derivative 两条通道的 lock-in/harmonic regression；
- amplitude 由可辨识性和 queue budget 共同约束。
# 2. 新的目标操作区：浅队列饱和带

F-BBR 不再将“每个负半周期都排空到零队列”作为正常目标。

目标改为：

\[
\overline{R_{\mathrm{send}}}
\approx
\overline{R_{\mathrm{delivery}}}
\]

\[
q_{\min,\mathrm{cycle}}
\in [q_L,q_H]
\]

\[
\dot q_{\mathrm{mean}}\approx 0
\]

其中：

- `q_min_cycle`：负向 lobe 后半段的周期内最低排队时延；
- `q_L > 0`：防止瓶颈暂时欠载的最低队列储备；
- `q_H`：浅队列上界；
- `q_peak_cap`：探针正向 lobe 允许的峰值上界。

生产控制中禁止用“平均 pacing rate 长期高于瓶颈公平份额”维持队列。正队列储备属于 inflight/短期 correction 维度，不得写入 sustainable bandwidth estimate。

## 2.1 初始队列储备带

开发期默认：

```text
q_reserve_low_bdp  = 0.02
q_reserve_high_bdp = 0.05
q_peak_cap_bdp     = 0.10
```

换算为 queue delay：

\[
q_L = 0.02\cdot RTprop
\]

\[
q_H = 0.05\cdot RTprop
\]

\[
q_{\mathrm{peak-cap}}=0.10\cdot RTprop
\]

这些是需要实验验证的初始值，不得声称为普适常数。最终应从：

```text
1%, 2%, 5%, 10% BDP
```

的扫描中选取能维持目标利用率的最小储备。

## 2.2 RTprop 校准

正常 TRACK 不要求队列触零；但没有绝对 RTT 基准时，纯交流频率响应无法判断 standing queue 的绝对高度。

因此保留稀疏校准：

```text
正常窗口：目标是 q_min 落入 [q_L, q_H]
校准窗口：低占空比、更深负向 lobe，尝试刷新 RTprop anchor
```

触发校准的条件：

```text
RTprop anchor TTL 到期
路径/native min_rtt 明显变化
sender qdelay 与外部行为长期冲突
连续多个窗口 q_floor 判断不稳定
长期没有 trusted_bw
```

校准失败不得关闭频域搜索，只降低 queue-band confidence 和禁止 LOCK/publication。

---

# 3. 窗口状态机

新增独立的事件触发采样状态机：

```cpp
enum class EventWindowState {
  kIdleListen,
  kTriggerArmed,
  kCapture,
  kContinuousTrack,
  kPaused,
  kPostBaselineSettling
};
```

语义：

## 3.1 IDLE_LISTEN

- CRUISE 内持续运行；
- 使用固定相位 bin 或轻量 cycle accumulator；
- 保留最近 `2 * carrier_period` 的 ring buffer；
- 每完成一个候选周期执行一次轻量 trigger detector；
- 不运行完整 OPI score；
- 即使连续多个周期/多个 CRUISE 无触发，也不得关闭载波搜索。

## 3.2 TRIGGER_ARMED

- 检测到一个满足 trigger 条件的 DRate 周期；
- 从 ring buffer 回溯到该周期的估计起点；
- 该周期只标记为 `trigger_cycle`；
- 可以存入窗口，但不得参与最终 optimality score 和 baseline decision；
- 下一个完整周期开始进入独立确认。

## 3.3 CAPTURE

- 顺序累积独立确认周期；
- 达到最小周期数后，检查估计置信区间；
- 若信息充分，提前完成窗口；
- 若仍不足，继续到最大周期数；
- 不得固定等待 12 cycles 才第一次决策。

## 3.4 CONTINUOUS_TRACK

若第一个窗口完成后周期响应仍连续：

- 进入密集滑窗；
- 继续使用最近的连续响应区间；
- 窗口长度保持 6–10 个有效周期；
- 窗口起点步进改为 `0.5 carrier cycle`；
- 诊断候选允许 `0.25 cycle` 步进；
- 控制决策最短间隔仍为 `1 full cycle`；
- trusted candidate 的独立样本间隔至少为 `window_length / 2`。

密集滑窗用于提高样本丰度，但不能把所有重叠窗口当成独立证据。

## 3.5 PAUSED

当连续周期不再符合预期：

- 停止构造完整采样窗口；
- 保留 IDLE_LISTEN 轻量检测；
- baseline 不因低分自动变化；
- probe period/amplitude 可根据失败原因自适应；
- 下一个符合条件的周期出现时重新触发；
- 不得全面回退并关闭频域尝试。

## 3.6 POST_BASELINE_SETTLING

每次 search baseline 或 probe amplitude/period 变化后：

- 至少跳过 1 个完整 carrier cycle；
- 该周期不允许触发或评分；
- 防止 baseline ramp 瞬态被误识别为 DRate 周期响应。

---

# 4. 双通道动态起点 Trigger Detector

动态窗口思路保留，但 trigger 从 DRate-only 改成：

```text
actual input measurable
AND
(
    delivery-rate branch pass
    OR
    queue-derivative branch pass
)
```

另设 hard-safety 旁路。Trigger 只负责何时开窗，不等于运行点得分。

## 4.1 实际输入前提

actual input 必须由真实 packet emission bytes 在固定 phase bin 中重建：

$$
x(t)=\frac{R_{\mathrm{actual}}(t)-B_s(t)g_q(t)}{B_s(t)g_q(t)}
$$

commanded pacing 仅作 actuator diagnostic。不得重新引入：

```text
actual amplitude / commanded amplitude >= 0.75
```

Hard invalid 仅包括：

```text
actual input information insufficient
coverage insufficient
regression singular / severely ill-conditioned
application-limited
recovery
未建模的 baseline / servo / period step
```

## 4.2 Delivery-rate 分支

在已知载波频率上计算：

$$
H_d(j\omega)=\frac{D(j\omega)}{X(j\omega)}
$$

以及：

$$
\eta_d=\frac{|D(f_m)|}{\max_{f\in\mathcal N(f_m)}|D(f)|+\epsilon}
$$

$$
\rho_d=\operatorname{phaseInvariantMatch}(x,d)
$$

启动初值：

```text
delivery prominence >= 2.0
delivery normalized match >= 0.50
delivery response >= 4 SMSS/cycle
period error <= 15%
```

持续初值：

```text
delivery prominence >= 1.5
delivery normalized match >= 0.35
period error <= 20%
```

## 4.3 Queue-derivative 分支

使用 latest/raw RTT，不使用 SRTT 作为主信号：

$$
q_i=\max(RTT_i-RTprop_{\mathrm{anchor}},0)
$$

不要对噪声 RTT 做简单逐 bin 差分。优先在 harmonic regression 中直接计算：

$$
H_{\dot q}(j\omega)=\frac{j\omega Q(j\omega)}{X(j\omega)}
$$

若需要时域导数，使用局部线性或 Savitzky–Golay，不得使用裸一阶差分。

定义：

$$
\eta_q=\frac{|\dot Q(f_m)|}{\max_{f\in\mathcal N(f_m)}|\dot Q(f)|+\epsilon}
$$

$$
\rho_q=\operatorname{phaseInvariantMatch}(x,\dot q)
$$

启动初值：

```text
queue prominence >= 2.0
queue normalized match >= 0.50
period error <= 15%
RTT phase coverage >= 75%
```

持续初值：

```text
queue prominence >= 1.5
queue normalized match >= 0.35
period error <= 20%
```

queue-response 的绝对下限采用噪声自适应门槛：

$$
A_q\ge\max\left(3\cdot1.4826MAD(q_{\mathrm{residual}}),\;2\cdot RTT_{\mathrm{timestamp\ resolution}}\right)
$$

## 4.4 联合触发

两个分支分别输出：

```text
C_d ∈ [0,1]
C_q ∈ [0,1]
```

联合：

$$
C_{\mathrm{trigger}}=1-(1-C_d)(1-C_q)
$$

触发来源必须 trace：

```text
DELIVERY_ONLY
QUEUE_ONLY
BOTH
HARD_SAFETY_ONLY
NONE
```

这必须修复当前单流满载时 36 个 pulser cycles、0 trigger 的问题：若 DRate 被削平但 queue-derivative 与 actual input 相干，必须进入 `QUEUE_ONLY`。

## 4.5 Hard-safety 旁路

满足任一：

```text
loss >= hard threshold
ECN >= hard threshold
q_floor > hard_queue_multiple * q_H
q95 接近 buffer saturation
queue trend 持续强正
```

无需等待频域 trigger，直接允许 queue servo 进入强排空状态。

Hard-safety：

- 不生成高 score；
- 不生成 trusted candidate；
- 不声明频域可测；
- 只负责保护和把系统送回可辨识区。

## 4.6 动态起点

检测到 trigger 后，从 ring buffer 回溯：

- 以 actual-input carrier phase boundary 为主；
- 在 ±0.25 cycle 内选 matched-filter likelihood 最大的边界；
- 输出 `detected_cycle_start` 和 `alignment_error_cycles`。

Trigger cycle 必须排除：

```text
trigger_cycle_excluded_from_score = true
trigger_cycle_excluded_from_direction = true
trigger_cycle_excluded_from_candidate = true
trigger_cycle_excluded_from_trusted = true
```
# 5. 避免选择偏差

动态起点会天然选择“看起来像周期”的数据，因此必须强制以下不变式：

1. `trigger_cycle` 不参与 `S_target`；
2. `trigger_cycle` 不参与 gradient equivalence；
3. `trigger_cycle` 不生成 trusted candidate；
4. 至少需要 4 个触发后独立周期才能输出第一次方向判断；
5. 至少需要 6 个触发后有效周期才能输出 LOCKable score；
6. trigger threshold 与 score threshold 分开配置；
7. 必须设置负对照，验证 trigger 之后并不会系统性抬高 non-ideal score。

---

# 6. 动态窗口长度与顺序停止

固定 12-cycle memory 导致上一轮第一次 decision 约 6.86 s，过慢。

新窗口：

```text
min_direction_cycles = 4
min_score_cycles     = 6
target_cycles        = 8
max_cycles           = 12
```

每增加 1 个周期后更新递归回归和置信区间。

## 6.1 方向判断提前停止

满足全部条件即可提前输出方向：

```text
effective_cycles >= 4
gradient 90% CI 不跨 0
或 underload/overload state evidence >= 0.75
direction 在最近 2 次周期更新中符号一致
```

## 6.2 score/LOCK 判断停止

满足全部条件即可完成窗口：

```text
effective_cycles >= 6
measurement SE 达标
queue-band estimate CI 足够窄
S_target 最近 2 次更新变化 <= 0.10
classification 最近 2 次一致
```

## 6.3 达到最大长度仍不确定

```text
window result = UNCERTAIN
baseline ordinary update = HOLD
频域监听继续
根据失败原因调 period/amplitude
```

禁止因为一个不确定窗口而永久退出搜索。

---

# 7. 连续周期下的密集滑窗

当一个完整窗口结束后，若下一个周期仍满足：

```text
eta_d >= 1.5
rho_d >= 0.35
period match <= 20%
```

则保持 `CONTINUOUS_TRACK`。

默认：

```text
tracking_window_cycles = 8
tracking_stride_cycles = 0.5
diagnostic_stride_cycles = 0.25
control_decision_stride_cycles = 1.0
```

停止连续滑窗：

```text
连续 2 个周期不满足 continue 条件
或 change-point / active-set step
或 loss/recovery
或 baseline/amplitude/period 改变
```

使用双周期失败而不是单周期失败，避免偶然噪声造成频繁开关。

---

# 8. Probe period 与 amplitude 自适应

双通道实现后，DRate 弱不再自动等价于“需要增大 amplitude”。必须根据两条通道分别判断。

## 8.1 分支诊断

每个周期输出：

```text
actual input measurable?
delivery branch measurable?
queue branch measurable?
both weak?
probe queue cost?
packetization floor?
```

## 8.2 自适应规则

### 情况 A：DRate 弱、queue branch 强

```text
瓶颈已饱和，扰动主要进入 queue
不增加 amplitude
保持或降低 amplitude
```

### 情况 B：DRate 强、queue branch 弱

```text
偏欠载
保持 amplitude
由 slow-loop SEEK_UP 调 baseline
```

### 情况 C：两条通道都弱，但 actual input 可测

优先：

```text
carrier_period *= 1.25
```

达到 period 上限后：

```text
commanded_amplitude *= 1.20
```

单次 amplitude 变化不超过 20%。

### 情况 D：actual input 不可测

优先检查：

```text
pacer lateness
cwnd-limited fraction
send quantum
packetization
phase coverage
pulser lease
```

然后延长 period、调整 phase/bin 或重新 pulser election。

## 8.3 Probe queue budget

$$
\Delta Q_{\mathrm{probe}}=\int[R_{\mathrm{probe}}(t)]^+dt
$$

上限：

```text
ACQUIRE/SEEK <= 0.10 BDP
TRACK <= 0.05 BDP
LOCKED <= 0.02 BDP
EMERGENCY_DRAIN：关闭强 probe
```

## 8.4 长期无 trigger

即使连续多个 CRUISE 无 trigger：

- 不关闭 F-BBR frequency search；
- 每个 eligible CRUISE 继续双通道 listener 和 acquisition；
- 可以降低 duty cycle，但不能为零；
- period/amplitude/slot/phase 状态跨 CRUISE 保存；
- 必须 trace 实际尝试次数。
# 9. Pulser / watcher

保留上一轮已验证的 single-pulser 思路，但修正其用途。

## 9.1 Pulser

只有 pulser 的事件窗口可以驱动该流的 search-baseline 更新。

## 9.2 Watcher

watcher 可以用公共周期响应：

```text
检测 bottleneck carrier 是否存在
更新公共 queue/RTprop diagnostics
检测 pulser collision
为下一轮 pulser 选择 period/phase
```

但 watcher 不能仅凭其他流的 carrier 更新自己的 sustainable-rate baseline。

## 9.3 轮换延迟问题

上一轮每条流重新成为 pulser 需要多个 CRUISE，导致 SEEK 恢复极慢。

新规则：

- 一旦 pulser 进入连续可测区，允许其在当前 CRUISE 内完成多个密集决策；
- pulser lease 不按“一个固定窗口”立即结束；
- lease 持续到：
  - 连续响应消失；
  - 发生 baseline change 后重新 settling 仍无法恢复；
  - 达到 `max_pulser_lease_cycles`；
  - hard congestion；
- 默认：

```text
min_pulser_lease_cycles = 8
max_pulser_lease_cycles = 32
```

公平轮换仍必须保证，但不能为了轮换牺牲一次已建立的高质量观测连续段。

---

# 10. 新核心：RTT 级浅队列快环

事件窗口只负责慢速可持续速率辨识，不能继续承担连续 queue control。新增独立：

```text
FBBRQueueReserveServo
```

## 10.1 目标带

开发初值：

```text
q_reserve_low_bdp = 0.02
q_reserve_high_bdp = 0.05
q_peak_cap_bdp = 0.10
```

$$
q_L=0.02RTprop,\qquad q_H=0.05RTprop
$$

这些值必须通过 0/1/2/5/10% BDP sweep 验证，不得称为普适常数。

## 10.2 快环统计

每 RTT 更新：

```text
q_floor_fast = P20(qdelay samples over last RTT)
q_median_fast
q_peak_fast = P90(qdelay samples over last RTT)
queue_trend_fast = robust slope over last 2 RTT
delivery_median_fast
loss_ratio_fast
ecn_ratio_fast
```

若 RTT 样本不足：

- 保持上一 factor；
- 禁止 queue-up correction；
- hard loss 仍可生效。

## 10.3 最终 pacing

定义 queue servo factor：

```text
g_q(t)
```

最终 CRUISE pacing：

$$
R_{\mathrm{final}}(t)=B_s\cdot g_q(t)\cdot[1+a(t)\phi(t)]
$$

其中：

- `B_s` 是 slow-loop search baseline；
- `g_q` 是临时 queue reserve correction；
- `g_q` 不得写入 trusted_bw 或 BBRv2 MaxBw。

## 10.4 高队列连续排空

$$
e_H=\max\left(\frac{q_{\mathrm{floor,fast}}-q_H}{RTprop},0\right)
$$

$$
v_H=\max\left(\frac{RTprop\cdot\dot q_{\mathrm{fast}}}{q_H+\epsilon},0\right)
$$

每 RTT：

$$
\delta_{\downarrow}=\operatorname{clip}(K_He_H+K_Vv_H,0,0.05)
$$

$$
R_{\mathrm{servo}}=B_s(1-\delta_{\downarrow})
$$

若 delivery median 有效且 queue 明显高：

$$
R_{\mathrm{servo}}=\min\left(B_s(1-\delta_{\downarrow}),0.99D_{\mathrm{median,fast}}\right)
$$

默认：

```text
queue_servo_high_gain = 0.50
queue_servo_trend_gain = 0.10
queue_servo_down_step_max = 0.05 per RTT
```

必须连续执行，直到 queue 进入目标带；不能只在事件窗后执行一次。

## 10.5 低队列储备恢复

$$
e_L=\max\left(\frac{q_L-q_{\mathrm{peak,fast}}}{RTprop},0\right)
$$

仅在以下全部满足时允许：

```text
无 loss/ECN
queue trend 非正
非 recovery
flow backlogged
有 UNDERLOAD 频域证据或利用率明显下降
```

$$
\delta_{\uparrow}=\operatorname{clip}(K_Le_L,0,0.02)
$$

$$
R_{\mathrm{servo}}=B_s(1+\delta_{\uparrow})
$$

它只是临时 queue reserve recovery，不直接上调 trusted_bw。

## 10.6 目标带内

若：

```text
q_floor_fast >= q_L
q_peak_fast <= q_H
abs(queue_trend_fast) small
```

则 `g_q` 每 RTT 最多恢复 1%–2%，平滑回到 1.0。

## 10.7 Emergency drain

满足任一：

```text
loss >= 2%
ECN >= hard threshold
q_floor_fast >= 4*q_H
q95 接近 buffer cap
queue trend 持续强正
```

进入：

```text
QUEUE_SERVO_EMERGENCY_DRAIN
```

允许 `g_q <= 0.70`。恢复后双通道 listener 自动继续，不得关闭 F-BBR。

## 10.8 Capture 污染控制

- servo factor 以 RTT 为单位 piecewise constant；
- 所有 factor step 写入 bin trace；
- actual-input regression 使用真实发包；
- 回归中加入 slow servo trend；
- 一个 carrier cycle 内 servo step >2% 时，该 cycle 标记 `servo_transition_cycle`，不参与 score；
- hard-safety 可以终止当前窗口。

## 10.9 Queue servo 与 search baseline 的边界

### 纯 queue excess

如果：

```text
frequency sustainable direction ≈ 0
delivery median 接近 search baseline
只是 standing queue 高
```

则：

```text
queue servo drain
slow baseline HOLD
```

### Search baseline 确实过高

只有同时存在至少两类证据：

```text
servo 连续 N RTT 饱和下调且 queue 仍高
H_qdot 强且 H_d 长期压缩
utility/sustainable direction 显著为负
capacity/native model 下降
loss/ECN
```

才允许将一小部分 servo correction 吸收进 search baseline：

```text
servo_commit_min_rtts = 4
servo_commit_step_max = 2%
```

$$
B_{s,next}=B_s(1-\min(0.02,\delta_{commit}))
$$

普通情况下不得低于：

$$
B_{s,next}\ge0.99D_{median}
$$

该动作标记：

```text
QUEUE_SAFETY_BASELINE_COMMIT
```

它不是 trusted publication。

# 11. 运行点评分

必须分别输出：

```text
C_meas
S_sat
S_band
S_stable
S_target
```

## 11.1 Measurement confidence

Hard-invalid 仅包括：

```text
actual input information insufficient
coverage insufficient
regression singular
application-limited
recovery
unmodeled baseline/servo transition
```

一条输出通道弱不代表整个窗口 invalid。只要另一条通道可辨识，就允许输出方向。

其余置信项采用加权几何平均：

```text
C_delivery_channel
C_queue_channel
C_period
C_delay
C_regression
C_stationarity
C_rtprop
```

## 11.2 通道分配

$$
G_d=|H_d|,\qquad G_q=|H_{\dot q}|
$$

$$
P_d=\frac{G_d}{G_d+G_q+\epsilon},\qquad P_q=\frac{G_q}{G_d+G_q+\epsilon}
$$

解释：

```text
P_d 高：输入主要转化为 delivery，偏欠载。
P_q 高：输入主要转化为 queue，偏饱和/过载。
```

## 11.3 Saturation score

$$
S_{sat}=1-(1-S_{clip})(1-S_{queue-channel})
$$

不要求 DRate clipping 和 queue response 同时很强。

## 11.4 Queue band score

$$
S_{band}=\begin{cases}
\exp[-((q_L-q_{min})/\sigma_L)^2],&q_{min}<q_L\\
1,&q_L\le q_{min}\le q_H\\
\exp[-((q_{min}-q_H)/\sigma_H)^2],&q_{min}>q_H
\end{cases}
$$

## 11.5 Stability score

检查：

```text
queue trend
q95 cap
servo factor 是否接近 1
是否仍在 emergency drain
```

若 queue servo 长期显著小于 1，窗口不能 LOCK。

## 11.6 总分

$$
S_{target}=C_{meas}\cdot S_{sat}^{0.4}S_{band}^{0.35}S_{stable}^{0.25}
$$

loss/ECN 是独立安全门，不作为连续乘法项把总分压成 0。

## 11.7 阈值校准

不得直接把固定的 `0.70/0.80` 当作最终真理。必须：

1. development seeds 生成 ROC/PR；
2. 在 false trusted publication <=5% 条件下选 threshold；
3. 冻结；
4. held-out seeds 验证；
5. 输出 bootstrap CI；
6. 不得使用 held-out seeds 反复调参。
# 12. 频域慢环方向与 search baseline 控制

queue servo 已承担主要 queue correction，slow loop 只控制 sustainable-rate baseline，不能重复处理同一 queue error。

## 12.1 Frequency direction

$$
D_{freq}=w_p(2P_d-1)+w_gD_{utility}
$$

默认：

```text
channel_split_weight = 0.60
utility_gradient_weight = 0.40
```

若 gradient CI 很宽，降低其权重，而不是将整个窗口 invalid。

## 12.2 Queue error

$$
E_q=\begin{cases}
+(q_L-q_{min})/q_L,&q_{min}<q_L\\
0,&q_L\le q_{min}\le q_H\\
-(q_{min}-q_H)/q_H,&q_{min}>q_H
\end{cases}
$$

## 12.3 Slow direction

$$
D_{slow}=0.60D_{freq}+0.25E_q-0.15Q_{trend}
$$

queue 权重低于旧版，避免快环和慢环重复下调。

## 12.4 SEEK_UP

要求：

```text
D_slow > threshold
queue below or inside reserve band
无 congestion
连续 2 个独立 decision 同方向
```

步长：

```text
first confirmed <= 3%
continued <= 5%
strong underload <= 8%
```

## 12.5 SEEK_DOWN

只有 sustainable-rate 负方向成立时才能普通下调。普通下调上限 2%–5%，不得恢复固定 10% DRAIN。

## 12.6 TRACK

满足：

```text
S_target 高
q_min 位于 reserve band
queue trend 近 0
servo factor 接近 1
abs(D_slow) small
```

单次 baseline 变化 <=2%。

## 12.7 Bracket

UNDERLOAD 与 sustainable OVERLOAD 更新双边界。只有双边有效才使用几何中点：

$$
B_m=\sqrt{B_LB_H}
$$

单边 queue safety bound 不能冒充完整 bracket。

## 12.8 更新后的窗口处理

任一 search baseline 更新后：

- 终止旧 dense window；
- 进入 1 cycle settling；
- 重新双通道触发；
- 旧窗口不得继续评价新 baseline。
# 13. trusted_bw

trusted_bw 只表示可持续 delivery share，不包含 queue-servo 临时正/负修正。

候选来源：

```text
TRACK baseline
浅队列饱和区的 delivery plateau
完整 bracket midpoint
```

禁止来源：

```text
trigger cycle
queue-servo temporary pacing
hard/mild drain pacing
单边 bound
高度重叠全部窗口
最高分窗口 full-window mean DRate
```

发布要求：

```text
至少 2 个非高度重叠 LOCKable windows
candidate CV <= 5%
candidate max/min <= 1.10
q_min 位于 [qL,qH]
q95 <= q_peak_cap
queue trend 近 0
queue servo factor 接近 1
低 loss/ECN
RTprop confidence 足够
```

如果始终没有 trusted_bw：

- 非 CRUISE 使用 native BBR；
- CRUISE 内继续双通道监听、queue servo 和 persistent search；
- provisional state 跨 CRUISE 保存；
- provisional baseline 不应用于 REFILL/UP/DOWN；
- 不得把无 publication 记为 MAPE=0。
# 14. 关键配置默认值

所有配置只允许使用 `f_bbr.*`，集中放入 `FBBRConfig` 和 `fbbr_default.conf`。

```text
f_bbr.enabled = true
f_bbr.persistent_across_cruises = true
f_bbr.never_disable_on_unresolved = true

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

f_bbr.trigger.period_tolerance_start = 0.15
f_bbr.trigger.period_tolerance_continue = 0.20
f_bbr.trigger.phase_coverage_min = 0.75
f_bbr.trigger.delay_min_rtprop = 0.50
f_bbr.trigger.delay_max_rtprop = 1.50

f_bbr.window.min_direction_cycles = 4
f_bbr.window.min_score_cycles = 6
f_bbr.window.target_cycles = 8
f_bbr.window.max_cycles = 12
f_bbr.window.tracking_cycles = 8
f_bbr.window.diagnostic_stride_cycles = 0.25
f_bbr.window.control_stride_cycles = 0.50
f_bbr.window.trusted_independent_stride_cycles = 4
f_bbr.window.bad_cycles_to_pause = 2
f_bbr.window.post_update_settling_cycles = 1

f_bbr.queue_reserve.low_bdp = 0.02
f_bbr.queue_reserve.high_bdp = 0.05
f_bbr.queue_reserve.peak_cap_bdp = 0.10

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

f_bbr.direction.channel_split_weight = 0.60
f_bbr.direction.utility_gradient_weight = 0.40
f_bbr.direction.slow_frequency_weight = 0.60
f_bbr.direction.slow_queue_weight = 0.25
f_bbr.direction.slow_trend_weight = 0.15

f_bbr.control.seek_first_step_max = 0.03
f_bbr.control.seek_continuous_step_max = 0.05
f_bbr.control.seek_far_underload_step_max = 0.08
f_bbr.control.seek_down_step_max = 0.05
f_bbr.control.track_step_max = 0.02
f_bbr.control.hard_loss_threshold = 0.02
f_bbr.control.hard_loss_beta = 0.70

f_bbr.probe.period_increase_factor = 1.25
f_bbr.probe.amplitude_increase_factor = 1.20
f_bbr.probe.max_amplitude_change = 0.20
f_bbr.probe.acquire_queue_budget_bdp = 0.10
f_bbr.probe.track_queue_budget_bdp = 0.05
f_bbr.probe.locked_queue_budget_bdp = 0.02

f_bbr.pulser.min_lease_cycles = 8
f_bbr.pulser.max_lease_cycles = 32
f_bbr.pulser.watcher_amplitude = 0.005

f_bbr.trusted.min_independent_windows = 2
f_bbr.trusted.candidate_cv_max = 0.05
f_bbr.trusted.candidate_ratio_max = 1.10
```

禁止散落 magic number。
# 15. Trace

## 15.1 Cycle trigger trace

文件：

```text
flowN_fbbr_trigger_cycles.csv
```

字段至少包括：

```text
flow_id
cruise_id
cycle_id
cycle_start/end
is_pulser
event_window_state
carrier_period
commanded_amplitude
actual_input_amplitude
actual_input_energy
actual_input_snr
phase_coverage

delivery_amp
delivery_prominence
delivery_match
delivery_period_error
delivery_trigger_pass
delivery_continue_pass
delivery_reason

queue_derivative_amp
queue_prominence
queue_match
queue_period_error
queue_trigger_pass
queue_continue_pass
queue_noise_floor
queue_reason

combined_trigger_source
combined_confidence
detected_cycle_start
alignment_error_cycles
hard_safety
```

## 15.2 Queue servo trace

新增：

```text
flowN_fbbr_queue_servo.csv
```

字段：

```text
time
flow_id
cruise_id
servo_state
search_baseline_bps
servo_factor
final_nonprobe_baseline_bps
q_floor_fast
q_median_fast
q_peak_fast
q_low
q_high
q_peak_cap
queue_trend_fast
delivery_median_fast_bps
loss_ratio_fast
ecn_ratio_fast
down_correction
up_correction
consecutive_drain_rtts
baseline_commit_eligible
baseline_commit_applied
baseline_commit_bps
reason
```

## 15.3 Event-window trace

`flowN_fbbr_event_windows.csv` 新增：

```text
trigger_source
H_delivery_gain
H_queue_derivative_gain
delivery_partition
queue_partition
queue_servo_factor_mean
queue_servo_transition_cycles
```

并保留：

```text
C_meas
S_sat
S_band
S_stable
S_target
D_freq
E_queue
D_slow
q_min/q95/trend
baseline before/after
candidate/trusted
overlap/independence
invalid reason
```

## 15.4 CRUISE trace

必须记录：

```text
search_active
dual_trigger_attempts
delivery_triggers
queue_triggers
both_triggers
hard_safety_events
event_windows
dense_windows
queue_servo_updates
queue_servo_drain_rtts
queue_servo_recovery_rtts
baseline_commits
slow_loop_updates
trusted_publications
persistent_retry
```
# 16. Deterministic tests

至少新增并通过：

1. **DRate-only trigger**：actual input 清晰、DRate 强、queue 弱；来源 `DELIVERY_ONLY`，方向为正。
2. **Queue-only trigger**：DRate 被削平、queue derivative 强；来源 `QUEUE_ONLY`，分类为饱和/排队。
3. **Both trigger**：两条通道均清晰；来源 `BOTH`。
4. **两通道都弱但深队列**：hard queue/loss；来源 `HARD_SAFETY`，只触发 servo，不生成 candidate。
5. **单流满载无 DRate 周期**：必须由 queue branch 开窗，修复 0 trigger。
6. **Trigger selection bias**：trigger cycle 强、后续无确认；不得 score/update/candidate。
7. **顺序停止**：4 cycles 输出方向，6 cycles score 稳定，不等满 12 cycles。
8. **Dense overlap**：0.25-cycle diagnostic、0.5-cycle control，trusted independence >=4 cycles。
9. **Queue servo 连续排空**：初始 q=1 BDP，连续 RTT drain，进入 band 前不得停止。
10. **Queue reserve recovery**：q<qL，up correction <=2%/RTT，不更新 trusted。
11. **不重复修正**：queue 高但 sustainable direction≈0；servo drain，slow baseline HOLD。
12. **Baseline commit**：servo 饱和 >=4 RTT、queue 仍高、频域负方向；commit <=2%，普通情况下不低于 0.99 delivery median。
13. **Hard drain**：loss 3%，factor 0.70，恢复后搜索自动继续。
14. **Persistent no trusted**：连续 5 个 CRUISE，无 trusted 但 search/servo/listener 均 active。
15. **F-BBR/FreqCCv4 isolation**：FreqCCv4 回归不变、混合 trace 无交叉。
# 17. 实验与验收

禁止直接跑一个长矩阵替代基础诊断。

## 17.1 Phase A：代码隔离和 self-test

```text
build PASS
F-BBR self-test PASS
FreqCCv4 regression PASS
source isolation PASS
mixed trace isolation PASS
```

## 17.2 Phase B：双通道 trigger

受控场景：

```text
underload delivery response
saturated queue response
both response
period mismatch
same-frequency noise
ACK compression
capacity step
deep queue with weak carrier
```

验收：

```text
delivery precision/recall >=90%/80%
queue precision/recall >=90%/80%
combined false trigger <=5%
median trigger latency <=1.5 cycles
trigger-cycle score leakage = 0
```

## 17.3 Phase C：真实单流 trigger

```text
1 flow
100 Mbps
2 BDP
45 s
10 independent randomized seeds
```

必须加入有种子的：

```text
start jitter
ACK jitter
small cross traffic or pacing jitter
```

验收：

```text
>=80% runs 产生 QUEUE_ONLY 或 BOTH trigger
capture ratio >=70%
不能再次出现全部 pulser cycles 0 trigger
```

## 17.4 Phase D：Queue servo

```text
4 flows
100 Mbps
initial queue = 0.25/0.5/1.0 BDP
buffer = 2 BDP
```

验收：

```text
q95 从约1.1 BDP降至 <=0.10 BDP
进入 reserve band 中位时间 <=10 RTT
排队期间 utilization >=95%
进入 band 后 factor 回到 [0.98,1.02]
```

## 17.5 Phase E：Current vs New A/B

```text
A = 当前 DRate-only event version
B = dual-channel + queue servo
```

验收：

```text
1-flow trigger recall 显著提升
4-flow event completion >=70%
valid direction >=60%
median first direction <=4 cycles
B 的 q95 至少降低80%
```

## 17.6 Phase F：Score vs GT

GT 包括：

```text
utilization
q_min reserve band
q95 cap
queue trend
servo factor near unity
loss/ECN
```

验收：

```text
Spearman rho >=0.60
cluster bootstrap 95% lower >0.40
target recall >=70%
high-score non-target FPR <=5%
```

必须产生真实 target-region samples，否则判实验不可辨识。

## 17.7 Phase G：搜索与 trusted_bw

```text
1/2/4/8 flows
100 Mbps
2 BDP
10 independent randomized seeds
```

验收：

```text
70% flows 在8个独立 decisions内进入TRACK
70% flows 在3个eligible CRUISE内形成candidate
更新后 fair-share 绝对误差中位数下降
trusted median MAPE <=5%
P90 APE <=10%
bias <=3%
cross-flow CV <=8%
Jain >=0.99
```

## 17.8 Phase H：动态环境

至少：

```text
active set 1→2→4→2→1
capacity 100↔70 Mbps
background 0→20→40→0 Mbps
RTT step
ACK compression
non-congestive random loss
pulser collision
```

验收：

```text
变化后3 eligible CRUISE内恢复方向/bracket
升容后 steady utilization >=95%
降容后 q95 <=0.10 BDP
hard recovery 后 search resume=100%
```

## 17.9 Phase I：Queue reserve sweep

扫描：

```text
0/1/2/5/10% BDP
```

选择满足：

```text
utilization >=99%
q95 <=0.10 BDP
```

所需的最小 reserve。

## 17.10 随机性要求

当前旧实验不同 seed 的关键 trace 完全相同。本轮必须增加显式、可重复随机性：

```text
flow start jitter
ACK timing jitter
small seeded cross traffic
packet-size variation
capacity micro-jitter
```

要求：

- 每项可开关；
- 所有随机源记录 seed；
- development 与 held-out seeds 分离；
- 报告 trace hash 唯一性；
- 不同 seeds 仍相同时，判 `FAIL_EXPERIMENT_NOT_IDENTIFIABLE`。
# 18. 最终实现原则

必须在代码和报告中明确：

1. F-BBR 只修改独立 F-BBR 路径；
2. DRate carrier 只是一个触发通道，不是唯一入口；
3. 单流满载时 queue-derivative 通道必须可触发；
4. trigger cycle 不参与 score；
5. queue servo 每 RTT 连续工作，不等待事件窗口；
6. queue servo 只改变临时 pacing，不直接改变 trusted_bw；
7. slow loop 不得与 queue servo 重复处理同一 queue error；
8. DRate 弱而 queue branch 强时不得盲目增加 amplitude；
9. 深 queue 必须先通过 servo 回到可辨识区；
10. 没有 trusted_bw 时仍在每个 eligible CRUISE 继续双通道频域尝试；
11. 只有双边 bracket 才能形成 midpoint candidate；
12. 所有控制效果必须报告更新后的绝对 fair-share error；
13. 没有 publication 时，trusted 精度为 undefined/FAIL；
14. 多 seed 必须具有真实、可重复的随机差异。
# 19. F-BBR 独立性验收

在性能实验之前必须完成：

```bash
rg -n "F-BBR|FBBR|fbbr|kFBBR" \
  src/dqc/model/thirdparty/congestion/freqccv4_sender.h \
  src/dqc/model/thirdparty/congestion/freqccv4_sender.cc \
  scratch/freqccv4_4flow.cc \
  examples/CCconfig/freqccv4_default.conf
```

结果必须为空。

再执行 F-BBR/FreqCCv4 混合隔离回归，要求：

```text
F-BBR flow 只生成 fbbr trace
FreqCCv4 flow 只生成 freq trace
交叉 trace 文件数 = 0
两种算法分别加载自己的配置文件
F-BBR 修改前后 FreqCCv4 deterministic result 一致
```

隔离失败必须使用：

```text
FAIL_FBBR_FREQCCV4_ISOLATION
```

---

# 19.1 禁止的捷径

严禁：

1. 用理论 fair share 或 queue oracle 输入生产控制；
2. 修改 BBRv2 MaxBw sample/filter；
3. 只放宽 score threshold；
4. 只增大 probe amplitude；
5. DRate 弱时直接判 no trigger；
6. queue-only trigger 未经独立确认就产生 candidate；
7. 把 queue-servo pacing 当作 trusted_bw；
8. 每 RTT servo 与 slow loop 同时重复下调；
9. 恢复固定 10% 普通 DRAIN；
10. 把所有重叠窗口视为独立；
11. unresolved 后关闭 frequency search；
12. 无 publication 时报告 MAPE=0；
13. 不引入随机性却报告多 seed CI；
14. 修改 FreqCCv4 实现 F-BBR。

# 20. Codex 最终交付

必须输出：

```text
1. FBBR_DUAL_CHANNEL_CURRENT_CODE_MAP.md
2. FBBR_DUAL_CHANNEL_QUEUE_SERVO_DESIGN_REPORT.md
3. 修改文件清单
4. 配置默认值表
5. F-BBR/FreqCCv4 隔离报告
6. deterministic self-test 日志
7. delivery/queue/combined trigger 报告
8. queue servo 闭环报告
9. current-vs-new A/B 报告
10. randomized multi-seed 报告
11. score-vs-GT 报告
12. baseline convergence/overshoot 报告
13. trusted_bw precision/coverage/cadence 报告
14. dynamic tracking 报告
15. queue-reserve sweep
16. failure cases
17. 一键复现脚本和精确命令
```

最终标签只允许：

```text
PASS
FAIL_FBBR_FREQCCV4_ISOLATION
FAIL_DELIVERY_TRIGGER
FAIL_QUEUE_TRIGGER
FAIL_COMBINED_TRIGGER
FAIL_QUEUE_SERVO
FAIL_EVENT_WINDOW_NOT_MEASURABLE
FAIL_SCORE_NOT_CORRELATED
FAIL_SEARCH_NOT_CONVERGED
FAIL_BASELINE_OVERSHOOT
FAIL_QUEUE_RESERVE_TARGET
FAIL_TRUSTED_BW_INACCURATE
FAIL_DYNAMIC_TRACKING
FAIL_PERSISTENT_SEARCH_DISABLED
FAIL_EXPERIMENT_NOT_IDENTIFIABLE
```

最终必须证明：

```text
1. 单流满载时即使 DRate 周期弱，也能通过 queue-derivative 开窗；
2. 深 queue 通过 RTT 级 servo 连续回到浅队列带；
3. servo 不污染 sustainable baseline 和 trusted_bw；
4. 双通道窗口在欠载、饱和、排队和动态变化中给出正确方向；
5. search baseline 更新降低 fair-share 绝对误差；
6. 长期无 trusted_bw 时仍持续搜索；
7. trusted_bw 最终反映可持续平衡份额，同时 queue 维持最小必要储备。
```
