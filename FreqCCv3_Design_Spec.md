# FreqCCv3 频域检测方案细化与当前结论

更新时间：2026-04-12

## 1. 结论先行

这版文档先给出明确结论：

1. `FreqCCv3` 在线主方案不能依赖 `INT`、链路利用率或瓶颈队列占用，因为互联网里拿不到这些信息。  
   `util_ratio/q_ratio` 只能用于 ns-3 离线实验打标签和验理论，不能进入在线控制逻辑。

2. 你要的核心判据已经可以严格表述为“窗口级”规则，而不是整个 `CRUISE+REFILL` 阶段级规则：  
   对下一阶段的每个检测窗口，只判断两类状态：
   - `NONOVER`：满载以下，包括恰好满载
   - `OVER`：过载

3. 在默认自然 `UP ≈ 0.5 ~ 1.2 RTT` 的工作区间，我已经找到了一个比旧规则更强的**联合频域范式**：  
   - `recvrate` 侧不再只看“单峰接近发送频率”，而是看“是否保留上一 `UP` 发送模板的带内形状”；  
   - `RTT` 侧也不再做 `maxRTT` 时域门控，而是看“是否仍与 `recvrate` 保持频率对齐、且自身主峰没有明显上漂”。  
   这个范式在 `dur1` 的四个 2-flow 标定场景上达到：
   - `neighbor_bal_acc = 0.6771`
   - `hit_nonover_rate = 0.7629`
   - `clean_over_rate = 0.5914`
   明显高于旧基线的 `0.6472`。

4. 但这个新范式**不是对所有实验时长都稳定**。  
   我把同一组阈值固定后，拿去验证更长的强制 `UP` 时长，结果在 `1.5 RTT` 以后快速退化，到 `2 RTT` 以后基本塌成“几乎只会报 `OVER`”。  
   所以它当前只能被表述为：
   - 对**默认短 `UP` 工作区间**有效；
   - 不是“对任意 `UP` 驻留时长都通用”的全局规则。

5. 现阶段最合理的工程结论是：
   - 频域检测不仅值得保留，而且已经出现了一个**可解释、可复现、优于旧基线**的窗口级联合特征范式；
   - 但它现在仍更适合作为“默认短 `UP` 场景下的受保护在线候选信号”与“离线标定工具”；
   - 还不适合作为对所有配置一刀切启用的唯一在线控制依据。

6. 我又单独做了你要求的“换输入信号”A/B：  
   不覆盖原方案，新增了一个并行实验路径，把 `recvrate` 从 `BandwidthLatest()` 切到更原始的 ACK 事件级交付速率样本 `delivery_rate_latest = sample.sample_max_bandwidth`，导出为 `_recvrate_raw.txt`，同时给 `FreqCCv3` 校准场景增加了 `probe_recv_signal_mode=delivery_rate_latest` 开关。  
   结果是：
   - **当前这版 raw delivery-rate 输入没有带来更好的主方案准确率**；
   - 它确实让 `recvrate` 的主峰更接近发送参考频率、模板距离也更小；
   - 但它会明显削弱 `OVER` 窗口和 `NONOVER` 窗口之间的可分性，导致 `clean_over_rate` 大幅下降；
   - 所以在当前实现层级上，**不应直接把在线主输入从 `BandwidthLatest()` 换成这版 raw delivery-rate**。

### 1.1 换成更原始交付速率后，到底有没有用

我用和当前主方案完全相同的固定联合规则做了对照：

- 规则：`recv_template_align_rttupper`
- 固定阈值：`D_recv <= 0.45`，`E_recv >= 0.08`，`A <= 0.45`，`rho_rtt <= 0.60`
- 其他参数：`W_recv = 0.75 RTT`，`W_rtt = 1.5 RTT`，`overlap = 0.9`，`gate = [0.5, 1.3] * f_up`
- 场景：`2-flow calibration`，`bg = 0 / 4 / 8 / 12 Mbps`

结果如下：

| 目标最小 `UP` 时长 | 输入 | `neighbor_bal_acc` | `hit_nonover_rate` | `clean_over_rate` |
| --- | --- | ---: | ---: | ---: |
| `0.5 RTT` | `recvrate = BandwidthLatest()` | `0.7133` | `0.7331` | `0.6935` |
| `0.5 RTT` | `recvrate_raw = delivery_rate_latest` | `0.5816` | `0.7247` | `0.4385` |
| `1.0 RTT` | `recvrate = BandwidthLatest()` | `0.7058` | `0.7338` | `0.6778` |
| `1.0 RTT` | `recvrate_raw = delivery_rate_latest` | `0.6164` | `0.7321` | `0.5008` |
| `1.5 RTT` | `recvrate = BandwidthLatest()` | `0.5907` | `0.3292` | `0.8523` |
| `1.5 RTT` | `recvrate_raw = delivery_rate_latest` | `0.4995` | `0.4083` | `0.5907` |
| `3.0 RTT` | `recvrate = BandwidthLatest()` | `0.4977` | `0.0000` | `0.9953` |
| `3.0 RTT` | `recvrate_raw = delivery_rate_latest` | `0.4955` | `0.0000` | `0.9909` |

这组表说明得很直接：

- 在当前最有价值的短 `UP` 区间（`0.5 ~ 1.0 RTT`），raw 输入**稳定掉点**；
- 主要不是 `hit_nonover_rate` 变差，而是 `clean_over_rate` 明显变差；
- 也就是说，raw 输入让更多 `OVER` 窗口也长得像“命中发送频率”，因此主方案更难把它们剔掉。

更具体地看 `NONOVER` 窗口里的接收频域特征，raw 输入确实更“像发送端”：

- `dur1` 中，`recv_peak_close_sender` 的中位数从 `0.4365` 降到 `0.3738`
- `dur1` 中，`recv_shape_dist` 的中位数从 `0.4874` 降到 `0.3910`

但这恰恰解释了为什么主方案更差：

- 它增强了“有频率”的表现；
- 却没有同步增强“只有 NONOVER 才有这种频率”的区分度；
- 于是 `OVER` 窗口也更容易通过 `recvrate` 侧的筛选，最后拖垮 `clean_over_rate`。

我还用简单基线做了一个旁证。  
在 `dur1` 上，仅用“主频接近经验中心 + RTT below-ratio”这套简单规则时：

- `recvrate`：最佳 `neighbor_bal_acc = 0.5038`
- `recvrate_raw`：最佳 `neighbor_bal_acc = 0.5168`

raw 在这个非常弱的基线上有**很小的提升**，但这个提升远不足以支撑把主方案整体切过去，因为主方案真正关心的是联合判据下的窗口级区分能力，而不是“更容易看到一个频率峰”本身。

所以当前结论是：

- `recvrate_raw` 可以保留为**实验输入**和**离线诊断 trace**；
- 但在线主方案仍应默认使用 `BandwidthLatest()` 路径；
- 如果后续还想继续深挖“更原始的输入”，下一步应该尝试的不是当前这个 ACK 事件级 `sample.sample_max_bandwidth`，而是更底层的逐样本 `sample.bandwidth` 导出与对照。

### 1.2 多窗口计数确认：应该怎么设计，阈值取多少

你这次提的思路是对的，但要先解决一个关键问题：

- 现在 STFT 窗口是高度重叠的，`overlap = 0.9`
- 所以如果直接按“最近 `5 RTT` 里有 `3` 个窗口命中”来数，阈值会虚高地偏松
- 因为一小段局部响应就能贡献很多彼此高度相关的窗口

所以我这次没有直接数原始窗口数，而是先做了一个**重叠去重后的时间支持计数器**。

#### 设计

在保持原窗口级联合规则不变的前提下：

- 基础窗口规则仍然是  
  `D_recv <= 0.45`，`E_recv >= 0.08`，`A <= 0.45`，`rho_rtt <= 0.60`
- 对当前窗口，向前看一个**因果历史窗口** `H RTT`
- 把这段历史按 `B RTT` 划成多个时间 bin
- 如果这些 bin 里至少有 `K` 个 bin 各自出现过至少一个 `NONOVER` 基础命中窗口
- 并且**当前窗口本身也先通过基础规则**
- 才把当前窗口最终确认为 `NONOVER`

也就是：

- 不要求命中窗口连续
- 但要求它们分布在多个独立时间 bin 中
- 从而避免“同一团重叠窗口反复投票”

这是一个在线可实现的**因果确认器**，不是离线回看式修补。

#### 我实际扫过的参数

我对以下组合做了系统扫描：

- `H ∈ {2,3,4,5,6,7,8} RTT`
- `B ∈ {0.5,1.0,1.5,2.0} RTT`
- `K ∈ {1,2,3,4,5,6}`

先在最有价值的短 `UP` 工作区间上搜索，也就是：

- `dur0.5`
- `dur1.0`
- 每个时长下再覆盖 `bg = 0 / 4 / 8 / 12 Mbps`

#### 搜索结果

短 `UP` 联合集合上的基础窗口规则表现为：

- `neighbor_bal_acc = 0.7095`
- `hit_nonover_rate = 0.7335`
- `clean_over_rate = 0.6855`

最优确认器是：

- `H = 2 RTT`
- `B = 0.5 RTT`
- `K = 2`

它把结果提升到：

- `neighbor_bal_acc = 0.7238`
- `hit_nonover_rate = 0.6663`
- `clean_over_rate = 0.7813`
- `confirmed_precision = 0.5034`

这里的变化方向很清楚：

- `hit_nonover_rate` 略降
- 但 `clean_over_rate` 明显上升
- 最终平衡准确率更高

这说明这个确认器本质上是在做一件正确的事：

- 它牺牲一部分激进的 `NONOVER` 命中
- 换来更稳的 `OVER` 剔除能力

#### 稳定性验证

我又分别在 `dur0.5` 和 `dur1.0` 上单独搜索，最优组合仍然都是：

- `H = 2 RTT`
- `B = 0.5 RTT`
- `K = 2`

对应结果：

| 场景 | 基础规则 `neighbor_bal_acc` | 加确认器后 |
| --- | ---: | ---: |
| `dur0.5` | `0.7133` | `0.7311` |
| `dur1.0` | `0.7058` | `0.7167` |

所以这个阈值不是偶然撞出来的，它在默认短 `UP` 工作区间内是稳定的。

#### 你举的例子：`5 RTT` 里至少 `3` 个命中

我也专门测了你举的这个更直观组合：

- `H = 5 RTT`
- `B = 1 RTT`
- `K = 3`

在短 `UP` 联合集合上，它的结果是：

- `neighbor_bal_acc = 0.6902`

明显低于：

- 基础规则的 `0.7095`
- 以及最优确认器的 `0.7238`

原因是它的确认跨度太长了：

- 会把较早的旧命中带得太久
- 同时对当前是否真的还处于 `NONOVER` 变得不够敏感

#### 泛化到长 `UP` 的结果

把最优确认器 `H = 2 RTT, B = 0.5 RTT, K = 2` 固定后：

| 场景 | 基础规则 | 加确认器后 |
| --- | ---: | ---: |
| `dur1.5` | `0.5907` | `0.5528` |
| `dur3.0` | `0.4977` | `0.5000` |

也就是说：

- 它对默认短 `UP` 区间有效
- 对已经失效的长 `UP` 区间没有补救能力
- 甚至会进一步把 `NONOVER` 全部压掉

所以这条确认器的正确定位应该是：

- **作为默认短 `UP` 工作区间里的保守确认层**
- 而不是拿来修复长 `UP` 失效问题的万能补丁

#### 当前建议

如果你要把这个思路放进 `FreqCCv3` 主方案，我现在建议写成：

- 先按窗口级联合规则给出 `base_nonover`
- 再做一个因果确认：
  在最近 `2 RTT` 中，按 `0.5 RTT` 划 bin
- 若至少 `2` 个 bin 出现过 `base_nonover`
- 且当前窗口本身也是 `base_nonover`
- 才输出最终 `confirmed_nonover`

这个版本是我目前通过多轮实验得到的最稳阈值。

### 1.3 RTT 支路改用 `srtt` 后，结果明显更好

你后来明确提到“RTT 支路可以直接试 `srtt`，不要再用原始 RTT 样本”。  
这件事我已经按当前主方案重跑过一遍，结论很直接：

- **在当前保留的主范式里，`srtt` 明显优于原始 `rtt`。**
- 而且这个提升不是只体现在单个阈值点上，而是同时出现在：
  - `dur1` 标定集上的窗口级主范式搜索；
  - `dur0.5 + dur1` 短 `UP` 联合集上的时间确认器搜索。

#### 1.3.1 `dur1` 上的主范式对照

保持当前保留的主范式不变：

- 范式：`recv_template_align_rttupper`
- 前端：`W_recv = 0.75 RTT`，`W_rtt = 1.5 RTT`，`overlap = 0.9`
- 频带：`[0.5, 1.3] * f_up`
- `nfft = 4 * win_len`

只把 RTT 支路输入从原始 `rtt(ms)` 换成 `smoothed_rtt(ms)` 后，`dur1_bg0/bg4/bg8/bg12` 的最优结果变成：

| RTT 输入 | 最优参数 | `neighbor_bal_acc` | `hit_nonover_rate` | `clean_over_rate` |
| --- | --- | ---: | ---: | ---: |
| 原始 `rtt` | `D_recv<=0.45, E_recv>=0.08, A<=0.22, rho_rtt<=0.60` | `0.6634` | `0.7162` | `0.6107` |
| `srtt` | `D_recv<=0.45, E_recv>=0.08, A<=0.22, rho_rtt<=0.60` | **`0.7304`** | `0.6768` | **`0.7840`** |

这里最关键的变化不是“更容易报 `NONOVER`”，而是：

- `clean_over_rate` 从 `0.6107` 抬到 `0.7840`
- 也就是 `srtt` 显著增强了 `OVER` 的可剔除性

换句话说，`srtt` 作为 RTT 侧输入后，RTT 支路变得更像一个**稳定的慢响应约束器**，而不再像原始 RTT 那样被大量瞬时尖峰拖着跑。

#### 1.3.2 短 `UP` 联合集上的时间确认器对照

我又在 `dur0.5 + dur1` 的短 `UP` 联合集上，把当前固定规则

- `D_recv <= 0.45`
- `E_recv >= 0.08`
- `A <= 0.45`
- `rho_rtt <= 0.60`

分别配上：

- 原始 `rtt`
- `srtt`

再做同样的 `H/B/K` 搜索，结果是：

| RTT 输入 | 基础规则 `neighbor_bal_acc` | 最优确认器 | 加确认器后 |
| --- | ---: | --- | ---: |
| 原始 `rtt` | `0.6928` | `H=2 RTT, B=0.5 RTT, K=2` | `0.7106` |
| `srtt` | **`0.7357`** | `H=2 RTT, B=0.5 RTT, K=2` | **`0.7501`** |

对应最优确认器下的细项指标：

| RTT 输入 | `hit_nonover_rate` | `clean_over_rate` |
| --- | ---: | ---: |
| 原始 `rtt` | `0.6988` | `0.7224` |
| `srtt` | `0.6208` | **`0.8795`** |

这说明两件事：

1. 最优时间确认器结构**没变**，仍然是：
   - `H = 2 RTT`
   - `B = 0.5 RTT`
   - `K = 2`
2. 真正变强的是 RTT 支路本身：
   - `srtt` 会牺牲一部分激进的 `NONOVER` 命中；
   - 但会显著提高 `OVER` 的干净剔除；
   - 最后让整体平衡准确率更高。

#### 1.3.3 当前工程结论

所以在当前这套方案里，RTT 支路的默认建议已经可以更新为：

- **在线检测优先使用 `srtt`，而不是原始 RTT 样本。**

原因不是因为 `srtt` 更“平滑所以看起来更好看”，而是因为当前任务里 RTT 支路承担的角色本来就更接近：

- 判断是否还和 `recvrate` 处在同一响应块里；
- 判断是否已经出现上漂；
- 作为一个慢变化、低噪声的辅助约束器。

在这个角色分工下，`srtt` 比原始 RTT 更合适。

---

## 2. 在线可观测量与不可观测量

### 2.1 在线真正可用的量

`FreqCCv3` 在线只能使用以下端到端量：

- 上一个 `UP` 阶段的发送端波动参考频率 `f_up`
- 下一阶段 `INT = CRUISE + REFILL` 中的 `recvrate` 样本
- 下一阶段中的原始 `RTT` 或 `srtt` 样本
- `min_rtt`
- BBRv2/FreqCCv3 当前维护的 `max_bw/max_filter`

### 2.2 只能在实验里使用的 oracle

以下信息只能在 ns-3 离线实验里使用：

- 瓶颈队列占用
- 链路利用率
- 各流队列份额
- 真正的 `UNDER/FULL/OVER` 标签

这些量只能做两件事：

1. 判断你画在图里的理论推导是否成立；
2. 选出最稳的 STFT 参数与阈值。

**它们不能进入互联网在线版本的控制规则。**

---

## 3. 为什么不能做“锁相/单频硬匹配”

你的判断是对的：不能要求接收侧和 RTT 侧的检测频率与发送频率严格相等。

原因有四类。

### 3.1 ACK 时钟与接收样本不是发送波形本身

接收侧看到的是 ACK 驱动后的交付过程，不是发送端理想正弦波本体。  
ACK 聚合、ACK compression、delayed ACK、stretched ACK 都会改变样本时序和峰值位置。

### 3.2 队列与瓶颈链路会改变响应

`UP` 阶段注入的是发送侧速率扰动；到后继 `INT` 阶段时，瓶颈队列会对这个扰动做衰减、积累和相位改写。  
因此后继 `recvrate/RTT` 里出现的，不一定还是“同频、同相、同幅”的干净分量。

### 3.3 短窗口 STFT 自身有分辨率偏差

短窗口越短，时间定位越好，但频率分辨率越差。  
这意味着窗口太短时，主峰容易偏到邻近频点，或者被旁瓣和低频趋势吃掉。

### 3.4 当前实现看的是短时响应，不是 ICC 那种整周期稳定检测

ICC 的周期性判定是“整轮/多轮 RTT”的频谱与趋势联合判定；  
而 `FreqCCv3` 现在试图做的是“前一轮 `UP` 注入，后一轮 `INT` 看响应”的短窗检测，这本来就更难。

所以正确做法不是“锁定某个精确频率”，而是：

- 开放检测；
- 在允许频带内搜峰；
- 命中时使用“最接近参考频率的候选峰”；
- 阈值从宽到严离线扫参。

---

## 4. ICC 和 FreqCCv3 的本质区别

这部分必须说清楚，否则会误导设计。

### 4.1 ICC 不是短窗 STFT 检测器

仓库里的 `ICC` 实现本质上是：

- 取一轮或多轮周期数据；
- 对 `cwnd` 和 `srtt` 做频谱分析；
- 比较两者频谱形状；
- 再结合趋势一致性与跨周期稳定性判定“是否存在可协调周期”。

它不是“每 40ms/80ms 滑一个 STFT 窗，看单窗主峰命不命中”的检测器。

### 4.2 FreqCCv3 当前做的是“前激励、后响应”

`FreqCCv3` 当前问题更难：

- `UP` 阶段持续时间短，通常约一个 RTT；
- 后继 `CRUISE+REFILL` 中才做检测；
- 需要判断前一个 `UP` 的发送波动，是否在下一阶段的 `recvrate/RTT` 中留下了可识别响应。

所以 ICC 只能提供“用频谱结构而不是单个频点”的启发，不能直接照抄。

### 4.3 当前代码里还有一个实现层差异

`FreqCCv3` 默认在线分析使用的是 ACK 驱动下的 `BandwidthLatest()` 序列，而不是 ICC 那种更长周期、结构更稳定的周期量。  
对应代码见：

- `NS3.27/src/dqc/model/thirdparty/congestion/freqccv3_sender.cc`
- `NS3.27/src/dqc/model/dqc_trace.cc`

这意味着：

- 你当前 trace 上的 `recvrate` 本身就更短时、更易受 ACK 动态影响；
- 它能做频域分析，但比 ICC 的输入更脆弱。

这次新增的并行实验版本还做了两件事：

- 额外导出 `_recvrate_raw.txt`，其值来自 ACK 事件级 `delivery_rate_latest = sample.sample_max_bandwidth`
- 给校准场景增加了 `probe_recv_signal_mode=delivery_rate_latest` 开关，用来显式切换 `FreqCCv3` 内部 interval analysis 的输入信号

但实验结果表明：**当前这一级 raw delivery-rate 还不足以替代默认的 `BandwidthLatest()` 主输入。**

---

## 5. FreqCCv3-v3 的在线主方案

下面给出我认为目前最完整、最可落地、并且和你最新要求一致的在线方案。

### 5.1 设计目标

目标不是在线恢复真实队列长度，而是判断：

- 当前检测窗口是否处于 `NONOVER`
- 如果是 `NONOVER`，能否把该窗口的接收速率当成更可信的带宽上界候选

### 5.2 `UP` 阶段：只负责注入激励与缓存参考

在每个有效 `UP` 阶段结束时，缓存以下量：

- `f_up`：该次 `UP` 实际采用的发送波动频率
- `up_start/up_end`
- `sender_rate_up_mean`
- `sender_rate_up_spectrum_template`：仅作调试或增强版检测使用

其中“有效 `UP`”至少满足：

- `UP` 内完成不少于 2 个波动周期
- 没有因早期异常立即截断到无法形成可检测频谱

这和你“不要改波动幅度，波动幅度仍然是 `miu2`”完全一致。  
`UP` 阶段只注入，不在这里做控制决策。

### 5.3 `INT = CRUISE + REFILL`：窗口级检测

将 `INT` 划分为多个重叠窗口，每个窗口独立判断。

#### 5.3.1 `recvrate` 窗口检测

对每个 `recvrate` 窗口，不再只看“主峰是不是贴着 `f_up`”，而是提取一个更完整的带内频谱描述：

1. 把窗口内样本按 `1 ms` 重采样到均匀时间轴；
2. 去均值、乘 Hann 窗；
3. 在 `nfft = 4 * win_len` 下做 FFT；
4. 只在开放频带 `B = [0.5, 1.3] * f_up` 内找候选峰；
5. 若带内有多个局部峰，只保留“幅度不低于带内最大峰 `70%` 且最接近 `f_up`”的那个；
6. 在同一频带上再构造 `24` 维归一化 band-shape，和上一 `UP` 的发送模板做形状距离比较。

最终为每个窗口保留三个 `recvrate` 侧特征：

- `rho_recv = f_recv / f_up`
- `E_recv`：带内能量占比
- `D_recv`：当前 `recvrate` band-shape 到上一 `UP` 发送模板的 `L1` 距离

#### 5.3.2 `RTT` 窗口检测

`RTT` 侧也改成频域检测，但这里**不再强求 RTT 去硬匹配发送模板**。  
离线重做后的结果表明，`RTT` 更像一个“是否上漂、是否和 `recvrate` 失配”的频域信号。

对每个与 `recvrate` 窗口同中心的 `RTT` 窗口：

1. 同样按 `1 ms` 重采样；
2. 同样在 `B = [0.5, 1.3] * f_up` 内取主峰；
3. 只保留

- `rho_rtt = f_rtt / f_up`
- `A = |rho_recv - rho_rtt|`

其中：

- `rho_rtt` 描述 RTT 主峰是否明显上漂；
- `A` 描述 `recvrate` 与 RTT 是否还保持同一响应块里的频率对齐。

### 5.4 窗口标签定义

对每个检测窗口，定义新的联合频域判据：

- `NONOVER` 当且仅当：
  - `D_recv <= theta_shape`
  - `E_recv >= theta_energy`
  - `A <= theta_align`
  - `rho_rtt <= theta_rtt_hi`

- `OVER`：其余所有情况

这里的含义是：

- `recvrate` 必须仍然保留上一 `UP` 激励的带内形状；
- `RTT` 不必和发送模板同形，但必须还和 `recvrate` 处在同一个响应块里；
- 一旦 `RTT` 主峰明显上漂，或 `recvrate/RTT` 失配，就判成 `OVER`。

在当前默认短 `UP` 标定集上的最佳阈值是：

- `theta_shape = 0.45`
- `theta_energy = 0.08`
- `theta_align = 0.45`
- `theta_rtt_hi = 0.60`

这个组合是这次重做后得到的**原创经验规则**，不是直接照搬哪篇论文里的阈值表。  
借鉴自 ICC / 当前 FreqCCv3 代码的地方只有“band-shape 模板比较”这个基本思路；  
真正把它和 `RTT` 的对齐/上漂特征组合起来，是这轮 ns-3 标定里新导出的结果。

### 5.5 离线真值标签必须做“因果配对”

这里要特别强调一个前面容易做错的点：

- 频域检测窗口是在**观测时刻** `W_obs` 上做的；
- 但离线实验里给这个窗口打队列真值标签时，不能直接用 `W_obs` 时刻的队列状态；
- 必须把窗口回投到“这段 `recvrate` 波动真正形成的时段”再配对。

对 `recvrate` 来说，当前更合理的因果锚点不是“包刚到瓶颈入口”的时刻，而是“这批包在接收端形成交付波动”的时刻。  
因此我最后采用的离线真值配对是：

- `t_cause ≈ t_obs - min_rtt / 2`

并做一个宽松但稳健的匹配：

- 用窗口内样本回投后的 `P10 ~ P90` 区间
- 再在两侧各扩 `0.25 * W_recv`

对应脚本实现见：

- `NS3.27/scripts/freqccv3_window_overload_eval.py`
- `NS3.27/scripts/freqccv3_window_dualfreq_eval.py`

我还测试了两种更早的回投方式：

- `arrival`：回投到“到达瓶颈队列入口”的时刻
- `band`：把“到达瓶颈入口”到“接收端形成波动”整段都作为真值窗口

结果是这两种都会把真值窗口回投得过早，导致大量窗口被错误标成 `NONOVER`，判据失真。  
所以当前最可信的离线真值配对是 `delivery`，不是 `arrival/band`。

### 5.5.1 离线评分还要做“邻域命中”而不是死板逐窗对齐

你后面又补充了一点，这个我认为是对的：

- 如果某个理论窗口本来就属于“欠载/满载以下”，那么它前后相邻 `0.5 RTT` 以内的窗口，本质上也可以接受为同一个理论块；
- 在线规则里仍然只有 `NONOVER/OVER` 两类；
- 但离线实验评分时，不能要求“恰好这个窗口自己命中”，而应该允许“邻域内至少有一个窗口命中就算有效”。

因此这轮重跑后，我把窗口实验分成两种口径：

- **严格逐窗口口径**：仍然保留，方便和之前结果对比；
- **邻域命中口径**：这是本轮新的主指标。

对一条流的窗口序列，设容忍半径为：

- `R_match = 0.5 * min_rtt`

则定义：

- `hit_nonover_rate`：
  - 对所有理论 `NONOVER` 窗口，若其中心时刻前后 `R_match` 内存在至少一个被检测成 `NONOVER` 的窗口，则记为命中；
- `clean_over_rate`：
  - 对所有理论 `OVER` 窗口，若其中心时刻前后 `R_match` 内不存在任何被检测成 `NONOVER` 的窗口，则记为干净；
- `neighbor_bal_acc = 0.5 * (hit_nonover_rate + clean_over_rate)`。

这个定义比单纯“逐窗口 accuracy”更符合你最后说的“只要相邻窗口里至少有一个命中就算有效”。

---

## 6. `max_filter` 的具体修正逻辑

你之前担心“上一个阶段满载，下一个阶段过载/欠载，怎么把多个阶段的结果用来校准 `max_filter`”。  
我认为最稳的做法不是看粗粒度阶段标签，而是直接看窗口序列。

### 6.1 基本原则

只有在满足以下条件时，才允许频域检测影响 `max_filter`：

- 本次 `INT` 中存在一段连续 `NONOVER` 窗口；
- 这段窗口的 `recvrate` 模板命中稳定；
- 这段窗口的 `recvrate/RTT` 峰值对齐稳定；
- 这段窗口的 `RTT` 主峰不上漂；
- 该段窗口的平均接收速率本身不剧烈抖动。

否则，频域检测只记录，不控制。

### 6.2 选择哪一段窗口作为修正候选

对一次 `INT` 中的窗口序列，按以下优先级选取候选块。

#### 情况 A：存在 `NONOVER -> OVER` 转移

取**转移前最后一段连续 `NONOVER` 窗口块**。

原因：  
这通常对应“激励后的响应还存在、但再往后队列已继续堆高”的边界点，最接近你想要的“满载但未明显过载”的工作点。

#### 情况 B：整个 `INT` 都是 `NONOVER`

取时间最长的一段 `NONOVER` 窗口块。

#### 情况 C：没有 `NONOVER`

本轮不修正 `max_filter`。

### 6.3 如何从候选块生成带宽修正值

设候选块内每个窗口的平均接收速率为 `r_i`。

定义：

- `bw_resp = trimmed_mean(r_i, trim=20%)`

即去掉最高和最低 20% 后取均值。

这样做是为了：

- 避免单窗尖峰把结果拉高；
- 避免单窗 ACK 压缩把结果拉低。

### 6.4 在线修正规则

若候选块满足以下置信条件：

- 连续 `NONOVER` 窗口数 `>= 2`
- 连续时长 `>= max(W_recv, 0.5 * min_rtt)`
- 候选块内窗口均值的变异系数 `CV <= 0.15`

则：

1. 用 `bw_resp` 作为本轮可信带宽候选；
2. 令

`max_filter = min(max_filter, bw_resp)`

3. 从当前时刻起到下一次有效 `UP` 结束前，停止再用更大的样本去抬高该 `max_filter`。

这是你提出的“如果频域特征和满载以下状态能稳对应上，就直接让对应位置的接收速率均值替代 BBRv2 高估带宽，并冻结 `max_filter` 更新”的具体实现版本。

### 6.5 为什么当前还不能直接启用这条规则

因为它的前提是：

- 后继 `INT` 中必须稳定出现和 `UP` 激励相关的联合频域响应；
- 且这个响应至少要在**默认短 `UP` 工作区间**内跨背景流量重复出现；
- 若 `UP` 被人工拉长到 `1.5 RTT` 以上，还必须重新标定阈值。

而目前 ns-3 实验只证明了前两条的一部分，没有证明“任意 `UP` 时长都成立”。

---

## 7. STFT 参数该怎么定

这部分只讨论你关心的 STFT / band-shape 参数，不改 `miu2` 幅度。

### 7.1 这轮联合频域重做后，最佳配置已经换了

围绕 6 组代表性配置，我把新的联合规则

- `D_recv <= theta_shape`
- `E_recv >= theta_energy`
- `A <= theta_align`
- `rho_rtt <= theta_rtt_hi`

重新做了参数复标定。  
校准集仍然是：

- `dur1_bg0`
- `dur1_bg4`
- `dur1_bg8`
- `dur1_bg12`

比较结果如下：

| 配置 | `W_recv` | `W_rtt` | overlap | search band | `nfft` | 最佳 `neighbor_bal_acc` |
| --- | ---: | ---: | ---: | --- | ---: | ---: |
| `cfg_a` | `0.75 RTT` | `1.5 RTT` | `0.9` | `[0.5, 1.3] * f_up` | `4x` | **0.6771** |
| `cfg_d` | `0.5 RTT` | `1.5 RTT` | `0.9` | `[0.5, 1.3] * f_up` | `4x` | `0.6700` |
| `cfg_e` | `1.0 RTT` | `1.5 RTT` | `0.9` | `[0.5, 1.3] * f_up` | `4x` | `0.6619` |
| `cfg_f` | `0.75 RTT` | `2.0 RTT` | `0.9` | `[0.5, 1.3] * f_up` | `4x` | `0.6506` |
| `cfg_c` | `0.5 RTT` | `2.5 RTT` | `0.9` | `[0.7, 1.3] * f_up` | `4x` | `0.5539` |
| `cfg_b` | `0.75 RTT` | `1.5 RTT` | `0.8` | `[0.7, 1.1] * f_up` | `2x` | `0.5528` |

结论很明确：

- 对新的联合频域范式，**更开放的频带 `[0.5, 1.3] * f_up` 更合适**；
- `RTT` 窗口不宜再拉到 `2.5 RTT`；
- 旧的窄频带 / 低零填充配置会明显损伤新的联合判别力。

### 7.2 当前建议的默认参数

如果目标是服务于这次重做后的联合频域规则，我建议默认使用：

- `W_recv = 0.75 * min_rtt`
- `W_rtt = 1.5 * min_rtt`
- `overlap = 0.9`
- `search_band = [0.5, 1.3] * f_up`
- `nfft = 4 * win_len`
- `uniform_step = 1 ms`
- `shape_bins = 24`
- `local_peak_floor = 0.7 * band_max`

### 7.3 一个可接受的备选配置

如果你更想偏向“少误报、`OVER` 更干净”，可选：

- `W_recv = 0.5 * min_rtt`
- `W_rtt = 1.5 * min_rtt`
- `overlap = 0.9`
- `search_band = [0.5, 1.3] * f_up`
- `nfft = 4 * win_len`

它的最佳点达到：

- `neighbor_bal_acc = 0.6700`
- `hit_nonover_rate = 0.7085`
- `clean_over_rate = 0.6315`

也就是：

- 比主配置少打中一些 `NONOVER`
- 但 `OVER` 会更干净

### 7.4 参数背后的信号处理依据

这次结果和经典频谱估计文献是一致的：

- 窄窗下单 taper STFT 的频率分辨率本来就很差；
- 频带如果开得太窄，会把真实响应簇切掉；
- 但如果只用“软打分”而不加结构性约束，又会迅速塌成误报。

所以当前最合理的折中不是继续缩窄频带，而是：

- 保持开放频带；
- 用 `recvrate` 模板形状约束真响应；
- 再用 `RTT` 的上漂和对齐特征做第二层判别。

---

## 8. 当前实验结果

### 8.1 多场景扫参结果：为什么“接收频率总比发送频率低”

在当前 ns-3 traces 上，最佳参数附近的 `recvrate` 频率比值不是围绕 `1.0`，而是稳定落在 `0.80 ~ 0.85` 左右。

联合扫参得到的各数据集 `recvrate/ref_freq` 中位数为：

- `bg0`: `0.8358`
- `bg4`: `0.8449`
- `bg8`: `0.8418`
- `bg12`: `0.8402`
- `4flow`: `0.8496`
- `8flow`: `0.8414`
- `16flow`: `0.8010`

这说明：

1. 这不是单次噪声，而是**系统性低偏**；
2. 当前 trace 管线下，“接收侧检测频率 ≈ 发送频率”这个假设不成立；
3. 如果在线规则硬把中心锁在 `1.0`，就会天然漏检大量真实窗口。

### 8.1.1 进一步定位：为什么会稳定落在 `0.84` 左右

这次我专门把“为什么 `recvrate` 检测频率总比发送频率低”拆开重查了一遍，结论已经比较清楚：

**主因不是链路真的把 `60Hz` 物理地稳定变成了 `50Hz`，而是当前 `recvrate` trace 的定义本身就会把频谱往低频拉。**

关键事实有三个。

#### 事实 1：当前 `recvrate` trace 不是“接收端原始速率”

当前 trace 管线里写到 `*_recvrate.txt` 的量，其实是：

- ACK 事件触发时的 `BandwidthLatest()`

而不是：

- 接收端原始 goodput
- 也不是单个 ACK 事件的原始 `ack_rate`

对应实现见：

- `NS3.27/src/dqc/model/dqc_sender.cc`
- `NS3.27/src/dqc/model/dqc_trace.cc`

也就是说，名字叫 `recvrate`，但它本质上是：

- **ACK 驱动**
- **发送端观测**
- **经 BBRv2/FreqCCv3 内部采样器处理过的瞬时带宽量**

这和“真正的接收端速率样本”已经不是同一个信号。

#### 事实 2：`BandwidthLatest()` 自带“每 RTT 内 peak-hold”形状

更关键的是，`BandwidthLatest()` 的更新逻辑不是“每次 ACK 到来就直接写当前样本”，而是：

- 在一个 round 内，只要当前样本更大，才更新 `bandwidth_latest_`
- round 结束时，再把它重置成这一轮的 `sample_max_bandwidth`

对应实现见：

- `NS3.27/src/dqc/model/thirdparty/congestion/quic_bbr2_misc.cc`

这等价于对 ACK 驱动带宽样本施加了一个**按 RTT round 分段的 running-max / peak-hold 非线性变换**。

而 `QuicBandwidthSampler` 里单个带宽样本本身又是：

- `sample.bandwidth = min(send_rate, ack_rate)`

对应实现见：

- `NS3.27/src/dqc/model/thirdparty/congestion/quic_bandwidth_sampler.cc`

所以当前 `recvrate` trace 实际经历的是：

1. ACK 时钟重采样  
2. `min(send_rate, ack_rate)` 裁剪  
3. 每 RTT round 的 running-max / peak-hold

这套管线天然就会把原始 `UP` 激励的谱线改形。

#### 事实 3：一个极简合成实验就能复现 `0.84x`

为了确认这个结论不是拍脑袋，我做了一个最小合成实验：

- 输入：理想 `60Hz` 正弦
- 采样步长：按实际 trace 的 ACK 间隔，约 `0.7ms`
- 处理：每 `1 RTT = 40ms` 内做 cumulative max，round 边界重置

结果非常直接：

- 原始纯正弦整段频谱主峰：`1.000x`
- 经过 RTT-round peak-hold 后的整段频谱主峰：`0.833x`

这和实际 ns-3 trace 的结果高度一致：

- `bg8` 场景：发送端 `UP` 主峰 `1.014x`，后继 `INT` 的 `recvrate` 主峰中位数 `0.8396x`
- `bg12` 场景：发送端 `UP` 主峰 `1.031x`，后继 `INT` 的 `recvrate` 主峰中位数 `0.8507x`

而且频谱峰簇形状也非常像。

实际 `bg8` 第一段 `INT` 的前几大峰值是：

- `37.69Hz`
- `50.38Hz`
- `75.38Hz`
- `62.68Hz`

对应的合成 peak-hold 信号前几大峰值是：

- `50.07Hz`
- `35.17Hz`
- `60.01Hz`
- `74.92Hz`

两者都呈现出“参考频率附近的一串离散峰簇”，而不再是单一的 `60Hz` 干净主峰。  
这说明当前看到的低偏，**主要是观测量定义造成的谱形重排**。

这部分诊断脚本已加入仓库：

- `NS3.27/scripts/freqccv3_freq_bias_diagnose.py`

代表性输出保存在：

- `/tmp/freqccv3_updur_20260411/dur1_bg8_freq_bias.json`
- `/tmp/freqccv3_updur_20260411/dur1_bg12_freq_bias.json`

#### 这对方案意味着什么

所以“接收速率样本检测频率比发送频率低很多”的主因，现在可以明确表述为：

1. **不是单纯 STFT 算错了**；
2. **也不主要是链路传播本身把频率物理下移了**；
3. **而是当前被拿来做频域分析的 `recvrate`，本来就不是原始接收速率，而是 ACK 驱动 + running-max 处理后的 `BandwidthLatest()`。**

换句话说：

- 如果继续用 `BandwidthLatest()` 做在线频域输入，就必须接受“参考中心不会稳定在 `1.0`”
- 并且它的谱线上会天然出现 `0.83x` 左右及其附近峰簇
- 这不是简单靠把阈值调严或调松就能消掉的

### 8.1.2 STFT 仍然有次级问题，但不是这次低偏的主因

虽然主因已经找到了，但 STFT 这边确实还有一个**次级放大问题**：

- 当前窗口常取 `0.75 * min_rtt`
- 在本实验里 `min_rtt ≈ 40ms`
- 所以 `W_recv ≈ 30ms`
- 对 `60Hz` 来说只有约 `1.8` 个周期

这会带来两个后果：

1. 频率分辨率很差；
2. 窗口内 dominant-bin 很容易在候选频带里跳到错误频点。

我用纯 `60Hz` 正弦做了对照：

- 整段 FFT 主峰几乎正好在 `1.0`
- 但如果切成 `0.75 * min_rtt` 的短窗，dominant-bin 也会受 `nfft` 和窗长影响而抖动

不过，**对纯正弦来说这个效应通常还不足以单独把中心稳定推到 `0.84`**。  
真正把中心拉到 `0.84` 左右的，还是上面那层 `BandwidthLatest()` 的 peak-hold 非线性。

因此更准确的归因应该是：

- **主因**：`BandwidthLatest()` 观测量定义
- **次因**：短窗 STFT 进一步放大了谱峰错选和不稳定

### 8.2 `UP on/off` 因果试验

为了回答“当前 `UP` 波动到底能不能在后继阶段引起响应”，我做了最小 A/B 试验：

- `on`：`UP` 保留 `miu2` 波动
- `off`：`UP` 幅度设为 0
- 比较 `UP` 内发送端频谱，以及后继 `INT` 中 `recvrate/RTT` 的窄频带能量提升倍数

#### `under` 场景

- `UP` 发送端窄带能量提升：`20.11x`
- 后继 `INT` 中 `recvrate` 窄带能量提升：`0.89x`
- 后继 `INT` 中 RTT 窄带能量提升：`1.03x`

结论：  
发送端注入非常强，但后继阶段没有稳定响应。

#### `full` 场景

- `UP` 发送端窄带能量提升：`19.20x`
- 后继 `INT` 中 `recvrate` 窄带能量提升：`1.00x`
- 后继 `INT` 中 RTT 窄带能量提升：`0.65x`

结论：  
后继 `INT` 仍然没有可用的因果提升。

#### `over` 场景

- `UP` 发送端 `on` 侧窄带分量仍然明显，但 `off` 侧无可比基线，提升倍数不可直接计算
- 后继 `INT` 中 `recvrate` 窄带能量提升：`1.04x`
- 后继 `INT` 中 RTT 窄带能量提升：`1.41x`

结论：  
只有 RTT 侧出现了一定增强，但模式不稳定，不能单独作为在线控制依据。

### 8.3 旧窗口级 `NONOVER/OVER` 规则重跑结果（历史对照）

这一节已经按**两层修正**重跑：

- 第一层：队列真值窗口必须配对到 `delivery` 因果窗口，而不是观测窗口；
- 第二层：离线评分采用你刚刚明确的“邻域命中”口径，即理论 `NONOVER` 窗口前后 `0.5 RTT` 内只要有一个检测窗口命中，就算有效。

在线规则本身没有变，仍然是：

- `NONOVER` 当且仅当
  - `recvrate` 主峰接近前一 `UP` 的参考频率；
  - 且窗口内 `90%` 以上 RTT 样本低于前两个 `INT` 的 `maxRTT`。

这轮我重点比较了三类设置：

- 经验中心：`center = 0.8355`
- 固定中心：`center = 1.0`
- 因果回投窗口：`delivery + (P10~P90)`，以及稍微放宽的 `delivery + (P10~P90) + 0.35 * W_recv`

#### 方案 A：经验中心 `0.8355`

最佳点：

- `search_band = [0.7, 1.3] * f_up`
- `close_tol = 0.10`
- `rtt_below_ratio = 0.9`
- 因果配对：`delivery`
- 回投区间：`P10~P90`
- 两侧扩张：`0.25 * W_recv`
- 邻域命中容忍：`±0.5 RTT`

结果：

- 邻域平衡准确率 `neighbor_bal_acc`：`0.5843`
- 理论 `NONOVER` 窗口命中率 `hit_nonover_rate`：`0.7085`
- 理论 `OVER` 窗口干净率 `clean_over_rate`：`0.4601`
- 预测 `NONOVER` 的理论支撑率 `supported_pred_nonover_rate`：`0.3057`
- 作为对照，严格逐窗平衡准确率：`0.5970`

把回投窗口略放宽到 `0.35 * W_recv` 后，结果几乎不变：

- `neighbor_bal_acc = 0.5842`
- `hit_nonover_rate = 0.7083`
- `clean_over_rate = 0.4602`

也就是说，“稍微放宽前后窗口”**没有带来实质改进**，但至少证明这一结论对小范围放宽是稳定的。

#### 方案 B：固定中心 `1.0`

最佳点：

- `search_band = [0.7, 1.3] * f_up`
- `close_tol = 0.25`
- `rtt_below_ratio = 0.9`
- 因果配对：`delivery`
- 回投区间：`P10~P90`
- 两侧扩张：`0.25 * W_recv`
- 邻域命中容忍：`±0.5 RTT`

结果：

- `neighbor_bal_acc = 0.5798`
- `hit_nonover_rate = 0.7249`
- `clean_over_rate = 0.4346`
- `supported_pred_nonover_rate = 0.3080`
- 严格逐窗平衡准确率：`0.6033`

这个方案的特点是：

- `NONOVER` 邻域命中率略高；
- 但 `OVER` 邻域干净率更低；
- 所以按你新定义的邻域命中口径，整体反而略差于经验中心方案。

#### 方案 C：进一步放宽回投分位区间

我还试了：

- `P05~P95 + 0.35 * W_recv`

结果与上面两种主方案几乎完全一致：

- 经验中心时 `neighbor_bal_acc = 0.5840`
- 固定中心时 `neighbor_bal_acc = 0.5795`

说明当前结果的瓶颈**不在“配对窗口再放宽一点”**，而在检测信号本身仍然不够可分。

### 8.3.1 因果配对模式比较

我额外比较了三种回投方式：

- `arrival`：窗口回投到“到达瓶颈入口”时刻
- `delivery`：窗口回投到“接收端交付波动形成”时刻
- `band`：把 `arrival -> delivery` 整段都作为真值窗口

结果很明确：

- `arrival` 会把窗口回投得过早，`tol=0.25` 时几乎塌成“所有真值都是 `NONOVER`”
- `band` 也会明显过宽，判据失真
- `delivery` 最稳，且与问题定义最一致

所以当前报告的窗口准确率，统一以 `delivery` 因果配对为准。

### 8.4 对这些准确率的解释

如果只看严格逐窗指标，会觉得结果还勉强能看；  
但按你刚修正后的“邻域命中”口径，最好的 `neighbor_bal_acc` 也只有大约 `0.58`。

更关键的是，它其实是下面这个结构：

- 理论 `NONOVER` 邻域命中率可以做到大约 `0.71 ~ 0.72`；
- 但理论 `OVER` 邻域干净率只有大约 `0.43 ~ 0.46`；
- 被算法判成 `NONOVER` 的窗口里，真正落在理论 `NONOVER` 邻域中的只有大约 `0.31`。

也就是说：

- 这条规则**确实能在不少“该出现特征”的地方打中**；
- 但它也会在大量本不该打中的 `OVER` 区域里误触发。

从分场景看，这个问题也很明显：

- `bg0`：几乎所有理论 `NONOVER` 都能命中，但 `OVER` 干净率很差；
- `bg4`：反过来，`OVER` 干净率较高，但 `NONOVER` 命中率很低；
- `bg12`：两边都不理想，是当前最难场景之一。

这意味着：

- 当前这条窗口规则**还没有足够强的判别力**；
- 它可以做“候选信号”或辅助冻结条件；
- 但还不足以直接作为“已经确认满载，因此立刻用该窗口平均接收速率替换 `max_filter`”的强真值。

### 8.5 旧规则下的 `UP` 持续时间 sweep：拉长发送端波动多久才够

你后面又提出一个很关键的问题：

- 会不会不是 `STFT` 参数本身的问题，而是 `UP` 阶段的发送速率波动持续得还不够久；
- 需要先回答：最少要让 `UP` 波动持续多久，后继窗口级 `NONOVER/OVER` 判断才可能达到高可信度。

为此，我在 ns-3 里加了一个**仅用于实验**的最小 `PROBE_UP` 驻留门控：

- 只延迟“因排队而退出 `PROBE_UP`”；
- 不延迟 risky/loss 风格的退出；
- 不改波动幅度，幅度仍然固定为 `miu2`；
- 不把波动延续到 `CRUISE/REFILL`，避免污染观测窗口。

对应代码改动在：

- `NS3.27/src/dqc/model/thirdparty/congestion/quic_bbr2_probe_bw.cc`
- `NS3.27/src/dqc/model/thirdparty/congestion/freqccv3_sender.cc`
- `NS3.27/src/dqc/model/dqc_sender.cc`
- `NS3.27/scratch/freqccv3_2flow_calibration.cc`

批量实验脚本在：

- `NS3.27/scripts/freqccv3_up_duration_sweep.py`

实验配置如下：

- 场景：`2-flow calibration`
- 背景流：`0 / 4 / 8 / 12 Mbps`
- 瓶颈：`20 Mbps, 18 ms`
- `probe_freq_hz = 60`
- `probe_amp_mode = miu2`
- `sim_time = 20 s`
- 评估规则：仍然使用你确认后的窗口规则
  - `recvrate` 主峰在 `[0.7, 1.3] * f_up` 内搜最近峰
  - `close_tol = 0.10`
  - `RTT` 门控为窗口内 `90%` 以上样本小于前两阶段 `maxRTT`
  - 因果配对：`delivery + P10~P90 + 0.25 * W_recv`
  - 邻域命中容忍：`±0.5 RTT`

扫描的目标最小时长为：

- `0, 0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 4.0, 5.0, 6.0 RTT`

#### 8.5.1 结果表

| 目标最小时长 (RTT) | 实际 `UP` 中位数 (RTT) | P10 | P90 | `neighbor_bal_acc` | `hit_nonover_rate` | `clean_over_rate` |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| 0.0 | 1.175 | 1.020 | 1.575 | 0.5837 | 0.7897 | 0.3776 |
| 0.5 | 1.175 | 1.025 | 1.797 | 0.5998 | 0.6015 | 0.5981 |
| 1.0 | 1.175 | 1.025 | 1.538 | **0.6472** | 0.7024 | 0.5921 |
| 1.5 | 1.500 | 1.500 | 1.633 | 0.6222 | 0.6809 | 0.5635 |
| 2.0 | 2.000 | 2.000 | 2.155 | 0.6322 | 0.7149 | 0.5495 |
| 2.5 | 2.500 | 2.500 | 2.500 | 0.5091 | 0.9950 | 0.0233 |
| 3.0 | 3.000 | 2.888 | 3.025 | 0.5078 | 0.9976 | 0.0180 |
| 4.0 | 4.000 | 2.565 | 4.000 | 0.5110 | 0.9960 | 0.0261 |
| 5.0 | 4.650 | 1.837 | 5.000 | 0.5028 | 0.9943 | 0.0113 |
| 6.0 | 3.625 | 1.290 | 5.595 | 0.5069 | 1.0000 | 0.0138 |

原始汇总输出保存在：

- `/tmp/freqccv3_updur_20260411/duration_sweep_summary.json`

#### 8.5.2 该怎么解释

这张表给出的信息非常明确。

第一，**不存在“`UP` 越长，后继窗口检测越准”的单调关系**。  
最好的点出现在目标 `1.0 RTT`，但它的实际 `UP` 中位数只有 `1.175 RTT`，和默认不加门控时几乎一样。

第二，**把 `UP` 拉长到 `1.5 ~ 2.0 RTT` 只能带来有限改进，并没有把准确率推到高可信度区间**。  
最好的邻域平衡准确率也只有：

- `neighbor_bal_acc = 0.6472`

如果把“高可信度”理解为：

- `neighbor_bal_acc >= 0.75`，或者
- `hit_nonover_rate` 和 `clean_over_rate` 同时接近 `0.7 ~ 0.8`

那么这轮 sweep **没有任何一个时长点达标**。

第三，**一旦把 `UP` 强行拉到 `2.5 RTT` 以上，判据会明显塌成“几乎把所有窗口都当成 `NONOVER`”**。  
这时会出现一种很危险的假象：

- `hit_nonover_rate` 接近 `1.0`
- 但 `clean_over_rate` 只剩 `0.01 ~ 0.03`

这不是检测变强了，而是分类器几乎失去对 `OVER` 的排斥能力了。

第四，从实际 `UP` 时长分布也能看出：

- 在 `bg0/bg4` 这类较轻场景里，门控可以比较稳定地把 `UP` 顶到目标时长；
- 但在 `bg8/bg12` 尤其是 `bg12` 下，`5~6 RTT` 目标常常达不到，说明 risky/loss 路径仍会更早终止 `UP`；
- 即使在那些真正达到更长 `UP` 的场景里，窗口级判别力也没有同步提升。

所以这轮实验的结论不是“需要把 `UP` 拉得更长”，而是：

- **当前问题的主瓶颈不是 `UP` 波动时长不够**；
- 默认 `FreqCCv3` 其实已经自然提供了大约 `1.1 ~ 1.2 RTT` 的 `UP` 波动；
- 继续硬拉长 `UP`，不会把后继 `INT` 的频域响应变成高可信检测信号；
- 超过大约 `2 RTT` 后，反而会明显破坏 `OVER` 的可分性。

#### 8.5.3 对主方案的直接影响

因此，`FreqCCv3` 主方案现在应该明确写成：

- 在线版本**不需要**为了频域检测而专门把 `UP` 拉长到 `2.5 RTT` 以上；
- 若保留这个实验门控，也只适合离线标定，不适合进入默认在线逻辑；
- 当前更合理的在线前提时长可以认为是“默认自然 `UP`，约 `1.0 ~ 1.5 RTT`”，而不是越长越好。

---

### 8.6 重做后的联合频域范式：最终结果

这次按你的原始要求，我把“`recvrate` 与 RTT 都走频域特征”这条路完整重做了一遍。  
这一节的结论，**优先级高于 8.3 ~ 8.5 那套旧规则结果**。  
我最终保留下来的，不是“两个主峰都去贴发送频率”，而是下面这个**组合特征范式**：

- `recvrate` 必须保留上一 `UP` 发送模板的带内形状；
- `RTT` 不要求和发送模板同形，但必须和 `recvrate` 仍处在同一响应块里；
- 若 RTT 主峰明显上漂，或者 `recvrate/RTT` 两者失配，就判成 `OVER`。

对应到窗口级规则就是：

- `D_recv <= 0.45`
- `E_recv >= 0.08`
- `A = |rho_recv - rho_rtt| <= 0.45`
- `rho_rtt <= 0.60`

其中：

- `rho_recv = f_recv / f_up`
- `rho_rtt = f_rtt / f_up`
- `D_recv` 是 `recvrate` band-shape 到上一 `UP` 发送模板的 `L1` 距离
- `E_recv` 是 `recvrate` 带内能量占比

#### 8.6.1 在 `dur1` 标定集上的结果

校准集：

- `dur1_bg0`
- `dur1_bg4`
- `dur1_bg8`
- `dur1_bg12`

最优结果：

- 配置：`W_recv = 0.75 RTT`, `W_rtt = 1.5 RTT`, `overlap = 0.9`
- 频带：`[0.5, 1.3] * f_up`
- `nfft = 4 * win_len`
- 阈值：
  - `theta_shape = 0.45`
  - `theta_energy = 0.08`
  - `theta_align = 0.45`
  - `theta_rtt_hi = 0.60`

指标：

- `neighbor_bal_acc = 0.6771`
- `hit_nonover_rate = 0.7629`
- `clean_over_rate = 0.5914`
- `supported_pred_nonover_rate = 0.4249`
- 严格逐窗 `balanced_accuracy = 0.5440`
- 严格逐窗 `accuracy = 0.7374`

相对旧基线：

- 旧最佳窗口规则：`neighbor_bal_acc = 0.6472`
- 新联合频域规则：`neighbor_bal_acc = 0.6771`
- 绝对提升：`+0.0299`

也就是说，在**你真正关心的默认短 `UP` 工作区间**里，这条新规则已经是明确优于旧方案的。

#### 8.6.2 为什么这条规则有效

从特征分布看，新的判别力主要来自三个事实：

1. `NONOVER` 窗口里，`recvrate` 更容易保留上一 `UP` 的发送模板形状；  
   所以 `D_recv` 在 `NONOVER` 上整体更小。

2. `OVER` 窗口里，RTT 主峰更容易往上漂；  
   所以直接约束 `rho_rtt <= 0.60` 比拿 RTT 去硬贴发送模板更有效。

3. `OVER` 窗口里，`recvrate` 与 RTT 更容易失配；  
   所以 `A = |rho_recv - rho_rtt|` 是一个比“两个都接近发送频率”更稳的联合特征。

换句话说：

- `recvrate` 更像“是否还保留激励痕迹”的主检测器；
- RTT 更像“是否已经上漂 / 是否还和 `recvrate` 同步”的辅助检测器。

这就是为什么“`recv-template + RTT对齐/上漂`”优于“`recv峰值 + RTT峰值都贴发送频率`”。

#### 8.6.3 软打分为什么反而不如硬规则

我还单独比较了硬规则和软打分：

- 最优硬规则：`neighbor_bal_acc = 0.6771`
- 最优软打分：`neighbor_bal_acc = 0.5024`

软打分的典型塌缩模式是：

- `hit_nonover_rate` 很高
- 但 `clean_over_rate` 接近 0

也就是它更容易把大量 `OVER` 也打成 `NONOVER`。  
因此这次我不建议主方案走“纯权重打分”，而建议保留结构化硬约束。

#### 8.6.4 这条规则的泛化边界

我把上面这组**固定阈值**直接拿去验证不同 `UP` 驻留时长，结果如下：

| `UP` 目标时长 | `neighbor_bal_acc` | `hit_nonover_rate` | `clean_over_rate` |
| --- | ---: | ---: | ---: |
| `0.5 RTT` | `0.7083` | `0.7709` | `0.6457` |
| `1.0 RTT` | `0.6771` | `0.7629` | `0.5914` |
| `1.5 RTT` | `0.5723` | `0.3052` | `0.8394` |
| `2.0 RTT` | `0.5023` | `0.0046` | `1.0000` |
| `2.5 RTT` | `0.5000` | `0.0000` | `1.0000` |
| `3.0 RTT` | `0.4994` | `0.0000` | `0.9988` |
| `4.0 RTT` | `0.4962` | `0.0005` | `0.9918` |
| `5.0 RTT` | `0.4854` | `0.0002` | `0.9706` |
| `6.0 RTT` | `0.4810` | `0.0020` | `0.9600` |

所有时长合并后的总结果是：

- `neighbor_bal_acc = 0.5335`
- `hit_nonover_rate = 0.1832`
- `clean_over_rate = 0.8838`

这个表说明两件事：

1. 新规则**确实适配默认短 `UP`**，尤其是 `0.5 ~ 1.0 RTT`；
2. 它**不适配被人工拉长到 `1.5 RTT` 以上的 `UP`**。

所以当前最准确的工程表述是：

- 这是一条**短 `UP` 工作区间专用**的联合频域规则；
- 它不是“只要有 `UP` 激励就永远通用”的全局规则。

#### 8.6.5 这条规则的原创性从哪里来

这部分你前面专门问过，我这里明确回答：

- 不是从某个现成论文里抄了一张“阶段转移表”；
- 也不是从 ICC 里直接照搬阈值。

真正来自已有工作的只有两点启发：

- 用 band-shape 而不是只看单峰；
- 频域检测必须结合端到端可观测量，而不是假设能拿到队列真值。

而下面这些内容，是这轮 ns-3 重做里新导出的：

- `recvrate` 用发送模板匹配
- RTT 不再做 `maxRTT` 门控，而改成“上漂 + 对齐”双特征
- `rho_rtt <= 0.60` 这个经验上界
- `A <= 0.45` 这个跨信号对齐约束

所以这条最终规则本身是**实验归纳出来的原创组合特征范式**。

代表性输出文件：

- `/tmp/freqccv3_dualfreq_dur1_align.json`
- `/tmp/freqccv3_dualfreq_dur1_core.json`

#### 8.6.6 RTT 输入换成 `srtt` 的复跑结果

在不改 `recvrate` 支路、不改发送端 `UP` 调制、不改 STFT 前端配置的前提下，  
我把 RTT 支路输入从原始 `rtt(ms)` 换成了 `_rtt.txt` 第 4 列的 `smoothed_rtt(ms)`，并复跑了当前保留的主范式 `recv_template_align_rttupper`。

`dur1_bg0/bg4/bg8/bg12` 上，最优点如下：

| RTT 输入 | 最优参数 | `neighbor_bal_acc` | `hit_nonover_rate` | `clean_over_rate` |
| --- | --- | ---: | ---: | ---: |
| 原始 `rtt` | `D_recv<=0.45, E_recv>=0.08, A<=0.22, rho_rtt<=0.60` | `0.6634` | `0.7162` | `0.6107` |
| `srtt` | `D_recv<=0.45, E_recv>=0.08, A<=0.22, rho_rtt<=0.60` | **`0.7304`** | `0.6768` | **`0.7840`** |

也就是说：

- 最优阈值组合没有变；
- 但同一组合在 `srtt` 上显著更强；
- 提升主要来自 `OVER` 剔除能力，而不是更激进地报 `NONOVER`。

我又把同样的 RTT 输入切换带到短 `UP` 联合集的时间确认器里，得到：

| RTT 输入 | 基础规则 | 最优确认器后 |
| --- | ---: | ---: |
| 原始 `rtt` | `0.6928` | `0.7106` |
| `srtt` | **`0.7357`** | **`0.7501`** |

而且最优确认器仍然保持不变：

- `H = 2 RTT`
- `B = 0.5 RTT`
- `K = 2`

所以当前最合理的更新不是“改时间确认器结构”，而是：

- **保留当前 `H/B/K`**
- **把 RTT 支路默认输入切到 `srtt`**

这轮复跑的结果文件保存在：

- `/tmp/freqccv3_srtt_20260413/dur1_alignonly_rtt.json`
- `/tmp/freqccv3_srtt_20260413/dur1_alignonly_srtt.json`
- `/tmp/freqccv3_srtt_20260413/temporal_shortup_rtt.json`
- `/tmp/freqccv3_srtt_20260413/temporal_shortup_srtt.json`

## 9. 为什么现在不应该强行让代码输出更多 `recvfreq/rttfreq`

这个问题现在已经清楚了。

### 9.1 不是单纯门限太严

如果只是门限太严，那么 A/B 因果试验应该能看到：

- `UP on` 比 `UP off`
- 在后继 `INT` 的 `recvrate/RTT` 里有明显能量提升

但当前大多数场景里看不到这个提升。

### 9.2 强行放宽只会让 trace 文件“看起来有东西”

如果继续放宽接受条件，确实可能重新产生 `recvfreq/rttfreq` 文件，  
但那更可能只是把噪声窗也放进来，而不是解决“后继阶段是否真实有响应”这个核心问题。

### 9.3 现在更合理的做法

当前更合理的顺序是：

1. 先把离线因果性确认做扎实；
2. 先回答“后继 `INT` 是否真的含有稳定响应”；
3. 只有这个问题回答为“是”，才值得继续调在线接受门限。

---

## 10. 后续建议：接下来应该怎么做

### 10.1 第一优先级

把新的联合频域规则只放在**默认短 `UP`** 的真实工作区间内继续细化：

- 固定 `miu2`
- 固定 `f_up`
- 保持默认自然 `UP`，不要再强行把 `UP` 拉到 `2 RTT` 以上
- 继续只改背景流，把 `under/full/over` 做干净
- 对比 `UP on/off`
- 看后继 `INT` 的 `recvrate-template`、`RTT` 上漂与 `recv/rtt` 对齐是否稳定出现

如果这一步都过不了，就说明“前激励、后响应”链条本身还没有建立起来。

### 10.2 第二优先级

如果你想把这条规则从“短 `UP` 特化版”继续推成更普适的版本，优先排查这四件事：

- `RTT` 侧是否应该从单 taper FFT 升级成 multitaper
- `recvrate` 侧是否应该从“单窗 band-shape”升级成“连续窗 band-shape 轨迹”
- `RTT` 主峰上漂阈值是否应改成和 `UP` 实际驻留时长相关的自适应阈值
- 是否应引入 reassignment / synchrosqueezing 之类更适合短窗弱谱线的时频表示

### 10.3 第三优先级

若后续继续推进在线修正，建议按下面的顺序上车：

- 第一步：只在默认短 `UP` 场景下，把这条联合规则作为 `max_filter` 冻结条件
- 第二步：只用连续 `NONOVER` 窗口块的 `trimmed_mean(recvrate)` 生成候选值
- 第三步：再从“冻结”升级到 `min(max_filter, bw_resp)` 的硬钳制

---

## 11. 我建议的最终版本表述

如果你要把 `FreqCCv3` 写成论文或设计文档中的正式方案，我建议把主张写成下面这句，而不是更激进的版本：

> `FreqCCv3` 在 `UP` 阶段注入短时频率激励，并缓存该次 `UP` 的发送带内频谱模板；在下一 `INT` 阶段，对 `recvrate` 与 RTT 做窗口级联合频域检测。当检测窗口同时满足“接收速率 band-shape 与上一 `UP` 发送模板足够接近”“RTT 主峰未明显上漂”以及“`recvrate` 与 RTT 的主峰仍保持频率对齐”时，将该窗口视为 `NONOVER` 候选，并用其接收速率均值作为带宽上界修正候选。该规则当前仅对默认短 `UP` 工作区间完成了离线校准，不应直接外推到任意 `UP` 时长。`

这句话是严谨的，因为它既保留了这次重做后已经证实的强项，也把适用边界说清楚了。

---

## 12. 参考资料

下面这些资料对这个方案最有参考价值：

1. ICC / 当前代码里“band-shape 模板比较”的直接来源  
   - EuroSys 2025: *Introspective Congestion Control for Consistent High Performance*  
   - 当前仓库实现：`NS3.27/src/dqc/model/thirdparty/congestion/freqccv3_sender.cc`

2. BBR 的设计背景  
   - *BBR: Congestion-Based Congestion Control*（ACM Queue）

3. 端到端主动探测的可行性  
   - *End-to-end Available Bandwidth Estimation Using Packet Chirps*  
   - *CapProbe: A Simple and Accurate Capacity Estimation Technique*

4. 短窗频谱估计的经典限制  
   - Fredric J. Harris, 1978, *On the Use of Windows for Harmonic Analysis with the Discrete Fourier Transform*  
   - D. J. Thomson, 1982, *Spectrum Estimation and Harmonic Analysis*

5. 适合弱谱线和短窗场景的改进方向  
   - F. Auger, P. Flandrin, 1995, *Improving the Readability of Time-Frequency and Time-Scale Representations by the Reassignment Method*

这些资料共同支持三点：

- 端到端主动注入结构化探测是合理方向；
- 单纯短窗 STFT + 单峰命中，天然会受时频分辨率和谱泄漏限制；
- 因此“模板 + 对齐 + 上漂”这类结构化组合特征，比单阈值单峰规则更合理。

---

## 13. 本次结论的落地版本

因此，我建议你把 `FreqCCv3` 的当前状态定为：

- **方案层面：已经从旧的 `recv + maxRTT` 门控，升级成新的联合频域规则**
- **实验层面：已经证明它在默认短 `UP` 区间优于旧基线**
- **边界层面：还没证明它能跨任意 `UP` 时长稳定成立**
- **在线控制层面：可以进入“受保护候选规则”，但还不该无条件直接改 `max_filter`**

如果后面你要继续推进，我建议直接做两件事：

1. 先把这条联合频域规则接进 `max_filter` 冻结逻辑，但只在默认短 `UP` 条件下启用；  
2. 再专门研究“如何把它从 `0.5 ~ 1 RTT` 的有效区间，扩展到更长 `UP` 而不塌掉”。

---

## 14. `UP` 调制与 `DOWN` 调制的直接对照（2026-04-21）

这一步专门回答一个更窄的问题：

- **如果把离线实验里的发送端频率调制，从 `probeBW_up` 改到 `probeBW_down`，在不改判别规则、不改阈值、不改时间确认器的前提下，负载状态分辨准确率会不会更高？**

### 14.1 对照原则

这次对照里，除了“调制发生在哪个 BBRv2 子阶段”之外，其余条件全部固定：

- 场景：2-flow `manual` 标定场景
- 瓶颈：`20 Mbps`, `18 ms`
- 调制频率：`60 Hz`
- 调制幅度：`miu2`
- 背景流：`0 / 4 / 8 / 12 Mbps`
- `UP` 最小时长门限：`0.5 RTT` 与 `1.0 RTT`
- 汇总数据集：`dur0.5 + dur1`
- `recv` 输入：`recvrate`
- RTT 输入：`srtt`
- 前端参数：`W_recv = 0.75 RTT`, `W_rtt = 1.5 RTT`, `overlap = 0.9`
- 频带门：`[0.5, 1.3] * f_ref`
- `nfft = 4 * win_len`
- 固定窗口规则：
  - `D_recv <= 0.45`
  - `E_recv >= 0.08`
  - `A <= 0.45`
  - `rho_rtt <= 0.60`
- 固定时间确认器：
  - `H = 2 RTT`
  - `B = 0.5 RTT`
  - `K = 2`

也就是说，这里比较的是：

- `UP` 方案：调制打在 `probeBW_up`
- `DOWN` 方案：调制打在 `probeBW_down`

而不是重新给 `DOWN` 单独调一套更有利的规则。

### 14.2 `DOWN` 调制是否真的打在 `probeBW_down`

我先做了一个 `full` 场景冒烟，确认改动不是“参数传到了日志里，但真实发送波动还在 `UP`”。

冒烟目录：

- `/tmp/freqccv3_down_smoke`

关键信号对齐结果是：

- `bbrmode` 中：
  - `3.20803  probeBW_up`
  - `3.27186  probeBW_down`
  - `5.74987  probeBW_up`
  - `5.79883  probeBW_down`
- `sendrate` 中：
  - `probeBW_up` 起点只抬高到该轮 `UP` 的发送基线；
  - 真正的周期性波动从 `3.27186 s`、`5.79883 s` 这两个 `probeBW_down` 起点开始出现。

例如在 `3.24 ~ 3.31 s` 这一段里：

- `3.27186 s` 之前，`sendrate` 基本保持在 `18725 kbps`；
- `3.27186 s` 之后，出现明显的周期波动：
  - `13032 -> 14830 -> 16628 -> 18425 -> ... -> 4044 -> 7040 -> 10636 ...`

因此可以确认：

- **当前实现里，`DOWN` 版调制已经真实落在 `probeBW_down`，不是名义上的切换。**

需要注意的是：

- `*_upphase.txt` 仍然记录上一轮 `UP` 段信息；
- 它不能再被当成 `DOWN` 调制的相位标记；
- `DOWN` 评估必须改从 `*_bbrmode.txt` 中提取 `probeBW_down` 区间。

### 14.3 数据目录与评估输出

`DOWN` 数据目录：

- `/tmp/freqccv3_downphase_20260421/dur0p5_bg0`
- `/tmp/freqccv3_downphase_20260421/dur0p5_bg4`
- `/tmp/freqccv3_downphase_20260421/dur0p5_bg8`
- `/tmp/freqccv3_downphase_20260421/dur0p5_bg12`
- `/tmp/freqccv3_downphase_20260421/dur1_bg0`
- `/tmp/freqccv3_downphase_20260421/dur1_bg4`
- `/tmp/freqccv3_downphase_20260421/dur1_bg8`
- `/tmp/freqccv3_downphase_20260421/dur1_bg12`

固定规则下的 `DOWN` 评估输出：

- `/tmp/freqccv3_downphase_20260421/temporal_shortdown_srtt_fixed.json`

为避免口径争议，我又用同一套脚本、同一组固定参数，把旧 `UP` 数据重算了一遍：

- `/tmp/freqccv3_downphase_20260421/temporal_shortup_srtt_fixed_recalc.json`

### 14.4 结果

固定窗口规则、不加时间确认器时：

| 调制相位 | `neighbor_bal_acc` | `hit_nonover_rate` | `clean_over_rate` |
| --- | ---: | ---: | ---: |
| `UP` | **`0.7357`** | **`0.6809`** | **`0.7905`** |
| `DOWN` | `0.6914` | `0.6761` | `0.7067` |

固定窗口规则 + 固定时间确认器 `H=2, B=0.5, K=2` 后：

| 调制相位 | `neighbor_bal_acc` | `hit_nonover_rate` | `clean_over_rate` | `confirmed_precision` |
| --- | ---: | ---: | ---: | ---: |
| `UP` | **`0.7501`** | **`0.6208`** | **`0.8795`** | **`0.6845`** |
| `DOWN` | `0.7128` | `0.6137` | `0.8119` | `0.6092` |

如果看相对下降幅度：

- 基础规则：`0.7357 -> 0.6914`，下降 `4.43` 个百分点
- 加确认器后：`0.7501 -> 0.7128`，下降 `3.73` 个百分点
- `clean_over_rate`：`0.8795 -> 0.8119`，下降 `6.76` 个百分点
- `confirmed_precision`：`0.6845 -> 0.6092`，下降 `7.53` 个百分点

### 14.5 这一步的结论

结论很直接：

- **把调制从 `probeBW_up` 改到 `probeBW_down`，在当前这套联合频域判定范式下，并没有提升负载状态分辨准确率，反而整体更差。**

因此，至少在当前版本里：

- 离线实验主线仍应保持 **`UP` 调制**；
- `DOWN` 调制可以保留为对照分支，但不应替代当前主方案。

这说明当前方案更依赖这样的链条：

- 先在 `UP` 中打激励；
- 再在后继窗口里观察 `recvrate/srtt` 的响应；
- 用这组后继响应去判断 `NONOVER/OVER`。

把激励改到 `DOWN` 后，这条链条并没有变得更清晰，反而削弱了后续窗口里的可分性。

---

## 15. 保持 `UP` 调制，但改成 “delivery-rate 频率 + min-SRTT” 联合判定（2026-04-21）

这一版回到你指定的主前提：

- **仍然使用 `probeBW_up` 阶段的发送速率波动**
- **不再使用 `DOWN` 调制**

但窗口级判定规则改成你这次给定的形式。

### 15.1 判定规则

我按你后续澄清后的定义实现为：

- `probRTT` 在代入判定式时，直接取 `min_rtt`
- `max_rtt` 取最近两个 `ProbeBW` 周期里的区间最大 RTT，机制上等价于一个“最近两轮窗口”的 `max-filter`

于是窗口 `w` 被判成 `NONOVER` 的条件是：

1. 该窗口里的 `srtt` 最小值满足：

   `min(srtt in w) < min_rtt + (recent2_max_rtt - min_rtt) / 2`

2. 该窗口里的 `delivery rate` 主频满足：

   `0.75 * f_send < f_delivery < 1.5 * f_send`

其中：

- `f_send` 来自前一 `UP` 阶段发送速率波动的实际主频；
- `f_delivery` 来自当前检测窗口里的 `delivery-rate` 频谱主峰；
- 不满足以上两个条件时，**不判定**，即保持 `abstain`。

### 15.2 这次采用的准确率定义

这次不再看 `balanced accuracy`，而只按你要求的口径统计：

- 只有当算法把一个窗口判成 `NONOVER` 时，这个窗口才进入准确率统计；
- 如果该窗口真实也是 `NONOVER`，准确率不受损；
- 只有当该窗口真实其实是 `OVER` 时，准确率才下降。

所以本节主指标定义为：

- `positive_precision = TP / predicted_nonover_total`

也就是：

- **被判成 `NONOVER` 的窗口里，有多少比例确实不是过载。**

这个指标下：

- `abstain` 不扣分；
- 因此它必须和 `prediction_rate` 一起看，否则可能出现“精度很高，但几乎不报”的情况。

### 15.3 实验设置

为保证口径干净，我重新生成了一套新的 `UP` 数据，而不是复用旧目录：

- `/tmp/freqccv3_uprule_20260421/dur0p5_bg0`
- `/tmp/freqccv3_uprule_20260421/dur0p5_bg4`
- `/tmp/freqccv3_uprule_20260421/dur0p5_bg8`
- `/tmp/freqccv3_uprule_20260421/dur0p5_bg12`
- `/tmp/freqccv3_uprule_20260421/dur1_bg0`
- `/tmp/freqccv3_uprule_20260421/dur1_bg4`
- `/tmp/freqccv3_uprule_20260421/dur1_bg8`
- `/tmp/freqccv3_uprule_20260421/dur1_bg12`

统一设置：

- 调制相位：`UP`
- 调制频率：`60 Hz`
- 调制幅度：`miu2`
- `UP` 时长：`0.5 RTT` 与 `1.0 RTT`
- 背景流：`0 / 4 / 8 / 12 Mbps`
- RTT 输入：`srtt`
- 检测窗口：`W_delivery = 0.75 RTT`
- SRTT 对齐窗口：`W_srtt = 1.5 RTT`
- 频率判定门限：`[0.75, 1.5] * f_send`
- SRTT 门限比例：`0.5`

新增评估脚本：

- `NS3.27/scripts/freqccv3_window_up_delivery_srttmin_eval.py`

输出结果：

- 原始 `delivery rate` 口径：`/tmp/freqccv3_uprule_20260421/up_delivery_srttmin_recvrate_raw.json`
- 平滑 `recvrate` 对照口径：`/tmp/freqccv3_uprule_20260421/up_delivery_srttmin_recvrate.json`

### 15.4 主结果：按字面使用 `delivery-rate`，即 `recvrate_raw`

当 `delivery-rate` 侧输入按字面采用 `recvrate_raw` 时，整体结果是：

| 输入 | `positive_precision` | `false_positive_rate` | `prediction_rate` | `pred_nonover_total` |
| --- | ---: | ---: | ---: | ---: |
| `recvrate_raw + srtt` | **`1.0000`** | **`0.0000`** | `0.0021` | `84` |

也就是说：

- 被它判成 `NONOVER` 的 84 个窗口，**全部都是真的 `NONOVER`**
- 没有一个窗口误判到 `OVER`

但代价也非常明显：

- 总窗口数是 `40086`
- 它只对其中 `84` 个窗口给出了 `NONOVER` 判断
- 覆盖率只有 `0.21%`

换句话说，这条规则在你指定的“只惩罚误判过载”口径下，当前结果是：

- **精度满分**
- **但极其保守**

按分场景展开，只有这些场景出现了正判：

| 场景 | `pred_nonover_total` | `positive_precision` |
| --- | ---: | ---: |
| `dur0p5_bg0` | `15` | `1.0` |
| `dur0p5_bg4` | `14` | `1.0` |
| `dur0p5_bg8` | `9` | `1.0` |
| `dur1_bg0` | `19` | `1.0` |
| `dur1_bg4` | `10` | `1.0` |
| `dur1_bg8` | `17` | `1.0` |

而：

- `dur0p5_bg12`
- `dur1_bg12`

这两个更重负载场景里，这条规则一次 `NONOVER` 都没有报出来。

### 15.5 对照：如果把 delivery 侧改回 `recvrate`

我又额外跑了一个对照，把 delivery 侧输入从 `recvrate_raw` 换成 `recvrate`，其余规则完全不变。

结果是：

| 输入 | `positive_precision` | `false_positive_rate` | `prediction_rate` | `pred_nonover_total` |
| --- | ---: | ---: | ---: | ---: |
| `recvrate + srtt` | **`1.0000`** | **`0.0000`** | `0.0001` | `5` |

也就是说：

- 用 `recvrate` 时，这条规则变得更保守；
- 仍然没有误判；
- 但只剩 `5` 个正判窗口。

因此，如果严格按你这次写的“delivery rate”去理解，那么：

- **`recvrate_raw` 比 `recvrate` 更符合你的字面要求**
- 并且它至少还能给出 `84` 个有效 `NONOVER` 窗口
- `recvrate` 在这条规则下几乎退化成“几乎永不触发”

### 15.6 这一步的结论

这版规则的结论要分成两层看：

第一层，只按你指定的新准确率定义：

- **这版规则当前是成立的**
- 因为它报出来的 `NONOVER` 窗口，没有误判到 `OVER`

第二层，如果看“能不能实际稳定给出足够多的有效窗口”：

- **它现在过于保守**
- 主要问题不是“报了很多但报错”，而是“几乎不报”

所以更准确的工程表述应该是：

- 这条 `UP + delivery-rate frequency + min-SRTT` 规则，当前已经具备**高置信低误报**特性；
- 但它还没有具备足够的**触发覆盖率**，暂时更适合作为一个“高置信候选门”，而不是主判定器。

### 15.7 在“不允许误判过载”的前提下继续调参

既然这版规则的主要矛盾不是“误判太多”，而是“几乎不报”，那后续调参目标就应该改成：

- **约束 `fp_over = 0` 不变**
- **在这个约束下，让 `pred_nonover_total` 尽量大**

也就是：

- 先保证“只要报 `NONOVER`，就别报错”
- 再在这个前提下，尽量提高覆盖率

#### 15.7.1 粗扫：`freq gate` 与 `srtt` 门限比例

我先在 `recvrate_raw + srtt` 口径下做了一个小范围 sweep：

- `freq_ratio_low ∈ {0.65, 0.70, 0.75}`
- `freq_ratio_high ∈ {1.50, 1.60, 1.70}`
- `srtt_threshold_frac ∈ {0.50, 0.60, 0.75, 1.00}`

输出文件：

- `/tmp/freqccv3_uprule_20260421/up_delivery_srttmin_sweep_recvrate_raw.json`

粗扫的核心现象非常清楚：

1. **在这组范围里，频率门限几乎不敏感。**

   例如：

   - `0.65 ~ 1.50`
   - `0.70 ~ 1.60`
   - `0.75 ~ 1.70`

   只要 `srtt_threshold_frac` 固定，`pred_nonover_total` 基本是同一量级，差异很小。

2. **真正决定覆盖率的是 `srtt_threshold_frac`。**

   在粗扫里：

   - `0.50` 时：大约 `389` 个正判，`fp_over = 0`
   - `0.60` 时：大约 `1880` 个正判，`fp_over = 0`
   - `0.75` 时：大约 `5300` 个正判，但已经开始出现 `fp_over = 8`
   - `1.00` 时：几乎全部窗口都被放出来，但误判过载已经失控

所以粗扫直接说明：

- 频率门限不是当前主瓶颈；
- `srtt` 门限比例才是这条规则的真正“控制旋钮”。

#### 15.7.2 细扫：固定原始频率门限 `0.75 ~ 1.50`

既然频率门限不敏感，我就把它固定回你最初给的原始形式：

- `0.75 * f_send < f_delivery < 1.5 * f_send`

然后只细扫：

- `srtt_threshold_frac ∈ {0.60, 0.62, 0.64, 0.66, 0.68, 0.70, 0.72, 0.74}`

输出文件：

- `/tmp/freqccv3_uprule_20260421/up_delivery_srttmin_refine_075_150.json`

细扫结果如下：

| `srtt_threshold_frac` | `pred_nonover_total` | `fp_over` | `positive_precision` |
| --- | ---: | ---: | ---: |
| `0.60` | `1880` | `0` | `1.0000` |
| `0.62` | `2118` | `0` | `1.0000` |
| `0.64` | `2653` | `0` | `1.0000` |
| `0.66` | `2901` | `0` | `1.0000` |
| `0.68` | `3478` | `0` | `1.0000` |
| `0.70` | **`4124`** | **`0`** | **`1.0000`** |
| `0.72` | `4384` | `2` | `0.9995` |
| `0.74` | `5023` | `7` | `0.9986` |

这说明：

- 在原始频率门限不变的前提下，`srtt_threshold_frac = 0.70` 是当前观测到的**零误判上界附近**；
- 再往上放到 `0.72`，虽然覆盖率还会继续增大，但已经开始把 `OVER` 窗口错放进来。

所以，如果你的优先级是：

- **绝对不想把过载误判成非过载**

那么当前最合适的推荐值就是：

- **频率门限仍用 `0.75 ~ 1.50`**
- **`srtt_threshold_frac` 从 `0.50` 提高到 `0.70`**

#### 15.7.3 推荐配置的收益

把这条规则从你原始给定的：

- `srtt_threshold_frac = 0.50`

调到当前推荐的：

- `srtt_threshold_frac = 0.70`

在保持：

- `fp_over = 0`
- `positive_precision = 1.0000`

不变的前提下，`pred_nonover_total` 从：

- `84`

提升到：

- **`4124`**

也就是：

- **提高了约 `49x`**

对应整体覆盖率从：

- `0.21%`

提高到：

- **`10.29%`**

分场景拆开看，推荐配置 `0.75 ~ 1.50 + srtt_threshold_frac = 0.70` 的结果是：

| 场景 | `pred_nonover_total` | `prediction_rate` | `fp_over` |
| --- | ---: | ---: | ---: |
| `dur0p5_bg0` | `1250` | `27.47%` | `0` |
| `dur0p5_bg4` | `473` | `9.59%` | `0` |
| `dur0p5_bg8` | `202` | `5.07%` | `0` |
| `dur0p5_bg12` | `101` | `1.58%` | `0` |
| `dur1_bg0` | `1271` | `30.08%` | `0` |
| `dur1_bg4` | `434` | `8.71%` | `0` |
| `dur1_bg8` | `286` | `6.39%` | `0` |
| `dur1_bg12` | `107` | `1.64%` | `0` |

可以看出：

- 低背景负载场景里，这条规则已经能给出相对可观的 `NONOVER` 窗口；
- 背景越重，覆盖率还是会显著下降；
- 但至少它已经不再是“几乎永不触发”的状态。

#### 15.7.4 当前建议

因此，这一版规则目前最合理的工程建议是：

1. 主结构保持不变：
   - `UP` 调制
   - `delivery-rate` 主频判定
   - `window_min_srtt` 门限

2. 在线门限优先采用：
   - `0.75 * f_send < f_delivery < 1.50 * f_send`
   - `window_min_srtt < min_rtt + 0.70 * (recent2_max_rtt - min_rtt)`

3. 这条规则仍应定位为：
   - **高置信 `NONOVER` 候选门**
   - 而不是单独承担全部状态识别任务的主判定器
