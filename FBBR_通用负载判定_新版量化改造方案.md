# FBBR 通用负载判定新版量化改造方案

> 依据：仓库根目录 `ghtjykukli.pdf`（2026-07-21 13:07 更新版；1 页；SHA-256 `352a930f3e2bd3c2490ad3162ed7f718dc5f4c7a0a18e44ada9905504f5ce2f0`；替代 12:14 版本和此前 `regime判定.pdf`）
>
> 对照基线：当前工作区中的 `kFBBR` / `time_waveform` 路径
>
> 本文目标：把 PDF 的判定图完整翻译为有固定优先级、可编码、可测试、可追踪的规则；本文不包含代码修改。
>
> **范围硬约束：本文把更新版 PDF 的 18 个确定性叶子编号为 N01-N18；这些判定以及“判定后调整注入基线/ProbedBw”的整条控制链只属于 `kFBBR`，不属于 `kFBBRAdaptive`，二者不得共用执行器。**

## 1. 结论与改造边界

新版不是在现有 `R1-R6` 或旧 PDF 的 N01-N21 上追加条件，而是要用 13:07 更新的 `ghtjykukli.pdf` 替换 `kFBBR` 的整棵负载判定树。核心变化有八项：

1. 横切分类只看 SRTT，不再让 DRate 上横切直接决定 Regime。SRTT 上横切和下横切各只接受 PDF 明列的三种真实形态。
2. 上/下横切候选只要不满足对应三种形态，就不算“已识别横切”，统一回退到 `SRTT has_wave` 主树；不能用一个兜底 `else` 强行判为重复横切。
3. SRTT 负肩 L1 下，DRate 无普通波动不再直接判不确定，而是继续用 `srtt_max > MaxRTT`、`srtt_min < RTprop`、默认值分成 Regime III/I/II。
4. 更新版共有 N01-N18 十八个确定性叶子；树内不再有“不确定叶子”，`INCONCLUSIVE` 只用于输入无效、覆盖不足或必须谓词无法计算。
5. 普通 DRate `has_wave` 明确忽略横切；DRate 周期性相似只把已验真的上横切作为硬否决，左右边缘线和顺位中间削平可以遮罩/简化处理。
6. 增加两个连接级状态：`MaxRTT` 和 `RTpropDRate`。两者可跨 Cruise 继承，不再用现有 `srtt_up/srtt_low` 代替。
7. Regime I/III 的下一注入基线改为 PDF 指定的 50% 摆幅公式；Regime II 同时把窗口平均 DRate 写入 pacing baseline 和 ProbedBw。
8. 输入无效的不确定处理保持当前 `kFBBR` 的后探/放大迭代；新增脚注中的“任一信号连续两个窗口无波动”直接复用同一状态机，先每次只补一个周期并用最新两周期慢速滑动复判，仍无恢复才按 1.25 倍放大，任一信号恢复波动后退出。

从结构上看，这版比旧树更合理，也更容易可靠实现：它先把“疑似横切”验真为六种有明确机制的形态，证据不足就回到普通波动而不是强行分类；SRTT 负责主形态，DRate 只在指定节点提供周期性或抖动佐证，信号角色不再混杂；上、下方向的 20%/30% 长度门和不同状态动作也被明确分开。这里的“更合理”是指假设更清楚、误判出口更安全、可测试性更强，不代表 20%/30% 等数值已经统计最优，仍需用第 10 节的合成与多 seed 回归校准。

本次行为改造只作用于正式 `kFBBR`（构造参数 `fbbr_window_baseline=true`）。`kFBBRAdaptive` 和 `legacy_spectral` 保持现状。最多只能复用无状态、无控制副作用的底层信号工具，例如重采样、MAD、稳健斜率和自相关；N01-N18 分类器、MaxRTT/RTpropDRate 状态、50% 基线公式、Regime II ProbedBw 更新、两窗无波动触发状态以及第 6 节保留的 `kFBBR` 不确定迭代状态机都必须由 `kFBBR` 专属调用链拥有。

## 2. PDF 判定语义的统一解释

### 2.1 Regime 含义

| PDF 名称 | 代码枚举建议 | 含义 | 控制方向 |
|---|---|---|---|
| Regime I | `kRegimeIUnderload` | 欠载 | 提高或居中注入基线 |
| Regime II | `kRegimeIIFullLoad` | 满载/目标负载 | 基线和 ProbedBw 取窗口平均 DRate |
| Regime III | `kRegimeIIIOverload` | 过载 | 降低或居中注入基线 |
| 不确定 | `kInconclusive` | 当前证据不足 | 不改基线；先补采一个周期，第二次仍不确定再调整探针幅度 |

### 2.2 时间定义

- 固定调制频率为 `f0`，发送波周期 `T = 1 / f0`。
- 周期性相似判断另外从实际 pacing/SRate 样本估计 `T_srate`：优先使用相邻同相位峰值、谷值或调制翻转点的时间差中位数。`T` 用于窗口长度和持续时间门；`T_srate` 用于比较响应周期，避免调度、pacing floor 或离散采样使实际 SRate 周期轻微偏离配置周期时误判。
- 每次正式判定窗口固定为 `W = 2T`。
- PDF 分方向规定长横线：连续上横切线必须 `L_top > 0.20T`，连续下横切线必须 `L_bottom > 0.30T`，均不是相对两周期窗口 `W`。比较严格使用 `>`；恰好 20%/30% 不命中 U2/L2。
- 初次采集、基线改变或不确定分支放大探针幅度后的采集都沿用现有时序：等待 1 个当前 SRTT 使新注入到达接收端，然后采集完整 `2T`。
- 重采样步长建议固定为 `dt = clamp(T/40, 1 ms, 5 ms)`；默认 5 Hz 时正好为 5 ms、每周期 40 点。任一信号每周期少于 20 个有效网格点时，本窗口只能判不确定。
- 原始覆盖率至少 85%；相邻真实样本间隔超过 `0.10T` 的区间不插值。DRate app-limited 样本占比超过 25% 时判为输入不足，不允许用低 DRate 直接推导欠载。
- 新树的 DRate 必须来自 `delivery_rate_latest`。`bandwidth_latest` 不能作为正式新版判定输入，只可保留给 legacy/对照实验。

### 2.3 “周期性相似波动”

它只表示响应周期与发送周期接近，不要求响应形状与三角发送波一致。必须同时满足：

1. 已去掉窗口左右边缘相连的水平横切线，并遮掉“顺位中间削平”区间；这些允许预处理的横切不会单独否决 periodic，但遮罩后仍要满足覆盖率和完整周期要求。所有上横切证据必须从未遮罩视图保留。
2. 遮罩后的有效覆盖率不少于 50%，原始窗口覆盖率不少于 85%。
3. 至少存在一个相对完整的波动周期：有峰、有谷，并各有一段持续时间不少于 `0.15T` 的上升和下降过程。
4. 从实际 SRate 估计的周期 `T_srate` 有效，响应信号自相关估计周期 `T_hat` 满足 `abs(T_hat - T_srate) / T_srate <= 0.20`，即允许响应周期位于实际 SRate 周期的 `[0.80, 1.20]`。恰好 20% 通过，超过才失败。
5. `corr(x(t), x(t + T_hat)) >= 0.50`。
6. **已验真的上横切是硬否决项**：同一信号只要存在正半周期肩部、连续上横线或一般上横切短接触，`periodic_similar=false`。左右边缘横线和顺位中间削平允许按第 1 项遮罩；下横切本身不设硬否决，但若处理后无法形成完整周期，仍会因第 2-5 项失败。DRate 横切不直接决定 Regime，只由其上横切否决 DRate 自身 periodic。

实现上直接复用当前 `AnalyzeCycleCompleteness()` 主路径及其“去顺位中间削平后重算”路径：保留峰谷完整性、上升/下降持续时间和自相关门限，不重新发明另一套周期检测；只把周期参照从名义 `T` 改为实际 `T_srate`，并把响应/SRate 周期误差门从 15% 放宽到 20%。`kFBBR` 使用独立参数 `fbbr.regime.period_tolerance_ratio=0.20` 和 `fbbr.regime.min_periodicity_correlation=0.50`；不得修改 `kFBBRAdaptive` 继续使用的旧 `waveform.period_tolerance_ratio=0.15`。现有 sender/response NCC、三角波同形度、相位同向率继续作为诊断字段，不新增为“周期性相似”的必要条件。

若实际 SRate 没有完整峰谷/翻转点、被 pacing floor 长时间截断或无法得到有限正值 `T_srate`，不得回退到响应信号自身周期来证明“与 SRate 相近”；应设置 `periodic_similarity_input_valid=false` 并记录 `INVALID_SRATE_PERIOD`，不能把输入无效偷换成“有效但不相似”。分类走到必须区分 periodic/非 periodic 的节点时只能返回不确定。本次放宽只作用于周期性相似谓词，不改变重复横切 motif 的 15% 间隔门、`0.15T` 上升/下降持续时间门、0.50 自相关门或上横切否决门。

该谓词与下一节普通“波动”的谓词完全独立：`periodic_similar` 仍按原来的周期完整性、自相关和周期误差判断，不调用 `has_wave`，也不受普通波动的陡度门限影响。

实现接口必须把周期相似表示为三态 `MATCH / NO_MATCH / INVALID_INPUT`，而不是一个默认 false 的裸 `bool`。第 3 节表格中的 “periodic/非 periodic” 均隐含 `periodic_similarity_input_valid=true`；输入无效时不得落入“非 periodic”一侧。

### 2.4 “波动”与“几乎无波动”

这里的“波动”不是“周期性相似波动”的弱化版，而是一个独立的**有效抖动检测**：不看周期、不和发送波做相关、不要求正负半周期对称；但必须同时出现可观幅度和足够陡的真实样本变化。只有缓慢、平滑地上升再下降不能算普通“波动”。

选择时域组合判据而不是单一方差/频谱的依据是：waveform length、阈值化相邻变化和 slope-sign-change 能分别描述路径活动、显著样本变化和方向反转；而经典实时检测工作也表明斜率单独使用不够，需要与幅度、持续范围等证据联合。[Hudgins 等的时域特征工作](https://doi.org/10.1109/10.204774)、[Pan–Tompkins 的斜率/幅度/宽度联合检测](https://doi.org/10.1109/TBME.1985.325532)均采用了这种低复杂度思路。噪声尺度使用 MAD 类稳健估计，避免少量 ACK 尖峰把门限整体抬高；MAD 及其稳健替代的统计性质可参见 [Rousseeuw–Croux](https://doi.org/10.1080/01621459.1993.10476408)。

#### 2.4.1 每周期预处理

SRTT 和 DRate 的普通波动使用相同三道门，但输入视图不同：

- SRTT 普通 `has_wave` 使用去边缘线、去顺位中间异常后的分析视图，但不遮掉已验真的 U1-U3/L1-L3 本体。该特征既供未命中六种 accepted clip 时的回退树使用，也必须在所有最终窗口计算，供第 6.2 节连续两窗计数使用。
- **DRate 普通 `has_wave` 必须使用未遮掉横切的原始有效视图**（只做缺失标记和三点中值去单点毛刺）。上/下/左右/中间横切均不能直接把 `drate_has_wave` 置 false；只要原始 DRate 仍通过幅度、陡变样本和往返三门，就算存在普通波动。

随后分别处理：

1. 按发送周期边界把两周期窗口切为 `C1`、`C2`；两个周期独立计算，不按正/负发送半周期切分。
2. 使用第 2.2 节的等时间隔网格。一个周期至少保留 20 个有效点；只有相邻两点都有效时才计算差分。
3. `y = median_filter_3(x)` 只用于去掉单点毛刺；不能做低通拟合或线性去趋势，否则会人为消除真正的抖动或制造方向反转。
4. 噪声尺度从 `x-y` 估计：

```text
sigma_x = 1.4826 * MAD(x - y)
```

#### 2.4.2 门 A：稳健幅度必须足够大

```text
A_pp      = P95(y) - P05(y)
level_ref = max(abs(median(y)), epsilon)
amp_ok    = A_pp >= max(6*sigma_x, 0.02*level_ref)
```

`P95-P05` 防止单个异常点制造大振幅；`6*sigma_x` 排除测量噪声；相对电平 2% 防止数值上有变化但对 RTT/速率没有实际意义。三者均为建议标定初值，不是 PDF 明示常数。

#### 2.4.3 门 B：必须存在足够陡的样本变化

对每个有效相邻点定义：

```text
delta_i = y[i] - y[i-1]
slope_i = delta_i / dt

# g_min 是相对“一个 A_pp 在一个 T 内变化”的归一化斜率倍数
g_i = abs(slope_i) * T / A_pp

theta_step = max(3*sqrt(2)*sigma_x,
                 g_min*A_pp*dt/T)
active_i = abs(delta_i) >= theta_step
```

建议 `g_min=3.5`。在理想连续采样下，一个周期为 `T` 的平滑正弦最大归一化斜率约为 `pi`，平滑三角波约为 2，因此二者都低于 3.5；局部较陡、在短时间内形成明显样本变化的抖动才会成为 active step。默认每周期约 40 点时，该项等价于要求单步变化至少约 `8.75% * A_pp`。噪声项 `3*sqrt(2)*sigma_x` 同时防止高频小噪声被当成陡变。

设有效相邻差分数为 `M`，显著变化数为 `K=sum(active_i)`：

```text
active_steps_ok = K >= max(4, ceil(0.10*M))
```

这会排除只有一个孤立尖峰造成的“上一步、下一步”两个变化，也排除整周期绝大部分时间都平滑、只在一个采样点跳变的信号。

#### 2.4.4 门 C：显著变化必须形成真实往返

只累计 active step：

```text
up_change   = sum(max(delta_i,  0) for active_i)
down_change = sum(max(-delta_i, 0) for active_i)
sig_path    = up_change + down_change

active_signs = compress_consecutive_signs(sign(delta_i) for active_i)
reversals    = len(active_signs) - 1

return_ok = up_change   >= 0.20*A_pp
            and down_change >= 0.20*A_pp
            and sig_path    >= 0.80*A_pp
            and reversals   >= 1
```

这里不要求 `up_change == down_change`，也不比较正负持续时间，所以不包含半周期对称假设；`0.20*A_pp` 只保证不是单向漂移，允许一侧变化远大于另一侧。方向反转前后允许夹着若干不活跃的平缓样本，因此先压缩 active sign，而不是只检查三个紧邻点。

#### 2.4.5 最终普通波动谓词

```text
cycle_has_wave = amp_ok and active_steps_ok and return_ok
has_wave = C1.cycle_has_wave or C2.cycle_has_wave
```

- 任意一个完整周期形成有效抖动即可，两个周期不需要形状相似，也不需要在同一相位出现。
- `has_wave=false` 统一覆盖四类无法识别为普通波动的状态：幅度不足、只有平滑起伏、只有单向漂移、只有孤立尖峰/噪声。
- SRTT 和 DRate 使用完全相同的算法和无量纲门限，只分别使用自己的 `A_pp`、`sigma_x` 和电平；但必须遵守上面的输入视图区别。

两个谓词允许出现四种组合，不能互相覆盖：

| 信号例子 | `has_wave` | `periodic_similar` | 解释 |
|---|---:|---:|---|
| 非周期、非对称、陡峭抖动 | true | false | 普通“波动”分支需要的典型情况 |
| 平滑但周期为 `T` 的正弦响应 | false | true | 可算周期性相似，但不算普通抖动 |
| 周期为 `T` 且含陡峭往返变化 | true | true | 两种特征同时成立，按判定树所在分支使用 |
| 平坦、缓慢漂移或噪声 | false | false | 无可识别波动证据 |

不建议使用 Teager–Kaiser 能量作为这里的主判据：该类算子适合窄带或 AM–FM 振荡能量/瞬时频率分析，[原始能量算子工作](https://doi.org/10.1109/97.404130)也强调调制正弦场景；本分支刻意不要求周期或单分量振荡，而且相邻差分已经能直接表达用户需要的“真实斜率样本变化”。TKEO 可留作离线诊断，不进入正式决策。

### 2.5 共享横切机制下的两类证据：连续段与重复短接触

肩部削平与上/下横切属于同一种限幅机制：原信号继续变化，但观测值被限制在近似固定的阈值上。信号 declipping 研究会显式约束削平样本与 clipping threshold 的一致性，并强调噪声下必须可靠区分 clipped/unclipped samples，而不能把普通缓变直接当成削平。[Kitić 等的一致性 declipping 模型](https://doi.org/10.1109/ICASSP.2013.6638804)和 [Harvilla–Stern 的噪声下 clipped-sample 分类](https://doi.org/10.21437/Interspeech.2015-531)支持这种解释。[Laguna–Lerch 的检测方法](https://musicinformatics.gatech.edu/wp-content_nondefault/uploads/2016/09/Laguna_Lerch_2016_An-Efficient-Algorithm-For-Clipping-Detection-And-Declipping-Audio.pdf)则把“横切电平检测”和“各个横切区间定位”分开，并同时使用电平分布聚集、时域平坦度和区间边界；这正好说明一般横切不能被“是否存在一条长连续平台”代替。

因此实现一个共享的 `DetectHorizontalClipEvidence()`，但必须输出两种不同粒度的证据：

1. `ContinuousHorizontalSegment`：一条连续、可测长度的平台，供肩部、U2/L2 长横线和左右边缘线使用。
2. `RepeatedClipLineEvidence`：若干短而可分离的横切接触片段反复聚集在同一电平，供 U3/L3“一般横切”使用；它不继承连续段的 `0.15T` 最短长度门，更不要求任一片段超过 20%/30%。

两者共享同一套去噪、噪声尺度、电平容差、近水平步长、侧翼变化和横切边界定义，区别只在证据如何聚合；不能为 U3/L3 另造一个只看“多个值差不多”的宽松检测器。输入必须是未去趋势、只经三点中值去单点毛刺的等时间隔序列。设：

```text
y       = median_filter_3(x)
A_pp    = P95(y) - P05(y)
sigma_x = 1.4826 * MAD(x - y)
s_i     = robust_local_slope(y, centered_window=0.10T)
S80     = P80(abs(s_i))
r_i     = x_i - y_i
sigma_s = 1.4826 * MAD((r_i-r_(i-1))/dt)

theta_flat = max(3*sigma_s, 0.10*S80)
theta_side = max(3*sigma_s, 0.25*S80)
theta_kink = max(3*sigma_s, 0.25*S80)
```

`robust_local_slope` 建议使用窗口内 Theil–Sen 中位斜率，避免少量 ACK 尖峰支配普通最小二乘结果；其稳健斜率定义可参见 [Sen 1968](https://doi.org/10.1080/01621459.1968.10480934)。局部多项式求导和分段斜率变点可分别参见 [Savitzky–Golay](https://doi.org/10.1021/ac60214a047)及 [CPOP 斜率变点工作](https://doi.org/10.1080/10618600.2018.1512868)。本实现不需要运行完整 CPOP，只借用“分段斜率而非单点差分”的建模原则。

#### 2.5.1 连续横切段

一个连续候选区间 `H=[a,b]` 只有同时满足以下条件，才设置 `continuous_horizontal=true`：

1. **长度和覆盖**：至少 3 个有效点，区间有效覆盖率不低于 85%，`L=b-a >= 0.15T`。
2. **绝大多数内部步长确实近水平**：只使用两个端点都在 `H` 内的相邻步长 `d_i=(y_i-y_(i-1))/dt`，要求 `flat_fraction = count(abs(d_i)<=theta_flat)/valid_internal_step_count >= 0.90`。这里不能让局部斜率窗口跨过候选边界，也不能只用区间首尾平均斜率；前者会污染真平台，后者会把圆顶/圆底的正负斜率相互抵消。
3. **电平稳定**：

```text
level_span = P95(y_H) - P05(y_H)
level_span <= max(4*sigma_x, 0.10*A_pp)
abs(theil_sen_slope(H))*L <= max(3*sigma_x, 0.05*A_pp)
```

4. **常数模型不劣于斜线模型**：对同一段分别拟合常数和直线，按 `BIC=n*ln(max(RSS/n,epsilon))+k*ln(n)` 计算，常数模型 `k=1`、直线模型 `k=2`，要求 `BIC_constant <= BIC_linear`。BIC 只作辅助门，不能替代第 2、3、5 项。
5. **有可见且突变的横切边界**：对左右各取 `w=max(4*dt, 0.10T)` 的邻接上下文，估计宏观 `s_left/s_right`；再分别取边界外最后 2 个内部步长和边界内最先 2 个内部步长，得到 `q_out/q_in`。每个在窗口内可观测的边界都必须满足：

```text
abs(s_side) >= theta_side
abs(s_side)*w >= max(3*sigma_x, 0.10*A_pp)
abs(q_in) <= theta_flat
abs(q_out - q_in) >= theta_kink
```

内部水平段要求左右两个边界都通过；与窗口左/右端相连的水平段只检查可观测的一侧。宏观侧翼证明横切前后确有足够信号变化，`q_out/q_in` 则证明斜率是在不超过约 `2*dt` 的边界内突然变成近零。二者共同把真正“撞到横切线”的平台与斜率逐渐趋零的普通平缓变化分开。

若 `A_pp < max(6*sigma_x, epsilon)`，整窗没有足够动态范围去证明横切，所有横切特征必须为 false，不能因为整窗近似常数就判为长横线；恰等于门限时允许继续接受后续严格验真。

#### 2.5.2 一般横切：同一电平上的重复短接触

“SRTT 波形被一条假想横切线上/下横切”描述的是**横切电平反复出现**，不是一条连续平台足够长。其片段可以短、可以在两周期内断续出现，单个片段和全部片段的时长占比都可以明显低于 U2 的 20% 或 L2 的 30%；只要多个独立接触稳定落在同一个上/下限电平，并保留进入、离开该限幅线的证据，就应算一般横切。

先分别在上端和下端估计候选横切电平 `h`。不能直接取单个 max/min；在对应极端 20% 幅值带内，对样本电平做一维稳健聚类，选择跨两个周期均有成员的最外侧簇，取其加权中位数为 `h`：

```text
upper_band_i = y_i >= P05(y) + 0.80*A_pp
lower_band_i = y_i <= P95(y) - 0.80*A_pp
epsilon_h = max(4*sigma_x, 0.05*A_pp)
contact_i = abs(y_i - h) <= epsilon_h
```

将相邻 contact 样本组成短片段；被一个无效网格点或不超过 `0.025T` 的小缺口隔开的片段可以保留为同一 contact event，但缺口不计入横切时长，也不能跨越大于 `0.05T` 的采样空洞。一个 `RepeatedClipLineEvidence` 必须同时满足：

1. **低占比也可成立，但证据不能只有单点**：两个发送周期各至少有 2 个 contact 样本，整窗至少 `max(4,ceil(0.05*N_valid))` 个；这些样本可以分属多个短片段，不要求存在 `L>=0.15T` 的单段，也不设置 20%/30% 总时长下限。
2. **多段横切必须覆盖足够大的窗口时间跨度**：只对最终属于同一候选横切电平 `h` 的 contact 片段计算：

```text
contact_span = latest_contact_end - earliest_contact_start
contact_span_ratio = contact_span / W       # W = 2T
span_ok = contact_span_ratio >= 0.50
```

恰好 50% 通过，49.999% 不通过。这里是最早片段到最晚片段的**包络时间跨度**，中间空白计入 span；不是各片段时长之和，中间空白和采样缺口都不能伪装成横切持续时长。实现时优先用整数时间比较 `2*contact_span_us >= window_duration_us`，避免浮点舍入把恰好 50% 判失败；`window_duration` 取本次完整判定窗的 `2T`，不是有效样本首尾间隔。
3. **确实总在同一条线**：全部 contact 样本满足 `P95-P05 <= epsilon_h`；两个周期的 contact 中位电平差不超过 `max(3*sigma_x,0.05*A_pp)`。该门检查的是跨片段电平一致性，而不是各片段是否相连。
4. **跨周期复现**：每周期至少形成一个 contact event；选择最匹配的一对 event，要求 `abs(delta_t-T)/T <= 0.15`。每个周期在 contact event 以外还必须离开 `h` 至少 `max(4*sigma_x,0.10*A_pp)`，证明不是整窗平坦或长期量化在同一值。
5. **聚合后的接触仍然近水平**：只统计片段内部的有效步长，要求至少 2 个内部步长且 `flat_fraction_pooled >= 0.90`。孤立 contact 样本不能贡献平坦步长，只能辅助电平聚类。
6. **具有横切边界而不是自然圆顶/圆底**：对每个可观测 event 使用第 2.5.1 节相同的 `theta_side/theta_kink`。至少 75% 的可观测边界通过，且两个周期都至少证明一次“显著侧翼 -> 近水平 contact”的斜率突变。上横切还要求接触前后样本位于 `h` 下方，下横切要求位于 `h` 上方。
7. **限幅反事实成立**：分别用 contact event 左、右侧翼做稳健直线外推。上横切要求至少一个周期的两侧外推包络会超过 `h + max(3*sigma_x,0.05*A_pp)`；下横切要求低于 `h - max(3*sigma_x,0.05*A_pp)`。该门用来拒绝自然到达同一极值后平滑返回的圆顶/圆底，以及普通量化三角拐点。

第 1 项的 5% 只是“至少有多少接触样本才能验真”的证据量门，不是横切持续时间门；第 2 项的 50% 是用户指定的多段横切窗口跨度硬门。正式区分始终是：U2/L2 检查**单条连续段长度**，U3/L3 检查**多个短片段是否反复落在同一电平，且首末片段横跨至少半个窗口**。若采样稀疏到每周期不足 2 个 contact 样本，就没有足够信息证明一般横切，应回退而不是降低为单点极值判断。每周期 2 点、总样本 5%、边界通过率 75% 和外推越界 5% 均是建议标定初值；50% span 不参与这组参数的自由放宽，并独立于 U2/L2 的 20%/30% 连续长度门。

### 2.6 SRTT 上/下横切各自只允许三种有效情况

`ghtjykukli.pdf` 把横切收敛为 SRTT 的两个方向、每个方向三个形态。先用第 2.5 节共享模块分别提取连续段和重复短接触证据，再严格按下列顺序派生；DRate 的横切证据不能直接决定 Regime，其中上横切用于否决其 `periodic_similar`，其余横切只供允许的遮罩/简化和诊断。

#### 2.6.1 上横切三种情况

1. **U1：SRTT 正半周期肩部削平，且削平后仍能识别周期波动**。候选是真实内部水平段，对齐后与 SRate 正半周期重合率至少 75%，并满足正半周期局部上极值和相反肩斜率：

```text
P95(y_in_positive_half) - median(y_H)
    <= max(3*sigma_x, 0.10*A_pp)
s_left >= theta_side and s_right <= -theta_side
abs(s_left)*w  >= max(3*sigma_x, 0.15*A_pp)
abs(s_right)*w >= max(3*sigma_x, 0.15*A_pp)
```

   还必须设置 `shoulder_cycle_recognizable=true`：在未遮掉肩部本体的时间轴上，至少找到一组有序的“稳健谷值 → 上肩平台 → 稳健谷值”，两个外侧谷值的间隔相对 `T_srate` 误差不超过 20%，谷到肩、肩到谷各自持续不少于 `0.15T`，且峰谷摆幅通过第 2.4.2 节幅度门。它只证明削平后仍看得到一个周期骨架，不要求肩部两侧形状相同，也不要求自相关 0.50。该谓词同样必须三态化；真实肩部候选已经成立但 `T_srate`/覆盖不足使骨架无法计算时，不能把 INVALID 当成 false 后静默落入可能产生不同结果的 U3/L3 或普通波动分支，应返回不确定；若同一连续段独立满足结果等价的 U2，则可以按 U2 决策并记录 U1 输入无效。

2. **U2：一条连续上横切线严格长于 `0.20T`**。必须是共享门确认的真实上水平段，并接近窗口稳健上极值：`P95(y)-median(y_H) <= max(3*sigma_x,0.10*A_pp)`。恰好 `0.20T` 不命中。
3. **U3：一般上横切——波形反复接触同一条假想上横切线**。按第 2.5.2 节形成有效 `RepeatedClipLineEvidence`：两个周期都在同一上限电平 `h_top` 出现短 contact，可以断续、实际横切时长占比低，不要求任何一条连续片段达到 `0.15T` 或 `0.20T`；但首末片段时间跨度必须 `>=0.50W`，并通过同电平聚类、跨周期复现、聚合近水平步长、横切边界和上侧外推越界门。U2 失败本身不能证明 U3，必须有这组正证据。

上横切内部优先级固定为 `U1 > U2 > U3`。同一证据同时满足肩部和长线时按 U1；长线和重复 motif 同时满足时按 U2。

#### 2.6.2 下横切三种情况

1. **L1：SRTT 负半周期肩部削平，且削平后仍能识别周期波动**。候选是真实内部水平段，对齐后与 SRate 负半周期重合率至少 75%，并满足负半周期局部下极值和相反肩斜率：

```text
median(y_H) - P05(y_in_negative_half)
    <= max(3*sigma_x, 0.10*A_pp)
s_left <= -theta_side and s_right >= theta_side
abs(s_left)*w  >= max(3*sigma_x, 0.15*A_pp)
abs(s_right)*w >= max(3*sigma_x, 0.15*A_pp)
```

   同样要求 `shoulder_cycle_recognizable=true`，但有序骨架改为“稳健峰值 → 下肩平台 → 稳健峰值”；两个外侧峰值间隔、两段持续时间和幅度门与 U1 相同。

2. **L2：一条连续下横切线严格长于 `0.30T`**。必须是共享门确认的真实下水平段，并接近窗口稳健下极值：`median(y_H)-P05(y) <= max(3*sigma_x,0.10*A_pp)`。恰好 `0.30T` 不命中。
3. **L3：一般下横切——波形反复接触同一条假想下横切线**。使用与 U3 对称的 `RepeatedClipLineEvidence`：短 contact 可以断续且实际横切时长占比低，不要求任何连续片段达到 `0.15T` 或 `0.30T`，但首末片段时间跨度必须 `>=0.50W`；同电平、跨周期、聚合近水平步长和边界门相同，反事实外推方向改为低于 `h_bottom`。L2 失败不能自动变成 L3。

下横切内部优先级固定为 `L1 > L2 > L3`。

这里的 `shoulder_cycle_recognizable` 与第 2.3 节 `periodic_similar` 必须分开：前者是 U1/L1 的局部结构验真，允许肩部本身存在；后者是正式“周期性相似波动”，仍受上横切硬否决、自相关和完整周期等全部规则约束。不能因为 U1 的残余周期可识别就把 SRTT `periodic_similar` 置 true。L1 残余骨架输入无效时也不能默认 false 后直接降为 L2/L3：只有其他独立证据在所有可能解释下得到完全相同的 Regime 和状态动作时才可继续，否则返回输入不确定。

#### 2.6.3 连续长横切与一般横切的硬边界

| 判别问题 | U2/L2 连续长横切 | U3/L3 一般横切 |
|---|---|---|
| 核心证据 | 一条不间断的真实水平段 | 多个短 contact 反复落在同一横切电平 |
| 长度/跨度条件 | 上严格 `L>0.20T`；下严格 `L>0.30T` | 不设 20%/30% 连续或累计时长门；但首末 contact 的包络时间跨度必须 `>=0.50W` |
| 是否必须跨周期复现 | 否，一条长线就足够 | 是，两个周期都要有同电平接触事件 |
| 是否允许断续 | 否，缺口会终止该连续段 | 是，小片段之间可以断开，缺口不计入横切时长 |
| 防误判重点 | 单段平坦度、电平稳定、横切边界 | 跨片段同电平、跨周期复现、聚合平坦步长、边界和反事实越界 |
| 二者同时成立 | U2/L2 优先 | 只在更高优先级形态未命中时使用 |

所以，“最长连续段只有 `0.06T`、两个周期各出现若干短横切片段、所有片段稳定落在同一个 `h_top`，且首末片段跨度达到 `0.50W`”可以命中 U3；它不应因为连续或累计横切时长没有超过 `0.20T` 而被丢弃。反过来，即使片段电平一致，只要首末跨度 `<0.50W`，或者没有横切边界、只出现一个周期，仍然不是 U3/L3。

#### 2.6.4 横切验真失败必须回退

```text
recognized_upper = U1 or U2 or U3
recognized_lower = L1 or L2 or L3

if recognized_upper:
    enter_upper_clip_subtree()
else if recognized_lower:
    enter_lower_clip_subtree()
else:
    enter_srtt_has_wave_fallback()
```

“检测到疑似上/下方向”“局部变平”“半周期振幅变小”都不足以进入横切子树。只有六种形态之一完整验真才算存在可分类横切；不满足时必须放弃该横切候选，使用清理后 SRTT 的普通 `has_wave` 分支。实现中禁止把 `else` 写成 U3/L3：一般横切必须有跨两个周期、同一电平的重复短接触正证据。

若上下方向同时存在有效形态，按 PDF 的 `if/else` 让上横切优先，并记录 `both_clip_directions`；若上方向只有未验真的疑似候选、下方向存在完整 L1/L2/L3，则下横切仍可进入，疑似上横切不能阻挡已验真的下横切。

普通圆顶/圆底、缓慢进入和离开平台、某半周期振幅偏小、只有一侧波形可见、模板拟合差、仅靠相邻点相等形成的量化台阶，一律不能成为 U1/L1。肩部必须保留 `continuous_horizontal=true`、两个横切边界和半周期局部极值三类证据。

### 2.7 左右水平横切线与宽泛的顺位中间削平

两者机制不同，不能继续共用 `plateau_candidate`：

- **左右边缘横线**仍是限幅/窗口截断产生的真实水平线，必须通过第 2.5 节共享标准；左线连接第一个有效点，右线连接最后一个有效点，只允许缺少窗口外不可观测的那一个横切边界。
- **顺位中间削平**不是限幅证据，而是原本应沿同一顺位继续上升或下降的内部趋势被局部驻留、减速、反向、小折线或样本顺位异常打断。它不要求水平、不要求接近极值，也不要求前后斜率相反；适合按局部分段斜率变化识别。MOSUM 和 CPOP 类工作正是通过局部跳变/斜率变化把这种结构变化与单一趋势分开，[MOSUM 的分段线性检测](https://doi.org/10.1080/00401706.2024.2308202)给出了线性复杂度的可行依据。

对至少含 3 个有效点的内部候选 `M=[a,b]`，左右上下文宽度 `w=max(4*dt,0.10T)`，使用 Theil–Sen 分别得到 `s_left`、`s_right`，候选内保留逐点稳健局部斜率 `s_i`。建议初值：

```text
0.05T <= duration(M) <= 0.35T
theta_trend    = max(3*sigma_s, 0.20*S80)
theta_mismatch = max(3*sigma_s, 0.50*abs(s_ref))
s_ref          = median(s_left, s_right)

outside_continues = s_left*s_right > 0
                    and min(abs(s_left),abs(s_right)) >= theta_trend
                    and abs(s_left-s_right)
                        <= max(3*sigma_s,
                               0.75*max(abs(s_left),abs(s_right)))

mismatch_i = sign(s_i) != sign(s_ref)
             or abs(s_i) < 0.50*abs(s_ref)
             or abs(s_i-s_ref) >= theta_mismatch
M_valid          = valid_slope_count(M)
K_mismatch       = count(mismatch_i in M)
mismatch_ratio   = K_mismatch / M_valid
max_mismatch_run = 最长连续 mismatch_i 数
mismatch_samples_ok = K_mismatch >= max(2, ceil(0.25*M_valid))
                      and max_mismatch_run >= 2

s_entry = median(s_i in first third of M)
s_exit  = median(s_i in last third of M)
entry_change = abs(s_entry - s_left)
exit_change  = abs(s_right - s_exit)

left_anchor     = left_context_fit(a)
right_anchor    = right_context_fit(b)
bridge(t)       = 连接 (a,left_anchor) 与 (b,right_anchor) 的直线
bridge_deviation = P95(abs(y_M - bridge(M)))

middle_sequential = outside_continues
                    and mismatch_samples_ok
                    and entry_change >= theta_mismatch
                    and exit_change >= theta_mismatch
                    and bridge_deviation
                        >= max(3*sigma_x, 0.05*A_pp)
```

样本占比不使用 50%：候选边界和局部稳健斜率窗口会引入若干过渡点，50% 会漏掉只有 2-4 个样本但幅度明显的真实顺位异常。改为 25% 并要求至少 2 个连续异常斜率；默认每周期 40 点、候选最多 14 点时，实际至少需要 2-4 个异常斜率。单个离群点会被三点中值和连续数门拒绝。

为证明这是局部打断而不是整段趋势自然改变，候选入口和出口都要出现相对于 `s_ref` 的斜率偏离，且出口后恢复到与入口前同方向的趋势。实现可以用两个局部 slope-change 点框定 `[a,b]`；每个周期最多遮罩总计 `0.35T`。候选冲突时按无量纲 `score=(bridge_deviation/max(3*sigma_x,0.05*A_pp))*mismatch_ratio` 从高到低选择互不重叠区间，不能无限扩大 mask 来强行制造周期相关性。

顺位中间削平明确**不要求**第 2.5 节的 `continuous_horizontal`、90% 近零斜率、低电平跨度、极值位置或相反肩斜率。只要前后确实是同方向连续趋势，而中间有超过噪声和 5% 摆幅的局部斜率不符合，就可以遮罩。反过来，天然峰/谷的前后斜率相反，不能作为顺位中间削平；平滑曲线没有两个局部 slope-change 边界，也不能通过。

必须同时保留“原始证据视图”和“清理后特征视图”，提取顺序固定为：

1. 在未遮罩信号上同时提取 `ContinuousHorizontalSegment` 和 `RepeatedClipLineEvidence`，形成之后不可被 mask 改写的 `raw_clip_evidence`；
2. 只从 SRTT 原始证据派生 U1/U2/U3、L1/L2/L3 六种可分类横切形态：U1/U2/L1/L2 读取连续段，U3/L3 读取重复短接触；DRate 横切证据不直接分类；
3. 复制 valid bitmap 建立 `periodic_mask`，遮掉左右边缘横线；
4. 在该视图上检测顺位中间异常，只遮掉不与已保留肩部区间重叠的候选；
5. SRTT 普通 `has_wave` 在上述清理视图上计算；DRate 普通 `has_wave` 则回到未遮罩的原始有效视图，只做缺失处理和三点中值去单点毛刺；
6. `periodic_similar` 在保留原时间轴的 `periodic_mask` 上计算，但必须回看未遮罩的 `raw_clip_evidence`：已验真的上横切为硬否决；下横切、左右边缘线和顺位中间削平本身不否决，只能通过覆盖率与完整周期门间接使 periodic 失败；
7. 分类器严格按 N01-N18 读取证据：横切子树只读六种已验真的 SRTT `raw_clip_evidence`，普通波动读取各自规定的输入视图，周期条件读取 `periodic_mask` 及原始上横切否决位；六种均未命中时直接进入 SRTT `has_wave` 回退树。

PDF 中“未命中有效横切后进入 SRTT 波动分支”描述的是**分类语义和证据优先级**。工程上可以提前构造清理视图，因为 U1/U2 也要读取处理后的 DRate `periodic_similar`；但绝不能原地删除原始连续段、重复短接触或肩部证据。分类器在 U3/L2/L3 可以短路不需要的 periodic 计算，但为了第 6.2 节跨窗触发，每个最终窗口仍必须计算 SRTT/DRate 两个普通 `has_wave` 及各自有效位，不能因已进入横切叶子就跳过。

SRTT 和 DRate 必须各自检测、各自遮罩。任何区间同时满足肩部和中间候选时，肩部优先并禁止遮罩；否则会删掉 U1/L1 的 SRTT 证据，或掩盖 DRate 的上横切 periodic 硬否决。特别地，DRate 普通 `has_wave` 不读取这些 mask，不能因任意方向横切而被强制置为 false。

两类削平最终边界如下：

| 维度 | 肩部削平 | 顺位中间削平 |
|---|---|---|
| 产生机制 | 上/下阈值限幅，原波形极值被横切 | 同向演化趋势中的局部驻留、减速、反向或顺位异常 |
| 必须是真水平线 | 是，调用 `DetectContinuousHorizontalSegments()` | 否 |
| 边界斜率 | 横切边界突变；宏观前后斜率相反 | 外部前后斜率同向；内部斜率明显不符合且随后恢复 |
| 位置要求 | 对齐后的正/负发送半周期局部极值 | 波形内部，不能是天然峰/谷或窗口边缘 |
| 最小证据 | 90% 近水平、电平稳定、两个横切边界、半周期极值；U1/L1 还要有肩部前后的完整周期骨架 | 至少 2 个连续且总数不少于 25% 的内部斜率不符合、双边变化、偏离 bridge 超过噪声/5% 摆幅 |
| 在判定树中的作用 | SRTT 正肩 U1/负肩 L1 是横切子树直接特征；其中上肩否决同一信号 periodic，下肩本身不作硬否决 | 只生成清理 mask，自己不直接决定 Regime |
| 冲突优先级 | 保留 | 与肩部重叠时放弃 mask |

## 3. `kFBBR` 专属的新版完整有序判定树

以下顺序就是代码顺序。任何一个叶子命中后立即返回。`DRate periodic` 指第 2.3 节三态谓词，`has_wave` 指第 2.4 节普通波动；表内所有“非 periodic”“无波动”都要求对应输入有效。更新版 PDF 有 18 个确定性叶子，不确定不编号。

| 优先级/规则号 | 条件 | 结果 | 同步状态动作 |
|---:|---|---|---|
| N01 | U1 SRTT 正半周期肩部削平；DRate periodic | Regime II | 无 |
| N02 | U1；DRate 有效但非 periodic | Regime III | `MaxRTT = window_srtt_max` |
| N03 | U2 SRTT 连续上横切线 `L > 0.20T`；DRate periodic | Regime II | 无 |
| N04 | U2；DRate 有效但非 periodic | Regime III | `MaxRTT = window_srtt_max` |
| N05 | U3 SRTT 在同一假想上横切线反复出现多小段，首末包络 `>=0.50W` | Regime III | `MaxRTT = window_srtt_max` |
| N06 | L1 SRTT 负半周期肩部削平；DRate has_wave | Regime I | 刷新 RTprop；`RTpropDRate = mindrate` |
| N07 | L1；DRate 无普通波动；`srtt_max > MaxRTT` | Regime III | 仅比较，不刷新下横切状态 |
| N08 | 同 N07 前提且 N07 未命中；`srtt_min < RTprop` | Regime I | 仅比较，不刷新下横切状态 |
| N09 | L1；DRate 无普通波动；max/min 均未越界 | Regime II | 无 |
| N10 | L2 SRTT 连续下横切线 `L > 0.30T` | Regime I | 保留既有 RTprop 刷新；`RTpropDRate = mindrate` |
| N11 | L3 SRTT 在同一假想下横切线反复出现多小段，首末包络 `>=0.50W` | Regime I | 刷新 RTprop；`RTpropDRate = mindrate` |
| N12 | 六种横切均未命中；SRTT has_wave；`srtt_max > MaxRTT` | Regime III | 仅比较 |
| N13 | 同 N12 前提且 N12 未命中；`srtt_min < RTprop` | Regime I | 仅比较 |
| N14 | SRTT has_wave，N12/N13 均未命中 | Regime II | 无 |
| N15 | 六种横切均未命中；SRTT 无波动；DRate has_wave | Regime I | 无 |
| N16 | SRTT/DRate 均无波动；`srtt_max > MaxRTT` | Regime III | 仅比较 |
| N17 | 同 N16 前提且 N16 未命中；`srtt_min < RTprop` | Regime I | 仅比较 |
| N18 | SRTT/DRate 均无波动，N16/N17 均未命中 | Regime II | 无 |

等价伪代码如下：

```text
upper_case = first_match(U1_positive_shoulder,
                         U2_long_top_strict_20pct,
                         U3_repeated_top_clip_envelope_50pct)
lower_case = first_match(L1_negative_shoulder,
                         L2_long_bottom_strict_30pct,
                         L3_repeated_bottom_clip_envelope_50pct)

if upper_case == U1:
    if drate.periodic == INVALID_INPUT: return INCONCLUSIVE
    if drate.periodic == MATCH:         return II         # N01
    else:                              return III+MAXRTT # N02
else if upper_case == U2:
    if drate.periodic == INVALID_INPUT: return INCONCLUSIVE
    if drate.periodic == MATCH:         return II         # N03
    else:                              return III+MAXRTT # N04
else if upper_case == U3:
    return III + SET_MAX_RTT                                # N05
else if lower_case == L1:
    if !drate.wave_input_valid: return INCONCLUSIVE
    if drate.has_wave:
        return I + REFRESH_RTPROP + SET_RTPROP_DRATE        # N06
    else:
        if max_rtt_valid and srtt.max > MaxRTT: return III # N07
        else if rtprop_valid and srtt.min < RTprop: return I # N08
        else: return II                                     # N09
else if lower_case == L2:
    return I + REFRESH_RTPROP + SET_RTPROP_DRATE             # N10
else if lower_case == L3:
    return I + REFRESH_RTPROP + SET_RTPROP_DRATE             # N11
else:
    # 包括：完全没有横切，以及有疑似横切但不属于 U1-U3/L1-L3
    if !srtt.wave_input_valid: return INCONCLUSIVE
    if srtt.has_wave:
        if max_rtt_valid and srtt.max > MaxRTT: return III   # N12
        else if rtprop_valid and srtt.min < RTprop: return I # N13
        else: return II                                      # N14
    else:
        if !drate.wave_input_valid: return INCONCLUSIVE
        if drate.has_wave: return I                           # N15
        else if max_rtt_valid and srtt.max > MaxRTT: return III # N16
        else if rtprop_valid and srtt.min < RTprop: return I # N17
        else: return II                                      # N18
```

### 3.1 冲突时的确定性处理

因为这是有序树，冲突必须服从优先级，而不是投票：

- DRate 上/下横切不产生分类优先级，不能触发 MaxRTT、RTprop 或 RTpropDRate；普通 `drate_has_wave` 完全忽略横切，只有 DRate 上横切否决其 `periodic_similar`。
- SRTT 有效上、下形态同时出现：上横切子树优先，同时记录 `both_clip_directions`。
- 上横切内部 `U1 > U2 > U3`；下横切内部 `L1 > L2 > L3`。同一组横切证据满足多个形态时只记录最高优先级 rule。
- 疑似上/下横切没有通过对应三形态时，不产生冲突优先级，必须回退到 N12-N18；不能把“存在 candidate”当成“存在 accepted clip”。
- DRate `periodic_similar=true` 与 DRate 已验真上横切互斥；与下横切、左右边缘线或已遮罩 middle 同时存在不构成不变量违规，只要周期完整性仍通过。

## 4. `kFBBR` 专属连接级状态和生命周期

建议新增：

```cpp
bool max_rtt_valid_;
double max_rtt_ms_;
bool rtprop_drate_valid_;
QuicBandwidth rtprop_drate_;
uint64_t max_rtt_source_cruise_id_;
uint64_t rtprop_drate_source_cruise_id_;
```

状态更新规则：

- 所有更新都发生在第 6.2 节跨窗触发检查之后；若当前窗口因 `TWO_WINDOW_NO_WAVE` 被转入重试，即使纯分类器返回 N02/N04/N05 或 N06/N10/N11，也只写 trace，不执行任何状态副作用。
- N02/N04/N05 命中且窗口 SRTT 统计有效时，直接赋值 `MaxRTT = window_srtt_max`；这是“本次上横切过载窗口内的最大值”，不是与旧值再取 max。N01/N03 为 Regime II，不更新 MaxRTT。
- N06/N10/N11 命中且 DRate 统计有效时，在执行既有下横切 RTprop 刷新后赋值 `RTpropDRate = mindrate`。L1 下 DRate 无波动得到的 N07/N08/N09 只做 MaxRTT/RTprop 比较，不刷新 RTprop，也不写 RTpropDRate。
- 每轮 `EnterCruise()` 的初始 pacing baseline 明确取进入该轮时的 Native BBR `MaxBandwidth()` 快照；该值无效时才回退到 `BandwidthEstimate()`。不能用上一轮 ProbedBw 或上一轮最终注入 baseline 作为新一轮起点。
- `EnterCruise()` 不清空二者，也不再使用“当前 MaxBw 与上一 Cruise 相差小于 25%”作为继承门槛。
- 只在连接初始化/销毁时清空；路径变化时仍按 PDF 允许继承，直到新的 N02/N04/N05 或 N06/N10/N11 覆盖。
- N07/N12/N16 在 `max_rtt_valid=false` 时跳过 MaxRTT 比较；N08/N13/N17 使用 BBR 模型当前有效 RTprop。相等不算大于或小于，继续落到后续条件，最终在各自子树默认 Regime II。
- `RTpropDRate` 用于执行器前还要满足 `0 < RTpropDRate <= maxdrate`。继承值高于当前 `maxdrate` 时视为本窗口不可用，但不销毁保存值。

注意：PDF 的“更新 RTprop 不变，增加记录 RTpropDRate”解释为保留当前下横切触发的 RTprop 更新机制，再增加 DRate 参考值；不是禁止更新 RTprop。副作用必须跟随最终选中的 N06/N10/N11，不能因为出现未通过 L1-L3 的疑似下横切，或 L1+DRate 无波动进入 N07/N08/N09，就提前更新。

## 5. `kFBBR` 专属的 Regime 到注入基线/ProbedBw 执行器

> 本节是 PDF 中 `kFBBR` 判定结果的专属控制动作，不是 FBBR 系列通用执行器。即使 `kFBBRAdaptive` 内部存在名称相似的 underload/full-load/overload 状态，也不得把这些状态传入本节公式。

调用所有权必须先于公式判断：

```text
if algorithm_mode != kFBBR:
    forbidden: ClassifyFbbrRegimeV2()
    forbidden: ComputeFbbrV2InjectionBaseline()
    forbidden: ApplyFbbrV2RegimeDecision()
    forbidden: update MaxRTT / RTpropDRate / PDF ProbedBw
```

`kFBBRAdaptive` 继续执行其现有 adaptive baseline step、delta/queue guard、确认计数和 ProbedBw 逻辑；本方案不改变它的控制量、停止条件或状态生命周期。

执行顺序必须是 `纯分类 -> 第 6.2 节跨窗触发检查 -> 状态副作用 -> baseline/ProbedBw`。`TWO_WINDOW_NO_WAVE` 一旦在本窗触发，后两步整体跳过；不能先更新 MaxRTT/RTprop/ProbedBw 再进入重试。

所有统计量从同一份有效、等时间隔、未去趋势的 DRate 序列计算，避免当前按 ACK 事件逐样本平均造成 ACK 密集区权重过高：

```text
mindrate = min(DRate_clean)
maxdrate = max(DRate_clean)
meandrate = arithmetic_mean(DRate_clean)   # 等间隔后即时间平均
swing = maxdrate - mindrate
reference_gap = maxdrate - RTpropDRate
use_midpoint = RTpropDRate_valid_for_this_window
               and swing > 0.50 * reference_gap
midpoint = mindrate + swing / 2
```

比较仍严格使用 `>`；等于 50% 走“否则”分支。

### 5.1 Regime I

```text
next_baseline = use_midpoint ? midpoint : maxdrate
```

### 5.2 Regime II

```text
next_baseline = meandrate
ProbedBw      = meandrate
```

- 每个 Regime II 窗口都覆盖更新二者，直到本轮 Cruise 结束。
- Regime I/III 不更新 ProbedBw；本轮从未出现 Regime II 时，不发布新的 FBBR ProbedBw，Cruise 后继续使用 Native BBR 回退。
- Cruise 结束时发布最后一个有效 Regime II 的 ProbedBw 候选。

### 5.3 Regime III

```text
next_baseline = use_midpoint ? midpoint : mindrate
```

### 5.4 边界保护

- `next_baseline = max(next_baseline, pacing.minimum_rate_mbps)`。
- DRate 统计无效、非有限、`mindrate <= 0` 或 `maxdrate < mindrate` 时，本次控制动作降级为不确定，不修改 baseline/ProbedBw/MaxRTT/RTpropDRate。
- `RTpropDRate` 缺失或不满足 `RTpropDRate <= maxdrate` 时，`use_midpoint=false`：Regime I 取 `maxdrate`，Regime III 取 `mindrate`。
- 基线改变后等待 1 SRTT，再从新探针 epoch 采集完整 `2T`；不能把改变前后的样本放进同一个判定窗。
- PDF 没有“最多调整 8 次”的停止规则。对 `kFBBR`，`waveform.max_baseline_adjustments` 不应再改变分类/执行结果，可降为只告警计数；搜索一直持续到 Cruise 结束。这个变化不能作用于 `kFBBRAdaptive` 的现有停止/保护逻辑。

示例：`mindrate=80 Mbps`、`maxdrate=120 Mbps`、`RTpropDRate=60 Mbps`，则 `swing=40`、阈值为 `0.5*(120-60)=30`，命中 midpoint，Regime I/III 都取 100 Mbps。若 `mindrate=100 Mbps`，则 `swing=20 <= 30`，Regime I 取 120 Mbps，Regime III 取 100 Mbps。

## 6. 保持当前 `kFBBR` 的不确定迭代、后探与幅度调整

### 6.1 现有不确定状态机原样保留

这一部分不按此前方案简化，保持当前 `kFBBR` `time_waveform` 的可观测行为。设进入本轮 Cruise 时的初始探针幅度为 `A0`，当前幅度为 `Ak`：

```text
新幅度轮次：等待 1 SRTT -> 采集 C1+C2 -> 判定

第一次不确定：
  baseline 不变，Ak 不变
  只新增 C3
  用 C2+C3 组成新的两周期窗口重判

同一幅度下第二次仍不确定：
  if Ak < 2.0*A0:
      Ak+1 = min(ceil(1.25*Ak), 2.0*A0)
      baseline 不变
      建立新 probe epoch，等待 1 SRTT
      从新的 C1+C2 重新开始
  else:
      幅度保持 2.0*A0
      再后探一个周期
      持续用最新两个周期滚动重判
```

当前实现采集扩展窗时会保留三周期 `C1+C2+C3` 供 trace，但正式第二次分类读取后两个周期 `C2+C3`；`C1+C2` 只作为 prior-window 诊断。达到幅度上限后，每次不确定均把窗口向后推进一个周期，例如 `C2+C3 -> C3+C4`，直到得到确定 Regime 或 Cruise 结束。

需要特别注意当前函数命名容易误导：`UsesAdaptiveLoadJudgment()` 对 `kFBBR` 也返回 true，因为 `fbbr_window_baseline_enabled_=true`；正是这条路径让扩展的三周期采集只分析后两个周期。重构为 V2 专属入口时不能因为删除“Adaptive”命名就丢掉该行为。

必须保留并限定为 `kFBBR` 专属：

- `waveform.max_inconclusive_extensions = 1`：每个新幅度轮次先只补一个周期；
- `waveform.inconclusive_signal_amplification_factor = 1.25`；
- `waveform.inconclusive_signal_amplification_max_ratio = 2.0`，上限相对本轮 Cruise 的 `waveform_initial_probe_amplitude_bps`；
- `AmplifyWaveformProbeAfterInconclusive()` 的“第二次不确定后放大并重新 settle”语义；新实现若改函数名，也必须保持上述状态转换。

“波形放大”和“调整幅度”在当前代码里是同一个动作：修改三角探针的 `current_probe_amplitude_bps`。它不修改 pacing baseline、调制频率或三角波形状。只有从 2 周期窗后探到第 3 周期的过程中，baseline、幅度和 probe epoch 必须保持不变，确保 C2/C3 同属一个实验条件；幅度一旦变化，旧周期立即失效，必须等待 1 SRTT 并重新采集两周期。

### 6.2 新增：连续两窗不波动后的保真增强与自适应幅度调制

这是 13:07 PDF 新增的跨窗口规则，不是 N01-N18 中的新分类叶子。按补充口径，它**不另造一套保真评分或另一套增减幅控制器**，而是把“连续两窗不波动”转换为现有不确定状态机的一个新进入原因：

```text
retry_reason = INVALID_CLASSIFICATION_INPUT
               or TWO_WINDOW_NO_WAVE

TWO_WINDOW_NO_WAVE = (srtt_no_wave_streak >= 2)
                     or (drate_no_wave_streak >= 2)
```

- 对 SRTT、DRate 分别维护 streak；任一信号连续两个有效、已最终化窗口为 `has_wave=false` 就触发，不能收紧为二者都连续两窗不波动。
- 每个不同的 `window_second_cycle_id` 最多计数一次。某信号本窗有波动则其 streak 清零，无波动则加一；对应输入无效时该信号 streak 冻结。
- “连续窗口”按判定时间顺序计算，不要求两个窗口来自相同 baseline；否则第一个确定结果改变 baseline 后会使该脚注事实上无法触发。新 Cruise 开始时才清零两个 streak。
- 若状态机此前空闲，触发的第二个窗口视为“第一次不确定”；若已因输入无效处于重试中，只把 `TWO_WINDOW_NO_WAVE` OR 进 reason mask，不重置扩展/放大阶段。无论哪种情况，该触发窗的 N01-N18 结果只保留 trace，不执行 baseline、ProbedBw、MaxRTT、RTprop 或 RTpropDRate 副作用。

随后完全复用第 6.1 节的节奏，但**每次只向后补一个周期**：

```text
触发窗为 C1+C2：
  baseline、幅度、probe epoch 不变
  只采 C3，再用 C2+C3 复判              # 窗口只前进 1T

若 C2+C3 中 SRTT 或 DRate 任一恢复 has_wave：
  清除 TWO_WINDOW_NO_WAVE 和两个 streak
  if 本窗分类输入有效：
      停止重试，接受 C2+C3 的 N01-N18 结果并执行对应副作用
  else:
      只保留 INVALID_CLASSIFICATION_INPUT，继续原不确定重试

若 C2+C3 仍未恢复，或分类输入仍无效：
  视为同一幅度下第二次不确定
  Ak+1 = min(ceil(1.25*Ak), 2.0*A0)
  baseline 不变；新建 probe epoch；等待 1 SRTT
  在新幅度下重新采集 C1+C2

新幅度下若首窗仍未恢复：
  只补 C3，以 C2+C3 复判；仍未恢复才再次增幅

达到 2.0*A0 后仍未恢复：
  幅度保持不变
  每次只补一个周期，窗口 C2+C3 -> C3+C4 -> C4+C5
```

因此“慢一点滚动”的量化含义是窗口起点和终点每次都只增加 `T`，相邻分析窗保留一个共同周期；严禁 `C1+C2 -> C3+C4` 这样一次跳过 `2T`。只有变幅时因为不能混用不同 probe epoch，才等待 1 SRTT 并重新收集两个全新周期。

退出条件严格按 PDF：进入该模式后，后续有效滑窗中只要 `srtt_has_wave || drate_has_wave` 就清除 `TWO_WINDOW_NO_WAVE`，不要求触发它的那个信号单独恢复，也不要求两个信号同时恢复；退出时把两个 streak 一并清零，要求未来重新积满两窗才能再次触发。退出前 baseline 和所有判定副作用保持冻结；若退出窗分类输入有效，就恢复执行该窗分类；若仍有 `INVALID_CLASSIFICATION_INPUT`，则只按原输入无效重试继续。二者继续无波动时，即使 N16/N17/N18 能给出确定 Regime，也不提前退出。

实现上复用同一组 `inconclusive_extension_count`、`inconclusive_amplification_count`、`current_probe_amplitude_bps` 和滚动窗口缓存，只新增位掩码 `retry_reason_mask`、两个 no-wave streak 与 `wave_fidelity_enhancement_active`。这样既落实 PDF 的保真增强/自适应幅度，又严格保持原 FBBR 的一次后探、第二次放大、1.25 倍增长和 2 倍封顶语义。整条路径只属于 `kFBBR`，`kFBBRAdaptive` 不读取。

## 7. 建议的数据结构与函数拆分

分类器必须是纯函数，特征提取和状态副作用分开：

```cpp
struct ContinuousHorizontalEvidence {
  bool valid;
  bool left_boundary_verified;
  bool right_boundary_verified;
  double start_s;
  double end_s;
  double duration_ratio_of_period;
  double flat_fraction;
  double level_span_ratio;
  double robust_slope;
  double left_context_slope;
  double right_context_slope;
  double bic_linear_minus_constant;
};

struct RepeatedClipLineEvidence {
  bool valid;
  bool is_upper;
  double clip_level;
  uint32_t contact_fragment_count;
  uint32_t contact_sample_count;
  uint8_t contact_cycle_mask;        // bit0=C1, bit1=C2，必须为 0b11
  double contact_sample_ratio;
  double contact_time_span_ratio_of_window;
  double pooled_flat_fraction;
  double contact_level_span_ratio;
  double cross_cycle_level_delta_ratio;
  double event_period_error_ratio;
  double verified_boundary_fraction;
  double extrapolated_overshoot_ratio;
};

struct MiddleSequentialEvidence {
  bool valid;
  double start_s;
  double end_s;
  double duration_ratio_of_period;
  double left_context_slope;
  double right_context_slope;
  double reference_slope;
  double slope_mismatch_ratio;
  double bridge_deviation_ratio;
};

enum class SrttClipCase {
  kNone,
  kU1PositiveShoulder,
  kU2LongTopLine,
  kU3RepeatedTopClip,
  kL1NegativeShoulder,
  kL2LongBottomLine,
  kL3RepeatedBottomClip,
};

struct FbbrSrttClipCases {
  bool suspected_top_candidate;     // 只用于 trace，不能分类
  bool suspected_bottom_candidate;  // 只用于 trace，不能分类
  bool u1_positive_shoulder;
  bool u2_long_top_line;
  bool u3_repeated_top_clip;
  bool l1_negative_shoulder;
  bool l2_long_bottom_line;
  bool l3_repeated_bottom_clip;
  SrttClipCase selected_case;       // 六者均 false 时必须 kNone
};

struct SignalRegimeFeatures {
  bool input_valid;
  bool wave_activity_input_valid;
  bool ordinary_wave_uses_raw_valid_view; // DRate=true，SRTT=false
  bool repeated_top_clip;
  bool repeated_bottom_clip;
  bool long_top_line;
  bool long_bottom_line;
  bool positive_shoulder_clip;
  bool negative_shoulder_clip;
  bool periodic_similarity_input_valid;
  bool periodic_similar;
  bool has_wave;
  bool left_edge_line_masked;
  bool right_edge_line_masked;
  bool middle_sequential_masked;
  bool positive_shoulder_horizontal_verified;
  bool negative_shoulder_horizontal_verified;
  bool positive_shoulder_cycle_input_valid;
  bool negative_shoulder_cycle_input_valid;
  bool positive_shoulder_cycle_recognizable;
  bool negative_shoulder_cycle_recognizable;
  uint32_t continuous_horizontal_count;
  uint32_t repeated_clip_contact_fragment_count;
  uint32_t repeated_clip_contact_sample_count;
  uint8_t repeated_clip_contact_cycle_mask;
  double repeated_clip_contact_time_span_ratio_of_window;
  double longest_top_line_ratio_of_period;
  double longest_bottom_line_ratio_of_period;
  double horizontal_best_flat_fraction;
  double horizontal_best_level_span_ratio;
  double horizontal_best_abs_slope_ratio;
  double repeated_clip_pooled_flat_fraction;
  double repeated_clip_level_span_ratio;
  double repeated_clip_boundary_verified_fraction;
  double repeated_clip_extrapolated_overshoot_ratio;
  double middle_sequential_mask_ratio;
  double middle_best_slope_mismatch_ratio;
  double middle_best_bridge_deviation_ratio;
  double estimated_period_ratio;
  double estimated_srate_period_ratio;
  double response_srate_period_error_ratio;
  double periodicity_correlation;
  double wave_amplitude;
  double wave_noise_sigma;
  double wave_amplitude_to_level_ratio;
  double wave_step_threshold;
  double wave_active_step_ratio;
  double wave_up_change_ratio;
  double wave_down_change_ratio;
  double wave_significant_path_ratio;
  uint32_t wave_slope_reversals;
  uint8_t wave_active_cycle_mask;      // bit0=C1, bit1=C2
  const char* wave_failure_reason;     // NONE/LOW_AMP/SMOOTH_ONLY/ONE_WAY/SPIKE
};

struct FbbrRegimeContext {
  bool max_rtt_valid;
  double max_rtt_ms;
  bool rtprop_valid;
  double rtprop_ms;
};

struct FbbrRegimeDecision {
  WaveformClassification classification;
  const char* rule_id;               // N01..N18；输入无效时为空
  bool update_max_rtt;
  bool refresh_rtprop;
  bool update_rtprop_drate;
};

struct FbbrUnifiedRetryState {
  uint8_t srtt_no_wave_streak;       // 有效最终窗计数，饱和即可
  uint8_t drate_no_wave_streak;
  bool enhancement_active;
  uint64_t last_counted_window_second_cycle_id;
  uint8_t retry_reason_mask;          // bit0=INVALID_INPUT, bit1=TWO_WINDOW_NO_WAVE
};
```

建议函数边界：

1. `DetectHorizontalClipEvidence()`：共享预处理和门限，同时输出连续横切段与重复短接触证据，不判断 Regime。
2. `DetectContinuousHorizontalSegments()`：实现第 2.5.1 节单段长度、平坦度、电平和边界验真，供 U1/U2/L1/L2 与边缘线使用。
3. `DetectRepeatedClipLineContacts()`：实现第 2.5.2 节同电平短片段聚类、`contact_span/W>=0.50`、跨周期复现、聚合边界和反事实越界，供 U3/L3 使用；不得读取 20%/30% 长线门。
4. `DeriveSixSrttClipCases()`：从 SRTT 的两类原始证据派生 U1/U2/U3、L1/L2/L3；不接受兜底 clip。
5. `DetectShoulderClips()`：只对连续横切段做半周期、局部极值、相反肩斜率及“谷-上肩-谷”/“峰-下肩-峰”残余周期骨架验真。
6. `DetectMiddleSequentialDisturbances()`：按第 2.7 节的同向外部趋势、斜率不符合和 bridge deviation 产生独立 mask，不调用横切门。
7. `DetectOrdinaryWaveActivity()`：只实现第 2.4 节三道门，不调用周期检测，也不读取发送波半周期标签；调用者显式传入视图，SRTT 用清理视图、DRate 用原始有效视图。
8. `AnalyzePeriodicSimilarity()`：接收实际 SRate 序列，先估计有效 `T_srate`，再复用现有 `AnalyzeCycleCompleteness()` 实现第 2.3 节的周期性相似判定；只读取上横切硬否决位，不能把下/左右/middle 横切一概否决。
9. `BuildSignalRegimeFeatures()`：按第 2.7 节固定顺序调用上述检测器，并组合 SRTT/DRate 特征，同时保留普通波动视图与 periodic mask 的来源标识。
10. `ClassifyFbbrRegimeV2()`：`kFBBR` 专属，只执行第 3 节有序树，不读写成员变量。
11. `ApplyFbbrV2RegimeStateUpdates()`：`kFBBR` 专属，按最终 rule 执行 MaxRTT、RTprop、RTpropDRate 动作。
12. `ComputeFbbrV2InjectionBaseline()`：`kFBBR` 专属，只实现第 5 节公式。
13. `ApplyFbbrV2RegimeDecision()`：`kFBBR` 专属，更新 pacing baseline/ProbedBw 并安排下一窗口。
14. `UpdateFbbrNoWaveRetryState()`：按最终化窗口和唯一 `window_second_cycle_id` 更新两个 streak；达到两窗时产生 `TWO_WINDOW_NO_WAVE`，并将该窗送入现有不确定状态机。
15. `AdvanceFbbrUnifiedRetryState()`：复用现有阶段；第一次失败保持 baseline/幅度/epoch、只补一个周期并在最新两个周期上复判，第二次失败按 1.25 倍变幅，变幅后统一等待 1 SRTT 并从两个新周期开始，2 倍封顶后继续单周期滚动。

主路径调用顺序固定为：

```text
features = BuildSignalRegimeFeatures(window)  # 两个 has_wave 每窗必算
decision = ClassifyFbbrRegimeV2(features, context)
retry    = UpdateFbbrNoWaveRetryState(features, decision)

if retry.active:
    trace decision but suppress all decision side effects
    AdvanceFbbrUnifiedRetryState()
else if decision is valid:
    ApplyFbbrV2RegimeStateUpdates(decision)
    ApplyFbbrV2RegimeDecision(decision)
else:
    enter/continue existing INVALID_INPUT retry
```

不要继续把新版条件塞进现有 `ClassifyWaveformState()` 的 `adaptive_guard_enabled` 分支。应把 `kFBBR` 新树和执行器单独实现，现有分类/控制函数保留给 `kFBBRAdaptive`。只允许抽取返回信号数值或无状态特征的纯工具；任何会返回 baseline、修改 pacing/ProbedBw、写连接级状态或推进控制状态机的函数都禁止在二者之间复用。

建议在运行入口做一次显式所有权分派，而不是在共享执行函数内部不断判断 Adaptive 开关：

```cpp
if (algorithm_ == kFBBR) {
  RunFbbrV2RegimePipeline();       // PDF N01-N18 + actuator + unified retry
} else if (algorithm_ == kFBBRAdaptive) {
  RunFbbrAdaptivePipeline();       // 保持现有实现
}
```

## 8. 现状差异与具体代码改造点

### 8.1 主要行为差异

| 项目 | 当前 `kFBBR` | 新版要求 |
|---|---|---|
| 分类入口 | 与 Adaptive 共用 `R1-R6` | 独立 N01-N18 有序树 |
| 判定后执行器 | 当前部分 baseline/decision 路径与 Adaptive 交织 | PDF 50% 公式和 Regime II ProbedBw 只允许 `kFBBR` 调用；不确定分支保留当前 FBBR 的后探/放大/滚动语义 |
| 可复用范围 | 共享分类与控制分支 | 只复用重采样/MAD/斜率/相关等无状态信号工具，禁止复用控制副作用 |
| 横切入口 | BIC/plateau/DRate 特征交织 | 只允许 SRTT U1-U3/L1-L3 六种验真形态进入横切子树；连续长线与同电平断续短接触分别提取；DRate 横切不分类 |
| 横切验真失败 | 可能被兜底 clip/plateau 分支吸收 | 无条件回退到 SRTT `has_wave` 主树 |
| 肩部削平 | `DetectDualSignalPlateaus()` 可要求另一信号同步相反肩斜率 | 只有 SRTT 正肩 U1/负肩 L1 进入分类；各自独立通过连续横切段标准，不要求 DRate 同步肩部 |
| 顺位中间削平 | 先要求低斜率水平段，再检查前后同向 | 不要求水平；前后同向趋势中任何超过噪声/幅度门的局部斜率不符合均可遮罩 |
| both-clipped | 当前可进入欠载且提前刷新 RTprop | 严格服从上分支优先级，副作用跟随最终 rule |
| 满载定义 | SRTT 相似为主 | U1/U2 下由 DRate periodic 得到；波动回退树越界条件均未命中时默认 Regime II |
| SRTT 波动回退 | 最近过/欠载窗口 SRTT 均值 `srtt_up/low` | 只比较当前窗口 `srtt_max > MaxRTT`、`srtt_min < RTprop`，不再比较 mean |
| 下横切参考 | 刷新 RTprop | 保持既有刷新，并新增 `RTpropDRate=mindrate` |
| Regime I 基线 | 直接窗口 max DRate | 50% 公式：midpoint 或 max |
| Regime III 基线 | 直接窗口 min DRate | 50% 公式：midpoint 或 min |
| Regime II | 当前已取窗口 mean | baseline 和 ProbedBw 都持续取窗口 mean |
| 不确定 | 先补一周期，第二次不确定后放大探针 | 原样保留：每个幅度先后探 1 周期，再按 1.25 倍放大，封顶 2 倍后逐周期滚动 |
| 连续无普通波动 | 无独立触发条件 | SRTT 或 DRate 任一连续两窗 `has_wave=false` 后转入原不确定状态机；每次只补 1 周期、最新两周期复判，任一恢复波动即退出 |
| 跨 Cruise | srtt/baseline bounds 受 25% MaxBw 门控 | MaxRTT、RTpropDRate 无条件保留 |
| DRate 统计 | ACK 事件样本直接 min/max/mean | 等时间隔有效序列 min/max/time-mean |

### 8.2 文件级落点

| 文件 | 计划修改 |
|---|---|
| `NS3.27/src/dqc/model/thirdparty/congestion/fbbr_sender.h` | 新特征、决策、MaxRTT/RTpropDRate 状态、两个 no-wave streak、保真增强状态和配置字段；声明彼此独立的 `kFBBR` V2 与 Adaptive 分类/执行接口 |
| `NS3.27/src/dqc/model/thirdparty/congestion/fbbr_sender.cc` | 将 `DetectDualSignalPlateaus()` 拆为连续横切段、重复短接触、肩部派生和 middle disturbance；新增 `RunFbbrV2RegimePipeline()` 专属 Analyze/Classify/Apply 路径；实现 50% 公式、DRate 原始视图普通波动、上横切 periodic 硬否决，并让输入无效与连续两窗无波动共用当前 FBBR 的单周期后探/放大转换；不得修改 `RunFbbrAdaptivePipeline()` 的控制行为 |
| `NS3.27/examples/CCconfig/fbbr_default.conf` | 增加/启用量化阈值；保留并固定验证不确定扩展次数、幅度放大倍数和幅度上限；仅将已失效的旧 delta/bounds 参数标为 inactive |
| `NS3.27/scratch/generic_p2p_switch_flows.cc` | 解析新增配置键 |
| `NS3.27/scratch/fbbr_4flow.cc` | 同步解析新增配置键和 CLI 映射，避免两个入口配置不一致 |
| `NS3.27/src/dqc/model/dqc_trace.cc` | 更新 cruise-load CSV 表头 |
| `FreqCCv4_Complete_Method.md` | 实现后更新当前 FBBR/Adaptive 行为说明，避免文档继续描述旧 R1-R6 |

建议新增/明确的配置默认值：

```ini
# 仅属于 kFBBR/PDF 判定与执行；kFBBRAdaptive 禁止读取
fbbr.regime.long_top_horizontal_duration_ratio = 0.20
fbbr.regime.long_bottom_horizontal_duration_ratio = 0.30
fbbr.regime.actuator.midpoint_trigger_ratio = 0.50

# 保持当前 kFBBR 不确定迭代语义；Adaptive 对这些现有键的行为冻结，不随本方案改变
waveform.initial_settle_rtt_mult = 1.0
waveform.post_adjust_settle_rtt_mult = 1.0
waveform.initial_window_periods = 2.0
waveform.extended_window_periods = 3.0
waveform.max_window_periods = 3.0
waveform.max_inconclusive_extensions = 1
waveform.inconclusive_signal_amplification_factor = 1.25
waveform.inconclusive_signal_amplification_max_ratio = 2.0

# PDF 新增的跨窗保真增强：只新增触发/退出/滑窗步长，幅度复用上面的不确定参数
fbbr.wave_fidelity.no_wave_trigger_windows = 2
fbbr.wave_fidelity.stop_on_either_wave = true
fbbr.wave_fidelity.retry_window_advance_periods = 1

# 普通“波动”专用：与 periodic_similar 配置完全分离
waveform.activity.amplitude_noise_multiplier = 6.0
waveform.activity.min_level_ratio = 0.02
waveform.activity.step_noise_multiplier = 3.0
waveform.activity.min_normalized_step_slope = 3.5
waveform.activity.min_active_steps = 4
waveform.activity.min_active_step_ratio = 0.10
waveform.activity.min_directional_change_ratio = 0.20
waveform.activity.min_significant_path_ratio = 0.80
waveform.activity.min_slope_reversals = 1

# 共享横切基础门；连续段和重复短接触共用斜率/电平/边界定义
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

# 共享横线的派生特征
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
waveform.horizontal.extreme_distance_ratio = 0.10
waveform.shoulder.min_half_overlap_ratio = 0.75
waveform.shoulder.min_side_change_ratio = 0.15
waveform.shoulder.max_residual_cycle_period_error_ratio = 0.20
waveform.shoulder.min_residual_cycle_leg_duration_ratio = 0.15

# 顺位中间削平：与 horizontal/shoulder 配置完全分离
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

# 周期性相似波动
fbbr.regime.period_tolerance_ratio = 0.20
fbbr.regime.min_periodicity_correlation = 0.50
fbbr.regime.periodic_upper_clip_is_hard_veto = true
waveform.min_cycle_coverage_ratio = 0.85
waveform.masked_min_cycle_coverage_ratio = 0.50
```

上横线 20%、下横线 30%、周期误差 20% 和 midpoint 50% 可进入配置以便受控 A/B；多段横切 `contact_span/W>=0.50` 是本规则的硬边界。正式回归必须固定验证五个值分别为 0.20、0.30、0.20、0.50、0.50，不能在生产配置中静默放宽 span 门。

现有 `waveform.plateau_*` 和 `waveform.clip_*` 不能再由一个 `DetectDualSignalPlateaus()` 同时解释两种机制。实现迁移期可兼容解析旧键，但正式 `kFBBR` 决策只能读取上面分离后的 `horizontal.*`、`shoulder.*` 和 `middle.*`；`kFBBRAdaptive` 继续使用旧键，避免行为被连带修改。`no_wave_trigger_windows=2`、“任一恢复即停止”和 `retry_window_advance_periods=1` 都是 PDF 加补充口径后的硬语义，不建议在生产中可调；实际增幅仍只读取现有 1.25/2.0 参数。

## 9. Trace 与可观测性

每个分析窗口至少输出以下新字段，否则很难证明实现真的走了 PDF 分支：

```text
algorithm_mode,regime_pipeline_owner,regime_actuator_owner,
regime_rule_id,regime,
srtt_suspected_top_candidate,srtt_suspected_bottom_candidate,
srtt_u1_positive_shoulder,srtt_u2_long_top_line,srtt_u3_repeated_top_clip,
srtt_l1_negative_shoulder,srtt_l2_long_bottom_line,srtt_l3_repeated_bottom_clip,
srtt_selected_clip_case,both_clip_directions,
clip_candidate_rejected_to_wave_fallback,fallback_entered,
srtt_upper_clip_periodic_veto,drate_upper_clip_periodic_veto,
srtt_lower_clip_ignored_for_periodic,drate_lower_clip_ignored_for_periodic,
srtt_wave_input_view,drate_wave_input_view,
srtt_repeated_top_clip,srtt_repeated_bottom_clip,
srtt_top_contact_fragment_count,srtt_bottom_contact_fragment_count,
srtt_top_contact_sample_count,srtt_bottom_contact_sample_count,
srtt_top_contact_cycle_mask,srtt_bottom_contact_cycle_mask,
srtt_top_contact_span_ratio_of_window,srtt_bottom_contact_span_ratio_of_window,
srtt_top_contact_level_ms,srtt_bottom_contact_level_ms,
srtt_top_contact_level_span_ratio,srtt_bottom_contact_level_span_ratio,
srtt_top_contact_pooled_flat_fraction,srtt_bottom_contact_pooled_flat_fraction,
srtt_top_contact_boundary_verified_fraction,srtt_bottom_contact_boundary_verified_fraction,
srtt_top_contact_extrapolated_overshoot_ratio,srtt_bottom_contact_extrapolated_overshoot_ratio,
drate_top_contact_fragment_count,drate_bottom_contact_fragment_count,
drate_top_contact_sample_count,drate_bottom_contact_sample_count,
drate_top_contact_cycle_mask,drate_bottom_contact_cycle_mask,
drate_top_contact_span_ratio_of_window,drate_bottom_contact_span_ratio_of_window,
srtt_long_top_line_ratio,srtt_long_bottom_line_ratio,
srtt_positive_shoulder,srtt_negative_shoulder,
srtt_positive_shoulder_cycle_input_valid,
srtt_negative_shoulder_cycle_input_valid,
srtt_positive_shoulder_cycle_recognizable,
srtt_negative_shoulder_cycle_recognizable,
srtt_continuous_horizontal_count,drate_continuous_horizontal_count,
srtt_shoulder_horizontal_verified,drate_shoulder_horizontal_verified,
srtt_horizontal_flat_fraction,drate_horizontal_flat_fraction,
srtt_horizontal_level_span_ratio,drate_horizontal_level_span_ratio,
srtt_horizontal_abs_slope_ratio,drate_horizontal_abs_slope_ratio,
srtt_horizontal_left_boundary_verified,drate_horizontal_left_boundary_verified,
srtt_horizontal_right_boundary_verified,drate_horizontal_right_boundary_verified,
srtt_middle_slope_mismatch_ratio,drate_middle_slope_mismatch_ratio,
srtt_middle_bridge_deviation_ratio,drate_middle_bridge_deviation_ratio,
srtt_has_wave,drate_has_wave,
srtt_wave_failure_reason,drate_wave_failure_reason,
srtt_wave_amplitude,drate_wave_amplitude,
srtt_wave_noise_sigma,drate_wave_noise_sigma,
srtt_wave_step_threshold,drate_wave_step_threshold,
srtt_wave_active_step_ratio,drate_wave_active_step_ratio,
srtt_wave_up_change_ratio,srtt_wave_down_change_ratio,
drate_wave_up_change_ratio,drate_wave_down_change_ratio,
srtt_wave_significant_path_ratio,drate_wave_significant_path_ratio,
srtt_wave_slope_reversals,drate_wave_slope_reversals,
srtt_wave_active_cycle_mask,drate_wave_active_cycle_mask,
srtt_periodic_input_valid,drate_periodic_input_valid,
srtt_periodic_similar,drate_periodic_similar,
estimated_srate_period_s,
srtt_estimated_period_s,drate_estimated_period_s,
srtt_srate_period_error_ratio,drate_srate_period_error_ratio,
srtt_edge_mask_ratio,drate_edge_mask_ratio,
srtt_middle_mask_ratio,drate_middle_mask_ratio,
max_rtt_valid,max_rtt_before_ms,max_rtt_after_ms,
rtprop_before_ms,rtprop_after_ms,
rtprop_drate_valid,rtprop_drate_before_bps,rtprop_drate_after_bps,
mindrate_bps,maxdrate_bps,meandrate_bps,
swing_bps,reference_gap_bps,midpoint_triggered,
baseline_before_bps,baseline_after_bps,
probed_bw_before_bps,probed_bw_after_bps,
window_first_cycle_id,window_second_cycle_id,
collection_window_periods,analysis_uses_later_two_cycles,
prior_window_rule_id,prior_window_regime,
inconclusive_extension_count,inconclusive_amplification_count,
initial_probe_amplitude_bps,current_probe_amplitude_bps,
inconclusive_amplitude_cap_bps,rolling_retry_count,
srtt_no_wave_streak,drate_no_wave_streak,
wave_fidelity_enhancement_active,wave_fidelity_just_entered,
retry_reason_mask,no_wave_triggered,
classification_suppressed_for_retry,state_updates_suppressed_for_retry,
retry_window_advance_periods,retry_window_stride_cycles,
amplitude_before_bps,amplitude_after_bps
```

还应增加运行时不变量告警：

- `algorithm_mode != kFBBR` 却出现 `regime_rule_id=N01..N18`、`regime_pipeline_owner=fbbr_v2`、`regime_actuator_owner=fbbr_v2` 或保真增强状态；
- `kFBBRAdaptive` 窗口触发第 5 节 midpoint/max/min/mean 公式、PDF ProbedBw 更新、MaxRTT/RTpropDRate 更新或第 6.2 节两窗无波动重试；Adaptive 自己原有的不确定重试行为不计为违规，但不得消费 N01-N18 的结果推进 `kFBBR` V2 状态机；
- `periodic_similar=true` 且同一信号存在已验真的**上**肩、连续上横线或重复上横切；反过来，只有下横切时把 periodic 无条件置 false 也应告警；
- `periodic_similar=true` 但 `periodic_similarity_input_valid=false`、`T_srate` 无效，或 `abs(T_hat-T_srate)/T_srate > 0.20`；
- `shoulder_clip=true` 但 `continuous_horizontal=false`、任一可观测横切边界未验真、半周期极值门失败，或对应的肩部残余周期骨架输入无效/不可识别；
- 同一个区间既被保留为 shoulder 又进入 middle mask；
- `middle_sequential=true` 但外部斜率不同向、`K_mismatch < max(2,ceil(0.25*M_valid))`、最长连续异常少于 2，或 bridge deviation 未通过；
- 单周期 middle mask 超过 `0.35T`，或 mask 删除了原时间轴而不是只改变 valid 位；
- `has_wave=true` 但幅度门、active-step 门或 return 门任一为 false；
- DRate 普通 `has_wave` 使用了 edge/middle/clip mask，或因任意横切证据被直接置 false；SRTT 普通 `has_wave` 未使用规定的清理视图；
- 普通波动特征计算读取了发送波正/负半周期标签，或把 `periodic_similar` 当成前置条件；
- 仅有 `suspected_top/bottom_candidate=true`、六种 accepted case 均为 false，却没有进入普通波动回退树；
- U3/L3 的 contact cycle mask 不是 `0b11`、任一周期 contact 样本少于 2、总数/5% 证据量门失败、`contact_span/W < 0.50`、片段电平不一致、聚合近水平步长不足、边界通过率低于 75% 或反事实越界失败，却仍被设为 true；
- U3/L3 读取了 `continuous_min_duration_ratio`、20%/30% 长线门，或因为最长/累计横切时长低而被直接拒绝；
- DRate 横切证据直接产生 rule、Regime 或 MaxRTT/RTprop/RTpropDRate 副作用；
- rule 为 N02/N04/N05 但窗口 SRTT max 无效；
- rule 为 N06/N10/N11 但 mindrate 无效；
- U2 在 `L_top <= 0.20T` 时为 true，或 L2 在 `L_bottom <= 0.30T` 时为 true；
- 上下 accepted case 同时存在却没有按上横切优先，或上方向仅为疑似候选时阻断了有效下横切；
- baseline 改变后的窗口包含改变前样本；
- 第一次不确定就改变探针幅度或 baseline，或者补采后的正式分析窗口不是严格由前一窗口 C2 和新 C3 组成；
- 同一幅度下第二次不确定、`Ak < 2*A0`，却没有按 `min(ceil(1.25*Ak),2*A0)` 更新幅度并等待 1 SRTT；
- 幅度改变后仍复用旧幅度周期，或 `Ak > 2*A0`；
- 达到 `2*A0` 后继续尝试增幅，或没有保持幅度并逐周期滚动最新两周期；
- 不确定期间 pacing baseline、调制频率或三角波形状发生变化；
- SRTT 或 DRate 任一 no-wave streak 到 2 后没有以 `TWO_WINDOW_NO_WAVE` 进入现有不确定状态机，或把 PDF 的“或”错误实现为二者都到 2；
- 输入无效的信号错误改变自身 no-wave streak、相同 `window_second_cycle_id` 被重复计数，或正常 baseline 变化错误清零 streak；
- 触发窗仍执行了其 N01-N18 的 baseline/ProbedBw/MaxRTT/RTprop/RTpropDRate 副作用；
- 增强模式的相邻重试窗不是 `C1+C2 -> C2+C3 -> C3+C4` 这样每次只前进一个周期，或在未变幅时跳成不重叠的 `C1+C2 -> C3+C4`；
- 同一幅度下第一次单周期后探仍无恢复却没有按原不确定逻辑增幅，或幅度越出 `[A0,2.00A0]`；
- 幅度改变后未新建 epoch/等待 1 SRTT，或保真增强修改了 baseline、ProbedBw、调制频率及分类状态；
- 增强后任一信号恢复波动仍未退出，或退出窗没有恢复执行其正式分类与副作用；
- 新 Cruise 未清零 no-wave streak/增强状态，或 `kFBBRAdaptive` 读取、写入这些状态。

## 10. 测试与验收矩阵

### 10.1 纯分类单元测试

对 N01-N18 每个叶子至少构造一个 `SignalRegimeFeatures` 用例，断言最终 Regime、精确 `rule_id`、MaxRTT/RTprop/RTpropDRate 副作用位，以及更低优先级的冲突特征不会改变结果。最小叶子矩阵如下：

| 输入 | 期望 rule/结果 | 期望状态动作 |
|---|---|---|
| U1 + DRate periodic | N01 / II | 无 |
| U1 + DRate 有效非 periodic | N02 / III | 仅 MaxRTT |
| U2 + DRate periodic | N03 / II | 无 |
| U2 + DRate 有效非 periodic | N04 / III | 仅 MaxRTT |
| U3 | N05 / III | 仅 MaxRTT |
| L1 + DRate has_wave | N06 / I | RTprop + RTpropDRate |
| L1 + DRate 无普通波动；max 越界 | N07 / III | 无 |
| L1 + DRate 无普通波动；max 未越界、min 越界 | N08 / I | 无 |
| L1 + DRate 无普通波动；max/min 均未越界 | N09 / II | 无 |
| L2 | N10 / I | RTprop + RTpropDRate |
| L3 | N11 / I | RTprop + RTpropDRate |
| 无 accepted clip；SRTT has_wave；max 越界 | N12 / III | 无 |
| 同上；max 未越界；min 越界 | N13 / I | 无 |
| 同上；max/min 均未越界 | N14 / II | 无 |
| 无 accepted clip；SRTT 无波动；DRate has_wave | N15 / I | 无 |
| SRTT/DRate 均无波动；max 越界 | N16 / III | 无 |
| 同上；max 未越界；min 越界 | N17 / I | 无 |
| 同上；max/min 均未越界 | N18 / II | 无 |

必须单列以下优先级用例：

1. U1/U2/U3 与 L1/L2/L3 同时有效时，上方向优先；上方向内部 U1 > U2 > U3，下方向内部 L1 > L2 > L3。
2. 只有疑似上横切、U1-U3 均失败，而 L1/L2/L3 有效时，必须进入下横切；疑似上横切不能抢占优先级。
3. 疑似上或下横切存在但六种 case 均失败时，必须根据 SRTT `has_wave` 进入 N12-N18，不能停在横切子树或兜底成 U3/L3。
4. 只有 DRate 上/下横切时，不能直接产生任何 rule 或状态更新；若 SRTT 六种 case 均失败，仍走普通波动回退树。
5. U1/U2 所需 DRate periodic 输入无效时返回不确定，不能按“非 periodic”进入 N02/N04；L1 所需 DRate wave 输入无效时同样不确定。
6. U2 长度恰为 `0.20T` 不命中、`0.2001T` 命中；L2 恰为 `0.30T` 不命中、`0.3001T` 命中。
7. U3/L3 必须在两个周期都形成同电平短 contact 集合，首末 contact 时间跨度至少为 `0.50W`，并通过最小接触样本、聚合平坦步长、边界、周期位置和反事实越界门；仅仅 U1/U2、L1/L2 失败时不得命中。
8. L1+DRate 无波动以及普通回退树都要单测严格比较：SRTT max 恰等于 MaxRTT 不命中 N07/N12/N16，略大才命中；SRTT min 恰等于 RTprop 不命中 N08/N13/N17，略小才命中；两个比较均不成立时分别为 N09/N14/N18。
9. DRate 自身存在已验真**上**横切时，即使周期相关性高也必须 `periodic_similar=false`；只有下横切时不得作为硬否决，左右边缘和 middle 经允许处理后仍可通过 periodic。无论哪种 DRate 横切都不能直接分类。

### 10.2 特征检测单元测试

每种信号至少覆盖：

- 平滑正弦、平滑三角波、非三角但同周期波：`periodic_similar` 按原规则通过；其中平滑正弦/三角的 `has_wave=false`，证明两个谓词没有混用；
- 响应相对实际 SRate 的周期误差 19.9% 和恰好 20% 通过，20.1% 失败；
- 实际 `T_srate` 与名义 `T` 略有偏差时必须以 `T_srate` 为分母；不能用名义周期把本应通过的响应误判失败；
- SRate 周期无效、被 pacing floor 长时间截断或只有单侧波形时，`periodic_similarity_input_valid=false` 且原因是 `INVALID_SRATE_PERIOD`；任何需要区分 periodic/非 periodic 的分支必须不确定，不能按“非 periodic”继续分类；
- 自相关 0.50 通过，0.49 失败；
- 非周期、正负不对称但含多次陡峭往返的抖动：`has_wave=true`、`periodic_similar=false`；
- 对同一段 DRate 构造上横切、下横切、左右边缘线和 middle：普通 `drate_has_wave` 始终读取原始有效视图，只要幅度/active-step/往返三门成立就保持 true；不得因为存在横切或 periodic=false 而变 false；
- 对 SRTT 构造左右边缘线或 middle，验证普通 `srtt_has_wave` 读取清理视图；再对 DRate 使用同样信号，验证其普通波动视图未套用同一 mask；
- 构造直接命中 U3、L2、L3 的窗口，仍必须得到 SRTT/DRate 两个普通 `has_wave` 及有效位，并能推进第 6.2 节 streak；分类短路不得省略跨窗触发所需特征；
- 单向陡坡：即使幅度和 active step 数通过，也因缺少反向累计量/反转而 `has_wave=false`；
- 大幅但平滑的单次上升下降：幅度门通过、陡度门失败，`wave_failure_reason=SMOOTH_ONLY`；
- 单点大尖峰：`P95-P05` 或最少 4 个 active step 门失败；
- 高频小噪声：`6*sigma_x` 幅度门或 `3*sqrt(2)*sigma_x` 步长门失败；
- `up_change/down_change` 可以明显不相等，只要两者分别不少于 `0.20*A_pp` 即通过，证明没有对称性要求；
- `g_i=3.5` 计为 active，略低于 3.5 不计；active step 数恰为 4 通过、3 失败；
- 平滑圆顶/圆底不能误判真实水平线；
- 平滑圆顶/圆底即使区间平均斜率接近零，也必须因 `flat_fraction` 或横切边界失败而不能成为真横线；
- 缓慢进入/离开的近水平段必须因横切边界失败，不能成为肩部；
- 量化三角拐点即使出现连续相等样本，也不能误判肩部削平；
- 真正硬削平的平台必须同时通过 90% 近水平、电平跨度、总漂移、常数模型和两个横切边界；任一门失败都不能设置 shoulder；
- U1 必须在上肩两侧形成“谷-肩-谷”、L1 必须形成“峰-肩-峰”，外侧同类极值间隔误差恰为 20% 通过、20.1% 失败，任一上升/下降段短于 `0.15T` 或幅度门失败都不能成为肩部叶子；
- 肩部水平/相位候选成立但残余周期输入无效时，不能把 INVALID 当成 false 后进入结果不同的 U3/L3/普通回退；应返回不确定。另构造结果完全等价的独立 U2 证据，验证允许按 U2 继续并保留 U1-invalid trace；
- 肩部残余周期骨架可识别不等于 `periodic_similar=true`：U1 的 SRTT 仍因上横切硬否决 periodic，L1 则继续按正式 periodic 的其他门独立计算；
- SRTT 肩部可以在 DRate 没有同步相反肩斜率时独立成立，反之亦然；
- 只有半周期振幅变小或只剩另一半周期，但没有真横线时，`shoulder_clip=false`；
- 两周期同电平真上/下横切能被 SRTT 和 DRate 底层通用检测器识别；只有 SRTT 证据可以派生 U1-U3/L1-L3，DRate 证据只参与自身特征，其中只有上横切硬否决 periodic；
- 只有一个周期出现局部硬平台或短 contact 时，绝不能成为 U3/L3；只有同时满足半周期对齐、局部极值和肩部边界时才可成为 U1/L1，否则是未验真候选并回退；
- 真上平台长度恰为 `0.20T` 和真下平台长度恰为 `0.30T` 分别不能成为 U2/L2，略大于各自阈值才通过；
- 构造最长单段仅 `0.03T-0.08T`、累计占比低于 20%/30%、两个周期都在同一电平出现多个短 contact、首末跨度 `>=0.50W` 的波形，必须能命中 U3/L3；证明一般横切没有误用长线门；
- 保持其他证据完全相同，`contact_span=0.4999W` 必须失败并回退，恰好 `0.50W` 必须通过 span 门；该边界使用 `>=`，不能误写为 `>`；
- 将上述短 contact 拆成断续片段且保留足够内部平坦步长时仍须命中；小缺口不计入横切时长，超过 `0.05T` 的缺失区间不得合并；
- U3/L3 的 contact 集合只出现一个周期、每周期只有一个点、首末跨度小于 `0.50W`、跨周期电平差超门、聚合平坦度不足、边界不足或侧翼外推不越界时，整个 repeated case 失败并回退；
- 两周期自然圆顶/圆底即使极值高度接近，也必须因缺少聚合平坦/横切边界或反事实越界证据而不能成为 U3/L3；
- 边缘横线遮罩后时间轴长度不变；
- SRTT/DRate 的顺位中间 mask 相互独立；
- 同向上升趋势中间出现水平驻留、明显减速、短暂反向或折线偏离时，即使不是水平线，也能成为顺位中间削平；
- 同向下降趋势具有上述四种内部异常时得到相同结论，不能只支持上升方向；
- 天然峰/谷因左右上下文斜率反向，不能成为顺位中间削平；
- 平滑单调曲线因没有成对局部 slope-change 边界，不能成为顺位中间削平；
- 中间偏离小于 `3*sigma_x` 或 `0.05*A_pp` 时不遮罩；恰等门限通过，略低失败；
- `M_valid=14` 时连续 4 个 mismatch 通过、3 个失败；`M_valid=8` 时连续 2 个通过；
- mismatch 总数达到 25% 但全部孤立不连续时失败，证明降低样本占比后不会接收散点噪声；
- shoulder 与 middle 候选重叠时必须保留 shoulder，middle mask 不得覆盖该区间；
- 单周期多个 middle 候选总长度超过 `0.35T` 时，只保留最高分非重叠候选且总 mask 不越界；
- 信号 `A_pp=6*sigma_x` 通过普通波动的幅度门限，略低失败；
- 任一已验真的**上**肩、连续上横线或重复上横切存在时，同一信号的 periodic 标志被否决；对应的下肩、连续下横线、重复下横切单独存在时不得硬否决，只由遮罩后覆盖率/完整周期/周期误差/相关性决定结果；
- 左右边缘线与顺位中间削平经遮罩或简化处理后，若剩余覆盖率和完整周期均通过，`periodic_similar` 仍应为 true。

### 10.3 执行器单元测试

以下用例全部以 `algorithm_mode=kFBBR` 运行：

| 场景 | 期望 |
|---|---|
| `swing > 0.5*reference_gap`, Regime I | midpoint |
| `swing == 0.5*reference_gap`, Regime I | maxdrate |
| `swing > 0.5*reference_gap`, Regime III | midpoint |
| `swing == 0.5*reference_gap`, Regime III | mindrate |
| RTpropDRate 无效, Regime I | maxdrate |
| RTpropDRate 无效, Regime III | mindrate |
| Regime II | baseline=mean 且 ProbedBw=mean |
| 结果低于 pacing floor | clamp 到 floor |

还必须增加执行器所有权隔离测试：

1. 对完全相同的合成 `SignalRegimeFeatures`，`kFBBR` 可以进入 N01-N18 并调用 PDF 执行器；`kFBBRAdaptive` 不得调用分类器或执行器。
2. 在 `kFBBRAdaptive` 下预置可命中 Regime I/II/III 的输入，断言 `MaxRTT`、`RTpropDRate` 和 PDF ProbedBw 均无任何变化；其原有 extension/amplification 计数只做行为冻结对比，不要求为 0。
3. 对 Adaptive 现有 baseline step、delta/queue guard、确认计数和停止条件做修改前后 golden trace 对比，要求逐窗口输出一致。
4. 任何共享工具函数只能返回样本/特征；测试中不得观察到 pacing baseline、ProbedBw 或连接级状态副作用。

### 10.4 状态机测试

- 在幅度 `A0` 下首窗必须是 C1+C2；第一次不确定后只采 C3，正式重判必须使用 C2+C3，期间 baseline、幅度和 probe epoch 不变。
- 同一幅度下 C2+C3 仍不确定时，幅度从 `A0` 变为 `1.25A0`；等待 1 SRTT，旧周期全部失效，再从新 C1+C2 开始。
- 连续保持不确定时，幅度序列必须为 `A0 -> 1.25A0 -> 1.5625A0 -> 1.953125A0 -> 2.0A0`，每个幅度只在第二次不确定后增长，且不得超过 `2.0A0`；整数 bps 按 `ceil` 后再 clamp。
- 到达 `2.0A0` 后不再改变幅度；继续不确定时逐周期后探，分析窗严格滚动为 C2+C3、C3+C4、C4+C5。
- 幅度变化只修改 `current_probe_amplitude_bps`；全过程 baseline、调制频率和三角波形状不因“不确定”而改变。
- 任一轮得到确定 Regime 并改变 baseline 后，旧滚动窗口失效，等待 1 SRTT 后以当前幅度重新从 C1+C2 开始。
- 新 Cruise 开始时 baseline 取当时 MaxBw，探针幅度重新初始化为该轮 `A0`，MaxRTT/RTpropDRate 保留，ProbedBw 本轮新鲜度清零。
- 本轮无 Regime II 时不得发布新的 FBBR ProbedBw。
- 连续有效窗依次为 `(SRTT无波动, DRate有波动)`、`(SRTT无波动, DRate有波动)` 时，第二窗结束后必须因 SRTT streak=2 进入增强；证明触发条件是“或”而不是“且”。增强至少实际运行一个新窗，不能被触发窗中的 DRate 波动同窗关闭。
- 对称地，只有 DRate 连续两窗无波动也必须触发；两信号各自交替无波动、任一 streak 始终不到 2 时不得触发。
- 每个信号的输入有效性独立控制自己的 streak：输入无效只冻结对应信号；相同 `window_second_cycle_id` 的重复判定不能二次累计。baseline 在两个普通窗口之间改变也不能清零 streak，新 Cruise 才清零。
- 触发的第二窗无论得到 N01-N18 中哪一个结果（包括“SRTT 无波动、DRate 有波动”的 N15），都必须设置 `classification_suppressed_for_retry=true`，其 baseline/ProbedBw/MaxRTT/RTprop/RTpropDRate 全部不变。
- 触发窗 C1+C2 后只能新采 C3，并以 C2+C3 复判；不得等待并重新采 C3+C4，也不得一次前进两个周期。C2+C3 中任一信号有波动时清除 `TWO_WINDOW_NO_WAVE` 并把两个 streak 归零；分类输入有效时执行该窗正式分类，仍无效时仅以 `INVALID_INPUT` 继续重试。
- 若 C2+C3 二者仍无波动或分类输入仍无效，才按“同幅度第二次不确定”把 `A0` 增至 `1.25A0`；变幅只产生一个新 epoch 和一次 1 SRTT settle。
- 新幅度首窗 C1+C2 仍无恢复时，必须先补 C3、用 C2+C3 再确认，不能每个两周期窗都直接增幅；连续失败的幅度序列及 2 倍上限与原不确定逻辑完全相同。
- 达到 `2.0A0` 后，分析窗严格 `C1+C2 -> C2+C3 -> C3+C4 -> C4+C5`，每次只增加一个周期；任一窗恢复波动即退出。
- 原输入无效不确定与 `TWO_WINDOW_NO_WAVE` 同时存在时只推进同一个重试状态机一次，不能重复补周期或重复 1.25 增幅。
- 新 Cruise 同时清零 streak、增强状态和 retry reason；`kFBBRAdaptive` 的对应字段始终不变。

### 10.5 仿真验收

建议分三层：

1. **确定性合成 trace 回放**：为 N01-N18 生成 SRTT/DRate 两周期数据，要求 rule_id 100% 命中预期；另生成疑似上/下横切但不满足六种 case 的数据，要求准确进入波动回退树。
2. **单流受控场景**：固定容量/RTT，分别制造欠载、满载、过载、上/下限截断和 ACK 聚合；检查 baseline 公式、MaxRTT/RTpropDRate 更新、普通波动输入视图区别、保真增强和滚动窗。
3. **正式矩阵回归**：复用现有 FBBR 多流、多 seed、动态带宽/RTT、随机丢包/ACK 不稳定脚本。至少 10 seeds；`kFBBR` 除吞吐、RTT、公平性外还统计各 rule 命中率、六种横切验真率、横切候选回退率、不确定连续长度、两窗无波动触发/退出次数、单周期后探次数、增幅次数和错误不变量计数；`kFBBRAdaptive` 做行为冻结回归，不统计/消费 N01-N18。

通过门槛建议：

- N01-N18 单元测试和公式边界测试 100% 通过；
- 合成 trace rule_id 准确率 100%；
- 运行时不变量违规数为 0；
- 任意确定 Regime 窗口的 baseline 实际值与公式值误差不超过 1 bps（浮点转整数前允许 `1e-9` 相对误差）；
- Cruise 跨代状态测试中 MaxRTT/RTpropDRate 继承率 100%；
- 第一次不确定后的单周期补采期间幅度变化次数为 0；同一幅度第二次不确定后的幅度序列、1.25 倍增长、2.0 倍封顶和 settle 次数与第 6 节完全一致；
- 连续两窗按“或”触发、触发窗副作用冻结、每次严格只前进一个周期、任一恢复即退出、第二次仍失败才增幅及 epoch 隔离测试 100% 通过；
- `kFBBRAdaptive` 的 N01-N18 命中数、PDF 执行器调用数、PDF 状态写入数和保真增强状态变化数全部为 0，且 golden trace 无非预期变化。

## 11. 推荐实施顺序

1. 先新增纯数据结构、N01-N18 纯分类器和完整叶子测试，不接控制路径。
2. 把 `DetectDualSignalPlateaus()` 拆成连续横切段、重复短接触、肩部派生和宽泛 middle disturbance；先补齐第 2.5-2.7 节特征测试，再接分类树。
3. 在 `AnalyzeWaveformWindow()` 旁建立 `kFBBR` 专用分析入口，保留 Adaptive 原路径作行为对照。
4. 只在 `RunFbbrV2RegimePipeline()` 接入 MaxRTT/RTpropDRate 生命周期和最终 rule 后的副作用。
5. 替换 `kFBBR` 基线执行器为 50% 公式，接入 Regime II ProbedBw。
6. 将新版分类输入无效结果接入 `kFBBR` 现有后探/放大状态机，保持“一次扩展、第二次放大、2 倍封顶后滚动”的行为；Adaptive 原状态机不动。
7. 实现第 6.2 节 no-wave streak 和统一 retry reason，把连续两窗无波动接入原不确定状态机；先用确定性 trace 验证触发窗副作用冻结、单周期滑窗、退出、增幅与 epoch 隔离。
8. 扩充 trace，完成合成回放、单流受控和多流多 seed 回归。
9. 验证通过后清理 `kFBBR` 已失效的旧 R1-R6、adaptive bounds、delta/queue guard 参数；不要提前删除 Adaptive 仍在使用的字段。

## 12. 完成定义

只有同时满足以下条件，才能认为“完全按照新版 PDF 改造”完成：

- `kFBBR` 运行时确定性分类只可能返回 N01-N18，且分支顺序与第 3 节一致；只有输入无效、覆盖不足或必须谓词无法计算时返回不确定，N01-N18 本身没有不确定叶子；
- 横切分类只读取 SRTT：上横切只允许 U1/U2/U3，下横切只允许 L1/L2/L3；DRate 横切不得直接决定 Regime；
- 六种 SRTT 横切 case 均未验真时，无论是否存在疑似候选，都准确回退到 SRTT `has_wave` 分支；U3/L3 绝不允许充当失败兜底；
- 上下 accepted case 同时存在时上方向优先；上方向只有疑似候选、下方向存在 accepted case 时允许下方向分类；
- 长横线、边缘线和肩部使用连续横切段；U3/L3 使用同一横切机制下的重复短接触聚合，允许实际横切时长低占比和断续，但首末片段时间跨度必须 `>=0.50W`，并共享电平、平坦步长、侧翼及边界标准；
- 肩部削平一定有可追踪的真实水平段、两个横切边界和半周期局部极值证据；
- 顺位中间削平使用独立的同向外部趋势/斜率不符合判据，不再要求真实水平线，且永不遮罩肩部证据；
- periodic 只约束周期和完整性、不约束与发送三角波同形；同一信号的已验真上横切是硬否决，下横切不是，左右边缘线和 middle 允许遮罩/简化；
- DRate 普通 `has_wave` 不看任何横切，始终从原始有效视图判断幅度、陡变样本和往返变化；SRTT 普通 `has_wave` 按规定使用清理视图；
- MaxRTT、RTpropDRate 的更新点和跨 Cruise 生命周期正确；
- 只有 `kFBBR` 的 Regime I/III 严格执行 50% 公式、Regime II 同时更新 baseline/ProbedBw；
- `kFBBRAdaptive` 对 N01-N18、MaxRTT/RTpropDRate、PDF 执行器和保真增强状态的调用/写入计数全部为 0；其自身既有不确定扩展/幅度行为通过 golden trace 冻结回归；
- 不确定严格执行第 6 节：第一次只补一个周期，第二次按 1.25 倍调整探针幅度并重新 settle，2 倍封顶后逐周期滚动；不确定期间 baseline 不变；
- SRTT 或 DRate 任一连续两个有效最终窗无普通波动后，冻结触发窗的全部判定副作用；状态机空闲时把它作为第一次不确定，已有输入无效重试时只合并 reason 而不重置阶段；随后每次只后探一个周期、用最新两周期复判，二者任一恢复就清除该 reason，否则按原状态机在同幅度第二次仍失败后以 1.25 倍增幅、2 倍封顶；
- 全部分支、边界、冲突、状态机和公式测试通过，trace 能复原每次决策。
