# Frequency-BBRv2：基于 Delivery Rate 的收敛门控频域机制

> 版本：2026-07-02  
> 适用对象：在 **BBRv2** 上实现的 CRUISE 阶段频域调制与可信交付速率修正机制。  
> 核心修改：**退出稳定收敛状态的判断不再使用经过 max-filter 的 `BBR.max_bw`，而使用与 BBRv2 full-pipe estimator 语义一致的 fresh non-app-limited `RS.delivery_rate`。**

---

## 1. 设计目标

Frequency-BBRv2 的目标不是永久替代 BBRv2 的带宽模型，而是在 BBRv2 的带宽估计出现明显波动时，临时启用 CRUISE 阶段的频域工具，提取一个更可信、更稳定的交付速率参考值，用于平滑或校正当前的有效带宽估计。

整体原则如下：

1. **BBRv2 稳定收敛时**：关闭频域工具，直接使用 BBRv2 原生带宽估计。
2. **BBRv2 退出稳定收敛时**：在后续 CRUISE 阶段重新开启频域工具。
3. **频域工具获得可信交付速率参考值后**：将该参考值与 BBRv2 原生带宽估计进行置信加权融合。
4. **BBRv2 再次稳定后**：频域权重逐步衰减到 0，算法回退到原生 BBRv2 行为。

本机制特别强调：

> **稳定退出检测使用 fresh delivery-rate observations，而不是使用经过两周期 max-filter 处理后的 `BBR.max_bw`。**

原因是 `BBR.max_bw` 的作用是保留最近窗口内的高 delivery-rate sample，本身具有记忆效应。它适合作为 BBRv2 的带宽模型变量，但不适合作为“当前是否波动”的唯一观测量。

---

## 2. 与 BBRv2 原生机制的对齐

BBRv2 的 full-pipe estimator 使用的是 delivery-rate sample，而不是 CRUISE 阶段的平均值或中位数。

在 BBRv2 中，full-pipe estimator 的基本逻辑是：如果若干个非 app-limited round 内，尝试提高发送速率后 delivery rate 的增长仍然小于 25%，则认为已经填满管道，即已经接近可用带宽平台。典型条件为：

\[
RS.delivery\_rate < 1.25 \times BBR.full\_bw
\]

连续出现 3 个这样的非 app-limited round 后，设置：

\[
BBR.filled\_pipe = true
\]

或者在较新的草案表述中：

\[
BBR.full\_bw\_now = true, \quad BBR.full\_bw\_reached = true
\]

因此，Frequency-BBRv2 的稳定收敛判断和退出稳定判断都应尽量对齐这个语义：

> **观测对象应是 fresh non-app-limited delivery-rate sample，而不是 max-filtered MaxBw。**

---

## 3. 关键变量定义

### 3.1 Fresh delivery-rate indicator

定义第 \(r\) 个 round 的新鲜交付速率指标为：

\[
D_r
\]

其中 \(D_r\) 来自 BBRv2 ACK 处理过程中的 `RS.delivery_rate`，并且必须满足：

\[
RS.is\_app\_limited = false
\]

也就是说，只使用非 app-limited 的 delivery-rate sample。

### 3.2 推荐实现方式

严格对齐 BBRv2 full-pipe estimator 的实现方式：

\[
D_r = RS.delivery\_rate \quad \text{when } BBR.round\_start \land !RS.is\_app\_limited
\]

工程实现中，为了降低单个 ACK sample 的偶然噪声，也可以在一个 round 内收集多个非 app-limited delivery-rate sample，并取该 round 内的最大值：

\[
D_r = \max_{s \in round\ r}\{RS_s.delivery\_rate \mid !RS_s.is\_app\_limited\}
\]

这仍然与 BBRv2 的带宽采样思想保持一致，因为 BBR 的带宽模型本身倾向于使用 delivery-rate sample 的高值来估计瓶颈带宽，而不是使用平均值或中位数。

### 3.3 BBRv2 原生带宽估计

定义 BBRv2 原生带宽估计为：

\[
B^{native}_r
\]

如果实现中保留了 BBRv2 的上下界变量，则建议使用：

\[
B^{native}_r = \min(BBR.max\_bw, BBR.bw\_hi, BBR.bw\_lo)
\]

如果当前 ns-3 或实验实现没有完整实现 `bw_hi` / `bw_lo`，则可以退化为：

\[
B^{native}_r = BBR.max\_bw
\]

注意：**不建议直接覆盖 `BBR.max_bw`**。Frequency-BBRv2 应计算一个额外的有效带宽：

\[
B^{eff}_r
\]

该值用于 pacing rate 和 BDP/cwnd 计算。

### 3.4 频域可信交付速率参考值

定义频域工具输出的可信交付速率参考值为：

\[
F_r
\]

它来自 CRUISE 阶段频域响应最可信的观测窗口：

\[
F_r = mean(DRate(t)), \quad t \in W^*
\]

其中 \(W^*\) 是频域得分最高的可信窗口。

### 3.5 频域融合权重

定义频域参考值权重为：

\[
w_r \in [0,1]
\]

- \(w_r=0\)：完全使用 BBRv2 原生带宽估计。
- \(w_r=1\)：当前有效带宽完全由频域可信参考值主导。
- \(0<w_r<1\)：BBRv2 原生估计和频域参考值加权融合。

---

## 4. 稳定收敛状态判断

### 4.1 状态变量

维护以下变量：

```c
bool bbr_stable;
double full_drate_ref;
int stable_cnt;
```

其中：

- `bbr_stable`：Frequency-BBRv2 自定义的“BBRv2 稳定收敛状态”。
- `full_drate_ref`：当前 full-pipe 判断使用的 delivery-rate 基线。
- `stable_cnt`：连续多少个非 app-limited round 未出现超过 25% 的 delivery-rate 增长。

### 4.2 初始化

在连接开始、进入 Startup、或者退出稳定后重新开始检测时，执行：

```c
full_drate_ref = 0;
stable_cnt = 0;
bbr_stable = false;
```

### 4.3 每个非 app-limited round 的判断

当一个新的 round 产生有效 \(D_r\) 后，执行：

如果：

\[
D_r \geq 1.25 \times full\_drate\_ref
\]

说明 delivery rate 仍然存在明显增长趋势，因此更新基线并清零计数：

```c
full_drate_ref = D_r;
stable_cnt = 0;
```

如果：

\[
D_r < 1.25 \times full\_drate\_ref
\]

说明当前 round 没有出现超过 25% 的 delivery-rate 增长，则：

```c
stable_cnt++;
```

当：

\[
stable\_cnt \geq 3
\]

并且 BBRv2 已经处于 ProbeBW 阶段时，认为 BBRv2 进入稳定收敛区域：

\[
bbr\_stable = true
\]

### 4.4 为什么不使用 CRUISE 平均值或中位数判断稳定

在 BBRv2 未稳定时，Frequency-BBRv2 的频域工具可能已经在 CRUISE 阶段开启，此时 CRUISE 内部的 delivery-rate samples 会受到三角调制影响。如果直接使用 CRUISE 内的平均值或中位数判断稳定，可能会把频域工具自身引入的可控波动误认为 BBRv2 的自然收敛特征。

因此，稳定判断应对齐 BBRv2 原生 full-pipe estimator 的语义，使用 fresh non-app-limited `RS.delivery_rate`，而不是 CRUISE 窗口均值、中位数或经过 max-filter 的 `BBR.max_bw`。

---

## 5. 退出稳定收敛状态判断

### 5.1 退出判断的观测对象

退出稳定状态的判断使用：

\[
D_r
\]

即 fresh non-app-limited delivery-rate indicator。

不使用：

\[
BBR.max\_bw
\]

原因是 `BBR.max_bw` 是经过窗口 max-filter 处理后的模型变量。它可能因为旧高样本而保持不变，也可能因为短时高样本而被放大并持续一段时间。因此，直接用 `max_bw` 判断“当前是否波动”会滞后或失真。

### 5.2 单 round 波动率

定义第 \(r\) 个 round 的 delivery-rate 波动率：

\[
V_r = \frac{|D_r - D_{r-1}|}{D_{r-1}}
\]

其中 \(D_r\) 和 \(D_{r-1}\) 都必须是有效的非 app-limited delivery-rate indicator。

### 5.3 退出稳定条件

当满足以下任一条件时，判断 BBRv2 退出稳定收敛状态：

#### 条件 1：单 round 大幅波动

\[
V_r > 0.25
\]

即当前 round 的 fresh delivery-rate indicator 相对上一 round 的变化超过 25%。

#### 条件 2：连续两个 round 中等幅度波动

\[
V_r > 0.15 \quad \land \quad V_{r-1} > 0.15
\]

即连续两个 round 的 fresh delivery-rate indicator 波动都超过 15%。

### 5.4 方向无关原则

退出稳定判断不区分上跳和下跳：

\[
D_r > D_{r-1}
\]

或：

\[
D_r < D_{r-1}
\]

都只通过绝对波动率 \(V_r\) 处理。

原因是：

- 上跳可能是真实可用带宽增加，也可能是 ProbeBW_UP、队列释放或 ACK 聚合造成的偏高 sample。
- 下跳可能是真实竞争增强，也可能是短期 drain、采样相位或噪声造成的偏低 sample。

因此，方向本身不能决定 BBRv2 是否可靠。**只要 fresh delivery-rate indicator 出现超过阈值的周期级波动，就说明当前 BBRv2 带宽模型需要频域工具重新验证。**

### 5.5 触发动作

一旦触发退出稳定：

```c
bbr_stable = false;
stable_cnt = 0;
full_drate_ref = D_r;
freq_tool_needed = true;
```

同时提升频域参考权重：

\[
w_r \leftarrow \min(1, w_r + \eta_{on})
\]

建议初始值：

\[
\eta_{on}=0.5
\]

也就是第一次检测到明显波动时，频域参考值获得 50% 的融合权重；如果后续仍然不稳定，权重可进一步提高到 1。

---

## 6. CRUISE 阶段频域工具的开启与关闭

### 6.1 频域工具只在 ProbeBW_CRUISE 阶段执行

Frequency-BBRv2 的频域调制只作用于 BBRv2 的 CRUISE 阶段：

\[
BBR.state = ProbeBW\_CRUISE
\]

在其他 ProbeBW 子阶段中，不执行频域三角调制。

### 6.2 开启条件

当：

\[
bbr\_stable = false
\]

并且当前进入：

\[
ProbeBW\_CRUISE
\]

则开启频域工具：

```c
freq_tool_on = true;
```

发送速率在 CRUISE 基础上叠加零均值三角调制：

\[
R_{send}(t) = B^{eff}_r \cdot \left(1 + A \cdot tri(2\pi f_m t)\right)
\]

其中：

\[
tri(2\pi f_m t) \in [-1,1]
\]

建议调制幅度：

\[
A = 0.03 \sim 0.05
\]

即 3% 到 5%。

调制频率为：

\[
f_m
\]

其中 \(f_m\) 应在实现中固定，并保证一个观测窗口内能够覆盖足够数量的调制周期。

### 6.3 关闭条件

当：

\[
bbr\_stable = true
\]

则关闭频域工具：

```c
freq_tool_on = false;
modulation_amp = 0;
```

此时发送速率回退为 BBRv2 原生 CRUISE 行为：

\[
R_{send}(t) = BBRv2.pacing\_gain \cdot B^{native}_r
\]

在 CRUISE 阶段，通常：

\[
BBRv2.pacing\_gain = 1.0
\]

因此稳定后不再持续对 CRUISE 阶段进行频域调制。

---

## 7. 频域可信参考值的生成

### 7.1 采样对象

当频域工具开启时，在 CRUISE 阶段记录以下时间序列：

\[
DRate(t)
\]

\[
SRTT(t)
\]

其中 `DRate(t)` 来自 ACK 返回的 delivery-rate sample 或按照时间轴重采样后的交付速率序列。

### 7.2 观测窗口长度

令一个频域观测窗口为 \(W\)，窗口长度建议覆盖 \(N_f\) 个调制周期：

\[
|W| = \frac{N_f}{f_m}
\]

建议：

\[
N_f = 3 \sim 5
\]

如果窗口过短，频域主频识别不稳定；如果窗口过长，可能跨越多个网络状态，导致参考值滞后。

### 7.3 主频匹配条件

对窗口 \(W\) 内的 `DRate(t)` 和 `SRTT(t)` 做频域分析，得到主频：

\[
f_D(W) = dominant\_freq(DRate(t), W)
\]

\[
f_R(W) = dominant\_freq(SRTT(t), W)
\]

要求主频与调制频率匹配：

\[
\frac{|f_D(W)-f_m|}{f_m} \leq \epsilon_f
\]

\[
\frac{|f_R(W)-f_m|}{f_m} \leq \epsilon_f
\]

建议：

\[
\epsilon_f = 0.15 \sim 0.20
\]

### 7.4 能量显著性条件

定义调制频率附近能量占比：

\[
E_D^{ratio}(W) = \frac{E_D(f_m, W)}{E_D(total, W)}
\]

\[
E_R^{ratio}(W) = \frac{E_R(f_m, W)}{E_R(total, W)}
\]

要求：

\[
E_D^{ratio}(W) \geq \rho_D
\]

\[
E_R^{ratio}(W) \geq \rho_R
\]

建议初始取：

\[
\rho_D = 0.25
\]

\[
\rho_R = 0.25
\]

### 7.5 可信窗口得分

可以定义频域窗口得分：

\[
Score(W) = \alpha Q_{freq}(W) + \beta Q_{energy}(W) + \gamma Q_{cons}(W) + \delta Q_{wave}(W)
\]

其中：

- \(Q_{freq}\)：DRate / SRTT 主频是否接近 \(f_m\)。
- \(Q_{energy}\)：调制频率附近的频域能量是否足够显著。
- \(Q_{cons}\)：多个相邻窗口中频域响应是否一致。
- \(Q_{wave}\)：DRate 波形是否与调制信号存在可识别对应关系。

简化权重可以取：

\[
\alpha = 0.35, \quad \beta = 0.35, \quad \gamma = 0.20, \quad \delta = 0.10
\]

得到窗口置信度：

\[
C(W) = Score(W)
\]

当：

\[
C(W) \geq C_{valid}
\]

认为窗口可信。

建议：

\[
C_{valid}=0.60
\]

### 7.6 选择最佳窗口

如果多个窗口满足可信条件，则选择得分最高的窗口：

\[
W^* = \arg\max_W Score(W)
\]

并计算频域可信交付速率参考值：

\[
F_r = mean(DRate(t)), \quad t \in W^*
\]

为了降低偶发 ACK 压缩或异常 sample 的影响，也可以使用截尾均值：

\[
F_r = trimmed\_mean_{10\%-90\%}(DRate(t)), \quad t \in W^*
\]

建议实现优先采用截尾均值；如果实验中 sample 已经较平滑，也可以使用普通均值。

### 7.7 频域参考值有效期

频域参考值不应无限期有效。建议设置有效期：

\[
TTL_F = 2 \text{ ProbeBW cycles}
\]

或者：

\[
TTL_F = 6 \text{ non-app-limited rounds}
\]

当超过有效期且未获得新的可信窗口时：

```c
F_valid = false;
```

---

## 8. MaxBw 与频域可信值的取舍策略

### 8.1 不覆盖 BBR.max_bw

Frequency-BBRv2 不直接执行：

\[
BBR.max\_bw \leftarrow F_r
\]

原因是 `BBR.max_bw` 是 BBRv2 原生模型的一部分，具有窗口 max-filter 语义。直接覆盖它会破坏 BBRv2 的模型一致性。

正确方式是额外计算：

\[
B^{eff}_r
\]

并将其用于 pacing rate 和 BDP/cwnd 计算。

### 8.2 稳定状态下的取舍

当：

\[
bbr\_stable = true
\]

则：

\[
w_r = 0
\]

\[
B^{target}_r = B^{native}_r
\]

\[
B^{eff}_r = B^{native}_r
\]

此时频域工具关闭，频域参考值不参与最终带宽估计。

### 8.3 非稳定但尚未获得可信频域值

当：

\[
bbr\_stable = false
\]

但：

\[
F_r \text{ invalid}
\]

则暂时使用：

\[
B^{target}_r = B^{native}_r
\]

同时在后续 CRUISE 阶段开启频域工具，等待生成可信 \(F_r\)。

### 8.4 非稳定且获得可信频域值

当：

\[
bbr\_stable = false
\]

并且：

\[
F_r \text{ valid}
\]

则进行加权融合：

\[
B^{target}_r = (1-w_r) B^{native}_r + w_r F_r
\]

该融合不区分：

\[
F_r < B^{native}_r
\]

或：

\[
F_r > B^{native}_r
\]

因为频域参考值的作用不是单纯下修 MaxBw，而是在 BBRv2 带宽模型出现波动时提供更稳定的交付速率参考。

### 8.5 权重设置

建议根据频域置信度 \(C(W^*)\) 设置权重：

\[
w_r = clip\left(0.5 + 0.5 \times \frac{C(W^*)-C_{valid}}{C_{high}-C_{valid}}, 0.5, 1.0\right)
\]

其中：

\[
C_{valid}=0.60
\]

\[
C_{high}=0.90
\]

含义是：

- 当窗口刚达到可信门槛时，频域值至少获得 50% 权重。
- 当窗口置信度达到 0.90 或更高时，频域值获得 100% 权重。

如果实现中暂时不计算连续置信分数，也可以使用离散规则：

\[
w_r =
\begin{cases}
0.5, & \text{first valid frequency reference after instability} \\
1.0, & \text{consecutive instability or high-confidence frequency window}
\end{cases}
\]

### 8.6 有效带宽更新平滑

为了避免 \(B^{eff}_r\) 在 BBRv2 原生估计和频域参考值之间剧烈切换，建议使用更新速率限制器：

\[
B^{eff}_r = B^{eff}_{r-1} + clip\left(B^{target}_r - B^{eff}_{r-1}, -\lambda_{down}B^{eff}_{r-1}, \lambda_{up}B^{eff}_{r-1}\right)
\]

建议初始取：

\[
\lambda_{up}=0.10
\]

\[
\lambda_{down}=0.20
\]

即每个 round 或每个更新周期内：

- 向上最多增加 10%；
- 向下最多降低 20%。

如果实现中希望完全保留 BBRv2 自身对上调的谨慎性，可以进一步降低：

\[
\lambda_{up}=0.05
\]

---

## 9. 重新进入稳定后的频域权重释放

当 BBRv2 再次满足稳定收敛判断：

\[
stable\_cnt \geq 3
\]

则设置：

```c
bbr_stable = true;
freq_tool_on = false;
```

但频域权重不必瞬间归零，而是逐步释放：

\[
w_r \leftarrow \max(0, w_r - \eta_{off})
\]

建议：

\[
\eta_{off}=0.25
\]

也就是连续 4 个稳定更新周期后，频域权重完全释放：

\[
w_r = 0
\]

最终：

\[
B^{eff}_r = B^{native}_r
\]

此时 Frequency-BBRv2 完全回退到原生 BBRv2 行为。

---

## 10. 完整状态机

### State A：Stable BBRv2 / Frequency Off

进入条件：

\[
stable\_cnt \geq 3
\]

并且 BBRv2 已经进入 ProbeBW 阶段。

行为：

\[
freq\_tool\_on = false
\]

\[
A=0
\]

\[
w_r \rightarrow 0
\]

\[
B^{eff}_r = B^{native}_r
\]

同时继续监测 fresh delivery-rate indicator \(D_r\)。

### State B：Unstable BBRv2 / Frequency Needed

进入条件：

\[
V_r > 0.25
\]

或：

\[
V_r > 0.15 \land V_{r-1} > 0.15
\]

行为：

\[
bbr\_stable=false
\]

\[
freq\_tool\_needed=true
\]

\[
w_r \leftarrow \min(1, w_r + \eta_{on})
\]

如果当前不是 CRUISE 阶段，则暂不调制，等待进入 CRUISE。

### State C：ProbeBW_CRUISE / Frequency Active

进入条件：

\[
bbr\_stable=false
\]

并且：

\[
BBR.state=ProbeBW\_CRUISE
\]

行为：

\[
freq\_tool\_on=true
\]

\[
A=3\% \sim 5\%
\]

记录 `DRate(t)` 和 `SRTT(t)`，并执行频域窗口筛选。

### State D：Frequency Reference Valid

进入条件：

\[
F_r \text{ valid}
\]

行为：

\[
B^{target}_r = (1-w_r)B^{native}_r + w_rF_r
\]

\[
B^{eff}_r = slew\_limited(B^{target}_r)
\]

频域参考值在有效期内持续参与带宽融合。

### State E：Re-stabilization / Release Frequency Weight

进入条件：

\[
stable\_cnt \geq 3
\]

行为：

\[
freq\_tool\_on=false
\]

\[
w_r \leftarrow \max(0, w_r - \eta_{off})
\]

当：

\[
w_r=0
\]

则：

\[
F\_valid=false
\]

\[
B^{eff}_r = B^{native}_r
\]

---

## 11. 完整伪代码

```c
// ============================================================
// Frequency-BBRv2: delivery-rate-based convergence gating
// ============================================================

// ---------- parameters ----------
const double FULL_BW_GROWTH_TH = 1.25;  // BBRv2 full-pipe threshold
const int    FULL_BW_CNT_TH    = 3;     // BBRv2 full-pipe round count

const double EXIT_JUMP_HARD    = 0.25;  // one-round volatility trigger
const double EXIT_JUMP_SOFT    = 0.15;  // two-round volatility trigger

const double ETA_ON            = 0.50;
const double ETA_OFF           = 0.25;

const double C_VALID           = 0.60;
const double C_HIGH            = 0.90;

const double MAX_UP_STEP       = 0.10;
const double MAX_DOWN_STEP     = 0.20;

const double A_ACTIVE          = 0.03;  // can tune to 0.05

// ---------- state variables ----------
bool bbr_stable       = false;
bool freq_tool_needed = false;
bool freq_tool_on     = false;
bool F_valid          = false;

int stable_cnt = 0;

double full_drate_ref = 0.0;
double D_round        = 0.0;
double D_prev         = 0.0;
double V_round        = 0.0;
double V_prev         = 0.0;

double F_ref          = 0.0;
double F_conf         = 0.0;
double w_freq         = 0.0;

double B_native       = 0.0;
double B_target       = 0.0;
double B_eff          = 0.0;
double B_eff_prev     = 0.0;

// ============================================================
// 1. Collect fresh delivery-rate indicator
// ============================================================

if (round_start) {
    D_prev = D_round;
    D_round = 0.0;
}

if (!rs.is_app_limited) {
    // Engineering option: use max non-app-limited delivery-rate sample in this round.
    D_round = max(D_round, rs.delivery_rate);
}

bool D_valid = (D_round > 0.0);
bool D_prev_valid = (D_prev > 0.0);

// ============================================================
// 2. Enter stable convergence using BBRv2-like full-pipe logic
// ============================================================

if (round_end && D_valid) {
    if (full_drate_ref == 0.0) {
        full_drate_ref = D_round;
        stable_cnt = 0;
    }
    else if (D_round >= FULL_BW_GROWTH_TH * full_drate_ref) {
        // Delivery rate is still growing by at least 25%.
        full_drate_ref = D_round;
        stable_cnt = 0;
    }
    else {
        // One more non-app-limited round without 25% growth.
        stable_cnt++;
    }

    if (stable_cnt >= FULL_BW_CNT_TH && IsInProbeBWState()) {
        bbr_stable = true;
        freq_tool_needed = false;
    }
}

// ============================================================
// 3. Exit stable convergence using fresh delivery-rate volatility
// ============================================================

if (round_end && bbr_stable && D_valid && D_prev_valid) {
    V_round = fabs(D_round - D_prev) / D_prev;

    bool volatility_trigger =
        (V_round > EXIT_JUMP_HARD) ||
        (V_round > EXIT_JUMP_SOFT && V_prev > EXIT_JUMP_SOFT);

    if (volatility_trigger) {
        bbr_stable = false;
        stable_cnt = 0;
        full_drate_ref = D_round;

        freq_tool_needed = true;
        w_freq = min(1.0, w_freq + ETA_ON);
    }

    V_prev = V_round;
}

// ============================================================
// 4. Switch frequency tool only in ProbeBW_CRUISE
// ============================================================

if (!bbr_stable && IsProbeBWCRUISE()) {
    freq_tool_on = true;
    modulation_amp = A_ACTIVE;
} else {
    freq_tool_on = false;
    modulation_amp = 0.0;
}

// ============================================================
// 5. Frequency-domain reference extraction
// ============================================================

if (freq_tool_on) {
    UpdateFrequencyWindow(rs.delivery_rate, srtt, now);

    if (FindCredibleFrequencyWindow(&F_ref, &F_conf)) {
        F_valid = true;

        // Confidence-weighted frequency reference weight.
        double conf_weight =
            0.5 + 0.5 * (F_conf - C_VALID) / (C_HIGH - C_VALID);

        conf_weight = clamp(conf_weight, 0.5, 1.0);
        w_freq = max(w_freq, conf_weight);

        RefreshFrequencyReferenceTTL();
    }
}

if (F_valid && FrequencyReferenceExpired()) {
    F_valid = false;
}

// ============================================================
// 6. Native BBRv2 bandwidth estimate
// ============================================================

if (HasBBRv2Bounds()) {
    B_native = min(BBR.max_bw, min(BBR.bw_hi, BBR.bw_lo));
} else {
    B_native = BBR.max_bw;
}

// ============================================================
// 7. Bandwidth fusion
// ============================================================

if (!bbr_stable && F_valid) {
    B_target = (1.0 - w_freq) * B_native + w_freq * F_ref;
} else {
    B_target = B_native;
}

// ============================================================
// 8. Slew-rate limited effective bandwidth update
// ============================================================

double delta = B_target - B_eff_prev;
double max_up = MAX_UP_STEP * B_eff_prev;
double max_down = MAX_DOWN_STEP * B_eff_prev;

delta = clamp(delta, -max_down, max_up);
B_eff = B_eff_prev + delta;

// ============================================================
// 9. Release frequency weight after re-stabilization
// ============================================================

if (bbr_stable) {
    w_freq = max(0.0, w_freq - ETA_OFF);

    if (w_freq == 0.0) {
        F_valid = false;
    }
}

// ============================================================
// 10. Final pacing rate
// ============================================================

pacing_rate = BBR.pacing_gain * B_eff *
              (1.0 + modulation_amp * triangle_wave(f_m, now));

B_eff_prev = B_eff;
```

---

## 12. 建议默认参数表

| 参数 | 含义 | 建议初始值 | 说明 |
|---|---:|---:|---|
| `FULL_BW_GROWTH_TH` | full-pipe 增长阈值 | 1.25 | 对齐 BBRv2 25% 依据 |
| `FULL_BW_CNT_TH` | full-pipe 连续 round 数 | 3 | 对齐 BBRv2 3 rounds 依据 |
| `EXIT_JUMP_HARD` | 单 round 退出稳定阈值 | 0.25 | 与 BBRv2 25% 尺度一致 |
| `EXIT_JUMP_SOFT` | 连续两个 round 退出阈值 | 0.15 | 用于更早识别持续波动 |
| `ETA_ON` | 频域权重提升步长 | 0.50 | 首次波动后频域至少占 50% |
| `ETA_OFF` | 稳定后频域权重释放步长 | 0.25 | 约 4 个稳定更新周期释放完 |
| `A_ACTIVE` | 三角调制幅度 | 0.03–0.05 | 3%–5%，零均值调制 |
| `N_f` | 每个频域窗口包含调制周期数 | 3–5 | 平衡频域可靠性与响应速度 |
| `epsilon_f` | 主频匹配容忍度 | 0.15–0.20 | 频域主频接近 \(f_m\) 的阈值 |
| `rho_D` | DRate 频域能量阈值 | 0.25 | 可根据实验调参 |
| `rho_R` | SRTT 频域能量阈值 | 0.25 | 可根据实验调参 |
| `C_VALID` | 可信窗口最低分数 | 0.60 | 低于该值不更新 \(F_r\) |
| `C_HIGH` | 高频域置信阈值 | 0.90 | 达到该值可令 \(w=1\) |
| `MAX_UP_STEP` | 有效带宽单周期最大上调 | 0.10 | 防止上调过快 |
| `MAX_DOWN_STEP` | 有效带宽单周期最大下调 | 0.20 | 允许更快抑制过估计 |
| `TTL_F` | 频域参考值有效期 | 2 ProbeBW cycles / 6 rounds | 防止过旧频域参考持续生效 |

---

## 13. 可直接写入论文的方法描述

### 中文版本

Frequency-BBRv2 采用基于 BBRv2 收敛状态的频域门控机制，同时控制 CRUISE 阶段频域调制的执行以及频域可信交付速率参考值与 BBRv2 原生带宽估计之间的取舍。与直接观察 `BBR.max_bw` 不同，本机制使用 fresh non-app-limited delivery-rate sample 作为稳定进入和退出判断的观测量，从而与 BBRv2 full-pipe estimator 的语义保持一致。具体而言，当连续三个非 app-limited round 中 delivery rate 均未相对基线增长超过 25% 时，认为 BBRv2 进入稳定收敛区域，并关闭 CRUISE 阶段的频域工具，直接使用 BBRv2 原生带宽估计。当 fresh delivery-rate indicator 在单个 round 内波动超过 25%，或在连续两个 round 内均波动超过 15% 时，认为 BBRv2 退出稳定收敛区域。此时 Frequency-BBRv2 在后续 CRUISE 阶段重新开启频域工具，并从频域响应最可信的窗口中提取交付速率参考值。最终有效带宽由 BBRv2 原生带宽估计与频域可信参考值进行置信加权融合，并通过更新速率限制器避免剧烈跳变。当 BBRv2 再次满足多轮收敛条件后，频域权重逐步衰减为零，算法回退到原生 BBRv2 行为。

### English version

Frequency-BBRv2 introduces a convergence-gated frequency-domain mechanism that jointly controls CRUISE-phase modulation and the selection between BBRv2's native bandwidth estimate and the frequency-domain delivery-rate reference. Instead of using the max-filtered `BBR.max_bw` as the stability-exit signal, Frequency-BBRv2 monitors fresh non-application-limited delivery-rate observations, which is consistent with the semantics of BBRv2's full-pipe estimator. A flow is regarded as entering the stable-convergence region when its delivery rate fails to grow by more than 25% over three non-application-limited rounds. In this stable state, CRUISE-phase frequency modulation is disabled and the effective bandwidth falls back to the native BBRv2 bandwidth estimate. When the fresh delivery-rate indicator changes by more than 25% in one round, or by more than 15% over two consecutive rounds, BBRv2 is regarded as leaving the stable-convergence region. Frequency-BBRv2 then reactivates the frequency-domain tool in subsequent CRUISE phases and extracts a credible delivery-rate reference from the best frequency-response window. The final effective bandwidth is obtained by confidence-weighted fusion between BBRv2's native bandwidth estimate and the frequency-domain reference, followed by a slew-rate limiter. Once BBRv2 again satisfies the multi-round convergence condition, the frequency-domain weight decays to zero and the algorithm falls back to native BBRv2 behavior.

---

## 14. 实现注意事项

1. **退出稳定检测不要使用 `BBR.max_bw`。** 退出检测使用 fresh `RS.delivery_rate`，因为 `BBR.max_bw` 是 max-filtered model state，不是当前观测样本。
2. **不要覆盖 `BBR.max_bw`。** 频域结果应生成额外的 `B_eff`，用于 pacing/cwnd 计算。
3. **频域工具只在 CRUISE 执行。** 如果当前不是 `ProbeBW_CRUISE`，即使已经退出稳定，也等待下一次 CRUISE。
4. **稳定后关闭调制。** 一旦重新满足 25% / 3 rounds 的稳定条件，后续 CRUISE 不再执行频域调制。
5. **CRUISE 内部均值/中位数不用于稳定判断。** 因为频域工具开启时，CRUISE 样本本身会包含调制波动。
6. **频域参考值需要 TTL。** 防止过旧的 \(F_r\) 在网络状态变化后仍然参与融合。
7. **权重释放要渐进。** 稳定后 \(w_r\) 逐步衰减到 0，避免有效带宽瞬间从 \(F_r\) 跳回 \(B^{native}\)。

---

## 15. 参考资料

1. Cardwell et al., *BBR Congestion Control*, Internet-Draft, BBRv2 draft.  
   https://datatracker.ietf.org/doc/html/draft-cardwell-iccrg-bbr-congestion-control-01

2. IETF CCWG, *BBR Congestion Control*, current working draft.  
   https://ietf-wg-ccwg.github.io/draft-ietf-ccwg-bbr/draft-ietf-ccwg-bbr.html

3. Cardwell et al., *BBR: Congestion-Based Congestion Control*, ACM Queue.  
   https://queue.acm.org/detail.cfm?id=3022184
