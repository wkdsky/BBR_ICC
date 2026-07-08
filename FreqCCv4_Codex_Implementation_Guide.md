# FreqCCv4 收敛门控与频域参考反馈：Codex 落地实现指南

> 目标：把上一轮对 Codex 建议的评估，以及最新修正后的“频域权重随 BBRv2 重新收敛证据递减、权重归零后再关闭频域调制”的机制，整理成可以直接指导代码实现的版本。

---

## 0. 总体结论

当前 `FreqCCv4Sender` 已经具备 CRUISE 阶段扰动和频域窗口分析能力，但仍缺两个关键闭环：

1. **收敛门控闭环**：稳定后关闭频域工具，失稳后重新开启；
2. **控制面反馈闭环**：把频域工具得到的可信交付速率参考值反馈到 pacing 决策，而不是只做观测分析。

最终落地原则如下：

> **FreqCCv4 用 fresh non-app-limited delivery-rate 的波动来判断 BBRv2 是否退出稳定收敛区域；一旦失稳，在 ProbeBW_CRUISE 中开启频域工具，并用频域可信交付速率参考值参与 pacing 层带宽参考。随着 BBRv2 重新积累 3 个 round 的 full-pipe 收敛证据，频域参考权重逐步降低。当 `stable_cnt == 3` 时，频域权重归零，并同步关闭 CRUISE 阶段频域调制。**

这里最重要的修正是：

> **不要在频域调制关闭后继续用旧的频域参考值做 2–4 个 round 的过渡。**

原因是：调制关闭后，`F_ref` 不再被刷新；继续使用旧 `F_ref` 会让 stale reference 影响 pacing。正确做法是在 **频域工具仍然开启的 re-convergence 阶段** 完成权重释放；当权重已经降到 0 后，再关闭频域工具。

---

## 1. 需要先修正的工程语义

### 1.1 `D_r` 必须来自正确的 fresh non-app-limited sample

机制中定义的 round 级交付速率指标为：

```math
D_r = \max_{s \in r}\{s.delivery\_rate \mid s.is\_app\_limited=false\}
```

这个 `D_r` 不能用 `ExportDebugState().last_sample_is_app_limited` 凑合实现。

原因是：

- `D_r` 需要的是**产生 round 内最大 delivery-rate 的那个 sample** 是否为 non-app-limited；
- `last_sample_is_app_limited` 只是最后一个 packet sample 的标志；
- 一个 round 内最大 delivery-rate sample 不一定是最后一个 sample。

错误使用 `last_sample_is_app_limited` 会造成两类污染：

```text
1. app-limited 的高 delivery-rate sample 被误纳入 D_r；
2. non-app-limited 的高 delivery-rate sample 被最后一个 app-limited sample 误杀。
```

因此，第一步必须在 `Bbr2Sender` / `Bbr2NetworkModel` / congestion event 路径上暴露只读 sample view。

建议接口：

```cpp
struct FreqBbr2SampleView {
  bool round_start;
  bool sample_valid;
  QuicBandwidth sample_max_bandwidth;
  bool sample_is_app_limited;
  bool in_recovery;
};
```

`FreqCCv4Sender` 只消费这个只读 view，不直接改 BBRv2 内部模型。

---

### 1.2 第一版 `B_native` 使用当前仓库已有的 `BandwidthEstimate()`

机制文档中不要强行写：

```math
B_{native}=\min(BBR.max\_bw, BBR.bw\_hi, BBR.bw\_lo)
```

因为当前仓库实现没有完整的 `bw_hi` 控制面。第一版应按当前代码实现写为：

```math
B_{native}=BandwidthEstimate()
```

如果当前仓库中已有：

```cpp
BandwidthEstimate() = min(MaxBandwidth(), bandwidth_lo_)
```

那么第一版控制面就以这个结果作为原生 BBRv2 带宽参考。

也就是说：

```math
B_{target}=(1-w_f)B_{native}+w_fF_{ref}
```

最终只在 `PacingRate()` 层做 scale：

```math
scale = \frac{B_{target}}{B_{native}}
```

```math
PacingRate_{new}=PacingRate_{bbr2}\cdot scale\cdot(1+A\cdot tri(f_mt))
```

第一版不要直接覆盖：

```cpp
MaxBandwidth()
BandwidthEstimate()
cwnd
BDP
inflight_hi
inflight_lo
```

---

### 1.3 第一版只接 pacing，不接 cwnd/BDP

理论上，`B_eff` 可以用于 pacing 和 BDP/cwnd 计算。但工程第一版应保守落地：

> **只在 pacing 层用频域参考做速率缩放，不修改 BBRv2 内部带宽滤波器和 cwnd/BDP 模型。**

这样做的好处是：

1. 改动小，易回滚；
2. 不破坏 BBRv2 内部状态机；
3. 归因清晰，实验中性能变化主要来自 pacing reference，而不是多个控制变量同时改变。

---

## 2. 核心状态变量

建议在 `FreqCCv4Sender` 中新增如下变量。

```cpp
// round-level fresh delivery-rate indicator
QuicBandwidth D_round_;
bool D_round_valid_ = false;
QuicBandwidth D_prev_;
bool D_prev_valid_ = false;

// full-pipe style convergence evidence
QuicBandwidth full_drate_ref_;
bool full_drate_ref_valid_ = false;
int stable_cnt_ = 0;
bool bbr_stable_ = false;

// round-level volatility
bool prev_V_valid_ = false;
double prev_V_ = 0.0;

// frequency reference
QuicBandwidth F_ref_;
bool F_ref_valid_ = false;
double F_conf_ = 0.0;
uint64_t F_ref_update_round_ = 0;

// control weight
static constexpr int kStableRounds = 3;
double w_freq_ = 0.0;

// trace / control switch
bool freq_gate_trace_only_ = true;     // 第一版建议默认 true
bool freq_control_enabled_ = false;    // trace 验证后再打开
```

建议参数初值：

| 参数 | 初值 | 含义 |
|---|---:|---|
| `kStableRounds` | `3` | 对齐 BBRv2 full-pipe estimator 的三轮判断 |
| `kFullBwGrowth` | `1.25` | delivery rate 增长超过 25% 视为仍未收敛 |
| `kExitOneRound` | `0.25` | 单 round 波动超过 25% 退出稳定 |
| `kExitTwoRound` | `0.15` | 连续两个 round 波动超过 15% 退出稳定 |
| `scale_min` | `0.75` | 第一版 pacing scale 下界 |
| `scale_max` | `1.10` | 第一版 pacing scale 上界 |
| `A_ACTIVE` | 沿用当前配置 | CRUISE 三角波幅度 |

后续 trace 稳定后，`scale_min/scale_max` 可以放宽到 `[0.5, 1.25]`。

---

## 3. Fresh round-level delivery-rate 的维护

### 3.1 更新原则

`D_round_` 表示当前 packet-timed round 内的最大 fresh non-app-limited delivery-rate sample。

只纳入满足以下条件的 sample：

```cpp
sample_valid == true
sample_is_app_limited == false
in_recovery == false
```

如果要进一步降低 ACK compression 对 `D_round_` 的影响，可以在 trace 之后追加最小样本数量限制，例如：

```cpp
round_non_app_limited_sample_count >= 2 或 4
```

但第一版不建议直接改成 mean/median，因为 BBRv2 full-pipe 语义本来更接近“高 delivery-rate sample 是否仍显著增长”，而不是平均速率是否稳定。

---

### 3.2 伪代码

每次收到 BBRv2 sample view 时执行：

```cpp
void FreqCCv4Sender::OnBbr2Sample(const FreqBbr2SampleView& view) {
  if (view.round_start) {
    // 当前 round_start 到来时，上一轮 D_round_ 已完成。
    FinalizeCompletedRound();

    D_prev_ = D_round_;
    D_prev_valid_ = D_round_valid_;

    D_round_ = QuicBandwidth::Zero();
    D_round_valid_ = false;
  }

  if (view.sample_valid &&
      !view.sample_is_app_limited &&
      !view.in_recovery) {
    if (!D_round_valid_ || view.sample_max_bandwidth > D_round_) {
      D_round_ = view.sample_max_bandwidth;
      D_round_valid_ = true;
    }
  }
}
```

注意：

- `FinalizeCompletedRound()` 必须在 reset `D_round_` 之前执行；
- `D_prev_` 与 `D_round_` 都必须是 valid 才能计算波动率；
- 不要用 max-filtered `MaxBandwidth()` 或 `BandwidthEstimate()` 替代 `D_round_`。

---

## 4. 退出稳定收敛状态：用 fresh delivery-rate 波动，不用 MaxBw 波动

### 4.1 为什么不用 `BBR.max_bw`

`BBR.max_bw` 是 max-filtered model variable，不是 fresh observation。

它的优点是抗短时低估，缺点是：

1. 真实 delivery rate 已经下降时，旧高样本可能仍停留在 max-filter 中；
2. 短时 ACK compression、队列释放或 ProbeBW_UP 后的高样本可能抬高 `max_bw` 并保持一段时间；
3. 用它做退出稳定判断，会把“模型记忆值”的波动误当作“当前网络交付能力”的波动。

因此，退出稳定判断应使用：

```math
D_r = fresh\ non\text{-}app\text{-}limited\ delivery\text{-}rate\ indicator
```

而不是：

```math
M_r = BBR.max\_bw
```

---

### 4.2 退出稳定条件

定义 round 级波动率：

```math
V_r=\frac{|D_r-D_{r-1}|}{D_{r-1}}
```

当满足任一条件时，认为 BBRv2 退出稳定收敛区域：

```math
V_r > 0.25
```

或：

```math
V_r > 0.15 \land V_{r-1} > 0.15
```

不区分上跳和下跳。

原因是：

- 上跳可能是真实带宽增加，也可能是 ProbeBW_UP、队列释放、ACK aggregation 造成的高估；
- 下跳可能是真实竞争增强，也可能是短期采样扰动；
- 方向本身不能说明可信度，**波动幅度才说明 BBRv2 当前带宽模型可信度下降**。

### 4.3 退出稳定伪代码

```cpp
void FreqCCv4Sender::CheckExitStable() {
  if (!bbr_stable_) return;
  if (!D_round_valid_ || !D_prev_valid_) return;
  if (D_prev_ <= QuicBandwidth::Zero()) return;

  double V = AbsBandwidthDiffRatio(D_round_, D_prev_);

  bool volatility =
      (V > kExitOneRound) ||
      (V > kExitTwoRound && prev_V_valid_ && prev_V_ > kExitTwoRound);

  if (volatility) {
    bbr_stable_ = false;
    stable_cnt_ = 0;

    // 失稳后重新以当前 D_round_ 为 full-pipe 参考基线。
    full_drate_ref_ = D_round_;
    full_drate_ref_valid_ = true;

    // 频域工具在后续 ProbeBW_CRUISE 中重新开启。
    // 权重由 stable_cnt_ 映射得到，此时 stable_cnt_=0，所以 w_freq_=1。
    UpdateFrequencyWeight();
  }

  prev_V_ = V;
  prev_V_valid_ = true;
}
```

---

## 5. 重新进入稳定收敛：沿用 BBRv2 的 25% / 3 rounds 语义

### 5.1 判断逻辑

BBRv2 full-pipe estimator 的核心语义是：

> 如果连续 3 个 non-app-limited round 中，delivery rate 增长均不足 25%，则认为已经观测到 full-pipe / bandwidth plateau。

FreqCCv4 的重新收敛也沿用这个语义。

设当前 round 的 fresh delivery-rate 指标为 `D_round_`，参考基线为 `full_drate_ref_`。

如果：

```math
D_r \ge 1.25D_{ref}
```

则说明 delivery rate 仍在显著增长：

```math
D_{ref}\leftarrow D_r
```

```math
stable\_cnt\leftarrow0
```

如果：

```math
D_r < 1.25D_{ref}
```

则说明这一轮没有出现超过 25% 的显著增长：

```math
stable\_cnt\leftarrow stable\_cnt+1
```

当：

```math
stable\_cnt \ge 3
```

则认为 BBRv2 重新进入稳定收敛区域。

---

### 5.2 重新收敛伪代码

```cpp
void FreqCCv4Sender::UpdateReconvergenceEvidence() {
  if (bbr_stable_) return;
  if (!D_round_valid_) return;

  if (!full_drate_ref_valid_) {
    full_drate_ref_ = D_round_;
    full_drate_ref_valid_ = true;
    stable_cnt_ = 0;
    UpdateFrequencyWeight();
    return;
  }

  if (D_round_ >= kFullBwGrowth * full_drate_ref_) {
    // 仍在显著增长，不认为稳定。
    full_drate_ref_ = D_round_;
    stable_cnt_ = 0;
  } else {
    stable_cnt_ = std::min(stable_cnt_ + 1, kStableRounds);
  }

  if (stable_cnt_ >= kStableRounds && IsInProbeBW()) {
    bbr_stable_ = true;
    stable_cnt_ = kStableRounds;
  }

  UpdateFrequencyWeight();
}
```

---

## 6. 最新修正：频域权重随重新收敛证据递减，权重归零后同步关闭调制

### 6.1 不采用“关闭调制后再用旧频域值过渡”

不建议实现如下逻辑：

```text
bbr_stable = true
modulation_amp = 0
继续用旧 F_ref 参与 2–4 个 round 的 B_eff 过渡
```

原因是：

- 调制关闭后，`F_ref` 不再被刷新；
- 继续使用旧 `F_ref` 会引入 stale reference；
- 如果 BBRv2 已经满足 3-round full-pipe 收敛证据，则应认为原生 BBRv2 模型已足以支撑稳态控制。

正确逻辑应为：

> **在 BBRv2 重新收敛过程中，也就是 `stable_cnt = 0,1,2` 时，频域工具继续开启，且频域参考权重随 `stable_cnt` 增大而递减。到 `stable_cnt = 3` 时，频域权重降为 0，并同步关闭频域调制。**

---

### 6.2 权重映射公式

定义：

```math
K_{stable}=3
```

```math
w_f = 1 - \frac{stable\_cnt}{K_{stable}}
```

并限制在 `[0,1]`：

```math
w_f = clip\left(1 - \frac{stable\_cnt}{3}, 0, 1\right)
```

对应关系：

| `stable_cnt` | BBRv2 重新收敛证据 | 频域权重 `w_f` | CRUISE 频域工具 |
|---:|---|---:|---|
| 0 | 尚无稳定证据 | 1 | ON |
| 1 | 1 个 round 无 25% 显著增长 | 2/3 | ON |
| 2 | 2 个 round 无 25% 显著增长 | 1/3 | ON |
| 3 | 3 个 round 无 25% 显著增长 | 0 | OFF |

也就是说：

```text
只要 w_f > 0，频域工具仍然有机会刷新 F_ref；
一旦频域工具关闭，w_f 必须已经等于 0。
```

---

### 6.3 权重更新伪代码

```cpp
void FreqCCv4Sender::UpdateFrequencyWeight() {
  if (bbr_stable_) {
    w_freq_ = 0.0;
    return;
  }

  double w = 1.0 - static_cast<double>(stable_cnt_) /
                    static_cast<double>(kStableRounds);
  w_freq_ = Clamp(w, 0.0, 1.0);
}
```

---

## 7. CRUISE 阶段频域工具开关

### 7.1 开启条件

在当前设计中，不再额外引入 safe-to-probe 判断。原因是：

- 频域工具只放在 BBRv2 ProbeBW_CRUISE 阶段；
- CRUISE 阶段本身已有 ECN/loss 相关检测和 BBRv2 原生控制；
- 三角波调制是零均值扰动，不改变长期 inflight 注入水平；
- 该调制的目的不是持续改变发送水平，而是产生可识别频域响应。

因此，开关规则为：

```cpp
freq_tool_on = !bbr_stable_ && IsProbeBwCruise();
```

### 7.2 关闭条件

当：

```cpp
bbr_stable_ == true
```

此时必然：

```cpp
stable_cnt_ == 3
w_freq_ == 0
```

同步关闭频域调制：

```cpp
freq_tool_on = false;
modulation_amp = 0;
```

### 7.3 `ShouldOscillate()` 修改建议

当前 `ShouldOscillate()` 只看阶段、幅度和频率。需要加上收敛门控。

```cpp
bool FreqCCv4Sender::ShouldOscillate() const {
  if (bbr_stable_) return false;
  if (!IsProbeBwCruise()) return false;
  if (modulation_amp_ <= 0) return false;
  if (modulation_frequency_ <= 0) return false;
  return true;
}
```

---

## 8. 频域参考值 `F_ref` 的生成与有效期

### 8.1 更新位置

在 `FinalizeCruise()` 或等价的 CRUISE 窗口结束逻辑中更新：

```cpp
if (credible_window_found) {
  F_ref_ = best_window_mean_delivery_rate;
  F_conf_ = best_window_score;
  F_ref_valid_ = true;
  F_ref_update_round_ = current_round_index_;
}
```

`F_ref_` 的含义是：

```math
F_{ref}=mean(DRate(t)),\quad t\in W^*
```

其中 `W*` 是频域响应得分最高的可信窗口。

### 8.2 什么时候使用 `F_ref`

`F_ref_` 只有在以下条件同时满足时参与 pacing：

```cpp
!bbr_stable_
F_ref_valid_
w_freq_ > 0
```

如果 `bbr_stable_ == true`，则即使 `F_ref_valid_ == true`，也不能继续使用：

```cpp
if (bbr_stable_) {
  F_ref_valid_ = false;   // 或至少忽略它
}
```

### 8.3 防止 stale reference

建议增加一个极短 TTL，只在不稳定阶段防止长时间使用旧 `F_ref`：

```cpp
static constexpr int kMaxFreqRefAgeRounds = 1;  // 保守第一版
```

使用条件：

```cpp
bool FRefUsable() const {
  if (!F_ref_valid_) return false;
  if (bbr_stable_) return false;
  if (w_freq_ <= 0.0) return false;
  if (current_round_index_ - F_ref_update_round_ > kMaxFreqRefAgeRounds) return false;
  return true;
}
```

如果 trace 后发现 `FinalizeCruise()` 更新频率较低，可以把 `kMaxFreqRefAgeRounds` 放宽到 2 或按 ProbeBW cycle 计数。

核心原则不变：

> **调制关闭后不能继续用旧 `F_ref`。**

---

## 9. MaxBw / native estimate 与频域值的融合

### 9.1 原生参考值

第一版：

```math
B_{native}=BandwidthEstimate()
```

不要直接用：

```math
BBR.max\_bw
```

也不要写当前仓库没有实现的完整：

```math
\min(BBR.max\_bw,bw\_hi,bw\_lo)
```

### 9.2 频域融合

如果 `F_ref` 可用：

```math
B_{target}=(1-w_f)B_{native}+w_fF_{ref}
```

如果 `F_ref` 不可用：

```math
B_{target}=B_{native}
```

注意：

- 不区分 `F_ref < B_native` 还是 `F_ref > B_native`；
- 频域值不是单纯 downward cap；
- 它是 BBRv2 失稳阶段的 credible delivery-rate reference。

### 9.3 Pacing 层 scale

```math
scale=\frac{B_{target}}{B_{native}}
```

建议第一版限制：

```math
scale\in[0.75,1.10]
```

后续可放宽：

```math
scale\in[0.5,1.25]
```

如果 `B_native` 为 0 或无效：

```cpp
scale = 1.0;
```

### 9.4 PacingRate 修改

```cpp
QuicBandwidth FreqCCv4Sender::PacingRate() const {
  QuicBandwidth base = Bbr2Sender::PacingRate();

  double scale = 1.0;
  if (freq_control_enabled_ && FRefUsable()) {
    QuicBandwidth B_native = BandwidthEstimate();
    QuicBandwidth B_target =
        (1.0 - w_freq_) * B_native + w_freq_ * F_ref_;

    if (B_native > QuicBandwidth::Zero()) {
      scale = B_target / B_native;
      scale = Clamp(scale, scale_min_, scale_max_);
    }
  }

  double osc = 0.0;
  if (ShouldOscillate()) {
    osc = modulation_amp_ * TriangleWave(modulation_frequency_, now);
  }

  return base * scale * (1.0 + osc);
}
```

说明：

- `scale` 是频域参考反馈到控制面的通道；
- `osc` 是 CRUISE 阶段频域激励；
- 当 `bbr_stable_ == true` 时，`ShouldOscillate()` 为 false，`FRefUsable()` 也为 false，因此完全回到原生 BBRv2。

---

## 10. 完整状态机

### State 1：Native BBRv2 Stable

条件：

```cpp
bbr_stable_ == true
stable_cnt_ == 3
w_freq_ == 0
```

行为：

```cpp
freq_tool_on = false;
modulation_amp = 0;
F_ref 不参与 pacing;
B_target = B_native;
```

退出条件：

```math
V_r > 0.25
```

或：

```math
V_r > 0.15 \land V_{r-1} > 0.15
```

---

### State 2：Unstable / Frequency Dominant

进入条件：

```cpp
volatility_trigger == true
```

状态初始化：

```cpp
bbr_stable_ = false;
stable_cnt_ = 0;
full_drate_ref_ = D_round_;
w_freq_ = 1;
```

行为：

```cpp
if (IsProbeBwCruise()) freq_tool_on = true;
if (FRefUsable()) B_target = F_ref;
else B_target = B_native;
```

---

### State 3：Re-converging / Shared Reference

条件：

```cpp
bbr_stable_ == false
stable_cnt_ == 1 或 2
```

权重：

```math
w_f=1-\frac{stable\_cnt}{3}
```

行为：

```cpp
freq_tool_on = IsProbeBwCruise();
B_target = (1 - w_freq_) * B_native + w_freq_ * F_ref;
```

如果 `F_ref` 不可用，则：

```cpp
B_target = B_native;
```

---

### State 4：Handover Completed

条件：

```cpp
stable_cnt_ == 3
```

同步执行：

```cpp
bbr_stable_ = true;
w_freq_ = 0;
freq_tool_on = false;
F_ref_valid_ = false;  // 或忽略
B_target = B_native;
```

---

## 11. `FinalizeCompletedRound()` 汇总伪代码

```cpp
void FreqCCv4Sender::FinalizeCompletedRound() {
  if (!D_round_valid_) return;

  // 1. 如果当前稳定，先检查是否退出稳定。
  if (bbr_stable_) {
    CheckExitStable();
  }

  // 2. 如果当前不稳定，更新重新收敛证据。
  if (!bbr_stable_) {
    UpdateReconvergenceEvidence();
  }

  // 3. 根据 stable_cnt/bbr_stable 更新频域权重。
  UpdateFrequencyWeight();

  // 4. 稳定后必须停止使用 F_ref。
  if (bbr_stable_) {
    F_ref_valid_ = false;
  }

  // 5. trace 所有关键变量。
  TraceFreqGateState();
}
```

注意顺序：

1. 稳定状态下先检测退出；
2. 退出后立即进入不稳定，并用当前 `D_round_` 初始化 `full_drate_ref_`；
3. 不稳定状态下更新 25% / 3 rounds 重新收敛证据；
4. 权重由 `stable_cnt_` 映射得到；
5. `stable_cnt_ == 3` 后权重归零并关闭调制。

---

## 12. Trace-only 验证清单

第一版强烈建议先 trace-only，不直接启用 `freq_control_enabled_`。

必须 trace：

```text
round_id
D_round
D_prev
D_round_valid
D_prev_valid
V_round
prev_V_round
full_drate_ref
stable_cnt
bbr_stable
IsProbeBwCruise
freq_tool_on
ShouldOscillate
F_ref
F_conf
F_ref_valid
F_ref_age_rounds
w_freq
B_native
B_target
pacing_scale
base_pacing_rate
final_pacing_rate
```

重点检查：

1. 稳定长流中 `bbr_stable` 是否频繁抖动；
2. `V_round > 25%` 是否经常由单个异常 ACK sample 触发；
3. `stable_cnt` 是否能在稳定阶段自然增长到 3；
4. `w_freq` 是否按照 `1, 2/3, 1/3, 0` 下降；
5. `freq_tool_on` 是否只在 `!bbr_stable && CRUISE` 时打开；
6. `bbr_stable == true` 后 `F_ref` 是否不再参与 pacing；
7. `pacing_scale` 是否被 clamp 住，没有频繁大幅跳变。

如果发现 `D_round` 对 ACK compression 过敏，再考虑增加：

```cpp
round_non_app_limited_sample_count >= 2 或 4
```

或者把退出检测从 single round 改为 cycle-level fresh sample 聚合，但聚合对象仍然必须是 fresh non-app-limited delivery-rate，而不是 max-filtered `MaxBw`。

---

## 13. Codex 实现顺序

建议按以下顺序让 Codex 落代码。

### Step 1：补 sample view hook

在 `Bbr2Sender` / `Bbr2NetworkModel` / congestion event 路径补只读 hook，暴露：

```cpp
round_start
sample_valid
sample_max_bandwidth
sample_is_app_limited
in_recovery
```

不要用 `ExportDebugState().last_sample_is_app_limited` 替代。

---

### Step 2：在 `FreqCCv4Sender` 维护 round 级变量

新增：

```cpp
D_round_
D_prev_
full_drate_ref_
stable_cnt_
bbr_stable_
prev_V_
```

实现：

```cpp
OnBbr2Sample()
FinalizeCompletedRound()
CheckExitStable()
UpdateReconvergenceEvidence()
UpdateFrequencyWeight()
```

---

### Step 3：修改 `ShouldOscillate()`

添加门控：

```cpp
!bbr_stable_ && IsProbeBwCruise()
```

---

### Step 4：在 `FinalizeCruise()` 更新 `F_ref`

当找到可信频域窗口后：

```cpp
F_ref_ = best_window_mean_delivery_rate;
F_conf_ = best_window_score;
F_ref_valid_ = true;
F_ref_update_round_ = current_round_index_;
```

---

### Step 5：在 `PacingRate()` 层接入 scale

第一版：

```cpp
B_native = BandwidthEstimate();
B_target = (1 - w_freq_) * B_native + w_freq_ * F_ref_;
scale = Clamp(B_target / B_native, scale_min, scale_max);
```

最终：

```cpp
return Bbr2Sender::PacingRate() * scale * (1 + triangle_offset);
```

但默认先 trace-only：

```cpp
freq_control_enabled_ = false;
```

确认 trace 正常后再打开。

---

## 14. 论文/方案表述版本

### 中文

FreqCCv4 使用 fresh non-app-limited delivery-rate 的周期波动来判断 BBRv2 是否退出稳定收敛区域，而不是使用经过 max-filter 的 `MaxBw`。当 round 级 delivery-rate 指标在单个 round 内波动超过 25%，或在两个连续 round 内均波动超过 15% 时，认为 BBRv2 当前带宽模型可信度下降，并在后续 ProbeBW_CRUISE 阶段重新开启频域工具。频域工具从可信频域响应窗口中提取交付速率参考值 `F_ref`，并在 pacing 层与原生 BBRv2 带宽估计 `B_native` 进行加权融合。

在 BBRv2 重新收敛过程中，频域工具保持开启，频域参考权重根据 BBRv2 的三轮 full-pipe 证据逐步下降：`w_f = 1 - stable_cnt / 3`。当连续三个 non-app-limited round 中 delivery rate 均未出现超过 25% 的增长时，认为 BBRv2 原生带宽模型已足以支撑稳态控制，此时频域权重降为 0，并同步关闭 CRUISE 阶段频域调制。该设计避免了调制关闭后继续使用陈旧频域参考值，同时实现了从频域稳定参考向原生 BBRv2 控制的平滑交还。

### English

FreqCCv4 detects the departure from BBRv2's stable-convergence region using volatility in fresh non-application-limited delivery-rate observations, rather than the max-filtered `MaxBw`. If the round-level delivery-rate indicator changes by more than 25% in one round, or by more than 15% over two consecutive rounds, BBRv2's bandwidth model is considered temporarily unreliable, and the frequency-domain tool is reactivated in subsequent ProbeBW_CRUISE phases. The tool extracts a credible delivery-rate reference `F_ref` from the best frequency-domain response window and fuses it with the native BBRv2 bandwidth estimate `B_native` at the pacing layer.

During BBRv2's re-convergence process, the frequency-domain tool remains active, while the frequency-reference weight decays according to BBRv2's three-round full-pipe evidence: `w_f = 1 - stable_cnt / 3`. Once three consecutive non-application-limited rounds show no delivery-rate increase above 25%, BBRv2's native bandwidth model is considered sufficiently reliable for steady-state control. At that point, the frequency weight becomes zero and CRUISE-phase modulation is disabled simultaneously. This avoids using stale frequency-domain references after modulation is turned off, while enabling a smooth handover back to native BBRv2 control.

---

## 15. 参考依据

- BBR / BBRv2 的基本模型基于 delivery rate、RTT 和 loss rate 构建网络路径模型，并据此控制发送速率与 inflight。
- BBR full-pipe estimator 的核心依据是：连续若干 non-app-limited rounds 中 delivery rate 增长不足 25%，即可认为已经观测到 full pipe / bandwidth plateau。
- BBR 的 delivery-rate sampler 需要标记 app-limited 样本，因为 app-limited 阶段的 delivery rate 不能代表拥塞控制允许的网络交付能力。

参考文档：

1. IETF CCWG BBR Draft: `https://ietf-wg-ccwg.github.io/draft-ietf-ccwg-bbr/draft-ietf-ccwg-bbr.html`
2. BBR Congestion Control Internet-Draft: `https://datatracker.ietf.org/doc/html/draft-cardwell-iccrg-bbr-congestion-control`
3. Cardwell et al., BBR: Congestion-Based Congestion Control, ACM Queue, 2016.
