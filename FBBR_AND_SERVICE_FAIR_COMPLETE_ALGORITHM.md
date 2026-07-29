# FBBR 与 FBBR-ServiceFair 完整算法说明

> 本文只说明两种实际入口：FBBR 和 FBBR-ServiceFair。
> 两者共享频率激励、SRTT 时域裁剪判定、Goertzel DRate 分量判定、可信带宽、服务一致性 inflight
> envelope；ServiceFair 只在 Cruise baseline、过载候选和 TrustedBw
> 发布处增加公平控制。

> 本文以 NS3.27/src/dqc/model/thirdparty/congestion/fbbr_sender.cc
> 与 fbbr_sender.h 的当前实现为准。文中不描述其他拥塞控制分支，也不把
> 观察量误写成控制量。

## 1. 范围、身份与已核对的边界

### 1.1 两个算法

| 外部名称 | 枚举 | 行为 |
|---|---|---|
| FBBR | kFBBR | FBBR pacing、波形负载判定、TrustedBw、服务一致性 cwnd 上限 |
| FBBR-ServiceFair | kFBBRServiceFair | 完整继承 FBBR，并在 Cruise 加入 service/qdelay 公平控制 |

二者共用以下状态和公式：

- 最终 pacing target 与 base target 的分段历史；
- 累计 delivered 的阶梯历史；
- planned inflight、positive probe credit、service inflight；
- service-consistent envelope 与最终 cwnd cap；
- SRTT 时域窗口、Goertzel DRate 分量判定、N01--N16 分类、GuardBw、TrustedBw；
- 原生 BBRv2 的 MaxBw、MinRtt、ProbeBW phase、ProbeRTT、loss/recovery
  和 ACK aggregation。

ServiceFair 不创建第二个 BBR model，也不创建第二套 envelope。

### 1.2 明确不属于本文的内容

FBBR 和 ServiceFair 不改写原生 BBRv2 的：

- Startup、Drain、ProbeBW 和 ProbeRTT 状态机；
- MaxBw filter、MinRtt 基础测量、loss/recovery；
- inflight_hi、inflight_lo；
- native cwnd 的内部计算；
- MaxAckHeight ACK aggregation headroom。

FBBR 的新增执行器只位于最终 cwnd getter：

\[
Cwnd_{final}=\min(Cwnd_{native},I_{cap})
\]

它不会把 cap、queue debt 或 envelope 反写到上述原生状态。

### 1.3 Goertzel 核对

FBBR 与 ServiceFair 现在在每个有效 Cruise 窗口中实际运行项目根目录
`Goertzel算法.md` 所定义的**广义 Goertzel**。它只计算配置注入频率

\[
f_0=cruise\_modulation\_freq\_hz
\]

处的复 DTFT 系数，不计算完整频谱，也不搜索峰值频率。它的唯一控制用途是：
确认实际 sender rate 和 DRate 是否都含有该注入频率的相干分量。

- 输入：统一重采样后的 sender rate 与 DRate；
- 输出：复系数的实部、虚部、相位、功率、幅值估计、归一化相干功率和
  `component_present`；
- 最终判断：DRate window、两侧 Goertzel 输入均有效，且
  `sender_present && drate_present`，才是 DRate match；
- 不比较两条曲线的逐点形状、相关系数、周期误差或相位；相位只写 trace，
  因为反馈传播时间会改变它；
- 该结果进入 N01、N02、N03、N04、N07、N08、N13 和 no-wave retry。

SRTT 的 horizontal line、shoulder、clip 与时间加权均值仍是时域算法，负责
识别负载状态和统计窗口；它们不再替代 DRate 的频域分量判定。

`kFBBR` 与 `kFBBRServiceFair` 始终锁定为 `time_waveform` 检测器。
即使配置文件填写 `cruise_detector.mode=legacy_spectral`，
`ConfigureFBBR()` 也会对这两个入口强制选择 `time_waveform`；旧 FFT/频谱
窗口选择只保留给不使用 service envelope 的其他分支。因此本文的 Goertzel
路径不是可选回退，而是这两个算法当前唯一的 sender--DRate 频域判定。

## 2. 符号、单位与总览

| 符号 | 含义 | 单位 |
|---|---|---|
| \(t\) | 单调时刻 | s 或 us |
| \(R(t)\) | 实际返回给 pacer 的最终目标速率 | bit/s |
| \(R_b(t)\) | 去除正向 probing 后的 base target | bit/s |
| \(B\) | 当前 Cruise injection baseline | bit/s |
| \(A_{req}\) | 配置模式请求的单边频率激励振幅 | bit/s |
| \(A_{eff}\) | 当前采样窗口实际发出的单边激励振幅 | bit/s |
| \(f\) | 激励频率 | Hz |
| \(T=1/f\) | 激励周期 | s |
| \(\tau_{fb}\) | 当前 probe epoch RTT；sender 与反馈窗口的对齐延迟 | s |
| \(P\) | 当前有效 RTprop | s |
| \(D(t)\) | 原生 sampler 的累计 acked/delivered bytes | bytes |
| \(I_{plan}\) | 一个 RTprop 内按实际 pacing 计划发送的 bytes | bytes |
| \(I_{probe}^{+}\) | 已实际命令的正向 probing bytes | bytes |
| \(I_{service}\) | 一个 RTprop 内已确认的服务 bytes | bytes |
| \(I_{env}\) | 服务一致性 envelope | bytes |
| \(E_{ack}\) | 原生 MaxAckHeight() | bytes |
| \(MSS\) | kDefaultTCPMSS | bytes |

速率到字节的统一换算：

\[
Bytes=\frac{Rate_{bps}\times Duration_{us}}{8{,}000{,}000}
\]

控制链路如下：

~~~text
原生 BBRv2
  -> pacing 基线 / ProbeBW phase / native cwnd
  -> FBBR 频率激励与 SRTT 时域观察 / Goertzel DRate 分量检测
  -> FBBR Regime I/II/III 与 TrustedBw
  -> 最终 target/base 分段历史
  -> delivered 阶梯历史
  -> service-consistent envelope
  -> min(native cwnd, envelope cap)

ServiceFair 仅在 Cruise baseline、Regime III、TrustedBw 发布处插入控制。
~~~

## 3. 运行时顺序

### 3.1 发送事件

每次 OnPacketSent 的顺序是：

1. 设置 current time；
2. 调用 PacingRate；
3. 形成最终 \(R(t)\) 与 base target \(R_b(t)\)；
4. 对 FBBR/ServiceFair 记录或延长 rate segment；
5. 记录发送端实际 pacing 历史；
6. 调用原生 BBRv2 的 OnPacketSent。

因此 planned inflight 使用的是 pacer 真正收到的速率，不是一个预估的
MaxBw，也不是之后才计算的频率振幅。

### 3.2 ACK/congestion event

每次 OnCongestionEvent 的关键顺序：

1. 收集本批 ACK bytes；
2. 用当前波形响应修正原生 MaxBw/MinBw sample 的激励污染；
3. 先运行原生 BBRv2 事件处理；
4. 原生 model 更新累计 acked、delivery rate、app-limited、cwnd、phase；
5. 记录 DRate、SRTT、ACK window 和 GuardBw 所需样本；
6. 有 ACK 时记录更新后的累计 delivered point；
7. ServiceFair 有 ACK 时更新 qdelay 与 service rate；
8. 在 ProbeBW Cruise 中推进时域波形状态机；
9. 更新 envelope telemetry。

累计 delivered 必须在原生 BBRv2 处理之后记录，才能保证 \(D(t)\) 与本次
ACK 对应。

### 3.3 读取 cwnd

FBBR 或 ServiceFair 读取 cwnd 时：

1. 构造当前 envelope snapshot；
2. 若 history 未覆盖完整 RTprop、尚未完成初始 Drain，或不在 ProbeBW，
   原样返回 \(Cwnd_{native}\)；
3. 否则返回 \(\min(Cwnd_{native},I_{cap})\)。

## 4. FBBR pacing 与频率激励

### 4.1 激励的启用条件

频率激励仅在所有条件成立时启用：

- 已完成初始 Drain；
- 当前 mode 为 ProbeBW；
- 当前 phase 为 Probe Cruise；
- Cruise 已进入；
- \(f>0\)、\(A_{eff}>0\)；
- current injection baseline、probe epoch、波形状态都有效；
- 若 convergence gate 开启，当前不处于 stable 状态。

不满足时，FBBR 返回原生或 TrustedBw 路径形成的普通 pacing，不叠加波形。
当 \(B\le R_{floor}\) 时，波形本身关闭，但 Cruise 状态机与普通 Regime
判定继续运行；此窗口不会因“无波形”进入 signal-fidelity retry。

### 4.2 Cruise baseline 与最终 target

在时域 Cruise 内，baseline 为当前 \(B\)，phase gain 固定为 1。非 Cruise
阶段可使用当前 phase gain \(g(t)\) 乘以可用 TrustedBw；不可用时退回
原生 BBR pacing。

令波形 offset 为 \(O(t)\)，最终 pacing 为：

\[
R(t)=\max(R_{floor},\ B_{pace}(t)+
    \operatorname{int64}(A_{eff}W(t)))
\]

其中：

- \(B_{pace}\) 是 Cruise baseline、TrustedBw phase gain 或 native pacing
  三者中当前路径实际选择的值；
- \(R_{floor}\) 是 minimum_pacing_rate_bps；
- 实现对三角波乘法、基线加法都做 int64 饱和保护，再通过已有的有下限加法形成
  最终整数 bit/s。

FBBR 还记录 base target \(R_b(t)\)。它走与 \(R(t)\) 相同的 baseline、限速、
单位和取整路径，但：

- 正的 BBR phase gain 被截为 1；
- 正的 waveform offset 被移除；
- 负的 phase 和负半周期仍保留；
- 始终保证 \(R_b(t)\le R(t)\)。

可写成：

\[
R_b(t)=
\max\left(R_{floor},B_{base}(t)+\min(0,O(t))\right)
\]

\[
B_{base}(t)=
\begin{cases}
B,& \text{Cruise}\\
\min(g(t),1)\times TrustedBw,& \text{可信带宽路径}\\
\text{去除正 phase 后的 native pacing},& \text{native 路径}
\end{cases}
\]

base target 只用于识别已经实际发出的正向 probing credit，不是另一个 pacer
命令。

### 4.3 三角波

时域波形以 probe epoch 为零点，且固定为负半周期先行。令：

\[
q=\frac{(t-t_{epoch})\bmod T}{T},\qquad T=\frac{1}{f}
\]

\[
W(q)=
\begin{cases}
-4q,&0\le q<0.25\\
4q-2,&0.25\le q<0.75\\
4-4q,&0.75\le q<1
\end{cases}
\]

所以一个周期是：

~~~text
0 -> -1 -> 0 -> +1 -> 0
~~~

\[
O(t)=\operatorname{int64}(A_{eff}\cdot W(q))
\]

这个 offset 的使用位置只有 PacingRate；FBBR base target 的负向路径使用相同
的 floor 和整数保护。波形观察器从 ACK 侧反向确认响应，不会直接把某个单独
SRTT 样本加到 pacing 上。

### 4.4 振幅

振幅来自当前配置模式：

| 模式 | 公式 |
|---|---|
| fixed | \(A_{req}=A_{fixed}\) |
| MIU2/3/4/8 | \(A_{req}=MaxBw/\{2,3,4,8\}\) |
| SR2/3/4/8/12/16 | \(A_{req}=NativePacing/\{2,3,4,8,12,16\}\) |

Cruise 进入时把当前值保存为初始 probe amplitude。不确定窗口的重试会将当前
振幅乘以 1.25，但不得超过初始振幅的 2 倍：

\[
A_{retry}=\min(1.25A_{current},2A_{initial})
\]

每次进入新的采样窗口、以及 baseline 调整后重新采样前，使用当前
current_injection_baseline_bw_ 重新计算保护振幅。令 \(F=R_{floor}\)，则：

\[
A_{eff}=
\begin{cases}
0,&B\le F\\
\min(A_{req},B-F),&B>F
\end{cases}
\]

因此负峰满足 \(B-A_{eff}\ge F\)，不会被 final floor 压成一段平线。
裁剪只影响当前窗口的 \(A_{eff}\)，不修改 \(A_{req}\)；baseline 回升时会自动
恢复原请求。若窗口被裁剪，或 \(B\le F\) 而无波动，则不会执行上述 1.25 倍
不确定结果增幅。默认 pacing.minimum_rate_mbps 为 0.2 Mbps，但该值始终可由
配置覆盖。

该放大只在连续不确定且允许重试时使用；它不改变长期 TrustedBw。

### 4.5 convergence gate

每个 round 从非 app-limited、非 recovery 的 delivery-rate sample 取最大值
\(D_{round}\)，相邻 round 相对变化：

\[
v_{round}=\frac{|D_{round}-D_{prev}|}{D_{prev}}
\]

默认规则：

- 单轮 \(v_{round}>0.25\) 退出稳定；
- 当前和上一轮都 \(>0.15\) 退出稳定；
- 连续 3 个无 full-pipe growth 的 round 进入 stable；
- 若相对 reference 增长达到 \(1.25\)，更新 reference 并重置计数。

gate 关闭激励，但不会清空 envelope history，也不会修改原生 cwnd。

## 5. Cruise 窗口、重采样与原始样本

### 5.1 原始 history

FBBR/ServiceFair 在内存维护四类输入：

| history | 样本 | 使用位置 |
|---|---|---|
| sender rate | 实际返回的最终 pacing target | Goertzel 确认发送端确实含有 \(f_0\) 分量 |
| delivery rate | 最新 delivery rate、ACK bytes、app-limited | Goertzel 确认 DRate 含有 \(f_0\) 分量；窗口 min/max 与负载分类 |
| SRTT | smoothed RTT | qdelay、SRTT 波形、时间加权均值 |
| delivered counter | 原生累计 acked bytes | interval service rate、service inflight、envelope |

前三类是波形观察 history；第四类是服务历史。两类历史不能互相替代。

对任一反馈分析窗口 \([a,b]\)，sender rate 取 \([a-\tau_{fb},
b-\tau_{fb}]\)，而 DRate 与 SRTT 取 \([a,b]\)。这里
\(\tau_{fb}=\texttt{probe\_epoch\_rtt\_}\)，用来把发出的激励与其 ACK
侧响应放到同一观察区间。该时间平移不是相位门限：Goertzel 的相位仍只进入
trace，不参与分类。

### 5.2 窗口状态机

一个 Cruise 的 SRTT 时域检测与 DRate 频域检测顺序：

~~~text
进入 Cruise
  -> 等待 1 个 probe-epoch RTT
  -> 收集初始 2 个周期
  -> 分析
  -> 不确定时最多扩至 3 个周期
  -> 仍不可判定时按规则增大激励后重新等待和采样
  -> 已得到可信结果或 Cruise 结束
~~~

默认 5 Hz 时 \(T=200\) ms。每次 baseline 调整后再次等待一个 RTT，避免把
旧 pacing 对应的反馈与新 baseline 混在同一窗口。

### 5.3 统一重采样

观察器不把不规则 ACK 时间直接送入形状判断。采样步长：

\[
h=\operatorname{clamp}\left(\frac{T}{40},1\text{ ms},5\text{ ms}\right)
\]

最大允许插值 gap：

\[
G_{max}=0.10T
\]

对于窗口 \([t_0,t_1]\)，网格数：

\[
N=\left\lfloor\frac{t_1-t_0}{h}\right\rfloor+1
\]

原始样本 \(x_k\) 映射到最近网格：

\[
i_k=\left\lfloor\frac{t_k-t_0}{h}+0.5\right\rfloor
\]

同一 bin 的值取中位数：

\[
x_i=\operatorname{median}\{x_k\mid i_k=i\}
\]

若两个有效 bin 的间隔不大于 \(G_{max}\)，其间缺口做线性插值：

\[
x_{i} = x_l+\frac{i-l}{r-l}(x_r-x_l),\quad l<i<r
\]

超过 \(G_{max}\) 的缺口保持 invalid，不伪造数据。覆盖率为：

\[
coverage=\frac{\#\{i\mid valid_i\}}{N}
\]

重采样既是 SRTT 时域裁剪识别的输入，也是 Goertzel 的均匀时间基。对
Goertzel，FBBR/ServiceFair 不会把 invalid bin 填零、填均值或跳过后压缩时间；
sender 或 DRate 任一网格存在 invalid，目标分量结果就标为无效。这样不会把
缺样模式误解释为频率能量。

### 5.4 输入有效性

DRate window 有效必须满足：

- 原始 DRate 样本不少于 4；
- ACK bytes 总和大于 0；
- \(coverage\ge0.85\)；
- app-limited 样本比例不大于 0.25。

SRTT 有两层有效性，不能混为一个 bool：

- `srtt_input_valid`：原始 SRTT 样本不少于 4，且重采样
  \(coverage\ge0.85\)；
- `srtt_stats_valid`：先满足上述统计前提，再由第 7 节的时间加权均值取得完整
  左边界 anchor。N10--N12、N14--N16 的 SRTT min/max 阈值分支读取这个状态；
- SRTT ordinary-wave/clip 还有独立的两周期检查：清理后的每个周期至少有 20 个
  有效网格。N01--N09 的 clip case 读取该 ordinary-wave 输入状态，而非用
  time-weighted mean 代替它。

除上述 DRate window 条件外，Goertzel match 还要求：

- sender 与 DRate 的重采样向量长度相同且至少有 4 点；
- 两条向量的每个网格均为 finite 且 valid；
- \(0<f_0<f_s/2\)，其中 \(f_s=1/h\)。

任一条件不满足时，`component_present` 不产生假阳性，依赖 DRate match 的
N 规则输出 INCONCLUSIVE。

## 6. 辅助算法

本节列出仍会在 FBBR/ServiceFair 路径上执行的细节算法。每项都给出计算和
实际用途。未特别标记为固定语义的数值均为当前默认配置；相应的 FBBRConfig
字段可调整阈值，但不会改变计算顺序或控制落点。

### 6.1 三点中值滤波

对三个相邻且有效的值：

\[
\tilde{x_i}=\operatorname{median}(x_{i-1},x_i,x_{i+1})
\]

首尾或邻点无效时保留原值。它用于：

- ordinary-wave activity 的抗尖峰处理；
- SRTT 局部斜率、平台、shoulder 判断。

它不改写原始累计 delivered，也不用于 service inflight 或 Goertzel 的 DRate
match。

### 6.2 分位数、MAD 噪声、振幅和有效步

对排序后样本 \(x_{(0)},\ldots,x_{(n-1)}\)，分位数使用线性插值。令：

\[
p=q(n-1),\quad l=\lfloor p\rfloor,\quad u=\lceil p\rceil
\]

\[
Q_q=
\begin{cases}
x_{(l)},&l=u\\
x_{(l)}+(p-l)(x_{(u)}-x_{(l)}),&l\ne u
\end{cases}
\]

先用三点中值滤波得到残差 \(e_i=x_i-\tilde{x_i}\)。鲁棒噪声：

\[
\sigma=1.4826\cdot\operatorname{median}
\left(|e_i-\operatorname{median}(e)|\right)
\]

一个周期的振幅定义为：

\[
A_{obs}=Q_{0.95}(x)-Q_{0.05}(x)
\]

水平量级：

\[
L=\max(|\operatorname{median}(x)|,10^{-12})
\]

振幅必须满足：

\[
A_{obs}\ge\max(6\sigma,0.02L)
\]

相邻网格的有效变化门限：

\[
\theta_{step}=
\max\left(3\sqrt{2}\sigma,\ 3.5A_{obs}\frac{h}{T}\right)
\]

当 \(|x_i-x_{i-1}|\ge\theta_{step}\) 时才计为 active step。每周期还要求
至少 4 个 active step，或至少占有效 steps 的 10%。

用途：排除 ACK compression 的尖刺、纯平滑漂移和低幅噪声，决定 SRTT
ordinary wave 是否存在。

### 6.3 ordinary-wave activity

记上升总量为 \(U\)，下降总量绝对值为 \(V\)，路径长度为 \(L_p=U+V\)。
完整波要求：

\[
U\ge0.20A_{obs},\quad
V\ge0.20A_{obs},\quad
L_p\ge0.80A_{obs}
\]

并至少发生一次 slope sign reversal。半周期波允许其中一个方向不超过
\(0.25A_{obs}\)，但另一个方向与路径长度仍必须通过门限。

用途：这是 SRTT clip 证据生效的先决条件。只有“水平线”而没有可观察的
SRTT 波形，不能直接当作裁剪，而应进入 fallback/retry。

SRTT ordinary-wave 使用去除边缘水平段和中间扰动后的有效视图。它只作为
SRTT clip 证据的前提；DRate 是否有发送频率响应由下一节的 Goertzel 结果
单独决定。

### 6.4 广义 Goertzel：目标频率分量与 DRate match

这一节是 FBBR/ServiceFair 的 sender--DRate 相似判断本体。它不是估计
DRate 的周期，也不要求 DRate 的三角形形状、振幅或相位与 sender 相同；只问：

> 两条实际观测到的速率序列是否都含有注入频率 \(f_0\) 的相干分量？

令均匀重采样后的某一条序列为 \(x[0],\ldots,x[N-1]\)，先去 DC：

\[
y[n]=x[n]-\bar{x},\qquad
\bar{x}=\frac{1}{N}\sum_{n=0}^{N-1}x[n]
\]

采样率 \(f_s=1/h\)，直接使用物理目标频率而不是把它 round 到整数 DFT bin：

\[
\omega_0=2\pi\frac{f_0}{f_s}=2\pi f_0h
\]

初始化 \(s[-1]=s[-2]=0\)，对每一个样本运行项目内 `Goertzel算法.md` 的二阶
递推：

\[
s[n]=y[n]+2\cos(\omega_0)s[n-1]-s[n-2]
\]

最后两个状态恢复目标频率处的**复数输出**：

\[
\operatorname{Re}X=s[N-1]-\cos(\omega_0)s[N-2]
\]

\[
\operatorname{Im}X=\sin(\omega_0)s[N-2]
\]

因此 Goertzel 的原始输出不是 bool，而是

\[
X(f_0)=\operatorname{Re}X+j\operatorname{Im}X
\]

代码随后计算：

\[
P(f_0)=|X(f_0)|^2=(\operatorname{Re}X)^2+(\operatorname{Im}X)^2
\]

\[
A_{f_0}=\frac{2\sqrt{P(f_0)}}{N}
\]

其中 \(A_{f_0}\) 是未加窗正弦的幅值估计，仅用于 trace；它不是判决门限。
为了消除窗口长度和速率量纲，实际判决量是归一化相干功率：

\[
\boxed{
\eta(f_0)=
\frac{P(f_0)}{N\sum_{n=0}^{N-1}y[n]^2}
}
\]

由 Cauchy--Schwarz 不等式，\(0\le\eta\le1\)。纯目标正弦在正常窗口中约为
\(0.5\)；三角激励的基频也会得到高值。当前分量存在判决为：

\[
present(x)=
\bigl[\eta(f_0)\ge\theta_G\bigr],qquad
\theta_G=0.10
\]

\(\theta_G\) 由 `goertzel.min_coherent_power_ratio` 配置，默认 \(0.10\)，
解析时允许范围为 \([10^{-6},1]\)。常量序列返回 `NO_VARIATION` 并视为分量
不存在；不完整、非有限或违反 Nyquist 的输入返回无效，而不是补样后继续计算。

对 sender 得到 \((valid_s,present_s)\)，对 DRate 得到
\((valid_d,present_d)\)。最终 FBBR 的 DRate match 是：

\[
\boxed{
match_{DRate}=
valid_{DRate-window}\land valid_s\land valid_d\land
present_s\land present_d
}
\]

这就是现有字段 `drate_similar` 的精确定义：**同频分量共同存在**，不是逐点
波形相似度。\(\arg X=\operatorname{atan2}(\operatorname{Im}X,
\operatorname{Re}X)\) 同样输出到 trace，但不参与 `match_{DRate}`，因为 RTT
反馈延迟会改变响应相位。

实际计算 sender 分量时，分析窗口 \([a,b]\) 会先按第 5.1 节移到
\([a-\tau_{fb},b-\tau_{fb}]\)；DRate 保持在 \([a,b]\)。两条序列各自去 DC
后再运行相同的递推，所以没有要求它们拥有相同振幅、波形形状或相位。

Goertzel helper 的返回状态也区分“可用但未命中”和“输入无效”：

| `decision_reason` | `input_valid` | `component_present` | 含义 |
|---|---:|---:|---|
| `TARGET_COMPONENT_PRESENT` | true | true | 目标频率分量达到 \(\theta_G\) |
| `TARGET_COMPONENT_ABSENT` | true | false | 输入完整，但目标分量不足 |
| `NO_VARIATION` | true | false | 去 DC 后能量低于数值下限 |
| `INVALID_SERIES`、`INVALID_TARGET_FREQUENCY` | false | false | 样本数/向量结构/步长或 Nyquist 前提不成立 |
| `INCOMPLETE_RESAMPLED_SERIES`、`NONFINITE_GOERTZEL_OUTPUT` | false | false | 存在 invalid/non-finite 网格，或计算结果不有限 |

实际调用位置是 FBBR/ServiceFair Cruise 窗口的 sender rate 与 DRate 重采样
完成后；N01、N02、N03、N04、N07、N08、N13 读取该结果，two-window no-wave
retry 也读取它。该 match 的控制输入只有本节列出的 valid/present；其他
DRate 形状、周期和相位指标均不参与 FBBR/ServiceFair 的判决。

### 6.5 平台、裁剪和 shoulder

SRTT 时域检测先找连续水平段，再识别 top/bottom repeated contact 和两侧
shoulder。核心条件包括：

| 证据 | 主要门限 | 用途 |
|---|---|---|
| 连续 SRTT 水平段 | 有效覆盖至少 0.85，flat fraction 至少 0.90 | 候选 long top/bottom line |
| 局部平坦 | 局部斜率不超过振幅归一化门限 0.10 | 排除倾斜趋势 |
| 平台稳定 | level span 不超过 0.10，total drift 不超过 0.05 | 排除缓慢漂移 |
| long top line | 持续至少 \(0.20T\) | N03/N04 |
| long bottom line | 持续至少 \(0.30T\) | N07/N08 |
| repeated contact | 至少 2 个周期接触、至少 4 个总接触样本 | N05/N09 |

shoulder 使用相邻两侧局部线性拟合。若两侧斜率方向相反，且各自的
\(|slope|\times duration\) 都达到最小变化量，则是有效的 opposing shoulder。
正 shoulder 是上界证据，负 shoulder 是下界证据。

用途：这些 SRTT 证据决定 N01--N11 的 clip case。它们只影响分类和 RTprop
信息更新；需要 DRate 响应的规则再读取 6.4 节的 Goertzel match。

### 6.6 鲁棒队列梯度

FBBR 时域窗口还计算排队延迟的鲁棒梯度。令每个有效 SRTT 样本的队列量为：

\[
q_i=\max(0,SRTT_i-P)
\]

先按 \(Q_{0.05}\) 与 \(Q_{0.95}\) winsorize：

\[
\tilde{q_i}=\operatorname{clamp}(q_i,Q_{0.05},Q_{0.95})
\]

对 \((t_i,\tilde{q_i})\) 做线性回归：

\[
g_{raw}=
\frac{\sum_i(t_i-\bar{t})(\tilde{q_i}-\bar{q})}
{\sum_i(t_i-\bar{t})^2}
\]

相邻梯度：

\[
d_i=\frac{\tilde{q_i}-\tilde{q}_{i-1}}{t_i-t_{i-1}}
\]

其噪声为：

\[
\sigma_g=1.4826\cdot\operatorname{median}
\left(|d_i-\operatorname{median}(d)|\right)
\]

最终：

\[
g=
\operatorname{clamp}\left(
\begin{cases}
0,&|g_{raw}|\le2\sigma_g\\
g_{raw},&\text{否则}
\end{cases},
-0.5,0.5\right)
\]

用途：它在 FBBR/ServiceFair 的时域窗口中计算并记录队列变化质量、q90 和
梯度 trace。当前 FBBR Regime executor 不读取该梯度决定 baseline，因此它是
观察量，不能被解释成另一条降速公式。

## 7. 时间加权 SRTT 均值

这是 FBBR/ServiceFair 实际使用的均值，不是“按 ACK 个数平均”。

### 7.1 阶梯保持语义

对 SRTT history，时刻 \(s_j\) 的样本 \(r_j\) 在下一有效样本出现前保持：

\[
SRTT(t)=r_j,\quad t\in[s_j,s_{j+1})
\]

窗口 \([a,b]\) 必须有一个 \(s_j\le a\) 的左边界 anchor。没有 anchor 直接
判无效，不能把窗口内第一条 ACK 样本倒灌到窗口起点。

### 7.2 公式

令各历史片段与窗口的重叠时长为：

\[
\Delta t_j=
\max\left(0,\min(s_{j+1},b)-\max(s_j,a)\right)
\]

最后一个片段的 \(s_{j+1}\) 取 \(b\)。时间加权均值：

\[
\boxed{
\overline{SRTT}_{time}=
\frac{\sum_j r_j\Delta t_j}{b-a}
}
\]

实现同时要求：

\[
\sum_j\Delta t_j=b-a
\]

否则覆盖不完整，结果无效。

### 7.3 使用位置

在每个 FBBR/ServiceFair 时域窗口中：

1. 先由重采样序列得到 SRTT min、max 和候选统计；
2. 若 SRTT 输入有效，调用时间加权计算替换窗口 mean；
3. 将该结果与 SRTT stats validity 一起送入 FBBR 分类窗口；
4. 无 anchor 或不完整覆盖时，不把 SRTT window 当成有效分类输入；
5. 同时写入 trace，避免 ACK 密度差异改变“均值”含义。

N01--N16 的阈值直接使用 SRTT min/max；时间加权 mean 的重要作用是保证
窗口统计具有真实时间语义并作为 SRTT window 完整性的有效性门。

## 8. FBBR 负载分类

### 8.1 分类输出

时域窗口最终只输出四类之一：

| 输出 | 含义 | FBBR 动作 |
|---|---|---|
| UNDERLOAD | Regime I | 尝试提高 baseline |
| FULL_LOAD | Regime II | 保持或以窗口累计 delivered 锁定 baseline/TrustedBw |
| OVERLOAD | Regime III | 降低 baseline |
| INCONCLUSIVE | 输入不足或证据冲突 | hold，等待下一窗口 |

### 8.2 N01--N16 判定树

SRTT clip case 只有在 SRTT input valid 且 ordinary wave 存在时生效。优先级：

~~~text
U1 positive shoulder
U2 long top line
U3 repeated top clip
L1 negative shoulder
L2 long bottom line
L3 repeated bottom clip
无 clip，进入波形/阈值 fallback
~~~

| 规则 | 条件 | 输出 | 信息层副作用 |
|---|---|---|---|
| N01 | U1 且 DRate Goertzel match | FULL_LOAD | 无 |
| N02 | U1 且 DRate Goertzel mismatch | OVERLOAD | 无 |
| N03 | U2 且 DRate Goertzel match | FULL_LOAD | 无 |
| N04 | U2 且 DRate Goertzel mismatch | OVERLOAD | 无 |
| N05 | U3 repeated top clip | OVERLOAD | 无 |
| N06 | L1 negative shoulder | FULL_LOAD | 无 |
| N07 | L2 且 DRate Goertzel match | UNDERLOAD | 刷新 RTprop 与低位 DRate |
| N08 | L2 且 DRate Goertzel mismatch | FULL_LOAD | 无 |
| N09 | L3 repeated bottom clip | UNDERLOAD | 更新低位 DRate |
| N10 | 无 clip、SRTT 有 wave 且 SRTTmax 超历史 MaxSRTT | OVERLOAD | 无 |
| N11 | 无 clip、SRTT 有 wave 且 SRTTmin 低于 RTprop | UNDERLOAD | 刷新 RTprop 与低位 DRate |
| N12 | 无 clip、SRTT 有 wave 的阈值 fallback | I/II/III | 无 |
| N13 | 无 clip、SRTT 无 wave、DRate Goertzel match | UNDERLOAD | 无 |
| N14 | 无 clip、SRTT 无 wave、SRTTmax 超 MaxSRTT | OVERLOAD | 无 |
| N15 | 无 clip、SRTT 无 wave、SRTTmin 低于 RTprop | UNDERLOAD | 刷新 RTprop 与低位 DRate |
| N16 | 无 clip、SRTT 无 wave 的阈值 fallback | I/II/III | 无 |

任何需要 DRate Goertzel match 的规则，若 Goertzel 输入无效，则输出
INCONCLUSIVE；若输入有效而任一侧目标分量不存在，则是 mismatch，不是无效。

### 8.3 字段映射与两窗 no-wave retry

在 FBBR/ServiceFair 的 service-envelope 路径里，兼容字段的语义已经改为：

\[
\texttt{drate.wave.input\_valid}=
valid_{DRate-window}\land valid_s\land valid_d
\]

\[
\texttt{drate.wave.has\_wave}=
\texttt{drate.periodic\_similar}=match_{DRate}
\]

因此 N07 读取的 `drate.wave.has_wave`、N01/N03/N13 读取的
`drate.periodic == kMatch`，以及 N02/N04/N08 读取的 `kNoMatch`，都是同一
Goertzel 判定的不同接口名；它们不再调用 DRate 的 ordinary-wave、肩部、裁剪
或自相关逻辑。

SRTT 与 DRate 的 no-wave streak 分开计数，只对输入有效的唯一两周期窗口更新：

- 任一信号有 wave 时将自己的 streak 清零；有效但无 wave 时加一；无效输入不加；
- 任一 streak 连续达到 2 时，当前分类改为 INCONCLUSIVE，分类副作用和状态更新
  都被冻结，并进入 rolling retry；
- retry 激活期间，直到 SRTT ordinary wave 或 DRate Goertzel match 任一恢复，
  才退出并清空两个 streak；否则继续抑制当前窗口的分类。

这条机制解释了为什么 Goertzel mismatch 在 N02/N04/N08 中可以作为有效证据，
但当 SRTT ordinary wave 也连续缺失时，连续有效的 DRate 无分量仍会触发后续的
探针增强/重采样，而不会永久把旧结论直接施加到 baseline。

### 8.4 N12/N16 的三段阈值

前提是 SRTT stats、RTprop、MaxSRTT 都有效，且：

\[
MaxSRTT\ge RTprop>0
\]

过载门限：

\[
T_{overload}=
\max\left(1.10RTprop,\
RTprop+\frac{MaxSRTT-RTprop}{3}\right)
\]

分类：

| 条件 | 输出 |
|---|---|
| \(SRTT_{max}>T_{overload}\) | OVERLOAD |
| \(SRTT_{max}<1.05RTprop\) | UNDERLOAD |
| 其余 | FULL_LOAD |

这不是 BDP/inflight 门限，也不读取 SRTT mean。

### 8.5 RTprop 与低位 DRate 的信息更新

FBBR 当前使用的窗口长度 \(P\) 为：

\[
P=
\begin{cases}
SRTT_{low},&\text{已有有效的 FBBR 低位 RTT 观测}\\
MinRtt_{native},&\text{否则}
\end{cases}
\]

N07、N11、N15 在分类有效时，将当前窗口的 \(SRTT_{min}\) 发布为新的
\(SRTT_{low}\)，并记录：

\[
DRate_{low}=d_{min}
\]

N09 只记录 \(DRate_{low}=d_{min}\)，不刷新 RTprop。发布时间会保留最小
1 us 的数值下限，避免零 RTT。

\(P\) 的使用位置包括时域窗口长度、planned inflight 积分、delivered service
窗口和 ServiceFair 的 \(\alpha\)/qdelay 阈值。低位 DRate 是信息/trace 状态；
FBBR 的 baseline 仍只按第 9.1 节的 Regime executor 更新。

## 9. FBBR baseline、GuardBw 与 TrustedBw

### 9.1 FBBR Regime executor

记：

- \(m=MinBw\)；
- \(M=MaxBw\)；
- \(d_{min},d_{max}\) 为当前有效 DRate window 最小/最大值；
- \(B\) 为当前 Cruise baseline；
- \(R_{min}\) 为 minimum pacing rate。

任何 DRate 输入无效或 INCONCLUSIVE 都保持 baseline 不变。

#### Regime I：UNDERLOAD

\[
B'=
\begin{cases}
d_{max},& B<d_{max}<M\\
\frac{d_{max}+M}{2},&
d_{max}<M\ \land\ \frac{d_{max}+M}{2}>B\\
1.02B,& \text{其他情况}
\end{cases}
\]

最终：

\[
B'=\max(R_{min},B')
\]

用途：利用窗口中已看到的最高 delivered 或 native MaxBw 中点上探；两者都
没有给出更高候选时才做 2% 增长。

#### Regime II：FULL_LOAD

若本 Cruise 尚未发生过 Regime I 或 III，保持当前 baseline。

若本 Cruise 已发生过 I 或 III，必须从窗口边界的累计 delivered counter
得到独立候选：

\[
R_{II}=
\frac{8[D(t_{end})-D(t_{start})]}{t_{end}-t_{start}}
\]

只有完整 anchor、无 counter reset、分母正且 counter 单调时该值有效。
有效时：

\[
B'=TrustedBw=R_{II}
\]

无效时保持 baseline，不回退到 DRate 样本均值伪装成测量。

#### Regime III：OVERLOAD

\[
B'=
\begin{cases}
d_{min},&m<d_{min}<B\\
\frac{m+d_{min}}{2},&
d_{min}>m\ \land\ \frac{m+d_{min}}{2}<B\\
0.98B,& \text{其他情况}
\end{cases}
\]

\[
B'=\max(R_{min},B')
\]

这只修改当前 Cruise baseline；它不直接写 native MaxBw、inflight_hi 或
inflight_lo。

### 9.2 GuardBw：两极低通

Guard 原始速率窗口从累计 delivered 计算：

\[
G_{raw}=
\frac{8\Delta D}
{\max(\Delta t_{ack},\Delta t_{send})}
\]

样本前提：

- 已 ACK bytes 大于 0；
- send/ack 时间都有效；
- 窗口长度至少为 \(\max(50\text{ ms},CurrentRTT)\)；
- delivery elapsed 不小于可用 min RTT；
- 整个窗口不 app-limited。

两级低通：

\[
G_1[n]=\frac{7G_1[n-1]+G_{raw}[n]}{8}
\]

\[
G_2[n]=\frac{7G_2[n-1]+G_1[n]}{8}
\]

\(G_2\) 才是可发布的 GuardBw；第一极从不单独作为 pacing 控制值。可信
波形结果发布时，会把两个 stage 同时锚定到该 TrustedBw。

### 9.3 TrustedBw 选择

FBBR Cruise 结束时优先级：

1. 合法 Regime II 的窗口累计 delivered rate；
2. 本 Cruise 更新过的 GuardBw；
3. FBBR 可用的上一轮 TrustedBw；
4. native MaxBw、Cruise 初值、BandwidthEstimate、pacing floor 的顺序
   fallback。

ServiceFair 在 Cruise 入口和 ProbeBW pacing 中可以使用上一轮 TrustedBw
初始化或维持 baseline；但 Cruise 结束时，若没有 Regime II 或 Guard 候选，
它选择 native fallback，再执行第 11.5 节的有限修正。两者都不会把 fallback
直接写入历史积分。

TrustedBw 影响后续 pacing baseline；它不会改写 delivered history、target
history 或 envelope 过去的积分。

### 9.4 激励对原生带宽样本的校正

波形本身会抬高或压低 delivery sample。为避免把正半周期误当作长期容量，
FBBR 在调用原生 BBRv2 事件处理前使用已观测响应幅度校正 sample。

令预测 delivery 中心为 \(C\)，实际响应幅度为 \(A_r\)：

\[
A_r=response\_gain\times A_{emitted}
\]

MaxBw sample 衰减：

\[
\gamma_{max}=\frac{C}{C+A_r}
\]

MinBw sample 的反向校正仅在 \(0<A_r<C\) 时启用：

\[
\gamma_{min}=\frac{C}{C-A_r}
\]

不满足数值前提时两个因子均为 1。用途是把频率激励从 native MaxBw/MinBw
测量中尽量剥离；它不改变最终 envelope 公式。

## 10. FBBR 服务一致性 inflight envelope

### 10.1 rate segment history

每个 rate segment 保存：

~~~text
[start, end), target_rate, base_target_rate
~~~

目标改变时关闭前一段并开始新段；同一时刻更新直接替换打开段，不生成零长度
段。时间倒退会使 history integrity 失效。

历史保留覆盖边界所需的 anchor segment。任何间隙、无效速率、
\(R_b>R\)、RTprop 非法或未覆盖完整窗口都会禁用 projection。

### 10.2 planned inflight

令窗口为 \([t-P,t]\)。对每个与窗口相交的 segment \(j\)，重叠时间为
\(\Delta t_j\)：

\[
\boxed{
I_{plan}(t)=
\sum_j\frac{R_j\Delta t_j}{8{,}000{,}000}
}
\]

最终按最近整数 byte 取整并饱和到 QuicByteCount 上限。

用途：表示 FBBR 实际命令在一个 RTprop 内应形成的 inflight，不使用
TrustedBw 的静态乘积替代。

### 10.3 positive probe credit

只计算已经命令的正差：

\[
\boxed{
I_{probe}^{+}(t)=
\sum_j\frac{\max(R_j-R_{b,j},0)\Delta t_j}
{8{,}000{,}000}
}
\]

它包含正 phase gain 和正三角波在实际 target 中留下的增量；不包含：

- 负 phase 或负半周期；
- 未真正进入 target 的“潜在 probe”；
- 固定百分比 headroom；
- 额外 gain。

用途：容量上升时允许已发出的 probing 暂时扩张 envelope，随后由真实
delivered service 验证。

### 10.4 delivered 阶梯历史

每个点为：

~~~text
(timestamp, cumulative_delivered_bytes, app_limited)
~~~

它来自原生 model 的 total_bytes_acked。语义是：

\[
D(t)=D(t_i),\quad t\in[t_i,t_{i+1})
\]

处理规则：

- 时间倒退：history integrity 失效；
- 同一 timestamp：counter 取最大值，app-limited 做 OR；
- 新 timestamp 但 counter 下降：认为发生新 counter generation，
  清空旧 history、记录 reset 时刻；
- 清理时保留边界之前最近一个阶梯 anchor。

不对累计 delivered 做线性插值，也不用 application sent bytes 或单个
delivery-rate sample 代替。

### 10.5 service history 有效性

窗口 \([t-P,t]\) 的 service history 必须同时满足：

- \(P\) 有限且大于 0；
- 存在 \(t-P\) 左侧或恰在边界的 anchor；
- history integrity 有效；
- 窗口内没有 counter reset；
- counter 从 anchor 到当前单调不减；
- 当前以及从 anchor 到当前的全部样本都非 app-limited；
- 当前时刻至少已过一个 RTprop。

任一项失败时，服务量不应被解释为路径容量下降。

### 10.6 service inflight 与 envelope

\[
I_{service}(t)=\max(0,D(t)-D(t-P))
\]

服务预算使用饱和加法：

\[
I_{budget}=\operatorname{SatAdd}(I_{service},I_{probe}^{+})
\]

\[
\boxed{
I_{env}(t)=
\begin{cases}
\min(I_{plan},I_{budget}),&\text{service history valid}\\
I_{plan},&\text{否则}
\end{cases}
}
\]

含义：

- 稳定服务量足以覆盖计划时：\(I_{env}=I_{plan}\)；
- 实际服务下降：\(I_{service}+I_{probe}^{+}<I_{plan}\)，上限收缩；
- 发现 app-limited、reset 或缺 anchor：退回计划上限，不把应用空闲当作
  拥塞；
- 正向 probing 只提供已发出数据的信用，不凭空制造容量。

### 10.7 native headroom 与最终 cap

\[
E_{ack}=MaxAckHeight()
\]

当前 offload budget 固定为 0。先饱和加法：

\[
I_{raw}=\operatorname{SatAdd}(
\operatorname{SatAdd}(I_{env},E_{ack}),0)
\]

\[
I_{floor}=\max(MinPipeCwnd,I_{raw})
\]

然后按 MSS 向上对齐：

\[
I_{cap}=
\begin{cases}
I_{floor},&I_{floor}\bmod MSS=0\\
I_{floor}+MSS-(I_{floor}\bmod MSS),&\text{否则}
\end{cases}
\]

\[
\boxed{Cwnd_{final}=\min(Cwnd_{native},I_{cap})}
\]

若实际 inflight 已高于 cap，FBBR 不丢包、不把 cap 抬到当前 inflight、不写
持久 recovery 状态；仅停止新增发送，等待 ACK 自然排空。

### 10.8 激活条件和观测量

projection active 的必要条件：

~~~text
FBBR 或 ServiceFair
且 RTprop 有效
且 target history 覆盖完整 RTprop
且已完成 Drain
且当前在 ProbeBW
~~~

不要求 service history valid；无有效 service history 时只是退化为
\(I_{env}=I_{plan}\)。

以下量仅用于 trace/summary，不回控：

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

## 11. FBBR-ServiceFair

### 11.1 作用范围

ServiceFair 继承第 4--10 节全部 FBBR 行为，仅增加：

1. 每个 Cruise 至多一次的 service/qdelay AIMD；
2. Regime III 的 yield cap 与 service floor；
3. Cruise 结束时 TrustedBw 发布修正。

它不能直接修改 \(I_{service}\)、\(I_{probe}^{+}\)、\(I_{env}\)、\(I_{cap}\)、
MaxAckHeight、native MaxBw、native cwnd、ProbeBW phase 或 ProbeRTT。

### 11.2 ACK 信号：qdelay、EWMA 与 service rate

排队延迟样本：

\[
q_{sample}=\max(SRTT-RTprop,0)
\]

首次样本直接初始化；后续使用：

\[
q_{ewma}[n]=\frac{7q_{ewma}[n-1]+q_{sample}[n]}{8}
\]

这个 EWMA 仅在 ServiceFair、ProbeBW、ACK 到达时更新。

service rate 取同一个 RTprop 窗口的累计 delivered：

\[
S(t)=\frac{8[D(t)-D(t-P)]}{P}
\]

它复用第 10.5 节全部服务历史条件；任何 app-limited、reset、缺 anchor 或
无效 RTprop 都使 service history invalid，不允许 AIMD 使用该 rate。

### 11.3 每 Cruise AIMD

定义：

\[
\alpha=\frac{0.5\times8\times MSS}{P}
\]

\[
q_{low}=\max(1\text{ ms},0.03P)
\]

\[
q_{high}=\max(2\text{ ms},0.10P)
\]

\[
q_{trend}=q_{ewma,current}-q_{ewma,previous}
\]

\[
T_{trend}=q_{low}
\]

常数：

\[
\beta=0.995,\qquad R_{fair,min}=1\text{ Mbps}
\]

每个 Cruise 一进入函数就写入 update marker，因此包括 skip 在内，同一个
Cruise 都不会重复执行 AIMD。

前提失败时：

| 条件 | 动作 |
|---|---|
| baseline 无效 | SKIP_INVALID_HISTORY |
| 当前 app-limited 且 service history 无效 | SKIP_APP_LIMITED |
| RTprop、qdelay、service history、service rate 或 \(\alpha\) 无效 | SKIP_INVALID_HISTORY |

通过前提后：

| 条件 | 动作 | 新 baseline |
|---|---|---|
| \(q_{ewma}>q_{high}\)，或 \(q_{trend}>T_{trend}\) | MD | \(\min(B,\max(R_{fair,min},\beta B))\) |
| \(q_{ewma}<q_{low}\) | AI | \(B+\alpha\)，并且不高于 \(\max(B,MaxBw)\) |
| 其他 | HOLD | \(B\) |

它保存本轮 qdelay 和 service rate，下一 Cruise 再计算 trend 和 rate change；
rate change 本身只是 trace：

\[
\frac{S_{current}}{S_{previous}}-1
\]

### 11.4 Regime III 的公平限制

FBBR 先得到原始 Regime III 候选 \(C_{raw}\)。ServiceFair 再计算：

\[
C_{yield}=\beta B
\]

\[
C_0=\min(C_{raw},C_{yield})
\]

若 service history 有效且 \(S>0\)：

\[
C_{floor}=\min(C_{yield},0.98S)
\]

\[
\boxed{
C_{final}=\min(C_{yield},\max(C_0,C_{floor}))
}
\]

无有效 service history 时：

\[
C_{final}=C_0
\]

这保证 ServiceFair 在过载时绝不高于 \(0.995B\)，同时避免在已确认服务仍高时
把候选压到远低于近期真实服务率。

Regime I 与 II 的 FBBR 语义不变：

- I：不额外施加 ServiceFair 减速；
- II：合法累计 delivered 候选优先级最高，不允许被公平逻辑或 MaxBw 改写。

### 11.5 TrustedBw 发布修正

在 FBBR 选出候选之后，ServiceFair 只做以下修正：

| 条件 | 结果 |
|---|---|
| 合法 Regime II cumulative-delivery 候选 | 原样保留 |
| 本 Cruise 最后有效分类为 UNDERLOAD | \(\max(candidate,B_{final})\)，若 MaxBw 有效则不高于 MaxBw |
| 最后有效分类为 OVERLOAD | \(\min(candidate,B_{final})\)，有效 service 时可抬到不超过 \(\min(B_{final},0.98S)\) |
| 无有效分类但已经完成 AI 或 MD | \(B_{final}\) |
| HOLD、skip 或无有效历史 | 原候选不变 |

所有结果最终受 pacing floor 保护。该步骤只修改将要发布的 TrustedBw，不回写
已经形成的 target/base segment、delivered point 或 envelope snapshot。

### 11.6 生命周期与重置

以下情形清空 ServiceFair 私有信号并清除陈旧 TrustedBw：

- connection migration；
- 原生状态从非 Startup 回到 Startup；
- delivered counter 发生新 generation。

重置时刻不能被任意 service-rate 窗口跨越。

## 12. 联合伪代码

~~~text
OnPacketSent(now):
    target, base_target = PacingRate(now)
    RecordRateSegment(now, target, base_target)
    NativeBbrOnPacketSent()

OnCongestionEvent(event):
    ApplyWaveformSampleCorrectionToNativeModel()
    NativeBbrOnCongestionEvent(event)
    RecordSenderRateDeliveryRateAndSrtt(event)

    if event.has_ack:
        RecordDeliveredPoint(now, native_total_bytes_acked, app_limited)

    if algorithm == ServiceFair and event.has_ack:
        q = max(SRTT - RTprop, 0)
        q_ewma = EWMA_7_8(q_ewma, q)
        service_valid, service_rate =
            MeasureDeliveredRateOverOneRtprop()

    if in ProbeBW Cruise:
        window = BuildTimeWaveformWindow()
        if window is due:
            sender_component = Goertzel(sender_rate[a-tau_fb, b-tau_fb], f0)
            drate_component = Goertzel(delivery_rate, f0)
            drate_match = drate_window_valid \
                and sender_component.input_valid \
                and drate_component.input_valid \
                and sender_component.present and drate_component.present
            classification = ClassifyN01ToN16(window)
            raw = ExecuteFbbrRegime(classification)
            if algorithm == ServiceFair and classification == OVERLOAD:
                baseline = ApplyYieldCapAndServiceFloor(raw)
            else:
                baseline = raw

EnterCruise():
    baseline = PreviousTrustedOrNativeFallback()
    if algorithm == ServiceFair:
        RunAtMostOneAimdUpdateForThisCruise()
    WaitOneRttThenCollectWaveform()

FinalizeCruise():
    candidate = SelectFbbrTrustedBw()
    if algorithm == ServiceFair:
        candidate = CorrectTrustedBw(candidate)
    PublishTrustedBw(candidate)

GetCongestionWindow():
    plan = IntegrateTargetOverOneRtprop()
    credit = IntegratePositiveTargetMinusBase()
    service = DeliveredDifferenceOverOneRtprop()
    envelope = service_valid ? min(plan, service + credit) : plan
    cap = MssAlignUp(max(MinPipeCwnd, envelope + MaxAckHeight()))
    return projection_active ? min(native_cwnd, cap) : native_cwnd
~~~

## 13. 数值、安全与可观测性

### 13.1 数值规则

- 所有 bandwidth、时间、分母和浮点中间量必须有限且为正；
- byte 加法使用饱和加法；
- \(\Delta t\le0\)、counter 倒退、无左边界 anchor、无效 rate 均使相应
  计算无效；
- 所有 cap 最终不超过 QuicByteCount 最大值；
- MSS 对齐只能向上，不得因为截断小于 MinPipeCwnd；
- app-limited 是 service history 的污染条件，不设宽限时间或补偿 gain。

### 13.2 关键 trace

FBBR envelope trace 至少应能观察：

~~~text
target, base_target
plan_inflight, positive_probe_credit
service_inflight, service_history_valid, app_limited_contaminated
envelope, extra_acked, inflight_cap
native_cwnd, actual_inflight, service_restriction, enforced_excess
~~~

FBBR Cruise-window trace 还记录：

~~~text
goertzel_target_frequency_hz
sender_goertzel_{input_valid,component_present,real,imag,phase_rad,power,amplitude,coherent_power_ratio,reason}
drate_goertzel_{input_valid,component_present,real,imag,phase_rad,power,amplitude,coherent_power_ratio,reason}
goertzel_component_match
~~~

其中 `phase_rad` 只用于诊断传播延迟；控制判决只读取两侧的
`component_present` 与输入有效性。

ServiceFair trace 的关键字段：

~~~text
cruise_id, action
qdelay_ewma, qdelay_trend
service_rate, service_rate_change
alpha, beta
raw_regime_candidate, final_regime_candidate
~~~

### 13.3 应覆盖的自测

FBBR：

- 目标/base history 的连续覆盖和积分；
- 正向 probe credit 只计算真实正差；
- delivered 阶梯 anchor、同 timestamp merge、counter reset；
- app-limited 使 service history 失效；
- service envelope、ACK headroom、MSS 对齐和 cwnd min；
- N01--N16、Regime I/II/III、GuardBw；
- Goertzel 的目标频率、RTT 对齐、相位偏移、异频拒绝、非整数频率、常量输入和
  缺样拒绝；
- 两窗 no-wave streak：无效不计数、任一信号恢复后退出、冻结分类副作用；
- 时间加权 SRTT 对 ACK 密度不敏感，并要求左边界 anchor。

ServiceFair：

- qdelay EWMA 和 \(\alpha\) 的精确值；
- 每 Cruise 最多一次 AI/MD/hold/skip；
- Regime III yield cap 与 service floor；
- Regime II 候选不被改写；
- migration、Startup 回退、counter reset 后没有陈旧公平信号。

## 14. 实现索引

| 内容 | 主要实现位置 |
|---|---|
| 发送、ACK、pacing、cwnd 入口 | fbbr_sender.cc 的 OnPacketSent、OnCongestionEvent、PacingRate、GetCongestionWindow |
| SRTT 时域窗口、重采样、Goertzel、N01--N16 | fbbr_sender.cc 的 FBBR/ServiceFair Cruise 窗口分析与状态机 |
| Goertzel 递推、复数输出和判决量 | 根目录 `Goertzel算法.md`；fbbr_sender.cc 的 Goertzel helper |
| 时间加权 SRTT | fbbr_sender.cc 的 SRTT window helper |
| rate/delivered history 与 envelope | fbbr_sender.cc 的 FBBR service-envelope helpers |
| ServiceFair qdelay、AIMD、Regime III、发布修正 | fbbr_sender.cc 的 ServiceFair helpers |
| 配置与状态字段 | fbbr_sender.h 的 FBBRConfig 和 FBBRSender 状态 |

本文中的“FBBR”均指 kFBBR；“ServiceFair”均指 kFBBRServiceFair。除这两种
算法外，不需要通过本文推断任何其他分支的行为。
