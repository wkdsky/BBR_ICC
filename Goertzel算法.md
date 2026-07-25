# Goertzel 算法（校正版）

## 1. 基本概念

Goertzel 算法是一种用于计算离散信号在**少数几个指定频率点**上的 DFT/DTFT 值的高效算法。它常用于 DTMF 双音多频信号检测、窄带频率检测、嵌入式实时信号处理等场景。

设输入信号为：

$$
x[0],x[1],\ldots,x[N-1]
$$

目标频率为 $f_0$ Hz，采样频率为 $f_s$ Hz，则对应的归一化角频率为：

$$
\omega_0 = 2\pi \frac{f_0}{f_s}
$$

如果目标频率正好对应 $N$ 点 DFT 的第 $k$ 个频点，则有：

$$
\omega_0 = \frac{2\pi k}{N}, \qquad k=\frac{Nf_0}{f_s}
$$

在 DTMF 检测中，由于标准 DTMF 频率通常不一定刚好落在整数 DFT bin 上，工程上常使用：

$$
k \approx \operatorname{round}\left(\frac{Nf_0}{f_s}\right)
$$

或者采用非整数频点的 generalized Goertzel 形式。

---

## 2. DFT/DTFT 目标值

对于长度为 $N$ 的有限序列，目标频率 $\omega_0$ 处的 DFT/DTFT 采样值可写为：

$$
X(\omega_0)=\sum_{n=0}^{N-1}x[n]e^{-j\omega_0 n}
$$

当 $\omega_0=2\pi k/N$ 时，它就是 $N$ 点 DFT 的第 $k$ 个频率系数：

$$
X[k]=\sum_{n=0}^{N-1}x[n]e^{-j2\pi kn/N}
$$

Goertzel 算法的目的不是计算完整频谱，而是高效计算一个或少数几个 $X[k]$ 或 $X(\omega_0)$。

---

## 3. Goertzel 递推公式

Goertzel 算法可用一个二阶递推系统实现。定义状态变量 $s[n]$，初始条件为：

$$
s[-1]=s[-2]=0
$$

递推公式为：

$$
s[n]=x[n]+2\cos(\omega_0)s[n-1]-s[n-2],\qquad n=0,1,\ldots,N-1
$$

如果写成 DFT bin $k$ 的形式，则为：

$$
s[n]=x[n]+2\cos\left(\frac{2\pi k}{N}\right)s[n-1]-s[n-2]
$$

注意这里的反馈系数是：

$$
2\cos(\omega_0)
$$

而不是 $\cos(\omega_0)$。这一点很关键。

---

## 4. 由状态变量恢复 DFT 系数

处理完 $N$ 个采样点后，目标频率处的复频谱值可由最后两个状态恢复：

$$
X(\omega_0)=s[N-1]-e^{-j\omega_0}s[N-2]
$$

因此其实部和虚部可写为：

$$
\operatorname{Re}\{X(\omega_0)\}=s[N-1]-\cos(\omega_0)s[N-2]
$$

$$
\operatorname{Im}\{X(\omega_0)\}=\sin(\omega_0)s[N-2]
$$

不同资料中由于 DFT 符号约定、状态变量索引方式或最终相位修正方式不同，复数结果的相位表达可能略有差异；但用于频率检测时，通常关心的是幅值或能量，下面的模平方公式是一致的。

---

## 5. 幅值与能量计算

Goertzel 算法常用于检测某个频率是否存在，因此通常计算的是频率能量，即 DFT 系数的模平方：

$$
P(\omega_0)=|X(\omega_0)|^2
$$

由最后两个状态可以得到：

$$
P(\omega_0)
=s^2[N-1]+s^2[N-2]-2\cos(\omega_0)s[N-1]s[N-2]
$$

如果写成 $k$ 的形式：

$$
P[k]
=s^2[N-1]+s^2[N-2]
-2\cos\left(\frac{2\pi k}{N}\right)s[N-1]s[N-2]
$$

需要注意：上式得到的是**能量或功率谱意义上的模平方**，不是幅值本身。幅值应为：

$$
|X(\omega_0)|=\sqrt{P(\omega_0)}
$$

如果输入是实值正弦信号，且频率正好落在目标频点上、未加窗，则正弦幅度可近似估计为：

$$
A \approx \frac{2|X(\omega_0)|}{N}
$$

如果使用窗函数 $w[n]$，则应进行 coherent gain 修正：

$$
A \approx \frac{2|X(\omega_0)|}{\sum_{n=0}^{N-1}w[n]}
$$

---

## 6. 滤波器形式

递推状态 $s[n]$ 对应一个二阶 IIR 系统：

$$
H_s(z)=\frac{1}{1-2\cos(\omega_0)z^{-1}+z^{-2}}
$$

但 Goertzel 的最终输出并不是简单取该 IIR 系统的最后一个状态，而是还要通过最后一步：

$$
X(\omega_0)=s[N-1]-e^{-j\omega_0}s[N-2]
$$

因此，从输入到最终复频谱输出的一种常见传递函数表示为：

$$
H(z)=\frac{1-e^{-j\omega_0}z^{-1}}{1-2\cos(\omega_0)z^{-1}+z^{-2}}
$$

也有资料将其写成等价的相位变体，例如：

$$
H(z)=\frac{e^{j\omega_0}-z^{-1}}{1-2\cos(\omega_0)z^{-1}+z^{-2}}
$$

这两种写法主要相差一个相位因子；如果只关心幅值或能量检测，结果不受该整体相位差影响。

若希望得到归一化幅值，可以在最终结果中再除以 $N$，或按具体幅值定义进行归一化。

---

## 7. 用于 DTMF 检测的基本流程

DTMF 信号由一个低频组频率和一个高频组频率叠加而成。常见频率包括：

- 低频组：697 Hz、770 Hz、852 Hz、941 Hz
- 高频组：1209 Hz、1336 Hz、1477 Hz、1633 Hz

使用 Goertzel 进行 DTMF 检测时，一般流程如下：

1. 选择采样率 $f_s$ 和分析窗口长度 $N$；
2. 对每个 DTMF 候选频率 $f_i$ 计算对应的 $k_i$ 或 $\omega_i$；
3. 对每个候选频率运行一次 Goertzel 递推；
4. 计算每个候选频率处的能量 $P(\omega_i)$；
5. 在低频组和高频组中分别选择能量最大的频率；
6. 根据两个频率的组合判断按键。

Goertzel 适合 DTMF 的原因是：DTMF 只需要检测少数几个固定频率，没有必要计算完整 FFT 频谱。

---

## 8. 复杂度与适用场景

对于一个目标频率，Goertzel 的复杂度为：

$$
O(N)
$$

如果需要检测 $M$ 个目标频率，总复杂度为：

$$
O(MN)
$$

而完整 FFT 的复杂度通常为：

$$
O(N\log N)
$$

因此，当只需要少数几个频率点时，Goertzel 往往比 FFT 更合适；当需要大量频率点甚至完整频谱时，FFT 通常更高效。

---

## 9. 对原图片文字的主要修正

原图片转写内容中有几处需要修正：

1. **递推公式中的系数错误**  
   原式写成了类似 $\cos(2\pi f n)Q_{n-1}$ 的形式。标准 Goertzel 递推应为：

   $$
   s[n]=x[n]+2\cos(\omega_0)s[n-1]-s[n-2]
   $$

   系数应是固定的 $2\cos(\omega_0)$，不应随 $n$ 变化。

2. **“幅度 A”与“能量/模平方”混淆**  
   原式：

   $$
   Q_{N-1}^{2}+Q_{N-2}^{2}-2\cos(2\pi f)Q_{N-1}Q_{N-2}
   $$

   实际表示的是 $|X|^2$，即模平方或频率能量；若要幅值，应取平方根。

3. **滤波器传递函数写法不完整**  
   Goertzel 的状态递推对应：

   $$
   H_s(z)=\frac{1}{1-2\cos(\omega_0)z^{-1}+z^{-2}}
   $$

   最终复输出还需要包含一个前向修正项，因此常见完整形式为：

   $$
   H(z)=\frac{1-e^{-j\omega_0}z^{-1}}{1-2\cos(\omega_0)z^{-1}+z^{-2}}
   $$

4. **第二张图中的“前缀和差”推导不可靠**  
   更标准的推导是从单个 DFT 系数出发，将其转化为二阶递推结构，而不是将 $x[n]$ 写成原图中那种不严谨的前缀和差形式。

5. **频率符号需要区分**  
   如果 $f_0$ 是物理频率，公式中应使用 $\omega_0=2\pi f_0/f_s$；如果 $f$ 表示归一化频率 cycles/sample，则可写成 $\omega_0=2\pi f$。

---

## 参考资料

1. MathWorks, *goertzel — Discrete-Time Fourier transform with second-order Goertzel algorithm*.
2. STMicroelectronics, *The Goertzel algorithm to compute individual terms of the discrete Fourier transform (DFT)*, DT0089.
3. Texas Instruments, *Modified Goertzel Algorithm in DTMF Detection Using the TMS320C80 DSP*, SPRA066.


关于Goertzel是否适用于freqccv4的讨论：
可以用 Goertzel 做改进，但不适合直接一换一替代当前 FFTW 方案。按现
  在 freqccv4 的设计，当前 FFTW 更合适；如果后面只做在线窄带检测，
  Goertzel 更合适。

  当前实现用的是 FFTW：

  - NS3.27/src/dqc/model/thirdparty/congestion/freqccv4_sender.cc:9
    引入 <fftw3.h>

  - NS3.27/src/dqc/model/thirdparty/congestion/
    freqccv4_sender.cc:896 对重采样后的窗口信号做 Hann 去均值、零填
    充 FFT

  - 它不是只看配置频率 f0，而是在 [0.7f0, 1.3f0] 内找峰值，并输出：
      - peak_freq_hz
      - target_amp
      - noise_floor
      - band_shape
      - band_energy_rel / band_peak_rel

  这些结果后面会用于 drate_freq_score、srtt_freq_score、drate_gain、
  srtt_snr、drate_waveform_quality 等窗口评分。

  Goertzel 的优势是：只计算一个或少数几个指定频率点。你这个 Goertzel
  算法.md:1 里也明确说了，它适合“少数几个指定频率点”的 DFT/DTFT 检
  测。如果只想问：

  > drate / srtt 在调制频率 f0 上有没有明显能量？

  那 Goertzel 很合适，甚至比 FFTW 更轻、更容易在线化，也能减少 FFTW
  依赖。

  但如果要保留现在 FreqCCv4 的完整判断逻辑，单点 Goertzel 不够。因为
  当前逻辑需要知道：

  - 峰值是不是落在 f0 附近，而不只是 f0 有能量
  - 0.7f0 ~ 1.3f0 频带形状
  - 频带外噪声底
  - drate 的频谱形状是否像 srate

  这些是 FFT 更自然。如果用 Goertzel 做完整替代，就得跑一组 Goertzel
  bank，比如对 0.7f0 ~ 1.3f0 采 16 或 32 个频点，再额外采噪声频点。
  这样可以做，但本质上是在手写一个窄带 DTFT/Goertzel 频谱器，收益会
  变小。

  我的判断：

  - 当前 FreqCCv4 这套窗口评分逻辑：FFT/FFTW 更合适。
  - 如果改成轻量在线检测器：Goertzel 更合适。
  - 最好的折中是：保留 FFTW 版作为离线/诊断完整评分；新增 Goertzel
    窄带快速路径，用于只判断 drate 和 srtt 是否响应 f0。

  如果真要用 Goertzel，建议不要用整数 bin 近似，应该用 generalized
  Goertzel / 任意频率 DTFT 形式，直接计算 omega = 2π f0 / fs。这里
  fs = 1 / kSampleStepSec = 1000Hz，f0 通常很低，窗口长度又未必让 f0
  正好落在整数 DFT bin 上，整数 k = round(N f0 / fs) 容易引入偏差。