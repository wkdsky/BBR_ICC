# FreqCCv4 实现指南

当前实现以 Native BBRv2 为模型主体，只在 CRUISE 增加测量调制，并只在 REFILL、UP、DOWN 替换 pacing bandwidth baseline。

实现入口：

- `FreqCCv4Sender::BuildCruiseWindowResult()`：构造窗口并独立计算 Delivery Rate/SRTT 频域完整性。
- `FreqCCv4Sender::RunTrustedBwSelection()`：只在双信号门控通过的 NORMAL/MERGED 窗口中排序。
- `FreqCCv4Sender::PublishTrustedBwSelection()`：在 CRUISE 结束时发布本轮结果。
- `FreqCCv4Sender::ClearTrustedBwApplication()`：新 CRUISE 开始时关闭旧应用窗口。
- `FreqCCv4Sender::ClearTrustedBw()`：明确失效时清零带宽、有效位、来源和应用状态。
- `FreqCCv4Sender::PacingRate()`：REFILL/UP/DOWN 直接选择 TrustedBw 或 NativeBw 基线。

双信号门控：

```cpp
joint_spectral_integrity_score =
    std::min(drate_spectral_integrity_score,
             srtt_spectral_integrity_score);

dual_signal_spectral_gate_pass =
    drate_spectral_gate_pass &&
    srtt_spectral_gate_pass;
```

任一信号缺样本、覆盖不足、时间戳异常、数值异常、频率不可分辨、频谱无效或未通过自己的门限时，窗口不可用于 TrustedBw。扩大窗口救援也执行同一门控；不存在只依赖单个信号的候选路径。

Pacing：

```text
CRUISE:
    final = Native BBR pacing + triangle modulation

REFILL / UP / DOWN:
    fresh valid TrustedBw -> final = phase_gain × TrustedBw
    otherwise             -> final = Native BBR pacing

STARTUP / DRAIN / PROBE_RTT:
    final = Native BBR pacing
```

禁止把 TrustedBw 写入 Native MaxBw filter、BandwidthEstimate、BDP、cwnd、inflight_hi 或 inflight_lo。
