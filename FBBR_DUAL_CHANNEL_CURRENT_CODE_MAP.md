# F-BBR Dual-Channel Current Code Map

生成时间：2026-07-13（改造前基线）

## 输入状态

规范要求预读的 `FBBR_EVENT_TRIGGERED_DYNAMIC_WINDOW_FINAL_REPORT.md`、
`FBBR_EVENT_WINDOW_CURRENT_CODE_MAP.md` 和 `CURRENT_FBBR_CODE_MAP.md` 在当前工作区不存在。
本映射以现有 F-BBR 源码、配置、runner、analyzer 和已有 `NS3.27/docs/fbbr_*`
实验产物为事实基线。F-BBR 源文件当前均为工作区新增文件，本轮修改必须保留其已有实现。

## 算法隔离与构造

- CC 类型：`NS3.27/src/dqc/model/thirdparty/include/proto_types.h:130` 的 `kFBBR`。
- factory：`proto_send_algorithm_interface.cc:394` 独立构造 `FBBRSender`。
- sender 分发：`dqc_sender.cc:1005` 的 `ConfigureFBBR()` 仅在 `kFBBR` 下调用。
- 通用 runner：`scratch/generic_p2p_switch_flows.cc:983` 将 CLI `F-BBR` 映射到
  `kFBBR`，并在 `:1219` 调用独立 F-BBR 配置。
- build：`src/dqc/wscript:106,181` 注册 `fbbr_frequency_search.{cc,h}`。
- FreqCCv4 路径不参与 F-BBR 构造、配置或控制。

## CRUISE 进入和离开调用链

```text
Bbr2ProbeBwMode phase transition
  -> FBBRSender::OnProbeBwPhaseEntered()          fbbr_sender.cc:2889
     -> EnterCruise(now)                          fbbr_sender.cc:2900
        -> ClearTrustedBwApplication()
        -> ResetCruiseWindowState()
        -> FBBRFrequencySearchInitializeCruise()  fbbr_sender.cc:814
           -> freeze/restore RTprop anchor
           -> initialize carrier/pulser/search baseline
           -> initialize event-window state

non-CRUISE phase transition
  -> FBBRSender::OnProbeBwPhaseEntered()
     -> LeaveCruise(now)                          fbbr_sender.cc:2926
        -> FinalizeCruise()
           -> FBBRFrequencySearchFinalizeCruise() fbbr_sender.cc:2473
        -> clear per-CRUISE acquisition state
```

`ShouldDelayProbeBwCruiseExit()`（`fbbr_sender.cc:2961`）按搜索状态和扩展预算延长
CRUISE；persistent state 由 `FBBRFrequencySearchInitializeCruise()` 在下一次 CRUISE
恢复。

## 实际信号采集路径

- actual packet emission：`OnPacketSent()`（`fbbr_sender.cc:3555`）在真正交给
  `Bbr2Sender::OnPacketSent()` 前调用 `FBBRFrequencySearchAccumulateSend()`
  （`:1369`），按固定 phase bin 累积 `sent_bytes`、commanded/native pacing、
  inflight、app/cwnd/recovery 状态。
- delivery：`OnCongestionEventStarted()`（`:2950`）调用
  `FBBRFrequencySearchAccumulateAck()`（`:1407`），累积 `acked_bytes`；phase-bin
  完成时由 `FBBRFrequencySearchFinalizePhaseBin()`（`:1479`）用实际 ACK bytes / bin
  duration 重建 delivery rate。
- latest RTT：`FBBRFrequencySearchAccumulateAck()` 从
  `rtt_stats_->latest_rtt()` 采样，bin 内取中位数；queue delay 为
  `latest_rtt - fbbr_probe_signature_.rtprop_s`，不使用 SRTT。
- loss：ACK 事件的 `bytes_lost` 进入 phase bin，并计算每 bin loss ratio。
- ECN：`GetBytesEcnInRounds()` 的增量进入 phase bin，并计算每 bin ECN ratio。
- actual normalized input：analysis 中使用
  `(actual_send_bps - native_pacing_bps) / native_pacing_bps`，commanded amplitude
  仅用于 realized/diagnostic ratio。

## Carrier、Pulser/Watcher 与事件窗口

- carrier：`FBBRFrequencySearchInitializeCruise()` 选择 RTT slot、周期、code、phase、
  amplitude；`FBBRFrequencySearchCodedSineValue()`（`:1196`）生成载波。
- pulser/watcher：初始化时按 flow identity 和全局 rotation epoch 选单 pulser；
  `FBBRSearchUpdateCarrierSense()`（`:1227`）供 watcher 检测公共 carrier；只有
  pulser window 能进入 control。
- event state：`EventWindowState` 位于 `fbbr_frequency_search.h`；sender 的
  `FBBRFrequencySearchFinalizeReadyBlocks()`（`:1563`）驱动
  `IDLE_LISTEN -> TRIGGER_ARMED -> CAPTURE -> CONTINUOUS_TRACK/PAUSED ->
  POST_BASELINE_SETTLING`。
- 当前 trigger：`FBBRFrequencySearch::AnalyzeTriggerCycle()`
  （`fbbr_frequency_search.cc:1063`）仅分析 delivery-rate carrier；actual input
  measurable 后仍要求 delivery prominence、match、period 和绝对 response bytes。
- trigger cycle exclusion：capture 从 `(trigger_cycle + 1) * bins_per_cycle` 开始；
  event result 的 `trigger_cycle_excluded_from_score` 在 `fbbr_sender.cc:1612`
  显式记录。
- dynamic window：4 cycles 起做方向判断，6 cycles 起做 score，最大 12 cycles；
  continuous track 使用 0.5-cycle 滑动，控制独立间隔目前硬编码为 1 cycle，
  trusted 独立间隔目前为 `window_length / 2`。

## RTprop、Queue Estimate 与 Score

- 初始 anchor：native `MinRtt()` / `MinOrInitialRtt()`，置信度 0.5。
- anchor 刷新：`FBBRSearchUpdateRtpropAnchor()`（`fbbr_sender.cc:2106`）使用负
  half-cycle RTT P05 更新/确认。
- window queue：`AnalyzeBlock()` 中从 raw/latest RTT qdelay 计算 cycle floor、q95、
  amplitude、drain ratio 和 robust cycle trend。
- reserve band：已有 `q_reserve_low_bdp=0.02`、`high=0.05`、`peak=0.10`。
- 当前 score：measurement confidence 仍显式依赖 delivery prominence/match；
  saturation 使用 delivery clipping OR queue storage；target score 当前为
  `C_meas * S_sat^0.4 * S_band^0.4 * S_stable^0.2`。

## Search Baseline、DRAIN、TRACK、LOCK 与 Trusted Publication

- baseline：`FBBRSearchSearchBaselineBps()`（`fbbr_sender.cc:1312`），更新通过
  `FBBRSearchStartBaselineTransition()`（`:1337`）做 raised-cosine log ramp。
- control：`FBBRSearchController::Decide()`
  （`fbbr_frequency_search.cc:2222`）。UNDERLOAD/OVERLOAD 建双边 bracket，只有
  双边有效时取几何中点。
- 当前 queue correction：仅在 completed event window 后由 controller 做一次
  `mild_drain_target()`，普通步长 2%-5%；没有独立 RTT 快环。
- TRACK/LOCK：near-optimal、score、queue band、RTprop confidence 和独立窗口共同
  门控。
- trusted：仅接受独立 LOCKable windows 的 baseline/candidate/bracket consensus；
  需要至少两个候选且 CV/ratio 通过。发布在
  `FBBRSearchApplyWindowDecision()`（`fbbr_sender.cc:2157`）处理。
- pacing：`PacingRate()`（`fbbr_sender.cc:3700`）当前 CRUISE 输出为
  `search_baseline * (1 + amplitude * carrier)`；尚无 `g_q`。

## 配置映射

- 结构：`FBBRFrequencySearchConfig`（`fbbr_frequency_search.h`）。
- parser：`SetFBBRFrequencySearchConfigValue()`
  （`fbbr_frequency_search.cc:795`），正式输入前缀为 `f_bbr.*`。
- 默认文件：`NS3.27/examples/CCconfig/fbbr_default.conf`。
- 已有配置覆盖：persistent search、delivery-only event trigger、dynamic window、
  reserve band、probe adaptation、pulser lease、search controller、trusted consensus。
- 本轮新增落点：queue trigger、queue servo、direction weights、baseline commit、
  trusted servo-unity gate；不得写入 FreqCCv4 配置。

## Trace 映射

- callback transport：`FBBRSender::CruiseLoadTraceCallback` 将结构化 CSV row 交给
  `DqcSender`，再由 `DqcTrace` 按 row label 分流。
- `flowN_fbbr_trigger_cycles.csv`：`dqc_trace.cc:815`；当前 delivery-only 字段。
- `flowN_fbbr_bins.csv`：`dqc_trace.cc:840`；实际 send/delivery/latest RTT/qdelay。
- `flowN_fbbr_event_windows.csv`：`dqc_trace.cc:863`；score/control/trusted。
- `flowN_fbbr_cruises.csv`：`dqc_trace.cc:921`；persistent/search 汇总。
- `flowN_fbbr_diagnostic_windows.csv`：`dqc_trace.cc:954`；重叠 trace-only 窗口。
- `flowN_fbbr_gate_trace.csv`：`dqc_trace.cc:773`；pacing/gate audit。
- 本轮新增：`flowN_fbbr_queue_servo.csv`，并扩展 trigger/window/cruise 字段。

## 改造落点

| 规范项 | 代码落点 |
|---|---|
| delivery + queue-derivative dual trigger | `AnalyzeTriggerCycle()` 与 `FBBRTriggerCycleResult` |
| trigger source/confidence/alignment | `fbbr_frequency_search.{h,cc}` + trigger trace |
| channel allocation and revised score | `AnalyzeBlock()` + block/event trace |
| RTT-level queue reserve servo | `fbbr_sender.{h,cc}`，ACK/RTT update 与 `PacingRate()` |
| servo transition contamination | phase-bin accumulator/sample + block hard-invalid |
| baseline commit | sender queue-servo update，独立 reason，<=2% |
| persistent counters | sender per-CRUISE counters + cruise trace |
| deterministic tests | `FBBRFrequencySearch::RunSelfTests()` |
| real/random validation | `run_fbbr_dual_channel_queue_servo_validation.sh` + analyzer |
| isolation regression | source grep、FreqCCv4 deterministic run、mixed trace audit |

