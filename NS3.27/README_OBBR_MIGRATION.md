# oBBR 的 ns-3 / DQC 迁移

`oBBR/nginx-quic/src/event/quic/ngx_bbr.c` 是本仓库 oBBR 的参考实现。
DQC 版本仍使用自己的 QUIC sender、恢复和 pacing 基础设施，但 oBBR 自身的
RTT、cwnd 与 gain-cycle 决策已向参考代码对齐。

## 已对齐行为

- nginx-quic 在 ACK frame 中用最大新确认包的 `ack_time - send_time` 更新
  `qc->latest_rtt`，不扣除 ACK delay；DQC `RttStats` 现在保存同一语义的
  `latest_raw_rtt`，oBBR 仅在 `rtt_updated` 时更新自己的这份最新原始 RTT。
- 与参考实现的 `ngx_generate_sample()` 一样，oBBR 只有在当前 ACK 产生有效
  delivery sample 时才消费该最新原始 RTT；它可以来自当前 ACK，也可以来自
  上一个更新 RTT 的 ACK frame。
- oBBR 的 `min_rtt`、RTT 膨胀计数和丢包时的 cwnd gain 都使用这份原始 RTT，
  不再混用 DQC `RttStats::latest_rtt()` 的 ACK-delay 修正值，也不再从一次
  批量 ACK 的所有包中取最小 RTT。
- 参考代码在 `sampler->rtt <= min_rtt` 时刷新 10 秒 min-RTT 窗口；DQC 版本
  采用相同的相等比较和刷新时机。
- PROBE_RTT 持续时间按参考代码使用 `min(2 * srtt, 200ms)`；这里只有 srtt
  继续使用 DQC 的 ACK-delay 修正平滑 RTT。
- PROBE_BW 低增益 phase 按 ACK 前 inflight 判断是否已经排空到 BDP。
- 参考代码的 `ngx_bbr_extra_ack_gain` 固定为 0，因此 DQC oBBR 不再把
  ACK-aggregation 额度加入 cwnd。
- 评分阶段的失败回退按 `ngx_win_filter_max()` 语义把保存带宽重新插入 max-filter，
  不会重置并丢弃窗口内更高的带宽样本。

## 保留的 DQC 边界

- nginx-quic 固定以 32 MSS 初始化 oBBR；DQC 保留调用方传入的初始 cwnd，方便
  各算法在同一 ns-3 场景下使用统一连接配置。
- 两个实现的 delivery-rate sampler、ACK 批处理、丢包恢复和 packet pacing
  基础设施不同，因此无法承诺逐 ACK 的完全一致。
- DQC 的 `RttStats` 仍供 QUIC 的 loss detection、PTO 和平滑 RTT 使用；新增的
  原始 RTT 字段不改变其他算法的既有 RTT 行为，本次只改变 oBBR 控制路径的消费方式。

核心实现位于：

- `src/dqc/model/thirdparty/congestion/obbr_sender.{h,cc}`
- `src/dqc/model/thirdparty/{include,src}/rtt_stats.{h,cc}`

构建和快速场景验证：

```bash
cd /home/wkd/FreqBBR/NS3.27
./waf build -j4
./waf --run "scratch/generic_p2p_switch_flows --nFlows=4 --simTime=5 \
  --algos=oBBR --startTimes=0 --flowStopTimes=5 --initialRates=0 \
  --accessRate=1Gbps --serviceRate=100Mbps --accessDelayMs=1 \
  --serviceDelayMs=19 --switchBufferBdp=2"
```
