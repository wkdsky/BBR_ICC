# ns-3.47 CUBIC 到本地 DQC/ns-3.27 的迁移

## 迁移基线

- 最新稳定版基线：ns-3.47，发布于 2026-02-16。
- 官方源码：`ns-3.47/src/internet/model/tcp-cubic.cc` 和
  `tcp-cubic.h`。
- 下载源码 SHA-256：
  - `tcp-cubic.cc`：`442be0e2d4d1faccab603d87252b226567ffa2d6abf6a07cb79baf1e09625606`
  - `tcp-cubic.h`：`f3417e647f1ac2bc47137b79d0f0122ceb3be5bd7488c6397229b6df5eca6417`
- 标准交叉检查：RFC 9438（2023-08，取代 RFC 8312）。

官方 ns-3.47 的 `TcpCubic` 仍明确说明其实现沿用 Linux 3.14 风格的
HyStart，而不是声称逐条实现 RFC 9438/HyStart++。本迁移因此以
**ns-3.47 实际源码行为**为准，并用 RFC 9438 校验 CUBIC 的核心参数、
三次增长曲线、Reno-friendly 区域、`beta=0.7` 退避和 fast convergence。

## 本地实现

- 算法实现：
  `src/dqc/model/thirdparty/congestion/ns3_cubic_sender.{cc,h}`
- 类型：`dqc::kNs3Cubic`
- 实验名称：`CUBIC`
- 旧实现：`dqc::kCubicBytes` 保持不变，避免破坏已有实验可复现性。

保留的 ns-3.47 行为包括：

- `C=0.4`、`beta=0.7`；
- fast convergence 和 TCP friendliness 默认开启；
- CUBIC epoch、`K`、origin point 和逐 ACK `cnt` 增窗；
- 无丢包时 `cnt<=20`，且增长上限为每两个 ACK 增加一个 MSS；
- HyStart 的 ACK-train 和 delay 两种检测；
- 16 包 HyStart 门槛、8 个 RTT 样本、2ms ACK 间隔、4ms/1000ms delay clamp；
- 丢包时更新 `W_max`、结束 epoch，并把窗口退避到 `0.7*cwnd`。

## DQC 适配边界

本仓库的统一实验使用 DQC/QUIC 数据面，而 ns-3 官方 `TcpCubic` 接收
`TcpSocketState`。适配层只替换传输接口，不修改上述 CUBIC 控制律：

- DQC ACK 字节数按 1460B MSS 换算为 `segmentsAcked`；
- DQC `RttStats` 提供 `latest_rtt` 和 `min_rtt`；
- DQC 包号界定一次恢复期，PRR 控制恢复期间的发送；
- DQC 必须提供 pacing，因此沿用仓库窗口型 CC 的宿主规则：慢启动
  `2*cwnd/srtt`，拥塞避免 `1.25*cwnd/srtt`；这不是 CUBIC 控制律本身；
- `SetNumEmulatedConnections` 对本算法无效，因为 ns-3 `TcpCubic` 表示单个
  TCP 连接。正式验证使用 `emulatedConnections=1`。

## 构建与运行

```bash
cd /home/wkd/FreqBBR/NS3.27
./waf build

python3 examples/ConcurrentFlow/run_4cc_comparison.py \
  --only-cc CUBIC \
  --experiment-dir /home/wkd/FreqBBR/NS3.27/results/cubic_4flows_shared_100M_2BDP_180s_20260719 \
  --sim-time 180 --start-times 0 --stop-times 180 \
  --access-rate 1Gbps --service-rate 100Mbps \
  --access-delay-ms 1 --service-delay-ms 19 \
  --switch-buffer-bdp 2 --data-generator-batch 2048 \
  --stream-buffer-bytes 67108864 --enable-heavy-trace \
  --enable-convergence-gate-trace --gate-trace-mode sampled_pacing \
  --direct-binary
```

该配置与
`/mnt/nasDisk_ds3617/wkd/1FreqBBR/five_cc_4flows_shared_100M_2BDP_180s_20260719`
的 4 流、100Mbps、约 40ms RTT、2BDP、180s 场景一致。

## 验证记录

- 全量 `./waf build`：通过。
- 10s 冒烟实验：通过；4 个 goodput、RTT、qdelay、sendrate、inflight trace
  均非空，绘图/汇总成功。
- 180s 正式实验：通过，仿真和绘图返回码均为 0。
  - 聚合吞吐：97.912445 Mbps；
  - 瓶颈利用率：0.979124；
  - 平均排队时延：70.481782 ms；
  - P95 排队时延：83.174400 ms；
  - 4 流 Jain 公平性：0.982502。
- 正式结果：
  `results/cubic_4flows_shared_100M_2BDP_180s_20260719/`，其中
  `return_codes.json`、`compare/summary_metrics.csv` 和对比图可直接复核。

上游 `TcpCubic` 文件使用 GPL-2.0-only，本适配文件保留了来源和许可证说明。
