# BBR-R 的 ns-3 / DQC 迁移

本仓库已将 `../BBR-R-Source-Code-main.zip` 中的 Linux `bbr_r.c` 接入
ns-3 的 DQC 拥塞控制框架。场景参数使用 `BBR-R` 即可选择该算法：

```text
--algos=BBR-R
```

## 迁移边界

参考源码是 Linux 4.14 TCP 拥塞控制模块，依赖 `tcp_sock`、内核 pacing、
rate sample、jiffies 和 `minmax` 等设施，不能直接编译进 ns-3。当前实现以
本仓库 `ProtoBbrSender`（DQC BBRv1）为宿主；在 DQC 能表达的范围内，BBR-R
分支向参考实现对齐：

- 在 `PROBE_BW` 且 pacing gain 不高于 1.0 时检测 RTT 膨胀；
- RTT 不高于 `1.05 * min_rtt` 时复位竞争状态；
- 连续 10 个膨胀 RTT 样本后，以窗口最小 RTT 更新自适应参考值；
- RTT 达到参考值的 1.25 倍时，pacing 使用 `0.80 * max_bw`；
- 中等 RTT 膨胀时，pacing 使用 `0.95 * max_bw`；
- gain-cycle phase 3 不做带宽折减；
- min-RTT 窗口为 13 秒，10 ms 范围内的 RTT 样本刷新时间戳；
- 低增益 phase 需要同时满足“经过自适应 RTT 周期”和“inflight 已排空到
  BDP”才进入下一 phase。

此外，BBR-R 专用的宿主参数和事件语义也已对齐：

- STARTUP pacing/cwnd gain 固定为 Linux `bbr_high_gain = 739 / 256`，DRAIN
  pacing gain 固定为 `88 / 256`；
- PROBE_BW cwnd gain 固定为参考实现的 `2.0`，不受 DQC 通用 BBR flag 影响；
- RTT 膨胀判定使用 DQC `BandwidthSample` 中未扣除 ACK delay 的原始 RTT，而非
  `RttStats::latest_rtt()`；
- 10 个膨胀样本的最小 RTT 使用参考源码同款三候选 `minmax_rtt` 算法；
- PROBE_BW 高/低增益 phase 均按 ACK 前 inflight 判定；高增益要求严格超过目标
  inflight，低增益同时满足自适应 RTT 周期和 BDP 排空；
- BBR-R 不叠加 DQC/QUIC BBR 的 ACK-aggregation cwnd allowance，保持 Linux
  BBRv1 的 cwnd 计算口径。

`bbr_r.c` 将 RTT 自适应变量定义成文件级全局变量。直接照搬会让 ns-3 中
的四条流共享状态并相互污染，因此迁移版将这些变量放入 `BbrRSender`，每个
sender 独立维护。BBR-R 仅调整 pacing 使用的带宽，cwnd 仍使用未经折减的
BBR 带宽估计，与参考源码的 `bw1` / `bw2` 分工一致。

仍保留的 DQC 边界是：每个 ACK frame 可能批量确认多个包、没有 Linux TCP 的
TSO/GSO/FQ pacing 与 socket recovery 细节，也没有 Linux BBRv1 的 traffic
policer 长期带宽采样。因此本实现是尽可能贴近 `bbr_r.c` 的 DQC 版本，不宣称
跨传输栈的逐事件完全等价。

核心实现位于：

- `src/dqc/model/thirdparty/congestion/bbr_r_sender.{h,cc}`
- `src/dqc/model/thirdparty/congestion/proto_bbr_sender.{h,cc}` 中的变体钩子
- `src/dqc/model/thirdparty/congestion/proto_send_algorithm_interface.cc` 中的工厂入口
- `scratch/generic_p2p_switch_flows.cc` 中的 `BBR-R` 场景名称

Waf 和 CMake 的 DQC 构建清单均已加入 BBR-R。统一实验 runner、绘图脚本
也已将 `BBR-R` 作为可选算法，并能输出 goodput、RTT、OWD、queue delay、
带宽、原始 delivery-rate、pacing、inflight、BBR mode 和共享队列 trace。

## 构建和运行

```bash
cd /home/wkd/FreqBBR/NS3.27
./waf build -j4
```

快速四流测试：

```bash
python3 examples/ConcurrentFlow/run_4cc_comparison.py \
  --only-cc BBR-R \
  --experiment-dir results/bbrr_smoke \
  --n-flows 4 --sim-time 5 --start-times 0 --stop-times 5 \
  --access-rate 1Gbps --service-rate 100Mbps \
  --access-delay-ms 1 --service-delay-ms 19 \
  --switch-buffer-bdp 2 \
  --data-generator-batch 2048 \
  --stream-buffer-bytes 67108864 \
  --enable-heavy-trace --direct-binary
```

与给定参考目录完全同构的 180 秒验证：

```bash
python3 examples/ConcurrentFlow/run_4cc_comparison.py \
  --only-cc BBR-R \
  --experiment-dir results/bbrr_4flows_shared_100M_2BDP_180s \
  --n-flows 4 --sim-time 180 --start-times 0 --stop-times 180 \
  --initial-rates 0 \
  --access-rate 1Gbps --service-rate 100Mbps \
  --access-delay-ms 1 --service-delay-ms 19 \
  --switch-buffer-bdp 2 --endpoint-queue-bytes 1073741824 \
  --data-generator-batch 2048 \
  --stream-buffer-bytes 67108864 \
  --enable-convergence-gate-trace --gate-trace-mode sampled_pacing \
  --enable-heavy-trace --enable-queue-trace \
  --queue-sample-interval-us 200 --direct-binary --no-plot
```

生成同口径汇总 CSV 和图片：

```bash
python3 examples/ConcurrentFlow/plot_4cc_comparison.py \
  --experiment-dir results/bbrr_4flows_shared_100M_2BDP_180s \
  --service-rate 100Mbps --n-flows 4 \
  --ccs BBR-R --trace-names BBR-R \
  --sample-step-s 0.1 --warmup-s 5 \
  --flow-start-times 0 --flow-stop-times 180 --sim-time 180
```

上述重 trace 配置单次会产生约 0.5 GB 输出。只验证算法可运行时，建议使用
快速命令或关闭 heavy / 逐事件 queue trace。

## 本次验证结果

本次实际输出在：

```text
results/bbrr_4flows_shared_100M_2BDP_180s_20260719/
```

验证参数与
`/mnt/nasDisk_ds3617/wkd/1FreqBBR/five_cc_4flows_shared_100M_2BDP_180s_20260719`
一致。按 5 秒 warm-up 后的统一绘图脚本口径：

| CC | 聚合吞吐 (Mbps) | 利用率 | 平均队列时延 (ms) | P95 队列时延 (ms) | Jain 公平性 |
|---|---:|---:|---:|---:|---:|
| BBR-R | 97.7195 | 0.9772 | 5.0236 | 14.4634 | 0.9847 |
| BBRv2 | 97.9365 | 0.9794 | 72.5569 | 83.7504 | 0.9532 |
| BBRv2plus | 97.8383 | 0.9784 | 59.2428 | 83.1744 | 0.8768 |
| oBBR | 95.7808 | 0.9578 | 50.6213 | 60.4282 | 0.9751 |
| FBBR | 95.4809 | 0.9548 | 24.7470 | 57.8123 | 0.8963 |

BBR-R 的仿真返回码为 0，四条流均完成 180 秒；总接收速率为 97.612 Mbps，
丢包率为 0。pacing/max-bw trace 中四条流都出现了大量约 0.95 的比例，且
出现了 0.80 折减样本，说明 RTT 自适应 pacing 分支在实际多流场景中被执行。

这是一份 DQC-hosted BBR-R 对齐实现；其核心 RTT 降速规则、固定 gain、phase
判定和 cwnd 口径均已向 Linux 参考源码收敛，但不宣称复现 Linux 内核的定时、
TSO、FQ pacing、traffic policer 或 socket 恢复细节。
