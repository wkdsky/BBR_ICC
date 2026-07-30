# Cellular Taxi 单流 CC 最终实验报告

生成日期：2026-07-30

## 最终结论

在同一条 Taxi 动态上行 trace、单流、170 秒实验中，优化后的 FBBR 达到
16.360240 Mbps 应用好吞吐和 99.876% 链路利用率。吞吐在六种算法中排名
第二，仅比 Cubic 低 0.052800 Mbps（0.322%）；同时 FBBR 的上行排队延迟
平均值、P95、P99 和平均 RTT 都是高吞吐算法中最低，并且保持零丢包。

FBBR 最终关键指标：

- 应用好吞吐：16.360240 Mbps
- 链路利用率：99.876%
- 上行排队延迟（平均 / P95 / P99）：14.028 / 55.000 / 164.000 ms
- 平均 RTT：34.969 ms
- 上行 / 下行丢包率：0.000% / 0.000%
- Jain 公平性：1.000000（单流）

## 实验设置

| 项目 | 配置 |
| :--- | :--- |
| 场景 | `taxi_1`，单流 |
| 实验时长 | 170 秒 |
| 上行 | `scratch/celluar_trace/trace_taxi`，Taxi 动态带宽 |
| 下行 | `scratch/48mbps_data.trace`，固定 48.128 Mbps ACK 链路 |
| 传播时延 | `mm-delay 10`，单向 10 ms，基础 RTT 20 ms |
| 队列 | Mahimahi droptail，100 packets |
| 算法 | `bbr_r`、`bbrv2`、`bbrv2plus`、`cubic`、`obbr`、`fbbr` |
| 指标来源 | sender 完成摘要和 Mahimahi uplink/downlink 日志 |

Taxi trace 包含 243157 个 packet-delivery slot，时间范围 0--170818 ms，
有效时长 170.818 秒，按 1504-byte Mahimahi slot 计算的平均容量为
17.127381 Mbps。逐秒容量的最小值、P50、P95、最大值分别为
0.337、20.105、25.700、28.492 Mbps。文件 SHA-256 为
`07e5e74d1b480ec193d981b278f3e13751ceaea5d84f858ace1b19d6a4e13108`。

除 FBBR 外的五种算法取自完整实验
[`cellular_results_20260730_112819`](/home/llj/ICC-Introspective-Congestion-Control-for-Consistent-High-Performance/User-space/cellular_results_20260730_112819/)。最终
FBBR 取自优化后完整实验
[`cellular_results_20260730_122100`](/home/llj/ICC-Introspective-Congestion-Control-for-Consistent-High-Performance/User-space/cellular_results_20260730_122100/)。
两组均使用相同 trace、时长、流数、时延和队列配置，所有 `mm-link` 退出码均为 0。

## 六种 CC 最终指标

以下排队延迟全部来自 Mahimahi 上行 `-` 事件。用户要求的平均、P95、P99
分别独立列出，最大值用于补充观察极端容量下降时的尾部情况。

| 吞吐排名 | 算法 | 应用好吞吐 Mbps | 线速 Mbps | 利用率 | 排队平均 ms | 排队 P95 ms | 排队 P99 ms | 排队最大 ms | 上行丢包率 | 平均 RTT ms | 数据 OWD 估计 ms | Jain |
| ---: | :--- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 1 | cubic | 16.413040 | 17.161343 | 100.198% | 48.697 | 116.000 | 316.000 | 2129.000 | 0.193% (480/249002) | 69.693 | 58.697 | 1.000000 |
| 2 | **fbbr** | **16.360240** | **17.106077** | **99.876%** | **14.028** | **55.000** | **164.000** | 1715.000 | **0.000% (0/247682)** | **34.969** | **24.028** | 1.000000 |
| 3 | bbrv2 | 16.287680 | 17.030232 | 99.433% | 42.564 | 109.000 | 301.000 | 2379.000 | 0.283% (699/247270) | 63.498 | 52.564 | 1.000000 |
| 4 | bbrv2plus | 16.283200 | 17.025603 | 99.406% | 44.277 | 121.000 | 317.000 | 2185.000 | 0.952% (2369/248930) | 65.214 | 54.277 | 1.000000 |
| 5 | obbr | 16.276880 | 17.018909 | 99.367% | 42.010 | 107.000 | 263.000 | 911.000 | 0.603% (1496/247892) | 62.954 | 52.010 | 1.000000 |
| 6 | bbr_r | 16.243840 | 16.984362 | 99.165% | 28.135 | 103.000 | 247.000 | 891.000 | 0.302% (744/246632) | 49.082 | 38.135 | 1.000000 |

下行六种算法均为 0 丢包。利用率轻微超过 100% 是 Mahimahi slot 容量口径、
应用包大小和实验边界归一化造成的测量差异，不代表产生了额外链路容量。

## 指标对比

相对 BBRv2，最终 FBBR：

- 吞吐提高 0.445%。
- 平均排队延迟降低 67.043%。
- 排队 P95 降低 49.541%。
- 排队 P99 降低 45.515%。
- 平均 RTT 降低 44.929%。
- 上行丢包从 0.283% 降为 0。

相对吞吐第一的 Cubic，FBBR 吞吐低 0.322%，但平均排队延迟降低
71.194%，P95 降低 52.586%，P99 降低 48.101%，平均 RTT 降低
49.825%，并消除了 0.193% 的上行丢包。

### FBBR 优化前后

| FBBR 版本 | 应用好吞吐 Mbps | 利用率 | 排队平均 ms | 排队 P95 ms | 排队 P99 ms | 平均 RTT ms | 上行丢包率 | 控制器样本数 |
| :--- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 优化前 | 5.928968 | 36.196% | 3.676 | 14.000 | 52.000 | 25.443 | 0.000% | 89737 |
| 优化后 | 16.360240 | 99.876% | 14.028 | 55.000 | 164.000 | 34.969 | 0.000% | 247617 |

吞吐提高 175.937%，利用率提高 63.680 个百分点。优化前较低的排队和 RTT
建立在链路只利用 36.196% 的基础上；恢复接近满利用率后，FBBR 仍然保持了
六种高吞吐结果中最低的平均、P95、P99 排队延迟和最低 RTT。

## 原始实验数据索引

| 算法 | 结果目录 | 控制器样本 | 完成时间 s | 状态 |
| :--- | :--- | ---: | ---: | :--- |
| bbr_r | [`cellular_results_20260730_112819/bbr_r`](/home/llj/ICC-Introspective-Congestion-Control-for-Consistent-High-Performance/User-space/cellular_results_20260730_112819/bbr_r/) | 245858 | 170.002 | ok，mm-link=0 |
| bbrv2 | [`cellular_results_20260730_112819/bbrv2`](/home/llj/ICC-Introspective-Congestion-Control-for-Consistent-High-Performance/User-space/cellular_results_20260730_112819/bbrv2/) | 246522 | 170.002 | ok，mm-link=0 |
| bbrv2plus | [`cellular_results_20260730_112819/bbrv2plus`](/home/llj/ICC-Introspective-Congestion-Control-for-Consistent-High-Performance/User-space/cellular_results_20260730_112819/bbrv2plus/) | 246455 | 170.002 | ok，mm-link=0 |
| cubic | [`cellular_results_20260730_112819/cubic`](/home/llj/ICC-Introspective-Congestion-Control-for-Consistent-High-Performance/User-space/cellular_results_20260730_112819/cubic/) | 248417 | 170.000 | ok，mm-link=0 |
| obbr | [`cellular_results_20260730_112819/obbr`](/home/llj/ICC-Introspective-Congestion-Control-for-Consistent-High-Performance/User-space/cellular_results_20260730_112819/obbr/) | 246361 | 170.004 | ok，mm-link=0 |
| fbbr | [`cellular_results_20260730_122100/fbbr`](/home/llj/ICC-Introspective-Congestion-Control-for-Consistent-High-Performance/User-space/cellular_results_20260730_122100/fbbr/) | 247617 | 170.000 | ok，mm-link=0 |

每个算法目录中的数据文件含义一致：

- `summary.txt`：完成状态、应用好吞吐、样本数。
- `flow1_sender.out`：每秒控制器状态以及最终吞吐、RTT、完成时间。
- `flow1_dr.csv`：逐 ACK delivery-rate 样本；FBBR 文件还包含控制诊断记录。
- `uplink.log`：Mahimahi 上行到达、离开、丢包和排队延迟原始事件。
- `downlink.log`：Mahimahi 下行原始事件。
- `run_status.txt`：`mm-link` 退出状态。
- `inner_run.sh`：实验实际执行的 sender 命令。

汇总与图表：

- 原五种对照算法及旧 FBBR 指标：
  [`cellular_results_20260730_112819/metrics_table.md`](/home/llj/ICC-Introspective-Congestion-Control-for-Consistent-High-Performance/User-space/cellular_results_20260730_112819/metrics_table.md)
- 最终 FBBR 指标：
  [`cellular_results_20260730_122100/metrics_table.md`](/home/llj/ICC-Introspective-Congestion-Control-for-Consistent-High-Performance/User-space/cellular_results_20260730_122100/metrics_table.md)
- 对照算法 delivery-rate 图：
  [`cellular_results_20260730_112819/cellular_delivery_rate_scatter.png`](/home/llj/ICC-Introspective-Congestion-Control-for-Consistent-High-Performance/User-space/cellular_results_20260730_112819/cellular_delivery_rate_scatter.png)
- 最终 FBBR delivery-rate 图：
  [`cellular_results_20260730_122100/cellular_delivery_rate_scatter.png`](/home/llj/ICC-Introspective-Congestion-Control-for-Consistent-High-Performance/User-space/cellular_results_20260730_122100/cellular_delivery_rate_scatter.png)
- 跨实验机器可读汇总：[`cellular_metrics_summary_v2.tsv`](/home/llj/ICC-Introspective-Congestion-Control-for-Consistent-High-Performance/User-space/cellular_metrics_summary_v2.tsv)

## 本地 Cellular Trace 场景清单

### 当前项目可直接使用的 Mahimahi trace

| 文件 | 类型 | 状态 | 说明 |
| :--- | :--- | :--- | :--- |
| `scratch/celluar_trace/trace_taxi` | 动态移动蜂窝上行 | 可直接运行 | 170.818 秒，本报告使用的唯一真实动态 cellular trace |
| `scratch/48mbps_data.trace` | 固定下行 | 可直接运行 | 1499.999 秒，48.128 Mbps；当前只作为 ACK 下行，不是独立移动场景 |
| `scratch/celluar_trace/100mbps.trace` | 标称固定 100 Mbps | 当前不可作为有效 Mahimahi 场景 | 60 行全部为 `100000000`；这是速率值列表，不是正确的 packet-delivery timestamp 序列，需要重新生成 |

当前项目及其 Git 历史中没有发现 Taxi 之外的其他动态 Mahimahi cellular
trace；`run_cellular_trace.sh` 也固定使用 `trace_taxi`。

### 本机其他可转换蜂窝数据

另一个本地仓库 `/home/wkd/FreqBBR` 中存在四组原始蜂窝测量：

| 数据 | 路径 | 类型 | 原始跨度 | 行数 | 网络制式 | 当前状态 |
| :--- | :--- | :--- | :--- | ---: | :--- | :--- |
| car1 | `oBBR/nginx-quic/scripts/nets/traces/car1.csv` | 移动车载动态 | 28 分 38 秒 | 1646 | LTE / 5G | 有对应 `car1.sh` tc 回放脚本，未转换为 Mahimahi trace |
| car2 | `oBBR/nginx-quic/scripts/nets/traces/car2.csv` | 移动车载动态 | 19 分 22 秒 | 1155 | LTE / 5G / HSPA+ / UMTS | 有对应 `car2.sh` tc 回放脚本，未转换为 Mahimahi trace |
| static1 | `oBBR/nginx-quic/scripts/nets/traces/static1.csv` | 静态 5G | 58 分 17 秒 | 3148 | 5G | 固定位置测量，不是移动场景 |
| static2 | `oBBR/nginx-quic/scripts/nets/traces/static2.csv` | 静态 5G | 61 分 19 秒 | 3322 | 5G | 固定位置测量，不是移动场景 |

这些 CSV 包含时间、位置、速度、网络制式、无线信号、上下行 bitrate 和 ping
统计；对应 shell 脚本按秒修改 Linux `tc` 的带宽和时延。因此，本机确实还有
`car1`、`car2` 两个可扩展为新动态 cellular 场景的数据集，但它们不能直接传给
当前 `mm-link`。接入当前统一实验框架需要把每秒速率转换为 Mahimahi packet
delivery timestamps；若还要保留原始动态 RTT，则 runner 还需要支持随时间改变
传播时延，而不是固定 `mm-delay 10`。
