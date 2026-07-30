# Car1 / Static1 真实蜂窝 Trace CC 对比实验

> 状态：32 位吞吐计数修复并通过回归后，car1/static1 于 2026-07-30 14:37:15 CST 从 trace 起点重新并行运行。旧实验于 14:14:16 停止并保留作诊断，不纳入最终排名。

## 实验与数据位置

- 项目：`/home/llj/ICC-Introspective-Congestion-Control-for-Consistent-High-Performance/User-space`
- car1 修复后正式结果：`cellular_results_car1_20260730_143715`
- static1 修复后正式结果：`cellular_results_static1_20260730_143715`
- 修复前诊断结果：`cellular_results_car1_20260730_125255` / `cellular_results_static1_20260730_125255`
- car1 原始 CSV / mm-link trace：`scratch/celluar_trace/car1.csv` / `trace_car1`
- static1 原始 CSV / mm-link trace：`scratch/celluar_trace/static1.csv` / `trace_static1`

## Trace 与运行配置

| 场景 | CSV 样本 | 完整周期 | mm-link 平均容量 | 原始 CSV SHA-256 |
| :--- | ---: | ---: | ---: | :--- |
| car1 | 1,646 | 1,646 秒（27:26） | 31.737412 Mbps | `4a240d0a5ede58f72d4795024f008d62cf0543bb4c731eb5eeb705b3cc04eb67` |
| static1 | 3,148 | 3,148 秒（52:28） | 43.423071 Mbps | `d670a3463d683959094f47f6caacb70d65bd4081dc24d5b3a93b35f8a04495b1` |

转换口径：`DL_bitrate` 为 Kbit/s，每行持续 1 秒；低于 1 Kbit/s 的值按原 tc 脚本替换为 1 Kbit/s；从原脚本下标 8 开始回放。六种 CC 共用 100 包 droptail 队列、固定 10 ms 单向传播时延（20 ms 基础 RTT）和固定 48 Mbps ACK 方向 trace。car1/static1 使用独立端口 19000/20000 并行运行。

## 已完成结果

队列时延来自 Mahimahi 上行 `-` 事件，平均、P95、P99 均使用全部成功离队包精确统计。

| 场景 | CC | 状态 | 应用好吞吐 | 线速 | 利用率 | 队列平均 | 队列 P95 | 队列 P99 | RTT 平均 | 上行丢包 |
| :--- | :--- | :--- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| car1 | bbrv2 | ok，已校正 32 位溢出 | 26.710290 Mbps | 27.927922 Mbps | 87.997% | 20.784 ms | 75 ms | 212 ms | 41.758 ms | 1.127686% |
| car1 | bbrv2plus | ok，已校正 32 位溢出 | 26.689511 Mbps | 27.906196 Mbps | 87.928% | 21.832 ms | 75 ms | 227 ms | 42.804 ms | 1.983523% |
| static1 | bbrv2 | ok，已校正 32 位溢出 | 36.365555 Mbps | 38.023385 Mbps | 87.565% | 16.088 ms | 44 ms | 104 ms | 37.000 ms | 1.042303% |

应用好吞吐按接收端返回的 64 字节数据 ACK 数乘以 1,404 字节应用负载重算。线速利用率分母是包含 UDP/IP 开销的 mm-link 容量；应用负载容量分别为 car1 30.353765 Mbps、static1 41.529967 Mbps，因此三项应用负载利用率同样为 87.997%、87.928%、87.565%。

## 根因诊断

### 主因：CTCP 吞吐计数器发生 32 位有符号整数溢出

`User-space/ctcp.hh` 的 `transmitted_bytes` 和 `tot_bytes_transmitted` 都是 32 位 `int`，每个有效 ACK 增加 `data_size=1436-sizeof(TCPHeader)=1404` 字节。长实验确认的数据超过 2 GB 后发生有符号溢出，runner 和 `analyze_cellular_results.py` 又直接采用溢出后的 sender 摘要，因而把正常吞吐报告成低值。

| 场景/CC | 数据 ACK | 实际确认负载 | 2^32 回绕次数 | 回绕后预测 B/s | sender 报告 B/s |
| :--- | ---: | ---: | ---: | ---: | ---: |
| car1/bbrv2 | 3,914,275 | 5,495,642,100 B | 1 | 729,450.063 | 729,443 |
| car1/bbrv2plus | 3,911,230 | 5,491,366,920 B | 1 | 726,852.748 | 726,848 |
| static1/bbrv2 | 10,192,198 | 14,309,845,992 B | 3 | 452,650.605 | 452,648 |

预测值与日志仅差 3--7 B/s（少量收尾 ACK 的统计边界），证明低吞吐来自计数回绕，不是 BBRv2 没有发送数据。170 秒 Taxi 对照的数据量未超过 2 GB，bbrv2 的应用好吞吐/线速为 16.287680/17.030232 Mbps，也与该结论一致。

### 次因：低速段触发通用 CTCP 两秒无 ACK 重置

`User-space/ctcp.hh` 在连续 2 秒无 ACK 时调用 `congctrl.init()`，清空控制器模型并跳过当前未确认窗口。该逻辑位于模板传输层，bbrv2、bbrv2plus、obbr、bbr_r、fbbr、cubic 都会执行，不是 BBRv2 专属逻辑，也不会跨进程污染下一种 CC。

- car1/bbrv2 与 bbrv2plus 都有 58 次 timeout 和 59 次 init（含初始 init）；中止前的 obbr 也已出现 59 次 timeout。
- car1/bbrv2 的 54 个无 ACK 事件段中，52 个覆盖 `DL_bitrate <= 2 Kbit/s`，最长 ACK 间隔 8,165 ms。
- static1/bbrv2 有 94 次 timeout，其中 89 个事件段覆盖 `DL_bitrate <= 2 Kbit/s`，最长 ACK 间隔 5,148 ms。
- 因此重置主要由真实 trace 的近断流区间触发。它会损失控制器历史并影响断流恢复，但不能解释先前的 5.8/3.6 Mbps 报告值。

### 顺序污染排除

每种 CC 都由 runner 启动新的 sender 进程、receiver 和 mm-link namespace，且每个 mm-link 从 trace 时间 0 开始；控制器静态变量只存在于对应 sender 进程。没有证据表明 bbrv2 状态传入后续算法。若不先修复计数器，后续算法仍会得到错误吞吐，这是共享统计代码的问题，而不是运行顺序的问题。

## 修复与回归

1. 已将 `ctcp.hh` 的单流/累计字节和包计数改为 `uint64_t`，byte-switched 条件中的乘法也随左操作数提升为 64 位，并重新编译 sender。
2. 240 秒、100.009984 Mbps 固定链路回归确认约 202 万数据 ACK、约 2.84 GB 应用负载；sender 正确报告 94.664800 Mbps，线速 98.990005 Mbps，`mm-link=0`，没有回绕。
3. car1/static1 各六种 CC 的 10 秒并行短测共 12 项全部 `status=ok`、`mm-link=0`，吞吐均为正且无端口/进程干扰。
4. 未调整 2 秒无 ACK 时的全量 `init()`；这是所有 CC 共用的既有传输层语义，保持算法框架和比较口径不变。

## 巡检记录

- 12:57：两场景 bbrv2 均存活，日志连续增长，无已完成 CC。
- 13:30：car1/bbrv2 正常完成并进入 bbrv2plus；static1/bbrv2 仍在运行。发现 sender 摘要与线速严重背离。
- 14:09：car1/bbrv2plus、static1/bbrv2 完成；异常低 sender 摘要复现。
- 14:14：按要求停止两条实验进程树，未运行后续完整 CC。
- 14:23：完成模 2^32、ACK 数和短 Taxi 对照核验，确认主因是 `ctcp.hh` 32 位吞吐计数溢出；排除 BBRv2 跨算法状态污染。
- 14:33：完成超过 2 GB 的固定 100 Mbps 回归，64 位 sender 摘要与 Mahimahi 指标一致。
- 14:36：两个场景的全部六种 CC 并行短测通过。
- 14:37：修复后的 car1/static1 完整实验从头并行启动。

修复后的完整六算法排名运行中。
