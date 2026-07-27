# FBBR-hybrid 量化负载判定分支

`FBBR-hybrid` 是按仓库根目录 `ghtjykukli.pdf` 与
`FBBR_通用负载判定_新版量化改造方案.md` 实现的历史实验分支。它与当前
`FBBR` 的 service envelope 以及 `FBBR-adaptive` 保持独立，用拥塞控制类型
`kFBBRHybrid` 承载 N01–N16 判定树、连接状态和执行器。

## 使用

四流验证程序：

```bash
./waf --run "scratch/fbbr_4flow --algo=FBBR-hybrid \
  --fbbrConfig=examples/CCconfig/fbbr_default.conf \
  --sim_time=30 --flowSizeBytes=0 --smokeMode=false \
  --enableCruiseWindowTrace=true --trace_path=/path/to/output"
```

通用批量拓扑程序的算法列表同样接受 `FBBR-hybrid`。默认参数集中在
`examples/CCconfig/fbbr_default.conf` 的 “FBBR-hybrid 专属” 段。

合成回归入口：

```bash
./waf --run "scratch/fbbr_4flow --fbbrHybridSelfTest=true"
```

## 管线与状态隔离

混合分支在两周期窗口上对 sender rate、最新 delivery rate 和 SRTT 做
统一时间重采样。SRTT 上/下横切分别按 U1/U2/U3 与 L1/L2/L3 验真，
上横切优先；疑似横切若未通过验真会回退到普通波动树。DRate 横切不直接
分类，其中上横切只是否决 DRate 周期相似，普通波动判定不受横切掩码影响。
SRTT 横切只有在同一窗口里还观测到普通 SRTT 波动时才作为有效输入；
这里的普通波动只要单个正半周期或负半周期成立就算。若只看到横线候选
而没有这类波动，就只保留候选痕迹并回退到普通波动树，不进入横切分类，
也不更新横切相关的上下界。

纯分类器严格按 N01–N16 返回 Regime I（欠载）、II（满载）或 III
（过载）。输入覆盖或必需谓词无效时才返回不确定。连接级 `MaxRTT`、
`srtt_low/RTprop` 与 `RTpropDRate` 跨 Cruise 保存；除 PDF 判定窗口来源
外，可信 ProbeRTT 和第三 Cruise 兜底平台也可以事务式更新低位参考。

执行器按 `adaptive_change.pdf` 做兼容分层：

- Regime II：本轮 `TrustedBw` 取窗口 DRate 时间均值；注入基线保持不变。
- N12/N16 作为各自回退分支的 `else` 时，若当前窗口平均 SRTT 高于
  `RTprop + (MaxRTT - RTprop) / 4`，或当前 inflight 达到 `1.1BDP`，则判
  Regime III，否则判 Regime II。BDP 通过 BBR 模型当前 `model_.BDP()`
  计算：有 `TrustedBw` 覆盖时使用 `TrustedBw * RTT`，没有 `TrustedBw`
  时退回原 BBRv2 `MaxBandwidth * RTT`。两个谓词至少一个可计算，否则返回
  不确定。
- Cruise 结束时只发布已经由 Regime II 锁定的 `TrustedBw`，不再用上一
  Cruise 交付速率时间加权均值兜底生成 `tmpBw`。
- N12/N16 无论判 Regime II 还是 Regime III，都不更新 MaxRTT、
  RTpropDRate、baseline_low 或 baseline_up。分类后的状态更新只发生在
  N07/N09/N11/N15 与 N02/N04/N05/N10/N14 这些明确分支中。
- N07/N09/N11/N15 的 Regime I 用窗口最小 DRate 更新
  `baseline_low/RTpropDRate`；其中 N09 保持 RTprop 不变，其余刷新
  RTprop。N02/N04/N05/N10/N14 的 Regime III 用窗口最大 DRate 更新
  `baseline_up/MaxRTT`。
- Hybrid 不使用 Adaptive 的 25% MaxBw 跨 Cruise 门控。已经获得的
  `baseline_low/up`、`RTpropDRate`、`MaxRTT` 和 BBR 模型 `RTprop` 均无条件
  继承到后续 Cruise，直到被对应规则的新有效窗口覆盖。
- Regime I/III 首先尝试 Adaptive 夹逼目标。两侧边界有效且
  `baseline_up > baseline_low` 时，Regime I 在当前基线低于
  `low + gap/2` 时取该目标，Regime III 在当前基线高于
  `low + gap/4` 时取该目标。
- 夹逼条件不成立时，再严格判断
  `maxdrate-mindrate > 0.5*(maxdrate-RTpropDRate)`；成立则取
  `mindrate + 0.5*(maxdrate-mindrate)`。这里 `RTpropDRate` 是 RTprop
  对应的交付速率，保证公式两边量纲同为 bit/s。
- 上述两级均不成立时，Regime I 取 `maxdrate`，Regime III 取
  `mindrate`。
- SRTT 比较边界不沿用 Adaptive 的窗口均值状态：`srtt_low` 明确是带来源
  时间戳的 Hybrid `RTprop`，`srtt_max` 明确取连接级 `MaxRTT`。
- DRate 统计或执行器输入无效时，整个动作以事务方式放弃，不能部分更新
  baseline、`baseline_low/up`、TrustedBw、MaxRTT、RTprop 或
  RTpropDRate。

低位参考补全与 ProbeRTT 规则：

- N01-N16 分类器不因最小 SRTT 精确接触 `srtt_low/RTprop` 额外更新低位
  参考；分类副作用只跟随上面显式列出的 rule。第三 Cruise 下界搜索和
  ProbeRTT 仍可以用各自事务式路径补全低位参考。
- 前两个 Cruise 后两项低位速率仍缺失，则从第三个 Cruise 起按每个完成
  窗口乘 0.80 降低 pacing baseline。`<0.5BDP` 使用搜索开始时冻结的
  MaxBw 与可信 `srtt_low` 计算；达到后进入稳定平台，至少保持 200ms 且
  跨一个 packet-timed RTT。即使 Cruise 中途结束，平台 pacing 上限也继续
  生效，完成前不得提交 RTprop 或退出事务。
- 每次 ProbeRTT 同样至少保持 200ms 并跨一个 packet-timed RTT，0.5BDP
  用 Hybrid 保存的可信 `srtt_low` 计算。ProbeRTT 完成时间晚于当前来源
  时间时可更新 `srtt_low`；若低位速率尚缺失，或旧 `srtt_low` 大于本次
  ProbeRTT 的 RTprop，则用同一 ProbeRTT 的 DRate 同时更新
  `baseline_low/RTpropDRate`。
- ProbeRTT DRate 不写入 MaxBw。稳定平台的首轮仅用于排空过渡；优先对其后
  有效 DRE 样本取下中位数，样本不足时退回整个平台的下中位数。DRE 本身
  取 send/ACK rate 较小者以抑制 ACK compression；下中位数避免单个 ACK
  间隔把 minimum 拉低，也避免 burst 把 maximum/mean 抬高。

连续两个有效滑窗中 SRTT 或 DRate 任一信号无普通波动时进入保真增强。
触发窗口冻结分类副作用，先后探一个周期；再次失败后探针幅度乘 1.25，
最多到本 Cruise 初始幅度的 2 倍，达到上限仍滚动采窗。进入增强后任一信号
恢复普通波动就退出。

`FBBR-adaptive` 不调用上述 V1 分类器、状态或执行器；当前 `FBBR` 走独立的
service-envelope 路径。窗口轨迹中的 `regime_pipeline_owner` /
`regime_actuator_owner` 可用于自动审计：混合分支固定为
`fbbr_hybrid_v2`，旧分支固定为 `legacy`。

## 关键默认门限

文档给定的硬语义保持固定：两窗触发、任一波动恢复、每次滚动一个周期、
重复小横切首末跨度至少 0.50W、DRate 上横切周期硬否决。主要可调门限包括：

- 普通波动：幅度噪声倍数 6、最小幅度比例 0.02、归一化步进斜率 3.5、
  有效步比例 0.10、方向变化比例 0.20、显著路径比例 0.80。
- 连续横线：最短 0.15T；U2/L2 最终长度阈值分别严格大于 0.20T/0.30T。
- 重复短接触：每周期至少 2 点、总计至少 4 点、首末跨度至少 0.50W。
- 周期相似：周期误差不超过 0.20，相关系数至少 0.50。

量化轨迹在原 waveform CSV 后追加了原始候选、六类证据、普通波动三门、
周期输入有效性、Nxx rule、前后状态、两窗 streak、抑制位、窗口推进和幅度
计数。任何分析工具都应按表头名称读取，不应依赖列号。

本次开发和阈值检验的独立实验目录为
`/mnt/nasDisk_ds3617/wkd/1FreqBBR/fbbr_quant_20260721_codex`。
