# FBBR-hybrid 量化负载判定分支

`FBBR-hybrid` 是按仓库根目录 `ghtjykukli.pdf` 与
`FBBR_通用负载判定_新版量化改造方案.md` 实现的新算法分支。它保留原
`FBBR` 和 `FBBR-adaptive` 的行为，用独立拥塞控制类型
`kFBBRHybrid` 承载新版 N01–N18 判定树、连接状态和执行器。

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

纯分类器严格按 N01–N18 返回 Regime I（欠载）、II（满载）或 III
（过载）。输入覆盖或必需谓词无效时才返回不确定。连接级 `MaxRTT` 与
`RTpropDRate` 跨 Cruise 保存；触发更新的规则分别只有 N02/N04/N05 和
N06/N10/N11。

执行器规则为：

- Regime II：下一注入基线和本轮 `ProbedBw` 都取窗口 DRate 时间均值。
- Regime I/III：若
  `maxdrate-mindrate > 0.5*(maxdrate-RTpropDRate)`，两者都取
  `mindrate + 0.5*(maxdrate-mindrate)`；否则分别取 max/min。
- DRate 统计或执行器输入无效时，整个动作以事务方式放弃，不能部分更新
  baseline、ProbedBw、MaxRTT、RTprop 或 RTpropDRate。

连续两个有效滑窗中 SRTT 或 DRate 任一信号无普通波动时进入保真增强。
触发窗口冻结分类副作用，先后探一个周期；再次失败后探针幅度乘 1.25，
最多到本 Cruise 初始幅度的 2 倍，达到上限仍滚动采窗。进入增强后任一信号
恢复普通波动就退出。

`FBBR`、`FBBR-adaptive` 不调用上述分类器、状态或执行器。窗口轨迹中的
`regime_pipeline_owner` / `regime_actuator_owner` 可用于自动审计：混合分支
固定为 `fbbr_hybrid_v2`，旧分支固定为 `legacy`。

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
