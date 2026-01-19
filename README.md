ICC（Inter-flow Congestion Control）的发送速率波动与频域检测相关算法主要位于以下位置：

  核心文件位置

  | 实现版本    | 文件路径
                              |
  |---------|-------------------------------------------
  --------------------|
  | 用户空间实现  | /home/wkd/BBR_ICC/GenericCC/ICC.cc
                            |
  | NS3仿真实现 | /home/wkd/BBR_ICC/NS3.27/src/internet/
  model/tcp-periodicDC.cc |

  ---
  频域检测算法 (find_fm 函数)

  用户空间版本

  - 文件: GenericCC/ICC.cc
  - 行号: 120-185

  NS3仿真版本

  - 文件: NS3.27/src/internet/model/tcp-periodicDC.cc
  - 行号: 965-1052

  算法核心步骤：

  1. 使用 FFTW3
  库对拥塞窗口(CWND)和排队延迟(Qd)序列做FFT变换
  2. 提取主导频率 fm
  3. 计算CWND和RTT频谱的相似度 (similarity)
  4. 根据相似度阈值判断周期性状态

  ---
  发送速率波动检测

  数据采集 (时域)

  - 用户空间 (ICC.cc:312-318):
  P_cwnd.push_back(_the_window);   // 拥塞窗口样本
  P_qd.push_back(rtt_measured);    // 排队延迟样本
  - NS3 (tcp-periodicDC.cc:391-392):
  QdArray.push_back(Qd);
  CwndArray.push_back(tcb->m_cWnd);

  周期性检测逻辑 (tcp-periodicDC.cc:435-439)

  if(fm>0 && oldFm-std::min(5*fmth,2.0)<=fm &&
     fm<=oldFm+std::min(5*fmth,2.0) &&
     curAm0>=0.9*preAm0 && curAm0<=1.1*preAm0)
    // 检测到周期性状态

  ---
  关键参数

  - fm: 主导频率 (Hz)
  - fs: 采样频率 = n/cycle
  - similarity: FFT频谱归一化差异
  - simiD: 时域指标，判断Qd和CWND变化的相关性



  BBRv2 PROBE_BW 各阶段退出条件

  1. PROBE_DOWN (探测下降阶段) 退出条件

  1) 退出到 PROBE_REFILL 的条件（quic_bbr2_probe_bw.cc:126-180）：

  1.1) 快速退出：如果上一轮 stopped_risky_probe=true 且 probed_too_high=false，在第一轮结束时直接进入  REFILL（第142-145行）
  1.2) 时间到达：IsTimeToProbeBandwidth() 返回 true（第150-153行），具体包括：
    - 周期持续时间超过 probe_wait_time（随机值 = base_duration + 随机延迟）
    - 或者满足 Reno 共存条件：rounds_since_probe >= min(probe_bw_probe_max_rounds, reno_rounds)

  2) 退出到 PROBE_CRUISE 的条件：

  2.1) 比例时间退出：HasStayedLongEnoughInProbeDown() 返回 true，即在 PROBE_DOWN 阶段停留时间超过  MinRtt（第155-159行）

  2.2) 排空完成：同时满足以下两个条件（第161-179行）：
    - prior_in_flight <= inflight_hi_with_headroom（有足够的 headroom，默认设定为0.85 inflight_hi）
    - prior_in_flight < BDP（已排空到目标值以下）

  ---
  2. PROBE_CRUISE (巡航阶段) 退出条件

  1) 退出到 PROBE_REFILL（quic_bbr2_probe_bw.cc:379-389）：
  - IsTimeToProbeBandwidth() 返回 true，即：
    - 周期持续时间超过 probe_wait_time
    - 或者满足 Reno 共存条件

  --- 
  3. PROBE_REFILL (填充阶段) 退出条件

  1) 退出到 PROBE_UP（quic_bbr2_probe_bw.cc:391-401）：
  - rounds_in_phase > 0 且 当前是一轮的结束（end_of_round_trip = true）
  - 即：完成一个完整的 RTT 轮次后退出

  ---
  4. PROBE_UP (探测上升阶段) 退出条件

  1) 退出到 PROBE_DOWN（quic_bbr2_probe_bw.cc:403-450）：

  1.1) 探测过高：MaybeAdaptUpperBounds() 返回 ADAPTED_PROBED_TOO_HIGH（第407-410行）
    - 条件：丢包事件数 >= probe_bw_full_loss_count 且 IsInflightTooHigh() 为 true
  
  1.2) 风险探测（is_risky）：上一周期 probed_too_high=true ( 丢包事件 ≥ 2 且 (丢包率 > 2% 或 ECN 标记过多)) 且 prior_in_flight >= inflight_hi（第419-426行）
  
  1.3) 队列堆积（is_queuing）：rounds_in_phase > 0 且 prior_in_flight >= queuing_threshold（第428-444行）
    - 其中 queuing_threshold = probe_bw_probe_inflight_gain * BDP + 2*MSS + MaxAckHeight

  状态转换图：

  PROBE_DOWN ──┬──> PROBE_REFILL ──> PROBE_UP ──┐
               │                                 │
               └──> PROBE_CRUISE ──> PROBE_REFILL│
                                                 │
               <─────────────────────────────────┘

  ---
  inflight_hi 与 BDP 的关系

  1. 初始值

  - inflight_hi 初始值为 std::numeric_limits<QuicByteCount>::max()（无穷大）
  - 在 STARTUP 退出时首次被设置为 BDP：
  // quic_bbr2_startup.cc:96, 128
  model_->set_inflight_hi(bdp);

  2. 运行时的动态变化

  | 场景                    | inflight_hi 的变化                                    | 与 BDP
  的关系                  |
  |-----------------------|----------------------------------------------------|---------------
  -------------|
  | STARTUP 退出            | 设为 BDP                                             |
  inflight_hi = BDP          |
  | PROBE_UP 探测成功         | 逐渐增加（每轮 +1 MSS 起步，指数增长）
     | inflight_hi > BDP          |
  | PROBE_UP 探测过高         | 降低为 max(inflight_at_send, target * (1-β))，其中 β=0.3 |
  inflight_hi ≈ 0.7 * target |
  | PROBE_DOWN/CRUISE 无丢包 | 如果 inflight_at_send > inflight_hi，则提升              | 可能
  > BDP                   |

  3. 通常谁更小？

  在稳态下，inflight_hi 通常略大于或等于 BDP：

  inflight_hi ≥ BDP  （正常情况）

  原因：
  1. BBRv2 的目标是让 inflight_hi 略高于 BDP，以便在 PROBE_UP 阶段探测更多带宽
  2. PROBE_UP 阶段会持续增加 inflight_hi，直到检测到丢包/拥塞
  3. 检测到拥塞后，inflight_hi 被削减，但通常仍 ≥ BDP

  特殊情况（inflight_hi < BDP）：
  - 刚经历严重丢包，inflight_hi 被大幅削减
  - 网络带宽突然增加，BDP 变大但 inflight_hi 还未来得及探测上去

  设计意图：inflight_hi 代表"不会导致过度丢包的最大 inflight 量"，它应该略高于
  BDP，以便充分利用带宽，同时通过 headroom 预留空间避免持续排队。


freqccv2，以BBRv2为蓝本，但down的pacing gain小一些，改为0.5，退出条件改为当前RTT小于等于1.05倍的min RTT或至少运行了两个min rtt的时间，up的pacing gain小一些，改为1.1（这些都不要改动原来的BBRv2），此外，让其也在新的BBRv2上进行波动，波动方式和模式与freqcc相同，但给波动模式增加一个 refill_up模式，即让波动只存在于refill和up阶段，其他参数和选项与freqcc一致

 * FreqCCv2 features:
 *  Amplitude mode : 2miu,3miu,4miu,8miu,2sr,3sr,4sr,8sr or Mbps value
 *   - PROBE_DOWN pacing gain: 0.5 (vs BBRv2's 0.75)
 *   - PROBE_UP pacing gain: 1.1 (vs BBRv2's 1.25)
 *   - PROBE_DOWN exit: RTT <= 1.05*min_RTT OR duration >= 2*min_RTT
 *   - Supports oscillation modes: after_drain, only_probeBW, refill_up
  1. after_drain -  在Drain阶段结束后开始波动，所有模式都波动
  2. only_probeBW - 仅在ProbeBW阶段波动
  3. refill_up -  仅在PROBE_REFILL和PROBE_UP阶段波动

  kFixed,      // Fixed amplitude in bps
  kMiu2,       // 1/2 of max bandwidth (miu)
  kMiu3,       // 1/3 of max bandwidth
  kMiu4,       // 1/4 of max bandwidth
  kMiu8,       // 1/8 of max bandwidth
  kSR2,        // 1/2 of current sending rate
  kSR3,        // 1/3 of current sending rate
  kSR4,        // 1/4 of current sending rate
  kSR8,        // 1/8 of current sending rate


freqccv3：Cruise阶段结束时，用new_refill替换原来的refill阶段，在new_refill中检查inflight与BDP的情况：
- 若Inflight > 0.75*BDP + 2*MSS + MaxAckHeight ，则更新参数，且设置pacing_gain=0.75，直到inflight<=0.75*BDP + 2*MSS + MaxAckHeight 时退出；
- 若inflight<=0.70*BDP + 2*MSS + MaxAckHeight ，更新参数，设置pacing_gain与原refill一致，直到inflight>=0.75*BDP + 2*MSS + MaxAckHeight 时退出；
- 若以上两个条件都不满足，更新参数后直接退出；
- 更新参数按原refill规则更新。

  与原版 BBRv2 REFILL 的区别

  | 特性              | 原版 BBRv2 REFILL | FreqCCv3 NEW_REFILL
  |
  |-----------------|-----------------|---------------------|
  | pacing_gain     | 固定 1.0          | 动态 (0.75 或 1.0)
   |
  | 持续时间            | 固定 1 RTT        | 根据 inflight
  动态调整    |
  | 进入时 inflight 过高 | 继续以 1.0 发送      | 先用 0.75 排空
            |
  | 进入时 inflight 过低 | 继续以 1.0 发送      | 用 1.0 填充
           |
  | 进入时 inflight 合适 | 等待 1 RTT        | 立即退出
         |

  设计目的

  1. 避免 PROBE_UP 时 inflight 过高：如果从 CRUISE 出来时
  inflight 已经偏高，先用 pacing_gain=0.75 排空，避免进入
  PROBE_UP 后立即触发拥塞
  2. 加速 PROBE_UP 的到来：如果 inflight
  已经在合适范围（0.70~0.75 BDP），不需要等待完整的 1
  RTT，可以立即进入 PROBE_UP
  3. 确保 PROBE_UP 起点一致：无论从什么状态进入，都尽量让
  inflight 稳定在 ~0.75*BDP 附近再开始探测

new_refill退出后进入up阶段，在up阶段按freqccv2 的波动方式进行周期性波动（只在up阶段波动，其余阶段不波动，即没有波动模式这个参数，但有其他参数，包括频率和幅度），Up阶段的退出条件与原方案保持一致

  # 使用默认参数（所有流相同）
  ./waf --run "8_freqccv3"

  # 为不同流设置不同频率
  ./waf --run "8_freqccv3 --freq1=1.0 --freq2=2.0 --freq3=1.5 
  --freq4=0.5"

  # 为不同流设置不同振幅模式
  ./waf --run "8_freqccv3 --amp1=miu2 --amp2=miu4 --amp3=sr2 
  --amp4=miu8"

  # 混合配置
  ./waf --run "8_freqccv3 --freq1=1.0 --amp1=miu2 --freq2=2.0 
  --amp2=miu4 --sim_time=100"

接收速率是 BandwidthEstimate()记录的，它是BBR的带宽窗口估计，是一个平滑的、持久的值，不会跟随发送速率快速振荡。ICC中分析的是瞬时ACK速率。

速率对比画图：python3 scripts/analyze_oscillation.py 2>&1

  | 文件            | 表头
                         |
  |---------------|----------------------------------------------
  -------------------|
  | _rtt.txt      | #time(s) seq rtt(ms) smoothed_rtt(ms)
                     |
  | _recvrate.txt | #time(s) instant_recv_rate(kbps) 
  bw_estimate(kbps)              |
  | _upphase.txt  | #start_time(s) duration(ms) freq(Hz) cycles 
  bw_estimate(kbps)   |
  | _sendrate.txt | #time(s) pacing_rate(kbps)
                     |
  | _bw.txt       | #time(s) bandwidth(kbps)
                     |
  | _bbrmode.txt  | #time(s) mode
                     |
  | _owd.txt      | #time(s) seq owd(ms) size(bytes)
                     |
  | _good.txt     | #time(s) goodput(kbps)
                     |
  | _stats.txt    | #loss_rate(%) avg_throughput(kbps) 
  avg_owd(ms) total_recv_bytes |