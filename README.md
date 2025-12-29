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