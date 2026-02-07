#pragma once
#include <iostream>
#include <fstream>
#include <string>
#include <set>
#include "ns3/callback.h"
namespace ns3{
enum DqcTraceEnable:uint16_t{
    E_DQC_OWD=0x01,
    E_DQC_RTT=0x02,
    E_DQC_BW=0x04,
    E_DQC_GOODPUT=0x08,
    E_DQC_STAT=0x10,
    E_DQC_SEND_RATE=0x20,
    E_DQC_RECV_RATE=0x40,
    E_DQC_BBR_MODE=0x80,
    E_DQC_UP_PHASE=0x100,
    E_DQC_FREQ_ANALYSIS=0x200,
    E_DQC_INFLIGHT=0x400,
    E_DQC_LOSS_RATE=0x800,
    E_DQC_ACK_EVENT=0x1000,
    E_DQC_ACK_EPISODE=0x2000,
    E_DQC_QUEUE_DELAY=0x4000,
    E_DQC_ALL=E_DQC_OWD|E_DQC_RTT|E_DQC_BW|E_DQC_STAT,
};
void set_dqc_trace_folder(std::string &path);
struct AckEventRecord{
    double time_sec{0.0};
    uint64_t acked_bytes{0};
    uint32_t acked_pkts{0};
    uint64_t largest_acked{0};
    uint32_t ack_delay_ms{0};
    uint32_t rtt_ms{0};
    double ack_interval_ms{0.0};
    double ack_rate_kbps{0.0};
    double pacing_rate_kbps{0.0};
    double sample_bias{0.0};
};
struct AckEpisodeRecord{
    int32_t type{0};
    double start_sec{0.0};
    double end_sec{0.0};
    double duration_ms{0.0};
    uint32_t ack_events{0};
    uint64_t acked_bytes{0};
    double iat_min_ms{0.0};
    double iat_max_ms{0.0};
    double ack_rate_peak_kbps{0.0};
    double pacing_rate_mean_kbps{0.0};
    double bias_peak{0.0};
};
class DqcTrace{
public:
    DqcTrace(int id=0);
    ~DqcTrace();
    typedef Callback<void,uint32_t,uint64_t,uint64_t,uint64_t,float> TraceStats;
    void SetStatsTraceFuc(TraceStats cb){
        m_traceStatsCb=cb;
    }
    void SetCongestionControlType(uint32_t type);
    void Log(std::string name,uint16_t enable);
    void OnOwd(uint32_t seq,uint32_t owd,uint32_t size);
    void OnRtt(uint32_t seq,uint32_t rtt,uint32_t smoothed_rtt);
    void OnBw(int32_t kbps);
    void OnGoodput(uint32_t kbps);
    void OnSendRate(int32_t kbps);
    void OnRecvRate(int32_t instant_kbps);
    void OnQueueDelay(uint32_t queue_delay_ms,uint32_t latest_rtt_ms,uint32_t min_rtt_ms);
    void OnInflight(int32_t inflight_bytes,int32_t cwnd_bytes);
    void OnBbrMode(int32_t mode);
    void OnLossRate(double time_sec,float loss_rate,float cumulative_loss_rate);
    void OnAckEvent(const AckEventRecord &record);
    void OnAckEpisode(const AckEpisodeRecord &record);
    void OnUpPhase(double start_time,double duration_ms,double freq_hz,bool exit_due_to_queueing,int cycles,float pacing_gain,int32_t bw_estimate_kbps);
    void OnFreqAnalysis(double start_time, double duration_sec, double sender_peak_freq_hz, double receiver_peak_freq_hz, int32_t avg_rate_kbps);
    void OnStats(uint64_t recv_count,uint64_t largest,
                 uint64_t recv_bytes,uint64_t duration,
                       float avg_owd);
private:
    void Close();
    void OpenOwdFile();
    void OpenRttFile();
    void OpenBandwidthFile();
    void OpenGoodputFile();
    void OpenSendRateFile();
    void OpenRecvRateFile();
    void OpenQueueDelayFile();
    void OpenInflightFile();
    void OpenBbrModeFile();
    void OpenLossRateFile();
    void OpenAckEventFile();
    void OpenAckEpisodeFile();
    void OpenUpPhaseFile();
    void OpenFreqAnalysisFile();
    void OpenStatsFile();
    void CloseOwdFile();
    void CloseRttFile();
    void CloseBandwidthFile();
    void CloseGoodputFile();
    void CloseSendRateFile();
    void CloseRecvRateFile();
    void CloseQueueDelayFile();
    void CloseInflightFile();
    void CloseBbrModeFile();
    void CloseLossRateFile();
    void CloseAckEventFile();
    void CloseAckEpisodeFile();
    void CloseUpPhaseFile();
    void CloseFreqAnalysisFile();
    void CloseStatsFile();
    int m_id=0;
    std::string m_name;       // Store the log name for lazy file opening
    uint16_t m_enable=0;      // Store the enable flags
    uint32_t m_ccType=0;
    TraceStats m_traceStatsCb;
    std::fstream m_owd;
    std::fstream m_rtt;
    std::fstream m_bw;
    std::fstream m_googput;
    std::fstream m_sendRate;
    std::fstream m_recvRate;
    std::fstream m_queueDelay;
    std::fstream m_inflight;
    std::fstream m_bbrMode;
    std::fstream m_lossRate;
    std::fstream m_ackEvent;
    std::fstream m_ackEpisode;
    std::fstream m_upPhase;
    std::fstream m_freqAnalysis;
    std::fstream m_stats;
    int32_t m_lastBbrMode = -1;  // Track last BBR mode to avoid duplicate records
    int64_t m_lastBwTimeUs = -1; // Track last bw timestamp to avoid same-time duplicates
};
class DqcTraceState{
public:
    DqcTraceState(std::string name);
    ~DqcTraceState();
    void OnStats(uint32_t id,uint64_t recv_count,uint64_t largest,
                 uint64_t recv_bytes,float avg_owd);
    void Flush(uint32_t capacity,uint32_t simulation_time);
    void RecordRuningTime(float millis,float mimutes);
    void ReisterAvgDelayId(uint32_t id);
    //compare inter-protocol fairness and rtt unfairness
    void RegisterCongestionType(uint32_t id,uint32_t type=0);
    void Reset();
private:
    std::fstream m_stats;
    int m_count=0;
    uint64_t m_recvCount=0;
    uint64_t m_totalRecv=0;
    uint64_t m_totalRecvBytes=0;
    uint64_t m_delayCount=0;
    uint64_t m_sumDelay=0;
    std::set<uint32_t> m_delayIds;
    std::set<uint32_t> m_ccType1Ids;
    uint64_t m_ccType1TotalRecvBytes=0;
    uint64_t m_ccType2TotalRecvBytes=0;
};
}
