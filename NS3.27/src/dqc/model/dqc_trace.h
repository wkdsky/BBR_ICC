#pragma once
#include <iostream>
#include <fstream>
#include <string>
#include <set>
#include "ns3/callback.h"
namespace ns3{
enum DqcTraceEnable:uint32_t{
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
    E_DQC_QUEUE_DELAY=0x4000,
    E_DQC_RECV_RATE_RAW=0x8000,
    E_DQC_FREQCCV4_LOAD=0x10000,
    E_DQC_FREQCCV4_GATE=0x20000,
    E_DQC_FBBR_LOAD=0x40000,
    E_DQC_FBBR_GATE=0x80000,
    E_DQC_ALL=E_DQC_OWD|E_DQC_RTT|E_DQC_BW|E_DQC_STAT,
};
void set_dqc_trace_folder(std::string &path);
class DqcTrace{
public:
    DqcTrace(int id=0);
    ~DqcTrace();
    typedef Callback<void,uint32_t,uint64_t,uint64_t,uint64_t,float> TraceStats;
    void SetStatsTraceFuc(TraceStats cb){
        m_traceStatsCb=cb;
    }
    void SetCongestionControlType(uint32_t type);
    void Log(std::string name,uint32_t enable);
    void OnOwd(uint32_t seq,uint32_t owd,uint32_t size);
    void OnRtt(uint32_t seq,uint32_t rtt,uint32_t smoothed_rtt);
    void OnBw(int32_t kbps);
    void OnGoodput(uint32_t kbps);
    void OnSendRate(int32_t kbps);
    void OnRecvRate(int32_t bandwidth_latest_kbps);
    void OnRecvRateRaw(int32_t delivery_rate_kbps);
    void OnQueueDelay(uint32_t queue_delay_ms,uint32_t latest_rtt_ms,uint32_t min_rtt_ms);
    void OnInflight(int32_t inflight_bytes,int32_t cwnd_bytes);
    void OnBbrMode(int32_t mode);
    void OnLossRate(double time_sec,float loss_rate,float cumulative_loss_rate);
    void OnUpPhase(double start_time,double duration_ms,double freq_hz,bool exit_due_to_queueing,int cycles,float pacing_gain,int32_t bw_estimate_kbps);
    void OnFreqAnalysis(double start_time, double duration_sec, double sender_peak_freq_hz, double receiver_peak_freq_hz, int32_t avg_rate_kbps);
    void OnRttFreqAnalysis(double start_time, double duration_sec, double sender_peak_freq_hz, double rtt_peak_freq_hz, double avg_smoothed_rtt_ms);
    void OnFreqCCv4Load(double window_start_s, double window_end_s,
                        double p_underload, double p_full_load,
                        double p_overload, double confidence,
                        std::string label, bool low_confidence,
                        std::string diagnostics);
    void OnFBBRLoad(double window_start_s, double window_end_s,
                    double p_underload, double p_full_load,
                    double p_overload, double confidence,
                    std::string label, bool low_confidence,
                    std::string diagnostics);
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
    void OpenRecvRateRawFile();
    void OpenQueueDelayFile();
    void OpenInflightFile();
    void OpenBbrModeFile();
    void OpenLossRateFile();
    void OpenUpPhaseFile();
    void OpenFreqAnalysisFile();
    void OpenRttFreqAnalysisFile();
    void OpenFreqCCv4LoadFile();
    void OpenFreqCCv4CruiseSummaryFile();
    void OpenFreqCCv4GateFile();
    void OpenFbbrGateFile();
    void OpenFbbrTriggerCycleFile();
    void OpenFbbrBinFile();
    void OpenFbbrEventWindowFile();
    void OpenFbbrCruiseFile();
    void OpenFbbrDiagnosticWindowFile();
    void OpenFbbrQueueServoFile();
    void OpenStatsFile();
    void CloseOwdFile();
    void CloseRttFile();
    void CloseBandwidthFile();
    void CloseGoodputFile();
    void CloseSendRateFile();
    void CloseRecvRateFile();
    void CloseRecvRateRawFile();
    void CloseQueueDelayFile();
    void CloseInflightFile();
    void CloseBbrModeFile();
    void CloseLossRateFile();
    void CloseUpPhaseFile();
    void CloseFreqAnalysisFile();
    void CloseRttFreqAnalysisFile();
    void CloseFreqCCv4LoadFile();
    void CloseFreqCCv4CruiseSummaryFile();
    void CloseFreqCCv4GateFile();
    void CloseFbbrGateFile();
    void CloseFbbrTriggerCycleFile();
    void CloseFbbrBinFile();
    void CloseFbbrEventWindowFile();
    void CloseFbbrCruiseFile();
    void CloseFbbrDiagnosticWindowFile();
    void CloseFbbrQueueServoFile();
    void CloseStatsFile();
    int m_id=0;
    std::string m_name;       // Store the log name for lazy file opening
    uint32_t m_enable=0;      // Store the enable flags
    uint32_t m_ccType=0;
    TraceStats m_traceStatsCb;
    std::fstream m_owd;
    std::fstream m_rtt;
    std::fstream m_bw;
    std::fstream m_googput;
    std::fstream m_sendRate;
    std::fstream m_recvRate;
    std::fstream m_recvRateRaw;
    std::fstream m_queueDelay;
    std::fstream m_inflight;
    std::fstream m_bbrMode;
    std::fstream m_lossRate;
    std::fstream m_upPhase;
    std::fstream m_freqAnalysis;
    std::fstream m_rttFreqAnalysis;
    std::fstream m_freqccv4Load;
    std::fstream m_freqccv4CruiseSummary;
    std::fstream m_freqccv4Gate;
    std::fstream m_fbbrGate;
    std::fstream m_fbbrTriggerCycle;
    std::fstream m_fbbrBin;
    std::fstream m_fbbrEventWindow;
    std::fstream m_fbbrCruise;
    std::fstream m_fbbrDiagnosticWindow;
    std::fstream m_fbbrQueueServo;
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
    void Flush(uint32_t capacity,double simulation_time);
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
