#include <unistd.h>
#include <dirent.h>
#include <memory.h>
#include <string>
#include "ns3/dqc_trace.h"
#include "ns3/simulator.h"
#include "proto_types.h"
//app http://www.cplusplus.com/reference/fstream/fstream/open/
namespace ns3{
std::string kDqcTracePath;
void set_dqc_trace_folder(std::string &path){
    kDqcTracePath=path;
}
DqcTrace::DqcTrace(int id):m_id(id){}
DqcTrace::~DqcTrace(){
    Close();
}
void DqcTrace::SetCongestionControlType(uint32_t type){
    m_ccType=type;
}
void DqcTrace::Log(std::string name,uint32_t enable){
    m_name = name;
    m_enable = enable;
    if(m_enable & E_DQC_FBBR_LOAD){
        OpenFBBRWaveformSearchFile();
    }
}
void DqcTrace::OpenOwdFile(){
    if(!(m_enable & E_DQC_OWD) || m_owd.is_open()) return;
    char buf[FILENAME_MAX];
    memset(buf,0,FILENAME_MAX);
    std::string path;
    if(0==kDqcTracePath.size()){
        path= std::string (getcwd(buf, FILENAME_MAX)) + "/traces/"
            +m_name+"_owd.txt";
    }else{
        path=std::string(kDqcTracePath)+m_name+"_owd.txt";
    }
    m_owd.open(path.c_str(), std::fstream::out);
    if(m_owd.is_open()){
        m_owd<<"#time(s)\tseq\towd(ms)\tsize(bytes)"<<std::endl;
    }
}
void DqcTrace::OpenRttFile(){
    if(!(m_enable & E_DQC_RTT) || m_rtt.is_open()) return;
    char buf[FILENAME_MAX];
    memset(buf,0,FILENAME_MAX);
    std::string path;
    if(0==kDqcTracePath.size()){
        path= std::string (getcwd(buf, FILENAME_MAX)) + "/traces/"
            +m_name+"_rtt.txt";
    }else{
        path=std::string(kDqcTracePath)+m_name+"_rtt.txt";
    }
    m_rtt.open(path.c_str(), std::fstream::out);
    if(m_rtt.is_open()){
        m_rtt<<"#time(s)\tseq\trtt(ms)\tsmoothed_rtt(ms)"<<std::endl;
    }
}
void DqcTrace::OpenBandwidthFile(){
    if(!(m_enable & E_DQC_BW) || m_bw.is_open()) return;
    char buf[FILENAME_MAX];
    memset(buf,0,FILENAME_MAX);
    std::string path;
    if(0==kDqcTracePath.size()){
        path = std::string (getcwd(buf, FILENAME_MAX)) + "/traces/"
                +m_name+"_bw.txt";
    }else{
        path=std::string(kDqcTracePath)+m_name+"_bw.txt";
    }
    m_bw.open(path.c_str(), std::fstream::out);
    if(m_bw.is_open()){
        m_bw<<"#time(s)\tbandwidth(kbps)"<<std::endl;
    }
}
void DqcTrace::OpenGoodputFile(){
    if(!(m_enable & E_DQC_GOODPUT) || m_googput.is_open()) return;
    char buf[FILENAME_MAX];
    memset(buf,0,FILENAME_MAX);
    std::string path;
    if(0==kDqcTracePath.size()){
        path=std::string (getcwd(buf, FILENAME_MAX)) + "/traces/"
            +m_name+"_good.txt";
    }else{
        path=std::string(kDqcTracePath)+m_name+"_good.txt";
    }
    m_googput.open(path.c_str(), std::fstream::out);
    if(m_googput.is_open()){
        m_googput<<"#time(s)\tgoodput(kbps)"<<std::endl;
    }
}
void DqcTrace::OpenSendRateFile(){
    if(!(m_enable & E_DQC_SEND_RATE) || m_sendRate.is_open()) return;
    char buf[FILENAME_MAX];
    memset(buf,0,FILENAME_MAX);
    std::string path;
    if(0==kDqcTracePath.size()){
        path=std::string (getcwd(buf, FILENAME_MAX)) + "/traces/"
            +m_name+"_sendrate.txt";
    }else{
        path=std::string(kDqcTracePath)+m_name+"_sendrate.txt";
    }
    m_sendRate.open(path.c_str(), std::fstream::out);
    if(m_sendRate.is_open()){
        m_sendRate<<"#time(s)\tpacing_rate(kbps)"<<std::endl;
    }
}
void DqcTrace::OpenRecvRateFile(){
    if(!(m_enable & E_DQC_RECV_RATE) || m_recvRate.is_open()) return;
    char buf[FILENAME_MAX];
    memset(buf,0,FILENAME_MAX);
    std::string path;
    if(0==kDqcTracePath.size()){
        path=std::string (getcwd(buf, FILENAME_MAX)) + "/traces/"
            +m_name+"_recvrate.txt";
    }else{
        path=std::string(kDqcTracePath)+m_name+"_recvrate.txt";
    }
    m_recvRate.open(path.c_str(), std::fstream::out);
    if(m_recvRate.is_open()){
        m_recvRate<<"#time(s)\tbandwidth_latest_recv_rate(kbps)"<<std::endl;
    }
}
void DqcTrace::OpenRecvRateRawFile(){
    if(!(m_enable & E_DQC_RECV_RATE_RAW) || m_recvRateRaw.is_open()) return;
    char buf[FILENAME_MAX];
    memset(buf,0,FILENAME_MAX);
    std::string path;
    if(0==kDqcTracePath.size()){
        path=std::string (getcwd(buf, FILENAME_MAX)) + "/traces/"
            +m_name+"_recvrate_raw.txt";
    }else{
        path=std::string(kDqcTracePath)+m_name+"_recvrate_raw.txt";
    }
    m_recvRateRaw.open(path.c_str(), std::fstream::out);
    if(m_recvRateRaw.is_open()){
        m_recvRateRaw<<"#time(s)\tdelivery_rate_sample(kbps)"<<std::endl;
    }
}
void DqcTrace::OpenQueueDelayFile(){
    if(!(m_enable & E_DQC_QUEUE_DELAY) || m_queueDelay.is_open()) return;
    char buf[FILENAME_MAX];
    memset(buf,0,FILENAME_MAX);
    std::string path;
    if(0==kDqcTracePath.size()){
        path=std::string (getcwd(buf, FILENAME_MAX)) + "/traces/"
            +m_name+"_qdelay.txt";
    }else{
        path=std::string(kDqcTracePath)+m_name+"_qdelay.txt";
    }
    m_queueDelay.open(path.c_str(), std::fstream::out);
    if(m_queueDelay.is_open()){
        m_queueDelay<<"#time(s)\tqueue_delay(ms)\tlatest_rtt(ms)\tmin_rtt(ms)"<<std::endl;
    }
}
void DqcTrace::OpenInflightFile(){
    if(!(m_enable & E_DQC_INFLIGHT) || m_inflight.is_open()) return;
    char buf[FILENAME_MAX];
    memset(buf,0,FILENAME_MAX);
    std::string path;
    if(0==kDqcTracePath.size()){
        path=std::string (getcwd(buf, FILENAME_MAX)) + "/traces/"
            +m_name+"_inflight.txt";
    }else{
        path=std::string(kDqcTracePath)+m_name+"_inflight.txt";
    }
    m_inflight.open(path.c_str(), std::fstream::out);
    if(m_inflight.is_open()){
        m_inflight<<"#time(s)\tinflight(bytes)\tcwnd(bytes)"<<std::endl;
    }
}
void DqcTrace::OpenBbrModeFile(){
    if(!(m_enable & E_DQC_BBR_MODE) || m_bbrMode.is_open()) return;
    char buf[FILENAME_MAX];
    memset(buf,0,FILENAME_MAX);
    std::string path;
    if(0==kDqcTracePath.size()){
        path=std::string (getcwd(buf, FILENAME_MAX)) + "/traces/"
            +m_name+"_bbrmode.txt";
    }else{
        path=std::string(kDqcTracePath)+m_name+"_bbrmode.txt";
    }
    m_bbrMode.open(path.c_str(), std::fstream::out);
    if(m_bbrMode.is_open()){
        m_bbrMode<<"#time(s)\tmode"<<std::endl;
    }
}
void DqcTrace::OpenLossRateFile(){
    if(!(m_enable & E_DQC_LOSS_RATE) || m_lossRate.is_open()) return;
    char buf[FILENAME_MAX];
    memset(buf,0,FILENAME_MAX);
    std::string path;
    if(0==kDqcTracePath.size()){
        path= std::string (getcwd(buf, FILENAME_MAX)) + "/traces/"
            +m_name+"_lossrate.txt";
    }else{
        path=std::string(kDqcTracePath)+m_name+"_lossrate.txt";
    }
    m_lossRate.open(path.c_str(), std::fstream::out);
    if(m_lossRate.is_open()){
        m_lossRate<<"#time(s)\tloss_rate(%)\tcum_loss_rate(%)"<<std::endl;
    }
}
void DqcTrace::OpenUpPhaseFile(){
    if(!(m_enable & E_DQC_UP_PHASE) || m_upPhase.is_open()) return;
    char buf[FILENAME_MAX];
    memset(buf,0,FILENAME_MAX);
    std::string path;
    if(0==kDqcTracePath.size()){
        path=std::string (getcwd(buf, FILENAME_MAX)) + "/traces/"
            +m_name+"_upphase.txt";
    }else{
        path=std::string(kDqcTracePath)+m_name+"_upphase.txt";
    }
    m_upPhase.open(path.c_str(), std::fstream::out);
    if(m_upPhase.is_open()){
        m_upPhase<<"#start_time(s)\tduration(ms)\tfreq(Hz)\t1.25BDP_exit\tcycles\tpacing_gain\tbw_estimate(kbps)"<<std::endl;
    }
}
void DqcTrace::OpenStatsFile(){
    if(!(m_enable & E_DQC_STAT) || m_stats.is_open()) return;
    char buf[FILENAME_MAX];
    memset(buf,0,FILENAME_MAX);
    std::string path;
    if(0==kDqcTracePath.size()){
        path = std::string (getcwd(buf, FILENAME_MAX)) + "/traces/"
            +m_name+"_stats.txt";
    }else{
        path=std::string(kDqcTracePath)+m_name+"_stats.txt";
    }
    m_stats.open(path.c_str(), std::fstream::out);
    if(m_stats.is_open()){
        m_stats<<"#loss_rate(%)\tavg_throughput(kbps)\tavg_owd(ms)\ttotal_recv_bytes"<<std::endl;
    }
}
void DqcTrace::OnOwd(uint32_t seq,uint32_t owd,uint32_t size){
    OpenOwdFile();  // Lazy open
    if(m_owd.is_open()){
        float now=Simulator::Now().GetSeconds();
        m_owd<<now<<"\t"<<seq<<"\t"<<owd<<"\t"<<size<<"\n";
    }    
}
void DqcTrace::OnRtt(uint32_t seq,uint32_t rtt,uint32_t smoothed_rtt){
    OpenRttFile();  // Lazy open
    if(m_rtt.is_open()){
        float now=Simulator::Now().GetSeconds();
        m_rtt<<now<<"\t"<<seq<<"\t"<<rtt<<"\t"<<smoothed_rtt<<"\n";
    }
}
void DqcTrace::OnBw(int32_t kbps){
    OpenBandwidthFile();  // Lazy open
    int64_t now_us=Simulator::Now().GetMicroSeconds();
    // Only record once per simulation timestamp
    if(now_us == m_lastBwTimeUs){
        return;
    }
    m_lastBwTimeUs = now_us;
    if(m_bw.is_open()){
        float now=Simulator::Now().GetSeconds();
        m_bw<<now<<"\t"<<kbps<<"\n";
    }

}
void DqcTrace::OnGoodput(uint32_t kbps){
    OpenGoodputFile();  // Lazy open
	if(m_googput.is_open()){
		float now=Simulator::Now().GetSeconds();
        m_googput<<now<<"\t"<<kbps<<"\n";
	}
}
void DqcTrace::OnSendRate(int32_t kbps){
    OpenSendRateFile();  // Lazy open
    if(m_sendRate.is_open()){
        float now=Simulator::Now().GetSeconds();
        m_sendRate<<now<<"\t"<<kbps<<"\n";
    }
}
void DqcTrace::OnRecvRate(int32_t bandwidth_latest_kbps){
    OpenRecvRateFile();  // Lazy open
    if(m_recvRate.is_open()){
        float now=Simulator::Now().GetSeconds();
        m_recvRate<<now<<"\t"<<bandwidth_latest_kbps<<"\n";
    }
}
void DqcTrace::OnRecvRateRaw(int32_t delivery_rate_kbps){
    OpenRecvRateRawFile();  // Lazy open
    if(m_recvRateRaw.is_open()){
        float now=Simulator::Now().GetSeconds();
        m_recvRateRaw<<now<<"\t"<<delivery_rate_kbps<<"\n";
    }
}
void DqcTrace::OnQueueDelay(uint32_t queue_delay_ms,uint32_t latest_rtt_ms,uint32_t min_rtt_ms){
    OpenQueueDelayFile();
    if(m_queueDelay.is_open()){
        float now=Simulator::Now().GetSeconds();
        m_queueDelay<<now<<"\t"<<queue_delay_ms<<"\t"<<latest_rtt_ms<<"\t"<<min_rtt_ms<<"\n";
    }
}
void DqcTrace::OnInflight(int32_t inflight_bytes,int32_t cwnd_bytes){
    OpenInflightFile();  // Lazy open
    if(m_inflight.is_open()){
        float now=Simulator::Now().GetSeconds();
        m_inflight<<now<<"\t"<<inflight_bytes<<"\t"<<cwnd_bytes<<"\n";
    }
}
void DqcTrace::OnBbrMode(int32_t mode){
    OpenBbrModeFile();  // Lazy open

    // Only record if mode changed from last recorded mode
    if(mode == m_lastBbrMode){
        return;  // Skip recording if mode hasn't changed
    }

    m_lastBbrMode = mode;  // Update last mode

    if(m_bbrMode.is_open()){
        float now=Simulator::Now().GetSeconds();
        // mode encoding:
        // 0: STARTUP
        // 1: DRAIN
        // 2: PROBE_BW_DOWN
        // 3: PROBE_BW_CRUISE
        // 4: PROBE_BW_REFILL
        // 5: PROBE_BW_UP
        // 6: PROBE_RTT
        const char* mode_names[] = {
            "start",
            "drain",
            "probeBW_down",
            "probeBW_cruise",
            "probeBW_refill",
            "probeBW_up",
            "probeRTT",
            "probeBW_pre_up",
            "probeBW_guard",
            "probeBW_post_up",
            "probeBW_down_slightly"
        };
        const char* mode_name = (mode >= 0 && mode <= 10) ? mode_names[mode] : "unknown";
        m_bbrMode<<now<<"\t"<<mode_name<<"\n";
    }
}
void DqcTrace::OnLossRate(double time_sec,float loss_rate,float cumulative_loss_rate){
    OpenLossRateFile();  // Lazy open
    if(m_lossRate.is_open()){
        m_lossRate<<time_sec<<"\t"<<loss_rate<<"\t"<<cumulative_loss_rate<<"\n";
    }
}
void DqcTrace::OnUpPhase(double start_time,double duration_ms,double freq_hz,bool exit_due_to_queueing,int cycles,float pacing_gain,int32_t bw_estimate_kbps){
    OpenUpPhaseFile();  // Lazy open
    if(m_upPhase.is_open()){
        m_upPhase<<start_time<<"\t"<<duration_ms<<"\t"<<freq_hz<<"\t"<<(exit_due_to_queueing?"true":"false")<<"\t"<<cycles<<"\t"<<pacing_gain<<"\t"<<bw_estimate_kbps<<"\n";
    }
}
void DqcTrace::OnFreqAnalysis(double start_time, double duration_sec, double sender_peak_freq_hz, double receiver_peak_freq_hz, int32_t avg_rate_kbps){
    OpenFreqAnalysisFile(); // Lazy open
    if(m_freqAnalysis.is_open()){
        m_freqAnalysis<<start_time<<"\t"<<duration_sec<<"\t"<<sender_peak_freq_hz<<"\t"<<receiver_peak_freq_hz<<"\t"<<avg_rate_kbps<<"\n";
    }
}
void DqcTrace::OnRttFreqAnalysis(double start_time, double duration_sec, double sender_peak_freq_hz, double rtt_peak_freq_hz, double avg_smoothed_rtt_ms){
    OpenRttFreqAnalysisFile(); // Lazy open
    if(m_rttFreqAnalysis.is_open()){
        m_rttFreqAnalysis<<start_time<<"\t"<<duration_sec<<"\t"<<sender_peak_freq_hz<<"\t"<<rtt_peak_freq_hz<<"\t"<<avg_smoothed_rtt_ms<<"\n";
    }
}
void DqcTrace::OnFBBRLoad(double window_start_s, double window_end_s,
                              double p_underload, double p_full_load,
                              double p_overload, double confidence,
                              std::string label, bool low_confidence,
                              std::string diagnostics){
    (void)window_start_s;
    (void)window_end_s;
    (void)p_underload;
    (void)p_full_load;
    (void)p_overload;
    (void)confidence;
    (void)low_confidence;
    if(label == "FREQ_GATE_TRACE"){
        OpenFBBRGateFile();
        if(m_fbbrGate.is_open()){
            m_fbbrGate<<diagnostics<<"\n";
        }
    }else if(label == "WAVEFORM_SEARCH"){
        OpenFBBRWaveformSearchFile();
        if(m_fbbrWaveformSearch.is_open()){
            m_fbbrWaveformSearch<<diagnostics<<"\n";
        }
    }else if(label == "CRUISE_SUMMARY"){
        OpenFBBRCruiseSummaryFile();
        if(m_fbbrCruiseSummary.is_open()){
            m_fbbrCruiseSummary<<diagnostics<<"\n";
        }
    }else if(label == "FBBR_FLOW_SUMMARY"){
        OpenFBBRSummaryFile();
        if(m_fbbrSummary.is_open()){
            m_fbbrSummary<<diagnostics<<"\n";
        }
    }else{
        OpenFBBRLoadFile();
        if(m_fbbrLoad.is_open()){
            m_fbbrLoad<<diagnostics<<"\n";
        }
    }
}
void DqcTrace::OnStats(uint64_t recv_count,uint64_t largest,
                        uint64_t recv_bytes,uint64_t duration,
                       float avg_owd){
    OpenStatsFile();  // Lazy open
	if(m_stats.is_open()){
        double loss_rate=10000.0-10000.0*recv_count/largest;
        m_stats<<(float)(loss_rate/100)<<std::endl;
        uint32_t avg_kbps=recv_bytes*8/duration;
        m_stats<<avg_kbps<<std::endl;
        m_stats<<avg_owd<<std::endl;
        m_stats<<recv_bytes<<std::endl;
        m_stats.flush();
    }
    if(!m_traceStatsCb.IsNull()){
        m_traceStatsCb(m_id,recv_count,largest,recv_bytes,avg_owd);
    }
}
void DqcTrace::Close(){
    CloseOwdFile();
    CloseRttFile();
    CloseBandwidthFile();
    CloseGoodputFile();
    CloseSendRateFile();
    CloseRecvRateFile();
    CloseRecvRateRawFile();
    CloseQueueDelayFile();
    CloseInflightFile();
    CloseBbrModeFile();
    CloseLossRateFile();
    CloseUpPhaseFile();
    CloseFreqAnalysisFile();
    CloseRttFreqAnalysisFile();
    CloseFBBRLoadFile();
    CloseFBBRCruiseSummaryFile();
    CloseFBBRGateFile();
    CloseFBBRWaveformSearchFile();
    CloseFBBRSummaryFile();
    CloseStatsFile();
}
void DqcTrace::CloseOwdFile(){
    if(m_owd.is_open()){
        m_owd.close();
    }    
}
void DqcTrace::CloseRttFile(){
    if(m_rtt.is_open()){
        m_rtt.close();
    }    
}
void DqcTrace::CloseBandwidthFile(){
    if(m_bw.is_open()){
        m_bw.close();
    }
} 
void DqcTrace::CloseGoodputFile(){
    if(m_googput.is_open()){
        m_googput.flush();
        m_googput.close();
    }
}
void DqcTrace::CloseSendRateFile(){
    if(m_sendRate.is_open()){
        m_sendRate.flush();
        m_sendRate.close();
    }
}
void DqcTrace::CloseRecvRateFile(){
    if(m_recvRate.is_open()){
        m_recvRate.flush();
        m_recvRate.close();
    }
}
void DqcTrace::CloseRecvRateRawFile(){
    if(m_recvRateRaw.is_open()){
        m_recvRateRaw.flush();
        m_recvRateRaw.close();
    }
}
void DqcTrace::CloseQueueDelayFile(){
    if(m_queueDelay.is_open()){
        m_queueDelay.flush();
        m_queueDelay.close();
    }
}
void DqcTrace::CloseInflightFile(){
    if(m_inflight.is_open()){
        m_inflight.flush();
        m_inflight.close();
    }
}
void DqcTrace::CloseBbrModeFile(){
    if(m_bbrMode.is_open()){
        m_bbrMode.flush();
        m_bbrMode.close();
    }
}
void DqcTrace::CloseLossRateFile(){
    if(m_lossRate.is_open()){
        m_lossRate.flush();
        m_lossRate.close();
    }
}
void DqcTrace::CloseUpPhaseFile(){
    if(m_upPhase.is_open()){
        m_upPhase.flush();
        m_upPhase.close();
    }
}
void DqcTrace::CloseFreqAnalysisFile(){
    if(m_freqAnalysis.is_open()){
        m_freqAnalysis.flush();
        m_freqAnalysis.close();
    }
}
void DqcTrace::CloseRttFreqAnalysisFile(){
    if(m_rttFreqAnalysis.is_open()){
        m_rttFreqAnalysis.flush();
        m_rttFreqAnalysis.close();
    }
}
void DqcTrace::CloseFBBRLoadFile(){
    if(m_fbbrLoad.is_open()){
        m_fbbrLoad.flush();
        m_fbbrLoad.close();
    }
}
void DqcTrace::CloseFBBRCruiseSummaryFile(){
    if(m_fbbrCruiseSummary.is_open()){
        m_fbbrCruiseSummary.flush();
        m_fbbrCruiseSummary.close();
    }
}
void DqcTrace::CloseFBBRGateFile(){
    if(m_fbbrGate.is_open()){
        m_fbbrGate.flush();
        m_fbbrGate.close();
    }
}
void DqcTrace::CloseFBBRWaveformSearchFile(){
    if(m_fbbrWaveformSearch.is_open()){
        m_fbbrWaveformSearch.flush();
        m_fbbrWaveformSearch.close();
    }
}
void DqcTrace::CloseFBBRSummaryFile(){
    if(m_fbbrSummary.is_open()){
        m_fbbrSummary.flush();
        m_fbbrSummary.close();
    }
}
void DqcTrace::CloseStatsFile(){
    if(m_stats.is_open()){
        m_stats.close();
    }
}
void DqcTrace::OpenFreqAnalysisFile(){
    if(!(m_enable & E_DQC_FREQ_ANALYSIS) || m_freqAnalysis.is_open()) return;
    char buf[FILENAME_MAX];
    memset(buf,0,FILENAME_MAX);
    std::string path;
    if(0==kDqcTracePath.size()){
        path=std::string (getcwd(buf, FILENAME_MAX)) + "/traces/"
            +m_name+"_recvfreq.txt";
    }else{
        path=std::string(kDqcTracePath)+m_name+"_recvfreq.txt";
    }
    m_freqAnalysis.open(path.c_str(), std::fstream::out);
    if(m_freqAnalysis.is_open()){
        m_freqAnalysis<<"#start_time(s)\tduration(s)\tsender_peak_freq(Hz)\treceiver_peak_freq(Hz)\tavg_rate(kbps)"<<std::endl;
    }
}
void DqcTrace::OpenRttFreqAnalysisFile(){
    if(!(m_enable & E_DQC_FREQ_ANALYSIS) || m_rttFreqAnalysis.is_open()) return;
    char buf[FILENAME_MAX];
    memset(buf,0,FILENAME_MAX);
    std::string path;
    if(0==kDqcTracePath.size()){
        path=std::string (getcwd(buf, FILENAME_MAX)) + "/traces/"
            +m_name+"_rttfreq.txt";
    }else{
        path=std::string(kDqcTracePath)+m_name+"_rttfreq.txt";
    }
    m_rttFreqAnalysis.open(path.c_str(), std::fstream::out);
    if(m_rttFreqAnalysis.is_open()){
        m_rttFreqAnalysis<<"#start_time(s)\tduration(s)\tsender_peak_freq(Hz)\trtt_peak_freq(Hz)\tavg_smoothed_rtt(ms)"<<std::endl;
    }
}
void DqcTrace::OpenFBBRLoadFile(){
    if(!(m_enable & E_DQC_FBBR_LOAD) || m_fbbrLoad.is_open()) return;
    char buf[FILENAME_MAX];
    memset(buf,0,FILENAME_MAX);
    std::string path;
    if(0==kDqcTracePath.size()){
        path=std::string (getcwd(buf, FILENAME_MAX)) + "/traces/"
            +m_name+"_cruise_full_load_quality.csv";
    }else{
        path=std::string(kDqcTracePath)+m_name+"_cruise_full_load_quality.csv";
    }
    m_fbbrLoad.open(path.c_str(), std::fstream::out);
    if(m_fbbrLoad.is_open()){
        m_fbbrLoad<<"cruise_id,window_start_time,window_end_time,configured_modulation_freq_hz"
                       <<",srate_peak_freq_hz,drate_peak_freq_hz,srtt_peak_freq_hz"
                       <<",drate_freq_score,srtt_freq_score,freq_quality"
                       <<",drate_target_amp,srate_target_amp,drate_gain,drate_amplitude_score"
                       <<",srtt_target_amp,srtt_noise_floor,srtt_snr,srtt_amplitude_score"
                       <<",drate_waveform_quality,srtt_waveform_quality,waveform_quality"
                       <<",cycle_frequency_stability,cycle_phase_stability,consistency_quality"
	                       <<",srtt_top_clip_ratio,srtt_bottom_clip_ratio,srtt_distortion_score"
	                       <<",is_full_load_candidate,full_load_quality,full_load_rank_in_cruise"
	                       <<",is_best_full_load_window,low_confidence,label"
	                       <<",full_load_quality_v1,full_load_quality_v2"
	                       <<",drate_spectral_integrity_score"
	                       <<",srtt_spectral_integrity_score"
	                       <<",joint_spectral_integrity_score"
	                       <<",drate_spectral_gate_pass"
	                       <<",srtt_spectral_gate_pass"
	                       <<",dual_signal_spectral_gate_pass"
	                       <<",limiting_spectral_signal"
	                       <<",spectral_invalid_reason,drate_snr"
	                       <<",drate_band_energy_rel,srtt_band_energy_rel"
	                       <<",drate_band_peak_rel,srtt_band_peak_rel"
	                       <<",srate_peak_width_hz,drate_peak_width_hz"
	                       <<",srtt_peak_width_hz,drate_width_ratio"
	                       <<",srtt_width_ratio,drate_phase_coherence"
	                       <<",srtt_phase_coherence,window_source"<<std::endl;
    }
}
void DqcTrace::OpenFBBRCruiseSummaryFile(){
    if(!(m_enable & E_DQC_FBBR_LOAD) || m_fbbrCruiseSummary.is_open()) return;
    char buf[FILENAME_MAX];
    memset(buf,0,FILENAME_MAX);
    std::string path;
    if(0==kDqcTracePath.size()){
        path=std::string (getcwd(buf, FILENAME_MAX)) + "/traces/"
            +m_name+"_cruise_best_full_load_window.csv";
    }else{
        path=std::string(kDqcTracePath)+m_name+"_cruise_best_full_load_window.csv";
    }
    m_fbbrCruiseSummary.open(path.c_str(), std::fstream::out);
    if(m_fbbrCruiseSummary.is_open()){
        m_fbbrCruiseSummary<<"cruise_id,cruise_start_time,cruise_end_time,candidate_count"
                                <<",best_full_load_window_exists,best_window_start_time,best_window_end_time"
                                <<",best_full_load_quality,best_drate_freq_score,best_srtt_freq_score"
	                                <<",best_srtt_waveform_quality,best_drate_amplitude_score,best_srtt_amplitude_score"
	                                <<",best_drate_mean_kbps"
	                                <<",cruise_end_native_bw_kbps,fair_share_bandwidth_kbps"
	                                <<",best_beq,best_beq_source"
	                                <<",best_full_load_quality_v1,best_full_load_quality_v2"
	                                <<",drate_spectral_integrity_score"
	                                <<",srtt_spectral_integrity_score"
	                                <<",joint_spectral_integrity_score"
	                                <<",drate_spectral_gate_pass"
	                                <<",srtt_spectral_gate_pass"
	                                <<",dual_signal_spectral_gate_pass"
	                                <<",limiting_spectral_signal"
	                                <<",best_spectral_invalid_reason"
	                                <<",selection_native_bw_bps,beq_valid"
	                                <<",beq_cruise_id,beq_fresh"
	                                <<",beq_application_valid"
	                                <<",detector_mode,waveform_final_state"
	                                <<",waveform_decision_count"
	                                <<",waveform_baseline_adjustments"
	                                <<",waveform_amplitude_reductions"
	                                <<",waveform_underload_located"
		                                <<",waveform_beq_source"
		                                <<",fbbr_max_bw_valid"
		                                <<",fbbr_max_bw_bps"
		                                <<",fbbr_srtt_low_rtprop_valid"
		                                <<",fbbr_srtt_low_rtprop_ms"
		                                <<",fbbr_max_srtt_valid"
		                                <<",fbbr_max_srtt_ms"
		                                <<std::endl;
    }
}
void DqcTrace::OpenFBBRWaveformSearchFile(){
    if(!(m_enable & E_DQC_FBBR_LOAD) ||
       m_fbbrWaveformSearch.is_open()) return;
    char buf[FILENAME_MAX];
    memset(buf,0,FILENAME_MAX);
    std::string path;
    if(0==kDqcTracePath.size()){
        path=std::string (getcwd(buf, FILENAME_MAX)) + "/traces/flow"
            +std::to_string(m_id)+"_cruise_waveform_search.csv";
    }else{
        path=std::string(kDqcTracePath)+"flow"+std::to_string(m_id)
            +"_cruise_waveform_search.csv";
    }
    m_fbbrWaveformSearch.open(path.c_str(), std::fstream::out);
    if(m_fbbrWaveformSearch.is_open()){
        m_fbbrWaveformSearch
            <<"time_s,cruise_id,decision_id,waveform_state,detector_mode"
            <<",cruise_cwnd_gain"
            <<",probe_epoch_start_s,probe_epoch_rtt_s"
            <<",response_window_start_s,response_window_end_s"
            <<",negative_half_first,window_periods,extended_window"
            <<",analysis_cycle_start_s,analysis_cycle_end_s"
            <<",analysis_cycle_periods,analysis_uses_later_cycle"
            <<",prior_cycle_srtt_input_valid,prior_cycle_srtt_similar"
            <<",prior_cycle_drate_input_valid,prior_cycle_drate_similar"
            <<",prior_cycle_classification"
            <<",sender_sample_count,drate_sample_count,srtt_sample_count"
            <<",srtt_stat_sample_count,srtt_stats_valid"
            <<",srtt_window_mean_ms,srtt_window_min_ms,srtt_window_max_ms"
            <<",coverage_ratio,app_limited_ratio,sender_waveform_valid"
            <<",goertzel_target_frequency_hz"
            <<",sender_goertzel_input_valid,sender_goertzel_component_present"
            <<",sender_goertzel_real,sender_goertzel_imag,sender_goertzel_phase_rad"
            <<",sender_goertzel_power,sender_goertzel_amplitude"
            <<",sender_goertzel_coherent_power_ratio,sender_goertzel_reason"
            <<",drate_goertzel_input_valid,drate_goertzel_component_present"
            <<",drate_goertzel_real,drate_goertzel_imag,drate_goertzel_phase_rad"
            <<",drate_goertzel_power,drate_goertzel_amplitude"
            <<",drate_goertzel_coherent_power_ratio,drate_goertzel_reason"
            <<",goertzel_component_match"
            <<",best_lag_s"
            <<",srtt_input_valid,srtt_similar_frequency,srtt_similar"
            <<",srtt_similar_without_middle,srtt_effective_similar"
            <<",srtt_masked_period_s,srtt_masked_periodicity_correlation"
            <<",srtt_cycle_complete,srtt_positive_half_clipped"
            <<",srtt_negative_half_clipped"
            <<",srtt_only_negative_half,srtt_only_positive_half"
            <<",srtt_positive_half_span_ms,srtt_negative_half_span_ms"
            <<",srtt_clip_ambiguous"
            <<",bic_srtt_shape_valid,bic_srtt_top_clip"
            <<",bic_srtt_bottom_clip,bic_srtt_both_clipped"
            <<",bic_srtt_top_motif_count,bic_srtt_bottom_motif_count"
            <<",bic_srtt_selected_segment_count,bic_srtt_selected_score"
            <<",bic_srtt_top_min_rounded_bic_margin"
            <<",bic_srtt_bottom_min_rounded_bic_margin"
            <<",bic_srtt_top_combined_rounded_bic_margin"
            <<",bic_srtt_bottom_combined_rounded_bic_margin"
            <<",bic_srtt_top_pair_sharp_motif_count"
            <<",bic_srtt_bottom_pair_sharp_motif_count"
            <<",true_bottom_clip_rtprop_refresh_applied"
            <<",true_bottom_clip_rtprop_before_ms"
            <<",true_bottom_clip_rtprop_after_ms"
            <<",true_bottom_clip_min_rtt_timestamp_before_s"
            <<",true_bottom_clip_min_rtt_timestamp_after_s"
            <<",true_bottom_clip_probe_rtt_deadline_after_s"
            <<",bic_srtt_invalid_reason"
            <<",srtt_direct_ncc,srtt_integral_ncc,srtt_derivative_ncc"
            <<",srtt_slope_direction_agreement,srtt_period_s"
            <<",srtt_period_error_ratio,srtt_periodicity_correlation"
            <<",srtt_fitted_amplitude"
            <<",srtt_noise_sigma,srtt_response_snr"
            <<",srtt_completeness_score"
            <<",drate_input_valid,drate_similar"
            <<",drate_similar_without_middle,drate_effective_similar"
            <<",drate_masked_period_s,drate_masked_periodicity_correlation"
            <<",drate_ncc"
            <<",drate_slope_direction_agreement,drate_period_s"
            <<",drate_period_error_ratio,drate_periodicity_correlation"
            <<",current_drate_response_amplitude_bps,drate_noise_sigma"
            <<",drate_response_snr,drate_completeness_score"
            <<",has_last_similar_drate_amplitude"
            <<",last_similar_drate_amplitude_bps"
            <<",delta_source,raw_delta_bw_bps,applied_delta_bw_bps"
            <<",delta_reference_bps,window_extreme_gap_bps"
            <<",actuator_step_multiplier,queue_delay_ms"
            <<",queue_delay_min_rtt_ratio,overload_confirmation_count"
            <<",underload_located,classification,action"
            <<",baseline_before_bps"
            <<",baseline_after_bps,amplitude_before_bps"
            <<",amplitude_after_bps,maxbw_attenuation_factor"
            <<",maxbw_actual_fluctuation_amplitude_bps"
            <<",maxbw_delivery_response_gain,maxbw_response_observed"
            <<",beq_baseline_locked"
            <<",beq_candidate_update_count"
            <<",search_continues_after_full_load"
            <<",beq_candidate_bps,beq_candidate_source"
            <<",invalid_reason,decision_rule"
            <<",delivery_rate_stat_sample_count"
            <<",delivery_rate_stats_valid"
            <<",delivery_rate_window_min_bps"
            <<",delivery_rate_window_max_bps"
            <<",delivery_rate_window_mean_bps"
            <<",latest_beq_bps,smoothed_beq_bps"
            <<",drate_positive_half_clipped,drate_negative_half_clipped"
            <<",drate_only_negative_half"
            <<",drate_positive_half_span_bps,drate_negative_half_span_bps"
            <<",positive_half_clips_simultaneous"
            <<",srtt_middle_sequential_plateau"
            <<",drate_middle_sequential_plateau,drate_middle_any_plateau"
            <<",drate_has_waveform,plateau_candidate_count"
            <<",middle_sequential_candidate_count"
            <<",top_clip,bottom_clip,clip_shoulders_opposite"
            <<",clip_shoulder_slope_before,clip_shoulder_slope_after"
            <<",other_shoulder_slope_before,other_shoulder_slope_after"
            <<",clip_shoulder_change_before,clip_shoulder_change_after"
            <<",clip_minimum_shoulder_change"
            <<",clip_duration_ratio,clip_level_span_ratio"
            <<",clip_half_overlap_ratio,clip_extreme_distance_ratio"
            <<",boundary_lift_time_s"
            <<",boundary_delta_bps,amplitude_reduction"
            <<",clip_floor_confirmation"
            <<",algorithm_mode,regime_pipeline_owner,regime_actuator_owner"
            <<",regime_rule_id,unsuppressed_regime"
            <<",srtt_suspected_top_candidate,srtt_suspected_bottom_candidate"
            <<",srtt_u1_positive_shoulder,srtt_u2_long_top_line,srtt_u3_repeated_top_clip"
            <<",srtt_l1_negative_shoulder,srtt_l2_long_bottom_line,srtt_l3_repeated_bottom_clip"
            <<",srtt_selected_clip_case,both_clip_directions"
            <<",clip_candidate_rejected_to_wave_fallback,fallback_entered"
            <<",srtt_upper_clip_periodic_veto,drate_upper_clip_periodic_veto"
            <<",srtt_lower_clip_ignored_for_periodic,drate_lower_clip_ignored_for_periodic"
            <<",srtt_wave_input_view,drate_wave_input_view"
            <<",srtt_top_contact_fragment_count,srtt_bottom_contact_fragment_count"
            <<",srtt_top_contact_sample_count,srtt_bottom_contact_sample_count"
            <<",srtt_top_contact_cycle_mask,srtt_bottom_contact_cycle_mask"
            <<",srtt_top_contact_span_ratio_of_window,srtt_bottom_contact_span_ratio_of_window"
            <<",srtt_top_contact_pooled_flat_fraction,srtt_bottom_contact_pooled_flat_fraction"
            <<",srtt_top_contact_boundary_verified_fraction,srtt_bottom_contact_boundary_verified_fraction"
            <<",srtt_top_contact_extrapolated_overshoot_ratio,srtt_bottom_contact_extrapolated_overshoot_ratio"
            <<",drate_top_contact_fragment_count,drate_bottom_contact_fragment_count"
            <<",drate_top_contact_sample_count,drate_bottom_contact_sample_count"
            <<",drate_top_contact_cycle_mask,drate_bottom_contact_cycle_mask"
            <<",drate_top_contact_span_ratio_of_window,drate_bottom_contact_span_ratio_of_window"
            <<",srtt_long_top_line_ratio,srtt_long_bottom_line_ratio"
            <<",srtt_positive_shoulder_cycle_input_valid,srtt_negative_shoulder_cycle_input_valid"
            <<",srtt_positive_shoulder_cycle_recognizable,srtt_negative_shoulder_cycle_recognizable"
            <<",srtt_continuous_horizontal_count,drate_continuous_horizontal_count"
            <<",srtt_middle_mask_ratio,drate_middle_mask_ratio"
            <<",srtt_middle_slope_mismatch_ratio,drate_middle_slope_mismatch_ratio"
            <<",srtt_middle_bridge_deviation_ratio,drate_middle_bridge_deviation_ratio"
            <<",srtt_has_wave,drate_has_wave"
            <<",srtt_wave_failure_reason,drate_wave_failure_reason"
            <<",srtt_wave_amplitude,drate_wave_amplitude"
            <<",srtt_wave_noise_sigma,drate_wave_noise_sigma"
            <<",srtt_wave_step_threshold,drate_wave_step_threshold"
            <<",srtt_wave_active_step_ratio,drate_wave_active_step_ratio"
            <<",srtt_wave_up_change_ratio,srtt_wave_down_change_ratio"
            <<",drate_wave_up_change_ratio,drate_wave_down_change_ratio"
            <<",srtt_wave_significant_path_ratio,drate_wave_significant_path_ratio"
            <<",srtt_wave_slope_reversals,drate_wave_slope_reversals"
            <<",srtt_wave_active_cycle_mask,drate_wave_active_cycle_mask"
            <<",srtt_periodic_input_valid,drate_periodic_input_valid"
            <<",srtt_periodic_similar,drate_periodic_similar"
            <<",estimated_srate_period_s,srtt_estimated_period_s,drate_estimated_period_s"
            <<",srtt_srate_period_error_ratio,drate_srate_period_error_ratio"
            <<",srtt_edge_mask_ratio,drate_edge_mask_ratio"
            <<",fbbr_inflight_bdp_valid,fbbr_inflight_bytes,fbbr_bdp_bytes"
            <<",max_srtt_valid,max_srtt_before_ms,max_srtt_after_ms"
            <<",fbbr_max_bw_before_valid,fbbr_max_bw_before_bps"
            <<",fbbr_max_bw_after_valid,fbbr_max_bw_after_bps"
            <<",fbbr_rtprop_valid,fbbr_rtprop_ms"
            <<",fbbr_max_srtt_valid,fbbr_max_srtt_ms"
            <<",fbbr_regime_i_maxdrate_triggered"
            <<",fbbr_regime_i_maxbw_midpoint_valid"
            <<",fbbr_regime_i_maxbw_midpoint_bps"
            <<",fbbr_regime_i_maxbw_midpoint_triggered"
            <<",fbbr_regime_i_growth_triggered"
            <<",window_first_cycle_id,window_second_cycle_id"
            <<",srtt_no_wave_streak,drate_no_wave_streak"
            <<",wave_fidelity_enhancement_active,wave_fidelity_just_entered"
            <<",retry_reason_mask,no_wave_triggered"
            <<",classification_suppressed_for_retry,state_updates_suppressed_for_retry"
            <<",retry_window_advance_periods,retry_window_stride_cycles"
            <<",inconclusive_extension_count,inconclusive_amplification_count"
            <<",initial_probe_amplitude_bps,current_probe_amplitude_bps"
            <<",inconclusive_amplitude_cap_bps,rolling_retry_count"
            <<",fbbr_previous_beq,fbbr_previous_beq_source"
            <<",fbbr_plan_inflight,fbbr_service_inflight"
            <<",fbbr_positive_probe_credit,fbbr_service_budget"
            <<",fbbr_envelope,fbbr_extra_acked,fbbr_inflight_cap"
            <<",fbbr_native_cwnd,fbbr_actual_inflight"
            <<",fbbr_service_history_valid"
            <<",fbbr_app_limited_contaminated"
            <<",fbbr_projection_active,fbbr_service_restriction"
            <<",fbbr_enforced_excess,fbbr_cap_binding_fraction\n";
    }
}
void DqcTrace::OpenFBBRSummaryFile(){
    if(!(m_enable & E_DQC_FBBR_LOAD) || m_fbbrSummary.is_open()) return;
    char buf[FILENAME_MAX];
    memset(buf,0,FILENAME_MAX);
    std::string path;
    if(0==kDqcTracePath.size()){
        path=std::string (getcwd(buf, FILENAME_MAX)) + "/traces/flow"
            +std::to_string(m_id)+"_fbbr_service_envelope_summary.csv";
    }else{
        path=std::string(kDqcTracePath)+"flow"+std::to_string(m_id)
            +"_fbbr_service_envelope_summary.csv";
    }
    m_fbbrSummary.open(path.c_str(), std::fstream::out);
    if(m_fbbrSummary.is_open()){
        m_fbbrSummary
            <<"algorithm,flow_id,previous_beq_time_ratio"
            <<",previous_beq_guard_source_time_ratio"
            <<",previous_beq_invalid_time_ratio"
            <<",projection_active_time_ratio"
            <<",service_history_valid_time_ratio"
            <<",app_limited_fallback_time_ratio"
            <<",plan_only_fallback_time_ratio"
            <<",service_limited_time_ratio,cap_binding_time_ratio"
            <<",mean_plan_inflight,p95_plan_inflight"
            <<",mean_service_inflight,p95_service_inflight"
            <<",mean_probe_credit,p95_probe_credit"
            <<",mean_extra_acked,p95_extra_acked"
            <<",mean_service_restriction,p95_service_restriction"
            <<",mean_enforced_excess,p95_enforced_excess\n";
    }
}
void DqcTrace::OpenFBBRGateFile(){
    if(!(m_enable & E_DQC_FBBR_GATE) || m_fbbrGate.is_open()) return;
    char buf[FILENAME_MAX];
    memset(buf,0,FILENAME_MAX);
    std::string path;
    if(0==kDqcTracePath.size()){
        path=std::string (getcwd(buf, FILENAME_MAX)) + "/traces/flow"
            +std::to_string(m_id)+"_freq_gate_trace.csv";
    }else{
        path=std::string(kDqcTracePath)+"flow"+std::to_string(m_id)+"_freq_gate_trace.csv";
    }
    m_fbbrGate.open(path.c_str(), std::fstream::out);
    if(m_fbbrGate.is_open()){
	        m_fbbrGate<<"time,flow_id,row_type,round_id,bbr_state,probe_bw_phase,is_cruise"
	                       <<",d_round,d_prev,d_round_valid,d_prev_valid,v_round,prev_v_round"
	                       <<",full_drate_ref,stable_cnt,bbr_stable,just_exited"
	                       <<",freq_tool_needed,freq_tool_on,w_freq"
	                       <<",unstable_episode_id,unstable_episode_active"
	                       <<",selection_native_bw_bps,current_native_bw_bps"
	                       <<",beq_bps,beq_valid,beq_conf"
	                       <<",beq_source,beq_cruise_id,beq_fresh"
	                       <<",beq_application_valid"
	                       <<",beq_ready_for_post_cruise"
	                       <<",beq_application_phase,beq_invalid_reason"
	                       <<",drate_spectral_integrity_score"
	                       <<",srtt_spectral_integrity_score"
	                       <<",joint_spectral_integrity_score"
	                       <<",drate_spectral_gate_pass,srtt_spectral_gate_pass"
	                       <<",dual_signal_spectral_gate_pass,limiting_spectral_signal"
	                       <<",pacing_base_bw_bps,pacing_base_source,phase_pacing_gain"
	                       <<",native_pacing_bps,final_pacing_rate_bps"
	                       <<",amplitude_bps,amplitude_bps_eff,triangle_wave"
	                       <<",current_delivery_rate,maxbw_attenuation_factor"
	                       <<",maxbw_filter_input_bps"
	                       <<",minbw_correction_factor"
	                       <<",minbw_filter_input_bps,minbw_bps"
	                       <<",maxbw_actual_fluctuation_amplitude_bps"
	                       <<",maxbw_delivery_response_gain"
	                       <<",maxbw_response_observed"
	                       <<",sample_is_app_limited,sample_valid"
	                       <<",merged_rescue_attempted,merged_rescue_success"
	                       <<",beq_selection_compute_us"
	                       <<",normal_window_count,merged_window_count"
	                       <<",spectral_invalid_count"
	                       <<",beq_cleared_on_cruise_start"<<std::endl;
    }
}
DqcTraceState::DqcTraceState(std::string name){
    char buf[FILENAME_MAX];
    memset(buf,0,FILENAME_MAX);
    std::string path;
    if(0==kDqcTracePath.size()){
        path=std::string (getcwd(buf, FILENAME_MAX)) + "/traces/"
            +name+"_all_stats.txt";
    }else{
        path=std::string(kDqcTracePath)+name+"_all_stats.txt";
    }
    m_stats.open(path.c_str(), std::fstream::out);     
}
DqcTraceState::~DqcTraceState(){
    if(m_stats.is_open()){
        m_stats.close();
    }
}
void DqcTraceState::OnStats(uint32_t id,uint64_t recv_count,uint64_t largest,
                 uint64_t recv_bytes,float avg_owd){
    m_recvCount+=recv_count;
    m_totalRecv+=largest;
    m_totalRecvBytes+=recv_bytes;
    if(!m_ccType1Ids.empty()){
        auto it=m_ccType1Ids.find(id);
        if(it!=m_ccType1Ids.end()){
            m_ccType1TotalRecvBytes+=recv_bytes;
        }else{
            m_ccType2TotalRecvBytes+=recv_bytes;
        }
    }
    if(m_delayIds.empty()){
        return;
    }
    auto it=m_delayIds.find(id);
    if(it!=m_delayIds.end()){
        m_delayCount+=recv_count;
        m_sumDelay+=(avg_owd*recv_count);
    }
}
void DqcTraceState::Flush(uint32_t capacity,double simulation_time){
    m_count++;
    double average_rate=simulation_time > 0.0
        ? 1.0*m_totalRecvBytes*8/simulation_time
        : 0.0;
    double util=(average_rate/capacity);
    double loss=m_totalRecv > 0 ? 10000.0-10000.0*m_recvCount/m_totalRecv : 0.0;
    float loss_rate=loss/100;
    double delay=0.0;
    double ratio=0.0;
    if(m_ccType2TotalRecvBytes>0){
        ratio=1.0*m_ccType1TotalRecvBytes/m_ccType2TotalRecvBytes;
    }
    if(!m_delayIds.empty()&&m_delayCount>0){
        delay=1.0*m_sumDelay/m_delayCount;
    }
    if(m_stats.is_open()){
        m_stats<<m_count<<"\t"<<(float)loss_rate<<"\t"
        <<(float)average_rate<<"\t"<<(float)delay<<"\t"
        <<(float)util<<"\t"<<(float)ratio<<"\t"
        <<std::endl; 
    }
    Reset();
}
void DqcTraceState::RecordRuningTime(float millis,float mimutes){
    if(m_stats.is_open()){
        m_stats<<millis<<"\t"<<mimutes<<std::endl;
    }
}
void DqcTraceState::ReisterAvgDelayId(uint32_t id){
    m_delayIds.insert(id);
}
void DqcTraceState::RegisterCongestionType(uint32_t id,uint32_t type){
    if(!type){
        m_ccType1Ids.insert(id);
    }
}
void DqcTraceState::Reset(){
    m_recvCount=0;
    m_totalRecv=0;
    m_totalRecvBytes=0;
    m_delayCount=0;
    m_sumDelay=0;
    m_delayIds.clear();
}
}
