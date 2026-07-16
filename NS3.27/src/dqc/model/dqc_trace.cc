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
    if(m_enable & E_DQC_FREQCCV4_LOAD){
        OpenFreqCCv4WaveformSearchFile();
    }
    // Validation consumers require schema-bearing files even when a flow has
    // no eligible block (for example the forced-underload control).
    if(m_enable & E_DQC_FBBR_LOAD){
        OpenFbbrTriggerCycleFile();
        OpenFbbrBinFile();
        OpenFbbrEventWindowFile();
        OpenFbbrDiagnosticWindowFile();
        OpenFbbrCruiseFile();
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
void DqcTrace::OnFreqCCv4Load(double window_start_s, double window_end_s,
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
        OpenFreqCCv4GateFile();
        if(m_freqccv4Gate.is_open()){
            m_freqccv4Gate<<diagnostics<<"\n";
        }
    }else if(label == "WAVEFORM_SEARCH"){
        OpenFreqCCv4WaveformSearchFile();
        if(m_freqccv4WaveformSearch.is_open()){
            m_freqccv4WaveformSearch<<diagnostics<<"\n";
        }
    }else if(label == "CRUISE_SUMMARY"){
        OpenFreqCCv4CruiseSummaryFile();
        if(m_freqccv4CruiseSummary.is_open()){
            m_freqccv4CruiseSummary<<diagnostics<<"\n";
        }
    }else{
        OpenFreqCCv4LoadFile();
        if(m_freqccv4Load.is_open()){
            m_freqccv4Load<<diagnostics<<"\n";
        }
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
    if(label == "FBBR_TRIGGER_CYCLE"){
        OpenFbbrTriggerCycleFile();
        if(m_fbbrTriggerCycle.is_open()) m_fbbrTriggerCycle<<diagnostics<<"\n";
    }else if(label == "FBBR_BIN"){
        OpenFbbrBinFile();
        if(m_fbbrBin.is_open()) m_fbbrBin<<diagnostics<<"\n";
    }else if(label == "FBBR_EVENT_WINDOW"){
        OpenFbbrEventWindowFile();
        if(m_fbbrEventWindow.is_open()) m_fbbrEventWindow<<diagnostics<<"\n";
    }else if(label == "FBBR_DIAGNOSTIC_WINDOW"){
        OpenFbbrDiagnosticWindowFile();
        if(m_fbbrDiagnosticWindow.is_open()){
            m_fbbrDiagnosticWindow<<diagnostics<<"\n";
        }
    }else if(label == "FBBR_CRUISE"){
        OpenFbbrCruiseFile();
        if(m_fbbrCruise.is_open()) m_fbbrCruise<<diagnostics<<"\n";
    }else if(label == "FBBR_QUEUE_SERVO"){
        OpenFbbrQueueServoFile();
        if(m_fbbrQueueServo.is_open()) m_fbbrQueueServo<<diagnostics<<"\n";
    }else if(label == "FREQ_GATE_TRACE"){
        OpenFbbrGateFile();
        if(m_fbbrGate.is_open()) m_fbbrGate<<diagnostics<<"\n";
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
    CloseFreqCCv4LoadFile();
    CloseFreqCCv4CruiseSummaryFile();
    CloseFreqCCv4GateFile();
    CloseFreqCCv4WaveformSearchFile();
    CloseFbbrGateFile();
    CloseFbbrTriggerCycleFile();
    CloseFbbrBinFile();
    CloseFbbrEventWindowFile();
    CloseFbbrCruiseFile();
    CloseFbbrDiagnosticWindowFile();
    CloseFbbrQueueServoFile();
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
void DqcTrace::CloseFreqCCv4LoadFile(){
    if(m_freqccv4Load.is_open()){
        m_freqccv4Load.flush();
        m_freqccv4Load.close();
    }
}
void DqcTrace::CloseFreqCCv4CruiseSummaryFile(){
    if(m_freqccv4CruiseSummary.is_open()){
        m_freqccv4CruiseSummary.flush();
        m_freqccv4CruiseSummary.close();
    }
}
void DqcTrace::CloseFreqCCv4GateFile(){
    if(m_freqccv4Gate.is_open()){
        m_freqccv4Gate.flush();
        m_freqccv4Gate.close();
    }
}
void DqcTrace::CloseFreqCCv4WaveformSearchFile(){
    if(m_freqccv4WaveformSearch.is_open()){
        m_freqccv4WaveformSearch.flush();
        m_freqccv4WaveformSearch.close();
    }
}
void DqcTrace::CloseFbbrGateFile(){
    if(m_fbbrGate.is_open()){
        m_fbbrGate.flush();
        m_fbbrGate.close();
    }
}
void DqcTrace::CloseFbbrTriggerCycleFile(){
    if(m_fbbrTriggerCycle.is_open()){
        m_fbbrTriggerCycle.flush();
        m_fbbrTriggerCycle.close();
    }
}
void DqcTrace::CloseFbbrBinFile(){
    if(m_fbbrBin.is_open()){
        m_fbbrBin.flush();
        m_fbbrBin.close();
    }
}
void DqcTrace::CloseFbbrEventWindowFile(){
    if(m_fbbrEventWindow.is_open()){
        m_fbbrEventWindow.flush();
        m_fbbrEventWindow.close();
    }
}
void DqcTrace::CloseFbbrCruiseFile(){
    if(m_fbbrCruise.is_open()){
        m_fbbrCruise.flush();
        m_fbbrCruise.close();
    }
}
void DqcTrace::CloseFbbrDiagnosticWindowFile(){
    if(m_fbbrDiagnosticWindow.is_open()){
        m_fbbrDiagnosticWindow.flush();
        m_fbbrDiagnosticWindow.close();
    }
}
void DqcTrace::CloseFbbrQueueServoFile(){
    if(m_fbbrQueueServo.is_open()){
        m_fbbrQueueServo.flush();
        m_fbbrQueueServo.close();
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
void DqcTrace::OpenFreqCCv4LoadFile(){
    if(!(m_enable & E_DQC_FREQCCV4_LOAD) || m_freqccv4Load.is_open()) return;
    char buf[FILENAME_MAX];
    memset(buf,0,FILENAME_MAX);
    std::string path;
    if(0==kDqcTracePath.size()){
        path=std::string (getcwd(buf, FILENAME_MAX)) + "/traces/"
            +m_name+"_cruise_full_load_quality.csv";
    }else{
        path=std::string(kDqcTracePath)+m_name+"_cruise_full_load_quality.csv";
    }
    m_freqccv4Load.open(path.c_str(), std::fstream::out);
    if(m_freqccv4Load.is_open()){
        m_freqccv4Load<<"cruise_id,window_start_time,window_end_time,configured_modulation_freq_hz"
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
void DqcTrace::OpenFreqCCv4CruiseSummaryFile(){
    if(!(m_enable & E_DQC_FREQCCV4_LOAD) || m_freqccv4CruiseSummary.is_open()) return;
    char buf[FILENAME_MAX];
    memset(buf,0,FILENAME_MAX);
    std::string path;
    if(0==kDqcTracePath.size()){
        path=std::string (getcwd(buf, FILENAME_MAX)) + "/traces/"
            +m_name+"_cruise_best_full_load_window.csv";
    }else{
        path=std::string(kDqcTracePath)+m_name+"_cruise_best_full_load_window.csv";
    }
    m_freqccv4CruiseSummary.open(path.c_str(), std::fstream::out);
    if(m_freqccv4CruiseSummary.is_open()){
        m_freqccv4CruiseSummary<<"cruise_id,cruise_start_time,cruise_end_time,candidate_count"
                                <<",best_full_load_window_exists,best_window_start_time,best_window_end_time"
                                <<",best_full_load_quality,best_drate_freq_score,best_srtt_freq_score"
	                                <<",best_srtt_waveform_quality,best_drate_amplitude_score,best_srtt_amplitude_score"
	                                <<",best_drate_mean_kbps"
	                                <<",cruise_end_native_bw_kbps,fair_share_bandwidth_kbps"
	                                <<",best_trusted_bw,best_trusted_bw_source"
	                                <<",best_full_load_quality_v1,best_full_load_quality_v2"
	                                <<",drate_spectral_integrity_score"
	                                <<",srtt_spectral_integrity_score"
	                                <<",joint_spectral_integrity_score"
	                                <<",drate_spectral_gate_pass"
	                                <<",srtt_spectral_gate_pass"
	                                <<",dual_signal_spectral_gate_pass"
	                                <<",limiting_spectral_signal"
	                                <<",best_spectral_invalid_reason"
	                                <<",selection_native_bw_bps,trusted_bw_valid"
	                                <<",trusted_bw_cruise_id,trusted_bw_fresh"
	                                <<",trusted_bw_application_valid"
	                                <<",detector_mode,waveform_final_state"
	                                <<",waveform_decision_count"
	                                <<",waveform_baseline_adjustments"
	                                <<",waveform_amplitude_reductions"
	                                <<",waveform_underload_located"
	                                <<",waveform_trusted_source"
	                                <<std::endl;
    }
}
void DqcTrace::OpenFreqCCv4WaveformSearchFile(){
    if(!(m_enable & E_DQC_FREQCCV4_LOAD) ||
       m_freqccv4WaveformSearch.is_open()) return;
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
    m_freqccv4WaveformSearch.open(path.c_str(), std::fstream::out);
    if(m_freqccv4WaveformSearch.is_open()){
        m_freqccv4WaveformSearch
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
            <<",srtt_window_mean_ms,srtt_window_max_ms"
            <<",latest_waveform_overload_srtt_mean_valid"
            <<",latest_waveform_overload_srtt_mean_ms"
            <<",coverage_ratio,app_limited_ratio,sender_waveform_valid"
            <<",best_lag_s"
            <<",srtt_input_valid,srtt_similar_frequency,srtt_similar"
            <<",srtt_similar_without_middle,srtt_effective_similar"
            <<",srtt_masked_period_s,srtt_masked_periodicity_correlation"
            <<",srtt_cycle_complete,srtt_positive_half_clipped"
            <<",srtt_negative_half_clipped,srtt_clip_ambiguous"
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
            <<",amplitude_after_bps,trusted_baseline_locked"
            <<",trusted_bw_candidate_update_count"
            <<",search_continues_after_full_load"
            <<",trusted_bw_candidate_bps,trusted_bw_candidate_source"
            <<",invalid_reason,decision_rule"
            <<",delivery_rate_stat_sample_count"
            <<",delivery_rate_stats_valid"
            <<",delivery_rate_window_min_bps"
            <<",delivery_rate_window_max_bps"
            <<",delivery_rate_window_mean_bps"
            <<",latest_trusted_bw_bps,smoothed_trusted_bw_bps"
            <<",drate_positive_half_clipped,drate_negative_half_clipped"
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
            <<",clip_floor_confirmation\n";
    }
}
void DqcTrace::OpenFreqCCv4GateFile(){
    if(!(m_enable & E_DQC_FREQCCV4_GATE) || m_freqccv4Gate.is_open()) return;
    char buf[FILENAME_MAX];
    memset(buf,0,FILENAME_MAX);
    std::string path;
    if(0==kDqcTracePath.size()){
        path=std::string (getcwd(buf, FILENAME_MAX)) + "/traces/flow"
            +std::to_string(m_id)+"_freq_gate_trace.csv";
    }else{
        path=std::string(kDqcTracePath)+"flow"+std::to_string(m_id)+"_freq_gate_trace.csv";
    }
    m_freqccv4Gate.open(path.c_str(), std::fstream::out);
    if(m_freqccv4Gate.is_open()){
	        m_freqccv4Gate<<"time,flow_id,row_type,round_id,bbr_state,probe_bw_phase,is_cruise"
	                       <<",d_round,d_prev,d_round_valid,d_prev_valid,v_round,prev_v_round"
	                       <<",full_drate_ref,stable_cnt,bbr_stable,just_exited"
	                       <<",freq_tool_needed,freq_tool_on,w_freq"
	                       <<",unstable_episode_id,unstable_episode_active"
	                       <<",selection_native_bw_bps,current_native_bw_bps"
	                       <<",trusted_bw_bps,trusted_bw_valid,trusted_bw_conf"
	                       <<",trusted_bw_source,trusted_bw_cruise_id,trusted_bw_fresh"
	                       <<",trusted_bw_application_valid"
	                       <<",trusted_bw_ready_for_post_cruise"
	                       <<",trusted_bw_application_phase,trusted_bw_invalid_reason"
	                       <<",drate_spectral_integrity_score"
	                       <<",srtt_spectral_integrity_score"
	                       <<",joint_spectral_integrity_score"
	                       <<",drate_spectral_gate_pass,srtt_spectral_gate_pass"
	                       <<",dual_signal_spectral_gate_pass,limiting_spectral_signal"
	                       <<",pacing_base_bw_bps,pacing_base_source,phase_pacing_gain"
	                       <<",native_pacing_bps,final_pacing_rate_bps"
	                       <<",amplitude_bps,amplitude_bps_eff,triangle_wave"
	                       <<",current_delivery_rate,sample_is_app_limited,sample_valid"
	                       <<",merged_rescue_attempted,merged_rescue_success"
	                       <<",trusted_bw_selection_compute_us"
	                       <<",normal_window_count,merged_window_count"
	                       <<",spectral_invalid_count"
	                       <<",trusted_bw_cleared_on_cruise_start"<<std::endl;
    }
}
void DqcTrace::OpenFbbrGateFile(){
    if(!(m_enable & E_DQC_FBBR_GATE) || m_fbbrGate.is_open()) return;
    char buf[FILENAME_MAX];
    memset(buf,0,FILENAME_MAX);
    std::string path;
    if(0==kDqcTracePath.size()){
        path=std::string(getcwd(buf,FILENAME_MAX))+"/traces/flow"+
             std::to_string(m_id)+"_fbbr_gate_trace.csv";
    }else{
        path=kDqcTracePath+"flow"+std::to_string(m_id)+"_fbbr_gate_trace.csv";
    }
    m_fbbrGate.open(path.c_str(), std::fstream::out);
    if(m_fbbrGate.is_open()){
        m_fbbrGate<<"time,flow_id,row_type,round_id,bbr_state,probe_bw_phase,is_cruise"
                  <<",d_round,d_prev,d_round_valid,d_prev_valid,v_round,prev_v_round"
                  <<",full_drate_ref,stable_cnt,bbr_stable,just_exited"
                  <<",freq_tool_needed,freq_tool_on,w_freq"
                  <<",unstable_episode_id,unstable_episode_active"
                  <<",selection_native_bw_bps,current_native_bw_bps"
                  <<",trusted_bw_bps,trusted_bw_valid,trusted_bw_conf"
                  <<",trusted_bw_source,trusted_bw_cruise_id,trusted_bw_fresh"
                  <<",trusted_bw_application_valid"
                  <<",trusted_bw_ready_for_post_cruise"
                  <<",trusted_bw_application_phase,trusted_bw_invalid_reason"
                  <<",drate_spectral_integrity_score"
                  <<",srtt_spectral_integrity_score"
                  <<",joint_spectral_integrity_score"
                  <<",drate_spectral_gate_pass,srtt_spectral_gate_pass"
                  <<",dual_signal_spectral_gate_pass,limiting_spectral_signal"
                  <<",pacing_base_bw_bps,pacing_base_source,phase_pacing_gain"
                  <<",native_pacing_bps,final_pacing_rate_bps"
                  <<",amplitude_bps,amplitude_bps_eff,triangle_wave"
                  <<",current_delivery_rate,sample_is_app_limited,sample_valid"
                  <<",merged_rescue_attempted,merged_rescue_success"
                  <<",trusted_bw_selection_compute_us"
                  <<",normal_window_count,merged_window_count"
                  <<",spectral_invalid_count"
                  <<",trusted_bw_cleared_on_cruise_start"
                  <<",algorithm,probe_waveform,probe_factor"
                  <<",probe_period_rtts,probe_code_id"<<std::endl;
    }
}
void DqcTrace::OpenFbbrTriggerCycleFile(){
    if(!(m_enable & E_DQC_FBBR_LOAD) || m_fbbrTriggerCycle.is_open()) return;
    char buf[FILENAME_MAX];
    memset(buf,0,FILENAME_MAX);
    std::string path;
    if(0==kDqcTracePath.size()){
        path=std::string(getcwd(buf,FILENAME_MAX))+"/traces/flow"+
             std::to_string(m_id)+"_fbbr_trigger_cycles.csv";
    }else{
        path=kDqcTracePath+"flow"+std::to_string(m_id)+
             "_fbbr_trigger_cycles.csv";
    }
    m_fbbrTriggerCycle.open(path.c_str(),std::fstream::out);
    if(m_fbbrTriggerCycle.is_open()){
        m_fbbrTriggerCycle
            <<"trace_prefix,compat_trace_prefix,algorithm,flow_id,cruise_id,cycle_id"
            <<",cycle_start_s,cycle_end_s,window_state,pulser_role,carrier_period_s"
            <<",actual_input_amp,actual_input_energy,delivery_response_amp_bps"
            <<",delivery_response_bytes,period_estimate_s,period_error_ratio"
            <<",spectral_prominence_eta,normalized_match_rho,selected_delay_s"
            <<",phase_coverage,trigger_pass,continue_pass,trigger_reason,pause_reason"
            <<",app_limited_fraction,recovery_fraction,baseline_drift"
            <<",actual_input_measurable,period_match,weak_periodic_response"
            <<",commanded_amplitude,actual_input_snr,delivery_period_estimate_s"
            <<",delivery_period_error,delivery_prominence,delivery_match"
            <<",delivery_trigger_pass,delivery_continue_pass,delivery_reason"
            <<",queue_derivative_amp,queue_period_estimate_s,queue_period_error"
            <<",queue_prominence,queue_match,queue_noise_floor"
            <<",queue_trigger_pass,queue_continue_pass,queue_reason"
            <<",combined_trigger_source,combined_confidence,detected_cycle_start"
            <<",alignment_error_cycles,hard_safety\n";
    }
}
void DqcTrace::OpenFbbrBinFile(){
    if(!(m_enable & E_DQC_FBBR_LOAD) || m_fbbrBin.is_open()) return;
    char buf[FILENAME_MAX];
    memset(buf,0,FILENAME_MAX);
    std::string path;
    if(0==kDqcTracePath.size()){
        path=std::string(getcwd(buf,FILENAME_MAX))+"/traces/flow"+
             std::to_string(m_id)+"_fbbr_bins.csv";
    }else{
        path=kDqcTracePath+"flow"+std::to_string(m_id)+"_fbbr_bins.csv";
    }
    m_fbbrBin.open(path.c_str(),std::fstream::out);
    if(m_fbbrBin.is_open()){
        m_fbbrBin
            <<"trace_prefix,compat_trace_prefix,algorithm,flow_id,cruise_id,block_id,cycle_id,bin_id"
            <<",time_start_s,time_end_s,frequency_hz,period_rtts,code_id,code_sign"
            <<",phase_rad,coded_excitation,native_pacing_bps,commanded_pacing_bps"
            <<",actual_send_bps,sent_bytes,acked_bytes,delivery_rate_bps"
            <<",latest_rtt_us,qdelay_us,loss_ratio,ecn_ratio"
            <<",app_limited_fraction,cwnd_limited_fraction,coverage,rtt_valid,valid"
            <<",recovery_fraction,waveform,queue_servo_factor"
            <<",queue_servo_transition\n";
    }
}
void DqcTrace::OpenFbbrEventWindowFile(){
    if(!(m_enable & E_DQC_FBBR_LOAD) || m_fbbrEventWindow.is_open()) return;
    char buf[FILENAME_MAX];
    memset(buf,0,FILENAME_MAX);
    std::string path;
    if(0==kDqcTracePath.size()){
        path=std::string(getcwd(buf,FILENAME_MAX))+"/traces/flow"+
             std::to_string(m_id)+"_fbbr_event_windows.csv";
    }else{
        path=kDqcTracePath+"flow"+std::to_string(m_id)+"_fbbr_event_windows.csv";
    }
    m_fbbrEventWindow.open(path.c_str(),std::fstream::out);
    if(m_fbbrEventWindow.is_open()){
        m_fbbrEventWindow
            <<"trace_prefix,compat_trace_prefix,algorithm,flow_id,cruise_id,block_id,window_start_s,window_end_s"
            <<",frequency_hz,period_rtts,code_id,initial_phase_rad,target_amplitude_ratio"
            <<",realized_amplitude_ratio,rtprop_frozen_us,selected_delay_us,delay_ratio"
            <<",delay_at_search_boundary,cross_block_delay_stable,valid_cycles"
            <<",phase_bin_coverage,non_app_limited_fraction,input_cycle_coherence"
            <<",native_baseline_drift,delivery_baseline_drift,loss_ratio,ecn_ratio"
            <<",C_coverage,C_input,C_stationarity,C_cycle,C_response,C_delay,C_regression,C_meas"
            <<",snr_input,snr_delivery,snr_queue,snr_utility,r2_input,r2_delivery,r2_queue,r2_utility"
            <<",condition_input,condition_delivery,condition_queue,condition_utility"
            <<",G_d,G_q,phase_d,phase_q,phase_J,R2_d,R2_q,G_d_pos"
            <<",delta_q_pos_us,G_q_pos,q_zero_us,q_probe_max_us,q_floor_us,q95_us,q_amp_us"
            <<",drain_ratio,q_trend,g_lockin,g_fd,g_fused,h_fd,gradient_agreement"
            <<",S_full,S_lowq,S_stationary,S_safe,S_opt,rate_adjustment_signal,classification"
            <<",candidate_valid,candidate_bps,candidate_robust_cv,candidate_relative_ci_width"
            <<",candidate_cycle_count,candidate_invalid_reason,invalid_reason"
            <<",actual_input_amplitude_ratio,cwnd_limited_fraction,input_carrier_snr"
            <<",signature_leakage,residual_to_own_carrier_ratio,collision_suspected"
            <<",S_raw,E_under,E_over,direction_score,search_state_before,search_state_after"
            <<",baseline_before_bps,proposed_next_baseline_bps,applied_next_baseline_bps"
            <<",log_step,update_reason,hard_loss_abort,underload_bound_valid"
            <<",underload_bound_bps,overload_bound_valid,overload_bound_bps,bracket_width"
            <<",same_direction_streak,consecutive_dynamic,consecutive_near_optimal"
            <<",lock_candidate,locked,window_candidate_valid,window_candidate_bps"
            <<",trusted_bw_published,trusted_bw_control_bps,trusted_conf_control"
            <<",fbbr_rtprop_anchor_us,fbbr_rtprop_confidence,fbbr_rtprop_source"
            <<",gradient_se,gradient_ci90_low,gradient_ci90_high"
            <<",gradient_ci95_low,gradient_ci95_high,gradient_equivalent"
            <<",delivery_median_bps,search_active,unresolved_cruises"
            <<",unresolved_decisions,search_attempts,eligible_cruises"
            <<",last_failure_reason,control_baseline_source,trusted_bw_valid"
            <<",is_pulser,pulser_lease_remaining,waveform,recovery_fraction"
            <<",effective_cycles,carrier_detected,decision_collision_suspected"
            <<",search_generation,election_backoff_cycles,pulser_lease_count"
            <<",collision_count,provisional_state_valid,provisional_age_cruises"
            <<",carrier_sense_snr,carrier_sense_amplitude"
            <<",event_window_id,trigger_cycle_id,capture_start_s,capture_end_s"
            <<",window_length_cycles,window_stride_cycles,sequential_stop_reason"
            <<",window_state,trigger_cycle_excluded_from_score,overlap_fraction"
            <<",independent_for_control,independent_for_trusted,lockable_score"
            <<",S_sat,S_band,S_stable,S_target,q_min_us,q_L_us,q_H_us"
            <<",q_peak_cap_us,q95_event_us,queue_trend_event,D_freq,E_q,D_total"
            <<",delivery_spectral_prominence,delivery_normalized_match"
            <<",trigger_source,H_delivery_gain,H_queue_derivative_gain"
            <<",delivery_partition,queue_partition,C_delivery_channel"
            <<",C_queue_channel,queue_spectral_prominence,queue_normalized_match"
            <<",queue_servo_factor_mean,queue_servo_transition_cycles\n";
    }
}
void DqcTrace::OpenFbbrCruiseFile(){
    if(!(m_enable & E_DQC_FBBR_LOAD) || m_fbbrCruise.is_open()) return;
    char buf[FILENAME_MAX];
    memset(buf,0,FILENAME_MAX);
    std::string path;
    if(0==kDqcTracePath.size()){
        path=std::string(getcwd(buf,FILENAME_MAX))+"/traces/flow"+
             std::to_string(m_id)+"_fbbr_cruises.csv";
    }else{
        path=kDqcTracePath+"flow"+std::to_string(m_id)+"_fbbr_cruises.csv";
    }
    m_fbbrCruise.open(path.c_str(),std::fstream::out);
    if(m_fbbrCruise.is_open()){
        m_fbbrCruise
            <<"trace_prefix,compat_trace_prefix,algorithm,flow_id,cruise_id,start_time_s,end_time_s"
            <<",probe_enabled,probe_disabled_reason,frequency_hz,period_rtts,code_id"
            <<",amplitude_ratio,block_count,underload_count,near_optimal_count"
            <<",overload_count,dynamic_count,invalid_count,consensus_valid"
            <<",raw_candidate_bps,consensus_confidence,robust_cv,relative_ci_width"
            <<",consensus_block_count,consensus_cycle_count,source_block_id"
            <<",consensus_invalid_reason,publication_valid,published_bps"
            <<",history_update_action,publication_invalid_reason,trusted_bw_bps"
            <<",trusted_bw_confidence,trusted_bw_source,trusted_bw_invalid_reason"
            <<",fair_share_bps,search_active,search_state,eligible_cruises"
            <<",search_attempts,unresolved_cruises,unresolved_decisions"
            <<",valid_direction_decisions,last_failure_reason,is_pulser"
            <<",pulser_lease_remaining,waveform,carrier_detected,search_generation"
            <<",pulser_lease_count,collision_count,watcher_decisions,drain_decisions"
            <<",seek_decisions,track_decisions,persistent_unresolved_decisions"
            <<",trusted_bw_age_cruises,carrier_sense_snr"
            <<",carrier_sense_amplitude,dual_trigger_attempts,delivery_triggers"
            <<",queue_triggers,both_triggers,hard_safety_events,event_windows"
            <<",dense_windows,queue_servo_updates,queue_servo_drain_rtts"
            <<",queue_servo_recovery_rtts,baseline_commits,slow_loop_updates"
            <<",trusted_publications,persistent_retry\n";
    }
}
void DqcTrace::OpenFbbrDiagnosticWindowFile(){
    if(!(m_enable & E_DQC_FBBR_LOAD) || m_fbbrDiagnosticWindow.is_open()) return;
    char buf[FILENAME_MAX];
    memset(buf,0,FILENAME_MAX);
    std::string path;
    if(0==kDqcTracePath.size()){
        path=std::string(getcwd(buf,FILENAME_MAX))+"/traces/flow"+
             std::to_string(m_id)+"_fbbr_diagnostic_windows.csv";
    }else{
        path=kDqcTracePath+"flow"+std::to_string(m_id)+
             "_fbbr_diagnostic_windows.csv";
    }
    m_fbbrDiagnosticWindow.open(path.c_str(),std::fstream::out);
    if(m_fbbrDiagnosticWindow.is_open()){
        m_fbbrDiagnosticWindow
            <<"trace_prefix,compat_trace_prefix,algorithm,flow_id,cruise_id,window_id,window_start_s,window_end_s"
            <<",frequency_hz,period_rtts,code_id,initial_phase_rad,target_amplitude_ratio"
            <<",realized_amplitude_ratio,rtprop_frozen_us,selected_delay_us,delay_ratio"
            <<",delay_at_search_boundary,cross_block_delay_stable,valid_cycles"
            <<",phase_bin_coverage,non_app_limited_fraction,input_cycle_coherence"
            <<",native_baseline_drift,delivery_baseline_drift,loss_ratio,ecn_ratio"
            <<",C_coverage,C_input,C_stationarity,C_cycle,C_response,C_delay,C_regression,C_meas"
            <<",snr_input,snr_delivery,snr_queue,snr_utility,r2_input,r2_delivery,r2_queue,r2_utility"
            <<",condition_input,condition_delivery,condition_queue,condition_utility"
            <<",G_d,G_q,phase_d,phase_q,phase_J,R2_d,R2_q,G_d_pos"
            <<",delta_q_pos_us,G_q_pos,q_zero_us,q_probe_max_us,q_floor_us,q95_us,q_amp_us"
            <<",drain_ratio,q_trend,g_lockin,g_fd,g_fused,h_fd,gradient_agreement"
            <<",S_full,S_lowq,S_stationary,S_safe,S_opt,rate_adjustment_signal,classification"
            <<",candidate_valid,candidate_bps,candidate_robust_cv,candidate_relative_ci_width"
            <<",candidate_cycle_count,candidate_invalid_reason,invalid_reason"
            <<",shadow_window,decision_eligible\n";
    }
}
void DqcTrace::OpenFbbrQueueServoFile(){
    if(!(m_enable & E_DQC_FBBR_LOAD) || m_fbbrQueueServo.is_open()) return;
    char buf[FILENAME_MAX];
    memset(buf,0,FILENAME_MAX);
    std::string path;
    if(0==kDqcTracePath.size()){
        path=std::string(getcwd(buf,FILENAME_MAX))+"/traces/flow"+
             std::to_string(m_id)+"_fbbr_queue_servo.csv";
    }else{
        path=kDqcTracePath+"flow"+std::to_string(m_id)+
             "_fbbr_queue_servo.csv";
    }
    m_fbbrQueueServo.open(path.c_str(),std::fstream::out);
    if(m_fbbrQueueServo.is_open()){
        m_fbbrQueueServo
            <<"trace_prefix,compat_trace_prefix,algorithm,time_s,flow_id,cruise_id"
            <<",servo_state,search_baseline_bps,servo_factor"
            <<",final_nonprobe_baseline_bps,q_floor_fast_us,q_median_fast_us"
            <<",q_peak_fast_us,q_low_us,q_high_us,q_peak_cap_us"
            <<",queue_trend_fast,delivery_median_fast_bps,loss_ratio_fast"
            <<",ecn_ratio_fast,down_correction,up_correction"
            <<",consecutive_drain_rtts,baseline_commit_eligible"
            <<",baseline_commit_applied,baseline_commit_bps,reason\n";
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
