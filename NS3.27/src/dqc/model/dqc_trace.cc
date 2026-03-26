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
void DqcTrace::Log(std::string name,uint16_t enable){
    // Just store the name and enable flags, files will be opened lazily on first write
    m_name = name;
    m_enable = enable;
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
        m_recvRate<<"#time(s)\tinstant_recv_rate(kbps)"<<std::endl;
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
void DqcTrace::OpenAckEventFile(){
    if(!(m_enable & E_DQC_ACK_EVENT) || m_ackEvent.is_open()) return;
    char buf[FILENAME_MAX];
    memset(buf,0,FILENAME_MAX);
    std::string path;
    if(0==kDqcTracePath.size()){
        path= std::string (getcwd(buf, FILENAME_MAX)) + "/traces/"
            +m_name+"_ackevent.txt";
    }else{
        path=std::string(kDqcTracePath)+m_name+"_ackevent.txt";
    }
    m_ackEvent.open(path.c_str(), std::fstream::out);
    if(m_ackEvent.is_open()){
        m_ackEvent<<"#time(s)\tacked_bytes\tacked_pkts\tlargest_acked\tack_delay(ms)\trtt(ms)\t"
                  <<"ack_interval(ms)\tack_rate(kbps)\tpacing_rate(kbps)\tsample_bias"<<std::endl;
    }
}
void DqcTrace::OpenAckEpisodeFile(){
    if(!(m_enable & E_DQC_ACK_EPISODE) || m_ackEpisode.is_open()) return;
    char buf[FILENAME_MAX];
    memset(buf,0,FILENAME_MAX);
    std::string path;
    if(0==kDqcTracePath.size()){
        path= std::string (getcwd(buf, FILENAME_MAX)) + "/traces/"
            +m_name+"_ackepisode.txt";
    }else{
        path=std::string(kDqcTracePath)+m_name+"_ackepisode.txt";
    }
    m_ackEpisode.open(path.c_str(), std::fstream::out);
    if(m_ackEpisode.is_open()){
        m_ackEpisode<<"#type\tstart(s)\tend(s)\tduration(ms)\tack_events\tacked_bytes\t"
                    <<"iat_min(ms)\tiat_max(ms)\tack_rate_peak(kbps)\t"
                    <<"pacing_rate_mean(kbps)\tbias_peak"<<std::endl;
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
        if (m_ccType == dqc::kFreqCCv4) {
            m_upPhase<<"#start_time(s)\tduration(ms)\tfreq(Hz)\t1.25BDP_exit\tcycles\tbw_estimate(kbps)"<<std::endl;
        } else {
            m_upPhase<<"#start_time(s)\tduration(ms)\tfreq(Hz)\t1.25BDP_exit\tcycles\tpacing_gain\tbw_estimate(kbps)"<<std::endl;
        }
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
        m_owd<<now<<"\t"<<seq<<"\t"<<owd<<"\t"<<size<<std::endl;
    }    
}
void DqcTrace::OnRtt(uint32_t seq,uint32_t rtt,uint32_t smoothed_rtt){
    OpenRttFile();  // Lazy open
    if(m_rtt.is_open()){
        float now=Simulator::Now().GetSeconds();
        m_rtt<<now<<"\t"<<seq<<"\t"<<rtt<<"\t"<<smoothed_rtt<<std::endl;
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
        m_bw<<now<<"\t"<<kbps<<std::endl;
    }

}
void DqcTrace::OnGoodput(uint32_t kbps){
    OpenGoodputFile();  // Lazy open
	if(m_googput.is_open()){
		float now=Simulator::Now().GetSeconds();
        m_googput<<now<<"\t"<<kbps<<std::endl;
	}
}
void DqcTrace::OnSendRate(int32_t kbps){
    OpenSendRateFile();  // Lazy open
    if(m_sendRate.is_open()){
        float now=Simulator::Now().GetSeconds();
        m_sendRate<<now<<"\t"<<kbps<<std::endl;
    }
}
void DqcTrace::OnRecvRate(int32_t instant_kbps){
    OpenRecvRateFile();  // Lazy open
    if(m_recvRate.is_open()){
        float now=Simulator::Now().GetSeconds();
        m_recvRate<<now<<"\t"<<instant_kbps<<std::endl;
    }
}
void DqcTrace::OnQueueDelay(uint32_t queue_delay_ms,uint32_t latest_rtt_ms,uint32_t min_rtt_ms){
    OpenQueueDelayFile();
    if(m_queueDelay.is_open()){
        float now=Simulator::Now().GetSeconds();
        m_queueDelay<<now<<"\t"<<queue_delay_ms<<"\t"<<latest_rtt_ms<<"\t"<<min_rtt_ms<<std::endl;
    }
}
void DqcTrace::OnInflight(int32_t inflight_bytes,int32_t cwnd_bytes){
    OpenInflightFile();  // Lazy open
    if(m_inflight.is_open()){
        float now=Simulator::Now().GetSeconds();
        m_inflight<<now<<"\t"<<inflight_bytes<<"\t"<<cwnd_bytes<<std::endl;
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
        m_bbrMode<<now<<"\t"<<mode_name<<std::endl;
    }
}
void DqcTrace::OnLossRate(double time_sec,float loss_rate,float cumulative_loss_rate){
    OpenLossRateFile();  // Lazy open
    if(m_lossRate.is_open()){
        m_lossRate<<time_sec<<"\t"<<loss_rate<<"\t"<<cumulative_loss_rate<<std::endl;
    }
}
void DqcTrace::OnAckEvent(const AckEventRecord &record){
    OpenAckEventFile();  // Lazy open
    if(m_ackEvent.is_open()){
        m_ackEvent<<record.time_sec<<"\t"<<record.acked_bytes<<"\t"<<record.acked_pkts<<"\t"
                  <<record.largest_acked<<"\t"<<record.ack_delay_ms<<"\t"<<record.rtt_ms<<"\t"
                  <<record.ack_interval_ms<<"\t"<<record.ack_rate_kbps<<"\t"
                  <<record.pacing_rate_kbps<<"\t"<<record.sample_bias<<std::endl;
    }
}
void DqcTrace::OnAckEpisode(const AckEpisodeRecord &record){
    OpenAckEpisodeFile();  // Lazy open
    if(m_ackEpisode.is_open()){
        const char* type_name = (record.type==0) ? "compress" : "aggregate";
        m_ackEpisode<<type_name<<"\t"<<record.start_sec<<"\t"<<record.end_sec<<"\t"
                    <<record.duration_ms<<"\t"<<record.ack_events<<"\t"<<record.acked_bytes<<"\t"
                    <<record.iat_min_ms<<"\t"<<record.iat_max_ms<<"\t"
                    <<record.ack_rate_peak_kbps<<"\t"<<record.pacing_rate_mean_kbps<<"\t"
                    <<record.bias_peak<<std::endl;
    }
}
void DqcTrace::OnUpPhase(double start_time,double duration_ms,double freq_hz,bool exit_due_to_queueing,int cycles,float pacing_gain,int32_t bw_estimate_kbps){
    OpenUpPhaseFile();  // Lazy open
    if(m_upPhase.is_open()){
        if (m_ccType == dqc::kFreqCCv4) {
            m_upPhase<<start_time<<"\t"<<duration_ms<<"\t"<<freq_hz<<"\t"<<(exit_due_to_queueing?"true":"false")<<"\t"<<cycles<<"\t"<<bw_estimate_kbps<<std::endl;
        } else {
            m_upPhase<<start_time<<"\t"<<duration_ms<<"\t"<<freq_hz<<"\t"<<(exit_due_to_queueing?"true":"false")<<"\t"<<cycles<<"\t"<<pacing_gain<<"\t"<<bw_estimate_kbps<<std::endl;
        }
    }
}
void DqcTrace::OnFreqAnalysis(double start_time, double duration_sec, double sender_peak_freq_hz, double receiver_peak_freq_hz, int32_t avg_rate_kbps){
    OpenFreqAnalysisFile(); // Lazy open
    if(m_freqAnalysis.is_open()){
        m_freqAnalysis<<start_time<<"\t"<<duration_sec<<"\t"<<sender_peak_freq_hz<<"\t"<<receiver_peak_freq_hz<<"\t"<<avg_rate_kbps<<std::endl;
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
    CloseQueueDelayFile();
    CloseInflightFile();
    CloseBbrModeFile();
    CloseLossRateFile();
    CloseAckEventFile();
    CloseAckEpisodeFile();
    CloseUpPhaseFile();
    CloseFreqAnalysisFile();
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
void DqcTrace::CloseAckEventFile(){
    if(m_ackEvent.is_open()){
        m_ackEvent.flush();
        m_ackEvent.close();
    }
}
void DqcTrace::CloseAckEpisodeFile(){
    if(m_ackEpisode.is_open()){
        m_ackEpisode.flush();
        m_ackEpisode.close();
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
            +m_name+"_freq.txt";
    }else{
        path=std::string(kDqcTracePath)+m_name+"_freq.txt";
    }
    m_freqAnalysis.open(path.c_str(), std::fstream::out);
    if(m_freqAnalysis.is_open()){
        m_freqAnalysis<<"#start_time(s)\tduration(s)\tsender_peak_freq(Hz)\treceiver_peak_freq(Hz)\tavg_rate(kbps)"<<std::endl;
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
void DqcTraceState::Flush(uint32_t capacity,uint32_t simulation_time){
    m_count++;
    double average_rate=1.0*m_totalRecvBytes*8/simulation_time;
    double util=(average_rate/capacity);
    double loss=10000.0-10000.0*m_recvCount/m_totalRecv;
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
