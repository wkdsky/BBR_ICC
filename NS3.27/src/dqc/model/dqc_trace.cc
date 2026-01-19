#include <unistd.h>
#include <dirent.h>
#include <memory.h>
#include <string>
#include "ns3/dqc_trace.h"
#include "ns3/simulator.h"
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
        m_recvRate<<"#time(s)\tinstant_recv_rate(kbps)\tbw_estimate(kbps)"<<std::endl;
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
        m_upPhase<<"#start_time(s)\tduration(ms)\tfreq(Hz)\tcycles\tbw_estimate(kbps)"<<std::endl;
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
void DqcTrace::OnRecvRate(int32_t instant_kbps,int32_t bw_estimate_kbps){
    OpenRecvRateFile();  // Lazy open
    if(m_recvRate.is_open()){
        float now=Simulator::Now().GetSeconds();
        m_recvRate<<now<<"\t"<<instant_kbps<<"\t"<<bw_estimate_kbps<<std::endl;
    }
}
void DqcTrace::OnBbrMode(int32_t mode){
    OpenBbrModeFile();  // Lazy open
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
            "probeRTT"
        };
        const char* mode_name = (mode >= 0 && mode <= 6) ? mode_names[mode] : "unknown";
        m_bbrMode<<now<<"\t"<<mode_name<<std::endl;
    }
}
void DqcTrace::OnUpPhase(double start_time,double duration_ms,double freq_hz,int cycles,int32_t bw_estimate_kbps){
    OpenUpPhaseFile();  // Lazy open
    if(m_upPhase.is_open()){
        m_upPhase<<start_time<<"\t"<<duration_ms<<"\t"<<freq_hz<<"\t"<<cycles<<"\t"<<bw_estimate_kbps<<std::endl;
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
    CloseBbrModeFile();
    CloseUpPhaseFile();
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
void DqcTrace::CloseBbrModeFile(){
    if(m_bbrMode.is_open()){
        m_bbrMode.flush();
        m_bbrMode.close();
    }
}
void DqcTrace::CloseUpPhaseFile(){
    if(m_upPhase.is_open()){
        m_upPhase.flush();
        m_upPhase.close();
    }
}
void DqcTrace::CloseStatsFile(){
    if(m_stats.is_open()){
        m_stats.close();
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
    uint64_t m_delayCount=0;
    uint64_t m_sumDelay=0;
    m_delayIds.clear();
}
}
