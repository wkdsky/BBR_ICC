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
    if(enable&E_DQC_OWD){
        OpenOwdFile(name);
    }
    if(enable&E_DQC_RTT){
        OpenRttFile(name);
    }
    if(enable&E_DQC_BW){
        OpenBandwidthFile(name);
    }
    if(enable&E_DQC_GOODPUT){
        OpenGoodputFile(name);
    }
    if(enable&E_DQC_SEND_RATE){
        OpenSendRateFile(name);
    }
    if(enable&E_DQC_RECV_RATE){
        OpenRecvRateFile(name);
    }
    if(enable&E_DQC_BBR_MODE){
        OpenBbrModeFile(name);
    }
    if(enable&E_DQC_STAT){
        OpenStatsFile(name);
    }
}
void DqcTrace::OpenOwdFile(std::string name){
    char buf[FILENAME_MAX];
    memset(buf,0,FILENAME_MAX);
    std::string path;
    if(0==kDqcTracePath.size()){
        path= std::string (getcwd(buf, FILENAME_MAX)) + "/traces/"
            +name+"_owd.txt";
    }else{
        path=std::string(kDqcTracePath)+name+"_owd.txt";
    }
    m_owd.open(path.c_str(), std::fstream::out);    
}
void DqcTrace::OpenRttFile(std::string name){
    char buf[FILENAME_MAX];
    memset(buf,0,FILENAME_MAX);
    std::string path;
    if(0==kDqcTracePath.size()){
        path= std::string (getcwd(buf, FILENAME_MAX)) + "/traces/"
            +name+"_rtt.txt";
    }else{
        path=std::string(kDqcTracePath)+name+"_rtt.txt";
    }

    m_rtt.open(path.c_str(), std::fstream::out);    
}
void DqcTrace::OpenBandwidthFile(std::string name){
    char buf[FILENAME_MAX];
    memset(buf,0,FILENAME_MAX);
    std::string path;
    if(0==kDqcTracePath.size()){
        path = std::string (getcwd(buf, FILENAME_MAX)) + "/traces/"
                +name+"_bw.txt";
    }else{
        path=std::string(kDqcTracePath)+name+"_bw.txt";
    }
    m_bw.open(path.c_str(), std::fstream::out);     
}
void DqcTrace::OpenGoodputFile(std::string name){
    char buf[FILENAME_MAX];
    memset(buf,0,FILENAME_MAX);
    std::string path;
    if(0==kDqcTracePath.size()){
        path=std::string (getcwd(buf, FILENAME_MAX)) + "/traces/"
            +name+"_good.txt";
    }else{
        path=std::string(kDqcTracePath)+name+"_good.txt";
    }
    m_googput.open(path.c_str(), std::fstream::out);
}
void DqcTrace::OpenSendRateFile(std::string name){
    char buf[FILENAME_MAX];
    memset(buf,0,FILENAME_MAX);
    std::string path;
    if(0==kDqcTracePath.size()){
        path=std::string (getcwd(buf, FILENAME_MAX)) + "/traces/"
            +name+"_sendrate.txt";
    }else{
        path=std::string(kDqcTracePath)+name+"_sendrate.txt";
    }
    m_sendRate.open(path.c_str(), std::fstream::out);
}
void DqcTrace::OpenRecvRateFile(std::string name){
    char buf[FILENAME_MAX];
    memset(buf,0,FILENAME_MAX);
    std::string path;
    if(0==kDqcTracePath.size()){
        path=std::string (getcwd(buf, FILENAME_MAX)) + "/traces/"
            +name+"_recvrate.txt";
    }else{
        path=std::string(kDqcTracePath)+name+"_recvrate.txt";
    }
    m_recvRate.open(path.c_str(), std::fstream::out);
}
void DqcTrace::OpenBbrModeFile(std::string name){
    char buf[FILENAME_MAX];
    memset(buf,0,FILENAME_MAX);
    std::string path;
    if(0==kDqcTracePath.size()){
        path=std::string (getcwd(buf, FILENAME_MAX)) + "/traces/"
            +name+"_bbrmode.txt";
    }else{
        path=std::string(kDqcTracePath)+name+"_bbrmode.txt";
    }
    m_bbrMode.open(path.c_str(), std::fstream::out);
}
void DqcTrace::OpenStatsFile(std::string name){
    char buf[FILENAME_MAX];
    memset(buf,0,FILENAME_MAX);
    std::string path;
    if(0==kDqcTracePath.size()){
        path = std::string (getcwd(buf, FILENAME_MAX)) + "/traces/"
            +name+"_stats.txt";
    }else{
        path=std::string(kDqcTracePath)+name+"_stats.txt";
    }

    m_stats.open(path.c_str(), std::fstream::out);    
}
void DqcTrace::OnOwd(uint32_t seq,uint32_t owd,uint32_t size){
    if(m_owd.is_open()){
        float now=Simulator::Now().GetSeconds();
        m_owd<<now<<"\t"<<seq<<"\t"<<owd<<"\t"<<size<<std::endl;
    }    
}
void DqcTrace::OnRtt(uint32_t seq,uint32_t rtt){
    if(m_rtt.is_open()){
        float now=Simulator::Now().GetSeconds();
        m_rtt<<now<<"\t"<<seq<<"\t"<<rtt<<std::endl;
    }    
}
void DqcTrace::OnBw(int32_t kbps){
    if(m_bw.is_open()){
        float now=Simulator::Now().GetSeconds();
        m_bw<<now<<"\t"<<kbps<<std::endl;
    }       
    
}
void DqcTrace::OnGoodput(uint32_t kbps){
	if(m_googput.is_open()){
		float now=Simulator::Now().GetSeconds();
        m_googput<<now<<"\t"<<kbps<<std::endl;
	}
}
void DqcTrace::OnSendRate(int32_t kbps){
    if(m_sendRate.is_open()){
        float now=Simulator::Now().GetSeconds();
        m_sendRate<<now<<"\t"<<kbps<<std::endl;
    }
}
void DqcTrace::OnRecvRate(int32_t kbps){
    if(m_recvRate.is_open()){
        float now=Simulator::Now().GetSeconds();
        m_recvRate<<now<<"\t"<<kbps<<std::endl;
    }
}
void DqcTrace::OnBbrMode(int32_t mode){
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
void DqcTrace::OnStats(uint64_t recv_count,uint64_t largest,
                        uint64_t recv_bytes,uint64_t duration,
                       float avg_owd){
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
