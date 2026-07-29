#include <algorithm>
#include <limits>
#include <string>
#include <cstdint>
#include "ns3/simulator.h"
#include "ns3/dqc_sender.h"
#include "ns3/dqc_trace.h"
#include "ns3/log.h"
#include "ns3/time_tag.h"
#include "ns3/flow-id-tag.h"
#include "ns3/byte_codec.h"
#include "ns3/proto_utils.h"
#include "ns3/freqcc_sender.h"
#include "ns3/freqccv2_sender.h"
#include "ns3/freqccv3_sender.h"
#include "ns3/fbbr_sender.h"
#include "ns3/obbr_sender.h"
#include "proto_bbr_sender.h"
#include "ns3/quic_bbr2_sender.h"
#include "ns3/quic_bbr2plus_sender.h"
#include "ns3/proto_send_algorithm_interface.h"
using namespace dqc;
namespace ns3{
NS_LOG_COMPONENT_DEFINE("dqcsender");
using namespace dqc;

namespace {

int32_t
ProtoBbrModeToTraceIndex(ProtoBbrSender::Mode mode)
{
    switch (mode) {
    case ProtoBbrSender::STARTUP:
        return 0;
    case ProtoBbrSender::DRAIN:
        return 1;
    case ProtoBbrSender::PROBE_BW:
        return 3;
    case ProtoBbrSender::PROBE_RTT:
        return 6;
    }
    return 3;
}

bool
IsBbr2StyleAlgorithm(CongestionControlType type)
{
    return type == kBBRv2 || type == kBBRv2Ecn ||
           type == kBBRv2NoProbeRtt ||
           type == kBBRv2Plus || type == kBBRv2PlusEcn ||
           type == kFreqCC || type == kFreqCCv2 ||
           type == kFreqCCv3 || type == kFBBR ||
           type == kFBBRServiceFair;
}

bool
IsFBBRAlgorithm(CongestionControlType type)
{
    return type == kFBBR || type == kFBBRServiceFair;
}

std::string
Bbr2ProbePhaseName(Bbr2ProbeBwMode::CyclePhase phase)
{
    return Bbr2ProbeBwMode::CyclePhaseToString(phase);
}

FBBRGateTraceMode
ParseFBBRGateTraceMode(const std::string& mode)
{
    if(mode == "off"){
        return FBBRGateTraceMode::kOff;
    }
    if(mode == "sampled_pacing"){
        return FBBRGateTraceMode::kSampledPacing;
    }
    if(mode == "full"){
        return FBBRGateTraceMode::kFull;
    }
    return FBBRGateTraceMode::kRoundOnly;
}

}  // namespace
// in order to get the ip addr of node
void ConvertIp32(uint32_t ip,std::string &addr){
 uint8_t first=(ip&0xff000000)>>24;
 uint8_t second=(ip&0x00ff0000)>>16;
 uint8_t third=(ip&0x0000ff00)>>8;
 uint8_t fourth=(ip&0x000000ff);
 std::string dot=std::string(".");
 addr=std::to_string(first)+dot+std::to_string(second)+dot+std::to_string(third)+dot+std::to_string(fourth);
}
int FakePackeWriter::SendTo(const char*buf,size_t size,dqc::SocketAddress &dst){
    Ptr<Packet> p=Create<Packet>((uint8_t*)buf,size);
	m_sender->SendToNetwork(p);
	return size;
}
/*kQueueLimit kShadow kBBRv2 kBBR kPOTEN kCubicBytes*/
DqcSender::DqcSender(bool ecn):DqcSender(kBBR,ecn){}
DqcSender::DqcSender(dqc::CongestionControlType cc_type,bool ecn,bool engine_time)
:m_writer(this)
,m_alarmFactory(new ProcessAlarmFactory(&m_timeDriver))
,m_connection(&m_clock,m_alarmFactory.get(), cc_type){
	m_ecn=ecn;
    m_enableEngineTimer=engine_time;
}
void DqcSender::SetBwTraceFuc(TraceBandwidth cb){
	m_traceBwCb=cb;
	if(m_traceBwCb.IsNull()){
		NS_LOG_INFO("bw trace is null");
		abort();
	}
}
void DqcSender::SetMaxBandwidth(uint32_t bps){
    m_connection.SetMaxBandwidth(bps);
}
void DqcSender::SetSentSeqTraceFuc(TraceSentSeq cb){
	m_traceSentSeqCb=cb;
	if(m_traceSentSeqCb.IsNull()){
		NS_LOG_INFO("bw trace is null");
		abort();
	}
}
void DqcSender::SetTraceLossPacketDelay(TraceLossPacketDelay cb){
    m_traceLossDelay=cb;
    EnsureLossTraceHooked();
}
void DqcSender::SetLossRateTraceFuc(TraceLossRate cb){
    m_traceLossRateCb=cb;
    EnsureLossTraceHooked();
}
void DqcSender::SetTraceOwdAtSender(TraceOwdAtSender cb){
	 m_traceOwd=cb;
}
void DqcSender::SetRttTraceFuc(TraceRtt cb){
    m_traceRttCb=cb;
    EnsureRttTraceHooked();
}
void DqcSender::SetQueueDelayTraceFuc(TraceQueueDelay cb){
    m_traceQueueDelayCb=cb;
}
void DqcSender::SetSendRateTraceFuc(TraceSendRate cb){
    m_traceSendRateCb=cb;
}
void DqcSender::SetRecvRateTraceFuc(TraceRecvRate cb){
    m_traceRecvRateCb=cb;
}
void DqcSender::SetRecvRateRawTraceFuc(TraceRecvRateRaw cb){
    m_traceRecvRateRawCb=cb;
}
void DqcSender::SetInflightTraceFuc(TraceInflight cb){
    m_traceInflightCb=cb;
}
void DqcSender::SetBbrModeTraceFuc(TraceBbrMode cb){
    m_traceBbrModeCb=cb;
}
void DqcSender::SetUpPhaseTraceFuc(TraceUpPhase cb){
    m_traceUpPhaseCb=cb;
}
void DqcSender::SetFreqAnalysisTraceFuc(TraceFreqAnalysis cb){
    m_traceFreqAnalysisCb=cb;
}
void DqcSender::SetRttFreqAnalysisTraceFuc(TraceRttFreqAnalysis cb){
    m_traceRttFreqAnalysisCb=cb;
}
void DqcSender::SetFBBRLoadTraceFuc(TraceFBBRLoad cb){
    m_traceFBBRLoadCb=cb;
}
void DqcSender::SetEquivalenceAuditTracePrefix(const std::string& prefix){
    m_equivalenceAuditPrefix=prefix;
    CloseEquivalenceAuditTrace();
    if(m_equivalenceAuditPrefix.empty()){
        return;
    }
    m_equivalenceSent.open((m_equivalenceAuditPrefix+"_sent_audit.csv").c_str(),
                           std::fstream::out);
    if(m_equivalenceSent.is_open()){
        m_equivalenceSent
            <<"flow_id,packet_number,send_time_s,packet_size"
            <<",commanded_pacing_rate,search_baseline,commanded_probe_offset"
            <<",carrier_phase,pacer_requested_delay_us"
            <<",ideal_send_time_before_us,ideal_next_send_time_us"
            <<",actual_emission_time_us,emission_lateness_us,send_quantum"
            <<",burst_tokens,lumpy_tokens,bytes_in_flight,cwnd"
            <<",is_cwnd_limited,is_app_limited,is_pacing_limited"
            <<",fine_grained,is_pulser,search_active,cum_sent_bytes\n";
    }
    m_equivalenceAcked.open((m_equivalenceAuditPrefix+"_acked_audit.csv").c_str(),
                            std::fstream::out);
    if(m_equivalenceAcked.is_open()){
        m_equivalenceAcked<<"time_s,sent_time_s,bytes_acked,cum_acked_bytes\n";
    }
    m_equivalencePacing.open((m_equivalenceAuditPrefix+"_pacing_audit.csv").c_str(),
                             std::fstream::out);
    if(m_equivalencePacing.is_open()){
        m_equivalencePacing<<"time_s,native_pacing_bps,final_pacing_rate_bps"
                           <<",current_native_bw_bps,pacing_base_bw_bps"
                           <<",pacing_base_source,phase_pacing_gain"
                           <<",should_oscillate,trusted_bw_valid\n";
    }
    m_equivalenceSentBytes=0;
    m_equivalenceAckedBytes=0;
    EnsureLossTraceHooked();
}
void DqcSender::CloseEquivalenceAuditTrace(){
    if(m_equivalenceSent.is_open()){
        m_equivalenceSent.close();
    }
    if(m_equivalenceAcked.is_open()){
        m_equivalenceAcked.close();
    }
    if(m_equivalencePacing.is_open()){
        m_equivalencePacing.close();
    }
}
void DqcSender::OnPacketLossInfo(dqc::PacketNumber seq,uint32_t rtt,uint32_t bytes_lost,dqc::ProtoTime sent_ts){
    uint64_t window_id=GetLossWindowId(sent_ts);
    m_lossWindows[window_id].lost_bytes+=bytes_lost;
    FlushLossWindows(false);
    if(!m_traceLossDelay.IsNull()){
        int32_t num=(int32_t)seq.ToUint64();
        m_traceLossDelay(num,rtt,bytes_lost);
    }
}
void DqcSender::OnPacketSent(dqc::PacketNumber seq,dqc::ProtoTime sent_ts,uint32_t bytes_sent){
    uint64_t window_id=GetLossWindowId(sent_ts);
    m_lossWindows[window_id].sent_bytes+=bytes_sent;
    if(m_equivalenceSent.is_open()){
        m_equivalenceSentBytes+=bytes_sent;
        SendPacketManager *manager=m_connection.GetSentPacketManager();
        PacingSender::DebugState pacer;
        ByteCount inflight=0;
        ByteCount cwnd=0;
        if(manager){
            pacer=manager->GetPacingDebugState();
            manager->InFlight(&inflight,&cwnd);
            if(pacer.commanded_pacing_bps == 0){
                pacer.commanded_pacing_bps = manager->PacingRate().ToBitsPerSecond();
            }
        }
        const bool is_cwnd_limited = cwnd > 0 && inflight >= cwnd;
        m_equivalenceSent
            <<m_id<<","<<seq.ToUint64()<<","<<
              static_cast<double>(sent_ts.ToDebuggingValue())/1e6<<","<<bytes_sent
            <<","<<pacer.commanded_pacing_bps
            <<",0"
            <<",0"
            <<",0"
            <<","<<pacer.requested_delay_us
            <<","<<pacer.ideal_send_time_before_us
            <<","<<pacer.ideal_next_send_time_us
            <<","<<pacer.actual_emission_time_us
            <<","<<pacer.emission_lateness_us
            <<","<<kDefaultTCPMSS
            <<","<<pacer.burst_tokens<<","<<pacer.lumpy_tokens
            <<","<<inflight<<","<<cwnd
            <<","<<(is_cwnd_limited?"true":"false")
            <<",false"
            <<","<<(pacer.pacing_limited?"true":"false")
            <<","<<(pacer.fine_grained?"true":"false")
            <<",false"
            <<",false"
            <<","<<m_equivalenceSentBytes<<"\n";
    }
}
void DqcSender::OnPacketAcked(dqc::ProtoTime sent_ts,uint32_t bytes_acked){
    uint64_t window_id=GetLossWindowId(sent_ts);
    m_lossWindows[window_id].acked_bytes+=bytes_acked;
    if(m_equivalenceAcked.is_open()){
        m_equivalenceAckedBytes+=bytes_acked;
        const double now_s=Simulator::Now().GetSeconds();
        const double sent_s=
            static_cast<double>(sent_ts.ToDebuggingValue())/1000000.0;
        m_equivalenceAcked<<now_s<<","
                          <<sent_s<<","
                          <<bytes_acked<<","
                          <<m_equivalenceAckedBytes<<"\n";
    }
    FlushLossWindows(false);
}
uint64_t DqcSender::GetLossWindowId(dqc::ProtoTime sent_ts) const{
    uint64_t sent_us=static_cast<uint64_t>(sent_ts.ToDebuggingValue());
    return sent_us/m_lossWindowIntervalUs;
}
void DqcSender::FlushLossWindows(bool final_flush){
    if(m_traceLossRateCb.IsNull()){
        return;
    }
    double now_sec=0.0;
    if(final_flush){
        now_sec=Simulator::Now().GetSeconds();
    }
    while(!m_lossWindows.empty()){
        auto it=m_lossWindows.begin();
        const LossWindowStats &stats=it->second;
        if(stats.sent_bytes==0){
            m_lossWindows.erase(it);
            continue;
        }
        if(!final_flush && stats.acked_bytes+stats.lost_bytes<stats.sent_bytes){
            break;
        }
        uint64_t window_lost=stats.lost_bytes;
        uint64_t accounted=stats.acked_bytes+stats.lost_bytes;
        if(final_flush && accounted<stats.sent_bytes){
            window_lost+=stats.sent_bytes-accounted;
        }
        m_cumSentBytes+=stats.sent_bytes;
        m_cumLostBytes+=window_lost;
        float loss_rate=(float)(100.0*window_lost/stats.sent_bytes);
        float cumulative_loss_rate=0.0f;
        if(m_cumSentBytes>0){
            cumulative_loss_rate=(float)(100.0*m_cumLostBytes/m_cumSentBytes);
        }
        double time_sec=(double)((it->first+1)*m_lossWindowIntervalUs)/1000000.0;
        if(final_flush && time_sec>now_sec){
            time_sec=now_sec;
        }
        m_traceLossRateCb(time_sec,loss_rate,cumulative_loss_rate);
        m_lossWindows.erase(it);
    }
}
void DqcSender::EnsureLossTraceHooked(){
    if(m_lossTraceHooked){
        return;
    }
    SendPacketManager *sent_manager=m_connection.GetSentPacketManager();
    if(sent_manager){
        sent_manager->SetTracePacketSent([this](PacketNumber seq,PacketLength bytes_sent,ProtoTime sent_ts){
            OnPacketSent(seq,sent_ts,bytes_sent);
        });
        sent_manager->SetTracePacketAcked([this](PacketNumber,PacketLength bytes_acked,ProtoTime sent_ts){
            OnPacketAcked(sent_ts,bytes_acked);
        });
        m_connection.SetTraceLossPacketDelay([this](PacketNumber seq,uint32_t rtt,PacketLength bytes_lost,ProtoTime sent_ts){
            OnPacketLossInfo(seq,rtt,bytes_lost,sent_ts);
        });
        m_lossTraceHooked=true;
    }
}
void DqcSender::EnsureRttTraceHooked(){
    if(m_rttTraceHooked){
        return;
    }
    SendPacketManager *sent_manager=m_connection.GetSentPacketManager();
    if(sent_manager){
        sent_manager->SetTraceRtt([this](PacketNumber largest_acked,
                                         uint32_t rtt_ms,
                                         uint32_t smoothed_rtt_ms){
            if(!m_traceRttCb.IsNull()){
                m_traceRttCb(static_cast<uint32_t>(largest_acked.ToUint64()),
                             rtt_ms,
                             smoothed_rtt_ms);
            }
        });
        m_rttTraceHooked=true;
    }
}
void DqcSender::Bind(uint16_t port){
    if (m_socket== NULL) {
        m_socket = Socket::CreateSocket (GetNode (),UdpSocketFactory::GetTypeId ());
        auto local = InetSocketAddress{Ipv4Address::GetAny (), port};
        auto res = m_socket->Bind (local);
        NS_ASSERT (res == 0);
    }
    m_bindPort=port;
    m_socket->SetRecvCallback (MakeCallback(&DqcSender::RecvPacket,this));
    if(m_ecn){
	m_socket->SetIpTos(0x01);
    }
	m_connection.set_packet_writer(&m_writer);
	m_connection.SetTraceSentSeq(this);
    m_stream=m_connection.GetOrCreateStream(m_streamId);
    m_stream->set_stream_vistor(this);
	SendPacketManager *sent_manager=m_connection.GetSentPacketManager();
    EnsureRttTraceHooked();

    // Set UP phase trace callback for FreqCCv3
    SendAlgorithmInterface* algo = sent_manager->GetSendAlgorithm();
    if(algo && algo->GetCongestionControlType() == kFreqCCv3){
        FreqCCv3Sender* freqccv3 = static_cast<FreqCCv3Sender*>(algo);
        freqccv3->SetQueueDelayTraceCallback([this](uint32_t queue_delay_ms,
                                                    uint32_t latest_rtt_ms,
                                                    uint32_t min_rtt_ms){
            if(!m_traceQueueDelayCb.IsNull()){
                m_traceQueueDelayCb(queue_delay_ms, latest_rtt_ms, min_rtt_ms);
            }
        });
        freqccv3->SetUpPhaseTraceCallback([this](double start_time, double duration_ms, double freq_hz, bool exit_due_to_queueing, int cycles, float pacing_gain, int32_t bw_kbps){
            if(!m_traceUpPhaseCb.IsNull()){
                m_traceUpPhaseCb(start_time, duration_ms, freq_hz, exit_due_to_queueing, cycles, pacing_gain, bw_kbps);
            }
        });
        // Set Freq Analysis Trace Callback
        // Parameters: (start_time_s, adopted_window_ms, sender_peak_freq_hz, receiver_peak_freq_hz, avg_rate_kbps)
        freqccv3->SetFreqAnalysisTraceCallback([this](double start_time, double adopted_window_ms, double sender_peak_freq_hz, double receiver_peak_freq_hz, int32_t avg_rate_kbps){
            if(!m_traceFreqAnalysisCb.IsNull()){
                m_traceFreqAnalysisCb(start_time, adopted_window_ms, sender_peak_freq_hz, receiver_peak_freq_hz, avg_rate_kbps);
            }
        });
        freqccv3->SetRttFreqAnalysisTraceCallback([this](double start_time, double adopted_window_ms, double sender_peak_freq_hz, double rtt_peak_freq_hz, double avg_smoothed_rtt_ms){
            if(!m_traceRttFreqAnalysisCb.IsNull()){
                m_traceRttFreqAnalysisCb(start_time, adopted_window_ms, sender_peak_freq_hz, rtt_peak_freq_hz, avg_smoothed_rtt_ms);
            }
        });
    } else if(algo && IsFBBRAlgorithm(algo->GetCongestionControlType())){
        FBBRSender* fbbr = static_cast<FBBRSender*>(algo);
        fbbr->SetTraceFlowId(m_id);
        fbbr->SetQueueDelayTraceCallback([this](uint32_t queue_delay_ms,
                                                    uint32_t latest_rtt_ms,
                                                    uint32_t min_rtt_ms){
            if(!m_traceQueueDelayCb.IsNull()){
                m_traceQueueDelayCb(queue_delay_ms, latest_rtt_ms, min_rtt_ms);
            }
        });
        fbbr->SetCruiseLoadTraceCallback([this](double window_start_s,
                                                    double window_end_s,
                                                    double p_underload,
                                                    double p_full_load,
                                                    double p_overload,
                                                    double confidence,
                                                    const std::string& label,
                                                    bool low_confidence,
                                                    const std::string& diagnostics){
            if(!m_traceFBBRLoadCb.IsNull()){
                m_traceFBBRLoadCb(window_start_s, window_end_s,
                                      p_underload, p_full_load, p_overload,
                                      confidence, label, low_confidence,
                                      diagnostics);
            }
        });
    } else if(algo && (algo->GetCongestionControlType() == kBBRv2 ||
                       algo->GetCongestionControlType() == kBBRv2Ecn ||
                       algo->GetCongestionControlType() == kBBRv2NoProbeRtt ||
                       algo->GetCongestionControlType() == kBBRv2Plus ||
                       algo->GetCongestionControlType() == kBBRv2PlusEcn)){
        Bbr2Sender* bbrv2 = static_cast<Bbr2Sender*>(algo);
        bbrv2->SetQueueDelayTraceCallback([this](uint32_t queue_delay_ms,
                                                 uint32_t latest_rtt_ms,
                                                 uint32_t min_rtt_ms){
            if(!m_traceQueueDelayCb.IsNull()){
                m_traceQueueDelayCb(queue_delay_ms, latest_rtt_ms, min_rtt_ms);
            }
        });
    }
}
InetSocketAddress DqcSender::GetLocalAddress(){
    Ptr<Node> node=GetNode();
    Ptr<Ipv4> ipv4 = node->GetObject<Ipv4> ();
    Ipv4Address local_ip = ipv4->GetAddress (1, 0).GetLocal ();
	return InetSocketAddress{local_ip,m_bindPort};
}
void DqcSender::ConfigurePeer(Ipv4Address addr,uint16_t port){
	m_peerIp=addr;
	m_peerPort=port;
	std::string ip_str;
	ConvertIp32(addr.Get(),ip_str);
	m_remote=SocketAddress(ip_str,port);
	NS_LOG_INFO(m_remote.ToString());
}
void DqcSender::DataGenerator(int times){
    if(!m_stream){
        return ;
    }
    char data[1500];
    int i=0;
    for (i=0;i<1500;i++){
        data[i]=RandomLetter::Instance()->GetLetter();
    }
    bool success=false;
    for(i=0;i<times;i++){
        if(m_pakcetLimit&&(m_packetGenerated>m_packetAllowed)){
            break;
        }
        uint32_t piece_size = 1500;
        if(m_dataChunkVariationBytes > 0){
            const uint32_t variation = std::min<uint32_t>(
                600, m_dataChunkVariationBytes);
            const uint64_t mixed =
                (static_cast<uint64_t>(m_id) + 1) * 0x9e3779b97f4a7c15ULL ^
                (static_cast<uint64_t>(m_packetGenerated) + 1) *
                    0xbf58476d1ce4e5b9ULL ^ m_dataChunkVariationSeed;
            const uint32_t span = 2 * variation + 1;
            const int32_t offset = static_cast<int32_t>(mixed % span) -
                                   static_cast<int32_t>(variation);
            piece_size = static_cast<uint32_t>(std::max<int32_t>(
                200, std::min<int32_t>(1500, 1400 + offset)));
        }
        std::string piece(data,piece_size);
        success=m_stream->WriteDataToBuffer(piece);
        if(!success){
            break;
        }
        m_packetGenerated++;
    }
}
void DqcSender::StartApplication(){
    m_running=true;
    DataGenerator(m_dataGeneratorBatch);
    if(m_enableEngineTimer){
        m_connection.SendInitData();
        UpdateEngineEvent();        
    }else{
        m_processTimer=Simulator::ScheduleNow(&DqcSender::Process,this);
    }

}
void DqcSender::StopApplication(){
    m_running=false;
	m_processTimer.Cancel();
	m_engineTimer.Cancel();
    FlushLossWindows(true);
    CloseEquivalenceAuditTrace();
    FinalizeCongestionControlTrace();
    m_sinks.clear();
}

void DqcSender::FinalizeCongestionControlTrace(){
    SendPacketManager* sent_manager = m_connection.GetSentPacketManager();
    SendAlgorithmInterface* algo =
        sent_manager ? sent_manager->GetSendAlgorithm() : nullptr;
    if (algo &&
        (algo->GetCongestionControlType() == kFBBR ||
         algo->GetCongestionControlType() == kFBBRServiceFair)) {
        static_cast<FBBRSender*>(algo)->FinalizeFbbrTrace();
    }
}
void DqcSender::RecvPacket(Ptr<Socket> socket){
    if(!m_running){return;}
	Address remoteAddr;
	auto p = socket->RecvFrom (remoteAddr);
	uint32_t recv=p->GetSize ();
    ProtoTime now=m_clock.Now();
    uint8_t buf[1500]={'\0'};
    p->CopyData(buf,recv);
    ProtoReceivedPacket packet((char*)buf,recv,now);
    m_connection.ProcessUdpPacket(m_self,m_remote,packet);
    PostProceeAfterReceiveFromPeer();
    if(m_enableEngineTimer){
        //when acked, new packet may be allowed to be sent out,so update callback timer;
        UpdateEngineEvent();        
    }
	
}
void DqcSender::SendToNetwork(Ptr<Packet> p){
	uint32_t ms=Simulator::Now().GetMilliSeconds();
	m_lastSentTs=ms;
	TimeTag tag;
    tag.SetSentTime (ms);
	p->AddPacketTag (tag);
    if (m_id != 0)
    {
        FlowIdTag flowTag;
        flowTag.SetFlowId(m_id);
        p->AddPacketTag(flowTag);
    }
	if(!m_traceBwCb.IsNull()){
		QuicBandwidth send_bw=m_connection.EstimatedBandwidth();
		SendPacketManager *sent_manager=m_connection.GetSentPacketManager();
		SendAlgorithmInterface* algo = sent_manager->GetSendAlgorithm();
		if(algo && IsBbr2StyleAlgorithm(algo->GetCongestionControlType())){
			Bbr2Sender* bbr2 = static_cast<Bbr2Sender*>(algo);
			send_bw = bbr2->ExportDebugState().bandwidth_hi;
		}
			m_traceBwCb((int32_t)send_bw.ToKBitsPerSecond());
		//NS_LOG_INFO("bw "<<std::to_string((int32_t)send_bw.ToKBitsPerSecond()));
	}
	    if(!m_traceQueueDelayCb.IsNull()){
	        SendPacketManager *sent_manager=m_connection.GetSentPacketManager();
	        SendAlgorithmInterface* algo = sent_manager->GetSendAlgorithm();
	        if(algo && algo->GetCongestionControlType() == kOBBR){
	            ObbrSender* bbr = static_cast<ObbrSender*>(algo);
	            const uint32_t latest_rtt_ms = sent_manager->GetRttStats()->latest_rtt().ToMilliseconds();
	            const uint32_t min_rtt_ms = bbr->GetMinRtt().ToMilliseconds();
	            const uint32_t queue_delay_ms = latest_rtt_ms > min_rtt_ms ? latest_rtt_ms - min_rtt_ms : 0;
	            m_traceQueueDelayCb(queue_delay_ms, latest_rtt_ms, min_rtt_ms);
	        } else if(algo && (algo->GetCongestionControlType() == kBBR ||
	                         algo->GetCongestionControlType() == kBBRR)){
	            ProtoBbrSender* bbr = static_cast<ProtoBbrSender*>(algo);
	            const uint32_t latest_rtt_ms = sent_manager->GetRttStats()->latest_rtt().ToMilliseconds();
	            const uint32_t min_rtt_ms = bbr->GetMinRtt().ToMilliseconds();
	            const uint32_t queue_delay_ms = latest_rtt_ms > min_rtt_ms ? latest_rtt_ms - min_rtt_ms : 0;
	            m_traceQueueDelayCb(queue_delay_ms, latest_rtt_ms, min_rtt_ms);
	        } else if(algo && algo->GetCongestionControlType() == kNs3Cubic){
	            const RttStats* rtt_stats = sent_manager->GetRttStats();
	            const uint32_t latest_rtt_ms = rtt_stats->latest_rtt().ToMilliseconds();
	            const uint32_t min_rtt_ms = rtt_stats->min_rtt().ToMilliseconds();
	            const uint32_t queue_delay_ms = latest_rtt_ms > min_rtt_ms ? latest_rtt_ms - min_rtt_ms : 0;
	            m_traceQueueDelayCb(queue_delay_ms, latest_rtt_ms, min_rtt_ms);
	        }
	    }
	    // Trace send rate (pacing rate)
	    if(!m_traceSendRateCb.IsNull()){
        SendPacketManager *sent_manager=m_connection.GetSentPacketManager();
        SendAlgorithmInterface* algo = sent_manager->GetSendAlgorithm();
        if(algo){
            ByteCount in_flight = 0;
            ByteCount cwnd = 0;
            sent_manager->InFlight(&in_flight, &cwnd);
            QuicBandwidth pacing_rate = algo->PacingRate(in_flight);
            m_traceSendRateCb((int32_t)pacing_rate.ToKBitsPerSecond());
        }
    }
    // Trace BBR mode
    if(!m_traceBbrModeCb.IsNull()){
        SendPacketManager *sent_manager=m_connection.GetSentPacketManager();
        SendAlgorithmInterface* algo = sent_manager->GetSendAlgorithm();
        if(algo && algo->GetCongestionControlType() == kFreqCC){
            FreqccSender* freqcc = static_cast<FreqccSender*>(algo);
            int32_t mode_index = freqcc->GetCurrentBbrModeIndex();
            m_traceBbrModeCb(mode_index);
        } else if(algo && algo->GetCongestionControlType() == kFreqCCv2){
            FreqCCv2Sender* freqccv2 = static_cast<FreqCCv2Sender*>(algo);
            int32_t mode_index = freqccv2->GetCurrentBbrModeIndex();
            m_traceBbrModeCb(mode_index);
        } else if(algo && algo->GetCongestionControlType() == kFreqCCv3){
            FreqCCv3Sender* freqccv3 = static_cast<FreqCCv3Sender*>(algo);
            int32_t mode_index = freqccv3->GetCurrentBbrModeIndex();
            m_traceBbrModeCb(mode_index);
	        } else if(algo && IsBbr2StyleAlgorithm(algo->GetCongestionControlType())){
	            Bbr2Sender* bbrv2 = static_cast<Bbr2Sender*>(algo);
	            m_traceBbrModeCb(bbrv2->GetCurrentBbrModeIndex());
	        } else if(algo && algo->GetCongestionControlType() == kOBBR){
	            ObbrSender* bbr = static_cast<ObbrSender*>(algo);
	            m_traceBbrModeCb(bbr->GetCurrentBbrModeIndex());
	        } else if(algo && (algo->GetCongestionControlType() == kBBR ||
	                         algo->GetCongestionControlType() == kBBRR)){
	            ProtoBbrSender* bbr = static_cast<ProtoBbrSender*>(algo);
	            m_traceBbrModeCb(ProtoBbrModeToTraceIndex(bbr->ExportDebugState().mode));
	        }
	    }
    // Trace inflight bytes and cwnd
    if(!m_traceInflightCb.IsNull()){
        SendPacketManager *sent_manager=m_connection.GetSentPacketManager();
        ByteCount in_flight = 0;
        ByteCount cwnd = 0;
        sent_manager->InFlight(&in_flight, &cwnd);
        m_traceInflightCb((int32_t)in_flight, (int32_t)cwnd);
    }
    m_socket->SendTo(p,0,InetSocketAddress{m_peerIp,m_peerPort});
}
void DqcSender::OnSent(dqc::PacketNumber seq,dqc::ProtoTime sent_ts) {
	if(!m_traceSentSeqCb.IsNull()){
		int32_t sent=(int32_t)seq.ToUint64();
		m_traceSentSeqCb(sent);
	}
    (void)sent_ts;
    // Packet-level pacer audit is emitted by the SendPacketManager callback
    // after pacing state has been updated; avoid a duplicate legacy row.
}
void DqcSender::Process(){
    if(!m_running){return ;}
    if(m_processTimer.IsExpired()){
    	CheckNoPacketOut();
    	ProtoTime now=m_clock.Now();
    	m_timeDriver.HeartBeat(now);
    	m_connection.Process();
        Time next=MicroSeconds(m_packetInteval);
        m_processTimer=Simulator::Schedule(next,&DqcSender::Process,this);
    }
}
void DqcSender::CheckNoPacketOut(){
    if(!m_running){return ;}
	int64_t now_ms=Simulator::Now().GetMilliSeconds();
	if(m_lastSentTs!=0){
		if((now_ms-m_lastSentTs)>5000){
			SendPacketManager *sent_manager=m_connection.GetSentPacketManager();
			int32_t largest_sent=(int32_t)(m_connection.GetMaxSentSeq().ToUint64());
			int32_t largest_acked=(int32_t)(sent_manager->largest_acked().ToUint64());
			int buffer=m_stream->BufferedBytes();
			ByteCount in_flight=0;
			ByteCount cwnd=0;
			sent_manager->InFlight(&in_flight,&cwnd);
			NS_LOG_ERROR(__FILE__<<std::to_string(largest_sent)<<" "<<std::to_string(largest_acked));
			NS_LOG_ERROR(std::to_string(buffer)
			<<" "<<std::to_string(m_stream->get_send_buffer_len())
			<<" "<<sent_manager->CheckCanSend()
			<<" "<<std::to_string(in_flight)
			<<" cwnd "<<std::to_string(cwnd)
			<<"fres "<<m_connection.GetFastRetrans()
			);
		}
	}
}
void DqcSender::EngineEvent(){
    if(!m_running){return ;}
	if(m_engineTimer.IsExpired()){
		UpdateEngineEvent();
		CheckNoPacketOut();
	}
}
void DqcSender::UpdateEngineEvent(){
    ProtoTime now=m_clock.Now();
    m_timeDriver.ExecuteCallback(now);
    ProtoTime nextEventTime=m_timeDriver.PeekNextEventTime();
	if(!m_engineTimer.IsExpired()){m_engineTimer.Cancel();}
    CHECK(nextEventTime!=ProtoTime::Infinite());
    CHECK(nextEventTime>=now);
    Time next=MicroSeconds((nextEventTime-now).ToMicroseconds());
    m_engineTimer=Simulator::Schedule(next,&DqcSender::EngineEvent,this);
}
void DqcSender::SetSenderId(uint32_t id){
    if(m_id!=0||id==0){
        return;
    }
    m_id=id;
}
void DqcSender::RegisterOnewayDelaySink(OneWayDelaySink *sink){
    bool existed=false;
    for(auto it=m_sinks.begin();it!=m_sinks.end();it++){
        if(sink==(*it)){
            existed=true;
            break;
        }
    }
    if(!existed){
        m_sinks.push_back(sink);
    }
}
void DqcSender::SetCongestionId(uint32_t cid){
    m_connection.SetThisCongestionId(cid);
}
void DqcSender::SetNumEmulatedConnections(int num_connections){
    m_connection.SetThisNumEmulatedConnections(num_connections);
}
void DqcSender::PostProceeAfterReceiveFromPeer(){
    std::pair<PacketNumber,TimeDelta> delay=m_connection.GetOneWayDelayInfo();
    bool newSample=false;
    if(m_lastAckedSeq==PacketNumber(0)){
        newSample=true;
    }else{
        if(delay.first>m_lastAckedSeq){
            newSample=true;
        }
    }
    if(newSample){
        m_lastAckedSeq=delay.first;
        uint32_t seq=(uint32_t)m_lastAckedSeq.ToUint64();
        uint32_t owd=(uint32_t)delay.second.ToMilliseconds();
        if(!m_traceOwd.IsNull()){
            m_traceOwd(seq,owd);
        }
        for(auto it=m_sinks.begin();it!=m_sinks.end();it++){
            (*it)->OnOneWayDelaySample(m_id,seq,owd);
        }
        if(!m_traceRecvRateCb.IsNull()){
            SendPacketManager *sent_manager=m_connection.GetSentPacketManager();
            SendAlgorithmInterface* algo = sent_manager->GetSendAlgorithm();
            if(algo){
                // BandwidthEstimate returns the estimated bandwidth (min of max_bw, bw_lo)
                QuicBandwidth bw_estimate = algo->BandwidthEstimate();
                // Get instant receive rate (bandwidth_latest) if available
                int32_t instant_kbps = 0;
                if(IsBbr2StyleAlgorithm(algo->GetCongestionControlType())){
                    Bbr2Sender* bbrv2 = static_cast<Bbr2Sender*>(algo);
                    instant_kbps = (int32_t)bbrv2->BandwidthLatest().ToKBitsPerSecond();
                } else if(algo->GetCongestionControlType() == kFreqCC){
                    FreqccSender* freqcc = static_cast<FreqccSender*>(algo);
                    instant_kbps = (int32_t)freqcc->BandwidthLatest().ToKBitsPerSecond();
                } else if(algo->GetCongestionControlType() == kFreqCCv2){
                    FreqCCv2Sender* freqccv2 = static_cast<FreqCCv2Sender*>(algo);
                    instant_kbps = (int32_t)freqccv2->BandwidthLatest().ToKBitsPerSecond();
                } else if(algo->GetCongestionControlType() == kFreqCCv3){
                    FreqCCv3Sender* freqccv3 = static_cast<FreqCCv3Sender*>(algo);
                    instant_kbps = (int32_t)freqccv3->BandwidthLatest().ToKBitsPerSecond();
                } else if(algo->GetCongestionControlType() == kOBBR){
                    ObbrSender* obbr = static_cast<ObbrSender*>(algo);
                    instant_kbps = (int32_t)obbr->BandwidthLatest().ToKBitsPerSecond();
                } else if(algo->GetCongestionControlType() == kBBR ||
                          algo->GetCongestionControlType() == kBBRR){
                    ProtoBbrSender* bbr = static_cast<ProtoBbrSender*>(algo);
                    instant_kbps =
                        (int32_t)bbr->BandwidthLatest().ToKBitsPerSecond();
                } else {
                    // Fallback to bandwidth estimate for other algorithms
                    instant_kbps = (int32_t)bw_estimate.ToKBitsPerSecond();
                }
                m_traceRecvRateCb(instant_kbps);
            }
        }
        if(!m_traceRecvRateRawCb.IsNull()){
            SendPacketManager *sent_manager=m_connection.GetSentPacketManager();
            SendAlgorithmInterface* algo = sent_manager->GetSendAlgorithm();
            if(algo){
                int32_t raw_kbps = 0;
                if(IsBbr2StyleAlgorithm(algo->GetCongestionControlType())){
                    Bbr2Sender* bbrv2 = static_cast<Bbr2Sender*>(algo);
                    raw_kbps = static_cast<int32_t>(bbrv2->DeliveryRateLatest().ToKBitsPerSecond());
                } else if(algo->GetCongestionControlType() == kOBBR){
                    ObbrSender* obbr = static_cast<ObbrSender*>(algo);
                    raw_kbps = static_cast<int32_t>(obbr->BandwidthLatest().ToKBitsPerSecond());
                } else if(algo->GetCongestionControlType() == kBBR ||
                          algo->GetCongestionControlType() == kBBRR){
                    ProtoBbrSender* bbr = static_cast<ProtoBbrSender*>(algo);
                    raw_kbps = static_cast<int32_t>(
                        bbr->BandwidthLatest().ToKBitsPerSecond());
                }
                if(raw_kbps > 0){
                    m_traceRecvRateRawCb(raw_kbps);
                }
            }
        }
    }
}
void DqcSender::SetFreqCCIntervalWindowMultiplier(double multiplier){
    SendPacketManager *sent_manager=m_connection.GetSentPacketManager();
    SendAlgorithmInterface* algo = sent_manager->GetSendAlgorithm();
    if(algo && algo->GetCongestionControlType() == kFreqCCv3){
        FreqCCv3Sender* freqccv3 = static_cast<FreqCCv3Sender*>(algo);
        freqccv3->SetIntervalWindowMultiplier(multiplier);
    }
}

void DqcSender::SetFreqCCMinProbeUpDurationRttMultiplier(double multiplier){
    SendPacketManager *sent_manager=m_connection.GetSentPacketManager();
    SendAlgorithmInterface* algo = sent_manager->GetSendAlgorithm();
    if(algo && algo->GetCongestionControlType() == kFreqCCv3){
        FreqCCv3Sender* freqccv3 = static_cast<FreqCCv3Sender*>(algo);
        freqccv3->SetMinProbeUpDurationRttMultiplier(multiplier);
    }
}

void DqcSender::SetFBBRFairShareBandwidth(uint64_t fair_share_bps){
    SendPacketManager *sent_manager=m_connection.GetSentPacketManager();
    SendAlgorithmInterface* algo = sent_manager->GetSendAlgorithm();
    if(algo && IsFBBRAlgorithm(algo->GetCongestionControlType())){
        FBBRSender* fbbr = static_cast<FBBRSender*>(algo);
        fbbr->SetFairShareBandwidthBps(fair_share_bps);
    }
}

bool DqcSender::GetBbr2ExperimentSnapshot(Bbr2ExperimentSnapshot *snapshot) const{
    if(snapshot == nullptr){
        return false;
    }
    SendPacketManager *sent_manager =
        const_cast<ProtoCon&>(m_connection).GetSentPacketManager();
    if(sent_manager == nullptr){
        return false;
    }
    SendAlgorithmInterface* algo = sent_manager->GetSendAlgorithm();
    if(algo == nullptr || !IsBbr2StyleAlgorithm(algo->GetCongestionControlType())){
        return false;
    }
    Bbr2Sender* bbrv2 = static_cast<Bbr2Sender*>(algo);
    Bbr2Sender::DebugState state = bbrv2->ExportDebugState();
    ByteCount in_flight = 0;
    ByteCount cwnd = 0;
    sent_manager->InFlight(&in_flight, &cwnd);

    snapshot->bbr_state = bbrv2->GetCurrentBbrModeIndex();
    snapshot->probe_phase = state.mode == Bbr2Mode::PROBE_BW
        ? Bbr2ProbePhaseName(state.probe_bw.phase)
        : Bbr2ProbePhaseName(Bbr2ProbeBwMode::CyclePhase::PROBE_NOT_STARTED);
    snapshot->pacing_gain = bbrv2->PacingGain();
    snapshot->pacing_rate_bps =
        static_cast<uint64_t>(algo->PacingRate(in_flight).ToBitsPerSecond());
    snapshot->max_bw_bps = static_cast<uint64_t>(std::max<int64_t>(
        0, state.bandwidth_hi.ToBitsPerSecond()));
    snapshot->delivery_rate_bps =
        static_cast<uint64_t>(bbrv2->DeliveryRateLatest().ToBitsPerSecond());
    snapshot->cwnd_bytes = static_cast<uint64_t>(cwnd);
    snapshot->inflight_bytes = static_cast<uint64_t>(in_flight);
    const RttStats *rtt_stats = sent_manager->GetRttStats();
    TimeDelta srtt = rtt_stats->smoothed_rtt();
    if(srtt.IsZero()){
        srtt = rtt_stats->SmoothedOrInitialRtt();
    }
    TimeDelta min_rtt = rtt_stats->min_rtt();
    if(min_rtt.IsZero()){
        min_rtt = state.min_rtt;
    }
    snapshot->srtt_us = static_cast<uint64_t>(srtt.ToMicroseconds());
    snapshot->min_rtt_us = static_cast<uint64_t>(min_rtt.ToMicroseconds());
    snapshot->delivered_bytes = static_cast<uint64_t>(bbrv2->TotalBytesAcked());
    snapshot->sent_bytes = static_cast<uint64_t>(bbrv2->TotalBytesSent());
    snapshot->acked_bytes = static_cast<uint64_t>(bbrv2->TotalBytesAcked());
    snapshot->lost_bytes = static_cast<uint64_t>(bbrv2->TotalBytesLost());
    snapshot->ecn_bytes_in_round =
        static_cast<uint64_t>(bbrv2->GetBytesEcnInRounds());
    snapshot->last_ack_time_s =
        static_cast<double>(bbrv2->LastAckEventTime().ToDebuggingValue()) /
        1000000.0;
    snapshot->probe_phase_start_time_s =
        static_cast<double>(state.probe_bw.phase_start_time.ToDebuggingValue()) /
        1000000.0;
    return true;
}

void DqcSender::SetBbr2ForcedProbeUp(double probe_up_time_s,
                                     double min_probe_up_duration_s){
    SendPacketManager *sent_manager=m_connection.GetSentPacketManager();
    SendAlgorithmInterface* algo = sent_manager->GetSendAlgorithm();
    if(algo && IsBbr2StyleAlgorithm(algo->GetCongestionControlType())){
        Bbr2Sender* bbrv2 = static_cast<Bbr2Sender*>(algo);
        bbrv2->SetExperimentalForcedProbeUp(
            ProtoTime::Zero() +
                TimeDelta::FromMicroseconds(
                    static_cast<int64_t>(probe_up_time_s * 1000000.0)),
            TimeDelta::FromMicroseconds(
                static_cast<int64_t>(min_probe_up_duration_s * 1000000.0)));
    }
}

void DqcSender::SetBbr2StrictProbeUp(uint32_t probe_order,
                                      uint32_t total_probe_orders,
                                      double probe_up_time_s,
                                      double min_probe_up_duration_s,
                                      double max_probe_up_duration_s){
    SendPacketManager *sent_manager=m_connection.GetSentPacketManager();
    SendAlgorithmInterface* algo = sent_manager->GetSendAlgorithm();
    if(algo && IsBbr2StyleAlgorithm(algo->GetCongestionControlType())){
        Bbr2Sender* bbrv2 = static_cast<Bbr2Sender*>(algo);
        const auto duration_from_seconds = [](double seconds) {
            return TimeDelta::FromMicroseconds(
                static_cast<int64_t>(std::max(0.0, seconds) * 1000000.0));
        };
        bbrv2->SetExperimentalStrictProbeUp(
            probe_order,
            total_probe_orders,
            ProtoTime::Zero() + duration_from_seconds(probe_up_time_s),
            duration_from_seconds(min_probe_up_duration_s),
            duration_from_seconds(max_probe_up_duration_s));
    }
}

void DqcSender::SetBbr2ExperimentPhaseTrace(
    Bbr2ExperimentPhaseTrace callback){
    SendPacketManager *sent_manager=m_connection.GetSentPacketManager();
    SendAlgorithmInterface* algo = sent_manager->GetSendAlgorithm();
    if(!algo || !IsBbr2StyleAlgorithm(algo->GetCongestionControlType())){
        return;
    }
    Bbr2Sender* bbrv2 = static_cast<Bbr2Sender*>(algo);
    if(!callback){
        bbrv2->SetExperimentProbePhaseTraceCallback(nullptr);
        return;
    }
    bbrv2->SetExperimentProbePhaseTraceCallback(
        [callback](Bbr2ProbeBwMode::CyclePhase phase, ProtoTime now){
            callback(static_cast<double>(now.ToDebuggingValue()) / 1000000.0,
                     Bbr2ProbePhaseName(phase));
        });
}

void DqcSender::SetBbr2MaxCongestionWindowPackets(uint32_t packets){
    SendPacketManager *sent_manager=m_connection.GetSentPacketManager();
    SendAlgorithmInterface* algo = sent_manager->GetSendAlgorithm();
    if(algo && IsBbr2StyleAlgorithm(algo->GetCongestionControlType())){
        Bbr2Sender* bbrv2 = static_cast<Bbr2Sender*>(algo);
        bbrv2->SetExperimentalMaxCongestionWindowPackets(packets);
    }
}

void DqcSender::SetStreamSendBufferBytes(uint32_t bytes){
    if(m_stream){
        m_stream->set_max_send_buf_len(bytes);
    }
}

void DqcSender::SetPacketLimitBytes(uint64_t bytes){
    if(bytes == 0){
        m_pakcetLimit = false;
        return;
    }
    const uint64_t packets = (bytes + 1499) / 1500;
    m_packetAllowed = static_cast<int>(std::min<uint64_t>(
        packets, static_cast<uint64_t>(std::numeric_limits<int>::max())));
    m_pakcetLimit = true;
}

void DqcSender::SetDataGeneratorBatch(uint32_t packets_per_fill){
    if(packets_per_fill > 0){
        m_dataGeneratorBatch = packets_per_fill;
    }
}

void DqcSender::SetProcessIntervalUs(int64_t interval_us){
    if(interval_us > 0){
        m_packetInteval = interval_us;
    }
}

void DqcSender::ConfigureFBBR(const dqc::FBBRConfig& config,
                                uint32_t flow_id){
    double freq_hz = config.default_modulation_freq_hz;
    double fixed_mbps = config.default_fixed_amplitude_mbps;
    auto it = config.flow.find(flow_id);
    if(it != config.flow.end()){
        if(it->second.has_modulation_freq_hz){
            freq_hz = it->second.modulation_freq_hz;
        }
        if(it->second.has_fixed_amplitude_mbps){
            fixed_mbps = it->second.fixed_amplitude_mbps;
        }
    }

    ConfigureFreqCC(freq_hz, config.default_amplitude_mode, fixed_mbps,
                    "after_drain", config.waveform_recv_signal_mode);

    SendPacketManager *sent_manager=m_connection.GetSentPacketManager();
    SendAlgorithmInterface* algo = sent_manager->GetSendAlgorithm();
    if(algo && IsFBBRAlgorithm(algo->GetCongestionControlType())){
        FBBRSender* fbbr = static_cast<FBBRSender*>(algo);
        fbbr->ConfigureFBBR(config);
    }
}

void DqcSender::ConfigureBbr2Plus(const dqc::Bbr2PlusConfig& config){
    SendPacketManager *sent_manager=m_connection.GetSentPacketManager();
    SendAlgorithmInterface* algo = sent_manager->GetSendAlgorithm();
    if(algo && (algo->GetCongestionControlType() == kBBRv2Plus ||
                algo->GetCongestionControlType() == kBBRv2PlusEcn)){
        Bbr2PlusSender* bbr2plus = static_cast<Bbr2PlusSender*>(algo);
        bbr2plus->Configure(config);
    }
}

void DqcSender::SetDataChunkVariationBytes(uint32_t variation_bytes,
                                           uint64_t variation_seed){
    m_dataChunkVariationBytes = variation_bytes;
    m_dataChunkVariationSeed = variation_seed;
}

void DqcSender::ConfigureFBBRConvergenceGate(
    bool enable_trace,
    bool enable_control,
    const std::string& gate_trace_mode,
    uint64_t gate_trace_sample_interval_us){
    SendPacketManager *sent_manager=m_connection.GetSentPacketManager();
    SendAlgorithmInterface* algo = sent_manager->GetSendAlgorithm();
    if(algo && IsFBBRAlgorithm(algo->GetCongestionControlType())){
        FBBRSender* fbbr = static_cast<FBBRSender*>(algo);
        fbbr->SetTraceFlowId(m_id);
        fbbr->SetConvergenceGateTraceEnabled(enable_trace);
        fbbr->SetConvergenceGateControlEnabled(enable_control);
        fbbr->SetGateTraceMode(
            ParseFBBRGateTraceMode(gate_trace_mode),
            gate_trace_sample_interval_us);
    }
}

void DqcSender::ConfigureFreqCC(double freq_hz, const std::string& amplitude_mode, double fixed_mbps, const std::string& osc_mode, const std::string& recv_signal_mode){
    SendPacketManager *sent_manager=m_connection.GetSentPacketManager();
    SendAlgorithmInterface* algo = sent_manager->GetSendAlgorithm();
    if(algo && algo->GetCongestionControlType() == kFreqCC){
        FreqccSender* freqcc = static_cast<FreqccSender*>(algo);

        // Set frequency
        freqcc->SetOscillationFrequency(freq_hz);

        // Set amplitude mode
        FreqAmplitudeMode amp_mode = FreqAmplitudeMode::kFixed;
        uint64_t fixed_bps = static_cast<uint64_t>(fixed_mbps * 1000000);

        if(amplitude_mode == "2miu" || amplitude_mode == "miu2"){
            amp_mode = FreqAmplitudeMode::kMiu2;
        } else if(amplitude_mode == "3miu" || amplitude_mode == "miu3"){
            amp_mode = FreqAmplitudeMode::kMiu3;
        } else if(amplitude_mode == "4miu" || amplitude_mode == "miu4"){
            amp_mode = FreqAmplitudeMode::kMiu4;
        } else if(amplitude_mode == "8miu" || amplitude_mode == "miu8"){
            amp_mode = FreqAmplitudeMode::kMiu8;
        } else if(amplitude_mode == "2sr" || amplitude_mode == "sr2"){
            amp_mode = FreqAmplitudeMode::kSR2;
        } else if(amplitude_mode == "3sr" || amplitude_mode == "sr3"){
            amp_mode = FreqAmplitudeMode::kSR3;
        } else if(amplitude_mode == "4sr" || amplitude_mode == "sr4"){
            amp_mode = FreqAmplitudeMode::kSR4;
        } else if(amplitude_mode == "8sr" || amplitude_mode == "sr8"){
            amp_mode = FreqAmplitudeMode::kSR8;
        } else {
            // Default to fixed mode
            // If amplitude_mode is "0" or empty, use fixed_mbps parameter
            // Otherwise try to parse amplitude_mode as a number (Mbps)
            amp_mode = FreqAmplitudeMode::kFixed;
            if(amplitude_mode != "0" && !amplitude_mode.empty()) {
                try {
                    double val = std::stod(amplitude_mode);
                    if(val > 0) {
                        fixed_bps = static_cast<uint64_t>(val * 1000000);
                    }
                    // If val == 0, keep fixed_bps from fixed_mbps parameter
                } catch(...) {
                    // Keep fixed_bps from parameter
                }
            }
            // Otherwise use fixed_bps which was already set from fixed_mbps
        }
        freqcc->SetOscillationAmplitude(amp_mode, fixed_bps);

        // Set oscillation mode
        if(osc_mode == "only_probeBW" || osc_mode == "only_probebw"){
            freqcc->SetOscillationMode(FreqOscillationMode::kOnlyProbeBW);
        } else {
            freqcc->SetOscillationMode(FreqOscillationMode::kAfterDrain);
        }
    } else if(algo && algo->GetCongestionControlType() == kFreqCCv2){
        FreqCCv2Sender* freqccv2 = static_cast<FreqCCv2Sender*>(algo);

        // Set frequency
        freqccv2->SetOscillationFrequency(freq_hz);

        // Set amplitude mode
        FreqCCv2AmplitudeMode amp_mode = FreqCCv2AmplitudeMode::kFixed;
        uint64_t fixed_bps = static_cast<uint64_t>(fixed_mbps * 1000000);

        if(amplitude_mode == "2miu" || amplitude_mode == "miu2"){
            amp_mode = FreqCCv2AmplitudeMode::kMiu2;
        } else if(amplitude_mode == "3miu" || amplitude_mode == "miu3"){
            amp_mode = FreqCCv2AmplitudeMode::kMiu3;
        } else if(amplitude_mode == "4miu" || amplitude_mode == "miu4"){
            amp_mode = FreqCCv2AmplitudeMode::kMiu4;
        } else if(amplitude_mode == "8miu" || amplitude_mode == "miu8"){
            amp_mode = FreqCCv2AmplitudeMode::kMiu8;
        } else if(amplitude_mode == "2sr" || amplitude_mode == "sr2"){
            amp_mode = FreqCCv2AmplitudeMode::kSR2;
        } else if(amplitude_mode == "3sr" || amplitude_mode == "sr3"){
            amp_mode = FreqCCv2AmplitudeMode::kSR3;
        } else if(amplitude_mode == "4sr" || amplitude_mode == "sr4"){
            amp_mode = FreqCCv2AmplitudeMode::kSR4;
        } else if(amplitude_mode == "8sr" || amplitude_mode == "sr8"){
            amp_mode = FreqCCv2AmplitudeMode::kSR8;
        } else {
            // Default to fixed mode
            amp_mode = FreqCCv2AmplitudeMode::kFixed;
            if(amplitude_mode != "0" && !amplitude_mode.empty()) {
                try {
                    double val = std::stod(amplitude_mode);
                    if(val > 0) {
                        fixed_bps = static_cast<uint64_t>(val * 1000000);
                    }
                } catch(...) {
                    // Keep fixed_bps from parameter
                }
            }
        }
        freqccv2->SetOscillationAmplitude(amp_mode, fixed_bps);

        // Set oscillation mode (FreqCCv2 supports additional refill_up mode)
        if(osc_mode == "only_probeBW" || osc_mode == "only_probebw"){
            freqccv2->SetOscillationMode(FreqCCv2OscillationMode::kOnlyProbeBW);
        } else if(osc_mode == "refill_up" || osc_mode == "refillup"){
            freqccv2->SetOscillationMode(FreqCCv2OscillationMode::kRefillUp);
        } else {
            freqccv2->SetOscillationMode(FreqCCv2OscillationMode::kAfterDrain);
        }
    } else if(algo && algo->GetCongestionControlType() == kFreqCCv3){
        FreqCCv3Sender* freqccv3 = static_cast<FreqCCv3Sender*>(algo);

        // Set frequency
        freqccv3->SetOscillationFrequency(freq_hz);

        // Set amplitude mode (only used during PROBE_UP)
        FreqCCv3AmplitudeMode amp_mode = FreqCCv3AmplitudeMode::kFixed;
        uint64_t fixed_bps = static_cast<uint64_t>(fixed_mbps * 1000000);

        if(amplitude_mode == "2miu" || amplitude_mode == "miu2"){
            amp_mode = FreqCCv3AmplitudeMode::kMiu2;
        } else if(amplitude_mode == "3miu" || amplitude_mode == "miu3"){
            amp_mode = FreqCCv3AmplitudeMode::kMiu3;
        } else if(amplitude_mode == "4miu" || amplitude_mode == "miu4"){
            amp_mode = FreqCCv3AmplitudeMode::kMiu4;
        } else if(amplitude_mode == "8miu" || amplitude_mode == "miu8"){
            amp_mode = FreqCCv3AmplitudeMode::kMiu8;
        } else if(amplitude_mode == "2sr" || amplitude_mode == "sr2"){
            amp_mode = FreqCCv3AmplitudeMode::kSR2;
        } else if(amplitude_mode == "3sr" || amplitude_mode == "sr3"){
            amp_mode = FreqCCv3AmplitudeMode::kSR3;
        } else if(amplitude_mode == "4sr" || amplitude_mode == "sr4"){
            amp_mode = FreqCCv3AmplitudeMode::kSR4;
        } else if(amplitude_mode == "8sr" || amplitude_mode == "sr8"){
            amp_mode = FreqCCv3AmplitudeMode::kSR8;
        } else {
            // Default to fixed mode
            amp_mode = FreqCCv3AmplitudeMode::kFixed;
            if(amplitude_mode != "0" && !amplitude_mode.empty()) {
                try {
                    double val = std::stod(amplitude_mode);
                    if(val > 0) {
                        fixed_bps = static_cast<uint64_t>(val * 1000000);
                    }
                } catch(...) {
                    // Keep fixed_bps from parameter
                }
            }
        }
        freqccv3->SetOscillationAmplitude(amp_mode, fixed_bps);
        bool use_delivery_rate_latest =
            recv_signal_mode == "delivery_rate_latest" ||
            recv_signal_mode == "delivery_latest" ||
            recv_signal_mode == "raw_delivery" ||
            recv_signal_mode == "recvrate_raw";
        freqccv3->SetRecvSignalMode(use_delivery_rate_latest);
    } else if(algo && IsFBBRAlgorithm(algo->GetCongestionControlType())){
        FBBRSender* fbbr = static_cast<FBBRSender*>(algo);
        fbbr->SetOscillationFrequency(freq_hz);

        FBBRAmplitudeMode amp_mode = FBBRAmplitudeMode::kFixed;
        uint64_t fixed_bps = static_cast<uint64_t>(fixed_mbps * 1000000);

        if(amplitude_mode == "2miu" || amplitude_mode == "miu2"){
            amp_mode = FBBRAmplitudeMode::kMiu2;
        } else if(amplitude_mode == "3miu" || amplitude_mode == "miu3"){
            amp_mode = FBBRAmplitudeMode::kMiu3;
        } else if(amplitude_mode == "4miu" || amplitude_mode == "miu4"){
            amp_mode = FBBRAmplitudeMode::kMiu4;
        } else if(amplitude_mode == "8miu" || amplitude_mode == "miu8"){
            amp_mode = FBBRAmplitudeMode::kMiu8;
        } else if(amplitude_mode == "2sr" || amplitude_mode == "sr2"){
            amp_mode = FBBRAmplitudeMode::kSR2;
        } else if(amplitude_mode == "3sr" || amplitude_mode == "sr3"){
            amp_mode = FBBRAmplitudeMode::kSR3;
        } else if(amplitude_mode == "4sr" || amplitude_mode == "sr4"){
            amp_mode = FBBRAmplitudeMode::kSR4;
        } else if(amplitude_mode == "8sr" || amplitude_mode == "sr8"){
            amp_mode = FBBRAmplitudeMode::kSR8;
        } else if(amplitude_mode == "12sr" || amplitude_mode == "sr12"){
            amp_mode = FBBRAmplitudeMode::kSR12;
        } else if(amplitude_mode == "16sr" || amplitude_mode == "sr16"){
            amp_mode = FBBRAmplitudeMode::kSR16;
        } else {
            amp_mode = FBBRAmplitudeMode::kFixed;
            if(amplitude_mode != "0" && !amplitude_mode.empty()) {
                try {
                    double val = std::stod(amplitude_mode);
                    if(val > 0) {
                        fixed_bps = static_cast<uint64_t>(val * 1000000);
                    }
                } catch(...) {
                }
            }
        }
        fbbr->SetOscillationAmplitude(amp_mode, fixed_bps);
        bool use_delivery_rate_latest =
            recv_signal_mode == "delivery_rate_latest" ||
            recv_signal_mode == "delivery_latest" ||
            recv_signal_mode == "raw_delivery" ||
            recv_signal_mode == "recvrate_raw";
        fbbr->SetRecvSignalMode(use_delivery_rate_latest);
    }
}
}  // namespace ns3
