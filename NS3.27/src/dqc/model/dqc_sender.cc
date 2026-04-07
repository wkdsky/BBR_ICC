#include <algorithm>
#include <string>
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
           type == kFreqCCv3;
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
void DqcSender::OnPacketLossInfo(dqc::PacketNumber seq,uint32_t rtt,uint32_t bytes_lost,dqc::ProtoTime sent_ts){
    uint64_t window_id=GetLossWindowId(sent_ts);
    m_lossWindows[window_id].lost_bytes+=bytes_lost;
    FlushLossWindows(false);
    if(!m_traceLossDelay.IsNull()){
        int32_t num=(int32_t)seq.ToUint64();
        m_traceLossDelay(num,rtt,bytes_lost);
    }
}
void DqcSender::OnPacketSent(dqc::ProtoTime sent_ts,uint32_t bytes_sent){
    uint64_t window_id=GetLossWindowId(sent_ts);
    m_lossWindows[window_id].sent_bytes+=bytes_sent;
}
void DqcSender::OnPacketAcked(dqc::ProtoTime sent_ts,uint32_t bytes_acked){
    uint64_t window_id=GetLossWindowId(sent_ts);
    m_lossWindows[window_id].acked_bytes+=bytes_acked;
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
        sent_manager->SetTracePacketSent([this](PacketNumber,PacketLength bytes_sent,ProtoTime sent_ts){
            OnPacketSent(sent_ts,bytes_sent);
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
    std::string piece(data,1500);
    bool success=false;
    for(i=0;i<times;i++){
        if(m_pakcetLimit&&(m_packetGenerated>m_packetAllowed)){
            break;
        }
        success=m_stream->WriteDataToBuffer(piece);
        if(!success){
            break;
        }
        m_packetGenerated++;
    }
}
void DqcSender::StartApplication(){
    m_running=true;
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
    m_sinks.clear();
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
	        } else if(algo && algo->GetCongestionControlType() == kBBR){
	            ProtoBbrSender* bbr = static_cast<ProtoBbrSender*>(algo);
	            const uint32_t latest_rtt_ms = sent_manager->GetRttStats()->latest_rtt().ToMilliseconds();
	            const uint32_t min_rtt_ms = bbr->GetMinRtt().ToMilliseconds();
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
	        } else if(algo && algo->GetCongestionControlType() == kBBR){
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
                } else {
                    // Fallback to bandwidth estimate for other algorithms
                    instant_kbps = (int32_t)bw_estimate.ToKBitsPerSecond();
                }
                m_traceRecvRateCb(instant_kbps);
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

void DqcSender::ConfigureFreqCC(double freq_hz, const std::string& amplitude_mode, double fixed_mbps, const std::string& osc_mode){
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
        // Note: FreqCCv3 only oscillates during PROBE_UP, no oscillation mode setting needed
    }
}
}  // namespace ns3
