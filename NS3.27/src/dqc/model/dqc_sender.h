#pragma once
#include <memory>
#include <map>
#include <vector>
#include "ns3/event-id.h"
#include "ns3/callback.h"
#include "ns3/application.h"
#include "ns3/socket.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/dqc_clock.h"
#include "ns3/proto_stream.h"
#include "ns3/proto_bandwidth.h"
#include "ns3/proto_time.h"
#include "ns3/packet_number.h"
#include "ns3/socket_address.h"
#include "ns3/proto_socket.h"
#include "ns3/process_alarm_factory.h"
#include "ns3/proto_con.h"
#include "ns3/dqc_trace.h"
namespace ns3{
class DqcSender;
class OneWayDelaySink{
public:
    virtual ~OneWayDelaySink(){}
    virtual bool NeedRegisterToSender(uint32_t id){ return false;}
    virtual void OnOneWayDelaySample(uint32_t id,uint32_t seq,uint32_t owd){}
};
class FakePackeWriter:public dqc::Socket{
public:
	FakePackeWriter(DqcSender *sender):m_sender(sender){}
	~FakePackeWriter(){}
	int SendTo(const char*buf,size_t size,dqc::SocketAddress &dst) override;
private:
	DqcSender *m_sender{nullptr};
};
class DqcSender: public Application,
public dqc::ProtoStream::StreamCanWriteVisitor,
public dqc::ProtoCon::TraceSentSeq{
public:
    DqcSender(bool ecn=false);
    DqcSender(dqc::CongestionControlType cc_type,bool ecn=false,bool engine_time=true); 
    ~DqcSender(){}
    void SetMaxBandwidth(uint32_t bps);
    typedef Callback<void,int32_t> TraceBandwidth;
    void SetBwTraceFuc(TraceBandwidth cb);
    typedef Callback<void,int32_t> TraceSentSeq;
    void SetSentSeqTraceFuc(TraceSentSeq cb);
    typedef Callback<void,uint32_t,uint32_t,uint32_t> TraceLossPacketDelay;
    void SetTraceLossPacketDelay(TraceLossPacketDelay cb);
    typedef Callback<void,double,float,float> TraceLossRate;
    void SetLossRateTraceFuc(TraceLossRate cb);
    typedef Callback<void,uint32_t,uint32_t> TraceOwdAtSender;
    void SetTraceOwdAtSender(TraceOwdAtSender cb);
    typedef Callback<void,uint32_t,uint32_t,uint32_t> TraceRtt;
    void SetRttTraceFuc(TraceRtt cb);
    typedef Callback<void,int32_t> TraceSendRate;
    void SetSendRateTraceFuc(TraceSendRate cb);
    typedef Callback<void,int32_t,int32_t> TraceRecvRate;
    void SetRecvRateTraceFuc(TraceRecvRate cb);
    typedef Callback<void,int32_t,int32_t> TraceInflight;
    void SetInflightTraceFuc(TraceInflight cb);
    typedef Callback<void,int32_t> TraceBbrMode;
    void SetBbrModeTraceFuc(TraceBbrMode cb);
    typedef Callback<void,double,double,double,bool,int,float,int32_t> TraceUpPhase;
    void SetUpPhaseTraceFuc(TraceUpPhase cb);
    typedef Callback<void,double,double,double,double,int32_t> TraceFreqAnalysis;
    void SetFreqAnalysisTraceFuc(TraceFreqAnalysis cb);
    typedef Callback<void,const AckEventRecord &> TraceAckEvent;
    void SetAckEventTraceFuc(TraceAckEvent cb);
    typedef Callback<void,const AckEpisodeRecord &> TraceAckEpisode;
    void SetAckEpisodeTraceFuc(TraceAckEpisode cb);
    void Bind(uint16_t port);
    InetSocketAddress GetLocalAddress();
    void ConfigurePeer(Ipv4Address addr,uint16_t port);    
    void OnCanWrite() override{
        DataGenerator(2);
    }
    void SendToNetwork(Ptr<Packet> p);
    void OnSent(dqc::PacketNumber seq,dqc::ProtoTime sent_ts) override;
    void OnPacketLossInfo(dqc::PacketNumber seq,uint32_t rtt,uint32_t bytes_lost,dqc::ProtoTime sent_ts);
    uint32_t GetId() const {return m_id;}
    void SetSenderId(uint32_t id);
    void RegisterOnewayDelaySink(OneWayDelaySink *sink);
    void SetCongestionId(uint32_t cid);
	void SetNumEmulatedConnections(int num_connections);
    // FreqCC configuration methods
    void ConfigureFreqCC(double freq_hz, const std::string& amplitude_mode, double fixed_mbps=0.0, const std::string& osc_mode="after_drain");
private:
	void DataGenerator(int times);
	virtual void StartApplication() override;
	virtual void StopApplication() override;
    void RecvPacket(Ptr<Socket> socket);
    void Process();
    void CheckNoPacketOut();
    void EngineEvent();
    void UpdateEngineEvent();
    void OnPacketSent(dqc::ProtoTime sent_ts,uint32_t bytes_sent);
    void OnPacketAcked(dqc::ProtoTime sent_ts,uint32_t bytes_acked);
    void FlushLossWindows(bool final_flush);
    uint64_t GetLossWindowId(dqc::ProtoTime sent_ts) const;
    void EnsureLossTraceHooked();
    void EnsureAckTraceHooked();
    void OnAckEventInternal(dqc::ProtoTime ack_receive_time,uint64_t acked_bytes,
                            uint32_t acked_pkts,dqc::PacketNumber largest_acked,
                            uint32_t ack_delay_ms,uint32_t rtt_ms);
    void StartAckEpisode(int32_t type,int64_t start_us,double ack_interval_ms,
                         uint64_t acked_bytes,double ack_rate_kbps,
                         double pacing_rate_kbps,double sample_bias);
    void UpdateAckEpisode(int64_t ack_us,double ack_interval_ms,uint64_t acked_bytes,
                          double ack_rate_kbps,double pacing_rate_kbps,double sample_bias);
    void EmitAckEpisode(int64_t end_us);
    void PostProceeAfterReceiveFromPeer();
    bool m_ecn{false};
    bool m_running{false};
    uint32_t m_id{0};
    FakePackeWriter m_writer;
    Ipv4Address m_peerIp;
    uint16_t m_peerPort;
    uint16_t m_bindPort;
    dqc::SocketAddress m_self;
    dqc::SocketAddress m_remote;
    uint32_t m_streamId{0};
    Ptr<Socket> m_socket;
    DqcSimuClock m_clock;
    dqc::MainEngine m_timeDriver;
    std::shared_ptr<dqc::AlarmFactory> m_alarmFactory;
    dqc::ProtoCon m_connection;
    dqc::ProtoStream *m_stream{nullptr};
    bool m_enableEngineTimer{false};
    EventId m_engineTimer;
    EventId m_processTimer;
    int64_t m_packetInteval{100};//0.5 ms
    int m_packetGenerated{0};
	bool m_pakcetLimit{false};
    int m_packetAllowed{50000};
    std::vector<OneWayDelaySink*> m_sinks;
	TraceBandwidth m_traceBwCb;
	int64_t m_lastSentTs{0};
    dqc::PacketNumber m_lastAckedSeq{dqc::PacketNumber(0)};
	TraceSentSeq m_traceSentSeqCb;
    TraceLossPacketDelay m_traceLossDelay;
    TraceOwdAtSender m_traceOwd;
    TraceRtt m_traceRttCb;
    TraceSendRate m_traceSendRateCb;
    TraceRecvRate m_traceRecvRateCb;
    TraceInflight m_traceInflightCb;
    TraceBbrMode m_traceBbrModeCb;
    TraceUpPhase m_traceUpPhaseCb;
    TraceFreqAnalysis m_traceFreqAnalysisCb;
    TraceLossRate m_traceLossRateCb;
    TraceAckEvent m_traceAckEventCb;
    TraceAckEpisode m_traceAckEpisodeCb;
    bool m_lossTraceHooked{false};
    bool m_ackTraceHooked{false};
    struct LossWindowStats{
        uint64_t sent_bytes{0};
        uint64_t acked_bytes{0};
        uint64_t lost_bytes{0};
    };
    std::map<uint64_t,LossWindowStats> m_lossWindows;
    uint64_t m_lossWindowIntervalUs{100000};
    uint64_t m_cumSentBytes{0};
    uint64_t m_cumLostBytes{0};

    bool m_ackBaselineInitialized{false};
    double m_ackBaselineIatMs{0.0};
    double m_ackBaselineBytes{0.0};
    double m_ackBaselineRateKbps{0.0};
    double m_ackEwmaAlpha{0.1};
    double m_ackCompIatFactor{0.5};
    double m_ackCompRateFactor{1.5};
    double m_ackAggIatFactor{2.0};
    double m_ackAggBytesFactor{2.0};
    int m_ackEpisodeNonMatchLimit{2};
    dqc::ProtoTime m_lastAckRecvTime{dqc::ProtoTime::Zero()};
    int32_t m_ackEpisodeType{-1};
    int m_ackEpisodeNonMatchCount{0};
    int64_t m_ackEpisodeStartUs{0};
    int64_t m_ackEpisodeLastMatchUs{0};
    uint32_t m_ackEpisodeEvents{0};
    uint64_t m_ackEpisodeBytes{0};
    double m_ackEpisodeIatMinMs{0.0};
    double m_ackEpisodeIatMaxMs{0.0};
    double m_ackEpisodeAckRatePeakKbps{0.0};
    double m_ackEpisodePacingRateSumKbps{0.0};
    uint32_t m_ackEpisodePacingRateCount{0};
    double m_ackEpisodeBiasPeak{0.0};
};
}
