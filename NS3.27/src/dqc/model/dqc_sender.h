#pragma once
#include <memory>
#include <map>
#include <vector>
#include <string>
#include <fstream>
#include <functional>
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
namespace dqc{
struct FBBRConfig;
struct Bbr2PlusConfig;
}
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
    typedef Callback<void,uint32_t,uint32_t,uint32_t> TraceQueueDelay;
    void SetQueueDelayTraceFuc(TraceQueueDelay cb);
    typedef Callback<void,int32_t> TraceSendRate;
    void SetSendRateTraceFuc(TraceSendRate cb);
    typedef Callback<void,int32_t> TraceRecvRate;
    void SetRecvRateTraceFuc(TraceRecvRate cb);
    typedef Callback<void,int32_t> TraceRecvRateRaw;
    void SetRecvRateRawTraceFuc(TraceRecvRateRaw cb);
    typedef Callback<void,int32_t,int32_t> TraceInflight;
    void SetInflightTraceFuc(TraceInflight cb);
    typedef Callback<void,int32_t> TraceBbrMode;
    void SetBbrModeTraceFuc(TraceBbrMode cb);
    typedef Callback<void,double,double,double,bool,int,float,int32_t> TraceUpPhase;
    void SetUpPhaseTraceFuc(TraceUpPhase cb);
    typedef Callback<void,double,double,double,double,int32_t> TraceFreqAnalysis;
    void SetFreqAnalysisTraceFuc(TraceFreqAnalysis cb);
    typedef Callback<void,double,double,double,double,double> TraceRttFreqAnalysis;
    void SetRttFreqAnalysisTraceFuc(TraceRttFreqAnalysis cb);
    typedef Callback<void,double,double,double,double,double,double,std::string,bool,std::string> TraceFBBRLoad;
    void SetFBBRLoadTraceFuc(TraceFBBRLoad cb);
    void Bind(uint16_t port);
    InetSocketAddress GetLocalAddress();
    void ConfigurePeer(Ipv4Address addr,uint16_t port);    
    void OnCanWrite() override{
        DataGenerator(m_dataGeneratorBatch);
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
	    void ConfigureFreqCC(double freq_hz, const std::string& amplitude_mode, double fixed_mbps=0.0, const std::string& osc_mode="after_drain", const std::string& recv_signal_mode="delivery_rate_latest");
	    void ConfigureFBBR(const dqc::FBBRConfig& config, uint32_t flow_id);
	    void ConfigureBbr2Plus(const dqc::Bbr2PlusConfig& config);
	    void ConfigureFBBRConvergenceGate(bool enable_trace,
	                                          bool enable_control,
	                                          const std::string& gate_trace_mode="round_only",
	                                          uint64_t gate_trace_sample_interval_us=1000);
    void SetFreqCCIntervalWindowMultiplier(double multiplier);
    void SetFreqCCMinProbeUpDurationRttMultiplier(double multiplier);
    void SetFBBRFairShareBandwidth(uint64_t fair_share_bps);
    struct Bbr2ExperimentSnapshot {
        int32_t bbr_state{0};
        std::string probe_phase;
        double pacing_gain{0.0};
        uint64_t pacing_rate_bps{0};
        uint64_t max_bw_bps{0};
        uint64_t delivery_rate_bps{0};
        uint64_t cwnd_bytes{0};
        uint64_t inflight_bytes{0};
        uint64_t srtt_us{0};
        uint64_t min_rtt_us{0};
        uint64_t delivered_bytes{0};
        uint64_t sent_bytes{0};
        uint64_t acked_bytes{0};
        uint64_t lost_bytes{0};
        uint64_t ecn_bytes_in_round{0};
        double last_ack_time_s{0.0};
        double probe_phase_start_time_s{0.0};
    };
    bool GetBbr2ExperimentSnapshot(Bbr2ExperimentSnapshot *snapshot) const;
    void SetBbr2ForcedProbeUp(double probe_up_time_s,
                              double min_probe_up_duration_s);
    void SetBbr2StrictProbeUp(uint32_t probe_order,
                              uint32_t total_probe_orders,
                              double probe_up_time_s,
                              double min_probe_up_duration_s,
                              double max_probe_up_duration_s);
    using Bbr2ExperimentPhaseTrace =
        std::function<void(double, const std::string&)>;
    void SetBbr2ExperimentPhaseTrace(Bbr2ExperimentPhaseTrace callback);
    void SetBbr2MaxCongestionWindowPackets(uint32_t packets);
    void SetStreamSendBufferBytes(uint32_t bytes);
    void SetPacketLimitBytes(uint64_t bytes);
    void SetDataGeneratorBatch(uint32_t packets_per_fill);
    void FinalizeCongestionControlTrace();
    void SetDataChunkVariationBytes(uint32_t variation_bytes,
                                    uint64_t variation_seed = 0);
    void SetProcessIntervalUs(int64_t interval_us);
    void SetEquivalenceAuditTracePrefix(const std::string& prefix);
private:
	void DataGenerator(int times);
	virtual void StartApplication() override;
	virtual void StopApplication() override;
    void RecvPacket(Ptr<Socket> socket);
    void Process();
    void CheckNoPacketOut();
    void EngineEvent();
    void UpdateEngineEvent();
    void OnPacketSent(dqc::PacketNumber seq,dqc::ProtoTime sent_ts,uint32_t bytes_sent);
    void OnPacketAcked(dqc::ProtoTime sent_ts,uint32_t bytes_acked);
    void FlushLossWindows(bool final_flush);
    uint64_t GetLossWindowId(dqc::ProtoTime sent_ts) const;
    void EnsureLossTraceHooked();
    void EnsureRttTraceHooked();
    void CloseEquivalenceAuditTrace();
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
    uint32_t m_dataGeneratorBatch{2};
    uint32_t m_dataChunkVariationBytes{0};
    uint64_t m_dataChunkVariationSeed{0};
    std::vector<OneWayDelaySink*> m_sinks;
	TraceBandwidth m_traceBwCb;
	int64_t m_lastSentTs{0};
    dqc::PacketNumber m_lastAckedSeq{dqc::PacketNumber(0)};
	TraceSentSeq m_traceSentSeqCb;
    TraceLossPacketDelay m_traceLossDelay;
    TraceOwdAtSender m_traceOwd;
    TraceRtt m_traceRttCb;
    TraceQueueDelay m_traceQueueDelayCb;
    TraceSendRate m_traceSendRateCb;
    TraceRecvRate m_traceRecvRateCb;
    TraceRecvRateRaw m_traceRecvRateRawCb;
    TraceInflight m_traceInflightCb;
    TraceBbrMode m_traceBbrModeCb;
    TraceUpPhase m_traceUpPhaseCb;
    TraceFreqAnalysis m_traceFreqAnalysisCb;
    TraceRttFreqAnalysis m_traceRttFreqAnalysisCb;
    TraceFBBRLoad m_traceFBBRLoadCb;
    TraceLossRate m_traceLossRateCb;
    std::string m_equivalenceAuditPrefix;
    std::fstream m_equivalenceSent;
    std::fstream m_equivalenceAcked;
    std::fstream m_equivalencePacing;
    uint64_t m_equivalenceSentBytes{0};
    uint64_t m_equivalenceAckedBytes{0};
    bool m_lossTraceHooked{false};
    bool m_rttTraceHooked{false};
    struct LossWindowStats{
        uint64_t sent_bytes{0};
        uint64_t acked_bytes{0};
        uint64_t lost_bytes{0};
    };
    std::map<uint64_t,LossWindowStats> m_lossWindows;
    uint64_t m_lossWindowIntervalUs{100000};
    uint64_t m_cumSentBytes{0};
    uint64_t m_cumLostBytes{0};
};
}
