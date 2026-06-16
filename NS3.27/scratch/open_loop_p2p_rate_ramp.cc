/**
 * Open-loop P2P rate-ramp experiment without congestion control.
 *
 * Sender target rate:
 *   linear baseline: 220 Mbps -> 450 Mbps over 5 s
 *   triangle modulation: same waveform as FreqCCv3 CalculateOscillationOffset()
 *   modulation frequency: 5 Hz
 *   modulation peak amplitude: 80 Mbps
 *
 * Topology:
 *   n0 ---- 350 Mbps, 20 ms one-way delay ---- n1
 *   device queue on n0 is one BDP by default, using RTT = 2 * one-way delay.
 */

#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/flow-id-tag.h"
#include "ns3/internet-module.h"
#include "ns3/ipv4-global-routing-helper.h"
#include "ns3/network-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/traffic-control-module.h"

#include "queue_occupancy_trace_helper.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <string>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("open-loop-p2p-rate-ramp");

namespace {

double g_sender_start_time_s = 0.0;
double g_sender_duration_s = 5.0;
double g_drain_time_s = 0.5;

double g_link_bw_mbps = 350.0;
double g_one_way_delay_ms = 20.0;
double g_buffer_bdp = 1.0;
bool g_bdp_uses_rtt = true;

double g_start_rate_mbps = 220.0;
double g_end_rate_mbps = 450.0;
double g_triangle_freq_hz = 5.0;
double g_triangle_amp_mbps = 80.0;

uint32_t g_packet_size_bytes = 1200;
uint32_t g_flow_id = 1;
uint16_t g_udp_port = 5000;

double g_trace_interval_ms = 10.0;
std::string g_trace_path = "traces/open_loop_p2p_rate_ramp";
std::string g_trace_name = "open_loop_p2p_rate_ramp";

uint64_t
ToBps(double mbps)
{
    if (mbps <= 0.0)
    {
        return 0;
    }
    return static_cast<uint64_t>(mbps * 1000000.0 + 0.5);
}

uint32_t
ComputeBdpQueueBytes(uint64_t link_bps,
                     double one_way_delay_ms,
                     double buffer_bdp,
                     bool bdp_uses_rtt)
{
    const double bdp_delay_ms = bdp_uses_rtt ? 2.0 * one_way_delay_ms : one_way_delay_ms;
    const double queue_bytes = static_cast<double>(link_bps) * bdp_delay_ms * buffer_bdp / 8000.0;
    return static_cast<uint32_t>(std::max(1.0, queue_bytes));
}

double
ClampNonNegative(double value)
{
    return value > 0.0 ? value : 0.0;
}

class OpenLoopRampSender : public Application
{
  public:
    static TypeId GetTypeId()
    {
        static TypeId tid = TypeId("ns3::OpenLoopRampSender")
                                .SetParent<Application>()
                                .SetGroupName("Applications")
                                .AddConstructor<OpenLoopRampSender>();
        return tid;
    }

    void Configure(Address peer,
                   uint32_t packet_size_bytes,
                   double start_rate_mbps,
                   double end_rate_mbps,
                   double start_time_s,
                   double duration_s,
                   double triangle_freq_hz,
                   double triangle_amp_mbps,
                   uint32_t flow_id)
    {
        m_peer = peer;
        m_packetSizeBytes = packet_size_bytes;
        m_startRateMbps = start_rate_mbps;
        m_endRateMbps = end_rate_mbps;
        m_startTimeS = start_time_s;
        m_durationS = duration_s;
        m_triangleFreqHz = triangle_freq_hz;
        m_triangleAmpMbps = triangle_amp_mbps;
        m_flowId = flow_id;
    }

    uint64_t GetTotalTxBytes() const
    {
        return m_totalTxBytes;
    }

    double GetStartTimeSeconds() const
    {
        return m_startTimeS;
    }

    double GetStopTimeSeconds() const
    {
        return m_startTimeS + m_durationS;
    }

    double GetElapsedSeconds(double now_s) const
    {
        return std::max(0.0, now_s - m_startTimeS);
    }

    double GetBaselineRateMbpsAt(double now_s) const
    {
        if (now_s < m_startTimeS || now_s > GetStopTimeSeconds())
        {
            return 0.0;
        }
        if (m_durationS <= 0.0)
        {
            return m_endRateMbps;
        }
        const double elapsed_s = std::min(GetElapsedSeconds(now_s), m_durationS);
        const double progress = elapsed_s / m_durationS;
        return m_startRateMbps + (m_endRateMbps - m_startRateMbps) * progress;
    }

    double GetTriangleValueAt(double now_s) const
    {
        if (now_s < m_startTimeS || now_s > GetStopTimeSeconds() || m_triangleFreqHz <= 0.0)
        {
            return 0.0;
        }

        const double elapsed_s = GetElapsedSeconds(now_s);
        const double period_s = 1.0 / m_triangleFreqHz;
        const double phase = std::fmod(elapsed_s, period_s) / period_s;

        if (phase < 0.25)
        {
            return phase * 4.0;
        }
        if (phase < 0.75)
        {
            return 2.0 - phase * 4.0;
        }
        return phase * 4.0 - 4.0;
    }

    double GetTargetRateMbpsAt(double now_s) const
    {
        const double baseline_mbps = GetBaselineRateMbpsAt(now_s);
        if (baseline_mbps <= 0.0)
        {
            return 0.0;
        }
        return ClampNonNegative(baseline_mbps + m_triangleAmpMbps * GetTriangleValueAt(now_s));
    }

  private:
    void StartApplication() override
    {
        m_running = true;
        m_startTimeS = Simulator::Now().GetSeconds();
        m_totalTxBytes = 0;

        m_socket = Socket::CreateSocket(GetNode(), UdpSocketFactory::GetTypeId());
        m_socket->Connect(m_peer);

        Simulator::ScheduleNow(&OpenLoopRampSender::SendPacket, this);
    }

    void StopApplication() override
    {
        m_running = false;
        if (m_sendEvent.IsRunning())
        {
            Simulator::Cancel(m_sendEvent);
        }
        if (m_socket != nullptr)
        {
            m_socket->Close();
            m_socket = nullptr;
        }
    }

    void SendPacket()
    {
        if (!m_running || m_socket == nullptr)
        {
            return;
        }

        const double now_s = Simulator::Now().GetSeconds();
        if (now_s >= GetStopTimeSeconds())
        {
            return;
        }

        Ptr<Packet> packet = Create<Packet>(m_packetSizeBytes);
        packet->AddPacketTag(FlowIdTag(m_flowId));

        const int sent_bytes = m_socket->Send(packet);
        if (sent_bytes > 0)
        {
            m_totalTxBytes += static_cast<uint64_t>(sent_bytes);
        }

        ScheduleNextSend();
    }

    void ScheduleNextSend()
    {
        if (!m_running)
        {
            return;
        }

        const double now_s = Simulator::Now().GetSeconds();
        const double rate_mbps = GetTargetRateMbpsAt(now_s);
        if (rate_mbps <= 0.0)
        {
            m_sendEvent = Simulator::Schedule(MilliSeconds(1), &OpenLoopRampSender::SendPacket, this);
            return;
        }

        const double interval_s = (static_cast<double>(m_packetSizeBytes) * 8.0) /
                                  (rate_mbps * 1000000.0);
        m_sendEvent = Simulator::Schedule(Seconds(interval_s), &OpenLoopRampSender::SendPacket, this);
    }

    Ptr<Socket> m_socket;
    Address m_peer;
    EventId m_sendEvent;
    bool m_running{false};

    uint32_t m_packetSizeBytes{1200};
    double m_startRateMbps{100.0};
    double m_endRateMbps{450.0};
    double m_durationS{30.0};
    double m_triangleFreqHz{5.0};
    double m_triangleAmpMbps{80.0};
    uint32_t m_flowId{1};

    double m_startTimeS{0.0};
    uint64_t m_totalTxBytes{0};
};

NS_OBJECT_ENSURE_REGISTERED(OpenLoopRampSender);

class OpenLoopRateTracer
{
  public:
    OpenLoopRateTracer(Ptr<OpenLoopRampSender> sender,
                       Ptr<PacketSink> sink,
                       const std::string& folder,
                       const std::string& trace_name,
                       double interval_s,
                       double stop_time_s)
        : m_sender(sender),
          m_sink(sink),
          m_intervalS(interval_s),
          m_stopTimeS(stop_time_s)
    {
        EnsureDirectoryExists(folder);
        m_stream.open((folder + trace_name + "_rate_trace.txt").c_str(), std::fstream::out);
        if (m_stream.is_open())
        {
            m_stream << "#time_s\tbaseline_mbps\ttriangle_value\ttarget_mbps"
                     << "\ttx_mbps\trx_mbps\ttotal_tx_bytes\ttotal_rx_bytes" << std::endl;
        }
    }

    void Start()
    {
        m_lastSampleS = Simulator::Now().GetSeconds();
        m_lastTxBytes = m_sender != nullptr ? m_sender->GetTotalTxBytes() : 0;
        m_lastRxBytes = m_sink != nullptr ? m_sink->GetTotalRx() : 0;
        WriteSample();
    }

  private:
    void WriteSample()
    {
        if (!m_stream.is_open() || m_sender == nullptr || m_sink == nullptr)
        {
            return;
        }

        const double now_s = Simulator::Now().GetSeconds();
        const uint64_t total_tx_bytes = m_sender->GetTotalTxBytes();
        const uint64_t total_rx_bytes = m_sink->GetTotalRx();
        const double delta_s = std::max(1e-9, now_s - m_lastSampleS);
        const double tx_mbps =
            static_cast<double>(total_tx_bytes - m_lastTxBytes) * 8.0 / delta_s / 1000000.0;
        const double rx_mbps =
            static_cast<double>(total_rx_bytes - m_lastRxBytes) * 8.0 / delta_s / 1000000.0;

        m_stream << std::fixed << std::setprecision(6)
                 << now_s << "\t"
                 << m_sender->GetBaselineRateMbpsAt(now_s) << "\t"
                 << m_sender->GetTriangleValueAt(now_s) << "\t"
                 << m_sender->GetTargetRateMbpsAt(now_s) << "\t"
                 << tx_mbps << "\t"
                 << rx_mbps << "\t"
                 << total_tx_bytes << "\t"
                 << total_rx_bytes << std::endl;

        m_lastSampleS = now_s;
        m_lastTxBytes = total_tx_bytes;
        m_lastRxBytes = total_rx_bytes;

        if (now_s + m_intervalS <= m_stopTimeS)
        {
            Simulator::Schedule(Seconds(m_intervalS), &OpenLoopRateTracer::WriteSample, this);
        }
    }

    Ptr<OpenLoopRampSender> m_sender;
    Ptr<PacketSink> m_sink;
    std::fstream m_stream;
    double m_intervalS{0.01};
    double m_stopTimeS{30.0};
    double m_lastSampleS{0.0};
    uint64_t m_lastTxBytes{0};
    uint64_t m_lastRxBytes{0};
};

class QueueDropTracer
{
  public:
    QueueDropTracer(Ptr<NetDevice> device, const std::string& folder, const std::string& trace_name)
    {
        Ptr<PointToPointNetDevice> ptp_device = DynamicCast<PointToPointNetDevice>(device);
        if (ptp_device == nullptr)
        {
            return;
        }

        m_queue = ptp_device->GetQueue();
        if (m_queue == nullptr)
        {
            return;
        }

        EnsureDirectoryExists(folder);
        m_stream.open((folder + trace_name + "_drop_trace.txt").c_str(), std::fstream::out);
        if (m_stream.is_open())
        {
            m_stream << "#time_s\tpacket_bytes\tflow_id\tcum_drop_packets\tcum_drop_bytes" << std::endl;
        }

        m_queue->TraceConnectWithoutContext("Drop", MakeCallback(&QueueDropTracer::OnDrop, this));
    }

    uint64_t GetDropPackets() const
    {
        return m_dropPackets;
    }

    uint64_t GetDropBytes() const
    {
        return m_dropBytes;
    }

  private:
    void OnDrop(Ptr<const Packet> packet)
    {
        if (packet == nullptr)
        {
            return;
        }

        const uint32_t packet_bytes = packet->GetSize();
        uint32_t flow_id = 0;
        FlowIdTag flow_tag;
        if (packet->PeekPacketTag(flow_tag))
        {
            flow_id = flow_tag.GetFlowId();
        }

        ++m_dropPackets;
        m_dropBytes += packet_bytes;

        if (m_stream.is_open())
        {
            m_stream << std::fixed << std::setprecision(6)
                     << Simulator::Now().GetSeconds() << "\t"
                     << packet_bytes << "\t"
                     << flow_id << "\t"
                     << m_dropPackets << "\t"
                     << m_dropBytes << std::endl;
        }
    }

    Ptr<Queue<Packet>> m_queue;
    std::fstream m_stream;
    uint64_t m_dropPackets{0};
    uint64_t m_dropBytes{0};
};

void
WriteScenarioConfig(const std::string& folder,
                    const std::string& trace_name,
                    uint64_t link_bps,
                    uint32_t queue_bytes,
                    double sim_stop_s)
{
    EnsureDirectoryExists(folder);
    std::fstream stream((folder + trace_name + "_config.txt").c_str(), std::fstream::out);
    if (!stream.is_open())
    {
        return;
    }

    stream << std::fixed << std::setprecision(6)
           << "sender_start_time_s\t" << g_sender_start_time_s << std::endl
           << "sender_duration_s\t" << g_sender_duration_s << std::endl
           << "drain_time_s\t" << g_drain_time_s << std::endl
           << "sim_stop_s\t" << sim_stop_s << std::endl
           << "link_bw_mbps\t" << g_link_bw_mbps << std::endl
           << "link_bps\t" << link_bps << std::endl
           << "one_way_delay_ms\t" << g_one_way_delay_ms << std::endl
           << "buffer_bdp\t" << g_buffer_bdp << std::endl
           << "bdp_uses_rtt\t" << (g_bdp_uses_rtt ? 1 : 0) << std::endl
           << "queue_bytes\t" << queue_bytes << std::endl
           << "start_rate_mbps\t" << g_start_rate_mbps << std::endl
           << "end_rate_mbps\t" << g_end_rate_mbps << std::endl
           << "triangle_freq_hz\t" << g_triangle_freq_hz << std::endl
           << "triangle_amp_mbps\t" << g_triangle_amp_mbps << std::endl
           << "packet_size_bytes\t" << g_packet_size_bytes << std::endl
           << "trace_interval_ms\t" << g_trace_interval_ms << std::endl;
}

void
RunScenario()
{
    const uint64_t link_bps = ToBps(g_link_bw_mbps);
    const uint32_t queue_bytes =
        ComputeBdpQueueBytes(link_bps, g_one_way_delay_ms, g_buffer_bdp, g_bdp_uses_rtt);
    const double sim_stop_s = g_sender_start_time_s + g_sender_duration_s + g_drain_time_s;

    SetQueueOccupancyTraceFolder(g_trace_path);
    const std::string trace_folder = GetQueueOccupancyTraceFolder();
    WriteScenarioConfig(trace_folder, g_trace_name, link_bps, queue_bytes, sim_stop_s);

    NodeContainer nodes;
    nodes.Create(2);

    InternetStackHelper internet;
    internet.Install(nodes);

    PointToPointHelper p2p;
    p2p.SetDeviceAttribute("DataRate", DataRateValue(DataRate(link_bps)));
    p2p.SetChannelAttribute("Delay", TimeValue(MilliSeconds(g_one_way_delay_ms)));
    p2p.SetQueue("ns3::DropTailQueue",
                 "Mode",
                 StringValue("QUEUE_MODE_BYTES"),
                 "MaxBytes",
                 UintegerValue(queue_bytes));

    NetDeviceContainer devices = p2p.Install(nodes);

    std::shared_ptr<QueueOccupancyTracer> queue_tracer =
        InstallBottleneckQueueOccupancyTrace(devices.Get(0), g_trace_name, 1);
    std::shared_ptr<QueueDropTracer> drop_tracer =
        std::make_shared<QueueDropTracer>(devices.Get(0), trace_folder, g_trace_name);

    Ipv4AddressHelper ipv4;
    ipv4.SetBase("10.1.1.0", "255.255.255.0");
    Ipv4InterfaceContainer interfaces = ipv4.Assign(devices);
    TrafficControlHelper tch;
    tch.Uninstall(devices);
    Ipv4GlobalRoutingHelper::PopulateRoutingTables();

    PacketSinkHelper sink_helper("ns3::UdpSocketFactory",
                                 InetSocketAddress(Ipv4Address::GetAny(), g_udp_port));
    ApplicationContainer sink_apps = sink_helper.Install(nodes.Get(1));
    sink_apps.Start(Seconds(0.0));
    sink_apps.Stop(Seconds(sim_stop_s));
    Ptr<PacketSink> sink = DynamicCast<PacketSink>(sink_apps.Get(0));

    Ptr<OpenLoopRampSender> sender = CreateObject<OpenLoopRampSender>();
    sender->Configure(InetSocketAddress(interfaces.GetAddress(1), g_udp_port),
                      g_packet_size_bytes,
                      g_start_rate_mbps,
                      g_end_rate_mbps,
                      g_sender_start_time_s,
                      g_sender_duration_s,
                      g_triangle_freq_hz,
                      g_triangle_amp_mbps,
                      g_flow_id);
    nodes.Get(0)->AddApplication(sender);
    sender->SetStartTime(Seconds(g_sender_start_time_s));
    sender->SetStopTime(Seconds(g_sender_start_time_s + g_sender_duration_s));

    std::shared_ptr<OpenLoopRateTracer> rate_tracer =
        std::make_shared<OpenLoopRateTracer>(sender,
                                             sink,
                                             trace_folder,
                                             g_trace_name,
                                             g_trace_interval_ms / 1000.0,
                                             sim_stop_s);
    Simulator::Schedule(Seconds(g_sender_start_time_s), &OpenLoopRateTracer::Start, rate_tracer.get());

    std::cout << "=== Open-loop P2P rate-ramp experiment ===" << std::endl;
    std::cout << "sender_duration=" << g_sender_duration_s << "s, sim_stop=" << sim_stop_s
              << "s" << std::endl;
    std::cout << "link=" << g_link_bw_mbps << "Mbps, one_way_delay=" << g_one_way_delay_ms
              << "ms" << std::endl;
    std::cout << "queue=" << queue_bytes << " bytes (" << g_buffer_bdp
              << " BDP, basis=" << (g_bdp_uses_rtt ? "RTT" : "one-way") << ")" << std::endl;
    std::cout << "baseline=" << g_start_rate_mbps << "->" << g_end_rate_mbps
              << "Mbps, triangle_freq=" << g_triangle_freq_hz
              << "Hz, triangle_amp=" << g_triangle_amp_mbps << "Mbps" << std::endl;
    std::cout << "packet_size=" << g_packet_size_bytes << " bytes" << std::endl;
    std::cout << "trace_folder=" << trace_folder << std::endl;
    std::cout << "=========================================" << std::endl;

    Simulator::Stop(Seconds(sim_stop_s));
    Simulator::Run();
    Simulator::Destroy();

    (void)queue_tracer;
    std::cout << "drop_packets=" << drop_tracer->GetDropPackets()
              << ", drop_bytes=" << drop_tracer->GetDropBytes() << std::endl;
    std::cout << "total_tx_bytes=" << sender->GetTotalTxBytes()
              << ", total_rx_bytes=" << sink->GetTotalRx() << std::endl;
}

} // namespace

int
main(int argc, char* argv[])
{
    CommandLine cmd;
    cmd.AddValue("sender_start_time_s", "Sender start time in seconds", g_sender_start_time_s);
    cmd.AddValue("sender_duration_s", "Open-loop sender duration in seconds", g_sender_duration_s);
    cmd.AddValue("drain_time_s", "Extra simulation time after sender stops", g_drain_time_s);

    cmd.AddValue("link_bw_mbps", "P2P link bandwidth in Mbps", g_link_bw_mbps);
    cmd.AddValue("one_way_delay_ms", "P2P one-way propagation delay in ms", g_one_way_delay_ms);
    cmd.AddValue("buffer_bdp", "Sender-side device queue size in BDP units", g_buffer_bdp);
    cmd.AddValue("bdp_uses_rtt", "Use RTT rather than one-way delay for BDP queue sizing", g_bdp_uses_rtt);

    cmd.AddValue("start_rate_mbps", "Linear baseline start rate in Mbps", g_start_rate_mbps);
    cmd.AddValue("end_rate_mbps", "Linear baseline end rate in Mbps", g_end_rate_mbps);
    cmd.AddValue("triangle_freq_hz", "Triangle modulation frequency in Hz", g_triangle_freq_hz);
    cmd.AddValue("triangle_amp_mbps", "Triangle modulation peak amplitude in Mbps", g_triangle_amp_mbps);

    cmd.AddValue("packet_size_bytes", "UDP payload size in bytes", g_packet_size_bytes);
    cmd.AddValue("flow_id", "FlowIdTag used by queue occupancy trace", g_flow_id);
    cmd.AddValue("udp_port", "UDP sink port", g_udp_port);

    cmd.AddValue("trace_interval_ms", "Rate trace sampling interval in ms", g_trace_interval_ms);
    cmd.AddValue("trace_path", "Output trace directory", g_trace_path);
    cmd.AddValue("trace_name", "Output trace file prefix", g_trace_name);
    cmd.Parse(argc, argv);

    RunScenario();
    return 0;
}
