/** Network topology
 *
 *    edge_bw, edge_delay                        edge_bw, edge_delay
 * n0----L0--------|                        |------L3-------n4
 *                 |   bottleneck_bw, D ms |
 *                 n2--------L2------------n3
 *    edge_bw, edge_delay |                | edge_bw, edge_delay
 * n1----L1--------------|                |------L4-------n5
 *
 * Flow A: n0 -> n4, DQC FreqCCv3 probing flow with oscillation enabled
 * Flow B: n1 -> n5, UDP CBR background flow to pin queue state
 *
 * This scenario is intended for frequency-response calibration:
 *   - the configured probe phase injects oscillation on Flow A
 *   - INT = CRUISE + REFILL is inspected for recvrate / smoothed RTT response
 *   - bottleneck queue occupancy is traced to label UNDER / FULL / OVER
 */

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/applications-module.h"
#include "ns3/ipv4-global-routing-helper.h"
#include "ns3/traffic-control-module.h"
#include "ns3/dqc-module.h"
#include "ns3/log.h"

#include <algorithm>
#include <cmath>
#include <cctype>
#include <cstdint>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <sstream>
#include <string>
#include <vector>

#include "queue_occupancy_trace_helper.h"

using namespace ns3;
using namespace dqc;

#ifndef DQC_SCENARIO_LOG_COMPONENT
#define DQC_SCENARIO_LOG_COMPONENT "freqccv3-2flow-calibration"
#endif

#ifndef DQC_SCENARIO_INSTANCE
#define DQC_SCENARIO_INSTANCE "freqccv3_2flow_calibration"
#endif

#ifndef DQC_SCENARIO_TITLE
#define DQC_SCENARIO_TITLE "FreqCCv3 2-Flow Calibration"
#endif

#ifndef DQC_SCENARIO_CC
#define DQC_SCENARIO_CC kFreqCCv3
#endif

NS_LOG_COMPONENT_DEFINE(DQC_SCENARIO_LOG_COMPONENT);

namespace {

constexpr uint32_t kProbeFlowId = 1;
constexpr uint32_t kNumTaggedFlows = 1;

double g_sim_time = 20.0;
std::string g_trace_path;
std::string g_queue_state = "full";

double g_edge_bw_mbps = 100.0;
double g_edge_delay_ms = 1.0;
double g_bottleneck_bw_mbps = 20.0;
double g_bottleneck_delay_ms = 18.0;
double g_buffer_bdp = 1.0;

double g_probe_start_time = 0.5;
double g_bg_start_time = 0.5;

double g_probe_freq_hz = 60.0;
std::string g_probe_amp_mode = "miu2";
std::string g_probe_recv_signal_mode = "bandwidth_latest";
double g_probe_fixed_mbps = 0.0;
double g_probe_max_mbps = 0.0;
double g_interval_window_rtt_mult = 1.0;
double g_probe_up_min_rtt_mult = 0.0;

double g_bg_rate_mbps = 0.0;
uint32_t g_bg_packet_size = 1200;

bool g_dynamic_delay_enable = false;
double g_dynamic_delay_period_s = 5.0;
std::string g_dynamic_edge_delay_seq_ms;
std::string g_dynamic_bottleneck_delay_seq_ms;

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
ComputeQueueBytes(uint64_t bottleneck_bps, double edge_delay_ms, double bottleneck_delay_ms, double buffer_bdp)
{
    double rtt_ms = 2.0 * (2.0 * edge_delay_ms + bottleneck_delay_ms);
    double queue_delay_ms = std::max(1.0, buffer_bdp * rtt_ms);
    double queue_bytes = static_cast<double>(bottleneck_bps) * queue_delay_ms / 8000.0;
    return static_cast<uint32_t>(std::max(1.0, queue_bytes));
}

std::string
StripAsciiWhitespace(std::string value)
{
    value.erase(std::remove_if(value.begin(),
                               value.end(),
                               [](unsigned char ch) { return std::isspace(ch) != 0; }),
                value.end());
    return value;
}

std::vector<double>
ParseDelaySequenceMs(const std::string& raw)
{
    std::vector<double> values;
    std::stringstream stream(raw);
    std::string token;
    while (std::getline(stream, token, ','))
    {
        token = StripAsciiWhitespace(token);
        if (token.empty())
        {
            continue;
        }
        try
        {
            values.push_back(std::stod(token));
        }
        catch (const std::exception&)
        {
            NS_FATAL_ERROR("Invalid delay sequence token: " << token);
        }
    }
    return values;
}

std::vector<double>
BuildDelayScheduleMs(const std::string& raw, double fallback_ms)
{
    std::vector<double> values = ParseDelaySequenceMs(raw);
    if (values.empty())
    {
        values.push_back(fallback_ms);
    }
    return values;
}

std::string
FormatDelaySchedule(const std::vector<double>& values)
{
    std::ostringstream stream;
    for (size_t i = 0; i < values.size(); ++i)
    {
        if (i > 0)
        {
            stream << ",";
        }
        stream << values[i];
    }
    return stream.str();
}

Ptr<PointToPointChannel>
GetPointToPointChannel(const NetDeviceContainer& devices)
{
    Ptr<PointToPointNetDevice> device = DynamicCast<PointToPointNetDevice>(devices.Get(0));
    NS_ABORT_MSG_IF(device == nullptr, "Expected PointToPointNetDevice");
    Ptr<PointToPointChannel> channel = DynamicCast<PointToPointChannel>(device->GetChannel());
    NS_ABORT_MSG_IF(channel == nullptr, "Expected PointToPointChannel");
    return channel;
}

void
ApplyPropagationDelayStep(const std::vector<Ptr<PointToPointChannel>>& edge_channels,
                          const std::vector<Ptr<PointToPointChannel>>& bottleneck_channels,
                          double edge_delay_ms,
                          double bottleneck_delay_ms,
                          uint32_t step_index)
{
    for (const Ptr<PointToPointChannel>& channel : edge_channels)
    {
        channel->SetPropagationDelay(MilliSeconds(edge_delay_ms));
    }
    for (const Ptr<PointToPointChannel>& channel : bottleneck_channels)
    {
        channel->SetPropagationDelay(MilliSeconds(bottleneck_delay_ms));
    }

    std::cout << "[delay-step] t=" << Simulator::Now().GetSeconds() << "s"
              << " step=" << step_index
              << " edge_delay_ms=" << edge_delay_ms
              << " bottleneck_delay_ms=" << bottleneck_delay_ms
              << " path_oneway_ms=" << (2.0 * edge_delay_ms + bottleneck_delay_ms)
              << std::endl;
}

void
ConfigurePropagationDelaySchedule(const std::vector<Ptr<PointToPointChannel>>& edge_channels,
                                  const std::vector<Ptr<PointToPointChannel>>& bottleneck_channels,
                                  const std::vector<double>& edge_delay_schedule_ms,
                                  const std::vector<double>& bottleneck_delay_schedule_ms)
{
    if (!g_dynamic_delay_enable)
    {
        return;
    }
    if (g_dynamic_delay_period_s <= 0.0)
    {
        NS_FATAL_ERROR("dynamic_delay_period_s must be > 0");
    }

    ApplyPropagationDelayStep(edge_channels,
                              bottleneck_channels,
                              edge_delay_schedule_ms.front(),
                              bottleneck_delay_schedule_ms.front(),
                              0);

    uint32_t step_index = 1;
    for (double t = g_dynamic_delay_period_s; t < g_sim_time; t += g_dynamic_delay_period_s, ++step_index)
    {
        const double edge_delay_ms = edge_delay_schedule_ms[step_index % edge_delay_schedule_ms.size()];
        const double bottleneck_delay_ms =
            bottleneck_delay_schedule_ms[step_index % bottleneck_delay_schedule_ms.size()];

        Simulator::Schedule(Seconds(t),
                            &ApplyPropagationDelayStep,
                            edge_channels,
                            bottleneck_channels,
                            edge_delay_ms,
                            bottleneck_delay_ms,
                            step_index);
    }
}

void
ApplyQueueStatePreset()
{
    if (g_queue_state == "manual")
    {
        return;
    }

    const double bottleneck = g_bottleneck_bw_mbps;
    if (g_queue_state == "under")
    {
        if (g_bg_rate_mbps <= 0.0)
        {
            g_bg_rate_mbps = 0.20 * bottleneck;
        }
        return;
    }

    if (g_queue_state == "full")
    {
        if (g_bg_rate_mbps <= 0.0)
        {
            g_bg_rate_mbps = 0.50 * bottleneck;
        }
        return;
    }

    if (g_queue_state == "over")
    {
        if (g_bg_rate_mbps <= 0.0)
        {
            g_bg_rate_mbps = 0.80 * bottleneck;
        }
        return;
    }

    std::cerr << "Unknown queue_state='" << g_queue_state
              << "', fallback to manual values." << std::endl;
}

Ptr<DqcSender>
InstallProbeFlow(Ptr<Node> sender,
                 Ptr<Node> receiver,
                 uint16_t send_port,
                 uint16_t recv_port,
                 float start_time,
                 float stop_time,
                 DqcTrace* trace,
                 DqcTraceState* stat)
{
    Ptr<DqcSender> send_app = CreateObject<DqcSender>(DQC_SCENARIO_CC, false);
    Ptr<DqcReceiver> recv_app = CreateObject<DqcReceiver>(100);
    sender->AddApplication(send_app);
    receiver->AddApplication(recv_app);

    send_app->SetNumEmulatedConnections(1);
    Ptr<Ipv4> ipv4 = receiver->GetObject<Ipv4>();
    Ipv4Address receiver_ip = ipv4->GetAddress(1, 0).GetLocal();

    recv_app->Bind(recv_port);
    send_app->Bind(send_port);
    send_app->ConfigurePeer(receiver_ip, recv_port);
    send_app->SetStartTime(Seconds(start_time));
    send_app->SetStopTime(Seconds(stop_time));
    recv_app->SetStartTime(Seconds(start_time));
    recv_app->SetStopTime(Seconds(stop_time));

    if (g_probe_max_mbps > 0.0)
    {
        send_app->SetMaxBandwidth(ToBps(g_probe_max_mbps));
    }

    send_app->SetSenderId(kProbeFlowId);
    send_app->SetCongestionId(kProbeFlowId);

    if (trace != nullptr)
    {
        send_app->SetBwTraceFuc(MakeCallback(&DqcTrace::OnBw, trace));
        send_app->SetRttTraceFuc(MakeCallback(&DqcTrace::OnRtt, trace));
        send_app->SetQueueDelayTraceFuc(MakeCallback(&DqcTrace::OnQueueDelay, trace));
        send_app->SetSendRateTraceFuc(MakeCallback(&DqcTrace::OnSendRate, trace));
        send_app->SetRecvRateTraceFuc(MakeCallback(&DqcTrace::OnRecvRate, trace));
        send_app->SetRecvRateRawTraceFuc(MakeCallback(&DqcTrace::OnRecvRateRaw, trace));
        send_app->SetInflightTraceFuc(MakeCallback(&DqcTrace::OnInflight, trace));
        send_app->SetBbrModeTraceFuc(MakeCallback(&DqcTrace::OnBbrMode, trace));
        send_app->SetUpPhaseTraceFuc(MakeCallback(&DqcTrace::OnUpPhase, trace));
        send_app->SetFreqAnalysisTraceFuc(MakeCallback(&DqcTrace::OnFreqAnalysis, trace));
        send_app->SetRttFreqAnalysisTraceFuc(MakeCallback(&DqcTrace::OnRttFreqAnalysis, trace));
        send_app->SetFreqCCv4LoadTraceFuc(MakeCallback(&DqcTrace::OnFreqCCv4Load, trace));
        send_app->SetLossRateTraceFuc(MakeCallback(&DqcTrace::OnLossRate, trace));

        recv_app->SetOwdTraceFuc(MakeCallback(&DqcTrace::OnOwd, trace));
        recv_app->SetGoodputTraceFuc(MakeCallback(&DqcTrace::OnGoodput, trace));
        recv_app->SetStatsTraceFuc(MakeCallback(&DqcTrace::OnStats, trace));
        if (stat != nullptr)
        {
            trace->SetStatsTraceFuc(MakeCallback(&DqcTraceState::OnStats, stat));
        }
    }

    send_app->ConfigureFreqCC(g_probe_freq_hz,
                              g_probe_amp_mode,
                              g_probe_fixed_mbps,
                              "up",
                              g_probe_recv_signal_mode);
    send_app->SetFreqCCIntervalWindowMultiplier(g_interval_window_rtt_mult);
    send_app->SetFreqCCMinProbeUpDurationRttMultiplier(g_probe_up_min_rtt_mult);
    return send_app;
}

void
InstallBackgroundUdpFlow(Ptr<Node> sender,
                         Ptr<Node> receiver,
                         uint16_t port,
                         float start_time,
                         float stop_time)
{
    PacketSinkHelper sink_helper("ns3::UdpSocketFactory",
                                 InetSocketAddress(Ipv4Address::GetAny(), port));
    ApplicationContainer sink_apps = sink_helper.Install(receiver);
    sink_apps.Start(Seconds(start_time));
    sink_apps.Stop(Seconds(stop_time));

    if (g_bg_rate_mbps <= 0.0)
    {
        return;
    }

    Ptr<Ipv4> ipv4 = receiver->GetObject<Ipv4>();
    Ipv4Address receiver_ip = ipv4->GetAddress(1, 0).GetLocal();

    OnOffHelper onoff("ns3::UdpSocketFactory", InetSocketAddress(receiver_ip, port));
    onoff.SetAttribute("PacketSize", UintegerValue(g_bg_packet_size));
    onoff.SetAttribute("DataRate", DataRateValue(DataRate(ToBps(g_bg_rate_mbps))));
    onoff.SetAttribute("OnTime",
                       StringValue("ns3::ConstantRandomVariable[Constant=1]"));
    onoff.SetAttribute("OffTime",
                       StringValue("ns3::ConstantRandomVariable[Constant=0]"));
    ApplicationContainer apps = onoff.Install(sender);
    apps.Start(Seconds(start_time));
    apps.Stop(Seconds(stop_time));
}

void
RunCalibrationScenario()
{
    ApplyQueueStatePreset();

    const std::vector<double> edge_delay_schedule_ms =
        BuildDelayScheduleMs(g_dynamic_edge_delay_seq_ms, g_edge_delay_ms);
    const std::vector<double> bottleneck_delay_schedule_ms =
        BuildDelayScheduleMs(g_dynamic_bottleneck_delay_seq_ms, g_bottleneck_delay_ms);
    const double initial_edge_delay_ms = edge_delay_schedule_ms.front();
    const double initial_bottleneck_delay_ms = bottleneck_delay_schedule_ms.front();

    NodeContainer nodes;
    nodes.Create(6);

    NodeContainer n0n2(nodes.Get(0), nodes.Get(2));
    NodeContainer n1n2(nodes.Get(1), nodes.Get(2));
    NodeContainer n2n3(nodes.Get(2), nodes.Get(3));
    NodeContainer n3n4(nodes.Get(3), nodes.Get(4));
    NodeContainer n3n5(nodes.Get(3), nodes.Get(5));

    InternetStackHelper internet;
    internet.Install(nodes);

    PointToPointHelper p2p;
    TrafficControlHelper tch;

    uint64_t edge_bps = ToBps(g_edge_bw_mbps);
    uint64_t bottleneck_bps = ToBps(g_bottleneck_bw_mbps);
    uint32_t edge_queue_bytes =
        ComputeQueueBytes(edge_bps, initial_edge_delay_ms, initial_bottleneck_delay_ms, g_buffer_bdp);
    uint32_t bottleneck_queue_bytes = ComputeQueueBytes(
        bottleneck_bps, initial_edge_delay_ms, initial_bottleneck_delay_ms, g_buffer_bdp);

    p2p.SetQueue("ns3::DropTailQueue",
                 "Mode",
                 StringValue("QUEUE_MODE_BYTES"),
                 "MaxBytes",
                 UintegerValue(edge_queue_bytes));
    p2p.SetDeviceAttribute("DataRate", DataRateValue(DataRate(edge_bps)));
    p2p.SetChannelAttribute("Delay", TimeValue(MilliSeconds(initial_edge_delay_ms)));
    NetDeviceContainer devn0n2 = p2p.Install(n0n2);
    NetDeviceContainer devn1n2 = p2p.Install(n1n2);
    NetDeviceContainer devn3n4 = p2p.Install(n3n4);
    NetDeviceContainer devn3n5 = p2p.Install(n3n5);

    p2p.SetQueue("ns3::DropTailQueue",
                 "Mode",
                 StringValue("QUEUE_MODE_BYTES"),
                 "MaxBytes",
                 UintegerValue(bottleneck_queue_bytes));
    p2p.SetDeviceAttribute("DataRate", DataRateValue(DataRate(bottleneck_bps)));
    p2p.SetChannelAttribute("Delay", TimeValue(MilliSeconds(initial_bottleneck_delay_ms)));
    NetDeviceContainer devn2n3 = p2p.Install(n2n3);

    std::vector<Ptr<PointToPointChannel>> edge_channels = {
        GetPointToPointChannel(devn0n2),
        GetPointToPointChannel(devn1n2),
        GetPointToPointChannel(devn3n4),
        GetPointToPointChannel(devn3n5)};
    std::vector<Ptr<PointToPointChannel>> bottleneck_channels = {GetPointToPointChannel(devn2n3)};
    ConfigurePropagationDelaySchedule(edge_channels,
                                      bottleneck_channels,
                                      edge_delay_schedule_ms,
                                      bottleneck_delay_schedule_ms);

    std::vector<std::shared_ptr<QueueOccupancyTracer>> queue_tracers;
    queue_tracers.push_back(InstallBottleneckQueueOccupancyTrace(
        devn2n3.Get(0), DQC_SCENARIO_INSTANCE, kNumTaggedFlows));

    Ipv4AddressHelper ipv4;
    ipv4.SetBase("10.1.1.0", "255.255.255.0");
    Ipv4InterfaceContainer i0i2 = ipv4.Assign(devn0n2);
    tch.Uninstall(devn0n2);

    ipv4.SetBase("10.1.2.0", "255.255.255.0");
    Ipv4InterfaceContainer i1i2 = ipv4.Assign(devn1n2);
    tch.Uninstall(devn1n2);

    ipv4.SetBase("10.1.3.0", "255.255.255.0");
    Ipv4InterfaceContainer i2i3 = ipv4.Assign(devn2n3);
    tch.Uninstall(devn2n3);

    ipv4.SetBase("10.1.4.0", "255.255.255.0");
    Ipv4InterfaceContainer i3i4 = ipv4.Assign(devn3n4);
    tch.Uninstall(devn3n4);

    ipv4.SetBase("10.1.5.0", "255.255.255.0");
    Ipv4InterfaceContainer i3i5 = ipv4.Assign(devn3n5);
    tch.Uninstall(devn3n5);

    Ipv4GlobalRoutingHelper::PopulateRoutingTables();

    std::string instance = std::string(DQC_SCENARIO_INSTANCE) + "_" + g_queue_state;
    std::unique_ptr<DqcTraceState> stat(new DqcTraceState(instance));
    std::unique_ptr<DqcTrace> trace(new DqcTrace(1));
    stat->ReisterAvgDelayId(1);
    stat->RegisterCongestionType(1);
    trace->Log(instance + "_probe",
               DqcTraceEnable::E_DQC_GOODPUT | DqcTraceEnable::E_DQC_RTT |
                   DqcTraceEnable::E_DQC_BW | DqcTraceEnable::E_DQC_OWD |
                   DqcTraceEnable::E_DQC_STAT | DqcTraceEnable::E_DQC_SEND_RATE |
                   DqcTraceEnable::E_DQC_RECV_RATE | DqcTraceEnable::E_DQC_RECV_RATE_RAW |
                   DqcTraceEnable::E_DQC_INFLIGHT |
                   DqcTraceEnable::E_DQC_BBR_MODE | DqcTraceEnable::E_DQC_UP_PHASE |
                   DqcTraceEnable::E_DQC_FREQ_ANALYSIS | DqcTraceEnable::E_DQC_LOSS_RATE |
                   DqcTraceEnable::E_DQC_QUEUE_DELAY |
                   DqcTraceEnable::E_DQC_FREQCCV4_LOAD);

    InstallProbeFlow(nodes.Get(0),
                     nodes.Get(4),
                     1000,
                     5000,
                     static_cast<float>(g_probe_start_time),
                     static_cast<float>(g_sim_time),
                     trace.get(),
                     stat.get());

    InstallBackgroundUdpFlow(nodes.Get(1),
                             nodes.Get(5),
                             7000,
                             static_cast<float>(g_bg_start_time),
                             static_cast<float>(g_sim_time));

    Simulator::Stop(Seconds(g_sim_time));
    Simulator::Run();
    Simulator::Destroy();

    stat->Flush(static_cast<uint32_t>(bottleneck_bps), static_cast<uint32_t>(g_sim_time));
}

} // namespace

int
main(int argc, char* argv[])
{
    CommandLine cmd;
    cmd.AddValue("sim_time", "Simulation time in seconds", g_sim_time);
    cmd.AddValue("trace_path", "Output trace directory path", g_trace_path);
    cmd.AddValue("queue_state",
                 "Queue-state preset: under | full | over | manual",
                 g_queue_state);

    cmd.AddValue("edge_bw_mbps", "Edge link bandwidth in Mbps", g_edge_bw_mbps);
    cmd.AddValue("edge_delay_ms", "Edge link one-way delay in ms", g_edge_delay_ms);
    cmd.AddValue("bottleneck_bw_mbps",
                 "Bottleneck link bandwidth in Mbps",
                 g_bottleneck_bw_mbps);
    cmd.AddValue("bottleneck_delay_ms",
                 "Bottleneck link one-way delay in ms",
                 g_bottleneck_delay_ms);
    cmd.AddValue("buffer_bdp",
                 "Bottleneck queue size in BDP units",
                 g_buffer_bdp);

    cmd.AddValue("probe_start", "Probe-flow start time in seconds", g_probe_start_time);
    cmd.AddValue("bg_start", "Background-flow start time in seconds", g_bg_start_time);

    cmd.AddValue("probe_freq_hz", "Probe-flow oscillation frequency", g_probe_freq_hz);
    cmd.AddValue("probe_amp_mode", "Probe-flow amplitude mode", g_probe_amp_mode);
    cmd.AddValue("probe_recv_signal_mode",
                 "FreqCCv3 interval analysis input: bandwidth_latest | delivery_rate_latest",
                 g_probe_recv_signal_mode);
    cmd.AddValue("probe_fixed_mbps",
                 "Probe-flow fixed oscillation amplitude in Mbps",
                 g_probe_fixed_mbps);
    cmd.AddValue("probe_max_mbps",
                 "Probe-flow bandwidth cap in Mbps",
                 g_probe_max_mbps);
    cmd.AddValue("interval_win_rtt_mult",
                 "Interval-phase STFT window multiplier on min_rtt",
                 g_interval_window_rtt_mult);
    cmd.AddValue("probe_up_min_rtt_mult",
                 "Experimental minimum PROBE_UP duration in RTT multiples",
                 g_probe_up_min_rtt_mult);

    cmd.AddValue("bg_rate_mbps",
                 "Background UDP CBR rate in Mbps",
                 g_bg_rate_mbps);
    cmd.AddValue("bg_packet_size",
                 "Background UDP packet size in bytes",
                 g_bg_packet_size);
    cmd.AddValue("dynamic_delay_enable",
                 "Enable deterministic propagation-delay changes",
                 g_dynamic_delay_enable);
    cmd.AddValue("dynamic_delay_period_s",
                 "Propagation-delay update period in seconds",
                 g_dynamic_delay_period_s);
    cmd.AddValue("dynamic_edge_delay_seq_ms",
                 "Comma-separated edge-link one-way delays in ms, cycled every period",
                 g_dynamic_edge_delay_seq_ms);
    cmd.AddValue("dynamic_bottleneck_delay_seq_ms",
                 "Comma-separated bottleneck-link one-way delays in ms, cycled every period",
                 g_dynamic_bottleneck_delay_seq_ms);
    cmd.Parse(argc, argv);

    ApplyQueueStatePreset();

    if (!g_trace_path.empty())
    {
        if (g_trace_path.back() != '/')
        {
            g_trace_path.push_back('/');
        }
        set_dqc_trace_folder(g_trace_path);
    }
    SetQueueOccupancyTraceFolder(g_trace_path);

    std::cout << "=== " << DQC_SCENARIO_TITLE << " ===" << std::endl;
    std::cout << "queue_state=" << g_queue_state << std::endl;
    std::cout << "sim_time=" << g_sim_time << "s" << std::endl;
    std::cout << "bottleneck=" << g_bottleneck_bw_mbps << "Mbps, "
              << g_bottleneck_delay_ms << "ms" << std::endl;
    std::cout << "buffer=" << g_buffer_bdp << " BDP" << std::endl;
    std::cout << "probe_freq=" << g_probe_freq_hz << "Hz, probe_amp_mode=" << g_probe_amp_mode
              << ", probe_recv_signal_mode=" << g_probe_recv_signal_mode
              << ", probe_fixed=" << g_probe_fixed_mbps << "Mbps" << std::endl;
    std::cout << "probe_cap=" << g_probe_max_mbps << "Mbps, background_rate=" << g_bg_rate_mbps
              << "Mbps" << std::endl;
    std::cout << "interval_window_multiplier=" << g_interval_window_rtt_mult << std::endl;
    std::cout << "probe_up_min_rtt_mult=" << g_probe_up_min_rtt_mult << std::endl;
    if (g_dynamic_delay_enable)
    {
        std::cout << "dynamic_delay_period_s=" << g_dynamic_delay_period_s << std::endl;
        std::cout << "dynamic_edge_delay_seq_ms="
                  << FormatDelaySchedule(BuildDelayScheduleMs(g_dynamic_edge_delay_seq_ms, g_edge_delay_ms))
                  << std::endl;
        std::cout << "dynamic_bottleneck_delay_seq_ms="
                  << FormatDelaySchedule(
                         BuildDelayScheduleMs(g_dynamic_bottleneck_delay_seq_ms, g_bottleneck_delay_ms))
                  << std::endl;
    }
    std::cout << "===============================" << std::endl;

    RunCalibrationScenario();
    return 0;
}
