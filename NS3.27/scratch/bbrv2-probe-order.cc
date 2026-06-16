/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * BBRv2 multi-flow probe-order experiment.
 *
 * This scratch program is intentionally small and self-contained. It uses the
 * experimental Bbr2Sender forced PROBE_UP hook added in
 * src/dqc/model/thirdparty/congestion/quic_bbr2_sender.{h,cc} and samples a
 * fixed-bin CSV for post-processing under experiments/bbrv2_probe_order.
 */

#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/dqc-module.h"
#include "ns3/internet-module.h"
#include "ns3/ipv4-global-routing-helper.h"
#include "ns3/network-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/point-to-point-net-device.h"
#include "ns3/traffic-control-module.h"

#include <algorithm>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <sstream>
#include <string>
#include <vector>

using namespace ns3;

namespace {

struct FlowRuntime {
  Ptr<DqcSender> sender;
  uint32_t flow_id;
  uint32_t probe_order;
  double forced_probe_time_s;
  bool in_probe_up = false;
  bool recorded_probe_up = false;
  double probe_start_s = -1.0;
  double probe_end_s = -1.0;
  uint64_t probe_start_sent = 0;
  uint64_t probe_end_sent = 0;
  uint64_t probe_start_acked = 0;
  uint64_t probe_end_acked = 0;
  double srtt_sum_us = 0.0;
  uint64_t srtt_samples = 0;
  uint64_t max_queue_bytes = 0;
};

std::ofstream g_timeseries;
std::ofstream g_intervals;
std::vector<FlowRuntime> g_flows;
Ptr<Queue<Packet> > g_bottleneck_queue;
uint64_t g_last_sent_bytes[1024];
uint64_t g_last_acked_bytes[1024];
uint32_t g_run_id = 1;
uint32_t g_seed = 1;
uint32_t g_n_flows = 8;
uint64_t g_bottleneck_bps = 400000000000ULL;
double g_bin_s = 0.00001;
double g_sim_time_s = 1.0;
double g_observation_start_s = 0.2;

std::string
ModeName(int32_t mode)
{
  switch (mode) {
  case 0:
    return "STARTUP";
  case 1:
    return "DRAIN";
  case 2:
    return "PROBE_BW_DOWN";
  case 3:
    return "PROBE_BW_CRUISE";
  case 4:
    return "PROBE_BW_REFILL";
  case 5:
    return "PROBE_BW_UP";
  case 6:
    return "PROBE_RTT";
  case 7:
    return "PROBE_BW_PRE_UP";
  case 8:
    return "PROBE_BW_GUARD";
  case 9:
    return "PROBE_BW_POST_UP";
  case 10:
    return "PROBE_BW_DOWN_SLIGHTLY";
  default:
    return "UNKNOWN";
  }
}

std::vector<uint32_t>
BuildProbeOrderByFlow(uint32_t n_flows, uint32_t seed)
{
  std::vector<uint32_t> flow_ids(n_flows);
  std::iota(flow_ids.begin(), flow_ids.end(), 0);
  uint64_t state = seed ? seed : 1;
  for (uint32_t i = n_flows; i > 1; --i) {
    state = state * 6364136223846793005ULL + 1442695040888963407ULL;
    uint32_t j = static_cast<uint32_t>(state % i);
    std::swap(flow_ids[i - 1], flow_ids[j]);
  }

  std::vector<uint32_t> order_by_flow(n_flows, 0);
  for (uint32_t order = 1; order <= n_flows; ++order) {
    order_by_flow[flow_ids[order - 1]] = order;
  }
  return order_by_flow;
}

Ipv4Address
GetReceiverAddress(Ptr<Node> receiver)
{
  Ptr<Ipv4> ipv4 = receiver->GetObject<Ipv4>();
  return ipv4->GetAddress(1, 0).GetLocal();
}

Ptr<DqcSender>
InstallFlow(Ptr<Node> sender,
            Ptr<Node> receiver,
            uint32_t flow_id,
            uint16_t send_port,
            uint16_t recv_port,
            double app_start_s,
            double app_stop_s,
            uint32_t send_buffer_bytes,
            uint32_t fill_batch,
            uint32_t max_cwnd_packets)
{
  Ptr<DqcSender> send_app = CreateObject<DqcSender>(dqc::kBBRv2NoProbeRtt, false);
  Ptr<DqcReceiver> recv_app = CreateObject<DqcReceiver>(1);
  sender->AddApplication(send_app);
  receiver->AddApplication(recv_app);

  recv_app->Bind(recv_port);
  send_app->Bind(send_port);
  send_app->ConfigurePeer(GetReceiverAddress(receiver), recv_port);
  send_app->SetSenderId(flow_id + 1);
  send_app->SetCongestionId(flow_id + 1);
  send_app->SetDataGeneratorBatch(fill_batch);
  send_app->SetStreamSendBufferBytes(send_buffer_bytes);
  send_app->SetBbr2MaxCongestionWindowPackets(max_cwnd_packets);

  send_app->SetStartTime(Seconds(app_start_s));
  send_app->SetStopTime(Seconds(app_stop_s));
  recv_app->SetStartTime(Seconds(app_start_s));
  recv_app->SetStopTime(Seconds(app_stop_s));

  return send_app;
}

void
CloseProbeInterval(FlowRuntime& flow, double now_s)
{
  if (!flow.in_probe_up) {
    return;
  }
  DqcSender::Bbr2ExperimentSnapshot snapshot;
  if (!flow.sender->GetBbr2ExperimentSnapshot(&snapshot)) {
    return;
  }
  flow.in_probe_up = false;
  flow.recorded_probe_up = true;
  flow.probe_end_s = now_s;
  flow.probe_end_sent = snapshot.sent_bytes;
  flow.probe_end_acked = snapshot.acked_bytes;

  double avg_srtt_us = 0.0;
  if (flow.srtt_samples > 0) {
    avg_srtt_us = flow.srtt_sum_us / static_cast<double>(flow.srtt_samples);
  }
  g_intervals << g_run_id << "," << g_seed << "," << flow.flow_id << ","
              << flow.probe_order << "," << std::fixed << std::setprecision(9)
              << flow.forced_probe_time_s << "," << flow.probe_start_s << ","
              << flow.probe_end_s << ","
              << (flow.probe_end_s - flow.probe_start_s) << ","
              << (flow.probe_end_sent - flow.probe_start_sent) << ","
              << (flow.probe_end_acked - flow.probe_start_acked) << ","
              << avg_srtt_us << "," << flow.max_queue_bytes << "\n";
}

void
Sample()
{
  const double now_s = Simulator::Now().GetSeconds();
  const uint32_t queue_bytes =
      g_bottleneck_queue ? g_bottleneck_queue->GetNBytes() : 0;

  for (FlowRuntime& flow : g_flows) {
    DqcSender::Bbr2ExperimentSnapshot snapshot;
    if (!flow.sender->GetBbr2ExperimentSnapshot(&snapshot)) {
      continue;
    }

    uint64_t sent_delta = 0;
    uint64_t acked_delta = 0;
    if (snapshot.sent_bytes >= g_last_sent_bytes[flow.flow_id]) {
      sent_delta = snapshot.sent_bytes - g_last_sent_bytes[flow.flow_id];
    }
    if (snapshot.acked_bytes >= g_last_acked_bytes[flow.flow_id]) {
      acked_delta = snapshot.acked_bytes - g_last_acked_bytes[flow.flow_id];
    }
    g_last_sent_bytes[flow.flow_id] = snapshot.sent_bytes;
    g_last_acked_bytes[flow.flow_id] = snapshot.acked_bytes;

    uint64_t send_rate_bps = 0;
    if (g_bin_s > 0.0) {
      send_rate_bps =
          static_cast<uint64_t>((static_cast<double>(sent_delta) * 8.0) /
                                g_bin_s);
    }

    const bool is_probe_up = snapshot.probe_phase == "PROBE_UP";
    const bool is_controlled_probe =
        is_probe_up && !flow.recorded_probe_up &&
        now_s >= flow.forced_probe_time_s &&
        snapshot.probe_phase_start_time_s + g_bin_s >=
            flow.forced_probe_time_s;
    if (is_controlled_probe && !flow.in_probe_up &&
        now_s >= g_observation_start_s) {
      flow.in_probe_up = true;
      flow.probe_start_s = snapshot.probe_phase_start_time_s > 0.0
          ? snapshot.probe_phase_start_time_s
          : now_s;
      flow.probe_start_sent = snapshot.sent_bytes;
      flow.probe_start_acked = snapshot.acked_bytes;
      flow.srtt_sum_us = 0.0;
      flow.srtt_samples = 0;
      flow.max_queue_bytes = queue_bytes;
    } else if (!is_probe_up && flow.in_probe_up) {
      CloseProbeInterval(flow, now_s);
    }

    if (flow.in_probe_up) {
      flow.srtt_sum_us += static_cast<double>(snapshot.srtt_us);
      ++flow.srtt_samples;
      flow.max_queue_bytes = std::max<uint64_t>(flow.max_queue_bytes,
                                                queue_bytes);
    }

    g_timeseries << g_run_id << "," << g_seed << "," << std::fixed
                 << std::setprecision(9) << now_s << "," << flow.flow_id
                 << "," << flow.probe_order << "," << ModeName(snapshot.bbr_state)
                 << "," << snapshot.probe_phase << "," << std::setprecision(6)
                 << snapshot.pacing_gain << "," << snapshot.pacing_rate_bps
                 << "," << send_rate_bps << "," << snapshot.delivery_rate_bps
                 << "," << snapshot.cwnd_bytes << "," << snapshot.inflight_bytes
                 << "," << snapshot.srtt_us << "," << snapshot.min_rtt_us
                 << "," << snapshot.delivered_bytes << ","
                 << snapshot.sent_bytes << "," << snapshot.acked_bytes << ","
                 << sent_delta << "," << acked_delta << "," << g_bin_s
                 << "," << queue_bytes << "," << snapshot.lost_bytes << ","
                 << snapshot.ecn_bytes_in_round << ","
                 << std::setprecision(9) << snapshot.last_ack_time_s << "\n";
  }

  if (now_s + g_bin_s <= g_sim_time_s + 1e-12) {
    Simulator::Schedule(Seconds(g_bin_s), &Sample);
  }
}

uint32_t
PacketsForBytes(uint64_t bytes)
{
  const uint64_t packets = (bytes + dqc::kDefaultTCPMSS - 1) / dqc::kDefaultTCPMSS;
  return static_cast<uint32_t>(std::max<uint64_t>(packets, 2000));
}

}  // namespace

int
main(int argc, char* argv[])
{
  std::string output_dir = "experiments/bbrv2_probe_order/results/raw";
  uint64_t access_bps = 800000000000ULL;
  double base_rtt_s = 0.0002;
  double bottleneck_delay_s = 0.00005;
  double app_start_s = 0.0;
  double probe_delta_rtt = 1.0;
  double forced_probe_duration_rtt = 1.0;
  double queue_bdp = 4.0;
  uint32_t send_buffer_mib = 512;
  uint32_t fill_batch = 8192;
  uint32_t max_cwnd_bdp = 4;

  CommandLine cmd;
  cmd.AddValue("runId", "Run id written to CSV", g_run_id);
  cmd.AddValue("seed", "Seed used to shuffle flow_id to probe_order", g_seed);
  cmd.AddValue("nFlows", "Number of long BBRv2 flows", g_n_flows);
  cmd.AddValue("bottleneckBps", "Bottleneck link rate in bit/s", g_bottleneck_bps);
  cmd.AddValue("accessBps", "Access link rate in bit/s", access_bps);
  cmd.AddValue("baseRtt", "End-to-end base RTT in seconds", base_rtt_s);
  cmd.AddValue("bottleneckDelay", "One-way bottleneck delay in seconds", bottleneck_delay_s);
  cmd.AddValue("queueBdp", "DropTail bottleneck queue in BDP multiples", queue_bdp);
  cmd.AddValue("simTime", "Simulation duration in seconds", g_sim_time_s);
  cmd.AddValue("observationStart", "Stable-window base time in seconds", g_observation_start_s);
  cmd.AddValue("bin", "Sampling bin in seconds", g_bin_s);
  cmd.AddValue("probeDeltaRtt", "Probe-order spacing in base RTT multiples", probe_delta_rtt);
  cmd.AddValue("probeDurationRtt", "Minimum forced PROBE_UP duration in base RTT multiples", forced_probe_duration_rtt);
  cmd.AddValue("sendBufferMiB", "Per-flow DQC send buffer size in MiB", send_buffer_mib);
  cmd.AddValue("fillBatch", "Packets inserted whenever the DQC stream asks for data", fill_batch);
  cmd.AddValue("maxCwndBdp", "Per-flow BBRv2 max cwnd in per-flow BDP multiples", max_cwnd_bdp);
  cmd.AddValue("outputDir", "Raw output directory", output_dir);
  cmd.Parse(argc, argv);

  if (g_n_flows == 0 || g_n_flows > 1024) {
    std::cerr << "nFlows must be in [1, 1024]\n";
    return 1;
  }

  const std::string run_prefix =
      output_dir + "/run_" + std::to_string(g_run_id) + "_seed_" +
      std::to_string(g_seed);
  g_timeseries.open(run_prefix + "_timeseries.csv");
  g_intervals.open(run_prefix + "_probe_intervals.csv");
  if (!g_timeseries.is_open() || !g_intervals.is_open()) {
    std::cerr << "Failed to open output CSV files under " << output_dir << "\n";
    return 1;
  }

  g_timeseries
      << "run_id,seed,time,flow_id,probe_order,bbr_state,probe_phase,"
      << "pacing_gain,pacing_rate_bps,send_rate_bps,delivery_rate_bps,"
      << "cwnd_bytes,inflight_bytes,srtt_us,min_rtt_us,delivered_bytes,"
      << "sent_bytes,acked_bytes,sent_bytes_in_bin,acked_bytes_in_bin,"
      << "bin_duration,queue_bytes,lost_bytes,ecn_bytes_in_round,last_ack_time\n";
  g_intervals
      << "run_id,seed,flow_id,probe_order,forced_probe_time,"
      << "probe_start_time,probe_end_time,probe_duration,"
      << "bytes_sent_in_probe,bytes_acked_in_probe,avg_srtt_us,"
      << "max_queue_bytes\n";

  RngSeedManager::SetSeed(g_seed);
  RngSeedManager::SetRun(g_run_id);

  const double one_way_access_delay_s =
      std::max(0.0, (base_rtt_s / 2.0 - bottleneck_delay_s) / 2.0);
  const uint64_t bdp_bytes =
      static_cast<uint64_t>((static_cast<long double>(g_bottleneck_bps) *
                             static_cast<long double>(base_rtt_s)) /
                            8.0L);
  const uint32_t queue_bytes =
      static_cast<uint32_t>(std::max<long double>(
          1.0L,
          static_cast<long double>(bdp_bytes) * queue_bdp));
  const uint64_t per_flow_bdp_bytes = std::max<uint64_t>(1, bdp_bytes / g_n_flows);
  const uint32_t max_cwnd_packets =
      PacketsForBytes(per_flow_bdp_bytes * std::max<uint32_t>(1, max_cwnd_bdp));
  const uint32_t send_buffer_bytes =
      std::max<uint32_t>(send_buffer_mib * 1024u * 1024u,
                         static_cast<uint32_t>(std::min<uint64_t>(
                             per_flow_bdp_bytes * 2, UINT32_MAX)));

  NodeContainer senders;
  NodeContainer routers;
  NodeContainer receivers;
  senders.Create(g_n_flows);
  routers.Create(2);
  receivers.Create(g_n_flows);

  InternetStackHelper internet;
  internet.Install(senders);
  internet.Install(routers);
  internet.Install(receivers);

  PointToPointHelper access;
  access.SetDeviceAttribute("DataRate", DataRateValue(DataRate(access_bps)));
  access.SetChannelAttribute("Delay", TimeValue(Seconds(one_way_access_delay_s)));
  access.SetQueue("ns3::DropTailQueue",
                  "Mode",
                  StringValue("QUEUE_MODE_BYTES"),
                  "MaxBytes",
                  UintegerValue(std::max<uint32_t>(queue_bytes, 1024 * 1024)));

  PointToPointHelper bottleneck;
  bottleneck.SetDeviceAttribute("DataRate", DataRateValue(DataRate(g_bottleneck_bps)));
  bottleneck.SetChannelAttribute("Delay", TimeValue(Seconds(bottleneck_delay_s)));
  bottleneck.SetQueue("ns3::DropTailQueue",
                      "Mode",
                      StringValue("QUEUE_MODE_BYTES"),
                      "MaxBytes",
                      UintegerValue(queue_bytes));

  Ipv4AddressHelper ipv4;
  TrafficControlHelper tch;

  for (uint32_t i = 0; i < g_n_flows; ++i) {
    NodeContainer pair(senders.Get(i), routers.Get(0));
    NetDeviceContainer dev = access.Install(pair);
    std::ostringstream subnet;
    subnet << "10.1." << (i + 1) << ".0";
    ipv4.SetBase(subnet.str().c_str(), "255.255.255.0");
    ipv4.Assign(dev);
    tch.Uninstall(dev);
  }

  NetDeviceContainer bottleneck_dev =
      bottleneck.Install(NodeContainer(routers.Get(0), routers.Get(1)));
  ipv4.SetBase("10.2.0.0", "255.255.255.0");
  ipv4.Assign(bottleneck_dev);
  tch.Uninstall(bottleneck_dev);
  Ptr<PointToPointNetDevice> bottleneck_netdev =
      DynamicCast<PointToPointNetDevice>(bottleneck_dev.Get(0));
  if (bottleneck_netdev) {
    g_bottleneck_queue = bottleneck_netdev->GetQueue();
  }

  for (uint32_t i = 0; i < g_n_flows; ++i) {
    NodeContainer pair(routers.Get(1), receivers.Get(i));
    NetDeviceContainer dev = access.Install(pair);
    std::ostringstream subnet;
    subnet << "10.3." << (i + 1) << ".0";
    ipv4.SetBase(subnet.str().c_str(), "255.255.255.0");
    ipv4.Assign(dev);
    tch.Uninstall(dev);
  }

  Ipv4GlobalRoutingHelper::PopulateRoutingTables();

  std::vector<uint32_t> order_by_flow = BuildProbeOrderByFlow(g_n_flows, g_seed);
  g_flows.resize(g_n_flows);
  std::fill(std::begin(g_last_sent_bytes), std::end(g_last_sent_bytes), 0);
  std::fill(std::begin(g_last_acked_bytes), std::end(g_last_acked_bytes), 0);

  for (uint32_t i = 0; i < g_n_flows; ++i) {
    Ptr<DqcSender> sender =
        InstallFlow(senders.Get(i),
                    receivers.Get(i),
                    i,
                    static_cast<uint16_t>(10000 + i),
                    static_cast<uint16_t>(20000 + i),
                    app_start_s,
                    g_sim_time_s,
                    send_buffer_bytes,
                    fill_batch,
                    max_cwnd_packets);
    const uint32_t probe_order = order_by_flow[i];
    const double forced_time =
        g_observation_start_s + (probe_order - 1) * probe_delta_rtt * base_rtt_s;
    sender->SetBbr2ForcedProbeUp(forced_time,
                                 forced_probe_duration_rtt * base_rtt_s);
    g_flows[i].sender = sender;
    g_flows[i].flow_id = i;
    g_flows[i].probe_order = probe_order;
    g_flows[i].forced_probe_time_s = forced_time;
  }

  std::cout << "BBRv2 probe-order run_id=" << g_run_id
            << " seed=" << g_seed << " nFlows=" << g_n_flows
            << " bottleneckBps=" << g_bottleneck_bps
            << " baseRtt=" << base_rtt_s
            << " queueBytes=" << queue_bytes
            << " maxCwndPackets=" << max_cwnd_packets
            << " outputPrefix=" << run_prefix << std::endl;

  Simulator::Schedule(Seconds(g_bin_s), &Sample);
  Simulator::Stop(Seconds(g_sim_time_s));
  Simulator::Run();
  for (FlowRuntime& flow : g_flows) {
    CloseProbeInterval(flow, g_sim_time_s);
  }
  Simulator::Destroy();
  return 0;
}
