/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Test 3: dynamic propagation-RTT experiment.
 *
 * Four long-lived flows share a fixed bottleneck.  Only the propagation
 * delay of the access links changes at configured boundaries; bottleneck
 * capacity, queue bytes, and the population remain unchanged.
 */

#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/dqc-module.h"
#include "ns3/fbbr_config_loader.h"
#include "ns3/internet-module.h"
#include "ns3/ipv4-global-routing-helper.h"
#include "ns3/network-module.h"
#include "ns3/point-to-point-channel.h"
#include "ns3/point-to-point-module.h"
#include "ns3/point-to-point-net-device.h"
#include "ns3/traffic-control-module.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <exception>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

using namespace ns3;

namespace {

const uint32_t kFixedFlows = 4;
const char kDefaultFBBRConfig[] =
    "/home/wkd/FreqBBR/NS3.27/examples/CCconfig/fbbr_default.conf";

struct FlowRuntime {
  Ptr<DqcSender> sender;
  Ptr<DqcReceiver> receiver;
  uint32_t flow_id = 0;
};

struct PropagationRttStage {
  double start_s = 0.0;
  double base_rtt_s = 0.0;
};

std::string
FileToken(const std::string& value)
{
  std::string token;
  token.reserve(value.size());
  for (char character : value) {
    if ((character >= 'a' && character <= 'z') ||
        (character >= 'A' && character <= 'Z') ||
        (character >= '0' && character <= '9')) {
      token.push_back(character);
    } else {
      token.push_back('_');
    }
  }
  return token;
}

std::string
CsvEscape(const std::string& value)
{
  if (value.find_first_of(",\"\n\r") == std::string::npos) {
    return value;
  }
  std::string escaped = "\"";
  for (char character : value) {
    if (character == '\"') {
      escaped += "\"\"";
    } else {
      escaped.push_back(character);
    }
  }
  escaped += "\"";
  return escaped;
}

dqc::CongestionControlType
ResolveAlgorithm(const std::string& algorithm)
{
  if (algorithm == "BBR-R") {
    return dqc::kBBRR;
  }
  if (algorithm == "oBBR") {
    return dqc::kOBBR;
  }
  if (algorithm == "BBRv2+") {
    return dqc::kBBRv2Plus;
  }
  if (algorithm == "CUBIC") {
    return dqc::kCubicBytes;
  }
  if (algorithm == "BBRv2-formal" || algorithm == "BBRv2") {
    return dqc::kBBRv2;
  }
  if (algorithm == "FBBR") {
    return dqc::kFBBR;
  }
  return dqc::kBBRv2;
}

bool
IsKnownAlgorithm(const std::string& algorithm)
{
  return algorithm == "BBR-R" || algorithm == "oBBR" ||
      algorithm == "BBRv2+" || algorithm == "CUBIC" ||
      algorithm == "BBRv2-formal" || algorithm == "BBRv2" ||
      algorithm == "FBBR";
}

bool
IsFBBRAlgorithm(const std::string& algorithm)
{
  return algorithm == "FBBR";
}

bool
LoadExperimentFBBRConfig(const std::string& path, dqc::FBBRConfig* config)
{
  std::string error;
  if (!dqc::LoadFBBRConfigFile(path, config, &error)) {
    std::cerr << "[fbbr-config error] " << error << std::endl;
    return false;
  }
  std::cout << "[fbbr-config] loaded " << path << std::endl;
  return true;
}

bool
ParsePropagationRttProfile(const std::string& profile,
                           std::vector<PropagationRttStage>* stages,
                           std::string* error)
{
  if (stages == nullptr || error == nullptr) {
    return false;
  }
  stages->clear();
  error->clear();
  std::stringstream entries(profile);
  std::string entry;
  while (std::getline(entries, entry, ',')) {
    const size_t separator = entry.find(':');
    if (separator == std::string::npos || separator == 0 ||
        separator + 1 >= entry.size()) {
      *error = "each propagationRttProfile entry must be start_s:base_rtt_s";
      return false;
    }
    try {
      PropagationRttStage stage;
      stage.start_s = std::stod(entry.substr(0, separator));
      stage.base_rtt_s = std::stod(entry.substr(separator + 1));
      if (!std::isfinite(stage.start_s) || !std::isfinite(stage.base_rtt_s) ||
          stage.start_s < 0.0 || stage.base_rtt_s <= 0.0) {
        *error = "propagationRttProfile values must be finite and positive";
        return false;
      }
      if (!stages->empty() && stage.start_s <= stages->back().start_s) {
        *error = "propagationRttProfile starts must be strictly increasing";
        return false;
      }
      stages->push_back(stage);
    } catch (const std::exception&) {
      *error = "propagationRttProfile contains an invalid number";
      return false;
    }
  }
  if (stages->empty() || std::abs(stages->front().start_s) > 1e-9) {
    *error = "propagationRttProfile must begin with 0:<base_rtt_s>";
    return false;
  }
  return true;
}

double
OneWayAccessDelayForBaseRtt(double base_rtt_s, double bottleneck_delay_s)
{
  return (base_rtt_s * 0.5 - bottleneck_delay_s) * 0.5;
}

void
ApplyAccessPropagationDelay(std::vector<Ptr<PointToPointChannel> > channels,
                            Time delay,
                            double base_rtt_s)
{
  for (const Ptr<PointToPointChannel>& channel : channels) {
    if (channel != nullptr) {
      channel->SetPropagationDelay(delay);
    }
  }
  std::cout << "[dynamic-rtt] t=" << Simulator::Now().GetSeconds()
            << " baseRtt=" << base_rtt_s
            << " accessOneWay=" << delay.GetSeconds() << std::endl;
}

class DynamicRttExperiment {
 public:
  DynamicRttExperiment(const std::string& algorithm,
                       uint32_t seed,
                       uint32_t run_id,
                       uint64_t bottleneck_bps,
                       uint32_t queue_bytes,
                       double bottleneck_delay_s,
                       double simulation_time_s,
                       double sample_interval_s,
                       double measurement_guard_s,
                       const std::vector<PropagationRttStage>& rtt_stages,
                       Ptr<Queue<Packet> > bottleneck_queue,
                       const std::string& output_prefix)
      : algorithm_(algorithm),
        ideal_(algorithm == "BBRv2-formal"),
        seed_(seed),
        run_id_(run_id),
        bottleneck_bps_(bottleneck_bps),
        queue_bytes_(queue_bytes),
        bottleneck_delay_s_(bottleneck_delay_s),
        simulation_time_s_(simulation_time_s),
        sample_interval_s_(sample_interval_s),
        measurement_guard_s_(measurement_guard_s),
        rtt_stages_(rtt_stages),
        bottleneck_queue_(bottleneck_queue),
        output_prefix_(output_prefix) {}

  void SetFlows(const std::vector<FlowRuntime>& flows)
  {
    flows_ = flows;
  }

  void OpenOutputs()
  {
    profile_.open((output_prefix_ + "_rtt_profile.csv").c_str());
    timeseries_.open((output_prefix_ + "_rtt_timeseries.csv").c_str());
    controller_trace_.open((output_prefix_ + "_controller_trace.csv").c_str());
    summary_.open((output_prefix_ + "_run_summary.csv").c_str());
    metadata_.open((output_prefix_ + "_metadata.json").c_str());
    if (!profile_.is_open() || !timeseries_.is_open() ||
        !controller_trace_.is_open() || !summary_.is_open() ||
        !metadata_.is_open()) {
      std::cerr << "Failed to open Test 3 output files under "
                << output_prefix_ << std::endl;
      std::exit(1);
    }

    profile_ << "algorithm,mode,seed,run_id,stage_index,stage_start_s,"
             << "stage_end_s,configured_base_rtt_s,configured_base_rtt_ms,"
             << "access_one_way_delay_s,expected_bdp_bytes\n";
    for (size_t index = 0; index < rtt_stages_.size(); ++index) {
      const PropagationRttStage& stage = rtt_stages_[index];
      const double stage_end_s = index + 1 < rtt_stages_.size()
          ? rtt_stages_[index + 1].start_s
          : simulation_time_s_;
      const double access_delay_s =
          OneWayAccessDelayForBaseRtt(stage.base_rtt_s, bottleneck_delay_s_);
      const uint64_t expected_bdp_bytes = static_cast<uint64_t>(
          static_cast<long double>(bottleneck_bps_) * stage.base_rtt_s / 8.0L);
      profile_ << algorithm_ << "," << ModeName() << "," << seed_ << ","
               << run_id_ << "," << index << "," << std::fixed
               << std::setprecision(6) << stage.start_s << "," << stage_end_s
               << "," << stage.base_rtt_s << ","
               << stage.base_rtt_s * 1000.0 << "," << access_delay_s << ","
               << expected_bdp_bytes << "\n";
    }

    timeseries_ << "algorithm,mode,seed,run_id,time_s,stage_index,"
                << "configured_base_rtt_s,configured_base_rtt_ms,"
                << "expected_bdp_bytes,aggregate_inflight_bytes,"
                << "excess_inflight_bytes,queue_bytes,queue_delay_ms,"
                << "sum_pacing_bps,sum_max_bw_bps,snapshot_flow_count,"
                << "mean_srtt_us,mean_min_rtt_us,mean_latest_rtt_us";
    for (uint32_t flow_id = 1; flow_id <= kFixedFlows; ++flow_id) {
      timeseries_ << ",flow" << flow_id << "_received_bytes";
    }
    timeseries_ << "\n";
    controller_trace_
        << "algorithm,mode,seed,run_id,time_s,stage_index,flow_id,bbr_state,"
        << "probe_phase,pacing_gain,pacing_rate_bps,max_bw_bps,"
        << "bandwidth_estimate_bps,delivery_rate_bps,cwnd_bytes,"
        << "inflight_bytes,transport_srtt_us,transport_min_rtt_us,"
        << "transport_latest_rtt_us,"
        << "model_min_rtt_us,model_min_rtt_timestamp_s,inflight_hi_bytes,"
        << "inflight_lo_bytes,cwnd_mode_cap_bytes,cwnd_global_cap_bytes,"
        << "fbbr_beq_valid,fbbr_beq_bps,fbbr_injection_baseline_bps,"
        << "fbbr_beq_source,fbbr_waveform_last_action,delivered_bytes,"
        << "lost_bytes,last_ack_time_s,probe_phase_start_time_s\n";
    summary_ << "algorithm,mode,seed,run_id,simulation_time_s,active_flows,"
             << "capacity_bps,queue_bytes,profile_stages,queue_drop_packets,"
             << "queue_drop_bytes,validation_pass\n";
  }

  void StartMeasurement()
  {
    measurement_active_ = true;
  }

  void FinishMeasurement()
  {
    measurement_active_ = false;
  }

  void OnQueueDrop(Ptr<const Packet> packet)
  {
    ++queue_drop_packets_;
    if (packet != nullptr) {
      queue_drop_bytes_ += packet->GetSize();
    }
  }

  void Sample()
  {
    const double now_s = Simulator::Now().GetSeconds();
    if (measurement_active_) {
      WriteSample(now_s);
    }
    if (now_s + sample_interval_s_ < simulation_time_s_ + 1e-12) {
      Simulator::Schedule(Seconds(sample_interval_s_),
                          &DynamicRttExperiment::Sample, this);
    }
  }

  bool Finish()
  {
    const bool valid = sample_count_ > 0 && !rtt_stages_.empty();
    summary_ << algorithm_ << "," << ModeName() << "," << seed_ << ","
             << run_id_ << "," << simulation_time_s_ << "," << kFixedFlows
             << "," << bottleneck_bps_ << "," << queue_bytes_ << ","
             << rtt_stages_.size() << "," << queue_drop_packets_ << ","
             << queue_drop_bytes_ << "," << (valid ? 1 : 0) << "\n";
    metadata_ << "{\n"
              << "  \"experiment\": \"test3-dynamic-rtt\",\n"
              << "  \"algorithm\": \"" << algorithm_ << "\",\n"
              << "  \"mode\": \"" << ModeName() << "\",\n"
              << "  \"seed\": " << seed_ << ",\n"
              << "  \"run_id\": " << run_id_ << ",\n"
              << "  \"simulation_time_s\": " << simulation_time_s_ << ",\n"
              << "  \"active_flows\": " << kFixedFlows << ",\n"
              << "  \"capacity_bps\": " << bottleneck_bps_ << ",\n"
              << "  \"queue_bytes\": " << queue_bytes_ << ",\n"
              << "  \"bottleneck_delay_s\": " << bottleneck_delay_s_ << ",\n"
              << "  \"profile_stages\": " << rtt_stages_.size() << ",\n"
              << "  \"sample_count\": " << sample_count_ << ",\n"
              << "  \"validation_pass\": " << (valid ? "true" : "false")
              << "\n}\n";
    return valid;
  }

 private:
  std::string ModeName() const
  {
    return ideal_ ? "ideal" : "original";
  }

  size_t StageIndexForTime(double time_s) const
  {
    size_t index = 0;
    for (size_t candidate = 1; candidate < rtt_stages_.size(); ++candidate) {
      if (time_s + 1e-9 < rtt_stages_[candidate].start_s) {
        break;
      }
      index = candidate;
    }
    return index;
  }

  void WriteSample(double now_s)
  {
    const size_t stage_index = StageIndexForTime(now_s);
    const PropagationRttStage& stage = rtt_stages_[stage_index];
    const uint64_t expected_bdp_bytes = static_cast<uint64_t>(
        static_cast<long double>(bottleneck_bps_) * stage.base_rtt_s / 8.0L);
    const uint64_t queue_bytes = bottleneck_queue_ != nullptr
        ? bottleneck_queue_->GetNBytes()
        : 0;
    uint64_t aggregate_inflight = 0;
    uint64_t sum_pacing = 0;
    uint64_t sum_max_bw = 0;
    uint64_t sum_srtt_us = 0;
    uint64_t sum_min_rtt_us = 0;
    uint64_t sum_latest_rtt_us = 0;
    uint32_t snapshot_flow_count = 0;
    for (const FlowRuntime& flow : flows_) {
      DqcSender::Bbr2ExperimentSnapshot snapshot;
      if (flow.sender->GetBbr2ExperimentSnapshot(&snapshot)) {
        ++snapshot_flow_count;
        aggregate_inflight += snapshot.inflight_bytes;
        sum_pacing += snapshot.pacing_rate_bps;
        sum_max_bw += snapshot.max_bw_bps;
        sum_srtt_us += snapshot.srtt_us;
        sum_min_rtt_us += snapshot.min_rtt_us;
        sum_latest_rtt_us += snapshot.latest_rtt_us;
        WriteControllerTrace(now_s, stage_index, flow.flow_id, snapshot);
        continue;
      }
      uint64_t srtt_us = 0;
      uint64_t min_rtt_us = 0;
      uint64_t latest_rtt_us = 0;
      if (flow.sender->GetTransportRttSnapshot(
              &srtt_us, &min_rtt_us, &latest_rtt_us)) {
        ++snapshot_flow_count;
        sum_srtt_us += srtt_us;
        sum_min_rtt_us += min_rtt_us;
        sum_latest_rtt_us += latest_rtt_us;
      }
    }
    const uint64_t excess_inflight = aggregate_inflight > expected_bdp_bytes
        ? aggregate_inflight - expected_bdp_bytes
        : 0;
    const double queue_delay_ms = static_cast<double>(queue_bytes) * 8.0 /
        static_cast<double>(bottleneck_bps_) * 1000.0;
    const double mean_srtt_us = snapshot_flow_count > 0
        ? static_cast<double>(sum_srtt_us) / snapshot_flow_count
        : 0.0;
    const double mean_min_rtt_us = snapshot_flow_count > 0
        ? static_cast<double>(sum_min_rtt_us) / snapshot_flow_count
        : 0.0;
    const double mean_latest_rtt_us = snapshot_flow_count > 0
        ? static_cast<double>(sum_latest_rtt_us) / snapshot_flow_count
        : 0.0;
    timeseries_ << algorithm_ << "," << ModeName() << "," << seed_ << ","
                << run_id_ << "," << std::fixed << std::setprecision(6)
                << now_s << "," << stage_index << "," << stage.base_rtt_s
                << "," << stage.base_rtt_s * 1000.0 << ","
                << expected_bdp_bytes << "," << aggregate_inflight << ","
                << excess_inflight << "," << queue_bytes << ","
                << queue_delay_ms << "," << sum_pacing << "," << sum_max_bw
                << "," << snapshot_flow_count << "," << mean_srtt_us << ","
                << mean_min_rtt_us << "," << mean_latest_rtt_us;
    for (const FlowRuntime& flow : flows_) {
      timeseries_ << "," << flow.receiver->GetReceivedBytes();
    }
    timeseries_ << "\n";
    ++sample_count_;
  }

  void WriteControllerTrace(
      double now_s,
      size_t stage_index,
      uint32_t flow_id,
      const DqcSender::Bbr2ExperimentSnapshot& snapshot)
  {
    controller_trace_ << algorithm_ << "," << ModeName() << "," << seed_
                      << "," << run_id_ << "," << std::fixed
                      << std::setprecision(6) << now_s << "," << stage_index
                      << "," << flow_id << "," << snapshot.bbr_state << ","
                      << CsvEscape(snapshot.probe_phase) << ","
                      << snapshot.pacing_gain << ","
                      << snapshot.pacing_rate_bps << ","
                      << snapshot.max_bw_bps << ","
                      << snapshot.bandwidth_estimate_bps << ","
                      << snapshot.delivery_rate_bps << ","
                      << snapshot.cwnd_bytes << ","
                      << snapshot.inflight_bytes << ","
                      << snapshot.srtt_us << ","
                      << snapshot.min_rtt_us << ","
                      << snapshot.latest_rtt_us << ","
                      << snapshot.model_min_rtt_us << ","
                      << snapshot.model_min_rtt_timestamp_s << ","
                      << snapshot.inflight_hi_bytes << ","
                      << snapshot.inflight_lo_bytes << ","
                      << snapshot.cwnd_mode_cap_bytes << ","
                      << snapshot.cwnd_global_cap_bytes << ","
                      << (snapshot.fbbr_beq_valid ? 1 : 0) << ","
                      << snapshot.fbbr_beq_bps << ","
                      << snapshot.fbbr_injection_baseline_bps << ","
                      << CsvEscape(snapshot.fbbr_beq_source) << ","
                      << CsvEscape(snapshot.fbbr_waveform_last_action) << ","
                      << snapshot.delivered_bytes << ","
                      << snapshot.lost_bytes << ","
                      << snapshot.last_ack_time_s << ","
                      << snapshot.probe_phase_start_time_s << "\n";
  }

  std::string algorithm_;
  bool ideal_ = false;
  uint32_t seed_ = 0;
  uint32_t run_id_ = 0;
  uint64_t bottleneck_bps_ = 0;
  uint32_t queue_bytes_ = 0;
  double bottleneck_delay_s_ = 0.0;
  double simulation_time_s_ = 0.0;
  double sample_interval_s_ = 0.0;
  double measurement_guard_s_ = 0.0;
  std::vector<PropagationRttStage> rtt_stages_;
  Ptr<Queue<Packet> > bottleneck_queue_;
  std::string output_prefix_;
  std::vector<FlowRuntime> flows_;
  bool measurement_active_ = false;
  uint64_t queue_drop_packets_ = 0;
  uint64_t queue_drop_bytes_ = 0;
  uint64_t sample_count_ = 0;
  std::ofstream profile_;
  std::ofstream timeseries_;
  std::ofstream controller_trace_;
  std::ofstream summary_;
  std::ofstream metadata_;
};

FlowRuntime
InstallFlow(dqc::CongestionControlType cc_type,
            const std::string& algorithm,
            const dqc::FBBRConfig& fbbr_config,
            Ptr<Node> sender,
            Ptr<Node> receiver,
            uint32_t flow_index,
            double simulation_time_s,
            uint32_t send_buffer_bytes)
{
  const uint16_t port = static_cast<uint16_t>(9000 + flow_index);
  Ptr<DqcReceiver> receiver_app = CreateObject<DqcReceiver>(1000);
  receiver->AddApplication(receiver_app);
  receiver_app->Bind(port);
  receiver_app->SetStartTime(Seconds(0.001));
  receiver_app->SetStopTime(Seconds(simulation_time_s));

  Ptr<DqcSender> sender_app = CreateObject<DqcSender>(cc_type, false, true);
  sender->AddApplication(sender_app);
  sender_app->SetSenderId(flow_index + 1);
  sender_app->Bind(static_cast<uint16_t>(10000 + flow_index));
  sender_app->ConfigurePeer(receiver_app->GetLocalAddress().GetIpv4(), port);
  sender_app->SetStreamSendBufferBytes(send_buffer_bytes);
  if (IsFBBRAlgorithm(algorithm)) {
    sender_app->ConfigureFBBR(fbbr_config, flow_index);
  }
  sender_app->SetStartTime(Seconds(0.001));
  sender_app->SetStopTime(Seconds(simulation_time_s));

  FlowRuntime flow;
  flow.sender = sender_app;
  flow.receiver = receiver_app;
  flow.flow_id = flow_index + 1;
  return flow;
}

}  // namespace

int
main(int argc, char* argv[])
{
  std::string algorithm = "BBRv2";
  std::string output_dir = "results/test3/raw/DYN-RTT";
  std::string propagation_rtt_profile =
      "0:0.040,60:0.120,120:0.030,180:0.080,240:0.040";
  std::string fbbr_config = kDefaultFBBRConfig;
  uint32_t seed = 1;
  uint32_t run_id = 1;
  uint64_t bottleneck_bps = 100000000ULL;
  uint64_t access_bps = 1000000000ULL;
  double bottleneck_delay_s = 0.010;
  double initial_queue_bdp = 2.0;
  double simulation_time_s = 300.0;
  double sample_interval_s = 0.100;
  double measurement_guard_s = 0.010;

  CommandLine cmd;
  cmd.AddValue("algorithm",
               "BBR-R, oBBR, BBRv2+, CUBIC, BBRv2-formal, BBRv2, or FBBR",
               algorithm);
  cmd.AddValue("outputDir", "Directory for this controller's raw output", output_dir);
  cmd.AddValue("seed", "Deterministic DQC and ns-3 random seed", seed);
  cmd.AddValue("runId", "Run identifier persisted in output", run_id);
  cmd.AddValue("bottleneckBps", "Fixed bottleneck capacity in bit/s", bottleneck_bps);
  cmd.AddValue("accessBps", "Fixed access-link capacity in bit/s", access_bps);
  cmd.AddValue("bottleneckDelay", "Fixed bottleneck one-way delay in seconds",
               bottleneck_delay_s);
  cmd.AddValue("initialQueueBdp",
               "Queue bytes expressed in BDPs of the initial RTT stage",
               initial_queue_bdp);
  cmd.AddValue("simulationTime", "Total simulation duration in seconds",
               simulation_time_s);
  cmd.AddValue("sampleInterval", "Raw time-series interval in seconds",
               sample_interval_s);
  cmd.AddValue("measurementGuard", "Initial and final measurement guard", measurement_guard_s);
  cmd.AddValue("propagationRttProfile",
               "Comma-separated start_s:base_rtt_s stages beginning at 0",
               propagation_rtt_profile);
  cmd.AddValue("fbbrConfig", "FBBR key=value configuration path", fbbr_config);
  cmd.Parse(argc, argv);

  std::vector<PropagationRttStage> rtt_stages;
  std::string profile_error;
  if (!ParsePropagationRttProfile(propagation_rtt_profile, &rtt_stages,
                                  &profile_error)) {
    std::cerr << "Invalid propagation RTT profile: " << profile_error << std::endl;
    return 1;
  }
  if (!IsKnownAlgorithm(algorithm) || bottleneck_bps == 0 || access_bps == 0 ||
      bottleneck_delay_s <= 0.0 || initial_queue_bdp <= 0.0 ||
      simulation_time_s <= 0.0 || sample_interval_s <= 0.0 ||
      measurement_guard_s < 0.0 ||
      2.0 * measurement_guard_s >= simulation_time_s) {
    std::cerr << "Invalid Test 3 arguments" << std::endl;
    return 1;
  }
  for (const PropagationRttStage& stage : rtt_stages) {
    if (stage.start_s >= simulation_time_s ||
        OneWayAccessDelayForBaseRtt(stage.base_rtt_s, bottleneck_delay_s) <= 0.0) {
      std::cerr << "Each RTT stage must begin before simulation end and exceed "
                << "twice the bottleneck one-way delay" << std::endl;
      return 1;
    }
  }

  const double initial_base_rtt_s = rtt_stages.front().base_rtt_s;
  const double initial_access_delay_s =
      OneWayAccessDelayForBaseRtt(initial_base_rtt_s, bottleneck_delay_s);
  const uint64_t initial_bdp_bytes = static_cast<uint64_t>(
      static_cast<long double>(bottleneck_bps) * initial_base_rtt_s / 8.0L);
  const uint32_t queue_bytes = static_cast<uint32_t>(std::max<long double>(
      1.0L, static_cast<long double>(initial_bdp_bytes) * initial_queue_bdp));
  const uint32_t send_buffer_bytes = 16U * 1024U * 1024U;
  const bool ideal = algorithm == "BBRv2-formal";
  const std::string output_prefix = output_dir + "/" + FileToken(algorithm) +
      "_seed" + std::to_string(seed) + "_run" + std::to_string(run_id);

  RngSeedManager::SetSeed(seed);
  RngSeedManager::SetRun(run_id);
  dqc::SendPacketManager::SetDeterministicRandomSeed(seed, run_id);

  NodeContainer senders;
  NodeContainer routers;
  NodeContainer receivers;
  senders.Create(kFixedFlows);
  routers.Create(2);
  receivers.Create(kFixedFlows);

  InternetStackHelper internet;
  internet.Install(senders);
  internet.Install(routers);
  internet.Install(receivers);

  PointToPointHelper access;
  access.SetDeviceAttribute("DataRate", DataRateValue(DataRate(access_bps)));
  access.SetChannelAttribute("Delay", TimeValue(Seconds(initial_access_delay_s)));
  access.SetQueue("ns3::DropTailQueue", "Mode", StringValue("QUEUE_MODE_BYTES"),
                  "MaxBytes", UintegerValue(std::max<uint32_t>(queue_bytes, 1024 * 1024)));

  PointToPointHelper bottleneck;
  bottleneck.SetDeviceAttribute("DataRate", DataRateValue(DataRate(bottleneck_bps)));
  bottleneck.SetChannelAttribute("Delay", TimeValue(Seconds(bottleneck_delay_s)));
  bottleneck.SetQueue("ns3::DropTailQueue", "Mode", StringValue("QUEUE_MODE_BYTES"),
                      "MaxBytes", UintegerValue(queue_bytes));

  Ipv4AddressHelper ipv4;
  TrafficControlHelper traffic_control;
  std::vector<Ptr<PointToPointChannel> > access_channels;
  for (uint32_t index = 0; index < kFixedFlows; ++index) {
    NodeContainer pair(senders.Get(index), routers.Get(0));
    NetDeviceContainer devices = access.Install(pair);
    Ptr<PointToPointNetDevice> device =
        DynamicCast<PointToPointNetDevice>(devices.Get(0));
    Ptr<PointToPointChannel> channel = device != nullptr
        ? DynamicCast<PointToPointChannel>(device->GetChannel())
        : nullptr;
    if (channel != nullptr) {
      access_channels.push_back(channel);
    }
    std::ostringstream subnet;
    subnet << "10.11." << (index + 1) << ".0";
    ipv4.SetBase(subnet.str().c_str(), "255.255.255.0");
    ipv4.Assign(devices);
    traffic_control.Uninstall(devices);
  }

  NetDeviceContainer bottleneck_devices =
      bottleneck.Install(NodeContainer(routers.Get(0), routers.Get(1)));
  ipv4.SetBase("10.12.0.0", "255.255.255.0");
  ipv4.Assign(bottleneck_devices);
  traffic_control.Uninstall(bottleneck_devices);
  Ptr<PointToPointNetDevice> bottleneck_device =
      DynamicCast<PointToPointNetDevice>(bottleneck_devices.Get(0));
  Ptr<Queue<Packet> > bottleneck_queue = bottleneck_device != nullptr
      ? bottleneck_device->GetQueue()
      : nullptr;

  for (uint32_t index = 0; index < kFixedFlows; ++index) {
    NodeContainer pair(routers.Get(1), receivers.Get(index));
    NetDeviceContainer devices = access.Install(pair);
    Ptr<PointToPointNetDevice> device =
        DynamicCast<PointToPointNetDevice>(devices.Get(0));
    Ptr<PointToPointChannel> channel = device != nullptr
        ? DynamicCast<PointToPointChannel>(device->GetChannel())
        : nullptr;
    if (channel != nullptr) {
      access_channels.push_back(channel);
    }
    std::ostringstream subnet;
    subnet << "10.13." << (index + 1) << ".0";
    ipv4.SetBase(subnet.str().c_str(), "255.255.255.0");
    ipv4.Assign(devices);
    traffic_control.Uninstall(devices);
  }
  if (access_channels.size() != 2 * kFixedFlows) {
    std::cerr << "Failed to discover all access channels" << std::endl;
    return 1;
  }
  Ipv4GlobalRoutingHelper::PopulateRoutingTables();

  dqc::FBBRConfig fbbr_config_values;
  if (IsFBBRAlgorithm(algorithm) &&
      !LoadExperimentFBBRConfig(fbbr_config, &fbbr_config_values)) {
    return 1;
  }
  const dqc::CongestionControlType cc_type = ResolveAlgorithm(algorithm);
  std::vector<FlowRuntime> flows;
  flows.reserve(kFixedFlows);
  for (uint32_t index = 0; index < kFixedFlows; ++index) {
    flows.push_back(InstallFlow(cc_type, algorithm, fbbr_config_values,
                                senders.Get(index), receivers.Get(index), index,
                                simulation_time_s, send_buffer_bytes));
  }

  if (ideal) {
    const double settle_s = 20.0;
    const double gap_s = 0.150;
    for (uint32_t index = 0; index < kFixedFlows; ++index) {
      flows[index].sender->SetBbr2StrictProbeUp(
          index + 1, kFixedFlows, settle_s + index * gap_s, 0.040, 0.080);
    }
  }

  DynamicRttExperiment experiment(
      algorithm, seed, run_id, bottleneck_bps, queue_bytes, bottleneck_delay_s,
      simulation_time_s, sample_interval_s, measurement_guard_s, rtt_stages,
      bottleneck_queue, output_prefix);
  experiment.SetFlows(flows);
  experiment.OpenOutputs();
  if (bottleneck_queue != nullptr) {
    bottleneck_queue->TraceConnectWithoutContext(
        "Drop", MakeCallback(&DynamicRttExperiment::OnQueueDrop, &experiment));
  }

  for (size_t index = 1; index < rtt_stages.size(); ++index) {
    const PropagationRttStage& stage = rtt_stages[index];
    const double access_delay_s =
        OneWayAccessDelayForBaseRtt(stage.base_rtt_s, bottleneck_delay_s);
    Simulator::Schedule(Seconds(stage.start_s), &ApplyAccessPropagationDelay,
                        access_channels, Seconds(access_delay_s),
                        stage.base_rtt_s);
  }
  Simulator::Schedule(Seconds(measurement_guard_s),
                      &DynamicRttExperiment::StartMeasurement, &experiment);
  Simulator::Schedule(Seconds(measurement_guard_s + sample_interval_s),
                      &DynamicRttExperiment::Sample, &experiment);
  Simulator::Schedule(Seconds(simulation_time_s - measurement_guard_s),
                      &DynamicRttExperiment::FinishMeasurement, &experiment);
  Simulator::Stop(Seconds(simulation_time_s));
  Simulator::Run();
  const bool valid = experiment.Finish();
  Simulator::Destroy();
  std::cout << "test3-dynamic-rtt algorithm=" << algorithm
            << " validation=" << (valid ? "PASS" : "FAIL")
            << " output=" << output_prefix << std::endl;
  return valid ? 0 : 1;
}
