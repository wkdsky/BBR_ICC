/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Test 1: dynamic congestion-control comparison.
 *
 * The active flow population follows 2 -> 4 -> 8 -> 16 -> 8 -> 4 -> 2
 * over one simulation.  Every controller keeps its native implementation.
 * BBRv2-ideal is the sole exception: it uses BBRv2 with an experiment-only
 * gate that admits one ProbeBW-UP at a time in flow-number order.
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
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <numeric>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

using namespace ns3;

namespace {

const uint32_t kMaximumFlows = 16;
const double kMinuteWindowS = 60.0;

struct StageConfig {
  uint32_t index = 0;
  uint32_t active_flows = 0;
  std::string label;
  double start_s = 0.0;
  double end_s = 0.0;
};

struct FlowLifetime {
  double start_s = 0.0;
  double stop_s = 0.0;
};

struct FlowRuntime {
  Ptr<DqcSender> sender;
  Ptr<DqcReceiver> receiver;
  uint32_t flow_id = 0;
  double start_s = 0.0;
  double stop_s = 0.0;
  int32_t active_event = -1;
};

struct ProbeEvent {
  uint32_t event_id = 0;
  uint32_t stage_index = 0;
  std::string stage_label;
  uint32_t active_flows = 0;
  uint32_t flow_id = 0;
  uint32_t probe_order = 0;
  bool theory_applicable = false;
  bool strict_controlled = false;
  bool pre_all_other_cruise = false;
  uint32_t pre_other_up_count = 0;
  double start_time_s = -1.0;
  double end_time_s = -1.0;
  double duration_s = 0.0;
  uint64_t max_bw_before_bps = 0;
  uint64_t max_bw_peak_bps = 0;
  uint64_t max_bw_after_bps = 0;
  uint64_t delivery_rate_peak_bps = 0;
  uint64_t up_pacing_rate_bps = 0;
  uint64_t sum_pacing_start_bps = 0;
  double theory_service_bps = 0.0;
  double theory_max_bw_bps = 0.0;
  double effective_service_bps = 0.0;
  double theory_error_bps = 0.0;
  double theory_error_pct = 0.0;
  uint64_t max_queue_bytes = 0;
  uint64_t max_aggregate_inflight_bytes = 0;
  uint32_t max_concurrent_up = 0;
};

struct StageAccumulator {
  StageConfig config;
  bool active = false;
  bool probe_rtt_seen = false;
  double measurement_start_s = 0.0;
  double measurement_end_s = 0.0;
  uint64_t queue_drop_packets_start = 0;
  uint64_t queue_drop_bytes_start = 0;
  uint64_t queue_drop_packets_end = 0;
  uint64_t queue_drop_bytes_end = 0;
  std::vector<uint64_t> received_bytes_start;
  std::vector<uint64_t> received_bytes_end;
  std::vector<double> aggregate_inflight_samples;
  std::vector<double> excess_inflight_samples;
  std::vector<double> queue_bytes_samples;
  std::vector<double> queue_delay_samples_ms;
  std::vector<double> sum_pacing_samples;
  std::vector<double> bandwidth_estimate_samples;
};

struct MinuteAccumulator {
  uint64_t sample_count = 0;
  double sum_active_flows = 0.0;
  double sum_aggregate_inflight_bytes = 0.0;
  double sum_excess_inflight_bytes = 0.0;
  double sum_queue_bytes = 0.0;
  double sum_queue_delay_ms = 0.0;
  double sum_pacing_bps = 0.0;
  double sum_bandwidth_estimate_bps = 0.0;
  bool throughput_recorded = false;
  std::vector<double> flow_active_duration_s;
  std::vector<uint64_t> flow_received_bytes;
  std::vector<double> flow_goodput_bps;
  double aggregate_goodput_bps = 0.0;
  double mean_flow_goodput_bps = 0.0;
  double jain_fairness = 0.0;
};

double
Mean(const std::vector<double>& values)
{
  if (values.empty()) {
    return 0.0;
  }
  return std::accumulate(values.begin(), values.end(), 0.0) /
      static_cast<double>(values.size());
}

double
Percentile(std::vector<double> values, double percentile)
{
  if (values.empty()) {
    return 0.0;
  }
  std::sort(values.begin(), values.end());
  const double rank = percentile * static_cast<double>(values.size());
  const size_t index = static_cast<size_t>(std::max(
      0.0, std::ceil(rank) - 1.0));
  return values[std::min(index, values.size() - 1)];
}

double
Maximum(const std::vector<double>& values)
{
  return values.empty() ? 0.0
                        : *std::max_element(values.begin(), values.end());
}

double
JainFairness(const std::vector<double>& values)
{
  if (values.empty()) {
    return 0.0;
  }
  double sum = 0.0;
  double sum_squares = 0.0;
  for (double value : values) {
    sum += value;
    sum_squares += value * value;
  }
  if (sum_squares <= 0.0) {
    return 0.0;
  }
  return sum * sum / (static_cast<double>(values.size()) * sum_squares);
}

std::string
FileToken(const std::string& value)
{
  std::string token;
  token.reserve(value.size());
  for (char c : value) {
    if ((c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') ||
        (c >= '0' && c <= '9')) {
      token.push_back(c);
    } else {
      token.push_back('_');
    }
  }
  return token;
}

std::string
CsvEscape(const std::string& value)
{
  std::string escaped;
  escaped.reserve(value.size() + 2);
  escaped.push_back('"');
  for (char c : value) {
    if (c == '"') {
      escaped.push_back('"');
    }
    escaped.push_back(c);
  }
  escaped.push_back('"');
  return escaped;
}

bool
IsSupportedFlowPattern(const std::string& flow_pattern)
{
  return flow_pattern == "dynamic" || flow_pattern == "8to16" ||
      flow_pattern == "4to8to16to8to4" ||
      flow_pattern == "steady1" || flow_pattern == "steady8" ||
      flow_pattern == "steady16";
}

std::vector<StageConfig>
BuildStages(double simulation_time_s, const std::string& flow_pattern)
{
  if (flow_pattern == "steady1" || flow_pattern == "steady8" ||
      flow_pattern == "steady16") {
    const uint32_t active_flows = flow_pattern == "steady1"
        ? 1
        : flow_pattern == "steady8" ? 8 : 16;
    StageConfig stage;
    stage.index = 0;
    stage.active_flows = active_flows;
    stage.label = "N" + std::to_string(active_flows) + "_steady";
    stage.start_s = 0.0;
    stage.end_s = simulation_time_s;
    return {stage};
  }

  if (flow_pattern == "8to16") {
    const uint32_t counts[] = {8, 16};
    const char* labels[] = {"N8_baseline", "N16_join"};
    const double duration = simulation_time_s / 2.0;
    std::vector<StageConfig> stages;
    stages.reserve(2);
    for (uint32_t i = 0; i < 2; ++i) {
      StageConfig stage;
      stage.index = i;
      stage.active_flows = counts[i];
      stage.label = labels[i];
      stage.start_s = static_cast<double>(i) * duration;
      stage.end_s = i == 1 ? simulation_time_s
                           : static_cast<double>(i + 1) * duration;
      stages.push_back(stage);
    }
    return stages;
  }

  if (flow_pattern == "4to8to16to8to4") {
    const uint32_t counts[] = {4, 8, 16, 8, 4};
    const char* labels[] = {
        "N4_rise", "N8_rise", "N16_peak", "N8_fall", "N4_fall"};
    const double duration = simulation_time_s / 5.0;
    std::vector<StageConfig> stages;
    stages.reserve(5);
    for (uint32_t i = 0; i < 5; ++i) {
      StageConfig stage;
      stage.index = i;
      stage.active_flows = counts[i];
      stage.label = labels[i];
      stage.start_s = static_cast<double>(i) * duration;
      stage.end_s = i == 4 ? simulation_time_s
                           : static_cast<double>(i + 1) * duration;
      stages.push_back(stage);
    }
    return stages;
  }

  const uint32_t counts[] = {2, 4, 8, 16, 8, 4, 2};
  const char* labels[] = {
      "N2_rise", "N4_rise", "N8_rise", "N16_peak",
      "N8_fall", "N4_fall", "N2_fall"};
  const double duration = simulation_time_s / 7.0;
  std::vector<StageConfig> stages;
  stages.reserve(7);
  for (uint32_t i = 0; i < 7; ++i) {
    StageConfig stage;
    stage.index = i;
    stage.active_flows = counts[i];
    stage.label = labels[i];
    stage.start_s = static_cast<double>(i) * duration;
    stage.end_s = i == 6 ? simulation_time_s
                         : static_cast<double>(i + 1) * duration;
    stages.push_back(stage);
  }
  return stages;
}

FlowLifetime
LifetimeForFlow(uint32_t flow_id,
                double stage_duration_s,
                double simulation_time_s,
                const std::string& flow_pattern)
{
  FlowLifetime lifetime;
  lifetime.start_s = 0.001;
  lifetime.stop_s = simulation_time_s;
  if (flow_pattern == "steady1" || flow_pattern == "steady8" ||
      flow_pattern == "steady16") {
    const uint32_t active_flows = flow_pattern == "steady1"
        ? 1
        : flow_pattern == "steady8" ? 8 : 16;
    if (flow_id > active_flows) {
      lifetime.start_s = simulation_time_s;
      lifetime.stop_s = simulation_time_s;
    }
    return lifetime;
  }
  if (flow_pattern == "8to16") {
    if (flow_id <= 8) {
      return lifetime;
    }
    lifetime.start_s = stage_duration_s;
    return lifetime;
  }
  if (flow_pattern == "4to8to16to8to4") {
    if (flow_id <= 4) {
      return lifetime;
    }
    if (flow_id <= 8) {
      lifetime.start_s = stage_duration_s;
      lifetime.stop_s = 4.0 * stage_duration_s;
      return lifetime;
    }
    lifetime.start_s = 2.0 * stage_duration_s;
    lifetime.stop_s = 3.0 * stage_duration_s;
    return lifetime;
  }
  if (flow_id <= 2) {
    return lifetime;
  }
  if (flow_id <= 4) {
    lifetime.start_s = stage_duration_s;
    lifetime.stop_s = 6.0 * stage_duration_s;
    return lifetime;
  }
  if (flow_id <= 8) {
    lifetime.start_s = 2.0 * stage_duration_s;
    lifetime.stop_s = 5.0 * stage_duration_s;
    return lifetime;
  }
  lifetime.start_s = 3.0 * stage_duration_s;
  lifetime.stop_s = 4.0 * stage_duration_s;
  return lifetime;
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
  if (algorithm == "BBRv2-ideal" || algorithm == "BBRv2") {
    return dqc::kBBRv2;
  }
  if (algorithm == "FBBR") {
    return dqc::kFBBR;
  }
  if (algorithm == "FBBR-ServiceFair") {
    return dqc::kFBBRServiceFair;
  }
  return dqc::kBBRv2;
}

bool
IsKnownAlgorithm(const std::string& algorithm)
{
  return algorithm == "BBR-R" || algorithm == "oBBR" ||
      algorithm == "BBRv2+" || algorithm == "CUBIC" ||
      algorithm == "BBRv2-ideal" || algorithm == "BBRv2" ||
      algorithm == "FBBR" || algorithm == "FBBR-ServiceFair";
}

bool
IsFBBRAlgorithm(const std::string& algorithm)
{
  return algorithm == "FBBR" || algorithm == "FBBR-ServiceFair";
}

dqc::FBBRConfig
BuildFBBRConfig(bool diagnostic_trace)
{
  dqc::FBBRConfig config;
  config.pacing_minimum_rate_mbps = 0.2;
  if (diagnostic_trace) {
    config.trace_gate_trace_mode = "sampled_pacing";
    config.trace_gate_trace_sample_interval_us = 20000;
    config.trace_enable_cruise_window_trace = true;
    config.trace_enable_trusted_bw_selection_trace = true;
  } else {
    config.trace_gate_trace_mode = "off";
    config.trace_enable_cruise_window_trace = false;
    config.trace_enable_trusted_bw_selection_trace = false;
  }
  return config;
}

class Experiment {
 public:
  Experiment(const std::string& algorithm,
             bool ideal,
             uint32_t seed,
             uint32_t run_id,
             uint64_t bottleneck_bps,
             uint64_t bdp_bytes,
             double simulation_time_s,
             double sample_interval_s,
             double stage_edge_guard_s,
             double ideal_settle_s,
             double strict_gap_s,
             double strict_min_up_s,
             double strict_max_up_s,
             Ptr<Queue<Packet> > bottleneck_queue,
             const std::vector<StageConfig>& stages,
             const std::string& output_prefix,
             const std::string& flow_pattern,
             bool diagnostic_trace)
      : algorithm_(algorithm),
        ideal_(ideal),
        seed_(seed),
        run_id_(run_id),
        bottleneck_bps_(bottleneck_bps),
        bdp_bytes_(bdp_bytes),
        simulation_time_s_(simulation_time_s),
        sample_interval_s_(sample_interval_s),
        stage_edge_guard_s_(stage_edge_guard_s),
        ideal_settle_s_(ideal_settle_s),
        strict_gap_s_(strict_gap_s),
        strict_min_up_s_(strict_min_up_s),
        strict_max_up_s_(strict_max_up_s),
        bottleneck_queue_(bottleneck_queue),
        stages_config_(stages),
        output_prefix_(output_prefix),
        flow_pattern_(flow_pattern),
        diagnostic_trace_(diagnostic_trace),
        stage_data_(stages.size()),
        minute_data_(static_cast<size_t>(
            std::ceil(simulation_time_s / kMinuteWindowS))),
        active_up_(kMaximumFlows, false) {
    for (size_t i = 0; i < stages_config_.size(); ++i) {
      stage_data_[i].config = stages_config_[i];
    }
  }

  void SetFlows(const std::vector<FlowRuntime>& flows)
  {
    flows_ = flows;
  }

  void StartMinuteThroughputTrace()
  {
    previous_minute_received_bytes_.assign(flows_.size(), 0);
    for (size_t i = 0; i < flows_.size(); ++i) {
      previous_minute_received_bytes_[i] =
          flows_[i].receiver->GetReceivedBytes();
    }
    for (size_t i = 0; i < minute_data_.size(); ++i) {
      const double window_end_s = std::min(
          simulation_time_s_, static_cast<double>(i + 1) * kMinuteWindowS);
      Simulator::Schedule(Seconds(window_end_s),
                          &Experiment::RecordMinuteThroughput, this, i);
    }
  }

  void EnableDiagnosticTracing()
  {
    if (!diagnostic_trace_) {
      return;
    }
    for (const FlowRuntime& flow : flows_) {
      flow.sender->ConfigureFBBRConvergenceGate(
          true, false, "sampled_pacing", 20000);
      flow.sender->SetFBBRLoadTraceFuc(
          MakeCallback(&Experiment::OnFbbrLoadTrace, this));
    }
  }

  void OpenOutputs()
  {
    events_.open((output_prefix_ + "_events.csv").c_str());
    stage_metrics_.open((output_prefix_ + "_stage_metrics.csv").c_str());
    flow_metrics_.open((output_prefix_ + "_flow_metrics.csv").c_str());
    minute_metrics_.open((output_prefix_ + "_minute_metrics.csv").c_str());
    minute_flow_metrics_.open(
        (output_prefix_ + "_minute_flow_metrics.csv").c_str());
    run_summary_.open((output_prefix_ + "_run_summary.csv").c_str());
    metadata_.open((output_prefix_ + "_metadata.json").c_str());
    if (diagnostic_trace_) {
      diagnostic_queue_trace_.open(
          (output_prefix_ + "_diagnostic_queue_trace.csv").c_str());
      diagnostic_flow_trace_.open(
          (output_prefix_ + "_diagnostic_flow_trace.csv").c_str());
      diagnostic_fbbr_trace_.open(
          (output_prefix_ + "_diagnostic_fbbr_trace.csv").c_str());
    }
    if (!events_.is_open() || !stage_metrics_.is_open() ||
        !flow_metrics_.is_open() || !minute_metrics_.is_open() ||
        !minute_flow_metrics_.is_open() ||
        !run_summary_.is_open() ||
        !metadata_.is_open() ||
        (diagnostic_trace_ &&
         (!diagnostic_queue_trace_.is_open() ||
          !diagnostic_flow_trace_.is_open() ||
          !diagnostic_fbbr_trace_.is_open()))) {
      std::cerr << "Failed to open output files under " << output_prefix_
                << std::endl;
      std::exit(1);
    }

    events_
        << "algorithm,mode,seed,run_id,stage_index,stage_label,active_flows,event_id,"
        << "flow_id,probe_order,theory_applicable,strict_controlled,"
        << "pre_all_other_cruise,pre_other_up_count,start_time_s,end_time_s,"
        << "duration_s,max_bw_before_bps,max_bw_peak_bps,max_bw_after_bps,"
        << "delivery_rate_peak_bps,up_pacing_rate_bps,sum_pacing_start_bps,"
        << "theory_service_bps,theory_max_bw_bps,effective_service_bps,"
        << "theory_error_bps,theory_error_pct,max_queue_bytes,"
        << "max_aggregate_inflight_bytes,max_concurrent_up\n";
    stage_metrics_
        << "algorithm,mode,seed,run_id,stage_index,stage_label,active_flows,"
        << "stage_start_s,stage_end_s,measurement_start_s,"
        << "measurement_end_s,duration_s,sample_count,"
        << "aggregate_goodput_bps,utilization_pct,jain_fairness,"
        << "mean_flow_goodput_bps,min_flow_goodput_bps,max_flow_goodput_bps,"
        << "mean_excess_inflight_bytes,mean_excess_inflight_bdp,"
        << "p95_excess_inflight_bytes,p95_excess_inflight_bdp,"
        << "max_excess_inflight_bdp,mean_aggregate_inflight_bytes,"
        << "p95_aggregate_inflight_bytes,mean_queue_bytes,"
        << "mean_queue_delay_ms,p50_queue_delay_ms,p95_queue_delay_ms,p99_queue_delay_ms,"
        << "max_queue_delay_ms,queue_drop_packets,queue_drop_bytes,"
        << "mean_sum_pacing_bps,mean_bandwidth_estimate_bps,probe_rtt_seen,"
        << "ideal_up_events,theory_applicable_up_events,"
        << "mean_theory_error_pct,max_abs_theory_error_pct\n";
    flow_metrics_
        << "algorithm,mode,seed,run_id,stage_index,stage_label,active_flows,flow_id,"
        << "received_bytes,goodput_bps,goodput_share_pct\n";
    minute_metrics_
        << "algorithm,mode,seed,run_id,minute_index,window_start_s,"
        << "window_end_s,sample_count,mean_active_flows,"
        << "mean_aggregate_inflight_bytes,mean_excess_inflight_bytes,"
        << "mean_excess_inflight_bdp,mean_queue_bytes,mean_queue_delay_ms,"
        << "mean_sum_pacing_bps,mean_bandwidth_estimate_bps,"
        << "aggregate_goodput_bps,mean_flow_goodput_bps,jain_fairness,"
        << "throughput_recorded\n";
    minute_flow_metrics_
        << "algorithm,mode,seed,run_id,minute_index,window_start_s,"
        << "window_end_s,flow_id,active_in_window,active_duration_s,"
        << "received_bytes,goodput_bps\n";
    run_summary_
        << "algorithm,mode,seed,run_id,simulation_time_s,stages,"
        << "expected_ideal_up_events,observed_ideal_up_events,"
        << "max_concurrent_up,probe_rtt_seen,ideal_sequence_validation,"
        << "validation_pass\n";
    if (diagnostic_trace_) {
      diagnostic_queue_trace_
          << "time_s,stage_index,stage_label,active_flows,queue_bytes,"
          << "queue_delay_ms,aggregate_inflight_bytes,excess_inflight_bytes,"
          << "sum_pacing_bps,sum_max_bw_bps,queue_drop_packets,"
          << "queue_drop_bytes\n";
      diagnostic_flow_trace_
          << "time_s,stage_index,stage_label,active_flows,flow_id,bbr_state,"
          << "probe_phase,pacing_gain,pacing_rate_bps,max_bw_bps,"
          << "delivery_rate_bps,cwnd_bytes,inflight_bytes,srtt_us,min_rtt_us,"
          << "delivered_bytes,sent_bytes,acked_bytes,lost_bytes\n";
      diagnostic_fbbr_trace_
          << "callback_time_s,flow_id,window_start_s,window_end_s,p_underload,"
          << "p_full_load,p_overload,confidence,label,low_confidence,"
          << "diagnostics\n";
    }
  }

  void OnQueueDrop(Ptr<const Packet> packet)
  {
    ++queue_drop_packets_;
    if (packet != nullptr) {
      queue_drop_bytes_ += packet->GetSize();
    }
  }

  void StartStage(uint32_t stage_index)
  {
    if (stage_index >= stage_data_.size()) {
      return;
    }
    StageAccumulator& stage = stage_data_[stage_index];
    stage.active = true;
    stage.measurement_start_s = Simulator::Now().GetSeconds();
    stage.queue_drop_packets_start = queue_drop_packets_;
    stage.queue_drop_bytes_start = queue_drop_bytes_;
    stage.received_bytes_start.resize(stage.config.active_flows, 0);
    for (uint32_t i = 0; i < stage.config.active_flows && i < flows_.size(); ++i) {
      stage.received_bytes_start[i] = flows_[i].receiver->GetReceivedBytes();
    }
    active_stage_index_ = static_cast<int32_t>(stage_index);
  }

  void FinishStage(uint32_t stage_index)
  {
    if (stage_index >= stage_data_.size()) {
      return;
    }
    StageAccumulator& stage = stage_data_[stage_index];
    stage.measurement_end_s = Simulator::Now().GetSeconds();
    stage.queue_drop_packets_end = queue_drop_packets_;
    stage.queue_drop_bytes_end = queue_drop_bytes_;
    stage.received_bytes_end.resize(stage.config.active_flows, 0);
    for (uint32_t i = 0; i < stage.config.active_flows && i < flows_.size(); ++i) {
      stage.received_bytes_end[i] = flows_[i].receiver->GetReceivedBytes();
    }
    stage.active = false;
    if (active_stage_index_ == static_cast<int32_t>(stage_index)) {
      active_stage_index_ = -1;
    }
  }

  void ConfigureIdealStage(uint32_t stage_index)
  {
    if (!ideal_ || stage_index >= stages_config_.size()) {
      return;
    }
    const StageConfig& stage = stages_config_[stage_index];
    const double target_start_s = stage.start_s + ideal_settle_s_;
    for (uint32_t i = 0; i < stage.active_flows && i < flows_.size(); ++i) {
      flows_[i].sender->SetBbr2StrictProbeUp(
          i + 1, stage.active_flows, target_start_s + i * strict_gap_s_,
          strict_min_up_s_, strict_max_up_s_);
    }
  }

  void OnProbePhase(uint32_t flow_index,
                    double event_time_s,
                    const std::string& phase)
  {
    Simulator::ScheduleNow(&Experiment::HandleProbePhase, this, flow_index,
                           event_time_s, phase);
  }

  void Sample()
  {
    const double now_s = Simulator::Now().GetSeconds();
    if (active_stage_index_ >= 0 &&
        static_cast<size_t>(active_stage_index_) < stage_data_.size()) {
      StageAccumulator& stage =
          stage_data_[static_cast<size_t>(active_stage_index_)];
      if (stage.active) {
        const uint64_t queue_bytes = bottleneck_queue_
            ? bottleneck_queue_->GetNBytes()
            : 0;
        uint64_t aggregate_inflight = 0;
        uint64_t sum_pacing = 0;
        uint64_t sum_bandwidth_estimate = 0;
        for (uint32_t i = 0;
             i < stage.config.active_flows && i < flows_.size(); ++i) {
          DqcSender::Bbr2ExperimentSnapshot snapshot;
          if (!flows_[i].sender->GetBbr2ExperimentSnapshot(&snapshot)) {
            continue;
          }
          if (diagnostic_trace_) {
            WriteDiagnosticFlowTrace(now_s, stage.config, i + 1, snapshot);
          }
          aggregate_inflight += snapshot.inflight_bytes;
          sum_pacing += snapshot.pacing_rate_bps;
          sum_bandwidth_estimate += snapshot.max_bw_bps;
          if (snapshot.bbr_state == 6) {
            stage.probe_rtt_seen = true;
            probe_rtt_seen_ = true;
          }
        }
        const uint64_t excess_inflight = aggregate_inflight > bdp_bytes_
            ? aggregate_inflight - bdp_bytes_
            : 0;
        const double queue_delay_ms = static_cast<double>(queue_bytes) * 8.0 /
            static_cast<double>(bottleneck_bps_) * 1000.0;
        stage.aggregate_inflight_samples.push_back(
            static_cast<double>(aggregate_inflight));
        stage.excess_inflight_samples.push_back(
            static_cast<double>(excess_inflight));
        stage.queue_bytes_samples.push_back(static_cast<double>(queue_bytes));
        stage.queue_delay_samples_ms.push_back(queue_delay_ms);
        stage.sum_pacing_samples.push_back(static_cast<double>(sum_pacing));
        stage.bandwidth_estimate_samples.push_back(
            static_cast<double>(sum_bandwidth_estimate));
        RecordMinuteSample(now_s, stage.config.active_flows,
                           aggregate_inflight, excess_inflight, queue_bytes,
                           queue_delay_ms, sum_pacing, sum_bandwidth_estimate);
        if (diagnostic_trace_) {
          WriteDiagnosticQueueTrace(now_s, stage.config, queue_bytes,
                                    queue_delay_ms, aggregate_inflight,
                                    excess_inflight, sum_pacing,
                                    sum_bandwidth_estimate);
        }
        UpdateActiveEvents(queue_bytes, aggregate_inflight);
      }
    }
    if (now_s + sample_interval_s_ <= simulation_time_s_ + 1e-12) {
      Simulator::Schedule(Seconds(sample_interval_s_), &Experiment::Sample,
                          this);
    }
  }

  bool Finish()
  {
    for (size_t i = 0; i < flows_.size(); ++i) {
      if (flows_[i].active_event >= 0) {
        EndEvent(static_cast<uint32_t>(i), simulation_time_s_);
      }
    }
    for (size_t i = 0; i < stage_data_.size(); ++i) {
      if (stage_data_[i].active) {
        FinishStage(static_cast<uint32_t>(i));
      }
    }

    const bool ideal_sequence_valid = ValidateIdealSequence();
    const bool validation_pass = !ideal_ || ideal_sequence_valid;
    WriteEvents();
    WriteStageMetrics();
    WriteMinuteMetrics();
    WriteMinuteFlowMetrics();
    WriteRunSummary(ideal_sequence_valid, validation_pass);
    WriteMetadata(ideal_sequence_valid, validation_pass);

    std::cout << "test1-dynamic algorithm=" << algorithm_
              << " mode=" << ModeName()
              << " stages=" << stages_config_.size()
              << " idealEvents=" << events_data_.size()
              << " maxConcurrentUp=" << max_concurrent_up_
              << " probeRttSeen=" << (probe_rtt_seen_ ? 1 : 0)
              << " validation=" << (validation_pass ? "PASS" : "FAIL")
              << std::endl;
    return validation_pass;
  }

 private:
  std::string ModeName() const
  {
    return ideal_ ? "ideal" : "original";
  }

  std::string ActiveFlowSequence() const
  {
    std::ostringstream sequence;
    for (size_t i = 0; i < stages_config_.size(); ++i) {
      if (i > 0) {
        sequence << ",";
      }
      sequence << stages_config_[i].active_flows;
    }
    return sequence.str();
  }

  int32_t FindStage(double event_time_s) const
  {
    for (size_t i = 0; i < stages_config_.size(); ++i) {
      const StageConfig& stage = stages_config_[i];
      if (event_time_s >= stage.start_s - 1e-9 &&
          event_time_s <= stage.end_s + 1e-9) {
        return static_cast<int32_t>(i);
      }
    }
    return -1;
  }

  void HandleProbePhase(uint32_t flow_index,
                        double event_time_s,
                        const std::string& phase)
  {
    if (!ideal_ || flow_index >= flows_.size()) {
      return;
    }
    if (phase == "PROBE_UP") {
      if (flows_[flow_index].active_event >= 0) {
        EndEvent(flow_index, event_time_s);
      }
      BeginEvent(flow_index, event_time_s);
      return;
    }
    if (flows_[flow_index].active_event >= 0) {
      EndEvent(flow_index, event_time_s);
    }
  }

  void BeginEvent(uint32_t flow_index, double event_time_s)
  {
    const int32_t stage_index = FindStage(event_time_s);
    if (stage_index < 0 || static_cast<size_t>(stage_index) >= stage_data_.size()) {
      return;
    }
    const StageConfig& stage = stages_config_[static_cast<size_t>(stage_index)];
    std::vector<DqcSender::Bbr2ExperimentSnapshot> snapshots(stage.active_flows);
    uint64_t sum_max_bw = 0;
    uint64_t sum_pacing = 0;
    uint32_t other_up_count = 0;
    bool all_other_cruise = true;
    for (uint32_t i = 0; i < stage.active_flows && i < flows_.size(); ++i) {
      flows_[i].sender->GetBbr2ExperimentSnapshot(&snapshots[i]);
      sum_max_bw += snapshots[i].max_bw_bps;
      sum_pacing += snapshots[i].pacing_rate_bps;
      if (i != flow_index) {
        if (snapshots[i].probe_phase == "PROBE_UP") {
          ++other_up_count;
        }
        if (snapshots[i].probe_phase != "PROBE_CRUISE") {
          all_other_cruise = false;
        }
      }
    }
    if (flow_index >= snapshots.size()) {
      return;
    }

    const DqcSender::Bbr2ExperimentSnapshot& owner = snapshots[flow_index];
    ProbeEvent event;
    event.event_id = static_cast<uint32_t>(events_data_.size() + 1);
    event.stage_index = static_cast<uint32_t>(stage_index);
    event.stage_label = stage.label;
    event.active_flows = stage.active_flows;
    event.flow_id = flows_[flow_index].flow_id;
    event.probe_order = flows_[flow_index].flow_id;
    event.pre_all_other_cruise = all_other_cruise;
    event.pre_other_up_count = other_up_count;
    event.strict_controlled = true;
    event.theory_applicable = other_up_count == 0;
    event.start_time_s = event_time_s;
    event.max_bw_before_bps = owner.max_bw_bps;
    event.max_bw_peak_bps = owner.max_bw_bps;
    event.delivery_rate_peak_bps = owner.delivery_rate_bps;
    event.up_pacing_rate_bps = owner.pacing_rate_bps;
    event.sum_pacing_start_bps = sum_pacing;
    const double owner_max_bw = static_cast<double>(owner.max_bw_bps);
    const double other_max_bw = static_cast<double>(
        sum_max_bw >= owner.max_bw_bps ? sum_max_bw - owner.max_bw_bps : 0);
    const double probe_rate = 1.25 * owner_max_bw;
    const double denominator = probe_rate + other_max_bw;
    if (denominator > 0.0) {
      event.theory_service_bps = static_cast<double>(bottleneck_bps_) *
          probe_rate / denominator;
      event.theory_max_bw_bps = std::max(owner_max_bw,
                                          event.theory_service_bps);
    }
    if (sum_pacing > 0) {
      event.effective_service_bps = static_cast<double>(bottleneck_bps_) *
          static_cast<double>(owner.pacing_rate_bps) /
          static_cast<double>(sum_pacing);
    }
    const uint64_t queue_bytes = bottleneck_queue_
        ? bottleneck_queue_->GetNBytes()
        : 0;
    uint64_t aggregate_inflight = 0;
    for (const DqcSender::Bbr2ExperimentSnapshot& snapshot : snapshots) {
      aggregate_inflight += snapshot.inflight_bytes;
    }
    active_up_[flow_index] = true;
    const uint32_t active_count = ActiveUpCount();
    max_concurrent_up_ = std::max(max_concurrent_up_, active_count);
    event.max_queue_bytes = queue_bytes;
    event.max_aggregate_inflight_bytes = aggregate_inflight;
    event.max_concurrent_up = active_count;
    events_data_.push_back(event);
    flows_[flow_index].active_event =
        static_cast<int32_t>(events_data_.size() - 1);
  }

  void EndEvent(uint32_t flow_index, double event_time_s)
  {
    const int32_t event_index = flows_[flow_index].active_event;
    if (event_index < 0 ||
        static_cast<size_t>(event_index) >= events_data_.size()) {
      return;
    }
    DqcSender::Bbr2ExperimentSnapshot snapshot;
    flows_[flow_index].sender->GetBbr2ExperimentSnapshot(&snapshot);
    ProbeEvent& event = events_data_[static_cast<size_t>(event_index)];
    event.end_time_s = event_time_s;
    event.duration_s = std::max(0.0, event.end_time_s - event.start_time_s);
    event.max_bw_after_bps = snapshot.max_bw_bps;
    event.max_bw_peak_bps = std::max(event.max_bw_peak_bps,
                                     snapshot.max_bw_bps);
    event.delivery_rate_peak_bps = std::max(event.delivery_rate_peak_bps,
                                            snapshot.delivery_rate_bps);
    if (event.theory_max_bw_bps > 0.0) {
      event.theory_error_bps =
          static_cast<double>(event.max_bw_after_bps) -
          event.theory_max_bw_bps;
      event.theory_error_pct = 100.0 * event.theory_error_bps /
          event.theory_max_bw_bps;
    }
    active_up_[flow_index] = false;
    flows_[flow_index].active_event = -1;
  }

  void UpdateActiveEvents(uint64_t queue_bytes, uint64_t aggregate_inflight)
  {
    for (size_t i = 0; i < flows_.size(); ++i) {
      const int32_t event_index = flows_[i].active_event;
      if (event_index < 0 ||
          static_cast<size_t>(event_index) >= events_data_.size()) {
        continue;
      }
      ProbeEvent& event = events_data_[static_cast<size_t>(event_index)];
      event.max_queue_bytes = std::max(event.max_queue_bytes, queue_bytes);
      event.max_aggregate_inflight_bytes = std::max(
          event.max_aggregate_inflight_bytes, aggregate_inflight);
      event.max_concurrent_up = std::max(event.max_concurrent_up,
                                         ActiveUpCount());
    }
  }

  uint32_t ActiveUpCount() const
  {
    return static_cast<uint32_t>(std::count(active_up_.begin(),
                                            active_up_.end(), true));
  }

  void WriteDiagnosticFlowTrace(
      double now_s,
      const StageConfig& stage,
      uint32_t flow_id,
      const DqcSender::Bbr2ExperimentSnapshot& snapshot)
  {
    diagnostic_flow_trace_ << std::fixed << std::setprecision(6) << now_s
                           << "," << stage.index << "," << stage.label
                           << "," << stage.active_flows << "," << flow_id
                           << "," << snapshot.bbr_state << ","
                           << CsvEscape(snapshot.probe_phase) << ","
                           << snapshot.pacing_gain << ","
                           << snapshot.pacing_rate_bps << ","
                           << snapshot.max_bw_bps << ","
                           << snapshot.delivery_rate_bps << ","
                           << snapshot.cwnd_bytes << ","
                           << snapshot.inflight_bytes << ","
                           << snapshot.srtt_us << ","
                           << snapshot.min_rtt_us << ","
                           << snapshot.delivered_bytes << ","
                           << snapshot.sent_bytes << ","
                           << snapshot.acked_bytes << ","
                           << snapshot.lost_bytes << "\n";
  }

  void WriteDiagnosticQueueTrace(double now_s,
                                 const StageConfig& stage,
                                 uint64_t queue_bytes,
                                 double queue_delay_ms,
                                 uint64_t aggregate_inflight,
                                 uint64_t excess_inflight,
                                 uint64_t sum_pacing,
                                 uint64_t sum_bandwidth_estimate)
  {
    diagnostic_queue_trace_ << std::fixed << std::setprecision(6) << now_s
                            << "," << stage.index << "," << stage.label
                            << "," << stage.active_flows << ","
                            << queue_bytes << "," << queue_delay_ms << ","
                            << aggregate_inflight << "," << excess_inflight
                            << "," << sum_pacing << ","
                            << sum_bandwidth_estimate << ","
                            << queue_drop_packets_ << ","
                            << queue_drop_bytes_ << "\n";
  }

  void OnFbbrLoadTrace(double window_start_s,
                       double window_end_s,
                       double p_underload,
                       double p_full_load,
                       double p_overload,
                       double confidence,
                       std::string label,
                       bool low_confidence,
                       std::string diagnostics)
  {
    if (!diagnostic_trace_ || !diagnostic_fbbr_trace_.is_open()) {
      return;
    }
    uint32_t flow_id = 0;
    if (label == "FREQ_GATE_TRACE" || label == "SERVICE_FAIRNESS") {
      std::istringstream fields(diagnostics);
      double trace_time_s = 0.0;
      char separator = '\0';
      if (fields >> trace_time_s >> separator >> flow_id && separator != ',') {
        flow_id = 0;
      }
    }
    const double now_s = Simulator::Now().GetSeconds();
    diagnostic_fbbr_trace_ << std::fixed << std::setprecision(6) << now_s
                           << "," << flow_id << "," << window_start_s
                           << "," << window_end_s << "," << p_underload
                           << "," << p_full_load << "," << p_overload
                           << "," << confidence << "," << CsvEscape(label)
                           << "," << (low_confidence ? 1 : 0) << ","
                           << CsvEscape(diagnostics) << "\n";
  }

  void RecordMinuteSample(double now_s,
                          uint32_t active_flows,
                          uint64_t aggregate_inflight,
                          uint64_t excess_inflight,
                          uint64_t queue_bytes,
                          double queue_delay_ms,
                          uint64_t sum_pacing,
                          uint64_t sum_bandwidth_estimate)
  {
    if (minute_data_.empty()) {
      return;
    }
    size_t minute_index = static_cast<size_t>(
        std::floor(now_s / kMinuteWindowS));
    minute_index = std::min(minute_index, minute_data_.size() - 1);
    MinuteAccumulator& minute = minute_data_[minute_index];
    ++minute.sample_count;
    minute.sum_active_flows += static_cast<double>(active_flows);
    minute.sum_aggregate_inflight_bytes +=
        static_cast<double>(aggregate_inflight);
    minute.sum_excess_inflight_bytes += static_cast<double>(excess_inflight);
    minute.sum_queue_bytes += static_cast<double>(queue_bytes);
    minute.sum_queue_delay_ms += queue_delay_ms;
    minute.sum_pacing_bps += static_cast<double>(sum_pacing);
    minute.sum_bandwidth_estimate_bps +=
        static_cast<double>(sum_bandwidth_estimate);
  }

  void RecordMinuteThroughput(size_t minute_index)
  {
    if (minute_index >= minute_data_.size()) {
      return;
    }
    MinuteAccumulator& minute = minute_data_[minute_index];
    const double window_start_s =
        static_cast<double>(minute_index) * kMinuteWindowS;
    const double window_end_s = std::min(
        simulation_time_s_, window_start_s + kMinuteWindowS);
    const double window_duration_s = std::max(1e-9, window_end_s - window_start_s);
    minute.flow_active_duration_s.assign(flows_.size(), 0.0);
    minute.flow_received_bytes.assign(flows_.size(), 0);
    minute.flow_goodput_bps.assign(flows_.size(), 0.0);

    uint64_t aggregate_received_bytes = 0;
    std::vector<double> active_flow_goodputs;
    for (size_t i = 0; i < flows_.size(); ++i) {
      const uint64_t current_received_bytes =
          flows_[i].receiver->GetReceivedBytes();
      const uint64_t previous_received_bytes =
          i < previous_minute_received_bytes_.size()
              ? previous_minute_received_bytes_[i]
              : 0;
      const uint64_t received_bytes = current_received_bytes >= previous_received_bytes
          ? current_received_bytes - previous_received_bytes
          : 0;
      if (i < previous_minute_received_bytes_.size()) {
        previous_minute_received_bytes_[i] = current_received_bytes;
      }
      const double active_start_s = std::max(window_start_s, flows_[i].start_s);
      const double active_end_s = std::min(window_end_s, flows_[i].stop_s);
      const double active_duration_s =
          std::max(0.0, active_end_s - active_start_s);
      const double flow_goodput_bps = active_duration_s > 0.0
          ? static_cast<double>(received_bytes) * 8.0 / active_duration_s
          : 0.0;
      minute.flow_active_duration_s[i] = active_duration_s;
      minute.flow_received_bytes[i] = received_bytes;
      minute.flow_goodput_bps[i] = flow_goodput_bps;
      aggregate_received_bytes += received_bytes;
      if (active_duration_s > 0.0) {
        active_flow_goodputs.push_back(flow_goodput_bps);
      }
    }
    minute.aggregate_goodput_bps =
        static_cast<double>(aggregate_received_bytes) * 8.0 / window_duration_s;
    minute.mean_flow_goodput_bps = Mean(active_flow_goodputs);
    minute.jain_fairness = JainFairness(active_flow_goodputs);
    minute.throughput_recorded = true;
  }

  bool ValidateIdealSequence() const
  {
    if (!ideal_) {
      return true;
    }
    bool valid = true;
    for (const StageConfig& stage : stages_config_) {
      std::vector<const ProbeEvent*> stage_events;
      for (const ProbeEvent& event : events_data_) {
        if (event.stage_index == stage.index) {
          stage_events.push_back(&event);
        }
      }
      if (stage_events.size() != stage.active_flows) {
        valid = false;
        continue;
      }
      std::sort(stage_events.begin(), stage_events.end(),
                [](const ProbeEvent* lhs, const ProbeEvent* rhs) {
                  return lhs->start_time_s < rhs->start_time_s;
                });
      for (size_t i = 0; i < stage_events.size(); ++i) {
        const ProbeEvent& event = *stage_events[i];
        valid = valid && event.flow_id == i + 1 &&
            event.probe_order == i + 1 && event.pre_other_up_count == 0 &&
            event.max_concurrent_up == 1 &&
            event.end_time_s >= event.start_time_s;
        if (i > 0) {
          valid = valid && event.start_time_s + 1e-9 >=
              stage_events[i - 1]->end_time_s;
        }
      }
    }
    return valid;
  }

  void WriteEvents()
  {
    for (const ProbeEvent& event : events_data_) {
      events_ << algorithm_ << "," << ModeName() << "," << seed_ << ","
              << run_id_ << ","
              << event.stage_index << "," << event.stage_label << ","
              << event.active_flows << "," << event.event_id << ","
              << event.flow_id << "," << event.probe_order << ","
              << (event.theory_applicable ? 1 : 0) << ","
              << (event.strict_controlled ? 1 : 0) << ","
              << (event.pre_all_other_cruise ? 1 : 0) << ","
              << event.pre_other_up_count << "," << std::fixed
              << std::setprecision(9) << event.start_time_s << ","
              << event.end_time_s << "," << event.duration_s << ","
              << event.max_bw_before_bps << "," << event.max_bw_peak_bps
              << "," << event.max_bw_after_bps << ","
              << event.delivery_rate_peak_bps << ","
              << event.up_pacing_rate_bps << ","
              << event.sum_pacing_start_bps << "," << std::setprecision(6)
              << event.theory_service_bps << "," << event.theory_max_bw_bps
              << "," << event.effective_service_bps << ","
              << event.theory_error_bps << "," << event.theory_error_pct
              << "," << event.max_queue_bytes << ","
              << event.max_aggregate_inflight_bytes << ","
              << event.max_concurrent_up << "\n";
    }
  }

  void WriteStageMetrics()
  {
    const double bdp = static_cast<double>(std::max<uint64_t>(1, bdp_bytes_));
    for (const StageAccumulator& stage : stage_data_) {
      const double duration_s = std::max(
          1e-9, stage.measurement_end_s - stage.measurement_start_s);
      std::vector<double> flow_goodputs;
      std::vector<uint64_t> received_bytes(stage.config.active_flows, 0);
      double aggregate_goodput_bps = 0.0;
      for (uint32_t i = 0; i < stage.config.active_flows; ++i) {
        const uint64_t begin = i < stage.received_bytes_start.size()
            ? stage.received_bytes_start[i]
            : 0;
        const uint64_t end = i < stage.received_bytes_end.size()
            ? stage.received_bytes_end[i]
            : begin;
        received_bytes[i] = end >= begin ? end - begin : 0;
        const double goodput_bps = static_cast<double>(received_bytes[i]) * 8.0 /
            duration_s;
        flow_goodputs.push_back(goodput_bps);
        aggregate_goodput_bps += goodput_bps;
      }
      const double max_flow_goodput = Maximum(flow_goodputs);
      const double min_flow_goodput = flow_goodputs.empty() ? 0.0
          : *std::min_element(flow_goodputs.begin(), flow_goodputs.end());
      const uint64_t drop_packets = stage.queue_drop_packets_end >=
              stage.queue_drop_packets_start
          ? stage.queue_drop_packets_end - stage.queue_drop_packets_start
          : 0;
      const uint64_t drop_bytes = stage.queue_drop_bytes_end >=
              stage.queue_drop_bytes_start
          ? stage.queue_drop_bytes_end - stage.queue_drop_bytes_start
          : 0;
      std::vector<double> theory_errors;
      uint32_t ideal_events = 0;
      uint32_t theory_events = 0;
      for (const ProbeEvent& event : events_data_) {
        if (event.stage_index != stage.config.index) {
          continue;
        }
        ++ideal_events;
        if (event.theory_applicable) {
          ++theory_events;
          theory_errors.push_back(event.theory_error_pct);
        }
      }
      double max_abs_theory_error = 0.0;
      for (double error : theory_errors) {
        max_abs_theory_error = std::max(max_abs_theory_error, std::abs(error));
      }

      stage_metrics_ << algorithm_ << "," << ModeName() << "," << seed_ << ","
                     << run_id_ << ","
                     << stage.config.index << "," << stage.config.label << ","
                     << stage.config.active_flows << "," << std::fixed
                     << std::setprecision(6) << stage.config.start_s << ","
                     << stage.config.end_s << "," << stage.measurement_start_s
                     << "," << stage.measurement_end_s << "," << duration_s
                     << "," << stage.queue_delay_samples_ms.size() << ","
                     << aggregate_goodput_bps << ","
                     << 100.0 * aggregate_goodput_bps /
                            static_cast<double>(bottleneck_bps_) << ","
                     << JainFairness(flow_goodputs) << ","
                     << Mean(flow_goodputs) << "," << min_flow_goodput << ","
                     << max_flow_goodput << ","
                     << Mean(stage.excess_inflight_samples) << ","
                     << Mean(stage.excess_inflight_samples) / bdp << ","
                     << Percentile(stage.excess_inflight_samples, 0.95) << ","
                     << Percentile(stage.excess_inflight_samples, 0.95) / bdp
                     << "," << Maximum(stage.excess_inflight_samples) / bdp
                     << "," << Mean(stage.aggregate_inflight_samples) << ","
                     << Percentile(stage.aggregate_inflight_samples, 0.95)
                     << "," << Mean(stage.queue_bytes_samples) << ","
                     << Mean(stage.queue_delay_samples_ms) << ","
                     << Percentile(stage.queue_delay_samples_ms, 0.50) << ","
                     << Percentile(stage.queue_delay_samples_ms, 0.95) << ","
                     << Percentile(stage.queue_delay_samples_ms, 0.99) << ","
                     << Maximum(stage.queue_delay_samples_ms) << ","
                     << drop_packets << "," << drop_bytes << ","
                     << Mean(stage.sum_pacing_samples) << ","
                     << Mean(stage.bandwidth_estimate_samples) << ","
                     << (stage.probe_rtt_seen ? 1 : 0) << ","
                     << ideal_events << "," << theory_events << ","
                     << Mean(theory_errors) << "," << max_abs_theory_error
                     << "\n";

      for (uint32_t i = 0; i < stage.config.active_flows; ++i) {
        const double goodput_bps = flow_goodputs[i];
        const double share_pct = aggregate_goodput_bps > 0.0
            ? 100.0 * goodput_bps / aggregate_goodput_bps
            : 0.0;
        flow_metrics_ << algorithm_ << "," << ModeName() << "," << seed_ << ","
                      << run_id_ << ","
                      << stage.config.index << "," << stage.config.label
                      << "," << stage.config.active_flows << "," << i + 1
                      << "," << received_bytes[i] << "," << goodput_bps
                      << "," << share_pct << "\n";
      }
    }
  }

  void WriteMinuteMetrics()
  {
    const double bdp = static_cast<double>(std::max<uint64_t>(1, bdp_bytes_));
    for (size_t i = 0; i < minute_data_.size(); ++i) {
      const MinuteAccumulator& minute = minute_data_[i];
      const double divisor = minute.sample_count == 0
          ? 1.0
          : static_cast<double>(minute.sample_count);
      const double mean_active_flows = minute.sum_active_flows / divisor;
      const double mean_aggregate_inflight =
          minute.sum_aggregate_inflight_bytes / divisor;
      const double mean_excess_inflight =
          minute.sum_excess_inflight_bytes / divisor;
      const double mean_queue_bytes = minute.sum_queue_bytes / divisor;
      const double mean_queue_delay = minute.sum_queue_delay_ms / divisor;
      const double mean_pacing = minute.sum_pacing_bps / divisor;
      const double mean_bandwidth = minute.sum_bandwidth_estimate_bps / divisor;
      const double window_start_s = static_cast<double>(i) * kMinuteWindowS;
      const double window_end_s = std::min(
          simulation_time_s_, window_start_s + kMinuteWindowS);

      minute_metrics_ << algorithm_ << "," << ModeName() << "," << seed_
                      << "," << run_id_ << "," << i << "," << std::fixed
                      << std::setprecision(6) << window_start_s << ","
                      << window_end_s << "," << minute.sample_count << ","
                      << mean_active_flows << "," << mean_aggregate_inflight
                      << "," << mean_excess_inflight << ","
                      << mean_excess_inflight / bdp << "," << mean_queue_bytes
                      << "," << mean_queue_delay << "," << mean_pacing << ","
                      << mean_bandwidth << "," << minute.aggregate_goodput_bps
                      << "," << minute.mean_flow_goodput_bps << ","
                      << minute.jain_fairness << ","
                      << (minute.throughput_recorded ? 1 : 0) << "\n";
    }
  }

  void WriteMinuteFlowMetrics()
  {
    for (size_t minute_index = 0; minute_index < minute_data_.size();
         ++minute_index) {
      const MinuteAccumulator& minute = minute_data_[minute_index];
      const double window_start_s =
          static_cast<double>(minute_index) * kMinuteWindowS;
      const double window_end_s = std::min(
          simulation_time_s_, window_start_s + kMinuteWindowS);
      for (size_t flow_index = 0; flow_index < flows_.size(); ++flow_index) {
        const double active_duration_s =
            flow_index < minute.flow_active_duration_s.size()
                ? minute.flow_active_duration_s[flow_index]
                : 0.0;
        const uint64_t received_bytes =
            flow_index < minute.flow_received_bytes.size()
                ? minute.flow_received_bytes[flow_index]
                : 0;
        const double goodput_bps =
            flow_index < minute.flow_goodput_bps.size()
                ? minute.flow_goodput_bps[flow_index]
                : 0.0;
        minute_flow_metrics_ << algorithm_ << "," << ModeName() << ","
                             << seed_ << "," << run_id_ << ","
                             << minute_index << "," << std::fixed
                             << std::setprecision(6) << window_start_s << ","
                             << window_end_s << "," << flows_[flow_index].flow_id
                             << "," << (active_duration_s > 0.0 ? 1 : 0)
                             << "," << active_duration_s << ","
                             << received_bytes << "," << goodput_bps << "\n";
      }
    }
  }

  void WriteRunSummary(bool ideal_sequence_valid, bool validation_pass)
  {
    const uint32_t expected_events = ideal_
        ? std::accumulate(stages_config_.begin(), stages_config_.end(), 0u,
                          [](uint32_t total, const StageConfig& stage) {
                            return total + stage.active_flows;
                          })
        : 0;
    run_summary_ << algorithm_ << "," << ModeName() << "," << seed_ << ","
                 << run_id_ << "," << simulation_time_s_ << ","
                 << stages_config_.size() << "," << expected_events << ","
                 << events_data_.size() << "," << max_concurrent_up_ << ","
                 << (probe_rtt_seen_ ? 1 : 0) << ","
                 << (ideal_sequence_valid ? 1 : 0) << ","
                 << (validation_pass ? 1 : 0) << "\n";
  }

  void WriteMetadata(bool ideal_sequence_valid, bool validation_pass)
  {
    metadata_ << "{\n"
              << "  \"algorithm\": \"" << algorithm_ << "\",\n"
              << "  \"mode\": \"" << ModeName() << "\",\n"
              << "  \"seed\": " << seed_ << ",\n"
              << "  \"run_id\": " << run_id_ << ",\n"
              << "  \"simulation_time_s\": " << simulation_time_s_ << ",\n"
              << "  \"capacity_bps\": " << bottleneck_bps_ << ",\n"
              << "  \"base_bdp_bytes\": " << bdp_bytes_ << ",\n"
              << "  \"flow_pattern\": \"" << flow_pattern_ << "\",\n"
              << "  \"dynamic_flow_sequence\": \""
              << ActiveFlowSequence() << "\",\n"
              << "  \"minute_window_s\": " << kMinuteWindowS << ",\n"
              << "  \"diagnostic_trace\": "
              << (diagnostic_trace_ ? "true" : "false") << ",\n"
              << "  \"probe_rtt_seen\": "
              << (probe_rtt_seen_ ? "true" : "false") << ",\n"
              << "  \"ideal_sequence_validation\": "
              << (ideal_sequence_valid ? "true" : "false") << ",\n"
              << "  \"validation_pass\": "
              << (validation_pass ? "true" : "false") << "\n"
              << "}\n";
  }

  std::string algorithm_;
  bool ideal_;
  uint32_t seed_;
  uint32_t run_id_;
  uint64_t bottleneck_bps_;
  uint64_t bdp_bytes_;
  double simulation_time_s_;
  double sample_interval_s_;
  double stage_edge_guard_s_;
  double ideal_settle_s_;
  double strict_gap_s_;
  double strict_min_up_s_;
  double strict_max_up_s_;
  Ptr<Queue<Packet> > bottleneck_queue_;
  std::vector<StageConfig> stages_config_;
  std::string output_prefix_;
  std::string flow_pattern_;
  bool diagnostic_trace_;
  std::vector<FlowRuntime> flows_;
  std::vector<StageAccumulator> stage_data_;
  std::vector<MinuteAccumulator> minute_data_;
  std::vector<uint64_t> previous_minute_received_bytes_;
  std::vector<ProbeEvent> events_data_;
  std::vector<bool> active_up_;
  int32_t active_stage_index_ = -1;
  uint64_t queue_drop_packets_ = 0;
  uint64_t queue_drop_bytes_ = 0;
  uint32_t max_concurrent_up_ = 0;
  bool probe_rtt_seen_ = false;
  std::ofstream events_;
  std::ofstream stage_metrics_;
  std::ofstream flow_metrics_;
  std::ofstream minute_metrics_;
  std::ofstream minute_flow_metrics_;
  std::ofstream run_summary_;
  std::ofstream metadata_;
  std::ofstream diagnostic_queue_trace_;
  std::ofstream diagnostic_flow_trace_;
  std::ofstream diagnostic_fbbr_trace_;
};

FlowRuntime
InstallFlow(dqc::CongestionControlType cc_type,
            const std::string& algorithm,
            const dqc::FBBRConfig& fbbr_config,
            Ptr<Node> sender,
            Ptr<Node> receiver,
            uint32_t flow_index,
            double app_start_s,
            double app_stop_s,
            uint32_t send_buffer_bytes)
{
  const uint16_t port = static_cast<uint16_t>(9000 + flow_index);
  Ptr<DqcReceiver> recv_app = CreateObject<DqcReceiver>(1000);
  receiver->AddApplication(recv_app);
  recv_app->Bind(port);
  recv_app->SetStartTime(Seconds(app_start_s));
  recv_app->SetStopTime(Seconds(app_stop_s));

  Ptr<DqcSender> send_app = CreateObject<DqcSender>(cc_type, false, false);
  sender->AddApplication(send_app);
  send_app->SetSenderId(flow_index + 1);
  send_app->Bind(static_cast<uint16_t>(10000 + flow_index));
  send_app->ConfigurePeer(recv_app->GetLocalAddress().GetIpv4(), port);
  send_app->SetStreamSendBufferBytes(send_buffer_bytes);
  if (IsFBBRAlgorithm(algorithm)) {
    send_app->ConfigureFBBR(fbbr_config, flow_index + 1);
  }
  send_app->SetStartTime(Seconds(app_start_s));
  send_app->SetStopTime(Seconds(app_stop_s));

  FlowRuntime flow;
  flow.sender = send_app;
  flow.receiver = recv_app;
  flow.flow_id = flow_index + 1;
  flow.start_s = app_start_s;
  flow.stop_s = app_stop_s;
  return flow;
}

}  // namespace

int
main(int argc, char* argv[])
{
  std::string algorithm = "BBRv2";
  std::string output_dir = "results/test1/raw";
  std::string flow_pattern = "dynamic";
  uint32_t seed = 1;
  uint32_t run_id = 1;
  uint64_t bottleneck_bps = 100000000ULL;
  uint64_t access_bps = 1000000000ULL;
  double base_rtt_s = 0.040;
  double bottleneck_delay_s = 0.010;
  double queue_bdp = 40.0;
  double simulation_time_s = 1800.0;
  double sample_interval_s = 0.100;
  double stage_edge_guard_s = 0.010;
  double ideal_settle_s = 20.0;
  double strict_gap_s = 0.150;
  double strict_min_up_s = 0.040;
  double strict_max_up_s = 0.080;
  bool diagnostic_trace = false;

  CommandLine cmd;
  cmd.AddValue("algorithm",
               "BBR-R, oBBR, BBRv2+, CUBIC, BBRv2-ideal, BBRv2, FBBR, or FBBR-ServiceFair",
               algorithm);
  cmd.AddValue("outputDir", "Relative output directory", output_dir);
  cmd.AddValue("flowPattern",
               "dynamic, 8to16, 4to8to16to8to4, steady1, steady8, or steady16 active-flow pattern",
               flow_pattern);
  cmd.AddValue("seed", "Deterministic DQC and ns-3 random seed", seed);
  cmd.AddValue("runId", "Run identifier persisted in CSV", run_id);
  cmd.AddValue("bottleneckBps", "Bottleneck capacity in bit/s", bottleneck_bps);
  cmd.AddValue("accessBps", "Access-link capacity in bit/s", access_bps);
  cmd.AddValue("baseRtt", "End-to-end propagation RTT in seconds", base_rtt_s);
  cmd.AddValue("bottleneckDelay", "One-way bottleneck delay in seconds",
               bottleneck_delay_s);
  cmd.AddValue("queueBdp", "DropTail bottleneck buffer in BDP multiples",
               queue_bdp);
  cmd.AddValue("simulationTime", "Total dynamic simulation duration in seconds",
               simulation_time_s);
  cmd.AddValue("sampleInterval", "Stage metric sample interval in seconds",
               sample_interval_s);
  cmd.AddValue("stageEdgeGuard", "Excluded time at each stage edge in seconds",
               stage_edge_guard_s);
  cmd.AddValue("idealSettle", "BBRv2-ideal settle time per stage in seconds",
               ideal_settle_s);
  cmd.AddValue("strictGap", "Target spacing between ideal UPs in seconds",
               strict_gap_s);
  cmd.AddValue("strictMinUp", "Ideal UP minimum residence in seconds",
               strict_min_up_s);
  cmd.AddValue("strictMaxUp", "Ideal UP hard maximum residence in seconds",
               strict_max_up_s);
  cmd.AddValue("diagnosticTrace",
               "Write 20 ms queue, flow, and FBBR decision traces",
               diagnostic_trace);
  cmd.Parse(argc, argv);

  if (!IsKnownAlgorithm(algorithm) || !IsSupportedFlowPattern(flow_pattern) ||
      bottleneck_bps == 0 ||
      base_rtt_s <= 0.0 || bottleneck_delay_s <= 0.0 || queue_bdp <= 0.0 ||
      simulation_time_s <= 0.0 || sample_interval_s <= 0.0 ||
      stage_edge_guard_s < 0.0 || ideal_settle_s <= 0.0 || strict_gap_s <= 0.0 ||
      strict_min_up_s < 0.0 || strict_max_up_s <= 0.0) {
    std::cerr << "Invalid test1 dynamic arguments" << std::endl;
    return 1;
  }

  const bool ideal = algorithm == "BBRv2-ideal";
  if (diagnostic_trace && !IsFBBRAlgorithm(algorithm)) {
    std::cerr << "diagnosticTrace is currently available only for FBBR variants"
              << std::endl;
    return 1;
  }
  if (ideal && flow_pattern != "dynamic") {
    std::cerr << "BBRv2-ideal currently requires flowPattern=dynamic"
              << std::endl;
    return 1;
  }
  const std::vector<StageConfig> stages =
      BuildStages(simulation_time_s, flow_pattern);
  const double stage_duration_s =
      simulation_time_s / static_cast<double>(stages.size());
  if (2.0 * stage_edge_guard_s >= stage_duration_s ||
      (ideal && ideal_settle_s + (kMaximumFlows - 1) * strict_gap_s +
                    strict_max_up_s >= stage_duration_s)) {
    std::cerr << "A stage is too short for the requested ideal UP sequence"
              << std::endl;
    return 1;
  }

  const double one_way_access_delay_s =
      (base_rtt_s * 0.5 - bottleneck_delay_s) * 0.5;
  if (one_way_access_delay_s <= 0.0) {
    std::cerr << "baseRtt must exceed twice bottleneckDelay" << std::endl;
    return 1;
  }
  const uint64_t bdp_bytes = static_cast<uint64_t>(
      static_cast<long double>(bottleneck_bps) * base_rtt_s / 8.0L);
  const uint32_t queue_bytes = static_cast<uint32_t>(std::max<long double>(
      1.0L, static_cast<long double>(bdp_bytes) * queue_bdp));
  const uint32_t send_buffer_bytes = 16U * 1024U * 1024U;
  const std::string output_prefix = output_dir + "/" + FileToken(algorithm) +
      "_seed" + std::to_string(seed) + "_run" + std::to_string(run_id);

  RngSeedManager::SetSeed(seed);
  RngSeedManager::SetRun(run_id);
  dqc::SendPacketManager::SetDeterministicRandomSeed(seed, run_id);

  NodeContainer senders;
  NodeContainer routers;
  NodeContainer receivers;
  senders.Create(kMaximumFlows);
  routers.Create(2);
  receivers.Create(kMaximumFlows);

  InternetStackHelper internet;
  internet.Install(senders);
  internet.Install(routers);
  internet.Install(receivers);

  PointToPointHelper access;
  access.SetDeviceAttribute("DataRate", DataRateValue(DataRate(access_bps)));
  access.SetChannelAttribute("Delay", TimeValue(Seconds(one_way_access_delay_s)));
  access.SetQueue("ns3::DropTailQueue", "Mode",
                  StringValue("QUEUE_MODE_BYTES"), "MaxBytes",
                  UintegerValue(std::max<uint32_t>(queue_bytes, 1024 * 1024)));

  PointToPointHelper bottleneck;
  bottleneck.SetDeviceAttribute("DataRate",
                                DataRateValue(DataRate(bottleneck_bps)));
  bottleneck.SetChannelAttribute("Delay", TimeValue(Seconds(bottleneck_delay_s)));
  bottleneck.SetQueue("ns3::DropTailQueue", "Mode",
                     StringValue("QUEUE_MODE_BYTES"), "MaxBytes",
                     UintegerValue(queue_bytes));

  Ipv4AddressHelper ipv4;
  TrafficControlHelper traffic_control;
  for (uint32_t i = 0; i < kMaximumFlows; ++i) {
    NodeContainer pair(senders.Get(i), routers.Get(0));
    NetDeviceContainer devices = access.Install(pair);
    std::ostringstream subnet;
    subnet << "10.1." << (i + 1) << ".0";
    ipv4.SetBase(subnet.str().c_str(), "255.255.255.0");
    ipv4.Assign(devices);
    traffic_control.Uninstall(devices);
  }

  NetDeviceContainer bottleneck_devices =
      bottleneck.Install(NodeContainer(routers.Get(0), routers.Get(1)));
  ipv4.SetBase("10.2.0.0", "255.255.255.0");
  ipv4.Assign(bottleneck_devices);
  traffic_control.Uninstall(bottleneck_devices);
  Ptr<PointToPointNetDevice> bottleneck_device =
      DynamicCast<PointToPointNetDevice>(bottleneck_devices.Get(0));
  Ptr<Queue<Packet> > bottleneck_queue = bottleneck_device
      ? bottleneck_device->GetQueue()
      : nullptr;

  for (uint32_t i = 0; i < kMaximumFlows; ++i) {
    NodeContainer pair(routers.Get(1), receivers.Get(i));
    NetDeviceContainer devices = access.Install(pair);
    std::ostringstream subnet;
    subnet << "10.3." << (i + 1) << ".0";
    ipv4.SetBase(subnet.str().c_str(), "255.255.255.0");
    ipv4.Assign(devices);
    traffic_control.Uninstall(devices);
  }
  Ipv4GlobalRoutingHelper::PopulateRoutingTables();

  const dqc::CongestionControlType cc_type = ResolveAlgorithm(algorithm);
  const dqc::FBBRConfig fbbr_config = BuildFBBRConfig(diagnostic_trace);
  std::vector<FlowRuntime> flows;
  flows.reserve(kMaximumFlows);
  for (uint32_t i = 0; i < kMaximumFlows; ++i) {
    const FlowLifetime lifetime =
        LifetimeForFlow(i + 1, stage_duration_s, simulation_time_s,
                        flow_pattern);
    flows.push_back(InstallFlow(cc_type, algorithm, fbbr_config,
                                senders.Get(i), receivers.Get(i), i,
                                lifetime.start_s, lifetime.stop_s,
                                send_buffer_bytes));
  }

  Experiment experiment(algorithm, ideal, seed, run_id, bottleneck_bps,
                        bdp_bytes, simulation_time_s, sample_interval_s,
                        stage_edge_guard_s, ideal_settle_s, strict_gap_s,
                        strict_min_up_s, strict_max_up_s, bottleneck_queue,
                        stages, output_prefix, flow_pattern, diagnostic_trace);
  experiment.SetFlows(flows);
  experiment.OpenOutputs();
  experiment.EnableDiagnosticTracing();
  experiment.StartMinuteThroughputTrace();
  if (bottleneck_queue != nullptr) {
    bottleneck_queue->TraceConnectWithoutContext(
        "Drop", MakeCallback(&Experiment::OnQueueDrop, &experiment));
  }
  if (ideal) {
    for (uint32_t i = 0; i < kMaximumFlows; ++i) {
      flows[i].sender->SetBbr2ExperimentPhaseTrace(
          [&experiment, i](double event_time_s, const std::string& phase) {
            experiment.OnProbePhase(i, event_time_s, phase);
          });
    }
    experiment.ConfigureIdealStage(0);
    for (uint32_t stage_index = 1;
         stage_index < stages.size(); ++stage_index) {
      Simulator::Schedule(Seconds(stages[stage_index].start_s +
                                  stage_edge_guard_s),
                          &Experiment::ConfigureIdealStage, &experiment,
                          stage_index);
    }
  }

  for (uint32_t stage_index = 0;
       stage_index < stages.size(); ++stage_index) {
    const StageConfig& stage = stages[stage_index];
    Simulator::Schedule(Seconds(stage.start_s + stage_edge_guard_s),
                        &Experiment::StartStage, &experiment, stage_index);
    Simulator::Schedule(Seconds(stage.end_s - stage_edge_guard_s),
                        &Experiment::FinishStage, &experiment, stage_index);
  }

  std::cout << "test1-dynamic algorithm=" << algorithm
            << " mode=" << (ideal ? "ideal" : "original")
            << " flowPattern=" << flow_pattern
            << " diagnosticTrace=" << (diagnostic_trace ? 1 : 0)
            << " simulationTime=" << simulation_time_s
            << " capacity=" << bottleneck_bps
            << " queueBytes=" << queue_bytes
            << " output=" << output_prefix << std::endl;
  Simulator::Schedule(Seconds(stage_edge_guard_s + sample_interval_s),
                      &Experiment::Sample, &experiment);
  Simulator::Stop(Seconds(simulation_time_s));
  Simulator::Run();
  const bool validation_pass = experiment.Finish();
  Simulator::Destroy();
  return validation_pass ? 0 : 2;
}
