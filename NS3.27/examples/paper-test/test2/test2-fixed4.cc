/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Test 2: fixed four-flow parameter matrix.
 *
 * One bottleneck scenario is run per invocation.  All four DQC flows remain
 * active for the full simulation, so capacity, RTT, and buffer changes can be
 * compared without a population transition confounding the measurements.
 */

#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/dqc-module.h"
#include "ns3/fbbr_config_loader.h"
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
#include <vector>

using namespace ns3;

namespace {

const uint32_t kFixedFlows = 4;
const double kMinuteWindowS = 60.0;
const char kDefaultFBBRConfig[] =
    "/home/wkd/FreqBBR/NS3.27/examples/CCconfig/fbbr_default.conf";

struct FlowRuntime {
  Ptr<DqcSender> sender;
  Ptr<DqcReceiver> receiver;
  uint32_t flow_id = 0;
  double start_s = 0.0;
  double stop_s = 0.0;
  int32_t active_event = -1;
};

struct MinuteAccumulator {
  uint64_t sample_count = 0;
  double sum_aggregate_inflight_bytes = 0.0;
  double sum_excess_inflight_bytes = 0.0;
  double sum_queue_bytes = 0.0;
  double sum_queue_delay_ms = 0.0;
  double sum_pacing_bps = 0.0;
  double sum_bandwidth_estimate_bps = 0.0;
  bool throughput_recorded = false;
  std::vector<uint64_t> flow_received_bytes;
  std::vector<double> flow_goodput_bps;
  double aggregate_goodput_bps = 0.0;
  double mean_flow_goodput_bps = 0.0;
  double jain_fairness = 0.0;
};

struct ProbeEvent {
  uint32_t event_id = 0;
  uint32_t flow_id = 0;
  uint32_t probe_order = 0;
  uint32_t pre_other_up_count = 0;
  uint32_t max_concurrent_up = 0;
  double start_time_s = -1.0;
  double end_time_s = -1.0;
  double duration_s = 0.0;
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

class FixedFourExperiment {
 public:
  FixedFourExperiment(const std::string& algorithm,
                      uint32_t seed,
                      uint32_t run_id,
                      uint64_t bottleneck_bps,
                      uint64_t bdp_bytes,
                      double simulation_time_s,
                      double sample_interval_s,
                      double measurement_guard_s,
                      double ideal_settle_s,
                      double strict_gap_s,
                      double strict_min_up_s,
                      double strict_max_up_s,
                      Ptr<Queue<Packet> > bottleneck_queue,
                      const std::string& output_prefix,
                      double base_rtt_s,
                      double bottleneck_delay_s,
                      uint64_t access_bps,
                      double queue_bdp,
                      uint32_t queue_bytes)
      : algorithm_(algorithm),
        ideal_(algorithm == "BBRv2-formal"),
        seed_(seed),
        run_id_(run_id),
        bottleneck_bps_(bottleneck_bps),
        bdp_bytes_(bdp_bytes),
        simulation_time_s_(simulation_time_s),
        sample_interval_s_(sample_interval_s),
        measurement_guard_s_(measurement_guard_s),
        ideal_settle_s_(ideal_settle_s),
        strict_gap_s_(strict_gap_s),
        strict_min_up_s_(strict_min_up_s),
        strict_max_up_s_(strict_max_up_s),
        bottleneck_queue_(bottleneck_queue),
        output_prefix_(output_prefix),
        base_rtt_s_(base_rtt_s),
        bottleneck_delay_s_(bottleneck_delay_s),
        access_bps_(access_bps),
        queue_bdp_(queue_bdp),
        queue_bytes_(queue_bytes),
        minute_data_(static_cast<size_t>(
            std::ceil(simulation_time_s / kMinuteWindowS))),
        active_up_(kFixedFlows, false)
  {
  }

  void SetFlows(const std::vector<FlowRuntime>& flows)
  {
    flows_ = flows;
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
    if (!events_.is_open() || !stage_metrics_.is_open() ||
        !flow_metrics_.is_open() || !minute_metrics_.is_open() ||
        !minute_flow_metrics_.is_open() || !run_summary_.is_open() ||
        !metadata_.is_open()) {
      std::cerr << "Failed to open Test 2 output files under "
                << output_prefix_ << std::endl;
      std::exit(1);
    }

    events_ << "algorithm,mode,seed,run_id,event_id,flow_id,probe_order,"
            << "pre_other_up_count,max_concurrent_up,start_time_s,end_time_s,"
            << "duration_s\n";
    stage_metrics_
        << "algorithm,mode,seed,run_id,stage_index,stage_label,active_flows,"
        << "stage_start_s,stage_end_s,measurement_start_s,measurement_end_s,"
        << "duration_s,sample_count,aggregate_goodput_bps,utilization_pct,"
        << "jain_fairness,mean_flow_goodput_bps,min_flow_goodput_bps,"
        << "max_flow_goodput_bps,mean_excess_inflight_bytes,"
        << "mean_excess_inflight_bdp,p95_excess_inflight_bytes,"
        << "p95_excess_inflight_bdp,max_excess_inflight_bdp,"
        << "mean_aggregate_inflight_bytes,p95_aggregate_inflight_bytes,"
        << "mean_queue_bytes,mean_queue_delay_ms,p50_queue_delay_ms,"
        << "p95_queue_delay_ms,p99_queue_delay_ms,max_queue_delay_ms,"
        << "queue_drop_packets,queue_drop_bytes,mean_sum_pacing_bps,"
        << "mean_bandwidth_estimate_bps,probe_rtt_seen\n";
    flow_metrics_ << "algorithm,mode,seed,run_id,stage_index,stage_label,"
                  << "active_flows,flow_id,received_bytes,goodput_bps,"
                  << "goodput_share_pct\n";
    minute_metrics_ << "algorithm,mode,seed,run_id,minute_index,window_start_s,"
                    << "window_end_s,sample_count,mean_active_flows,"
                    << "mean_aggregate_inflight_bytes,mean_excess_inflight_bytes,"
                    << "mean_excess_inflight_bdp,mean_queue_bytes,"
                    << "mean_queue_delay_ms,mean_sum_pacing_bps,"
                    << "mean_bandwidth_estimate_bps,aggregate_goodput_bps,"
                    << "mean_flow_goodput_bps,jain_fairness,throughput_recorded\n";
    minute_flow_metrics_ << "algorithm,mode,seed,run_id,minute_index,"
                         << "window_start_s,window_end_s,flow_id,active_in_window,"
                         << "active_duration_s,received_bytes,goodput_bps\n";
    run_summary_ << "algorithm,mode,seed,run_id,simulation_time_s,stages,"
                 << "active_flows,expected_ideal_up_events,"
                 << "observed_ideal_up_events,max_concurrent_up,probe_rtt_seen,"
                 << "ideal_sequence_validation,validation_pass\n";
  }

  void StartMeasurement()
  {
    measurement_active_ = true;
    measurement_start_s_ = Simulator::Now().GetSeconds();
    queue_drop_packets_start_ = queue_drop_packets_;
    queue_drop_bytes_start_ = queue_drop_bytes_;
    received_bytes_start_.assign(flows_.size(), 0);
    for (size_t i = 0; i < flows_.size(); ++i) {
      received_bytes_start_[i] = flows_[i].receiver->GetReceivedBytes();
    }
  }

  void FinishMeasurement()
  {
    measurement_end_s_ = Simulator::Now().GetSeconds();
    queue_drop_packets_end_ = queue_drop_packets_;
    queue_drop_bytes_end_ = queue_drop_bytes_;
    received_bytes_end_.assign(flows_.size(), 0);
    for (size_t i = 0; i < flows_.size(); ++i) {
      received_bytes_end_[i] = flows_[i].receiver->GetReceivedBytes();
    }
    measurement_active_ = false;
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
                          &FixedFourExperiment::RecordMinuteThroughput,
                          this, i);
    }
  }

  void ConfigureFormalProbeUps()
  {
    if (!ideal_) {
      return;
    }
    for (uint32_t i = 0; i < kFixedFlows && i < flows_.size(); ++i) {
      flows_[i].sender->SetBbr2StrictProbeUp(
          i + 1, kFixedFlows, ideal_settle_s_ + i * strict_gap_s_,
          strict_min_up_s_, strict_max_up_s_);
    }
  }

  void OnProbePhase(uint32_t flow_index,
                    double event_time_s,
                    const std::string& phase)
  {
    Simulator::ScheduleNow(&FixedFourExperiment::HandleProbePhase, this,
                           flow_index, event_time_s, phase);
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
      const uint64_t queue_bytes = bottleneck_queue_
          ? bottleneck_queue_->GetNBytes()
          : 0;
      uint64_t aggregate_inflight = 0;
      uint64_t sum_pacing = 0;
      uint64_t sum_bandwidth_estimate = 0;
      for (size_t i = 0; i < flows_.size(); ++i) {
        DqcSender::Bbr2ExperimentSnapshot snapshot;
        if (!flows_[i].sender->GetBbr2ExperimentSnapshot(&snapshot)) {
          continue;
        }
        aggregate_inflight += snapshot.inflight_bytes;
        sum_pacing += snapshot.pacing_rate_bps;
        sum_bandwidth_estimate += snapshot.max_bw_bps;
        if (snapshot.bbr_state == 6) {
          probe_rtt_seen_ = true;
        }
      }
      const uint64_t excess_inflight = aggregate_inflight > bdp_bytes_
          ? aggregate_inflight - bdp_bytes_
          : 0;
      const double queue_delay_ms = static_cast<double>(queue_bytes) * 8.0 /
          static_cast<double>(bottleneck_bps_) * 1000.0;
      aggregate_inflight_samples_.push_back(
          static_cast<double>(aggregate_inflight));
      excess_inflight_samples_.push_back(
          static_cast<double>(excess_inflight));
      queue_bytes_samples_.push_back(static_cast<double>(queue_bytes));
      queue_delay_samples_ms_.push_back(queue_delay_ms);
      sum_pacing_samples_.push_back(static_cast<double>(sum_pacing));
      bandwidth_estimate_samples_.push_back(
          static_cast<double>(sum_bandwidth_estimate));
      RecordMinuteSample(now_s, aggregate_inflight, excess_inflight,
                         queue_bytes, queue_delay_ms, sum_pacing,
                         sum_bandwidth_estimate);
      UpdateActiveEvents();
    }
    if (now_s + sample_interval_s_ <= simulation_time_s_ + 1e-12) {
      Simulator::Schedule(Seconds(sample_interval_s_),
                          &FixedFourExperiment::Sample, this);
    }
  }

  bool Finish()
  {
    for (size_t i = 0; i < flows_.size(); ++i) {
      if (flows_[i].active_event >= 0) {
        EndEvent(static_cast<uint32_t>(i), simulation_time_s_);
      }
    }
    if (measurement_active_) {
      FinishMeasurement();
    }
    const bool ideal_sequence_valid = ValidateIdealSequence();
    const bool validation_pass = !ideal_ || ideal_sequence_valid;
    WriteEvents();
    WriteStageMetrics();
    WriteMinuteMetrics();
    WriteMinuteFlowMetrics();
    WriteRunSummary(ideal_sequence_valid, validation_pass);
    WriteMetadata(ideal_sequence_valid, validation_pass);
    std::cout << "test2-fixed4 algorithm=" << algorithm_
              << " mode=" << ModeName()
              << " idealEvents=" << events_data_.size()
              << " maxConcurrentUp=" << max_concurrent_up_
              << " validation=" << (validation_pass ? "PASS" : "FAIL")
              << std::endl;
    return validation_pass;
  }

 private:
  std::string ModeName() const
  {
    return ideal_ ? "ideal" : "original";
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
    uint32_t other_up_count = 0;
    for (uint32_t i = 0; i < kFixedFlows && i < flows_.size(); ++i) {
      if (i != flow_index && active_up_[i]) {
        ++other_up_count;
      }
    }
    ProbeEvent event;
    event.event_id = static_cast<uint32_t>(events_data_.size() + 1);
    event.flow_id = flows_[flow_index].flow_id;
    event.probe_order = flows_[flow_index].flow_id;
    event.pre_other_up_count = other_up_count;
    event.start_time_s = event_time_s;
    active_up_[flow_index] = true;
    const uint32_t active_count = ActiveUpCount();
    max_concurrent_up_ = std::max(max_concurrent_up_, active_count);
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
    ProbeEvent& event = events_data_[static_cast<size_t>(event_index)];
    event.end_time_s = event_time_s;
    event.duration_s = std::max(0.0, event.end_time_s - event.start_time_s);
    active_up_[flow_index] = false;
    flows_[flow_index].active_event = -1;
  }

  uint32_t ActiveUpCount() const
  {
    return static_cast<uint32_t>(std::count(active_up_.begin(),
                                            active_up_.end(), true));
  }

  void UpdateActiveEvents()
  {
    for (size_t i = 0; i < flows_.size(); ++i) {
      const int32_t event_index = flows_[i].active_event;
      if (event_index < 0 ||
          static_cast<size_t>(event_index) >= events_data_.size()) {
        continue;
      }
      ProbeEvent& event = events_data_[static_cast<size_t>(event_index)];
      event.max_concurrent_up = std::max(event.max_concurrent_up,
                                         ActiveUpCount());
    }
  }

  void RecordMinuteSample(double now_s,
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
    const double window_duration_s = std::max(
        1e-9, window_end_s - window_start_s);
    minute.flow_received_bytes.assign(flows_.size(), 0);
    minute.flow_goodput_bps.assign(flows_.size(), 0.0);

    uint64_t aggregate_received_bytes = 0;
    std::vector<double> flow_goodputs;
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
      previous_minute_received_bytes_[i] = current_received_bytes;
      const double active_start_s = std::max(window_start_s, flows_[i].start_s);
      const double active_end_s = std::min(window_end_s, flows_[i].stop_s);
      const double active_duration_s =
          std::max(0.0, active_end_s - active_start_s);
      const double flow_goodput_bps = active_duration_s > 0.0
          ? static_cast<double>(received_bytes) * 8.0 / active_duration_s
          : 0.0;
      minute.flow_received_bytes[i] = received_bytes;
      minute.flow_goodput_bps[i] = flow_goodput_bps;
      aggregate_received_bytes += received_bytes;
      if (active_duration_s > 0.0) {
        flow_goodputs.push_back(flow_goodput_bps);
      }
    }
    minute.aggregate_goodput_bps =
        static_cast<double>(aggregate_received_bytes) * 8.0 / window_duration_s;
    minute.mean_flow_goodput_bps = Mean(flow_goodputs);
    minute.jain_fairness = JainFairness(flow_goodputs);
    minute.throughput_recorded = true;
  }

  bool ValidateIdealSequence() const
  {
    if (!ideal_) {
      return true;
    }
    if (events_data_.size() != kFixedFlows) {
      return false;
    }
    std::vector<const ProbeEvent*> ordered_events;
    ordered_events.reserve(events_data_.size());
    for (const ProbeEvent& event : events_data_) {
      ordered_events.push_back(&event);
    }
    std::sort(ordered_events.begin(), ordered_events.end(),
              [](const ProbeEvent* lhs, const ProbeEvent* rhs) {
                return lhs->start_time_s < rhs->start_time_s;
              });
    for (size_t i = 0; i < ordered_events.size(); ++i) {
      const ProbeEvent& event = *ordered_events[i];
      if (event.flow_id != i + 1 || event.probe_order != i + 1 ||
          event.pre_other_up_count != 0 || event.max_concurrent_up != 1 ||
          event.end_time_s < event.start_time_s) {
        return false;
      }
      if (i > 0 && event.start_time_s + 1e-9 <
                       ordered_events[i - 1]->end_time_s) {
        return false;
      }
    }
    return true;
  }

  void WriteEvents()
  {
    for (const ProbeEvent& event : events_data_) {
      events_ << algorithm_ << "," << ModeName() << "," << seed_ << ","
              << run_id_ << "," << event.event_id << "," << event.flow_id
              << "," << event.probe_order << "," << event.pre_other_up_count
              << "," << event.max_concurrent_up << "," << std::fixed
              << std::setprecision(9) << event.start_time_s << ","
              << event.end_time_s << "," << event.duration_s << "\n";
    }
  }

  void WriteStageMetrics()
  {
    const double duration_s = std::max(
        1e-9, measurement_end_s_ - measurement_start_s_);
    std::vector<double> flow_goodputs;
    std::vector<uint64_t> received_bytes(flows_.size(), 0);
    double aggregate_goodput_bps = 0.0;
    for (size_t i = 0; i < flows_.size(); ++i) {
      const uint64_t begin = i < received_bytes_start_.size()
          ? received_bytes_start_[i]
          : 0;
      const uint64_t end = i < received_bytes_end_.size()
          ? received_bytes_end_[i]
          : begin;
      received_bytes[i] = end >= begin ? end - begin : 0;
      const double goodput_bps =
          static_cast<double>(received_bytes[i]) * 8.0 / duration_s;
      flow_goodputs.push_back(goodput_bps);
      aggregate_goodput_bps += goodput_bps;
    }
    const double bdp = static_cast<double>(std::max<uint64_t>(1, bdp_bytes_));
    const uint64_t drop_packets = queue_drop_packets_end_ >= queue_drop_packets_start_
        ? queue_drop_packets_end_ - queue_drop_packets_start_
        : 0;
    const uint64_t drop_bytes = queue_drop_bytes_end_ >= queue_drop_bytes_start_
        ? queue_drop_bytes_end_ - queue_drop_bytes_start_
        : 0;
    const double min_flow_goodput = flow_goodputs.empty() ? 0.0
        : *std::min_element(flow_goodputs.begin(), flow_goodputs.end());
    const double max_flow_goodput = Maximum(flow_goodputs);
    const double mean_excess_inflight = Mean(excess_inflight_samples_);
    const double p95_excess_inflight =
        Percentile(excess_inflight_samples_, 0.95);
    stage_metrics_ << algorithm_ << "," << ModeName() << "," << seed_ << ","
                   << run_id_ << ",0,N4_steady," << kFixedFlows << ",0.000000,"
                   << simulation_time_s_ << "," << measurement_start_s_ << ","
                   << measurement_end_s_ << "," << duration_s << ","
                   << queue_delay_samples_ms_.size() << ","
                   << aggregate_goodput_bps << ","
                   << 100.0 * aggregate_goodput_bps /
                          static_cast<double>(bottleneck_bps_) << ","
                   << JainFairness(flow_goodputs) << "," << Mean(flow_goodputs)
                   << "," << min_flow_goodput << "," << max_flow_goodput << ","
                   << mean_excess_inflight << "," << mean_excess_inflight / bdp
                   << "," << p95_excess_inflight << ","
                   << p95_excess_inflight / bdp << ","
                   << Maximum(excess_inflight_samples_) / bdp << ","
                   << Mean(aggregate_inflight_samples_) << ","
                   << Percentile(aggregate_inflight_samples_, 0.95) << ","
                   << Mean(queue_bytes_samples_) << ","
                   << Mean(queue_delay_samples_ms_) << ","
                   << Percentile(queue_delay_samples_ms_, 0.50) << ","
                   << Percentile(queue_delay_samples_ms_, 0.95) << ","
                   << Percentile(queue_delay_samples_ms_, 0.99) << ","
                   << Maximum(queue_delay_samples_ms_) << "," << drop_packets
                   << "," << drop_bytes << "," << Mean(sum_pacing_samples_)
                   << "," << Mean(bandwidth_estimate_samples_) << ","
                   << (probe_rtt_seen_ ? 1 : 0) << "\n";
    for (size_t i = 0; i < flow_goodputs.size(); ++i) {
      const double share_pct = aggregate_goodput_bps > 0.0
          ? 100.0 * flow_goodputs[i] / aggregate_goodput_bps
          : 0.0;
      flow_metrics_ << algorithm_ << "," << ModeName() << "," << seed_ << ","
                    << run_id_ << ",0,N4_steady," << kFixedFlows << ","
                    << i + 1 << "," << received_bytes[i] << ","
                    << flow_goodputs[i] << "," << share_pct << "\n";
    }
  }

  void WriteMinuteMetrics()
  {
    const double bdp = static_cast<double>(std::max<uint64_t>(1, bdp_bytes_));
    for (size_t i = 0; i < minute_data_.size(); ++i) {
      const MinuteAccumulator& minute = minute_data_[i];
      const double window_start_s = static_cast<double>(i) * kMinuteWindowS;
      const double window_end_s = std::min(
          simulation_time_s_, window_start_s + kMinuteWindowS);
      const double divisor = std::max<uint64_t>(1, minute.sample_count);
      minute_metrics_ << algorithm_ << "," << ModeName() << "," << seed_ << ","
                      << run_id_ << "," << i << "," << window_start_s << ","
                      << window_end_s << "," << minute.sample_count << ","
                      << kFixedFlows << ","
                      << minute.sum_aggregate_inflight_bytes / divisor << ","
                      << minute.sum_excess_inflight_bytes / divisor << ","
                      << minute.sum_excess_inflight_bytes / divisor / bdp << ","
                      << minute.sum_queue_bytes / divisor << ","
                      << minute.sum_queue_delay_ms / divisor << ","
                      << minute.sum_pacing_bps / divisor << ","
                      << minute.sum_bandwidth_estimate_bps / divisor << ","
                      << minute.aggregate_goodput_bps << ","
                      << minute.mean_flow_goodput_bps << ","
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
        const double active_start_s = std::max(window_start_s,
                                               flows_[flow_index].start_s);
        const double active_end_s = std::min(window_end_s,
                                             flows_[flow_index].stop_s);
        const double active_duration_s =
            std::max(0.0, active_end_s - active_start_s);
        const uint64_t received_bytes = flow_index < minute.flow_received_bytes.size()
            ? minute.flow_received_bytes[flow_index]
            : 0;
        const double goodput_bps = flow_index < minute.flow_goodput_bps.size()
            ? minute.flow_goodput_bps[flow_index]
            : 0.0;
        minute_flow_metrics_ << algorithm_ << "," << ModeName() << ","
                           << seed_ << "," << run_id_ << "," << minute_index
                           << "," << window_start_s << "," << window_end_s
                           << "," << flow_index + 1 << ","
                           << (active_duration_s > 0.0 ? 1 : 0) << ","
                           << active_duration_s << "," << received_bytes << ","
                           << goodput_bps << "\n";
      }
    }
  }

  void WriteRunSummary(bool ideal_sequence_valid, bool validation_pass)
  {
    const uint32_t expected_events = ideal_ ? kFixedFlows : 0;
    run_summary_ << algorithm_ << "," << ModeName() << "," << seed_ << ","
                 << run_id_ << "," << simulation_time_s_ << ",1,"
                 << kFixedFlows << "," << expected_events << ","
                 << events_data_.size() << "," << max_concurrent_up_ << ","
                 << (probe_rtt_seen_ ? 1 : 0) << ","
                 << (ideal_sequence_valid ? 1 : 0) << ","
                 << (validation_pass ? 1 : 0) << "\n";
  }

  void WriteMetadata(bool ideal_sequence_valid, bool validation_pass)
  {
    metadata_ << "{\n"
              << "  \"experiment\": \"test2-fixed4\",\n"
              << "  \"algorithm\": \"" << algorithm_ << "\",\n"
              << "  \"mode\": \"" << ModeName() << "\",\n"
              << "  \"seed\": " << seed_ << ",\n"
              << "  \"run_id\": " << run_id_ << ",\n"
              << "  \"simulation_time_s\": " << simulation_time_s_ << ",\n"
              << "  \"active_flows\": " << kFixedFlows << ",\n"
              << "  \"capacity_bps\": " << bottleneck_bps_ << ",\n"
              << "  \"access_bps\": " << access_bps_ << ",\n"
              << "  \"base_rtt_s\": " << base_rtt_s_ << ",\n"
              << "  \"bottleneck_delay_s\": " << bottleneck_delay_s_ << ",\n"
              << "  \"base_bdp_bytes\": " << bdp_bytes_ << ",\n"
              << "  \"queue_bdp\": " << queue_bdp_ << ",\n"
              << "  \"queue_bytes\": " << queue_bytes_ << ",\n"
              << "  \"ideal_sequence_validation\": "
              << (ideal_sequence_valid ? "true" : "false") << ",\n"
              << "  \"validation_pass\": "
              << (validation_pass ? "true" : "false") << "\n"
              << "}\n";
  }

  std::string algorithm_;
  bool ideal_ = false;
  uint32_t seed_ = 0;
  uint32_t run_id_ = 0;
  uint64_t bottleneck_bps_ = 0;
  uint64_t bdp_bytes_ = 0;
  double simulation_time_s_ = 0.0;
  double sample_interval_s_ = 0.0;
  double measurement_guard_s_ = 0.0;
  double ideal_settle_s_ = 0.0;
  double strict_gap_s_ = 0.0;
  double strict_min_up_s_ = 0.0;
  double strict_max_up_s_ = 0.0;
  Ptr<Queue<Packet> > bottleneck_queue_;
  std::string output_prefix_;
  double base_rtt_s_ = 0.0;
  double bottleneck_delay_s_ = 0.0;
  uint64_t access_bps_ = 0;
  double queue_bdp_ = 0.0;
  uint32_t queue_bytes_ = 0;
  std::vector<FlowRuntime> flows_;
  std::vector<MinuteAccumulator> minute_data_;
  std::vector<uint64_t> previous_minute_received_bytes_;
  std::vector<uint64_t> received_bytes_start_;
  std::vector<uint64_t> received_bytes_end_;
  std::vector<double> aggregate_inflight_samples_;
  std::vector<double> excess_inflight_samples_;
  std::vector<double> queue_bytes_samples_;
  std::vector<double> queue_delay_samples_ms_;
  std::vector<double> sum_pacing_samples_;
  std::vector<double> bandwidth_estimate_samples_;
  std::vector<ProbeEvent> events_data_;
  std::vector<bool> active_up_;
  bool measurement_active_ = false;
  double measurement_start_s_ = 0.0;
  double measurement_end_s_ = 0.0;
  uint64_t queue_drop_packets_ = 0;
  uint64_t queue_drop_bytes_ = 0;
  uint64_t queue_drop_packets_start_ = 0;
  uint64_t queue_drop_bytes_start_ = 0;
  uint64_t queue_drop_packets_end_ = 0;
  uint64_t queue_drop_bytes_end_ = 0;
  uint32_t max_concurrent_up_ = 0;
  bool probe_rtt_seen_ = false;
  std::ofstream events_;
  std::ofstream stage_metrics_;
  std::ofstream flow_metrics_;
  std::ofstream minute_metrics_;
  std::ofstream minute_flow_metrics_;
  std::ofstream run_summary_;
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

  // Preserve sub-100-us pacing opportunities in the high-bandwidth scenario.
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
  flow.start_s = 0.001;
  flow.stop_s = simulation_time_s;
  return flow;
}

}  // namespace

int
main(int argc, char* argv[])
{
  std::string algorithm = "BBRv2";
  std::string output_dir = "results/test2/raw";
  uint32_t seed = 1;
  uint32_t run_id = 1;
  uint64_t bottleneck_bps = 100000000ULL;
  uint64_t access_bps = 1000000000ULL;
  double base_rtt_s = 0.040;
  double bottleneck_delay_s = 0.010;
  double queue_bdp = 2.0;
  double simulation_time_s = 300.0;
  double sample_interval_s = 0.100;
  double measurement_guard_s = 0.010;
  double ideal_settle_s = 20.0;
  double strict_gap_s = 0.150;
  double strict_min_up_s = 0.040;
  double strict_max_up_s = 0.080;
  std::string fbbr_config = kDefaultFBBRConfig;

  CommandLine cmd;
  cmd.AddValue("algorithm",
               "BBR-R, oBBR, BBRv2+, CUBIC, BBRv2-formal, BBRv2, or FBBR",
               algorithm);
  cmd.AddValue("outputDir", "Relative output directory", output_dir);
  cmd.AddValue("seed", "Deterministic DQC and ns-3 random seed", seed);
  cmd.AddValue("runId", "Run identifier persisted in CSV", run_id);
  cmd.AddValue("bottleneckBps", "Bottleneck capacity in bit/s", bottleneck_bps);
  cmd.AddValue("accessBps", "Access-link capacity in bit/s", access_bps);
  cmd.AddValue("baseRtt", "End-to-end propagation RTT in seconds", base_rtt_s);
  cmd.AddValue("bottleneckDelay", "One-way bottleneck delay in seconds",
               bottleneck_delay_s);
  cmd.AddValue("queueBdp", "DropTail bottleneck buffer in BDP multiples",
               queue_bdp);
  cmd.AddValue("simulationTime", "Total simulation duration in seconds",
               simulation_time_s);
  cmd.AddValue("sampleInterval", "Aggregate metric sample interval in seconds",
               sample_interval_s);
  cmd.AddValue("measurementGuard",
               "Excluded time at each measurement-window edge in seconds",
               measurement_guard_s);
  cmd.AddValue("idealSettle", "BBRv2-formal settle time in seconds",
               ideal_settle_s);
  cmd.AddValue("strictGap", "Spacing between BBRv2-formal ProbeBW-UP entries",
               strict_gap_s);
  cmd.AddValue("strictMinUp", "BBRv2-formal minimum UP residence in seconds",
               strict_min_up_s);
  cmd.AddValue("strictMaxUp", "BBRv2-formal maximum UP residence in seconds",
               strict_max_up_s);
  cmd.AddValue("fbbrConfig", "FBBR key=value configuration path", fbbr_config);
  cmd.Parse(argc, argv);

  const bool ideal = algorithm == "BBRv2-formal";
  if (!IsKnownAlgorithm(algorithm) || bottleneck_bps == 0 || access_bps == 0 ||
      base_rtt_s <= 0.0 || bottleneck_delay_s <= 0.0 || queue_bdp <= 0.0 ||
      simulation_time_s <= 0.0 || sample_interval_s <= 0.0 ||
      measurement_guard_s < 0.0 || strict_gap_s <= 0.0 ||
      strict_min_up_s < 0.0 || strict_max_up_s <= 0.0) {
    std::cerr << "Invalid Test 2 arguments" << std::endl;
    return 1;
  }
  if (2.0 * measurement_guard_s >= simulation_time_s) {
    std::cerr << "Measurement guard leaves no Test 2 observation window"
              << std::endl;
    return 1;
  }
  if (ideal && ideal_settle_s + (kFixedFlows - 1) * strict_gap_s +
                   strict_max_up_s >= simulation_time_s - measurement_guard_s) {
    std::cerr << "Test 2 is too short for the BBRv2-formal UP sequence"
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
  senders.Create(kFixedFlows);
  routers.Create(2);
  receivers.Create(kFixedFlows);

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
  for (uint32_t i = 0; i < kFixedFlows; ++i) {
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

  for (uint32_t i = 0; i < kFixedFlows; ++i) {
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
  dqc::FBBRConfig fbbr_config_values;
  if (IsFBBRAlgorithm(algorithm) &&
      !LoadExperimentFBBRConfig(fbbr_config, &fbbr_config_values)) {
    return 1;
  }
  std::vector<FlowRuntime> flows;
  flows.reserve(kFixedFlows);
  for (uint32_t i = 0; i < kFixedFlows; ++i) {
    flows.push_back(InstallFlow(cc_type, algorithm, fbbr_config_values,
                                senders.Get(i), receivers.Get(i), i,
                                simulation_time_s, send_buffer_bytes));
  }

  FixedFourExperiment experiment(
      algorithm, seed, run_id, bottleneck_bps, bdp_bytes, simulation_time_s,
      sample_interval_s, measurement_guard_s, ideal_settle_s, strict_gap_s,
      strict_min_up_s, strict_max_up_s, bottleneck_queue, output_prefix,
      base_rtt_s, bottleneck_delay_s, access_bps, queue_bdp, queue_bytes);
  experiment.SetFlows(flows);
  experiment.OpenOutputs();
  experiment.StartMinuteThroughputTrace();
  if (bottleneck_queue != nullptr) {
    bottleneck_queue->TraceConnectWithoutContext(
        "Drop", MakeCallback(&FixedFourExperiment::OnQueueDrop, &experiment));
  }
  if (ideal) {
    for (uint32_t i = 0; i < kFixedFlows; ++i) {
      flows[i].sender->SetBbr2ExperimentPhaseTrace(
          [&experiment, i](double event_time_s, const std::string& phase) {
            experiment.OnProbePhase(i, event_time_s, phase);
          });
    }
    experiment.ConfigureFormalProbeUps();
  }

  Simulator::Schedule(Seconds(measurement_guard_s),
                      &FixedFourExperiment::StartMeasurement, &experiment);
  Simulator::Schedule(Seconds(simulation_time_s - measurement_guard_s),
                      &FixedFourExperiment::FinishMeasurement, &experiment);
  Simulator::Schedule(Seconds(measurement_guard_s + sample_interval_s),
                      &FixedFourExperiment::Sample, &experiment);
  std::cout << "test2-fixed4 algorithm=" << algorithm
            << " capacity=" << bottleneck_bps
            << " baseRtt=" << base_rtt_s
            << " queueBytes=" << queue_bytes
            << " output=" << output_prefix << std::endl;
  Simulator::Stop(Seconds(simulation_time_s));
  Simulator::Run();
  const bool validation_pass = experiment.Finish();
  Simulator::Destroy();
  return validation_pass ? 0 : 2;
}
