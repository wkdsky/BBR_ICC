// Copyright 2019 The Chromium Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "quic_bbr2_sender.h"

#include <cstddef>
#include <limits>

#include "quic_bandwidth_sampler.h"
#include "quic_bbr2_drain.h"
#include "quic_bbr2_misc.h"
#include "proto_bandwidth.h"
#include "flag_impl.h"
#include "flag_util_impl.h"
#include "quic_logging.h"
#include "rtt_stats.h"
#include "unacked_packet_map.h"
namespace dqc {

namespace {
// Constants based on TCP defaults.
// The minimum CWND to ensure delayed acks don't reduce bandwidth measurements.
// Does not inflate the pacing rate.
const QuicByteCount kDefaultMinimumCongestionWindow = 4 * kMaxSegmentSize;

const float kInitialPacingGain = 2.885f;

const int kMaxModeChangesPerCongestionEvent = 4;

struct ExperimentalStrictProbeUpScheduler {
  bool enabled = false;
  uint32_t total_orders = 0;
  uint32_t next_order = 1;
  uint32_t active_order = 0;
};

ExperimentalStrictProbeUpScheduler& StrictProbeUpScheduler() {
  static ExperimentalStrictProbeUpScheduler scheduler;
  return scheduler;
}
}  // namespace

// Call |member_function_call| based on the current Bbr2Mode we are in. e.g.
//
//   auto result = BBR2_MODE_DISPATCH(Foo());
//
// is equivalent to:
//
//   Bbr2ModeBase& Bbr2Sender::GetCurrentMode() {
//     if (mode_ == Bbr2Mode::STARTUP) { return startup_; }
//     if (mode_ == Bbr2Mode::DRAIN) { return drain_; }
//     ...
//   }
//   auto result = GetCurrentMode().Foo();
//
// Except that BBR2_MODE_DISPATCH guarantees the call to Foo() is non-virtual.
//
#define BBR2_MODE_DISPATCH(member_function_call)     \
  (mode_ == Bbr2Mode::STARTUP                        \
       ? (startup_.member_function_call)             \
       : (mode_ == Bbr2Mode::PROBE_BW                \
              ? (probe_bw_.member_function_call)     \
              : (mode_ == Bbr2Mode::DRAIN            \
                     ? (drain_.member_function_call) \
                     : (probe_rtt_or_die().member_function_call))))

Bbr2Sender::Bbr2Sender(QuicTime now,
                       const RttStats* rtt_stats,
                       const QuicUnackedPacketMap* unacked_packets,
                       QuicPacketCount initial_cwnd_in_packets,
                       QuicPacketCount max_cwnd_in_packets,
                       Random* random,
                       QuicConnectionStats* stats,
                       bool enable_ecn,
                       QuicBbrSender* old_sender,
                       CongestionControlType congestion_control_type,
                       bool enable_probe_rtt)
    : mode_(Bbr2Mode::STARTUP),
      rtt_stats_(rtt_stats),
      unacked_packets_(unacked_packets),
      random_(random),
      connection_stats_(stats),
      congestion_control_type_(congestion_control_type),
      enable_probe_rtt_(enable_probe_rtt),
      params_(kDefaultMinimumCongestionWindow,
              max_cwnd_in_packets * kDefaultTCPMSS),
      model_(&params_,
             rtt_stats->SmoothedOrInitialRtt(),
             rtt_stats->last_update_time(),
             /*cwnd_gain=*/params_.startup_cwnd_gain,
             /*pacing_gain=*/params_.startup_pacing_gain,
             old_sender ? &old_sender->sampler_ : nullptr),
      initial_cwnd_(
          cwnd_limits().ApplyLimits(initial_cwnd_in_packets * kDefaultTCPMSS)),
      cwnd_(initial_cwnd_),
      pacing_rate_(kInitialPacingGain * QuicBandwidth::FromBytesAndTimeDelta(
                                            cwnd_,
                                            rtt_stats->SmoothedOrInitialRtt())),
      startup_(this, &model_, now),
      drain_(this, &model_),
      probe_bw_(this, &model_),
      probe_rtt_(this, &model_),
      last_sample_is_app_limited_(false) {
  QUIC_DVLOG(2) << this << " Initializing Bbr2Sender. mode:" << mode_
                << ", PacingRate:" << pacing_rate_ << ", Cwnd:" << cwnd_
                << ", CwndLimits:" << cwnd_limits() << "  @ " << now;
  params_.enable_ecn=enable_ecn;
  DCHECK_EQ(mode_, Bbr2Mode::STARTUP);
}

QuicLimits<QuicByteCount> Bbr2Sender::GetCwndLimitsByMode() const {
  switch (mode_) {
    case Bbr2Mode::STARTUP:
      return startup_.GetCwndLimits();
    case Bbr2Mode::PROBE_BW:
      return probe_bw_.GetCwndLimits();
    case Bbr2Mode::DRAIN:
      return drain_.GetCwndLimits();
    case Bbr2Mode::PROBE_RTT:
      return probe_rtt_.GetCwndLimits();
    default:
      QUIC_NOTREACHED();
      return Unlimited<QuicByteCount>();
  }
}

const QuicLimits<QuicByteCount>& Bbr2Sender::cwnd_limits() const {
  return Params().cwnd_limits;
}

void Bbr2Sender::AdjustNetworkParameters(QuicBandwidth bandwidth,
                               TimeDelta rtt,
                               bool allow_cwnd_to_decrease) {
  model_.UpdateNetworkParameters(bandwidth, rtt);

  if (mode_ != Bbr2Mode::STARTUP) {
    return;
  }

  const QuicByteCount prior_cwnd = cwnd_;
  QuicBandwidth effective_bandwidth =
      std::max(bandwidth, model_.BandwidthEstimate());
  connection_stats_->cwnd_bootstrapping_rtt_us =
      model_.MinRtt().ToMicroseconds();

  cwnd_ = cwnd_limits().ApplyLimits(
      std::min(max_cwnd_when_network_parameters_adjusted_,
               model_.BDP(effective_bandwidth)));

  if (!allow_cwnd_to_decrease) {
    cwnd_ = std::max(cwnd_, prior_cwnd);
  }

  pacing_rate_ = std::max(pacing_rate_,
                          QuicBandwidth::FromBytesAndTimeDelta(
                              cwnd_, model_.MinRtt()));
}

void Bbr2Sender::SetInitialCongestionWindowInPackets(
    QuicPacketCount congestion_window) {
  if (mode_ == Bbr2Mode::STARTUP) {
    // The cwnd limits is unchanged and still applies to the new cwnd.
    cwnd_ = cwnd_limits().ApplyLimits(congestion_window * kDefaultTCPMSS);
  }
}

void Bbr2Sender::OnCongestionEvent(bool /*rtt_updated*/,
                                   QuicByteCount prior_in_flight,
                                   QuicTime event_time,
                                   const AckedPacketVector& acked_packets,
                                   const LostPacketVector& lost_packets) {
  QUIC_DVLOG(3) << this
                << " OnCongestionEvent. prior_in_flight:" << prior_in_flight
                << " prior_cwnd:" << cwnd_ << "  @ " << event_time;
  Bbr2CongestionEvent congestion_event;
  congestion_event.prior_cwnd = cwnd_;
  congestion_event.prior_bytes_in_flight = prior_in_flight;
  congestion_event.is_probing_for_bandwidth =
      BBR2_MODE_DISPATCH(IsProbingForBandwidth());

  model_.OnCongestionEventStart(event_time, acked_packets, lost_packets,
                                &congestion_event);

  if (InSlowStart()) {
    if (!lost_packets.empty()) {
      connection_stats_->slowstart_packets_lost += lost_packets.size();
      connection_stats_->slowstart_bytes_lost += congestion_event.bytes_lost;
    }
    if (congestion_event.end_of_round_trip) {
      ++connection_stats_->slowstart_num_rtts;
    }
  }

  if (!acked_packets.empty()) {
    last_ack_event_time_ = event_time;
  }
  if (congestion_event.end_of_round_trip&&params_.enable_ecn){
      UpdateRoundTripAlpha();
  }
  OnCongestionEventStarted(congestion_event);
  // Number of mode changes allowed for this congestion event.
  int mode_changes_allowed = kMaxModeChangesPerCongestionEvent;
  while (true) {
    Bbr2Mode next_mode = BBR2_MODE_DISPATCH(
        OnCongestionEvent(prior_in_flight, event_time, acked_packets,
                          lost_packets, congestion_event));

    if (next_mode == mode_) {
      break;
    }

    QUIC_DVLOG(2) << this << " Mode change:  " << mode_ << " ==> " << next_mode
                  << "  @ " << event_time;
    BBR2_MODE_DISPATCH(Leave(event_time, &congestion_event));
    mode_ = next_mode;
    BBR2_MODE_DISPATCH(Enter(event_time, &congestion_event));
    --mode_changes_allowed;
    if (mode_changes_allowed < 0) {
      QUIC_BUG << "Exceeded max number of mode changes per congestion event.";
      break;
    }
  }

  UpdatePacingRate(congestion_event.bytes_acked);
  QUIC_BUG_IF(pacing_rate_.IsZero()) << "Pacing rate must not be zero!";

  UpdateCongestionWindow(congestion_event.bytes_acked);
  QUIC_BUG_IF(cwnd_ == 0u) << "Congestion window must not be zero!";

  model_.OnCongestionEventFinish(unacked_packets_->GetLeastUnacked(),
                                 congestion_event);
  if (congestion_event.end_of_round_trip) {
    bytes_ecn_in_round_=0;
  }
  last_sample_is_app_limited_ =
      congestion_event.last_packet_send_state.is_valid
          ? congestion_event.last_packet_send_state.is_app_limited
          : congestion_event.last_sample_is_app_limited;
  if (!last_sample_is_app_limited_) {
    has_non_app_limited_sample_ = true;
  }
  if (congestion_event.bytes_in_flight == 0 &&
      Params().avoid_unnecessary_probe_rtt) {
    OnEnterQuiescence(event_time);
  }

  if (queue_delay_trace_cb_ && rtt_stats_ != nullptr) {
    uint32_t latest_rtt_ms =
        static_cast<uint32_t>(rtt_stats_->latest_rtt().ToMilliseconds());
    uint32_t min_rtt_ms =
        static_cast<uint32_t>(rtt_stats_->MinOrInitialRtt().ToMilliseconds());
    uint32_t queue_delay_ms = 0;
    if (latest_rtt_ms > min_rtt_ms) {
      queue_delay_ms = latest_rtt_ms - min_rtt_ms;
    }
    queue_delay_trace_cb_(queue_delay_ms, latest_rtt_ms, min_rtt_ms);
  }

  /*QUIC_DVLOG(3)
      << this << " END CongestionEvent(acked:" << acked_packets
      << ", lost:" << lost_packets.size() << ") "
      << ", Mode:" << mode_ << ", RttCount:" << model_.RoundTripCount()
      << ", BytesInFlight:" << congestion_event.bytes_in_flight
      << ", PacingRate:" << PacingRate(0) << ", CWND:" << GetCongestionWindow()
      << ", PacingGain:" << model_.pacing_gain()
      << ", CwndGain:" << model_.cwnd_gain()
      << ", BandwidthEstimate(kbps):" << BandwidthEstimate().ToKBitsPerSecond()
      << ", MinRTT(us):" << model_.MinRtt().ToMicroseconds()
      << ", BDP:" << model_.BDP(BandwidthEstimate())
      << ", BandwidthLatest(kbps):"
      << model_.bandwidth_latest().ToKBitsPerSecond()
      << ", BandwidthLow(kbps):" << model_.bandwidth_lo().ToKBitsPerSecond()
      << ", BandwidthHigh(kbps):" << model_.MaxBandwidth().ToKBitsPerSecond()
      << ", InflightLatest:" << model_.inflight_latest()
      << ", InflightLow:" << model_.inflight_lo()
      << ", InflightHigh:" << model_.inflight_hi()
      << ", TotalAcked:" << model_.total_bytes_acked()
      << ", TotalLost:" << model_.total_bytes_lost()
      << ", TotalSent:" << model_.total_bytes_sent() << "  @ " << event_time;*/
}

void Bbr2Sender::UpdatePacingRate(QuicByteCount bytes_acked) {
  if (BandwidthEstimate().IsZero()) {
    return;
  }

  if (model_.total_bytes_acked() == bytes_acked) {
    // After the first ACK, cwnd_ is still the initial congestion window.
    pacing_rate_ = QuicBandwidth::FromBytesAndTimeDelta(cwnd_, model_.MinRtt());
    return;
  }

  QuicBandwidth target_rate = model_.pacing_gain() * model_.BandwidthEstimate();
  if (model_.full_bandwidth_reached()) {
    pacing_rate_ = target_rate;
    return;
  }
  if (params_.decrease_startup_pacing_at_end_of_round &&
      model_.pacing_gain() < Params().startup_pacing_gain) {
    pacing_rate_ = target_rate;
    return;
  }
  if (params_.bw_lo_mode_ != Bbr2Params::DEFAULT &&
      model_.loss_events_in_round() > 0) {
    pacing_rate_ = target_rate;
    return;
  }

  if (target_rate > pacing_rate_) {
    pacing_rate_ = target_rate;
  }
}

void Bbr2Sender::UpdateCongestionWindow(QuicByteCount bytes_acked) {
  QuicByteCount target_cwnd = GetTargetCongestionWindow(model_.cwnd_gain());
  const QuicByteCount compensation = GetCwndCompensationBytes();
  if (compensation > std::numeric_limits<QuicByteCount>::max() - target_cwnd) {
    target_cwnd = std::numeric_limits<QuicByteCount>::max();
  } else {
    target_cwnd += compensation;
  }

  const QuicByteCount prior_cwnd = cwnd_;
  if (model_.full_bandwidth_reached() || Params().startup_include_extra_acked) {
    target_cwnd += model_.MaxAckHeight();
    cwnd_ = std::min(prior_cwnd + bytes_acked, target_cwnd);
  } else if (prior_cwnd < target_cwnd || prior_cwnd < 2 * initial_cwnd_) {
    cwnd_ = prior_cwnd + bytes_acked;
  }
  const QuicByteCount desired_cwnd = cwnd_;

  cwnd_ = GetCwndLimitsByMode().ApplyLimits(cwnd_);
  const QuicByteCount model_limited_cwnd = cwnd_;

  cwnd_ = cwnd_limits().ApplyLimits(cwnd_);

  QUIC_DVLOG(3) << this << " Updating CWND. target_cwnd:" << target_cwnd
                << ", max_ack_height:" << model_.MaxAckHeight()
                << ", full_bw:" << model_.full_bandwidth_reached()
                << ", bytes_acked:" << bytes_acked
                << ", inflight_lo:" << model_.inflight_lo()
                << ", inflight_hi:" << model_.inflight_hi() << ". (prior_cwnd) "
                << prior_cwnd << " => (desired_cwnd) " << desired_cwnd
                << " => (model_limited_cwnd) " << model_limited_cwnd
                << " => (final_cwnd) " << cwnd_;
}

QuicByteCount Bbr2Sender::GetTargetCongestionWindow(float gain) const {
  return std::max(model_.BDP(model_.BandwidthEstimate(), gain),
                  cwnd_limits().Min());
}

QuicByteCount Bbr2Sender::GetCwndCompensationBytes() const {
  return 0;
}

void Bbr2Sender::OnPacketSent(QuicTime sent_time,
                              QuicByteCount bytes_in_flight,
                              QuicPacketNumber packet_number,
                              QuicByteCount bytes,
                              HasRetransmittableData is_retransmittable) {
  QUIC_DVLOG(3) << this << " OnPacketSent: pkn:" << packet_number
                << ", bytes:" << bytes << ", cwnd:" << cwnd_
                << ", inflight:" << bytes_in_flight + bytes
                << ", total_sent:" << model_.total_bytes_sent() + bytes
                << ", total_acked:" << model_.total_bytes_acked()
                << ", total_lost:" << model_.total_bytes_lost() << "  @ "
                << sent_time;
  if (InSlowStart()) {
    ++connection_stats_->slowstart_packets_sent;
    connection_stats_->slowstart_bytes_sent += bytes;
  }
  if (bytes_in_flight == 0 && Params().avoid_unnecessary_probe_rtt) {
    OnExitQuiescence(sent_time);
  }
  model_.OnPacketSent(sent_time, bytes_in_flight, packet_number, bytes,
                      is_retransmittable);
}

void Bbr2Sender::OnPacketNeutered(QuicPacketNumber packet_number) {
  model_.OnPacketNeutered(packet_number);
}

bool Bbr2Sender::CanSend(QuicByteCount bytes_in_flight) {
  const bool result = bytes_in_flight < GetCongestionWindow();
  return result;
}

QuicByteCount Bbr2Sender::GetCongestionWindow() const {
  // TODO(wub): Implement Recovery?
  return cwnd_;
}

QuicBandwidth Bbr2Sender::PacingRate(QuicByteCount /*bytes_in_flight*/) const {
  return pacing_rate_;
}

void Bbr2Sender::OnApplicationLimited(QuicByteCount bytes_in_flight) {
  if (bytes_in_flight >= GetCongestionWindow()) {
    return;
  }
  if (Params().flexible_app_limited && IsPipeSufficientlyFull()) {
    return;
  }

  model_.OnApplicationLimited();
  QUIC_DVLOG(2) << this << " Becoming application limited. Last sent packet: "
                << model_.last_sent_packet()
                << ", CWND: " << GetCongestionWindow();
}
void Bbr2Sender::OnUpdateEcnBytes(uint64_t ecn_ce_count){
    QuicByteCount previous=ecn_ce_count_;
    if(ecn_ce_count>previous&&params_.enable_ecn){
        ecn_ce_count_=ecn_ce_count;
        bytes_ecn_in_round_+=(ecn_ce_count_-previous);
        model_.OnEcnUpdate();
    }
}

void Bbr2Sender::SetExperimentalForcedProbeUp(
    QuicTime probe_up_time,
    TimeDelta min_probe_up_duration) {
  experimental_forced_probe_up_enabled_ = true;
  experimental_forced_probe_up_started_ = false;
  experimental_forced_probe_up_time_ = probe_up_time;
  experimental_forced_probe_up_start_time_ = QuicTime::Zero();
  experimental_forced_probe_up_min_duration_ = min_probe_up_duration;
}

void Bbr2Sender::SetExperimentalStrictProbeUp(
    uint32_t probe_order,
    uint32_t total_probe_orders,
    QuicTime probe_up_time,
    TimeDelta min_probe_up_duration,
    TimeDelta max_probe_up_duration) {
  SetExperimentalForcedProbeUp(probe_up_time, min_probe_up_duration);
  experimental_strict_probe_up_enabled_ = probe_order > 0 &&
      total_probe_orders > 0 && probe_order <= total_probe_orders;
  experimental_strict_probe_up_finished_ = false;
  experimental_strict_probe_up_order_ = probe_order;
  experimental_strict_probe_up_total_orders_ = total_probe_orders;
  experimental_strict_probe_up_max_duration_ =
      max_probe_up_duration < min_probe_up_duration
          ? min_probe_up_duration
          : max_probe_up_duration;

  if (!experimental_strict_probe_up_enabled_) {
    return;
  }

  // The scenario configures order one first, before Simulator::Run().  Reset
  // the process-local scheduler so repeated in-process experiments do not
  // inherit a completed token sequence.
  if (probe_order == 1) {
    ExperimentalStrictProbeUpScheduler& scheduler = StrictProbeUpScheduler();
    scheduler.enabled = true;
    scheduler.total_orders = total_probe_orders;
    scheduler.next_order = 1;
    scheduler.active_order = 0;
  }
}

bool Bbr2Sender::ShouldForceProbeUp(QuicTime now) const {
  if (!experimental_forced_probe_up_enabled_ ||
      experimental_forced_probe_up_started_ ||
      now < experimental_forced_probe_up_time_) {
    return false;
  }
  if (!experimental_strict_probe_up_enabled_) {
    return true;
  }
  const ExperimentalStrictProbeUpScheduler& scheduler =
      StrictProbeUpScheduler();
  return scheduler.enabled && scheduler.active_order == 0 &&
         scheduler.next_order == experimental_strict_probe_up_order_ &&
         scheduler.next_order <= scheduler.total_orders;
}

void Bbr2Sender::MarkExperimentalForcedProbeUpStarted(QuicTime now) {
  experimental_forced_probe_up_started_ = true;
  experimental_forced_probe_up_start_time_ = now;
  if (experimental_strict_probe_up_enabled_) {
    ExperimentalStrictProbeUpScheduler& scheduler = StrictProbeUpScheduler();
    if (scheduler.enabled && scheduler.next_order ==
                                 experimental_strict_probe_up_order_ &&
        scheduler.active_order == 0) {
      scheduler.active_order = experimental_strict_probe_up_order_;
    }
  }
}

bool Bbr2Sender::ExperimentalForcedProbeUpExitAllowed(QuicTime now) {
  if (!experimental_forced_probe_up_enabled_ ||
      experimental_forced_probe_up_min_duration_.IsZero() ||
      experimental_forced_probe_up_start_time_ == QuicTime::Zero()) {
    return true;
  }
  const bool exit_allowed = now - experimental_forced_probe_up_start_time_ >=
      experimental_forced_probe_up_min_duration_;
  if (exit_allowed && experimental_strict_probe_up_enabled_ &&
      !experimental_strict_probe_up_finished_) {
    ExperimentalStrictProbeUpScheduler& scheduler = StrictProbeUpScheduler();
    if (scheduler.enabled && scheduler.active_order ==
                                 experimental_strict_probe_up_order_) {
      scheduler.active_order = 0;
      scheduler.next_order = experimental_strict_probe_up_order_ + 1;
    }
    experimental_strict_probe_up_finished_ = true;
  }
  return exit_allowed;
}

bool Bbr2Sender::ExperimentalForcedProbeUpMustExit(QuicTime now) const {
  return experimental_forced_probe_up_enabled_ &&
         experimental_forced_probe_up_started_ &&
         experimental_forced_probe_up_start_time_ != QuicTime::Zero() &&
         !experimental_strict_probe_up_max_duration_.IsZero() &&
         now - experimental_forced_probe_up_start_time_ >=
             experimental_strict_probe_up_max_duration_;
}

bool Bbr2Sender::ShouldBlockNativeProbeUpForExperiment() const {
  // Keep the gate closed after the controlled sequence too.  The scenario
  // stops shortly afterwards, which makes every observed UP attributable to a
  // single scheduled event.
  return experimental_strict_probe_up_enabled_;
}

void Bbr2Sender::SetExperimentalMaxCongestionWindowPackets(
    QuicPacketCount max_cwnd_in_packets) {
  if (max_cwnd_in_packets == 0) {
    return;
  }
  params_.cwnd_limits =
      QuicLimits<QuicByteCount>(params_.cwnd_limits.Min(),
                                max_cwnd_in_packets * kDefaultTCPMSS);
}

QuicByteCount Bbr2Sender::GetTargetBytesInflight() const {
  QuicByteCount bdp = model_.BDP(model_.BandwidthEstimate());
  return std::min(bdp, GetCongestionWindow());
}

QuicByteCount Bbr2Sender::GetCwndModeUpperBoundForExperiment() const {
  return GetCwndLimitsByMode().Max();
}

QuicByteCount Bbr2Sender::GetCwndGlobalUpperBoundForExperiment() const {
  return cwnd_limits().Max();
}

/*void Bbr2Sender::PopulateConnectionStats(QuicConnectionStats* stats) const {
  stats->num_ack_aggregation_epochs = model_.num_ack_aggregation_epochs();
}*/

void Bbr2Sender::OnEnterQuiescence(QuicTime now) {
  last_quiescence_start_ = now;
}

void Bbr2Sender::OnExitQuiescence(QuicTime now) {
  if (last_quiescence_start_ != QuicTime::Zero()) {
    Bbr2Mode next_mode = BBR2_MODE_DISPATCH(
        OnExitQuiescence(now, std::min(now, last_quiescence_start_)));
    if (next_mode != mode_) {
      BBR2_MODE_DISPATCH(Leave(now, nullptr));
      mode_ = next_mode;
      BBR2_MODE_DISPATCH(Enter(now, nullptr));
    }
    last_quiescence_start_ = QuicTime::Zero();
  }
}

bool Bbr2Sender::ShouldSendProbingPacket() const {
  // TODO(wub): Implement ShouldSendProbingPacket properly.
  if (!BBR2_MODE_DISPATCH(IsProbingForBandwidth())) {
    return false;
  }

  // TODO(b/77975811): If the pipe is highly under-utilized, consider not
  // sending a probing transmission, because the extra bandwidth is not needed.
  // If flexible_app_limited is enabled, check if the pipe is sufficiently full.
  if (Params().flexible_app_limited) {
    const bool is_pipe_sufficiently_full = IsPipeSufficientlyFull();
    QUIC_DVLOG(3) << this << " CWND: " << GetCongestionWindow()
                  << ", inflight: " << unacked_packets_->bytes_in_flight()
                  << ", pacing_rate: " << PacingRate(0)
                  << ", flexible_app_limited: true, ShouldSendProbingPacket: "
                  << !is_pipe_sufficiently_full;
    return !is_pipe_sufficiently_full;
  } else {
    return true;
  }
}

bool Bbr2Sender::IsPipeSufficientlyFull() const {
  QuicByteCount bytes_in_flight = unacked_packets_->bytes_in_flight();
  // See if we need more bytes in flight to see more bandwidth.
  if (mode_ == Bbr2Mode::STARTUP) {
    // STARTUP exits if it doesn't observe a 25% bandwidth increase, so the CWND
    // must be more than 25% above the target.
    return bytes_in_flight >= GetTargetCongestionWindow(1.5);
  }
  if (model_.pacing_gain() > 1) {
    // Super-unity PROBE_BW doesn't exit until 1.25 * BDP is achieved.
    return bytes_in_flight >= GetTargetCongestionWindow(model_.pacing_gain());
  }
  // If bytes_in_flight are above the target congestion window, it should be
  // possible to observe the same or more bandwidth if it's available.
  return bytes_in_flight >= GetTargetCongestionWindow(1.1);
}
void Bbr2Sender::UpdateRoundTripAlpha(){
    uint32_t delivered_ce=uint32_t(ecn_ce_count_-alpha_last_delivered_ce_);
    QuicByteCount total_bytes_sent=unacked_packets_->delivered();
    uint32_t delivered=0;
    if(total_bytes_sent>=alpha_last_delivered_){
        delivered=total_bytes_sent-alpha_last_delivered_;
    }
    if(delivered==0){return;}
	uint64_t ce_ratio = (uint64_t)delivered_ce << BBR_SCALE;
	ce_ratio=ce_ratio/delivered;
	uint32_t gain =params_.ecn_alpha_gain;
	uint64_t alpha = ((BBR_UNIT - gain) * params_.ecn_alpha) >> BBR_SCALE;
	alpha += (gain * ce_ratio) >> BBR_SCALE;
	params_.ecn_alpha = std::min((uint32_t)alpha, (uint32_t)BBR_UNIT);
    alpha_last_delivered_=total_bytes_sent;
    alpha_last_delivered_ce_=ecn_ce_count_;
    if(mode_==Bbr2Mode::STARTUP){
        startup_.CheckEcnTooHigh(ce_ratio);
    }
}
std::string Bbr2Sender::GetDebugState() const {
  std::ostringstream stream;
  stream << ExportDebugState();
  return stream.str();
}

Bbr2Sender::DebugState Bbr2Sender::ExportDebugState() const {
  DebugState s;
  s.mode = mode_;
  s.round_trip_count = model_.RoundTripCount();
  s.bandwidth_hi = model_.MaxBandwidth();
  s.bandwidth_lo = model_.bandwidth_lo();
  s.bandwidth_est = BandwidthEstimate();
  s.inflight_hi = model_.inflight_hi();
  s.inflight_lo = model_.inflight_lo();
  s.max_ack_height = model_.MaxAckHeight();
  s.min_rtt = model_.MinRtt();
  s.min_rtt_timestamp = model_.MinRttTimestamp();
  s.congestion_window = cwnd_;
  s.pacing_rate = pacing_rate_;
  s.last_sample_is_app_limited = last_sample_is_app_limited_;
  s.end_of_app_limited_phase = model_.end_of_app_limited_phase();

  s.startup = startup_.ExportDebugState();
  s.drain = drain_.ExportDebugState();
  s.probe_bw = probe_bw_.ExportDebugState();
  s.probe_rtt = probe_rtt_.ExportDebugState();

  return s;
}

int32_t Bbr2Sender::GetCurrentBbrModeIndex() const {
  // Get debug state
  DebugState state = ExportDebugState();

  // Convert mode and probe_bw phase to single index
  // 0: STARTUP
  // 1: DRAIN
  // 2: PROBE_BW_DOWN
  // 3: PROBE_BW_CRUISE
  // 4: PROBE_BW_REFILL
  // 5: PROBE_BW_UP
  // 6: PROBE_RTT
  switch (state.mode) {
    case Bbr2Mode::STARTUP:
      return 0;
    case Bbr2Mode::DRAIN:
      return 1;
    case Bbr2Mode::PROBE_BW:
      // Map probe_bw phase to index
      switch (state.probe_bw.phase) {
        case Bbr2ProbeBwMode::CyclePhase::PROBE_DOWN:
          return 2;
        case Bbr2ProbeBwMode::CyclePhase::PROBE_CRUISE:
          return 3;
        case Bbr2ProbeBwMode::CyclePhase::PROBE_REFILL:
          return 4;
        case Bbr2ProbeBwMode::CyclePhase::PROBE_UP:
          return 5;
        case Bbr2ProbeBwMode::CyclePhase::PROBE_PRE_UP:
          return 7;
        case Bbr2ProbeBwMode::CyclePhase::PROBE_GUARD:
          return 8;
        case Bbr2ProbeBwMode::CyclePhase::PROBE_POST_UP:
          return 9;
        case Bbr2ProbeBwMode::CyclePhase::PROBE_DOWN_SLIGHTLY:
          return 10;
        case Bbr2ProbeBwMode::CyclePhase::PROBE_NOT_STARTED:
        default:
          return 2;  // Default to PROBE_DOWN
      }
    case Bbr2Mode::PROBE_RTT:
      return 6;
    default:
      return 0;
  }
}

void Bbr2Sender::OnCongestionEventStarted(
    const Bbr2CongestionEvent& /*congestion_event*/) {}

bool Bbr2Sender::EnablePlusProbeBwPhases() const {
  return false;
}

bool Bbr2Sender::ShouldStartProbeOnRound() const {
  return false;
}

bool Bbr2Sender::ShouldAdvanceMaxBandwidthFilterOnRoundStart(
    Bbr2ProbeBwMode::CyclePhase /*phase*/) const {
  return false;
}

void Bbr2Sender::OnMaxBandwidthFilterAdvanced(
    Bbr2ProbeBwMode::CyclePhase /*phase*/) {}

bool Bbr2Sender::ShouldEnterProbeUpFromGuard() const {
  return true;
}

bool Bbr2Sender::ShouldProbeAgainFromPostUp() const {
  return false;
}

bool Bbr2Sender::ShouldDelayProbeUpExit(QuicTime /*now*/) const {
  return false;
}

bool Bbr2Sender::ShouldExitProbeUpAfterRound() const {
  return false;
}

bool Bbr2Sender::ShouldDelayProbeBwCruiseExit(QuicTime /*now*/) const {
  return false;
}

bool Bbr2Sender::ConsumeStartupRestartRequest() {
  return false;
}

void Bbr2Sender::PrepareForStartupRestart() {
  // STARTUP requires a fresh lower bound and full-bandwidth detector.
  model_.clear_bandwidth_lo();
  model_.clear_inflight_lo();
  model_.ResetFullBandwidthForStartup();
  model_.RestartRoundEarly();
}

bool Bbr2Sender::HasCustomProbeDownLogic() const {
  return false;
}

bool Bbr2Sender::ShouldExitCustomProbeDown(
    QuicByteCount /*bytes_in_flight*/,
    QuicByteCount /*bdp*/) const {
  return false;
}

float Bbr2Sender::GetProbeBwPacingGain(Bbr2ProbeBwMode::CyclePhase /*phase*/,
                                       float pacing_gain) const {
  return pacing_gain;
}

float Bbr2Sender::GetProbeBwCwndGain(Bbr2ProbeBwMode::CyclePhase /*phase*/,
                                     float cwnd_gain) const {
  return cwnd_gain;
}

void Bbr2Sender::OnProbeBwPhaseEntered(Bbr2ProbeBwMode::CyclePhase phase,
                                       QuicTime now) {
  if (experiment_probe_phase_trace_callback_) {
    experiment_probe_phase_trace_callback_(phase, now);
  }
}

std::ostream& operator<<(std::ostream& os, const Bbr2Sender::DebugState& s) {
  os << "mode: " << s.mode << "\n";
  os << "round_trip_count: " << s.round_trip_count << "\n";
  os << "bandwidth_hi ~ lo ~ est: " << s.bandwidth_hi << " ~ " << s.bandwidth_lo
     << " ~ " << s.bandwidth_est << "\n";
  os << "min_rtt: " << s.min_rtt << "\n";
  os << "min_rtt_timestamp: " << s.min_rtt_timestamp << "\n";
  os << "congestion_window: " << s.congestion_window << "\n";
  os << "pacing_rate: " << s.pacing_rate << "\n";
  os << "last_sample_is_app_limited: " << s.last_sample_is_app_limited << "\n";

  if (s.mode == Bbr2Mode::STARTUP) {
    os << s.startup;
  }

  if (s.mode == Bbr2Mode::DRAIN) {
    os << s.drain;
  }

  if (s.mode == Bbr2Mode::PROBE_BW) {
    os << s.probe_bw;
  }

  if (s.mode == Bbr2Mode::PROBE_RTT) {
    os << s.probe_rtt;
  }

  return os;
}

}  // namespace quic
