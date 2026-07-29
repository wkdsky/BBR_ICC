#include "obbr_sender.h"
#include "unacked_packet_map.h"
#include "flag_impl.h"
#include "flag_util_impl.h"
#include "rtt_stats.h"
#include "random.h"
#include "proto_constants.h"
#include <ostream>

namespace dqc{
namespace {
/* Pace at ~1% below estimated bw, on average, to reduce queue at bottleneck.
 * In order to help drive the network toward lower queues and low latency while
 * maintaining high utilization, the average pacing rate aims to be slightly
 * lower than the estimated bandwidth. This is an important aspect of the
 * design.
 */
const int kPacingMarginPercent = 1;
// Constants based on TCP defaults.
// The minimum CWND to ensure delayed acks don't reduce bandwidth measurements.
// Does not inflate the pacing rate.
const QuicByteCount kDefaultMinimumCongestionWindow = 4 * kMaxSegmentSize;

// The gain used for the STARTUP, equal to 2/ln(2).
const float kDefaultHighGain = 2.885f;
// The gain used in STARTUP after loss has been detected.
// 1.5 is enough to allow for 25% exogenous loss and still observe a 25% growth
// in measured bandwidth.
const float kStartupAfterLossGain = 1.5f;
const float kDefaultDrainGain = 1.0f / kDefaultHighGain;
const float kObbrProbeBwCwndGain = 2.0f;
// The cycle of gains used during the PROBE_BW stage.
const float kPacingGain[] = {1.25, 0.75, 1, 1, 1, 1, 1, 1};
// The length of the gain cycle.
const size_t kGainCycleLength = sizeof(kPacingGain) / sizeof(kPacingGain[0]);
// The size of the bandwidth filter window, in round-trips.
const QuicRoundTripCount kBandwidthWindowSize = kGainCycleLength + 2;

// The time after which the current min_rtt value expires.
const TimeDelta kMinRttExpiry = TimeDelta::FromSeconds(10);
// The minimum time the connection can spend in PROBE_RTT mode.
const TimeDelta kProbeRttTime = TimeDelta::FromMilliseconds(200);
// If the bandwidth does not increase by the factor of |kStartupGrowthTarget|
// within |kRoundTripsWithoutGrowthBeforeExitingStartup| rounds, the connection
// will exit the STARTUP mode.
const float kStartupGrowthTarget = 1.25;//1.5; //do no figure out why I set it 1.5
const QuicRoundTripCount kRoundTripsWithoutGrowthBeforeExitingStartup = 3;
// Coefficient of target congestion window to use when basing PROBE_RTT on BDP.
const float kModerateProbeRttMultiplier = 0.75;
// Coefficient to determine if a new RTT is sufficiently similar to min_rtt that
// we don't need to enter PROBE_RTT.
const float kSimilarMinRttThreshold = 1.125;
const float kObbrU = 0.5f;
const float kObbrMaxCwndGain = 2.05f;
const float kObbrProbeUpMinGain = 1.25f;
const float kObbrBwDropThreshold = 0.75f;
const float kObbrQueueingRttThreshold = 2.5f;
const size_t kObbrSignalWindow = 30;
const TimeDelta kObbrLossWindow = TimeDelta::FromMilliseconds(3000);
const TimeDelta kObbrScoreWindow = TimeDelta::FromMilliseconds(200);

}  // namespace


ObbrSender::DebugState::DebugState(const ObbrSender& sender)
    : mode(sender.mode_),
      max_bandwidth(sender.max_bandwidth_.GetBest()),
      round_trip_count(sender.round_trip_count_),
      gain_cycle_index(sender.cycle_current_offset_),
      congestion_window(sender.congestion_window_),
      is_at_full_bandwidth(sender.is_at_full_bandwidth_),
      bandwidth_at_last_round(sender.bandwidth_at_last_round_),
      rounds_without_bandwidth_gain(sender.rounds_without_bandwidth_gain_),
      min_rtt(sender.min_rtt_),
      min_rtt_timestamp(sender.min_rtt_timestamp_),
      recovery_state(sender.recovery_state_),
      recovery_window(sender.recovery_window_),
      last_sample_is_app_limited(sender.last_sample_is_app_limited_),
      end_of_app_limited_phase(sender.sampler_.end_of_app_limited_phase()) {}

ObbrSender::DebugState::DebugState(const DebugState& state) = default;

ObbrSender::ObbrSender(ProtoTime now,
                     const RttStats* rtt_stats,
                     const UnackedPacketMap* unacked_packets,
                     QuicPacketCount initial_tcp_congestion_window,
                     QuicPacketCount max_tcp_congestion_window,
                     Random* random,bool drain_to_target)
    : rtt_stats_(rtt_stats),
      unacked_packets_(unacked_packets),
      random_(random),
      mode_(STARTUP),
      round_trip_count_(0),
      max_bandwidth_(kBandwidthWindowSize, QuicBandwidth::Zero(), 0),
      bandwidth_latest_(QuicBandwidth::Zero()),
      min_rtt_(TimeDelta::Zero()),
      min_rtt_timestamp_(ProtoTime::Zero()),
      congestion_window_(initial_tcp_congestion_window * kDefaultTCPMSS),
      initial_congestion_window_(initial_tcp_congestion_window *
                                 kDefaultTCPMSS),
      max_congestion_window_(max_tcp_congestion_window * kDefaultTCPMSS),
      min_congestion_window_(kDefaultMinimumCongestionWindow),
      high_gain_(kDefaultHighGain),
      high_cwnd_gain_(kDefaultHighGain),
      drain_gain_(kDefaultDrainGain),
      pacing_rate_(QuicBandwidth::Zero()),
      pacing_gain_(1),
      congestion_window_gain_(1),
      congestion_window_gain_constant_(kObbrProbeBwCwndGain),
      num_startup_rtts_(kRoundTripsWithoutGrowthBeforeExitingStartup),
      exit_startup_on_loss_(false),
      cycle_current_offset_(0),
      last_cycle_start_(ProtoTime::Zero()),
      is_at_full_bandwidth_(false),
      rounds_without_bandwidth_gain_(0),
      bandwidth_at_last_round_(QuicBandwidth::Zero()),
      exiting_quiescence_(false),
      exit_probe_rtt_at_(ProtoTime::Zero()),
      probe_rtt_round_passed_(false),
      last_sample_is_app_limited_(false),
      has_non_app_limited_sample_(false),
      flexible_app_limited_(false),
      recovery_state_(NOT_IN_RECOVERY),
      recovery_window_(max_congestion_window_),
      is_app_limited_recovery_(false),
      slower_startup_(false),
      rate_based_startup_(false),
      startup_rate_reduction_multiplier_(0),
      startup_bytes_lost_(0),
      drain_to_target_(drain_to_target),
      probe_rtt_based_on_bdp_(false),
      probe_rtt_skipped_if_similar_rtt_(false),
      probe_rtt_disabled_if_app_limited_(false),
      app_limited_since_last_probe_rtt_(false),
      min_rtt_since_last_probe_rtt_(TimeDelta::Infinite()),
      always_get_bw_sample_when_acked_(
          GetQuicReloadableFlag(quic_always_get_bw_sample_when_acked)),
      obbr_latest_raw_rtt_(TimeDelta::Zero()),
      obbr_has_latest_raw_rtt_(false),
      obbr_has_current_delivery_sample_(false),
      obbr_recent_probe_bw_samples_(),
      obbr_bw_down_count_(0),
      obbr_up_rtt_count_(0),
      obbr_cc_stage_(-1),
      obbr_reset_score_window_(false),
      obbr_score_time_(ProtoTime::Zero()),
      obbr_loss_timestamp_(ProtoTime::Zero()),
      obbr_last_revert_time_(ProtoTime::Zero()),
      obbr_saved_bandwidth_(QuicBandwidth::Zero()),
      obbr_score_sent_base_(0),
      obbr_score_delivered_base_(0),
      obbr_score1_(0),
      obbr_score2_(0),
      obbr_score3_(0),
      obbr_score4_(0) {
  /*if (stats_) {
    stats_->slowstart_count = 0;
    stats_->slowstart_start_time = QuicTime::Zero();
  }*/
  EnterStartupMode(now);
}

ObbrSender::~ObbrSender() {}
void ObbrSender::SetInitialCongestionWindowInPackets(
    QuicPacketCount congestion_window) {
  if (mode_ == STARTUP) {
    initial_congestion_window_ = congestion_window * kDefaultTCPMSS;
    congestion_window_ = congestion_window * kDefaultTCPMSS;
  }
}

bool ObbrSender::InSlowStart() const {
  return mode_ == STARTUP;
}

void ObbrSender::OnPacketSent(ProtoTime sent_time,
                             QuicByteCount bytes_in_flight,
                             QuicPacketNumber packet_number,
                             QuicByteCount bytes,
                             HasRetransmittableData is_retransmittable) {

  last_sent_packet_ = packet_number;

  if (bytes_in_flight == 0 && sampler_.is_app_limited()) {
    exiting_quiescence_ = true;
  }

  sampler_.OnPacketSent(sent_time, packet_number, bytes, bytes_in_flight,
                        is_retransmittable);
}

bool ObbrSender::CanSend(QuicByteCount bytes_in_flight) {
  return bytes_in_flight < GetCongestionWindow();
}

QuicBandwidth ObbrSender::PacingRate(QuicByteCount bytes_in_flight) const {
  if (pacing_rate_.IsZero()) {
    return high_gain_ * QuicBandwidth::FromBytesAndTimeDelta(
                            initial_congestion_window_, GetMinRtt());
  }
  return pacing_rate_;
}

QuicBandwidth ObbrSender::BandwidthEstimate() const {
  return max_bandwidth_.GetBest();
}

QuicByteCount ObbrSender::GetCongestionWindow() const {
  if (mode_ == PROBE_RTT) {
    return ProbeRttCongestionWindow();
  }

  if (InRecovery() && !(rate_based_startup_ && mode_ == STARTUP)) {
    return std::min(congestion_window_, recovery_window_);
  }

  return congestion_window_;
}

QuicByteCount ObbrSender::GetSlowStartThreshold() const {
  return 0;
}

bool ObbrSender::InRecovery() const {
  return recovery_state_ != NOT_IN_RECOVERY;
}

bool ObbrSender::ShouldSendProbingPacket() const {
  if (pacing_gain_ <= 1) {
    return false;
  }

  // TODO(b/77975811): If the pipe is highly under-utilized, consider not
  // sending a probing transmission, because the extra bandwidth is not needed.
  // If flexible_app_limited is enabled, check if the pipe is sufficiently full.
  if (flexible_app_limited_) {
    return !IsPipeSufficientlyFull();
  } else {
    return true;
  }
}

bool ObbrSender::IsPipeSufficientlyFull() const {
  // See if we need more bytes in flight to see more bandwidth.
  if (mode_ == STARTUP) {
    // STARTUP exits if it doesn't observe a 25% bandwidth increase, so the CWND
    // must be more than 25% above the target.
    return unacked_packets_->bytes_in_flight() >=
           GetTargetCongestionWindow(1.5);
  }
  if (pacing_gain_ > 1) {
    // Super-unity PROBE_BW doesn't exit until 1.25 * BDP is achieved.
    return unacked_packets_->bytes_in_flight() >=
           GetTargetCongestionWindow(pacing_gain_);
  }
  // If bytes_in_flight are above the target congestion window, it should be
  // possible to observe the same or more bandwidth if it's available.
  return unacked_packets_->bytes_in_flight() >= GetTargetCongestionWindow(1.1);
}

void ObbrSender::AdjustNetworkParameters(QuicBandwidth bandwidth,
                                        TimeDelta rtt,
                                        bool allow_cwnd_to_decrease) {
  if (!bandwidth.IsZero()) {
    max_bandwidth_.Update(bandwidth, round_trip_count_);
  }
  if (!rtt.IsZero() && (min_rtt_ > rtt || min_rtt_.IsZero())) {
    min_rtt_ = rtt;
  }
  if (GetQuicReloadableFlag(quic_fix_bbr_cwnd_in_bandwidth_resumption) &&
      mode_ == STARTUP) {
    if (bandwidth.IsZero()) {
      // Ignore bad bandwidth samples.
      QUIC_RELOADABLE_FLAG_COUNT_N(quic_fix_bbr_cwnd_in_bandwidth_resumption, 3,
                                   3);
      return;
    }
    const QuicByteCount new_cwnd =
        std::max(kMinInitialCongestionWindow * kDefaultTCPMSS,
                 std::min(kMaxInitialCongestionWindow * kDefaultTCPMSS,
                          bandwidth * rtt_stats_->SmoothedOrInitialRtt()));
    // Decreases cwnd gain and pacing gain. Please note, if pacing_rate_ has
    // been calculated, it cannot decrease in STARTUP phase.
    set_high_gain(kDefaultHighGain);
    set_high_cwnd_gain(kDefaultHighGain);
    if (new_cwnd > congestion_window_) {
      QUIC_RELOADABLE_FLAG_COUNT_N(quic_fix_bbr_cwnd_in_bandwidth_resumption, 1,
                                   3);
    } else {
      QUIC_RELOADABLE_FLAG_COUNT_N(quic_fix_bbr_cwnd_in_bandwidth_resumption, 2,
                                   3);
    }
    if (new_cwnd < congestion_window_ && !allow_cwnd_to_decrease) {
      // Only decrease cwnd if allow_cwnd_to_decrease is true.
      return;
    }
    congestion_window_ = new_cwnd;
  }
}

void ObbrSender::OnCongestionEvent(bool rtt_updated,
                                  QuicByteCount prior_in_flight,
                                  ProtoTime event_time,
                                  const AckedPacketVector& acked_packets,
                                  const LostPacketVector& lost_packets) {
  const QuicByteCount total_bytes_acked_before = sampler_.total_bytes_acked();

  // In nginx-quic, qc->latest_rtt is updated only when the ACK frame's
  // largest packet is newly acknowledged.  It remains the RTT attached to
  // subsequent valid delivery samples until the next such ACK arrives.
  if (rtt_updated && !rtt_stats_->latest_raw_rtt().IsZero()) {
    obbr_latest_raw_rtt_ = rtt_stats_->latest_raw_rtt();
    obbr_has_latest_raw_rtt_ = true;
  }
  obbr_has_current_delivery_sample_ = false;

  bool is_round_start = false;
  bool min_rtt_expired = false;

  DiscardLostPackets(lost_packets);

  // Input the new data into the BBR model of the connection.
  if (!acked_packets.empty()) {
    QuicPacketNumber last_acked_packet = acked_packets.rbegin()->packet_number;
    is_round_start = UpdateRoundTripCounter(last_acked_packet);
    min_rtt_expired = UpdateBandwidthAndMinRtt(event_time, acked_packets);
    UpdateRecoveryState(last_acked_packet, !lost_packets.empty(),
                        is_round_start);

  }

  // Handle logic specific to PROBE_BW mode.
  if (mode_ == PROBE_BW) {
    UpdateGainCyclePhase(event_time, prior_in_flight, !lost_packets.empty());
  }

  // Handle logic specific to STARTUP and DRAIN modes.
  if (is_round_start && !is_at_full_bandwidth_) {
    CheckIfFullBandwidthReached();
  }
  MaybeExitStartupOrDrain(event_time);

  // Handle logic specific to PROBE_RTT.
  MaybeEnterOrExitProbeRtt(event_time, is_round_start, min_rtt_expired);

  // Calculate number of packets acked and lost.
  QuicByteCount bytes_acked =
      sampler_.total_bytes_acked() - total_bytes_acked_before;
  QuicByteCount bytes_lost = 0;
  for (const auto& packet : lost_packets) {
    bytes_lost += packet.bytes_lost;
  }
  if (bytes_lost > 0) {
    obbr_loss_timestamp_ = event_time;
  }
  UpdateObbrState(event_time, bytes_acked, !lost_packets.empty());

  // After the model is updated, recalculate the pacing rate and congestion
  // window.
  CalculatePacingRate();
  CalculateCongestionWindow(bytes_acked);
  CalculateRecoveryWindow(bytes_acked, bytes_lost);

  // Cleanup internal state.
  sampler_.RemoveObsoletePackets(unacked_packets_->GetLeastUnacked());
}

CongestionControlType ObbrSender::GetCongestionControlType() const {
  return kOBBR;
}

TimeDelta ObbrSender::GetMinRtt() const {
  return !min_rtt_.IsZero() ? min_rtt_ : rtt_stats_->initial_rtt();
}

int32_t ObbrSender::GetCurrentBbrModeIndex() const {
  switch (mode_) {
    case STARTUP:
      return 0;
    case DRAIN:
      return 1;
    case PROBE_BW:
      return 3;
    case PROBE_RTT:
      return 6;
  }
  return 3;
}

QuicByteCount ObbrSender::GetTargetCongestionWindow(float gain) const {
  QuicByteCount bdp = GetMinRtt() * BandwidthEstimate();
  QuicByteCount congestion_window = gain * bdp;

  // BDP estimate will be zero if no bandwidth samples are available yet.
  if (congestion_window == 0) {
    congestion_window = gain * initial_congestion_window_;
  }

  return std::max(congestion_window, min_congestion_window_);
}

QuicByteCount ObbrSender::ProbeRttCongestionWindow() const {
  if (probe_rtt_based_on_bdp_) {
    return GetTargetCongestionWindow(kModerateProbeRttMultiplier);
  }
  return min_congestion_window_;
}

void ObbrSender::EnterStartupMode(ProtoTime now) {
  mode_ = STARTUP;
  pacing_gain_ = high_gain_;
  congestion_window_gain_ = high_cwnd_gain_;
}

void ObbrSender::EnterProbeBandwidthMode(ProtoTime now) {
  mode_ = PROBE_BW;
  congestion_window_gain_ = congestion_window_gain_constant_;

  // Pick a random offset for the gain cycle out of {0, 2..7} range. 1 is
  // excluded because in that case increased gain and decreased gain would not
  // follow each other.
  cycle_current_offset_ = random_->nextInt() % (kGainCycleLength - 1);
  if (cycle_current_offset_ >= 1) {
    cycle_current_offset_ += 1;
  }

  last_cycle_start_ = now;
  pacing_gain_ = kPacingGain[cycle_current_offset_];
}

void ObbrSender::DiscardLostPackets(const LostPacketVector& lost_packets) {
  for (const LostPacket& packet : lost_packets) {
    sampler_.OnPacketLost(packet.packet_number);
    if (mode_ == STARTUP) {
      /*if (stats_) {
        ++stats_->slowstart_packets_lost;
        stats_->slowstart_bytes_lost += packet.bytes_lost;
      }*/
      if (startup_rate_reduction_multiplier_ != 0) {
        startup_bytes_lost_ += packet.bytes_lost;
      }
    }
  }
}

bool ObbrSender::UpdateRoundTripCounter(QuicPacketNumber last_acked_packet) {
  if (!current_round_trip_end_.IsInitialized()||
      last_acked_packet > current_round_trip_end_) {
    round_trip_count_++;
    current_round_trip_end_ = last_sent_packet_;
    /*if (stats_ && InSlowStart()) {
      ++stats_->slowstart_num_rtts;
    }*/
    return true;
  }

  return false;
}

bool ObbrSender::UpdateBandwidthAndMinRtt(
    ProtoTime now,
    const AckedPacketVector& acked_packets) {
  DCHECK(!acked_packets.empty());
  for (const auto& packet : acked_packets) {
    if (!always_get_bw_sample_when_acked_ && packet.bytes_acked == 0) {
      // Skip acked packets with 0 in flight bytes when updating bandwidth.
      continue;
    }
    const QuicBandwidth reference_bandwidth = BandwidthEstimate();
    BandwidthSample bandwidth_sample =
        sampler_.OnPacketAcknowledged(now, packet.packet_number);
    if (always_get_bw_sample_when_acked_ &&
        !bandwidth_sample.state_at_send.is_valid) {
      // From the sampler's perspective, the packet has never been sent, or the
      // packet has been acked or marked as lost previously.
      continue;
    }

    last_sample_is_app_limited_ = bandwidth_sample.state_at_send.is_app_limited;
    has_non_app_limited_sample_ |=
        !bandwidth_sample.state_at_send.is_app_limited;
    if (!bandwidth_sample.bandwidth.IsZero()) {
      bandwidth_latest_ = bandwidth_sample.bandwidth;
    }
    if (!bandwidth_sample.rtt.IsZero()) {
      obbr_has_current_delivery_sample_ = true;
    }

    if (!bandwidth_sample.state_at_send.is_app_limited ||
        bandwidth_sample.bandwidth > BandwidthEstimate()) {
      max_bandwidth_.Update(bandwidth_sample.bandwidth, round_trip_count_);
    }

    if (mode_ == PROBE_BW &&
        !bandwidth_sample.state_at_send.is_app_limited &&
        !bandwidth_sample.bandwidth.IsZero()) {
      obbr_recent_probe_bw_samples_.push_back(bandwidth_sample.bandwidth);
      if (obbr_recent_probe_bw_samples_.size() > kObbrSignalWindow) {
        obbr_recent_probe_bw_samples_.pop_front();
      }

      if (!reference_bandwidth.IsZero() &&
          bandwidth_sample.bandwidth <
              reference_bandwidth * kObbrBwDropThreshold) {
        ++obbr_bw_down_count_;
      } else {
        obbr_bw_down_count_ = 0;
      }
    }
  }

  // ngx_generate_sample() supplies the previously recorded raw latest RTT to
  // BBR only when it has a valid delivery sample for this ACK event.
  if (!obbr_has_current_delivery_sample_ || !obbr_has_latest_raw_rtt_) {
    return false;
  }
  const TimeDelta raw_rtt = obbr_latest_raw_rtt_;
  min_rtt_since_last_probe_rtt_ =
      std::min(min_rtt_since_last_probe_rtt_, raw_rtt);

  // Do not expire min_rtt if none was ever available.
  bool min_rtt_expired =
      !min_rtt_.IsZero() && (now > (min_rtt_timestamp_ + kMinRttExpiry));

  if (min_rtt_expired || raw_rtt <= min_rtt_ || min_rtt_.IsZero()) {
    //QUIC_DVLOG(2) << "Min RTT updated, old value: " << min_rtt_
    //              << ", new value: " << raw_rtt
    //              << ", current time: " << now.ToDebuggingValue();

    if (min_rtt_expired && ShouldExtendMinRttExpiry()) {
      min_rtt_expired = false;
    } else {
      min_rtt_ = raw_rtt;
    }
    min_rtt_timestamp_ = now;
    // Reset since_last_probe_rtt fields.
    min_rtt_since_last_probe_rtt_ = TimeDelta::Infinite();
    app_limited_since_last_probe_rtt_ = false;
  }
  DCHECK(!min_rtt_.IsZero());

  return min_rtt_expired;
}

bool ObbrSender::ShouldExtendMinRttExpiry() const {
  if (probe_rtt_disabled_if_app_limited_ && app_limited_since_last_probe_rtt_) {
    // Extend the current min_rtt if we've been app limited recently.
    return true;
  }
  const bool min_rtt_increased_since_last_probe =
      min_rtt_since_last_probe_rtt_ > min_rtt_ * kSimilarMinRttThreshold;
  if (probe_rtt_skipped_if_similar_rtt_ && app_limited_since_last_probe_rtt_ &&
      !min_rtt_increased_since_last_probe) {
    // Extend the current min_rtt if we've been app limited recently and an rtt
    // has been measured in that time that's less than 12.5% more than the
    // current min_rtt.
    return true;
  }
  return false;
}

void ObbrSender::UpdateGainCyclePhase(ProtoTime now,
                                     QuicByteCount prior_in_flight,
                                     bool has_losses) {
  // In most cases, the cycle is advanced after an RTT passes.
  bool should_advance_gain_cycling = now - last_cycle_start_ > GetMinRtt();

  // If the pacing gain is above 1.0, the connection is trying to probe the
  // bandwidth by increasing the number of bytes in flight to at least
  // pacing_gain * BDP.  Make sure that it actually reaches the target, as long
  // as there are no losses suggesting that the buffers are not able to hold
  // that much.
  if (pacing_gain_ > 1.0 && !has_losses &&
      prior_in_flight < GetTargetCongestionWindow(pacing_gain_)) {
    should_advance_gain_cycling = false;
  }

  // If pacing gain is below 1.0, the connection is trying to drain the extra
  // queue which could have been incurred by probing prior to it.  nginx-quic
  // evaluates the ACK-preceding inflight amount against BDP.
  if (pacing_gain_ < 1.0 &&
      prior_in_flight <= GetTargetCongestionWindow(1)) {
    should_advance_gain_cycling = true;
  }

  if (should_advance_gain_cycling) {
    cycle_current_offset_ = (cycle_current_offset_ + 1) % kGainCycleLength;
    last_cycle_start_ = now;
    // Stay in low gain mode until the target BDP is hit.
    // Low gain mode will be exited immediately when the target BDP is achieved.
    if (drain_to_target_ && pacing_gain_ < 1 &&
        kPacingGain[cycle_current_offset_] == 1 &&
        prior_in_flight > GetTargetCongestionWindow(1)) {
      return;
    }
    pacing_gain_ = kPacingGain[cycle_current_offset_];
  }
}

void ObbrSender::CheckIfFullBandwidthReached() {
  if (last_sample_is_app_limited_) {
    return;
  }

  QuicBandwidth target = bandwidth_at_last_round_ * kStartupGrowthTarget;
  if (BandwidthEstimate() >= target) {
    bandwidth_at_last_round_ = BandwidthEstimate();
    rounds_without_bandwidth_gain_ = 0;
    return;
  }

  rounds_without_bandwidth_gain_++;
  if ((rounds_without_bandwidth_gain_ >= num_startup_rtts_) ||
      (exit_startup_on_loss_ && InRecovery())) {
    DCHECK(has_non_app_limited_sample_);
    is_at_full_bandwidth_ = true;
  }
}

void ObbrSender::MaybeExitStartupOrDrain(ProtoTime now) {
  if (mode_ == STARTUP && is_at_full_bandwidth_) {
    OnExitStartup(now);
    mode_ = DRAIN;
    pacing_gain_ = drain_gain_;
    congestion_window_gain_ = high_cwnd_gain_;
  }
  if (mode_ == DRAIN &&
      unacked_packets_->bytes_in_flight() <= GetTargetCongestionWindow(1)) {
    EnterProbeBandwidthMode(now);
  }
}

void ObbrSender::OnExitStartup(ProtoTime now) {
  DCHECK_EQ(mode_, STARTUP);
  /*if (stats_) {
    DCHECK_NE(stats_->slowstart_start_time, QuicTime::Zero());
    if (now > stats_->slowstart_start_time) {
      stats_->slowstart_duration =
          now - stats_->slowstart_start_time + stats_->slowstart_duration;
    }
    stats_->slowstart_start_time = QuicTime::Zero();
  }*/
}

void ObbrSender::MaybeEnterOrExitProbeRtt(ProtoTime now,
                                         bool is_round_start,
                                         bool min_rtt_expired) {
  if (min_rtt_expired && !exiting_quiescence_ && mode_ != PROBE_RTT) {
    if (InSlowStart()) {
      OnExitStartup(now);
    }
    mode_ = PROBE_RTT;
    pacing_gain_ = 1;
    // Do not decide on the time to exit PROBE_RTT until the |bytes_in_flight|
    // is at the target small value.
    exit_probe_rtt_at_ = ProtoTime::Zero();
  }

  if (mode_ == PROBE_RTT) {
    sampler_.OnAppLimited();

    if (exit_probe_rtt_at_ == ProtoTime::Zero()) {
      // If the window has reached the appropriate size, schedule exiting
      // PROBE_RTT.  The CWND during PROBE_RTT is kMinimumCongestionWindow, but
      // we allow an extra packet since QUIC checks CWND before sending a
      // packet.
      if (unacked_packets_->bytes_in_flight() <
          ProbeRttCongestionWindow() + kMaxOutgoingPacketSize) {
        // nginx-quic holds PROBE_RTT for min(2 * srtt, 200ms).  Its srtt is
        // ACK-delay-corrected, so use DQC's smoothed RTT for this timer only.
        const TimeDelta probe_rtt_duration = std::min(
            rtt_stats_->SmoothedOrInitialRtt() * 2, kProbeRttTime);
        exit_probe_rtt_at_ = now + probe_rtt_duration;
        probe_rtt_round_passed_ = false;
      }
    } else {
      if (is_round_start) {
        probe_rtt_round_passed_ = true;
      }
      if (now >= exit_probe_rtt_at_ && probe_rtt_round_passed_) {
        min_rtt_timestamp_ = now;
        if (!is_at_full_bandwidth_) {
          EnterStartupMode(now);
        } else {
          EnterProbeBandwidthMode(now);
        }
      }
    }
  }

  exiting_quiescence_ = false;
}

void ObbrSender::UpdateRecoveryState(QuicPacketNumber last_acked_packet,
                                    bool has_losses,
                                    bool is_round_start) {
  // Exit recovery when there are no losses for a round.
  if (has_losses) {
    end_recovery_at_ = last_sent_packet_;
  }

  switch (recovery_state_) {
    case NOT_IN_RECOVERY:
      // Enter conservation on the first loss.
      if (has_losses) {
        recovery_state_ = CONSERVATION;
        // This will cause the |recovery_window_| to be set to the correct
        // value in CalculateRecoveryWindow().
        recovery_window_ = 0;
        // Since the conservation phase is meant to be lasting for a whole
        // round, extend the current round as if it were started right now.
        current_round_trip_end_ = last_sent_packet_;
        if (GetQuicReloadableFlag(quic_bbr_app_limited_recovery) &&
            last_sample_is_app_limited_) {
          //QUIC_RELOADABLE_FLAG_COUNT(quic_bbr_app_limited_recovery);
          is_app_limited_recovery_ = true;
        }
      }
      break;

    case CONSERVATION:
      if (is_round_start) {
        recovery_state_ = GROWTH;
      }
      //QUIC_FALLTHROUGH_INTENDED;

    case GROWTH:
      // Exit recovery if appropriate.
      if (!has_losses && last_acked_packet > end_recovery_at_) {
        recovery_state_ = NOT_IN_RECOVERY;
        is_app_limited_recovery_ = false;
      }

      break;
  }
  if (recovery_state_ != NOT_IN_RECOVERY && is_app_limited_recovery_) {
    sampler_.OnAppLimited();
  }
}

void ObbrSender::ResetObbrScoreWindow(ProtoTime now) {
  obbr_score_sent_base_ = sampler_.total_bytes_sent();
  obbr_score_delivered_base_ = sampler_.total_bytes_acked();
  obbr_score_time_ = now;
  obbr_reset_score_window_ = false;
}

void ObbrSender::UpdateObbrState(ProtoTime now,
                                     QuicByteCount bytes_acked,
                                     bool has_losses) {
  // nginx-quic reaches ngx_bbr_update_cc_mode() only after a valid current
  // delivery sample has been generated.
  if (!obbr_has_current_delivery_sample_ || !obbr_has_latest_raw_rtt_) {
    return;
  }

  if (mode_ == STARTUP) {
    const QuicByteCount total_acked = sampler_.total_bytes_acked();
    if (total_acked == 0 ||
        sampler_.total_bytes_lost() * 100 < total_acked * 10) {
      return;
    }
  }

  const TimeDelta min_rtt = GetMinRtt();
  const TimeDelta raw_rtt = obbr_latest_raw_rtt_;

  if (!min_rtt.IsZero() && raw_rtt > min_rtt * kObbrQueueingRttThreshold) {
    ++obbr_up_rtt_count_;
  } else {
    obbr_up_rtt_count_ = 0;
  }

  if (has_losses && !min_rtt.IsZero()) {
    const TimeDelta queueing_rtt = raw_rtt - min_rtt;
    float gain = 1.0f;
    if (queueing_rtt > TimeDelta::Zero()) {
      gain += kObbrU * static_cast<float>(queueing_rtt.ToMicroseconds()) /
              static_cast<float>(min_rtt.ToMicroseconds());
    }
    congestion_window_gain_ = std::min(gain, kObbrMaxCwndGain);
    obbr_loss_timestamp_ = now;
  } else if (!InRecovery()) {
    const QuicByteCount base_target = GetTargetCongestionWindow(1.0f);
    if (base_target > 0) {
      const double step =
          (0.01 * static_cast<double>(bytes_acked)) /
          static_cast<double>(base_target);
      congestion_window_gain_ = std::min<float>(
          kObbrMaxCwndGain, congestion_window_gain_ + step);
    }
  }

  if (has_losses &&
      (obbr_bw_down_count_ >= kObbrSignalWindow ||
       obbr_up_rtt_count_ > kObbrSignalWindow) &&
      obbr_cc_stage_ == -1) {
    obbr_cc_stage_ = 0;
    obbr_reset_score_window_ = true;
  }

  if (mode_ == PROBE_RTT) {
    obbr_reset_score_window_ = true;
  }

  if (obbr_reset_score_window_) {
    ResetObbrScoreWindow(now);
  }

  if (obbr_cc_stage_ == -1 || !obbr_score_time_.IsInitialized() ||
      now - obbr_score_time_ <= kObbrScoreWindow) {
    return;
  }

  const QuicByteCount sent = sampler_.total_bytes_sent() - obbr_score_sent_base_;
  const QuicByteCount delivered =
      sampler_.total_bytes_acked() - obbr_score_delivered_base_;
  const QuicByteCount unacked = sent > delivered ? sent - delivered : 0;
  const int64_t score =
      static_cast<int64_t>(delivered) - 10 * static_cast<int64_t>(unacked);

  if (obbr_cc_stage_ < 2) {
    obbr_score1_ = obbr_score2_;
    obbr_score2_ = score;
    if (obbr_cc_stage_ == 1 && !obbr_recent_probe_bw_samples_.empty()) {
      int64_t bandwidth_sum_bps = 0;
      for (const auto& sample : obbr_recent_probe_bw_samples_) {
        bandwidth_sum_bps += sample.ToBitsPerSecond();
      }
      const int64_t avg_bps =
          bandwidth_sum_bps / static_cast<int64_t>(obbr_recent_probe_bw_samples_.size());
      if (avg_bps > 0) {
        obbr_saved_bandwidth_ = BandwidthEstimate();
        max_bandwidth_.Reset(QuicBandwidth::FromBitsPerSecond(avg_bps),
                             round_trip_count_);
      }
      obbr_up_rtt_count_ = 0;
      obbr_bw_down_count_ = 0;
    }
  } else {
    obbr_score3_ = obbr_score4_;
    obbr_score4_ = score;
    if (obbr_cc_stage_ == 3) {
      const bool can_revert =
          !obbr_saved_bandwidth_.IsZero() &&
          (!obbr_last_revert_time_.IsInitialized() ||
           now - obbr_last_revert_time_ > GetMinRtt() * 10);
      if (can_revert &&
          (obbr_score3_ + obbr_score4_) < (obbr_score1_ + obbr_score2_)) {
        // ngx_win_filter_max() reinserts the saved value into the BBR max
        // filter; it does not discard a better bandwidth sample already in
        // the window.
        max_bandwidth_.Update(obbr_saved_bandwidth_, round_trip_count_);
        obbr_last_revert_time_ = now;
      }
      obbr_cc_stage_ = -1;
    }
  }

  if (obbr_cc_stage_ != -1) {
    ++obbr_cc_stage_;
  }
  ResetObbrScoreWindow(now);
}

void ObbrSender::CalculatePacingRate() {
  if (BandwidthEstimate().IsZero()) {
    return;
  }

  QuicBandwidth target_rate = pacing_gain_ * BandwidthEstimate();
  if (is_at_full_bandwidth_) {
    pacing_rate_ = target_rate;
    return;
  }

  // Pace at the rate of initial_window / RTT as soon as RTT measurements are
  // available.
  if (pacing_rate_.IsZero() && !rtt_stats_->min_rtt().IsZero()) {
    pacing_rate_ = QuicBandwidth::FromBytesAndTimeDelta(
        initial_congestion_window_, rtt_stats_->min_rtt());
    return;
  }
  // Slow the pacing rate in STARTUP once loss has ever been detected.
  const bool has_ever_detected_loss = end_recovery_at_.IsInitialized();
  if (slower_startup_ && has_ever_detected_loss &&
      has_non_app_limited_sample_) {
    pacing_rate_ = kStartupAfterLossGain * BandwidthEstimate();
    return;
  }

  // Slow the pacing rate in STARTUP by the bytes_lost / CWND.
  if (startup_rate_reduction_multiplier_ != 0 && has_ever_detected_loss &&
      has_non_app_limited_sample_) {
    pacing_rate_ =
        (1 - (startup_bytes_lost_ * startup_rate_reduction_multiplier_ * 1.0f /
              congestion_window_)) *
        target_rate;
    // Ensure the pacing rate doesn't drop below the startup growth target times
    // the bandwidth estimate.
    pacing_rate_ =
        std::max(pacing_rate_, kStartupGrowthTarget * BandwidthEstimate());
    return;
  }

  // Do not decrease the pacing rate during startup.
  pacing_rate_ = std::max(pacing_rate_, target_rate);
}

void ObbrSender::CalculateCongestionWindow(QuicByteCount bytes_acked) {
  if (mode_ == PROBE_RTT) {
    return;
  }

  float cwnd_gain = congestion_window_gain_;
  if (pacing_gain_ > 1.0f) {
    cwnd_gain = std::max(kObbrProbeUpMinGain, cwnd_gain);
    const ProtoTime now = unacked_packets_->GetLastPacketSentTime();
    if (!now.IsInitialized() ||
        !obbr_loss_timestamp_.IsInitialized() ||
        now - obbr_loss_timestamp_ > kObbrLossWindow) {
      cwnd_gain = std::max(cwnd_gain, kObbrMaxCwndGain);
    }
  }

  QuicByteCount target_window = GetTargetCongestionWindow(cwnd_gain);
  target_window += 3 * GetQuantum();
  // ngx_bbr_extra_ack_gain is zero in the oBBR reference implementation, so
  // ACK aggregation must not inflate its target congestion window.

  // Instead of immediately setting the target CWND as the new one, BBR grows
  // the CWND towards |target_window| by only increasing it |bytes_acked| at a
  // time.
  const bool add_bytes_acked =
      !GetQuicReloadableFlag(quic_bbr_no_bytes_acked_in_startup_recovery) ||
      !InRecovery();
  if (is_at_full_bandwidth_) {
    congestion_window_ =
        std::min(target_window, congestion_window_ + bytes_acked);
  } else if (add_bytes_acked &&
             (congestion_window_ < target_window ||
              sampler_.total_bytes_acked() < initial_congestion_window_)) {
    // If the connection is not yet out of startup phase, do not decrease the
    // window.
    congestion_window_ = congestion_window_ + bytes_acked;
  }

  // Enforce the limits on the congestion window.
  if (congestion_window_gain_ < kObbrMaxCwndGain) {
    congestion_window_ =
        std::min(congestion_window_, GetTargetCongestionWindow(cwnd_gain));
  }
  congestion_window_ = std::max(congestion_window_, min_congestion_window_);
  congestion_window_ = std::min(congestion_window_, max_congestion_window_);
}

QuicByteCount ObbrSender::GetQuantum() const {
  if (pacing_rate_.IsZero()) {
    return kDefaultTCPMSS;
  }
  const int64_t bytes_per_second = pacing_rate_.ToBytesPerSecond();
  if (bytes_per_second < static_cast<int64_t>(1.2 * 1024 * 1024 / 8)) {
    return kDefaultTCPMSS;
  }
  if (bytes_per_second < static_cast<int64_t>(24 * 1024 * 1024 / 8)) {
    return 2 * kDefaultTCPMSS;
  }
  return std::min<QuicByteCount>(bytes_per_second / 1000, 64 * 1024);
}

void ObbrSender::CalculateRecoveryWindow(QuicByteCount bytes_acked,
                                        QuicByteCount bytes_lost) {
  if (rate_based_startup_ && mode_ == STARTUP) {
    return;
  }

  if (recovery_state_ == NOT_IN_RECOVERY) {
    return;
  }

  // Set up the initial recovery window.
  if (recovery_window_ == 0) {
    recovery_window_ = unacked_packets_->bytes_in_flight() + bytes_acked;
    recovery_window_ = std::max(min_congestion_window_, recovery_window_);
    return;
  }

  // Remove losses from the recovery window, while accounting for a potential
  // integer underflow.
  recovery_window_ = recovery_window_ >= bytes_lost
                         ? recovery_window_ - bytes_lost
                         : kMaxSegmentSize;

  // In CONSERVATION mode, just subtracting losses is sufficient.  In GROWTH,
  // release additional |bytes_acked| to achieve a slow-start-like behavior.
  if (recovery_state_ == GROWTH) {
    recovery_window_ += bytes_acked;
  }

  // Sanity checks.  Ensure that we always allow to send at least an MSS or
  // |bytes_acked| in response, whichever is larger.
  recovery_window_ = std::max(
      recovery_window_, unacked_packets_->bytes_in_flight() + bytes_acked);
  if (GetQuicReloadableFlag(quic_bbr_one_mss_conservation)) {
    recovery_window_ =
        std::max(recovery_window_,
                 unacked_packets_->bytes_in_flight() + kMaxSegmentSize);
  }
  recovery_window_ = std::max(min_congestion_window_, recovery_window_);
}

std::string ObbrSender::GetDebugState() const {
  std::ostringstream stream;
  stream << ExportDebugState()
         << "\noBBR raw latest RTT (us): "
         << (obbr_has_latest_raw_rtt_
                 ? obbr_latest_raw_rtt_.ToMicroseconds()
                 : 0)
         << "\noBBR current delivery sample: "
         << (obbr_has_current_delivery_sample_ ? "yes" : "no");
  return stream.str();
}

void ObbrSender::OnApplicationLimited(QuicByteCount bytes_in_flight) {
  if (bytes_in_flight >= GetCongestionWindow()) {
    return;
  }
  if (flexible_app_limited_ && IsPipeSufficientlyFull()) {
    return;
  }

  app_limited_since_last_probe_rtt_ = true;
  sampler_.OnAppLimited();
  DLOG(INFO) << "Becoming application limited. Last sent packet: "
                << last_sent_packet_ << ", CWND: " << GetCongestionWindow();
}

ObbrSender::DebugState ObbrSender::ExportDebugState() const {
  return DebugState(*this);
}

static std::string ModeToString(ObbrSender::Mode mode) {
  switch (mode) {
    case ObbrSender::STARTUP:
      return "STARTUP";
    case ObbrSender::DRAIN:
      return "DRAIN";
    case ObbrSender::PROBE_BW:
      return "PROBE_BW";
    case ObbrSender::PROBE_RTT:
      return "PROBE_RTT";
  }
  return "???";
}

std::ostream& operator<<(std::ostream& os, const ObbrSender::Mode& mode) {
  os << ModeToString(mode);
  return os;
}

std::ostream& operator<<(std::ostream& os, const ObbrSender::DebugState& state) {
  os << "Mode: " << ModeToString(state.mode) << std::endl;
  os << "Maximum bandwidth: " << state.max_bandwidth << std::endl;
  os << "Round trip counter: " << state.round_trip_count << std::endl;
  os << "Gain cycle index: " << static_cast<int>(state.gain_cycle_index)
     << std::endl;
  os << "Congestion window: " << state.congestion_window << " bytes"
     << std::endl;

  if (state.mode == ObbrSender::STARTUP) {
    os << "(startup) Bandwidth at last round: " << state.bandwidth_at_last_round
       << std::endl;
    os << "(startup) Rounds without gain: "
       << state.rounds_without_bandwidth_gain << std::endl;
  }

  os << "Minimum RTT: " << state.min_rtt << std::endl;
  os << "Minimum RTT timestamp: " << state.min_rtt_timestamp.ToDebuggingValue()
     << std::endl;

  os << "Last sample is app-limited: "
     << (state.last_sample_is_app_limited ? "yes" : "no");

  return os;
}

}


