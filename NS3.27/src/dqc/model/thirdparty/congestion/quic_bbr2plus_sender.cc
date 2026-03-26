#include "quic_bbr2plus_sender.h"

#include <algorithm>
#include <limits>

#include "quic_bbr2_probe_bw.h"
#include "quic_logging.h"

namespace dqc {

namespace {

constexpr float kDefaultRttCompStartupGain = 2.885f;
constexpr float kDefaultRttCompGain = 2.0f;
constexpr float kDefaultRttCompRttVarThresh = 0.4f;
constexpr uint32_t kDefaultRcMinRttWindowRounds = 4;
constexpr uint32_t kDefaultRttCompJitterWindowRounds = 4;
constexpr float kDefaultFastConvRttThresh = 1.1f;
constexpr float kDefaultFastConvPreUpThresh = 0.02f;
constexpr int64_t kDefaultFastConvRttErrorUs = 2000;
constexpr uint32_t kDefaultFastConvRoundsToAdvanceBwFilter = 25;
constexpr float kDefaultPreUpGain = 1.10f;
constexpr float kDefaultDownSlightlyGain = 0.90f;

TimeDelta InfiniteDelta() {
  return TimeDelta::Infinite();
}

TimeDelta ZeroOr(const TimeDelta& value, const TimeDelta& fallback) {
  return value.IsZero() || value.IsInfinite() ? fallback : value;
}

}  // namespace

Bbr2PlusSender::Bbr2PlusSender(QuicTime now,
                               const RttStats* rtt_stats,
                               const QuicUnackedPacketMap* unacked_packets,
                               QuicPacketCount initial_cwnd_in_packets,
                               QuicPacketCount max_cwnd_in_packets,
                               Random* random,
                               QuicConnectionStats* stats,
                               bool enable_ecn)
    : Bbr2Sender(now,
                 rtt_stats,
                 unacked_packets,
                 initial_cwnd_in_packets,
                 max_cwnd_in_packets,
                 random,
                 stats,
                 enable_ecn),
      enable_ecn_(enable_ecn),
      rc_min_rtt_filter_(kDefaultRcMinRttWindowRounds, InfiniteDelta(), 0),
      max_jitter_filter_(kDefaultRttCompJitterWindowRounds, TimeDelta::Zero(), 0),
      last_round_count_(model_.RoundTripCount()),
      ever_measured_min_rtt_(InfiniteDelta()),
      last_round_srtt_(TimeDelta::Zero()),
      curr_round_srtt_(TimeDelta::Zero()),
      last_round_min_rtt_(InfiniteDelta()),
      curr_round_min_rtt_(InfiniteDelta()),
      min_rtt_in_cruise_(InfiniteDelta()),
      last_min_rtt_in_cruise_(InfiniteDelta()),
      min_rtt_before_probe_(InfiniteDelta()),
      current_time_(now),
      probe_phase_start_time_(QuicTime::Zero()),
      probe_extension_state_(ProbeExtensionState::kInactive),
      rounds_since_last_bw_advance_(0),
      rc_min_rtt_win_rounds_(kDefaultRcMinRttWindowRounds),
      rtt_comp_startup_gain_(kDefaultRttCompStartupGain),
      rtt_comp_gain_(kDefaultRttCompGain),
      rtt_comp_rttvar_thresh_(kDefaultRttCompRttVarThresh),
      rtt_comp_jitter_win_rounds_(kDefaultRttCompJitterWindowRounds),
      fast_conv_rtt_thresh_(kDefaultFastConvRttThresh),
      fast_conv_preup_thresh_(kDefaultFastConvPreUpThresh),
      fast_conv_rtt_error_(TimeDelta::FromMicroseconds(kDefaultFastConvRttErrorUs)),
      fast_conv_rounds_to_advance_bw_filter_(
          kDefaultFastConvRoundsToAdvanceBwFilter),
      pre_up_pacing_gain_(kDefaultPreUpGain),
      down_slightly_pacing_gain_(kDefaultDownSlightlyGain),
      rtt_compensation_enabled_(true),
      fast_convergence_enabled_(true),
      copa_style_(false) {
  QUIC_DVLOG(2) << this << " Initializing Bbr2PlusSender @ " << now;
}

void Bbr2PlusSender::OnPacketSent(QuicTime sent_time,
                                  QuicByteCount bytes_in_flight,
                                  QuicPacketNumber packet_number,
                                  QuicByteCount bytes,
                                  HasRetransmittableData is_retransmittable) {
  current_time_ = sent_time;
  Bbr2Sender::OnPacketSent(sent_time, bytes_in_flight, packet_number, bytes,
                           is_retransmittable);
}

void Bbr2PlusSender::OnCongestionEvent(bool rtt_updated,
                                       QuicByteCount prior_in_flight,
                                       QuicTime event_time,
                                       const AckedPacketVector& acked_packets,
                                       const LostPacketVector& lost_packets) {
  current_time_ = event_time;
  const QuicRoundTripCount previous_round = model_.RoundTripCount();
  const Bbr2ProbeBwMode::CyclePhase previous_phase = GetCurrentProbeBwPhase();

  UpdateLatestRttSample();
  Bbr2Sender::OnCongestionEvent(rtt_updated, prior_in_flight, event_time,
                                acked_packets, lost_packets);
  AdvanceRoundIfNeeded(previous_round);

  const Bbr2ProbeBwMode::CyclePhase current_phase = GetCurrentProbeBwPhase();
  OnProbePhaseChange(previous_phase, current_phase, event_time);
}

QuicBandwidth Bbr2PlusSender::PacingRate(QuicByteCount bytes_in_flight) const {
  const QuicBandwidth base_rate = Bbr2Sender::PacingRate(bytes_in_flight);
  if (mode_ != Bbr2Mode::PROBE_BW) {
    return base_rate;
  }

  if (GetCurrentProbeBwPhase() != Bbr2ProbeBwMode::CyclePhase::PROBE_UP) {
    return base_rate;
  }

  switch (probe_extension_state_) {
    case ProbeExtensionState::kPreUp:
      return RateFromGain(pre_up_pacing_gain_);
    case ProbeExtensionState::kGuard:
      return RateFromGain(1.0f);
    case ProbeExtensionState::kDownSlightly:
      return RateFromGain(down_slightly_pacing_gain_);
    case ProbeExtensionState::kPostUp:
      return RateFromGain(1.0f);
    case ProbeExtensionState::kInactive:
    default:
      return base_rate;
  }
}

QuicByteCount Bbr2PlusSender::GetCongestionWindow() const {
  return Bbr2Sender::GetCongestionWindow() + GetRttCompensationBytes();
}

int32_t Bbr2PlusSender::GetCurrentBbrModeIndex() const {
  if (mode_ == Bbr2Mode::PROBE_BW &&
      GetCurrentProbeBwPhase() == Bbr2ProbeBwMode::CyclePhase::PROBE_UP) {
    switch (probe_extension_state_) {
      case ProbeExtensionState::kPreUp:
        return 7;
      case ProbeExtensionState::kGuard:
        return 8;
      case ProbeExtensionState::kPostUp:
        return 9;
      case ProbeExtensionState::kDownSlightly:
        return 10;
      case ProbeExtensionState::kInactive:
      default:
        break;
    }
  }
  return Bbr2Sender::GetCurrentBbrModeIndex();
}

Bbr2ProbeBwMode::CyclePhase Bbr2PlusSender::GetCurrentProbeBwPhase() const {
  DebugState state = ExportDebugState();
  if (state.mode == Bbr2Mode::PROBE_BW) {
    return state.probe_bw.phase;
  }
  return Bbr2ProbeBwMode::CyclePhase::PROBE_NOT_STARTED;
}

void Bbr2PlusSender::UpdateLatestRttSample() {
  if (rtt_stats_ == nullptr) {
    return;
  }
  const TimeDelta latest_rtt = rtt_stats_->latest_rtt();
  if (latest_rtt.IsZero() || latest_rtt.IsInfinite()) {
    return;
  }

  if (ever_measured_min_rtt_.IsInfinite() || latest_rtt < ever_measured_min_rtt_) {
    ever_measured_min_rtt_ = latest_rtt;
  }

  if (curr_round_min_rtt_.IsInfinite() || latest_rtt < curr_round_min_rtt_) {
    curr_round_min_rtt_ = latest_rtt;
  }

  const TimeDelta smoothed_rtt = ZeroOr(rtt_stats_->smoothed_rtt(), latest_rtt);
  if (curr_round_srtt_.IsZero() || curr_round_srtt_.IsInfinite()) {
    curr_round_srtt_ = smoothed_rtt;
  } else {
    curr_round_srtt_ = TimeDelta::FromMicroseconds(
        (7 * curr_round_srtt_.ToMicroseconds() + smoothed_rtt.ToMicroseconds()) /
        8);
  }

  if (mode_ == Bbr2Mode::PROBE_BW &&
      GetCurrentProbeBwPhase() == Bbr2ProbeBwMode::CyclePhase::PROBE_CRUISE) {
    if (min_rtt_in_cruise_.IsInfinite() || latest_rtt < min_rtt_in_cruise_) {
      min_rtt_in_cruise_ = latest_rtt;
    }
  }
}

void Bbr2PlusSender::AdvanceRoundIfNeeded(QuicRoundTripCount previous_round) {
  const QuicRoundTripCount current_round = model_.RoundTripCount();
  if (current_round == previous_round) {
    return;
  }
  last_round_count_ = current_round;
  OnRoundStart();
}

void Bbr2PlusSender::OnRoundStart() {
  ++rounds_since_last_bw_advance_;

  if (!curr_round_min_rtt_.IsInfinite()) {
    rc_min_rtt_filter_.SetWindowLength(rc_min_rtt_win_rounds_);
    rc_min_rtt_filter_.Update(curr_round_min_rtt_, last_round_count_);
    last_round_min_rtt_ = curr_round_min_rtt_;
  }
  if (!curr_round_srtt_.IsZero()) {
    last_round_srtt_ = curr_round_srtt_;
  }

  if (rtt_stats_ != nullptr && !rtt_stats_->mean_deviation().IsZero()) {
    max_jitter_filter_.SetWindowLength(rtt_comp_jitter_win_rounds_);
    max_jitter_filter_.Update(rtt_stats_->mean_deviation(), last_round_count_);
  }

  if (mode_ == Bbr2Mode::PROBE_BW &&
      GetCurrentProbeBwPhase() == Bbr2ProbeBwMode::CyclePhase::PROBE_CRUISE &&
      !min_rtt_in_cruise_.IsInfinite()) {
    last_min_rtt_in_cruise_ = min_rtt_in_cruise_;
    min_rtt_in_cruise_ = InfiniteDelta();
  }

  curr_round_min_rtt_ = InfiniteDelta();
  curr_round_srtt_ = TimeDelta::Zero();
  UpdateLatestRttSample();

  if (fast_convergence_enabled_ && ShouldAdvanceBandwidthFilter()) {
    model_.AdvanceMaxBandwidthFilter();
    rounds_since_last_bw_advance_ = 0;
  }

  if (GetCurrentProbeBwPhase() == Bbr2ProbeBwMode::CyclePhase::PROBE_UP) {
    switch (probe_extension_state_) {
      case ProbeExtensionState::kPreUp:
        probe_extension_state_ = ProbeExtensionState::kGuard;
        break;
      case ProbeExtensionState::kGuard:
        probe_extension_state_ = ShouldEnterAggressiveProbe()
                                     ? ProbeExtensionState::kInactive
                                     : ProbeExtensionState::kDownSlightly;
        break;
      case ProbeExtensionState::kDownSlightly:
        probe_extension_state_ = ProbeExtensionState::kPostUp;
        break;
      case ProbeExtensionState::kPostUp:
        probe_extension_state_ = ProbeExtensionState::kInactive;
        break;
      case ProbeExtensionState::kInactive:
      default:
        break;
    }
  }
}

void Bbr2PlusSender::OnProbePhaseChange(
    Bbr2ProbeBwMode::CyclePhase previous_phase,
    Bbr2ProbeBwMode::CyclePhase current_phase,
    QuicTime event_time) {
  if (mode_ != Bbr2Mode::PROBE_BW) {
    ResetProbeExtensionState();
    return;
  }

  if (current_phase == Bbr2ProbeBwMode::CyclePhase::PROBE_CRUISE &&
      previous_phase != current_phase) {
    min_rtt_in_cruise_ = InfiniteDelta();
  }

  if (current_phase == Bbr2ProbeBwMode::CyclePhase::PROBE_UP &&
      previous_phase != current_phase) {
    min_rtt_before_probe_ = ZeroOr(last_round_min_rtt_, model_.MinRtt());
    probe_phase_start_time_ = event_time;
    probe_extension_state_ = fast_convergence_enabled_
                                 ? ProbeExtensionState::kPreUp
                                 : ProbeExtensionState::kInactive;
    return;
  }

  if (previous_phase == Bbr2ProbeBwMode::CyclePhase::PROBE_UP &&
      current_phase != previous_phase) {
    ResetProbeExtensionState();
  }
}

bool Bbr2PlusSender::ShouldAdvanceBandwidthFilter() const {
  if (!fast_convergence_enabled_) {
    return false;
  }
  if (mode_ != Bbr2Mode::PROBE_BW ||
      GetCurrentProbeBwPhase() != Bbr2ProbeBwMode::CyclePhase::PROBE_CRUISE) {
    return false;
  }

  if (rounds_since_last_bw_advance_ >=
      static_cast<int>(fast_conv_rounds_to_advance_bw_filter_)) {
    return true;
  }

  if (curr_round_min_rtt_.IsInfinite()) {
    return false;
  }
  const TimeDelta reference =
      rc_min_rtt_filter_.GetBest().IsInfinite() ? model_.MinRtt()
                                                : rc_min_rtt_filter_.GetBest();
  if (reference.IsZero() || reference.IsInfinite()) {
    return false;
  }
  return curr_round_min_rtt_ > reference * fast_conv_rtt_thresh_;
}

bool Bbr2PlusSender::ShouldEnterAggressiveProbe() const {
  const TimeDelta baseline = copa_style_
                                 ? ZeroOr(last_round_min_rtt_, model_.MinRtt())
                                 : ZeroOr(last_round_srtt_, model_.MinRtt());
  const TimeDelta candidate = copa_style_
                                  ? ZeroOr(curr_round_min_rtt_, model_.MinRtt())
                                  : ZeroOr(curr_round_srtt_, model_.MinRtt());
  if (baseline.IsZero() || baseline.IsInfinite() || candidate.IsZero() ||
      candidate.IsInfinite()) {
    return true;
  }
  TimeDelta allowance = baseline * fast_conv_preup_thresh_;
  if (allowance > fast_conv_rtt_error_) {
    allowance = fast_conv_rtt_error_;
  }
  return candidate <= baseline + allowance;
}

QuicBandwidth Bbr2PlusSender::RateFromGain(float gain) const {
  if (BandwidthEstimate().IsZero()) {
    return Bbr2Sender::PacingRate(0);
  }
  return gain * BandwidthEstimate();
}

QuicByteCount Bbr2PlusSender::GetRttCompensationBytes() const {
  if (!rtt_compensation_enabled_ || mode_ == Bbr2Mode::PROBE_RTT ||
      rtt_stats_ == nullptr) {
    return 0;
  }

  const TimeDelta reference_rtt = model_.MinRtt();
  const TimeDelta jitter = max_jitter_filter_.GetBest();
  if (reference_rtt.IsZero() || reference_rtt.IsInfinite() || jitter.IsZero()) {
    return 0;
  }

  const TimeDelta threshold = reference_rtt * rtt_comp_rttvar_thresh_;
  if (jitter <= threshold) {
    return 0;
  }

  const float gain =
      mode_ == Bbr2Mode::STARTUP ? rtt_comp_startup_gain_ : rtt_comp_gain_;
  const QuicBandwidth bw = model_.MaxBandwidth().IsZero() ? BandwidthEstimate()
                                                          : model_.MaxBandwidth();
  if (bw.IsZero()) {
    return 0;
  }
  return bw * (jitter * gain);
}

void Bbr2PlusSender::ResetProbeExtensionState() {
  probe_extension_state_ = ProbeExtensionState::kInactive;
  probe_phase_start_time_ = QuicTime::Zero();
}

}  // namespace dqc
