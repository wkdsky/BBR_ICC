#include "quic_bbr2plus_sender.h"

#include <algorithm>

#include "quic_bbr2_probe_bw.h"
#include "quic_logging.h"

namespace dqc {

namespace {

constexpr float kDefaultRttCompStartupGain = 2.885f;
constexpr float kDefaultRttCompGain = 2.0f;
constexpr float kDefaultRttCompRttVarThresh = 0.50f;
constexpr uint32_t kDefaultRttCompJitterWindowRounds = 4;
constexpr float kDefaultFastConvRttThresh = 1.10f;
constexpr float kDefaultFastConvProbeRttGrowthThresh = 1.02f;
constexpr uint32_t kDefaultFastConvProbeCycleBase = 8;
constexpr uint32_t kDefaultFastConvProbeCycleRandom = 4;
constexpr uint32_t kDefaultFastConvRoundsToAdvanceBwFilter = 25;
constexpr float kDefaultSwitchToBbr2RttMultiplier = 1.10f;
constexpr float kDefaultSwitchToBbr2PlusRttMultiplier = 1.05f;
constexpr uint32_t kDefaultSwitchToBbr2CruiseCount = 2;
constexpr uint32_t kDefaultSwitchToBbr2PlusCruiseCount = 4;
constexpr float kDefaultPreUpGain = 1.10f;
constexpr float kDefaultDownSlightlyGain = 0.90f;

TimeDelta InfiniteDelta() {
  return TimeDelta::Infinite();
}

TimeDelta ZeroOr(const TimeDelta& value, const TimeDelta& fallback) {
  return value.IsZero() || value.IsInfinite() ? fallback : value;
}

float AtLeastOneOr(float value, float fallback) {
  return value >= 1.0f ? value : fallback;
}

float PositiveOr(float value, float fallback) {
  return value > 0.0f ? value : fallback;
}

float UnitIntervalOr(float value, float fallback) {
  return value >= 0.0f && value <= 1.0f ? value : fallback;
}

float NonNegativeOr(float value, float fallback) {
  return value >= 0.0f ? value : fallback;
}

uint32_t AtLeastOneOr(uint32_t value, uint32_t fallback) {
  return value == 0 ? fallback : value;
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
      max_jitter_filter_(kDefaultRttCompJitterWindowRounds,
                         TimeDelta::Zero(),
                         0),
      prior_round_srtt_(TimeDelta::Zero()),
      last_round_srtt_(TimeDelta::Zero()),
      curr_round_srtt_(TimeDelta::Zero()),
      prior_round_min_rtt_(InfiniteDelta()),
      last_round_min_rtt_(InfiniteDelta()),
      curr_round_min_rtt_(InfiniteDelta()),
      min_rtt_before_probe_(InfiniteDelta()),
      probe_up_min_rtt_(InfiniteDelta()),
      current_cruise_min_rtt_(InfiniteDelta()),
      last_cruise_min_rtt_(InfiniteDelta()),
      rounds_since_last_bw_advance_(0),
      probe_wait_rounds_(0),
      consecutive_high_rtt_cruises_(0),
      consecutive_low_rtt_cruises_(0),
      use_bbr2plus_probe_bw_(true),
      startup_restart_requested_(false),
      rtt_comp_startup_gain_(kDefaultRttCompStartupGain),
      rtt_comp_gain_(kDefaultRttCompGain),
      rtt_comp_rttvar_thresh_(kDefaultRttCompRttVarThresh),
      rtt_comp_jitter_win_rounds_(kDefaultRttCompJitterWindowRounds),
      fast_conv_rtt_thresh_(kDefaultFastConvRttThresh),
      fast_conv_probe_rtt_growth_thresh_(
          kDefaultFastConvProbeRttGrowthThresh),
      fast_conv_rtt_error_(InfiniteDelta()),
      fast_conv_probe_cycle_base_(kDefaultFastConvProbeCycleBase),
      fast_conv_probe_cycle_random_(kDefaultFastConvProbeCycleRandom),
      fast_conv_rounds_to_advance_bw_filter_(
          kDefaultFastConvRoundsToAdvanceBwFilter),
      switch_to_bbr2_rtt_multiplier_(kDefaultSwitchToBbr2RttMultiplier),
      switch_to_bbr2plus_rtt_multiplier_(
          kDefaultSwitchToBbr2PlusRttMultiplier),
      switch_to_bbr2_cruise_count_(kDefaultSwitchToBbr2CruiseCount),
      switch_to_bbr2plus_cruise_count_(kDefaultSwitchToBbr2PlusCruiseCount),
      pre_up_pacing_gain_(kDefaultPreUpGain),
      down_slightly_pacing_gain_(kDefaultDownSlightlyGain),
      rtt_compensation_enabled_(true),
      fast_convergence_enabled_(true),
      copa_style_(true) {
  QUIC_DVLOG(2) << this << " Initializing Bbr2PlusSender @ " << now;
}

void Bbr2PlusSender::Configure(const Bbr2PlusConfig& config) {
  const bool was_rtt_aware_probe_enabled = fast_convergence_enabled_;
  fast_convergence_enabled_ = config.enable_rtt_aware_probe;
  rtt_compensation_enabled_ = config.enable_rtt_compensation;
  copa_style_ = config.use_min_rtt_for_probe_guard;

  rtt_comp_jitter_win_rounds_ = AtLeastOneOr(
      config.rtt_jitter_window_rounds, kDefaultRttCompJitterWindowRounds);
  max_jitter_filter_.SetWindowLength(rtt_comp_jitter_win_rounds_);

  fast_conv_probe_rtt_growth_thresh_ = AtLeastOneOr(
      config.probe_rtt_growth_multiplier,
      kDefaultFastConvProbeRttGrowthThresh);
  fast_conv_rtt_thresh_ = AtLeastOneOr(
      config.bandwidth_drop_rtt_multiplier, kDefaultFastConvRttThresh);
  fast_conv_rtt_error_ = config.probe_rtt_error_cap_us == 0
                             ? InfiniteDelta()
                             : TimeDelta::FromMicroseconds(
                                   static_cast<int64_t>(
                                       config.probe_rtt_error_cap_us));
  fast_conv_probe_cycle_base_ = config.probe_cycle_base_rounds;
  fast_conv_probe_cycle_random_ = config.probe_cycle_random_rounds;
  fast_conv_rounds_to_advance_bw_filter_ = AtLeastOneOr(
      config.bandwidth_filter_force_advance_rounds,
      kDefaultFastConvRoundsToAdvanceBwFilter);
  pre_up_pacing_gain_ =
      PositiveOr(config.probe_try_pacing_gain, kDefaultPreUpGain);
  down_slightly_pacing_gain_ =
      PositiveOr(config.probe_down_pacing_gain, kDefaultDownSlightlyGain);

  switch_to_bbr2_rtt_multiplier_ = AtLeastOneOr(
      config.switch_to_bbr2_rtt_multiplier,
      kDefaultSwitchToBbr2RttMultiplier);
  switch_to_bbr2plus_rtt_multiplier_ = AtLeastOneOr(
      config.switch_to_bbr2plus_rtt_multiplier,
      kDefaultSwitchToBbr2PlusRttMultiplier);
  switch_to_bbr2_cruise_count_ = config.switch_to_bbr2_cruise_count;
  switch_to_bbr2plus_cruise_count_ = config.switch_to_bbr2plus_cruise_count;

  rtt_comp_rttvar_thresh_ = NonNegativeOr(
      config.rtt_jitter_threshold_multiplier, kDefaultRttCompRttVarThresh);
  rtt_comp_startup_gain_ = PositiveOr(
      config.startup_jitter_cwnd_gain, kDefaultRttCompStartupGain);
  rtt_comp_gain_ =
      PositiveOr(config.jitter_cwnd_gain, kDefaultRttCompGain);

  params_.loss_threshold = UnitIntervalOr(config.loss_threshold, 0.02f);
  params_.beta = UnitIntervalOr(config.beta, 0.30f);

  if (!fast_convergence_enabled_) {
    use_bbr2plus_probe_bw_ = false;
    startup_restart_requested_ = false;
    consecutive_high_rtt_cruises_ = 0;
    consecutive_low_rtt_cruises_ = 0;
  } else if (!was_rtt_aware_probe_enabled) {
    use_bbr2plus_probe_bw_ = true;
    consecutive_high_rtt_cruises_ = 0;
    consecutive_low_rtt_cruises_ = 0;
  }
}

void Bbr2PlusSender::OnCongestionEvent(
    bool rtt_updated,
    QuicByteCount prior_in_flight,
    QuicTime event_time,
    const AckedPacketVector& acked_packets,
    const LostPacketVector& lost_packets) {
  // rtt_stats_->latest_rtt() is only new when the send manager says so.
  // Sampling stale values on every ACK would make a phase boundary look like
  // a valid RTT measurement.
  if (rtt_updated) {
    UpdateLatestRttSample();
  }
  Bbr2Sender::OnCongestionEvent(rtt_updated, prior_in_flight, event_time,
                                acked_packets, lost_packets);
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

  if (mode_ != Bbr2Mode::PROBE_BW) {
    return;
  }
  const Bbr2ProbeBwMode::CyclePhase phase = GetCurrentProbeBwPhase();
  if (phase == Bbr2ProbeBwMode::CyclePhase::PROBE_CRUISE &&
      (current_cruise_min_rtt_.IsInfinite() ||
       latest_rtt < current_cruise_min_rtt_)) {
    current_cruise_min_rtt_ = latest_rtt;
  }
}

void Bbr2PlusSender::OnCongestionEventStarted(
    const Bbr2CongestionEvent& congestion_event) {
  if (!congestion_event.sample_max_bandwidth.IsZero() &&
      congestion_event.sample_max_bandwidth >= model_.MaxBandwidth()) {
    rounds_since_last_bw_advance_ = 0;
  }

  if (congestion_event.end_of_round_trip) {
    OnRoundStart(congestion_event);
  }
}

void Bbr2PlusSender::OnRoundStart(
    const Bbr2CongestionEvent& /*congestion_event*/) {
  ++rounds_since_last_bw_advance_;
  const Bbr2ProbeBwMode::CyclePhase phase = GetCurrentProbeBwPhase();
  if (IsUsingBbr2PlusProbeBw() &&
      phase == Bbr2ProbeBwMode::CyclePhase::PROBE_CRUISE) {
    --probe_wait_rounds_;
  }

  // This is the MinRTTcurr_rtt used at the end of the immediately preceding
  // PROBE_UP round by the continuous-probing test.
  if (phase == Bbr2ProbeBwMode::CyclePhase::PROBE_UP &&
      !curr_round_min_rtt_.IsInfinite()) {
    probe_up_min_rtt_ = curr_round_min_rtt_;
  }

  if (!curr_round_min_rtt_.IsInfinite()) {
    prior_round_min_rtt_ = last_round_min_rtt_;
    last_round_min_rtt_ = curr_round_min_rtt_;

    if (rtt_stats_ != nullptr && !rtt_stats_->mean_deviation().IsZero()) {
      max_jitter_filter_.SetWindowLength(rtt_comp_jitter_win_rounds_);
      max_jitter_filter_.Update(rtt_stats_->mean_deviation(),
                                model_.RoundTripCount());
    }
  }
  if (!curr_round_srtt_.IsZero()) {
    prior_round_srtt_ = last_round_srtt_;
    last_round_srtt_ = curr_round_srtt_;
  }

  curr_round_min_rtt_ = InfiniteDelta();
  curr_round_srtt_ = TimeDelta::Zero();
}

void Bbr2PlusSender::FinalizeCruiseAndMaybeSwitchMode() {
  if (current_cruise_min_rtt_.IsInfinite()) {
    return;
  }

  last_cruise_min_rtt_ = current_cruise_min_rtt_;
  current_cruise_min_rtt_ = InfiniteDelta();
  if (!fast_convergence_enabled_) {
    return;
  }

  const TimeDelta rtprop = model_.MinRtt();
  if (rtprop.IsZero() || rtprop.IsInfinite()) {
    return;
  }

  if (use_bbr2plus_probe_bw_) {
    if (switch_to_bbr2_cruise_count_ == 0) {
      consecutive_high_rtt_cruises_ = 0;
    } else if (last_cruise_min_rtt_ >
               rtprop * switch_to_bbr2_rtt_multiplier_) {
      ++consecutive_high_rtt_cruises_;
    } else {
      consecutive_high_rtt_cruises_ = 0;
    }
    consecutive_low_rtt_cruises_ = 0;

    if (switch_to_bbr2_cruise_count_ > 0 &&
        consecutive_high_rtt_cruises_ >= switch_to_bbr2_cruise_count_) {
      use_bbr2plus_probe_bw_ = false;
      startup_restart_requested_ = true;
      consecutive_high_rtt_cruises_ = 0;
      consecutive_low_rtt_cruises_ = 0;
      QUIC_DVLOG(2) << this
                    << " BBRv2+ dual mode: persistent Cruise RTT "
                    << last_cruise_min_rtt_ << " over RTprop " << rtprop
                    << "; switching to native BBRv2 ProbeBW and restarting"
                    << " STARTUP.";
    }
    return;
  }

  if (switch_to_bbr2plus_cruise_count_ == 0) {
    consecutive_low_rtt_cruises_ = 0;
  } else if (last_cruise_min_rtt_ <=
             rtprop * switch_to_bbr2plus_rtt_multiplier_) {
    ++consecutive_low_rtt_cruises_;
  } else {
    consecutive_low_rtt_cruises_ = 0;
  }
  consecutive_high_rtt_cruises_ = 0;

  if (switch_to_bbr2plus_cruise_count_ > 0 &&
      consecutive_low_rtt_cruises_ >= switch_to_bbr2plus_cruise_count_) {
    use_bbr2plus_probe_bw_ = true;
    consecutive_low_rtt_cruises_ = 0;
    QUIC_DVLOG(2) << this
                  << " BBRv2+ dual mode: Cruise RTT returned near RTprop; "
                  << "restoring RTT-aware BBRv2+ ProbeBW.";
  }
}

bool Bbr2PlusSender::ShouldAdvanceBandwidthFilter() const {
  if (!IsUsingBbr2PlusProbeBw() || mode_ != Bbr2Mode::PROBE_BW) {
    return false;
  }

  if (rounds_since_last_bw_advance_ >=
      static_cast<int>(fast_conv_rounds_to_advance_bw_filter_)) {
    return true;
  }
  if (last_round_min_rtt_.IsInfinite()) {
    return false;
  }

  // Algorithm 1 compares the RTT minimum of this round directly with RTprop.
  const TimeDelta rtprop = model_.MinRtt();
  if (rtprop.IsZero() || rtprop.IsInfinite()) {
    return false;
  }
  return last_round_min_rtt_ > rtprop * fast_conv_rtt_thresh_;
}

bool Bbr2PlusSender::ShouldEnterAggressiveProbe() const {
  if (!IsUsingBbr2PlusProbeBw()) {
    return false;
  }

  const TimeDelta baseline =
      copa_style_ ? ZeroOr(prior_round_min_rtt_, model_.MinRtt())
                  : ZeroOr(prior_round_srtt_, model_.MinRtt());
  const TimeDelta candidate =
      copa_style_ ? ZeroOr(last_round_min_rtt_, model_.MinRtt())
                  : ZeroOr(last_round_srtt_, model_.MinRtt());
  if (baseline.IsZero() || baseline.IsInfinite() || candidate.IsZero() ||
      candidate.IsInfinite()) {
    return true;
  }

  TimeDelta allowance =
      baseline * (fast_conv_probe_rtt_growth_thresh_ - 1.0f);
  if (allowance > fast_conv_rtt_error_) {
    allowance = fast_conv_rtt_error_;
  }
  return candidate <= baseline + allowance;
}

bool Bbr2PlusSender::ShouldProbeAgain() const {
  if (!IsUsingBbr2PlusProbeBw()) {
    return false;
  }

  const TimeDelta baseline = ZeroOr(min_rtt_before_probe_, model_.MinRtt());
  const TimeDelta candidate = ZeroOr(probe_up_min_rtt_, model_.MinRtt());
  if (baseline.IsZero() || baseline.IsInfinite() || candidate.IsZero() ||
      candidate.IsInfinite()) {
    return false;
  }

  TimeDelta allowance =
      baseline * (fast_conv_probe_rtt_growth_thresh_ - 1.0f);
  if (allowance > fast_conv_rtt_error_) {
    allowance = fast_conv_rtt_error_;
  }
  return candidate <= baseline + allowance;
}

void Bbr2PlusSender::PickProbeWaitRounds() {
  probe_wait_rounds_ = static_cast<int>(fast_conv_probe_cycle_base_);
  if (fast_conv_probe_cycle_random_ == 0 || random_ == nullptr) {
    return;
  }
  probe_wait_rounds_ += static_cast<int>(
      random_->nextInt() % fast_conv_probe_cycle_random_);
}

QuicByteCount Bbr2PlusSender::GetCwndCompensationBytes() const {
  if (!rtt_compensation_enabled_ || mode_ == Bbr2Mode::PROBE_RTT ||
      rtt_stats_ == nullptr) {
    return 0;
  }

  const TimeDelta rtprop = model_.MinRtt();
  const TimeDelta jitter = max_jitter_filter_.GetBest();
  if (rtprop.IsZero() || rtprop.IsInfinite() || jitter.IsZero() ||
      jitter.IsInfinite() || jitter <= rtprop * rtt_comp_rttvar_thresh_) {
    return 0;
  }

  // BBRv2's target cwnd already contains a cwnd gain.  Applying the same gain
  // to BW*jitter implements BDP = BW*(RTprop + jitter) in this host model.
  const float gain = mode_ == Bbr2Mode::STARTUP ? rtt_comp_startup_gain_
                                                 : rtt_comp_gain_;
  const QuicBandwidth bw = model_.MaxBandwidth().IsZero()
                               ? BandwidthEstimate()
                               : model_.MaxBandwidth();
  return bw.IsZero() ? 0 : bw * (jitter * gain);
}

void Bbr2PlusSender::ResetProbeCycleState() {
  min_rtt_before_probe_ = InfiniteDelta();
  probe_up_min_rtt_ = InfiniteDelta();
}

bool Bbr2PlusSender::EnablePlusProbeBwPhases() const {
  return IsUsingBbr2PlusProbeBw();
}

bool Bbr2PlusSender::ShouldStartProbeOnRound() const {
  return IsUsingBbr2PlusProbeBw() && probe_wait_rounds_ <= 0;
}

bool Bbr2PlusSender::ShouldAdvanceMaxBandwidthFilterOnRoundStart(
    Bbr2ProbeBwMode::CyclePhase phase) const {
  if (!IsUsingBbr2PlusProbeBw()) {
    return false;
  }
  if (phase != Bbr2ProbeBwMode::CyclePhase::PROBE_CRUISE &&
      phase != Bbr2ProbeBwMode::CyclePhase::PROBE_DOWN &&
      phase != Bbr2ProbeBwMode::CyclePhase::PROBE_DOWN_SLIGHTLY) {
    return false;
  }
  return ShouldAdvanceBandwidthFilter();
}

void Bbr2PlusSender::OnMaxBandwidthFilterAdvanced(
    Bbr2ProbeBwMode::CyclePhase /*phase*/) {
  rounds_since_last_bw_advance_ = 0;
}

bool Bbr2PlusSender::ShouldEnterProbeUpFromGuard() const {
  return ShouldEnterAggressiveProbe();
}

bool Bbr2PlusSender::ShouldProbeAgainFromPostUp() const {
  return ShouldProbeAgain();
}

bool Bbr2PlusSender::ConsumeStartupRestartRequest() {
  const bool requested = startup_restart_requested_;
  startup_restart_requested_ = false;
  return requested;
}

float Bbr2PlusSender::GetProbeBwPacingGain(
    Bbr2ProbeBwMode::CyclePhase phase,
    float pacing_gain) const {
  if (!IsUsingBbr2PlusProbeBw()) {
    return pacing_gain;
  }
  switch (phase) {
    case Bbr2ProbeBwMode::CyclePhase::PROBE_PRE_UP:
      return pre_up_pacing_gain_;
    case Bbr2ProbeBwMode::CyclePhase::PROBE_DOWN_SLIGHTLY:
      return down_slightly_pacing_gain_;
    case Bbr2ProbeBwMode::CyclePhase::PROBE_GUARD:
    case Bbr2ProbeBwMode::CyclePhase::PROBE_POST_UP:
      return 1.0f;
    default:
      return pacing_gain;
  }
}

void Bbr2PlusSender::OnProbeBwPhaseEntered(
    Bbr2ProbeBwMode::CyclePhase phase,
    QuicTime /*now*/) {
  // Entering REFILL marks the end of a Cruise phase.  This timing makes the
  // dual-mode test use the full preceding Cruise interval, as Algorithm 2
  // requires, regardless of whether the active mode is BBRv2+ or native BBRv2.
  if (phase == Bbr2ProbeBwMode::CyclePhase::PROBE_REFILL) {
    FinalizeCruiseAndMaybeSwitchMode();
  } else if (phase == Bbr2ProbeBwMode::CyclePhase::PROBE_CRUISE) {
    current_cruise_min_rtt_ = InfiniteDelta();
  }

  if (!IsUsingBbr2PlusProbeBw()) {
    return;
  }

  if (phase == Bbr2ProbeBwMode::CyclePhase::PROBE_DOWN ||
      phase == Bbr2ProbeBwMode::CyclePhase::PROBE_DOWN_SLIGHTLY) {
    PickProbeWaitRounds();
  }

  if (phase == Bbr2ProbeBwMode::CyclePhase::PROBE_UP) {
    min_rtt_before_probe_ = ZeroOr(last_round_min_rtt_, model_.MinRtt());
    probe_up_min_rtt_ = InfiniteDelta();
    return;
  }

  if (phase != Bbr2ProbeBwMode::CyclePhase::PROBE_GUARD &&
      phase != Bbr2ProbeBwMode::CyclePhase::PROBE_UP &&
      phase != Bbr2ProbeBwMode::CyclePhase::PROBE_POST_UP) {
    ResetProbeCycleState();
  }
}

}  // namespace dqc
