#include "quic_bbr2plus_sender.h"

#include <algorithm>
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
constexpr float kDefaultFastConvProbeAgainThresh = 0.02f;
constexpr uint32_t kDefaultFastConvProbeCycleBase = 8;
constexpr uint32_t kDefaultFastConvProbeCycleRandom = 4;
constexpr uint32_t kDefaultFastConvRoundsToAdvanceBwFilter = 25;
constexpr uint32_t kDefaultMaxProbeAgainPerCycle = 1;
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
      prior_round_srtt_(TimeDelta::Zero()),
      last_round_srtt_(TimeDelta::Zero()),
      curr_round_srtt_(TimeDelta::Zero()),
      prior_round_min_rtt_(InfiniteDelta()),
      last_round_min_rtt_(InfiniteDelta()),
      curr_round_min_rtt_(InfiniteDelta()),
      min_rtt_before_probe_(InfiniteDelta()),
      probe_up_min_rtt_(InfiniteDelta()),
      probe_again_count_in_cycle_(0),
      rounds_since_last_bw_advance_(0),
      probe_wait_rounds_(0),
      rc_min_rtt_win_rounds_(kDefaultRcMinRttWindowRounds),
      rtt_comp_startup_gain_(kDefaultRttCompStartupGain),
      rtt_comp_gain_(kDefaultRttCompGain),
      rtt_comp_rttvar_thresh_(kDefaultRttCompRttVarThresh),
      rtt_comp_jitter_win_rounds_(kDefaultRttCompJitterWindowRounds),
      fast_conv_rtt_thresh_(kDefaultFastConvRttThresh),
      fast_conv_preup_thresh_(kDefaultFastConvPreUpThresh),
      fast_conv_rtt_error_(TimeDelta::FromMicroseconds(kDefaultFastConvRttErrorUs)),
      fast_conv_probe_again_thresh_(kDefaultFastConvProbeAgainThresh),
      fast_conv_probe_cycle_base_(kDefaultFastConvProbeCycleBase),
      fast_conv_probe_cycle_random_(kDefaultFastConvProbeCycleRandom),
      fast_conv_rounds_to_advance_bw_filter_(
          kDefaultFastConvRoundsToAdvanceBwFilter),
      max_probe_again_per_cycle_(kDefaultMaxProbeAgainPerCycle),
      pre_up_pacing_gain_(kDefaultPreUpGain),
      down_slightly_pacing_gain_(kDefaultDownSlightlyGain),
      rtt_compensation_enabled_(true),
      fast_convergence_enabled_(true),
      copa_style_(false) {
  QUIC_DVLOG(2) << this << " Initializing Bbr2PlusSender @ " << now;
}

void Bbr2PlusSender::OnCongestionEvent(bool rtt_updated,
                                       QuicByteCount prior_in_flight,
                                       QuicTime event_time,
                                       const AckedPacketVector& acked_packets,
                                       const LostPacketVector& lost_packets) {
  UpdateLatestRttSample();
  Bbr2Sender::OnCongestionEvent(rtt_updated, prior_in_flight, event_time,
                                acked_packets, lost_packets);
}

QuicByteCount Bbr2PlusSender::GetCongestionWindow() const {
  return Bbr2Sender::GetCongestionWindow() + GetRttCompensationBytes();
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

  if (mode_ == Bbr2Mode::PROBE_BW &&
      GetCurrentProbeBwPhase() == Bbr2ProbeBwMode::CyclePhase::PROBE_UP) {
    if (probe_up_min_rtt_.IsInfinite() || latest_rtt < probe_up_min_rtt_) {
      probe_up_min_rtt_ = latest_rtt;
    }
  }
}

void Bbr2PlusSender::OnCongestionEventStarted(
    const Bbr2CongestionEvent& congestion_event) {
  if (!congestion_event.sample_max_bandwidth.IsZero() &&
      congestion_event.sample_max_bandwidth >= model_.MaxBandwidth()) {
    rounds_since_last_bw_advance_ = 0;
  }

  if (!congestion_event.end_of_round_trip) {
    return;
  }
  OnRoundStart(congestion_event);
}

void Bbr2PlusSender::OnRoundStart(
    const Bbr2CongestionEvent& congestion_event) {
  ++rounds_since_last_bw_advance_;
  if (mode_ == Bbr2Mode::PROBE_BW) {
    Bbr2ProbeBwMode::CyclePhase phase = GetCurrentProbeBwPhase();
    if (phase == Bbr2ProbeBwMode::CyclePhase::PROBE_CRUISE) {
      --probe_wait_rounds_;
    }
  }

  if (!curr_round_min_rtt_.IsInfinite()) {
    rc_min_rtt_filter_.SetWindowLength(rc_min_rtt_win_rounds_);
    rc_min_rtt_filter_.Update(curr_round_min_rtt_, model_.RoundTripCount());
    prior_round_min_rtt_ = last_round_min_rtt_;
    last_round_min_rtt_ = curr_round_min_rtt_;
  }
  if (!curr_round_srtt_.IsZero()) {
    prior_round_srtt_ = last_round_srtt_;
    last_round_srtt_ = curr_round_srtt_;
  }

  if (rtt_stats_ != nullptr && !rtt_stats_->mean_deviation().IsZero()) {
    max_jitter_filter_.SetWindowLength(rtt_comp_jitter_win_rounds_);
    max_jitter_filter_.Update(rtt_stats_->mean_deviation(),
                              model_.RoundTripCount());
  }

  curr_round_min_rtt_ = InfiniteDelta();
  curr_round_srtt_ = TimeDelta::Zero();
  if (congestion_event.bytes_acked > 0 || congestion_event.bytes_lost > 0) {
    UpdateLatestRttSample();
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

  if (last_round_min_rtt_.IsInfinite()) {
    return false;
  }
  const TimeDelta reference =
      rc_min_rtt_filter_.GetBest().IsInfinite() ? model_.MinRtt()
                                                : rc_min_rtt_filter_.GetBest();
  if (reference.IsZero() || reference.IsInfinite()) {
    return false;
  }
  return last_round_min_rtt_ > reference * fast_conv_rtt_thresh_;
}

bool Bbr2PlusSender::ShouldEnterAggressiveProbe() const {
  const TimeDelta baseline = copa_style_
                                 ? ZeroOr(prior_round_min_rtt_, model_.MinRtt())
                                 : ZeroOr(prior_round_srtt_, model_.MinRtt());
  const TimeDelta candidate = copa_style_
                                  ? ZeroOr(last_round_min_rtt_, model_.MinRtt())
                                  : ZeroOr(last_round_srtt_, model_.MinRtt());
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

bool Bbr2PlusSender::ShouldProbeAgain() const {
  if (probe_again_count_in_cycle_ >= max_probe_again_per_cycle_) {
    return false;
  }

  const TimeDelta baseline = ZeroOr(min_rtt_before_probe_, model_.MinRtt());
  const TimeDelta candidate = ZeroOr(probe_up_min_rtt_, model_.MinRtt());
  if (baseline.IsZero() || baseline.IsInfinite() || candidate.IsZero() ||
      candidate.IsInfinite()) {
    return false;
  }

  TimeDelta allowance = baseline * fast_conv_probe_again_thresh_;
  if (allowance > fast_conv_rtt_error_) {
    allowance = fast_conv_rtt_error_;
  }
  if (candidate > baseline + allowance) {
    return false;
  }

  ++probe_again_count_in_cycle_;
  return true;
}

void Bbr2PlusSender::PickProbeWaitRounds() {
  probe_wait_rounds_ = static_cast<int>(fast_conv_probe_cycle_base_);
  if (fast_conv_probe_cycle_random_ == 0 || random_ == nullptr) {
    return;
  }
  probe_wait_rounds_ +=
      static_cast<int>(random_->nextInt() % fast_conv_probe_cycle_random_);
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

void Bbr2PlusSender::ResetProbeCycleState() {
  min_rtt_before_probe_ = InfiniteDelta();
  probe_up_min_rtt_ = InfiniteDelta();
  probe_again_count_in_cycle_ = 0;
}

bool Bbr2PlusSender::EnablePlusProbeBwPhases() const {
  return fast_convergence_enabled_;
}

bool Bbr2PlusSender::ShouldStartProbeOnRound() const {
  return fast_convergence_enabled_ && probe_wait_rounds_ <= 0;
}

bool Bbr2PlusSender::ShouldAdvanceMaxBandwidthFilterOnRoundStart(
    Bbr2ProbeBwMode::CyclePhase phase) const {
  if (!fast_convergence_enabled_) {
    return false;
  }
  if (phase != Bbr2ProbeBwMode::CyclePhase::PROBE_CRUISE &&
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

float Bbr2PlusSender::GetProbeBwPacingGain(
    Bbr2ProbeBwMode::CyclePhase phase,
    float pacing_gain) const {
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
