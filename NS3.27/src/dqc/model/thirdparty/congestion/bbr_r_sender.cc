#include "bbr_r_sender.h"

#include <algorithm>
#include <limits>
#include <sstream>

namespace dqc {
namespace {

constexpr uint64_t kRequiredRttWindow = 10;
constexpr int64_t kRttRatioScale = 100;
constexpr int64_t kResetRttRatio = 105;
constexpr int64_t kStrongReductionRttRatio = 125;
constexpr double kStrongBandwidthFactor = 0.80;
// The reference source names this lesser_factor_0_9 but initializes it to .95.
constexpr double kMildBandwidthFactor = 0.95;
// Preserve Linux bbr_r.c's fixed-point values exactly: BBR_UNIT is 256,
// bbr_high_gain is 739, and bbr_drain_gain is 88.
constexpr float kReferenceStartupGain = 739.0f / 256.0f;
constexpr float kReferenceDrainGain = 88.0f / 256.0f;
const TimeDelta kBbrRMinRttExpiry = TimeDelta::FromSeconds(13);
const TimeDelta kMinRttTimestampSlack = TimeDelta::FromMilliseconds(10);

}  // namespace

BbrRSender::BbrRSender(ProtoTime now,
                       const RttStats* rtt_stats,
                       const UnackedPacketMap* unacked_packets,
                       QuicPacketCount initial_tcp_congestion_window,
                       QuicPacketCount max_tcp_congestion_window,
                       Random* random)
    : ProtoBbrSender(now,
                     rtt_stats,
                     unacked_packets,
                     initial_tcp_congestion_window,
                     max_tcp_congestion_window,
                     random,
                     false),
      required_rtt_count_(0),
      adjusting_mode_is_on_(false),
      min_rtt_for_adjusting_(TimeDelta::Zero()),
      current_rtt_sample_(TimeDelta::Zero()),
      has_current_rtt_sample_(false),
      pacing_bandwidth_factor_(1.0) {
  ResetInflatedRttWindow(0, std::numeric_limits<uint32_t>::max());
  set_high_gain(kReferenceStartupGain);
  set_high_cwnd_gain(kReferenceStartupGain);
  set_drain_gain(kReferenceDrainGain);
}

BbrRSender::~BbrRSender() = default;

CongestionControlType BbrRSender::GetCongestionControlType() const {
  return kBBRR;
}

void BbrRSender::OnCongestionEvent(
    bool rtt_updated,
    QuicByteCount prior_in_flight,
    ProtoTime event_time,
    const AckedPacketVector& acked_packets,
    const LostPacketVector& lost_packets) {
  // bbr_r.c uses the current rate sample only.  Do not reuse an RTT from a
  // preceding ACK-only event when the DQC event has no valid rate sample.
  current_rtt_sample_ = TimeDelta::Zero();
  has_current_rtt_sample_ = false;
  ProtoBbrSender::OnCongestionEvent(rtt_updated, prior_in_flight, event_time,
                                    acked_packets, lost_packets);
}

void BbrRSender::ResetRttAdjustment(TimeDelta base_min_rtt) {
  ResetInflatedRttWindow(0, std::numeric_limits<uint32_t>::max());
  required_rtt_count_ = 0;
  adjusting_mode_is_on_ = false;
  min_rtt_for_adjusting_ = base_min_rtt;
}

void BbrRSender::ResetInflatedRttWindow(uint64_t sample_index,
                                        int64_t rtt_us) {
  const RttMinSample sample = {sample_index, rtt_us};
  inflated_rtt_window_[0] = sample;
  inflated_rtt_window_[1] = sample;
  inflated_rtt_window_[2] = sample;
}

int64_t BbrRSender::UpdateInflatedRttMinimum(int64_t rtt_us) {
  const RttMinSample sample = {required_rtt_count_, rtt_us};

  // This is the three-candidate minmax_rtt tracker in bbr_r.c, ported with
  // the same strict window and subwindow comparisons.
  if (sample.rtt_us <= inflated_rtt_window_[0].rtt_us ||
      sample.sample_index - inflated_rtt_window_[2].sample_index >
          kRequiredRttWindow) {
    ResetInflatedRttWindow(sample.sample_index, sample.rtt_us);
    return inflated_rtt_window_[0].rtt_us;
  }

  if (sample.rtt_us <= inflated_rtt_window_[1].rtt_us) {
    inflated_rtt_window_[2] = sample;
    inflated_rtt_window_[1] = sample;
  } else if (sample.rtt_us <= inflated_rtt_window_[2].rtt_us) {
    inflated_rtt_window_[2] = sample;
  }

  const uint64_t elapsed =
      sample.sample_index - inflated_rtt_window_[0].sample_index;
  if (elapsed > kRequiredRttWindow) {
    inflated_rtt_window_[0] = inflated_rtt_window_[1];
    inflated_rtt_window_[1] = inflated_rtt_window_[2];
    inflated_rtt_window_[2] = sample;
    if (sample.sample_index - inflated_rtt_window_[0].sample_index >
        kRequiredRttWindow) {
      inflated_rtt_window_[0] = inflated_rtt_window_[1];
      inflated_rtt_window_[1] = inflated_rtt_window_[2];
      inflated_rtt_window_[2] = sample;
    }
  } else if (inflated_rtt_window_[1].sample_index ==
                 inflated_rtt_window_[0].sample_index &&
             elapsed > kRequiredRttWindow / 4) {
    inflated_rtt_window_[2] = sample;
    inflated_rtt_window_[1] = sample;
  } else if (inflated_rtt_window_[2].sample_index ==
                 inflated_rtt_window_[1].sample_index &&
             elapsed > kRequiredRttWindow / 2) {
    inflated_rtt_window_[2] = sample;
  }

  return inflated_rtt_window_[0].rtt_us;
}

QuicBandwidth BbrRSender::GetPacingBandwidthForRate() {
  const QuicBandwidth bandwidth = BandwidthEstimate();
  pacing_bandwidth_factor_ = 1.0;

  const DebugState state = ExportDebugState();
  if (!has_current_rtt_sample_ || state.mode != PROBE_BW ||
      state.pacing_gain > 1.0f) {
    return bandwidth;
  }

  const TimeDelta base_min_rtt = GetMinRtt();
  if (min_rtt_for_adjusting_.IsZero()) {
    min_rtt_for_adjusting_ = base_min_rtt;
  }

  const int64_t latest_us = current_rtt_sample_.ToMicroseconds();
  const int64_t base_us = std::max<int64_t>(1, base_min_rtt.ToMicroseconds());
  if (kRttRatioScale * latest_us <= kResetRttRatio * base_us) {
    ResetRttAdjustment(base_min_rtt);
  } else {
    ++required_rtt_count_;
    const int64_t window_min_us = UpdateInflatedRttMinimum(latest_us);
    if (required_rtt_count_ >= kRequiredRttWindow) {
      adjusting_mode_is_on_ = true;
      min_rtt_for_adjusting_ = TimeDelta::FromMicroseconds(window_min_us);
    }
  }

  // bbr_r.c explicitly bypasses the reduction in gain-cycle phase 3.
  if (state.gain_cycle_index == 3) {
    return bandwidth;
  }

  const int64_t reference_us = std::max<int64_t>(
      1, min_rtt_for_adjusting_.ToMicroseconds());
  if (kRttRatioScale * latest_us >=
      kStrongReductionRttRatio * reference_us) {
    pacing_bandwidth_factor_ = kStrongBandwidthFactor;
  } else if ((!adjusting_mode_is_on_ &&
              kRttRatioScale * latest_us >=
                  kResetRttRatio * reference_us) ||
             (adjusting_mode_is_on_ && latest_us > reference_us)) {
    pacing_bandwidth_factor_ = kMildBandwidthFactor;
  }

  return pacing_bandwidth_factor_ * bandwidth;
}

void BbrRSender::OnUpdatedRttSample(TimeDelta sample_rtt) {
  if (sample_rtt.IsZero()) {
    return;
  }
  current_rtt_sample_ = sample_rtt;
  has_current_rtt_sample_ = true;
}

TimeDelta BbrRSender::GetMinRttExpiry() const {
  return kBbrRMinRttExpiry;
}

bool BbrRSender::ShouldRefreshMinRttTimestamp(
    TimeDelta sample_min_rtt,
    TimeDelta current_min_rtt,
    bool min_rtt_expired) const {
  return min_rtt_expired ||
         sample_min_rtt < current_min_rtt + kMinRttTimestampSlack;
}

TimeDelta BbrRSender::GetGainCycleDuration() const {
  return min_rtt_for_adjusting_.IsZero()
             ? GetMinRtt()
             : min_rtt_for_adjusting_;
}

bool BbrRSender::RequireDrainTargetBeforeGainCycleAdvance() const {
  return true;
}

bool BbrRSender::UsePriorInflightForGainCycleDrain() const {
  return true;
}

bool BbrRSender::RequireProbeInflightStrictlyAboveTarget() const {
  return true;
}

bool BbrRSender::ShouldAddAckAggregationToCongestionWindow() const {
  return false;
}

float BbrRSender::GetProbeBandwidthCongestionWindowGain() const {
  return 2.0f;
}

std::string BbrRSender::GetDebugState() const {
  std::ostringstream stream;
  stream << ProtoBbrSender::GetDebugState()
         << "\nBBR-R RTT adjustment: "
         << (adjusting_mode_is_on_ ? "active" : "learning")
         << "\nBBR-R inflated RTT samples: " << required_rtt_count_
         << "\nBBR-R adjustment min RTT (us): "
         << min_rtt_for_adjusting_.ToMicroseconds()
         << "\nBBR-R raw RTT sample (us): "
         << (has_current_rtt_sample_
                 ? current_rtt_sample_.ToMicroseconds()
                 : 0)
         << "\nBBR-R pacing bandwidth factor: "
         << pacing_bandwidth_factor_;
  return stream.str();
}

}  // namespace dqc
