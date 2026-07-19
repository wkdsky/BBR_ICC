#include "bbr_r_sender.h"

#include <algorithm>
#include <cstddef>
#include <sstream>

#include "rtt_stats.h"

namespace dqc {
namespace {

constexpr size_t kRequiredRttWindow = 10;
constexpr double kResetRttRatio = 1.05;
constexpr double kStrongReductionRttRatio = 1.25;
constexpr double kStrongBandwidthFactor = 0.80;
// The reference source names this lesser_factor_0_9 but initializes it to .95.
constexpr double kMildBandwidthFactor = 0.95;
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
      rtt_stats_for_bbr_r_(rtt_stats),
      required_rtt_count_(0),
      adjusting_mode_is_on_(false),
      min_rtt_for_adjusting_(TimeDelta::Zero()),
      pacing_bandwidth_factor_(1.0) {}

BbrRSender::~BbrRSender() = default;

CongestionControlType BbrRSender::GetCongestionControlType() const {
  return kBBRR;
}

void BbrRSender::ResetRttAdjustment(TimeDelta base_min_rtt) {
  recent_inflated_rtt_us_.clear();
  required_rtt_count_ = 0;
  adjusting_mode_is_on_ = false;
  min_rtt_for_adjusting_ = base_min_rtt;
}

QuicBandwidth BbrRSender::GetPacingBandwidthForRate() {
  const QuicBandwidth bandwidth = BandwidthEstimate();
  pacing_bandwidth_factor_ = 1.0;

  const DebugState state = ExportDebugState();
  const TimeDelta latest_rtt = rtt_stats_for_bbr_r_->latest_rtt();
  if (bandwidth.IsZero() || latest_rtt.IsZero() ||
      state.mode != PROBE_BW || state.pacing_gain > 1.0f) {
    return bandwidth;
  }

  const TimeDelta base_min_rtt = GetMinRtt();
  if (min_rtt_for_adjusting_.IsZero()) {
    min_rtt_for_adjusting_ = base_min_rtt;
  }

  const int64_t latest_us = latest_rtt.ToMicroseconds();
  const int64_t base_us = std::max<int64_t>(1, base_min_rtt.ToMicroseconds());
  if (static_cast<double>(latest_us) <= kResetRttRatio * base_us) {
    ResetRttAdjustment(base_min_rtt);
  } else {
    ++required_rtt_count_;
    recent_inflated_rtt_us_.push_back(latest_us);
    if (recent_inflated_rtt_us_.size() > kRequiredRttWindow) {
      recent_inflated_rtt_us_.pop_front();
    }
    if (required_rtt_count_ >= kRequiredRttWindow &&
        recent_inflated_rtt_us_.size() == kRequiredRttWindow) {
      adjusting_mode_is_on_ = true;
      const int64_t window_min_us = *std::min_element(
          recent_inflated_rtt_us_.begin(), recent_inflated_rtt_us_.end());
      min_rtt_for_adjusting_ = TimeDelta::FromMicroseconds(window_min_us);
    }
  }

  // bbr_r.c explicitly bypasses the reduction in gain-cycle phase 3.
  if (state.gain_cycle_index == 3) {
    return bandwidth;
  }

  const int64_t reference_us = std::max<int64_t>(
      1, min_rtt_for_adjusting_.ToMicroseconds());
  if (static_cast<double>(latest_us) >=
      kStrongReductionRttRatio * reference_us) {
    pacing_bandwidth_factor_ = kStrongBandwidthFactor;
  } else if ((!adjusting_mode_is_on_ &&
              static_cast<double>(latest_us) >=
                  kResetRttRatio * reference_us) ||
             (adjusting_mode_is_on_ && latest_us > reference_us)) {
    pacing_bandwidth_factor_ = kMildBandwidthFactor;
  }

  return pacing_bandwidth_factor_ * bandwidth;
}

TimeDelta BbrRSender::GetMinRttExpiry() const {
  return kBbrRMinRttExpiry;
}

bool BbrRSender::ShouldRefreshMinRttTimestamp(
    TimeDelta sample_min_rtt,
    TimeDelta current_min_rtt,
    bool min_rtt_expired) const {
  return min_rtt_expired ||
         sample_min_rtt <= current_min_rtt + kMinRttTimestampSlack;
}

TimeDelta BbrRSender::GetGainCycleDuration() const {
  return min_rtt_for_adjusting_.IsZero()
             ? GetMinRtt()
             : min_rtt_for_adjusting_;
}

bool BbrRSender::RequireDrainTargetBeforeGainCycleAdvance() const {
  return true;
}

std::string BbrRSender::GetDebugState() const {
  std::ostringstream stream;
  stream << ProtoBbrSender::GetDebugState()
         << "\nBBR-R RTT adjustment: "
         << (adjusting_mode_is_on_ ? "active" : "learning")
         << "\nBBR-R inflated RTT samples: " << required_rtt_count_
         << "\nBBR-R adjustment min RTT (us): "
         << min_rtt_for_adjusting_.ToMicroseconds()
         << "\nBBR-R pacing bandwidth factor: "
         << pacing_bandwidth_factor_;
  return stream.str();
}

}  // namespace dqc
