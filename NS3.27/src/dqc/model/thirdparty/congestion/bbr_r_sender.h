#pragma once

#include <cstdint>
#include <deque>
#include <string>

#include "proto_bbr_sender.h"

namespace dqc {

// DQC-hosted migration of BBR-R (Zheng et al., Computer Networks 2024).
//
// BBR-R retains BBRv1's bandwidth model and congestion window, but reduces
// the bandwidth used for pacing when persistent RTT inflation indicates
// multi-flow queue competition.  All adaptive state is per sender; the Linux
// reference implementation stores it in file-level globals, which would make
// independent ns-3 flows interfere with each other.
class BbrRSender : public ProtoBbrSender {
 public:
  BbrRSender(ProtoTime now,
             const RttStats* rtt_stats,
             const UnackedPacketMap* unacked_packets,
             QuicPacketCount initial_tcp_congestion_window,
             QuicPacketCount max_tcp_congestion_window,
             Random* random);
  ~BbrRSender() override;

  CongestionControlType GetCongestionControlType() const override;
  std::string GetDebugState() const override;

  bool IsRttAdjustmentActive() const { return adjusting_mode_is_on_; }
  double GetPacingBandwidthFactor() const { return pacing_bandwidth_factor_; }
  TimeDelta GetAdjustmentMinRtt() const { return min_rtt_for_adjusting_; }

 protected:
  QuicBandwidth GetPacingBandwidthForRate() override;
  TimeDelta GetMinRttExpiry() const override;
  bool ShouldRefreshMinRttTimestamp(TimeDelta sample_min_rtt,
                                    TimeDelta current_min_rtt,
                                    bool min_rtt_expired) const override;
  TimeDelta GetGainCycleDuration() const override;
  bool RequireDrainTargetBeforeGainCycleAdvance() const override;

 private:
  void ResetRttAdjustment(TimeDelta base_min_rtt);

  const RttStats* rtt_stats_for_bbr_r_;
  std::deque<int64_t> recent_inflated_rtt_us_;
  uint64_t required_rtt_count_;
  bool adjusting_mode_is_on_;
  TimeDelta min_rtt_for_adjusting_;
  double pacing_bandwidth_factor_;
};

}  // namespace dqc
