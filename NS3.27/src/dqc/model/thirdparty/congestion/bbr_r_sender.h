#pragma once

#include <cstdint>
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
  void OnCongestionEvent(bool rtt_updated,
                         QuicByteCount prior_in_flight,
                         ProtoTime event_time,
                         const AckedPacketVector& acked_packets,
                         const LostPacketVector& lost_packets) override;

  bool IsRttAdjustmentActive() const { return adjusting_mode_is_on_; }
  double GetPacingBandwidthFactor() const { return pacing_bandwidth_factor_; }
  TimeDelta GetAdjustmentMinRtt() const { return min_rtt_for_adjusting_; }

 protected:
  QuicBandwidth GetPacingBandwidthForRate() override;
  TimeDelta GetMinRttExpiry() const override;
  bool ShouldRefreshMinRttTimestamp(TimeDelta sample_min_rtt,
                                    TimeDelta current_min_rtt,
                                    bool min_rtt_expired) const override;
  void OnUpdatedRttSample(TimeDelta sample_rtt) override;
  TimeDelta GetGainCycleDuration() const override;
  bool RequireDrainTargetBeforeGainCycleAdvance() const override;
  bool UsePriorInflightForGainCycleDrain() const override;
  bool RequireProbeInflightStrictlyAboveTarget() const override;
  bool ShouldAddAckAggregationToCongestionWindow() const override;
  float GetProbeBandwidthCongestionWindowGain() const override;

 private:
  struct RttMinSample {
    uint64_t sample_index;
    int64_t rtt_us;
  };

  void ResetRttAdjustment(TimeDelta base_min_rtt);
  void ResetInflatedRttWindow(uint64_t sample_index, int64_t rtt_us);
  int64_t UpdateInflatedRttMinimum(int64_t rtt_us);

  RttMinSample inflated_rtt_window_[3];
  uint64_t required_rtt_count_;
  bool adjusting_mode_is_on_;
  TimeDelta min_rtt_for_adjusting_;
  TimeDelta current_rtt_sample_;
  bool has_current_rtt_sample_;
  double pacing_bandwidth_factor_;
};

}  // namespace dqc
