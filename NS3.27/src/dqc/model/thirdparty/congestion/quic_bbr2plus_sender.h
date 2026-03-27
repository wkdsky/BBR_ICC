// DQC BBRv2plus sender built on top of the existing QUIC BBRv2 model.

#ifndef QUIC_BBR2PLUS_SENDER_H_
#define QUIC_BBR2PLUS_SENDER_H_

#include <cstdint>

#include "proto_windowed_filter.h"
#include "quic_bbr2_sender.h"
#include "quic_export.h"

namespace dqc {

class QUIC_EXPORT_PRIVATE Bbr2PlusSender final : public Bbr2Sender {
 public:
  Bbr2PlusSender(QuicTime now,
                 const RttStats* rtt_stats,
                 const QuicUnackedPacketMap* unacked_packets,
                 QuicPacketCount initial_cwnd_in_packets,
                 QuicPacketCount max_cwnd_in_packets,
                 Random* random,
                 QuicConnectionStats* stats,
                 bool enable_ecn = false);

  ~Bbr2PlusSender() override = default;

  CongestionControlType GetCongestionControlType() const override {
    return enable_ecn_ ? kBBRv2PlusEcn : kBBRv2Plus;
  }

  void OnCongestionEvent(bool rtt_updated,
                         QuicByteCount prior_in_flight,
                         QuicTime event_time,
                         const AckedPacketVector& acked_packets,
                         const LostPacketVector& lost_packets) override;

  QuicByteCount GetCongestionWindow() const override;

 private:
  using MinRttWindow =
      WindowedFilter<TimeDelta,
                     MinFilter<TimeDelta>,
                     QuicRoundTripCount,
                     QuicRoundTripCount>;
  using MaxJitterWindow =
      WindowedFilter<TimeDelta,
                     MaxFilter<TimeDelta>,
                     QuicRoundTripCount,
                     QuicRoundTripCount>;

  Bbr2ProbeBwMode::CyclePhase GetCurrentProbeBwPhase() const;
  void UpdateLatestRttSample();
  void OnRoundStart(const Bbr2CongestionEvent& congestion_event);
  bool ShouldAdvanceBandwidthFilter() const;
  bool ShouldEnterAggressiveProbe() const;
  bool ShouldProbeAgain() const;
  void PickProbeWaitRounds();
  QuicByteCount GetRttCompensationBytes() const;
  void ResetProbeCycleState();

  void OnCongestionEventStarted(
      const Bbr2CongestionEvent& congestion_event) override;
  bool EnablePlusProbeBwPhases() const override;
  bool ShouldStartProbeOnRound() const override;
  bool ShouldAdvanceMaxBandwidthFilterOnRoundStart(
      Bbr2ProbeBwMode::CyclePhase phase) const override;
  void OnMaxBandwidthFilterAdvanced(Bbr2ProbeBwMode::CyclePhase phase) override;
  bool ShouldEnterProbeUpFromGuard() const override;
  bool ShouldProbeAgainFromPostUp() const override;
  float GetProbeBwPacingGain(Bbr2ProbeBwMode::CyclePhase phase,
                             float pacing_gain) const override;
  void OnProbeBwPhaseEntered(Bbr2ProbeBwMode::CyclePhase phase,
                             QuicTime now) override;

  const bool enable_ecn_;

  // RTT-aware plus state adapted to the DQC BBRv2 host implementation.
  MinRttWindow rc_min_rtt_filter_;
  MaxJitterWindow max_jitter_filter_;
  TimeDelta prior_round_srtt_;
  TimeDelta last_round_srtt_;
  TimeDelta curr_round_srtt_;
  TimeDelta prior_round_min_rtt_;
  TimeDelta last_round_min_rtt_;
  TimeDelta curr_round_min_rtt_;
  TimeDelta min_rtt_before_probe_;
  TimeDelta probe_up_min_rtt_;
  mutable uint32_t probe_again_count_in_cycle_;
  int rounds_since_last_bw_advance_;
  int probe_wait_rounds_;

  // Parameter surface kept close to the Linux BBRv2plus defaults.
  uint32_t rc_min_rtt_win_rounds_;
  float rtt_comp_startup_gain_;
  float rtt_comp_gain_;
  float rtt_comp_rttvar_thresh_;
  uint32_t rtt_comp_jitter_win_rounds_;
  float fast_conv_rtt_thresh_;
  float fast_conv_preup_thresh_;
  TimeDelta fast_conv_rtt_error_;
  float fast_conv_probe_again_thresh_;
  uint32_t fast_conv_probe_cycle_base_;
  uint32_t fast_conv_probe_cycle_random_;
  uint32_t fast_conv_rounds_to_advance_bw_filter_;
  uint32_t max_probe_again_per_cycle_;
  float pre_up_pacing_gain_;
  float down_slightly_pacing_gain_;
  bool rtt_compensation_enabled_;
  bool fast_convergence_enabled_;
  bool copa_style_;
};

}  // namespace dqc

#endif  // QUIC_BBR2PLUS_SENDER_H_
