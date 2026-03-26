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

  void OnPacketSent(QuicTime sent_time,
                    QuicByteCount bytes_in_flight,
                    QuicPacketNumber packet_number,
                    QuicByteCount bytes,
                    HasRetransmittableData is_retransmittable) override;

  void OnCongestionEvent(bool rtt_updated,
                         QuicByteCount prior_in_flight,
                         QuicTime event_time,
                         const AckedPacketVector& acked_packets,
                         const LostPacketVector& lost_packets) override;

  QuicBandwidth PacingRate(QuicByteCount bytes_in_flight) const override;

  QuicByteCount GetCongestionWindow() const override;

  int32_t GetCurrentBbrModeIndex() const override;

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

  enum class ProbeExtensionState : uint8_t {
    kInactive,
    kPreUp,
    kGuard,
    kDownSlightly,
    kPostUp,
  };

  Bbr2ProbeBwMode::CyclePhase GetCurrentProbeBwPhase() const;
  void UpdateLatestRttSample();
  void AdvanceRoundIfNeeded(QuicRoundTripCount previous_round);
  void OnRoundStart();
  void OnProbePhaseChange(Bbr2ProbeBwMode::CyclePhase previous_phase,
                          Bbr2ProbeBwMode::CyclePhase current_phase,
                          QuicTime event_time);
  bool ShouldAdvanceBandwidthFilter() const;
  bool ShouldEnterAggressiveProbe() const;
  QuicBandwidth RateFromGain(float gain) const;
  QuicByteCount GetRttCompensationBytes() const;
  void ResetProbeExtensionState();

  const bool enable_ecn_;

  // RTT-aware state borrowed from Linux BBRv2plus.
  MinRttWindow rc_min_rtt_filter_;
  MaxJitterWindow max_jitter_filter_;
  QuicRoundTripCount last_round_count_;
  TimeDelta ever_measured_min_rtt_;
  TimeDelta last_round_srtt_;
  TimeDelta curr_round_srtt_;
  TimeDelta last_round_min_rtt_;
  TimeDelta curr_round_min_rtt_;
  TimeDelta min_rtt_in_cruise_;
  TimeDelta last_min_rtt_in_cruise_;
  TimeDelta min_rtt_before_probe_;
  QuicTime current_time_;
  QuicTime probe_phase_start_time_;

  ProbeExtensionState probe_extension_state_;
  int rounds_since_last_bw_advance_;

  // Parameter surface kept close to the Linux BBRv2plus defaults.
  uint32_t rc_min_rtt_win_rounds_;
  float rtt_comp_startup_gain_;
  float rtt_comp_gain_;
  float rtt_comp_rttvar_thresh_;
  uint32_t rtt_comp_jitter_win_rounds_;
  float fast_conv_rtt_thresh_;
  float fast_conv_preup_thresh_;
  TimeDelta fast_conv_rtt_error_;
  uint32_t fast_conv_rounds_to_advance_bw_filter_;
  float pre_up_pacing_gain_;
  float down_slightly_pacing_gain_;
  bool rtt_compensation_enabled_;
  bool fast_convergence_enabled_;
  bool copa_style_;
};

}  // namespace dqc

#endif  // QUIC_BBR2PLUS_SENDER_H_
