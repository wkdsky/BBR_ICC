// DQC BBRv2+ sender built on top of the existing QUIC BBRv2 model.

#ifndef QUIC_BBR2PLUS_SENDER_H_
#define QUIC_BBR2PLUS_SENDER_H_

#include <cstdint>

#include "proto_windowed_filter.h"
#include "quic_bbr2_sender.h"
#include "quic_export.h"

namespace dqc {

// Runtime configuration for the mechanisms described in BBRv2+ (CN 2022).
// The defaults follow the paper's Table 2 where it specifies a value.  The
// cwnd-gain defaults preserve the BBRv2 host's existing BDP scaling.
struct QUIC_EXPORT_PRIVATE Bbr2PlusConfig {
  bool enable_rtt_aware_probe = true;
  bool enable_rtt_compensation = true;
  bool use_min_rtt_for_probe_guard = true;

  // ProbeTry/ProbeUp and BtlBW expiry (gamma and theta in the paper).
  float probe_rtt_growth_multiplier = 1.02f;
  float bandwidth_drop_rtt_multiplier = 1.10f;
  // Zero applies the paper's gamma comparison directly; a positive value
  // enables the optional Linux-prototype compatibility cap.
  uint32_t probe_rtt_error_cap_us = 0;
  uint32_t probe_cycle_base_rounds = 8;
  uint32_t probe_cycle_random_rounds = 4;
  uint32_t bandwidth_filter_force_advance_rounds = 25;
  float probe_try_pacing_gain = 1.10f;
  float probe_down_pacing_gain = 0.90f;

  // Dual ProbeBW mode (lambda1, lambda2, eta1 and eta2 in the paper).
  float switch_to_bbr2_rtt_multiplier = 1.10f;
  float switch_to_bbr2plus_rtt_multiplier = 1.05f;
  uint32_t switch_to_bbr2_cruise_count = 2;
  uint32_t switch_to_bbr2plus_cruise_count = 4;

  // BDP compensation under RTT jitter (mu in the paper).
  float rtt_jitter_threshold_multiplier = 0.50f;
  uint32_t rtt_jitter_window_rounds = 4;
  float startup_jitter_cwnd_gain = 2.885f;
  float jitter_cwnd_gain = 2.0f;

  // BBRv2 loss response parameters.  The paper recommends selecting alpha
  // for the bottleneck buffer; defaults retain the BBRv2 host behavior.
  float loss_threshold = 0.02f;
  float beta = 0.30f;
};

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

  void Configure(const Bbr2PlusConfig& config);
  bool IsUsingBbr2PlusProbeBw() const {
    return fast_convergence_enabled_ && use_bbr2plus_probe_bw_;
  }

 private:
  using MaxJitterWindow =
      WindowedFilter<TimeDelta,
                     MaxFilter<TimeDelta>,
                     QuicRoundTripCount,
                     QuicRoundTripCount>;

  Bbr2ProbeBwMode::CyclePhase GetCurrentProbeBwPhase() const;
  void UpdateLatestRttSample();
  void OnRoundStart(const Bbr2CongestionEvent& congestion_event);
  void FinalizeCruiseAndMaybeSwitchMode();
  bool ShouldAdvanceBandwidthFilter() const;
  bool ShouldEnterAggressiveProbe() const;
  bool ShouldProbeAgain() const;
  void PickProbeWaitRounds();
  void ResetProbeCycleState();

  void OnCongestionEventStarted(
      const Bbr2CongestionEvent& congestion_event) override;
  QuicByteCount GetCwndCompensationBytes() const override;
  bool EnablePlusProbeBwPhases() const override;
  bool ShouldStartProbeOnRound() const override;
  bool ShouldAdvanceMaxBandwidthFilterOnRoundStart(
      Bbr2ProbeBwMode::CyclePhase phase) const override;
  void OnMaxBandwidthFilterAdvanced(Bbr2ProbeBwMode::CyclePhase phase) override;
  bool ShouldEnterProbeUpFromGuard() const override;
  bool ShouldProbeAgainFromPostUp() const override;
  bool ConsumeStartupRestartRequest() override;
  float GetProbeBwPacingGain(Bbr2ProbeBwMode::CyclePhase phase,
                             float pacing_gain) const override;
  void OnProbeBwPhaseEntered(Bbr2ProbeBwMode::CyclePhase phase,
                             QuicTime now) override;

  const bool enable_ecn_;

  // RTT state used by ProbeTry and BtlBW expiry.
  MaxJitterWindow max_jitter_filter_;
  TimeDelta prior_round_srtt_;
  TimeDelta last_round_srtt_;
  TimeDelta curr_round_srtt_;
  TimeDelta prior_round_min_rtt_;
  TimeDelta last_round_min_rtt_;
  TimeDelta curr_round_min_rtt_;
  TimeDelta min_rtt_before_probe_;
  TimeDelta probe_up_min_rtt_;
  TimeDelta current_cruise_min_rtt_;
  TimeDelta last_cruise_min_rtt_;
  int rounds_since_last_bw_advance_;
  int probe_wait_rounds_;

  // Dual-mode state.
  uint32_t consecutive_high_rtt_cruises_;
  uint32_t consecutive_low_rtt_cruises_;
  bool use_bbr2plus_probe_bw_;
  bool startup_restart_requested_;

  // Tunable parameters.
  float rtt_comp_startup_gain_;
  float rtt_comp_gain_;
  float rtt_comp_rttvar_thresh_;
  uint32_t rtt_comp_jitter_win_rounds_;
  float fast_conv_rtt_thresh_;
  float fast_conv_probe_rtt_growth_thresh_;
  TimeDelta fast_conv_rtt_error_;
  uint32_t fast_conv_probe_cycle_base_;
  uint32_t fast_conv_probe_cycle_random_;
  uint32_t fast_conv_rounds_to_advance_bw_filter_;
  float switch_to_bbr2_rtt_multiplier_;
  float switch_to_bbr2plus_rtt_multiplier_;
  uint32_t switch_to_bbr2_cruise_count_;
  uint32_t switch_to_bbr2plus_cruise_count_;
  float pre_up_pacing_gain_;
  float down_slightly_pacing_gain_;
  bool rtt_compensation_enabled_;
  bool fast_convergence_enabled_;
  bool copa_style_;
};

}  // namespace dqc

#endif  // QUIC_BBR2PLUS_SENDER_H_
