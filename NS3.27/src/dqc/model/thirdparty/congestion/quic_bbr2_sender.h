// Copyright 2019 The Chromium Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef QUICHE_QUIC_CORE_CONGESTION_CONTROL_BBR2_SENDER_H_
#define QUICHE_QUIC_CORE_CONGESTION_CONTROL_BBR2_SENDER_H_

#include <cstdint>
#include <functional>

#include "quic_bandwidth_sampler.h"
#include "quic_bbr2_drain.h"
#include "quic_bbr2_misc.h"
#include "quic_bbr2_probe_bw.h"
#include "quic_bbr2_probe_rtt.h"
#include "quic_bbr2_startup.h"
#include "quic_bbr_sender.h"
#include "rtt_stats.h"
#include "proto_send_algorithm_interface.h"
#include "proto_windowed_filter.h"
#include "proto_bandwidth.h"
#include "proto_types.h"
#include "quic_export.h"
#include "random.h"
#include "quic_logging.h"
namespace dqc {

class QUIC_EXPORT_PRIVATE Bbr2Sender : public SendAlgorithmInterface {
 public:
  Bbr2Sender(QuicTime now,
             const RttStats* rtt_stats,
             const QuicUnackedPacketMap* unacked_packets,
             QuicPacketCount initial_cwnd_in_packets,
             QuicPacketCount max_cwnd_in_packets,
             Random* random,
             QuicConnectionStats* stats,
             bool enable_ecn=false,
             QuicBbrSender* old_sender=nullptr,
             CongestionControlType congestion_control_type = kBBRv2,
             bool enable_probe_rtt = true);

  ~Bbr2Sender() override = default;

  // Start implementation of SendAlgorithmInterface.
  bool InSlowStart() const override { return mode_ == Bbr2Mode::STARTUP; }

  bool InRecovery() const override {
    // TODO(wub): Implement Recovery.
    return false;
  }

  bool ShouldSendProbingPacket() const override;

  /*void SetFromConfig(const QuicConfig& config,
                     Perspective perspective) override;

  void ApplyConnectionOptions(const QuicTagVector& connection_options) override;*/

  void AdjustNetworkParameters(QuicBandwidth bandwidth,
                               TimeDelta rtt,
                               bool allow_cwnd_to_decrease) override;
  void SetNumEmulatedConnections(int num_connections) override{}
  void SetInitialCongestionWindowInPackets(
      QuicPacketCount congestion_window) override;

  void OnCongestionEvent(bool rtt_updated,
                         QuicByteCount prior_in_flight,
                         QuicTime event_time,
                         const AckedPacketVector& acked_packets,
                         const LostPacketVector& lost_packets) override;

  void OnPacketSent(QuicTime sent_time,
                    QuicByteCount bytes_in_flight,
                    QuicPacketNumber packet_number,
                    QuicByteCount bytes,
                    HasRetransmittableData is_retransmittable) override;

  void OnPacketNeutered(QuicPacketNumber packet_number) override;

  void OnRetransmissionTimeout(bool /*packets_retransmitted*/) override {}

  void OnConnectionMigration() override {}

  bool CanSend(QuicByteCount bytes_in_flight) override;

  QuicBandwidth PacingRate(QuicByteCount bytes_in_flight) const override;

  QuicBandwidth BandwidthEstimate() const override {
    return model_.BandwidthEstimate();
  }

  QuicBandwidth DeliveryRateLatest() const {
    return model_.delivery_rate_latest();
  }

  // Get the latest bandwidth sample (instant receive rate)
  QuicBandwidth BandwidthLatest() const {
    return model_.bandwidth_latest();
  }

  float PacingGain() const { return model_.pacing_gain(); }
  QuicByteCount TotalBytesSent() const { return model_.total_bytes_sent(); }
  QuicByteCount TotalBytesAcked() const { return model_.total_bytes_acked(); }
  QuicByteCount TotalBytesLost() const { return model_.total_bytes_lost(); }
  QuicTime LastAckEventTime() const { return last_ack_event_time_; }

  QuicByteCount GetCongestionWindow() const override;

  QuicByteCount GetSlowStartThreshold() const override { return 0; }

  CongestionControlType GetCongestionControlType() const override {
    return congestion_control_type_;
  }

  std::string GetDebugState() const override;

  void OnApplicationLimited(QuicByteCount bytes_in_flight) override;
  void OnUpdateEcnBytes(uint64_t ecn_ce_count) override;
  //void PopulateConnectionStats(QuicConnectionStats* stats) const override;
  // End implementation of SendAlgorithmInterface.

  const Bbr2Params& Params() const { return params_; }

  QuicByteCount GetMinimumCongestionWindow() const {
    return cwnd_limits().Min();
  }

  // Returns the min of BDP and congestion window.
  QuicByteCount GetTargetBytesInflight() const;
  bool IsProbeRttEnabled() const { return enable_probe_rtt_; }

  bool IsBandwidthOverestimateAvoidanceEnabled() const {
    return model_.IsBandwidthOverestimateAvoidanceEnabled();
  }
  QuicByteCount GetBytesEcnInRounds() const{return bytes_ecn_in_round_;}
  struct QUIC_EXPORT_PRIVATE DebugState {
    Bbr2Mode mode;

    // Shared states.
    QuicRoundTripCount round_trip_count;
    QuicBandwidth bandwidth_hi = QuicBandwidth::Zero();
    QuicBandwidth bandwidth_lo = QuicBandwidth::Zero();
    QuicBandwidth bandwidth_est = QuicBandwidth::Zero();
    QuicByteCount inflight_hi;
    QuicByteCount inflight_lo;
    QuicByteCount max_ack_height;
    TimeDelta min_rtt = TimeDelta::Zero();
    QuicTime min_rtt_timestamp = QuicTime::Zero();
    QuicByteCount congestion_window;
    QuicBandwidth pacing_rate = QuicBandwidth::Zero();
    bool last_sample_is_app_limited;
    QuicPacketNumber end_of_app_limited_phase;

    // Mode-specific debug states.
    Bbr2StartupMode::DebugState startup;
    Bbr2DrainMode::DebugState drain;
    Bbr2ProbeBwMode::DebugState probe_bw;
    Bbr2ProbeRttMode::DebugState probe_rtt;
  };

  DebugState ExportDebugState() const;

  // Get current BBR mode as an index for tracing
  // 0: STARTUP, 1: DRAIN, 2: PROBE_BW_DOWN, 3: PROBE_BW_CRUISE,
  // 4: PROBE_BW_REFILL, 5: PROBE_BW_UP, 6: PROBE_RTT
  virtual int32_t GetCurrentBbrModeIndex() const;

  // Experimental hook used by experiments/bbrv2_probe_order. It forces one
  // PROBE_UP entry at the first PROBE_BW ACK event on or after the target time.
  void SetExperimentalForcedProbeUp(QuicTime probe_up_time,
                                    TimeDelta min_probe_up_duration);
  bool ShouldForceProbeUp(QuicTime now) const;
  void MarkExperimentalForcedProbeUpStarted(QuicTime now);
  bool ExperimentalForcedProbeUpExitAllowed(QuicTime now) const;
  void SetExperimentalMaxCongestionWindowPackets(
      QuicPacketCount max_cwnd_in_packets);

  using QueueDelayTraceCallback =
      std::function<void(uint32_t queue_delay_ms,
                         uint32_t latest_rtt_ms,
                         uint32_t min_rtt_ms)>;
  void SetQueueDelayTraceCallback(QueueDelayTraceCallback cb) {
    queue_delay_trace_cb_ = cb;
  }

 protected:
  // Protected members for subclasses like FreqccSender and FreqCCv2Sender
  // Note: Declaration order matters for initialization
  Bbr2Mode mode_;
  const RttStats* const rtt_stats_;
  const QuicUnackedPacketMap* const unacked_packets_;
  Random* random_;
  QuicConnectionStats* connection_stats_;
  const CongestionControlType congestion_control_type_;
  const bool enable_probe_rtt_;
  Bbr2Params params_;

  // Max congestion window when adjusting network parameters.
  QuicByteCount max_cwnd_when_network_parameters_adjusted_ =
      kMaxInitialCongestionWindow * kDefaultTCPMSS;

  // model_ depends on params_
  Bbr2NetworkModel model_;
  // initial_cwnd_ uses cwnd_limits() which uses params_
  const QuicByteCount initial_cwnd_;
  // cwnd_ is initialized from initial_cwnd_
  QuicByteCount cwnd_;

  // Helper functions for subclasses
  QuicByteCount GetTargetCongestionWindow(float gain) const;
  const QuicLimits<QuicByteCount>& cwnd_limits() const;
  virtual void OnCongestionEventStarted(
      const Bbr2CongestionEvent& congestion_event);
  virtual bool EnablePlusProbeBwPhases() const;
  virtual bool ShouldStartProbeOnRound() const;
  virtual bool ShouldAdvanceMaxBandwidthFilterOnRoundStart(
      Bbr2ProbeBwMode::CyclePhase phase) const;
  virtual void OnMaxBandwidthFilterAdvanced(Bbr2ProbeBwMode::CyclePhase phase);
  virtual bool ShouldEnterProbeUpFromGuard() const;
  virtual bool ShouldProbeAgainFromPostUp() const;
  virtual bool ShouldDelayProbeUpExit(QuicTime now) const;
  virtual bool ShouldDelayProbeBwCruiseExit(QuicTime now) const;
  virtual float GetProbeBwPacingGain(Bbr2ProbeBwMode::CyclePhase phase,
                                     float pacing_gain) const;
  virtual void OnProbeBwPhaseEntered(Bbr2ProbeBwMode::CyclePhase phase,
                                     QuicTime now);

 private:
  void UpdatePacingRate(QuicByteCount bytes_acked);
  void UpdateCongestionWindow(QuicByteCount bytes_acked);
  void OnEnterQuiescence(QuicTime now);
  void OnExitQuiescence(QuicTime now);

  // Helper function for BBR2_MODE_DISPATCH.
  Bbr2ProbeRttMode& probe_rtt_or_die() {
    DCHECK_EQ(mode_, Bbr2Mode::PROBE_RTT);
    return probe_rtt_;
  }

  const Bbr2ProbeRttMode& probe_rtt_or_die() const {
    DCHECK_EQ(mode_, Bbr2Mode::PROBE_RTT);
    return probe_rtt_;
  }

  uint64_t RandomUint64(uint64_t max) const {
    return random_->nextInt()% max;
  }

  // Returns true if there are enough bytes in flight to ensure more bandwidth
  // will be observed if present.
  bool IsPipeSufficientlyFull() const;

  // Cwnd limits imposed by the current Bbr2 mode.
  QuicLimits<QuicByteCount> GetCwndLimitsByMode() const;

  void UpdateRoundTripAlpha();

  // Current pacing rate.
  QuicBandwidth pacing_rate_;

  QuicTime last_quiescence_start_ = QuicTime::Zero();

  Bbr2StartupMode startup_;
  Bbr2DrainMode drain_;
  Bbr2ProbeBwMode probe_bw_;
  Bbr2ProbeRttMode probe_rtt_;

  // Debug only.
  bool has_non_app_limited_sample_ = false;
  bool last_sample_is_app_limited_;
  QueueDelayTraceCallback queue_delay_trace_cb_;
  QuicTime last_ack_event_time_ = QuicTime::Zero();
  bool experimental_forced_probe_up_enabled_ = false;
  bool experimental_forced_probe_up_started_ = false;
  QuicTime experimental_forced_probe_up_time_ = QuicTime::Zero();
  QuicTime experimental_forced_probe_up_start_time_ = QuicTime::Zero();
  TimeDelta experimental_forced_probe_up_min_duration_ = TimeDelta::Zero();
  
  QuicByteCount ecn_ce_count_{0};
  QuicByteCount alpha_last_delivered_{0};
  QuicByteCount alpha_last_delivered_ce_{0};
  QuicByteCount bytes_ecn_in_round_{0};
  friend class Bbr2StartupMode;
  friend class Bbr2DrainMode;
  friend class Bbr2ProbeBwMode;
  friend class Bbr2ProbeRttMode;
};

QUIC_EXPORT_PRIVATE std::ostream& operator<<(
    std::ostream& os,
    const Bbr2Sender::DebugState& state);

}  // namespace quic

#endif  // QUICHE_QUIC_CORE_CONGESTION_CONTROL_BBR2_SENDER_H_
