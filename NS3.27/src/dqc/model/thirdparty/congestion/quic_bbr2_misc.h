// Copyright 2019 The Chromium Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef QUICHE_QUIC_CORE_CONGESTION_CONTROL_BBR2_MISC_H_
#define QUICHE_QUIC_CORE_CONGESTION_CONTROL_BBR2_MISC_H_

#include <algorithm>
#include <limits>

#include "quic_bandwidth_sampler.h"
#include "proto_constants.h"
#include "proto_windowed_filter.h"
#include "proto_bandwidth.h"
#include "proto_time.h"
#include "proto_types.h"
#include "quic_export.h"

namespace dqc {

template <typename T>
class QUIC_EXPORT_PRIVATE QuicLimits {
 public:
  QuicLimits(T min, T max) : min_(min), max_(max) {}

  // If [min, max] is an empty range, i.e. min > max, this function returns max,
  // because typically a value larger than max means "risky".
  T ApplyLimits(T raw_value) const {
    return std::min(max_, std::max(min_, raw_value));
  }

  T Min() const { return min_; }
  T Max() const { return max_; }

 private:
  T min_;
  T max_;
};

template <typename T>
QUIC_EXPORT_PRIVATE inline QuicLimits<T> MinMax(T min, T max) {
  return QuicLimits<T>(min, max);
}

template <typename T>
QUIC_EXPORT_PRIVATE inline QuicLimits<T> NoLessThan(T min) {
  return QuicLimits<T>(min, std::numeric_limits<T>::max());
}

template <typename T>
QUIC_EXPORT_PRIVATE inline QuicLimits<T> NoGreaterThan(T max) {
  return QuicLimits<T>(std::numeric_limits<T>::min(), max);
}

template <typename T>
QUIC_EXPORT_PRIVATE inline QuicLimits<T> Unlimited() {
  return QuicLimits<T>(std::numeric_limits<T>::min(),
                   std::numeric_limits<T>::max());
}

template <typename T>
QUIC_EXPORT_PRIVATE inline std::ostream& operator<<(std::ostream& os,
                                                    const QuicLimits<T>& limits) {
  return os << "[" << limits.Min() << ", " << limits.Max() << "]";
}
#define BBR_SCALE 8	/* scaling factor for fractions in BBR (e.g. gains) */
#define BBR_UNIT (1 << BBR_SCALE)
// Bbr2Params contains all parameters of a Bbr2Sender.
struct QUIC_EXPORT_PRIVATE Bbr2Params {
  Bbr2Params(QuicByteCount cwnd_min, QuicByteCount cwnd_max);
      

  /*
   * STARTUP parameters.
   */

  // The gain for CWND in startup.
  float startup_cwnd_gain = 2.0;
  float startup_pacing_gain = 2.885;

  // STARTUP or PROBE_UP are exited if the total bandwidth growth is less than
  // |full_bw_threshold| in the last |startup_full_bw_rounds| round trips.
  float full_bw_threshold = 1.25;
  // Legacy local name kept for code that has not yet been migrated.
  float startup_full_bw_threshold = 1.25;

  QuicRoundTripCount startup_full_bw_rounds = 3;

  // Number of rounds to stay in STARTUP when there is a persistent queue.
  // 0 disables this queue-based STARTUP exit.
  QuicRoundTripCount max_startup_queue_rounds = 0;

  // The minimum number of loss marking events to exit STARTUP.
  int64_t startup_full_loss_count =8;
      //GetQuicFlag(FLAGS_quic_bbr2_default_startup_full_loss_count);

  // If true, always exit STARTUP on loss, even if bandwidth is still growing.
  bool always_exit_startup_on_excess_loss = false;

  // If true, include extra acked during STARTUP.
  bool startup_include_extra_acked = false;

  /*
   * DRAIN parameters.
   */
  float drain_cwnd_gain = 2.0;
  float drain_pacing_gain = 1.0 / 2.885;

  /*
   * PROBE_BW parameters.
   */
  // Max amount of randomness to inject in round counting for Reno-coexistence.
  QuicRoundTripCount probe_bw_max_probe_rand_rounds = 2;

  // Max number of rounds before probing for Reno-coexistence.
  uint32_t probe_bw_probe_max_rounds = 63;

  // Multiplier to get Reno-style probe epoch duration as: k * BDP round trips.
  // If zero, disables Reno-style BDP-scaled coexistence mechanism.
  float probe_bw_probe_reno_gain = 1.0;

  // Minimum duration for BBR-native probes.
  TimeDelta probe_bw_probe_base_duration =
      TimeDelta::FromMilliseconds(2000
          /*GetQuicFlag(FLAGS_quic_bbr2_default_probe_bw_base_duration_ms)*/);

  // The upper bound of the random amount of BBR-native probes.
  TimeDelta probe_bw_probe_max_rand_duration =
      TimeDelta::FromMilliseconds(1000
          /*GetQuicFlag(FLAGS_quic_bbr2_default_probe_bw_max_rand_duration_ms)*/);

  // The minimum number of loss marking events to exit the PROBE_UP phase.
  int64_t probe_bw_full_loss_count =2;
      //GetQuicFlag(FLAGS_quic_bbr2_default_probe_bw_full_loss_count);

  // Legacy local parameter. Upstream now uses |full_bw_threshold| for PROBE_UP
  // queue detection; keep this only for local experiments.
  float probe_bw_probe_inflight_gain = 1.25;

  // Pacing gains.
  float probe_bw_probe_up_pacing_gain = 1.25;
  float probe_bw_probe_down_pacing_gain = 0.91;
  float probe_bw_default_pacing_gain = 1.0;

  float probe_bw_cwnd_gain = 2.0;
  float probe_up_cwnd_gain = 2.25;

  /*
   * PROBE_UP parameters.
   */
  bool probe_up_ignore_inflight_hi = true;
  bool probe_up_simplify_inflight_hi = false;

  // Number of rounds to stay in PROBE_UP when there is a persistent queue.
  // 0 disables this queue-based PROBE_UP exit.
  QuicRoundTripCount max_probe_up_queue_rounds = 0;

  /*
   * PROBE_RTT parameters.
   */
  float probe_rtt_inflight_target_bdp_fraction = 0.5;
  TimeDelta probe_rtt_period = TimeDelta::FromMilliseconds(10000
      /*GetQuicFlag(FLAGS_quic_bbr2_default_probe_rtt_period_ms)*/);
  TimeDelta probe_rtt_duration = TimeDelta::FromMilliseconds(200);

  /*
   * Parameters used by multiple modes.
   */

  // The initial value of the max ack height filter's window length.
  QuicRoundTripCount initial_max_ack_height_filter_window =10;
      //GetQuicFlag(FLAGS_quic_bbr2_default_initial_ack_height_filter_window);

  // Fraction of unutilized headroom to try to leave in path upon high loss.
  float inflight_hi_headroom =0.15;//0.01; //0.15 in early version
     // GetQuicFlag(FLAGS_quic_bbr2_default_inflight_hi_headroom);

  // Estimate startup/bw probing has gone too far if loss rate exceeds this.
  float loss_threshold = 0.02;//GetQuicFlag(FLAGS_quic_bbr2_default_loss_threshold);

  // A common factor for multiplicative decreases. Used for adjusting
  // bandwidth_lo, inflight_lo and inflight_hi upon losses.
  float beta = 0.3;

  QuicLimits<QuicByteCount> cwnd_limits;

  /*
   * Experimental flags from QuicConfig.
   */

  // Indicates app-limited calls should be ignored as long as there's
  // enough data inflight to see more bandwidth when necessary.
  bool flexible_app_limited = false;

  // Can be disabled by connection option 'B2NA'.
  bool add_ack_height_to_queueing_threshold = true;

  // Can be disabled by connection option 'B2RP'.
  bool avoid_unnecessary_probe_rtt = true;

  // Can be disabled by connection option 'B2CL'.
  bool avoid_too_low_probe_bw_cwnd =false;

  // Can be enabled by connection option 'B2LO'.
  bool ignore_inflight_lo = false;

  // Can be enabled by connection optoin 'B2HI'.
  bool limit_inflight_hi_by_cwnd = false;

  // Can be enabled by connection option 'B2H2'.
  bool limit_inflight_hi_by_max_delivered = false;

  // Can be disabled by connection option 'B2SL'.
  bool startup_loss_exit_use_max_delivered_for_inflight_hi = true;

  // Can be enabled by connection option 'B2DL'.
  bool use_bytes_delivered_for_inflight_hi = false;

  // Can be disabled by connection option 'B2RC'.
  bool enable_reno_coexistence = true;

  // For experimentation to improve fast convergence upon loss.
  enum QuicBandwidthLoMode : uint8_t {
    DEFAULT = 0,
    MIN_RTT_REDUCTION = 1,
    INFLIGHT_REDUCTION = 2,
    CWND_REDUCTION = 3,
  };

  // Different modes change bandwidth_lo_ differently upon loss.
  QuicBandwidthLoMode bw_lo_mode_ = QuicBandwidthLoMode::DEFAULT;

  // Set the pacing gain based on recent bandwidth increase in STARTUP.
  bool decrease_startup_pacing_at_end_of_round = false;

  bool enable_ecn{false};
  uint32_t full_ecn_count{2};
  uint32_t ecn_alpha_gain{BBR_UNIT * 1 / 16};//
  uint32_t ecn_factor{BBR_UNIT * 1 / 3};//
  uint32_t ecn_thresh{BBR_UNIT * 1 / 2};//
  uint32_t ecn_alpha{BBR_UNIT};
};

class QUIC_EXPORT_PRIVATE RoundTripCounter {
 public:
  RoundTripCounter();

  QuicRoundTripCount Count() const { return round_trip_count_; }

  QuicPacketNumber last_sent_packet() const { return last_sent_packet_; }

  // Must be called in ascending packet number order.
  void OnPacketSent(QuicPacketNumber packet_number);

  // Return whether a round trip has just completed.
  bool OnPacketsAcked(QuicPacketNumber last_acked_packet);

  void RestartRound();

 private:
  QuicRoundTripCount round_trip_count_;
  QuicPacketNumber last_sent_packet_;
  // The last sent packet number of the current round trip.
  QuicPacketNumber end_of_round_trip_;
};

class QUIC_EXPORT_PRIVATE MinRttFilter {
 public:
  MinRttFilter(TimeDelta initial_min_rtt,
               QuicTime initial_min_rtt_timestamp);

  void Update(TimeDelta sample_rtt, QuicTime now);

  void ForceUpdate(TimeDelta sample_rtt, QuicTime now);

  TimeDelta Get() const { return min_rtt_; }

  QuicTime GetTimestamp() const { return min_rtt_timestamp_; }

 private:
  TimeDelta min_rtt_;
  // Time when the current value of |min_rtt_| was assigned.
  QuicTime min_rtt_timestamp_;
};

class QUIC_EXPORT_PRIVATE Bbr2MaxBandwidthFilter {
 public:
  void Update(QuicBandwidth sample) {
    max_bandwidth_[1] = std::max(sample, max_bandwidth_[1]);
  }

  void Advance() {
    if (max_bandwidth_[1].IsZero()) {
      return;
    }

    max_bandwidth_[0] = max_bandwidth_[1];
    max_bandwidth_[1] = QuicBandwidth::Zero();
  }

  QuicBandwidth Get() const {
    return std::max(max_bandwidth_[0], max_bandwidth_[1]);
  }

  // Force set the max bandwidth (for special cases like early exit from PROBE_UP)
  void ForceSet(QuicBandwidth bandwidth) {
    max_bandwidth_[0] = bandwidth;
    max_bandwidth_[1] = bandwidth;
  }

 private:
  QuicBandwidth max_bandwidth_[2] = {QuicBandwidth::Zero(),
                                     QuicBandwidth::Zero()};
};

class QUIC_EXPORT_PRIVATE Bbr2MinBandwidthFilter {
 public:
  void Update(QuicBandwidth sample) {
    if (sample.IsZero()) {
      return;
    }
    if (min_bandwidth_[1].IsZero() || sample < min_bandwidth_[1]) {
      min_bandwidth_[1] = sample;
    }
  }

  void Advance() {
    if (min_bandwidth_[1].IsZero()) {
      return;
    }

    min_bandwidth_[0] = min_bandwidth_[1];
    min_bandwidth_[1] = QuicBandwidth::Zero();
  }

  QuicBandwidth Get() const {
    if (min_bandwidth_[0].IsZero()) {
      return min_bandwidth_[1];
    }
    if (min_bandwidth_[1].IsZero()) {
      return min_bandwidth_[0];
    }
    return std::min(min_bandwidth_[0], min_bandwidth_[1]);
  }

 private:
  QuicBandwidth min_bandwidth_[2] = {QuicBandwidth::Zero(),
                                     QuicBandwidth::Zero()};
};

// Information that are meaningful only when Bbr2Sender::OnCongestionEvent is
// running.
struct QUIC_EXPORT_PRIVATE Bbr2CongestionEvent {
  QuicTime event_time = QuicTime::Zero();

  // The congestion window prior to the processing of the ack/loss events.
  QuicByteCount prior_cwnd;

  // Total bytes inflight before the processing of the ack/loss events.
  QuicByteCount prior_bytes_in_flight = 0;

  // Total bytes inflight after the processing of the ack/loss events.
  QuicByteCount bytes_in_flight = 0;

  // Total bytes acked from acks in this event.
  QuicByteCount bytes_acked = 0;

  // Total bytes lost from losses in this event.
  QuicByteCount bytes_lost = 0;

  // Whether acked_packets indicates the end of a round trip.
  bool end_of_round_trip = false;

  // TODO(wub): After deprecating --quic_one_bw_sample_per_ack_event, use
  // last_packet_send_state.is_app_limited instead of this field.
  // Whether the last bandwidth sample from acked_packets is app limited.
  // false if acked_packets is empty.
  bool last_sample_is_app_limited = false;

  // Whether |sample_max_bandwidth| below is a valid sampler output.
  bool sample_valid = false;

  // Whether |sample_max_bandwidth| below is from an app-limited sample.
  // This is the sampler-level flag for the max delivery-rate sample in this
  // event, not the last packet's app-limited state.
  bool sample_is_app_limited = false;

  // When the event happened, whether the sender is probing for bandwidth.
  bool is_probing_for_bandwidth = false;

  // Minimum rtt of all bandwidth samples from acked_packets.
  // TimeDelta::Infinite() if acked_packets is empty.
  TimeDelta sample_min_rtt = TimeDelta::Infinite();

  // Maximum bandwidth of all bandwidth samples from acked_packets.
  QuicBandwidth sample_max_bandwidth = QuicBandwidth::Zero();

  // The send state of the largest packet in acked_packets, unless it is empty.
  // If acked_packets is empty, it's the send state of the largest packet in
  // lost_packets.
  QuicSendTimeState last_packet_send_state;
};

// Bbr2NetworkModel takes low level congestion signals(packets sent/acked/lost)
// as input and produces BBRv2 model parameters like inflight_(hi|lo),
// bandwidth_(hi|lo), bandwidth and rtt estimates, etc.
class QUIC_EXPORT_PRIVATE Bbr2NetworkModel {
 public:
  Bbr2NetworkModel(const Bbr2Params* params,
                   TimeDelta initial_rtt,
                   QuicTime initial_rtt_timestamp,
                   float cwnd_gain,
                   float pacing_gain,
                   const QuicBandwidthSampler* old_sampler);

  void OnPacketSent(QuicTime sent_time,
                    QuicByteCount bytes_in_flight,
                    QuicPacketNumber packet_number,
                    QuicByteCount bytes,
                    HasRetransmittableData is_retransmittable);

  void OnCongestionEventStart(QuicTime event_time,
                              const AckedPacketVector& acked_packets,
                              const LostPacketVector& lost_packets,
                              Bbr2CongestionEvent* congestion_event);

  void OnCongestionEventFinish(QuicPacketNumber least_unacked_packet,
                               const Bbr2CongestionEvent& congestion_event);

  // Update the model without a congestion event.
  // Max bandwidth is updated if |bandwidth| is larger than existing max
  // bandwidth. Min rtt is updated if |rtt| is non-zero and smaller than
  // existing min rtt.
  void UpdateNetworkParameters(QuicBandwidth bandwidth, TimeDelta rtt);

  // Update inflight/bandwidth short-term lower bounds.
  void AdaptLowerBounds(const Bbr2CongestionEvent& congestion_event);

  // Restart the current round trip as if it is starting now.
  void RestartRoundEarly();
  void RestartRound() { RestartRoundEarly(); }

  // Restore the full-bandwidth plateau detector to its initial STARTUP state.
  // BBRv2+ uses this when its dual-mode detector restarts from STARTUP.
  void ResetFullBandwidthForStartup() {
    full_bandwidth_reached_ = false;
    full_bandwidth_baseline_ = QuicBandwidth::Zero();
    rounds_without_bandwidth_growth_ = 0;
    rounds_with_queueing_ = 0;
  }

  void AdvanceMaxBandwidthFilter() {
    max_bandwidth_filter_.Advance();
    min_bandwidth_filter_.Advance();
  }

  void ForceSetMaxBandwidth(QuicBandwidth bandwidth) {
    max_bandwidth_filter_.ForceSet(bandwidth);
  }

  // Scale only the delivery-rate sample that enters the max-bandwidth
  // filter.  The sampler output, congestion-event sample, and latest-rate
  // signals remain raw so measurement users do not observe a modified
  // delivery rate.  The default factor is 1, preserving native BBRv2.
  void SetMaxBandwidthSampleAttenuation(double factor);
  double max_bandwidth_sample_attenuation() const {
    return max_bandwidth_sample_attenuation_;
  }
  QuicBandwidth max_bandwidth_filter_input() const {
    return max_bandwidth_filter_input_;
  }

  // Min bandwidth uses the same two-cycle advancement as MaxBandwidth, but
  // accepts only non-app-limited delivery-rate samples collected in Cruise.
  void SetMinBandwidthSampleCollection(bool enabled) {
    collect_min_bandwidth_samples_ = enabled;
  }
  void SetMinBandwidthSampleCorrection(double factor);
  QuicBandwidth MinBandwidth() const {
    return min_bandwidth_filter_.Get();
  }
  double min_bandwidth_sample_correction() const {
    return min_bandwidth_sample_correction_;
  }
  QuicBandwidth min_bandwidth_filter_input() const {
    return min_bandwidth_filter_input_;
  }

  void OnApplicationLimited() { bandwidth_sampler_.OnAppLimited(); }
  void OnEcnUpdate();

  QuicByteCount BDP() const {
    return MaxBandwidth() * MinRtt();
  }

  QuicByteCount BDP(QuicBandwidth bandwidth) const {
    return bandwidth * MinRtt();
  }

  QuicByteCount BDP(QuicBandwidth bandwidth, float gain) const {
    return bandwidth * MinRtt() * gain;
  }

  TimeDelta MinRtt() const { return min_rtt_filter_.Get(); }

  QuicTime MinRttTimestamp() const { return min_rtt_filter_.GetTimestamp(); }

  // Replace MinRtt and restart its PROBE_RTT expiry interval at |now|.
  // Callers must provide an independently validated empty-queue RTT sample.
  void ForceUpdateMinRtt(TimeDelta min_rtt, QuicTime now) {
    min_rtt_filter_.ForceUpdate(min_rtt, now);
  }

  // TODO(wub): If we do this too frequently, we can potentailly postpone
  // PROBE_RTT indefinitely. Observe how it works in production and improve it.
  void PostponeMinRttTimestamp(TimeDelta duration) {
    min_rtt_filter_.ForceUpdate(MinRtt(), MinRttTimestamp() + duration);
  }

  QuicBandwidth MaxBandwidth() const { return max_bandwidth_filter_.Get(); }

  QuicByteCount MaxAckHeight() const {
    return bandwidth_sampler_.max_ack_height();
  }

  // 2 packets. Used to indicate the typical number of bytes ACKed at once.
  QuicByteCount QueueingThresholdExtraBytes() const {
    return 2 * kDefaultTCPMSS;
  }

  bool cwnd_limited_before_aggregation_epoch() const {
    return cwnd_limited_before_aggregation_epoch_;
  }

  void EnableOverestimateAvoidance() {
    bandwidth_sampler_.EnableOverestimateAvoidance();
  }

  bool IsBandwidthOverestimateAvoidanceEnabled() const {
    return bandwidth_sampler_.IsOverestimateAvoidanceEnabled();
  }

  void OnPacketNeutered(QuicPacketNumber packet_number) {
    bandwidth_sampler_.OnPacketNeutered(packet_number);
  }

  uint64_t num_ack_aggregation_epochs() const {
    return bandwidth_sampler_.num_ack_aggregation_epochs();
  }

  bool MaybeExpireMinRtt(const Bbr2CongestionEvent& congestion_event);

  QuicBandwidth BandwidthEstimate() const {
    return std::min(MaxBandwidth(), bandwidth_lo_);
  }

  QuicRoundTripCount RoundTripCount() const {
    return round_trip_counter_.Count();
  }

  bool IsCongestionWindowLimited(
      const Bbr2CongestionEvent& congestion_event) const;

  bool IsInflightTooHigh(const Bbr2CongestionEvent& congestion_event,
                         int64_t max_loss_events) const;

  bool IsInflightTooHighWithEcn(
      const Bbr2CongestionEvent& congestion_event,
      int64_t max_loss_events,
      QuicByteCount delivered_ce) const;

  // Check bandwidth growth in the past round. Must be called at the end of a
  // round. Returns true if there was sufficient bandwidth growth and false
  // otherwise. If it has been too many rounds without growth, also sets
  // |full_bandwidth_reached_| to true.
  bool HasBandwidthGrowth(const Bbr2CongestionEvent& congestion_event);

  // Increments rounds_with_queueing_ if the minimum bytes in flight during the
  // round is greater than the BDP * |target_gain|.
  void CheckPersistentQueue(const Bbr2CongestionEvent& congestion_event,
                            float target_gain);

  QuicPacketNumber last_sent_packet() const {
    return round_trip_counter_.last_sent_packet();
  }

  QuicByteCount total_bytes_acked() const {
    return bandwidth_sampler_.total_bytes_acked();
  }

  QuicByteCount total_bytes_lost() const {
    return bandwidth_sampler_.total_bytes_lost();
  }

  QuicByteCount total_bytes_sent() const {
    return bandwidth_sampler_.total_bytes_sent();
  }

  bool is_app_limited() const {
    return bandwidth_sampler_.is_app_limited();
  }

  int64_t loss_events_in_round() const { return loss_events_in_round_; }

  QuicByteCount max_bytes_delivered_in_round() const {
    return max_bytes_delivered_in_round_;
  }

  QuicByteCount min_bytes_in_flight_in_round() const {
    return min_bytes_in_flight_in_round_;
  }

  bool inflight_hi_limited_in_round() const {
    return inflight_hi_limited_in_round_;
  }

  QuicPacketNumber end_of_app_limited_phase() const {
    return bandwidth_sampler_.end_of_app_limited_phase();
  }

  QuicBandwidth delivery_rate_latest() const { return delivery_rate_latest_; }
  QuicBandwidth bandwidth_latest() const { return bandwidth_latest_; }
  QuicBandwidth bandwidth_lo() const { return bandwidth_lo_; }
  static QuicBandwidth bandwidth_lo_default() {
    return QuicBandwidth::Infinite();
  }
  void clear_bandwidth_lo() { bandwidth_lo_ = bandwidth_lo_default(); }

  QuicByteCount inflight_latest() const { return inflight_latest_; }
  QuicByteCount inflight_lo() const { return inflight_lo_; }
  static QuicByteCount inflight_lo_default() {
    return std::numeric_limits<QuicByteCount>::max();
  }
  void clear_inflight_lo() { inflight_lo_ = inflight_lo_default(); }
  void cap_inflight_lo(QuicByteCount cap);

  QuicByteCount inflight_hi_with_headroom() const;
  QuicByteCount inflight_hi() const { return inflight_hi_; }
  static QuicByteCount inflight_hi_default() {
    return std::numeric_limits<QuicByteCount>::max();
  }
  void set_inflight_hi(QuicByteCount inflight_hi) {
    inflight_hi_ = inflight_hi;
  }

  float cwnd_gain() const { return cwnd_gain_; }
  void set_cwnd_gain(float cwnd_gain) { cwnd_gain_ = cwnd_gain; }

  float pacing_gain() const { return pacing_gain_; }
  void set_pacing_gain(float pacing_gain) { pacing_gain_ = pacing_gain; }

  bool improve_adjust_network_parameters() const {
    return improve_adjust_network_parameters_;
  }

  bool full_bandwidth_reached() const { return full_bandwidth_reached_; }
  void set_full_bandwidth_reached() { full_bandwidth_reached_ = true; }
  QuicBandwidth full_bandwidth_baseline() const {
    return full_bandwidth_baseline_;
  }
  QuicRoundTripCount rounds_without_bandwidth_growth() const {
    return rounds_without_bandwidth_growth_;
  }
  QuicRoundTripCount rounds_with_queueing() const {
    return rounds_with_queueing_;
  }

 private:
  // Called when a new round trip starts.
  void OnNewRound();

  const Bbr2Params& Params() const { return *params_; }
  const Bbr2Params* const params_;
  RoundTripCounter round_trip_counter_;

  // Bandwidth sampler provides BBR with the bandwidth measurements at
  // individual points.
  QuicBandwidthSampler bandwidth_sampler_;
  // The filter that tracks the maximum bandwidth over multiple recent round
  // trips.
  Bbr2MaxBandwidthFilter max_bandwidth_filter_;
  double max_bandwidth_sample_attenuation_ = 1.0;
  QuicBandwidth max_bandwidth_filter_input_ = QuicBandwidth::Zero();
  Bbr2MinBandwidthFilter min_bandwidth_filter_;
  bool collect_min_bandwidth_samples_ = false;
  double min_bandwidth_sample_correction_ = 1.0;
  QuicBandwidth min_bandwidth_filter_input_ = QuicBandwidth::Zero();
  MinRttFilter min_rtt_filter_;

  // Bytes lost in the current round. Updated once per congestion event.
  QuicByteCount bytes_lost_in_round_ = 0;
  // Number of loss marking events in the current round.
  int64_t loss_events_in_round_ = 0;

  // A max of bytes delivered among all congestion events in the current round.
  QuicByteCount max_bytes_delivered_in_round_ = 0;

  // The minimum bytes in flight during this round.
  QuicByteCount min_bytes_in_flight_in_round_ =
      std::numeric_limits<QuicByteCount>::max();

  // True if sending was limited by inflight_hi anytime in the current round.
  bool inflight_hi_limited_in_round_ = false;

  // Max bandwidth in the current round. Updated once per congestion event.
  QuicBandwidth delivery_rate_latest_ = QuicBandwidth::Zero();
  // Max bandwidth in the current round. Updated once per congestion event.
  QuicBandwidth bandwidth_latest_ = QuicBandwidth::Zero();
  // Max bandwidth of recent rounds. Updated once per round.
  QuicBandwidth bandwidth_lo_ = bandwidth_lo_default();

  // Max inflight in the current round. Updated once per congestion event.
  QuicByteCount inflight_latest_ = 0;
  // Max inflight of recent rounds. Updated once per round.
  QuicByteCount inflight_lo_ = inflight_lo_default();
  QuicByteCount inflight_hi_ = inflight_hi_default();

  float cwnd_gain_;
  float pacing_gain_;

  // Whether we are cwnd limited prior to the start of the current aggregation
  // epoch.
  bool cwnd_limited_before_aggregation_epoch_ = false;

  // bandwidth_lo_ at the beginning of a round with loss. Only used when the
  // non-default bandwidth_lo modes are enabled.
  QuicBandwidth prior_bandwidth_lo_ = QuicBandwidth::Zero();

  // STARTUP-centric fields, also used by PROBE_UP queue experiments.
  bool full_bandwidth_reached_ = false;
  QuicBandwidth full_bandwidth_baseline_ = QuicBandwidth::Zero();
  QuicRoundTripCount rounds_without_bandwidth_growth_ = 0;

  // Used by STARTUP and PROBE_UP to decide when to exit.
  QuicRoundTripCount rounds_with_queueing_ = 0;

  uint32_t ecn_in_round_=0;
  const bool improve_adjust_network_parameters_ =false;
     // GetQuicReloadableFlag(quic_bbr2_improve_adjust_network_parameters);
};

enum class Bbr2Mode : uint8_t {
  // Startup phase of the connection.
  STARTUP,
  // After achieving the highest possible bandwidth during the startup, lower
  // the pacing rate in order to drain the queue.
  DRAIN,
  // Cruising mode.
  PROBE_BW,
  // Temporarily slow down sending in order to empty the buffer and measure
  // the real minimum RTT.
  PROBE_RTT,
};

QUIC_EXPORT_PRIVATE inline std::ostream& operator<<(std::ostream& os,
                                                    const Bbr2Mode& mode) {
  switch (mode) {
    case Bbr2Mode::STARTUP:
      return os << "STARTUP";
    case Bbr2Mode::DRAIN:
      return os << "DRAIN";
    case Bbr2Mode::PROBE_BW:
      return os << "PROBE_BW";
    case Bbr2Mode::PROBE_RTT:
      return os << "PROBE_RTT";
  }
  return os << "<Invalid Mode>";
}

// The base class for all BBRv2 modes. A Bbr2Sender is in one mode at a time,
// this interface is used to implement mode-specific behaviors.
class Bbr2Sender;
class QUIC_EXPORT_PRIVATE Bbr2ModeBase {
 public:
  Bbr2ModeBase(Bbr2Sender* sender, Bbr2NetworkModel* model)
      : sender_(sender), model_(model) {}

  virtual ~Bbr2ModeBase() = default;

  // Called when entering/leaving this mode.
  // congestion_event != nullptr means BBRv2 is switching modes in the context
  // of a ack and/or loss.
  virtual void Enter(QuicTime now,
                     const Bbr2CongestionEvent* congestion_event) = 0;
  virtual void Leave(QuicTime now,
                     const Bbr2CongestionEvent* congestion_event) = 0;

  virtual Bbr2Mode OnCongestionEvent(
      QuicByteCount prior_in_flight,
      QuicTime event_time,
      const AckedPacketVector& acked_packets,
      const LostPacketVector& lost_packets,
      const Bbr2CongestionEvent& congestion_event) = 0;

  virtual QuicLimits<QuicByteCount> GetCwndLimits() const = 0;

  virtual bool IsProbingForBandwidth() const = 0;

  virtual Bbr2Mode OnExitQuiescence(QuicTime now,
                                    QuicTime quiescence_start_time) = 0;

 protected:
  Bbr2Sender* const sender_;
  Bbr2NetworkModel* model_;
};

QUIC_EXPORT_PRIVATE inline QuicByteCount BytesInFlight(
    const QuicSendTimeState& send_state) {
  DCHECK(send_state.is_valid);
  if (send_state.bytes_in_flight != 0) {
    return send_state.bytes_in_flight;
  }
  return send_state.total_bytes_sent - send_state.total_bytes_acked -
         send_state.total_bytes_lost;
}

}  // namespace quic

#endif  // QUICHE_QUIC_CORE_CONGESTION_CONTROL_BBR2_MISC_H_
