// FBBR - BBRv2 with fixed-frequency CRUISE modulation and FBBR branches.

#ifndef FBBR_SENDER_H_
#define FBBR_SENDER_H_

#include <cstdint>
#include <deque>
#include <functional>
#include <limits>
#include <map>
#include <string>
#include <vector>

#include "quic_bbr2_sender.h"
#include "quic_export.h"

namespace dqc {

struct FBBRRateSample {
  QuicTime time;
  QuicBandwidth rate;
  bool valid = true;
  bool is_app_limited = false;
  QuicByteCount acked_bytes = 0;
};

struct FBBRRttSample {
  QuicTime time;
  double rtt_ms;
};

enum class FBBRAmplitudeMode {
  kFixed,
  kMiu2,
  kMiu3,
  kMiu4,
  kMiu8,
  kSR2,
  kSR3,
  kSR4,
  kSR8,
  kSR12,
  kSR16,
  // A configurable 1/N fraction of the current sending rate.
  kSRFraction,
};

// Parses the canonical Nsr form (and the legacy srN alias). N is restricted
// to the inclusive range [1, 20], so 20sr means current_sending_rate / 20.
bool ParseFBBRSendingRateDenominator(const std::string& value,
                                     uint32_t* denominator);

// Valid values are fixed_mbps, a positive Mbps literal, legacy Miu modes, or
// an Nsr sending-rate fraction accepted by ParseFBBRSendingRateDenominator().
bool IsValidFBBRAmplitudeMode(const std::string& value);

enum class FBBRGateTraceMode {
  kOff,
  kRoundOnly,
  kSampledPacing,
  kFull,
};

enum class FBBRPacingBaseSource {
  kNativeBbr,
  kBeq,
  kMaxBwFlatTrial,
  kWaveformCruiseBaseline,
};

enum class WaveformCruiseState {
  kDisabled,
  kWaitInitialSettle,
  kCollectCycle,
  kExtendCycle,
  kAnalyzeCycle,
  kWaitPostAdjustmentSettle,
  kBeqLocked,
  kFallbackLocked,
};

enum class WaveformClassification {
  kFullLoad,
  kUnderload,
  kOverload,
  kInconclusive,
};

struct FBBRFlowConfig {
  bool has_modulation_freq_hz = false;
  double modulation_freq_hz = 5.0;
  bool has_fixed_amplitude_mbps = false;
  double fixed_amplitude_mbps = 50.0;
};

struct FBBRConfig {
  double default_modulation_freq_hz = 5.0;
  std::string default_amplitude_mode = "fixed_mbps";
  double default_fixed_amplitude_mbps = 50.0;
  double pacing_minimum_rate_mbps = 0.2;
  std::map<uint32_t, FBBRFlowConfig> flow;

  double stability_single_round_exit_threshold = 0.25;
  double stability_consecutive_exit_threshold = 0.15;
  uint32_t stability_stable_rounds = 3;
  double stability_full_pipe_growth_threshold = 1.25;

  bool beq_clear_on_cruise_start = true;

  double waveform_initial_window_periods = 2.0;
  double waveform_extended_window_periods = 3.0;
  double waveform_max_window_periods = 3.0;
  double waveform_period_tolerance_ratio = 0.15;
  double waveform_min_periodicity_correlation = 0.50;
  double waveform_min_cycle_coverage_ratio = 0.85;
  double waveform_masked_min_cycle_coverage_ratio = 0.50;
  double waveform_local_slope_window_period_ratio = 0.05;
  double waveform_min_local_slope_window_ms = 5.0;
  double waveform_clip_min_duration_ratio = 0.15;
  double waveform_clip_min_half_overlap_ratio = 0.75;
  double waveform_clip_max_slope_ratio = 0.10;
  double waveform_delta_drate_amplitude_ratio = 0.50;
  double waveform_delta_fallback_baseline_ratio = 0.25;
  double waveform_plateau_max_level_span_ratio = 0.15;
  // Active obvious-clipping checks in the time-waveform control path.
  double waveform_plateau_extreme_distance_ratio = 0.15;
  uint32_t waveform_max_baseline_adjustments = 8;
  double waveform_inconclusive_signal_amplification_factor = 1.25;
  double waveform_inconclusive_signal_amplification_max_ratio = 2.0;
  double waveform_max_app_limited_sample_ratio = 0.25;
  double waveform_max_interpolation_gap_period_ratio = 0.10;

  // FBBR uses generalized Goertzel at the injected
  // frequency.  This is |X(f0)|^2 / (N * sum((x - mean(x))^2)).
  double goertzel_min_coherent_power_ratio = 0.10;

  // FBBR quantified regime classifier and actuator.
  double fbbr_regime_long_top_horizontal_duration_ratio = 0.20;
  double fbbr_regime_long_bottom_horizontal_duration_ratio = 0.30;

  double waveform_activity_amplitude_noise_multiplier = 6.0;
  double waveform_activity_min_level_ratio = 0.02;
  double waveform_activity_step_noise_multiplier = 3.0;
  double waveform_activity_min_normalized_step_slope = 3.5;
  uint32_t waveform_activity_min_active_steps = 4;
  double waveform_activity_min_active_step_ratio = 0.10;
  double waveform_activity_min_directional_change_ratio = 0.20;
  double waveform_activity_min_significant_path_ratio = 0.80;
  uint32_t waveform_activity_min_slope_reversals = 1;

  double waveform_horizontal_continuous_min_duration_ratio = 0.15;
  double waveform_horizontal_min_valid_coverage_ratio = 0.85;
  double waveform_horizontal_min_flat_fraction = 0.90;
  double waveform_horizontal_max_local_slope_ratio = 0.10;
  double waveform_horizontal_min_side_slope_ratio = 0.25;
  double waveform_horizontal_min_boundary_kink_ratio = 0.25;
  double waveform_horizontal_max_level_span_ratio = 0.10;
  double waveform_horizontal_max_total_drift_ratio = 0.05;
  double waveform_horizontal_min_side_change_ratio = 0.10;
  double waveform_horizontal_amplitude_noise_multiplier = 6.0;
  double waveform_horizontal_level_span_noise_multiplier = 4.0;
  double waveform_horizontal_slope_noise_multiplier = 3.0;
  double waveform_horizontal_extreme_distance_ratio = 0.10;

  double waveform_repeated_clip_max_period_error_ratio = 0.15;
  double waveform_repeated_clip_max_level_delta_ratio = 0.05;
  double waveform_repeated_clip_contact_level_tolerance_ratio = 0.05;
  uint32_t waveform_repeated_clip_min_contact_samples_per_cycle = 2;
  uint32_t waveform_repeated_clip_min_total_contact_samples = 4;
  double waveform_repeated_clip_min_contact_sample_ratio = 0.05;
  double waveform_repeated_clip_min_pooled_flat_fraction = 0.90;
  double waveform_repeated_clip_min_verified_boundary_fraction = 0.75;
  double waveform_repeated_clip_min_outside_excursion_ratio = 0.10;
  double waveform_repeated_clip_min_extrapolated_overshoot_ratio = 0.05;
  double waveform_repeated_clip_merge_gap_ratio = 0.025;
  double waveform_repeated_clip_max_missing_gap_ratio = 0.05;

  double waveform_shoulder_min_half_overlap_ratio = 0.75;
  double waveform_shoulder_min_side_change_ratio = 0.15;
  double waveform_shoulder_max_residual_cycle_period_error_ratio = 0.20;
  double waveform_shoulder_min_residual_cycle_leg_duration_ratio = 0.15;

  double waveform_middle_min_duration_ratio = 0.05;
  double waveform_middle_max_duration_ratio = 0.35;
  double waveform_middle_context_duration_ratio = 0.10;
  double waveform_middle_min_trend_slope_ratio = 0.20;
  double waveform_middle_max_context_slope_delta_ratio = 0.75;
  double waveform_middle_min_slope_mismatch_ratio = 0.50;
  double waveform_middle_min_mismatching_sample_ratio = 0.25;
  uint32_t waveform_middle_min_mismatching_samples = 2;
  uint32_t waveform_middle_min_consecutive_mismatching_samples = 2;
  double waveform_middle_min_bridge_deviation_ratio = 0.05;
  double waveform_middle_noise_multiplier = 3.0;
  double waveform_middle_max_mask_ratio_per_cycle = 0.35;

  double fbbr_regime_period_tolerance_ratio = 0.20;
  double fbbr_regime_min_periodicity_correlation = 0.50;

  std::string trace_gate_trace_mode = "round_only";
  uint64_t trace_gate_trace_sample_interval_us = 10000;
  bool trace_enable_cruise_window_trace = true;
  bool trace_enable_beq_selection_trace = true;
};

class QUIC_EXPORT_PRIVATE FBBRSender : public Bbr2Sender {
 public:
  FBBRSender(QuicTime now,
                 const RttStats* rtt_stats,
                 const QuicUnackedPacketMap* unacked_packets,
                 QuicPacketCount initial_cwnd_in_packets,
                 QuicPacketCount max_cwnd_in_packets,
                 Random* random,
                 QuicConnectionStats* stats,
                 bool enable_ecn = false,
                 CongestionControlType congestion_control_type = kFBBR,
                 bool enable_probe_rtt = true);
  ~FBBRSender() override = default;

  void SetOscillationFrequency(double freq_hz);
  void SetOscillationAmplitude(FBBRAmplitudeMode mode,
                               uint64_t fixed_bps = 0,
                               uint32_t sr_denominator = 1);
  void SetRecvSignalMode(bool use_delivery_rate_latest);
  void SetFairShareBandwidthBps(uint64_t fair_share_bps);
  void SetConvergenceGateTraceEnabled(bool enabled);
  void SetConvergenceGateControlEnabled(bool enabled);
  void ConfigureFBBR(const FBBRConfig& config);
  void SetTraceFlowId(uint32_t flow_id);
  void SetGateTraceMode(FBBRGateTraceMode mode,
                        uint64_t sample_interval_us);
  void FinalizeFbbrTrace();

  // A compact read-only control-plane snapshot used by Test 3 diagnostics.
  struct ExperimentState {
    bool beq_valid = false;
    QuicBandwidth beq = QuicBandwidth::Zero();
    QuicBandwidth injection_baseline = QuicBandwidth::Zero();
    std::string beq_source = "none";
    std::string waveform_last_action = "none";
  };
  ExperimentState ExportExperimentState() const;

  CongestionControlType GetCongestionControlType() const override {
    return Bbr2Sender::GetCongestionControlType();
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

  void OnApplicationLimited(QuicByteCount bytes_in_flight) override;
  void OnUpdateEcnBytes(uint64_t ecn_ce_count) override;
  bool MarkProbeRttAppLimited() const override { return true; }

  QuicBandwidth PacingRate(QuicByteCount bytes_in_flight) const override;
  QuicByteCount GetCongestionWindow() const override;

  int32_t GetCurrentBbrModeIndex() const override;

  typedef std::function<void(double window_start_s,
                             double window_end_s,
                             double p_underload,
                             double p_full_load,
                             double p_overload,
                             double confidence,
                             const std::string& label,
                             bool low_confidence,
                             const std::string& diagnostics)> CruiseLoadTraceCallback;
  void SetCruiseLoadTraceCallback(CruiseLoadTraceCallback cb) {
    cruise_load_trace_cb_ = cb;
  }

  typedef std::function<void(double time_s,
                             uint64_t native_pacing_bps,
                             uint64_t final_pacing_bps,
                             uint64_t current_native_bw_bps,
                             uint64_t pacing_base_bw_bps,
                             const std::string& pacing_base_source,
                             double phase_pacing_gain,
                             bool should_oscillate,
                             bool beq_valid)> PacingAuditTraceCallback;
  void SetPacingAuditTraceCallback(PacingAuditTraceCallback cb) {
    pacing_audit_trace_cb_ = cb;
  }

 private:
  struct AckWindowSample {
    QuicTime time;
    QuicByteCount acked_bytes;
    bool has_loss;
  };

  struct BeqSelectionResult {
    QuicBandwidth native_bw;
    QuicBandwidth beq;
    bool beq_valid;
    double beq_conf;
    const char* beq_source;
  };


  struct ResampledWaveformSeries {
    std::vector<double> values;
    std::vector<bool> valid;
    double coverage_ratio = 0.0;
  };

  struct GoertzelComponentResult {
    bool input_valid = false;
    bool component_present = false;
    double target_frequency_hz = 0.0;
    double real = 0.0;
    double imag = 0.0;
    double phase_rad = 0.0;
    double power = 0.0;
    double amplitude = 0.0;
    double coherent_power_ratio = 0.0;
    const char* decision_reason = "INVALID_INPUT";
  };

  struct CycleCompletenessResult {
    bool valid = false;
    double coverage_ratio = 0.0;
    double estimated_period = 0.0;
    double period_error_ratio = std::numeric_limits<double>::infinity();
    double periodicity_correlation = -1.0;
    bool has_peak = false;
    bool has_trough = false;
    double rising_duration_ratio = 0.0;
    double falling_duration_ratio = 0.0;
    double turning_point_score = 0.0;
    double monotonicity_score = 0.0;
    double completeness_score = 0.0;
    std::string invalid_reason = "not_analyzed";
  };

  struct TemplateFitResult {
    bool valid = false;
    double alpha = 0.0;
    double beta = 0.0;
    double fitted_response_amplitude = 0.0;
    double robust_noise_sigma = 0.0;
    double response_snr = 0.0;
  };

  struct PlateauDetectionResult {
    bool valid = false;
    bool positive_half_clipped = false;
    bool negative_half_clipped = false;
    bool srtt_only_negative_half = false;
    bool srtt_only_positive_half = false;
    bool top_clip = false;
    bool bottom_clip = false;
    bool ambiguous = false;
    bool drate_positive_half_clipped = false;
    bool drate_negative_half_clipped = false;
    bool drate_only_negative_half = false;
    bool drate_clip_ambiguous = false;
    bool positive_half_clips_simultaneous = false;
    bool srtt_middle_sequential_plateau = false;
    bool drate_middle_sequential_plateau = false;
    bool drate_middle_any_plateau = false;
    bool drate_has_waveform = false;
    size_t plateau_candidate_count = 0;
    size_t middle_sequential_candidate_count = 0;
    std::vector<bool> srtt_middle_sequential_mask;
    std::vector<bool> drate_middle_sequential_mask;
    double plateau_start = 0.0;
    double plateau_end = 0.0;
    double plateau_duration_ratio = 0.0;
    double plateau_mean = 0.0;
    double plateau_level_span_ratio = 0.0;
    double plateau_extreme_distance_ratio = 1.0;
    double plateau_abs_slope = 0.0;
    double srtt_positive_half_span_ms = 0.0;
    double srtt_negative_half_span_ms = 0.0;
    double drate_positive_half_span_bps = 0.0;
    double drate_negative_half_span_bps = 0.0;
    double shoulder_slope_before = 0.0;
    double shoulder_slope_after = 0.0;
    double other_shoulder_slope_before = 0.0;
    double other_shoulder_slope_after = 0.0;
    bool shoulders_opposite = false;
    double shoulder_change_before = 0.0;
    double shoulder_change_after = 0.0;
    double minimum_shoulder_change = 0.0;
    double phase_at_plateau_start = 0.0;
    double phase_at_plateau_end = 0.0;
    double half_overlap_ratio = 0.0;
    std::string invalid_reason = "not_analyzed";
  };

  struct BicClippingDetectionResult {
    bool valid = false;
    bool top_clip = false;
    bool bottom_clip = false;
    bool both_clipped = false;
    size_t top_motif_count = 0;
    size_t bottom_motif_count = 0;
    size_t selected_segment_count = 0;
    double selected_score = std::numeric_limits<double>::infinity();
    double top_clip_min_rounded_bic_margin = 0.0;
    double bottom_clip_min_rounded_bic_margin = 0.0;
    double top_clip_combined_rounded_bic_margin = 0.0;
    double bottom_clip_combined_rounded_bic_margin = 0.0;
    size_t top_clip_pair_sharp_motif_count = 0;
    size_t bottom_clip_pair_sharp_motif_count = 0;
    std::string invalid_reason = "not_analyzed";
  };

  enum class PeriodicSimilarityResult {
    kMatch,
    kNoMatch,
    kInvalidInput,
  };

  enum class SrttClipCase {
    kNone,
    kU1PositiveShoulder,
    kU2LongTopLine,
    kU3RepeatedTopClip,
    kL1NegativeShoulder,
    kL2LongBottomLine,
    kL3RepeatedBottomClip,
  };

  struct ContinuousHorizontalEvidence {
    bool valid = false;
    bool is_upper = false;
    bool is_lower = false;
    bool touches_left_edge = false;
    bool touches_right_edge = false;
    bool left_boundary_verified = false;
    bool right_boundary_verified = false;
    size_t start_index = 0;
    size_t end_index = 0;
    double start_s = 0.0;
    double end_s = 0.0;
    double duration_ratio_of_period = 0.0;
    double level = 0.0;
    double flat_fraction = 0.0;
    double level_span_ratio = 0.0;
    double robust_slope = 0.0;
    double left_context_slope = 0.0;
    double right_context_slope = 0.0;
    double bic_linear_minus_constant = 0.0;
  };

  struct RepeatedClipLineEvidence {
    bool valid = false;
    bool is_upper = false;
    double clip_level = 0.0;
    uint32_t contact_fragment_count = 0;
    uint32_t contact_sample_count = 0;
    uint8_t contact_cycle_mask = 0;
    double contact_sample_ratio = 0.0;
    double contact_time_span_ratio_of_window = 0.0;
    double pooled_flat_fraction = 0.0;
    double contact_level_span_ratio = 0.0;
    double cross_cycle_level_delta_ratio = 0.0;
    double event_period_error_ratio =
        std::numeric_limits<double>::infinity();
    double verified_boundary_fraction = 0.0;
    double extrapolated_overshoot_ratio = 0.0;
  };

  struct MiddleSequentialEvidence {
    bool valid = false;
    size_t start_index = 0;
    size_t end_index = 0;
    double duration_ratio_of_period = 0.0;
    double left_context_slope = 0.0;
    double right_context_slope = 0.0;
    double reference_slope = 0.0;
    double slope_mismatch_ratio = 0.0;
    double bridge_deviation_ratio = 0.0;
    double score = 0.0;
  };

  struct WaveActivityFeatures {
    bool input_valid = false;
    bool has_wave = false;
    double amplitude = 0.0;
    double noise_sigma = 0.0;
    double amplitude_to_level_ratio = 0.0;
    double step_threshold = 0.0;
    double active_step_ratio = 0.0;
    double up_change_ratio = 0.0;
    double down_change_ratio = 0.0;
    double significant_path_ratio = 0.0;
    uint32_t slope_reversals = 0;
    uint8_t active_cycle_mask = 0;
    const char* failure_reason = "INVALID_INPUT";
  };

  struct SignalRegimeFeatures {
    bool input_valid = false;
    bool ordinary_wave_uses_raw_valid_view = false;
    WaveActivityFeatures wave;
    PeriodicSimilarityResult periodic =
        PeriodicSimilarityResult::kInvalidInput;
    bool periodic_similarity_input_valid = false;
    bool periodic_similar = false;
    bool upper_clip_periodic_veto = false;
    bool lower_clip_ignored_for_periodic = false;
    bool suspected_top_candidate = false;
    bool suspected_bottom_candidate = false;
    bool positive_shoulder = false;
    bool negative_shoulder = false;
    bool positive_shoulder_cycle_input_valid = false;
    bool negative_shoulder_cycle_input_valid = false;
    bool positive_shoulder_cycle_recognizable = false;
    bool negative_shoulder_cycle_recognizable = false;
    bool long_top_line = false;
    bool long_bottom_line = false;
    bool repeated_top_clip = false;
    bool repeated_bottom_clip = false;
    bool left_edge_line_masked = false;
    bool right_edge_line_masked = false;
    bool middle_sequential_masked = false;
    uint32_t continuous_horizontal_count = 0;
    double longest_top_line_ratio_of_period = 0.0;
    double longest_bottom_line_ratio_of_period = 0.0;
    double edge_mask_ratio = 0.0;
    double middle_mask_ratio = 0.0;
    double middle_best_slope_mismatch_ratio = 0.0;
    double middle_best_bridge_deviation_ratio = 0.0;
    double estimated_period_s = 0.0;
    double estimated_srate_period_s = 0.0;
    double response_srate_period_error_ratio =
        std::numeric_limits<double>::infinity();
    double periodicity_correlation = -1.0;
    RepeatedClipLineEvidence top_repeated_clip;
    RepeatedClipLineEvidence bottom_repeated_clip;
  };

  struct FbbrRegimeFeatures {
    bool input_valid = false;
    SignalRegimeFeatures srtt;
    SignalRegimeFeatures drate;
    SrttClipCase selected_clip_case = SrttClipCase::kNone;
    bool both_clip_directions = false;
    bool clip_candidate_rejected_to_wave_fallback = false;
    bool fallback_entered = false;
    bool srtt_stats_valid = false;
    double srtt_min_ms = 0.0;
    double srtt_mean_ms = 0.0;
    double srtt_max_ms = 0.0;
    bool inflight_bdp_valid = false;
    QuicByteCount inflight_bytes = 0;
    QuicByteCount bdp_bytes = 0;
    bool drate_stats_valid = false;
    double mindrate_bps = 0.0;
    double maxdrate_bps = 0.0;
    double meandrate_bps = 0.0;
    double estimated_srate_period_s = 0.0;
  };

  struct FbbrRegimeContext {
    bool max_srtt_valid = false;
    double max_srtt_ms = 0.0;
    bool min_rtt_valid = false;
    double min_rtt_ms = 0.0;
  };

  struct FbbrRegimeDecision {
    WaveformClassification classification =
        WaveformClassification::kInconclusive;
    const char* rule_id = "";
    bool refresh_min_rtt = false;
  };

  struct FbbrActuatorResult {
    bool valid = false;
    bool update_baseline = false;
    double next_baseline_bps = 0.0;
    bool update_beq = false;
    double beq_bps = 0.0;
    bool regime_iii_mindrate_triggered = false;
    bool regime_iii_minbw_midpoint_triggered = false;
    double regime_iii_minbw_midpoint_bps = 0.0;
    bool regime_iii_decrease_triggered = false;
    bool regime_i_maxdrate_triggered = false;
    bool regime_i_maxbw_midpoint_triggered = false;
    double regime_i_maxbw_midpoint_bps = 0.0;
    bool regime_i_growth_triggered = false;
  };

  struct FbbrMaxSrttObservation {
    QuicTime update_time = QuicTime::Zero();
    TimeDelta half_window = TimeDelta::Zero();
    QuicBandwidth max_bw = QuicBandwidth::Zero();
  };

  struct WaveformWindowAnalysis {
    QuicTime probe_epoch_start = QuicTime::Zero();
    TimeDelta probe_epoch_rtt = TimeDelta::Zero();
    QuicTime collection_window_start = QuicTime::Zero();
    QuicTime collection_window_end = QuicTime::Zero();
    double collection_window_periods = 0.0;
    QuicTime window_start = QuicTime::Zero();
    QuicTime window_end = QuicTime::Zero();
    double window_periods = 0.0;
    bool extended_window = false;
    bool analysis_uses_later_cycle = false;
    bool prior_cycle_srtt_input_valid = false;
    bool prior_cycle_srtt_similar = false;
    bool prior_cycle_drate_input_valid = false;
    bool prior_cycle_drate_similar = false;
    WaveformClassification prior_cycle_classification =
        WaveformClassification::kInconclusive;
    size_t sender_sample_count = 0;
    size_t drate_sample_count = 0;
    size_t srtt_sample_count = 0;
    size_t srtt_stat_sample_count = 0;
    bool srtt_stats_valid = false;
    double srtt_mean_ms = 0.0;
    double srtt_min_ms = 0.0;
    double srtt_max_ms = 0.0;
    bool overload_queue_sample_valid = false;
    size_t overload_queue_sample_count = 0;
    double overload_q90_s = 0.0;
    double overload_queue_gradient_raw = 0.0;
    double overload_queue_gradient_noise = 0.0;
    double overload_queue_gradient = 0.0;
    size_t delivery_rate_stat_sample_count = 0;
    bool delivery_rate_stats_valid = false;
    double delivery_rate_min_bps = 0.0;
    double delivery_rate_max_bps = 0.0;
    double delivery_rate_mean_bps = 0.0;
    double latest_beq_bps = 0.0;
    double smoothed_beq_bps = 0.0;
    double coverage_ratio = 0.0;
    double app_limited_ratio = 0.0;
    bool sender_waveform_valid = false;
    GoertzelComponentResult sender_goertzel;
    GoertzelComponentResult drate_goertzel;
    bool goertzel_component_match = false;
    double best_lag_s = 0.0;
    bool drate_input_valid = false;
    double drate_ncc = 0.0;
    double drate_slope_direction_agreement = 0.0;
    CycleCompletenessResult drate_completeness;
    TemplateFitResult drate_fit;
    bool drate_similar = false;
    bool drate_similar_without_middle = false;
    bool drate_effective_similar = false;
    bool drate_match = false;
    bool srtt_input_valid = false;
    double srtt_direct_ncc = 0.0;
    double srtt_integral_ncc = 0.0;
    double srtt_derivative_ncc = 0.0;
    double srtt_slope_direction_agreement = 0.0;
    CycleCompletenessResult srtt_completeness;
    TemplateFitResult srtt_fit;
    bool srtt_similar_frequency = false;
    bool srtt_similar = false;
    bool srtt_similar_without_middle = false;
    bool srtt_effective_similar = false;
    bool srtt_cycle_complete = false;
    bool srtt_positive_half_clipped = false;
    bool srtt_negative_half_clipped = false;
    bool srtt_only_negative_half = false;
    bool srtt_only_positive_half = false;
    bool srtt_clip_ambiguous = false;
    BicClippingDetectionResult bic_clipping;
    bool true_bottom_clip_min_rtt_refresh_applied = false;
    double true_bottom_clip_min_rtt_before_ms = 0.0;
    double true_bottom_clip_min_rtt_after_ms = 0.0;
    double true_bottom_clip_min_rtt_timestamp_before_s = 0.0;
    double true_bottom_clip_min_rtt_timestamp_after_s = 0.0;
    double true_bottom_clip_probe_rtt_deadline_after_s = 0.0;
    bool srtt_match = false;
    bool fbbr_pipeline = false;
    FbbrRegimeFeatures fbbr_features;
    FbbrRegimeDecision fbbr_decision;
    WaveformClassification unsuppressed_classification =
        WaveformClassification::kInconclusive;
    bool no_wave_triggered = false;
    bool wave_fidelity_just_entered = false;
    bool wave_fidelity_enhancement_active = false;
    bool classification_suppressed_for_retry = false;
    bool state_updates_suppressed_for_retry = false;
    uint8_t retry_reason_mask = 0;
    uint8_t srtt_no_wave_streak = 0;
    uint8_t drate_no_wave_streak = 0;
    uint64_t window_first_cycle_id = 0;
    uint64_t window_second_cycle_id = 0;
    double max_srtt_before_ms = 0.0;
    double max_srtt_after_ms = 0.0;
    bool fbbr_max_bw_before_valid = false;
    double fbbr_max_bw_before_bps = 0.0;
    bool fbbr_max_bw_after_valid = false;
    double fbbr_max_bw_after_bps = 0.0;
    bool model_min_rtt_valid = false;
    double model_min_rtt_ms = 0.0;
    bool fbbr_max_srtt_valid = false;
    double fbbr_max_srtt_ms = 0.0;
    bool fbbr_regime_i_maxdrate_triggered = false;
    bool fbbr_regime_i_maxbw_midpoint_valid = false;
    double fbbr_regime_i_maxbw_midpoint_bps = 0.0;
    bool fbbr_regime_i_maxbw_midpoint_triggered = false;
    bool fbbr_regime_i_growth_triggered = false;
    PlateauDetectionResult plateau;
    CycleCompletenessResult drate_without_middle_completeness;
    CycleCompletenessResult srtt_without_middle_completeness;
    double boundary_lift_time_s = 0.0;
    double boundary_sender_phase = 0.0;
    double boundary_rate_bps = 0.0;
    double boundary_delta_bps = 0.0;
    bool boundary_found = false;
    bool cycle_detected_but_incomplete = false;
    double current_drate_response_amplitude_bps = 0.0;
    const char* delta_source = "NONE";
    double raw_delta_bw_bps = 0.0;
    double applied_delta_bw_bps = 0.0;
    double delta_reference_bps = 0.0;
    double window_extreme_gap_bps = 0.0;
    double actuator_step_multiplier = 1.0;
    double queue_delay_ms = 0.0;
    double queue_delay_min_rtt_ratio = -1.0;
    uint32_t overload_confirmation_count = 0;
    WaveformClassification classification =
        WaveformClassification::kInconclusive;
    const char* decision_rule = "R6";
    std::string invalid_reason = "none";
  };

  void OnProbeBwPhaseEntered(Bbr2ProbeBwMode::CyclePhase phase,
                             QuicTime now) override;
  bool HasCustomProbeDownLogic() const override;
  bool ShouldExitCustomProbeDown(QuicByteCount bytes_in_flight,
                                 QuicByteCount bdp) const override;
  bool ShouldDelayProbeBwCruiseExit(QuicTime now) const override;
  float GetProbeBwPacingGain(Bbr2ProbeBwMode::CyclePhase phase,
                             float pacing_gain) const override;
  float GetProbeBwCwndGain(Bbr2ProbeBwMode::CyclePhase phase,
                           float cwnd_gain) const override;
  void OnCongestionEventStarted(
      const Bbr2CongestionEvent& congestion_event) override;
  void UpdateNativeMinRtt(TimeDelta min_rtt,
                          QuicTime source_time,
                          bool from_probe_rtt);
  void ResetBeqForNativeMinRttRise();

  Bbr2ProbeBwMode::CyclePhase GetCurrentProbeBwPhase() const;
  bool ShouldStartMaxBwFlatTrial() const;
  void StartMaxBwFlatTrial(QuicTime now);
  void UpdateMaxBwFlatTrial(const Bbr2CongestionEvent& congestion_event);
  void FinishMaxBwFlatTrial(QuicTime now,
                             bool accepted,
                             const char* reason);
  void ResetMaxBwFlatTrialState();
  bool IsMaxBwFlatTrialPacingActive() const;
  bool HasActiveWaveformCycle(QuicTime now) const;
  bool IsWaveformCycleBoundary(QuicTime now) const;
  QuicTime NextWaveformCycleBoundary(QuicTime now) const;
  bool BaseShouldOscillate() const;
  bool ShouldOscillate() const;
  uint64_t GetCurrentAmplitudeBps() const;
  void RefreshProbeAmplitudeProtection();
  bool IsProbeAmplitudeProtectionActive() const;
  bool IsProbeAmplitudeSuppressedByFloor() const;
  double TriangleWave(QuicTime now) const;

  void EnterCruise(QuicTime now);
  void LeaveCruise(QuicTime now);
  static bool ShouldEnableMinRttProbeDown(
      bool previous_cruise_min_rtt_updated,
      QuicByteCount bytes_in_flight,
      QuicByteCount bdp);
  static bool ShouldExitMinRttProbeDown(QuicByteCount bytes_in_flight,
                                        QuicByteCount bdp);
  void FinalizeCruise(QuicTime now);
  void ResetWaveformCruiseState(QuicTime now);
  void RunWaveformCruiseStateMachine(QuicTime now);
  bool IsFbbr() const;
  bool HasUsableFbbrPreviousBeq() const;
  bool ComputeFbbrTimeWeightedSrttMeanMs(
      QuicTime window_start,
      QuicTime window_end,
      double* srtt_mean_ms) const;
  void ApplyFbbrClassification(
      const WaveformWindowAnalysis& analysis,
      QuicTime now);
  WaveformWindowAnalysis AnalyzeFbbrWindow(
      QuicTime window_start,
      QuicTime window_end,
      double window_periods,
      bool extended_window) const;
  void UpdateFbbrRetryState(WaveformWindowAnalysis* analysis);
  void ApplyFbbrRegimeStateUpdates(
      WaveformWindowAnalysis* trace_analysis,
      QuicTime now);
  void RefreshMinRttFromTrueBottomClip(WaveformWindowAnalysis* analysis,
                                       QuicTime now);
  void ScheduleWaveformCollectionAfterSettle(QuicTime now,
                                             bool initial_settle);
  void StartWaveformCollectionAt(QuicTime cycle_start,
                                 double window_periods,
                                 bool extended_window);
  void ApplyWaveformClassification(const WaveformWindowAnalysis& analysis,
                                   QuicTime now);
  bool AmplifyWaveformProbeAfterInconclusive(
      const WaveformWindowAnalysis& analysis,
      QuicTime now);
  WaveformWindowAnalysis AnalyzeWaveformWindow(QuicTime window_start,
                                               QuicTime window_end,
                                               double window_periods,
                                               bool extended_window) const;
  static double ComputeMaxBwAttenuationFactor(
      double delivery_center_bps,
      double actual_fluctuation_amplitude_bps);
  static double ComputeMinBwCorrectionFactor(
      double delivery_center_bps,
      double actual_fluctuation_amplitude_bps);
  uint64_t CurrentEmittedProbeAmplitudeBps() const;
  double CurrentActualDeliveryFluctuationAmplitudeBps() const;
  double CurrentMaxBwAttenuationFactor() const;
  double CurrentMinBwCorrectionFactor() const;
  void ResetMaxBwAttenuationEstimator();
  void UpdateMaxBwAttenuationEstimator(
      double delivery_center_bps,
      double actual_fluctuation_amplitude_bps,
      double emitted_fluctuation_amplitude_bps);
  void UpdateMaxBwAttenuationFromWaveform(
      const WaveformWindowAnalysis& analysis);
  TimeDelta CurrentFbbrMaxSrttObservationRtt() const;
  bool ComputeFbbrMaxSrttAround(QuicTime center_time,
                                  TimeDelta half_window,
                                  double* max_srtt_ms,
                                  size_t* sample_count) const;
  void RecordFbbrMaxBwUpdateForMaxSrtt(QuicTime event_time,
                                         QuicBandwidth max_bw);
  void FinalizePendingFbbrMaxSrttObservations(QuicTime now);
  void EmitWaveformSearchTrace(const WaveformWindowAnalysis& analysis,
                               const std::string& action,
                               double baseline_before_bps,
                               double amplitude_before_bps) const;
  void PublishWaveformBeq();
  void PublishFbbrCruiseBeq(QuicTime now);
  void PublishBeqSelection(const BeqSelectionResult& selection);
  void EmitCruiseSummaryTrace(QuicTime now) const;
  void UpdateRoundDeliveryRateSample(
      const Bbr2CongestionEvent& congestion_event);
  void FinalizeCompletedRound(
      const Bbr2CongestionEvent& congestion_event);
  bool CheckExitStable(QuicBandwidth completed_d_round,
                       bool completed_d_valid,
                       double v_round);
  void UpdateReconvergenceEvidence(QuicBandwidth completed_d_round,
                                   bool completed_d_valid);
  void UpdateFreqWeightAndToolState();
  void ClearBeq(const char* reason);
  void ClearBeqApplication(const char* reason) const;
  bool IsBeqApplicationPhase(Bbr2ProbeBwMode::CyclePhase phase) const;
  bool IsCruisePhase(Bbr2ProbeBwMode::CyclePhase phase) const;
  static const char* PhaseApplicationName(Bbr2ProbeBwMode::CyclePhase phase);
  static const char* PacingBaseSourceName(FBBRPacingBaseSource source);
  void EmitConvergenceGateTrace(
      const Bbr2CongestionEvent& congestion_event,
      QuicBandwidth completed_d_round,
      bool completed_d_valid,
      QuicBandwidth previous_d_round,
      bool previous_d_valid,
      double v_round,
      bool v_round_valid,
      double previous_v_round,
      bool just_exited) const;
  void EmitPacingTrace(QuicBandwidth b_native,
                       QuicBandwidth pacing_base_bw,
                       FBBRPacingBaseSource pacing_base_source,
	                       QuicBandwidth native_pacing,
	                       QuicBandwidth final_pacing,
	                       double phase_pacing_gain,
	                       int64_t modulation_amp_bps,
	                       int64_t modulation_amp_eff_bps,
                       double triangle_wave,
                       bool actual_modulation_on) const;
  void EmitFreqGateCsvRow(const char* row_type,
                          QuicTime event_time,
                          QuicBandwidth completed_d_round,
                          bool completed_d_valid,
                          QuicBandwidth previous_d_round,
                          bool previous_d_valid,
                          double v_round,
                          double previous_v_round,
                          bool just_exited,
                          QuicBandwidth b_native,
	                          QuicBandwidth pacing_base_bw,
	                          FBBRPacingBaseSource pacing_base_source,
	                          QuicBandwidth native_pacing,
	                          QuicBandwidth final_pacing,
	                          double phase_pacing_gain,
	                          int64_t modulation_amp_bps,
                          int64_t modulation_amp_eff_bps,
                          double triangle_wave,
                          QuicBandwidth current_delivery_rate,
                          bool sample_is_app_limited,
                          bool sample_valid) const;

  std::vector<FBBRRateSample> SelectRateSamples(
      const std::deque<FBBRRateSample>& history,
      QuicTime start,
      QuicTime end) const;
  std::vector<FBBRRttSample> SelectRttSamples(
      const std::deque<FBBRRttSample>& history,
      QuicTime start,
      QuicTime end) const;
  std::vector<double> ResampleRateSeries(
      const std::vector<FBBRRateSample>& samples,
      QuicTime start,
      QuicTime end,
      double sample_step_s) const;
  std::vector<double> ResampleRttSeries(
      const std::vector<FBBRRttSample>& samples,
      QuicTime start,
      QuicTime end,
      double sample_step_s) const;
  ResampledWaveformSeries ResampleRateWaveform(
      const std::vector<FBBRRateSample>& samples,
      QuicTime start,
      QuicTime end,
      double sample_step_s,
      double max_interpolation_gap_s) const;
  ResampledWaveformSeries ResampleRttWaveform(
      const std::vector<FBBRRttSample>& samples,
      QuicTime start,
      QuicTime end,
      double sample_step_s,
      double max_interpolation_gap_s) const;
  GoertzelComponentResult AnalyzeGoertzelComponent(
      const ResampledWaveformSeries& series,
      double sample_step_s,
      double target_frequency_hz) const;
  static double ComputeNormalizedCrossCorrelation(
      const std::vector<double>& lhs,
      const std::vector<double>& rhs,
      const std::vector<bool>& valid);
  static double ComputeSlopeDirectionAgreement(
      const std::vector<double>& lhs,
      const std::vector<double>& rhs,
      const std::vector<bool>& valid);
  static double ComputeLaggedCorrelation(
      const std::vector<double>& values,
      const std::vector<bool>& valid,
      size_t lag_samples,
      size_t* pair_count);
  static bool HasMacroOpposingShoulders(
      double slope_before,
      double slope_after,
      double shoulder_duration_s,
      double minimum_abs_slope,
      double minimum_signal_change);
  static bool HasMacroSameDirectionShoulders(double slope_before,
                                             double slope_after);
  static bool HasDualMacroOpposingShoulders(
      double first_before,
      double first_after,
      double first_minimum_abs_slope,
      double first_minimum_signal_change,
      double second_before,
      double second_after,
      double second_minimum_abs_slope,
      double second_minimum_signal_change,
      double shoulder_duration_s);
  static std::vector<double> RobustNormalize(
      const std::vector<double>& values,
      const std::vector<bool>& valid);
  static TemplateFitResult EstimateRobustNoise(
      const std::vector<double>& response,
      const std::vector<double>& waveform_template,
      const std::vector<bool>& valid);
  CycleCompletenessResult AnalyzeCycleCompleteness(
      const std::vector<double>& values,
      const std::vector<bool>& valid,
      double sample_step_s,
      double expected_period_s,
      double minimum_coverage_ratio = -1.0) const;
  PlateauDetectionResult DetectDualSignalPlateaus(
      const std::vector<double>& srtt,
      const std::vector<double>& srtt_slopes,
      const std::vector<double>& drate,
      const std::vector<double>& drate_slopes,
      const std::vector<double>& sender_residual,
      const std::vector<bool>& valid,
      double sample_step_s,
      double period_s,
      double srtt_noise_sigma,
      double drate_noise_sigma) const;
  bool LocateBoundaryLiftPoint(
      const PlateauDetectionResult& plateau,
      const std::vector<double>& srtt_slopes,
      const std::vector<bool>& valid,
      QuicTime receiver_window_start,
      double sample_step_s,
      double best_lag_s,
      double* lift_time_s,
      double* sender_phase,
      double* boundary_rate_bps,
      double* boundary_delta_bps) const;
  static std::vector<double> DetrendLinear(
      const std::vector<double>& values,
      const std::vector<bool>& valid);
  static std::vector<double> MedianFilter3(
      const std::vector<double>& values,
      const std::vector<bool>& valid);
  static std::vector<double> ComputeLocalLinearSlopes(
      const std::vector<double>& values,
      const std::vector<bool>& valid,
      double sample_step_s,
      double slope_window_s);
  static std::vector<double> BuildQueueIntegralTemplate(
      const std::vector<double>& sender_residual,
      const std::vector<bool>& valid,
      double sample_step_s,
      double period_s);
  QuicTime AlignToNextTriangleCycle(QuicTime time) const;
  TimeDelta CurrentSmoothedRtt() const;
  static const char* WaveformStateName(WaveformCruiseState state);
  static const char* WaveformClassificationName(
      WaveformClassification classification);
  struct DeliveryRateWindowStats {
    size_t sample_count = 0;
    bool valid = false;
    double min_bps = 0.0;
    double max_bps = 0.0;
    double mean_bps = 0.0;
  };
  static DeliveryRateWindowStats ComputeDeliveryRateWindowStats(
      const std::vector<FBBRRateSample>& samples);
  struct TimeWeightedDeliveryRateStats {
    size_t sample_count = 0;
    bool valid = false;
    double mean_bps = 0.0;
  };
  TimeWeightedDeliveryRateStats ComputeTimeWeightedDeliveryRate(
      const std::vector<FBBRRateSample>& samples,
      QuicTime window_start,
      QuicTime window_end,
      double rtprop_ms,
      bool require_srtt_range) const;
  bool ComputeFbbrCruiseFallbackBeq(QuicTime window_start,
                                    QuicTime window_end,
                                    TimeDelta rtprop,
                                    QuicBandwidth* beq,
                                    const char** source) const;
  struct SrttWindowStats {
    size_t sample_count = 0;
    bool valid = false;
    double mean_ms = 0.0;
    double min_ms = 0.0;
    double max_ms = 0.0;
  };
  static SrttWindowStats ComputeSrttWindowStats(
      const std::vector<FBBRRttSample>& samples);
  struct WaveformDecisionInputs {
    bool prechecks_valid = false;
    bool srtt_input_valid = false;
    bool srtt_window_stats_valid = false;
    double srtt_mean_ms = 0.0;
    double srtt_min_ms = 0.0;
    double srtt_max_ms = 0.0;
    bool drate_input_valid = false;
    bool srtt_similar = false;
    bool srtt_similar_without_middle = false;
    bool drate_similar = false;
    bool drate_similar_without_middle = false;
    bool srtt_positive_half_clipped = false;
    bool srtt_negative_half_clipped = false;
    bool srtt_only_negative_half = false;
    bool srtt_only_positive_half = false;
    bool bic_srtt_top_clip = false;
    bool bic_srtt_bottom_clip = false;
    bool drate_positive_half_clipped = false;
    bool drate_only_negative_half = false;
    bool positive_half_clips_simultaneous = false;
    bool drate_has_waveform = false;
    bool drate_middle_any_plateau = false;
  };
  static WaveformClassification ClassifyWaveformState(
      const WaveformDecisionInputs& inputs,
      const char** decision_rule = nullptr);
  static FbbrRegimeDecision ClassifyFbbrRegime(
      const FbbrRegimeFeatures& features,
      const FbbrRegimeContext& context);
  static FbbrActuatorResult ComputeFbbrInjectionBaseline(
      WaveformClassification classification,
      double mindrate_bps,
      double maxdrate_bps,
      double regime_ii_delivery_rate_bps,
      bool min_bw_valid,
      double min_bw_bps,
      bool max_bw_valid,
      double max_bw_bps,
      double current_baseline_bps,
      bool regime_i_or_iii_seen_this_cruise,
      double minimum_rate_bps);
  WaveActivityFeatures DetectOrdinaryWaveActivity(
      const std::vector<double>& values,
      const std::vector<bool>& valid,
      double sample_step_s,
      double period_s,
      bool allow_half_cycle_wave = false) const;
  std::vector<ContinuousHorizontalEvidence>
  DetectContinuousHorizontalSegments(
      const std::vector<double>& values,
      const std::vector<bool>& valid,
      double sample_step_s,
      double period_s) const;
  RepeatedClipLineEvidence DetectRepeatedClipLineContacts(
      const std::vector<double>& values,
      const std::vector<bool>& valid,
      double sample_step_s,
      double period_s,
      bool upper) const;
  std::vector<MiddleSequentialEvidence>
  DetectMiddleSequentialDisturbances(
      const std::vector<double>& values,
      const std::vector<bool>& valid,
      const std::vector<bool>& protected_mask,
      double sample_step_s,
      double period_s) const;
  PeriodicSimilarityResult AnalyzeFbbrPeriodicSimilarity(
      const std::vector<double>& values,
      const std::vector<bool>& original_valid,
      const std::vector<bool>& periodic_valid,
      double sample_step_s,
      double period_s,
      double srate_period_s,
      bool verified_upper_clip,
      SignalRegimeFeatures* features) const;
  static double EstimateActualSignalPeriod(
      const std::vector<double>& values,
      const std::vector<bool>& valid,
      double sample_step_s,
      double nominal_period_s,
      double* correlation);
  static bool ShouldRefreshMinRttForTrueClip(bool top_clip,
                                             bool bottom_clip);
  static BicClippingDetectionResult DetectBicSrttClipping(
      const std::vector<double>& srtt,
      const std::vector<bool>& valid,
      double noise_sigma);
  static double Clamp01(double value);
  static double LogisticScore(double value, double threshold, double slope);

  double configured_modulation_freq_hz_;
  FBBRAmplitudeMode amplitude_mode_;
  uint64_t fixed_amplitude_bps_;
  uint32_t sr_amplitude_denominator_;
  uint64_t minimum_pacing_rate_bps_;
  bool drain_completed_;
  bool in_cruise_;
  bool maxbw_flat_trial_active_;
  uint32_t maxbw_flat_good_rounds_;
  QuicBandwidth maxbw_flat_trial_rate_;
  QuicBandwidth maxbw_flat_fallback_baseline_;
  TimeDelta maxbw_flat_entry_srtt_;
  TimeDelta maxbw_flat_round_max_srtt_;
  QuicByteCount maxbw_flat_round_start_delivered_;
  QuicTime maxbw_flat_round_start_time_;
  bool maxbw_flat_round_started_;
  bool maxbw_flat_failure_pending_;
  bool maxbw_flat_ecn_failure_pending_;
  QuicTime maxbw_flat_trial_start_time_;
  TimeDelta maxbw_flat_trial_elapsed_;
  bool maxbw_flat_trial_finished_;
  mutable bool maxbw_flat_native_exit_requested_;
  mutable QuicTime maxbw_flat_native_exit_request_time_;
  mutable bool maxbw_flat_cruise_exit_delay_active_;
  mutable QuicTime maxbw_flat_cruise_exit_not_before_;
  bool maxbw_flat_waveform_epoch_pending_;
  uint64_t maxbw_flat_last_ecn_ce_count_;
  bool cruise_exit_force_immediate_;
  mutable bool cruise_exit_pending_;
  mutable QuicTime cruise_exit_cycle_end_;
  bool current_cruise_min_rtt_updated_;
  bool previous_cruise_min_rtt_updated_;
  bool min_rtt_probe_down_active_;
  TimeDelta cruise_min_rtt_at_entry_;
  QuicByteCount latest_congestion_event_prior_inflight_;
  bool latest_congestion_event_prior_inflight_valid_;
  QuicByteCount latest_congestion_event_inflight_;
  bool latest_congestion_event_inflight_valid_;
  double cruise_modulation_freq_hz_;
  QuicTime cruise_start_time_;
  mutable QuicTime current_time_;
  mutable QuicTime last_ack_time_;
  bool use_delivery_rate_latest_for_signal_history_;
  int64_t cruise_id_;
  WaveformCruiseState waveform_cruise_state_;
  QuicBandwidth initial_cruise_baseline_bw_;
  QuicBandwidth current_injection_baseline_bw_;
  // Keep the requested amplitude separate from the amount emitted in this
  // sampling window so a later baseline increase can restore the request.
  uint64_t current_probe_amplitude_bps_;
  uint64_t effective_probe_amplitude_bps_;
  uint64_t waveform_initial_probe_amplitude_bps_;
  bool probe_amplitude_capped_for_window_;
  bool probe_amplitude_suppressed_by_floor_for_window_;
  bool max_bw_response_observed_;
  double max_bw_delivery_response_gain_;
  double max_bw_observation_center_bps_;
  double max_bw_observation_baseline_bps_;
  double max_bw_actual_fluctuation_amplitude_bps_;
  double max_bw_attenuation_factor_;
  mutable double current_probe_bw_phase_gain_;
  QuicTime probe_epoch_start_time_;
  TimeDelta probe_epoch_rtt_;
  QuicTime waveform_settle_start_;
  QuicTime waveform_settle_end_;
  QuicTime waveform_window_start_;
  QuicTime waveform_window_end_;
  double waveform_window_periods_;
  bool waveform_window_extended_;
  bool underload_located_;
  bool beq_baseline_locked_;
  bool has_last_similar_drate_amplitude_;
  double last_similar_drate_amplitude_bps_;
  bool waveform_delta_reference_valid_;
  double waveform_delta_reference_bps_;
  uint32_t consecutive_overload_count_;
  const char* waveform_last_delta_source_;
  double waveform_last_raw_delta_bw_bps_;
  double waveform_last_applied_delta_bw_bps_;
  uint32_t baseline_adjustment_count_;
  uint32_t inconclusive_extension_count_;
  uint32_t waveform_inconclusive_amplification_count_;
  bool fbbr_max_srtt_valid_;
  double fbbr_max_srtt_ms_;
  uint64_t fbbr_max_srtt_source_cruise_id_;
  uint8_t fbbr_srtt_no_wave_streak_;
  uint8_t fbbr_drate_no_wave_streak_;
  bool fbbr_wave_fidelity_enhancement_active_;
  uint8_t fbbr_retry_reason_mask_;
  uint64_t fbbr_last_counted_window_second_cycle_id_;
  uint32_t fbbr_rolling_retry_count_;
  bool fbbr_regime_ii_seen_this_cruise_;
  QuicBandwidth fbbr_cruise_beq_;
  uint32_t floor_clip_confirmation_count_;
  int waveform_last_clip_direction_;
  uint32_t waveform_decision_count_;
  uint32_t waveform_amplitude_reduction_count_;
  uint32_t beq_candidate_update_count_;
  QuicBandwidth beq_candidate_;
  const char* beq_candidate_source_;
  QuicBandwidth fbbr_latest_beq_;
  QuicBandwidth fbbr_smoothed_beq_;
  bool fbbr_smoothed_beq_valid_;
  std::string waveform_last_action_;
  std::string waveform_last_invalid_reason_;

  bool waveform_negative_half_first_;
  double waveform_initial_window_periods_;
  double waveform_extended_window_periods_;
  double waveform_max_window_periods_;
  double waveform_period_tolerance_ratio_;
  double waveform_min_periodicity_correlation_;
  double waveform_min_cycle_coverage_ratio_;
  double waveform_masked_min_cycle_coverage_ratio_;
  double waveform_local_slope_window_period_ratio_;
  double waveform_min_local_slope_window_ms_;
  double waveform_clip_min_duration_ratio_;
  double waveform_clip_min_half_overlap_ratio_;
  double waveform_clip_max_slope_ratio_;
  double waveform_delta_drate_amplitude_ratio_;
  double waveform_delta_fallback_baseline_ratio_;
  double waveform_plateau_max_level_span_ratio_;
  double waveform_plateau_extreme_distance_ratio_;
  uint32_t waveform_max_baseline_adjustments_;
  uint32_t waveform_max_inconclusive_extensions_;
  double waveform_inconclusive_signal_amplification_factor_;
  double waveform_inconclusive_signal_amplification_max_ratio_;
  double waveform_max_app_limited_sample_ratio_;
  double waveform_max_interpolation_gap_period_ratio_;
  double goertzel_min_coherent_power_ratio_;

  double fbbr_regime_long_top_horizontal_duration_ratio_;
  double fbbr_regime_long_bottom_horizontal_duration_ratio_;
  uint32_t fbbr_wave_fidelity_no_wave_trigger_windows_;
  uint32_t fbbr_wave_fidelity_retry_window_advance_periods_;
  double waveform_activity_amplitude_noise_multiplier_;
  double waveform_activity_min_level_ratio_;
  double waveform_activity_step_noise_multiplier_;
  double waveform_activity_min_normalized_step_slope_;
  uint32_t waveform_activity_min_active_steps_;
  double waveform_activity_min_active_step_ratio_;
  double waveform_activity_min_directional_change_ratio_;
  double waveform_activity_min_significant_path_ratio_;
  uint32_t waveform_activity_min_slope_reversals_;
  double waveform_horizontal_continuous_min_duration_ratio_;
  double waveform_horizontal_min_valid_coverage_ratio_;
  double waveform_horizontal_min_flat_fraction_;
  double waveform_horizontal_max_local_slope_ratio_;
  double waveform_horizontal_min_side_slope_ratio_;
  double waveform_horizontal_min_boundary_kink_ratio_;
  double waveform_horizontal_max_level_span_ratio_;
  double waveform_horizontal_max_total_drift_ratio_;
  double waveform_horizontal_min_side_change_ratio_;
  double waveform_horizontal_amplitude_noise_multiplier_;
  double waveform_horizontal_level_span_noise_multiplier_;
  double waveform_horizontal_slope_noise_multiplier_;
  double waveform_horizontal_extreme_distance_ratio_;
  double waveform_repeated_clip_max_period_error_ratio_;
  double waveform_repeated_clip_max_level_delta_ratio_;
  double waveform_repeated_clip_contact_level_tolerance_ratio_;
  uint32_t waveform_repeated_clip_min_contact_samples_per_cycle_;
  uint32_t waveform_repeated_clip_min_total_contact_samples_;
  double waveform_repeated_clip_min_contact_sample_ratio_;
  double waveform_repeated_clip_min_contact_span_ratio_of_window_;
  double waveform_repeated_clip_min_pooled_flat_fraction_;
  double waveform_repeated_clip_min_verified_boundary_fraction_;
  double waveform_repeated_clip_min_outside_excursion_ratio_;
  double waveform_repeated_clip_min_extrapolated_overshoot_ratio_;
  double waveform_repeated_clip_merge_gap_ratio_;
  double waveform_repeated_clip_max_missing_gap_ratio_;
  double waveform_shoulder_min_half_overlap_ratio_;
  double waveform_shoulder_min_side_change_ratio_;
  double waveform_shoulder_max_residual_cycle_period_error_ratio_;
  double waveform_shoulder_min_residual_cycle_leg_duration_ratio_;
  double waveform_middle_min_duration_ratio_;
  double waveform_middle_max_duration_ratio_;
  double waveform_middle_context_duration_ratio_;
  double waveform_middle_min_trend_slope_ratio_;
  double waveform_middle_max_context_slope_delta_ratio_;
  double waveform_middle_min_slope_mismatch_ratio_;
  double waveform_middle_min_mismatching_sample_ratio_;
  uint32_t waveform_middle_min_mismatching_samples_;
  uint32_t waveform_middle_min_consecutive_mismatching_samples_;
  double waveform_middle_min_bridge_deviation_ratio_;
  double waveform_middle_noise_multiplier_;
  double waveform_middle_max_mask_ratio_per_cycle_;
  double fbbr_regime_period_tolerance_ratio_;
  double fbbr_regime_min_periodicity_correlation_;
  bool fbbr_regime_periodic_upper_clip_is_hard_veto_;

  uint64_t fair_share_bandwidth_bps_;

  std::deque<FBBRRateSample> sender_rate_history_;
  std::deque<FBBRRateSample> delivery_rate_history_;
  std::deque<FBBRRttSample> srtt_history_;
  std::deque<AckWindowSample> ack_window_history_;
  std::deque<FbbrMaxSrttObservation>
      pending_fbbr_max_srtt_observations_;
  bool cruise_freq_tool_active_;

  CruiseLoadTraceCallback cruise_load_trace_cb_;
  PacingAuditTraceCallback pacing_audit_trace_cb_;

  bool bbr_stable_;
  uint32_t stable_cnt_;
  QuicBandwidth d_round_;
  QuicBandwidth d_prev_;
  bool d_round_valid_;
  bool d_prev_valid_;
  QuicBandwidth full_drate_ref_;
  bool full_drate_ref_valid_;
  double prev_v_round_;
  bool freq_tool_needed_;
  mutable bool freq_tool_on_;
  QuicBandwidth beq_;
  bool beq_valid_;
  double beq_conf_;
  const char* beq_source_;
  uint64_t beq_cruise_id_;
  mutable bool beq_fresh_;
  mutable bool beq_application_valid_;
  mutable bool beq_ready_for_post_cruise_;
  mutable const char* beq_application_phase_;
  mutable bool beq_cleared_on_cruise_start_;
  bool native_max_bw_pacing_after_min_rtt_rise_;
  bool native_probe_rtt_reset_active_;
  TimeDelta native_probe_rtt_min_rtt_before_;
  mutable std::string beq_invalid_reason_;
  uint64_t unstable_episode_id_;
  bool unstable_episode_active_;
  mutable QuicBandwidth selection_native_bw_;
  mutable double drate_spectral_integrity_score_;
  mutable double srtt_spectral_integrity_score_;
  mutable double joint_spectral_integrity_score_;
  mutable bool drate_spectral_gate_pass_;
  mutable bool srtt_spectral_gate_pass_;
  mutable bool dual_signal_spectral_gate_pass_;
  mutable const char* limiting_spectral_signal_;
  mutable bool merged_rescue_attempted_;
  mutable bool merged_rescue_success_;
  mutable uint64_t beq_selection_compute_us_;
  mutable size_t normal_window_count_;
  mutable size_t merged_window_count_;
  mutable size_t spectral_invalid_count_;
  bool enable_convergence_gate_trace_;
  bool enable_convergence_gate_control_;
  bool beq_clear_on_cruise_start_;
  double stable_single_round_exit_threshold_;
  double stable_consecutive_exit_threshold_;
  uint32_t stable_rounds_;
  double stable_full_pipe_growth_threshold_;
  uint32_t trace_flow_id_;
  FBBRGateTraceMode gate_trace_mode_;
  TimeDelta gate_trace_sample_interval_;
  mutable QuicTime last_pacing_gate_trace_time_;


  // FBBR no longer enforces an inflight service envelope; the related
  // histories, telemetry snapshots and emitters were removed.
  bool fbbr_regime_i_or_iii_seen_this_cruise_;

  static constexpr size_t kMaxHistorySamples = 20000;
  static constexpr uint32_t kStableRounds = 3;
  static constexpr double kDefaultOscillationFreqHz = 1.0;
  static constexpr double kSampleStepSec = 0.001;
};

}  // namespace dqc

#endif  // FBBR_SENDER_H_
