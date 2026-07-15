// FreqCCv4 - BBRv2 with fixed-frequency CRUISE-only rate modulation.

#ifndef FREQCCV4_SENDER_H_
#define FREQCCV4_SENDER_H_

#include <cstdint>
#include <deque>
#include <functional>
#include <limits>
#include <map>
#include <ostream>
#include <string>
#include <vector>

#include "quic_bbr2_sender.h"
#include "quic_export.h"

namespace dqc {

struct FreqCCv4RateSample {
  QuicTime time;
  QuicBandwidth rate;
  bool valid = true;
  bool is_app_limited = false;
  QuicByteCount acked_bytes = 0;
};

struct FreqCCv4RttSample {
  QuicTime time;
  double rtt_ms;
};

enum class FreqCCv4AmplitudeMode {
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
};

enum class FreqCCv4GateTraceMode {
  kOff,
  kRoundOnly,
  kSampledPacing,
  kFull,
};

enum class FreqCCv4PacingBaseSource {
  kNativeBbr,
  kTrustedBw,
  kWaveformCruiseBaseline,
};

enum class FreqCCv4CruiseDetectorMode {
  kTimeWaveform,
  kLegacySpectral,
};

enum class WaveformCruiseState {
  kDisabled,
  kWaitInitialSettle,
  kCollectCycle,
  kExtendCycle,
  kAnalyzeCycle,
  kWaitPostAdjustmentSettle,
  kTrustedLocked,
  kFallbackLocked,
};

enum class WaveformClassification {
  kFullLoad,
  kUnderload,
  kOverload,
  kInconclusive,
};

struct FreqBbrFlowConfig {
  bool has_modulation_freq_hz = false;
  double modulation_freq_hz = 5.0;
  bool has_fixed_amplitude_mbps = false;
  double fixed_amplitude_mbps = 50.0;
};

struct FreqBbrConfig {
  double default_modulation_freq_hz = 5.0;
  std::string default_amplitude_mode = "fixed_mbps";
  double default_fixed_amplitude_mbps = 50.0;
  double pacing_minimum_rate_mbps = 1.0;
  std::map<uint32_t, FreqBbrFlowConfig> flow;

  double stability_single_round_exit_threshold = 0.25;
  double stability_consecutive_exit_threshold = 0.15;
  uint32_t stability_stable_rounds = 3;
  double stability_full_pipe_growth_threshold = 1.25;

  double spectral_drate_integrity_threshold = 0.25;
  double spectral_srtt_integrity_threshold = 0.25;
  double spectral_min_drate_snr = 1.5;
  double spectral_min_srtt_snr = 1.5;
  double spectral_max_drate_width_ratio = 2.0;
  double spectral_max_srtt_width_ratio = 2.5;
  double spectral_min_drate_phase_coherence = 0.5;
  double spectral_min_srtt_phase_coherence = 0.5;
  double spectral_freq_sigma_ratio = 0.08;
  double spectral_snr_slope = 2.0;
  double spectral_energy_threshold = 0.10;
  double spectral_energy_slope = 20.0;
  double spectral_width_r0_drate = 1.5;
  double spectral_width_r0_srtt = 2.0;
  double spectral_width_sigma = 0.8;

  bool merged_rescue_enable = true;
  double merged_rescue_window_multiplier = 2.0;
  uint32_t merged_rescue_max_passes = 1;
  double merged_rescue_max_trend_ratio = 0.20;
  double merged_rescue_confidence_discount = 0.8;

  bool trusted_bw_clear_on_cruise_start = true;

  std::string cruise_detector_mode = "time_waveform";
  std::string waveform_recv_signal_mode = "delivery_rate_latest";
  double waveform_initial_settle_rtt_mult = 1.0;
  double waveform_post_adjust_settle_rtt_mult = 1.0;
  bool waveform_negative_half_first = true;
  double waveform_initial_window_periods = 2.0;
  double waveform_extended_window_periods = 3.0;
  double waveform_max_window_periods = 3.0;
  double waveform_period_tolerance_ratio = 0.15;
  double waveform_min_periodicity_correlation = 0.50;
  double waveform_min_cycle_coverage_ratio = 0.85;
  double waveform_masked_min_cycle_coverage_ratio = 0.50;
  double waveform_min_completeness_score = 0.60;
  double waveform_min_rising_duration_ratio = 0.15;
  double waveform_min_falling_duration_ratio = 0.15;
  double waveform_min_shape_ncc = 0.35;
  double waveform_min_slope_direction_agreement = 0.65;
  double waveform_min_response_snr = 2.0;
  double waveform_local_slope_window_period_ratio = 0.05;
  double waveform_min_local_slope_window_ms = 5.0;
  double waveform_clip_min_duration_ratio = 0.15;
  double waveform_clip_min_half_overlap_ratio = 0.75;
  double waveform_clip_max_slope_ratio = 0.10;
  double waveform_delta_drate_amplitude_ratio = 0.50;
  double waveform_delta_fallback_baseline_ratio = 0.25;
  double waveform_adaptive_delta_fallback_baseline_ratio = 0.10;
  double waveform_delta_ewma_alpha = 0.125;
  double waveform_delta_min_baseline_ratio = 0.02;
  double waveform_delta_max_baseline_ratio = 0.15;
  double waveform_overload_max_delta_multiplier = 6.0;
  double waveform_underload_max_delta_multiplier = 2.0;
  uint32_t waveform_overload_confirmations = 2;
  bool waveform_queue_guard_enabled = true;
  double waveform_queue_low_min_rtt_ratio = 0.10;
  double waveform_queue_target_min_rtt_ratio = 0.25;
  double waveform_queue_high_min_rtt_ratio = 0.75;

  // Parsed for configuration compatibility only unless noted otherwise.
  double waveform_min_drate_ncc = 0.50;
  double waveform_min_srtt_integral_ncc = 0.45;
  double waveform_min_srtt_derivative_ncc = 0.45;
  double waveform_plateau_min_duration_ratio = 0.10;
  double waveform_plateau_max_slope_ratio = 0.20;
  double waveform_plateau_max_level_span_ratio = 0.15;
  // Active obvious-clipping guards in the time-waveform control path.
  double waveform_plateau_extreme_distance_ratio = 0.15;
  double waveform_baseline_step_ratio = 0.25;
  double waveform_amplitude_floor_ratio = 0.125;
  uint32_t waveform_clip_floor_confirmations = 2;
  uint32_t waveform_max_baseline_adjustments = 8;
  uint32_t waveform_max_inconclusive_extensions = 1;
  double waveform_max_app_limited_sample_ratio = 0.25;
  double waveform_max_interpolation_gap_period_ratio = 0.10;

  std::string trace_gate_trace_mode = "round_only";
  uint64_t trace_gate_trace_sample_interval_us = 10000;
  bool trace_enable_cruise_window_trace = true;
  bool trace_enable_trusted_bw_selection_trace = true;
};

class QUIC_EXPORT_PRIVATE FreqCCv4Sender final : public Bbr2Sender {
 public:
  FreqCCv4Sender(QuicTime now,
                 const RttStats* rtt_stats,
                 const QuicUnackedPacketMap* unacked_packets,
                 QuicPacketCount initial_cwnd_in_packets,
                 QuicPacketCount max_cwnd_in_packets,
                 Random* random,
                 QuicConnectionStats* stats,
                 bool enable_ecn = false,
                 bool hybrid_window_baseline = false,
                 bool adaptive_guard = false);
  ~FreqCCv4Sender() override = default;

  void SetOscillationFrequency(double freq_hz);
  void SetOscillationAmplitude(FreqCCv4AmplitudeMode mode,
                               uint64_t fixed_bps = 0);
  void SetRecvSignalMode(bool use_delivery_rate_latest);
  void SetCruiseWindowConfig(double min_cycles_per_window,
                             double window_step_ratio);
  void SetFairShareBandwidthBps(uint64_t fair_share_bps);
  void SetCruiseBaselineCapBps(uint64_t cap_bps);
  void SetConvergenceGateTraceEnabled(bool enabled);
  void SetConvergenceGateControlEnabled(bool enabled);
  void ConfigureFreqBbr(const FreqBbrConfig& config);
  void SetTraceFlowId(uint32_t flow_id);
  void SetGateTraceMode(FreqCCv4GateTraceMode mode,
                        uint64_t sample_interval_us);
  static bool RunConvergenceGateStateMachineSelfTest(std::ostream& os);
  static bool RunTrustedBwSelectionSelfTest(std::ostream& os);
  static bool RunTrustedBwPacingSelfTest(std::ostream& os);
  static bool RunWaveformCruiseSelfTest(std::ostream& os);
  static bool RunHybridBaselineSelfTest(std::ostream& os);

  CongestionControlType GetCongestionControlType() const override {
    return hybrid_window_baseline_enabled_
               ? kFreqCCv4Hybrid
               : (adaptive_guard_enabled_ ? kFreqCCv4Adaptive : kFreqCCv4);
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
                             bool trusted_bw_valid)> PacingAuditTraceCallback;
  void SetPacingAuditTraceCallback(PacingAuditTraceCallback cb) {
    pacing_audit_trace_cb_ = cb;
  }

 private:
  struct SpectrumProfile {
    double peak_freq_hz;
    double band_peak_rel;
    double band_energy_rel;
    double target_amp;
    double noise_floor;
    bool noise_floor_valid;
    double peak_width_hz;
    double freq_step_hz;
    std::vector<double> band_shape;
    bool valid;
  };

  struct WindowSignalResult {
    SpectrumProfile profile;
    double mean_value;
    std::vector<double> values;
    bool valid;
  };

  struct AckWindowSample {
    QuicTime time;
    QuicByteCount acked_bytes;
    bool has_loss;
  };

  struct CycleQualityMetrics {
    double waveform_quality;
    double top_clip_ratio;
    double bottom_clip_ratio;
    double distortion_score;
    double cycle_frequency_stability;
    double cycle_phase_stability;
    int valid_cycles;
    bool phase_reliable;
  };

  struct CruiseWindowResult {
    int64_t cruise_id;
    QuicTime window_start;
    QuicTime window_end;
    double configured_modulation_freq_hz;
    double srate_peak_freq_hz;
    double drate_peak_freq_hz;
    double srtt_peak_freq_hz;
    double drate_mean_kbps;
    double srate_freq_score;
    double drate_freq_score;
    double srtt_freq_score;
    double freq_quality;
    double drate_target_amp;
    double drate_noise_floor;
    double drate_snr;
    double srate_target_amp;
    double drate_gain;
    double drate_amplitude_score;
    double srtt_target_amp;
    double srtt_noise_floor;
    double srtt_snr;
    double srtt_amplitude_score;
    double amplitude_quality;
    double drate_waveform_quality;
    double srtt_waveform_quality;
    double waveform_quality;
    double cycle_frequency_stability;
    double cycle_phase_stability;
    double consistency_quality;
    double srtt_top_clip_ratio;
    double srtt_bottom_clip_ratio;
    double srtt_distortion_score;
    bool srate_valid;
    bool drate_valid;
    bool srtt_valid;
    size_t srate_sample_count;
    size_t drate_sample_count;
    size_t srtt_sample_count;
    int valid_cycle_count;
    bool is_full_load_candidate;
    double full_load_quality;
    double full_load_quality_v1;
    double full_load_quality_v2;
    double drate_band_energy_rel;
    double srtt_band_energy_rel;
    double drate_band_peak_rel;
    double srtt_band_peak_rel;
    double srate_peak_width_hz;
    double drate_peak_width_hz;
    double srtt_peak_width_hz;
    double drate_width_ratio;
    double srtt_width_ratio;
    double drate_phase_coherence;
    double srtt_phase_coherence;
    double drate_spectral_integrity_score;
    double srtt_spectral_integrity_score;
    double joint_spectral_integrity_score;
    bool drate_spectral_gate_pass;
    bool srtt_spectral_gate_pass;
    bool dual_signal_spectral_gate_pass;
    const char* limiting_spectral_signal;
    std::string spectral_invalid_reason;
    std::string window_source;
    int full_load_rank_in_cruise;
    bool is_best_full_load_window;
    bool low_confidence;
    std::string label;
  };

  struct TrustedBwSelectionResult {
    QuicBandwidth native_bw;
    QuicBandwidth trusted_bw;
    bool trusted_bw_valid;
    double trusted_bw_conf;
    const char* trusted_bw_source;
    double drate_spectral_integrity_score;
    double srtt_spectral_integrity_score;
    double joint_spectral_integrity_score;
    bool drate_spectral_gate_pass;
    bool srtt_spectral_gate_pass;
    bool dual_signal_spectral_gate_pass;
    const char* limiting_spectral_signal;
    bool merged_rescue_attempted;
    bool merged_rescue_success;
    uint64_t trusted_bw_selection_compute_us;
    size_t normal_window_count;
    size_t merged_window_count;
    size_t spectral_invalid_count;
  };

  struct ResampledWaveformSeries {
    std::vector<double> values;
    std::vector<bool> valid;
    double coverage_ratio = 0.0;
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
    bool top_clip = false;
    bool bottom_clip = false;
    bool ambiguous = false;
    bool drate_positive_half_clipped = false;
    bool drate_negative_half_clipped = false;
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
    size_t delivery_rate_stat_sample_count = 0;
    bool delivery_rate_stats_valid = false;
    double delivery_rate_min_bps = 0.0;
    double delivery_rate_max_bps = 0.0;
    double delivery_rate_mean_bps = 0.0;
    double latest_trusted_bw_bps = 0.0;
    double smoothed_trusted_bw_bps = 0.0;
    double coverage_ratio = 0.0;
    double app_limited_ratio = 0.0;
    bool sender_waveform_valid = false;
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
    bool srtt_clip_ambiguous = false;
    bool srtt_match = false;
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
  float GetProbeBwCwndGain(Bbr2ProbeBwMode::CyclePhase phase,
                           float cwnd_gain) const override;
  void OnCongestionEventStarted(
      const Bbr2CongestionEvent& congestion_event) override;

  Bbr2ProbeBwMode::CyclePhase GetCurrentProbeBwPhase() const;
  bool BaseShouldOscillate() const;
  bool ShouldOscillate() const;
  uint64_t GetCurrentAmplitudeBps() const;
  double TriangleWave(QuicTime now) const;

  void EnterCruise(QuicTime now);
  void LeaveCruise(QuicTime now);
  void ResetCruiseWindowState();
  void RunDueCruiseWindowAnalysis(QuicTime now);
  void AnalyzeCruiseWindow(QuicTime window_start,
                           QuicTime window_end,
                           TimeDelta min_rtt,
                           double window_duration_s);
  CruiseWindowResult BuildCruiseWindowResult(QuicTime window_start,
                                             QuicTime window_end,
                                             TimeDelta min_rtt,
                                             double window_duration_s,
                                             const std::string& window_source);
  void FinalizeCruise(QuicTime now);
  void ResetWaveformCruiseState(QuicTime now);
  void RunWaveformCruiseStateMachine(QuicTime now);
  void ScheduleWaveformCollectionAfterSettle(QuicTime now,
                                             bool initial_settle);
  void StartWaveformCollectionAt(QuicTime cycle_start,
                                 double window_periods,
                                 bool extended_window);
  void ApplyWaveformClassification(const WaveformWindowAnalysis& analysis,
                                   QuicTime now);
  WaveformWindowAnalysis AnalyzeWaveformWindow(QuicTime window_start,
                                               QuicTime window_end,
                                               double window_periods,
                                               bool extended_window) const;
  void EmitWaveformSearchTrace(const WaveformWindowAnalysis& analysis,
                               const std::string& action,
                               double baseline_before_bps,
                               double amplitude_before_bps) const;
  void PublishWaveformTrustedBw();
  TrustedBwSelectionResult RunTrustedBwSelection(QuicTime now);
  void PublishTrustedBwSelection(const TrustedBwSelectionResult& selection);
  void RankCruiseWindows(const CruiseWindowResult* selected_window);
  void EmitCruiseWindowTrace(const CruiseWindowResult& result);
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
  void ClearTrustedBw(const char* reason);
  void ClearTrustedBwApplication(const char* reason) const;
  bool IsReliableSpectralWindow(const CruiseWindowResult& result) const;
  double ComputeRateTrendRatio(QuicTime start, QuicTime end) const;
  bool IsTrustedBwApplicationPhase(Bbr2ProbeBwMode::CyclePhase phase) const;
  bool IsCruisePhase(Bbr2ProbeBwMode::CyclePhase phase) const;
  static const char* PhaseApplicationName(Bbr2ProbeBwMode::CyclePhase phase);
  static const char* PacingBaseSourceName(FreqCCv4PacingBaseSource source);
  double ComputePhaseCoherence(const std::vector<double>& values,
                               double sample_step_s,
                               double ref_freq_hz,
                               bool* valid) const;
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
                       FreqCCv4PacingBaseSource pacing_base_source,
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
	                          FreqCCv4PacingBaseSource pacing_base_source,
	                          QuicBandwidth native_pacing,
	                          QuicBandwidth final_pacing,
	                          double phase_pacing_gain,
	                          int64_t modulation_amp_bps,
                          int64_t modulation_amp_eff_bps,
                          double triangle_wave,
                          QuicBandwidth current_delivery_rate,
                          bool sample_is_app_limited,
                          bool sample_valid) const;

  std::vector<FreqCCv4RateSample> SelectRateSamples(
      const std::deque<FreqCCv4RateSample>& history,
      QuicTime start,
      QuicTime end) const;
  std::vector<FreqCCv4RttSample> SelectRttSamples(
      const std::deque<FreqCCv4RttSample>& history,
      QuicTime start,
      QuicTime end) const;
  std::vector<double> ResampleRateSeries(
      const std::vector<FreqCCv4RateSample>& samples,
      QuicTime start,
      QuicTime end,
      double sample_step_s) const;
  std::vector<double> ResampleRttSeries(
      const std::vector<FreqCCv4RttSample>& samples,
      QuicTime start,
      QuicTime end,
      double sample_step_s) const;
  ResampledWaveformSeries ResampleRateWaveform(
      const std::vector<FreqCCv4RateSample>& samples,
      QuicTime start,
      QuicTime end,
      double sample_step_s,
      double max_interpolation_gap_s) const;
  ResampledWaveformSeries ResampleRttWaveform(
      const std::vector<FreqCCv4RttSample>& samples,
      QuicTime start,
      QuicTime end,
      double sample_step_s,
      double max_interpolation_gap_s) const;
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
      const std::vector<FreqCCv4RateSample>& samples);
  static double SmoothTrustedBandwidth(double historical_smoothed_bps,
                                       bool historical_valid,
                                       double latest_trusted_bps);
  struct WaveformDecisionInputs {
    bool prechecks_valid = false;
    bool srtt_input_valid = false;
    bool drate_input_valid = false;
    bool srtt_similar = false;
    bool srtt_similar_without_middle = false;
    bool drate_similar = false;
    bool drate_similar_without_middle = false;
    bool srtt_positive_half_clipped = false;
    bool srtt_negative_half_clipped = false;
    bool drate_positive_half_clipped = false;
    bool positive_half_clips_simultaneous = false;
    bool drate_has_waveform = false;
    bool drate_middle_any_plateau = false;
  };
  static WaveformClassification ClassifyWaveformState(
      const WaveformDecisionInputs& inputs,
      const char** decision_rule = nullptr);
  static const char* CruiseDetectorModeName(
      FreqCCv4CruiseDetectorMode mode);

  WindowSignalResult AnalyzeRateSeries(
      const std::vector<FreqCCv4RateSample>& samples,
      QuicTime start,
      QuicTime end,
      double reference_freq_hz,
      bool detrend) const;
  WindowSignalResult AnalyzeRttSeries(
      const std::vector<FreqCCv4RttSample>& samples,
      QuicTime start,
      QuicTime end,
      double reference_freq_hz,
      bool detrend) const;
  SpectrumProfile BuildSpectrumProfile(const std::vector<double>& values,
                                       double sample_step_s,
                                       double ref_freq_hz) const;
  double ComputeSpectrumShapeDistance(const std::vector<double>& lhs,
                                      const std::vector<double>& rhs) const;
  CycleQualityMetrics AnalyzeCycleQuality(
      const std::vector<double>& values,
      double sample_step_s,
      double ref_freq_hz,
      bool estimate_waveform_distortion) const;
  double ComputeFreqScore(double peak_freq_hz,
                          double reference_freq_hz,
                          double freq_tolerance_hz) const;
  double ComputeCongestionScore(QuicTime window_start,
                                QuicTime window_end) const;

  static double Clamp01(double value);
  static double ScoreThreshold(double value, double min_value, double target);
  static double ExpFreqScore(double delta_f, double sigma_f);
  static double LogisticScore(double value, double threshold, double slope);
  static double WidthScore(double width_ratio, double r0, double sigma);
  static const char* LabelToString(int label);

  const bool hybrid_window_baseline_enabled_;
  const bool adaptive_guard_enabled_;
  double configured_modulation_freq_hz_;
  FreqCCv4AmplitudeMode amplitude_mode_;
  uint64_t fixed_amplitude_bps_;
  uint64_t minimum_pacing_rate_bps_;
  bool drain_completed_;
  bool in_cruise_;
  double cruise_modulation_freq_hz_;
  QuicTime cruise_start_time_;
  QuicTime next_cruise_window_start_;
  mutable QuicTime current_time_;
  mutable QuicTime last_ack_time_;
  bool use_delivery_rate_latest_for_signal_history_;
  bool min_rtt_warning_logged_;
  int64_t cruise_id_;
  FreqCCv4CruiseDetectorMode cruise_detector_mode_;
  WaveformCruiseState waveform_cruise_state_;
  QuicBandwidth initial_cruise_baseline_bw_;
  QuicBandwidth current_injection_baseline_bw_;
  uint64_t current_probe_amplitude_bps_;
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
  bool trusted_baseline_locked_;
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
  uint32_t floor_clip_confirmation_count_;
  int waveform_last_clip_direction_;
  uint32_t waveform_decision_count_;
  uint32_t waveform_amplitude_reduction_count_;
  uint32_t trusted_bw_candidate_update_count_;
  QuicBandwidth trusted_bw_candidate_;
  const char* trusted_bw_candidate_source_;
  QuicBandwidth hybrid_latest_trusted_bw_;
  QuicBandwidth hybrid_smoothed_trusted_bw_;
  bool hybrid_smoothed_trusted_bw_valid_;
  QuicBandwidth adaptive_trusted_bw_history_;
  bool adaptive_trusted_bw_history_valid_;
  std::string waveform_last_action_;
  std::string waveform_last_invalid_reason_;

  double waveform_initial_settle_rtt_mult_;
  double waveform_post_adjust_settle_rtt_mult_;
  bool waveform_negative_half_first_;
  double waveform_initial_window_periods_;
  double waveform_extended_window_periods_;
  double waveform_max_window_periods_;
  double waveform_period_tolerance_ratio_;
  double waveform_min_periodicity_correlation_;
  double waveform_min_cycle_coverage_ratio_;
  double waveform_masked_min_cycle_coverage_ratio_;
  double waveform_min_completeness_score_;
  double waveform_min_rising_duration_ratio_;
  double waveform_min_falling_duration_ratio_;
  double waveform_min_shape_ncc_;
  double waveform_min_slope_direction_agreement_;
  double waveform_min_drate_ncc_;
  double waveform_min_srtt_integral_ncc_;
  double waveform_min_srtt_derivative_ncc_;
  double waveform_min_response_snr_;
  double waveform_local_slope_window_period_ratio_;
  double waveform_min_local_slope_window_ms_;
  double waveform_clip_min_duration_ratio_;
  double waveform_clip_min_half_overlap_ratio_;
  double waveform_clip_max_slope_ratio_;
  double waveform_delta_drate_amplitude_ratio_;
  double waveform_delta_fallback_baseline_ratio_;
  double waveform_adaptive_delta_fallback_baseline_ratio_;
  double waveform_delta_ewma_alpha_;
  double waveform_delta_min_baseline_ratio_;
  double waveform_delta_max_baseline_ratio_;
  double waveform_overload_max_delta_multiplier_;
  double waveform_underload_max_delta_multiplier_;
  uint32_t waveform_overload_confirmations_;
  bool waveform_queue_guard_enabled_;
  double waveform_queue_low_min_rtt_ratio_;
  double waveform_queue_target_min_rtt_ratio_;
  double waveform_queue_high_min_rtt_ratio_;
  double waveform_plateau_min_duration_ratio_;
  double waveform_plateau_max_slope_ratio_;
  double waveform_plateau_max_level_span_ratio_;
  double waveform_plateau_extreme_distance_ratio_;
  double waveform_baseline_step_ratio_;
  double waveform_amplitude_floor_ratio_;
  uint32_t waveform_clip_floor_confirmations_;
  uint32_t waveform_max_baseline_adjustments_;
  uint32_t waveform_max_inconclusive_extensions_;
  double waveform_max_app_limited_sample_ratio_;
  double waveform_max_interpolation_gap_period_ratio_;

  double min_cruise_cycles_per_window_;
  double cruise_window_step_ratio_;
  double freq_tolerance_ratio_;
  double min_full_load_quality_for_reliable_window_;
  double default_ecn_congestion_ratio_;
  uint64_t fair_share_bandwidth_bps_;
  uint64_t cruise_baseline_cap_bps_;

  std::deque<FreqCCv4RateSample> sender_rate_history_;
  std::deque<FreqCCv4RateSample> delivery_rate_history_;
  std::deque<FreqCCv4RttSample> srtt_history_;
  std::deque<AckWindowSample> ack_window_history_;
  std::vector<CruiseWindowResult> current_cruise_windows_;
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
  QuicBandwidth trusted_bw_;
  bool trusted_bw_valid_;
  double trusted_bw_conf_;
  const char* trusted_bw_source_;
  uint64_t trusted_bw_cruise_id_;
  mutable bool trusted_bw_fresh_;
  mutable bool trusted_bw_application_valid_;
  mutable bool trusted_bw_ready_for_post_cruise_;
  mutable const char* trusted_bw_application_phase_;
  mutable bool trusted_bw_cleared_on_cruise_start_;
  mutable std::string trusted_bw_invalid_reason_;
  uint64_t unstable_episode_id_;
  bool unstable_episode_active_;
  double w_freq_;
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
  mutable uint64_t trusted_bw_selection_compute_us_;
  mutable size_t normal_window_count_;
  mutable size_t merged_window_count_;
  mutable size_t spectral_invalid_count_;
  bool enable_convergence_gate_trace_;
  bool enable_convergence_gate_control_;
  bool trusted_bw_clear_on_cruise_start_;
  double stable_single_round_exit_threshold_;
  double stable_consecutive_exit_threshold_;
  uint32_t stable_rounds_;
  double stable_full_pipe_growth_threshold_;
  double drate_spectral_integrity_threshold_;
  double srtt_spectral_integrity_threshold_;
  double min_drate_snr_;
  double min_srtt_snr_;
  double max_drate_width_ratio_;
  double max_srtt_width_ratio_;
  double min_drate_phase_coherence_;
  double min_srtt_phase_coherence_;
  double freq_sigma_ratio_;
  double snr_slope_;
  double energy_threshold_;
  double energy_slope_;
  double width_r0_drate_;
  double width_r0_srtt_;
  double width_sigma_;
  bool merged_rescue_enabled_;
  double merged_window_multiplier_;
  uint32_t max_merged_passes_;
  double merged_window_max_trend_ratio_;
  double merged_confidence_discount_;
  uint32_t trace_flow_id_;
  FreqCCv4GateTraceMode gate_trace_mode_;
  TimeDelta gate_trace_sample_interval_;
  mutable QuicTime last_pacing_gate_trace_time_;

  static constexpr size_t kMaxHistorySamples = 20000;
  static constexpr uint32_t kStableRounds = 3;
  static constexpr double kDefaultOscillationFreqHz = 1.0;
  static constexpr double kSampleStepSec = 0.001;
  static constexpr double kMinDrateFreqScoreForCandidate = 0.60;
  static constexpr double kMinSrttFreqScoreForCandidate = 0.60;
  static constexpr double kTargetDrateGain = 0.30;
  static constexpr double kMinSrttSnr = 1.50;
  static constexpr double kTargetSrttSnr = 3.00;
  static constexpr double kMaxDrateShapeDistance = 0.40;
  static constexpr double kMaxPhaseStdCycles = 0.25;
  static constexpr double kBandLowRatio = 0.70;
  static constexpr double kBandHighRatio = 1.30;
  static constexpr int kBandShapeBins = 16;
  static constexpr int kFftZeroPadMultiplier = 4;
};

}  // namespace dqc

#endif  // FREQCCV4_SENDER_H_
