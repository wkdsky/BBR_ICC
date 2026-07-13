// FreqCCv4 - BBRv2 with fixed-frequency CRUISE-only rate modulation.

#ifndef FREQCCV4_SENDER_H_
#define FREQCCV4_SENDER_H_

#include <cstdint>
#include <deque>
#include <functional>
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
                 bool enable_ecn = false);
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

  CongestionControlType GetCongestionControlType() const override {
    return kFreqCCv4;
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

  void OnProbeBwPhaseEntered(Bbr2ProbeBwMode::CyclePhase phase,
                             QuicTime now) override;
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

  double configured_modulation_freq_hz_;
  FreqCCv4AmplitudeMode amplitude_mode_;
  uint64_t fixed_amplitude_bps_;
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
  mutable const char* trusted_bw_invalid_reason_;
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
