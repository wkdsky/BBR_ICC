// FreqCCv4 - BBRv2 with fixed-frequency CRUISE-only rate modulation.

#ifndef FREQCCV4_SENDER_H_
#define FREQCCV4_SENDER_H_

#include <cstdint>
#include <deque>
#include <functional>
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

 private:
  struct SpectrumProfile {
    double peak_freq_hz;
    double band_peak_rel;
    double band_energy_rel;
    double target_amp;
    double noise_floor;
    bool noise_floor_valid;
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
    int full_load_rank_in_cruise;
    bool is_best_full_load_window;
    bool low_confidence;
    std::string label;
  };

  void OnProbeBwPhaseEntered(Bbr2ProbeBwMode::CyclePhase phase,
                             QuicTime now) override;

  Bbr2ProbeBwMode::CyclePhase GetCurrentProbeBwPhase() const;
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
  void FinalizeCruise(QuicTime now);
  void EmitCruiseWindowTrace(const CruiseWindowResult& result);
  void EmitCruiseSummaryTrace(QuicTime now) const;

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

  std::deque<FreqCCv4RateSample> sender_rate_history_;
  std::deque<FreqCCv4RateSample> delivery_rate_history_;
  std::deque<FreqCCv4RttSample> srtt_history_;
  std::deque<AckWindowSample> ack_window_history_;
  std::vector<CruiseWindowResult> current_cruise_windows_;

  CruiseLoadTraceCallback cruise_load_trace_cb_;

  static constexpr size_t kMaxHistorySamples = 20000;
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
