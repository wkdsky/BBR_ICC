#include "fbbr_sender.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <numeric>
#include <sstream>

#include <fftw3.h>

#include "quic_bbr2_probe_bw.h"
#include "quic_logging.h"

namespace dqc {

namespace {

constexpr const char* kTrustedBwSourceNone = "NONE";
constexpr const char* kTrustedBwSourceNormal = "NORMAL_SPECTRAL";
constexpr const char* kTrustedBwSourceMerged = "MERGED_SPECTRAL";
constexpr const char* kTrustedBwSourceFBBRFrequencySearch =
    "F_BBR_LOW_QUEUE_PLATEAU";
constexpr const char* kLimitingSpectralSignalDrate = "DRATE";
constexpr const char* kLimitingSpectralSignalSrtt = "SRTT";
constexpr const char* kLimitingSpectralSignalEqual = "EQUAL";

uint64_t FbbrStableHash(uint64_t value) {
  value += 0x9e3779b97f4a7c15ULL;
  value = (value ^ (value >> 30)) * 0xbf58476d1ce4e5b9ULL;
  value = (value ^ (value >> 27)) * 0x94d049bb133111ebULL;
  return value ^ (value >> 31);
}

double ClampValue(double value, double low, double high) {
  return std::max(low, std::min(high, value));
}

double MedianOfSorted(const std::vector<double>& sorted) {
  if (sorted.empty()) {
    return 0.0;
  }
  const size_t mid = sorted.size() / 2;
  if (sorted.size() % 2 == 1) {
    return sorted[mid];
  }
  return 0.5 * (sorted[mid - 1] + sorted[mid]);
}

double Median(std::vector<double> values) {
  if (values.empty()) {
    return 0.0;
  }
  std::sort(values.begin(), values.end());
  return MedianOfSorted(values);
}

double Percentile(std::vector<double> values, double fraction) {
  if (values.empty()) return 0.0;
  std::sort(values.begin(), values.end());
  const double position = ClampValue(fraction, 0.0, 1.0) *
                          static_cast<double>(values.size() - 1);
  const size_t low = static_cast<size_t>(std::floor(position));
  const size_t high = static_cast<size_t>(std::ceil(position));
  if (low == high) return values[low];
  return values[low] + (values[high] - values[low]) * (position - low);
}

std::string AppendReason(const std::string& current, const char* reason) {
  if (current.empty() || current == "none") {
    return reason;
  }
  return current + "|" + reason;
}

QuicBandwidth BandwidthFromBps(double bps) {
  return QuicBandwidth::FromBitsPerSecond(
      static_cast<int64_t>(std::llround(std::max(0.0, bps))));
}

bool HasValidRateCoverage(const std::vector<FBBRRateSample>& samples,
                          QuicTime start,
                          QuicTime end,
                          double reference_freq_hz) {
  if (samples.size() < 4 || end <= start || reference_freq_hz <= 0.0) {
    return false;
  }
  for (size_t i = 0; i < samples.size(); ++i) {
    if (samples[i].rate.IsZero() ||
        (i > 0 && samples[i].time <= samples[i - 1].time)) {
      return false;
    }
  }
  const double duration_s =
      static_cast<double>((end - start).ToMicroseconds()) / 1000000.0;
  const double covered_s =
      static_cast<double>((samples.back().time - samples.front().time)
                              .ToMicroseconds()) /
      1000000.0;
  return duration_s * reference_freq_hz >= 2.0 &&
         covered_s * reference_freq_hz >= 2.0 &&
         covered_s / duration_s >= 0.75;
}

bool HasValidRttCoverage(const std::vector<FBBRRttSample>& samples,
                         QuicTime start,
                         QuicTime end,
                         double reference_freq_hz) {
  if (samples.size() < 4 || end <= start || reference_freq_hz <= 0.0) {
    return false;
  }
  for (size_t i = 0; i < samples.size(); ++i) {
    if (!std::isfinite(samples[i].rtt_ms) || samples[i].rtt_ms <= 0.0 ||
        (i > 0 && samples[i].time <= samples[i - 1].time)) {
      return false;
    }
  }
  const double duration_s =
      static_cast<double>((end - start).ToMicroseconds()) / 1000000.0;
  const double covered_s =
      static_cast<double>((samples.back().time - samples.front().time)
                              .ToMicroseconds()) /
      1000000.0;
  return duration_s * reference_freq_hz >= 2.0 &&
         covered_s * reference_freq_hz >= 2.0 &&
         covered_s / duration_s >= 0.75;
}

}  // namespace

constexpr size_t FBBRSender::kMaxHistorySamples;
constexpr uint32_t FBBRSender::kStableRounds;
constexpr double FBBRSender::kDefaultOscillationFreqHz;
constexpr double FBBRSender::kSampleStepSec;
constexpr double FBBRSender::kMinDrateFreqScoreForCandidate;
constexpr double FBBRSender::kMinSrttFreqScoreForCandidate;
constexpr double FBBRSender::kTargetDrateGain;
constexpr double FBBRSender::kMinSrttSnr;
constexpr double FBBRSender::kTargetSrttSnr;
constexpr double FBBRSender::kMaxDrateShapeDistance;
constexpr double FBBRSender::kMaxPhaseStdCycles;
constexpr double FBBRSender::kBandLowRatio;
constexpr double FBBRSender::kBandHighRatio;
constexpr int FBBRSender::kBandShapeBins;
constexpr int FBBRSender::kFftZeroPadMultiplier;

FBBRSender::FBBRSender(
    const ProtoClock* clock,
    QuicTime now,
    const RttStats* rtt_stats,
    const QuicUnackedPacketMap* unacked_packets,
    QuicPacketCount initial_cwnd_in_packets,
    QuicPacketCount max_cwnd_in_packets,
    Random* random,
    QuicConnectionStats* stats,
    bool enable_ecn)
    : Bbr2Sender(now,
                 rtt_stats,
                 unacked_packets,
                 initial_cwnd_in_packets,
                 max_cwnd_in_packets,
                 random,
                 stats,
                 enable_ecn,
                 nullptr,
                 kFBBR,
                 true),
	      configured_modulation_freq_hz_(5.0),
      clock_(clock),
      amplitude_mode_(FBBRAmplitudeMode::kFixed),
      fixed_amplitude_bps_(0),
      drain_completed_(false),
      in_cruise_(false),
	      cruise_modulation_freq_hz_(5.0),
      cruise_start_time_(QuicTime::Zero()),
      next_cruise_window_start_(QuicTime::Zero()),
      current_time_(now),
      last_ack_time_(QuicTime::Zero()),
      use_delivery_rate_latest_for_signal_history_(false),
      min_rtt_warning_logged_(false),
      cruise_id_(0),
      min_cruise_cycles_per_window_(4.0),
      cruise_window_step_ratio_(0.25),
      freq_tolerance_ratio_(0.20),
      min_full_load_quality_for_reliable_window_(0.50),
      default_ecn_congestion_ratio_(0.02),
	      fair_share_bandwidth_bps_(0),
	      cruise_freq_tool_active_(false),
	      bbr_stable_(true),
	      stable_cnt_(kStableRounds),
      d_round_(QuicBandwidth::Zero()),
      d_prev_(QuicBandwidth::Zero()),
      d_round_valid_(false),
      d_prev_valid_(false),
      full_drate_ref_(QuicBandwidth::Zero()),
      full_drate_ref_valid_(false),
	      prev_v_round_(0.0),
	      freq_tool_needed_(false),
	      freq_tool_on_(false),
	      trusted_bw_(QuicBandwidth::Zero()),
	      trusted_bw_valid_(false),
	      trusted_bw_conf_(0.0),
	      trusted_bw_source_(kTrustedBwSourceNone),
	      trusted_bw_cruise_id_(0),
	      trusted_bw_fresh_(false),
	      trusted_bw_application_valid_(false),
	      trusted_bw_ready_for_post_cruise_(false),
	      trusted_bw_application_phase_("NONE"),
	      trusted_bw_cleared_on_cruise_start_(false),
	      trusted_bw_invalid_reason_("none"),
      unstable_episode_id_(0),
      unstable_episode_active_(false),
	      w_freq_(0.0),
	      selection_native_bw_(QuicBandwidth::Zero()),
	      drate_spectral_integrity_score_(0.0),
	      srtt_spectral_integrity_score_(0.0),
	      joint_spectral_integrity_score_(0.0),
	      drate_spectral_gate_pass_(false),
	      srtt_spectral_gate_pass_(false),
	      dual_signal_spectral_gate_pass_(false),
	      limiting_spectral_signal_(kLimitingSpectralSignalEqual),
	      merged_rescue_attempted_(false),
	      merged_rescue_success_(false),
	      trusted_bw_selection_compute_us_(0),
	      normal_window_count_(0),
	      merged_window_count_(0),
	      spectral_invalid_count_(0),
	      enable_convergence_gate_trace_(true),
	      enable_convergence_gate_control_(false),
	      trusted_bw_clear_on_cruise_start_(true),
	      stable_single_round_exit_threshold_(0.25),
	      stable_consecutive_exit_threshold_(0.15),
	      stable_rounds_(kStableRounds),
	      stable_full_pipe_growth_threshold_(1.25),
	      drate_spectral_integrity_threshold_(0.25),
	      srtt_spectral_integrity_threshold_(0.25),
	      min_drate_snr_(1.5),
	      min_srtt_snr_(1.5),
	      max_drate_width_ratio_(2.0),
	      max_srtt_width_ratio_(2.5),
	      min_drate_phase_coherence_(0.5),
	      min_srtt_phase_coherence_(0.5),
	      freq_sigma_ratio_(0.08),
	      snr_slope_(2.0),
	      energy_threshold_(0.10),
	      energy_slope_(20.0),
	      width_r0_drate_(1.5),
	      width_r0_srtt_(2.0),
      width_sigma_(0.8),
	      merged_rescue_enabled_(true),
	      merged_window_multiplier_(2.0),
	      max_merged_passes_(1),
	      merged_window_max_trend_ratio_(0.20),
	      merged_confidence_discount_(0.8),
	      trace_flow_id_(0),
      gate_trace_mode_(FBBRGateTraceMode::kRoundOnly),
      gate_trace_sample_interval_(TimeDelta::FromMilliseconds(1)),
      last_pacing_gate_trace_time_(QuicTime::Zero()) {
  QUIC_DVLOG(2) << this << " Initializing FBBRSender @ " << now
                << "; DefaultEcnCongestionRatio="
                << default_ecn_congestion_ratio_;
}

FBBRSender::~FBBRSender() = default;

void FBBRSender::SetOscillationFrequency(double freq_hz) {
  configured_modulation_freq_hz_ = freq_hz;
}

void FBBRSender::SetOscillationAmplitude(FBBRAmplitudeMode mode,
                                             uint64_t fixed_bps) {
  amplitude_mode_ = mode;
  fixed_amplitude_bps_ = fixed_bps;
}

void FBBRSender::SetRecvSignalMode(bool use_delivery_rate_latest) {
  use_delivery_rate_latest_for_signal_history_ = use_delivery_rate_latest;
}

void FBBRSender::SetCruiseWindowConfig(double min_cycles_per_window,
                                           double window_step_ratio) {
  if (min_cycles_per_window > 0.0) {
    min_cruise_cycles_per_window_ = min_cycles_per_window;
  }
  if (window_step_ratio > 0.0) {
    cruise_window_step_ratio_ = window_step_ratio;
  }
}

void FBBRSender::SetFairShareBandwidthBps(uint64_t fair_share_bps) {
  fair_share_bandwidth_bps_ = fair_share_bps;
}

void FBBRSender::SetConvergenceGateTraceEnabled(bool enabled) {
  enable_convergence_gate_trace_ = enabled;
}

void FBBRSender::SetConvergenceGateControlEnabled(bool enabled) {
  enable_convergence_gate_control_ = enabled;
}

void FBBRSender::ConfigureFBBR(const FBBRConfig& config) {
  fbbr_frequency_search_config_ = config.frequency_search;
  stable_single_round_exit_threshold_ =
      std::max(0.0, config.stability_single_round_exit_threshold);
  stable_consecutive_exit_threshold_ =
      std::max(0.0, config.stability_consecutive_exit_threshold);
  stable_rounds_ = std::max<uint32_t>(1, config.stability_stable_rounds);
  if (bbr_stable_) {
    stable_cnt_ = stable_rounds_;
  }
  stable_full_pipe_growth_threshold_ =
      std::max(1.0, config.stability_full_pipe_growth_threshold);

  drate_spectral_integrity_threshold_ =
      Clamp01(config.spectral_drate_integrity_threshold);
  srtt_spectral_integrity_threshold_ =
      Clamp01(config.spectral_srtt_integrity_threshold);
  min_drate_snr_ = std::max(0.0, config.spectral_min_drate_snr);
  min_srtt_snr_ = std::max(0.0, config.spectral_min_srtt_snr);
  max_drate_width_ratio_ = std::max(0.0, config.spectral_max_drate_width_ratio);
  max_srtt_width_ratio_ = std::max(0.0, config.spectral_max_srtt_width_ratio);
  min_drate_phase_coherence_ =
      Clamp01(config.spectral_min_drate_phase_coherence);
  min_srtt_phase_coherence_ =
      Clamp01(config.spectral_min_srtt_phase_coherence);
  freq_sigma_ratio_ = std::max(0.0, config.spectral_freq_sigma_ratio);
  snr_slope_ = std::max(0.0, config.spectral_snr_slope);
  energy_threshold_ = Clamp01(config.spectral_energy_threshold);
  energy_slope_ = std::max(0.0, config.spectral_energy_slope);
  width_r0_drate_ = std::max(0.0, config.spectral_width_r0_drate);
  width_r0_srtt_ = std::max(0.0, config.spectral_width_r0_srtt);
  width_sigma_ = std::max(1e-6, config.spectral_width_sigma);

  merged_rescue_enabled_ = config.merged_rescue_enable;
  merged_window_multiplier_ =
      std::max(1.0, config.merged_rescue_window_multiplier);
  max_merged_passes_ = std::min<uint32_t>(
      1, std::max<uint32_t>(0, config.merged_rescue_max_passes));
  merged_window_max_trend_ratio_ =
      std::max(0.0, config.merged_rescue_max_trend_ratio);
  merged_confidence_discount_ =
      Clamp01(config.merged_rescue_confidence_discount);

  trusted_bw_clear_on_cruise_start_ = config.trusted_bw_clear_on_cruise_start;
  fbbr_history_valid_ = false;
  fbbr_history_bandwidth_bps_ = 0.0;
  ClearTrustedBw("configuration_changed");
}

void FBBRSender::SetTraceFlowId(uint32_t flow_id) {
  trace_flow_id_ = flow_id;
}

void FBBRSender::SetGateTraceMode(FBBRGateTraceMode mode,
                                       uint64_t sample_interval_us) {
  gate_trace_mode_ = mode;
  gate_trace_sample_interval_ = TimeDelta::FromMicroseconds(
      static_cast<int64_t>(std::max<uint64_t>(1, sample_interval_us)));
}

bool FBBRSender::RunConvergenceGateStateMachineSelfTest(
    std::ostream& os) {
  struct TestState {
    bool bbr_stable = false;
    uint32_t stable_cnt = 0;
    bool d_prev_valid = false;
    double d_prev = 0.0;
    bool full_ref_valid = false;
    double full_ref = 0.0;
    double prev_v_round = 0.0;
    bool freq_tool_needed = true;
    bool freq_tool_on = true;
    bool trusted_bw_valid = true;
    double w_freq = 1.0;

    struct Step {
      double d = 0.0;
      double v = -1.0;
      bool just_exited = false;
      uint32_t stable_cnt = 0;
      bool bbr_stable = false;
      double full_ref = 0.0;
      bool freq_tool_on = false;
      bool trusted_bw_valid = false;
      double w_freq = 0.0;
    };

    void UpdateWeightAndToolState() {
      if (bbr_stable) {
        w_freq = 0.0;
        freq_tool_needed = false;
        freq_tool_on = false;
        return;
      }
      if (stable_cnt >= 3) {
        stable_cnt = 3;
        bbr_stable = true;
        w_freq = 0.0;
        freq_tool_needed = false;
        freq_tool_on = false;
        trusted_bw_valid = false;
        return;
      }
      w_freq = std::max(0.0, std::min(1.0, 1.0 -
          static_cast<double>(stable_cnt) / 3.0));
      freq_tool_on = freq_tool_needed;
    }

    Step Round(double d) {
      double v = -1.0;
      if (d_prev_valid && d_prev > 0.0) {
        v = std::abs(d - d_prev) / d_prev;
      }

      bool just_exited = false;
      if (bbr_stable && v >= 0.0) {
        const bool exit_stable =
            v > 0.25 || (v > 0.15 && prev_v_round > 0.15);
        prev_v_round = v;
        if (exit_stable) {
          bbr_stable = false;
          stable_cnt = 0;
          full_ref = d;
          full_ref_valid = true;
          freq_tool_needed = true;
          w_freq = 1.0;
          just_exited = true;
        }
      }

      if (!bbr_stable && !just_exited) {
        if (!full_ref_valid) {
          full_ref = d;
          full_ref_valid = true;
          stable_cnt = 0;
        } else if (d >= 1.25 * full_ref) {
          full_ref = d;
          stable_cnt = 0;
        } else if (stable_cnt < 3) {
          ++stable_cnt;
        }
        if (stable_cnt >= 3) {
          stable_cnt = 3;
          bbr_stable = true;
          freq_tool_needed = false;
          w_freq = 0.0;
          trusted_bw_valid = false;
          freq_tool_on = false;
          prev_v_round = 0.0;
        }
      }

      UpdateWeightAndToolState();
      d_prev = d;
      d_prev_valid = true;
      return {d,
              v,
              just_exited,
              stable_cnt,
              bbr_stable,
              full_ref_valid ? full_ref : 0.0,
              freq_tool_on,
              trusted_bw_valid,
              w_freq};
    }
  };

  auto nearly = [](double lhs, double rhs) {
    return std::abs(lhs - rhs) < 1e-6;
  };
  bool pass = true;
  auto require = [&pass, &os](bool condition, const std::string& message) {
    if (!condition) {
      pass = false;
      os << "FAIL: " << message << "\n";
    }
  };
  auto print_table = [&os](const std::string& name,
                           const std::vector<TestState::Step>& steps) {
    os << "\n## " << name << "\n";
    os << "step,d_round,v_round,just_exited,stable_cnt,bbr_stable,"
          "full_ref,freq_tool_on,trusted_bw_valid,w_freq\n";
    for (size_t i = 0; i < steps.size(); ++i) {
      const auto& s = steps[i];
      os << i << "," << s.d << "," << s.v << ","
         << (s.just_exited ? "true" : "false") << ","
         << s.stable_cnt << "," << (s.bbr_stable ? "true" : "false")
         << "," << s.full_ref << ","
         << (s.freq_tool_on ? "true" : "false") << ","
         << (s.trusted_bw_valid ? "true" : "false") << ","
         << s.w_freq << "\n";
    }
  };

  os << "# FBBR convergence-gate state-machine self-test\n";

  {
    TestState s;
    std::vector<TestState::Step> steps;
    for (double d : {100.0, 110.0, 115.0, 118.0}) {
      steps.push_back(s.Round(d));
    }
    print_table("Sequence 1: normal reconvergence", steps);
    require(steps[0].stable_cnt == 0, "seq1 step0 stable_cnt must be 0");
    require(steps[1].stable_cnt == 1, "seq1 step1 stable_cnt must be 1");
    require(steps[2].stable_cnt == 2, "seq1 step2 stable_cnt must be 2");
    require(steps[3].stable_cnt == 3, "seq1 step3 stable_cnt must be 3");
    require(steps[3].bbr_stable, "seq1 final bbr_stable must be true");
    require(nearly(steps[3].w_freq, 0.0), "seq1 final w_freq must be 0");
    require(!steps[3].freq_tool_on,
            "seq1 final freq_tool_on must be false");
  }

  {
    TestState s;
    s.bbr_stable = true;
    s.stable_cnt = 3;
    s.d_prev = 100.0;
    s.d_prev_valid = true;
    s.freq_tool_needed = false;
    s.freq_tool_on = false;
    s.trusted_bw_valid = false;
    s.w_freq = 0.0;
    std::vector<TestState::Step> steps = {s.Round(130.0)};
    print_table("Sequence 2: single >25% stable exit", steps);
    require(steps[0].just_exited, "seq2 must just_exit");
    require(!steps[0].bbr_stable, "seq2 bbr_stable must be false");
    require(steps[0].stable_cnt == 0,
            "seq2 just_exit round must keep stable_cnt 0");
    require(nearly(steps[0].w_freq, 1.0),
            "seq2 just_exit round must keep w_freq 1");
  }

  {
    TestState s;
    s.bbr_stable = true;
    s.stable_cnt = 3;
    s.freq_tool_needed = false;
    s.freq_tool_on = false;
    s.trusted_bw_valid = false;
    s.w_freq = 0.0;
    std::vector<TestState::Step> steps;
    for (double d : {100.0, 116.0, 134.0}) {
      steps.push_back(s.Round(d));
    }
    print_table("Sequence 3: two consecutive >15% stable exits", steps);
    require(!steps[1].just_exited,
            "seq3 first 16% round must not exit by itself");
    require(steps[2].just_exited,
            "seq3 second >15% round must just_exit");
    require(steps[2].stable_cnt == 0,
            "seq3 just_exit round must keep stable_cnt 0");
    require(nearly(steps[2].w_freq, 1.0),
            "seq3 just_exit round must keep w_freq 1");
  }

  {
    TestState s;
    s.bbr_stable = false;
    s.stable_cnt = 0;
    s.full_ref = 130.0;
    s.full_ref_valid = true;
    s.freq_tool_needed = true;
    s.freq_tool_on = true;
    s.trusted_bw_valid = true;
    s.w_freq = 1.0;
    std::vector<TestState::Step> steps;
    for (double d : {135.0, 138.0, 140.0}) {
      steps.push_back(s.Round(d));
    }
    print_table("Sequence 4: reconverge after exit", steps);
    require(steps[0].stable_cnt == 1, "seq4 step0 stable_cnt must be 1");
    require(steps[1].stable_cnt == 2, "seq4 step1 stable_cnt must be 2");
    require(steps[2].stable_cnt == 3, "seq4 step2 stable_cnt must be 3");
    require(nearly(steps[0].w_freq, 2.0 / 3.0),
            "seq4 step0 w_freq must be 2/3");
    require(nearly(steps[1].w_freq, 1.0 / 3.0),
            "seq4 step1 w_freq must be 1/3");
    require(nearly(steps[2].w_freq, 0.0), "seq4 final w_freq must be 0");
    require(steps[2].bbr_stable, "seq4 final bbr_stable must be true");
    require(!steps[2].trusted_bw_valid, "seq4 final trusted_bw_valid must be false");
    require(!steps[2].freq_tool_on,
            "seq4 final freq_tool_on must be false");
  }

  {
    TestState s;
    s.bbr_stable = false;
    s.stable_cnt = 0;
    s.full_ref = 100.0;
    s.full_ref_valid = true;
    s.freq_tool_needed = true;
    s.freq_tool_on = true;
    s.trusted_bw_valid = true;
    s.w_freq = 1.0;
    std::vector<TestState::Step> steps;
    for (double d : {130.0, 132.0, 134.0}) {
      steps.push_back(s.Round(d));
    }
    print_table("Sequence 5: growth resets reconvergence", steps);
    require(steps[0].stable_cnt == 0,
            "seq5 30% growth must reset stable_cnt to 0");
    require(nearly(steps[0].full_ref, 130.0),
            "seq5 30% growth must refresh full_ref");
    require(steps[1].stable_cnt == 1, "seq5 step1 stable_cnt must be 1");
    require(steps[2].stable_cnt == 2, "seq5 step2 stable_cnt must be 2");
    require(nearly(steps[0].w_freq, 1.0), "seq5 step0 w_freq must be 1");
    require(nearly(steps[1].w_freq, 2.0 / 3.0),
            "seq5 step1 w_freq must be 2/3");
    require(nearly(steps[2].w_freq, 1.0 / 3.0),
            "seq5 step2 w_freq must be 1/3");
  }

	  os << "\nRESULT: " << (pass ? "PASS" : "FAIL") << "\n";
	  return pass;
}

bool FBBRSender::RunTrustedBwSelectionSelfTest(std::ostream& os) {
  bool pass = true;
  auto require = [&pass, &os](bool condition, const char* message) {
    if (!condition) {
      pass = false;
      os << "FAIL: " << message << "\n";
    }
  };
  auto check = [&require, &os](double drate_score,
                               double srtt_score,
                               double drate_threshold,
                               double srtt_threshold,
                               bool drate_valid,
                               bool srtt_valid) {
    const double joint_score = std::min(drate_score, srtt_score);
    const bool drate_gate =
        drate_valid && std::isfinite(drate_score) &&
        drate_score >= drate_threshold;
    const bool srtt_gate =
        srtt_valid && std::isfinite(srtt_score) &&
        srtt_score >= srtt_threshold;
    const bool dual_gate = drate_gate && srtt_gate;
    os << drate_score << "," << srtt_score << "," << joint_score << ","
       << (drate_gate ? "true" : "false") << ","
       << (srtt_gate ? "true" : "false") << ","
       << (dual_gate ? "true" : "false") << "\n";
    require(std::abs(joint_score - std::min(drate_score, srtt_score)) < 1e-12,
            "joint score must be the strict minimum");
    return dual_gate;
  };

  os << "# FBBR TrustedBw dual-signal selection self-test\n";
  os << "drate_score,srtt_score,joint_score,drate_gate,srtt_gate,dual_gate\n";
  require(!check(0.90, 0.20, 0.25, 0.25, true, true),
          "low SRTT integrity must reject the window");
  require(!check(0.30, 0.95, 0.40, 0.25, true, true),
          "low Delivery Rate integrity must reject the window");
  require(check(0.85, 0.82, 0.40, 0.40, true, true),
          "both signals above their thresholds must pass");
  require(!check(0.85, 0.82, 0.40, 0.40, true, false),
          "missing SRTT evidence must reject the window");
  require(!check(0.85, 0.82, 0.40, 0.40, false, true),
          "missing Delivery Rate evidence must reject the window");
  os << "\nRESULT: " << (pass ? "PASS" : "FAIL") << "\n";
  return pass;
}

bool FBBRSender::RunTrustedBwPacingSelfTest(std::ostream& os) {
  bool pass = true;
  auto require = [&pass, &os](bool condition, const char* message) {
    if (!condition) {
      pass = false;
      os << "FAIL: " << message << "\n";
    }
  };
  auto check_phase = [&require, &os](const char* phase,
                                     double native_gbps,
                                     double trusted_gbps,
                                     bool trusted_valid,
                                     double gain,
                                     double expected_gbps) {
    const double pacing_base_gbps =
        trusted_valid ? trusted_gbps : native_gbps;
    const double target_gbps = gain * pacing_base_gbps;
    os << phase << "," << native_gbps << "," << trusted_gbps << ","
       << (trusted_valid ? "TRUSTED_BW" : "NATIVE_BBR") << ","
       << gain << "," << target_gbps << "\n";
    require(std::abs(target_gbps - expected_gbps) < 1e-12,
            "phase pacing must be gain times the selected bandwidth baseline");
  };

  os << "# FBBR TrustedBw pacing self-test\n";
  os << "phase,native_gbps,trusted_gbps,pacing_base_source,gain,target_gbps\n";
  check_phase("REFILL", 100.0, 80.0, true, 1.0, 80.0);
  check_phase("UP", 100.0, 80.0, true, 1.25, 100.0);
  check_phase("DOWN", 100.0, 80.0, true, 0.9, 72.0);
  check_phase("REFILL", 100.0, 0.0, false, 1.0, 100.0);
  check_phase("UP", 100.0, 0.0, false, 1.25, 125.0);
  check_phase("DOWN", 100.0, 0.0, false, 0.9, 90.0);
  const double cruise_native_pacing = 100.0;
  const double amplitude_ratio = 0.05;
  const double coded_sine = 0.8;
  const double cruise_pacing =
      cruise_native_pacing * (1.0 + amplitude_ratio * coded_sine);
  require(std::abs(cruise_pacing - 104.0) < 1e-12,
          "F-BBR CRUISE must multiply native pacing by probe waveform factor");
  os << "\nRESULT: " << (pass ? "PASS" : "FAIL") << "\n";
  return pass;
}

bool FBBRSender::RunFBBRFrequencySearchSelfTests(std::ostream& os) {
  bool pass = FBBRFrequencySearch::RunSelfTests(os);
  uint64_t previous_winner = 0;
  bool election_pass = true;
  for (uint32_t epoch_slot = 0; epoch_slot < 4; ++epoch_slot) {
    uint64_t winner = 0;
    uint32_t pulser_count = 0;
    for (uint64_t flow = 1; flow <= 4; ++flow) {
      if ((flow - 1) % 4 == epoch_slot) {
        winner = flow;
        ++pulser_count;
      }
    }
    election_pass = election_pass && pulser_count == 1 && winner != 0 &&
                    winner != previous_winner;
    previous_winner = winner;
  }
  if (!election_pass) {
    os << "FAIL: four-flow time-slot pulser election rotation\n";
  } else {
    os << "PASS: four-flow time-slot pulser election rotation\n";
  }
  pass = pass && election_pass;
  os << "OVERALL RESULT: " << (pass ? "PASS" : "FAIL") << "\n";
  return pass;
}
Bbr2ProbeBwMode::CyclePhase FBBRSender::GetCurrentProbeBwPhase() const {
  DebugState state = ExportDebugState();
  if (state.mode == Bbr2Mode::PROBE_BW) {
    return state.probe_bw.phase;
  }
  return Bbr2ProbeBwMode::CyclePhase::PROBE_NOT_STARTED;
}

bool FBBRSender::BaseShouldOscillate() const {
  if (fbbr_frequency_search_config_.frequency_search_enabled) {
    return fbbr_probe_active_ && in_cruise_ &&
           fbbr_queue_servo_state_.state !=
               FBBRQueueServoState::kEmergencyDrain &&
           mode_ == Bbr2Mode::PROBE_BW &&
           GetCurrentProbeBwPhase() ==
               Bbr2ProbeBwMode::CyclePhase::PROBE_CRUISE;
  }
  if (!fbbr_frequency_search_config_.legacy_spectral_path_enabled) {
    return false;
  }
  if (GetCurrentAmplitudeBps() == 0 || configured_modulation_freq_hz_ <= 0.0) {
    return false;
  }
  if (!drain_completed_ || mode_ != Bbr2Mode::PROBE_BW) {
    return false;
  }
  return GetCurrentProbeBwPhase() == Bbr2ProbeBwMode::CyclePhase::PROBE_CRUISE &&
         in_cruise_ && cruise_start_time_ != QuicTime::Zero();
}

bool FBBRSender::ShouldOscillate() const {
  const bool base_should = BaseShouldOscillate();
  if (!enable_convergence_gate_control_) {
    return base_should;
  }
  return base_should && !bbr_stable_;
}

uint64_t FBBRSender::GetCurrentAmplitudeBps() const {
  QuicBandwidth max_bw = BandwidthEstimate();
  QuicBandwidth base_rate = Bbr2Sender::PacingRate(0);

  switch (amplitude_mode_) {
    case FBBRAmplitudeMode::kFixed:
      return fixed_amplitude_bps_;
    case FBBRAmplitudeMode::kMiu2:
      return max_bw.ToBitsPerSecond() / 2;
    case FBBRAmplitudeMode::kMiu3:
      return max_bw.ToBitsPerSecond() / 3;
    case FBBRAmplitudeMode::kMiu4:
      return max_bw.ToBitsPerSecond() / 4;
    case FBBRAmplitudeMode::kMiu8:
      return max_bw.ToBitsPerSecond() / 8;
    case FBBRAmplitudeMode::kSR2:
      return base_rate.ToBitsPerSecond() / 2;
    case FBBRAmplitudeMode::kSR3:
      return base_rate.ToBitsPerSecond() / 3;
    case FBBRAmplitudeMode::kSR4:
      return base_rate.ToBitsPerSecond() / 4;
    case FBBRAmplitudeMode::kSR8:
      return base_rate.ToBitsPerSecond() / 8;
    case FBBRAmplitudeMode::kSR12:
      return base_rate.ToBitsPerSecond() / 12;
    case FBBRAmplitudeMode::kSR16:
      return base_rate.ToBitsPerSecond() / 16;
    default:
      return 0;
  }
}

double FBBRSender::TriangleWave(QuicTime now) const {
  if (cruise_modulation_freq_hz_ <= 0.0 ||
      cruise_start_time_ == QuicTime::Zero()) {
    return 0.0;
  }
  const double elapsed_s =
      static_cast<double>((now - cruise_start_time_).ToMicroseconds()) /
      1000000.0;
  const double period_s = 1.0 / cruise_modulation_freq_hz_;
  if (period_s <= 0.0) {
    return 0.0;
  }
  const double q = std::fmod(std::max(0.0, elapsed_s), period_s) / period_s;
  if (q < 0.25) {
    return 4.0 * q;
  }
  if (q < 0.75) {
    return 2.0 - 4.0 * q;
  }
  return 4.0 * q - 4.0;
}

void FBBRSender::FBBRFrequencySearchInitializeCruise(QuicTime now) {
  const FBBRSearchControllerState previous_search_state =
      fbbr_search_state_;
  bool dynamic_reacquire_at_entry = false;
  fbbr_probe_active_ = false;
  fbbr_block_invalidated_ = false;
  fbbr_probe_disabled_reason_ = "none";
  fbbr_rtprop_frozen_ = model_.MinRtt();
  if (fbbr_rtprop_frozen_.IsZero() && rtt_stats_ != nullptr) {
    fbbr_rtprop_frozen_ = rtt_stats_->MinOrInitialRtt();
  }
  fbbr_phase_bin_accumulators_.clear();
  fbbr_phase_bins_.clear();
  fbbr_block_results_.clear();
  fbbr_next_block_id_ = 0;
  fbbr_next_shadow_window_id_ = 0;
  fbbr_previous_block_delay_s_ = std::numeric_limits<double>::quiet_NaN();
  fbbr_control_decisions_.clear();
  fbbr_next_analysis_output_bin_ = -1;
  fbbr_baseline_transition_pending_ = false;
  fbbr_baseline_transition_start_ = QuicTime::Zero();
  fbbr_baseline_transition_end_ = QuicTime::Zero();
  fbbr_trusted_published_in_cruise_ = false;
  fbbr_period_reselected_ = false;
  fbbr_period_scan_count_ = 0;
  fbbr_event_window_state_ = EventWindowState::kIdleListen;
  fbbr_next_trigger_cycle_ = -1;
  fbbr_trigger_cycle_id_ = -1;
  fbbr_capture_start_bin_ = -1;
  fbbr_next_tracking_start_bin_ = -1;
  fbbr_last_control_window_start_bin_ = -1;
  fbbr_last_trusted_window_start_bin_ = -1;
  fbbr_next_event_window_id_ = 0;
  fbbr_bad_cycle_streak_ = 0;
  fbbr_no_trigger_streak_ = 0;
  fbbr_settling_cycles_remaining_ = 0;
  fbbr_pulser_lease_age_cycles_ = 0;
  fbbr_previous_sequential_direction_sign_ = 0;
  fbbr_previous_sequential_classification_ =
      FbbrOperatingPointClassification::kInvalid;
  fbbr_previous_sequential_score_ = 0.0;
  fbbr_have_previous_sequential_result_ = false;
  fbbr_latest_trigger_cycle_result_ = FBBRTriggerCycleResult();
  fbbr_active_trigger_source_ = "NONE";
  fbbr_queue_servo_state_ = FBBRQueueServoStateData();
  fbbr_latest_queue_servo_decision_ = FBBRQueueServoDecision();
  fbbr_queue_servo_q_samples_s_.clear();
  fbbr_queue_servo_delivery_samples_bps_.clear();
  fbbr_queue_servo_q_median_history_s_.clear();
  fbbr_queue_servo_acked_bytes_ = 0;
  fbbr_queue_servo_lost_bytes_ = 0;
  fbbr_queue_servo_ecn_bytes_ = 0;
  fbbr_queue_servo_transition_pending_ = false;
  fbbr_latest_sustainable_direction_ = 0.0;
  fbbr_queue_servo_updates_ = 0;
  fbbr_queue_servo_drain_rtts_ = 0;
  fbbr_queue_servo_recovery_rtts_ = 0;
  fbbr_queue_servo_baseline_commits_ = 0;
  fbbr_dual_trigger_attempts_ = 0;
  fbbr_delivery_triggers_ = 0;
  fbbr_queue_triggers_ = 0;
  fbbr_both_triggers_ = 0;
  fbbr_hard_safety_events_ = 0;
  fbbr_last_ecn_bytes_in_round_ = GetBytesEcnInRounds();
  fbbr_native_bw_at_cruise_start_bps_ =
      static_cast<double>(BandwidthEstimate().ToBitsPerSecond());
  fbbr_probe_signature_ = FbbrProbeSignature();
  fbbr_probe_signature_.flow_identity = static_cast<uint64_t>(trace_flow_id_);
  fbbr_probe_signature_.cruise_id =
      static_cast<uint64_t>(std::max<int64_t>(0, cruise_id_));

  if (!fbbr_frequency_search_config_.frequency_search_enabled) {
    fbbr_probe_disabled_reason_ = "frequency_search_disabled";
    return;
  }
  if (fbbr_rtprop_frozen_.IsZero()) {
    fbbr_probe_disabled_reason_ = "rtprop_unavailable";
    return;
  }
  if (fbbr_frequency_search_config_.probe_period_rtt_slots.empty() ||
      fbbr_frequency_search_config_.probe_code_length_cycles != 4 ||
      fbbr_frequency_search_config_.analysis_cycles_per_window == 0 ||
      fbbr_frequency_search_config_.phase_bins_per_cycle == 0 ||
      fbbr_frequency_search_config_.min_valid_analysis_cycles >
          fbbr_frequency_search_config_.analysis_cycles_per_window) {
    fbbr_probe_disabled_reason_ = "probe_config_invalid";
    return;
  }
  if (InRecovery()) {
    fbbr_probe_disabled_reason_ = "recovery";
    return;
  }
  const DebugState state = ExportDebugState();
  fbbr_last_sample_app_limited_ = state.last_sample_is_app_limited;
  if (state.last_sample_is_app_limited) {
    fbbr_probe_disabled_reason_ = "app_limited_at_start";
    return;
  }
  // Stability and prior congestion select a safe initial mode/baseline; they
  // do not terminate persistent identification in an eligible CRUISE.

  auto mix = [](uint64_t value) {
    value += 0x9e3779b97f4a7c15ULL;
    value = (value ^ (value >> 30)) * 0xbf58476d1ce4e5b9ULL;
    value = (value ^ (value >> 27)) * 0x94d049bb133111ebULL;
    return value ^ (value >> 31);
  };
  const uint64_t flow_identity = fbbr_probe_signature_.flow_identity;
  const uint64_t cruise_identity = fbbr_probe_signature_.cruise_id;
  const uint64_t signature_hash =
      mix(flow_identity ^ mix(cruise_identity + 0x46424252ULL));
  uint32_t slot = static_cast<uint32_t>(
      fbbr_frequency_search_config_.probe_period_rtt_slots.size() / 2);
  for (uint32_t index = 0;
       index < fbbr_frequency_search_config_.probe_period_rtt_slots.size(); ++index) {
    if (fbbr_frequency_search_config_.probe_period_rtt_slots[index] == 8) {
      slot = index;
      break;
    }
  }
  const uint32_t period_rtts_slot =
      fbbr_frequency_search_config_.probe_period_rtt_slots[slot];
  const double rtprop_s =
      static_cast<double>(fbbr_rtprop_frozen_.ToMicroseconds()) / 1e6;
  const double amplitude_budget =
      M_PI * fbbr_frequency_search_config_.probe_queue_budget_bdp /
      static_cast<double>(period_rtts_slot);
  const double amplitude_ratio = std::min(
      fbbr_frequency_search_config_.probe_max_amplitude_ratio,
      std::min(fbbr_frequency_search_config_.probe_target_amplitude_ratio,
               amplitude_budget));
  if (amplitude_ratio < fbbr_frequency_search_config_.probe_min_amplitude_ratio) {
    fbbr_probe_disabled_reason_ = "queue_budget_amplitude_low";
    return;
  }
  const QuicBandwidth native_pacing = BandwidthEstimate();
  const double native_bps =
      static_cast<double>(native_pacing.ToBitsPerSecond());
  if (native_bps <= 0.0) {
    fbbr_probe_disabled_reason_ = "native_pacing_zero";
    return;
  }
  const double rtt_period_s = period_rtts_slot * rtprop_s;
  // B is bits/s, hence the factor 8 when translating the positive-half
  // packetization budget into bytes.
  const double packet_period_s =
      8.0 * fbbr_frequency_search_config_.min_extra_probe_mss * M_PI *
      static_cast<double>(kDefaultTCPMSS) /
      std::max(amplitude_ratio * native_bps, 1.0);
  const double period_s = ClampValue(
      std::max(rtt_period_s, packet_period_s),
      fbbr_frequency_search_config_.carrier_period_min_s,
      fbbr_frequency_search_config_.carrier_period_max_s);
  const int64_t period_us = static_cast<int64_t>(std::llround(period_s * 1e6));
  if (period_us <= 0) {
    fbbr_probe_disabled_reason_ = "carrier_period_invalid";
    return;
  }
  // If the full positive-byte target cannot fit below period_max, keep the
  // maximum safe period and let actual-input SNR drive later amplitude/period
  // adaptation.  Packetization mismatch is not a search termination reason.
  fbbr_period_ = TimeDelta::FromMicroseconds(period_us);
  const uint32_t period_rtts = static_cast<uint32_t>(std::max<int64_t>(
      1, static_cast<int64_t>(std::llround(period_s / rtprop_s))));

  fbbr_probe_signature_.flow_identity = flow_identity;
  fbbr_probe_signature_.cruise_id = cruise_identity;
  fbbr_probe_signature_.period_slot = slot;
  fbbr_probe_signature_.period_rtts = period_rtts;
  fbbr_probe_signature_.code_id = static_cast<uint32_t>(
      mix(signature_hash ^ 0x434f4445ULL) % 3ULL);
  // Walsh chips change only at carrier boundaries.  Hash between the two
  // zero-crossing phases (0 and pi), preserving a stable per-flow phase while
  // guaranteeing that a chip flip cannot create a pacing discontinuity.
  fbbr_probe_signature_.initial_phase_rad =
      (mix(signature_hash ^ 0x5048415345ULL) & 1ULL) ? M_PI : 0.0;
  fbbr_probe_signature_.rtprop_s = rtprop_s;
  fbbr_probe_signature_.period_s = period_s;
  fbbr_probe_signature_.frequency_hz = 1.0 / period_s;
  fbbr_probe_signature_.amplitude_ratio = amplitude_ratio;
  fbbr_probe_signature_.waveform = fbbr_frequency_search_config_.probe_waveform;
  if (fbbr_opi_rtprop_anchor_.IsZero() ||
      fbbr_rtprop_frozen_ < fbbr_opi_rtprop_anchor_) {
    fbbr_opi_rtprop_anchor_ = fbbr_rtprop_frozen_;
    fbbr_opi_rtprop_confidence_ = 0.50;
    fbbr_opi_rtprop_source_ = "NATIVE_MIN_RTT";
  }
  fbbr_probe_signature_.rtprop_s = static_cast<double>(
      fbbr_opi_rtprop_anchor_.ToMicroseconds()) / 1e6;
  fbbr_search_state_ = FBBRSearchController::Initialize(
      native_bps, amplitude_ratio, period_s);
  fbbr_search_state_.carrier_scan_index = slot;
  const double rotation_interval_s = std::max(
      0.25,
      std::max<uint32_t>(fbbr_frequency_search_config_.min_pulser_lease_cycles,
                         fbbr_frequency_search_config_.max_pulser_lease_cycles) *
          period_s);
  const double global_time_s = static_cast<double>(
      (now - QuicTime::Zero()).ToMicroseconds()) / 1e6;
  const uint32_t rotation_slots = std::max<uint32_t>(
      1, fbbr_frequency_search_config_.pulser_backoff_max_cycles);
  const uint32_t epoch_slot = static_cast<uint32_t>(
      std::floor(global_time_s / rotation_interval_s)) % rotation_slots;
  const uint64_t flow_identity_zero_based = flow_identity > 0
      ? flow_identity - 1
      : 0;
  fbbr_search_state_.is_pulser =
      flow_identity_zero_based % rotation_slots == epoch_slot;
  fbbr_search_state_.pulser_lease_count =
      fbbr_search_state_.is_pulser ? 1 : 0;
  fbbr_search_state_.carrier_detected = false;
  fbbr_search_state_.carrier_detection_streak = 0;
  fbbr_search_state_.last_carrier_sense_bin = -1;
  fbbr_search_state_.election_start_cycle = 0;
  fbbr_search_state_.election_backoff_cycles =
      FBBRSearchElectionBackoff(fbbr_search_state_.search_generation);
  fbbr_search_state_.watcher_cooldown_cycles =
      fbbr_frequency_search_config_.pulser_lease_decisions;
  ++fbbr_search_state_.eligible_cruises;
  if (fbbr_frequency_search_config_.persistent_across_cruises &&
      previous_search_state.search_active) {
    fbbr_search_state_.eligible_cruises =
        previous_search_state.eligible_cruises + 1;
    fbbr_search_state_.search_attempts =
        previous_search_state.search_attempts + 1;
    fbbr_search_state_.unresolved_cruises =
        previous_search_state.unresolved_cruises;
    fbbr_search_state_.unresolved_decisions =
        previous_search_state.unresolved_decisions;
    fbbr_search_state_.valid_direction_decisions =
        previous_search_state.valid_direction_decisions;
    fbbr_search_state_.search_generation =
        previous_search_state.search_generation;
    fbbr_search_state_.last_failure_reason =
        previous_search_state.last_failure_reason;
    const double native_change = previous_search_state.cruise_entry_native_bps > 0.0
        ? std::abs(native_bps - previous_search_state.cruise_entry_native_bps) /
              previous_search_state.cruise_entry_native_bps
        : 0.0;
    dynamic_reacquire_at_entry =
        native_change > fbbr_frequency_search_config_.dynamic_native_change_threshold;
    if (dynamic_reacquire_at_entry) {
      fbbr_search_state_.state = FBBRSearchState::kDynamicReacquire;
      const double retained_search_bps =
          previous_search_state.current_search_bps > 0.0
              ? previous_search_state.current_search_bps
              : native_bps;
      fbbr_search_state_.current_search_bps = ClampValue(
          retained_search_bps,
          fbbr_frequency_search_config_.min_search_scale * native_bps,
          fbbr_frequency_search_config_.max_search_scale * native_bps);
      fbbr_search_state_.pending_search_bps =
          fbbr_search_state_.current_search_bps;
      fbbr_search_state_.underload_bound_valid = false;
      fbbr_search_state_.overload_bound_valid = false;
      fbbr_search_state_.provisional_validation_pending = false;
      fbbr_search_state_.provisional_age_cruises = 0;
      fbbr_search_state_.search_generation =
          previous_search_state.search_generation + 1;
      fbbr_search_state_.last_failure_reason =
          "native_change_dynamic_reacquire";
      ClearTrustedBw("dynamic_native_change");
    }
    const double anchor_s = static_cast<double>(
        fbbr_opi_rtprop_anchor_.ToMicroseconds()) / 1e6;
    const double rtprop_change =
        previous_search_state.rtprop_s_when_bracket_created > 0.0
            ? std::abs(anchor_s -
                       previous_search_state.rtprop_s_when_bracket_created) /
                  previous_search_state.rtprop_s_when_bracket_created
            : 0.0;
    const bool bracket_fresh = !dynamic_reacquire_at_entry &&
        previous_search_state.provisional_age_cruises <
            fbbr_frequency_search_config_.bracket_ttl_cruises &&
        native_change <= fbbr_frequency_search_config_.provisional_native_change_max &&
        rtprop_change <= fbbr_frequency_search_config_.provisional_rtprop_change_max;
    if (bracket_fresh && (previous_search_state.underload_bound_valid ||
                          previous_search_state.overload_bound_valid)) {
      fbbr_search_state_.underload_bound_valid =
          previous_search_state.underload_bound_valid;
      fbbr_search_state_.underload_bound_bps =
          previous_search_state.underload_bound_bps;
      fbbr_search_state_.overload_bound_valid =
          previous_search_state.overload_bound_valid;
      fbbr_search_state_.overload_bound_bps =
          previous_search_state.overload_bound_bps;
      fbbr_search_state_.bracket_generation =
          previous_search_state.bracket_generation;
      fbbr_search_state_.bracket_age_windows =
          previous_search_state.bracket_age_windows;
      fbbr_search_state_.native_bps_when_bracket_created =
          previous_search_state.native_bps_when_bracket_created;
      fbbr_search_state_.rtprop_s_when_bracket_created =
          previous_search_state.rtprop_s_when_bracket_created;
      fbbr_search_state_.provisional_validation_pending =
          previous_search_state.underload_bound_valid &&
          previous_search_state.overload_bound_valid;
      fbbr_search_state_.provisional_age_cruises =
          previous_search_state.provisional_age_cruises + 1;
    }
    if (!dynamic_reacquire_at_entry &&
        previous_search_state.unresolved_cruises > 0) {
      fbbr_search_state_.state = FBBRSearchState::kPersistentUnresolved;
      fbbr_search_state_.current_amplitude_ratio = ClampValue(
          previous_search_state.current_amplitude_ratio,
          fbbr_frequency_search_config_.probe_min_amplitude_ratio,
          fbbr_frequency_search_config_.probe_max_amplitude_ratio);
      fbbr_probe_signature_.amplitude_ratio =
          fbbr_search_state_.current_amplitude_ratio;
      if (!fbbr_search_state_.provisional_validation_pending &&
          previous_search_state.current_search_bps > 0.0) {
        fbbr_search_state_.current_search_bps = ClampValue(
            previous_search_state.current_search_bps,
            fbbr_frequency_search_config_.min_search_scale * native_bps,
            fbbr_frequency_search_config_.max_search_scale * native_bps);
        fbbr_search_state_.pending_search_bps =
            fbbr_search_state_.current_search_bps;
      }
    }
  }
  fbbr_baseline_transition_from_bps_ = fbbr_search_state_.current_search_bps;
  fbbr_baseline_transition_to_bps_ = fbbr_search_state_.current_search_bps;
  const int64_t nominal_feedback_bins = static_cast<int64_t>(std::llround(
      fbbr_probe_signature_.rtprop_s /
      std::max(period_s /
                   static_cast<double>(fbbr_frequency_search_config_.phase_bins_per_cycle),
               1e-9)));
  fbbr_next_analysis_output_bin_ =
      static_cast<int64_t>(fbbr_frequency_search_config_.settling_cycles_per_window) *
          fbbr_frequency_search_config_.phase_bins_per_cycle +
      nominal_feedback_bins;
  fbbr_next_trigger_cycle_ = std::max<int64_t>(
      fbbr_frequency_search_config_.carrier_sensing_cycles,
      (fbbr_next_analysis_output_bin_ +
       static_cast<int64_t>(fbbr_frequency_search_config_.phase_bins_per_cycle) - 1) /
          static_cast<int64_t>(fbbr_frequency_search_config_.phase_bins_per_cycle));
  cruise_modulation_freq_hz_ = fbbr_probe_signature_.frequency_hz;
  fbbr_probe_active_ = true;
  if (fbbr_search_state_.unresolved_cruises == 0 &&
      !dynamic_reacquire_at_entry) {
    fbbr_search_state_.state = fbbr_search_state_.is_pulser
        ? FBBRSearchState::kAcquireInput
        : FBBRSearchState::kWatcher;
  }
  fbbr_search_state_.election_backoff_cycles =
      FBBRSearchElectionBackoff(fbbr_search_state_.search_generation);
  fbbr_search_state_.pulser_lease_remaining =
      fbbr_frequency_search_config_.persistent_across_cruises &&
              previous_search_state.search_active &&
              previous_search_state.pulser_lease_remaining > 0
          ? previous_search_state.pulser_lease_remaining
          : fbbr_frequency_search_config_.max_pulser_lease_cycles;
  fbbr_probe_disabled_reason_ = "none";
  freq_tool_on_ = true;
  cruise_freq_tool_active_ = true;
  FBBRFrequencySearchEnsureBinsThrough(now);
  QUIC_DVLOG(1) << "F-BBR Frequency Search probe initialized cruise_id=" << cruise_id_
                << " flow_identity=" << flow_identity
                << " period_rtts=" << period_rtts
                << " frequency_hz=" << fbbr_probe_signature_.frequency_hz
                << " code_id=" << fbbr_probe_signature_.code_id
                << " amplitude_ratio=" << amplitude_ratio;
}

int64_t FBBRSender::FBBRFrequencySearchBinIndex(QuicTime now) const {
  if (cruise_start_time_ == QuicTime::Zero() || now < cruise_start_time_ ||
      fbbr_period_.IsZero() ||
      fbbr_frequency_search_config_.phase_bins_per_cycle == 0) {
    return -1;
  }
  const int64_t elapsed_us = (now - cruise_start_time_).ToMicroseconds();
  return elapsed_us *
         static_cast<int64_t>(fbbr_frequency_search_config_.phase_bins_per_cycle) /
         fbbr_period_.ToMicroseconds();
}

void FBBRSender::FBBRFrequencySearchEnsureBinsThrough(QuicTime now) {
  const int64_t target = FBBRFrequencySearchBinIndex(now);
  if (target < 0) return;
  const int64_t bins_per_cycle =
      static_cast<int64_t>(fbbr_frequency_search_config_.phase_bins_per_cycle);
  const int64_t period_us = fbbr_period_.ToMicroseconds();
  while (static_cast<int64_t>(fbbr_phase_bin_accumulators_.size()) <= target) {
    FbbrPhaseBinAccumulator accumulator;
    accumulator.bin_index =
        static_cast<int64_t>(fbbr_phase_bin_accumulators_.size());
    const int64_t start_us =
        period_us * accumulator.bin_index / bins_per_cycle;
    const int64_t end_us =
        period_us * (accumulator.bin_index + 1) / bins_per_cycle;
    accumulator.time_start_s =
        static_cast<double>((cruise_start_time_ - QuicTime::Zero())
                                .ToMicroseconds() + start_us) /
        1e6;
    accumulator.time_end_s =
        static_cast<double>((cruise_start_time_ - QuicTime::Zero())
                                .ToMicroseconds() + end_us) /
        1e6;
    fbbr_phase_bin_accumulators_.push_back(accumulator);
  }
}

double FBBRSender::FBBRFrequencySearchCodedSineValue(QuicTime now) const {
  if (!fbbr_probe_active_ || cruise_start_time_ == QuicTime::Zero() ||
      now < cruise_start_time_) {
    return 0.0;
  }
  const double elapsed_s =
      static_cast<double>((now - cruise_start_time_).ToMicroseconds()) / 1e6;
  const double waveform =
      FBBRFrequencySearch::ProbeWaveform(fbbr_probe_signature_, elapsed_s);
  const int64_t cycle = static_cast<int64_t>(std::floor(
      elapsed_s / std::max(fbbr_probe_signature_.period_s, 1e-9)));
  if (cycle < static_cast<int64_t>(
                  fbbr_frequency_search_config_.carrier_sensing_cycles)) {
    return 0.0;
  }
  if (fbbr_search_state_.is_pulser) {
    return waveform;
  }
  return waveform * fbbr_frequency_search_config_.watcher_probe_amplitude /
      std::max(fbbr_probe_signature_.amplitude_ratio, 1e-9);
}

uint32_t FBBRSender::FBBRSearchElectionBackoff(
    uint64_t generation) const {
  const uint32_t maximum = std::max<uint32_t>(
      1, fbbr_frequency_search_config_.pulser_backoff_max_cycles);
  const uint64_t identity = fbbr_probe_signature_.flow_identity ^
      FbbrStableHash(fbbr_probe_signature_.cruise_id ^ generation);
  return 1 + static_cast<uint32_t>(FbbrStableHash(identity) % maximum);
}

void FBBRSender::FBBRSearchUpdateCarrierSense() {
  if (fbbr_search_state_.is_pulser || fbbr_phase_bins_.size() < 16 ||
      fbbr_probe_signature_.period_s <= 0.0) {
    return;
  }
  std::vector<const FbbrPhaseBinSample*> samples;
  const size_t maximum_samples = static_cast<size_t>(
      std::max<uint32_t>(2, fbbr_frequency_search_config_.carrier_sensing_cycles) *
      fbbr_frequency_search_config_.phase_bins_per_cycle);
  for (auto it = fbbr_phase_bins_.rbegin(); it != fbbr_phase_bins_.rend() &&
       samples.size() < maximum_samples; ++it) {
    if (it->rtt_valid && it->valid) samples.push_back(&*it);
  }
  if (samples.size() < 12) return;
  const int64_t newest_bin = samples.back()->bin_index;
  if (newest_bin == fbbr_search_state_.last_carrier_sense_bin) return;
  fbbr_search_state_.last_carrier_sense_bin = newest_bin;
  std::reverse(samples.begin(), samples.end());
  double rtt_mean = 0.0;
  double delivery_mean = 0.0;
  for (const auto* sample : samples) {
    rtt_mean += sample->latest_rtt_s;
    delivery_mean += sample->delivery_rate_bps;
  }
  rtt_mean /= samples.size();
  delivery_mean /= samples.size();
  auto amplitude = [&samples](double frequency,
                              const std::function<double(
                                  const FbbrPhaseBinSample&)>& value) {
    double sin_sum = 0.0;
    double cos_sum = 0.0;
    for (const auto* sample : samples) {
      const double time = 0.5 * (sample->time_start_s + sample->time_end_s);
      const double centered = value(*sample);
      sin_sum += centered * std::sin(2.0 * M_PI * frequency * time);
      cos_sum += centered * std::cos(2.0 * M_PI * frequency * time);
    }
    return 2.0 * std::hypot(sin_sum, cos_sum) / samples.size();
  };
  double best_snr = 0.0;
  double best_carrier = 0.0;
  for (uint32_t period_rtts : fbbr_frequency_search_config_.probe_period_rtt_slots) {
    const double period = ClampValue(
        period_rtts * std::max(fbbr_probe_signature_.rtprop_s, 1e-9),
        fbbr_frequency_search_config_.carrier_period_min_s,
        fbbr_frequency_search_config_.carrier_period_max_s);
    const double frequency = 1.0 / std::max(period, 1e-9);
    const auto rtt_value = [rtt_mean](const FbbrPhaseBinSample& sample) {
      return sample.latest_rtt_s - rtt_mean;
    };
    const auto delivery_value = [delivery_mean](const FbbrPhaseBinSample& sample) {
      return (sample.delivery_rate_bps - delivery_mean) /
             std::max(delivery_mean, 1.0);
    };
    const double carrier = std::max(
        amplitude(frequency, rtt_value) /
            std::max(fbbr_probe_signature_.rtprop_s, 1e-9),
        amplitude(frequency, delivery_value));
    const double noise = std::max(
        std::max(amplitude(0.75 * frequency, rtt_value) /
                     std::max(fbbr_probe_signature_.rtprop_s, 1e-9),
                 amplitude(1.25 * frequency, rtt_value) /
                     std::max(fbbr_probe_signature_.rtprop_s, 1e-9)),
        std::max(amplitude(0.75 * frequency, delivery_value),
                 amplitude(1.25 * frequency, delivery_value)));
    best_snr = std::max(best_snr, carrier / std::max(noise, 1e-4));
    best_carrier = std::max(best_carrier, carrier);
  }
  fbbr_search_state_.carrier_sense_snr = best_snr;
  fbbr_search_state_.carrier_sense_amplitude = best_carrier;
  const bool evidence =
      best_snr >= fbbr_frequency_search_config_.carrier_detection_snr_min;
  if (evidence) {
    fbbr_search_state_.carrier_detection_streak = std::min<uint32_t>(
        3, fbbr_search_state_.carrier_detection_streak + 1);
  } else if (fbbr_search_state_.carrier_detection_streak > 0) {
    --fbbr_search_state_.carrier_detection_streak;
  }
  fbbr_search_state_.carrier_detected =
      fbbr_search_state_.carrier_detection_streak >= 2;
  if (fbbr_search_state_.carrier_detected) {
    fbbr_search_state_.state = FBBRSearchState::kWatcher;
  }
}

double FBBRSender::FBBRSearchSearchBaselineBps(QuicTime now) const {
  if (!fbbr_frequency_search_config_.search_controller_enabled ||
      fbbr_baseline_transition_to_bps_ <= 0.0) {
    return static_cast<double>(BandwidthEstimate().ToBitsPerSecond());
  }
  if (!fbbr_baseline_transition_pending_ ||
      fbbr_baseline_transition_start_ == QuicTime::Zero()) {
    return fbbr_baseline_transition_to_bps_;
  }
  if (now <= fbbr_baseline_transition_start_) {
    return fbbr_baseline_transition_from_bps_;
  }
  if (now >= fbbr_baseline_transition_end_) {
    return fbbr_baseline_transition_to_bps_;
  }
  const double elapsed = static_cast<double>(
      (now - fbbr_baseline_transition_start_).ToMicroseconds());
  const double duration = static_cast<double>(
      (fbbr_baseline_transition_end_ -
       fbbr_baseline_transition_start_).ToMicroseconds());
  return FBBRSearchController::RaisedCosineLogRamp(
      fbbr_baseline_transition_from_bps_,
      fbbr_baseline_transition_to_bps_, elapsed / std::max(duration, 1.0));
}

void FBBRSender::FBBRSearchStartBaselineTransition(
    QuicTime now,
    double next_baseline_bps) {
  if (fbbr_period_.IsZero() || cruise_start_time_ == QuicTime::Zero() ||
      next_baseline_bps <= 0.0) {
    return;
  }
  const double from_bps = FBBRSearchSearchBaselineBps(now);
  const int64_t elapsed_us = std::max<int64_t>(
      0, (now - cruise_start_time_).ToMicroseconds());
  const int64_t period_us = fbbr_period_.ToMicroseconds();
  const int64_t start_cycle = (elapsed_us + period_us - 1) / period_us;
  fbbr_baseline_transition_start_ =
      cruise_start_time_ + TimeDelta::FromMicroseconds(start_cycle * period_us);
  fbbr_baseline_transition_end_ =
      fbbr_baseline_transition_start_ + fbbr_period_;
  fbbr_baseline_transition_from_bps_ = from_bps;
  fbbr_baseline_transition_to_bps_ = next_baseline_bps;
  fbbr_baseline_transition_pending_ = true;

  const int64_t bins_per_cycle =
      static_cast<int64_t>(fbbr_frequency_search_config_.phase_bins_per_cycle);
  const int64_t nominal_feedback_bins = static_cast<int64_t>(std::llround(
      fbbr_probe_signature_.rtprop_s /
      std::max(fbbr_probe_signature_.period_s / bins_per_cycle, 1e-9)));
  const int64_t analysis_input_cycle =
      start_cycle +
      static_cast<int64_t>(fbbr_frequency_search_config_.settling_cycles_per_window);
  fbbr_next_analysis_output_bin_ =
      analysis_input_cycle * bins_per_cycle + nominal_feedback_bins;
}

void FBBRSender::FBBRFrequencySearchAccumulateSend(
    QuicTime sent_time,
    QuicByteCount bytes_in_flight,
    QuicByteCount bytes,
    bool retransmittable) {
  if (!fbbr_probe_active_ || !retransmittable) return;
  FBBRFrequencySearchEnsureBinsThrough(sent_time);
  const int64_t index = FBBRFrequencySearchBinIndex(sent_time);
  if (index < 0 ||
      index >= static_cast<int64_t>(fbbr_phase_bin_accumulators_.size())) {
    return;
  }
  FbbrPhaseBinAccumulator& bin =
      fbbr_phase_bin_accumulators_[static_cast<size_t>(index)];
  const double search_baseline_bps = FBBRSearchSearchBaselineBps(sent_time);
  const double servo_factor = ClampValue(
      fbbr_queue_servo_state_.factor, 0.10, 1.02);
  const double native_bps = search_baseline_bps * servo_factor;
  const double commanded_bps = native_bps *
      (1.0 + fbbr_probe_signature_.amplitude_ratio *
                 FBBRFrequencySearchCodedSineValue(sent_time));
  bin.sent_bytes += bytes;
  bin.native_pacing_bps_bytes += native_bps * bytes;
  bin.commanded_pacing_bps_bytes += commanded_bps * bytes;
  bin.bytes_in_flight_bytes += static_cast<double>(bytes_in_flight) * bytes;
  bin.queue_servo_factor_bytes += servo_factor * bytes;
  ++bin.send_events;
  if (fbbr_last_sample_app_limited_) {
    bin.app_limited_sent_bytes += bytes;
  }
  if (bytes_in_flight >= GetCongestionWindow()) {
    bin.cwnd_limited_sent_bytes += bytes;
  }
  if (InRecovery()) {
    bin.recovery_sent_bytes += bytes;
  }
  if (enable_convergence_gate_control_ && bbr_stable_) {
    bin.phase_transition = true;
    fbbr_block_invalidated_ = true;
  }
  if (fbbr_queue_servo_transition_pending_) {
    bin.queue_servo_transition = true;
    fbbr_queue_servo_transition_pending_ = false;
  }
}

void FBBRSender::FBBRFrequencySearchAccumulateAck(
    const Bbr2CongestionEvent& congestion_event) {
  if (!fbbr_probe_active_) return;
  FBBRFrequencySearchEnsureBinsThrough(congestion_event.event_time);
  const int64_t index = FBBRFrequencySearchBinIndex(congestion_event.event_time);
  if (index < 0 ||
      index >= static_cast<int64_t>(fbbr_phase_bin_accumulators_.size())) {
    return;
  }
  FbbrPhaseBinAccumulator& bin =
      fbbr_phase_bin_accumulators_[static_cast<size_t>(index)];
  bin.acked_bytes += congestion_event.bytes_acked;
  bin.lost_bytes += congestion_event.bytes_lost;
  ++bin.ack_events;
  const bool app_limited = congestion_event.sample_is_app_limited ||
      congestion_event.last_sample_is_app_limited ||
      (congestion_event.last_packet_send_state.is_valid &&
       congestion_event.last_packet_send_state.is_app_limited);
  fbbr_last_sample_app_limited_ = app_limited;
  if (app_limited) {
    bin.app_limited_acked_bytes += congestion_event.bytes_acked;
  }
  if (InRecovery()) {
    bin.recovery_acked_bytes += congestion_event.bytes_acked;
  }
  if (rtt_stats_ != nullptr && !rtt_stats_->latest_rtt().IsZero()) {
    const double latest_rtt_s =
        static_cast<double>(rtt_stats_->latest_rtt().ToMicroseconds()) / 1e6;
    bin.latest_rtt_samples_s.push_back(latest_rtt_s);
    const double anchor_s = fbbr_opi_rtprop_anchor_.IsZero()
        ? fbbr_probe_signature_.rtprop_s
        : static_cast<double>(fbbr_opi_rtprop_anchor_.ToMicroseconds()) / 1e6;
    fbbr_queue_servo_q_samples_s_.push_back(
        std::max(0.0, latest_rtt_s - anchor_s));
  }
  const uint64_t ecn_in_round = GetBytesEcnInRounds();
  const uint64_t ecn_delta = ecn_in_round >= fbbr_last_ecn_bytes_in_round_
      ? ecn_in_round - fbbr_last_ecn_bytes_in_round_
      : ecn_in_round;
  bin.ecn_marked_bytes += ecn_delta;
  fbbr_queue_servo_acked_bytes_ += congestion_event.bytes_acked;
  fbbr_queue_servo_lost_bytes_ += congestion_event.bytes_lost;
  fbbr_queue_servo_ecn_bytes_ += ecn_delta;
  if (congestion_event.sample_valid &&
      !congestion_event.sample_max_bandwidth.IsZero()) {
    fbbr_queue_servo_delivery_samples_bps_.push_back(
        static_cast<double>(
            congestion_event.sample_max_bandwidth.ToBitsPerSecond()));
  }
  fbbr_last_ecn_bytes_in_round_ = ecn_in_round;
  if (fbbr_frequency_search_config_.search_controller_enabled &&
      fbbr_search_state_.state != FBBRSearchState::kEmergencyDrain) {
    const double event_loss_ratio =
        congestion_event.bytes_acked + congestion_event.bytes_lost > 0
            ? static_cast<double>(congestion_event.bytes_lost) /
                  static_cast<double>(congestion_event.bytes_acked +
                                      congestion_event.bytes_lost)
            : 0.0;
    const double event_ecn_ratio = congestion_event.bytes_acked > 0
        ? static_cast<double>(ecn_delta) / congestion_event.bytes_acked
        : 0.0;
    if (event_loss_ratio >= fbbr_frequency_search_config_.hard_loss_threshold ||
        event_ecn_ratio >= fbbr_frequency_search_config_.hard_ecn_threshold) {
      FbbrOperatingPointBlockResult abort;
      abort.cruise_id = fbbr_probe_signature_.cruise_id;
      abort.block_id = fbbr_next_block_id_++;
      abort.start_time_s = static_cast<double>(
          (congestion_event.event_time - QuicTime::Zero()).ToMicroseconds()) /
          1e6;
      abort.end_time_s = abort.start_time_s;
      abort.loss_ratio = event_loss_ratio;
      abort.ecn_ratio = event_ecn_ratio;
      abort.classification =
          FbbrOperatingPointClassification::kBufferSaturated;
      abort.invalid_reason = "hard_congestion_window_abort";
      abort.independent_for_control = false;
      abort.independent_for_trusted = false;
      abort.trigger_cycle_excluded_from_score = true;
      fbbr_block_results_.push_back(abort);
      fbbr_next_analysis_output_bin_ = -1;
      FBBRSearchApplyWindowDecision(congestion_event.event_time, abort, -1);
      FBBRFrequencySearchEmitBlockTrace(abort);
    }
  }
  if (GetCurrentProbeBwPhase() !=
      Bbr2ProbeBwMode::CyclePhase::PROBE_CRUISE) {
    bin.phase_transition = true;
  }
  if (congestion_event.end_of_round_trip) {
    FBBRQueueServoUpdate(congestion_event);
  }
}

void FBBRSender::FBBRQueueServoUpdate(
    const Bbr2CongestionEvent& congestion_event) {
  if (!fbbr_frequency_search_config_.queue_servo_enabled) {
    fbbr_queue_servo_state_ = FBBRQueueServoStateData();
    fbbr_queue_servo_q_samples_s_.clear();
    fbbr_queue_servo_delivery_samples_bps_.clear();
    fbbr_queue_servo_acked_bytes_ = 0;
    fbbr_queue_servo_lost_bytes_ = 0;
    fbbr_queue_servo_ecn_bytes_ = 0;
    return;
  }
  QueueServoFastStats stats;
  stats.samples_sufficient = fbbr_queue_servo_q_samples_s_.size() >= 3;
  if (!fbbr_queue_servo_q_samples_s_.empty()) {
    stats.q_floor_s = Percentile(fbbr_queue_servo_q_samples_s_, 0.20);
    stats.q_median_s = Median(fbbr_queue_servo_q_samples_s_);
    stats.q_peak_s = Percentile(fbbr_queue_servo_q_samples_s_, 0.90);
    if (!fbbr_queue_servo_q_median_history_s_.empty()) {
      stats.queue_trend_s_per_s =
          (stats.q_median_s - fbbr_queue_servo_q_median_history_s_.back()) /
          std::max(fbbr_probe_signature_.rtprop_s, 1e-9);
    }
    fbbr_queue_servo_q_median_history_s_.push_back(stats.q_median_s);
    while (fbbr_queue_servo_q_median_history_s_.size() > 2) {
      fbbr_queue_servo_q_median_history_s_.pop_front();
    }
  }
  stats.delivery_median_bps = Median(
      fbbr_queue_servo_delivery_samples_bps_);
  const uint64_t congestion_bytes = fbbr_queue_servo_acked_bytes_ +
                                    fbbr_queue_servo_lost_bytes_;
  stats.loss_ratio = congestion_bytes > 0
      ? static_cast<double>(fbbr_queue_servo_lost_bytes_) / congestion_bytes
      : 0.0;
  stats.ecn_ratio = fbbr_queue_servo_acked_bytes_ > 0
      ? static_cast<double>(fbbr_queue_servo_ecn_bytes_) /
            fbbr_queue_servo_acked_bytes_
      : 0.0;

  FBBRQueueServoInput input;
  input.search_baseline_bps = FBBRSearchSearchBaselineBps(
      congestion_event.event_time);
  input.rtprop_s = fbbr_probe_signature_.rtprop_s;
  input.q_floor_s = stats.q_floor_s;
  input.q_median_s = stats.q_median_s;
  input.q_peak_s = stats.q_peak_s;
  input.queue_trend_s_per_s = stats.queue_trend_s_per_s;
  input.delivery_median_bps = stats.delivery_median_bps;
  input.loss_ratio = stats.loss_ratio;
  input.ecn_ratio = stats.ecn_ratio;
  input.sustainable_direction = fbbr_latest_sustainable_direction_;
  input.samples_sufficient = stats.samples_sufficient;
  input.in_recovery = InRecovery();
  input.flow_backlogged = !fbbr_last_sample_app_limited_;
  input.underload_evidence = fbbr_latest_sustainable_direction_ > 0.20 ||
      (stats.delivery_median_bps > 0.0 && input.search_baseline_bps > 0.0 &&
       stats.delivery_median_bps < 0.95 * input.search_baseline_bps);

  const double old_factor = fbbr_queue_servo_state_.factor;
  FBBRQueueServoDecision decision = FBBRQueueReserveServo::Update(
      fbbr_frequency_search_config_, input, &fbbr_queue_servo_state_);
  ++fbbr_queue_servo_updates_;
  if (decision.state == FBBRQueueServoState::kDrain ||
      decision.state == FBBRQueueServoState::kEmergencyDrain) {
    ++fbbr_queue_servo_drain_rtts_;
  } else if (decision.state == FBBRQueueServoState::kReserveRecovery ||
             decision.state == FBBRQueueServoState::kTargetBand) {
    ++fbbr_queue_servo_recovery_rtts_;
  }
  if (std::abs(decision.factor - old_factor) > 0.02 + 1e-9) {
    fbbr_queue_servo_transition_pending_ = true;
  }

  bool commit_applied = false;
  double commit_bps = 0.0;
  if (decision.baseline_commit_eligible &&
      decision.baseline_commit_bps > 0.0 &&
      decision.baseline_commit_bps < input.search_baseline_bps &&
      !fbbr_baseline_transition_pending_) {
    commit_bps = decision.baseline_commit_bps;
    FBBRSearchStartBaselineTransition(congestion_event.event_time, commit_bps);
    fbbr_search_state_.current_search_bps = commit_bps;
    fbbr_search_state_.pending_search_bps = commit_bps;
    fbbr_queue_servo_state_.factor = ClampValue(
        decision.factor * input.search_baseline_bps /
            std::max(commit_bps, 1.0),
        fbbr_frequency_search_config_.queue_servo_emergency_factor, 1.02);
    decision.factor = fbbr_queue_servo_state_.factor;
    decision.final_nonprobe_baseline_bps = commit_bps * decision.factor;
    decision.reason += "|QUEUE_SAFETY_BASELINE_COMMIT";
    commit_applied = true;
    ++fbbr_queue_servo_baseline_commits_;
  }
  fbbr_latest_queue_servo_decision_ = decision;
  FBBREmitQueueServoTrace(congestion_event.event_time, stats, decision,
                          commit_applied, commit_bps);

  fbbr_queue_servo_q_samples_s_.clear();
  fbbr_queue_servo_delivery_samples_bps_.clear();
  fbbr_queue_servo_acked_bytes_ = 0;
  fbbr_queue_servo_lost_bytes_ = 0;
  fbbr_queue_servo_ecn_bytes_ = 0;
}

void FBBRSender::FBBREmitQueueServoTrace(
    QuicTime now,
    const QueueServoFastStats& stats,
    const FBBRQueueServoDecision& decision,
    bool baseline_commit_applied,
    double baseline_commit_bps) const {
  if (!cruise_load_trace_cb_) return;
  const double time_s = static_cast<double>(
      (now - QuicTime::Zero()).ToMicroseconds()) / 1e6;
  std::ostringstream row;
  row << "fbbr,none,F-BBR," << time_s << "," << trace_flow_id_ << ","
      << cruise_id_ << "," << FBBRQueueServoStateName(decision.state) << ","
      << FBBRSearchSearchBaselineBps(now) << "," << decision.factor << ","
      << decision.final_nonprobe_baseline_bps << ","
      << stats.q_floor_s * 1e6 << "," << stats.q_median_s * 1e6 << ","
      << stats.q_peak_s * 1e6 << "," << decision.q_low_s * 1e6 << ","
      << decision.q_high_s * 1e6 << "," << decision.q_peak_cap_s * 1e6 << ","
      << stats.queue_trend_s_per_s << "," << stats.delivery_median_bps << ","
      << stats.loss_ratio << "," << stats.ecn_ratio << ","
      << decision.down_correction << "," << decision.up_correction << ","
      << decision.consecutive_drain_rtts << ","
      << (decision.baseline_commit_eligible ? 1 : 0) << ","
      << (baseline_commit_applied ? 1 : 0) << "," << baseline_commit_bps << ","
      << decision.reason;
  cruise_load_trace_cb_(time_s, time_s, 0.0, 0.0, 0.0, 0.0,
                        "FBBR_QUEUE_SERVO", false, row.str());
}

FbbrPhaseBinSample FBBRSender::FBBRFrequencySearchFinalizePhaseBin(
    const FbbrPhaseBinAccumulator& accumulator) const {
  FbbrPhaseBinSample sample;
  sample.bin_index = accumulator.bin_index;
  const int64_t bins_per_cycle =
      static_cast<int64_t>(fbbr_frequency_search_config_.phase_bins_per_cycle);
  sample.cycle_index = accumulator.bin_index / bins_per_cycle;
  sample.phase_bin_index = static_cast<uint32_t>(
      accumulator.bin_index % bins_per_cycle);
  sample.time_start_s = accumulator.time_start_s;
  sample.time_end_s = accumulator.time_end_s;
  const double duration_s = sample.time_end_s - sample.time_start_s;
  const double cruise_start_s =
      static_cast<double>((cruise_start_time_ - QuicTime::Zero())
                              .ToMicroseconds()) /
      1e6;
  const double relative_center_s =
      0.5 * (sample.time_start_s + sample.time_end_s) - cruise_start_s;
  sample.phase_rad = fbbr_probe_signature_.initial_phase_rad +
      2.0 * M_PI * relative_center_s /
          std::max(fbbr_probe_signature_.period_s, 1e-9);
  sample.code_sign = fbbr_probe_signature_.waveform == "sine"
      ? FBBRFrequencySearch::WalshSign(
            fbbr_probe_signature_.code_id, sample.cycle_index)
      : 1;
  sample.coded_excitation = FBBRFrequencySearch::ProbeWaveform(
      fbbr_probe_signature_, relative_center_s);
  sample.sent_bytes = accumulator.sent_bytes;
  sample.acked_bytes = accumulator.acked_bytes;
  sample.lost_bytes = accumulator.lost_bytes;
  sample.ecn_marked_bytes = accumulator.ecn_marked_bytes;
  if (accumulator.sent_bytes > 0) {
    sample.native_pacing_bps =
        accumulator.native_pacing_bps_bytes / accumulator.sent_bytes;
    sample.commanded_pacing_bps =
        accumulator.commanded_pacing_bps_bytes / accumulator.sent_bytes;
    sample.bytes_in_flight =
        accumulator.bytes_in_flight_bytes / accumulator.sent_bytes;
    sample.queue_servo_factor =
        accumulator.queue_servo_factor_bytes / accumulator.sent_bytes;
  }
  if (duration_s > 0.0) {
    sample.actual_send_bps = 8.0 * accumulator.sent_bytes / duration_s;
    sample.delivery_rate_bps = 8.0 * accumulator.acked_bytes / duration_s;
  }
  if (!accumulator.latest_rtt_samples_s.empty()) {
    sample.latest_rtt_s = Median(accumulator.latest_rtt_samples_s);
    sample.qdelay_s = std::max(
        0.0, sample.latest_rtt_s - fbbr_probe_signature_.rtprop_s);
    sample.rtt_valid = true;
  }
  const double congestion_bytes = static_cast<double>(
      accumulator.acked_bytes + accumulator.lost_bytes);
  sample.loss_ratio = congestion_bytes > 0.0
      ? accumulator.lost_bytes / congestion_bytes
      : 0.0;
  sample.ecn_ratio = accumulator.acked_bytes > 0
      ? static_cast<double>(accumulator.ecn_marked_bytes) /
            accumulator.acked_bytes
      : (accumulator.ecn_marked_bytes > 0 ? 1.0 : 0.0);
  const double activity_bytes = static_cast<double>(
      accumulator.sent_bytes + accumulator.acked_bytes);
  sample.app_limited_fraction = activity_bytes > 0.0
      ? (accumulator.app_limited_sent_bytes +
         accumulator.app_limited_acked_bytes) /
            activity_bytes
      : 1.0;
  sample.cwnd_limited_fraction = accumulator.sent_bytes > 0
      ? static_cast<double>(accumulator.cwnd_limited_sent_bytes) /
            accumulator.sent_bytes
      : 1.0;
  sample.recovery_fraction = activity_bytes > 0.0
      ? static_cast<double>(accumulator.recovery_sent_bytes +
                            accumulator.recovery_acked_bytes) /
            activity_bytes
      : 0.0;
  // Coverage expresses whether the sender actually realized this fixed time
  // bin.  A bin with no ACK remains a legitimate zero-delivery observation;
  // missing RTT is separately represented by rtt_valid and never zero-filled.
  sample.coverage = accumulator.send_events > 0 ? 1.0 : 0.0;
  sample.valid = duration_s > 0.0 && sample.native_pacing_bps > 0.0 &&
                 accumulator.send_events > 0;
  sample.phase_transition = accumulator.phase_transition;
  sample.queue_servo_transition = accumulator.queue_servo_transition;
  return sample;
}

void FBBRSender::FBBRFrequencySearchFinalizeReadyBlocks(QuicTime now,
                                                   bool include_tail) {
  if (!fbbr_probe_active_ || fbbr_period_.IsZero()) return;
  FBBRFrequencySearchEnsureBinsThrough(now);
  fbbr_phase_bins_.clear();
  fbbr_phase_bins_.reserve(fbbr_phase_bin_accumulators_.size());
  for (const auto& accumulator : fbbr_phase_bin_accumulators_) {
    fbbr_phase_bins_.push_back(FBBRFrequencySearchFinalizePhaseBin(accumulator));
  }
  FBBRSearchUpdateCarrierSense();
  const int64_t bins_per_cycle =
      static_cast<int64_t>(fbbr_frequency_search_config_.phase_bins_per_cycle);
  const double now_s = static_cast<double>(
      (now - QuicTime::Zero()).ToMicroseconds()) / 1e6;

  if (fbbr_frequency_search_config_.search_controller_enabled) {
    if (fbbr_frequency_search_config_.event_triggered_windows_enabled) {
      int64_t last_completed_bin = -1;
      for (const auto& bin : fbbr_phase_bins_) {
        if (bin.time_end_s <= now_s) {
          last_completed_bin = std::max(last_completed_bin, bin.bin_index);
        }
      }
      const int64_t completed_cycle =
          (last_completed_bin + 1) / bins_per_cycle - 1;
      auto analyze_window = [this, now, bins_per_cycle](
          int64_t first_bin, int64_t window_cycles,
          EventWindowState state, double stride_cycles,
          const std::string& stop_reason, bool force_uncertain) {
        FBBRFrequencySearchConfig window_config =
            fbbr_frequency_search_config_;
        window_config.analysis_cycles = static_cast<uint32_t>(window_cycles);
        window_config.min_valid_cycles = static_cast<uint32_t>(std::max<int64_t>(
            1, std::min<int64_t>(window_cycles,
                fbbr_frequency_search_config_.min_direction_cycles)));
        window_config.min_valid_analysis_cycles =
            window_config.min_valid_cycles;
        FbbrOperatingPointBlockResult result = FBBRFrequencySearch::AnalyzeBlock(
            window_config, fbbr_probe_signature_, fbbr_phase_bins_, first_bin,
            window_cycles * bins_per_cycle, fbbr_next_block_id_,
            fbbr_previous_block_delay_s_);
        result.event_window_id = fbbr_next_event_window_id_++;
        result.trigger_cycle_id = fbbr_trigger_cycle_id_;
        result.capture_start_s = result.start_time_s;
        result.capture_end_s = result.end_time_s;
        result.window_length_cycles = static_cast<double>(window_cycles);
        result.window_stride_cycles = stride_cycles;
        result.sequential_stop_reason = stop_reason;
        result.event_window_state = state;
        result.trigger_source = fbbr_active_trigger_source_;
        result.trigger_cycle_excluded_from_score =
            first_bin >= (fbbr_trigger_cycle_id_ + 1) * bins_per_cycle;
        result.overlap_fraction = state == EventWindowState::kContinuousTrack
            ? std::max(0.0, 1.0 - stride_cycles /
                  std::max(result.window_length_cycles, 1.0))
            : 0.0;
        result.independent_for_control =
            fbbr_last_control_window_start_bin_ < 0 ||
            first_bin - fbbr_last_control_window_start_bin_ >=
                static_cast<int64_t>(std::ceil(
                    fbbr_frequency_search_config_.control_decision_stride_cycles *
                    bins_per_cycle));
        result.independent_for_trusted =
            fbbr_last_trusted_window_start_bin_ < 0 ||
            first_bin - fbbr_last_trusted_window_start_bin_ >=
                std::max<int64_t>(
                    window_cycles * bins_per_cycle / 2,
                    static_cast<int64_t>(
                        fbbr_frequency_search_config_.
                            trusted_independent_stride_cycles) *
                        bins_per_cycle);
        result.lockable_score = result.lockable_score &&
            window_cycles >= static_cast<int64_t>(
                fbbr_frequency_search_config_.min_score_cycles);
        if (!result.independent_for_trusted || !result.lockable_score) {
          result.candidate.valid = false;
          result.candidate.invalid_reason = !result.lockable_score
              ? "score_cycles_insufficient"
              : "overlap_not_independent_for_trusted";
        }
        if (force_uncertain) {
          result.classification = FbbrOperatingPointClassification::kInvalid;
          result.candidate.valid = false;
          result.candidate.invalid_reason = "sequential_uncertain";
          result.invalid_reason = AppendReason(result.invalid_reason,
                                               "sequential_uncertain");
        }
        if (!fbbr_search_state_.is_pulser) {
          result.classification = FbbrOperatingPointClassification::kInvalid;
          result.candidate.valid = false;
          result.candidate.invalid_reason = "watcher_observation";
          result.invalid_reason = AppendReason(result.invalid_reason,
                                               "watcher_observation");
        }
        if (fbbr_block_invalidated_) {
          result.classification = FbbrOperatingPointClassification::kInvalid;
          result.measurement_confidence = 0.0;
          result.candidate.valid = false;
          result.candidate.invalid_reason = "block_state_changed";
          result.invalid_reason = AppendReason(result.invalid_reason,
                                               "block_state_changed");
          fbbr_block_invalidated_ = false;
        }
        fbbr_previous_block_delay_s_ = result.selected_delay_s;
        fbbr_block_results_.push_back(result);
        if (fbbr_frequency_search_config_.trace_verbosity >= 2) {
          const int64_t last_bin = first_bin + window_cycles * bins_per_cycle;
          for (int64_t index = first_bin; index < last_bin; ++index) {
            FBBRFrequencySearchEmitBinTrace(
                fbbr_phase_bins_[static_cast<size_t>(index)],
                fbbr_next_block_id_);
          }
        }
        FBBRSearchUpdateRtpropAnchor(
            result, first_bin, window_cycles * bins_per_cycle);
        ++fbbr_next_block_id_;
        FBBRSearchApplyWindowDecision(now, result, first_bin);
        if (result.independent_for_control) {
          fbbr_last_control_window_start_bin_ = first_bin;
        }
        if (!fbbr_control_decisions_.empty() &&
            fbbr_control_decisions_.back().window_candidate_valid &&
            result.independent_for_trusted) {
          fbbr_last_trusted_window_start_bin_ = first_bin;
        }
        FBBRFrequencySearchEmitBlockTrace(result);
        return !fbbr_control_decisions_.empty() &&
            (std::abs(fbbr_control_decisions_.back().applied_next_baseline_bps -
                      fbbr_control_decisions_.back().baseline_before_bps) /
                 std::max(fbbr_control_decisions_.back().baseline_before_bps,
                          1.0) > 1e-6 ||
             fbbr_control_decisions_.back().hard_loss_abort);
      };

      while (fbbr_next_trigger_cycle_ >= 0 &&
             fbbr_next_trigger_cycle_ <= completed_cycle) {
        const int64_t cycle = fbbr_next_trigger_cycle_;
        FBBRTriggerCycleResult trigger =
            FBBRFrequencySearch::AnalyzeTriggerCycle(
                fbbr_frequency_search_config_, fbbr_probe_signature_,
                fbbr_phase_bins_, cycle, fbbr_event_window_state_,
                fbbr_search_state_.is_pulser);
        fbbr_latest_trigger_cycle_result_ = trigger;
        ++fbbr_dual_trigger_attempts_;
        if (trigger.combined_trigger_source == "DELIVERY_ONLY") {
          ++fbbr_delivery_triggers_;
        } else if (trigger.combined_trigger_source == "QUEUE_ONLY") {
          ++fbbr_queue_triggers_;
        } else if (trigger.combined_trigger_source == "BOTH") {
          ++fbbr_both_triggers_;
        } else if (trigger.combined_trigger_source == "HARD_SAFETY_ONLY") {
          ++fbbr_hard_safety_events_;
        }
        bool restart_carrier = false;

        if (fbbr_event_window_state_ ==
            EventWindowState::kPostBaselineSettling) {
          if (fbbr_settling_cycles_remaining_ > 0) {
            --fbbr_settling_cycles_remaining_;
          }
          trigger.trigger_pass = false;
          trigger.continue_pass = false;
          trigger.trigger_reason = "post_baseline_settling";
          if (fbbr_settling_cycles_remaining_ == 0) {
            fbbr_event_window_state_ = EventWindowState::kIdleListen;
          }
        } else if (fbbr_event_window_state_ == EventWindowState::kIdleListen ||
                   fbbr_event_window_state_ == EventWindowState::kPaused) {
          if (trigger.trigger_pass) {
            fbbr_trigger_cycle_id_ = cycle;
            fbbr_active_trigger_source_ = trigger.combined_trigger_source;
            fbbr_capture_start_bin_ = (cycle + 1) * bins_per_cycle;
            fbbr_event_window_state_ = EventWindowState::kCapture;
            fbbr_bad_cycle_streak_ = 0;
            fbbr_no_trigger_streak_ = 0;
            fbbr_have_previous_sequential_result_ = false;
            trigger.window_state = EventWindowState::kTriggerArmed;
          } else {
            ++fbbr_no_trigger_streak_;
            if (fbbr_event_window_state_ == EventWindowState::kPaused) {
              trigger.pause_reason = "periodic_response_absent";
            }
          }
        } else if (fbbr_event_window_state_ == EventWindowState::kCapture) {
          fbbr_bad_cycle_streak_ = trigger.continue_pass
              ? 0 : fbbr_bad_cycle_streak_ + 1;
          const int64_t capture_start_cycle =
              fbbr_capture_start_bin_ / bins_per_cycle;
          const int64_t capture_cycles = cycle - capture_start_cycle + 1;
          if (capture_cycles >= 3) {
            FBBRFrequencySearchConfig sequential_config =
                fbbr_frequency_search_config_;
            sequential_config.min_valid_analysis_cycles =
                static_cast<uint32_t>(capture_cycles);
            FbbrOperatingPointBlockResult sequential =
                FBBRFrequencySearch::AnalyzeBlock(
                    sequential_config, fbbr_probe_signature_, fbbr_phase_bins_,
                    fbbr_capture_start_bin_, capture_cycles * bins_per_cycle,
                    fbbr_next_block_id_, fbbr_previous_block_delay_s_);
            const int sign = sequential.direction_score > 0.0 ? 1 :
                             sequential.direction_score < 0.0 ? -1 : 0;
            const bool direction_stable = fbbr_have_previous_sequential_result_ &&
                sign != 0 && sign == fbbr_previous_sequential_direction_sign_;
            const bool direction_evidence =
                sequential.gradient_ci90_low > 0.0 ||
                sequential.gradient_ci90_high < 0.0 ||
                sequential.underload_evidence >=
                    fbbr_frequency_search_config_.direction_evidence_min ||
                sequential.overload_evidence >=
                    fbbr_frequency_search_config_.direction_evidence_min;
            const bool direction_ready = capture_cycles >=
                    static_cast<int64_t>(
                        fbbr_frequency_search_config_.min_direction_cycles) &&
                direction_stable && direction_evidence;
            const bool score_stable = fbbr_have_previous_sequential_result_ &&
                sequential.classification ==
                    fbbr_previous_sequential_classification_ &&
                std::abs(sequential.target_score -
                         fbbr_previous_sequential_score_) <=
                    fbbr_frequency_search_config_.sequential_score_delta_max;
            const bool score_ready = capture_cycles >=
                    static_cast<int64_t>(
                        fbbr_frequency_search_config_.min_score_cycles) &&
                score_stable && sequential.measurement_confidence >=
                    fbbr_frequency_search_config_.measurement_confidence_update_min;
            const bool reached_max = capture_cycles >=
                static_cast<int64_t>(
                    fbbr_frequency_search_config_.max_window_cycles);
            fbbr_previous_sequential_direction_sign_ = sign;
            fbbr_previous_sequential_classification_ = sequential.classification;
            fbbr_previous_sequential_score_ = sequential.target_score;
            fbbr_have_previous_sequential_result_ = true;
            if (direction_ready || score_ready || reached_max) {
              const bool changed = analyze_window(
                  fbbr_capture_start_bin_, capture_cycles,
                  EventWindowState::kCapture, 0.0,
                  direction_ready ? "DIRECTION_CI_STABLE" :
                  score_ready ? "SCORE_AND_CLASS_STABLE" :
                                "MAX_CYCLES_UNCERTAIN",
                  reached_max && !direction_ready && !score_ready);
              if (changed) {
                fbbr_event_window_state_ =
                    EventWindowState::kPostBaselineSettling;
                fbbr_settling_cycles_remaining_ =
                    fbbr_frequency_search_config_.post_update_settling_cycles;
                fbbr_capture_start_bin_ = -1;
                fbbr_next_tracking_start_bin_ = -1;
              } else if (trigger.continue_pass) {
                fbbr_event_window_state_ = EventWindowState::kContinuousTrack;
                const int64_t stride_bins = std::max<int64_t>(
                    1, static_cast<int64_t>(std::llround(
                        fbbr_frequency_search_config_.tracking_stride_cycles *
                        bins_per_cycle)));
                fbbr_next_tracking_start_bin_ =
                    fbbr_capture_start_bin_ + stride_bins;
              } else {
                fbbr_event_window_state_ = EventWindowState::kPaused;
              }
            }
          }
        } else if (fbbr_event_window_state_ ==
                   EventWindowState::kContinuousTrack) {
          fbbr_bad_cycle_streak_ = trigger.continue_pass
              ? 0 : fbbr_bad_cycle_streak_ + 1;
          if (fbbr_bad_cycle_streak_ >=
              fbbr_frequency_search_config_.bad_cycles_to_pause) {
            fbbr_event_window_state_ = EventWindowState::kPaused;
            trigger.pause_reason = "two_bad_cycles";
            fbbr_next_tracking_start_bin_ = -1;
          } else {
            const int64_t tracking_cycles =
                fbbr_frequency_search_config_.tracking_window_cycles;
            const int64_t tracking_bins = tracking_cycles * bins_per_cycle;
            const int64_t completed_bins = (completed_cycle + 1) * bins_per_cycle;
            const int64_t stride_bins = std::max<int64_t>(
                1, static_cast<int64_t>(std::llround(
                    fbbr_frequency_search_config_.tracking_stride_cycles *
                    bins_per_cycle)));
            while (fbbr_next_tracking_start_bin_ >= 0 &&
                   fbbr_next_tracking_start_bin_ + tracking_bins <=
                       completed_bins) {
              const bool changed = analyze_window(
                  fbbr_next_tracking_start_bin_, tracking_cycles,
                  EventWindowState::kContinuousTrack,
                  fbbr_frequency_search_config_.tracking_stride_cycles,
                  "CONTINUOUS_TRACK", false);
              if (changed) {
                fbbr_event_window_state_ =
                    EventWindowState::kPostBaselineSettling;
                fbbr_settling_cycles_remaining_ =
                    fbbr_frequency_search_config_.post_update_settling_cycles;
                fbbr_next_tracking_start_bin_ = -1;
                break;
              }
              fbbr_next_tracking_start_bin_ += stride_bins;
            }
          }
        }

        if (fbbr_search_state_.is_pulser) {
          ++fbbr_pulser_lease_age_cycles_;
          if (fbbr_search_state_.pulser_lease_remaining > 0) {
            --fbbr_search_state_.pulser_lease_remaining;
          }
          if (fbbr_pulser_lease_age_cycles_ >=
                  fbbr_frequency_search_config_.max_pulser_lease_cycles &&
              fbbr_event_window_state_ != EventWindowState::kCapture &&
              fbbr_event_window_state_ != EventWindowState::kContinuousTrack) {
            fbbr_search_state_.is_pulser = false;
            fbbr_search_state_.state = FBBRSearchState::kWatcher;
          }
        }

        if ((fbbr_event_window_state_ == EventWindowState::kIdleListen ||
             fbbr_event_window_state_ == EventWindowState::kPaused) &&
            fbbr_search_state_.is_pulser &&
            fbbr_no_trigger_streak_ >=
                fbbr_frequency_search_config_.min_direction_cycles &&
            (trigger.weak_periodic_response ||
             trigger.trigger_reason == "actual_input_unmeasurable")) {
          const double old_period = fbbr_probe_signature_.period_s;
          const double new_period = ClampValue(
              old_period * fbbr_frequency_search_config_.period_increase_factor,
              fbbr_frequency_search_config_.carrier_period_min_s,
              fbbr_frequency_search_config_.carrier_period_max_s);
          if (new_period > old_period + 1e-9) {
            fbbr_period_ = TimeDelta::FromMicroseconds(
                static_cast<int64_t>(std::llround(new_period * 1e6)));
            fbbr_probe_signature_.period_s = new_period;
            fbbr_probe_signature_.frequency_hz = 1.0 / new_period;
            fbbr_probe_signature_.period_rtts = static_cast<uint32_t>(
                std::max<int64_t>(1, static_cast<int64_t>(std::llround(
                    new_period /
                    std::max(fbbr_probe_signature_.rtprop_s, 1e-9)))));
            fbbr_search_state_.carrier_period_s = new_period;
            fbbr_period_reselected_ = true;
            ++fbbr_period_scan_count_;
            restart_carrier = true;
          } else {
            const double maximum_factor = 1.0 +
                fbbr_frequency_search_config_.max_amplitude_change_per_update;
            const double factor = std::min(
                fbbr_frequency_search_config_.amplitude_increase_factor,
                maximum_factor);
            const double queue_budget_limit = 2.0 * M_PI *
                fbbr_frequency_search_config_.acquire_probe_budget_bdp /
                std::max<uint32_t>(1, fbbr_probe_signature_.period_rtts);
            const double new_amplitude = ClampValue(
                factor * fbbr_probe_signature_.amplitude_ratio,
                fbbr_frequency_search_config_.probe_min_amplitude_ratio,
                std::min(fbbr_frequency_search_config_.probe_max_amplitude_ratio,
                         queue_budget_limit));
            if (new_amplitude > fbbr_probe_signature_.amplitude_ratio + 1e-9) {
              fbbr_probe_signature_.amplitude_ratio = new_amplitude;
              fbbr_search_state_.current_amplitude_ratio = new_amplitude;
              restart_carrier = true;
            }
          }
        }

        trigger.window_state = fbbr_event_window_state_;
        FBBREmitTriggerCycleTrace(trigger);
        ++fbbr_next_trigger_cycle_;
        if (restart_carrier) {
          cruise_start_time_ = now;
          fbbr_phase_bin_accumulators_.clear();
          fbbr_phase_bins_.clear();
          fbbr_previous_block_delay_s_ =
              std::numeric_limits<double>::quiet_NaN();
          fbbr_event_window_state_ = EventWindowState::kPostBaselineSettling;
          fbbr_settling_cycles_remaining_ =
              fbbr_frequency_search_config_.post_update_settling_cycles;
          fbbr_no_trigger_streak_ = 0;
          fbbr_bad_cycle_streak_ = 0;
            fbbr_trigger_cycle_id_ = -1;
            fbbr_active_trigger_source_ = "NONE";
          fbbr_capture_start_bin_ = -1;
          fbbr_next_tracking_start_bin_ = -1;
          fbbr_next_trigger_cycle_ = std::max<int64_t>(
              fbbr_frequency_search_config_.carrier_sensing_cycles,
              fbbr_frequency_search_config_.post_update_settling_cycles);
          FBBRFrequencySearchEnsureBinsThrough(now);
          return;
        }
      }
      (void)include_tail;
      return;
    }

    const int64_t block_bins = static_cast<int64_t>(
        fbbr_frequency_search_config_.analysis_cycles_per_window) * bins_per_cycle;
    while (fbbr_next_analysis_output_bin_ >= 0 && block_bins > 0) {
      const int64_t first_bin = fbbr_next_analysis_output_bin_;
      const int64_t last_bin = first_bin + block_bins - 1;
      const FbbrPhaseBinSample* last =
          last_bin >= 0 &&
                  static_cast<size_t>(last_bin) < fbbr_phase_bins_.size()
              ? &fbbr_phase_bins_[static_cast<size_t>(last_bin)]
              : nullptr;
      if (last == nullptr || last->time_end_s > now_s) break;
      FBBRFrequencySearchConfig window_config = fbbr_frequency_search_config_;
      window_config.analysis_cycles =
          window_config.analysis_cycles_per_window;
      window_config.min_valid_cycles =
          window_config.min_valid_analysis_cycles;
      FbbrOperatingPointBlockResult result = FBBRFrequencySearch::AnalyzeBlock(
          window_config, fbbr_probe_signature_, fbbr_phase_bins_, first_bin,
          block_bins, fbbr_next_block_id_, fbbr_previous_block_delay_s_);
      if (!fbbr_search_state_.is_pulser) {
        result.classification = FbbrOperatingPointClassification::kInvalid;
        result.candidate.valid = false;
        result.candidate.invalid_reason = "watcher_observation";
        result.invalid_reason = AppendReason(result.invalid_reason,
                                             "watcher_observation");
      }
      if (fbbr_block_invalidated_) {
        result.classification = FbbrOperatingPointClassification::kInvalid;
        result.measurement_confidence = 0.0;
        result.candidate.valid = false;
        result.candidate.invalid_reason = "block_state_changed";
        result.invalid_reason = AppendReason(result.invalid_reason,
                                             "block_state_changed");
        fbbr_block_invalidated_ = false;
      }
      fbbr_previous_block_delay_s_ = result.selected_delay_s;
      fbbr_block_results_.push_back(result);
      if (fbbr_frequency_search_config_.trace_verbosity >= 2) {
        for (int64_t index = first_bin; index <= last_bin; ++index) {
          FBBRFrequencySearchEmitBinTrace(
              fbbr_phase_bins_[static_cast<size_t>(index)],
              fbbr_next_block_id_);
        }
      }
      FBBRSearchUpdateRtpropAnchor(result, first_bin, block_bins);
      ++fbbr_next_block_id_;
      fbbr_next_analysis_output_bin_ = -1;
      FBBRSearchApplyWindowDecision(now, result, first_bin);
      FBBRFrequencySearchEmitBlockTrace(result);
      // ApplyWindowDecision schedules exactly one later control window.  It
      // cannot already be complete at this ACK event, so avoid reusing bins
      // from a different fixed-baseline interval.
      break;
    }
    (void)include_tail;
    return;
  }
  const int64_t warmup_bins =
      fbbr_frequency_search_config_.warmup_cycles * bins_per_cycle;
  const int64_t nominal_feedback_bins = static_cast<int64_t>(std::llround(
      fbbr_probe_signature_.rtprop_s /
      std::max(fbbr_probe_signature_.period_s / bins_per_cycle, 1e-9)));
  const int64_t first_analysis_bin = warmup_bins + nominal_feedback_bins;
  const int64_t block_bins =
      fbbr_frequency_search_config_.analysis_cycles * bins_per_cycle;
  while (true) {
    const int64_t first_bin =
        first_analysis_bin +
        static_cast<int64_t>(fbbr_next_block_id_) * block_bins;
    const int64_t last_bin = first_bin + block_bins - 1;
    const FbbrPhaseBinSample* last =
        last_bin >= 0 && static_cast<size_t>(last_bin) < fbbr_phase_bins_.size()
            ? &fbbr_phase_bins_[static_cast<size_t>(last_bin)]
            : nullptr;
    // The output block already starts one nominal RTprop after the aligned
    // input code block.  Once its final phase bin is closed no future sample is
    // needed: delay refinement only looks backward into send-side bins.
    if (last == nullptr || last->time_end_s > now_s) {
      break;
    }
    FbbrOperatingPointBlockResult result = FBBRFrequencySearch::AnalyzeBlock(
        fbbr_frequency_search_config_, fbbr_probe_signature_, fbbr_phase_bins_,
        first_bin, block_bins, fbbr_next_block_id_,
        fbbr_previous_block_delay_s_);
    if (fbbr_block_invalidated_) {
      result.classification = FbbrOperatingPointClassification::kInvalid;
      result.measurement_confidence = 0.0;
      result.candidate.valid = false;
      result.candidate.invalid_reason = "block_state_changed";
      result.invalid_reason = AppendReason(result.invalid_reason,
                                           "block_state_changed");
      fbbr_block_invalidated_ = false;
    }
    fbbr_previous_block_delay_s_ = result.selected_delay_s;
    fbbr_block_results_.push_back(result);
    if (fbbr_frequency_search_config_.trace_verbosity >= 2) {
      for (int64_t index = first_bin; index <= last_bin; ++index) {
        FBBRFrequencySearchEmitBinTrace(
            fbbr_phase_bins_[static_cast<size_t>(index)],
            fbbr_next_block_id_);
      }
    }
    FBBRFrequencySearchEmitBlockTrace(result);
    ++fbbr_next_block_id_;
  }

  // Shadow windows are deliberately kept out of fbbr_block_results_.  They
  // use a private config copy and are emitted only as diagnostics, so enabling
  // this path cannot affect consensus, history, pacing, or CRUISE lifetime.
  if (fbbr_frequency_search_config_.validation_shadow_windows &&
      fbbr_frequency_search_config_.validation_shadow_analysis_cycles > 0 &&
      fbbr_frequency_search_config_.validation_shadow_stride_cycles > 0) {
    const int64_t shadow_bins = static_cast<int64_t>(
        fbbr_frequency_search_config_.validation_shadow_analysis_cycles) * bins_per_cycle;
    const int64_t shadow_stride_bins = static_cast<int64_t>(
        fbbr_frequency_search_config_.validation_shadow_stride_cycles) * bins_per_cycle;
    while (true) {
      const int64_t first_bin = first_analysis_bin +
          static_cast<int64_t>(fbbr_next_shadow_window_id_) * shadow_stride_bins;
      const int64_t last_bin = first_bin + shadow_bins - 1;
      const FbbrPhaseBinSample* last =
          last_bin >= 0 && static_cast<size_t>(last_bin) < fbbr_phase_bins_.size()
              ? &fbbr_phase_bins_[static_cast<size_t>(last_bin)]
              : nullptr;
      if (last == nullptr || last->time_end_s > now_s) break;
      FBBRFrequencySearchConfig shadow_config = fbbr_frequency_search_config_;
      shadow_config.analysis_cycles =
          shadow_config.validation_shadow_analysis_cycles;
      shadow_config.min_valid_cycles =
          shadow_config.validation_shadow_min_valid_cycles;
      FbbrOperatingPointBlockResult result = FBBRFrequencySearch::AnalyzeBlock(
          shadow_config, fbbr_probe_signature_, fbbr_phase_bins_, first_bin,
          shadow_bins, fbbr_next_shadow_window_id_,
          std::numeric_limits<double>::quiet_NaN());
      result.candidate.valid = false;
      result.candidate.bandwidth_bps = 0.0;
      result.candidate.invalid_reason = "validation_shadow_window";
      FBBRFrequencySearchEmitShadowWindowTrace(result);
      ++fbbr_next_shadow_window_id_;
    }
  }

  if (!include_tail) return;
  const int64_t tail_first =
      first_analysis_bin +
      static_cast<int64_t>(fbbr_next_block_id_) * block_bins;
  int64_t completed_tail_bins = 0;
  for (int64_t index = tail_first;
       index < static_cast<int64_t>(fbbr_phase_bins_.size()); ++index) {
    if (fbbr_phase_bins_[static_cast<size_t>(index)].time_end_s <= now_s) {
      ++completed_tail_bins;
    }
  }
  if (completed_tail_bins > 0) {
    FbbrOperatingPointBlockResult tail;
    tail.cruise_id = fbbr_probe_signature_.cruise_id;
    tail.block_id = fbbr_next_block_id_;
    tail.frequency_hz = fbbr_probe_signature_.frequency_hz;
    tail.period_rtts = fbbr_probe_signature_.period_rtts;
    tail.code_id = fbbr_probe_signature_.code_id;
    tail.target_amplitude_ratio = fbbr_probe_signature_.amplitude_ratio;
    tail.rtprop_frozen_s = fbbr_probe_signature_.rtprop_s;
    tail.valid_cycles = static_cast<uint32_t>(
        completed_tail_bins / bins_per_cycle);
    tail.classification = FbbrOperatingPointClassification::kInvalid;
    tail.invalid_reason = "incomplete_tail";
    if (tail_first < static_cast<int64_t>(fbbr_phase_bins_.size())) {
      tail.start_time_s = fbbr_phase_bins_[tail_first].time_start_s;
      tail.end_time_s = fbbr_phase_bins_[tail_first + completed_tail_bins - 1]
                            .time_end_s;
    }
    FBBRFrequencySearchEmitBlockTrace(tail);
  }
}

void FBBRSender::FBBRSearchUpdateRtpropAnchor(
    const FbbrOperatingPointBlockResult& result,
    int64_t first_output_bin,
    int64_t output_bin_count) {
  if (result.classification == FbbrOperatingPointClassification::kInvalid ||
      result.classification == FbbrOperatingPointClassification::kDynamic ||
      result.loss_ratio > 0.0) {
    return;
  }
  std::vector<double> negative_rtt_s;
  const int64_t end = std::min<int64_t>(
      first_output_bin + output_bin_count,
      static_cast<int64_t>(fbbr_phase_bins_.size()));
  for (int64_t index = std::max<int64_t>(0, first_output_bin);
       index < end; ++index) {
    const FbbrPhaseBinSample& bin =
        fbbr_phase_bins_[static_cast<size_t>(index)];
    if (bin.rtt_valid && bin.coded_excitation < 0.0 &&
        bin.actual_send_bps <= 1.01 * bin.native_pacing_bps &&
        bin.app_limited_fraction <= 0.0) {
      negative_rtt_s.push_back(bin.latest_rtt_s);
    }
  }
  if (negative_rtt_s.size() < 4) return;
  std::sort(negative_rtt_s.begin(), negative_rtt_s.end());
  const size_t p05_index = static_cast<size_t>(std::floor(
      0.05 * static_cast<double>(negative_rtt_s.size() - 1)));
  const TimeDelta candidate = TimeDelta::FromMicroseconds(
      static_cast<int64_t>(std::llround(negative_rtt_s[p05_index] * 1e6)));
  if (!candidate.IsZero()) {
    const bool lower = fbbr_opi_rtprop_anchor_.IsZero() ||
                       candidate < fbbr_opi_rtprop_anchor_;
    const bool confirms_anchor = !fbbr_opi_rtprop_anchor_.IsZero() &&
        candidate.ToMicroseconds() <=
            static_cast<int64_t>(std::llround(
                1.02 * fbbr_opi_rtprop_anchor_.ToMicroseconds()));
    if (lower) {
      fbbr_opi_rtprop_anchor_ = candidate;
      fbbr_opi_rtprop_source_ = "NEGATIVE_HALF_P05";
      fbbr_probe_signature_.rtprop_s = static_cast<double>(
          candidate.ToMicroseconds()) / 1e6;
    } else if (confirms_anchor) {
      fbbr_opi_rtprop_source_ = "NEGATIVE_HALF_CONFIRMED";
    }
    if (lower || confirms_anchor) {
      fbbr_opi_rtprop_confidence_ = std::min(
          1.0, fbbr_opi_rtprop_confidence_ + 0.20);
    }
  }
}

void FBBRSender::FBBRSearchApplyWindowDecision(
    QuicTime now,
    const FbbrOperatingPointBlockResult& result,
    int64_t first_output_bin) {
  FbbrOperatingPointBlockResult control_result = result;
  control_result.rtprop_confidence = fbbr_opi_rtprop_confidence_;
  if (!fbbr_search_state_.is_pulser) {
    control_result.collision_suspected = false;
  }
  FBBRWindowControlDecision decision = FBBRSearchController::Decide(
      fbbr_frequency_search_config_, control_result,
      static_cast<double>(BandwidthEstimate().ToBitsPerSecond()),
      static_cast<double>(fbbr_opi_rtprop_anchor_.ToMicroseconds()) / 1e6,
      &fbbr_search_state_);
  if (control_result.classification !=
          FbbrOperatingPointClassification::kInvalid &&
      control_result.classification !=
          FbbrOperatingPointClassification::kDynamic) {
    fbbr_latest_sustainable_direction_ = control_result.frequency_direction;
  }
  fbbr_control_decisions_.push_back(decision);

  const bool pulser_collision =
      fbbr_search_state_.is_pulser && result.collision_suspected;
  fbbr_search_state_.collision_suspected = pulser_collision;
  if (pulser_collision) {
    ++fbbr_search_state_.collision_count;
    ++fbbr_search_state_.search_generation;
    fbbr_search_state_.is_pulser = false;
    fbbr_search_state_.carrier_detected = false;
    fbbr_search_state_.carrier_detection_streak = 0;
    fbbr_search_state_.state = FBBRSearchState::kPulserElection;
    const double elapsed_s = static_cast<double>(
        (now - cruise_start_time_).ToMicroseconds()) / 1e6;
    fbbr_search_state_.election_start_cycle = static_cast<int64_t>(std::floor(
        elapsed_s / std::max(fbbr_probe_signature_.period_s, 1e-9)));
    fbbr_search_state_.election_backoff_cycles =
        FBBRSearchElectionBackoff(fbbr_search_state_.search_generation);
  }

  const bool input_snr_low =
      result.invalid_reason.find("input_snr_low") != std::string::npos;
  if (!fbbr_frequency_search_config_.event_triggered_windows_enabled &&
      input_snr_low &&
      fbbr_frequency_search_config_.analysis_cycles_per_window < 16) {
    fbbr_frequency_search_config_.analysis_cycles_per_window = std::min<uint32_t>(
        16, fbbr_frequency_search_config_.analysis_cycles_per_window + 4);
    decision.update_reason += "|MEMORY_INCREASE";
    fbbr_control_decisions_.back() = decision;
  }

  if (!fbbr_frequency_search_config_.event_triggered_windows_enabled &&
      fbbr_search_state_.is_pulser &&
      fbbr_search_state_.pulser_lease_remaining > 0) {
    --fbbr_search_state_.pulser_lease_remaining;
    if (fbbr_search_state_.pulser_lease_remaining == 0) {
      fbbr_search_state_.is_pulser = false;
      fbbr_search_state_.carrier_detected = false;
      fbbr_search_state_.carrier_detection_streak = 0;
      ++fbbr_search_state_.search_generation;
      fbbr_search_state_.state = FBBRSearchState::kPulserElection;
      const double elapsed_s = static_cast<double>(
          (now - cruise_start_time_).ToMicroseconds()) / 1e6;
      fbbr_search_state_.election_start_cycle = static_cast<int64_t>(std::floor(
          elapsed_s / std::max(fbbr_probe_signature_.period_s, 1e-9)));
      fbbr_search_state_.election_backoff_cycles =
          FBBRSearchElectionBackoff(fbbr_search_state_.search_generation);
      fbbr_search_state_.pulser_lease_remaining =
          fbbr_frequency_search_config_.pulser_lease_decisions;
      fbbr_search_state_.watcher_cooldown_cycles =
          fbbr_frequency_search_config_.pulser_lease_decisions;
    }
  } else if (!fbbr_frequency_search_config_.event_triggered_windows_enabled &&
             !fbbr_search_state_.is_pulser) {
    if (fbbr_search_state_.watcher_cooldown_cycles > 0) {
      --fbbr_search_state_.watcher_cooldown_cycles;
    } else {
      fbbr_search_state_.carrier_detected = false;
      fbbr_search_state_.carrier_detection_streak = 0;
      ++fbbr_search_state_.search_generation;
      const double elapsed_s = static_cast<double>(
          (now - cruise_start_time_).ToMicroseconds()) / 1e6;
      fbbr_search_state_.election_start_cycle = static_cast<int64_t>(std::floor(
          elapsed_s / std::max(fbbr_probe_signature_.period_s, 1e-9)));
      fbbr_search_state_.election_backoff_cycles =
          FBBRSearchElectionBackoff(fbbr_search_state_.search_generation);
      fbbr_search_state_.watcher_cooldown_cycles =
          fbbr_frequency_search_config_.pulser_lease_decisions;
      fbbr_search_state_.state = FBBRSearchState::kPulserElection;
    }
  }

  if (!fbbr_frequency_search_config_.event_triggered_windows_enabled &&
      decision.request_period_increase && fbbr_period_scan_count_ < 3 &&
      !fbbr_frequency_search_config_.probe_period_rtt_slots.empty()) {
    const double old_period_s = fbbr_probe_signature_.period_s;
    fbbr_search_state_.carrier_scan_index =
        (fbbr_search_state_.carrier_scan_index + 1) %
        fbbr_frequency_search_config_.probe_period_rtt_slots.size();
    const double candidate_period_s =
        fbbr_frequency_search_config_.probe_period_rtt_slots[
            fbbr_search_state_.carrier_scan_index] *
        std::max(fbbr_probe_signature_.rtprop_s, 1e-9);
    const double new_period_s = ClampValue(
        candidate_period_s,
        fbbr_frequency_search_config_.carrier_period_min_s,
        fbbr_frequency_search_config_.carrier_period_max_s);
    if (std::abs(new_period_s - old_period_s) > 1e-9) {
      fbbr_period_reselected_ = true;
      ++fbbr_period_scan_count_;
      fbbr_period_ = TimeDelta::FromMicroseconds(
          static_cast<int64_t>(std::llround(new_period_s * 1e6)));
      fbbr_probe_signature_.period_s = new_period_s;
      fbbr_probe_signature_.frequency_hz = 1.0 / new_period_s;
      fbbr_probe_signature_.period_rtts = static_cast<uint32_t>(
          std::max<int64_t>(1, static_cast<int64_t>(std::llround(
              new_period_s /
              std::max(fbbr_probe_signature_.rtprop_s, 1e-9)))));
      fbbr_search_state_.carrier_period_s = new_period_s;
      fbbr_search_state_.control_window_index = 0;
      fbbr_search_state_.consecutive_invalid = 0;
      fbbr_search_state_.input_unrealized_streak = 0;
      fbbr_search_state_.state = FBBRSearchState::kAcquireInput;
      cruise_start_time_ = now;
      fbbr_phase_bin_accumulators_.clear();
      fbbr_phase_bins_.clear();
      fbbr_previous_block_delay_s_ =
          std::numeric_limits<double>::quiet_NaN();
      fbbr_baseline_transition_pending_ = false;
      fbbr_baseline_transition_start_ = QuicTime::Zero();
      fbbr_baseline_transition_end_ = QuicTime::Zero();
      fbbr_baseline_transition_from_bps_ =
          decision.applied_next_baseline_bps;
      fbbr_baseline_transition_to_bps_ =
          decision.applied_next_baseline_bps;
      const int64_t bins_per_cycle =
          fbbr_frequency_search_config_.phase_bins_per_cycle;
      const int64_t feedback_bins = static_cast<int64_t>(std::llround(
          fbbr_probe_signature_.rtprop_s /
          std::max(new_period_s / bins_per_cycle, 1e-9)));
      fbbr_next_analysis_output_bin_ =
          fbbr_frequency_search_config_.settling_cycles_per_window * bins_per_cycle +
          feedback_bins;
      FBBRFrequencySearchEnsureBinsThrough(now);
      return;
    }
  }

  double target_actual_amplitude =
      fbbr_frequency_search_config_.actual_amplitude_target_acquire;
  double queue_budget_bdp =
      fbbr_frequency_search_config_.acquire_probe_budget_bdp;
  switch (decision.state_after) {
    case FBBRSearchState::kSeek:
      target_actual_amplitude =
          fbbr_frequency_search_config_.actual_amplitude_target_seek;
      break;
    case FBBRSearchState::kTrack:
    case FBBRSearchState::kLockCandidate:
      target_actual_amplitude =
          fbbr_frequency_search_config_.actual_amplitude_target_track;
      queue_budget_bdp = fbbr_frequency_search_config_.track_probe_budget_bdp;
      break;
    case FBBRSearchState::kLocked:
      target_actual_amplitude =
          fbbr_frequency_search_config_.actual_amplitude_target_locked;
      queue_budget_bdp = fbbr_frequency_search_config_.locked_probe_budget_bdp;
      break;
    case FBBRSearchState::kDrain:
    case FBBRSearchState::kEmergencyDrain:
      target_actual_amplitude =
          fbbr_frequency_search_config_.actual_amplitude_target_drain;
      queue_budget_bdp = 0.0;
      break;
    default:
      break;
  }
  if (fbbr_search_state_.is_pulser && result.independent_for_control) {
    const double step = ClampValue(
        std::min(fbbr_frequency_search_config_.amplitude_step_max,
                 fbbr_frequency_search_config_.max_amplitude_change_per_update),
        0.0, 0.50);
    double factor = 1.0;
    if (decision.request_amplitude_decrease || decision.hard_loss_abort ||
        result.loss_ratio >= fbbr_frequency_search_config_.soft_loss_threshold ||
        result.ecn_ratio >= fbbr_frequency_search_config_.soft_ecn_threshold) {
      factor = 1.0 - step;
    } else if (result.actual_input_amplitude_ratio > 0.0 &&
               result.input_cycle_coherence >= 0.50 &&
               !result.collision_suspected) {
      factor = std::pow(
          target_actual_amplitude /
              std::max(result.actual_input_amplitude_ratio, 1e-6),
          ClampValue(fbbr_frequency_search_config_.amplitude_adaptation_mu, 0.0, 1.0));
      factor = ClampValue(factor, 1.0 - step, 1.0 + step);
    } else if (decision.request_amplitude_increase &&
               !result.collision_suspected) {
      factor = 1.0 + step;
    }
    const double waveform_integral_factor =
        fbbr_probe_signature_.waveform == "sine" ? M_PI : 2.0 * M_PI;
    const double queue_budget_limit = waveform_integral_factor *
        queue_budget_bdp /
        std::max<uint32_t>(1, fbbr_probe_signature_.period_rtts);
    const double amplitude_max = queue_budget_bdp > 0.0
        ? std::min(fbbr_frequency_search_config_.probe_max_amplitude_ratio,
                   queue_budget_limit)
        : fbbr_frequency_search_config_.probe_min_amplitude_ratio;
    fbbr_search_state_.current_amplitude_ratio = ClampValue(
        factor * fbbr_search_state_.current_amplitude_ratio,
        fbbr_frequency_search_config_.probe_min_amplitude_ratio,
        amplitude_max);
  }
  fbbr_probe_signature_.amplitude_ratio =
      fbbr_search_state_.current_amplitude_ratio;

  if (decision.trusted_bw_published &&
      !fbbr_trusted_published_in_cruise_) {
    trusted_bw_ = BandwidthFromBps(decision.trusted_bw_bps);
    trusted_bw_valid_ = !trusted_bw_.IsZero();
    trusted_bw_conf_ = decision.trusted_confidence;
    trusted_bw_source_ = "F_BBR_LOCK_CONSENSUS";
    trusted_bw_cruise_id_ = fbbr_probe_signature_.cruise_id;
    trusted_bw_age_cruises_ = 0;
    trusted_bw_fresh_ = trusted_bw_valid_;
    trusted_bw_application_valid_ = trusted_bw_valid_;
    trusted_bw_ready_for_post_cruise_ = trusted_bw_valid_;
    trusted_bw_application_phase_ = "POST_CRUISE_READY";
    trusted_bw_invalid_reason_ = "none";
    fbbr_trusted_published_in_cruise_ = trusted_bw_valid_;
  }

  if (fbbr_search_state_.control_window_index >=
      fbbr_frequency_search_config_.max_control_windows) {
    // The BBR phase machine may leave CRUISE at its normal boundary.  Keep
    // search_active/probe state intact so this is not observable as a search
    // termination and the next eligible CRUISE resumes acquisition.
    fbbr_probe_disabled_reason_ = "cruise_decision_budget_reached";
    return;
  }
  const double relative_change = decision.baseline_before_bps > 0.0
      ? std::abs(decision.applied_next_baseline_bps -
                 decision.baseline_before_bps) /
            decision.baseline_before_bps
      : 0.0;
  if (relative_change > 1e-6 || first_output_bin < 0) {
    FBBRSearchStartBaselineTransition(
        now, decision.applied_next_baseline_bps);
  } else if (!fbbr_frequency_search_config_.event_triggered_windows_enabled) {
    const int64_t stride_bins = static_cast<int64_t>(std::max<uint32_t>(
        1, fbbr_frequency_search_config_.decision_stride_cycles)) *
        fbbr_frequency_search_config_.phase_bins_per_cycle;
    fbbr_next_analysis_output_bin_ = first_output_bin + stride_bins;
  }
}

void FBBRSender::FBBRSearchEmitControlTrace(
    const FbbrOperatingPointBlockResult& /*result*/,
    const FBBRWindowControlDecision& /*decision*/) const {
  // Control fields are appended atomically to the production block row in
  // FBBRFrequencySearchEmitBlockTrace(), keeping one row per fixed-baseline window.
}

bool FBBRSender::FBBRFrequencySearchHasNativeChangeEvidence(
    double candidate_bps) const {
  if (!fbbr_history_valid_ || fbbr_history_bandwidth_bps_ <= 0.0 ||
      fbbr_native_bw_at_cruise_start_bps_ <= 0.0) {
    return false;
  }
  const double native_end =
      static_cast<double>(BandwidthEstimate().ToBitsPerSecond());
  const double native_change =
      (native_end - fbbr_native_bw_at_cruise_start_bps_) /
      fbbr_native_bw_at_cruise_start_bps_;
  const double native_vs_history_change =
      (fbbr_native_bw_at_cruise_start_bps_ - fbbr_history_bandwidth_bps_) /
      fbbr_history_bandwidth_bps_;
  const double candidate_change =
      (candidate_bps - fbbr_history_bandwidth_bps_) /
      fbbr_history_bandwidth_bps_;
  return (std::abs(native_change) >= 0.20 &&
          native_change * candidate_change > 0.0) ||
         (std::abs(native_vs_history_change) >= 0.20 &&
          native_vs_history_change * candidate_change > 0.0);
}

void FBBRSender::FBBRFrequencySearchPublishConsensus(
    const FbbrCruiseConsensusResult& consensus) {
  const FbbrHistoryUpdateResult update = FBBRFrequencySearch::StabilizeHistory(
      fbbr_frequency_search_config_, consensus, fbbr_history_valid_,
      fbbr_history_bandwidth_bps_,
      FBBRFrequencySearchHasNativeChangeEvidence(consensus.raw_candidate_bps));
  if (!update.valid) {
    ClearTrustedBw(update.invalid_reason.c_str());
    fbbr_history_update_action_ = update.action;
    return;
  }
  fbbr_history_valid_ = true;
  fbbr_history_bandwidth_bps_ = update.published_bps;
  fbbr_history_update_action_ = update.action;
  fbbr_trusted_bw_raw_candidate_bps_ = update.raw_candidate_bps;
  fbbr_trusted_bw_smoothed_bps_ = update.published_bps;
  fbbr_trusted_bw_robust_cv_ = consensus.robust_cv;
  fbbr_trusted_bw_relative_ci_width_ = consensus.relative_ci_width;
  fbbr_trusted_bw_cycle_count_ = consensus.cycle_count;
  fbbr_trusted_bw_block_id_ = consensus.source_block_id;
  trusted_bw_ = BandwidthFromBps(update.published_bps);
  trusted_bw_valid_ = !trusted_bw_.IsZero();
  trusted_bw_conf_ = consensus.confidence;
  trusted_bw_source_ = kTrustedBwSourceFBBRFrequencySearch;
  trusted_bw_cruise_id_ = fbbr_probe_signature_.cruise_id;
  trusted_bw_fresh_ = trusted_bw_valid_;
  trusted_bw_application_valid_ = trusted_bw_valid_;
  trusted_bw_ready_for_post_cruise_ = trusted_bw_valid_;
  trusted_bw_application_phase_ = "POST_CRUISE_READY";
  trusted_bw_invalid_reason_ = "none";
  selection_native_bw_ = BandwidthEstimate();
  dual_signal_spectral_gate_pass_ = false;
  drate_spectral_gate_pass_ = false;
  srtt_spectral_gate_pass_ = false;
}

void FBBRSender::FBBRFrequencySearchFinalizeCruise(QuicTime now) {
  FBBRFrequencySearchFinalizeReadyBlocks(now, true);
  FbbrCruiseConsensusResult consensus;
  FbbrHistoryUpdateResult update;
  if (fbbr_frequency_search_config_.search_controller_enabled) {
    consensus.valid = fbbr_trusted_published_in_cruise_ && trusted_bw_valid_;
    consensus.raw_candidate_bps =
        consensus.valid ? trusted_bw_.ToBitsPerSecond() : 0.0;
    consensus.confidence = consensus.valid ? trusted_bw_conf_ : 0.0;
    consensus.block_count = static_cast<uint32_t>(
        fbbr_search_state_.trusted_candidates_bps.size());
    consensus.invalid_reason = consensus.valid ? "none" :
        "adaptive_search_not_locked";
    update.valid = consensus.valid;
    update.raw_candidate_bps = consensus.raw_candidate_bps;
    update.published_bps = consensus.raw_candidate_bps;
    update.confidence = consensus.confidence;
    update.action = consensus.valid ? "F_BBR_LOCK_PUBLISH" : "REJECT";
    update.invalid_reason = consensus.invalid_reason;
    if (!consensus.valid) {
      ++fbbr_search_state_.unresolved_cruises;
      fbbr_search_state_.search_active = true;
      fbbr_search_state_.state = FBBRSearchState::kPersistentUnresolved;
      fbbr_search_state_.last_failure_reason = consensus.invalid_reason;
      if (trusted_bw_valid_ &&
          trusted_bw_age_cruises_ <= fbbr_frequency_search_config_.trusted_ttl_cruises) {
        trusted_bw_application_valid_ = true;
        trusted_bw_ready_for_post_cruise_ = true;
        trusted_bw_fresh_ = true;
        trusted_bw_cruise_id_ = fbbr_probe_signature_.cruise_id;
        trusted_bw_application_phase_ = "POST_CRUISE_TTL_REUSE";
        trusted_bw_invalid_reason_ = "search_unresolved_reuse_trusted";
      } else {
        ClearTrustedBw("adaptive_search_not_locked");
      }
    } else {
      fbbr_search_state_.unresolved_cruises = 0;
    }
    FBBRFrequencySearchEmitCruiseSummary(now, consensus, update);
    return;
  }
  if (!fbbr_probe_active_) {
    consensus.invalid_reason = fbbr_probe_disabled_reason_;
    update.action = "REJECT";
    update.invalid_reason = fbbr_probe_disabled_reason_;
    ClearTrustedBw(fbbr_probe_disabled_reason_.c_str());
  } else {
    consensus = FBBRFrequencySearch::BuildCruiseConsensus(
        fbbr_frequency_search_config_, fbbr_block_results_);
    update = FBBRFrequencySearch::StabilizeHistory(
        fbbr_frequency_search_config_, consensus, fbbr_history_valid_,
        fbbr_history_bandwidth_bps_,
        FBBRFrequencySearchHasNativeChangeEvidence(consensus.raw_candidate_bps));
    if (update.valid) {
      fbbr_history_valid_ = true;
      fbbr_history_bandwidth_bps_ = update.published_bps;
      fbbr_history_update_action_ = update.action;
      fbbr_trusted_bw_raw_candidate_bps_ = update.raw_candidate_bps;
      fbbr_trusted_bw_smoothed_bps_ = update.published_bps;
      fbbr_trusted_bw_robust_cv_ = consensus.robust_cv;
      fbbr_trusted_bw_relative_ci_width_ = consensus.relative_ci_width;
      fbbr_trusted_bw_cycle_count_ = consensus.cycle_count;
      fbbr_trusted_bw_block_id_ = consensus.source_block_id;
      trusted_bw_ = BandwidthFromBps(update.published_bps);
      trusted_bw_valid_ = !trusted_bw_.IsZero();
      trusted_bw_conf_ = consensus.confidence;
      trusted_bw_source_ = kTrustedBwSourceFBBRFrequencySearch;
      trusted_bw_cruise_id_ = fbbr_probe_signature_.cruise_id;
      trusted_bw_age_cruises_ = 0;
      trusted_bw_fresh_ = trusted_bw_valid_;
      trusted_bw_application_valid_ = trusted_bw_valid_;
      trusted_bw_ready_for_post_cruise_ = trusted_bw_valid_;
      trusted_bw_application_phase_ = "POST_CRUISE_READY";
      trusted_bw_invalid_reason_ = "none";
      selection_native_bw_ = BandwidthEstimate();
      dual_signal_spectral_gate_pass_ = false;
    } else {
      fbbr_history_update_action_ = update.action;
      ClearTrustedBw(update.invalid_reason.c_str());
    }
  }
  FBBRFrequencySearchEmitCruiseSummary(now, consensus, update);
}

void FBBRSender::FBBRFrequencySearchEmitBinTrace(
    const FbbrPhaseBinSample& sample,
    uint64_t block_id) const {
  if (!cruise_load_trace_cb_) return;
  std::ostringstream row;
  row << "FBBR_BIN,FBBR_BIN,F-BBR," << trace_flow_id_ << ","
      << fbbr_probe_signature_.cruise_id << "," << block_id << ","
      << sample.cycle_index << "," << sample.phase_bin_index << ","
      << sample.time_start_s << "," << sample.time_end_s << ","
      << fbbr_probe_signature_.frequency_hz << ","
      << fbbr_probe_signature_.period_rtts << ","
      << fbbr_probe_signature_.code_id << "," << sample.code_sign << ","
      << sample.phase_rad << "," << sample.coded_excitation << ","
      << sample.native_pacing_bps << "," << sample.commanded_pacing_bps << ","
      << sample.actual_send_bps << "," << sample.sent_bytes << ","
      << sample.acked_bytes << "," << sample.delivery_rate_bps << ","
      << sample.latest_rtt_s * 1e6 << "," << sample.qdelay_s * 1e6 << ","
      << sample.loss_ratio << "," << sample.ecn_ratio << ","
      << sample.app_limited_fraction << ","
      << sample.cwnd_limited_fraction << "," << sample.coverage << ","
      << (sample.rtt_valid ? "true" : "false") << ","
      << (sample.valid ? "true" : "false") << ","
      << sample.recovery_fraction << "," << fbbr_probe_signature_.waveform
      << "," << sample.queue_servo_factor << ","
      << (sample.queue_servo_transition ? "true" : "false");
  cruise_load_trace_cb_(sample.time_start_s, sample.time_end_s, 0, 0, 0, 0,
                        "FBBR_BIN", !sample.valid, row.str());
}

void FBBRSender::FBBREmitTriggerCycleTrace(
    const FBBRTriggerCycleResult& r) const {
  if (!cruise_load_trace_cb_) return;
  std::ostringstream row;
  row << "FBBR_TRIGGER_CYCLE,FBBR_TRIGGER_CYCLE,F-BBR," << trace_flow_id_
      << "," << r.cruise_id << "," << r.cycle_id << ","
      << r.cycle_start_s << "," << r.cycle_end_s << ","
      << EventWindowStateName(r.window_state) << ","
      << (r.is_pulser ? "PULSER" : "WATCHER") << ","
      << r.carrier_period_s << "," << r.actual_input_amplitude_ratio << ","
      << r.actual_input_energy << "," << r.delivery_response_amplitude_bps
      << "," << r.delivery_response_bytes << "," << r.period_estimate_s
      << "," << r.period_error_ratio << "," << r.spectral_prominence << ","
      << r.normalized_match << "," << r.selected_delay_s << ","
      << r.phase_coverage << "," << (r.trigger_pass ? "true" : "false")
      << "," << (r.continue_pass ? "true" : "false") << ","
      << r.trigger_reason << "," << r.pause_reason << ","
      << r.app_limited_fraction << "," << r.recovery_fraction << ","
      << r.baseline_drift << ","
      << (r.actual_input_measurable ? "true" : "false") << ","
      << (r.period_match ? "true" : "false") << ","
      << (r.weak_periodic_response ? "true" : "false") << ","
      << r.commanded_amplitude_ratio << "," << r.actual_input_snr << ","
      << r.delivery_period_estimate_s << ","
      << r.delivery_period_error_ratio << ","
      << r.delivery_spectral_prominence << ","
      << r.delivery_normalized_match << ","
      << (r.delivery_trigger_pass ? "true" : "false") << ","
      << (r.delivery_continue_pass ? "true" : "false") << ","
      << r.delivery_reason << "," << r.queue_derivative_amplitude << ","
      << r.queue_period_estimate_s << "," << r.queue_period_error_ratio << ","
      << r.queue_spectral_prominence << "," << r.queue_normalized_match << ","
      << r.queue_noise_floor << ","
      << (r.queue_trigger_pass ? "true" : "false") << ","
      << (r.queue_continue_pass ? "true" : "false") << ","
      << r.queue_reason << "," << r.combined_trigger_source << ","
      << r.combined_confidence << "," << r.detected_cycle_start_s << ","
      << r.alignment_error_cycles << ","
      << (r.hard_safety ? "true" : "false");
  cruise_load_trace_cb_(r.cycle_start_s, r.cycle_end_s, 0, 0, 0, 0,
                        "FBBR_TRIGGER_CYCLE", !r.trigger_pass, row.str());
}

void FBBRSender::FBBRFrequencySearchEmitBlockTrace(
    const FbbrOperatingPointBlockResult& r) const {
  if (!cruise_load_trace_cb_) return;
  std::ostringstream row;
  row << "FBBR_EVENT_WINDOW,FBBR_EVENT_WINDOW,F-BBR," << trace_flow_id_ << ","
      << r.cruise_id << "," << r.block_id << "," << r.start_time_s << ","
      << r.end_time_s << "," << r.frequency_hz << "," << r.period_rtts << ","
      << r.code_id << "," << r.initial_phase_rad << ","
      << r.target_amplitude_ratio << "," << r.realized_amplitude_ratio << ","
      << r.rtprop_frozen_s * 1e6 << "," << r.selected_delay_s * 1e6 << ","
      << r.delay_ratio << "," << (r.delay_at_search_boundary ? "true" : "false")
      << "," << (r.cross_block_delay_stable ? "true" : "false") << ","
      << r.valid_cycles << "," << r.phase_bin_coverage << ","
      << r.non_app_limited_fraction << "," << r.input_cycle_coherence << ","
      << r.native_baseline_drift << "," << r.delivery_baseline_drift << ","
      << r.loss_ratio << "," << r.ecn_ratio << ","
      << r.confidence_coverage << "," << r.confidence_input << ","
      << r.confidence_stationarity << "," << r.confidence_cycle << ","
      << r.confidence_response << "," << r.confidence_delay << ","
      << r.confidence_regression << "," << r.measurement_confidence << ","
      << r.input_fit.snr << "," << r.delivery_fit.snr << ","
      << r.queue_fit.snr << "," << r.utility_fit.snr << ","
      << r.input_fit.r_squared << "," << r.delivery_fit.r_squared << ","
      << r.queue_fit.r_squared << "," << r.utility_fit.r_squared << ","
      << r.input_fit.condition_number << "," << r.delivery_fit.condition_number
      << "," << r.queue_fit.condition_number << ","
      << r.utility_fit.condition_number << "," << r.response.delivery_gain << ","
      << r.response.queue_storage_gain << ","
      << r.response.delivery_phase_rad << "," << r.response.queue_phase_rad << ","
      << r.response.utility_phase_rad << ","
      << r.response.delivery_harmonic_ratio << ","
      << r.response.queue_harmonic_ratio << "," << r.positive_delivery_gain
      << "," << r.positive_queue_build_s * 1e6 << ","
      << r.positive_queue_build_gain << "," << r.q_zero_s * 1e6 << ","
      << r.q_probe_max_s * 1e6 << "," << r.q_floor_s * 1e6 << ","
      << r.q95_s * 1e6 << "," << r.q_amplitude_s * 1e6 << ","
      << r.drain_ratio << "," << r.queue_trend_per_cycle << ","
      << r.gradient_lockin << "," << r.gradient_finite_difference << ","
      << r.gradient_fused << "," << r.curvature_finite_difference << ","
      << (r.gradient_agreement ? "true" : "false") << ","
      << r.full_score << "," << r.low_queue_score << ","
      << r.stationary_score << "," << r.safe_score << ","
      << r.optimality_score << "," << r.rate_adjustment_signal << ","
      << FbbrOperatingPointClassificationName(r.classification) << ","
      << (r.candidate.valid ? "true" : "false") << ","
      << r.candidate.bandwidth_bps << "," << r.candidate.robust_cv << ","
      << r.candidate.relative_ci_width << "," << r.candidate.cycle_count << ","
      << r.candidate.invalid_reason << "," << r.invalid_reason << ","
      << r.actual_input_amplitude_ratio << ","
      << r.cwnd_limited_fraction << "," << r.input_carrier_snr << ","
      << r.signature_leakage << ","
      << r.residual_to_own_carrier_ratio << ","
      << (r.collision_suspected ? "true" : "false") << ","
      << r.raw_optimality_score << "," << r.underload_evidence << ","
      << r.overload_evidence << "," << r.direction_score;
  if (!fbbr_control_decisions_.empty() &&
      fbbr_frequency_search_config_.search_controller_enabled) {
    const FBBRWindowControlDecision& d = fbbr_control_decisions_.back();
    const double midpoint = fbbr_search_state_.underload_bound_valid &&
                                    fbbr_search_state_.overload_bound_valid &&
                                    fbbr_search_state_.underload_bound_bps <
                                        fbbr_search_state_.overload_bound_bps
                                ? std::sqrt(
                                      fbbr_search_state_.underload_bound_bps *
                                      fbbr_search_state_.overload_bound_bps)
                                : 0.0;
    const double width = midpoint > 0.0
        ? (fbbr_search_state_.overload_bound_bps -
           fbbr_search_state_.underload_bound_bps) / midpoint
        : 0.0;
    row << "," << FBBRSearchStateName(d.state_before) << ","
        << FBBRSearchStateName(d.state_after) << ","
        << d.baseline_before_bps << "," << d.proposed_baseline_bps << ","
        << d.applied_next_baseline_bps << "," << d.log_step << ","
        << d.update_reason << "," << (d.hard_loss_abort ? "true" : "false")
        << "," << (fbbr_search_state_.underload_bound_valid ? "true" : "false")
        << "," << fbbr_search_state_.underload_bound_bps << ","
        << (fbbr_search_state_.overload_bound_valid ? "true" : "false")
        << "," << fbbr_search_state_.overload_bound_bps << "," << width << ","
        << fbbr_search_state_.same_direction_streak << ","
        << fbbr_search_state_.consecutive_dynamic << ","
        << fbbr_search_state_.consecutive_near_optimal << ","
        << (d.lock_candidate ? "true" : "false") << ","
        << (d.locked ? "true" : "false") << ","
        << (d.window_candidate_valid ? "true" : "false") << ","
        << d.window_candidate_bps << ","
        << (d.trusted_bw_published ? "true" : "false") << ","
        << d.trusted_bw_bps << "," << d.trusted_confidence << ","
        << fbbr_opi_rtprop_anchor_.ToMicroseconds() << ","
        << fbbr_opi_rtprop_confidence_ << "," << fbbr_opi_rtprop_source_
        << "," << r.gradient_se << "," << r.gradient_ci90_low << ","
        << r.gradient_ci90_high << "," << r.gradient_ci95_low << ","
        << r.gradient_ci95_high << ","
        << (r.gradient_equivalent ? "true" : "false") << ","
        << r.delivery_median_bps << ","
        << (fbbr_search_state_.search_active ? "true" : "false") << ","
        << fbbr_search_state_.unresolved_cruises << ","
        << fbbr_search_state_.unresolved_decisions << ","
        << fbbr_search_state_.search_attempts << ","
        << fbbr_search_state_.eligible_cruises << ","
        << fbbr_search_state_.last_failure_reason << ",ADAPTIVE_SEARCH,"
        << (trusted_bw_valid_ ? "true" : "false") << ","
        << (fbbr_search_state_.is_pulser ? "true" : "false") << ","
        << fbbr_search_state_.pulser_lease_remaining << ","
        << fbbr_probe_signature_.waveform << "," << r.recovery_fraction << ","
        << r.effective_cycles << ","
        << (fbbr_search_state_.carrier_detected ? "true" : "false") << ","
        << (d.collision_suspected ? "true" : "false") << ","
        << fbbr_search_state_.search_generation << ","
        << fbbr_search_state_.election_backoff_cycles << ","
        << fbbr_search_state_.pulser_lease_count << ","
        << fbbr_search_state_.collision_count << ","
        << (fbbr_search_state_.provisional_validation_pending ? "true" : "false")
        << "," << fbbr_search_state_.provisional_age_cruises << ","
        << fbbr_search_state_.carrier_sense_snr << ","
        << fbbr_search_state_.carrier_sense_amplitude;
  } else {
    row << ",DISABLED,DISABLED,0,0,0,0,NONE,false,false,0,false,0,0,0,0,0,"
           "false,false,false,0,false,0,0,0,0,NONE,0,0,0,0,0,false,0,false,"
           "0,0,0,0,none,NATIVE_BBR,false,false,0,none,0,0,false,false,0,0,0,0,false,0,0,0";
  }
  row << "," << r.event_window_id << "," << r.trigger_cycle_id << ","
      << r.capture_start_s << "," << r.capture_end_s << ","
      << r.window_length_cycles << "," << r.window_stride_cycles << ","
      << r.sequential_stop_reason << ","
      << EventWindowStateName(r.event_window_state) << ","
      << (r.trigger_cycle_excluded_from_score ? "true" : "false") << ","
      << r.overlap_fraction << ","
      << (r.independent_for_control ? "true" : "false") << ","
      << (r.independent_for_trusted ? "true" : "false") << ","
      << (r.lockable_score ? "true" : "false") << ","
      << r.saturation_score << "," << r.queue_band_score << ","
      << r.queue_stability_score << "," << r.target_score << ","
      << r.q_floor_s * 1e6 << "," << r.q_reserve_low_s * 1e6 << ","
      << r.q_reserve_high_s * 1e6 << "," << r.q_peak_cap_s * 1e6 << ","
      << r.q95_s * 1e6 << "," << r.queue_trend_per_cycle << ","
      << r.frequency_direction << "," << r.queue_band_error << ","
      << r.total_direction << "," << r.delivery_spectral_prominence << ","
      << r.delivery_normalized_match << "," << r.trigger_source << ","
      << r.response.delivery_gain << ","
      << r.response.queue_storage_gain << "," << r.delivery_partition << ","
      << r.queue_partition << "," << r.confidence_delivery_channel << ","
      << r.confidence_queue_channel << "," << r.queue_spectral_prominence << ","
      << r.queue_normalized_match << "," << r.queue_servo_factor_mean << ","
      << r.queue_servo_transition_cycles;
  cruise_load_trace_cb_(r.start_time_s, r.end_time_s, 0, r.optimality_score, 0,
                        r.measurement_confidence, "FBBR_EVENT_WINDOW",
                        r.classification ==
                            FbbrOperatingPointClassification::kInvalid,
                        row.str());
}

void FBBRSender::FBBRFrequencySearchEmitShadowWindowTrace(
    const FbbrOperatingPointBlockResult& r) const {
  if (!cruise_load_trace_cb_) return;
  std::ostringstream row;
  row << "FBBR_DIAGNOSTIC_WINDOW,FBBR_DIAGNOSTIC_WINDOW,F-BBR,"
      << trace_flow_id_ << "," << r.cruise_id << "," << r.block_id << ","
      << r.start_time_s << "," << r.end_time_s << ","
      << r.frequency_hz << "," << r.period_rtts << "," << r.code_id << ","
      << r.initial_phase_rad << "," << r.target_amplitude_ratio << ","
      << r.realized_amplitude_ratio << "," << r.rtprop_frozen_s * 1e6 << ","
      << r.selected_delay_s * 1e6 << "," << r.delay_ratio << ","
      << (r.delay_at_search_boundary ? "true" : "false") << ","
      << (r.cross_block_delay_stable ? "true" : "false") << ","
      << r.valid_cycles << "," << r.phase_bin_coverage << ","
      << r.non_app_limited_fraction << "," << r.input_cycle_coherence << ","
      << r.native_baseline_drift << "," << r.delivery_baseline_drift << ","
      << r.loss_ratio << "," << r.ecn_ratio << ","
      << r.confidence_coverage << "," << r.confidence_input << ","
      << r.confidence_stationarity << "," << r.confidence_cycle << ","
      << r.confidence_response << "," << r.confidence_delay << ","
      << r.confidence_regression << "," << r.measurement_confidence << ","
      << r.input_fit.snr << "," << r.delivery_fit.snr << ","
      << r.queue_fit.snr << "," << r.utility_fit.snr << ","
      << r.input_fit.r_squared << "," << r.delivery_fit.r_squared << ","
      << r.queue_fit.r_squared << "," << r.utility_fit.r_squared << ","
      << r.input_fit.condition_number << "," << r.delivery_fit.condition_number
      << "," << r.queue_fit.condition_number << ","
      << r.utility_fit.condition_number << "," << r.response.delivery_gain << ","
      << r.response.queue_storage_gain << ","
      << r.response.delivery_phase_rad << "," << r.response.queue_phase_rad << ","
      << r.response.utility_phase_rad << ","
      << r.response.delivery_harmonic_ratio << ","
      << r.response.queue_harmonic_ratio << "," << r.positive_delivery_gain
      << "," << r.positive_queue_build_s * 1e6 << ","
      << r.positive_queue_build_gain << "," << r.q_zero_s * 1e6 << ","
      << r.q_probe_max_s * 1e6 << "," << r.q_floor_s * 1e6 << ","
      << r.q95_s * 1e6 << "," << r.q_amplitude_s * 1e6 << ","
      << r.drain_ratio << "," << r.queue_trend_per_cycle << ","
      << r.gradient_lockin << "," << r.gradient_finite_difference << ","
      << r.gradient_fused << "," << r.curvature_finite_difference << ","
      << (r.gradient_agreement ? "true" : "false") << ","
      << r.full_score << "," << r.low_queue_score << ","
      << r.stationary_score << "," << r.safe_score << ","
      << r.optimality_score << "," << r.rate_adjustment_signal << ","
      << FbbrOperatingPointClassificationName(r.classification) << ",false,0,0,0,0,"
      << "validation_shadow_window," << r.invalid_reason << ",true,false";
  cruise_load_trace_cb_(r.start_time_s, r.end_time_s, 0, r.optimality_score, 0,
                        r.measurement_confidence, "FBBR_DIAGNOSTIC_WINDOW",
                        r.classification == FbbrOperatingPointClassification::kInvalid,
                        row.str());
}

void FBBRSender::FBBRFrequencySearchEmitCruiseSummary(
    QuicTime now,
    const FbbrCruiseConsensusResult& consensus,
    const FbbrHistoryUpdateResult& update) const {
  if (!cruise_load_trace_cb_) return;
  uint32_t underload = 0, near_optimal = 0, overload = 0, dynamic = 0, invalid = 0;
  uint32_t watcher_decisions = 0, drain_decisions = 0, seek_decisions = 0;
  uint32_t track_decisions = 0, unresolved_decisions = 0;
  uint32_t event_windows = 0, dense_windows = 0, slow_loop_updates = 0;
  uint32_t trusted_publications = 0;
  for (const auto& block : fbbr_block_results_) {
    if (block.window_length_cycles > 0.0) ++event_windows;
    if (block.event_window_state == EventWindowState::kContinuousTrack) {
      ++dense_windows;
    }
    switch (block.classification) {
      case FbbrOperatingPointClassification::kUnderload: ++underload; break;
      case FbbrOperatingPointClassification::kNearOptimal: ++near_optimal; break;
      case FbbrOperatingPointClassification::kQueuedOverload: ++overload; break;
      case FbbrOperatingPointClassification::kBufferSaturated: ++overload; break;
      case FbbrOperatingPointClassification::kDynamic: ++dynamic; break;
      case FbbrOperatingPointClassification::kInvalid: ++invalid; break;
    }
  }
  for (const auto& decision : fbbr_control_decisions_) {
    if (std::abs(decision.applied_next_baseline_bps -
                 decision.baseline_before_bps) >
        std::max(1.0, 1e-6 * decision.baseline_before_bps)) {
      ++slow_loop_updates;
    }
    if (decision.trusted_bw_published) ++trusted_publications;
    switch (decision.state_after) {
      case FBBRSearchState::kWatcher: ++watcher_decisions; break;
      case FBBRSearchState::kDrain:
      case FBBRSearchState::kEmergencyDrain: ++drain_decisions; break;
      case FBBRSearchState::kSeek: ++seek_decisions; break;
      case FBBRSearchState::kTrack:
      case FBBRSearchState::kLockCandidate:
      case FBBRSearchState::kLocked: ++track_decisions; break;
      case FBBRSearchState::kPersistentUnresolved:
      case FBBRSearchState::kDynamicReacquire: ++unresolved_decisions; break;
      default: break;
    }
  }
  const double start_s = static_cast<double>(
      (cruise_start_time_ - QuicTime::Zero()).ToMicroseconds()) / 1e6;
  const double end_s = static_cast<double>(
      (now - QuicTime::Zero()).ToMicroseconds()) / 1e6;
  std::ostringstream row;
  row << "FBBR_CRUISE,FBBR_CRUISE,F-BBR," << trace_flow_id_ << ","
      << fbbr_probe_signature_.cruise_id << "," << start_s << "," << end_s << ","
      << (fbbr_probe_active_ ? "true" : "false") << ","
      << fbbr_probe_disabled_reason_ << "," << fbbr_probe_signature_.frequency_hz
      << "," << fbbr_probe_signature_.period_rtts << ","
      << fbbr_probe_signature_.code_id << ","
      << fbbr_probe_signature_.amplitude_ratio << ","
      << fbbr_block_results_.size() << "," << underload << "," << near_optimal
      << "," << overload << "," << dynamic << "," << invalid << ","
      << (consensus.valid ? "true" : "false") << ","
      << consensus.raw_candidate_bps << "," << consensus.confidence << ","
      << consensus.robust_cv << "," << consensus.relative_ci_width << ","
      << consensus.block_count << "," << consensus.cycle_count << ","
      << consensus.source_block_id << "," << consensus.invalid_reason << ","
      << (update.valid ? "true" : "false") << "," << update.published_bps << ","
      << update.action << "," << update.invalid_reason << ","
      << (trusted_bw_valid_ ? trusted_bw_.ToBitsPerSecond() : 0) << ","
      << trusted_bw_conf_ << "," << trusted_bw_source_ << ","
      << trusted_bw_invalid_reason_ << ","
      << static_cast<double>(fair_share_bandwidth_bps_) << ","
      << (fbbr_search_state_.search_active ? "true" : "false") << ","
      << FBBRSearchStateName(fbbr_search_state_.state) << ","
      << fbbr_search_state_.eligible_cruises << ","
      << fbbr_search_state_.search_attempts << ","
      << fbbr_search_state_.unresolved_cruises << ","
      << fbbr_search_state_.unresolved_decisions << ","
      << fbbr_search_state_.valid_direction_decisions << ","
      << fbbr_search_state_.last_failure_reason << ","
      << (fbbr_search_state_.is_pulser ? "true" : "false") << ","
      << fbbr_search_state_.pulser_lease_remaining << ","
      << fbbr_probe_signature_.waveform << ","
      << (fbbr_search_state_.carrier_detected ? "true" : "false") << ","
      << fbbr_search_state_.search_generation << ","
      << fbbr_search_state_.pulser_lease_count << ","
      << fbbr_search_state_.collision_count << ","
      << watcher_decisions << "," << drain_decisions << ","
      << seek_decisions << "," << track_decisions << ","
      << unresolved_decisions << "," << trusted_bw_age_cruises_ << ","
      << fbbr_search_state_.carrier_sense_snr << ","
      << fbbr_search_state_.carrier_sense_amplitude << ","
      << fbbr_dual_trigger_attempts_ << "," << fbbr_delivery_triggers_ << ","
      << fbbr_queue_triggers_ << "," << fbbr_both_triggers_ << ","
      << fbbr_hard_safety_events_ << "," << event_windows << ","
      << dense_windows << "," << fbbr_queue_servo_updates_ << ","
      << fbbr_queue_servo_drain_rtts_ << ","
      << fbbr_queue_servo_recovery_rtts_ << ","
      << fbbr_queue_servo_baseline_commits_ << "," << slow_loop_updates << ","
      << trusted_publications << ","
      << (fbbr_search_state_.unresolved_cruises > 0 ? "true" : "false");
  cruise_load_trace_cb_(start_s, end_s, 0, 0, 0, consensus.confidence,
                        "FBBR_CRUISE", !consensus.valid, row.str());
}

void FBBRSender::OnProbeBwPhaseEntered(Bbr2ProbeBwMode::CyclePhase phase,
                                           QuicTime now) {
  if (phase == Bbr2ProbeBwMode::CyclePhase::PROBE_CRUISE) {
    EnterCruise(now);
    return;
  }
  if (in_cruise_) {
    LeaveCruise(now);
  }
}

void FBBRSender::EnterCruise(QuicTime now) {
  in_cruise_ = true;
  ++cruise_id_;
  if (trusted_bw_valid_) ++trusted_bw_age_cruises_;
  cruise_start_time_ = now;
  trusted_bw_cleared_on_cruise_start_ = false;
  if (!trusted_bw_clear_on_cruise_start_) {
    QUIC_DVLOG(1) << "FBBR: trusted_bw.clear_on_cruise_start=false "
                     "is overridden to preserve fresh-only application";
  }
  ClearTrustedBwApplication("cruise_start");
  cruise_modulation_freq_hz_ = configured_modulation_freq_hz_;
  freq_tool_on_ = ShouldOscillate();
  cruise_freq_tool_active_ = freq_tool_on_;
  min_rtt_warning_logged_ = false;
  current_cruise_windows_.clear();
  ResetCruiseWindowState();
  if (fbbr_frequency_search_config_.frequency_search_enabled) {
    FBBRFrequencySearchInitializeCruise(now);
  }
  QUIC_DVLOG(2) << "FBBR: Entering PROBE_CRUISE @ " << now
                << ", cruise_id=" << cruise_id_
                << ", fixed_freq=" << cruise_modulation_freq_hz_
                << "Hz, amplitude_bps=" << GetCurrentAmplitudeBps();
}

void FBBRSender::LeaveCruise(QuicTime now) {
  QUIC_DVLOG(2) << "FBBR: Leaving PROBE_CRUISE @ " << now;
  FinalizeCruise(now);
  in_cruise_ = false;
  freq_tool_on_ = false;
  cruise_freq_tool_active_ = false;
  cruise_start_time_ = QuicTime::Zero();
  cruise_modulation_freq_hz_ = configured_modulation_freq_hz_;
  ResetCruiseWindowState();
  current_cruise_windows_.clear();
}

void FBBRSender::ResetCruiseWindowState() {
  TimeDelta min_rtt = model_.MinRtt();
  if (min_rtt.IsZero() && rtt_stats_ != nullptr) {
    min_rtt = rtt_stats_->MinOrInitialRtt();
  }
  if (!min_rtt.IsZero() && cruise_start_time_ != QuicTime::Zero()) {
    next_cruise_window_start_ = cruise_start_time_ + min_rtt;
  } else {
    next_cruise_window_start_ = QuicTime::Zero();
  }
}

void FBBRSender::OnCongestionEventStarted(
    const Bbr2CongestionEvent& congestion_event) {
  UpdateRoundDeliveryRateSample(congestion_event);
  if (fbbr_frequency_search_config_.frequency_search_enabled && in_cruise_) {
    FBBRFrequencySearchAccumulateAck(congestion_event);
  }
  if (congestion_event.end_of_round_trip) {
    FinalizeCompletedRound(congestion_event);
  }
}

bool FBBRSender::ShouldDelayProbeBwCruiseExit(QuicTime now) const {
  if (!fbbr_frequency_search_config_.frequency_search_enabled || !fbbr_probe_active_ ||
      cruise_start_time_ == QuicTime::Zero() || fbbr_period_.IsZero()) {
    return false;
  }
  if (fbbr_frequency_search_config_.search_controller_enabled) {
    if (fbbr_search_state_.state == FBBRSearchState::kLocked ||
        fbbr_search_state_.state ==
            FBBRSearchState::kEmergencyDrain ||
        fbbr_search_state_.control_window_index >=
            fbbr_frequency_search_config_.max_control_windows ||
        InRecovery() ||
        fbbr_last_sample_app_limited_) {
      return false;
    }
    const int64_t rtt_limit_us = fbbr_rtprop_frozen_.ToMicroseconds() *
        static_cast<int64_t>(fbbr_frequency_search_config_.max_cruise_extension_rtts);
    const int64_t time_limit_us = static_cast<int64_t>(std::llround(
        fbbr_frequency_search_config_.max_cruise_extension_s * 1e6));
    const int64_t extension_limit_us = std::min(rtt_limit_us, time_limit_us);
    return (now - cruise_start_time_).ToMicroseconds() < extension_limit_us;
  }
  const uint64_t required_cycles =
      static_cast<uint64_t>(fbbr_frequency_search_config_.warmup_cycles) +
      static_cast<uint64_t>(fbbr_frequency_search_config_.analysis_cycles);
  const int64_t required_us =
      static_cast<int64_t>(required_cycles) * fbbr_period_.ToMicroseconds() +
      static_cast<int64_t>(std::ceil(
          fbbr_frequency_search_config_.delay_search_ratio_max *
          fbbr_rtprop_frozen_.ToMicroseconds()));
  return now - cruise_start_time_ < TimeDelta::FromMicroseconds(required_us);
}

void FBBRSender::UpdateRoundDeliveryRateSample(
    const Bbr2CongestionEvent& congestion_event) {
  // InRecovery() is currently a Bbr2Sender stub in this tree, but keep this
  // guard here so the D_round definition is ready once recovery is wired.
  const bool in_recovery = InRecovery();
  if (!congestion_event.sample_valid ||
      congestion_event.sample_max_bandwidth.IsZero() ||
      congestion_event.sample_is_app_limited || in_recovery) {
    return;
  }

  if (!d_round_valid_ || congestion_event.sample_max_bandwidth > d_round_) {
    d_round_ = congestion_event.sample_max_bandwidth;
    d_round_valid_ = true;
  }
}

void FBBRSender::FinalizeCompletedRound(
    const Bbr2CongestionEvent& congestion_event) {
  const QuicBandwidth completed_d_round = d_round_;
  const bool completed_d_valid = d_round_valid_;
  const QuicBandwidth previous_d_round = d_prev_;
  const bool previous_d_valid = d_prev_valid_;
  const double previous_v_round = prev_v_round_;

  double v_round = 0.0;
  bool v_round_valid = false;
  if (completed_d_valid && previous_d_valid && !previous_d_round.IsZero()) {
    const double current_bps =
        static_cast<double>(completed_d_round.ToBitsPerSecond());
    const double previous_bps =
        static_cast<double>(previous_d_round.ToBitsPerSecond());
    v_round = std::abs(current_bps - previous_bps) / previous_bps;
    v_round_valid = true;
  }

  bool just_exited = false;
  if (bbr_stable_) {
    just_exited =
        CheckExitStable(completed_d_round, completed_d_valid, v_round);
  }

  if (!bbr_stable_ && !just_exited) {
    UpdateReconvergenceEvidence(completed_d_round, completed_d_valid);
  }

  UpdateFreqWeightAndToolState();
  EmitConvergenceGateTrace(congestion_event,
                           completed_d_round,
                           completed_d_valid,
                           previous_d_round,
                           previous_d_valid,
                           v_round,
                           v_round_valid,
                           previous_v_round,
                           just_exited);

  if (completed_d_valid) {
    d_prev_ = completed_d_round;
    d_prev_valid_ = true;
  }
  d_round_ = QuicBandwidth::Zero();
  d_round_valid_ = false;
}

bool FBBRSender::CheckExitStable(QuicBandwidth completed_d_round,
                                     bool completed_d_valid,
                                     double v_round) {
  if (!completed_d_valid || !d_prev_valid_ || d_prev_.IsZero()) {
    return false;
  }

  const bool exit_stable =
      v_round > stable_single_round_exit_threshold_ ||
      (v_round > stable_consecutive_exit_threshold_ &&
       prev_v_round_ > stable_consecutive_exit_threshold_);
  prev_v_round_ = v_round;
  if (!exit_stable) {
    return false;
  }

  bbr_stable_ = false;
  stable_cnt_ = 0;
  full_drate_ref_ = completed_d_round;
  full_drate_ref_valid_ = true;
  freq_tool_needed_ = true;
  w_freq_ = 1.0;
  ++unstable_episode_id_;
  unstable_episode_active_ = true;
  return true;
}

void FBBRSender::UpdateReconvergenceEvidence(
    QuicBandwidth completed_d_round,
    bool completed_d_valid) {
  if (!completed_d_valid) {
    return;
  }

  if (!full_drate_ref_valid_) {
    full_drate_ref_ = completed_d_round;
    full_drate_ref_valid_ = true;
    stable_cnt_ = 0;
    return;
  }

  if (completed_d_round >= full_drate_ref_ *
                                static_cast<float>(
                                    stable_full_pipe_growth_threshold_)) {
    full_drate_ref_ = completed_d_round;
    stable_cnt_ = 0;
    return;
  }

  if (stable_cnt_ < stable_rounds_) {
    ++stable_cnt_;
  }

  if (stable_cnt_ >= stable_rounds_) {
    stable_cnt_ = stable_rounds_;
    bbr_stable_ = true;
    freq_tool_needed_ = false;
    w_freq_ = 0.0;
    unstable_episode_active_ = false;
    ClearTrustedBw("stable_closure");
    freq_tool_on_ = false;
    prev_v_round_ = 0.0;
  }
}

void FBBRSender::UpdateFreqWeightAndToolState() {
  if (bbr_stable_) {
    w_freq_ = 0.0;
    freq_tool_needed_ = false;
    unstable_episode_active_ = false;
    if (enable_convergence_gate_control_) {
      ClearTrustedBw("stable_closure");
      freq_tool_on_ = false;
    } else {
      freq_tool_on_ = ShouldOscillate();
      if (in_cruise_ && freq_tool_on_) {
        cruise_freq_tool_active_ = true;
      }
    }
    return;
  }

  if (stable_cnt_ >= stable_rounds_) {
    stable_cnt_ = stable_rounds_;
    bbr_stable_ = true;
    w_freq_ = 0.0;
    freq_tool_needed_ = false;
    unstable_episode_active_ = false;
    ClearTrustedBw("stable_closure");
    freq_tool_on_ = false;
    return;
  }

  w_freq_ =
      Clamp01(1.0 - static_cast<double>(stable_cnt_) /
                        static_cast<double>(stable_rounds_));
  freq_tool_on_ = freq_tool_needed_ && BaseShouldOscillate();
  if (in_cruise_ && freq_tool_on_) {
    cruise_freq_tool_active_ = true;
  }
}

void FBBRSender::ClearTrustedBw(const char* reason) {
  trusted_bw_ = QuicBandwidth::Zero();
  trusted_bw_valid_ = false;
  trusted_bw_conf_ = 0.0;
  trusted_bw_source_ = kTrustedBwSourceNone;
  trusted_bw_cruise_id_ = 0;
  trusted_bw_age_cruises_ = 0;
  trusted_bw_invalid_reason_ = reason == nullptr ? "unknown" : reason;
  ClearTrustedBwApplication(reason);
}

void FBBRSender::ClearTrustedBwApplication(const char* reason) const {
  trusted_bw_fresh_ = false;
  trusted_bw_application_valid_ = false;
  trusted_bw_ready_for_post_cruise_ = false;
  trusted_bw_application_phase_ = "NONE";
  if (reason != nullptr && std::string(reason) == "cruise_start") {
    trusted_bw_cleared_on_cruise_start_ = true;
  }
}

bool FBBRSender::IsReliableSpectralWindow(
    const CruiseWindowResult& result) const {
  return result.dual_signal_spectral_gate_pass &&
         result.is_full_load_candidate && !result.low_confidence &&
         result.full_load_quality_v2 >=
             min_full_load_quality_for_reliable_window_ &&
         std::isfinite(result.drate_mean_kbps) &&
         result.drate_mean_kbps > 0.0;
}

double FBBRSender::ComputeRateTrendRatio(QuicTime start,
                                             QuicTime end) const {
  const auto samples = SelectRateSamples(delivery_rate_history_, start, end);
  if (samples.size() < 4) {
    return std::numeric_limits<double>::infinity();
  }
  std::vector<double> first;
  std::vector<double> second;
  std::vector<double> all;
  first.reserve(samples.size() / 2);
  second.reserve(samples.size() / 2 + 1);
  all.reserve(samples.size());
  const QuicTime mid =
      start + TimeDelta::FromMicroseconds((end - start).ToMicroseconds() / 2);
  for (const auto& sample : samples) {
    const double kbps = static_cast<double>(sample.rate.ToKBitsPerSecond());
    if (!std::isfinite(kbps) || kbps <= 0.0) {
      return std::numeric_limits<double>::infinity();
    }
    all.push_back(kbps);
    if (sample.time < mid) {
      first.push_back(kbps);
    } else {
      second.push_back(kbps);
    }
  }
  if (first.empty() || second.empty()) {
    return std::numeric_limits<double>::infinity();
  }
  const double median_all = Median(all);
  if (median_all <= 1e-9) {
    return std::numeric_limits<double>::infinity();
  }
  return std::abs(Median(first) - Median(second)) / median_all;
}

void FBBRSender::PublishTrustedBwSelection(
    const TrustedBwSelectionResult& selection) {
  selection_native_bw_ = selection.native_bw;
  drate_spectral_integrity_score_ =
      selection.drate_spectral_integrity_score;
  srtt_spectral_integrity_score_ =
      selection.srtt_spectral_integrity_score;
  joint_spectral_integrity_score_ =
      selection.joint_spectral_integrity_score;
  drate_spectral_gate_pass_ = selection.drate_spectral_gate_pass;
  srtt_spectral_gate_pass_ = selection.srtt_spectral_gate_pass;
  dual_signal_spectral_gate_pass_ =
      selection.dual_signal_spectral_gate_pass;
  limiting_spectral_signal_ = selection.limiting_spectral_signal;
  merged_rescue_attempted_ = selection.merged_rescue_attempted;
  merged_rescue_success_ = selection.merged_rescue_success;
  trusted_bw_selection_compute_us_ =
      selection.trusted_bw_selection_compute_us;
  normal_window_count_ = selection.normal_window_count;
  merged_window_count_ = selection.merged_window_count;
  spectral_invalid_count_ = selection.spectral_invalid_count;

  const bool valid_selection =
      selection.trusted_bw_valid &&
      selection.dual_signal_spectral_gate_pass &&
      !selection.trusted_bw.IsZero() &&
      std::isfinite(static_cast<double>(
          selection.trusted_bw.ToBitsPerSecond()));
  if (!valid_selection) {
    ClearTrustedBw("no_dual_signal_trusted_bw");
    return;
  }

  trusted_bw_ = selection.trusted_bw;
  trusted_bw_valid_ = true;
  trusted_bw_conf_ = selection.trusted_bw_conf;
  trusted_bw_source_ = selection.trusted_bw_source;
  trusted_bw_cruise_id_ =
      static_cast<uint64_t>(std::max<int64_t>(0, cruise_id_));
  trusted_bw_fresh_ = true;
  trusted_bw_application_valid_ = true;
  trusted_bw_ready_for_post_cruise_ = true;
  trusted_bw_application_phase_ = "POST_CRUISE_READY";
  trusted_bw_invalid_reason_ = "none";
}

const char* FBBRSender::PhaseApplicationName(
    Bbr2ProbeBwMode::CyclePhase phase) {
  switch (phase) {
    case Bbr2ProbeBwMode::CyclePhase::PROBE_CRUISE:
      return "CRUISE";
    case Bbr2ProbeBwMode::CyclePhase::PROBE_REFILL:
      return "REFILL";
    case Bbr2ProbeBwMode::CyclePhase::PROBE_UP:
      return "UP";
    case Bbr2ProbeBwMode::CyclePhase::PROBE_DOWN:
    case Bbr2ProbeBwMode::CyclePhase::PROBE_DOWN_SLIGHTLY:
      return "DOWN";
    default:
      return "NONE";
  }
}

const char* FBBRSender::PacingBaseSourceName(
    FBBRPacingBaseSource source) {
  if (source == FBBRPacingBaseSource::kTrustedBw) return "TRUSTED_BW";
  if (source == FBBRPacingBaseSource::kAdaptiveSearch) {
    return "ADAPTIVE_SEARCH";
  }
  return "NATIVE_BBR";
}

bool FBBRSender::IsCruisePhase(
    Bbr2ProbeBwMode::CyclePhase phase) const {
  return mode_ == Bbr2Mode::PROBE_BW &&
         phase == Bbr2ProbeBwMode::CyclePhase::PROBE_CRUISE;
}

bool FBBRSender::IsTrustedBwApplicationPhase(
    Bbr2ProbeBwMode::CyclePhase phase) const {
  if (mode_ != Bbr2Mode::PROBE_BW) {
    return false;
  }
  return phase == Bbr2ProbeBwMode::CyclePhase::PROBE_REFILL ||
         phase == Bbr2ProbeBwMode::CyclePhase::PROBE_UP ||
         phase == Bbr2ProbeBwMode::CyclePhase::PROBE_DOWN ||
         phase == Bbr2ProbeBwMode::CyclePhase::PROBE_DOWN_SLIGHTLY;
}

void FBBRSender::EmitConvergenceGateTrace(
    const Bbr2CongestionEvent& congestion_event,
    QuicBandwidth completed_d_round,
    bool completed_d_valid,
    QuicBandwidth previous_d_round,
    bool previous_d_valid,
    double v_round,
    bool v_round_valid,
    double previous_v_round,
    bool just_exited) const {
  if (!enable_convergence_gate_trace_) {
    return;
  }

  QUIC_DVLOG(1)
      << "FBBR convergence_gate"
      << " round_id=" << model_.RoundTripCount()
      << " d_round_bps="
      << (completed_d_valid ? completed_d_round.ToBitsPerSecond() : 0)
      << " d_prev_bps="
      << (previous_d_valid ? previous_d_round.ToBitsPerSecond() : 0)
      << " v_round=" << (v_round_valid ? v_round : -1.0)
      << " stable_cnt=" << stable_cnt_
      << " bbr_stable=" << bbr_stable_
      << " freq_tool_on=" << freq_tool_on_
      << " trusted_bw_bps="
      << (trusted_bw_valid_ ? trusted_bw_.ToBitsPerSecond() : 0)
      << " trusted_bw_valid=" << trusted_bw_valid_
      << " trusted_bw_fresh=" << trusted_bw_fresh_
      << " trusted_bw_application_valid="
      << trusted_bw_application_valid_
      << " dual_signal_spectral_gate_pass="
      << dual_signal_spectral_gate_pass_;

  if (gate_trace_mode_ == FBBRGateTraceMode::kOff) {
    return;
  }
  const QuicBandwidth current_native_bw = BandwidthEstimate();
  const QuicBandwidth native_pacing = Bbr2Sender::PacingRate(0);
  EmitFreqGateCsvRow("round",
                     congestion_event.event_time,
                     completed_d_round,
                     completed_d_valid,
                     previous_d_round,
                     previous_d_valid,
                     v_round_valid ? v_round : -1.0,
                     previous_v_round,
                     just_exited,
                     current_native_bw,
                     current_native_bw,
                     FBBRPacingBaseSource::kNativeBbr,
                     native_pacing,
                     native_pacing,
                     static_cast<double>(PacingGain()),
                     0,
                     0,
                     0.0,
                     congestion_event.sample_max_bandwidth,
                     congestion_event.sample_is_app_limited,
                     congestion_event.sample_valid);
}

void FBBRSender::EmitPacingTrace(
    QuicBandwidth b_native,
    QuicBandwidth pacing_base_bw,
    FBBRPacingBaseSource pacing_base_source,
    QuicBandwidth native_pacing,
    QuicBandwidth final_pacing,
    double phase_pacing_gain,
    int64_t modulation_amp_bps,
    int64_t modulation_amp_eff_bps,
    double triangle_wave,
    bool actual_modulation_on) const {
  if (!enable_convergence_gate_trace_) {
    return;
  }

  QUIC_DVLOG(2)
      << "FBBR pacing_gate"
      << " phase=" << PhaseApplicationName(GetCurrentProbeBwPhase())
      << " current_native_bw_bps=" << b_native.ToBitsPerSecond()
      << " pacing_base_bw_bps=" << pacing_base_bw.ToBitsPerSecond()
      << " pacing_base_source=" << PacingBaseSourceName(pacing_base_source)
      << " phase_pacing_gain=" << phase_pacing_gain
      << " native_pacing_bps=" << native_pacing.ToBitsPerSecond()
      << " final_pacing_rate_bps=" << final_pacing.ToBitsPerSecond()
      << " trusted_bw_bps="
      << (trusted_bw_valid_ ? trusted_bw_.ToBitsPerSecond() : 0)
      << " trusted_bw_fresh=" << trusted_bw_fresh_
      << " trusted_bw_application_valid="
      << trusted_bw_application_valid_
      << " triangle_wave=" << triangle_wave
      << " actual_modulation_on=" << actual_modulation_on;

  bool emit_csv = gate_trace_mode_ == FBBRGateTraceMode::kFull;
  if (gate_trace_mode_ == FBBRGateTraceMode::kSampledPacing) {
    if (!last_pacing_gate_trace_time_.IsInitialized() ||
        current_time_ - last_pacing_gate_trace_time_ >=
            gate_trace_sample_interval_) {
      emit_csv = true;
    }
  }
  if (!emit_csv) {
    return;
  }
  last_pacing_gate_trace_time_ = current_time_;

  EmitFreqGateCsvRow("pacing",
                     current_time_,
                     d_round_,
                     d_round_valid_,
                     d_prev_,
                     d_prev_valid_,
                     -1.0,
                     prev_v_round_,
                     false,
                     b_native,
                     pacing_base_bw,
                     pacing_base_source,
                     native_pacing,
                     final_pacing,
                     phase_pacing_gain,
                     modulation_amp_bps,
                     modulation_amp_eff_bps,
                     triangle_wave,
                     DeliveryRateLatest(),
                     false,
                     false);
}

void FBBRSender::EmitFreqGateCsvRow(
    const char* row_type,
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
    bool sample_valid) const {
  if (!enable_convergence_gate_trace_ || !cruise_load_trace_cb_) {
    return;
  }

  const double time_s =
      static_cast<double>((event_time - QuicTime::Zero()).ToMicroseconds()) /
      1000000.0;
  const Bbr2ProbeBwMode::CyclePhase phase = GetCurrentProbeBwPhase();
  std::ostringstream row;
  row << time_s << ","
      << trace_flow_id_ << ","
      << row_type << ","
      << model_.RoundTripCount() << ","
      << GetCurrentBbrModeIndex() << ","
      << static_cast<int>(phase) << ","
      << (IsCruisePhase(phase) ? "true" : "false") << ","
      << (completed_d_valid ? completed_d_round.ToBitsPerSecond() : 0) << ","
      << (previous_d_valid ? previous_d_round.ToBitsPerSecond() : 0) << ","
      << (completed_d_valid ? "true" : "false") << ","
      << (previous_d_valid ? "true" : "false") << ","
      << v_round << ","
      << previous_v_round << ","
      << (full_drate_ref_valid_ ? full_drate_ref_.ToBitsPerSecond() : 0) << ","
      << stable_cnt_ << ","
      << (bbr_stable_ ? "true" : "false") << ","
      << (just_exited ? "true" : "false") << ","
      << (freq_tool_needed_ ? "true" : "false") << ","
      << (freq_tool_on_ ? "true" : "false") << ","
      << w_freq_ << ","
      << unstable_episode_id_ << ","
      << (unstable_episode_active_ ? "true" : "false") << ","
      << selection_native_bw_.ToBitsPerSecond() << ","
      << b_native.ToBitsPerSecond() << ","
      << (trusted_bw_valid_ ? trusted_bw_.ToBitsPerSecond() : 0) << ","
      << (trusted_bw_valid_ ? "true" : "false") << ","
      << trusted_bw_conf_ << ","
      << trusted_bw_source_ << ","
      << trusted_bw_cruise_id_ << ","
      << (trusted_bw_fresh_ ? "true" : "false") << ","
      << (trusted_bw_application_valid_ ? "true" : "false") << ","
      << (trusted_bw_ready_for_post_cruise_ ? "true" : "false") << ","
      << trusted_bw_application_phase_ << ","
      << trusted_bw_invalid_reason_ << ","
      << drate_spectral_integrity_score_ << ","
      << srtt_spectral_integrity_score_ << ","
      << joint_spectral_integrity_score_ << ","
      << (drate_spectral_gate_pass_ ? "true" : "false") << ","
      << (srtt_spectral_gate_pass_ ? "true" : "false") << ","
      << (dual_signal_spectral_gate_pass_ ? "true" : "false") << ","
      << limiting_spectral_signal_ << ","
      << pacing_base_bw.ToBitsPerSecond() << ","
      << PacingBaseSourceName(pacing_base_source) << ","
      << phase_pacing_gain << ","
      << native_pacing.ToBitsPerSecond() << ","
      << final_pacing.ToBitsPerSecond() << ","
      << modulation_amp_bps << ","
      << modulation_amp_eff_bps << ","
      << triangle_wave << ","
      << current_delivery_rate.ToBitsPerSecond() << ","
      << (sample_is_app_limited ? "true" : "false") << ","
      << (sample_valid ? "true" : "false") << ","
      << (merged_rescue_attempted_ ? "true" : "false") << ","
      << (merged_rescue_success_ ? "true" : "false") << ","
      << trusted_bw_selection_compute_us_ << ","
      << normal_window_count_ << ","
      << merged_window_count_ << ","
      << spectral_invalid_count_ << ","
      << (trusted_bw_cleared_on_cruise_start_ ? "true" : "false") << ","
      << "F-BBR" << ","
      << (fbbr_frequency_search_config_.frequency_search_enabled ? "CODED_SINE" : "LEGACY_TRIANGLE")
      << ","
      << (1.0 + (fbbr_frequency_search_config_.frequency_search_enabled
                     ? fbbr_probe_signature_.amplitude_ratio * triangle_wave
                     : 0.0)) << ","
      << fbbr_probe_signature_.period_rtts << ","
      << fbbr_probe_signature_.code_id;
  cruise_load_trace_cb_(time_s,
                        time_s,
                        0.0,
                        0.0,
                        0.0,
                        0.0,
                        "FREQ_GATE_TRACE",
                        false,
                        row.str());
}
void FBBRSender::OnPacketSent(QuicTime sent_time,
                                  QuicByteCount bytes_in_flight,
                                  QuicPacketNumber packet_number,
                                  QuicByteCount bytes,
                                  HasRetransmittableData is_retransmittable) {
  current_time_ = sent_time;
  QuicBandwidth sender_rate = PacingRate(bytes_in_flight);
  if (!fbbr_frequency_search_config_.frequency_search_enabled &&
      fbbr_frequency_search_config_.legacy_spectral_path_enabled) {
    sender_rate_history_.push_back({sent_time, sender_rate});
    while (sender_rate_history_.size() > kMaxHistorySamples) {
      sender_rate_history_.pop_front();
    }
  }

  if (fbbr_frequency_search_config_.frequency_search_enabled && in_cruise_) {
    FBBRFrequencySearchAccumulateSend(
        sent_time, bytes_in_flight, bytes,
        is_retransmittable == HAS_RETRANSMITTABLE_DATA);
  }

  Bbr2Sender::OnPacketSent(sent_time,
                           bytes_in_flight,
                           packet_number,
                           bytes,
                           is_retransmittable);
}

void FBBRSender::OnCongestionEvent(
    bool rtt_updated,
    QuicByteCount prior_in_flight,
    QuicTime event_time,
    const AckedPacketVector& acked_packets,
    const LostPacketVector& lost_packets) {
  current_time_ = event_time;
  QuicByteCount acked_bytes = 0;
  for (const auto& ack : acked_packets) {
    acked_bytes += ack.bytes_acked;
  }

  Bbr2Sender::OnCongestionEvent(rtt_updated,
                                prior_in_flight,
                                event_time,
                                acked_packets,
                                lost_packets);

  if (fbbr_frequency_search_config_.frequency_search_enabled && !in_cruise_ &&
      trusted_bw_application_valid_) {
    const DebugState state = ExportDebugState();
    if (state.last_sample_is_app_limited) {
      ClearTrustedBwApplication("post_cruise_app_limited");
      trusted_bw_invalid_reason_ = "post_cruise_app_limited";
    }
    const double trusted_bps = static_cast<double>(trusted_bw_.ToBitsPerSecond());
    const double native_bps = static_cast<double>(
        BandwidthEstimate().ToBitsPerSecond());
    if (trusted_bps > 0.0 && native_bps > 0.0 &&
        std::abs(native_bps - trusted_bps) / trusted_bps > 0.20) {
      ClearTrustedBwApplication("post_cruise_native_deviation");
      trusted_bw_invalid_reason_ = "post_cruise_native_deviation";
    }
    if (trusted_bw_age_cruises_ > fbbr_frequency_search_config_.trusted_ttl_cruises) {
      ClearTrustedBw("trusted_ttl_expired");
    }
    uint64_t lost_bytes = 0;
    for (const auto& lost : lost_packets) lost_bytes += lost.bytes_lost;
    const double event_loss_ratio = acked_bytes + lost_bytes > 0
        ? static_cast<double>(lost_bytes) /
              static_cast<double>(acked_bytes + lost_bytes)
        : 0.0;
    const double event_ecn_ratio = acked_bytes > 0
        ? static_cast<double>(GetBytesEcnInRounds()) / acked_bytes
        : 0.0;
    if (event_loss_ratio > fbbr_frequency_search_config_.max_loss_ratio_for_trusted ||
        event_ecn_ratio > fbbr_frequency_search_config_.max_ecn_ratio_for_trusted) {
      ClearTrustedBwApplication("post_cruise_congestion");
      trusted_bw_invalid_reason_ = "post_cruise_congestion";
    }
    TimeDelta current_rtprop = model_.MinRtt();
    if (!fbbr_rtprop_frozen_.IsZero() && !current_rtprop.IsZero()) {
      const double rtprop_step = std::abs(
          static_cast<double>(current_rtprop.ToMicroseconds() -
                              fbbr_rtprop_frozen_.ToMicroseconds())) /
          std::max<double>(1.0, fbbr_rtprop_frozen_.ToMicroseconds());
      if (rtprop_step > 0.20) {
        ClearTrustedBwApplication("post_cruise_rtprop_step");
        trusted_bw_invalid_reason_ = "post_cruise_rtprop_step";
      }
    }
  }

  if (mode_ != Bbr2Mode::PROBE_BW) {
    ClearTrustedBw("non_probe_bw");
  } else if (enable_convergence_gate_control_ && bbr_stable_) {
    ClearTrustedBw("stable_closure");
  }

  if (!drain_completed_ && mode_ == Bbr2Mode::PROBE_BW) {
    drain_completed_ = true;
  }

  if (!fbbr_frequency_search_config_.frequency_search_enabled &&
      fbbr_frequency_search_config_.legacy_spectral_path_enabled &&
      last_ack_time_ != QuicTime::Zero() && event_time > last_ack_time_ &&
      acked_bytes > 0) {
    QuicBandwidth recv_signal = use_delivery_rate_latest_for_signal_history_
                                    ? DeliveryRateLatest()
                                    : BandwidthLatest();
    delivery_rate_history_.push_back({event_time, recv_signal});
    if (rtt_stats_ != nullptr) {
      TimeDelta smoothed_rtt = rtt_stats_->smoothed_rtt();
      if (smoothed_rtt.IsZero()) {
        smoothed_rtt = rtt_stats_->SmoothedOrInitialRtt();
      }
      if (!smoothed_rtt.IsZero()) {
        srtt_history_.push_back(
            {event_time,
             static_cast<double>(smoothed_rtt.ToMicroseconds()) / 1000.0});
      }
    }
    ack_window_history_.push_back(
        {event_time, acked_bytes, !lost_packets.empty()});

    while (delivery_rate_history_.size() > kMaxHistorySamples) {
      delivery_rate_history_.pop_front();
    }
    while (srtt_history_.size() > kMaxHistorySamples) {
      srtt_history_.pop_front();
    }
    while (ack_window_history_.size() > kMaxHistorySamples) {
      ack_window_history_.pop_front();
    }
  }
  last_ack_time_ = event_time;

  if (in_cruise_ && mode_ == Bbr2Mode::PROBE_BW &&
      GetCurrentProbeBwPhase() == Bbr2ProbeBwMode::CyclePhase::PROBE_CRUISE) {
    if (fbbr_frequency_search_config_.frequency_search_enabled) {
      FBBRFrequencySearchFinalizeReadyBlocks(event_time, false);
    } else {
      RunDueCruiseWindowAnalysis(event_time);
    }
  }
}

QuicBandwidth FBBRSender::PacingRate(
    QuicByteCount bytes_in_flight) const {
  // PacingRate() is queried by the pacer between packet/ACK callbacks.  Using
  // the last callback timestamp freezes or phase-shifts the carrier under
  // multi-flow scheduling; query the transport clock so commanded and actual
  // input are defined on the same time axis.
  if (clock_ != nullptr) current_time_ = clock_->Now();
  const QuicBandwidth native_pacing =
      Bbr2Sender::PacingRate(bytes_in_flight);
  const QuicBandwidth native_bw = BandwidthEstimate();
  const Bbr2ProbeBwMode::CyclePhase phase = GetCurrentProbeBwPhase();
  const double phase_gain = static_cast<double>(PacingGain());

  QuicBandwidth pacing_base_bw = native_bw;
  FBBRPacingBaseSource pacing_base_source =
      FBBRPacingBaseSource::kNativeBbr;
  const bool trusted_value_is_usable =
      trusted_bw_valid_ && !trusted_bw_.IsZero() &&
      std::isfinite(static_cast<double>(trusted_bw_.ToBitsPerSecond()));
  const bool trusted_result_is_fresh =
      trusted_bw_fresh_ &&
      trusted_bw_cruise_id_ ==
          static_cast<uint64_t>(std::max<int64_t>(0, cruise_id_));
  const bool stable_allows_application =
      !enable_convergence_gate_control_ || !bbr_stable_;
  const bool use_trusted_bw =
      IsTrustedBwApplicationPhase(phase) &&
      trusted_bw_application_valid_ &&
      trusted_bw_ready_for_post_cruise_ &&
      trusted_result_is_fresh &&
      trusted_value_is_usable &&
      stable_allows_application;

  if (use_trusted_bw) {
    pacing_base_bw = trusted_bw_;
    pacing_base_source = FBBRPacingBaseSource::kTrustedBw;
    trusted_bw_application_phase_ = PhaseApplicationName(phase);
  } else {
    trusted_bw_application_phase_ = PhaseApplicationName(phase);
  }

  QuicBandwidth baseline_pacing = native_pacing;
  const bool use_adaptive_cruise_baseline =
      fbbr_frequency_search_config_.search_controller_enabled &&
      fbbr_probe_active_ && in_cruise_ &&
      phase == Bbr2ProbeBwMode::CyclePhase::PROBE_CRUISE;
  if (use_trusted_bw) {
    baseline_pacing =
        static_cast<float>(phase_gain) * pacing_base_bw;
  } else if (use_adaptive_cruise_baseline) {
    // CRUISE has unit pacing gain. Use the F-BBR search baseline directly;
    // using the parent's cached pacing rate here would reintroduce per-ACK
    // drift and could apply the phase gain twice.
    const double search_baseline_bps =
        FBBRSearchSearchBaselineBps(current_time_);
    const double servo_factor = ClampValue(
        fbbr_queue_servo_state_.factor, 0.10, 1.02);
    baseline_pacing = BandwidthFromBps(
        search_baseline_bps * servo_factor);
    pacing_base_bw = BandwidthFromBps(search_baseline_bps);
    pacing_base_source = FBBRPacingBaseSource::kAdaptiveSearch;
  }

  const bool base_should_oscillate = BaseShouldOscillate();
  // Frequency Search freezes the probe decision for the CRUISE block.  A convergence
  // transition may invalidate the block, but must never change its waveform
  // half-way through.  The legacy path retains its historical gate behavior.
  const bool should_oscillate = fbbr_frequency_search_config_.frequency_search_enabled
      ? base_should_oscillate
      : (enable_convergence_gate_control_
             ? (base_should_oscillate && !bbr_stable_)
             : base_should_oscillate);
  freq_tool_on_ = should_oscillate;
  const double probe_wave = should_oscillate
      ? (fbbr_frequency_search_config_.frequency_search_enabled
             ? FBBRFrequencySearchCodedSineValue(current_time_)
             : TriangleWave(current_time_))
      : 0.0;
  const int64_t amplitude_bps = should_oscillate
      ? (fbbr_frequency_search_config_.frequency_search_enabled
             ? static_cast<int64_t>(fbbr_probe_signature_.amplitude_ratio *
                                    baseline_pacing.ToBitsPerSecond())
             : static_cast<int64_t>(GetCurrentAmplitudeBps()))
      : 0;
  // F-BBR uses a multiplicative relative dither so its queue budget and
  // identifiability have the same meaning at every bottleneck rate.  A pure
  // sinusoid has one carrier; therefore its second harmonic is attributable
  // to saturation/zero-queue nonlinearities rather than the input waveform.
  const int64_t offset_bps =
      static_cast<int64_t>(amplitude_bps * probe_wave);
  int64_t final_bps = static_cast<int64_t>(baseline_pacing.ToBitsPerSecond()) +
                      offset_bps;
  final_bps = std::max<int64_t>(1000, final_bps);
  const QuicBandwidth final_pacing =
      QuicBandwidth::FromBitsPerSecond(static_cast<uint64_t>(final_bps));
  const QuicBandwidth returned_pacing =
      (!should_oscillate && !use_trusted_bw && !use_adaptive_cruise_baseline)
          ? native_pacing : final_pacing;

  EmitPacingTrace(native_bw,
                  pacing_base_bw,
                  pacing_base_source,
                  native_pacing,
                  returned_pacing,
                  phase_gain,
                  amplitude_bps,
                  amplitude_bps,
                  probe_wave,
                  should_oscillate);
  if (pacing_audit_trace_cb_) {
    const double time_s =
        static_cast<double>((current_time_ - QuicTime::Zero())
                                .ToMicroseconds()) /
        1000000.0;
    pacing_audit_trace_cb_(
        time_s,
        native_pacing.ToBitsPerSecond(),
        returned_pacing.ToBitsPerSecond(),
        native_bw.ToBitsPerSecond(),
        pacing_base_bw.ToBitsPerSecond(),
        PacingBaseSourceName(pacing_base_source),
        phase_gain,
        should_oscillate,
        trusted_bw_valid_);
  }
  return returned_pacing;
}

FbbrPacketPacingDebugState FBBRSender::ExportPacketPacingDebugState(
    QuicTime sent_time,
    QuicByteCount bytes_in_flight,
    uint64_t commanded_pacing_bps) const {
  FbbrPacketPacingDebugState state;
  const bool cruise_search = fbbr_frequency_search_config_.frequency_search_enabled &&
      fbbr_probe_active_ && in_cruise_ &&
      GetCurrentProbeBwPhase() ==
          Bbr2ProbeBwMode::CyclePhase::PROBE_CRUISE;
  state.search_baseline_bps = cruise_search
      ? static_cast<uint64_t>(std::llround(std::max(
            0.0, FBBRSearchSearchBaselineBps(sent_time) *
                ClampValue(fbbr_queue_servo_state_.factor, 0.10, 1.02))))
      : BandwidthEstimate().ToBitsPerSecond();
  state.commanded_probe_offset_bps =
      static_cast<int64_t>(commanded_pacing_bps) -
      static_cast<int64_t>(state.search_baseline_bps);
  if (cruise_search && !fbbr_period_.IsZero() &&
      cruise_start_time_ != QuicTime::Zero() && sent_time >= cruise_start_time_) {
    const double elapsed_s = static_cast<double>(
        (sent_time - cruise_start_time_).ToMicroseconds()) / 1e6;
    state.carrier_phase = std::fmod(
        fbbr_probe_signature_.initial_phase_rad +
            2.0 * M_PI * elapsed_s /
                std::max(fbbr_probe_signature_.period_s, 1e-9),
        2.0 * M_PI);
  }
  state.cwnd_bytes = GetCongestionWindow();
  state.search_active = fbbr_search_state_.search_active;
  state.is_pulser = cruise_search &&
      fbbr_search_state_.is_pulser;
  state.is_cwnd_limited = bytes_in_flight >= state.cwnd_bytes;
  state.is_app_limited = fbbr_last_sample_app_limited_;
  return state;
}

int32_t FBBRSender::GetCurrentBbrModeIndex() const {
  return Bbr2Sender::GetCurrentBbrModeIndex();
}

void FBBRSender::RunDueCruiseWindowAnalysis(QuicTime now) {
  if (fbbr_frequency_search_config_.frequency_search_enabled) {
    FBBRFrequencySearchFinalizeReadyBlocks(now, false);
    return;
  }
  if (!fbbr_frequency_search_config_.legacy_spectral_path_enabled) {
    return;
  }
  if (configured_modulation_freq_hz_ <= 0.0 ||
      cruise_start_time_ == QuicTime::Zero()) {
    return;
  }

  TimeDelta min_rtt = model_.MinRtt();
  if (min_rtt.IsZero() && rtt_stats_ != nullptr) {
    min_rtt = rtt_stats_->MinOrInitialRtt();
  }
  if (min_rtt.IsZero()) {
    if (!min_rtt_warning_logged_) {
      QUIC_DVLOG(1) << "FBBR: minRTT unavailable; skip CRUISE "
                       "window classification instead of RTT alignment";
      min_rtt_warning_logged_ = true;
    }
    return;
  }

  if (next_cruise_window_start_ == QuicTime::Zero()) {
    next_cruise_window_start_ = cruise_start_time_ + min_rtt;
  }

  const double min_rtt_s =
      static_cast<double>(min_rtt.ToMicroseconds()) / 1000000.0;
  const double window_duration_s =
      std::max(min_cruise_cycles_per_window_ /
                   configured_modulation_freq_hz_,
               2.0 * min_rtt_s);
  const TimeDelta window_duration =
      TimeDelta::FromMicroseconds(static_cast<int64_t>(
          window_duration_s * 1000000.0));
  TimeDelta step = TimeDelta::FromMicroseconds(static_cast<int64_t>(
      cruise_window_step_ratio_ * window_duration.ToMicroseconds()));
  if (step < TimeDelta::FromMilliseconds(1)) {
    step = TimeDelta::FromMilliseconds(1);
  }

  while (next_cruise_window_start_ != QuicTime::Zero() &&
         next_cruise_window_start_ + window_duration <= now &&
         in_cruise_) {
    AnalyzeCruiseWindow(next_cruise_window_start_,
                        next_cruise_window_start_ + window_duration,
                        min_rtt,
                        window_duration_s);
    next_cruise_window_start_ = next_cruise_window_start_ + step;
  }
}

void FBBRSender::AnalyzeCruiseWindow(QuicTime window_start,
                                         QuicTime window_end,
                                         TimeDelta min_rtt,
                                         double window_duration_s) {
  current_cruise_windows_.push_back(BuildCruiseWindowResult(
      window_start, window_end, min_rtt, window_duration_s, "NORMAL"));
}

FBBRSender::CruiseWindowResult FBBRSender::BuildCruiseWindowResult(
    QuicTime window_start,
    QuicTime window_end,
    TimeDelta min_rtt,
    double window_duration_s,
    const std::string& window_source) {
  const double reference_freq = configured_modulation_freq_hz_;
  const double freq_tolerance =
      std::max(freq_tolerance_ratio_ * reference_freq,
               2.0 / window_duration_s);

  const QuicTime sender_start = window_start - min_rtt;
  const QuicTime sender_end = window_end - min_rtt;

  auto srate_samples =
      SelectRateSamples(sender_rate_history_, sender_start, sender_end);
  auto drate_samples =
      SelectRateSamples(delivery_rate_history_, window_start, window_end);
  auto srtt_samples = SelectRttSamples(srtt_history_, window_start, window_end);

  WindowSignalResult srate =
      AnalyzeRateSeries(srate_samples, sender_start, sender_end, reference_freq,
                        false);
  WindowSignalResult drate =
      AnalyzeRateSeries(drate_samples, window_start, window_end, reference_freq,
                        false);
  WindowSignalResult srtt =
      AnalyzeRttSeries(srtt_samples, window_start, window_end, reference_freq,
                       true);
  const bool drate_input_valid = HasValidRateCoverage(
      drate_samples, window_start, window_end, reference_freq);
  const bool srtt_input_valid = HasValidRttCoverage(
      srtt_samples, window_start, window_end, reference_freq);
  drate.valid = drate.valid && drate_input_valid;
  srtt.valid = srtt.valid && srtt_input_valid;

  CruiseWindowResult result = {cruise_id_, window_start, window_end};
  result.window_source = window_source;
  result.configured_modulation_freq_hz = reference_freq;
  result.srate_peak_freq_hz = srate.profile.peak_freq_hz;
  result.drate_peak_freq_hz = drate.profile.peak_freq_hz;
  result.srtt_peak_freq_hz = srtt.profile.peak_freq_hz;
  result.drate_mean_kbps = drate.valid ? drate.mean_value : 0.0;
  result.srate_valid = srate.valid;
  result.drate_valid = drate.valid;
  result.srtt_valid = srtt.valid;
  result.srate_sample_count = srate_samples.size();
  result.drate_sample_count = drate_samples.size();
  result.srtt_sample_count = srtt_samples.size();
  result.full_load_rank_in_cruise = -1;
  result.is_best_full_load_window = false;

  result.srate_freq_score =
      srate.valid ? ComputeFreqScore(srate.profile.peak_freq_hz,
                                     reference_freq,
                                     freq_tolerance)
                  : 0.0;
  result.drate_freq_score =
      drate.valid ? ComputeFreqScore(drate.profile.peak_freq_hz,
                                     reference_freq,
                                     freq_tolerance)
                  : 0.0;
  result.srtt_freq_score =
      srtt.valid ? ComputeFreqScore(srtt.profile.peak_freq_hz,
                                    reference_freq,
                                    freq_tolerance)
                 : 0.0;
  result.freq_quality =
      std::min(result.drate_freq_score, result.srtt_freq_score);

  const double epsilon = 1e-9;
  result.srate_target_amp = srate.valid ? srate.profile.target_amp : 0.0;
  result.drate_target_amp = drate.valid ? drate.profile.target_amp : 0.0;
  result.drate_noise_floor = drate.profile.noise_floor;
  result.drate_snr =
      drate.profile.noise_floor_valid
          ? result.drate_target_amp / std::max(result.drate_noise_floor, epsilon)
          : 0.0;
  result.drate_gain =
      result.drate_target_amp / std::max(result.srate_target_amp, epsilon);
  result.drate_amplitude_score =
      Clamp01(result.drate_gain / kTargetDrateGain);

  result.srtt_target_amp = srtt.valid ? srtt.profile.target_amp : 0.0;
  result.srtt_noise_floor = srtt.profile.noise_floor;
  result.srtt_snr =
      srtt.profile.noise_floor_valid
          ? result.srtt_target_amp / std::max(result.srtt_noise_floor, epsilon)
          : 0.0;
  result.srtt_amplitude_score =
      srtt.profile.noise_floor_valid
          ? ScoreThreshold(result.srtt_snr, min_srtt_snr_, kTargetSrttSnr)
          : 0.5;
  if (srtt.valid && !srtt.profile.noise_floor_valid) {
    QUIC_DVLOG(1) << "FBBR: SRTT noise-floor estimate failed; "
                     "using srtt_amplitude_score=0.5";
  }
  result.amplitude_quality =
      0.4 * result.drate_amplitude_score +
      0.6 * result.srtt_amplitude_score;

  double drate_shape_distance = kMaxDrateShapeDistance;
  if (drate.valid && srate.valid) {
    drate_shape_distance = ComputeSpectrumShapeDistance(
        drate.profile.band_shape, srate.profile.band_shape);
    result.drate_waveform_quality =
        Clamp01(1.0 - drate_shape_distance / kMaxDrateShapeDistance);
  } else {
    result.drate_waveform_quality = 0.5;
    QUIC_DVLOG(1) << "FBBR: missing srate/drate spectrum shape; "
                     "using drate_waveform_quality=0.5";
  }

  CycleQualityMetrics srtt_cycles =
      AnalyzeCycleQuality(srtt.values, kSampleStepSec, reference_freq, true);
  result.srtt_waveform_quality = srtt_cycles.waveform_quality;
  result.srtt_top_clip_ratio = srtt_cycles.top_clip_ratio;
  result.srtt_bottom_clip_ratio = srtt_cycles.bottom_clip_ratio;
  result.srtt_distortion_score = srtt_cycles.distortion_score;
  result.cycle_frequency_stability = srtt_cycles.cycle_frequency_stability;
  result.cycle_phase_stability = srtt_cycles.cycle_phase_stability;
  result.valid_cycle_count = srtt_cycles.valid_cycles;
  if (srtt.valid && srtt_cycles.valid_cycles <= 0) {
    QUIC_DVLOG(1) << "FBBR: SRTT cycle metrics unavailable; "
                     "using waveform/consistency defaults";
  }
  if (!srtt_cycles.phase_reliable) {
    result.cycle_phase_stability = 0.5;
    QUIC_DVLOG(1) << "FBBR: SRTT phase stability unreliable; "
                     "using cycle_phase_stability=0.5";
  }

  result.waveform_quality =
      0.4 * result.drate_waveform_quality +
      0.6 * result.srtt_waveform_quality;
  result.consistency_quality =
      0.5 * result.cycle_frequency_stability +
      0.5 * result.cycle_phase_stability;

  result.drate_band_energy_rel =
      drate.valid ? drate.profile.band_energy_rel : 0.0;
  result.srtt_band_energy_rel =
      srtt.valid ? srtt.profile.band_energy_rel : 0.0;
  result.drate_band_peak_rel =
      drate.valid ? drate.profile.band_peak_rel : 0.0;
  result.srtt_band_peak_rel =
      srtt.valid ? srtt.profile.band_peak_rel : 0.0;
  result.srate_peak_width_hz =
      srate.valid ? srate.profile.peak_width_hz : 0.0;
  result.drate_peak_width_hz =
      drate.valid ? drate.profile.peak_width_hz : 0.0;
  result.srtt_peak_width_hz =
      srtt.valid ? srtt.profile.peak_width_hz : 0.0;
  const double minimum_resolvable_width =
      std::max(1e-9, 1.0 / std::max(window_duration_s, 1e-9));
  const double drate_width_denom =
      std::max(drate.valid ? drate.profile.freq_step_hz : 0.0,
               minimum_resolvable_width);
  const double srtt_width_denom =
      std::max(srtt.valid ? srtt.profile.freq_step_hz : 0.0,
               minimum_resolvable_width);
  result.drate_width_ratio =
      drate.valid ? result.drate_peak_width_hz / drate_width_denom
                  : std::numeric_limits<double>::infinity();
  result.srtt_width_ratio =
      srtt.valid ? result.srtt_peak_width_hz / srtt_width_denom
                 : std::numeric_limits<double>::infinity();
  bool drate_phase_valid = false;
  bool srtt_phase_valid = false;
  result.drate_phase_coherence =
      ComputePhaseCoherence(drate.values, kSampleStepSec, reference_freq,
                            &drate_phase_valid);
  result.srtt_phase_coherence =
      ComputePhaseCoherence(srtt.values, kSampleStepSec, reference_freq,
                            &srtt_phase_valid);

const double sigma_f = std::max(freq_sigma_ratio_ * reference_freq,
                                  0.5 / std::max(window_duration_s, 1e-9));
  const double q_freq_drate =
      drate.valid ? ExpFreqScore(std::abs(result.drate_peak_freq_hz -
                                          reference_freq),
                                 sigma_f)
                  : 0.0;
  const double q_freq_srtt =
      srtt.valid ? ExpFreqScore(std::abs(result.srtt_peak_freq_hz -
                                         reference_freq),
                                sigma_f)
                 : 0.0;
  const double q_snr_drate =
      drate.profile.noise_floor_valid
          ? LogisticScore(result.drate_snr, min_drate_snr_, snr_slope_)
          : 0.0;
  const double q_snr_srtt =
      srtt.profile.noise_floor_valid
          ? LogisticScore(result.srtt_snr, min_srtt_snr_, snr_slope_)
          : 0.0;
  const double q_energy_drate =
      LogisticScore(result.drate_band_energy_rel, energy_threshold_,
                    energy_slope_);
  const double q_energy_srtt =
      LogisticScore(result.srtt_band_energy_rel, energy_threshold_,
                    energy_slope_);
  const double q_width_drate =
      WidthScore(result.drate_width_ratio, width_r0_drate_, width_sigma_);
  const double q_width_srtt =
      WidthScore(result.srtt_width_ratio, width_r0_srtt_, width_sigma_);
  const double q_phase_drate =
      drate_phase_valid ? Clamp01(result.drate_phase_coherence) : 0.0;
  const double q_phase_srtt =
      srtt_phase_valid ? Clamp01(result.srtt_phase_coherence) : 0.0;

  const double drate_product =
      q_freq_drate * q_snr_drate * q_energy_drate *
      q_width_drate * q_phase_drate;
  const double srtt_product =
      q_freq_srtt * q_snr_srtt * q_energy_srtt *
      q_width_srtt * q_phase_srtt;
  result.drate_spectral_integrity_score =
      drate.valid && std::isfinite(drate_product)
          ? Clamp01(std::pow(std::max(0.0, drate_product), 1.0 / 5.0))
          : 0.0;
  result.srtt_spectral_integrity_score =
      srtt.valid && std::isfinite(srtt_product)
          ? Clamp01(std::pow(std::max(0.0, srtt_product), 1.0 / 5.0))
          : 0.0;
  result.joint_spectral_integrity_score =
      std::min(result.drate_spectral_integrity_score,
               result.srtt_spectral_integrity_score);
  if (result.drate_spectral_integrity_score <
      result.srtt_spectral_integrity_score) {
    result.limiting_spectral_signal = kLimitingSpectralSignalDrate;
  } else if (result.srtt_spectral_integrity_score <
             result.drate_spectral_integrity_score) {
    result.limiting_spectral_signal = kLimitingSpectralSignalSrtt;
  } else {
    result.limiting_spectral_signal = kLimitingSpectralSignalEqual;
  }

  result.drate_spectral_gate_pass =
      result.drate_valid && drate_input_valid &&
      drate.profile.noise_floor_valid && drate_phase_valid &&
      std::isfinite(result.drate_snr) &&
      result.drate_snr >= min_drate_snr_ &&
      std::isfinite(result.drate_width_ratio) &&
      result.drate_width_ratio <= max_drate_width_ratio_ &&
      std::isfinite(result.drate_phase_coherence) &&
      result.drate_phase_coherence >= min_drate_phase_coherence_ &&
      std::isfinite(result.drate_spectral_integrity_score) &&
      result.drate_spectral_integrity_score >=
          drate_spectral_integrity_threshold_;
  result.srtt_spectral_gate_pass =
      result.srtt_valid && srtt_input_valid &&
      srtt.profile.noise_floor_valid && srtt_phase_valid &&
      std::isfinite(result.srtt_snr) &&
      result.srtt_snr >= min_srtt_snr_ &&
      std::isfinite(result.srtt_width_ratio) &&
      result.srtt_width_ratio <= max_srtt_width_ratio_ &&
      std::isfinite(result.srtt_phase_coherence) &&
      result.srtt_phase_coherence >= min_srtt_phase_coherence_ &&
      std::isfinite(result.srtt_spectral_integrity_score) &&
      result.srtt_spectral_integrity_score >=
          srtt_spectral_integrity_threshold_;
  result.dual_signal_spectral_gate_pass =
      result.drate_spectral_gate_pass &&
      result.srtt_spectral_gate_pass;

  std::string invalid_reason = "none";
  if (!result.drate_valid || !drate_input_valid) {
    invalid_reason = AppendReason(invalid_reason, "drate_invalid");
  }
  if (!result.srtt_valid || !srtt_input_valid) {
    invalid_reason = AppendReason(invalid_reason, "srtt_invalid");
  }
  if (!drate.profile.noise_floor_valid) {
    invalid_reason = AppendReason(invalid_reason, "drate_noise_invalid");
  }
  if (!srtt.profile.noise_floor_valid) {
    invalid_reason = AppendReason(invalid_reason, "srtt_noise_invalid");
  }
  if (!std::isfinite(result.drate_snr) ||
      result.drate_snr < min_drate_snr_) {
    invalid_reason = AppendReason(invalid_reason, "drate_snr_low");
  }
  if (!std::isfinite(result.srtt_snr) ||
      result.srtt_snr < min_srtt_snr_) {
    invalid_reason = AppendReason(invalid_reason, "srtt_snr_low");
  }
  if (!std::isfinite(result.drate_width_ratio) ||
      result.drate_width_ratio > max_drate_width_ratio_) {
    invalid_reason = AppendReason(invalid_reason, "drate_peak_broad");
  }
  if (!std::isfinite(result.srtt_width_ratio) ||
      result.srtt_width_ratio > max_srtt_width_ratio_) {
    invalid_reason = AppendReason(invalid_reason, "srtt_peak_broad");
  }
  if (!drate_phase_valid ||
      result.drate_phase_coherence < min_drate_phase_coherence_) {
    invalid_reason = AppendReason(invalid_reason, "drate_phase_invalid");
  }
  if (!srtt_phase_valid ||
      result.srtt_phase_coherence < min_srtt_phase_coherence_) {
    invalid_reason = AppendReason(invalid_reason, "srtt_phase_invalid");
  }
  if (result.drate_spectral_integrity_score <
      drate_spectral_integrity_threshold_) {
    invalid_reason =
        AppendReason(invalid_reason, "drate_integrity_score_low");
  }
  if (result.srtt_spectral_integrity_score <
      srtt_spectral_integrity_threshold_) {
    invalid_reason =
        AppendReason(invalid_reason, "srtt_integrity_score_low");
  }
  result.spectral_invalid_reason = invalid_reason;

  result.is_full_load_candidate =
      result.dual_signal_spectral_gate_pass &&
      result.drate_freq_score >= kMinDrateFreqScoreForCandidate &&
      result.srtt_freq_score >= kMinSrttFreqScoreForCandidate &&
      std::isfinite(result.drate_mean_kbps) &&
      result.drate_mean_kbps > 0.0;
  const double base_quality =
      Clamp01(0.35 * result.freq_quality +
              0.25 * result.waveform_quality +
              0.20 * result.amplitude_quality +
              0.20 * result.consistency_quality);
  result.full_load_quality_v1 = base_quality;
  result.full_load_quality = base_quality;
  result.full_load_quality_v2 =
      result.dual_signal_spectral_gate_pass
          ? Clamp01(result.joint_spectral_integrity_score * base_quality)
          : 0.0;
  result.label = result.is_full_load_candidate
                     ? "FULL_LOAD_CANDIDATE"
                     : "NOT_FULL_LOAD_CANDIDATE";

  const bool srate_unstable =
      !result.srate_valid ||
      result.srate_freq_score < kMinDrateFreqScoreForCandidate;
  const bool samples_insufficient =
      result.drate_sample_count < 4 || result.srtt_sample_count < 4;
  const int expected_cycles =
      static_cast<int>(std::floor(window_duration_s * reference_freq));
  const bool cycles_insufficient =
      expected_cycles < 2 || result.valid_cycle_count < 2;
  result.low_confidence =
      !result.dual_signal_spectral_gate_pass ||
      srate_unstable || samples_insufficient || cycles_insufficient ||
      result.full_load_quality < min_full_load_quality_for_reliable_window_;

  QUIC_DVLOG(2) << "FBBR: CRUISE full-load window ["
                << (window_start - QuicTime::Zero()).ToMicroseconds() / 1000000.0
                << ", "
                << (window_end - QuicTime::Zero()).ToMicroseconds() / 1000000.0
                << "] cruise_id=" << cruise_id_
                << ", candidate=" << result.is_full_load_candidate
                << ", quality=" << result.full_load_quality
                << ", low_confidence=" << result.low_confidence
                << ", drate_freq_score=" << result.drate_freq_score
                << ", srtt_freq_score=" << result.srtt_freq_score
                << ", drate_gain=" << result.drate_gain
	                << ", srtt_snr=" << result.srtt_snr;
  return result;
}

FBBRSender::TrustedBwSelectionResult
FBBRSender::RunTrustedBwSelection(QuicTime now) {
  const auto compute_start = std::chrono::steady_clock::now();
  TrustedBwSelectionResult selection = {
      BandwidthEstimate(),
      QuicBandwidth::Zero(),
      false,
      0.0,
      kTrustedBwSourceNone,
      0.0,
      0.0,
      0.0,
      false,
      false,
      false,
      kLimitingSpectralSignalEqual,
      false,
      false,
      0,
      0,
      0,
      0};

  for (const CruiseWindowResult& result : current_cruise_windows_) {
    if (result.window_source == "NORMAL") {
      ++selection.normal_window_count;
    } else if (result.window_source == "MERGED") {
      ++selection.merged_window_count;
    }
    if (!result.dual_signal_spectral_gate_pass) {
      ++selection.spectral_invalid_count;
    }
  }

  auto finish = [&selection, compute_start]() {
    const auto compute_end = std::chrono::steady_clock::now();
    selection.trusted_bw_selection_compute_us =
        static_cast<uint64_t>(
            std::chrono::duration_cast<std::chrono::microseconds>(
                compute_end - compute_start)
                .count());
    return selection;
  };

  if (!cruise_freq_tool_active_ ||
      (enable_convergence_gate_control_ && bbr_stable_)) {
    return finish();
  }

  auto pick_best = [this](const std::string& source) -> CruiseWindowResult* {
    CruiseWindowResult* best = nullptr;
    for (CruiseWindowResult& result : current_cruise_windows_) {
      if (result.window_source != source ||
          !result.dual_signal_spectral_gate_pass ||
          !IsReliableSpectralWindow(result)) {
        continue;
      }
      if (best == nullptr ||
          result.full_load_quality_v2 > best->full_load_quality_v2) {
        best = &result;
      }
    }
    return best;
  };

  CruiseWindowResult* best = pick_best("NORMAL");
  const char* source = kTrustedBwSourceNormal;
  double confidence_discount = 1.0;

  if (best == nullptr && merged_rescue_enabled_ && max_merged_passes_ > 0) {
    selection.merged_rescue_attempted = true;
    TimeDelta min_rtt = model_.MinRtt();
    if (min_rtt.IsZero() && rtt_stats_ != nullptr) {
      min_rtt = rtt_stats_->MinOrInitialRtt();
    }
    const size_t normal_count_before_merge = current_cruise_windows_.size();
    if (!min_rtt.IsZero()) {
      for (size_t i = 0; i < normal_count_before_merge; ++i) {
        const CruiseWindowResult& normal = current_cruise_windows_[i];
        if (normal.window_source != "NORMAL") {
          continue;
        }
        const TimeDelta normal_duration =
            normal.window_end - normal.window_start;
        if (normal_duration.ToMicroseconds() <= 0) {
          continue;
        }
        QuicTime merged_end =
            normal.window_start +
            TimeDelta::FromMicroseconds(static_cast<int64_t>(
                merged_window_multiplier_ *
                static_cast<double>(normal_duration.ToMicroseconds())));
        if (merged_end > now) {
          merged_end = now;
        }
        if (merged_end <= normal.window_start) {
          continue;
        }
        const double merged_duration_s =
            static_cast<double>((merged_end - normal.window_start)
                                    .ToMicroseconds()) /
            1000000.0;
        CruiseWindowResult merged = BuildCruiseWindowResult(
            normal.window_start,
            merged_end,
            min_rtt,
            merged_duration_s,
            "MERGED");
        const double trend_ratio =
            ComputeRateTrendRatio(merged.window_start, merged.window_end);
        if (!std::isfinite(trend_ratio) ||
            trend_ratio > merged_window_max_trend_ratio_) {
          merged.low_confidence = true;
          merged.is_full_load_candidate = false;
          merged.spectral_invalid_reason =
              AppendReason(merged.spectral_invalid_reason,
                           "merged_trend_high");
          merged.full_load_quality_v2 = 0.0;
        }
        if (!merged.dual_signal_spectral_gate_pass) {
          ++selection.spectral_invalid_count;
        }
        current_cruise_windows_.push_back(merged);
        ++selection.merged_window_count;
      }
    }
    best = pick_best("MERGED");
    if (best != nullptr) {
      selection.merged_rescue_success = true;
      source = kTrustedBwSourceMerged;
      confidence_discount = merged_confidence_discount_;
    }
  }

  if (best == nullptr || !best->dual_signal_spectral_gate_pass) {
    return finish();
  }

  const double trusted_bps = best->drate_mean_kbps * 1000.0;
  if (!std::isfinite(trusted_bps) || trusted_bps <= 0.0) {
    return finish();
  }
  selection.trusted_bw = BandwidthFromBps(trusted_bps);
  selection.trusted_bw_valid = !selection.trusted_bw.IsZero();
  selection.trusted_bw_conf =
      Clamp01(confidence_discount * best->full_load_quality_v2);
  selection.trusted_bw_source = source;
  selection.drate_spectral_integrity_score =
      best->drate_spectral_integrity_score;
  selection.srtt_spectral_integrity_score =
      best->srtt_spectral_integrity_score;
  selection.joint_spectral_integrity_score =
      std::min(selection.drate_spectral_integrity_score,
               selection.srtt_spectral_integrity_score);
  selection.drate_spectral_gate_pass =
      best->drate_spectral_gate_pass;
  selection.srtt_spectral_gate_pass =
      best->srtt_spectral_gate_pass;
  selection.dual_signal_spectral_gate_pass =
      selection.drate_spectral_gate_pass &&
      selection.srtt_spectral_gate_pass;
  selection.limiting_spectral_signal =
      best->limiting_spectral_signal;
  if (!selection.dual_signal_spectral_gate_pass) {
    selection.trusted_bw = QuicBandwidth::Zero();
    selection.trusted_bw_valid = false;
    selection.trusted_bw_source = kTrustedBwSourceNone;
  }
  return finish();
}
void FBBRSender::RankCruiseWindows(
    const CruiseWindowResult* selected_window) {
  (void)selected_window;
  std::vector<size_t> candidate_indices;
  for (size_t i = 0; i < current_cruise_windows_.size(); ++i) {
    current_cruise_windows_[i].full_load_rank_in_cruise = -1;
    current_cruise_windows_[i].is_best_full_load_window = false;
    if (current_cruise_windows_[i].is_full_load_candidate) {
      candidate_indices.push_back(i);
    }
  }
  std::sort(candidate_indices.begin(),
            candidate_indices.end(),
            [this](size_t lhs, size_t rhs) {
              return current_cruise_windows_[lhs].full_load_quality_v2 >
                     current_cruise_windows_[rhs].full_load_quality_v2;
            });

  bool marked_best = false;
  for (size_t rank = 0; rank < candidate_indices.size(); ++rank) {
    CruiseWindowResult& result =
        current_cruise_windows_[candidate_indices[rank]];
    result.full_load_rank_in_cruise = static_cast<int>(rank + 1);
    if (!marked_best && IsReliableSpectralWindow(result)) {
      result.is_best_full_load_window = true;
      marked_best = true;
    }
  }
}

void FBBRSender::FinalizeCruise(QuicTime now) {
  if (fbbr_frequency_search_config_.frequency_search_enabled) {
    FBBRFrequencySearchFinalizeCruise(now);
    return;
  }
  if (!fbbr_frequency_search_config_.legacy_spectral_path_enabled) {
    ClearTrustedBw("frequency_search_and_legacy_disabled");
    return;
  }
  RunDueCruiseWindowAnalysis(now);
  const TrustedBwSelectionResult selection =
      RunTrustedBwSelection(now);
  PublishTrustedBwSelection(selection);
  RankCruiseWindows(nullptr);

  for (const CruiseWindowResult& result : current_cruise_windows_) {
    EmitCruiseWindowTrace(result);
  }

  EmitCruiseSummaryTrace(now);
}

void FBBRSender::EmitCruiseWindowTrace(
    const CruiseWindowResult& result) {
  if (!cruise_load_trace_cb_) {
    return;
  }
  const double start_s =
      static_cast<double>((result.window_start - QuicTime::Zero())
                              .ToMicroseconds()) /
      1000000.0;
  const double end_s =
      static_cast<double>((result.window_end - QuicTime::Zero())
                              .ToMicroseconds()) /
      1000000.0;
  std::ostringstream row;
  row << result.cruise_id << ","
      << start_s << ","
      << end_s << ","
      << result.configured_modulation_freq_hz << ","
      << result.srate_peak_freq_hz << ","
      << result.drate_peak_freq_hz << ","
      << result.srtt_peak_freq_hz << ","
      << result.drate_freq_score << ","
      << result.srtt_freq_score << ","
      << result.freq_quality << ","
      << result.drate_target_amp << ","
      << result.srate_target_amp << ","
      << result.drate_gain << ","
      << result.drate_amplitude_score << ","
      << result.srtt_target_amp << ","
      << result.srtt_noise_floor << ","
      << result.srtt_snr << ","
      << result.srtt_amplitude_score << ","
      << result.drate_waveform_quality << ","
      << result.srtt_waveform_quality << ","
      << result.waveform_quality << ","
      << result.cycle_frequency_stability << ","
      << result.cycle_phase_stability << ","
      << result.consistency_quality << ","
      << result.srtt_top_clip_ratio << ","
      << result.srtt_bottom_clip_ratio << ","
      << result.srtt_distortion_score << ","
      << (result.is_full_load_candidate ? "true" : "false") << ","
      << result.full_load_quality << ","
	      << result.full_load_rank_in_cruise << ","
	      << (result.is_best_full_load_window ? "true" : "false") << ","
	      << (result.low_confidence ? "true" : "false") << ","
	      << result.label << ","
	      << result.full_load_quality_v1 << ","
	      << result.full_load_quality_v2 << ","
	      << result.drate_spectral_integrity_score << ","
	      << result.srtt_spectral_integrity_score << ","
	      << result.joint_spectral_integrity_score << ","
	      << (result.drate_spectral_gate_pass ? "true" : "false") << ","
	      << (result.srtt_spectral_gate_pass ? "true" : "false") << ","
	      << (result.dual_signal_spectral_gate_pass ? "true" : "false") << ","
	      << result.limiting_spectral_signal << ","
	      << result.spectral_invalid_reason << ","
	      << result.drate_snr << ","
	      << result.drate_band_energy_rel << ","
	      << result.srtt_band_energy_rel << ","
	      << result.drate_band_peak_rel << ","
	      << result.srtt_band_peak_rel << ","
	      << result.srate_peak_width_hz << ","
	      << result.drate_peak_width_hz << ","
	      << result.srtt_peak_width_hz << ","
	      << result.drate_width_ratio << ","
	      << result.srtt_width_ratio << ","
	      << result.drate_phase_coherence << ","
	      << result.srtt_phase_coherence << ","
	      << result.window_source;
  cruise_load_trace_cb_(start_s,
                        end_s,
                        0.0,
                        result.full_load_quality,
                        0.0,
                        result.freq_quality,
                        result.label,
                        result.low_confidence,
                        row.str());
}

void FBBRSender::EmitCruiseSummaryTrace(QuicTime now) const {
  if (!cruise_load_trace_cb_) {
    return;
  }

	  const CruiseWindowResult* best = nullptr;
	  int candidate_count = 0;
	  for (const CruiseWindowResult& result : current_cruise_windows_) {
	    if (!result.is_full_load_candidate) {
	      continue;
	    }
	    ++candidate_count;
	    if (!IsReliableSpectralWindow(result)) {
	      continue;
	    }
	    if (best == nullptr ||
	        result.full_load_quality_v2 > best->full_load_quality_v2) {
	      best = &result;
	    }
	  }

  const double cruise_start_s =
      cruise_start_time_ == QuicTime::Zero()
          ? -1.0
          : static_cast<double>((cruise_start_time_ - QuicTime::Zero())
                                    .ToMicroseconds()) /
                1000000.0;
  const double cruise_end_s =
      static_cast<double>((now - QuicTime::Zero()).ToMicroseconds()) /
      1000000.0;
  const double best_start_s =
      best == nullptr
          ? -1.0
          : static_cast<double>((best->window_start - QuicTime::Zero())
                                    .ToMicroseconds()) /
                1000000.0;
  const double best_end_s =
      best == nullptr
          ? -1.0
          : static_cast<double>((best->window_end - QuicTime::Zero())
                                    .ToMicroseconds()) /
                1000000.0;
  const uint64_t cruise_end_native_bw_kbps =
      BandwidthEstimate().ToKBitsPerSecond();
  const QuicBandwidth summary_selection_native_bw =
      !selection_native_bw_.IsZero() ? selection_native_bw_
                                     : BandwidthEstimate();
  const double fair_share_bandwidth_kbps =
      static_cast<double>(fair_share_bandwidth_bps_) / 1000.0;

  std::ostringstream row;
  row << cruise_id_ << ","
      << cruise_start_s << ","
      << cruise_end_s << ","
      << candidate_count << ","
      << (best != nullptr ? "true" : "false") << ","
      << best_start_s << ","
      << best_end_s << ","
      << (best != nullptr ? best->full_load_quality : 0.0) << ","
      << (best != nullptr ? best->drate_freq_score : 0.0) << ","
      << (best != nullptr ? best->srtt_freq_score : 0.0) << ","
      << (best != nullptr ? best->srtt_waveform_quality : 0.0) << ","
      << (best != nullptr ? best->drate_amplitude_score : 0.0) << ","
	      << (best != nullptr ? best->srtt_amplitude_score : 0.0) << ","
	      << (best != nullptr ? best->drate_mean_kbps : 0.0) << ","
	      << cruise_end_native_bw_kbps << ","
	      << fair_share_bandwidth_kbps << ","
	      << (trusted_bw_valid_ ? trusted_bw_.ToBitsPerSecond() : 0) << ","
	      << trusted_bw_source_ << ","
	      << (best != nullptr ? best->full_load_quality_v1 : 0.0) << ","
	      << (best != nullptr ? best->full_load_quality_v2 : 0.0) << ","
	      << (best != nullptr ? best->drate_spectral_integrity_score : 0.0) << ","
	      << (best != nullptr ? best->srtt_spectral_integrity_score : 0.0) << ","
	      << (best != nullptr ? best->joint_spectral_integrity_score : 0.0) << ","
	      << (best != nullptr && best->drate_spectral_gate_pass ? "true" : "false") << ","
	      << (best != nullptr && best->srtt_spectral_gate_pass ? "true" : "false") << ","
	      << (best != nullptr && best->dual_signal_spectral_gate_pass ? "true" : "false") << ","
	      << (best != nullptr ? best->limiting_spectral_signal
	                          : kLimitingSpectralSignalEqual) << ","
	      << (best != nullptr ? best->spectral_invalid_reason : "none") << ","
	      << summary_selection_native_bw.ToBitsPerSecond() << ","
	      << (trusted_bw_valid_ ? "true" : "false") << ","
	      << trusted_bw_cruise_id_ << ","
	      << (trusted_bw_fresh_ ? "true" : "false") << ","
	      << (trusted_bw_application_valid_ ? "true" : "false");
  cruise_load_trace_cb_(cruise_start_s,
                        cruise_end_s,
                        0.0,
                        best != nullptr ? best->full_load_quality : 0.0,
                        0.0,
                        0.0,
                        "CRUISE_SUMMARY",
                        best == nullptr,
                        row.str());
}

std::vector<FBBRRateSample> FBBRSender::SelectRateSamples(
    const std::deque<FBBRRateSample>& history,
    QuicTime start,
    QuicTime end) const {
  std::vector<FBBRRateSample> out;
  for (const auto& sample : history) {
    if (sample.time >= start && sample.time <= end) {
      out.push_back(sample);
    }
  }
  return out;
}

std::vector<FBBRRttSample> FBBRSender::SelectRttSamples(
    const std::deque<FBBRRttSample>& history,
    QuicTime start,
    QuicTime end) const {
  std::vector<FBBRRttSample> out;
  for (const auto& sample : history) {
    if (sample.time >= start && sample.time <= end) {
      out.push_back(sample);
    }
  }
  return out;
}

std::vector<double> FBBRSender::ResampleRateSeries(
    const std::vector<FBBRRateSample>& samples,
    QuicTime start,
    QuicTime end,
    double sample_step_s) const {
  std::vector<double> values;
  if (samples.size() < 2 || end <= start || sample_step_s <= 0.0) {
    return values;
  }
  const double duration_s =
      static_cast<double>((end - start).ToMicroseconds()) / 1000000.0;
  const int sample_count =
      std::max(0, static_cast<int>(duration_s / sample_step_s));
  if (sample_count < 8) {
    return values;
  }
  values.resize(sample_count);
  size_t idx = 0;
  for (int i = 0; i < sample_count; ++i) {
    QuicTime target = start + TimeDelta::FromMicroseconds(
                                  static_cast<int64_t>(i * sample_step_s *
                                                       1000000.0));
    while (idx + 1 < samples.size() && samples[idx + 1].time < target) {
      ++idx;
    }
    if (idx + 1 >= samples.size()) {
      values[i] = static_cast<double>(samples.back().rate.ToKBitsPerSecond());
      continue;
    }
    const double t1 =
        static_cast<double>((samples[idx].time - QuicTime::Zero())
                                .ToMicroseconds()) /
        1000000.0;
    const double t2 =
        static_cast<double>((samples[idx + 1].time - QuicTime::Zero())
                                .ToMicroseconds()) /
        1000000.0;
    const double tt =
        static_cast<double>((target - QuicTime::Zero()).ToMicroseconds()) /
        1000000.0;
    const double v1 =
        static_cast<double>(samples[idx].rate.ToKBitsPerSecond());
    const double v2 =
        static_cast<double>(samples[idx + 1].rate.ToKBitsPerSecond());
    double frac = 0.0;
    if (std::abs(t2 - t1) > 1e-12) {
      frac = Clamp01((tt - t1) / (t2 - t1));
    }
    values[i] = v1 + frac * (v2 - v1);
  }
  return values;
}

std::vector<double> FBBRSender::ResampleRttSeries(
    const std::vector<FBBRRttSample>& samples,
    QuicTime start,
    QuicTime end,
    double sample_step_s) const {
  std::vector<double> values;
  if (samples.size() < 2 || end <= start || sample_step_s <= 0.0) {
    return values;
  }
  const double duration_s =
      static_cast<double>((end - start).ToMicroseconds()) / 1000000.0;
  const int sample_count =
      std::max(0, static_cast<int>(duration_s / sample_step_s));
  if (sample_count < 8) {
    return values;
  }
  values.resize(sample_count);
  size_t idx = 0;
  for (int i = 0; i < sample_count; ++i) {
    QuicTime target = start + TimeDelta::FromMicroseconds(
                                  static_cast<int64_t>(i * sample_step_s *
                                                       1000000.0));
    while (idx + 1 < samples.size() && samples[idx + 1].time < target) {
      ++idx;
    }
    if (idx + 1 >= samples.size()) {
      values[i] = samples.back().rtt_ms;
      continue;
    }
    const double t1 =
        static_cast<double>((samples[idx].time - QuicTime::Zero())
                                .ToMicroseconds()) /
        1000000.0;
    const double t2 =
        static_cast<double>((samples[idx + 1].time - QuicTime::Zero())
                                .ToMicroseconds()) /
        1000000.0;
    const double tt =
        static_cast<double>((target - QuicTime::Zero()).ToMicroseconds()) /
        1000000.0;
    double frac = 0.0;
    if (std::abs(t2 - t1) > 1e-12) {
      frac = Clamp01((tt - t1) / (t2 - t1));
    }
    values[i] = samples[idx].rtt_ms +
                frac * (samples[idx + 1].rtt_ms - samples[idx].rtt_ms);
  }
  return values;
}

FBBRSender::WindowSignalResult FBBRSender::AnalyzeRateSeries(
    const std::vector<FBBRRateSample>& samples,
    QuicTime start,
    QuicTime end,
    double reference_freq_hz,
    bool detrend) const {
  WindowSignalResult result{
      {0.0, 0.0, 0.0, 0.0, 0.0, false, 0.0, 0.0, {}, false},
      0.0,
      {},
      false};
  std::vector<double> values =
      ResampleRateSeries(samples, start, end, kSampleStepSec);
  if (values.empty()) {
    return result;
  }
  result.mean_value =
      std::accumulate(values.begin(), values.end(), 0.0) / values.size();
  if (detrend && values.size() >= 2) {
    const double first = values.front();
    const double last = values.back();
    for (size_t i = 0; i < values.size(); ++i) {
      const double frac = static_cast<double>(i) /
                          static_cast<double>(values.size() - 1);
      values[i] -= first + frac * (last - first);
    }
  }
  result.profile = BuildSpectrumProfile(values, kSampleStepSec,
                                        reference_freq_hz);
  result.values = values;
  result.valid = result.profile.valid;
  return result;
}

FBBRSender::WindowSignalResult FBBRSender::AnalyzeRttSeries(
    const std::vector<FBBRRttSample>& samples,
    QuicTime start,
    QuicTime end,
    double reference_freq_hz,
    bool detrend) const {
  WindowSignalResult result{
      {0.0, 0.0, 0.0, 0.0, 0.0, false, 0.0, 0.0, {}, false},
      0.0,
      {},
      false};
  std::vector<double> values =
      ResampleRttSeries(samples, start, end, kSampleStepSec);
  if (values.empty()) {
    return result;
  }
  result.mean_value =
      std::accumulate(values.begin(), values.end(), 0.0) / values.size();
  if (detrend && values.size() >= 2) {
    const double first = values.front();
    const double last = values.back();
    for (size_t i = 0; i < values.size(); ++i) {
      const double frac = static_cast<double>(i) /
                          static_cast<double>(values.size() - 1);
      values[i] -= first + frac * (last - first);
    }
  }
  result.profile = BuildSpectrumProfile(values, kSampleStepSec,
                                        reference_freq_hz);
  result.values = values;
  result.valid = result.profile.valid;
  return result;
}

FBBRSender::SpectrumProfile FBBRSender::BuildSpectrumProfile(
    const std::vector<double>& values,
    double sample_step_s,
    double ref_freq_hz) const {
  SpectrumProfile profile{
      0.0, 0.0, 0.0, 0.0, 0.0, false, 0.0, 0.0, {}, false};
  if (values.size() < 8 || !std::isfinite(sample_step_s) ||
      sample_step_s <= 0.0 || !std::isfinite(ref_freq_hz) ||
      ref_freq_hz <= 0.0 ||
      ref_freq_hz >= 0.5 / sample_step_s ||
      static_cast<double>(values.size()) * sample_step_s * ref_freq_hz < 2.0 ||
      std::any_of(values.begin(), values.end(), [](double value) {
        return !std::isfinite(value);
      })) {
    return profile;
  }

  const int signal_len = static_cast<int>(values.size());
  const int nfft = std::max(signal_len, signal_len * kFftZeroPadMultiplier);
  double* in = static_cast<double*>(fftw_malloc(sizeof(double) * nfft));
  fftw_complex* out = static_cast<fftw_complex*>(
      fftw_malloc(sizeof(fftw_complex) * (nfft / 2 + 1)));
  if (in == nullptr || out == nullptr) {
    if (in != nullptr) fftw_free(in);
    if (out != nullptr) fftw_free(out);
    return profile;
  }

  const double mean =
      std::accumulate(values.begin(), values.end(), 0.0) / values.size();
  for (int i = 0; i < signal_len; ++i) {
    double hann = 1.0;
    if (signal_len > 1) {
      hann = 0.5 *
             (1.0 - std::cos(2.0 * M_PI * i / (signal_len - 1)));
    }
    in[i] = (values[i] - mean) * hann;
  }
  for (int i = signal_len; i < nfft; ++i) {
    in[i] = 0.0;
  }

  fftw_plan plan = fftw_plan_dft_r2c_1d(nfft, in, out, FFTW_ESTIMATE);
  if (plan == nullptr) {
    fftw_free(in);
    fftw_free(out);
    return profile;
  }
  fftw_execute(plan);

	  const double fs = 1.0 / sample_step_s;
	  const double freq_step = fs / nfft;
  profile.freq_step_hz = freq_step;
  const int k_min = 1;
  const int k_max = nfft / 2;
  std::vector<double> magnitudes(k_max + 1, 0.0);
  double total_energy = 0.0;
  for (int k = k_min; k <= k_max; ++k) {
    const double mag =
        std::sqrt(out[k][0] * out[k][0] + out[k][1] * out[k][1]);
    if (!std::isfinite(mag)) {
      fftw_destroy_plan(plan);
      fftw_free(in);
      fftw_free(out);
      return profile;
    }
    magnitudes[k] = mag;
    total_energy += mag;
  }

  const double band_low_hz = kBandLowRatio * ref_freq_hz;
  const double band_high_hz = kBandHighRatio * ref_freq_hz;
  int band_k_low =
      std::max(k_min, static_cast<int>(std::ceil(band_low_hz / freq_step)));
  int band_k_high =
      std::min(k_max, static_cast<int>(std::floor(band_high_hz / freq_step)));

  if (band_k_low <= band_k_high && total_energy > 0.0) {
    double band_energy = 0.0;
    double max_mag = -1.0;
    int peak_k = band_k_low;
    for (int k = band_k_low; k <= band_k_high; ++k) {
      band_energy += magnitudes[k];
      if (magnitudes[k] > max_mag) {
        max_mag = magnitudes[k];
        peak_k = k;
      }
    }
    profile.target_amp = max_mag;
    std::vector<double> noise_magnitudes;
    noise_magnitudes.reserve(k_max - k_min + 1);
    for (int k = k_min; k <= k_max; ++k) {
      if (k < band_k_low || k > band_k_high) {
        noise_magnitudes.push_back(magnitudes[k]);
      }
    }
    if (!noise_magnitudes.empty()) {
      const size_t mid = noise_magnitudes.size() / 2;
      std::nth_element(noise_magnitudes.begin(),
                       noise_magnitudes.begin() + mid,
                       noise_magnitudes.end());
      profile.noise_floor = noise_magnitudes[mid];
      profile.noise_floor_valid = std::isfinite(profile.noise_floor) &&
                                  profile.noise_floor >= 0.0;
    }
	    profile.peak_freq_hz = peak_k * freq_step;
    profile.peak_width_hz = band_high_hz - band_low_hz;
    if (max_mag > 0.0) {
      const double half_peak = 0.5 * max_mag;
      int left_k = peak_k;
      int right_k = peak_k;
      bool found_left = false;
      bool found_right = false;
      for (int k = peak_k; k >= band_k_low; --k) {
        if (magnitudes[k] < half_peak) {
          left_k = k;
          found_left = true;
          break;
        }
      }
      for (int k = peak_k; k <= band_k_high; ++k) {
        if (magnitudes[k] < half_peak) {
          right_k = k;
          found_right = true;
          break;
        }
      }
      if (found_left && found_right && right_k > left_k) {
        profile.peak_width_hz =
            std::max(freq_step, (right_k - left_k) * freq_step);
      }
    }
    const double window_duration_s =
        static_cast<double>(values.size()) * sample_step_s;
    profile.peak_width_hz = std::max(
        profile.peak_width_hz,
        std::max(freq_step, 1.0 / std::max(window_duration_s, 1e-9)));
	    if (peak_k > k_min && peak_k < k_max) {
      const double left_mag = magnitudes[peak_k - 1];
      const double center_mag = magnitudes[peak_k];
      const double right_mag = magnitudes[peak_k + 1];
      const double denom = left_mag - 2.0 * center_mag + right_mag;
      if (std::abs(denom) > 1e-12) {
        double offset = 0.5 * (left_mag - right_mag) / denom;
        offset = std::max(-1.0, std::min(1.0, offset));
        profile.peak_freq_hz = (peak_k + offset) * freq_step;
      }
    }
    profile.band_energy_rel = band_energy / total_energy;
    profile.band_peak_rel = max_mag / total_energy;
    profile.band_shape.assign(kBandShapeBins, 0.0);
    for (int i = 0; i < kBandShapeBins; ++i) {
      const double target_freq =
          band_low_hz +
          (band_high_hz - band_low_hz) * static_cast<double>(i) /
              static_cast<double>(kBandShapeBins - 1);
      const double raw_index = target_freq / freq_step;
      int left_k = static_cast<int>(std::floor(raw_index));
      left_k = std::max(k_min, std::min(k_max, left_k));
      int right_k = std::max(k_min, std::min(k_max, left_k + 1));
      double frac = raw_index - left_k;
      if (right_k == left_k) {
        frac = 0.0;
      }
      profile.band_shape[i] =
          magnitudes[left_k] * (1.0 - frac) + magnitudes[right_k] * frac;
    }
    const double shape_sum =
        std::accumulate(profile.band_shape.begin(),
                        profile.band_shape.end(),
                        0.0);
    if (shape_sum > 0.0) {
      for (double& value : profile.band_shape) {
        value /= shape_sum;
      }
      profile.valid = true;
    } else {
      profile.band_shape.clear();
    }
  }

  fftw_destroy_plan(plan);
  fftw_free(in);
  fftw_free(out);
  return profile;
}

double FBBRSender::ComputeSpectrumShapeDistance(
    const std::vector<double>& lhs,
    const std::vector<double>& rhs) const {
  if (lhs.empty() || rhs.empty() || lhs.size() != rhs.size()) {
    return std::numeric_limits<double>::infinity();
  }
  double distance = 0.0;
  for (size_t i = 0; i < lhs.size(); ++i) {
    distance += std::abs(lhs[i] - rhs[i]);
  }
  return distance;
}

double FBBRSender::ComputePhaseCoherence(
    const std::vector<double>& values,
    double sample_step_s,
    double ref_freq_hz,
    bool* valid) const {
  if (valid != nullptr) {
    *valid = false;
  }
  if (values.size() < 8 || sample_step_s <= 0.0 || ref_freq_hz <= 0.0) {
    return 0.0;
  }
  const int samples_per_cycle =
      static_cast<int>(std::round((1.0 / ref_freq_hz) / sample_step_s));
  if (samples_per_cycle < 4) {
    return 0.0;
  }
  const int cycle_count =
      static_cast<int>(values.size()) / samples_per_cycle;
  if (cycle_count < 2) {
    return 0.0;
  }

  double sum_real = 0.0;
  double sum_imag = 0.0;
  double sum_abs = 0.0;
  int valid_cycles = 0;
  for (int cycle = 0; cycle < cycle_count; ++cycle) {
    const int begin = cycle * samples_per_cycle;
    const int end = begin + samples_per_cycle;
    double cycle_mean = 0.0;
    for (int i = begin; i < end; ++i) {
      cycle_mean += values[i];
    }
    cycle_mean /= static_cast<double>(samples_per_cycle);

    double real = 0.0;
    double imag = 0.0;
    for (int i = begin; i < end; ++i) {
      const double t = static_cast<double>(i) * sample_step_s;
      const double x = values[i] - cycle_mean;
      const double phase = -2.0 * M_PI * ref_freq_hz * t;
      real += x * std::cos(phase);
      imag += x * std::sin(phase);
    }
    const double amp = std::sqrt(real * real + imag * imag);
    if (amp <= 1e-12) {
      continue;
    }
    sum_real += real;
    sum_imag += imag;
    sum_abs += amp;
    ++valid_cycles;
  }

  if (valid_cycles < 2 || sum_abs <= 1e-12) {
    return 0.0;
  }
  if (valid != nullptr) {
    *valid = true;
  }
  return Clamp01(std::sqrt(sum_real * sum_real + sum_imag * sum_imag) /
                 (sum_abs + 1e-12));
}

FBBRSender::CycleQualityMetrics FBBRSender::AnalyzeCycleQuality(
    const std::vector<double>& values,
    double sample_step_s,
    double ref_freq_hz,
    bool estimate_waveform_distortion) const {
  CycleQualityMetrics metrics{0.5, 0.0, 0.0, 0.5, 0.5, 0.5, 0, false};
  if (values.size() < 8 || sample_step_s <= 0.0 || ref_freq_hz <= 0.0) {
    return metrics;
  }

  const int samples_per_cycle =
      static_cast<int>(std::round((1.0 / ref_freq_hz) / sample_step_s));
  if (samples_per_cycle < 4) {
    return metrics;
  }
  const int cycle_count =
      static_cast<int>(values.size()) / samples_per_cycle;
  if (cycle_count <= 0) {
    return metrics;
  }

  std::vector<double> phase_offsets;
  std::vector<int> peak_indices;
  double top_clip_sum = 0.0;
  double bottom_clip_sum = 0.0;
  double incompleteness_sum = 0.0;
  double asymmetry_sum = 0.0;

  for (int cycle = 0; cycle < cycle_count; ++cycle) {
    const int begin = cycle * samples_per_cycle;
    const int end = begin + samples_per_cycle;
    auto minmax = std::minmax_element(values.begin() + begin,
                                      values.begin() + end);
    const double cycle_min = *minmax.first;
    const double cycle_max = *minmax.second;
    const double amplitude = cycle_max - cycle_min;
    if (amplitude <= 1e-12) {
      incompleteness_sum += 1.0;
      continue;
    }

    ++metrics.valid_cycles;
    const int peak_index =
        static_cast<int>(std::distance(values.begin() + begin, minmax.second));
    peak_indices.push_back(peak_index);
    phase_offsets.push_back(
        static_cast<double>(peak_index) /
        static_cast<double>(samples_per_cycle));

    const double high_threshold = cycle_max - 0.10 * amplitude;
    const double low_threshold = cycle_min + 0.10 * amplitude;
    const double slope_threshold = 0.02 * amplitude;
    int top_clip = 0;
    int bottom_clip = 0;
    int positive_slope = 0;
    int negative_slope = 0;
    for (int i = begin + 1; i < end; ++i) {
      const double slope = values[i] - values[i - 1];
      if (slope > slope_threshold) {
        ++positive_slope;
      }
      if (slope < -slope_threshold) {
        ++negative_slope;
      }
      if (values[i] >= high_threshold && std::abs(slope) <= slope_threshold) {
        ++top_clip;
      }
      if (values[i] <= low_threshold && std::abs(slope) <= slope_threshold) {
        ++bottom_clip;
      }
    }
    top_clip_sum += static_cast<double>(top_clip) /
                    static_cast<double>(samples_per_cycle);
    bottom_clip_sum += static_cast<double>(bottom_clip) /
                       static_cast<double>(samples_per_cycle);
    double cycle_incompleteness = 0.0;
    if (positive_slope <= 0) {
      cycle_incompleteness += 0.5;
    }
    if (negative_slope <= 0) {
      cycle_incompleteness += 0.5;
    }
    incompleteness_sum += Clamp01(cycle_incompleteness);

    const int half = samples_per_cycle / 2;
    double first_half_delta = 0.0;
    double second_half_delta = 0.0;
    for (int i = begin + 1; i < begin + half; ++i) {
      first_half_delta += std::abs(values[i] - values[i - 1]);
    }
    for (int i = begin + half + 1; i < end; ++i) {
      second_half_delta += std::abs(values[i] - values[i - 1]);
    }
    const double denom = first_half_delta + second_half_delta + 1e-12;
    asymmetry_sum +=
        std::abs(first_half_delta - second_half_delta) / denom;
  }

  if (metrics.valid_cycles <= 0) {
    return metrics;
  }

  const double valid_cycles_d = static_cast<double>(metrics.valid_cycles);
  metrics.top_clip_ratio = top_clip_sum / valid_cycles_d;
  metrics.bottom_clip_ratio = bottom_clip_sum / valid_cycles_d;
  const double cycle_incompleteness = incompleteness_sum / valid_cycles_d;
  const double cycle_asymmetry = asymmetry_sum / valid_cycles_d;

  double phase_instability = 0.5;
  if (phase_offsets.size() >= 2) {
    const double mean_phase =
        std::accumulate(phase_offsets.begin(), phase_offsets.end(), 0.0) /
        phase_offsets.size();
    double variance = 0.0;
    for (double phase : phase_offsets) {
      double diff = std::abs(phase - mean_phase);
      diff = std::min(diff, 1.0 - diff);
      variance += diff * diff;
    }
    const double phase_std =
        std::sqrt(variance / static_cast<double>(phase_offsets.size()));
    metrics.cycle_phase_stability =
        Clamp01(1.0 - phase_std / kMaxPhaseStdCycles);
    phase_instability = 1.0 - metrics.cycle_phase_stability;
    metrics.phase_reliable = true;
  }

  if (peak_indices.size() >= 2) {
    double freq_score_sum = 0.0;
    int freq_score_count = 0;
    for (size_t i = 1; i < peak_indices.size(); ++i) {
      const int delta_samples =
          samples_per_cycle +
          peak_indices[i] - peak_indices[i - 1];
      if (delta_samples <= 0) {
        continue;
      }
      const double estimated_freq =
          1.0 / (static_cast<double>(delta_samples) * sample_step_s);
      freq_score_sum += ComputeFreqScore(
          estimated_freq, ref_freq_hz, 0.20 * ref_freq_hz);
      ++freq_score_count;
    }
    if (freq_score_count > 0) {
      metrics.cycle_frequency_stability =
          freq_score_sum / static_cast<double>(freq_score_count);
    }
  } else if (metrics.valid_cycles >= 2) {
    metrics.cycle_frequency_stability = 1.0;
  }

  if (estimate_waveform_distortion) {
    metrics.distortion_score =
        Clamp01(0.25 * metrics.top_clip_ratio +
                0.25 * metrics.bottom_clip_ratio +
                0.20 * cycle_incompleteness +
                0.15 * cycle_asymmetry +
                0.15 * phase_instability);
    metrics.waveform_quality = Clamp01(1.0 - metrics.distortion_score);
  } else {
    metrics.distortion_score = 0.5;
    metrics.waveform_quality = 0.5;
  }

  return metrics;
}

double FBBRSender::ComputeFreqScore(double peak_freq_hz,
                                        double reference_freq_hz,
                                        double freq_tolerance_hz) const {
  if (reference_freq_hz <= 0.0 || freq_tolerance_hz <= 0.0 ||
      peak_freq_hz <= 0.0) {
    return 0.0;
  }
  return Clamp01(1.0 - std::abs(peak_freq_hz - reference_freq_hz) /
                           freq_tolerance_hz);
}

double FBBRSender::ExpFreqScore(double delta_f, double sigma_f) {
  if (sigma_f <= 0.0) {
    return 0.0;
  }
  const double z = delta_f / sigma_f;
  return Clamp01(std::exp(-0.5 * z * z));
}

double FBBRSender::LogisticScore(double value,
                                     double threshold,
                                     double slope) {
  if (slope <= 0.0) {
    return value >= threshold ? 1.0 : 0.0;
  }
  const double x = ClampValue(slope * (value - threshold), -60.0, 60.0);
  return Clamp01(1.0 / (1.0 + std::exp(-x)));
}

double FBBRSender::WidthScore(double width_ratio,
                                  double r0,
                                  double sigma) {
  if (sigma <= 0.0 || !std::isfinite(width_ratio)) {
    return 0.0;
  }
  const double excess = std::max(0.0, width_ratio - r0);
  const double z = excess / sigma;
  return Clamp01(std::exp(-0.5 * z * z));
}

double FBBRSender::ComputeCongestionScore(QuicTime window_start,
                                              QuicTime window_end) const {
  QuicByteCount acked_bytes = 0;
  bool has_loss = false;
  for (const auto& sample : ack_window_history_) {
    if (sample.time >= window_start && sample.time <= window_end) {
      acked_bytes += sample.acked_bytes;
      has_loss = has_loss || sample.has_loss;
    }
  }
  double ecn_score = 0.0;
  const QuicByteCount ecn_bytes = GetBytesEcnInRounds();
  if (acked_bytes > 0 && default_ecn_congestion_ratio_ > 0.0) {
    const double ecn_ratio =
        static_cast<double>(ecn_bytes) / static_cast<double>(acked_bytes);
    ecn_score = Clamp01(ecn_ratio / default_ecn_congestion_ratio_);
  } else if (ecn_bytes > 0) {
    ecn_score = 1.0;
  }
  const double loss_score = has_loss ? 1.0 : 0.0;
  return std::max(ecn_score, loss_score);
}

double FBBRSender::Clamp01(double value) {
  if (!std::isfinite(value)) return 0.0;
  if (value < 0.0) return 0.0;
  if (value > 1.0) return 1.0;
  return value;
}

double FBBRSender::ScoreThreshold(double value,
                                      double min_value,
                                      double target) {
  if (target <= min_value) {
    return value >= target ? 1.0 : 0.0;
  }
  return Clamp01((value - min_value) / (target - min_value));
}

const char* FBBRSender::LabelToString(int label) {
  return label == 1 ? "FULL_LOAD_CANDIDATE"
                    : "NOT_FULL_LOAD_CANDIDATE";
}

}  // namespace dqc
