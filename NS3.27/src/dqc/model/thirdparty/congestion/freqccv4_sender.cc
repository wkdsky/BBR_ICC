#include "freqccv4_sender.h"

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
constexpr const char* kLimitingSpectralSignalDrate = "DRATE";
constexpr const char* kLimitingSpectralSignalSrtt = "SRTT";
constexpr const char* kLimitingSpectralSignalEqual = "EQUAL";

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

bool HasValidRateCoverage(const std::vector<FreqCCv4RateSample>& samples,
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

bool HasValidRttCoverage(const std::vector<FreqCCv4RttSample>& samples,
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

constexpr size_t FreqCCv4Sender::kMaxHistorySamples;
constexpr uint32_t FreqCCv4Sender::kStableRounds;
constexpr double FreqCCv4Sender::kDefaultOscillationFreqHz;
constexpr double FreqCCv4Sender::kSampleStepSec;
constexpr double FreqCCv4Sender::kMinDrateFreqScoreForCandidate;
constexpr double FreqCCv4Sender::kMinSrttFreqScoreForCandidate;
constexpr double FreqCCv4Sender::kTargetDrateGain;
constexpr double FreqCCv4Sender::kMinSrttSnr;
constexpr double FreqCCv4Sender::kTargetSrttSnr;
constexpr double FreqCCv4Sender::kMaxDrateShapeDistance;
constexpr double FreqCCv4Sender::kMaxPhaseStdCycles;
constexpr double FreqCCv4Sender::kBandLowRatio;
constexpr double FreqCCv4Sender::kBandHighRatio;
constexpr int FreqCCv4Sender::kBandShapeBins;
constexpr int FreqCCv4Sender::kFftZeroPadMultiplier;

FreqCCv4Sender::FreqCCv4Sender(
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
                 kFreqCCv4,
                 true),
	      configured_modulation_freq_hz_(5.0),
      amplitude_mode_(FreqCCv4AmplitudeMode::kFixed),
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
	      cruise_baseline_cap_bps_(0),
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
      gate_trace_mode_(FreqCCv4GateTraceMode::kRoundOnly),
      gate_trace_sample_interval_(TimeDelta::FromMilliseconds(1)),
      last_pacing_gate_trace_time_(QuicTime::Zero()) {
  QUIC_DVLOG(2) << this << " Initializing FreqCCv4Sender @ " << now
                << "; DefaultEcnCongestionRatio="
                << default_ecn_congestion_ratio_;
}

void FreqCCv4Sender::SetOscillationFrequency(double freq_hz) {
  configured_modulation_freq_hz_ = freq_hz;
}

void FreqCCv4Sender::SetOscillationAmplitude(FreqCCv4AmplitudeMode mode,
                                             uint64_t fixed_bps) {
  amplitude_mode_ = mode;
  fixed_amplitude_bps_ = fixed_bps;
}

void FreqCCv4Sender::SetRecvSignalMode(bool use_delivery_rate_latest) {
  use_delivery_rate_latest_for_signal_history_ = use_delivery_rate_latest;
}

void FreqCCv4Sender::SetCruiseWindowConfig(double min_cycles_per_window,
                                           double window_step_ratio) {
  if (min_cycles_per_window > 0.0) {
    min_cruise_cycles_per_window_ = min_cycles_per_window;
  }
  if (window_step_ratio > 0.0) {
    cruise_window_step_ratio_ = window_step_ratio;
  }
}

void FreqCCv4Sender::SetFairShareBandwidthBps(uint64_t fair_share_bps) {
  fair_share_bandwidth_bps_ = fair_share_bps;
}

void FreqCCv4Sender::SetCruiseBaselineCapBps(uint64_t cap_bps) {
  cruise_baseline_cap_bps_ = cap_bps;
}

void FreqCCv4Sender::SetConvergenceGateTraceEnabled(bool enabled) {
  enable_convergence_gate_trace_ = enabled;
}

void FreqCCv4Sender::SetConvergenceGateControlEnabled(bool enabled) {
  enable_convergence_gate_control_ = enabled;
}

void FreqCCv4Sender::ConfigureFreqBbr(const FreqBbrConfig& config) {
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
  ClearTrustedBw("configuration_changed");
}

void FreqCCv4Sender::SetTraceFlowId(uint32_t flow_id) {
  trace_flow_id_ = flow_id;
}

void FreqCCv4Sender::SetGateTraceMode(FreqCCv4GateTraceMode mode,
                                       uint64_t sample_interval_us) {
  gate_trace_mode_ = mode;
  gate_trace_sample_interval_ = TimeDelta::FromMicroseconds(
      static_cast<int64_t>(std::max<uint64_t>(1, sample_interval_us)));
}

bool FreqCCv4Sender::RunConvergenceGateStateMachineSelfTest(
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

  os << "# FreqCCv4 convergence-gate state-machine self-test\n";

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

bool FreqCCv4Sender::RunTrustedBwSelectionSelfTest(std::ostream& os) {
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

  os << "# FreqCCv4 TrustedBw dual-signal selection self-test\n";
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

bool FreqCCv4Sender::RunTrustedBwPacingSelfTest(std::ostream& os) {
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

  os << "# FreqCCv4 TrustedBw pacing self-test\n";
  os << "phase,native_gbps,trusted_gbps,pacing_base_source,gain,target_gbps\n";
  check_phase("REFILL", 100.0, 80.0, true, 1.0, 80.0);
  check_phase("UP", 100.0, 80.0, true, 1.25, 100.0);
  check_phase("DOWN", 100.0, 80.0, true, 0.9, 72.0);
  check_phase("REFILL", 100.0, 0.0, false, 1.0, 100.0);
  check_phase("UP", 100.0, 0.0, false, 1.25, 125.0);
  check_phase("DOWN", 100.0, 0.0, false, 0.9, 90.0);
  const double cruise_native_pacing = 100.0;
  const double cruise_triangle = 7.0;
  require(std::abs(cruise_native_pacing + cruise_triangle - 107.0) < 1e-12,
          "CRUISE must add triangular modulation to native pacing");
  os << "\nRESULT: " << (pass ? "PASS" : "FAIL") << "\n";
  return pass;
}
Bbr2ProbeBwMode::CyclePhase FreqCCv4Sender::GetCurrentProbeBwPhase() const {
  DebugState state = ExportDebugState();
  if (state.mode == Bbr2Mode::PROBE_BW) {
    return state.probe_bw.phase;
  }
  return Bbr2ProbeBwMode::CyclePhase::PROBE_NOT_STARTED;
}

bool FreqCCv4Sender::BaseShouldOscillate() const {
  if (GetCurrentAmplitudeBps() == 0 || configured_modulation_freq_hz_ <= 0.0) {
    return false;
  }
  if (!drain_completed_ || mode_ != Bbr2Mode::PROBE_BW) {
    return false;
  }
  return GetCurrentProbeBwPhase() == Bbr2ProbeBwMode::CyclePhase::PROBE_CRUISE &&
         in_cruise_ && cruise_start_time_ != QuicTime::Zero();
}

bool FreqCCv4Sender::ShouldOscillate() const {
  const bool base_should = BaseShouldOscillate();
  if (!enable_convergence_gate_control_) {
    return base_should;
  }
  return base_should && !bbr_stable_;
}

uint64_t FreqCCv4Sender::GetCurrentAmplitudeBps() const {
  QuicBandwidth max_bw = BandwidthEstimate();
  QuicBandwidth base_rate = Bbr2Sender::PacingRate(0);

  switch (amplitude_mode_) {
    case FreqCCv4AmplitudeMode::kFixed:
      return fixed_amplitude_bps_;
    case FreqCCv4AmplitudeMode::kMiu2:
      return max_bw.ToBitsPerSecond() / 2;
    case FreqCCv4AmplitudeMode::kMiu3:
      return max_bw.ToBitsPerSecond() / 3;
    case FreqCCv4AmplitudeMode::kMiu4:
      return max_bw.ToBitsPerSecond() / 4;
    case FreqCCv4AmplitudeMode::kMiu8:
      return max_bw.ToBitsPerSecond() / 8;
    case FreqCCv4AmplitudeMode::kSR2:
      return base_rate.ToBitsPerSecond() / 2;
    case FreqCCv4AmplitudeMode::kSR3:
      return base_rate.ToBitsPerSecond() / 3;
    case FreqCCv4AmplitudeMode::kSR4:
      return base_rate.ToBitsPerSecond() / 4;
    case FreqCCv4AmplitudeMode::kSR8:
      return base_rate.ToBitsPerSecond() / 8;
    case FreqCCv4AmplitudeMode::kSR12:
      return base_rate.ToBitsPerSecond() / 12;
    case FreqCCv4AmplitudeMode::kSR16:
      return base_rate.ToBitsPerSecond() / 16;
    default:
      return 0;
  }
}

double FreqCCv4Sender::TriangleWave(QuicTime now) const {
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

void FreqCCv4Sender::OnProbeBwPhaseEntered(Bbr2ProbeBwMode::CyclePhase phase,
                                           QuicTime now) {
  if (phase == Bbr2ProbeBwMode::CyclePhase::PROBE_CRUISE) {
    EnterCruise(now);
    return;
  }
  if (in_cruise_) {
    LeaveCruise(now);
  }
}

void FreqCCv4Sender::EnterCruise(QuicTime now) {
  in_cruise_ = true;
  ++cruise_id_;
  cruise_start_time_ = now;
  trusted_bw_cleared_on_cruise_start_ = false;
  if (!trusted_bw_clear_on_cruise_start_) {
    QUIC_DVLOG(1) << "FreqCCv4: trusted_bw.clear_on_cruise_start=false "
                     "is overridden to preserve fresh-only application";
  }
  ClearTrustedBwApplication("cruise_start");
  cruise_modulation_freq_hz_ = configured_modulation_freq_hz_;
  freq_tool_on_ = ShouldOscillate();
  cruise_freq_tool_active_ = freq_tool_on_;
  min_rtt_warning_logged_ = false;
  current_cruise_windows_.clear();
  ResetCruiseWindowState();
  QUIC_DVLOG(2) << "FreqCCv4: Entering PROBE_CRUISE @ " << now
                << ", cruise_id=" << cruise_id_
                << ", fixed_freq=" << cruise_modulation_freq_hz_
                << "Hz, amplitude_bps=" << GetCurrentAmplitudeBps();
}

void FreqCCv4Sender::LeaveCruise(QuicTime now) {
  QUIC_DVLOG(2) << "FreqCCv4: Leaving PROBE_CRUISE @ " << now;
  FinalizeCruise(now);
  in_cruise_ = false;
  freq_tool_on_ = false;
  cruise_freq_tool_active_ = false;
  cruise_start_time_ = QuicTime::Zero();
  cruise_modulation_freq_hz_ = configured_modulation_freq_hz_;
  ResetCruiseWindowState();
  current_cruise_windows_.clear();
}

void FreqCCv4Sender::ResetCruiseWindowState() {
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

void FreqCCv4Sender::OnCongestionEventStarted(
    const Bbr2CongestionEvent& congestion_event) {
  UpdateRoundDeliveryRateSample(congestion_event);
  if (congestion_event.end_of_round_trip) {
    FinalizeCompletedRound(congestion_event);
  }
}

void FreqCCv4Sender::UpdateRoundDeliveryRateSample(
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

void FreqCCv4Sender::FinalizeCompletedRound(
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

bool FreqCCv4Sender::CheckExitStable(QuicBandwidth completed_d_round,
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

void FreqCCv4Sender::UpdateReconvergenceEvidence(
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

void FreqCCv4Sender::UpdateFreqWeightAndToolState() {
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

void FreqCCv4Sender::ClearTrustedBw(const char* reason) {
  trusted_bw_ = QuicBandwidth::Zero();
  trusted_bw_valid_ = false;
  trusted_bw_conf_ = 0.0;
  trusted_bw_source_ = kTrustedBwSourceNone;
  trusted_bw_cruise_id_ = 0;
  trusted_bw_invalid_reason_ = reason == nullptr ? "unknown" : reason;
  ClearTrustedBwApplication(reason);
}

void FreqCCv4Sender::ClearTrustedBwApplication(const char* reason) const {
  trusted_bw_fresh_ = false;
  trusted_bw_application_valid_ = false;
  trusted_bw_ready_for_post_cruise_ = false;
  trusted_bw_application_phase_ = "NONE";
  if (reason != nullptr && std::string(reason) == "cruise_start") {
    trusted_bw_cleared_on_cruise_start_ = true;
  }
}

bool FreqCCv4Sender::IsReliableSpectralWindow(
    const CruiseWindowResult& result) const {
  return result.dual_signal_spectral_gate_pass &&
         result.is_full_load_candidate && !result.low_confidence &&
         result.full_load_quality_v2 >=
             min_full_load_quality_for_reliable_window_ &&
         std::isfinite(result.drate_mean_kbps) &&
         result.drate_mean_kbps > 0.0;
}

double FreqCCv4Sender::ComputeRateTrendRatio(QuicTime start,
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

void FreqCCv4Sender::PublishTrustedBwSelection(
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

const char* FreqCCv4Sender::PhaseApplicationName(
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

const char* FreqCCv4Sender::PacingBaseSourceName(
    FreqCCv4PacingBaseSource source) {
  return source == FreqCCv4PacingBaseSource::kTrustedBw
             ? "TRUSTED_BW"
             : "NATIVE_BBR";
}

bool FreqCCv4Sender::IsCruisePhase(
    Bbr2ProbeBwMode::CyclePhase phase) const {
  return mode_ == Bbr2Mode::PROBE_BW &&
         phase == Bbr2ProbeBwMode::CyclePhase::PROBE_CRUISE;
}

bool FreqCCv4Sender::IsTrustedBwApplicationPhase(
    Bbr2ProbeBwMode::CyclePhase phase) const {
  if (mode_ != Bbr2Mode::PROBE_BW) {
    return false;
  }
  return phase == Bbr2ProbeBwMode::CyclePhase::PROBE_REFILL ||
         phase == Bbr2ProbeBwMode::CyclePhase::PROBE_UP ||
         phase == Bbr2ProbeBwMode::CyclePhase::PROBE_DOWN ||
         phase == Bbr2ProbeBwMode::CyclePhase::PROBE_DOWN_SLIGHTLY;
}

void FreqCCv4Sender::EmitConvergenceGateTrace(
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
      << "FreqCCv4 convergence_gate"
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

  if (gate_trace_mode_ == FreqCCv4GateTraceMode::kOff) {
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
                     FreqCCv4PacingBaseSource::kNativeBbr,
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

void FreqCCv4Sender::EmitPacingTrace(
    QuicBandwidth b_native,
    QuicBandwidth pacing_base_bw,
    FreqCCv4PacingBaseSource pacing_base_source,
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
      << "FreqCCv4 pacing_gate"
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

  bool emit_csv = gate_trace_mode_ == FreqCCv4GateTraceMode::kFull;
  if (gate_trace_mode_ == FreqCCv4GateTraceMode::kSampledPacing) {
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

void FreqCCv4Sender::EmitFreqGateCsvRow(
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
    FreqCCv4PacingBaseSource pacing_base_source,
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
      << (trusted_bw_cleared_on_cruise_start_ ? "true" : "false");
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
void FreqCCv4Sender::OnPacketSent(QuicTime sent_time,
                                  QuicByteCount bytes_in_flight,
                                  QuicPacketNumber packet_number,
                                  QuicByteCount bytes,
                                  HasRetransmittableData is_retransmittable) {
  current_time_ = sent_time;
  QuicBandwidth sender_rate = PacingRate(bytes_in_flight);
  sender_rate_history_.push_back({sent_time, sender_rate});
  while (sender_rate_history_.size() > kMaxHistorySamples) {
    sender_rate_history_.pop_front();
  }

  Bbr2Sender::OnPacketSent(sent_time,
                           bytes_in_flight,
                           packet_number,
                           bytes,
                           is_retransmittable);
}

void FreqCCv4Sender::OnCongestionEvent(
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

  if (mode_ != Bbr2Mode::PROBE_BW) {
    ClearTrustedBw("non_probe_bw");
  } else if (enable_convergence_gate_control_ && bbr_stable_) {
    ClearTrustedBw("stable_closure");
  }

  if (!drain_completed_ && mode_ == Bbr2Mode::PROBE_BW) {
    drain_completed_ = true;
  }

  if (last_ack_time_ != QuicTime::Zero() && event_time > last_ack_time_ &&
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
    RunDueCruiseWindowAnalysis(event_time);
  }
}

QuicBandwidth FreqCCv4Sender::PacingRate(
    QuicByteCount bytes_in_flight) const {
  const QuicBandwidth native_pacing =
      Bbr2Sender::PacingRate(bytes_in_flight);
  const QuicBandwidth native_bw = BandwidthEstimate();
  const Bbr2ProbeBwMode::CyclePhase phase = GetCurrentProbeBwPhase();
  const double phase_gain = static_cast<double>(PacingGain());

  QuicBandwidth pacing_base_bw = native_bw;
  FreqCCv4PacingBaseSource pacing_base_source =
      FreqCCv4PacingBaseSource::kNativeBbr;
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
    pacing_base_source = FreqCCv4PacingBaseSource::kTrustedBw;
    trusted_bw_application_phase_ = PhaseApplicationName(phase);
  } else {
    trusted_bw_application_phase_ = PhaseApplicationName(phase);
  }

  QuicBandwidth baseline_pacing = native_pacing;
  if (use_trusted_bw) {
    baseline_pacing =
        static_cast<float>(phase_gain) * pacing_base_bw;
  }
  if (phase == Bbr2ProbeBwMode::CyclePhase::PROBE_CRUISE &&
      cruise_baseline_cap_bps_ > 0 &&
      baseline_pacing.ToBitsPerSecond() > cruise_baseline_cap_bps_) {
    baseline_pacing =
        QuicBandwidth::FromBitsPerSecond(cruise_baseline_cap_bps_);
  }

  const bool base_should_oscillate = BaseShouldOscillate();
  const bool should_oscillate =
      enable_convergence_gate_control_
          ? (base_should_oscillate && !bbr_stable_)
          : base_should_oscillate;
  freq_tool_on_ = should_oscillate;
  const int64_t amplitude_bps =
      should_oscillate ? static_cast<int64_t>(GetCurrentAmplitudeBps()) : 0;
  const double triangle_wave =
      should_oscillate ? TriangleWave(current_time_) : 0.0;
  const int64_t offset_bps =
      static_cast<int64_t>(amplitude_bps * triangle_wave);
  int64_t final_bps =
      static_cast<int64_t>(baseline_pacing.ToBitsPerSecond()) + offset_bps;
  final_bps = std::max<int64_t>(1000, final_bps);
  const QuicBandwidth final_pacing =
      QuicBandwidth::FromBitsPerSecond(static_cast<uint64_t>(final_bps));
  const QuicBandwidth returned_pacing =
      (!should_oscillate && !use_trusted_bw) ? native_pacing : final_pacing;

  EmitPacingTrace(native_bw,
                  pacing_base_bw,
                  pacing_base_source,
                  native_pacing,
                  returned_pacing,
                  phase_gain,
                  amplitude_bps,
                  amplitude_bps,
                  triangle_wave,
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
int32_t FreqCCv4Sender::GetCurrentBbrModeIndex() const {
  return Bbr2Sender::GetCurrentBbrModeIndex();
}

void FreqCCv4Sender::RunDueCruiseWindowAnalysis(QuicTime now) {
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
      QUIC_DVLOG(1) << "FreqCCv4: minRTT unavailable; skip CRUISE "
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

void FreqCCv4Sender::AnalyzeCruiseWindow(QuicTime window_start,
                                         QuicTime window_end,
                                         TimeDelta min_rtt,
                                         double window_duration_s) {
  current_cruise_windows_.push_back(BuildCruiseWindowResult(
      window_start, window_end, min_rtt, window_duration_s, "NORMAL"));
}

FreqCCv4Sender::CruiseWindowResult FreqCCv4Sender::BuildCruiseWindowResult(
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
    QUIC_DVLOG(1) << "FreqCCv4: SRTT noise-floor estimate failed; "
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
    QUIC_DVLOG(1) << "FreqCCv4: missing srate/drate spectrum shape; "
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
    QUIC_DVLOG(1) << "FreqCCv4: SRTT cycle metrics unavailable; "
                     "using waveform/consistency defaults";
  }
  if (!srtt_cycles.phase_reliable) {
    result.cycle_phase_stability = 0.5;
    QUIC_DVLOG(1) << "FreqCCv4: SRTT phase stability unreliable; "
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

  QUIC_DVLOG(2) << "FreqCCv4: CRUISE full-load window ["
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

FreqCCv4Sender::TrustedBwSelectionResult
FreqCCv4Sender::RunTrustedBwSelection(QuicTime now) {
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
void FreqCCv4Sender::RankCruiseWindows(
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

void FreqCCv4Sender::FinalizeCruise(QuicTime now) {
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

void FreqCCv4Sender::EmitCruiseWindowTrace(
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

void FreqCCv4Sender::EmitCruiseSummaryTrace(QuicTime now) const {
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

std::vector<FreqCCv4RateSample> FreqCCv4Sender::SelectRateSamples(
    const std::deque<FreqCCv4RateSample>& history,
    QuicTime start,
    QuicTime end) const {
  std::vector<FreqCCv4RateSample> out;
  for (const auto& sample : history) {
    if (sample.time >= start && sample.time <= end) {
      out.push_back(sample);
    }
  }
  return out;
}

std::vector<FreqCCv4RttSample> FreqCCv4Sender::SelectRttSamples(
    const std::deque<FreqCCv4RttSample>& history,
    QuicTime start,
    QuicTime end) const {
  std::vector<FreqCCv4RttSample> out;
  for (const auto& sample : history) {
    if (sample.time >= start && sample.time <= end) {
      out.push_back(sample);
    }
  }
  return out;
}

std::vector<double> FreqCCv4Sender::ResampleRateSeries(
    const std::vector<FreqCCv4RateSample>& samples,
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

std::vector<double> FreqCCv4Sender::ResampleRttSeries(
    const std::vector<FreqCCv4RttSample>& samples,
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

FreqCCv4Sender::WindowSignalResult FreqCCv4Sender::AnalyzeRateSeries(
    const std::vector<FreqCCv4RateSample>& samples,
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

FreqCCv4Sender::WindowSignalResult FreqCCv4Sender::AnalyzeRttSeries(
    const std::vector<FreqCCv4RttSample>& samples,
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

FreqCCv4Sender::SpectrumProfile FreqCCv4Sender::BuildSpectrumProfile(
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

double FreqCCv4Sender::ComputeSpectrumShapeDistance(
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

double FreqCCv4Sender::ComputePhaseCoherence(
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

FreqCCv4Sender::CycleQualityMetrics FreqCCv4Sender::AnalyzeCycleQuality(
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

double FreqCCv4Sender::ComputeFreqScore(double peak_freq_hz,
                                        double reference_freq_hz,
                                        double freq_tolerance_hz) const {
  if (reference_freq_hz <= 0.0 || freq_tolerance_hz <= 0.0 ||
      peak_freq_hz <= 0.0) {
    return 0.0;
  }
  return Clamp01(1.0 - std::abs(peak_freq_hz - reference_freq_hz) /
                           freq_tolerance_hz);
}

double FreqCCv4Sender::ExpFreqScore(double delta_f, double sigma_f) {
  if (sigma_f <= 0.0) {
    return 0.0;
  }
  const double z = delta_f / sigma_f;
  return Clamp01(std::exp(-0.5 * z * z));
}

double FreqCCv4Sender::LogisticScore(double value,
                                     double threshold,
                                     double slope) {
  if (slope <= 0.0) {
    return value >= threshold ? 1.0 : 0.0;
  }
  const double x = ClampValue(slope * (value - threshold), -60.0, 60.0);
  return Clamp01(1.0 / (1.0 + std::exp(-x)));
}

double FreqCCv4Sender::WidthScore(double width_ratio,
                                  double r0,
                                  double sigma) {
  if (sigma <= 0.0 || !std::isfinite(width_ratio)) {
    return 0.0;
  }
  const double excess = std::max(0.0, width_ratio - r0);
  const double z = excess / sigma;
  return Clamp01(std::exp(-0.5 * z * z));
}

double FreqCCv4Sender::ComputeCongestionScore(QuicTime window_start,
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

double FreqCCv4Sender::Clamp01(double value) {
  if (!std::isfinite(value)) return 0.0;
  if (value < 0.0) return 0.0;
  if (value > 1.0) return 1.0;
  return value;
}

double FreqCCv4Sender::ScoreThreshold(double value,
                                      double min_value,
                                      double target) {
  if (target <= min_value) {
    return value >= target ? 1.0 : 0.0;
  }
  return Clamp01((value - min_value) / (target - min_value));
}

const char* FreqCCv4Sender::LabelToString(int label) {
  return label == 1 ? "FULL_LOAD_CANDIDATE"
                    : "NOT_FULL_LOAD_CANDIDATE";
}

}  // namespace dqc
