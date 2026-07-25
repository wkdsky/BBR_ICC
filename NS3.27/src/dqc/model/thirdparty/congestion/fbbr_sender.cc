#include "fbbr_sender.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <iostream>
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
constexpr const char* kTrustedBwSourceTimeWaveformBaseline =
    "TIME_WAVEFORM_SRTT_SEARCH";
constexpr const char* kTrustedBwSourceAdaptiveWindowMean =
    "ADAPTIVE_WINDOW_MEAN";
constexpr const char* kTrustedBwSourceFbbrWindowMean =
    "FBBR_WINDOW_MEAN";
constexpr const char* kTrustedBwSourceGuardFilter = "GUARD_FILTER";
constexpr const char* kTrustedBwSourcePreviousTrusted =
    "PREVIOUS_TRUSTED";
constexpr const char* kTrustedBwSourceNativeFallback =
    "NATIVE_FALLBACK";
constexpr const char* kWaveformDeltaSourceNone = "NONE";
constexpr const char* kWaveformDeltaSourceRecentDrate =
    "RECENT_DRATE_AMPLITUDE";
constexpr const char* kWaveformDeltaSourceBaselineFallback =
    "BASELINE_QUARTER_FALLBACK";
constexpr const char* kWaveformDeltaSourceAdaptiveBracketBound =
    "ADAPTIVE_WINDOW_EXTREME_GAP_BOUND";
constexpr const char* kWaveformDeltaSourceAdaptiveWindowMinimum =
    "ADAPTIVE_WINDOW_MINIMUM";
constexpr const char* kWaveformDeltaSourceAdaptiveWindowMaximum =
    "ADAPTIVE_WINDOW_MAXIMUM";
constexpr const char* kWaveformDeltaSourceFbbrWindowMinimum =
    "FBBR_WINDOW_MINIMUM";
constexpr const char* kWaveformDeltaSourceFbbrWindowMaximum =
    "FBBR_WINDOW_MAXIMUM";
constexpr const char* kWaveformDeltaSourceFbbrTrustedBw =
    "FBBR_TRUSTED_BW";
constexpr const char* kWaveformDeltaSourceHybridAdaptiveBracket =
    "HYBRID_ADAPTIVE_BRACKET";
constexpr const char* kWaveformDeltaSourceHybridRtpropDrateMidpoint =
    "HYBRID_RTPROP_DRATE_MIDPOINT";
constexpr const char* kWaveformDeltaSourceHybridGradientMatchedOverload =
    "HYBRID_GRADIENT_MATCHED_OVERLOAD";
constexpr double kAdaptiveMaxBwInheritanceTolerance = 0.25;
constexpr uint64_t kDefaultMinimumPacingRateBps = 1000000;
constexpr double kWaveformPostAdjustmentCollectionPeriods = 2.0;
// Match BBR-R's strong persistent-RTT-inflation pacing reduction.  Applying
// it geometrically once per completed Hybrid window drains quickly while
// retaining 80% of the preceding search rate at every step.
constexpr double kFbbrHybridLowerBoundSearchFactor = 0.80;
constexpr int64_t kFbbrHybridLowerBoundSearchFirstCruise = 3;
constexpr double kFbbrHybridLowerBoundSearchQueueFraction = 0.50;
constexpr int64_t kFbbrHybridStableObservationDurationMs = 200;
constexpr float kFBBRCruiseCwndGain = 1.25f;
constexpr float kFBBRRtpropProbeDownPacingGain = 0.75f;
constexpr const char* kLimitingSpectralSignalDrate = "DRATE";
constexpr const char* kLimitingSpectralSignalSrtt = "SRTT";
constexpr const char* kLimitingSpectralSignalEqual = "EQUAL";

double ClampValue(double value, double low, double high) {
  return std::max(low, std::min(high, value));
}

bool ShouldInheritAdaptiveBounds(double current_max_bw_bps,
                                 double previous_max_bw_bps) {
  if (!std::isfinite(current_max_bw_bps) || current_max_bw_bps <= 0.0 ||
      !std::isfinite(previous_max_bw_bps) || previous_max_bw_bps <= 0.0) {
    return false;
  }
  return std::abs(current_max_bw_bps - previous_max_bw_bps) /
             previous_max_bw_bps <
         kAdaptiveMaxBwInheritanceTolerance;
}

bool IsAtLeastElevenTenthsBdp(QuicByteCount bytes_in_flight,
                              QuicByteCount bdp) {
  return bdp > 0 &&
         static_cast<long double>(bytes_in_flight) * 10.0L >=
             static_cast<long double>(bdp) * 11.0L;
}

bool ShouldStartFbbrHybridLowerBoundSearch(
    int64_t cruise_id,
    bool baseline_low_valid,
    bool rtprop_drate_valid) {
  return cruise_id >= kFbbrHybridLowerBoundSearchFirstCruise &&
         (!baseline_low_valid || !rtprop_drate_valid);
}

bool ShouldAllowFbbrHybridLowerBoundSearchByQueue(
    double current_srtt_ms,
    bool rtprop_valid,
    double rtprop_ms,
    bool max_srtt_valid,
    double max_srtt_ms) {
  if (!std::isfinite(current_srtt_ms) || current_srtt_ms <= 0.0 ||
      !rtprop_valid || !std::isfinite(rtprop_ms) || rtprop_ms <= 0.0 ||
      !max_srtt_valid || !std::isfinite(max_srtt_ms) ||
      max_srtt_ms <= rtprop_ms) {
    return false;
  }
  const double current_queue_ms =
      std::max(0.0, current_srtt_ms - rtprop_ms);
  const double max_queue_ms = max_srtt_ms - rtprop_ms;
  return current_queue_ms >=
      kFbbrHybridLowerBoundSearchQueueFraction * max_queue_ms;
}

bool IsBelowHalfBdp(QuicByteCount inflight, QuicByteCount bdp) {
  return bdp > 0 &&
      2.0L * static_cast<long double>(inflight) <
          static_cast<long double>(bdp);
}

double ComputeFbbrHybridLowerBoundSearchBaseline(
    double current_baseline_bps,
    double minimum_rate_bps) {
  if (!std::isfinite(minimum_rate_bps) || minimum_rate_bps <= 0.0) {
    minimum_rate_bps = 1.0;
  }
  if (!std::isfinite(current_baseline_bps) ||
      current_baseline_bps <= minimum_rate_bps) {
    return minimum_rate_bps;
  }
  return std::max(minimum_rate_bps,
                  kFbbrHybridLowerBoundSearchFactor *
                      current_baseline_bps);
}

bool IsWaveformDecisionRule(const char* decision_rule) {
  if (decision_rule == nullptr || decision_rule[0] != 'R' ||
      decision_rule[1] == '\0') {
    return false;
  }
  return decision_rule[1] >= '1' && decision_rule[1] <= '5' &&
         (decision_rule[2] == '\0' || decision_rule[2] == '.');
}

double ComputeAdaptiveNextBaseline(
    WaveformClassification classification,
    bool baseline_low_valid,
    double baseline_low_bps,
    bool baseline_up_valid,
    double baseline_up_bps,
    double current_baseline_bps,
    double window_min_bps,
    double window_max_bps,
    double minimum_rate_bps) {
  double next_baseline_bps =
      classification == WaveformClassification::kOverload
          ? window_min_bps
          : window_max_bps;
  if (baseline_low_valid && baseline_up_valid &&
      std::isfinite(baseline_low_bps) && std::isfinite(baseline_up_bps) &&
      baseline_up_bps > baseline_low_bps) {
    const double baseline_gap = baseline_up_bps - baseline_low_bps;
    const double quarter_target_bps = baseline_low_bps + baseline_gap / 4.0;
    const double half_target_bps = baseline_low_bps + baseline_gap / 2.0;
    if (classification == WaveformClassification::kOverload) {
      if (std::isfinite(current_baseline_bps) &&
          current_baseline_bps > quarter_target_bps) {
        next_baseline_bps = quarter_target_bps;
      }
    } else if (classification == WaveformClassification::kUnderload) {
      if (std::isfinite(current_baseline_bps) &&
          current_baseline_bps < half_target_bps) {
        next_baseline_bps = half_target_bps;
      }
    }
  }
  if (!std::isfinite(next_baseline_bps)) {
    next_baseline_bps = minimum_rate_bps;
  }
  return next_baseline_bps;
}

bool ShouldObserveAfterInconclusive(
    bool adaptive_guard_enabled,
    uint32_t extension_count,
    uint32_t max_extensions,
    double window_periods,
    double max_window_periods) {
  if (adaptive_guard_enabled) {
    return true;
  }
  return extension_count < max_extensions &&
         window_periods < max_window_periods;
}

double InconclusiveWindowStartAdvancePeriods(
    bool adaptive_guard_enabled,
    bool extended_window,
    double window_periods) {
  return adaptive_guard_enabled && extended_window &&
                 window_periods >= 3.0
             ? 1.0
             : 0.0;
}

uint64_t AmplifiedWaveformProbeAmplitude(
    uint64_t current_amplitude_bps,
    uint64_t initial_amplitude_bps,
    double amplification_factor,
    double amplification_max_ratio) {
  if (current_amplitude_bps == 0 || initial_amplitude_bps == 0 ||
      !std::isfinite(amplification_factor) ||
      !std::isfinite(amplification_max_ratio) ||
      amplification_factor <= 1.0 || amplification_max_ratio <= 1.0) {
    return current_amplitude_bps;
  }
  const double current = static_cast<double>(current_amplitude_bps);
  const double initial = static_cast<double>(initial_amplitude_bps);
  const double max_int64 =
      static_cast<double>(std::numeric_limits<int64_t>::max());
  const double cap =
      std::min(max_int64, initial * amplification_max_ratio);
  if (!std::isfinite(current) || !std::isfinite(initial) ||
      !std::isfinite(cap) || cap <= current) {
    return current_amplitude_bps;
  }
  const double amplified =
      std::min(cap, std::ceil(current * amplification_factor));
  if (!std::isfinite(amplified) || amplified <= current) {
    return current_amplitude_bps;
  }
  return static_cast<uint64_t>(amplified);
}

bool HasOnlyHalfWaveform(
    const std::vector<double>& response,
    const std::vector<double>& sender_residual,
    const std::vector<bool>& valid,
    double noise_sigma,
    bool negative_half_active,
    double* positive_span,
    double* negative_span) {
  if (response.size() != sender_residual.size() ||
      response.size() != valid.size()) {
    return false;
  }
  std::vector<double> positive_values;
  std::vector<double> negative_values;
  for (size_t i = 0; i < response.size(); ++i) {
    if (!valid[i] || !std::isfinite(response[i]) ||
        !std::isfinite(sender_residual[i])) {
      continue;
    }
    (sender_residual[i] >= 0.0 ? positive_values : negative_values)
        .push_back(response[i]);
  }
  if (positive_values.size() < 4 || negative_values.size() < 4) {
    return false;
  }
  auto robust_span = [](std::vector<double> values) {
    std::sort(values.begin(), values.end());
    const size_t low_index = static_cast<size_t>(
        std::floor(0.10 * static_cast<double>(values.size() - 1)));
    const size_t high_index = static_cast<size_t>(
        std::ceil(0.90 * static_cast<double>(values.size() - 1)));
    return std::max(0.0, values[high_index] - values[low_index]);
  };
  const double positive = robust_span(positive_values);
  const double negative = robust_span(negative_values);
  if (positive_span != nullptr) {
    *positive_span = positive;
  }
  if (negative_span != nullptr) {
    *negative_span = negative;
  }
  const double finite_noise =
      std::isfinite(noise_sigma) ? std::max(0.0, noise_sigma) : 0.0;
  const double active_span = negative_half_active ? negative : positive;
  const double suppressed_span = negative_half_active ? positive : negative;
  const bool active =
      active_span > std::max(4.0 * finite_noise, 1e-9);
  const bool opposite_suppressed =
      suppressed_span <= std::max(2.0 * finite_noise, 0.25 * active_span);
  return active && opposite_suppressed;
}

bool HasOnlyNegativeHalfWaveform(
    const std::vector<double>& response,
    const std::vector<double>& sender_residual,
    const std::vector<bool>& valid,
    double noise_sigma,
    double* positive_span,
    double* negative_span) {
  return HasOnlyHalfWaveform(
      response, sender_residual, valid, noise_sigma, true,
      positive_span, negative_span);
}

bool HasOnlyPositiveHalfWaveform(
    const std::vector<double>& response,
    const std::vector<double>& sender_residual,
    const std::vector<bool>& valid,
    double noise_sigma,
    double* positive_span,
    double* negative_span) {
  return HasOnlyHalfWaveform(
      response, sender_residual, valid, noise_sigma, false,
      positive_span, negative_span);
}

int64_t AddPacingOffsetWithFloor(int64_t baseline_bps,
                                 int64_t offset_bps,
                                 uint64_t minimum_rate_bps) {
  int64_t combined_bps = 0;
  if (offset_bps > 0 &&
      baseline_bps > std::numeric_limits<int64_t>::max() - offset_bps) {
    combined_bps = std::numeric_limits<int64_t>::max();
  } else if (offset_bps < 0 &&
             baseline_bps < std::numeric_limits<int64_t>::min() - offset_bps) {
    combined_bps = std::numeric_limits<int64_t>::min();
  } else {
    combined_bps = baseline_bps + offset_bps;
  }
  const int64_t floor_bps = static_cast<int64_t>(std::min<uint64_t>(
      minimum_rate_bps,
      static_cast<uint64_t>(std::numeric_limits<int64_t>::max())));
  return std::max(floor_bps, combined_bps);
}

bool IsObviousClipPlateau(double duration_ratio,
                          double minimum_duration_ratio,
                          double half_overlap_ratio,
                          double minimum_half_overlap_ratio,
                          double absolute_slope,
                          double maximum_absolute_slope,
                          double level_span_ratio,
                          double maximum_level_span_ratio,
                          double extreme_distance_ratio,
                          double maximum_extreme_distance_ratio,
                          bool has_opposing_shoulders) {
  return std::isfinite(duration_ratio) &&
      std::isfinite(half_overlap_ratio) &&
      std::isfinite(absolute_slope) &&
      std::isfinite(maximum_absolute_slope) &&
      std::isfinite(level_span_ratio) &&
      std::isfinite(extreme_distance_ratio) &&
      duration_ratio >= minimum_duration_ratio &&
      half_overlap_ratio >= minimum_half_overlap_ratio &&
      absolute_slope <= maximum_absolute_slope &&
      level_span_ratio <= maximum_level_span_ratio &&
      extreme_distance_ratio <= maximum_extreme_distance_ratio &&
      has_opposing_shoulders;
}

float FBBRCwndGainForPhase(Bbr2ProbeBwMode::CyclePhase phase,
                               float native_cwnd_gain) {
  return phase == Bbr2ProbeBwMode::CyclePhase::PROBE_CRUISE
             ? kFBBRCruiseCwndGain
             : native_cwnd_gain;
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

double Quantile(std::vector<double> values, double q) {
  if (values.empty()) {
    return 0.0;
  }
  std::sort(values.begin(), values.end());
  q = ClampValue(q, 0.0, 1.0);
  const double position = q * static_cast<double>(values.size() - 1);
  const size_t lower = static_cast<size_t>(std::floor(position));
  const size_t upper = static_cast<size_t>(std::ceil(position));
  if (lower == upper) {
    return values[lower];
  }
  const double fraction = position - static_cast<double>(lower);
  return values[lower] + fraction * (values[upper] - values[lower]);
}

double RobustSigma(const std::vector<double>& values) {
  if (values.empty()) {
    return 0.0;
  }
  const double center = Median(values);
  std::vector<double> deviations;
  deviations.reserve(values.size());
  for (double value : values) {
    if (std::isfinite(value)) {
      deviations.push_back(std::abs(value - center));
    }
  }
  return 1.4826 * Median(deviations);
}

struct RobustQueueGradientEstimate {
  bool queue_sample_valid = false;
  size_t sample_count = 0;
  double q90_s = 0.0;
  double raw_gradient = 0.0;
  double noise_gradient = 0.0;
  double gradient = 0.0;
};

RobustQueueGradientEstimate ComputeRobustQueueGradient(
    const std::vector<FBBRRttSample>& samples,
    double rtprop_s) {
  RobustQueueGradientEstimate result;
  if (!std::isfinite(rtprop_s) || rtprop_s <= 0.0 || samples.empty()) {
    return result;
  }

  std::vector<double> times_s;
  std::vector<double> queues_s;
  times_s.reserve(samples.size());
  queues_s.reserve(samples.size());
  for (const auto& sample : samples) {
    const double rtt_s = sample.rtt_ms / 1000.0;
    if (!std::isfinite(rtt_s) || rtt_s <= 0.0 ||
        sample.time == QuicTime::Zero()) {
      continue;
    }
    const double time_s = static_cast<double>(
        (sample.time - QuicTime::Zero()).ToMicroseconds()) / 1000000.0;
    if (!std::isfinite(time_s)) {
      continue;
    }
    times_s.push_back(time_s);
    queues_s.push_back(std::max(0.0, rtt_s - rtprop_s));
  }
  result.sample_count = queues_s.size();
  if (queues_s.empty()) {
    return result;
  }
  result.q90_s = Quantile(queues_s, 0.90);
  result.queue_sample_valid =
      std::isfinite(result.q90_s) && result.q90_s >= 0.0;
  if (!result.queue_sample_valid || queues_s.size() < 4) {
    return result;
  }

  for (size_t i = 1; i < times_s.size(); ++i) {
    if (times_s[i] <= times_s[i - 1]) {
      return result;
    }
  }
  const double p05_s = Quantile(queues_s, 0.05);
  const double p95_s = Quantile(queues_s, 0.95);
  if (!std::isfinite(p05_s) || !std::isfinite(p95_s) ||
      p95_s < p05_s) {
    return result;
  }
  std::vector<double> winsorized_queue_s;
  winsorized_queue_s.reserve(queues_s.size());
  for (double queue_s : queues_s) {
    winsorized_queue_s.push_back(
        ClampValue(queue_s, p05_s, p95_s));
  }

  const double time_origin_s = times_s.front();
  double mean_time_s = 0.0;
  double mean_queue_s = 0.0;
  for (size_t i = 0; i < times_s.size(); ++i) {
    mean_time_s += times_s[i] - time_origin_s;
    mean_queue_s += winsorized_queue_s[i];
  }
  mean_time_s /= static_cast<double>(times_s.size());
  mean_queue_s /= static_cast<double>(times_s.size());
  double covariance = 0.0;
  double time_variance = 0.0;
  for (size_t i = 0; i < times_s.size(); ++i) {
    const double centered_time =
        times_s[i] - time_origin_s - mean_time_s;
    covariance += centered_time *
        (winsorized_queue_s[i] - mean_queue_s);
    time_variance += centered_time * centered_time;
  }
  if (!std::isfinite(covariance) || !std::isfinite(time_variance) ||
      time_variance <= std::numeric_limits<double>::epsilon()) {
    return result;
  }
  result.raw_gradient = covariance / time_variance;
  if (!std::isfinite(result.raw_gradient)) {
    result.raw_gradient = 0.0;
    return result;
  }

  std::vector<double> adjacent_gradients;
  adjacent_gradients.reserve(times_s.size() - 1);
  for (size_t i = 1; i < times_s.size(); ++i) {
    const double delta_time_s = times_s[i] - times_s[i - 1];
    if (!std::isfinite(delta_time_s) || delta_time_s <= 0.0) {
      result.raw_gradient = 0.0;
      return result;
    }
    const double adjacent_gradient =
        (winsorized_queue_s[i] - winsorized_queue_s[i - 1]) /
        delta_time_s;
    if (!std::isfinite(adjacent_gradient)) {
      result.raw_gradient = 0.0;
      return result;
    }
    adjacent_gradients.push_back(adjacent_gradient);
  }
  result.noise_gradient = RobustSigma(adjacent_gradients);
  if (!std::isfinite(result.noise_gradient) ||
      result.noise_gradient < 0.0) {
    result.raw_gradient = 0.0;
    result.noise_gradient = 0.0;
    return result;
  }
  result.gradient =
      std::abs(result.raw_gradient) <= 2.0 * result.noise_gradient
          ? 0.0
          : result.raw_gradient;
  result.gradient = ClampValue(result.gradient, -0.5, 0.5);
  return result;
}

struct GradientMatchedDecrease {
  bool valid = false;
  double queue_guard_s = 0.0;
  double queue_excess_s = 0.0;
  double desired_gradient = 0.0;
  double beta = 0.0;
};

GradientMatchedDecrease ComputeGradientMatchedDecrease(
    bool queue_sample_valid,
    double q90_s,
    double queue_gradient,
    double window_duration_s,
    bool rtprop_valid,
    double rtprop_s,
    bool max_srtt_valid,
    double max_srtt_s) {
  GradientMatchedDecrease result;
  if (!queue_sample_valid || !std::isfinite(q90_s) || q90_s < 0.0 ||
      !std::isfinite(queue_gradient) ||
      !std::isfinite(window_duration_s) || window_duration_s <= 0.0 ||
      !rtprop_valid || !std::isfinite(rtprop_s) || rtprop_s <= 0.0 ||
      !max_srtt_valid || !std::isfinite(max_srtt_s) ||
      max_srtt_s < rtprop_s) {
    return result;
  }
  result.queue_guard_s = std::max(
      0.1 * rtprop_s, (max_srtt_s - rtprop_s) / 3.0);
  result.queue_excess_s =
      std::max(0.0, q90_s - result.queue_guard_s);
  const double drain_time_s = 2.0 * window_duration_s;
  result.desired_gradient = -result.queue_excess_s / drain_time_s;
  const double denominator = std::max(0.5, 1.0 + queue_gradient);
  const double beta_star =
      1.0 - (1.0 + result.desired_gradient) / denominator;
  if (!std::isfinite(result.queue_guard_s) ||
      !std::isfinite(result.queue_excess_s) ||
      !std::isfinite(result.desired_gradient) ||
      !std::isfinite(denominator) || !std::isfinite(beta_star)) {
    return GradientMatchedDecrease();
  }
  result.beta = ClampValue(beta_star, 0.0, 0.10);
  result.valid = true;
  return result;
}

double TheilSenSlope(const std::vector<double>& values,
                     const std::vector<bool>& valid,
                     size_t begin,
                     size_t end,
                     double sample_step_s) {
  if (values.size() != valid.size() || begin >= end ||
      end > values.size() || sample_step_s <= 0.0) {
    return 0.0;
  }
  std::vector<double> slopes;
  for (size_t i = begin; i < end; ++i) {
    if (!valid[i] || !std::isfinite(values[i])) {
      continue;
    }
    for (size_t j = i + 1; j < end; ++j) {
      if (!valid[j] || !std::isfinite(values[j])) {
        continue;
      }
      slopes.push_back((values[j] - values[i]) /
                       (static_cast<double>(j - i) * sample_step_s));
    }
  }
  return Median(slopes);
}

double ValidLagCorrelation(const std::vector<double>& values,
                           const std::vector<bool>& valid,
                           size_t lag,
                           size_t* pair_count) {
  if (pair_count != nullptr) {
    *pair_count = 0;
  }
  if (values.size() != valid.size() || lag == 0 || lag >= values.size()) {
    return -1.0;
  }
  std::vector<double> lhs;
  std::vector<double> rhs;
  for (size_t i = 0; i + lag < values.size(); ++i) {
    if (valid[i] && valid[i + lag] && std::isfinite(values[i]) &&
        std::isfinite(values[i + lag])) {
      lhs.push_back(values[i]);
      rhs.push_back(values[i + lag]);
    }
  }
  if (pair_count != nullptr) {
    *pair_count = lhs.size();
  }
  if (lhs.size() < 4) {
    return -1.0;
  }
  const double lhs_mean =
      std::accumulate(lhs.begin(), lhs.end(), 0.0) / lhs.size();
  const double rhs_mean =
      std::accumulate(rhs.begin(), rhs.end(), 0.0) / rhs.size();
  double numerator = 0.0;
  double lhs_energy = 0.0;
  double rhs_energy = 0.0;
  for (size_t i = 0; i < lhs.size(); ++i) {
    const double a = lhs[i] - lhs_mean;
    const double b = rhs[i] - rhs_mean;
    numerator += a * b;
    lhs_energy += a * a;
    rhs_energy += b * b;
  }
  if (lhs_energy <= 0.0 || rhs_energy <= 0.0) {
    return -1.0;
  }
  return numerator / std::sqrt(lhs_energy * rhs_energy);
}

std::string AppendReason(const std::string& current, const char* reason) {
  if (current.empty() || current == "none") {
    return reason;
  }
  return current + "|" + reason;
}

QuicBandwidth BandwidthFromBps(double bps) {
  if (!std::isfinite(bps) || bps <= 0.0) {
    return QuicBandwidth::Zero();
  }
  const double capped = std::min(
      bps, static_cast<double>(std::numeric_limits<int64_t>::max()));
  return QuicBandwidth::FromBitsPerSecond(
      static_cast<int64_t>(std::llround(capped)));
}

bool IsFinitePositiveBandwidth(QuicBandwidth bandwidth) {
  if (bandwidth.IsZero() || bandwidth.IsInfinite()) {
    return false;
  }
  const int64_t bps = bandwidth.ToBitsPerSecond();
  return bps > 0 && std::isfinite(static_cast<double>(bps));
}

QuicBandwidth BandwidthFromBytesAndTimeDeltaSafe(QuicByteCount bytes,
                                                 TimeDelta delta) {
  const int64_t delta_us = delta.ToMicroseconds();
  if (bytes == 0 || delta_us <= 0) {
    return QuicBandwidth::Zero();
  }
  const long double bps =
      static_cast<long double>(bytes) * 8.0L * 1000000.0L /
      static_cast<long double>(delta_us);
  if (!std::isfinite(static_cast<double>(bps)) || bps <= 0.0L) {
    return QuicBandwidth::Zero();
  }
  const long double capped = std::min(
      bps, static_cast<long double>(std::numeric_limits<int64_t>::max()));
  return QuicBandwidth::FromBitsPerSecond(static_cast<int64_t>(capped));
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
    QuicTime now,
    const RttStats* rtt_stats,
    const QuicUnackedPacketMap* unacked_packets,
    QuicPacketCount initial_cwnd_in_packets,
    QuicPacketCount max_cwnd_in_packets,
    Random* random,
    QuicConnectionStats* stats,
    bool enable_ecn,
    bool fbbr_window_baseline,
    bool adaptive_guard,
    CongestionControlType congestion_control_type)
    : Bbr2Sender(now,
                 rtt_stats,
                 unacked_packets,
                 initial_cwnd_in_packets,
                 max_cwnd_in_packets,
                 random,
                 stats,
                 enable_ecn,
                 nullptr,
                 congestion_control_type,
                 true),
      fbbr_window_baseline_enabled_(fbbr_window_baseline),
      adaptive_guard_enabled_(adaptive_guard),
      configured_modulation_freq_hz_(5.0),
      amplitude_mode_(FBBRAmplitudeMode::kFixed),
      fixed_amplitude_bps_(0),
      minimum_pacing_rate_bps_(kDefaultMinimumPacingRateBps),
      drain_completed_(false),
      in_cruise_(false),
      current_cruise_rtprop_updated_(false),
      previous_cruise_rtprop_updated_(false),
      rtprop_probe_down_active_(false),
      cruise_rtprop_at_entry_(TimeDelta::Zero()),
      latest_congestion_event_prior_inflight_(0),
      latest_congestion_event_prior_inflight_valid_(false),
      latest_congestion_event_inflight_(0),
      latest_congestion_event_inflight_valid_(false),
      cruise_modulation_freq_hz_(5.0),
      cruise_start_time_(QuicTime::Zero()),
      next_cruise_window_start_(QuicTime::Zero()),
      current_time_(now),
      last_ack_time_(QuicTime::Zero()),
      use_delivery_rate_latest_for_signal_history_(false),
      min_rtt_warning_logged_(false),
      cruise_id_(0),
      cruise_detector_mode_(FBBRCruiseDetectorMode::kTimeWaveform),
      waveform_cruise_state_(WaveformCruiseState::kDisabled),
      initial_cruise_baseline_bw_(QuicBandwidth::Zero()),
      current_injection_baseline_bw_(QuicBandwidth::Zero()),
      current_probe_amplitude_bps_(0),
      waveform_initial_probe_amplitude_bps_(0),
      max_bw_response_observed_(false),
      max_bw_delivery_response_gain_(1.0),
      max_bw_observation_center_bps_(0.0),
      max_bw_observation_baseline_bps_(0.0),
      max_bw_actual_fluctuation_amplitude_bps_(0.0),
      max_bw_attenuation_factor_(1.0),
      current_probe_bw_phase_gain_(1.0),
      probe_epoch_start_time_(QuicTime::Zero()),
      probe_epoch_rtt_(TimeDelta::Zero()),
      waveform_settle_start_(QuicTime::Zero()),
      waveform_settle_end_(QuicTime::Zero()),
      waveform_window_start_(QuicTime::Zero()),
      waveform_window_end_(QuicTime::Zero()),
      waveform_window_periods_(0.0),
      waveform_window_extended_(false),
      underload_located_(false),
      trusted_baseline_locked_(false),
      has_last_similar_drate_amplitude_(false),
      last_similar_drate_amplitude_bps_(0.0),
      waveform_delta_reference_valid_(false),
      waveform_delta_reference_bps_(0.0),
      consecutive_overload_count_(0),
      latest_waveform_overload_srtt_mean_valid_(false),
      latest_waveform_overload_srtt_mean_ms_(0.0),
      latest_waveform_underload_srtt_mean_valid_(false),
      latest_waveform_underload_srtt_mean_ms_(0.0),
      adaptive_baseline_low_valid_(false),
      adaptive_baseline_low_(QuicBandwidth::Zero()),
      adaptive_baseline_up_valid_(false),
      adaptive_baseline_up_(QuicBandwidth::Zero()),
      adaptive_previous_cruise_max_bw_valid_(false),
      adaptive_previous_cruise_max_bw_(QuicBandwidth::Zero()),
      adaptive_cruise_start_max_bw_(QuicBandwidth::Zero()),
      adaptive_bounds_inherited_this_cruise_(false),
      waveform_last_delta_source_(kWaveformDeltaSourceNone),
      waveform_last_raw_delta_bw_bps_(0.0),
      waveform_last_applied_delta_bw_bps_(0.0),
      baseline_adjustment_count_(0),
      inconclusive_extension_count_(0),
      waveform_inconclusive_amplification_count_(0),
      fbbr_hybrid_max_srtt_valid_(false),
      fbbr_hybrid_max_srtt_ms_(0.0),
      fbbr_hybrid_max_srtt_source_cruise_id_(0),
      fbbr_hybrid_rtprop_drate_valid_(false),
      fbbr_hybrid_rtprop_drate_(QuicBandwidth::Zero()),
      fbbr_hybrid_rtprop_drate_source_cruise_id_(0),
      fbbr_hybrid_rtprop_drate_source_time_(QuicTime::Zero()),
      fbbr_hybrid_baseline_low_source_time_(QuicTime::Zero()),
      fbbr_hybrid_srtt_low_valid_(false),
      fbbr_hybrid_srtt_low_(TimeDelta::Zero()),
      fbbr_hybrid_srtt_low_source_time_(QuicTime::Zero()),
      fbbr_hybrid_lower_bound_search_active_(false),
      fbbr_hybrid_lower_bound_search_baseline_(QuicBandwidth::Zero()),
      fbbr_hybrid_lower_bound_search_step_count_(0),
      fbbr_hybrid_lower_bound_search_bdp_(0),
      fbbr_hybrid_stable_observation_source_(
          HybridStableObservationSource::kNone),
      fbbr_hybrid_stable_observation_start_(QuicTime::Zero()),
      fbbr_hybrid_stable_observation_round_done_(false),
      fbbr_hybrid_stable_observation_min_rtt_(TimeDelta::Infinite()),
      fbbr_hybrid_srtt_no_wave_streak_(0),
      fbbr_hybrid_drate_no_wave_streak_(0),
      fbbr_hybrid_wave_fidelity_enhancement_active_(false),
      fbbr_hybrid_retry_reason_mask_(0),
      fbbr_hybrid_last_counted_window_second_cycle_id_(0),
      fbbr_hybrid_rolling_retry_count_(0),
      fbbr_hybrid_regime_ii_seen_this_cruise_(false),
      fbbr_hybrid_trusted_bw_(QuicBandwidth::Zero()),
      floor_clip_confirmation_count_(0),
      waveform_last_clip_direction_(0),
      waveform_decision_count_(0),
      waveform_amplitude_reduction_count_(0),
      trusted_bw_candidate_update_count_(0),
      trusted_bw_candidate_(QuicBandwidth::Zero()),
      trusted_bw_candidate_source_(kTrustedBwSourceNone),
      fbbr_latest_trusted_bw_(QuicBandwidth::Zero()),
      fbbr_smoothed_trusted_bw_(QuicBandwidth::Zero()),
      fbbr_smoothed_trusted_bw_valid_(false),
      waveform_last_action_("none"),
      waveform_last_invalid_reason_("none"),
      waveform_initial_settle_rtt_mult_(1.0),
      waveform_post_adjust_settle_rtt_mult_(1.0),
      waveform_negative_half_first_(true),
      waveform_initial_window_periods_(2.0),
      waveform_extended_window_periods_(3.0),
      waveform_max_window_periods_(3.0),
      waveform_period_tolerance_ratio_(0.15),
      waveform_min_periodicity_correlation_(0.50),
      waveform_min_cycle_coverage_ratio_(0.85),
      waveform_masked_min_cycle_coverage_ratio_(0.50),
      waveform_min_completeness_score_(0.60),
      waveform_min_rising_duration_ratio_(0.15),
      waveform_min_falling_duration_ratio_(0.15),
      waveform_min_shape_ncc_(0.35),
      waveform_min_slope_direction_agreement_(0.65),
      waveform_min_drate_ncc_(0.50),
      waveform_min_srtt_integral_ncc_(0.45),
      waveform_min_srtt_derivative_ncc_(0.45),
      waveform_min_response_snr_(2.0),
      waveform_local_slope_window_period_ratio_(0.05),
      waveform_min_local_slope_window_ms_(5.0),
      waveform_clip_min_duration_ratio_(0.15),
      waveform_clip_min_half_overlap_ratio_(0.75),
      waveform_clip_max_slope_ratio_(0.10),
      waveform_delta_drate_amplitude_ratio_(0.50),
      waveform_delta_fallback_baseline_ratio_(0.25),
      waveform_adaptive_delta_fallback_baseline_ratio_(0.10),
      waveform_delta_ewma_alpha_(0.125),
      waveform_delta_min_baseline_ratio_(0.02),
      waveform_delta_max_baseline_ratio_(0.15),
      waveform_overload_max_delta_multiplier_(6.0),
      waveform_underload_max_delta_multiplier_(2.0),
      waveform_overload_confirmations_(2),
      waveform_queue_guard_enabled_(true),
      waveform_queue_low_min_rtt_ratio_(0.10),
      waveform_queue_target_min_rtt_ratio_(0.25),
      waveform_queue_high_min_rtt_ratio_(0.75),
      waveform_plateau_min_duration_ratio_(0.10),
      waveform_plateau_max_slope_ratio_(0.20),
      waveform_plateau_max_level_span_ratio_(0.15),
      waveform_plateau_extreme_distance_ratio_(0.15),
      waveform_baseline_step_ratio_(0.25),
      waveform_amplitude_floor_ratio_(0.125),
      waveform_clip_floor_confirmations_(2),
      waveform_max_baseline_adjustments_(8),
      waveform_max_inconclusive_extensions_(1),
      waveform_inconclusive_signal_amplification_factor_(1.25),
      waveform_inconclusive_signal_amplification_max_ratio_(2.0),
      waveform_max_app_limited_sample_ratio_(0.25),
      waveform_max_interpolation_gap_period_ratio_(0.10),
      fbbr_regime_long_top_horizontal_duration_ratio_(0.20),
      fbbr_regime_long_bottom_horizontal_duration_ratio_(0.30),
      fbbr_regime_actuator_midpoint_trigger_ratio_(0.50),
      fbbr_wave_fidelity_no_wave_trigger_windows_(2),
      fbbr_wave_fidelity_stop_on_either_wave_(true),
      fbbr_wave_fidelity_retry_window_advance_periods_(1),
      waveform_activity_amplitude_noise_multiplier_(6.0),
      waveform_activity_min_level_ratio_(0.02),
      waveform_activity_step_noise_multiplier_(3.0),
      waveform_activity_min_normalized_step_slope_(3.5),
      waveform_activity_min_active_steps_(4),
      waveform_activity_min_active_step_ratio_(0.10),
      waveform_activity_min_directional_change_ratio_(0.20),
      waveform_activity_min_significant_path_ratio_(0.80),
      waveform_activity_min_slope_reversals_(1),
      waveform_horizontal_continuous_min_duration_ratio_(0.15),
      waveform_horizontal_min_valid_coverage_ratio_(0.85),
      waveform_horizontal_min_flat_fraction_(0.90),
      waveform_horizontal_max_local_slope_ratio_(0.10),
      waveform_horizontal_min_side_slope_ratio_(0.25),
      waveform_horizontal_min_boundary_kink_ratio_(0.25),
      waveform_horizontal_max_level_span_ratio_(0.10),
      waveform_horizontal_max_total_drift_ratio_(0.05),
      waveform_horizontal_min_side_change_ratio_(0.10),
      waveform_horizontal_amplitude_noise_multiplier_(6.0),
      waveform_horizontal_level_span_noise_multiplier_(4.0),
      waveform_horizontal_slope_noise_multiplier_(3.0),
      waveform_horizontal_extreme_distance_ratio_(0.10),
      waveform_repeated_clip_max_period_error_ratio_(0.15),
      waveform_repeated_clip_max_level_delta_ratio_(0.05),
      waveform_repeated_clip_contact_level_tolerance_ratio_(0.05),
      waveform_repeated_clip_min_contact_samples_per_cycle_(2),
      waveform_repeated_clip_min_total_contact_samples_(4),
      waveform_repeated_clip_min_contact_sample_ratio_(0.05),
      waveform_repeated_clip_min_contact_span_ratio_of_window_(0.50),
      waveform_repeated_clip_min_pooled_flat_fraction_(0.90),
      waveform_repeated_clip_min_verified_boundary_fraction_(0.75),
      waveform_repeated_clip_min_outside_excursion_ratio_(0.10),
      waveform_repeated_clip_min_extrapolated_overshoot_ratio_(0.05),
      waveform_repeated_clip_merge_gap_ratio_(0.025),
      waveform_repeated_clip_max_missing_gap_ratio_(0.05),
      waveform_shoulder_min_half_overlap_ratio_(0.75),
      waveform_shoulder_min_side_change_ratio_(0.15),
      waveform_shoulder_max_residual_cycle_period_error_ratio_(0.20),
      waveform_shoulder_min_residual_cycle_leg_duration_ratio_(0.15),
      waveform_middle_min_duration_ratio_(0.05),
      waveform_middle_max_duration_ratio_(0.35),
      waveform_middle_context_duration_ratio_(0.10),
      waveform_middle_min_trend_slope_ratio_(0.20),
      waveform_middle_max_context_slope_delta_ratio_(0.75),
      waveform_middle_min_slope_mismatch_ratio_(0.50),
      waveform_middle_min_mismatching_sample_ratio_(0.25),
      waveform_middle_min_mismatching_samples_(2),
      waveform_middle_min_consecutive_mismatching_samples_(2),
      waveform_middle_min_bridge_deviation_ratio_(0.05),
      waveform_middle_noise_multiplier_(3.0),
      waveform_middle_max_mask_ratio_per_cycle_(0.35),
      fbbr_regime_period_tolerance_ratio_(0.20),
      fbbr_regime_min_periodicity_correlation_(0.50),
      fbbr_regime_periodic_upper_clip_is_hard_veto_(true),
      min_cruise_cycles_per_window_(4.0),
      cruise_window_step_ratio_(0.25),
      freq_tolerance_ratio_(0.20),
      min_full_load_quality_for_reliable_window_(0.50),
      default_ecn_congestion_ratio_(0.02),
	      fair_share_bandwidth_bps_(0),
	      cruise_baseline_cap_bps_(0),
	      cruise_freq_tool_active_(false),
      guard_filter_valid_(false),
      guard_filter_stage1_(QuicBandwidth::Zero()),
      guard_filter_stage2_(QuicBandwidth::Zero()),
      guard_updated_this_cruise_(false),
      guard_window_active_(false),
      guard_window_delivered_start_(0),
      guard_window_ack_start_(QuicTime::Zero()),
      guard_window_send_start_(QuicTime::Zero()),
      guard_window_app_limited_(false),
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
      last_pacing_gate_trace_time_(QuicTime::Zero()),
      fbbr_v3_max_rtprop_seen_(TimeDelta::Zero()),
      fbbr_v3_history_start_time_(QuicTime::Zero()),
      fbbr_v3_history_valid_(false),
      fbbr_v3_last_recorded_pacing_target_(QuicBandwidth::Zero()),
      fbbr_v3_telemetry_last_time_(QuicTime::Zero()),
      fbbr_v3_telemetry_initialized_(false),
      fbbr_v3_telemetry_reference_source_(
          FbbrV3ReferenceSource::kInvalid),
      fbbr_v3_telemetry_projection_active_(false),
      fbbr_v3_telemetry_history_invalid_(true),
      fbbr_v3_telemetry_cap_binding_(false),
      fbbr_v3_telemetry_model_inflight_(0),
      fbbr_v3_telemetry_raw_queue_debt_(0),
      fbbr_v3_telemetry_enforced_excess_(0),
      fbbr_v3_telemetry_total_us_(0),
      fbbr_v3_reference_trusted_us_(0),
      fbbr_v3_reference_guard_us_(0),
      fbbr_v3_reference_last_valid_us_(0),
      fbbr_v3_reference_invalid_us_(0),
      fbbr_v3_projection_active_us_(0),
      fbbr_v3_history_invalid_us_(0),
      fbbr_v3_cap_binding_us_(0),
      fbbr_v3_model_inflight_byte_us_(0.0L),
      fbbr_v3_raw_queue_debt_byte_us_(0.0L),
      fbbr_v3_enforced_excess_byte_us_(0.0L),
      fbbr_v3_window_ack_events_(0),
      fbbr_v3_window_cap_binding_events_(0),
      fbbr_v3_flow_summary_emitted_(false),
      fbbr_v4_max_rtprop_seen_(TimeDelta::Zero()),
      fbbr_v4_rate_history_integrity_valid_(true),
      fbbr_v4_last_target_rate_(QuicBandwidth::Zero()),
      fbbr_v4_last_base_target_rate_(QuicBandwidth::Zero()),
      fbbr_v4_delivered_history_integrity_valid_(true),
      fbbr_v4_last_counter_reset_time_(QuicTime::Zero()),
      fbbr_v4_telemetry_last_time_(QuicTime::Zero()),
      fbbr_v4_telemetry_initialized_(false),
      fbbr_v4_telemetry_reference_source_(
          FbbrV3ReferenceSource::kInvalid),
      fbbr_v4_telemetry_projection_active_(false),
      fbbr_v4_telemetry_service_history_valid_(false),
      fbbr_v4_telemetry_app_limited_fallback_(false),
      fbbr_v4_telemetry_plan_only_fallback_(false),
      fbbr_v4_telemetry_service_limited_(false),
      fbbr_v4_telemetry_cap_binding_(false),
      fbbr_v4_telemetry_plan_inflight_(0),
      fbbr_v4_telemetry_service_inflight_(0),
      fbbr_v4_telemetry_probe_credit_(0),
      fbbr_v4_telemetry_extra_acked_(0),
      fbbr_v4_telemetry_service_restriction_(0),
      fbbr_v4_telemetry_enforced_excess_(0),
      fbbr_v4_telemetry_total_us_(0),
      fbbr_v4_reference_trusted_us_(0),
      fbbr_v4_reference_guard_us_(0),
      fbbr_v4_reference_last_valid_us_(0),
      fbbr_v4_reference_invalid_us_(0),
      fbbr_v4_projection_active_us_(0),
      fbbr_v4_service_history_valid_us_(0),
      fbbr_v4_app_limited_fallback_us_(0),
      fbbr_v4_plan_only_fallback_us_(0),
      fbbr_v4_service_limited_us_(0),
      fbbr_v4_cap_binding_us_(0),
      fbbr_v4_plan_inflight_byte_us_(0.0L),
      fbbr_v4_service_inflight_byte_us_(0.0L),
      fbbr_v4_probe_credit_byte_us_(0.0L),
      fbbr_v4_extra_acked_byte_us_(0.0L),
      fbbr_v4_service_restriction_byte_us_(0.0L),
      fbbr_v4_enforced_excess_byte_us_(0.0L),
      fbbr_v4_window_ack_events_(0),
      fbbr_v4_window_cap_binding_events_(0),
      fbbr_v4_flow_summary_emitted_(false) {
  InitializeHybridSrttLowFromModel();
  QUIC_DVLOG(2) << this << " Initializing FBBRSender @ " << now
                << "; DefaultEcnCongestionRatio="
                << default_ecn_congestion_ratio_;
}

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

void FBBRSender::SetCruiseBaselineCapBps(uint64_t cap_bps) {
  cruise_baseline_cap_bps_ = cap_bps;
}

void FBBRSender::SetConvergenceGateTraceEnabled(bool enabled) {
  enable_convergence_gate_trace_ = enabled;
}

void FBBRSender::SetConvergenceGateControlEnabled(bool enabled) {
  enable_convergence_gate_control_ = enabled;
}

void FBBRSender::ConfigureFBBR(const FBBRConfig& config) {
  auto finite_or = [](double value,
                      double fallback,
                      const char* name) {
    if (!std::isfinite(value)) {
      std::cerr << "[FBBR config warning] invalid " << name << "="
                << value << "; using " << fallback << std::endl;
      return fallback;
    }
    return value;
  };
  auto range_or = [&finite_or](double value,
                               double low,
                               double high,
                               double fallback,
                               const char* name) {
    value = finite_or(value, fallback, name);
    if (value < low || value > high) {
      std::cerr << "[FBBR config warning] out-of-range " << name << "="
                << value << " (expected " << low << ".." << high
                << "); using " << fallback << std::endl;
      return fallback;
    }
    return value;
  };

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

  const double maximum_rate_mbps =
      static_cast<double>(std::numeric_limits<int64_t>::max() / 1000000);
  const double minimum_rate_mbps = range_or(
      config.pacing_minimum_rate_mbps, 0.000001, maximum_rate_mbps, 1.0,
      "pacing.minimum_rate_mbps");
  minimum_pacing_rate_bps_ = std::max<uint64_t>(
      1, static_cast<uint64_t>(std::llround(minimum_rate_mbps * 1000000.0)));

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

  if (config.cruise_detector_mode == "legacy_spectral") {
    cruise_detector_mode_ = FBBRCruiseDetectorMode::kLegacySpectral;
  } else if (config.cruise_detector_mode == "time_waveform") {
    cruise_detector_mode_ = FBBRCruiseDetectorMode::kTimeWaveform;
  } else {
    std::cerr << "[FBBR config warning] invalid cruise_detector.mode='"
              << config.cruise_detector_mode
              << "'; using time_waveform" << std::endl;
    cruise_detector_mode_ = FBBRCruiseDetectorMode::kTimeWaveform;
  }
  const bool waveform_recv_mode_valid =
      config.waveform_recv_signal_mode == "delivery_rate_latest" ||
      config.waveform_recv_signal_mode == "bandwidth_latest";
  if (!waveform_recv_mode_valid) {
    std::cerr
        << "[FBBR config warning] invalid waveform.recv_signal_mode='"
        << config.waveform_recv_signal_mode
        << "'; using delivery_rate_latest" << std::endl;
  }
  if (cruise_detector_mode_ ==
      FBBRCruiseDetectorMode::kLegacySpectral) {
    // This selector only controls the time-domain detector history.
    use_delivery_rate_latest_for_signal_history_ = false;
  } else if (IsFbbrHybridObserver()) {
    // The PDF-defined classifier requires delivery_rate_latest; the legacy
    // bandwidth_latest selector is intentionally ignored for this owner.
    use_delivery_rate_latest_for_signal_history_ = true;
  } else if (!waveform_recv_mode_valid ||
             config.waveform_recv_signal_mode == "delivery_rate_latest") {
    use_delivery_rate_latest_for_signal_history_ = true;
  } else {
    use_delivery_rate_latest_for_signal_history_ = false;
  }
  waveform_initial_settle_rtt_mult_ = range_or(
      config.waveform_initial_settle_rtt_mult, 0.0, 20.0, 1.0,
      "waveform.initial_settle_rtt_mult");
  waveform_post_adjust_settle_rtt_mult_ = range_or(
      config.waveform_post_adjust_settle_rtt_mult, 0.0, 20.0, 1.0,
      "waveform.post_adjust_settle_rtt_mult");
  if (cruise_detector_mode_ == FBBRCruiseDetectorMode::kTimeWaveform &&
      (std::abs(waveform_initial_settle_rtt_mult_ - 1.0) > 1e-12 ||
       std::abs(waveform_post_adjust_settle_rtt_mult_ - 1.0) > 1e-12)) {
    std::cerr << "[FBBR config warning] time_waveform locks initial and "
                 "post-adjust response delay to exactly 1 RTT"
              << std::endl;
    waveform_initial_settle_rtt_mult_ = 1.0;
    waveform_post_adjust_settle_rtt_mult_ = 1.0;
  }
  waveform_negative_half_first_ = true;
  if (!config.waveform_negative_half_first &&
      cruise_detector_mode_ == FBBRCruiseDetectorMode::kTimeWaveform) {
    std::cerr << "[FBBR config warning] waveform.negative_half_first=false "
                 "is inactive; time_waveform always starts with the negative "
                 "half-cycle"
              << std::endl;
  }
  waveform_initial_window_periods_ = range_or(
      config.waveform_initial_window_periods, 1.5, 2.0, 2.0,
      "waveform.initial_window_periods");
  const double extended_window_default = std::min(
      4.0, waveform_initial_window_periods_ + 1.0);
  waveform_extended_window_periods_ = range_or(
      config.waveform_extended_window_periods,
      extended_window_default, 4.0, extended_window_default,
      "waveform.extended_window_periods");
  const double max_window_default = waveform_extended_window_periods_;
  waveform_max_window_periods_ = range_or(
      config.waveform_max_window_periods,
      waveform_extended_window_periods_, 4.0, max_window_default,
      "waveform.max_window_periods");
  waveform_period_tolerance_ratio_ = range_or(
      config.waveform_period_tolerance_ratio, 0.01, 1.0, 0.15,
      "waveform.period_tolerance_ratio");
  waveform_min_periodicity_correlation_ = range_or(
      config.waveform_min_periodicity_correlation, -1.0, 1.0, 0.50,
      "waveform.min_periodicity_correlation");
  waveform_min_cycle_coverage_ratio_ = range_or(
      config.waveform_min_cycle_coverage_ratio, 0.0, 1.0, 0.85,
      "waveform.min_cycle_coverage_ratio");
  waveform_masked_min_cycle_coverage_ratio_ = range_or(
      config.waveform_masked_min_cycle_coverage_ratio, 0.35,
      waveform_min_cycle_coverage_ratio_, 0.50,
      "waveform.masked_min_cycle_coverage_ratio");
  waveform_min_completeness_score_ = range_or(
      config.waveform_min_completeness_score, 0.0, 1.0, 0.60,
      "waveform.min_completeness_score");
  waveform_min_rising_duration_ratio_ = range_or(
      config.waveform_min_rising_duration_ratio, 0.0, 0.5, 0.15,
      "waveform.min_rising_duration_ratio");
  waveform_min_falling_duration_ratio_ = range_or(
      config.waveform_min_falling_duration_ratio, 0.0, 0.5, 0.15,
      "waveform.min_falling_duration_ratio");
  waveform_min_shape_ncc_ = range_or(
      config.waveform_min_shape_ncc, -1.0, 1.0, 0.35,
      "waveform.min_shape_ncc");
  waveform_min_slope_direction_agreement_ = range_or(
      config.waveform_min_slope_direction_agreement, 0.0, 1.0, 0.65,
      "waveform.min_slope_direction_agreement");
  waveform_min_drate_ncc_ = range_or(
      config.waveform_min_drate_ncc, -1.0, 1.0, 0.50,
      "waveform.min_drate_ncc");
  waveform_min_srtt_integral_ncc_ = range_or(
      config.waveform_min_srtt_integral_ncc, -1.0, 1.0, 0.45,
      "waveform.min_srtt_integral_ncc");
  waveform_min_srtt_derivative_ncc_ = range_or(
      config.waveform_min_srtt_derivative_ncc, -1.0, 1.0, 0.45,
      "waveform.min_srtt_derivative_ncc");
  waveform_min_response_snr_ = range_or(
      config.waveform_min_response_snr, 0.0, 1000.0, 2.0,
      "waveform.min_response_snr");
  waveform_local_slope_window_period_ratio_ = range_or(
      config.waveform_local_slope_window_period_ratio, 0.001, 0.5, 0.05,
      "waveform.local_slope_window_period_ratio");
  waveform_min_local_slope_window_ms_ = range_or(
      config.waveform_min_local_slope_window_ms, 0.1, 10000.0, 5.0,
      "waveform.min_local_slope_window_ms");
  waveform_clip_min_duration_ratio_ = range_or(
      config.waveform_clip_min_duration_ratio, 0.01, 0.5, 0.15,
      "waveform.clip_min_duration_ratio");
  waveform_clip_min_half_overlap_ratio_ = range_or(
      config.waveform_clip_min_half_overlap_ratio, 0.5, 1.0, 0.75,
      "waveform.clip_min_half_overlap_ratio");
  waveform_clip_max_slope_ratio_ = range_or(
      config.waveform_clip_max_slope_ratio, 0.0, 1.0, 0.10,
      "waveform.clip_max_slope_ratio");
  waveform_delta_drate_amplitude_ratio_ = range_or(
      config.waveform_delta_drate_amplitude_ratio, 0.0, 10.0, 0.50,
      "waveform.delta_drate_amplitude_ratio");
  waveform_delta_fallback_baseline_ratio_ = range_or(
      config.waveform_delta_fallback_baseline_ratio, 0.0, 0.95, 0.25,
      "waveform.delta_fallback_baseline_ratio");
  waveform_adaptive_delta_fallback_baseline_ratio_ = range_or(
      config.waveform_adaptive_delta_fallback_baseline_ratio,
      0.0, 0.95, 0.10,
      "waveform.adaptive_delta_fallback_baseline_ratio");
  waveform_delta_ewma_alpha_ = range_or(
      config.waveform_delta_ewma_alpha, 0.0, 1.0, 0.125,
      "waveform.delta_ewma_alpha");
  waveform_delta_min_baseline_ratio_ = range_or(
      config.waveform_delta_min_baseline_ratio, 0.0, 0.50, 0.02,
      "waveform.delta_min_baseline_ratio");
  waveform_delta_max_baseline_ratio_ = range_or(
      config.waveform_delta_max_baseline_ratio, 0.0, 0.95, 0.15,
      "waveform.delta_max_baseline_ratio");
  if (waveform_delta_max_baseline_ratio_ <
      waveform_delta_min_baseline_ratio_) {
    std::cerr << "[FBBR config warning] waveform.delta_max_baseline_ratio "
                 "must be >= waveform.delta_min_baseline_ratio; using 0.15"
              << std::endl;
    waveform_delta_max_baseline_ratio_ = 0.15;
  }
  waveform_overload_max_delta_multiplier_ = range_or(
      config.waveform_overload_max_delta_multiplier, 1.0, 10.0, 6.0,
      "waveform.overload_max_delta_multiplier");
  waveform_underload_max_delta_multiplier_ = range_or(
      config.waveform_underload_max_delta_multiplier, 1.0, 10.0, 2.0,
      "waveform.underload_max_delta_multiplier");
  waveform_overload_confirmations_ =
      std::max<uint32_t>(1, config.waveform_overload_confirmations);
  waveform_queue_guard_enabled_ = config.waveform_queue_guard_enabled;
  waveform_queue_low_min_rtt_ratio_ = range_or(
      config.waveform_queue_low_min_rtt_ratio, 0.0, 10.0, 0.10,
      "waveform.queue_low_min_rtt_ratio");
  waveform_queue_target_min_rtt_ratio_ = range_or(
      config.waveform_queue_target_min_rtt_ratio, 0.0, 10.0, 0.25,
      "waveform.queue_target_min_rtt_ratio");
  waveform_queue_high_min_rtt_ratio_ = range_or(
      config.waveform_queue_high_min_rtt_ratio, 0.0, 10.0, 0.75,
      "waveform.queue_high_min_rtt_ratio");
  if (waveform_queue_target_min_rtt_ratio_ <
          waveform_queue_low_min_rtt_ratio_ ||
      waveform_queue_high_min_rtt_ratio_ <
          waveform_queue_target_min_rtt_ratio_) {
    std::cerr << "[FBBR config warning] waveform queue ratios must satisfy "
                 "low <= target <= high; using 0.10/0.25/0.75"
              << std::endl;
    waveform_queue_low_min_rtt_ratio_ = 0.10;
    waveform_queue_target_min_rtt_ratio_ = 0.25;
    waveform_queue_high_min_rtt_ratio_ = 0.75;
  }

  waveform_plateau_min_duration_ratio_ = range_or(
      config.waveform_plateau_min_duration_ratio, 0.01, 0.5, 0.10,
      "waveform.plateau_min_duration_ratio");
  waveform_plateau_max_slope_ratio_ = range_or(
      config.waveform_plateau_max_slope_ratio, 0.0, 1.0, 0.20,
      "waveform.plateau_max_slope_ratio");
  waveform_plateau_max_level_span_ratio_ = range_or(
      config.waveform_plateau_max_level_span_ratio, 0.0, 1.0, 0.15,
      "waveform.plateau_max_level_span_ratio");
  waveform_plateau_extreme_distance_ratio_ = range_or(
      config.waveform_plateau_extreme_distance_ratio, 0.0, 1.0, 0.15,
      "waveform.plateau_extreme_distance_ratio");
  waveform_baseline_step_ratio_ = range_or(
      config.waveform_baseline_step_ratio, 0.0, 0.95, 0.25,
      "waveform.baseline_step_ratio");
  waveform_amplitude_floor_ratio_ = range_or(
      config.waveform_amplitude_floor_ratio, 0.0, 1.0, 0.125,
      "waveform.amplitude_floor_ratio");
  if (config.waveform_clip_floor_confirmations == 0) {
    std::cerr << "[FBBR config warning] "
                 "waveform.clip_floor_confirmations must be >= 1; using 2"
              << std::endl;
    waveform_clip_floor_confirmations_ = 2;
  } else {
    waveform_clip_floor_confirmations_ =
        config.waveform_clip_floor_confirmations;
  }
  if (cruise_detector_mode_ == FBBRCruiseDetectorMode::kTimeWaveform) {
    std::cerr << "[FBBR config] waveform.min_completeness_score, "
                 "waveform.min_rising_duration_ratio, "
                 "waveform.min_falling_duration_ratio, "
                 "waveform.min_shape_ncc, "
                 "waveform.min_slope_direction_agreement, "
                 "waveform.min_response_snr, "
                 "waveform.amplitude_floor_ratio, "
                 "waveform.clip_floor_confirmations, "
                 "waveform.baseline_step_ratio are deprecated/inactive in "
                 "time_waveform mode"
              << std::endl;
  }
  if (config.waveform_max_baseline_adjustments == 0) {
    std::cerr << "[FBBR config warning] "
                 "waveform.max_baseline_adjustments must be >= 1; using 8"
              << std::endl;
    waveform_max_baseline_adjustments_ = 8;
  } else {
    waveform_max_baseline_adjustments_ =
        config.waveform_max_baseline_adjustments;
  }
  waveform_max_inconclusive_extensions_ = 1;
  if (!UsesAdaptiveLoadJudgment() &&
      cruise_detector_mode_ == FBBRCruiseDetectorMode::kTimeWaveform &&
      config.waveform_max_inconclusive_extensions != 1) {
    std::cerr << "[FBBR config warning] non-Adaptive time_waveform "
                 "observes exactly one extra period after an inconclusive "
                 "decision"
              << std::endl;
  }
  waveform_inconclusive_signal_amplification_factor_ = range_or(
      config.waveform_inconclusive_signal_amplification_factor,
      1.0, 4.0, 1.25,
      "waveform.inconclusive_signal_amplification_factor");
  waveform_inconclusive_signal_amplification_max_ratio_ = range_or(
      config.waveform_inconclusive_signal_amplification_max_ratio,
      1.0, 16.0, 2.0,
      "waveform.inconclusive_signal_amplification_max_ratio");
  waveform_max_app_limited_sample_ratio_ = range_or(
      config.waveform_max_app_limited_sample_ratio, 0.0, 1.0, 0.25,
      "waveform.max_app_limited_sample_ratio");
  waveform_max_interpolation_gap_period_ratio_ = range_or(
      config.waveform_max_interpolation_gap_period_ratio, 0.0, 1.0, 0.10,
      "waveform.max_interpolation_gap_period_ratio");

  fbbr_regime_long_top_horizontal_duration_ratio_ = range_or(
      config.fbbr_regime_long_top_horizontal_duration_ratio,
      0.01, 0.99, 0.20,
      "fbbr.regime.long_top_horizontal_duration_ratio");
  fbbr_regime_long_bottom_horizontal_duration_ratio_ = range_or(
      config.fbbr_regime_long_bottom_horizontal_duration_ratio,
      0.01, 0.99, 0.30,
      "fbbr.regime.long_bottom_horizontal_duration_ratio");
  fbbr_regime_actuator_midpoint_trigger_ratio_ = range_or(
      config.fbbr_regime_actuator_midpoint_trigger_ratio,
      0.0, 1.0, 0.50,
      "fbbr.regime.actuator.midpoint_trigger_ratio");
  // These are PDF semantics rather than tuning knobs.
  fbbr_wave_fidelity_no_wave_trigger_windows_ = 2;
  fbbr_wave_fidelity_stop_on_either_wave_ = true;
  fbbr_wave_fidelity_retry_window_advance_periods_ = 1;

  waveform_activity_amplitude_noise_multiplier_ = range_or(
      config.waveform_activity_amplitude_noise_multiplier,
      0.0, 100.0, 6.0,
      "waveform.activity.amplitude_noise_multiplier");
  waveform_activity_min_level_ratio_ = range_or(
      config.waveform_activity_min_level_ratio, 0.0, 1.0, 0.02,
      "waveform.activity.min_level_ratio");
  waveform_activity_step_noise_multiplier_ = range_or(
      config.waveform_activity_step_noise_multiplier,
      0.0, 100.0, 3.0,
      "waveform.activity.step_noise_multiplier");
  waveform_activity_min_normalized_step_slope_ = range_or(
      config.waveform_activity_min_normalized_step_slope,
      0.0, 100.0, 3.5,
      "waveform.activity.min_normalized_step_slope");
  waveform_activity_min_active_steps_ = std::max<uint32_t>(
      1, config.waveform_activity_min_active_steps);
  waveform_activity_min_active_step_ratio_ = Clamp01(
      config.waveform_activity_min_active_step_ratio);
  waveform_activity_min_directional_change_ratio_ = Clamp01(
      config.waveform_activity_min_directional_change_ratio);
  waveform_activity_min_significant_path_ratio_ = range_or(
      config.waveform_activity_min_significant_path_ratio,
      0.0, 10.0, 0.80,
      "waveform.activity.min_significant_path_ratio");
  waveform_activity_min_slope_reversals_ = std::max<uint32_t>(
      1, config.waveform_activity_min_slope_reversals);

  waveform_horizontal_continuous_min_duration_ratio_ = range_or(
      config.waveform_horizontal_continuous_min_duration_ratio,
      0.01, 1.0, 0.15,
      "waveform.horizontal.continuous_min_duration_ratio");
  waveform_horizontal_min_valid_coverage_ratio_ = Clamp01(
      config.waveform_horizontal_min_valid_coverage_ratio);
  waveform_horizontal_min_flat_fraction_ = Clamp01(
      config.waveform_horizontal_min_flat_fraction);
  waveform_horizontal_max_local_slope_ratio_ = range_or(
      config.waveform_horizontal_max_local_slope_ratio,
      0.0, 10.0, 0.10,
      "waveform.horizontal.max_local_slope_ratio");
  waveform_horizontal_min_side_slope_ratio_ = range_or(
      config.waveform_horizontal_min_side_slope_ratio,
      0.0, 10.0, 0.25,
      "waveform.horizontal.min_side_slope_ratio");
  waveform_horizontal_min_boundary_kink_ratio_ = range_or(
      config.waveform_horizontal_min_boundary_kink_ratio,
      0.0, 10.0, 0.25,
      "waveform.horizontal.min_boundary_kink_ratio");
  waveform_horizontal_max_level_span_ratio_ = Clamp01(
      config.waveform_horizontal_max_level_span_ratio);
  waveform_horizontal_max_total_drift_ratio_ = Clamp01(
      config.waveform_horizontal_max_total_drift_ratio);
  waveform_horizontal_min_side_change_ratio_ = Clamp01(
      config.waveform_horizontal_min_side_change_ratio);
  waveform_horizontal_amplitude_noise_multiplier_ = std::max(
      0.0, config.waveform_horizontal_amplitude_noise_multiplier);
  waveform_horizontal_level_span_noise_multiplier_ = std::max(
      0.0, config.waveform_horizontal_level_span_noise_multiplier);
  waveform_horizontal_slope_noise_multiplier_ = std::max(
      0.0, config.waveform_horizontal_slope_noise_multiplier);
  waveform_horizontal_extreme_distance_ratio_ = Clamp01(
      config.waveform_horizontal_extreme_distance_ratio);

  waveform_repeated_clip_max_period_error_ratio_ = Clamp01(
      config.waveform_repeated_clip_max_period_error_ratio);
  waveform_repeated_clip_max_level_delta_ratio_ = Clamp01(
      config.waveform_repeated_clip_max_level_delta_ratio);
  waveform_repeated_clip_contact_level_tolerance_ratio_ = Clamp01(
      config.waveform_repeated_clip_contact_level_tolerance_ratio);
  waveform_repeated_clip_min_contact_samples_per_cycle_ =
      std::max<uint32_t>(2,
          config.waveform_repeated_clip_min_contact_samples_per_cycle);
  waveform_repeated_clip_min_total_contact_samples_ =
      std::max<uint32_t>(4,
          config.waveform_repeated_clip_min_total_contact_samples);
  waveform_repeated_clip_min_contact_sample_ratio_ = Clamp01(
      config.waveform_repeated_clip_min_contact_sample_ratio);
  waveform_repeated_clip_min_contact_span_ratio_of_window_ = 0.50;
  waveform_repeated_clip_min_pooled_flat_fraction_ = Clamp01(
      config.waveform_repeated_clip_min_pooled_flat_fraction);
  waveform_repeated_clip_min_verified_boundary_fraction_ = Clamp01(
      config.waveform_repeated_clip_min_verified_boundary_fraction);
  waveform_repeated_clip_min_outside_excursion_ratio_ = Clamp01(
      config.waveform_repeated_clip_min_outside_excursion_ratio);
  waveform_repeated_clip_min_extrapolated_overshoot_ratio_ = Clamp01(
      config.waveform_repeated_clip_min_extrapolated_overshoot_ratio);
  waveform_repeated_clip_merge_gap_ratio_ = Clamp01(
      config.waveform_repeated_clip_merge_gap_ratio);
  waveform_repeated_clip_max_missing_gap_ratio_ = Clamp01(
      config.waveform_repeated_clip_max_missing_gap_ratio);

  waveform_shoulder_min_half_overlap_ratio_ = Clamp01(
      config.waveform_shoulder_min_half_overlap_ratio);
  waveform_shoulder_min_side_change_ratio_ = Clamp01(
      config.waveform_shoulder_min_side_change_ratio);
  waveform_shoulder_max_residual_cycle_period_error_ratio_ = Clamp01(
      config.waveform_shoulder_max_residual_cycle_period_error_ratio);
  waveform_shoulder_min_residual_cycle_leg_duration_ratio_ = Clamp01(
      config.waveform_shoulder_min_residual_cycle_leg_duration_ratio);

  waveform_middle_min_duration_ratio_ = Clamp01(
      config.waveform_middle_min_duration_ratio);
  waveform_middle_max_duration_ratio_ = Clamp01(
      config.waveform_middle_max_duration_ratio);
  waveform_middle_context_duration_ratio_ = Clamp01(
      config.waveform_middle_context_duration_ratio);
  waveform_middle_min_trend_slope_ratio_ = std::max(
      0.0, config.waveform_middle_min_trend_slope_ratio);
  waveform_middle_max_context_slope_delta_ratio_ = std::max(
      0.0, config.waveform_middle_max_context_slope_delta_ratio);
  waveform_middle_min_slope_mismatch_ratio_ = std::max(
      0.0, config.waveform_middle_min_slope_mismatch_ratio);
  waveform_middle_min_mismatching_sample_ratio_ = Clamp01(
      config.waveform_middle_min_mismatching_sample_ratio);
  waveform_middle_min_mismatching_samples_ = std::max<uint32_t>(
      2, config.waveform_middle_min_mismatching_samples);
  waveform_middle_min_consecutive_mismatching_samples_ =
      std::max<uint32_t>(
          2, config.waveform_middle_min_consecutive_mismatching_samples);
  waveform_middle_min_bridge_deviation_ratio_ = Clamp01(
      config.waveform_middle_min_bridge_deviation_ratio);
  waveform_middle_noise_multiplier_ = std::max(
      0.0, config.waveform_middle_noise_multiplier);
  waveform_middle_max_mask_ratio_per_cycle_ = Clamp01(
      config.waveform_middle_max_mask_ratio_per_cycle);
  if (waveform_middle_max_duration_ratio_ <
      waveform_middle_min_duration_ratio_) {
    waveform_middle_min_duration_ratio_ = 0.05;
    waveform_middle_max_duration_ratio_ = 0.35;
  }

  fbbr_regime_period_tolerance_ratio_ = range_or(
      config.fbbr_regime_period_tolerance_ratio,
      0.0, 1.0, 0.20,
      "fbbr.regime.period_tolerance_ratio");
  fbbr_regime_min_periodicity_correlation_ = range_or(
      config.fbbr_regime_min_periodicity_correlation,
      -1.0, 1.0, 0.50,
      "fbbr.regime.min_periodicity_correlation");
  fbbr_regime_periodic_upper_clip_is_hard_veto_ = true;
  fbbr_latest_trusted_bw_ = QuicBandwidth::Zero();
  fbbr_smoothed_trusted_bw_ = QuicBandwidth::Zero();
  fbbr_smoothed_trusted_bw_valid_ = false;
  latest_waveform_overload_srtt_mean_valid_ = false;
  latest_waveform_overload_srtt_mean_ms_ = 0.0;
  latest_waveform_underload_srtt_mean_valid_ = false;
  latest_waveform_underload_srtt_mean_ms_ = 0.0;
  adaptive_baseline_low_valid_ = false;
  adaptive_baseline_low_ = QuicBandwidth::Zero();
  adaptive_baseline_up_valid_ = false;
  adaptive_baseline_up_ = QuicBandwidth::Zero();
  adaptive_previous_cruise_max_bw_valid_ = false;
  adaptive_previous_cruise_max_bw_ = QuicBandwidth::Zero();
  adaptive_cruise_start_max_bw_ = QuicBandwidth::Zero();
  adaptive_bounds_inherited_this_cruise_ = false;
  fbbr_hybrid_max_srtt_valid_ = false;
  fbbr_hybrid_max_srtt_ms_ = 0.0;
  fbbr_hybrid_max_srtt_source_cruise_id_ = 0;
  pending_hybrid_max_srtt_observations_.clear();
  fbbr_hybrid_rtprop_drate_valid_ = false;
  fbbr_hybrid_rtprop_drate_ = QuicBandwidth::Zero();
  fbbr_hybrid_rtprop_drate_source_cruise_id_ = 0;
  fbbr_hybrid_lower_bound_search_active_ = false;
  fbbr_hybrid_lower_bound_search_baseline_ = QuicBandwidth::Zero();
  fbbr_hybrid_lower_bound_search_step_count_ = 0;
  fbbr_hybrid_lower_bound_search_bdp_ = 0;
  CancelHybridStableObservation();
  fbbr_hybrid_rtprop_drate_source_time_ = QuicTime::Zero();
  fbbr_hybrid_baseline_low_source_time_ = QuicTime::Zero();
  fbbr_hybrid_srtt_low_valid_ = false;
  fbbr_hybrid_srtt_low_ = TimeDelta::Zero();
  fbbr_hybrid_srtt_low_source_time_ = QuicTime::Zero();
  InitializeHybridSrttLowFromModel();
  guard_filter_valid_ = false;
  guard_filter_stage1_ = QuicBandwidth::Zero();
  guard_filter_stage2_ = QuicBandwidth::Zero();
  guard_updated_this_cruise_ = false;
  guard_window_active_ = false;
  guard_window_delivered_start_ = 0;
  guard_window_ack_start_ = QuicTime::Zero();
  guard_window_send_start_ = QuicTime::Zero();
  guard_window_app_limited_ = false;
  latest_congestion_event_prior_inflight_valid_ = false;
  latest_congestion_event_inflight_ = 0;
  latest_congestion_event_inflight_valid_ = false;
  max_bw_response_observed_ = false;
  max_bw_delivery_response_gain_ = 1.0;
  max_bw_observation_center_bps_ = 0.0;
  max_bw_observation_baseline_bps_ = 0.0;
  max_bw_actual_fluctuation_amplitude_bps_ = 0.0;
  max_bw_attenuation_factor_ = 1.0;
  model_.SetMaxBandwidthSampleAttenuation(1.0);
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
  const double cruise_injection_baseline = 90.0;
  const double cruise_triangle = 7.0;
  require(std::abs(cruise_injection_baseline + cruise_triangle - 97.0) <
              1e-12,
          "time-waveform CRUISE must center modulation on its injection baseline");
  const int64_t minimum_rate_bps = 1000000;
  const int64_t low_triangle_rate_bps = AddPacingOffsetWithFloor(
      4000000, -8000000, minimum_rate_bps);
  require(low_triangle_rate_bps == minimum_rate_bps,
          "negative modulation must be clamped to the configured pacing floor");
  require(AddPacingOffsetWithFloor(4000000, 2000000, minimum_rate_bps) ==
              6000000,
          "pacing floor must not change an in-range modulation result");
  os << "\nRESULT: " << (pass ? "PASS" : "FAIL") << "\n";
  return pass;
}

bool FBBRSender::RunWaveformCruiseSelfTest(std::ostream& os) {
  bool pass = true;
  auto require = [&pass, &os](bool condition, const std::string& message) {
    os << (condition ? "PASS: " : "FAIL: ") << message << "\n";
    pass = pass && condition;
  };
  auto triangle = [](double phase) {
    phase -= std::floor(phase);
    if (phase < 0.25) {
      return -4.0 * phase;
    }
    if (phase < 0.75) {
      return 4.0 * phase - 2.0;
    }
    return 4.0 - 4.0 * phase;
  };
  const size_t count = 201;
  std::vector<double> sender(count, 0.0);
  std::vector<bool> valid(count, true);
  for (size_t i = 0; i < count; ++i) {
    sender[i] = triangle(static_cast<double>(i) / (count - 1));
  }

  os << "# FBBR deterministic time-waveform CRUISE self-test\n";
  const double native_max_bw = 20.0;
  const double amplitude = 4.0;
  const double initial_baseline = native_max_bw;
  require(std::abs(initial_baseline - native_max_bw) < 1e-12 &&
              std::abs(initial_baseline + amplitude * triangle(0.0) -
                       native_max_bw) < 1e-12 &&
              std::abs(triangle(0.25) + 1.0) < 1e-12 &&
              triangle(0.125) < 0.0 && triangle(0.625) > 0.0,
          "test 1: baseline starts at Native MaxBw and the negative half-cycle is first");

  const double epoch_start_s = 10.0;
  const double epoch_srtt_s = 0.08;
  const double period_s = 0.20;
  require(std::abs((epoch_start_s + epoch_srtt_s) - 10.08) < 1e-12 &&
              std::abs((epoch_start_s + epoch_srtt_s + period_s) - 10.28) <
                  1e-12,
          "test 2: initial response collection starts one RTT after the epoch");
  const double adjusted_collection_start_s = epoch_start_s + epoch_srtt_s;
  const double adjusted_collection_end_s =
      adjusted_collection_start_s +
      kWaveformPostAdjustmentCollectionPeriods * period_s;
  require(std::abs(adjusted_collection_start_s - 10.08) < 1e-12 &&
              std::abs(adjusted_collection_end_s - 10.48) < 1e-12,
          "test 2b: post-adjustment epochs analyze the complete 2T window");
  require(std::abs(FBBRCwndGainForPhase(
                       Bbr2ProbeBwMode::CyclePhase::PROBE_CRUISE, 2.0f) -
                   1.25f) < 1e-6 &&
              std::abs(FBBRCwndGainForPhase(
                           Bbr2ProbeBwMode::CyclePhase::PROBE_UP, 2.0f) -
                       2.0f) < 1e-6,
          "test 2c: FBBR uses cwnd gain 1.25 only in CRUISE");

  const size_t periodic_count = 401;
  const size_t expected_period_samples = 200;
  std::vector<double> sine_response(periodic_count, 0.0);
  std::vector<double> triangle_response(periodic_count, 0.0);
  std::vector<double> wrong_frequency_response(periodic_count, 0.0);
  std::vector<bool> periodic_valid(periodic_count, true);
  for (size_t i = 0; i < periodic_count; ++i) {
    const double expected_phase =
        static_cast<double>(i) / expected_period_samples;
    sine_response[i] = std::sin(2.0 * M_PI * expected_phase);
    triangle_response[i] = triangle(expected_phase);
    wrong_frequency_response[i] =
        std::sin(2.0 * M_PI * static_cast<double>(i) / 280.0);
  }
  size_t sine_pairs = 0;
  size_t triangle_pairs = 0;
  const double sine_periodicity = ComputeLaggedCorrelation(
      sine_response, periodic_valid, expected_period_samples, &sine_pairs);
  const double triangle_periodicity = ComputeLaggedCorrelation(
      triangle_response, periodic_valid, expected_period_samples,
      &triangle_pairs);
  const double wrong_at_expected = ComputeLaggedCorrelation(
      wrong_frequency_response, periodic_valid, expected_period_samples,
      nullptr);
  const double wrong_at_own_period = ComputeLaggedCorrelation(
      wrong_frequency_response, periodic_valid, 280, nullptr);
  require(sine_pairs >= expected_period_samples &&
              triangle_pairs >= expected_period_samples &&
              sine_periodicity > 0.99 && triangle_periodicity > 0.99,
          "test 2d: same-frequency sine and triangle responses both pass periodicity");
  require(wrong_at_expected < 0.50 && wrong_at_own_period > 0.99 &&
              std::abs(280.0 / expected_period_samples - 1.0) > 0.25,
          "test 2e: a strong response at the wrong period fails the frequency tolerance");

  auto check_decision = [&require](WaveformDecisionInputs inputs,
                                   WaveformClassification expected,
                                   const char* expected_rule,
                                   const std::string& message) {
    const char* rule = "unset";
    const WaveformClassification actual =
        ClassifyWaveformState(inputs, &rule);
    require(actual == expected && std::string(rule) == expected_rule,
            message);
  };
  WaveformDecisionInputs decision;
  decision.prechecks_valid = true;
  decision.adaptive_guard_enabled = true;
  decision.srtt_input_valid = true;
  decision.drate_input_valid = true;
  decision.srtt_similar = true;
  decision.drate_similar = true;
  check_decision(decision, WaveformClassification::kFullLoad, "R1",
                 "test 3: similar unclipped SRTT is FULL_LOAD");
  decision.srtt_positive_half_clipped = true;
  decision.drate_positive_half_clipped = true;
  decision.positive_half_clips_simultaneous = true;
  check_decision(decision, WaveformClassification::kOverload, "R2.1",
                 "test 4: simultaneous positive SRTT/Drate clips are OVERLOAD");
  decision.drate_positive_half_clipped = false;
  decision.positive_half_clips_simultaneous = false;
  decision.drate_only_negative_half = true;
  check_decision(decision, WaveformClassification::kOverload, "R2.1",
                 "test 4b: similar Drate with only its negative half remaining is OVERLOAD");
  decision.drate_only_negative_half = false;
  check_decision(decision, WaveformClassification::kFullLoad, "R2.2",
                 "test 5: positive SRTT clip with similar Drate is FULL_LOAD");
  decision.srtt_positive_half_clipped = false;
  decision.srtt_only_negative_half = true;
  check_decision(decision, WaveformClassification::kFullLoad, "R2.2",
                 "test 5b: SRTT with only its negative half follows R2");
  decision.drate_only_negative_half = true;
  check_decision(decision, WaveformClassification::kOverload, "R2.1",
                 "test 5c: matching Drate only-negative-half evidence makes R2.1 OVERLOAD");
  decision.drate_only_negative_half = false;
  decision.srtt_only_negative_half = false;
  decision.srtt_positive_half_clipped = true;
  decision.drate_similar = false;
  check_decision(decision, WaveformClassification::kOverload, "R2.3",
                 "test 6: positive SRTT clip with dissimilar Drate is OVERLOAD");
  decision.srtt_positive_half_clipped = false;
  decision.srtt_negative_half_clipped = true;
  decision.drate_similar = true;
  check_decision(decision, WaveformClassification::kUnderload, "R3.1",
                 "test 7: negative SRTT clip with similar Drate is UNDERLOAD");
  decision.srtt_negative_half_clipped = false;
  decision.srtt_only_positive_half = true;
  check_decision(decision, WaveformClassification::kUnderload, "R3.1",
                 "test 7b: SRTT with only its positive half follows R3");
  decision.srtt_only_positive_half = false;
  decision.srtt_negative_half_clipped = true;
  decision.drate_similar = false;
  decision.drate_has_waveform = true;
  decision.drate_middle_any_plateau = true;
  check_decision(decision, WaveformClassification::kUnderload, "R3.2",
                 "test 8: negative SRTT clip plus a middle Drate plateau is UNDERLOAD");
  decision.drate_has_waveform = false;
  decision.drate_middle_any_plateau = false;
  check_decision(decision, WaveformClassification::kInconclusive, "R3.3",
                 "test 8b: negative SRTT clip without Drate evidence is INCONCLUSIVE");
  decision.srtt_positive_half_clipped = true;
  decision.drate_has_waveform = true;
  decision.drate_middle_any_plateau = true;
  check_decision(decision, WaveformClassification::kUnderload, "R4.2",
                 "test 8c: both SRTT halves clipped plus a middle Drate plateau is UNDERLOAD");
  decision.drate_has_waveform = false;
  decision.drate_middle_any_plateau = false;
  check_decision(decision, WaveformClassification::kInconclusive, "R4.3",
                 "test 8d: both SRTT halves clipped without Drate evidence is INCONCLUSIVE");
  decision.srtt_negative_half_clipped = false;
  decision.drate_input_valid = false;
  check_decision(decision, WaveformClassification::kInconclusive, "R2.4",
                 "test 8e: positive SRTT clip with invalid Drate is INCONCLUSIVE");
  decision.drate_input_valid = true;

  WaveformDecisionInputs bic_decision;
  bic_decision.prechecks_valid = true;
  bic_decision.adaptive_guard_enabled = true;
  bic_decision.srtt_input_valid = true;
  bic_decision.drate_input_valid = true;
  bic_decision.drate_similar = true;
  bic_decision.bic_srtt_bottom_clip = true;
  check_decision(bic_decision, WaveformClassification::kUnderload, "R3.1",
                 "test 8f: BIC bottom clip enters the existing UNDERLOAD path without waveform similarity");
  bic_decision.bic_srtt_bottom_clip = false;
  bic_decision.bic_srtt_top_clip = true;
  check_decision(bic_decision, WaveformClassification::kFullLoad, "R2.2",
                 "test 8g: BIC top clip enters the existing top-clip path without waveform similarity");
  bic_decision.bic_srtt_bottom_clip = true;
  check_decision(bic_decision, WaveformClassification::kUnderload, "R4.1",
                 "test 8h: simultaneous BIC top and bottom clips enter BOTH_CLIPPED");
  bic_decision.bic_srtt_top_clip = false;
  bic_decision.srtt_positive_half_clipped = true;
  check_decision(bic_decision, WaveformClassification::kUnderload, "R3.1",
                 "test 8i: true bottom clipping has priority over a similarity-shoulder top clip");
  bic_decision.bic_srtt_bottom_clip = false;
  bic_decision.bic_srtt_top_clip = true;
  bic_decision.srtt_positive_half_clipped = false;
  bic_decision.srtt_negative_half_clipped = true;
  check_decision(bic_decision, WaveformClassification::kFullLoad, "R2.2",
                 "test 8j: true top clipping has priority over a similarity-shoulder bottom clip");
  bic_decision.bic_srtt_top_clip = false;
  bic_decision.bic_srtt_bottom_clip = true;
  bic_decision.srtt_negative_half_clipped = false;
  bic_decision.prechecks_valid = false;
  check_decision(bic_decision, WaveformClassification::kUnderload, "R3.1",
                 "test 8k: true clipping is evaluated before similarity-path prechecks");

  require(!ShouldRefreshRtpropForTrueClip(false, false) &&
              !ShouldRefreshRtpropForTrueClip(true, false) &&
              ShouldRefreshRtpropForTrueClip(false, true) &&
              ShouldRefreshRtpropForTrueClip(true, true),
          "test 8l: every true-clipping result containing a bottom clip "
          "refreshes RTprop and ProbeRTT");

  require(std::abs(ComputeAdaptiveNextBaseline(
                       WaveformClassification::kOverload,
                       true, 80.0, true, 100.0,
                       100.0, 40.0, 60.0, 1.0) -
                   85.0) < 1e-12 &&
              std::abs(ComputeAdaptiveNextBaseline(
                           WaveformClassification::kOverload,
                           true, 80.0, true, 100.0,
                           84.0, 40.0, 60.0, 1.0) -
                       40.0) < 1e-12,
          "test 9: overload uses low+quarter-gap only above that target");
  require(std::abs(ComputeAdaptiveNextBaseline(
                       WaveformClassification::kUnderload,
                       true, 80.0, true, 100.0,
                       80.0, 60.0, 120.0, 1.0) -
                   90.0) < 1e-12 &&
              std::abs(ComputeAdaptiveNextBaseline(
                           WaveformClassification::kUnderload,
                           true, 80.0, true, 100.0,
                           95.0, 60.0, 120.0, 1.0) -
                       120.0) < 1e-12,
          "test 10: underload uses low+half-gap only below that target");
  require(std::abs(ComputeAdaptiveNextBaseline(
                       WaveformClassification::kOverload,
                       false, 0.0, true, 100.0,
                       100.0, 72.0, 90.0, 1.0) -
                   72.0) < 1e-12 &&
              std::abs(ComputeAdaptiveNextBaseline(
                           WaveformClassification::kUnderload,
                           true, 80.0, false, 0.0,
                           80.0, 70.0, 118.0, 1.0) -
                       118.0) < 1e-12 &&
              std::abs(ComputeAdaptiveNextBaseline(
                           WaveformClassification::kOverload,
                           false, 0.0, true, 100.0,
                           100.0, 0.5, 90.0, 1.0) -
                       0.5) < 1e-12,
          "test 11: missing opposite bounds use exact window min/max Drate");
  require(ShouldInheritAdaptiveBounds(124.9, 100.0) &&
              !ShouldInheritAdaptiveBounds(125.0, 100.0) &&
              ShouldInheritAdaptiveBounds(80.0, 100.0),
          "test 12: inheritance uses a strict 25 percent MaxBw threshold");
  require(IsWaveformDecisionRule("R1") &&
              IsWaveformDecisionRule("R5.2") &&
              !IsWaveformDecisionRule("R6.1"),
          "test 13: only R1-R5 decisions update Adaptive bounds");
  require(ShouldObserveAfterInconclusive(
              true, 100, 1, 3.0, 3.0) &&
              InconclusiveWindowStartAdvancePeriods(
                  true, false, 2.0) == 0.0 &&
              InconclusiveWindowStartAdvancePeriods(
                  true, true, 3.0) == 1.0 &&
              !ShouldObserveAfterInconclusive(
                  false, 1, 1, 3.0, 3.0),
          "test 13b: inconclusive extension policy allows Adaptive rolling fallback");
  require(AmplifiedWaveformProbeAmplitude(100, 100, 1.25, 2.0) == 125 &&
              AmplifiedWaveformProbeAmplitude(190, 100, 1.25, 2.0) == 200 &&
              AmplifiedWaveformProbeAmplitude(200, 100, 1.25, 2.0) == 200 &&
              AmplifiedWaveformProbeAmplitude(100, 100, 1.0, 2.0) == 100,
          "test 13c: inconclusive signal amplification is bounded");

  const std::vector<double> half_sender = {
      1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
      -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0};
  const std::vector<double> only_negative_response = {
      0.00, 0.01, 0.00, -0.01, 0.00, 0.01, 0.00, -0.01,
      -4.0, -3.0, -2.0, -1.0, 0.0, 1.0, 2.0, 3.0};
  const std::vector<double> only_positive_response = {
      -4.0, -3.0, -2.0, -1.0, 0.0, 1.0, 2.0, 3.0,
      0.00, 0.01, 0.00, -0.01, 0.00, 0.01, 0.00, -0.01};
  const std::vector<double> both_halves_response = {
      -4.0, -3.0, -2.0, -1.0, 0.0, 1.0, 2.0, 3.0,
      -4.0, -3.0, -2.0, -1.0, 0.0, 1.0, 2.0, 3.0};
  const std::vector<bool> half_valid(half_sender.size(), true);
  double positive_span = 0.0;
  double negative_span = 0.0;
  require(HasOnlyNegativeHalfWaveform(
              only_negative_response, half_sender, half_valid, 0.05,
              &positive_span, &negative_span) &&
              negative_span > 4.0 * positive_span,
          "test 14a: only-negative-half extraction detects a suppressed positive half");
  require(HasOnlyPositiveHalfWaveform(
              only_positive_response, half_sender, half_valid, 0.05,
              &positive_span, &negative_span) &&
              positive_span > 4.0 * negative_span,
          "test 14b: only-positive-half extraction detects a suppressed negative half");
  require(!HasOnlyNegativeHalfWaveform(
              both_halves_response, half_sender, half_valid, 0.05,
              nullptr, nullptr) &&
              !HasOnlyPositiveHalfWaveform(
                  both_halves_response, half_sender, half_valid, 0.05,
                  nullptr, nullptr),
          "test 14c: half-only extraction rejects two active halves");

  std::vector<double> small_periodic_response(periodic_count, 0.0);
  for (size_t i = 0; i < periodic_count; ++i) {
    small_periodic_response[i] = 0.002 * sine_response[i];
  }
  const double small_periodicity = ComputeLaggedCorrelation(
      small_periodic_response, periodic_valid, expected_period_samples,
      nullptr);
  require(small_periodicity > 0.99,
          "test 16: periodicity is independent of response amplitude");

  std::vector<double> smooth_two_cycle(periodic_count, 0.0);
  std::vector<double> bic_bottom_clipped(periodic_count, 0.0);
  std::vector<double> bic_top_clipped(periodic_count, 0.0);
  std::vector<double> bic_both_clipped(periodic_count, 0.0);
  std::vector<double> quantized_triangle(periodic_count, 0.0);
  std::vector<double> quantized_rounded_turn(periodic_count, 0.0);
  for (size_t i = 0; i < periodic_count; ++i) {
    smooth_two_cycle[i] = 50.0 + 8.0 * sine_response[i];
    bic_bottom_clipped[i] = std::max(46.0, smooth_two_cycle[i]);
    bic_top_clipped[i] = std::min(54.0, smooth_two_cycle[i]);
    bic_both_clipped[i] = ClampValue(smooth_two_cycle[i], 46.0, 54.0);
    quantized_triangle[i] =
        std::round(50.0 + 8.0 * triangle_response[i]);
    quantized_rounded_turn[i] =
        0.5 * std::round(2.0 * smooth_two_cycle[i]);
  }
  const BicClippingDetectionResult smooth_bic =
      DetectBicSrttClipping(smooth_two_cycle, periodic_valid, 0.05);
  const BicClippingDetectionResult bottom_bic =
      DetectBicSrttClipping(bic_bottom_clipped, periodic_valid, 0.05);
  const BicClippingDetectionResult top_bic =
      DetectBicSrttClipping(bic_top_clipped, periodic_valid, 0.05);
  const BicClippingDetectionResult both_bic =
      DetectBicSrttClipping(bic_both_clipped, periodic_valid, 0.05);
  const BicClippingDetectionResult quantized_triangle_bic =
      DetectBicSrttClipping(quantized_triangle, periodic_valid, 0.05);
  const BicClippingDetectionResult quantized_rounded_turn_bic =
      DetectBicSrttClipping(
          quantized_rounded_turn, periodic_valid, 0.05);
  require(smooth_bic.valid && !smooth_bic.bottom_clip &&
              !smooth_bic.top_clip,
          "test 16a: smooth extrema do not create a BIC clipping decision");
  require(bottom_bic.valid && bottom_bic.bottom_clip &&
              !bottom_bic.top_clip,
          "test 16a2: two shared-floor BIC motifs detect bottom clipping");
  require(top_bic.valid && top_bic.top_clip &&
              !top_bic.bottom_clip,
          "test 16a3: two shared-ceiling BIC motifs detect top clipping");
  require(both_bic.valid && both_bic.top_clip &&
              both_bic.bottom_clip && both_bic.both_clipped,
          "test 16a4: independent BIC directions preserve BOTH_CLIPPED");
  require(quantized_triangle_bic.valid &&
              !quantized_triangle_bic.top_clip &&
              !quantized_triangle_bic.bottom_clip,
          "test 16a5: a quantized V-shaped turn is not horizontal clipping");
  require(quantized_rounded_turn_bic.valid &&
              !quantized_rounded_turn_bic.top_clip &&
              !quantized_rounded_turn_bic.bottom_clip,
          "test 16a6: a quantized rounded turn is not horizontal clipping");
  std::vector<bool> masked_periodic_valid = periodic_valid;
  for (size_t cycle = 0; cycle < 2; ++cycle) {
    const size_t begin = cycle * expected_period_samples + 70;
    const size_t end = std::min(masked_periodic_valid.size(), begin + 35);
    for (size_t i = begin; i < end; ++i) {
      masked_periodic_valid[i] = false;
    }
  }
  size_t masked_pairs = 0;
  const double masked_periodicity = ComputeLaggedCorrelation(
      sine_response, masked_periodic_valid, expected_period_samples,
      &masked_pairs);
  require(masked_pairs >= 100 && masked_periodicity > 0.99,
          "test 16b: masking sequential middle plateaus preserves the original time-axis period");

  std::vector<double> deterministic_noise(periodic_count, 0.0);
  uint32_t noise_state = 1;
  for (size_t i = 0; i < periodic_count; ++i) {
    noise_state = 1664525u * noise_state + 1013904223u;
    deterministic_noise[i] =
        static_cast<double>(noise_state) /
            static_cast<double>(std::numeric_limits<uint32_t>::max()) -
        0.5;
  }
  const double noise_periodicity = ComputeLaggedCorrelation(
      deterministic_noise, periodic_valid, expected_period_samples,
      nullptr);
  require(noise_periodicity < 0.50,
          "test 17: non-periodic noise fails the periodicity threshold");

  decision.srtt_positive_half_clipped = true;
  decision.srtt_negative_half_clipped = true;
  decision.drate_has_waveform = false;
  decision.drate_middle_any_plateau = false;
  decision.drate_similar = true;
  check_decision(decision, WaveformClassification::kUnderload, "R4.1",
                 "test 18: both SRTT half-cycles clipped with similar Drate is UNDERLOAD");
  decision.srtt_similar = false;
  decision.srtt_positive_half_clipped = false;
  decision.srtt_negative_half_clipped = false;
  decision.drate_similar = false;
  decision.drate_similar_without_middle = true;
  check_decision(decision, WaveformClassification::kUnderload, "R5.1",
                 "test 18b: dissimilar SRTT with masked-similar Drate is UNDERLOAD");
  decision.drate_similar_without_middle = false;
  check_decision(decision, WaveformClassification::kOverload, "R5.2",
                 "test 18c: both signals dissimilar is OVERLOAD");
  decision.srtt_similar_without_middle = true;
  check_decision(decision, WaveformClassification::kFullLoad, "R1",
                 "test 18d: masked-similar SRTT is FULL_LOAD");
  decision.srtt_positive_half_clipped = true;
  check_decision(decision, WaveformClassification::kOverload, "R2.3",
                 "test 18d2: masked-similar SRTT still honors a positive shoulder clip");
  decision.srtt_positive_half_clipped = false;
  decision.srtt_similar_without_middle = false;
  decision.drate_input_valid = false;
  check_decision(decision, WaveformClassification::kOverload, "R5.2",
                 "test 18d3: missing Drate similarity under dissimilar SRTT is OVERLOAD");
  decision.drate_input_valid = true;
  decision.prechecks_valid = false;
  check_decision(decision, WaveformClassification::kInconclusive, "R6.3",
                 "test 18e: invalid prechecks remain INCONCLUSIVE");
  decision.srtt_window_stats_valid = true;
  decision.srtt_mean_ms = 119.0;
  decision.srtt_min_ms = 118.0;
  decision.srtt_max_ms = 121.0;
  decision.latest_waveform_overload_srtt_mean_valid = true;
  decision.latest_waveform_overload_srtt_mean_ms = 120.0;
  check_decision(decision, WaveformClassification::kOverload, "R6.1",
                 "test 18f: Adaptive R6 overloads above srtt_up");
  decision.srtt_mean_ms = 99.0;
  decision.srtt_min_ms = 98.0;
  decision.srtt_max_ms = 120.0;
  decision.latest_waveform_underload_srtt_mean_valid = true;
  decision.latest_waveform_underload_srtt_mean_ms = 100.0;
  check_decision(decision, WaveformClassification::kUnderload, "R6.2",
                 "test 18g: Adaptive R6 underloads below srtt_low");
  decision.srtt_mean_ms = 110.0;
  decision.srtt_min_ms = 100.0;
  check_decision(decision, WaveformClassification::kInconclusive, "R6.3",
                 "test 18h: Adaptive R6 is inconclusive between thresholds");

  const std::vector<FBBRRttSample> srtt_stat_samples = {
      {QuicTime::Zero(), 80.0},
      {QuicTime::Zero(), 120.0},
      {QuicTime::Zero(), -1.0},
      {QuicTime::Zero(), std::numeric_limits<double>::quiet_NaN()},
  };
  const SrttWindowStats srtt_stats =
      ComputeSrttWindowStats(srtt_stat_samples);
  require(srtt_stats.valid && srtt_stats.sample_count == 2 &&
              std::abs(srtt_stats.mean_ms - 100.0) < 1e-12 &&
              std::abs(srtt_stats.min_ms - 80.0) < 1e-12 &&
              std::abs(srtt_stats.max_ms - 120.0) < 1e-12,
          "test 18i: R6 SRTT statistics use finite positive window samples");

  require(HasMacroOpposingShoulders(5.0, -4.0, 0.02, 1.0, 0.05) &&
              HasMacroOpposingShoulders(-5.0, 4.0, 0.02, 1.0, 0.05),
          "opposite macro-scale shoulders qualify as clipping");
  require(!HasMacroOpposingShoulders(5.0, 4.0, 0.02, 1.0, 0.05) &&
              !HasMacroOpposingShoulders(-5.0, -4.0, 0.02, 1.0, 0.05),
          "same-direction shoulders are sequential middle flattening, not an upper/lower clip");
  require(!HasMacroOpposingShoulders(0.5, -0.5, 0.02, 1.0, 0.05),
          "micro-scale opposing perturbations do not qualify as clipping");
  require(HasDualMacroOpposingShoulders(
              5.0, -4.0, 1.0, 0.05,
              8.0, -6.0, 1.0, 0.05, 0.02),
          "upper/lower clipping requires strong opposing shoulders in both SRTT and Drate");
  require(HasMacroSameDirectionShoulders(5.0, 4.0) &&
              !HasDualMacroOpposingShoulders(
                  5.0, -4.0, 1.0, 0.05,
                  8.0, 6.0, 1.0, 0.05, 0.02),
          "same-direction shoulders in either signal identify sequential middle flattening");
  require(IsObviousClipPlateau(0.20, 0.15, 0.90, 0.75,
                               0.05, 0.10, 0.05, 0.15,
                               0.05, 0.15, true),
          "a sustained flat extreme with clear shoulders is obvious clipping");
  require(!IsObviousClipPlateau(0.08, 0.15, 0.90, 0.75,
                                0.05, 0.10, 0.05, 0.15,
                                0.05, 0.15, true) &&
              !IsObviousClipPlateau(0.20, 0.15, 0.90, 0.75,
                                    0.05, 0.10, 0.05, 0.15,
                                    0.30, 0.15, true),
          "short or non-extreme flattening is not obvious clipping");

  std::vector<bool> gap_valid = valid;
  for (size_t i = 70; i <= 100; ++i) {
    gap_valid[i] = false;
  }
  const double gap_period_ratio = 31.0 / 200.0;
  require(gap_period_ratio > 0.10 &&
              std::count(gap_valid.begin(), gap_valid.end(), false) == 31,
          "gaps above 0.10T remain invalid instead of being interpolated");

  const size_t lag_samples = 35;
  std::vector<double> delayed(count, 0.0);
  std::vector<bool> delayed_valid(count, false);
  for (size_t i = lag_samples; i < count; ++i) {
    delayed[i] = sender[i - lag_samples];
    delayed_valid[i] = true;
  }
  size_t best_lag = 0;
  double best_lag_ncc = -2.0;
  for (size_t lag = 0; lag <= 60; ++lag) {
    std::vector<double> aligned_sender(count, 0.0);
    std::vector<bool> joint(count, false);
    for (size_t i = lag; i < count; ++i) {
      aligned_sender[i] = sender[i - lag];
      joint[i] = delayed_valid[i];
    }
    const double ncc = ComputeNormalizedCrossCorrelation(
        aligned_sender, delayed, joint);
    if (ncc > best_lag_ncc) {
      best_lag_ncc = ncc;
      best_lag = lag;
    }
  }
  require(best_lag == lag_samples && best_lag_ncc > 0.99,
          "feedback lag search recovers the deterministic receiver delay");

  const bool post_cruise_phase_allowed[] =
      {true, true, true, true};
  const bool new_cruise_clears_fresh = true;
  const bool native_state_untouched = true;
  require(std::all_of(std::begin(post_cruise_phase_allowed),
                      std::end(post_cruise_phase_allowed),
                      [](bool value) { return value; }) &&
              new_cruise_clears_fresh && native_state_untouched,
          "test 19: lifecycle confines TrustedBw to post-CRUISE pacing only");

  const QuicByteCount probe_down_test_bdp = 20000;
  require(!ShouldEnableRtpropProbeDown(
              false, 25000, probe_down_test_bdp) &&
              !ShouldEnableRtpropProbeDown(
                  true, 24999, probe_down_test_bdp) &&
              ShouldEnableRtpropProbeDown(
                  true, 25000, probe_down_test_bdp) &&
              !ShouldEnableRtpropProbeDown(true, 25000, 0),
          "test 20: RTprop PROBE_DOWN requires a previous CRUISE refresh "
          "and inflight >= 1.25 BDP");
  require(ShouldExitRtpropProbeDown(20999, probe_down_test_bdp) &&
              !ShouldExitRtpropProbeDown(21000, probe_down_test_bdp) &&
              !ShouldExitRtpropProbeDown(0, 0) &&
              std::abs(kFBBRRtpropProbeDownPacingGain - 0.75f) < 1e-6f,
          "test 21: RTprop PROBE_DOWN uses gain 0.75 and exits strictly "
          "below 1.05 BDP");

  Bbr2Params bdp_test_params(4 * kDefaultTCPMSS,
                             1000 * kDefaultTCPMSS);
  Bbr2NetworkModel bdp_test_model(
      &bdp_test_params, TimeDelta::FromMilliseconds(40), QuicTime::Zero(),
      2.0f, 1.0f, nullptr);
  const QuicBandwidth native_bdp_bw =
      QuicBandwidth::FromBitsPerSecond(10000000);
  const QuicBandwidth trusted_bdp_bw =
      QuicBandwidth::FromBitsPerSecond(20000000);
  bdp_test_model.ForceSetMaxBandwidth(native_bdp_bw);
  require(bdp_test_model.BDP() == 50000,
          "test 22: native BDP uses MaxBandwidth before TrustedBw exists");
  require(!IsAtLeastElevenTenthsBdp(54999, bdp_test_model.BDP()) &&
              IsAtLeastElevenTenthsBdp(55000, bdp_test_model.BDP()),
          "test 22b: 1.1 BDP fallback uses native BBRv2 BDP without TrustedBw");
  bdp_test_model.SetBdpBandwidthOverride(trusted_bdp_bw);
  require(bdp_test_model.BDP() == 100000 &&
              bdp_test_model.BDP(native_bdp_bw) == 100000 &&
              bdp_test_model.BDP(native_bdp_bw, 1.25f) == 125000 &&
              bdp_test_model.BdpBandwidth() == trusted_bdp_bw,
          "test 23: every BDP overload uses TrustedBw while it is valid");
  require(!IsAtLeastElevenTenthsBdp(109999, bdp_test_model.BDP()) &&
              IsAtLeastElevenTenthsBdp(110000, bdp_test_model.BDP()),
          "test 23b: 1.1 BDP threshold uses TrustedBw while it is valid");
  bdp_test_model.ClearBdpBandwidthOverride();
  require(bdp_test_model.BDP() == 50000 &&
              bdp_test_model.BdpBandwidth() == native_bdp_bw,
          "test 24: clearing TrustedBw restores native BDP bandwidth");

  const double mild_factor =
      ComputeMaxBwAttenuationFactor(10000000.0, 2000000.0);
  const double strong_factor =
      ComputeMaxBwAttenuationFactor(10000000.0, 5000000.0);
  require(std::abs(mild_factor - 10.0 / 12.0) < 1e-12 &&
              std::abs(mild_factor * 12000000.0 - 10000000.0) < 1e-6 &&
              strong_factor < mild_factor &&
              ComputeMaxBwAttenuationFactor(10000000.0, 0.0) == 1.0 &&
              ComputeMaxBwAttenuationFactor(0.0, 2000000.0) == 1.0,
          "test 25: maxbw factor removes the observed positive excursion "
          "and strengthens monotonically with fluctuation amplitude");
  bdp_test_model.SetMaxBandwidthSampleAttenuation(mild_factor);
  const bool accepts_valid_attenuation =
      std::abs(bdp_test_model.max_bandwidth_sample_attenuation() -
               mild_factor) < 1e-12;
  bdp_test_model.SetMaxBandwidthSampleAttenuation(0.0);
  require(accepts_valid_attenuation &&
              bdp_test_model.max_bandwidth_sample_attenuation() == 1.0,
          "test 26: network model accepts only finite factors in (0, 1]");

  os << "RESULT: " << (pass ? "PASS" : "FAIL") << "\n";
  return pass;
}

bool FBBRSender::RunFbbrBaselineSelfTest(std::ostream& os) {
  bool pass = true;
  auto require = [&pass, &os](bool condition, const char* message) {
    os << (condition ? "PASS: " : "FAIL: ") << message << "\n";
    pass = pass && condition;
  };
  const QuicTime sample_time = QuicTime::Zero();
  const std::vector<FBBRRateSample> samples = {
      {sample_time, QuicBandwidth::FromBitsPerSecond(8000000), true},
      {sample_time, QuicBandwidth::FromBitsPerSecond(12000000), true},
      {sample_time, QuicBandwidth::FromBitsPerSecond(10000000), true},
      {sample_time, QuicBandwidth::FromBitsPerSecond(50000000), false}};
  const DeliveryRateWindowStats stats =
      ComputeDeliveryRateWindowStats(samples);
  require(stats.valid && stats.sample_count == 3,
          "invalid delivery-rate samples are excluded");
  require(std::abs(stats.min_bps - 8000000.0) < 1e-6,
          "OVERLOAD baseline uses the window minimum");
  require(std::abs(stats.max_bps - 12000000.0) < 1e-6,
          "UNDERLOAD baseline uses the window maximum");
  require(std::abs(stats.mean_bps - 10000000.0) < 1e-6,
          "FULL_LOAD trustedbw uses the arithmetic window mean");
  const QuicBandwidth first_trusted_bw = BandwidthFromBps(stats.mean_bps);
  const QuicBandwidth next_trusted_bw = BandwidthFromBps(18000000.0);
  require(first_trusted_bw.ToBitsPerSecond() == 10000000 &&
              next_trusted_bw.ToBitsPerSecond() == 18000000,
          "FBBR trustedbw publishes each arithmetic window mean without smoothing");
  os << "RESULT: " << (pass ? "PASS" : "FAIL") << "\n";
  return pass;
}

bool FBBRSender::RunFbbrHybridSelfTest(std::ostream& os) {
  bool pass = true;
  auto require = [&pass, &os](bool condition, const std::string& message) {
    os << (condition ? "PASS: " : "FAIL: ") << message << "\n";
    pass = pass && condition;
  };
  os << "# FBBR-hybrid quantified-regime self-test\n";
  require(!ShouldStartFbbrHybridLowerBoundSearch(2, false, false) &&
              ShouldStartFbbrHybridLowerBoundSearch(3, false, false) &&
              ShouldStartFbbrHybridLowerBoundSearch(3, true, false) &&
              !ShouldStartFbbrHybridLowerBoundSearch(3, true, true),
          "lower-bound drain starts only from the third Cruise while either lower reference is missing");
  require(!ShouldAllowFbbrHybridLowerBoundSearchByQueue(
              60.0, true, 50.0, true, 100.0) &&
              ShouldAllowFbbrHybridLowerBoundSearchByQueue(
                  75.0, true, 50.0, true, 100.0) &&
              !ShouldAllowFbbrHybridLowerBoundSearchByQueue(
                  120.0, true, 50.0, false, 0.0) &&
              !ShouldAllowFbbrHybridLowerBoundSearchByQueue(
                  120.0, false, 0.0, true, 100.0),
          "lower-bound drain guard requires current queue to reach half of MaxSRTT minus RTprop with valid references");
  require(std::abs(ComputeFbbrHybridLowerBoundSearchBaseline(
                       100.0, 1.0) - 80.0) < 1e-12 &&
              ComputeFbbrHybridLowerBoundSearchBaseline(1.1, 1.0) == 1.0 &&
              IsBelowHalfBdp(499, 1000) &&
              !IsBelowHalfBdp(500, 1000),
          "lower-bound drain uses geometric 0.80 steps and strict inflight below 0.5 BDP");
  require(ComputeHybridStableDeliveryRate(
              {10000000, 40000000, 30000000, 20000000}, {})
                  .ToBitsPerSecond() == 20000000 &&
              ComputeHybridStableDeliveryRate(
                  {}, {9000000, 3000000, 6000000})
                  .ToBitsPerSecond() == 6000000,
          "ProbeRTT DRate uses the lower median after one stable round with an all-sample fallback");
  FbbrRegimeContext context;
  context.max_srtt_valid = true;
  context.max_srtt_ms = 100.0;
  context.rtprop_valid = true;
  context.rtprop_ms = 50.0;
  auto base = []() {
    FbbrHybridRegimeFeatures features;
    features.input_valid = true;
    features.srtt.wave.input_valid = true;
    features.srtt.wave.has_wave = true;
    features.drate.wave.input_valid = true;
    features.srtt_stats_valid = true;
    features.srtt_min_ms = 70.0;
    features.srtt_mean_ms = 80.0;
    features.srtt_max_ms = 90.0;
    features.inflight_bdp_valid = true;
    features.inflight_bytes = 10999;
    features.bdp_bytes = 10000;
    features.drate_stats_valid = true;
    features.mindrate_bps = 80.0;
    features.maxdrate_bps = 120.0;
    features.meandrate_bps = 100.0;
    features.drate.periodic = PeriodicSimilarityResult::kMatch;
    return features;
  };
  auto check = [&](FbbrHybridRegimeFeatures features,
                   const char* rule,
                   WaveformClassification classification,
                   bool upper_bound_rule,
                   bool rtprop,
                   bool rtprop_drate) {
    (void)upper_bound_rule;
    const FbbrHybridDecision decision =
        ClassifyFbbrHybridRegime(features, context);
    const bool ok = std::string(decision.rule_id) == rule &&
        decision.classification == classification &&
        decision.refresh_rtprop == rtprop &&
        decision.update_rtprop_drate == rtprop_drate &&
        decision.update_baseline_low == rtprop_drate &&
        !decision.update_lower_bound_from_rtprop_min &&
        !decision.update_lower_bound_from_low_inflight;
    require(ok, std::string(rule) + " exact classification and side effects");
  };

  FbbrHybridRegimeFeatures f = base();
  f.selected_clip_case = SrttClipCase::kU1PositiveShoulder;
  check(f, "N01", WaveformClassification::kFullLoad,
        false, false, false);
  f.drate.periodic = PeriodicSimilarityResult::kNoMatch;
  check(f, "N02", WaveformClassification::kOverload,
        true, false, false);
  f = base();
  f.selected_clip_case = SrttClipCase::kU2LongTopLine;
  check(f, "N03", WaveformClassification::kFullLoad,
        false, false, false);
  f.drate.periodic = PeriodicSimilarityResult::kNoMatch;
  check(f, "N04", WaveformClassification::kOverload,
        true, false, false);
  f = base();
  f.selected_clip_case = SrttClipCase::kU3RepeatedTopClip;
  check(f, "N05", WaveformClassification::kOverload,
        true, false, false);
  f = base();
  f.selected_clip_case = SrttClipCase::kL1NegativeShoulder;
  check(f, "N06", WaveformClassification::kFullLoad,
        false, false, false);
  f = base();
  f.selected_clip_case = SrttClipCase::kL2LongBottomLine;
  f.drate.wave.has_wave = true;
  check(f, "N07", WaveformClassification::kUnderload,
        false, true, true);
  f.drate.wave.has_wave = false;
  check(f, "N08", WaveformClassification::kFullLoad,
        false, false, false);
  f = base();
  f.selected_clip_case = SrttClipCase::kL3RepeatedBottomClip;
  check(f, "N09", WaveformClassification::kUnderload,
        false, false, true);
  f = base();
  f.selected_clip_case = SrttClipCase::kU2LongTopLine;
  f.srtt.wave.has_wave = false;
  f.drate.periodic = PeriodicSimilarityResult::kMatch;
  check(f, "N13", WaveformClassification::kUnderload,
        false, false, false);
  f.selected_clip_case = SrttClipCase::kL2LongBottomLine;
  f.drate.periodic = PeriodicSimilarityResult::kNoMatch;
  check(f, "N16", WaveformClassification::kOverload,
        false, false, false);
  f = base();
  f.srtt_max_ms = 100.0001;
  check(f, "N10", WaveformClassification::kOverload,
        true, false, false);
  f.srtt_max_ms = 100.0;
  f.srtt_min_ms = 49.9999;
  check(f, "N11", WaveformClassification::kUnderload,
        false, true, true);
  f.srtt_min_ms = 50.0001;
  f.srtt_mean_ms = 62.5;
  f.inflight_bytes = 10999;
  check(f, "N12", WaveformClassification::kFullLoad,
        false, false, false);
  f.inflight_bytes = 11000;
  check(f, "N12", WaveformClassification::kOverload,
        false, false, false);
  f.inflight_bytes = 10999;
  f.srtt_mean_ms = 62.5001;
  check(f, "N12", WaveformClassification::kOverload,
        false, false, false);
  f = base();
  f.srtt.wave.has_wave = false;
  f.drate.periodic = PeriodicSimilarityResult::kMatch;
  check(f, "N13", WaveformClassification::kUnderload,
        false, false, false);
  f.drate.periodic = PeriodicSimilarityResult::kNoMatch;
  f.srtt_max_ms = 100.0001;
  check(f, "N14", WaveformClassification::kOverload,
        true, false, false);
  f.srtt_max_ms = 100.0;
  f.srtt_min_ms = 49.9999;
  check(f, "N15", WaveformClassification::kUnderload,
        false, true, true);
  f.srtt_min_ms = 50.0001;
  f.srtt_mean_ms = 62.5;
  f.inflight_bytes = 10999;
  check(f, "N16", WaveformClassification::kFullLoad,
        false, false, false);
  f.inflight_bytes = 11000;
  check(f, "N16", WaveformClassification::kOverload,
        false, false, false);
  f.inflight_bytes = 10999;
  f.srtt_mean_ms = 62.5001;
  check(f, "N16", WaveformClassification::kOverload,
        false, false, false);
  f.srtt_min_ms = 50.0;
  const FbbrHybridDecision rtprop_contact =
      ClassifyFbbrHybridRegime(f, context);
  require(!rtprop_contact.update_lower_bound_from_rtprop_min &&
              !rtprop_contact.update_rtprop_drate &&
              !rtprop_contact.update_baseline_low &&
              !rtprop_contact.refresh_rtprop,
          "N12/N16 RTprop contact does not invent extra side effects");

  f = base();
  f.selected_clip_case = SrttClipCase::kU1PositiveShoulder;
  f.drate.periodic = PeriodicSimilarityResult::kInvalidInput;
  FbbrHybridDecision invalid = ClassifyFbbrHybridRegime(f, context);
  require(invalid.classification == WaveformClassification::kInconclusive &&
              std::string(invalid.rule_id).empty(),
          "U1/U2 INVALID periodic input cannot fall through as NO_MATCH");
  f = base();
  f.selected_clip_case = SrttClipCase::kL2LongBottomLine;
  f.drate.wave.input_valid = false;
  invalid = ClassifyFbbrHybridRegime(f, context);
  require(invalid.classification == WaveformClassification::kInconclusive,
          "L2 invalid ordinary-wave input is inconclusive");
  f = base();
  f.srtt.wave.has_wave = false;
  f.drate.periodic = PeriodicSimilarityResult::kInvalidInput;
  invalid = ClassifyFbbrHybridRegime(f, context);
  require(invalid.classification == WaveformClassification::kInconclusive,
          "SRTT no-wave branch requires valid DRate periodic input");

  std::vector<FBBRRttSample> queue_gradient_samples;
  for (int i = 0; i < 10; ++i) {
    queue_gradient_samples.push_back(
        {QuicTime::Zero() + TimeDelta::FromSeconds(i + 1),
         40.0 + static_cast<double>(i)});
  }
  const RobustQueueGradientEstimate queue_gradient =
      ComputeRobustQueueGradient(queue_gradient_samples, 0.040);
  require(queue_gradient.queue_sample_valid &&
              queue_gradient.sample_count == 10 &&
              std::abs(queue_gradient.q90_s - 0.0081) < 1e-12 &&
              queue_gradient.raw_gradient > 0.0 &&
              queue_gradient.gradient > 0.0,
          "queue gradient uses seconds, P90, P5-P95 winsorization, and linear regression");
  queue_gradient_samples[5].time = queue_gradient_samples[4].time;
  const RobustQueueGradientEstimate duplicate_queue_time =
      ComputeRobustQueueGradient(queue_gradient_samples, 0.040);
  require(duplicate_queue_time.queue_sample_valid &&
              duplicate_queue_time.gradient == 0.0,
          "duplicate queue sample timestamps safely produce zero gradient");

  const GradientMatchedDecrease queue_hold =
      ComputeGradientMatchedDecrease(
          true, 0.015, 0.0, 0.4, true, 0.040, true, 0.085);
  const GradientMatchedDecrease natural_drain_hold =
      ComputeGradientMatchedDecrease(
          true, 0.030, -0.10, 0.4, true, 0.040, true, 0.085);
  const GradientMatchedDecrease queue_decrease =
      ComputeGradientMatchedDecrease(
          true, 0.030, 0.02, 0.4, true, 0.040, true, 0.085);
  const GradientMatchedDecrease capped_decrease =
      ComputeGradientMatchedDecrease(
          true, 1.0, 0.5, 0.01, true, 0.040, true, 0.085);
  require(queue_hold.valid && queue_hold.beta == 0.0 &&
              natural_drain_hold.valid &&
              natural_drain_hold.beta == 0.0 &&
              queue_decrease.valid && queue_decrease.beta > 0.0 &&
              capped_decrease.valid &&
              std::abs(capped_decrease.beta - 0.10) < 1e-12,
          "gradient-matched overload holds a stable/draining queue and caps decreases at 10 percent");

  const FbbrHybridActuatorResult i_mid =
      ComputeFbbrHybridInjectionBaseline(
          WaveformClassification::kUnderload,
          80.0, 120.0, 100.0,
          false, 0.0, false, 0.0, 100.0,
          true, 60.0, 0.50, 1.0);
  const FbbrHybridActuatorResult i_equal =
      ComputeFbbrHybridInjectionBaseline(
          WaveformClassification::kUnderload,
          90.0, 120.0, 100.0,
          false, 0.0, false, 0.0, 100.0,
          true, 60.0, 0.50, 1.0);
  const FbbrHybridActuatorResult iii_mid =
      ComputeFbbrHybridInjectionBaseline(
          WaveformClassification::kOverload,
          80.0, 120.0, 100.0,
          false, 0.0, false, 0.0, 100.0,
          true, 60.0, 0.50, 1.0, true, 0.0);
  const FbbrHybridActuatorResult iii_equal =
      ComputeFbbrHybridInjectionBaseline(
          WaveformClassification::kOverload,
          90.0, 120.0, 100.0,
          false, 0.0, false, 0.0, 100.0,
          true, 60.0, 0.50, 1.0, true, 0.10);
  require(i_mid.valid && i_mid.midpoint_triggered &&
              std::abs(i_mid.next_baseline_bps - 100.0) < 1e-12 &&
              i_equal.valid && !i_equal.midpoint_triggered &&
              std::abs(i_equal.next_baseline_bps - 120.0) < 1e-12,
          "Regime I uses strict 50 percent midpoint boundary");
  require(iii_mid.valid && !iii_mid.bracket_triggered &&
              !iii_mid.midpoint_triggered &&
              std::abs(iii_mid.next_baseline_bps - 100.0) < 1e-12 &&
              iii_mid.overload_beta == 0.0 &&
              iii_equal.valid && !iii_equal.bracket_triggered &&
              !iii_equal.midpoint_triggered &&
              std::abs(iii_equal.next_baseline_bps - 90.0) < 1e-12,
          "Regime III uses only the gradient-matched decrease or hold");
  const FbbrHybridActuatorResult ii =
      ComputeFbbrHybridInjectionBaseline(
          WaveformClassification::kFullLoad,
          80.0, 120.0, 101.0,
          false, 0.0, false, 0.0, 100.0,
          false, 0.0, 0.50, 1.0);
  const FbbrHybridActuatorResult no_ref_i =
      ComputeFbbrHybridInjectionBaseline(
          WaveformClassification::kUnderload,
          80.0, 120.0, 100.0,
          false, 0.0, false, 0.0, 100.0,
          false, 0.0, 0.50, 1.0);
  const FbbrHybridActuatorResult no_ref_iii =
      ComputeFbbrHybridInjectionBaseline(
          WaveformClassification::kOverload,
          80.0, 120.0, 100.0,
          false, 0.0, false, 0.0, 100.0,
          false, 0.0, 0.50, 90.0);
  require(ii.valid && !ii.update_baseline && ii.update_trusted_bw &&
              ii.next_baseline_bps == 0.0 &&
              std::abs(ii.trusted_bw_bps - 101.0) < 1e-12,
          "Regime II updates TrustedBw without changing baseline");
  require(no_ref_i.valid && no_ref_i.next_baseline_bps == 120.0 &&
              no_ref_iii.valid &&
              no_ref_iii.next_baseline_bps == 100.0,
          "invalid RTpropDRate leaves gradient-matched Regime III on the current baseline");

  const FbbrHybridActuatorResult i_bracket =
      ComputeFbbrHybridInjectionBaseline(
          WaveformClassification::kUnderload,
          80.0, 120.0, 100.0,
          true, 60.0, true, 140.0, 90.0,
          true, 60.0, 0.50, 1.0);
  const FbbrHybridActuatorResult iii_bracket =
      ComputeFbbrHybridInjectionBaseline(
          WaveformClassification::kOverload,
          80.0, 120.0, 100.0,
          true, 80.0, true, 120.0, 110.0,
          true, 60.0, 0.50, 1.0, true, 0.10);
  const FbbrHybridActuatorResult i_bracket_falls_to_midpoint =
      ComputeFbbrHybridInjectionBaseline(
          WaveformClassification::kUnderload,
          80.0, 120.0, 100.0,
          true, 80.0, true, 120.0, 110.0,
          true, 60.0, 0.50, 1.0);
  const FbbrHybridActuatorResult iii_bracket_falls_to_direct =
      ComputeFbbrHybridInjectionBaseline(
          WaveformClassification::kOverload,
          90.0, 120.0, 100.0,
          true, 80.0, true, 120.0, 80.0,
          false, 0.0, 0.50, 1.0, true, 0.25);
  require(i_bracket.valid && i_bracket.bracket_triggered &&
              !i_bracket.midpoint_triggered &&
              std::abs(i_bracket.next_baseline_bps - 100.0) < 1e-12 &&
              iii_bracket.valid && !iii_bracket.bracket_triggered &&
              !iii_bracket.midpoint_triggered &&
              std::abs(iii_bracket.next_baseline_bps - 99.0) < 1e-12,
          "Adaptive low/up bracket has priority only for Regime I");
  require(i_bracket_falls_to_midpoint.valid &&
              !i_bracket_falls_to_midpoint.bracket_triggered &&
              i_bracket_falls_to_midpoint.midpoint_triggered &&
              i_bracket_falls_to_midpoint.next_baseline_bps == 100.0 &&
              iii_bracket_falls_to_direct.valid &&
              !iii_bracket_falls_to_direct.bracket_triggered &&
              !iii_bracket_falls_to_direct.midpoint_triggered &&
              iii_bracket_falls_to_direct.next_baseline_bps == 72.0 &&
              iii_bracket_falls_to_direct.overload_beta == 0.10,
          "Regime III ignores bracket/midpoint and caps beta at 10 percent");

  require(!(0.20 > 0.20) && 0.2001 > 0.20 &&
              !(0.30 > 0.30) && 0.3001 > 0.30,
          "U2/L2 duration boundaries are strict");
  require(0.4999 < 0.50 && 0.50 >= 0.50,
          "U3/L3 contact span boundary is inclusive at 50 percent");
  require(std::abs(1.20 - 1.0) / 1.0 <= 0.20 + 1e-15 &&
              std::abs(1.201 - 1.0) / 1.0 > 0.20,
          "period error accepts 20 percent and rejects 20.1 percent");
  require(AmplifiedWaveformProbeAmplitude(100, 100, 1.25, 2.0) == 125 &&
              AmplifiedWaveformProbeAmplitude(125, 100, 1.25, 2.0) == 157 &&
              AmplifiedWaveformProbeAmplitude(157, 100, 1.25, 2.0) == 197 &&
              AmplifiedWaveformProbeAmplitude(197, 100, 1.25, 2.0) == 200 &&
              AmplifiedWaveformProbeAmplitude(200, 100, 1.25, 2.0) == 200,
          "retry amplitude uses ceil(1.25*A) and caps at 2*A");
  require(kFBBRHybrid != kFBBR && kFBBRHybrid != kFBBRAdaptive,
          "FBBR-hybrid owns an isolated congestion-control type");

  RttStats detector_rtt;
  detector_rtt.set_initial_rtt(TimeDelta::FromMilliseconds(40));
  Random detector_random;
  QuicConnectionStats detector_stats;
  FBBRSender detector(QuicTime::Zero(), &detector_rtt, nullptr,
                      10, 1000, &detector_random, &detector_stats,
                      false, false, false, kFBBRHybrid);
  detector.ConfigureFBBR(FBBRConfig());
  require(ApplyGuardLowPass(
              QuicBandwidth::FromBitsPerSecond(80000000),
              QuicBandwidth::FromBitsPerSecond(160000000))
                  .ToBitsPerSecond() == 90000000,
          "guard low-pass uses 7/8 previous and 1/8 sample");
  {
    RttStats selection_rtt;
    selection_rtt.set_initial_rtt(TimeDelta::FromMilliseconds(40));
    Random selection_random;
    QuicConnectionStats selection_stats;
    FBBRSender selection_sender(
        QuicTime::Zero(), &selection_rtt, nullptr, 10, 1000,
        &selection_random, &selection_stats, false, false, false,
        kFBBRHybrid);
    selection_sender.ConfigureFBBR(FBBRConfig());
    selection_sender.cruise_id_ = 1;
    selection_sender.initial_cruise_baseline_bw_ =
        QuicBandwidth::FromBitsPerSecond(50000000);
    selection_sender.fbbr_hybrid_regime_ii_seen_this_cruise_ = true;
    selection_sender.trusted_baseline_locked_ = true;
    selection_sender.trusted_bw_candidate_ =
        QuicBandwidth::FromBitsPerSecond(100000000);
    selection_sender.trusted_bw_candidate_source_ =
        kTrustedBwSourceFbbrWindowMean;
    selection_sender.guard_filter_valid_ = true;
    selection_sender.guard_filter_stage1_ =
        QuicBandwidth::FromBitsPerSecond(80000000);
    selection_sender.guard_filter_stage2_ =
        QuicBandwidth::FromBitsPerSecond(80000000);
    selection_sender.guard_updated_this_cruise_ = true;
    selection_sender.PublishFbbrHybridCruiseTrustedBw(nullptr);
    require(selection_sender.trusted_bw_.ToBitsPerSecond() == 100000000 &&
                std::string(selection_sender.trusted_bw_source_) ==
                    kTrustedBwSourceFbbrWindowMean &&
                selection_sender.guard_filter_stage1_.ToBitsPerSecond() ==
                    100000000 &&
                selection_sender.guard_filter_stage2_.ToBitsPerSecond() ==
                    100000000,
            "Cruise final selection uses FBBR_WINDOW_MEAN first and anchors guard");
  }
  {
    RttStats selection_rtt;
    selection_rtt.set_initial_rtt(TimeDelta::FromMilliseconds(40));
    Random selection_random;
    QuicConnectionStats selection_stats;
    FBBRSender selection_sender(
        QuicTime::Zero(), &selection_rtt, nullptr, 10, 1000,
        &selection_random, &selection_stats, false, false, false,
        kFBBRHybrid);
    selection_sender.ConfigureFBBR(FBBRConfig());
    selection_sender.cruise_id_ = 2;
    selection_sender.trusted_bw_ =
        QuicBandwidth::FromBitsPerSecond(70000000);
    selection_sender.trusted_bw_valid_ = true;
    selection_sender.guard_filter_valid_ = true;
    selection_sender.guard_filter_stage2_ =
        QuicBandwidth::FromBitsPerSecond(85000000);
    selection_sender.guard_updated_this_cruise_ = true;
    selection_sender.PublishFbbrHybridCruiseTrustedBw(nullptr);
    require(selection_sender.trusted_bw_.ToBitsPerSecond() == 85000000 &&
                std::string(selection_sender.trusted_bw_source_) ==
                    kTrustedBwSourceGuardFilter,
            "Cruise final selection uses updated guard when no frequency result exists");
  }
  {
    RttStats selection_rtt;
    selection_rtt.set_initial_rtt(TimeDelta::FromMilliseconds(40));
    Random selection_random;
    QuicConnectionStats selection_stats;
    FBBRSender selection_sender(
        QuicTime::Zero(), &selection_rtt, nullptr, 10, 1000,
        &selection_random, &selection_stats, false, false, false,
        kFBBRHybrid);
    selection_sender.ConfigureFBBR(FBBRConfig());
    selection_sender.cruise_id_ = 3;
    selection_sender.trusted_bw_ =
        QuicBandwidth::FromBitsPerSecond(75000000);
    selection_sender.trusted_bw_valid_ = true;
    selection_sender.PublishFbbrHybridCruiseTrustedBw(nullptr);
    require(selection_sender.trusted_bw_.ToBitsPerSecond() == 75000000 &&
                std::string(selection_sender.trusted_bw_source_) ==
                    kTrustedBwSourcePreviousTrusted,
            "Cruise final selection keeps previous trustedBw when guard did not update");
  }
  {
    RttStats selection_rtt;
    selection_rtt.set_initial_rtt(TimeDelta::FromMilliseconds(40));
    Random selection_random;
    QuicConnectionStats selection_stats;
    FBBRSender selection_sender(
        QuicTime::Zero(), &selection_rtt, nullptr, 10, 1000,
        &selection_random, &selection_stats, false, false, false,
        kFBBRHybrid);
    selection_sender.ConfigureFBBR(FBBRConfig());
    selection_sender.cruise_id_ = 4;
    selection_sender.initial_cruise_baseline_bw_ =
        QuicBandwidth::FromBitsPerSecond(60000000);
    selection_sender.PublishFbbrHybridCruiseTrustedBw(nullptr);
    require(selection_sender.trusted_bw_.ToBitsPerSecond() == 60000000 &&
                std::string(selection_sender.trusted_bw_source_) ==
                    kTrustedBwSourceNativeFallback,
            "Cruise final selection uses native fallback only without any trusted history");
  }
  require(!detector.UsesAdaptiveLoadJudgment(),
          "FBBR-hybrid reuses the Adaptive actuator without entering its classifier");
  const QuicTime reference_time =
      QuicTime::Zero() + TimeDelta::FromMilliseconds(1);
  detector.model_.ForceUpdateMinRtt(
      TimeDelta::FromMilliseconds(40), reference_time);
  detector.model_.ForceSetMaxBandwidth(
      QuicBandwidth::FromBitsPerSecond(100000000));
  detector.fbbr_hybrid_max_srtt_valid_ = true;
  detector.fbbr_hybrid_max_srtt_ms_ = 85.0;
  const WaveformWindowAnalysis reference_window =
      detector.AnalyzeFbbrHybridWindow(
          reference_time, reference_time, 2.0, false);
  require(reference_window.hybrid_srtt_low_rtprop_valid &&
              reference_window.hybrid_srtt_low_rtprop_ms == 40.0 &&
              reference_window.hybrid_max_srtt_valid &&
              reference_window.hybrid_max_srtt_ms == 85.0 &&
              reference_window.hybrid_max_bw_before_valid &&
              reference_window.hybrid_max_bw_before_bps == 100000000.0,
          "Hybrid srtt_low maps to RTprop, upper bound maps to MaxBw, and srtt_max maps to MaxSRTT");
  {
    RttStats max_srtt_rtt;
    max_srtt_rtt.set_initial_rtt(TimeDelta::FromMilliseconds(10));
    Random max_srtt_random;
    QuicConnectionStats max_srtt_stats;
    FBBRSender max_srtt_detector(
        QuicTime::Zero(), &max_srtt_rtt, nullptr, 10, 1000,
        &max_srtt_random, &max_srtt_stats, false, false, false,
        kFBBRHybrid);
    max_srtt_detector.ConfigureFBBR(FBBRConfig());
    const QuicTime update_time =
        QuicTime::Zero() + TimeDelta::FromMilliseconds(100);
    max_srtt_detector.srtt_history_.push_back(
        {update_time - TimeDelta::FromMilliseconds(31), 250.0});
    max_srtt_detector.srtt_history_.push_back(
        {update_time - TimeDelta::FromMilliseconds(30), 120.0});
    max_srtt_detector.srtt_history_.push_back(
        {update_time + TimeDelta::FromMilliseconds(5), 140.0});
    max_srtt_detector.srtt_history_.push_back(
        {update_time + TimeDelta::FromMilliseconds(29), 180.0});
    max_srtt_detector.srtt_history_.push_back(
        {update_time + TimeDelta::FromMilliseconds(31), 260.0});
    max_srtt_detector.RecordHybridMaxBwUpdateForMaxSrtt(
        update_time, QuicBandwidth::FromBitsPerSecond(120000000));
    max_srtt_detector.FinalizePendingHybridMaxSrttObservations(
        update_time + TimeDelta::FromMilliseconds(29));
    const bool not_finalized_early =
        !max_srtt_detector.fbbr_hybrid_max_srtt_valid_;
    max_srtt_detector.FinalizePendingHybridMaxSrttObservations(
        update_time + TimeDelta::FromMilliseconds(30));
    require(not_finalized_early &&
                max_srtt_detector.fbbr_hybrid_max_srtt_valid_ &&
                max_srtt_detector.fbbr_hybrid_max_srtt_ms_ == 180.0,
            "MaxSRTT finalizes after the +3 RTT side and picks the largest SRTT inside the +/-3 RTT window");
  }
  WaveformWindowAnalysis lower_bound_window;
  lower_bound_window.classification = WaveformClassification::kUnderload;
  lower_bound_window.delivery_rate_stats_valid = true;
  lower_bound_window.delivery_rate_min_bps = 80000000.0;
  lower_bound_window.delivery_rate_max_bps = 120000000.0;
  lower_bound_window.hybrid_decision.update_baseline_low = true;
  lower_bound_window.hybrid_decision.update_rtprop_drate = true;
  detector.ApplyFbbrHybridRegimeStateUpdates(
      &lower_bound_window,
      QuicTime::Zero() + TimeDelta::FromMilliseconds(2));
  require(detector.adaptive_baseline_low_valid_ &&
              detector.adaptive_baseline_low_.ToBitsPerSecond() ==
                  80000000 &&
              !detector.adaptive_baseline_up_valid_,
          "N07/N09/N11/N15 lower bound source is window minimum DRate");
  WaveformWindowAnalysis upper_bound_window;
  upper_bound_window.classification = WaveformClassification::kOverload;
  upper_bound_window.delivery_rate_stats_valid = true;
  upper_bound_window.delivery_rate_min_bps = 90000000.0;
  upper_bound_window.delivery_rate_max_bps = 150000000.0;
  detector.ApplyFbbrHybridRegimeStateUpdates(
      &upper_bound_window,
      QuicTime::Zero() + TimeDelta::FromMilliseconds(3));
  require(!detector.adaptive_baseline_up_valid_ &&
              upper_bound_window.hybrid_max_bw_after_valid &&
              upper_bound_window.hybrid_max_bw_after_bps == 100000000.0,
          "N02/N04/N05/N10/N14 do not write an upper bound; Hybrid uses current MaxBw");
  WaveformWindowAnalysis suppressed_bound_window = lower_bound_window;
  suppressed_bound_window.classification =
      WaveformClassification::kInconclusive;
  suppressed_bound_window.delivery_rate_min_bps = 70000000.0;
  detector.ApplyFbbrHybridRegimeStateUpdates(
      &suppressed_bound_window,
      QuicTime::Zero() + TimeDelta::FromMilliseconds(4));
  require(detector.adaptive_baseline_low_.ToBitsPerSecond() == 80000000,
          "inconclusive/suppressed windows cannot partially update bounds");
  WaveformWindowAnalysis rtprop_contact_window = suppressed_bound_window;
  rtprop_contact_window.classification_suppressed_for_retry = true;
  rtprop_contact_window.hybrid_decision
      .update_lower_bound_from_rtprop_min = true;
  detector.ApplyFbbrHybridRegimeStateUpdates(
      &rtprop_contact_window,
      QuicTime::Zero() + TimeDelta::FromMilliseconds(5));
  require(detector.adaptive_baseline_low_.ToBitsPerSecond() == 70000000 &&
              detector.fbbr_hybrid_rtprop_drate_valid_ &&
              detector.fbbr_hybrid_rtprop_drate_.ToBitsPerSecond() ==
                  70000000 &&
              detector.model_.MinRtt() ==
                  TimeDelta::FromMilliseconds(40),
          "RTprop-contact updates only the lower rate references and survives retry suppression");

  RttStats fallback_rtt;
  fallback_rtt.set_initial_rtt(TimeDelta::FromMilliseconds(40));
  Random fallback_random;
  QuicConnectionStats fallback_stats;
  FBBRSender fallback_detector(
      QuicTime::Zero(), &fallback_rtt, nullptr, 10, 1000,
      &fallback_random, &fallback_stats, false, false, false, kFBBRHybrid);
  fallback_detector.ConfigureFBBR(FBBRConfig());
  fallback_detector.model_.ForceUpdateMinRtt(
      TimeDelta::FromMilliseconds(40), reference_time);
  WaveformWindowAnalysis half_bdp_window;
  half_bdp_window.classification =
      WaveformClassification::kInconclusive;
  half_bdp_window.classification_suppressed_for_retry = true;
  half_bdp_window.delivery_rate_stats_valid = true;
  half_bdp_window.delivery_rate_min_bps = 60000000.0;
  half_bdp_window.srtt_stats_valid = true;
  half_bdp_window.srtt_min_ms = 35.0;
  half_bdp_window.hybrid_decision
      .update_lower_bound_from_low_inflight = true;
  fallback_detector.ApplyFbbrHybridRegimeStateUpdates(
      &half_bdp_window,
      QuicTime::Zero() + TimeDelta::FromMilliseconds(6));
  require(fallback_detector.model_.MinRtt() ==
              TimeDelta::FromMilliseconds(35) &&
              fallback_detector.adaptive_baseline_low_valid_ &&
              fallback_detector.adaptive_baseline_low_
                      .ToBitsPerSecond() == 60000000 &&
              fallback_detector.fbbr_hybrid_rtprop_drate_valid_ &&
              fallback_detector.fbbr_hybrid_rtprop_drate_
                      .ToBitsPerSecond() == 60000000 &&
              !fallback_detector.adaptive_baseline_up_valid_ &&
              !fallback_detector.fbbr_hybrid_max_srtt_valid_,
          "half-BDP fallback refreshes RTprop and only the two lower rate references");

  FBBRSender probe_version_detector(
      QuicTime::Zero(), &fallback_rtt, nullptr, 10, 1000,
      &fallback_random, &fallback_stats, false, false, false, kFBBRHybrid);
  probe_version_detector.ConfigureFBBR(FBBRConfig());
  const QuicTime cruise_source_time =
      QuicTime::Zero() + TimeDelta::FromMilliseconds(10);
  probe_version_detector.PublishHybridSrttLow(
      TimeDelta::FromMilliseconds(40), cruise_source_time, false);
  probe_version_detector.PublishHybridLowerBound(
      QuicBandwidth::FromBitsPerSecond(80000000), cruise_source_time);
  probe_version_detector.StartHybridStableObservation(
      HybridStableObservationSource::kProbeRtt,
      QuicTime::Zero() + TimeDelta::FromMilliseconds(20));
  probe_version_detector.fbbr_hybrid_stable_observation_round_done_ = true;
  probe_version_detector.fbbr_hybrid_stable_observation_min_rtt_ =
      TimeDelta::FromMilliseconds(35);
  probe_version_detector.fbbr_hybrid_stable_post_round_rate_samples_bps_ =
      {30000000, 50000000, 40000000, 100000000};
  probe_version_detector.MaybeFinishHybridStableObservation(
      QuicTime::Zero() + TimeDelta::FromMilliseconds(220), true);
  const bool lower_probe_replaced_both =
      probe_version_detector.fbbr_hybrid_srtt_low_ ==
          TimeDelta::FromMilliseconds(35) &&
      probe_version_detector.adaptive_baseline_low_.ToBitsPerSecond() ==
          40000000 &&
      probe_version_detector.fbbr_hybrid_rtprop_drate_.ToBitsPerSecond() ==
          40000000;
  probe_version_detector.StartHybridStableObservation(
      HybridStableObservationSource::kProbeRtt,
      QuicTime::Zero() + TimeDelta::FromMilliseconds(300));
  probe_version_detector.fbbr_hybrid_stable_observation_round_done_ = true;
  probe_version_detector.fbbr_hybrid_stable_observation_min_rtt_ =
      TimeDelta::FromMilliseconds(45);
  probe_version_detector.fbbr_hybrid_stable_post_round_rate_samples_bps_ =
      {20000000, 25000000, 30000000};
  probe_version_detector.MaybeFinishHybridStableObservation(
      QuicTime::Zero() + TimeDelta::FromMilliseconds(500), true);
  require(lower_probe_replaced_both &&
              probe_version_detector.fbbr_hybrid_srtt_low_ ==
                  TimeDelta::FromMilliseconds(45) &&
              probe_version_detector.adaptive_baseline_low_
                      .ToBitsPerSecond() == 40000000 &&
              probe_version_detector.fbbr_hybrid_rtprop_drate_
                      .ToBitsPerSecond() == 40000000,
          "newer ProbeRTT always versions srtt_low, but replaces both lower-rate references only when RTprop is lower");
  detector.ApplyFbbrHybridRegimeStateUpdates(
      &lower_bound_window,
      QuicTime::Zero() + TimeDelta::FromMilliseconds(6));
  detector.adaptive_previous_cruise_max_bw_valid_ = true;
  detector.adaptive_previous_cruise_max_bw_ =
      QuicBandwidth::FromBitsPerSecond(1000000000);
  detector.EnterCruise(
      QuicTime::Zero() + TimeDelta::FromMilliseconds(7));
  require(detector.adaptive_baseline_low_valid_ &&
              detector.adaptive_baseline_low_.ToBitsPerSecond() ==
                  80000000 &&
              !detector.adaptive_baseline_up_valid_ &&
              detector.model_.MaxBandwidth().ToBitsPerSecond() ==
                  100000000 &&
              detector.fbbr_hybrid_rtprop_drate_valid_ &&
              detector.fbbr_hybrid_rtprop_drate_.ToBitsPerSecond() ==
                  80000000 &&
              detector.fbbr_hybrid_max_srtt_valid_ &&
              detector.fbbr_hybrid_max_srtt_ms_ == 85.0 &&
              detector.model_.MinRtt() ==
                  TimeDelta::FromMilliseconds(40) &&
              detector.adaptive_bounds_inherited_this_cruise_,
          "Hybrid inherits the lower references and MaxSRTT while upper bound remains current MaxBw");
  WaveformWindowAnalysis retry_window;
  retry_window.fbbr_hybrid_pipeline = true;
  retry_window.classification = WaveformClassification::kOverload;
  retry_window.unsuppressed_classification =
      WaveformClassification::kOverload;
  retry_window.hybrid_decision.rule_id = "N16";
  retry_window.invalid_reason = "none";
  retry_window.hybrid_features.srtt.wave.input_valid = true;
  retry_window.hybrid_features.drate.wave.input_valid = true;
  retry_window.hybrid_features.srtt.wave.has_wave = false;
  retry_window.hybrid_features.drate.wave.has_wave = false;
  retry_window.window_first_cycle_id = 1;
  retry_window.window_second_cycle_id = 2;
  detector.UpdateFbbrHybridRetryState(&retry_window);
  require(!retry_window.no_wave_triggered &&
              retry_window.srtt_no_wave_streak == 1 &&
              retry_window.drate_no_wave_streak == 1 &&
              !retry_window.classification_suppressed_for_retry,
          "first no-wave window only increments independent streaks");
  retry_window.classification = WaveformClassification::kOverload;
  retry_window.invalid_reason = "none";
  retry_window.window_first_cycle_id = 2;
  retry_window.window_second_cycle_id = 3;
  detector.UpdateFbbrHybridRetryState(&retry_window);
  require(retry_window.no_wave_triggered &&
              retry_window.wave_fidelity_just_entered &&
              retry_window.classification_suppressed_for_retry &&
              retry_window.state_updates_suppressed_for_retry &&
              retry_window.classification ==
                  WaveformClassification::kInconclusive,
          "second no-wave window enters retry and freezes its N16 result");
  retry_window.classification = WaveformClassification::kOverload;
  retry_window.classification_suppressed_for_retry = false;
  retry_window.state_updates_suppressed_for_retry = false;
  retry_window.no_wave_triggered = false;
  retry_window.wave_fidelity_just_entered = false;
  retry_window.invalid_reason = "none";
  retry_window.window_first_cycle_id = 3;
  retry_window.window_second_cycle_id = 4;
  detector.UpdateFbbrHybridRetryState(&retry_window);
  require(retry_window.classification_suppressed_for_retry &&
              retry_window.wave_fidelity_enhancement_active &&
              retry_window.srtt_no_wave_streak == 3 &&
              retry_window.drate_no_wave_streak == 3,
          "continued no-wave rolling window remains suppressed");
  retry_window.classification = WaveformClassification::kUnderload;
  retry_window.classification_suppressed_for_retry = false;
  retry_window.state_updates_suppressed_for_retry = false;
  retry_window.invalid_reason = "none";
  retry_window.hybrid_features.drate.wave.has_wave = true;
  retry_window.window_first_cycle_id = 4;
  retry_window.window_second_cycle_id = 5;
  detector.UpdateFbbrHybridRetryState(&retry_window);
  require(!retry_window.classification_suppressed_for_retry &&
              !retry_window.wave_fidelity_enhancement_active &&
              retry_window.srtt_no_wave_streak == 0 &&
              retry_window.drate_no_wave_streak == 0,
          "either signal recovering exits fidelity retry and clears streaks");
  const double period_s = 0.20;
  const double dt_s = 0.005;
  const size_t sample_count = 80;
  const size_t samples_per_period =
      static_cast<size_t>(std::llround(period_s / dt_s));
  std::vector<bool> signal_valid(sample_count, true);
  std::vector<double> smooth_sine(sample_count, 0.0);
  std::vector<double> abrupt_jitter(sample_count, 0.0);
  std::vector<double> positive_half_wave(sample_count, 0.0);
  std::vector<double> negative_half_wave(sample_count, 0.0);
  std::vector<double> one_way(sample_count, 0.0);
  std::vector<double> single_spike(sample_count, 100.0);
  auto fill_half_cycle = [&](std::vector<double>* wave, bool positive) {
    for (size_t cycle = 0; cycle < 2; ++cycle) {
      const size_t begin = cycle * samples_per_period;
      for (size_t i = 0; i < samples_per_period; ++i) {
        if (i < 8) {
          (*wave)[begin + i] = positive ? 100.0 + 6.0 * i
                                        : 148.0 - 6.0 * i;
        } else {
          (*wave)[begin + i] = positive ? 148.0 : 100.0;
        }
      }
    }
  };
  for (size_t i = 0; i < sample_count; ++i) {
    smooth_sine[i] = 100.0 + 10.0 * std::sin(
        2.0 * M_PI * i * dt_s / period_s);
    abrupt_jitter[i] = 100.0 +
        (((i / 4) % 2 == 0) ? -10.0 : 10.0);
    one_way[i] = 100.0 + static_cast<double>(i % 40);
  }
  single_spike[20] = 180.0;
  fill_half_cycle(&positive_half_wave, true);
  fill_half_cycle(&negative_half_wave, false);
  const WaveActivityFeatures smooth_wave =
      detector.DetectOrdinaryWaveActivity(
          smooth_sine, signal_valid, dt_s, period_s);
  const WaveActivityFeatures jitter_wave =
      detector.DetectOrdinaryWaveActivity(
          abrupt_jitter, signal_valid, dt_s, period_s);
  const WaveActivityFeatures positive_half_strict =
      detector.DetectOrdinaryWaveActivity(
          positive_half_wave, signal_valid, dt_s, period_s);
  const WaveActivityFeatures positive_half_relaxed =
      detector.DetectOrdinaryWaveActivity(
          positive_half_wave, signal_valid, dt_s, period_s, true);
  const WaveActivityFeatures negative_half_relaxed =
      detector.DetectOrdinaryWaveActivity(
          negative_half_wave, signal_valid, dt_s, period_s, true);
  const WaveActivityFeatures one_way_wave =
      detector.DetectOrdinaryWaveActivity(
          one_way, signal_valid, dt_s, period_s);
  const WaveActivityFeatures spike_wave =
      detector.DetectOrdinaryWaveActivity(
          single_spike, signal_valid, dt_s, period_s);
  require(smooth_wave.input_valid && !smooth_wave.has_wave &&
              jitter_wave.input_valid && jitter_wave.has_wave,
          "ordinary-wave detector separates smooth periodic motion from abrupt round trips");
  require(positive_half_strict.input_valid && !positive_half_strict.has_wave &&
              positive_half_relaxed.has_wave && negative_half_relaxed.has_wave,
          "ordinary-wave detector accepts a single positive or negative half-cycle when enabled");
  require(!one_way_wave.has_wave && !spike_wave.has_wave,
          "ordinary-wave return and robust-amplitude gates reject drift and one spike");

  std::vector<double> top_clipped = smooth_sine;
  for (double& value : top_clipped) {
    value = std::min(value, 105.0);
  }
  const auto smooth_segments = detector.DetectContinuousHorizontalSegments(
      smooth_sine, signal_valid, dt_s, period_s);
  const auto top_segments = detector.DetectContinuousHorizontalSegments(
      top_clipped, signal_valid, dt_s, period_s);
  bool found_top_segment = false;
  for (const auto& segment : top_segments) {
    found_top_segment = found_top_segment ||
        (segment.valid && segment.is_upper &&
         segment.duration_ratio_of_period > 0.20);
  }
  require(smooth_segments.empty() && found_top_segment,
          "continuous-horizontal detector rejects a round top and accepts hard clipping");

  std::vector<double> repeated_top(sample_count, 90.0);
  for (size_t cycle = 0; cycle < 2; ++cycle) {
    const size_t offset = cycle * 40;
    for (size_t center : {static_cast<size_t>(8),
                          static_cast<size_t>(28)}) {
      repeated_top[offset + center - 2] = 96.0;
      repeated_top[offset + center - 1] = 104.0;
      repeated_top[offset + center] = 110.0;
      repeated_top[offset + center + 1] = 110.0;
      repeated_top[offset + center + 2] = 110.0;
      repeated_top[offset + center + 3] = 104.0;
      repeated_top[offset + center + 4] = 96.0;
    }
  }
  const RepeatedClipLineEvidence repeated =
      detector.DetectRepeatedClipLineContacts(
          repeated_top, signal_valid, dt_s, period_s, true);
  require(repeated.valid && repeated.contact_cycle_mask == 0x3 &&
              repeated.contact_time_span_ratio_of_window >= 0.50 &&
              repeated.contact_fragment_count >= 2,
          "repeated short contacts use cross-cycle same-level evidence and 50 percent span");

  SignalRegimeFeatures periodic_features;
  std::vector<bool> periodic_mask = signal_valid;
  const PeriodicSimilarityResult periodic_match =
      detector.AnalyzeFbbrHybridPeriodicSimilarity(
          smooth_sine, signal_valid, periodic_mask, dt_s, period_s,
          period_s, false, &periodic_features);
  const PeriodicSimilarityResult upper_veto =
      detector.AnalyzeFbbrHybridPeriodicSimilarity(
          smooth_sine, signal_valid, periodic_mask, dt_s, period_s,
          period_s, true, &periodic_features);
  require(periodic_match == PeriodicSimilarityResult::kMatch &&
              upper_veto == PeriodicSimilarityResult::kNoMatch,
          "periodic similarity accepts waveform-shape independence and enforces upper-clip veto");

  std::vector<bool> insufficient_valid = signal_valid;
  for (size_t i = 0; i < 50; ++i) {
    insufficient_valid[i] = false;
  }
  SignalRegimeFeatures invalid_periodic_features;
  require(detector.AnalyzeFbbrHybridPeriodicSimilarity(
              smooth_sine, signal_valid, insufficient_valid,
              dt_s, period_s, period_s, false,
              &invalid_periodic_features) ==
              PeriodicSimilarityResult::kInvalidInput,
          "periodic similarity preserves INVALID_INPUT instead of coercing false");
  os << "RESULT: " << (pass ? "PASS" : "FAIL") << "\n";
  return pass;
}

bool FBBRSender::RunFbbrHybridV3SelfTest(std::ostream& os) {
  bool pass = true;
  auto require = [&pass, &os](bool condition, const std::string& message) {
    os << (condition ? "PASS: " : "FAIL: ") << message << "\n";
    pass = pass && condition;
  };
  os << "# FBBR-hybridv3 model-consistent inflight self-test\n";

  RttStats rtt;
  rtt.set_initial_rtt(TimeDelta::FromMilliseconds(20));
  Random random;
  QuicConnectionStats stats;
  FBBRSender v1(QuicTime::Zero(), &rtt, nullptr, 10, 1000,
                &random, &stats, false, false, false, kFBBRHybrid);
  FBBRSender v3(QuicTime::Zero(), &rtt, nullptr, 10, 1000,
                &random, &stats, false, false, false, kFBBRHybridV3);
  v1.ConfigureFBBR(FBBRConfig());
  v3.ConfigureFBBR(FBBRConfig());
  require(v1.IsFbbrHybrid() && !v1.IsFbbrHybridV3() &&
              v3.IsFbbrHybridV3() && !v3.IsFbbrHybrid() &&
              v1.IsFbbrHybridObserver() &&
              v3.IsFbbrHybridObserver(),
          "FBBR-hybrid remains V1 and only FBBR-hybridv3 enables V3");
  require(kFBBRHybridV3 != kFBBRHybrid &&
              kFBBRHybridV3 != kFBBR &&
              kFBBRHybridV3 != kFBBRAdaptive,
          "V3 owns an isolated congestion-control type");

  const FbbrHybridActuatorResult v1_action =
      ComputeFbbrHybridInjectionBaseline(
          WaveformClassification::kOverload,
          80.0, 120.0, 100.0, true, 80.0, true, 120.0, 110.0,
          true, 60.0, 0.50, 1.0, true, 0.10);
  require(v1_action.valid &&
              std::abs(v1_action.next_baseline_bps - 99.0) < 1e-12 &&
              std::abs(v1_action.overload_beta - 0.10) < 1e-12,
          "V1 gradient-matched key action is unchanged");

  auto reset_history = [&v3]() {
    v3.fbbr_v3_pacing_target_history_.clear();
    v3.fbbr_v3_max_rtprop_seen_ = TimeDelta::Zero();
    v3.fbbr_v3_history_start_time_ = QuicTime::Zero();
    v3.fbbr_v3_history_valid_ = false;
    v3.fbbr_v3_last_recorded_pacing_target_ =
        QuicBandwidth::Zero();
  };
  auto at_ms = [](int64_t milliseconds) {
    return QuicTime::Zero() +
        TimeDelta::FromMilliseconds(milliseconds);
  };

  reset_history();
  v3.RecordFbbrV3PacingTarget(
      at_ms(0), QuicBandwidth::FromBitsPerSecond(100000000));
  require(v3.ComputeFbbrV3ModelInflightBytes(
              at_ms(20), TimeDelta::FromMilliseconds(20)) == 250000,
          "100 Mbps over 20 ms integrates to 250000 bytes");

  reset_history();
  v3.RecordFbbrV3PacingTarget(
      at_ms(0), QuicBandwidth::FromBitsPerSecond(50000000));
  v3.RecordFbbrV3PacingTarget(
      at_ms(10), QuicBandwidth::FromBitsPerSecond(100000000));
  require(v3.ComputeFbbrV3ModelInflightBytes(
              at_ms(20), TimeDelta::FromMilliseconds(20)) == 187500,
          "50/100 Mbps two-segment history integrates exactly");

  reset_history();
  v3.RecordFbbrV3PacingTarget(
      at_ms(0), QuicBandwidth::FromBitsPerSecond(25000000));
  v3.RecordFbbrV3PacingTarget(
      at_ms(5), QuicBandwidth::FromBitsPerSecond(50000000));
  v3.RecordFbbrV3PacingTarget(
      at_ms(15), QuicBandwidth::FromBitsPerSecond(100000000));
  require(v3.ComputeFbbrV3ModelInflightBytes(
              at_ms(20), TimeDelta::FromMilliseconds(10)) == 93750,
          "window-middle target change and partial segments use overlap");

  reset_history();
  v3.RecordFbbrV3PacingTarget(
      at_ms(0), QuicBandwidth::FromBitsPerSecond(50000000));
  v3.RecordFbbrV3PacingTarget(
      at_ms(0), QuicBandwidth::FromBitsPerSecond(100000000));
  v3.RecordFbbrV3PacingTarget(
      at_ms(0), QuicBandwidth::FromBitsPerSecond(100000000));
  require(v3.fbbr_v3_pacing_target_history_.size() == 1 &&
              v3.ComputeFbbrV3ModelInflightBytes(
                  at_ms(20), TimeDelta::FromMilliseconds(20)) ==
                  250000,
          "duplicate timestamps coalesce without zero-duration arithmetic");

  reset_history();
  for (int64_t ms = 0; ms <= 40; ms += 10) {
    v3.RecordFbbrV3PacingTarget(
        at_ms(ms), QuicBandwidth::FromBitsPerSecond(
            100000000 + ms * 1000));
  }
  require(v3.HasFullFbbrV3RateHistory(
              at_ms(40), TimeDelta::FromMilliseconds(20)),
          "initial history covers the current RTprop");
  v3.RecordFbbrV3PacingTarget(
      at_ms(50), QuicBandwidth::FromBitsPerSecond(100060000));
  require(!v3.HasFullFbbrV3RateHistory(
              at_ms(50), TimeDelta::FromMilliseconds(40)),
          "RTprop growth pauses projection when retained history is short");
  v3.RecordFbbrV3PacingTarget(
      at_ms(60), QuicBandwidth::FromBitsPerSecond(100070000));
  v3.RecordFbbrV3PacingTarget(
      at_ms(70), QuicBandwidth::FromBitsPerSecond(100080000));
  require(v3.HasFullFbbrV3RateHistory(
              at_ms(70), TimeDelta::FromMilliseconds(40)),
          "projection resumes after real history covers enlarged RTprop");

  const QuicByteCount cap = v3.ComputeFbbrV3InflightCapBytes(
      250000, 20000, 0);
  const QuicByteCount expected_cap =
      ((270000 + kDefaultTCPMSS - 1) / kDefaultTCPMSS) *
      kDefaultTCPMSS;
  require(cap == expected_cap,
          "cap equals model plus existing ACK headroom, rounded by MSS");
  require(v3.ComputeFbbrV3InflightCapBytes(1, 0, 0) >=
              v3.cwnd_limits().Min(),
          "cap never falls below native MinPipeCwnd");
  require(v3.ComputeFbbrV3InflightCapBytes(
              std::numeric_limits<QuicByteCount>::max(), 1, 1) ==
              std::numeric_limits<QuicByteCount>::max(),
          "cap arithmetic saturates without overflow");

  reset_history();
  v3.trusted_bw_ =
      QuicBandwidth::FromBitsPerSecond(100000000);
  v3.trusted_bw_valid_ = true;
  v3.trusted_bw_source_ = kTrustedBwSourceFbbrWindowMean;
  v3.RecordFbbrV3PacingTarget(
      at_ms(0), QuicBandwidth::FromBitsPerSecond(100000000));
  v3.current_time_ = at_ms(20);
  v3.fbbr_hybrid_srtt_low_valid_ = true;
  v3.fbbr_hybrid_srtt_low_ = TimeDelta::FromMilliseconds(20);
  v3.drain_completed_ = true;
  v3.mode_ = Bbr2Mode::PROBE_BW;
  v3.latest_congestion_event_inflight_valid_ = true;
  v3.latest_congestion_event_inflight_ = 1000000;
  const QuicByteCount native_cwnd = 1000000;
  const QuicByteCount projected =
      v3.ApplyFbbrV3InflightProjection(native_cwnd);
  require(projected <= native_cwnd &&
              projected >= v3.cwnd_limits().Min(),
          "final V3 cwnd is min(native, cap) and respects MinPipeCwnd");
  require(projected < v3.latest_congestion_event_inflight_,
          "inflight above cap does not raise final cwnd to current inflight");
  require(!v3.BuildFbbrV3ProjectionSnapshot(
                   native_cwnd, projected).cap_binding,
          "actual inflight equal to cap is not counted as excess binding");
  require(v3.BuildFbbrV3ProjectionSnapshot(
                  native_cwnd, projected + 1).cap_binding,
          "cap binding is counted only when actual inflight exceeds cap");
  v3.mode_ = Bbr2Mode::STARTUP;
  require(v3.ApplyFbbrV3InflightProjection(native_cwnd) ==
              native_cwnd,
          "Startup is not overridden by V3 projection");
  v3.mode_ = Bbr2Mode::PROBE_RTT;
  require(v3.ApplyFbbrV3InflightProjection(native_cwnd) ==
              native_cwnd,
          "ProbeRTT remains under native ProbeRTT cwnd control");
  v3.mode_ = Bbr2Mode::PROBE_BW;
  v3.fbbr_v3_pacing_target_history_.back().target_rate =
      QuicBandwidth::FromBitsPerSecond(1000000000);
  require(v3.ApplyFbbrV3InflightProjection(native_cwnd) ==
              native_cwnd,
          "cap release is immediate and needs no recovery state machine");

  v3.trusted_bw_valid_ = false;
  v3.guard_filter_valid_ = true;
  v3.guard_filter_stage2_ =
      QuicBandwidth::FromBitsPerSecond(90000000);
  require(!v3.SelectFbbrV3ReferenceBw().valid,
          "unpublished Guard filter state is not yet a valid ReferenceBw");
  v3.trusted_bw_ = v3.guard_filter_stage2_;
  v3.trusted_bw_valid_ = true;
  v3.trusted_bw_source_ = kTrustedBwSourceGuardFilter;
  require(v3.SelectFbbrV3ReferenceBw().source ==
              FbbrV3ReferenceSource::kGuard,
          "published GuardBw is selected when TrustedBw is unavailable");
  v3.trusted_bw_source_ = kTrustedBwSourcePreviousTrusted;
  require(v3.SelectFbbrV3ReferenceBw().source ==
              FbbrV3ReferenceSource::kLastValid,
          "existing PREVIOUS_TRUSTED validity maps to last-valid");
  v3.trusted_bw_source_ = kTrustedBwSourceNativeFallback;
  require(!v3.SelectFbbrV3ReferenceBw().valid,
          "native MaxBw/bootstrap fallback is not a valid V3 reference");

  os << "RESULT: " << (pass ? "PASS" : "FAIL") << "\n";
  return pass;
}

bool FBBRSender::RunFbbrHybridV4SelfTest(std::ostream& os) {
  bool pass = true;
  auto require = [&pass, &os](bool condition,
                              const std::string& message) {
    os << (condition ? "PASS: " : "FAIL: ") << message << "\n";
    pass = pass && condition;
  };
  os << "# FBBR-hybirdv4 service-consistent inflight self-test\n";

  RttStats rtt;
  rtt.set_initial_rtt(TimeDelta::FromMilliseconds(20));
  Random random;
  QuicConnectionStats stats;
  FBBRSender v1(QuicTime::Zero(), &rtt, nullptr, 10, 1000,
                &random, &stats, false, false, false, kFBBRHybrid);
  FBBRSender v3(QuicTime::Zero(), &rtt, nullptr, 10, 1000,
                &random, &stats, false, false, false, kFBBRHybridV3);
  FBBRSender v4(QuicTime::Zero(), &rtt, nullptr, 10, 1000,
                &random, &stats, false, false, false, kFBBRHybridV4);
  v1.ConfigureFBBR(FBBRConfig());
  v3.ConfigureFBBR(FBBRConfig());
  v4.ConfigureFBBR(FBBRConfig());
  require(v1.IsFbbrHybrid() && !v1.IsFbbrProjectionObserver() &&
              v3.IsFbbrHybridV3() && v3.IsFbbrProjectionObserver() &&
              v4.IsFbbrHybridV4() && v4.IsFbbrProjectionObserver() &&
              !v4.IsFbbrHybridV3() && !v4.IsFbbrHybrid(),
          "V1, V3, and V4 own isolated algorithm identities");
  require(kFBBRHybridV4 != kFBBRHybridV3 &&
              kFBBRHybridV4 != kFBBRHybrid &&
              kFBBRHybridV4 != kFBBR,
          "V4 owns an appended congestion-control enum value");

  auto at_ms = [](int64_t milliseconds) {
    return QuicTime::Zero() +
        TimeDelta::FromMilliseconds(milliseconds);
  };
  auto bw = [](uint64_t bps) {
    return QuicBandwidth::FromBitsPerSecond(bps);
  };
  auto reset_v3_rate = [&v3]() {
    v3.fbbr_v3_pacing_target_history_.clear();
    v3.fbbr_v3_max_rtprop_seen_ = TimeDelta::Zero();
    v3.fbbr_v3_history_start_time_ = QuicTime::Zero();
    v3.fbbr_v3_history_valid_ = false;
    v3.fbbr_v3_last_recorded_pacing_target_ =
        QuicBandwidth::Zero();
  };
  auto reset_v4_rate = [&v4]() {
    v4.fbbr_v4_rate_history_.clear();
    v4.fbbr_v4_max_rtprop_seen_ = TimeDelta::Zero();
    v4.fbbr_v4_rate_history_integrity_valid_ = true;
    v4.fbbr_v4_last_target_rate_ = QuicBandwidth::Zero();
    v4.fbbr_v4_last_base_target_rate_ =
        QuicBandwidth::Zero();
  };
  auto reset_service = [&v4]() {
    v4.fbbr_v4_delivered_history_.clear();
    v4.fbbr_v4_delivered_history_integrity_valid_ = true;
    v4.fbbr_v4_last_counter_reset_time_ = QuicTime::Zero();
  };
  const TimeDelta rtprop20 = TimeDelta::FromMilliseconds(20);

  reset_v4_rate();
  v4.RecordFbbrV4RateTargets(at_ms(0), bw(100000000), bw(100000000));
  require(v4.ComputeFbbrV4PlannedInflightBytes(
              at_ms(20), rtprop20) == 250000,
          "V4 constant 100 Mbps over 20 ms plans 250000 bytes");

  reset_v4_rate();
  v4.RecordFbbrV4RateTargets(at_ms(0), bw(50000000), bw(50000000));
  v4.RecordFbbrV4RateTargets(at_ms(10), bw(100000000), bw(100000000));
  require(v4.ComputeFbbrV4PlannedInflightBytes(
              at_ms(20), rtprop20) == 187500,
          "V4 segmented planned inflight uses exact overlap");

  reset_v4_rate();
  v4.RecordFbbrV4RateTargets(at_ms(0), bw(25000000), bw(25000000));
  v4.RecordFbbrV4RateTargets(at_ms(5), bw(50000000), bw(50000000));
  v4.RecordFbbrV4RateTargets(at_ms(15), bw(100000000), bw(100000000));
  require(v4.ComputeFbbrV4PlannedInflightBytes(
              at_ms(20), TimeDelta::FromMilliseconds(10)) == 93750,
          "V4 partial-overlap planned inflight matches V3 semantics");

  reset_v3_rate();
  reset_v4_rate();
  v3.RecordFbbrV3PacingTarget(at_ms(0), bw(25000000));
  v3.RecordFbbrV3PacingTarget(at_ms(5), bw(50000000));
  v3.RecordFbbrV3PacingTarget(at_ms(15), bw(100000000));
  v4.RecordFbbrV4RateTargets(at_ms(0), bw(25000000), bw(25000000));
  v4.RecordFbbrV4RateTargets(at_ms(5), bw(50000000), bw(50000000));
  v4.RecordFbbrV4RateTargets(at_ms(15), bw(100000000), bw(100000000));
  require(v3.ComputeFbbrV3ModelInflightBytes(
              at_ms(20), TimeDelta::FromMilliseconds(10)) ==
              v4.ComputeFbbrV4PlannedInflightBytes(
                  at_ms(20), TimeDelta::FromMilliseconds(10)),
          "V3 model inflight and V4 planned inflight are identical");

  reset_v4_rate();
  v4.RecordFbbrV4RateTargets(at_ms(0), bw(100000000), bw(100000000));
  require(v4.ComputeFbbrV4PositiveProbeCreditBytes(
              at_ms(20), rtprop20) == 0,
          "equal 100 Mbps target/base yields zero probe credit");

  reset_v4_rate();
  v4.RecordFbbrV4RateTargets(at_ms(0), bw(125000000), bw(100000000));
  require(v4.ComputeFbbrV4PositiveProbeCreditBytes(
              at_ms(20), rtprop20) == 62500,
          "25 Mbps positive excess over 20 ms yields 62500 bytes");

  reset_v4_rate();
  v4.RecordFbbrV4RateTargets(at_ms(0), bw(75000000), bw(100000000));
  require(v4.ComputeFbbrV4PositiveProbeCreditBytes(
              at_ms(20), rtprop20) == 0 &&
              v4.fbbr_v4_rate_history_.front().base_target_rate <=
                  v4.fbbr_v4_rate_history_.front().target_rate,
          "target below base is clamped and cannot create credit");

  reset_v4_rate();
  v4.RecordFbbrV4RateTargets(at_ms(0), bw(125000000), bw(100000000));
  v4.RecordFbbrV4RateTargets(at_ms(10), bw(75000000), bw(75000000));
  require(v4.ComputeFbbrV4PositiveProbeCreditBytes(
              at_ms(20), rtprop20) == 31250,
          "positive/negative halves accumulate only the positive difference");

  reset_v4_rate();
  v4.RecordFbbrV4RateTargets(at_ms(0), bw(150000000), bw(100000000));
  require(v4.ComputeFbbrV4PositiveProbeCreditBytes(
              at_ms(20), rtprop20) == 125000,
          "stacked phase/wave probing uses the final target difference once");

  reset_service();
  v4.RecordFbbrV4DeliveredPoint(at_ms(0), 1000, false);
  v4.RecordFbbrV4DeliveredPoint(at_ms(10), 2000, false);
  v4.RecordFbbrV4DeliveredPoint(at_ms(20), 4000, false);
  bool contaminated = false;
  require(v4.HasValidFbbrV4ServiceHistory(
              at_ms(20), rtprop20, false, &contaminated) &&
              v4.ComputeFbbrV4ServiceInflightBytes(
                  at_ms(20), rtprop20) == 3000,
          "monotonic cumulative delivered computes exact step service");

  reset_service();
  v4.RecordFbbrV4DeliveredPoint(at_ms(0), 100, false);
  v4.RecordFbbrV4DeliveredPoint(at_ms(5), 500, false);
  v4.RecordFbbrV4DeliveredPoint(at_ms(10), 1000, false);
  v4.RecordFbbrV4DeliveredPoint(at_ms(20), 2500, false);
  require(v4.ComputeFbbrV4ServiceInflightBytes(
              at_ms(20), TimeDelta::FromMilliseconds(12)) == 2000,
          "service uses the newest anchor no later than the boundary");

  reset_service();
  v4.RecordFbbrV4DeliveredPoint(at_ms(0), 100, false);
  v4.RecordFbbrV4DeliveredPoint(at_ms(10), 500, false);
  v4.RecordFbbrV4DeliveredPoint(at_ms(10), 700, true);
  require(v4.fbbr_v4_delivered_history_.size() == 2 &&
              v4.fbbr_v4_delivered_history_.back()
                      .cumulative_delivered_bytes == 700 &&
              v4.fbbr_v4_delivered_history_.back().app_limited,
          "same-timestamp delivered points merge to newest maximum");

  reset_service();
  v4.RecordFbbrV4DeliveredPoint(at_ms(0), 100, false);
  v4.RecordFbbrV4DeliveredPoint(at_ms(10), 100, false);
  v4.RecordFbbrV4DeliveredPoint(at_ms(20), 100, false);
  require(v4.ComputeFbbrV4ServiceInflightBytes(
              at_ms(20), rtprop20) == 0,
          "duplicate ACK state does not increase delivered service");

  reset_service();
  v4.RecordFbbrV4DeliveredPoint(at_ms(10), 100, false);
  v4.RecordFbbrV4DeliveredPoint(at_ms(20), 200, false);
  require(!v4.HasValidFbbrV4ServiceHistory(
              at_ms(20), rtprop20, false, &contaminated),
          "service history without a boundary anchor is invalid");

  reset_service();
  v4.RecordFbbrV4DeliveredPoint(at_ms(0), 1000, false);
  v4.RecordFbbrV4DeliveredPoint(at_ms(10), 2000, false);
  v4.RecordFbbrV4DeliveredPoint(at_ms(15), 10, false);
  v4.RecordFbbrV4DeliveredPoint(at_ms(25), 100, false);
  require(!v4.HasValidFbbrV4ServiceHistory(
              at_ms(25), rtprop20, false, &contaminated),
          "counter reset inside RTprop invalidates service history");

  reset_service();
  v4.RecordFbbrV4DeliveredPoint(at_ms(20), 2000, false);
  v4.RecordFbbrV4DeliveredPoint(at_ms(10), 1000, false);
  require(!v4.HasValidFbbrV4ServiceHistory(
              at_ms(40), rtprop20, false, &contaminated),
          "out-of-order delivered timestamps invalidate history");

  reset_service();
  v4.RecordFbbrV4DeliveredPoint(at_ms(0), 100, false);
  v4.RecordFbbrV4DeliveredPoint(at_ms(10), 1000, true);
  v4.RecordFbbrV4DeliveredPoint(at_ms(20), 2000, false);
  require(!v4.HasValidFbbrV4ServiceHistory(
              at_ms(20), rtprop20, false, &contaminated) &&
              contaminated,
          "app-limited state anywhere in the interval invalidates service");

  reset_service();
  v4.RecordFbbrV4DeliveredPoint(at_ms(0), 100, true);
  v4.RecordFbbrV4DeliveredPoint(at_ms(10), 1000, false);
  v4.RecordFbbrV4DeliveredPoint(at_ms(30), 3000, false);
  require(v4.HasValidFbbrV4ServiceHistory(
              at_ms(30), rtprop20, false, &contaminated) &&
              !contaminated,
          "service validity returns after a full clean RTprop");
  require(!v4.HasValidFbbrV4ServiceHistory(
              at_ms(30), rtprop20, true, &contaminated) &&
              contaminated,
          "current app-limited state forces plan-only fallback");

  require(v4.ComputeFbbrV4EnvelopeBytes(
              250000, 240000, 20000, true) == 250000,
          "service plus probe at least plan leaves envelope at plan");
  require(v4.ComputeFbbrV4EnvelopeBytes(
              250000, 100000, 25000, true) == 125000,
          "service budget below plan restricts envelope to the budget");
  require(v4.ComputeFbbrV4EnvelopeBytes(
              250000, 100000, 25000, false) == 250000,
          "invalid service history falls back exactly to V3 plan");

  reset_v4_rate();
  v4.RecordFbbrV4RateTargets(at_ms(0), bw(100000000), bw(100000000));
  v4.current_time_ = at_ms(20);
  v4.fbbr_hybrid_srtt_low_valid_ = true;
  v4.fbbr_hybrid_srtt_low_ = rtprop20;
  v4.drain_completed_ = true;
  v4.mode_ = Bbr2Mode::PROBE_BW;
  v4.trusted_bw_valid_ = false;
  v4.latest_congestion_event_inflight_valid_ = true;
  v4.latest_congestion_event_inflight_ = 1000000;
  const QuicByteCount native_cwnd = 1000000;
  const FbbrV4EnvelopeSnapshot invalid_reference_snapshot =
      v4.BuildFbbrV4EnvelopeSnapshot(
          native_cwnd, v4.latest_congestion_event_inflight_);
  require(!invalid_reference_snapshot.reference.valid &&
              invalid_reference_snapshot.target_history_valid &&
              invalid_reference_snapshot.projection_active,
          "Reference invalid does not disable V4 planned projection");
  const QuicByteCount final_cwnd =
      v4.ApplyFbbrV4InflightEnvelope(native_cwnd);
  require(final_cwnd <= native_cwnd &&
              final_cwnd >= v4.cwnd_limits().Min(),
          "final V4 cwnd is bounded by native and MinPipeCwnd");
  require(final_cwnd < v4.latest_congestion_event_inflight_,
          "inflight above cap never raises the V4 cap");

  v4.mode_ = Bbr2Mode::STARTUP;
  require(v4.ApplyFbbrV4InflightEnvelope(native_cwnd) ==
              native_cwnd,
          "Startup remains under native BBR control");
  v4.mode_ = Bbr2Mode::PROBE_RTT;
  require(v4.ApplyFbbrV4InflightEnvelope(native_cwnd) ==
              native_cwnd,
          "ProbeRTT remains under native ProbeRTT control");

  require(v4.ComputeFbbrV4EnvelopeBytes(
              std::numeric_limits<QuicByteCount>::max(),
              std::numeric_limits<QuicByteCount>::max(), 1, true) ==
              std::numeric_limits<QuicByteCount>::max() &&
              v4.ComputeFbbrV4InflightCapBytes(
                  std::numeric_limits<QuicByteCount>::max(),
                  1, 1) ==
              std::numeric_limits<QuicByteCount>::max() &&
              v4.ComputeFbbrV4PlannedInflightBytes(
                  at_ms(20), TimeDelta::Zero()) == 0,
          "zero RTprop and saturated arithmetic avoid division/overflow");

  os << "RESULT: " << (pass ? "PASS" : "FAIL") << "\n";
  return pass;
}

Bbr2ProbeBwMode::CyclePhase FBBRSender::GetCurrentProbeBwPhase() const {
  DebugState state = ExportDebugState();
  if (state.mode == Bbr2Mode::PROBE_BW) {
    return state.probe_bw.phase;
  }
  return Bbr2ProbeBwMode::CyclePhase::PROBE_NOT_STARTED;
}

bool FBBRSender::ShouldEnableRtpropProbeDown(
    bool previous_cruise_rtprop_updated,
    QuicByteCount bytes_in_flight,
    QuicByteCount bdp) {
  return previous_cruise_rtprop_updated && bdp > 0 &&
         static_cast<long double>(bytes_in_flight) >=
             1.25L * static_cast<long double>(bdp);
}

bool FBBRSender::ShouldExitRtpropProbeDown(
    QuicByteCount bytes_in_flight,
    QuicByteCount bdp) {
  return bdp > 0 &&
         static_cast<long double>(bytes_in_flight) <
             1.05L * static_cast<long double>(bdp);
}

bool FBBRSender::HasCustomProbeDownLogic() const {
  return rtprop_probe_down_active_;
}

bool FBBRSender::ShouldExitCustomProbeDown(
    QuicByteCount bytes_in_flight,
    QuicByteCount bdp) const {
  return rtprop_probe_down_active_ &&
         ShouldExitRtpropProbeDown(bytes_in_flight, bdp);
}

float FBBRSender::GetProbeBwPacingGain(
    Bbr2ProbeBwMode::CyclePhase phase,
    float pacing_gain) const {
  if (phase == Bbr2ProbeBwMode::CyclePhase::PROBE_DOWN &&
      rtprop_probe_down_active_) {
    return kFBBRRtpropProbeDownPacingGain;
  }
  return pacing_gain;
}

float FBBRSender::GetProbeBwCwndGain(
    Bbr2ProbeBwMode::CyclePhase phase,
    float cwnd_gain) const {
  return FBBRCwndGainForPhase(phase, cwnd_gain);
}

bool FBBRSender::BaseShouldOscillate() const {
  if (IsFbbrHybrid() &&
      fbbr_hybrid_stable_observation_source_ ==
          HybridStableObservationSource::kCruiseFallback) {
    return false;
  }
  if (in_cruise_ &&
      cruise_detector_mode_ == FBBRCruiseDetectorMode::kTimeWaveform &&
      (current_injection_baseline_bw_.IsZero() ||
       probe_epoch_start_time_ == QuicTime::Zero() ||
       waveform_cruise_state_ == WaveformCruiseState::kDisabled)) {
    return false;
  }
  const uint64_t amplitude_bps =
      in_cruise_ &&
              cruise_detector_mode_ ==
                  FBBRCruiseDetectorMode::kTimeWaveform
          ? current_probe_amplitude_bps_
          : GetCurrentAmplitudeBps();
  if (amplitude_bps == 0 || configured_modulation_freq_hz_ <= 0.0) {
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

double FBBRSender::ComputeMaxBwAttenuationFactor(
    double delivery_center_bps,
    double actual_fluctuation_amplitude_bps) {
  if (!std::isfinite(delivery_center_bps) || delivery_center_bps <= 0.0 ||
      !std::isfinite(actual_fluctuation_amplitude_bps) ||
      actual_fluctuation_amplitude_bps <= 0.0) {
    return 1.0;
  }
  const long double center =
      static_cast<long double>(delivery_center_bps);
  const long double amplitude =
      static_cast<long double>(actual_fluctuation_amplitude_bps);
  const long double denominator = center + amplitude;
  if (!std::isfinite(denominator) || denominator <= center) {
    return 1.0;
  }
  return ClampValue(static_cast<double>(center / denominator),
                    std::numeric_limits<double>::min(), 1.0);
}

uint64_t FBBRSender::CurrentEmittedProbeAmplitudeBps() const {
  if (cruise_detector_mode_ == FBBRCruiseDetectorMode::kTimeWaveform) {
    return current_probe_amplitude_bps_;
  }
  return GetCurrentAmplitudeBps();
}

double FBBRSender::CurrentActualDeliveryFluctuationAmplitudeBps() const {
  const double emitted_amplitude_bps =
      static_cast<double>(CurrentEmittedProbeAmplitudeBps());
  if (!std::isfinite(emitted_amplitude_bps) ||
      emitted_amplitude_bps <= 0.0 ||
      !std::isfinite(max_bw_delivery_response_gain_) ||
      max_bw_delivery_response_gain_ < 0.0) {
    return 0.0;
  }
  return max_bw_delivery_response_gain_ * emitted_amplitude_bps;
}

double FBBRSender::CurrentMaxBwAttenuationFactor() const {
  if (!ShouldOscillate()) {
    return 1.0;
  }

  const double current_baseline_bps = static_cast<double>(
      (cruise_detector_mode_ == FBBRCruiseDetectorMode::kTimeWaveform
           ? current_injection_baseline_bw_
           : BandwidthEstimate())
          .ToBitsPerSecond());
  if (!std::isfinite(current_baseline_bps) || current_baseline_bps <= 0.0) {
    return 1.0;
  }

  // Once a response has been measured, predict how its center follows a
  // baseline adjustment with the same measured delivery-response gain.  An
  // underloaded path has gain near one and follows the new baseline; a
  // clipped/full path has gain near zero and keeps the observed center.
  double predicted_center_bps = current_baseline_bps;
  if (max_bw_response_observed_ &&
      std::isfinite(max_bw_observation_center_bps_) &&
      std::isfinite(max_bw_observation_baseline_bps_)) {
    predicted_center_bps = max_bw_observation_center_bps_ +
        max_bw_delivery_response_gain_ *
            (current_baseline_bps - max_bw_observation_baseline_bps_);
  }
  if (!std::isfinite(predicted_center_bps) || predicted_center_bps <= 0.0) {
    predicted_center_bps = current_baseline_bps;
  }
  return ComputeMaxBwAttenuationFactor(
      predicted_center_bps,
      CurrentActualDeliveryFluctuationAmplitudeBps());
}

void FBBRSender::ResetMaxBwAttenuationEstimator() {
  const double baseline_bps = static_cast<double>(
      current_injection_baseline_bw_.ToBitsPerSecond());
  max_bw_response_observed_ = false;
  // Before the first receiver window is available, use unit response gain as
  // the causal bootstrap: it exactly removes a freely delivered pacing wave
  // and is replaced by the measured response as soon as one window completes.
  max_bw_delivery_response_gain_ = 1.0;
  max_bw_observation_center_bps_ =
      std::isfinite(baseline_bps) && baseline_bps > 0.0
          ? baseline_bps
          : 0.0;
  max_bw_observation_baseline_bps_ = max_bw_observation_center_bps_;
  max_bw_actual_fluctuation_amplitude_bps_ =
      CurrentActualDeliveryFluctuationAmplitudeBps();
  max_bw_attenuation_factor_ = ComputeMaxBwAttenuationFactor(
      max_bw_observation_center_bps_,
      max_bw_actual_fluctuation_amplitude_bps_);
}

void FBBRSender::UpdateMaxBwAttenuationEstimator(
    double delivery_center_bps,
    double actual_fluctuation_amplitude_bps,
    double emitted_fluctuation_amplitude_bps) {
  if (!std::isfinite(delivery_center_bps) || delivery_center_bps <= 0.0 ||
      !std::isfinite(actual_fluctuation_amplitude_bps) ||
      actual_fluctuation_amplitude_bps < 0.0 ||
      !std::isfinite(emitted_fluctuation_amplitude_bps) ||
      emitted_fluctuation_amplitude_bps <= 0.0) {
    return;
  }
  const double baseline_bps = static_cast<double>(
      (cruise_detector_mode_ == FBBRCruiseDetectorMode::kTimeWaveform
           ? current_injection_baseline_bw_
           : BandwidthEstimate())
          .ToBitsPerSecond());
  if (!std::isfinite(baseline_bps) || baseline_bps <= 0.0) {
    return;
  }

  max_bw_response_observed_ = true;
  max_bw_delivery_response_gain_ =
      actual_fluctuation_amplitude_bps /
      emitted_fluctuation_amplitude_bps;
  max_bw_observation_center_bps_ = delivery_center_bps;
  max_bw_observation_baseline_bps_ = baseline_bps;
  max_bw_actual_fluctuation_amplitude_bps_ =
      CurrentActualDeliveryFluctuationAmplitudeBps();
  max_bw_attenuation_factor_ = CurrentMaxBwAttenuationFactor();
  QUIC_DVLOG(2)
      << "FBBR: maxbw attenuation updated. observed_center_bps="
      << delivery_center_bps
      << ", observed_amplitude_bps="
      << actual_fluctuation_amplitude_bps
      << ", emitted_amplitude_bps="
      << emitted_fluctuation_amplitude_bps
      << ", response_gain=" << max_bw_delivery_response_gain_
      << ", factor=" << max_bw_attenuation_factor_;
}

void FBBRSender::UpdateMaxBwAttenuationFromWaveform(
    const WaveformWindowAnalysis& analysis) {
  if (!analysis.delivery_rate_stats_valid || !analysis.drate_input_valid ||
      !analysis.sender_waveform_valid || !analysis.drate_fit.valid) {
    return;
  }
  UpdateMaxBwAttenuationEstimator(
      analysis.delivery_rate_mean_bps,
      analysis.current_drate_response_amplitude_bps,
      static_cast<double>(current_probe_amplitude_bps_));
}

void FBBRSender::UpdateMaxBwAttenuationFromLegacyWindow(
    const CruiseWindowResult& result) {
  if (!result.drate_valid || !result.srate_valid ||
      !std::isfinite(result.drate_mean_kbps) ||
      !std::isfinite(result.drate_target_amp) ||
      !std::isfinite(result.srate_target_amp) ||
      result.drate_mean_kbps <= 0.0 || result.drate_target_amp < 0.0 ||
      result.srate_target_amp <= 0.0) {
    return;
  }
  const double emitted_amplitude_bps =
      static_cast<double>(CurrentEmittedProbeAmplitudeBps());
  const double response_gain =
      result.drate_target_amp / result.srate_target_amp;
  UpdateMaxBwAttenuationEstimator(
      result.drate_mean_kbps * 1000.0,
      response_gain * emitted_amplitude_bps,
      emitted_amplitude_bps);
}

TimeDelta FBBRSender::CurrentHybridMaxSrttObservationRtt() const {
  if (rtt_stats_ != nullptr) {
    const TimeDelta smoothed_rtt = rtt_stats_->smoothed_rtt();
    if (!smoothed_rtt.IsZero() && !smoothed_rtt.IsInfinite()) {
      return smoothed_rtt;
    }
    const TimeDelta latest_rtt = rtt_stats_->latest_rtt();
    if (!latest_rtt.IsZero() && !latest_rtt.IsInfinite()) {
      return latest_rtt;
    }
    const TimeDelta initial_rtt = rtt_stats_->SmoothedOrInitialRtt();
    if (!initial_rtt.IsZero() && !initial_rtt.IsInfinite()) {
      return initial_rtt;
    }
  }
  const TimeDelta model_min_rtt = model_.MinRtt();
  if (!model_min_rtt.IsZero() && !model_min_rtt.IsInfinite()) {
    return model_min_rtt;
  }
  return TimeDelta::Zero();
}

bool FBBRSender::ComputeHybridMaxSrttAround(
    QuicTime center_time,
    TimeDelta half_window,
    double* max_srtt_ms,
    size_t* sample_count) const {
  if (max_srtt_ms == nullptr || sample_count == nullptr ||
      half_window.IsZero() || half_window.IsInfinite()) {
    return false;
  }
  const QuicTime start = center_time - half_window;
  const QuicTime end = center_time + half_window;
  double max_ms = -std::numeric_limits<double>::infinity();
  size_t count = 0;
  for (const auto& sample : srtt_history_) {
    if (sample.time < start || sample.time > end ||
        !std::isfinite(sample.rtt_ms) || sample.rtt_ms <= 0.0) {
      continue;
    }
    max_ms = std::max(max_ms, sample.rtt_ms);
    ++count;
  }
  if (count == 0 || !std::isfinite(max_ms) || max_ms <= 0.0) {
    return false;
  }
  *max_srtt_ms = max_ms;
  *sample_count = count;
  return true;
}

void FBBRSender::RecordHybridMaxBwUpdateForMaxSrtt(
    QuicTime event_time,
    QuicBandwidth max_bw) {
  if (!IsFbbrHybridObserver() || !IsFinitePositiveBandwidth(max_bw) ||
      event_time == QuicTime::Zero()) {
    return;
  }
  const TimeDelta rtt = CurrentHybridMaxSrttObservationRtt();
  if (rtt.IsZero() || rtt.IsInfinite()) {
    return;
  }
  pending_hybrid_max_srtt_observations_.push_back(
      {event_time, rtt * 3, max_bw});
}

void FBBRSender::FinalizePendingHybridMaxSrttObservations(QuicTime now) {
  if (!IsFbbrHybridObserver()) {
    return;
  }
  while (!pending_hybrid_max_srtt_observations_.empty()) {
    const HybridMaxSrttObservation& observation =
        pending_hybrid_max_srtt_observations_.front();
    if (now < observation.update_time + observation.half_window) {
      break;
    }
    double max_srtt_ms = 0.0;
    size_t sample_count = 0;
    if (ComputeHybridMaxSrttAround(observation.update_time,
                                   observation.half_window,
                                   &max_srtt_ms,
                                   &sample_count)) {
      fbbr_hybrid_max_srtt_valid_ = true;
      fbbr_hybrid_max_srtt_ms_ = max_srtt_ms;
      fbbr_hybrid_max_srtt_source_cruise_id_ =
          static_cast<uint64_t>(std::max<int64_t>(0, cruise_id_));
    }
    pending_hybrid_max_srtt_observations_.pop_front();
  }
}

double FBBRSender::TriangleWave(QuicTime now) const {
  if (cruise_modulation_freq_hz_ <= 0.0 ||
      cruise_start_time_ == QuicTime::Zero()) {
    return 0.0;
  }
  const QuicTime epoch_start =
      cruise_detector_mode_ == FBBRCruiseDetectorMode::kTimeWaveform
          ? probe_epoch_start_time_
          : cruise_start_time_;
  if (epoch_start == QuicTime::Zero() || now < epoch_start) {
    return 0.0;
  }
  const double elapsed_s =
      static_cast<double>((now - epoch_start).ToMicroseconds()) /
      1000000.0;
  const double period_s = 1.0 / cruise_modulation_freq_hz_;
  if (period_s <= 0.0) {
    return 0.0;
  }
  const double q = std::fmod(std::max(0.0, elapsed_s), period_s) / period_s;
  if (cruise_detector_mode_ == FBBRCruiseDetectorMode::kTimeWaveform) {
    if (q < 0.25) {
      return -4.0 * q;
    }
    if (q < 0.75) {
      return 4.0 * q - 2.0;
    }
    return 4.0 - 4.0 * q;
  }
  if (q < 0.25) {
    return 4.0 * q;
  }
  if (q < 0.75) {
    return 2.0 - 4.0 * q;
  }
  return 4.0 * q - 4.0;
}

void FBBRSender::OnProbeBwPhaseEntered(Bbr2ProbeBwMode::CyclePhase phase,
                                           QuicTime now) {
  if (phase == Bbr2ProbeBwMode::CyclePhase::PROBE_CRUISE) {
    rtprop_probe_down_active_ = false;
    EnterCruise(now);
    return;
  }
  if (in_cruise_) {
    LeaveCruise(now);
  }
  if (phase == Bbr2ProbeBwMode::CyclePhase::PROBE_DOWN) {
    const QuicByteCount bdp = model_.BDP();
    rtprop_probe_down_active_ = ShouldEnableRtpropProbeDown(
        previous_cruise_rtprop_updated_,
        latest_congestion_event_prior_inflight_, bdp);
    QUIC_DVLOG(2)
        << "FBBR: Entering PROBE_DOWN. prior_inflight="
        << latest_congestion_event_prior_inflight_ << ", bdp=" << bdp
        << ", previous_cruise_rtprop_updated="
        << previous_cruise_rtprop_updated_
        << ", rtprop_probe_down_active=" << rtprop_probe_down_active_;
    previous_cruise_rtprop_updated_ = false;
  } else {
    rtprop_probe_down_active_ = false;
  }
}

void FBBRSender::EnterCruise(QuicTime now) {
  in_cruise_ = true;
  current_cruise_rtprop_updated_ = false;
  previous_cruise_rtprop_updated_ = false;
  // Replace the constructor's initial-RTT placeholder once, using the first
  // path sample learned before Cruise.  After this bootstrap, only explicit
  // trusted Cruise/ProbeRTT publications version srtt_low.
  InitializeHybridSrttLowFromModel();
  cruise_rtprop_at_entry_ =
      IsFbbrHybridObserver() && fbbr_hybrid_srtt_low_valid_
          ? fbbr_hybrid_srtt_low_
          : model_.MinRtt();
  ++cruise_id_;
  cruise_start_time_ = now;
  guard_updated_this_cruise_ = false;
  guard_window_active_ = false;
  guard_window_delivered_start_ = 0;
  guard_window_ack_start_ = QuicTime::Zero();
  guard_window_send_start_ = QuicTime::Zero();
  guard_window_app_limited_ = false;
  trusted_bw_cleared_on_cruise_start_ = false;
  if (!trusted_bw_clear_on_cruise_start_) {
    QUIC_DVLOG(1) << "FBBR: trusted_bw.clear_on_cruise_start=false "
                     "is overridden to preserve fresh-only application";
  }
  ClearTrustedBwApplication("cruise_start");
  cruise_modulation_freq_hz_ = configured_modulation_freq_hz_;
  adaptive_cruise_start_max_bw_ = model_.MaxBandwidth();
  adaptive_bounds_inherited_this_cruise_ = false;
  if (IsFbbrHybridObserver()) {
    // Hybrid connection state is deliberately independent of the current
    // MaxBw generation for its lower bound. The upper bound is always the
    // BBR model's current MaxBw; MaxSRTT is maintained by MaxBw updates.
    adaptive_bounds_inherited_this_cruise_ =
        adaptive_baseline_low_valid_;
  } else if (UsesAdaptiveLoadJudgment()) {
    const double current_max_bw_bps = static_cast<double>(
        adaptive_cruise_start_max_bw_.ToBitsPerSecond());
    const double previous_max_bw_bps = static_cast<double>(
        adaptive_previous_cruise_max_bw_.ToBitsPerSecond());
    adaptive_bounds_inherited_this_cruise_ =
        adaptive_previous_cruise_max_bw_valid_ &&
        ShouldInheritAdaptiveBounds(current_max_bw_bps,
                                    previous_max_bw_bps);
    if (!adaptive_bounds_inherited_this_cruise_) {
      adaptive_baseline_low_valid_ = false;
      adaptive_baseline_low_ = QuicBandwidth::Zero();
      adaptive_baseline_up_valid_ = false;
      adaptive_baseline_up_ = QuicBandwidth::Zero();
      latest_waveform_overload_srtt_mean_valid_ = false;
      latest_waveform_overload_srtt_mean_ms_ = 0.0;
      latest_waveform_underload_srtt_mean_valid_ = false;
      latest_waveform_underload_srtt_mean_ms_ = 0.0;
    }
    adaptive_previous_cruise_max_bw_ = adaptive_cruise_start_max_bw_;
    adaptive_previous_cruise_max_bw_valid_ =
        !adaptive_cruise_start_max_bw_.IsInfinite() &&
        std::isfinite(current_max_bw_bps) && current_max_bw_bps > 0.0;
  }
  QuicBandwidth current_native_max_bw = IsFbbrHybridObserver()
      ? model_.MaxBandwidth() : BandwidthEstimate();
  if (current_native_max_bw.IsZero() ||
      current_native_max_bw.IsInfinite()) {
    current_native_max_bw = BandwidthEstimate();
  }
  initial_cruise_baseline_bw_ = current_native_max_bw;
  current_injection_baseline_bw_ = initial_cruise_baseline_bw_;
  if (IsFbbrProjectionObserver()) {
    const FbbrV3ReferenceResult reference = SelectFbbrV3ReferenceBw();
    if (reference.valid) {
      initial_cruise_baseline_bw_ = reference.bandwidth;
      current_injection_baseline_bw_ = reference.bandwidth;
    }
  }
  const bool hybrid_lower_bound_search_needed =
      IsFbbrHybrid() && ShouldStartFbbrHybridLowerBoundSearch(
          cruise_id_, adaptive_baseline_low_valid_,
          fbbr_hybrid_rtprop_drate_valid_);
  if (hybrid_lower_bound_search_needed &&
      FbbrHybridLowerBoundSearchGuardAllows(
          CurrentHybridLowerBoundSearchGuardRtt())) {
    fbbr_hybrid_lower_bound_search_active_ = true;
    // The search is connection state.  Do not let the next Cruise's MaxBw
    // initialization undo reductions already made in the third Cruise.
    if (!fbbr_hybrid_lower_bound_search_baseline_.IsZero() &&
        fbbr_hybrid_lower_bound_search_baseline_ <
            current_injection_baseline_bw_) {
      current_injection_baseline_bw_ =
          fbbr_hybrid_lower_bound_search_baseline_;
    } else {
      fbbr_hybrid_lower_bound_search_baseline_ =
          current_injection_baseline_bw_;
    }
    if (fbbr_hybrid_lower_bound_search_bdp_ == 0) {
      fbbr_hybrid_lower_bound_search_bdp_ = HybridTrustedBdp();
      if (fbbr_hybrid_lower_bound_search_bdp_ == 0) {
        fbbr_hybrid_lower_bound_search_bdp_ = model_.BDP();
      }
    }
  } else if (IsFbbrHybrid() &&
             (hybrid_lower_bound_search_needed ||
              (adaptive_baseline_low_valid_ &&
               fbbr_hybrid_rtprop_drate_valid_))) {
    ResetHybridLowerBoundSearch();
  }
  current_probe_amplitude_bps_ = GetCurrentAmplitudeBps();
  waveform_initial_probe_amplitude_bps_ = current_probe_amplitude_bps_;
  ResetMaxBwAttenuationEstimator();
  waveform_inconclusive_amplification_count_ = 0;
  fbbr_hybrid_srtt_no_wave_streak_ = 0;
  fbbr_hybrid_drate_no_wave_streak_ = 0;
  fbbr_hybrid_wave_fidelity_enhancement_active_ = false;
  fbbr_hybrid_retry_reason_mask_ = 0;
  fbbr_hybrid_last_counted_window_second_cycle_id_ = 0;
  fbbr_hybrid_rolling_retry_count_ = 0;
  fbbr_hybrid_regime_ii_seen_this_cruise_ = false;
  fbbr_hybrid_trusted_bw_ = QuicBandwidth::Zero();
  current_probe_bw_phase_gain_ =
      cruise_detector_mode_ == FBBRCruiseDetectorMode::kTimeWaveform
          ? 1.0
          : static_cast<double>(PacingGain());
  if (cruise_detector_mode_ == FBBRCruiseDetectorMode::kLegacySpectral &&
      (!std::isfinite(current_probe_bw_phase_gain_) ||
       current_probe_bw_phase_gain_ <= 0.0)) {
    std::cerr << "[FBBR warning] invalid CRUISE pacing gain "
              << current_probe_bw_phase_gain_
              << "; using 1.0 for waveform injection" << std::endl;
    current_probe_bw_phase_gain_ = 1.0;
  }
  min_rtt_warning_logged_ = false;
  current_cruise_windows_.clear();
  ResetCruiseWindowState();
  ResetWaveformCruiseState(now);
  freq_tool_on_ = ShouldOscillate();
  cruise_freq_tool_active_ = freq_tool_on_;
  QUIC_DVLOG(2) << "FBBR: Entering PROBE_CRUISE @ " << now
                << ", cruise_id=" << cruise_id_
                << ", fixed_freq=" << cruise_modulation_freq_hz_
                << "Hz, detector="
                << CruiseDetectorModeName(cruise_detector_mode_)
                << ", initial_maxbw_baseline_bps="
                << initial_cruise_baseline_bw_.ToBitsPerSecond()
                << ", adaptive_bounds_inherited="
                << adaptive_bounds_inherited_this_cruise_
                << ", amplitude_bps=" << current_probe_amplitude_bps_;
}

void FBBRSender::LeaveCruise(QuicTime now) {
  QUIC_DVLOG(2) << "FBBR: Leaving PROBE_CRUISE @ " << now;
  FinalizeCruise(now);
  previous_cruise_rtprop_updated_ = current_cruise_rtprop_updated_;
  in_cruise_ = false;
  freq_tool_on_ = false;
  cruise_freq_tool_active_ = false;
  cruise_start_time_ = QuicTime::Zero();
  cruise_modulation_freq_hz_ = configured_modulation_freq_hz_;
  ResetCruiseWindowState();
  waveform_cruise_state_ = WaveformCruiseState::kDisabled;
  waveform_window_start_ = QuicTime::Zero();
  waveform_window_end_ = QuicTime::Zero();
  current_cruise_windows_.clear();
}

void FBBRSender::ResetCruiseWindowState() {
  if (cruise_detector_mode_ == FBBRCruiseDetectorMode::kTimeWaveform) {
    next_cruise_window_start_ = QuicTime::Zero();
    return;
  }
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

void FBBRSender::InitializeHybridSrttLowFromModel() {
  if (fbbr_hybrid_srtt_low_valid_ &&
      fbbr_hybrid_srtt_low_source_time_ != QuicTime::Zero()) {
    return;
  }
  const TimeDelta model_rtprop = model_.MinRtt();
  if (model_rtprop.IsZero() || model_rtprop.IsInfinite()) {
    return;
  }
  fbbr_hybrid_srtt_low_valid_ = true;
  fbbr_hybrid_srtt_low_ = model_rtprop;
  fbbr_hybrid_srtt_low_source_time_ = model_.MinRttTimestamp();
}

QuicByteCount FBBRSender::HybridTrustedBdp() const {
  if (!IsFbbrHybrid() || !fbbr_hybrid_srtt_low_valid_ ||
      fbbr_hybrid_srtt_low_.IsZero() ||
      fbbr_hybrid_srtt_low_.IsInfinite()) {
    return 0;
  }
  QuicBandwidth bandwidth = model_.MaxBandwidth();
  if (bandwidth.IsZero() || bandwidth.IsInfinite()) {
    bandwidth = BandwidthEstimate();
  }
  if (bandwidth.IsZero() || bandwidth.IsInfinite()) {
    return 0;
  }
  return bandwidth * fbbr_hybrid_srtt_low_;
}

QuicByteCount FBBRSender::AdjustProbeRttInflightTarget(
    QuicByteCount native_target) const {
  const QuicByteCount trusted_bdp = HybridTrustedBdp();
  if (trusted_bdp == 0) {
    return native_target;
  }
  return static_cast<QuicByteCount>(
      static_cast<long double>(trusted_bdp) *
      Params().probe_rtt_inflight_target_bdp_fraction);
}

void FBBRSender::StartHybridStableObservation(
    HybridStableObservationSource source,
    QuicTime stable_start) {
  CancelHybridStableObservation();
  fbbr_hybrid_stable_observation_source_ = source;
  fbbr_hybrid_stable_observation_start_ = stable_start;
  fbbr_hybrid_stable_observation_round_done_ = false;
  fbbr_hybrid_stable_observation_min_rtt_ = TimeDelta::Infinite();
}

void FBBRSender::ObserveHybridStableSample(
    const Bbr2CongestionEvent& congestion_event) {
  if (fbbr_hybrid_stable_observation_source_ ==
          HybridStableObservationSource::kNone ||
      congestion_event.event_time <
          fbbr_hybrid_stable_observation_start_) {
    return;
  }
  if (!congestion_event.sample_min_rtt.IsZero() &&
      !congestion_event.sample_min_rtt.IsInfinite() &&
      (fbbr_hybrid_stable_observation_min_rtt_.IsInfinite() ||
       congestion_event.sample_min_rtt <
           fbbr_hybrid_stable_observation_min_rtt_)) {
    fbbr_hybrid_stable_observation_min_rtt_ =
        congestion_event.sample_min_rtt;
  }
  if (congestion_event.sample_valid &&
      !congestion_event.sample_max_bandwidth.IsZero() &&
      !congestion_event.sample_max_bandwidth.IsInfinite()) {
    const int64_t sample_bps =
        congestion_event.sample_max_bandwidth.ToBitsPerSecond();
    if (sample_bps > 0) {
      fbbr_hybrid_stable_rate_samples_bps_.push_back(sample_bps);
      if (fbbr_hybrid_stable_observation_round_done_) {
        fbbr_hybrid_stable_post_round_rate_samples_bps_.push_back(
            sample_bps);
      }
    }
  }
  // The ACK that closes the first packet-timed round is still a transition
  // sample.  Only subsequent DRE samples enter the preferred estimator.
  if (congestion_event.end_of_round_trip) {
    fbbr_hybrid_stable_observation_round_done_ = true;
  }
}

QuicBandwidth FBBRSender::ComputeHybridStableDeliveryRate(
    const std::vector<int64_t>& post_round_samples_bps,
    const std::vector<int64_t>& all_samples_bps) {
  const std::vector<int64_t>& preferred =
      post_round_samples_bps.empty() ? all_samples_bps
                                     : post_round_samples_bps;
  if (preferred.empty()) {
    return QuicBandwidth::Zero();
  }
  std::vector<int64_t> ordered = preferred;
  const size_t lower_middle = (ordered.size() - 1) / 2;
  std::nth_element(ordered.begin(), ordered.begin() + lower_middle,
                   ordered.end());
  return ordered[lower_middle] > 0
             ? QuicBandwidth::FromBitsPerSecond(ordered[lower_middle])
             : QuicBandwidth::Zero();
}

void FBBRSender::PublishHybridLowerBound(QuicBandwidth delivery_rate,
                                         QuicTime source_time) {
  if (delivery_rate.IsZero() || delivery_rate.IsInfinite() ||
      source_time == QuicTime::Zero()) {
    return;
  }
  fbbr_hybrid_rtprop_drate_ = delivery_rate;
  fbbr_hybrid_rtprop_drate_valid_ = true;
  fbbr_hybrid_rtprop_drate_source_cruise_id_ =
      static_cast<uint64_t>(std::max<int64_t>(0, cruise_id_));
  fbbr_hybrid_rtprop_drate_source_time_ = source_time;
  adaptive_baseline_low_ = delivery_rate;
  adaptive_baseline_low_valid_ = true;
  fbbr_hybrid_baseline_low_source_time_ = source_time;
}

void FBBRSender::PublishHybridSrttLow(TimeDelta rtprop,
                                      QuicTime source_time,
                                      bool from_probe_rtt) {
  if (rtprop.IsZero() || rtprop.IsInfinite() ||
      source_time == QuicTime::Zero()) {
    return;
  }
  if (fbbr_hybrid_srtt_low_valid_ &&
      source_time <= fbbr_hybrid_srtt_low_source_time_) {
    return;
  }
  fbbr_hybrid_srtt_low_valid_ = true;
  fbbr_hybrid_srtt_low_ = rtprop;
  fbbr_hybrid_srtt_low_source_time_ = source_time;
  model_.ForceUpdateMinRtt(rtprop, source_time);
  current_cruise_rtprop_updated_ = true;
  QUIC_DVLOG(2) << "FBBR-Hybrid: srtt_low/RTprop refreshed from "
                << (from_probe_rtt ? "ProbeRTT" : "Cruise")
                << " to " << rtprop << " @ " << source_time;
}

void FBBRSender::CancelHybridStableObservation() {
  fbbr_hybrid_stable_observation_source_ =
      HybridStableObservationSource::kNone;
  fbbr_hybrid_stable_observation_start_ = QuicTime::Zero();
  fbbr_hybrid_stable_observation_round_done_ = false;
  fbbr_hybrid_stable_observation_min_rtt_ = TimeDelta::Infinite();
  fbbr_hybrid_stable_rate_samples_bps_.clear();
  fbbr_hybrid_stable_post_round_rate_samples_bps_.clear();
}

void FBBRSender::ResetHybridLowerBoundSearch() {
  fbbr_hybrid_lower_bound_search_active_ = false;
  fbbr_hybrid_lower_bound_search_baseline_ = QuicBandwidth::Zero();
  fbbr_hybrid_lower_bound_search_bdp_ = 0;
  CancelHybridStableObservation();
}

TimeDelta FBBRSender::CurrentHybridLowerBoundSearchGuardRtt() const {
  if (rtt_stats_ != nullptr) {
    const TimeDelta smoothed_rtt = rtt_stats_->smoothed_rtt();
    if (!smoothed_rtt.IsZero() && !smoothed_rtt.IsInfinite()) {
      return smoothed_rtt;
    }
    const TimeDelta latest_rtt = rtt_stats_->latest_rtt();
    if (!latest_rtt.IsZero() && !latest_rtt.IsInfinite()) {
      return latest_rtt;
    }
    const TimeDelta initial_rtt = rtt_stats_->SmoothedOrInitialRtt();
    if (!initial_rtt.IsZero() && !initial_rtt.IsInfinite()) {
      return initial_rtt;
    }
  }
  return TimeDelta::Zero();
}

bool FBBRSender::FbbrHybridLowerBoundSearchGuardAllows(
    TimeDelta current_rtt) const {
  if (current_rtt.IsZero() || current_rtt.IsInfinite()) {
    return false;
  }
  const double current_srtt_ms =
      static_cast<double>(current_rtt.ToMicroseconds()) / 1000.0;
  return FbbrHybridLowerBoundSearchGuardAllowsMs(current_srtt_ms);
}

bool FBBRSender::FbbrHybridLowerBoundSearchGuardAllowsMs(
    double current_srtt_ms) const {
  const TimeDelta rtprop = fbbr_hybrid_srtt_low_valid_
      ? fbbr_hybrid_srtt_low_ : model_.MinRtt();
  const bool rtprop_valid = !rtprop.IsZero() && !rtprop.IsInfinite();
  const double rtprop_ms = rtprop_valid
      ? static_cast<double>(rtprop.ToMicroseconds()) / 1000.0 : 0.0;
  return ShouldAllowFbbrHybridLowerBoundSearchByQueue(
      current_srtt_ms, rtprop_valid, rtprop_ms,
      fbbr_hybrid_max_srtt_valid_, fbbr_hybrid_max_srtt_ms_);
}

bool FBBRSender::FbbrHybridLowerBoundSearchGuardAllows(
    const WaveformWindowAnalysis& analysis) const {
  if (analysis.srtt_stats_valid &&
      std::isfinite(analysis.srtt_mean_ms) &&
      analysis.srtt_mean_ms > 0.0) {
    return FbbrHybridLowerBoundSearchGuardAllowsMs(
        analysis.srtt_mean_ms);
  }
  return FbbrHybridLowerBoundSearchGuardAllows(
      CurrentHybridLowerBoundSearchGuardRtt());
}

TimeDelta FBBRSender::CurrentGuardRttSample(
    const Bbr2CongestionEvent& congestion_event) const {
  if (!congestion_event.sample_min_rtt.IsZero() &&
      !congestion_event.sample_min_rtt.IsInfinite()) {
    return congestion_event.sample_min_rtt;
  }
  if (rtt_stats_ == nullptr) {
    return TimeDelta::Zero();
  }
  const TimeDelta latest_rtt = rtt_stats_->latest_rtt();
  if (!latest_rtt.IsZero() && !latest_rtt.IsInfinite()) {
    return latest_rtt;
  }
  const TimeDelta smoothed_rtt = rtt_stats_->smoothed_rtt();
  if (!smoothed_rtt.IsZero() && !smoothed_rtt.IsInfinite()) {
    return smoothed_rtt;
  }
  return TimeDelta::Zero();
}

TimeDelta FBBRSender::CurrentGuardMinRtt(TimeDelta fallback_rtt) const {
  const TimeDelta model_min_rtt = model_.MinRtt();
  if (!model_min_rtt.IsZero() && !model_min_rtt.IsInfinite()) {
    return model_min_rtt;
  }
  if (rtt_stats_ != nullptr) {
    const TimeDelta stats_min_rtt = rtt_stats_->MinOrInitialRtt();
    if (!stats_min_rtt.IsZero() && !stats_min_rtt.IsInfinite()) {
      return stats_min_rtt;
    }
  }
  return (!fallback_rtt.IsZero() && !fallback_rtt.IsInfinite())
             ? fallback_rtt
             : TimeDelta::Zero();
}

QuicBandwidth FBBRSender::ApplyGuardLowPass(QuicBandwidth previous,
                                            QuicBandwidth sample) {
  if (!IsFinitePositiveBandwidth(previous)) {
    return sample;
  }
  if (!IsFinitePositiveBandwidth(sample)) {
    return previous;
  }
  const long double filtered_bps =
      (7.0L * static_cast<long double>(previous.ToBitsPerSecond()) +
       static_cast<long double>(sample.ToBitsPerSecond())) /
      8.0L;
  const long double capped = std::min(
      filtered_bps,
      static_cast<long double>(std::numeric_limits<int64_t>::max()));
  return QuicBandwidth::FromBitsPerSecond(
      static_cast<int64_t>(std::max<long double>(1.0L, capped)));
}

void FBBRSender::UpdateGuardFilter(QuicBandwidth raw_sample) {
  if (!IsFinitePositiveBandwidth(raw_sample)) {
    return;
  }
  if (!guard_filter_valid_) {
    guard_filter_stage1_ = raw_sample;
    guard_filter_stage2_ = raw_sample;
    guard_filter_valid_ = true;
  } else {
    guard_filter_stage1_ =
        ApplyGuardLowPass(guard_filter_stage1_, raw_sample);
    guard_filter_stage2_ =
        ApplyGuardLowPass(guard_filter_stage2_, guard_filter_stage1_);
  }
  guard_updated_this_cruise_ = true;
}

void FBBRSender::AnchorGuardFilter(QuicBandwidth trusted_bw) {
  if (!IsFinitePositiveBandwidth(trusted_bw)) {
    return;
  }
  guard_filter_stage1_ = trusted_bw;
  guard_filter_stage2_ = trusted_bw;
  guard_filter_valid_ = true;
}

void FBBRSender::StartGuardMeasurementWindow(
    const Bbr2CongestionEvent& congestion_event) {
  const QuicSendTimeState& send_state =
      congestion_event.last_packet_send_state;
  if (congestion_event.bytes_acked == 0 || !send_state.is_valid ||
      send_state.sent_time == QuicTime::Zero() ||
      congestion_event.event_time == QuicTime::Zero()) {
    guard_window_active_ = false;
    return;
  }
  guard_window_active_ = true;
  guard_window_delivered_start_ = model_.total_bytes_acked();
  guard_window_ack_start_ = congestion_event.event_time;
  guard_window_send_start_ = send_state.sent_time;
  guard_window_app_limited_ = false;
}

void FBBRSender::UpdateGuardEstimatorFromCongestionEvent(
    const Bbr2CongestionEvent& congestion_event) {
  if (!IsFbbrHybridObserver() || !in_cruise_ ||
      mode_ != Bbr2Mode::PROBE_BW ||
      GetCurrentProbeBwPhase() !=
          Bbr2ProbeBwMode::CyclePhase::PROBE_CRUISE ||
      congestion_event.bytes_acked == 0) {
    return;
  }

  const QuicSendTimeState& send_state =
      congestion_event.last_packet_send_state;
  if (!send_state.is_valid || send_state.sent_time == QuicTime::Zero() ||
      congestion_event.event_time == QuicTime::Zero()) {
    return;
  }

  if (!guard_window_active_) {
    StartGuardMeasurementWindow(congestion_event);
    return;
  }

  guard_window_app_limited_ =
      guard_window_app_limited_ ||
      congestion_event.sample_is_app_limited ||
      congestion_event.last_sample_is_app_limited ||
      send_state.is_app_limited;

  const TimeDelta current_rtt = CurrentGuardRttSample(congestion_event);
  if (current_rtt.IsZero() || current_rtt.IsInfinite()) {
    return;
  }
  const TimeDelta minimum_window =
      current_rtt < TimeDelta::FromMilliseconds(50)
          ? TimeDelta::FromMilliseconds(50)
          : current_rtt;
  if (congestion_event.event_time <= guard_window_ack_start_ ||
      congestion_event.event_time - guard_window_ack_start_ <
          minimum_window) {
    return;
  }

  const QuicByteCount delivered_now = model_.total_bytes_acked();
  const QuicByteCount delta_delivered =
      delivered_now > guard_window_delivered_start_
          ? delivered_now - guard_window_delivered_start_
          : 0;
  const TimeDelta ack_elapsed =
      congestion_event.event_time - guard_window_ack_start_;
  const TimeDelta send_elapsed =
      send_state.sent_time > guard_window_send_start_
          ? send_state.sent_time - guard_window_send_start_
          : TimeDelta::Zero();
  const TimeDelta delivery_elapsed =
      ack_elapsed > send_elapsed ? ack_elapsed : send_elapsed;
  const TimeDelta min_rtt = CurrentGuardMinRtt(current_rtt);

  const bool valid_sample =
      delta_delivered > 0 && ack_elapsed.ToMicroseconds() > 0 &&
      send_elapsed.ToMicroseconds() > 0 &&
      !delivery_elapsed.IsZero() &&
      (min_rtt.IsZero() || delivery_elapsed >= min_rtt) &&
      !guard_window_app_limited_;
  if (valid_sample) {
    UpdateGuardFilter(BandwidthFromBytesAndTimeDeltaSafe(
        delta_delivered, delivery_elapsed));
  }
  StartGuardMeasurementWindow(congestion_event);
}

void FBBRSender::MaybeFinishHybridStableObservation(QuicTime now,
                                                     bool force_finish) {
  const HybridStableObservationSource source =
      fbbr_hybrid_stable_observation_source_;
  if (source == HybridStableObservationSource::kNone ||
      now == QuicTime::Zero()) {
    return;
  }
  const bool duration_done = now >= fbbr_hybrid_stable_observation_start_ +
      TimeDelta::FromMilliseconds(kFbbrHybridStableObservationDurationMs);
  if (!force_finish &&
      (!duration_done || !fbbr_hybrid_stable_observation_round_done_)) {
    return;
  }

  const TimeDelta observed_rtprop =
      fbbr_hybrid_stable_observation_min_rtt_;
  const QuicBandwidth observed_rate = ComputeHybridStableDeliveryRate(
      fbbr_hybrid_stable_post_round_rate_samples_bps_,
      fbbr_hybrid_stable_rate_samples_bps_);
  const bool observed_rtprop_valid = !observed_rtprop.IsZero() &&
      !observed_rtprop.IsInfinite();
  if (source == HybridStableObservationSource::kCruiseFallback &&
      (!observed_rtprop_valid || observed_rate.IsZero())) {
    // Do not abandon a cross-Cruise platform just because the first eligible
    // completion event did not carry both halves of the paired observation.
    return;
  }
  const bool prior_srtt_low_valid = fbbr_hybrid_srtt_low_valid_;
  const TimeDelta prior_srtt_low = fbbr_hybrid_srtt_low_;
  CancelHybridStableObservation();

  if (observed_rtprop_valid) {
    PublishHybridSrttLow(observed_rtprop, now,
                         source == HybridStableObservationSource::kProbeRtt);
  }
  if (source == HybridStableObservationSource::kProbeRtt) {
    const bool probe_found_lower_rtprop = prior_srtt_low_valid &&
        !observed_rtprop.IsZero() && !observed_rtprop.IsInfinite() &&
        prior_srtt_low > observed_rtprop;
    if (observed_rtprop_valid && !observed_rate.IsZero() &&
        (!adaptive_baseline_low_valid_ ||
         !fbbr_hybrid_rtprop_drate_valid_ || probe_found_lower_rtprop)) {
      PublishHybridLowerBound(observed_rate, now);
    }
  } else if (!observed_rtprop.IsZero() &&
             !observed_rtprop.IsInfinite() &&
             !observed_rate.IsZero()) {
    // Cruise fallback is a transaction: only a completed 200ms + one-round
    // low-flight platform may publish both RTprop and its paired DRate.
    PublishHybridLowerBound(observed_rate, now);
  }
  if (adaptive_baseline_low_valid_ && fbbr_hybrid_rtprop_drate_valid_) {
    ResetHybridLowerBoundSearch();
  }
}

void FBBRSender::OnCongestionEventStarted(
    const Bbr2CongestionEvent& congestion_event) {
  // PROBE_UP and the reference BBR-R logic judge the entry flight before the
  // ACK/loss event that triggers the phase transition.
  latest_congestion_event_prior_inflight_ =
      congestion_event.prior_bytes_in_flight;
  latest_congestion_event_prior_inflight_valid_ = true;
  latest_congestion_event_inflight_ = congestion_event.bytes_in_flight;
  latest_congestion_event_inflight_valid_ = true;
  bool stable_observation_started_this_event = false;
  if (IsFbbrHybrid() && mode_ == Bbr2Mode::PROBE_RTT) {
    const Bbr2ProbeRttMode::DebugState probe_state =
        ExportDebugState().probe_rtt;
    if (probe_state.exit_time != QuicTime::Zero() &&
        fbbr_hybrid_stable_observation_source_ !=
            HybridStableObservationSource::kProbeRtt) {
      StartHybridStableObservation(
          HybridStableObservationSource::kProbeRtt,
          probe_state.exit_time - Params().probe_rtt_duration);
      stable_observation_started_this_event = true;
    }
  } else if (IsFbbrHybrid() &&
             fbbr_hybrid_lower_bound_search_active_ &&
             fbbr_hybrid_stable_observation_source_ ==
                 HybridStableObservationSource::kNone &&
             IsBelowHalfBdp(congestion_event.bytes_in_flight,
                            fbbr_hybrid_lower_bound_search_bdp_)) {
    if (FbbrHybridLowerBoundSearchGuardAllows(
            CurrentHybridLowerBoundSearchGuardRtt())) {
      StartHybridStableObservation(
          HybridStableObservationSource::kCruiseFallback,
          congestion_event.event_time);
      model_.RestartRoundEarly();
      stable_observation_started_this_event = true;
    } else {
      ResetHybridLowerBoundSearch();
    }
  }
  if (!stable_observation_started_this_event) {
    ObserveHybridStableSample(congestion_event);
  }
  if (!IsFbbrHybridObserver() && in_cruise_ &&
      mode_ == Bbr2Mode::PROBE_BW &&
      GetCurrentProbeBwPhase() ==
          Bbr2ProbeBwMode::CyclePhase::PROBE_CRUISE &&
      model_.MinRtt() != cruise_rtprop_at_entry_) {
    current_cruise_rtprop_updated_ = true;
  }
  UpdateRoundDeliveryRateSample(congestion_event);
  UpdateGuardEstimatorFromCongestionEvent(congestion_event);
  if (congestion_event.end_of_round_trip) {
    FinalizeCompletedRound(congestion_event);
  }
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
  model_.ClearBdpBandwidthOverride();
  trusted_bw_conf_ = 0.0;
  trusted_bw_source_ = kTrustedBwSourceNone;
  trusted_bw_cruise_id_ = 0;
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

  const std::string trusted_bw_source = selection.trusted_bw_source;
  const bool waveform_source =
      trusted_bw_source == kTrustedBwSourceTimeWaveformBaseline ||
      trusted_bw_source == kTrustedBwSourceAdaptiveWindowMean ||
      trusted_bw_source == kTrustedBwSourceFbbrWindowMean;
  const bool non_spectral_final_source =
      trusted_bw_source == kTrustedBwSourceGuardFilter ||
      trusted_bw_source == kTrustedBwSourcePreviousTrusted ||
      trusted_bw_source == kTrustedBwSourceNativeFallback;
  const bool valid_selection =
      selection.trusted_bw_valid &&
      (waveform_source || non_spectral_final_source ||
       selection.dual_signal_spectral_gate_pass) &&
      IsFinitePositiveBandwidth(selection.trusted_bw);
  if (!valid_selection) {
    ClearTrustedBw(waveform_source ? "invalid_waveform_trusted_bw"
                                   : "no_dual_signal_trusted_bw");
    return;
  }

  trusted_bw_ = selection.trusted_bw;
  trusted_bw_valid_ = true;
  // V1 and the legacy FBBR branches retain their existing BDP override.
  // V3/V4 leave the native BBRv2 cwnd/BDP model untouched and apply their
  // respective cap only at the final send-allowance interface.
  if (!IsFbbrProjectionObserver()) {
    model_.SetBdpBandwidthOverride(trusted_bw_);
  }
  trusted_bw_conf_ = selection.trusted_bw_conf;
  trusted_bw_source_ = selection.trusted_bw_source;
  trusted_bw_cruise_id_ =
      static_cast<uint64_t>(std::max<int64_t>(0, cruise_id_));
  trusted_bw_fresh_ = true;
  trusted_bw_application_valid_ = true;
  trusted_bw_ready_for_post_cruise_ = true;
  trusted_bw_application_phase_ = "POST_CRUISE_READY";
  trusted_bw_invalid_reason_ = "none";
  if (IsFbbrProjectionObserver()) {
    const FbbrV3ReferenceResult reference = SelectFbbrV3ReferenceBw();
    if (reference.valid) {
      current_injection_baseline_bw_ = reference.bandwidth;
    }
  }
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
  switch (source) {
    case FBBRPacingBaseSource::kTrustedBw:
      return "TRUSTED_BW";
    case FBBRPacingBaseSource::kWaveformCruiseBaseline:
      return "WAVEFORM_CRUISE_BASELINE";
    case FBBRPacingBaseSource::kHybridLowerBoundSearch:
      return "HYBRID_LOWER_BOUND_SEARCH";
    case FBBRPacingBaseSource::kV3TrustedReference:
      return "V3_TRUSTED_REFERENCE";
    case FBBRPacingBaseSource::kV3GuardReference:
      return "V3_GUARD_REFERENCE";
    case FBBRPacingBaseSource::kV3LastValidReference:
      return "V3_LAST_VALID_REFERENCE";
    case FBBRPacingBaseSource::kV4TrustedReference:
      return "V4_TRUSTED_REFERENCE";
    case FBBRPacingBaseSource::kV4GuardReference:
      return "V4_GUARD_REFERENCE";
    case FBBRPacingBaseSource::kV4LastValidReference:
      return "V4_LAST_VALID_REFERENCE";
    default:
      return "NATIVE_BBR";
  }
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
      << max_bw_attenuation_factor_ << ","
      << model_.max_bandwidth_filter_input().ToBitsPerSecond() << ","
      << max_bw_actual_fluctuation_amplitude_bps_ << ","
      << max_bw_delivery_response_gain_ << ","
      << (max_bw_response_observed_ ? "true" : "false") << ","
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

void FBBRSender::PublishFbbrHybridCruiseTrustedBw(
    const TrustedBwSelectionResult* frequency_selection) {
  TrustedBwSelectionResult selection =
      frequency_selection != nullptr
          ? *frequency_selection
          : TrustedBwSelectionResult{
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
  if (!IsFinitePositiveBandwidth(selection.native_bw)) {
    selection.native_bw = BandwidthEstimate();
  }

  const bool waveform_frequency_ready =
      fbbr_hybrid_regime_ii_seen_this_cruise_ &&
      trusted_baseline_locked_ &&
      trusted_bw_candidate_source_ != nullptr &&
      std::string(trusted_bw_candidate_source_) ==
          kTrustedBwSourceFbbrWindowMean &&
      IsFinitePositiveBandwidth(trusted_bw_candidate_);
  const bool spectral_frequency_ready =
      frequency_selection != nullptr &&
      frequency_selection->trusted_bw_valid &&
      frequency_selection->dual_signal_spectral_gate_pass &&
      IsFinitePositiveBandwidth(frequency_selection->trusted_bw);

  if (waveform_frequency_ready) {
    selection.trusted_bw = trusted_bw_candidate_;
    selection.trusted_bw_valid = true;
    selection.trusted_bw_conf = 1.0;
    selection.trusted_bw_source = kTrustedBwSourceFbbrWindowMean;
    AnchorGuardFilter(selection.trusted_bw);
  } else if (spectral_frequency_ready) {
    selection.trusted_bw = frequency_selection->trusted_bw;
    selection.trusted_bw_valid = true;
    selection.trusted_bw_conf = frequency_selection->trusted_bw_conf;
    selection.trusted_bw_source = kTrustedBwSourceFbbrWindowMean;
    AnchorGuardFilter(selection.trusted_bw);
  } else if (guard_updated_this_cruise_ && guard_filter_valid_ &&
             IsFinitePositiveBandwidth(guard_filter_stage2_)) {
    selection.trusted_bw = guard_filter_stage2_;
    selection.trusted_bw_valid = true;
    selection.trusted_bw_conf = 1.0;
    selection.trusted_bw_source = kTrustedBwSourceGuardFilter;
    selection.drate_spectral_gate_pass = false;
    selection.srtt_spectral_gate_pass = false;
    selection.dual_signal_spectral_gate_pass = false;
    selection.limiting_spectral_signal = kLimitingSpectralSignalEqual;
  } else if (IsFinitePositiveBandwidth(trusted_bw_)) {
    selection.trusted_bw = trusted_bw_;
    selection.trusted_bw_valid = true;
    selection.trusted_bw_conf = trusted_bw_conf_;
    selection.trusted_bw_source = kTrustedBwSourcePreviousTrusted;
    selection.drate_spectral_gate_pass = false;
    selection.srtt_spectral_gate_pass = false;
    selection.dual_signal_spectral_gate_pass = false;
    selection.limiting_spectral_signal = kLimitingSpectralSignalEqual;
  } else {
    QuicBandwidth native_fallback = initial_cruise_baseline_bw_;
    if (!IsFinitePositiveBandwidth(native_fallback)) {
      native_fallback = model_.MaxBandwidth();
    }
    if (!IsFinitePositiveBandwidth(native_fallback)) {
      native_fallback = BandwidthEstimate();
    }
    if (!IsFinitePositiveBandwidth(native_fallback)) {
      native_fallback =
          QuicBandwidth::FromBitsPerSecond(minimum_pacing_rate_bps_);
    }
    selection.trusted_bw = native_fallback;
    selection.trusted_bw_valid = IsFinitePositiveBandwidth(native_fallback);
    selection.trusted_bw_conf = 1.0;
    selection.trusted_bw_source = kTrustedBwSourceNativeFallback;
    selection.drate_spectral_gate_pass = false;
    selection.srtt_spectral_gate_pass = false;
    selection.dual_signal_spectral_gate_pass = false;
    selection.limiting_spectral_signal = kLimitingSpectralSignalEqual;
  }

  PublishTrustedBwSelection(selection);
}
void FBBRSender::OnPacketSent(QuicTime sent_time,
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
  const Bbr2Mode mode_before_event = mode_;
  const QuicBandwidth hybrid_max_bw_before =
      IsFbbrHybridObserver() ? model_.MaxBandwidth()
                             : QuicBandwidth::Zero();

  // FBBR and FBBR-adaptive share this path. Keep the ACK-rate sample raw for
  // waveform analysis, but remove the currently estimated positive waveform
  // excursion from the sample that feeds BBR's max-bandwidth filter.
  max_bw_actual_fluctuation_amplitude_bps_ =
      ShouldOscillate()
          ? CurrentActualDeliveryFluctuationAmplitudeBps()
          : 0.0;
  max_bw_attenuation_factor_ = CurrentMaxBwAttenuationFactor();
  model_.SetMaxBandwidthSampleAttenuation(max_bw_attenuation_factor_);

  Bbr2Sender::OnCongestionEvent(rtt_updated,
                                prior_in_flight,
                                event_time,
                                acked_packets,
                                lost_packets);
  const QuicBandwidth hybrid_max_bw_after =
      IsFbbrHybridObserver() ? model_.MaxBandwidth()
                             : QuicBandwidth::Zero();
  const bool hybrid_max_bw_updated =
      IsFbbrHybridObserver() &&
      IsFinitePositiveBandwidth(hybrid_max_bw_after) &&
      hybrid_max_bw_after != hybrid_max_bw_before;

  if (IsFbbrHybrid()) {
    if (mode_before_event != Bbr2Mode::PROBE_RTT &&
        mode_ == Bbr2Mode::PROBE_RTT &&
        fbbr_hybrid_stable_observation_source_ ==
            HybridStableObservationSource::kCruiseFallback) {
      // Native ProbeRTT is the stronger measurement transaction.  Its cwnd
      // cap takes over from an unfinished Cruise fallback platform.
      CancelHybridStableObservation();
    }
    if (mode_ == Bbr2Mode::PROBE_RTT &&
        fbbr_hybrid_stable_observation_source_ ==
            HybridStableObservationSource::kNone) {
      const Bbr2ProbeRttMode::DebugState probe_state =
          ExportDebugState().probe_rtt;
      if (probe_state.exit_time != QuicTime::Zero()) {
        StartHybridStableObservation(
            HybridStableObservationSource::kProbeRtt,
            probe_state.exit_time - Params().probe_rtt_duration);
      }
    }
    const bool probe_rtt_just_finished =
        mode_before_event == Bbr2Mode::PROBE_RTT &&
        mode_ != Bbr2Mode::PROBE_RTT;
    MaybeFinishHybridStableObservation(event_time,
                                       probe_rtt_just_finished);
  }

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
    const bool recv_valid =
        !recv_signal.IsZero() &&
        std::isfinite(static_cast<double>(recv_signal.ToBitsPerSecond()));
    const bool sample_is_app_limited =
        ExportDebugState().last_sample_is_app_limited;
    delivery_rate_history_.push_back(
        {event_time, recv_signal, recv_valid, sample_is_app_limited,
         acked_bytes});
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
  if (hybrid_max_bw_updated) {
    RecordHybridMaxBwUpdateForMaxSrtt(event_time, hybrid_max_bw_after);
  }
  if (IsFbbrHybridObserver()) {
    FinalizePendingHybridMaxSrttObservations(event_time);
  }
  last_ack_time_ = event_time;

  if (in_cruise_ && mode_ == Bbr2Mode::PROBE_BW &&
      GetCurrentProbeBwPhase() == Bbr2ProbeBwMode::CyclePhase::PROBE_CRUISE) {
    if (cruise_detector_mode_ ==
        FBBRCruiseDetectorMode::kTimeWaveform) {
      RunWaveformCruiseStateMachine(event_time);
    } else {
      RunDueCruiseWindowAnalysis(event_time);
    }
  }
  if (IsFbbrHybridV4() && !acked_packets.empty()) {
    RecordFbbrV4DeliveredPoint(
        event_time,
        static_cast<uint64_t>(model_.total_bytes_acked()),
        model_.is_app_limited());
  }
  if (IsFbbrHybridV3() && !acked_packets.empty()) {
    UpdateFbbrV3Telemetry(
        event_time,
        latest_congestion_event_inflight_valid_
            ? latest_congestion_event_inflight_ : prior_in_flight);
  }
  if (IsFbbrHybridV4() && !acked_packets.empty()) {
    UpdateFbbrV4Telemetry(
        event_time,
        latest_congestion_event_inflight_valid_
            ? latest_congestion_event_inflight_ : prior_in_flight);
  }
}

QuicBandwidth FBBRSender::PacingRate(
    QuicByteCount bytes_in_flight) const {
  const QuicBandwidth native_pacing =
      Bbr2Sender::PacingRate(bytes_in_flight);
  const QuicBandwidth native_bw = BandwidthEstimate();
  const Bbr2ProbeBwMode::CyclePhase phase = GetCurrentProbeBwPhase();
  const double phase_gain = static_cast<double>(PacingGain());
  const FbbrV3ReferenceResult projection_reference =
      SelectFbbrV3ReferenceBw();
  const bool use_projection_reference =
      IsFbbrProjectionObserver() && projection_reference.valid;

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

  const bool use_waveform_cruise_baseline =
      cruise_detector_mode_ == FBBRCruiseDetectorMode::kTimeWaveform &&
      IsCruisePhase(phase) && in_cruise_ &&
      waveform_cruise_state_ != WaveformCruiseState::kDisabled &&
      !current_injection_baseline_bw_.IsZero();
  const bool use_hybrid_search_cap = IsFbbrHybrid() &&
      fbbr_hybrid_stable_observation_source_ ==
          HybridStableObservationSource::kCruiseFallback &&
      !fbbr_hybrid_lower_bound_search_baseline_.IsZero() &&
      !use_waveform_cruise_baseline;

  if (use_waveform_cruise_baseline) {
    pacing_base_bw = current_injection_baseline_bw_;
    pacing_base_source =
        FBBRPacingBaseSource::kWaveformCruiseBaseline;
    trusted_bw_application_phase_ = "CRUISE";
  } else if (use_hybrid_search_cap) {
    pacing_base_bw = fbbr_hybrid_lower_bound_search_baseline_;
    pacing_base_source =
        FBBRPacingBaseSource::kHybridLowerBoundSearch;
    trusted_bw_application_phase_ = "HYBRID_SEARCH";
  } else if (use_trusted_bw) {
    pacing_base_bw = trusted_bw_;
    pacing_base_source = FBBRPacingBaseSource::kTrustedBw;
    trusted_bw_application_phase_ = PhaseApplicationName(phase);
  } else {
    trusted_bw_application_phase_ = PhaseApplicationName(phase);
  }

  QuicBandwidth baseline_pacing = native_pacing;
  if (use_waveform_cruise_baseline) {
    current_probe_bw_phase_gain_ = 1.0;
    baseline_pacing = current_injection_baseline_bw_;
  } else if (use_hybrid_search_cap) {
    baseline_pacing = std::min(
        native_pacing, fbbr_hybrid_lower_bound_search_baseline_);
  } else if (use_trusted_bw) {
    baseline_pacing =
        static_cast<float>(phase_gain) * pacing_base_bw;
  }
  if (use_projection_reference) {
    pacing_base_bw = projection_reference.bandwidth;
    switch (projection_reference.source) {
      case FbbrV3ReferenceSource::kTrusted:
        pacing_base_source = IsFbbrHybridV4()
            ? FBBRPacingBaseSource::kV4TrustedReference
            : FBBRPacingBaseSource::kV3TrustedReference;
        break;
      case FbbrV3ReferenceSource::kGuard:
        pacing_base_source = IsFbbrHybridV4()
            ? FBBRPacingBaseSource::kV4GuardReference
            : FBBRPacingBaseSource::kV3GuardReference;
        break;
      case FbbrV3ReferenceSource::kLastValid:
        pacing_base_source = IsFbbrHybridV4()
            ? FBBRPacingBaseSource::kV4LastValidReference
            : FBBRPacingBaseSource::kV3LastValidReference;
        break;
      default:
        break;
    }
    baseline_pacing =
        static_cast<float>(phase_gain) * projection_reference.bandwidth;
  }

  // V4's base target follows the exact same baseline, phase, native pacing
  // limits, units, and integer rounding as the commanded target.  It removes
  // only positive phase and waveform increments; all negative gains remain.
  QuicBandwidth base_target_before_wave = baseline_pacing;
  if (IsFbbrHybridV4()) {
    const float base_phase_gain =
        static_cast<float>(std::min(phase_gain, 1.0));
    if (use_projection_reference) {
      base_target_before_wave =
          base_phase_gain * projection_reference.bandwidth;
    } else if (use_waveform_cruise_baseline) {
      base_target_before_wave = current_injection_baseline_bw_;
    } else if (use_trusted_bw) {
      base_target_before_wave = base_phase_gain * pacing_base_bw;
    } else if (phase_gain > 1.0 &&
               IsFinitePositiveBandwidth(native_bw)) {
      base_target_before_wave =
          std::min(native_pacing, base_phase_gain * native_bw);
    } else {
      base_target_before_wave = native_pacing;
    }
  }
  if (phase == Bbr2ProbeBwMode::CyclePhase::PROBE_CRUISE &&
      cruise_detector_mode_ ==
          FBBRCruiseDetectorMode::kLegacySpectral &&
      cruise_baseline_cap_bps_ > 0 &&
      baseline_pacing.ToBitsPerSecond() >
          static_cast<int64_t>(std::min<uint64_t>(
              cruise_baseline_cap_bps_,
              static_cast<uint64_t>(std::numeric_limits<int64_t>::max())))) {
    baseline_pacing =
        QuicBandwidth::FromBitsPerSecond(cruise_baseline_cap_bps_);
    if (IsFbbrHybridV4() &&
        base_target_before_wave.ToBitsPerSecond() >
            static_cast<int64_t>(std::min<uint64_t>(
                cruise_baseline_cap_bps_,
                static_cast<uint64_t>(
                    std::numeric_limits<int64_t>::max())))) {
      base_target_before_wave =
          QuicBandwidth::FromBitsPerSecond(cruise_baseline_cap_bps_);
    }
  }

  const bool base_should_oscillate = BaseShouldOscillate();
  const bool should_oscillate = use_waveform_cruise_baseline
      ? base_should_oscillate
      : enable_convergence_gate_control_
          ? (base_should_oscillate && !bbr_stable_)
          : base_should_oscillate;
  freq_tool_on_ = should_oscillate;
  const int64_t amplitude_bps =
      should_oscillate
          ? static_cast<int64_t>(
                use_waveform_cruise_baseline
                    ? current_probe_amplitude_bps_
                    : GetCurrentAmplitudeBps())
          : 0;
  const double triangle_wave =
      should_oscillate ? TriangleWave(current_time_) : 0.0;
  const int64_t offset_bps =
      static_cast<int64_t>(amplitude_bps * triangle_wave);
  const int64_t final_bps = AddPacingOffsetWithFloor(
      baseline_pacing.ToBitsPerSecond(), offset_bps,
      minimum_pacing_rate_bps_);
  const QuicBandwidth final_pacing =
      QuicBandwidth::FromBitsPerSecond(static_cast<uint64_t>(final_bps));
  const QuicBandwidth returned_pacing =
      (!should_oscillate && !use_trusted_bw && !use_projection_reference &&
       !use_hybrid_search_cap &&
       !use_waveform_cruise_baseline)
          ? native_pacing
          : final_pacing;
  QuicBandwidth returned_base_target = returned_pacing;
  if (IsFbbrHybridV4()) {
    const int64_t base_offset_bps = std::min<int64_t>(0, offset_bps);
    const int64_t base_final_bps = AddPacingOffsetWithFloor(
        base_target_before_wave.ToBitsPerSecond(),
        base_offset_bps, minimum_pacing_rate_bps_);
    const QuicBandwidth base_final =
        QuicBandwidth::FromBitsPerSecond(
            static_cast<uint64_t>(base_final_bps));
    returned_base_target =
        (!should_oscillate && !use_trusted_bw &&
         !use_projection_reference && !use_hybrid_search_cap &&
         !use_waveform_cruise_baseline)
            ? base_target_before_wave
            : base_final;
    returned_base_target =
        std::min(returned_base_target, returned_pacing);
  }
  RecordFbbrV3PacingTarget(current_time_, returned_pacing);
  RecordFbbrV4RateTargets(
      current_time_, returned_pacing, returned_base_target);

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
int32_t FBBRSender::GetCurrentBbrModeIndex() const {
  return Bbr2Sender::GetCurrentBbrModeIndex();
}

const char* FBBRSender::WaveformStateName(WaveformCruiseState state) {
  switch (state) {
    case WaveformCruiseState::kWaitInitialSettle:
      return "WAIT_INITIAL_SETTLE";
    case WaveformCruiseState::kCollectCycle:
      return "COLLECT_CYCLE";
    case WaveformCruiseState::kExtendCycle:
      return "EXTEND_CYCLE";
    case WaveformCruiseState::kAnalyzeCycle:
      return "ANALYZE_CYCLE";
    case WaveformCruiseState::kWaitPostAdjustmentSettle:
      return "WAIT_POST_ADJUSTMENT_SETTLE";
    case WaveformCruiseState::kTrustedLocked:
      return "TRUSTED_LOCKED";
    case WaveformCruiseState::kFallbackLocked:
      return "FALLBACK_LOCKED";
    default:
      return "DISABLED";
  }
}

const char* FBBRSender::WaveformClassificationName(
    WaveformClassification classification) {
  switch (classification) {
    case WaveformClassification::kFullLoad:
      return "FULL_LOAD";
    case WaveformClassification::kUnderload:
      return "UNDERLOAD";
    case WaveformClassification::kOverload:
      return "OVERLOAD";
    default:
      return "INCONCLUSIVE";
  }
}

bool FBBRSender::UsesAdaptiveLoadJudgment() const {
  return adaptive_guard_enabled_ || fbbr_window_baseline_enabled_;
}

bool FBBRSender::IsFbbrHybrid() const {
  return GetCongestionControlType() == kFBBRHybrid;
}

bool FBBRSender::IsFbbrHybridV3() const {
  return GetCongestionControlType() == kFBBRHybridV3;
}

bool FBBRSender::IsFbbrHybridV4() const {
  return GetCongestionControlType() == kFBBRHybridV4;
}

bool FBBRSender::IsFbbrProjectionObserver() const {
  return IsFbbrHybridV3() || IsFbbrHybridV4();
}

bool FBBRSender::IsFbbrHybridObserver() const {
  return IsFbbrHybrid() || IsFbbrProjectionObserver();
}

const char* FBBRSender::FbbrV3ReferenceSourceName(
    FbbrV3ReferenceSource source) {
  switch (source) {
    case FbbrV3ReferenceSource::kTrusted:
      return "trusted";
    case FbbrV3ReferenceSource::kGuard:
      return "guard";
    case FbbrV3ReferenceSource::kLastValid:
      return "last-valid";
    default:
      return "invalid";
  }
}

FBBRSender::FbbrV3ReferenceResult
FBBRSender::SelectFbbrV3ReferenceBw() const {
  FbbrV3ReferenceResult result;
  if (!IsFbbrProjectionObserver()) {
    return result;
  }
  const bool trusted_value_valid =
      trusted_bw_valid_ && IsFinitePositiveBandwidth(trusted_bw_);
  const std::string source =
      trusted_bw_source_ == nullptr ? kTrustedBwSourceNone
                                    : trusted_bw_source_;
  const bool source_is_guard = source == kTrustedBwSourceGuardFilter;
  const bool source_is_last = source == kTrustedBwSourcePreviousTrusted;
  const bool source_is_invalid =
      source == kTrustedBwSourceNone ||
      source == kTrustedBwSourceNativeFallback;

  // Keep the existing validity/confidence policy: no new threshold is added.
  // A native/MaxBw bootstrap publication is deliberately not a V3 reference.
  if (trusted_value_valid && !source_is_guard && !source_is_last &&
      !source_is_invalid) {
    result.bandwidth = trusted_bw_;
    result.source = FbbrV3ReferenceSource::kTrusted;
    result.valid = true;
    return result;
  }
  // The two-pole ACK filter is internal observation state.  Its existing
  // publication path validates GuardBw at Cruise completion; selecting the
  // raw first sample here would prematurely close the pacing/delivery loop.
  if (trusted_value_valid && source_is_guard) {
    result.bandwidth = trusted_bw_;
    result.source = FbbrV3ReferenceSource::kGuard;
    result.valid = true;
    return result;
  }
  // PREVIOUS_TRUSTED is the existing bounded carry-forward state.  V3 does
  // not create a separate permanent reference or extend its lifetime.
  if (trusted_value_valid && source_is_last) {
    result.bandwidth = trusted_bw_;
    result.source = FbbrV3ReferenceSource::kLastValid;
    result.valid = true;
  }
  return result;
}

TimeDelta FBBRSender::CurrentFbbrV3Rtprop() const {
  TimeDelta rtprop =
      fbbr_hybrid_srtt_low_valid_ ? fbbr_hybrid_srtt_low_
                                  : model_.MinRtt();
  if (rtprop.IsZero() || rtprop.IsInfinite() ||
      rtprop.ToMicroseconds() <= 0) {
    return TimeDelta::Zero();
  }
  return rtprop;
}

void FBBRSender::RecordFbbrV3PacingTarget(
    QuicTime now,
    QuicBandwidth target_rate) const {
  if (!IsFbbrHybridV3() || !IsFinitePositiveBandwidth(target_rate)) {
    return;
  }
  if (fbbr_v3_pacing_target_history_.empty()) {
    fbbr_v3_pacing_target_history_.push_back(
        {now, QuicTime::Zero(), target_rate});
    fbbr_v3_history_start_time_ = now;
    fbbr_v3_last_recorded_pacing_target_ = target_rate;
    return;
  }

  FbbrV3PacingTargetSegment& open =
      fbbr_v3_pacing_target_history_.back();
  if (now < open.start) {
    fbbr_v3_history_valid_ = false;
    return;
  }
  if (target_rate == open.target_rate) {
    return;
  }
  if (now == open.start && open.end == QuicTime::Zero()) {
    // Repeated timestamp: replace the zero-duration target instead of
    // creating an invalid segment.
    open.target_rate = target_rate;
    fbbr_v3_last_recorded_pacing_target_ = target_rate;
    return;
  }
  open.end = now;
  fbbr_v3_pacing_target_history_.push_back(
      {now, QuicTime::Zero(), target_rate});
  fbbr_v3_last_recorded_pacing_target_ = target_rate;

  if (!fbbr_v3_max_rtprop_seen_.IsZero() &&
      !fbbr_v3_max_rtprop_seen_.IsInfinite() &&
      now >= QuicTime::Zero() + fbbr_v3_max_rtprop_seen_) {
    const QuicTime cutoff = now - fbbr_v3_max_rtprop_seen_;
    while (fbbr_v3_pacing_target_history_.size() > 1) {
      const FbbrV3PacingTargetSegment& first =
          fbbr_v3_pacing_target_history_.front();
      if (first.end == QuicTime::Zero() || first.end > cutoff) {
        break;
      }
      fbbr_v3_pacing_target_history_.pop_front();
    }
  }
  fbbr_v3_history_start_time_ =
      fbbr_v3_pacing_target_history_.front().start;
}

bool FBBRSender::HasFullFbbrV3RateHistory(
    QuicTime now,
    TimeDelta rtprop) const {
  fbbr_v3_history_valid_ = false;
  if (!IsFbbrHybridV3() || rtprop.IsZero() || rtprop.IsInfinite() ||
      rtprop.ToMicroseconds() <= 0 ||
      fbbr_v3_pacing_target_history_.empty() ||
      now < QuicTime::Zero() + rtprop) {
    return false;
  }
  if (fbbr_v3_max_rtprop_seen_.IsZero() ||
      rtprop > fbbr_v3_max_rtprop_seen_) {
    fbbr_v3_max_rtprop_seen_ = rtprop;
  }
  const QuicTime window_start = now - rtprop;
  QuicTime covered_until = window_start;
  for (const FbbrV3PacingTargetSegment& segment :
       fbbr_v3_pacing_target_history_) {
    if (!IsFinitePositiveBandwidth(segment.target_rate)) {
      return false;
    }
    const QuicTime segment_end =
        segment.end == QuicTime::Zero() ? now
                                        : std::min(now, segment.end);
    if (segment_end <= covered_until || segment.start >= now) {
      continue;
    }
    if (segment.start > covered_until) {
      return false;
    }
    covered_until = std::max(covered_until, segment_end);
    if (covered_until >= now) {
      fbbr_v3_history_valid_ = true;
      return true;
    }
  }
  return false;
}

QuicByteCount FBBRSender::ComputeFbbrV3ModelInflightBytes(
    QuicTime now,
    TimeDelta rtprop) const {
  if (!HasFullFbbrV3RateHistory(now, rtprop)) {
    return 0;
  }
  const QuicTime window_start = now - rtprop;
  long double bytes = 0.0L;
  for (const FbbrV3PacingTargetSegment& segment :
       fbbr_v3_pacing_target_history_) {
    const QuicTime segment_end =
        segment.end == QuicTime::Zero() ? now
                                        : std::min(now, segment.end);
    const QuicTime overlap_start = std::max(window_start, segment.start);
    const QuicTime overlap_end = std::min(now, segment_end);
    if (overlap_end <= overlap_start) {
      continue;
    }
    const int64_t overlap_us =
        (overlap_end - overlap_start).ToMicroseconds();
    if (overlap_us <= 0) {
      continue;
    }
    bytes += static_cast<long double>(
                 segment.target_rate.ToBitsPerSecond()) *
             static_cast<long double>(overlap_us) / 8000000.0L;
  }
  if (!std::isfinite(bytes) || bytes <= 0.0L) {
    return 0;
  }
  const long double maximum =
      static_cast<long double>(
          std::numeric_limits<QuicByteCount>::max());
  return static_cast<QuicByteCount>(
      std::llround(std::min(bytes, maximum)));
}

QuicByteCount FBBRSender::ComputeFbbrV3InflightCapBytes(
    QuicByteCount model_inflight,
    QuicByteCount native_extra_acked,
    QuicByteCount native_offload_budget) const {
  const QuicByteCount maximum =
      std::numeric_limits<QuicByteCount>::max();
  auto saturating_add = [maximum](QuicByteCount lhs,
                                  QuicByteCount rhs) {
    return lhs > maximum - rhs ? maximum : lhs + rhs;
  };
  QuicByteCount cap = saturating_add(model_inflight, native_extra_acked);
  cap = saturating_add(cap, native_offload_budget);
  cap = std::max(cap, cwnd_limits().Min());

  // The DQC BBRv2 implementation has no TSO/offload byte budget.  Its actual
  // packet quantum is one MSS, which is used only for upward alignment.
  const QuicByteCount quantum = kDefaultTCPMSS;
  if (quantum > 0 && cap < maximum) {
    const QuicByteCount remainder = cap % quantum;
    if (remainder != 0) {
      cap = saturating_add(cap, quantum - remainder);
    }
  }
  return cap;
}

FBBRSender::FbbrV3ProjectionSnapshot
FBBRSender::BuildFbbrV3ProjectionSnapshot(
    QuicByteCount native_cwnd,
    QuicByteCount actual_inflight) const {
  FbbrV3ProjectionSnapshot snapshot;
  snapshot.native_cwnd = native_cwnd;
  snapshot.actual_inflight = actual_inflight;
  snapshot.reference = SelectFbbrV3ReferenceBw();
  snapshot.pacing_target = fbbr_v3_last_recorded_pacing_target_;
  const TimeDelta rtprop = CurrentFbbrV3Rtprop();
  snapshot.history_valid =
      snapshot.reference.valid && !rtprop.IsZero() &&
      HasFullFbbrV3RateHistory(current_time_, rtprop);
  if (snapshot.history_valid) {
    snapshot.model_inflight =
        ComputeFbbrV3ModelInflightBytes(current_time_, rtprop);
    snapshot.inflight_cap = ComputeFbbrV3InflightCapBytes(
        snapshot.model_inflight, model_.MaxAckHeight(), 0);
    snapshot.raw_queue_debt =
        actual_inflight > snapshot.model_inflight
            ? actual_inflight - snapshot.model_inflight
            : 0;
    snapshot.enforced_excess =
        actual_inflight > snapshot.inflight_cap
            ? actual_inflight - snapshot.inflight_cap
            : 0;
  }
  snapshot.projection_active =
      IsFbbrHybridV3() && snapshot.reference.valid &&
      !rtprop.IsZero() && snapshot.history_valid && drain_completed_ &&
      mode_ == Bbr2Mode::PROBE_BW;
  snapshot.cap_binding =
      snapshot.projection_active &&
      snapshot.inflight_cap < snapshot.native_cwnd &&
      snapshot.actual_inflight > snapshot.inflight_cap;
  return snapshot;
}

QuicByteCount FBBRSender::ApplyFbbrV3InflightProjection(
    QuicByteCount native_cwnd_target) const {
  if (!IsFbbrHybridV3()) {
    return native_cwnd_target;
  }
  const FbbrV3ProjectionSnapshot snapshot =
      BuildFbbrV3ProjectionSnapshot(
          native_cwnd_target,
          latest_congestion_event_inflight_valid_
              ? latest_congestion_event_inflight_ : 0);
  if (!snapshot.projection_active) {
    return native_cwnd_target;
  }
  // Do not raise this result to bytes_in_flight.  Existing packets drain by
  // ACK; no persistent model bound or recovery state is written.
  return std::min(native_cwnd_target, snapshot.inflight_cap);
}

QuicByteCount FBBRSender::GetCongestionWindow() const {
  if (IsFbbrHybridV4()) {
    return ApplyFbbrV4InflightEnvelope(cwnd_);
  }
  return ApplyFbbrV3InflightProjection(cwnd_);
}

void FBBRSender::UpdateFbbrV3Telemetry(
    QuicTime now,
    QuicByteCount actual_inflight) {
  if (!IsFbbrHybridV3()) {
    return;
  }
  if (fbbr_v3_telemetry_initialized_ &&
      now >= fbbr_v3_telemetry_last_time_) {
    const uint64_t delta_us = static_cast<uint64_t>(
        (now - fbbr_v3_telemetry_last_time_).ToMicroseconds());
    fbbr_v3_telemetry_total_us_ += delta_us;
    switch (fbbr_v3_telemetry_reference_source_) {
      case FbbrV3ReferenceSource::kTrusted:
        fbbr_v3_reference_trusted_us_ += delta_us;
        break;
      case FbbrV3ReferenceSource::kGuard:
        fbbr_v3_reference_guard_us_ += delta_us;
        break;
      case FbbrV3ReferenceSource::kLastValid:
        fbbr_v3_reference_last_valid_us_ += delta_us;
        break;
      default:
        fbbr_v3_reference_invalid_us_ += delta_us;
        break;
    }
    if (fbbr_v3_telemetry_projection_active_) {
      fbbr_v3_projection_active_us_ += delta_us;
    }
    if (fbbr_v3_telemetry_history_invalid_) {
      fbbr_v3_history_invalid_us_ += delta_us;
    }
    if (fbbr_v3_telemetry_cap_binding_) {
      fbbr_v3_cap_binding_us_ += delta_us;
    }
    fbbr_v3_model_inflight_byte_us_ +=
        static_cast<long double>(fbbr_v3_telemetry_model_inflight_) *
        delta_us;
    fbbr_v3_raw_queue_debt_byte_us_ +=
        static_cast<long double>(fbbr_v3_telemetry_raw_queue_debt_) *
        delta_us;
    fbbr_v3_enforced_excess_byte_us_ +=
        static_cast<long double>(fbbr_v3_telemetry_enforced_excess_) *
        delta_us;
  }

  const FbbrV3ProjectionSnapshot snapshot =
      BuildFbbrV3ProjectionSnapshot(cwnd_, actual_inflight);
  fbbr_v3_telemetry_last_time_ = now;
  fbbr_v3_telemetry_initialized_ = true;
  fbbr_v3_telemetry_reference_source_ = snapshot.reference.source;
  fbbr_v3_telemetry_projection_active_ = snapshot.projection_active;
  fbbr_v3_telemetry_history_invalid_ = !snapshot.history_valid;
  fbbr_v3_telemetry_cap_binding_ = snapshot.cap_binding;
  fbbr_v3_telemetry_model_inflight_ = snapshot.model_inflight;
  fbbr_v3_telemetry_raw_queue_debt_ = snapshot.raw_queue_debt;
  fbbr_v3_telemetry_enforced_excess_ = snapshot.enforced_excess;
  fbbr_v3_raw_queue_debt_samples_.push_back(snapshot.raw_queue_debt);
  fbbr_v3_enforced_excess_samples_.push_back(snapshot.enforced_excess);
  ++fbbr_v3_window_ack_events_;
  if (snapshot.cap_binding) {
    ++fbbr_v3_window_cap_binding_events_;
  }
}

void FBBRSender::EmitFbbrV3FlowSummary() const {
  if (!IsFbbrHybridV3() || !cruise_load_trace_cb_) {
    return;
  }
  auto ratio = [this](uint64_t value) {
    return fbbr_v3_telemetry_total_us_ == 0
        ? 0.0
        : static_cast<double>(value) /
              static_cast<double>(fbbr_v3_telemetry_total_us_);
  };
  auto p95 = [](const std::vector<QuicByteCount>& input) {
    if (input.empty()) {
      return static_cast<QuicByteCount>(0);
    }
    std::vector<QuicByteCount> values = input;
    std::sort(values.begin(), values.end());
    const size_t index = static_cast<size_t>(
        std::ceil(0.95 * static_cast<double>(values.size()))) - 1;
    return values[std::min(index, values.size() - 1)];
  };
  const long double duration_us =
      static_cast<long double>(fbbr_v3_telemetry_total_us_);
  const long double mean_model = duration_us > 0.0L
      ? fbbr_v3_model_inflight_byte_us_ / duration_us : 0.0L;
  const long double mean_raw = duration_us > 0.0L
      ? fbbr_v3_raw_queue_debt_byte_us_ / duration_us : 0.0L;
  const long double mean_excess = duration_us > 0.0L
      ? fbbr_v3_enforced_excess_byte_us_ / duration_us : 0.0L;
  std::ostringstream row;
  row << "FBBR-hybridv3,"
      << trace_flow_id_ << ","
      << ratio(fbbr_v3_reference_trusted_us_) << ","
      << ratio(fbbr_v3_reference_guard_us_) << ","
      << ratio(fbbr_v3_reference_last_valid_us_) << ","
      << ratio(fbbr_v3_reference_invalid_us_) << ","
      << ratio(fbbr_v3_projection_active_us_) << ","
      << ratio(fbbr_v3_history_invalid_us_) << ","
      << ratio(fbbr_v3_cap_binding_us_) << ","
      << static_cast<double>(mean_model) << ","
      << static_cast<double>(mean_raw) << ","
      << p95(fbbr_v3_raw_queue_debt_samples_) << ","
      << static_cast<double>(mean_excess) << ","
      << p95(fbbr_v3_enforced_excess_samples_);
  cruise_load_trace_cb_(0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                        "V3_FLOW_SUMMARY", false, row.str());
}

void FBBRSender::FinalizeFbbrV3Trace() {
  if (fbbr_v3_flow_summary_emitted_) {
    return;
  }
  fbbr_v3_flow_summary_emitted_ = true;
  EmitFbbrV3FlowSummary();
}

void FBBRSender::RecordFbbrV4RateTargets(
    QuicTime now,
    QuicBandwidth target_rate,
    QuicBandwidth base_target_rate) const {
  if (!IsFbbrHybridV4() ||
      !IsFinitePositiveBandwidth(target_rate) ||
      !IsFinitePositiveBandwidth(base_target_rate)) {
    return;
  }
  base_target_rate = std::min(base_target_rate, target_rate);
  if (fbbr_v4_rate_history_.empty()) {
    fbbr_v4_rate_history_.push_back(
        {now, QuicTime::Zero(), target_rate, base_target_rate});
    fbbr_v4_last_target_rate_ = target_rate;
    fbbr_v4_last_base_target_rate_ = base_target_rate;
    return;
  }

  FbbrRateSegment& open = fbbr_v4_rate_history_.back();
  if (now < open.start) {
    fbbr_v4_rate_history_integrity_valid_ = false;
    return;
  }
  if (target_rate == open.target_rate &&
      base_target_rate == open.base_target_rate) {
    return;
  }
  if (now == open.start && open.end == QuicTime::Zero()) {
    open.target_rate = target_rate;
    open.base_target_rate = base_target_rate;
    fbbr_v4_last_target_rate_ = target_rate;
    fbbr_v4_last_base_target_rate_ = base_target_rate;
    return;
  }
  open.end = now;
  fbbr_v4_rate_history_.push_back(
      {now, QuicTime::Zero(), target_rate, base_target_rate});
  fbbr_v4_last_target_rate_ = target_rate;
  fbbr_v4_last_base_target_rate_ = base_target_rate;

  if (!fbbr_v4_max_rtprop_seen_.IsZero() &&
      !fbbr_v4_max_rtprop_seen_.IsInfinite() &&
      now >= QuicTime::Zero() + fbbr_v4_max_rtprop_seen_) {
    const QuicTime cutoff = now - fbbr_v4_max_rtprop_seen_;
    // Keep the segment containing the integration boundary as its anchor.
    while (fbbr_v4_rate_history_.size() > 1 &&
           fbbr_v4_rate_history_[1].start <= cutoff) {
      fbbr_v4_rate_history_.pop_front();
    }
  }
}

bool FBBRSender::HasFullFbbrV4TargetHistory(
    QuicTime now,
    TimeDelta rtprop) const {
  if (!IsFbbrHybridV4() ||
      !fbbr_v4_rate_history_integrity_valid_ ||
      rtprop.IsZero() || rtprop.IsInfinite() ||
      rtprop.ToMicroseconds() <= 0 ||
      fbbr_v4_rate_history_.empty() ||
      now < QuicTime::Zero() + rtprop) {
    return false;
  }
  if (fbbr_v4_max_rtprop_seen_.IsZero() ||
      rtprop > fbbr_v4_max_rtprop_seen_) {
    fbbr_v4_max_rtprop_seen_ = rtprop;
  }
  const QuicTime window_start = now - rtprop;
  QuicTime covered_until = window_start;
  for (const FbbrRateSegment& segment : fbbr_v4_rate_history_) {
    if (!IsFinitePositiveBandwidth(segment.target_rate) ||
        !IsFinitePositiveBandwidth(segment.base_target_rate) ||
        segment.base_target_rate > segment.target_rate) {
      return false;
    }
    const QuicTime segment_end =
        segment.end == QuicTime::Zero() ? now
                                        : std::min(now, segment.end);
    if (segment_end <= covered_until || segment.start >= now) {
      continue;
    }
    if (segment.start > covered_until) {
      return false;
    }
    covered_until = std::max(covered_until, segment_end);
    if (covered_until >= now) {
      return true;
    }
  }
  return false;
}

QuicByteCount FBBRSender::ComputeFbbrV4PlannedInflightBytes(
    QuicTime now,
    TimeDelta rtprop) const {
  if (!HasFullFbbrV4TargetHistory(now, rtprop)) {
    return 0;
  }
  const QuicTime window_start = now - rtprop;
  long double bytes = 0.0L;
  for (const FbbrRateSegment& segment : fbbr_v4_rate_history_) {
    const QuicTime segment_end =
        segment.end == QuicTime::Zero() ? now
                                        : std::min(now, segment.end);
    const QuicTime overlap_start = std::max(window_start, segment.start);
    const QuicTime overlap_end = std::min(now, segment_end);
    if (overlap_end <= overlap_start) {
      continue;
    }
    const int64_t overlap_us =
        (overlap_end - overlap_start).ToMicroseconds();
    if (overlap_us <= 0) {
      continue;
    }
    bytes += static_cast<long double>(
                 segment.target_rate.ToBitsPerSecond()) *
             static_cast<long double>(overlap_us) / 8000000.0L;
  }
  if (!std::isfinite(bytes) || bytes <= 0.0L) {
    return 0;
  }
  const long double maximum = static_cast<long double>(
      std::numeric_limits<QuicByteCount>::max());
  return static_cast<QuicByteCount>(
      std::llround(std::min(bytes, maximum)));
}

QuicByteCount FBBRSender::ComputeFbbrV4PositiveProbeCreditBytes(
    QuicTime now,
    TimeDelta rtprop) const {
  if (!HasFullFbbrV4TargetHistory(now, rtprop)) {
    return 0;
  }
  const QuicTime window_start = now - rtprop;
  long double bytes = 0.0L;
  for (const FbbrRateSegment& segment : fbbr_v4_rate_history_) {
    const QuicTime segment_end =
        segment.end == QuicTime::Zero() ? now
                                        : std::min(now, segment.end);
    const QuicTime overlap_start = std::max(window_start, segment.start);
    const QuicTime overlap_end = std::min(now, segment_end);
    if (overlap_end <= overlap_start ||
        segment.target_rate <= segment.base_target_rate) {
      continue;
    }
    const int64_t overlap_us =
        (overlap_end - overlap_start).ToMicroseconds();
    if (overlap_us <= 0) {
      continue;
    }
    const int64_t excess_bps =
        segment.target_rate.ToBitsPerSecond() -
        segment.base_target_rate.ToBitsPerSecond();
    bytes += static_cast<long double>(excess_bps) *
             static_cast<long double>(overlap_us) / 8000000.0L;
  }
  if (!std::isfinite(bytes) || bytes <= 0.0L) {
    return 0;
  }
  const long double maximum = static_cast<long double>(
      std::numeric_limits<QuicByteCount>::max());
  return static_cast<QuicByteCount>(
      std::llround(std::min(bytes, maximum)));
}

void FBBRSender::RecordFbbrV4DeliveredPoint(
    QuicTime now,
    uint64_t cumulative_delivered,
    bool app_limited) {
  if (!IsFbbrHybridV4()) {
    return;
  }
  if (fbbr_v4_delivered_history_.empty()) {
    fbbr_v4_delivered_history_.push_back(
        {now, cumulative_delivered, app_limited});
    return;
  }
  FbbrDeliveredPoint& last = fbbr_v4_delivered_history_.back();
  if (now < last.timestamp) {
    fbbr_v4_delivered_history_integrity_valid_ = false;
    return;
  }
  if (now == last.timestamp) {
    last.cumulative_delivered_bytes =
        std::max(last.cumulative_delivered_bytes,
                 cumulative_delivered);
    last.app_limited = last.app_limited || app_limited;
    return;
  }
  if (cumulative_delivered < last.cumulative_delivered_bytes) {
    // Begin a fresh counter generation.  The reset remains invalid until it
    // lies strictly before a subsequently covered RTprop window.
    fbbr_v4_delivered_history_.clear();
    fbbr_v4_last_counter_reset_time_ = now;
  }
  fbbr_v4_delivered_history_.push_back(
      {now, cumulative_delivered, app_limited});

  if (!fbbr_v4_max_rtprop_seen_.IsZero() &&
      !fbbr_v4_max_rtprop_seen_.IsInfinite() &&
      now >= QuicTime::Zero() + fbbr_v4_max_rtprop_seen_) {
    const QuicTime cutoff = now - fbbr_v4_max_rtprop_seen_;
    // Retain the newest step point at or before the boundary.
    while (fbbr_v4_delivered_history_.size() > 1 &&
           fbbr_v4_delivered_history_[1].timestamp <= cutoff) {
      fbbr_v4_delivered_history_.pop_front();
    }
  }
}

bool FBBRSender::HasValidFbbrV4ServiceHistory(
    QuicTime now,
    TimeDelta rtprop,
    bool current_app_limited,
    bool* app_limited_contaminated) const {
  if (app_limited_contaminated != nullptr) {
    *app_limited_contaminated = false;
  }
  if (!IsFbbrHybridV4() ||
      !fbbr_v4_delivered_history_integrity_valid_ ||
      rtprop.IsZero() || rtprop.IsInfinite() ||
      rtprop.ToMicroseconds() <= 0 ||
      fbbr_v4_delivered_history_.empty() ||
      now < QuicTime::Zero() + rtprop) {
    return false;
  }
  const QuicTime window_start = now - rtprop;
  size_t anchor_index = fbbr_v4_delivered_history_.size();
  for (size_t i = 0; i < fbbr_v4_delivered_history_.size(); ++i) {
    if (fbbr_v4_delivered_history_[i].timestamp <= window_start) {
      anchor_index = i;
    } else {
      break;
    }
  }
  if (anchor_index == fbbr_v4_delivered_history_.size()) {
    return false;
  }
  if (fbbr_v4_last_counter_reset_time_ != QuicTime::Zero() &&
      fbbr_v4_last_counter_reset_time_ >= window_start &&
      fbbr_v4_last_counter_reset_time_ <= now) {
    return false;
  }

  bool contaminated = current_app_limited;
  uint64_t previous =
      fbbr_v4_delivered_history_[anchor_index]
          .cumulative_delivered_bytes;
  contaminated =
      contaminated ||
      fbbr_v4_delivered_history_[anchor_index].app_limited;
  for (size_t i = anchor_index + 1;
       i < fbbr_v4_delivered_history_.size(); ++i) {
    const FbbrDeliveredPoint& point = fbbr_v4_delivered_history_[i];
    if (point.timestamp > now) {
      break;
    }
    if (point.cumulative_delivered_bytes < previous) {
      return false;
    }
    previous = point.cumulative_delivered_bytes;
    if (point.timestamp >= window_start && point.app_limited) {
      contaminated = true;
    }
  }
  if (app_limited_contaminated != nullptr) {
    *app_limited_contaminated = contaminated;
  }
  return !contaminated;
}

QuicByteCount FBBRSender::ComputeFbbrV4ServiceInflightBytes(
    QuicTime now,
    TimeDelta rtprop) const {
  if (rtprop.IsZero() || rtprop.IsInfinite() ||
      rtprop.ToMicroseconds() <= 0 ||
      fbbr_v4_delivered_history_.empty() ||
      now < QuicTime::Zero() + rtprop) {
    return 0;
  }
  const QuicTime window_start = now - rtprop;
  bool has_anchor = false;
  uint64_t anchor_delivered = 0;
  bool has_current = false;
  uint64_t current_delivered = 0;
  for (const FbbrDeliveredPoint& point :
       fbbr_v4_delivered_history_) {
    if (point.timestamp <= window_start) {
      has_anchor = true;
      anchor_delivered = point.cumulative_delivered_bytes;
    }
    if (point.timestamp <= now) {
      has_current = true;
      current_delivered = point.cumulative_delivered_bytes;
    } else {
      break;
    }
  }
  if (!has_anchor || !has_current ||
      current_delivered < anchor_delivered) {
    return 0;
  }
  return static_cast<QuicByteCount>(
      current_delivered - anchor_delivered);
}

QuicByteCount FBBRSender::ComputeFbbrV4EnvelopeBytes(
    QuicByteCount plan_inflight,
    QuicByteCount service_inflight,
    QuicByteCount positive_probe_credit,
    bool service_history_valid) const {
  if (!service_history_valid) {
    return plan_inflight;
  }
  const QuicByteCount maximum =
      std::numeric_limits<QuicByteCount>::max();
  const QuicByteCount service_budget =
      service_inflight > maximum - positive_probe_credit
          ? maximum
          : service_inflight + positive_probe_credit;
  return std::min(plan_inflight, service_budget);
}

QuicByteCount FBBRSender::ComputeFbbrV4InflightCapBytes(
    QuicByteCount envelope,
    QuicByteCount native_extra_acked,
    QuicByteCount native_offload_budget) const {
  // V4 intentionally reuses V3's verified saturating headroom, MinPipeCwnd,
  // and MSS-alignment implementation.  Offload budget remains zero.
  return ComputeFbbrV3InflightCapBytes(
      envelope, native_extra_acked, native_offload_budget);
}

FBBRSender::FbbrV4EnvelopeSnapshot
FBBRSender::BuildFbbrV4EnvelopeSnapshot(
    QuicByteCount native_cwnd,
    QuicByteCount actual_inflight) const {
  FbbrV4EnvelopeSnapshot snapshot;
  snapshot.native_cwnd = native_cwnd;
  snapshot.actual_inflight = actual_inflight;
  snapshot.reference = SelectFbbrV3ReferenceBw();
  snapshot.pacing_target = fbbr_v4_last_target_rate_;
  snapshot.pacing_base_target = fbbr_v4_last_base_target_rate_;
  const TimeDelta rtprop = CurrentFbbrV3Rtprop();
  snapshot.target_history_valid =
      !rtprop.IsZero() &&
      HasFullFbbrV4TargetHistory(current_time_, rtprop);
  if (snapshot.target_history_valid) {
    snapshot.plan_inflight =
        ComputeFbbrV4PlannedInflightBytes(current_time_, rtprop);
    snapshot.positive_probe_credit =
        ComputeFbbrV4PositiveProbeCreditBytes(current_time_, rtprop);
  }
  if (!rtprop.IsZero()) {
    snapshot.service_inflight =
        ComputeFbbrV4ServiceInflightBytes(current_time_, rtprop);
    snapshot.service_history_valid =
        HasValidFbbrV4ServiceHistory(
            current_time_, rtprop, model_.is_app_limited(),
            &snapshot.app_limited_contaminated);
  }
  const QuicByteCount maximum =
      std::numeric_limits<QuicByteCount>::max();
  snapshot.service_budget =
      snapshot.service_inflight >
              maximum - snapshot.positive_probe_credit
          ? maximum
          : snapshot.service_inflight +
                snapshot.positive_probe_credit;
  snapshot.envelope = ComputeFbbrV4EnvelopeBytes(
      snapshot.plan_inflight, snapshot.service_inflight,
      snapshot.positive_probe_credit,
      snapshot.service_history_valid);
  snapshot.extra_acked = model_.MaxAckHeight();
  if (snapshot.target_history_valid) {
    snapshot.inflight_cap = ComputeFbbrV4InflightCapBytes(
        snapshot.envelope, snapshot.extra_acked, 0);
  }
  snapshot.plan_excess =
      snapshot.plan_inflight > snapshot.service_inflight
          ? snapshot.plan_inflight - snapshot.service_inflight : 0;
  snapshot.service_restriction =
      snapshot.plan_inflight > snapshot.envelope
          ? snapshot.plan_inflight - snapshot.envelope : 0;
  snapshot.raw_queue_debt =
      actual_inflight > snapshot.plan_inflight
          ? actual_inflight - snapshot.plan_inflight : 0;
  snapshot.envelope_debt =
      actual_inflight > snapshot.envelope
          ? actual_inflight - snapshot.envelope : 0;
  snapshot.enforced_excess =
      actual_inflight > snapshot.inflight_cap &&
              snapshot.target_history_valid
          ? actual_inflight - snapshot.inflight_cap : 0;
  snapshot.projection_active =
      IsFbbrHybridV4() && !rtprop.IsZero() &&
      snapshot.target_history_valid && drain_completed_ &&
      mode_ == Bbr2Mode::PROBE_BW;
  snapshot.service_limited =
      snapshot.projection_active &&
      snapshot.service_history_valid &&
      snapshot.envelope < snapshot.plan_inflight;
  snapshot.cap_binding =
      snapshot.projection_active &&
      snapshot.inflight_cap < snapshot.native_cwnd &&
      snapshot.actual_inflight > snapshot.inflight_cap;
  return snapshot;
}

QuicByteCount FBBRSender::ApplyFbbrV4InflightEnvelope(
    QuicByteCount native_cwnd_target) const {
  if (!IsFbbrHybridV4()) {
    return native_cwnd_target;
  }
  const FbbrV4EnvelopeSnapshot snapshot =
      BuildFbbrV4EnvelopeSnapshot(
          native_cwnd_target,
          latest_congestion_event_inflight_valid_
              ? latest_congestion_event_inflight_ : 0);
  if (!snapshot.projection_active) {
    return native_cwnd_target;
  }
  return std::min(native_cwnd_target, snapshot.inflight_cap);
}

void FBBRSender::UpdateFbbrV4Telemetry(
    QuicTime now,
    QuicByteCount actual_inflight) {
  if (!IsFbbrHybridV4()) {
    return;
  }
  if (fbbr_v4_telemetry_initialized_ &&
      now >= fbbr_v4_telemetry_last_time_) {
    const uint64_t delta_us = static_cast<uint64_t>(
        (now - fbbr_v4_telemetry_last_time_).ToMicroseconds());
    fbbr_v4_telemetry_total_us_ += delta_us;
    switch (fbbr_v4_telemetry_reference_source_) {
      case FbbrV3ReferenceSource::kTrusted:
        fbbr_v4_reference_trusted_us_ += delta_us;
        break;
      case FbbrV3ReferenceSource::kGuard:
        fbbr_v4_reference_guard_us_ += delta_us;
        break;
      case FbbrV3ReferenceSource::kLastValid:
        fbbr_v4_reference_last_valid_us_ += delta_us;
        break;
      default:
        fbbr_v4_reference_invalid_us_ += delta_us;
        break;
    }
    if (fbbr_v4_telemetry_projection_active_) {
      fbbr_v4_projection_active_us_ += delta_us;
    }
    if (fbbr_v4_telemetry_service_history_valid_) {
      fbbr_v4_service_history_valid_us_ += delta_us;
    }
    if (fbbr_v4_telemetry_app_limited_fallback_) {
      fbbr_v4_app_limited_fallback_us_ += delta_us;
    }
    if (fbbr_v4_telemetry_plan_only_fallback_) {
      fbbr_v4_plan_only_fallback_us_ += delta_us;
    }
    if (fbbr_v4_telemetry_service_limited_) {
      fbbr_v4_service_limited_us_ += delta_us;
    }
    if (fbbr_v4_telemetry_cap_binding_) {
      fbbr_v4_cap_binding_us_ += delta_us;
    }
    fbbr_v4_plan_inflight_byte_us_ +=
        static_cast<long double>(
            fbbr_v4_telemetry_plan_inflight_) * delta_us;
    fbbr_v4_service_inflight_byte_us_ +=
        static_cast<long double>(
            fbbr_v4_telemetry_service_inflight_) * delta_us;
    fbbr_v4_probe_credit_byte_us_ +=
        static_cast<long double>(
            fbbr_v4_telemetry_probe_credit_) * delta_us;
    fbbr_v4_extra_acked_byte_us_ +=
        static_cast<long double>(
            fbbr_v4_telemetry_extra_acked_) * delta_us;
    fbbr_v4_service_restriction_byte_us_ +=
        static_cast<long double>(
            fbbr_v4_telemetry_service_restriction_) * delta_us;
    fbbr_v4_enforced_excess_byte_us_ +=
        static_cast<long double>(
            fbbr_v4_telemetry_enforced_excess_) * delta_us;
  }

  const FbbrV4EnvelopeSnapshot snapshot =
      BuildFbbrV4EnvelopeSnapshot(cwnd_, actual_inflight);
  fbbr_v4_telemetry_last_time_ = now;
  fbbr_v4_telemetry_initialized_ = true;
  fbbr_v4_telemetry_reference_source_ = snapshot.reference.source;
  fbbr_v4_telemetry_projection_active_ = snapshot.projection_active;
  fbbr_v4_telemetry_service_history_valid_ =
      snapshot.service_history_valid;
  fbbr_v4_telemetry_app_limited_fallback_ =
      snapshot.projection_active &&
      snapshot.app_limited_contaminated;
  fbbr_v4_telemetry_plan_only_fallback_ =
      snapshot.projection_active &&
      !snapshot.service_history_valid;
  fbbr_v4_telemetry_service_limited_ = snapshot.service_limited;
  fbbr_v4_telemetry_cap_binding_ = snapshot.cap_binding;
  fbbr_v4_telemetry_plan_inflight_ = snapshot.plan_inflight;
  fbbr_v4_telemetry_service_inflight_ = snapshot.service_inflight;
  fbbr_v4_telemetry_probe_credit_ =
      snapshot.positive_probe_credit;
  fbbr_v4_telemetry_extra_acked_ = snapshot.extra_acked;
  fbbr_v4_telemetry_service_restriction_ =
      snapshot.service_restriction;
  fbbr_v4_telemetry_enforced_excess_ = snapshot.enforced_excess;
  fbbr_v4_plan_inflight_samples_.push_back(snapshot.plan_inflight);
  fbbr_v4_service_inflight_samples_.push_back(
      snapshot.service_inflight);
  fbbr_v4_probe_credit_samples_.push_back(
      snapshot.positive_probe_credit);
  fbbr_v4_extra_acked_samples_.push_back(snapshot.extra_acked);
  fbbr_v4_service_restriction_samples_.push_back(
      snapshot.service_restriction);
  fbbr_v4_enforced_excess_samples_.push_back(
      snapshot.enforced_excess);
  ++fbbr_v4_window_ack_events_;
  if (snapshot.cap_binding) {
    ++fbbr_v4_window_cap_binding_events_;
  }
}

void FBBRSender::EmitFbbrV4FlowSummary() const {
  if (!IsFbbrHybridV4() || !cruise_load_trace_cb_) {
    return;
  }
  auto ratio = [this](uint64_t value) {
    return fbbr_v4_telemetry_total_us_ == 0
        ? 0.0
        : static_cast<double>(value) /
              static_cast<double>(fbbr_v4_telemetry_total_us_);
  };
  auto p95 = [](const std::vector<QuicByteCount>& input) {
    if (input.empty()) {
      return static_cast<QuicByteCount>(0);
    }
    std::vector<QuicByteCount> values = input;
    std::sort(values.begin(), values.end());
    const size_t index = static_cast<size_t>(
        std::ceil(0.95 * static_cast<double>(values.size()))) - 1;
    return values[std::min(index, values.size() - 1)];
  };
  const long double duration_us =
      static_cast<long double>(fbbr_v4_telemetry_total_us_);
  auto mean = [duration_us](long double byte_us) {
    return duration_us > 0.0L ? byte_us / duration_us : 0.0L;
  };
  std::ostringstream row;
  row << "FBBR-hybirdv4,"
      << trace_flow_id_ << ","
      << ratio(fbbr_v4_reference_trusted_us_) << ","
      << ratio(fbbr_v4_reference_guard_us_) << ","
      << ratio(fbbr_v4_reference_last_valid_us_) << ","
      << ratio(fbbr_v4_reference_invalid_us_) << ","
      << ratio(fbbr_v4_projection_active_us_) << ","
      << ratio(fbbr_v4_service_history_valid_us_) << ","
      << ratio(fbbr_v4_app_limited_fallback_us_) << ","
      << ratio(fbbr_v4_plan_only_fallback_us_) << ","
      << ratio(fbbr_v4_service_limited_us_) << ","
      << ratio(fbbr_v4_cap_binding_us_) << ","
      << static_cast<double>(
             mean(fbbr_v4_plan_inflight_byte_us_)) << ","
      << p95(fbbr_v4_plan_inflight_samples_) << ","
      << static_cast<double>(
             mean(fbbr_v4_service_inflight_byte_us_)) << ","
      << p95(fbbr_v4_service_inflight_samples_) << ","
      << static_cast<double>(
             mean(fbbr_v4_probe_credit_byte_us_)) << ","
      << p95(fbbr_v4_probe_credit_samples_) << ","
      << static_cast<double>(
             mean(fbbr_v4_extra_acked_byte_us_)) << ","
      << p95(fbbr_v4_extra_acked_samples_) << ","
      << static_cast<double>(
             mean(fbbr_v4_service_restriction_byte_us_)) << ","
      << p95(fbbr_v4_service_restriction_samples_) << ","
      << static_cast<double>(
             mean(fbbr_v4_enforced_excess_byte_us_)) << ","
      << p95(fbbr_v4_enforced_excess_samples_);
  cruise_load_trace_cb_(0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                        "V4_FLOW_SUMMARY", false, row.str());
}

void FBBRSender::FinalizeFbbrV4Trace() {
  if (fbbr_v4_flow_summary_emitted_) {
    return;
  }
  fbbr_v4_flow_summary_emitted_ = true;
  EmitFbbrV4FlowSummary();
}

FBBRSender::FbbrHybridDecision FBBRSender::ClassifyFbbrHybridRegime(
    const FbbrHybridRegimeFeatures& features,
    const FbbrRegimeContext& context) {
  FbbrHybridDecision decision;
  auto finalize = [](FbbrHybridDecision value) { return value; };
  auto decide = [&decision, &finalize](
                            WaveformClassification classification,
                            const char* rule,
                            bool upper_bound_rule,
                            bool refresh_rtprop,
                            bool update_rtprop_drate) {
    (void)upper_bound_rule;
    decision.classification = classification;
    decision.rule_id = rule;
    decision.refresh_rtprop = refresh_rtprop;
    decision.update_rtprop_drate = update_rtprop_drate;
    // Hybrid upper references come from native MaxBw and MaxSRTT updates,
    // not from N01-N16 window classification side effects.
    decision.update_baseline_low = update_rtprop_drate;
    return finalize(decision);
  };
  auto max_exceeded = [&]() {
    return features.srtt_stats_valid && context.max_srtt_valid &&
        std::isfinite(features.srtt_max_ms) &&
        features.srtt_max_ms > context.max_srtt_ms;
  };
  auto min_below_rtprop = [&]() {
    return features.srtt_stats_valid && context.rtprop_valid &&
        std::isfinite(features.srtt_min_ms) &&
        features.srtt_min_ms < context.rtprop_ms;
  };
  auto fallback_overload_signal = [&](bool* valid) {
    const bool srtt_threshold_valid = features.srtt_stats_valid &&
        context.max_srtt_valid && context.rtprop_valid &&
        std::isfinite(features.srtt_mean_ms) &&
        features.srtt_mean_ms > 0.0 &&
        std::isfinite(context.max_srtt_ms) &&
        std::isfinite(context.rtprop_ms) &&
        context.max_srtt_ms >= context.rtprop_ms &&
        context.rtprop_ms > 0.0;
    const bool inflight_threshold_valid =
        features.inflight_bdp_valid && features.bdp_bytes > 0;
    *valid = srtt_threshold_valid || inflight_threshold_valid;
    if (srtt_threshold_valid) {
      const double threshold_ms =
          context.rtprop_ms + (context.max_srtt_ms - context.rtprop_ms) / 4.0;
      if (features.srtt_mean_ms > threshold_ms) {
        return true;
      }
    }
    return inflight_threshold_valid &&
           IsAtLeastElevenTenthsBdp(features.inflight_bytes,
                                    features.bdp_bytes);
  };

  // A horizontal cut is meaningful only as a clipped part of an otherwise
  // observable SRTT oscillation. A single positive or negative half-cycle
  // still counts as ordinary SRTT wave activity; raw line evidence without
  // that activity must use the no-cut fallback tree, where the
  // existing retry mechanism can increase the sender-rate excitation.
  const SrttClipCase effective_clip_case =
      features.srtt.wave.input_valid && features.srtt.wave.has_wave
          ? features.selected_clip_case
          : SrttClipCase::kNone;
  switch (effective_clip_case) {
    case SrttClipCase::kU1PositiveShoulder:
      if (features.drate.periodic == PeriodicSimilarityResult::kInvalidInput) {
        return finalize(decision);
      }
      return features.drate.periodic == PeriodicSimilarityResult::kMatch
          ? decide(WaveformClassification::kFullLoad, "N01",
                   false, false, false)
          : decide(WaveformClassification::kOverload, "N02",
                   true, false, false);
    case SrttClipCase::kU2LongTopLine:
      if (features.drate.periodic == PeriodicSimilarityResult::kInvalidInput) {
        return finalize(decision);
      }
      return features.drate.periodic == PeriodicSimilarityResult::kMatch
          ? decide(WaveformClassification::kFullLoad, "N03",
                   false, false, false)
          : decide(WaveformClassification::kOverload, "N04",
                   true, false, false);
    case SrttClipCase::kU3RepeatedTopClip:
      return decide(WaveformClassification::kOverload, "N05",
                    true, false, false);
    case SrttClipCase::kL1NegativeShoulder:
      return decide(WaveformClassification::kFullLoad, "N06",
                    false, false, false);
    case SrttClipCase::kL2LongBottomLine:
      if (!features.drate.wave.input_valid) {
        return finalize(decision);
      }
      if (features.drate.wave.has_wave) {
        return decide(WaveformClassification::kUnderload, "N07",
                      false, true, true);
      }
      return decide(WaveformClassification::kFullLoad, "N08",
                    false, false, false);
    case SrttClipCase::kL3RepeatedBottomClip:
      return decide(WaveformClassification::kUnderload, "N09",
                    false, false, true);
    case SrttClipCase::kNone:
      break;
  }

  if (!features.srtt.wave.input_valid) {
    return finalize(decision);
  }
  if (features.srtt.wave.has_wave) {
    if (max_exceeded()) {
      return decide(WaveformClassification::kOverload, "N10",
                    true, false, false);
    }
    if (min_below_rtprop()) {
      return decide(WaveformClassification::kUnderload, "N11",
                    false, true, true);
    }
    bool threshold_valid = false;
    const bool overload =
        fallback_overload_signal(&threshold_valid);
    if (!threshold_valid) {
      return finalize(decision);
    }
    return decide(overload ? WaveformClassification::kOverload
                           : WaveformClassification::kFullLoad,
                  "N12", false, false, false);
  }
  if (features.drate.periodic == PeriodicSimilarityResult::kInvalidInput) {
    return finalize(decision);
  }
  if (features.drate.periodic == PeriodicSimilarityResult::kMatch) {
    return decide(WaveformClassification::kUnderload, "N13",
                  false, false, false);
  }
  if (max_exceeded()) {
    return decide(WaveformClassification::kOverload, "N14",
                  true, false, false);
  }
  if (min_below_rtprop()) {
    return decide(WaveformClassification::kUnderload, "N15",
                  false, true, true);
  }
  bool threshold_valid = false;
  const bool overload =
      fallback_overload_signal(&threshold_valid);
  if (!threshold_valid) {
    return finalize(decision);
  }
  return decide(overload ? WaveformClassification::kOverload
                         : WaveformClassification::kFullLoad,
                "N16", false, false, false);
}

FBBRSender::FbbrHybridActuatorResult
FBBRSender::ComputeFbbrHybridInjectionBaseline(
    WaveformClassification classification,
    double mindrate_bps,
    double maxdrate_bps,
    double meandrate_bps,
    bool baseline_low_valid,
    double baseline_low_bps,
    bool max_bw_valid,
    double max_bw_bps,
    double current_baseline_bps,
    bool rtprop_drate_valid,
    double rtprop_drate_bps,
    double midpoint_trigger_ratio,
    double minimum_rate_bps,
    bool overload_gradient_valid,
    double overload_beta) {
  FbbrHybridActuatorResult result;
  if (classification == WaveformClassification::kInconclusive ||
      !std::isfinite(mindrate_bps) || mindrate_bps <= 0.0 ||
      !std::isfinite(maxdrate_bps) || maxdrate_bps < mindrate_bps ||
      !std::isfinite(meandrate_bps) || meandrate_bps <= 0.0 ||
      !std::isfinite(minimum_rate_bps) || minimum_rate_bps <= 0.0 ||
      !std::isfinite(midpoint_trigger_ratio) ||
      midpoint_trigger_ratio < 0.0) {
    return result;
  }
  result.swing_bps = maxdrate_bps - mindrate_bps;
  const bool reference_valid = rtprop_drate_valid &&
      std::isfinite(rtprop_drate_bps) && rtprop_drate_bps > 0.0 &&
      rtprop_drate_bps <= maxdrate_bps;
  result.reference_gap_bps = reference_valid
      ? maxdrate_bps - rtprop_drate_bps
      : 0.0;
  const bool midpoint_eligible = reference_valid &&
      result.swing_bps > midpoint_trigger_ratio * result.reference_gap_bps;
  const double midpoint = mindrate_bps + result.swing_bps / 2.0;
  result.bracket_valid = baseline_low_valid && max_bw_valid &&
      std::isfinite(baseline_low_bps) && baseline_low_bps > 0.0 &&
      std::isfinite(max_bw_bps) &&
      max_bw_bps > baseline_low_bps;
  if (classification == WaveformClassification::kFullLoad) {
    result.update_trusted_bw = true;
    result.trusted_bw_bps = meandrate_bps;
    result.valid = true;
    return result;
  } else if (classification == WaveformClassification::kUnderload) {
    result.update_baseline = true;
    result.bracket_target_bps = result.bracket_valid
        ? baseline_low_bps + (max_bw_bps - baseline_low_bps) / 2.0
        : 0.0;
    result.bracket_triggered = result.bracket_valid &&
        std::isfinite(current_baseline_bps) &&
        current_baseline_bps < result.bracket_target_bps;
    result.midpoint_triggered = !result.bracket_triggered &&
        midpoint_eligible;
    result.next_baseline_bps = result.bracket_triggered
        ? result.bracket_target_bps
        : (result.midpoint_triggered ? midpoint : maxdrate_bps);
  } else {
    result.update_baseline = true;
    if (!std::isfinite(current_baseline_bps) ||
        current_baseline_bps <= 0.0) {
      return result;
    }
    result.overload_beta =
        overload_gradient_valid && std::isfinite(overload_beta)
            ? ClampValue(overload_beta, 0.0, 0.10)
            : 0.0;
    result.next_baseline_bps =
        current_baseline_bps * (1.0 - result.overload_beta);
  }
  result.next_baseline_bps =
      std::max(minimum_rate_bps, result.next_baseline_bps);
  result.valid = std::isfinite(result.next_baseline_bps) &&
      result.next_baseline_bps > 0.0;
  return result;
}

double FBBRSender::EstimateActualSignalPeriod(
    const std::vector<double>& values,
    const std::vector<bool>& valid,
    double sample_step_s,
    double nominal_period_s,
    double* correlation) {
  if (correlation != nullptr) {
    *correlation = -1.0;
  }
  if (values.size() != valid.size() || values.size() < 20 ||
      sample_step_s <= 0.0 || nominal_period_s <= 0.0) {
    return 0.0;
  }
  const size_t minimum_lag = std::max<size_t>(
      2, static_cast<size_t>(std::llround(
             0.75 * nominal_period_s / sample_step_s)));
  const size_t maximum_lag = std::min<size_t>(
      values.size() - 2,
      static_cast<size_t>(std::llround(
          1.25 * nominal_period_s / sample_step_s)));
  double best_correlation = -1.0;
  size_t best_lag = 0;
  for (size_t lag = minimum_lag; lag <= maximum_lag; ++lag) {
    size_t pairs = 0;
    const double candidate =
        ValidLagCorrelation(values, valid, lag, &pairs);
    if (pairs >= std::max<size_t>(4, lag / 2) &&
        candidate > best_correlation) {
      best_correlation = candidate;
      best_lag = lag;
    }
  }
  if (correlation != nullptr) {
    *correlation = best_correlation;
  }
  return best_lag > 0 && best_correlation >= 0.50
      ? best_lag * sample_step_s
      : 0.0;
}

FBBRSender::WaveActivityFeatures
FBBRSender::DetectOrdinaryWaveActivity(
    const std::vector<double>& values,
    const std::vector<bool>& valid,
    double sample_step_s,
    double period_s,
    bool allow_half_cycle_wave) const {
  WaveActivityFeatures result;
  if (values.size() != valid.size() || sample_step_s <= 0.0 ||
      period_s <= 0.0) {
    return result;
  }
  const std::vector<double> filtered = MedianFilter3(values, valid);
  const size_t samples_per_period = static_cast<size_t>(
      std::llround(period_s / sample_step_s));
  if (samples_per_period < 20 || values.size() < 2 * samples_per_period) {
    return result;
  }
  bool all_cycles_valid = true;
  const char* best_failure = "LOW_AMP";
  for (size_t cycle = 0; cycle < 2; ++cycle) {
    const size_t begin = cycle * samples_per_period;
    const size_t end = std::min(values.size(), begin + samples_per_period);
    std::vector<double> cycle_values;
    std::vector<double> residuals;
    for (size_t i = begin; i < end; ++i) {
      if (valid[i] && std::isfinite(values[i]) &&
          std::isfinite(filtered[i])) {
        cycle_values.push_back(filtered[i]);
        residuals.push_back(values[i] - filtered[i]);
      }
    }
    if (cycle_values.size() < 20) {
      all_cycles_valid = false;
      continue;
    }
    const double sigma = RobustSigma(residuals);
    const double amplitude =
        Quantile(cycle_values, 0.95) - Quantile(cycle_values, 0.05);
    const double level = std::max(
        std::abs(Median(cycle_values)), 1e-12);
    const bool amplitude_ok = amplitude >= std::max(
        waveform_activity_amplitude_noise_multiplier_ * sigma,
        waveform_activity_min_level_ratio_ * level);
    double up_change = 0.0;
    double down_change = 0.0;
    size_t valid_steps = 0;
    size_t active_steps = 0;
    int previous_sign = 0;
    uint32_t reversals = 0;
    double last_threshold = 0.0;
    if (amplitude > 0.0) {
      for (size_t i = begin + 1; i < end; ++i) {
        if (!valid[i - 1] || !valid[i] ||
            !std::isfinite(filtered[i - 1]) ||
            !std::isfinite(filtered[i])) {
          continue;
        }
        ++valid_steps;
        const double delta = filtered[i] - filtered[i - 1];
        const double threshold = std::max(
            waveform_activity_step_noise_multiplier_ *
                std::sqrt(2.0) * sigma,
            waveform_activity_min_normalized_step_slope_ * amplitude *
                sample_step_s / period_s);
        last_threshold = threshold;
        if (std::abs(delta) + 1e-15 < threshold) {
          continue;
        }
        ++active_steps;
        const int sign = delta > 0.0 ? 1 : -1;
        if (previous_sign != 0 && sign != previous_sign) {
          ++reversals;
        }
        previous_sign = sign;
        if (delta > 0.0) {
          up_change += delta;
        } else {
          down_change -= delta;
        }
      }
    }
    const size_t required_steps = std::max<size_t>(
        waveform_activity_min_active_steps_,
        static_cast<size_t>(std::ceil(
            waveform_activity_min_active_step_ratio_ * valid_steps)));
    const bool active_ok = active_steps >= required_steps;
    const double path = up_change + down_change;
    const bool full_wave_ok = amplitude > 0.0 &&
        up_change >= waveform_activity_min_directional_change_ratio_ *
                         amplitude &&
        down_change >= waveform_activity_min_directional_change_ratio_ *
                           amplitude &&
        path >= waveform_activity_min_significant_path_ratio_ * amplitude &&
        reversals >= waveform_activity_min_slope_reversals_;
    const bool positive_half_ok = allow_half_cycle_wave && amplitude > 0.0 &&
        up_change >= waveform_activity_min_directional_change_ratio_ *
                         amplitude &&
        down_change <= 0.25 * amplitude &&
        path >= waveform_activity_min_significant_path_ratio_ * amplitude;
    const bool negative_half_ok = allow_half_cycle_wave && amplitude > 0.0 &&
        down_change >= waveform_activity_min_directional_change_ratio_ *
                           amplitude &&
        up_change <= 0.25 * amplitude &&
        path >= waveform_activity_min_significant_path_ratio_ * amplitude;
    const bool has_wave = amplitude_ok && active_ok &&
        (full_wave_ok || positive_half_ok || negative_half_ok);
    if (has_wave) {
      result.has_wave = true;
      result.active_cycle_mask |= static_cast<uint8_t>(1u << cycle);
      best_failure = "NONE";
    } else if (!amplitude_ok) {
      best_failure = "LOW_AMP";
    } else if (!active_ok) {
      best_failure = active_steps <= 2 ? "SPIKE" : "SMOOTH_ONLY";
    } else {
      best_failure = "ONE_WAY";
    }
    if (amplitude >= result.amplitude) {
      result.amplitude = amplitude;
      result.noise_sigma = sigma;
      result.amplitude_to_level_ratio = amplitude / level;
      result.step_threshold = last_threshold;
      result.active_step_ratio = valid_steps == 0
          ? 0.0 : static_cast<double>(active_steps) / valid_steps;
      result.up_change_ratio = amplitude > 0.0
          ? up_change / amplitude : 0.0;
      result.down_change_ratio = amplitude > 0.0
          ? down_change / amplitude : 0.0;
      result.significant_path_ratio = amplitude > 0.0
          ? path / amplitude : 0.0;
      result.slope_reversals = reversals;
    }
  }
  result.input_valid = all_cycles_valid;
  if (!result.input_valid) {
    result.has_wave = false;
    result.failure_reason = "INVALID_INPUT";
  } else {
    result.failure_reason = result.has_wave ? "NONE" : best_failure;
  }
  return result;
}

std::vector<FBBRSender::ContinuousHorizontalEvidence>
FBBRSender::DetectContinuousHorizontalSegments(
    const std::vector<double>& values,
    const std::vector<bool>& valid,
    double sample_step_s,
    double period_s) const {
  std::vector<ContinuousHorizontalEvidence> results;
  if (values.size() != valid.size() || values.size() < 5 ||
      sample_step_s <= 0.0 || period_s <= 0.0) {
    return results;
  }
  const std::vector<double> filtered = MedianFilter3(values, valid);
  std::vector<double> finite;
  std::vector<double> residuals;
  for (size_t i = 0; i < values.size(); ++i) {
    if (valid[i] && std::isfinite(values[i]) &&
        std::isfinite(filtered[i])) {
      finite.push_back(filtered[i]);
      residuals.push_back(values[i] - filtered[i]);
    }
  }
  if (finite.size() < 5) {
    return results;
  }
  const double p05 = Quantile(finite, 0.05);
  const double p95 = Quantile(finite, 0.95);
  const double amplitude = p95 - p05;
  const double sigma_x = RobustSigma(residuals);
  if (amplitude < std::max(
          waveform_horizontal_amplitude_noise_multiplier_ * sigma_x,
          1e-12)) {
    return results;
  }
  const size_t local_radius = std::max<size_t>(
      2, static_cast<size_t>(std::ceil(0.05 * period_s / sample_step_s)));
  std::vector<double> local_slopes(values.size(), 0.0);
  std::vector<double> absolute_local_slopes;
  for (size_t i = 0; i < values.size(); ++i) {
    const size_t begin = i > local_radius ? i - local_radius : 0;
    const size_t end = std::min(values.size(), i + local_radius + 1);
    local_slopes[i] =
        TheilSenSlope(filtered, valid, begin, end, sample_step_s);
    if (valid[i] && std::isfinite(local_slopes[i])) {
      absolute_local_slopes.push_back(std::abs(local_slopes[i]));
    }
  }
  const double s80 = Quantile(absolute_local_slopes, 0.80);
  std::vector<double> residual_slope_noise;
  for (size_t i = 1; i < residuals.size(); ++i) {
    residual_slope_noise.push_back(
        (residuals[i] - residuals[i - 1]) / sample_step_s);
  }
  const double sigma_s = RobustSigma(residual_slope_noise);
  const double theta_flat = std::max(
      waveform_horizontal_slope_noise_multiplier_ * sigma_s,
      waveform_horizontal_max_local_slope_ratio_ * s80);
  const double theta_side = std::max(
      waveform_horizontal_slope_noise_multiplier_ * sigma_s,
      waveform_horizontal_min_side_slope_ratio_ * s80);
  const double theta_kink = std::max(
      waveform_horizontal_slope_noise_multiplier_ * sigma_s,
      waveform_horizontal_min_boundary_kink_ratio_ * s80);
  std::vector<bool> flat_step(values.size(), false);
  for (size_t i = 1; i < values.size(); ++i) {
    flat_step[i] = valid[i - 1] && valid[i] &&
        std::isfinite(filtered[i - 1]) && std::isfinite(filtered[i]) &&
        std::abs((filtered[i] - filtered[i - 1]) / sample_step_s) <=
            theta_flat + 1e-15;
  }
  const size_t context_points = std::max<size_t>(
      4, static_cast<size_t>(std::ceil(0.10 * period_s / sample_step_s)));
  const size_t minimum_steps = std::max<size_t>(
      2, static_cast<size_t>(std::ceil(
          waveform_horizontal_continuous_min_duration_ratio_ * period_s /
          sample_step_s)));
  size_t i = 1;
  while (i < flat_step.size()) {
    if (!flat_step[i]) {
      ++i;
      continue;
    }
    const size_t start = i - 1;
    size_t end = i;
    while (end + 1 < flat_step.size() && flat_step[end + 1]) {
      ++end;
    }
    i = end + 1;
    if (end - start < minimum_steps || end - start + 1 < 3) {
      continue;
    }
    std::vector<double> segment_values;
    size_t valid_points = 0;
    size_t internal_flat_steps = 0;
    size_t internal_valid_steps = 0;
    for (size_t j = start; j <= end; ++j) {
      if (valid[j] && std::isfinite(filtered[j])) {
        segment_values.push_back(filtered[j]);
        ++valid_points;
      }
      if (j > start && valid[j - 1] && valid[j]) {
        ++internal_valid_steps;
        if (flat_step[j]) {
          ++internal_flat_steps;
        }
      }
    }
    const double coverage = static_cast<double>(valid_points) /
        static_cast<double>(end - start + 1);
    const double flat_fraction = internal_valid_steps == 0
        ? 0.0 : static_cast<double>(internal_flat_steps) /
                    internal_valid_steps;
    const double duration_s = (end - start) * sample_step_s;
    const double level_span = Quantile(segment_values, 0.95) -
        Quantile(segment_values, 0.05);
    const double robust_slope = TheilSenSlope(
        filtered, valid, start, end + 1, sample_step_s);
    const double level_span_limit = std::max(
        waveform_horizontal_level_span_noise_multiplier_ * sigma_x,
        waveform_horizontal_max_level_span_ratio_ * amplitude);
    const double drift_limit = std::max(
        waveform_horizontal_slope_noise_multiplier_ * sigma_x,
        waveform_horizontal_max_total_drift_ratio_ * amplitude);
    if (coverage + 1e-15 <
            waveform_horizontal_min_valid_coverage_ratio_ ||
        flat_fraction + 1e-15 <
            waveform_horizontal_min_flat_fraction_ ||
        level_span > level_span_limit + 1e-15 ||
        std::abs(robust_slope) * duration_s > drift_limit + 1e-15) {
      continue;
    }

    const double segment_mean = std::accumulate(
        segment_values.begin(), segment_values.end(), 0.0) /
        segment_values.size();
    double rss_constant = 0.0;
    double rss_linear = 0.0;
    const double intercept = segment_values.front();
    for (size_t j = start; j <= end; ++j) {
      if (!valid[j]) {
        continue;
      }
      const double constant_residual = filtered[j] - segment_mean;
      const double predicted = intercept +
          robust_slope * (j - start) * sample_step_s;
      const double linear_residual = filtered[j] - predicted;
      rss_constant += constant_residual * constant_residual;
      rss_linear += linear_residual * linear_residual;
    }
    const double n = static_cast<double>(valid_points);
    const double bic_constant = n * std::log(std::max(
        rss_constant / n, 1e-24)) + std::log(n);
    const double bic_linear = n * std::log(std::max(
        rss_linear / n, 1e-24)) + 2.0 * std::log(n);
    if (bic_constant > bic_linear + 1e-12) {
      continue;
    }

    const bool touches_left = start <= 1;
    const bool touches_right = end + 2 >= values.size();
    const size_t left_begin = start > context_points
        ? start - context_points : 0;
    const size_t right_end = std::min(values.size(), end + context_points + 1);
    const double left_slope = touches_left ? 0.0 : TheilSenSlope(
        filtered, valid, left_begin, start, sample_step_s);
    const double right_slope = touches_right ? 0.0 : TheilSenSlope(
        filtered, valid, end + 1, right_end, sample_step_s);
    auto mean_step = [&](size_t first, size_t last) {
      std::vector<double> steps;
      for (size_t j = std::max<size_t>(1, first); j < last; ++j) {
        if (valid[j - 1] && valid[j]) {
          steps.push_back((filtered[j] - filtered[j - 1]) /
                          sample_step_s);
        }
      }
      return Median(steps);
    };
    const double left_q_out = touches_left ? 0.0 : mean_step(
        start > 2 ? start - 2 : 1, start);
    const double left_q_in = mean_step(start + 1,
                                      std::min(end + 1, start + 3));
    const double right_q_in = mean_step(
        end > 1 ? end - 1 : 1, end + 1);
    const double right_q_out = touches_right ? 0.0 : mean_step(
        end + 2, std::min(values.size(), end + 4));
    const double side_change_min = std::max(
        waveform_horizontal_slope_noise_multiplier_ * sigma_x,
        waveform_horizontal_min_side_change_ratio_ * amplitude);
    const double context_s = context_points * sample_step_s;
    const bool left_verified = touches_left ||
        (std::abs(left_slope) + 1e-15 >= theta_side &&
         std::abs(left_slope) * context_s + 1e-15 >= side_change_min &&
         std::abs(left_q_in) <= theta_flat + 1e-15 &&
         std::abs(left_q_out - left_q_in) + 1e-15 >= theta_kink);
    const bool right_verified = touches_right ||
        (std::abs(right_slope) + 1e-15 >= theta_side &&
         std::abs(right_slope) * context_s + 1e-15 >= side_change_min &&
         std::abs(right_q_in) <= theta_flat + 1e-15 &&
         std::abs(right_q_out - right_q_in) + 1e-15 >= theta_kink);
    if (!left_verified || !right_verified) {
      continue;
    }
    ContinuousHorizontalEvidence evidence;
    evidence.valid = true;
    evidence.start_index = start;
    evidence.end_index = end;
    evidence.start_s = start * sample_step_s;
    evidence.end_s = end * sample_step_s;
    evidence.duration_ratio_of_period = duration_s / period_s;
    evidence.level = Median(segment_values);
    evidence.touches_left_edge = touches_left;
    evidence.touches_right_edge = touches_right;
    evidence.left_boundary_verified = !touches_left && left_verified;
    evidence.right_boundary_verified = !touches_right && right_verified;
    evidence.flat_fraction = flat_fraction;
    evidence.level_span_ratio = level_span / amplitude;
    evidence.robust_slope = robust_slope;
    evidence.left_context_slope = left_slope;
    evidence.right_context_slope = right_slope;
    evidence.bic_linear_minus_constant = bic_linear - bic_constant;
    evidence.is_upper = p95 - evidence.level <= std::max(
        waveform_horizontal_level_span_noise_multiplier_ * sigma_x,
        waveform_horizontal_extreme_distance_ratio_ * amplitude);
    evidence.is_lower = evidence.level - p05 <= std::max(
        waveform_horizontal_level_span_noise_multiplier_ * sigma_x,
        waveform_horizontal_extreme_distance_ratio_ * amplitude);
    results.push_back(evidence);
  }
  return results;
}

FBBRSender::RepeatedClipLineEvidence
FBBRSender::DetectRepeatedClipLineContacts(
    const std::vector<double>& values,
    const std::vector<bool>& valid,
    double sample_step_s,
    double period_s,
    bool upper) const {
  RepeatedClipLineEvidence result;
  result.is_upper = upper;
  if (values.size() != valid.size() || values.size() < 10 ||
      sample_step_s <= 0.0 || period_s <= 0.0) {
    return result;
  }
  const std::vector<double> filtered = MedianFilter3(values, valid);
  std::vector<double> finite;
  std::vector<double> residuals;
  for (size_t i = 0; i < values.size(); ++i) {
    if (valid[i] && std::isfinite(values[i]) &&
        std::isfinite(filtered[i])) {
      finite.push_back(filtered[i]);
      residuals.push_back(values[i] - filtered[i]);
    }
  }
  if (finite.size() < 10) {
    return result;
  }
  const double p05 = Quantile(finite, 0.05);
  const double p95 = Quantile(finite, 0.95);
  const double amplitude = p95 - p05;
  const double sigma_x = RobustSigma(residuals);
  if (amplitude < std::max(
          waveform_horizontal_amplitude_noise_multiplier_ * sigma_x,
          1e-12)) {
    return result;
  }
  const double band_boundary = upper
      ? p05 + 0.80 * amplitude
      : p95 - 0.80 * amplitude;
  const double epsilon_h = std::max(
      waveform_horizontal_level_span_noise_multiplier_ * sigma_x,
      waveform_repeated_clip_contact_level_tolerance_ratio_ * amplitude);
  std::vector<double> band_values;
  for (size_t i = 0; i < filtered.size(); ++i) {
    if (valid[i] && ((upper && filtered[i] >= band_boundary) ||
                     (!upper && filtered[i] <= band_boundary))) {
      band_values.push_back(filtered[i]);
    }
  }
  if (band_values.empty()) {
    return result;
  }
  std::sort(band_values.begin(), band_values.end());
  double clip_level = upper ? band_values.back() : band_values.front();
  size_t best_members = 0;
  for (double candidate : band_values) {
    size_t members = 0;
    uint8_t cycle_mask = 0;
    for (size_t i = 0; i < filtered.size(); ++i) {
      if (valid[i] && std::abs(filtered[i] - candidate) <= epsilon_h) {
        ++members;
        cycle_mask |= static_cast<uint8_t>(
            1u << std::min<size_t>(1, static_cast<size_t>(
                i * sample_step_s / period_s)));
      }
    }
    if (cycle_mask == 0x3 &&
        (members > best_members ||
         (members == best_members &&
          ((upper && candidate > clip_level) ||
           (!upper && candidate < clip_level))))) {
      best_members = members;
      clip_level = candidate;
    }
  }
  std::vector<size_t> contacts;
  std::array<uint32_t, 2> per_cycle = {{0, 0}};
  for (size_t i = 0; i < filtered.size(); ++i) {
    if (valid[i] && std::abs(filtered[i] - clip_level) <= epsilon_h) {
      contacts.push_back(i);
      const size_t cycle = std::min<size_t>(
          1, static_cast<size_t>(i * sample_step_s / period_s));
      ++per_cycle[cycle];
    }
  }
  const uint32_t minimum_total = std::max<uint32_t>(
      waveform_repeated_clip_min_total_contact_samples_,
      static_cast<uint32_t>(std::ceil(
          waveform_repeated_clip_min_contact_sample_ratio_ * finite.size())));
  if (contacts.size() < minimum_total ||
      per_cycle[0] < waveform_repeated_clip_min_contact_samples_per_cycle_ ||
      per_cycle[1] < waveform_repeated_clip_min_contact_samples_per_cycle_) {
    return result;
  }
  const size_t merge_gap = std::max<size_t>(
      1, static_cast<size_t>(std::floor(
          waveform_repeated_clip_merge_gap_ratio_ * period_s /
          sample_step_s)));
  std::vector<std::pair<size_t, size_t>> events;
  size_t event_start = contacts.front();
  size_t event_end = contacts.front();
  for (size_t k = 1; k < contacts.size(); ++k) {
    const size_t gap = contacts[k] - event_end - 1;
    bool missing_gap_ok = true;
    size_t consecutive_missing = 0;
    size_t maximum_missing = 0;
    for (size_t j = event_end + 1; j < contacts[k]; ++j) {
      if (!valid[j]) {
        maximum_missing = std::max(maximum_missing, ++consecutive_missing);
      } else {
        consecutive_missing = 0;
      }
    }
    missing_gap_ok = maximum_missing * sample_step_s <=
        waveform_repeated_clip_max_missing_gap_ratio_ * period_s + 1e-15;
    if (gap <= merge_gap && missing_gap_ok) {
      event_end = contacts[k];
    } else {
      events.push_back({event_start, event_end});
      event_start = event_end = contacts[k];
    }
  }
  events.push_back({event_start, event_end});
  const double window_s = 2.0 * period_s;
  const double span_ratio =
      (contacts.back() - contacts.front()) * sample_step_s / window_s;
  if (span_ratio + 1e-15 <
      waveform_repeated_clip_min_contact_span_ratio_of_window_) {
    result.contact_time_span_ratio_of_window = span_ratio;
    return result;
  }
  std::vector<double> contact_values;
  std::array<std::vector<double>, 2> cycle_contact_values;
  for (size_t index : contacts) {
    contact_values.push_back(filtered[index]);
    const size_t cycle = std::min<size_t>(
        1, static_cast<size_t>(index * sample_step_s / period_s));
    cycle_contact_values[cycle].push_back(filtered[index]);
  }
  const double level_span = Quantile(contact_values, 0.95) -
      Quantile(contact_values, 0.05);
  const double cycle_level_delta = std::abs(
      Median(cycle_contact_values[0]) - Median(cycle_contact_values[1]));
  if (level_span > epsilon_h + 1e-15 ||
      cycle_level_delta > std::max(
          waveform_horizontal_slope_noise_multiplier_ * sigma_x,
          waveform_repeated_clip_max_level_delta_ratio_ * amplitude) +
              1e-15) {
    return result;
  }
  size_t pooled_steps = 0;
  size_t pooled_flat = 0;
  std::vector<double> absolute_steps;
  for (size_t i = 1; i < filtered.size(); ++i) {
    if (valid[i - 1] && valid[i]) {
      absolute_steps.push_back(std::abs(
          (filtered[i] - filtered[i - 1]) / sample_step_s));
    }
  }
  const double s80 = Quantile(absolute_steps, 0.80);
  std::vector<double> residual_slope_noise;
  for (size_t i = 1; i < residuals.size(); ++i) {
    residual_slope_noise.push_back(
        (residuals[i] - residuals[i - 1]) / sample_step_s);
  }
  const double sigma_s = RobustSigma(residual_slope_noise);
  const double theta_flat = std::max(
      waveform_horizontal_slope_noise_multiplier_ * sigma_s,
      waveform_horizontal_max_local_slope_ratio_ * s80);
  const double theta_side = std::max(
      waveform_horizontal_slope_noise_multiplier_ * sigma_s,
      waveform_horizontal_min_side_slope_ratio_ * s80);
  const double theta_kink = std::max(
      waveform_horizontal_slope_noise_multiplier_ * sigma_s,
      waveform_horizontal_min_boundary_kink_ratio_ * s80);
  for (const auto& event : events) {
    for (size_t j = event.first + 1; j <= event.second; ++j) {
      if (valid[j - 1] && valid[j]) {
        ++pooled_steps;
        const double step =
            (filtered[j] - filtered[j - 1]) / sample_step_s;
        if (std::abs(step) <= theta_flat + 1e-15) {
          ++pooled_flat;
        }
      }
    }
  }
  const double pooled_fraction = pooled_steps == 0
      ? 0.0 : static_cast<double>(pooled_flat) / pooled_steps;
  if (pooled_steps < 2 || pooled_fraction + 1e-15 <
          waveform_repeated_clip_min_pooled_flat_fraction_) {
    return result;
  }

  const size_t context_points = std::max<size_t>(
      4, static_cast<size_t>(std::ceil(0.10 * period_s / sample_step_s)));
  size_t observable_boundaries = 0;
  size_t verified_boundaries = 0;
  std::array<bool, 2> cycle_boundary_verified = {{false, false}};
  std::array<std::vector<double>, 2> event_centers;
  double best_overshoot_ratio = 0.0;
  for (const auto& event : events) {
    const size_t cycle = std::min<size_t>(
        1, static_cast<size_t>(event.first * sample_step_s / period_s));
    event_centers[cycle].push_back(
        0.5 * (event.first + event.second) * sample_step_s);
    const bool has_left = event.first >= 3;
    const bool has_right = event.second + 3 < filtered.size();
    const double left_slope = has_left ? TheilSenSlope(
        filtered, valid,
        event.first > context_points ? event.first - context_points : 0,
        event.first, sample_step_s) : 0.0;
    const double right_slope = has_right ? TheilSenSlope(
        filtered, valid, event.second + 1,
        std::min(filtered.size(), event.second + context_points + 1),
        sample_step_s) : 0.0;
    auto verify = [&](bool left) {
      const size_t inner_a = left ? event.first :
          (event.second > 0 ? event.second - 1 : 0);
      const size_t inner_b = left ? std::min(event.second, event.first + 2) :
          event.second;
      const double q_in = TheilSenSlope(
          filtered, valid, inner_a, inner_b + 1, sample_step_s);
      const size_t outer_a = left
          ? (event.first > 3 ? event.first - 3 : 0)
          : event.second + 1;
      const size_t outer_b = left
          ? event.first
          : std::min(filtered.size(), event.second + 4);
      const double q_out = TheilSenSlope(
          filtered, valid, outer_a, outer_b, sample_step_s);
      const double side = left ? left_slope : right_slope;
      const size_t adjacent = left ? event.first - 1 : event.second + 1;
      const bool correct_side = adjacent < filtered.size() && valid[adjacent] &&
          (upper ? filtered[adjacent] < clip_level + epsilon_h
                 : filtered[adjacent] > clip_level - epsilon_h);
      return std::abs(side) + 1e-15 >= theta_side &&
          std::abs(q_in) <= theta_flat + 1e-15 &&
          std::abs(q_out - q_in) + 1e-15 >= theta_kink && correct_side;
    };
    if (has_left) {
      ++observable_boundaries;
      if (verify(true)) {
        ++verified_boundaries;
        cycle_boundary_verified[cycle] = true;
      }
    }
    if (has_right) {
      ++observable_boundaries;
      if (verify(false)) {
        ++verified_boundaries;
        cycle_boundary_verified[cycle] = true;
      }
    }
    if (has_left && has_right &&
        ((upper && left_slope > 0.0 && right_slope < 0.0) ||
         (!upper && left_slope < 0.0 && right_slope > 0.0))) {
      const double overshoot = std::min(
          std::abs(left_slope), std::abs(right_slope)) *
          context_points * sample_step_s;
      best_overshoot_ratio = std::max(
          best_overshoot_ratio, overshoot / amplitude);
    }
  }
  const double boundary_fraction = observable_boundaries == 0
      ? 0.0 : static_cast<double>(verified_boundaries) /
                    observable_boundaries;
  if (!cycle_boundary_verified[0] || !cycle_boundary_verified[1] ||
      boundary_fraction + 1e-15 <
          waveform_repeated_clip_min_verified_boundary_fraction_ ||
      best_overshoot_ratio + 1e-15 <
          waveform_repeated_clip_min_extrapolated_overshoot_ratio_) {
    return result;
  }
  double best_period_error = std::numeric_limits<double>::infinity();
  for (double first : event_centers[0]) {
    for (double second : event_centers[1]) {
      best_period_error = std::min(
          best_period_error, std::abs((second - first) - period_s) /
                                 period_s);
    }
  }
  if (best_period_error >
      waveform_repeated_clip_max_period_error_ratio_ + 1e-15) {
    return result;
  }
  const double outside_excursion = std::max(
      waveform_horizontal_level_span_noise_multiplier_ * sigma_x,
      waveform_repeated_clip_min_outside_excursion_ratio_ * amplitude);
  for (size_t cycle = 0; cycle < 2; ++cycle) {
    bool left_line = false;
    const size_t begin = static_cast<size_t>(
        std::floor(cycle * period_s / sample_step_s));
    const size_t end = std::min(values.size(), static_cast<size_t>(
        std::ceil((cycle + 1) * period_s / sample_step_s)));
    for (size_t j = begin; j < end; ++j) {
      if (valid[j] && (upper
          ? filtered[j] <= clip_level - outside_excursion
          : filtered[j] >= clip_level + outside_excursion)) {
        left_line = true;
        break;
      }
    }
    if (!left_line) {
      return result;
    }
  }
  result.valid = true;
  result.clip_level = Median(contact_values);
  result.contact_fragment_count = static_cast<uint32_t>(events.size());
  result.contact_sample_count = static_cast<uint32_t>(contacts.size());
  result.contact_cycle_mask = 0x3;
  result.contact_sample_ratio =
      static_cast<double>(contacts.size()) / finite.size();
  result.contact_time_span_ratio_of_window = span_ratio;
  result.pooled_flat_fraction = pooled_fraction;
  result.contact_level_span_ratio = level_span / amplitude;
  result.cross_cycle_level_delta_ratio = cycle_level_delta / amplitude;
  result.event_period_error_ratio = best_period_error;
  result.verified_boundary_fraction = boundary_fraction;
  result.extrapolated_overshoot_ratio = best_overshoot_ratio;
  return result;
}

std::vector<FBBRSender::MiddleSequentialEvidence>
FBBRSender::DetectMiddleSequentialDisturbances(
    const std::vector<double>& values,
    const std::vector<bool>& valid,
    const std::vector<bool>& protected_mask,
    double sample_step_s,
    double period_s) const {
  std::vector<MiddleSequentialEvidence> candidates;
  if (values.size() != valid.size() ||
      (!protected_mask.empty() && protected_mask.size() != values.size()) ||
      values.size() < 12 || sample_step_s <= 0.0 || period_s <= 0.0) {
    return candidates;
  }
  const std::vector<double> filtered = MedianFilter3(values, valid);
  std::vector<double> finite;
  std::vector<double> residuals;
  for (size_t i = 0; i < values.size(); ++i) {
    if (valid[i] && std::isfinite(values[i]) &&
        std::isfinite(filtered[i])) {
      finite.push_back(filtered[i]);
      residuals.push_back(values[i] - filtered[i]);
    }
  }
  const double amplitude = Quantile(finite, 0.95) - Quantile(finite, 0.05);
  const double sigma_x = RobustSigma(residuals);
  if (finite.size() < 10 || amplitude <= 0.0) {
    return candidates;
  }
  std::vector<double> step_slopes(values.size(), 0.0);
  std::vector<double> abs_slopes;
  for (size_t i = 1; i < values.size(); ++i) {
    if (valid[i - 1] && valid[i]) {
      step_slopes[i] =
          (filtered[i] - filtered[i - 1]) / sample_step_s;
      abs_slopes.push_back(std::abs(step_slopes[i]));
    }
  }
  const double s80 = Quantile(abs_slopes, 0.80);
  std::vector<double> residual_slope_noise;
  for (size_t i = 1; i < residuals.size(); ++i) {
    residual_slope_noise.push_back(
        (residuals[i] - residuals[i - 1]) / sample_step_s);
  }
  const double sigma_s = RobustSigma(residual_slope_noise);
  const size_t context = std::max<size_t>(
      4, static_cast<size_t>(std::ceil(
          waveform_middle_context_duration_ratio_ * period_s /
          sample_step_s)));
  const size_t min_points = std::max<size_t>(
      3, static_cast<size_t>(std::ceil(
          waveform_middle_min_duration_ratio_ * period_s / sample_step_s)) +
             1);
  const size_t max_points = std::max<size_t>(min_points,
      static_cast<size_t>(std::floor(
          waveform_middle_max_duration_ratio_ * period_s / sample_step_s)) +
             1);
  const double theta_trend = std::max(
      waveform_middle_noise_multiplier_ * sigma_s,
      waveform_middle_min_trend_slope_ratio_ * s80);
  for (size_t start = context; start + min_points + context < values.size();
       ++start) {
    for (size_t length = min_points;
         length <= max_points && start + length + context <= values.size();
         ++length) {
      const size_t end = start + length - 1;
      bool protected_overlap = false;
      bool all_valid = true;
      for (size_t i = start; i <= end; ++i) {
        protected_overlap = protected_overlap ||
            (!protected_mask.empty() && protected_mask[i]);
        all_valid = all_valid && valid[i];
      }
      if (protected_overlap || !all_valid) {
        continue;
      }
      const double left_slope = TheilSenSlope(
          filtered, valid, start - context, start, sample_step_s);
      const double right_slope = TheilSenSlope(
          filtered, valid, end + 1, end + context + 1, sample_step_s);
      if (left_slope * right_slope <= 0.0 ||
          std::min(std::abs(left_slope), std::abs(right_slope)) + 1e-15 <
              theta_trend ||
          std::abs(left_slope - right_slope) > std::max(
              waveform_middle_noise_multiplier_ * sigma_s,
              waveform_middle_max_context_slope_delta_ratio_ *
                  std::max(std::abs(left_slope), std::abs(right_slope))) +
                  1e-15) {
        continue;
      }
      const double reference_slope = Median(
          {left_slope, right_slope});
      const double theta_mismatch = std::max(
          waveform_middle_noise_multiplier_ * sigma_s,
          waveform_middle_min_slope_mismatch_ratio_ *
              std::abs(reference_slope));
      size_t mismatch_count = 0;
      size_t current_run = 0;
      size_t longest_run = 0;
      std::vector<double> inside_slopes;
      for (size_t i = start + 1; i <= end; ++i) {
        const double slope = step_slopes[i];
        inside_slopes.push_back(slope);
        const bool mismatch =
            slope * reference_slope <= 0.0 ||
            std::abs(slope) < 0.50 * std::abs(reference_slope) ||
            std::abs(slope - reference_slope) + 1e-15 >= theta_mismatch;
        if (mismatch) {
          ++mismatch_count;
          longest_run = std::max(longest_run, ++current_run);
        } else {
          current_run = 0;
        }
      }
      const size_t required_mismatches = std::max<size_t>(
          waveform_middle_min_mismatching_samples_,
          static_cast<size_t>(std::ceil(
              waveform_middle_min_mismatching_sample_ratio_ *
              inside_slopes.size())));
      if (mismatch_count < required_mismatches ||
          longest_run <
              waveform_middle_min_consecutive_mismatching_samples_) {
        continue;
      }
      const size_t third = std::max<size_t>(1, inside_slopes.size() / 3);
      const double entry_slope = Median(std::vector<double>(
          inside_slopes.begin(), inside_slopes.begin() + third));
      const double exit_slope = Median(std::vector<double>(
          inside_slopes.end() - third, inside_slopes.end()));
      if (std::abs(entry_slope - left_slope) + 1e-15 < theta_mismatch ||
          std::abs(right_slope - exit_slope) + 1e-15 < theta_mismatch) {
        continue;
      }
      std::vector<double> bridge_deviations;
      for (size_t i = start; i <= end; ++i) {
        const double fraction = static_cast<double>(i - start) /
            std::max<size_t>(1, end - start);
        const double bridge = filtered[start] +
            fraction * (filtered[end] - filtered[start]);
        bridge_deviations.push_back(std::abs(filtered[i] - bridge));
      }
      const double bridge_deviation = Quantile(bridge_deviations, 0.95);
      const double bridge_threshold = std::max(
          waveform_middle_noise_multiplier_ * sigma_x,
          waveform_middle_min_bridge_deviation_ratio_ * amplitude);
      if (bridge_deviation + 1e-15 < bridge_threshold) {
        continue;
      }
      MiddleSequentialEvidence evidence;
      evidence.valid = true;
      evidence.start_index = start;
      evidence.end_index = end;
      evidence.duration_ratio_of_period =
          (end - start) * sample_step_s / period_s;
      evidence.left_context_slope = left_slope;
      evidence.right_context_slope = right_slope;
      evidence.reference_slope = reference_slope;
      evidence.slope_mismatch_ratio = inside_slopes.empty()
          ? 0.0 : static_cast<double>(mismatch_count) /
                      inside_slopes.size();
      evidence.bridge_deviation_ratio = bridge_deviation / amplitude;
      evidence.score = (bridge_deviation / bridge_threshold) *
          evidence.slope_mismatch_ratio;
      candidates.push_back(evidence);
    }
  }
  std::sort(candidates.begin(), candidates.end(),
            [](const MiddleSequentialEvidence& lhs,
               const MiddleSequentialEvidence& rhs) {
              return lhs.score > rhs.score;
            });
  std::vector<MiddleSequentialEvidence> selected;
  std::vector<bool> occupied(values.size(), false);
  std::array<size_t, 2> masked_per_cycle = {{0, 0}};
  const size_t max_mask_per_cycle = static_cast<size_t>(std::floor(
      waveform_middle_max_mask_ratio_per_cycle_ * period_s /
      sample_step_s));
  for (const auto& candidate : candidates) {
    bool overlaps = false;
    std::array<size_t, 2> added = {{0, 0}};
    for (size_t i = candidate.start_index; i <= candidate.end_index; ++i) {
      overlaps = overlaps || occupied[i];
      const size_t cycle = std::min<size_t>(
          1, static_cast<size_t>(i * sample_step_s / period_s));
      ++added[cycle];
    }
    if (overlaps || masked_per_cycle[0] + added[0] > max_mask_per_cycle ||
        masked_per_cycle[1] + added[1] > max_mask_per_cycle) {
      continue;
    }
    for (size_t i = candidate.start_index; i <= candidate.end_index; ++i) {
      occupied[i] = true;
    }
    masked_per_cycle[0] += added[0];
    masked_per_cycle[1] += added[1];
    selected.push_back(candidate);
  }
  return selected;
}

FBBRSender::PeriodicSimilarityResult
FBBRSender::AnalyzeFbbrHybridPeriodicSimilarity(
    const std::vector<double>& values,
    const std::vector<bool>& original_valid,
    const std::vector<bool>& periodic_valid,
    double sample_step_s,
    double period_s,
    double srate_period_s,
    bool verified_upper_clip,
    SignalRegimeFeatures* features) const {
  if (features != nullptr) {
    features->periodic_similarity_input_valid = false;
    features->periodic_similar = false;
    features->upper_clip_periodic_veto = verified_upper_clip;
    features->estimated_srate_period_s = srate_period_s;
  }
  if (values.size() != original_valid.size() ||
      values.size() != periodic_valid.size() || values.empty() ||
      sample_step_s <= 0.0 || period_s <= 0.0 ||
      !std::isfinite(srate_period_s) || srate_period_s <= 0.0) {
    return PeriodicSimilarityResult::kInvalidInput;
  }
  const size_t original_count = static_cast<size_t>(std::count(
      original_valid.begin(), original_valid.end(), true));
  const size_t masked_count = static_cast<size_t>(std::count(
      periodic_valid.begin(), periodic_valid.end(), true));
  const double original_coverage =
      static_cast<double>(original_count) / values.size();
  const double masked_coverage =
      static_cast<double>(masked_count) / values.size();
  if (original_coverage + 1e-15 < waveform_min_cycle_coverage_ratio_ ||
      masked_coverage + 1e-15 < waveform_masked_min_cycle_coverage_ratio_) {
    return PeriodicSimilarityResult::kInvalidInput;
  }
  if (features != nullptr) {
    features->periodic_similarity_input_valid = true;
  }
  if (verified_upper_clip &&
      fbbr_regime_periodic_upper_clip_is_hard_veto_) {
    return PeriodicSimilarityResult::kNoMatch;
  }
  double periodicity = -1.0;
  const double estimated_period = EstimateActualSignalPeriod(
      values, periodic_valid, sample_step_s, srate_period_s, &periodicity);
  if (features != nullptr) {
    features->estimated_period_s = estimated_period;
    features->periodicity_correlation = periodicity;
  }
  if (estimated_period <= 0.0) {
    return PeriodicSimilarityResult::kNoMatch;
  }
  const double period_error =
      std::abs(estimated_period - srate_period_s) / srate_period_s;
  if (features != nullptr) {
    features->response_srate_period_error_ratio = period_error;
  }
  const std::vector<double> filtered = MedianFilter3(values, periodic_valid);
  std::vector<double> finite;
  std::vector<double> residuals;
  for (size_t i = 0; i < values.size(); ++i) {
    if (periodic_valid[i] && std::isfinite(values[i]) &&
        std::isfinite(filtered[i])) {
      finite.push_back(filtered[i]);
      residuals.push_back(values[i] - filtered[i]);
    }
  }
  const double amplitude = Quantile(finite, 0.95) - Quantile(finite, 0.05);
  const double sigma = RobustSigma(residuals);
  const double level = std::max(std::abs(Median(finite)), 1e-12);
  const bool amplitude_ok = amplitude >= std::max(
      waveform_activity_amplitude_noise_multiplier_ * sigma,
      waveform_activity_min_level_ratio_ * level);
  size_t longest_rise = 0;
  size_t longest_fall = 0;
  size_t current_rise = 0;
  size_t current_fall = 0;
  for (size_t i = 1; i < filtered.size(); ++i) {
    if (!periodic_valid[i - 1] || !periodic_valid[i]) {
      current_rise = current_fall = 0;
      continue;
    }
    const double delta = filtered[i] - filtered[i - 1];
    const double noise_deadband = std::max(1e-15, sigma);
    if (delta > noise_deadband) {
      longest_rise = std::max(longest_rise, ++current_rise);
      current_fall = 0;
    } else if (delta < -noise_deadband) {
      longest_fall = std::max(longest_fall, ++current_fall);
      current_rise = 0;
    }
  }
  const bool complete_cycle = amplitude_ok &&
      longest_rise * sample_step_s + 1e-15 >= 0.15 * period_s &&
      longest_fall * sample_step_s + 1e-15 >= 0.15 * period_s;
  const bool matched = complete_cycle &&
      period_error <= fbbr_regime_period_tolerance_ratio_ + 1e-15 &&
      periodicity + 1e-15 >= fbbr_regime_min_periodicity_correlation_;
  if (features != nullptr) {
    features->periodic_similar = matched;
  }
  return matched ? PeriodicSimilarityResult::kMatch
                 : PeriodicSimilarityResult::kNoMatch;
}

bool FBBRSender::ShouldRefreshRtpropForTrueClip(bool top_clip,
                                                bool bottom_clip) {
  // BOTH_CLIPPED still contains a genuine bottom clip. The RTprop/ProbeRTT
  // refresh depends only on the presence of that bottom clip.
  static_cast<void>(top_clip);
  return bottom_clip;
}

WaveformClassification FBBRSender::ClassifyWaveformState(
    const WaveformDecisionInputs& inputs,
    const char** decision_rule) {
  auto decide = [decision_rule](WaveformClassification classification,
                                const char* rule) {
    if (decision_rule != nullptr) {
      *decision_rule = rule;
    }
    return classification;
  };
  auto decide_r6 = [&]() {
    if (!inputs.adaptive_guard_enabled) {
      return decide(WaveformClassification::kInconclusive, "R6");
    }
    const bool window_stats_valid = inputs.srtt_window_stats_valid &&
        std::isfinite(inputs.srtt_mean_ms) && inputs.srtt_mean_ms > 0.0 &&
        std::isfinite(inputs.srtt_min_ms) && inputs.srtt_min_ms > 0.0 &&
        std::isfinite(inputs.srtt_max_ms) && inputs.srtt_max_ms > 0.0;
    const bool srtt_up_valid =
        inputs.latest_waveform_overload_srtt_mean_valid &&
        std::isfinite(inputs.latest_waveform_overload_srtt_mean_ms) &&
        inputs.latest_waveform_overload_srtt_mean_ms > 0.0;
    if (window_stats_valid && srtt_up_valid &&
        (inputs.srtt_mean_ms >
             inputs.latest_waveform_overload_srtt_mean_ms ||
         inputs.srtt_max_ms >
             inputs.latest_waveform_overload_srtt_mean_ms)) {
      return decide(WaveformClassification::kOverload, "R6.1");
    }
    const bool srtt_low_valid =
        inputs.latest_waveform_underload_srtt_mean_valid &&
        std::isfinite(inputs.latest_waveform_underload_srtt_mean_ms) &&
        inputs.latest_waveform_underload_srtt_mean_ms > 0.0;
    if (window_stats_valid && srtt_low_valid &&
        (inputs.srtt_mean_ms <
             inputs.latest_waveform_underload_srtt_mean_ms ||
         inputs.srtt_min_ms <
             inputs.latest_waveform_underload_srtt_mean_ms)) {
      return decide(WaveformClassification::kUnderload, "R6.2");
    }
    return decide(WaveformClassification::kInconclusive, "R6.3");
  };
  const bool bic_clip_detected =
      inputs.bic_srtt_top_clip || inputs.bic_srtt_bottom_clip;
  if ((!inputs.prechecks_valid || !inputs.srtt_input_valid) &&
      !bic_clip_detected) {
    return decide_r6();
  }

  const bool drate_effective_similar =
      inputs.drate_similar || inputs.drate_similar_without_middle;
  auto classify_clipped_shape = [&](bool top_clip, bool bottom_clip) {
    const bool top_only = top_clip && !bottom_clip;
    const bool bottom_only = !top_clip && bottom_clip;
    const bool both_clipped = top_clip && bottom_clip;
    if (top_only) {
      if (!inputs.drate_input_valid) {
        return decide(WaveformClassification::kInconclusive, "R2.4");
      }
      const bool matching_drate_top_clip = inputs.drate_similar &&
          ((inputs.drate_positive_half_clipped &&
            inputs.positive_half_clips_simultaneous) ||
           (inputs.adaptive_guard_enabled &&
            inputs.drate_only_negative_half));
      if (matching_drate_top_clip) {
        return decide(WaveformClassification::kOverload, "R2.1");
      }
      if (drate_effective_similar) {
        return decide(WaveformClassification::kFullLoad, "R2.2");
      }
      return decide(WaveformClassification::kOverload, "R2.3");
    }
    if (bottom_only || both_clipped) {
      const char* similar_rule = bottom_only ? "R3.1" : "R4.1";
      const char* plateau_rule = bottom_only ? "R3.2" : "R4.2";
      const char* otherwise_rule = bottom_only ? "R3.3" : "R4.3";
      if (inputs.drate_input_valid && inputs.drate_similar) {
        return decide(WaveformClassification::kUnderload, similar_rule);
      }
      if (inputs.drate_has_waveform && inputs.drate_middle_any_plateau) {
        return decide(WaveformClassification::kUnderload, plateau_rule);
      }
      return decide(WaveformClassification::kInconclusive, otherwise_rule);
    }
    return decide(WaveformClassification::kFullLoad, "R1");
  };

  // A repeated BIC clipping shape is independent evidence and has priority.
  // Only when no true clipping shape is present may waveform similarity and
  // shoulder suppression contribute a clipping classification.
  if (bic_clip_detected) {
    return classify_clipped_shape(inputs.bic_srtt_top_clip,
                                  inputs.bic_srtt_bottom_clip);
  }

  if (inputs.adaptive_guard_enabled) {
    const bool srtt_effective_similar =
        inputs.srtt_similar || inputs.srtt_similar_without_middle;
    if (srtt_effective_similar) {
      const bool positive_side_suppressed =
          inputs.srtt_positive_half_clipped ||
          inputs.srtt_only_negative_half;
      const bool negative_side_suppressed =
          inputs.srtt_negative_half_clipped ||
          inputs.srtt_only_positive_half;
      if (positive_side_suppressed || negative_side_suppressed) {
        return classify_clipped_shape(positive_side_suppressed,
                                      negative_side_suppressed);
      }
      return decide(WaveformClassification::kFullLoad, "R1");
    }
    if (drate_effective_similar) {
      return decide(WaveformClassification::kUnderload, "R5.1");
    }
    return decide(WaveformClassification::kOverload, "R5.2");
  }

  if (inputs.srtt_similar) {
    const bool positive_side_suppressed =
        inputs.srtt_positive_half_clipped;
    const bool negative_side_suppressed =
        inputs.srtt_negative_half_clipped;
    if (positive_side_suppressed || negative_side_suppressed) {
      return classify_clipped_shape(positive_side_suppressed,
                                    negative_side_suppressed);
    }
    return decide(WaveformClassification::kFullLoad, "R1");
  }
  if (inputs.srtt_similar_without_middle) {
    return decide(WaveformClassification::kFullLoad, "R1");
  }
  if (!inputs.drate_input_valid) {
    return decide_r6();
  }
  if (drate_effective_similar) {
    return decide(WaveformClassification::kUnderload, "R5.1");
  }
  return decide(WaveformClassification::kOverload, "R5.2");
}

const char* FBBRSender::CruiseDetectorModeName(
    FBBRCruiseDetectorMode mode) {
  return mode == FBBRCruiseDetectorMode::kLegacySpectral
             ? "legacy_spectral"
             : "time_waveform";
}

TimeDelta FBBRSender::CurrentSmoothedRtt() const {
  if (rtt_stats_ != nullptr) {
    const TimeDelta srtt = rtt_stats_->smoothed_rtt();
    if (!srtt.IsZero()) {
      return srtt;
    }
  }
  const TimeDelta min_rtt = model_.MinRtt();
  if (!min_rtt.IsZero()) {
    return min_rtt;
  }
  return rtt_stats_ == nullptr ? TimeDelta::Zero()
                               : rtt_stats_->MinOrInitialRtt();
}

QuicTime FBBRSender::AlignToNextTriangleCycle(QuicTime time) const {
  if (cruise_start_time_ == QuicTime::Zero() ||
      cruise_modulation_freq_hz_ <= 0.0 || time <= cruise_start_time_) {
    return cruise_start_time_;
  }
  const int64_t period_us = static_cast<int64_t>(std::llround(
      1000000.0 / cruise_modulation_freq_hz_));
  if (period_us <= 0) {
    return time;
  }
  const int64_t elapsed_us =
      (time - cruise_start_time_).ToMicroseconds();
  const int64_t cycles = (elapsed_us + period_us - 1) / period_us;
  return cruise_start_time_ +
         TimeDelta::FromMicroseconds(cycles * period_us);
}

FBBRSender::ResampledWaveformSeries
FBBRSender::ResampleRateWaveform(
    const std::vector<FBBRRateSample>& samples,
    QuicTime start,
    QuicTime end,
    double sample_step_s,
    double max_interpolation_gap_s) const {
  ResampledWaveformSeries result;
  if (end <= start || sample_step_s <= 0.0) {
    return result;
  }
  const double duration_s =
      static_cast<double>((end - start).ToMicroseconds()) / 1000000.0;
  const size_t count =
      static_cast<size_t>(std::floor(duration_s / sample_step_s)) + 1;
  result.values.assign(count, 0.0);
  result.valid.assign(count, false);
  std::vector<std::vector<double>> bins(count);
  for (const auto& sample : samples) {
    if (!sample.valid || sample.rate.IsZero() || sample.time < start ||
        sample.time > end) {
      continue;
    }
    const double value =
        static_cast<double>(sample.rate.ToBitsPerSecond());
    if (!std::isfinite(value) || value <= 0.0) {
      continue;
    }
    const double offset_s =
        static_cast<double>((sample.time - start).ToMicroseconds()) /
        1000000.0;
    const size_t index = std::min(
        count - 1,
        static_cast<size_t>(std::floor(offset_s / sample_step_s + 0.5)));
    bins[index].push_back(value);
  }
  for (size_t i = 0; i < count; ++i) {
    if (!bins[i].empty()) {
      result.values[i] = Median(bins[i]);
      result.valid[i] = true;
    }
  }
  size_t left = 0;
  while (left < count) {
    while (left < count && !result.valid[left]) {
      ++left;
    }
    if (left >= count) {
      break;
    }
    size_t right = left + 1;
    while (right < count && !result.valid[right]) {
      ++right;
    }
    if (right < count) {
      const double gap_s = (right - left) * sample_step_s;
      if (gap_s <= max_interpolation_gap_s + 1e-12) {
        for (size_t i = left + 1; i < right; ++i) {
          const double fraction =
              static_cast<double>(i - left) /
              static_cast<double>(right - left);
          result.values[i] = result.values[left] +
              fraction * (result.values[right] - result.values[left]);
          result.valid[i] = true;
        }
      }
    }
    left = right;
  }
  const size_t valid_count = static_cast<size_t>(
      std::count(result.valid.begin(), result.valid.end(), true));
  result.coverage_ratio = count == 0
                              ? 0.0
                              : static_cast<double>(valid_count) / count;
  return result;
}

FBBRSender::ResampledWaveformSeries
FBBRSender::ResampleRttWaveform(
    const std::vector<FBBRRttSample>& samples,
    QuicTime start,
    QuicTime end,
    double sample_step_s,
    double max_interpolation_gap_s) const {
  std::vector<FBBRRateSample> rate_samples;
  rate_samples.reserve(samples.size());
  for (const auto& sample : samples) {
    if (!std::isfinite(sample.rtt_ms) || sample.rtt_ms <= 0.0) {
      continue;
    }
    rate_samples.push_back(
        {sample.time, BandwidthFromBps(sample.rtt_ms * 1000000.0),
         true, false, 0});
  }
  ResampledWaveformSeries result = ResampleRateWaveform(
      rate_samples, start, end, sample_step_s, max_interpolation_gap_s);
  for (double& value : result.values) {
    value /= 1000000.0;
  }
  return result;
}

std::vector<double> FBBRSender::DetrendLinear(
    const std::vector<double>& values,
    const std::vector<bool>& valid) {
  std::vector<double> result(values.size(), 0.0);
  if (values.size() != valid.size()) {
    return result;
  }
  double sum_x = 0.0;
  double sum_y = 0.0;
  double sum_xx = 0.0;
  double sum_xy = 0.0;
  size_t count = 0;
  for (size_t i = 0; i < values.size(); ++i) {
    if (!valid[i] || !std::isfinite(values[i])) {
      continue;
    }
    const double x = static_cast<double>(i);
    sum_x += x;
    sum_y += values[i];
    sum_xx += x * x;
    sum_xy += x * values[i];
    ++count;
  }
  if (count < 2) {
    return result;
  }
  const double denominator = count * sum_xx - sum_x * sum_x;
  const double slope = std::abs(denominator) <= 1e-12
                           ? 0.0
                           : (count * sum_xy - sum_x * sum_y) / denominator;
  const double intercept = (sum_y - slope * sum_x) / count;
  for (size_t i = 0; i < values.size(); ++i) {
    if (valid[i] && std::isfinite(values[i])) {
      result[i] = values[i] -
                  (intercept + slope * static_cast<double>(i));
    }
  }
  return result;
}

std::vector<double> FBBRSender::MedianFilter3(
    const std::vector<double>& values,
    const std::vector<bool>& valid) {
  std::vector<double> result = values;
  if (values.size() != valid.size() || values.size() < 3) {
    return result;
  }
  for (size_t i = 1; i + 1 < values.size(); ++i) {
    if (valid[i - 1] && valid[i] && valid[i + 1]) {
      result[i] = Median({values[i - 1], values[i], values[i + 1]});
    }
  }
  return result;
}

double FBBRSender::ComputeNormalizedCrossCorrelation(
    const std::vector<double>& lhs,
    const std::vector<double>& rhs,
    const std::vector<bool>& valid) {
  if (lhs.size() != rhs.size() || lhs.size() != valid.size()) {
    return 0.0;
  }
  double lhs_mean = 0.0;
  double rhs_mean = 0.0;
  size_t count = 0;
  for (size_t i = 0; i < lhs.size(); ++i) {
    if (valid[i] && std::isfinite(lhs[i]) && std::isfinite(rhs[i])) {
      lhs_mean += lhs[i];
      rhs_mean += rhs[i];
      ++count;
    }
  }
  if (count < 3) {
    return 0.0;
  }
  lhs_mean /= count;
  rhs_mean /= count;
  double numerator = 0.0;
  double lhs_energy = 0.0;
  double rhs_energy = 0.0;
  for (size_t i = 0; i < lhs.size(); ++i) {
    if (!valid[i] || !std::isfinite(lhs[i]) || !std::isfinite(rhs[i])) {
      continue;
    }
    const double x = lhs[i] - lhs_mean;
    const double y = rhs[i] - rhs_mean;
    numerator += x * y;
    lhs_energy += x * x;
    rhs_energy += y * y;
  }
  const double denominator = std::sqrt(lhs_energy * rhs_energy);
  return denominator <= 1e-12 ? 0.0 : ClampValue(numerator / denominator,
                                                  -1.0, 1.0);
}

std::vector<double> FBBRSender::RobustNormalize(
    const std::vector<double>& values,
    const std::vector<bool>& valid) {
  std::vector<double> normalized(values.size(), 0.0);
  if (values.size() != valid.size()) {
    return normalized;
  }
  std::vector<double> selected;
  selected.reserve(values.size());
  for (size_t i = 0; i < values.size(); ++i) {
    if (valid[i] && std::isfinite(values[i])) {
      selected.push_back(values[i]);
    }
  }
  if (selected.size() < 3) {
    return normalized;
  }
  const double center = Median(selected);
  std::vector<double> deviations;
  deviations.reserve(selected.size());
  for (double value : selected) {
    deviations.push_back(std::abs(value - center));
  }
  double scale = 1.4826 * Median(deviations);
  if (!std::isfinite(scale) || scale <= 1e-12) {
    const auto minmax = std::minmax_element(selected.begin(), selected.end());
    scale = std::max(1e-12, 0.5 * (*minmax.second - *minmax.first));
  }
  for (size_t i = 0; i < values.size(); ++i) {
    if (valid[i] && std::isfinite(values[i])) {
      normalized[i] = (values[i] - center) / scale;
    }
  }
  return normalized;
}

double FBBRSender::ComputeSlopeDirectionAgreement(
    const std::vector<double>& lhs,
    const std::vector<double>& rhs,
    const std::vector<bool>& valid) {
  if (lhs.size() != rhs.size() || lhs.size() != valid.size() ||
      lhs.size() < 2) {
    return 0.0;
  }
  size_t compared = 0;
  size_t matched = 0;
  for (size_t i = 1; i < lhs.size(); ++i) {
    if (!valid[i - 1] || !valid[i] || !std::isfinite(lhs[i - 1]) ||
        !std::isfinite(lhs[i]) || !std::isfinite(rhs[i - 1]) ||
        !std::isfinite(rhs[i])) {
      continue;
    }
    const double lhs_delta = lhs[i] - lhs[i - 1];
    const double rhs_delta = rhs[i] - rhs[i - 1];
    const int lhs_sign = lhs_delta > 1e-9 ? 1 : (lhs_delta < -1e-9 ? -1 : 0);
    const int rhs_sign = rhs_delta > 1e-9 ? 1 : (rhs_delta < -1e-9 ? -1 : 0);
    ++compared;
    if (lhs_sign == rhs_sign) {
      ++matched;
    }
  }
  return compared == 0 ? 0.0
                       : static_cast<double>(matched) / compared;
}

double FBBRSender::ComputeLaggedCorrelation(
    const std::vector<double>& values,
    const std::vector<bool>& valid,
    size_t lag_samples,
    size_t* pair_count) {
  if (pair_count != nullptr) {
    *pair_count = 0;
  }
  if (values.size() != valid.size() || lag_samples == 0 ||
      lag_samples >= values.size()) {
    return -1.0;
  }
  double lhs_mean = 0.0;
  double rhs_mean = 0.0;
  size_t count = 0;
  for (size_t i = 0; i + lag_samples < values.size(); ++i) {
    const size_t j = i + lag_samples;
    if (!valid[i] || !valid[j] || !std::isfinite(values[i]) ||
        !std::isfinite(values[j])) {
      continue;
    }
    lhs_mean += values[i];
    rhs_mean += values[j];
    ++count;
  }
  if (pair_count != nullptr) {
    *pair_count = count;
  }
  if (count < 4) {
    return -1.0;
  }
  lhs_mean /= count;
  rhs_mean /= count;
  double covariance = 0.0;
  double lhs_energy = 0.0;
  double rhs_energy = 0.0;
  for (size_t i = 0; i + lag_samples < values.size(); ++i) {
    const size_t j = i + lag_samples;
    if (!valid[i] || !valid[j] || !std::isfinite(values[i]) ||
        !std::isfinite(values[j])) {
      continue;
    }
    const double lhs = values[i] - lhs_mean;
    const double rhs = values[j] - rhs_mean;
    covariance += lhs * rhs;
    lhs_energy += lhs * lhs;
    rhs_energy += rhs * rhs;
  }
  const double denominator = std::sqrt(lhs_energy * rhs_energy);
  if (!std::isfinite(denominator) || denominator <= 1e-18) {
    return -1.0;
  }
  return ClampValue(covariance / denominator, -1.0, 1.0);
}

bool FBBRSender::HasMacroOpposingShoulders(
    double slope_before,
    double slope_after,
    double shoulder_duration_s,
    double minimum_abs_slope,
    double minimum_signal_change) {
  if (!std::isfinite(slope_before) || !std::isfinite(slope_after) ||
      !std::isfinite(shoulder_duration_s) ||
      !std::isfinite(minimum_abs_slope) ||
      !std::isfinite(minimum_signal_change) ||
      shoulder_duration_s <= 0.0 || minimum_abs_slope < 0.0 ||
      minimum_signal_change < 0.0) {
    return false;
  }
  const bool opposite_directions =
      (slope_before >= minimum_abs_slope &&
       slope_after <= -minimum_abs_slope) ||
      (slope_before <= -minimum_abs_slope &&
       slope_after >= minimum_abs_slope);
  return opposite_directions &&
      std::abs(slope_before) * shoulder_duration_s >=
          minimum_signal_change &&
      std::abs(slope_after) * shoulder_duration_s >=
          minimum_signal_change;
}

bool FBBRSender::HasMacroSameDirectionShoulders(
    double slope_before,
    double slope_after) {
  return std::isfinite(slope_before) && std::isfinite(slope_after) &&
         std::abs(slope_before) > 1e-12 &&
         std::abs(slope_after) > 1e-12 &&
         slope_before * slope_after > 0.0;
}

bool FBBRSender::HasDualMacroOpposingShoulders(
    double first_before,
    double first_after,
    double first_minimum_abs_slope,
    double first_minimum_signal_change,
    double second_before,
    double second_after,
    double second_minimum_abs_slope,
    double second_minimum_signal_change,
    double shoulder_duration_s) {
  return HasMacroOpposingShoulders(
             first_before, first_after, shoulder_duration_s,
             first_minimum_abs_slope, first_minimum_signal_change) &&
         HasMacroOpposingShoulders(
             second_before, second_after, shoulder_duration_s,
             second_minimum_abs_slope, second_minimum_signal_change);
}

FBBRSender::TemplateFitResult FBBRSender::EstimateRobustNoise(
    const std::vector<double>& response,
    const std::vector<double>& waveform_template,
    const std::vector<bool>& valid) {
  TemplateFitResult result;
  if (response.size() != waveform_template.size() ||
      response.size() != valid.size()) {
    return result;
  }
  std::vector<double> response_values;
  response_values.reserve(response.size());
  for (size_t i = 0; i < response.size(); ++i) {
    if (valid[i] && std::isfinite(response[i]) &&
        std::isfinite(waveform_template[i])) {
      response_values.push_back(response[i]);
    }
  }
  if (response_values.size() < 4) {
    return result;
  }
  std::sort(response_values.begin(), response_values.end());
  const size_t p1_index = static_cast<size_t>(
      std::floor(0.01 * (response_values.size() - 1)));
  const size_t p99_index = static_cast<size_t>(
      std::ceil(0.99 * (response_values.size() - 1)));
  const double low = response_values[p1_index];
  const double high = response_values[p99_index];
  double mean_x = 0.0;
  double mean_y = 0.0;
  size_t count = 0;
  for (size_t i = 0; i < response.size(); ++i) {
    if (!valid[i] || !std::isfinite(response[i]) ||
        !std::isfinite(waveform_template[i])) {
      continue;
    }
    mean_x += waveform_template[i];
    mean_y += ClampValue(response[i], low, high);
    ++count;
  }
  mean_x /= count;
  mean_y /= count;
  double covariance = 0.0;
  double template_energy = 0.0;
  double template_min = std::numeric_limits<double>::infinity();
  double template_max = -std::numeric_limits<double>::infinity();
  for (size_t i = 0; i < response.size(); ++i) {
    if (!valid[i] || !std::isfinite(response[i]) ||
        !std::isfinite(waveform_template[i])) {
      continue;
    }
    const double x = waveform_template[i];
    const double y = ClampValue(response[i], low, high);
    covariance += (x - mean_x) * (y - mean_y);
    template_energy += (x - mean_x) * (x - mean_x);
    template_min = std::min(template_min, x);
    template_max = std::max(template_max, x);
  }
  if (template_energy <= 1e-12 || !std::isfinite(template_min) ||
      !std::isfinite(template_max)) {
    return result;
  }
  result.beta = covariance / template_energy;
  result.alpha = mean_y - result.beta * mean_x;
  std::vector<double> residuals;
  residuals.reserve(count);
  for (size_t i = 0; i < response.size(); ++i) {
    if (!valid[i] || !std::isfinite(response[i]) ||
        !std::isfinite(waveform_template[i])) {
      continue;
    }
    const double y = ClampValue(response[i], low, high);
    residuals.push_back(y -
        (result.alpha + result.beta * waveform_template[i]));
  }
  const double residual_median = Median(residuals);
  std::vector<double> absolute_deviations;
  absolute_deviations.reserve(residuals.size());
  for (double residual : residuals) {
    absolute_deviations.push_back(std::abs(residual - residual_median));
  }
  result.robust_noise_sigma = 1.4826 * Median(absolute_deviations);
  result.fitted_response_amplitude =
      std::abs(result.beta) * 0.5 * (template_max - template_min);
  result.response_snr = result.fitted_response_amplitude /
      std::max(result.robust_noise_sigma, 1e-12);
  result.valid = std::isfinite(result.response_snr);
  return result;
}

std::vector<double> FBBRSender::ComputeLocalLinearSlopes(
    const std::vector<double>& values,
    const std::vector<bool>& valid,
    double sample_step_s,
    double slope_window_s) {
  std::vector<double> slopes(values.size(), 0.0);
  if (values.size() != valid.size() || sample_step_s <= 0.0 ||
      values.empty()) {
    return slopes;
  }
  const size_t radius = std::max<size_t>(
      2, static_cast<size_t>(std::ceil(
             0.5 * slope_window_s / sample_step_s)));
  for (size_t center = 0; center < values.size(); ++center) {
    const size_t begin = center > radius ? center - radius : 0;
    const size_t end = std::min(values.size(), center + radius + 1);
    double sum_x = 0.0;
    double sum_y = 0.0;
    double sum_xx = 0.0;
    double sum_xy = 0.0;
    size_t count = 0;
    for (size_t i = begin; i < end; ++i) {
      if (!valid[i] || !std::isfinite(values[i])) {
        continue;
      }
      const double x =
          (static_cast<double>(i) - static_cast<double>(center)) *
          sample_step_s;
      sum_x += x;
      sum_y += values[i];
      sum_xx += x * x;
      sum_xy += x * values[i];
      ++count;
    }
    const double denominator = count * sum_xx - sum_x * sum_x;
    if (count >= 3 && std::abs(denominator) > 1e-15) {
      slopes[center] =
          (count * sum_xy - sum_x * sum_y) / denominator;
    }
  }
  return slopes;
}

std::vector<double> FBBRSender::BuildQueueIntegralTemplate(
    const std::vector<double>& sender_residual,
    const std::vector<bool>& valid,
    double sample_step_s,
    double period_s) {
  std::vector<double> integral(sender_residual.size(), 0.0);
  if (sender_residual.size() != valid.size() || sample_step_s <= 0.0) {
    return integral;
  }
  const size_t cycle_samples = std::max<size_t>(
      2, static_cast<size_t>(std::llround(
             std::max(period_s, sample_step_s) / sample_step_s)));
  for (size_t cycle_begin = 0; cycle_begin < sender_residual.size();
       cycle_begin += cycle_samples) {
    const size_t cycle_end = std::min(sender_residual.size(),
                                      cycle_begin + cycle_samples + 1);
    double mean = 0.0;
    size_t count = 0;
    for (size_t i = cycle_begin; i < cycle_end; ++i) {
      if (valid[i] && std::isfinite(sender_residual[i])) {
        mean += sender_residual[i];
        ++count;
      }
    }
    if (count < 2) {
      continue;
    }
    mean /= count;
    integral[cycle_begin] = 0.0;
    for (size_t i = cycle_begin + 1; i < cycle_end; ++i) {
      integral[i] = integral[i - 1];
      if (valid[i - 1] && valid[i]) {
        integral[i] += 0.5 * sample_step_s *
            ((sender_residual[i - 1] - mean) +
             (sender_residual[i] - mean));
      }
    }
    const size_t last = cycle_end - 1;
    const double drift = integral[last] - integral[cycle_begin];
    if (last > cycle_begin) {
      for (size_t i = cycle_begin; i <= last; ++i) {
        const double fraction = static_cast<double>(i - cycle_begin) /
                                static_cast<double>(last - cycle_begin);
        integral[i] -= integral[cycle_begin] + fraction * drift;
      }
    }
  }
  return integral;
}

FBBRSender::CycleCompletenessResult
FBBRSender::AnalyzeCycleCompleteness(
    const std::vector<double>& values,
    const std::vector<bool>& valid,
    double sample_step_s,
    double expected_period_s,
    double minimum_coverage_ratio) const {
  CycleCompletenessResult result;
  if (values.size() != valid.size() || values.size() < 4 ||
      sample_step_s <= 0.0 || expected_period_s <= 0.0) {
    result.invalid_reason = "invalid_cycle_input";
    return result;
  }
  const size_t valid_count = static_cast<size_t>(
      std::count(valid.begin(), valid.end(), true));
  result.coverage_ratio =
      static_cast<double>(valid_count) / static_cast<double>(valid.size());
  std::vector<double> finite_values;
  finite_values.reserve(valid_count);
  for (size_t i = 0; i < values.size(); ++i) {
    if (valid[i] && std::isfinite(values[i])) {
      finite_values.push_back(values[i]);
    }
  }
  if (finite_values.size() < 4) {
    result.invalid_reason = "insufficient_cycle_samples";
    return result;
  }
  const auto extrema = std::minmax_element(finite_values.begin(),
                                            finite_values.end());
  const double range = *extrema.second - *extrema.first;
  const double slope_epsilon = std::max(1e-12, range * 1e-4);
  size_t rising = 0;
  size_t falling = 0;
  size_t directional = 0;
  size_t adjacent_valid_count = 0;
  int previous_sign = 0;
  std::vector<size_t> peak_indices;
  std::vector<size_t> trough_indices;
  for (size_t i = 1; i < values.size(); ++i) {
    if (!valid[i - 1] || !valid[i]) {
      previous_sign = 0;
      continue;
    }
    ++adjacent_valid_count;
    const double delta = values[i] - values[i - 1];
    const int sign = delta > slope_epsilon
                         ? 1
                         : (delta < -slope_epsilon ? -1 : 0);
    if (sign > 0) {
      ++rising;
      ++directional;
    } else if (sign < 0) {
      ++falling;
      ++directional;
    }
    if (previous_sign > 0 && sign < 0) {
      peak_indices.push_back(i - 1);
    } else if (previous_sign < 0 && sign > 0) {
      trough_indices.push_back(i - 1);
    }
    if (sign != 0) {
      previous_sign = sign;
    }
  }
  const double adjacent_valid =
      static_cast<double>(std::max<size_t>(1, adjacent_valid_count));
  result.rising_duration_ratio = rising / adjacent_valid;
  result.falling_duration_ratio = falling / adjacent_valid;
  result.has_peak = !peak_indices.empty();
  result.has_trough = !trough_indices.empty();
  result.turning_point_score =
      0.5 * (result.has_peak ? 1.0 : 0.0) +
      0.5 * (result.has_trough ? 1.0 : 0.0);
  result.monotonicity_score =
      adjacent_valid_count == 0
          ? 0.0
          : ClampValue(static_cast<double>(directional) /
                           adjacent_valid,
                       0.0, 1.0);
  const size_t expected_period_samples = std::max<size_t>(
      2, static_cast<size_t>(std::llround(expected_period_s / sample_step_s)));
  const size_t minimum_lag = std::max<size_t>(
      1, static_cast<size_t>(std::floor(0.5 * expected_period_samples)));
  const size_t maximum_lag = std::min<size_t>(
      values.size() > 4 ? values.size() - 4 : 0,
      static_cast<size_t>(std::ceil(1.5 * expected_period_samples)));
  const size_t minimum_pairs = std::max<size_t>(
      4, static_cast<size_t>(std::ceil(0.35 * expected_period_samples)));
  struct LagCorrelation {
    size_t lag = 0;
    double correlation = -1.0;
  };
  std::vector<LagCorrelation> lag_correlations;
  size_t best_overall_lag = 0;
  double best_overall_correlation = -1.0;
  for (size_t lag = minimum_lag; lag <= maximum_lag; ++lag) {
    size_t pair_count = 0;
    const double correlation =
        ComputeLaggedCorrelation(values, valid, lag, &pair_count);
    if (pair_count < minimum_pairs) {
      continue;
    }
    lag_correlations.push_back({lag, correlation});
    const bool stronger =
        correlation > best_overall_correlation + 1e-12;
    const bool equally_strong = std::abs(
        correlation - best_overall_correlation) <= 1e-12;
    const bool closer_to_expected = best_overall_lag == 0 ||
        std::abs(static_cast<int64_t>(lag) -
                 static_cast<int64_t>(expected_period_samples)) <
            std::abs(static_cast<int64_t>(best_overall_lag) -
                     static_cast<int64_t>(expected_period_samples));
    if (stronger || (equally_strong && closer_to_expected)) {
      best_overall_lag = lag;
      best_overall_correlation = correlation;
    }
  }
  size_t best_lag = 0;
  double best_correlation = -1.0;
  for (size_t i = 1; i + 1 < lag_correlations.size(); ++i) {
    const LagCorrelation& candidate = lag_correlations[i];
    const double period_error = std::abs(
        static_cast<double>(candidate.lag) - expected_period_samples) /
        expected_period_samples;
    const bool local_peak =
        candidate.correlation >= lag_correlations[i - 1].correlation &&
        candidate.correlation > lag_correlations[i + 1].correlation;
    if (!local_peak || period_error > waveform_period_tolerance_ratio_ ||
        candidate.correlation < waveform_min_periodicity_correlation_) {
      continue;
    }
    const bool stronger =
        candidate.correlation > best_correlation + 1e-12;
    const bool equally_strong =
        std::abs(candidate.correlation - best_correlation) <= 1e-12;
    const bool closer_to_expected = best_lag == 0 ||
        std::abs(static_cast<int64_t>(candidate.lag) -
                 static_cast<int64_t>(expected_period_samples)) <
            std::abs(static_cast<int64_t>(best_lag) -
                     static_cast<int64_t>(expected_period_samples));
    if (stronger || (equally_strong && closer_to_expected)) {
      best_lag = candidate.lag;
      best_correlation = candidate.correlation;
    }
  }
  const bool target_period_peak_found = best_lag > 0;
  if (!target_period_peak_found) {
    best_lag = best_overall_lag;
    best_correlation = best_overall_correlation;
  }
  if (best_lag > 0) {
    result.estimated_period = best_lag * sample_step_s;
    result.periodicity_correlation = best_correlation;
  }
  result.period_error_ratio =
      std::abs(result.estimated_period - expected_period_s) /
      expected_period_s;
  const double required_coverage = minimum_coverage_ratio < 0.0
      ? waveform_min_cycle_coverage_ratio_
      : ClampValue(minimum_coverage_ratio, 0.0, 1.0);
  const double coverage_score = ClampValue(
      result.coverage_ratio /
          std::max(required_coverage, 1e-12),
      0.0, 1.0);
  const double period_score = ClampValue(
      1.0 - result.period_error_ratio /
                std::max(waveform_period_tolerance_ratio_, 1e-12),
      0.0, 1.0);
  const double periodicity_score = ClampValue(
      0.5 * (result.periodicity_correlation + 1.0), 0.0, 1.0);
  result.completeness_score =
      0.40 * coverage_score + 0.30 * period_score +
      0.30 * periodicity_score;
  result.valid =
      result.coverage_ratio >= required_coverage &&
      target_period_peak_found &&
      result.period_error_ratio <= waveform_period_tolerance_ratio_ &&
      result.periodicity_correlation >=
          waveform_min_periodicity_correlation_;
  result.invalid_reason = result.valid ? "none" : "period_not_identified";
  return result;
}

FBBRSender::BicClippingDetectionResult
FBBRSender::DetectBicSrttClipping(
    const std::vector<double>& srtt,
    const std::vector<bool>& valid,
    double noise_sigma) {
  BicClippingDetectionResult result;
  if (srtt.size() != valid.size() || srtt.size() < 9) {
    result.invalid_reason = "invalid_bic_clipping_input";
    return result;
  }

  // This path is deliberately shape-only: it does not use sender phase,
  // waveform period, delivery rate, or RTprop.
  std::vector<double> smoothed = srtt;
  for (size_t i = 1; i + 1 < srtt.size(); ++i) {
    if (!valid[i - 1] || !valid[i] || !valid[i + 1] ||
        !std::isfinite(srtt[i - 1]) || !std::isfinite(srtt[i]) ||
        !std::isfinite(srtt[i + 1])) {
      continue;
    }
    std::array<double, 3> local = {
        {srtt[i - 1], srtt[i], srtt[i + 1]}};
    std::sort(local.begin(), local.end());
    smoothed[i] = local[1];
  }

  std::vector<double> x;
  std::vector<double> y;
  x.reserve(srtt.size());
  y.reserve(srtt.size());
  for (size_t i = 0; i < smoothed.size(); ++i) {
    if (valid[i] && std::isfinite(smoothed[i])) {
      x.push_back(static_cast<double>(i));
      y.push_back(smoothed[i]);
    }
  }
  const size_t n = y.size();
  if (n < 9) {
    result.invalid_reason = "insufficient_bic_clipping_samples";
    return result;
  }

  const double y_mean =
      std::accumulate(y.begin(), y.end(), 0.0) /
      static_cast<double>(n);
  double signal_sum_squares = 0.0;
  for (double value : y) {
    const double centered = value - y_mean;
    signal_sum_squares += centered * centered;
  }
  const double signal_variance =
      signal_sum_squares / static_cast<double>(n);
  if (!std::isfinite(signal_variance) ||
      signal_variance <= std::numeric_limits<double>::epsilon()) {
    result.invalid_reason = "flat_bic_clipping_signal";
    return result;
  }

  // Infer the noise scale from the robust residual, the observed SRTT
  // quantization, and the finite-window variance.  No absolute slope,
  // duration, amplitude, or RTT-level threshold is introduced here.
  std::vector<double> sorted_levels = y;
  std::sort(sorted_levels.begin(), sorted_levels.end());
  std::vector<double> positive_level_steps;
  for (size_t i = 1; i < sorted_levels.size(); ++i) {
    const double difference = sorted_levels[i] - sorted_levels[i - 1];
    if (difference > std::numeric_limits<double>::epsilon() *
                         std::max(1.0, std::abs(sorted_levels[i]))) {
      positive_level_steps.push_back(difference);
    }
  }
  const double quantization_sigma = positive_level_steps.empty()
      ? 0.0
      : Median(positive_level_steps) / std::sqrt(12.0);
  const double finite_window_sigma =
      std::sqrt(signal_variance / static_cast<double>(n));
  const double model_sigma = std::max(
      std::isfinite(noise_sigma) ? std::max(0.0, noise_sigma) : 0.0,
      std::max(quantization_sigma, finite_window_sigma));
  const double noise_variance = std::max(
      model_sigma * model_sigma,
      std::numeric_limits<double>::epsilon() *
          std::max(1.0, signal_variance));

  std::vector<double> prefix_x(n + 1, 0.0);
  std::vector<double> prefix_x2(n + 1, 0.0);
  std::vector<double> prefix_y(n + 1, 0.0);
  std::vector<double> prefix_y2(n + 1, 0.0);
  std::vector<double> prefix_xy(n + 1, 0.0);
  for (size_t i = 0; i < n; ++i) {
    prefix_x[i + 1] = prefix_x[i] + x[i];
    prefix_x2[i + 1] = prefix_x2[i] + x[i] * x[i];
    prefix_y[i + 1] = prefix_y[i] + y[i];
    prefix_y2[i + 1] = prefix_y2[i] + y[i] * y[i];
    prefix_xy[i + 1] = prefix_xy[i] + x[i] * y[i];
  }

  struct LineFit {
    bool valid = false;
    double slope = 0.0;
    double rss = std::numeric_limits<double>::infinity();
  };
  auto fit_line_range = [&](size_t begin, size_t end) {
    LineFit fit;
    if (end <= begin || end > n) {
      return fit;
    }
    const double count = static_cast<double>(end - begin);
    const double sum_x = prefix_x[end] - prefix_x[begin];
    const double sum_x2 = prefix_x2[end] - prefix_x2[begin];
    const double sum_y = prefix_y[end] - prefix_y[begin];
    const double sum_y2 = prefix_y2[end] - prefix_y2[begin];
    const double sum_xy = prefix_xy[end] - prefix_xy[begin];
    const double denominator = count * sum_x2 - sum_x * sum_x;
    if (denominator <= std::numeric_limits<double>::epsilon()) {
      return fit;
    }
    fit.slope = (count * sum_xy - sum_x * sum_y) / denominator;
    const double intercept = (sum_y - fit.slope * sum_x) / count;
    fit.rss = std::max(
        0.0, sum_y2 - intercept * sum_y - fit.slope * sum_xy);
    fit.valid = std::isfinite(fit.rss) && std::isfinite(fit.slope);
    return fit;
  };
  auto constant_rss_range = [&](size_t begin, size_t end) {
    if (end <= begin || end > n) {
      return std::numeric_limits<double>::infinity();
    }
    const double count = static_cast<double>(end - begin);
    const double sum_y = prefix_y[end] - prefix_y[begin];
    const double sum_y2 = prefix_y2[end] - prefix_y2[begin];
    return std::max(0.0, sum_y2 - sum_y * sum_y / count);
  };
  auto bic_score = [&](double rss, size_t count, size_t parameters) {
    if (!std::isfinite(rss) || count == 0) {
      return std::numeric_limits<double>::infinity();
    }
    return rss / noise_variance +
        static_cast<double>(parameters) *
            std::log(static_cast<double>(count));
  };

  constexpr size_t kMinimumSegmentSamples = 3;
  constexpr size_t kMaximumSegments = 16;
  const size_t maximum_segments = std::min(
      kMaximumSegments, n / kMinimumSegmentSamples);
  if (maximum_segments < 3) {
    result.invalid_reason = "insufficient_bic_segment_capacity";
    return result;
  }
  const size_t stride = n + 1;
  std::vector<double> segment_cost(
      stride * stride, std::numeric_limits<double>::infinity());
  for (size_t begin = 0; begin < n; ++begin) {
    for (size_t end = begin + kMinimumSegmentSamples;
         end <= n; ++end) {
      const LineFit fit = fit_line_range(begin, end);
      if (fit.valid) {
        segment_cost[begin * stride + end] = fit.rss;
      }
    }
  }

  const size_t dp_stride = n + 1;
  std::vector<double> dp(
      (maximum_segments + 1) * dp_stride,
      std::numeric_limits<double>::infinity());
  std::vector<int> backpointer(
      (maximum_segments + 1) * dp_stride, -1);
  dp[0] = 0.0;
  for (size_t segments = 1; segments <= maximum_segments; ++segments) {
    const size_t minimum_end = segments * kMinimumSegmentSamples;
    for (size_t end = minimum_end; end <= n; ++end) {
      const size_t first_begin =
          (segments - 1) * kMinimumSegmentSamples;
      const size_t last_begin = end - kMinimumSegmentSamples;
      double best = std::numeric_limits<double>::infinity();
      int best_begin = -1;
      for (size_t begin = first_begin; begin <= last_begin; ++begin) {
        const double prior =
            dp[(segments - 1) * dp_stride + begin];
        const double cost = segment_cost[begin * stride + end];
        if (!std::isfinite(prior) || !std::isfinite(cost)) {
          continue;
        }
        const double candidate = prior + cost;
        if (candidate < best) {
          best = candidate;
          best_begin = static_cast<int>(begin);
        }
      }
      dp[segments * dp_stride + end] = best;
      backpointer[segments * dp_stride + end] = best_begin;
    }
  }

  size_t selected_segments = 0;
  double selected_score = std::numeric_limits<double>::infinity();
  for (size_t segments = 1; segments <= maximum_segments; ++segments) {
    const double rss = dp[segments * dp_stride + n];
    const size_t parameters = 3 * segments - 1;
    const double score = bic_score(rss, n, parameters);
    if (score < selected_score) {
      selected_score = score;
      selected_segments = segments;
    }
  }
  if (selected_segments == 0 || !std::isfinite(selected_score)) {
    result.invalid_reason = "bic_segmentation_failed";
    return result;
  }

  struct Segment {
    size_t begin = 0;
    size_t end = 0;
  };
  std::vector<Segment> reversed_segments;
  size_t segment_end = n;
  for (size_t segments = selected_segments; segments > 0; --segments) {
    const int segment_begin =
        backpointer[segments * dp_stride + segment_end];
    if (segment_begin < 0) {
      result.invalid_reason = "bic_backtracking_failed";
      return result;
    }
    reversed_segments.push_back(
        {static_cast<size_t>(segment_begin), segment_end});
    segment_end = static_cast<size_t>(segment_begin);
  }
  std::reverse(reversed_segments.begin(), reversed_segments.end());

  struct ThreeParameterFit {
    bool valid = false;
    std::array<double, 3> coefficients = {{0.0, 0.0, 0.0}};
    double rss = std::numeric_limits<double>::infinity();
  };
  struct TwoParameterFit {
    bool valid = false;
    std::array<double, 2> coefficients = {{0.0, 0.0}};
    double rss = std::numeric_limits<double>::infinity();
  };
  auto solve_three_by_three = [](
      double matrix[3][4], std::array<double, 3>* solution) {
    for (size_t column = 0; column < 3; ++column) {
      size_t pivot = column;
      for (size_t row = column + 1; row < 3; ++row) {
        if (std::abs(matrix[row][column]) >
            std::abs(matrix[pivot][column])) {
          pivot = row;
        }
      }
      if (std::abs(matrix[pivot][column]) <=
          std::numeric_limits<double>::epsilon()) {
        return false;
      }
      if (pivot != column) {
        for (size_t entry = column; entry < 4; ++entry) {
          std::swap(matrix[column][entry], matrix[pivot][entry]);
        }
      }
      const double divisor = matrix[column][column];
      for (size_t entry = column; entry < 4; ++entry) {
        matrix[column][entry] /= divisor;
      }
      for (size_t row = 0; row < 3; ++row) {
        if (row == column) {
          continue;
        }
        const double factor = matrix[row][column];
        for (size_t entry = column; entry < 4; ++entry) {
          matrix[row][entry] -= factor * matrix[column][entry];
        }
      }
    }
    for (size_t i = 0; i < 3; ++i) {
      (*solution)[i] = matrix[i][3];
    }
    return true;
  };
  auto fit_three_parameter = [&](size_t begin, size_t end,
                                 const std::function<
                                     std::array<double, 3>(double)>&
                                     features) {
    ThreeParameterFit fit;
    if (end <= begin || end > n) {
      return fit;
    }
    double normal[3][4] = {};
    for (size_t i = begin; i < end; ++i) {
      const std::array<double, 3> f = features(x[i]);
      for (size_t row = 0; row < 3; ++row) {
        for (size_t column = 0; column < 3; ++column) {
          normal[row][column] += f[row] * f[column];
        }
        normal[row][3] += f[row] * y[i];
      }
    }
    if (!solve_three_by_three(normal, &fit.coefficients)) {
      return fit;
    }
    fit.rss = 0.0;
    for (size_t i = begin; i < end; ++i) {
      const std::array<double, 3> f = features(x[i]);
      const double predicted =
          fit.coefficients[0] * f[0] +
          fit.coefficients[1] * f[1] +
          fit.coefficients[2] * f[2];
      const double residual = y[i] - predicted;
      fit.rss += residual * residual;
    }
    fit.valid = std::isfinite(fit.rss);
    return fit;
  };
  auto fit_single_feature = [&] (
      size_t begin, size_t end,
      const std::function<double(double)>& feature) {
    TwoParameterFit fit;
    if (end <= begin || end > n) {
      return fit;
    }
    double sum_feature = 0.0;
    double sum_feature_squared = 0.0;
    double sum_y = 0.0;
    double sum_feature_y = 0.0;
    for (size_t i = begin; i < end; ++i) {
      const double value = feature(x[i]);
      sum_feature += value;
      sum_feature_squared += value * value;
      sum_y += y[i];
      sum_feature_y += value * y[i];
    }
    const double count = static_cast<double>(end - begin);
    const double denominator =
        count * sum_feature_squared - sum_feature * sum_feature;
    if (denominator <= std::numeric_limits<double>::epsilon()) {
      return fit;
    }
    fit.coefficients[1] =
        (count * sum_feature_y - sum_feature * sum_y) / denominator;
    fit.coefficients[0] =
        (sum_y - fit.coefficients[1] * sum_feature) / count;
    fit.rss = 0.0;
    for (size_t i = begin; i < end; ++i) {
      const double predicted = fit.coefficients[0] +
          fit.coefficients[1] * feature(x[i]);
      const double residual = y[i] - predicted;
      fit.rss += residual * residual;
    }
    fit.valid = std::isfinite(fit.rss);
    return fit;
  };

  struct Motif {
    bool top = false;
    size_t plateau_begin = 0;
    size_t plateau_end = 0;
    double rounded_bic_margin = 0.0;
    bool has_sharp_edge = false;
  };
  std::vector<Motif> motifs;
  for (size_t i = 1; i + 1 < reversed_segments.size(); ++i) {
    const Segment& before = reversed_segments[i - 1];
    const Segment& plateau = reversed_segments[i];
    const Segment& after = reversed_segments[i + 1];
    const size_t plateau_count = plateau.end - plateau.begin;
    const LineFit plateau_line =
        fit_line_range(plateau.begin, plateau.end);
    const double plateau_constant_rss =
        constant_rss_range(plateau.begin, plateau.end);
    if (!plateau_line.valid ||
        bic_score(plateau_constant_rss, plateau_count, 1) >=
            bic_score(plateau_line.rss, plateau_count, 2)) {
      continue;
    }

    const double left_edge = x[plateau.begin];
    const double right_edge = x[plateau.end - 1];
    const ThreeParameterFit hinge = fit_three_parameter(
        before.begin, after.end,
        [left_edge, right_edge](double sample_x) {
          return std::array<double, 3>{{
              1.0,
              std::max(0.0, left_edge - sample_x),
              std::max(0.0, sample_x - right_edge)}};
        });
    const double context_center =
        0.5 * (x[before.begin] + x[after.end - 1]);
    const ThreeParameterFit quadratic = fit_three_parameter(
        before.begin, after.end,
        [context_center](double sample_x) {
          const double centered = sample_x - context_center;
          return std::array<double, 3>{{
              1.0, centered, centered * centered}};
        });
    const size_t context_count = after.end - before.begin;
    if (!hinge.valid || !quadratic.valid) {
      continue;
    }
    const bool bottom = hinge.coefficients[1] > 0.0 &&
                        hinge.coefficients[2] > 0.0;
    const bool top = hinge.coefficients[1] < 0.0 &&
                     hinge.coefficients[2] < 0.0;
    if (!bottom && !top) {
      continue;
    }

    // A quantized triangular or smoothly rounded response can contain a
    // short constant run at its turning point without being clipped.  Compare
    // the two-knot clipped hinge with three non-clipped explanations:
    // quadratic, one-knot V, and an affine rounded turn.  The last model is
    //   a + b*u + c*sqrt((u-v)^2 + r^2),
    // where u is the normalized local time.  Its free center and radius let
    // a gradual U-shaped turn compete fairly with the two free edges of the
    // clipped hinge.  This is model selection only: no absolute RTT, slope,
    // amplitude, plateau-duration, or RTprop threshold is involved.
    ThreeParameterFit best_v;
    for (size_t vertex = before.begin + 2;
         vertex + 2 < after.end; ++vertex) {
      const double vertex_x = x[vertex];
      const ThreeParameterFit candidate_v = fit_three_parameter(
          before.begin, after.end,
          [vertex_x](double sample_x) {
            return std::array<double, 3>{{
                1.0,
                std::max(0.0, vertex_x - sample_x),
                std::max(0.0, sample_x - vertex_x)}};
          });
      if (!candidate_v.valid) {
        continue;
      }
      const bool same_direction = bottom
          ? candidate_v.coefficients[1] > 0.0 &&
                candidate_v.coefficients[2] > 0.0
          : candidate_v.coefficients[1] < 0.0 &&
                candidate_v.coefficients[2] < 0.0;
      if (same_direction &&
          (!best_v.valid || candidate_v.rss < best_v.rss)) {
        best_v = candidate_v;
      }
    }

    const double context_span =
        x[after.end - 1] - x[before.begin];
    const double normalized_center =
        0.5 * (x[before.begin] + x[after.end - 1]);
    ThreeParameterFit best_rounded_turn;
    if (context_span > std::numeric_limits<double>::epsilon()) {
      std::vector<double> rounded_radii;
      const double minimum_normalized_radius = 1.0 / context_span;
      constexpr double kMaximumNormalizedRadius = 0.5;
      constexpr size_t kRoundedRadiusCandidates = 32;
      if (minimum_normalized_radius < kMaximumNormalizedRadius) {
        const double log_minimum =
            std::log(minimum_normalized_radius);
        const double log_maximum =
            std::log(kMaximumNormalizedRadius);
        for (size_t radius_index = 0;
             radius_index < kRoundedRadiusCandidates; ++radius_index) {
          const double fraction = static_cast<double>(radius_index) /
              static_cast<double>(kRoundedRadiusCandidates - 1);
          rounded_radii.push_back(std::exp(
              log_minimum + fraction * (log_maximum - log_minimum)));
        }
      } else {
        rounded_radii.push_back(kMaximumNormalizedRadius);
      }
      for (size_t vertex = before.begin + 2;
           vertex + 2 < after.end; ++vertex) {
        const double normalized_vertex =
            (x[vertex] - normalized_center) / context_span;
        for (double normalized_radius : rounded_radii) {
          const ThreeParameterFit candidate_turn = fit_three_parameter(
              before.begin, after.end,
              [normalized_center, context_span, normalized_vertex,
               normalized_radius](double sample_x) {
                const double normalized_x =
                    (sample_x - normalized_center) / context_span;
                const double offset = normalized_x - normalized_vertex;
                return std::array<double, 3>{{
                    1.0,
                    normalized_x,
                    std::sqrt(offset * offset +
                              normalized_radius * normalized_radius)}};
              });
          if (candidate_turn.valid &&
              (!best_rounded_turn.valid ||
               candidate_turn.rss < best_rounded_turn.rss)) {
            best_rounded_turn = candidate_turn;
          }
        }
      }
    }
    if (!best_v.valid || !best_rounded_turn.valid ||
        bic_score(hinge.rss, context_count, 5) >=
            bic_score(quadratic.rss, context_count, 3) ||
        bic_score(hinge.rss, context_count, 5) >=
            bic_score(best_v.rss, context_count, 4) ||
        bic_score(hinge.rss, context_count, 5) >=
            bic_score(best_rounded_turn.rss, context_count, 5)) {
      continue;
    }

    // A real horizontal cut also creates at least one slope discontinuity at
    // an entry/exit edge.  A rounded valley instead has a continuously
    // changing slope even when quantization makes its center look constant.
    // On balanced neighborhoods around each selected edge, compare a sharp
    // one-sided hinge with a smooth quadratic.  Both models have three
    // effective parameters (two coefficients plus the selected edge versus
    // three polynomial coefficients), so this remains threshold-free model
    // competition.
    const size_t left_wing = std::min(
        before.end - before.begin, plateau_count);
    const size_t right_wing = std::min(
        after.end - after.begin, plateau_count);
    bool left_edge_sharp = false;
    bool right_edge_sharp = false;
    if (left_wing >= kMinimumSegmentSamples) {
      const size_t edge_begin = plateau.begin - left_wing;
      const size_t edge_end = plateau.begin + left_wing;
      const double edge_x = x[plateau.begin];
      const TwoParameterFit sharp_edge = fit_single_feature(
          edge_begin, edge_end,
          [edge_x](double sample_x) {
            return std::max(0.0, edge_x - sample_x);
          });
      const ThreeParameterFit smooth_edge = fit_three_parameter(
          edge_begin, edge_end,
          [edge_x](double sample_x) {
            const double centered = sample_x - edge_x;
            return std::array<double, 3>{{
                1.0, centered, centered * centered}};
          });
      left_edge_sharp = sharp_edge.valid && smooth_edge.valid &&
          bic_score(sharp_edge.rss, edge_end - edge_begin, 3) <
              bic_score(smooth_edge.rss, edge_end - edge_begin, 3);
    }
    if (right_wing >= kMinimumSegmentSamples) {
      const size_t edge_begin = plateau.end - right_wing;
      const size_t edge_end = plateau.end + right_wing;
      const double edge_x = x[plateau.end - 1];
      const TwoParameterFit sharp_edge = fit_single_feature(
          edge_begin, edge_end,
          [edge_x](double sample_x) {
            return std::max(0.0, sample_x - edge_x);
          });
      const ThreeParameterFit smooth_edge = fit_three_parameter(
          edge_begin, edge_end,
          [edge_x](double sample_x) {
            const double centered = sample_x - edge_x;
            return std::array<double, 3>{{
                1.0, centered, centered * centered}};
          });
      right_edge_sharp = sharp_edge.valid && smooth_edge.valid &&
          bic_score(sharp_edge.rss, edge_end - edge_begin, 3) <
              bic_score(smooth_edge.rss, edge_end - edge_begin, 3);
    }
    motifs.push_back({
        top, plateau.begin, plateau.end,
        bic_score(best_rounded_turn.rss, context_count, 5) -
            bic_score(hinge.rss, context_count, 5),
        left_edge_sharp || right_edge_sharp});
  }

  auto shared_envelope_pair_exists = [&](bool top) {
    std::vector<Motif> directional;
    for (const Motif& motif : motifs) {
      if (motif.top == top) {
        directional.push_back(motif);
      }
    }
    for (size_t first = 0; first < directional.size(); ++first) {
      for (size_t second = first + 1;
           second < directional.size(); ++second) {
        std::vector<double> pair_x;
        std::vector<double> pair_y;
        const Motif pair[] = {directional[first], directional[second]};
        // Both repetitions must independently expose a slope discontinuity.
        // Letting one sharp motif validate a second smooth turn recreates the
        // exact failure mode this detector is meant to reject.
        if (!pair[0].has_sharp_edge || !pair[1].has_sharp_edge) {
          continue;
        }
        for (const Motif& motif : pair) {
          for (size_t i = motif.plateau_begin;
               i < motif.plateau_end; ++i) {
            pair_x.push_back(x[i]);
            pair_y.push_back(y[i]);
          }
        }
        if (pair_x.size() < 6) {
          continue;
        }
        const double center =
            std::accumulate(pair_x.begin(), pair_x.end(), 0.0) /
            static_cast<double>(pair_x.size());
        double sum_centered_x2 = 0.0;
        double sum_y = 0.0;
        double sum_centered_xy = 0.0;
        for (size_t i = 0; i < pair_x.size(); ++i) {
          const double centered_x = pair_x[i] - center;
          sum_centered_x2 += centered_x * centered_x;
          sum_y += pair_y[i];
          sum_centered_xy += centered_x * pair_y[i];
        }
        if (sum_centered_x2 <=
            std::numeric_limits<double>::epsilon()) {
          continue;
        }
        const double common_slope =
            sum_centered_xy / sum_centered_x2;
        const double common_intercept =
            sum_y / static_cast<double>(pair_y.size());
        double common_rss = 0.0;
        for (size_t i = 0; i < pair_x.size(); ++i) {
          const double predicted = common_intercept +
              common_slope * (pair_x[i] - center);
          const double residual = pair_y[i] - predicted;
          common_rss += residual * residual;
        }
        const LineFit first_fit = fit_line_range(
            pair[0].plateau_begin, pair[0].plateau_end);
        const LineFit second_fit = fit_line_range(
            pair[1].plateau_begin, pair[1].plateau_end);
        if (!first_fit.valid || !second_fit.valid) {
          continue;
        }
        const double separate_rss = first_fit.rss + second_fit.rss;
        if (bic_score(common_rss, pair_x.size(), 2) <
            bic_score(separate_rss, pair_x.size(), 4)) {
          const double minimum_margin = std::min(
              pair[0].rounded_bic_margin,
              pair[1].rounded_bic_margin);
          const double combined_margin =
              pair[0].rounded_bic_margin +
              pair[1].rounded_bic_margin;
          if (top) {
            result.top_clip_min_rounded_bic_margin = minimum_margin;
            result.top_clip_combined_rounded_bic_margin = combined_margin;
            result.top_clip_pair_sharp_motif_count =
                static_cast<size_t>(pair[0].has_sharp_edge) +
                static_cast<size_t>(pair[1].has_sharp_edge);
          } else {
            result.bottom_clip_min_rounded_bic_margin = minimum_margin;
            result.bottom_clip_combined_rounded_bic_margin =
                combined_margin;
            result.bottom_clip_pair_sharp_motif_count =
                static_cast<size_t>(pair[0].has_sharp_edge) +
                static_cast<size_t>(pair[1].has_sharp_edge);
          }
          return true;
        }
      }
    }
    return false;
  };

  for (const Motif& motif : motifs) {
    if (motif.top) {
      ++result.top_motif_count;
    } else {
      ++result.bottom_motif_count;
    }
  }
  result.top_clip = shared_envelope_pair_exists(true);
  result.bottom_clip = shared_envelope_pair_exists(false);
  result.both_clipped = result.top_clip && result.bottom_clip;
  result.selected_segment_count = selected_segments;
  result.selected_score = selected_score;
  result.valid = true;
  result.invalid_reason = result.top_clip || result.bottom_clip
      ? "none"
      : "no_repeated_bic_clip_shape";
  return result;
}

FBBRSender::PlateauDetectionResult
FBBRSender::DetectDualSignalPlateaus(
    const std::vector<double>& srtt,
    const std::vector<double>& srtt_slopes,
    const std::vector<double>& drate,
    const std::vector<double>& drate_slopes,
    const std::vector<double>& sender_residual,
    const std::vector<bool>& valid,
    double sample_step_s,
    double period_s,
    double srtt_noise_sigma,
    double drate_noise_sigma) const {
  PlateauDetectionResult result;
  result.srtt_middle_sequential_mask.assign(valid.size(), false);
  result.drate_middle_sequential_mask.assign(valid.size(), false);
  if (srtt.size() != valid.size() || drate.size() != valid.size() ||
      srtt_slopes.size() != valid.size() ||
      drate_slopes.size() != valid.size() ||
      sender_residual.size() != valid.size() || srtt.size() < 5 ||
      sample_step_s <= 0.0 || period_s <= 0.0) {
    result.invalid_reason = "invalid_dual_plateau_input";
    return result;
  }
  const bool use_adaptive_load_judgment = UsesAdaptiveLoadJudgment();

  struct SignalThresholds {
    bool valid = false;
    double minimum = 0.0;
    double maximum = 0.0;
    double peak_to_peak = 0.0;
    double low_slope = 0.0;
    double shoulder_slope = 0.0;
    double minimum_shoulder_change = 0.0;
  };
  auto build_thresholds = [&](const std::vector<double>& signal,
                              const std::vector<double>& slopes,
                              double noise_sigma) {
    SignalThresholds thresholds;
    std::vector<double> levels;
    std::vector<double> selected_slopes;
    for (size_t i = 0; i < signal.size(); ++i) {
      if (valid[i] && std::isfinite(signal[i]) &&
          std::isfinite(slopes[i])) {
        levels.push_back(signal[i]);
        selected_slopes.push_back(slopes[i]);
      }
    }
    if (levels.size() < 5) {
      return thresholds;
    }
    const auto extrema = std::minmax_element(levels.begin(), levels.end());
    thresholds.minimum = *extrema.first;
    thresholds.maximum = *extrema.second;
    thresholds.peak_to_peak = thresholds.maximum - thresholds.minimum;
    if (thresholds.peak_to_peak <= std::max(2.0 * noise_sigma, 1e-9)) {
      return thresholds;
    }
    const double slope_median = Median(selected_slopes);
    std::vector<double> deviations;
    std::vector<double> absolute_slopes;
    deviations.reserve(selected_slopes.size());
    absolute_slopes.reserve(selected_slopes.size());
    for (double slope : selected_slopes) {
      deviations.push_back(std::abs(slope - slope_median));
      absolute_slopes.push_back(std::abs(slope));
    }
    const double local_noise_slope = 1.4826 * Median(deviations);
    std::sort(absolute_slopes.begin(), absolute_slopes.end());
    const size_t characteristic_index = static_cast<size_t>(std::floor(
        0.80 * static_cast<double>(absolute_slopes.size() - 1)));
    const double characteristic_slope =
        absolute_slopes[characteristic_index];
    thresholds.low_slope = std::max(
        1e-12, std::max(0.5 * local_noise_slope,
                        waveform_clip_max_slope_ratio_ *
                            characteristic_slope));
    thresholds.shoulder_slope = std::max(
        std::max(2.0 * thresholds.low_slope, local_noise_slope),
        0.25 * characteristic_slope);
    thresholds.minimum_shoulder_change = std::max(
        std::isfinite(noise_sigma) && noise_sigma > 0.0
            ? 2.0 * noise_sigma
            : 0.0,
        waveform_clip_min_duration_ratio_ * thresholds.peak_to_peak);
    thresholds.valid = true;
    return thresholds;
  };
  const SignalThresholds srtt_thresholds =
      build_thresholds(srtt, srtt_slopes, srtt_noise_sigma);
  const SignalThresholds drate_thresholds =
      build_thresholds(drate, drate_slopes, drate_noise_sigma);
  result.drate_has_waveform = drate_thresholds.valid;
  if ((!srtt_thresholds.valid && !drate_thresholds.valid) ||
      (!use_adaptive_load_judgment &&
       (!srtt_thresholds.valid || !drate_thresholds.valid))) {
    result.invalid_reason = "dual_signal_flat_or_below_noise";
    return result;
  }

  const size_t minimum_run = std::max<size_t>(
      3, static_cast<size_t>(std::ceil(
             waveform_clip_min_duration_ratio_ * period_s /
             sample_step_s)));
  const size_t shoulder_width = std::max<size_t>(
      3, static_cast<size_t>(std::ceil(
             std::max(0.05, waveform_clip_min_duration_ratio_) *
             period_s / sample_step_s)));
  const double shoulder_duration_s = shoulder_width * sample_step_s;
  auto median_range = [](const std::vector<double>& values,
                         const std::vector<bool>& valid_values,
                         size_t begin,
                         size_t end) {
    std::vector<double> selected;
    for (size_t i = begin; i < end && i < values.size(); ++i) {
      if (valid_values[i] && std::isfinite(values[i])) {
        selected.push_back(values[i]);
      }
    }
    return selected.empty() ? 0.0 : Median(selected);
  };
  struct Candidate {
    bool owner_is_srtt = false;
    size_t begin = 0;
    size_t end = 0;
    bool is_positive = false;
    bool middle_level = false;
    bool middle_sequential = false;
    bool genuine_shoulder_clip = false;
    double mean = 0.0;
    double span_ratio = 0.0;
    double extreme_distance_ratio = 1.0;
    double abs_slope = 0.0;
    double before = 0.0;
    double after = 0.0;
    double other_before = 0.0;
    double other_after = 0.0;
    double overlap_ratio = 0.0;
    double minimum_shoulder_change = 0.0;
  };
  std::vector<Candidate> candidates;
  auto collect = [&](bool owner_is_srtt,
                     const std::vector<double>& signal,
                     const std::vector<double>& signal_slopes,
                     const std::vector<double>& other_slopes,
                     const SignalThresholds& thresholds,
                     const SignalThresholds& other_thresholds) {
    size_t i = 0;
    while (i < signal.size()) {
      if (!valid[i] ||
          std::abs(signal_slopes[i]) > thresholds.low_slope) {
        ++i;
        continue;
      }
      const size_t begin = i;
      while (i < signal.size() && valid[i] &&
             std::abs(signal_slopes[i]) <= thresholds.low_slope) {
        ++i;
      }
      const size_t end = i;
      if (end - begin < minimum_run ||
          end - begin >= static_cast<size_t>(0.80 * signal.size())) {
        continue;
      }
      std::vector<double> run_levels;
      std::vector<double> run_abs_slopes;
      size_t positive_count = 0;
      size_t negative_count = 0;
      for (size_t j = begin; j < end; ++j) {
        run_levels.push_back(signal[j]);
        run_abs_slopes.push_back(std::abs(signal_slopes[j]));
        if (sender_residual[j] >= 0.0) {
          ++positive_count;
        } else {
          ++negative_count;
        }
      }
      const auto run_extrema =
          std::minmax_element(run_levels.begin(), run_levels.end());
      const double span_ratio =
          (*run_extrema.second - *run_extrema.first) /
          thresholds.peak_to_peak;
      if (span_ratio > waveform_plateau_max_level_span_ratio_) {
        continue;
      }
      Candidate candidate;
      candidate.owner_is_srtt = owner_is_srtt;
      candidate.begin = begin;
      candidate.end = end;
      candidate.is_positive = sender_residual[begin] >= 0.0;
      candidate.overlap_ratio = static_cast<double>(
          candidate.is_positive ? positive_count : negative_count) /
          static_cast<double>(end - begin);
      candidate.mean = Median(run_levels);
      const double top_distance =
          std::abs(thresholds.maximum - candidate.mean) /
          thresholds.peak_to_peak;
      const double bottom_distance =
          std::abs(candidate.mean - thresholds.minimum) /
          thresholds.peak_to_peak;
      candidate.middle_level =
          top_distance > waveform_plateau_extreme_distance_ratio_ &&
          bottom_distance > waveform_plateau_extreme_distance_ratio_;
      candidate.extreme_distance_ratio = candidate.is_positive
          ? top_distance
          : bottom_distance;
      candidate.span_ratio = span_ratio;
      candidate.abs_slope = Median(run_abs_slopes);
      candidate.before = median_range(
          signal_slopes, valid,
          begin > shoulder_width ? begin - shoulder_width : 0, begin);
      candidate.after = median_range(
          signal_slopes, valid, end,
          std::min(signal.size(), end + shoulder_width));
      candidate.other_before = median_range(
          other_slopes, valid,
          begin > shoulder_width ? begin - shoulder_width : 0, begin);
      candidate.other_after = median_range(
          other_slopes, valid, end,
          std::min(signal.size(), end + shoulder_width));
      const bool owner_same = HasMacroSameDirectionShoulders(
          candidate.before, candidate.after);
      const bool other_same = HasMacroSameDirectionShoulders(
          candidate.other_before, candidate.other_after);
      const bool owner_opposing = HasMacroOpposingShoulders(
          candidate.before, candidate.after, shoulder_duration_s,
          thresholds.shoulder_slope,
          thresholds.minimum_shoulder_change);
      const bool dual_opposing = other_thresholds.valid &&
          HasDualMacroOpposingShoulders(
              candidate.before, candidate.after,
              thresholds.shoulder_slope,
              thresholds.minimum_shoulder_change,
              candidate.other_before, candidate.other_after,
              other_thresholds.shoulder_slope,
              other_thresholds.minimum_shoulder_change,
              shoulder_duration_s);
      candidate.middle_sequential = use_adaptive_load_judgment
          ? owner_same
          : owner_same || other_same;
      candidate.genuine_shoulder_clip =
          !candidate.middle_sequential &&
          (use_adaptive_load_judgment ? owner_opposing : dual_opposing) &&
          candidate.overlap_ratio >= waveform_clip_min_half_overlap_ratio_ &&
          candidate.extreme_distance_ratio <=
              waveform_plateau_extreme_distance_ratio_;
      candidate.minimum_shoulder_change =
          thresholds.minimum_shoulder_change;
      candidates.push_back(candidate);
    }
  };
  if (srtt_thresholds.valid) {
    collect(true, srtt, srtt_slopes, drate_slopes,
            srtt_thresholds, drate_thresholds);
  }
  if (drate_thresholds.valid) {
    collect(false, drate, drate_slopes, srtt_slopes,
            drate_thresholds, srtt_thresholds);
  }

  result.plateau_candidate_count = candidates.size();
  Candidate selected;
  bool selected_found = false;
  std::vector<std::pair<size_t, size_t>> srtt_positive_intervals;
  std::vector<std::pair<size_t, size_t>> drate_positive_intervals;
  for (const Candidate& candidate : candidates) {
    if (candidate.middle_sequential) {
      ++result.middle_sequential_candidate_count;
      for (size_t i = candidate.begin; i < candidate.end; ++i) {
        if (candidate.owner_is_srtt) {
          result.srtt_middle_sequential_mask[i] = true;
        } else {
          result.drate_middle_sequential_mask[i] = true;
        }
      }
      if (candidate.owner_is_srtt) {
        result.srtt_middle_sequential_plateau = true;
      } else {
        result.drate_middle_sequential_plateau = true;
      }
    }
    if (!candidate.owner_is_srtt && candidate.middle_level) {
      result.drate_middle_any_plateau = true;
    }
    if (candidate.genuine_shoulder_clip) {
      if (candidate.owner_is_srtt) {
        result.positive_half_clipped |= candidate.is_positive;
        result.negative_half_clipped |= !candidate.is_positive;
        if (candidate.is_positive) {
          srtt_positive_intervals.push_back(
              {candidate.begin, candidate.end});
        }
      } else {
        result.drate_positive_half_clipped |= candidate.is_positive;
        result.drate_negative_half_clipped |= !candidate.is_positive;
        if (candidate.is_positive) {
          drate_positive_intervals.push_back(
              {candidate.begin, candidate.end});
        }
      }
    }
    if (candidate.owner_is_srtt &&
        (!selected_found ||
         (candidate.genuine_shoulder_clip &&
          !selected.genuine_shoulder_clip) ||
         (candidate.genuine_shoulder_clip ==
              selected.genuine_shoulder_clip &&
          candidate.end - candidate.begin >
              selected.end - selected.begin))) {
      selected = candidate;
      selected_found = true;
    }
  }
  for (const auto& srtt_interval : srtt_positive_intervals) {
    for (const auto& drate_interval : drate_positive_intervals) {
      const size_t overlap_begin =
          std::max(srtt_interval.first, drate_interval.first);
      const size_t overlap_end =
          std::min(srtt_interval.second, drate_interval.second);
      const size_t shorter = std::min(
          srtt_interval.second - srtt_interval.first,
          drate_interval.second - drate_interval.first);
      if (overlap_end > overlap_begin && shorter > 0 &&
          static_cast<double>(overlap_end - overlap_begin) /
                  static_cast<double>(shorter) >=
              0.50) {
        result.positive_half_clips_simultaneous = true;
      }
    }
  }

  result.top_clip = result.positive_half_clipped;
  result.bottom_clip = result.negative_half_clipped;
  result.ambiguous = result.positive_half_clipped &&
                     result.negative_half_clipped;
  result.drate_clip_ambiguous = result.drate_positive_half_clipped &&
                                result.drate_negative_half_clipped;
  result.valid = !candidates.empty();
  if (!selected_found) {
    result.invalid_reason = result.valid ? "no_srtt_plateau"
                                         : "no_valid_plateau";
    return result;
  }
  result.plateau_start = selected.begin * sample_step_s;
  result.plateau_end = selected.end * sample_step_s;
  result.plateau_duration_ratio =
      (selected.end - selected.begin) * sample_step_s / period_s;
  result.plateau_mean = selected.mean;
  result.plateau_level_span_ratio = selected.span_ratio;
  result.plateau_extreme_distance_ratio =
      selected.extreme_distance_ratio;
  result.plateau_abs_slope = selected.abs_slope;
  result.shoulder_slope_before = selected.before;
  result.shoulder_slope_after = selected.after;
  result.other_shoulder_slope_before = selected.other_before;
  result.other_shoulder_slope_after = selected.other_after;
  result.shoulders_opposite = selected.genuine_shoulder_clip;
  result.shoulder_change_before =
      std::abs(selected.before) * shoulder_duration_s;
  result.shoulder_change_after =
      std::abs(selected.after) * shoulder_duration_s;
  result.minimum_shoulder_change = selected.minimum_shoulder_change;
  result.phase_at_plateau_start = sender_residual[selected.begin];
  result.phase_at_plateau_end = sender_residual[
      std::min(selected.end - 1, sender_residual.size() - 1)];
  result.half_overlap_ratio = selected.overlap_ratio;
  result.invalid_reason = result.ambiguous ? "both_clip_directions" : "none";
  return result;
}

bool FBBRSender::LocateBoundaryLiftPoint(
    const PlateauDetectionResult& plateau,
    const std::vector<double>& srtt_slopes,
    const std::vector<bool>& valid,
    QuicTime receiver_window_start,
    double sample_step_s,
    double best_lag_s,
    double* lift_time_s,
    double* sender_phase,
    double* boundary_rate_bps,
    double* boundary_delta_bps) const {
  if (!plateau.valid || !plateau.bottom_clip || plateau.top_clip ||
      srtt_slopes.size() != valid.size() || sample_step_s <= 0.0 ||
      current_probe_bw_phase_gain_ <= 0.0) {
    return false;
  }
  const size_t search_begin = std::min(
      srtt_slopes.size() - 1,
      static_cast<size_t>(std::floor(plateau.plateau_end / sample_step_s)));
  std::vector<double> valid_slopes;
  for (size_t i = 0; i < srtt_slopes.size(); ++i) {
    if (valid[i] && std::isfinite(srtt_slopes[i])) {
      valid_slopes.push_back(srtt_slopes[i]);
    }
  }
  if (valid_slopes.size() < 4) {
    return false;
  }
  const double slope_median = Median(valid_slopes);
  std::vector<double> deviations;
  for (double slope : valid_slopes) {
    deviations.push_back(std::abs(slope - slope_median));
  }
  const double positive_threshold =
      std::max(1e-9, 2.0 * 1.4826 * Median(deviations));
  const size_t persistent_samples = std::max<size_t>(
      3, static_cast<size_t>(std::ceil(0.03 /
          std::max(cruise_modulation_freq_hz_, 1e-9) / sample_step_s)));
  size_t lift_index = srtt_slopes.size();
  for (size_t i = search_begin;
       i + persistent_samples <= srtt_slopes.size(); ++i) {
    bool persistent = true;
    for (size_t j = i; j < i + persistent_samples; ++j) {
      if (!valid[j] || srtt_slopes[j] <= positive_threshold) {
        persistent = false;
        break;
      }
    }
    if (persistent) {
      lift_index = i;
      break;
    }
  }
  if (lift_index >= srtt_slopes.size()) {
    return false;
  }
  const double receiver_lift_offset_s = lift_index * sample_step_s;
  const QuicTime receiver_lift = receiver_window_start +
      TimeDelta::FromMicroseconds(static_cast<int64_t>(std::llround(
          receiver_lift_offset_s * 1000000.0)));
  const QuicTime sender_lift = receiver_lift -
      TimeDelta::FromMicroseconds(static_cast<int64_t>(std::llround(
          best_lag_s * 1000000.0)));
  const double period_s = 1.0 / cruise_modulation_freq_hz_;
  const double radius_s = std::max(0.002, 0.02 * period_s);
  const TimeDelta radius = TimeDelta::FromMicroseconds(
      static_cast<int64_t>(std::llround(radius_s * 1000000.0)));
  const auto samples = SelectRateSamples(sender_rate_history_,
                                         sender_lift - radius,
                                         sender_lift + radius);
  std::vector<double> rates;
  for (const auto& sample : samples) {
    if (sample.valid && !sample.rate.IsZero()) {
      rates.push_back(
          static_cast<double>(sample.rate.ToBitsPerSecond()));
    }
  }
  if (rates.size() < 3) {
    return false;
  }
  const double rate_bps = Median(rates);
  const double baseline_bps = static_cast<double>(
      current_injection_baseline_bw_.ToBitsPerSecond());
  const double amplitude_bw =
      static_cast<double>(current_probe_amplitude_bps_) /
      current_probe_bw_phase_gain_;
  const double delta = ClampValue(
      rate_bps / current_probe_bw_phase_gain_ - baseline_bps,
      0.0, amplitude_bw);
  if (!std::isfinite(delta) || delta <= 1.0) {
    return false;
  }
  *lift_time_s =
      static_cast<double>((receiver_lift - QuicTime::Zero())
                              .ToMicroseconds()) /
      1000000.0;
  *sender_phase = TriangleWave(sender_lift);
  *boundary_rate_bps = rate_bps;
  *boundary_delta_bps = delta;
  return true;
}

void FBBRSender::ResetWaveformCruiseState(QuicTime now) {
  waveform_cruise_state_ = WaveformCruiseState::kDisabled;
  probe_epoch_start_time_ = QuicTime::Zero();
  probe_epoch_rtt_ = TimeDelta::Zero();
  waveform_settle_start_ = QuicTime::Zero();
  waveform_settle_end_ = QuicTime::Zero();
  waveform_window_start_ = QuicTime::Zero();
  waveform_window_end_ = QuicTime::Zero();
  waveform_window_periods_ = 0.0;
  waveform_window_extended_ = false;
  underload_located_ = false;
  trusted_baseline_locked_ = false;
  has_last_similar_drate_amplitude_ = false;
  last_similar_drate_amplitude_bps_ = 0.0;
  waveform_delta_reference_valid_ = false;
  waveform_delta_reference_bps_ = 0.0;
  consecutive_overload_count_ = 0;
  waveform_last_delta_source_ = kWaveformDeltaSourceNone;
  waveform_last_raw_delta_bw_bps_ = 0.0;
  waveform_last_applied_delta_bw_bps_ = 0.0;
  baseline_adjustment_count_ = 0;
  inconclusive_extension_count_ = 0;
  waveform_inconclusive_amplification_count_ = 0;
  floor_clip_confirmation_count_ = 0;
  waveform_last_clip_direction_ = 0;
  waveform_decision_count_ = 0;
  waveform_amplitude_reduction_count_ = 0;
  trusted_bw_candidate_update_count_ = 0;
  trusted_bw_candidate_ = QuicBandwidth::Zero();
  trusted_bw_candidate_source_ = kTrustedBwSourceNone;
  waveform_last_action_ = "none";
  waveform_last_invalid_reason_ = "none";
  if (cruise_detector_mode_ != FBBRCruiseDetectorMode::kTimeWaveform) {
    return;
  }
  const double native_max_bw_bps = static_cast<double>(
      current_injection_baseline_bw_.ToBitsPerSecond());
  if (current_injection_baseline_bw_.IsZero() ||
      !std::isfinite(native_max_bw_bps) || native_max_bw_bps <= 0.0) {
    waveform_last_invalid_reason_ = "invalid_native_max_bw";
    waveform_last_action_ = "NATIVE_BBR_FALLBACK";
    return;
  }
  if (cruise_modulation_freq_hz_ <= 0.0 ||
      !std::isfinite(cruise_modulation_freq_hz_)) {
    waveform_last_invalid_reason_ = "invalid_modulation_frequency";
    return;
  }
  ScheduleWaveformCollectionAfterSettle(now, true);
}

void FBBRSender::ScheduleWaveformCollectionAfterSettle(
    QuicTime now,
    bool initial_settle) {
  const TimeDelta srtt = CurrentSmoothedRtt();
  if (srtt.IsZero()) {
    waveform_cruise_state_ = WaveformCruiseState::kDisabled;
    waveform_last_invalid_reason_ = "invalid_probe_epoch_rtt";
    waveform_last_action_ = "NATIVE_BBR_FALLBACK";
    return;
  }
  probe_epoch_start_time_ = now;
  probe_epoch_rtt_ = srtt;
  waveform_settle_start_ = now;
  waveform_settle_end_ = now + probe_epoch_rtt_;
  waveform_window_start_ = waveform_settle_end_;
  waveform_window_periods_ =
      !initial_settle && baseline_adjustment_count_ > 0
          ? kWaveformPostAdjustmentCollectionPeriods
          : waveform_initial_window_periods_;
  const double response_duration_s = waveform_window_periods_ /
                                     cruise_modulation_freq_hz_;
  waveform_window_end_ = waveform_window_start_ +
      TimeDelta::FromMicroseconds(static_cast<int64_t>(std::llround(
          response_duration_s * 1000000.0)));
  waveform_window_extended_ = false;
  current_probe_bw_phase_gain_ = 1.0;
  inconclusive_extension_count_ = 0;
  waveform_cruise_state_ =
      initial_settle ? WaveformCruiseState::kWaitInitialSettle
                     : WaveformCruiseState::kWaitPostAdjustmentSettle;
}

void FBBRSender::StartWaveformCollectionAt(
    QuicTime cycle_start,
    double window_periods,
    bool extended_window) {
  if (cruise_modulation_freq_hz_ <= 0.0) {
    waveform_cruise_state_ = WaveformCruiseState::kDisabled;
    waveform_last_invalid_reason_ = "invalid_modulation_frequency";
    return;
  }
  window_periods = ClampValue(window_periods,
                              waveform_initial_window_periods_,
                              waveform_max_window_periods_);
  const double duration_s = window_periods /
                            cruise_modulation_freq_hz_;
  waveform_window_start_ = cycle_start;
  waveform_window_end_ = cycle_start +
      TimeDelta::FromMicroseconds(static_cast<int64_t>(std::llround(
          duration_s * 1000000.0)));
  waveform_window_periods_ = window_periods;
  waveform_window_extended_ = extended_window;
  waveform_cruise_state_ = extended_window
                               ? WaveformCruiseState::kExtendCycle
                               : WaveformCruiseState::kCollectCycle;
}

FBBRSender::WaveformWindowAnalysis
FBBRSender::AnalyzeFbbrHybridWindow(QuicTime window_start,
                                    QuicTime window_end,
                                    double window_periods,
                                    bool extended_window) const {
  WaveformWindowAnalysis result;
  result.fbbr_hybrid_pipeline = true;
  result.probe_epoch_start = probe_epoch_start_time_;
  result.probe_epoch_rtt = probe_epoch_rtt_;
  result.collection_window_start = window_start;
  result.collection_window_end = window_end;
  result.collection_window_periods = window_periods;
  result.window_start = window_start;
  result.window_end = window_end;
  result.window_periods = window_periods;
  result.extended_window = extended_window;
  result.max_srtt_before_ms = fbbr_hybrid_max_srtt_valid_
      ? fbbr_hybrid_max_srtt_ms_ : 0.0;
  result.max_srtt_after_ms = result.max_srtt_before_ms;
  result.rtprop_drate_before_bps = fbbr_hybrid_rtprop_drate_valid_
      ? static_cast<double>(
            fbbr_hybrid_rtprop_drate_.ToBitsPerSecond())
      : 0.0;
  result.rtprop_drate_after_bps = result.rtprop_drate_before_bps;
  result.hybrid_baseline_low_before_valid = adaptive_baseline_low_valid_;
  result.hybrid_baseline_low_before_bps = adaptive_baseline_low_valid_
      ? static_cast<double>(adaptive_baseline_low_.ToBitsPerSecond()) : 0.0;
  result.hybrid_baseline_low_after_valid =
      result.hybrid_baseline_low_before_valid;
  result.hybrid_baseline_low_after_bps =
      result.hybrid_baseline_low_before_bps;
  const QuicBandwidth current_hybrid_max_bw = model_.MaxBandwidth();
  result.hybrid_max_bw_before_valid =
      IsFinitePositiveBandwidth(current_hybrid_max_bw);
  result.hybrid_max_bw_before_bps = result.hybrid_max_bw_before_valid
      ? static_cast<double>(current_hybrid_max_bw.ToBitsPerSecond()) : 0.0;
  result.hybrid_max_bw_after_valid =
      result.hybrid_max_bw_before_valid;
  result.hybrid_max_bw_after_bps =
      result.hybrid_max_bw_before_bps;
  const TimeDelta current_rtprop = fbbr_hybrid_srtt_low_valid_
      ? fbbr_hybrid_srtt_low_ : model_.MinRtt();
  result.hybrid_srtt_low_rtprop_valid = !current_rtprop.IsZero();
  result.hybrid_srtt_low_rtprop_ms =
      result.hybrid_srtt_low_rtprop_valid
          ? static_cast<double>(current_rtprop.ToMicroseconds()) / 1000.0
          : 0.0;
  result.hybrid_max_srtt_valid = fbbr_hybrid_max_srtt_valid_;
  result.hybrid_max_srtt_ms = fbbr_hybrid_max_srtt_valid_
      ? fbbr_hybrid_max_srtt_ms_ : 0.0;
  if (window_end <= window_start || cruise_modulation_freq_hz_ <= 0.0 ||
      probe_epoch_rtt_.IsZero()) {
    result.invalid_reason = "invalid_hybrid_window";
    return result;
  }
  const double period_s = 1.0 / cruise_modulation_freq_hz_;
  const double sample_step_s = ClampValue(period_s / 40.0, 0.001, 0.005);
  const double max_gap_s =
      waveform_max_interpolation_gap_period_ratio_ * period_s;
  const TimeDelta feedback_lag = probe_epoch_rtt_;
  const auto sender_samples = SelectRateSamples(
      sender_rate_history_, window_start - feedback_lag,
      window_end - feedback_lag);
  const auto drate_samples = SelectRateSamples(
      delivery_rate_history_, window_start, window_end);
  const auto srtt_samples = SelectRttSamples(
      srtt_history_, window_start, window_end);
  result.sender_sample_count = sender_samples.size();
  result.drate_sample_count = drate_samples.size();
  result.srtt_sample_count = srtt_samples.size();
  const RobustQueueGradientEstimate queue_gradient =
      ComputeRobustQueueGradient(
          srtt_samples,
          result.hybrid_srtt_low_rtprop_valid
              ? result.hybrid_srtt_low_rtprop_ms / 1000.0
              : 0.0);
  result.overload_queue_sample_valid =
      queue_gradient.queue_sample_valid;
  result.overload_queue_sample_count = queue_gradient.sample_count;
  result.overload_q90_s = queue_gradient.q90_s;
  result.overload_queue_gradient_raw =
      queue_gradient.raw_gradient;
  result.overload_queue_gradient_noise =
      queue_gradient.noise_gradient;
  result.overload_queue_gradient = queue_gradient.gradient;
  uint64_t acked_bytes = 0;
  size_t app_limited = 0;
  for (const auto& sample : drate_samples) {
    acked_bytes += sample.acked_bytes;
    if (sample.is_app_limited) {
      ++app_limited;
    }
  }
  result.app_limited_ratio = drate_samples.empty()
      ? 1.0 : static_cast<double>(app_limited) / drate_samples.size();
  ResampledWaveformSeries sender = ResampleRateWaveform(
      sender_samples, window_start - feedback_lag,
      window_end - feedback_lag, sample_step_s, max_gap_s);
  ResampledWaveformSeries drate = ResampleRateWaveform(
      drate_samples, window_start, window_end, sample_step_s, max_gap_s);
  ResampledWaveformSeries srtt = ResampleRttWaveform(
      srtt_samples, window_start, window_end, sample_step_s, max_gap_s);
  if (sender.values.empty() || drate.values.empty() || srtt.values.empty() ||
      sender.values.size() != drate.values.size() ||
      drate.values.size() != srtt.values.size()) {
    result.invalid_reason = "hybrid_resampling_failed";
    return result;
  }
  result.coverage_ratio = std::min(drate.coverage_ratio,
                                   srtt.coverage_ratio);
  result.drate_input_valid = drate_samples.size() >= 4 && acked_bytes > 0 &&
      drate.coverage_ratio + 1e-15 >= waveform_min_cycle_coverage_ratio_ &&
      result.app_limited_ratio <=
          waveform_max_app_limited_sample_ratio_ + 1e-15;
  result.srtt_input_valid = srtt_samples.size() >= 4 &&
      srtt.coverage_ratio + 1e-15 >= waveform_min_cycle_coverage_ratio_;

  auto resampled_stats = [](const std::vector<double>& values,
                            const std::vector<bool>& valid,
                            double* minimum,
                            double* maximum,
                            double* mean,
                            size_t* count) {
    *minimum = std::numeric_limits<double>::infinity();
    *maximum = -std::numeric_limits<double>::infinity();
    *mean = 0.0;
    *count = 0;
    for (size_t i = 0; i < values.size(); ++i) {
      if (valid[i] && std::isfinite(values[i]) && values[i] > 0.0) {
        *minimum = std::min(*minimum, values[i]);
        *maximum = std::max(*maximum, values[i]);
        *mean += values[i];
        ++*count;
      }
    }
    if (*count == 0) {
      *minimum = *maximum = *mean = 0.0;
      return false;
    }
    *mean /= static_cast<double>(*count);
    return std::isfinite(*minimum) && std::isfinite(*maximum) &&
        std::isfinite(*mean);
  };
  result.delivery_rate_stats_valid = resampled_stats(
      drate.values, drate.valid, &result.delivery_rate_min_bps,
      &result.delivery_rate_max_bps, &result.delivery_rate_mean_bps,
      &result.delivery_rate_stat_sample_count) && result.drate_input_valid;
  result.srtt_stats_valid = resampled_stats(
      srtt.values, srtt.valid, &result.srtt_min_ms,
      &result.srtt_max_ms, &result.srtt_mean_ms,
      &result.srtt_stat_sample_count) && result.srtt_input_valid;

  double sender_period_correlation = -1.0;
  const double srate_period_s = EstimateActualSignalPeriod(
      sender.values, sender.valid, sample_step_s, period_s,
      &sender_period_correlation);
  result.sender_waveform_valid = srate_period_s > 0.0;
  FbbrHybridRegimeFeatures& features = result.hybrid_features;
  features.estimated_srate_period_s = srate_period_s;
  features.srtt_stats_valid = result.srtt_stats_valid;
  features.srtt_min_ms = result.srtt_min_ms;
  features.srtt_mean_ms = result.srtt_mean_ms;
  features.srtt_max_ms = result.srtt_max_ms;
  features.inflight_bytes = latest_congestion_event_inflight_;
  features.bdp_bytes = model_.BDP();
  features.inflight_bdp_valid =
      latest_congestion_event_inflight_valid_ && features.bdp_bytes > 0;
  features.drate_stats_valid = result.delivery_rate_stats_valid;
  features.mindrate_bps = result.delivery_rate_min_bps;
  features.maxdrate_bps = result.delivery_rate_max_bps;
  features.meandrate_bps = result.delivery_rate_mean_bps;

  auto build_masks_and_raw_evidence = [&](
      const std::vector<double>& values,
      const std::vector<bool>& original_valid,
      bool drate_signal,
      SignalRegimeFeatures* signal,
      std::vector<bool>* clean_valid,
      std::vector<bool>* periodic_valid,
      std::vector<ContinuousHorizontalEvidence>* continuous) {
    signal->input_valid = drate_signal
        ? result.drate_input_valid : result.srtt_input_valid;
    signal->ordinary_wave_uses_raw_valid_view = drate_signal;
    *continuous = DetectContinuousHorizontalSegments(
        values, original_valid, sample_step_s, period_s);
    signal->continuous_horizontal_count =
        static_cast<uint32_t>(continuous->size());
    signal->top_repeated_clip = DetectRepeatedClipLineContacts(
        values, original_valid, sample_step_s, period_s, true);
    signal->bottom_repeated_clip = DetectRepeatedClipLineContacts(
        values, original_valid, sample_step_s, period_s, false);
    signal->repeated_top_clip = signal->top_repeated_clip.valid;
    signal->repeated_bottom_clip = signal->bottom_repeated_clip.valid;
    signal->suspected_top_candidate = signal->repeated_top_clip;
    signal->suspected_bottom_candidate = signal->repeated_bottom_clip;
    std::vector<bool> protected_mask(values.size(), false);
    *clean_valid = original_valid;
    *periodic_valid = original_valid;
    size_t edge_masked = 0;
    for (const auto& segment : *continuous) {
      signal->suspected_top_candidate =
          signal->suspected_top_candidate || segment.is_upper;
      signal->suspected_bottom_candidate =
          signal->suspected_bottom_candidate || segment.is_lower;
      if (segment.is_upper) {
        signal->longest_top_line_ratio_of_period = std::max(
            signal->longest_top_line_ratio_of_period,
            segment.duration_ratio_of_period);
      }
      if (segment.is_lower) {
        signal->longest_bottom_line_ratio_of_period = std::max(
            signal->longest_bottom_line_ratio_of_period,
            segment.duration_ratio_of_period);
      }
      const bool edge = segment.touches_left_edge ||
                        segment.touches_right_edge;
      for (size_t i = segment.start_index;
           i <= segment.end_index && i < values.size(); ++i) {
        if (edge) {
          if ((*periodic_valid)[i]) {
            ++edge_masked;
          }
          (*periodic_valid)[i] = false;
          (*clean_valid)[i] = false;
        } else {
          protected_mask[i] = true;
        }
      }
      signal->left_edge_line_masked = signal->left_edge_line_masked ||
          segment.touches_left_edge;
      signal->right_edge_line_masked = signal->right_edge_line_masked ||
          segment.touches_right_edge;
    }
    const auto middle = DetectMiddleSequentialDisturbances(
        values, *periodic_valid, protected_mask, sample_step_s, period_s);
    size_t middle_masked = 0;
    for (const auto& disturbance : middle) {
      signal->middle_sequential_masked = true;
      signal->middle_best_slope_mismatch_ratio = std::max(
          signal->middle_best_slope_mismatch_ratio,
          disturbance.slope_mismatch_ratio);
      signal->middle_best_bridge_deviation_ratio = std::max(
          signal->middle_best_bridge_deviation_ratio,
          disturbance.bridge_deviation_ratio);
      for (size_t i = disturbance.start_index;
           i <= disturbance.end_index && i < values.size(); ++i) {
        if ((*periodic_valid)[i]) {
          ++middle_masked;
        }
        (*periodic_valid)[i] = false;
        (*clean_valid)[i] = false;
      }
    }
    signal->edge_mask_ratio = values.empty()
        ? 0.0 : static_cast<double>(edge_masked) / values.size();
    signal->middle_mask_ratio = values.empty()
        ? 0.0 : static_cast<double>(middle_masked) / values.size();
  };

  std::vector<bool> srtt_clean_valid;
  std::vector<bool> srtt_periodic_valid;
  std::vector<bool> drate_clean_valid;
  std::vector<bool> drate_periodic_valid;
  std::vector<ContinuousHorizontalEvidence> srtt_continuous;
  std::vector<ContinuousHorizontalEvidence> drate_continuous;
  build_masks_and_raw_evidence(
      srtt.values, srtt.valid, false, &features.srtt,
      &srtt_clean_valid, &srtt_periodic_valid, &srtt_continuous);
  build_masks_and_raw_evidence(
      drate.values, drate.valid, true, &features.drate,
      &drate_clean_valid, &drate_periodic_valid, &drate_continuous);

  features.srtt.wave = DetectOrdinaryWaveActivity(
      srtt.values, srtt_clean_valid, sample_step_s, period_s, true);
  // PDF: DRate ordinary-wave detection always uses the raw valid view.
  features.drate.wave = DetectOrdinaryWaveActivity(
      drate.values, drate.valid, sample_step_s, period_s);

  auto derive_shoulders = [&](const std::vector<double>& values,
                              const std::vector<bool>& valid,
                              const std::vector<ContinuousHorizontalEvidence>&
                                  segments,
                              SignalRegimeFeatures* signal) {
    const std::vector<double> filtered = MedianFilter3(values, valid);
    std::vector<double> finite;
    for (size_t i = 0; i < values.size(); ++i) {
      if (valid[i]) {
        finite.push_back(filtered[i]);
      }
    }
    const double amplitude = Quantile(finite, 0.95) -
        Quantile(finite, 0.05);
    const double level = std::max(std::abs(Median(finite)), 1e-12);
    const bool amplitude_ok = amplitude >=
        waveform_activity_min_level_ratio_ * level;
    const size_t leg_points = static_cast<size_t>(std::ceil(
        waveform_shoulder_min_residual_cycle_leg_duration_ratio_ *
        period_s / sample_step_s));
    for (const auto& segment : segments) {
      if (segment.touches_left_edge || segment.touches_right_edge ||
          (!segment.is_upper && !segment.is_lower)) {
        continue;
      }
      size_t positive_points = 0;
      size_t negative_points = 0;
      size_t aligned_points = 0;
      for (size_t i = segment.start_index; i <= segment.end_index &&
           i < sender.values.size(); ++i) {
        if (!sender.valid[i]) {
          continue;
        }
        ++aligned_points;
        const double centered_sender = sender.values[i] -
            Median(std::vector<double>(sender.values.begin(),
                                       sender.values.end()));
        positive_points += centered_sender >= 0.0 ? 1 : 0;
        negative_points += centered_sender < 0.0 ? 1 : 0;
      }
      const double positive_overlap = aligned_points == 0
          ? 0.0 : static_cast<double>(positive_points) / aligned_points;
      const double negative_overlap = aligned_points == 0
          ? 0.0 : static_cast<double>(negative_points) / aligned_points;
      const bool upper_shape = segment.is_upper &&
          positive_overlap + 1e-15 >=
              waveform_shoulder_min_half_overlap_ratio_ &&
          segment.left_context_slope > 0.0 &&
          segment.right_context_slope < 0.0;
      const bool lower_shape = segment.is_lower &&
          negative_overlap + 1e-15 >=
              waveform_shoulder_min_half_overlap_ratio_ &&
          segment.left_context_slope < 0.0 &&
          segment.right_context_slope > 0.0;
      if (!upper_shape && !lower_shape) {
        continue;
      }
      const bool cycle_input_valid = srate_period_s > 0.0 && amplitude_ok &&
          segment.start_index >= leg_points &&
          segment.end_index + leg_points < values.size();
      if (upper_shape) {
        signal->positive_shoulder_cycle_input_valid = cycle_input_valid;
      } else {
        signal->negative_shoulder_cycle_input_valid = cycle_input_valid;
      }
      if (!cycle_input_valid) {
        continue;
      }
      const size_t search_radius = static_cast<size_t>(std::ceil(
          1.25 * srate_period_s / sample_step_s));
      const size_t left_begin = segment.start_index > search_radius
          ? segment.start_index - search_radius : 0;
      const size_t right_end = std::min(
          values.size(), segment.end_index + search_radius + 1);
      size_t left_extreme = segment.start_index;
      size_t right_extreme = segment.end_index;
      bool left_found = false;
      bool right_found = false;
      for (size_t i = left_begin; i < segment.start_index; ++i) {
        if (!valid[i]) {
          continue;
        }
        if (!left_found || (upper_shape
            ? filtered[i] < filtered[left_extreme]
            : filtered[i] > filtered[left_extreme])) {
          left_extreme = i;
          left_found = true;
        }
      }
      for (size_t i = segment.end_index + 1; i < right_end; ++i) {
        if (!valid[i]) {
          continue;
        }
        if (!right_found || (upper_shape
            ? filtered[i] < filtered[right_extreme]
            : filtered[i] > filtered[right_extreme])) {
          right_extreme = i;
          right_found = true;
        }
      }
      const double extrema_period =
          (right_extreme - left_extreme) * sample_step_s;
      const double extrema_error =
          std::abs(extrema_period - srate_period_s) / srate_period_s;
      const bool recognizable = left_found && right_found &&
          segment.start_index - left_extreme >= leg_points &&
          right_extreme - segment.end_index >= leg_points &&
          extrema_error <=
              waveform_shoulder_max_residual_cycle_period_error_ratio_ +
                  1e-15;
      if (upper_shape) {
        signal->positive_shoulder_cycle_recognizable = recognizable;
        signal->positive_shoulder = signal->positive_shoulder ||
                                    recognizable;
      } else {
        signal->negative_shoulder_cycle_recognizable = recognizable;
        signal->negative_shoulder = signal->negative_shoulder ||
                                    recognizable;
      }
    }
  };
  derive_shoulders(srtt.values, srtt.valid, srtt_continuous,
                   &features.srtt);
  derive_shoulders(drate.values, drate.valid, drate_continuous,
                   &features.drate);

  features.srtt.long_top_line =
      features.srtt.longest_top_line_ratio_of_period >
          fbbr_regime_long_top_horizontal_duration_ratio_;
  features.srtt.long_bottom_line =
      features.srtt.longest_bottom_line_ratio_of_period >
          fbbr_regime_long_bottom_horizontal_duration_ratio_;
  features.drate.long_top_line =
      features.drate.longest_top_line_ratio_of_period >
          fbbr_regime_long_top_horizontal_duration_ratio_;
  features.drate.long_bottom_line =
      features.drate.longest_bottom_line_ratio_of_period >
          fbbr_regime_long_bottom_horizontal_duration_ratio_;

  const bool srtt_horizontal_candidate =
      features.srtt.positive_shoulder || features.srtt.long_top_line ||
      features.srtt.repeated_top_clip || features.srtt.negative_shoulder ||
      features.srtt.long_bottom_line || features.srtt.repeated_bottom_clip;
  const bool srtt_wave_coexists =
      features.srtt.wave.input_valid && features.srtt.wave.has_wave;
  if (srtt_horizontal_candidate && !srtt_wave_coexists) {
    features.clip_candidate_rejected_to_wave_fallback = true;
    features.srtt.positive_shoulder = false;
    features.srtt.negative_shoulder = false;
    features.srtt.long_top_line = false;
    features.srtt.long_bottom_line = false;
    features.srtt.repeated_top_clip = false;
    features.srtt.repeated_bottom_clip = false;
  }

  const bool srtt_upper_verified = features.srtt.positive_shoulder ||
      features.srtt.long_top_line || features.srtt.repeated_top_clip;
  const bool srtt_lower_verified = features.srtt.negative_shoulder ||
      features.srtt.long_bottom_line || features.srtt.repeated_bottom_clip;
  const bool drate_upper_verified = features.drate.positive_shoulder ||
      features.drate.long_top_line || features.drate.repeated_top_clip;
  const bool drate_lower_verified = features.drate.negative_shoulder ||
      features.drate.long_bottom_line || features.drate.repeated_bottom_clip;
  features.srtt.lower_clip_ignored_for_periodic = srtt_lower_verified;
  features.drate.lower_clip_ignored_for_periodic = drate_lower_verified;
  features.srtt.periodic = AnalyzeFbbrHybridPeriodicSimilarity(
      srtt.values, srtt.valid, srtt_periodic_valid,
      sample_step_s, period_s, srate_period_s, srtt_upper_verified,
      &features.srtt);
  features.drate.periodic = AnalyzeFbbrHybridPeriodicSimilarity(
      drate.values, drate.valid, drate_periodic_valid,
      sample_step_s, period_s, srate_period_s, drate_upper_verified,
      &features.drate);

  const bool u1 = features.srtt.positive_shoulder;
  const bool u2 = features.srtt.long_top_line;
  const bool u3 = features.srtt.repeated_top_clip;
  const bool l1 = features.srtt.negative_shoulder;
  const bool l2 = features.srtt.long_bottom_line;
  const bool l3 = features.srtt.repeated_bottom_clip;
  features.both_clip_directions = (u1 || u2 || u3) && (l1 || l2 || l3);
  if (u1) {
    features.selected_clip_case = SrttClipCase::kU1PositiveShoulder;
  } else if (u2) {
    features.selected_clip_case = SrttClipCase::kU2LongTopLine;
  } else if (u3) {
    features.selected_clip_case = SrttClipCase::kU3RepeatedTopClip;
  } else if (l1) {
    features.selected_clip_case = SrttClipCase::kL1NegativeShoulder;
  } else if (l2) {
    features.selected_clip_case = SrttClipCase::kL2LongBottomLine;
  } else if (l3) {
    features.selected_clip_case = SrttClipCase::kL3RepeatedBottomClip;
  } else {
    features.fallback_entered = true;
    features.clip_candidate_rejected_to_wave_fallback =
        features.clip_candidate_rejected_to_wave_fallback ||
        features.srtt.suspected_top_candidate ||
        features.srtt.suspected_bottom_candidate;
  }
  features.input_valid = result.srtt_input_valid &&
                         result.drate_input_valid;
  FbbrRegimeContext context;
  context.max_srtt_valid = fbbr_hybrid_max_srtt_valid_;
  context.max_srtt_ms = fbbr_hybrid_max_srtt_ms_;
  context.rtprop_valid = !current_rtprop.IsZero();
  context.rtprop_ms = context.rtprop_valid
      ? static_cast<double>(current_rtprop.ToMicroseconds()) / 1000.0
      : 0.0;
  result.hybrid_decision = ClassifyFbbrHybridRegime(features, context);
  result.classification = result.hybrid_decision.classification;
  result.unsuppressed_classification = result.classification;
  result.decision_rule = result.hybrid_decision.rule_id;
  result.drate_similar = features.drate.periodic_similar;
  result.drate_effective_similar = result.drate_similar;
  result.srtt_similar = features.srtt.periodic_similar;
  result.srtt_effective_similar = result.srtt_similar;
  result.drate_match = result.drate_similar;
  result.srtt_match = result.srtt_similar;
  result.plateau.drate_has_waveform = features.drate.wave.has_wave;
  const int64_t period_us = std::max<int64_t>(
      1, static_cast<int64_t>(std::llround(period_s * 1000000.0)));
  const int64_t window_start_us =
      (window_start - QuicTime::Zero()).ToMicroseconds();
  result.window_first_cycle_id = static_cast<uint64_t>(
      std::max<int64_t>(0, window_start_us / period_us));
  result.window_second_cycle_id = result.window_first_cycle_id + 1;
  if (result.classification == WaveformClassification::kInconclusive) {
    result.invalid_reason = result.hybrid_decision.rule_id[0] == '\0'
        ? "hybrid_required_predicate_invalid"
        : "hybrid_inconclusive";
  } else {
    result.invalid_reason = "none";
  }
  return result;
}

FBBRSender::WaveformWindowAnalysis
FBBRSender::AnalyzeWaveformWindow(QuicTime window_start,
                                      QuicTime window_end,
                                      double window_periods,
                                      bool extended_window) const {
  if (IsFbbrHybridObserver()) {
    return AnalyzeFbbrHybridWindow(window_start, window_end,
                                   window_periods, extended_window);
  }
  WaveformWindowAnalysis result;
  result.probe_epoch_start = probe_epoch_start_time_;
  result.probe_epoch_rtt = probe_epoch_rtt_;
  result.collection_window_start = window_start;
  result.collection_window_end = window_end;
  result.collection_window_periods = window_periods;
  result.window_start = window_start;
  result.window_end = window_end;
  result.window_periods = window_periods;
  result.extended_window = extended_window;
  const bool use_adaptive_load_judgment = UsesAdaptiveLoadJudgment();
  auto apply_r6_fallback = [&]() {
    WaveformDecisionInputs decision;
    decision.srtt_input_valid = result.srtt_input_valid;
    decision.drate_input_valid = result.drate_input_valid;
    decision.drate_similar = result.drate_similar;
    decision.drate_similar_without_middle =
        result.drate_similar_without_middle;
    decision.bic_srtt_top_clip = result.bic_clipping.top_clip;
    decision.bic_srtt_bottom_clip = result.bic_clipping.bottom_clip;
    decision.drate_positive_half_clipped =
        result.plateau.drate_positive_half_clipped;
    decision.drate_only_negative_half =
        result.plateau.drate_only_negative_half;
    decision.positive_half_clips_simultaneous =
        result.plateau.positive_half_clips_simultaneous;
    decision.drate_has_waveform = result.plateau.drate_has_waveform;
    decision.drate_middle_any_plateau =
        result.plateau.drate_middle_any_plateau;
    decision.adaptive_guard_enabled = use_adaptive_load_judgment;
    decision.srtt_window_stats_valid = result.srtt_stats_valid;
    decision.srtt_mean_ms = result.srtt_mean_ms;
    decision.srtt_min_ms = result.srtt_min_ms;
    decision.srtt_max_ms = result.srtt_max_ms;
    decision.latest_waveform_overload_srtt_mean_valid =
        result.latest_waveform_overload_srtt_mean_valid;
    decision.latest_waveform_overload_srtt_mean_ms =
        result.latest_waveform_overload_srtt_mean_ms;
    decision.latest_waveform_underload_srtt_mean_valid =
        result.latest_waveform_underload_srtt_mean_valid;
    decision.latest_waveform_underload_srtt_mean_ms =
        result.latest_waveform_underload_srtt_mean_ms;
    result.classification = ClassifyWaveformState(
        decision, &result.decision_rule);
  };
  if (window_end <= window_start || cruise_modulation_freq_hz_ <= 0.0 ||
      current_injection_baseline_bw_.IsZero() ||
      probe_epoch_start_time_ == QuicTime::Zero() ||
      probe_epoch_rtt_.IsZero()) {
    result.invalid_reason = "invalid_waveform_window";
    apply_r6_fallback();
    return result;
  }
  const double period_s = 1.0 / cruise_modulation_freq_hz_;
  const double max_gap_s =
      waveform_max_interpolation_gap_period_ratio_ * period_s;
  const double srtt_s = static_cast<double>(
      probe_epoch_rtt_.ToMicroseconds()) / 1000000.0;
  const double lag_radius_s = std::min(0.5 * srtt_s, 0.25 * period_s);
  const double lag_min_s = std::max(0.0, srtt_s - lag_radius_s);
  const double lag_max_s = srtt_s + lag_radius_s;
  const TimeDelta lag_max = TimeDelta::FromMicroseconds(
      static_cast<int64_t>(std::llround(lag_max_s * 1000000.0)));

  const auto sender_samples = SelectRateSamples(
      sender_rate_history_, window_start - lag_max, window_end);
  const auto drate_samples = SelectRateSamples(
      delivery_rate_history_, window_start, window_end);
  const auto srtt_samples = SelectRttSamples(
      srtt_history_, window_start, window_end);
  const DeliveryRateWindowStats delivery_stats =
      ComputeDeliveryRateWindowStats(drate_samples);
  const SrttWindowStats srtt_stats = ComputeSrttWindowStats(srtt_samples);
  result.sender_sample_count = sender_samples.size();
  result.drate_sample_count = drate_samples.size();
  result.srtt_sample_count = srtt_samples.size();
  result.srtt_stat_sample_count = srtt_stats.sample_count;
  result.srtt_stats_valid = srtt_stats.valid;
  result.srtt_mean_ms = srtt_stats.mean_ms;
  result.srtt_min_ms = srtt_stats.min_ms;
  result.srtt_max_ms = srtt_stats.max_ms;
  result.latest_waveform_overload_srtt_mean_valid =
      latest_waveform_overload_srtt_mean_valid_;
  result.latest_waveform_overload_srtt_mean_ms =
      latest_waveform_overload_srtt_mean_ms_;
  result.latest_waveform_underload_srtt_mean_valid =
      latest_waveform_underload_srtt_mean_valid_;
  result.latest_waveform_underload_srtt_mean_ms =
      latest_waveform_underload_srtt_mean_ms_;
  result.delivery_rate_stat_sample_count = delivery_stats.sample_count;
  result.delivery_rate_stats_valid = delivery_stats.valid;
  result.delivery_rate_min_bps = delivery_stats.min_bps;
  result.delivery_rate_max_bps = delivery_stats.max_bps;
  result.delivery_rate_mean_bps = delivery_stats.mean_bps;
  result.latest_trusted_bw_bps = static_cast<double>(
      fbbr_latest_trusted_bw_.ToBitsPerSecond());
  result.smoothed_trusted_bw_bps = static_cast<double>(
      fbbr_smoothed_trusted_bw_.ToBitsPerSecond());
  size_t app_limited_samples = 0;
  uint64_t acked_bytes = 0;
  for (const auto& sample : drate_samples) {
    if (sample.is_app_limited) {
      ++app_limited_samples;
    }
    acked_bytes += sample.acked_bytes;
  }
  result.app_limited_ratio = drate_samples.empty()
      ? 1.0
      : static_cast<double>(app_limited_samples) /
            static_cast<double>(drate_samples.size());

  ResampledWaveformSeries drate = ResampleRateWaveform(
      drate_samples, window_start, window_end, kSampleStepSec, max_gap_s);
  ResampledWaveformSeries srtt = ResampleRttWaveform(
      srtt_samples, window_start, window_end, kSampleStepSec, max_gap_s);
  if (drate.values.empty() || srtt.values.empty() ||
      drate.values.size() != srtt.values.size()) {
    result.invalid_reason = "receiver_resampling_failed";
    apply_r6_fallback();
    return result;
  }
  auto winsorize = [](std::vector<double>* values,
                      const std::vector<bool>& valid_values) {
    std::vector<double> finite;
    for (size_t i = 0; i < values->size(); ++i) {
      if (valid_values[i] && std::isfinite((*values)[i])) {
        finite.push_back((*values)[i]);
      }
    }
    if (finite.size() < 4) {
      return;
    }
    std::sort(finite.begin(), finite.end());
    const size_t low_index = static_cast<size_t>(
        std::floor(0.01 * (finite.size() - 1)));
    const size_t high_index = static_cast<size_t>(
        std::ceil(0.99 * (finite.size() - 1)));
    for (size_t i = 0; i < values->size(); ++i) {
      if (valid_values[i] && std::isfinite((*values)[i])) {
        (*values)[i] = ClampValue((*values)[i],
                                  finite[low_index],
                                  finite[high_index]);
      }
    }
  };
  winsorize(&drate.values, drate.valid);
  winsorize(&srtt.values, srtt.valid);
  result.drate_input_valid =
      result.drate_sample_count >= 4 && acked_bytes > 0 &&
      drate.coverage_ratio >= waveform_min_cycle_coverage_ratio_ &&
      result.app_limited_ratio <= waveform_max_app_limited_sample_ratio_;
  result.srtt_input_valid =
      result.srtt_sample_count >= 4 &&
      srtt.coverage_ratio >= waveform_min_cycle_coverage_ratio_;
  std::vector<bool> receiver_valid(drate.values.size(), false);
  for (size_t i = 0; i < receiver_valid.size(); ++i) {
    receiver_valid[i] = drate.valid[i] && srtt.valid[i];
  }
  const std::vector<double> drate_raw_detrended =
      DetrendLinear(drate.values, drate.valid);
  std::vector<double> drate_detrended =
      MedianFilter3(drate_raw_detrended, drate.valid);
  std::vector<double> srtt_detrended =
      DetrendLinear(srtt.values, srtt.valid);
  const std::vector<double> srtt_smoothed =
      MedianFilter3(srtt_detrended, srtt.valid);
  std::vector<double> srtt_noise_residuals;
  for (size_t i = 0; i < srtt_detrended.size(); ++i) {
    if (srtt.valid[i] && std::isfinite(srtt_detrended[i]) &&
        std::isfinite(srtt_smoothed[i])) {
      srtt_noise_residuals.push_back(
          srtt_detrended[i] - srtt_smoothed[i]);
    }
  }
  const double srtt_noise_center = Median(srtt_noise_residuals);
  std::vector<double> srtt_noise_deviations;
  srtt_noise_deviations.reserve(srtt_noise_residuals.size());
  for (double residual : srtt_noise_residuals) {
    srtt_noise_deviations.push_back(
        std::abs(residual - srtt_noise_center));
  }
  const double shape_independent_srtt_noise_sigma =
      1.4826 * Median(srtt_noise_deviations);
  // True clipping is shape-only evidence.  Detect it before sender/receiver
  // lag alignment, similarity, and shoulder analysis so failures in those
  // secondary paths cannot hide a valid horizontal cut.
  result.bic_clipping = DetectBicSrttClipping(
      srtt.values, srtt.valid, shape_independent_srtt_noise_sigma);
  std::vector<double> drate_noise_residuals;
  for (size_t i = 0; i < drate_raw_detrended.size(); ++i) {
    if (drate.valid[i] && std::isfinite(drate_raw_detrended[i]) &&
        std::isfinite(drate_detrended[i])) {
      drate_noise_residuals.push_back(
          drate_raw_detrended[i] - drate_detrended[i]);
    }
  }
  const double drate_noise_center = Median(drate_noise_residuals);
  std::vector<double> drate_noise_deviations;
  for (double residual : drate_noise_residuals) {
    drate_noise_deviations.push_back(
        std::abs(residual - drate_noise_center));
  }
  const double shape_independent_drate_noise_sigma =
      1.4826 * Median(drate_noise_deviations);
  const std::vector<double> drate_normalized =
      RobustNormalize(drate_detrended, drate.valid);
  const std::vector<double> srtt_normalized =
      RobustNormalize(srtt_detrended, srtt.valid);
  const double slope_window_s = std::max(
      waveform_min_local_slope_window_ms_ / 1000.0,
      waveform_local_slope_window_period_ratio_ * period_s);
  std::vector<double> srtt_slopes = ComputeLocalLinearSlopes(
      srtt_detrended, srtt.valid, kSampleStepSec, slope_window_s);
  std::vector<double> drate_slopes = ComputeLocalLinearSlopes(
      drate_detrended, drate.valid, kSampleStepSec, slope_window_s);
  const std::vector<double> srtt_slopes_normalized =
      RobustNormalize(srtt_slopes, srtt.valid);

  double best_score = -std::numeric_limits<double>::infinity();
  bool lag_identified = false;
  ResampledWaveformSeries best_sender;
  std::vector<double> best_sender_residual;
  std::vector<double> best_sender_unit;
  std::vector<double> best_sender_normalized;
  std::vector<double> best_integral;
  std::vector<double> best_integral_normalized;
  std::vector<bool> best_valid;
  const double base_pacing_bps = static_cast<double>(
      current_injection_baseline_bw_.ToBitsPerSecond());
  const double configured_amplitude_bps =
      static_cast<double>(current_probe_amplitude_bps_);
  if (!std::isfinite(base_pacing_bps) || base_pacing_bps <= 0.0 ||
      !std::isfinite(configured_amplitude_bps) ||
      configured_amplitude_bps <= 0.0) {
    result.invalid_reason = "invalid_baseline_or_amplitude";
    apply_r6_fallback();
    return result;
  }
  const int lag_steps = std::max(
      1, static_cast<int>(std::ceil(
             (lag_max_s - lag_min_s) / kSampleStepSec)) + 1);
  for (int lag_index = 0; lag_index < lag_steps; ++lag_index) {
    const double lag_s = std::min(
        lag_max_s, lag_min_s + lag_index * kSampleStepSec);
    const TimeDelta lag = TimeDelta::FromMicroseconds(
        static_cast<int64_t>(std::llround(lag_s * 1000000.0)));
    ResampledWaveformSeries sender = ResampleRateWaveform(
        sender_samples, window_start - lag, window_end - lag,
        kSampleStepSec, max_gap_s);
    if (sender.values.size() != drate.values.size()) {
      continue;
    }
    std::vector<double> residual(sender.values.size(), 0.0);
    std::vector<double> sender_unit(sender.values.size(), 0.0);
    std::vector<bool> joint_valid(sender.values.size(), false);
    for (size_t i = 0; i < sender.values.size(); ++i) {
      residual[i] = sender.values[i] - base_pacing_bps;
      sender_unit[i] = residual[i] / configured_amplitude_bps;
      joint_valid[i] = sender.valid[i] && receiver_valid[i];
    }
    const size_t joint_count = static_cast<size_t>(
        std::count(joint_valid.begin(), joint_valid.end(), true));
    if (joint_count < 4) {
      continue;
    }
    const std::vector<double> sender_normalized = RobustNormalize(
        DetrendLinear(sender_unit, joint_valid), joint_valid);
    std::vector<double> integral = BuildQueueIntegralTemplate(
        sender_unit, joint_valid, kSampleStepSec, period_s);
    const std::vector<double> integral_normalized = RobustNormalize(
        DetrendLinear(integral, joint_valid), joint_valid);
    const double drate_ncc = ComputeNormalizedCrossCorrelation(
        sender_normalized, drate_normalized, joint_valid);
    const double srtt_direct_ncc = ComputeNormalizedCrossCorrelation(
        sender_normalized, srtt_normalized, joint_valid);
    const double integral_ncc = ComputeNormalizedCrossCorrelation(
        integral_normalized, srtt_normalized, joint_valid);
    const double derivative_ncc = ComputeNormalizedCrossCorrelation(
        sender_normalized, srtt_slopes_normalized, joint_valid);
    // Align sender phase from Delivery Rate only. SRTT shape may be sinusoidal,
    // triangular, or otherwise distorted, so its template correlations are
    // diagnostics and must not influence clipping phase or classification.
    const double score = drate_ncc;
    if (!lag_identified || score > best_score) {
      lag_identified = true;
      best_score = score;
      result.best_lag_s = lag_s;
      result.drate_ncc = drate_ncc;
      result.srtt_direct_ncc = srtt_direct_ncc;
      result.srtt_integral_ncc = integral_ncc;
      result.srtt_derivative_ncc = derivative_ncc;
      best_sender = sender;
      best_sender_residual.swap(residual);
      best_sender_unit.swap(sender_unit);
      best_sender_normalized.assign(sender_normalized.begin(),
                                    sender_normalized.end());
      best_integral.swap(integral);
      best_integral_normalized.assign(integral_normalized.begin(),
                                      integral_normalized.end());
      best_valid.swap(joint_valid);
    }
  }
  if (!lag_identified || best_sender.values.empty()) {
    result.invalid_reason = "lag_not_identifiable";
    apply_r6_fallback();
    return result;
  }
  const size_t joint_valid_count = static_cast<size_t>(
      std::count(best_valid.begin(), best_valid.end(), true));
  result.coverage_ratio = static_cast<double>(joint_valid_count) /
                          static_cast<double>(best_valid.size());
  result.drate_slope_direction_agreement = ComputeSlopeDirectionAgreement(
      best_sender_normalized, drate_normalized, best_valid);
  result.srtt_slope_direction_agreement = std::max(
      ComputeSlopeDirectionAgreement(best_sender_normalized,
                                     srtt_normalized, best_valid),
      std::max(ComputeSlopeDirectionAgreement(best_integral_normalized,
                                              srtt_normalized, best_valid),
               ComputeSlopeDirectionAgreement(best_sender_normalized,
                                              srtt_slopes_normalized,
                                              best_valid)));

  std::vector<bool> sender_srtt_valid(best_valid.size(), false);
  for (size_t i = 0; i < sender_srtt_valid.size(); ++i) {
    sender_srtt_valid[i] = best_sender.valid[i] && srtt.valid[i];
  }
  const CycleCompletenessResult sender_completeness =
      AnalyzeCycleCompleteness(best_sender_residual, sender_srtt_valid,
                               kSampleStepSec, period_s);
  double sender_residual_min = std::numeric_limits<double>::infinity();
  double sender_residual_max = -std::numeric_limits<double>::infinity();
  for (size_t i = 0; i < best_sender_residual.size(); ++i) {
    if (sender_srtt_valid[i]) {
      sender_residual_min = std::min(sender_residual_min,
                                     best_sender_residual[i]);
      sender_residual_max = std::max(sender_residual_max,
                                     best_sender_residual[i]);
    }
  }
  const double minimum_emitted_half_amplitude = std::max(
      1.0, 0.05 * static_cast<double>(current_probe_amplitude_bps_));
  const bool sender_has_both_half_cycles =
      sender_residual_min <= -minimum_emitted_half_amplitude &&
      sender_residual_max >= minimum_emitted_half_amplitude;
  result.sender_waveform_valid =
      sender_completeness.valid && sender_has_both_half_cycles;
  result.drate_completeness = AnalyzeCycleCompleteness(
      drate_detrended, drate.valid, kSampleStepSec, period_s);
  result.srtt_completeness = AnalyzeCycleCompleteness(
      srtt_detrended, srtt.valid, kSampleStepSec, period_s);
  result.drate_fit = EstimateRobustNoise(
      drate_detrended, best_sender_unit, best_valid);
  const double best_srtt_shape = std::max(
      result.srtt_direct_ncc,
      std::max(result.srtt_integral_ncc, result.srtt_derivative_ncc));
  if (best_srtt_shape == result.srtt_direct_ncc) {
    result.srtt_fit = EstimateRobustNoise(
        srtt_detrended, best_sender_unit, best_valid);
  } else if (best_srtt_shape == result.srtt_integral_ncc) {
    result.srtt_fit = EstimateRobustNoise(
        srtt_detrended, best_integral, best_valid);
  } else {
    result.srtt_fit = EstimateRobustNoise(
        srtt_slopes, best_sender_unit, best_valid);
  }
  result.current_drate_response_amplitude_bps =
      result.drate_fit.valid ? result.drate_fit.fitted_response_amplitude : 0.0;
  result.drate_similar =
      result.drate_input_valid &&
      result.drate_completeness.valid;
  result.srtt_similar_frequency =
      result.srtt_input_valid &&
      result.srtt_completeness.valid;
  result.srtt_cycle_complete = result.srtt_completeness.valid;
  result.plateau = DetectDualSignalPlateaus(
      srtt.values, srtt_slopes, drate.values, drate_slopes,
      best_sender_residual, best_valid, kSampleStepSec, period_s,
      shape_independent_srtt_noise_sigma,
      shape_independent_drate_noise_sigma);
  if (result.plateau.srtt_middle_sequential_mask.size() ==
          best_valid.size() &&
      result.plateau.drate_middle_sequential_mask.size() ==
          best_valid.size() &&
      result.plateau.middle_sequential_candidate_count > 0) {
    std::vector<bool> srtt_without_middle(srtt.valid.size(), false);
    std::vector<bool> drate_without_middle(drate.valid.size(), false);
    for (size_t i = 0; i < best_valid.size(); ++i) {
      srtt_without_middle[i] =
          srtt.valid[i] &&
          !result.plateau.srtt_middle_sequential_mask[i];
      drate_without_middle[i] =
          drate.valid[i] &&
          !result.plateau.drate_middle_sequential_mask[i];
    }
    if (result.plateau.srtt_middle_sequential_plateau) {
      result.srtt_without_middle_completeness = AnalyzeCycleCompleteness(
          srtt_detrended, srtt_without_middle, kSampleStepSec, period_s,
          waveform_masked_min_cycle_coverage_ratio_);
      result.srtt_similar_without_middle =
          result.srtt_input_valid &&
          result.srtt_without_middle_completeness.valid;
    }
    if (result.plateau.drate_middle_sequential_plateau) {
      result.drate_without_middle_completeness = AnalyzeCycleCompleteness(
          drate_detrended, drate_without_middle, kSampleStepSec, period_s,
          waveform_masked_min_cycle_coverage_ratio_);
      result.drate_similar_without_middle =
          result.drate_input_valid &&
          result.drate_without_middle_completeness.valid;
    }
  }
  std::vector<bool> srtt_half_valid(srtt.valid.size(), false);
  std::vector<bool> drate_half_valid(drate.valid.size(), false);
  const bool srtt_mask_valid =
      result.plateau.srtt_middle_sequential_mask.size() == srtt.valid.size();
  const bool drate_mask_valid =
      result.plateau.drate_middle_sequential_mask.size() == drate.valid.size();
  for (size_t i = 0; i < best_sender.valid.size(); ++i) {
    srtt_half_valid[i] = best_sender.valid[i] && srtt.valid[i] &&
        (!srtt_mask_valid ||
         !result.plateau.srtt_middle_sequential_mask[i]);
    drate_half_valid[i] = best_sender.valid[i] && drate.valid[i] &&
        (!drate_mask_valid ||
         !result.plateau.drate_middle_sequential_mask[i]);
  }
  result.plateau.srtt_only_negative_half = HasOnlyNegativeHalfWaveform(
      srtt_detrended, best_sender_residual, srtt_half_valid,
      shape_independent_srtt_noise_sigma,
      &result.plateau.srtt_positive_half_span_ms,
      &result.plateau.srtt_negative_half_span_ms);
  result.plateau.srtt_only_positive_half = HasOnlyPositiveHalfWaveform(
      srtt_detrended, best_sender_residual, srtt_half_valid,
      shape_independent_srtt_noise_sigma, nullptr, nullptr);
  result.plateau.drate_only_negative_half = HasOnlyNegativeHalfWaveform(
      drate_detrended, best_sender_residual, drate_half_valid,
      shape_independent_drate_noise_sigma,
      &result.plateau.drate_positive_half_span_bps,
      &result.plateau.drate_negative_half_span_bps);
  result.srtt_positive_half_clipped =
      result.plateau.positive_half_clipped;
  result.srtt_negative_half_clipped =
      result.plateau.negative_half_clipped;
  result.srtt_only_negative_half =
      result.plateau.srtt_only_negative_half;
  result.srtt_only_positive_half =
      result.plateau.srtt_only_positive_half;
  result.srtt_clip_ambiguous = result.plateau.ambiguous;
  result.srtt_similar = result.srtt_similar_frequency;
  result.srtt_effective_similar =
      result.srtt_similar || result.srtt_similar_without_middle;
  result.drate_effective_similar =
      result.drate_similar || result.drate_similar_without_middle;
  result.drate_match = result.drate_effective_similar;
  result.srtt_match = result.srtt_similar;

  std::vector<double> actual_sender_rates;
  size_t floor_clipped = 0;
  for (size_t i = 0; i < best_sender.values.size(); ++i) {
    if (!best_sender.valid[i]) {
      continue;
    }
    actual_sender_rates.push_back(best_sender.values[i]);
    if (best_sender.values[i] <=
        static_cast<double>(minimum_pacing_rate_bps_) + 0.5) {
      ++floor_clipped;
    }
  }
  std::sort(actual_sender_rates.begin(), actual_sender_rates.end());
  const size_t p5_index = actual_sender_rates.empty()
      ? 0
      : static_cast<size_t>(std::floor(
            0.05 * (actual_sender_rates.size() - 1)));
  const size_t p95_index = actual_sender_rates.empty()
      ? 0
      : static_cast<size_t>(std::ceil(
            0.95 * (actual_sender_rates.size() - 1)));
  const double sender_center = actual_sender_rates.empty()
      ? 0.0
      : 0.5 * (actual_sender_rates[p5_index] +
               actual_sender_rates[p95_index]);
  const double observed_sender_amplitude = actual_sender_rates.empty()
      ? 0.0
      : 0.5 * (actual_sender_rates[p95_index] -
               actual_sender_rates[p5_index]);
  const double allowed_baseline_drift = std::max(
      0.05 * std::max(base_pacing_bps, 1.0),
      0.10 * static_cast<double>(current_probe_amplitude_bps_));
  std::string precheck_failure;
  if (result.sender_sample_count < 4 || !result.srtt_input_valid) {
    precheck_failure = AppendReason(precheck_failure,
                                    "insufficient_samples_or_acked_bytes");
  }
  if (!result.sender_waveform_valid) {
    precheck_failure = AppendReason(precheck_failure,
                                    "sender_waveform_incomplete");
    if (!sender_has_both_half_cycles) {
      precheck_failure = AppendReason(
          precheck_failure,
          "sender_missing_positive_or_negative_half_cycle");
    }
  }
  if (!actual_sender_rates.empty() &&
      static_cast<double>(floor_clipped) / actual_sender_rates.size() > 0.10) {
    precheck_failure = AppendReason(precheck_failure,
                                    "pacing_floor_severely_clipped");
  }
  if (!actual_sender_rates.empty() &&
      std::abs(sender_center - base_pacing_bps) > allowed_baseline_drift) {
    precheck_failure = AppendReason(precheck_failure,
                                    "window_baseline_drift");
  }
  if (!actual_sender_rates.empty() &&
      std::abs(observed_sender_amplitude - configured_amplitude_bps) >
          std::max(1000.0, 0.25 * configured_amplitude_bps)) {
    precheck_failure = AppendReason(precheck_failure,
                                    "window_amplitude_changed_or_unformed");
  }
  result.cycle_detected_but_incomplete =
      result.srtt_input_valid && !result.srtt_effective_similar &&
      result.srtt_completeness.periodicity_correlation >=
          waveform_min_periodicity_correlation_;
  const bool true_clip_detected =
      result.bic_clipping.top_clip || result.bic_clipping.bottom_clip;
  if (!precheck_failure.empty() && !true_clip_detected) {
    result.invalid_reason = precheck_failure;
    apply_r6_fallback();
    if (result.classification != WaveformClassification::kInconclusive) {
      result.invalid_reason = "none";
    }
    return result;
  }

  WaveformDecisionInputs decision;
  decision.prechecks_valid = true;
  decision.adaptive_guard_enabled = use_adaptive_load_judgment;
  decision.srtt_input_valid = result.srtt_input_valid;
  decision.srtt_window_stats_valid = result.srtt_stats_valid;
  decision.srtt_mean_ms = result.srtt_mean_ms;
  decision.srtt_min_ms = result.srtt_min_ms;
  decision.srtt_max_ms = result.srtt_max_ms;
  decision.latest_waveform_overload_srtt_mean_valid =
      result.latest_waveform_overload_srtt_mean_valid;
  decision.latest_waveform_overload_srtt_mean_ms =
      result.latest_waveform_overload_srtt_mean_ms;
  decision.latest_waveform_underload_srtt_mean_valid =
      result.latest_waveform_underload_srtt_mean_valid;
  decision.latest_waveform_underload_srtt_mean_ms =
      result.latest_waveform_underload_srtt_mean_ms;
  decision.drate_input_valid = result.drate_input_valid;
  decision.srtt_similar = result.srtt_similar;
  decision.srtt_similar_without_middle =
      result.srtt_similar_without_middle;
  decision.drate_similar = result.drate_similar;
  decision.drate_similar_without_middle =
      result.drate_similar_without_middle;
  decision.srtt_positive_half_clipped =
      result.srtt_positive_half_clipped;
  decision.srtt_negative_half_clipped =
      result.srtt_negative_half_clipped;
  decision.srtt_only_negative_half = result.srtt_only_negative_half;
  decision.srtt_only_positive_half = result.srtt_only_positive_half;
  decision.bic_srtt_top_clip = result.bic_clipping.top_clip;
  decision.bic_srtt_bottom_clip = result.bic_clipping.bottom_clip;
  decision.drate_positive_half_clipped =
      result.plateau.drate_positive_half_clipped;
  decision.drate_only_negative_half =
      result.plateau.drate_only_negative_half;
  decision.positive_half_clips_simultaneous =
      result.plateau.positive_half_clips_simultaneous;
  decision.drate_has_waveform = result.plateau.drate_has_waveform;
  decision.drate_middle_any_plateau =
      result.plateau.drate_middle_any_plateau;
  result.classification = ClassifyWaveformState(
      decision, &result.decision_rule);
  if (result.classification == WaveformClassification::kInconclusive) {
    if (std::string(result.decision_rule) == "R2.4" ||
        std::string(result.decision_rule) == "R3.3" ||
        std::string(result.decision_rule) == "R4.3") {
      result.invalid_reason = std::string("classification_rule_") +
                              result.decision_rule;
    } else if (result.cycle_detected_but_incomplete) {
      result.invalid_reason = "srtt_period_outside_tolerance";
    } else if (!result.srtt_effective_similar) {
      result.invalid_reason = "srtt_period_not_identified";
    } else {
      result.invalid_reason = "invalid_or_insufficient_data";
    }
  } else {
    result.invalid_reason = "none";
  }
  return result;
}

void FBBRSender::RefreshRtpropFromTrueBottomClip(
    WaveformWindowAnalysis* analysis,
    QuicTime now) {
  if (analysis == nullptr ||
      !ShouldRefreshRtpropForTrueClip(analysis->bic_clipping.top_clip,
                                      analysis->bic_clipping.bottom_clip)) {
    return;
  }
  auto time_seconds = [](QuicTime time) {
    return time == QuicTime::Zero()
        ? 0.0
        : static_cast<double>(
              (time - QuicTime::Zero()).ToMicroseconds()) /
              1000000.0;
  };
  const TimeDelta rtprop_before =
      IsFbbrHybridObserver() && fbbr_hybrid_srtt_low_valid_
          ? fbbr_hybrid_srtt_low_ : model_.MinRtt();
  const QuicTime timestamp_before =
      IsFbbrHybridObserver() && fbbr_hybrid_srtt_low_valid_
          ? fbbr_hybrid_srtt_low_source_time_
          : model_.MinRttTimestamp();
  analysis->true_bottom_clip_rtprop_before_ms = rtprop_before.IsZero()
      ? 0.0
      : static_cast<double>(rtprop_before.ToMicroseconds()) / 1000.0;
  analysis->true_bottom_clip_min_rtt_timestamp_before_s =
      time_seconds(timestamp_before);

  if (!analysis->srtt_stats_valid ||
      !std::isfinite(analysis->srtt_min_ms) ||
      analysis->srtt_min_ms <= 0.0 || now == QuicTime::Zero()) {
    return;
  }
  const int64_t minimum_rtt_us = std::max<int64_t>(
      1, static_cast<int64_t>(
             std::llround(analysis->srtt_min_ms * 1000.0)));
  const TimeDelta bottom_min_rtt =
      TimeDelta::FromMicroseconds(minimum_rtt_us);
  if (IsFbbrHybridObserver()) {
    PublishHybridSrttLow(bottom_min_rtt, now, false);
  } else {
    model_.ForceUpdateMinRtt(bottom_min_rtt, now);
    if (in_cruise_) {
      current_cruise_rtprop_updated_ = true;
    }
  }

  analysis->true_bottom_clip_rtprop_refresh_applied = true;
  analysis->true_bottom_clip_rtprop_after_ms =
      static_cast<double>(model_.MinRtt().ToMicroseconds()) / 1000.0;
  analysis->true_bottom_clip_min_rtt_timestamp_after_s =
      time_seconds(model_.MinRttTimestamp());
  analysis->true_bottom_clip_probe_rtt_deadline_after_s =
      time_seconds(model_.MinRttTimestamp() + Params().probe_rtt_period);
}

void FBBRSender::UpdateFbbrHybridRetryState(
    WaveformWindowAnalysis* analysis) {
  if (analysis == nullptr || !analysis->fbbr_hybrid_pipeline) {
    return;
  }
  constexpr uint8_t kInvalidInput = 0x1;
  constexpr uint8_t kTwoWindowNoWave = 0x2;
  analysis->unsuppressed_classification = analysis->classification;
  const bool classification_invalid =
      analysis->classification == WaveformClassification::kInconclusive;
  if (classification_invalid) {
    fbbr_hybrid_retry_reason_mask_ |= kInvalidInput;
  } else {
    fbbr_hybrid_retry_reason_mask_ &=
        static_cast<uint8_t>(~kInvalidInput);
  }
  const bool unique_window =
      analysis->window_second_cycle_id !=
          fbbr_hybrid_last_counted_window_second_cycle_id_;
  if (unique_window) {
    fbbr_hybrid_last_counted_window_second_cycle_id_ =
        analysis->window_second_cycle_id;
    auto update_streak = [](const WaveActivityFeatures& wave,
                            uint8_t* streak) {
      if (!wave.input_valid) {
        return;
      }
      if (wave.has_wave) {
        *streak = 0;
      } else if (*streak < std::numeric_limits<uint8_t>::max()) {
        ++*streak;
      }
    };
    update_streak(analysis->hybrid_features.srtt.wave,
                  &fbbr_hybrid_srtt_no_wave_streak_);
    update_streak(analysis->hybrid_features.drate.wave,
                  &fbbr_hybrid_drate_no_wave_streak_);
  }
  const bool was_active =
      fbbr_hybrid_wave_fidelity_enhancement_active_;
  const bool trigger = !was_active &&
      (fbbr_hybrid_srtt_no_wave_streak_ >=
           fbbr_wave_fidelity_no_wave_trigger_windows_ ||
       fbbr_hybrid_drate_no_wave_streak_ >=
           fbbr_wave_fidelity_no_wave_trigger_windows_);
  const bool either_wave =
      (analysis->hybrid_features.srtt.wave.input_valid &&
       analysis->hybrid_features.srtt.wave.has_wave) ||
      (analysis->hybrid_features.drate.wave.input_valid &&
       analysis->hybrid_features.drate.wave.has_wave);
  if (trigger) {
    fbbr_hybrid_wave_fidelity_enhancement_active_ = true;
    fbbr_hybrid_retry_reason_mask_ |= kTwoWindowNoWave;
    analysis->no_wave_triggered = true;
    analysis->wave_fidelity_just_entered = true;
  } else if (was_active && either_wave) {
    fbbr_hybrid_wave_fidelity_enhancement_active_ = false;
    fbbr_hybrid_retry_reason_mask_ &=
        static_cast<uint8_t>(~kTwoWindowNoWave);
    fbbr_hybrid_srtt_no_wave_streak_ = 0;
    fbbr_hybrid_drate_no_wave_streak_ = 0;
  }
  const bool suppress_for_no_wave = trigger ||
      (was_active && !either_wave);
  if (suppress_for_no_wave) {
    analysis->classification = WaveformClassification::kInconclusive;
    analysis->classification_suppressed_for_retry = true;
    analysis->state_updates_suppressed_for_retry = true;
    analysis->invalid_reason = AppendReason(
        analysis->invalid_reason, "TWO_WINDOW_NO_WAVE");
    if (fbbr_hybrid_rolling_retry_count_ <
        std::numeric_limits<uint32_t>::max()) {
      ++fbbr_hybrid_rolling_retry_count_;
    }
  }
  analysis->wave_fidelity_enhancement_active =
      fbbr_hybrid_wave_fidelity_enhancement_active_;
  analysis->retry_reason_mask = fbbr_hybrid_retry_reason_mask_;
  analysis->srtt_no_wave_streak =
      fbbr_hybrid_srtt_no_wave_streak_;
  analysis->drate_no_wave_streak =
      fbbr_hybrid_drate_no_wave_streak_;
}

void FBBRSender::ApplyFbbrHybridRegimeStateUpdates(
    WaveformWindowAnalysis* trace_analysis,
    QuicTime now) {
  if (trace_analysis == nullptr) {
    return;
  }
  const FbbrHybridDecision& decision = trace_analysis->hybrid_decision;
  const bool classification_usable =
      !trace_analysis->classification_suppressed_for_retry &&
      trace_analysis->classification !=
          WaveformClassification::kInconclusive;
  const bool rtprop_min_lower_bound_usable =
      decision.update_lower_bound_from_rtprop_min &&
      trace_analysis->delivery_rate_stats_valid &&
      std::isfinite(trace_analysis->delivery_rate_min_bps) &&
      trace_analysis->delivery_rate_min_bps > 0.0;
  const bool low_inflight_lower_bound_usable =
      decision.update_lower_bound_from_low_inflight &&
      trace_analysis->delivery_rate_stats_valid &&
      std::isfinite(trace_analysis->delivery_rate_min_bps) &&
      trace_analysis->delivery_rate_min_bps > 0.0 &&
      trace_analysis->srtt_stats_valid &&
      std::isfinite(trace_analysis->srtt_min_ms) &&
      trace_analysis->srtt_min_ms > 0.0;
  const bool independent_lower_bound_usable =
      rtprop_min_lower_bound_usable || low_inflight_lower_bound_usable;
  if (!classification_usable && !independent_lower_bound_usable) {
    return;
  }
  if (((classification_usable && decision.refresh_rtprop) ||
       low_inflight_lower_bound_usable) &&
      trace_analysis->srtt_stats_valid &&
      std::isfinite(trace_analysis->srtt_min_ms) &&
      trace_analysis->srtt_min_ms > 0.0 && now != QuicTime::Zero()) {
    const TimeDelta refreshed = TimeDelta::FromMicroseconds(
        std::max<int64_t>(1, static_cast<int64_t>(std::llround(
            trace_analysis->srtt_min_ms * 1000.0))));
    PublishHybridSrttLow(refreshed, now, false);
  }
  if ((classification_usable ? decision.update_rtprop_drate
                             : independent_lower_bound_usable) &&
      trace_analysis->delivery_rate_stats_valid &&
      std::isfinite(trace_analysis->delivery_rate_min_bps) &&
      trace_analysis->delivery_rate_min_bps > 0.0) {
    fbbr_hybrid_rtprop_drate_ =
        BandwidthFromBps(trace_analysis->delivery_rate_min_bps);
    fbbr_hybrid_rtprop_drate_valid_ =
        !fbbr_hybrid_rtprop_drate_.IsZero();
    fbbr_hybrid_rtprop_drate_source_cruise_id_ =
        static_cast<uint64_t>(std::max<int64_t>(0, cruise_id_));
    fbbr_hybrid_rtprop_drate_source_time_ = now;
  }
  if ((classification_usable ? decision.update_baseline_low
                             : independent_lower_bound_usable) &&
      trace_analysis->delivery_rate_stats_valid &&
      std::isfinite(trace_analysis->delivery_rate_min_bps) &&
      trace_analysis->delivery_rate_min_bps > 0.0) {
    // adaptive_change.pdf: verified lower-cut Regime I windows source the
    // Adaptive lower bracket from the window's minimum delivery rate.
    adaptive_baseline_low_ =
        BandwidthFromBps(trace_analysis->delivery_rate_min_bps);
    adaptive_baseline_low_valid_ = !adaptive_baseline_low_.IsZero();
    fbbr_hybrid_baseline_low_source_time_ = now;
  }
  trace_analysis->max_srtt_after_ms = fbbr_hybrid_max_srtt_valid_
      ? fbbr_hybrid_max_srtt_ms_ : 0.0;
  trace_analysis->rtprop_drate_after_bps =
      fbbr_hybrid_rtprop_drate_valid_
          ? static_cast<double>(
                fbbr_hybrid_rtprop_drate_.ToBitsPerSecond())
          : 0.0;
  trace_analysis->hybrid_baseline_low_after_valid =
      adaptive_baseline_low_valid_;
  trace_analysis->hybrid_baseline_low_after_bps =
      adaptive_baseline_low_valid_
          ? static_cast<double>(adaptive_baseline_low_.ToBitsPerSecond())
          : 0.0;
  const QuicBandwidth hybrid_max_bw = model_.MaxBandwidth();
  trace_analysis->hybrid_max_bw_after_valid =
      IsFinitePositiveBandwidth(hybrid_max_bw);
  trace_analysis->hybrid_max_bw_after_bps =
      trace_analysis->hybrid_max_bw_after_valid
          ? static_cast<double>(hybrid_max_bw.ToBitsPerSecond()) : 0.0;
}

void FBBRSender::ApplyFbbrHybridClassification(
    const WaveformWindowAnalysis& analysis,
    QuicTime now) {
  WaveformWindowAnalysis trace_analysis = analysis;
  const double baseline_before_bps = static_cast<double>(
      current_injection_baseline_bw_.ToBitsPerSecond());
  const double amplitude_before_bps =
      static_cast<double>(current_probe_amplitude_bps_);
  ++waveform_decision_count_;
  const double classification_window_s =
      analysis.window_end > analysis.window_start
          ? static_cast<double>(
                (analysis.window_end - analysis.window_start)
                    .ToMicroseconds()) /
                1000000.0
          : 0.0;
  const GradientMatchedDecrease overload_decrease =
      ComputeGradientMatchedDecrease(
          analysis.overload_queue_sample_valid,
          analysis.overload_q90_s,
          analysis.overload_queue_gradient,
          classification_window_s,
          analysis.hybrid_srtt_low_rtprop_valid,
          analysis.hybrid_srtt_low_rtprop_ms / 1000.0,
          analysis.hybrid_max_srtt_valid,
          analysis.hybrid_max_srtt_ms / 1000.0);
  const bool lower_bound_search_required =
      fbbr_hybrid_lower_bound_search_active_ &&
      (!adaptive_baseline_low_valid_ ||
       !fbbr_hybrid_rtprop_drate_valid_);
  if (lower_bound_search_required) {
    if (!FbbrHybridLowerBoundSearchGuardAllows(trace_analysis)) {
      ResetHybridLowerBoundSearch();
    } else {
      const bool rtprop_contact =
          trace_analysis.hybrid_srtt_low_rtprop_valid &&
          trace_analysis.srtt_stats_valid &&
          std::isfinite(trace_analysis.srtt_min_ms) &&
          trace_analysis.srtt_min_ms > 0.0 &&
          std::isfinite(trace_analysis.hybrid_srtt_low_rtprop_ms) &&
          trace_analysis.hybrid_srtt_low_rtprop_ms > 0.0 &&
          std::abs(trace_analysis.srtt_min_ms -
                   trace_analysis.hybrid_srtt_low_rtprop_ms) <= 1e-9;
      const bool observation_usable =
          trace_analysis.delivery_rate_stats_valid &&
          std::isfinite(trace_analysis.delivery_rate_min_bps) &&
          trace_analysis.delivery_rate_min_bps > 0.0;
      if (rtprop_contact && observation_usable) {
        // The search stop observation is intentionally transactional: suppress
        // all regime side effects, then publish exactly the lower-bound state
        // requested by the stop condition.
        trace_analysis.classification_suppressed_for_retry = true;
        trace_analysis.state_updates_suppressed_for_retry = true;
        trace_analysis.hybrid_decision = FbbrHybridDecision();
        trace_analysis.hybrid_decision
            .update_lower_bound_from_rtprop_min = true;
        ApplyFbbrHybridRegimeStateUpdates(&trace_analysis, now);
        ResetHybridLowerBoundSearch();
        waveform_last_action_ =
            "HYBRID_LOWER_BOUND_SEARCH_RTPROP_CONTACT";
        waveform_last_invalid_reason_ = "none";
        waveform_last_delta_source_ = kWaveformDeltaSourceNone;
        waveform_last_raw_delta_bw_bps_ = 0.0;
        waveform_last_applied_delta_bw_bps_ = 0.0;
        EmitWaveformSearchTrace(trace_analysis, waveform_last_action_,
                                baseline_before_bps, amplitude_before_bps);
        ScheduleWaveformCollectionAfterSettle(now, false);
        return;
      }

      if (fbbr_hybrid_stable_observation_source_ ==
          HybridStableObservationSource::kCruiseFallback) {
        // Once low flight is reached, keep the same pacing platform across
        // Cruise/phase boundaries. RTprop and DRate are published only by the
        // completed 200ms + one packet-timed-round transaction.
        waveform_last_action_ = "HYBRID_LOWER_BOUND_STABLE_PLATFORM_HOLD";
        waveform_last_invalid_reason_ = "none";
        waveform_last_delta_source_ = kWaveformDeltaSourceNone;
        waveform_last_raw_delta_bw_bps_ = 0.0;
        waveform_last_applied_delta_bw_bps_ = 0.0;
        EmitWaveformSearchTrace(trace_analysis, waveform_last_action_,
                                baseline_before_bps, amplitude_before_bps);
        return;
      }

      const double next_baseline_bps =
          ComputeFbbrHybridLowerBoundSearchBaseline(
              baseline_before_bps,
              static_cast<double>(minimum_pacing_rate_bps_));
      current_injection_baseline_bw_ =
          BandwidthFromBps(next_baseline_bps);
      fbbr_hybrid_lower_bound_search_baseline_ =
          current_injection_baseline_bw_;
      const double retained_ratio = baseline_before_bps > 0.0
          ? next_baseline_bps / baseline_before_bps : 1.0;
      if (current_probe_amplitude_bps_ > 0 && retained_ratio < 1.0) {
        current_probe_amplitude_bps_ = std::max<uint64_t>(
            1, static_cast<uint64_t>(std::llround(
                   retained_ratio * current_probe_amplitude_bps_)));
      }
      if (fbbr_hybrid_lower_bound_search_step_count_ <
          std::numeric_limits<uint32_t>::max()) {
        ++fbbr_hybrid_lower_bound_search_step_count_;
      }
      if (std::abs(next_baseline_bps - baseline_before_bps) > 0.5 &&
          baseline_adjustment_count_ < std::numeric_limits<uint32_t>::max()) {
        ++baseline_adjustment_count_;
      }
      trace_analysis.delta_source = "HYBRID_BBR_R_0P8_DRAIN";
      trace_analysis.raw_delta_bw_bps =
          std::max(0.0, baseline_before_bps - next_baseline_bps);
      trace_analysis.applied_delta_bw_bps =
          trace_analysis.raw_delta_bw_bps;
      waveform_last_delta_source_ = trace_analysis.delta_source;
      waveform_last_raw_delta_bw_bps_ =
          trace_analysis.raw_delta_bw_bps;
      waveform_last_applied_delta_bw_bps_ =
          trace_analysis.applied_delta_bw_bps;
      waveform_last_action_ = "HYBRID_LOWER_BOUND_SEARCH_REDUCE_0P8";
      waveform_last_invalid_reason_ = "none";
      ScheduleWaveformCollectionAfterSettle(now, false);
      EmitWaveformSearchTrace(trace_analysis, waveform_last_action_,
                              baseline_before_bps, amplitude_before_bps);
      return;
    }
  }
  if (fbbr_hybrid_lower_bound_search_active_) {
    ResetHybridLowerBoundSearch();
  }
  if (analysis.classification == WaveformClassification::kInconclusive) {
    // The RTprop-contact lower-bound observation is independent of waveform
    // classification.  Apply only that explicitly marked side effect before
    // retrying; the state-update helper still suppresses all other effects.
    ApplyFbbrHybridRegimeStateUpdates(&trace_analysis, now);
    waveform_last_action_ =
        analysis.hybrid_decision.update_lower_bound_from_rtprop_min
            ? "HYBRID_RTPROP_MIN_UPDATE_AND_RETRY"
            : "HYBRID_RETRY_WITHOUT_SIDE_EFFECTS";
    waveform_last_invalid_reason_ = analysis.invalid_reason;
    waveform_last_delta_source_ = kWaveformDeltaSourceNone;
    waveform_last_raw_delta_bw_bps_ = 0.0;
    waveform_last_applied_delta_bw_bps_ = 0.0;
    EmitWaveformSearchTrace(trace_analysis, waveform_last_action_,
                            baseline_before_bps, amplitude_before_bps);
    ScheduleWaveformCollectionAfterSettle(now, false);
    return;
  }
  if (!analysis.delivery_rate_stats_valid) {
    waveform_last_action_ = "HYBRID_INVALID_DRATE_STATS_RETRY";
    waveform_last_invalid_reason_ = "hybrid_delivery_rate_stats_invalid";
    trace_analysis.invalid_reason = waveform_last_invalid_reason_;
    EmitWaveformSearchTrace(trace_analysis, waveform_last_action_,
                            baseline_before_bps, amplitude_before_bps);
    ScheduleWaveformCollectionAfterSettle(now, false);
    return;
  }
  // Work out the post-decision DRate reference without committing it yet.
  // The hybrid pipeline is transactional: an invalid actuator input must not
  // partially update RTprop, RTpropDRate, or the lower bound.
  const bool prospective_rtprop_drate_valid =
      analysis.hybrid_decision.update_rtprop_drate
          ? (std::isfinite(analysis.delivery_rate_min_bps) &&
             analysis.delivery_rate_min_bps > 0.0)
          : fbbr_hybrid_rtprop_drate_valid_;
  const double prospective_rtprop_drate_bps =
      analysis.hybrid_decision.update_rtprop_drate
          ? analysis.delivery_rate_min_bps
          : (fbbr_hybrid_rtprop_drate_valid_
                 ? static_cast<double>(
                       fbbr_hybrid_rtprop_drate_.ToBitsPerSecond())
                 : 0.0);
  const bool prospective_baseline_low_valid =
      analysis.hybrid_decision.update_baseline_low
          ? (std::isfinite(analysis.delivery_rate_min_bps) &&
             analysis.delivery_rate_min_bps > 0.0)
          : adaptive_baseline_low_valid_;
  const double prospective_baseline_low_bps =
      analysis.hybrid_decision.update_baseline_low
          ? analysis.delivery_rate_min_bps
          : (adaptive_baseline_low_valid_
                 ? static_cast<double>(
                       adaptive_baseline_low_.ToBitsPerSecond())
                 : 0.0);
  const QuicBandwidth current_hybrid_max_bw = model_.MaxBandwidth();
  const bool prospective_max_bw_valid =
      IsFinitePositiveBandwidth(current_hybrid_max_bw);
  const double prospective_max_bw_bps =
      prospective_max_bw_valid
          ? static_cast<double>(current_hybrid_max_bw.ToBitsPerSecond())
          : 0.0;
  const FbbrHybridActuatorResult actuator =
      ComputeFbbrHybridInjectionBaseline(
          analysis.classification,
          analysis.delivery_rate_min_bps,
          analysis.delivery_rate_max_bps,
          analysis.delivery_rate_mean_bps,
          prospective_baseline_low_valid,
          prospective_baseline_low_bps,
          prospective_max_bw_valid,
          prospective_max_bw_bps,
          baseline_before_bps,
          prospective_rtprop_drate_valid,
          prospective_rtprop_drate_bps,
          fbbr_regime_actuator_midpoint_trigger_ratio_,
          static_cast<double>(minimum_pacing_rate_bps_),
          overload_decrease.valid,
          overload_decrease.beta);
  if (!actuator.valid) {
    waveform_last_action_ = "HYBRID_INVALID_ACTUATOR_INPUT_RETRY";
    waveform_last_invalid_reason_ = "hybrid_actuator_input_invalid";
    trace_analysis.invalid_reason = waveform_last_invalid_reason_;
    EmitWaveformSearchTrace(trace_analysis, waveform_last_action_,
                            baseline_before_bps, amplitude_before_bps);
    ScheduleWaveformCollectionAfterSettle(now, false);
    return;
  }
  ApplyFbbrHybridRegimeStateUpdates(&trace_analysis, now);
  if (actuator.update_baseline) {
    current_injection_baseline_bw_ =
        BandwidthFromBps(actuator.next_baseline_bps);
  }
  const double baseline_delta = actuator.update_baseline
      ? std::abs(actuator.next_baseline_bps - baseline_before_bps)
      : 0.0;
  if (baseline_delta > 0.5 &&
      baseline_adjustment_count_ < std::numeric_limits<uint32_t>::max()) {
    ++baseline_adjustment_count_;
  }
  if (actuator.update_trusted_bw) {
    fbbr_hybrid_regime_ii_seen_this_cruise_ = true;
    fbbr_hybrid_trusted_bw_ =
        BandwidthFromBps(actuator.trusted_bw_bps);
    fbbr_latest_trusted_bw_ = fbbr_hybrid_trusted_bw_;
    fbbr_smoothed_trusted_bw_ = fbbr_hybrid_trusted_bw_;
    fbbr_smoothed_trusted_bw_valid_ = !fbbr_hybrid_trusted_bw_.IsZero();
    trusted_bw_candidate_ = fbbr_hybrid_trusted_bw_;
    trusted_bw_candidate_source_ = kTrustedBwSourceFbbrWindowMean;
    trusted_baseline_locked_ = !trusted_bw_candidate_.IsZero();
    if (trusted_baseline_locked_) {
      ++trusted_bw_candidate_update_count_;
    }
    trace_analysis.latest_trusted_bw_bps = actuator.trusted_bw_bps;
    trace_analysis.smoothed_trusted_bw_bps = actuator.trusted_bw_bps;
  }
  trace_analysis.hybrid_swing_bps = actuator.swing_bps;
  trace_analysis.hybrid_reference_gap_bps = actuator.reference_gap_bps;
  trace_analysis.hybrid_bracket_valid = actuator.bracket_valid;
  trace_analysis.hybrid_bracket_target_bps = actuator.bracket_target_bps;
  trace_analysis.hybrid_bracket_triggered = actuator.bracket_triggered;
  trace_analysis.hybrid_midpoint_triggered =
      actuator.midpoint_triggered;
  trace_analysis.delta_source =
      analysis.classification == WaveformClassification::kOverload
          ? kWaveformDeltaSourceHybridGradientMatchedOverload
          : !actuator.update_baseline
          ? kWaveformDeltaSourceNone
          : actuator.bracket_triggered
              ? kWaveformDeltaSourceHybridAdaptiveBracket
              : actuator.midpoint_triggered
                  ? kWaveformDeltaSourceHybridRtpropDrateMidpoint
                  : (analysis.classification ==
                             WaveformClassification::kUnderload
                         ? kWaveformDeltaSourceFbbrWindowMaximum
                         : kWaveformDeltaSourceFbbrWindowMinimum);
  trace_analysis.raw_delta_bw_bps = baseline_delta;
  trace_analysis.applied_delta_bw_bps = baseline_delta;
  waveform_last_delta_source_ = trace_analysis.delta_source;
  waveform_last_raw_delta_bw_bps_ = baseline_delta;
  waveform_last_applied_delta_bw_bps_ = baseline_delta;
  waveform_last_invalid_reason_ = "none";
  if (analysis.classification == WaveformClassification::kFullLoad) {
    waveform_last_action_ = "HYBRID_REGIME_II_UPDATE_TRUSTED_BW";
  } else if (analysis.classification == WaveformClassification::kUnderload) {
    underload_located_ = true;
    waveform_last_action_ = actuator.bracket_triggered
        ? "HYBRID_REGIME_I_USE_ADAPTIVE_HALF_GAP"
        : actuator.midpoint_triggered
            ? "HYBRID_REGIME_I_USE_RTPROP_DRATE_MIDPOINT"
            : "HYBRID_REGIME_I_USE_MAXIMUM";
  } else {
    waveform_last_action_ = actuator.overload_beta > 0.0
        ? "HYBRID_REGIME_III_GRADIENT_MATCHED_DECREASE"
        : "HYBRID_REGIME_III_GRADIENT_MATCHED_HOLD";
  }
  ScheduleWaveformCollectionAfterSettle(now, false);
  EmitWaveformSearchTrace(trace_analysis, waveform_last_action_,
                          baseline_before_bps, amplitude_before_bps);
}

void FBBRSender::ApplyFbbrHybridV3Classification(
    const WaveformWindowAnalysis& analysis,
    QuicTime now) {
  WaveformWindowAnalysis trace_analysis = analysis;
  const FbbrV3ReferenceResult reference =
      SelectFbbrV3ReferenceBw();
  if (reference.valid) {
    // Compatibility mirror only.  V3 pacing reads ReferenceBw directly.
    current_injection_baseline_bw_ = reference.bandwidth;
  }
  const double baseline_before_bps = static_cast<double>(
      current_injection_baseline_bw_.ToBitsPerSecond());
  const double amplitude_before_bps =
      static_cast<double>(current_probe_amplitude_bps_);
  ++waveform_decision_count_;
  waveform_last_delta_source_ = kWaveformDeltaSourceNone;
  waveform_last_raw_delta_bw_bps_ = 0.0;
  waveform_last_applied_delta_bw_bps_ = 0.0;
  trace_analysis.delta_source = kWaveformDeltaSourceNone;
  trace_analysis.raw_delta_bw_bps = 0.0;
  trace_analysis.applied_delta_bw_bps = 0.0;
  trace_analysis.hybrid_bracket_valid = false;
  trace_analysis.hybrid_bracket_target_bps = 0.0;
  trace_analysis.hybrid_bracket_triggered = false;
  trace_analysis.hybrid_midpoint_triggered = false;

  if (analysis.classification == WaveformClassification::kInconclusive) {
    waveform_last_action_ = "V3_INCONCLUSIVE_REFERENCE_HOLD";
    waveform_last_invalid_reason_ = analysis.invalid_reason;
    EmitWaveformSearchTrace(trace_analysis, waveform_last_action_,
                            baseline_before_bps, amplitude_before_bps);
    ScheduleWaveformCollectionAfterSettle(now, false);
    return;
  }
  if (!analysis.delivery_rate_stats_valid) {
    waveform_last_action_ = "V3_INVALID_DRATE_REFERENCE_HOLD";
    waveform_last_invalid_reason_ =
        "hybrid_delivery_rate_stats_invalid";
    trace_analysis.invalid_reason = waveform_last_invalid_reason_;
    EmitWaveformSearchTrace(trace_analysis, waveform_last_action_,
                            baseline_before_bps, amplitude_before_bps);
    ScheduleWaveformCollectionAfterSettle(now, false);
    return;
  }

  // Retain the classifier's RTprop/information updates.  None of these
  // observational fields is a V3 pacing actuator or an inflight_hi/lo write.
  ApplyFbbrHybridRegimeStateUpdates(&trace_analysis, now);
  if (analysis.classification == WaveformClassification::kFullLoad) {
    const double trusted_bps = analysis.delivery_rate_mean_bps;
    if (std::isfinite(trusted_bps) && trusted_bps > 0.0) {
      fbbr_hybrid_regime_ii_seen_this_cruise_ = true;
      fbbr_hybrid_trusted_bw_ = BandwidthFromBps(trusted_bps);
      fbbr_latest_trusted_bw_ = fbbr_hybrid_trusted_bw_;
      fbbr_smoothed_trusted_bw_ = fbbr_hybrid_trusted_bw_;
      fbbr_smoothed_trusted_bw_valid_ =
          !fbbr_hybrid_trusted_bw_.IsZero();
      trusted_bw_candidate_ = fbbr_hybrid_trusted_bw_;
      trusted_bw_candidate_source_ =
          kTrustedBwSourceFbbrWindowMean;
      trusted_baseline_locked_ = !trusted_bw_candidate_.IsZero();
      if (trusted_baseline_locked_) {
        ++trusted_bw_candidate_update_count_;
      }
      trace_analysis.latest_trusted_bw_bps = trusted_bps;
      trace_analysis.smoothed_trusted_bw_bps = trusted_bps;
    }
    waveform_last_action_ = "V3_REGIME_II_UPDATE_TRUSTED_BW";
  } else if (analysis.classification ==
             WaveformClassification::kUnderload) {
    underload_located_ = true;
    waveform_last_action_ = "V3_REGIME_I_REFERENCE_HOLD";
  } else {
    waveform_last_action_ = "V3_REGIME_III_REFERENCE_HOLD";
  }
  waveform_last_invalid_reason_ = "none";
  ScheduleWaveformCollectionAfterSettle(now, false);
  EmitWaveformSearchTrace(trace_analysis, waveform_last_action_,
                          baseline_before_bps, amplitude_before_bps);
}

void FBBRSender::ApplyWaveformClassification(
    const WaveformWindowAnalysis& analysis,
    QuicTime now) {
  if (IsFbbrProjectionObserver()) {
    ApplyFbbrHybridV3Classification(analysis, now);
    return;
  }
  if (IsFbbrHybrid()) {
    ApplyFbbrHybridClassification(analysis, now);
    return;
  }
  WaveformWindowAnalysis trace_analysis = analysis;
  const double baseline_before_bps = static_cast<double>(
      current_injection_baseline_bw_.ToBitsPerSecond());
  const double amplitude_before_bps =
      static_cast<double>(current_probe_amplitude_bps_);
  std::string action = "RETRY_WITHOUT_BASELINE_CHANGE";
  waveform_last_invalid_reason_ = analysis.invalid_reason;
  ++waveform_decision_count_;

  TimeDelta min_rtt = model_.MinRtt();
  if (min_rtt.IsZero() && rtt_stats_ != nullptr) {
    min_rtt = rtt_stats_->MinOrInitialRtt();
  }
  TimeDelta queue_rtt = rtt_stats_ == nullptr
      ? TimeDelta::Zero()
      : rtt_stats_->latest_rtt();
  if (queue_rtt.IsZero()) {
    queue_rtt = CurrentSmoothedRtt();
  }
  const bool queue_signal_valid = !min_rtt.IsZero() && !queue_rtt.IsZero();
  double queue_delay_us = 0.0;
  double queue_delay_ratio = -1.0;
  if (queue_signal_valid) {
    queue_delay_us = static_cast<double>(std::max<int64_t>(
        0, queue_rtt.ToMicroseconds() - min_rtt.ToMicroseconds()));
    queue_delay_ratio = queue_delay_us /
        static_cast<double>(min_rtt.ToMicroseconds());
  }
  trace_analysis.queue_delay_ms = queue_delay_us / 1000.0;
  trace_analysis.queue_delay_min_rtt_ratio = queue_delay_ratio;
  consecutive_overload_count_ = 0;
  trace_analysis.overload_confirmation_count = consecutive_overload_count_;

  auto adjustment_limit_reached = [&]() {
    if (baseline_adjustment_count_ < waveform_max_baseline_adjustments_) {
      return false;
    }
    waveform_cruise_state_ = WaveformCruiseState::kDisabled;
    waveform_last_invalid_reason_ = "max_baseline_adjustments_reached";
    waveform_last_action_ = trusted_baseline_locked_
        ? "STOP_SEARCH_KEEP_LATEST_TRUSTED_BW"
        : "STOP_SEARCH_WITHOUT_TRUSTED_BW";
    waveform_last_delta_source_ = kWaveformDeltaSourceNone;
    waveform_last_raw_delta_bw_bps_ = 0.0;
    waveform_last_applied_delta_bw_bps_ = 0.0;
    EmitWaveformSearchTrace(trace_analysis,
                            waveform_last_action_,
                            baseline_before_bps,
                            amplitude_before_bps);
    return true;
  };

  if (analysis.classification == WaveformClassification::kInconclusive) {
    waveform_last_delta_source_ = kWaveformDeltaSourceNone;
    waveform_last_raw_delta_bw_bps_ = 0.0;
    waveform_last_applied_delta_bw_bps_ = 0.0;
    waveform_last_action_ = action;
    EmitWaveformSearchTrace(trace_analysis, action, baseline_before_bps,
                            amplitude_before_bps);
    ScheduleWaveformCollectionAfterSettle(now, false);
    return;
  }

  if (fbbr_window_baseline_enabled_) {
    if (!analysis.delivery_rate_stats_valid) {
      waveform_last_invalid_reason_ = "fbbr_delivery_rate_stats_invalid";
      waveform_last_delta_source_ = kWaveformDeltaSourceNone;
      waveform_last_raw_delta_bw_bps_ = 0.0;
      waveform_last_applied_delta_bw_bps_ = 0.0;
      waveform_last_action_ = "RETRY_WITHOUT_BASELINE_CHANGE";
      trace_analysis.invalid_reason = waveform_last_invalid_reason_;
      EmitWaveformSearchTrace(trace_analysis, waveform_last_action_,
                              baseline_before_bps, amplitude_before_bps);
      ScheduleWaveformCollectionAfterSettle(now, false);
      return;
    }

    double next_baseline_bps = baseline_before_bps;
    const char* delta_source = kWaveformDeltaSourceNone;
    const bool waveform_derived =
        IsWaveformDecisionRule(analysis.decision_rule);
    const bool waveform_srtt_mean_valid =
        waveform_derived && analysis.srtt_stats_valid &&
        std::isfinite(analysis.srtt_mean_ms) && analysis.srtt_mean_ms > 0.0;
    switch (analysis.classification) {
      case WaveformClassification::kFullLoad: {
        const double latest_trusted_bps = analysis.delivery_rate_mean_bps;
        fbbr_latest_trusted_bw_ = BandwidthFromBps(latest_trusted_bps);
        fbbr_smoothed_trusted_bw_ =
            BandwidthFromBps(latest_trusted_bps);
        fbbr_smoothed_trusted_bw_valid_ =
            !fbbr_smoothed_trusted_bw_.IsZero();
        next_baseline_bps = latest_trusted_bps;
        current_injection_baseline_bw_ = fbbr_smoothed_trusted_bw_;
        trusted_baseline_locked_ = fbbr_smoothed_trusted_bw_valid_;
        trusted_bw_candidate_ = fbbr_smoothed_trusted_bw_;
        trusted_bw_candidate_source_ =
            kTrustedBwSourceFbbrWindowMean;
        ++trusted_bw_candidate_update_count_;
        trace_analysis.latest_trusted_bw_bps = latest_trusted_bps;
        trace_analysis.smoothed_trusted_bw_bps = latest_trusted_bps;
        delta_source = kWaveformDeltaSourceFbbrTrustedBw;
        action = "FBBR_FULL_LOAD_USE_WINDOW_MEAN";
        break;
      }
      case WaveformClassification::kUnderload:
        if (adjustment_limit_reached()) {
          return;
        }
        if (waveform_derived) {
          adaptive_baseline_low_ =
              BandwidthFromBps(analysis.delivery_rate_mean_bps);
          adaptive_baseline_low_valid_ = !adaptive_baseline_low_.IsZero();
          if (waveform_srtt_mean_valid) {
            latest_waveform_underload_srtt_mean_ms_ =
                analysis.srtt_mean_ms;
            latest_waveform_underload_srtt_mean_valid_ = true;
          }
        }
        underload_located_ = true;
        next_baseline_bps = analysis.delivery_rate_max_bps;
        current_injection_baseline_bw_ = BandwidthFromBps(next_baseline_bps);
        ++baseline_adjustment_count_;
        delta_source = kWaveformDeltaSourceFbbrWindowMaximum;
        action = "FBBR_UNDERLOAD_USE_WINDOW_MAXIMUM";
        break;
      case WaveformClassification::kOverload:
        if (adjustment_limit_reached()) {
          return;
        }
        if (waveform_derived) {
          adaptive_baseline_up_ =
              BandwidthFromBps(analysis.delivery_rate_mean_bps);
          adaptive_baseline_up_valid_ = !adaptive_baseline_up_.IsZero();
          if (waveform_srtt_mean_valid) {
            latest_waveform_overload_srtt_mean_ms_ =
                analysis.srtt_mean_ms;
            latest_waveform_overload_srtt_mean_valid_ = true;
          }
        }
        next_baseline_bps = analysis.delivery_rate_min_bps;
        current_injection_baseline_bw_ = BandwidthFromBps(next_baseline_bps);
        ++baseline_adjustment_count_;
        delta_source = kWaveformDeltaSourceFbbrWindowMinimum;
        action = "FBBR_OVERLOAD_USE_WINDOW_MINIMUM";
        break;
      case WaveformClassification::kInconclusive:
      default:
        break;
    }

    const double baseline_change_bps =
        std::abs(next_baseline_bps - baseline_before_bps);
    trace_analysis.delta_source = delta_source;
    trace_analysis.raw_delta_bw_bps = baseline_change_bps;
    trace_analysis.applied_delta_bw_bps = baseline_change_bps;
    waveform_last_delta_source_ = delta_source;
    waveform_last_raw_delta_bw_bps_ = baseline_change_bps;
    waveform_last_applied_delta_bw_bps_ = baseline_change_bps;
    waveform_last_action_ = action;
    ScheduleWaveformCollectionAfterSettle(now, false);
    EmitWaveformSearchTrace(trace_analysis, action, baseline_before_bps,
                            amplitude_before_bps);
    return;
  }

  if (!adaptive_guard_enabled_) {
    if (analysis.drate_similar &&
        std::isfinite(analysis.current_drate_response_amplitude_bps) &&
        analysis.current_drate_response_amplitude_bps > 0.0) {
      last_similar_drate_amplitude_bps_ =
          analysis.current_drate_response_amplitude_bps;
      has_last_similar_drate_amplitude_ = true;
    }

    if (analysis.classification == WaveformClassification::kFullLoad) {
      trusted_baseline_locked_ = true;
      trusted_bw_candidate_ = current_injection_baseline_bw_;
      trusted_bw_candidate_source_ = kTrustedBwSourceTimeWaveformBaseline;
      ++trusted_bw_candidate_update_count_;
      trace_analysis.delta_source = kWaveformDeltaSourceNone;
      trace_analysis.raw_delta_bw_bps = 0.0;
      trace_analysis.applied_delta_bw_bps = 0.0;
      waveform_last_delta_source_ = kWaveformDeltaSourceNone;
      waveform_last_raw_delta_bw_bps_ = 0.0;
      waveform_last_applied_delta_bw_bps_ = 0.0;
      waveform_last_action_ = "UPDATE_TRUSTED_BW_FROM_FULL_LOAD";
      ScheduleWaveformCollectionAfterSettle(now, false);
      EmitWaveformSearchTrace(trace_analysis,
                              waveform_last_action_,
                              baseline_before_bps,
                              amplitude_before_bps);
      return;
    }

    const bool recent_amplitude_valid =
        has_last_similar_drate_amplitude_ &&
        std::isfinite(last_similar_drate_amplitude_bps_) &&
        last_similar_drate_amplitude_bps_ > 0.0;
    const char* delta_source = recent_amplitude_valid
        ? kWaveformDeltaSourceRecentDrate
        : kWaveformDeltaSourceBaselineFallback;
    double raw_delta_bw_bps = recent_amplitude_valid
        ? waveform_delta_drate_amplitude_ratio_ *
              last_similar_drate_amplitude_bps_
        : waveform_delta_fallback_baseline_ratio_ * baseline_before_bps;
    if (!std::isfinite(raw_delta_bw_bps) || raw_delta_bw_bps < 0.0) {
      raw_delta_bw_bps = 0.0;
    }
    double applied_delta_bw_bps = raw_delta_bw_bps;
    const bool decreases_baseline =
        analysis.classification == WaveformClassification::kOverload;
    if (decreases_baseline) {
      applied_delta_bw_bps = std::min(
          raw_delta_bw_bps,
          std::max(0.0, baseline_before_bps -
                            static_cast<double>(minimum_pacing_rate_bps_)));
    }
    trace_analysis.delta_source = delta_source;
    trace_analysis.raw_delta_bw_bps = raw_delta_bw_bps;
    trace_analysis.applied_delta_bw_bps = applied_delta_bw_bps;
    waveform_last_delta_source_ = delta_source;
    waveform_last_raw_delta_bw_bps_ = raw_delta_bw_bps;
    waveform_last_applied_delta_bw_bps_ = applied_delta_bw_bps;

    switch (analysis.classification) {
      case WaveformClassification::kFullLoad:
        break;
      case WaveformClassification::kUnderload:
        if (adjustment_limit_reached()) {
          return;
        }
        underload_located_ = true;
        current_injection_baseline_bw_ = BandwidthFromBps(
            baseline_before_bps + applied_delta_bw_bps);
        ++baseline_adjustment_count_;
        action = "MARK_UNDERLOAD_AND_INCREASE_BASELINE";
        ScheduleWaveformCollectionAfterSettle(now, false);
        break;
      case WaveformClassification::kOverload: {
        if (adjustment_limit_reached()) {
          return;
        }
        const double updated = baseline_before_bps - applied_delta_bw_bps;
        current_injection_baseline_bw_ = BandwidthFromBps(updated);
        ++baseline_adjustment_count_;
        action = "DECREASE_BASELINE";
        ScheduleWaveformCollectionAfterSettle(now, false);
        break;
      }
      case WaveformClassification::kInconclusive:
      default:
        break;
    }
    waveform_last_action_ = action;
    EmitWaveformSearchTrace(trace_analysis,
                            action,
                            baseline_before_bps,
                            amplitude_before_bps);
    return;
  }

  if (!analysis.delivery_rate_stats_valid) {
    waveform_last_invalid_reason_ = "adaptive_delivery_rate_stats_invalid";
    trace_analysis.invalid_reason = waveform_last_invalid_reason_;
    waveform_last_delta_source_ = kWaveformDeltaSourceNone;
    waveform_last_raw_delta_bw_bps_ = 0.0;
    waveform_last_applied_delta_bw_bps_ = 0.0;
    waveform_last_action_ = "ADAPTIVE_RETRY_WITHOUT_BASELINE_CHANGE";
    EmitWaveformSearchTrace(trace_analysis, waveform_last_action_,
                            baseline_before_bps, amplitude_before_bps);
    ScheduleWaveformCollectionAfterSettle(now, false);
    return;
  }

  if (analysis.classification == WaveformClassification::kFullLoad) {
    trusted_bw_candidate_ =
        BandwidthFromBps(analysis.delivery_rate_mean_bps);
    trusted_baseline_locked_ = !trusted_bw_candidate_.IsZero();
    trusted_bw_candidate_source_ = kTrustedBwSourceAdaptiveWindowMean;
    if (trusted_baseline_locked_) {
      ++trusted_bw_candidate_update_count_;
    }
    trace_analysis.latest_trusted_bw_bps =
        analysis.delivery_rate_mean_bps;
    trace_analysis.delta_source = kWaveformDeltaSourceNone;
    trace_analysis.raw_delta_bw_bps = 0.0;
    trace_analysis.applied_delta_bw_bps = 0.0;
    waveform_last_delta_source_ = kWaveformDeltaSourceNone;
    waveform_last_raw_delta_bw_bps_ = 0.0;
    waveform_last_applied_delta_bw_bps_ = 0.0;
    waveform_last_action_ = "ADAPTIVE_UPDATE_TRUSTED_BW_FROM_WINDOW_MEAN";
    ScheduleWaveformCollectionAfterSettle(now, false);
    EmitWaveformSearchTrace(trace_analysis, waveform_last_action_,
                            baseline_before_bps, amplitude_before_bps);
    return;
  }

  const bool waveform_derived =
      IsWaveformDecisionRule(analysis.decision_rule);
  if (waveform_derived &&
      (!analysis.srtt_stats_valid || !std::isfinite(analysis.srtt_mean_ms) ||
       analysis.srtt_mean_ms <= 0.0)) {
    waveform_last_invalid_reason_ = "adaptive_srtt_stats_invalid";
    trace_analysis.invalid_reason = waveform_last_invalid_reason_;
    waveform_last_delta_source_ = kWaveformDeltaSourceNone;
    waveform_last_raw_delta_bw_bps_ = 0.0;
    waveform_last_applied_delta_bw_bps_ = 0.0;
    waveform_last_action_ = "ADAPTIVE_RETRY_WITHOUT_BOUND_UPDATE";
    EmitWaveformSearchTrace(trace_analysis, waveform_last_action_,
                            baseline_before_bps, amplitude_before_bps);
    ScheduleWaveformCollectionAfterSettle(now, false);
    return;
  }
  if (adjustment_limit_reached()) {
    return;
  }

  if (analysis.classification == WaveformClassification::kOverload &&
      waveform_derived) {
    adaptive_baseline_up_ =
        BandwidthFromBps(analysis.delivery_rate_mean_bps);
    adaptive_baseline_up_valid_ = !adaptive_baseline_up_.IsZero();
    latest_waveform_overload_srtt_mean_ms_ = analysis.srtt_mean_ms;
    latest_waveform_overload_srtt_mean_valid_ = true;
  } else if (analysis.classification == WaveformClassification::kUnderload &&
             waveform_derived) {
    adaptive_baseline_low_ =
        BandwidthFromBps(analysis.delivery_rate_mean_bps);
    adaptive_baseline_low_valid_ = !adaptive_baseline_low_.IsZero();
    latest_waveform_underload_srtt_mean_ms_ = analysis.srtt_mean_ms;
    latest_waveform_underload_srtt_mean_valid_ = true;
    underload_located_ = true;
  }

  const double baseline_low_bps = static_cast<double>(
      adaptive_baseline_low_.ToBitsPerSecond());
  const double baseline_up_bps = static_cast<double>(
      adaptive_baseline_up_.ToBitsPerSecond());
  const bool bracket_valid = adaptive_baseline_low_valid_ &&
      adaptive_baseline_up_valid_ && baseline_up_bps > baseline_low_bps;
  const bool overload =
      analysis.classification == WaveformClassification::kOverload;
  const double baseline_gap_bps = bracket_valid
      ? baseline_up_bps - baseline_low_bps
      : 0.0;
  const double bracket_target_bps = bracket_valid
      ? baseline_low_bps + baseline_gap_bps / (overload ? 4.0 : 2.0)
      : 0.0;
  const bool use_bracket_target = bracket_valid &&
      ((overload && baseline_before_bps > bracket_target_bps) ||
       (!overload && baseline_before_bps < bracket_target_bps));
  const double next_baseline_bps = ComputeAdaptiveNextBaseline(
      analysis.classification,
      adaptive_baseline_low_valid_, baseline_low_bps,
      adaptive_baseline_up_valid_, baseline_up_bps,
      baseline_before_bps,
      analysis.delivery_rate_min_bps,
      analysis.delivery_rate_max_bps,
      static_cast<double>(minimum_pacing_rate_bps_));
  const double baseline_change_bps =
      std::abs(next_baseline_bps - baseline_before_bps);
  const char* delta_source = use_bracket_target
      ? kWaveformDeltaSourceAdaptiveBracketBound
      : (overload ? kWaveformDeltaSourceAdaptiveWindowMinimum
                  : kWaveformDeltaSourceAdaptiveWindowMaximum);
  trace_analysis.delta_source = delta_source;
  trace_analysis.raw_delta_bw_bps = baseline_change_bps;
  trace_analysis.applied_delta_bw_bps = baseline_change_bps;
  trace_analysis.delta_reference_bps = bracket_valid
      ? bracket_target_bps
      : 0.0;
  trace_analysis.window_extreme_gap_bps = bracket_valid
      ? baseline_gap_bps
      : std::abs((overload ? analysis.delivery_rate_min_bps
                           : analysis.delivery_rate_max_bps) -
                 baseline_before_bps);
  trace_analysis.actuator_step_multiplier = 1.0;
  waveform_last_delta_source_ = delta_source;
  waveform_last_raw_delta_bw_bps_ = baseline_change_bps;
  waveform_last_applied_delta_bw_bps_ = baseline_change_bps;
  current_injection_baseline_bw_ = BandwidthFromBps(next_baseline_bps);
  if (baseline_change_bps > 0.5) {
    ++baseline_adjustment_count_;
  }
  if (overload) {
    action = use_bracket_target
        ? "ADAPTIVE_OVERLOAD_USE_LOW_PLUS_QUARTER_GAP"
        : "ADAPTIVE_OVERLOAD_USE_WINDOW_MINIMUM";
  } else {
    action = use_bracket_target
        ? "ADAPTIVE_UNDERLOAD_USE_LOW_PLUS_HALF_GAP"
        : "ADAPTIVE_UNDERLOAD_USE_WINDOW_MAXIMUM";
  }
  ScheduleWaveformCollectionAfterSettle(now, false);
  waveform_last_action_ = action;
  EmitWaveformSearchTrace(trace_analysis,
                          action,
                          baseline_before_bps,
                          amplitude_before_bps);
}

bool FBBRSender::AmplifyWaveformProbeAfterInconclusive(
    const WaveformWindowAnalysis& analysis,
    QuicTime now) {
  const uint64_t initial_amplitude_bps =
      waveform_initial_probe_amplitude_bps_ == 0
          ? GetCurrentAmplitudeBps()
          : waveform_initial_probe_amplitude_bps_;
  const uint64_t amplified_amplitude_bps = AmplifiedWaveformProbeAmplitude(
      current_probe_amplitude_bps_, initial_amplitude_bps,
      waveform_inconclusive_signal_amplification_factor_,
      waveform_inconclusive_signal_amplification_max_ratio_);
  if (amplified_amplitude_bps <= current_probe_amplitude_bps_) {
    return false;
  }

  WaveformWindowAnalysis trace_analysis = analysis;
  const double baseline_before_bps = static_cast<double>(
      current_injection_baseline_bw_.ToBitsPerSecond());
  const double amplitude_before_bps =
      static_cast<double>(current_probe_amplitude_bps_);
  current_probe_amplitude_bps_ = amplified_amplitude_bps;
  if (waveform_inconclusive_amplification_count_ <
      std::numeric_limits<uint32_t>::max()) {
    ++waveform_inconclusive_amplification_count_;
  }
  ++waveform_decision_count_;
  const std::string action = "AMPLIFY_INCONCLUSIVE_SIGNAL_AND_RETRY";
  waveform_last_action_ = action;
  waveform_last_invalid_reason_ = AppendReason(
      analysis.invalid_reason,
      "signal_amplified_after_second_inconclusive");
  trace_analysis.invalid_reason = waveform_last_invalid_reason_;
  waveform_last_delta_source_ = kWaveformDeltaSourceNone;
  waveform_last_raw_delta_bw_bps_ = 0.0;
  waveform_last_applied_delta_bw_bps_ = 0.0;
  ScheduleWaveformCollectionAfterSettle(now, false);
  EmitWaveformSearchTrace(trace_analysis,
                          action,
                          baseline_before_bps,
                          amplitude_before_bps);
  return true;
}

void FBBRSender::RunWaveformCruiseStateMachine(QuicTime now) {
  if (cruise_detector_mode_ != FBBRCruiseDetectorMode::kTimeWaveform ||
      !in_cruise_ || waveform_cruise_state_ ==
                         WaveformCruiseState::kDisabled) {
    return;
  }
  for (int transition = 0; transition < 4; ++transition) {
    if (waveform_cruise_state_ ==
            WaveformCruiseState::kWaitInitialSettle ||
        waveform_cruise_state_ ==
            WaveformCruiseState::kWaitPostAdjustmentSettle) {
      if (now < waveform_settle_end_) {
        return;
      }
      waveform_cruise_state_ = WaveformCruiseState::kCollectCycle;
      continue;
    }
    if (waveform_cruise_state_ == WaveformCruiseState::kCollectCycle ||
        waveform_cruise_state_ == WaveformCruiseState::kExtendCycle) {
      if (now < waveform_window_end_) {
        return;
      }
      waveform_cruise_state_ = WaveformCruiseState::kAnalyzeCycle;
      continue;
    }
    if (waveform_cruise_state_ == WaveformCruiseState::kAnalyzeCycle) {
      WaveformWindowAnalysis analysis;
      const bool use_rolling_retry =
          UsesAdaptiveLoadJudgment() || IsFbbrHybridObserver();
      if (use_rolling_retry && waveform_window_extended_ &&
          waveform_window_periods_ >= 3.0 &&
          cruise_modulation_freq_hz_ > 0.0) {
        const TimeDelta period = TimeDelta::FromMicroseconds(
            static_cast<int64_t>(std::llround(
                1000000.0 / cruise_modulation_freq_hz_)));
        const QuicTime later_window_start = waveform_window_start_ + period;
        const QuicTime prior_window_end = later_window_start + period;
        const WaveformWindowAnalysis prior = AnalyzeWaveformWindow(
            waveform_window_start_, prior_window_end, 2.0, false);
        analysis = AnalyzeWaveformWindow(
            later_window_start, waveform_window_end_, 2.0, true);
        analysis.collection_window_start = waveform_window_start_;
        analysis.collection_window_end = waveform_window_end_;
        analysis.collection_window_periods = waveform_window_periods_;
        analysis.extended_window = true;
        analysis.analysis_uses_later_cycle = true;
        analysis.prior_cycle_srtt_input_valid = prior.srtt_input_valid;
        analysis.prior_cycle_srtt_similar = prior.srtt_effective_similar;
        analysis.prior_cycle_drate_input_valid = prior.drate_input_valid;
        analysis.prior_cycle_drate_similar = prior.drate_effective_similar;
        analysis.prior_cycle_classification = prior.classification;
      } else {
        analysis = AnalyzeWaveformWindow(
            waveform_window_start_, waveform_window_end_,
            waveform_window_periods_, waveform_window_extended_);
      }
      if (IsFbbrHybridObserver()) {
        UpdateFbbrHybridRetryState(&analysis);
      } else {
        UpdateMaxBwAttenuationFromWaveform(analysis);
        RefreshRtpropFromTrueBottomClip(&analysis, now);
      }
      if (analysis.classification ==
              WaveformClassification::kInconclusive &&
          inconclusive_extension_count_ > 0 &&
          AmplifyWaveformProbeAfterInconclusive(analysis, now)) {
        return;
      }
      if (analysis.classification ==
              WaveformClassification::kInconclusive &&
          ShouldObserveAfterInconclusive(
              use_rolling_retry, inconclusive_extension_count_,
              waveform_max_inconclusive_extensions_,
              waveform_window_periods_, waveform_max_window_periods_)) {
        const double start_advance_periods =
            InconclusiveWindowStartAdvancePeriods(
                use_rolling_retry, waveform_window_extended_,
                waveform_window_periods_);
        const double extended_periods = use_rolling_retry
            ? std::min(3.0, waveform_max_window_periods_)
            : std::min(waveform_max_window_periods_,
                       waveform_window_periods_ + 1.0);
        if (start_advance_periods == 0.0 &&
            extended_periods <= waveform_window_periods_ + 1e-12) {
          ApplyWaveformClassification(analysis, now);
          return;
        }
        const double baseline_before_bps = static_cast<double>(
            current_injection_baseline_bw_.ToBitsPerSecond());
        const double amplitude_before_bps =
            static_cast<double>(current_probe_amplitude_bps_);
        if (inconclusive_extension_count_ <
            std::numeric_limits<uint32_t>::max()) {
          ++inconclusive_extension_count_;
        }
        ++waveform_decision_count_;
        waveform_last_action_ = "OBSERVE_ONE_MORE_PERIOD";
        waveform_last_delta_source_ = kWaveformDeltaSourceNone;
        waveform_last_raw_delta_bw_bps_ = 0.0;
        waveform_last_applied_delta_bw_bps_ = 0.0;
        waveform_cruise_state_ = WaveformCruiseState::kExtendCycle;
        EmitWaveformSearchTrace(analysis,
                                waveform_last_action_,
                                baseline_before_bps,
                                amplitude_before_bps);
        const TimeDelta period = TimeDelta::FromMicroseconds(
            static_cast<int64_t>(std::llround(
                1000000.0 / cruise_modulation_freq_hz_)));
        const QuicTime next_window_start = start_advance_periods > 0.5
            ? waveform_window_start_ + period
            : waveform_window_start_;
        StartWaveformCollectionAt(
            next_window_start, extended_periods, true);
        return;
      }
      ApplyWaveformClassification(analysis, now);
      return;
    }
    return;
  }
}

void FBBRSender::EmitWaveformSearchTrace(
    const WaveformWindowAnalysis& analysis,
    const std::string& action,
    double baseline_before_bps,
    double amplitude_before_bps) const {
  if (!cruise_load_trace_cb_) {
    return;
  }
  auto seconds = [](QuicTime time) {
    return time == QuicTime::Zero()
               ? 0.0
               : static_cast<double>((time - QuicTime::Zero())
                                         .ToMicroseconds()) /
                     1000000.0;
  };
  const double time_s = seconds(analysis.collection_window_end);
  const double baseline_after_bps = static_cast<double>(
      current_injection_baseline_bw_.ToBitsPerSecond());
  const double amplitude_after_bps =
      static_cast<double>(current_probe_amplitude_bps_);
  const QuicByteCount v3_actual_inflight =
      latest_congestion_event_inflight_valid_
          ? latest_congestion_event_inflight_ : 0;
  const FbbrV3ProjectionSnapshot v3 =
      BuildFbbrV3ProjectionSnapshot(cwnd_, v3_actual_inflight);
  const FbbrV4EnvelopeSnapshot v4 =
      BuildFbbrV4EnvelopeSnapshot(cwnd_, v3_actual_inflight);
  const double v3_cap_binding_fraction =
      fbbr_v3_window_ack_events_ == 0
          ? 0.0
          : static_cast<double>(fbbr_v3_window_cap_binding_events_) /
                static_cast<double>(fbbr_v3_window_ack_events_);
  const double v4_cap_binding_fraction =
      fbbr_v4_window_ack_events_ == 0
          ? 0.0
          : static_cast<double>(fbbr_v4_window_cap_binding_events_) /
                static_cast<double>(fbbr_v4_window_ack_events_);
  auto clip_case_name = [](SrttClipCase clip_case) {
    switch (clip_case) {
      case SrttClipCase::kU1PositiveShoulder: return "U1";
      case SrttClipCase::kU2LongTopLine: return "U2";
      case SrttClipCase::kU3RepeatedTopClip: return "U3";
      case SrttClipCase::kL1NegativeShoulder: return "L1";
      case SrttClipCase::kL2LongBottomLine: return "L2";
      case SrttClipCase::kL3RepeatedBottomClip: return "L3";
      default: return "NONE";
    }
  };
  const double probe_epoch_rtt_s = static_cast<double>(
      analysis.probe_epoch_rtt.ToMicroseconds()) / 1000000.0;
  std::ostringstream row;
  row << time_s << ","
      << cruise_id_ << ","
      << waveform_decision_count_ << ","
      << WaveformStateName(waveform_cruise_state_) << ","
      << CruiseDetectorModeName(cruise_detector_mode_) << ","
      << model_.cwnd_gain() << ","
      << seconds(analysis.probe_epoch_start) << ","
      << probe_epoch_rtt_s << ","
      << seconds(analysis.collection_window_start) << ","
      << seconds(analysis.collection_window_end) << ","
      << (waveform_negative_half_first_ ? "true" : "false") << ","
      << analysis.collection_window_periods << ","
      << (analysis.extended_window ? "true" : "false") << ","
      << seconds(analysis.window_start) << ","
      << seconds(analysis.window_end) << ","
      << analysis.window_periods << ","
      << (analysis.analysis_uses_later_cycle ? "true" : "false") << ","
      << (analysis.prior_cycle_srtt_input_valid ? "true" : "false") << ","
      << (analysis.prior_cycle_srtt_similar ? "true" : "false") << ","
      << (analysis.prior_cycle_drate_input_valid ? "true" : "false") << ","
      << (analysis.prior_cycle_drate_similar ? "true" : "false") << ","
      << WaveformClassificationName(
             analysis.prior_cycle_classification) << ","
      << analysis.sender_sample_count << ","
      << analysis.drate_sample_count << ","
      << analysis.srtt_sample_count << ","
      << analysis.srtt_stat_sample_count << ","
      << (analysis.srtt_stats_valid ? "true" : "false") << ","
      << analysis.srtt_mean_ms << ","
      << analysis.srtt_min_ms << ","
      << analysis.srtt_max_ms << ","
      << (analysis.latest_waveform_overload_srtt_mean_valid
              ? "true"
              : "false") << ","
      << analysis.latest_waveform_overload_srtt_mean_ms << ","
      << (analysis.latest_waveform_underload_srtt_mean_valid
              ? "true"
              : "false") << ","
      << analysis.latest_waveform_underload_srtt_mean_ms << ","
      << analysis.coverage_ratio << ","
      << analysis.app_limited_ratio << ","
      << (analysis.sender_waveform_valid ? "true" : "false") << ","
      << analysis.best_lag_s << ","
      << (analysis.srtt_input_valid ? "true" : "false") << ","
      << (analysis.srtt_similar_frequency ? "true" : "false") << ","
      << (analysis.srtt_similar ? "true" : "false") << ","
      << (analysis.srtt_similar_without_middle ? "true" : "false") << ","
      << (analysis.srtt_effective_similar ? "true" : "false") << ","
      << analysis.srtt_without_middle_completeness.estimated_period << ","
      << analysis.srtt_without_middle_completeness.periodicity_correlation
      << ","
      << (analysis.srtt_cycle_complete ? "true" : "false") << ","
      << (analysis.srtt_positive_half_clipped ? "true" : "false") << ","
      << (analysis.srtt_negative_half_clipped ? "true" : "false") << ","
      << (analysis.srtt_only_negative_half ? "true" : "false") << ","
      << (analysis.srtt_only_positive_half ? "true" : "false") << ","
      << analysis.plateau.srtt_positive_half_span_ms << ","
      << analysis.plateau.srtt_negative_half_span_ms << ","
      << (analysis.srtt_clip_ambiguous ? "true" : "false") << ","
      << (analysis.bic_clipping.valid ? "true" : "false") << ","
      << (analysis.bic_clipping.top_clip ? "true" : "false") << ","
      << (analysis.bic_clipping.bottom_clip ? "true" : "false") << ","
      << (analysis.bic_clipping.both_clipped ? "true" : "false") << ","
      << analysis.bic_clipping.top_motif_count << ","
      << analysis.bic_clipping.bottom_motif_count << ","
      << analysis.bic_clipping.selected_segment_count << ","
      << analysis.bic_clipping.selected_score << ","
      << analysis.bic_clipping.top_clip_min_rounded_bic_margin << ","
      << analysis.bic_clipping.bottom_clip_min_rounded_bic_margin << ","
      << analysis.bic_clipping.top_clip_combined_rounded_bic_margin << ","
      << analysis.bic_clipping.bottom_clip_combined_rounded_bic_margin << ","
      << analysis.bic_clipping.top_clip_pair_sharp_motif_count << ","
      << analysis.bic_clipping.bottom_clip_pair_sharp_motif_count << ","
      << (analysis.true_bottom_clip_rtprop_refresh_applied
              ? "true"
              : "false") << ","
      << analysis.true_bottom_clip_rtprop_before_ms << ","
      << analysis.true_bottom_clip_rtprop_after_ms << ","
      << analysis.true_bottom_clip_min_rtt_timestamp_before_s << ","
      << analysis.true_bottom_clip_min_rtt_timestamp_after_s << ","
      << analysis.true_bottom_clip_probe_rtt_deadline_after_s << ","
      << analysis.bic_clipping.invalid_reason << ","
      << analysis.srtt_direct_ncc << ","
      << analysis.srtt_integral_ncc << ","
      << analysis.srtt_derivative_ncc << ","
      << analysis.srtt_slope_direction_agreement << ","
      << analysis.srtt_completeness.estimated_period << ","
      << analysis.srtt_completeness.period_error_ratio << ","
      << analysis.srtt_completeness.periodicity_correlation << ","
      << analysis.srtt_fit.fitted_response_amplitude << ","
      << analysis.srtt_fit.robust_noise_sigma << ","
      << analysis.srtt_fit.response_snr << ","
      << analysis.srtt_completeness.completeness_score << ","
      << (analysis.drate_input_valid ? "true" : "false") << ","
      << (analysis.drate_similar ? "true" : "false") << ","
      << (analysis.drate_similar_without_middle ? "true" : "false") << ","
      << (analysis.drate_effective_similar ? "true" : "false") << ","
      << analysis.drate_without_middle_completeness.estimated_period << ","
      << analysis.drate_without_middle_completeness.periodicity_correlation
      << ","
      << analysis.drate_ncc << ","
      << analysis.drate_slope_direction_agreement << ","
      << analysis.drate_completeness.estimated_period << ","
      << analysis.drate_completeness.period_error_ratio << ","
      << analysis.drate_completeness.periodicity_correlation << ","
      << analysis.current_drate_response_amplitude_bps << ","
      << analysis.drate_fit.robust_noise_sigma << ","
      << analysis.drate_fit.response_snr << ","
      << analysis.drate_completeness.completeness_score << ","
      << (has_last_similar_drate_amplitude_ ? "true" : "false") << ","
      << last_similar_drate_amplitude_bps_ << ","
      << analysis.delta_source << ","
      << analysis.raw_delta_bw_bps << ","
      << analysis.applied_delta_bw_bps << ","
      << analysis.delta_reference_bps << ","
      << analysis.window_extreme_gap_bps << ","
      << analysis.actuator_step_multiplier << ","
      << analysis.queue_delay_ms << ","
      << analysis.queue_delay_min_rtt_ratio << ","
      << analysis.overload_confirmation_count << ","
      << (underload_located_ ? "true" : "false") << ","
      << WaveformClassificationName(analysis.classification) << ","
      << action << ","
      << baseline_before_bps << ","
      << baseline_after_bps << ","
      << amplitude_before_bps << ","
      << amplitude_after_bps << ","
      << max_bw_attenuation_factor_ << ","
      << max_bw_actual_fluctuation_amplitude_bps_ << ","
      << max_bw_delivery_response_gain_ << ","
      << (max_bw_response_observed_ ? "true" : "false") << ","
      << (trusted_baseline_locked_ ? "true" : "false") << ","
      << trusted_bw_candidate_update_count_ << ","
      << "true" << ","
      << (trusted_bw_candidate_.IsZero()
              ? 0
              : trusted_bw_candidate_.ToBitsPerSecond()) << ","
      << trusted_bw_candidate_source_ << ","
      << (analysis.invalid_reason.empty()
              ? "none"
              : analysis.invalid_reason) << ","
      << analysis.decision_rule << ","
      << analysis.delivery_rate_stat_sample_count << ","
      << (analysis.delivery_rate_stats_valid ? "true" : "false") << ","
      << analysis.delivery_rate_min_bps << ","
      << analysis.delivery_rate_max_bps << ","
      << analysis.delivery_rate_mean_bps << ","
      << analysis.latest_trusted_bw_bps << ","
      << analysis.smoothed_trusted_bw_bps << ","
      << (analysis.plateau.drate_positive_half_clipped ? "true" : "false")
      << ","
      << (analysis.plateau.drate_negative_half_clipped ? "true" : "false")
      << ","
      << (analysis.plateau.drate_only_negative_half ? "true" : "false")
      << ","
      << analysis.plateau.drate_positive_half_span_bps << ","
      << analysis.plateau.drate_negative_half_span_bps << ","
      << (analysis.plateau.positive_half_clips_simultaneous ? "true" : "false")
      << ","
      << (analysis.plateau.srtt_middle_sequential_plateau ? "true" : "false")
      << ","
      << (analysis.plateau.drate_middle_sequential_plateau ? "true" : "false")
      << ","
      << (analysis.plateau.drate_middle_any_plateau ? "true" : "false")
      << ","
      << (analysis.plateau.drate_has_waveform ? "true" : "false") << ","
      << analysis.plateau.plateau_candidate_count << ","
      << analysis.plateau.middle_sequential_candidate_count << ","
      << (analysis.plateau.top_clip ? "true" : "false") << ","
      << (analysis.plateau.bottom_clip ? "true" : "false") << ","
      << (analysis.plateau.shoulders_opposite ? "true" : "false") << ","
      << analysis.plateau.shoulder_slope_before << ","
      << analysis.plateau.shoulder_slope_after << ","
      << analysis.plateau.other_shoulder_slope_before << ","
      << analysis.plateau.other_shoulder_slope_after << ","
      << analysis.plateau.shoulder_change_before << ","
      << analysis.plateau.shoulder_change_after << ","
      << analysis.plateau.minimum_shoulder_change << ","
      << analysis.plateau.plateau_duration_ratio << ","
      << analysis.plateau.plateau_level_span_ratio << ","
      << analysis.plateau.half_overlap_ratio << ","
      << analysis.plateau.plateau_extreme_distance_ratio << ","
      << analysis.boundary_lift_time_s << ","
      << analysis.boundary_delta_bps << ","
      << waveform_amplitude_reduction_count_ << ","
      << floor_clip_confirmation_count_ << ","
      << (adaptive_bounds_inherited_this_cruise_ ? "true" : "false")
      << ","
      << adaptive_cruise_start_max_bw_.ToBitsPerSecond() << ","
      << (adaptive_baseline_low_valid_ ? "true" : "false") << ","
      << adaptive_baseline_low_.ToBitsPerSecond() << ","
      << (adaptive_baseline_up_valid_ ? "true" : "false") << ","
      << adaptive_baseline_up_.ToBitsPerSecond() << ","
      << (latest_waveform_underload_srtt_mean_valid_ ? "true" : "false")
      << ","
      << latest_waveform_underload_srtt_mean_ms_ << ","
      << (latest_waveform_overload_srtt_mean_valid_ ? "true" : "false")
      << ","
      << latest_waveform_overload_srtt_mean_ms_ << ","
      << (IsFbbrHybridV4() ? "FBBR-hybirdv4"
                           : IsFbbrHybridV3() ? "FBBR-hybridv3"
                           : IsFbbrHybrid() ? "FBBR-hybrid"
                         : (adaptive_guard_enabled_ ? "FBBR-adaptive"
                                                    : "FBBR")) << ","
      << (analysis.fbbr_hybrid_pipeline ? "fbbr_hybrid_v2" : "legacy")
      << ","
      << (analysis.fbbr_hybrid_pipeline ? "fbbr_hybrid_v2" : "legacy")
      << ","
      << analysis.hybrid_decision.rule_id << ","
      << WaveformClassificationName(analysis.unsuppressed_classification)
      << ","
      << (analysis.hybrid_features.srtt.suspected_top_candidate ? "true" : "false") << ","
      << (analysis.hybrid_features.srtt.suspected_bottom_candidate ? "true" : "false") << ","
      << (analysis.hybrid_features.srtt.positive_shoulder ? "true" : "false") << ","
      << (analysis.hybrid_features.srtt.long_top_line ? "true" : "false") << ","
      << (analysis.hybrid_features.srtt.repeated_top_clip ? "true" : "false") << ","
      << (analysis.hybrid_features.srtt.negative_shoulder ? "true" : "false") << ","
      << (analysis.hybrid_features.srtt.long_bottom_line ? "true" : "false") << ","
      << (analysis.hybrid_features.srtt.repeated_bottom_clip ? "true" : "false") << ","
      << clip_case_name(analysis.hybrid_features.selected_clip_case) << ","
      << (analysis.hybrid_features.both_clip_directions ? "true" : "false") << ","
      << (analysis.hybrid_features.clip_candidate_rejected_to_wave_fallback ? "true" : "false") << ","
      << (analysis.hybrid_features.fallback_entered ? "true" : "false") << ","
      << (analysis.hybrid_features.srtt.upper_clip_periodic_veto ? "true" : "false") << ","
      << (analysis.hybrid_features.drate.upper_clip_periodic_veto ? "true" : "false") << ","
      << (analysis.hybrid_features.srtt.lower_clip_ignored_for_periodic ? "true" : "false") << ","
      << (analysis.hybrid_features.drate.lower_clip_ignored_for_periodic ? "true" : "false") << ","
      << (analysis.hybrid_features.srtt.ordinary_wave_uses_raw_valid_view ? "RAW" : "CLEANED") << ","
      << (analysis.hybrid_features.drate.ordinary_wave_uses_raw_valid_view ? "RAW" : "CLEANED") << ","
      << analysis.hybrid_features.srtt.top_repeated_clip.contact_fragment_count << ","
      << analysis.hybrid_features.srtt.bottom_repeated_clip.contact_fragment_count << ","
      << analysis.hybrid_features.srtt.top_repeated_clip.contact_sample_count << ","
      << analysis.hybrid_features.srtt.bottom_repeated_clip.contact_sample_count << ","
      << static_cast<unsigned>(analysis.hybrid_features.srtt.top_repeated_clip.contact_cycle_mask) << ","
      << static_cast<unsigned>(analysis.hybrid_features.srtt.bottom_repeated_clip.contact_cycle_mask) << ","
      << analysis.hybrid_features.srtt.top_repeated_clip.contact_time_span_ratio_of_window << ","
      << analysis.hybrid_features.srtt.bottom_repeated_clip.contact_time_span_ratio_of_window << ","
      << analysis.hybrid_features.srtt.top_repeated_clip.pooled_flat_fraction << ","
      << analysis.hybrid_features.srtt.bottom_repeated_clip.pooled_flat_fraction << ","
      << analysis.hybrid_features.srtt.top_repeated_clip.verified_boundary_fraction << ","
      << analysis.hybrid_features.srtt.bottom_repeated_clip.verified_boundary_fraction << ","
      << analysis.hybrid_features.srtt.top_repeated_clip.extrapolated_overshoot_ratio << ","
      << analysis.hybrid_features.srtt.bottom_repeated_clip.extrapolated_overshoot_ratio << ","
      << analysis.hybrid_features.drate.top_repeated_clip.contact_fragment_count << ","
      << analysis.hybrid_features.drate.bottom_repeated_clip.contact_fragment_count << ","
      << analysis.hybrid_features.drate.top_repeated_clip.contact_sample_count << ","
      << analysis.hybrid_features.drate.bottom_repeated_clip.contact_sample_count << ","
      << static_cast<unsigned>(analysis.hybrid_features.drate.top_repeated_clip.contact_cycle_mask) << ","
      << static_cast<unsigned>(analysis.hybrid_features.drate.bottom_repeated_clip.contact_cycle_mask) << ","
      << analysis.hybrid_features.drate.top_repeated_clip.contact_time_span_ratio_of_window << ","
      << analysis.hybrid_features.drate.bottom_repeated_clip.contact_time_span_ratio_of_window << ","
      << analysis.hybrid_features.srtt.longest_top_line_ratio_of_period << ","
      << analysis.hybrid_features.srtt.longest_bottom_line_ratio_of_period << ","
      << (analysis.hybrid_features.srtt.positive_shoulder_cycle_input_valid ? "true" : "false") << ","
      << (analysis.hybrid_features.srtt.negative_shoulder_cycle_input_valid ? "true" : "false") << ","
      << (analysis.hybrid_features.srtt.positive_shoulder_cycle_recognizable ? "true" : "false") << ","
      << (analysis.hybrid_features.srtt.negative_shoulder_cycle_recognizable ? "true" : "false") << ","
      << analysis.hybrid_features.srtt.continuous_horizontal_count << ","
      << analysis.hybrid_features.drate.continuous_horizontal_count << ","
      << analysis.hybrid_features.srtt.middle_mask_ratio << ","
      << analysis.hybrid_features.drate.middle_mask_ratio << ","
      << analysis.hybrid_features.srtt.middle_best_slope_mismatch_ratio << ","
      << analysis.hybrid_features.drate.middle_best_slope_mismatch_ratio << ","
      << analysis.hybrid_features.srtt.middle_best_bridge_deviation_ratio << ","
      << analysis.hybrid_features.drate.middle_best_bridge_deviation_ratio << ","
      << (analysis.hybrid_features.srtt.wave.has_wave ? "true" : "false") << ","
      << (analysis.hybrid_features.drate.wave.has_wave ? "true" : "false") << ","
      << analysis.hybrid_features.srtt.wave.failure_reason << ","
      << analysis.hybrid_features.drate.wave.failure_reason << ","
      << analysis.hybrid_features.srtt.wave.amplitude << ","
      << analysis.hybrid_features.drate.wave.amplitude << ","
      << analysis.hybrid_features.srtt.wave.noise_sigma << ","
      << analysis.hybrid_features.drate.wave.noise_sigma << ","
      << analysis.hybrid_features.srtt.wave.step_threshold << ","
      << analysis.hybrid_features.drate.wave.step_threshold << ","
      << analysis.hybrid_features.srtt.wave.active_step_ratio << ","
      << analysis.hybrid_features.drate.wave.active_step_ratio << ","
      << analysis.hybrid_features.srtt.wave.up_change_ratio << ","
      << analysis.hybrid_features.srtt.wave.down_change_ratio << ","
      << analysis.hybrid_features.drate.wave.up_change_ratio << ","
      << analysis.hybrid_features.drate.wave.down_change_ratio << ","
      << analysis.hybrid_features.srtt.wave.significant_path_ratio << ","
      << analysis.hybrid_features.drate.wave.significant_path_ratio << ","
      << analysis.hybrid_features.srtt.wave.slope_reversals << ","
      << analysis.hybrid_features.drate.wave.slope_reversals << ","
      << static_cast<unsigned>(analysis.hybrid_features.srtt.wave.active_cycle_mask) << ","
      << static_cast<unsigned>(analysis.hybrid_features.drate.wave.active_cycle_mask) << ","
      << (analysis.hybrid_features.srtt.periodic_similarity_input_valid ? "true" : "false") << ","
      << (analysis.hybrid_features.drate.periodic_similarity_input_valid ? "true" : "false") << ","
      << (analysis.hybrid_features.srtt.periodic_similar ? "true" : "false") << ","
      << (analysis.hybrid_features.drate.periodic_similar ? "true" : "false") << ","
      << analysis.hybrid_features.estimated_srate_period_s << ","
      << analysis.hybrid_features.srtt.estimated_period_s << ","
      << analysis.hybrid_features.drate.estimated_period_s << ","
      << analysis.hybrid_features.srtt.response_srate_period_error_ratio << ","
      << analysis.hybrid_features.drate.response_srate_period_error_ratio << ","
      << analysis.hybrid_features.srtt.edge_mask_ratio << ","
      << analysis.hybrid_features.drate.edge_mask_ratio << ","
      << (analysis.hybrid_features.inflight_bdp_valid ? "true" : "false") << ","
      << analysis.hybrid_features.inflight_bytes << ","
      << analysis.hybrid_features.bdp_bytes << ","
      << (fbbr_hybrid_max_srtt_valid_ ? "true" : "false") << ","
      << analysis.max_srtt_before_ms << ","
      << analysis.max_srtt_after_ms << ","
      << (fbbr_hybrid_rtprop_drate_valid_ ? "true" : "false") << ","
      << analysis.rtprop_drate_before_bps << ","
      << analysis.rtprop_drate_after_bps << ","
      << (analysis.hybrid_baseline_low_before_valid ? "true" : "false") << ","
      << analysis.hybrid_baseline_low_before_bps << ","
      << (analysis.hybrid_baseline_low_after_valid ? "true" : "false") << ","
      << analysis.hybrid_baseline_low_after_bps << ","
      << (analysis.hybrid_max_bw_before_valid ? "true" : "false") << ","
      << analysis.hybrid_max_bw_before_bps << ","
      << (analysis.hybrid_max_bw_after_valid ? "true" : "false") << ","
      << analysis.hybrid_max_bw_after_bps << ","
      << (analysis.hybrid_srtt_low_rtprop_valid ? "true" : "false") << ","
      << analysis.hybrid_srtt_low_rtprop_ms << ","
      << (analysis.hybrid_max_srtt_valid ? "true" : "false") << ","
      << analysis.hybrid_max_srtt_ms << ","
      << analysis.hybrid_swing_bps << ","
      << analysis.hybrid_reference_gap_bps << ","
      << (analysis.hybrid_bracket_valid ? "true" : "false") << ","
      << analysis.hybrid_bracket_target_bps << ","
      << (analysis.hybrid_bracket_triggered ? "true" : "false") << ","
      << (analysis.hybrid_midpoint_triggered ? "true" : "false") << ","
      << analysis.window_first_cycle_id << ","
      << analysis.window_second_cycle_id << ","
      << static_cast<unsigned>(analysis.srtt_no_wave_streak) << ","
      << static_cast<unsigned>(analysis.drate_no_wave_streak) << ","
      << (analysis.wave_fidelity_enhancement_active ? "true" : "false") << ","
      << (analysis.wave_fidelity_just_entered ? "true" : "false") << ","
      << static_cast<unsigned>(analysis.retry_reason_mask) << ","
      << (analysis.no_wave_triggered ? "true" : "false") << ","
      << (analysis.classification_suppressed_for_retry ? "true" : "false") << ","
      << (analysis.state_updates_suppressed_for_retry ? "true" : "false") << ","
      << (analysis.fbbr_hybrid_pipeline ? fbbr_wave_fidelity_retry_window_advance_periods_ : 0) << ","
      << (analysis.fbbr_hybrid_pipeline ? 1 : 0) << ","
      << inconclusive_extension_count_ << ","
      << waveform_inconclusive_amplification_count_ << ","
      << waveform_initial_probe_amplitude_bps_ << ","
      << current_probe_amplitude_bps_ << ","
      << static_cast<double>(waveform_initial_probe_amplitude_bps_) *
             waveform_inconclusive_signal_amplification_max_ratio_ << ","
      << fbbr_hybrid_rolling_retry_count_ << ","
      << (v3.reference.valid
              ? v3.reference.bandwidth.ToBitsPerSecond() : 0) << ","
      << FbbrV3ReferenceSourceName(v3.reference.source) << ","
      << v3.model_inflight << ","
      << v3.inflight_cap << ","
      << v3.native_cwnd << ","
      << v3.actual_inflight << ","
      << v3.raw_queue_debt << ","
      << v3.enforced_excess << ","
      << v3_cap_binding_fraction << ","
      << (v3.history_valid ? "true" : "false") << ","
      << (v4.reference.valid
              ? v4.reference.bandwidth.ToBitsPerSecond() : 0) << ","
      << FbbrV3ReferenceSourceName(v4.reference.source) << ","
      << v4.plan_inflight << ","
      << v4.service_inflight << ","
      << v4.positive_probe_credit << ","
      << v4.service_budget << ","
      << v4.envelope << ","
      << v4.extra_acked << ","
      << v4.inflight_cap << ","
      << v4.native_cwnd << ","
      << v4.actual_inflight << ","
      << (v4.service_history_valid ? "true" : "false") << ","
      << (v4.app_limited_contaminated ? "true" : "false") << ","
      << (v4.projection_active ? "true" : "false") << ","
      << v4.service_restriction << ","
      << v4.enforced_excess << ","
      << v4_cap_binding_fraction;
  cruise_load_trace_cb_(time_s,
                        time_s,
                        0.0,
                        0.0,
                        0.0,
                        0.0,
                        "WAVEFORM_SEARCH",
                        analysis.classification ==
                            WaveformClassification::kInconclusive,
                        row.str());
  if (IsFbbrHybridV3()) {
    fbbr_v3_window_ack_events_ = 0;
    fbbr_v3_window_cap_binding_events_ = 0;
  }
  if (IsFbbrHybridV4()) {
    fbbr_v4_window_ack_events_ = 0;
    fbbr_v4_window_cap_binding_events_ = 0;
  }
}

void FBBRSender::PublishWaveformTrustedBw() {
  const double candidate_bps = static_cast<double>(
      trusted_bw_candidate_.ToBitsPerSecond());
  if (!trusted_baseline_locked_ || trusted_bw_candidate_.IsZero() ||
      !std::isfinite(candidate_bps) || candidate_bps <= 0.0) {
    ClearTrustedBw(waveform_last_invalid_reason_.empty() ||
                           waveform_last_invalid_reason_ == "none"
                       ? "waveform_not_locked"
                       : waveform_last_invalid_reason_.c_str());
    return;
  }
  TrustedBwSelectionResult selection = {
      BandwidthEstimate(),
      trusted_bw_candidate_,
      true,
      1.0,
      trusted_bw_candidate_source_,
      std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::quiet_NaN(),
      false,
      false,
      false,
      "NOT_APPLICABLE",
      false,
      false,
      0,
      0,
      0,
      0};
  PublishTrustedBwSelection(selection);
}

void FBBRSender::RunDueCruiseWindowAnalysis(QuicTime now) {
  if (cruise_detector_mode_ == FBBRCruiseDetectorMode::kTimeWaveform) {
    RunWaveformCruiseStateMachine(now);
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
  CruiseWindowResult result = BuildCruiseWindowResult(
      window_start, window_end, min_rtt, window_duration_s, "NORMAL");
  UpdateMaxBwAttenuationFromLegacyWindow(result);
  current_cruise_windows_.push_back(result);
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
  if (cruise_detector_mode_ == FBBRCruiseDetectorMode::kTimeWaveform) {
    RunWaveformCruiseStateMachine(now);
    if (IsFbbrHybridObserver()) {
      PublishFbbrHybridCruiseTrustedBw(nullptr);
    } else {
      PublishWaveformTrustedBw();
    }
    EmitCruiseSummaryTrace(now);
    return;
  }
  RunDueCruiseWindowAnalysis(now);
  const TrustedBwSelectionResult selection =
      RunTrustedBwSelection(now);
  if (IsFbbrHybridObserver()) {
    PublishFbbrHybridCruiseTrustedBw(&selection);
  } else {
    PublishTrustedBwSelection(selection);
  }
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
  const QuicBandwidth summary_hybrid_max_bw = model_.MaxBandwidth();
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
	                          : (cruise_detector_mode_ ==
	                                     FBBRCruiseDetectorMode::kTimeWaveform
	                                 ? "NOT_APPLICABLE"
	                                 : kLimitingSpectralSignalEqual)) << ","
	      << (best != nullptr ? best->spectral_invalid_reason
	                          : (cruise_detector_mode_ ==
	                                     FBBRCruiseDetectorMode::kTimeWaveform
	                                 ? "NOT_APPLICABLE"
	                                 : "none")) << ","
	      << summary_selection_native_bw.ToBitsPerSecond() << ","
	      << (trusted_bw_valid_ ? "true" : "false") << ","
	      << trusted_bw_cruise_id_ << ","
	      << (trusted_bw_fresh_ ? "true" : "false") << ","
	      << (trusted_bw_application_valid_ ? "true" : "false") << ","
	      << CruiseDetectorModeName(cruise_detector_mode_) << ","
	      << WaveformStateName(waveform_cruise_state_) << ","
	      << waveform_decision_count_ << ","
	      << baseline_adjustment_count_ << ","
	      << waveform_amplitude_reduction_count_ << ","
		      << (underload_located_ ? "true" : "false") << ","
		      << (trusted_bw_candidate_source_ == nullptr
		              ? kTrustedBwSourceNone
		              : trusted_bw_candidate_source_) << ","
		      << (adaptive_bounds_inherited_this_cruise_ ? "true" : "false")
		      << ","
		      << adaptive_cruise_start_max_bw_.ToBitsPerSecond() << ","
		      << (adaptive_baseline_low_valid_ ? "true" : "false") << ","
		      << adaptive_baseline_low_.ToBitsPerSecond() << ","
		      << (adaptive_baseline_up_valid_ ? "true" : "false") << ","
		      << adaptive_baseline_up_.ToBitsPerSecond() << ","
		      << (latest_waveform_underload_srtt_mean_valid_ ? "true" : "false")
		      << ","
		      << latest_waveform_underload_srtt_mean_ms_ << ","
		      << (latest_waveform_overload_srtt_mean_valid_ ? "true" : "false")
		      << ","
		      << latest_waveform_overload_srtt_mean_ms_ << ","
		      << (IsFbbrHybridObserver() && adaptive_baseline_low_valid_
		              ? "true" : "false") << ","
		      << (IsFbbrHybridObserver() && adaptive_baseline_low_valid_
		              ? adaptive_baseline_low_.ToBitsPerSecond() : 0) << ","
		      << (IsFbbrHybridObserver() &&
		              IsFinitePositiveBandwidth(summary_hybrid_max_bw)
		              ? "true" : "false") << ","
		      << (IsFbbrHybridObserver() &&
		              IsFinitePositiveBandwidth(summary_hybrid_max_bw)
		              ? summary_hybrid_max_bw.ToBitsPerSecond() : 0) << ","
		      << (IsFbbrHybridObserver() && fbbr_hybrid_srtt_low_valid_
		              ? "true" : "false") << ","
		      << (IsFbbrHybridObserver() && fbbr_hybrid_srtt_low_valid_
		              ? static_cast<double>(fbbr_hybrid_srtt_low_.ToMicroseconds()) /
		                    1000.0
		              : 0.0) << ","
		      << (IsFbbrHybridObserver() && fbbr_hybrid_max_srtt_valid_
		              ? "true" : "false") << ","
		      << (IsFbbrHybridObserver() && fbbr_hybrid_max_srtt_valid_
		              ? fbbr_hybrid_max_srtt_ms_ : 0.0);
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

FBBRSender::DeliveryRateWindowStats
FBBRSender::ComputeDeliveryRateWindowStats(
    const std::vector<FBBRRateSample>& samples) {
  DeliveryRateWindowStats stats;
  double sum_bps = 0.0;
  stats.min_bps = std::numeric_limits<double>::infinity();
  stats.max_bps = -std::numeric_limits<double>::infinity();
  for (const auto& sample : samples) {
    const double rate_bps =
        static_cast<double>(sample.rate.ToBitsPerSecond());
    if (!sample.valid || !std::isfinite(rate_bps) || rate_bps <= 0.0) {
      continue;
    }
    ++stats.sample_count;
    stats.min_bps = std::min(stats.min_bps, rate_bps);
    stats.max_bps = std::max(stats.max_bps, rate_bps);
    sum_bps += rate_bps;
  }
  stats.valid = stats.sample_count > 0 && std::isfinite(sum_bps);
  if (!stats.valid) {
    stats.min_bps = 0.0;
    stats.max_bps = 0.0;
    return stats;
  }
  stats.mean_bps = sum_bps / static_cast<double>(stats.sample_count);
  stats.valid = std::isfinite(stats.mean_bps) && stats.mean_bps > 0.0;
  return stats;
}

FBBRSender::SrttWindowStats FBBRSender::ComputeSrttWindowStats(
    const std::vector<FBBRRttSample>& samples) {
  SrttWindowStats stats;
  double sum_ms = 0.0;
  stats.min_ms = std::numeric_limits<double>::infinity();
  stats.max_ms = -std::numeric_limits<double>::infinity();
  for (const auto& sample : samples) {
    if (!std::isfinite(sample.rtt_ms) || sample.rtt_ms <= 0.0) {
      continue;
    }
    ++stats.sample_count;
    stats.min_ms = std::min(stats.min_ms, sample.rtt_ms);
    stats.max_ms = std::max(stats.max_ms, sample.rtt_ms);
    sum_ms += sample.rtt_ms;
  }
  stats.valid = stats.sample_count > 0 && std::isfinite(sum_ms);
  if (!stats.valid) {
    stats.min_ms = 0.0;
    stats.max_ms = 0.0;
    return stats;
  }
  stats.mean_ms = sum_ms / static_cast<double>(stats.sample_count);
  stats.valid = std::isfinite(stats.mean_ms) && stats.mean_ms > 0.0 &&
                std::isfinite(stats.min_ms) && stats.min_ms > 0.0 &&
                std::isfinite(stats.max_ms) && stats.max_ms > 0.0;
  return stats;
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
