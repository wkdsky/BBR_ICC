#include "fbbr_sender.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>
#include <limits>
#include <numeric>
#include <sstream>

#include "quic_bbr2_probe_bw.h"
#include "quic_logging.h"

namespace dqc {

namespace {

constexpr const char* kTrustedBwSourceNone = "NONE";
constexpr const char* kTrustedBwSourceNormal = "NORMAL_SPECTRAL";
constexpr const char* kTrustedBwSourceMerged = "MERGED_SPECTRAL";
constexpr const char* kTrustedBwSourceTimeWaveformBaseline =
    "TIME_WAVEFORM_SRTT_SEARCH";
constexpr const char* kTrustedBwSourceFbbrWindowMean =
    "FBBR_WINDOW_MEAN";
constexpr const char* kTrustedBwSourceFbbrWindowDeliveredRate =
    "FBBR_WINDOW_DELIVERED_RATE";
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
constexpr const char* kWaveformDeltaSourceFbbrWindowMinimum =
    "FBBR_WINDOW_MINIMUM";
constexpr const char* kWaveformDeltaSourceFbbrWindowMaximum =
    "FBBR_WINDOW_MAXIMUM";
constexpr const char* kWaveformDeltaSourceFbbrTrustedBw =
    "FBBR_TRUSTED_BW";
constexpr const char* kWaveformDeltaSourceFbbrRegimeIMaxDrate =
    "FBBR_REGIME_I_MAX_DRATE";
constexpr const char* kWaveformDeltaSourceFbbrRegimeIMaxBwMidpoint =
    "FBBR_REGIME_I_MAXBW_MAXDRATE_MIDPOINT";
constexpr const char* kWaveformDeltaSourceFbbrRegimeIGrowth =
    "FBBR_REGIME_I_BASELINE_GROWTH_1P02";
constexpr const char* kWaveformDeltaSourceFbbrRegimeIIIMinDrate =
    "FBBR_REGIME_III_MIN_DRATE";
constexpr const char* kWaveformDeltaSourceFbbrRegimeIIIMinBwMidpoint =
    "FBBR_REGIME_III_MINBW_MINDRATE_MIDPOINT";
constexpr const char* kWaveformDeltaSourceFbbrRegimeIIIDecrease =
    "FBBR_REGIME_III_BASELINE_DECREASE_0P98";
constexpr const char* kWaveformDeltaSourceFbbrRegimeIIDeliveryRateBaseline =
    "FBBR_REGIME_II_DELIVERY_RATE_BASELINE";
constexpr uint64_t kDefaultMinimumPacingRateBps = 200000;
constexpr double kWaveformPostAdjustmentCollectionPeriods = 2.0;
constexpr float kFBBRCruiseCwndGain = 1.25f;
constexpr float kFBBRRtpropProbeDownPacingGain = 0.75f;
constexpr const char* kLimitingSpectralSignalDrate = "DRATE";
constexpr const char* kLimitingSpectralSignalSrtt = "SRTT";
constexpr const char* kLimitingSpectralSignalEqual = "EQUAL";

double ClampValue(double value, double low, double high) {
  return std::max(low, std::min(high, value));
}

struct ProbeAmplitudeProtection {
  uint64_t effective_amplitude_bps = 0;
  bool capped = false;
  bool suppressed_by_floor = false;
};

ProbeAmplitudeProtection ProtectProbeAmplitude(uint64_t baseline_bps,
                                                uint64_t requested_bps,
                                                uint64_t minimum_rate_bps) {
  if (baseline_bps <= minimum_rate_bps) {
    return {0, false, true};
  }
  const uint64_t available_amplitude_bps = baseline_bps - minimum_rate_bps;
  if (requested_bps > available_amplitude_bps) {
    return {available_amplitude_bps, true, false};
  }
  return {requested_bps, false, false};
}

int64_t SaturateBpsToInt64(uint64_t bps) {
  return static_cast<int64_t>(std::min<uint64_t>(
      bps, static_cast<uint64_t>(std::numeric_limits<int64_t>::max())));
}

int64_t ScaleTriangleOffset(uint64_t amplitude_bps, double triangle_wave) {
  if (amplitude_bps == 0 || !std::isfinite(triangle_wave)) {
    return 0;
  }
  const long double product = static_cast<long double>(amplitude_bps) *
      static_cast<long double>(ClampValue(triangle_wave, -1.0, 1.0));
  const long double maximum =
      static_cast<long double>(std::numeric_limits<int64_t>::max());
  const long double minimum =
      static_cast<long double>(std::numeric_limits<int64_t>::min());
  if (product >= maximum) {
    return std::numeric_limits<int64_t>::max();
  }
  if (product <= minimum) {
    return std::numeric_limits<int64_t>::min();
  }
  return static_cast<int64_t>(product);
}

bool ShouldObserveAfterInconclusive(
    uint32_t extension_count,
    uint32_t max_extensions,
    double window_periods,
    double max_window_periods) {
  return extension_count < max_extensions &&
         window_periods < max_window_periods;
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

}  // namespace

constexpr size_t FBBRSender::kMaxHistorySamples;
constexpr uint32_t FBBRSender::kStableRounds;
constexpr double FBBRSender::kDefaultOscillationFreqHz;
constexpr double FBBRSender::kSampleStepSec;
constexpr double FBBRSender::kFbbrServiceFairBeta;
constexpr uint64_t FBBRSender::kFbbrServiceFairMinimumBps;

FBBRSender::FBBRSender(
    QuicTime now,
    const RttStats* rtt_stats,
    const QuicUnackedPacketMap* unacked_packets,
    QuicPacketCount initial_cwnd_in_packets,
    QuicPacketCount max_cwnd_in_packets,
    Random* random,
    QuicConnectionStats* stats,
    bool enable_ecn,
    CongestionControlType congestion_control_type,
    bool enable_probe_rtt)
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
                 enable_probe_rtt),
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
      current_time_(now),
      last_ack_time_(QuicTime::Zero()),
      use_delivery_rate_latest_for_signal_history_(false),
      cruise_id_(0),
      waveform_cruise_state_(WaveformCruiseState::kDisabled),
      initial_cruise_baseline_bw_(QuicBandwidth::Zero()),
      current_injection_baseline_bw_(QuicBandwidth::Zero()),
      current_probe_amplitude_bps_(0),
      effective_probe_amplitude_bps_(0),
      waveform_initial_probe_amplitude_bps_(0),
      probe_amplitude_capped_for_window_(false),
      probe_amplitude_suppressed_by_floor_for_window_(false),
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
      waveform_last_delta_source_(kWaveformDeltaSourceNone),
      waveform_last_raw_delta_bw_bps_(0.0),
      waveform_last_applied_delta_bw_bps_(0.0),
      baseline_adjustment_count_(0),
      inconclusive_extension_count_(0),
      waveform_inconclusive_amplification_count_(0),
      fbbr_max_srtt_valid_(false),
      fbbr_max_srtt_ms_(0.0),
      fbbr_max_srtt_source_cruise_id_(0),
      fbbr_rtprop_valid_(false),
      fbbr_rtprop_(TimeDelta::Zero()),
      fbbr_rtprop_source_time_(QuicTime::Zero()),
      fbbr_srtt_no_wave_streak_(0),
      fbbr_drate_no_wave_streak_(0),
      fbbr_wave_fidelity_enhancement_active_(false),
      fbbr_retry_reason_mask_(0),
      fbbr_last_counted_window_second_cycle_id_(0),
      fbbr_rolling_retry_count_(0),
      fbbr_regime_ii_seen_this_cruise_(false),
      fbbr_cruise_trusted_bw_(QuicBandwidth::Zero()),
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
      waveform_local_slope_window_period_ratio_(0.05),
      waveform_min_local_slope_window_ms_(5.0),
      waveform_clip_min_duration_ratio_(0.15),
      waveform_clip_min_half_overlap_ratio_(0.75),
      waveform_clip_max_slope_ratio_(0.10),
      waveform_delta_drate_amplitude_ratio_(0.50),
      waveform_delta_fallback_baseline_ratio_(0.25),
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
      goertzel_min_coherent_power_ratio_(0.10),
      fbbr_regime_long_top_horizontal_duration_ratio_(0.20),
      fbbr_regime_long_bottom_horizontal_duration_ratio_(0.30),
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
	      fair_share_bandwidth_bps_(0),
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
	      trace_flow_id_(0),
      gate_trace_mode_(FBBRGateTraceMode::kRoundOnly),
      gate_trace_sample_interval_(TimeDelta::FromMilliseconds(1)),
      last_pacing_gate_trace_time_(QuicTime::Zero()),

      fbbr_max_rtprop_seen_(TimeDelta::Zero()),
      fbbr_rate_history_integrity_valid_(true),
      fbbr_last_target_rate_(QuicBandwidth::Zero()),
      fbbr_last_base_target_rate_(QuicBandwidth::Zero()),
      fbbr_delivered_history_integrity_valid_(true),
      fbbr_last_counter_reset_time_(QuicTime::Zero()),
      fbbr_regime_i_or_iii_seen_this_cruise_(false),
      service_fair_last_valid_regime_seen_this_cruise_(false),
      service_fair_last_valid_regime_this_cruise_(
          WaveformClassification::kInconclusive),
      service_fair_service_rate_(QuicBandwidth::Zero()),
      service_fair_previous_service_rate_(QuicBandwidth::Zero()),
      service_fair_qdelay_ewma_(TimeDelta::Zero()),
      service_fair_previous_qdelay_ewma_(TimeDelta::Zero()),
      service_fair_cycle_count_(0),
      service_fair_qdelay_valid_(false),
      service_fair_previous_qdelay_valid_(false),
      service_fair_service_history_valid_(false),
      service_fair_signal_reset_time_(QuicTime::Zero()),
      service_fair_last_update_cruise_id_(-1),
      service_fair_action_(FbbrServiceFairAction::kNotRun),
      service_fair_qdelay_trend_(TimeDelta::Zero()),
      service_fair_service_rate_change_(0.0),
      service_fair_alpha_bps_(0.0),
      service_fair_raw_regime_candidate_bps_(0.0),
      service_fair_final_regime_candidate_bps_(0.0),
      fbbr_telemetry_last_time_(QuicTime::Zero()),
      fbbr_telemetry_initialized_(false),
      fbbr_telemetry_previous_trusted_source_(
          FbbrPreviousTrustedSource::kInvalid),
      fbbr_telemetry_projection_active_(false),
      fbbr_telemetry_service_history_valid_(false),
      fbbr_telemetry_app_limited_fallback_(false),
      fbbr_telemetry_plan_only_fallback_(false),
      fbbr_telemetry_service_limited_(false),
      fbbr_telemetry_cap_binding_(false),
      fbbr_telemetry_plan_inflight_(0),
      fbbr_telemetry_service_inflight_(0),
      fbbr_telemetry_probe_credit_(0),
      fbbr_telemetry_extra_acked_(0),
      fbbr_telemetry_service_restriction_(0),
      fbbr_telemetry_enforced_excess_(0),
      fbbr_telemetry_total_us_(0),
      fbbr_previous_trusted_us_(0),
      fbbr_previous_trusted_guard_source_us_(0),
      fbbr_previous_trusted_invalid_us_(0),
      fbbr_projection_active_us_(0),
      fbbr_service_history_valid_us_(0),
      fbbr_app_limited_fallback_us_(0),
      fbbr_plan_only_fallback_us_(0),
      fbbr_service_limited_us_(0),
      fbbr_cap_binding_us_(0),
      fbbr_plan_inflight_byte_us_(0.0L),
      fbbr_service_inflight_byte_us_(0.0L),
      fbbr_probe_credit_byte_us_(0.0L),
      fbbr_extra_acked_byte_us_(0.0L),
      fbbr_service_restriction_byte_us_(0.0L),
      fbbr_enforced_excess_byte_us_(0.0L),
      fbbr_window_ack_events_(0),
      fbbr_window_cap_binding_events_(0),
      fbbr_flow_summary_emitted_(false) {
  InitializeFbbrRtpropFromModel();
  QUIC_DVLOG(2) << this << " Initializing FBBRSender @ " << now;
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
      config.pacing_minimum_rate_mbps, 0.000001, maximum_rate_mbps, 0.2,
      "pacing.minimum_rate_mbps");
  minimum_pacing_rate_bps_ = std::max<uint64_t>(
      1, static_cast<uint64_t>(std::llround(minimum_rate_mbps * 1000000.0)));

  trusted_bw_clear_on_cruise_start_ = config.trusted_bw_clear_on_cruise_start;
  const bool waveform_recv_mode_valid =
      config.waveform_recv_signal_mode == "delivery_rate_latest" ||
      config.waveform_recv_signal_mode == "bandwidth_latest";
  if (!waveform_recv_mode_valid) {
    std::cerr
        << "[FBBR config warning] invalid waveform.recv_signal_mode='"
        << config.waveform_recv_signal_mode
        << "'; using delivery_rate_latest" << std::endl;
  }
  if (config.waveform_recv_signal_mode == "bandwidth_latest") {
    std::cerr << "[FBBR config warning] waveform.recv_signal_mode="
                 "bandwidth_latest is ignored; FBBR uses delivery_rate_latest"
              << std::endl;
  }
  use_delivery_rate_latest_for_signal_history_ = true;
  waveform_initial_settle_rtt_mult_ = range_or(
      config.waveform_initial_settle_rtt_mult, 0.0, 20.0, 1.0,
      "waveform.initial_settle_rtt_mult");
  waveform_post_adjust_settle_rtt_mult_ = range_or(
      config.waveform_post_adjust_settle_rtt_mult, 0.0, 20.0, 1.0,
      "waveform.post_adjust_settle_rtt_mult");
  if (std::abs(waveform_initial_settle_rtt_mult_ - 1.0) > 1e-12 ||
      std::abs(waveform_post_adjust_settle_rtt_mult_ - 1.0) > 1e-12) {
    std::cerr << "[FBBR config warning] time_waveform locks initial and "
                 "post-adjust response delay to exactly 1 RTT"
              << std::endl;
    waveform_initial_settle_rtt_mult_ = 1.0;
    waveform_post_adjust_settle_rtt_mult_ = 1.0;
  }
  waveform_negative_half_first_ = true;
  if (!config.waveform_negative_half_first) {
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
  if (config.waveform_max_inconclusive_extensions != 1) {
    std::cerr << "[FBBR config warning] FBBR time_waveform "
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
  goertzel_min_coherent_power_ratio_ = range_or(
      config.goertzel_min_coherent_power_ratio, 1e-6, 1.0, 0.10,
      "goertzel.min_coherent_power_ratio");

  fbbr_regime_long_top_horizontal_duration_ratio_ = range_or(
      config.fbbr_regime_long_top_horizontal_duration_ratio,
      0.01, 0.99, 0.20,
      "fbbr.regime.long_top_horizontal_duration_ratio");
  fbbr_regime_long_bottom_horizontal_duration_ratio_ = range_or(
      config.fbbr_regime_long_bottom_horizontal_duration_ratio,
      0.01, 0.99, 0.30,
      "fbbr.regime.long_bottom_horizontal_duration_ratio");
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
  fbbr_max_srtt_valid_ = false;
  fbbr_max_srtt_ms_ = 0.0;
  fbbr_max_srtt_source_cruise_id_ = 0;
  pending_fbbr_max_srtt_observations_.clear();
  fbbr_rtprop_valid_ = false;
  fbbr_rtprop_ = TimeDelta::Zero();
  fbbr_rtprop_source_time_ = QuicTime::Zero();
  InitializeFbbrRtpropFromModel();
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
  model_.SetMinBandwidthSampleCollection(false);
  model_.SetMinBandwidthSampleCorrection(1.0);
  fbbr_regime_i_or_iii_seen_this_cruise_ = false;
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

bool FBBRSender::ShouldExitProbeUpAfterRound() const {
  // ServiceFair can deliberately pace below native MaxBw and cap inflight
  // below the native queue-building threshold. In that case native ProbeBW_UP has
  // no reachable queue exit.  Preserve one complete packet-timed probing
  // round, then allow the cycle to continue so MaxBw and fairness feedback
  // keep receiving fresh ProbeBW cycles.
  return IsFbbrServiceFair() && HasUsableFbbrPreviousTrustedBw();
}

bool FBBRSender::BaseShouldOscillate() const {
  if (in_cruise_ &&
      (current_injection_baseline_bw_.IsZero() ||
       probe_epoch_start_time_ == QuicTime::Zero() ||
       waveform_cruise_state_ == WaveformCruiseState::kDisabled)) {
    return false;
  }
  const uint64_t amplitude_bps =
      in_cruise_
          ? effective_probe_amplitude_bps_
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

void FBBRSender::RefreshProbeAmplitudeProtection() {
  const ProbeAmplitudeProtection protection = ProtectProbeAmplitude(
      current_injection_baseline_bw_.ToBitsPerSecond(),
      current_probe_amplitude_bps_, minimum_pacing_rate_bps_);
  effective_probe_amplitude_bps_ = protection.effective_amplitude_bps;
  probe_amplitude_capped_for_window_ = protection.capped;
  probe_amplitude_suppressed_by_floor_for_window_ =
      protection.suppressed_by_floor;
}

bool FBBRSender::IsProbeAmplitudeProtectionActive() const {
  return probe_amplitude_capped_for_window_ ||
      probe_amplitude_suppressed_by_floor_for_window_;
}

bool FBBRSender::IsProbeAmplitudeSuppressedByFloor() const {
  return probe_amplitude_suppressed_by_floor_for_window_;
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

double FBBRSender::ComputeMinBwCorrectionFactor(
    double delivery_center_bps,
    double actual_fluctuation_amplitude_bps) {
  if (!std::isfinite(delivery_center_bps) || delivery_center_bps <= 0.0 ||
      !std::isfinite(actual_fluctuation_amplitude_bps) ||
      actual_fluctuation_amplitude_bps <= 0.0 ||
      actual_fluctuation_amplitude_bps >= delivery_center_bps) {
    return 1.0;
  }
  const long double center =
      static_cast<long double>(delivery_center_bps);
  const long double denominator = center -
      static_cast<long double>(actual_fluctuation_amplitude_bps);
  if (!std::isfinite(denominator) || denominator <= 0.0L) {
    return 1.0;
  }
  const double factor = static_cast<double>(center / denominator);
  return std::isfinite(factor) && factor >= 1.0 ? factor : 1.0;
}

uint64_t FBBRSender::CurrentEmittedProbeAmplitudeBps() const {
  return effective_probe_amplitude_bps_;
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
      current_injection_baseline_bw_.ToBitsPerSecond());
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

double FBBRSender::CurrentMinBwCorrectionFactor() const {
  if (!ShouldOscillate()) {
    return 1.0;
  }

  const double current_baseline_bps = static_cast<double>(
      current_injection_baseline_bw_.ToBitsPerSecond());
  if (!std::isfinite(current_baseline_bps) || current_baseline_bps <= 0.0) {
    return 1.0;
  }

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
  return ComputeMinBwCorrectionFactor(
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
      current_injection_baseline_bw_.ToBitsPerSecond());
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
      static_cast<double>(CurrentEmittedProbeAmplitudeBps()));
}

TimeDelta FBBRSender::CurrentFbbrMaxSrttObservationRtt() const {
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

bool FBBRSender::ComputeFbbrMaxSrttAround(
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

void FBBRSender::RecordFbbrMaxBwUpdateForMaxSrtt(
    QuicTime event_time,
    QuicBandwidth max_bw) {
  if (!UsesFbbrServiceEnvelope() || !IsFinitePositiveBandwidth(max_bw) ||
      event_time == QuicTime::Zero()) {
    return;
  }
  const TimeDelta rtt = CurrentFbbrMaxSrttObservationRtt();
  if (rtt.IsZero() || rtt.IsInfinite()) {
    return;
  }
  pending_fbbr_max_srtt_observations_.push_back(
      {event_time, rtt * 3, max_bw});
}

void FBBRSender::FinalizePendingFbbrMaxSrttObservations(QuicTime now) {
  if (!UsesFbbrServiceEnvelope()) {
    return;
  }
  while (!pending_fbbr_max_srtt_observations_.empty()) {
    const FbbrMaxSrttObservation& observation =
        pending_fbbr_max_srtt_observations_.front();
    if (now < observation.update_time + observation.half_window) {
      break;
    }
    double max_srtt_ms = 0.0;
    size_t sample_count = 0;
    if (ComputeFbbrMaxSrttAround(observation.update_time,
                                   observation.half_window,
                                   &max_srtt_ms,
                                   &sample_count)) {
      fbbr_max_srtt_valid_ = true;
      fbbr_max_srtt_ms_ = max_srtt_ms;
      fbbr_max_srtt_source_cruise_id_ =
          static_cast<uint64_t>(std::max<int64_t>(0, cruise_id_));
    }
    pending_fbbr_max_srtt_observations_.pop_front();
  }
}

double FBBRSender::TriangleWave(QuicTime now) const {
  if (cruise_modulation_freq_hz_ <= 0.0 ||
      cruise_start_time_ == QuicTime::Zero()) {
    return 0.0;
  }
  const QuicTime epoch_start = probe_epoch_start_time_;
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
  if (q < 0.25) {
    return -4.0 * q;
  }
  if (q < 0.75) {
    return 4.0 * q - 2.0;
  }
  return 4.0 - 4.0 * q;
}

void FBBRSender::OnProbeBwPhaseEntered(Bbr2ProbeBwMode::CyclePhase phase,
                                           QuicTime now) {
  Bbr2Sender::OnProbeBwPhaseEntered(phase, now);
  model_.SetMinBandwidthSampleCollection(
      UsesFbbrServiceEnvelope() &&
      phase == Bbr2ProbeBwMode::CyclePhase::PROBE_CRUISE);
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
  fbbr_regime_i_or_iii_seen_this_cruise_ = false;
  if (IsFbbrServiceFair()) {
    service_fair_last_valid_regime_seen_this_cruise_ = false;
    service_fair_last_valid_regime_this_cruise_ =
        WaveformClassification::kInconclusive;
  }
  current_cruise_rtprop_updated_ = false;
  previous_cruise_rtprop_updated_ = false;
  // Replace the constructor's initial-RTT placeholder once, using the first
  // path sample learned before Cruise.  After this bootstrap, only explicit
  // trusted Cruise/ProbeRTT publications version srtt_low.
  InitializeFbbrRtpropFromModel();
  cruise_rtprop_at_entry_ =
      UsesFbbrServiceEnvelope() && fbbr_rtprop_valid_
          ? fbbr_rtprop_
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
  QuicBandwidth current_native_max_bw = UsesFbbrServiceEnvelope()
      ? model_.MaxBandwidth() : BandwidthEstimate();
  if (current_native_max_bw.IsZero() ||
      current_native_max_bw.IsInfinite()) {
    current_native_max_bw = BandwidthEstimate();
  }
  initial_cruise_baseline_bw_ = current_native_max_bw;
  current_injection_baseline_bw_ = initial_cruise_baseline_bw_;
  if (UsesFbbrServiceEnvelope() && HasUsableFbbrPreviousTrustedBw()) {
    initial_cruise_baseline_bw_ = trusted_bw_;
    current_injection_baseline_bw_ = trusted_bw_;
  }
  if (IsFbbrServiceFair()) {
    RunFbbrServiceFairCycleUpdate(now);
  }
  current_probe_amplitude_bps_ = GetCurrentAmplitudeBps();
  waveform_initial_probe_amplitude_bps_ = current_probe_amplitude_bps_;
  RefreshProbeAmplitudeProtection();
  ResetMaxBwAttenuationEstimator();
  waveform_inconclusive_amplification_count_ = 0;
  fbbr_srtt_no_wave_streak_ = 0;
  fbbr_drate_no_wave_streak_ = 0;
  fbbr_wave_fidelity_enhancement_active_ = false;
  fbbr_retry_reason_mask_ = 0;
  fbbr_last_counted_window_second_cycle_id_ = 0;
  fbbr_rolling_retry_count_ = 0;
  fbbr_regime_ii_seen_this_cruise_ = false;
  fbbr_cruise_trusted_bw_ = QuicBandwidth::Zero();
  current_probe_bw_phase_gain_ = 1.0;
  ResetWaveformCruiseState(now);
  freq_tool_on_ = ShouldOscillate();
  cruise_freq_tool_active_ = freq_tool_on_;
  QUIC_DVLOG(2) << "FBBR: Entering PROBE_CRUISE @ " << now
                << ", cruise_id=" << cruise_id_
                << ", fixed_freq=" << cruise_modulation_freq_hz_
                << "Hz, initial_maxbw_baseline_bps="
                << initial_cruise_baseline_bw_.ToBitsPerSecond()
                << ", requested_amplitude_bps="
                << current_probe_amplitude_bps_
                << ", effective_amplitude_bps="
                << effective_probe_amplitude_bps_;
}

void FBBRSender::LeaveCruise(QuicTime now) {
  QUIC_DVLOG(2) << "FBBR: Leaving PROBE_CRUISE @ " << now;
  FinalizeCruise(now);
  previous_cruise_rtprop_updated_ = current_cruise_rtprop_updated_;
  in_cruise_ = false;
  fbbr_regime_i_or_iii_seen_this_cruise_ = false;
  model_.SetMinBandwidthSampleCollection(false);
  freq_tool_on_ = false;
  cruise_freq_tool_active_ = false;
  cruise_start_time_ = QuicTime::Zero();
  cruise_modulation_freq_hz_ = configured_modulation_freq_hz_;
  waveform_cruise_state_ = WaveformCruiseState::kDisabled;
  waveform_window_start_ = QuicTime::Zero();
  waveform_window_end_ = QuicTime::Zero();
}

void FBBRSender::InitializeFbbrRtpropFromModel() {
  if (fbbr_rtprop_valid_ &&
      fbbr_rtprop_source_time_ != QuicTime::Zero()) {
    return;
  }
  const TimeDelta model_rtprop = model_.MinRtt();
  if (model_rtprop.IsZero() || model_rtprop.IsInfinite()) {
    return;
  }
  fbbr_rtprop_valid_ = true;
  fbbr_rtprop_ = model_rtprop;
  fbbr_rtprop_source_time_ = model_.MinRttTimestamp();
}




void FBBRSender::PublishFbbrRtprop(TimeDelta rtprop,
                                      QuicTime source_time,
                                      bool from_probe_rtt) {
  if (rtprop.IsZero() || rtprop.IsInfinite() ||
      source_time == QuicTime::Zero()) {
    return;
  }
  if (fbbr_rtprop_valid_ &&
      source_time <= fbbr_rtprop_source_time_) {
    return;
  }
  fbbr_rtprop_valid_ = true;
  fbbr_rtprop_ = rtprop;
  fbbr_rtprop_source_time_ = source_time;
  model_.ForceUpdateMinRtt(rtprop, source_time);
  current_cruise_rtprop_updated_ = true;
  QUIC_DVLOG(2) << "FBBR: srtt_low/RTprop refreshed from "
                << (from_probe_rtt ? "ProbeRTT" : "Cruise")
                << " to " << rtprop << " @ " << source_time;
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
  if (!UsesFbbrServiceEnvelope() || !in_cruise_ ||
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


void FBBRSender::OnCongestionEventStarted(
    const Bbr2CongestionEvent& congestion_event) {
  // PROBE_UP and the reference BBR-R logic judge the entry flight before the
  // ACK/loss event that triggers the phase transition.
  latest_congestion_event_prior_inflight_ =
      congestion_event.prior_bytes_in_flight;
  latest_congestion_event_prior_inflight_valid_ = true;
  latest_congestion_event_inflight_ = congestion_event.bytes_in_flight;
  latest_congestion_event_inflight_valid_ = true;
  if (!UsesFbbrServiceEnvelope() && in_cruise_ &&
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
    unstable_episode_active_ = false;
    ClearTrustedBw("stable_closure");
    freq_tool_on_ = false;
    prev_v_round_ = 0.0;
  }
}

void FBBRSender::UpdateFreqWeightAndToolState() {
  if (bbr_stable_) {
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
    freq_tool_needed_ = false;
    unstable_episode_active_ = false;
    ClearTrustedBw("stable_closure");
    freq_tool_on_ = false;
    return;
  }

  freq_tool_on_ = freq_tool_needed_ && BaseShouldOscillate();
  if (in_cruise_ && freq_tool_on_) {
    cruise_freq_tool_active_ = true;
  }
}

void FBBRSender::ClearTrustedBw(const char* reason) {
  const std::string clear_reason = reason == nullptr ? "unknown" : reason;
  const bool preserve_fbbr_probe_rtt =
      IsFbbr() && mode_ == Bbr2Mode::PROBE_RTT &&
      (clear_reason == "non_probe_bw" ||
       clear_reason == "stable_closure");
  const bool preserve_service_fair_control =
      IsFbbrServiceFair() && trusted_bw_valid_ &&
      IsFinitePositiveBandwidth(trusted_bw_) &&
      (clear_reason == "stable_closure" ||
       (mode_ == Bbr2Mode::PROBE_RTT &&
        clear_reason == "non_probe_bw"));
  if (preserve_fbbr_probe_rtt || preserve_service_fair_control) {
    trusted_bw_invalid_reason_ =
        preserve_fbbr_probe_rtt || mode_ == Bbr2Mode::PROBE_RTT
            ? "probe_rtt_preserved"
            : "stable_control_preserved";
    ClearTrustedBwApplication(reason);
    return;
  }
  trusted_bw_ = QuicBandwidth::Zero();
  trusted_bw_valid_ = false;
  model_.ClearBdpBandwidthOverride();
  trusted_bw_conf_ = 0.0;
  trusted_bw_source_ = kTrustedBwSourceNone;
  trusted_bw_cruise_id_ = 0;
  trusted_bw_invalid_reason_ = clear_reason;
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

void FBBRSender::PublishTrustedBwSelection(
    const TrustedBwSelectionResult& selection) {
  selection_native_bw_ = selection.native_bw;
  // Preserve the established CSV columns while FBBR no longer runs the
  // legacy spectral selector that produced them.
  drate_spectral_integrity_score_ = 0.0;
  srtt_spectral_integrity_score_ = 0.0;
  joint_spectral_integrity_score_ = 0.0;
  drate_spectral_gate_pass_ = false;
  srtt_spectral_gate_pass_ = false;
  dual_signal_spectral_gate_pass_ = false;
  limiting_spectral_signal_ = "NOT_APPLICABLE";
  merged_rescue_attempted_ = false;
  merged_rescue_success_ = false;
  trusted_bw_selection_compute_us_ = 0;
  normal_window_count_ = 0;
  merged_window_count_ = 0;
  spectral_invalid_count_ = 0;

  const bool valid_selection = selection.trusted_bw_valid &&
      IsFinitePositiveBandwidth(selection.trusted_bw);
  if (!valid_selection) {
    ClearTrustedBw("invalid_waveform_trusted_bw");
    return;
  }

  trusted_bw_ = selection.trusted_bw;
  trusted_bw_valid_ = true;
  // FBBR and ServiceFair keep BBRv2's native cwnd/BDP model and enforce
  // their envelope only at the final send-allowance interface.
  trusted_bw_conf_ = selection.trusted_bw_conf;
  trusted_bw_source_ = selection.trusted_bw_source;
  trusted_bw_cruise_id_ =
      static_cast<uint64_t>(std::max<int64_t>(0, cruise_id_));
  trusted_bw_fresh_ = true;
  trusted_bw_application_valid_ = true;
  trusted_bw_ready_for_post_cruise_ = true;
  trusted_bw_application_phase_ = "POST_CRUISE_READY";
  trusted_bw_invalid_reason_ = "none";
  if (HasUsableFbbrPreviousTrustedBw()) {
    current_injection_baseline_bw_ = trusted_bw_;
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
  const double trace_freq_weight = bbr_stable_
      ? 0.0
      : (UsesFbbrServiceEnvelope()
             ? 1.0
             : Clamp01(1.0 - static_cast<double>(stable_cnt_) /
                                  static_cast<double>(stable_rounds_)));
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
      << trace_freq_weight << ","
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
      << model_.min_bandwidth_sample_correction() << ","
      << model_.min_bandwidth_filter_input().ToBitsPerSecond() << ","
      << model_.MinBandwidth().ToBitsPerSecond() << ","
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

void FBBRSender::PublishFbbrCruiseTrustedBw() {
  TrustedBwSelectionResult selection = {
      BandwidthEstimate(),
      QuicBandwidth::Zero(),
      false,
      0.0,
      kTrustedBwSourceNone};
  if (!IsFinitePositiveBandwidth(selection.native_bw)) {
    selection.native_bw = BandwidthEstimate();
  }

  const std::string waveform_candidate_source =
      trusted_bw_candidate_source_ == nullptr
          ? kTrustedBwSourceNone : trusted_bw_candidate_source_;
  const bool waveform_candidate_source_valid =
      waveform_candidate_source == kTrustedBwSourceFbbrWindowMean ||
      waveform_candidate_source == kTrustedBwSourceFbbrWindowDeliveredRate;
  const bool waveform_frequency_ready =
      fbbr_regime_ii_seen_this_cruise_ &&
      trusted_baseline_locked_ &&
      waveform_candidate_source_valid &&
      IsFinitePositiveBandwidth(trusted_bw_candidate_);

  if (waveform_frequency_ready) {
    selection.trusted_bw = trusted_bw_candidate_;
    selection.trusted_bw_valid = true;
    selection.trusted_bw_conf = 1.0;
    selection.trusted_bw_source = trusted_bw_candidate_source_;
    AnchorGuardFilter(selection.trusted_bw);
  } else if (guard_updated_this_cruise_ && guard_filter_valid_ &&
             IsFinitePositiveBandwidth(guard_filter_stage2_)) {
    selection.trusted_bw = guard_filter_stage2_;
    selection.trusted_bw_valid = true;
    selection.trusted_bw_conf = 1.0;
    selection.trusted_bw_source = kTrustedBwSourceGuardFilter;
  } else if (HasUsableFbbrPreviousTrustedBw()) {
    selection.trusted_bw = trusted_bw_;
    selection.trusted_bw_valid = true;
    selection.trusted_bw_conf = trusted_bw_conf_;
    selection.trusted_bw_source = kTrustedBwSourcePreviousTrusted;
  } else {
    QuicBandwidth native_fallback = model_.MaxBandwidth();
    if (!IsFinitePositiveBandwidth(native_fallback)) {
      native_fallback = initial_cruise_baseline_bw_;
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
  }

  if (IsFbbrServiceFair()) {
    ApplyFbbrServiceFairTrustedBwCorrection(
        &selection, waveform_frequency_ready);
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

void FBBRSender::OnConnectionMigration() {
  if (IsFbbrServiceFair()) {
    ResetFbbrServiceFairState();
    ClearTrustedBw("connection_migration");
  }
  Bbr2Sender::OnConnectionMigration();
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
  const QuicBandwidth fbbr_max_bw_before =
      UsesFbbrServiceEnvelope() ? model_.MaxBandwidth()
                                : QuicBandwidth::Zero();

  // Keep the ACK-rate sample raw for waveform analysis, but remove the
  // estimated positive waveform excursion from BBR's max-bandwidth sample.
  max_bw_actual_fluctuation_amplitude_bps_ =
      ShouldOscillate()
          ? CurrentActualDeliveryFluctuationAmplitudeBps()
          : 0.0;
  max_bw_attenuation_factor_ = CurrentMaxBwAttenuationFactor();
  model_.SetMaxBandwidthSampleAttenuation(max_bw_attenuation_factor_);
  model_.SetMinBandwidthSampleCorrection(
      UsesFbbrServiceEnvelope() ? CurrentMinBwCorrectionFactor() : 1.0);

  Bbr2Sender::OnCongestionEvent(rtt_updated,
                                prior_in_flight,
                                event_time,
                                acked_packets,
                                lost_packets);
  if (IsFbbrServiceFair() &&
      mode_before_event != Bbr2Mode::STARTUP &&
      mode_ == Bbr2Mode::STARTUP) {
    ResetFbbrServiceFairState();
  }
  const QuicBandwidth fbbr_max_bw_after =
      UsesFbbrServiceEnvelope() ? model_.MaxBandwidth()
                                : QuicBandwidth::Zero();
  const bool fbbr_max_bw_updated =
      UsesFbbrServiceEnvelope() &&
      IsFinitePositiveBandwidth(fbbr_max_bw_after) &&
      fbbr_max_bw_after != fbbr_max_bw_before;

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
  if (fbbr_max_bw_updated) {
    RecordFbbrMaxBwUpdateForMaxSrtt(event_time, fbbr_max_bw_after);
  }
  if (UsesFbbrServiceEnvelope()) {
    FinalizePendingFbbrMaxSrttObservations(event_time);
  }
  last_ack_time_ = event_time;

  // Record the cumulative counter before a due waveform analysis so an ACK
  // exactly at the window end contributes to Delivered(t_end). Points after
  // the boundary remain excluded by the interval lookup.
  if (UsesFbbrServiceEnvelope() && !acked_packets.empty()) {
    RecordFbbrDeliveredPoint(
        event_time,
        static_cast<uint64_t>(model_.total_bytes_acked()),
        model_.is_app_limited());
  }
  if (IsFbbrServiceFair() && !acked_packets.empty()) {
    UpdateFbbrServiceFairSignals(event_time);
  }

  if (in_cruise_ && mode_ == Bbr2Mode::PROBE_BW &&
      GetCurrentProbeBwPhase() == Bbr2ProbeBwMode::CyclePhase::PROBE_CRUISE) {
    // FBBR now uses the time-domain waveform detector exclusively.
    RunWaveformCruiseStateMachine(event_time);
  }
  if (UsesFbbrServiceEnvelope() && !acked_packets.empty()) {
    UpdateFbbrTelemetry(
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
  const bool use_previous_trusted_bw =
      UsesFbbrServiceEnvelope() && mode_ == Bbr2Mode::PROBE_BW &&
      HasUsableFbbrPreviousTrustedBw() &&
      (!IsCruisePhase(phase) ||
       !fbbr_regime_i_or_iii_seen_this_cruise_);

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
      IsCruisePhase(phase) && in_cruise_ &&
      waveform_cruise_state_ != WaveformCruiseState::kDisabled;

  if (use_waveform_cruise_baseline) {
    pacing_base_bw = current_injection_baseline_bw_;
    pacing_base_source =
        FBBRPacingBaseSource::kWaveformCruiseBaseline;
    trusted_bw_application_phase_ = "CRUISE";
  } else if (use_previous_trusted_bw) {
    pacing_base_bw = trusted_bw_;
    pacing_base_source = FBBRPacingBaseSource::kTrustedBw;
    trusted_bw_application_phase_ = PhaseApplicationName(phase);
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
  } else if (use_previous_trusted_bw) {
    baseline_pacing = static_cast<float>(phase_gain) * trusted_bw_;
  } else if (use_trusted_bw) {
    baseline_pacing =
        static_cast<float>(phase_gain) * pacing_base_bw;
  }
  // FBBR's base target follows the exact same baseline, phase, native pacing
  // limits, units, and integer rounding as the commanded target.  It removes
  // only positive phase and waveform increments; all negative gains remain.
  QuicBandwidth base_target_before_wave = baseline_pacing;
  if (UsesFbbrServiceEnvelope()) {
    const float base_phase_gain =
        static_cast<float>(std::min(phase_gain, 1.0));
    if (use_waveform_cruise_baseline) {
      base_target_before_wave = current_injection_baseline_bw_;
    } else if (use_previous_trusted_bw) {
      base_target_before_wave = base_phase_gain * trusted_bw_;
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
  const bool base_should_oscillate = BaseShouldOscillate();
  const bool should_oscillate = use_waveform_cruise_baseline
      ? base_should_oscillate
      : enable_convergence_gate_control_
          ? (base_should_oscillate && !bbr_stable_)
          : base_should_oscillate;
  freq_tool_on_ = should_oscillate;
  const uint64_t requested_amplitude_bps =
      should_oscillate
          ? (use_waveform_cruise_baseline
                 ? current_probe_amplitude_bps_
                 : GetCurrentAmplitudeBps())
          : 0;
  const uint64_t effective_amplitude_bps =
      should_oscillate
          ? (use_waveform_cruise_baseline
                 ? effective_probe_amplitude_bps_
                 : requested_amplitude_bps)
          : 0;
  const int64_t amplitude_bps = SaturateBpsToInt64(requested_amplitude_bps);
  const int64_t effective_amplitude_bps_int64 =
      SaturateBpsToInt64(effective_amplitude_bps);
  const double triangle_wave =
      should_oscillate ? TriangleWave(current_time_) : 0.0;
  const int64_t offset_bps =
      ScaleTriangleOffset(effective_amplitude_bps, triangle_wave);
  const int64_t final_bps = AddPacingOffsetWithFloor(
      SaturateBpsToInt64(baseline_pacing.ToBitsPerSecond()), offset_bps,
      minimum_pacing_rate_bps_);
  const QuicBandwidth final_pacing =
      QuicBandwidth::FromBitsPerSecond(static_cast<uint64_t>(final_bps));
  const QuicBandwidth returned_pacing =
      (!should_oscillate && !use_trusted_bw &&
       !use_previous_trusted_bw &&
       !use_waveform_cruise_baseline)
          ? native_pacing
          : final_pacing;
  QuicBandwidth returned_base_target = returned_pacing;
  if (UsesFbbrServiceEnvelope()) {
    const int64_t base_offset_bps = std::min<int64_t>(0, offset_bps);
    const int64_t base_final_bps = AddPacingOffsetWithFloor(
        SaturateBpsToInt64(base_target_before_wave.ToBitsPerSecond()),
        base_offset_bps, minimum_pacing_rate_bps_);
    const QuicBandwidth base_final =
        QuicBandwidth::FromBitsPerSecond(
            static_cast<uint64_t>(base_final_bps));
    returned_base_target =
        (!should_oscillate && !use_trusted_bw &&
         !use_previous_trusted_bw &&
         !use_waveform_cruise_baseline)
            ? base_target_before_wave
            : base_final;
    returned_base_target =
        std::min(returned_base_target, returned_pacing);
  }
  RecordFbbrRateTargets(
      current_time_, returned_pacing, returned_base_target);

  EmitPacingTrace(native_bw,
                  pacing_base_bw,
                  pacing_base_source,
                  native_pacing,
                  returned_pacing,
                  phase_gain,
                  amplitude_bps,
                  effective_amplitude_bps_int64,
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

bool FBBRSender::IsFbbr() const {
  return GetCongestionControlType() == kFBBR;
}

bool FBBRSender::IsFbbrServiceFair() const {
  return GetCongestionControlType() == kFBBRServiceFair;
}

bool FBBRSender::UsesFbbrServiceEnvelope() const {
  return IsFbbr() || IsFbbrServiceFair();
}

bool FBBRSender::HasUsableFbbrPreviousTrustedBw() const {
  return UsesFbbrServiceEnvelope() && trusted_bw_valid_ &&
      IsFinitePositiveBandwidth(trusted_bw_);
}


TimeDelta FBBRSender::CurrentFbbrRtprop() const {
  TimeDelta rtprop =
      fbbr_rtprop_valid_ ? fbbr_rtprop_
                                  : model_.MinRtt();
  if (rtprop.IsZero() || rtprop.IsInfinite() ||
      rtprop.ToMicroseconds() <= 0) {
    return TimeDelta::Zero();
  }
  return rtprop;
}


QuicByteCount FBBRSender::GetCongestionWindow() const {
  if (UsesFbbrServiceEnvelope()) {
    return ApplyFbbrInflightEnvelope(cwnd_);
  }
  return cwnd_;
}


void FBBRSender::RecordFbbrRateTargets(
    QuicTime now,
    QuicBandwidth target_rate,
    QuicBandwidth base_target_rate) const {
  if (!UsesFbbrServiceEnvelope() ||
      !IsFinitePositiveBandwidth(target_rate) ||
      !IsFinitePositiveBandwidth(base_target_rate)) {
    return;
  }
  base_target_rate = std::min(base_target_rate, target_rate);
  if (fbbr_rate_history_.empty()) {
    fbbr_rate_history_.push_back(
        {now, QuicTime::Zero(), target_rate, base_target_rate});
    fbbr_last_target_rate_ = target_rate;
    fbbr_last_base_target_rate_ = base_target_rate;
    return;
  }

  FbbrRateSegment& open = fbbr_rate_history_.back();
  if (now < open.start) {
    fbbr_rate_history_integrity_valid_ = false;
    return;
  }
  if (target_rate == open.target_rate &&
      base_target_rate == open.base_target_rate) {
    return;
  }
  if (now == open.start && open.end == QuicTime::Zero()) {
    open.target_rate = target_rate;
    open.base_target_rate = base_target_rate;
    fbbr_last_target_rate_ = target_rate;
    fbbr_last_base_target_rate_ = base_target_rate;
    return;
  }
  open.end = now;
  fbbr_rate_history_.push_back(
      {now, QuicTime::Zero(), target_rate, base_target_rate});
  fbbr_last_target_rate_ = target_rate;
  fbbr_last_base_target_rate_ = base_target_rate;

  if (!fbbr_max_rtprop_seen_.IsZero() &&
      !fbbr_max_rtprop_seen_.IsInfinite() &&
      now >= QuicTime::Zero() + fbbr_max_rtprop_seen_) {
    const QuicTime cutoff = now - fbbr_max_rtprop_seen_;
    // Keep the segment containing the integration boundary as its anchor.
    while (fbbr_rate_history_.size() > 1 &&
           fbbr_rate_history_[1].start <= cutoff) {
      fbbr_rate_history_.pop_front();
    }
  }
}

bool FBBRSender::HasFullFbbrTargetHistory(
    QuicTime now,
    TimeDelta rtprop) const {
  if (!UsesFbbrServiceEnvelope() ||
      !fbbr_rate_history_integrity_valid_ ||
      rtprop.IsZero() || rtprop.IsInfinite() ||
      rtprop.ToMicroseconds() <= 0 ||
      fbbr_rate_history_.empty() ||
      now < QuicTime::Zero() + rtprop) {
    return false;
  }
  if (fbbr_max_rtprop_seen_.IsZero() ||
      rtprop > fbbr_max_rtprop_seen_) {
    fbbr_max_rtprop_seen_ = rtprop;
  }
  const QuicTime window_start = now - rtprop;
  QuicTime covered_until = window_start;
  for (const FbbrRateSegment& segment : fbbr_rate_history_) {
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

QuicByteCount FBBRSender::ComputeFbbrPlannedInflightBytes(
    QuicTime now,
    TimeDelta rtprop) const {
  if (!HasFullFbbrTargetHistory(now, rtprop)) {
    return 0;
  }
  const QuicTime window_start = now - rtprop;
  long double bytes = 0.0L;
  for (const FbbrRateSegment& segment : fbbr_rate_history_) {
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

QuicByteCount FBBRSender::ComputeFbbrPositiveProbeCreditBytes(
    QuicTime now,
    TimeDelta rtprop) const {
  if (!HasFullFbbrTargetHistory(now, rtprop)) {
    return 0;
  }
  const QuicTime window_start = now - rtprop;
  long double bytes = 0.0L;
  for (const FbbrRateSegment& segment : fbbr_rate_history_) {
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

void FBBRSender::RecordFbbrDeliveredPoint(
    QuicTime now,
    uint64_t cumulative_delivered,
    bool app_limited) {
  if (!UsesFbbrServiceEnvelope()) {
    return;
  }
  if (fbbr_delivered_history_.empty()) {
    fbbr_delivered_history_.push_back(
        {now, cumulative_delivered, app_limited});
    return;
  }
  FbbrDeliveredPoint& last = fbbr_delivered_history_.back();
  if (now < last.timestamp) {
    fbbr_delivered_history_integrity_valid_ = false;
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
    ResetFbbrServiceFairState();
    fbbr_delivered_history_.clear();
    fbbr_last_counter_reset_time_ = now;
  }
  fbbr_delivered_history_.push_back(
      {now, cumulative_delivered, app_limited});

  TimeDelta retention = fbbr_max_rtprop_seen_;
  if (cruise_modulation_freq_hz_ > 0.0 &&
      std::isfinite(cruise_modulation_freq_hz_) &&
      waveform_max_window_periods_ > 0.0 &&
      std::isfinite(waveform_max_window_periods_)) {
    const TimeDelta waveform_retention = TimeDelta::FromMicroseconds(
        std::max<int64_t>(1, static_cast<int64_t>(std::ceil(
            1000000.0 * waveform_max_window_periods_ /
            cruise_modulation_freq_hz_))));
    if (retention.IsZero() || retention.IsInfinite() ||
        waveform_retention > retention) {
      retention = waveform_retention;
    }
  }
  if (!retention.IsZero() && !retention.IsInfinite() &&
      now >= QuicTime::Zero() + retention) {
    QuicTime cutoff = now - retention;
    if (in_cruise_ && waveform_window_start_ != QuicTime::Zero() &&
        waveform_window_start_ < now) {
      cutoff = std::min(cutoff, waveform_window_start_);
    }
    // Retain the newest step point at or before the boundary.
    while (fbbr_delivered_history_.size() > 1 &&
           fbbr_delivered_history_[1].timestamp <= cutoff) {
      fbbr_delivered_history_.pop_front();
    }
  }
}

bool FBBRSender::ComputeFbbrIntervalDeliveryRateBps(
    QuicTime window_start,
    QuicTime window_end,
    double* delivery_rate_bps) const {
  if (delivery_rate_bps != nullptr) {
    *delivery_rate_bps = 0.0;
  }
  if (!UsesFbbrServiceEnvelope() || delivery_rate_bps == nullptr ||
      !fbbr_delivered_history_integrity_valid_ ||
      window_end <= window_start ||
      fbbr_delivered_history_.empty()) {
    return false;
  }
  if (fbbr_last_counter_reset_time_ != QuicTime::Zero() &&
      fbbr_last_counter_reset_time_ >= window_start &&
      fbbr_last_counter_reset_time_ <= window_end) {
    return false;
  }

  bool start_found = false;
  bool end_found = false;
  uint64_t delivered_at_start = 0;
  uint64_t delivered_at_end = 0;
  for (const FbbrDeliveredPoint& point : fbbr_delivered_history_) {
    if (point.timestamp <= window_start) {
      start_found = true;
      delivered_at_start = point.cumulative_delivered_bytes;
    }
    if (point.timestamp <= window_end) {
      end_found = true;
      delivered_at_end = point.cumulative_delivered_bytes;
    } else {
      break;
    }
  }
  const int64_t duration_us =
      (window_end - window_start).ToMicroseconds();
  if (!start_found || !end_found || duration_us <= 0 ||
      delivered_at_end < delivered_at_start) {
    return false;
  }
  const long double rate_bps =
      static_cast<long double>(delivered_at_end - delivered_at_start) *
      8000000.0L / static_cast<long double>(duration_us);
  if (!std::isfinite(rate_bps) || rate_bps <= 0.0L ||
      rate_bps > static_cast<long double>(
                     std::numeric_limits<int64_t>::max())) {
    return false;
  }
  *delivery_rate_bps = static_cast<double>(rate_bps);
  return std::isfinite(*delivery_rate_bps) && *delivery_rate_bps > 0.0;
}

bool FBBRSender::ComputeFbbrTimeWeightedSrttMeanMs(
    QuicTime window_start,
    QuicTime window_end,
    double* srtt_mean_ms) const {
  if (srtt_mean_ms != nullptr) {
    *srtt_mean_ms = 0.0;
  }
  if (!UsesFbbrServiceEnvelope() || srtt_mean_ms == nullptr ||
      window_end <= window_start || srtt_history_.empty()) {
    return false;
  }

  bool anchor_found = false;
  double current_srtt_ms = 0.0;
  QuicTime cursor = window_start;
  long double weighted_srtt_ms_us = 0.0L;
  int64_t covered_us = 0;

  for (const FBBRRttSample& sample : srtt_history_) {
    if (sample.time > window_end) {
      break;
    }
    if (!std::isfinite(sample.rtt_ms) || sample.rtt_ms <= 0.0) {
      continue;
    }
    if (sample.time <= window_start) {
      anchor_found = true;
      current_srtt_ms = sample.rtt_ms;
      continue;
    }
    if (!anchor_found) {
      return false;
    }
    if (sample.time >= window_end) {
      break;
    }
    const int64_t duration_us = (sample.time - cursor).ToMicroseconds();
    if (duration_us > 0) {
      weighted_srtt_ms_us +=
          static_cast<long double>(current_srtt_ms) * duration_us;
      covered_us += duration_us;
      cursor = sample.time;
    }
    current_srtt_ms = sample.rtt_ms;
  }

  if (!anchor_found) {
    return false;
  }
  const int64_t final_duration_us =
      (window_end - cursor).ToMicroseconds();
  if (final_duration_us > 0) {
    weighted_srtt_ms_us +=
        static_cast<long double>(current_srtt_ms) * final_duration_us;
    covered_us += final_duration_us;
  }
  const int64_t window_duration_us =
      (window_end - window_start).ToMicroseconds();
  if (window_duration_us <= 0 || covered_us != window_duration_us ||
      !std::isfinite(weighted_srtt_ms_us) ||
      weighted_srtt_ms_us <= 0.0L) {
    return false;
  }
  const long double mean_ms =
      weighted_srtt_ms_us / static_cast<long double>(window_duration_us);
  if (!std::isfinite(mean_ms) || mean_ms <= 0.0L) {
    return false;
  }
  *srtt_mean_ms = static_cast<double>(mean_ms);
  return std::isfinite(*srtt_mean_ms) && *srtt_mean_ms > 0.0;
}

bool FBBRSender::HasValidFbbrServiceHistory(
    QuicTime now,
    TimeDelta rtprop,
    bool current_app_limited,
    bool* app_limited_contaminated) const {
  if (app_limited_contaminated != nullptr) {
    *app_limited_contaminated = false;
  }
  if (!UsesFbbrServiceEnvelope() ||
      !fbbr_delivered_history_integrity_valid_ ||
      rtprop.IsZero() || rtprop.IsInfinite() ||
      rtprop.ToMicroseconds() <= 0 ||
      fbbr_delivered_history_.empty() ||
      now < QuicTime::Zero() + rtprop) {
    return false;
  }
  const QuicTime window_start = now - rtprop;
  size_t anchor_index = fbbr_delivered_history_.size();
  for (size_t i = 0; i < fbbr_delivered_history_.size(); ++i) {
    if (fbbr_delivered_history_[i].timestamp <= window_start) {
      anchor_index = i;
    } else {
      break;
    }
  }
  if (anchor_index == fbbr_delivered_history_.size()) {
    return false;
  }
  if (fbbr_last_counter_reset_time_ != QuicTime::Zero() &&
      fbbr_last_counter_reset_time_ >= window_start &&
      fbbr_last_counter_reset_time_ <= now) {
    return false;
  }

  bool contaminated = current_app_limited;
  uint64_t previous =
      fbbr_delivered_history_[anchor_index]
          .cumulative_delivered_bytes;
  contaminated =
      contaminated ||
      fbbr_delivered_history_[anchor_index].app_limited;
  for (size_t i = anchor_index + 1;
       i < fbbr_delivered_history_.size(); ++i) {
    const FbbrDeliveredPoint& point = fbbr_delivered_history_[i];
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

void FBBRSender::ResetFbbrServiceFairState() {
  if (!IsFbbrServiceFair()) {
    return;
  }
  service_fair_service_rate_ = QuicBandwidth::Zero();
  service_fair_previous_service_rate_ = QuicBandwidth::Zero();
  service_fair_qdelay_ewma_ = TimeDelta::Zero();
  service_fair_previous_qdelay_ewma_ = TimeDelta::Zero();
  service_fair_cycle_count_ = 0;
  service_fair_qdelay_valid_ = false;
  service_fair_previous_qdelay_valid_ = false;
  service_fair_service_history_valid_ = false;
  service_fair_signal_reset_time_ = current_time_;
  service_fair_last_update_cruise_id_ = -1;
  service_fair_action_ = FbbrServiceFairAction::kNotRun;
  service_fair_qdelay_trend_ = TimeDelta::Zero();
  service_fair_service_rate_change_ = 0.0;
  service_fair_alpha_bps_ = 0.0;
  service_fair_raw_regime_candidate_bps_ = 0.0;
  service_fair_final_regime_candidate_bps_ = 0.0;
  service_fair_last_valid_regime_seen_this_cruise_ = false;
  service_fair_last_valid_regime_this_cruise_ =
      WaveformClassification::kInconclusive;
}

void FBBRSender::UpdateFbbrServiceFairSignals(QuicTime now) {
  if (!IsFbbrServiceFair() || mode_ != Bbr2Mode::PROBE_BW) {
    return;
  }
  const TimeDelta rtprop = CurrentFbbrRtprop();
  const TimeDelta srtt = CurrentSmoothedRtt();
  if (!rtprop.IsZero() && !rtprop.IsInfinite() &&
      !srtt.IsZero() && !srtt.IsInfinite()) {
    const int64_t sample_us = std::max<int64_t>(
        0, srtt.ToMicroseconds() - rtprop.ToMicroseconds());
    if (!service_fair_qdelay_valid_) {
      service_fair_qdelay_ewma_ = TimeDelta::FromMicroseconds(sample_us);
      service_fair_qdelay_valid_ = true;
    } else {
      const int64_t old_us = service_fair_qdelay_ewma_.ToMicroseconds();
      const int64_t ewma_us = static_cast<int64_t>(std::llround(
          (7.0 * static_cast<double>(old_us) +
           static_cast<double>(sample_us)) /
          8.0));
      service_fair_qdelay_ewma_ =
          TimeDelta::FromMicroseconds(std::max<int64_t>(0, ewma_us));
    }
  }

  service_fair_service_history_valid_ = false;
  if (rtprop.IsZero() || rtprop.IsInfinite() ||
      rtprop.ToMicroseconds() <= 0 ||
      now < QuicTime::Zero() + rtprop) {
    return;
  }
  const QuicTime window_start = now - rtprop;
  if (service_fair_signal_reset_time_ != QuicTime::Zero() &&
      service_fair_signal_reset_time_ >= window_start) {
    return;
  }
  bool app_limited_contaminated = false;
  if (!HasValidFbbrServiceHistory(
          now, rtprop, model_.is_app_limited(),
          &app_limited_contaminated)) {
    return;
  }
  double service_rate_bps = 0.0;
  if (!ComputeFbbrIntervalDeliveryRateBps(
          window_start, now, &service_rate_bps)) {
    return;
  }
  service_fair_service_rate_ = BandwidthFromBps(service_rate_bps);
  service_fair_service_history_valid_ =
      IsFinitePositiveBandwidth(service_fair_service_rate_);
}

double FBBRSender::ComputeFbbrServiceFairAlphaBps(TimeDelta rtprop) {
  if (rtprop.IsZero() || rtprop.IsInfinite() ||
      rtprop.ToMicroseconds() <= 0) {
    return 0.0;
  }
  const double rtprop_s =
      static_cast<double>(rtprop.ToMicroseconds()) / 1000000.0;
  return 0.5 * 8.0 * static_cast<double>(kDefaultTCPMSS) /
      rtprop_s;
}

double FBBRSender::ApplyFbbrServiceFairRegimeIIIControlLimit(
    double raw_candidate_bps,
    double current_baseline_bps,
    double service_rate_bps,
    bool service_history_valid) {
  if (!std::isfinite(raw_candidate_bps) || raw_candidate_bps <= 0.0 ||
      !std::isfinite(current_baseline_bps) ||
      current_baseline_bps <= 0.0) {
    return raw_candidate_bps;
  }
  const double yield_cap_bps =
      kFbbrServiceFairBeta * current_baseline_bps;
  double next_bps = std::min(raw_candidate_bps, yield_cap_bps);
  if (service_history_valid &&
      std::isfinite(service_rate_bps) && service_rate_bps > 0.0) {
    const double service_floor_bps =
        std::min(yield_cap_bps, 0.98 * service_rate_bps);
    next_bps = std::max(next_bps, service_floor_bps);
  }
  return std::min(yield_cap_bps, next_bps);
}

void FBBRSender::ApplyFbbrServiceFairTrustedBwCorrection(
    TrustedBwSelectionResult* selection,
    bool regime_ii_candidate_selected) {
  if (!IsFbbrServiceFair() || selection == nullptr ||
      !selection->trusted_bw_valid ||
      !IsFinitePositiveBandwidth(selection->trusted_bw) ||
      !IsFinitePositiveBandwidth(current_injection_baseline_bw_)) {
    return;
  }

  const double candidate_bps =
      static_cast<double>(
          selection->trusted_bw.ToBitsPerSecond());
  const double final_cruise_baseline_bps =
      static_cast<double>(
          current_injection_baseline_bw_.ToBitsPerSecond());
  double corrected_bps = candidate_bps;

  // A legal Regime II cumulative-delivery interval is the highest-priority
  // measurement and must not be altered by either fairness or MaxBw.
  if (regime_ii_candidate_selected) {
    service_fair_raw_regime_candidate_bps_ = candidate_bps;
    service_fair_final_regime_candidate_bps_ = candidate_bps;
    EmitFbbrServiceFairTrace(
        current_time_, "TRUSTED_PUBLISH",
        WaveformClassification::kFullLoad);
    return;
  }

  if (service_fair_last_valid_regime_seen_this_cruise_ &&
      service_fair_last_valid_regime_this_cruise_ ==
          WaveformClassification::kUnderload) {
    corrected_bps =
        std::max(candidate_bps, final_cruise_baseline_bps);
    const QuicBandwidth max_bw = model_.MaxBandwidth();
    if (IsFinitePositiveBandwidth(max_bw)) {
      corrected_bps = std::min(
          corrected_bps,
          static_cast<double>(max_bw.ToBitsPerSecond()));
    }
  } else if (service_fair_last_valid_regime_seen_this_cruise_ &&
             service_fair_last_valid_regime_this_cruise_ ==
                 WaveformClassification::kOverload) {
    corrected_bps =
        std::min(candidate_bps, final_cruise_baseline_bps);
    if (service_fair_service_history_valid_ &&
        IsFinitePositiveBandwidth(service_fair_service_rate_)) {
      const double service_floor_bps = std::min(
          final_cruise_baseline_bps,
          0.98 * static_cast<double>(
              service_fair_service_rate_.ToBitsPerSecond()));
      corrected_bps =
          std::max(corrected_bps, service_floor_bps);
    }
  } else if (service_fair_action_ ==
                 FbbrServiceFairAction::kAdditiveIncrease ||
             service_fair_action_ ==
                 FbbrServiceFairAction::kMultiplicativeDecrease) {
    // A Cruise may end before the waveform classifier has a complete
    // decision.  Preserve a completed queue/service AIMD step on Bcruise;
    // HOLD and invalid histories leave the normal candidate untouched.
    corrected_bps = final_cruise_baseline_bps;
  }
  corrected_bps = std::max(
      static_cast<double>(minimum_pacing_rate_bps_),
      corrected_bps);

  service_fair_raw_regime_candidate_bps_ = candidate_bps;
  service_fair_final_regime_candidate_bps_ = corrected_bps;
  selection->trusted_bw = BandwidthFromBps(corrected_bps);
  selection->trusted_bw_valid =
      IsFinitePositiveBandwidth(selection->trusted_bw);
  EmitFbbrServiceFairTrace(
      current_time_, "TRUSTED_PUBLISH",
      service_fair_last_valid_regime_seen_this_cruise_
          ? service_fair_last_valid_regime_this_cruise_
          : WaveformClassification::kInconclusive);
}

const char* FBBRSender::FbbrServiceFairActionName(
    FbbrServiceFairAction action) {
  switch (action) {
    case FbbrServiceFairAction::kAdditiveIncrease:
      return "ADDITIVE_INCREASE";
    case FbbrServiceFairAction::kMultiplicativeDecrease:
      return "MULTIPLICATIVE_DECREASE";
    case FbbrServiceFairAction::kHold:
      return "HOLD";
    case FbbrServiceFairAction::kSkipAppLimited:
      return "SKIP_APP_LIMITED";
    case FbbrServiceFairAction::kSkipInvalidHistory:
      return "SKIP_INVALID_HISTORY";
    default:
      return "NOT_RUN";
  }
}

void FBBRSender::EmitFbbrServiceFairTrace(
    QuicTime now,
    const char* event,
    WaveformClassification classification) const {
  if (!IsFbbrServiceFair() || !cruise_load_trace_cb_) {
    return;
  }
  const double time_s = now == QuicTime::Zero()
      ? 0.0
      : static_cast<double>(
            (now - QuicTime::Zero()).ToMicroseconds()) /
            1000000.0;
  const double qdelay_ewma_ms =
      static_cast<double>(service_fair_qdelay_ewma_.ToMicroseconds()) / 1000.0;
  const double qdelay_trend_ms =
      static_cast<double>(service_fair_qdelay_trend_.ToMicroseconds()) / 1000.0;
  const bool baseline_valid =
      IsFinitePositiveBandwidth(current_injection_baseline_bw_);
  std::ostringstream row;
  row << time_s << ","
      << trace_flow_id_ << ","
      << cruise_id_ << ","
      << (event == nullptr ? "UNKNOWN" : event) << ","
      << WaveformClassificationName(classification) << ","
      << (baseline_valid
              ? current_injection_baseline_bw_.ToBitsPerSecond() : 0) << ","
      << (baseline_valid ? "true" : "false") << ","
      << service_fair_cycle_count_ << ","
      << FbbrServiceFairActionName(service_fair_action_) << ","
      << qdelay_ewma_ms << ","
      << qdelay_trend_ms << ","
      << (IsFinitePositiveBandwidth(service_fair_service_rate_)
              ? service_fair_service_rate_.ToBitsPerSecond() : 0) << ","
      << service_fair_service_rate_change_ << ","
      << service_fair_alpha_bps_ << ","
      << kFbbrServiceFairBeta << ","
      << service_fair_raw_regime_candidate_bps_ << ","
      << service_fair_final_regime_candidate_bps_;
  cruise_load_trace_cb_(time_s, time_s, 0.0, 0.0, 0.0, 0.0,
                        "SERVICE_FAIRNESS", false, row.str());
}

void FBBRSender::RunFbbrServiceFairCycleUpdate(QuicTime now) {
  if (!IsFbbrServiceFair() || mode_ != Bbr2Mode::PROBE_BW ||
      service_fair_last_update_cruise_id_ == cruise_id_) {
    return;
  }
  service_fair_last_update_cruise_id_ = cruise_id_;
  service_fair_qdelay_trend_ = TimeDelta::Zero();
  service_fair_service_rate_change_ = 0.0;
  if (!IsFinitePositiveBandwidth(current_injection_baseline_bw_)) {
    service_fair_action_ = FbbrServiceFairAction::kSkipInvalidHistory;
    EmitFbbrServiceFairTrace(
        now, "CYCLE_UPDATE",
        WaveformClassification::kInconclusive);
    return;
  }

  service_fair_raw_regime_candidate_bps_ =
      static_cast<double>(
          current_injection_baseline_bw_.ToBitsPerSecond());
  service_fair_final_regime_candidate_bps_ =
      service_fair_raw_regime_candidate_bps_;

  const TimeDelta rtprop = CurrentFbbrRtprop();
  service_fair_alpha_bps_ = ComputeFbbrServiceFairAlphaBps(rtprop);
  if (model_.is_app_limited() && !service_fair_service_history_valid_) {
    service_fair_action_ = FbbrServiceFairAction::kSkipAppLimited;
    EmitFbbrServiceFairTrace(
        now, "CYCLE_UPDATE",
        WaveformClassification::kInconclusive);
    return;
  }
  if (rtprop.IsZero() || rtprop.IsInfinite() ||
      rtprop.ToMicroseconds() <= 0 ||
      !service_fair_qdelay_valid_ || !service_fair_service_history_valid_ ||
      !IsFinitePositiveBandwidth(service_fair_service_rate_) ||
      service_fair_alpha_bps_ <= 0.0) {
    service_fair_action_ = FbbrServiceFairAction::kSkipInvalidHistory;
    EmitFbbrServiceFairTrace(
        now, "CYCLE_UPDATE",
        WaveformClassification::kInconclusive);
    return;
  }

  if (service_fair_previous_qdelay_valid_) {
    service_fair_qdelay_trend_ = TimeDelta::FromMicroseconds(
        service_fair_qdelay_ewma_.ToMicroseconds() -
        service_fair_previous_qdelay_ewma_.ToMicroseconds());
  }
  if (IsFinitePositiveBandwidth(service_fair_previous_service_rate_)) {
    service_fair_service_rate_change_ =
        static_cast<double>(service_fair_service_rate_.ToBitsPerSecond()) /
            static_cast<double>(
                service_fair_previous_service_rate_.ToBitsPerSecond()) -
        1.0;
  }

  const int64_t rtprop_us = rtprop.ToMicroseconds();
  const int64_t q_low_us = std::max<int64_t>(
      1000, static_cast<int64_t>(std::llround(0.03 * rtprop_us)));
  const int64_t q_high_us = std::max<int64_t>(
      2000, static_cast<int64_t>(std::llround(0.10 * rtprop_us)));
  const int64_t trend_threshold_us = q_low_us;
  const double current_baseline_bps =
      static_cast<double>(
          current_injection_baseline_bw_.ToBitsPerSecond());
  double next_baseline_bps = current_baseline_bps;
  if (service_fair_qdelay_ewma_.ToMicroseconds() > q_high_us ||
      (service_fair_previous_qdelay_valid_ &&
       service_fair_qdelay_trend_.ToMicroseconds() > trend_threshold_us)) {
    next_baseline_bps = std::min(
        current_baseline_bps,
        std::max(static_cast<double>(kFbbrServiceFairMinimumBps),
                 kFbbrServiceFairBeta * current_baseline_bps));
    service_fair_action_ =
        FbbrServiceFairAction::kMultiplicativeDecrease;
  } else if (service_fair_qdelay_ewma_.ToMicroseconds() < q_low_us) {
    next_baseline_bps += service_fair_alpha_bps_;
    const QuicBandwidth max_bw = model_.MaxBandwidth();
    if (IsFinitePositiveBandwidth(max_bw)) {
      next_baseline_bps = std::min(
          next_baseline_bps,
          std::max(current_baseline_bps,
                   static_cast<double>(
                       max_bw.ToBitsPerSecond())));
    }
    service_fair_action_ = FbbrServiceFairAction::kAdditiveIncrease;
  } else {
    service_fair_action_ = FbbrServiceFairAction::kHold;
  }

  current_injection_baseline_bw_ =
      BandwidthFromBps(next_baseline_bps);
  service_fair_final_regime_candidate_bps_ =
      static_cast<double>(
          current_injection_baseline_bw_.ToBitsPerSecond());
  service_fair_previous_qdelay_ewma_ = service_fair_qdelay_ewma_;
  service_fair_previous_qdelay_valid_ = true;
  service_fair_previous_service_rate_ = service_fair_service_rate_;
  ++service_fair_cycle_count_;
  EmitFbbrServiceFairTrace(
      now, "CYCLE_UPDATE",
      WaveformClassification::kInconclusive);
}

QuicByteCount FBBRSender::ComputeFbbrServiceInflightBytes(
    QuicTime now,
    TimeDelta rtprop) const {
  if (rtprop.IsZero() || rtprop.IsInfinite() ||
      rtprop.ToMicroseconds() <= 0 ||
      fbbr_delivered_history_.empty() ||
      now < QuicTime::Zero() + rtprop) {
    return 0;
  }
  const QuicTime window_start = now - rtprop;
  bool has_anchor = false;
  uint64_t anchor_delivered = 0;
  bool has_current = false;
  uint64_t current_delivered = 0;
  for (const FbbrDeliveredPoint& point :
       fbbr_delivered_history_) {
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

QuicByteCount FBBRSender::ComputeFbbrEnvelopeBytes(
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

QuicByteCount FBBRSender::ComputeFbbrInflightCapBytes(
    QuicByteCount envelope,
    QuicByteCount native_extra_acked,
    QuicByteCount native_offload_budget) const {
  const QuicByteCount maximum = std::numeric_limits<QuicByteCount>::max();
  auto saturating_add = [maximum](QuicByteCount lhs, QuicByteCount rhs) {
    return lhs > maximum - rhs ? maximum : lhs + rhs;
  };
  QuicByteCount cap = saturating_add(envelope, native_extra_acked);
  cap = saturating_add(cap, native_offload_budget);
  cap = std::max(cap, cwnd_limits().Min());

  const QuicByteCount quantum = kDefaultTCPMSS;
  if (quantum > 0 && cap < maximum) {
    const QuicByteCount remainder = cap % quantum;
    if (remainder != 0) {
      cap = saturating_add(cap, quantum - remainder);
    }
  }
  return cap;
}

FBBRSender::FbbrEnvelopeSnapshot
FBBRSender::BuildFbbrEnvelopeSnapshot(
    QuicByteCount native_cwnd,
    QuicByteCount actual_inflight) const {
  FbbrEnvelopeSnapshot snapshot;
  snapshot.native_cwnd = native_cwnd;
  snapshot.actual_inflight = actual_inflight;
  snapshot.previous_trusted_bw_valid =
      HasUsableFbbrPreviousTrustedBw();
  snapshot.previous_trusted_bw_from_guard =
      snapshot.previous_trusted_bw_valid &&
      std::string(trusted_bw_source_ == nullptr
                      ? kTrustedBwSourceNone : trusted_bw_source_) ==
          kTrustedBwSourceGuardFilter;
  snapshot.pacing_target = fbbr_last_target_rate_;
  snapshot.pacing_base_target = fbbr_last_base_target_rate_;
  const TimeDelta rtprop = CurrentFbbrRtprop();
  snapshot.target_history_valid =
      !rtprop.IsZero() &&
      HasFullFbbrTargetHistory(current_time_, rtprop);
  if (snapshot.target_history_valid) {
    snapshot.plan_inflight =
        ComputeFbbrPlannedInflightBytes(current_time_, rtprop);
    snapshot.positive_probe_credit =
        ComputeFbbrPositiveProbeCreditBytes(current_time_, rtprop);
  }
  if (!rtprop.IsZero()) {
    snapshot.service_inflight =
        ComputeFbbrServiceInflightBytes(current_time_, rtprop);
    snapshot.service_history_valid =
        HasValidFbbrServiceHistory(
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
  snapshot.envelope = ComputeFbbrEnvelopeBytes(
      snapshot.plan_inflight, snapshot.service_inflight,
      snapshot.positive_probe_credit,
      snapshot.service_history_valid);
  snapshot.extra_acked = model_.MaxAckHeight();
  if (snapshot.target_history_valid) {
    snapshot.inflight_cap = ComputeFbbrInflightCapBytes(
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
      UsesFbbrServiceEnvelope() && !rtprop.IsZero() &&
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

QuicByteCount FBBRSender::ApplyFbbrInflightEnvelope(
    QuicByteCount native_cwnd_target) const {
  if (!UsesFbbrServiceEnvelope()) {
    return native_cwnd_target;
  }
  const FbbrEnvelopeSnapshot snapshot =
      BuildFbbrEnvelopeSnapshot(
          native_cwnd_target,
          latest_congestion_event_inflight_valid_
              ? latest_congestion_event_inflight_ : 0);
  if (!snapshot.projection_active) {
    return native_cwnd_target;
  }
  return std::min(native_cwnd_target, snapshot.inflight_cap);
}

void FBBRSender::UpdateFbbrTelemetry(
    QuicTime now,
    QuicByteCount actual_inflight) {
  if (!UsesFbbrServiceEnvelope()) {
    return;
  }
  if (fbbr_telemetry_initialized_ &&
      now >= fbbr_telemetry_last_time_) {
    const uint64_t delta_us = static_cast<uint64_t>(
        (now - fbbr_telemetry_last_time_).ToMicroseconds());
    fbbr_telemetry_total_us_ += delta_us;
    switch (fbbr_telemetry_previous_trusted_source_) {
      case FbbrPreviousTrustedSource::kTrusted:
        fbbr_previous_trusted_us_ += delta_us;
        break;
      case FbbrPreviousTrustedSource::kGuard:
        fbbr_previous_trusted_guard_source_us_ += delta_us;
        break;
      case FbbrPreviousTrustedSource::kInvalid:
      default:
        fbbr_previous_trusted_invalid_us_ += delta_us;
        break;
    }
    if (fbbr_telemetry_projection_active_) {
      fbbr_projection_active_us_ += delta_us;
    }
    if (fbbr_telemetry_service_history_valid_) {
      fbbr_service_history_valid_us_ += delta_us;
    }
    if (fbbr_telemetry_app_limited_fallback_) {
      fbbr_app_limited_fallback_us_ += delta_us;
    }
    if (fbbr_telemetry_plan_only_fallback_) {
      fbbr_plan_only_fallback_us_ += delta_us;
    }
    if (fbbr_telemetry_service_limited_) {
      fbbr_service_limited_us_ += delta_us;
    }
    if (fbbr_telemetry_cap_binding_) {
      fbbr_cap_binding_us_ += delta_us;
    }
    fbbr_plan_inflight_byte_us_ +=
        static_cast<long double>(
            fbbr_telemetry_plan_inflight_) * delta_us;
    fbbr_service_inflight_byte_us_ +=
        static_cast<long double>(
            fbbr_telemetry_service_inflight_) * delta_us;
    fbbr_probe_credit_byte_us_ +=
        static_cast<long double>(
            fbbr_telemetry_probe_credit_) * delta_us;
    fbbr_extra_acked_byte_us_ +=
        static_cast<long double>(
            fbbr_telemetry_extra_acked_) * delta_us;
    fbbr_service_restriction_byte_us_ +=
        static_cast<long double>(
            fbbr_telemetry_service_restriction_) * delta_us;
    fbbr_enforced_excess_byte_us_ +=
        static_cast<long double>(
            fbbr_telemetry_enforced_excess_) * delta_us;
  }

  const FbbrEnvelopeSnapshot snapshot =
      BuildFbbrEnvelopeSnapshot(cwnd_, actual_inflight);
  fbbr_telemetry_last_time_ = now;
  fbbr_telemetry_initialized_ = true;
  fbbr_telemetry_previous_trusted_source_ =
      !snapshot.previous_trusted_bw_valid
          ? FbbrPreviousTrustedSource::kInvalid
          : snapshot.previous_trusted_bw_from_guard
              ? FbbrPreviousTrustedSource::kGuard
              : FbbrPreviousTrustedSource::kTrusted;
  fbbr_telemetry_projection_active_ = snapshot.projection_active;
  fbbr_telemetry_service_history_valid_ =
      snapshot.service_history_valid;
  fbbr_telemetry_app_limited_fallback_ =
      snapshot.projection_active &&
      snapshot.app_limited_contaminated;
  fbbr_telemetry_plan_only_fallback_ =
      snapshot.projection_active &&
      !snapshot.service_history_valid;
  fbbr_telemetry_service_limited_ = snapshot.service_limited;
  fbbr_telemetry_cap_binding_ = snapshot.cap_binding;
  fbbr_telemetry_plan_inflight_ = snapshot.plan_inflight;
  fbbr_telemetry_service_inflight_ = snapshot.service_inflight;
  fbbr_telemetry_probe_credit_ =
      snapshot.positive_probe_credit;
  fbbr_telemetry_extra_acked_ = snapshot.extra_acked;
  fbbr_telemetry_service_restriction_ =
      snapshot.service_restriction;
  fbbr_telemetry_enforced_excess_ = snapshot.enforced_excess;
  fbbr_plan_inflight_samples_.push_back(snapshot.plan_inflight);
  fbbr_service_inflight_samples_.push_back(
      snapshot.service_inflight);
  fbbr_probe_credit_samples_.push_back(
      snapshot.positive_probe_credit);
  fbbr_extra_acked_samples_.push_back(snapshot.extra_acked);
  fbbr_service_restriction_samples_.push_back(
      snapshot.service_restriction);
  fbbr_enforced_excess_samples_.push_back(
      snapshot.enforced_excess);
  ++fbbr_window_ack_events_;
  if (snapshot.cap_binding) {
    ++fbbr_window_cap_binding_events_;
  }
}

void FBBRSender::EmitFbbrFlowSummary() const {
  if (!UsesFbbrServiceEnvelope() || !cruise_load_trace_cb_) {
    return;
  }
  auto ratio = [this](uint64_t value) {
    return fbbr_telemetry_total_us_ == 0
        ? 0.0
        : static_cast<double>(value) /
              static_cast<double>(fbbr_telemetry_total_us_);
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
      static_cast<long double>(fbbr_telemetry_total_us_);
  auto mean = [duration_us](long double byte_us) {
    return duration_us > 0.0L ? byte_us / duration_us : 0.0L;
  };
  std::ostringstream row;
  row << (IsFbbrServiceFair() ? "FBBR-ServiceFair" : "FBBR")
      << ","
      << trace_flow_id_ << ","
      << ratio(fbbr_previous_trusted_us_) << ","
      << ratio(fbbr_previous_trusted_guard_source_us_) << ","
      << ratio(fbbr_previous_trusted_invalid_us_) << ","
      << ratio(fbbr_projection_active_us_) << ","
      << ratio(fbbr_service_history_valid_us_) << ","
      << ratio(fbbr_app_limited_fallback_us_) << ","
      << ratio(fbbr_plan_only_fallback_us_) << ","
      << ratio(fbbr_service_limited_us_) << ","
      << ratio(fbbr_cap_binding_us_) << ","
      << static_cast<double>(
             mean(fbbr_plan_inflight_byte_us_)) << ","
      << p95(fbbr_plan_inflight_samples_) << ","
      << static_cast<double>(
             mean(fbbr_service_inflight_byte_us_)) << ","
      << p95(fbbr_service_inflight_samples_) << ","
      << static_cast<double>(
             mean(fbbr_probe_credit_byte_us_)) << ","
      << p95(fbbr_probe_credit_samples_) << ","
      << static_cast<double>(
             mean(fbbr_extra_acked_byte_us_)) << ","
      << p95(fbbr_extra_acked_samples_) << ","
      << static_cast<double>(
             mean(fbbr_service_restriction_byte_us_)) << ","
      << p95(fbbr_service_restriction_samples_) << ","
      << static_cast<double>(
             mean(fbbr_enforced_excess_byte_us_)) << ","
      << p95(fbbr_enforced_excess_samples_);
  cruise_load_trace_cb_(0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                        "FBBR_FLOW_SUMMARY", false, row.str());
}

void FBBRSender::FinalizeFbbrTrace() {
  if (fbbr_flow_summary_emitted_) {
    return;
  }
  fbbr_flow_summary_emitted_ = true;
  EmitFbbrFlowSummary();
}

FBBRSender::FbbrRegimeDecision FBBRSender::ClassifyFbbrRegime(
    const FbbrRegimeFeatures& features,
    const FbbrRegimeContext& context) {
  FbbrRegimeDecision decision;
  auto finalize = [](FbbrRegimeDecision value) { return value; };
  auto decide = [&decision, &finalize](
                            WaveformClassification classification,
                            const char* rule,
                            bool upper_bound_rule,
                            bool refresh_rtprop,
                            bool update_rtprop_drate) {
    (void)upper_bound_rule;
    (void)update_rtprop_drate;
    decision.classification = classification;
    decision.rule_id = rule;
    decision.refresh_rtprop = refresh_rtprop;
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
  auto fallback_regime = [&](bool* valid) {
    *valid = features.srtt_stats_valid &&
        context.max_srtt_valid && context.rtprop_valid &&
        std::isfinite(features.srtt_max_ms) &&
        features.srtt_max_ms > 0.0 &&
        std::isfinite(context.max_srtt_ms) &&
        std::isfinite(context.rtprop_ms) &&
        context.max_srtt_ms >= context.rtprop_ms &&
        context.rtprop_ms > 0.0;
    if (!*valid) {
      return WaveformClassification::kInconclusive;
    }
    const double overload_threshold_ms = std::max(
        1.10 * context.rtprop_ms,
        context.rtprop_ms +
            (context.max_srtt_ms - context.rtprop_ms) / 3.0);
    if (features.srtt_max_ms > overload_threshold_ms) {
      return WaveformClassification::kOverload;
    }
    if (features.srtt_max_ms < 1.05 * context.rtprop_ms) {
      return WaveformClassification::kUnderload;
    }
    return WaveformClassification::kFullLoad;
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
    const WaveformClassification classification =
        fallback_regime(&threshold_valid);
    if (!threshold_valid) {
      return finalize(decision);
    }
    return decide(classification, "N12", false, false, false);
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
  const WaveformClassification classification =
      fallback_regime(&threshold_valid);
  if (!threshold_valid) {
    return finalize(decision);
  }
  return decide(classification, "N16", false, false, false);
}


FBBRSender::FbbrActuatorResult
FBBRSender::ComputeFbbrInjectionBaseline(
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
    double minimum_rate_bps) {
  FbbrActuatorResult result;
  if (classification == WaveformClassification::kInconclusive ||
      !std::isfinite(mindrate_bps) || mindrate_bps <= 0.0 ||
      !std::isfinite(maxdrate_bps) || maxdrate_bps < mindrate_bps ||
      !std::isfinite(minimum_rate_bps) || minimum_rate_bps <= 0.0) {
    return result;
  }

  if (classification == WaveformClassification::kFullLoad) {
    if (!regime_i_or_iii_seen_this_cruise) {
      if (!std::isfinite(current_baseline_bps) ||
          current_baseline_bps <= 0.0) {
        return result;
      }
    } else {
      if (!std::isfinite(regime_ii_delivery_rate_bps) ||
          regime_ii_delivery_rate_bps <= 0.0) {
        return result;
      }
      result.update_baseline = true;
      result.next_baseline_bps = regime_ii_delivery_rate_bps;
      result.update_trusted_bw = true;
      result.trusted_bw_bps = regime_ii_delivery_rate_bps;
    }
    result.valid = true;
    return result;
  }

  if (!std::isfinite(current_baseline_bps) ||
      current_baseline_bps <= 0.0) {
    return result;
  }
  result.update_baseline = true;

  if (classification == WaveformClassification::kOverload) {
    const bool usable_min_bw = min_bw_valid &&
        std::isfinite(min_bw_bps) && min_bw_bps > 0.0;
    result.regime_iii_mindrate_triggered =
        usable_min_bw &&
        mindrate_bps < current_baseline_bps && mindrate_bps > min_bw_bps;
    result.regime_iii_minbw_midpoint_bps = usable_min_bw
        ? min_bw_bps + (mindrate_bps - min_bw_bps) / 2.0
        : 0.0;
    result.regime_iii_minbw_midpoint_triggered =
        !result.regime_iii_mindrate_triggered && usable_min_bw &&
        mindrate_bps > min_bw_bps &&
        result.regime_iii_minbw_midpoint_bps < current_baseline_bps;
    result.regime_iii_decrease_triggered =
        !result.regime_iii_mindrate_triggered &&
        !result.regime_iii_minbw_midpoint_triggered;
    result.next_baseline_bps = result.regime_iii_mindrate_triggered
        ? mindrate_bps
        : result.regime_iii_minbw_midpoint_triggered
            ? result.regime_iii_minbw_midpoint_bps
            : current_baseline_bps * 0.98;
  } else {
    const bool usable_max_bw = max_bw_valid &&
        std::isfinite(max_bw_bps) && max_bw_bps > 0.0;
    result.regime_i_maxdrate_triggered =
        usable_max_bw &&
        maxdrate_bps > current_baseline_bps && maxdrate_bps < max_bw_bps;
    result.regime_i_maxbw_midpoint_bps = usable_max_bw
        ? maxdrate_bps + (max_bw_bps - maxdrate_bps) / 2.0
        : 0.0;
    result.regime_i_maxbw_midpoint_triggered =
        !result.regime_i_maxdrate_triggered && usable_max_bw &&
        maxdrate_bps < max_bw_bps &&
        result.regime_i_maxbw_midpoint_bps > current_baseline_bps;
    result.regime_i_growth_triggered =
        !result.regime_i_maxdrate_triggered &&
        !result.regime_i_maxbw_midpoint_triggered;
    result.next_baseline_bps = result.regime_i_maxdrate_triggered
        ? maxdrate_bps
        : result.regime_i_maxbw_midpoint_triggered
            ? result.regime_i_maxbw_midpoint_bps
            : current_baseline_bps * 1.02;
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
FBBRSender::AnalyzeFbbrPeriodicSimilarity(
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
    return decide(WaveformClassification::kInconclusive, "R6");
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
          inputs.drate_positive_half_clipped &&
          inputs.positive_half_clips_simultaneous;
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

  // Masking a sequential middle plateau preserves the remaining waveform
  // evidence, including a genuine shoulder clip.
  const bool srtt_effective_similar =
      inputs.srtt_similar || inputs.srtt_similar_without_middle;
  if (srtt_effective_similar) {
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
  if (drate_effective_similar) {
    return decide(WaveformClassification::kUnderload, "R5.1");
  }
  return decide(WaveformClassification::kOverload, "R5.2");
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

FBBRSender::GoertzelComponentResult
FBBRSender::AnalyzeGoertzelComponent(
    const ResampledWaveformSeries& series,
    double sample_step_s,
    double target_frequency_hz) const {
  GoertzelComponentResult result;
  result.target_frequency_hz = target_frequency_hz;
  if (series.values.size() < 4 ||
      series.values.size() != series.valid.size() ||
      !std::isfinite(sample_step_s) || sample_step_s <= 0.0) {
    result.decision_reason = "INVALID_SERIES";
    return result;
  }
  const double nyquist_hz = 0.5 / sample_step_s;
  if (!std::isfinite(target_frequency_hz) || target_frequency_hz <= 0.0 ||
      target_frequency_hz >= nyquist_hz) {
    result.decision_reason = "INVALID_TARGET_FREQUENCY";
    return result;
  }

  double mean = 0.0;
  for (size_t i = 0; i < series.values.size(); ++i) {
    if (!series.valid[i] || !std::isfinite(series.values[i])) {
      result.decision_reason = "INCOMPLETE_RESAMPLED_SERIES";
      return result;
    }
    mean += series.values[i];
  }
  const double sample_count = static_cast<double>(series.values.size());
  mean /= sample_count;

  constexpr double kTwoPi = 6.283185307179586476925286766559;
  const double omega = kTwoPi * target_frequency_hz * sample_step_s;
  const double cosine = std::cos(omega);
  const double sine = std::sin(omega);
  const double coefficient = 2.0 * cosine;
  double previous = 0.0;
  double previous_previous = 0.0;
  double centered_energy = 0.0;
  for (double value : series.values) {
    const double centered = value - mean;
    centered_energy += centered * centered;
    const double state = centered + coefficient * previous -
        previous_previous;
    previous_previous = previous;
    previous = state;
  }

  result.input_valid = true;
  result.real = previous - cosine * previous_previous;
  result.imag = sine * previous_previous;
  result.power = result.real * result.real + result.imag * result.imag;
  if (!std::isfinite(centered_energy) || !std::isfinite(result.power) ||
      centered_energy < 0.0 || result.power < 0.0) {
    result.input_valid = false;
    result.decision_reason = "NONFINITE_GOERTZEL_OUTPUT";
    return result;
  }

  const double numerical_floor = std::numeric_limits<double>::epsilon() *
      std::max(1.0, sample_count * mean * mean);
  if (centered_energy <= numerical_floor) {
    result.decision_reason = "NO_VARIATION";
    return result;
  }

  result.amplitude = 2.0 * std::sqrt(result.power) / sample_count;
  result.coherent_power_ratio = std::max(
      0.0, std::min(1.0, result.power / (sample_count * centered_energy)));
  result.phase_rad = std::atan2(result.imag, result.real);
  result.component_present =
      result.coherent_power_ratio + 1e-15 >=
      goertzel_min_coherent_power_ratio_;
  result.decision_reason = result.component_present
      ? "TARGET_COMPONENT_PRESENT" : "TARGET_COMPONENT_ABSENT";
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
  if (!srtt_thresholds.valid || !drate_thresholds.valid) {
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
      const bool dual_opposing = other_thresholds.valid &&
          HasDualMacroOpposingShoulders(
              candidate.before, candidate.after,
              thresholds.shoulder_slope,
              thresholds.minimum_shoulder_change,
              candidate.other_before, candidate.other_after,
              other_thresholds.shoulder_slope,
              other_thresholds.minimum_shoulder_change,
              shoulder_duration_s);
      candidate.middle_sequential = owner_same || other_same;
      candidate.genuine_shoulder_clip =
          !candidate.middle_sequential &&
          dual_opposing &&
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
  RefreshProbeAmplitudeProtection();
  const double native_max_bw_bps = static_cast<double>(
      current_injection_baseline_bw_.ToBitsPerSecond());
  if (current_injection_baseline_bw_.IsInfinite() ||
      !std::isfinite(native_max_bw_bps)) {
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
  // A baseline update may have changed the safe negative excursion while the
  // prior window was being classified. Recompute before its replacement.
  RefreshProbeAmplitudeProtection();
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
  RefreshProbeAmplitudeProtection();
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
FBBRSender::AnalyzeFbbrWindow(QuicTime window_start,
                                    QuicTime window_end,
                                    double window_periods,
                                    bool extended_window) const {
  WaveformWindowAnalysis result;
  result.fbbr_pipeline = true;
  result.probe_epoch_start = probe_epoch_start_time_;
  result.probe_epoch_rtt = probe_epoch_rtt_;
  result.collection_window_start = window_start;
  result.collection_window_end = window_end;
  result.collection_window_periods = window_periods;
  result.window_start = window_start;
  result.window_end = window_end;
  result.window_periods = window_periods;
  result.extended_window = extended_window;
  result.max_srtt_before_ms = fbbr_max_srtt_valid_
      ? fbbr_max_srtt_ms_ : 0.0;
  result.max_srtt_after_ms = result.max_srtt_before_ms;
  const QuicBandwidth current_fbbr_max_bw = model_.MaxBandwidth();
  result.fbbr_max_bw_before_valid =
      IsFinitePositiveBandwidth(current_fbbr_max_bw);
  result.fbbr_max_bw_before_bps = result.fbbr_max_bw_before_valid
      ? static_cast<double>(current_fbbr_max_bw.ToBitsPerSecond()) : 0.0;
  result.fbbr_max_bw_after_valid =
      result.fbbr_max_bw_before_valid;
  result.fbbr_max_bw_after_bps =
      result.fbbr_max_bw_before_bps;
  const TimeDelta current_rtprop = fbbr_rtprop_valid_
      ? fbbr_rtprop_ : model_.MinRtt();
  result.fbbr_rtprop_valid = !current_rtprop.IsZero();
  result.fbbr_rtprop_ms =
      result.fbbr_rtprop_valid
          ? static_cast<double>(current_rtprop.ToMicroseconds()) / 1000.0
          : 0.0;
  result.fbbr_max_srtt_valid = fbbr_max_srtt_valid_;
  result.fbbr_max_srtt_ms = fbbr_max_srtt_valid_
      ? fbbr_max_srtt_ms_ : 0.0;
  if (window_end <= window_start || cruise_modulation_freq_hz_ <= 0.0 ||
      probe_epoch_rtt_.IsZero()) {
    result.invalid_reason = "invalid_fbbr_window";
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
          result.fbbr_rtprop_valid
              ? result.fbbr_rtprop_ms / 1000.0
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
    result.invalid_reason = "fbbr_resampling_failed";
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
  if (UsesFbbrServiceEnvelope() && result.srtt_stats_valid) {
    double time_weighted_srtt_mean_ms = 0.0;
    result.srtt_stats_valid = ComputeFbbrTimeWeightedSrttMeanMs(
        window_start, window_end, &time_weighted_srtt_mean_ms);
    result.srtt_mean_ms = result.srtt_stats_valid
        ? time_weighted_srtt_mean_ms : 0.0;
  }

  const bool use_goertzel_component_match = UsesFbbrServiceEnvelope();
  GoertzelComponentResult sender_goertzel;
  GoertzelComponentResult drate_goertzel;
  double srate_period_s = 0.0;
  if (use_goertzel_component_match) {
    // The injected frequency is known.  FBBR and ServiceFair therefore use
    // the direct generalized Goertzel response at that frequency, rather
    // than estimating a period from an autocorrelation peak.
    sender_goertzel = AnalyzeGoertzelComponent(
        sender, sample_step_s, cruise_modulation_freq_hz_);
    drate_goertzel = AnalyzeGoertzelComponent(
        drate, sample_step_s, cruise_modulation_freq_hz_);
    result.sender_goertzel = sender_goertzel;
    result.drate_goertzel = drate_goertzel;
    result.sender_waveform_valid = sender_goertzel.input_valid &&
        sender_goertzel.component_present;
    srate_period_s = period_s;
  } else {
    double sender_period_correlation = -1.0;
    srate_period_s = EstimateActualSignalPeriod(
        sender.values, sender.valid, sample_step_s, period_s,
        &sender_period_correlation);
    result.sender_waveform_valid = srate_period_s > 0.0;
  }
  FbbrRegimeFeatures& features = result.fbbr_features;
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
  if (!use_goertzel_component_match) {
    build_masks_and_raw_evidence(
        drate.values, drate.valid, true, &features.drate,
        &drate_clean_valid, &drate_periodic_valid, &drate_continuous);
  } else {
    features.drate.input_valid = result.drate_input_valid;
    features.drate.ordinary_wave_uses_raw_valid_view = true;
  }

  features.srtt.wave = DetectOrdinaryWaveActivity(
      srtt.values, srtt_clean_valid, sample_step_s, period_s, true);
  if (use_goertzel_component_match) {
    const bool component_input_valid = result.drate_input_valid &&
        sender_goertzel.input_valid && drate_goertzel.input_valid;
    result.goertzel_component_match = component_input_valid &&
        sender_goertzel.component_present && drate_goertzel.component_present;
    features.drate.wave.input_valid = component_input_valid;
    features.drate.wave.has_wave = result.goertzel_component_match;
    features.drate.wave.amplitude = drate_goertzel.amplitude;
    features.drate.wave.amplitude_to_level_ratio =
        drate_goertzel.amplitude /
        std::max(std::abs(result.delivery_rate_mean_bps), 1e-12);
    features.drate.wave.failure_reason = !component_input_valid
        ? "GOERTZEL_INPUT_INVALID"
        : (result.goertzel_component_match
               ? "GOERTZEL_COMPONENT_MATCH"
               : "GOERTZEL_COMPONENT_ABSENT");
    result.current_drate_response_amplitude_bps = drate_goertzel.amplitude;
  } else {
    // PDF: DRate ordinary-wave detection always uses the raw valid view.
    features.drate.wave = DetectOrdinaryWaveActivity(
        drate.values, drate.valid, sample_step_s, period_s);
  }

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
  if (!use_goertzel_component_match) {
    derive_shoulders(drate.values, drate.valid, drate_continuous,
                     &features.drate);
  }

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
  if (use_goertzel_component_match) {
    // SRTT clipping remains a time-domain classification input.  The only
    // sender/DRate similarity predicate is co-presence of the injected
    // frequency component, so clip masks and clip direction do not veto it.
    features.drate.upper_clip_periodic_veto = false;
    features.drate.lower_clip_ignored_for_periodic = false;
    features.drate.estimated_srate_period_s = period_s;
    features.drate.periodic_similarity_input_valid =
        features.drate.wave.input_valid;
    features.drate.periodic_similar = result.goertzel_component_match;
    features.drate.periodic = !features.drate.wave.input_valid
        ? PeriodicSimilarityResult::kInvalidInput
        : (result.goertzel_component_match
               ? PeriodicSimilarityResult::kMatch
               : PeriodicSimilarityResult::kNoMatch);
  } else {
    features.srtt.periodic = AnalyzeFbbrPeriodicSimilarity(
        srtt.values, srtt.valid, srtt_periodic_valid,
        sample_step_s, period_s, srate_period_s, srtt_upper_verified,
        &features.srtt);
    features.drate.periodic = AnalyzeFbbrPeriodicSimilarity(
        drate.values, drate.valid, drate_periodic_valid,
        sample_step_s, period_s, srate_period_s, drate_upper_verified,
        &features.drate);
  }

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
  context.max_srtt_valid = fbbr_max_srtt_valid_;
  context.max_srtt_ms = fbbr_max_srtt_ms_;
  context.rtprop_valid = !current_rtprop.IsZero();
  context.rtprop_ms = context.rtprop_valid
      ? static_cast<double>(current_rtprop.ToMicroseconds()) / 1000.0
      : 0.0;
  result.fbbr_decision = ClassifyFbbrRegime(features, context);
  result.classification = result.fbbr_decision.classification;
  result.unsuppressed_classification = result.classification;
  result.decision_rule = result.fbbr_decision.rule_id;
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
    result.invalid_reason = result.fbbr_decision.rule_id[0] == '\0'
        ? "fbbr_required_predicate_invalid"
        : "fbbr_inconclusive";
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
  if (UsesFbbrServiceEnvelope()) {
    return AnalyzeFbbrWindow(window_start, window_end,
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
    decision.srtt_window_stats_valid = result.srtt_stats_valid;
    decision.srtt_mean_ms = result.srtt_mean_ms;
    decision.srtt_min_ms = result.srtt_min_ms;
    decision.srtt_max_ms = result.srtt_max_ms;
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
  decision.srtt_input_valid = result.srtt_input_valid;
  decision.srtt_window_stats_valid = result.srtt_stats_valid;
  decision.srtt_mean_ms = result.srtt_mean_ms;
  decision.srtt_min_ms = result.srtt_min_ms;
  decision.srtt_max_ms = result.srtt_max_ms;
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
      UsesFbbrServiceEnvelope() && fbbr_rtprop_valid_
          ? fbbr_rtprop_ : model_.MinRtt();
  const QuicTime timestamp_before =
      UsesFbbrServiceEnvelope() && fbbr_rtprop_valid_
          ? fbbr_rtprop_source_time_
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
  if (UsesFbbrServiceEnvelope()) {
    PublishFbbrRtprop(bottom_min_rtt, now, false);
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

void FBBRSender::UpdateFbbrRetryState(
    WaveformWindowAnalysis* analysis) {
  if (analysis == nullptr || !analysis->fbbr_pipeline) {
    return;
  }
  constexpr uint8_t kInvalidInput = 0x1;
  constexpr uint8_t kTwoWindowNoWave = 0x2;
  analysis->unsuppressed_classification = analysis->classification;
  if (IsProbeAmplitudeSuppressedByFloor()) {
    // At or below the pacing floor there was intentionally no injected
    // waveform. Keep the ordinary regime decision, but do not turn that
    // absence into a signal-fidelity retry.
    fbbr_wave_fidelity_enhancement_active_ = false;
    fbbr_retry_reason_mask_ &=
        static_cast<uint8_t>(~kTwoWindowNoWave);
    fbbr_srtt_no_wave_streak_ = 0;
    fbbr_drate_no_wave_streak_ = 0;
    analysis->wave_fidelity_enhancement_active = false;
    analysis->retry_reason_mask = fbbr_retry_reason_mask_;
    analysis->srtt_no_wave_streak = 0;
    analysis->drate_no_wave_streak = 0;
    return;
  }
  const bool classification_invalid =
      analysis->classification == WaveformClassification::kInconclusive;
  if (classification_invalid) {
    fbbr_retry_reason_mask_ |= kInvalidInput;
  } else {
    fbbr_retry_reason_mask_ &=
        static_cast<uint8_t>(~kInvalidInput);
  }
  const bool unique_window =
      analysis->window_second_cycle_id !=
          fbbr_last_counted_window_second_cycle_id_;
  if (unique_window) {
    fbbr_last_counted_window_second_cycle_id_ =
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
    update_streak(analysis->fbbr_features.srtt.wave,
                  &fbbr_srtt_no_wave_streak_);
    update_streak(analysis->fbbr_features.drate.wave,
                  &fbbr_drate_no_wave_streak_);
  }
  const bool was_active =
      fbbr_wave_fidelity_enhancement_active_;
  const bool trigger = !was_active &&
      (fbbr_srtt_no_wave_streak_ >=
           fbbr_wave_fidelity_no_wave_trigger_windows_ ||
       fbbr_drate_no_wave_streak_ >=
           fbbr_wave_fidelity_no_wave_trigger_windows_);
  const bool either_wave =
      (analysis->fbbr_features.srtt.wave.input_valid &&
       analysis->fbbr_features.srtt.wave.has_wave) ||
      (analysis->fbbr_features.drate.wave.input_valid &&
       analysis->fbbr_features.drate.wave.has_wave);
  if (trigger) {
    fbbr_wave_fidelity_enhancement_active_ = true;
    fbbr_retry_reason_mask_ |= kTwoWindowNoWave;
    analysis->no_wave_triggered = true;
    analysis->wave_fidelity_just_entered = true;
  } else if (was_active && either_wave) {
    fbbr_wave_fidelity_enhancement_active_ = false;
    fbbr_retry_reason_mask_ &=
        static_cast<uint8_t>(~kTwoWindowNoWave);
    fbbr_srtt_no_wave_streak_ = 0;
    fbbr_drate_no_wave_streak_ = 0;
  }
  const bool suppress_for_no_wave = trigger ||
      (was_active && !either_wave);
  if (suppress_for_no_wave) {
    analysis->classification = WaveformClassification::kInconclusive;
    analysis->classification_suppressed_for_retry = true;
    analysis->state_updates_suppressed_for_retry = true;
    analysis->invalid_reason = AppendReason(
        analysis->invalid_reason, "TWO_WINDOW_NO_WAVE");
    if (fbbr_rolling_retry_count_ <
        std::numeric_limits<uint32_t>::max()) {
      ++fbbr_rolling_retry_count_;
    }
  }
  analysis->wave_fidelity_enhancement_active =
      fbbr_wave_fidelity_enhancement_active_;
  analysis->retry_reason_mask = fbbr_retry_reason_mask_;
  analysis->srtt_no_wave_streak =
      fbbr_srtt_no_wave_streak_;
  analysis->drate_no_wave_streak =
      fbbr_drate_no_wave_streak_;
}

void FBBRSender::ApplyFbbrRegimeStateUpdates(
    WaveformWindowAnalysis* trace_analysis,
    QuicTime now) {
  if (trace_analysis == nullptr) {
    return;
  }
  const FbbrRegimeDecision& decision = trace_analysis->fbbr_decision;
  const bool classification_usable =
      !trace_analysis->classification_suppressed_for_retry &&
      trace_analysis->classification !=
          WaveformClassification::kInconclusive;
  if (classification_usable && decision.refresh_rtprop &&
      trace_analysis->srtt_stats_valid &&
      std::isfinite(trace_analysis->srtt_min_ms) &&
      trace_analysis->srtt_min_ms > 0.0 && now != QuicTime::Zero()) {
    const TimeDelta refreshed = TimeDelta::FromMicroseconds(
        std::max<int64_t>(1, static_cast<int64_t>(std::llround(
            trace_analysis->srtt_min_ms * 1000.0))));
    PublishFbbrRtprop(refreshed, now, false);
  }
  trace_analysis->max_srtt_after_ms = fbbr_max_srtt_valid_
      ? fbbr_max_srtt_ms_ : 0.0;
  const QuicBandwidth max_bw = model_.MaxBandwidth();
  trace_analysis->fbbr_max_bw_after_valid =
      IsFinitePositiveBandwidth(max_bw);
  trace_analysis->fbbr_max_bw_after_bps =
      trace_analysis->fbbr_max_bw_after_valid
          ? static_cast<double>(max_bw.ToBitsPerSecond()) : 0.0;
}

void FBBRSender::ApplyFbbrClassification(
    const WaveformWindowAnalysis& analysis,
    QuicTime now) {
  const bool is_service_fair = IsFbbrServiceFair();
  WaveformWindowAnalysis trace_analysis = analysis;
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
  trace_analysis.fbbr_regime_i_maxdrate_triggered = false;
  trace_analysis.fbbr_regime_i_maxbw_midpoint_valid = false;
  trace_analysis.fbbr_regime_i_maxbw_midpoint_bps = 0.0;
  trace_analysis.fbbr_regime_i_maxbw_midpoint_triggered = false;
  trace_analysis.fbbr_regime_i_growth_triggered = false;
  if (is_service_fair) {
    service_fair_raw_regime_candidate_bps_ = baseline_before_bps;
    service_fair_final_regime_candidate_bps_ = baseline_before_bps;
  }

  if (analysis.classification == WaveformClassification::kInconclusive) {
    waveform_last_action_ = "FBBR_INCONCLUSIVE_BASELINE_HOLD";
    waveform_last_invalid_reason_ = analysis.invalid_reason;
    EmitWaveformSearchTrace(trace_analysis, waveform_last_action_,
                            baseline_before_bps, amplitude_before_bps);
    EmitFbbrServiceFairTrace(
        now, "REGIME_EXECUTOR", analysis.classification);
    ScheduleWaveformCollectionAfterSettle(now, false);
    return;
  }
  if (!analysis.delivery_rate_stats_valid) {
    waveform_last_action_ = "FBBR_INVALID_DRATE_BASELINE_HOLD";
    waveform_last_invalid_reason_ =
        "fbbr_delivery_rate_stats_invalid";
    trace_analysis.invalid_reason = waveform_last_invalid_reason_;
    EmitWaveformSearchTrace(trace_analysis, waveform_last_action_,
                            baseline_before_bps, amplitude_before_bps);
    EmitFbbrServiceFairTrace(
        now, "REGIME_EXECUTOR", analysis.classification);
    ScheduleWaveformCollectionAfterSettle(now, false);
    return;
  }

  const QuicBandwidth min_bw = model_.MinBandwidth();
  const QuicBandwidth max_bw = model_.MaxBandwidth();
  const bool min_bw_valid = IsFinitePositiveBandwidth(min_bw);
  const bool max_bw_valid = IsFinitePositiveBandwidth(max_bw);
  double interval_delivery_rate_bps = 0.0;
  bool interval_delivery_rate_valid = false;
  if (analysis.classification == WaveformClassification::kFullLoad) {
    interval_delivery_rate_valid = ComputeFbbrIntervalDeliveryRateBps(
        analysis.window_start, analysis.window_end,
        &interval_delivery_rate_bps);
    trace_analysis.delivery_rate_mean_bps =
        interval_delivery_rate_valid ? interval_delivery_rate_bps : 0.0;
    if (fbbr_regime_i_or_iii_seen_this_cruise_ &&
        !interval_delivery_rate_valid) {
      waveform_last_action_ =
          "FBBR_REGIME_II_INVALID_DELIVERED_INTERVAL_HOLD";
      waveform_last_invalid_reason_ =
          "fbbr_regime_ii_delivered_interval_invalid";
      trace_analysis.invalid_reason = waveform_last_invalid_reason_;
      EmitWaveformSearchTrace(trace_analysis, waveform_last_action_,
                              baseline_before_bps, amplitude_before_bps);
      EmitFbbrServiceFairTrace(
          now, "REGIME_EXECUTOR", analysis.classification);
      ScheduleWaveformCollectionAfterSettle(now, false);
      return;
    }
  }
  const FbbrActuatorResult actuator = ComputeFbbrInjectionBaseline(
      analysis.classification,
      analysis.delivery_rate_min_bps,
      analysis.delivery_rate_max_bps,
      interval_delivery_rate_bps,
      min_bw_valid,
      min_bw_valid
          ? static_cast<double>(min_bw.ToBitsPerSecond()) : 0.0,
      max_bw_valid,
      max_bw_valid
          ? static_cast<double>(max_bw.ToBitsPerSecond()) : 0.0,
      baseline_before_bps,
      fbbr_regime_i_or_iii_seen_this_cruise_,
      static_cast<double>(minimum_pacing_rate_bps_));
  if (!actuator.valid) {
    waveform_last_action_ = "FBBR_INVALID_ACTUATOR_INPUT_HOLD";
    waveform_last_invalid_reason_ = "fbbr_actuator_input_invalid";
    trace_analysis.invalid_reason = waveform_last_invalid_reason_;
    EmitWaveformSearchTrace(trace_analysis, waveform_last_action_,
                            baseline_before_bps, amplitude_before_bps);
    EmitFbbrServiceFairTrace(
        now, "REGIME_EXECUTOR", analysis.classification);
    ScheduleWaveformCollectionAfterSettle(now, false);
    return;
  }

  ApplyFbbrRegimeStateUpdates(&trace_analysis, now);
  const double raw_candidate_bps = actuator.update_baseline
      ? actuator.next_baseline_bps : baseline_before_bps;
  double final_candidate_bps = raw_candidate_bps;
  if (is_service_fair) {
    service_fair_last_valid_regime_seen_this_cruise_ = true;
    service_fair_last_valid_regime_this_cruise_ =
        analysis.classification;
    service_fair_raw_regime_candidate_bps_ = raw_candidate_bps;
    if (analysis.classification == WaveformClassification::kOverload) {
      final_candidate_bps = ApplyFbbrServiceFairRegimeIIIControlLimit(
          raw_candidate_bps, baseline_before_bps,
          IsFinitePositiveBandwidth(service_fair_service_rate_)
              ? static_cast<double>(
                    service_fair_service_rate_.ToBitsPerSecond())
              : 0.0,
          service_fair_service_history_valid_);
    }
    service_fair_final_regime_candidate_bps_ = final_candidate_bps;
    current_injection_baseline_bw_ =
        BandwidthFromBps(final_candidate_bps);
  } else if (actuator.update_baseline) {
    current_injection_baseline_bw_ =
        BandwidthFromBps(actuator.next_baseline_bps);
  }
  if (analysis.classification == WaveformClassification::kUnderload ||
      analysis.classification == WaveformClassification::kOverload) {
    fbbr_regime_i_or_iii_seen_this_cruise_ = true;
  }
  if (actuator.update_trusted_bw) {
    fbbr_regime_ii_seen_this_cruise_ = true;
    fbbr_cruise_trusted_bw_ = BandwidthFromBps(actuator.trusted_bw_bps);
    fbbr_latest_trusted_bw_ = fbbr_cruise_trusted_bw_;
    fbbr_smoothed_trusted_bw_ = fbbr_cruise_trusted_bw_;
    fbbr_smoothed_trusted_bw_valid_ = !fbbr_cruise_trusted_bw_.IsZero();
    trusted_bw_candidate_ = fbbr_cruise_trusted_bw_;
    trusted_bw_candidate_source_ =
        kTrustedBwSourceFbbrWindowDeliveredRate;
    trusted_baseline_locked_ = !trusted_bw_candidate_.IsZero();
    if (trusted_baseline_locked_) {
      ++trusted_bw_candidate_update_count_;
    }
    trace_analysis.latest_trusted_bw_bps = actuator.trusted_bw_bps;
    trace_analysis.smoothed_trusted_bw_bps = actuator.trusted_bw_bps;
  }
  const double raw_baseline_delta = actuator.update_baseline
      ? std::abs(raw_candidate_bps - baseline_before_bps) : 0.0;
  const double applied_baseline_delta = is_service_fair
      ? std::abs(final_candidate_bps - baseline_before_bps)
      : raw_baseline_delta;
  if (applied_baseline_delta > 0.5 &&
      baseline_adjustment_count_ < std::numeric_limits<uint32_t>::max()) {
    ++baseline_adjustment_count_;
  }
  trace_analysis.fbbr_regime_i_maxdrate_triggered =
      actuator.regime_i_maxdrate_triggered;
  trace_analysis.fbbr_regime_i_maxbw_midpoint_valid = max_bw_valid;
  trace_analysis.fbbr_regime_i_maxbw_midpoint_bps =
      actuator.regime_i_maxbw_midpoint_bps;
  trace_analysis.fbbr_regime_i_maxbw_midpoint_triggered =
      actuator.regime_i_maxbw_midpoint_triggered;
  trace_analysis.fbbr_regime_i_growth_triggered =
      actuator.regime_i_growth_triggered;

  if (analysis.classification == WaveformClassification::kOverload) {
    trace_analysis.delta_source = actuator.regime_iii_mindrate_triggered
        ? kWaveformDeltaSourceFbbrRegimeIIIMinDrate
        : actuator.regime_iii_minbw_midpoint_triggered
            ? kWaveformDeltaSourceFbbrRegimeIIIMinBwMidpoint
            : kWaveformDeltaSourceFbbrRegimeIIIDecrease;
  } else if (analysis.classification ==
             WaveformClassification::kUnderload) {
    trace_analysis.delta_source = actuator.regime_i_maxdrate_triggered
        ? kWaveformDeltaSourceFbbrRegimeIMaxDrate
        : actuator.regime_i_maxbw_midpoint_triggered
            ? kWaveformDeltaSourceFbbrRegimeIMaxBwMidpoint
            : kWaveformDeltaSourceFbbrRegimeIGrowth;
  } else if (actuator.update_trusted_bw && actuator.update_baseline) {
    trace_analysis.delta_source =
        kWaveformDeltaSourceFbbrRegimeIIDeliveryRateBaseline;
  }
  trace_analysis.raw_delta_bw_bps = raw_baseline_delta;
  trace_analysis.applied_delta_bw_bps = applied_baseline_delta;
  waveform_last_delta_source_ = trace_analysis.delta_source;
  waveform_last_raw_delta_bw_bps_ = raw_baseline_delta;
  waveform_last_applied_delta_bw_bps_ = applied_baseline_delta;
  waveform_last_invalid_reason_ = "none";

  if (analysis.classification == WaveformClassification::kFullLoad) {
    waveform_last_action_ = actuator.update_trusted_bw
        ? "FBBR_REGIME_II_UPDATE_CRUISE_TRUSTED_BW_AND_BASELINE"
        : "FBBR_REGIME_II_CURRENT_BASELINE_HOLD";
  } else if (analysis.classification ==
             WaveformClassification::kUnderload) {
    underload_located_ = true;
    waveform_last_action_ = actuator.regime_i_maxdrate_triggered
        ? "FBBR_REGIME_I_USE_MAX_DRATE"
        : actuator.regime_i_maxbw_midpoint_triggered
            ? "FBBR_REGIME_I_USE_MAXBW_MAXDRATE_MIDPOINT"
            : "FBBR_REGIME_I_GROW_BASELINE_1P02";
  } else {
    waveform_last_action_ = actuator.regime_iii_mindrate_triggered
        ? "FBBR_REGIME_III_USE_MIN_DRATE"
        : actuator.regime_iii_minbw_midpoint_triggered
            ? "FBBR_REGIME_III_USE_MINBW_MINDRATE_MIDPOINT"
            : "FBBR_REGIME_III_DECREASE_BASELINE_0P98";
  }
  EmitWaveformSearchTrace(trace_analysis, waveform_last_action_,
                          baseline_before_bps, amplitude_before_bps);
  EmitFbbrServiceFairTrace(
      now, "REGIME_EXECUTOR", analysis.classification);
  ScheduleWaveformCollectionAfterSettle(now, false);
}


void FBBRSender::ApplyWaveformClassification(
    const WaveformWindowAnalysis& analysis,
    QuicTime now) {
  if (UsesFbbrServiceEnvelope()) {
    ApplyFbbrClassification(analysis, now);
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

bool FBBRSender::AmplifyWaveformProbeAfterInconclusive(
    const WaveformWindowAnalysis& analysis,
    QuicTime now) {
  if (IsProbeAmplitudeProtectionActive()) {
    return false;
  }
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
  RefreshProbeAmplitudeProtection();
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
  if (!in_cruise_ || waveform_cruise_state_ ==
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
      RefreshProbeAmplitudeProtection();
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
          UsesFbbrServiceEnvelope();
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
      if (UsesFbbrServiceEnvelope()) {
        UpdateFbbrRetryState(&analysis);
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
              inconclusive_extension_count_,
              waveform_max_inconclusive_extensions_,
              waveform_window_periods_, waveform_max_window_periods_)) {
        const double start_advance_periods =
            use_rolling_retry && waveform_window_extended_ &&
                    waveform_window_periods_ >= 3.0
                ? 1.0
                : 0.0;
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
  const QuicByteCount actual_inflight =
      latest_congestion_event_inflight_valid_
          ? latest_congestion_event_inflight_ : 0;
  const FbbrEnvelopeSnapshot fbbr =
      BuildFbbrEnvelopeSnapshot(cwnd_, actual_inflight);
  const double fbbr_cap_binding_fraction =
      fbbr_window_ack_events_ == 0
          ? 0.0
          : static_cast<double>(fbbr_window_cap_binding_events_) /
                static_cast<double>(fbbr_window_ack_events_);
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
      << "time_waveform" << ","
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
      << analysis.coverage_ratio << ","
      << analysis.app_limited_ratio << ","
      << (analysis.sender_waveform_valid ? "true" : "false") << ","
      << analysis.sender_goertzel.target_frequency_hz << ","
      << (analysis.sender_goertzel.input_valid ? "true" : "false") << ","
      << (analysis.sender_goertzel.component_present ? "true" : "false")
      << ","
      << analysis.sender_goertzel.real << ","
      << analysis.sender_goertzel.imag << ","
      << analysis.sender_goertzel.phase_rad << ","
      << analysis.sender_goertzel.power << ","
      << analysis.sender_goertzel.amplitude << ","
      << analysis.sender_goertzel.coherent_power_ratio << ","
      << analysis.sender_goertzel.decision_reason << ","
      << (analysis.drate_goertzel.input_valid ? "true" : "false") << ","
      << (analysis.drate_goertzel.component_present ? "true" : "false")
      << ","
      << analysis.drate_goertzel.real << ","
      << analysis.drate_goertzel.imag << ","
      << analysis.drate_goertzel.phase_rad << ","
      << analysis.drate_goertzel.power << ","
      << analysis.drate_goertzel.amplitude << ","
      << analysis.drate_goertzel.coherent_power_ratio << ","
      << analysis.drate_goertzel.decision_reason << ","
      << (analysis.goertzel_component_match ? "true" : "false") << ","
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
      << (IsFbbrServiceFair() ? "FBBR-ServiceFair" : "FBBR") << ","
      << (analysis.fbbr_pipeline ? "fbbr_v2" : "legacy")
      << ","
      << (analysis.fbbr_pipeline ? "fbbr_v2" : "legacy")
      << ","
      << analysis.fbbr_decision.rule_id << ","
      << WaveformClassificationName(analysis.unsuppressed_classification)
      << ","
      << (analysis.fbbr_features.srtt.suspected_top_candidate ? "true" : "false") << ","
      << (analysis.fbbr_features.srtt.suspected_bottom_candidate ? "true" : "false") << ","
      << (analysis.fbbr_features.srtt.positive_shoulder ? "true" : "false") << ","
      << (analysis.fbbr_features.srtt.long_top_line ? "true" : "false") << ","
      << (analysis.fbbr_features.srtt.repeated_top_clip ? "true" : "false") << ","
      << (analysis.fbbr_features.srtt.negative_shoulder ? "true" : "false") << ","
      << (analysis.fbbr_features.srtt.long_bottom_line ? "true" : "false") << ","
      << (analysis.fbbr_features.srtt.repeated_bottom_clip ? "true" : "false") << ","
      << clip_case_name(analysis.fbbr_features.selected_clip_case) << ","
      << (analysis.fbbr_features.both_clip_directions ? "true" : "false") << ","
      << (analysis.fbbr_features.clip_candidate_rejected_to_wave_fallback ? "true" : "false") << ","
      << (analysis.fbbr_features.fallback_entered ? "true" : "false") << ","
      << (analysis.fbbr_features.srtt.upper_clip_periodic_veto ? "true" : "false") << ","
      << (analysis.fbbr_features.drate.upper_clip_periodic_veto ? "true" : "false") << ","
      << (analysis.fbbr_features.srtt.lower_clip_ignored_for_periodic ? "true" : "false") << ","
      << (analysis.fbbr_features.drate.lower_clip_ignored_for_periodic ? "true" : "false") << ","
      << (analysis.fbbr_features.srtt.ordinary_wave_uses_raw_valid_view ? "RAW" : "CLEANED") << ","
      << (analysis.fbbr_features.drate.ordinary_wave_uses_raw_valid_view ? "RAW" : "CLEANED") << ","
      << analysis.fbbr_features.srtt.top_repeated_clip.contact_fragment_count << ","
      << analysis.fbbr_features.srtt.bottom_repeated_clip.contact_fragment_count << ","
      << analysis.fbbr_features.srtt.top_repeated_clip.contact_sample_count << ","
      << analysis.fbbr_features.srtt.bottom_repeated_clip.contact_sample_count << ","
      << static_cast<unsigned>(analysis.fbbr_features.srtt.top_repeated_clip.contact_cycle_mask) << ","
      << static_cast<unsigned>(analysis.fbbr_features.srtt.bottom_repeated_clip.contact_cycle_mask) << ","
      << analysis.fbbr_features.srtt.top_repeated_clip.contact_time_span_ratio_of_window << ","
      << analysis.fbbr_features.srtt.bottom_repeated_clip.contact_time_span_ratio_of_window << ","
      << analysis.fbbr_features.srtt.top_repeated_clip.pooled_flat_fraction << ","
      << analysis.fbbr_features.srtt.bottom_repeated_clip.pooled_flat_fraction << ","
      << analysis.fbbr_features.srtt.top_repeated_clip.verified_boundary_fraction << ","
      << analysis.fbbr_features.srtt.bottom_repeated_clip.verified_boundary_fraction << ","
      << analysis.fbbr_features.srtt.top_repeated_clip.extrapolated_overshoot_ratio << ","
      << analysis.fbbr_features.srtt.bottom_repeated_clip.extrapolated_overshoot_ratio << ","
      << analysis.fbbr_features.drate.top_repeated_clip.contact_fragment_count << ","
      << analysis.fbbr_features.drate.bottom_repeated_clip.contact_fragment_count << ","
      << analysis.fbbr_features.drate.top_repeated_clip.contact_sample_count << ","
      << analysis.fbbr_features.drate.bottom_repeated_clip.contact_sample_count << ","
      << static_cast<unsigned>(analysis.fbbr_features.drate.top_repeated_clip.contact_cycle_mask) << ","
      << static_cast<unsigned>(analysis.fbbr_features.drate.bottom_repeated_clip.contact_cycle_mask) << ","
      << analysis.fbbr_features.drate.top_repeated_clip.contact_time_span_ratio_of_window << ","
      << analysis.fbbr_features.drate.bottom_repeated_clip.contact_time_span_ratio_of_window << ","
      << analysis.fbbr_features.srtt.longest_top_line_ratio_of_period << ","
      << analysis.fbbr_features.srtt.longest_bottom_line_ratio_of_period << ","
      << (analysis.fbbr_features.srtt.positive_shoulder_cycle_input_valid ? "true" : "false") << ","
      << (analysis.fbbr_features.srtt.negative_shoulder_cycle_input_valid ? "true" : "false") << ","
      << (analysis.fbbr_features.srtt.positive_shoulder_cycle_recognizable ? "true" : "false") << ","
      << (analysis.fbbr_features.srtt.negative_shoulder_cycle_recognizable ? "true" : "false") << ","
      << analysis.fbbr_features.srtt.continuous_horizontal_count << ","
      << analysis.fbbr_features.drate.continuous_horizontal_count << ","
      << analysis.fbbr_features.srtt.middle_mask_ratio << ","
      << analysis.fbbr_features.drate.middle_mask_ratio << ","
      << analysis.fbbr_features.srtt.middle_best_slope_mismatch_ratio << ","
      << analysis.fbbr_features.drate.middle_best_slope_mismatch_ratio << ","
      << analysis.fbbr_features.srtt.middle_best_bridge_deviation_ratio << ","
      << analysis.fbbr_features.drate.middle_best_bridge_deviation_ratio << ","
      << (analysis.fbbr_features.srtt.wave.has_wave ? "true" : "false") << ","
      << (analysis.fbbr_features.drate.wave.has_wave ? "true" : "false") << ","
      << analysis.fbbr_features.srtt.wave.failure_reason << ","
      << analysis.fbbr_features.drate.wave.failure_reason << ","
      << analysis.fbbr_features.srtt.wave.amplitude << ","
      << analysis.fbbr_features.drate.wave.amplitude << ","
      << analysis.fbbr_features.srtt.wave.noise_sigma << ","
      << analysis.fbbr_features.drate.wave.noise_sigma << ","
      << analysis.fbbr_features.srtt.wave.step_threshold << ","
      << analysis.fbbr_features.drate.wave.step_threshold << ","
      << analysis.fbbr_features.srtt.wave.active_step_ratio << ","
      << analysis.fbbr_features.drate.wave.active_step_ratio << ","
      << analysis.fbbr_features.srtt.wave.up_change_ratio << ","
      << analysis.fbbr_features.srtt.wave.down_change_ratio << ","
      << analysis.fbbr_features.drate.wave.up_change_ratio << ","
      << analysis.fbbr_features.drate.wave.down_change_ratio << ","
      << analysis.fbbr_features.srtt.wave.significant_path_ratio << ","
      << analysis.fbbr_features.drate.wave.significant_path_ratio << ","
      << analysis.fbbr_features.srtt.wave.slope_reversals << ","
      << analysis.fbbr_features.drate.wave.slope_reversals << ","
      << static_cast<unsigned>(analysis.fbbr_features.srtt.wave.active_cycle_mask) << ","
      << static_cast<unsigned>(analysis.fbbr_features.drate.wave.active_cycle_mask) << ","
      << (analysis.fbbr_features.srtt.periodic_similarity_input_valid ? "true" : "false") << ","
      << (analysis.fbbr_features.drate.periodic_similarity_input_valid ? "true" : "false") << ","
      << (analysis.fbbr_features.srtt.periodic_similar ? "true" : "false") << ","
      << (analysis.fbbr_features.drate.periodic_similar ? "true" : "false") << ","
      << analysis.fbbr_features.estimated_srate_period_s << ","
      << analysis.fbbr_features.srtt.estimated_period_s << ","
      << analysis.fbbr_features.drate.estimated_period_s << ","
      << analysis.fbbr_features.srtt.response_srate_period_error_ratio << ","
      << analysis.fbbr_features.drate.response_srate_period_error_ratio << ","
      << analysis.fbbr_features.srtt.edge_mask_ratio << ","
      << analysis.fbbr_features.drate.edge_mask_ratio << ","
      << (analysis.fbbr_features.inflight_bdp_valid ? "true" : "false") << ","
      << analysis.fbbr_features.inflight_bytes << ","
      << analysis.fbbr_features.bdp_bytes << ","
      << (fbbr_max_srtt_valid_ ? "true" : "false") << ","
      << analysis.max_srtt_before_ms << ","
      << analysis.max_srtt_after_ms << ","
      << (analysis.fbbr_max_bw_before_valid ? "true" : "false") << ","
      << analysis.fbbr_max_bw_before_bps << ","
      << (analysis.fbbr_max_bw_after_valid ? "true" : "false") << ","
      << analysis.fbbr_max_bw_after_bps << ","
      << (analysis.fbbr_rtprop_valid ? "true" : "false") << ","
      << analysis.fbbr_rtprop_ms << ","
      << (analysis.fbbr_max_srtt_valid ? "true" : "false") << ","
      << analysis.fbbr_max_srtt_ms << ","
      << (analysis.fbbr_regime_i_maxdrate_triggered ? "true"
                                                       : "false") << ","
      << (analysis.fbbr_regime_i_maxbw_midpoint_valid ? "true"
                                                         : "false") << ","
      << analysis.fbbr_regime_i_maxbw_midpoint_bps << ","
      << (analysis.fbbr_regime_i_maxbw_midpoint_triggered ? "true"
                                                             : "false") << ","
      << (analysis.fbbr_regime_i_growth_triggered ? "true"
                                                     : "false") << ","
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
      << (analysis.fbbr_pipeline ? fbbr_wave_fidelity_retry_window_advance_periods_ : 0) << ","
      << (analysis.fbbr_pipeline ? 1 : 0) << ","
      << inconclusive_extension_count_ << ","
      << waveform_inconclusive_amplification_count_ << ","
      << waveform_initial_probe_amplitude_bps_ << ","
      << current_probe_amplitude_bps_ << ","
      << static_cast<double>(waveform_initial_probe_amplitude_bps_) *
             waveform_inconclusive_signal_amplification_max_ratio_ << ","
      << fbbr_rolling_retry_count_ << ","
      << (fbbr.previous_trusted_bw_valid
              ? trusted_bw_.ToBitsPerSecond() : 0) << ","
      << (!fbbr.previous_trusted_bw_valid
              ? "invalid"
              : fbbr.previous_trusted_bw_from_guard ? "guard" : "trusted")
      << ","
      << fbbr.plan_inflight << ","
      << fbbr.service_inflight << ","
      << fbbr.positive_probe_credit << ","
      << fbbr.service_budget << ","
      << fbbr.envelope << ","
      << fbbr.extra_acked << ","
      << fbbr.inflight_cap << ","
      << fbbr.native_cwnd << ","
      << fbbr.actual_inflight << ","
      << (fbbr.service_history_valid ? "true" : "false") << ","
      << (fbbr.app_limited_contaminated ? "true" : "false") << ","
      << (fbbr.projection_active ? "true" : "false") << ","
      << fbbr.service_restriction << ","
      << fbbr.enforced_excess << ","
      << fbbr_cap_binding_fraction;
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
  if (UsesFbbrServiceEnvelope()) {
    fbbr_window_ack_events_ = 0;
    fbbr_window_cap_binding_events_ = 0;
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
      trusted_bw_candidate_source_};
  PublishTrustedBwSelection(selection);
}

void FBBRSender::FinalizeCruise(QuicTime now) {
  RunWaveformCruiseStateMachine(now);
  PublishFbbrCruiseTrustedBw();
  EmitCruiseSummaryTrace(now);
}

void FBBRSender::EmitCruiseSummaryTrace(QuicTime now) const {
  if (!cruise_load_trace_cb_) {
    return;
  }

  const int candidate_count = 0;

  const double cruise_start_s =
      cruise_start_time_ == QuicTime::Zero()
          ? -1.0
          : static_cast<double>((cruise_start_time_ - QuicTime::Zero())
                                    .ToMicroseconds()) /
                1000000.0;
  const double cruise_end_s =
      static_cast<double>((now - QuicTime::Zero()).ToMicroseconds()) /
      1000000.0;
  const uint64_t cruise_end_native_bw_kbps =
      BandwidthEstimate().ToKBitsPerSecond();
  const QuicBandwidth summary_selection_native_bw =
      !selection_native_bw_.IsZero() ? selection_native_bw_
                                     : BandwidthEstimate();
  const QuicBandwidth summary_fbbr_max_bw = model_.MaxBandwidth();
  const double fair_share_bandwidth_kbps =
      static_cast<double>(fair_share_bandwidth_bps_) / 1000.0;

  std::ostringstream row;
  row << cruise_id_ << ","
      << cruise_start_s << ","
      << cruise_end_s << ","
      << candidate_count << ","
      << "false,-1,-1,0,0,0,0,0,0,0"
      << ","
	      << cruise_end_native_bw_kbps << ","
	      << fair_share_bandwidth_kbps << ","
	      << (trusted_bw_valid_ ? trusted_bw_.ToBitsPerSecond() : 0) << ","
	      << trusted_bw_source_ << ","
	      << "0,0,0,0,0,false,false,false,NOT_APPLICABLE,NOT_APPLICABLE,"
	      << summary_selection_native_bw.ToBitsPerSecond() << ","
	      << (trusted_bw_valid_ ? "true" : "false") << ","
	      << trusted_bw_cruise_id_ << ","
	      << (trusted_bw_fresh_ ? "true" : "false") << ","
	      << (trusted_bw_application_valid_ ? "true" : "false") << ","
	      << "time_waveform" << ","
	      << WaveformStateName(waveform_cruise_state_) << ","
	      << waveform_decision_count_ << ","
	      << baseline_adjustment_count_ << ","
	      << waveform_amplitude_reduction_count_ << ","
		      << (underload_located_ ? "true" : "false") << ","
		      << (trusted_bw_candidate_source_ == nullptr
		              ? kTrustedBwSourceNone
		              : trusted_bw_candidate_source_) << ","
		      << (UsesFbbrServiceEnvelope() &&
		              IsFinitePositiveBandwidth(summary_fbbr_max_bw)
		              ? "true" : "false") << ","
		      << (UsesFbbrServiceEnvelope() &&
		              IsFinitePositiveBandwidth(summary_fbbr_max_bw)
		              ? summary_fbbr_max_bw.ToBitsPerSecond() : 0) << ","
		      << (UsesFbbrServiceEnvelope() && fbbr_rtprop_valid_
		              ? "true" : "false") << ","
		      << (UsesFbbrServiceEnvelope() && fbbr_rtprop_valid_
		              ? static_cast<double>(fbbr_rtprop_.ToMicroseconds()) /
		                    1000.0
		              : 0.0) << ","
		      << (UsesFbbrServiceEnvelope() && fbbr_max_srtt_valid_
		              ? "true" : "false") << ","
		      << (UsesFbbrServiceEnvelope() && fbbr_max_srtt_valid_
		              ? fbbr_max_srtt_ms_ : 0.0);
  cruise_load_trace_cb_(cruise_start_s,
                        cruise_end_s,
                        0.0,
                        0.0,
                        0.0,
                        0.0,
                        "CRUISE_SUMMARY",
                        true,
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

double FBBRSender::Clamp01(double value) {
  if (!std::isfinite(value)) return 0.0;
  if (value < 0.0) return 0.0;
  if (value > 1.0) return 1.0;
  return value;
}

}  // namespace dqc
