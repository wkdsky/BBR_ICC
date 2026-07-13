#include "fbbr_frequency_search.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <complex>
#include <cstdlib>
#include <functional>
#include <iomanip>
#include <limits>
#include <map>
#include <numeric>
#include <random>
#include <sstream>

namespace dqc {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kEpsilon = 1e-12;
constexpr double kProbeMssBytes = 1460.0;

double Clamp(double value, double low, double high) {
  return std::max(low, std::min(high, value));
}

double Clamp01(double value) {
  return std::isfinite(value) ? Clamp(value, 0.0, 1.0) : 0.0;
}

double Logistic(double value) {
  return 1.0 / (1.0 + std::exp(-Clamp(value, -60.0, 60.0)));
}

std::string Trim(const std::string& value) {
  const std::string whitespace = " \t\r\n";
  const size_t first = value.find_first_not_of(whitespace);
  if (first == std::string::npos) return std::string();
  const size_t last = value.find_last_not_of(whitespace);
  return value.substr(first, last - first + 1);
}

bool ParseDouble(const std::string& text, double* value) {
  if (value == nullptr) return false;
  char* end = nullptr;
  const double parsed = std::strtod(text.c_str(), &end);
  if (end == text.c_str() || !Trim(end).empty() || !std::isfinite(parsed)) {
    return false;
  }
  *value = parsed;
  return true;
}

bool ParseUint(const std::string& text, uint32_t* value) {
  if (value == nullptr) return false;
  char* end = nullptr;
  const unsigned long parsed = std::strtoul(text.c_str(), &end, 10);
  if (end == text.c_str() || !Trim(end).empty() ||
      parsed > std::numeric_limits<uint32_t>::max()) {
    return false;
  }
  *value = static_cast<uint32_t>(parsed);
  return true;
}

bool ParseBool(const std::string& text, bool* value) {
  if (value == nullptr) return false;
  std::string normalized = Trim(text);
  std::transform(normalized.begin(), normalized.end(), normalized.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  if (normalized == "true" || normalized == "1" || normalized == "yes" ||
      normalized == "on") {
    *value = true;
    return true;
  }
  if (normalized == "false" || normalized == "0" || normalized == "no" ||
      normalized == "off") {
    *value = false;
    return true;
  }
  return false;
}

double Median(std::vector<double> values) {
  values.erase(std::remove_if(values.begin(), values.end(),
                              [](double v) { return !std::isfinite(v); }),
               values.end());
  if (values.empty()) return 0.0;
  std::sort(values.begin(), values.end());
  const size_t mid = values.size() / 2;
  return values.size() % 2 == 0 ? 0.5 * (values[mid - 1] + values[mid])
                                : values[mid];
}

double Percentile(std::vector<double> values, double fraction) {
  values.erase(std::remove_if(values.begin(), values.end(),
                              [](double v) { return !std::isfinite(v); }),
               values.end());
  if (values.empty()) return 0.0;
  std::sort(values.begin(), values.end());
  const double position = Clamp01(fraction) * (values.size() - 1);
  const size_t low = static_cast<size_t>(std::floor(position));
  const size_t high = static_cast<size_t>(std::ceil(position));
  if (low == high) return values[low];
  return values[low] + (values[high] - values[low]) * (position - low);
}

double WeightedMedian(std::vector<std::pair<double, double>> values) {
  values.erase(std::remove_if(values.begin(), values.end(),
                              [](const std::pair<double, double>& item) {
                                return !std::isfinite(item.first) ||
                                       !std::isfinite(item.second) ||
                                       item.second <= 0.0;
                              }),
               values.end());
  if (values.empty()) return 0.0;
  std::sort(values.begin(), values.end(),
            [](const std::pair<double, double>& lhs,
               const std::pair<double, double>& rhs) {
              return lhs.first < rhs.first;
            });
  double total = 0.0;
  for (const auto& item : values) total += item.second;
  double cumulative = 0.0;
  for (const auto& item : values) {
    cumulative += item.second;
    if (cumulative >= 0.5 * total) return item.first;
  }
  return values.back().first;
}

double RobustCv(const std::vector<double>& values) {
  const double center = Median(values);
  if (center <= kEpsilon) return std::numeric_limits<double>::infinity();
  std::vector<double> deviations;
  deviations.reserve(values.size());
  for (double value : values) deviations.push_back(std::abs(value - center));
  return 1.4826 * Median(deviations) / center;
}

std::string AppendReason(const std::string& current,
                         const std::string& reason) {
  if (reason.empty() || reason == "none") return current;
  if (current.empty() || current == "none") return reason;
  return current + "|" + reason;
}

struct AlignedSample {
  double output_time_s = 0.0;
  double input_time_s = 0.0;
  int64_t input_cycle = -1;
  double theta = 0.0;
  int code_sign = 1;
  double z = 0.0;
  double basis_sin = 0.0;
  double basis_cos = 0.0;
  double waveform_template = 0.0;
  double native_bps = 0.0;
  double commanded_bps = 0.0;
  double sent_bps = 0.0;
  double delivery_bps = 0.0;
  double qdelay_s = 0.0;
  double latest_rtt_s = 0.0;
  double loss_ratio = 0.0;
  double ecn_ratio = 0.0;
  double app_limited_fraction = 0.0;
  double cwnd_limited_fraction = 0.0;
  double recovery_fraction = 0.0;
  double queue_servo_factor = 1.0;
  double coverage = 0.0;
  double weight = 0.0;
  bool q_valid = false;
  bool phase_transition = false;
  bool queue_servo_transition = false;
};

struct FitInput {
  std::array<double, 7> basis;
  double value = 0.0;
  double weight = 0.0;
};

bool Solve7(double matrix[7][7], const double rhs[7], double solution[7],
            double* condition_number) {
  double augmented[7][8];
  double largest_pivot = 0.0;
  double smallest_pivot = std::numeric_limits<double>::infinity();
  for (int row = 0; row < 7; ++row) {
    for (int col = 0; col < 7; ++col) augmented[row][col] = matrix[row][col];
    augmented[row][7] = rhs[row];
  }
  for (int col = 0; col < 7; ++col) {
    int pivot_row = col;
    double pivot_abs = std::abs(augmented[col][col]);
    for (int row = col + 1; row < 7; ++row) {
      if (std::abs(augmented[row][col]) > pivot_abs) {
        pivot_abs = std::abs(augmented[row][col]);
        pivot_row = row;
      }
    }
    if (!std::isfinite(pivot_abs) || pivot_abs < 1e-18) return false;
    if (pivot_row != col) {
      for (int k = col; k < 8; ++k) {
        std::swap(augmented[pivot_row][k], augmented[col][k]);
      }
    }
    largest_pivot = std::max(largest_pivot, pivot_abs);
    smallest_pivot = std::min(smallest_pivot, pivot_abs);
    const double pivot = augmented[col][col];
    for (int k = col; k < 8; ++k) augmented[col][k] /= pivot;
    for (int row = 0; row < 7; ++row) {
      if (row == col) continue;
      const double factor = augmented[row][col];
      for (int k = col; k < 8; ++k) {
        augmented[row][k] -= factor * augmented[col][k];
      }
    }
  }
  for (int i = 0; i < 7; ++i) {
    solution[i] = augmented[i][7];
    if (!std::isfinite(solution[i])) return false;
  }
  if (condition_number != nullptr) {
    *condition_number = largest_pivot /
                        std::max(smallest_pivot, std::numeric_limits<double>::min());
  }
  return true;
}

FbbrHarmonicFitResult FitHarmonic(const std::vector<FitInput>& samples,
                                  const FBBRFrequencySearchConfig& config) {
  FbbrHarmonicFitResult result;
  result.sample_count = static_cast<uint32_t>(samples.size());
  if (samples.size() < 12) {
    result.invalid_reason = "fit_samples_insufficient";
    return result;
  }
  double normal[7][7] = {};
  double rhs[7] = {};
  double weight_sum = 0.0;
  double weighted_mean = 0.0;
  for (const auto& sample : samples) {
    if (sample.weight <= 0.0 || !std::isfinite(sample.value)) continue;
    weight_sum += sample.weight;
    weighted_mean += sample.weight * sample.value;
    for (int row = 0; row < 7; ++row) {
      rhs[row] += sample.weight * sample.basis[row] * sample.value;
      for (int col = 0; col < 7; ++col) {
        normal[row][col] +=
            sample.weight * sample.basis[row] * sample.basis[col];
      }
    }
  }
  if (weight_sum <= kEpsilon) {
    result.invalid_reason = "fit_weight_zero";
    return result;
  }
  weighted_mean /= weight_sum;
  double diagonal_mean = 0.0;
  for (int i = 0; i < 7; ++i) diagonal_mean += normal[i][i];
  diagonal_mean /= 7.0;
  const double ridge = config.ridge_epsilon * std::max(diagonal_mean, 1.0);
  for (int i = 1; i < 7; ++i) normal[i][i] += ridge;

  double beta[7] = {};
  if (!Solve7(normal, rhs, beta, &result.condition_number)) {
    result.invalid_reason = "fit_singular";
    return result;
  }
  if (!std::isfinite(result.condition_number) ||
      result.condition_number > config.max_condition_number) {
    result.invalid_reason = "fit_condition_high";
    return result;
  }
  for (int i = 0; i < 7; ++i) result.beta[i] = beta[i];

  double residual_sum = 0.0;
  double total_sum = 0.0;
  for (const auto& sample : samples) {
    double predicted = 0.0;
    for (int i = 0; i < 7; ++i) predicted += beta[i] * sample.basis[i];
    const double residual = sample.value - predicted;
    residual_sum += sample.weight * residual * residual;
    const double centered = sample.value - weighted_mean;
    total_sum += sample.weight * centered * centered;
  }
  result.residual_variance =
      residual_sum / std::max<double>(1.0, samples.size() - 7.0);
  result.r_squared = total_sum <= kEpsilon
                         ? (residual_sum <= 1e-10 ? 1.0 : 0.0)
                         : Clamp01(1.0 - residual_sum / total_sum);

  for (int col = 0; col < 7; ++col) {
    double unit[7] = {};
    double inverse_col[7] = {};
    double ignored_condition = 0.0;
    unit[col] = 1.0;
    if (!Solve7(normal, unit, inverse_col, &ignored_condition)) {
      result.invalid_reason = "fit_inverse_failed";
      return result;
    }
    result.standard_error[col] = std::sqrt(
        std::max(0.0, result.residual_variance * inverse_col[col]));
  }
  const double fundamental = std::hypot(result.beta[2], result.beta[3]);
  const double fundamental_error =
      std::hypot(result.standard_error[2], result.standard_error[3]);
  if (fundamental <= kEpsilon && fundamental_error <= kEpsilon) {
    result.snr = 0.0;
  } else {
    result.snr = fundamental / std::max(fundamental_error, 1e-9);
  }
  result.valid = std::all_of(result.beta.begin(), result.beta.end(),
                             [](double value) { return std::isfinite(value); });
  result.invalid_reason = result.valid ? "none" : "fit_non_finite";
  return result;
}

const FbbrPhaseBinSample* FindBin(const std::vector<FbbrPhaseBinSample>& bins,
                                  int64_t bin_index) {
  if (bin_index < 0 || bins.empty()) return nullptr;
  if (static_cast<size_t>(bin_index) < bins.size() &&
      bins[static_cast<size_t>(bin_index)].bin_index == bin_index) {
    return &bins[static_cast<size_t>(bin_index)];
  }
  const auto it = std::lower_bound(
      bins.begin(), bins.end(), bin_index,
      [](const FbbrPhaseBinSample& sample, int64_t index) {
        return sample.bin_index < index;
      });
  return it != bins.end() && it->bin_index == bin_index ? &*it : nullptr;
}

bool InterpolateInput(const std::vector<FbbrPhaseBinSample>& bins,
                      double time_s,
                      double* native_bps,
                      double* commanded_bps,
                      double* sent_bps,
                      double* queue_servo_factor,
                      double* coverage,
                      double* app_limited_fraction,
                      bool* phase_transition,
                      bool* queue_servo_transition) {
  if (bins.empty()) return false;
  const double bin_s = bins.front().time_end_s - bins.front().time_start_s;
  if (bin_s <= 0.0 || time_s < bins.front().time_start_s) return false;
  const double raw = (time_s - bins.front().time_start_s) / bin_s - 0.5;
  const int64_t left_index = static_cast<int64_t>(std::floor(raw));
  const double fraction = raw - std::floor(raw);
  const FbbrPhaseBinSample* left = FindBin(bins, left_index);
  const FbbrPhaseBinSample* right = FindBin(bins, left_index + 1);
  if (left == nullptr || right == nullptr || !left->valid || !right->valid ||
      left->native_pacing_bps <= 0.0 || right->native_pacing_bps <= 0.0) {
    return false;
  }
  const auto interpolate = [fraction](double lhs, double rhs) {
    return lhs + fraction * (rhs - lhs);
  };
  *native_bps = interpolate(left->native_pacing_bps, right->native_pacing_bps);
  *commanded_bps =
      interpolate(left->commanded_pacing_bps, right->commanded_pacing_bps);
  *sent_bps = interpolate(left->actual_send_bps, right->actual_send_bps);
  *queue_servo_factor = interpolate(left->queue_servo_factor,
                                    right->queue_servo_factor);
  *coverage = std::min(left->coverage, right->coverage);
  *app_limited_fraction = interpolate(left->app_limited_fraction,
                                      right->app_limited_fraction);
  *phase_transition = left->phase_transition || right->phase_transition;
  *queue_servo_transition = left->queue_servo_transition ||
                            right->queue_servo_transition;
  return std::isfinite(*native_bps) && std::isfinite(*commanded_bps) &&
         std::isfinite(*sent_bps) && std::isfinite(*queue_servo_factor);
}

std::vector<AlignedSample> BuildAlignedSamples(
    const FbbrProbeSignature& signature,
    const std::vector<FbbrPhaseBinSample>& bins,
    int64_t first_output_bin,
    int64_t output_bin_count,
    double delay_s) {
  std::vector<AlignedSample> output;
  if (signature.period_s <= 0.0 || bins.empty()) return output;
  output.reserve(static_cast<size_t>(std::max<int64_t>(0, output_bin_count)));
  const double cruise_start_s = bins.front().time_start_s;
  for (int64_t offset = 0; offset < output_bin_count; ++offset) {
    const FbbrPhaseBinSample* bin = FindBin(bins, first_output_bin + offset);
    if (bin == nullptr) continue;
    AlignedSample sample;
    sample.output_time_s = 0.5 * (bin->time_start_s + bin->time_end_s);
    sample.input_time_s = sample.output_time_s - delay_s;
    if (!InterpolateInput(bins, sample.input_time_s,
                          &sample.native_bps, &sample.commanded_bps,
                          &sample.sent_bps, &sample.queue_servo_factor,
                          &sample.coverage,
                          &sample.app_limited_fraction,
                          &sample.phase_transition,
                          &sample.queue_servo_transition)) {
      continue;
    }
    const double relative = sample.input_time_s - cruise_start_s;
    if (relative < 0.0) continue;
    const double cycle_position = relative / signature.period_s;
    sample.input_cycle = static_cast<int64_t>(std::floor(cycle_position));
    sample.theta = signature.initial_phase_rad + 2.0 * kPi * cycle_position;
    sample.code_sign = signature.waveform == "sine"
        ? FBBRFrequencySearch::WalshSign(signature.code_id, sample.input_cycle)
        : 1;
    sample.z = FBBRFrequencySearch::ProbeWaveform(signature, relative);
    sample.basis_sin = sample.code_sign * std::sin(sample.theta);
    sample.basis_cos = sample.code_sign * std::cos(sample.theta);
    sample.waveform_template = signature.waveform == "sine" ? 0.0 : sample.z;
    sample.delivery_bps = bin->delivery_rate_bps;
    sample.qdelay_s = bin->qdelay_s;
    sample.latest_rtt_s = bin->latest_rtt_s;
    sample.q_valid = bin->rtt_valid;
    sample.loss_ratio = bin->loss_ratio;
    sample.ecn_ratio = bin->ecn_ratio;
    sample.app_limited_fraction = std::max(sample.app_limited_fraction,
                                           bin->app_limited_fraction);
    sample.cwnd_limited_fraction = bin->cwnd_limited_fraction;
    sample.recovery_fraction = bin->recovery_fraction;
    sample.coverage = std::min(sample.coverage, bin->coverage);
    sample.phase_transition = sample.phase_transition || bin->phase_transition;
    sample.queue_servo_transition = sample.queue_servo_transition ||
                                    bin->queue_servo_transition;
    sample.weight = Clamp01(sample.coverage) *
                    (bin->acked_bytes > 0 ? 1.0 : 0.35);
    if (sample.queue_servo_transition) sample.weight = 0.0;
    output.push_back(sample);
  }
  return output;
}

std::array<double, 7> BuildBasis(const AlignedSample& sample,
                                 double center_time_s,
                                 double span_s) {
  const double normalized_time =
      2.0 * (sample.output_time_s - center_time_s) / std::max(span_s, 1e-9);
  return {{1.0,
           normalized_time,
           sample.basis_sin,
           sample.basis_cos,
           std::sin(2.0 * sample.theta),
           std::cos(2.0 * sample.theta),
           sample.waveform_template}};
}

std::complex<double> Fundamental(const FbbrHarmonicFitResult& fit) {
  return std::complex<double>(fit.beta[2], fit.beta[3]);
}

std::complex<double> DirectFundamental(
    const std::vector<AlignedSample>& samples,
    const std::function<double(const AlignedSample&)>& getter,
    bool require_q) {
  double mean = 0.0;
  double weight_sum = 0.0;
  for (const auto& sample : samples) {
    if (require_q && !sample.q_valid) continue;
    const double value = getter(sample);
    if (!std::isfinite(value) || sample.weight <= 0.0) continue;
    mean += sample.weight * value;
    weight_sum += sample.weight;
  }
  if (weight_sum <= kEpsilon) return {0.0, 0.0};
  mean /= weight_sum;
  double in_phase = 0.0;
  double quadrature = 0.0;
  for (const auto& sample : samples) {
    if (require_q && !sample.q_valid) continue;
    const double value = getter(sample);
    if (!std::isfinite(value) || sample.weight <= 0.0) continue;
    in_phase += sample.weight * (value - mean) * sample.basis_sin;
    quadrature += sample.weight * (value - mean) * sample.basis_cos;
  }
  return {2.0 * in_phase / weight_sum, 2.0 * quadrature / weight_sum};
}

std::complex<double> SecondHarmonic(const FbbrHarmonicFitResult& fit) {
  return std::complex<double>(fit.beta[4], fit.beta[5]);
}

double ComplexPhase(const std::complex<double>& value) {
  return std::atan2(value.imag(), value.real());
}

double PerCycleCoherence(const std::vector<AlignedSample>& samples,
                         bool queue_channel,
                         bool utility_channel,
                         double d0,
                         double rtprop_s,
                         double lambda_q) {
  struct Accumulator {
    double real = 0.0;
    double imag = 0.0;
    double mean = 0.0;
    double weight = 0.0;
  };
  std::map<int64_t, std::vector<const AlignedSample*>> cycles;
  for (const auto& sample : samples) cycles[sample.input_cycle].push_back(&sample);
  double vector_real = 0.0;
  double vector_imag = 0.0;
  double magnitude_sum = 0.0;
  for (const auto& entry : cycles) {
    double mean = 0.0;
    double weight_sum = 0.0;
    for (const AlignedSample* sample : entry.second) {
      if (queue_channel && !sample->q_valid) continue;
      double value = sample->delivery_bps / std::max(d0, 1.0) - 1.0;
      if (queue_channel) value = sample->qdelay_s;
      if (utility_channel) {
        if (!sample->q_valid) continue;
        value = std::log(std::max(sample->delivery_bps, 1.0) /
                         std::max(d0, 1.0)) -
                lambda_q * std::log((rtprop_s + sample->qdelay_s) /
                                    std::max(rtprop_s, 1e-9));
      }
      mean += sample->weight * value;
      weight_sum += sample->weight;
    }
    if (weight_sum <= kEpsilon) continue;
    mean /= weight_sum;
    double real = 0.0;
    double imag = 0.0;
    for (const AlignedSample* sample : entry.second) {
      if (queue_channel && !sample->q_valid) continue;
      double value = sample->delivery_bps / std::max(d0, 1.0) - 1.0;
      if (queue_channel) value = sample->qdelay_s;
      if (utility_channel) {
        if (!sample->q_valid) continue;
        value = std::log(std::max(sample->delivery_bps, 1.0) /
                         std::max(d0, 1.0)) -
                lambda_q * std::log((rtprop_s + sample->qdelay_s) /
                                    std::max(rtprop_s, 1e-9));
      }
      real += sample->weight * (value - mean) * sample->basis_sin;
      imag += sample->weight * (value - mean) * sample->basis_cos;
    }
    const double magnitude = std::hypot(real, imag);
    if (magnitude <= kEpsilon) continue;
    vector_real += real;
    vector_imag += imag;
    magnitude_sum += magnitude;
  }
  return magnitude_sum <= kEpsilon
             ? 0.0
             : Clamp01(std::hypot(vector_real, vector_imag) / magnitude_sum);
}

double PerCycleInputCoherence(const std::vector<AlignedSample>& samples) {
  std::map<int64_t, std::vector<const AlignedSample*>> cycles;
  for (const auto& sample : samples) cycles[sample.input_cycle].push_back(&sample);
  double vector_real = 0.0;
  double vector_imag = 0.0;
  double magnitude_sum = 0.0;
  for (const auto& entry : cycles) {
    double mean = 0.0;
    double weight_sum = 0.0;
    for (const AlignedSample* sample : entry.second) {
      const double x = (sample->sent_bps - sample->native_bps) /
                       std::max(sample->native_bps, 1.0);
      mean += sample->weight * x;
      weight_sum += sample->weight;
    }
    if (weight_sum <= kEpsilon) continue;
    mean /= weight_sum;
    double real = 0.0;
    double imag = 0.0;
    for (const AlignedSample* sample : entry.second) {
      const double x = (sample->sent_bps - sample->native_bps) /
                       std::max(sample->native_bps, 1.0);
      real += sample->weight * (x - mean) * sample->basis_sin;
      imag += sample->weight * (x - mean) * sample->basis_cos;
    }
    const double magnitude = std::hypot(real, imag);
    if (magnitude <= kEpsilon) continue;
    vector_real += real;
    vector_imag += imag;
    magnitude_sum += magnitude;
  }
  return magnitude_sum <= kEpsilon
             ? 0.0
             : Clamp01(std::hypot(vector_real, vector_imag) / magnitude_sum);
}

double RobustSlope(const std::vector<double>& values) {
  std::vector<double> slopes;
  for (size_t i = 0; i < values.size(); ++i) {
    if (!std::isfinite(values[i])) continue;
    for (size_t j = i + 1; j < values.size(); ++j) {
      if (!std::isfinite(values[j])) continue;
      slopes.push_back((values[j] - values[i]) /
                       static_cast<double>(j - i));
    }
  }
  return Median(slopes);
}

double RegionMedian(const std::vector<AlignedSample>& samples,
                    int region,
                    const std::function<double(const AlignedSample&)>& getter,
                    bool require_q) {
  std::vector<std::pair<double, double>> values;
  for (const auto& sample : samples) {
    if (require_q && !sample.q_valid) continue;
    const bool selected = region > 0 ? sample.z >= 0.70
                          : region < 0 ? sample.z <= -0.25
                                       : std::abs(sample.z) <= 0.08;
    if (selected) values.push_back({getter(sample), sample.weight});
  }
  return WeightedMedian(values);
}

struct AnalysisAtDelay {
  std::vector<AlignedSample> samples;
  FbbrHarmonicFitResult input_fit;
  FbbrHarmonicFitResult delivery_fit;
  FbbrHarmonicFitResult queue_fit;
  FbbrHarmonicFitResult utility_fit;
  double d0 = 0.0;
  double score = -1.0;
};

AnalysisAtDelay AnalyzeDelay(const FBBRFrequencySearchConfig& config,
                             const FbbrProbeSignature& signature,
                             const std::vector<FbbrPhaseBinSample>& bins,
                             int64_t first_output_bin,
                             int64_t output_bin_count,
                             double delay_s) {
  AnalysisAtDelay analysis;
  analysis.samples = BuildAlignedSamples(signature, bins, first_output_bin,
                                         output_bin_count, delay_s);
  if (analysis.samples.size() < 12) return analysis;
  int64_t newest_cycle = analysis.samples.front().input_cycle;
  for (const auto& sample : analysis.samples) {
    newest_cycle = std::max(newest_cycle, sample.input_cycle);
  }
  const double forgetting = Clamp(config.forgetting_factor, 0.50, 1.0);
  for (auto& sample : analysis.samples) {
    const int64_t age = std::max<int64_t>(0, newest_cycle - sample.input_cycle);
    sample.weight *= std::pow(forgetting, static_cast<double>(age));
  }
  std::vector<double> delivery_values;
  for (const auto& sample : analysis.samples) {
    if (sample.delivery_bps > 0.0) delivery_values.push_back(sample.delivery_bps);
  }
  analysis.d0 = Median(delivery_values);
  if (analysis.d0 <= 0.0) return analysis;
  const double start = analysis.samples.front().output_time_s;
  const double end = analysis.samples.back().output_time_s;
  const double center = 0.5 * (start + end);
  const double span = std::max(end - start, 1e-9);
  std::vector<FitInput> input;
  std::vector<FitInput> delivery;
  std::vector<FitInput> queue;
  std::vector<FitInput> utility;
  for (const auto& sample : analysis.samples) {
    const std::array<double, 7> basis = BuildBasis(sample, center, span);
    const double x = (sample.sent_bps - sample.native_bps) /
                     std::max(sample.native_bps, 1.0);
    input.push_back({basis, x, sample.weight});
    const double yd = sample.delivery_bps / analysis.d0 - 1.0;
    delivery.push_back({basis, yd, sample.weight});
    if (sample.q_valid) {
      queue.push_back({basis, sample.qdelay_s, sample.weight});
      const double utility_value =
          std::log(std::max(sample.delivery_bps, 1.0) / analysis.d0) -
          config.utility_delay_weight *
              std::log((signature.rtprop_s + sample.qdelay_s) /
                       std::max(signature.rtprop_s, 1e-9));
      utility.push_back({basis, utility_value, sample.weight});
    }
  }
  analysis.input_fit = FitHarmonic(input, config);
  analysis.delivery_fit = FitHarmonic(delivery, config);
  analysis.queue_fit = FitHarmonic(queue, config);
  analysis.utility_fit = FitHarmonic(utility, config);
  const double delivery_strength = analysis.delivery_fit.valid
      ? analysis.delivery_fit.snr *
            PerCycleCoherence(analysis.samples, false, false, analysis.d0,
                              signature.rtprop_s, config.utility_delay_weight)
      : 0.0;
  const double queue_strength = analysis.queue_fit.valid
      ? analysis.queue_fit.snr *
            PerCycleCoherence(analysis.samples, true, false, analysis.d0,
                              signature.rtprop_s, config.utility_delay_weight)
      : 0.0;
  analysis.score = std::max(delivery_strength, queue_strength);
  return analysis;
}

void FillCandidate(const FBBRFrequencySearchConfig& config,
                   const std::vector<AlignedSample>& samples,
                   double d0,
                   FbbrOperatingPointBlockResult* result) {
  FbbrTrustedBwCandidate candidate;
  candidate.invalid_reason = "not_near_optimal";
  if (result->classification != FbbrOperatingPointClassification::kNearOptimal) {
    result->candidate = candidate;
    return;
  }
  if (!result->lockable_score || !result->independent_for_trusted ||
      !result->trigger_cycle_excluded_from_score) {
    candidate.invalid_reason = "window_not_independent_or_lockable";
    result->candidate = candidate;
    return;
  }
  std::map<int64_t, std::vector<std::pair<double, double>>> plateau;
  for (const auto& sample : samples) {
    const double observable_queue_s = config.q_zero_absolute_s;
    if (!sample.q_valid || sample.z < 0.50 ||
        sample.qdelay_s <= observable_queue_s ||
        sample.qdelay_s > result->q_probe_max_s ||
        result->positive_delivery_gain >
            config.positive_delivery_gain_transition ||
        sample.delivery_bps < d0 || sample.app_limited_fraction > 0.0 ||
        sample.loss_ratio > 0.0 || sample.ecn_ratio > 0.0) {
      continue;
    }
    plateau[sample.input_cycle].push_back(
        {sample.delivery_bps, sample.weight});
  }
  std::vector<double> cycle_bandwidths;
  for (const auto& entry : plateau) {
    if (entry.second.size() < config.min_plateau_bins_per_cycle) continue;
    cycle_bandwidths.push_back(WeightedMedian(entry.second));
  }
  candidate.cycle_count = static_cast<uint32_t>(cycle_bandwidths.size());
  if (cycle_bandwidths.size() < config.min_plateau_cycles_per_block) {
    candidate.invalid_reason = "plateau_cycles_insufficient";
    result->candidate = candidate;
    return;
  }
  candidate.bandwidth_bps = Median(cycle_bandwidths);
  candidate.robust_cv = RobustCv(cycle_bandwidths);
  candidate.relative_ci_width =
      (Percentile(cycle_bandwidths, 0.95) -
       Percentile(cycle_bandwidths, 0.05)) /
      std::max(candidate.bandwidth_bps, 1.0);
  if (candidate.robust_cv > config.max_candidate_robust_cv) {
    candidate.invalid_reason = "candidate_cv_high";
  } else if (candidate.relative_ci_width >
             config.max_candidate_relative_ci_width) {
    candidate.invalid_reason = "candidate_ci_wide";
  } else if (!std::isfinite(candidate.bandwidth_bps) ||
             candidate.bandwidth_bps <= 0.0) {
    candidate.invalid_reason = "candidate_rate_invalid";
  } else {
    candidate.valid = true;
    candidate.invalid_reason = "none";
  }
  result->candidate = candidate;
}

}  // namespace

const char* FbbrOperatingPointClassificationName(
    FbbrOperatingPointClassification classification) {
  switch (classification) {
    case FbbrOperatingPointClassification::kUnderload:
      return "UNDERLOAD";
    case FbbrOperatingPointClassification::kNearOptimal:
      return "NEAR_OPTIMAL";
    case FbbrOperatingPointClassification::kQueuedOverload:
      return "QUEUED_OVERLOAD";
    case FbbrOperatingPointClassification::kBufferSaturated:
      return "BUFFER_SATURATED";
    case FbbrOperatingPointClassification::kDynamic:
      return "DYNAMIC";
    case FbbrOperatingPointClassification::kInvalid:
    default:
      return "INVALID";
  }
}

const char* EventWindowStateName(EventWindowState state) {
  switch (state) {
    case EventWindowState::kIdleListen: return "IDLE_LISTEN";
    case EventWindowState::kTriggerArmed: return "TRIGGER_ARMED";
    case EventWindowState::kCapture: return "CAPTURE";
    case EventWindowState::kContinuousTrack: return "CONTINUOUS_TRACK";
    case EventWindowState::kPaused: return "PAUSED";
    case EventWindowState::kPostBaselineSettling:
      return "POST_BASELINE_SETTLING";
  }
  return "IDLE_LISTEN";
}

const char* FBBRSearchStateName(FBBRSearchState state) {
  switch (state) {
    case FBBRSearchState::kDisabled: return "DISABLED";
    case FBBRSearchState::kAcquireInput: return "ACQUIRE_INPUT";
    case FBBRSearchState::kPulserElection: return "PULSER_ELECTION";
    case FBBRSearchState::kWatcher: return "WATCHER";
    case FBBRSearchState::kDrain: return "DRAIN";
    case FBBRSearchState::kSeek: return "SEEK";
    case FBBRSearchState::kTrack: return "TRACK";
    case FBBRSearchState::kLockCandidate: return "LOCK_CANDIDATE";
    case FBBRSearchState::kLocked: return "LOCKED";
    case FBBRSearchState::kEmergencyDrain: return "EMERGENCY_DRAIN";
    case FBBRSearchState::kDynamicReacquire: return "DYNAMIC_REACQUIRE";
    case FBBRSearchState::kPersistentUnresolved:
      return "PERSISTENT_UNRESOLVED";
  }
  return "DISABLED";
}

const char* FBBRQueueServoStateName(FBBRQueueServoState state) {
  switch (state) {
    case FBBRQueueServoState::kHold: return "HOLD";
    case FBBRQueueServoState::kDrain: return "DRAIN";
    case FBBRQueueServoState::kReserveRecovery: return "RESERVE_RECOVERY";
    case FBBRQueueServoState::kTargetBand: return "TARGET_BAND";
    case FBBRQueueServoState::kEmergencyDrain: return "EMERGENCY_DRAIN";
  }
  return "HOLD";
}

bool SetFBBRFrequencySearchConfigValue(FBBRFrequencySearchConfig* config,
                             const std::string& raw_key,
                             const std::string& value) {
  if (config == nullptr) return false;
  std::string key = raw_key;
  if (key.compare(0, 6, "f_bbr.") == 0) key = key.substr(6);
  else if (key.compare(0, 6, "opiv2.") == 0) key = key.substr(6);
  else return false;
  if (key.compare(0, 6, "opiv3.") == 0) key = key.substr(6);
  if (key.compare(0, 17, "frequency_search.") == 0) key = key.substr(17);
  if (key.compare(0, 18, "persistent_search.") == 0) key = key.substr(18);

  double d = 0.0;
  uint32_t u = 0;
  bool b = false;
#define FBBR_DOUBLE(KEY, FIELD) \
  if (key == KEY && ParseDouble(value, &d)) { config->FIELD = d; return true; }
#define FBBR_UINT(KEY, FIELD) \
  if (key == KEY && ParseUint(value, &u)) { config->FIELD = u; return true; }
#define FBBR_BOOL(KEY, FIELD) \
  if (key == KEY && ParseBool(value, &b)) { config->FIELD = b; return true; }
  FBBR_BOOL("frequency_search_enabled", frequency_search_enabled)
  FBBR_BOOL("legacy_spectral_path_enabled", legacy_spectral_path_enabled)
  if (key == "probe_period_rtt_slots") {
    std::vector<uint32_t> slots;
    std::stringstream stream(value);
    std::string token;
    while (std::getline(stream, token, ',')) {
      if (!ParseUint(Trim(token), &u) || u == 0) return false;
      slots.push_back(u);
    }
    if (slots.empty()) return false;
    config->probe_period_rtt_slots = slots;
    return true;
  }
  FBBR_UINT("probe_code_length_cycles", probe_code_length_cycles)
  FBBR_DOUBLE("probe_target_amplitude_ratio", probe_target_amplitude_ratio)
  FBBR_DOUBLE("probe_min_amplitude_ratio", probe_min_amplitude_ratio)
  FBBR_DOUBLE("probe_max_amplitude_ratio", probe_max_amplitude_ratio)
  FBBR_DOUBLE("probe_queue_budget_bdp", probe_queue_budget_bdp)
  FBBR_UINT("warmup_cycles", warmup_cycles)
  FBBR_UINT("analysis_cycles", analysis_cycles)
  FBBR_UINT("min_valid_cycles", min_valid_cycles)
  FBBR_UINT("phase_bins_per_cycle", phase_bins_per_cycle)
  FBBR_DOUBLE("min_bin_coverage", min_bin_coverage)
  FBBR_DOUBLE("min_non_app_limited_fraction", min_non_app_limited_fraction)
  FBBR_DOUBLE("max_input_amplitude_error_ratio", max_input_amplitude_error_ratio)
  FBBR_DOUBLE("min_input_cycle_coherence", min_input_cycle_coherence)
  FBBR_DOUBLE("max_native_baseline_drift", max_native_baseline_drift)
  FBBR_DOUBLE("delay_search_ratio_min", delay_search_ratio_min)
  FBBR_DOUBLE("delay_search_ratio_max", delay_search_ratio_max)
  FBBR_DOUBLE("delay_search_ratio_step", delay_search_ratio_step)
  FBBR_DOUBLE("max_cross_block_delay_shift_ratio", max_cross_block_delay_shift_ratio)
  FBBR_DOUBLE("ridge_epsilon", ridge_epsilon)
  FBBR_DOUBLE("max_condition_number", max_condition_number)
  FBBR_DOUBLE("min_detectable_snr", min_detectable_snr)
  FBBR_DOUBLE("min_measurement_confidence", min_measurement_confidence)
  FBBR_DOUBLE("utility_delay_weight", utility_delay_weight)
  FBBR_DOUBLE("q_zero_ratio", q_zero_ratio)
  FBBR_DOUBLE("q_zero_absolute_s", q_zero_absolute_s)
  FBBR_DOUBLE("q_probe_max_ratio", q_probe_max_ratio)
  FBBR_DOUBLE("min_drain_ratio", min_drain_ratio)
  FBBR_DOUBLE("max_queue_trend_per_cycle", max_queue_trend_per_cycle)
  FBBR_DOUBLE("max_probe_q95_ratio", max_probe_q95_ratio)
  FBBR_DOUBLE("max_loss_ratio_for_trusted", max_loss_ratio_for_trusted)
  FBBR_DOUBLE("max_ecn_ratio_for_trusted", max_ecn_ratio_for_trusted)
  FBBR_DOUBLE("gradient_zero_soft", gradient_zero_soft)
  FBBR_DOUBLE("gradient_zero_hard", gradient_zero_hard)
  FBBR_DOUBLE("positive_delivery_gain_saturated", positive_delivery_gain_saturated)
  FBBR_DOUBLE("positive_delivery_gain_transition", positive_delivery_gain_transition)
  FBBR_DOUBLE("positive_delivery_gain_underload", positive_delivery_gain_underload)
  FBBR_DOUBLE("queue_build_gain_threshold", queue_build_gain_threshold)
  FBBR_DOUBLE("min_full_score", min_full_score)
  FBBR_DOUBLE("min_low_queue_score", min_low_queue_score)
  FBBR_DOUBLE("min_stationary_score", min_stationary_score)
  FBBR_DOUBLE("min_safe_score", min_safe_score)
  FBBR_DOUBLE("min_optimality_score", min_optimality_score)
  FBBR_UINT("min_plateau_bins_per_cycle", min_plateau_bins_per_cycle)
  FBBR_UINT("min_plateau_cycles_per_block", min_plateau_cycles_per_block)
  FBBR_DOUBLE("max_candidate_robust_cv", max_candidate_robust_cv)
  FBBR_DOUBLE("max_candidate_relative_ci_width", max_candidate_relative_ci_width)
  FBBR_DOUBLE("max_interblock_candidate_diff", max_interblock_candidate_diff)
  FBBR_DOUBLE("history_small_change", history_small_change)
  FBBR_DOUBLE("history_medium_change", history_medium_change)
  FBBR_DOUBLE("medium_change_min_confidence", medium_change_min_confidence)
  FBBR_DOUBLE("large_change_min_confidence", large_change_min_confidence)
  FBBR_UINT("trace_verbosity", trace_verbosity)
  FBBR_BOOL("validation_shadow_windows", validation_shadow_windows)
  FBBR_UINT("validation_shadow_analysis_cycles", validation_shadow_analysis_cycles)
  FBBR_UINT("validation_shadow_stride_cycles", validation_shadow_stride_cycles)
  FBBR_UINT("validation_shadow_min_valid_cycles", validation_shadow_min_valid_cycles)
  FBBR_BOOL("search_controller_enabled", search_controller_enabled)
  FBBR_BOOL("persistent_across_cruises", persistent_across_cruises)
  FBBR_BOOL("never_disable_on_unresolved", never_disable_on_unresolved)
  FBBR_UINT("carrier_sensing_cycles", carrier_sensing_cycles)
  FBBR_UINT("pulser_backoff_max_cycles", pulser_backoff_max_cycles)
  FBBR_UINT("pulser_lease_decisions", pulser_lease_decisions)
  FBBR_DOUBLE("watcher_probe_amplitude", watcher_probe_amplitude)
  FBBR_UINT("settling_cycles_per_window", settling_cycles_per_window)
  FBBR_UINT("analysis_cycles_per_window", analysis_cycles_per_window)
  FBBR_UINT("min_valid_analysis_cycles", min_valid_analysis_cycles)
  FBBR_UINT("decision_stride_cycles", decision_stride_cycles)
  FBBR_DOUBLE("forgetting_factor", forgetting_factor)
  FBBR_UINT("min_control_windows", min_control_windows)
  FBBR_UINT("target_control_windows", target_control_windows)
  FBBR_UINT("max_control_windows", max_control_windows)
  FBBR_UINT("max_cruise_extension_rtts", max_cruise_extension_rtts)
  FBBR_DOUBLE("max_cruise_extension_s", max_cruise_extension_s)
  FBBR_DOUBLE("carrier_period_min_s", carrier_period_min_s)
  FBBR_DOUBLE("carrier_period_max_s", carrier_period_max_s)
  if (key == "waveform" || key == "probe_waveform") {
    const std::string normalized = Trim(value);
    if (normalized != "sine" && normalized != "asymmetric_zero_mean") {
      return false;
    }
    config->probe_waveform = normalized;
    return true;
  }
  FBBR_DOUBLE("target_amplitude_ratio", probe_target_amplitude_ratio)
  FBBR_DOUBLE("min_amplitude_ratio", probe_min_amplitude_ratio)
  FBBR_DOUBLE("max_amplitude_ratio", probe_max_amplitude_ratio)
  FBBR_DOUBLE("min_extra_probe_mss", min_extra_probe_mss)
  FBBR_DOUBLE("actual_amplitude_target_acquire", actual_amplitude_target_acquire)
  FBBR_DOUBLE("actual_amplitude_target_seek", actual_amplitude_target_seek)
  FBBR_DOUBLE("actual_amplitude_target_track", actual_amplitude_target_track)
  FBBR_DOUBLE("actual_amplitude_target_drain", actual_amplitude_target_drain)
  FBBR_DOUBLE("actual_amplitude_target_locked", actual_amplitude_target_locked)
  FBBR_DOUBLE("amplitude_adaptation_mu", amplitude_adaptation_mu)
  FBBR_DOUBLE("amplitude_step_max", amplitude_step_max)
  FBBR_DOUBLE("probe_queue_budget_acquire_bdp", probe_queue_budget_acquire_bdp)
  FBBR_DOUBLE("probe_queue_budget_track_bdp", probe_queue_budget_track_bdp)
  FBBR_DOUBLE("probe_queue_budget_drain_bdp", probe_queue_budget_drain_bdp)
  FBBR_DOUBLE("carrier_detection_snr_min", carrier_detection_snr_min)
  FBBR_DOUBLE("carrier_detection_amplitude_min", carrier_detection_amplitude_min)
  FBBR_DOUBLE("realized_amplitude_ratio_min", realized_amplitude_ratio_min)
  FBBR_DOUBLE("realized_amplitude_ratio_max", realized_amplitude_ratio_max)
  FBBR_DOUBLE("input_coherence_min", input_coherence_min)
  FBBR_DOUBLE("input_snr_min", input_snr_min)
  FBBR_DOUBLE("max_cwnd_limited_fraction", max_cwnd_limited_fraction)
  FBBR_DOUBLE("max_app_limited_fraction", max_app_limited_fraction)
  FBBR_DOUBLE("max_recovery_fraction", max_recovery_fraction)
  FBBR_DOUBLE("phase_coverage_min", phase_coverage_min)
  FBBR_DOUBLE("baseline_drift_max", baseline_drift_max)
  FBBR_DOUBLE("regression_condition_max", regression_condition_max)
  FBBR_DOUBLE("actual_input_min_ratio", actual_input_min_ratio)
  FBBR_DOUBLE("max_signature_leakage", max_signature_leakage)
  FBBR_DOUBLE("max_residual_to_carrier_ratio", max_residual_to_carrier_ratio)
  FBBR_DOUBLE("measurement_confidence_update_min", measurement_confidence_update_min)
  FBBR_DOUBLE("measurement_confidence_track_min", measurement_confidence_track_min)
  FBBR_DOUBLE("measurement_confidence_lock_min", measurement_confidence_lock_min)
  FBBR_DOUBLE("near_optimal_score_threshold", near_optimal_score_threshold)
  FBBR_DOUBLE("near_optimal_direction_abs_max", near_optimal_direction_abs_max)
  FBBR_DOUBLE("utility_gradient_scale", utility_gradient_scale)
  FBBR_DOUBLE("utility_loss_weight", utility_loss_weight)
  FBBR_DOUBLE("utility_ecn_weight", utility_ecn_weight)
  FBBR_DOUBLE("ordinary_up_step_max", ordinary_up_step_max)
  FBBR_DOUBLE("confirmed_up_step_max", confirmed_up_step_max)
  FBBR_DOUBLE("ordinary_down_step_max", ordinary_down_step_max)
  FBBR_DOUBLE("track_step_max", track_step_max)
  FBBR_DOUBLE("q_floor_enter_drain_ratio", q_floor_enter_drain_ratio)
  FBBR_DOUBLE("q_floor_exit_drain_ratio", q_floor_exit_drain_ratio)
  FBBR_DOUBLE("drain_baseline_beta", drain_baseline_beta)
  FBBR_DOUBLE("drain_delivery_beta", drain_delivery_beta)
  FBBR_UINT("underload_confirmation_windows", underload_confirmation_windows)
  FBBR_DOUBLE("same_direction_streak_multiplier_max", same_direction_streak_multiplier_max)
  FBBR_DOUBLE("bracket_lock_width", bracket_lock_width)
  FBBR_UINT("bracket_ttl_windows", bracket_ttl_windows)
  FBBR_DOUBLE("min_search_scale", min_search_scale)
  FBBR_DOUBLE("max_search_scale", max_search_scale)
  FBBR_DOUBLE("soft_loss_threshold", soft_loss_threshold)
  FBBR_DOUBLE("hard_loss_threshold", hard_loss_threshold)
  FBBR_DOUBLE("hard_loss_beta", hard_loss_beta)
  FBBR_DOUBLE("soft_ecn_threshold", soft_ecn_threshold)
  FBBR_DOUBLE("hard_ecn_threshold", hard_ecn_threshold)
  FBBR_DOUBLE("dynamic_native_change_threshold", dynamic_native_change_threshold)
  FBBR_DOUBLE("dynamic_delivery_trend_threshold", dynamic_delivery_trend_threshold)
  FBBR_DOUBLE("dynamic_delay_shift_threshold", dynamic_delay_shift_threshold)
  FBBR_UINT("dynamic_reset_windows", dynamic_reset_windows)
  FBBR_UINT("trusted_candidate_min_windows", trusted_candidate_min_windows)
  FBBR_DOUBLE("trusted_candidate_cv_max", trusted_candidate_cv_max)
  FBBR_DOUBLE("trusted_candidate_ratio_max", trusted_candidate_ratio_max)
  FBBR_UINT("trusted_ttl_cruises", trusted_ttl_cruises)
  FBBR_UINT("bracket_ttl_cruises", bracket_ttl_cruises)
  FBBR_DOUBLE("provisional_native_change_max", provisional_native_change_max)
  FBBR_DOUBLE("provisional_rtprop_change_max", provisional_rtprop_change_max)
  FBBR_DOUBLE("rtprop_confidence_lock_min", rtprop_confidence_lock_min)
  FBBR_BOOL("event_triggered_windows_enabled", event_triggered_windows_enabled)
  FBBR_DOUBLE("trigger.delivery.prominence_start",
              delivery_trigger_prominence_start)
  FBBR_DOUBLE("trigger.delivery.prominence_continue",
              delivery_trigger_prominence_continue)
  FBBR_DOUBLE("trigger.delivery.match_start", delivery_trigger_match_start)
  FBBR_DOUBLE("trigger.delivery.match_continue", delivery_trigger_match_continue)
  FBBR_DOUBLE("trigger.delivery.min_response_mss",
              delivery_trigger_min_response_mss)
  FBBR_DOUBLE("trigger.queue.prominence_start", queue_trigger_prominence_start)
  FBBR_DOUBLE("trigger.queue.prominence_continue",
              queue_trigger_prominence_continue)
  FBBR_DOUBLE("trigger.queue.match_start", queue_trigger_match_start)
  FBBR_DOUBLE("trigger.queue.match_continue", queue_trigger_match_continue)
  FBBR_DOUBLE("trigger.queue.noise_mad_multiplier",
              queue_trigger_noise_mad_multiplier)
  FBBR_DOUBLE("trigger.queue.min_timestamp_quanta",
              queue_trigger_min_timestamp_quanta)
  FBBR_DOUBLE("trigger.period_tolerance_start",
              trigger_period_tolerance_start)
  FBBR_DOUBLE("trigger.period_tolerance_continue",
              trigger_period_tolerance_continue)
  FBBR_DOUBLE("trigger.phase_coverage_min", trigger_phase_coverage_min)
  FBBR_DOUBLE("trigger_period_tolerance_ratio", trigger_period_tolerance_ratio)
  FBBR_DOUBLE("trigger_spectral_prominence_min", trigger_spectral_prominence_min)
  FBBR_DOUBLE("continue_spectral_prominence_min", continue_spectral_prominence_min)
  FBBR_DOUBLE("trigger_normalized_match_min", trigger_normalized_match_min)
  FBBR_DOUBLE("continue_normalized_match_min", continue_normalized_match_min)
  FBBR_DOUBLE("min_delivery_response_bytes_per_cycle_mss",
              min_delivery_response_bytes_per_cycle_mss)
  FBBR_DOUBLE("trigger_phase_coverage_min", trigger_phase_coverage_min)
  FBBR_UINT("min_direction_cycles", min_direction_cycles)
  FBBR_UINT("min_score_cycles", min_score_cycles)
  FBBR_UINT("target_window_cycles", target_window_cycles)
  FBBR_UINT("max_window_cycles", max_window_cycles)
  FBBR_UINT("tracking_window_cycles", tracking_window_cycles)
  FBBR_DOUBLE("tracking_stride_cycles", tracking_stride_cycles)
  FBBR_DOUBLE("diagnostic_stride_cycles", diagnostic_stride_cycles)
  FBBR_DOUBLE("control_decision_stride_cycles", control_decision_stride_cycles)
  FBBR_UINT("trusted_independent_stride_cycles",
            trusted_independent_stride_cycles)
  FBBR_UINT("bad_cycles_to_pause", bad_cycles_to_pause)
  FBBR_UINT("post_update_settling_cycles", post_update_settling_cycles)
  FBBR_DOUBLE("sequential_score_delta_max", sequential_score_delta_max)
  FBBR_DOUBLE("direction_evidence_min", direction_evidence_min)
  FBBR_DOUBLE("q_reserve_low_bdp", q_reserve_low_bdp)
  FBBR_DOUBLE("q_reserve_high_bdp", q_reserve_high_bdp)
  FBBR_DOUBLE("q_peak_cap_bdp", q_peak_cap_bdp)
  FBBR_DOUBLE("queue_reserve.low_bdp", q_reserve_low_bdp)
  FBBR_DOUBLE("queue_reserve.high_bdp", q_reserve_high_bdp)
  FBBR_DOUBLE("queue_reserve.peak_cap_bdp", q_peak_cap_bdp)
  FBBR_DOUBLE("direction.channel_split_weight", channel_split_weight)
  FBBR_DOUBLE("direction.utility_gradient_weight", utility_gradient_weight)
  FBBR_DOUBLE("direction.slow_frequency_weight", slow_frequency_weight)
  FBBR_DOUBLE("direction.slow_queue_weight", slow_queue_weight)
  FBBR_DOUBLE("direction.slow_trend_weight", slow_trend_weight)
  FBBR_DOUBLE("direction_frequency_weight", direction_frequency_weight)
  FBBR_DOUBLE("direction_queue_weight", direction_queue_weight)
  FBBR_DOUBLE("direction_trend_weight", direction_trend_weight)
  FBBR_DOUBLE("period_increase_factor", period_increase_factor)
  FBBR_DOUBLE("amplitude_increase_factor", amplitude_increase_factor)
  FBBR_DOUBLE("max_amplitude_change_per_update",
              max_amplitude_change_per_update)
  FBBR_DOUBLE("acquire_probe_budget_bdp", acquire_probe_budget_bdp)
  FBBR_DOUBLE("track_probe_budget_bdp", track_probe_budget_bdp)
  FBBR_DOUBLE("locked_probe_budget_bdp", locked_probe_budget_bdp)
  FBBR_DOUBLE("mild_drain_step_min", mild_drain_step_min)
  FBBR_DOUBLE("mild_drain_step_max", mild_drain_step_max)
  FBBR_DOUBLE("mild_drain_delivery_floor", mild_drain_delivery_floor)
  FBBR_UINT("min_pulser_lease_cycles", min_pulser_lease_cycles)
  FBBR_UINT("max_pulser_lease_cycles", max_pulser_lease_cycles)
  FBBR_BOOL("queue_servo.enabled", queue_servo_enabled)
  FBBR_DOUBLE("queue_servo.update_rtts", queue_servo_update_rtts)
  FBBR_DOUBLE("queue_servo.high_gain", queue_servo_high_gain)
  FBBR_DOUBLE("queue_servo.trend_gain", queue_servo_trend_gain)
  FBBR_DOUBLE("queue_servo.low_gain", queue_servo_low_gain)
  FBBR_DOUBLE("queue_servo.down_step_max", queue_servo_down_step_max)
  FBBR_DOUBLE("queue_servo.up_step_max", queue_servo_up_step_max)
  FBBR_DOUBLE("queue_servo.recovery_step_max",
              queue_servo_recovery_step_max)
  FBBR_DOUBLE("queue_servo.delivery_drain_factor",
              queue_servo_delivery_drain_factor)
  FBBR_DOUBLE("queue_servo.hard_queue_multiple",
              queue_servo_hard_queue_multiple)
  FBBR_DOUBLE("queue_servo.emergency_factor",
              queue_servo_emergency_factor)
  FBBR_UINT("queue_servo.commit_min_rtts", queue_servo_commit_min_rtts)
  FBBR_DOUBLE("queue_servo.commit_step_max", queue_servo_commit_step_max)
#undef FBBR_DOUBLE
#undef FBBR_UINT
#undef FBBR_BOOL
  return false;
}

int FBBRFrequencySearch::WalshSign(uint32_t code_id, int64_t cycle_index) {
  static const int kBalancedCodes[3][4] = {
      {1, -1, -1, 1},
      {1, -1, 1, -1},
      {1, 1, -1, -1},
  };
  const uint32_t row = code_id % 3u;
  const uint32_t column = static_cast<uint32_t>(cycle_index) & 3u;
  return kBalancedCodes[row][column];
}

double FBBRFrequencySearch::CodedSine(
    const FbbrProbeSignature& signature,
    double time_since_cruise_start_s) {
  if (signature.period_s <= 0.0 || time_since_cruise_start_s < 0.0) return 0.0;
  const double cycle_position = time_since_cruise_start_s / signature.period_s;
  const int64_t cycle = static_cast<int64_t>(std::floor(cycle_position));
  const double theta = signature.initial_phase_rad + 2.0 * kPi * cycle_position;
  return WalshSign(signature.code_id, cycle) * std::sin(theta);
}

double FBBRFrequencySearch::ProbeWaveform(
    const FbbrProbeSignature& signature,
    double time_since_cruise_start_s) {
  if (signature.period_s <= 0.0 || time_since_cruise_start_s < 0.0) return 0.0;
  if (signature.waveform == "sine") {
    return CodedSine(signature, time_since_cruise_start_s);
  }
  const double cycle_position = time_since_cruise_start_s / signature.period_s;
  const double phase = cycle_position - std::floor(cycle_position);
  if (phase < 0.25) {
    return std::sin(kPi * phase / 0.25);
  }
  return -(1.0 / 3.0) *
      std::sin(kPi * (phase - 0.25) / 0.75);
}

FBBRTriggerCycleResult FBBRFrequencySearch::AnalyzeTriggerCycle(
    const FBBRFrequencySearchConfig& config,
    const FbbrProbeSignature& signature,
    const std::vector<FbbrPhaseBinSample>& bins,
    int64_t output_cycle,
    EventWindowState window_state,
    bool is_pulser) {
  FBBRTriggerCycleResult result;
  result.cruise_id = signature.cruise_id;
  result.cycle_id = output_cycle;
  result.window_state = window_state;
  result.is_pulser = is_pulser;
  result.carrier_period_s = signature.period_s;
  if (signature.period_s <= 0.0 || signature.rtprop_s <= 0.0 ||
      config.phase_bins_per_cycle == 0 || output_cycle < 0) {
    result.trigger_reason = "signature_invalid";
    return result;
  }

  const int64_t bins_per_cycle = config.phase_bins_per_cycle;
  const int64_t first_bin = output_cycle * bins_per_cycle;
  const FbbrPhaseBinSample* first = FindBin(bins, first_bin);
  const FbbrPhaseBinSample* last =
      FindBin(bins, first_bin + bins_per_cycle - 1);
  if (first == nullptr || last == nullptr) {
    result.trigger_reason = "cycle_incomplete";
    return result;
  }
  result.cycle_start_s = first->time_start_s;
  result.cycle_end_s = last->time_end_s;

  std::vector<double> times;
  std::vector<double> delivery;
  std::vector<double> queue_times;
  std::vector<double> queue;
  double coverage_sum = 0.0;
  double app_sum = 0.0;
  double recovery_sum = 0.0;
  double loss_sum = 0.0;
  double ecn_sum = 0.0;
  std::vector<double> native_values;
  const int64_t spectral_first_bin = std::max<int64_t>(
      0, first_bin - bins_per_cycle);
  for (int64_t index = spectral_first_bin;
       index < first_bin + bins_per_cycle; ++index) {
    const FbbrPhaseBinSample* bin = FindBin(bins, index);
    if (bin == nullptr || !bin->valid) continue;
    times.push_back(0.5 * (bin->time_start_s + bin->time_end_s));
    delivery.push_back(bin->delivery_rate_bps);
    if (bin->rtt_valid) {
      queue_times.push_back(times.back());
      queue.push_back(bin->qdelay_s);
    }
    coverage_sum += bin->coverage;
    app_sum += bin->app_limited_fraction;
    recovery_sum += bin->recovery_fraction;
    loss_sum += bin->loss_ratio;
    ecn_sum += bin->ecn_ratio;
    native_values.push_back(
        bin->native_pacing_bps / std::max(bin->queue_servo_factor, 1e-9));
  }
  if (times.size() < static_cast<size_t>(std::max<int64_t>(12, bins_per_cycle * 3 / 4))) {
    result.trigger_reason = "cycle_samples_insufficient";
    return result;
  }
  result.phase_coverage = coverage_sum / times.size();
  result.app_limited_fraction = app_sum / times.size();
  result.recovery_fraction = recovery_sum / times.size();
  const double native_median = Median(native_values);
  result.baseline_drift = native_median > 0.0
      ? (Percentile(native_values, 0.95) - Percentile(native_values, 0.05)) /
            native_median
      : std::numeric_limits<double>::infinity();

  auto projection = [](const std::vector<double>& sample_times,
                       const std::vector<double>& values,
                       double frequency_hz) {
    std::complex<double> output(0.0, 0.0);
    if (values.size() != sample_times.size() || values.empty() ||
        frequency_hz <= 0.0) {
      return output;
    }
    double mean = 0.0;
    double window_sum = 0.0;
    for (size_t i = 0; i < values.size(); ++i) {
      const double window = values.size() <= 1 ? 1.0 :
          0.5 - 0.5 * std::cos(2.0 * kPi * i / (values.size() - 1.0));
      mean += window * values[i];
      window_sum += window;
    }
    mean /= std::max(window_sum, 1e-9);
    for (size_t i = 0; i < values.size(); ++i) {
      const double window = values.size() <= 1 ? 1.0 :
          0.5 - 0.5 * std::cos(2.0 * kPi * i / (values.size() - 1.0));
      const double theta = 2.0 * kPi * frequency_hz * sample_times[i];
      output += window * (values[i] - mean) *
                std::complex<double>(std::sin(theta), std::cos(theta));
    }
    return 2.0 * output / std::max(window_sum, 1e-9);
  };
  auto estimate_period = [&projection, &signature](
                             const std::vector<double>& sample_times,
                             const std::vector<double>& values) {
    double best_frequency = 1.0 / signature.period_s;
    double best_amplitude = 0.0;
    for (int step = 0; step <= 18; ++step) {
      const double ratio = 0.80 + 0.025 * step;
      const double frequency = ratio / signature.period_s;
      const double amplitude = std::abs(
          projection(sample_times, values, frequency));
      if (amplitude > best_amplitude) {
        best_amplitude = amplitude;
        best_frequency = frequency;
      }
    }
    return 1.0 / std::max(best_frequency, 1e-9);
  };
  auto normalized_match = [](const std::vector<double>& values,
                             double amplitude) {
    if (values.empty()) return 0.0;
    const double mean = std::accumulate(values.begin(), values.end(), 0.0) /
                        values.size();
    double variance = 0.0;
    for (double value : values) variance += (value - mean) * (value - mean);
    variance /= std::max<size_t>(1, values.size());
    return Clamp01(amplitude /
                   std::max(std::sqrt(2.0 * variance), 1e-12));
  };
  auto cycle_repeat_correlation = [&bins, first_bin, bins_per_cycle](
                                      bool queue_channel) {
    std::vector<double> previous;
    std::vector<double> current;
    previous.reserve(static_cast<size_t>(bins_per_cycle));
    current.reserve(static_cast<size_t>(bins_per_cycle));
    for (int64_t phase_bin = 0; phase_bin < bins_per_cycle; ++phase_bin) {
      const FbbrPhaseBinSample* lhs = FindBin(
          bins, first_bin - bins_per_cycle + phase_bin);
      const FbbrPhaseBinSample* rhs = FindBin(bins, first_bin + phase_bin);
      if (lhs == nullptr || rhs == nullptr || !lhs->valid || !rhs->valid ||
          (queue_channel && (!lhs->rtt_valid || !rhs->rtt_valid))) {
        continue;
      }
      previous.push_back(queue_channel ? lhs->qdelay_s
                                       : lhs->delivery_rate_bps);
      current.push_back(queue_channel ? rhs->qdelay_s
                                      : rhs->delivery_rate_bps);
    }
    if (previous.size() < static_cast<size_t>(
            std::max<int64_t>(8, bins_per_cycle / 2))) {
      return -1.0;
    }
    const double previous_mean = std::accumulate(
        previous.begin(), previous.end(), 0.0) / previous.size();
    const double current_mean = std::accumulate(
        current.begin(), current.end(), 0.0) / current.size();
    double covariance = 0.0;
    double previous_energy = 0.0;
    double current_energy = 0.0;
    for (size_t i = 0; i < previous.size(); ++i) {
      const double lhs = previous[i] - previous_mean;
      const double rhs = current[i] - current_mean;
      covariance += lhs * rhs;
      previous_energy += lhs * lhs;
      current_energy += rhs * rhs;
    }
    return covariance /
        std::max(std::sqrt(previous_energy * current_energy), 1e-18);
  };
  const double carrier_hz = 1.0 / signature.period_s;
  const double omega = 2.0 * kPi * carrier_hz;
  const double target_amplitude = std::abs(
      projection(times, delivery, carrier_hz));
  double delivery_neighbor_amplitude = 0.0;
  for (double ratio : {1.50, 1.75}) {
    delivery_neighbor_amplitude = std::max(
        delivery_neighbor_amplitude,
        std::abs(projection(times, delivery, ratio * carrier_hz)));
  }
  result.delivery_response_amplitude_bps = target_amplitude;
  result.delivery_response_bytes = target_amplitude * signature.period_s / 8.0;
  result.delivery_spectral_prominence = target_amplitude /
      std::max(delivery_neighbor_amplitude, 1.0);
  result.delivery_normalized_match = normalized_match(delivery, target_amplitude);
  result.delivery_period_estimate_s = estimate_period(times, delivery);
  result.delivery_period_error_ratio = std::abs(
      result.delivery_period_estimate_s - signature.period_s) /
      signature.period_s;
  if (cycle_repeat_correlation(false) >= 0.50) {
    result.delivery_period_estimate_s = signature.period_s;
    result.delivery_period_error_ratio = 0.0;
  }

  double queue_target_amplitude = 0.0;
  if (queue.size() >= static_cast<size_t>(std::max<int64_t>(8, bins_per_cycle / 2))) {
    const double queue_slope = RobustSlope(queue);
    const double queue_center = 0.5 * static_cast<double>(queue.size() - 1);
    std::vector<double> queue_signal;
    queue_signal.reserve(queue.size());
    for (size_t i = 0; i < queue.size(); ++i) {
      queue_signal.push_back(queue[i] -
          queue_slope * (static_cast<double>(i) - queue_center));
    }
    if (queue_signal.size() >= 5) {
      std::vector<double> smoothed(queue_signal);
      for (size_t i = 2; i + 2 < queue_signal.size(); ++i) {
        smoothed[i] = (-3.0 * queue_signal[i - 2] +
                       12.0 * queue_signal[i - 1] +
                       17.0 * queue_signal[i] +
                       12.0 * queue_signal[i + 1] -
                       3.0 * queue_signal[i + 2]) / 35.0;
      }
      queue_signal.swap(smoothed);
    }
    const std::complex<double> queue_component =
        projection(queue_times, queue_signal, carrier_hz);
    queue_target_amplitude = std::abs(queue_component);
    result.queue_derivative_amplitude = omega * queue_target_amplitude;
    double queue_neighbor_amplitude = 0.0;
    // The production asymmetric waveform has intentional integer harmonics.
    // Use off-harmonic neighbors so legitimate probe energy is not treated as
    // the queue detector's noise reference.
    for (double ratio : {1.75, 2.50}) {
      queue_neighbor_amplitude = std::max(
          queue_neighbor_amplitude,
          ratio * omega * std::abs(projection(
              queue_times, queue_signal, ratio * carrier_hz)));
    }
    result.queue_spectral_prominence = result.queue_derivative_amplitude /
        std::max(queue_neighbor_amplitude, 1e-9);
    result.queue_normalized_match = normalized_match(
        queue_signal, queue_target_amplitude);
    result.queue_period_estimate_s = estimate_period(queue_times, queue_signal);
    result.queue_period_error_ratio = std::abs(
        result.queue_period_estimate_s - signature.period_s) /
        signature.period_s;
    if (cycle_repeat_correlation(true) >= 0.50) {
      result.queue_period_estimate_s = signature.period_s;
      result.queue_period_error_ratio = 0.0;
    }

    const double queue_mean = std::accumulate(
        queue_signal.begin(), queue_signal.end(), 0.0) / queue_signal.size();
    std::vector<double> residuals;
    residuals.reserve(queue_signal.size());
    for (size_t i = 0; i < queue_signal.size(); ++i) {
      const double theta = 2.0 * kPi * carrier_hz * queue_times[i];
      const double predicted = queue_component.real() * std::sin(theta) +
                               queue_component.imag() * std::cos(theta);
      residuals.push_back(queue_signal[i] - queue_mean - predicted);
    }
    const double residual_center = Median(residuals);
    std::vector<double> absolute_deviations;
    absolute_deviations.reserve(residuals.size());
    for (double residual : residuals) {
      absolute_deviations.push_back(std::abs(residual - residual_center));
    }
    const double residual_mad = 1.4826 * Median(absolute_deviations);
    const double timestamp_quantum_s = 1e-6;
    result.queue_noise_floor = std::max(
        config.queue_trigger_noise_mad_multiplier * omega * residual_mad,
        config.queue_trigger_min_timestamp_quanta * omega *
            timestamp_quantum_s);
  }

  double best_match = -1.0;
  std::vector<AlignedSample> best_samples;
  for (double ratio = 0.50; ratio <= 1.50 + 1e-9; ratio += 0.05) {
    std::vector<AlignedSample> samples = BuildAlignedSamples(
        signature, bins, first_bin, bins_per_cycle, ratio * signature.rtprop_s);
    if (samples.size() < 12) continue;
    const std::complex<double> delivery_component = DirectFundamental(
        samples, [](const AlignedSample& sample) { return sample.delivery_bps; },
        false);
    std::vector<double> values;
    values.reserve(samples.size());
    for (const auto& sample : samples) values.push_back(sample.delivery_bps);
    const double mean = std::accumulate(values.begin(), values.end(), 0.0) /
                        values.size();
    double variance = 0.0;
    for (double value : values) variance += (value - mean) * (value - mean);
    variance /= std::max<size_t>(1, values.size());
    const double delivery_match = std::abs(delivery_component) /
                                  std::max(std::sqrt(2.0 * variance), 1.0);
    const std::complex<double> queue_component = DirectFundamental(
        samples, [](const AlignedSample& sample) { return sample.qdelay_s; },
        true);
    std::vector<double> q_values;
    for (const auto& sample : samples) {
      if (sample.q_valid) q_values.push_back(sample.qdelay_s);
    }
    const double queue_match = normalized_match(
        q_values, std::abs(queue_component));
    const double match = std::max(delivery_match, queue_match);
    if (match > best_match) {
      best_match = match;
      result.selected_delay_s = ratio * signature.rtprop_s;
      best_samples = std::move(samples);
    }
  }
  if (!best_samples.empty()) {
    const std::complex<double> input_component = DirectFundamental(
        best_samples,
        [](const AlignedSample& sample) {
          return (sample.sent_bps - sample.native_bps) /
                 std::max(sample.native_bps, 1.0);
        }, false);
    result.actual_input_amplitude_ratio = std::abs(input_component);
    result.commanded_amplitude_ratio = signature.amplitude_ratio;
    double input_energy = 0.0;
    for (const auto& sample : best_samples) {
      const double value = (sample.sent_bps - sample.native_bps) /
                           std::max(sample.native_bps, 1.0);
      input_energy += value * value;
    }
    result.actual_input_energy =
        std::sqrt(input_energy / std::max<size_t>(1, best_samples.size()));
    double input_residual_energy = 0.0;
    for (const auto& sample : best_samples) {
      const double input = (sample.sent_bps - sample.native_bps) /
                           std::max(sample.native_bps, 1.0);
      const double predicted = input_component.real() * sample.basis_sin +
                               input_component.imag() * sample.basis_cos;
      input_residual_energy += (input - predicted) * (input - predicted);
    }
    result.actual_input_snr = result.actual_input_amplitude_ratio /
        std::max(std::sqrt(input_residual_energy /
                           std::max<size_t>(1, best_samples.size())), 1e-9);

    double delivery_mean_aligned = 0.0;
    for (const auto& sample : best_samples) {
      delivery_mean_aligned += sample.delivery_bps;
    }
    delivery_mean_aligned /= best_samples.size();
    double input_squared = 0.0;
    double input_delivery = 0.0;
    for (const auto& sample : best_samples) {
      const double input_bps = sample.sent_bps - sample.native_bps;
      input_squared += input_bps * input_bps;
      input_delivery += input_bps *
                        (sample.delivery_bps - delivery_mean_aligned);
    }
    const double causal_gain = input_delivery /
                               std::max(input_squared, 1.0);
    std::vector<double> residual;
    residual.reserve(best_samples.size());
    for (const auto& sample : best_samples) {
      residual.push_back(sample.delivery_bps - delivery_mean_aligned -
          causal_gain * (sample.sent_bps - sample.native_bps));
    }
    double residual_neighbor = 0.0;
    for (double ratio : {1.50, 1.75}) {
      double sin_sum = 0.0;
      double cos_sum = 0.0;
      for (size_t i = 0; i < best_samples.size(); ++i) {
        const double theta = 2.0 * kPi * ratio * carrier_hz *
                             best_samples[i].output_time_s;
        sin_sum += residual[i] * std::sin(theta);
        cos_sum += residual[i] * std::cos(theta);
      }
      residual_neighbor = std::max(
          residual_neighbor,
          2.0 * std::hypot(sin_sum, cos_sum) / best_samples.size());
    }
    result.delivery_spectral_prominence = std::max(
        result.delivery_spectral_prominence,
        result.delivery_response_amplitude_bps /
            std::max(residual_neighbor, 1.0));
  }
  const double actual_input_bytes = result.actual_input_amplitude_ratio *
      native_median * signature.period_s / 8.0;
  result.actual_input_measurable =
      result.actual_input_amplitude_ratio >=
          std::max(config.actual_input_min_ratio,
                   config.carrier_detection_amplitude_min) &&
      actual_input_bytes >= 2.0 * kProbeMssBytes;

  const double response_bytes_min =
      config.delivery_trigger_min_response_mss * kProbeMssBytes;
  const bool response_bytes_pass =
      result.delivery_response_bytes >= response_bytes_min;
  result.weak_periodic_response =
      result.delivery_spectral_prominence >=
          config.delivery_trigger_prominence_start &&
      !response_bytes_pass && result.queue_derivative_amplitude <
          result.queue_noise_floor;
  const bool stationary = result.baseline_drift <= config.baseline_drift_max;
  const bool eligible = result.phase_coverage >=
                            config.trigger_phase_coverage_min &&
                        result.app_limited_fraction <=
                            config.max_app_limited_fraction &&
                        result.recovery_fraction <=
                            config.max_recovery_fraction &&
                        stationary;
  const double queue_coverage = times.empty()
      ? 0.0 : static_cast<double>(queue.size()) / times.size();
  const bool delivery_period_start = result.delivery_period_error_ratio <=
      config.trigger_period_tolerance_start;
  const bool delivery_period_continue = result.delivery_period_error_ratio <=
      config.trigger_period_tolerance_continue;
  const bool queue_period_start = result.queue_period_error_ratio <=
      config.trigger_period_tolerance_start;
  const bool queue_period_continue = result.queue_period_error_ratio <=
      config.trigger_period_tolerance_continue;
  result.delivery_trigger_pass = result.actual_input_measurable && eligible &&
      delivery_period_start && response_bytes_pass &&
      result.delivery_spectral_prominence >=
          config.delivery_trigger_prominence_start &&
      result.delivery_normalized_match >= config.delivery_trigger_match_start;
  result.delivery_continue_pass = result.actual_input_measurable && eligible &&
      delivery_period_continue &&
      result.delivery_spectral_prominence >=
          config.delivery_trigger_prominence_continue &&
      result.delivery_normalized_match >= config.delivery_trigger_match_continue;
  result.queue_trigger_pass = result.actual_input_measurable && eligible &&
      queue_coverage >= config.trigger_phase_coverage_min && queue_period_start &&
      result.queue_derivative_amplitude >= result.queue_noise_floor &&
      result.queue_spectral_prominence >= config.queue_trigger_prominence_start &&
      result.queue_normalized_match >= config.queue_trigger_match_start;
  result.queue_continue_pass = result.actual_input_measurable && eligible &&
      queue_coverage >= config.trigger_phase_coverage_min &&
      queue_period_continue &&
      result.queue_derivative_amplitude >= result.queue_noise_floor &&
      result.queue_spectral_prominence >=
          config.queue_trigger_prominence_continue &&
      result.queue_normalized_match >= config.queue_trigger_match_continue;

  auto branch_confidence = [](double prominence, double prominence_min,
                              double match, double match_min,
                              double period_error, double tolerance,
                              bool absolute_response) {
    if (!absolute_response) return 0.0;
    return Clamp01(std::pow(std::max(1e-9,
        Clamp01(prominence / std::max(prominence_min, 1e-9)) *
        Clamp01(match / std::max(match_min, 1e-9)) *
        Clamp01(1.0 - period_error / std::max(tolerance, 1e-9))),
        1.0 / 3.0));
  };
  const double delivery_confidence = branch_confidence(
      result.delivery_spectral_prominence,
      config.delivery_trigger_prominence_start,
      result.delivery_normalized_match, config.delivery_trigger_match_start,
      result.delivery_period_error_ratio,
      config.trigger_period_tolerance_start, response_bytes_pass);
  const double queue_confidence = branch_confidence(
      result.queue_spectral_prominence, config.queue_trigger_prominence_start,
      result.queue_normalized_match, config.queue_trigger_match_start,
      result.queue_period_error_ratio, config.trigger_period_tolerance_start,
      result.queue_derivative_amplitude >= result.queue_noise_floor);
  result.combined_confidence = 1.0 -
      (1.0 - delivery_confidence) * (1.0 - queue_confidence);

  const double q_high = config.q_reserve_high_bdp * signature.rtprop_s;
  const double q_peak_cap = config.q_peak_cap_bdp * signature.rtprop_s;
  const double loss_ratio = loss_sum / std::max<size_t>(1, times.size());
  const double ecn_ratio = ecn_sum / std::max<size_t>(1, times.size());
  const double queue_floor = Percentile(queue, 0.20);
  const double queue_peak = Percentile(queue, 0.95);
  const double queue_trend = queue.size() >= 4 ? RobustSlope(queue) : 0.0;
  result.hard_safety = loss_ratio >= config.hard_loss_threshold ||
      ecn_ratio >= config.hard_ecn_threshold ||
      (q_high > 0.0 && queue_floor >=
          config.queue_servo_hard_queue_multiple * q_high) ||
      (q_peak_cap > 0.0 && queue_peak >=
          config.queue_servo_hard_queue_multiple * q_peak_cap) ||
      (q_high > 0.0 && queue_trend > q_high / 8.0);

  if (result.delivery_trigger_pass && result.queue_trigger_pass) {
    result.combined_trigger_source = "BOTH";
  } else if (result.delivery_trigger_pass) {
    result.combined_trigger_source = "DELIVERY_ONLY";
  } else if (result.queue_trigger_pass) {
    result.combined_trigger_source = "QUEUE_ONLY";
  } else if (result.hard_safety) {
    result.combined_trigger_source = "HARD_SAFETY_ONLY";
  }
  result.trigger_pass = is_pulser && result.actual_input_measurable && eligible &&
      (result.delivery_trigger_pass || result.queue_trigger_pass);
  result.continue_pass = result.actual_input_measurable && eligible &&
      (result.delivery_continue_pass || result.queue_continue_pass);
  result.period_estimate_s = result.delivery_trigger_pass ||
          result.delivery_continue_pass
      ? result.delivery_period_estimate_s : result.queue_period_estimate_s;
  result.period_error_ratio = result.delivery_trigger_pass ||
          result.delivery_continue_pass
      ? result.delivery_period_error_ratio : result.queue_period_error_ratio;
  result.period_match = result.delivery_trigger_pass || result.queue_trigger_pass;
  result.spectral_prominence = std::max(
      result.delivery_spectral_prominence, result.queue_spectral_prominence);
  result.normalized_match = std::max(
      result.delivery_normalized_match, result.queue_normalized_match);
  result.detected_cycle_start_s = result.cycle_start_s;
  result.alignment_error_cycles = 0.0;

  result.delivery_reason = result.delivery_trigger_pass ? "pass" :
      (!response_bytes_pass ? "response_low" :
       !delivery_period_start ? "period_mismatch" :
       result.delivery_spectral_prominence <
               config.delivery_trigger_prominence_start ? "prominence_low" :
       result.delivery_normalized_match < config.delivery_trigger_match_start
           ? "match_low" : "ineligible");
  result.queue_reason = result.queue_trigger_pass ? "pass" :
      (queue_coverage < config.trigger_phase_coverage_min ? "rtt_coverage_low" :
       result.queue_derivative_amplitude < result.queue_noise_floor
           ? "response_below_noise" :
       !queue_period_start ? "period_mismatch" :
       result.queue_spectral_prominence < config.queue_trigger_prominence_start
           ? "prominence_low" :
       result.queue_normalized_match < config.queue_trigger_match_start
           ? "match_low" : "ineligible");

  if (result.trigger_pass) result.trigger_reason = "dual_channel_trigger_pass";
  else if (!is_pulser) result.trigger_reason = "watcher_observation";
  else if (!result.actual_input_measurable) result.trigger_reason = "actual_input_unmeasurable";
  else if (result.hard_safety) result.trigger_reason = "hard_safety_only";
  else if (!result.period_match) result.trigger_reason = "both_period_mismatch";
  else if (result.weak_periodic_response) result.trigger_reason = "both_channels_weak";
  else if (result.phase_coverage < config.trigger_phase_coverage_min)
    result.trigger_reason = "coverage_low";
  else if (result.app_limited_fraction > config.max_app_limited_fraction)
    result.trigger_reason = "app_limited";
  else if (result.recovery_fraction > config.max_recovery_fraction)
    result.trigger_reason = "recovery";
  else result.trigger_reason = "dual_channel_no_trigger";
  return result;
}

FbbrOperatingPointBlockResult FBBRFrequencySearch::AnalyzeBlock(
    const FBBRFrequencySearchConfig& config,
    const FbbrProbeSignature& signature,
    const std::vector<FbbrPhaseBinSample>& bins,
    int64_t first_output_bin,
    int64_t output_bin_count,
    uint64_t block_id,
    double previous_block_delay_s) {
  FbbrOperatingPointBlockResult result;
  result.cruise_id = signature.cruise_id;
  result.block_id = block_id;
  result.frequency_hz = signature.frequency_hz;
  result.period_rtts = signature.period_rtts;
  result.code_id = signature.code_id;
  result.initial_phase_rad = signature.initial_phase_rad;
  result.target_amplitude_ratio = signature.amplitude_ratio;
  result.rtprop_frozen_s = signature.rtprop_s;
  result.q_reserve_low_s = config.q_reserve_low_bdp * signature.rtprop_s;
  result.q_reserve_high_s = config.q_reserve_high_bdp * signature.rtprop_s;
  result.q_peak_cap_s = config.q_peak_cap_bdp * signature.rtprop_s;
  result.q_zero_s = std::max(config.q_zero_absolute_s,
                             config.q_zero_ratio * signature.rtprop_s);
  result.q_probe_max_s = result.q_peak_cap_s;
  const FbbrPhaseBinSample* first = FindBin(bins, first_output_bin);
  const FbbrPhaseBinSample* last =
      FindBin(bins, first_output_bin + output_bin_count - 1);
  if (first != nullptr) result.start_time_s = first->time_start_s;
  if (last != nullptr) result.end_time_s = last->time_end_s;
  if (signature.rtprop_s <= 0.0 || signature.period_s <= 0.0 ||
      output_bin_count <= 0) {
    result.invalid_reason = "signature_invalid";
    return result;
  }

  AnalysisAtDelay best;
  double best_ratio = config.delay_search_ratio_min;
  for (double ratio = config.delay_search_ratio_min;
       ratio <= config.delay_search_ratio_max + 1e-9;
       ratio += std::max(config.delay_search_ratio_step, 0.01)) {
    AnalysisAtDelay current = AnalyzeDelay(
        config, signature, bins, first_output_bin, output_bin_count,
        ratio * signature.rtprop_s);
    if (current.score > best.score) {
      best = current;
      best_ratio = ratio;
    }
  }
  result.selected_delay_s = best_ratio * signature.rtprop_s;
  result.delay_ratio = best_ratio;
  result.delay_at_search_boundary =
      std::abs(best_ratio - config.delay_search_ratio_min) < 1e-6 ||
      std::abs(best_ratio - config.delay_search_ratio_max) < 1e-6;
  result.cross_block_delay_stable =
      !std::isfinite(previous_block_delay_s) || previous_block_delay_s <= 0.0 ||
      std::abs(result.selected_delay_s - previous_block_delay_s) <=
          config.max_cross_block_delay_shift_ratio * signature.rtprop_s;
  result.input_fit = best.input_fit;
  result.delivery_fit = best.delivery_fit;
  result.queue_fit = best.queue_fit;
  result.utility_fit = best.utility_fit;

  if (best.samples.empty() || best.d0 <= 0.0) {
    result.invalid_reason = "aligned_samples_insufficient";
    return result;
  }

  std::vector<double> native_values;
  std::vector<double> delivery_values;
  std::vector<double> q_values;
  std::vector<double> servo_factors;
  std::map<int64_t, uint32_t> bins_per_cycle;
  std::map<int64_t, uint32_t> valid_bins_per_cycle;
  std::map<int64_t, bool> servo_transition_by_cycle;
  double coverage_sum = 0.0;
  double non_app_sum = 0.0;
  double cwnd_limited_sum = 0.0;
  double recovery_sum = 0.0;
  double loss_bytes_ratio_sum = 0.0;
  double ecn_bytes_ratio_sum = 0.0;
  bool phase_transition = false;
  for (const auto& sample : best.samples) {
    native_values.push_back(sample.native_bps /
                            std::max(sample.queue_servo_factor, 1e-9));
    delivery_values.push_back(sample.delivery_bps);
    if (sample.q_valid) q_values.push_back(sample.qdelay_s);
    servo_factors.push_back(sample.queue_servo_factor);
    servo_transition_by_cycle[sample.input_cycle] =
        servo_transition_by_cycle[sample.input_cycle] ||
        sample.queue_servo_transition;
    ++bins_per_cycle[sample.input_cycle];
    if (sample.coverage >= config.min_bin_coverage) {
      ++valid_bins_per_cycle[sample.input_cycle];
    }
    coverage_sum += sample.coverage;
    non_app_sum += 1.0 - Clamp01(sample.app_limited_fraction);
    cwnd_limited_sum += Clamp01(sample.cwnd_limited_fraction);
    recovery_sum += Clamp01(sample.recovery_fraction);
    loss_bytes_ratio_sum += sample.loss_ratio;
    ecn_bytes_ratio_sum += sample.ecn_ratio;
    phase_transition = phase_transition || sample.phase_transition;
  }
  result.phase_bin_coverage = coverage_sum / best.samples.size();
  result.non_app_limited_fraction = non_app_sum / best.samples.size();
  result.cwnd_limited_fraction = cwnd_limited_sum / best.samples.size();
  result.recovery_fraction = recovery_sum / best.samples.size();
  result.loss_ratio = loss_bytes_ratio_sum / best.samples.size();
  result.ecn_ratio = ecn_bytes_ratio_sum / best.samples.size();
  for (const auto& entry : bins_per_cycle) {
    const uint32_t valid = valid_bins_per_cycle[entry.first];
    if (!servo_transition_by_cycle[entry.first] && entry.second > 0 &&
        static_cast<double>(valid) / entry.second >= 0.75) {
      ++result.valid_cycles;
    }
  }
  const double native_median = Median(native_values);
  result.native_baseline_drift = native_median <= 0.0
      ? std::numeric_limits<double>::infinity()
      : (Percentile(native_values, 0.95) - Percentile(native_values, 0.05)) /
            native_median;
  const size_t half = delivery_values.size() / 2;
  std::vector<double> delivery_first(delivery_values.begin(),
                                     delivery_values.begin() + half);
  std::vector<double> delivery_second(delivery_values.begin() + half,
                                      delivery_values.end());
  result.delivery_baseline_drift =
      std::abs(Median(delivery_first) - Median(delivery_second)) /
      std::max(best.d0, 1.0);
  result.q95_s = Percentile(q_values, 0.95);
  result.q_amplitude_s = Percentile(q_values, 0.95) - Percentile(q_values, 0.05);
  result.delivery_median_bps = Median(delivery_values);
  result.queue_servo_factor_mean = servo_factors.empty()
      ? 1.0 : std::accumulate(servo_factors.begin(), servo_factors.end(), 0.0) /
          servo_factors.size();
  result.queue_servo_transition_cycles = static_cast<uint32_t>(std::count_if(
      servo_transition_by_cycle.begin(), servo_transition_by_cycle.end(),
      [](const std::pair<const int64_t, bool>& entry) { return entry.second; }));

  const std::complex<double> input_a1 = DirectFundamental(
      best.samples,
      [](const AlignedSample& sample) {
        return (sample.sent_bps - sample.native_bps) /
               std::max(sample.native_bps, 1.0);
      },
      false);
  const std::complex<double> commanded_a1 = DirectFundamental(
      best.samples,
      [](const AlignedSample& sample) {
        return (sample.commanded_bps - sample.native_bps) /
               std::max(sample.native_bps, 1.0);
      },
      false);
  const double actual_fundamental_amplitude = std::abs(input_a1);
  result.actual_input_amplitude_ratio = signature.waveform == "sine"
      ? actual_fundamental_amplitude
      : std::max(actual_fundamental_amplitude,
                 std::abs(result.input_fit.beta[6]));
  result.realized_amplitude_ratio = actual_fundamental_amplitude /
      std::max(std::abs(commanded_a1), 1e-9);
  result.input_carrier_snr = actual_fundamental_amplitude /
      std::max(std::sqrt(std::max(0.0, result.input_fit.residual_variance)),
               1e-9);
  result.input_fit.snr = result.input_carrier_snr;
  // Signature isolation is an output-side test.  Packetization noise in this
  // flow's actual input is already covered by input SNR/coherence and must not
  // be mislabeled as a multi-flow signature collision.
  const std::complex<double> delivery_a1 = DirectFundamental(
      best.samples,
      [&best](const AlignedSample& sample) {
        return sample.delivery_bps / std::max(best.d0, 1.0) - 1.0;
      },
      false);
  {
    double delivery_variance = 0.0;
    double weight_sum = 0.0;
    for (const auto& sample : best.samples) {
      const double value = sample.delivery_bps / std::max(best.d0, 1.0) - 1.0;
      delivery_variance += sample.weight * value * value;
      weight_sum += sample.weight;
    }
    result.delivery_normalized_match = Clamp01(
        std::abs(delivery_a1) /
        std::max(std::sqrt(2.0 * delivery_variance /
                           std::max(weight_sum, 1e-9)), 1e-9));
    auto amplitude_at = [&best](double frequency_hz) {
      double mean = 0.0;
      double weight = 0.0;
      for (const auto& sample : best.samples) {
        mean += sample.weight * sample.delivery_bps;
        weight += sample.weight;
      }
      mean /= std::max(weight, 1e-9);
      double sin_sum = 0.0;
      double cos_sum = 0.0;
      for (const auto& sample : best.samples) {
        const double theta = 2.0 * kPi * frequency_hz * sample.output_time_s;
        sin_sum += sample.weight * (sample.delivery_bps - mean) *
                   std::sin(theta);
        cos_sum += sample.weight * (sample.delivery_bps - mean) *
                   std::cos(theta);
      }
      return 2.0 * std::hypot(sin_sum, cos_sum) /
             std::max(weight, 1e-9);
    };
    double neighbor = 0.0;
    for (double ratio : {1.50, 1.75}) {
      neighbor = std::max(neighbor,
          amplitude_at(ratio * signature.frequency_hz));
    }
    result.delivery_spectral_prominence =
        amplitude_at(signature.frequency_hz) / std::max(neighbor, 1.0);
  }
  const std::complex<double> queue_a1 = DirectFundamental(
      best.samples,
      [](const AlignedSample& sample) { return sample.qdelay_s; },
      true);
  {
    double queue_mean = 0.0;
    double queue_weight = 0.0;
    for (const auto& sample : best.samples) {
      if (!sample.q_valid || sample.weight <= 0.0) continue;
      queue_mean += sample.weight * sample.qdelay_s;
      queue_weight += sample.weight;
    }
    queue_mean /= std::max(queue_weight, 1e-9);
    double queue_variance = 0.0;
    for (const auto& sample : best.samples) {
      if (!sample.q_valid || sample.weight <= 0.0) continue;
      const double residual = sample.qdelay_s - queue_mean;
      queue_variance += sample.weight * residual * residual;
    }
    result.queue_normalized_match = Clamp01(
        std::abs(queue_a1) /
        std::max(std::sqrt(2.0 * queue_variance /
                           std::max(queue_weight, 1e-9)), 1e-12));
    auto queue_derivative_amplitude_at = [&best](double frequency_hz) {
      double mean = 0.0;
      double weight = 0.0;
      for (const auto& sample : best.samples) {
        if (!sample.q_valid || sample.weight <= 0.0) continue;
        mean += sample.weight * sample.qdelay_s;
        weight += sample.weight;
      }
      mean /= std::max(weight, 1e-9);
      double sin_sum = 0.0;
      double cos_sum = 0.0;
      for (const auto& sample : best.samples) {
        if (!sample.q_valid || sample.weight <= 0.0) continue;
        const double theta = 2.0 * kPi * frequency_hz * sample.output_time_s;
        sin_sum += sample.weight * (sample.qdelay_s - mean) * std::sin(theta);
        cos_sum += sample.weight * (sample.qdelay_s - mean) * std::cos(theta);
      }
      const double queue_amplitude = 2.0 * std::hypot(sin_sum, cos_sum) /
                                     std::max(weight, 1e-9);
      return 2.0 * kPi * frequency_hz * queue_amplitude;
    };
    const double target = queue_derivative_amplitude_at(signature.frequency_hz);
    double neighbor = 0.0;
    for (double ratio : {1.50, 1.75}) {
      neighbor = std::max(neighbor, queue_derivative_amplitude_at(
          ratio * signature.frequency_hz));
    }
    result.queue_spectral_prominence = target / std::max(neighbor, 1e-9);
  }
  const std::complex<double> utility_a1 = DirectFundamental(
      best.samples,
      [&config, &signature, &best](const AlignedSample& sample) {
        return std::log(std::max(sample.delivery_bps, 1.0) /
                        std::max(best.d0, 1.0)) -
               config.utility_delay_weight *
                   std::log((signature.rtprop_s + sample.qdelay_s) /
                            std::max(signature.rtprop_s, 1e-9));
      },
      true);
  const double own_delivery_amplitude = std::abs(delivery_a1);
  double maximum_cross_amplitude = 0.0;
  for (uint32_t other_code = 0; other_code < 3; ++other_code) {
    if (other_code == signature.code_id % 3u) continue;
    double weighted_sin = 0.0;
    double weighted_cos = 0.0;
    double weight_sum = 0.0;
    for (const AlignedSample& sample : best.samples) {
      const double yd = sample.delivery_bps / std::max(best.d0, 1.0) - 1.0;
      const int sign = FBBRFrequencySearch::WalshSign(
          other_code, sample.input_cycle);
      weighted_sin += sample.weight * yd * sign * std::sin(sample.theta);
      weighted_cos += sample.weight * yd * sign * std::cos(sample.theta);
      weight_sum += sample.weight;
    }
    const double amplitude = 2.0 * std::hypot(weighted_sin, weighted_cos) /
                             std::max(weight_sum, 1e-9);
    maximum_cross_amplitude = std::max(maximum_cross_amplitude, amplitude);
  }
  result.signature_leakage = maximum_cross_amplitude /
      std::max(own_delivery_amplitude, 1e-9);
  result.residual_to_own_carrier_ratio = std::sqrt(std::max(
      0.0, result.delivery_fit.residual_variance)) /
      std::max(own_delivery_amplitude, 1e-9);
  result.collision_suspected =
      result.queue_servo_transition_cycles == 0 &&
      std::abs(result.queue_servo_factor_mean - 1.0) <= 0.05 &&
      result.actual_input_amplitude_ratio >= config.actual_input_min_ratio &&
      result.input_carrier_snr >= 0.75 * config.input_snr_min &&
      own_delivery_amplitude >=
          0.10 * result.actual_input_amplitude_ratio &&
      result.signature_leakage > config.max_signature_leakage &&
      result.residual_to_own_carrier_ratio >
          config.max_residual_to_carrier_ratio;
  const std::complex<double> delivery_response =
      std::abs(input_a1) > kEpsilon
          ? delivery_a1 / input_a1
          : std::complex<double>(0.0, 0.0);
  const std::complex<double> queue_response =
      std::abs(input_a1) > kEpsilon
          ? queue_a1 / input_a1
          : std::complex<double>(0.0, 0.0);
  const std::complex<double> utility_response =
      std::abs(input_a1) > kEpsilon
          ? utility_a1 / input_a1
          : std::complex<double>(0.0, 0.0);
  const double omega = 2.0 * kPi * signature.frequency_hz;
  result.response.delivery_gain = std::abs(delivery_response);
  result.response.queue_storage_gain = omega * std::abs(queue_response);
  result.response.delivery_phase_rad = ComplexPhase(delivery_response);
  result.response.queue_phase_rad = ComplexPhase(queue_response);
  result.response.utility_phase_rad = ComplexPhase(utility_response);
  result.response.delivery_harmonic_ratio =
      std::abs(SecondHarmonic(result.delivery_fit)) /
      std::max(std::abs(Fundamental(result.delivery_fit)), 1e-9);
  result.response.queue_harmonic_ratio =
      std::abs(SecondHarmonic(result.queue_fit)) /
      std::max(std::abs(Fundamental(result.queue_fit)), 1e-9);

  result.input_cycle_coherence = PerCycleInputCoherence(best.samples);
  const double delivery_coherence =
      PerCycleCoherence(best.samples, false, false, best.d0,
                        signature.rtprop_s, config.utility_delay_weight);
  const double queue_coherence =
      PerCycleCoherence(best.samples, true, false, best.d0,
                        signature.rtprop_s, config.utility_delay_weight);
  const double utility_coherence =
      PerCycleCoherence(best.samples, false, true, best.d0,
                        signature.rtprop_s, config.utility_delay_weight);
  result.confidence_cycle =
      std::max(delivery_coherence, std::max(queue_coherence, utility_coherence));

  const auto sent_getter = [](const AlignedSample& sample) { return sample.sent_bps; };
  const auto delivery_getter = [](const AlignedSample& sample) {
    return sample.delivery_bps;
  };
  const auto q_getter = [](const AlignedSample& sample) { return sample.qdelay_s; };
  const double sent_plus = RegionMedian(best.samples, 1, sent_getter, false);
  const double sent_zero = RegionMedian(best.samples, 0, sent_getter, false);
  const double delivery_plus =
      RegionMedian(best.samples, 1, delivery_getter, false);
  const double delivery_zero =
      RegionMedian(best.samples, 0, delivery_getter, false);
  const double q_plus = RegionMedian(best.samples, 1, q_getter, true);
  const double q_zero_region = RegionMedian(best.samples, 0, q_getter, true);
  result.positive_delivery_gain = Clamp(
      (delivery_plus - delivery_zero) /
          std::max(sent_plus - sent_zero, 1.0),
      0.0, 1.5);
  result.positive_queue_build_s = q_plus - q_zero_region;
  const double phase_domain_queue_build_gain =
      omega * result.positive_queue_build_s /
      std::max(result.actual_input_amplitude_ratio, 1e-6);
  // An integrating queue can have equal medians in P+ and P0 even though it
  // builds throughout P+ (the early/late samples cancel).  Retain the direct
  // phase-domain delta in positive_queue_build_s, and use the larger of that
  // directional estimate and the independently fitted storage response as the
  // dimensionless positive-build evidence.
  result.positive_queue_build_gain = std::max(
      phase_domain_queue_build_gain, result.response.queue_storage_gain);

  std::map<int64_t, std::vector<const AlignedSample*>> cycles;
  for (const auto& sample : best.samples) cycles[sample.input_cycle].push_back(&sample);
  std::vector<double> cycle_floors;
  std::vector<double> cycle_means;
  uint32_t drained_cycles = 0;
  for (const auto& entry : cycles) {
    std::vector<double> cycle_q;
    std::vector<double> negative_q;
    std::vector<const AlignedSample*> ordered = entry.second;
    std::sort(ordered.begin(), ordered.end(),
              [](const AlignedSample* lhs, const AlignedSample* rhs) {
                return lhs->input_time_s < rhs->input_time_s;
              });
    for (const AlignedSample* sample : ordered) {
      if (!sample->q_valid) continue;
      cycle_q.push_back(sample->qdelay_s);
      if (sample->z < 0.0) negative_q.push_back(sample->qdelay_s);
    }
    if (cycle_q.empty()) continue;
    const size_t late_negative_begin = negative_q.size() / 2;
    std::vector<double> late_negative(
        negative_q.begin() + late_negative_begin, negative_q.end());
    cycle_floors.push_back(Percentile(
        late_negative.empty() ? cycle_q : late_negative, 0.10));
    cycle_means.push_back(std::accumulate(cycle_q.begin(), cycle_q.end(), 0.0) /
                          cycle_q.size());
    const double end_q = ordered.empty() || !ordered.back()->q_valid
                             ? std::numeric_limits<double>::infinity()
                             : ordered.back()->qdelay_s;
    if (!negative_q.empty() &&
        *std::min_element(negative_q.begin(), negative_q.end()) <=
            result.q_reserve_high_s &&
        end_q <= result.q_peak_cap_s) {
      ++drained_cycles;
    }
  }
  result.q_floor_s = Median(cycle_floors);
  double effective_cycle_weight = 0.0;
  if (!cycles.empty()) {
    const int64_t newest_cycle = cycles.rbegin()->first;
    for (const auto& entry : cycles) {
      effective_cycle_weight += std::pow(
          Clamp(config.forgetting_factor, 0.50, 1.0),
          static_cast<double>(newest_cycle - entry.first));
    }
  }
  result.effective_cycles = effective_cycle_weight;
  const size_t observed_cycle_count = cycles.size();
  result.drain_ratio = cycles.empty()
      ? 0.0
      : static_cast<double>(drained_cycles) / cycles.size();
  const double queue_slope = RobustSlope(cycle_means);
  result.queue_trend_per_cycle =
      queue_slope / std::max(signature.rtprop_s, 1e-9);

  result.gradient_lockin = utility_response.real();
  const auto utility_value = [&config, &signature, &best](
                                 const AlignedSample& sample) {
    return std::log(std::max(sample.delivery_bps, 1.0) /
                    std::max(best.d0, 1.0)) -
           config.utility_delay_weight *
               std::log((signature.rtprop_s + sample.qdelay_s) /
                        std::max(signature.rtprop_s, 1e-9));
  };
  const double j_plus = RegionMedian(best.samples, 1, utility_value, true);
  const double j_zero = RegionMedian(best.samples, 0, utility_value, true);
  const double j_minus = RegionMedian(best.samples, -1, utility_value, true);
  const auto x_getter = [](const AlignedSample& sample) {
    return (sample.sent_bps - sample.native_bps) /
           std::max(sample.native_bps, 1.0);
  };
  const double a_plus = RegionMedian(best.samples, 1, x_getter, false);
  const double a_minus = std::abs(RegionMedian(best.samples, -1, x_getter, false));
  result.gradient_finite_difference =
      (j_plus - j_minus) / std::max(a_plus + a_minus, 1e-6);
  const double a_average = 0.5 * (a_plus + a_minus);
  result.curvature_finite_difference =
      (j_plus - 2.0 * j_zero + j_minus) /
      std::max(a_average * a_average, 1e-8);
  result.gradient_agreement =
      result.gradient_lockin * result.gradient_finite_difference >= 0.0 ||
      (std::abs(result.gradient_lockin) <= config.gradient_zero_hard &&
       std::abs(result.gradient_finite_difference) <= config.gradient_zero_hard);
  result.gradient_fused = result.gradient_agreement
      ? 0.5 * (result.gradient_lockin + result.gradient_finite_difference)
      : (std::abs(result.gradient_lockin) <
                 std::abs(result.gradient_finite_difference)
             ? result.gradient_lockin
             : result.gradient_finite_difference);

  std::vector<double> cycle_gradients;
  for (const auto& entry : cycles) {
    std::vector<AlignedSample> cycle_samples;
    cycle_samples.reserve(entry.second.size());
    for (const AlignedSample* sample : entry.second) {
      cycle_samples.push_back(*sample);
    }
    const std::complex<double> cycle_input = DirectFundamental(
        cycle_samples,
        [](const AlignedSample& sample) {
          return (sample.sent_bps - sample.native_bps) /
                 std::max(sample.native_bps, 1.0);
        },
        false);
    const std::complex<double> cycle_utility = DirectFundamental(
        cycle_samples,
        [&config, &signature, &best](const AlignedSample& sample) {
          return std::log(std::max(sample.delivery_bps, 1.0) /
                          std::max(best.d0, 1.0)) -
                 config.utility_delay_weight *
                     std::log((signature.rtprop_s + sample.qdelay_s) /
                              std::max(signature.rtprop_s, 1e-9));
        },
        true);
    if (std::abs(cycle_input) > 1e-6) {
      const double gradient = (cycle_utility / cycle_input).real();
      if (std::isfinite(gradient)) cycle_gradients.push_back(gradient);
    }
  }
  if (cycle_gradients.size() >= 2) {
    const double center = Median(cycle_gradients);
    double squared_error = 0.0;
    for (double gradient : cycle_gradients) {
      squared_error += (gradient - center) * (gradient - center);
    }
    result.gradient_se = std::sqrt(
        squared_error /
        (cycle_gradients.size() * (cycle_gradients.size() - 1.0)));
  } else {
    const double utility_fundamental_se = std::hypot(
        result.utility_fit.standard_error[2],
        result.utility_fit.standard_error[3]);
    result.gradient_se = utility_fundamental_se /
        std::max(std::abs(input_a1), 1e-9);
  }
  result.gradient_ci90_low = result.gradient_fused - 1.645 * result.gradient_se;
  result.gradient_ci90_high = result.gradient_fused + 1.645 * result.gradient_se;
  result.gradient_ci95_low = result.gradient_fused - 1.960 * result.gradient_se;
  result.gradient_ci95_high = result.gradient_fused + 1.960 * result.gradient_se;
  result.gradient_equivalent =
      result.gradient_ci95_low >= -config.gradient_zero_soft &&
      result.gradient_ci95_high <= config.gradient_zero_soft;

  result.confidence_coverage = Clamp01(
      result.phase_bin_coverage / std::max(config.phase_coverage_min, 1e-6));
  // Command-to-emission gain is diagnostic.  Confidence follows the energy
  // and separation of the actual emitted carrier, even when the pacer has
  // attenuated it substantially.
  result.confidence_input = std::pow(std::max(0.0,
      Clamp01(result.input_carrier_snr / 3.0) *
      Clamp01(result.input_fit.r_squared) *
      Clamp01(result.input_cycle_coherence)), 1.0 / 3.0);
  result.confidence_stationarity = std::sqrt(
      std::exp(-std::pow(result.native_baseline_drift /
                             std::max(config.max_native_baseline_drift, 1e-6),
                         2.0)) *
      std::exp(-std::pow(result.delivery_baseline_drift /
                             std::max(config.max_native_baseline_drift, 1e-6),
                         2.0)));
  result.confidence_delivery_channel = Clamp01(std::pow(std::max(1e-9,
      Clamp01(result.delivery_spectral_prominence /
              std::max(config.delivery_trigger_prominence_continue, 1e-6)) *
      Clamp01(result.delivery_normalized_match /
              std::max(config.delivery_trigger_match_continue, 1e-6)) *
      Clamp01(result.delivery_fit.snr /
              std::max(config.min_detectable_snr, 1e-6))), 1.0 / 3.0));
  result.confidence_queue_channel = Clamp01(std::pow(std::max(1e-9,
      Clamp01(result.queue_spectral_prominence /
              std::max(config.queue_trigger_prominence_continue, 1e-6)) *
      Clamp01(result.queue_normalized_match /
              std::max(config.queue_trigger_match_continue, 1e-6)) *
      Clamp01(result.queue_fit.snr /
              std::max(config.min_detectable_snr, 1e-6))), 1.0 / 3.0));
  result.confidence_response = 1.0 -
      (1.0 - result.confidence_delivery_channel) *
      (1.0 - result.confidence_queue_channel);
  result.confidence_delay =
      (result.delay_at_search_boundary ? 0.65 : 1.0) *
      (result.cross_block_delay_stable ? 1.0 : 0.4);
  double maximum_condition = result.input_fit.condition_number;
  if (result.delivery_fit.valid) maximum_condition =
      std::max(maximum_condition, result.delivery_fit.condition_number);
  if (result.queue_fit.valid) maximum_condition =
      std::max(maximum_condition, result.queue_fit.condition_number);
  if (result.utility_fit.valid) maximum_condition =
      std::max(maximum_condition, result.utility_fit.condition_number);
  result.confidence_regression = Clamp01(
      1.0 - std::log10(std::max(1.0, maximum_condition)) /
                std::log10(std::max(10.0, config.max_condition_number)));

  std::string hard_failure = "none";
  const uint32_t required_valid_cycles =
      config.search_controller_enabled
          ? config.min_valid_analysis_cycles
          : config.min_valid_cycles;
  if (result.valid_cycles < required_valid_cycles)
    hard_failure = AppendReason(hard_failure, "valid_cycles_low");
  if (result.phase_bin_coverage < config.phase_coverage_min)
    hard_failure = AppendReason(hard_failure, "coverage_low");
  if (1.0 - result.non_app_limited_fraction > config.max_app_limited_fraction)
    hard_failure = AppendReason(hard_failure, "app_limited");
  if (result.recovery_fraction > config.max_recovery_fraction)
    hard_failure = AppendReason(hard_failure, "recovery_fraction_high");
  const double packetization_min_ratio =
      16.0 * kProbeMssBytes /
      std::max(native_median * signature.period_s, 1.0);
  if (result.actual_input_amplitude_ratio <
      std::max(config.actual_input_min_ratio, packetization_min_ratio))
    hard_failure = AppendReason(hard_failure, "actual_input_energy_low");
  if (result.collision_suspected)
    hard_failure = AppendReason(hard_failure, "signature_collision");
  if (!result.input_fit.valid ||
      (!result.delivery_fit.valid && !result.queue_fit.valid))
    hard_failure = AppendReason(hard_failure, "regression_invalid");
  if (maximum_condition > config.regression_condition_max)
    hard_failure = AppendReason(hard_failure, "regression_condition_high");
  if (result.native_baseline_drift > config.baseline_drift_max)
    hard_failure = AppendReason(hard_failure, "baseline_changed_inside_window");
  if (phase_transition)
    hard_failure = AppendReason(hard_failure, "phase_transition");

  if (hard_failure != "none") {
    result.invalid_reason = hard_failure;
    result.measurement_confidence = 0.0;
    result.classification = FbbrOperatingPointClassification::kInvalid;
    FillCandidate(config, best.samples, best.d0, &result);
    return result;
  }

  const double cwnd_confidence = Clamp01(
      1.0 - 0.5 * result.cwnd_limited_fraction);
  const std::array<std::pair<double, double>, 7> confidence_terms{{
      {result.confidence_input, 0.18},
      {result.confidence_response, 0.24},
      {result.confidence_coverage, 0.14},
      {result.confidence_delay, 0.10},
      {result.confidence_regression, 0.16},
      {result.confidence_stationarity, 0.10},
      {std::sqrt(std::max(0.0, result.confidence_cycle * cwnd_confidence)), 0.08},
  }};
  double confidence_log_sum = 0.0;
  double confidence_weight_sum = 0.0;
  for (const auto& term : confidence_terms) {
    confidence_log_sum += term.second *
                          std::log(std::max(term.first, 1e-6));
    confidence_weight_sum += term.second;
  }
  result.measurement_confidence = Clamp01(std::exp(
      confidence_log_sum / std::max(confidence_weight_sum, 1e-9)));

  const auto l_hi = [](double value, double threshold, double width) {
    return Logistic((value - threshold) / std::max(width, 1e-9));
  };
  const double clipping = Clamp(1.0 - result.positive_delivery_gain,
                                -1.0, 1.0);
  const double p_clip = l_hi(clipping, 0.20, 0.08);
  const double p_qbuild = l_hi(result.response.queue_storage_gain, 0.10, 0.05);
  const double channel_gain_sum = result.response.delivery_gain +
                                  result.response.queue_storage_gain;
  result.delivery_partition = channel_gain_sum > 1e-9
      ? result.response.delivery_gain / channel_gain_sum : 0.5;
  result.queue_partition = channel_gain_sum > 1e-9
      ? result.response.queue_storage_gain / channel_gain_sum : 0.5;
  result.saturation_score = 1.0 - (1.0 - p_clip) * (1.0 - p_qbuild);
  result.full_score = result.saturation_score;

  const double sigma_low = std::max(result.q_reserve_low_s / 2.0, 1e-9);
  const double sigma_high = std::max(result.q_reserve_high_s / 2.0, 1e-9);
  if (result.q_floor_s < result.q_reserve_low_s) {
    result.queue_band_score = std::exp(-std::pow(
        (result.q_reserve_low_s - result.q_floor_s) / sigma_low, 2.0));
    result.queue_band_error =
        (result.q_reserve_low_s - result.q_floor_s) /
        std::max(result.q_reserve_low_s, 1e-9);
  } else if (result.q_floor_s <= result.q_reserve_high_s) {
    result.queue_band_score = 1.0;
    result.queue_band_error = 0.0;
  } else {
    result.queue_band_score = std::exp(-std::pow(
        (result.q_floor_s - result.q_reserve_high_s) / sigma_high, 2.0));
    result.queue_band_error =
        -(result.q_floor_s - result.q_reserve_high_s) /
        std::max(result.q_reserve_high_s, 1e-9);
  }
  result.low_queue_score = result.queue_band_score;

  const double trend_scale = std::max(
      config.max_queue_trend_per_cycle, 0.01);
  const double trend_score = std::exp(-std::pow(
      std::abs(result.queue_trend_per_cycle) / trend_scale, 2.0));
  const double peak_excess = std::max(
      0.0, result.q95_s - result.q_peak_cap_s);
  const double peak_score = std::exp(-std::pow(
      peak_excess / std::max(result.q_peak_cap_s / 2.0, 1e-9), 2.0));
  const double growth_score = result.queue_trend_per_cycle <= 0.0
      ? 1.0
      : std::exp(-std::pow(
            result.queue_trend_per_cycle / trend_scale, 2.0));
  const double servo_unity_score = std::exp(-std::pow(
      std::abs(result.queue_servo_factor_mean - 1.0) / 0.05, 2.0));
  result.queue_stability_score = std::pow(std::max(
      trend_score * peak_score * growth_score * servo_unity_score, 1e-9),
      1.0 / 4.0);
  result.stationary_score = result.queue_stability_score;
  result.safe_score = (result.loss_ratio < config.soft_loss_threshold &&
                       result.ecn_ratio < config.soft_ecn_threshold)
      ? 1.0 : 0.0;
  constexpr double kScoreEpsilon = 1e-6;
  result.raw_optimality_score = std::exp(
      0.40 * std::log(result.saturation_score + kScoreEpsilon) +
      0.35 * std::log(result.queue_band_score + kScoreEpsilon) +
      0.25 * std::log(result.queue_stability_score + kScoreEpsilon));
  result.target_score = result.measurement_confidence *
                        result.raw_optimality_score;
  result.optimality_score = result.target_score;

  double normalized_gradient = 0.0;
  if (result.gradient_ci90_low > 0.0) {
    normalized_gradient = std::tanh(
        result.gradient_ci90_low /
        std::max(config.utility_gradient_scale, 1e-9));
  } else if (result.gradient_ci90_high < 0.0) {
    normalized_gradient = std::tanh(
        result.gradient_ci90_high /
        std::max(config.utility_gradient_scale, 1e-9));
  }
  const double gradient_confidence = Clamp01(
      std::abs(result.gradient_fused) /
      std::max(std::abs(result.gradient_fused) + result.gradient_se, 1e-9));
  const double effective_gradient_weight =
      config.utility_gradient_weight * gradient_confidence;
  const double channel_direction = 2.0 * result.delivery_partition - 1.0;
  result.frequency_direction = Clamp(
      config.channel_split_weight * channel_direction +
      effective_gradient_weight * normalized_gradient,
      -1.0, 1.0);
  const double normalized_trend = Clamp(
      result.queue_trend_per_cycle / trend_scale, -1.0, 1.0);
  result.total_direction = Clamp(
      config.slow_frequency_weight * result.frequency_direction +
      config.slow_queue_weight * result.queue_band_error -
      config.slow_trend_weight * normalized_trend,
      -1.0, 1.0);
  result.direction_score = result.total_direction;
  result.rate_adjustment_signal = result.total_direction;

  result.underload_evidence = Clamp01(std::max(
      l_hi(result.total_direction, 0.20, 0.10),
      l_hi(result.queue_band_error, 0.25, 0.15) *
          l_hi(result.response.delivery_gain, 0.65, 0.10)));
  result.overload_evidence = Clamp01(std::max(
      l_hi(-result.total_direction, 0.20, 0.10),
      l_hi(-result.queue_band_error, 0.20, 0.12)));

  const bool strong_gradient_disagreement =
      !result.gradient_agreement &&
      std::abs(result.gradient_lockin) > config.gradient_zero_soft &&
      std::abs(result.gradient_finite_difference) > config.gradient_zero_soft;
  const bool strong_state_gradient_conflict =
      std::abs(normalized_gradient) >= 0.25 &&
      std::abs(result.queue_band_error) >= 0.40 &&
      normalized_gradient * result.queue_band_error < 0.0;
  const bool ambiguous =
      (result.underload_evidence >= 0.65 &&
       result.overload_evidence >= 0.65) ||
      (strong_gradient_disagreement &&
       std::abs(result.gradient_fused) > config.gradient_zero_soft) ||
      strong_state_gradient_conflict;
  if (ambiguous) {
    result.total_direction = 0.0;
    result.direction_score = 0.0;
    result.rate_adjustment_signal = 0.0;
  }
  const bool hard_queue_growth =
      result.q_floor_s >= 3.0 * result.q_reserve_high_s &&
      result.queue_trend_per_cycle > 2.0 * trend_scale;
  const bool buffer_saturated =
      result.loss_ratio >= config.hard_loss_threshold ||
      result.ecn_ratio >= config.hard_ecn_threshold ||
      hard_queue_growth;
  const bool overload = result.overload_evidence >= 0.65 ||
      result.q_floor_s > result.q_reserve_high_s ||
      result.q95_s > result.q_peak_cap_s;
  const bool decisive_queue_overload =
      result.q_floor_s > result.q_reserve_high_s &&
      result.q95_s > result.q_peak_cap_s &&
      result.loss_ratio < config.hard_loss_threshold &&
      result.ecn_ratio < config.hard_ecn_threshold;
  const bool underload =
      result.underload_evidence >= 0.65 &&
      result.overload_evidence < 0.45 &&
      result.direction_score >= 0.20 &&
      result.q_floor_s <= result.q_reserve_high_s &&
      result.loss_ratio < config.soft_loss_threshold;
  result.lockable_score = observed_cycle_count >= config.min_score_cycles;
  const bool near_optimal =
      result.lockable_score &&
      result.measurement_confidence >=
          config.measurement_confidence_track_min &&
      result.target_score >= config.near_optimal_score_threshold &&
      result.gradient_equivalent &&
      std::abs(result.direction_score) <=
          config.near_optimal_direction_abs_max &&
      result.q_floor_s >= result.q_reserve_low_s &&
      result.q_floor_s <= result.q_reserve_high_s &&
      result.q95_s <= result.q_peak_cap_s &&
      std::abs(result.queue_servo_factor_mean - 1.0) <= 0.02 &&
      result.queue_servo_transition_cycles == 0 &&
      std::abs(result.queue_trend_per_cycle) <= trend_scale &&
      result.loss_ratio < config.soft_loss_threshold &&
      result.ecn_ratio < config.soft_ecn_threshold;

  if (result.measurement_confidence <
      config.measurement_confidence_update_min) {
    result.classification = FbbrOperatingPointClassification::kInvalid;
    result.invalid_reason = "measurement_confidence_low";
    result.optimality_score = 0.0;
    result.rate_adjustment_signal = 0.0;
  } else if (buffer_saturated) {
    result.classification =
        FbbrOperatingPointClassification::kBufferSaturated;
    result.invalid_reason = "hard_congestion";
  } else if (overload && decisive_queue_overload) {
    // Queue reserve violations are directly observable. A phase-gradient
    // conflict must not suppress the bounded 2-5% queue drain response.
    result.classification =
        FbbrOperatingPointClassification::kQueuedOverload;
    result.invalid_reason = "none";
  } else if (!result.cross_block_delay_stable ||
             result.native_baseline_drift > config.max_native_baseline_drift ||
             result.delivery_baseline_drift >
                 config.dynamic_delivery_trend_threshold ||
             result.confidence_cycle < 0.60 || ambiguous) {
    result.classification = FbbrOperatingPointClassification::kDynamic;
    result.invalid_reason = strong_gradient_disagreement
                                ? "gradient_disagreement"
                                : "dynamic_environment";
  } else if (near_optimal) {
    result.classification = FbbrOperatingPointClassification::kNearOptimal;
    result.invalid_reason = "none";
  } else if (overload) {
    result.classification =
        FbbrOperatingPointClassification::kQueuedOverload;
    result.invalid_reason = "none";
  } else if (underload) {
    result.classification = FbbrOperatingPointClassification::kUnderload;
    result.invalid_reason = "none";
  } else {
    result.classification = FbbrOperatingPointClassification::kDynamic;
    result.invalid_reason = "uncertain_operating_point";
  }
  FillCandidate(config, best.samples, best.d0, &result);
  return result;
}

FbbrCruiseConsensusResult FBBRFrequencySearch::BuildCruiseConsensus(
    const FBBRFrequencySearchConfig& config,
    const std::vector<FbbrOperatingPointBlockResult>& blocks) {
  FbbrCruiseConsensusResult consensus;
  consensus.invalid_reason = "no_near_optimal_run";
  if (blocks.empty()) return consensus;
  if (blocks.back().classification !=
          FbbrOperatingPointClassification::kNearOptimal ||
      !blocks.back().candidate.valid) {
    consensus.invalid_reason = "latest_block_not_near_optimal";
    return consensus;
  }
  std::vector<const FbbrOperatingPointBlockResult*> run;
  for (auto it = blocks.rbegin(); it != blocks.rend(); ++it) {
    if (it->classification != FbbrOperatingPointClassification::kNearOptimal ||
        !it->candidate.valid) {
      break;
    }
    run.push_back(&*it);
  }
  if (run.empty()) return consensus;
  std::reverse(run.begin(), run.end());
  std::vector<std::pair<double, double>> values;
  double min_value = std::numeric_limits<double>::infinity();
  double max_value = 0.0;
  double confidence_weighted = 0.0;
  double weight_sum = 0.0;
  for (const auto* block : run) {
    const double weight = std::max(block->measurement_confidence, 1e-6);
    values.push_back({block->candidate.bandwidth_bps, weight});
    min_value = std::min(min_value, block->candidate.bandwidth_bps);
    max_value = std::max(max_value, block->candidate.bandwidth_bps);
    confidence_weighted += weight * block->measurement_confidence;
    weight_sum += weight;
    consensus.robust_cv = std::max(consensus.robust_cv,
                                    block->candidate.robust_cv);
    consensus.relative_ci_width = std::max(
        consensus.relative_ci_width, block->candidate.relative_ci_width);
    consensus.cycle_count += block->candidate.cycle_count;
  }
  consensus.raw_candidate_bps = WeightedMedian(values);
  if (run.size() >= 2 &&
      (max_value - min_value) /
              std::max(consensus.raw_candidate_bps, 1.0) >
          config.max_interblock_candidate_diff) {
    consensus.invalid_reason = "interblock_candidate_conflict";
    return consensus;
  }
  consensus.block_count = static_cast<uint32_t>(run.size());
  consensus.source_block_id = run.back()->block_id;
  consensus.confidence = confidence_weighted / std::max(weight_sum, 1e-9);
  consensus.valid = consensus.raw_candidate_bps > 0.0 &&
                    std::isfinite(consensus.raw_candidate_bps);
  consensus.invalid_reason = consensus.valid ? "none" : "candidate_invalid";
  return consensus;
}

FbbrHistoryUpdateResult FBBRFrequencySearch::StabilizeHistory(
    const FBBRFrequencySearchConfig& config,
    const FbbrCruiseConsensusResult& consensus,
    bool old_valid,
    double old_bandwidth_bps,
    bool clear_native_change_evidence) {
  FbbrHistoryUpdateResult update;
  update.raw_candidate_bps = consensus.raw_candidate_bps;
  update.confidence = consensus.confidence;
  if (!consensus.valid) {
    update.action = "REJECT";
    update.invalid_reason = consensus.invalid_reason;
    return update;
  }
  if (!old_valid || old_bandwidth_bps <= 0.0) {
    update.valid = true;
    update.published_bps = consensus.raw_candidate_bps;
    update.action = "INITIAL_PUBLISH";
    update.invalid_reason = "none";
    return update;
  }
  const double relative_change =
      std::abs(consensus.raw_candidate_bps - old_bandwidth_bps) /
      old_bandwidth_bps;
  if (relative_change <= config.history_small_change) {
    const double alpha = Clamp(
        0.25 + 0.35 * (consensus.confidence - 0.75) / 0.25, 0.25, 0.60);
    update.valid = true;
    update.published_bps = std::exp(
        (1.0 - alpha) * std::log(old_bandwidth_bps) +
        alpha * std::log(consensus.raw_candidate_bps));
    update.action = "LOG_EWMA";
    update.invalid_reason = "none";
  } else if (relative_change <= config.history_medium_change) {
    if (consensus.confidence >= config.medium_change_min_confidence &&
        consensus.robust_cv <= 0.06) {
      update.valid = true;
      update.published_bps = consensus.raw_candidate_bps;
      update.action = "MEDIUM_CHANGE_ACCEPT";
      update.invalid_reason = "none";
    } else {
      update.action = "MEDIUM_CHANGE_REJECT";
      update.invalid_reason = "medium_unconfirmed_change";
    }
  } else if (consensus.confidence >= config.large_change_min_confidence &&
             consensus.robust_cv <= 0.05 && clear_native_change_evidence) {
    update.valid = true;
    update.published_bps = consensus.raw_candidate_bps;
    update.action = "LARGE_CHANGE_ACCEPT";
    update.invalid_reason = "none";
  } else {
    update.action = "LARGE_CHANGE_REJECT";
    update.invalid_reason = "large_unconfirmed_change";
  }
  return update;
}

FBBRQueueServoDecision FBBRQueueReserveServo::Update(
    const FBBRFrequencySearchConfig& config,
    const FBBRQueueServoInput& input,
    FBBRQueueServoStateData* state) {
  FBBRQueueServoDecision decision;
  if (state == nullptr || input.search_baseline_bps <= 0.0 ||
      input.rtprop_s <= 0.0) {
    decision.reason = "INVALID_INPUT";
    return decision;
  }
  const double q_low = config.q_reserve_low_bdp * input.rtprop_s;
  const double q_high = config.q_reserve_high_bdp * input.rtprop_s;
  const double q_peak_cap = config.q_peak_cap_bdp * input.rtprop_s;
  decision.q_low_s = q_low;
  decision.q_high_s = q_high;
  decision.q_peak_cap_s = q_peak_cap;

  const bool hard_queue = q_high > 0.0 &&
      input.q_floor_s >= config.queue_servo_hard_queue_multiple * q_high;
  const bool hard_peak = q_peak_cap > 0.0 &&
      input.q_peak_s >= config.queue_servo_hard_queue_multiple * q_peak_cap;
  const bool hard_congestion =
      input.loss_ratio >= config.hard_loss_threshold ||
      input.ecn_ratio >= config.hard_ecn_threshold || hard_queue || hard_peak;
  if (hard_congestion) {
    state->state = FBBRQueueServoState::kEmergencyDrain;
    state->factor = std::min(
        state->factor, config.queue_servo_emergency_factor);
    state->factor = Clamp(state->factor, 0.10,
                          config.queue_servo_emergency_factor);
    ++state->consecutive_drain_rtts;
    ++state->saturated_down_rtts;
    decision.down_correction = 1.0 - state->factor;
    decision.reason = input.loss_ratio >= config.hard_loss_threshold
        ? "HARD_LOSS" : input.ecn_ratio >= config.hard_ecn_threshold
        ? "HARD_ECN" : hard_queue ? "HARD_QUEUE_FLOOR" : "HARD_QUEUE_PEAK";
  } else if (!input.samples_sufficient) {
    state->state = FBBRQueueServoState::kHold;
    decision.reason = "RTT_SAMPLES_INSUFFICIENT";
  } else if (input.q_floor_s > q_high ||
             (input.q_peak_s > q_peak_cap &&
              input.queue_trend_s_per_s > 0.0)) {
    const double high_error = std::max(
        0.0, (input.q_floor_s - q_high) / input.rtprop_s);
    const double trend_error = std::max(
        0.0, input.rtprop_s * input.queue_trend_s_per_s /
                 std::max(q_high, 1e-9));
    const double down = Clamp(
        config.queue_servo_high_gain * high_error +
            config.queue_servo_trend_gain * trend_error,
        0.0, config.queue_servo_down_step_max);
    double target_factor = 1.0 - down;
    if (input.delivery_median_bps > 0.0) {
      target_factor = std::min(
          target_factor,
          config.queue_servo_delivery_drain_factor *
              input.delivery_median_bps / input.search_baseline_bps);
    }
    const double old_factor = state->factor;
    state->factor = Clamp(std::min(state->factor, target_factor),
                          config.queue_servo_emergency_factor, 1.0);
    state->state = FBBRQueueServoState::kDrain;
    ++state->consecutive_drain_rtts;
    if (state->factor <= 0.90 + 1e-9) ++state->saturated_down_rtts;
    decision.down_correction = std::max(0.0, old_factor - state->factor);
    decision.reason = "QUEUE_ABOVE_RESERVE";
  } else if (input.q_peak_s < q_low && !input.in_recovery &&
             input.flow_backlogged && input.underload_evidence &&
             input.loss_ratio <= 0.0 && input.ecn_ratio <= 0.0 &&
             input.queue_trend_s_per_s <= 0.0) {
    const double low_error = std::max(
        0.0, (q_low - input.q_peak_s) / input.rtprop_s);
    double up = Clamp(config.queue_servo_low_gain * low_error,
                      0.0, config.queue_servo_up_step_max);
    const bool recovering_from_saturated_drain =
        state->saturated_down_rtts > 0 && state->factor < 0.98;
    if (recovering_from_saturated_drain) {
      up = std::max(up, std::min(config.queue_servo_recovery_step_max,
                                 config.queue_servo_up_step_max));
    } else if (state->factor < 0.98) {
      up = std::max(up, 0.50 * std::min(
          config.queue_servo_recovery_step_max,
          config.queue_servo_up_step_max));
    }
    state->factor = std::min(1.02, state->factor + up);
    state->state = FBBRQueueServoState::kReserveRecovery;
    state->consecutive_drain_rtts = 0;
    if (state->factor >= 0.98) state->saturated_down_rtts = 0;
    decision.up_correction = up;
    decision.reason = "QUEUE_RESERVE_LOW";
  } else if (input.q_floor_s >= q_low && input.q_peak_s <= q_high &&
             std::abs(input.queue_trend_s_per_s) <=
                 0.01 * input.rtprop_s /
                     std::max(input.rtprop_s, 1e-9)) {
    const double delta = Clamp(1.0 - state->factor,
                               -config.queue_servo_recovery_step_max,
                               config.queue_servo_recovery_step_max);
    state->factor = Clamp(state->factor + delta, 0.10, 1.02);
    state->state = FBBRQueueServoState::kTargetBand;
    state->consecutive_drain_rtts = 0;
    state->saturated_down_rtts = 0;
    decision.up_correction = std::max(0.0, delta);
    decision.down_correction = std::max(0.0, -delta);
    decision.reason = "QUEUE_IN_TARGET_BAND";
  } else {
    state->state = FBBRQueueServoState::kHold;
    state->consecutive_drain_rtts = 0;
    decision.reason = "QUEUE_BETWEEN_GATES";
  }

  decision.state = state->state;
  decision.factor = state->factor;
  decision.final_nonprobe_baseline_bps =
      input.search_baseline_bps * state->factor;
  decision.consecutive_drain_rtts = state->consecutive_drain_rtts;
  decision.baseline_commit_eligible =
      state->saturated_down_rtts >= config.queue_servo_commit_min_rtts &&
      input.q_floor_s > q_high && input.sustainable_direction <= -0.20 &&
      input.delivery_median_bps > 0.0;
  if (decision.baseline_commit_eligible) {
    const double commit = std::min(
        config.queue_servo_commit_step_max,
        std::max(0.0, 1.0 - state->factor));
    decision.baseline_commit_bps = std::max(
        input.search_baseline_bps * (1.0 - commit),
        config.queue_servo_delivery_drain_factor * input.delivery_median_bps);
  }
  return decision;
}

FBBRSearchControllerState FBBRSearchController::Initialize(
    double native_baseline_bps,
    double amplitude_ratio,
    double carrier_period_s) {
  FBBRSearchControllerState state;
  state.state = native_baseline_bps > 0.0
                    ? FBBRSearchState::kAcquireInput
                    : FBBRSearchState::kDisabled;
  state.search_active = native_baseline_bps > 0.0;
  state.search_attempts = state.search_active ? 1 : 0;
  state.cruise_entry_native_bps = std::max(0.0, native_baseline_bps);
  state.current_search_bps = state.cruise_entry_native_bps;
  state.pending_search_bps = state.current_search_bps;
  state.current_amplitude_ratio = std::max(0.0, amplitude_ratio);
  state.carrier_period_s = std::max(0.0, carrier_period_s);
  return state;
}

double FBBRSearchController::RaisedCosineLogRamp(
    double from_bps,
    double to_bps,
    double progress) {
  if (from_bps <= 0.0 || to_bps <= 0.0) return std::max(from_bps, to_bps);
  const double tau = Clamp(progress, 0.0, 1.0);
  const double alpha = 0.5 * (1.0 - std::cos(kPi * tau));
  return std::exp((1.0 - alpha) * std::log(from_bps) +
                  alpha * std::log(to_bps));
}

FBBRWindowControlDecision FBBRSearchController::Decide(
    const FBBRFrequencySearchConfig& config,
    const FbbrOperatingPointBlockResult& window,
    double current_native_bps,
    double rtprop_anchor_s,
    FBBRSearchControllerState* state) {
  FBBRWindowControlDecision decision;
  if (state == nullptr) {
    decision.update_reason = "STATE_MISSING";
    return decision;
  }
  decision.state_before = state->state;
  decision.baseline_before_bps = state->current_search_bps;
  decision.proposed_baseline_bps = state->current_search_bps;
  decision.applied_next_baseline_bps = state->current_search_bps;
  decision.measurement_confidence = window.measurement_confidence;
  decision.raw_optimality_score = window.raw_optimality_score;
  decision.reported_optimality_score = window.optimality_score;
  decision.direction_score = window.direction_score;
  decision.underload_evidence = window.underload_evidence;
  decision.overload_evidence = window.overload_evidence;
  decision.classification = window.classification;
  decision.carrier_detected = state->carrier_detected;
  decision.carrier_sense_snr = state->carrier_sense_snr;
  decision.carrier_sense_amplitude = state->carrier_sense_amplitude;
  decision.collision_suspected = window.collision_suspected;
  if (window.independent_for_control) ++state->control_window_index;

  auto clear_bracket = [state]() {
    state->underload_bound_valid = false;
    state->overload_bound_valid = false;
    state->underload_bound_bps = 0.0;
    state->overload_bound_bps = 0.0;
    state->bracket_age_windows = 0;
    state->provisional_validation_pending = false;
    state->provisional_age_cruises = 0;
  };
  auto clear_lock = [state]() {
    state->consecutive_near_optimal = 0;
    state->locked_validation_windows = 0;
    state->trusted_candidates_bps.clear();
    state->trusted_candidate_confidence.clear();
    state->trusted_candidate_window_ids.clear();
  };
  auto mild_drain_target = [&config, &window](double baseline_bps) {
    const double queue_excess = std::min(
        1.0, std::max(0.0, -window.queue_band_error));
    const double trend_norm = std::min(
        1.0, std::max(0.0, window.queue_trend_per_cycle /
            std::max(config.max_queue_trend_per_cycle, 0.01)));
    const double step = Clamp(
        config.mild_drain_step_min + 0.03 * queue_excess +
            0.02 * trend_norm,
        config.mild_drain_step_min, config.mild_drain_step_max);
    const double rate_target = baseline_bps * (1.0 - step);
    const double delivery_floor = window.delivery_median_bps > 0.0
        ? config.mild_drain_delivery_floor * window.delivery_median_bps
        : 0.0;
    return std::min(baseline_bps, std::max(rate_target, delivery_floor));
  };

  const bool hard_congestion =
      window.loss_ratio >= config.hard_loss_threshold ||
      window.ecn_ratio >= config.hard_ecn_threshold ||
      window.classification ==
          FbbrOperatingPointClassification::kBufferSaturated;
  if (hard_congestion) {
    decision.measurement_valid = window.measurement_confidence >=
                                 config.measurement_confidence_update_min;
    decision.hard_loss_abort = true;
    decision.classification =
        FbbrOperatingPointClassification::kBufferSaturated;
    decision.proposed_baseline_bps = state->current_search_bps;
    decision.applied_next_baseline_bps = state->current_search_bps;
    decision.log_step = 0.0;
    decision.update_reason = "HARD_CONGESTION_QUEUE_SERVO";
    clear_bracket();
    clear_lock();
    state->same_direction_streak = 0;
    state->previous_direction_sign = 0;
    state->state = FBBRSearchState::kEmergencyDrain;
  } else if (!window.independent_for_control) {
    decision.measurement_valid = window.classification !=
        FbbrOperatingPointClassification::kInvalid;
    decision.update_reason = "OVERLAP_DIAGNOSTIC_HOLD";
  } else if (std::abs(window.queue_servo_factor_mean - 1.0) > 0.02 ||
             window.queue_servo_transition_cycles > 0) {
    decision.measurement_valid = window.classification !=
        FbbrOperatingPointClassification::kInvalid;
    decision.update_reason = "QUEUE_SERVO_TRANSIENT_BASELINE_HOLD";
    state->same_direction_streak = 0;
    state->previous_direction_sign = 0;
  } else if (window.loss_ratio >= config.soft_loss_threshold ||
             window.ecn_ratio >= config.soft_ecn_threshold) {
    decision.proposed_baseline_bps = state->current_search_bps;
    decision.applied_next_baseline_bps = state->current_search_bps;
    decision.log_step = 0.0;
    decision.update_reason = "SOFT_CONGESTION_QUEUE_SERVO_HOLD";
    state->same_direction_streak = 0;
    state->previous_direction_sign = 0;
    clear_lock();
    state->state = FBBRSearchState::kDrain;
  } else {
    const bool valid = window.classification !=
                           FbbrOperatingPointClassification::kInvalid &&
                       window.measurement_confidence >=
                           config.measurement_confidence_update_min;
    decision.measurement_valid = valid;
    if (!valid) {
      ++state->consecutive_invalid;
      state->consecutive_dynamic = 0;
      state->same_direction_streak = 0;
      state->previous_direction_sign = 0;
      clear_lock();
      decision.update_reason = "INVALID_HOLD";
      ++state->unresolved_decisions;
      state->last_failure_reason = window.invalid_reason;
      const bool input_unrealized =
          window.invalid_reason.find("actual_input_energy_low") !=
              std::string::npos ||
          window.invalid_reason.find("input_snr_low") != std::string::npos ||
          window.invalid_reason.find("weak_periodic_response") !=
              std::string::npos;
      const bool acquire_cwnd_drain = state->is_pulser &&
          window.cwnd_limited_fraction >= 0.50 &&
          window.delivery_median_bps > 0.0 &&
          window.actual_input_amplitude_ratio < 0.025;
      const bool acquire_queue_drain = state->is_pulser &&
          window.delivery_median_bps > 0.0 && rtprop_anchor_s > 0.0 &&
          (window.q_floor_s / rtprop_anchor_s >=
               config.q_floor_enter_drain_ratio ||
           (window.q_amplitude_s >= 0.01 * rtprop_anchor_s &&
            window.drain_ratio < 0.50));
      if (acquire_queue_drain || acquire_cwnd_drain) {
        if (acquire_cwnd_drain && !acquire_queue_drain) {
          decision.proposed_baseline_bps = mild_drain_target(
              state->current_search_bps);
        }
        decision.update_reason = acquire_queue_drain
            ? "ACQUIRE_QUEUE_SERVO_BASELINE_HOLD"
            : "ACQUIRE_MILD_CWND_DRAIN";
        state->state = FBBRSearchState::kDrain;
      } else if (input_unrealized && state->is_pulser) {
        ++state->input_unrealized_streak;
        decision.request_period_increase = true;
      }
      if (state->consecutive_invalid >= 2 && !acquire_cwnd_drain &&
          !acquire_queue_drain) {
        state->state = FBBRSearchState::kPersistentUnresolved;
        decision.update_reason = "INVALID_PERSISTENT_UNRESOLVED";
      }
      if (!state->is_pulser) {
        state->state = FBBRSearchState::kWatcher;
        decision.update_reason = "WATCHER_OBSERVATION_HOLD";
      }
    } else if (window.classification ==
               FbbrOperatingPointClassification::kDynamic) {
      state->consecutive_invalid = 0;
      ++state->consecutive_dynamic;
      ++state->bracket_age_windows;
      state->same_direction_streak = 0;
      state->previous_direction_sign = 0;
      clear_lock();
      decision.update_reason = "DYNAMIC_HOLD";
      if (state->consecutive_dynamic >= config.dynamic_reset_windows) {
        clear_bracket();
        state->state = FBBRSearchState::kDynamicReacquire;
        decision.proposed_baseline_bps = current_native_bps;
        decision.applied_next_baseline_bps = current_native_bps;
        decision.update_reason = "DYNAMIC_RESET_TO_CURRENT_NATIVE";
      }
    } else {
      state->consecutive_invalid = 0;
      state->consecutive_dynamic = 0;
      state->input_unrealized_streak = 0;
      ++state->valid_direction_decisions;
      const bool underload = window.classification ==
          FbbrOperatingPointClassification::kUnderload;
      const bool overload = window.classification ==
          FbbrOperatingPointClassification::kQueuedOverload;
      const bool sustainable_overload = overload &&
          window.frequency_direction <= -0.20;
      const bool near = window.classification ==
          FbbrOperatingPointClassification::kNearOptimal;
      bool provisional_validated = false;
      double provisional_target_bps = 0.0;
      if (state->provisional_validation_pending) {
        bool consistent = near;
        if (state->underload_bound_valid && state->overload_bound_valid &&
            state->underload_bound_bps < state->overload_bound_bps) {
          const double midpoint = std::sqrt(state->underload_bound_bps *
                                            state->overload_bound_bps);
          provisional_target_bps = midpoint;
          consistent = near ||
              (state->current_search_bps <= midpoint && underload) ||
              (state->current_search_bps >= midpoint && overload);
        } else if (state->underload_bound_valid) {
          consistent = underload || near;
        } else if (state->overload_bound_valid) {
          consistent = overload || near;
        }
        if (consistent) {
          state->provisional_validation_pending = false;
          state->provisional_age_cruises = 0;
          provisional_validated = true;
        } else {
          clear_bracket();
          decision.update_reason = "PROVISIONAL_VALIDATION_REJECT";
        }
      }
      if (underload) {
        decision.bracket_updated = true;
        state->underload_bound_valid = true;
        state->underload_bound_bps = std::max(
            state->underload_bound_bps, state->current_search_bps);
      }
      if (sustainable_overload) {
        decision.bracket_updated = true;
        state->overload_bound_valid = true;
        state->overload_bound_bps = state->overload_bound_bps > 0.0
            ? std::min(state->overload_bound_bps, state->current_search_bps)
            : state->current_search_bps;
      }
      const bool bracketed = state->underload_bound_valid &&
          state->overload_bound_valid &&
          state->underload_bound_bps < state->overload_bound_bps;
      if (decision.bracket_updated && bracketed) {
        ++state->bracket_generation;
        state->bracket_age_windows = 0;
        state->native_bps_when_bracket_created = current_native_bps;
        state->rtprop_s_when_bracket_created = rtprop_anchor_s;
      } else if (bracketed) {
        ++state->bracket_age_windows;
      }
      if (bracketed &&
          (state->bracket_age_windows > config.bracket_ttl_windows ||
           (state->native_bps_when_bracket_created > 0.0 &&
            std::abs(current_native_bps -
                     state->native_bps_when_bracket_created) /
                    state->native_bps_when_bracket_created >
                config.dynamic_native_change_threshold) ||
           (state->rtprop_s_when_bracket_created > 0.0 &&
            std::abs(rtprop_anchor_s -
                     state->rtprop_s_when_bracket_created) /
                    state->rtprop_s_when_bracket_created > 0.10))) {
        clear_bracket();
      }

      if (provisional_validated && bracketed) {
        decision.proposed_baseline_bps = provisional_target_bps > 0.0
            ? provisional_target_bps
            : std::sqrt(state->underload_bound_bps *
                        state->overload_bound_bps);
        decision.update_reason = "PROVISIONAL_VALIDATED_MIDPOINT";
        state->state = FBBRSearchState::kSeek;
      } else if (near &&
          window.lockable_score && window.independent_for_trusted &&
          window.measurement_confidence >=
              config.measurement_confidence_lock_min &&
          window.target_score >=
              config.near_optimal_score_threshold &&
          std::abs(window.direction_score) <=
              config.near_optimal_direction_abs_max &&
          window.loss_ratio < config.soft_loss_threshold &&
          window.ecn_ratio < config.soft_ecn_threshold &&
          window.q_floor_s >= window.q_reserve_low_s &&
          window.q_floor_s <= window.q_reserve_high_s &&
          window.q95_s <= window.q_peak_cap_s &&
          std::abs(window.queue_servo_factor_mean - 1.0) <= 0.02 &&
          window.queue_servo_transition_cycles == 0 &&
          window.rtprop_confidence >= config.rtprop_confidence_lock_min) {
        ++state->consecutive_near_optimal;
        std::vector<std::pair<double, double>> candidates;
        candidates.push_back({state->current_search_bps,
            window.measurement_confidence * window.raw_optimality_score});
        if (window.candidate.valid) {
          candidates.push_back({window.candidate.bandwidth_bps,
                                std::max(0.10, 1.0 -
                                    window.candidate.robust_cv)});
        }
        if (state->underload_bound_valid && state->overload_bound_valid &&
            state->underload_bound_bps < state->overload_bound_bps) {
          const double midpoint = std::sqrt(state->underload_bound_bps *
                                            state->overload_bound_bps);
          const double width = (state->overload_bound_bps -
                                state->underload_bound_bps) /
                               std::max(midpoint, 1.0);
          candidates.push_back({midpoint, std::max(0.05, 1.0 - width)});
        }
        decision.window_candidate_bps = WeightedMedian(candidates);
        decision.window_candidate_valid =
            decision.window_candidate_bps > 0.0;
        if (decision.window_candidate_valid) {
          state->trusted_candidates_bps.push_back(
              decision.window_candidate_bps);
          state->trusted_candidate_confidence.push_back(
              std::min(window.measurement_confidence,
                       window.target_score));
          state->trusted_candidate_window_ids.push_back(window.event_window_id);
        }
        decision.lock_candidate = state->consecutive_near_optimal >= 1;
        state->state = state->consecutive_near_optimal >= 2 &&
                               state->control_window_index >=
                                   config.min_control_windows
                           ? FBBRSearchState::kLocked
                           : FBBRSearchState::kLockCandidate;
        decision.locked = state->state == FBBRSearchState::kLocked;
        decision.update_reason = decision.locked ? "LOCKED_HOLD" :
                                                   "LOCK_CANDIDATE_HOLD";
        if (decision.locked) ++state->locked_validation_windows;

        const size_t count = state->trusted_candidates_bps.size();
        if (decision.locked &&
            count >= config.trusted_candidate_min_windows) {
          const double candidate_mean = std::accumulate(
              state->trusted_candidates_bps.begin(),
              state->trusted_candidates_bps.end(), 0.0) / count;
          double variance = 0.0;
          double minimum = std::numeric_limits<double>::infinity();
          double maximum = 0.0;
          std::vector<std::pair<double, double>> consensus;
          for (size_t i = 0; i < count; ++i) {
            const double value = state->trusted_candidates_bps[i];
            variance += (value - candidate_mean) * (value - candidate_mean);
            minimum = std::min(minimum, value);
            maximum = std::max(maximum, value);
            consensus.push_back({value,
                state->trusted_candidate_confidence[i]});
          }
          const double cv = candidate_mean > 0.0
              ? std::sqrt(variance / count) / candidate_mean
              : std::numeric_limits<double>::infinity();
          if (cv <= config.trusted_candidate_cv_max &&
              maximum / std::max(minimum, 1.0) <=
                  config.trusted_candidate_ratio_max) {
            decision.trusted_bw_published = true;
            decision.trusted_bw_bps = WeightedMedian(consensus);
            decision.trusted_confidence = std::min(
                Median(state->trusted_candidate_confidence),
                std::min(1.0, 1.0 - cv / 0.10));
          }
        }
      } else if (near &&
                 window.measurement_confidence >=
                     config.measurement_confidence_track_min) {
        clear_lock();
        decision.log_step = Clamp(
            0.02 * window.direction_score,
            -config.track_step_max, config.track_step_max);
        decision.proposed_baseline_bps = state->current_search_bps *
                                         std::exp(decision.log_step);
        decision.update_reason = "TRACK_SMALL_STEP";
        state->state = FBBRSearchState::kTrack;
      } else {
        clear_lock();
        const int sign = window.direction_score > 0.0 ? 1 :
                         window.direction_score < 0.0 ? -1 : 0;
        if (sign != 0 && sign == state->previous_direction_sign) {
          ++state->same_direction_streak;
        } else {
          state->same_direction_streak = sign == 0 ? 0 : 1;
        }
        state->previous_direction_sign = sign;
        if (state->underload_bound_valid && state->overload_bound_valid &&
            state->underload_bound_bps < state->overload_bound_bps) {
          decision.proposed_baseline_bps = std::sqrt(
              state->underload_bound_bps * state->overload_bound_bps);
          decision.update_reason = "BRACKET_GEOMETRIC_MIDPOINT";
          state->state = FBBRSearchState::kSeek;
        } else if (underload) {
          if (state->same_direction_streak <
              static_cast<int>(config.underload_confirmation_windows)) {
            decision.update_reason = "UNDERLOAD_CONFIRMATION_HOLD";
            state->state = FBBRSearchState::kSeek;
          } else {
            const double multiplier = std::min(
                config.same_direction_streak_multiplier_max,
                state->same_direction_streak >= 3 ? 1.50 : 1.25);
            const double magnitude = window.measurement_confidence *
                (1.0 - window.raw_optimality_score) *
                std::abs(window.direction_score);
            const double up_limit = state->same_direction_streak >= 3
                ? config.confirmed_up_step_max
                : config.ordinary_up_step_max;
            decision.log_step = std::min(
                up_limit,
                0.08 * magnitude * multiplier);
            decision.proposed_baseline_bps = state->current_search_bps *
                                             std::exp(decision.log_step);
            decision.update_reason = "CONFIRMED_UNDERLOAD_UP";
            state->state = FBBRSearchState::kSeek;
          }
        } else if (sustainable_overload) {
          const double step = std::min(
              config.ordinary_down_step_max,
              config.queue_servo_commit_step_max);
          decision.proposed_baseline_bps = state->current_search_bps *
                                           (1.0 - step);
          decision.log_step = std::log(std::max(1.0 - step, 1e-9));
          decision.update_reason = "SUSTAINABLE_OVERLOAD_DOWN";
          state->state = FBBRSearchState::kSeek;
        } else if (overload) {
          decision.update_reason = "QUEUE_SERVO_DRAIN_BASELINE_HOLD";
          state->state = FBBRSearchState::kDrain;
        } else {
          decision.update_reason = "AMBIGUOUS_HOLD";
        }
      }
    }
  }

  const double minimum = config.min_search_scale *
                         state->cruise_entry_native_bps;
  const double maximum = config.max_search_scale *
                         state->cruise_entry_native_bps;
  if (state->cruise_entry_native_bps > 0.0) {
    const double unclamped = decision.proposed_baseline_bps;
    decision.applied_next_baseline_bps =
        Clamp(unclamped, minimum, maximum);
    decision.search_range_exhausted =
        std::abs(unclamped - decision.applied_next_baseline_bps) > 1.0;
    if (decision.search_range_exhausted) {
      decision.update_reason += "|SEARCH_RANGE_EXHAUSTED";
    }
  } else {
    decision.applied_next_baseline_bps =
        decision.proposed_baseline_bps;
  }
  state->pending_search_bps = decision.applied_next_baseline_bps;
  state->current_search_bps = decision.applied_next_baseline_bps;
  decision.state_after = state->state;
  return decision;
}

namespace {

enum class SyntheticModel {
  kUnderload,
  kIdeal,
  kOverload,
  kDynamic,
};

std::vector<FbbrPhaseBinSample> MakeSyntheticBins(
    const FBBRFrequencySearchConfig& config,
    const FbbrProbeSignature& signature,
    SyntheticModel model,
    double feedback_delay_ratio,
    bool app_limited,
    double cross_frequency_amplitude,
    int different_code_id,
    bool collision,
    double baseline_override_bps = 0.0) {
  const uint32_t bins_per_cycle = config.phase_bins_per_cycle;
  const int cycles = 12;
  const int bin_count = cycles * bins_per_cycle + 8;
  const double bin_s = signature.period_s / bins_per_cycle;
  const double capacity = 100e6;
  const double model_baseline = model == SyntheticModel::kUnderload ? 80e6
                        : model == SyntheticModel::kOverload ? 108e6
                        : model == SyntheticModel::kIdeal ? 100e6
                                                          : capacity;
  const double baseline = baseline_override_bps > 0.0
      ? baseline_override_bps
      : model_baseline;
  std::vector<double> send(bin_count, baseline);
  std::vector<double> delivery_source(bin_count, capacity);
  std::vector<double> q_source(bin_count, 0.0);
  double queue_s = model == SyntheticModel::kOverload ? 0.02
      : model == SyntheticModel::kIdeal ?
          0.035 * signature.rtprop_s : 0.0;
  for (int i = 0; i < bin_count; ++i) {
    const double time_s = (i + 0.5) * bin_s;
    const double z = FBBRFrequencySearch::ProbeWaveform(signature, time_s);
    double other_code_z = 0.0;
    if (different_code_id >= 0) {
      FbbrProbeSignature other = signature;
      other.code_id = static_cast<uint32_t>(different_code_id);
      other_code_z = FBBRFrequencySearch::ProbeWaveform(other, time_s);
    }
    send[i] = baseline * (1.0 + signature.amplitude_ratio * z);
    if (app_limited && z > 0.0) send[i] = baseline;
    double capacity_now = capacity;
    if (model == SyntheticModel::kDynamic && i > bin_count / 2) {
      capacity_now = 0.8 * capacity;
    }
    if (model == SyntheticModel::kUnderload) {
      delivery_source[i] = send[i];
      queue_s = 0.0;
    } else if (model == SyntheticModel::kIdeal) {
      delivery_source[i] = capacity_now + 0.365 * (send[i] - capacity_now);
      queue_s = std::max(0.0, queue_s +
          (send[i] - delivery_source[i]) / capacity_now * bin_s);
    } else {
      const double queued_service_bps = queue_s * capacity_now / bin_s;
      delivery_source[i] = std::min(
          capacity_now, send[i] + queued_service_bps);
      queue_s = std::max(0.0, queue_s +
          (send[i] - delivery_source[i]) / capacity_now * bin_s);
    }
    double observed_queue_s = queue_s;
    if (cross_frequency_amplitude > 0.0) {
      delivery_source[i] *= 1.0 + cross_frequency_amplitude *
          std::sin(2.0 * kPi * 1.37 * signature.frequency_hz * time_s + 0.3);
      observed_queue_s = std::max(0.0, observed_queue_s +
          cross_frequency_amplitude * 0.1 * signature.rtprop_s *
          std::sin(2.0 * kPi * 1.37 * signature.frequency_hz * time_s));
    }
    if (different_code_id >= 0) {
      // The competing flow contaminates outputs, never this flow's realized
      // input.  Its complete Walsh block is orthogonal to our demodulator.
      delivery_source[i] = std::max(
          1.0, delivery_source[i] + 0.015 * capacity * other_code_z);
      observed_queue_s = std::max(
          0.0, observed_queue_s + 0.01 * signature.rtprop_s * other_code_z);
    }
    if (collision) {
      // Same-signature pollution is deliberately nonstationary across cycles;
      // a safe implementation must reduce coherence or reject the gradient.
      const int cycle = i / static_cast<int>(bins_per_cycle);
      const double factor = cycle % 3 == 0 ? 0.8 : (cycle % 3 == 1 ? -0.6 : 0.2);
      const double collision_wave = FBBRFrequencySearch::ProbeWaveform(
          signature, time_s + 0.11 * bin_s);
      delivery_source[i] = std::max(
          1.0, delivery_source[i] + factor * 0.04 * capacity * collision_wave);
      observed_queue_s = std::max(
          0.0, observed_queue_s +
                   factor * 0.08 * signature.rtprop_s * collision_wave);
    }
    q_source[i] = observed_queue_s;
  }
  const double delay_bins = feedback_delay_ratio * signature.rtprop_s / bin_s;
  std::vector<FbbrPhaseBinSample> bins;
  bins.reserve(bin_count);
  for (int i = 0; i < bin_count; ++i) {
    const double source_pos = i - delay_bins;
    const int left = static_cast<int>(std::floor(source_pos));
    const double fraction = source_pos - left;
    const auto interpolate = [left, fraction, &delivery_source](
                                 const std::vector<double>& values,
                                 double fallback) {
      if (left < 0 || left + 1 >= static_cast<int>(values.size())) return fallback;
      return values[left] + fraction * (values[left + 1] - values[left]);
    };
    const double delivered = interpolate(delivery_source, baseline);
    const double qdelay = interpolate(q_source, 0.0);
    FbbrPhaseBinSample bin;
    bin.bin_index = i;
    bin.cycle_index = i / bins_per_cycle;
    bin.phase_bin_index = i % bins_per_cycle;
    bin.time_start_s = i * bin_s;
    bin.time_end_s = (i + 1) * bin_s;
    const double center = (i + 0.5) * bin_s;
    bin.phase_rad = signature.initial_phase_rad +
                    2.0 * kPi * center / signature.period_s;
    bin.code_sign = signature.waveform == "sine"
        ? FBBRFrequencySearch::WalshSign(signature.code_id, bin.cycle_index)
        : 1;
    bin.coded_excitation = FBBRFrequencySearch::ProbeWaveform(signature, center);
    bin.native_pacing_bps = baseline;
    bin.commanded_pacing_bps = send[i];
    bin.actual_send_bps = send[i];
    bin.delivery_rate_bps = delivered;
    bin.sent_bytes = static_cast<uint64_t>(send[i] * bin_s / 8.0);
    bin.acked_bytes = static_cast<uint64_t>(delivered * bin_s / 8.0);
    bin.latest_rtt_s = signature.rtprop_s + qdelay;
    bin.qdelay_s = qdelay;
    bin.app_limited_fraction = app_limited && bin.coded_excitation > 0.0 ? 1.0 : 0.0;
    bin.coverage = 1.0;
    bin.rtt_valid = true;
    bin.valid = true;
    bins.push_back(bin);
  }
  return bins;
}

}  // namespace

bool FBBRFrequencySearch::RunSelfTests(std::ostream& os) {
  FBBRFrequencySearchConfig config;
  config.search_controller_enabled = false;
  config.min_score_cycles = 4;
  // Synthetic fluid traces are deliberately noiseless; retain production
  // thresholds except for the conditioning confidence, which is not the
  // subject of these classification tests.
  FbbrProbeSignature signature;
  signature.flow_identity = 1;
  signature.cruise_id = 1;
  signature.period_slot = 0;
  signature.period_rtts = 8;
  signature.code_id = 2;
  signature.initial_phase_rad = 0.0;
  signature.rtprop_s = 0.010;
  signature.period_s = 0.080;
  signature.frequency_hz = 12.5;
  signature.amplitude_ratio = 0.05;
  const int64_t first = 2 * config.phase_bins_per_cycle +
      static_cast<int64_t>(std::llround(
          signature.rtprop_s /
          (signature.period_s / config.phase_bins_per_cycle)));
  const int64_t count = config.analysis_cycles * config.phase_bins_per_cycle;
  bool passed = true;
  auto require = [&passed, &os](bool condition, const std::string& message) {
    if (!condition) {
      passed = false;
      os << "FAIL: " << message << "\n";
    }
  };
  auto analyze_bins = [&](const char* name,
                          const std::vector<FbbrPhaseBinSample>& bins) {
    const auto result = AnalyzeBlock(config, signature, bins, first, count, 0,
                                     std::numeric_limits<double>::quiet_NaN());
    os << name << "," << FbbrOperatingPointClassificationName(result.classification)
       << ",C=" << result.measurement_confidence
       << ",Gd=" << result.response.delivery_gain
       << ",Gq=" << result.response.queue_storage_gain
       << ",Gd+=" << result.positive_delivery_gain
       << ",Gq+=" << result.positive_queue_build_gain
       << ",R2d=" << result.response.delivery_harmonic_ratio
       << ",R2q=" << result.response.queue_harmonic_ratio
       << ",drain=" << result.drain_ratio
       << ",qfloor_us=" << result.q_floor_s * 1e6
       << ",g=" << result.gradient_fused
       << ",h=" << result.curvature_finite_difference
       << ",Sfull=" << result.full_score
       << ",Slowq=" << result.low_queue_score
       << ",Sstat=" << result.stationary_score
       << ",Ssafe=" << result.safe_score
       << ",Sopt=" << result.optimality_score
       << ",Ccycle=" << result.confidence_cycle
       << ",Ddrift=" << result.delivery_baseline_drift
       << ",g95=[" << result.gradient_ci95_low << ";"
       << result.gradient_ci95_high << "]"
       << ",candidate=" << (result.candidate.valid ? result.candidate.bandwidth_bps : 0.0)
       << ",candidate_cycles=" << result.candidate.cycle_count
       << ",candidate_reason=" << result.candidate.invalid_reason
       << ",reason=" << result.invalid_reason << "\n";
    return result;
  };
  auto run = [&](const char* name, SyntheticModel model,
                 double delay_ratio, bool app_limited,
                 double cross_frequency, int other_code, bool collision) {
    const auto bins = MakeSyntheticBins(config, signature, model, delay_ratio,
                                        app_limited, cross_frequency,
                                        other_code, collision);
    return analyze_bins(name, bins);
  };

  os << "# F-BBR event-triggered frequency-search deterministic tests\n";
  const auto underload = run("underload", SyntheticModel::kUnderload, 1.0,
                             false, 0.0, -1, false);
  require(underload.classification == FbbrOperatingPointClassification::kUnderload,
          "underload classification");
  require(underload.response.delivery_gain >= 0.80 &&
              underload.response.delivery_gain <= 1.20,
          "underload delivery gain");
  require(underload.response.queue_storage_gain < 0.15,
          "underload queue gain");
  require(!underload.candidate.valid, "underload cannot publish bandwidth");

  FbbrProbeSignature attenuated_signature = signature;
  attenuated_signature.amplitude_ratio = 0.10;
  auto attenuated_bins = MakeSyntheticBins(
      config, attenuated_signature, SyntheticModel::kUnderload, 1.0,
      false, 0.0, -1, false);
  for (auto& bin : attenuated_bins) {
    const double actual_bps = bin.native_pacing_bps *
        (1.0 + 0.03 * bin.coded_excitation);
    bin.actual_send_bps = actual_bps;
    bin.delivery_rate_bps = actual_bps;
    bin.sent_bytes = static_cast<uint64_t>(actual_bps *
        (bin.time_end_s - bin.time_start_s) / 8.0);
    bin.acked_bytes = bin.sent_bytes;
  }
  const auto attenuated = AnalyzeBlock(
      config, attenuated_signature, attenuated_bins, first, count, 90,
      std::numeric_limits<double>::quiet_NaN());
  os << "attenuated_input," <<
      FbbrOperatingPointClassificationName(attenuated.classification)
     << ",actual=" << attenuated.actual_input_amplitude_ratio
     << ",gain=" << attenuated.realized_amplitude_ratio
     << ",snr=" << attenuated.input_carrier_snr
     << ",reason=" << attenuated.invalid_reason << "\n";
  require(attenuated.realized_amplitude_ratio > 0.25 &&
              attenuated.realized_amplitude_ratio < 0.35 &&
              attenuated.classification !=
                  FbbrOperatingPointClassification::kInvalid,
          "commanded 10 percent / actual 3 percent remains measurable");

  const auto ideal = run("ideal_boundary", SyntheticModel::kIdeal, 1.0,
                         false, 0.0, -1, false);
  require(ideal.classification ==
              FbbrOperatingPointClassification::kNearOptimal,
          "ideal boundary is classified near optimal");
  require(ideal.q_floor_s >= ideal.q_reserve_low_s &&
              ideal.q_floor_s <= ideal.q_reserve_high_s,
          "ideal boundary keeps the shallow queue reserve");
  require(ideal.q95_s <= ideal.q_peak_cap_s,
          "ideal boundary respects queue peak cap");
  if (ideal.candidate.valid) {
    require(std::abs(ideal.candidate.bandwidth_bps - 100e6) / 100e6 <= 0.05,
            "ideal candidate error <= 5%");
  }

  const auto overload = run("persistent_overload", SyntheticModel::kOverload,
                            1.0, false, 0.0, -1, false);
  require(overload.classification ==
              FbbrOperatingPointClassification::kQueuedOverload ||
              overload.classification ==
                  FbbrOperatingPointClassification::kBufferSaturated ||
              overload.classification ==
                  FbbrOperatingPointClassification::kDynamic ||
              overload.classification ==
                  FbbrOperatingPointClassification::kInvalid,
          "persistent queue is overload or safely requests reacquisition");
  require(overload.q_floor_s > overload.q_zero_s,
          "overload standing queue detected");
  require(!overload.candidate.valid, "overload cannot publish bandwidth");

  const auto dynamic = run("dynamic_capacity", SyntheticModel::kDynamic, 1.0,
                           false, 0.0, -1, false);
  require(dynamic.classification == FbbrOperatingPointClassification::kDynamic ||
              dynamic.classification == FbbrOperatingPointClassification::kInvalid ||
              dynamic.classification ==
                  FbbrOperatingPointClassification::kQueuedOverload ||
              dynamic.classification ==
                  FbbrOperatingPointClassification::kBufferSaturated,
          "dynamic capacity rejected");
  require(!dynamic.candidate.valid, "dynamic capacity cannot publish");

  auto irregular_bins = MakeSyntheticBins(config, signature,
      SyntheticModel::kIdeal, 1.0, false, 0.0, -1, false);
  std::mt19937 ack_rng(0xFBB2026u);
  std::uniform_int_distribution<int> event_count_distribution(1, 5);
  bool ack_bytes_conserved = true;
  double naive_event_rate_sum = 0.0;
  uint64_t naive_event_count = 0;
  for (auto& bin : irregular_bins) {
    const uint64_t original_acked = bin.acked_bytes;
    const int event_count = event_count_distribution(ack_rng);
    uint64_t remaining = original_acked;
    uint64_t reconstructed = 0;
    // Randomly coalesce most bytes into one ACK while preserving the exact
    // fixed-bin total.  Event-equal rates become badly biased, whereas the bin
    // delivery rate is invariant by construction.
    for (int event = 0; event < event_count; ++event) {
      const uint64_t bytes = event + 1 == event_count
          ? remaining
          : (remaining == 0 ? 0 : ack_rng() % (remaining + 1));
      remaining -= bytes;
      reconstructed += bytes;
      const double event_interval_s =
          (bin.time_end_s - bin.time_start_s) /
          (event == 0 ? 20.0 : static_cast<double>(event_count));
      naive_event_rate_sum += 8.0 * bytes /
                              std::max(event_interval_s, 1e-9);
      ++naive_event_count;
    }
    ack_bytes_conserved = ack_bytes_conserved &&
                          reconstructed == original_acked;
    bin.acked_bytes = reconstructed;
    bin.delivery_rate_bps = 8.0 * reconstructed /
        std::max(bin.time_end_s - bin.time_start_s, 1e-9);
  }
  const auto irregular = analyze_bins("irregular_ack", irregular_bins);
  if (irregular.candidate.valid) {
    require(std::abs(irregular.candidate.bandwidth_bps - 100e6) / 100e6 <= 0.08,
            "fixed-bin irregular ACK candidate MAPE <= 8%");
  }
  require(ack_bytes_conserved, "irregular ACK aggregation conserves bytes");
  const double naive_event_mean = naive_event_rate_sum /
      std::max<uint64_t>(1, naive_event_count);
  require(!irregular.candidate.valid ||
              std::abs(irregular.candidate.bandwidth_bps - 100e6) <
                  std::abs(naive_event_mean - 100e6),
          "fixed-bin analysis outperforms ACK-event equal weighting");

  const auto app = run("app_limited", SyntheticModel::kIdeal, 1.0,
                       true, 0.0, -1, false);
  require(app.classification == FbbrOperatingPointClassification::kInvalid,
          "app-limited input realization rejected");
  require(!app.candidate.valid, "app-limited cannot publish");

  const auto cross_frequency = run("cross_frequency", SyntheticModel::kIdeal,
                                   1.0, false, 0.01, -1, false);
  require(!cross_frequency.candidate.valid,
          "different-frequency interference cannot publish");
  if (cross_frequency.candidate.valid) {
    require(std::abs(cross_frequency.candidate.bandwidth_bps - 100e6) / 100e6 <= 0.08,
            "cross-frequency candidate error <= 8%");
  }

  const auto other_code = run("different_walsh_code", SyntheticModel::kIdeal,
                              1.0, false, 0.0, 5, false);
  require(!other_code.candidate.valid,
          "different carrier contamination cannot publish");

  const auto collision = run("same_signature_collision", SyntheticModel::kIdeal,
                             1.0, false, 0.0, -1, true);
  require(collision.classification !=
              FbbrOperatingPointClassification::kNearOptimal ||
              collision.measurement_confidence < 0.85,
          "same-signature collision cannot high-confidence publish");

  for (double delay : {0.8, 1.0, 1.2}) {
    std::ostringstream name;
    name << "delay_" << delay;
    const auto delayed = run(name.str().c_str(), SyntheticModel::kUnderload,
                             delay, false, 0.0, -1, false);
    require(delayed.selected_delay_s / signature.rtprop_s >= 0.75 &&
                delayed.selected_delay_s / signature.rtprop_s <= 1.25,
            "delay refinement internal solution");
    require(delayed.gradient_fused > 0.0,
            "delay refinement preserves gradient sign");
  }
  const auto outside_delay = run("delay_outside", SyntheticModel::kUnderload, 1.4,
                                 false, 0.0, -1, false);
  require(!outside_delay.candidate.valid ||
              outside_delay.measurement_confidence < config.min_measurement_confidence,
          "out-of-range delay cannot publish");
  require(!outside_delay.candidate.valid,
          "out-of-range delay cannot create trusted candidate");

  os << "# F-BBR trigger detector and independence tests\n";
  const auto trigger_bins = MakeSyntheticBins(
      config, signature, SyntheticModel::kUnderload, 1.0,
      false, 0.0, -1, false);
  const auto trigger = AnalyzeTriggerCycle(
      config, signature, trigger_bins, 3,
      EventWindowState::kIdleListen, true);
  os << "trigger_correct,pass=" << (trigger.trigger_pass ? "true" : "false")
     << ",period_error=" << trigger.period_error_ratio
     << ",eta=" << trigger.spectral_prominence
     << ",rho=" << trigger.normalized_match
     << ",response_mss=" << trigger.delivery_response_bytes / kProbeMssBytes
     << ",reason=" << trigger.trigger_reason << "\n";
  require(trigger.trigger_pass && trigger.period_error_ratio <= 0.15 &&
              trigger.spectral_prominence >= 2.0 &&
              trigger.normalized_match >= 0.50 &&
              trigger.delivery_response_bytes >= 4.0 * kProbeMssBytes &&
              trigger.combined_trigger_source == "DELIVERY_ONLY",
          "delivery-only carrier arms event capture");

  auto queue_only_bins = trigger_bins;
  for (auto& bin : queue_only_bins) {
    const double center = 0.5 * (bin.time_start_s + bin.time_end_s);
    const double input_time = center - signature.rtprop_s;
    const double queue_wave = input_time >= 0.0
        ? std::sin(2.0 * kPi * input_time / signature.period_s) : 0.0;
    bin.delivery_rate_bps = 100e6;
    bin.acked_bytes = static_cast<uint64_t>(
        bin.delivery_rate_bps * (bin.time_end_s - bin.time_start_s) / 8.0);
    bin.qdelay_s = 0.00035 + 0.00020 * queue_wave;
    bin.latest_rtt_s = signature.rtprop_s + bin.qdelay_s;
    bin.rtt_valid = true;
  }
  const auto queue_only = AnalyzeTriggerCycle(
      config, signature, queue_only_bins, 3,
      EventWindowState::kIdleListen, true);
  os << "trigger_queue_only,source=" << queue_only.combined_trigger_source
     << ",eta_q=" << queue_only.queue_spectral_prominence
     << ",rho_q=" << queue_only.queue_normalized_match
     << ",A_qdot=" << queue_only.queue_derivative_amplitude
     << ",noise=" << queue_only.queue_noise_floor << "\n";
  require(queue_only.trigger_pass && !queue_only.delivery_trigger_pass &&
              queue_only.queue_trigger_pass &&
              queue_only.combined_trigger_source == "QUEUE_ONLY",
          "saturated queue-only carrier opens capture when delivery is flat");

  auto both_bins = trigger_bins;
  for (auto& bin : both_bins) {
    const double center = 0.5 * (bin.time_start_s + bin.time_end_s);
    const double input_time = center - signature.rtprop_s;
    const double queue_wave = input_time >= 0.0
        ? std::sin(2.0 * kPi * input_time / signature.period_s) : 0.0;
    bin.qdelay_s = 0.00035 + 0.00015 * queue_wave;
    bin.latest_rtt_s = signature.rtprop_s + bin.qdelay_s;
    bin.rtt_valid = true;
  }
  const auto both = AnalyzeTriggerCycle(
      config, signature, both_bins, 3,
      EventWindowState::kIdleListen, true);
  require(both.trigger_pass && both.delivery_trigger_pass &&
              both.queue_trigger_pass &&
              both.combined_trigger_source == "BOTH",
          "delivery and queue channels combine as BOTH");

  auto hard_safety_bins = queue_only_bins;
  for (auto& bin : hard_safety_bins) {
    bin.qdelay_s = 0.004;
    bin.latest_rtt_s = signature.rtprop_s + bin.qdelay_s;
  }
  const auto hard_safety = AnalyzeTriggerCycle(
      config, signature, hard_safety_bins, 3,
      EventWindowState::kIdleListen, true);
  require(!hard_safety.trigger_pass && hard_safety.hard_safety &&
              hard_safety.combined_trigger_source == "HARD_SAFETY_ONLY",
          "deep queue bypasses carrier scoring and enters hard safety only");
  const auto ack_aggregated = AnalyzeTriggerCycle(
      config, signature, trigger_bins, 3,
      EventWindowState::kIdleListen, true);
  require(ack_aggregated.trigger_pass,
          "fixed-bin trigger is invariant to ACK aggregation");

  auto false_period_bins = trigger_bins;
  for (auto& bin : false_period_bins) {
    bin.actual_send_bps = bin.native_pacing_bps;
    bin.sent_bytes = static_cast<uint64_t>(
        bin.native_pacing_bps * (bin.time_end_s - bin.time_start_s) / 8.0);
  }
  const auto false_period = AnalyzeTriggerCycle(
      config, signature, false_period_bins, 3,
      EventWindowState::kIdleListen, true);
  require(!false_period.trigger_pass &&
              !false_period.actual_input_measurable,
          "same-frequency output peak without actual input does not trigger");

  auto weak_bins = trigger_bins;
  const double weak_amplitude_bps = 8.0 * kProbeMssBytes /
                                    signature.period_s;
  for (auto& bin : weak_bins) {
    const double output_center = 0.5 * (bin.time_start_s + bin.time_end_s);
    const double input_time = output_center - signature.rtprop_s;
    const double z = input_time >= 0.0
        ? ProbeWaveform(signature, input_time) : 0.0;
    bin.delivery_rate_bps = 80e6 + weak_amplitude_bps * z;
    bin.acked_bytes = static_cast<uint64_t>(
        bin.delivery_rate_bps * (bin.time_end_s - bin.time_start_s) / 8.0);
  }
  const auto weak = AnalyzeTriggerCycle(
      config, signature, weak_bins, 3,
      EventWindowState::kIdleListen, true);
  require(!weak.trigger_pass && weak.weak_periodic_response &&
              weak.spectral_prominence >= 2.0 &&
              weak.delivery_response_bytes < 4.0 * kProbeMssBytes,
          "high-prominence one-MSS response extends period before capture");

  auto mismatch_bins = trigger_bins;
  for (auto& bin : mismatch_bins) {
    const double center = 0.5 * (bin.time_start_s + bin.time_end_s);
    bin.delivery_rate_bps = 80e6 + 4e6 * std::sin(
        2.0 * kPi * center / (1.40 * signature.period_s));
    bin.acked_bytes = static_cast<uint64_t>(
        bin.delivery_rate_bps * (bin.time_end_s - bin.time_start_s) / 8.0);
  }
  const auto mismatch = AnalyzeTriggerCycle(
      config, signature, mismatch_bins, 3,
      EventWindowState::kIdleListen, true);
  require(!mismatch.trigger_pass,
          "period-mismatched delivery response does not trigger");

  auto same_frequency_noise_bins = trigger_bins;
  for (auto& bin : same_frequency_noise_bins) {
    const double center = 0.5 * (bin.time_start_s + bin.time_end_s);
    const int64_t cycle = static_cast<int64_t>(
        std::floor(center / signature.period_s));
    const double sign = cycle % 2 == 0 ? 1.0 : -1.0;
    bin.delivery_rate_bps = 80e6 + sign * 4e6 * std::sin(
        2.0 * kPi * center / signature.period_s + 0.8);
    bin.acked_bytes = static_cast<uint64_t>(
        bin.delivery_rate_bps * (bin.time_end_s - bin.time_start_s) / 8.0);
  }
  const auto same_frequency_noise = AnalyzeTriggerCycle(
      config, signature, same_frequency_noise_bins, 3,
      EventWindowState::kIdleListen, true);
  require(!same_frequency_noise.trigger_pass,
          "phase-inconsistent same-frequency noise does not trigger");

  auto capacity_step_bins = trigger_bins;
  const double step_time = 3.5 * signature.period_s;
  for (auto& bin : capacity_step_bins) {
    const double center = 0.5 * (bin.time_start_s + bin.time_end_s);
    bin.delivery_rate_bps = center < step_time ? 70e6 : 90e6;
    bin.acked_bytes = static_cast<uint64_t>(
        bin.delivery_rate_bps * (bin.time_end_s - bin.time_start_s) / 8.0);
  }
  const auto capacity_step = AnalyzeTriggerCycle(
      config, signature, capacity_step_bins, 3,
      EventWindowState::kIdleListen, true);
  require(!capacity_step.trigger_pass,
          "capacity step without causal carrier response does not trigger");

  auto shared_noise_bins = trigger_bins;
  for (auto& bin : shared_noise_bins) {
    const double center = 0.5 * (bin.time_start_s + bin.time_end_s);
    bin.delivery_rate_bps += 1.5e6 * std::sin(
        2.0 * kPi * 1.7 * center / signature.period_s + 0.3);
    bin.acked_bytes = static_cast<uint64_t>(
        bin.delivery_rate_bps * (bin.time_end_s - bin.time_start_s) / 8.0);
  }
  const auto shared_noise = AnalyzeTriggerCycle(
      config, signature, shared_noise_bins, 3,
      EventWindowState::kIdleListen, true);
  require(shared_noise.trigger_pass,
          "correct carrier survives different-frequency shared fluctuation");

  const int trigger_true_positive =
      (trigger.trigger_pass ? 1 : 0) +
      (ack_aggregated.trigger_pass ? 1 : 0) +
      (shared_noise.trigger_pass ? 1 : 0);
  const int trigger_false_negative = 3 - trigger_true_positive;
  const std::array<bool, 5> negative_trigger_results{{
      false_period.trigger_pass, weak.trigger_pass, mismatch.trigger_pass,
      same_frequency_noise.trigger_pass, capacity_step.trigger_pass}};
  const int trigger_false_positive = static_cast<int>(std::count(
      negative_trigger_results.begin(), negative_trigger_results.end(), true));
  const int trigger_true_negative = 5 - trigger_false_positive;
  const double trigger_precision = static_cast<double>(trigger_true_positive) /
      std::max(1, trigger_true_positive + trigger_false_positive);
  const double trigger_recall = static_cast<double>(trigger_true_positive) / 3.0;
  const double trigger_false_rate = static_cast<double>(trigger_false_positive) /
      std::max(1, trigger_false_positive + trigger_true_negative);
  os << "trigger_suite,tp=" << trigger_true_positive
     << ",fn=" << trigger_false_negative
     << ",tn=" << trigger_true_negative
     << ",fp=" << trigger_false_positive
     << ",precision=" << trigger_precision
     << ",recall=" << trigger_recall
     << ",false_rate=" << trigger_false_rate
     << ",median_latency_cycles=1\n";
  require(trigger_precision >= 0.90 && trigger_recall >= 0.80 &&
              trigger_false_rate <= 0.05,
          "controlled trigger precision, recall, and false-rate thresholds");

  auto biased_bins = trigger_bins;
  const int64_t post_trigger_first = 4 * config.phase_bins_per_cycle;
  for (size_t i = static_cast<size_t>(post_trigger_first);
       i < biased_bins.size(); ++i) {
    biased_bins[i].delivery_rate_bps = 80e6;
    biased_bins[i].acked_bytes = static_cast<uint64_t>(
        80e6 * (biased_bins[i].time_end_s - biased_bins[i].time_start_s) /
        8.0);
  }
  auto post_trigger = AnalyzeBlock(
      config, signature, biased_bins, post_trigger_first,
      4 * config.phase_bins_per_cycle, 91,
      std::numeric_limits<double>::quiet_NaN());
  post_trigger.trigger_cycle_id = 3;
  post_trigger.trigger_cycle_excluded_from_score = true;
  require(!post_trigger.candidate.valid && post_trigger.target_score < 0.70,
          "high trigger cycle cannot leak into post-trigger score or candidate");

  require(ideal.queue_band_score > underload.queue_band_score &&
              ideal.queue_band_score > overload.queue_band_score,
          "shallow reserve band scores above empty and excessive queue");
  require(config.tracking_stride_cycles == 0.5 &&
              config.control_decision_stride_cycles >= 1.0 &&
              config.trusted_independent_stride_cycles >= 4 &&
              config.bad_cycles_to_pause == 2,
          "dense tracking keeps half-cycle diagnostics and full-cycle control");

  FbbrCruiseConsensusResult history_consensus;
  history_consensus.valid = true;
  history_consensus.confidence = 0.90;
  history_consensus.robust_cv = 0.03;
  history_consensus.raw_candidate_bps = 25e6;
  auto history = StabilizeHistory(config, history_consensus, false, 0.0, false);
  require(history.valid && history.action == "INITIAL_PUBLISH",
          "history initial publish");
  history_consensus.raw_candidate_bps = 25.5e6;
  history = StabilizeHistory(config, history_consensus, true, 25e6, false);
  require(history.valid && history.action == "LOG_EWMA", "history small EWMA");
  history_consensus.raw_candidate_bps = 35e6;
  history_consensus.confidence = 0.80;
  history = StabilizeHistory(config, history_consensus, true, 25e6, false);
  require(!history.valid, "low-confidence large change rejected");
  history_consensus.confidence = 0.95;
  history = StabilizeHistory(config, history_consensus, true, 25e6, true);
  require(history.valid && history.action == "LARGE_CHANGE_ACCEPT",
          "high-confidence large change accepted with native evidence");

  // Algorithm-identity regression: the legacy diagnostic is coverage-gated
  // here, so an invalid app-limited trace cannot become a high-confidence
  // legacy publication.  For the ideal trace both paths retain a monotone
  // response-strength relation without requiring equal numeric scores.
  const double legacy_app_confidence =
      app.phase_bin_coverage * app.non_app_limited_fraction;
  const double legacy_ideal_confidence =
      ideal.phase_bin_coverage * std::max(ideal.response.delivery_gain,
                                          ideal.response.queue_storage_gain);
  require(legacy_app_confidence < config.min_measurement_confidence,
          "identity regression rejects legacy high confidence when F-BBR invalid");
  require(legacy_ideal_confidence >= 0.50 * legacy_app_confidence,
          "identity regression keeps near-optimal legacy score monotone");

  os << "# F-BBR search-controller deterministic tests\n";
  FBBRFrequencySearchConfig control_config;
  control_config.min_control_windows = 2;
  auto make_control_window = [](
      FbbrOperatingPointClassification classification,
      double confidence,
      double raw_score,
      double direction) {
    FbbrOperatingPointBlockResult window;
    window.classification = classification;
    window.measurement_confidence = confidence;
    window.raw_optimality_score = raw_score;
    window.optimality_score = confidence * raw_score;
    window.direction_score = direction;
    window.underload_evidence = direction > 0.0 ? 0.90 : 0.10;
    window.overload_evidence = direction < 0.0 ? 0.90 : 0.10;
    window.realized_amplitude_ratio = 1.0;
    window.input_cycle_coherence = 1.0;
    window.input_carrier_snr = 20.0;
    window.drain_ratio = 1.0;
    window.rtprop_confidence = 1.0;
    window.target_score = confidence * raw_score;
    window.lockable_score = true;
    window.independent_for_control = true;
    window.independent_for_trusted = true;
    window.trigger_cycle_excluded_from_score = true;
    window.q_reserve_low_s = 0.0002;
    window.q_reserve_high_s = 0.0005;
    window.q_peak_cap_s = 0.001;
    window.q_floor_s = 0.00035;
    window.q95_s = 0.0008;
    return window;
  };

  auto search = FBBRSearchController::Initialize(20e6, 0.05, 0.08);
  auto phase_i = make_control_window(
      FbbrOperatingPointClassification::kUnderload, 0.90, 0.20, 0.80);
  auto decision = FBBRSearchController::Decide(
      control_config, phase_i, 20e6, 0.01, &search);
  require(std::abs(decision.applied_next_baseline_bps - 20e6) < 1.0,
          "first UNDERLOAD window holds for confirmation");
  decision = FBBRSearchController::Decide(
      control_config, phase_i, 20e6, 0.01, &search);
  require(decision.applied_next_baseline_bps > 20e6 &&
              decision.log_step <= control_config.ordinary_up_step_max,
          "second UNDERLOAD window raises baseline within 8 percent");

  search = FBBRSearchController::Initialize(25e6, 0.05, 0.08);
  auto ideal_control = make_control_window(
      FbbrOperatingPointClassification::kNearOptimal, 0.90, 0.80, 0.0);
  ideal_control.candidate.valid = true;
  ideal_control.candidate.bandwidth_bps = 25e6;
  ideal_control.candidate.robust_cv = 0.01;
  ideal_control.event_window_id = 1;
  decision = FBBRSearchController::Decide(
      control_config, ideal_control, 25e6, 0.01, &search);
  ideal_control.event_window_id = 2;
  require(decision.lock_candidate && !decision.locked,
          "first ideal window enters LOCK_CANDIDATE");
  decision = FBBRSearchController::Decide(
      control_config, ideal_control, 25e6, 0.01, &search);
  require(decision.locked && decision.trusted_bw_published &&
              std::abs(decision.trusted_bw_bps - 25e6) / 25e6 <= 0.02,
          "two ideal windows LOCK and publish stable trusted_bw");

  search = FBBRSearchController::Initialize(25e6, 0.05, 0.08);
  auto overlap_window = ideal_control;
  overlap_window.event_window_id = 3;
  overlap_window.independent_for_control = false;
  overlap_window.independent_for_trusted = false;
  decision = FBBRSearchController::Decide(
      control_config, overlap_window, 25e6, 0.01, &search);
  require(decision.update_reason == "OVERLAP_DIAGNOSTIC_HOLD" &&
              search.trusted_candidates_bps.empty() &&
              !decision.trusted_bw_published,
          "overlapping half-cycle windows are diagnostic, not trusted evidence");

  search = FBBRSearchController::Initialize(25e6, 0.05, 0.08);
  auto track_control = make_control_window(
      FbbrOperatingPointClassification::kNearOptimal, 0.70, 0.65, 0.50);
  decision = FBBRSearchController::Decide(
      control_config, track_control, 25e6, 0.01, &search);
  require(search.state == FBBRSearchState::kTrack &&
              std::abs(decision.log_step) <= control_config.track_step_max,
          "TRACK update is bounded by two percent");

  search = FBBRSearchController::Initialize(25.5e6, 0.05, 0.08);
  auto queued = make_control_window(
      FbbrOperatingPointClassification::kQueuedOverload,
      0.90, 0.20, -0.80);
  queued.q_floor_s = 0.002;
  queued.q_reserve_low_s = 0.0002;
  queued.q_reserve_high_s = 0.0005;
  queued.q_peak_cap_s = 0.001;
  queued.q95_s = 0.0012;
  queued.queue_band_error = -1.0;
  queued.drain_ratio = 0.20;
  queued.delivery_median_bps = 25e6;
  decision = FBBRSearchController::Decide(
      control_config, queued, 25.5e6, 0.01, &search);
  require(std::abs(decision.applied_next_baseline_bps - 25.5e6) < 1.0 &&
              decision.update_reason == "QUEUE_SERVO_DRAIN_BASELINE_HOLD" &&
              !decision.locked &&
              search.state == FBBRSearchState::kDrain,
          "standing queue drains through servo without duplicate slow-loop cut");

  search = FBBRSearchController::Initialize(30e6, 0.05, 0.08);
  auto servo_transient = phase_i;
  servo_transient.queue_servo_factor_mean = 0.95;
  servo_transient.queue_servo_transition_cycles = 0;
  decision = FBBRSearchController::Decide(
      control_config, servo_transient, 30e6, 0.01, &search);
  require(std::abs(decision.applied_next_baseline_bps - 30e6) < 1.0 &&
              decision.update_reason ==
                  "QUEUE_SERVO_TRANSIENT_BASELINE_HOLD",
          "slow search does not chase a temporary queue-servo correction");

  search = FBBRSearchController::Initialize(30e6, 0.05, 0.08);
  auto saturated = queued;
  saturated.classification =
      FbbrOperatingPointClassification::kBufferSaturated;
  saturated.loss_ratio = 0.03;
  decision = FBBRSearchController::Decide(
      control_config, saturated, 30e6, 0.01, &search);
  require(decision.hard_loss_abort &&
              std::abs(decision.applied_next_baseline_bps - 30e6) < 1.0 &&
              decision.update_reason == "HARD_CONGESTION_QUEUE_SERVO",
          "3 percent loss delegates immediate 0.70 drain to queue servo");
  auto recovered = phase_i;
  recovered.loss_ratio = 0.0;
  recovered.ecn_ratio = 0.0;
  decision = FBBRSearchController::Decide(
      control_config, recovered, 25e6, 0.01, &search);
  require(search.search_active &&
              search.state != FBBRSearchState::kEmergencyDrain,
          "hard congestion recovery automatically resumes frequency search");

  search = FBBRSearchController::Initialize(20e6, 0.05, 0.08);
  decision = FBBRSearchController::Decide(
      control_config, phase_i, 20e6, 0.01, &search);
  search.current_search_bps = 30e6;
  search.pending_search_bps = 30e6;
  queued.frequency_direction = -0.80;
  decision = FBBRSearchController::Decide(
      control_config, queued, 30e6, 0.01, &search);
  require(search.underload_bound_valid && search.overload_bound_valid &&
              std::abs(decision.applied_next_baseline_bps -
                       std::sqrt(20e6 * 30e6)) < 1.0,
          "UNDERLOAD/OVERLOAD bracket uses geometric midpoint");

  os << "# F-BBR RTT queue-reserve servo deterministic tests\n";
  FBBRQueueServoStateData servo_state;
  FBBRQueueServoInput servo_input;
  servo_input.search_baseline_bps = 100e6;
  servo_input.delivery_median_bps = 100e6;
  servo_input.rtprop_s = 0.010;
  servo_input.samples_sufficient = true;
  servo_input.flow_backlogged = true;
  servo_input.q_floor_s = 0.010;
  servo_input.q_median_s = 0.010;
  servo_input.q_peak_s = 0.011;
  double simulated_queue_s = servo_input.q_floor_s;
  bool drained_continuously = true;
  FBBRQueueServoDecision servo_decision;
  for (int rtt = 0; rtt < 10 &&
       simulated_queue_s > control_config.q_reserve_high_bdp *
                               servo_input.rtprop_s; ++rtt) {
    servo_input.q_floor_s = simulated_queue_s;
    servo_input.q_median_s = simulated_queue_s;
    servo_input.q_peak_s = simulated_queue_s + 0.0001;
    servo_decision = FBBRQueueReserveServo::Update(
        control_config, servo_input, &servo_state);
    drained_continuously = drained_continuously &&
        servo_decision.factor < 1.0 &&
        (servo_decision.state == FBBRQueueServoState::kDrain ||
         servo_decision.state == FBBRQueueServoState::kEmergencyDrain);
    simulated_queue_s = std::max(0.0, simulated_queue_s -
        (1.0 - servo_decision.factor) * servo_input.rtprop_s);
  }
  require(drained_continuously && simulated_queue_s <=
              control_config.q_reserve_high_bdp * servo_input.rtprop_s,
          "one-BDP queue drains continuously into reserve band within ten RTTs");
  servo_input.q_floor_s = 0.00025;
  servo_input.q_median_s = 0.00035;
  servo_input.q_peak_s = 0.00045;
  servo_input.queue_trend_s_per_s = 0.0;
  for (int rtt = 0; rtt < 20; ++rtt) {
    servo_decision = FBBRQueueReserveServo::Update(
        control_config, servo_input, &servo_state);
  }
  require(servo_decision.factor >= 0.98 && servo_decision.factor <= 1.02,
          "servo returns to unity after queue enters target band");

  servo_state = FBBRQueueServoStateData();
  servo_input.q_floor_s = 0.00075;
  servo_input.q_median_s = 0.00080;
  servo_input.q_peak_s = 0.00090;
  servo_input.queue_trend_s_per_s = 0.0;
  servo_input.delivery_median_bps = 100e6;
  servo_decision = FBBRQueueReserveServo::Update(
      control_config, servo_input, &servo_state);
  const double first_moderate_drain_factor = servo_decision.factor;
  servo_decision = FBBRQueueReserveServo::Update(
      control_config, servo_input, &servo_state);
  require(std::abs(servo_decision.factor - first_moderate_drain_factor) < 1e-9,
          "moderate queue correction is a baseline-relative target, not a cumulative cut");

  servo_state = FBBRQueueServoStateData();
  servo_input.q_floor_s = 0.0;
  servo_input.q_median_s = 0.0;
  servo_input.q_peak_s = 0.0;
  servo_input.underload_evidence = true;
  servo_decision = FBBRQueueReserveServo::Update(
      control_config, servo_input, &servo_state);
  require(servo_decision.factor > 1.0 && servo_decision.factor <= 1.02 &&
              !servo_decision.baseline_commit_eligible,
          "low reserve recovery is temporary and bounded by two percent");

  servo_state = FBBRQueueServoStateData();
  servo_input.underload_evidence = false;
  servo_input.q_floor_s = 0.002;
  servo_input.q_median_s = 0.002;
  servo_input.q_peak_s = 0.0022;
  servo_input.sustainable_direction = -0.80;
  for (uint32_t rtt = 0; rtt <
       control_config.queue_servo_commit_min_rtts + 2; ++rtt) {
    servo_decision = FBBRQueueReserveServo::Update(
        control_config, servo_input, &servo_state);
  }
  require(servo_decision.baseline_commit_eligible &&
              servo_decision.baseline_commit_bps >= 99e6 - 1.0 &&
              servo_decision.baseline_commit_bps <= 100e6 &&
              (100e6 - servo_decision.baseline_commit_bps) / 100e6 <= 0.020001,
          "sustained negative direction commits at most two percent and respects delivery floor");

  servo_state = FBBRQueueServoStateData();
  servo_input.loss_ratio = 0.03;
  servo_input.sustainable_direction = 0.0;
  servo_decision = FBBRQueueReserveServo::Update(
      control_config, servo_input, &servo_state);
  require(servo_decision.state == FBBRQueueServoState::kEmergencyDrain &&
              std::abs(servo_decision.factor - 0.70) < 1e-9,
          "three-percent loss enters emergency drain at factor 0.70");
  servo_input.loss_ratio = 0.0;

  search = FBBRSearchController::Initialize(25e6, 0.05, 0.08);
  auto dynamic_window = make_control_window(
      FbbrOperatingPointClassification::kDynamic, 0.90, 0.20, 0.0);
  decision = FBBRSearchController::Decide(
      control_config, dynamic_window, 40e6, 0.01, &search);
  require(std::abs(decision.applied_next_baseline_bps - 25e6) < 1.0,
          "one DYNAMIC window holds baseline");
  decision = FBBRSearchController::Decide(
      control_config, dynamic_window, 40e6, 0.01, &search);
  require(std::abs(decision.applied_next_baseline_bps - 37.5e6) < 1.0 &&
              decision.search_range_exhausted,
          "two DYNAMIC windows reset toward current native within range");

  search = FBBRSearchController::Initialize(25e6, 0.05, 0.08);
  auto unrealized = make_control_window(
      FbbrOperatingPointClassification::kInvalid, 0.0, 0.0, 0.0);
  unrealized.realized_amplitude_ratio = 0.40;
  unrealized.invalid_reason = "actual_input_energy_low";
  decision = FBBRSearchController::Decide(
      control_config, unrealized, 25e6, 0.01, &search);
  require(decision.request_period_increase &&
              std::abs(decision.applied_next_baseline_bps - 25e6) < 1.0,
          "unrealized input holds baseline and requests probe adaptation");
  decision = FBBRSearchController::Decide(
      control_config, unrealized, 25e6, 0.01, &search);
  require(search.state == FBBRSearchState::kPersistentUnresolved &&
              search.search_active &&
              !decision.trusted_bw_published,
          "repeated unrealized input remains persistent without publication");
  for (int cruise = 0; cruise < 5; ++cruise) {
    decision = FBBRSearchController::Decide(
        control_config, unrealized, 25e6, 0.01, &search);
    ++search.unresolved_cruises;
    require(search.search_active && !decision.trusted_bw_published,
            "every unresolved cruise keeps search active");
  }

  search = FBBRSearchController::Initialize(22e6, 0.05, 0.08);
  search.underload_bound_valid = true;
  search.underload_bound_bps = 20e6;
  search.overload_bound_valid = true;
  search.overload_bound_bps = 30e6;
  search.provisional_validation_pending = true;
  decision = FBBRSearchController::Decide(
      control_config, phase_i, 22e6, 0.01, &search);
  require(!search.provisional_validation_pending &&
              std::abs(decision.applied_next_baseline_bps -
                       std::sqrt(20e6 * 30e6)) < 1.0,
          "cross-cruise provisional bracket validates from native anchor");

  search = FBBRSearchController::Initialize(25e6, 0.05, 0.08);
  auto low_rtprop_confidence = ideal_control;
  low_rtprop_confidence.rtprop_confidence = 0.20;
  decision = FBBRSearchController::Decide(
      control_config, low_rtprop_confidence, 25e6, 0.01, &search);
  decision = FBBRSearchController::Decide(
      control_config, low_rtprop_confidence, 25e6, 0.01, &search);
  require(!decision.trusted_bw_published &&
              search.state != FBBRSearchState::kLocked,
          "low RTprop confidence prevents trusted publication");

  FbbrProbeSignature asymmetric_signature;
  asymmetric_signature.period_s = 1.0;
  asymmetric_signature.waveform = "asymmetric_zero_mean";
  double waveform_sum = 0.0;
  double positive_samples = 0.0;
  constexpr int kWaveformSamples = 12000;
  for (int i = 0; i < kWaveformSamples; ++i) {
    const double value = FBBRFrequencySearch::ProbeWaveform(
        asymmetric_signature, (i + 0.5) / kWaveformSamples);
    waveform_sum += value;
    if (value > 0.0) positive_samples += 1.0;
  }
  require(std::abs(waveform_sum / kWaveformSamples) < 1e-6 &&
              std::abs(positive_samples / kWaveformSamples - 0.25) < 1e-3,
          "asymmetric waveform is zero mean with one-quarter positive lobe");

  double previous_ramp = 20e6;
  for (int step = 0; step <= 100; ++step) {
    const double value = FBBRSearchController::RaisedCosineLogRamp(
        20e6, 30e6, step / 100.0);
    require(value + 1e-6 >= previous_ramp,
            "raised-cosine log ramp is monotone");
    previous_ramp = value;
  }
  require(std::abs(FBBRSearchController::RaisedCosineLogRamp(
                       20e6, 30e6, 0.0) - 20e6) < 1.0 &&
              std::abs(FBBRSearchController::RaisedCosineLogRamp(
                       20e6, 30e6, 1.0) - 30e6) < 1.0,
          "raised-cosine ramp preserves exact endpoints");

  os << "RESULT: " << (passed ? "PASS" : "FAIL") << "\n";
  return passed;
}

}  // namespace dqc
