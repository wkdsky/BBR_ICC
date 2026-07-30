#include "fbbr_config_loader.h"

#include "fbbr_sender.h"

#include <cctype>
#include <cstdint>
#include <fstream>
#include <string>

namespace dqc {
namespace {

std::string
Trim(const std::string& value)
{
  size_t begin = 0;
  while (begin < value.size() &&
         std::isspace(static_cast<unsigned char>(value[begin]))) {
    ++begin;
  }
  size_t end = value.size();
  while (end > begin &&
         std::isspace(static_cast<unsigned char>(value[end - 1]))) {
    --end;
  }
  return value.substr(begin, end - begin);
}

std::string
ToLower(std::string value)
{
  for (char& character : value) {
    character = static_cast<char>(
        std::tolower(static_cast<unsigned char>(character)));
  }
  return value;
}

bool
ParseBool(const std::string& value, bool* out)
{
  const std::string normalized = ToLower(Trim(value));
  if (normalized == "true" || normalized == "1" || normalized == "yes" ||
      normalized == "on") {
    *out = true;
    return true;
  }
  if (normalized == "false" || normalized == "0" || normalized == "no" ||
      normalized == "off") {
    *out = false;
    return true;
  }
  return false;
}

bool
ParseDouble(const std::string& value, double* out)
{
  try {
    size_t used = 0;
    const double parsed = std::stod(value, &used);
    if (Trim(value.substr(used)).empty()) {
      *out = parsed;
      return true;
    }
  } catch (...) {
  }
  return false;
}

bool
ParseUint32(const std::string& value, uint32_t* out)
{
  try {
    size_t used = 0;
    const unsigned long parsed = std::stoul(value, &used);
    if (Trim(value.substr(used)).empty()) {
      *out = static_cast<uint32_t>(parsed);
      return true;
    }
  } catch (...) {
  }
  return false;
}

bool
ParseUint64(const std::string& value, uint64_t* out)
{
  try {
    size_t used = 0;
    const unsigned long long parsed = std::stoull(value, &used);
    if (Trim(value.substr(used)).empty()) {
      *out = static_cast<uint64_t>(parsed);
      return true;
    }
  } catch (...) {
  }
  return false;
}

bool
Fail(const std::string& path,
     uint32_t line_no,
     const std::string& message,
     std::string* error)
{
  if (error != nullptr) {
    *error = path + ":" + std::to_string(line_no) + ": " + message;
  }
  return false;
}

bool
SetCommonValue(FBBRConfig* config,
               const std::string& key,
               const std::string& value,
               const std::string& path,
               uint32_t line_no,
               std::string* error)
{
  double double_value = 0.0;
  uint32_t uint32_value = 0;
  uint64_t uint64_value = 0;
  bool bool_value = false;

  if (key.rfind("flow.", 0) == 0) {
    const size_t id_begin = 5;
    const size_t id_end = key.find('.', id_begin);
    if (id_end == std::string::npos) {
      return Fail(path, line_no, "invalid flow key: " + key, error);
    }
    uint32_t flow_id = 0;
    if (!ParseUint32(key.substr(id_begin, id_end - id_begin), &flow_id)) {
      return Fail(path, line_no, "invalid flow id in key: " + key, error);
    }
    FBBRFlowConfig& flow = config->flow[flow_id];
    const std::string field = key.substr(id_end + 1);
    if (field == "modulation_freq_hz") {
      if (!ParseDouble(value, &double_value)) {
        return Fail(path, line_no, "invalid double for " + key, error);
      }
      flow.modulation_freq_hz = double_value;
      flow.has_modulation_freq_hz = true;
      return true;
    }
    if (field == "fixed_amplitude_mbps") {
      if (!ParseDouble(value, &double_value)) {
        return Fail(path, line_no, "invalid double for " + key, error);
      }
      flow.fixed_amplitude_mbps = double_value;
      flow.has_fixed_amplitude_mbps = true;
      return true;
    }
    return Fail(path, line_no, "unknown flow field: " + key, error);
  }

#define SET_DOUBLE(KEY, FIELD)                                                \
  if (key == KEY) {                                                           \
    if (!ParseDouble(value, &double_value)) {                                 \
      return Fail(path, line_no, "invalid double for " + key, error);        \
    }                                                                         \
    config->FIELD = double_value;                                             \
    return true;                                                              \
  }
#define SET_U32(KEY, FIELD)                                                   \
  if (key == KEY) {                                                           \
    if (!ParseUint32(value, &uint32_value)) {                                 \
      return Fail(path, line_no, "invalid uint for " + key, error);          \
    }                                                                         \
    config->FIELD = uint32_value;                                             \
    return true;                                                              \
  }
#define SET_U64(KEY, FIELD)                                                   \
  if (key == KEY) {                                                           \
    if (!ParseUint64(value, &uint64_value)) {                                 \
      return Fail(path, line_no, "invalid uint64 for " + key, error);        \
    }                                                                         \
    config->FIELD = uint64_value;                                             \
    return true;                                                              \
  }
#define SET_BOOL(KEY, FIELD)                                                  \
  if (key == KEY) {                                                           \
    if (!ParseBool(value, &bool_value)) {                                     \
      return Fail(path, line_no, "invalid bool for " + key, error);          \
    }                                                                         \
    config->FIELD = bool_value;                                               \
    return true;                                                              \
  }

  SET_DOUBLE("default_modulation_freq_hz", default_modulation_freq_hz)
  if (key == "default_amplitude_mode") {
    if (!IsValidFBBRAmplitudeMode(value)) {
      return Fail(path, line_no,
                  "invalid FBBR amplitude mode for " + key +
                      "; expected fixed_mbps, a positive Mbps value, "
                      "or 1sr..20sr",
                  error);
    }
    config->default_amplitude_mode = value;
    return true;
  }
  SET_DOUBLE("default_fixed_amplitude_mbps", default_fixed_amplitude_mbps)
  SET_DOUBLE("pacing.minimum_rate_mbps", pacing_minimum_rate_mbps)
  SET_DOUBLE("stability.single_round_exit_threshold",
             stability_single_round_exit_threshold)
  SET_DOUBLE("stability.consecutive_exit_threshold",
             stability_consecutive_exit_threshold)
  SET_U32("stability.stable_rounds", stability_stable_rounds)
  SET_DOUBLE("stability.full_pipe_growth_threshold",
             stability_full_pipe_growth_threshold)
  SET_BOOL("beq.clear_on_cruise_start", beq_clear_on_cruise_start)

  SET_DOUBLE("waveform.initial_window_periods", waveform_initial_window_periods)
  SET_DOUBLE("waveform.extended_window_periods", waveform_extended_window_periods)
  SET_DOUBLE("waveform.max_window_periods", waveform_max_window_periods)
  SET_DOUBLE("waveform.period_tolerance_ratio", waveform_period_tolerance_ratio)
  SET_DOUBLE("waveform.min_periodicity_correlation",
             waveform_min_periodicity_correlation)
  SET_DOUBLE("waveform.min_cycle_coverage_ratio", waveform_min_cycle_coverage_ratio)
  SET_DOUBLE("waveform.masked_min_cycle_coverage_ratio",
             waveform_masked_min_cycle_coverage_ratio)
  SET_DOUBLE("waveform.local_slope_window_period_ratio",
             waveform_local_slope_window_period_ratio)
  SET_DOUBLE("waveform.min_local_slope_window_ms", waveform_min_local_slope_window_ms)
  SET_DOUBLE("waveform.clip_min_duration_ratio", waveform_clip_min_duration_ratio)
  SET_DOUBLE("waveform.clip_min_half_overlap_ratio", waveform_clip_min_half_overlap_ratio)
  SET_DOUBLE("waveform.clip_max_slope_ratio", waveform_clip_max_slope_ratio)
  SET_DOUBLE("waveform.delta_drate_amplitude_ratio",
             waveform_delta_drate_amplitude_ratio)
  SET_DOUBLE("waveform.delta_fallback_baseline_ratio",
             waveform_delta_fallback_baseline_ratio)
  SET_DOUBLE("waveform.plateau_max_level_span_ratio",
             waveform_plateau_max_level_span_ratio)
  SET_DOUBLE("waveform.plateau_extreme_distance_ratio",
             waveform_plateau_extreme_distance_ratio)
  SET_U32("waveform.max_baseline_adjustments", waveform_max_baseline_adjustments)
  SET_DOUBLE("waveform.inconclusive_signal_amplification_factor",
             waveform_inconclusive_signal_amplification_factor)
  SET_DOUBLE("waveform.inconclusive_signal_amplification_max_ratio",
             waveform_inconclusive_signal_amplification_max_ratio)
  SET_DOUBLE("waveform.max_app_limited_sample_ratio",
             waveform_max_app_limited_sample_ratio)
  SET_DOUBLE("waveform.max_interpolation_gap_period_ratio",
             waveform_max_interpolation_gap_period_ratio)
  SET_DOUBLE("goertzel.min_coherent_power_ratio", goertzel_min_coherent_power_ratio)

  SET_DOUBLE("fbbr.regime.long_top_horizontal_duration_ratio",
             fbbr_regime_long_top_horizontal_duration_ratio)
  SET_DOUBLE("fbbr.regime.long_bottom_horizontal_duration_ratio",
             fbbr_regime_long_bottom_horizontal_duration_ratio)
  SET_DOUBLE("fbbr.regime.period_tolerance_ratio", fbbr_regime_period_tolerance_ratio)
  SET_DOUBLE("fbbr.regime.min_periodicity_correlation",
             fbbr_regime_min_periodicity_correlation)

  SET_DOUBLE("waveform.activity.amplitude_noise_multiplier",
             waveform_activity_amplitude_noise_multiplier)
  SET_DOUBLE("waveform.activity.min_level_ratio", waveform_activity_min_level_ratio)
  SET_DOUBLE("waveform.activity.step_noise_multiplier",
             waveform_activity_step_noise_multiplier)
  SET_DOUBLE("waveform.activity.min_normalized_step_slope",
             waveform_activity_min_normalized_step_slope)
  SET_U32("waveform.activity.min_active_steps", waveform_activity_min_active_steps)
  SET_DOUBLE("waveform.activity.min_active_step_ratio",
             waveform_activity_min_active_step_ratio)
  SET_DOUBLE("waveform.activity.min_directional_change_ratio",
             waveform_activity_min_directional_change_ratio)
  SET_DOUBLE("waveform.activity.min_significant_path_ratio",
             waveform_activity_min_significant_path_ratio)
  SET_U32("waveform.activity.min_slope_reversals",
          waveform_activity_min_slope_reversals)

  SET_DOUBLE("waveform.horizontal.continuous_min_duration_ratio",
             waveform_horizontal_continuous_min_duration_ratio)
  SET_DOUBLE("waveform.horizontal.min_valid_coverage_ratio",
             waveform_horizontal_min_valid_coverage_ratio)
  SET_DOUBLE("waveform.horizontal.min_flat_fraction",
             waveform_horizontal_min_flat_fraction)
  SET_DOUBLE("waveform.horizontal.max_local_slope_ratio",
             waveform_horizontal_max_local_slope_ratio)
  SET_DOUBLE("waveform.horizontal.min_side_slope_ratio",
             waveform_horizontal_min_side_slope_ratio)
  SET_DOUBLE("waveform.horizontal.min_boundary_kink_ratio",
             waveform_horizontal_min_boundary_kink_ratio)
  SET_DOUBLE("waveform.horizontal.max_level_span_ratio",
             waveform_horizontal_max_level_span_ratio)
  SET_DOUBLE("waveform.horizontal.max_total_drift_ratio",
             waveform_horizontal_max_total_drift_ratio)
  SET_DOUBLE("waveform.horizontal.min_side_change_ratio",
             waveform_horizontal_min_side_change_ratio)
  SET_DOUBLE("waveform.horizontal.amplitude_noise_multiplier",
             waveform_horizontal_amplitude_noise_multiplier)
  SET_DOUBLE("waveform.horizontal.level_span_noise_multiplier",
             waveform_horizontal_level_span_noise_multiplier)
  SET_DOUBLE("waveform.horizontal.slope_noise_multiplier",
             waveform_horizontal_slope_noise_multiplier)
  SET_DOUBLE("waveform.horizontal.extreme_distance_ratio",
             waveform_horizontal_extreme_distance_ratio)

  SET_DOUBLE("waveform.repeated_clip_max_period_error_ratio",
             waveform_repeated_clip_max_period_error_ratio)
  SET_DOUBLE("waveform.repeated_clip_max_level_delta_ratio",
             waveform_repeated_clip_max_level_delta_ratio)
  SET_DOUBLE("waveform.repeated_clip_contact_level_tolerance_ratio",
             waveform_repeated_clip_contact_level_tolerance_ratio)
  SET_U32("waveform.repeated_clip_min_contact_samples_per_cycle",
          waveform_repeated_clip_min_contact_samples_per_cycle)
  SET_U32("waveform.repeated_clip_min_total_contact_samples",
          waveform_repeated_clip_min_total_contact_samples)
  SET_DOUBLE("waveform.repeated_clip_min_contact_sample_ratio",
             waveform_repeated_clip_min_contact_sample_ratio)
  SET_DOUBLE("waveform.repeated_clip_min_pooled_flat_fraction",
             waveform_repeated_clip_min_pooled_flat_fraction)
  SET_DOUBLE("waveform.repeated_clip_min_verified_boundary_fraction",
             waveform_repeated_clip_min_verified_boundary_fraction)
  SET_DOUBLE("waveform.repeated_clip_min_outside_excursion_ratio",
             waveform_repeated_clip_min_outside_excursion_ratio)
  SET_DOUBLE("waveform.repeated_clip_min_extrapolated_overshoot_ratio",
             waveform_repeated_clip_min_extrapolated_overshoot_ratio)
  SET_DOUBLE("waveform.repeated_clip_merge_gap_ratio",
             waveform_repeated_clip_merge_gap_ratio)
  SET_DOUBLE("waveform.repeated_clip_max_missing_gap_ratio",
             waveform_repeated_clip_max_missing_gap_ratio)

  SET_DOUBLE("waveform.shoulder.min_half_overlap_ratio",
             waveform_shoulder_min_half_overlap_ratio)
  SET_DOUBLE("waveform.shoulder.min_side_change_ratio",
             waveform_shoulder_min_side_change_ratio)
  SET_DOUBLE("waveform.shoulder.max_residual_cycle_period_error_ratio",
             waveform_shoulder_max_residual_cycle_period_error_ratio)
  SET_DOUBLE("waveform.shoulder.min_residual_cycle_leg_duration_ratio",
             waveform_shoulder_min_residual_cycle_leg_duration_ratio)

  SET_DOUBLE("waveform.middle.min_duration_ratio", waveform_middle_min_duration_ratio)
  SET_DOUBLE("waveform.middle.max_duration_ratio", waveform_middle_max_duration_ratio)
  SET_DOUBLE("waveform.middle.context_duration_ratio",
             waveform_middle_context_duration_ratio)
  SET_DOUBLE("waveform.middle.min_trend_slope_ratio",
             waveform_middle_min_trend_slope_ratio)
  SET_DOUBLE("waveform.middle.max_context_slope_delta_ratio",
             waveform_middle_max_context_slope_delta_ratio)
  SET_DOUBLE("waveform.middle.min_slope_mismatch_ratio",
             waveform_middle_min_slope_mismatch_ratio)
  SET_DOUBLE("waveform.middle.min_mismatching_sample_ratio",
             waveform_middle_min_mismatching_sample_ratio)
  SET_U32("waveform.middle.min_mismatching_samples",
          waveform_middle_min_mismatching_samples)
  SET_U32("waveform.middle.min_consecutive_mismatching_samples",
          waveform_middle_min_consecutive_mismatching_samples)
  SET_DOUBLE("waveform.middle.min_bridge_deviation_ratio",
             waveform_middle_min_bridge_deviation_ratio)
  SET_DOUBLE("waveform.middle.noise_multiplier", waveform_middle_noise_multiplier)
  SET_DOUBLE("waveform.middle.max_mask_ratio_per_cycle",
             waveform_middle_max_mask_ratio_per_cycle)

  if (key == "trace.gate_trace_mode") {
    config->trace_gate_trace_mode = value;
    return true;
  }
  SET_U64("trace.gate_trace_sample_interval_us", trace_gate_trace_sample_interval_us)
  SET_BOOL("trace.enable_cruise_window_trace", trace_enable_cruise_window_trace)
  SET_BOOL("trace.enable_beq_selection_trace", trace_enable_beq_selection_trace)

#undef SET_DOUBLE
#undef SET_U32
#undef SET_U64
#undef SET_BOOL

  return Fail(path, line_no, "unknown key: " + key, error);
}

}  // namespace

bool
LoadFBBRConfigFile(const std::string& path,
                   FBBRConfig* config,
                   std::string* error)
{
  if (config == nullptr) {
    if (error != nullptr) {
      *error = "null FBBRConfig destination";
    }
    return false;
  }
  if (path.empty()) {
    if (error != nullptr) {
      *error = "empty FBBR configuration path";
    }
    return false;
  }

  std::ifstream input(path.c_str());
  if (!input.is_open()) {
    if (error != nullptr) {
      *error = "unable to open FBBR configuration: " + path;
    }
    return false;
  }

  FBBRConfig loaded;
  std::string line;
  uint32_t line_no = 0;
  while (std::getline(input, line)) {
    ++line_no;
    const size_t comment = line.find('#');
    if (comment != std::string::npos) {
      line = line.substr(0, comment);
    }
    line = Trim(line);
    if (line.empty()) {
      continue;
    }
    const size_t equals = line.find('=');
    if (equals == std::string::npos) {
      return Fail(path, line_no, "expected key=value", error);
    }
    const std::string key = Trim(line.substr(0, equals));
    const std::string value = Trim(line.substr(equals + 1));
    if (key.empty()) {
      return Fail(path, line_no, "empty key", error);
    }
    if (!SetCommonValue(&loaded, key, value, path, line_no, error)) {
      return false;
    }
  }

  *config = loaded;
  return true;
}

}  // namespace dqc
