/**
 * Generic N-flow DQC P2P dumbbell scenario.
 *
 * Topology:
 *
 *   sender[0] ----\                         /---- receiver[0]
 *   sender[1] -----+---- left ---- right ---+----- receiver[1]
 *   ...       ----/      switch    switch    \---- ...
 *
 * Flow i runs from sender[i] to receiver[i].  All flows share the single
 * left-switch -> right-switch bottleneck link.  The bottleneck egress queue is
 * constrained by the configured switch buffer; endpoint/access queues are
 * configured with a large byte limit.
 */

#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/dqc-module.h"
#include "ns3/fbbr_sender.h"
#include "ns3/internet-module.h"
#include "ns3/ipv4-global-routing-helper.h"
#include "ns3/network-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/send_packet_manager.h"
#include "ns3/traffic-control-module.h"

#include "queue_occupancy_trace_helper.h"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cstdio>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <random>
#include <sstream>
#include <set>
#include <string>
#include <vector>

using namespace ns3;
using namespace dqc;

NS_LOG_COMPONENT_DEFINE("generic-p2p-switch-flows");

namespace {

const char kDefaultFBBRConfig[] =
    "/home/wkd/FreqBBR/NS3.27/examples/CCconfig/fbbr_default.conf";
const uint32_t kDefaultEndpointQueueBytes = 1024u * 1024u * 1024u;

struct AlgorithmSpec
{
    dqc::CongestionControlType cc;
    std::string display_name;
    bool ecn;
    bool is_fbbr;
};

struct RateStep
{
    double time_s;
    uint64_t rate_bps;
};

struct ScenarioConfig
{
    uint32_t n_flows;
    double sim_time_s;
    std::string access_rate;
    std::string service_rate;
    uint64_t access_rate_bps;
    uint64_t service_rate_bps;
    double access_delay_ms;
    double service_delay_ms;
    uint32_t switch_buffer_bytes;
    double switch_buffer_bdp;
    uint32_t endpoint_queue_bytes;
    uint64_t flow_size_bytes;
    int64_t process_interval_us;
    uint32_t goodput_interval_ms;
    bool use_engine_timer;
    bool enable_trace;
    bool enable_heavy_trace;
    bool enable_queue_trace;
    uint32_t emulated_connections;
    uint32_t data_generator_batch;
    uint32_t packet_size_variation_bytes;
    uint32_t stream_buffer_bytes;
    bool enable_convergence_gate_trace;
    bool enable_convergence_gate_control;
    std::string gate_trace_mode;
    uint64_t gate_trace_sample_interval_us;
    std::string trace_path;
    std::string trace_name;
    uint32_t seed;
    uint32_t run_id;
    bool emit_run_meta;
    bool emit_bottleneck_queue_trace;
    uint32_t queue_sample_interval_us;
    bool enable_equivalence_audit;
    double ack_timing_jitter_us;
    double ack_jitter_interval_ms;
    std::vector<RateStep> background_schedule;
    std::vector<RateStep> capacity_schedule;
};

struct FlowConfig
{
    uint32_t index;
    AlgorithmSpec algo;
    double start_time_s;
    double stop_time_s;
    uint64_t sender_rate_cap_bps;
    std::vector<RateStep> rate_cap_schedule;
    std::string config_path;
    dqc::FBBRConfig fbbr_config;
};

std::string Trim(const std::string& in);
bool ParseDoubleValue(const std::string& value, double* out);
uint64_t ParseRateBps(const std::string& value, const std::string& name);

std::vector<RateStep>
ParseRateSchedule(const std::string& input,
                  uint64_t default_rate_bps,
                  const std::string& name)
{
    std::vector<RateStep> steps;
    if (Trim(input).empty())
    {
        steps.push_back({0.0, default_rate_bps});
        return steps;
    }
    std::stringstream stream(input);
    std::string token;
    while (std::getline(stream, token, ','))
    {
        const size_t colon = token.find(':');
        NS_ABORT_MSG_IF(colon == std::string::npos,
                        name << " entries must use time:rate: " << token);
        double time_s = 0.0;
        NS_ABORT_MSG_IF(!ParseDoubleValue(Trim(token.substr(0, colon)), &time_s) ||
                            time_s < 0.0,
                        "invalid " << name << " time: " << token);
        const uint64_t rate_bps =
            ParseRateBps(Trim(token.substr(colon + 1)), name);
        steps.push_back({time_s, rate_bps});
    }
    std::sort(steps.begin(), steps.end(),
              [](const RateStep& a, const RateStep& b) {
                  return a.time_s < b.time_s;
              });
    if (steps.empty() || steps.front().time_s > 0.0)
    {
        steps.insert(steps.begin(), {0.0, default_rate_bps});
    }
    return steps;
}

uint64_t
RateAt(const std::vector<RateStep>& steps, double time_s)
{
    uint64_t value = steps.empty() ? 0 : steps.front().rate_bps;
    for (const auto& step : steps)
    {
        if (step.time_s > time_s + 1e-12) break;
        value = step.rate_bps;
    }
    return value;
}

std::string
Trim(const std::string& in)
{
    size_t begin = 0;
    while (begin < in.size() &&
           std::isspace(static_cast<unsigned char>(in[begin])))
    {
        ++begin;
    }
    size_t end = in.size();
    while (end > begin &&
           std::isspace(static_cast<unsigned char>(in[end - 1])))
    {
        --end;
    }
    return in.substr(begin, end - begin);
}

std::string
ToLower(std::string value)
{
    for (size_t i = 0; i < value.size(); ++i)
    {
        value[i] =
            static_cast<char>(std::tolower(static_cast<unsigned char>(value[i])));
    }
    return value;
}

bool
ParseBoolValue(const std::string& value, bool* out)
{
    const std::string lower = ToLower(Trim(value));
    if (lower == "true" || lower == "1" || lower == "yes" || lower == "on")
    {
        *out = true;
        return true;
    }
    if (lower == "false" || lower == "0" || lower == "no" || lower == "off")
    {
        *out = false;
        return true;
    }
    return false;
}

bool
ParseDoubleValue(const std::string& value, double* out)
{
    try
    {
        size_t used = 0;
        const double parsed = std::stod(value, &used);
        if (Trim(value.substr(used)).empty())
        {
            *out = parsed;
            return true;
        }
    }
    catch (...)
    {
    }
    return false;
}

bool
ParseUintValue(const std::string& value, uint32_t* out)
{
    try
    {
        size_t used = 0;
        const unsigned long parsed = std::stoul(value, &used);
        if (Trim(value.substr(used)).empty())
        {
            *out = static_cast<uint32_t>(parsed);
            return true;
        }
    }
    catch (...)
    {
    }
    return false;
}

bool
ParseUint64Value(const std::string& value, uint64_t* out)
{
    try
    {
        size_t used = 0;
        const unsigned long long parsed = std::stoull(value, &used);
        if (Trim(value.substr(used)).empty())
        {
            *out = static_cast<uint64_t>(parsed);
            return true;
        }
    }
    catch (...)
    {
    }
    return false;
}

void
WarnConfigLine(const std::string& path,
               uint32_t line_no,
               const std::string& message)
{
    std::cerr << "[fbbr-config warning] " << path << ":" << line_no << ": "
              << message << std::endl;
}

template <typename Config>
bool
SetCommonFrequencyConfigValue(Config* config,
                              const std::string& key,
                              const std::string& value,
                              const std::string& path,
                              uint32_t line_no)
{
    double d = 0.0;
    uint32_t u32 = 0;
    bool b = false;
    uint64_t u64 = 0;

    if (key.rfind("flow.", 0) == 0)
    {
        const size_t id_begin = 5;
        const size_t id_end = key.find('.', id_begin);
        if (id_end == std::string::npos)
        {
            WarnConfigLine(path, line_no, "invalid flow key: " + key);
            return false;
        }
        uint32_t flow_id = 0;
        if (!ParseUintValue(key.substr(id_begin, id_end - id_begin), &flow_id))
        {
            WarnConfigLine(path, line_no, "invalid flow id in key: " + key);
            return false;
        }
        const std::string field = key.substr(id_end + 1);
        auto& flow = config->flow[flow_id];
        if (field == "modulation_freq_hz")
        {
            if (!ParseDoubleValue(value, &d))
            {
                WarnConfigLine(path, line_no, "invalid double for " + key);
                return false;
            }
            flow.modulation_freq_hz = d;
            flow.has_modulation_freq_hz = true;
            return true;
        }
        if (field == "fixed_amplitude_mbps")
        {
            if (!ParseDoubleValue(value, &d))
            {
                WarnConfigLine(path, line_no, "invalid double for " + key);
                return false;
            }
            flow.fixed_amplitude_mbps = d;
            flow.has_fixed_amplitude_mbps = true;
            return true;
        }
        WarnConfigLine(path, line_no, "unknown flow field: " + key);
        return false;
    }

#define SET_DOUBLE(KEY, FIELD)                                                \
    if (key == KEY)                                                           \
    {                                                                         \
        if (!ParseDoubleValue(value, &d))                                      \
        {                                                                     \
            WarnConfigLine(path, line_no, "invalid double for " + key);       \
            return false;                                                     \
        }                                                                     \
        config->FIELD = d;                                                    \
        return true;                                                          \
    }
#define SET_U32(KEY, FIELD)                                                   \
    if (key == KEY)                                                           \
    {                                                                         \
        if (!ParseUintValue(value, &u32))                                      \
        {                                                                     \
            WarnConfigLine(path, line_no, "invalid uint for " + key);         \
            return false;                                                     \
        }                                                                     \
        config->FIELD = u32;                                                  \
        return true;                                                          \
    }
#define SET_U64(KEY, FIELD)                                                   \
    if (key == KEY)                                                           \
    {                                                                         \
        if (!ParseUint64Value(value, &u64))                                    \
        {                                                                     \
            WarnConfigLine(path, line_no, "invalid uint64 for " + key);       \
            return false;                                                     \
        }                                                                     \
        config->FIELD = u64;                                                  \
        return true;                                                          \
    }
#define SET_BOOL(KEY, FIELD)                                                  \
    if (key == KEY)                                                           \
    {                                                                         \
        if (!ParseBoolValue(value, &b))                                        \
        {                                                                     \
            WarnConfigLine(path, line_no, "invalid bool for " + key);         \
            return false;                                                     \
        }                                                                     \
        config->FIELD = b;                                                    \
        return true;                                                          \
    }

    SET_DOUBLE("default_modulation_freq_hz", default_modulation_freq_hz)
    if (key == "default_amplitude_mode")
    {
        config->default_amplitude_mode = value;
        return true;
    }
    SET_DOUBLE("default_fixed_amplitude_mbps", default_fixed_amplitude_mbps)
    SET_DOUBLE("stability.single_round_exit_threshold",
               stability_single_round_exit_threshold)
    SET_DOUBLE("stability.consecutive_exit_threshold",
               stability_consecutive_exit_threshold)
    SET_U32("stability.stable_rounds", stability_stable_rounds)
    SET_DOUBLE("stability.full_pipe_growth_threshold",
               stability_full_pipe_growth_threshold)
    SET_BOOL("trusted_bw.clear_on_cruise_start",
             trusted_bw_clear_on_cruise_start)
    if (key == "trace.gate_trace_mode")
    {
        config->trace_gate_trace_mode = value;
        return true;
    }
    SET_U64("trace.gate_trace_sample_interval_us",
            trace_gate_trace_sample_interval_us)
    SET_BOOL("trace.enable_cruise_window_trace",
             trace_enable_cruise_window_trace)
    SET_BOOL("trace.enable_trusted_bw_selection_trace",
             trace_enable_trusted_bw_selection_trace)

#undef SET_DOUBLE
#undef SET_U32
#undef SET_U64
#undef SET_BOOL

    WarnConfigLine(path, line_no, "unknown key: " + key);
    return false;
}

bool
SetFBBRConfigValue(dqc::FBBRConfig* config,
                      const std::string& key,
                      const std::string& value,
                      const std::string& path,
                      uint32_t line_no)
{
    double d = 0.0;
    uint32_t u32 = 0;
    bool b = false;

    if (key == "waveform.recv_signal_mode")
    {
        config->waveform_recv_signal_mode = value;
        return true;
    }
    if (key == "waveform.negative_half_first")
    {
        if (!ParseBoolValue(value, &b))
        {
            WarnConfigLine(path, line_no, "invalid bool for " + key);
            return false;
        }
        config->waveform_negative_half_first = b;
        return true;
    }
    if (key == "pacing.minimum_rate_mbps")
    {
        if (!ParseDoubleValue(value, &d))
        {
            WarnConfigLine(path, line_no, "invalid double for " + key);
            return false;
        }
        config->pacing_minimum_rate_mbps = d;
        return true;
    }

#define SET_WAVEFORM_DOUBLE(KEY, FIELD)                                        \
    if (key == KEY)                                                           \
    {                                                                         \
        if (!ParseDoubleValue(value, &d))                                     \
        {                                                                     \
            WarnConfigLine(path, line_no, "invalid double for " + key);       \
            return false;                                                     \
        }                                                                     \
        config->FIELD = d;                                                    \
        return true;                                                          \
    }
#define SET_WAVEFORM_U32(KEY, FIELD)                                           \
    if (key == KEY)                                                           \
    {                                                                         \
        if (!ParseUintValue(value, &u32))                                     \
        {                                                                     \
            WarnConfigLine(path, line_no, "invalid uint for " + key);         \
            return false;                                                     \
        }                                                                     \
        config->FIELD = u32;                                                  \
        return true;                                                          \
    }
#define SET_WAVEFORM_BOOL(KEY, FIELD)                                          \
    if (key == KEY)                                                           \
    {                                                                         \
        if (!ParseBoolValue(value, &b))                                       \
        {                                                                     \
            WarnConfigLine(path, line_no, "invalid bool for " + key);         \
            return false;                                                     \
        }                                                                     \
        config->FIELD = b;                                                    \
        return true;                                                          \
    }

    SET_WAVEFORM_DOUBLE("waveform.initial_settle_rtt_mult",
                        waveform_initial_settle_rtt_mult)
    SET_WAVEFORM_DOUBLE("waveform.post_adjust_settle_rtt_mult",
                        waveform_post_adjust_settle_rtt_mult)
    SET_WAVEFORM_DOUBLE("waveform.initial_window_periods",
                        waveform_initial_window_periods)
    SET_WAVEFORM_DOUBLE("waveform.extended_window_periods",
                        waveform_extended_window_periods)
    SET_WAVEFORM_DOUBLE("waveform.max_window_periods",
                        waveform_max_window_periods)
    SET_WAVEFORM_DOUBLE("waveform.period_tolerance_ratio",
                        waveform_period_tolerance_ratio)
    SET_WAVEFORM_DOUBLE("waveform.min_periodicity_correlation",
                        waveform_min_periodicity_correlation)
    SET_WAVEFORM_DOUBLE("waveform.min_cycle_coverage_ratio",
                        waveform_min_cycle_coverage_ratio)
    SET_WAVEFORM_DOUBLE("waveform.masked_min_cycle_coverage_ratio",
                        waveform_masked_min_cycle_coverage_ratio)
    SET_WAVEFORM_DOUBLE("waveform.local_slope_window_period_ratio",
                        waveform_local_slope_window_period_ratio)
    SET_WAVEFORM_DOUBLE("waveform.min_local_slope_window_ms",
                        waveform_min_local_slope_window_ms)
    SET_WAVEFORM_DOUBLE("waveform.clip_min_duration_ratio",
                        waveform_clip_min_duration_ratio)
    SET_WAVEFORM_DOUBLE("waveform.clip_min_half_overlap_ratio",
                        waveform_clip_min_half_overlap_ratio)
    SET_WAVEFORM_DOUBLE("waveform.clip_max_slope_ratio",
                        waveform_clip_max_slope_ratio)
    SET_WAVEFORM_DOUBLE("waveform.delta_drate_amplitude_ratio",
                        waveform_delta_drate_amplitude_ratio)
    SET_WAVEFORM_DOUBLE("waveform.delta_fallback_baseline_ratio",
                        waveform_delta_fallback_baseline_ratio)
    SET_WAVEFORM_DOUBLE("waveform.plateau_min_duration_ratio",
                        waveform_plateau_min_duration_ratio)
    SET_WAVEFORM_DOUBLE("waveform.plateau_max_slope_ratio",
                        waveform_plateau_max_slope_ratio)
    SET_WAVEFORM_DOUBLE("waveform.plateau_max_level_span_ratio",
                        waveform_plateau_max_level_span_ratio)
    SET_WAVEFORM_DOUBLE("waveform.plateau_extreme_distance_ratio",
                        waveform_plateau_extreme_distance_ratio)
    SET_WAVEFORM_DOUBLE("waveform.baseline_step_ratio",
                        waveform_baseline_step_ratio)
    SET_WAVEFORM_DOUBLE("waveform.amplitude_floor_ratio",
                        waveform_amplitude_floor_ratio)
    SET_WAVEFORM_U32("waveform.clip_floor_confirmations",
                     waveform_clip_floor_confirmations)
    SET_WAVEFORM_U32("waveform.max_baseline_adjustments",
                     waveform_max_baseline_adjustments)
    SET_WAVEFORM_U32("waveform.max_inconclusive_extensions",
                     waveform_max_inconclusive_extensions)
    SET_WAVEFORM_DOUBLE("waveform.inconclusive_signal_amplification_factor",
                        waveform_inconclusive_signal_amplification_factor)
    SET_WAVEFORM_DOUBLE("waveform.inconclusive_signal_amplification_max_ratio",
                        waveform_inconclusive_signal_amplification_max_ratio)
    SET_WAVEFORM_DOUBLE("waveform.max_app_limited_sample_ratio",
                        waveform_max_app_limited_sample_ratio)
    SET_WAVEFORM_DOUBLE("waveform.max_interpolation_gap_period_ratio",
                        waveform_max_interpolation_gap_period_ratio)
    SET_WAVEFORM_DOUBLE("goertzel.min_coherent_power_ratio",
                        goertzel_min_coherent_power_ratio)
    SET_WAVEFORM_DOUBLE("fbbr.regime.long_top_horizontal_duration_ratio", fbbr_regime_long_top_horizontal_duration_ratio)
    SET_WAVEFORM_DOUBLE("fbbr.regime.long_bottom_horizontal_duration_ratio", fbbr_regime_long_bottom_horizontal_duration_ratio)
    SET_WAVEFORM_U32("fbbr.wave_fidelity.no_wave_trigger_windows", fbbr_wave_fidelity_no_wave_trigger_windows)
    SET_WAVEFORM_U32("fbbr.wave_fidelity.retry_window_advance_periods", fbbr_wave_fidelity_retry_window_advance_periods)
    SET_WAVEFORM_DOUBLE("waveform.activity.amplitude_noise_multiplier", waveform_activity_amplitude_noise_multiplier)
    SET_WAVEFORM_DOUBLE("waveform.activity.min_level_ratio", waveform_activity_min_level_ratio)
    SET_WAVEFORM_DOUBLE("waveform.activity.step_noise_multiplier", waveform_activity_step_noise_multiplier)
    SET_WAVEFORM_DOUBLE("waveform.activity.min_normalized_step_slope", waveform_activity_min_normalized_step_slope)
    SET_WAVEFORM_U32("waveform.activity.min_active_steps", waveform_activity_min_active_steps)
    SET_WAVEFORM_DOUBLE("waveform.activity.min_active_step_ratio", waveform_activity_min_active_step_ratio)
    SET_WAVEFORM_DOUBLE("waveform.activity.min_directional_change_ratio", waveform_activity_min_directional_change_ratio)
    SET_WAVEFORM_DOUBLE("waveform.activity.min_significant_path_ratio", waveform_activity_min_significant_path_ratio)
    SET_WAVEFORM_U32("waveform.activity.min_slope_reversals", waveform_activity_min_slope_reversals)
    SET_WAVEFORM_DOUBLE("waveform.horizontal.continuous_min_duration_ratio", waveform_horizontal_continuous_min_duration_ratio)
    SET_WAVEFORM_DOUBLE("waveform.horizontal.min_valid_coverage_ratio", waveform_horizontal_min_valid_coverage_ratio)
    SET_WAVEFORM_DOUBLE("waveform.horizontal.min_flat_fraction", waveform_horizontal_min_flat_fraction)
    SET_WAVEFORM_DOUBLE("waveform.horizontal.max_local_slope_ratio", waveform_horizontal_max_local_slope_ratio)
    SET_WAVEFORM_DOUBLE("waveform.horizontal.min_side_slope_ratio", waveform_horizontal_min_side_slope_ratio)
    SET_WAVEFORM_DOUBLE("waveform.horizontal.min_boundary_kink_ratio", waveform_horizontal_min_boundary_kink_ratio)
    SET_WAVEFORM_DOUBLE("waveform.horizontal.max_level_span_ratio", waveform_horizontal_max_level_span_ratio)
    SET_WAVEFORM_DOUBLE("waveform.horizontal.max_total_drift_ratio", waveform_horizontal_max_total_drift_ratio)
    SET_WAVEFORM_DOUBLE("waveform.horizontal.min_side_change_ratio", waveform_horizontal_min_side_change_ratio)
    SET_WAVEFORM_DOUBLE("waveform.horizontal.amplitude_noise_multiplier", waveform_horizontal_amplitude_noise_multiplier)
    SET_WAVEFORM_DOUBLE("waveform.horizontal.level_span_noise_multiplier", waveform_horizontal_level_span_noise_multiplier)
    SET_WAVEFORM_DOUBLE("waveform.horizontal.slope_noise_multiplier", waveform_horizontal_slope_noise_multiplier)
    SET_WAVEFORM_DOUBLE("waveform.horizontal.extreme_distance_ratio", waveform_horizontal_extreme_distance_ratio)
    SET_WAVEFORM_DOUBLE("waveform.repeated_clip_max_period_error_ratio", waveform_repeated_clip_max_period_error_ratio)
    SET_WAVEFORM_DOUBLE("waveform.repeated_clip_max_level_delta_ratio", waveform_repeated_clip_max_level_delta_ratio)
    SET_WAVEFORM_DOUBLE("waveform.repeated_clip_contact_level_tolerance_ratio", waveform_repeated_clip_contact_level_tolerance_ratio)
    SET_WAVEFORM_U32("waveform.repeated_clip_min_contact_samples_per_cycle", waveform_repeated_clip_min_contact_samples_per_cycle)
    SET_WAVEFORM_U32("waveform.repeated_clip_min_total_contact_samples", waveform_repeated_clip_min_total_contact_samples)
    SET_WAVEFORM_DOUBLE("waveform.repeated_clip_min_contact_sample_ratio", waveform_repeated_clip_min_contact_sample_ratio)
    SET_WAVEFORM_DOUBLE("waveform.repeated_clip_min_contact_span_ratio_of_window", waveform_repeated_clip_min_contact_span_ratio_of_window)
    SET_WAVEFORM_DOUBLE("waveform.repeated_clip_min_pooled_flat_fraction", waveform_repeated_clip_min_pooled_flat_fraction)
    SET_WAVEFORM_DOUBLE("waveform.repeated_clip_min_verified_boundary_fraction", waveform_repeated_clip_min_verified_boundary_fraction)
    SET_WAVEFORM_DOUBLE("waveform.repeated_clip_min_outside_excursion_ratio", waveform_repeated_clip_min_outside_excursion_ratio)
    SET_WAVEFORM_DOUBLE("waveform.repeated_clip_min_extrapolated_overshoot_ratio", waveform_repeated_clip_min_extrapolated_overshoot_ratio)
    SET_WAVEFORM_DOUBLE("waveform.repeated_clip_merge_gap_ratio", waveform_repeated_clip_merge_gap_ratio)
    SET_WAVEFORM_DOUBLE("waveform.repeated_clip_max_missing_gap_ratio", waveform_repeated_clip_max_missing_gap_ratio)
    SET_WAVEFORM_DOUBLE("waveform.shoulder.min_half_overlap_ratio", waveform_shoulder_min_half_overlap_ratio)
    SET_WAVEFORM_DOUBLE("waveform.shoulder.min_side_change_ratio", waveform_shoulder_min_side_change_ratio)
    SET_WAVEFORM_DOUBLE("waveform.shoulder.max_residual_cycle_period_error_ratio", waveform_shoulder_max_residual_cycle_period_error_ratio)
    SET_WAVEFORM_DOUBLE("waveform.shoulder.min_residual_cycle_leg_duration_ratio", waveform_shoulder_min_residual_cycle_leg_duration_ratio)
    SET_WAVEFORM_DOUBLE("waveform.middle.min_duration_ratio", waveform_middle_min_duration_ratio)
    SET_WAVEFORM_DOUBLE("waveform.middle.max_duration_ratio", waveform_middle_max_duration_ratio)
    SET_WAVEFORM_DOUBLE("waveform.middle.context_duration_ratio", waveform_middle_context_duration_ratio)
    SET_WAVEFORM_DOUBLE("waveform.middle.min_trend_slope_ratio", waveform_middle_min_trend_slope_ratio)
    SET_WAVEFORM_DOUBLE("waveform.middle.max_context_slope_delta_ratio", waveform_middle_max_context_slope_delta_ratio)
    SET_WAVEFORM_DOUBLE("waveform.middle.min_slope_mismatch_ratio", waveform_middle_min_slope_mismatch_ratio)
    SET_WAVEFORM_DOUBLE("waveform.middle.min_mismatching_sample_ratio", waveform_middle_min_mismatching_sample_ratio)
    SET_WAVEFORM_U32("waveform.middle.min_mismatching_samples", waveform_middle_min_mismatching_samples)
    SET_WAVEFORM_U32("waveform.middle.min_consecutive_mismatching_samples", waveform_middle_min_consecutive_mismatching_samples)
    SET_WAVEFORM_DOUBLE("waveform.middle.min_bridge_deviation_ratio", waveform_middle_min_bridge_deviation_ratio)
    SET_WAVEFORM_DOUBLE("waveform.middle.noise_multiplier", waveform_middle_noise_multiplier)
    SET_WAVEFORM_DOUBLE("waveform.middle.max_mask_ratio_per_cycle", waveform_middle_max_mask_ratio_per_cycle)
    SET_WAVEFORM_DOUBLE("fbbr.regime.period_tolerance_ratio", fbbr_regime_period_tolerance_ratio)
    SET_WAVEFORM_DOUBLE("fbbr.regime.min_periodicity_correlation", fbbr_regime_min_periodicity_correlation)
    SET_WAVEFORM_BOOL("fbbr.regime.periodic_upper_clip_is_hard_veto", fbbr_regime_periodic_upper_clip_is_hard_veto)

#undef SET_WAVEFORM_DOUBLE
#undef SET_WAVEFORM_U32
#undef SET_WAVEFORM_BOOL

    return SetCommonFrequencyConfigValue(
        config, key, value, path, line_no);
}

bool
LoadFBBRConfig(const std::string& path, dqc::FBBRConfig* config)
{
    if (path.empty())
    {
        return false;
    }

    std::ifstream in(path.c_str());
    if (!in.is_open())
    {
        std::cerr << "[fbbr-config warning] unable to open config: " << path
                  << "; using built-in defaults" << std::endl;
        return false;
    }

    std::string line;
    uint32_t line_no = 0;
    while (std::getline(in, line))
    {
        ++line_no;
        const size_t comment = line.find('#');
        if (comment != std::string::npos)
        {
            line = line.substr(0, comment);
        }
        line = Trim(line);
        if (line.empty())
        {
            continue;
        }
        const size_t eq = line.find('=');
        if (eq == std::string::npos)
        {
            WarnConfigLine(path, line_no, "expected key=value");
            continue;
        }
        const std::string key = Trim(line.substr(0, eq));
        const std::string value = Trim(line.substr(eq + 1));
        if (key.empty())
        {
            WarnConfigLine(path, line_no, "empty key");
            continue;
        }
        SetFBBRConfigValue(config, key, value, path, line_no);
    }
    std::cout << "[fbbr-config] loaded " << path << std::endl;
    return true;
}

std::vector<std::string>
SplitCommaList(const std::string& input)
{
    std::vector<std::string> out;
    std::stringstream ss(input);
    std::string item;
    while (std::getline(ss, item, ','))
    {
        out.push_back(Trim(item));
    }
    if (out.size() == 1 && out[0].empty())
    {
        out.clear();
    }
    return out;
}

std::vector<std::string>
ExpandStringList(const std::string& input,
                 uint32_t n,
                 const std::string& default_value,
                 const std::string& name)
{
    std::vector<std::string> values = SplitCommaList(input);
    if (values.empty())
    {
        values.assign(n, default_value);
        return values;
    }
    if (values.size() == 1)
    {
        values.assign(n, values[0]);
        return values;
    }
    NS_ABORT_MSG_IF(values.size() != n,
                    name << " must contain either 1 value or nFlows values");
    return values;
}

std::vector<double>
ExpandDoubleList(const std::string& input,
                 uint32_t n,
                 double default_value,
                 const std::string& name)
{
    const std::vector<std::string> tokens =
        ExpandStringList(input, n, "", name);
    std::vector<double> values;
    values.reserve(n);
    for (size_t i = 0; i < tokens.size(); ++i)
    {
        if (tokens[i].empty())
        {
            values.push_back(default_value);
            continue;
        }
        double value = 0.0;
        NS_ABORT_MSG_IF(!ParseDoubleValue(tokens[i], &value),
                        "invalid " << name << " value: " << tokens[i]);
        values.push_back(value);
    }
    return values;
}

uint64_t
ParseRateBps(const std::string& value, const std::string& name)
{
    const std::string trimmed = Trim(value);
    if (trimmed.empty() || trimmed == "0")
    {
        return 0;
    }
    DataRate rate(trimmed);
    const uint64_t bps = rate.GetBitRate();
    if (bps == 0 && name == "backgroundRateSchedule")
    {
        return 0;
    }
    NS_ABORT_MSG_IF(bps == 0, "invalid zero " << name << ": " << value);
    return bps;
}

std::vector<uint64_t>
ExpandRateList(const std::string& input,
               uint32_t n,
               const std::string& default_value,
               const std::string& name)
{
    const std::vector<std::string> tokens =
        ExpandStringList(input, n, default_value, name);
    std::vector<uint64_t> values;
    values.reserve(n);
    for (size_t i = 0; i < tokens.size(); ++i)
    {
        values.push_back(ParseRateBps(tokens[i], name));
    }
    return values;
}

std::vector<std::vector<RateStep>>
ExpandRateScheduleList(const std::string& input,
                       uint32_t n,
                       const std::vector<uint64_t>& default_rates)
{
    std::vector<std::string> tokens;
    if (!Trim(input).empty())
    {
        std::stringstream stream(input);
        std::string token;
        while (std::getline(stream, token, '@'))
        {
            tokens.push_back(Trim(token));
        }
    }
    if (tokens.size() == 1 && n > 1)
    {
        tokens.resize(n, tokens.front());
    }
    NS_ABORT_MSG_IF(!tokens.empty() && tokens.size() != n,
                    "rateSchedules must contain one schedule or " << n
                    << " schedules separated by @");

    std::vector<std::vector<RateStep>> schedules;
    schedules.reserve(n);
    for (uint32_t i = 0; i < n; ++i)
    {
        std::ostringstream name;
        name << "rateSchedules flow " << (i + 1);
        schedules.push_back(ParseRateSchedule(
            tokens.empty() ? "" : tokens[i], default_rates[i], name.str()));
    }
    return schedules;
}

uint32_t
ClampToU32(uint64_t value, const std::string& name)
{
    if (value > std::numeric_limits<uint32_t>::max())
    {
        std::cerr << "[warning] " << name << "=" << value
                  << " exceeds uint32_t max; clamping to "
                  << std::numeric_limits<uint32_t>::max() << std::endl;
        return std::numeric_limits<uint32_t>::max();
    }
    return static_cast<uint32_t>(value);
}

uint32_t
ComputeSwitchBufferBytes(uint64_t service_rate_bps,
                         double access_delay_ms,
                         double service_delay_ms,
                         double buffer_bdp)
{
    const double rtt_s =
        2.0 * (2.0 * access_delay_ms + service_delay_ms) / 1000.0;
    const double bytes =
        static_cast<double>(service_rate_bps) * rtt_s * buffer_bdp / 8.0;
    return static_cast<uint32_t>(
        std::max(1.0, std::min(bytes,
                               static_cast<double>(
                                   std::numeric_limits<uint32_t>::max()))));
}

class ValidationBottleneckTracer
{
  public:
    ValidationBottleneckTracer(Ptr<Queue<Packet>> queue,
                               const ScenarioConfig& scenario,
                               const std::vector<FlowConfig>& flows)
        : m_queue(queue), m_scenario(scenario), m_flows(flows)
    {
        const std::string path = scenario.trace_path + "bottleneck_queue.csv";
        m_stream.open(path.c_str(), std::fstream::out);
        if (m_stream.is_open())
        {
            m_stream << "time_s,queue_bytes,queue_packets,enqueue_bytes_delta,"
                        "dequeue_bytes_delta,drop_bytes_delta,ecn_marked_bytes_delta,"
                        "egress_rate_bps,capacity_bps,queue_bdp,active_fbbr_flows,"
                        "background_rate_bps,theoretical_fair_share_bps\n";
        }
    }

    void Connect()
    {
        if (m_queue == nullptr) return;
        m_queue->TraceConnectWithoutContext(
            "Enqueue", MakeCallback(&ValidationBottleneckTracer::OnEnqueue, this));
        m_queue->TraceConnectWithoutContext(
            "Dequeue", MakeCallback(&ValidationBottleneckTracer::OnDequeue, this));
        m_queue->TraceConnectWithoutContext(
            "Drop", MakeCallback(&ValidationBottleneckTracer::OnDrop, this));
        Simulator::ScheduleNow(&ValidationBottleneckTracer::Sample, this);
    }

    void OnEnqueue(Ptr<const Packet> packet)
    {
        if (packet) m_enqueue_bytes += packet->GetSize();
    }

    void OnDequeue(Ptr<const Packet> packet)
    {
        if (packet) m_dequeue_bytes += packet->GetSize();
    }

    void OnDrop(Ptr<const Packet> packet)
    {
        if (packet) m_drop_bytes += packet->GetSize();
    }

    void Sample()
    {
        const double now_s = Simulator::Now().GetSeconds();
        const double interval_s = m_scenario.queue_sample_interval_us / 1e6;
        const uint64_t capacity_bps = RateAt(m_scenario.capacity_schedule, now_s);
        const uint64_t background_bps = RateAt(m_scenario.background_schedule, now_s);
    uint32_t active = 0;
    for (const auto& flow : m_flows)
    {
            if (flow.algo.is_fbbr &&
                flow.start_time_s <= now_s &&
                now_s < flow.stop_time_s)
            {
                ++active;
            }
        }
        const double rtprop_s =
            2.0 * (2.0 * m_scenario.access_delay_ms +
                   m_scenario.service_delay_ms) / 1000.0;
        const double current_bdp_bytes = capacity_bps * rtprop_s / 8.0;
        const uint32_t queue_bytes = m_queue ? m_queue->GetNBytes() : 0;
        const uint32_t queue_packets = m_queue ? m_queue->GetNPackets() : 0;
        const double fair_share = active > 0
            ? static_cast<double>(capacity_bps > background_bps
                                      ? capacity_bps - background_bps : 0) / active
            : 0.0;
        if (m_stream.is_open())
        {
            m_stream << std::setprecision(12) << now_s << "," << queue_bytes
                     << "," << queue_packets << "," << m_enqueue_bytes << ","
                     << m_dequeue_bytes << "," << m_drop_bytes << ",0,"
                     << (interval_s > 0.0 ? 8.0 * m_dequeue_bytes / interval_s : 0.0)
                     << "," << capacity_bps << ","
                     << (current_bdp_bytes > 0.0 ? queue_bytes / current_bdp_bytes : 0.0)
                     << "," << active << "," << background_bps << ","
                     << fair_share << "\n";
        }
        m_enqueue_bytes = m_dequeue_bytes = m_drop_bytes = 0;
        if (now_s + interval_s <= m_scenario.sim_time_s + 1e-12)
        {
            Simulator::Schedule(MicroSeconds(m_scenario.queue_sample_interval_us),
                                &ValidationBottleneckTracer::Sample, this);
        }
    }

  private:
    Ptr<Queue<Packet>> m_queue;
    const ScenarioConfig& m_scenario;
    const std::vector<FlowConfig>& m_flows;
    std::fstream m_stream;
    uint64_t m_enqueue_bytes{0};
    uint64_t m_dequeue_bytes{0};
    uint64_t m_drop_bytes{0};
};

std::string
CommandOutput(const char* command)
{
    std::string output;
    FILE* pipe = popen(command, "r");
    if (pipe == nullptr) return output;
    char buffer[256];
    while (fgets(buffer, sizeof(buffer), pipe) != nullptr) output += buffer;
    pclose(pipe);
    while (!output.empty() &&
           (output.back() == '\n' || output.back() == '\r')) output.pop_back();
    return output;
}

std::string
JsonEscape(const std::string& value)
{
    std::ostringstream out;
    for (char c : value)
    {
        if (c == '\\' || c == '"') out << '\\';
        out << c;
    }
    return out.str();
}

void
EmitRunMeta(const ScenarioConfig& scenario, const std::vector<FlowConfig>& flows)
{
    if (!scenario.emit_run_meta || scenario.trace_path.empty()) return;
    const double rtprop_s =
        2.0 * (2.0 * scenario.access_delay_ms + scenario.service_delay_ms) / 1000.0;
    const double bdp_bytes = scenario.service_rate_bps * rtprop_s / 8.0;
    const std::string commit = CommandOutput("git rev-parse HEAD 2>/dev/null");
    const bool dirty = !CommandOutput("git status --porcelain 2>/dev/null").empty();
    std::ofstream out((scenario.trace_path + "run_meta.json").c_str());
    if (!out.is_open()) return;
    const std::string algorithm = flows.empty()
        ? "unknown" : flows.front().algo.display_name;
    std::string fbbr_config_path;
    for (const auto& flow : flows)
    {
        if (flow.algo.is_fbbr && fbbr_config_path.empty())
            fbbr_config_path = flow.config_path;
    }
    out << std::setprecision(12)
        << "{\n  \"algorithm\": \"" << JsonEscape(algorithm) << "\",\n"
        << "  \"seed\": " << scenario.seed << ",\n"
        << "  \"run_id\": " << scenario.run_id << ",\n"
        << "  \"capacity_bps\": " << scenario.service_rate_bps << ",\n"
        << "  \"configured_capacity_bps\": " << scenario.service_rate_bps << ",\n"
        << "  \"access_rate_bps\": " << scenario.access_rate_bps << ",\n"
        << "  \"rtprop_s\": " << rtprop_s << ",\n"
        << "  \"measured_rtprop_s\": " << rtprop_s << ",\n"
        << "  \"empty_path_rtt_s\": " << rtprop_s << ",\n"
        << "  \"scenario_bdp_bytes\": " << bdp_bytes << ",\n"
        << "  \"bdp_bytes\": " << bdp_bytes << ",\n"
        << "  \"buffer_bytes\": " << scenario.switch_buffer_bytes << ",\n"
        << "  \"configured_buffer_bytes\": " << scenario.switch_buffer_bytes << ",\n"
        << "  \"buffer_bdp\": " << scenario.switch_buffer_bytes / bdp_bytes << ",\n"
        << "  \"configured_buffer_bdp\": " << scenario.switch_buffer_bdp << ",\n"
        << "  \"ack_timing_jitter_us\": " << scenario.ack_timing_jitter_us << ",\n"
        << "  \"ack_jitter_interval_ms\": " << scenario.ack_jitter_interval_ms << ",\n"
        << "  \"packet_size_variation_bytes\": "
        << scenario.packet_size_variation_bytes << ",\n"
        << "  \"n_flows\": " << scenario.n_flows << ",\n"
        << "  \"algorithms\": [";
    for (size_t i = 0; i < flows.size(); ++i)
        out << (i ? ", " : "") << "\""
            << JsonEscape(flows[i].algo.display_name) << "\"";
    out << "],\n  \"flow_weights\": [";
    for (size_t i = 0; i < flows.size(); ++i) out << (i ? ", " : "") << 1;
    out << "],\n  \"flow_start_times_s\": [";
    for (size_t i = 0; i < flows.size(); ++i)
        out << (i ? ", " : "") << flows[i].start_time_s;
    out << "],\n  \"flow_stop_times_s\": [";
    for (size_t i = 0; i < flows.size(); ++i)
        out << (i ? ", " : "") << flows[i].stop_time_s;
    out << "],\n  \"per_flow_app_rate_limits_bps\": [";
    for (size_t i = 0; i < flows.size(); ++i)
        out << (i ? ", " : "") << flows[i].sender_rate_cap_bps;
    out << "],\n  \"per_flow_rate_schedules\": [";
    for (size_t i = 0; i < flows.size(); ++i)
    {
        if (i) out << ",";
        out << "[";
        for (size_t j = 0; j < flows[i].rate_cap_schedule.size(); ++j)
        {
            const RateStep& step = flows[i].rate_cap_schedule[j];
            out << (j ? "," : "") << "{\"time_s\":" << step.time_s
                << ",\"rate_bps\":" << step.rate_bps << "}";
        }
        out << "]";
    }
    out << "],\n  \"background_schedule\": [";
    for (size_t i = 0; i < scenario.background_schedule.size(); ++i)
        out << (i ? "," : "") << "{\"time_s\":"
            << scenario.background_schedule[i].time_s << ",\"rate_bps\":"
            << scenario.background_schedule[i].rate_bps << "}";
    out << "],\n  \"capacity_schedule\": [";
    for (size_t i = 0; i < scenario.capacity_schedule.size(); ++i)
        out << (i ? "," : "") << "{\"time_s\":"
            << scenario.capacity_schedule[i].time_s << ",\"rate_bps\":"
            << scenario.capacity_schedule[i].rate_bps << "}";
    out << "],\n  \"fbbr_config\": \""
        << JsonEscape(fbbr_config_path) << "\",\n"
        << "  \"git_commit\": \"" << JsonEscape(commit) << "\",\n"
        << "  \"dirty_worktree\": " << (dirty ? "true" : "false") << "\n}\n";
}

void
SetBackgroundRate(Ptr<OnOffApplication> app, uint64_t rate_bps)
{
    app->SetAttribute("DataRate",
                      DataRateValue(DataRate(std::max<uint64_t>(1, rate_bps))));
}

void
SetSenderRateCap(Ptr<DqcSender> app, uint64_t rate_bps, uint32_t flow_id)
{
    app->SetMaxBandwidth(ClampToU32(rate_bps, "rateSchedules"));
    std::cout << "[rate-cap] time_s=" << Simulator::Now().GetSeconds()
              << " flow=" << flow_id << " rate_bps=" << rate_bps << std::endl;
}

void
SetBottleneckRate(Ptr<PointToPointNetDevice> device, uint64_t rate_bps)
{
    device->SetDataRate(DataRate(rate_bps));
}

void
SetPointToPointDelay(Ptr<PointToPointChannel> channel, double delay_ms)
{
    channel->SetPropagationDelay(MilliSeconds(std::max(0.001, delay_ms)));
}

void
UpdateSenderFairShares(std::vector<Ptr<DqcSender>>* senders,
                       const std::vector<FlowConfig>* flows,
                       const ScenarioConfig* scenario,
                       double time_s)
{
    uint32_t active = 0;
    for (const auto& flow : *flows)
    {
        active += flow.algo.is_fbbr &&
                  flow.start_time_s <= time_s &&
                  time_s < flow.stop_time_s;
    }
    const uint64_t capacity = RateAt(scenario->capacity_schedule, time_s);
    const uint64_t background = RateAt(scenario->background_schedule, time_s);
    const uint64_t fair = active > 0 && capacity > background
        ? (capacity - background) / active : 0;
    for (size_t i = 0; i < senders->size(); ++i)
    {
        if ((*flows)[i].algo.is_fbbr)
            (*senders)[i]->SetFBBRFairShareBandwidth(fair);
    }
}

AlgorithmSpec
ParseAlgorithm(const std::string& name)
{
    const std::string key = Trim(name);
    if (key == "oBBR")
    {
        return {dqc::kOBBR, "oBBR", false, false};
    }
    if (key == "BBR-R" || key == "BBRR" || key == "bbr_r")
    {
        return {dqc::kBBRR, "BBR-R", false, false};
    }
    if (key == "CUBIC" || key == "Cubic" || key == "TcpCubic" ||
        key == "ns3-CUBIC")
    {
        return {dqc::kNs3Cubic, "CUBIC", false, false};
    }
    if (key == "BBRv2plus")
    {
        return {dqc::kBBRv2Plus, "BBRv2plus", false, false};
    }
    if (key == "BBRv2plusEcn")
    {
        return {dqc::kBBRv2PlusEcn, "BBRv2plusEcn", true, false};
    }
    if (key == "FBBR")
    {
        return {dqc::kFBBR, "FBBR", false, true};
    }
    if (key == "FBBR-ServiceFair")
    {
        return {dqc::kFBBRServiceFair, "FBBR-ServiceFair", false, true};
    }
    if (key == "FreqCCv3")
    {
        return {dqc::kFreqCCv3, "FreqCCv3", false, true};
    }
    if (key == "BBRv2")
    {
        return {dqc::kBBRv2, "BBRv2", false, false};
    }
    NS_ABORT_MSG("unsupported algorithm: " << name
                                           << " (supported: CUBIC, BBR-R, oBBR, BBRv2plus, "
                                              "FBBR, FBBR-ServiceFair, "
                                              "FreqCCv3, BBRv2)");
}

std::string
SubnetBase(uint32_t network_id)
{
    const uint32_t second = 1 + network_id / 256;
    const uint32_t third = network_id % 256;
    NS_ABORT_MSG_IF(second > 254, "too many point-to-point subnets");
    std::ostringstream oss;
    oss << "10." << second << "." << third << ".0";
    return oss.str();
}

void
SetNetDeviceQueueBytes(Ptr<NetDevice> device, uint32_t max_bytes)
{
    Ptr<PointToPointNetDevice> ptp = DynamicCast<PointToPointNetDevice>(device);
    NS_ABORT_MSG_IF(ptp == nullptr, "expected PointToPointNetDevice");
    Ptr<Queue<Packet>> queue = ptp->GetQueue();
    NS_ABORT_MSG_IF(queue == nullptr, "point-to-point queue is null");
    queue->SetAttribute("MaxBytes", UintegerValue(max_bytes));
}

void
AttachTraceCallbacks(Ptr<DqcSender> send_app,
                     Ptr<DqcReceiver> recv_app,
                     DqcTrace* trace,
                     DqcTraceState* stat,
                     uint32_t trace_enable)
{
    if (trace == nullptr)
    {
        return;
    }
    if (trace_enable & DqcTraceEnable::E_DQC_STAT)
    {
        recv_app->SetStatsTraceFuc(MakeCallback(&DqcTrace::OnStats, trace));
        trace->SetStatsTraceFuc(MakeCallback(&DqcTraceState::OnStats, stat));
    }
    if (trace_enable & DqcTraceEnable::E_DQC_OWD)
    {
        recv_app->SetOwdTraceFuc(MakeCallback(&DqcTrace::OnOwd, trace));
    }
    if (trace_enable & DqcTraceEnable::E_DQC_GOODPUT)
    {
        recv_app->SetGoodputTraceFuc(MakeCallback(&DqcTrace::OnGoodput, trace));
    }
    if (trace_enable & DqcTraceEnable::E_DQC_BW)
    {
        send_app->SetBwTraceFuc(MakeCallback(&DqcTrace::OnBw, trace));
    }
    if (trace_enable & DqcTraceEnable::E_DQC_RTT)
    {
        send_app->SetRttTraceFuc(MakeCallback(&DqcTrace::OnRtt, trace));
    }
    if (trace_enable & DqcTraceEnable::E_DQC_QUEUE_DELAY)
    {
        send_app->SetQueueDelayTraceFuc(
            MakeCallback(&DqcTrace::OnQueueDelay, trace));
    }
    if (trace_enable & DqcTraceEnable::E_DQC_SEND_RATE)
    {
        send_app->SetSendRateTraceFuc(
            MakeCallback(&DqcTrace::OnSendRate, trace));
    }
    if (trace_enable & DqcTraceEnable::E_DQC_RECV_RATE)
    {
        send_app->SetRecvRateTraceFuc(
            MakeCallback(&DqcTrace::OnRecvRate, trace));
    }
    if (trace_enable & DqcTraceEnable::E_DQC_RECV_RATE_RAW)
    {
        send_app->SetRecvRateRawTraceFuc(
            MakeCallback(&DqcTrace::OnRecvRateRaw, trace));
    }
    if (trace_enable & DqcTraceEnable::E_DQC_INFLIGHT)
    {
        send_app->SetInflightTraceFuc(
            MakeCallback(&DqcTrace::OnInflight, trace));
    }
    if (trace_enable & DqcTraceEnable::E_DQC_BBR_MODE)
    {
        send_app->SetBbrModeTraceFuc(
            MakeCallback(&DqcTrace::OnBbrMode, trace));
    }
    if (trace_enable & DqcTraceEnable::E_DQC_UP_PHASE)
    {
        send_app->SetUpPhaseTraceFuc(
            MakeCallback(&DqcTrace::OnUpPhase, trace));
    }
    if (trace_enable & DqcTraceEnable::E_DQC_FREQ_ANALYSIS)
    {
        send_app->SetFreqAnalysisTraceFuc(
            MakeCallback(&DqcTrace::OnFreqAnalysis, trace));
        send_app->SetRttFreqAnalysisTraceFuc(
            MakeCallback(&DqcTrace::OnRttFreqAnalysis, trace));
    }
    if (trace_enable &
        (DqcTraceEnable::E_DQC_FBBR_LOAD |
         DqcTraceEnable::E_DQC_FBBR_GATE))
    {
        send_app->SetFBBRLoadTraceFuc(
            MakeCallback(&DqcTrace::OnFBBRLoad, trace));
    }
    if (trace_enable & DqcTraceEnable::E_DQC_LOSS_RATE)
    {
        send_app->SetLossRateTraceFuc(
            MakeCallback(&DqcTrace::OnLossRate, trace));
    }
}

uint32_t
BuildTraceEnableMask(const ScenarioConfig& scenario,
                     const std::vector<FlowConfig>& flows)
{
    if (!scenario.enable_trace)
    {
        return 0;
    }

    uint32_t trace_enable = DqcTraceEnable::E_DQC_GOODPUT |
                            DqcTraceEnable::E_DQC_BBR_MODE |
                            DqcTraceEnable::E_DQC_LOSS_RATE |
                            DqcTraceEnable::E_DQC_STAT;
    if (scenario.enable_heavy_trace)
    {
        trace_enable |= DqcTraceEnable::E_DQC_OWD |
                        DqcTraceEnable::E_DQC_RTT |
                        DqcTraceEnable::E_DQC_BW |
                        DqcTraceEnable::E_DQC_SEND_RATE |
                        DqcTraceEnable::E_DQC_RECV_RATE |
                        DqcTraceEnable::E_DQC_RECV_RATE_RAW |
                        DqcTraceEnable::E_DQC_QUEUE_DELAY |
                        DqcTraceEnable::E_DQC_INFLIGHT |
                        DqcTraceEnable::E_DQC_FREQ_ANALYSIS;
    }
    for (size_t i = 0; i < flows.size(); ++i)
    {
        if (flows[i].algo.is_fbbr)
        {
            trace_enable |= DqcTraceEnable::E_DQC_FBBR_LOAD;
            if (scenario.enable_convergence_gate_trace)
            {
                trace_enable |= DqcTraceEnable::E_DQC_FBBR_GATE;
            }
        }
    }
    return trace_enable;
}

Ptr<DqcSender>
InstallDqcFlow(const FlowConfig& flow,
               Ptr<Node> sender,
               Ptr<Node> receiver,
               Ipv4Address receiver_ip,
               uint16_t send_port,
               uint16_t recv_port,
               const ScenarioConfig& scenario,
               DqcTrace* trace,
               DqcTraceState* stat,
               uint32_t trace_enable)
{
    Ptr<DqcSender> send_app =
        CreateObject<DqcSender>(flow.algo.cc,
                                flow.algo.ecn,
                                scenario.use_engine_timer);
    Ptr<DqcReceiver> recv_app =
        CreateObject<DqcReceiver>(scenario.goodput_interval_ms);

    sender->AddApplication(send_app);
    receiver->AddApplication(recv_app);

    recv_app->Bind(recv_port);
    send_app->Bind(send_port);
    send_app->ConfigurePeer(receiver_ip, recv_port);
    send_app->SetSenderId(flow.index + 1);
    send_app->SetCongestionId(flow.index + 1);
    send_app->SetNumEmulatedConnections(scenario.emulated_connections);
    send_app->SetProcessIntervalUs(scenario.process_interval_us);
    send_app->SetDataGeneratorBatch(scenario.data_generator_batch);
    send_app->SetDataChunkVariationBytes(
        scenario.packet_size_variation_bytes,
        (static_cast<uint64_t>(scenario.seed) << 32) | scenario.run_id);
    if (scenario.stream_buffer_bytes > 0)
    {
        send_app->SetStreamSendBufferBytes(scenario.stream_buffer_bytes);
    }
    if (scenario.flow_size_bytes > 0)
    {
        send_app->SetPacketLimitBytes(scenario.flow_size_bytes);
    }
    if (flow.sender_rate_cap_bps > 0)
    {
        send_app->SetMaxBandwidth(
            ClampToU32(flow.sender_rate_cap_bps, "initialRates"));
    }

    if (flow.algo.is_fbbr)
    {
        const uint64_t fair_share_bps =
            std::max<uint64_t>(1, scenario.service_rate_bps / scenario.n_flows);
        send_app->ConfigureFBBR(flow.fbbr_config, flow.index);
        send_app->ConfigureFBBRConvergenceGate(
            scenario.enable_convergence_gate_trace,
            scenario.enable_convergence_gate_control,
            scenario.gate_trace_mode,
            scenario.gate_trace_sample_interval_us);
        send_app->SetFBBRFairShareBandwidth(fair_share_bps);
    }

    AttachTraceCallbacks(send_app, recv_app, trace, stat, trace_enable);

    recv_app->SetStartTime(Seconds(0.0));
    recv_app->SetStopTime(Seconds(flow.stop_time_s));
    send_app->SetStartTime(Seconds(flow.start_time_s));
    send_app->SetStopTime(Seconds(flow.stop_time_s));

    return send_app;
}

std::vector<FlowConfig>
BuildFlowConfigs(uint32_t n_flows,
                 const std::string& algos,
                 const std::string& start_times,
                 const std::string& stop_times,
                 const std::string& initial_rates,
                 const std::string& rate_schedules,
                 const std::string& config_paths,
                 const std::string& fbbr_config_path,
                 const std::string& obbr_config_path,
                 const std::string& bbrv2plus_config_path,
                 const std::string& bbrv2_config_path)
{
    const std::vector<std::string> algo_tokens =
        ExpandStringList(algos, n_flows, "BBRv2", "algos");
    const std::vector<double> start_values =
        ExpandDoubleList(start_times, n_flows, 0.0, "startTimes");
    const std::vector<double> stop_values =
        ExpandDoubleList(stop_times, n_flows,
                         std::numeric_limits<double>::max(), "flowStopTimes");
    const std::vector<uint64_t> rate_caps =
        ExpandRateList(initial_rates, n_flows, "0", "initialRates");
    const std::vector<std::vector<RateStep>> expanded_rate_schedules =
        ExpandRateScheduleList(rate_schedules, n_flows, rate_caps);
    const std::vector<std::string> config_values =
        ExpandStringList(config_paths, n_flows, "", "configPaths");

    std::vector<FlowConfig> flows;
    flows.reserve(n_flows);
    for (uint32_t i = 0; i < n_flows; ++i)
    {
        FlowConfig flow;
        flow.index = i;
        flow.algo = ParseAlgorithm(algo_tokens[i]);
        flow.start_time_s = start_values[i];
        flow.stop_time_s = stop_values[i];
        flow.rate_cap_schedule = expanded_rate_schedules[i];
        flow.sender_rate_cap_bps = RateAt(flow.rate_cap_schedule, 0.0);

        if (!config_values[i].empty())
        {
            flow.config_path = config_values[i];
        }
        else if (flow.algo.is_fbbr)
        {
            flow.config_path = fbbr_config_path;
        }
        else if (flow.algo.display_name == "oBBR")
        {
            flow.config_path = obbr_config_path;
        }
        else if (flow.algo.display_name == "BBRv2plus" ||
                 flow.algo.display_name == "BBRv2plusEcn")
        {
            flow.config_path = bbrv2plus_config_path;
        }
        else if (flow.algo.display_name == "BBRv2")
        {
            flow.config_path = bbrv2_config_path;
        }

        if (flow.algo.is_fbbr)
        {
            LoadFBBRConfig(flow.config_path, &flow.fbbr_config);
        }
        else if (!flow.config_path.empty())
        {
            std::cout << "[config-info] " << flow.algo.display_name
                      << " config path recorded but this DQC implementation "
                         "has no loader hook: "
                      << flow.config_path << std::endl;
        }

        NS_ABORT_MSG_IF(flow.start_time_s < 0.0,
                        "startTimes values must be non-negative");
        NS_ABORT_MSG_IF(flow.stop_time_s <= flow.start_time_s,
                        "flowStopTimes must be greater than flowStartTimes");
        flows.push_back(flow);
    }
    return flows;
}

void
RunScenario(const ScenarioConfig& scenario,
            const std::vector<FlowConfig>& flows,
            DqcTraceState* stat)
{
    NodeContainer senders;
    senders.Create(scenario.n_flows);
    NodeContainer receivers;
    receivers.Create(scenario.n_flows);
    Ptr<Node> left_switch = CreateObject<Node>();
    Ptr<Node> right_switch = CreateObject<Node>();

    NodeContainer all_nodes;
    for (uint32_t i = 0; i < scenario.n_flows; ++i)
    {
        all_nodes.Add(senders.Get(i));
        all_nodes.Add(receivers.Get(i));
    }
    all_nodes.Add(left_switch);
    all_nodes.Add(right_switch);

    InternetStackHelper internet;
    internet.Install(all_nodes);

    PointToPointHelper p2p;
    TrafficControlHelper tch;

    p2p.SetQueue("ns3::DropTailQueue",
                 "Mode",
                 StringValue("QUEUE_MODE_BYTES"),
                 "MaxBytes",
                 UintegerValue(scenario.endpoint_queue_bytes));
    p2p.SetDeviceAttribute("DataRate",
                           DataRateValue(DataRate(scenario.access_rate_bps)));
    p2p.SetChannelAttribute(
        "Delay",
        TimeValue(MilliSeconds(scenario.access_delay_ms)));

    Ipv4AddressHelper ipv4;
    std::vector<NetDeviceContainer> access_devices;
    access_devices.reserve(scenario.n_flows);
    for (uint32_t i = 0; i < scenario.n_flows; ++i)
    {
        NodeContainer link(senders.Get(i), left_switch);
        NetDeviceContainer devices = p2p.Install(link);
        const std::string subnet = SubnetBase(i + 1);
        ipv4.SetBase(subnet.c_str(), "255.255.255.0");
        ipv4.Assign(devices);
        tch.Uninstall(devices);
        access_devices.push_back(devices);
    }

    p2p.SetQueue("ns3::DropTailQueue",
                 "Mode",
                 StringValue("QUEUE_MODE_BYTES"),
                 "MaxBytes",
                 UintegerValue(scenario.endpoint_queue_bytes));
    p2p.SetDeviceAttribute("DataRate",
                           DataRateValue(DataRate(scenario.service_rate_bps)));
    p2p.SetChannelAttribute(
        "Delay",
        TimeValue(MilliSeconds(scenario.service_delay_ms)));

    NodeContainer bottleneck_link(left_switch, right_switch);
    NetDeviceContainer bottleneck_devices = p2p.Install(bottleneck_link);
    SetNetDeviceQueueBytes(bottleneck_devices.Get(0),
                           scenario.switch_buffer_bytes);

    const std::string bottleneck_subnet = SubnetBase(scenario.n_flows + 1);
    ipv4.SetBase(bottleneck_subnet.c_str(), "255.255.255.0");
    Ipv4InterfaceContainer bottleneck_interfaces = ipv4.Assign(bottleneck_devices);
    tch.Uninstall(bottleneck_devices);

    p2p.SetQueue("ns3::DropTailQueue",
                 "Mode",
                 StringValue("QUEUE_MODE_BYTES"),
                 "MaxBytes",
                 UintegerValue(scenario.endpoint_queue_bytes));
    p2p.SetDeviceAttribute("DataRate",
                           DataRateValue(DataRate(scenario.access_rate_bps)));
    p2p.SetChannelAttribute(
        "Delay",
        TimeValue(MilliSeconds(scenario.access_delay_ms)));

    std::vector<Ipv4InterfaceContainer> receiver_interfaces;
    std::vector<Ptr<PointToPointChannel>> receiver_channels;
    receiver_interfaces.reserve(scenario.n_flows);
    receiver_channels.reserve(scenario.n_flows);
    for (uint32_t i = 0; i < scenario.n_flows; ++i)
    {
        NodeContainer receiver_link(right_switch, receivers.Get(i));
        NetDeviceContainer devices = p2p.Install(receiver_link);

        const std::string subnet = SubnetBase(scenario.n_flows + i + 2);
        ipv4.SetBase(subnet.c_str(), "255.255.255.0");
        Ipv4InterfaceContainer interfaces = ipv4.Assign(devices);
        tch.Uninstall(devices);
        receiver_interfaces.push_back(interfaces);
        receiver_channels.push_back(DynamicCast<PointToPointChannel>(
            devices.Get(0)->GetChannel()));
    }

    if (scenario.ack_timing_jitter_us > 0.0)
    {
        std::mt19937 jitter_rng(
            scenario.seed * 0x9e3779b9u ^ scenario.run_id * 0x85ebca6bu);
        std::uniform_real_distribution<double> jitter_us(
            -scenario.ack_timing_jitter_us,
            scenario.ack_timing_jitter_us);
        for (const auto& channel : receiver_channels)
        {
            const double delay_ms = scenario.access_delay_ms +
                                    jitter_us(jitter_rng) / 1000.0;
            SetPointToPointDelay(channel, delay_ms);
        }
    }

    Ipv4GlobalRoutingHelper::PopulateRoutingTables();

    std::vector<std::shared_ptr<QueueOccupancyTracer>> queue_tracers;
    if (scenario.enable_queue_trace)
    {
        std::ostringstream queue_trace_name;
        queue_trace_name << scenario.trace_name << "_shared";
        queue_tracers.push_back(InstallBottleneckQueueOccupancyTrace(
            bottleneck_devices.Get(0),
            queue_trace_name.str(),
            scenario.n_flows));
    }

    std::unique_ptr<ValidationBottleneckTracer> validation_queue_tracer;
    if (scenario.emit_bottleneck_queue_trace)
    {
        Ptr<PointToPointNetDevice> device =
            DynamicCast<PointToPointNetDevice>(bottleneck_devices.Get(0));
        validation_queue_tracer.reset(new ValidationBottleneckTracer(
            device->GetQueue(), scenario, flows));
        validation_queue_tracer->Connect();
    }

    bool has_background = false;
    for (const auto& step : scenario.background_schedule)
        has_background = has_background || step.rate_bps > 0;
    if (has_background)
    {
        const uint16_t background_port = 9000;
        PacketSinkHelper sink("ns3::UdpSocketFactory",
                              InetSocketAddress(Ipv4Address::GetAny(),
                                                background_port));
        ApplicationContainer sink_apps = sink.Install(right_switch);
        sink_apps.Start(Seconds(0.0));
        sink_apps.Stop(Seconds(scenario.sim_time_s));

        const uint64_t initial_rate = std::max<uint64_t>(
            1, RateAt(scenario.background_schedule, 0.0));
        OnOffHelper source(
            "ns3::UdpSocketFactory",
            InetSocketAddress(bottleneck_interfaces.GetAddress(1),
                              background_port));
        source.SetAttribute("PacketSize", UintegerValue(1200));
        source.SetAttribute("DataRate", DataRateValue(DataRate(initial_rate)));
        source.SetAttribute("OnTime",
                            StringValue("ns3::ConstantRandomVariable[Constant=1]"));
        source.SetAttribute("OffTime",
                            StringValue("ns3::ConstantRandomVariable[Constant=0]"));
        ApplicationContainer source_apps = source.Install(left_switch);
        source_apps.Start(Seconds(0.0));
        source_apps.Stop(Seconds(scenario.sim_time_s));
        Ptr<OnOffApplication> app =
            DynamicCast<OnOffApplication>(source_apps.Get(0));
        for (const auto& step : scenario.background_schedule)
        {
            Simulator::Schedule(Seconds(step.time_s), &SetBackgroundRate,
                                app, step.rate_bps);
        }
    }

    Ptr<PointToPointNetDevice> bottleneck_egress =
        DynamicCast<PointToPointNetDevice>(bottleneck_devices.Get(0));
    for (const auto& step : scenario.capacity_schedule)
    {
        Simulator::Schedule(Seconds(step.time_s), &SetBottleneckRate,
                            bottleneck_egress, step.rate_bps);
    }

    const uint32_t trace_enable = BuildTraceEnableMask(scenario, flows);
    std::vector<std::unique_ptr<DqcTrace>> traces;
    std::vector<Ptr<DqcSender>> send_apps;
    traces.reserve(scenario.n_flows);
    send_apps.reserve(scenario.n_flows);

    uint16_t send_port = 1000;
    uint16_t recv_port = 5000;
    for (uint32_t i = 0; i < scenario.n_flows; ++i)
    {
        uint32_t flow_trace_enable = trace_enable &
            ~(DqcTraceEnable::E_DQC_FBBR_LOAD |
              DqcTraceEnable::E_DQC_FBBR_GATE);
        if (flows[i].algo.is_fbbr)
        {
            flow_trace_enable |= DqcTraceEnable::E_DQC_FBBR_LOAD;
            if (scenario.enable_convergence_gate_trace)
                flow_trace_enable |= DqcTraceEnable::E_DQC_FBBR_GATE;
        }
        std::unique_ptr<DqcTrace> trace;
        DqcTrace* trace_ptr = nullptr;
        if (scenario.enable_trace)
        {
            trace.reset(new DqcTrace(i + 1));
            std::ostringstream log_name;
            log_name << scenario.trace_name << "_flow" << (i + 1) << "_"
                     << flows[i].algo.display_name;
            trace->Log(log_name.str(), flow_trace_enable);
            trace_ptr = trace.get();
        }
        stat->ReisterAvgDelayId(i + 1);
        if (i == 0)
        {
            stat->RegisterCongestionType(i + 1);
        }

        Ptr<DqcSender> sender = InstallDqcFlow(
            flows[i],
            senders.Get(i),
            receivers.Get(i),
            receiver_interfaces[i].GetAddress(1),
            send_port,
            recv_port,
            scenario,
            trace_ptr,
            stat,
            flow_trace_enable);
        send_apps.push_back(sender);
        for (const auto& step : flows[i].rate_cap_schedule)
        {
            if (step.time_s > 0.0 && step.time_s < scenario.sim_time_s)
            {
                Simulator::Schedule(Seconds(step.time_s), &SetSenderRateCap,
                                    sender, step.rate_bps, i + 1);
            }
        }
        if (trace)
        {
            traces.push_back(std::move(trace));
        }
        ++send_port;
        ++recv_port;
    }

    std::set<double> fair_share_change_times{0.0};
    for (const auto& flow : flows)
    {
        fair_share_change_times.insert(flow.start_time_s);
        if (flow.stop_time_s < scenario.sim_time_s)
            fair_share_change_times.insert(flow.stop_time_s);
    }
    for (const auto& step : scenario.background_schedule)
        fair_share_change_times.insert(step.time_s);
    for (const auto& step : scenario.capacity_schedule)
        fair_share_change_times.insert(step.time_s);
    for (double time_s : fair_share_change_times)
    {
        Simulator::Schedule(Seconds(time_s), &UpdateSenderFairShares,
                            &send_apps, &flows, &scenario, time_s);
    }

    EmitRunMeta(scenario, flows);
    Simulator::Stop(Seconds(scenario.sim_time_s));
    const auto wall_start = std::chrono::high_resolution_clock::now();
    Simulator::Run();
    const auto wall_end = std::chrono::high_resolution_clock::now();
    const std::chrono::duration<double> wall = wall_end - wall_start;

    for (const Ptr<DqcSender>& sender : send_apps)
    {
        sender->FinalizeCongestionControlTrace();
    }

    std::cout << "[runtime] sim_stop_s=" << scenario.sim_time_s
              << " simulator_now_s=" << Simulator::Now().GetSeconds()
              << " wall_seconds=" << wall.count()
              << " trace_enable=0x" << std::hex << trace_enable << std::dec
              << std::endl;

    Simulator::Destroy();
    stat->Flush(ClampToU32(scenario.service_rate_bps,
                           "totalServiceRate"),
                scenario.sim_time_s);
}

void
PrintConfiguration(const ScenarioConfig& scenario,
                   const std::vector<FlowConfig>& flows)
{
    std::cout << "=== Generic P2P Switch Flows Configuration ===" << std::endl;
    std::cout << "Topology: " << scenario.n_flows
              << " sender/receiver pairs through a shared dumbbell bottleneck"
              << std::endl;
    std::cout << "Sender/receiver access links: " << scenario.access_rate << ", "
              << scenario.access_delay_ms << " ms one-way" << std::endl;
    std::cout << "Shared bottleneck link: " << scenario.service_rate << ", "
              << scenario.service_delay_ms << " ms one-way" << std::endl;
    std::cout << "Shared bottleneck egress buffer: " << scenario.switch_buffer_bytes
              << " bytes (" << scenario.switch_buffer_bdp
              << " BDP if auto-computed)" << std::endl;
    std::cout << "Endpoint queue bytes: " << scenario.endpoint_queue_bytes
              << std::endl;
    std::cout << "Simulation time: " << scenario.sim_time_s << " s"
              << std::endl;
    std::cout << "Sender rate cap source: initialRates -> SetMaxBandwidth()"
              << std::endl;
    std::cout << "Trace path: "
              << (scenario.trace_path.empty() ? "default ./traces/"
                                              : scenario.trace_path)
              << std::endl;
    std::cout << "----------------------------------------------" << std::endl;
    for (size_t i = 0; i < flows.size(); ++i)
    {
        std::cout << "Flow " << (i + 1) << ": algo="
                  << flows[i].algo.display_name
                  << ", start=" << flows[i].start_time_s << " s"
                  << ", sender_cap_bps=" << flows[i].sender_rate_cap_bps;
        if (flows[i].rate_cap_schedule.size() > 1)
        {
            std::cout << ", rate_schedule=";
            for (size_t j = 0; j < flows[i].rate_cap_schedule.size(); ++j)
            {
                const RateStep& step = flows[i].rate_cap_schedule[j];
                std::cout << (j ? "," : "") << step.time_s << ":"
                          << step.rate_bps;
            }
        }
        if (!flows[i].config_path.empty())
        {
            std::cout << ", config=" << flows[i].config_path;
        }
        std::cout << std::endl;
    }
    std::cout << "==============================================" << std::endl;
}

} // namespace

int
main(int argc, char* argv[])
{
    uint32_t n_flows = 4;
    double sim_time_s = 30.0;
    std::string algos = "BBRv2";
    std::string start_times = "0";
    std::string stop_times = "";
    std::string initial_rates = "0";
    std::string rate_schedules = "";
    std::string per_flow_app_rate_limits = "";
    std::string background_rate_schedule = "";
    std::string capacity_schedule = "";
    std::string config_paths = "";

    std::string access_rate = "1Gbps";
    std::string service_rate = "20Mbps";
    double access_delay_ms = 1.0;
    double service_delay_ms = 10.0;
    double prop_delay_ms = -1.0;
    uint32_t switch_buffer_bytes = 0;
    double switch_buffer_bdp = 1.0;
    uint32_t endpoint_queue_bytes = kDefaultEndpointQueueBytes;

    uint64_t flow_size_bytes = 0;
    int64_t process_interval_us = 100;
    uint32_t goodput_interval_ms = 100;
    bool use_engine_timer = true;
    bool enable_trace = true;
    bool enable_heavy_trace = false;
    bool enable_queue_trace = true;
    bool emit_run_meta = true;
    bool emit_bottleneck_queue_trace = true;
    uint32_t queue_sample_interval_us = 200;
    bool enable_equivalence_audit = false;
    double ack_timing_jitter_us = 0.0;
    double ack_jitter_interval_ms = 10.0;
    uint32_t emulated_connections = 1;
    uint32_t data_generator_batch = 2;
    uint32_t packet_size_variation_bytes = 0;
    uint32_t stream_buffer_bytes = 0;

    bool enable_convergence_gate_trace = false;
    bool enable_convergence_gate_control = false;
    std::string gate_trace_mode = "round_only";
    uint64_t gate_trace_sample_interval_us = 10000;

    std::string fbbr_config = kDefaultFBBRConfig;
    std::string obbr_config = "";
    std::string bbrv2plus_config = "";
    std::string bbrv2_config = "";

    std::string trace_path = "";
    std::string output_dir = "";
    std::string trace_name = "generic_p2p_switch";
    uint32_t seed = 1;
    uint32_t run_id = 1;

    CommandLine cmd;
    cmd.AddValue("nFlows", "Number of concurrent DQC flows", n_flows);
    cmd.AddValue("simTime", "Simulation time in seconds", sim_time_s);
    cmd.AddValue("sim_time", "Alias of simTime", sim_time_s);
    cmd.AddValue("algos",
                 "Comma list of algorithms: oBBR, BBRv2plus, FBBR, FBBR-ServiceFair, FreqCCv3, BBRv2",
                 algos);
    cmd.AddValue("startTimes",
                 "Comma list of per-flow injection times in seconds",
                 start_times);
    cmd.AddValue("flowStartTimes",
                 "Alias of startTimes",
                 start_times);
    cmd.AddValue("flowStopTimes",
                 "Comma list of per-flow stop times in seconds",
                 stop_times);
    cmd.AddValue("initialRates",
                 "Comma list of per-flow sender pacing caps, e.g. 10Mbps; 0 "
                 "means no cap",
                 initial_rates);
    cmd.AddValue("perFlowAppRateLimits",
                 "Comma list of per-flow application rate limits",
                 per_flow_app_rate_limits);
    cmd.AddValue("rateSchedules",
                 "Per-flow time:rate steps; separate flows with @, e.g. "
                 "0:20Mbps,6:55Mbps@0:20Mbps,6:55Mbps",
                 rate_schedules);
    cmd.AddValue("backgroundRateSchedule",
                 "Comma list of time:rate steps for fixed UDP background",
                 background_rate_schedule);
    cmd.AddValue("capacitySchedule",
                 "Comma list of time:rate bottleneck capacity steps",
                 capacity_schedule);
    cmd.AddValue("configPaths",
                 "Comma list of per-flow config paths; FBBR/FreqCCv3 paths are loaded",
                 config_paths);
    cmd.AddValue("accessRate",
                 "Sender/receiver access link data rate",
                 access_rate);
    cmd.AddValue("serviceRate",
                 "Shared left-switch-to-right-switch bottleneck data rate",
                 service_rate);
    cmd.AddValue("accessDelayMs",
                 "Sender/receiver access one-way propagation delay in ms",
                 access_delay_ms);
    cmd.AddValue("serviceDelayMs",
                 "Shared bottleneck one-way propagation delay in ms",
                 service_delay_ms);
    cmd.AddValue("propDelayMs",
                 "If >=0, overrides both accessDelayMs and serviceDelayMs",
                 prop_delay_ms);
    cmd.AddValue("switchBufferBytes",
                 "Shared bottleneck egress buffer in bytes; 0 computes switchBufferBdp",
                 switch_buffer_bytes);
    cmd.AddValue("switchBufferBdp",
                 "Auto switch buffer size in path BDP units",
                 switch_buffer_bdp);
    cmd.AddValue("endpointQueueBytes",
                 "Large endpoint/access queue byte limit",
                 endpoint_queue_bytes);
    cmd.AddValue("flowSizeBytes",
                 "Per-flow send limit in bytes; 0 means unlimited",
                 flow_size_bytes);
    cmd.AddValue("processIntervalUs",
                 "DqcSender polling interval when engine timer is disabled",
                 process_interval_us);
    cmd.AddValue("goodputIntervalMs",
                 "DqcReceiver goodput trace interval",
                 goodput_interval_ms);
    cmd.AddValue("useEngineTimer",
                 "Use DQC engine alarm timer",
                 use_engine_timer);
    cmd.AddValue("enableTrace", "Enable DQC trace files", enable_trace);
    cmd.AddValue("enableHeavyTrace",
                 "Enable RTT/BW/send-rate/recv-rate/inflight/queue traces",
                 enable_heavy_trace);
    cmd.AddValue("enableQueueTrace",
                 "Enable switch egress queue occupancy trace",
                 enable_queue_trace);
    cmd.AddValue("emitRunMeta", "Emit run_meta.json", emit_run_meta);
    cmd.AddValue("emitBottleneckQueueTrace",
                 "Emit fixed-interval bottleneck_queue.csv",
                 emit_bottleneck_queue_trace);
    cmd.AddValue("ackTimingJitterUs",
                 "Seeded per-flow receiver-access timing offset in microseconds; 0 disables",
                 ack_timing_jitter_us);
    cmd.AddValue("ackJitterIntervalMs",
                 "Legacy ACK-jitter interval metadata (runtime delay changes are avoided)",
                 ack_jitter_interval_ms);
    cmd.AddValue("queueSampleIntervalUs",
                 "Fixed bottleneck queue sampling interval",
                 queue_sample_interval_us);
    cmd.AddValue("enableEquivalenceAudit",
                 "Emit sent/acked/pacing audit CSV for shadow A/B regression",
                 enable_equivalence_audit);
    cmd.AddValue("emulatedConnections",
                 "DQC emulated connections per sender",
                 emulated_connections);
    cmd.AddValue("dataGeneratorBatch",
                 "Packets written to DQC stream per fill callback",
                 data_generator_batch);
    cmd.AddValue("packetSizeVariationBytes",
                 "Seeded deterministic application chunk-size variation; 0 disables",
                 packet_size_variation_bytes);
    cmd.AddValue("streamBufferBytes",
                 "Optional DQC stream send buffer size; 0 keeps default",
                 stream_buffer_bytes);
    cmd.AddValue("enableConvergenceGateTrace",
                 "Enable FBBR convergence gate CSV trace",
                 enable_convergence_gate_trace);
    cmd.AddValue("enableConvergenceGateControl",
                 "Gate FBBR CRUISE modulation by BBR stability",
                 enable_convergence_gate_control);
    cmd.AddValue("gateTraceMode",
                 "FBBR gate trace mode: off, round_only, sampled_pacing, full",
                 gate_trace_mode);
    cmd.AddValue("gateTraceSampleIntervalUs",
                 "Minimum interval for sampled FBBR gate trace rows",
                 gate_trace_sample_interval_us);
    cmd.AddValue("fbbrConfig", "Default FBBR config path", fbbr_config);
    cmd.AddValue("obbrConfig", "oBBR config path metadata", obbr_config);
    cmd.AddValue("bbrv2plusConfig",
                 "BBRv2plus config path metadata",
                 bbrv2plus_config);
    cmd.AddValue("bbrv2Config", "BBRv2 config path metadata", bbrv2_config);
    cmd.AddValue("tracePath", "Output trace directory", trace_path);
    cmd.AddValue("trace_path", "Alias of tracePath", trace_path);
    cmd.AddValue("outputDir", "Alias of tracePath", output_dir);
    cmd.AddValue("traceName", "Trace filename prefix", trace_name);
    cmd.AddValue("seed", "ns-3 RNG seed", seed);
    cmd.AddValue("runId", "ns-3 RNG run id", run_id);
    cmd.Parse(argc, argv);

    NS_ABORT_MSG_IF(n_flows == 0, "nFlows must be > 0");
    NS_ABORT_MSG_IF(sim_time_s <= 0.0, "simTime must be > 0");
    NS_ABORT_MSG_IF(switch_buffer_bdp < 0.0,
                    "switchBufferBdp must be non-negative");
    NS_ABORT_MSG_IF(endpoint_queue_bytes == 0,
                    "endpointQueueBytes must be > 0");
    NS_ABORT_MSG_IF(process_interval_us <= 0,
                    "processIntervalUs must be > 0");
    NS_ABORT_MSG_IF(goodput_interval_ms == 0,
                    "goodputIntervalMs must be > 0");
    NS_ABORT_MSG_IF(emulated_connections == 0,
                    "emulatedConnections must be > 0");
    NS_ABORT_MSG_IF(data_generator_batch == 0,
                    "dataGeneratorBatch must be > 0");

    if (prop_delay_ms >= 0.0)
    {
        access_delay_ms = prop_delay_ms;
        service_delay_ms = prop_delay_ms;
    }
    if (!output_dir.empty())
    {
        trace_path = output_dir;
    }
    if (stop_times.empty())
    {
        std::ostringstream stops;
        stops << sim_time_s;
        stop_times = stops.str();
    }
    if (!per_flow_app_rate_limits.empty())
    {
        initial_rates = per_flow_app_rate_limits;
    }
    if (!trace_path.empty())
    {
        if (trace_path.back() != '/')
        {
            trace_path.push_back('/');
        }
        EnsureDirectoryExists(trace_path);
        set_dqc_trace_folder(trace_path);
    }
    SetQueueOccupancyTraceFolder(trace_path);

    RngSeedManager::SetSeed(seed);
    RngSeedManager::SetRun(run_id);
    SendPacketManager::SetDeterministicRandomSeed(seed, run_id);

    ScenarioConfig scenario;
    scenario.n_flows = n_flows;
    scenario.sim_time_s = sim_time_s;
    scenario.access_rate = access_rate;
    scenario.service_rate = service_rate;
    scenario.access_rate_bps = ParseRateBps(access_rate, "accessRate");
    scenario.service_rate_bps = ParseRateBps(service_rate, "serviceRate");
    scenario.access_delay_ms = access_delay_ms;
    scenario.service_delay_ms = service_delay_ms;
    scenario.switch_buffer_bdp = switch_buffer_bdp;
    scenario.switch_buffer_bytes = switch_buffer_bytes;
    scenario.endpoint_queue_bytes = endpoint_queue_bytes;
    scenario.flow_size_bytes = flow_size_bytes;
    scenario.process_interval_us = process_interval_us;
    scenario.goodput_interval_ms = goodput_interval_ms;
    scenario.use_engine_timer = use_engine_timer;
    scenario.enable_trace = enable_trace;
    scenario.enable_heavy_trace = enable_heavy_trace;
    scenario.enable_queue_trace = enable_queue_trace;
    scenario.emulated_connections = emulated_connections;
    scenario.data_generator_batch = data_generator_batch;
    scenario.packet_size_variation_bytes = packet_size_variation_bytes;
    scenario.stream_buffer_bytes = stream_buffer_bytes;
    scenario.enable_convergence_gate_trace = enable_convergence_gate_trace;
    scenario.enable_convergence_gate_control = enable_convergence_gate_control;
    scenario.gate_trace_mode = gate_trace_mode;
    scenario.gate_trace_sample_interval_us = gate_trace_sample_interval_us;
    scenario.trace_path = trace_path;
    scenario.trace_name = trace_name;
    scenario.seed = seed;
    scenario.run_id = run_id;
    scenario.emit_run_meta = emit_run_meta;
    scenario.emit_bottleneck_queue_trace = emit_bottleneck_queue_trace;
    scenario.queue_sample_interval_us = queue_sample_interval_us;
    scenario.enable_equivalence_audit = enable_equivalence_audit;
    scenario.ack_timing_jitter_us = ack_timing_jitter_us;
    scenario.ack_jitter_interval_ms = ack_jitter_interval_ms;
    scenario.background_schedule =
        ParseRateSchedule(background_rate_schedule, 0, "backgroundRateSchedule");
    scenario.capacity_schedule =
        ParseRateSchedule(capacity_schedule, scenario.service_rate_bps,
                          "capacitySchedule");

    NS_ABORT_MSG_IF(queue_sample_interval_us == 0,
                    "queueSampleIntervalUs must be > 0");

    if (scenario.switch_buffer_bytes == 0)
    {
        scenario.switch_buffer_bytes =
            ComputeSwitchBufferBytes(scenario.service_rate_bps,
                                     scenario.access_delay_ms,
                                     scenario.service_delay_ms,
                                     scenario.switch_buffer_bdp);
    }

    std::vector<FlowConfig> flows =
        BuildFlowConfigs(n_flows,
                         algos,
                         start_times,
                         stop_times,
                         initial_rates,
                         rate_schedules,
                         config_paths,
                         fbbr_config,
                         obbr_config,
                         bbrv2plus_config,
                         bbrv2_config);

    PrintConfiguration(scenario, flows);

    std::unique_ptr<DqcTraceState> stat(
        new DqcTraceState(scenario.trace_name));
    RunScenario(scenario, flows, stat.get());
    return 0;
}
