// F-BBR event-triggered frequency-search analysis core.
//
// This file contains the deterministic, transport-independent analysis core.
// Sender integration and phase-bin acquisition live in fbbr_sender.*.

#ifndef FBBR_FREQUENCY_SEARCH_H_
#define FBBR_FREQUENCY_SEARCH_H_

#include <array>
#include <cstdint>
#include <ostream>
#include <string>
#include <vector>

namespace dqc {

enum class FbbrOperatingPointClassification {
  kInvalid,
  kUnderload,
  kNearOptimal,
  kQueuedOverload,
  kBufferSaturated,
  kDynamic,
};

const char* FbbrOperatingPointClassificationName(
    FbbrOperatingPointClassification classification);

struct FBBRFrequencySearchConfig {
  bool frequency_search_enabled = true;
  bool legacy_spectral_path_enabled = false;
  std::vector<uint32_t> probe_period_rtt_slots{4, 6, 8, 10, 12};
  uint32_t probe_code_length_cycles = 4;
  double probe_target_amplitude_ratio = 0.05;
  double probe_min_amplitude_ratio = 0.005;
  double probe_max_amplitude_ratio = 0.15;
  double probe_queue_budget_bdp = 0.10;

  uint32_t warmup_cycles = 1;
  uint32_t analysis_cycles = 4;
  uint32_t min_valid_cycles = 3;
  uint32_t phase_bins_per_cycle = 16;
  double min_bin_coverage = 0.90;
  double min_non_app_limited_fraction = 0.95;
  double max_input_amplitude_error_ratio = 0.20;
  double min_input_cycle_coherence = 0.85;
  double max_native_baseline_drift = 0.10;

  double delay_search_ratio_min = 0.75;
  double delay_search_ratio_max = 1.50;
  double delay_search_ratio_step = 0.05;
  double max_cross_block_delay_shift_ratio = 0.10;
  double ridge_epsilon = 1e-8;
  double max_condition_number = 1e8;
  double min_detectable_snr = 3.0;
  double min_measurement_confidence = 0.75;

  double utility_delay_weight = 1.0;
  double q_zero_ratio = 0.05;
  double q_zero_absolute_s = 0.000001;
  double q_probe_max_ratio = 0.25;
  double min_drain_ratio = 0.75;
  double max_queue_trend_per_cycle = 0.10;
  double max_probe_q95_ratio = 0.50;
  double max_loss_ratio_for_trusted = 0.005;
  double max_ecn_ratio_for_trusted = 0.02;

  double gradient_zero_soft = 0.05;
  double gradient_zero_hard = 0.08;
  double positive_delivery_gain_saturated = 0.45;
  double positive_delivery_gain_transition = 0.65;
  double positive_delivery_gain_underload = 0.70;
  double queue_build_gain_threshold = 0.15;

  double min_full_score = 0.65;
  double min_low_queue_score = 0.75;
  double min_stationary_score = 0.70;
  double min_safe_score = 0.80;
  double min_optimality_score = 0.70;

  uint32_t min_plateau_bins_per_cycle = 2;
  uint32_t min_plateau_cycles_per_block = 4;
  double max_candidate_robust_cv = 0.08;
  double max_candidate_relative_ci_width = 0.15;
  double max_interblock_candidate_diff = 0.10;

  double history_small_change = 0.10;
  double history_medium_change = 0.25;
  double medium_change_min_confidence = 0.85;
  double large_change_min_confidence = 0.90;
  uint32_t trace_verbosity = 1;

  // Validation-only overlapping analysis.  Results are trace-only and must
  // never enter candidate consensus, history stabilization, or pacing.
  bool validation_shadow_windows = false;
  uint32_t validation_shadow_analysis_cycles = 6;
  uint32_t validation_shadow_stride_cycles = 1;
  uint32_t validation_shadow_min_valid_cycles = 5;

  // Production fast-probe/slow-baseline search.
  bool search_controller_enabled = true;
  bool persistent_across_cruises = true;
  bool never_disable_on_unresolved = true;
  uint32_t carrier_sensing_cycles = 2;
  uint32_t pulser_backoff_max_cycles = 4;
  uint32_t pulser_lease_decisions = 3;
  double watcher_probe_amplitude = 0.005;
  uint32_t settling_cycles_per_window = 1;
  uint32_t analysis_cycles_per_window = 12;
  uint32_t min_valid_analysis_cycles = 6;
  uint32_t decision_stride_cycles = 2;
  double forgetting_factor = 0.90;
  uint32_t min_control_windows = 3;
  uint32_t target_control_windows = 5;
  uint32_t max_control_windows = 8;
  uint32_t max_cruise_extension_rtts = 160;
  double max_cruise_extension_s = 10.0;

  double carrier_period_min_s = 0.02;
  double carrier_period_max_s = 1.00;
  double min_extra_probe_mss = 8.0;
  std::string probe_waveform = "asymmetric_zero_mean";
  double actual_amplitude_target_acquire = 0.0325;
  double actual_amplitude_target_seek = 0.0325;
  double actual_amplitude_target_track = 0.015;
  double actual_amplitude_target_drain = 0.0075;
  double actual_amplitude_target_locked = 0.0075;
  double amplitude_adaptation_mu = 0.25;
  double amplitude_step_max = 0.25;
  double probe_queue_budget_acquire_bdp = 0.10;
  double probe_queue_budget_track_bdp = 0.05;
  double probe_queue_budget_drain_bdp = 0.02;
  double carrier_detection_snr_min = 2.0;
  double carrier_detection_amplitude_min = 0.001;
  // realized_amplitude_ratio_* are diagnostics-only compatibility knobs.
  // Measurability is decided from the actual carrier energy/SNR below.
  double realized_amplitude_ratio_min = 0.75;
  double realized_amplitude_ratio_max = 1.25;
  double input_coherence_min = 0.0;
  double input_snr_min = 2.0;
  double max_cwnd_limited_fraction = 1.0;
  double max_app_limited_fraction = 0.20;
  double max_recovery_fraction = 0.10;
  double phase_coverage_min = 0.75;
  double baseline_drift_max = 0.05;
  double regression_condition_max = 1e5;
  double actual_input_min_ratio = 0.005;
  double max_signature_leakage = 0.35;
  double max_residual_to_carrier_ratio = 1.50;

  double measurement_confidence_update_min = 0.45;
  double measurement_confidence_track_min = 0.65;
  double measurement_confidence_lock_min = 0.80;
  double near_optimal_score_threshold = 0.70;
  double near_optimal_direction_abs_max = 0.20;
  double utility_gradient_scale = 0.10;
  double utility_loss_weight = 20.0;
  double utility_ecn_weight = 5.0;

  double ordinary_up_step_max = 0.05;
  double confirmed_up_step_max = 0.08;
  double ordinary_down_step_max = 0.10;
  double track_step_max = 0.02;
  double q_floor_enter_drain_ratio = 0.08;
  double q_floor_exit_drain_ratio = 0.04;
  double drain_baseline_beta = 0.90;
  double drain_delivery_beta = 0.95;
  uint32_t underload_confirmation_windows = 2;
  double same_direction_streak_multiplier_max = 1.50;
  double bracket_lock_width = 0.05;
  uint32_t bracket_ttl_windows = 4;
  double min_search_scale = 0.20;
  double max_search_scale = 1.50;

  double soft_loss_threshold = 0.005;
  double hard_loss_threshold = 0.020;
  double hard_loss_beta = 0.70;
  double soft_ecn_threshold = 0.020;
  double hard_ecn_threshold = 0.100;
  double dynamic_native_change_threshold = 0.10;
  double dynamic_delivery_trend_threshold = 0.15;
  double dynamic_delay_shift_threshold = 0.15;
  uint32_t dynamic_reset_windows = 2;

  uint32_t trusted_candidate_min_windows = 2;
  double trusted_candidate_cv_max = 0.05;
  double trusted_candidate_ratio_max = 1.10;
  uint32_t trusted_ttl_cruises = 3;
  uint32_t bracket_ttl_cruises = 3;
  double provisional_native_change_max = 0.10;
  double provisional_rtprop_change_max = 0.10;
  double rtprop_confidence_lock_min = 0.60;

  // Event-triggered dynamic windows. The trigger cycle is always excluded
  // from estimation, control, and trusted-bandwidth consensus.
  bool event_triggered_windows_enabled = true;
  double delivery_trigger_prominence_start = 2.0;
  double delivery_trigger_prominence_continue = 1.5;
  double delivery_trigger_match_start = 0.50;
  double delivery_trigger_match_continue = 0.35;
  double delivery_trigger_min_response_mss = 4.0;
  double queue_trigger_prominence_start = 2.0;
  double queue_trigger_prominence_continue = 1.5;
  double queue_trigger_match_start = 0.50;
  double queue_trigger_match_continue = 0.35;
  double queue_trigger_noise_mad_multiplier = 3.0;
  double queue_trigger_min_timestamp_quanta = 2.0;
  double trigger_period_tolerance_start = 0.15;
  double trigger_period_tolerance_continue = 0.20;
  double trigger_period_tolerance_ratio = 0.15;
  double trigger_spectral_prominence_min = 2.0;
  double continue_spectral_prominence_min = 1.5;
  double trigger_normalized_match_min = 0.50;
  double continue_normalized_match_min = 0.35;
  double min_delivery_response_bytes_per_cycle_mss = 4.0;
  double trigger_phase_coverage_min = 0.75;

  uint32_t min_direction_cycles = 4;
  uint32_t min_score_cycles = 6;
  uint32_t target_window_cycles = 8;
  uint32_t max_window_cycles = 12;
  uint32_t tracking_window_cycles = 8;
  double tracking_stride_cycles = 0.5;
  double diagnostic_stride_cycles = 0.25;
  double control_decision_stride_cycles = 1.0;
  uint32_t trusted_independent_stride_cycles = 4;
  uint32_t bad_cycles_to_pause = 2;
  uint32_t post_update_settling_cycles = 1;
  double sequential_score_delta_max = 0.10;
  double direction_evidence_min = 0.75;

  double q_reserve_low_bdp = 0.02;
  double q_reserve_high_bdp = 0.05;
  double q_peak_cap_bdp = 0.10;
  double channel_split_weight = 0.60;
  double utility_gradient_weight = 0.40;
  double slow_frequency_weight = 0.60;
  double slow_queue_weight = 0.25;
  double slow_trend_weight = 0.15;
  double direction_frequency_weight = 0.55;
  double direction_queue_weight = 0.35;
  double direction_trend_weight = 0.10;

  double period_increase_factor = 1.25;
  double amplitude_increase_factor = 1.20;
  double max_amplitude_change_per_update = 0.20;
  double acquire_probe_budget_bdp = 0.10;
  double track_probe_budget_bdp = 0.05;
  double locked_probe_budget_bdp = 0.02;

  double mild_drain_step_min = 0.02;
  double mild_drain_step_max = 0.05;
  double mild_drain_delivery_floor = 0.98;
  uint32_t min_pulser_lease_cycles = 8;
  uint32_t max_pulser_lease_cycles = 32;

  double queue_servo_update_rtts = 1.0;
  bool queue_servo_enabled = true;
  double queue_servo_high_gain = 0.50;
  double queue_servo_trend_gain = 0.10;
  double queue_servo_low_gain = 0.25;
  double queue_servo_down_step_max = 0.05;
  double queue_servo_up_step_max = 0.02;
  double queue_servo_recovery_step_max = 0.02;
  double queue_servo_delivery_drain_factor = 0.99;
  double queue_servo_hard_queue_multiple = 4.0;
  double queue_servo_emergency_factor = 0.70;
  uint32_t queue_servo_commit_min_rtts = 4;
  double queue_servo_commit_step_max = 0.02;
};

// Parses canonical f_bbr.* keys. Historical keys are accepted only as input
// compatibility aliases and are never emitted by F-BBR.
// Returns true only when the key is recognized and the value is valid.
bool SetFBBRFrequencySearchConfigValue(FBBRFrequencySearchConfig* config,
                             const std::string& key,
                             const std::string& value);

struct FbbrProbeSignature {
  uint64_t flow_identity = 0;
  uint64_t cruise_id = 0;
  uint32_t period_slot = 0;
  uint32_t period_rtts = 0;
  uint32_t code_id = 0;
  double initial_phase_rad = 0.0;
  double rtprop_s = 0.0;
  double period_s = 0.0;
  double frequency_hz = 0.0;
  double amplitude_ratio = 0.0;
  std::string waveform = "asymmetric_zero_mean";
};

struct FbbrPhaseBinAccumulator {
  int64_t bin_index = -1;
  double time_start_s = 0.0;
  double time_end_s = 0.0;
  uint64_t sent_bytes = 0;
  uint64_t acked_bytes = 0;
  uint64_t lost_bytes = 0;
  uint64_t ecn_marked_bytes = 0;
  uint64_t app_limited_sent_bytes = 0;
  uint64_t app_limited_acked_bytes = 0;
  uint64_t cwnd_limited_sent_bytes = 0;
  uint64_t recovery_sent_bytes = 0;
  uint64_t recovery_acked_bytes = 0;
  double native_pacing_bps_bytes = 0.0;
  double commanded_pacing_bps_bytes = 0.0;
  double bytes_in_flight_bytes = 0.0;
  double queue_servo_factor_bytes = 0.0;
  uint64_t send_events = 0;
  uint64_t ack_events = 0;
  std::vector<double> latest_rtt_samples_s;
  bool phase_transition = false;
  bool queue_servo_transition = false;
};

struct FbbrPhaseBinSample {
  int64_t bin_index = -1;
  int64_t cycle_index = -1;
  uint32_t phase_bin_index = 0;
  double time_start_s = 0.0;
  double time_end_s = 0.0;
  double phase_rad = 0.0;
  int code_sign = 1;
  double coded_excitation = 0.0;
  double native_pacing_bps = 0.0;
  double commanded_pacing_bps = 0.0;
  double actual_send_bps = 0.0;
  double delivery_rate_bps = 0.0;
  uint64_t sent_bytes = 0;
  uint64_t acked_bytes = 0;
  uint64_t lost_bytes = 0;
  uint64_t ecn_marked_bytes = 0;
  double bytes_in_flight = 0.0;
  double queue_servo_factor = 1.0;
  double latest_rtt_s = 0.0;
  double qdelay_s = 0.0;
  double loss_ratio = 0.0;
  double ecn_ratio = 0.0;
  double app_limited_fraction = 0.0;
  double cwnd_limited_fraction = 0.0;
  double recovery_fraction = 0.0;
  double coverage = 0.0;
  bool rtt_valid = false;
  bool valid = false;
  bool phase_transition = false;
  bool queue_servo_transition = false;
};

enum class EventWindowState {
  kIdleListen,
  kTriggerArmed,
  kCapture,
  kContinuousTrack,
  kPaused,
  kPostBaselineSettling,
};

const char* EventWindowStateName(EventWindowState state);

struct FBBRTriggerCycleResult {
  uint64_t cruise_id = 0;
  int64_t cycle_id = -1;
  double cycle_start_s = 0.0;
  double cycle_end_s = 0.0;
  EventWindowState window_state = EventWindowState::kIdleListen;
  bool is_pulser = false;
  double carrier_period_s = 0.0;
  double commanded_amplitude_ratio = 0.0;
  double actual_input_amplitude_ratio = 0.0;
  double actual_input_energy = 0.0;
  double actual_input_snr = 0.0;
  double delivery_response_amplitude_bps = 0.0;
  double delivery_response_bytes = 0.0;
  double delivery_period_estimate_s = 0.0;
  double delivery_period_error_ratio = 0.0;
  double delivery_spectral_prominence = 0.0;
  double delivery_normalized_match = 0.0;
  bool delivery_trigger_pass = false;
  bool delivery_continue_pass = false;
  std::string delivery_reason = "insufficient_data";
  double queue_derivative_amplitude = 0.0;
  double queue_period_estimate_s = 0.0;
  double queue_period_error_ratio = 0.0;
  double queue_spectral_prominence = 0.0;
  double queue_normalized_match = 0.0;
  double queue_noise_floor = 0.0;
  bool queue_trigger_pass = false;
  bool queue_continue_pass = false;
  std::string queue_reason = "insufficient_data";
  double period_estimate_s = 0.0;
  double period_error_ratio = 0.0;
  double spectral_prominence = 0.0;
  double normalized_match = 0.0;
  double selected_delay_s = 0.0;
  double phase_coverage = 0.0;
  double app_limited_fraction = 0.0;
  double recovery_fraction = 0.0;
  double baseline_drift = 0.0;
  bool actual_input_measurable = false;
  bool period_match = false;
  bool trigger_pass = false;
  bool continue_pass = false;
  bool weak_periodic_response = false;
  std::string combined_trigger_source = "NONE";
  double combined_confidence = 0.0;
  double detected_cycle_start_s = 0.0;
  double alignment_error_cycles = 0.0;
  bool hard_safety = false;
  std::string trigger_reason = "insufficient_data";
  std::string pause_reason = "none";
};

struct FbbrHarmonicFitResult {
  bool valid = false;
  std::array<double, 7> beta{{0, 0, 0, 0, 0, 0, 0}};
  std::array<double, 7> standard_error{{0, 0, 0, 0, 0, 0, 0}};
  double condition_number = 0.0;
  double r_squared = 0.0;
  double residual_variance = 0.0;
  double snr = 0.0;
  uint32_t sample_count = 0;
  std::string invalid_reason;
};

struct FbbrSignalResponseMetrics {
  double delivery_gain = 0.0;
  double queue_storage_gain = 0.0;
  double delivery_phase_rad = 0.0;
  double queue_phase_rad = 0.0;
  double utility_phase_rad = 0.0;
  double delivery_harmonic_ratio = 0.0;
  double queue_harmonic_ratio = 0.0;
};

struct FbbrTrustedBwCandidate {
  bool valid = false;
  double bandwidth_bps = 0.0;
  double robust_cv = 0.0;
  double relative_ci_width = 0.0;
  uint32_t cycle_count = 0;
  std::string invalid_reason;
};

struct FbbrOperatingPointBlockResult {
  uint64_t cruise_id = 0;
  uint64_t block_id = 0;
  double start_time_s = 0.0;
  double end_time_s = 0.0;
  double frequency_hz = 0.0;
  uint32_t period_rtts = 0;
  uint32_t code_id = 0;
  double initial_phase_rad = 0.0;
  double target_amplitude_ratio = 0.0;
  double actual_input_amplitude_ratio = 0.0;
  // Actual/commanded fundamental amplitude.  A value of one is ideal.
  double realized_amplitude_ratio = 0.0;
  double rtprop_frozen_s = 0.0;
  double selected_delay_s = 0.0;
  double rtprop_confidence = 0.0;
  double delay_ratio = 0.0;
  bool delay_at_search_boundary = false;
  bool cross_block_delay_stable = true;

  uint32_t valid_cycles = 0;
  double phase_bin_coverage = 0.0;
  double non_app_limited_fraction = 0.0;
  double cwnd_limited_fraction = 0.0;
  double recovery_fraction = 0.0;
  double input_cycle_coherence = 0.0;
  double input_carrier_snr = 0.0;
  double signature_leakage = 0.0;
  double residual_to_own_carrier_ratio = 0.0;
  bool collision_suspected = false;
  double native_baseline_drift = 0.0;
  double delivery_baseline_drift = 0.0;
  double loss_ratio = 0.0;
  double ecn_ratio = 0.0;

  double confidence_coverage = 0.0;
  double confidence_input = 0.0;
  double confidence_stationarity = 0.0;
  double confidence_cycle = 0.0;
  double confidence_response = 0.0;
  double confidence_delivery_channel = 0.0;
  double confidence_queue_channel = 0.0;
  double confidence_delay = 0.0;
  double confidence_regression = 0.0;
  double measurement_confidence = 0.0;

  FbbrHarmonicFitResult input_fit;
  FbbrHarmonicFitResult delivery_fit;
  FbbrHarmonicFitResult queue_fit;
  FbbrHarmonicFitResult utility_fit;
  FbbrSignalResponseMetrics response;

  double positive_delivery_gain = 0.0;
  double positive_queue_build_s = 0.0;
  double positive_queue_build_gain = 0.0;
  double q_zero_s = 0.0;
  double q_probe_max_s = 0.0;
  double q_floor_s = 0.0;
  double q95_s = 0.0;
  double q_amplitude_s = 0.0;
  double drain_ratio = 0.0;
  double queue_trend_per_cycle = 0.0;
  double effective_cycles = 0.0;
  double queue_servo_factor_mean = 1.0;
  uint32_t queue_servo_transition_cycles = 0;

  uint64_t event_window_id = 0;
  int64_t trigger_cycle_id = -1;
  double capture_start_s = 0.0;
  double capture_end_s = 0.0;
  double window_length_cycles = 0.0;
  double window_stride_cycles = 0.0;
  std::string sequential_stop_reason = "NONE";
  EventWindowState event_window_state = EventWindowState::kIdleListen;
  bool trigger_cycle_excluded_from_score = true;
  double overlap_fraction = 0.0;
  bool independent_for_control = true;
  bool independent_for_trusted = true;
  bool lockable_score = false;

  double q_reserve_low_s = 0.0;
  double q_reserve_high_s = 0.0;
  double q_peak_cap_s = 0.0;
  double saturation_score = 0.0;
  double queue_band_score = 0.0;
  double queue_stability_score = 0.0;
  double target_score = 0.0;
  double frequency_direction = 0.0;
  double queue_band_error = 0.0;
  double total_direction = 0.0;
  double delivery_spectral_prominence = 0.0;
  double delivery_normalized_match = 0.0;
  double queue_spectral_prominence = 0.0;
  double queue_normalized_match = 0.0;
  double delivery_partition = 0.5;
  double queue_partition = 0.5;
  std::string trigger_source = "NONE";

  double gradient_lockin = 0.0;
  double gradient_finite_difference = 0.0;
  double gradient_fused = 0.0;
  double curvature_finite_difference = 0.0;
  bool gradient_agreement = false;
  double gradient_se = 0.0;
  double gradient_ci90_low = 0.0;
  double gradient_ci90_high = 0.0;
  double gradient_ci95_low = 0.0;
  double gradient_ci95_high = 0.0;
  bool gradient_equivalent = false;
  double delivery_median_bps = 0.0;

  double full_score = 0.0;
  double low_queue_score = 0.0;
  double stationary_score = 0.0;
  double safe_score = 0.0;
  double raw_optimality_score = 0.0;
  double optimality_score = 0.0;
  double underload_evidence = 0.0;
  double overload_evidence = 0.0;
  double direction_score = 0.0;
  double rate_adjustment_signal = 0.0;
  FbbrOperatingPointClassification classification =
      FbbrOperatingPointClassification::kInvalid;
  FbbrTrustedBwCandidate candidate;
  std::string invalid_reason;
};

enum class FBBRSearchState {
  kDisabled,
  kAcquireInput,
  kPulserElection,
  kWatcher,
  kDrain,
  kSeek,
  kTrack,
  kLockCandidate,
  kLocked,
  kEmergencyDrain,
  kDynamicReacquire,
  kPersistentUnresolved,
};

const char* FBBRSearchStateName(FBBRSearchState state);

enum class FBBRQueueServoState {
  kHold,
  kDrain,
  kReserveRecovery,
  kTargetBand,
  kEmergencyDrain,
};

const char* FBBRQueueServoStateName(FBBRQueueServoState state);

struct FBBRQueueServoInput {
  double search_baseline_bps = 0.0;
  double rtprop_s = 0.0;
  double q_floor_s = 0.0;
  double q_median_s = 0.0;
  double q_peak_s = 0.0;
  double queue_trend_s_per_s = 0.0;
  double delivery_median_bps = 0.0;
  double loss_ratio = 0.0;
  double ecn_ratio = 0.0;
  double sustainable_direction = 0.0;
  bool samples_sufficient = false;
  bool in_recovery = false;
  bool flow_backlogged = true;
  bool underload_evidence = false;
};

struct FBBRQueueServoStateData {
  FBBRQueueServoState state = FBBRQueueServoState::kHold;
  double factor = 1.0;
  uint32_t consecutive_drain_rtts = 0;
  uint32_t saturated_down_rtts = 0;
};

struct FBBRQueueServoDecision {
  FBBRQueueServoState state = FBBRQueueServoState::kHold;
  double factor = 1.0;
  double final_nonprobe_baseline_bps = 0.0;
  double q_low_s = 0.0;
  double q_high_s = 0.0;
  double q_peak_cap_s = 0.0;
  double down_correction = 0.0;
  double up_correction = 0.0;
  uint32_t consecutive_drain_rtts = 0;
  bool baseline_commit_eligible = false;
  double baseline_commit_bps = 0.0;
  std::string reason = "HOLD";
};

class FBBRQueueReserveServo {
 public:
  static FBBRQueueServoDecision Update(
      const FBBRFrequencySearchConfig& config,
      const FBBRQueueServoInput& input,
      FBBRQueueServoStateData* state);
};

struct FBBRWindowControlDecision {
  bool measurement_valid = false;
  double measurement_confidence = 0.0;
  double raw_optimality_score = 0.0;
  double reported_optimality_score = 0.0;
  double direction_score = 0.0;
  double underload_evidence = 0.0;
  double overload_evidence = 0.0;
  FbbrOperatingPointClassification classification =
      FbbrOperatingPointClassification::kInvalid;
  FBBRSearchState state_before = FBBRSearchState::kDisabled;
  FBBRSearchState state_after = FBBRSearchState::kDisabled;

  double baseline_before_bps = 0.0;
  double proposed_baseline_bps = 0.0;
  double applied_next_baseline_bps = 0.0;
  double log_step = 0.0;
  std::string update_reason = "NONE";

  bool hard_loss_abort = false;
  bool bracket_updated = false;
  bool lock_candidate = false;
  bool locked = false;
  bool search_range_exhausted = false;
  bool request_period_increase = false;
  bool request_amplitude_increase = false;
  bool request_amplitude_decrease = false;
  bool trusted_bw_published = false;
  double trusted_bw_bps = 0.0;
  double trusted_confidence = 0.0;
  double window_candidate_bps = 0.0;
  bool window_candidate_valid = false;
  bool carrier_detected = false;
  double carrier_sense_snr = 0.0;
  double carrier_sense_amplitude = 0.0;
  uint32_t carrier_detection_streak = 0;
  int64_t last_carrier_sense_bin = -1;
  bool collision_suspected = false;
};

struct FBBRSearchControllerState {
  FBBRSearchState state = FBBRSearchState::kDisabled;
  double cruise_entry_native_bps = 0.0;
  double current_search_bps = 0.0;
  double pending_search_bps = 0.0;
  double native_bps_when_bracket_created = 0.0;
  double rtprop_s_when_bracket_created = 0.0;

  bool underload_bound_valid = false;
  double underload_bound_bps = 0.0;
  bool overload_bound_valid = false;
  double overload_bound_bps = 0.0;
  uint32_t bracket_generation = 0;
  uint32_t bracket_age_windows = 0;

  int same_direction_streak = 0;
  int previous_direction_sign = 0;
  uint32_t consecutive_dynamic = 0;
  uint32_t consecutive_invalid = 0;
  uint32_t consecutive_near_optimal = 0;
  uint32_t control_window_index = 0;
  uint32_t locked_validation_windows = 0;
  uint32_t input_unrealized_streak = 0;
  uint32_t unresolved_decisions = 0;
  uint32_t unresolved_cruises = 0;
  uint32_t eligible_cruises = 0;
  uint32_t search_attempts = 0;
  uint32_t valid_direction_decisions = 0;
  bool search_active = true;
  bool is_pulser = true;
  uint32_t pulser_lease_remaining = 0;
  uint32_t election_backoff_cycles = 0;
  uint32_t watcher_cooldown_cycles = 0;
  int64_t election_start_cycle = 0;
  uint32_t pulser_lease_count = 0;
  uint32_t collision_count = 0;
  double carrier_sense_snr = 0.0;
  double carrier_sense_amplitude = 0.0;
  uint32_t carrier_detection_streak = 0;
  int64_t last_carrier_sense_bin = -1;
  uint64_t search_generation = 0;
  std::string last_failure_reason = "none";

  double current_amplitude_ratio = 0.0;
  double carrier_period_s = 0.0;
  uint32_t carrier_scan_index = 0;
  bool carrier_detected = false;
  bool collision_suspected = false;
  bool provisional_validation_pending = false;
  uint32_t provisional_age_cruises = 0;
  std::vector<double> trusted_candidates_bps;
  std::vector<double> trusted_candidate_confidence;
  std::vector<uint64_t> trusted_candidate_window_ids;
  uint64_t last_control_window_id = 0;
};

class FBBRSearchController {
 public:
  static FBBRSearchControllerState Initialize(
      double native_baseline_bps,
      double amplitude_ratio,
      double carrier_period_s);

  static FBBRWindowControlDecision Decide(
      const FBBRFrequencySearchConfig& config,
      const FbbrOperatingPointBlockResult& window,
      double current_native_bps,
      double rtprop_anchor_s,
      FBBRSearchControllerState* state);

  static double RaisedCosineLogRamp(double from_bps,
                                    double to_bps,
                                    double progress);
};

struct FbbrCruiseConsensusResult {
  bool valid = false;
  double raw_candidate_bps = 0.0;
  double confidence = 0.0;
  double robust_cv = 0.0;
  double relative_ci_width = 0.0;
  uint32_t block_count = 0;
  uint32_t cycle_count = 0;
  uint64_t source_block_id = 0;
  std::string invalid_reason;
};

struct FbbrHistoryUpdateResult {
  bool valid = false;
  double raw_candidate_bps = 0.0;
  double published_bps = 0.0;
  double confidence = 0.0;
  std::string action;
  std::string invalid_reason;
};

class FBBRFrequencySearch {
 public:
  static FBBRTriggerCycleResult AnalyzeTriggerCycle(
      const FBBRFrequencySearchConfig& config,
      const FbbrProbeSignature& signature,
      const std::vector<FbbrPhaseBinSample>& bins,
      int64_t output_cycle,
      EventWindowState window_state,
      bool is_pulser);

  static FbbrOperatingPointBlockResult AnalyzeBlock(
      const FBBRFrequencySearchConfig& config,
      const FbbrProbeSignature& signature,
      const std::vector<FbbrPhaseBinSample>& bins,
      int64_t first_output_bin,
      int64_t output_bin_count,
      uint64_t block_id,
      double previous_block_delay_s);

  static FbbrCruiseConsensusResult BuildCruiseConsensus(
      const FBBRFrequencySearchConfig& config,
      const std::vector<FbbrOperatingPointBlockResult>& blocks);

  static FbbrHistoryUpdateResult StabilizeHistory(
      const FBBRFrequencySearchConfig& config,
      const FbbrCruiseConsensusResult& consensus,
      bool old_valid,
      double old_bandwidth_bps,
      bool clear_native_change_evidence);

  static int WalshSign(uint32_t code_id, int64_t cycle_index);
  static double CodedSine(const FbbrProbeSignature& signature,
                          double time_since_cruise_start_s);
  static double ProbeWaveform(const FbbrProbeSignature& signature,
                              double time_since_cruise_start_s);
  static bool RunSelfTests(std::ostream& os);
};

}  // namespace dqc

#endif  // FBBR_FREQUENCY_SEARCH_H_
