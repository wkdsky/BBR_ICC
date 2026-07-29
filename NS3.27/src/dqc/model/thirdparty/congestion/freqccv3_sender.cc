// FreqCCv3 - Enhanced Frequency-modulated Congestion Control based on modified BBRv2
// This algorithm modifies BBRv2's behavior with NEW_REFILL and oscillation during PROBE_UP

#include "freqccv3_sender.h"

#include <cmath>
#include <algorithm>
#include <limits>
#include <numeric>
#include <vector>
#include <fftw3.h>

#include "quic_logging.h"
#include "quic_bbr2_probe_bw.h"

namespace dqc {

namespace {
// Default oscillation parameters
const double kDefaultOscillationFreqHz = 1.0;  // 1 Hz default
}  // namespace

// ... (keep existing constexpr definitions)
// Define static constexpr members for linking
constexpr float FreqCCv3Sender::kMinUpPacingGain;
constexpr float FreqCCv3Sender::kMaxUpPacingGain;
constexpr float FreqCCv3Sender::kDefaultUpPacingGain;
constexpr double FreqCCv3Sender::kUpDurationVeryLongRttMultiple;
constexpr double FreqCCv3Sender::kUpDurationLongRttMultiple;
constexpr double FreqCCv3Sender::kUpDurationShortRttMultiple;
constexpr float FreqCCv3Sender::kPacingGainAdjustStep;

FreqCCv3Sender::FreqCCv3Sender(QuicTime now,
                               const RttStats* rtt_stats,
                               const QuicUnackedPacketMap* unacked_packets,
                               QuicPacketCount initial_cwnd_in_packets,
                               QuicPacketCount max_cwnd_in_packets,
                               Random* random,
                               QuicConnectionStats* stats,
                               bool enable_ecn)
    : Bbr2Sender(now, rtt_stats, unacked_packets, initial_cwnd_in_packets,
                 max_cwnd_in_packets, random, stats, enable_ecn),
      // Initialize oscillation parameters
      oscillation_freq_hz_(kDefaultOscillationFreqHz),
      current_oscillation_freq_hz_(kDefaultOscillationFreqHz),
      initial_freq_hz_(kDefaultOscillationFreqHz),
      amplitude_mode_(FreqCCv3AmplitudeMode::kFixed),
      fixed_amplitude_bps_(0),  // No oscillation by default
      drain_completed_(false),
      oscillation_start_time_(QuicTime::Zero()),
      last_mode_(Bbr2Mode::STARTUP),
      last_probe_bw_phase_(Bbr2ProbeBwMode::CyclePhase::PROBE_NOT_STARTED),
      current_time_(now),
      last_ack_time_(QuicTime::Zero()),
      new_refill_state_(NewRefillState::kNotInNewRefill),
      in_new_refill_(false),
      interval_window_multiplier_(1.0),
      min_probe_up_duration_rtt_multiplier_(0.0),
      // Initialize adaptive frequency state
      up_phase_count_(0),
      up_phase_start_time_(QuicTime::Zero()),
      last_up_duration_sec_(0.0),
      // Initialize adaptive pacing gain
      up_pacing_gain_(kDefaultUpPacingGain),
      current_up_pacing_gain_(kDefaultUpPacingGain),
      // Initialize bandwidth tracking for early exit handling
      bandwidth_before_up_(QuicBandwidth::Zero()),
      up_phase_exited_early_(false),
      use_delivery_rate_latest_for_signal_history_(false),
      last_down_phase_end_time_(QuicTime::Zero()),
      last_up_phase_end_time_(QuicTime::Zero()),
      last_up_phase_start_time_(QuicTime::Zero()),
      last_up_phase_peak_freq_(0.0),
      last_up_rtt_peak_freq_(0.0),
      last_up_window_size_(TimeDelta::Zero()),
      last_up_rtt_window_size_(TimeDelta::Zero()),
      last_up_phase_valid_(false),
      sender_max_peak_freq_hz_(0.0),
      last_up_sender_template_freq_hz_(0.0),
      last_up_sender_template_valid_(false),
      last_packet_sent_time_(QuicTime::Zero()) {
  QUIC_DVLOG(2) << this << " Initializing FreqCCv3Sender @ " << now;
}

void FreqCCv3Sender::SetOscillationFrequency(double freq_hz) {
  oscillation_freq_hz_ = freq_hz;
  initial_freq_hz_ = freq_hz;  // Store initial frequency for first UP phase
}

void FreqCCv3Sender::SetOscillationAmplitude(FreqCCv3AmplitudeMode mode, uint64_t fixed_bps) {
  amplitude_mode_ = mode;
  fixed_amplitude_bps_ = fixed_bps;
}

void FreqCCv3Sender::SetIntervalWindowMultiplier(double multiplier) {
  if (multiplier > 0.0) {
    interval_window_multiplier_ = multiplier;
  }
}

void FreqCCv3Sender::SetMinProbeUpDurationRttMultiplier(double multiplier) {
  if (multiplier > 0.0) {
    min_probe_up_duration_rtt_multiplier_ = multiplier;
  } else {
    min_probe_up_duration_rtt_multiplier_ = 0.0;
  }
}

void FreqCCv3Sender::SetRecvSignalMode(bool use_delivery_rate_latest) {
  use_delivery_rate_latest_for_signal_history_ = use_delivery_rate_latest;
}

uint64_t FreqCCv3Sender::GetCurrentAmplitudeBps() const {
  QuicBandwidth max_bw = BandwidthEstimate();
  QuicBandwidth base_rate = Bbr2Sender::PacingRate(0);

  switch (amplitude_mode_) {
    case FreqCCv3AmplitudeMode::kFixed:
      return fixed_amplitude_bps_;
    case FreqCCv3AmplitudeMode::kMiu2:
      return max_bw.ToBitsPerSecond() / 2;
    case FreqCCv3AmplitudeMode::kMiu3:
      return max_bw.ToBitsPerSecond() / 3;
    case FreqCCv3AmplitudeMode::kMiu4:
      return max_bw.ToBitsPerSecond() / 4;
    case FreqCCv3AmplitudeMode::kMiu8:
      return max_bw.ToBitsPerSecond() / 8;
    case FreqCCv3AmplitudeMode::kSR2:
      return base_rate.ToBitsPerSecond() / 2;
    case FreqCCv3AmplitudeMode::kSR3:
      return base_rate.ToBitsPerSecond() / 3;
    case FreqCCv3AmplitudeMode::kSR4:
      return base_rate.ToBitsPerSecond() / 4;
    case FreqCCv3AmplitudeMode::kSR8:
      return base_rate.ToBitsPerSecond() / 8;
    default:
      return 0;
  }
}

Bbr2ProbeBwMode::CyclePhase FreqCCv3Sender::GetCurrentProbeBwPhase() const {
  DebugState state = ExportDebugState();
  if (state.mode == Bbr2Mode::PROBE_BW) {
    return state.probe_bw.phase;
  }
  return Bbr2ProbeBwMode::CyclePhase::PROBE_NOT_STARTED;
}

bool FreqCCv3Sender::ShouldOscillate() const {
  if (GetCurrentAmplitudeBps() == 0) {
    return false;
  }

  // FreqCCv3 only injects oscillation during PROBE_UP.
  if (!drain_completed_ || mode_ != Bbr2Mode::PROBE_BW) {
    return false;
  }

  Bbr2ProbeBwMode::CyclePhase current_phase = GetCurrentProbeBwPhase();
  return current_phase == Bbr2ProbeBwMode::CyclePhase::PROBE_UP;
}

int64_t FreqCCv3Sender::CalculateOscillationOffset(QuicTime now) const {
  if (!ShouldOscillate() || oscillation_start_time_ == QuicTime::Zero()) {
    return 0;
  }

  // Calculate time since oscillation started
  TimeDelta elapsed = now - oscillation_start_time_;
  double elapsed_seconds = static_cast<double>(elapsed.ToMicroseconds()) / 1000000.0;

  // Calculate the period of oscillation
  double period = 1.0 / oscillation_freq_hz_;

  // Get position within the current period (0 to 1)
  double phase = fmod(elapsed_seconds, period) / period;

  // Calculate triangle wave: goes from 0 to 1 to -1 to 0 over one period
  // This creates a symmetric triangle wave (like ICC's rate adjustment pattern)
  double triangle_value;
  if (phase < 0.25) {
    // Rising from 0 to 1
    triangle_value = phase * 4.0;
  } else if (phase < 0.75) {
    // Falling from 1 to -1
    triangle_value = 2.0 - phase * 4.0;
  } else {
    // Rising from -1 to 0
    triangle_value = phase * 4.0 - 4.0;
  }

  // Calculate offset based on amplitude
  uint64_t amplitude_bps = GetCurrentAmplitudeBps();
  int64_t offset_bps = static_cast<int64_t>(triangle_value * amplitude_bps);

  return offset_bps;
}

QuicByteCount FreqCCv3Sender::CalculateInflightThreshold(float bdp_factor) const {
  // threshold = bdp_factor * BDP + 2*MSS + MaxAckHeight
  QuicByteCount bdp = model_.BDP(model_.MaxBandwidth());
  QuicByteCount threshold = static_cast<QuicByteCount>(bdp_factor * bdp) +
                            2 * kDefaultTCPMSS +
                            model_.MaxAckHeight();
  return threshold;
}

void FreqCCv3Sender::UpdateNewRefillState(QuicByteCount bytes_in_flight) {
  QuicByteCount high_threshold = CalculateInflightThreshold(kNewRefillHighThreshold);
  QuicByteCount low_threshold = CalculateInflightThreshold(kNewRefillLowThreshold);

  QUIC_DVLOG(3) << "FreqCCv3: UpdateNewRefillState - bytes_in_flight=" << bytes_in_flight
                << ", high_threshold=" << high_threshold
                << ", low_threshold=" << low_threshold
                << ", current_state=" << static_cast<int>(new_refill_state_);

  if (bytes_in_flight > high_threshold) {
    // Inflight > 0.75*BDP + 2*MSS + MaxAckHeight: need to drain
    new_refill_state_ = NewRefillState::kDraining;
    QUIC_DVLOG(3) << "FreqCCv3: Entering kDraining state, pacing_gain=0.75";
  } else if (bytes_in_flight < low_threshold) {
    // Inflight < 0.72*BDP + 2*MSS + MaxAckHeight: need to fill
    new_refill_state_ = NewRefillState::kFilling;
    QUIC_DVLOG(3) << "FreqCCv3: Entering kFilling state, pacing_gain=1.0";
  } else {
    // Neither condition met: done, exit immediately
    new_refill_state_ = NewRefillState::kDone;
    QUIC_DVLOG(3) << "FreqCCv3: Entering kDone state, will exit NEW_REFILL";
  }
}

float FreqCCv3Sender::GetNewRefillPacingGain() const {
  // Always return 1.0 to match BBRv2's refill behavior (requested by user)
  return 1.0f;
}

bool FreqCCv3Sender::ShouldExitNewRefill(QuicByteCount bytes_in_flight) const {
  if (!in_new_refill_) {
    return false;
  }

  QuicByteCount high_threshold = CalculateInflightThreshold(kNewRefillHighThreshold);
  QuicByteCount low_threshold = CalculateInflightThreshold(kNewRefillLowThreshold);

  switch (new_refill_state_) {
    case NewRefillState::kDraining:
      // Exit when inflight <= 0.75*BDP + 2*MSS + MaxAckHeight
      return bytes_in_flight <= high_threshold;

    case NewRefillState::kFilling:
      // Exit when inflight >= 0.72*BDP + 2*MSS + MaxAckHeight
      return bytes_in_flight >= low_threshold;

    case NewRefillState::kDone:
      // Exit immediately
      return true;

    case NewRefillState::kNotInNewRefill:
    default:
      return true;
  }
}

bool FreqCCv3Sender::ShouldDelayProbeUpExit(QuicTime now) const {
  if (min_probe_up_duration_rtt_multiplier_ <= 0.0 ||
      up_phase_start_time_ == QuicTime::Zero()) {
    return false;
  }

  TimeDelta min_rtt = model_.MinRtt();
  if (min_rtt.IsZero()) {
    return false;
  }

  TimeDelta elapsed = now - up_phase_start_time_;
  double required_us = min_probe_up_duration_rtt_multiplier_ *
                       static_cast<double>(min_rtt.ToMicroseconds());
  return static_cast<double>(elapsed.ToMicroseconds()) < required_us;
}

void FreqCCv3Sender::OnPacketSent(QuicTime sent_time,
                                  QuicByteCount bytes_in_flight,
                                  QuicPacketNumber packet_number,
                                  QuicByteCount bytes,
                                  HasRetransmittableData is_retransmittable) {
  // Update current time
  current_time_ = sent_time;

  // Track sender rate: Use the actual pacing rate (which includes oscillation)
  // This correctly captures the oscillating signal during UP phase
  QuicBandwidth sender_rate = PacingRate(bytes_in_flight);
  sender_rate_history_.push_back({sent_time, sender_rate});

  // Safety cap to prevent memory explosion
  while (sender_rate_history_.size() > 20000) sender_rate_history_.pop_front();

  last_packet_sent_time_ = sent_time;

  // Call parent implementation
  Bbr2Sender::OnPacketSent(sent_time, bytes_in_flight, packet_number, bytes, is_retransmittable);
}

void FreqCCv3Sender::OnCongestionEvent(bool rtt_updated,
                                       QuicByteCount prior_in_flight,
                                       QuicTime event_time,
                                       const AckedPacketVector& acked_packets,
                                       const LostPacketVector& lost_packets) {
  // Update current time
  current_time_ = event_time;

  QuicByteCount acked_bytes = 0;
  for (const auto& ack : acked_packets) {
      acked_bytes += ack.bytes_acked;
  }

  // Get phase before parent update
  Bbr2ProbeBwMode::CyclePhase phase_before = GetCurrentProbeBwPhase();

  // Check if we're in PROBE_UP and there are losses or ECN marks
  bool in_probe_up_before = (phase_before == Bbr2ProbeBwMode::CyclePhase::PROBE_UP);
  bool has_loss_or_ecn = (!lost_packets.empty()) || (GetBytesEcnInRounds() > 0);

  // Call parent implementation
  Bbr2Sender::OnCongestionEvent(rtt_updated, prior_in_flight, event_time,
                                acked_packets, lost_packets);

  // STFT: Sample the ACK-driven receive-rate series after parent update.
  // Preserve the legacy BandwidthLatest() path by default, and allow an
  // alternate raw-delivery path for controlled A/B experiments.
  if (last_ack_time_ != QuicTime::Zero() && event_time > last_ack_time_ &&
      acked_bytes > 0) {
      QuicBandwidth recv_signal = use_delivery_rate_latest_for_signal_history_
                                      ? DeliveryRateLatest()
                                      : BandwidthLatest();
      signal_history_.push_back({event_time, recv_signal});
      if (rtt_stats_ != nullptr) {
          TimeDelta smoothed_rtt = rtt_stats_->smoothed_rtt();
          if (smoothed_rtt.IsZero()) {
              smoothed_rtt = rtt_stats_->SmoothedOrInitialRtt();
          }
          if (!smoothed_rtt.IsZero()) {
              rtt_signal_history_.push_back(
                  {event_time, static_cast<double>(smoothed_rtt.ToMicroseconds()) / 1000.0});
          }
      }

      // Safety cap to prevent memory explosion if not cleared
      while (signal_history_.size() > 20000) signal_history_.pop_front();
      while (rtt_signal_history_.size() > 20000) rtt_signal_history_.pop_front();
  }
  last_ack_time_ = event_time;

  // Get current mode and phase after parent update
  Bbr2Mode current_mode = mode_;
  Bbr2ProbeBwMode::CyclePhase current_phase = GetCurrentProbeBwPhase();

  // Detect transition to PROBE_BW (first time after DRAIN)
  if (!drain_completed_) {
    if (current_mode == Bbr2Mode::PROBE_BW) {
      drain_completed_ = true;
    }
  }

  // Handle NEW_REFILL logic
  if (current_mode == Bbr2Mode::PROBE_BW) {
    // Record the exact end of PROBE_DOWN so interval analysis covers CRUISE + REFILL.
    if (phase_before == Bbr2ProbeBwMode::CyclePhase::PROBE_DOWN &&
        current_phase != Bbr2ProbeBwMode::CyclePhase::PROBE_DOWN) {
      last_down_phase_end_time_ = event_time;
    }

    // Detect entering REFILL phase (from CRUISE)
    if (current_phase == Bbr2ProbeBwMode::CyclePhase::PROBE_REFILL &&
        phase_before == Bbr2ProbeBwMode::CyclePhase::PROBE_CRUISE) {
      // Entering NEW_REFILL: determine initial state based on inflight
      in_new_refill_ = true;
      UpdateNewRefillState(prior_in_flight);

      QUIC_DVLOG(2) << "FreqCCv3: Entering NEW_REFILL @ " << event_time
                    << ", inflight=" << prior_in_flight
                    << ", state=" << static_cast<int>(new_refill_state_);

      // Apply the appropriate pacing gain
      model_.set_pacing_gain(GetNewRefillPacingGain());
    }
    // If in NEW_REFILL, check if we should continue or exit
    if (in_new_refill_ && current_phase == Bbr2ProbeBwMode::CyclePhase::PROBE_REFILL) {
      // Get current bytes in flight after processing
      QuicByteCount current_inflight = prior_in_flight;  // Use prior as approximation

      // Check exit conditions based on current state
      if (ShouldExitNewRefill(current_inflight)) {
        QUIC_DVLOG(2) << "FreqCCv3: Exiting NEW_REFILL, state=" << static_cast<int>(new_refill_state_)
                      << ", inflight=" << current_inflight;

        // Reset NEW_REFILL state
        in_new_refill_ = false;
        new_refill_state_ = NewRefillState::kNotInNewRefill;

        // Note: The parent BBRv2 will handle the transition to PROBE_UP
        // when it detects end_of_round_trip in UpdateProbeRefill
      } else {
        // Continue in NEW_REFILL with appropriate pacing gain
        model_.set_pacing_gain(GetNewRefillPacingGain());
      }
    }

    // Detect entering PROBE_UP phase
    // We need to reset oscillation_start_time_ whenever we transition INTO PROBE_UP
    bool in_probe_up = (current_phase == Bbr2ProbeBwMode::CyclePhase::PROBE_UP);
    bool was_in_probe_up_before = (last_probe_bw_phase_ == Bbr2ProbeBwMode::CyclePhase::PROBE_UP);

    // We're entering PROBE_UP if we're in it now but weren't in the last event
    bool entering_probe_up = in_probe_up && !was_in_probe_up_before;

    if (entering_probe_up) {
      // STFT: START of NEW UP Phase = END of Previous Interval
      // Analyze Previous Interval (from last DOWN end to now), i.e. CRUISE + REFILL.
      if (last_up_phase_valid_ && last_down_phase_end_time_ != QuicTime::Zero() &&
          last_down_phase_end_time_ < event_time) {
          // Analyze the interval against the previous UP sender template when available.
          double interval_ref_freq_hz = 0.0;
          double interval_threshold_hz = 0.0;
          if (last_up_sender_template_valid_ && last_up_sender_template_freq_hz_ > 0.0) {
              interval_ref_freq_hz = last_up_sender_template_freq_hz_;
          } else if (last_up_phase_peak_freq_ > 0.0) {
              interval_ref_freq_hz = last_up_phase_peak_freq_;
              interval_threshold_hz = last_up_phase_peak_freq_ * 2.0 / 3.0;
          }

          PerformFreqAnalysis(last_down_phase_end_time_, event_time,
                              interval_threshold_hz, interval_ref_freq_hz);

          double interval_rtt_ref_freq_hz = 0.0;
          double interval_rtt_threshold_hz = 0.0;
          if (last_up_sender_template_valid_ && last_up_sender_template_freq_hz_ > 0.0) {
              interval_rtt_ref_freq_hz = last_up_sender_template_freq_hz_;
          } else if (last_up_rtt_peak_freq_ > 0.0) {
              interval_rtt_ref_freq_hz = last_up_rtt_peak_freq_;
              interval_rtt_threshold_hz = last_up_rtt_peak_freq_ * 2.0 / 3.0;
          }

          PerformRttFreqAnalysis(last_down_phase_end_time_, event_time,
                                 interval_rtt_threshold_hz,
                                 interval_rtt_ref_freq_hz);

          // Clear history older than event_time (Start of this UP)
          while (!signal_history_.empty() && signal_history_.front().time < event_time) {
              signal_history_.pop_front();
          }
          while (!rtt_signal_history_.empty() && rtt_signal_history_.front().time < event_time) {
              rtt_signal_history_.pop_front();
          }
      } else {
          // Invalid or first run, clear history up to now
          while (!signal_history_.empty() && signal_history_.front().time < event_time) {
              signal_history_.pop_front();
          }
          while (!rtt_signal_history_.empty() && rtt_signal_history_.front().time < event_time) {
              rtt_signal_history_.pop_front();
          }
          if (signal_history_.empty()) signal_history_.clear();
          if (rtt_signal_history_.empty()) rtt_signal_history_.clear();
      }

      // DO NOT clear sender_rate_history here! We need it for STFT analysis when leaving UP phase
      // Clearing will be done after STFT analysis in the leaving_probe_up section

      // Reset UP state
      oscillation_start_time_ = event_time;
      up_phase_start_time_ = event_time;
      last_up_phase_start_time_ = event_time;
      last_up_phase_peak_freq_ = 0.0;
      last_up_rtt_peak_freq_ = 0.0;
      last_up_phase_valid_ = false;
      sender_max_peak_freq_hz_ = 0.0;  // NOW reset sender max peak freq AFTER Interval analysis
      last_up_sender_template_freq_hz_ = 0.0;
      last_up_sender_template_valid_ = false;
      last_up_sender_band_template_.clear();

      // Save bandwidth estimate before entering UP phase (for early exit handling)
      bandwidth_before_up_ = model_.MaxBandwidth();
      up_phase_exited_early_ = false;

      QUIC_DVLOG(2) << "FreqCCv3: Entering PROBE_UP @ " << event_time
                    << ", freq=" << oscillation_freq_hz_ << "Hz, up_phase_count=" << up_phase_count_
                    << ", up_pacing_gain=" << up_pacing_gain_
                    << ", amplitude_bps=" << GetCurrentAmplitudeBps()
                    << ", bandwidth_before_up=" << bandwidth_before_up_;

      // Save the frequency and pacing gain that will be used in this UP phase (for tracing later)
      current_oscillation_freq_hz_ = oscillation_freq_hz_;
      current_up_pacing_gain_ = up_pacing_gain_;

      // Apply adaptive pacing gain for PROBE_UP
      model_.set_pacing_gain(up_pacing_gain_);

      // Ensure NEW_REFILL state is reset
      in_new_refill_ = false;
      new_refill_state_ = NewRefillState::kNotInNewRefill;
    }

    // Detect leaving PROBE_UP phase
    // We leave PROBE_UP when we were in it last event but not now
    // Also reset if we're not in PROBE_UP and oscillation_start_time_ is set (cleanup)
    bool leaving_probe_up = !in_probe_up && was_in_probe_up_before;
    bool need_cleanup = !in_probe_up && (up_phase_start_time_ != QuicTime::Zero());

    if (leaving_probe_up || need_cleanup) {
      // Check if UP phase exited early due to loss/ECN
      // Early exit is detected if we were in PROBE_UP before parent update and had loss/ECN
      if (in_probe_up_before && has_loss_or_ecn && !in_probe_up) {
        up_phase_exited_early_ = true;
        QUIC_DVLOG(2) << "FreqCCv3: UP phase exited early due to loss/ECN @ " << event_time
                      << ", bandwidth_before_up=" << bandwidth_before_up_;
      }

      // Calculate UP phase duration and adapt frequency for next UP phase
      if (up_phase_start_time_ != QuicTime::Zero()) {
        TimeDelta up_duration = event_time - up_phase_start_time_;
        last_up_duration_sec_ = static_cast<double>(up_duration.ToMicroseconds()) / 1000000.0;

        // STFT: Mark end of UP phase
        last_up_phase_end_time_ = event_time;

        // Calculate how many cycles occurred in this UP phase
        double cycles_in_up = last_up_duration_sec_ * current_oscillation_freq_hz_;  // Use the frequency that WAS used in this UP phase

        // STFT: Check Validity
        if (cycles_in_up >= kMinCyclesPerUp) {
            last_up_phase_valid_ = true;

            // ===== SENDER RATE STFT ANALYSIS =====
            // Calculate sender_max_peak_freq_hz_ using sender rate signal
            // Filter sender rate samples in [last_up_phase_start_time_, last_up_phase_end_time_]
            std::vector<FreqSignalSample> sender_samples;
            for (const auto& s : sender_rate_history_) {
                if (s.time >= last_up_phase_start_time_ && s.time <= last_up_phase_end_time_) {
                    sender_samples.push_back(s);
                }
            }

            // std::cerr << "FreqCCv3: ===== SENDER RATE STFT DIAGNOSTICS =====" << std::endl;
            // std::cerr << "FreqCCv3: Sender rate samples in UP phase: " << sender_samples.size()
            //               << ", total history: " << sender_rate_history_.size()
            //               << ", UP duration: " << (last_up_phase_end_time_ - last_up_phase_start_time_).ToMilliseconds() << "ms" << std::endl;

            if (!sender_samples.empty()) {
                // Print sample range for verification
                // double first_rate = sender_samples.front().rate.ToKBitsPerSecond();
                // double last_rate = sender_samples.back().rate.ToKBitsPerSecond();
                // std::cerr << "FreqCCv3: Sender rate range: [" << first_rate << ", " << last_rate << "] kbps" << std::endl;
            }

            if (!sender_samples.empty()) {
                // Calculate STFT window size based on the oscillation frequency used in this UP phase
                double win_sec = CalculateSTFTWindowSize(current_oscillation_freq_hz_);

                TimeDelta sender_window_size = TimeDelta::FromMicroseconds(static_cast<int64_t>(win_sec * 1000000.0));
                TimeDelta sender_duration = last_up_phase_end_time_ - last_up_phase_start_time_;
                if (sender_window_size > sender_duration) sender_window_size = sender_duration;

                // Sliding window analysis with 90% overlap (like Python)
                double overlap = 0.90;
                TimeDelta step_size = TimeDelta::FromMicroseconds(static_cast<int64_t>(sender_window_size.ToMicroseconds() * (1.0 - overlap)));
                if (step_size < TimeDelta::FromMilliseconds(1)) step_size = TimeDelta::FromMilliseconds(1);

                std::vector<double> sender_peak_freqs;
                QuicTime win_start = last_up_phase_start_time_;
                int window_count = 0;

                while (win_start + sender_window_size <= last_up_phase_end_time_) {
                    QuicTime win_end = win_start + sender_window_size;

                    // Extract window samples
                    std::vector<FreqSignalSample> win_samples;
                    for (const auto& s : sender_samples) {
                        if (s.time >= win_start && s.time <= win_end) {
                            win_samples.push_back(s);
                        }
                    }

                    window_count++;

                    if (!win_samples.empty()) {
                        // ===== APPLY CONSTRAINTS for sender rate STFT =====
                        AnalysisResult result = AnalyzeWindow(win_samples, sender_window_size, current_oscillation_freq_hz_);

                        // Only accept VALID results that pass physical constraints
                        if (result.valid && result.peak_freq_hz > 0.0) {
                            sender_peak_freqs.push_back(result.peak_freq_hz);
                        }
                    }

                    // Advance window
                    win_start = win_start + step_size;
                }

                // Calculate max peak freq from sender rate
                if (!sender_peak_freqs.empty()) {
                    sender_max_peak_freq_hz_ = *std::max_element(sender_peak_freqs.begin(), sender_peak_freqs.end());
                } else {
                    sender_max_peak_freq_hz_ = 0.0;
                }

                double sender_ref_freq_hz =
                    (sender_max_peak_freq_hz_ > 0.0) ? sender_max_peak_freq_hz_
                                                     : current_oscillation_freq_hz_;
                CaptureSenderSpectrumTemplate(sender_samples, sender_duration,
                                              sender_ref_freq_hz);
                if (last_up_sender_template_valid_ &&
                    last_up_sender_template_freq_hz_ > 0.0) {
                    sender_ref_freq_hz = last_up_sender_template_freq_hz_;
                    if (sender_max_peak_freq_hz_ <= 0.0) {
                        sender_max_peak_freq_hz_ = sender_ref_freq_hz;
                    }
                }

                // During the current UP itself, keep the detector band-gated but do not
                // require ICC-style template similarity yet. The stored sender template is
                // intended for the following INT = CRUISE + REFILL interval.
                bool saved_template_valid = last_up_sender_template_valid_;
                last_up_sender_template_valid_ = false;
                PerformFreqAnalysis(last_up_phase_start_time_, last_up_phase_end_time_,
                                    0.0, sender_ref_freq_hz);
                PerformRttFreqAnalysis(last_up_phase_start_time_, last_up_phase_end_time_,
                                       0.0, sender_ref_freq_hz);
                last_up_sender_template_valid_ = saved_template_valid;
            } else {
                sender_max_peak_freq_hz_ = 0.0;
                PerformFreqAnalysis(last_up_phase_start_time_, last_up_phase_end_time_,
                                    0.0, current_oscillation_freq_hz_);
                PerformRttFreqAnalysis(last_up_phase_start_time_, last_up_phase_end_time_,
                                       0.0, current_oscillation_freq_hz_);
            }

            // Clear sender rate history after STFT analysis
            while (!sender_rate_history_.empty() && sender_rate_history_.front().time < event_time) {
                sender_rate_history_.pop_front();
            }
            while (!rtt_signal_history_.empty() && rtt_signal_history_.front().time < event_time) {
                rtt_signal_history_.pop_front();
            }
        } else {
            last_up_phase_valid_ = false;
            sender_max_peak_freq_hz_ = 0.0;
            // Abandon cache for this UP phase
            // We clear history up to now to ensure we don't use it
             while (!signal_history_.empty() && signal_history_.front().time < event_time) {
                signal_history_.pop_front();
            }
            while (!rtt_signal_history_.empty() && rtt_signal_history_.front().time < event_time) {
                rtt_signal_history_.pop_front();
            }
            // Clear sender rate history as well since this UP phase is invalid
            while (!sender_rate_history_.empty() && sender_rate_history_.front().time < event_time) {
                sender_rate_history_.pop_front();
            }
        }

        QUIC_DVLOG(2) << "FreqCCv3: Leaving PROBE_UP, duration=" << last_up_duration_sec_
                      << "s, cycles=" << cycles_in_up << ", freq=" << current_oscillation_freq_hz_ << "Hz"
                      << ", current_up_pacing_gain=" << current_up_pacing_gain_;

        // Adapt frequency if cycles are outside [kMinCyclesPerUp, kMaxCyclesPerUp]
        if (last_up_duration_sec_ > 0.001) {  // Avoid division by zero, minimum 1ms
          // Get min_rtt in seconds for calculating dynamic frequency limits
          double min_rtt_sec = static_cast<double>(model_.MinRtt().ToMicroseconds()) / 1000000.0;

          // Calculate dynamic minimum frequency: 2 cycles in 0.5*RTT_min
          // min_freq = 2 / (0.5 * RTT_min) = 4 / RTT_min
          double dynamic_min_freq_hz = kMinFreqHz;  // Default to static minimum
          if (min_rtt_sec > 0.0) {
            dynamic_min_freq_hz = 4.0 / min_rtt_sec;  // 2 cycles in 0.5*RTT_min
            // Ensure it's at least the static minimum
            if (dynamic_min_freq_hz < kMinFreqHz) {
              dynamic_min_freq_hz = kMinFreqHz;
            }
          }

          // Adapt frequency only if cycles are too high (cycles > kMaxCyclesPerUp)
          // We ignore the case where cycles < kMinCyclesPerUp as per request
          if (cycles_in_up > kMaxCyclesPerUp) {
            // Recalculate frequency to achieve kTargetCyclesPerUp cycles in the recorded duration
            // kTargetCyclesPerUp is (kMinCyclesPerUp + kMaxCyclesPerUp) / 2 = (2 + 5) / 2 = 3
            double new_freq = static_cast<double>(kTargetCyclesPerUp) / last_up_duration_sec_;

            // Clamp frequency to [dynamic_min_freq_hz, kMaxFreqHz]
            if (new_freq < dynamic_min_freq_hz) {
              new_freq = dynamic_min_freq_hz;
            } else if (new_freq > kMaxFreqHz) {
              new_freq = kMaxFreqHz;
            }

            QUIC_DVLOG(2) << "FreqCCv3: Adapting frequency from " << oscillation_freq_hz_
                          << "Hz to " << new_freq << "Hz (target " << kTargetCyclesPerUp
                          << " cycles in " << last_up_duration_sec_ << "s, min_freq="
                          << dynamic_min_freq_hz << "Hz based on RTT=" << min_rtt_sec << "s)";
            oscillation_freq_hz_ = new_freq;
          }

          // Adapt pacing gain based on UP phase duration in RTT multiples
          // Calculate UP phase duration in RTT multiples
          double up_duration_rtt_multiple = 0.0;
          if (min_rtt_sec > 0.0) {
            up_duration_rtt_multiple = last_up_duration_sec_ / min_rtt_sec;
          }

          float new_pacing_gain = up_pacing_gain_;

          // Apply the new pacing gain adjustment logic based on RTT multiples
          if (up_duration_rtt_multiple > kUpDurationVeryLongRttMultiple) {
            // Case 1: Up_time > 2.5*RTT_min, pacing gain = 1.25
            new_pacing_gain = kMaxUpPacingGain;
            QUIC_DVLOG(2) << "FreqCCv3: UP phase very long (" << up_duration_rtt_multiple
                          << " RTTs > " << kUpDurationVeryLongRttMultiple
                          << " RTTs), setting pacing gain to " << new_pacing_gain;
          }
          else if (up_duration_rtt_multiple >= kUpDurationLongRttMultiple &&
                   up_duration_rtt_multiple <= kUpDurationVeryLongRttMultiple) {
            // Case 2: 2.0*RTT_min <= Up_time <= 2.5*RTT_min
            // pacing gain = min(1.25, max(current_pacing_gain + 0.01, Up_time * 1.25 / 2.5))
            float calculated_gain = static_cast<float>(up_duration_rtt_multiple * kMaxUpPacingGain / kUpDurationVeryLongRttMultiple);
            float incremented_gain = up_pacing_gain_ + kPacingGainAdjustStep;
            new_pacing_gain = std::min(kMaxUpPacingGain, std::max(incremented_gain, calculated_gain));
            QUIC_DVLOG(2) << "FreqCCv3: UP phase slightly long (" << up_duration_rtt_multiple
                          << " RTTs in [" << kUpDurationLongRttMultiple << ", "
                          << kUpDurationVeryLongRttMultiple << "]), adjusting pacing gain from "
                          << up_pacing_gain_ << " to " << new_pacing_gain
                          << " (calculated=" << calculated_gain << ", incremented=" << incremented_gain << ")";
          }
          else if (up_duration_rtt_multiple > kUpDurationShortRttMultiple &&
                   up_duration_rtt_multiple < kUpDurationLongRttMultiple) {
            // Case 3: 1.5*RTT_min < Up_time < 2.0*RTT_min
            // pacing gain = max(1.00, min(current_pacing_gain - 0.01, Up_time * 1.00 / 1.5))
            float calculated_gain = static_cast<float>(up_duration_rtt_multiple * kMinUpPacingGain / kUpDurationShortRttMultiple);
            float decremented_gain = up_pacing_gain_ - kPacingGainAdjustStep;
            new_pacing_gain = std::max(kMinUpPacingGain, std::min(decremented_gain, calculated_gain));
            QUIC_DVLOG(2) << "FreqCCv3: UP phase slightly short (" << up_duration_rtt_multiple
                          << " RTTs in (" << kUpDurationShortRttMultiple << ", "
                          << kUpDurationLongRttMultiple << ")), adjusting pacing gain from "
                          << up_pacing_gain_ << " to " << new_pacing_gain
                          << " (calculated=" << calculated_gain << ", decremented=" << decremented_gain << ")";
          }
          else {
            // Case 4: Up_time <= 1.5*RTT_min, pacing gain = 1.00
            new_pacing_gain = kMinUpPacingGain;
            QUIC_DVLOG(2) << "FreqCCv3: UP phase too short (" << up_duration_rtt_multiple
                          << " RTTs <= " << kUpDurationShortRttMultiple
                          << " RTTs), setting pacing gain to " << new_pacing_gain;
          }

          // Final clamp to ensure bounds (should already be satisfied by logic above)
          new_pacing_gain = std::max(kMinUpPacingGain, std::min(kMaxUpPacingGain, new_pacing_gain));

          if (new_pacing_gain != up_pacing_gain_) {
            QUIC_DVLOG(2) << "FreqCCv3: Final pacing gain adjustment: " << up_pacing_gain_
                          << " -> " << new_pacing_gain
                          << " (UP duration: " << last_up_duration_sec_ << "s = "
                          << up_duration_rtt_multiple << " RTTs, min_rtt=" << min_rtt_sec << "s)";
            up_pacing_gain_ = new_pacing_gain;
          }

          // Call UP phase trace callback if set
          if (up_phase_trace_cb_) {
            double start_time_sec = static_cast<double>(up_phase_start_time_.ToDebuggingValue()) / 1000000.0;
            double duration_ms = last_up_duration_sec_ * 1000.0;
            int actual_cycles = static_cast<int>(cycles_in_up + 0.5);  // Round to nearest integer
            float freq_hz_used = static_cast<float>(current_oscillation_freq_hz_);  // The frequency that WAS used in this UP phase
            float pacing_gain_used = current_up_pacing_gain_;  // The pacing gain that WAS used in this UP phase
            int32_t bw_estimate_kbps = static_cast<int32_t>(model_.MaxBandwidth().ToKBitsPerSecond());
            bool exit_due_to_queueing = !up_phase_exited_early_;
            up_phase_trace_cb_(start_time_sec, duration_ms, freq_hz_used, exit_due_to_queueing, actual_cycles, pacing_gain_used, bw_estimate_kbps);
          }
        }

        up_phase_count_++;
      }
      // Reset oscillation state so next UP phase will be detected as new
      oscillation_start_time_ = QuicTime::Zero();
      up_phase_start_time_ = QuicTime::Zero();
      QUIC_DVLOG(2) << "FreqCCv3: Leaving PROBE_UP, stopping oscillation";
    }
  }

  // Update tracking variables
  last_mode_ = current_mode;
  last_probe_bw_phase_ = current_phase;
}

QuicBandwidth FreqCCv3Sender::PacingRate(QuicByteCount bytes_in_flight) const {
  // Get base pacing rate from BBRv2
  QuicBandwidth base_rate = Bbr2Sender::PacingRate(bytes_in_flight);

  // If in NEW_REFILL, apply the modified pacing gain
  if (in_new_refill_ && mode_ == Bbr2Mode::PROBE_BW) {
    Bbr2ProbeBwMode::CyclePhase current_phase = GetCurrentProbeBwPhase();
    if (current_phase == Bbr2ProbeBwMode::CyclePhase::PROBE_REFILL) {
      // The pacing gain is already set in OnCongestionEvent via model_.set_pacing_gain()
      // So base_rate should already reflect the NEW_REFILL pacing gain
    }
  }

  // Check if we should oscillate
  bool should_osc = ShouldOscillate();

  // If not oscillating (not in PROBE_UP), return base rate
  if (!should_osc) {
    return base_rate;
  }

  // Check oscillation_start_time_
  if (oscillation_start_time_ == QuicTime::Zero()) {
    // oscillation_start_time_ should have been set in OnCongestionEvent
    // If it's still Zero, we can't oscillate properly
    return base_rate;
  }

  // Use current_time_ which is updated on each packet sent and ACK received
  if (current_time_ == QuicTime::Zero()) {
    QUIC_DVLOG(2) << "FreqCCv3: PacingRate - current_time_ is Zero, returning base_rate";
    return base_rate;
  }

  // Calculate oscillation offset based on current time
  int64_t offset_bps = CalculateOscillationOffset(current_time_);

  // Apply offset to base rate
  int64_t base_bps = static_cast<int64_t>(base_rate.ToBitsPerSecond());
  // int64_t offset_bps = static_cast<int64_t>(offset.ToBitsPerSecond()); // No longer needed
  int64_t final_bps = base_bps + offset_bps;

  // Ensure pacing rate doesn't go negative or too low
  const int64_t min_rate_bps = 1000;  // 1 kbps minimum
  if (final_bps < min_rate_bps) {
    final_bps = min_rate_bps;
  }

  return QuicBandwidth::FromBitsPerSecond(static_cast<uint64_t>(final_bps));
}

QuicByteCount FreqCCv3Sender::GetCongestionWindow() const {
  QuicByteCount cwnd = cwnd_;

  // During NEW_REFILL, cap CWND to the high threshold (0.75 BDP)
  // This ensures we drain if inflight is too high, or stop filling when we reach the target.
  if (in_new_refill_) {
    QuicByteCount cap = CalculateInflightThreshold(kNewRefillHighThreshold);
    if (cwnd > cap) {
      cwnd = cap;
    }
  }

  return cwnd;
}

int32_t FreqCCv3Sender::GetCurrentBbrModeIndex() const {
  // Get debug state from parent
  DebugState state = ExportDebugState();

  // Convert mode and probe_bw phase to single index
  // 0: STARTUP
  // 1: DRAIN
  // 2: PROBE_BW_DOWN
  // 3: PROBE_BW_CRUISE
  // 4: PROBE_BW_REFILL (NEW_REFILL)
  // 5: PROBE_BW_UP
  // 6: PROBE_RTT
  switch (state.mode) {
    case Bbr2Mode::STARTUP:
      return 0;
    case Bbr2Mode::DRAIN:
      return 1;
    case Bbr2Mode::PROBE_BW:
      // Map probe_bw phase to index
      switch (state.probe_bw.phase) {
        case Bbr2ProbeBwMode::CyclePhase::PROBE_DOWN:
          return 2;
        case Bbr2ProbeBwMode::CyclePhase::PROBE_CRUISE:
          return 3;
        case Bbr2ProbeBwMode::CyclePhase::PROBE_REFILL:
          return 4;
        case Bbr2ProbeBwMode::CyclePhase::PROBE_UP:
          return 5;
        case Bbr2ProbeBwMode::CyclePhase::PROBE_NOT_STARTED:
        default:
          return 2;  // Default to PROBE_DOWN
      }
    case Bbr2Mode::PROBE_RTT:
      return 6;
    default:
      return 0;
  }
}

// ===== CONSTRAINT 3: ACK Signal Preprocessing =====
// Smooth the receiver rate signal to remove ACK-driven impulse noise.
// The smoothing window is chosen by the caller as a fraction of the STFT window.
std::vector<FreqSignalSample> FreqCCv3Sender::SmoothSignal(
    const std::vector<FreqSignalSample>& raw_samples,
    double smoothing_window_ms) const {

    if (raw_samples.empty()) return {};

    std::vector<FreqSignalSample> smoothed;
    smoothed.reserve(raw_samples.size());

    TimeDelta window = TimeDelta::FromMilliseconds(static_cast<int64_t>(smoothing_window_ms));

    for (size_t i = 0; i < raw_samples.size(); ++i) {
        QuicTime center_time = raw_samples[i].time;
        TimeDelta half_window = TimeDelta::FromMicroseconds(window.ToMicroseconds() / 2);
        QuicTime window_start = center_time - half_window;
        QuicTime window_end = center_time + half_window;

        // Collect samples within smoothing window
        double sum_rate = 0.0;
        int count = 0;

        for (size_t j = 0; j < raw_samples.size(); ++j) {
            if (raw_samples[j].time >= window_start && raw_samples[j].time <= window_end) {
                sum_rate += raw_samples[j].rate.ToKBitsPerSecond();
                count++;
            }
        }

        if (count > 0) {
            double avg_rate_kbps = sum_rate / count;
            QuicBandwidth smoothed_rate = QuicBandwidth::FromKBitsPerSecond(
                static_cast<int64_t>(avg_rate_kbps));
            smoothed.push_back({center_time, smoothed_rate});
        } else {
            // Fallback: use original sample if no neighbors found
            smoothed.push_back(raw_samples[i]);
        }
    }

    return smoothed;
}

// Simple window calculation for STFT based on ±5% frequency tolerance
// Window size should contain enough cycles to resolve the target frequency within ±5%
// For FFT: frequency resolution Δf = 1/T_window
// To detect f_target within ±5%, we need at least 2-3 cycles in the window
// Similar to Python code: window ≈ rtt_min (e.g., 80ms for 60Hz gives ~4.8 cycles)
double FreqCCv3Sender::CalculateSTFTWindowSize(double target_freq_hz) const {
    if (target_freq_hz <= 0.0) {
        return 0.08;  // Default 80ms like Python
    }

    // Strategy: Window should contain ~3-5 cycles of the target frequency
    // This gives reasonable frequency resolution while not being too long
    // cycles_in_window = window_sec * target_freq_hz
    // For ±5% tolerance, we want frequency resolution Δf ≤ 0.1 * target_freq
    // Δf = 1/T_window, so T_window ≥ 10/target_freq (for 10% resolution)
    // But we also want to match Python's behavior of using rtt_min (~80ms)

    // Use minimum of:
    // 1. 3 cycles at target frequency: 3 / target_freq_hz
    // 2. RTT_min (like Python does)
    double cycles_based_window = 3.0 / target_freq_hz;

    TimeDelta min_rtt = model_.MinRtt();
    double rtt_based_window = 0.08;  // Default 80ms like Python
    if (!min_rtt.IsZero()) {
        rtt_based_window = static_cast<double>(min_rtt.ToMicroseconds()) / 1000000.0;
    }

    // Take the maximum of cycles-based and RTT-based to ensure enough samples
    // But cap at a reasonable maximum (e.g., 200ms) to avoid too long windows
    double win_sec = std::max(cycles_based_window, rtt_based_window);

    // Minimum window: 50ms (to have enough samples for meaningful FFT)
    // Maximum window: 200ms (to avoid smoothing out frequency variations)
    const double kMinWindowSec = 0.05;
    const double kMaxWindowSec = 0.20;

    win_sec = std::max(kMinWindowSec, std::min(kMaxWindowSec, win_sec));

    return win_sec;
}

double FreqCCv3Sender::CalculateRttSTFTWindowSize(double target_freq_hz) const {
    double cycles_based_window = 0.0;
    if (target_freq_hz > 0.0) {
        cycles_based_window = 8.0 / target_freq_hz;
    }

    double rtt_based_window = 0.0;
    TimeDelta min_rtt = model_.MinRtt();
    if (min_rtt.IsZero() && rtt_stats_ != nullptr) {
        min_rtt = rtt_stats_->MinOrInitialRtt();
    }
    if (!min_rtt.IsZero()) {
        rtt_based_window =
            3.0 * static_cast<double>(min_rtt.ToMicroseconds()) / 1000000.0;
    }

    double win_sec = std::max(cycles_based_window, rtt_based_window);
    if (win_sec <= 0.0) {
        return 0.0;
    }

    const double kMaxWindowSec = 0.60;
    if (win_sec > kMaxWindowSec) {
        win_sec = kMaxWindowSec;
    }
    return win_sec;
}

FreqCCv3Sender::SpectrumProfile FreqCCv3Sender::BuildSpectrumProfile(
    const std::vector<double>& values,
    double sample_step_s,
    double ref_freq_hz) const {
    SpectrumProfile profile{0.0, 0.0, 0.0, {}, false};
    if (values.size() < 8 || sample_step_s <= 0.0 || ref_freq_hz <= 0.0) {
        return profile;
    }

    const int signal_len = static_cast<int>(values.size());
    const int nfft = std::max(signal_len, signal_len * kFftZeroPadMultiplier);
    double* in = (double*)fftw_malloc(sizeof(double) * nfft);
    fftw_complex* out =
        (fftw_complex*)fftw_malloc(sizeof(fftw_complex) * (nfft / 2 + 1));

    double mean = 0.0;
    for (double value : values) {
        mean += value;
    }
    mean /= signal_len;

    for (int i = 0; i < signal_len; ++i) {
        double hann = 0.5 * (1.0 - std::cos(2.0 * M_PI * i / (signal_len - 1)));
        in[i] = (values[i] - mean) * hann;
    }
    for (int i = signal_len; i < nfft; ++i) {
        in[i] = 0.0;
    }

    fftw_plan plan = fftw_plan_dft_r2c_1d(nfft, in, out, FFTW_ESTIMATE);
    fftw_execute(plan);

    const double fs = 1.0 / sample_step_s;
    const double freq_step = fs / nfft;
    const int k_min = 1;
    const int k_max = nfft / 2;

    std::vector<double> magnitudes(k_max + 1, 0.0);
    double total_energy = 0.0;
    for (int k = k_min; k <= k_max; ++k) {
        double mag = std::sqrt(out[k][0] * out[k][0] + out[k][1] * out[k][1]);
        magnitudes[k] = mag;
        total_energy += mag;
    }

    const double band_low_hz = kBandLowRatio * ref_freq_hz;
    const double band_high_hz = kBandHighRatio * ref_freq_hz;
    int band_k_low = std::max(k_min, static_cast<int>(std::ceil(band_low_hz / freq_step)));
    int band_k_high = std::min(k_max, static_cast<int>(std::floor(band_high_hz / freq_step)));

    if (band_k_low > band_k_high || total_energy <= 0.0) {
        fftw_destroy_plan(plan);
        fftw_free(in);
        fftw_free(out);
        return profile;
    }

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

    profile.peak_freq_hz = peak_k * freq_step;
    if (peak_k > k_min && peak_k < k_max) {
        double left_mag = magnitudes[peak_k - 1];
        double center_mag = magnitudes[peak_k];
        double right_mag = magnitudes[peak_k + 1];
        double denom = left_mag - 2.0 * center_mag + right_mag;
        if (std::abs(denom) > 1e-12) {
            double offset = 0.5 * (left_mag - right_mag) / denom;
            if (offset > 1.0) offset = 1.0;
            if (offset < -1.0) offset = -1.0;
            profile.peak_freq_hz = (peak_k + offset) * freq_step;
        }
    }

    profile.band_energy_ratio = band_energy / total_energy;
    profile.band_peak_rel = max_mag / total_energy;

    profile.band_shape.assign(kBandShapeBins, 0.0);
    if (kBandShapeBins == 1) {
        profile.band_shape[0] = max_mag;
    } else {
        for (int i = 0; i < kBandShapeBins; ++i) {
            double target_freq =
                band_low_hz +
                (band_high_hz - band_low_hz) * static_cast<double>(i) /
                    static_cast<double>(kBandShapeBins - 1);
            double raw_index = target_freq / freq_step;
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
    }

    double shape_sum = std::accumulate(profile.band_shape.begin(),
                                       profile.band_shape.end(), 0.0);
    if (shape_sum > 0.0) {
        for (double& value : profile.band_shape) {
            value /= shape_sum;
        }
    } else {
        profile.band_shape.clear();
    }

    profile.valid = profile.band_peak_rel >= kEnergyThresholdRatio &&
                    !profile.band_shape.empty();

    fftw_destroy_plan(plan);
    fftw_free(in);
    fftw_free(out);
    return profile;
}

double FreqCCv3Sender::ComputeSpectrumShapeDistance(
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

bool FreqCCv3Sender::CaptureSenderSpectrumTemplate(
    const std::vector<FreqSignalSample>& samples,
    TimeDelta window_duration,
    double ref_freq_hz) {
    last_up_sender_template_valid_ = false;
    last_up_sender_template_freq_hz_ = 0.0;
    last_up_sender_band_template_.clear();

    if (samples.empty() || ref_freq_hz <= 0.0) {
        return false;
    }

    double duration_s =
        static_cast<double>(window_duration.ToMicroseconds()) / 1000000.0;
    int sample_count = static_cast<int>(duration_s / 0.001);
    if (sample_count < 8) {
        return false;
    }

    std::vector<double> uniform_values(sample_count, 0.0);
    double t_start =
        (samples.front().time - QuicTime::Zero()).ToMicroseconds() / 1000000.0;
    size_t sample_idx = 0;
    for (int i = 0; i < sample_count; ++i) {
        double t_target = t_start + i * 0.001;
        while (sample_idx < samples.size() - 1) {
            double t_next =
                (samples[sample_idx + 1].time - QuicTime::Zero()).ToMicroseconds() /
                1000000.0;
            if (t_next >= t_target) break;
            sample_idx++;
        }

        double value;
        if (sample_idx >= samples.size() - 1) {
            value = static_cast<double>(samples.back().rate.ToKBitsPerSecond());
        } else {
            double t1 =
                (samples[sample_idx].time - QuicTime::Zero()).ToMicroseconds() /
                1000000.0;
            double t2 =
                (samples[sample_idx + 1].time - QuicTime::Zero()).ToMicroseconds() /
                1000000.0;
            double v1 = static_cast<double>(samples[sample_idx].rate.ToKBitsPerSecond());
            double v2 =
                static_cast<double>(samples[sample_idx + 1].rate.ToKBitsPerSecond());
            if (std::abs(t2 - t1) < 1e-6) {
                value = v1;
            } else {
                double fraction = (t_target - t1) / (t2 - t1);
                value = v1 + fraction * (v2 - v1);
            }
        }
        uniform_values[i] = value;
    }

    SpectrumProfile profile = BuildSpectrumProfile(uniform_values, 0.001, ref_freq_hz);
    if (!profile.valid) {
        return false;
    }

    last_up_sender_template_freq_hz_ =
        (profile.peak_freq_hz > 0.0) ? profile.peak_freq_hz : ref_freq_hz;
    last_up_sender_band_template_ = profile.band_shape;
    last_up_sender_template_valid_ = !last_up_sender_band_template_.empty();
    return last_up_sender_template_valid_;
}

// Helper to perform DFT with PHYSICAL CONSTRAINTS
// Returns (Peak Frequency Hz, Avg Rate kbps, Validity Flag)
FreqCCv3Sender::AnalysisResult FreqCCv3Sender::AnalyzeWindow(
    const std::vector<FreqSignalSample>& samples,
    TimeDelta window_duration,
    double expected_freq_hz) const {
    if (samples.empty()) return {0.0, 0, std::numeric_limits<double>::infinity(), false};

    double sum_rate = 0.0;
    for (const auto& s : samples) {
        sum_rate += s.rate.ToKBitsPerSecond();
    }
    int32_t avg_rate = static_cast<int32_t>(sum_rate / samples.size());

    double ref_freq_hz = expected_freq_hz;
    if (ref_freq_hz <= 0.0 && last_up_sender_template_valid_) {
        ref_freq_hz = last_up_sender_template_freq_hz_;
    }
    if (ref_freq_hz <= 0.0) {
        return {0.0, avg_rate, std::numeric_limits<double>::infinity(), false};
    }

    double duration_s = static_cast<double>(window_duration.ToMicroseconds()) / 1000000.0;
    int sample_count = static_cast<int>(duration_s / 0.001);
    if (sample_count < 8) {
        return {0.0, avg_rate, std::numeric_limits<double>::infinity(), false};
    }

    std::vector<double> uniform_values(sample_count, 0.0);
    double t_start =
        (samples.front().time - QuicTime::Zero()).ToMicroseconds() / 1000000.0;
    size_t sample_idx = 0;
    for (int i = 0; i < sample_count; ++i) {
        double t_target = t_start + i * 0.001;
        while (sample_idx < samples.size() - 1) {
            double t_next =
                (samples[sample_idx + 1].time - QuicTime::Zero()).ToMicroseconds() /
                1000000.0;
            if (t_next >= t_target) break;
            sample_idx++;
        }

        double value;
        if (sample_idx >= samples.size() - 1) {
            value = static_cast<double>(samples.back().rate.ToKBitsPerSecond());
        } else {
            double t1 =
                (samples[sample_idx].time - QuicTime::Zero()).ToMicroseconds() /
                1000000.0;
            double t2 =
                (samples[sample_idx + 1].time - QuicTime::Zero()).ToMicroseconds() /
                1000000.0;
            double v1 = static_cast<double>(samples[sample_idx].rate.ToKBitsPerSecond());
            double v2 =
                static_cast<double>(samples[sample_idx + 1].rate.ToKBitsPerSecond());
            if (std::abs(t2 - t1) < 1e-6) {
                value = v1;
            } else {
                double fraction = (t_target - t1) / (t2 - t1);
                value = v1 + fraction * (v2 - v1);
            }
        }
        uniform_values[i] = value;
    }

    SpectrumProfile profile = BuildSpectrumProfile(uniform_values, 0.001, ref_freq_hz);
    if (!profile.valid) {
        return {0.0, avg_rate, std::numeric_limits<double>::infinity(), false};
    }

    double shape_distance = 0.0;
    bool valid = true;
    if (last_up_sender_template_valid_ && !last_up_sender_band_template_.empty()) {
        shape_distance =
            ComputeSpectrumShapeDistance(profile.band_shape, last_up_sender_band_template_);
        if (!std::isfinite(shape_distance) ||
            shape_distance > kSpectrumShapeTolerance) {
            valid = false;
        }
    }

    return {profile.peak_freq_hz, avg_rate, shape_distance, valid};
}

FreqCCv3Sender::RttAnalysisResult FreqCCv3Sender::AnalyzeRttWindow(
    const std::vector<RttSignalSample>& samples,
    TimeDelta window_duration,
    double expected_freq_hz) const {
    if (samples.empty()) {
        return {0.0, 0.0, std::numeric_limits<double>::infinity(), false};
    }

    double sum_rtt = 0.0;
    for (const auto& s : samples) {
        sum_rtt += s.rtt_ms;
    }
    double avg_rtt_ms = sum_rtt / samples.size();

    double ref_freq_hz = expected_freq_hz;
    if (ref_freq_hz <= 0.0 && last_up_sender_template_valid_) {
        ref_freq_hz = last_up_sender_template_freq_hz_;
    }
    if (ref_freq_hz <= 0.0) {
        return {0.0, avg_rtt_ms, std::numeric_limits<double>::infinity(), false};
    }

    double duration_s = static_cast<double>(window_duration.ToMicroseconds()) / 1000000.0;
    int sample_count = static_cast<int>(duration_s / 0.001);
    if (sample_count < 8) {
        return {0.0, avg_rtt_ms, std::numeric_limits<double>::infinity(), false};
    }

    std::vector<double> uniform_values(sample_count, 0.0);
    double t_start =
        (samples.front().time - QuicTime::Zero()).ToMicroseconds() / 1000000.0;
    size_t sample_idx = 0;
    for (int i = 0; i < sample_count; ++i) {
        double t_target = t_start + i * 0.001;
        while (sample_idx < samples.size() - 1) {
            double t_next =
                (samples[sample_idx + 1].time - QuicTime::Zero()).ToMicroseconds() /
                1000000.0;
            if (t_next >= t_target) break;
            sample_idx++;
        }

        double value;
        if (sample_idx >= samples.size() - 1) {
            value = samples.back().rtt_ms;
        } else {
            double t1 =
                (samples[sample_idx].time - QuicTime::Zero()).ToMicroseconds() /
                1000000.0;
            double t2 =
                (samples[sample_idx + 1].time - QuicTime::Zero()).ToMicroseconds() /
                1000000.0;
            double v1 = samples[sample_idx].rtt_ms;
            double v2 = samples[sample_idx + 1].rtt_ms;
            if (std::abs(t2 - t1) < 1e-6) {
                value = v1;
            } else {
                double fraction = (t_target - t1) / (t2 - t1);
                value = v1 + fraction * (v2 - v1);
            }
        }
        uniform_values[i] = value;
    }

    SpectrumProfile profile = BuildSpectrumProfile(uniform_values, 0.001, ref_freq_hz);
    if (!profile.valid) {
        return {0.0, avg_rtt_ms, std::numeric_limits<double>::infinity(), false};
    }

    double shape_distance = 0.0;
    bool valid = true;
    if (last_up_sender_template_valid_ && !last_up_sender_band_template_.empty()) {
        shape_distance =
            ComputeSpectrumShapeDistance(profile.band_shape, last_up_sender_band_template_);
        if (!std::isfinite(shape_distance) ||
            shape_distance > kSpectrumShapeTolerance) {
            valid = false;
        }
    }

    return {profile.peak_freq_hz, avg_rtt_ms, shape_distance, valid};
}

void FreqCCv3Sender::PerformFreqAnalysis(QuicTime start_time, QuicTime end_time, double threshold_freq_hz, double expected_freq_hz) {
    if (start_time >= end_time || signal_history_.empty()) return;

    // Filter samples in range
    std::vector<FreqSignalSample> range_samples;
    for (const auto& s : signal_history_) {
        if (s.time >= start_time && s.time <= end_time) {
            range_samples.push_back(s);
        }
    }

    if (range_samples.empty()) return;

    TimeDelta duration = end_time - start_time;
    TimeDelta window_size = TimeDelta::Zero();

    // Calculate window size using the simple method
    if (expected_freq_hz > 0.0) {
        // Use expected frequency (for UP phase receiver rate analysis)
        double win_sec = CalculateSTFTWindowSize(expected_freq_hz);
        window_size = TimeDelta::FromMicroseconds(static_cast<int64_t>(win_sec * 1000000.0));
    } else {
        // For interval phase (CRUISE + REFILL), scan with multiplier * min_rtt.
        TimeDelta min_rtt = model_.MinRtt();
        if (!min_rtt.IsZero()) {
            window_size = TimeDelta::FromMicroseconds(
                static_cast<int64_t>(interval_window_multiplier_ * min_rtt.ToMicroseconds()));
        } else if (last_up_window_size_ > TimeDelta::Zero()) {
            window_size = last_up_window_size_;
        } else {
            window_size = duration;
        }
    }

    // Constraint: Cannot exceed available data
    if (window_size > duration) window_size = duration;

    // ===== CONSTRAINT 3: Apply Signal Smoothing (ACK preprocessing) =====
    double smoothing_window_ms =
        static_cast<double>(window_size.ToMicroseconds()) / 1000.0 * kSmoothingWindowFraction;
    std::vector<FreqSignalSample> smoothed_samples =
        SmoothSignal(range_samples, smoothing_window_ms);
    if (smoothed_samples.empty()) {
        smoothed_samples = range_samples;  // Fallback to raw if smoothing fails
    }

    // Store for future use
    last_up_window_size_ = window_size;

    // Sliding Window Analysis with 90% overlap
    double overlap = 0.90;
    TimeDelta step_size = TimeDelta::FromMicroseconds(static_cast<int64_t>(window_size.ToMicroseconds() * (1.0 - overlap)));
    if (step_size < TimeDelta::FromMilliseconds(1)) step_size = TimeDelta::FromMilliseconds(1);

    // Collect time-frequency pairs (with validity checking)
    std::vector<std::pair<double, double>> time_freq_pairs;  // (time_s, freq_hz)
    std::vector<int32_t> avg_rates;  // corresponding avg rates
    std::vector<double> detected_freqs;
    QuicTime win_start = start_time;

    while (win_start + window_size <= end_time) {
        QuicTime win_end = win_start + window_size;

        // Extract window samples from SMOOTHED signal
        std::vector<FreqSignalSample> win_samples;
        for (const auto& s : smoothed_samples) {
            if (s.time >= win_start && s.time <= win_end) {
                win_samples.push_back(s);
            }
        }

        // ===== APPLY CONSTRAINTS in AnalyzeWindow =====
        AnalysisResult result = AnalyzeWindow(win_samples, window_size, expected_freq_hz);

        // Apply threshold check if specified (interval phase)
        bool accept = result.valid;  // Start with AnalyzeWindow's validity decision
        if (accept && threshold_freq_hz > 0.0 && result.peak_freq_hz < threshold_freq_hz) {
            accept = false;
        }

        // Only accept VALID results that pass all constraints
        if (accept && result.peak_freq_hz > 0.0) {
            detected_freqs.push_back(result.peak_freq_hz);

            // Store time-frequency pair (use window start time)
            double time_s = (win_start - QuicTime::Zero()).ToMicroseconds() / 1000000.0;
            time_freq_pairs.push_back({time_s, result.peak_freq_hz});
            avg_rates.push_back(result.avg_rate_kbps);
        }

        // Advance
        win_start = win_start + step_size;
    }

    // Extract Single Peaks (similar to Python logic)
    // Only output peaks with duration > 0 (at least 2 consecutive windows)
    if (freq_analysis_trace_cb_ && !time_freq_pairs.empty()) {
        double window_size_s =
            static_cast<double>(window_size.ToMicroseconds()) / 1000000.0;
        size_t start_idx = 0;
        for (size_t i = 1; i < time_freq_pairs.size(); ++i) {
            double freq_diff = std::abs(time_freq_pairs[i].second - time_freq_pairs[i-1].second);

            // If frequency jump exceeds tolerance, this is a new peak
            if (freq_diff > kPeakFreqTolerance) {
                // Output the previous peak only if it passes duration check
                if (i - start_idx >= 1) {
                    double peak_start_time = time_freq_pairs[start_idx].first;
                    double peak_end_time = time_freq_pairs[i-1].first + window_size_s;
                    double peak_duration = peak_end_time - peak_start_time;

                    // Find max frequency in this peak segment
                    double max_freq = time_freq_pairs[start_idx].second;
                    int32_t avg_rate_sum = avg_rates[start_idx];
                    int32_t count = 1;
                    for (size_t j = start_idx + 1; j < i; ++j) {
                        if (time_freq_pairs[j].second > max_freq) {
                            max_freq = time_freq_pairs[j].second;
                        }
                        avg_rate_sum += avg_rates[j];
                        count++;
                    }
                    int32_t avg_rate = avg_rate_sum / count;

                    // ===== CONSTRAINT 4: Minimum Peak Duration Check =====
                    // Peak must span at least kMinPeakDurationCycles cycles
                    double min_duration_s = (expected_freq_hz > 0.0) ?
                        (kMinPeakDurationCycles / expected_freq_hz) : 0.020;  // Default 20ms

                    if (peak_duration >= min_duration_s) {
                        // Output: (start_time_s, duration_s, sender_peak_freq_hz, receiver_peak_freq_hz, avg_rate_kbps)
                        freq_analysis_trace_cb_(peak_start_time, peak_duration, sender_max_peak_freq_hz_, max_freq, avg_rate);
                    }
                    // Otherwise: reject transient peak
                }

                // Start new peak
                start_idx = i;
            }
        }

        // Output the last peak only if it passes duration check
        size_t last_peak_size = time_freq_pairs.size() - start_idx;
        if (last_peak_size >= 1) {
            double peak_start_time = time_freq_pairs[start_idx].first;
            double peak_end_time = time_freq_pairs.back().first + window_size_s;
            double peak_duration = peak_end_time - peak_start_time;

            double max_freq = time_freq_pairs[start_idx].second;
            int32_t avg_rate_sum = avg_rates[start_idx];
            int32_t count = 1;
            for (size_t j = start_idx + 1; j < time_freq_pairs.size(); ++j) {
                if (time_freq_pairs[j].second > max_freq) {
                    max_freq = time_freq_pairs[j].second;
                }
                avg_rate_sum += avg_rates[j];
                count++;
            }
            int32_t avg_rate = avg_rate_sum / count;

            // ===== CONSTRAINT 4: Minimum Peak Duration Check =====
            double min_duration_s = (expected_freq_hz > 0.0) ?
                (kMinPeakDurationCycles / expected_freq_hz) : 0.020;  // Default 20ms

            if (peak_duration >= min_duration_s) {
                freq_analysis_trace_cb_(peak_start_time, peak_duration, sender_max_peak_freq_hz_, max_freq, avg_rate);
            }
            // Otherwise: reject transient peak
        }
    }

    // Trajectory Stability Analysis for Controller Feedback
    // Instead of simple Max, use Median of detected frequencies to filter transient noise
    if (!detected_freqs.empty() && threshold_freq_hz <= 0.0) { // Only update controller state if not in threshold check mode (i.e. in UP phase)
        std::sort(detected_freqs.begin(), detected_freqs.end());
        double median_freq = detected_freqs[detected_freqs.size() / 2];

        // Update the tracked peak frequency with the robust median estimate
        if (median_freq > last_up_phase_peak_freq_) {
            last_up_phase_peak_freq_ = median_freq;
        }
    }
}

void FreqCCv3Sender::PerformRttFreqAnalysis(QuicTime start_time, QuicTime end_time,
                                            double threshold_freq_hz,
                                            double expected_freq_hz) {
    if (start_time >= end_time || rtt_signal_history_.empty()) return;

    std::vector<RttSignalSample> range_samples;
    for (const auto& s : rtt_signal_history_) {
        if (s.time >= start_time && s.time <= end_time) {
            range_samples.push_back(s);
        }
    }
    if (range_samples.empty()) return;

    TimeDelta duration = end_time - start_time;
    TimeDelta window_size = TimeDelta::Zero();
    if (expected_freq_hz > 0.0) {
        double win_sec = CalculateRttSTFTWindowSize(expected_freq_hz);
        if (win_sec > 0.0) {
            window_size =
                TimeDelta::FromMicroseconds(static_cast<int64_t>(win_sec * 1000000.0));
        }
    } else if (last_up_rtt_window_size_ > TimeDelta::Zero()) {
        window_size = last_up_rtt_window_size_;
    } else {
        double win_sec = CalculateRttSTFTWindowSize(last_up_rtt_peak_freq_);
        if (win_sec > 0.0) {
            window_size =
                TimeDelta::FromMicroseconds(static_cast<int64_t>(win_sec * 1000000.0));
        } else {
            TimeDelta min_rtt = model_.MinRtt();
            if (min_rtt.IsZero() && rtt_stats_ != nullptr) {
                min_rtt = rtt_stats_->MinOrInitialRtt();
            }
            window_size = (!min_rtt.IsZero())
                              ? TimeDelta::FromMicroseconds(3 * min_rtt.ToMicroseconds())
                              : duration;
        }
    }

    if (window_size > duration) window_size = duration;
    if (window_size <= TimeDelta::Zero()) return;

    last_up_rtt_window_size_ = window_size;

    TimeDelta step_size = TimeDelta::FromMicroseconds(
        static_cast<int64_t>(window_size.ToMicroseconds() * 0.1));
    if (step_size < TimeDelta::FromMilliseconds(5)) {
        step_size = TimeDelta::FromMilliseconds(5);
    }

    std::vector<std::pair<double, double>> time_freq_pairs;
    std::vector<double> avg_rtts_ms;
    std::vector<double> detected_freqs;
    QuicTime win_start = start_time;

    while (win_start + window_size <= end_time) {
        QuicTime win_end = win_start + window_size;

        std::vector<RttSignalSample> win_samples;
        for (const auto& s : range_samples) {
            if (s.time >= win_start && s.time <= win_end) {
                win_samples.push_back(s);
            }
        }

        RttAnalysisResult result =
            AnalyzeRttWindow(win_samples, window_size, expected_freq_hz);

        bool accept = result.valid;
        if (accept && threshold_freq_hz > 0.0 && result.peak_freq_hz < threshold_freq_hz) {
            accept = false;
        }

        if (accept && result.peak_freq_hz > 0.0) {
            detected_freqs.push_back(result.peak_freq_hz);
            double time_s = (win_start - QuicTime::Zero()).ToMicroseconds() / 1000000.0;
            time_freq_pairs.push_back({time_s, result.peak_freq_hz});
            avg_rtts_ms.push_back(result.avg_rtt_ms);
        }

        win_start = win_start + step_size;
    }

    if (rtt_freq_analysis_trace_cb_ && !time_freq_pairs.empty()) {
        double window_size_s =
            static_cast<double>(window_size.ToMicroseconds()) / 1000000.0;
        size_t start_idx = 0;
        for (size_t i = 1; i < time_freq_pairs.size(); ++i) {
            double freq_diff =
                std::abs(time_freq_pairs[i].second - time_freq_pairs[i - 1].second);
            if (freq_diff > kPeakFreqTolerance) {
                if (i - start_idx >= 1) {
                    double peak_start_time = time_freq_pairs[start_idx].first;
                    double peak_end_time = time_freq_pairs[i - 1].first + window_size_s;
                    double peak_duration = peak_end_time - peak_start_time;

                    double max_freq = time_freq_pairs[start_idx].second;
                    double avg_rtt_sum = avg_rtts_ms[start_idx];
                    int count = 1;
                    for (size_t j = start_idx + 1; j < i; ++j) {
                        if (time_freq_pairs[j].second > max_freq) {
                            max_freq = time_freq_pairs[j].second;
                        }
                        avg_rtt_sum += avg_rtts_ms[j];
                        count++;
                    }
                    double avg_rtt_ms = avg_rtt_sum / count;
                    double min_duration_s = (expected_freq_hz > 0.0)
                        ? (kMinPeakDurationCycles / expected_freq_hz) : 0.020;
                    if (peak_duration >= min_duration_s) {
                        rtt_freq_analysis_trace_cb_(peak_start_time, peak_duration,
                                                    sender_max_peak_freq_hz_, max_freq,
                                                    avg_rtt_ms);
                    }
                }
                start_idx = i;
            }
        }

        size_t last_peak_size = time_freq_pairs.size() - start_idx;
        if (last_peak_size >= 1) {
            double peak_start_time = time_freq_pairs[start_idx].first;
            double peak_end_time = time_freq_pairs.back().first + window_size_s;
            double peak_duration = peak_end_time - peak_start_time;

            double max_freq = time_freq_pairs[start_idx].second;
            double avg_rtt_sum = avg_rtts_ms[start_idx];
            int count = 1;
            for (size_t j = start_idx + 1; j < time_freq_pairs.size(); ++j) {
                if (time_freq_pairs[j].second > max_freq) {
                    max_freq = time_freq_pairs[j].second;
                }
                avg_rtt_sum += avg_rtts_ms[j];
                count++;
            }
            double avg_rtt_ms = avg_rtt_sum / count;
            double min_duration_s = (expected_freq_hz > 0.0)
                ? (kMinPeakDurationCycles / expected_freq_hz) : 0.020;
            if (peak_duration >= min_duration_s) {
                rtt_freq_analysis_trace_cb_(peak_start_time, peak_duration,
                                            sender_max_peak_freq_hz_, max_freq,
                                            avg_rtt_ms);
            }
        }
    }

    if (!detected_freqs.empty() && threshold_freq_hz <= 0.0) {
        std::sort(detected_freqs.begin(), detected_freqs.end());
        double median_freq = detected_freqs[detected_freqs.size() / 2];
        if (median_freq > last_up_rtt_peak_freq_) {
            last_up_rtt_peak_freq_ = median_freq;
        }
    }
}
}  // namespace dqc
