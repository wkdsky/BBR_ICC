// FreqCCv3 - Enhanced Frequency-modulated Congestion Control based on modified BBRv2
// This algorithm modifies BBRv2's behavior with NEW_REFILL and oscillation during PROBE_UP

#include "freqccv3_sender.h"

#include <cmath>
#include <algorithm>

#include "quic_logging.h"
#include "quic_bbr2_probe_bw.h"

namespace dqc {

namespace {
// Default oscillation parameters
const double kDefaultOscillationFreqHz = 1.0;  // 1 Hz default
}  // namespace

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
      initial_freq_hz_(kDefaultOscillationFreqHz),
      amplitude_mode_(FreqCCv3AmplitudeMode::kFixed),
      fixed_amplitude_bps_(0),  // No oscillation by default
      drain_completed_(false),
      oscillation_start_time_(QuicTime::Zero()),
      last_mode_(Bbr2Mode::STARTUP),
      last_probe_bw_phase_(Bbr2ProbeBwMode::CyclePhase::PROBE_NOT_STARTED),
      current_time_(now),
      new_refill_state_(NewRefillState::kNotInNewRefill),
      in_new_refill_(false),
      // Initialize adaptive frequency state
      up_phase_count_(0),
      up_phase_start_time_(QuicTime::Zero()),
      last_up_duration_sec_(0.0),
      // Initialize adaptive pacing gain
      up_pacing_gain_(kDefaultUpPacingGain) {
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
  // Only oscillate during PROBE_UP phase
  if (GetCurrentAmplitudeBps() == 0) {
    return false;
  }

  // Must be in PROBE_BW mode and PROBE_UP phase
  if (!drain_completed_ || mode_ != Bbr2Mode::PROBE_BW) {
    return false;
  }

  Bbr2ProbeBwMode::CyclePhase current_phase = GetCurrentProbeBwPhase();
  return current_phase == Bbr2ProbeBwMode::CyclePhase::PROBE_UP;
}

QuicBandwidth FreqCCv3Sender::CalculateOscillationOffset(QuicTime now) const {
  if (!ShouldOscillate() || oscillation_start_time_ == QuicTime::Zero()) {
    return QuicBandwidth::Zero();
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

  return QuicBandwidth::FromBitsPerSecond(offset_bps);
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
  } else if (bytes_in_flight <= low_threshold) {
    // Inflight <= 0.70*BDP + 2*MSS + MaxAckHeight: need to fill
    new_refill_state_ = NewRefillState::kFilling;
    QUIC_DVLOG(3) << "FreqCCv3: Entering kFilling state, pacing_gain=1.0";
  } else {
    // Neither condition met: done, exit immediately
    new_refill_state_ = NewRefillState::kDone;
    QUIC_DVLOG(3) << "FreqCCv3: Entering kDone state, will exit NEW_REFILL";
  }
}

float FreqCCv3Sender::GetNewRefillPacingGain() const {
  switch (new_refill_state_) {
    case NewRefillState::kDraining:
      return kNewRefillDrainingPacingGain;  // 0.75
    case NewRefillState::kFilling:
      return kNewRefillFillingPacingGain;   // 1.0
    case NewRefillState::kDone:
    case NewRefillState::kNotInNewRefill:
    default:
      return 1.0f;
  }
}

bool FreqCCv3Sender::ShouldExitNewRefill(QuicByteCount bytes_in_flight) const {
  if (!in_new_refill_) {
    return false;
  }

  QuicByteCount high_threshold = CalculateInflightThreshold(kNewRefillHighThreshold);

  switch (new_refill_state_) {
    case NewRefillState::kDraining:
      // Exit when inflight <= 0.75*BDP + 2*MSS + MaxAckHeight
      return bytes_in_flight <= high_threshold;

    case NewRefillState::kFilling:
      // Exit when inflight >= 0.75*BDP + 2*MSS + MaxAckHeight
      return bytes_in_flight >= high_threshold;

    case NewRefillState::kDone:
      // Exit immediately
      return true;

    case NewRefillState::kNotInNewRefill:
    default:
      return true;
  }
}

void FreqCCv3Sender::OnPacketSent(QuicTime sent_time,
                                  QuicByteCount bytes_in_flight,
                                  QuicPacketNumber packet_number,
                                  QuicByteCount bytes,
                                  HasRetransmittableData is_retransmittable) {
  // Update current time
  current_time_ = sent_time;

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

  // Get phase before parent update
  Bbr2ProbeBwMode::CyclePhase phase_before = GetCurrentProbeBwPhase();

  // Call parent implementation
  Bbr2Sender::OnCongestionEvent(rtt_updated, prior_in_flight, event_time,
                                acked_packets, lost_packets);

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
      // Reset oscillation start time when entering PROBE_UP
      oscillation_start_time_ = event_time;
      up_phase_start_time_ = event_time;  // Record UP phase start time for adaptive frequency

      QUIC_DVLOG(2) << "FreqCCv3: Entering PROBE_UP, starting oscillation @ " << event_time
                    << ", freq=" << oscillation_freq_hz_ << "Hz, up_phase_count=" << up_phase_count_
                    << ", up_pacing_gain=" << up_pacing_gain_
                    << ", amplitude_bps=" << GetCurrentAmplitudeBps();

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
    bool need_cleanup = !in_probe_up && (oscillation_start_time_ != QuicTime::Zero());

    if (leaving_probe_up || need_cleanup) {
      // Calculate UP phase duration and adapt frequency for next UP phase
      if (up_phase_start_time_ != QuicTime::Zero()) {
        TimeDelta up_duration = event_time - up_phase_start_time_;
        last_up_duration_sec_ = static_cast<double>(up_duration.ToMicroseconds()) / 1000000.0;

        // Calculate how many cycles occurred in this UP phase
        double cycles_in_up = last_up_duration_sec_ * oscillation_freq_hz_;

        QUIC_DVLOG(2) << "FreqCCv3: Leaving PROBE_UP, duration=" << last_up_duration_sec_
                      << "s, cycles=" << cycles_in_up << ", freq=" << oscillation_freq_hz_ << "Hz"
                      << ", current_up_pacing_gain=" << up_pacing_gain_;

        // Adapt frequency if cycles are outside [kMinCyclesPerUp, kMaxCyclesPerUp]
        if (last_up_duration_sec_ > 0.001) {  // Avoid division by zero, minimum 1ms
          if (cycles_in_up < kMinCyclesPerUp || cycles_in_up > kMaxCyclesPerUp) {
            // Recalculate frequency to achieve kTargetCyclesPerUp cycles in the recorded duration
            double new_freq = static_cast<double>(kTargetCyclesPerUp) / last_up_duration_sec_;

            // Clamp frequency to [kMinFreqHz, kMaxFreqHz]
            if (new_freq < kMinFreqHz) {
              new_freq = kMinFreqHz;
            } else if (new_freq > kMaxFreqHz) {
              new_freq = kMaxFreqHz;
            }

            QUIC_DVLOG(2) << "FreqCCv3: Adapting frequency from " << oscillation_freq_hz_
                          << "Hz to " << new_freq << "Hz (target " << kTargetCyclesPerUp
                          << " cycles in " << last_up_duration_sec_ << "s)";
            oscillation_freq_hz_ = new_freq;
          }

          // Adapt pacing gain based on UP phase duration
          // If UP phase was too short (< kMinUpDurationSec), reduce pacing gain
          // If UP phase was too long (> kMaxUpDurationSec), increase pacing gain
          float new_pacing_gain = up_pacing_gain_;
          if (last_up_duration_sec_ < kMinUpDurationSec) {
            // UP phase too short, reduce pacing gain to slow down growth
            new_pacing_gain -= kPacingGainAdjustStep;
            QUIC_DVLOG(2) << "FreqCCv3: UP phase too short (" << last_up_duration_sec_
                          << "s < " << kMinUpDurationSec << "s), reducing pacing gain";
          } else if (last_up_duration_sec_ > kMaxUpDurationSec) {
            // UP phase too long, increase pacing gain to speed up growth
            new_pacing_gain += kPacingGainAdjustStep;
            QUIC_DVLOG(2) << "FreqCCv3: UP phase too long (" << last_up_duration_sec_
                          << "s > " << kMaxUpDurationSec << "s), increasing pacing gain";
          }

          // Clamp pacing gain to [kMinUpPacingGain, kMaxUpPacingGain]
          if (new_pacing_gain < kMinUpPacingGain) {
            new_pacing_gain = kMinUpPacingGain;
          } else if (new_pacing_gain > kMaxUpPacingGain) {
            new_pacing_gain = kMaxUpPacingGain;
          }

          if (new_pacing_gain != up_pacing_gain_) {
            QUIC_DVLOG(2) << "FreqCCv3: Adapting UP pacing gain from " << up_pacing_gain_
                          << " to " << new_pacing_gain;
            up_pacing_gain_ = new_pacing_gain;
          }

          // Call UP phase trace callback if set
          if (up_phase_trace_cb_) {
            double start_time_sec = static_cast<double>(up_phase_start_time_.ToDebuggingValue()) / 1000000.0;
            double duration_ms = last_up_duration_sec_ * 1000.0;
            int actual_cycles = static_cast<int>(cycles_in_up + 0.5);  // Round to nearest integer
            int32_t bw_estimate_kbps = static_cast<int32_t>(model_.MaxBandwidth().ToKBitsPerSecond());
            up_phase_trace_cb_(start_time_sec, duration_ms, oscillation_freq_hz_, actual_cycles, bw_estimate_kbps);
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
  QuicBandwidth offset = CalculateOscillationOffset(current_time_);

  // Apply offset to base rate
  int64_t base_bps = static_cast<int64_t>(base_rate.ToBitsPerSecond());
  int64_t offset_bps = static_cast<int64_t>(offset.ToBitsPerSecond());
  int64_t final_bps = base_bps + offset_bps;

  // Ensure pacing rate doesn't go negative or too low
  const int64_t min_rate_bps = 1000;  // 1 kbps minimum
  if (final_bps < min_rate_bps) {
    final_bps = min_rate_bps;
  }

  return QuicBandwidth::FromBitsPerSecond(static_cast<uint64_t>(final_bps));
}

QuicByteCount FreqCCv3Sender::GetCongestionWindow() const {
  // Return BBRv2's original cwnd
  return cwnd_;
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

}  // namespace dqc
