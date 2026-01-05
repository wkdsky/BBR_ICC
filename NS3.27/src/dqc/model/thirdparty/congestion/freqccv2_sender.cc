// FreqCCv2 - Enhanced Frequency-modulated Congestion Control based on modified BBRv2
// This algorithm modifies BBRv2's behavior and adds oscillation capability

#include "freqccv2_sender.h"

#include <cmath>
#include <algorithm>

#include "quic_logging.h"
#include "quic_bbr2_probe_bw.h"

namespace dqc {

namespace {
// Default oscillation parameters
const double kDefaultOscillationFreqHz = 1.0;  // 1 Hz default
}  // namespace

FreqCCv2Sender::FreqCCv2Sender(QuicTime now,
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
      amplitude_mode_(FreqCCv2AmplitudeMode::kFixed),
      fixed_amplitude_bps_(0),  // No oscillation by default
      oscillation_mode_(FreqCCv2OscillationMode::kAfterDrain),
      drain_completed_(false),
      oscillation_start_time_(QuicTime::Zero()),
      last_mode_(Bbr2Mode::STARTUP),
      last_probe_bw_phase_(Bbr2ProbeBwMode::CyclePhase::PROBE_NOT_STARTED),
      current_time_(now),
      probe_down_start_time_(QuicTime::Zero()),
      min_rtt_at_probe_down_start_(TimeDelta::Zero()) {
  QUIC_DVLOG(2) << this << " Initializing FreqCCv2Sender @ " << now;
}

void FreqCCv2Sender::SetOscillationFrequency(double freq_hz) {
  oscillation_freq_hz_ = freq_hz;
}

void FreqCCv2Sender::SetOscillationAmplitude(FreqCCv2AmplitudeMode mode, uint64_t fixed_bps) {
  amplitude_mode_ = mode;
  fixed_amplitude_bps_ = fixed_bps;
}

void FreqCCv2Sender::SetOscillationMode(FreqCCv2OscillationMode mode) {
  oscillation_mode_ = mode;
}

uint64_t FreqCCv2Sender::GetCurrentAmplitudeBps() const {
  QuicBandwidth max_bw = BandwidthEstimate();
  QuicBandwidth base_rate = Bbr2Sender::PacingRate(0);

  switch (amplitude_mode_) {
    case FreqCCv2AmplitudeMode::kFixed:
      return fixed_amplitude_bps_;
    case FreqCCv2AmplitudeMode::kMiu2:
      return max_bw.ToBitsPerSecond() / 2;
    case FreqCCv2AmplitudeMode::kMiu3:
      return max_bw.ToBitsPerSecond() / 3;
    case FreqCCv2AmplitudeMode::kMiu4:
      return max_bw.ToBitsPerSecond() / 4;
    case FreqCCv2AmplitudeMode::kMiu8:
      return max_bw.ToBitsPerSecond() / 8;
    case FreqCCv2AmplitudeMode::kSR2:
      return base_rate.ToBitsPerSecond() / 2;
    case FreqCCv2AmplitudeMode::kSR3:
      return base_rate.ToBitsPerSecond() / 3;
    case FreqCCv2AmplitudeMode::kSR4:
      return base_rate.ToBitsPerSecond() / 4;
    case FreqCCv2AmplitudeMode::kSR8:
      return base_rate.ToBitsPerSecond() / 8;
    default:
      return 0;
  }
}

Bbr2ProbeBwMode::CyclePhase FreqCCv2Sender::GetCurrentProbeBwPhase() const {
  DebugState state = ExportDebugState();
  if (state.mode == Bbr2Mode::PROBE_BW) {
    return state.probe_bw.phase;
  }
  return Bbr2ProbeBwMode::CyclePhase::PROBE_NOT_STARTED;
}

bool FreqCCv2Sender::ShouldOscillate() const {
  // Check if amplitude is non-zero
  if (GetCurrentAmplitudeBps() == 0) {
    return false;
  }

  // Get current mode and phase
  Bbr2Mode current_mode = mode_;
  Bbr2ProbeBwMode::CyclePhase current_phase = GetCurrentProbeBwPhase();

  switch (oscillation_mode_) {
    case FreqCCv2OscillationMode::kAfterDrain:
      // Oscillate after drain is completed
      return drain_completed_;

    case FreqCCv2OscillationMode::kOnlyProbeBW:
      // Only oscillate during ProbeBW phase
      return drain_completed_ && (current_mode == Bbr2Mode::PROBE_BW);

    case FreqCCv2OscillationMode::kRefillUp:
      // Only oscillate during PROBE_REFILL and PROBE_UP phases
      if (!drain_completed_ || current_mode != Bbr2Mode::PROBE_BW) {
        return false;
      }
      return (current_phase == Bbr2ProbeBwMode::CyclePhase::PROBE_REFILL ||
              current_phase == Bbr2ProbeBwMode::CyclePhase::PROBE_UP);

    default:
      return false;
  }
}

QuicBandwidth FreqCCv2Sender::CalculateOscillationOffset(QuicTime now) const {
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

bool FreqCCv2Sender::ShouldExitProbeDown() const {
  // FreqCCv2's modified PROBE_DOWN exit condition:
  // Exit when: current RTT <= 1.05 * min_RTT OR phase lasted >= 2 * min_RTT

  if (probe_down_start_time_ == QuicTime::Zero()) {
    return false;
  }

  TimeDelta min_rtt = rtt_stats_->min_rtt();
  if (min_rtt.IsZero()) {
    return false;
  }

  // Check condition 1: current RTT <= 1.05 * min_RTT
  TimeDelta latest_rtt = rtt_stats_->latest_rtt();
  if (!latest_rtt.IsZero()) {
    double rtt_ratio = static_cast<double>(latest_rtt.ToMicroseconds()) /
                       static_cast<double>(min_rtt.ToMicroseconds());
    if (rtt_ratio <= kFreqCCv2RttExitThreshold) {
      QUIC_DVLOG(3) << "FreqCCv2: Exiting PROBE_DOWN due to RTT ratio " << rtt_ratio
                    << " <= " << kFreqCCv2RttExitThreshold;
      return true;
    }
  }

  // Check condition 2: phase lasted >= 2 * min_RTT
  if (current_time_ > probe_down_start_time_) {
    TimeDelta phase_duration = current_time_ - probe_down_start_time_;
    TimeDelta min_duration = min_rtt * kFreqCCv2MinProbeDownRounds;
    if (phase_duration >= min_duration) {
      QUIC_DVLOG(3) << "FreqCCv2: Exiting PROBE_DOWN due to duration "
                    << phase_duration.ToMicroseconds() << "us >= "
                    << min_duration.ToMicroseconds() << "us";
      return true;
    }
  }

  return false;
}

float FreqCCv2Sender::GetModifiedPacingGainForPhase() const {
  Bbr2ProbeBwMode::CyclePhase current_phase = GetCurrentProbeBwPhase();

  if (current_phase == Bbr2ProbeBwMode::CyclePhase::PROBE_UP) {
    return kFreqCCv2ProbeUpPacingGain;  // 1.05 instead of 1.25
  }
  if (current_phase == Bbr2ProbeBwMode::CyclePhase::PROBE_DOWN) {
    return kFreqCCv2ProbeDownPacingGain;  // 0.5 instead of 0.75
  }
  // For other phases, use default (1.0)
  return Params().probe_bw_default_pacing_gain;
}

void FreqCCv2Sender::OnPacketSent(QuicTime sent_time,
                                  QuicByteCount bytes_in_flight,
                                  QuicPacketNumber packet_number,
                                  QuicByteCount bytes,
                                  HasRetransmittableData is_retransmittable) {
  // Update current time
  current_time_ = sent_time;

  // Call parent implementation
  Bbr2Sender::OnPacketSent(sent_time, bytes_in_flight, packet_number, bytes, is_retransmittable);
}

void FreqCCv2Sender::OnCongestionEvent(bool rtt_updated,
                                       QuicByteCount prior_in_flight,
                                       QuicTime event_time,
                                       const AckedPacketVector& acked_packets,
                                       const LostPacketVector& lost_packets) {
  // Update current time
  current_time_ = event_time;

  // Get phase before parent update (for tracking PROBE_DOWN transitions)
  Bbr2ProbeBwMode::CyclePhase phase_before = GetCurrentProbeBwPhase();

  // Call parent implementation
  Bbr2Sender::OnCongestionEvent(rtt_updated, prior_in_flight, event_time,
                                acked_packets, lost_packets);

  // Get current mode and phase after parent update
  Bbr2Mode current_mode = mode_;
  Bbr2ProbeBwMode::CyclePhase current_phase = GetCurrentProbeBwPhase();
  last_mode_ = current_mode;
  last_probe_bw_phase_ = current_phase;

  // Detect transition to PROBE_BW (first time after DRAIN)
  if (!drain_completed_) {
    if (current_mode == Bbr2Mode::PROBE_BW) {
      drain_completed_ = true;
      oscillation_start_time_ = event_time;
    }
  }

  // Track PROBE_DOWN start time for FreqCCv2's modified exit condition
  if (current_mode == Bbr2Mode::PROBE_BW) {
    if (current_phase == Bbr2ProbeBwMode::CyclePhase::PROBE_DOWN &&
        phase_before != Bbr2ProbeBwMode::CyclePhase::PROBE_DOWN) {
      // Just entered PROBE_DOWN
      probe_down_start_time_ = event_time;
      min_rtt_at_probe_down_start_ = rtt_stats_->min_rtt();
      QUIC_DVLOG(3) << "FreqCCv2: Entering PROBE_DOWN @ " << event_time;
    }

    // Reset when leaving PROBE_DOWN
    if (current_phase != Bbr2ProbeBwMode::CyclePhase::PROBE_DOWN &&
        phase_before == Bbr2ProbeBwMode::CyclePhase::PROBE_DOWN) {
      probe_down_start_time_ = QuicTime::Zero();
    }
  }
}

QuicBandwidth FreqCCv2Sender::PacingRate(QuicByteCount bytes_in_flight) const {
  // Get base pacing rate from BBRv2
  QuicBandwidth base_rate = Bbr2Sender::PacingRate(bytes_in_flight);

  // Apply FreqCCv2's modified pacing gains for PROBE_BW phases
  if (mode_ == Bbr2Mode::PROBE_BW) {
    Bbr2ProbeBwMode::CyclePhase current_phase = GetCurrentProbeBwPhase();

    // Calculate the ratio to adjust the base rate
    float bbr2_gain = 1.0f;
    float freqccv2_gain = 1.0f;

    if (current_phase == Bbr2ProbeBwMode::CyclePhase::PROBE_UP) {
      bbr2_gain = Params().probe_bw_probe_up_pacing_gain;  // 1.25
      freqccv2_gain = kFreqCCv2ProbeUpPacingGain;          // 1.05
    } else if (current_phase == Bbr2ProbeBwMode::CyclePhase::PROBE_DOWN) {
      bbr2_gain = Params().probe_bw_probe_down_pacing_gain;  // 0.75
      freqccv2_gain = kFreqCCv2ProbeDownPacingGain;          // 0.5
    }

    if (bbr2_gain != freqccv2_gain && bbr2_gain > 0) {
      // Adjust base_rate: base_rate = base_rate / bbr2_gain * freqccv2_gain
      double adjustment = static_cast<double>(freqccv2_gain) / static_cast<double>(bbr2_gain);
      uint64_t adjusted_bps = static_cast<uint64_t>(
          static_cast<double>(base_rate.ToBitsPerSecond()) * adjustment);
      base_rate = QuicBandwidth::FromBitsPerSecond(adjusted_bps);
    }
  }

  // If not oscillating, return the (possibly modified) base rate
  if (!ShouldOscillate()) {
    return base_rate;
  }

  // Use current_time_ which is updated on each packet sent and ACK received
  if (current_time_ == QuicTime::Zero()) {
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

QuicByteCount FreqCCv2Sender::GetCongestionWindow() const {
  // Always return BBRv2's original cwnd, not affected by oscillation
  // This ensures BBRv2's state machine logic is not disturbed by oscillation
  // The actual sending rate is controlled by PacingRate(), which includes oscillation
  return cwnd_;
}

int32_t FreqCCv2Sender::GetCurrentBbrModeIndex() const {
  // Get debug state from parent
  DebugState state = ExportDebugState();

  // Convert mode and probe_bw phase to single index
  // 0: STARTUP
  // 1: DRAIN
  // 2: PROBE_BW_DOWN
  // 3: PROBE_BW_CRUISE
  // 4: PROBE_BW_REFILL
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
