// FreqCC - Frequency-modulated Congestion Control based on BBRv2
// This algorithm adds periodic oscillation to the sending rate,
// similar to how ICC adjusts its rate.

#include "freqcc_sender.h"

#include <cmath>
#include <algorithm>

#include "quic_logging.h"
#include "quic_bbr2_probe_bw.h"

namespace dqc {

namespace {
// Default oscillation parameters
const double kDefaultOscillationFreqHz = 1.0;  // 1 Hz default
}  // namespace

FreqccSender::FreqccSender(QuicTime now,
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
      amplitude_mode_(FreqAmplitudeMode::kFixed),
      fixed_amplitude_bps_(0),  // No oscillation by default
      oscillation_mode_(FreqOscillationMode::kAfterDrain),
      drain_completed_(false),
      oscillation_start_time_(QuicTime::Zero()),
      last_mode_(Bbr2Mode::STARTUP),
      current_time_(now) {
  QUIC_DVLOG(2) << this << " Initializing FreqccSender @ " << now;
}

void FreqccSender::SetOscillationFrequency(double freq_hz) {
  oscillation_freq_hz_ = freq_hz;
}

void FreqccSender::SetOscillationAmplitude(FreqAmplitudeMode mode, uint64_t fixed_bps) {
  amplitude_mode_ = mode;
  fixed_amplitude_bps_ = fixed_bps;
}

void FreqccSender::SetOscillationMode(FreqOscillationMode mode) {
  oscillation_mode_ = mode;
}

uint64_t FreqccSender::GetCurrentAmplitudeBps() const {
  QuicBandwidth max_bw = BandwidthEstimate();
  QuicBandwidth base_rate = Bbr2Sender::PacingRate(0);

  switch (amplitude_mode_) {
    case FreqAmplitudeMode::kFixed:
      return fixed_amplitude_bps_;
    case FreqAmplitudeMode::kMiu2:
      return max_bw.ToBitsPerSecond() / 2;
    case FreqAmplitudeMode::kMiu3:
      return max_bw.ToBitsPerSecond() / 3;
    case FreqAmplitudeMode::kMiu4:
      return max_bw.ToBitsPerSecond() / 4;
    case FreqAmplitudeMode::kMiu8:
      return max_bw.ToBitsPerSecond() / 8;
    case FreqAmplitudeMode::kSR2:
      return base_rate.ToBitsPerSecond() / 2;
    case FreqAmplitudeMode::kSR3:
      return base_rate.ToBitsPerSecond() / 3;
    case FreqAmplitudeMode::kSR4:
      return base_rate.ToBitsPerSecond() / 4;
    case FreqAmplitudeMode::kSR8:
      return base_rate.ToBitsPerSecond() / 8;
    default:
      return 0;
  }
}

bool FreqccSender::ShouldOscillate() const {
  // Check if amplitude is non-zero
  if (GetCurrentAmplitudeBps() == 0) {
    return false;
  }

  // Get current mode from parent
  Bbr2Mode current_mode = InSlowStart() ? Bbr2Mode::STARTUP :
      (InRecovery() ? Bbr2Mode::PROBE_RTT : last_mode_);

  switch (oscillation_mode_) {
    case FreqOscillationMode::kAfterDrain:
      // Oscillate after drain is completed
      return drain_completed_;
    case FreqOscillationMode::kOnlyProbeBW:
      // Only oscillate during ProbeBW phase
      return drain_completed_ && (current_mode == Bbr2Mode::PROBE_BW);
    default:
      return false;
  }
}

QuicBandwidth FreqccSender::CalculateOscillationOffset(QuicTime now) const {
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

void FreqccSender::OnPacketSent(QuicTime sent_time,
                                QuicByteCount bytes_in_flight,
                                QuicPacketNumber packet_number,
                                QuicByteCount bytes,
                                HasRetransmittableData is_retransmittable) {
  // Update current time
  current_time_ = sent_time;

  // Call parent implementation
  Bbr2Sender::OnPacketSent(sent_time, bytes_in_flight, packet_number, bytes, is_retransmittable);
}

void FreqccSender::OnCongestionEvent(bool rtt_updated,
                                     QuicByteCount prior_in_flight,
                                     QuicTime event_time,
                                     const AckedPacketVector& acked_packets,
                                     const LostPacketVector& lost_packets) {
  // Update current time
  current_time_ = event_time;

  // Call parent implementation
  Bbr2Sender::OnCongestionEvent(rtt_updated, prior_in_flight, event_time,
                                acked_packets, lost_packets);

  // Get current mode after parent update (directly from protected member)
  Bbr2Mode current_mode = mode_;
  last_mode_ = current_mode;

  // Detect transition to PROBE_BW (first time after DRAIN)
  // BBRv2 flow: STARTUP -> DRAIN -> PROBE_BW
  if (!drain_completed_) {
    // If we see PROBE_BW mode, we're past the initial startup and drain
    if (current_mode == Bbr2Mode::PROBE_BW) {
      drain_completed_ = true;
      oscillation_start_time_ = event_time;
    }
  }
}

QuicBandwidth FreqccSender::PacingRate(QuicByteCount bytes_in_flight) const {
  // Get base pacing rate from BBRv2
  QuicBandwidth base_rate = Bbr2Sender::PacingRate(bytes_in_flight);

  // If not oscillating, return base rate
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
  // This is similar to ICC's approach where it adjusts cur_rate
  int64_t base_bps = static_cast<int64_t>(base_rate.ToBitsPerSecond());
  int64_t offset_bps = static_cast<int64_t>(offset.ToBitsPerSecond());
  int64_t final_bps = base_bps + offset_bps;

  // Ensure pacing rate doesn't go negative or too low
  // Similar to ICC's: cur_rate = max(cur_rate, 0.2 * packet_size*8/1000 / min_rtt)
  const int64_t min_rate_bps = 1000;  // 1 kbps minimum
  if (final_bps < min_rate_bps) {
    final_bps = min_rate_bps;
  }

  return QuicBandwidth::FromBitsPerSecond(static_cast<uint64_t>(final_bps));
}

int32_t FreqccSender::GetCurrentBbrModeIndex() const {
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
