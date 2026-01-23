// FreqCCv4 - Enhanced Frequency-modulated Congestion Control based on modified BBRv2

#include "freqccv4_sender.h"

#include <cmath>
#include <algorithm>

#include "quic_logging.h"
#include "quic_bbr2_probe_bw.h"
#include "random.h" 

namespace dqc {

namespace {
// Default oscillation parameters
const double kDefaultOscillationFreqHz = 60.0;  
}  // namespace

// Define static constexpr members for linking
constexpr float FreqCCv4Sender::kNewRefillHighThreshold;
constexpr float FreqCCv4Sender::kNewRefillLowThreshold;
constexpr float FreqCCv4Sender::kNewRefillDrainingPacingGain;
constexpr float FreqCCv4Sender::kNewRefillFillingPacingGain;

FreqCCv4Sender::FreqCCv4Sender(QuicTime now,
                               const RttStats* rtt_stats,
                               const QuicUnackedPacketMap* unacked_packets,
                               QuicPacketCount initial_cwnd_in_packets,
                               QuicPacketCount max_cwnd_in_packets,
                               Random* random,
                               QuicConnectionStats* stats,
                               bool enable_ecn)
    : Bbr2Sender(now, rtt_stats, unacked_packets, initial_cwnd_in_packets,
                 max_cwnd_in_packets, random, stats, enable_ecn),
      oscillation_freq_hz_(kDefaultOscillationFreqHz),
      amplitude_mode_(FreqCCv4AmplitudeMode::kFixed),
      fixed_amplitude_bps_(0),
      drain_completed_(false),
      current_time_(now),
      cruise_start_time_(QuicTime::Zero()),
      fluctuation_start_time_(QuicTime::Zero()),
      fluctuation_duration_(TimeDelta::Zero()),
      last_cruise_duration_(TimeDelta::FromMilliseconds(1000)), // Default 1s
      current_fluctuation_freq_hz_(kDefaultOscillationFreqHz),
      last_fluctuation_freq_hz_(0.0),
      is_fluctuating_(false),
      fluctuation_done_in_current_cruise_(false),
      new_refill_state_(NewRefillStateV4::kNotInNewRefill),
      in_new_refill_(false),
      trace_freq_hz_(0.0),
      trace_start_time_(QuicTime::Zero()),
      trace_duration_(TimeDelta::Zero()),
      last_mode_(Bbr2Mode::STARTUP),
      last_probe_bw_phase_(Bbr2ProbeBwMode::CyclePhase::PROBE_NOT_STARTED) {
  QUIC_DVLOG(2) << this << " Initializing FreqCCv4Sender @ " << now;
}

void FreqCCv4Sender::SetOscillationFrequency(double freq_hz) {
  oscillation_freq_hz_ = freq_hz;
  current_fluctuation_freq_hz_ = freq_hz;
}

void FreqCCv4Sender::SetOscillationAmplitude(FreqCCv4AmplitudeMode mode, uint64_t fixed_bps) {
  amplitude_mode_ = mode;
  fixed_amplitude_bps_ = fixed_bps;
}

uint64_t FreqCCv4Sender::GetCurrentAmplitudeBps() const {
  QuicBandwidth max_bw = BandwidthEstimate();
  QuicBandwidth base_rate = Bbr2Sender::PacingRate(0);

  switch (amplitude_mode_) {
    case FreqCCv4AmplitudeMode::kFixed:
      return fixed_amplitude_bps_;
    case FreqCCv4AmplitudeMode::kMiu2:
      return max_bw.ToBitsPerSecond() / 2;
    case FreqCCv4AmplitudeMode::kMiu3:
      return max_bw.ToBitsPerSecond() / 3;
    case FreqCCv4AmplitudeMode::kMiu4:
      return max_bw.ToBitsPerSecond() / 4;
    case FreqCCv4AmplitudeMode::kMiu8:
      return max_bw.ToBitsPerSecond() / 8;
    case FreqCCv4AmplitudeMode::kSR2:
      return base_rate.ToBitsPerSecond() / 2;
    case FreqCCv4AmplitudeMode::kSR3:
      return base_rate.ToBitsPerSecond() / 3;
    case FreqCCv4AmplitudeMode::kSR4:
      return base_rate.ToBitsPerSecond() / 4;
    case FreqCCv4AmplitudeMode::kSR8:
      return base_rate.ToBitsPerSecond() / 8;
    default:
      return 0;
  }
}

Bbr2ProbeBwMode::CyclePhase FreqCCv4Sender::GetCurrentProbeBwPhase() const {
  DebugState state = ExportDebugState();
  if (state.mode == Bbr2Mode::PROBE_BW) {
    return state.probe_bw.phase;
  }
  return Bbr2ProbeBwMode::CyclePhase::PROBE_NOT_STARTED;
}

bool FreqCCv4Sender::ShouldOscillate() const {
  if (GetCurrentAmplitudeBps() == 0) {
    return false;
  }

  // If already fluctuating, continue
  if (is_fluctuating_) {
      return true;
  }
  
  // If already done fluctuation in this cruise phase, stop
  if (fluctuation_done_in_current_cruise_) {
      return false;
  }

  // Must be in PROBE_BW mode and PROBE_CRUISE phase to start
  if (!drain_completed_ || mode_ != Bbr2Mode::PROBE_BW) {
    return false;
  }

  Bbr2ProbeBwMode::CyclePhase current_phase = GetCurrentProbeBwPhase();
  if (current_phase != Bbr2ProbeBwMode::CyclePhase::PROBE_CRUISE) {
      return false;
  }
  
  // Check delay: 2 * min_rtt after cruise start
  if (cruise_start_time_ == QuicTime::Zero()) {
      return false;
  }
  
  TimeDelta min_rtt = model_.MinRtt();
  if (min_rtt.IsZero()) {
      return false;
  }
  
  TimeDelta delay = min_rtt * 2;
  if (current_time_ > cruise_start_time_ + delay) {
      return true;
  }

  return false;
}

void FreqCCv4Sender::CalculateFluctuationParams() {
    // 1. Calculate target duration: 1/3 of the last cruise duration
    double cruise_sec = 0.0;
    if (last_cruise_duration_ > TimeDelta::Zero()) {
        cruise_sec = static_cast<double>(last_cruise_duration_.ToMicroseconds()) / 1000000.0;
    } else {
        // Fallback if no valid last cruise duration (e.g., first run or error)
        cruise_sec = 1.0; 
    }

    double duration_sec = cruise_sec / 3.0;

    // Safety: prevent extremely short durations which cause huge frequencies
    if (duration_sec < 0.005) { // Min 5ms duration
        duration_sec = 0.005;
    }

    // 2. Calculate frequency to fit exactly 5 cycles
    double target_freq = 5.0 / duration_sec;

    // Update state
    current_fluctuation_freq_hz_ = target_freq;
    fluctuation_duration_ = TimeDelta::FromMicroseconds(static_cast<int64_t>(duration_sec * 1000000));
    
    // Log for debugging
    QUIC_DVLOG(2) << "FreqCCv4 Params: Cruise=" << cruise_sec 
                 << "s, Fluctuation=" << duration_sec 
                 << "s, Freq=" << current_fluctuation_freq_hz_ << "Hz (5 cycles)";
}

int64_t FreqCCv4Sender::CalculateOscillationOffset(QuicTime now) const {
  if (!is_fluctuating_ || fluctuation_start_time_ == QuicTime::Zero()) {
    return 0;
  }

  // Calculate time since oscillation started
  TimeDelta elapsed = now - fluctuation_start_time_;
  double elapsed_seconds = static_cast<double>(elapsed.ToMicroseconds()) / 1000000.0;

  // Calculate the period of oscillation
  double period = 1.0 / current_fluctuation_freq_hz_;

  // Get position within the current period (0 to 1)
  double phase = fmod(elapsed_seconds, period) / period;

  // Triangle wave
  double triangle_value;
  if (phase < 0.25) {
    triangle_value = phase * 4.0;
  } else if (phase < 0.75) {
    triangle_value = 2.0 - phase * 4.0;
  } else {
    triangle_value = phase * 4.0 - 4.0;
  }

  uint64_t amplitude_bps = GetCurrentAmplitudeBps();
  int64_t offset_bps = static_cast<int64_t>(triangle_value * amplitude_bps);

  return offset_bps;
}

QuicByteCount FreqCCv4Sender::CalculateInflightThreshold(float bdp_factor) const {
  QuicByteCount bdp = model_.BDP(model_.MaxBandwidth());
  QuicByteCount threshold = static_cast<QuicByteCount>(bdp_factor * bdp) +
                            2 * kDefaultTCPMSS +
                            model_.MaxAckHeight();
  return threshold;
}

void FreqCCv4Sender::UpdateNewRefillState(QuicByteCount bytes_in_flight) {
  QuicByteCount high_threshold = CalculateInflightThreshold(kNewRefillHighThreshold);
  QuicByteCount low_threshold = CalculateInflightThreshold(kNewRefillLowThreshold);

  if (bytes_in_flight > high_threshold) {
    new_refill_state_ = NewRefillStateV4::kDraining;
  } else if (bytes_in_flight < low_threshold) {
    new_refill_state_ = NewRefillStateV4::kFilling;
  } else {
    new_refill_state_ = NewRefillStateV4::kDone;
  }
}

float FreqCCv4Sender::GetNewRefillPacingGain() const {
  return 1.0f;
}

bool FreqCCv4Sender::ShouldExitNewRefill(QuicByteCount bytes_in_flight) const {
  if (!in_new_refill_) {
    return false;
  }

  QuicByteCount high_threshold = CalculateInflightThreshold(kNewRefillHighThreshold);

  switch (new_refill_state_) {
    case NewRefillStateV4::kDraining:
      return bytes_in_flight <= high_threshold;
    case NewRefillStateV4::kFilling:
      return bytes_in_flight >= high_threshold;
    case NewRefillStateV4::kDone:
      return true;
    case NewRefillStateV4::kNotInNewRefill:
    default:
      return true;
  }
}

void FreqCCv4Sender::OnPacketSent(QuicTime sent_time,
                                  QuicByteCount bytes_in_flight,
                                  QuicPacketNumber packet_number,
                                  QuicByteCount bytes,
                                  HasRetransmittableData is_retransmittable) {
  current_time_ = sent_time;
  Bbr2Sender::OnPacketSent(sent_time, bytes_in_flight, packet_number, bytes, is_retransmittable);
}

void FreqCCv4Sender::OnCongestionEvent(bool rtt_updated,
                                       QuicByteCount prior_in_flight,
                                       QuicTime event_time,
                                       const AckedPacketVector& acked_packets,
                                       const LostPacketVector& lost_packets) {
  current_time_ = event_time;

  Bbr2ProbeBwMode::CyclePhase phase_before = GetCurrentProbeBwPhase();

  Bbr2Sender::OnCongestionEvent(rtt_updated, prior_in_flight, event_time,
                                acked_packets, lost_packets);

  Bbr2Mode current_mode = mode_;
  Bbr2ProbeBwMode::CyclePhase current_phase = GetCurrentProbeBwPhase();

  if (!drain_completed_ && current_mode == Bbr2Mode::PROBE_BW) {
    drain_completed_ = true;
  }

  // Track phase transitions
  if (current_mode == Bbr2Mode::PROBE_BW) {
      
      // Entering PROBE_CRUISE
      if (current_phase == Bbr2ProbeBwMode::CyclePhase::PROBE_CRUISE && 
          phase_before != Bbr2ProbeBwMode::CyclePhase::PROBE_CRUISE) {
          cruise_start_time_ = event_time;
          is_fluctuating_ = false;
          fluctuation_done_in_current_cruise_ = false; // Reset for new cruise phase
          fluctuation_start_time_ = QuicTime::Zero();
      }
      
      // Exiting PROBE_CRUISE
      if (current_phase != Bbr2ProbeBwMode::CyclePhase::PROBE_CRUISE && 
          phase_before == Bbr2ProbeBwMode::CyclePhase::PROBE_CRUISE) {
          if (cruise_start_time_ != QuicTime::Zero()) {
              last_cruise_duration_ = event_time - cruise_start_time_;
          }
      }

      // Entering PROBE_UP
      if (current_phase == Bbr2ProbeBwMode::CyclePhase::PROBE_UP && 
          phase_before != Bbr2ProbeBwMode::CyclePhase::PROBE_UP) {
          // Force stop fluctuation if it's still running
          if (is_fluctuating_) {
              trace_duration_ = current_time_ - fluctuation_start_time_;
              is_fluctuating_ = false; 
          }
      }
      
      // NEW_REFILL Logic
      if (current_phase == Bbr2ProbeBwMode::CyclePhase::PROBE_REFILL &&
          phase_before == Bbr2ProbeBwMode::CyclePhase::PROBE_CRUISE) {
        in_new_refill_ = true;
        if (is_fluctuating_) {
             trace_duration_ = current_time_ - fluctuation_start_time_;
             is_fluctuating_ = false;
        }
        UpdateNewRefillState(prior_in_flight);
        model_.set_pacing_gain(GetNewRefillPacingGain());
      }

      // Exit PROBE_UP: Trigger Trace
      if (phase_before == Bbr2ProbeBwMode::CyclePhase::PROBE_UP &&
          current_phase != Bbr2ProbeBwMode::CyclePhase::PROBE_UP) {
          if (up_phase_trace_cb_) {
             double start_s = (trace_start_time_ == QuicTime::Zero()) ? 0.0 : (trace_start_time_ - QuicTime::Zero()).ToMicroseconds() / 1000000.0;
             double dur_ms = trace_duration_.ToMilliseconds();
             
             // Check if exit was due to queueing (inflight > 1.25 * BDP)
             QuicByteCount queue_threshold = CalculateInflightThreshold(1.25f);
             bool exit_due_to_queueing = prior_in_flight > queue_threshold;
             
             // Calculate actual cycles executed
             int cycles = static_cast<int>(std::round(trace_duration_.ToMicroseconds() * trace_freq_hz_ / 1000000.0));

             // Pass 0.0f for pacing_gain as requested (to be hidden/ignored)
             up_phase_trace_cb_(start_s, dur_ms, trace_freq_hz_, exit_due_to_queueing, cycles, 0.0f, model_.MaxBandwidth().ToKBitsPerSecond());
          }
      }
      
      if (in_new_refill_ && current_phase == Bbr2ProbeBwMode::CyclePhase::PROBE_REFILL) {
        QuicByteCount current_inflight = prior_in_flight;
        if (ShouldExitNewRefill(current_inflight)) {
          in_new_refill_ = false;
          new_refill_state_ = NewRefillStateV4::kNotInNewRefill;
        } else {
          model_.set_pacing_gain(GetNewRefillPacingGain());
        }
      }
      
      // If we entered UP, ensure new refill is reset
      if (current_phase == Bbr2ProbeBwMode::CyclePhase::PROBE_UP) {
          in_new_refill_ = false;
          new_refill_state_ = NewRefillStateV4::kNotInNewRefill;
      }
  }

  last_mode_ = current_mode;
  last_probe_bw_phase_ = current_phase;
}

QuicBandwidth FreqCCv4Sender::PacingRate(QuicByteCount bytes_in_flight) const {
  QuicBandwidth base_rate = Bbr2Sender::PacingRate(bytes_in_flight);

  // Safety check: stop fluctuation if not in PROBE_CRUISE
  if (is_fluctuating_) {
      if (GetCurrentProbeBwPhase() != Bbr2ProbeBwMode::CyclePhase::PROBE_CRUISE) {
           auto* mutable_this = const_cast<FreqCCv4Sender*>(this);
           mutable_this->is_fluctuating_ = false;
           return base_rate;
      }
  }

  bool should_start = ShouldOscillate();
  
  if (!is_fluctuating_ && should_start) {
      auto* mutable_this = const_cast<FreqCCv4Sender*>(this);
      
      // Calculate params (Freq and Duration) based on 1/3 cruise rule
      mutable_this->CalculateFluctuationParams();
      
      mutable_this->fluctuation_start_time_ = current_time_;
      mutable_this->is_fluctuating_ = true;
      
      // Save stats for tracing
      mutable_this->trace_start_time_ = mutable_this->fluctuation_start_time_;
      mutable_this->trace_duration_ = mutable_this->fluctuation_duration_;
      mutable_this->trace_freq_hz_ = mutable_this->current_fluctuation_freq_hz_;
      
       QUIC_DVLOG(2) << "FreqCCv4: Starting fluctuation @ " << current_time_
                    << ", freq=" << mutable_this->current_fluctuation_freq_hz_ 
                    << ", duration=" << mutable_this->fluctuation_duration_.ToMilliseconds() << "ms";
  }
  
  if (is_fluctuating_) {
      if (current_time_ > fluctuation_start_time_ + fluctuation_duration_) {
          // Stop fluctuation
          auto* mutable_this = const_cast<FreqCCv4Sender*>(this);
          mutable_this->is_fluctuating_ = false;
          mutable_this->fluctuation_done_in_current_cruise_ = true; // Mark done
           QUIC_DVLOG(2) << "FreqCCv4: Stopping fluctuation @ " << current_time_;
          return base_rate;
      }
      
      // Calculate offset
      int64_t offset_bps = CalculateOscillationOffset(current_time_);
      int64_t base_bps = static_cast<int64_t>(base_rate.ToBitsPerSecond());
      int64_t final_bps = base_bps + offset_bps;

      if (final_bps < 1000) final_bps = 1000;
      return QuicBandwidth::FromBitsPerSecond(static_cast<uint64_t>(final_bps));
  }

  return base_rate;
}

QuicByteCount FreqCCv4Sender::GetCongestionWindow() const {
  QuicByteCount cwnd = cwnd_;
  if (in_new_refill_) {
    QuicByteCount cap = CalculateInflightThreshold(kNewRefillHighThreshold);
    if (cwnd > cap) {
      cwnd = cap;
    }
  }
  return cwnd;
}

int32_t FreqCCv4Sender::GetCurrentBbrModeIndex() const {
  DebugState state = ExportDebugState();
  switch (state.mode) {
    case Bbr2Mode::STARTUP: return 0;
    case Bbr2Mode::DRAIN: return 1;
    case Bbr2Mode::PROBE_BW:
      switch (state.probe_bw.phase) {
        case Bbr2ProbeBwMode::CyclePhase::PROBE_DOWN: return 2;
        case Bbr2ProbeBwMode::CyclePhase::PROBE_CRUISE: return 3;
        case Bbr2ProbeBwMode::CyclePhase::PROBE_REFILL: return 4;
        case Bbr2ProbeBwMode::CyclePhase::PROBE_UP: return 5;
        default: return 2;
      }
    case Bbr2Mode::PROBE_RTT: return 6;
    default: return 0;
  }
}

}  // namespace dqc