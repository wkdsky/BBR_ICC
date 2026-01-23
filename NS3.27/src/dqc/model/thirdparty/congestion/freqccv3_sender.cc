// FreqCCv3 - Enhanced Frequency-modulated Congestion Control based on modified BBRv2
// This algorithm modifies BBRv2's behavior with NEW_REFILL and oscillation during PROBE_UP

#include "freqccv3_sender.h"

#include <cmath>
#include <algorithm>
#include <numeric>
#include <vector>
#include <fftw3.h>

#include "quic_logging.h"
#include "quic_bbr2_probe_bw.h"

namespace dqc {

namespace {
// Default oscillation parameters
const double kDefaultOscillationFreqHz = 1.0;  // 1 Hz default

// Helper for Linear Detrending
void LinearDetrend(std::vector<double>& data) {
    size_t n = data.size();
    if (n < 2) return;
    double sum_x = 0.0, sum_y = 0.0, sum_xy = 0.0, sum_xx = 0.0;
    for (size_t i = 0; i < n; ++i) {
        sum_x += i;
        sum_y += data[i];
        sum_xy += i * data[i];
        sum_xx += i * i;
    }
    double slope = (n * sum_xy - sum_x * sum_y) / (n * sum_xx - sum_x * sum_x);
    double intercept = (sum_y - slope * sum_x) / n;
    for (size_t i = 0; i < n; ++i) {
        data[i] -= (slope * i + intercept);
    }
}
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
      new_refill_state_(NewRefillState::kNotInNewRefill),
      in_new_refill_(false),
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
      last_up_phase_end_time_(QuicTime::Zero()),
      last_up_phase_start_time_(QuicTime::Zero()),
      last_up_phase_peak_freq_(0.0),
      last_up_window_size_(TimeDelta::Zero()),
      last_up_phase_valid_(false),
      sender_max_peak_freq_hz_(0.0),
      last_ack_time_(QuicTime::Zero()),
      last_packet_sent_time_(QuicTime::Zero()),
      // Initialize optimized window state
      optimized_window_sec_(0.0),
      window_optimized_(false) {
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
    // Inflight < 0.70*BDP + 2*MSS + MaxAckHeight: need to fill
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

  // STFT: Calculate Instantaneous Rate and Push to Buffer
  if (last_ack_time_ != QuicTime::Zero() && event_time > last_ack_time_) {
      QuicByteCount acked_bytes = 0;
      for (const auto& ack : acked_packets) {
          acked_bytes += ack.bytes_acked;
      }
      if (acked_bytes > 0) {
          TimeDelta delta = event_time - last_ack_time_;
          QuicBandwidth rate = QuicBandwidth::FromBytesAndTimeDelta(acked_bytes, delta);
          signal_history_.push_back({event_time, rate});
          
          // Safety cap to prevent memory explosion if not cleared
          while (signal_history_.size() > 20000) signal_history_.pop_front();
      }
  }
  last_ack_time_ = event_time;

  // Get phase before parent update
  Bbr2ProbeBwMode::CyclePhase phase_before = GetCurrentProbeBwPhase();

  // Check if we're in PROBE_UP and there are losses or ECN marks
  bool in_probe_up_before = (phase_before == Bbr2ProbeBwMode::CyclePhase::PROBE_UP);
  bool has_loss_or_ecn = (!lost_packets.empty()) || (GetBytesEcnInRounds() > 0);

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
      // STFT: START of NEW UP Phase = END of Previous Interval
      // Analyze Previous Interval (from last_up_phase_end_time_ to now)
      if (last_up_phase_valid_ && last_up_phase_end_time_ != QuicTime::Zero()) {
          // Analyze Interval with threshold
          // IMPORTANT: Use sender_max_peak_freq_hz_ from the previous UP phase before resetting it!
          PerformFreqAnalysis(last_up_phase_end_time_, event_time, last_up_phase_peak_freq_ * 2.0 / 3.0, 0.0);

          // Clear history older than event_time (Start of this UP)
          while (!signal_history_.empty() && signal_history_.front().time < event_time) {
              signal_history_.pop_front();
          }
      } else {
          // Invalid or first run, clear history up to now
          while (!signal_history_.empty() && signal_history_.front().time < event_time) {
              signal_history_.pop_front();
          }
          if (signal_history_.empty()) signal_history_.clear();
      }

      // DO NOT clear sender_rate_history here! We need it for STFT analysis when leaving UP phase
      // Clearing will be done after STFT analysis in the leaving_probe_up section

      // Reset UP state
      oscillation_start_time_ = event_time;
      up_phase_start_time_ = event_time;
      last_up_phase_start_time_ = event_time;
      last_up_phase_peak_freq_ = 0.0;
      last_up_phase_valid_ = false;
      sender_max_peak_freq_hz_ = 0.0;  // NOW reset sender max peak freq AFTER Interval analysis

      // Save bandwidth estimate before entering UP phase (for early exit handling)
      bandwidth_before_up_ = model_.MaxBandwidth();
      up_phase_exited_early_ = false;

      QUIC_DVLOG(2) << "FreqCCv3: Entering PROBE_UP, starting oscillation @ " << event_time
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
    bool need_cleanup = !in_probe_up && (oscillation_start_time_ != QuicTime::Zero());

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
                // ===== OPTIMIZED WINDOW DETERMINATION =====
                // If not yet optimized, run iterative window optimization
                double win_sec = 0.0;
                if (!window_optimized_) {
                    // std::cout << "FreqCCv3: First UP phase - performing window optimization" << std::endl;
                    win_sec = OptimizeSTFTWindow(sender_samples,
                                                 last_up_phase_start_time_,
                                                 last_up_phase_end_time_,
                                                 current_oscillation_freq_hz_);
                    optimized_window_sec_ = win_sec;
                    window_optimized_ = true;
                    // std::cout << "FreqCCv3: Window optimization complete. Using optimized_window_sec_="
                    //           << optimized_window_sec_ << "s for all future STFT analysis" << std::endl;
                } else {
                    // Use cached optimized window
                    win_sec = optimized_window_sec_;
                    // std::cout << "FreqCCv3: Using cached optimized window=" << win_sec << "s" << std::endl;
                }

                TimeDelta sender_window_size = TimeDelta::FromMicroseconds(static_cast<int64_t>(win_sec * 1000000.0));
                TimeDelta sender_duration = last_up_phase_end_time_ - last_up_phase_start_time_;
                if (sender_window_size > sender_duration) sender_window_size = sender_duration;

                // Sliding window analysis with 85% overlap
                double overlap = 0.85;
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
                        AnalysisResult result = AnalyzeWindow(win_samples, sender_window_size);

                        // NO max_valid_freq constraint - accept all positive frequencies
                        if (result.peak_freq_hz > 0.0) {
                            sender_peak_freqs.push_back(result.peak_freq_hz);
                            // std::cout << "FreqCCv3: Sender window #" << window_count
                            //           << " ACCEPTED: peak_freq=" << result.peak_freq_hz << " Hz" << std::endl;
                        }
                    }

                    // Advance window
                    win_start = win_start + step_size;
                }

                // std::cout << "FreqCCv3: Processed " << window_count
                //           << " sender windows, collected " << sender_peak_freqs.size() << " valid peaks" << std::endl;

                // Calculate max peak freq from sender rate
                if (!sender_peak_freqs.empty()) {
                    sender_max_peak_freq_hz_ = *std::max_element(sender_peak_freqs.begin(), sender_peak_freqs.end());
                    // std::cout << "FreqCCv3: Sender rate STFT SUCCESSFUL! max_peak_freq="
                    //           << sender_max_peak_freq_hz_ << "Hz from "
                    //           << sender_peak_freqs.size() << " windows" << std::endl;
                } else {
                    sender_max_peak_freq_hz_ = 0.0;
                    // std::cout << "FreqCCv3: Sender rate STFT FAILED: yielded no valid peaks" << std::endl;
                }
            } else {
                sender_max_peak_freq_hz_ = 0.0;
                // std::cout << "FreqCCv3: No sender rate samples for STFT analysis (empty sender_samples)" << std::endl;
            }

            // ===== RECEIVER RATE STFT ANALYSIS =====
            // Analyze THIS UP Phase (receiver rate)
            // From last_up_phase_start_time_ to last_up_phase_end_time_
            PerformFreqAnalysis(last_up_phase_start_time_, last_up_phase_end_time_, 0.0, current_oscillation_freq_hz_);

            // Clear sender rate history after STFT analysis
            while (!sender_rate_history_.empty() && sender_rate_history_.front().time < event_time) {
                sender_rate_history_.pop_front();
            }
        } else {
            last_up_phase_valid_ = false;
            sender_max_peak_freq_hz_ = 0.0;
            // Abandon cache for this UP phase
            // We clear history up to now to ensure we don't use it
             while (!signal_history_.empty() && signal_history_.front().time < event_time) {
                signal_history_.pop_front();
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

// Iterative window optimization for STFT
// Finds optimal window length where detected sender peak frequency matches target ±5%
double FreqCCv3Sender::OptimizeSTFTWindow(const std::vector<FreqSignalSample>& sender_samples,
                                          QuicTime start_time,
                                          QuicTime end_time,
                                          double target_freq_hz) {
    if (sender_samples.empty() || target_freq_hz <= 0.0) {
        // std::cout << "FreqCCv3: OptimizeSTFTWindow - invalid inputs (empty samples or target_freq <= 0)" << std::endl;
        return 0.1; // Return default minimum window
    }

    // Calculate initial window based on cycles and RTT
    double avg_cycles = (static_cast<double>(kMinCyclesPerUp) + static_cast<double>(kMaxCyclesPerUp)) / 2.0;
    double win_sec = avg_cycles / target_freq_hz;

    TimeDelta min_rtt = model_.MinRtt();
    if (!min_rtt.IsZero()) {
        double min_rtt_sec = static_cast<double>(min_rtt.ToMicroseconds()) / 1000000.0;
        if (min_rtt_sec > win_sec) {
            win_sec = min_rtt_sec;
        }
    }

    // Clamp to minimum window
    double min_win_sec = 0.1; // 100ms minimum
    if (win_sec < min_win_sec) {
        win_sec = min_win_sec;
    }

    // std::cout << "FreqCCv3: Window optimization starting with initial win_sec=" << win_sec
    //           << "s for target_freq=" << target_freq_hz << "Hz" << std::endl;

    // Iterative search parameters
    double best_window = win_sec;
    double best_error = 1.0;  // Start with worst possible error (100%)
    double current_window = win_sec;
    double step_factor = 0.1;  // 10% step size per iteration
    const double tolerance = 0.05;  // ±5% tolerance
    const int max_iterations = 20;
    int iteration = 0;

    TimeDelta total_duration = end_time - start_time;
    double max_window_sec = static_cast<double>(total_duration.ToMicroseconds()) / 1000000.0;

    // Iterative refinement loop
    while (iteration < max_iterations && best_error > tolerance) {
        iteration++;

        // Test current window
        TimeDelta test_window_duration = TimeDelta::FromMicroseconds(
            static_cast<int64_t>(current_window * 1000000.0));

        if (test_window_duration > total_duration) {
            test_window_duration = total_duration;
            current_window = static_cast<double>(total_duration.ToMicroseconds()) / 1000000.0;
        }

        // Run single FFT analysis on the full UP phase with current window
        std::vector<FreqSignalSample> win_samples;
        for (const auto& s : sender_samples) {
            if (s.time >= start_time && s.time <= (start_time + test_window_duration)) {
                win_samples.push_back(s);
            }
        }

        if (win_samples.empty()) {
            // std::cout << "FreqCCv3: Iteration " << iteration << " - no samples in window, skipping" << std::endl;
            step_factor *= 0.5;  // Reduce step size
            current_window = best_window;
            continue;
        }

        AnalysisResult result = AnalyzeWindow(win_samples, test_window_duration);
        double detected_freq = result.peak_freq_hz;

        if (detected_freq <= 0.0) {
            // std::cout << "FreqCCv3: Iteration " << iteration << " - no valid FFT peak detected" << std::endl;
            step_factor *= 0.5;
            current_window = best_window;
            continue;
        }

        // Calculate error relative to target
        double error = std::abs(detected_freq - target_freq_hz) / target_freq_hz;

        // std::cout << "FreqCCv3: Iteration " << iteration << " - window=" << current_window
        //           << "s, detected_freq=" << detected_freq << "Hz, target_freq=" << target_freq_hz
        //           << "Hz, error=" << (error * 100.0) << "%" << std::endl;

        // Update best if this is better
        if (error < best_error) {
            best_error = error;
            best_window = current_window;
            // std::cout << "FreqCCv3:   -> NEW BEST! error=" << (error * 100.0) << "%" << std::endl;

            // If good enough, we can stop
            if (error <= tolerance) {
                // std::cout << "FreqCCv3: Converged! Optimal window=" << best_window
                //           << "s with error=" << (error * 100.0) << "%" << std::endl;
                break;
            }
        } else {
            // Error increased, reduce step size
            step_factor *= 0.5;
        }

        // Adjust window for next iteration
        if (detected_freq > target_freq_hz) {
            // Detected freq too high -> need shorter window (higher frequency resolution)
            current_window = best_window * (1.0 - step_factor);
        } else {
            // Detected freq too low -> need longer window (lower frequency resolution)
            current_window = best_window * (1.0 + step_factor);
        }

        // Clamp window bounds
        current_window = std::max(min_win_sec, current_window);
        current_window = std::min(max_window_sec, current_window);
    }

    // std::cout << "FreqCCv3: Window optimization complete after " << iteration
    //           << " iterations. Best window=" << best_window << "s with error="
    //           << (best_error * 100.0) << "%" << std::endl;

    return best_window;
}

// Helper to perform DFT on a set of uniform samples
// Returns (Peak Frequency Hz, Avg Rate kbps)
FreqCCv3Sender::AnalysisResult FreqCCv3Sender::AnalyzeWindow(const std::vector<FreqSignalSample>& samples, TimeDelta window_duration) const {
    if (samples.empty()) return {0.0, 0};

    // 1. Calculate Average Rate
    double sum_rate = 0;
    for (const auto& s : samples) {
        sum_rate += s.rate.ToKBitsPerSecond();
    }
    int32_t avg_rate = static_cast<int32_t>(sum_rate / samples.size());

    // 2. Resample to uniform grid for FFT
    // Sample rate Fs=500Hz (2ms interval)
    double step_s = 0.002;
    double duration_s = static_cast<double>(window_duration.ToMicroseconds()) / 1000000.0;
    int N = static_cast<int>(duration_s / step_s);

    QUIC_DVLOG(3) << "FreqCCv3: AnalyzeWindow: samples=" << samples.size()
                  << ", duration=" << duration_s << "s, N=" << N << ", avg_rate=" << avg_rate << " kbps";

    if (N < 8) {
        QUIC_DVLOG(3) << "FreqCCv3: AnalyzeWindow: N=" << N << " < 8, too small for FFT";
        return {0.0, avg_rate}; // Too small for meaningful analysis
    }

    // Allocate buffers for FFTW
    double *in = (double*) fftw_malloc(sizeof(double) * N);
    fftw_complex *out = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * (N/2 + 1));
    
    // Linear interpolation to uniform grid
    double t_start = (samples.front().time - QuicTime::Zero()).ToMicroseconds() / 1000000.0;
    size_t sample_idx = 0;
    
    for (int i = 0; i < N; ++i) {
        double t_target = t_start + i * step_s;
        
        while (sample_idx < samples.size() - 1) {
            double t_next = (samples[sample_idx+1].time - QuicTime::Zero()).ToMicroseconds() / 1000000.0;
            if (t_next >= t_target) break;
            sample_idx++;
        }
        
        double val;
        if (sample_idx >= samples.size() - 1) {
            val = static_cast<double>(samples.back().rate.ToKBitsPerSecond());
        } else {
            double t1 = (samples[sample_idx].time - QuicTime::Zero()).ToMicroseconds() / 1000000.0;
            double t2 = (samples[sample_idx+1].time - QuicTime::Zero()).ToMicroseconds() / 1000000.0;
            double r1 = static_cast<double>(samples[sample_idx].rate.ToKBitsPerSecond());
            double r2 = static_cast<double>(samples[sample_idx+1].rate.ToKBitsPerSecond());
            
            if (std::abs(t2 - t1) < 1e-6) {
                val = r1;
            } else {
                double fraction = (t_target - t1) / (t2 - t1);
                val = r1 + fraction * (r2 - r1);
            }
        }
        in[i] = val;
    }

    // 3. System ID Preprocessing: Linear Detrending
    // Required to eliminate low frequency drift caused by ACK sampling
    std::vector<double> in_vec(in, in + N);
    LinearDetrend(in_vec);
    for (int i=0; i<N; ++i) in[i] = in_vec[i];

    // 4. Windowing (Hann)
    for (int i = 0; i < N; ++i) {
        double hann = 0.5 * (1.0 - std::cos(2.0 * M_PI * i / (N - 1)));
        in[i] *= hann;
    }

    // 5. FFT
    fftw_plan p = fftw_plan_dft_r2c_1d(N, in, out, FFTW_ESTIMATE);
    fftw_execute(p);

    // 6. Peak Detection with Physical Constraints and Sub-bin Estimation
    double Fs = 1.0 / step_s;
    double max_mag = -1.0;
    double peak_freq = 0.0;
    int peak_k = 0;

    // Physical constraints
    double T_total = duration_s;
    double min_freq_bound = 1.0 / T_total; // Ignore below 1/TotalTime
    
    // Find min and max bin
    int k_min = static_cast<int>(min_freq_bound * N / Fs);
    if (k_min < 1) k_min = 1;
    int k_max = N / 2; // Nyquist

    // Search for max magnitude
    for (int k = k_min; k < k_max; ++k) {
        // Power Spectrum Normalization: |X[k]|^2 / N
        double mag = (out[k][0]*out[k][0] + out[k][1]*out[k][1]) / N;
        if (mag > max_mag) {
            max_mag = mag;
            peak_k = k;
        }
    }

    // Sub-bin Parabolic Interpolation
    if (peak_k > k_min && peak_k < k_max) {
         double alpha = (out[peak_k-1][0]*out[peak_k-1][0] + out[peak_k-1][1]*out[peak_k-1][1]) / N;
         double beta = max_mag;
         double gamma = (out[peak_k+1][0]*out[peak_k+1][0] + out[peak_k+1][1]*out[peak_k+1][1]) / N;

         double delta = 0.5 * (alpha - gamma) / (alpha - 2.0 * beta + gamma);
         peak_freq = (peak_k + delta) * Fs / N;
    } else {
         peak_freq = peak_k * Fs / N;
    }

    QUIC_DVLOG(3) << "FreqCCv3: AnalyzeWindow FFT result: peak_k=" << peak_k
                  << ", peak_freq=" << peak_freq << " Hz, max_mag=" << max_mag
                  << ", k_range=[" << k_min << ", " << k_max << "]";

    // Check against noise floor (simple relative threshold)
    // If the peak is not significant enough (e.g., compared to average energy), it might be noise
    // For now, we return the found peak, but PerformFreqAnalysis can filter based on consistency.

    fftw_destroy_plan(p);
    fftw_free(in);
    fftw_free(out);

    return {peak_freq, avg_rate};
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

    // 1. Use Optimized Window if available, otherwise compute adaptively
    if (window_optimized_) {
        // Use the optimized window determined from sender rate analysis
        window_size = TimeDelta::FromMicroseconds(static_cast<int64_t>(optimized_window_sec_ * 1000000.0));
        std::cout << "FreqCCv3: PerformFreqAnalysis using optimized window="
                  << optimized_window_sec_ << "s (" << window_size.ToMilliseconds() << "ms)" << std::endl;
    } else if (expected_freq_hz > 0.0) {
        // Fallback: Adaptive Window Logic (should not happen after first UP phase)
        double avg_cycles = (static_cast<double>(kMinCyclesPerUp) + static_cast<double>(kMaxCyclesPerUp)) / 2.0;
        double win_sec = avg_cycles / expected_freq_hz;

        // Get RTTmin in seconds
        TimeDelta min_rtt = model_.MinRtt();
        if (!min_rtt.IsZero()) {
            double min_rtt_sec = static_cast<double>(min_rtt.ToMicroseconds()) / 1000000.0;
            if (min_rtt_sec > win_sec) {
                win_sec = min_rtt_sec;
            }
        }

        // Minimum window protection
        double min_win_sec = 0.1;
        if (win_sec < min_win_sec) {
            win_sec = min_win_sec;
        }

        window_size = TimeDelta::FromMicroseconds(static_cast<int64_t>(win_sec * 1000000.0));

        std::cout << "FreqCCv3: PerformFreqAnalysis fallback window=" << win_sec
                  << "s (expected_freq=" << expected_freq_hz << "Hz)" << std::endl;
    } else {
        // Interval Phase: Use stored window size from last UP phase or default
        if (last_up_window_size_ > TimeDelta::Zero()) {
            window_size = last_up_window_size_;
        } else {
            TimeDelta min_rtt = model_.MinRtt();
            window_size = (!min_rtt.IsZero()) ? min_rtt : duration;
        }
        if (window_size < TimeDelta::FromMilliseconds(50)) window_size = TimeDelta::FromMilliseconds(50);
    }

    // Constraint: Cannot exceed available data
    if (window_size > duration) window_size = duration;

    // Store for future use
    last_up_window_size_ = window_size;

    // 2. Sliding Window Analysis with Trajectory Tracking
    // Overlap 85% for smooth trajectory
    double overlap = 0.85;
    TimeDelta step_size = TimeDelta::FromMicroseconds(static_cast<int64_t>(window_size.ToMicroseconds() * (1.0 - overlap)));
    if (step_size < TimeDelta::FromMilliseconds(1)) step_size = TimeDelta::FromMilliseconds(1);

    // Collect time-frequency pairs
    std::vector<std::pair<double, double>> time_freq_pairs;  // (time_s, freq_hz)
    std::vector<int32_t> avg_rates;  // corresponding avg rates
    std::vector<double> detected_freqs;
    QuicTime win_start = start_time;

    while (win_start + window_size <= end_time) {
        QuicTime win_end = win_start + window_size;

        // Extract window samples
        std::vector<FreqSignalSample> win_samples;
        for (const auto& s : range_samples) {
            if (s.time >= win_start && s.time <= win_end) {
                win_samples.push_back(s);
            }
        }

        AnalysisResult result = AnalyzeWindow(win_samples, window_size);

        // NO max_valid_freq constraint - removed as per user request
        // Only apply threshold check if specified
        bool valid = true;
        if (threshold_freq_hz > 0.0 && result.peak_freq_hz < threshold_freq_hz) {
            valid = false;
        }

        if (valid && result.peak_freq_hz > 0.0) {
            detected_freqs.push_back(result.peak_freq_hz);

            // Store time-frequency pair (use window start time)
            double time_s = (win_start - QuicTime::Zero()).ToMicroseconds() / 1000000.0;
            time_freq_pairs.push_back({time_s, result.peak_freq_hz});
            avg_rates.push_back(result.avg_rate_kbps);
        }

        // Advance
        win_start = win_start + step_size;
    }

    // 3. Extract Single Peaks (similar to Python logic)
    // Only output peaks with duration > 0 (at least 2 consecutive windows)
    if (freq_analysis_trace_cb_ && !time_freq_pairs.empty()) {
        size_t start_idx = 0;
        for (size_t i = 1; i < time_freq_pairs.size(); ++i) {
            double freq_diff = std::abs(time_freq_pairs[i].second - time_freq_pairs[i-1].second);

            // If frequency jump exceeds tolerance, this is a new peak
            if (freq_diff > kPeakFreqTolerance) {
                // Output the previous peak only if it has duration > 0 (at least 2 windows)
                if (i - start_idx >= 2) {
                    double peak_start_time = time_freq_pairs[start_idx].first;
                    double peak_end_time = time_freq_pairs[i-1].first;
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

                    // Output: (start_time_s, duration_s, sender_peak_freq_hz, receiver_peak_freq_hz, avg_rate_kbps)
                    freq_analysis_trace_cb_(peak_start_time, peak_duration, sender_max_peak_freq_hz_, max_freq, avg_rate);
                }

                // Start new peak
                start_idx = i;
            }
        }

        // Output the last peak only if it has duration > 0 (at least 2 windows)
        size_t last_peak_size = time_freq_pairs.size() - start_idx;
        if (last_peak_size >= 2) {
            double peak_start_time = time_freq_pairs[start_idx].first;
            double peak_end_time = time_freq_pairs.back().first;
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

            freq_analysis_trace_cb_(peak_start_time, peak_duration, sender_max_peak_freq_hz_, max_freq, avg_rate);
        }
    }

    // 4. Trajectory Stability Analysis for Controller Feedback
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
}  // namespace dqc