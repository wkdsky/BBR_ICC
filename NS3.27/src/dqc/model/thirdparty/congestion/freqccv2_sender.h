// FreqCCv2 - Enhanced Frequency-modulated Congestion Control based on modified BBRv2
// This algorithm modifies BBRv2's PROBE_DOWN and PROBE_UP pacing gains:
//   - PROBE_DOWN pacing gain: 0.5 (instead of 0.75)
//   - PROBE_DOWN exit: RTT <= 1.05 * min_RTT OR phase lasted >= 2 * min_RTT
//   - PROBE_UP pacing gain: 1.05 (instead of 1.25)
// And adds periodic oscillation like ICC, where cwnd = rate * min_rtt

#ifndef FREQCCV2_SENDER_H_
#define FREQCCV2_SENDER_H_

#include <cstdint>
#include <string>

#include "quic_bbr2_sender.h"
#include "quic_export.h"

namespace dqc {

// Amplitude mode for the oscillation (same as FreqCC)
enum class FreqCCv2AmplitudeMode {
  kFixed,      // Fixed amplitude in bps
  kMiu2,       // 1/2 of max bandwidth (miu)
  kMiu3,       // 1/3 of max bandwidth
  kMiu4,       // 1/4 of max bandwidth
  kMiu8,       // 1/8 of max bandwidth
  kSR2,        // 1/2 of current sending rate
  kSR3,        // 1/3 of current sending rate
  kSR4,        // 1/4 of current sending rate
  kSR8,        // 1/8 of current sending rate
};

// Oscillation mode (extended from FreqCC with refill_up mode)
enum class FreqCCv2OscillationMode {
  kAfterDrain,   // Start oscillating after first Drain phase, continue in all modes
  kOnlyProbeBW,  // Only oscillate during ProbeBW phase
  kRefillUp,     // Only oscillate during PROBE_REFILL and PROBE_UP phases
};

class QUIC_EXPORT_PRIVATE FreqCCv2Sender final : public Bbr2Sender {
 public:
  FreqCCv2Sender(QuicTime now,
                 const RttStats* rtt_stats,
                 const QuicUnackedPacketMap* unacked_packets,
                 QuicPacketCount initial_cwnd_in_packets,
                 QuicPacketCount max_cwnd_in_packets,
                 Random* random,
                 QuicConnectionStats* stats,
                 bool enable_ecn = false);

  ~FreqCCv2Sender() override = default;

  // Configuration methods for oscillation parameters
  void SetOscillationFrequency(double freq_hz);  // Frequency in Hz
  void SetOscillationAmplitude(FreqCCv2AmplitudeMode mode, uint64_t fixed_bps = 0);
  void SetOscillationMode(FreqCCv2OscillationMode mode);

  // Override to return FreqCCv2 type
  CongestionControlType GetCongestionControlType() const override {
    return kFreqCCv2;
  }

  // Override OnPacketSent to track current time
  void OnPacketSent(QuicTime sent_time,
                    QuicByteCount bytes_in_flight,
                    QuicPacketNumber packet_number,
                    QuicByteCount bytes,
                    HasRetransmittableData is_retransmittable) override;

  // Override PacingRate to apply oscillation
  QuicBandwidth PacingRate(QuicByteCount bytes_in_flight) const override;

  // Override OnCongestionEvent to track mode transitions and apply modified behavior
  void OnCongestionEvent(bool rtt_updated,
                         QuicByteCount prior_in_flight,
                         QuicTime event_time,
                         const AckedPacketVector& acked_packets,
                         const LostPacketVector& lost_packets) override;

  // Override GetCongestionWindow - when oscillating, cwnd = rate * min_rtt (like ICC)
  QuicByteCount GetCongestionWindow() const override;

  // Get current BBR mode as an index for tracing
  // Returns: 0=STARTUP, 1=DRAIN, 2=PROBE_BW_DOWN, 3=PROBE_BW_CRUISE,
  //          4=PROBE_BW_REFILL, 5=PROBE_BW_UP, 6=PROBE_RTT
  int32_t GetCurrentBbrModeIndex() const;

  // FreqCCv2-specific parameters
  static constexpr float kFreqCCv2ProbeDownPacingGain = 0.5f;    // Modified from 0.75
  static constexpr float kFreqCCv2ProbeUpPacingGain = 1.05f;     // Modified from 1.25
  static constexpr float kFreqCCv2RttExitThreshold = 1.05f;      // RTT <= 1.05 * min_RTT to exit PROBE_DOWN
  static constexpr int kFreqCCv2MinProbeDownRounds = 2;          // At least 2 min_RTT duration

 private:
  // Calculate oscillation offset based on current time and parameters
  QuicBandwidth CalculateOscillationOffset(QuicTime now) const;

  // Check if oscillation should be active based on current mode and state
  bool ShouldOscillate() const;

  // Get the amplitude in bps based on current mode and network state
  uint64_t GetCurrentAmplitudeBps() const;

  // Get the current probe_bw phase
  Bbr2ProbeBwMode::CyclePhase GetCurrentProbeBwPhase() const;

  // Check if we should exit PROBE_DOWN based on FreqCCv2 criteria
  bool ShouldExitProbeDown() const;

  // Apply FreqCCv2's modified pacing gain for current phase
  float GetModifiedPacingGainForPhase() const;

  // Oscillation parameters
  double oscillation_freq_hz_;              // Frequency in Hz
  FreqCCv2AmplitudeMode amplitude_mode_;    // How to calculate amplitude
  uint64_t fixed_amplitude_bps_;            // Fixed amplitude if mode is kFixed
  FreqCCv2OscillationMode oscillation_mode_; // When to oscillate

  // Oscillation state
  bool drain_completed_;                    // True after first Drain phase completes
  QuicTime oscillation_start_time_;         // Time when oscillation started
  mutable Bbr2Mode last_mode_;              // Track mode for detecting transitions
  mutable Bbr2ProbeBwMode::CyclePhase last_probe_bw_phase_;  // Track probe_bw phase
  mutable QuicTime current_time_;           // Current time, updated on each packet sent

  // PROBE_DOWN tracking for FreqCCv2's modified exit condition
  QuicTime probe_down_start_time_;          // When PROBE_DOWN phase started
  TimeDelta min_rtt_at_probe_down_start_;   // min_RTT when entering PROBE_DOWN
};

}  // namespace dqc

#endif  // FREQCCV2_SENDER_H_
