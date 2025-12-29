// FreqCC - Frequency-modulated Congestion Control based on BBRv2
// This algorithm adds periodic oscillation to BBRv2's pacing rate

#ifndef FREQCC_SENDER_H_
#define FREQCC_SENDER_H_

#include <cstdint>
#include <string>

#include "quic_bbr2_sender.h"
#include "quic_export.h"

namespace dqc {

// Amplitude mode for the oscillation
enum class FreqAmplitudeMode {
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

// Oscillation mode
enum class FreqOscillationMode {
  kAfterDrain,   // Start oscillating after first Drain phase, continue in all modes
  kOnlyProbeBW,  // Only oscillate during ProbeBW phase
};

class QUIC_EXPORT_PRIVATE FreqccSender final : public Bbr2Sender {
 public:
  FreqccSender(QuicTime now,
               const RttStats* rtt_stats,
               const QuicUnackedPacketMap* unacked_packets,
               QuicPacketCount initial_cwnd_in_packets,
               QuicPacketCount max_cwnd_in_packets,
               Random* random,
               QuicConnectionStats* stats,
               bool enable_ecn = false);

  ~FreqccSender() override = default;

  // Configuration methods for oscillation parameters
  void SetOscillationFrequency(double freq_hz);  // Frequency in Hz
  void SetOscillationAmplitude(FreqAmplitudeMode mode, uint64_t fixed_bps = 0);
  void SetOscillationMode(FreqOscillationMode mode);

  // Override to return FreqCC type
  CongestionControlType GetCongestionControlType() const override {
    return kFreqCC;
  }

  // Override PacingRate to apply oscillation
  QuicBandwidth PacingRate(QuicByteCount bytes_in_flight) const override;

  // Override OnCongestionEvent to track mode transitions
  void OnCongestionEvent(bool rtt_updated,
                         QuicByteCount prior_in_flight,
                         QuicTime event_time,
                         const AckedPacketVector& acked_packets,
                         const LostPacketVector& lost_packets) override;

 private:
  // Calculate oscillation offset based on current time and parameters
  QuicBandwidth CalculateOscillationOffset(QuicTime now) const;

  // Check if oscillation should be active based on current mode and state
  bool ShouldOscillate() const;

  // Get the amplitude in bps based on current mode and network state
  uint64_t GetCurrentAmplitudeBps() const;

  // Oscillation parameters
  double oscillation_freq_hz_;           // Frequency in Hz
  FreqAmplitudeMode amplitude_mode_;     // How to calculate amplitude
  uint64_t fixed_amplitude_bps_;         // Fixed amplitude if mode is kFixed
  FreqOscillationMode oscillation_mode_; // When to oscillate

  // Oscillation state
  bool drain_completed_;                 // True after first Drain phase completes
  QuicTime oscillation_start_time_;      // Time when oscillation started
  mutable Bbr2Mode last_mode_;           // Track mode for detecting transitions
};

}  // namespace dqc

#endif  // FREQCC_SENDER_H_
