// FreqCCv4 - Enhanced Frequency-modulated Congestion Control based on modified BBRv2
// This algorithm modifies BBRv2's behavior:
//   - Replaces PROBE_REFILL with NEW_REFILL phase (same as FreqCCv3)
//   - Moves oscillation to PROBE_CRUISE phase
//   - No adaptive control of UP phase parameters
//   - Random frequency selection for fluctuation
//   - Fluctuation starts 2*min_rtt after PROBE_CRUISE start
//   - Fluctuation lasts max 1/3 of PROBE_CRUISE, at least 5 cycles

#ifndef FREQCCV4_SENDER_H_
#define FREQCCV4_SENDER_H_

#include <cstdint>
#include <string>
#include <functional>

#include "quic_bbr2_sender.h"
#include "quic_export.h"

namespace dqc {

// Amplitude mode for the oscillation (same as FreqCCv2/v3)
enum class FreqCCv4AmplitudeMode {
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

// NEW_REFILL sub-phase states (same as FreqCCv3)
enum class NewRefillStateV4 {
  kNotInNewRefill,    // Not in NEW_REFILL phase
  kDraining,          // Draining: inflight > 0.75*BDP threshold, pacing_gain=0.75
  kFilling,           // Filling: inflight <= 0.70*BDP threshold, pacing_gain=1.0
  kDone,              // Done: neither condition met, exit immediately
};

class QUIC_EXPORT_PRIVATE FreqCCv4Sender final : public Bbr2Sender {
 public:
  FreqCCv4Sender(QuicTime now,
                 const RttStats* rtt_stats,
                 const QuicUnackedPacketMap* unacked_packets,
                 QuicPacketCount initial_cwnd_in_packets,
                 QuicPacketCount max_cwnd_in_packets,
                 Random* random,
                 QuicConnectionStats* stats,
                 bool enable_ecn = false);

  ~FreqCCv4Sender() override = default;

  // Configuration methods for oscillation parameters
  void SetOscillationFrequency(double freq_hz);  // Base Frequency in Hz (though it will be randomized)
  void SetOscillationAmplitude(FreqCCv4AmplitudeMode mode, uint64_t fixed_bps = 0);

  // Override to return FreqCCv4 type
  CongestionControlType GetCongestionControlType() const override {
    return kFreqCCv4;
  }

  // Override OnPacketSent to track current time
  void OnPacketSent(QuicTime sent_time,
                    QuicByteCount bytes_in_flight,
                    QuicPacketNumber packet_number,
                    QuicByteCount bytes,
                    HasRetransmittableData is_retransmittable) override;

  // Override PacingRate to apply oscillation during PROBE_CRUISE
  QuicBandwidth PacingRate(QuicByteCount bytes_in_flight) const override;

  // Override OnCongestionEvent to implement NEW_REFILL logic and track phases
  void OnCongestionEvent(bool rtt_updated,
                         QuicByteCount prior_in_flight,
                         QuicTime event_time,
                         const AckedPacketVector& acked_packets,
                         const LostPacketVector& lost_packets) override;

  // Override GetCongestionWindow
  QuicByteCount GetCongestionWindow() const override;

  // Get current BBR mode as an index for tracing
  int32_t GetCurrentBbrModeIndex() const;

  // Callback for tracing UP phase info (kept for compatibility/logging if needed, though adaptive logic removed)
  typedef std::function<void(double start_time, double duration_ms, double freq_hz, bool exit_due_to_queueing, int cycles, float pacing_gain, int32_t bw_kbps)> UpPhaseTraceCallback;
  void SetUpPhaseTraceCallback(UpPhaseTraceCallback cb) { up_phase_trace_cb_ = cb; }

  // FreqCCv4-specific parameters for NEW_REFILL
  static constexpr float kNewRefillHighThreshold = 0.75f;   // Upper threshold for BDP
  static constexpr float kNewRefillLowThreshold = 0.72f;    // Lower threshold for BDP
  static constexpr float kNewRefillDrainingPacingGain = 0.75f;  // Pacing gain when draining
  static constexpr float kNewRefillFillingPacingGain = 1.0f;    // Pacing gain when filling (same as original refill)

 private:
  // Calculate oscillation offset based on current time and parameters
  int64_t CalculateOscillationOffset(QuicTime now) const;

  // Check if oscillation should be active (during PROBE_CRUISE after delay)
  bool ShouldOscillate() const;

  // Get the amplitude in bps based on current mode and network state
  uint64_t GetCurrentAmplitudeBps() const;

  // Get the current probe_bw phase
  Bbr2ProbeBwMode::CyclePhase GetCurrentProbeBwPhase() const;

  // Calculate the inflight threshold for NEW_REFILL
  QuicByteCount CalculateInflightThreshold(float bdp_factor) const;

  // Check and update NEW_REFILL state based on current inflight
  void UpdateNewRefillState(QuicByteCount bytes_in_flight);

  // Get pacing gain for NEW_REFILL based on current state
  float GetNewRefillPacingGain() const;

  // Check if we should exit NEW_REFILL and enter PROBE_UP
  bool ShouldExitNewRefill(QuicByteCount bytes_in_flight) const;

  // Calculate fluctuation parameters (duration = 1/3 cruise, 5 cycles)
  void CalculateFluctuationParams();

  // Oscillation parameters
  double oscillation_freq_hz_;              // Base/Initial Frequency in Hz
  FreqCCv4AmplitudeMode amplitude_mode_;    // How to calculate amplitude
  uint64_t fixed_amplitude_bps_;            // Fixed amplitude if mode is kFixed

  // Oscillation state for FreqCCv4
  bool drain_completed_;                    // True after first Drain phase completes
  QuicTime current_time_;                   // Current time, updated on each packet sent
  
  // PROBE_CRUISE oscillation state
  QuicTime cruise_start_time_;              // Time when PROBE_CRUISE started
  QuicTime fluctuation_start_time_;         // Time when fluctuation started
  TimeDelta fluctuation_duration_;          // Duration of fluctuation
  TimeDelta last_cruise_duration_;          // Duration of the last PROBE_CRUISE phase
  double current_fluctuation_freq_hz_;      // Frequency currently being used
  double last_fluctuation_freq_hz_;         // Last used frequency (to ensure >3Hz gap)
  bool is_fluctuating_;                     // Whether we are currently fluctuating
  bool fluctuation_done_in_current_cruise_; // Flag to ensure only one fluctuation per cruise
  
  // NEW_REFILL state
  NewRefillStateV4 new_refill_state_;       // Current state within NEW_REFILL
  bool in_new_refill_;                      // True when in NEW_REFILL phase

  // UP phase trace callback
  UpPhaseTraceCallback up_phase_trace_cb_;

  // Stats for trace (persisted across phases until reported)
  double trace_freq_hz_;
  QuicTime trace_start_time_;
  TimeDelta trace_duration_;

  mutable Bbr2Mode last_mode_;              // Track mode for detecting transitions
  mutable Bbr2ProbeBwMode::CyclePhase last_probe_bw_phase_;  // Track probe_bw phase
};

}  // namespace dqc

#endif  // FREQCCV4_SENDER_H_
