// FreqCCv3 - Enhanced Frequency-modulated Congestion Control based on modified BBRv2
// This algorithm modifies BBRv2's behavior:
//   - Replaces PROBE_REFILL with NEW_REFILL phase that adjusts based on inflight vs BDP
//   - Adds periodic oscillation only during PROBE_UP phase (like freqccv2)
//   - NEW_REFILL logic:
//     * If inflight > 0.75*BDP + 2*MSS + MaxAckHeight: pacing_gain=0.75 until inflight <= threshold
//     * If inflight <= 0.70*BDP + 2*MSS + MaxAckHeight: pacing_gain=1.0 (original refill) until inflight >= 0.75*BDP threshold
//     * Otherwise: update parameters and exit immediately

#ifndef FREQCCV3_SENDER_H_
#define FREQCCV3_SENDER_H_

#include <cstdint>
#include <string>
#include <functional>

#include "quic_bbr2_sender.h"
#include "quic_export.h"

namespace dqc {

// Amplitude mode for the oscillation (same as FreqCCv2)
enum class FreqCCv3AmplitudeMode {
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

// NEW_REFILL sub-phase states
enum class NewRefillState {
  kNotInNewRefill,    // Not in NEW_REFILL phase
  kDraining,          // Draining: inflight > 0.75*BDP threshold, pacing_gain=0.75
  kFilling,           // Filling: inflight <= 0.70*BDP threshold, pacing_gain=1.0
  kDone,              // Done: neither condition met, exit immediately
};

class QUIC_EXPORT_PRIVATE FreqCCv3Sender final : public Bbr2Sender {
 public:
  FreqCCv3Sender(QuicTime now,
                 const RttStats* rtt_stats,
                 const QuicUnackedPacketMap* unacked_packets,
                 QuicPacketCount initial_cwnd_in_packets,
                 QuicPacketCount max_cwnd_in_packets,
                 Random* random,
                 QuicConnectionStats* stats,
                 bool enable_ecn = false);

  ~FreqCCv3Sender() override = default;

  // Configuration methods for oscillation parameters
  void SetOscillationFrequency(double freq_hz);  // Frequency in Hz
  void SetOscillationAmplitude(FreqCCv3AmplitudeMode mode, uint64_t fixed_bps = 0);

  // Override to return FreqCCv3 type
  CongestionControlType GetCongestionControlType() const override {
    return kFreqCCv3;
  }

  // Override OnPacketSent to track current time
  void OnPacketSent(QuicTime sent_time,
                    QuicByteCount bytes_in_flight,
                    QuicPacketNumber packet_number,
                    QuicByteCount bytes,
                    HasRetransmittableData is_retransmittable) override;

  // Override PacingRate to apply oscillation during PROBE_UP
  QuicBandwidth PacingRate(QuicByteCount bytes_in_flight) const override;

  // Override OnCongestionEvent to implement NEW_REFILL logic
  void OnCongestionEvent(bool rtt_updated,
                         QuicByteCount prior_in_flight,
                         QuicTime event_time,
                         const AckedPacketVector& acked_packets,
                         const LostPacketVector& lost_packets) override;

  // Override GetCongestionWindow
  QuicByteCount GetCongestionWindow() const override;

  // Get current BBR mode as an index for tracing
  // Returns: 0=STARTUP, 1=DRAIN, 2=PROBE_BW_DOWN, 3=PROBE_BW_CRUISE,
  //          4=PROBE_BW_REFILL (NEW_REFILL), 5=PROBE_BW_UP, 6=PROBE_RTT
  int32_t GetCurrentBbrModeIndex() const;

  // Callback for tracing UP phase info
  typedef std::function<void(double start_time, double duration_ms, double freq_hz, int cycles, int32_t bw_kbps)> UpPhaseTraceCallback;
  void SetUpPhaseTraceCallback(UpPhaseTraceCallback cb) { up_phase_trace_cb_ = cb; }

  // FreqCCv3-specific parameters for NEW_REFILL
  static constexpr float kNewRefillHighThreshold = 0.75f;   // Upper threshold for BDP
  static constexpr float kNewRefillLowThreshold = 0.70f;    // Lower threshold for BDP
  static constexpr float kNewRefillDrainingPacingGain = 0.75f;  // Pacing gain when draining
  static constexpr float kNewRefillFillingPacingGain = 1.0f;    // Pacing gain when filling (same as original refill)

 private:
  // Calculate oscillation offset based on current time and parameters
  QuicBandwidth CalculateOscillationOffset(QuicTime now) const;

  // Check if oscillation should be active (only during PROBE_UP)
  bool ShouldOscillate() const;

  // Get the amplitude in bps based on current mode and network state
  uint64_t GetCurrentAmplitudeBps() const;

  // Get the current probe_bw phase
  Bbr2ProbeBwMode::CyclePhase GetCurrentProbeBwPhase() const;

  // Calculate the inflight threshold for NEW_REFILL
  // threshold = factor * BDP + 2*MSS + MaxAckHeight
  QuicByteCount CalculateInflightThreshold(float bdp_factor) const;

  // Check and update NEW_REFILL state based on current inflight
  void UpdateNewRefillState(QuicByteCount bytes_in_flight);

  // Get pacing gain for NEW_REFILL based on current state
  float GetNewRefillPacingGain() const;

  // Check if we should exit NEW_REFILL and enter PROBE_UP
  bool ShouldExitNewRefill(QuicByteCount bytes_in_flight) const;

  // Oscillation parameters (only used during PROBE_UP)
  double oscillation_freq_hz_;              // Frequency in Hz
  double initial_freq_hz_;                  // Initial frequency (for first UP phase)
  FreqCCv3AmplitudeMode amplitude_mode_;    // How to calculate amplitude
  uint64_t fixed_amplitude_bps_;            // Fixed amplitude if mode is kFixed

  // Oscillation state
  bool drain_completed_;                    // True after first Drain phase completes
  QuicTime oscillation_start_time_;         // Time when oscillation started (reset on entering PROBE_UP)
  mutable Bbr2Mode last_mode_;              // Track mode for detecting transitions
  mutable Bbr2ProbeBwMode::CyclePhase last_probe_bw_phase_;  // Track probe_bw phase
  mutable QuicTime current_time_;           // Current time, updated on each packet sent

  // NEW_REFILL state
  NewRefillState new_refill_state_;         // Current state within NEW_REFILL
  bool in_new_refill_;                      // True when in NEW_REFILL phase

  // Adaptive frequency state
  int up_phase_count_;                      // Count of UP phases completed
  QuicTime up_phase_start_time_;            // Start time of current UP phase
  double last_up_duration_sec_;             // Duration of last UP phase in seconds
  static constexpr int kTargetCyclesPerUp = 3;    // Target number of cycles per UP phase (reduced from 15)
  static constexpr int kMinCyclesPerUp = 2;       // Minimum cycles threshold (reduced from 10)
  static constexpr int kMaxCyclesPerUp = 5;       // Maximum cycles threshold (reduced from 20)
  static constexpr double kMinFreqHz = 1.0;       // Minimum frequency limit (1 Hz)
  static constexpr double kMaxFreqHz = 100.0;     // Maximum frequency limit (reduced from 500 Hz)

  // Adaptive pacing gain for PROBE_UP phase
  float up_pacing_gain_;                          // Current pacing gain for UP phase [1.05, 1.25]
  static constexpr float kMinUpPacingGain = 1.05f;    // Minimum pacing gain for UP
  static constexpr float kMaxUpPacingGain = 1.25f;    // Maximum pacing gain for UP
  static constexpr float kDefaultUpPacingGain = 1.25f; // Default (same as BBRv2)
  static constexpr double kTargetUpDurationSec = 0.5;  // Target UP phase duration in seconds
  static constexpr double kMinUpDurationSec = 0.3;     // Below this, reduce pacing gain
  static constexpr double kMaxUpDurationSec = 0.8;     // Above this, increase pacing gain
  static constexpr float kPacingGainAdjustStep = 0.02f; // Step size for pacing gain adjustment

  // UP phase trace callback
  UpPhaseTraceCallback up_phase_trace_cb_;
};

}  // namespace dqc

#endif  // FREQCCV3_SENDER_H_
