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
#include <deque>
#include <vector>
#include <complex>

#include "quic_bbr2_sender.h"
#include "quic_export.h"

namespace dqc {

// Structure to store signal samples for STFT analysis
struct FreqSignalSample {
  QuicTime time;
  QuicBandwidth rate;
};

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

  // Callback for tracing UP phase info (added pacing_gain parameter)
  typedef std::function<void(double start_time, double duration_ms, double freq_hz, bool exit_due_to_queueing, int cycles, float pacing_gain, int32_t bw_kbps)> UpPhaseTraceCallback;
  void SetUpPhaseTraceCallback(UpPhaseTraceCallback cb) { up_phase_trace_cb_ = cb; }

  // Callback for Freq Analysis Trace
  // Parameters: (start_time_s, duration_s, sender_peak_freq_hz, receiver_peak_freq_hz, avg_rate_kbps)
  // Now outputs individual frequency peaks detected in sliding windows, not entire UP phases
  typedef std::function<void(double, double, double, double, int32_t)> FreqAnalysisTraceCallback;
  void SetFreqAnalysisTraceCallback(FreqAnalysisTraceCallback cb) { freq_analysis_trace_cb_ = cb; }

  // FreqCCv3-specific parameters for NEW_REFILL
  static constexpr float kNewRefillHighThreshold = 0.75f;   // Upper threshold for BDP
  static constexpr float kNewRefillLowThreshold = 0.72f;    // Lower threshold for BDP
  static constexpr float kNewRefillDrainingPacingGain = 0.75f;  // Pacing gain when draining
  static constexpr float kNewRefillFillingPacingGain = 1.0f;    // Pacing gain when filling (same as original refill)

  // Single-peak detection tolerance (Hz)
  static constexpr double kPeakFreqTolerance = 10.0;  // 10 Hz tolerance for peak continuity

 private:
  // Calculate oscillation offset based on current time and parameters
  int64_t CalculateOscillationOffset(QuicTime now) const;

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
  double current_oscillation_freq_hz_;      // Frequency used in the current/last UP phase (for tracing)
  double initial_freq_hz_;                  // Initial frequency (for first UP phase)
  FreqCCv3AmplitudeMode amplitude_mode_;    // How to calculate amplitude
  uint64_t fixed_amplitude_bps_;            // Fixed amplitude if mode is kFixed

  // Oscillation state
  bool drain_completed_;                    // True after first Drain phase completes
  QuicTime oscillation_start_time_;         // Time when oscillation started (reset on entering PROBE_UP)
  mutable Bbr2Mode last_mode_;              // Track mode for detecting transitions
  mutable Bbr2ProbeBwMode::CyclePhase last_probe_bw_phase_;  // Track probe_bw phase
  mutable QuicTime current_time_;           // Current time, updated on each packet sent
  mutable QuicTime last_ack_time_;          // Time of last ACK for rate calc

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
  static constexpr double kMinFreqHz = 50.0;       // Minimum frequency limit (1 Hz)
  static constexpr double kMaxFreqHz = 100.0;     // Maximum frequency limit (reduced from 500 Hz)

  // Adaptive pacing gain for PROBE_UP phase (based on RTT multiples)
  float up_pacing_gain_;                          // Current pacing gain for UP phase [1.00, 1.25]
  float current_up_pacing_gain_;                  // Pacing gain used in the current/last UP phase (for tracing)
  static constexpr float kMinUpPacingGain = 1.00f;    // Minimum pacing gain for UP
  static constexpr float kMaxUpPacingGain = 1.25f;    // Maximum pacing gain for UP
  static constexpr float kDefaultUpPacingGain = 1.25f; // Default (same as BBRv2)

  // UP phase duration thresholds in RTT multiples
  static constexpr double kUpDurationVeryLongRttMultiple = 2.5;   // > 2.5 RTT: too long
  static constexpr double kUpDurationLongRttMultiple = 2.0;       // 2.0-2.5 RTT: slightly long
  static constexpr double kUpDurationShortRttMultiple = 1.5;      // 1.5-2.0 RTT: slightly short
  // < 1.5 RTT: too short

  static constexpr float kPacingGainAdjustStep = 0.01f; // Step size for pacing gain adjustment

  // UP phase trace callback
  UpPhaseTraceCallback up_phase_trace_cb_;

  // Freq analysis trace callback
  FreqAnalysisTraceCallback freq_analysis_trace_cb_;

  // Bandwidth estimate before entering UP phase (for early exit handling)
  QuicBandwidth bandwidth_before_up_;

  // Flag to track if UP phase exited early due to loss/ECN
  bool up_phase_exited_early_;

  // Perform STFT analysis on a segment of the signal
  void PerformFreqAnalysis(QuicTime start_time, QuicTime end_time, double threshold_freq_hz = 0.0, double expected_freq_hz = 0.0);

  // Helper to run DFT and find peak frequency
  struct AnalysisResult {
      double peak_freq_hz;
      int32_t avg_rate_kbps;
  };
  AnalysisResult AnalyzeWindow(const std::vector<FreqSignalSample>& samples, TimeDelta window_duration) const;
  
  // STFT Analysis State
  std::deque<FreqSignalSample> signal_history_;          // Receiver rate signal (ACK-based)
  std::deque<FreqSignalSample> sender_rate_history_;     // Sender rate signal (packet-send-based)
  QuicTime last_up_phase_end_time_;         // End time of the last UP phase
  QuicTime last_up_phase_start_time_;       // Start time of the last UP phase
  double last_up_phase_peak_freq_;          // Peak freq found in last UP phase
  TimeDelta last_up_window_size_;           // Window size used for last UP phase
  bool last_up_phase_valid_;                // Whether last UP phase had >= kMinCyclesPerUp
  double sender_max_peak_freq_hz_;          // Max peak freq from sender rate in current UP phase
  QuicTime last_packet_sent_time_;          // Time of last packet sent for sender rate calc

  // Optimized STFT window state
  double optimized_window_sec_;             // Cached optimized window length in seconds
  bool window_optimized_;                   // Whether window has been optimized

  // Helper function to optimize STFT window based on sender rate
  // Returns optimized window size in seconds
  double OptimizeSTFTWindow(const std::vector<FreqSignalSample>& sender_samples,
                            QuicTime start_time,
                            QuicTime end_time,
                            double target_freq_hz);
};

}  // namespace dqc

#endif  // FREQCCV3_SENDER_H_
