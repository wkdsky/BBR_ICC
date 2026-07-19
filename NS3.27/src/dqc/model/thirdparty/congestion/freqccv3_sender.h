#ifndef FREQCCV3_SENDER_H_
#define FREQCCV3_SENDER_H_

#include <functional>

#include "fbbr_sender.h"

namespace dqc {

// FreqCCv3 now carries the legacy pure FBBR algorithm:
// fixed-frequency triangle-wave CRUISE probing plus STFT analysis.
class QUIC_EXPORT_PRIVATE FreqCCv3Sender : public FBBRSender {
 public:
  FreqCCv3Sender(QuicTime now,
                 const RttStats* rtt_stats,
                 const QuicUnackedPacketMap* unacked_packets,
                 QuicPacketCount initial_cwnd_in_packets,
                 QuicPacketCount max_cwnd_in_packets,
                 Random* random,
                 QuicConnectionStats* stats,
                 bool enable_ecn = false)
      : FBBRSender(now,
                       rtt_stats,
                       unacked_packets,
                       initial_cwnd_in_packets,
                       max_cwnd_in_packets,
                       random,
                       stats,
                       enable_ecn,
                       false,
                       false,
                       kFreqCCv3) {}

  ~FreqCCv3Sender() override = default;

  CongestionControlType GetCongestionControlType() const override {
    return kFreqCCv3;
  }

  void SetIntervalWindowMultiplier(double) {}
  void SetMinProbeUpDurationRttMultiplier(double) {}

  using UpPhaseTraceCallback = std::function<void(double,
                                                  double,
                                                  double,
                                                  bool,
                                                  int,
                                                  float,
                                                  int32_t)>;
  void SetUpPhaseTraceCallback(UpPhaseTraceCallback) {}

  using FreqAnalysisTraceCallback =
      std::function<void(double, double, double, double, int32_t)>;
  void SetFreqAnalysisTraceCallback(FreqAnalysisTraceCallback) {}

  using RttFreqAnalysisTraceCallback =
      std::function<void(double, double, double, double, double)>;
  void SetRttFreqAnalysisTraceCallback(RttFreqAnalysisTraceCallback) {}
};

}  // namespace dqc

#endif  // FREQCCV3_SENDER_H_
