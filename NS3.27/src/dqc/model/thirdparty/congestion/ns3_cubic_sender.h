#pragma once

#include "prr_sender.h"
#include "proto_send_algorithm_interface.h"

#include <cstdint>

namespace dqc {

class RttStats;

// DQC transport adapter for the TcpCubic algorithm shipped by ns-3.47.
//
// TcpCubic in upstream ns-3 operates on TcpSocketState.  The experiment in
// this repository uses DQC/QUIC flows, so this class preserves the upstream
// CUBIC state machine and ACK-count arithmetic while adapting its inputs and
// recovery plumbing to SendAlgorithmInterface.
class Ns3CubicSender : public SendAlgorithmInterface {
 public:
  Ns3CubicSender(const ProtoClock* clock,
                 const RttStats* rtt_stats,
                 QuicPacketCount initial_congestion_window,
                 QuicPacketCount max_congestion_window,
                 QuicConnectionStats* stats);
  Ns3CubicSender(const Ns3CubicSender&) = delete;
  Ns3CubicSender& operator=(const Ns3CubicSender&) = delete;
  ~Ns3CubicSender() override;

  void AdjustNetworkParameters(QuicBandwidth bandwidth,
                               TimeDelta rtt,
                               bool allow_cwnd_to_decrease) override;
  void SetNumEmulatedConnections(int num_connections) override;
  void SetInitialCongestionWindowInPackets(
      QuicPacketCount congestion_window) override;
  void OnConnectionMigration() override;
  void OnCongestionEvent(bool rtt_updated,
                         QuicByteCount prior_in_flight,
                         ProtoTime event_time,
                         const AckedPacketVector& acked_packets,
                         const LostPacketVector& lost_packets) override;
  void OnPacketSent(ProtoTime sent_time,
                    QuicByteCount bytes_in_flight,
                    QuicPacketNumber packet_number,
                    QuicByteCount bytes,
                    HasRetransmittableData is_retransmittable) override;
  void OnRetransmissionTimeout(bool packets_retransmitted) override;
  bool CanSend(QuicByteCount bytes_in_flight) override;
  QuicBandwidth PacingRate(QuicByteCount bytes_in_flight) const override;
  QuicBandwidth BandwidthEstimate() const override;
  QuicByteCount GetCongestionWindow() const override;
  QuicByteCount GetSlowStartThreshold() const override;
  CongestionControlType GetCongestionControlType() const override;
  bool InSlowStart() const override;
  bool InRecovery() const override;
  bool ShouldSendProbingPacket() const override;
  std::string GetDebugState() const override;
  void OnApplicationLimited(QuicByteCount bytes_in_flight) override;

 private:
  bool IsCwndLimited(QuicByteCount bytes_in_flight) const;
  void OnPacketAcked(QuicPacketNumber packet_number,
                     QuicByteCount bytes_acked,
                     QuicByteCount prior_in_flight,
                     ProtoTime event_time);
  void OnPacketLost(QuicPacketNumber packet_number,
                    QuicByteCount bytes_lost,
                    QuicByteCount prior_in_flight);
  uint32_t CubicUpdate(uint32_t segments_acked, ProtoTime event_time);
  void UpdateRttAndHystart(ProtoTime event_time);
  void HystartUpdate(ProtoTime event_time, TimeDelta delay);
  void HystartReset(ProtoTime event_time);
  TimeDelta HystartDelayThreshold(TimeDelta delay) const;
  void CubicReset();
  void ResetConnectionState();

  const RttStats* rtt_stats_;
  QuicConnectionStats* stats_;
  PrrSender prr_;

  QuicPacketNumber largest_sent_packet_number_;
  QuicPacketNumber largest_acked_packet_number_;
  QuicPacketNumber largest_sent_at_last_cutback_;
  QuicPacketNumber hystart_end_packet_number_;

  QuicByteCount congestion_window_;
  QuicByteCount min_congestion_window_;
  QuicByteCount max_congestion_window_;
  QuicByteCount slowstart_threshold_;
  QuicByteCount initial_congestion_window_;

  uint64_t congestion_window_count_;
  uint64_t ack_count_;
  uint32_t tcp_congestion_window_;
  uint32_t last_max_congestion_window_;
  uint32_t bic_origin_point_;
  double bic_k_seconds_;
  TimeDelta delay_min_;
  ProtoTime epoch_start_;

  bool hystart_found_;
  ProtoTime hystart_round_start_;
  ProtoTime hystart_last_ack_;
  TimeDelta hystart_current_rtt_;
  uint32_t hystart_sample_count_;
};

}  // namespace dqc
