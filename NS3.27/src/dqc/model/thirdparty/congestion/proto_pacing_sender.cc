#include "proto_pacing_sender.h"
#include "logging.h"
#include "flag_impl.h"
#include "flag_util_impl.h"
namespace dqc{
namespace {
// Configured maximum size of the burst coming out of quiescence.  The burst
// is never larger than the current CWND in packets.
static const uint32_t kInitialUnpacedBurst = 10;

}  // namespace

PacingSender::PacingSender()
    : sender_(nullptr),
      max_pacing_rate_(QuicBandwidth::Zero()),
      burst_tokens_(kInitialUnpacedBurst),
      ideal_next_packet_send_time_(ProtoTime::Zero()),
      initial_burst_size_(kInitialUnpacedBurst),
      lumpy_tokens_(0),
      alarm_granularity_(TimeDelta::FromMilliseconds(1)),
      pacing_limited_(false) {
  if (GetQuicReloadableFlag(quic_donot_reset_ideal_next_packet_send_time)) {
    QUIC_RELOADABLE_FLAG_COUNT(quic_donot_reset_ideal_next_packet_send_time);
  }
}

PacingSender::~PacingSender() {}

void PacingSender::set_sender(SendAlgorithmInterface* sender) {
  DCHECK(sender != nullptr);
  sender_ = sender;
}
void PacingSender::OnOneWayDelaySample(ProtoTime event_time,QuicPacketNumber seq,ProtoTime sent_time,ProtoTime recv_time){
    sender_->OnOneWayDelaySample(event_time,seq,sent_time,recv_time);
}
void PacingSender::OnCongestionEvent(bool rtt_updated,
                                     QuicByteCount bytes_in_flight,
                                     ProtoTime event_time,
                                     const AckedPacketVector& acked_packets,
                                     const LostPacketVector& lost_packets) {
  DCHECK(sender_ != nullptr);
  if (!lost_packets.empty()) {
    // Clear any burst tokens when entering recovery.
    burst_tokens_ = 0;
  }
  sender_->OnCongestionEvent(rtt_updated, bytes_in_flight, event_time,
                             acked_packets, lost_packets);
}

void PacingSender::OnPacketSent(
    ProtoTime sent_time,
    QuicByteCount bytes_in_flight,
    QuicPacketNumber packet_number,
    QuicByteCount bytes,
    HasRetransmittableData has_retransmittable_data) {
  DCHECK(sender_ != nullptr);
  last_debug_state_ = DebugState();
  last_debug_state_.actual_emission_time_us = sent_time.ToDebuggingValue();
  last_debug_state_.ideal_send_time_before_us =
      ideal_next_packet_send_time_.ToDebuggingValue();
  if (last_debug_state_.ideal_send_time_before_us > 0) {
    last_debug_state_.emission_lateness_us = std::max<int64_t>(
        0, last_debug_state_.actual_emission_time_us -
               last_debug_state_.ideal_send_time_before_us);
  }
  sender_->OnPacketSent(sent_time, bytes_in_flight, packet_number, bytes,
                        has_retransmittable_data);
  if (has_retransmittable_data != HAS_RETRANSMITTABLE_DATA) {
    return;
  }
  // If in recovery, the connection is not coming out of quiescence.
  if (bytes_in_flight == 0 && !sender_->InRecovery()) {
    // Add more burst tokens anytime the connection is leaving quiescence, but
    // limit it to the equivalent of a single bulk write, not exceeding the
    // current CWND in packets.
    burst_tokens_ = std::min(
        initial_burst_size_,
        static_cast<uint32_t>(sender_->GetCongestionWindow() / kDefaultTCPMSS));
  }
  if (burst_tokens_ > 0) {
    --burst_tokens_;
    if (!GetQuicReloadableFlag(quic_donot_reset_ideal_next_packet_send_time)) {
      ideal_next_packet_send_time_ = ProtoTime::Zero();
    }
    pacing_limited_ = false;
    last_debug_state_.burst_tokens = burst_tokens_;
    last_debug_state_.lumpy_tokens = lumpy_tokens_;
    last_debug_state_.pacing_limited = pacing_limited_;
    last_debug_state_.ideal_next_send_time_us =
        ideal_next_packet_send_time_.ToDebuggingValue();
    return;
  }
  // The next packet should be sent as soon as the current packet has been
  // transferred.  PacingRate is based on bytes in flight including this packet.
  QuicBandwidth bw=PacingRate(bytes_in_flight + bytes);
  //QuicBandwidth sender_bw=sender_->PacingRate(bytes_in_flight+bytes);
  TimeDelta delay =bw.TransferTime(bytes);
  last_debug_state_.commanded_pacing_bps = bw.ToBitsPerSecond();
  last_debug_state_.requested_delay_us = delay.ToMicroseconds();
  if (!pacing_limited_ || lumpy_tokens_ == 0) {
    // Reset lumpy_tokens_ if either application or cwnd throttles sending or
    // token runs out.
    lumpy_tokens_ = std::max(
        1u, std::min(static_cast<uint32_t>(
                         GetQuicFlag(FLAG_quic_lumpy_pacing_size)),
                     static_cast<uint32_t>(
                         (sender_->GetCongestionWindow() *
                          GetQuicFlag(FLAG_quic_lumpy_pacing_cwnd_fraction)) /
                         kDefaultTCPMSS)));
    if (GetQuicReloadableFlag(quic_no_lumpy_pacing_at_low_bw) &&
        sender_->BandwidthEstimate() <
            QuicBandwidth::FromKBitsPerSecond(1200)) {
      QUIC_RELOADABLE_FLAG_COUNT(quic_no_lumpy_pacing_at_low_bw);
      // Below 1.2Mbps, send 1 packet at once, because one full-sized packet
      // is about 10ms of queueing.
      lumpy_tokens_ = 1u;
    }
  }
  // The coded carrier is only a few percent of the baseline. Sending a
  // lumpy group at one timestamp quantizes away that signal even when the
  // average pacing rate is correct, so use one packet per pacing interval
  // while its CRUISE identification probe is active.
  if (sender_->RequiresFineGrainedPacing()) {
    lumpy_tokens_ = 1;
    last_debug_state_.fine_grained = true;
  }
  --lumpy_tokens_;
  if (pacing_limited_) {
    // Make up for lost time since pacing throttles the sending.
    ideal_next_packet_send_time_ = ideal_next_packet_send_time_ + delay;
    //DLOG(INFO)<<ideal_next_packet_send_time_<<" "<<delay;
  } else {
    ideal_next_packet_send_time_ =
        std::max(ideal_next_packet_send_time_ + delay, sent_time + delay);
  }
  // Stop making up for lost time if underlying sender prevents sending.
  pacing_limited_ = sender_->CanSend(bytes_in_flight + bytes);
  last_debug_state_.burst_tokens = burst_tokens_;
  last_debug_state_.lumpy_tokens = lumpy_tokens_;
  last_debug_state_.pacing_limited = pacing_limited_;
  last_debug_state_.ideal_next_send_time_us =
      ideal_next_packet_send_time_.ToDebuggingValue();
}

PacingSender::DebugState PacingSender::ExportDebugState() const {
  return last_debug_state_;
}

void PacingSender::OnApplicationLimited() {
  // The send is application limited, stop making up for lost time.
  pacing_limited_ = false;
}

void PacingSender::SetBurstTokens(uint32_t burst_tokens) {
  initial_burst_size_ = burst_tokens;
  burst_tokens_ = std::min(
      initial_burst_size_,
      static_cast<uint32_t>(sender_->GetCongestionWindow() / kDefaultTCPMSS));
}

TimeDelta PacingSender::TimeUntilSend(
    ProtoTime now,
    QuicByteCount bytes_in_flight) const {
  DCHECK(sender_ != nullptr);

  if (!sender_->CanSend(bytes_in_flight)) {
    // The underlying sender prevents sending.
    return TimeDelta::Infinite();
  }

  if (burst_tokens_ > 0 || bytes_in_flight == 0 || lumpy_tokens_ > 0) {
    // Don't pace if we have burst tokens available or leaving quiescence.
    return TimeDelta::Zero();
  }

  // If the next send time is within the alarm granularity, send immediately.
  if (ideal_next_packet_send_time_ > now + alarm_granularity_) {
    DLOG(INFO) << "Delaying packet: "
                  << (ideal_next_packet_send_time_ - now).ToMicroseconds();
    return ideal_next_packet_send_time_ - now;
  }

  DLOG(INFO)<< "Sending packet now. ideal_next_packet_send_time: "
                << ideal_next_packet_send_time_ << ", now: " << now;
  return TimeDelta::Zero();
}

QuicBandwidth PacingSender::PacingRate(QuicByteCount bytes_in_flight) const {
  DCHECK(sender_ != nullptr);
  if (!max_pacing_rate_.IsZero()) {
    return QuicBandwidth::FromBitsPerSecond(
        std::min(max_pacing_rate_.ToBitsPerSecond(),
                 sender_->PacingRate(bytes_in_flight).ToBitsPerSecond()));
  }
  QuicBandwidth pacing_rate=sender_->PacingRate(bytes_in_flight);
  return  pacing_rate;
}
}
