/*
 * CUBIC control-law adaptation based on ns-3.47 TcpCubic.
 * Upstream files: src/internet/model/tcp-cubic.{cc,h}
 * Upstream license: GPL-2.0-only.
 */

#include "ns3_cubic_sender.h"

#include "proto_constants.h"
#include "rtt_stats.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>

namespace dqc {
namespace {

constexpr double kBeta = 0.7;
constexpr double kCubicScale = 0.4;
constexpr bool kFastConvergence = true;
constexpr bool kTcpFriendliness = true;
constexpr bool kHystart = true;
constexpr uint32_t kHystartLowWindowPackets = 16;
constexpr uint32_t kHystartMinSamples = 8;
constexpr int64_t kHystartAckDeltaUs = 2000;
constexpr int64_t kHystartDelayMinUs = 4000;
constexpr int64_t kHystartDelayMaxUs = 1000000;
constexpr int64_t kCubicDeltaUs = 10000;
constexpr uint32_t kCountClamp = 20;
constexpr QuicByteCount kMinimumCongestionWindow = 2 * kDefaultTCPMSS;
constexpr QuicByteCount kMaxBurstBytes = 3 * kDefaultTCPMSS;
constexpr QuicPacketCount kMaxResumptionCongestionWindow = 200;

uint32_t BytesToSegments(QuicByteCount bytes) {
  return static_cast<uint32_t>(
      std::max<QuicByteCount>(1, (bytes + kDefaultTCPMSS - 1) /
                                    kDefaultTCPMSS));
}

}  // namespace

Ns3CubicSender::Ns3CubicSender(
    const ProtoClock* clock,
    const RttStats* rtt_stats,
    QuicPacketCount initial_congestion_window,
    QuicPacketCount max_congestion_window,
    QuicConnectionStats* stats)
    : rtt_stats_(rtt_stats),
      stats_(stats),
      congestion_window_(initial_congestion_window * kDefaultTCPMSS),
      min_congestion_window_(kMinimumCongestionWindow),
      max_congestion_window_(max_congestion_window * kDefaultTCPMSS),
      slowstart_threshold_(max_congestion_window * kDefaultTCPMSS),
      initial_congestion_window_(initial_congestion_window * kDefaultTCPMSS),
      congestion_window_count_(0),
      ack_count_(0),
      tcp_congestion_window_(0),
      last_max_congestion_window_(0),
      bic_origin_point_(0),
      bic_k_seconds_(0.0),
      delay_min_(TimeDelta::Zero()),
      epoch_start_(ProtoTime::Zero()),
      hystart_found_(false),
      hystart_round_start_(ProtoTime::Zero()),
      hystart_last_ack_(ProtoTime::Zero()),
      hystart_current_rtt_(TimeDelta::Zero()),
      hystart_sample_count_(0) {
  (void)clock;
}

Ns3CubicSender::~Ns3CubicSender() = default;

void Ns3CubicSender::AdjustNetworkParameters(
    QuicBandwidth bandwidth,
    TimeDelta rtt,
    bool /*allow_cwnd_to_decrease*/) {
  if (bandwidth.IsZero() || rtt.IsZero()) {
    return;
  }
  const QuicByteCount target = bandwidth.ToBytesPerPeriod(rtt);
  congestion_window_ =
      std::max(min_congestion_window_,
               std::min(target,
                        kMaxResumptionCongestionWindow * kDefaultTCPMSS));
}

void Ns3CubicSender::SetNumEmulatedConnections(int /*num_connections*/) {
  // ns-3 TcpCubic models one TCP connection and has no N-connection knob.
}

void Ns3CubicSender::SetInitialCongestionWindowInPackets(
    QuicPacketCount congestion_window) {
  congestion_window_ = std::max(min_congestion_window_,
                                congestion_window * kDefaultTCPMSS);
  initial_congestion_window_ = congestion_window_;
}

void Ns3CubicSender::OnConnectionMigration() {
  ResetConnectionState();
}

void Ns3CubicSender::OnCongestionEvent(
    bool rtt_updated,
    QuicByteCount prior_in_flight,
    ProtoTime event_time,
    const AckedPacketVector& acked_packets,
    const LostPacketVector& lost_packets) {
  if (rtt_updated) {
    UpdateRttAndHystart(event_time);
  }

  for (const LostPacket& lost_packet : lost_packets) {
    OnPacketLost(lost_packet.packet_number, lost_packet.bytes_lost,
                 prior_in_flight);
  }
  for (const AckedPacket& acked_packet : acked_packets) {
    OnPacketAcked(acked_packet.packet_number, acked_packet.bytes_acked,
                  prior_in_flight, event_time);
  }
}

void Ns3CubicSender::OnPacketSent(
    ProtoTime /*sent_time*/,
    QuicByteCount /*bytes_in_flight*/,
    QuicPacketNumber packet_number,
    QuicByteCount bytes,
    HasRetransmittableData is_retransmittable) {
  if (InSlowStart() && stats_ != nullptr) {
    ++stats_->slowstart_packets_sent;
  }
  if (is_retransmittable != HAS_RETRANSMITTABLE_DATA) {
    return;
  }
  if (InRecovery()) {
    prr_.OnPacketSent(bytes);
  }
  largest_sent_packet_number_ = packet_number;
}

void Ns3CubicSender::OnRetransmissionTimeout(bool packets_retransmitted) {
  largest_sent_at_last_cutback_.Clear();
  if (!packets_retransmitted) {
    return;
  }
  CubicReset();
  slowstart_threshold_ = std::max(congestion_window_ / 2,
                                  min_congestion_window_);
  congestion_window_ = min_congestion_window_;
}

bool Ns3CubicSender::CanSend(QuicByteCount bytes_in_flight) {
  if (InRecovery()) {
    return prr_.CanSend(congestion_window_, bytes_in_flight,
                        slowstart_threshold_);
  }
  return congestion_window_ > bytes_in_flight;
}

QuicBandwidth Ns3CubicSender::PacingRate(
    QuicByteCount /*bytes_in_flight*/) const {
  const TimeDelta srtt = rtt_stats_->SmoothedOrInitialRtt();
  const QuicBandwidth window_rate =
      QuicBandwidth::FromBytesAndTimeDelta(congestion_window_, srtt);
  return window_rate * (InSlowStart() ? 2.0 : 1.25);
}

QuicBandwidth Ns3CubicSender::BandwidthEstimate() const {
  const TimeDelta srtt = rtt_stats_->smoothed_rtt();
  if (srtt.IsZero()) {
    return QuicBandwidth::Zero();
  }
  return QuicBandwidth::FromBytesAndTimeDelta(congestion_window_, srtt);
}

QuicByteCount Ns3CubicSender::GetCongestionWindow() const {
  return congestion_window_;
}

QuicByteCount Ns3CubicSender::GetSlowStartThreshold() const {
  return slowstart_threshold_;
}

CongestionControlType Ns3CubicSender::GetCongestionControlType() const {
  return kNs3Cubic;
}

bool Ns3CubicSender::InSlowStart() const {
  return congestion_window_ < slowstart_threshold_;
}

bool Ns3CubicSender::InRecovery() const {
  return largest_acked_packet_number_.IsInitialized() &&
         largest_sent_at_last_cutback_.IsInitialized() &&
         largest_acked_packet_number_ <= largest_sent_at_last_cutback_;
}

bool Ns3CubicSender::ShouldSendProbingPacket() const {
  return false;
}

std::string Ns3CubicSender::GetDebugState() const {
  std::ostringstream out;
  out << "ns3-cubic cwnd=" << congestion_window_
      << " ssthresh=" << slowstart_threshold_
      << " last_max_packets=" << last_max_congestion_window_
      << " epoch_us=" << epoch_start_.ToDebuggingValue();
  return out.str();
}

void Ns3CubicSender::OnApplicationLimited(
    QuicByteCount /*bytes_in_flight*/) {
  // Upstream ns-3.47 suppresses growth through m_isCwndLimited; it does not
  // reset the CUBIC epoch on application-limited notification.
}

bool Ns3CubicSender::IsCwndLimited(QuicByteCount bytes_in_flight) const {
  if (bytes_in_flight >= congestion_window_) {
    return true;
  }
  const QuicByteCount available = congestion_window_ - bytes_in_flight;
  return (InSlowStart() && bytes_in_flight > congestion_window_ / 2) ||
         available <= kMaxBurstBytes;
}

void Ns3CubicSender::OnPacketAcked(
    QuicPacketNumber packet_number,
    QuicByteCount bytes_acked,
    QuicByteCount prior_in_flight,
    ProtoTime event_time) {
  largest_acked_packet_number_.UpdateMax(packet_number);
  if (InRecovery()) {
    prr_.OnPacketAcked(bytes_acked);
    return;
  }
  if (!IsCwndLimited(prior_in_flight) ||
      congestion_window_ >= max_congestion_window_) {
    return;
  }

  const uint32_t segments_acked = BytesToSegments(bytes_acked);
  if (InSlowStart()) {
    if (kHystart && hystart_end_packet_number_.IsInitialized() &&
        packet_number > hystart_end_packet_number_) {
      HystartReset(event_time);
    }
    const QuicByteCount increase =
        static_cast<QuicByteCount>(segments_acked) * kDefaultTCPMSS;
    congestion_window_ =
        std::min(max_congestion_window_, congestion_window_ + increase);
    return;
  }

  congestion_window_count_ += segments_acked;
  const uint32_t count = CubicUpdate(segments_acked, event_time);
  if (congestion_window_count_ >= count) {
    congestion_window_ =
        std::min(max_congestion_window_, congestion_window_ + kDefaultTCPMSS);
    congestion_window_count_ -= count;
  }
}

void Ns3CubicSender::OnPacketLost(
    QuicPacketNumber packet_number,
    QuicByteCount bytes_lost,
    QuicByteCount prior_in_flight) {
  if (largest_sent_at_last_cutback_.IsInitialized() &&
      packet_number <= largest_sent_at_last_cutback_) {
    if (stats_ != nullptr && InSlowStart()) {
      ++stats_->slowstart_packets_lost;
      stats_->slowstart_bytes_lost += bytes_lost;
    }
    return;
  }

  if (stats_ != nullptr) {
    ++stats_->tcp_loss_events;
    if (InSlowStart()) {
      ++stats_->slowstart_packets_lost;
    }
  }
  prr_.OnPacketLost(prior_in_flight);

  const uint32_t segment_window = static_cast<uint32_t>(
      std::max<QuicByteCount>(1, congestion_window_ / kDefaultTCPMSS));
  if (kFastConvergence && segment_window < last_max_congestion_window_) {
    last_max_congestion_window_ = static_cast<uint32_t>(
        segment_window * (1.0 + kBeta) / 2.0);
  } else {
    last_max_congestion_window_ = segment_window;
  }
  epoch_start_ = ProtoTime::Zero();

  const uint32_t threshold_packets =
      std::max(static_cast<uint32_t>(segment_window * kBeta), 2U);
  slowstart_threshold_ = threshold_packets * kDefaultTCPMSS;
  congestion_window_ = std::max(slowstart_threshold_, min_congestion_window_);
  largest_sent_at_last_cutback_ = largest_sent_packet_number_;
  congestion_window_count_ = 0;
}

uint32_t Ns3CubicSender::CubicUpdate(uint32_t segments_acked,
                                     ProtoTime event_time) {
  const uint32_t segment_window = static_cast<uint32_t>(
      std::max<QuicByteCount>(1, congestion_window_ / kDefaultTCPMSS));
  ack_count_ += segments_acked;

  if (!epoch_start_.IsInitialized()) {
    epoch_start_ = event_time;
    ack_count_ = segments_acked;
    tcp_congestion_window_ = segment_window;
    if (last_max_congestion_window_ <= segment_window) {
      bic_k_seconds_ = 0.0;
      bic_origin_point_ = segment_window;
    } else {
      bic_k_seconds_ = std::cbrt(
          (last_max_congestion_window_ - segment_window) / kCubicScale);
      bic_origin_point_ = last_max_congestion_window_;
    }
  }

  const double elapsed_seconds =
      (event_time - epoch_start_ + delay_min_).ToMicroseconds() / 1000000.0;
  const double offset = std::abs(elapsed_seconds - bic_k_seconds_);
  const double delta_double = kCubicScale * offset * offset * offset;
  const uint32_t delta = static_cast<uint32_t>(std::min(
      delta_double, static_cast<double>(std::numeric_limits<uint32_t>::max())));

  uint32_t target = 0;
  if (elapsed_seconds < bic_k_seconds_) {
    target = delta < bic_origin_point_ ? bic_origin_point_ - delta : 0;
  } else {
    const uint64_t sum = static_cast<uint64_t>(bic_origin_point_) + delta;
    target = static_cast<uint32_t>(std::min<uint64_t>(
        sum, std::numeric_limits<uint32_t>::max()));
  }

  uint32_t count = target > segment_window
                       ? segment_window / (target - segment_window)
                       : 100 * segment_window;
  if (last_max_congestion_window_ == 0 && count > kCountClamp) {
    count = kCountClamp;
  }

  if (kTcpFriendliness) {
    const uint32_t scale = static_cast<uint32_t>(
        8.0 * (1024.0 + kBeta * 1024.0) /
        (3.0 * (1024.0 - kBeta * 1024.0)));
    const uint32_t delta_reno =
        std::max<uint32_t>(1, (segment_window * scale) >> 3);
    while (ack_count_ > delta_reno) {
      ack_count_ -= delta_reno;
      ++tcp_congestion_window_;
    }
    if (tcp_congestion_window_ > segment_window) {
      const uint32_t difference = tcp_congestion_window_ - segment_window;
      const uint32_t max_count = segment_window / difference;
      count = std::min(count, max_count);
    }
  }

  // Same upper growth bound as ns-3.47: at most one MSS per two ACKs.
  return std::max(count, 2U);
}

void Ns3CubicSender::UpdateRttAndHystart(ProtoTime event_time) {
  if (epoch_start_.IsInitialized() &&
      (event_time - epoch_start_).ToMicroseconds() < kCubicDeltaUs) {
    return;
  }
  const TimeDelta rtt = rtt_stats_->latest_rtt();
  if (rtt.IsZero()) {
    return;
  }
  if (delay_min_.IsZero() || rtt < delay_min_) {
    delay_min_ = rtt;
  }
  if (!kHystart || !InSlowStart() ||
      congestion_window_ < kHystartLowWindowPackets * kDefaultTCPMSS) {
    return;
  }
  if (!hystart_end_packet_number_.IsInitialized()) {
    HystartReset(event_time);
  }
  HystartUpdate(event_time, rtt);
}

void Ns3CubicSender::HystartUpdate(ProtoTime event_time, TimeDelta delay) {
  if (hystart_found_) {
    return;
  }

  if ((event_time - hystart_last_ack_).ToMicroseconds() <=
      kHystartAckDeltaUs) {
    hystart_last_ack_ = event_time;
    if ((event_time - hystart_round_start_) > delay_min_) {
      hystart_found_ = true;
    }
  }

  if (hystart_sample_count_ < kHystartMinSamples) {
    if (hystart_current_rtt_.IsZero() ||
        delay < hystart_current_rtt_) {
      hystart_current_rtt_ = delay;
    }
    ++hystart_sample_count_;
  } else if (hystart_current_rtt_ >
             delay_min_ + HystartDelayThreshold(delay_min_)) {
    hystart_found_ = true;
  }

  if (hystart_found_) {
    slowstart_threshold_ = congestion_window_;
  }
}

void Ns3CubicSender::HystartReset(ProtoTime event_time) {
  hystart_round_start_ = event_time;
  hystart_last_ack_ = event_time;
  hystart_end_packet_number_ = largest_sent_packet_number_;
  hystart_current_rtt_ = TimeDelta::Zero();
  hystart_sample_count_ = 0;
}

TimeDelta Ns3CubicSender::HystartDelayThreshold(TimeDelta delay) const {
  const int64_t threshold_us =
      std::max(kHystartDelayMinUs,
               std::min(delay.ToMicroseconds(), kHystartDelayMaxUs));
  return TimeDelta::FromMicroseconds(threshold_us);
}

void Ns3CubicSender::CubicReset() {
  bic_origin_point_ = 0;
  bic_k_seconds_ = 0.0;
  ack_count_ = 0;
  tcp_congestion_window_ = 0;
  delay_min_ = TimeDelta::Zero();
  epoch_start_ = ProtoTime::Zero();
  hystart_found_ = false;
  hystart_current_rtt_ = TimeDelta::Zero();
  hystart_sample_count_ = 0;
  hystart_end_packet_number_.Clear();
}

void Ns3CubicSender::ResetConnectionState() {
  prr_ = PrrSender();
  largest_sent_packet_number_.Clear();
  largest_acked_packet_number_.Clear();
  largest_sent_at_last_cutback_.Clear();
  CubicReset();
  last_max_congestion_window_ = 0;
  congestion_window_count_ = 0;
  congestion_window_ = initial_congestion_window_;
  slowstart_threshold_ = max_congestion_window_;
}

}  // namespace dqc
