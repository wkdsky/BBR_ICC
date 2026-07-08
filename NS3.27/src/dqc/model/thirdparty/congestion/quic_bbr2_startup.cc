// Copyright 2019 The Chromium Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "quic_bbr2_startup.h"
#include "quic_bbr2_misc.h"
#include "quic_bbr2_sender.h"
#include "quic_logging.h"
namespace dqc {

Bbr2StartupMode::Bbr2StartupMode(Bbr2Sender* sender,
                                 Bbr2NetworkModel* model,
                                 QuicTime now)
    : Bbr2ModeBase(sender, model) {
  // Increment instead of reset so data recorded before a sender switch is kept.
  ++sender_->connection_stats_->slowstart_count;
  if (!sender_->connection_stats_->slowstart_duration.IsRunning()) {
    sender_->connection_stats_->slowstart_duration.Start(now);
  }
  model_->set_pacing_gain(Params().startup_pacing_gain);
  model_->set_cwnd_gain(Params().startup_cwnd_gain);
}

void Bbr2StartupMode::Enter(QuicTime /*now*/,
                            const Bbr2CongestionEvent* /*congestion_event*/) {
  QUIC_BUG << "Bbr2StartupMode::Enter should not be called";
}

void Bbr2StartupMode::Leave(QuicTime now,
                            const Bbr2CongestionEvent* /*congestion_event*/) {
  sender_->connection_stats_->slowstart_duration.Stop(now);
  model_->clear_bandwidth_lo();
}

Bbr2Mode Bbr2StartupMode::OnCongestionEvent(
    QuicByteCount /*prior_in_flight*/,
    QuicTime /*event_time*/,
    const AckedPacketVector& /*acked_packets*/,
    const LostPacketVector& /*lost_packets*/,
    const Bbr2CongestionEvent& congestion_event) {
  if (model_->full_bandwidth_reached()) {
    QUIC_BUG << "In STARTUP, but full_bandwidth_reached is true.";
    return Bbr2Mode::DRAIN;
  }

  if (!congestion_event.end_of_round_trip) {
    return Bbr2Mode::STARTUP;
  }

  bool has_bandwidth_growth = model_->HasBandwidthGrowth(congestion_event);
  if (Params().max_startup_queue_rounds > 0 && !has_bandwidth_growth) {
    // 1.75 is less than the 2x CWND gain, but substantially more than 1.25x,
    // the minimum bandwidth increase expected during STARTUP.
    model_->CheckPersistentQueue(congestion_event, 1.75);
  }

  // TCP BBR always exits upon excessive losses. QUIC BBRv1 does not exit
  // upon excessive losses if enough bandwidth growth is observed or if the
  // sample was app limited.
  if (Params().always_exit_startup_on_excess_loss ||
      (!congestion_event.last_packet_send_state.is_app_limited &&
       !has_bandwidth_growth)) {
    CheckExcessiveLosses(congestion_event);
  }

  if (Params().decrease_startup_pacing_at_end_of_round) {
    DCHECK_GT(model_->pacing_gain(), 0);
    if (!congestion_event.last_packet_send_state.is_app_limited) {
      if (max_bw_at_round_beginning_ > QuicBandwidth::Zero()) {
        const float bandwidth_ratio =
            std::max(1., model_->MaxBandwidth().ToBitsPerSecond() /
                             static_cast<double>(
                                 max_bw_at_round_beginning_.ToBitsPerSecond()));
        const float new_gain =
            ((bandwidth_ratio - 1) *
             (Params().startup_pacing_gain - Params().full_bw_threshold)) +
            Params().full_bw_threshold;
        model_->set_pacing_gain(
            std::min(Params().startup_pacing_gain, new_gain));
        if (model_->bandwidth_lo() <
            model_->MaxBandwidth() * model_->pacing_gain()) {
          model_->clear_bandwidth_lo();
        }
      }
      max_bw_at_round_beginning_ = model_->MaxBandwidth();
    }
  }

  model_->set_cwnd_gain(Params().startup_cwnd_gain);

  // TODO(wub): Maybe implement STARTUP => PROBE_RTT.
  return model_->full_bandwidth_reached() ? Bbr2Mode::DRAIN
                                          : Bbr2Mode::STARTUP;
}
void Bbr2StartupMode::CheckEcnTooHigh(uint32_t ce_ratio){
    if(model_->full_bandwidth_reached()||!Params().enable_ecn){
        return;
    }
    if(ce_ratio>=Params().ecn_thresh){
        rounds_ecn_++;
    }else{
        rounds_ecn_=0;
    }
    if(rounds_ecn_>=Params().full_ecn_count){
        const QuicByteCount bdp = model_->BDP();
        model_->set_inflight_hi(bdp);
        model_->set_full_bandwidth_reached();
    }
}
void Bbr2StartupMode::CheckExcessiveLosses(
    const Bbr2CongestionEvent& congestion_event) {
  if (model_->full_bandwidth_reached()) {
    return;
  }

  DCHECK(congestion_event.end_of_round_trip);

  QUIC_DVLOG(3)
      << sender_
      << " CheckExcessiveLosses at end of round. loss_events_in_round:"
      << model_->loss_events_in_round()
      << ", threshold:" << Params().startup_full_loss_count << "  @ "
      << congestion_event.event_time;

  // At the end of a round trip. Check if loss is too high in this round.
  if (model_->IsInflightTooHighWithEcn(congestion_event,
                                       Params().startup_full_loss_count,
                                       sender_->GetBytesEcnInRounds())) {
    QuicByteCount new_inflight_hi = model_->BDP();
    if (Params().startup_loss_exit_use_max_delivered_for_inflight_hi &&
        new_inflight_hi < model_->max_bytes_delivered_in_round()) {
      new_inflight_hi = model_->max_bytes_delivered_in_round();
    }
    QUIC_DVLOG(3) << sender_
                  << " Exiting STARTUP due to loss. inflight_hi:"
                  << new_inflight_hi;
    model_->set_inflight_hi(new_inflight_hi);

    model_->set_full_bandwidth_reached();
    sender_->connection_stats_->bbr_exit_startup_due_to_loss = true;
  }
}

Bbr2StartupMode::DebugState Bbr2StartupMode::ExportDebugState() const {
  DebugState s;
  s.full_bandwidth_reached = model_->full_bandwidth_reached();
  s.full_bandwidth_baseline = model_->full_bandwidth_baseline();
  s.round_trips_without_bandwidth_growth =
      model_->rounds_without_bandwidth_growth();
  return s;
}

std::ostream& operator<<(std::ostream& os,
                         const Bbr2StartupMode::DebugState& state) {
  os << "[STARTUP] full_bandwidth_reached: " << state.full_bandwidth_reached
     << "\n";
  os << "[STARTUP] full_bandwidth_baseline: " << state.full_bandwidth_baseline
     << "\n";
  os << "[STARTUP] round_trips_without_bandwidth_growth: "
     << state.round_trips_without_bandwidth_growth << "\n";
  return os;
}

const Bbr2Params& Bbr2StartupMode::Params() const {
  return sender_->Params();
}

}  // namespace quic
