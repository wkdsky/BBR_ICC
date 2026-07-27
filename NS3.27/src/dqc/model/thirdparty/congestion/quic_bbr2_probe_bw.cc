// Copyright 2019 The Chromium Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "quic_bbr2_probe_bw.h"

#include "quic_bbr2_misc.h"
#include "quic_bbr2_sender.h"
#include "proto_bandwidth.h"
#include "quic_logging.h"
#include "flag_impl.h"
#include "flag_util_impl.h"
namespace dqc {

void Bbr2ProbeBwMode::Enter(QuicTime now,
                            const Bbr2CongestionEvent* /*congestion_event*/) {
  if (cycle_.phase == CyclePhase::PROBE_NOT_STARTED) {
    // First time entering PROBE_BW. Start a new probing cycle.
    EnterProbeDown(/*probed_too_high=*/false, /*stopped_risky_probe=*/false,
                   now);
  } else {
    // Transitioning from PROBE_RTT to PROBE_BW. Re-enter the last phase before
    // PROBE_RTT.
    DCHECK(cycle_.phase == CyclePhase::PROBE_CRUISE ||
           cycle_.phase == CyclePhase::PROBE_REFILL);
    cycle_.cycle_start_time = now;
    if (cycle_.phase == CyclePhase::PROBE_CRUISE) {
      EnterProbeCruise(now);
    } else if (cycle_.phase == CyclePhase::PROBE_REFILL) {
      EnterProbeRefill(cycle_.probe_up_rounds, now);
    }
  }
}

Bbr2Mode Bbr2ProbeBwMode::OnCongestionEvent(
    QuicByteCount prior_in_flight,
    QuicTime event_time,
    const AckedPacketVector& /*acked_packets*/,
    const LostPacketVector& /*lost_packets*/,
    const Bbr2CongestionEvent& congestion_event) {
  DCHECK_NE(cycle_.phase, CyclePhase::PROBE_NOT_STARTED);

  if (congestion_event.end_of_round_trip) {
    if (cycle_.cycle_start_time != event_time) {
      ++cycle_.rounds_since_probe;
    }
    if (cycle_.phase_start_time != event_time) {
      ++cycle_.rounds_in_phase;
    }
  }

  if (sender_->ShouldForceProbeUp(event_time)) {
    if (cycle_.phase != CyclePhase::PROBE_UP) {
      ForceEnterProbeUp(event_time);
      model_->set_pacing_gain(PacingGainForPhase(cycle_.phase));
      model_->set_cwnd_gain(CwndGainForPhase(cycle_.phase));
      return Bbr2Mode::PROBE_BW;
    }
    sender_->MarkExperimentalForcedProbeUpStarted(event_time);
  }

  bool switch_to_probe_rtt = false;

  if (cycle_.phase == CyclePhase::PROBE_UP) {
    UpdateProbeUp(prior_in_flight, congestion_event);
  } else if (cycle_.phase == CyclePhase::PROBE_PRE_UP) {
    UpdateProbePreUp(congestion_event);
  } else if (cycle_.phase == CyclePhase::PROBE_GUARD) {
    UpdateProbeGuard(congestion_event);
  } else if (cycle_.phase == CyclePhase::PROBE_POST_UP) {
    UpdateProbePostUp(congestion_event);
  } else if (cycle_.phase == CyclePhase::PROBE_DOWN) {
    UpdateProbeDown(prior_in_flight, congestion_event);
    // Maybe transition to PROBE_RTT at the end of this cycle.
    if (sender_->IsProbeRttEnabled() &&
        cycle_.phase != CyclePhase::PROBE_DOWN &&
        model_->MaybeExpireMinRtt(congestion_event)) {
      switch_to_probe_rtt = true;
    }
  } else if (cycle_.phase == CyclePhase::PROBE_DOWN_SLIGHTLY) {
    UpdateProbeDownSlightly(prior_in_flight, congestion_event);
    if (sender_->IsProbeRttEnabled() &&
        cycle_.phase != CyclePhase::PROBE_DOWN_SLIGHTLY &&
        model_->MaybeExpireMinRtt(congestion_event)) {
      switch_to_probe_rtt = true;
    }
  } else if (cycle_.phase == CyclePhase::PROBE_CRUISE) {
    UpdateProbeCruise(congestion_event);
  } else if (cycle_.phase == CyclePhase::PROBE_REFILL) {
    UpdateProbeRefill(congestion_event);
  }

  // BBRv2+ can ask to restart STARTUP after its dual-mode detector observes
  // persistent queueing in Cruise.  This takes precedence over a simultaneous
  // ProbeRTT transition, as the BBRv2+ design explicitly refreshes the
  // bandwidth model from STARTUP in that case.
  const bool switch_to_startup = sender_->ConsumeStartupRestartRequest();
  if (switch_to_startup) {
    switch_to_probe_rtt = false;
    sender_->PrepareForStartupRestart();
  }

  // Do not need to set gains when switching modes: Enter() does that for the
  // destination mode.
  if (!switch_to_probe_rtt && !switch_to_startup) {
    model_->set_pacing_gain(PacingGainForPhase(cycle_.phase));
    model_->set_cwnd_gain(CwndGainForPhase(cycle_.phase));
  }

  if (switch_to_startup) {
    return Bbr2Mode::STARTUP;
  }
  return switch_to_probe_rtt ? Bbr2Mode::PROBE_RTT : Bbr2Mode::PROBE_BW;
}

QuicLimits<QuicByteCount> Bbr2ProbeBwMode::GetCwndLimits() const {
  QuicByteCount upper_limit = 0;
  if (cycle_.phase == CyclePhase::PROBE_CRUISE) {
    upper_limit =
        std::min(model_->inflight_lo(), model_->inflight_hi_with_headroom());
  } else if (Params().probe_up_ignore_inflight_hi &&
             cycle_.phase == CyclePhase::PROBE_UP) {
    // Similar to STARTUP.
    upper_limit = model_->inflight_lo();
  } else {
    upper_limit = std::min(model_->inflight_lo(), model_->inflight_hi());
  }

  if (Params().avoid_too_low_probe_bw_cwnd) {
    // Ensure upper_limit is at least BDP + AckHeight.
    QuicByteCount bdp_with_ack_height =
        model_->BDP() + model_->MaxAckHeight();
    if (upper_limit < bdp_with_ack_height) {
      QUIC_DVLOG(3) << sender_ << " Rasing upper_limit from " << upper_limit
                    << " to " << bdp_with_ack_height;
      QUIC_CODE_COUNT(quic_bbr2_avoid_too_low_probe_bw_cwnd_in_effect);
      upper_limit = bdp_with_ack_height;
    }
  }

  return NoGreaterThan(upper_limit);
}

bool Bbr2ProbeBwMode::IsProbingForBandwidth() const {
  return cycle_.phase == CyclePhase::PROBE_REFILL ||
         cycle_.phase == CyclePhase::PROBE_PRE_UP ||
         cycle_.phase == CyclePhase::PROBE_GUARD ||
         cycle_.phase == CyclePhase::PROBE_UP ||
         cycle_.phase == CyclePhase::PROBE_POST_UP;
}

Bbr2Mode Bbr2ProbeBwMode::OnExitQuiescence(QuicTime now,
                                           QuicTime quiescence_start_time) {
  QUIC_DVLOG(3) << sender_ << " Postponing min_rtt_timestamp("
                << model_->MinRttTimestamp() << ") by "
                << now - quiescence_start_time;
  model_->PostponeMinRttTimestamp(now - quiescence_start_time);
  return Bbr2Mode::PROBE_BW;
}

void Bbr2ProbeBwMode::UpdateProbeDown(
    QuicByteCount prior_in_flight,
    const Bbr2CongestionEvent& congestion_event) {
  DCHECK_EQ(cycle_.phase, CyclePhase::PROBE_DOWN);

  bool advanced_max_bandwidth_filter = false;

  if (cycle_.rounds_in_phase == 1 && congestion_event.end_of_round_trip) {
    cycle_.is_sample_from_probing = false;

    if (!congestion_event.last_sample_is_app_limited) {
      QUIC_DVLOG(2)
          << sender_
          << " Advancing max bw filter after one round in PROBE_DOWN.";
      model_->AdvanceMaxBandwidthFilter();
      cycle_.has_advanced_max_bw = true;
      advanced_max_bandwidth_filter = true;
      sender_->OnMaxBandwidthFilterAdvanced(cycle_.phase);
    }

    if (!sender_->HasCustomProbeDownLogic() &&
        last_cycle_stopped_risky_probe_ && !last_cycle_probed_too_high_) {
      EnterProbeRefill(/*probe_up_rounds=*/0, congestion_event.event_time);
      return;
    }
  }

  // BBRv2+ expires stale BtlBW samples on RTT inflation in both Cruise and
  // Down. Avoid a duplicate advance on the native first Down round.
  if (congestion_event.end_of_round_trip &&
      !advanced_max_bandwidth_filter &&
      sender_->ShouldAdvanceMaxBandwidthFilterOnRoundStart(cycle_.phase)) {
    model_->AdvanceMaxBandwidthFilter();
    cycle_.has_advanced_max_bw = true;
    sender_->OnMaxBandwidthFilterAdvanced(cycle_.phase);
  }

  MaybeAdaptUpperBounds(congestion_event);

  QuicByteCount bytes_in_flight = congestion_event.bytes_in_flight;
  QuicByteCount bdp = model_->BDP();
  if (sender_->HasCustomProbeDownLogic()) {
    QUIC_DVLOG(3)
        << sender_
        << " Checking custom PROBE_DOWN exit. bytes_in_flight:"
        << bytes_in_flight << ", bdp:" << bdp;
    if (sender_->ShouldExitCustomProbeDown(bytes_in_flight, bdp)) {
      EnterProbeCruise(congestion_event.event_time);
    }
    return;
  }

  if (IsTimeToProbeBandwidth(congestion_event)) {
    EnterProbeRefill(/*probe_up_rounds=*/0, congestion_event.event_time);
    return;
  }

  if (HasStayedLongEnoughInProbeDown(congestion_event)) {
    QUIC_DVLOG(3) << sender_ << " Proportional time based PROBE_DOWN exit";
    EnterProbeCruise(congestion_event.event_time);
    return;
  }

  const QuicByteCount inflight_with_headroom =
      model_->inflight_hi_with_headroom();
  QUIC_DVLOG(3)
      << sender_
      << " Checking if have enough inflight headroom. prior_in_flight:"
      << prior_in_flight
      << " congestion_event.bytes_in_flight:"
      << congestion_event.bytes_in_flight
      << ", inflight_with_headroom:" << inflight_with_headroom;
  if (bytes_in_flight > inflight_with_headroom) {
    // Stay in PROBE_DOWN.
    return;
  }

  // Transition to PROBE_CRUISE iff we've drained to target.
  QUIC_DVLOG(3) << sender_ << " Checking if drained to target. bytes_in_flight:"
                << bytes_in_flight << ", bdp:" << bdp;
  if (bytes_in_flight < bdp) {
    EnterProbeCruise(congestion_event.event_time);
  }
}

Bbr2ProbeBwMode::AdaptUpperBoundsResult Bbr2ProbeBwMode::MaybeAdaptUpperBounds(
    const Bbr2CongestionEvent& congestion_event) {
  const QuicSendTimeState& send_state = congestion_event.last_packet_send_state;
  if (!send_state.is_valid) {
    QUIC_DVLOG(3) << sender_ << " " << cycle_.phase
                  << ": NOT_ADAPTED_INVALID_SAMPLE";
    return NOT_ADAPTED_INVALID_SAMPLE;
  }

  QuicByteCount inflight_at_send = BytesInFlight(send_state);
  if (Params().use_bytes_delivered_for_inflight_hi) {
    if (congestion_event.last_packet_send_state.total_bytes_acked <=
        model_->total_bytes_acked()) {
      inflight_at_send =
          model_->total_bytes_acked() -
          congestion_event.last_packet_send_state.total_bytes_acked;
    } else {
      QUIC_BUG << "Total_bytes_acked(" << model_->total_bytes_acked()
               << ") < send_state.total_bytes_acked("
               << congestion_event.last_packet_send_state.total_bytes_acked
               << ")";
    }
  }

  if (model_->IsInflightTooHighWithEcn(congestion_event,
                                       Params().probe_bw_full_loss_count,
                                       sender_->GetBytesEcnInRounds())) {
    if (cycle_.is_sample_from_probing) {
      cycle_.is_sample_from_probing = false;

      if (!send_state.is_app_limited ||
          Params().max_probe_up_queue_rounds > 0) {
        const QuicByteCount inflight_target =
            sender_->GetTargetBytesInflight() * (1.0 - Params().beta);
        if (inflight_at_send >= inflight_target) {
          // The new code does not change behavior.
          QUIC_CODE_COUNT(quic_bbr2_cut_inflight_hi_gradually_noop);
        } else {
          // The new code actually cuts inflight_hi slower than before.
          QUIC_CODE_COUNT(quic_bbr2_cut_inflight_hi_gradually_in_effect);
        }
        if (Params().limit_inflight_hi_by_max_delivered) {
          QuicByteCount new_inflight_hi =
              std::max(inflight_at_send, inflight_target);
          if (new_inflight_hi >= model_->max_bytes_delivered_in_round()) {
            QUIC_CODE_COUNT(quic_bbr2_cut_inflight_hi_max_delivered_noop);
          } else {
            QUIC_CODE_COUNT(quic_bbr2_cut_inflight_hi_max_delivered_in_effect);
            new_inflight_hi = model_->max_bytes_delivered_in_round();
          }
          QUIC_DVLOG(3) << sender_
                        << " Setting inflight_hi due to loss. new_inflight_hi:"
                        << new_inflight_hi
                        << ", inflight_at_send:" << inflight_at_send
                        << ", inflight_target:" << inflight_target
                        << ", max_bytes_delivered_in_round:"
                        << model_->max_bytes_delivered_in_round() << "  @ "
                        << congestion_event.event_time;
          model_->set_inflight_hi(new_inflight_hi);
        } else if (Params().limit_inflight_hi_by_cwnd) {
          const QuicByteCount cwnd_target =
              sender_->GetCongestionWindow() * (1.0 - Params().beta);
          if (inflight_at_send >= cwnd_target) {
            // The new code does not change behavior.
            QUIC_CODE_COUNT(quic_bbr2_cut_inflight_hi_cwnd_noop);
          } else {
            // The new code actually cuts inflight_hi slower than before.
            QUIC_CODE_COUNT(quic_bbr2_cut_inflight_hi_cwnd_in_effect);
          }
          model_->set_inflight_hi(std::max(inflight_at_send, cwnd_target));
        } else {
          model_->set_inflight_hi(std::max(inflight_at_send, inflight_target));
        }
      }

      QUIC_DVLOG(3) << sender_ << " " << cycle_.phase
                    << ": ADAPTED_PROBED_TOO_HIGH";
      return ADAPTED_PROBED_TOO_HIGH;
    }
    return ADAPTED_OK;
  }

  if (model_->inflight_hi() == model_->inflight_hi_default()) {
    QUIC_DVLOG(3) << sender_ << " " << cycle_.phase
                  << ": NOT_ADAPTED_INFLIGHT_HIGH_NOT_SET";
    return NOT_ADAPTED_INFLIGHT_HIGH_NOT_SET;
  }

  // Raise the upper bound for inflight.
  if (inflight_at_send > model_->inflight_hi()) {
    QUIC_DVLOG(3)
        << sender_ << " " << cycle_.phase
        << ": Adapting inflight_hi from inflight_at_send. inflight_at_send:"
        << inflight_at_send << ", old inflight_hi:" << model_->inflight_hi();
    model_->set_inflight_hi(inflight_at_send);
  }

  return ADAPTED_OK;
}

bool Bbr2ProbeBwMode::IsTimeToProbeBandwidth(
    const Bbr2CongestionEvent& congestion_event) const {
  if (sender_->EnablePlusProbeBwPhases() && sender_->ShouldStartProbeOnRound()) {
    return true;
  }

  if (HasCycleLasted(cycle_.probe_wait_time, congestion_event)) {
    return true;
  }

  if (IsTimeToProbeForRenoCoexistence(1.0, congestion_event)) {
    ++sender_->connection_stats_->bbr_num_short_cycles_for_reno_coexistence;
    return true;
  }
  return false;
}

// QUIC only. Used to prevent a Bbr2 flow from staying in PROBE_DOWN for too
// long, as seen in some multi-sender simulator tests.
bool Bbr2ProbeBwMode::HasStayedLongEnoughInProbeDown(
    const Bbr2CongestionEvent& congestion_event) const {
  // Stay in PROBE_DOWN for at most the time of a min rtt, as it is done in
  // BBRv1.
  // TODO(wub): Consider exit after a full round instead, which typically
  // indicates most(if not all) packets sent during PROBE_UP have been acked.
  return HasPhaseLasted(model_->MinRtt(), congestion_event);
}

bool Bbr2ProbeBwMode::HasCycleLasted(
    TimeDelta duration,
    const Bbr2CongestionEvent& congestion_event) const {
  bool result =
      (congestion_event.event_time - cycle_.cycle_start_time) > duration;
  QUIC_DVLOG(3) << sender_ << " " << cycle_.phase
                << ": HasCycleLasted=" << result << ". elapsed:"
                << (congestion_event.event_time - cycle_.cycle_start_time)
                << ", duration:" << duration;
  return result;
}

bool Bbr2ProbeBwMode::HasPhaseLasted(
    TimeDelta duration,
    const Bbr2CongestionEvent& congestion_event) const {
  bool result =
      (congestion_event.event_time - cycle_.phase_start_time) > duration;
  QUIC_DVLOG(3) << sender_ << " " << cycle_.phase
                << ": HasPhaseLasted=" << result << ". elapsed:"
                << (congestion_event.event_time - cycle_.phase_start_time)
                << ", duration:" << duration;
  return result;
}

bool Bbr2ProbeBwMode::IsTimeToProbeForRenoCoexistence(
    double probe_wait_fraction,
    const Bbr2CongestionEvent& /*congestion_event*/) const {
  if (!Params().enable_reno_coexistence) {
    return false;
  }

  uint64_t rounds = Params().probe_bw_probe_max_rounds;
  if (Params().probe_bw_probe_reno_gain > 0.0) {
    QuicByteCount target_bytes_inflight = sender_->GetTargetBytesInflight();
    uint64_t reno_rounds = Params().probe_bw_probe_reno_gain *
                           target_bytes_inflight / kDefaultTCPMSS;
    rounds = std::min(rounds, reno_rounds);
  }
  bool result = cycle_.rounds_since_probe >= (rounds * probe_wait_fraction);
  QUIC_DVLOG(3) << sender_ << " " << cycle_.phase
                << ": IsTimeToProbeForRenoCoexistence=" << result
                << ". rounds_since_probe:" << cycle_.rounds_since_probe
                << ", rounds:" << rounds
                << ", probe_wait_fraction:" << probe_wait_fraction;
  return result;
}

void Bbr2ProbeBwMode::RaiseInflightHighSlope() {
  DCHECK_EQ(cycle_.phase, CyclePhase::PROBE_UP);
  uint64_t growth_this_round = 1 << cycle_.probe_up_rounds;
  // The number 30 below means |growth_this_round| is capped at 1G and the lower
  // bound of |probe_up_bytes| is (practically) 1 mss, at this speed inflight_hi
  // grows by approximately 1 packet per packet acked.
  cycle_.probe_up_rounds = std::min<uint64_t>(cycle_.probe_up_rounds + 1, 30);
  uint64_t probe_up_bytes = sender_->GetCongestionWindow() / growth_this_round;
  cycle_.probe_up_bytes =
      std::max<QuicByteCount>(probe_up_bytes, kDefaultTCPMSS);
  QUIC_DVLOG(3) << sender_ << " Rasing inflight_hi slope. probe_up_rounds:"
                << cycle_.probe_up_rounds
                << ", probe_up_bytes:" << cycle_.probe_up_bytes;
}

void Bbr2ProbeBwMode::ProbeInflightHighUpward(
    const Bbr2CongestionEvent& congestion_event) {
  DCHECK_EQ(cycle_.phase, CyclePhase::PROBE_UP);
  if (Params().probe_up_ignore_inflight_hi) {
    // When inflight_hi is disabled in PROBE_UP, it increases when the number of
    // bytes delivered in a round is larger than inflight_hi.
    return;
  }

  if (Params().probe_up_simplify_inflight_hi) {
    // Raise inflight_hi exponentially if it was utilized this round.
    cycle_.probe_up_acked += congestion_event.bytes_acked;
    if (!congestion_event.end_of_round_trip) {
      return;
    }
    if (!model_->inflight_hi_limited_in_round() ||
        model_->loss_events_in_round() > 0) {
      cycle_.probe_up_acked = 0;
      return;
    }
  } else {
    if (congestion_event.prior_bytes_in_flight < congestion_event.prior_cwnd) {
      QUIC_DVLOG(3) << sender_
                    << " Raising inflight_hi early return: Not cwnd limited.";
      // Not fully utilizing cwnd, so can't safely grow.
      return;
    }

    if (congestion_event.prior_cwnd < model_->inflight_hi()) {
      QUIC_DVLOG(3)
          << sender_
          << " Raising inflight_hi early return: inflight_hi not fully used.";
      // Not fully using inflight_hi, so don't grow it.
      return;
    }

    // Increase inflight_hi by the number of probe_up_bytes within
    // probe_up_acked.
    cycle_.probe_up_acked += congestion_event.bytes_acked;
  }

  if (cycle_.probe_up_acked >= cycle_.probe_up_bytes) {
    uint64_t delta = cycle_.probe_up_acked / cycle_.probe_up_bytes;
    cycle_.probe_up_acked -= delta * cycle_.probe_up_bytes;
    QuicByteCount new_inflight_hi =
        model_->inflight_hi() + delta * kDefaultTCPMSS;
    if (new_inflight_hi > model_->inflight_hi()) {
      QUIC_DVLOG(3) << sender_ << " Raising inflight_hi from "
                    << model_->inflight_hi() << " to " << new_inflight_hi
                    << ". probe_up_bytes:" << cycle_.probe_up_bytes
                    << ", delta:" << delta
                    << ", (new)probe_up_acked:" << cycle_.probe_up_acked;

      model_->set_inflight_hi(new_inflight_hi);
    } else {
      QUIC_BUG << "Not growing inflight_hi due to wrap around. Old value:"
               << model_->inflight_hi() << ", new value:" << new_inflight_hi;
    }
  }

  if (congestion_event.end_of_round_trip) {
    RaiseInflightHighSlope();
  }
}

void Bbr2ProbeBwMode::UpdateProbeCruise(
    const Bbr2CongestionEvent& congestion_event) {
  DCHECK_EQ(cycle_.phase, CyclePhase::PROBE_CRUISE);
  MaybeAdaptUpperBounds(congestion_event);
  DCHECK(!cycle_.is_sample_from_probing);

  if (congestion_event.end_of_round_trip &&
      sender_->ShouldAdvanceMaxBandwidthFilterOnRoundStart(cycle_.phase)) {
    model_->AdvanceMaxBandwidthFilter();
    cycle_.has_advanced_max_bw = true;
    sender_->OnMaxBandwidthFilterAdvanced(cycle_.phase);
  }

  if (IsTimeToProbeBandwidth(congestion_event) &&
      !sender_->ShouldDelayProbeBwCruiseExit(congestion_event.event_time)) {
    EnterProbeRefill(/*probe_up_rounds=*/0, congestion_event.event_time);
    return;
  }
}

void Bbr2ProbeBwMode::UpdateProbeRefill(
    const Bbr2CongestionEvent& congestion_event) {
  DCHECK_EQ(cycle_.phase, CyclePhase::PROBE_REFILL);
  MaybeAdaptUpperBounds(congestion_event);
  DCHECK(!cycle_.is_sample_from_probing);

  if (cycle_.rounds_in_phase > 0 && congestion_event.end_of_round_trip) {
    if (sender_->EnablePlusProbeBwPhases()) {
      EnterProbePreUp(congestion_event.event_time);
    } else {
      EnterProbeUp(congestion_event.event_time);
    }
    return;
  }
}

void Bbr2ProbeBwMode::UpdateProbePreUp(
    const Bbr2CongestionEvent& congestion_event) {
  DCHECK_EQ(cycle_.phase, CyclePhase::PROBE_PRE_UP);
  MaybeAdaptUpperBounds(congestion_event);
  DCHECK(!cycle_.is_sample_from_probing);

  if (cycle_.rounds_in_phase > 0 && congestion_event.end_of_round_trip) {
    EnterProbeGuard(congestion_event.event_time);
  }
}

void Bbr2ProbeBwMode::UpdateProbeGuard(
    const Bbr2CongestionEvent& congestion_event) {
  DCHECK_EQ(cycle_.phase, CyclePhase::PROBE_GUARD);
  MaybeAdaptUpperBounds(congestion_event);
  DCHECK(!cycle_.is_sample_from_probing);

  if (cycle_.rounds_in_phase > 0 && congestion_event.end_of_round_trip) {
    if (sender_->ShouldEnterProbeUpFromGuard()) {
      EnterProbeUp(congestion_event.event_time);
    } else {
      EnterProbeDownSlightly(congestion_event.event_time);
    }
  }
}

void Bbr2ProbeBwMode::UpdateProbeUp(
    QuicByteCount prior_in_flight,
    const Bbr2CongestionEvent& congestion_event) {
  DCHECK_EQ(cycle_.phase, CyclePhase::PROBE_UP);
  if (MaybeAdaptUpperBounds(congestion_event) == ADAPTED_PROBED_TOO_HIGH) {
    if (!sender_->ExperimentalForcedProbeUpExitAllowed(
            congestion_event.event_time)) {
      return;
    }
    EnterProbeDown(/*probed_too_high=*/true, /*stopped_risky_probe=*/false,
                   congestion_event.event_time);
    return;
  }

  // TODO(wub): Consider exit PROBE_UP after a certain number(e.g. 64) of RTTs.

  ProbeInflightHighUpward(congestion_event);

  bool is_risky = false;
  bool is_queuing = false;
  if (last_cycle_probed_too_high_ && prior_in_flight >= model_->inflight_hi()) {
    is_risky = true;
    QUIC_DVLOG(3) << sender_
                  << " Probe is too risky. last_cycle_probed_too_high_:"
                  << last_cycle_probed_too_high_
                  << ", prior_in_flight:" << prior_in_flight
                  << ", inflight_hi:" << model_->inflight_hi();
    // TCP uses min_rtt instead of a full round:
    //   HasPhaseLasted(model_->MinRtt(), congestion_event)
  } else if (cycle_.rounds_in_phase > 0) {
    if (Params().max_probe_up_queue_rounds > 0) {
      if (congestion_event.end_of_round_trip) {
        model_->CheckPersistentQueue(congestion_event,
                                     Params().full_bw_threshold);
        if (model_->rounds_with_queueing() >=
            Params().max_probe_up_queue_rounds) {
          is_queuing = true;
        }
      }
    } else {
      QuicByteCount queuing_threshold_extra_bytes =
          model_->QueueingThresholdExtraBytes();
      if (Params().add_ack_height_to_queueing_threshold) {
        queuing_threshold_extra_bytes += model_->MaxAckHeight();
      }
      QuicByteCount queuing_threshold =
          (Params().full_bw_threshold * model_->BDP()) +
          queuing_threshold_extra_bytes;
      is_queuing = congestion_event.bytes_in_flight >= queuing_threshold;
      QUIC_DVLOG(3) << sender_
                    << " Checking if building up a queue. prior_in_flight:"
                    << prior_in_flight
                    << ", post_in_flight:" << congestion_event.bytes_in_flight
                    << ", threshold:" << queuing_threshold
                    << ", is_queuing:" << is_queuing
                    << ", max_bw:" << model_->MaxBandwidth()
                    << ", min_rtt:" << model_->MinRtt();
    }
  }

  if (is_risky || is_queuing) {
    if (!sender_->ExperimentalForcedProbeUpExitAllowed(
            congestion_event.event_time)) {
      return;
    }
    if (!is_risky && is_queuing &&
        sender_->ShouldDelayProbeUpExit(congestion_event.event_time)) {
      QUIC_DVLOG(3) << sender_
                    << " Delaying PROBE_UP exit due to sender-specific minimum"
                    << " UP duration gate.";
      return;
    }
    if (!is_risky && is_queuing && sender_->EnablePlusProbeBwPhases()) {
      EnterProbePostUp(congestion_event.event_time);
      return;
    }
    EnterProbeDown(/*probed_too_high=*/false, /*stopped_risky_probe=*/is_risky,
                   congestion_event.event_time);
    return;
  }

  // A sender may deliberately pace PROBE_UP from a connection-level
  // baseline below native MaxBw.  Such a sender cannot build the native BDP
  // queue that normally ends this phase, so let it retain the normal UP gain
  // for one complete packet-timed round and then continue the native cycle.
  if (cycle_.rounds_in_phase > 0 &&
      congestion_event.end_of_round_trip &&
      sender_->ShouldExitProbeUpAfterRound()) {
    EnterProbeDown(/*probed_too_high=*/false,
                   /*stopped_risky_probe=*/false,
                   congestion_event.event_time);
  }
}

void Bbr2ProbeBwMode::UpdateProbePostUp(
    const Bbr2CongestionEvent& congestion_event) {
  DCHECK_EQ(cycle_.phase, CyclePhase::PROBE_POST_UP);
  if (MaybeAdaptUpperBounds(congestion_event) == ADAPTED_PROBED_TOO_HIGH) {
    EnterProbeDown(/*probed_too_high=*/true, /*stopped_risky_probe=*/false,
                   congestion_event.event_time);
    return;
  }

  if (cycle_.rounds_in_phase > 0 && congestion_event.end_of_round_trip) {
    if (sender_->ShouldProbeAgainFromPostUp()) {
      EnterProbeUp(congestion_event.event_time);
    } else {
      EnterProbeDown(/*probed_too_high=*/false, /*stopped_risky_probe=*/false,
                     congestion_event.event_time);
    }
  }
}

void Bbr2ProbeBwMode::EnterProbeDown(bool probed_too_high,
                                     bool stopped_risky_probe,
                                     QuicTime now) {
  QUIC_DVLOG(2) << sender_ << " Phase change: " << cycle_.phase << " ==> "
                << CyclePhase::PROBE_DOWN << " after "
                << now - cycle_.phase_start_time << ", or "
                << cycle_.rounds_in_phase
                << " rounds. probed_too_high:" << probed_too_high
                << ", stopped_risky_probe:" << stopped_risky_probe << "  @ "
                << now;
  last_cycle_probed_too_high_ = probed_too_high;
  last_cycle_stopped_risky_probe_ = stopped_risky_probe;

  cycle_.cycle_start_time = now;
  cycle_.phase = CyclePhase::PROBE_DOWN;
  cycle_.rounds_in_phase = 0;
  cycle_.phase_start_time = now;
  ++sender_->connection_stats_->bbr_num_cycles;
  if (Params().bw_lo_mode_ != Bbr2Params::DEFAULT) {
    // Clear bandwidth_lo if it was set in PROBE_UP, because losses in PROBE_UP
    // should not permanently change bandwidth_lo.
    model_->clear_bandwidth_lo();
  }

  // Pick probe wait time.
  cycle_.rounds_since_probe =
      sender_->RandomUint64(Params().probe_bw_max_probe_rand_rounds);
  cycle_.probe_wait_time =
      Params().probe_bw_probe_base_duration +
      TimeDelta::FromMicroseconds(sender_->RandomUint64(
          Params().probe_bw_probe_max_rand_duration.ToMicroseconds()));

  cycle_.probe_up_bytes = std::numeric_limits<QuicByteCount>::max();
  cycle_.probe_up_app_limited_since_inflight_hi_limited_ = false;
  cycle_.has_advanced_max_bw = false;
  model_->RestartRoundEarly();
  sender_->OnProbeBwPhaseEntered(cycle_.phase, now);
}

void Bbr2ProbeBwMode::EnterProbeCruise(QuicTime now) {
  if (cycle_.phase == CyclePhase::PROBE_DOWN) {
    ExitProbeDown();
  }
  QUIC_DVLOG(2) << sender_ << " Phase change: " << cycle_.phase << " ==> "
                << CyclePhase::PROBE_CRUISE << " after "
                << now - cycle_.phase_start_time << ", or "
                << cycle_.rounds_in_phase << " rounds.  @ " << now;

  model_->cap_inflight_lo(model_->inflight_hi());
  cycle_.phase = CyclePhase::PROBE_CRUISE;
  cycle_.rounds_in_phase = 0;
  cycle_.phase_start_time = now;
  cycle_.is_sample_from_probing = false;
  sender_->OnProbeBwPhaseEntered(cycle_.phase, now);
}

void Bbr2ProbeBwMode::EnterProbeRefill(uint64_t probe_up_rounds, QuicTime now) {
  if (cycle_.phase == CyclePhase::PROBE_DOWN) {
    ExitProbeDown();
  }
  QUIC_DVLOG(2) << sender_ << " Phase change: " << cycle_.phase << " ==> "
                << CyclePhase::PROBE_REFILL << " after "
                << now - cycle_.phase_start_time << ", or "
                << cycle_.rounds_in_phase
                << " rounds. probe_up_rounds:" << probe_up_rounds << "  @ "
                << now;
  cycle_.phase = CyclePhase::PROBE_REFILL;
  cycle_.rounds_in_phase = 0;
  cycle_.phase_start_time = now;
  cycle_.is_sample_from_probing = false;
  last_cycle_stopped_risky_probe_ = false;

  model_->clear_bandwidth_lo();
  model_->clear_inflight_lo();
  cycle_.probe_up_rounds = probe_up_rounds;
  cycle_.probe_up_acked = 0;
  model_->RestartRoundEarly();
  sender_->OnProbeBwPhaseEntered(cycle_.phase, now);
}

void Bbr2ProbeBwMode::EnterProbePreUp(QuicTime now) {
  DCHECK_EQ(cycle_.phase, CyclePhase::PROBE_REFILL);
  QUIC_DVLOG(2) << sender_ << " Phase change: " << cycle_.phase << " ==> "
                << CyclePhase::PROBE_PRE_UP << " after "
                << now - cycle_.phase_start_time << ", or "
                << cycle_.rounds_in_phase << " rounds.  @ " << now;
  cycle_.phase = CyclePhase::PROBE_PRE_UP;
  cycle_.rounds_in_phase = 0;
  cycle_.phase_start_time = now;
  cycle_.is_sample_from_probing = false;
  model_->RestartRoundEarly();
  sender_->OnProbeBwPhaseEntered(cycle_.phase, now);
}

void Bbr2ProbeBwMode::EnterProbeGuard(QuicTime now) {
  DCHECK_EQ(cycle_.phase, CyclePhase::PROBE_PRE_UP);
  QUIC_DVLOG(2) << sender_ << " Phase change: " << cycle_.phase << " ==> "
                << CyclePhase::PROBE_GUARD << " after "
                << now - cycle_.phase_start_time << ", or "
                << cycle_.rounds_in_phase << " rounds.  @ " << now;
  cycle_.phase = CyclePhase::PROBE_GUARD;
  cycle_.rounds_in_phase = 0;
  cycle_.phase_start_time = now;
  cycle_.is_sample_from_probing = false;
  model_->RestartRoundEarly();
  sender_->OnProbeBwPhaseEntered(cycle_.phase, now);
}

void Bbr2ProbeBwMode::EnterProbeUp(QuicTime now) {
  DCHECK(cycle_.phase == CyclePhase::PROBE_REFILL ||
         cycle_.phase == CyclePhase::PROBE_GUARD ||
         cycle_.phase == CyclePhase::PROBE_POST_UP);
  QUIC_DVLOG(2) << sender_ << " Phase change: " << cycle_.phase << " ==> "
                << CyclePhase::PROBE_UP << " after "
                << now - cycle_.phase_start_time << ", or "
                << cycle_.rounds_in_phase << " rounds.  @ " << now;
  cycle_.phase = CyclePhase::PROBE_UP;
  cycle_.rounds_in_phase = 0;
  cycle_.phase_start_time = now;
  cycle_.is_sample_from_probing = true;
  RaiseInflightHighSlope();

  model_->RestartRoundEarly();
  sender_->OnProbeBwPhaseEntered(cycle_.phase, now);
}

void Bbr2ProbeBwMode::ForceEnterProbeUp(QuicTime now) {
  // Minimal experimental hook for controlled probe-order experiments. It
  // reuses normal PROBE_UP state initialization without waiting for REFILL.
  QUIC_DVLOG(2) << sender_ << " Forced phase change: " << cycle_.phase
                << " ==> " << CyclePhase::PROBE_UP << " after "
                << now - cycle_.phase_start_time << ", or "
                << cycle_.rounds_in_phase << " rounds.  @ " << now;
  cycle_.phase = CyclePhase::PROBE_UP;
  cycle_.rounds_in_phase = 0;
  cycle_.phase_start_time = now;
  cycle_.is_sample_from_probing = true;
  cycle_.probe_up_rounds = 0;
  cycle_.probe_up_acked = 0;
  RaiseInflightHighSlope();

  model_->clear_bandwidth_lo();
  model_->clear_inflight_lo();
  model_->RestartRoundEarly();
  sender_->MarkExperimentalForcedProbeUpStarted(now);
  sender_->OnProbeBwPhaseEntered(cycle_.phase, now);
}

void Bbr2ProbeBwMode::EnterProbePostUp(QuicTime now) {
  DCHECK_EQ(cycle_.phase, CyclePhase::PROBE_UP);
  QUIC_DVLOG(2) << sender_ << " Phase change: " << cycle_.phase << " ==> "
                << CyclePhase::PROBE_POST_UP << " after "
                << now - cycle_.phase_start_time << ", or "
                << cycle_.rounds_in_phase << " rounds.  @ " << now;
  cycle_.phase = CyclePhase::PROBE_POST_UP;
  cycle_.rounds_in_phase = 0;
  cycle_.phase_start_time = now;
  cycle_.is_sample_from_probing = true;
  model_->RestartRoundEarly();
  sender_->OnProbeBwPhaseEntered(cycle_.phase, now);
}

void Bbr2ProbeBwMode::UpdateProbeDownSlightly(
    QuicByteCount prior_in_flight,
    const Bbr2CongestionEvent& congestion_event) {
  DCHECK_EQ(cycle_.phase, CyclePhase::PROBE_DOWN_SLIGHTLY);

  if (congestion_event.end_of_round_trip &&
      sender_->ShouldAdvanceMaxBandwidthFilterOnRoundStart(cycle_.phase)) {
    model_->AdvanceMaxBandwidthFilter();
    cycle_.has_advanced_max_bw = true;
    sender_->OnMaxBandwidthFilterAdvanced(cycle_.phase);
  }

  MaybeAdaptUpperBounds(congestion_event);

  if (IsTimeToProbeBandwidth(congestion_event)) {
    EnterProbeRefill(/*probe_up_rounds=*/0, congestion_event.event_time);
    return;
  }

  const QuicByteCount inflight_with_headroom =
      model_->inflight_hi_with_headroom();
  if (prior_in_flight > inflight_with_headroom) {
    return;
  }

  QuicByteCount bdp = model_->BDP();
  if (prior_in_flight < bdp || HasStayedLongEnoughInProbeDown(congestion_event)) {
    EnterProbeCruise(congestion_event.event_time);
  }
}

void Bbr2ProbeBwMode::EnterProbeDownSlightly(QuicTime now) {
  DCHECK_EQ(cycle_.phase, CyclePhase::PROBE_GUARD);
  QUIC_DVLOG(2) << sender_ << " Phase change: " << cycle_.phase << " ==> "
                << CyclePhase::PROBE_DOWN_SLIGHTLY << " after "
                << now - cycle_.phase_start_time << ", or "
                << cycle_.rounds_in_phase << " rounds.  @ " << now;
  last_cycle_probed_too_high_ = false;
  last_cycle_stopped_risky_probe_ = false;

  cycle_.cycle_start_time = now;
  cycle_.phase = CyclePhase::PROBE_DOWN_SLIGHTLY;
  cycle_.rounds_in_phase = 0;
  cycle_.phase_start_time = now;
  cycle_.probe_up_bytes = std::numeric_limits<QuicByteCount>::max();
  cycle_.has_advanced_max_bw = false;
  cycle_.is_sample_from_probing = false;
  model_->RestartRoundEarly();
  sender_->OnProbeBwPhaseEntered(cycle_.phase, now);
}

void Bbr2ProbeBwMode::ExitProbeDown() {
  DCHECK_EQ(cycle_.phase, CyclePhase::PROBE_DOWN);
  if (!cycle_.has_advanced_max_bw) {
    QUIC_DVLOG(2) << sender_ << " Advancing max bw filter at end of cycle.";
    model_->AdvanceMaxBandwidthFilter();
    cycle_.has_advanced_max_bw = true;
  }
}

// static
const char* Bbr2ProbeBwMode::CyclePhaseToString(CyclePhase phase) {
  switch (phase) {
    case CyclePhase::PROBE_NOT_STARTED:
      return "PROBE_NOT_STARTED";
    case CyclePhase::PROBE_UP:
      return "PROBE_UP";
    case CyclePhase::PROBE_DOWN:
      return "PROBE_DOWN";
    case CyclePhase::PROBE_CRUISE:
      return "PROBE_CRUISE";
    case CyclePhase::PROBE_REFILL:
      return "PROBE_REFILL";
    case CyclePhase::PROBE_PRE_UP:
      return "PROBE_PRE_UP";
    case CyclePhase::PROBE_GUARD:
      return "PROBE_GUARD";
    case CyclePhase::PROBE_POST_UP:
      return "PROBE_POST_UP";
    case CyclePhase::PROBE_DOWN_SLIGHTLY:
      return "PROBE_DOWN_SLIGHTLY";
    default:
      break;
  }
  return "<Invalid CyclePhase>";
}

std::ostream& operator<<(std::ostream& os,
                         const Bbr2ProbeBwMode::CyclePhase phase) {
  return os << Bbr2ProbeBwMode::CyclePhaseToString(phase);
}

Bbr2ProbeBwMode::DebugState Bbr2ProbeBwMode::ExportDebugState() const {
  DebugState s;
  s.phase = cycle_.phase;
  s.cycle_start_time = cycle_.cycle_start_time;
  s.phase_start_time = cycle_.phase_start_time;
  return s;
}

std::ostream& operator<<(std::ostream& os,
                         const Bbr2ProbeBwMode::DebugState& state) {
  os << "[PROBE_BW] phase: " << state.phase << "\n";
  os << "[PROBE_BW] cycle_start_time: " << state.cycle_start_time << "\n";
  os << "[PROBE_BW] phase_start_time: " << state.phase_start_time << "\n";
  return os;
}

const Bbr2Params& Bbr2ProbeBwMode::Params() const {
  return sender_->Params();
}

float Bbr2ProbeBwMode::PacingGainForPhase(
    Bbr2ProbeBwMode::CyclePhase phase) const {
  if (phase == Bbr2ProbeBwMode::CyclePhase::PROBE_UP) {
    return Params().probe_bw_probe_up_pacing_gain;
  }
  if (phase == Bbr2ProbeBwMode::CyclePhase::PROBE_DOWN) {
    return sender_->GetProbeBwPacingGain(
        phase, Params().probe_bw_probe_down_pacing_gain);
  }
  return sender_->GetProbeBwPacingGain(phase,
                                       Params().probe_bw_default_pacing_gain);
}

float Bbr2ProbeBwMode::CwndGainForPhase(
    Bbr2ProbeBwMode::CyclePhase phase) const {
  return sender_->GetProbeBwCwndGain(phase, Params().probe_bw_cwnd_gain);
}

}  // namespace quic
