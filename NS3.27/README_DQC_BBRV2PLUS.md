# DQC BBRv2+ Migration Notes

The runtime host in this repository is DQC's existing QUIC `BBRv2`, not the
Linux TCP module.  `BBRv2plus` is therefore implemented as BBRv2+-specific
logic layered on the DQC BBRv2 packet model and state machine.

## Paper Mechanisms Covered

The implementation covers the functional mechanisms in Sections 4.2--4.4 of
Yang *et al.*, *BBRv2+: Enhancing BBRv2 for high mobility and high jitter
networks* (Computer Networks, 2022):

- Two-step `ProbeTry`: `PRE_UP` uses a 1.1 pacing gain for one RTT and
  `GUARD` uses 1.0 for one RTT.  The transition to `PROBE_UP` is gated by
  `MinRTTcurr <= gamma * MinRTTprev` (`gamma = 1.02` by default).
- Continuous probing: after `POST_UP`, BBRv2+ re-enters `PROBE_UP` whenever
  the probe RTT remains within the same gamma bound.  It is not artificially
  limited to one re-probe per cycle.
- Fast BtlBW expiry: during `PROBE_CRUISE`, `PROBE_DOWN`, and DQC's
  `PROBE_DOWN_SLIGHTLY` adaptation, a round whose minimum RTT exceeds
  `theta * RTprop` expires a max-bandwidth-filter slot (`theta = 1.10` by
  default).  A 25-round fallback prevents a stale filter from persisting.
- Dual ProbeBW mode: at the end of each Cruise interval, two consecutive
  Cruise RTT minima above `1.10 * RTprop` switch to native BBRv2 ProbeBW and
  restart Startup.  Four consecutive Cruise intervals at or below `1.05 * RTprop`
  restore the RTT-aware BBRv2+ ProbeBW state machine.
- Jitter-aware BDP compensation: the maximum RTT variation in the latest four
  RTT rounds is used when it exceeds `mu * RTprop` (`mu = 0.5`).  The resulting
  extra BDP is placed in BBRv2's real cwnd target, rather than only being added
  when `GetCongestionWindow()` is queried.
- BBRv2 loss threshold (`alpha`) and multiplicative reduction factor (`beta`)
  are exposed per sender.  The default remains BBRv2's `alpha=0.02`,
  `beta=0.30`; the paper's experiments commonly use `alpha=0.20`, selected to
  suit the bottleneck buffer.

## Configuration

All BBRv2+ paper parameters are available through `dqc::Bbr2PlusConfig` and
can be set for an individual DQC sender before the simulation starts:

```cpp
#include "ns3/dqc_sender.h"
#include "ns3/quic_bbr2plus_sender.h"

dqc::Bbr2PlusConfig config;
config.loss_threshold = 0.20f;
config.bandwidth_drop_rtt_multiplier = 1.10f;  // theta
config.probe_rtt_growth_multiplier = 1.02f;    // gamma
config.switch_to_bbr2_rtt_multiplier = 1.10f;  // lambda1
config.switch_to_bbr2plus_rtt_multiplier = 1.05f;  // lambda2
config.switch_to_bbr2_cruise_count = 2;        // eta1
config.switch_to_bbr2plus_cruise_count = 4;    // eta2
config.rtt_jitter_threshold_multiplier = 0.50f;  // mu
sender->ConfigureBbr2Plus(config);
```

`Bbr2PlusConfig` also exposes the probe interval, optional RTT error cap
(`0` keeps the paper's direct gamma comparison), jitter window, ProbeTry/Down
pacing gains, BDP compensation gains, and switches for RTT-aware probing and
RTT compensation.

## DQC-Host Adaptations

The behavior is semantically aligned with the paper, but it is not a bitwise
Linux-kernel reproduction:

- DQC's `latest_rtt`, `smoothed_rtt`, and `mean_deviation` replace Linux TCP
  internal RTT fields.
- The Linux ACK-phase implementation maps to DQC BBRv2's `PRE_UP`, `GUARD`,
  `POST_UP`, and `DOWN_SLIGHTLY` ProbeBW subphases.
- DQC's BBRv2 max-bandwidth filter and packet-timed round counter remain the
  underlying estimator and clock.
- Ordinary DQC `BBRv2` does not enable any of these extension hooks, so its
  default behavior is unchanged.

## Main Code Paths

- `src/dqc/model/thirdparty/congestion/quic_bbr2plus_sender.h`
- `src/dqc/model/thirdparty/congestion/quic_bbr2plus_sender.cc`
- `src/dqc/model/thirdparty/congestion/quic_bbr2_probe_bw.cc`
- `src/dqc/model/thirdparty/congestion/quic_bbr2_sender.{h,cc}`
- `src/dqc/model/dqc_sender.{h,cc}`
