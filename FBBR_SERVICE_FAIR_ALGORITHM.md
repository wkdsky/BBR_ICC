# FBBR-ServiceFair

`FBBR-ServiceFair` is an algorithm-level variant of `FBBR-hybirdv4`.
Its internal congestion-control type is `kFBBRServiceFair`.

## Relationship To V4

- `FBBR-hybirdv4` remains the unchanged service-consistent inflight-envelope
  baseline.
- `FBBR-ServiceFair` inherits that V4 envelope, target/base history,
  delivered-byte service history, pacing path, and V4 telemetry.
- The ServiceFair controller is active only for `kFBBRServiceFair`; it does
  not alter the V4 control path.

## Added Control

At most once per Cruise interval, ServiceFair uses a valid RTprop-sized,
non-app-limited delivered-service sample and a qdelay EWMA to update the
current Cruise baseline:

- Low qdelay applies additive increase with
  `alpha = 0.5 * 8 * MSS / RTprop` bps.
- Excess or rising qdelay applies multiplicative decrease with `beta = 0.995`.
- Regime III is bounded by the same beta cap and retains a service-rate floor.
- Cruise TrustedBw publication preserves the completed fairness correction,
  while a valid Regime II delivered-byte measurement remains authoritative.

The fairness trace is written as `flow<N>_service_fairness.csv` when FBBR load
tracing is enabled.

## Usage

```bash
./waf --run "fbbr_4flow --algo=FBBR-ServiceFair"
```

The algorithm is also accepted by `generic_p2p_switch_flows`.
