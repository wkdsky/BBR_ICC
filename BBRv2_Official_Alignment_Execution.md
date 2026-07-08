# BBRv2 Official Alignment Execution Plan

## Baseline

- Local target: `NS3.27/src/dqc/model/thirdparty/congestion/quic_bbr2_*`
- Local dependents to preserve:
  - `freqccv4_sender.*`
  - `quic_bbr2plus_sender.*`
  - DQC/ns-3 sender integration and trace hooks
- Official reference: Google QUICHE `main`
- Reference commit: `f1420cae27a7b3209d754e2e1363ee2ba290ffbd`
- Reference path: `quiche/quic/core/congestion_control/bbr2_*`

## Alignment Policy

This is not a raw file replacement. The local BBRv2 tree contains research
extensions and ns-3/DQC integration that are not present in upstream QUICHE.
The alignment should update the base BBRv2 behavior toward upstream while
preserving local extension points.

Preserve local-only behavior unless it directly conflicts with official BBRv2:

- ECN accounting and response hooks.
- FreqCCv4 delivery-rate tracing and congestion-state accessors.
- Bbr2Plus phase hooks and custom PROBE_BW phases.
- DQC/ns-3 public interfaces and existing construction paths.
- Existing congestion-control type plumbing.

## Diff Groups

### 1. Public API and sender integration

Official QUICHE has newer sender APIs and connection-option plumbing:

- `SetFromConfig`
- `ApplyConnectionOptions`
- `AdjustNetworkParameters(const NetworkParams&)`
- `PopulateConnectionStats`
- `ReduceMemoryUsage`
- `HasGoodBandwidthEstimateForResumption`
- ECN-aware `OnCongestionEvent` inputs

Local DQC has different interfaces and local extensions:

- `enable_ecn`
- `enable_probe_rtt`
- local `CongestionControlType`
- local ECN update hook
- FreqCC/Bbr2Plus virtual hooks

Execution policy:

- Keep local public construction and DQC integration stable.
- Add official-compatible behavior where possible without requiring a broad
  DQC API rewrite.
- Keep local ECN and tracing APIs, but avoid letting them block upstream core
  algorithm updates.

### 2. Parameter defaults

High-impact defaults differ from upstream:

- Startup cwnd gain: local `2.885`, upstream derived default `2.0`.
- Drain cwnd gain: local `2.885`, upstream derived default `2.0`.
- PROBE_DOWN pacing gain: local `0.75`, upstream `0.91`.
- Local still has `probe_bw_probe_inflight_gain`; upstream uses
  `full_bw_threshold * BDP()` style queue thresholds.

Execution policy:

- Move base BBRv2 defaults toward upstream.
- Keep local ECN/FreqCC-only parameters as extensions.
- Remove or stop relying on stale upstream-removed parameters where practical.

### 3. Network model and round tracking

Upstream has newer tracking in `Bbr2NetworkModel`:

- full-bandwidth state owned by the model
- `min_bytes_in_flight_in_round_`
- `max_bytes_delivered_in_round_`
- `inflight_hi_limited_in_round_`
- lower-bound adaptation improvements
- explicit restart/new-round helpers

Execution policy:

- Port model-owned full-bandwidth and round-local state.
- Update lower-bound logic while preserving local ECN response.
- Keep local delivery-rate sample fields used by FreqCCv4.

### 4. STARTUP behavior

Upstream STARTUP changed around bandwidth-growth detection, persistent queues,
and loss exit.

Execution policy:

- Move full-bandwidth detection to the model.
- Port upstream STARTUP exit checks.
- Preserve local ECN startup exit as an additional local extension.

### 5. PROBE_BW behavior

Upstream PROBE_BW changed substantially:

- queue threshold logic no longer uses local `probe_bw_probe_inflight_gain`
- PROBE_UP has newer inflight_hi and max-delivered handling
- Reno coexistence and app-limited handling are more explicit

Local adds extra phases:

- `PROBE_PRE_UP`
- `PROBE_GUARD`
- `PROBE_POST_UP`
- `PROBE_DOWN_SLIGHTLY`

Execution policy:

- Port upstream core PROBE_UP/PROBE_DOWN/REFILL/CRUISE logic.
- Preserve local custom phases and virtual hooks.
- Keep local custom phase behavior isolated so base BBRv2 follows upstream
  unless a subclass explicitly overrides behavior.

### 6. PROBE_RTT and DRAIN

These files are mostly API/type drift plus minor model usage changes.

Execution policy:

- Update to upstream helper usage where compatible.
- Preserve local `enable_probe_rtt` behavior.

### 7. Tests and validation

Validation should be staged:

- Compile the DQC/ns-3 module after each coherent migration chunk.
- Run available BBRv2/FreqCC smoke workloads.
- Add focused regression checks if the tree has an existing suitable test
  harness.

Minimum smoke checks:

- BBRv2 sender compiles.
- FreqCCv4 still compiles.
- Bbr2Plus still compiles.
- Existing sample simulation using DQC sender starts successfully.

## Execution Checklist

- [x] Confirm local BBRv2 file set and current dirty files.
- [x] Capture official baseline and local diff groups.
- [x] Update parameter defaults toward upstream.
- [x] Port `Bbr2NetworkModel` round/full-bandwidth tracking.
- [x] Port STARTUP official behavior while preserving ECN exit.
- [x] Port lower-bound and excessive-loss logic while preserving ECN hooks.
- [x] Port PROBE_BW official behavior while preserving local custom phases.
- [x] Review DRAIN and PROBE_RTT for upstream-compatible helper usage.
- [x] Compile affected targets.
- [x] Run available smoke simulation or tests.
- [x] Record remaining intentional divergences.

## Intentional Divergence Log

Keep this list updated as implementation proceeds.

- Local namespaces/types remain DQC-specific instead of QUICHE-native.
- Local ECN extension remains present.
- Local FreqCCv4/Bbr2Plus extension hooks remain present.
- Local `enable_probe_rtt` switch remains present.
- Local congestion-control type plumbing remains present.
- Local DQC sender interface keeps
  `AdjustNetworkParameters(QuicBandwidth, TimeDelta, bool)` instead of the
  QUICHE `NetworkParams` API.
- Local custom PROBE_BW phases remain in the base enum because Bbr2Plus and
  FreqCCv4 depend on them.
- Local `probe_bw_probe_inflight_gain` is retained as a legacy parameter, but
  base PROBE_UP queue detection now uses upstream `full_bw_threshold * BDP()`.

## Verification Log

- `./waf build`: passed.
- `./waf --run "bbrv2 --sim_time=1"`: passed.
- `./waf --run "freqccv4_2flow_calibration --sim_time=1"`: passed.
- `./waf --run "bbrv2plus_4flow --sim_time=1"`: passed.
