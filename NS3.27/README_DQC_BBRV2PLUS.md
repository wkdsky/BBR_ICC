# DQC BBRv2plus Migration Notes

This repository does not treat Linux `tcp_bbr2plus.c` as the runtime host.
The host is the existing DQC `BBRv2` implementation in ns-3.

## Design Rule

`BBRv2plus` is implemented as a set of incremental mechanisms on top of DQC
`BBRv2`, not as a line-by-line Linux port.

That means:

- DQC `BBRv2` remains the baseline control law and packet model.
- Only `BBRv2plus`-specific mechanisms are added on top.
- Default DQC `BBRv2` behavior must remain unchanged.
- Linux `BBRv2plus` is used as a semantic reference for the incremental logic,
  not as a requirement for identical internal plumbing.

## Implemented Incremental Mechanisms

The current DQC `BBRv2plus` adds these mechanisms relative to DQC `BBRv2`:

- RTT-variance-based congestion-window compensation.
- ProbeBW subphases:
  `PRE_UP`, `GUARD`, `POST_UP`, `DOWN_SLIGHTLY`.
- RTT-gated transition from `GUARD` to `UP`.
- RTT-gated optional re-probe from `POST_UP`.
- Probe cycle waiting in round units for fast convergence.
- RTT-triggered advancing of the max-bandwidth filter during non-probing
  phases.

## Deliberate DQC-Host Adaptations

Some Linux details are intentionally adapted instead of copied verbatim:

- Linux-specific ACK-phase machinery is not fully reproduced.
- DQC RTT statistics (`latest_rtt`, `smoothed_rtt`, `mean_deviation`) are used
  instead of Linux TCP internal RTT fields.
- The `POST_UP` re-probe is bounded to at most one extra re-probe per ProbeBW
  cycle, to keep the algorithm closer to the DQC `BBRv2` host cycle shape.

These choices are intentional. They preserve the meaning of the `plus`
mechanisms while keeping the implementation defensible as a DQC-native
variant.

## Current Interpretation For Papers

The safest description is:

`DQC BBRv2plus`: a DQC-hosted `BBRv2plus` migration that preserves DQC `BBRv2`
as the baseline and adds `plus`-specific RTT-aware probing and cwnd
compensation mechanisms.

The safest description is not:

- "exact Linux BBRv2plus reproduction"
- "bit-for-bit kernel-equivalent implementation"

## Main Code Paths

- `src/dqc/model/thirdparty/congestion/quic_bbr2plus_sender.h`
- `src/dqc/model/thirdparty/congestion/quic_bbr2plus_sender.cc`
- `src/dqc/model/thirdparty/congestion/quic_bbr2_probe_bw.h`
- `src/dqc/model/thirdparty/congestion/quic_bbr2_probe_bw.cc`
- `src/dqc/model/thirdparty/congestion/quic_bbr2_sender.h`
- `src/dqc/model/thirdparty/congestion/quic_bbr2_sender.cc`
