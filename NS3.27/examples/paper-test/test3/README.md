# Test 3: Dynamic Propagation RTT

`DYN-RTT` is an independent companion experiment to Test 2. It keeps four
long-lived flows, a 100 Mbit/s bottleneck, a 1,000,000-byte DropTail
bottleneck queue, flow population, and all link data rates fixed. Only the
propagation delay on the eight symmetric access links changes.

The configured end-to-end propagation RTT has five 60-second stages:

| Time (s) | Propagation RTT (ms) | Access-link one-way delay (ms) |
| ---: | ---: | ---: |
| 0 | 40 | 5.0 |
| 60 | 120 | 25.0 |
| 120 | 30 | 2.5 |
| 180 | 80 | 15.0 |
| 240 | 40 | 5.0 |

The one-way bottleneck propagation delay remains 10 ms. The queue is two
initial-RTT BDPs, so it remains 1,000,000 bytes when RTT changes. This makes
the experiment isolate controller behavior under a changing propagation BDP;
it does not introduce capacity, queue, cross-traffic, or flow-count changes.

Run all controllers from `NS3.27`:

```bash
examples/paper-test/test3/run_test3.sh --jobs=4
```

Run only FBBR with the repository-wide configuration:

```bash
examples/paper-test/test3/run_test3.sh --skip-build --jobs=1 --algorithm=FBBR
```

FBBR always receives `examples/CCconfig/fbbr_default.conf`. Raw controller
outputs are under `results/test3/raw/DYN-RTT`, including a per-flow
`controller_trace.csv` with BBR state, native/FBBR RTprop, cwnd/inflight
limits, pacing, and FBBR BEQ state. The analyzer produces
`results/test3/RESULTS.md`, per-stage and transition CSV summaries, and
`results/test3/figures/dynamic_rtt_response.png` contains one Fig.1-style figure with left-right subplots for aggregate throughput and mirrored queue-delay/relative-RTprop-error stage statistics.

For each 60-second RTT stage, settled metrics exclude its first 15 seconds:
aggregate goodput, utilization, per-flow goodput, Jain fairness, queue delay,
inflight/expected-BDP, and BBR-style SRTT/MinRTT when available. The transition
table covers the excluded 15 seconds and records queue behavior and the first
full five-second interval to reach 90% of bottleneck capacity.
