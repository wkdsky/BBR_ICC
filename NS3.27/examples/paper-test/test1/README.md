# Test 1: Dynamic ProbeBW Comparison

This directory contains the reusable ns-3.27 experiment source, runner, and
analysis program for the dynamic congestion-control comparison.

## Run

From `NS3.27`, run the full experiment:

```bash
examples/paper-test/test1/run_test1.sh
```

The runner builds the selected example, executes one deterministic 1800-second
simulation for each requested controller, validates the data, and regenerates
the markdown report and figures. All paths passed to the executable are
relative to `NS3.27`; results are written under `results/test1`.

Use a seven-stage 210-second smoke run to check the build, controller factories,
CSV schema, and plotting path:

```bash
examples/paper-test/test1/run_test1.sh --smoke
```

After a successful build, `--skip-build` reuses the existing binary.
Independent controller runs can be executed in deterministic manifest order in
parallel, for example `examples/paper-test/test1/run_test1.sh --jobs=4`.
The default is one job.

## Scenario

- Dumbbell bottleneck: C=100 Mbit/s, base RTT=40 ms, DropTail buffer=40 BDP.
- Up to 16 long-lived DQC flows.
- Dynamic active-flow sequence: `2 -> 4 -> 8 -> 16 -> 8 -> 4 -> 2`.
- The 1800-second simulation is divided evenly across the seven stages.
- Controllers: `BBR-R`, `oBBR`, `BBRv2+`, `CUBIC`, `BBRv2-ideal`, `BBRv2`,
  `FBBR`, and `FBBR-ServiceFair`.

`BBRv2` has no scheduling suffix and is the original BBRv2 implementation.
`BBRv2-ideal` is the only experiment-specific variant. It still constructs the
same BBRv2 controller, but uses the BBRv2 ProbeBW-UP admission gate so active
flows enter UP in flow-ID order. The gate gives each stage one sequence
`1..N`, and holds later entries until the preceding UP exits. It enforces the
requested invariant: no other flow is in ProbeBW-UP while one flow is probing.

The other seven algorithms use their original factory type and control logic.
No no-ProbeRTT aliases are selected by this experiment. ProbeRTT remains a
recorded observation, rather than a cross-algorithm modification. The
`pre_all_other_cruise` event field is retained as a diagnostic; it is not a
stronger replacement for the requested no-overlapping-UP condition.

For focused controller diagnostics, the executable also accepts
`--flowPattern=steady1`, `steady8`, or `steady16`. These run one unchanged
active-flow stage for the requested count and are useful for separating a
single-flow waveform, steady contention, and a join/leave transition. They
are diagnostic modes and are not consumed by the canonical seven-stage
analysis script.

## Outputs

`results/test1/raw/manifest.csv` is the authoritative list of files for the
latest runner invocation. It lists each run's summary, stage metrics, flow
metrics, UP events, one-minute metrics, and metadata.

`results/test1/summary/` contains:

- `comparison_metrics.csv`: every controller at every dynamic stage.
- `overall_comparison_metrics.csv`: equal-stage summary per controller.
- `all_runs.csv`, `all_stage_metrics.csv`, `all_flow_metrics.csv`,
  `all_ideal_up_events.csv`, `all_minute_metrics.csv`, and
  `all_minute_flow_metrics.csv`: concatenated raw
  measurements.
- `METRICS.md`: complete field-level comparison metric catalog.
- `fig1a_data.csv` and `fig1b_data.csv`: time-indexed data used by the plots.
- `fig1b_stage_data.csv`, `fig1b_minute_data.csv`, and
  `fig1b_minute_flow_data.csv`: retained stage, aggregate, and per-flow
  one-minute inputs for Fig. 1(b).
- `validation.csv`: lifecycle and ideal-UP ordering checks.

`results/test1/RESULTS.md` is regenerated after every completed run. It records
the configuration, validation result, and the N=16 comparison table.

`results/test1/figures/` contains two figures:

- `fig1a_maxbw_theory_vs_measured.png`: per-UP BBRv2-ideal theoretical and
  measured MaxBw positioned at the actual simulation time of each event.
- `fig1b_excess_inflight_and_queue_delay.png`: a four-panel one-minute
  comparison for all eight algorithms: queue delay, aggregate goodput, mean
  active-flow goodput, and Jain fairness.

Both plots use simulation time on the horizontal axis: labels appear every two
minutes (`2`, `4`, `6`, ...) and unlabeled minor ticks appear every minute.

Fig. 1(a) is deliberately limited to `BBRv2-ideal`, because its sequential
ProbeBW-UP fluid recurrence is the theory being evaluated. Fig. 1(b) and the
comparison CSVs include every algorithm, including the two FBBR branches.
