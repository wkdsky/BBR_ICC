#!/usr/bin/env python3
"""Summarize F-BBR command-to-emission diagnostics without command-gain gating."""

import argparse
import csv
import glob
import math
import os
import statistics
from collections import defaultdict


def number(row, key, default=0.0):
    try:
        value = float(row.get(key, default))
        return value if math.isfinite(value) else default
    except (TypeError, ValueError):
        return default


def percentile(values, q):
    values = sorted(values)
    if not values:
        return 0.0
    position = q * (len(values) - 1)
    left = int(math.floor(position))
    right = int(math.ceil(position))
    if left == right:
        return values[left]
    return values[left] + (position - left) * (values[right] - values[left])


def read_rows(patterns):
    rows = []
    for pattern in patterns:
        for path in sorted(glob.glob(pattern)):
            with open(path, newline="") as handle:
                for row in csv.DictReader(handle):
                    row["_path"] = path
                    rows.append(row)
    return rows


def summarize(blocks, packets):
    gains = [number(row, "realized_amplitude_ratio") for row in blocks]
    actual = [number(row, "actual_input_amplitude_ratio") for row in blocks]
    commanded = [number(row, "target_amplitude_ratio") for row in blocks]
    snr = [number(row, "input_carrier_snr") for row in blocks]
    conditions = [max(number(row, key) for key in
                      ("condition_input", "condition_delivery",
                       "condition_queue", "condition_utility"))
                  for row in blocks]
    lateness = [number(row, "emission_lateness_us") for row in packets]
    requested = [number(row, "pacer_requested_delay_us") for row in packets]
    positive_requested = [value for value in requested if value > 0]
    return {
        "decisions": len(blocks),
        "packets": len(packets),
        "median_command_to_emission_gain": statistics.median(gains) if gains else 0.0,
        "median_commanded_amplitude_ratio": statistics.median(commanded) if commanded else 0.0,
        "median_actual_amplitude_ratio": statistics.median(actual) if actual else 0.0,
        "median_actual_input_snr": statistics.median(snr) if snr else 0.0,
        "measurable_ratio": (sum(number(row, "C_meas") >= 0.45 and
                                 number(row, "input_carrier_snr") >= 2.0
                                 for row in blocks) / len(blocks)) if blocks else 0.0,
        "median_condition_number": statistics.median(conditions) if conditions else 0.0,
        "p50_emission_lateness_us": percentile(lateness, 0.50),
        "p95_emission_lateness_us": percentile(lateness, 0.95),
        "median_requested_delay_us": statistics.median(positive_requested)
        if positive_requested else 0.0,
        "cwnd_limited_packet_ratio": (sum(row.get("is_cwnd_limited") == "true"
                                           for row in packets) / len(packets))
        if packets else 0.0,
        "fine_grained_packet_ratio": (sum(row.get("fine_grained") == "true"
                                           for row in packets) / len(packets))
        if packets else 0.0,
        "multi_packet_token_ratio": (sum(number(row, "lumpy_tokens") > 0 or
                                          number(row, "burst_tokens") > 0
                                          for row in packets) / len(packets))
        if packets else 0.0,
    }


def write_grouped(path, keys, grouped_blocks, grouped_packets):
    fieldnames = list(keys) + list(summarize([], []).keys())
    with open(path, "w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        all_groups = sorted(set(grouped_blocks) | set(grouped_packets))
        for group in all_groups:
            prefix = dict(zip(keys, group if isinstance(group, tuple) else (group,)))
            prefix.update(summarize(grouped_blocks[group], grouped_packets[group]))
            writer.writerow(prefix)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("run_dirs", nargs="+")
    parser.add_argument("--output-dir", required=True)
    args = parser.parse_args()
    os.makedirs(args.output_dir, exist_ok=True)
    blocks = read_rows([os.path.join(path, "flow*_fbbr_event_windows.csv")
                        for path in args.run_dirs])
    packets = read_rows([os.path.join(path, "flow*_sent_audit.csv")
                         for path in args.run_dirs])

    block_by_flow, packet_by_flow = defaultdict(list), defaultdict(list)
    block_by_frequency, packet_by_frequency = defaultdict(list), defaultdict(list)
    block_by_rate, packet_by_rate = defaultdict(list), defaultdict(list)
    for row in blocks:
        flow = int(number(row, "flow_id"))
        frequency = round(number(row, "frequency_hz"), 3)
        rate_bucket = int(round(number(row, "baseline_before_bps") / 5e6) * 5_000_000)
        block_by_flow[(flow,)].append(row)
        block_by_frequency[(frequency,)].append(row)
        block_by_rate[(rate_bucket,)].append(row)
    for row in packets:
        flow = int(number(row, "flow_id"))
        packet_by_flow[(flow,)].append(row)
        # Packet rows do not duplicate carrier frequency; latency belongs to
        # the per-flow summary. Rate buckets retain scheduler diagnostics.
        rate_bucket = int(round(number(row, "search_baseline") / 5e6) * 5_000_000)
        packet_by_rate[(rate_bucket,)].append(row)

    write_grouped(os.path.join(args.output_dir, "actuator_transfer_by_flow.csv"),
                  ("flow_id",), block_by_flow, packet_by_flow)
    write_grouped(os.path.join(args.output_dir, "actuator_transfer_by_frequency.csv"),
                  ("frequency_hz",), block_by_frequency, packet_by_frequency)
    write_grouped(os.path.join(args.output_dir, "actuator_transfer_by_pacing_rate.csv"),
                  ("pacing_rate_bucket_bps",), block_by_rate, packet_by_rate)


if __name__ == "__main__":
    main()
