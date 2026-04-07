#!/usr/bin/env python3
"""
Analyze send rate and receive rate (RTT) oscillations in FreqCCv3 traces.
For short UP phases (30-80ms), we compare:
1. Send rate pattern (should show triangle wave oscillation)
2. RTT pattern (should show corresponding oscillation if receive rate varies)
"""

import numpy as np
import matplotlib.pyplot as plt
from scipy import signal
from scipy.fft import fft, fftfreq
import sys
import os

def load_trace(filename, cols=(0, 1)):
    """Load trace file with specified columns."""
    data = []
    with open(filename, 'r') as f:
        for line in f:
            parts = line.strip().split()
            if len(parts) >= max(cols) + 1:
                try:
                    row = [float(parts[c]) for c in cols]
                    data.append(row)
                except ValueError:
                    continue
    return np.array(data)

def extract_up_phases(bbrmode_file):
    """Extract UP phase time ranges from bbrmode trace."""
    up_phases = []
    in_up = False
    start = 0

    with open(bbrmode_file, 'r') as f:
        for line in f:
            parts = line.strip().split()
            if len(parts) >= 2:
                time = float(parts[0])
                mode = parts[1]

                if mode == 'probeBW_up' and not in_up:
                    start = time
                    in_up = True
                elif mode != 'probeBW_up' and in_up:
                    duration_ms = (time - start) * 1000
                    up_phases.append((start, time, duration_ms))
                    in_up = False

    return up_phases

def extract_data_in_range(data, t_start, t_end, extend_ratio=0.5):
    """Extract data points within time range, with optional extension."""
    duration = t_end - t_start
    extended_start = t_start - duration * extend_ratio
    extended_end = t_end + duration * extend_ratio

    mask = (data[:, 0] >= extended_start) & (data[:, 0] <= extended_end)
    return data[mask]

def compute_fft_freq(time_series, values, min_samples=10):
    """Compute FFT and find dominant frequency."""
    if len(values) < min_samples:
        return None, None, None

    # Interpolate to uniform sampling
    dt = np.median(np.diff(time_series))
    if dt <= 0 or np.isnan(dt):
        return None, None, None

    t_uniform = np.arange(time_series[0], time_series[-1], dt)
    if len(t_uniform) < min_samples:
        return None, None, None

    values_interp = np.interp(t_uniform, time_series, values)

    # Remove DC component
    values_detrend = values_interp - np.mean(values_interp)

    # Compute FFT
    n = len(values_detrend)
    yf = fft(values_detrend)
    xf = fftfreq(n, dt)

    # Only positive frequencies
    pos_mask = xf > 0
    xf_pos = xf[pos_mask]
    yf_pos = np.abs(yf[pos_mask])

    if len(yf_pos) == 0:
        return None, None, None

    # Find dominant frequency
    idx_max = np.argmax(yf_pos)
    dominant_freq = xf_pos[idx_max]

    return xf_pos, yf_pos, dominant_freq

def analyze_up_phase(up_idx, t_start, t_end, sendrate_data, rtt_data, expected_freq_hz=60):
    """Analyze a single UP phase."""
    duration_ms = (t_end - t_start) * 1000

    # Extract data
    sr_in_range = extract_data_in_range(sendrate_data, t_start, t_end, extend_ratio=0.2)
    rtt_in_range = extract_data_in_range(rtt_data, t_start, t_end, extend_ratio=0.2)

    print(f"\n=== UP Phase {up_idx}: {t_start:.4f}s - {t_end:.4f}s (duration={duration_ms:.1f}ms) ===")
    print(f"Expected cycles at {expected_freq_hz}Hz: {duration_ms/1000 * expected_freq_hz:.1f}")

    # Send rate analysis
    if len(sr_in_range) > 5:
        sr_mean = np.mean(sr_in_range[:, 1])
        sr_std = np.std(sr_in_range[:, 1])
        sr_range = np.max(sr_in_range[:, 1]) - np.min(sr_in_range[:, 1])
        print(f"Send rate: mean={sr_mean:.0f}kbps, std={sr_std:.0f}, range={sr_range:.0f}kbps")

        xf, yf, dom_freq = compute_fft_freq(sr_in_range[:, 0], sr_in_range[:, 1])
        if dom_freq is not None:
            print(f"Send rate dominant freq: {dom_freq:.1f}Hz")
    else:
        print(f"Send rate: insufficient samples ({len(sr_in_range)})")

    # RTT analysis
    if len(rtt_in_range) > 5:
        rtt_mean = np.mean(rtt_in_range[:, 1])
        rtt_std = np.std(rtt_in_range[:, 1])
        rtt_range = np.max(rtt_in_range[:, 1]) - np.min(rtt_in_range[:, 1])
        print(f"RTT: mean={rtt_mean:.1f}ms, std={rtt_std:.1f}ms, range={rtt_range:.1f}ms")

        xf, yf, dom_freq = compute_fft_freq(rtt_in_range[:, 0], rtt_in_range[:, 1])
        if dom_freq is not None:
            print(f"RTT dominant freq: {dom_freq:.1f}Hz")
    else:
        print(f"RTT: insufficient samples ({len(rtt_in_range)})")

    return sr_in_range, rtt_in_range

def plot_up_phase(up_idx, t_start, t_end, sr_data, rtt_data, output_dir):
    """Create plot for a single UP phase."""
    fig, axes = plt.subplots(2, 2, figsize=(14, 8))

    duration_ms = (t_end - t_start) * 1000
    fig.suptitle(f'UP Phase {up_idx}: {t_start:.4f}s - {t_end:.4f}s (duration={duration_ms:.1f}ms)')

    # Send rate time series
    ax1 = axes[0, 0]
    if len(sr_data) > 0:
        ax1.plot(sr_data[:, 0] * 1000, sr_data[:, 1], 'b.-', markersize=4)
        ax1.axvline(t_start * 1000, color='g', linestyle='--', label='UP start')
        ax1.axvline(t_end * 1000, color='r', linestyle='--', label='UP end')
    ax1.set_xlabel('Time (ms)')
    ax1.set_ylabel('Send Rate (kbps)')
    ax1.set_title('Send Rate')
    ax1.legend()
    ax1.grid(True)

    # RTT time series
    ax2 = axes[0, 1]
    if len(rtt_data) > 0:
        ax2.plot(rtt_data[:, 0] * 1000, rtt_data[:, 1], 'r.-', markersize=4)
        ax2.axvline(t_start * 1000, color='g', linestyle='--', label='UP start')
        ax2.axvline(t_end * 1000, color='r', linestyle='--', label='UP end')
    ax2.set_xlabel('Time (ms)')
    ax2.set_ylabel('RTT (ms)')
    ax2.set_title('RTT (reflects queuing delay)')
    ax2.legend()
    ax2.grid(True)

    # Send rate FFT
    ax3 = axes[1, 0]
    if len(sr_data) > 10:
        xf, yf, dom_freq = compute_fft_freq(sr_data[:, 0], sr_data[:, 1])
        if xf is not None:
            ax3.plot(xf, yf, 'b-')
            if dom_freq is not None:
                ax3.axvline(dom_freq, color='r', linestyle='--', label=f'Dominant: {dom_freq:.1f}Hz')
            ax3.axvline(60, color='g', linestyle=':', alpha=0.5, label='Expected 60Hz')
            ax3.legend()
    ax3.set_xlabel('Frequency (Hz)')
    ax3.set_ylabel('Amplitude')
    ax3.set_title('Send Rate FFT')
    ax3.set_xlim([0, 150])
    ax3.grid(True)

    # RTT FFT
    ax4 = axes[1, 1]
    if len(rtt_data) > 10:
        xf, yf, dom_freq = compute_fft_freq(rtt_data[:, 0], rtt_data[:, 1])
        if xf is not None:
            ax4.plot(xf, yf, 'r-')
            if dom_freq is not None:
                ax4.axvline(dom_freq, color='b', linestyle='--', label=f'Dominant: {dom_freq:.1f}Hz')
            ax4.axvline(60, color='g', linestyle=':', alpha=0.5, label='Expected 60Hz')
            ax4.legend()
    ax4.set_xlabel('Frequency (Hz)')
    ax4.set_ylabel('Amplitude')
    ax4.set_title('RTT FFT')
    ax4.set_xlim([0, 150])
    ax4.grid(True)

    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, f'up_phase_{up_idx}.png'), dpi=100)
    plt.close()

def main():
    trace_dir = '/home/wkd/BBR_ICC/NS3.27/traces'
    output_dir = '/home/wkd/BBR_ICC/NS3.27/analysis'

    os.makedirs(output_dir, exist_ok=True)

    # Analyze flow 1
    flow_id = 1
    prefix = f'freqccv3_4flow_{flow_id}'

    bbrmode_file = os.path.join(trace_dir, f'{prefix}_bbrmode.txt')
    sendrate_file = os.path.join(trace_dir, f'{prefix}_sendrate.txt')
    rtt_file = os.path.join(trace_dir, f'{prefix}_rtt.txt')

    print(f"Loading traces for flow {flow_id}...")

    # Load data
    sendrate_data = load_trace(sendrate_file, cols=(0, 1))  # time, rate_kbps
    rtt_data = load_trace(rtt_file, cols=(0, 2))  # time, rtt_ms (column 2, not 1)

    print(f"Loaded {len(sendrate_data)} sendrate samples, {len(rtt_data)} RTT samples")

    # Extract UP phases
    up_phases = extract_up_phases(bbrmode_file)
    print(f"\nFound {len(up_phases)} UP phases")

    # Analyze and plot each UP phase (first 10 and any > 50ms)
    analyzed = 0
    for i, (t_start, t_end, duration_ms) in enumerate(up_phases):
        if analyzed < 10 or duration_ms > 50:
            sr_data, rtt_data_phase = analyze_up_phase(
                i+1, t_start, t_end, sendrate_data, rtt_data, expected_freq_hz=60
            )
            plot_up_phase(i+1, t_start, t_end, sr_data, rtt_data_phase, output_dir)
            analyzed += 1

    print(f"\n\nPlots saved to {output_dir}/")
    print("\nSummary:")
    durations = [d for _, _, d in up_phases]
    print(f"  UP phase durations: min={min(durations):.1f}ms, max={max(durations):.1f}ms, mean={np.mean(durations):.1f}ms")
    print(f"  Expected cycles at 60Hz for mean duration: {np.mean(durations)/1000 * 60:.1f}")

if __name__ == '__main__':
    main()
