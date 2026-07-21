#!/usr/bin/env python3
"""Plot open-loop P2P rate-ramp traces and infer load-state boundaries."""

from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from scipy import signal
from matplotlib.lines import Line2D
from matplotlib.patches import Rectangle


# =========================
# Paper-style figure config
# =========================

BOUNDARY_COLOR = "#212529"
OPTIMAL_AREA_COLOR = "#f08c00"
OPTIMAL_AREA_TEXT_COLOR = "#7a3f00"
SEND_RATE_COLOR = "#0b7285"
DRATE_COLOR = "#d9480f"
CUBIC_COLOR = "#495057"
SRTT_COLOR = "#5f3dc4"
OPTIMAL_AREA_START_S = 2.2
OPTIMAL_AREA_END_S = 2.4
OPTIMAL_AREA_LABEL_X_OFFSET_S = 0.16
OPTIMAL_AREA_LABEL = "Kleinrock's optimal\noperating area"

SUPTITLE_FS = 28
PHASE_LABEL_FS = 25
AX_TITLE_FS = 23
LABEL_FS = 23
TICK_FS = 21
LEGEND_FS = 20
SMALL_LEGEND_FS = 16

DATA_LW = 3.8
RATE_SERIES_LW = 5.0
SERVICE_LW = 3.4
BOUNDARY_LW = 4.2
GRID_LW = 1.05
SPINE_LW = 2.4

# Manual figure layout.  The first-row spectrum panels are horizontally
# aligned with the three time-domain regions separated by phase boundaries.
FIG_LEFT = 0.145
FIG_RIGHT = 0.985
FIG_WIDTH = FIG_RIGHT - FIG_LEFT
SPECTRUM_Y0 = 0.705
SPECTRUM_H = 0.135
RATE_Y0 = 0.430
RATE_H = 0.185
RTT_Y0 = 0.095
RTT_H = 0.185
PHASE_AXIS_GAP = 0.010
MIN_PHASE_AXIS_W = 0.090

plt.rcParams.update(
    {
        "font.size": LABEL_FS,
        "font.weight": "bold",
        "axes.labelweight": "bold",
        "axes.titleweight": "bold",
        "axes.linewidth": SPINE_LW,
        "xtick.major.width": SPINE_LW,
        "ytick.major.width": SPINE_LW,
        "xtick.major.size": 8.0,
        "ytick.major.size": 8.0,
        "figure.titleweight": "bold",
        "legend.framealpha": 0.94,
    }
)


def style_axis(ax, tick_size: int = TICK_FS) -> None:
    ax.tick_params(
        axis="both",
        labelsize=tick_size,
        width=SPINE_LW,
        length=8.0,
    )

    for label in ax.get_xticklabels() + ax.get_yticklabels():
        label.set_fontweight("bold")

    for spine in ax.spines.values():
        spine.set_linewidth(SPINE_LW)


def read_config(path: Path) -> dict[str, float]:
    config: dict[str, float] = {}

    if not path.exists():
        return config

    with path.open() as stream:
        for line in stream:
            line = line.strip()

            if not line or line.startswith("#"):
                continue

            parts = line.split()

            if len(parts) < 2:
                continue

            try:
                config[parts[0]] = float(parts[1])
            except ValueError:
                continue

    return config


def remove_existing_outputs(paths: list[Path]) -> None:
    for path in paths:
        try:
            path.unlink()
        except FileNotFoundError:
            pass


def resample_queue_to_rate_times(
    rate_t: np.ndarray,
    queue_t: np.ndarray,
    queue_bytes: np.ndarray,
) -> np.ndarray:
    idx = np.searchsorted(queue_t, rate_t, side="right") - 1
    return np.where(idx >= 0, queue_bytes[np.maximum(idx, 0)], 0.0)


def rolling_stat(values: np.ndarray, window: int, op) -> np.ndarray:
    output = np.full(values.shape, np.nan, dtype=float)

    for i in range(window - 1, len(values)):
        output[i] = op(values[i - window + 1 : i + 1])

    return output


def first_time(
    t: np.ndarray,
    values: np.ndarray,
    threshold: float,
    after: float | None = None,
) -> float | None:
    mask = values >= threshold
    mask &= np.isfinite(values)

    if after is not None:
        mask &= t >= after

    hits = np.flatnonzero(mask)

    if len(hits) == 0:
        return None

    return float(t[hits[0]])


def infer_load_boundaries(
    t: np.ndarray,
    queue_fraction: np.ndarray,
    modulation_freq_hz: float,
    empty_fraction: float,
    overload_fraction: float,
) -> tuple[float | None, float | None, int]:
    """Infer where the SRTT response leaves lower and reaches upper clipping."""

    dt = float(np.median(np.diff(t)))
    period_s = 1.0 / modulation_freq_hz if modulation_freq_hz > 0.0 else 0.2
    window = max(3, int(round(period_s / dt)))

    # Lower clipping is gone once an entire modulation cycle stays above the
    # propagation-RTT floor.  The rolling minimum confirms the full cycle;
    # move the reported boundary back to the start of that confirmed run.
    cycle_min = rolling_stat(queue_fraction, window, np.min)
    under_to_full_end = first_time(t, cycle_min, empty_fraction)
    under_to_full = (
        max(float(t[0]), under_to_full_end - (window - 1) * dt)
        if under_to_full_end is not None
        else None
    )

    # Upper clipping begins at the first sample that reaches the configured
    # near-capacity threshold; no cycle-centering offset is appropriate here.
    full_to_overload = first_time(
        t,
        queue_fraction,
        overload_fraction,
        under_to_full,
    )

    return under_to_full, full_to_overload, window


def data_x_to_fig_x(fig, ref_ax, x_value: float) -> float:
    display_x = ref_ax.transData.transform((x_value, 0.0))[0]
    fig_x = fig.transFigure.inverted().transform((display_x, 0.0))[0]
    return fig_x


def add_load_boundaries(
    fig,
    time_axes,
    all_axes,
    under_to_full: float | None,
    full_to_overload: float | None,
) -> None:
    """Draw figure-level vertical load-state boundaries across all rows."""

    fig.canvas.draw()

    y_bottom = min(ax.get_position().y0 for ax in all_axes)
    y_top = max(ax.get_position().y1 for ax in all_axes)

    for boundary_x in [under_to_full, full_to_overload]:
        if boundary_x is None:
            continue

        fig_x = data_x_to_fig_x(fig, time_axes[0], boundary_x)

        line = Line2D(
            [fig_x, fig_x],
            [y_bottom, y_top],
            transform=fig.transFigure,
            color=BOUNDARY_COLOR,
            linewidth=BOUNDARY_LW,
            linestyle="--",
            alpha=0.95,
            zorder=30,
            clip_on=False,
        )

        fig.add_artist(line)


def add_optimal_operating_area(
    fig,
    time_axes,
    start_s: float,
    end_s: float,
) -> None:
    fig.canvas.draw()

    left = data_x_to_fig_x(fig, time_axes[0], start_s)
    right = data_x_to_fig_x(fig, time_axes[0], end_s)

    if right <= left:
        return

    for ax in time_axes:
        fill = Rectangle(
            (start_s, 0.0),
            end_s - start_s,
            1.0,
            transform=ax.get_xaxis_transform(),
            facecolor=OPTIMAL_AREA_COLOR,
            edgecolor="none",
            alpha=0.16,
            zorder=1.0,
        )
        ax.add_patch(fill)

        ax.axvline(start_s, color=OPTIMAL_AREA_COLOR, linewidth=4.0, zorder=1.1)
        ax.axvline(end_s, color=OPTIMAL_AREA_COLOR, linewidth=4.0, zorder=1.1)

    gap_bottom = time_axes[1].get_position().y1
    gap_top = time_axes[0].get_position().y0
    if gap_top > gap_bottom:
        gap_fill = Rectangle(
            (left, gap_bottom),
            right - left,
            gap_top - gap_bottom,
            transform=fig.transFigure,
            facecolor=OPTIMAL_AREA_COLOR,
            edgecolor="none",
            alpha=0.16,
            zorder=32,
            clip_on=False,
        )
        fig.add_artist(gap_fill)

        for edge_x in [left, right]:
            fig.add_artist(
                Line2D(
                    [edge_x, edge_x],
                    [gap_bottom, gap_top],
                    transform=fig.transFigure,
                    color=OPTIMAL_AREA_COLOR,
                    linewidth=4.0,
                    zorder=34,
                    clip_on=False,
                )
            )

    fig.text(
        data_x_to_fig_x(
            fig,
            time_axes[0],
            0.5 * (start_s + end_s) + OPTIMAL_AREA_LABEL_X_OFFSET_S,
        ),
        0.5 * (time_axes[0].get_position().y0 + time_axes[1].get_position().y1),
        OPTIMAL_AREA_LABEL,
        ha="center",
        va="center",
        fontsize=22,
        fontweight="bold",
        color=OPTIMAL_AREA_TEXT_COLOR,
        zorder=45,
    )


def add_regime_labels(
    fig,
    ref_ax,
    spectrum_axes,
    sender_start_s: float,
    under_to_full: float | None,
    full_to_overload: float | None,
    sender_stop_s: float,
) -> None:
    """Place Regime labels at the centers of the three equal time regions."""

    fig.canvas.draw()

    edge_0 = sender_start_s
    edge_1 = under_to_full if under_to_full is not None else sender_stop_s
    edge_2 = full_to_overload if full_to_overload is not None else sender_stop_s
    edge_3 = sender_stop_s

    edges = [
        min(max(edge_0, sender_start_s), sender_stop_s),
        min(max(edge_1, sender_start_s), sender_stop_s),
        min(max(edge_2, sender_start_s), sender_stop_s),
        min(max(edge_3, sender_start_s), sender_stop_s),
    ]

    regime_labels = [
        "Regime I:\nbottleneck-unsaturated",
        "Regime II:\nbottleneck-saturated",
        "Regime III:\nbuffer-saturated",
    ]

    fig_edges = [data_x_to_fig_x(fig, ref_ax, edge) for edge in edges]

    y_top = max(ax.get_position().y1 for ax in spectrum_axes)

    # Two-line phase labels are placed above the spectrum panels so that
    # they are centered within the regions separated by the dashed boundaries.
    label_y = y_top + 0.025

    for i, label in enumerate(regime_labels):
        left = fig_edges[i]
        right = fig_edges[i + 1]

        if right <= left:
            continue

        center_x = 0.5 * (left + right)

        fig.text(
            center_x,
            label_y,
            label,
            ha="center",
            va="bottom",
            fontsize=PHASE_LABEL_FS,
            fontweight="bold",
            linespacing=1.05,
            color="#212529",
            zorder=40,
        )


def normalized_spectrum(values: np.ndarray, fs_hz: float) -> tuple[np.ndarray, np.ndarray]:
    detrended = signal.detrend(values, type="linear")
    window = signal.windows.hann(len(detrended), sym=False)

    freqs = np.fft.rfftfreq(len(detrended), d=1.0 / fs_hz)
    energy = np.abs(np.fft.rfft(detrended * window)) ** 2

    if np.max(energy) > 0:
        energy = energy / np.max(energy)

    return freqs, energy


def top_freqs(
    freqs: np.ndarray,
    energy: np.ndarray,
    lo: float = 0.5,
    hi: float = 30.0,
    n: int = 5,
):
    mask = (freqs >= lo) & (freqs <= hi)
    f = freqs[mask]
    e = energy[mask]

    order = np.argsort(e)[::-1]
    picked: list[tuple[float, float]] = []

    for i in order:
        if all(abs(float(f[i]) - prev_f) >= 0.6 for prev_f, _ in picked):
            picked.append((float(f[i]), float(e[i])))

        if len(picked) >= n:
            break

    return picked


def triangle_value(phase: np.ndarray) -> np.ndarray:
    phase = np.mod(phase, 1.0)
    return np.where(
        phase < 0.25,
        4.0 * phase,
        np.where(phase < 0.75, 2.0 - 4.0 * phase, 4.0 * phase - 4.0),
    )


def measure_complete_wave_cycles(
    t: np.ndarray,
    values: np.ndarray,
    start_s: float,
    end_s: float,
    modulation_freq_hz: float,
    expected_peak_to_peak_mbps: float,
) -> tuple[float, int, int, list[tuple[float, float, float, bool]]]:
    """Measure complete response cycles using amplitude and triangle-shape gates."""

    if modulation_freq_hz <= 0.0 or end_s <= start_s:
        return 0.0, 0, 0, []

    period_s = 1.0 / modulation_freq_hz
    dt = float(np.median(np.diff(t)))
    expected_samples = max(1, int(round(period_s / dt)))
    cycle_count = max(0, int(np.floor((end_s - start_s) / period_s + 1e-9)))
    details: list[tuple[float, float, float, bool]] = []
    complete_count = 0

    for cycle_index in range(cycle_count):
        cycle_start = start_s + cycle_index * period_s
        cycle_end = cycle_start + period_s
        mask = (t >= cycle_start) & (t < cycle_end)
        cycle_values = values[mask]

        if len(cycle_values) < max(8, int(0.75 * expected_samples)):
            details.append((cycle_start, 0.0, 0.0, False))
            continue

        finite = np.isfinite(cycle_values)
        if np.count_nonzero(finite) < max(8, int(0.75 * expected_samples)):
            details.append((cycle_start, 0.0, 0.0, False))
            continue

        cycle_t = t[mask][finite]
        cycle_values = cycle_values[finite]
        peak_to_peak = float(
            np.percentile(cycle_values, 95) - np.percentile(cycle_values, 5)
        )
        amplitude_ratio = peak_to_peak / max(expected_peak_to_peak_mbps, 1e-9)

        centered_values = cycle_values - np.mean(cycle_values)
        value_norm = float(np.linalg.norm(centered_values))
        best_correlation = 0.0
        if value_norm > 1e-9:
            base_phase = (cycle_t - cycle_start) / period_s
            for shift in np.linspace(0.0, 1.0, 41, endpoint=False):
                template = triangle_value(base_phase - shift)
                template -= np.mean(template)
                denom = value_norm * float(np.linalg.norm(template))
                if denom > 1e-9:
                    best_correlation = max(
                        best_correlation,
                        float(np.dot(centered_values, template) / denom),
                    )

        complete = amplitude_ratio >= 0.60 and best_correlation >= 0.75
        complete_count += int(complete)
        details.append((cycle_start, amplitude_ratio, best_correlation, complete))

    complete_ratio = complete_count / cycle_count if cycle_count else 0.0
    return complete_ratio, complete_count, cycle_count, details


def plot_phase_spectrum(
    ax,
    t: np.ndarray,
    rx: np.ndarray,
    estimated_rtt_ms: np.ndarray,
    start_s: float,
    end_s: float,
    freq_max_hz: float,
    show_ylabel: bool,
) -> None:
    mask = (t >= start_s) & (t <= end_s)

    if np.count_nonzero(mask) < 8 or end_s <= start_s:
        ax.text(
            0.5,
            0.5,
            "insufficient samples",
            ha="center",
            va="center",
            transform=ax.transAxes,
            fontsize=LABEL_FS,
            fontweight="bold",
        )

        style_axis(ax, tick_size=TICK_FS)
        return

    phase_t = t[mask]
    fs_hz = 1.0 / float(np.median(np.diff(phase_t)))

    rx_freqs, rx_energy = normalized_spectrum(rx[mask], fs_hz)
    rtt_freqs, rtt_energy = normalized_spectrum(estimated_rtt_ms[mask], fs_hz)

    rx_mask = (rx_freqs >= 0.0) & (rx_freqs <= freq_max_hz)
    rtt_mask = (rtt_freqs >= 0.0) & (rtt_freqs <= freq_max_hz)

    energy_floor = 0.1

    ax.plot(
        rx_freqs[rx_mask],
        np.maximum(rx_energy[rx_mask], energy_floor),
        color=DRATE_COLOR,
        linewidth=DATA_LW,
        label="DRate spectrum",
    )

    ax.plot(
        rtt_freqs[rtt_mask],
        np.maximum(rtt_energy[rtt_mask], energy_floor),
        color=SRTT_COLOR,
        linewidth=DATA_LW,
        label="SRTT spectrum",
    )

    ax.set_ylim(energy_floor, 1.05)
    ax.set_xlim(0, freq_max_hz)

    ax.set_xlabel(
        "Frequency (Hz)",
        fontsize=LABEL_FS,
        fontweight="bold",
        labelpad=10,
    )

    if show_ylabel:
        ax.set_ylabel(
            "Normalized\nenergy",
            fontsize=LABEL_FS,
            fontweight="bold",
            labelpad=14,
        )

    ax.grid(
        True,
        which="both",
        color="#d0d0d0",
        linewidth=GRID_LW,
        alpha=0.75,
    )

    style_axis(ax, tick_size=TICK_FS)

    ax.legend(
        loc="upper right",
        ncol=1,
        frameon=True,
        prop={"size": SMALL_LEGEND_FS, "weight": "bold"},
        handlelength=2.2,
        handletextpad=0.45,
        borderpad=0.25,
        labelspacing=0.28,
    )


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)

    parser.add_argument("--trace-dir", default="traces/open_loop_p2p_rate_ramp")
    parser.add_argument("--trace-name", default="open_loop_p2p_rate_ramp")
    parser.add_argument("--output-tag", default=None)
    parser.add_argument("--empty-fraction", type=float, default=0.005)
    parser.add_argument("--overload-fraction", type=float, default=0.999)
    parser.add_argument("--freq-max-hz", type=float, default=30.0)

    return parser


def main() -> None:
    args = build_parser().parse_args()

    trace_dir = Path(args.trace_dir)
    trace_name = args.trace_name

    config = read_config(trace_dir / f"{trace_name}_config.txt")

    rate_path = trace_dir / f"{trace_name}_rate_trace.txt"
    queue_path = trace_dir / f"{trace_name}_bottleneck_queue.txt"

    rate = np.loadtxt(rate_path, comments="#")

    t = rate[:, 0]
    target = rate[:, 3]
    rx = rate[:, 5]
    cubic_rx = rate[:, 8] if rate.shape[1] > 8 else np.zeros_like(rx)

    queue = np.loadtxt(queue_path, comments="#", usecols=(0, 1))
    queue_at_t = resample_queue_to_rate_times(t, queue[:, 0], queue[:, 1])

    link_bw_mbps = config.get("link_bw_mbps", 350.0)
    sender_start_s = config.get("sender_start_time_s", 0.0)
    sender_duration_s = config.get("sender_duration_s", 5.0)
    sender_stop_s = sender_start_s + sender_duration_s
    triangle_freq_hz = config.get("triangle_freq_hz", 5.0)
    queue_bytes = config.get("queue_bytes", float(np.max(queue[:, 1])))
    base_rtt_ms = 2.0 * config.get("one_way_delay_ms", 20.0)

    queue_fraction = queue_at_t / max(queue_bytes, 1.0)

    estimated_rtt_ms = (
        base_rtt_ms
        + queue_at_t * 8.0 / (link_bw_mbps * 1_000_000.0) * 1000.0
    )

    regime_duration_s = sender_duration_s / 3.0
    under_to_full = sender_start_s + regime_duration_s
    full_to_overload = sender_start_s + 2.0 * regime_duration_s
    boundary_window = max(
        3,
        int(round((1.0 / max(triangle_freq_hz, 1e-9)) / np.median(np.diff(t)))),
    )

    regime_ii_wave_ratio, regime_ii_complete_cycles, regime_ii_total_cycles, cycle_details = (
        measure_complete_wave_cycles(
            t,
            rx,
            under_to_full,
            full_to_overload,
            triangle_freq_hz,
            2.0 * config.get("triangle_amp_mbps", 80.0),
        )
    )

    output_tag = args.output_tag

    if output_tag is None:
        output_tag = (
            f"{config.get('start_rate_mbps', 200.0):.0f}_"
            f"{config.get('end_rate_mbps', 450.0):.0f}_"
            f"{sender_duration_s:.0f}s_"
            f"{link_bw_mbps:.0f}mbps_"
            f"cubic{config.get('cubic_app_rate_mbps', 0.0):.0f}"
        )

    time_out = trace_dir / f"{trace_name}_{output_tag}_send_recv_rtt_curve.png"
    spectrum_out = trace_dir / f"{trace_name}_{output_tag}_rx_rtt_spectrum_energy.png"
    summary_out = trace_dir / f"{trace_name}_{output_tag}_load_boundaries.txt"
    remove_existing_outputs([time_out, spectrum_out, summary_out])

    fig = plt.figure(figsize=(19.0, 12.4), dpi=240)

    fig.suptitle(
        (
            "Triangle probe with TCP CUBIC cross traffic: "
            f"{link_bw_mbps:.0f} Mbps link, 4 BDP buffer"
        ),
        fontsize=SUPTITLE_FS,
        fontweight="bold",
        y=0.985,
    )

    regime_edges = [
        sender_start_s,
        under_to_full,
        full_to_overload,
        sender_stop_s,
    ]

    regime_edges = [
        min(max(edge, sender_start_s), sender_stop_s)
        for edge in regime_edges
    ]

    def time_to_fig_x(x_value: float) -> float:
        duration = max(sender_stop_s - sender_start_s, 1e-12)
        norm_x = (x_value - sender_start_s) / duration
        return FIG_LEFT + norm_x * FIG_WIDTH

    axes = [fig.add_axes([FIG_LEFT, RATE_Y0, FIG_WIDTH, RATE_H])]
    axes.append(
        fig.add_axes(
            [FIG_LEFT, RTT_Y0, FIG_WIDTH, RTT_H],
            sharex=axes[0],
        )
    )

    spectrum_axes = []
    for i in range(3):
        region_left = time_to_fig_x(regime_edges[i])
        region_right = time_to_fig_x(regime_edges[i + 1])

        ax_left = region_left + (PHASE_AXIS_GAP if i > 0 else 0.0)
        ax_right = region_right - (PHASE_AXIS_GAP if i < 2 else 0.0)

        if ax_right - ax_left < MIN_PHASE_AXIS_W:
            center = 0.5 * (region_left + region_right)
            ax_left = max(FIG_LEFT, center - 0.5 * MIN_PHASE_AXIS_W)
            ax_right = min(FIG_RIGHT, center + 0.5 * MIN_PHASE_AXIS_W)

        spectrum_axes.append(
            fig.add_axes([ax_left, SPECTRUM_Y0, ax_right - ax_left, SPECTRUM_H])
        )

    for i, ax in enumerate(spectrum_axes):
        plot_phase_spectrum(
            ax,
            t,
            rx,
            estimated_rtt_ms,
            regime_edges[i],
            regime_edges[i + 1],
            args.freq_max_hz,
            show_ylabel=(i == 0),
        )

    # =========================
    # Combined Rate subplot
    # =========================

    axes[0].plot(
        t,
        target,
        color=SEND_RATE_COLOR,
        linewidth=RATE_SERIES_LW,
        label="Send rate",
        zorder=3,
    )

    axes[0].plot(
        t[t > 0],
        rx[t > 0],
        color=DRATE_COLOR,
        linewidth=RATE_SERIES_LW,
        label="Probe DRate",
        zorder=4,
    )

    cubic_window = max(1, int(round(0.05 / max(float(np.median(np.diff(t))), 1e-9))))
    cubic_smoothed = np.convolve(
        cubic_rx,
        np.ones(cubic_window, dtype=float) / cubic_window,
        mode="same",
    )
    cubic_active = t >= under_to_full
    axes[0].plot(
        t[cubic_active],
        cubic_smoothed[cubic_active],
        color=CUBIC_COLOR,
        linewidth=3.0,
        linestyle="--",
        label="CUBIC DRate",
        zorder=2,
    )

    axes[0].axhline(
        link_bw_mbps,
        color="#c92a2a",
        linewidth=SERVICE_LW,
        linestyle=":",
        label="Link service rate",
    )

    axes[0].set_ylabel(
        "Rate\n(Mbps)",
        fontsize=LABEL_FS,
        fontweight="bold",
        labelpad=18,
    )

    rate_ymax = max(
        560,
        float(np.nanmax(target) + 25),
        float(np.nanmax(rx) + 25),
        float(np.nanmax(cubic_smoothed) + 25),
        link_bw_mbps + 45,
    )

    axes[0].set_ylim(0, rate_ymax)

    axes[0].set_xlim(
        sender_start_s,
        sender_stop_s,
    )

    # =========================
    # RTT subplot
    # =========================

    axes[1].plot(
        t,
        estimated_rtt_ms,
        color=SRTT_COLOR,
        linewidth=DATA_LW,
        label="SRTT",
    )

    axes[1].set_ylabel(
        "RTT\n(ms)",
        fontsize=LABEL_FS,
        fontweight="bold",
        labelpad=18,
    )

    axes[1].set_xlabel(
        "Time (s)",
        fontsize=LABEL_FS,
        fontweight="bold",
        labelpad=12,
    )

    axes[1].set_ylim(
        max(0, base_rtt_ms - 5),
        max(85, float(np.nanmax(estimated_rtt_ms) + 4)),
    )

    for i, ax in enumerate(axes):
        ax.grid(
            True,
            color="#d0d0d0",
            linewidth=GRID_LW,
            alpha=0.75,
        )

        style_axis(ax, tick_size=TICK_FS)

        if i == 0:
            legend = ax.legend(
                loc="lower right",
                ncol=4,
                frameon=True,
                prop={"size": LEGEND_FS, "weight": "bold"},
                handlelength=3.2,
            )
        else:
            legend = ax.legend(
                loc="lower right",
                ncol=1,
                frameon=True,
                prop={"size": LEGEND_FS, "weight": "bold"},
                handlelength=3.2,
            )
        legend.get_frame().set_facecolor("none")
        legend.get_frame().set_edgecolor("none")
        legend.get_frame().set_alpha(0.0)

    add_load_boundaries(
        fig,
        time_axes=axes,
        all_axes=spectrum_axes + axes,
        under_to_full=under_to_full,
        full_to_overload=full_to_overload,
    )

    add_regime_labels(
        fig,
        ref_ax=axes[0],
        spectrum_axes=spectrum_axes,
        sender_start_s=sender_start_s,
        under_to_full=under_to_full,
        full_to_overload=full_to_overload,
        sender_stop_s=sender_stop_s,
    )

    axes[0].text(
        0.5 * (under_to_full + full_to_overload),
        0.96,
        (
            "Regime II complete DRate cycles: "
            f"{regime_ii_complete_cycles}/{regime_ii_total_cycles} "
            f"({100.0 * regime_ii_wave_ratio:.0f}%)"
        ),
        transform=axes[0].get_xaxis_transform(),
        ha="center",
        va="top",
        fontsize=16,
        fontweight="bold",
        color="#7f2704",
        zorder=20,
    )

    fig.savefig(time_out, bbox_inches="tight", pad_inches=0.10)
    plt.close(fig)

    active = (t >= sender_start_s) & (t <= sender_stop_s)
    t_active = t[active]
    dt = float(np.median(np.diff(t_active)))
    fs_hz = 1.0 / dt

    rx_freqs, rx_energy = normalized_spectrum(rx[active], fs_hz)
    rtt_freqs, rtt_energy = normalized_spectrum(estimated_rtt_ms[active], fs_hz)

    rx_top = top_freqs(rx_freqs, rx_energy, hi=args.freq_max_hz)
    rtt_top = top_freqs(rtt_freqs, rtt_energy, hi=args.freq_max_hz)

    fig, axes = plt.subplots(
        2,
        1,
        figsize=(15.5, 9.2),
        dpi=240,
        sharex=True,
    )

    fig.subplots_adjust(
        left=0.145,
        right=0.98,
        top=0.86,
        bottom=0.205,
        hspace=0.38,
    )

    fig.suptitle(
        "Frequency-domain energy features",
        fontsize=SUPTITLE_FS,
        fontweight="bold",
    )

    for ax, freqs, energy, title, color, label in [
        (
            axes[0],
            rx_freqs,
            rx_energy,
            "DRate spectrum",
            DRATE_COLOR,
            "DRate",
        ),
        (
            axes[1],
            rtt_freqs,
            rtt_energy,
            "Estimated SRTT spectrum",
            SRTT_COLOR,
            "SRTT",
        ),
    ]:
        mask = (freqs >= 0.0) & (freqs <= args.freq_max_hz)

        ax.plot(
            freqs[mask],
            energy[mask],
            color=color,
            linewidth=DATA_LW,
            label=label,
        )

        ax.fill_between(
            freqs[mask],
            0,
            energy[mask],
            color=color,
            alpha=0.16,
        )

        ax.set_yscale("log")
        ax.set_ylim(1e-8, 1.4)

        ax.set_ylabel(
            "Normalized\nenergy",
            fontsize=LABEL_FS,
            fontweight="bold",
            labelpad=18,
        )

        ax.set_title(
            title,
            fontsize=AX_TITLE_FS,
            fontweight="bold",
        )

        ax.grid(
            True,
            which="both",
            color="#d0d0d0",
            linewidth=GRID_LW,
            alpha=0.75,
        )

        style_axis(ax, tick_size=TICK_FS)

        ax.legend(
            loc="upper right",
            ncol=1,
            frameon=True,
            prop={"size": LEGEND_FS, "weight": "bold"},
            handlelength=3.2,
        )

    axes[1].set_xlabel(
        "Frequency (Hz)",
        fontsize=LABEL_FS,
        fontweight="bold",
        labelpad=16,
    )

    axes[1].set_xlim(0, args.freq_max_hz)

    fig.text(
        0.5,
        0.035,
        f"FFT over active 0-{sender_duration_s:g} s interval; fs {fs_hz:.0f} Hz; linear detrend + Hann window",
        ha="center",
        va="bottom",
        fontsize=19,
        fontweight="bold",
        color="#495057",
    )

    fig.savefig(spectrum_out, bbox_inches="tight", pad_inches=0.10)
    plt.close(fig)

    with summary_out.open("w") as stream:
        stream.write("# Equal-duration regime and spectrum summary\n")
        stream.write(f"regime_i_to_ii_s\t{under_to_full}\n")
        stream.write(f"regime_ii_to_iii_s\t{full_to_overload}\n")
        stream.write(f"regime_duration_s\t{regime_duration_s}\n")
        stream.write(f"boundary_window_samples\t{boundary_window}\n")
        stream.write(f"regime_ii_complete_wave_ratio\t{regime_ii_wave_ratio}\n")
        stream.write(f"regime_ii_complete_cycles\t{regime_ii_complete_cycles}\n")
        stream.write(f"regime_ii_total_cycles\t{regime_ii_total_cycles}\n")
        stream.write("# cycle_start_s\tamplitude_ratio\tshape_correlation\tcomplete\n")
        for cycle_start, amplitude_ratio, correlation, complete in cycle_details:
            stream.write(
                f"regime_ii_cycle\t{cycle_start:.6f}\t{amplitude_ratio:.6f}\t"
                f"{correlation:.6f}\t{int(complete)}\n"
            )
        stream.write(
            "rx_top_freqs_hz_energy\t"
            + "\t".join(f"{f:.6f}:{e:.6g}" for f, e in rx_top)
            + "\n"
        )
        stream.write(
            "rtt_top_freqs_hz_energy\t"
            + "\t".join(f"{f:.6f}:{e:.6g}" for f, e in rtt_top)
            + "\n"
        )

    print(time_out)
    print(spectrum_out)
    print(summary_out)
    print(f"regime_i_to_ii_s={under_to_full}")
    print(f"regime_ii_to_iii_s={full_to_overload}")
    print(
        "regime_ii_complete_wave_ratio="
        f"{regime_ii_wave_ratio:.6f} "
        f"({regime_ii_complete_cycles}/{regime_ii_total_cycles} cycles)"
    )


if __name__ == "__main__":
    main()
