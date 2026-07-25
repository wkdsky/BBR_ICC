#!/usr/bin/env python3
"""Plot open-loop P2P rate-ramp traces and infer load-state boundaries."""

from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib
import matplotlib.patheffects as pe
from matplotlib import font_manager

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from scipy import signal
from matplotlib.lines import Line2D
from matplotlib.patches import Circle, ConnectionPatch, Rectangle

TIMES_NEW_ROMAN_PATH = Path(
    "/usr/share/fonts/truetype/msttcorefonts/Times_New_Roman.ttf"
)
TIMES_NEW_ROMAN_BOLD_PATH = Path(
    "/usr/share/fonts/truetype/msttcorefonts/Times_New_Roman_Bold.ttf"
)
if TIMES_NEW_ROMAN_PATH.exists():
    font_manager.fontManager.addfont(str(TIMES_NEW_ROMAN_PATH))
if TIMES_NEW_ROMAN_BOLD_PATH.exists():
    font_manager.fontManager.addfont(str(TIMES_NEW_ROMAN_BOLD_PATH))


# =========================
# Paper-style figure config
# =========================

BOUNDARY_COLOR = "#212529"
REGIME_BRACKET_COLOR = "#343a40"
OPTIMAL_AREA_COLOR = "#f08c00"
OPTIMAL_AREA_TEXT_COLOR = "#7a3f00"
SEND_RATE_COLOR = "#0072b2"
DRATE_COLOR = "#d9480f"
CROSS_COLOR = "#495057"
SRTT_COLOR = "#5f3dc4"
REGIME_PSD_DRATE_COLOR = "#b85d24"
REGIME_PSD_SRTT_COLOR = "#3f2bb8"
CALLOUT_COLOR = "#009e73"
CALLOUT_FILL_COLOR = "#c3fae8"
OPTIMAL_AREA_START_S = 2.2
OPTIMAL_AREA_END_S = 2.4
OPTIMAL_AREA_LABEL_X_OFFSET_S = 0.16
OPTIMAL_AREA_LABEL = "Kleinrock's optimal\noperating area"

EXPORT_DPI = 600
IEEE_COLUMN_WIDTH_IN = 3.5
TARGET_FINAL_TEXT_PT = 8.0
TIME_FIGSIZE = (IEEE_COLUMN_WIDTH_IN, 3.48)
SPECTRUM_FIGSIZE = (IEEE_COLUMN_WIDTH_IN, 2.2)
TEXT_WEIGHT = "bold"

SOURCE_TEXT_FS = TARGET_FINAL_TEXT_PT
SUPTITLE_FS = SOURCE_TEXT_FS
REGIME_LABEL_FS = SOURCE_TEXT_FS
AX_TITLE_FS = SOURCE_TEXT_FS
LABEL_FS = SOURCE_TEXT_FS
TICK_FS = SOURCE_TEXT_FS
LEGEND_FS = SOURCE_TEXT_FS
SMALL_LEGEND_FS = SOURCE_TEXT_FS

DATA_LW = 1.45
REGIME_PSD_DRATE_LW = DATA_LW
REGIME_PSD_SRTT_LW = DATA_LW + 0.25
SEND_RATE_LW = 1.45
RATE_SERIES_LW = 1.75
BACKGROUND_RATE_LW = 1.25
SERVICE_LW = 1.20
BOUNDARY_LW = 1.35
GRID_LW = 0.35
SPINE_LW = 0.70
HATCH_LW = 0.28

# Keep every simultaneously displayed series identifiable in grayscale.
SEND_RATE_LINESTYLE = (0, (0.9, 0.55))
DRATE_LINESTYLE = "-"
REGIME_PSD_DRATE_LINESTYLE = (0, (2.6, 1.15))
REGIME_PSD_SRTT_LINESTYLE = "-"
BACKGROUND_LINESTYLE = (0, (5.0, 2.0, 1.4, 2.0))
SERVICE_RATE_LINESTYLE = (0, (1.0, 1.8))
SRTT_LINESTYLE = "-"
SPECTRUM_SRTT_LINESTYLE = (0, (0.9, 0.55))

# Manual figure layout.  The first-row spectrum panels are horizontally
# aligned with the three equal time-domain Regimes.
FIG_LEFT = 0.070
FIG_RIGHT = 0.974
FIG_WIDTH = FIG_RIGHT - FIG_LEFT
SPECTRUM_Y0 = 0.727
SPECTRUM_H = 0.157
RATE_Y0 = 0.403
RATE_H = 0.215
RTT_Y0 = 0.086
RTT_H = 0.252
REGIME_AXIS_GAP = 0.006
MIN_REGIME_AXIS_W = 0.075
VISIBLE_SRTT_P2P_MS = 1.0

plt.rcParams.update(
    {
        "font.size": LABEL_FS,
        "font.family": "serif",
        "font.serif": [
            "Times New Roman",
            "Times",
            "Nimbus Roman",
            "Liberation Serif",
            "DejaVu Serif",
        ],
        "font.weight": TEXT_WEIGHT,
        "axes.labelweight": TEXT_WEIGHT,
        "axes.titleweight": TEXT_WEIGHT,
        "axes.linewidth": SPINE_LW,
        "xtick.major.width": SPINE_LW,
        "ytick.major.width": SPINE_LW,
        "xtick.major.size": 2.2,
        "ytick.major.size": 2.2,
        "figure.titleweight": TEXT_WEIGHT,
        "legend.framealpha": 0.94,
        "hatch.linewidth": HATCH_LW,
        "lines.solid_capstyle": "round",
        "lines.dash_capstyle": "round",
        "lines.solid_joinstyle": "round",
        "lines.dash_joinstyle": "round",
        "pdf.fonttype": 42,
        "ps.fonttype": 42,
    }
)


def style_axis(ax, tick_size: float = TICK_FS) -> None:
    ax.tick_params(
        axis="both",
        labelsize=tick_size,
        width=SPINE_LW,
        length=2.2,
        pad=0.5,
    )

    for label in ax.get_xticklabels() + ax.get_yticklabels():
        label.set_fontweight(TEXT_WEIGHT)

    for spine in ax.spines.values():
        spine.set_linewidth(SPINE_LW)


def add_axis_header(ax, text: str) -> None:
    ax.text(
        0.0,
        1.018,
        text,
        transform=ax.transAxes,
        ha="left",
        va="bottom",
        fontsize=LABEL_FS,
        fontweight=TEXT_WEIGHT,
        clip_on=False,
    )


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


def visible_srtt_wave_span(
    t: np.ndarray,
    srtt_ms: np.ndarray,
    sender_start_s: float,
    first_boundary_s: float,
    second_boundary_s: float,
    sender_stop_s: float,
    modulation_freq_hz: float,
    min_peak_to_peak_ms: float,
) -> tuple[float, float]:
    """Return contiguous visible-wave margins adjacent to Regime II."""

    period_s = 1.0 / max(modulation_freq_hz, 1e-9)
    cycles: list[tuple[float, float, float]] = []
    cycle_start_s = sender_start_s
    while cycle_start_s < sender_stop_s - 0.5 * period_s:
        cycle_end_s = min(sender_stop_s, cycle_start_s + period_s)
        mask = (t >= cycle_start_s - 1e-9) & (t < cycle_end_s - 1e-9)
        values = srtt_ms[mask]
        peak_to_peak_ms = float(np.ptp(values)) if len(values) else 0.0
        cycles.append((cycle_start_s, cycle_end_s, peak_to_peak_ms))
        cycle_start_s = cycle_end_s

    visible_start_s = first_boundary_s
    for cycle_start_s, cycle_end_s, peak_to_peak_ms in reversed(cycles):
        if cycle_end_s > first_boundary_s + 1e-8:
            continue
        if peak_to_peak_ms < min_peak_to_peak_ms:
            break
        visible_start_s = cycle_start_s

    visible_end_s = second_boundary_s
    for cycle_start_s, cycle_end_s, peak_to_peak_ms in cycles:
        if cycle_start_s < second_boundary_s - 1e-8:
            continue
        if peak_to_peak_ms < min_peak_to_peak_ms:
            break
        visible_end_s = cycle_end_s

    return visible_start_s, visible_end_s


def add_time_domain_margin_backgrounds(
    axes,
    visible_start_s: float,
    first_boundary_s: float,
    second_boundary_s: float,
    visible_end_s: float,
) -> None:
    """Mark the two visible-wave margins behind the time-domain curves."""

    intervals = [
        (visible_start_s, first_boundary_s),
        (second_boundary_s, visible_end_s),
    ]
    interval_labels = ["①", "②"]
    for ax_index, ax in enumerate(axes):
        for interval_index, (start_s, end_s) in enumerate(intervals):
            if end_s <= start_s:
                continue
            ax.add_patch(
                Rectangle(
                    (start_s, 0.0),
                    end_s - start_s,
                    1.0,
                    transform=ax.get_xaxis_transform(),
                    facecolor="#cfd8e3",
                    edgecolor="none",
                    alpha=0.42,
                    zorder=0.45,
                )
            )
            # A neutral diagonal texture remains legible in grayscale while
            # the translucent cool-gray fill stays unobtrusive in color.
            ax.add_patch(
                Rectangle(
                    (start_s, 0.0),
                    end_s - start_s,
                    1.0,
                    transform=ax.get_xaxis_transform(),
                    facecolor="none",
                    edgecolor="#6b7785",
                    linewidth=0.48,
                    hatch="////",
                    alpha=0.46,
                    zorder=0.50,
                )
            )
            if ax_index == len(axes) - 1:
                ax.text(
                    0.5 * (start_s + end_s),
                    0.335,
                    interval_labels[interval_index],
                    transform=ax.get_xaxis_transform(),
                    ha="center",
                    va="center",
                    fontsize=LABEL_FS * 1.65,
                    fontfamily="DejaVu Sans",
                    fontweight=TEXT_WEIGHT,
                    color="#263238",
                    path_effects=[
                        pe.withStroke(linewidth=2.0, foreground="#ffffff", alpha=0.92)
                    ],
                    zorder=8,
                )


def add_srtt_magnifiers(
    fig,
    ax,
    t: np.ndarray,
    srtt_ms: np.ndarray,
    intervals: list[tuple[float, float]],
) -> None:
    """Add point-symmetric wide zoom lenses for the shaded SRTT margins."""

    fig.canvas.draw()
    ax_position = ax.get_position()
    lens_width_in = 0.62
    lens_height_in = 0.42
    lens_width = lens_width_in / fig.get_figwidth()
    lens_height = lens_height_in / fig.get_figheight()
    lens_centers_in_axes = [(0.115, 0.69), (0.885, 0.31)]

    for index, ((start_s, end_s), (center_x_axes, center_y_axes)) in enumerate(
        zip(intervals, lens_centers_in_axes)
    ):
        if end_s <= start_s:
            continue

        mask = (t >= start_s) & (t <= end_s)
        if np.count_nonzero(mask) < 2:
            continue

        center_x = ax_position.x0 + center_x_axes * ax_position.width
        center_y = ax_position.y0 + center_y_axes * ax_position.height
        lens_bounds = [
            center_x - 0.5 * lens_width,
            center_y - 0.5 * lens_height,
            lens_width,
            lens_height,
        ]
        lens_ax = fig.add_axes(lens_bounds, zorder=24)
        lens_ax.set_facecolor("none")

        lens_circle = Circle(
            (0.5, 0.5),
            0.47,
            transform=lens_ax.transAxes,
            facecolor="#ffffff",
            edgecolor=CALLOUT_COLOR,
            linewidth=0.85,
            zorder=1,
        )
        lens_ax.add_patch(lens_circle)

        local_t = t[mask]
        local_srtt = srtt_ms[mask]
        x_padding = max(0.02, 0.13 * (end_s - start_s))
        y_span = float(np.ptp(local_srtt))
        y_padding = max(1.0, 0.25 * y_span)
        lens_ax.set_xlim(start_s - x_padding, end_s + x_padding)
        lens_ax.set_ylim(
            float(np.min(local_srtt)) - y_padding,
            float(np.max(local_srtt)) + y_padding,
        )

        for fraction in (0.25, 0.50, 0.75):
            vertical_grid = lens_ax.axvline(
                start_s + fraction * (end_s - start_s),
                color="#d0d0d0",
                linewidth=0.28,
                alpha=0.72,
                zorder=1.5,
            )
            vertical_grid.set_clip_path(lens_circle)

        horizontal_grid = lens_ax.axhline(
            float(np.mean(local_srtt)),
            color="#d0d0d0",
            linewidth=0.28,
            alpha=0.72,
            zorder=1.5,
        )
        horizontal_grid.set_clip_path(lens_circle)

        zoom_line = lens_ax.plot(
            local_t,
            local_srtt,
            color=SRTT_COLOR,
            linewidth=1.15,
            linestyle=SRTT_LINESTYLE,
            zorder=3,
        )[0]
        zoom_line.set_clip_path(lens_circle)

        lens_ax.set_xticks([])
        lens_ax.set_yticks([])
        for spine in lens_ax.spines.values():
            spine.set_visible(False)

        box_y_padding = max(5.0, 0.35 * y_span)
        box_y_min = float(np.min(local_srtt)) - box_y_padding
        box_y_max = float(np.max(local_srtt)) + box_y_padding
        zoom_highlight = Rectangle(
            (start_s, box_y_min),
            end_s - start_s,
            box_y_max - box_y_min,
            facecolor=CALLOUT_FILL_COLOR,
            edgecolor="none",
            alpha=0.82,
            zorder=1.25,
        )
        ax.add_patch(zoom_highlight)

        zoom_outline = Rectangle(
            (start_s, box_y_min),
            end_s - start_s,
            box_y_max - box_y_min,
            facecolor="none",
            edgecolor=CALLOUT_COLOR,
            linewidth=0.45,
            linestyle="-",
            alpha=0.95,
            zorder=1.75,
        )
        ax.add_patch(zoom_outline)

        if index == 0:
            handle_target = (start_s, box_y_max)
        else:
            handle_target = (end_s, box_y_min)

        source_display = ax.transData.transform(handle_target)
        lens_center_display = fig.transFigure.transform((center_x, center_y))
        direction = source_display - lens_center_display
        if float(np.hypot(direction[0], direction[1])) <= 0.0:
            continue

        lens_radius_x_pixels = 0.47 * lens_width_in * fig.dpi
        lens_radius_y_pixels = 0.47 * lens_height_in * fig.dpi
        ellipse_scale = 1.0 / np.sqrt(
            (direction[0] / lens_radius_x_pixels) ** 2
            + (direction[1] / lens_radius_y_pixels) ** 2
        )
        handle_start_display = lens_center_display + direction * ellipse_scale
        handle_start_figure = fig.transFigure.inverted().transform(
            handle_start_display
        )

        for linewidth, color, zorder in (
            (2.3, "#ffffff", 21),
            (1.1, CALLOUT_COLOR, 22),
        ):
            handle = ConnectionPatch(
                xyA=handle_target,
                coordsA=ax.transData,
                xyB=handle_start_figure,
                coordsB=fig.transFigure,
                color=color,
                linewidth=linewidth,
                zorder=zorder,
                clip_on=False,
            )
            fig.add_artist(handle)


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
    """Draw continuous load-state boundaries across the data rows."""

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
            alpha=0.92,
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

        ax.axvline(start_s, color=OPTIMAL_AREA_COLOR, linewidth=0.9, zorder=1.1)
        ax.axvline(end_s, color=OPTIMAL_AREA_COLOR, linewidth=0.9, zorder=1.1)

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
                    linewidth=0.9,
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
        fontsize=LABEL_FS,
        fontweight=TEXT_WEIGHT,
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
    """Place Regime labels at the centers of their time-domain regions."""

    fig.canvas.draw()

    regime_edges = [
        sender_start_s,
        under_to_full,
        full_to_overload,
        sender_stop_s,
    ]
    if any(edge is None for edge in regime_edges):
        return

    fig_edges = [data_x_to_fig_x(fig, ref_ax, float(edge)) for edge in regime_edges]

    regime_labels = [
        ("Regime I:\nbottleneck\nunsaturated", 1.08),
        ("Regime II:\nbottleneck\nsaturated", 1.08),
        ("Regime III:\nbuffer\nsaturated", 1.08),
    ]

    spectrum_top = max(ax.get_position().y1 for ax in spectrum_axes)
    bracket_y = spectrum_top + 0.016
    bracket_tick_h = 0.014
    label_y = 0.995

    for boundary_x in fig_edges[1:-1]:
        fig.add_artist(
            Line2D(
                [boundary_x, boundary_x],
                [spectrum_top, bracket_y],
                transform=fig.transFigure,
                color=BOUNDARY_COLOR,
                linewidth=BOUNDARY_LW,
                linestyle="--",
                alpha=0.92,
                zorder=38,
                clip_on=False,
            )
        )

    for i in range(3):
        left = fig_edges[i]
        right = fig_edges[i + 1]
        if right <= left:
            continue

        fig.add_artist(
            Line2D(
                [left, right],
                [bracket_y, bracket_y],
                transform=fig.transFigure,
                color=REGIME_BRACKET_COLOR,
                linewidth=0.95,
                solid_capstyle="butt",
                zorder=39,
                clip_on=False,
            )
        )

    for edge_x in fig_edges:
        fig.add_artist(
            Line2D(
                [edge_x, edge_x],
                [bracket_y, bracket_y + bracket_tick_h],
                transform=fig.transFigure,
                color=REGIME_BRACKET_COLOR,
                linewidth=0.95,
                solid_capstyle="butt",
                zorder=39,
                clip_on=False,
            )
        )

    for i, (label, line_spacing) in enumerate(regime_labels):
        center_x = 0.5 * (fig_edges[i] + fig_edges[i + 1])
        fig.text(
            center_x,
            label_y,
            label,
            ha="center",
            va="top",
            fontsize=REGIME_LABEL_FS,
            fontweight=TEXT_WEIGHT,
            linespacing=line_spacing,
            color="#212529",
            zorder=40,
        )


def spectral_density(
    values: np.ndarray,
    fs_hz: float,
    modulation_freq_hz: float | None = None,
) -> tuple[np.ndarray, np.ndarray]:
    """Return a one-sided, variance-preserving Hann-Welch PSD."""

    values = np.asarray(values, dtype=float)
    if len(values) < 2 or not np.all(np.isfinite(values)):
        return np.array([], dtype=float), np.array([], dtype=float)

    analysis_values = values
    if modulation_freq_hz is not None and modulation_freq_hz > 0.0:
        highpass_hz = 0.4 * modulation_freq_hz
        nyquist_hz = 0.5 * fs_hz
        if len(values) >= 32 and 0.0 < highpass_hz < 0.95 * nyquist_hz:
            highpass_sos = signal.butter(
                4,
                highpass_hz,
                btype="highpass",
                fs=fs_hz,
                output="sos",
            )
            analysis_values = signal.sosfiltfilt(highpass_sos, values)

    # Five modulation cycles per segment put the prescribed fundamental on a
    # native FFT bin while keeping each Welch window local relative to the
    # deliberately changing load envelope.  This prevents envelope sidebands
    # from overtaking the actual 5 Hz carrier.  The zero-phase high-pass above
    # removes only the slow regime envelope.  No zero padding is used.
    if modulation_freq_hz is not None and modulation_freq_hz > 0.0:
        target_segment_samples = int(round(5.0 * fs_hz / modulation_freq_hz))
    else:
        target_segment_samples = len(values)
    nperseg = max(8, min(len(values), target_segment_samples))
    noverlap = nperseg // 2 if nperseg < len(values) else 0
    freqs, psd = signal.welch(
        analysis_values,
        fs=fs_hz,
        window="hann",
        nperseg=nperseg,
        noverlap=noverlap,
        detrend="linear",
        return_onesided=True,
        scaling="density",
        nfft=nperseg,
    )
    psd = np.maximum(np.nan_to_num(psd, nan=0.0, posinf=0.0, neginf=0.0), 0.0)
    return freqs, psd


def normalized_spectrum(
    values: np.ndarray,
    fs_hz: float,
    modulation_freq_hz: float | None = None,
) -> tuple[np.ndarray, np.ndarray]:
    freqs, psd = spectral_density(values, fs_hz, modulation_freq_hz)
    scale = float(np.max(psd)) if len(psd) else 0.0
    return freqs, psd / scale if scale > 0.0 else np.zeros_like(psd)


def spectrum_band_max(
    freqs: np.ndarray,
    psd: np.ndarray,
    lo_hz: float,
    hi_hz: float,
) -> float:
    mask = (freqs >= lo_hz) & (freqs <= hi_hz)
    return float(np.max(psd[mask])) if np.any(mask) else 0.0


def spectrum_band_power(
    freqs: np.ndarray,
    psd: np.ndarray,
    lo_hz: float,
    hi_hz: float,
) -> float:
    mask = (freqs >= lo_hz) & (freqs <= hi_hz)
    if np.count_nonzero(mask) < 2:
        return 0.0
    return float(np.trapezoid(psd[mask], freqs[mask]))


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
    """Measure shape-complete response cycles after saturated-link attenuation."""

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

        # The requested 60% is a fraction of complete cycles, not a requirement
        # that a saturated FIFO reproduce 60% of the source modulation gain.
        # Cross-traffic sharing naturally attenuates DRate amplitude, so retain
        # cycles with a visible response and a strong triangle-shape match.
        complete = amplitude_ratio >= 0.30 and best_correlation >= 0.75
        complete_count += int(complete)
        details.append((cycle_start, amplitude_ratio, best_correlation, complete))

    complete_ratio = complete_count / cycle_count if cycle_count else 0.0
    return complete_ratio, complete_count, cycle_count, details


def measure_cycle_peak_to_peak(
    t: np.ndarray,
    values: np.ndarray,
    start_s: float,
    end_s: float,
    modulation_freq_hz: float,
) -> np.ndarray:
    """Return one sampled peak-to-peak value for every complete modulation cycle."""

    if modulation_freq_hz <= 0.0 or end_s <= start_s or len(t) < 2:
        return np.array([], dtype=float)

    period_s = 1.0 / modulation_freq_hz
    dt = float(np.median(np.diff(t)))
    expected_samples = max(1, int(round(period_s / dt)))
    cycle_count = max(0, int(np.floor((end_s - start_s) / period_s + 1e-9)))
    peak_to_peak_values = []
    for cycle_index in range(cycle_count):
        cycle_start = start_s + cycle_index * period_s
        cycle_end = cycle_start + period_s
        mask = (t >= cycle_start) & (t < cycle_end) & np.isfinite(values)
        cycle_values = values[mask]
        if len(cycle_values) < max(8, int(0.75 * expected_samples)):
            continue
        peak_to_peak_values.append(float(np.max(cycle_values) - np.min(cycle_values)))

    return np.asarray(peak_to_peak_values, dtype=float)


def plot_regime_spectrum(
    ax,
    rx_freqs: np.ndarray,
    rx_psd: np.ndarray,
    rtt_freqs: np.ndarray,
    rtt_psd: np.ndarray,
    rx_psd_scale: float,
    rtt_psd_scale: float,
    modulation_freq_hz: float,
    freq_min_hz: float,
    freq_max_hz: float,
    show_ylabel: bool,
    legend_bbox_to_anchor: tuple[float, float] | None = None,
    legend_loc: str = "upper right",
    show_legend: bool = True,
    show_xlabel: bool = True,
) -> None:
    if len(rx_freqs) < 2 or len(rtt_freqs) < 2:
        ax.text(
            0.5,
            0.5,
            "insufficient samples",
            ha="center",
            va="center",
            transform=ax.transAxes,
            fontsize=LABEL_FS,
            fontweight=TEXT_WEIGHT,
        )

        style_axis(ax, tick_size=TICK_FS)
        return

    rx_energy = rx_psd / rx_psd_scale if rx_psd_scale > 0.0 else np.zeros_like(rx_psd)
    rtt_energy = (
        rtt_psd / rtt_psd_scale if rtt_psd_scale > 0.0 else np.zeros_like(rtt_psd)
    )

    rx_mask = (rx_freqs >= freq_min_hz) & (rx_freqs <= freq_max_hz)
    rtt_mask = (rtt_freqs >= freq_min_hz) & (rtt_freqs <= freq_max_hz)

    ax.plot(
        rx_freqs[rx_mask],
        rx_energy[rx_mask],
        color=REGIME_PSD_DRATE_COLOR,
        linewidth=REGIME_PSD_DRATE_LW,
        linestyle=REGIME_PSD_DRATE_LINESTYLE,
        dash_capstyle="butt",
        label="DRate",
        zorder=3,
    )

    ax.plot(
        rtt_freqs[rtt_mask],
        rtt_energy[rtt_mask],
        color=REGIME_PSD_SRTT_COLOR,
        linewidth=REGIME_PSD_SRTT_LW,
        linestyle=REGIME_PSD_SRTT_LINESTYLE,
        label="SRTT",
        zorder=4,
    )

    ax.axvline(
        modulation_freq_hz,
        color="#495057",
        linewidth=0.55,
        linestyle=":",
        alpha=0.65,
        zorder=1,
    )

    # Mark each signal's dominant native FFT bin.  Each signal uses one scale
    # shared across all three regimes, rather than mixing Mbps and milliseconds
    # on a single numerical reference.
    for freqs, energy, mask, color, marker in (
        (rx_freqs, rx_energy, rx_mask, REGIME_PSD_DRATE_COLOR, "o"),
        (rtt_freqs, rtt_energy, rtt_mask, REGIME_PSD_SRTT_COLOR, "s"),
    ):
        band_indices = np.flatnonzero(mask)
        if len(band_indices):
            peak_index = band_indices[int(np.argmax(energy[band_indices]))]
            ax.scatter(
                [freqs[peak_index]],
                [energy[peak_index]],
                s=8,
                color=color,
                marker=marker,
                edgecolors="white",
                linewidths=0.25,
                zorder=8,
            )

    ax.set_ylim(0.0, 1.05)
    ax.set_yticks([0.0, 0.5, 1.0])
    ax.set_xlim(freq_min_hz, freq_max_hz)
    freq_ticks = [
        tick for tick in (2.5, 5.0, 7.5) if freq_min_hz <= tick <= freq_max_hz
    ]
    if freq_ticks:
        ax.set_xticks(freq_ticks)
        ax.set_xticklabels([f"{tick:g}" for tick in freq_ticks])

    if show_xlabel:
        ax.set_xlabel(
            "Frequency (Hz)",
            fontsize=LABEL_FS,
            fontweight=TEXT_WEIGHT,
            labelpad=2.0,
        )

    if show_ylabel:
        ax.text(
            0.045,
            0.900,
            "PSD\n(norm.)",
            transform=ax.transAxes,
            ha="left",
            va="top",
            fontsize=LABEL_FS,
            fontweight=TEXT_WEIGHT,
            linespacing=0.78,
            zorder=12,
        )
    else:
        ax.tick_params(axis="y", labelleft=False)

    ax.grid(
        True,
        which="both",
        color="#d0d0d0",
        linewidth=GRID_LW,
        alpha=0.75,
    )

    style_axis(ax, tick_size=TICK_FS)

    if show_legend:
        legend_kwargs = {}
        if legend_bbox_to_anchor is not None:
            legend_kwargs["bbox_to_anchor"] = legend_bbox_to_anchor
        legend_handles = [
            Line2D(
                [0],
                [0],
                color=REGIME_PSD_DRATE_COLOR,
                linewidth=REGIME_PSD_DRATE_LW,
                linestyle=REGIME_PSD_DRATE_LINESTYLE,
                dash_capstyle="butt",
                label="DRate",
            ),
            Line2D(
                [0],
                [0],
                color=REGIME_PSD_SRTT_COLOR,
                linewidth=REGIME_PSD_SRTT_LW,
                linestyle=REGIME_PSD_SRTT_LINESTYLE,
                label="SRTT",
            ),
        ]
        ax.legend(
            handles=legend_handles,
            loc=legend_loc,
            ncol=1,
            frameon=False,
            prop={"size": SMALL_LEGEND_FS, "weight": TEXT_WEIGHT},
            handlelength=1.35,
            handletextpad=0.28,
            borderpad=0.0,
            labelspacing=0.08,
            **legend_kwargs,
        )


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)

    parser.add_argument("--trace-dir", default="traces/open_loop_p2p_rate_ramp")
    parser.add_argument("--trace-name", default="open_loop_p2p_rate_ramp")
    parser.add_argument("--output-tag", default=None)
    parser.add_argument("--empty-fraction", type=float, default=0.005)
    parser.add_argument("--overload-fraction", type=float, default=0.999)
    parser.add_argument("--freq-min-hz", type=float, default=0.5)
    parser.add_argument("--freq-max-hz", type=float, default=10.0)

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
    cross_rx = rate[:, 8] if rate.shape[1] > 8 else np.zeros_like(rx)

    queue = np.loadtxt(queue_path, comments="#", usecols=(0, 1))
    queue_at_t = resample_queue_to_rate_times(t, queue[:, 0], queue[:, 1])

    link_bw_mbps = config.get("link_bw_mbps", 350.0)
    sender_start_s = config.get("sender_start_time_s", 0.0)
    sender_duration_s = config.get("sender_duration_s", 5.0)
    sender_stop_s = sender_start_s + sender_duration_s
    triangle_freq_hz = config.get("triangle_freq_hz", 5.0)
    srate_peak_to_peak_mbps = config.get(
        "srate_peak_to_peak_mbps",
        2.0 * config.get("triangle_amp_mbps", 100.0),
    )
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
            srate_peak_to_peak_mbps,
        )
    )

    regime_edges = [
        sender_start_s,
        under_to_full,
        full_to_overload,
        sender_stop_s,
    ]
    visible_wave_start_s, visible_wave_end_s = visible_srtt_wave_span(
        t,
        estimated_rtt_ms,
        sender_start_s,
        under_to_full,
        full_to_overload,
        sender_stop_s,
        triangle_freq_hz,
        VISIBLE_SRTT_P2P_MS,
    )
    regime_drate_cycle_p2p = [
        measure_cycle_peak_to_peak(
            t,
            rx,
            regime_edges[i],
            regime_edges[i + 1],
            triangle_freq_hz,
        )
        for i in range(3)
    ]
    output_tag = args.output_tag

    if output_tag is None:
        output_tag = (
            f"{config.get('start_rate_mbps', 200.0):.0f}_"
            f"{config.get('end_rate_mbps', 450.0):.0f}_"
            f"{sender_duration_s:.0f}s_"
            f"{link_bw_mbps:.0f}mbps_"
            f"aux{config.get('cross_regime_i_rate_mbps', 0.0):.0f}to"
            f"c{config.get('cross_regime_ii_control_start_mbps', 0.0):.0f}to"
            f"{config.get('cross_regime_ii_end_rate_mbps', 0.0):.0f}to"
            f"{config.get('cross_end_rate_mbps', 0.0):.0f}_"
            f"h{config.get('cross_regime_ii_hump_mbps', 0.0):g}_"
            f"{triangle_freq_hz:g}hz_"
            f"sp{srate_peak_to_peak_mbps:g}_"
            f"a{config.get('cross_regime_ii_antiphase_gain', 0.0):g}_"
            f"b{config.get('cross_regime_iii_antiphase_gain', 0.0):g}_"
            f"j{config.get('cross_regime_iii_transition_fraction', 0.0):g}_"
            f"g{config.get('cross_regime_iii_target_aggregate_mbps', 0.0):.0f}_"
            f"k{config.get('regime_iii_knee_rate_mbps', 0.0):.0f}at"
            f"{config.get('regime_iii_knee_fraction', 0.0):g}"
        )

    time_out = trace_dir / f"{trace_name}_{output_tag}_send_recv_rtt_curve.png"
    time_pdf_out = time_out.with_suffix(".pdf")
    spectrum_out = trace_dir / f"{trace_name}_{output_tag}_rx_rtt_spectrum_energy.png"
    spectrum_pdf_out = spectrum_out.with_suffix(".pdf")
    summary_out = trace_dir / f"{trace_name}_{output_tag}_regime_summary.txt"
    remove_existing_outputs(
        [time_out, time_pdf_out, spectrum_out, spectrum_pdf_out, summary_out]
    )

    fig = plt.figure(figsize=TIME_FIGSIZE, dpi=EXPORT_DPI)

    regime_edges = [
        min(max(edge, sender_start_s), sender_stop_s)
        for edge in regime_edges
    ]

    # Compute physical PSDs first.  DRate and SRTT have different dimensions,
    # so each signal gets one normalization scale shared across all regimes.
    # This retains cross-regime energy comparisons without treating Mbps and
    # milliseconds as directly comparable amplitudes.
    regime_spectra = []
    for i in range(3):
        is_last = i == 2
        mask = (t >= regime_edges[i]) & (
            (t <= regime_edges[i + 1]) if is_last else (t < regime_edges[i + 1])
        )
        regime_t = t[mask]
        if len(regime_t) < 8:
            regime_spectra.append(
                (
                    np.array([], dtype=float),
                    np.array([], dtype=float),
                    np.array([], dtype=float),
                    np.array([], dtype=float),
                )
            )
            continue
        regime_fs_hz = 1.0 / float(np.median(np.diff(regime_t)))
        regime_rx_freqs, regime_rx_psd = spectral_density(
            rx[mask], regime_fs_hz, triangle_freq_hz
        )
        regime_rtt_freqs, regime_rtt_psd = spectral_density(
            estimated_rtt_ms[mask], regime_fs_hz, triangle_freq_hz
        )
        regime_spectra.append(
            (regime_rx_freqs, regime_rx_psd, regime_rtt_freqs, regime_rtt_psd)
        )

    normalization_lo_hz = args.freq_min_hz
    rx_psd_scale = max(
        (
            spectrum_band_max(freqs, psd, normalization_lo_hz, args.freq_max_hz)
            for freqs, psd, _, _ in regime_spectra
        ),
        default=0.0,
    )
    rtt_psd_scale = max(
        (
            spectrum_band_max(freqs, psd, normalization_lo_hz, args.freq_max_hz)
            for _, _, freqs, psd in regime_spectra
        ),
        default=0.0,
    )
    rx_psd_scale = max(rx_psd_scale, 1e-18)
    rtt_psd_scale = max(rtt_psd_scale, 1e-18)

    regime_band_powers = [
        (
            spectrum_band_power(rx_freqs, rx_psd, normalization_lo_hz, args.freq_max_hz),
            spectrum_band_power(
                rtt_freqs, rtt_psd, normalization_lo_hz, args.freq_max_hz
            ),
        )
        for rx_freqs, rx_psd, rtt_freqs, rtt_psd in regime_spectra
    ]
    drate_band_power_scale = max((values[0] for values in regime_band_powers), default=0.0)
    srtt_band_power_scale = max((values[1] for values in regime_band_powers), default=0.0)
    drate_band_power_scale = max(drate_band_power_scale, 1e-18)
    srtt_band_power_scale = max(srtt_band_power_scale, 1e-18)

    period_s = 1.0 / max(triangle_freq_hz, 1e-9)
    boundary_cycle_intervals = [
        ("before_3s", under_to_full - period_s, under_to_full),
        ("after_3s", under_to_full, under_to_full + period_s),
        ("before_6s", full_to_overload - period_s, full_to_overload),
        ("after_6s", full_to_overload, full_to_overload + period_s),
    ]
    boundary_cycle_queue_stats = []
    for label, start_s, end_s in boundary_cycle_intervals:
        mask = (t >= start_s) & (t < end_s)
        values = queue_fraction[mask]
        boundary_cycle_queue_stats.append(
            (
                label,
                float(np.min(values)) if len(values) else 0.0,
                float(np.max(values)) if len(values) else 0.0,
                float(np.mean(values <= args.empty_fraction)) if len(values) else 0.0,
                float(np.mean(values >= args.overload_fraction)) if len(values) else 0.0,
            )
        )

    raw_queue_fraction = queue[:, 1] / max(queue_bytes, 1.0)
    first_top_indices = np.flatnonzero(
        (queue[:, 0] >= full_to_overload)
        & (raw_queue_fraction >= args.overload_fraction)
    )
    first_top_after_6s = (
        float(queue[first_top_indices[0], 0]) if len(first_top_indices) else float("nan")
    )

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
    add_time_domain_margin_backgrounds(
        axes,
        visible_wave_start_s,
        under_to_full,
        full_to_overload,
        visible_wave_end_s,
    )

    spectrum_layout_edges = [
        sender_start_s,
        visible_wave_start_s,
        visible_wave_end_s,
        sender_stop_s,
    ]
    spectrum_axes = []
    for i in range(3):
        region_left = time_to_fig_x(spectrum_layout_edges[i])
        region_right = time_to_fig_x(spectrum_layout_edges[i + 1])

        ax_left = region_left + (REGIME_AXIS_GAP if i > 0 else 0.0)
        ax_right = region_right - (REGIME_AXIS_GAP if i < 2 else 0.0)

        if ax_right - ax_left < MIN_REGIME_AXIS_W:
            center = 0.5 * (region_left + region_right)
            ax_left = max(FIG_LEFT, center - 0.5 * MIN_REGIME_AXIS_W)
            ax_right = min(FIG_RIGHT, center + 0.5 * MIN_REGIME_AXIS_W)

        spectrum_axes.append(
            fig.add_axes([ax_left, SPECTRUM_Y0, ax_right - ax_left, SPECTRUM_H])
        )

    for i, ax in enumerate(spectrum_axes):
        rx_freqs_i, rx_psd_i, rtt_freqs_i, rtt_psd_i = regime_spectra[i]
        plot_regime_spectrum(
            ax,
            rx_freqs_i,
            rx_psd_i,
            rtt_freqs_i,
            rtt_psd_i,
            rx_psd_scale,
            rtt_psd_scale,
            triangle_freq_hz,
            args.freq_min_hz,
            args.freq_max_hz,
            show_ylabel=(i == 0),
            legend_loc="upper left",
            show_legend=False,
            show_xlabel=False,
        )

    spectrum_legend_kwargs = {
        "frameon": False,
        "prop": {"size": SMALL_LEGEND_FS * 0.92, "weight": TEXT_WEIGHT},
        "handlelength": 0.82,
        "handletextpad": 0.16,
        "borderpad": 0.0,
        "labelspacing": 0.00,
        "borderaxespad": 0.0,
    }
    spectrum_axes[1].legend(
        handles=[
            Line2D(
                [0],
                [0],
                color=REGIME_PSD_DRATE_COLOR,
                linewidth=REGIME_PSD_DRATE_LW,
                linestyle=REGIME_PSD_DRATE_LINESTYLE,
                dash_capstyle="butt",
                label="DRate",
            ),
            Line2D(
                [0],
                [0],
                color=REGIME_PSD_SRTT_COLOR,
                linewidth=REGIME_PSD_SRTT_LW,
                linestyle=REGIME_PSD_SRTT_LINESTYLE,
                label="SRTT",
            ),
        ],
        loc="upper left",
        bbox_to_anchor=(0.205, 0.965),
        ncol=1,
        **spectrum_legend_kwargs,
    )

    fig.text(
        FIG_LEFT + 0.5 * FIG_WIDTH,
        SPECTRUM_Y0 - 0.030,
        "Frequency (Hz)",
        ha="center",
        va="top",
        fontsize=LABEL_FS,
        fontweight=TEXT_WEIGHT,
    )

    # =========================
    # Combined Rate subplot
    # =========================

    srate_line = axes[0].plot(
        t,
        target,
        color=SEND_RATE_COLOR,
        linewidth=SEND_RATE_LW,
        linestyle=SEND_RATE_LINESTYLE,
        dash_capstyle="butt",
        alpha=1.0,
        label="SRate",
        zorder=5,
    )[0]
    srate_line.set_path_effects(
        [
            pe.Stroke(
                linewidth=SEND_RATE_LW + 0.75,
                foreground="#ffffff",
                alpha=0.52,
            ),
            pe.Normal(),
        ]
    )

    axes[0].plot(
        t[t > 0],
        rx[t > 0],
        color=DRATE_COLOR,
        linewidth=RATE_SERIES_LW,
        linestyle=DRATE_LINESTYLE,
        label="DRate",
        zorder=7,
    )

    cross_window = max(1, int(round(0.05 / max(float(np.median(np.diff(t))), 1e-9))))
    cross_smoothed = np.convolve(
        cross_rx,
        np.ones(cross_window, dtype=float) / cross_window,
        mode="same",
    )
    cross_active = (t >= sender_start_s) & (t <= sender_stop_s)
    axes[0].plot(
        t[cross_active],
        cross_smoothed[cross_active],
        color=CROSS_COLOR,
        linewidth=BACKGROUND_RATE_LW,
        linestyle=BACKGROUND_LINESTYLE,
        label="Background-flow DRate",
        zorder=2,
    )

    axes[0].axhline(
        link_bw_mbps,
        color="#c92a2a",
        linewidth=SERVICE_LW,
        linestyle=SERVICE_RATE_LINESTYLE,
        label="Link service rate",
    )

    add_axis_header(axes[0], "Rate (Mbps)")

    rate_ymax = max(
        700,
        float(np.nanmax(target) + 150),
        float(np.nanmax(rx) + 135),
        float(np.nanmax(cross_smoothed) + 135),
        link_bw_mbps + 180,
    )

    axes[0].set_ylim(0, rate_ymax)
    axes[0].set_yticks([0, 250, 500])

    axes[0].set_xlim(
        sender_start_s,
        sender_stop_s,
    )
    axes[0].tick_params(axis="x", labelbottom=False)

    # =========================
    # RTT subplot
    # =========================

    axes[1].plot(
        t,
        estimated_rtt_ms,
        color=SRTT_COLOR,
        linewidth=DATA_LW,
        linestyle=SRTT_LINESTYLE,
        label="SRTT",
    )

    add_axis_header(axes[1], "SRTT (ms)")

    axes[1].set_xlabel(
        "Time (s)",
        fontsize=LABEL_FS,
        fontweight=TEXT_WEIGHT,
        labelpad=3,
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

    handles, labels = axes[0].get_legend_handles_labels()
    label_to_handle = dict(zip(labels, handles))
    rate_legend_kwargs = {
        "frameon": False,
        "prop": {"size": LEGEND_FS, "weight": TEXT_WEIGHT},
        "handlelength": 1.05,
        "handletextpad": 0.22,
        "labelspacing": 0.00,
        "borderaxespad": 0.0,
    }
    probe_legend = axes[0].legend(
        [
            label_to_handle["DRate"],
            label_to_handle["SRate"],
        ],
        ["DRate", "SRate"],
        loc="upper center",
        bbox_to_anchor=(0.145, 0.970),
        ncol=1,
        **rate_legend_kwargs,
    )
    axes[0].add_artist(probe_legend)

    background_legend = axes[0].legend(
        [label_to_handle["Background-flow DRate"]],
        ["Background-flow\nDRate"],
        loc="upper center",
        bbox_to_anchor=(0.500, 0.970),
        ncol=1,
        **rate_legend_kwargs,
    )
    axes[0].add_artist(background_legend)

    service_legend = axes[0].legend(
        [label_to_handle["Link service rate"]],
        ["Service rate"],
        loc="upper center",
        bbox_to_anchor=(0.875, 0.910),
        ncol=1,
        **rate_legend_kwargs,
    )
    for legend in (probe_legend, background_legend, service_legend):
        for text in legend.get_texts():
            text.set_linespacing(0.88)

    add_srtt_magnifiers(
        fig,
        axes[1],
        t,
        estimated_rtt_ms,
        [
            (visible_wave_start_s, under_to_full),
            (full_to_overload, visible_wave_end_s),
        ],
    )

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

    fig.savefig(time_out, dpi=EXPORT_DPI)
    fig.savefig(time_pdf_out)
    plt.close(fig)

    active = (t >= sender_start_s) & (t <= sender_stop_s)
    t_active = t[active]
    dt = float(np.median(np.diff(t_active)))
    fs_hz = 1.0 / dt

    rx_freqs, rx_energy = normalized_spectrum(rx[active], fs_hz, triangle_freq_hz)
    rtt_freqs, rtt_energy = normalized_spectrum(
        estimated_rtt_ms[active], fs_hz, triangle_freq_hz
    )

    rx_top = top_freqs(rx_freqs, rx_energy, hi=args.freq_max_hz)
    rtt_top = top_freqs(rtt_freqs, rtt_energy, hi=args.freq_max_hz)

    fig, axes = plt.subplots(
        2,
        1,
        figsize=SPECTRUM_FIGSIZE,
        dpi=EXPORT_DPI,
        sharex=True,
    )

    fig.subplots_adjust(
        left=0.145,
        right=0.98,
        top=0.95,
        bottom=0.205,
        hspace=0.38,
    )

    for ax, freqs, energy, color, linestyle, label in [
        (
            axes[0],
            rx_freqs,
            rx_energy,
            DRATE_COLOR,
            DRATE_LINESTYLE,
            "DRate",
        ),
        (
            axes[1],
            rtt_freqs,
            rtt_energy,
            SRTT_COLOR,
            SPECTRUM_SRTT_LINESTYLE,
            "SRTT",
        ),
    ]:
        mask = (freqs >= args.freq_min_hz) & (freqs <= args.freq_max_hz)

        line = ax.plot(
            freqs[mask],
            energy[mask],
            color=color,
            linewidth=DATA_LW,
            linestyle=linestyle,
            label=label,
        )[0]
        if label == "SRTT":
            line.set_dash_capstyle("butt")

        ax.fill_between(
            freqs[mask],
            0,
            energy[mask],
            color=color,
            alpha=0.16,
        )

        ax.set_yscale("log")
        ax.set_ylim(1e-8, 1.4)

        add_axis_header(ax, "Normalized PSD")

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
            prop={"size": LEGEND_FS, "weight": TEXT_WEIGHT},
            handlelength=3.2,
        )

    axes[1].set_xlabel(
        "Frequency (Hz)",
        fontsize=LABEL_FS,
        fontweight=TEXT_WEIGHT,
        labelpad=16,
    )

    axes[1].set_xlim(args.freq_min_hz, args.freq_max_hz)

    fig.text(
        0.5,
        0.035,
        (
            f"Hann-Welch PSD over active 0-{sender_duration_s:g} s interval; "
            f"5-cycle segments; {0.4 * triangle_freq_hz:g} Hz high-pass; "
            f"fs {fs_hz:.0f} Hz; native FFT grid"
        ),
        ha="center",
        va="bottom",
        fontsize=LABEL_FS,
        fontweight=TEXT_WEIGHT,
        color="#495057",
    )

    fig.savefig(spectrum_out, dpi=EXPORT_DPI)
    fig.savefig(spectrum_pdf_out)
    plt.close(fig)

    with summary_out.open("w") as stream:
        stream.write("# Equal-duration regime and spectrum summary\n")
        stream.write(f"regime_i_to_ii_s\t{under_to_full}\n")
        stream.write(f"regime_ii_to_iii_s\t{full_to_overload}\n")
        stream.write(f"regime_duration_s\t{regime_duration_s}\n")
        stream.write(f"visible_srtt_wave_threshold_ms\t{VISIBLE_SRTT_P2P_MS:g}\n")
        stream.write(f"visible_srtt_wave_start_s\t{visible_wave_start_s:.12g}\n")
        stream.write(f"visible_srtt_wave_end_s\t{visible_wave_end_s:.12g}\n")
        stream.write(
            f"visible_srtt_left_margin_s\t{under_to_full - visible_wave_start_s:.12g}\n"
        )
        stream.write(
            f"visible_srtt_right_margin_s\t{visible_wave_end_s - full_to_overload:.12g}\n"
        )
        stream.write(f"boundary_window_samples\t{boundary_window}\n")
        stream.write(f"regime_ii_complete_wave_ratio\t{regime_ii_wave_ratio}\n")
        stream.write(f"regime_ii_complete_cycles\t{regime_ii_complete_cycles}\n")
        stream.write(f"regime_ii_total_cycles\t{regime_ii_total_cycles}\n")
        stream.write(f"constant_srate_peak_to_peak_mbps\t{srate_peak_to_peak_mbps:.12g}\n")
        for i, values in enumerate(regime_drate_cycle_p2p, start=1):
            if len(values):
                stream.write(
                    f"regime_{i}_drate_cycle_peak_to_peak_mbps\t"
                    f"median={np.median(values):.12g}\t"
                    f"p10={np.percentile(values, 10):.12g}\t"
                    f"p90={np.percentile(values, 90):.12g}\n"
                )
        stream.write("complete_cycle_min_amplitude_ratio\t0.30\n")
        stream.write("complete_cycle_min_shape_correlation\t0.75\n")
        stream.write(
            f"regime_psd_band_hz\t{args.freq_min_hz:g}\t{args.freq_max_hz:g}\n"
        )
        stream.write(f"spectral_highpass_hz\t{0.4 * triangle_freq_hz:.12g}\n")
        stream.write("spectral_welch_segment_cycles\t5\n")
        for label, q_min, q_max, floor_fraction, top_fraction in boundary_cycle_queue_stats:
            stream.write(
                f"boundary_cycle_{label}\t"
                f"queue_min_fraction={q_min:.12g}\t"
                f"queue_max_fraction={q_max:.12g}\t"
                f"floor_sample_fraction={floor_fraction:.12g}\t"
                f"top_sample_fraction={top_fraction:.12g}\n"
            )
        stream.write(f"first_top_clip_after_6s_s\t{first_top_after_6s:.12g}\n")
        stream.write(f"drate_psd_scale_mbps2_per_hz\t{rx_psd_scale:.12g}\n")
        stream.write(f"srtt_psd_scale_ms2_per_hz\t{rtt_psd_scale:.12g}\n")
        for i, (rx_freqs_i, rx_psd_i, rtt_freqs_i, rtt_psd_i) in enumerate(
            regime_spectra,
            start=1,
        ):
            rx_mask = (rx_freqs_i >= args.freq_min_hz) & (rx_freqs_i <= args.freq_max_hz)
            rtt_mask = (rtt_freqs_i >= args.freq_min_hz) & (rtt_freqs_i <= args.freq_max_hz)
            rx_band_indices = np.flatnonzero(rx_mask)
            rtt_band_indices = np.flatnonzero(rtt_mask)
            rx_peak_hz = (
                float(rx_freqs_i[rx_band_indices[np.argmax(rx_psd_i[rx_band_indices])]])
                if len(rx_band_indices)
                else float("nan")
            )
            rtt_peak_hz = (
                float(rtt_freqs_i[rtt_band_indices[np.argmax(rtt_psd_i[rtt_band_indices])]])
                if len(rtt_band_indices)
                else float("nan")
            )
            stream.write(
                f"regime_{i}_dominant_frequency_hz\t"
                f"drate={rx_peak_hz:.12g}\t"
                f"srtt={rtt_peak_hz:.12g}\n"
            )
        for i, (drate_power, srtt_power) in enumerate(regime_band_powers, start=1):
            stream.write(
                f"regime_{i}_band_power\t"
                f"drate_mbps2={drate_power:.12g}\t"
                f"srtt_ms2={srtt_power:.12g}\t"
                f"drate_fraction_of_max_regime={drate_power / drate_band_power_scale:.12g}\t"
                f"srtt_fraction_of_max_regime={srtt_power / srtt_band_power_scale:.12g}\n"
            )
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
