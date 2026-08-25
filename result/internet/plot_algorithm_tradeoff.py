#!/usr/bin/env python3

from __future__ import annotations

import csv
import math
from pathlib import Path
import re

import matplotlib

matplotlib.use("Agg")
import matplotlib.patheffects as path_effects
import matplotlib.pyplot as plt
from matplotlib import font_manager
from matplotlib.lines import Line2D


ROOT = Path(__file__).resolve().parents[2]
TEST2_REPORT = ROOT / "NS3.27/results/test2/RESULTS.md"
TEST2_SUMMARY = ROOT / "NS3.27/results/test2/summary/scenario_key_metrics.csv"
INTERNET_REPORT = ROOT / "ICC拥塞控制算法编译与性能测试.md"
OUTPUT_DIR = ROOT / "result/internet"
PNG_PATH = OUTPUT_DIR / "algorithm_throughput_latency.png"
PDF_PATH = OUTPUT_DIR / "algorithm_throughput_latency.pdf"

ALGORITHMS = [
    "BBR-R",
    "oBBR",
    "BBRv2+",
    "CUBIC",
    "BBRv2",
    "FBBR",
]
INTERNET_ALGORITHMS = ALGORITHMS
COLORS = {
    "BBR-R": "#0072B2",
    "oBBR": "#D55E00",
    "BBRv2+": "#009E73",
    "CUBIC": "#CC79A7",
    "BBRv2": "#E69F00",
    "FBBR": "#56B4E9",
}
MARKERS = {
    "BBR-R": "o",
    "oBBR": "s",
    "BBRv2+": "^",
    "CUBIC": "D",
    "BBRv2": "v",
    "FBBR": "X",
}
BETTER_ARROW = (0.25, 0.52, 0.48, 0.30)
INTERNET_SOURCE_NAMES = {
    "bbr_r": "BBR-R",
    "bbrv2plus": "BBRv2+",
    "obbr": "oBBR",
    "bbrv2": "BBRv2",
    "fbbr": "FBBR",
    "cubic": "CUBIC",
}


def register_times_new_roman() -> None:
    for font_path in font_manager.findSystemFonts(fontext="ttf"):
        if Path(font_path).stem.lower() == "times_new_roman":
            font_manager.fontManager.addfont(font_path)
            return
    raise RuntimeError("Times New Roman is not installed")


def parse_percent(value: str) -> float:
    return float(value.strip().replace("%", ""))


def parse_milliseconds(value: str) -> float:
    return float(value.strip().replace("ms", "").strip())


def parse_test2_overall() -> dict[str, tuple[float, float]]:
    if not TEST2_REPORT.is_file():
        rows: dict[str, list[tuple[float, float]]] = {
            algorithm: [] for algorithm in ALGORITHMS
        }
        with TEST2_SUMMARY.open(encoding="utf-8", newline="") as handle:
            for row in csv.DictReader(handle):
                algorithm = row.get("algorithm", "")
                if algorithm not in rows:
                    continue
                rows[algorithm].append(
                    (
                        float(row["utilization_pct"]),
                        float(row["mean_queue_delay_ms"]),
                    )
                )
        scenario_counts = {len(rows[algorithm]) for algorithm in ALGORITHMS}
        if not scenario_counts or 0 in scenario_counts or len(scenario_counts) != 1:
            raise RuntimeError(
                "Test2 summary must contain the same positive number of scenarios "
                "for every controller"
            )
        scenario_count = scenario_counts.pop()
        return {
            algorithm: (
                sum(utilization for utilization, _ in rows[algorithm]) / scenario_count,
                sum(delay for _, delay in rows[algorithm]) / scenario_count,
            )
            for algorithm in ALGORITHMS
        }

    rows: dict[str, tuple[float, float]] = {}
    in_overall_table = False
    for raw_line in TEST2_REPORT.read_text(encoding="utf-8").splitlines():
        line = raw_line.strip()
        if line == "## 总体指标":
            in_overall_table = True
            continue
        if in_overall_table and line.startswith("## "):
            break
        if not in_overall_table or not line.startswith("|"):
            continue
        cells = [cell.strip().strip("*") for cell in line.strip("|").split("|")]
        if len(cells) != 9 or cells[0] not in ALGORITHMS:
            continue
        rows[cells[0]] = (parse_percent(cells[3]), parse_milliseconds(cells[5]))
    if set(rows) != set(ALGORITHMS):
        raise RuntimeError(f"Test2 overall table is incomplete: {sorted(rows)}")
    return rows


def parse_internet_data() -> dict[str, tuple[float, float]]:
    source_algorithms = set(INTERNET_SOURCE_NAMES)
    rows: list[tuple[str, float, float, int]] = []
    flow_count: int | None = None
    for raw_line in INTERNET_REPORT.read_text(encoding="utf-8").splitlines():
        line = raw_line.strip()
        if line.startswith("#"):
            flow_match = re.search(r"(\d+)\s*流", line)
            flow_count = int(flow_match.group(1)) if flow_match else None
            continue
        if not line.startswith("|"):
            continue
        cells = [cell.strip().strip("*") for cell in line.strip("|").split("|")]
        if len(cells) == 6 and cells[0] in source_algorithms:
            algorithm_index, throughput_index, delay_index = 0, 1, 2
        elif len(cells) == 7 and cells[1] in source_algorithms:
            algorithm_index, throughput_index, delay_index = 1, 2, 3
        else:
            continue
        if flow_count is None:
            raise RuntimeError(f"Missing flow count before Internet row: {raw_line}")
        rows.append(
            (
                INTERNET_SOURCE_NAMES[cells[algorithm_index]],
                float(cells[throughput_index]),
                float(cells[delay_index]),
                flow_count,
            )
        )
    expected_count = 9 * len(INTERNET_ALGORITHMS)
    if len(rows) != expected_count:
        raise RuntimeError(f"Expected {expected_count} Internet rows, parsed {len(rows)}")
    grouped: dict[str, list[tuple[float, float]]] = {
        algorithm: [] for algorithm in INTERNET_ALGORITHMS
    }
    for algorithm, throughput, delay, flow_count in rows:
        if flow_count not in {3, 10}:
            raise RuntimeError(f"Unexpected Internet flow count: {flow_count}")
        grouped[algorithm].append((throughput / flow_count, delay))
    if any(len(grouped[algorithm]) != 9 for algorithm in INTERNET_ALGORITHMS):
        raise RuntimeError({algorithm: len(grouped[algorithm]) for algorithm in INTERNET_ALGORITHMS})
    return {
        algorithm: (
            sum(throughput for throughput, _ in grouped[algorithm]) / 9.0,
            sum(delay for _, delay in grouped[algorithm]) / 9.0,
        )
        for algorithm in INTERNET_ALGORITHMS
    }


def legend_handles(algorithms: list[str]) -> list[Line2D]:
    return [
        Line2D(
            [],
            [],
            linestyle="none",
            marker=MARKERS[algorithm],
            markerfacecolor=COLORS[algorithm],
            markeredgecolor="white",
            markeredgewidth=0.65,
            markersize=5.8,
            label=algorithm,
        )
        for algorithm in algorithms
    ]


def configure_axis(axis: plt.Axes, xlabel: str, ylabel: str) -> None:
    axis.set_xlabel(xlabel, labelpad=2.0)
    axis.set_ylabel(ylabel, labelpad=1.0)
    axis.grid(True, color="#9A9A9A", alpha=0.24, linewidth=0.45)
    axis.set_axisbelow(True)
    axis.tick_params(which="major", direction="out", length=3.0, width=0.65)
    axis.tick_params(which="minor", length=0)
    for spine in axis.spines.values():
        spine.set_color("#222222")
        spine.set_linewidth(0.7)


def plot_panel(
    axis: plt.Axes,
    data: dict[str, tuple[float, float]],
    algorithms: list[str],
    panel_label: str,
    title: str,
) -> None:
    for algorithm in algorithms:
        throughput, delay = data[algorithm]
        axis.scatter(
            delay,
            throughput,
            s=48,
            marker=MARKERS[algorithm],
            color=COLORS[algorithm],
            edgecolor="white",
            linewidth=0.75,
            zorder=4,
        )
    axis.text(
        -0.13,
        1.035,
        panel_label,
        transform=axis.transAxes,
        fontsize=8.5,
        fontweight="bold",
        ha="left",
        va="bottom",
    )
    axis.text(
        0.5,
        1.035,
        title,
        transform=axis.transAxes,
        fontsize=8.2,
        ha="center",
        va="bottom",
    )
    arrow_x, arrow_y, text_x, text_y = BETTER_ARROW
    axis.annotate(
        "better",
        xy=(arrow_x, arrow_y),
        xycoords="axes fraction",
        xytext=(text_x, text_y),
        textcoords="axes fraction",
        fontsize=7.2,
        fontweight="bold",
        color="#303030",
        ha="left",
        va="bottom",
        bbox={
            "boxstyle": "round,pad=0.16",
            "facecolor": "white",
            "edgecolor": "none",
            "alpha": 0.78,
        },
        arrowprops={
            "arrowstyle": "-|>",
            "color": "#303030",
            "linewidth": 1.35,
            "mutation_scale": 13,
        },
    )

def main() -> None:
    register_times_new_roman()
    test2_data = parse_test2_overall()
    internet_data = parse_internet_data()

    style = {
        "font.family": "Times New Roman",
        "font.size": 7.6,
        "axes.labelsize": 8.0,
        "xtick.labelsize": 7.2,
        "ytick.labelsize": 7.2,
        "pdf.fonttype": 42,
        "ps.fonttype": 42,
        "axes.linewidth": 0.7,
    }
    with plt.rc_context(style):
        figure, axes = plt.subplots(1, 2, figsize=(3.25, 1.98))
        figure.subplots_adjust(left=0.16, right=0.985, bottom=0.21, top=0.78, wspace=0.34)

        test2_axis, internet_axis = axes
        configure_axis(test2_axis, "Queue delay (ms)", "Goodput util. (%)")
        test2_axis.set_xlim(0, 110)
        test2_axis.set_ylim(94.8, 97.3)
        test2_axis.set_xticks([0, 20, 40, 60, 80, 100])
        test2_axis.set_yticks([95.0, 95.5, 96.0, 96.5, 97.0])
        plot_panel(test2_axis, test2_data, ALGORITHMS, "(a) Static paths", "")

        configure_axis(
            internet_axis,
            "Queue delay (ms)",
            "Goodput (Mbit/s/flow)",
        )
        internet_axis.set_xlim(0, 18.5)
        internet_upper = max(
            100.0,
            math.ceil(max(value[0] for value in internet_data.values()) * 1.1 / 25.0)
            * 25.0,
        )
        internet_axis.set_ylim(0.0, internet_upper)
        internet_axis.set_xticks([0, 5, 10, 15])
        internet_axis.set_yticks(list(range(0, int(internet_upper), 50)))
        plot_panel(
            internet_axis,
            internet_data,
            INTERNET_ALGORITHMS,
            "(b) Internet paths",
            "",
        )

        figure.legend(
            handles=legend_handles(ALGORITHMS),
            loc="upper center",
            bbox_to_anchor=(0.57, 0.94),
            ncol=len(ALGORITHMS),
            fontsize=7.2,
            frameon=False,
            borderaxespad=0.0,
            handletextpad=0.22,
            columnspacing=0.42,
            handlelength=0.8,
        )

        figure.savefig(PNG_PATH, dpi=600, facecolor="white", bbox_inches="tight", pad_inches=0.015)
        figure.savefig(PDF_PATH, facecolor="white", bbox_inches="tight", pad_inches=0.015)
        plt.close(figure)

    print(f"Wrote {PNG_PATH} ({PNG_PATH.stat().st_size} bytes)")
    print(f"Wrote {PDF_PATH} ({PDF_PATH.stat().st_size} bytes)")
    print("Test2:", test2_data)
    print("Internet:", internet_data)


if __name__ == "__main__":
    main()
