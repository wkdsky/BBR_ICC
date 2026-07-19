#!/usr/bin/env python3
"""
Emit a per-cruise frequency-domain score table.

For every cruise, read the FBBR per-flow
  <run>/*_flowN_<cc>_cruise_best_full_load_window.csv
file and print, for the single window that was selected as the best full-load
window in that cruise, the delivery-rate and srtt frequency-domain scores.

The mean delivery rate over the selected window is also computed by averaging
the per-packet samples from the matching
  <run>/*_flowN_<cc>_recvrate_raw.txt
trace (kbps -> Mbps), which is the same raw trace that the FBBR debug plots
use for delivery-rate overlays.

Output is a fixed-width plain-text table written to stdout (and optionally to a
file via --output).
"""

import argparse
import csv
import math
from pathlib import Path
from typing import Dict, Iterable, List

import re

FLOW_RE = re.compile(r"flow(\d+)", re.IGNORECASE)


def flow_id_from_path(path: Path) -> int:
    match = FLOW_RE.search(path.name)
    return int(match.group(1)) if match else 0


def to_float(value: object, default: float = math.nan) -> float:
    if value is None:
        return default
    text = str(value).strip()
    if not text or text.lower() in {"nan", "none", "null"}:
        return default
    try:
        return float(text)
    except ValueError:
        return default


def find_best_window_files(run_dir: Path) -> List[Path]:
    return sorted(run_dir.glob("*_flow*_*_cruise_best_full_load_window.csv"))


def find_raw_recvrate_files(run_dir: Path) -> Dict[int, Path]:
    by_flow: Dict[int, Path] = {}
    for path in sorted(run_dir.glob("*_flow*_*_recvrate_raw.txt")):
        flow_id = flow_id_from_path(path)
        if flow_id > 0:
            by_flow[flow_id] = path
    return by_flow


def load_raw_recvrate_window(path: Path, start_s: float, end_s: float) -> float:
    """Mean delivery rate (Mbps) of samples within [start_s, end_s]."""
    if path is None or not path.exists():
        return math.nan
    total = 0.0
    count = 0
    with path.open("r", encoding="utf-8", errors="replace") as fh:
        for line in fh:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            fields = re.split(r"\s+", line)
            if len(fields) < 2:
                continue
            try:
                time_s = float(fields[0])
                value_kbps = float(fields[1])
            except ValueError:
                continue
            if start_s <= time_s <= end_s:
                total += value_kbps / 1000.0
                count += 1
    if count == 0:
        return math.nan
    return total / count


def collect_best_window_rows(
    run_dir: Path,
    recvrate_by_flow: Dict[int, Path],
) -> List[Dict[str, object]]:
    rows: List[Dict[str, object]] = []
    for path in find_best_window_files(run_dir):
        flow_id = flow_id_from_path(path)
        if flow_id <= 0:
            continue
        with path.open("r", encoding="utf-8", errors="replace", newline="") as fh:
            reader = csv.DictReader(fh)
            for raw in reader:
                win_start = to_float(raw.get("best_window_start_time"))
                win_end = to_float(raw.get("best_window_end_time"))
                # Skip cruises that found no full-load candidate window; they
                # would only clutter the score table with sentinel -1 / 0 rows.
                if not (math.isfinite(win_start) and math.isfinite(win_end)):
                    continue
                if win_start <= 0.0 or win_end <= 0.0:
                    continue
                if win_end <= win_start:
                    continue
                recv_path = recvrate_by_flow.get(flow_id)
                win_mean_drate_mbps = load_raw_recvrate_window(
                    recv_path, win_start, win_end
                )
                rows.append(
                    {
                        "flow_id": flow_id,
                        "cruise_id": int(to_float(raw.get("cruise_id"), 0.0)),
                        "cruise_start_time": to_float(raw.get("cruise_start_time")),
                        "cruise_end_time": to_float(raw.get("cruise_end_time")),
                        "best_window_start_time": win_start,
                        "best_window_end_time": win_end,
                        "best_full_load_quality": to_float(raw.get("best_full_load_quality")),
                        "best_drate_freq_score": to_float(raw.get("best_drate_freq_score")),
                        "best_srtt_freq_score": to_float(raw.get("best_srtt_freq_score")),
                        "win_mean_drate_mbps": win_mean_drate_mbps,
                    }
                )
    rows.sort(key=lambda item: (int(item["flow_id"]), float(item["cruise_end_time"])))
    return rows


def format_value(value: float, width: int, precision: int = 4) -> str:
    if not math.isfinite(value):
        return " ".rjust(width)
    return f"{value:>{width}.{precision}f}"


def format_time(value: float, width: int) -> str:
    if not math.isfinite(value):
        return " ".rjust(width)
    return f"{value:>{width}.3f}"


def format_int(value: int, width: int) -> str:
    return f"{value:>{width}d}"


def render_table(rows: List[Dict[str, object]], run_dir: Path) -> str:
    headers = [
        "flow",
        "cruise",
        "cruise_start_s",
        "cruise_end_s",
        "win_start_s",
        "win_end_s",
        "quality",
        "drate_freq",
        "srtt_freq",
        "win_mean_drate_mbps",
    ]
    widths = [4, 6, 13, 12, 12, 11, 9, 11, 11, 19]
    header_line = " ".join(h.ljust(w) for h, w in zip(headers, widths))
    sep_line = " ".join("-" * w for w in widths)

    lines: List[str] = []
    lines.append(f"FBBR per-cruise best-window frequency-domain scores")
    lines.append(f"run_dir: {run_dir}")
    lines.append("")
    lines.append(header_line)
    lines.append(sep_line)

    last_flow: int = -1
    for row in rows:
        flow_id = int(row["flow_id"])
        if last_flow != flow_id and last_flow != -1:
            lines.append("")
        last_flow = flow_id
        lines.append(
            " ".join(
                [
                    format_int(flow_id, widths[0]),
                    format_int(int(row["cruise_id"]), widths[1]),
                    format_time(float(row["cruise_start_time"]), widths[2]),
                    format_time(float(row["cruise_end_time"]), widths[3]),
                    format_time(float(row["best_window_start_time"]), widths[4]),
                    format_time(float(row["best_window_end_time"]), widths[5]),
                    format_value(float(row["best_full_load_quality"]), widths[6], precision=4),
                    format_value(float(row["best_drate_freq_score"]), widths[7], precision=4),
                    format_value(float(row["best_srtt_freq_score"]), widths[8], precision=4),
                    format_value(float(row["win_mean_drate_mbps"]), widths[9], precision=3),
                ]
            )
        )

    finite_dr = [r for r in rows if math.isfinite(float(r["best_drate_freq_score"]))]
    finite_sr = [r for r in rows if math.isfinite(float(r["best_srtt_freq_score"]))]
    finite_q = [r for r in rows if math.isfinite(float(r["best_full_load_quality"]))]
    finite_dr_mean = [r for r in rows if math.isfinite(float(r["win_mean_drate_mbps"]))]
    lines.append(sep_line)
    if finite_dr and finite_sr and finite_q:
        mean_q = sum(float(r["best_full_load_quality"]) for r in finite_q) / len(finite_q)
        mean_dr = sum(float(r["best_drate_freq_score"]) for r in finite_dr) / len(finite_dr)
        mean_sr = sum(float(r["best_srtt_freq_score"]) for r in finite_sr) / len(finite_sr)
        if finite_dr_mean:
            mean_drate_mbps = (
                sum(float(r["win_mean_drate_mbps"]) for r in finite_dr_mean)
                / len(finite_dr_mean)
            )
        else:
            mean_drate_mbps = math.nan
        lines.append(
            " ".join(
                [
                    "MEAN".ljust(widths[0]),
                    " ".ljust(widths[1]),
                    " ".ljust(widths[2]),
                    " ".ljust(widths[3]),
                    " ".ljust(widths[4]),
                    " ".ljust(widths[5]),
                    format_value(mean_q, widths[6], precision=4),
                    format_value(mean_dr, widths[7], precision=4),
                    format_value(mean_sr, widths[8], precision=4),
                    format_value(mean_drate_mbps, widths[9], precision=3),
                ]
            )
        )

    by_flow_counts: Dict[int, int] = {}
    for row in rows:
        flow_id = int(row["flow_id"])
        by_flow_counts[flow_id] = by_flow_counts.get(flow_id, 0) + 1
    lines.append("")
    lines.append("per-flow valid-cruise counts:")
    for flow_id in sorted(by_flow_counts):
        lines.append(f"  flow{flow_id}: {by_flow_counts[flow_id]}")
    lines.append(f"total cruises with selected best window: {len(rows)}")
    return "\n".join(lines) + "\n"


def parse_args(argv: Iterable[str] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Emit per-cruise drate/srtt frequency-domain score table."
    )
    parser.add_argument("--fbbr-run-dir", dest="run_dir", required=True, help="FBBR run directory.")
    parser.add_argument(
        "--output",
        default="",
        help="Output txt path. Defaults to <FBBR_RUN_DIR>/debug_plots/cruise_freq_scores.txt.",
    )
    return parser.parse_args(argv)


def main(argv: Iterable[str] = None) -> int:
    args = parse_args(argv)
    run_dir = Path(args.run_dir).expanduser().resolve()
    if not run_dir.exists():
        raise SystemExit(f"Run directory does not exist: {run_dir}")
    output_path = (
        Path(args.output).expanduser().resolve()
        if args.output
        else run_dir / "debug_plots" / "cruise_freq_scores.txt"
    )
    output_path.parent.mkdir(parents=True, exist_ok=True)

    rows = collect_best_window_rows(run_dir, find_raw_recvrate_files(run_dir))
    if not rows:
        raise SystemExit(
            f"No *_cruise_best_full_load_window.csv files found under {run_dir}"
        )

    rendered = render_table(rows, run_dir)
    output_path.write_text(rendered, encoding="utf-8")
    print(rendered)
    print(f"[cruise_freq_scores] wrote {output_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
