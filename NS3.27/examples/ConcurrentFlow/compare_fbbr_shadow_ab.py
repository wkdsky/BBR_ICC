#!/usr/bin/env python3
"""Bitwise comparison for validation shadow-window control-path isolation."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path


FILES = [
    "flow1_sent_audit.csv",
    "flow1_acked_audit.csv",
    "flow1_pacing_audit.csv",
    "fbbr_validation_flow1_F-BBR_bbrmode.txt",
    "flow1_fbbr_opiv2_bins.csv",
    "flow1_fbbr_opiv2_blocks.csv",
    "flow1_fbbr_opiv2_cruises.csv",
]


def digest(path: Path) -> str:
    result = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            result.update(chunk)
    return result.hexdigest()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("off", type=Path)
    parser.add_argument("on", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    rows = []
    for name in FILES:
        left, right = args.off / name, args.on / name
        exists = left.exists() and right.exists()
        left_hash = digest(left) if exists else ""
        right_hash = digest(right) if exists else ""
        rows.append(
            {
                "file": name, "both_exist": exists,
                "off_sha256": left_hash, "on_sha256": right_hash,
                "bitwise_identical": exists and left_hash == right_hash,
            }
        )
    result = {
        "comparisons": rows,
        "shadow_trace_exists_only_when_enabled": not (
            args.off / "flow1_fbbr_opiv2_shadow_windows.csv"
        ).exists() and (args.on / "flow1_fbbr_opiv2_shadow_windows.csv").exists(),
    }
    result["control_path_bitwise_identical"] = all(
        row["bitwise_identical"] for row in rows
    )
    result["pass"] = result["control_path_bitwise_identical"] and result[
        "shadow_trace_exists_only_when_enabled"
    ]
    text = json.dumps(result, indent=2) + "\n"
    if args.output:
        args.output.write_text(text)
    print(text, end="")
    return 0 if result["pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
