#!/usr/bin/env python3
"""Run sampling MPC labs (IROS 2024 track) and write sampling_lab_results.json."""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))

from sampling_labs import SAMPLING_LABS, run_lab, save_results  # noqa: E402


def main() -> int:
    parser = argparse.ArgumentParser(description="Run sampling MPC workshop labs")
    parser.add_argument("--lab", help="Single lab id (default: all)")
    parser.add_argument("--compare-nominal", action="store_true", help="Also run matching nominal preset")
    parser.add_argument("--list", action="store_true")
    args = parser.parse_args()

    if args.list:
        for lab in SAMPLING_LABS:
            print(f"{lab.id:22} session={lab.session}  {lab.title}")
        return 0

    labs = SAMPLING_LABS
    if args.lab:
        labs = [l for l in SAMPLING_LABS if l.id == args.lab]
        if not labs:
            raise SystemExit(f"Unknown lab {args.lab!r}")

    entries = []
    for lab in labs:
        print(f"\n=== {lab.id}: {lab.title} ===")
        entry = run_lab(lab.id, compare_nominal=args.compare_nominal)
        r = entry["result"]
        if "distance_m" in r and lab.run_fn != "flat":
            print(f"  sampling: {r['distance_m']:.2f} m  falls={r.get('falls', '-')}  success={r.get('success')}")
        else:
            print(f"  sampling: mean_vx={r.get('mean_vx', 0):.3f}  terminated={r.get('terminated')}")
        if "nominal_result" in entry:
            nr = entry["nominal_result"]
            if "distance_m" in nr and lab.run_fn != "flat":
                print(f"  nominal:  {nr['distance_m']:.2f} m  falls={nr.get('falls', '-')}")
            else:
                print(f"  nominal:  mean_vx={nr.get('mean_vx', 0):.3f}  terminated={nr.get('terminated')}")
        entries.append(entry)

    path = save_results(entries)
    print(f"\nWrote {path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
