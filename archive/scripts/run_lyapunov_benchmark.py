#!/usr/bin/env python3
"""Run Lyapunov MPC labs (RAL 2025 track) and write lyapunov_lab_results.json."""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))

from lyapunov_labs import LYAPUNOV_LABS, run_lab, save_results  # noqa: E402


def main() -> int:
    parser = argparse.ArgumentParser(description="Run Lyapunov MPC workshop labs")
    parser.add_argument("--lab", help="Single lab id")
    parser.add_argument("--list", action="store_true")
    args = parser.parse_args()

    if args.list:
        for lab in LYAPUNOV_LABS:
            print(f"{lab.id:22} session={lab.session}  {lab.title}")
        return 0

    labs = LYAPUNOV_LABS
    if args.lab:
        labs = [l for l in LYAPUNOV_LABS if l.id == args.lab]
        if not labs:
            raise SystemExit(f"Unknown lab {args.lab!r}")

    entries = []
    for lab in labs:
        print(f"\n=== {lab.id}: {lab.title} ===")
        entry = run_lab(lab.id)
        r = entry["result"]
        if "distance_m" in r and lab.run_fn != "flat":
            print(f"  {r['distance_m']:.2f} m  falls={r.get('falls')}  success={r.get('success')}")
        else:
            print(f"  mean_vx={r.get('mean_vx', 0):.3f}  terminated={r.get('terminated')}")
        entries.append(entry)

    print(f"\nWrote {save_results(entries)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
