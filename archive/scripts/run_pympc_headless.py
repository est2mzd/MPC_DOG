#!/usr/bin/env python3
"""Apply mpc_dog preset and run Quadruped-PyMPC simulation headless."""
from __future__ import annotations

import argparse
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
PYMPC = ROOT / "external" / "Quadruped-PyMPC"


def main() -> int:
    parser = argparse.ArgumentParser(description="Headless PyMPC sim for mpc_dog presets")
    parser.add_argument("preset", help="Preset name without .yaml, e.g. session01_flat_smoke")
    parser.add_argument("--seconds", type=float, default=8.0, help="Sim duration per episode")
    parser.add_argument(
        "--vel-cmd",
        default="forward",
        choices=("forward", "random", "forward+rotate", "human"),
    )
    args = parser.parse_args()

    session_id = Path(args.preset).stem
    log_dir = ROOT / "logs" / "pympc_sessions" / session_id
    log_dir.mkdir(parents=True, exist_ok=True)
    log_path = log_dir / "headless_sim.log"

    subprocess.run(
        [sys.executable, str(ROOT / "scripts" / "apply_pympc_preset.py"), args.preset],
        check=True,
    )

    runner = f"""
import sys
sys.path.insert(0, {repr(str(PYMPC))})
from quadruped_pympc import config as cfg
from simulation.simulation import run_simulation

run_simulation(
    qpympc_cfg=cfg,
    num_episodes=1,
    num_seconds_per_episode={args.seconds!r},
    render=False,
    base_vel_command_type={args.vel_cmd!r},
    ref_base_lin_vel=(0.4, 0.6),
)
print("HEADLESS_SIM_OK")
"""
    with log_path.open("w", encoding="utf-8") as log_f:
        proc = subprocess.run(
            [sys.executable, "-c", runner],
            cwd=PYMPC,
            stdout=log_f,
            stderr=subprocess.STDOUT,
            check=False,
        )
    print(f"log: {log_path}")
    if proc.returncode != 0:
        print(f"FAILED exit={proc.returncode}", file=sys.stderr)
        with log_path.open(encoding="utf-8") as log_f:
            tail = log_f.read()[-8000:]
        print(tail, file=sys.stderr)
        return proc.returncode
    print("HEADLESS_SIM_OK")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
