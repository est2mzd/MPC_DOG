"""Phase-0 first demo: Go2 under gravity, zero joint torque."""

from __future__ import annotations

from pathlib import Path

from mpc_dog.plant.mujoco_go2 import MujocoGo2
from mpc_dog.viz.gif import render_rollout_gif

ASSETS = Path(__file__).resolve().parents[3] / "notebook" / "assets"


def run_gravity_demo(out_gif: Path | None = None) -> Path:
    """Stand-ish keyframe, then τ=0. GIF shows actual foot GRF, net contact, and mg."""
    dest = Path(out_gif) if out_gif is not None else ASSETS / "00_mujoco_go2_demo.gif"
    plant = MujocoGo2(scene="flat", seed=0)
    return render_rollout_gif(
        plant,
        dest,
        n_steps=1200,
        capture_every=24,
        title="00 gravity demo  tau=0",
    )
