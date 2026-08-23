"""Apply a large raw torque, clip to actuator limits, record GIF."""

from __future__ import annotations

from pathlib import Path

import numpy as np

from mpc_dog.joint.clip import clip_torque
from mpc_dog.plant.mujoco_go2 import MujocoGo2
from mpc_dog.viz.gif import render_rollout_gif

ASSETS = Path(__file__).resolve().parents[3] / "notebook" / "assets"


def run_clip_demo(out_gif: Path | None = None) -> tuple[Path, np.ndarray, np.ndarray]:
    dest = Path(out_gif) if out_gif is not None else ASSETS / "01_clip_torque.gif"
    plant = MujocoGo2(scene="flat", seed=0)
    raw = np.full(12, 200.0, dtype=np.float64)
    clipped = clip_torque(raw, plant.model.actuator_ctrlrange)
    path = render_rollout_gif(
        plant,
        dest,
        n_steps=1200,
        capture_every=24,
        tau=clipped,
        title="01 clip  raw=200 N·m  applied=sat",
    )
    return path, raw, clipped
