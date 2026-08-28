"""Set MUJOCO_GL before gym_quadruped / mujoco are imported."""

from __future__ import annotations

import os


def ensure_mujoco_gl() -> str:
    if not os.environ.get("MUJOCO_GL"):
        os.environ["MUJOCO_GL"] = "egl"
    return os.environ["MUJOCO_GL"]
