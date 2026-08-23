"""Clip joint torques to MuJoCo actuator limits."""

from __future__ import annotations

import numpy as np


def clip_torque(tau: np.ndarray, ctrl_range: np.ndarray) -> np.ndarray:
    """``ctrl_range`` is ``(nu, 2)`` low/high from ``mjModel.actuator_ctrlrange``."""
    u = np.asarray(tau, dtype=np.float64).reshape(-1)
    lo = np.asarray(ctrl_range[:, 0], dtype=np.float64)
    hi = np.asarray(ctrl_range[:, 1], dtype=np.float64)
    if u.shape[0] != lo.shape[0]:
        raise ValueError(f"tau length {u.shape[0]} != nu {lo.shape[0]}")
    return np.clip(u, lo, hi)
