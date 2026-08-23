"""Joint-space PD. Used for kinematic height change, not as a WBC."""

from __future__ import annotations

import numpy as np

NQ_FREE = 7
NV_FREE = 6
NU = 12


def joint_pd(qpos: np.ndarray, qvel: np.ndarray, q_des: np.ndarray, kp: float, kd: float) -> np.ndarray:
    """``tau = kp (q_des - q) - kd qdot`` on the 12 actuated joints."""
    qj = np.asarray(qpos, dtype=np.float64).reshape(-1)[NQ_FREE : NQ_FREE + NU]
    vj = np.asarray(qvel, dtype=np.float64).reshape(-1)[NV_FREE : NV_FREE + NU]
    qd = np.asarray(q_des, dtype=np.float64).reshape(NU)
    return kp * (qd - qj) - kd * vj


def lerp(a: np.ndarray, b: np.ndarray, s: float) -> np.ndarray:
    u = float(np.clip(s, 0.0, 1.0))
    aa = np.asarray(a, dtype=np.float64)
    bb = np.asarray(b, dtype=np.float64)
    return (1.0 - u) * aa + u * bb
