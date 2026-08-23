"""Joint torques from foot forces: tau = -J^T F on actuated dofs."""

from __future__ import annotations

import mujoco
import numpy as np


def map_jt(model, data, geom_ids: np.ndarray, grf_world: np.ndarray) -> np.ndarray:
    """``grf_world`` is (4, 3) floor-to-foot. Returns actuator tau (12,)."""
    nv = int(model.nv)
    tau_full = np.zeros(nv, dtype=np.float64)
    jacp = np.zeros((3, nv), dtype=np.float64)
    forces = np.asarray(grf_world, dtype=np.float64).reshape(4, 3)
    for geom_id, force in zip(geom_ids, forces, strict=True):
        mujoco.mj_jacGeom(model, data, jacp, None, int(geom_id))
        tau_full -= jacp.T @ force
    nu = int(model.nu)
    return tau_full[-nu:]
