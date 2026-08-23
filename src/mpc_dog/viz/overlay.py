"""Add analysis arrows (GRF, net force) onto a MuJoCo render scene."""

from __future__ import annotations

import numpy as np

from mpc_dog.types.layouts import LEG_ORDER

# N -> metres of arrow length. Go2 weight ~150 N so 0.002 m/N ≈ 0.3 m down.
FORCE_SCALE_M_PER_N = 0.002
ARROW_RADIUS = 0.012

LEG_RGBA = {
    "FL": np.array([0.15, 0.55, 0.95, 0.95], dtype=np.float32),
    "FR": np.array([0.20, 0.80, 0.35, 0.95], dtype=np.float32),
    "RL": np.array([0.95, 0.55, 0.15, 0.95], dtype=np.float32),
    "RR": np.array([0.85, 0.20, 0.75, 0.95], dtype=np.float32),
}
NET_RGBA = np.array([0.95, 0.15, 0.15, 0.95], dtype=np.float32)
WEIGHT_RGBA = np.array([0.45, 0.45, 0.45, 0.80], dtype=np.float32)


def _add_arrow(scene, start: np.ndarray, vec: np.ndarray, rgba: np.ndarray) -> None:
    import mujoco

    length = float(np.linalg.norm(vec))
    if length < 1e-6 or scene.ngeom >= scene.maxgeom:
        return
    end = start + vec
    scene.ngeom += 1
    geom = scene.geoms[scene.ngeom - 1]
    mujoco.mjv_initGeom(
        geom,
        mujoco.mjtGeom.mjGEOM_CAPSULE,
        np.zeros(3),
        np.zeros(3),
        np.zeros(9),
        np.asarray(rgba, dtype=np.float32),
    )
    mujoco.mjv_connector(
        geom,
        mujoco.mjtGeom.mjGEOM_ARROW,
        ARROW_RADIUS,
        np.asarray(start, dtype=np.float64),
        np.asarray(end, dtype=np.float64),
    )


def overlay_contact_and_net(
    scene,
    feet_pos: np.ndarray,
    contact_forces: np.ndarray,
    com: np.ndarray,
    net_force: np.ndarray,
    weight: np.ndarray | None = None,
    *,
    scale: float = FORCE_SCALE_M_PER_N,
) -> None:
    """Draw per-foot contact (actual) and CoM net / weight arrows.

    ``contact_forces`` and ``net_force`` are Newtons in world frame.
    Weight is ``m * g`` (already a force). Caption gravity inclusion in the Notebook.
    """
    for i, leg in enumerate(LEG_ORDER):
        _add_arrow(scene, feet_pos[i], contact_forces[i] * scale, LEG_RGBA[leg])
    _add_arrow(scene, com, net_force * scale, NET_RGBA)
    if weight is not None:
        _add_arrow(scene, com, weight * scale, WEIGHT_RGBA)
