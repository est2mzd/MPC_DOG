"""S2 height demos: kinematic squat, then EqualShare plus height P."""

from __future__ import annotations

from pathlib import Path

import numpy as np

from mpc_dog.force.height_p import height_p_share
from mpc_dog.joint.map_jt import map_jt
from mpc_dog.joint.pd import joint_pd, lerp
from mpc_dog.plant.mujoco_go2 import MujocoGo2
from mpc_dog.viz.gif import render_rollout_gif

# Standing keyframe from gym-quadruped Go2. Squat folds thigh/calf ~3 cm.
Q_STAND = np.array([0.0, 0.9, -1.8] * 4, dtype=np.float64)
Q_SQUAT = np.array([0.0, 1.08, -2.05] * 4, dtype=np.float64)

Z_NOM = 0.29
Z_LOW = 0.26


def _piecewise_s(t: float, t0: float, t1: float) -> float:
    if t <= t0:
        return 0.0
    if t >= t1:
        return 1.0
    return (t - t0) / (t1 - t0)


def squat_q_des(t: float) -> np.ndarray:
    """Stand → squat → stand. Times in seconds after first step."""
    if t < 1.0:
        return Q_STAND.copy()
    if t < 2.5:
        return lerp(Q_STAND, Q_SQUAT, _piecewise_s(t, 1.0, 2.5))
    if t < 4.0:
        return Q_SQUAT.copy()
    if t < 5.5:
        return lerp(Q_SQUAT, Q_STAND, _piecewise_s(t, 4.0, 5.5))
    return Q_STAND.copy()


def z_ref_schedule(t: float) -> float:
    if t < 1.0:
        return Z_NOM
    if t < 3.5:
        return Z_LOW
    return Z_NOM


def run_height_kinematic_demo(out_gif: Path) -> tuple[Path, dict[str, np.ndarray]]:
    """S2-A: joint interpolation. No EqualShare, no horizon."""
    plant = MujocoGo2(scene="flat", seed=0)
    log_t: list[float] = []
    log_z: list[float] = []

    def tau_fn(p: MujocoGo2) -> np.ndarray:
        q_des = squat_q_des(float(p.data.time))
        log_t.append(float(p.data.time))
        log_z.append(float(p.base_pos()[2]))
        return joint_pd(p.data.qpos, p.data.qvel, q_des, kp=40.0, kd=2.0)

    path = render_rollout_gif(
        plant,
        Path(out_gif),
        n_steps=3250,
        capture_every=50,
        tau_fn=tau_fn,
        extra_lines=lambda p: [f"S2-A kinematic  q_des={'squat' if squat_q_des(float(p.data.time))[1] > 1.0 else 'stand'}"],
        title="03a height  joint lerp (no GRF command)",
    )
    return path, {"t": np.asarray(log_t), "z": np.asarray(log_z)}


def run_height_p_demo(out_gif: Path) -> tuple[Path, dict[str, np.ndarray]]:
    """S2-B: Fz_total = mg + kp(zref-z) - kd vz, split equally."""
    plant = MujocoGo2(scene="flat", seed=0)
    geoms = plant.foot_geom_ids()
    log_t: list[float] = []
    log_z: list[float] = []
    log_zr: list[float] = []

    def cmd_fn(p: MujocoGo2) -> np.ndarray:
        z_ref = z_ref_schedule(float(p.data.time))
        return height_p_share(
            p.mass_kg,
            float(p.base_pos()[2]),
            z_ref,
            float(p.data.qvel[2]),
            kp=1800.0,
            kd=120.0,
        )

    def tau_fn(p: MujocoGo2) -> np.ndarray:
        cmd = cmd_fn(p)
        log_t.append(float(p.data.time))
        log_z.append(float(p.base_pos()[2]))
        log_zr.append(z_ref_schedule(float(p.data.time)))
        return map_jt(p.model, p.data, geoms, cmd)

    path = render_rollout_gif(
        plant,
        Path(out_gif),
        n_steps=3250,
        capture_every=50,
        tau_fn=tau_fn,
        command_grf=cmd_fn,
        extra_lines=lambda p: [f"z_ref={z_ref_schedule(float(p.data.time)):.3f} m  height P on Fz"],
        title="03b height  EqualShare + height P",
    )
    return path, {"t": np.asarray(log_t), "z": np.asarray(log_z), "z_ref": np.asarray(log_zr)}
