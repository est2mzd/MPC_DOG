"""MuJoCo Go2 plant. gym-quadruped is the XML + step API; quadruped_pympc is not imported."""

from __future__ import annotations

from mpc_dog.viz.gl import ensure_mujoco_gl

ensure_mujoco_gl()

import numpy as np
from gym_quadruped.quadruped_env import QuadrupedEnv

from mpc_dog.types.layouts import LEG_ORDER

NU = 12


class MujocoGo2:
    """Torque-in, next state out. Contact forces are plant measurements, not commands."""

    def __init__(self, scene: str = "flat", sim_dt: float = 0.002, seed: int = 0) -> None:
        self.env = QuadrupedEnv(
            robot="go2",
            scene=scene,
            sim_dt=sim_dt,
            ref_base_lin_vel=0.0,
            ref_base_ang_vel=0.0,
        )
        self.sim_dt = float(sim_dt)
        self.reset(random=False, seed=seed)

    def reset(self, *, random: bool = False, seed: int = 0) -> None:
        self.env.reset(random=random, seed=seed)

    def step(self, tau: np.ndarray) -> None:
        action = np.asarray(tau, dtype=np.float64).reshape(NU)
        self.env.step(action)

    @property
    def model(self):
        return self.env.mjModel

    @property
    def data(self):
        return self.env.mjData

    @property
    def mass_kg(self) -> float:
        return float(self.model.body_subtreemass[1])

    def base_pos(self) -> np.ndarray:
        return np.asarray(self.data.qpos[0:3], dtype=np.float64).copy()

    def base_rpy(self) -> np.ndarray:
        return np.asarray(self.env.base_ori_euler_xyz, dtype=np.float64).copy()

    def base_lin_vel_world(self) -> np.ndarray:
        return np.asarray(self.data.qvel[0:3], dtype=np.float64).copy()

    def contact_on(self) -> np.ndarray:
        state, _ = self.env.feet_contact_state()
        return np.array([1.0 if state[leg] else 0.0 for leg in LEG_ORDER], dtype=np.float64)

    def com_world(self) -> np.ndarray:
        return np.asarray(self.data.subtree_com[1], dtype=np.float64).copy()

    def feet_pos_world(self) -> np.ndarray:
        pos = self.env.feet_pos(frame="world")
        return np.stack([np.asarray(pos[leg], dtype=np.float64) for leg in LEG_ORDER], axis=0)

    def feet_vel_world(self) -> np.ndarray:
        vel = self.env.feet_vel(frame="world", relative=False)
        return np.stack([np.asarray(vel[leg], dtype=np.float64) for leg in LEG_ORDER], axis=0)

    def contact_forces_world(self) -> np.ndarray:
        """Actual MuJoCo ground reaction on each foot, world frame, shape (4, 3)."""
        packed = self.env.feet_contact_state(frame="world", ground_reaction_forces=True)
        forces = packed[2]
        out = np.zeros((4, 3), dtype=np.float64)
        for i, leg in enumerate(LEG_ORDER):
            f = np.asarray(forces[leg], dtype=np.float64).reshape(-1)
            if f.size >= 3:
                out[i] = f[:3]
        return out

    def net_contact_force_world(self) -> np.ndarray:
        return self.contact_forces_world().sum(axis=0)

    def gravity_force_world(self) -> np.ndarray:
        g = float(self.model.opt.gravity[2])
        return np.array([0.0, 0.0, self.mass_kg * g], dtype=np.float64)

    def foot_geom_ids(self) -> np.ndarray:
        ids = self.env._feet_geom_id
        return np.array([ids.FL, ids.FR, ids.RL, ids.RR], dtype=np.int32)
