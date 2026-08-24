"""ROS-free MuJoCo execution boundary for qiayuanl/legged_control's A1.

This module intentionally does *not* implement OCS2 SQP.  It preserves the
upstream gait/state/input/WBC/hybrid-command interfaces, while replacing the
OCS2 policy with a deterministic instantaneous force planner and a linear
whole-body inverse-dynamics solve.
"""

from __future__ import annotations

import os
from dataclasses import dataclass
from pathlib import Path
from typing import Final, Sequence

os.environ.setdefault("MUJOCO_GL", "egl")

import mujoco
import numpy as np
from scipy.optimize import LinearConstraint, minimize


UPSTREAM_COMMIT: Final = "a7f381c0367e98e31c01336e678eef47e304d40d"
JOINT_NAMES: Final = (
    "LF_HAA", "LF_HFE", "LF_KFE",
    "LH_HAA", "LH_HFE", "LH_KFE",
    "RF_HAA", "RF_HFE", "RF_KFE",
    "RH_HAA", "RH_HFE", "RH_KFE",
)
# task.info R: contact-force input blocks are LF, RF, LH, RH.
CONTACT_NAMES: Final = ("LF", "RF", "LH", "RH")
FOOT_SITES: Final = tuple(f"{name}_FOOT" for name in CONTACT_NAMES)
DEFAULT_JOINT_POS: Final = np.array(
    [-0.20, 0.72, -1.44, -0.20, 0.72, -1.44,
      0.20, 0.72, -1.44, 0.20, 0.72, -1.44],
    dtype=float,
)
TORQUE_LIMIT: Final = 33.5
FRICTION_COEFF: Final = 0.3


# Exact templates transcribed from config/a1/gait.info.  The upstream OCS2
# ModeSchedule repeats switchingTimes over a gait period.
GAIT_TEMPLATES: Final = {
    "stance": (("STANCE",), (0.0, 0.5)),
    "trot": (("LF_RH", "RF_LH"), (0.0, 0.3, 0.6)),
    "standing_trot": (
        ("LF_RH", "STANCE", "RF_LH", "STANCE"),
        (0.0, 0.25, 0.3, 0.55, 0.6),
    ),
    "flying_trot": (
        ("LF_RH", "FLY", "RF_LH", "FLY"),
        (0.0, 0.15, 0.2, 0.35, 0.4),
    ),
    "pace": (
        ("LF_LH", "FLY", "RF_RH", "FLY"),
        (0.0, 0.28, 0.30, 0.58, 0.60),
    ),
    "standing_pace": (
        ("LF_LH", "STANCE", "RF_RH", "STANCE"),
        (0.0, 0.30, 0.35, 0.65, 0.70),
    ),
    "dynamic_walk": (
        ("LF_RF_RH", "RF_RH", "RF_LH_RH", "LF_RF_LH", "LF_LH", "LF_LH_RH"),
        (0.0, 0.2, 0.3, 0.5, 0.7, 0.8, 1.0),
    ),
    "static_walk": (
        ("LF_RF_RH", "RF_LH_RH", "LF_RF_LH", "LF_LH_RH"),
        (0.0, 0.3, 0.6, 0.9, 1.2),
    ),
    "amble": (
        ("RF_LH", "LF_LH", "LF_RH", "RF_RH"),
        (0.0, 0.15, 0.40, 0.55, 0.80),
    ),
    "pawup": (("RF_LH_RH",), (0.0, 2.0)),
}


def mode_contacts(mode: str) -> np.ndarray:
    """Map upstream mode labels to [LF, RF, LH, RH] stance flags."""
    if mode == "STANCE":
        return np.ones(4, dtype=bool)
    if mode == "FLY":
        return np.zeros(4, dtype=bool)
    members = set(mode.split("_"))
    return np.array([name in members for name in CONTACT_NAMES], dtype=bool)


@dataclass(frozen=True)
class ModeSchedule:
    """Periodic equivalent of the OCS2 ModeSchedule loaded from gait.info."""

    modes: tuple[str, ...]
    switching_times: tuple[float, ...]

    @classmethod
    def from_gait(cls, gait: str) -> "ModeSchedule":
        try:
            modes, times = GAIT_TEMPLATES[gait]
        except KeyError as exc:
            raise ValueError(f"unknown A1 gait {gait!r}") from exc
        return cls(tuple(modes), tuple(times))

    @property
    def period(self) -> float:
        return self.switching_times[-1] - self.switching_times[0]

    def mode_at(self, time: float) -> str:
        phase = (time - self.switching_times[0]) % self.period
        index = int(np.searchsorted(self.switching_times[1:], phase, side="right"))
        return self.modes[min(index, len(self.modes) - 1)]

    def mode_phase(self, time: float) -> tuple[str, int, float]:
        """Return active mode, index, and normalized phase within that mode."""
        cycle_time = (time - self.switching_times[0]) % self.period
        index = int(np.searchsorted(self.switching_times[1:], cycle_time, side="right"))
        index = min(index, len(self.modes) - 1)
        start, end = self.switching_times[index:index + 2]
        phase = (cycle_time - start) / (end - start)
        return self.modes[index], index, float(np.clip(phase, 0.0, 1.0))

    def contacts_at(self, time: float) -> np.ndarray:
        return mode_contacts(self.mode_at(time))


@dataclass(frozen=True)
class CentroidalContract:
    """Upstream 24D state/input layout from config/a1/task.info.

    State: normalized centroidal momentum[6], base xyz + ZYX[6], joints[12].
    Input: world contact forces LF/RF/LH/RH[12], joint velocities[12].
    """

    state: np.ndarray
    control: np.ndarray

    def __post_init__(self) -> None:
        if np.shape(self.state) != (24,) or np.shape(self.control) != (24,):
            raise ValueError("legged_control centroidal state and input must both be 24D")

    @property
    def contact_forces(self) -> np.ndarray:
        return self.control[:12].reshape(4, 3)

    @property
    def joint_velocities(self) -> np.ndarray:
        return self.control[12:]


@dataclass
class WbcResult:
    qacc: np.ndarray
    contact_forces: np.ndarray
    torque: np.ndarray
    dynamics_residual: float


@dataclass(frozen=True)
class MotionCommand:
    """Velocity command corresponding to upstream target trajectories."""

    vx: float = 0.0
    vy: float = 0.0
    yaw_rate: float = 0.0
    base_height: float = 0.32


@dataclass(frozen=True)
class ScenarioConfig:
    """Physical scenario inputs applied directly to the MuJoCo plant."""

    friction: float = 0.6
    payload_mass: float = 0.0
    push_force: tuple[float, float, float] = (0.0, 0.0, 0.0)
    push_start: float = 0.0
    push_duration: float = 0.0


@dataclass
class TimeSeriesMetrics:
    base_xyz: np.ndarray
    base_rpy: np.ndarray
    base_velocity: np.ndarray
    commanded_velocity: np.ndarray
    planned_contacts: np.ndarray
    measured_contacts: np.ndarray
    torque_saturated: np.ndarray


@dataclass
class RunResult:
    times: np.ndarray
    qpos: np.ndarray
    torques: np.ndarray
    modes: tuple[str, ...]
    frames: list[np.ndarray]
    max_dynamics_residual: float
    metrics: TimeSeriesMetrics
    fallen: bool
    fall_time: float | None

    @property
    def distance_xy(self) -> float:
        if len(self.metrics.base_xyz) < 2:
            return 0.0
        return float(np.linalg.norm(self.metrics.base_xyz[-1, :2] - self.metrics.base_xyz[0, :2]))

    @property
    def tracking_rmse(self) -> np.ndarray:
        error = self.metrics.base_velocity - self.metrics.commanded_velocity
        return np.sqrt(np.mean(error * error, axis=0)) if len(error) else np.zeros(3)

    @property
    def torque_saturation_fraction(self) -> float:
        return float(np.mean(self.metrics.torque_saturated)) if len(self.metrics.torque_saturated) else 0.0


def model_path() -> Path:
    return Path(__file__).with_name("models") / "a1.xml"


def load_a1_model(timestep: float = 0.002) -> mujoco.MjModel:
    model = mujoco.MjModel.from_xml_path(str(model_path()))
    model.opt.timestep = timestep
    return model


def project_friction(force: np.ndarray, mu: float = FRICTION_COEFF) -> np.ndarray:
    """Project onto WbcBase.cpp's unilateral square friction pyramid.

    Upstream inequalities are fz>=0 and |fx|,|fy|<=mu*fz (lines 154-169).
    """
    result = np.asarray(force, dtype=float).copy()
    result[2] = max(0.0, result[2])
    bound = mu * result[2]
    result[:2] = np.clip(result[:2], -bound, bound)
    return result


def plan_contact_forces(
    contacts: Sequence[bool],
    mass: float,
    desired_accel: Sequence[float] = (0.0, 0.0, 0.0),
    mu: float = FRICTION_COEFF,
) -> np.ndarray:
    """Instantaneous weight/acceleration force plan, not OCS2 SQP.

    This is the ROS-free replacement boundary for
    LeggedRobotInitializer::weightCompensatingInput and the first 12 entries of
    the upstream centroidal input. Active feet share m*(a-g); inactive feet are
    exactly zero, then every active force is projected to the WBC pyramid.
    """
    flags = np.asarray(contacts, dtype=bool)
    forces = np.zeros((4, 3))
    count = int(flags.sum())
    if count == 0:
        return forces
    total = mass * (np.asarray(desired_accel, dtype=float) + np.array([0.0, 0.0, 9.81]))
    forces[flags] = project_friction(total / count, mu)
    return forces


def hybrid_command(
    tau_ff: np.ndarray,
    q_des: np.ndarray,
    q: np.ndarray,
    dq_des: np.ndarray,
    dq: np.ndarray,
    kp: float = 0.0,
    kd: float = 3.0,
    torque_limit: float = TORQUE_LIMIT,
) -> np.ndarray:
    """LeggedController.cpp line 135: tau_ff + Kp(q*-q)+Kd(dq*-dq)."""
    command = tau_ff + kp * (q_des - q) + kd * (dq_des - dq)
    return np.clip(command, -torque_limit, torque_limit)


class A1HeadlessAdapter:
    """Headless simulator plus MuJoCo-matrix WBC-style inverse dynamics."""

    def __init__(
        self,
        gait: str = "stance",
        timestep: float = 0.002,
        control_dt: float = 0.01,
        seed: int = 0,
        command: MotionCommand | None = None,
        scenario: ScenarioConfig | None = None,
    ) -> None:
        self.model = load_a1_model(timestep)
        self.data = mujoco.MjData(self.model)
        self.schedule = ModeSchedule.from_gait(gait)
        self.control_dt = control_dt
        self.rng = np.random.default_rng(seed)
        self.command = command or MotionCommand()
        self.scenario = scenario or ScenarioConfig()
        if self.scenario.friction <= 0.0 or self.scenario.payload_mass < 0.0:
            raise ValueError("friction must be positive and payload_mass nonnegative")
        self.trunk_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "trunk")
        if self.scenario.payload_mass:
            # A compact payload at the trunk COM. mj_setConst updates subtree
            # mass used by MuJoCo's CRB equations.
            self.model.body_mass[self.trunk_id] += self.scenario.payload_mass
            mujoco.mj_setConst(self.model, self.data)
        for geom_name in ("floor", *(f"{name}_foot_geom" for name in CONTACT_NAMES)):
            geom_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, geom_name)
            self.model.geom_friction[geom_id, 0] = self.scenario.friction
        self.mass = float(mujoco.mj_getTotalmass(self.model))
        self.joint_qpos = np.array(
            [self.model.jnt_qposadr[mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)]
             for name in JOINT_NAMES]
        )
        self.joint_dof = np.array(
            [self.model.jnt_dofadr[mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)]
             for name in JOINT_NAMES]
        )
        self.site_ids = np.array(
            [mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, name) for name in FOOT_SITES]
        )
        self.foot_geom_ids = np.array(
            [mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, f"{name}_foot_geom")
             for name in CONTACT_NAMES]
        )
        self._scratch = mujoco.MjData(self.model)
        self._last_contacts = np.ones(4, dtype=bool)
        self._stance_anchor = np.zeros((4, 3))
        self._swing_start = np.zeros((4, 3))
        self._swing_target = np.zeros((4, 3))
        self._nominal_foot_base = np.zeros((4, 3))
        self._force_guess = np.zeros(12)
        self.reset()

    def reset(self) -> None:
        mujoco.mj_resetData(self.model, self.data)
        # At z=0.32 the nominal feet are 11.4 mm inside the floor. Starting at
        # 0.3314 places their 20 mm spheres tangent, avoiding a large artificial
        # contact impulse while retaining task.info's approximately 0.3 m pose.
        self.data.qpos[:3] = (0.0, 0.0, 0.3314)
        self.data.qpos[3:7] = (1.0, 0.0, 0.0, 0.0)
        self.data.qpos[self.joint_qpos] = DEFAULT_JOINT_POS
        mujoco.mj_forward(self.model, self.data)
        feet = self.data.site_xpos[self.site_ids].copy()
        self._nominal_foot_base[:] = feet - self.data.qpos[:3]
        self._stance_anchor[:] = feet
        self._swing_start[:] = feet
        self._swing_target[:] = feet
        self._last_contacts[:] = True
        self._force_guess[:] = 0.0

    def centroidal_contract(self, forces: np.ndarray) -> CentroidalContract:
        state = np.zeros(24)
        # OCS2 stores normalized momentum. MuJoCo's free-joint translational and
        # angular velocities provide a deterministic local observation proxy.
        state[:3] = self.data.qvel[:3]
        state[3:6] = self.data.qvel[3:6]
        state[6:9] = self.data.qpos[:3]
        quat = self.data.qpos[3:7]
        mat = np.empty(9)
        mujoco.mju_quat2Mat(mat, quat)
        # MuJoCo returns XYZ intrinsic angles only through matrix conversion;
        # store upstream ZYX as yaw,pitch,roll.
        rotation = mat.reshape(3, 3)
        pitch = np.arcsin(np.clip(-rotation[2, 0], -1.0, 1.0))
        state[9:12] = (
            np.arctan2(rotation[1, 0], rotation[0, 0]),
            pitch,
            np.arctan2(rotation[2, 1], rotation[2, 2]),
        )
        state[12:] = self.data.qpos[self.joint_qpos]
        control = np.r_[forces.reshape(-1), np.zeros(12)]
        return CentroidalContract(state, control)

    def foot_jacobian(self) -> np.ndarray:
        jacobian = np.zeros((12, self.model.nv))
        for index, site_id in enumerate(self.site_ids):
            jacp = np.zeros((3, self.model.nv))
            jacr = np.zeros((3, self.model.nv))
            mujoco.mj_jacSite(self.model, self.data, jacp, jacr, int(site_id))
            jacobian[3 * index:3 * index + 3] = jacp
        return jacobian

    def _base_rpy(self) -> np.ndarray:
        matrix = np.empty(9)
        mujoco.mju_quat2Mat(matrix, self.data.qpos[3:7])
        rotation = matrix.reshape(3, 3)
        return np.array([
            np.arctan2(rotation[2, 1], rotation[2, 2]),
            np.arcsin(np.clip(-rotation[2, 0], -1.0, 1.0)),
            np.arctan2(rotation[1, 0], rotation[0, 0]),
        ])

    def _jacobian_bias_acceleration(self, jacobian: np.ndarray) -> np.ndarray:
        """Finite-difference Jdot*qdot for WbcBase no-contact/swing tasks."""
        eps = 1e-5
        self._scratch.qpos[:] = self.data.qpos
        self._scratch.qvel[:] = self.data.qvel
        mujoco.mj_integratePos(self.model, self._scratch.qpos, self.data.qvel, eps)
        mujoco.mj_forward(self.model, self._scratch)
        future = np.zeros_like(jacobian)
        for index, site_id in enumerate(self.site_ids):
            jacp = np.zeros((3, self.model.nv))
            jacr = np.zeros((3, self.model.nv))
            mujoco.mj_jacSite(self.model, self._scratch, jacp, jacr, int(site_id))
            future[3 * index:3 * index + 3] = jacp
        return ((future - jacobian) / eps) @ self.data.qvel

    def _update_foot_targets(
        self,
        contacts: np.ndarray,
        mode_index: int,
        phase: float,
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        positions = self.data.site_xpos[self.site_ids].copy()
        jacobian = self.foot_jacobian()
        velocities = (jacobian @ self.data.qvel).reshape(4, 3)
        mode_duration = (
            self.schedule.switching_times[mode_index + 1]
            - self.schedule.switching_times[mode_index]
        )
        desired_pos = positions.copy()
        desired_vel = np.zeros((4, 3))
        desired_acc = np.zeros((4, 3))
        command_xy = np.array([self.command.vx, self.command.vy])

        for foot in range(4):
            if contacts[foot] and not self._last_contacts[foot]:
                self._stance_anchor[foot] = positions[foot]
            elif not contacts[foot] and self._last_contacts[foot]:
                self._swing_start[foot] = positions[foot]
                nominal = self._nominal_foot_base[foot]
                target = self.data.qpos[:3] + nominal
                target[:2] += 0.5 * self.schedule.period * command_xy
                target[2] = 0.021
                self._swing_target[foot] = target

            if contacts[foot]:
                desired_pos[foot] = self._stance_anchor[foot]
                # Contact stabilization augments upstream J*qdd=-Jdot*qdot
                # with Baumgarte terms to prevent numerical anchor drift.
                desired_acc[foot] = (
                    160.0 * (self._stance_anchor[foot] - positions[foot])
                    - 26.0 * velocities[foot]
                )
            else:
                p = float(np.clip(phase, 0.0, 1.0))
                blend = 3.0 * p * p - 2.0 * p * p * p
                blend_d = (6.0 * p - 6.0 * p * p) / mode_duration
                blend_dd = (6.0 - 12.0 * p) / (mode_duration * mode_duration)
                delta = self._swing_target[foot] - self._swing_start[foot]
                desired_pos[foot] = self._swing_start[foot] + blend * delta
                desired_vel[foot] = blend_d * delta
                desired_acc[foot] = blend_dd * delta
                height = 0.08  # task.info swingHeight
                # sin² gives zero lift-off/touch-down vertical velocity, unlike
                # the previous parabola's unphysical 1.07 m/s discontinuity.
                desired_pos[foot, 2] += height * np.sin(np.pi * p) ** 2
                desired_vel[foot, 2] += (
                    height * np.pi * np.sin(2.0 * np.pi * p) / mode_duration
                )
                desired_acc[foot, 2] += (
                    2.0 * height * np.pi**2 * np.cos(2.0 * np.pi * p)
                    / (mode_duration * mode_duration)
                )
                # WbcBase::formulateSwingLegTask: kp=350, kd=37.
                desired_acc[foot] += (
                    350.0 * (desired_pos[foot] - positions[foot])
                    + 37.0 * (desired_vel[foot] - velocities[foot])
                )
        self._last_contacts[:] = contacts
        return desired_pos, desired_vel, desired_acc

    def _desired_qacc(
        self,
        contacts: np.ndarray,
        mode_index: int,
        phase: float,
        jacobian: np.ndarray,
    ) -> np.ndarray:
        _, _, foot_accel = self._update_foot_targets(contacts, mode_index, phase)
        jdot_qdot = self._jacobian_bias_acceleration(jacobian)
        qacc = np.zeros(self.model.nv)
        rpy = self._base_rpy()
        qacc[:3] = np.array([
            8.0 * (self.command.vx - self.data.qvel[0]),
            8.0 * (self.command.vy - self.data.qvel[1]),
            120.0 * (self.command.base_height - self.data.qpos[2]) - 22.0 * self.data.qvel[2],
        ])
        qacc[3:6] = np.array([
            -55.0 * rpy[0] - 12.0 * self.data.qvel[3],
            -55.0 * rpy[1] - 12.0 * self.data.qvel[4],
            8.0 * (self.command.yaw_rate - self.data.qvel[5]),
        ])

        # Solve all four Cartesian tasks for joint acceleration. Stance rows map
        # WbcBase::formulateNoContactMotionTask; swing rows map
        # formulateSwingLegTask. The right-hand side includes Jdot*qdot.
        joint_jacobian = jacobian[:, self.joint_dof]
        rhs = foot_accel.reshape(-1) - jdot_qdot - jacobian[:, :6] @ qacc[:6]
        posture = (
            35.0 * (DEFAULT_JOINT_POS - self.data.qpos[self.joint_qpos])
            - 7.0 * self.data.qvel[self.joint_dof]
        )
        posture_weight = 0.5
        augmented_a = np.vstack((joint_jacobian, posture_weight * np.eye(12)))
        augmented_b = np.r_[rhs, posture_weight * posture]
        qacc[self.joint_dof] = np.clip(
            np.linalg.lstsq(augmented_a, augmented_b, rcond=1e-6)[0],
            -150.0,
            150.0,
        )
        return qacc

    def _optimize_contact_forces(
        self,
        contacts: np.ndarray,
        qacc: np.ndarray,
        mass_matrix: np.ndarray,
        jacobian: np.ndarray,
    ) -> np.ndarray:
        active = np.flatnonzero(np.repeat(contacts, 3))
        forces = np.zeros(12)
        if not len(active):
            return forces.reshape(4, 3)
        # Floating-base rows of M*qdd+b=J.T*f are the exact MuJoCo-frame
        # centroidal wrench target. This removes hand-coded force/frame signs.
        wrench_map = jacobian.T[:6, active]
        demand = (mass_matrix @ qacc + self.data.qfrc_bias)[:6]
        weights = np.diag([1.0, 1.0, 1.0, 4.0, 4.0, 4.0])
        weighted_map = weights @ wrench_map
        weighted_demand = weights @ demand
        count = int(np.sum(contacts))
        nominal_full = plan_contact_forces(
            contacts,
            self.mass,
            desired_accel=qacc[:3],
            mu=min(FRICTION_COEFF, self.scenario.friction),
        ).reshape(-1)
        nominal = nominal_full[active]
        mu = min(FRICTION_COEFF, self.scenario.friction)

        def objective(value: np.ndarray) -> float:
            residual = weighted_map @ value - weighted_demand
            regularization = value - nominal
            return 0.5 * float(residual @ residual + 1e-4 * regularization @ regularization)

        def gradient(value: np.ndarray) -> np.ndarray:
            return weighted_map.T @ (weighted_map @ value - weighted_demand) + 1e-4 * (value - nominal)

        pyramid = np.zeros((5 * count, 3 * count))
        for contact in range(count):
            block = np.array([
                [0.0, 0.0, 1.0],
                [-1.0, 0.0, mu],
                [1.0, 0.0, mu],
                [0.0, -1.0, mu],
                [0.0, 1.0, mu],
            ])
            pyramid[5 * contact:5 * contact + 5, 3 * contact:3 * contact + 3] = block
        initial = self._force_guess[active]
        if not np.any(initial):
            initial = nominal
        result = minimize(
            objective,
            initial,
            jac=gradient,
            method="SLSQP",
            constraints=(LinearConstraint(pyramid, 0.0, np.inf),),
            options={"ftol": 1e-8, "maxiter": 40, "disp": False},
        )
        value = result.x if result.success and np.all(np.isfinite(result.x)) else nominal
        forces[active] = value
        self._force_guess[:] = forces
        return forces.reshape(4, 3)

    def solve_wbc(self, contacts: np.ndarray, mode_index: int = 0, phase: float = 0.0) -> WbcResult:
        """Acceleration-level MuJoCo WBC with upstream task structure.

        The decision boundary remains WbcBase's [qdd, contact force, torque].
        Unlike the first adapter, forces are not fixed to an inconsistent
        equal-share guess: they satisfy the friction pyramid while minimizing
        floating-base EoM error. Actuated EoM then gives tau exactly before the
        upstream 33.5 Nm limit is applied.
        """
        mass_matrix = np.zeros((self.model.nv, self.model.nv))
        mujoco.mj_fullM(self.model, self.data, mass_matrix)
        jacobian = self.foot_jacobian()
        qacc = self._desired_qacc(contacts, mode_index, phase, jacobian)
        solved_force = self._optimize_contact_forces(
            contacts, qacc, mass_matrix, jacobian
        )
        generalized = (
            mass_matrix @ qacc
            + self.data.qfrc_bias
            - jacobian.T @ solved_force.reshape(-1)
        )
        torque_unclipped = generalized[self.joint_dof]
        torque = np.clip(torque_unclipped, -TORQUE_LIMIT, TORQUE_LIMIT)
        residual = generalized.copy()
        residual[self.joint_dof] -= torque
        return WbcResult(qacc, solved_force, torque, float(np.linalg.norm(residual)))

    def run(
        self,
        duration: float = 0.2,
        render: bool = False,
        width: int = 480,
        height: int = 360,
        fps: int = 30,
    ) -> RunResult:
        if duration <= 0:
            raise ValueError("duration must be positive")
        renderer = mujoco.Renderer(self.model, height=height, width=width) if render else None
        if renderer is not None:
            camera = mujoco.MjvCamera()
            camera.lookat[:] = (0.0, 0.0, 0.22)
            camera.distance, camera.azimuth, camera.elevation = 1.15, 135, -20
        else:
            camera = None

        times, qposes, torques, modes, frames, residuals = [], [], [], [], [], []
        xyz_series, rpy_series, velocity_series, command_series = [], [], [], []
        planned_series, measured_series, saturated_series = [], [], []
        next_control = 0.0
        next_frame = 0.0
        command = np.zeros(12)
        fallen = False
        fall_time = None
        while self.data.time < duration - 0.5 * self.model.opt.timestep:
            if self.data.time + 1e-12 >= next_control:
                mode, mode_index, phase = self.schedule.mode_phase(self.data.time)
                contacts = mode_contacts(mode)
                wbc = self.solve_wbc(contacts, mode_index, phase)
                contract = self.centroidal_contract(wbc.contact_forces)
                command = hybrid_command(
                    wbc.torque,
                    DEFAULT_JOINT_POS,
                    self.data.qpos[self.joint_qpos],
                    contract.joint_velocities,
                    self.data.qvel[self.joint_dof],
                )
                times.append(float(self.data.time))
                qposes.append(self.data.qpos.copy())
                torques.append(command.copy())
                modes.append(mode)
                residuals.append(wbc.dynamics_residual)
                xyz_series.append(self.data.qpos[:3].copy())
                rpy = self._base_rpy()
                rpy_series.append(rpy)
                velocity_series.append(
                    np.array([self.data.qvel[0], self.data.qvel[1], self.data.qvel[5]])
                )
                command_series.append(
                    np.array([self.command.vx, self.command.vy, self.command.yaw_rate])
                )
                planned_series.append(contacts.copy())
                measured = np.zeros(4, dtype=bool)
                for contact_index in range(self.data.ncon):
                    contact = self.data.contact[contact_index]
                    for foot_index, geom_id in enumerate(self.foot_geom_ids):
                        if contact.geom1 == geom_id or contact.geom2 == geom_id:
                            measured[foot_index] = True
                measured_series.append(measured)
                saturated_series.append(np.abs(command) >= TORQUE_LIMIT - 1e-9)
                if (
                    not fallen
                    and (self.data.qpos[2] < 0.18 or np.max(np.abs(rpy[:2])) > 0.9)
                ):
                    fallen = True
                    fall_time = float(self.data.time)
                next_control += self.control_dt
            push_end = self.scenario.push_start + self.scenario.push_duration
            if self.scenario.push_start <= self.data.time < push_end:
                self.data.xfrc_applied[self.trunk_id, :3] = self.scenario.push_force
            else:
                self.data.xfrc_applied[self.trunk_id, :] = 0.0
            self.data.ctrl[:] = command
            mujoco.mj_step(self.model, self.data)
            if renderer is not None and self.data.time + 1e-12 >= next_frame:
                renderer.update_scene(self.data, camera=camera)
                frames.append(renderer.render().copy())
                next_frame += 1.0 / fps
        if renderer is not None:
            renderer.close()
        return RunResult(
            times=np.asarray(times),
            qpos=np.asarray(qposes),
            torques=np.asarray(torques),
            modes=tuple(modes),
            frames=frames,
            max_dynamics_residual=max(residuals, default=0.0),
            metrics=TimeSeriesMetrics(
                base_xyz=np.asarray(xyz_series),
                base_rpy=np.asarray(rpy_series),
                base_velocity=np.asarray(velocity_series),
                commanded_velocity=np.asarray(command_series),
                planned_contacts=np.asarray(planned_series),
                measured_contacts=np.asarray(measured_series),
                torque_saturated=np.asarray(saturated_series),
            ),
            fallen=fallen,
            fall_time=fall_time,
        )


def save_gif(frames: Sequence[np.ndarray], path: str | Path, fps: int = 30) -> Path:
    if not frames:
        raise ValueError("cannot create GIF without rendered frames")
    from PIL import Image

    output = Path(path)
    output.parent.mkdir(parents=True, exist_ok=True)
    images = [Image.fromarray(np.asarray(frame, dtype=np.uint8)) for frame in frames]
    images[0].save(
        output,
        save_all=True,
        append_images=images[1:],
        duration=max(1, round(1000 / fps)),
        loop=0,
    )
    return output
