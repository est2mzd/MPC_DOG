"""Shared helpers for PyMPC workshop Jupyter notebooks."""
from __future__ import annotations

import copy
import json
import subprocess
import sys
from pathlib import Path
from typing import Any

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
PYMPC = ROOT / "external" / "Quadruped-PyMPC"
PRESET_DIR = ROOT / "configs" / "pympc_presets"
ASSETS = ROOT / "docs" / "pympc_2day" / "assets"
SCRIPTS = ROOT / "scripts"


def find_repo_root() -> Path:
    for p in [Path.cwd(), *Path.cwd().parents]:
        if (p / "scripts" / "pympc_lab.py").is_file():
            return p
    return ROOT


def ensure_pympc_on_path() -> None:
    pympc = find_repo_root() / "external" / "Quadruped-PyMPC"
    if not pympc.is_dir():
        raise FileNotFoundError(
            f"Quadruped-PyMPC not found at {pympc}. Run ./scripts/setup_references.sh"
        )
    if str(pympc) not in sys.path:
        sys.path.insert(0, str(pympc))


def apply_preset(preset_name: str, *, dry_run: bool = False) -> Path:
    """Apply session YAML to external PyMPC config.py."""
    repo = find_repo_root()
    cmd = [sys.executable, str(repo / "scripts" / "apply_pympc_preset.py"), preset_name]
    if dry_run:
        cmd.append("--dry-run")
    subprocess.run(cmd, check=True, cwd=repo)
    return PYMPC / "quadruped_pympc" / "config.py"


def load_preset_yaml(preset_name: str) -> dict:
    import yaml

    path = PRESET_DIR / f"{preset_name}.yaml"
    if not path.is_file():
        raise FileNotFoundError(path)
    with path.open(encoding="utf-8") as f:
        return yaml.safe_load(f)


def patch_config(**overrides: Any) -> None:
    """Patch in-memory config after import (does not write config.py)."""
    ensure_pympc_on_path()
    from quadruped_pympc import config as cfg

    for key, value in overrides.items():
        if key.startswith("mpc."):
            cfg.mpc_params[key.split(".", 1)[1]] = value
        elif key.startswith("sim."):
            cfg.simulation_params[key.split(".", 1)[1]] = value
        elif key == "step_freq":
            gait = cfg.simulation_params["gait"]
            cfg.simulation_params["gait_params"][gait]["step_freq"] = value
        elif key == "duty_factor":
            gait = cfg.simulation_params["gait"]
            cfg.simulation_params["gait_params"][gait]["duty_factor"] = value
        else:
            raise KeyError(f"Unknown override: {key}")


def run_flat_sim(
    seconds: float = 4.0,
    *,
    mu: float | None = None,
    step_freq: float | None = None,
    duty_factor: float | None = None,
    scene: str = "flat",
    use_foothold_optimization: bool | None = None,
    ref_z_scale: float | None = None,
    grf_max: float | None = None,
    vel_cmd: str = "forward",
) -> dict:
    """Run headless flat/rough sim and return metrics + time series."""
    ensure_pympc_on_path()
    from gym_quadruped.quadruped_env import QuadrupedEnv
    from gym_quadruped.utils.quadruped_utils import LegsAttr
    from quadruped_pympc import config as cfg
    from quadruped_pympc.quadruped_pympc_wrapper import QuadrupedPyMPC_Wrapper

    if mu is not None:
        cfg.mpc_params["mu"] = mu
    if step_freq is not None:
        gait = cfg.simulation_params["gait"]
        cfg.simulation_params["gait_params"][gait]["step_freq"] = step_freq
    if duty_factor is not None:
        gait = cfg.simulation_params["gait"]
        cfg.simulation_params["gait_params"][gait]["duty_factor"] = duty_factor
    if use_foothold_optimization is not None:
        cfg.mpc_params["use_foothold_optimization"] = use_foothold_optimization
    if grf_max is not None:
        cfg.mpc_params["grf_max"] = grf_max
    cfg.simulation_params["scene"] = scene
    if ref_z_scale is not None:
        cfg.simulation_params["ref_z"] = cfg.hip_height * ref_z_scale

    sim_dt = cfg.simulation_params["dt"]
    hip = cfg.hip_height
    env = QuadrupedEnv(
        robot=cfg.robot,
        scene=scene,
        sim_dt=sim_dt,
        ref_base_lin_vel=np.array([0.5, 0.8]) * hip,
        ref_base_ang_vel=(-0.2, 0.2),
        ground_friction_coeff=(0.5, 1.0),
        base_vel_command_type=vel_cmd,
        state_obs_names=(),
    )
    env.reset(random=False)
    wrapper = QuadrupedPyMPC_Wrapper(
        initial_feet_pos=env.feet_pos,
        legs_order=("FL", "FR", "RL", "RR"),
        feet_geom_id=env._feet_geom_id,
    )
    tau = LegsAttr(
        FL=np.zeros((3, 1)), FR=np.zeros((3, 1)),
        RL=np.zeros((3, 1)), RR=np.zeros((3, 1)),
    )
    n = int(seconds / sim_dt)
    vx, vy, vz, roll = [], [], [], []
    terminated = False
    for _ in range(n):
        ref_lin, ref_ang = env.target_base_vel()
        tau = wrapper.compute_actions(
            copy.deepcopy(env.com),
            copy.deepcopy(env.base_pos),
            env.base_lin_vel(frame="world"),
            env.base_ori_euler_xyz,
            env.base_ang_vel(frame="base"),
            env.feet_pos(frame="world"),
            env.hip_positions(frame="world"),
            env.legs_qvel_idx,
            None,
            ("FL", "FR", "RL", "RR"),
            sim_dt,
            ref_lin,
            ref_ang,
            env.step_num,
            env.mjData.qpos,
            env.mjData.qvel,
            env.feet_jacobians(frame="world"),
            env.feet_jacobians_dot(frame="world"),
            env.feet_vel(frame="world"),
            env.legs_qfrc_passive,
            env.legs_qfrc_bias,
            env.legs_mass_matrix,
            env.legs_qpos_idx,
            env.legs_qvel_idx,
            tau,
            env.get_base_inertia().flatten(),
            env.mjData.contact,
        )
        action = np.zeros(env.mjModel.nu)
        for leg in ["FL", "FR", "RL", "RR"]:
            action[getattr(env.legs_tau_idx, leg)] = tau[leg].flatten()
        _, _, term, trunc, _ = env.step(action=action)
        vx.append(float(env.base_lin_vel(frame="world")[0]))
        vy.append(float(env.base_lin_vel(frame="world")[1]))
        vz.append(float(env.base_pos[2]))
        roll.append(float(env.base_ori_euler_xyz[0]))
        if term or trunc:
            terminated = True
            break
    env.close()
    vx_arr = np.array(vx)
    return {
        "mean_vx": float(np.mean(vx_arr)),
        "std_vx": float(np.std(vx_arr)),
        "min_z": float(np.min(vz)),
        "max_roll_deg": float(np.degrees(np.max(np.abs(roll)))),
        "steps": len(vx),
        "terminated": terminated,
        "t": np.arange(len(vx)) * sim_dt,
        "vx": vx_arr,
        "vz": np.array(vz),
    }


def run_speed_terrain_sim(
    *,
    scene: str = "bumpy_flat",
    target_speed_kph: float = 5.0,
    min_distance_m: float = 20.0,
    max_seconds: float = 30.0,
    mu: float | None = None,
    step_freq: float | None = None,
    duty_factor: float | None = None,
    use_foothold_optimization: bool | None = None,
    ref_z_scale: float | None = None,
    grf_max: float | None = None,
    preset: str | None = "session04_speed_bumpy_base",
    speed_ramp_s: float = 4.0,
) -> dict:
    """Headless sim until min_distance_m or failure. Matches simulation.py control loop."""
    if str(SCRIPTS) not in sys.path:
        sys.path.insert(0, str(SCRIPTS))
    from workshop_terrain import install_custom_terrains

    install_custom_terrains()
    if preset:
        apply_preset(preset)

    ensure_pympc_on_path()
    import copy as _copy

    from gym_quadruped.quadruped_env import QuadrupedEnv
    from gym_quadruped.utils.quadruped_utils import LegsAttr
    from quadruped_pympc import config as cfg
    from quadruped_pympc.quadruped_pympc_wrapper import QuadrupedPyMPC_Wrapper

    target_mps = target_speed_kph * (1000.0 / 3600.0)
    vel_mult = target_mps / cfg.hip_height

    if mu is not None:
        cfg.mpc_params["mu"] = mu
    if step_freq is not None:
        gait = cfg.simulation_params["gait"]
        cfg.simulation_params["gait_params"][gait]["step_freq"] = step_freq
    if duty_factor is not None:
        gait = cfg.simulation_params["gait"]
        cfg.simulation_params["gait_params"][gait]["duty_factor"] = duty_factor
    if use_foothold_optimization is not None:
        cfg.mpc_params["use_foothold_optimization"] = use_foothold_optimization
    if grf_max is not None:
        cfg.mpc_params["grf_max"] = grf_max
    cfg.simulation_params["ref_z"] = cfg.hip_height * (ref_z_scale if ref_z_scale else 1.06)

    sim_dt = cfg.simulation_params["dt"]
    env = QuadrupedEnv(
        robot=cfg.robot,
        scene=scene,
        sim_dt=sim_dt,
        ref_base_lin_vel=np.array([vel_mult, vel_mult]) * cfg.hip_height,
        ref_base_ang_vel=(-0.15, 0.15),
        ground_friction_coeff=(0.5, 1.0),
        base_vel_command_type="forward",
        state_obs_names=(),
    )
    env.mjModel.opt.gravity[2] = -cfg.gravity_constant
    env.reset(random=False)
    x0 = float(env.base_pos[0])

    wrapper = QuadrupedPyMPC_Wrapper(
        initial_feet_pos=env.feet_pos,
        legs_order=("FL", "FR", "RL", "RR"),
        feet_geom_id=env._feet_geom_id,
    )
    tau = LegsAttr(
        FL=np.zeros((3, 1)), FR=np.zeros((3, 1)),
        RL=np.zeros((3, 1)), RR=np.zeros((3, 1)),
    )
    tau_soft = 0.9
    tau_limits = LegsAttr(
        FL=env.mjModel.actuator_ctrlrange[env.legs_tau_idx.FL] * tau_soft,
        FR=env.mjModel.actuator_ctrlrange[env.legs_tau_idx.FR] * tau_soft,
        RL=env.mjModel.actuator_ctrlrange[env.legs_tau_idx.RL] * tau_soft,
        RR=env.mjModel.actuator_ctrlrange[env.legs_tau_idx.RR] * tau_soft,
    )
    legs_order = ["FL", "FR", "RL", "RR"]
    heightmaps = None

    n_max = int(max_seconds / sim_dt)
    vx, vz, roll, xpos = [], [], [], []
    terminated = False
    for step_i in range(n_max):
        t = step_i * sim_dt
        if speed_ramp_s > 0 and t < speed_ramp_s:
            v_cmd = target_mps * (t / speed_ramp_s)
        else:
            v_cmd = target_mps
        env._ref_base_lin_vel_H = np.array([v_cmd, 0.0, 0.0])

        feet_pos = env.feet_pos(frame="world")
        feet_vel = env.feet_vel(frame="world")
        hip_pos = env.hip_positions(frame="world")
        base_lin_vel = env.base_lin_vel(frame="world")
        base_ang_vel = env.base_ang_vel(frame="base")
        base_ori_euler_xyz = env.base_ori_euler_xyz
        base_pos = _copy.deepcopy(env.base_pos)
        com_pos = _copy.deepcopy(env.com)
        ref_base_lin_vel, ref_base_ang_vel = env.target_base_vel()

        if cfg.simulation_params["use_inertia_recomputation"]:
            inertia = env.get_base_inertia().flatten()
        else:
            inertia = cfg.inertia.flatten()

        qpos, qvel = env.mjData.qpos, env.mjData.qvel
        legs_qvel_idx = env.legs_qvel_idx
        legs_qpos_idx = env.legs_qpos_idx
        joints_pos = LegsAttr(
            FL=legs_qvel_idx.FL, FR=legs_qvel_idx.FR,
            RL=legs_qvel_idx.RL, RR=legs_qvel_idx.RR,
        )

        tau = wrapper.compute_actions(
            com_pos,
            base_pos,
            base_lin_vel,
            base_ori_euler_xyz,
            base_ang_vel,
            feet_pos,
            hip_pos,
            joints_pos,
            heightmaps,
            legs_order,
            sim_dt,
            ref_base_lin_vel,
            ref_base_ang_vel,
            env.step_num,
            qpos,
            qvel,
            env.feet_jacobians(frame="world", return_rot_jac=False),
            env.feet_jacobians_dot(frame="world", return_rot_jac=False),
            feet_vel,
            env.legs_qfrc_passive,
            env.legs_qfrc_bias,
            env.legs_mass_matrix,
            legs_qpos_idx,
            legs_qvel_idx,
            tau,
            inertia,
            env.mjData.contact,
        )
        for leg in legs_order:
            tau_min, tau_max = tau_limits[leg][:, 0], tau_limits[leg][:, 1]
            tau[leg] = np.clip(tau[leg], tau_min, tau_max)

        action = np.zeros(env.mjModel.nu)
        for leg in legs_order:
            action[getattr(env.legs_tau_idx, leg)] = tau[leg].flatten()
        _, _, term, trunc, _ = env.step(action=action)

        vx.append(float(base_lin_vel[0]))
        vz.append(float(env.base_pos[2]))
        roll.append(float(base_ori_euler_xyz[0]))
        xpos.append(float(env.base_pos[0]))
        if term or trunc:
            terminated = True
            break
        if float(env.base_pos[0]) - x0 >= min_distance_m:
            break

    env.close()
    vx_arr = np.array(vx)
    distance_m = float(xpos[-1] - x0) if xpos else 0.0
    duration_s = len(vx) * sim_dt
    mean_kph = float(np.mean(vx_arr)) * 3.6 if len(vx_arr) else 0.0
    success = (not terminated) and distance_m >= min_distance_m and mean_kph >= 4.0
    return {
        "scene": scene,
        "target_speed_kph": target_speed_kph,
        "target_mps": target_mps,
        "min_distance_m": min_distance_m,
        "distance_m": distance_m,
        "duration_s": duration_s,
        "mean_vx": float(np.mean(vx_arr)) if len(vx_arr) else 0.0,
        "mean_kph": mean_kph,
        "std_vx": float(np.std(vx_arr)) if len(vx_arr) else 0.0,
        "min_z": float(np.min(vz)) if vz else 0.0,
        "max_roll_deg": float(np.degrees(np.max(np.abs(roll)))) if roll else 0.0,
        "steps": len(vx),
        "terminated": terminated,
        "success": success,
        "t": np.arange(len(vx)) * sim_dt,
        "vx": vx_arr,
        "vz": np.array(vz),
        "x": np.array(xpos) - x0 if xpos else np.array([]),
    }


def run_speed_terrain_sim_resilient(
    *,
    scene: str = "bumpy_flat",
    target_speed_kph: float = 5.0,
    min_distance_m: float = 20.0,
    max_seconds: float = 90.0,
    max_falls: int = 8,
    mu: float | None = None,
    step_freq: float | None = None,
    duty_factor: float | None = None,
    use_foothold_optimization: bool | None = None,
    ref_z_scale: float | None = None,
    grf_max: float | None = None,
    preset: str | None = "session04_speed_bumpy_base",
    speed_ramp_s: float = 12.0,
) -> dict:
    """Like run_speed_terrain_sim but resets on fall until cumulative distance >= min_distance_m."""
    if str(SCRIPTS) not in sys.path:
        sys.path.insert(0, str(SCRIPTS))
    from workshop_terrain import install_custom_terrains

    install_custom_terrains()
    if preset:
        apply_preset(preset)

    ensure_pympc_on_path()
    import copy as _copy

    from gym_quadruped.quadruped_env import QuadrupedEnv
    from gym_quadruped.utils.quadruped_utils import LegsAttr
    from quadruped_pympc import config as cfg
    from quadruped_pympc.quadruped_pympc_wrapper import QuadrupedPyMPC_Wrapper

    target_mps = target_speed_kph * (1000.0 / 3600.0)
    vel_mult = target_mps / cfg.hip_height
    if mu is not None:
        cfg.mpc_params["mu"] = mu
    if step_freq is not None:
        gait = cfg.simulation_params["gait"]
        cfg.simulation_params["gait_params"][gait]["step_freq"] = step_freq
    if duty_factor is not None:
        gait = cfg.simulation_params["gait"]
        cfg.simulation_params["gait_params"][gait]["duty_factor"] = duty_factor
    if use_foothold_optimization is not None:
        cfg.mpc_params["use_foothold_optimization"] = use_foothold_optimization
    if grf_max is not None:
        cfg.mpc_params["grf_max"] = grf_max
    cfg.simulation_params["ref_z"] = cfg.hip_height * (ref_z_scale if ref_z_scale else 1.06)

    sim_dt = cfg.simulation_params["dt"]
    env = QuadrupedEnv(
        robot=cfg.robot,
        scene=scene,
        sim_dt=sim_dt,
        ref_base_lin_vel=np.array([vel_mult, vel_mult]) * cfg.hip_height,
        ref_base_ang_vel=(-0.15, 0.15),
        ground_friction_coeff=(0.5, 1.0),
        base_vel_command_type="forward",
        state_obs_names=(),
    )
    env.mjModel.opt.gravity[2] = -cfg.gravity_constant
    env.reset(random=False)

    wrapper = QuadrupedPyMPC_Wrapper(
        initial_feet_pos=env.feet_pos,
        legs_order=("FL", "FR", "RL", "RR"),
        feet_geom_id=env._feet_geom_id,
    )
    tau = LegsAttr(
        FL=np.zeros((3, 1)), FR=np.zeros((3, 1)),
        RL=np.zeros((3, 1)), RR=np.zeros((3, 1)),
    )
    tau_soft = 0.9
    tau_limits = LegsAttr(
        FL=env.mjModel.actuator_ctrlrange[env.legs_tau_idx.FL] * tau_soft,
        FR=env.mjModel.actuator_ctrlrange[env.legs_tau_idx.FR] * tau_soft,
        RL=env.mjModel.actuator_ctrlrange[env.legs_tau_idx.RL] * tau_soft,
        RR=env.mjModel.actuator_ctrlrange[env.legs_tau_idx.RR] * tau_soft,
    )
    legs_order = ["FL", "FR", "RL", "RR"]
    heightmaps = None

    n_max = int(max_seconds / sim_dt)
    vx, vz, x_seg = [], [], []
    cumulative_m = 0.0
    falls = 0
    session_start = 0.0
    t_global = 0.0

    def reset_segment():
        nonlocal session_start
        env.reset(random=False)
        wrapper.reset(initial_feet_pos=env.feet_pos(frame="world"))
        session_start = float(env.base_pos[0])

    reset_segment()

    for step_i in range(n_max):
        t_seg = step_i * sim_dt - (len(vx) * 0)  # wall clock from loop
        t_global = step_i * sim_dt
        if speed_ramp_s > 0 and t_global < speed_ramp_s:
            v_cmd = target_mps * (t_global / speed_ramp_s)
        else:
            v_cmd = target_mps
        env._ref_base_lin_vel_H = np.array([v_cmd, 0.0, 0.0])

        feet_pos = env.feet_pos(frame="world")
        feet_vel = env.feet_vel(frame="world")
        hip_pos = env.hip_positions(frame="world")
        base_lin_vel = env.base_lin_vel(frame="world")
        base_ang_vel = env.base_ang_vel(frame="base")
        base_ori_euler_xyz = env.base_ori_euler_xyz
        base_pos = _copy.deepcopy(env.base_pos)
        com_pos = _copy.deepcopy(env.com)
        ref_base_lin_vel, ref_base_ang_vel = env.target_base_vel()

        if cfg.simulation_params["use_inertia_recomputation"]:
            inertia = env.get_base_inertia().flatten()
        else:
            inertia = cfg.inertia.flatten()

        qpos, qvel = env.mjData.qpos, env.mjData.qvel
        legs_qvel_idx = env.legs_qvel_idx
        legs_qpos_idx = env.legs_qpos_idx
        joints_pos = LegsAttr(
            FL=legs_qvel_idx.FL, FR=legs_qvel_idx.FR,
            RL=legs_qvel_idx.RL, RR=legs_qvel_idx.RR,
        )

        tau = wrapper.compute_actions(
            com_pos, base_pos, base_lin_vel, base_ori_euler_xyz, base_ang_vel,
            feet_pos, hip_pos, joints_pos, heightmaps, legs_order, sim_dt,
            ref_base_lin_vel, ref_base_ang_vel, env.step_num, qpos, qvel,
            env.feet_jacobians(frame="world", return_rot_jac=False),
            env.feet_jacobians_dot(frame="world", return_rot_jac=False),
            feet_vel, env.legs_qfrc_passive, env.legs_qfrc_bias, env.legs_mass_matrix,
            legs_qpos_idx, legs_qvel_idx, tau, inertia, env.mjData.contact,
        )
        for leg in legs_order:
            tau_min, tau_max = tau_limits[leg][:, 0], tau_limits[leg][:, 1]
            tau[leg] = np.clip(tau[leg], tau_min, tau_max)

        action = np.zeros(env.mjModel.nu)
        for leg in legs_order:
            action[getattr(env.legs_tau_idx, leg)] = tau[leg].flatten()
        _, _, term, trunc, _ = env.step(action=action)

        seg_x = float(env.base_pos[0]) - session_start
        vx.append(float(base_lin_vel[0]))
        vz.append(float(env.base_pos[2]))
        x_seg.append(cumulative_m + max(seg_x, 0.0))

        if term or trunc:
            cumulative_m += max(seg_x, 0.0)
            falls += 1
            if falls > max_falls or cumulative_m >= min_distance_m:
                break
            reset_segment()
            continue

        if seg_x + cumulative_m >= min_distance_m:
            cumulative_m = seg_x + cumulative_m
            break
    else:
        cumulative_m += max(float(env.base_pos[0]) - session_start, 0.0)

    env.close()
    vx_arr = np.array(vx)
    duration_s = len(vx) * sim_dt
    mean_kph = float(np.mean(vx_arr)) * 3.6 if len(vx_arr) else 0.0
    distance_m = float(cumulative_m)
    success = distance_m >= min_distance_m and falls <= max_falls
    return {
        "scene": scene,
        "target_speed_kph": target_speed_kph,
        "target_mps": target_mps,
        "min_distance_m": min_distance_m,
        "distance_m": distance_m,
        "duration_s": duration_s,
        "mean_vx": float(np.mean(vx_arr)) if len(vx_arr) else 0.0,
        "mean_kph": mean_kph,
        "falls": falls,
        "steps": len(vx),
        "terminated": falls > 0,
        "success": success,
        "t": np.arange(len(vx)) * sim_dt,
        "vx": vx_arr,
        "vz": np.array(vz),
        "x": np.array(x_seg),
    }


def compare_runs(runs: list[tuple[str, dict]]) -> None:
    """Plot vx time series for labeled runs."""
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(2, 1, figsize=(9, 5), sharex=True)
    for label, m in runs:
        axes[0].plot(m["t"], m["vx"], label=label, lw=1.5)
        axes[1].plot(m["t"], m["vz"], label=label, lw=1.5, alpha=0.8)
    axes[0].set_ylabel("vx [m/s]")
    axes[0].legend()
    axes[0].grid(True, alpha=0.3)
    axes[0].set_title("Forward velocity")
    axes[1].set_ylabel("base z [m]")
    axes[1].set_xlabel("time [s]")
    axes[1].legend()
    axes[1].grid(True, alpha=0.3)
    axes[1].set_title("Base height")
    fig.tight_layout()
    return fig


def load_param_study() -> list[dict]:
    path = ASSETS / "param_study_results.json"
    if not path.is_file():
        raise FileNotFoundError(f"Run: python scripts/run_parameter_study.py ({path})")
    return json.loads(path.read_text(encoding="utf-8"))


def load_speed_trial_log() -> list[dict]:
    path = ASSETS / "speed_terrain_trial_log.json"
    if not path.is_file():
        raise FileNotFoundError(f"Missing {path}")
    return json.loads(path.read_text(encoding="utf-8"))


def load_speed_winners() -> dict:
    path = ASSETS / "speed_terrain_results.json"
    if not path.is_file():
        raise FileNotFoundError(f"Missing {path}")
    return json.loads(path.read_text(encoding="utf-8"))


def plot_friction_cone(mu: float = 0.5, f_max: float = 150.0, ax=None):
    """Visualize friction cone in Fx-Fz plane."""
    import matplotlib.pyplot as plt

    if ax is None:
        _, ax = plt.subplots(figsize=(5, 5))
    fz = np.linspace(0, f_max, 100)
    fx_pos = mu * fz
    ax.fill_between(fz, -mu * fz, mu * fz, alpha=0.25, color="#2563eb", label=f"|Fx| ≤ μFz (μ={mu})")
    ax.plot(fz, fx_pos, "b--", lw=1.5)
    ax.plot(fz, -mu * fz, "b--", lw=1.5)
    ax.axhline(0, color="k", lw=0.5)
    ax.axvline(0, color="k", lw=0.5)
    ax.set_xlabel("Fz [N]")
    ax.set_ylabel("Fx [N]")
    ax.set_title("Friction cone (side view)")
    ax.set_aspect("equal")
    ax.grid(True, alpha=0.3)
    ax.legend()
    return ax


# MPC designer tuning guide — used in theory + demo notebooks
TUNING_GUIDE: list[dict] = [
    {
        "param": "mpc_params.mu",
        "what": "摩擦円錐の傾き（地面との摩擦係数モデル）",
        "raise": "水平GRFを取りやすい → 加速・旋回が積極的に",
        "lower": "水平力を抑える → 保守的・滑りにくい",
        "failure_symptom": "転倒・横滑り・足が刺さる",
        "failure_fix": "μを下げる / duty_factor↑ / step_freq↓",
        "success_sign": "狙ったvxに追従、姿勢安定",
    },
    {
        "param": "gait_params.trot.step_freq",
        "what": "歩調（1秒あたりの歩数）",
        "raise": "足回しが速い → 速走向き、MPC予測が追いにくい",
        "lower": "歩幅・支持が長い → 安定、低速向き",
        "failure_symptom": "足が地面に刺さる、MPC solve が間に合わない",
        "failure_fix": "step_freq↓、horizon↑、solver_mode='speed'",
        "success_sign": "滑らかなtrot、GRF矢印が周期的",
    },
    {
        "param": "gait_params.trot.duty_factor",
        "what": "1周期のうち支持脚の割合",
        "raise": "支持が長い → 安定、敏捷性↓",
        "lower": "遊脚が長い → 敏捷、着地精度要求↑",
        "failure_symptom": "着地で転倒、ダブルサポート不足",
        "failure_fix": "duty_factor↑（0.7–0.75）",
        "success_sign": "不整地でも支持中に姿勢回復",
    },
    {
        "param": "mpc_params.grf_max",
        "what": "1足あたり垂直GRF上限",
        "raise": "大きな蹴り → 加速↑、跳ね・オーバーシュート",
        "lower": "ソフトな着地、加速↓",
        "failure_symptom": "跳ねる、関節飽和",
        "failure_fix": "grf_max↓（≈ mg/4 × safety）",
        "success_sign": "滑らかな垂直力、過度な跳ねなし",
    },
    {
        "param": "simulation_params.ref_z",
        "what": "目標胴体高さ",
        "raise": "脚を伸ばす → 地面クリアランス↑",
        "lower": "低重心 → 安定だが地面接触リスク",
        "failure_symptom": "即転倒、足が浮く/刺さる",
        "failure_fix": "ref_z = hip_height × 1.05",
        "success_sign": "一定の胴体高さを維持",
    },
    {
        "param": "mpc_params.use_foothold_optimization",
        "what": "MPC内で着地点も最適化",
        "raise": "ON: 不整地向き",
        "lower": "OFF: 平坦・デバッグ向き",
        "failure_symptom": "変な位置に足を置く（地形モデル不一致）",
        "failure_fix": "OFFで比較 → 地形推定確認",
        "success_sign": "段差で足が安全な位置に着地",
    },
    {
        "param": "simulation_params.swing_position_gain_fb",
        "what": "スイング脚の位置PDゲイン",
        "raise": "足振りが硬い → オーバーシュート",
        "lower": "柔らかい → 着地精度↓",
        "failure_symptom": "スイング脚が振動",
        "failure_fix": "gain↓ または step_height↓",
        "success_sign": "滑らかなスイング軌道",
    },
    {
        "param": "mpc_params.horizon × dt",
        "what": "予測ホライゾン長",
        "raise": "先読み↑ → 計算重い",
        "lower": "反応速い → 先読み不足で転倒",
        "failure_symptom": "急停止・方向転換で転倒",
        "failure_fix": "horizon↑ または ref 速度を緩やかに",
        "success_sign": "指令変更に滑らかに追従",
    },
]
