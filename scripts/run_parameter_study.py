#!/usr/bin/env python3
"""Session 2 parameter study using simulation.py loop (metrics + plots)."""
from __future__ import annotations

import copy
import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
PYMPC = ROOT / "external" / "Quadruped-PyMPC"
ASSETS = ROOT / "docs" / "pympc_2day" / "assets"
ASSETS.mkdir(parents=True, exist_ok=True)
sys.path.insert(0, str(PYMPC))


def simulate(mu: float, step_freq: float, seconds: float = 6.0) -> dict:
    from gym_quadruped.quadruped_env import QuadrupedEnv
    from quadruped_pympc import config as cfg
    from quadruped_pympc.quadruped_pympc_wrapper import QuadrupedPyMPC_Wrapper

    cfg.mpc_params["mu"] = mu
    cfg.simulation_params["scene"] = "flat"
    cfg.mpc_params["use_foothold_optimization"] = False
    cfg.simulation_params["gait_params"]["trot"]["step_freq"] = step_freq

    sim_dt = cfg.simulation_params["dt"]
    hip = cfg.hip_height
    env = QuadrupedEnv(
        robot=cfg.robot,
        scene="flat",
        sim_dt=sim_dt,
        ref_base_lin_vel=np.array([0.5, 0.8]) * hip,
        ref_base_ang_vel=(-0.2, 0.2),
        ground_friction_coeff=(0.5, 1.0),
        base_vel_command_type="forward",
        state_obs_names=(),
    )
    env.reset(random=False)
    wrapper = QuadrupedPyMPC_Wrapper(
        initial_feet_pos=env.feet_pos,
        legs_order=("FL", "FR", "RL", "RR"),
        feet_geom_id=env._feet_geom_id,
    )
    tau = env.legs_qvel_idx  # placeholder LegsAttr-like; overwritten each step
    from gym_quadruped.utils.quadruped_utils import LegsAttr

    tau = LegsAttr(
        FL=np.zeros((3, 1)), FR=np.zeros((3, 1)),
        RL=np.zeros((3, 1)), RR=np.zeros((3, 1)),
    )
    n = int(seconds / sim_dt)
    vx, vz = [], []
    terminated = False
    for _ in range(n):
        feet_pos = env.feet_pos(frame="world")
        ref_lin, ref_ang = env.target_base_vel()
        inertia = env.get_base_inertia().flatten()
        tau = wrapper.compute_actions(
            copy.deepcopy(env.com),
            copy.deepcopy(env.base_pos),
            env.base_lin_vel(frame="world"),
            env.base_ori_euler_xyz,
            env.base_ang_vel(frame="base"),
            feet_pos,
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
            inertia,
            env.mjData.contact,
        )
        action = np.zeros(env.mjModel.nu)
        for leg in ["FL", "FR", "RL", "RR"]:
            action[getattr(env.legs_tau_idx, leg)] = tau[leg].flatten()
        _, _, term, trunc, _ = env.step(action=action)
        vx.append(float(env.base_lin_vel(frame="world")[0]))
        vz.append(float(env.base_pos[2]))
        if term or trunc:
            terminated = True
            break
    env.close()
    return {
        "mu": mu,
        "step_freq": step_freq,
        "mean_vx": float(np.mean(vx)),
        "min_z": float(np.min(vz)),
        "steps": len(vx),
        "terminated": terminated,
    }


def plot_results(results: list[dict]) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    mu_results = sorted([r for r in results if r["step_freq"] == 1.4], key=lambda r: r["mu"])
    freq_results = sorted([r for r in results if r["mu"] == 0.5], key=lambda r: r["step_freq"])

    fig, ax = plt.subplots(figsize=(6, 4))
    ax.plot([r["mu"] for r in mu_results], [r["mean_vx"] for r in mu_results], "o-", color="#2563eb", lw=2)
    ax.set_xlabel("Friction coefficient mu")
    ax.set_ylabel("Mean forward velocity vx [m/s]")
    ax.set_title("mu vs mean vx (step_freq=1.4 Hz, flat, Go2)")
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(ASSETS / "param_study_mu.png", dpi=150)
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(6, 4))
    ax.plot([r["step_freq"] for r in freq_results], [r["mean_vx"] for r in freq_results], "s-", color="#059669", lw=2)
    ax.set_xlabel("step_freq [Hz]")
    ax.set_ylabel("Mean forward velocity vx [m/s]")
    ax.set_title("step_freq vs mean vx (mu=0.5, flat, Go2)")
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(ASSETS / "param_study_step_freq.png", dpi=150)
    plt.close(fig)


def main() -> int:
    results = []
    for mu in [0.35, 0.45, 0.5, 0.55, 0.65]:
        print(f"mu={mu}")
        results.append(simulate(mu, 1.4))
    for freq in [1.0, 1.2, 1.4, 1.6, 1.8]:
        print(f"step_freq={freq}")
        results.append(simulate(0.5, freq))
    (ASSETS / "param_study_results.json").write_text(json.dumps(results, indent=2), encoding="utf-8")
    plot_results(results)
    for r in results:
        print(r)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
