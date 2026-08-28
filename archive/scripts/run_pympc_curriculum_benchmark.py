#!/usr/bin/env python3
"""Run 30 reproducible headless scenarios with the real Quadruped-PyMPC stack.

The benchmark uses the nominal acados controller, WBInterface, swing/stance
torque conversion, torque clipping, and MuJoCo plant. Each scenario runs in a
fresh subprocess so in-memory config changes and controller state cannot leak
between trials.
"""
from __future__ import annotations

import argparse
import copy
import json
import os
import subprocess
import sys
import time
from collections import Counter
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
PYMPC_ROOT = ROOT / "external" / "Quadruped-PyMPC"
OUTPUT_DIR = ROOT / "notebook_pympc" / "benchmark_results"
RESULTS_PATH = OUTPUT_DIR / "scenario_results.json"


@dataclass(frozen=True)
class Scenario:
    id: str
    difficulty: str
    description: str
    scene: str = "flat"
    duration_s: float = 5.0
    vx: float = 0.0
    vy: float = 0.0
    yaw_rate: float = 0.0
    ground_mu: float = 0.8
    mpc_mu: float = 0.42
    step_freq: float = 1.35
    duty_factor: float = 0.74
    ref_z_scale: float = 1.08
    foothold_optimization: bool = True


SCENARIOS = [
    # Easy: flat ground, small commands, one effect at a time.
    Scenario("E01", "easy", "Flat standstill", vx=0.0),
    Scenario("E02", "easy", "Flat forward 0.10 m/s", vx=0.10),
    Scenario("E03", "easy", "Flat forward 0.20 m/s", vx=0.20),
    Scenario("E04", "easy", "Flat forward 0.30 m/s", vx=0.30),
    Scenario("E05", "easy", "Flat backward -0.10 m/s", vx=-0.10),
    Scenario("E06", "easy", "Flat lateral 0.10 m/s", vy=0.10),
    Scenario("E07", "easy", "Flat yaw 0.15 rad/s", yaw_rate=0.15),
    Scenario("E08", "easy", "Flat diagonal 0.15/0.10 m/s", vx=0.15, vy=0.10),
    Scenario("E09", "easy", "Flat forward with high physical friction", vx=0.25, ground_mu=1.0),
    Scenario("E10", "easy", "Flat forward, fixed footholds", vx=0.20, foothold_optimization=False),
    # Normal: faster commands or one moderate terrain/turning challenge.
    Scenario("N01", "normal", "Flat forward 0.40 m/s", vx=0.40),
    Scenario("N02", "normal", "Flat forward 0.60 m/s", vx=0.60),
    Scenario("N03", "normal", "Flat forward plus yaw", vx=0.30, yaw_rate=0.25),
    Scenario("N04", "normal", "Flat lateral 0.25 m/s", vy=0.25),
    Scenario(
        "N05", "normal", "Random boxes conservative gait", scene="random_boxes",
        vx=0.20, step_freq=1.15, duty_factor=0.78,
    ),
    Scenario("N06", "normal", "Random boxes 0.35 m/s", scene="random_boxes", vx=0.35),
    Scenario(
        "N07", "normal", "Perlin diagonal 0.20/0.10 m/s", scene="perlin",
        vx=0.20, vy=0.10, step_freq=1.15, duty_factor=0.78,
    ),
    Scenario(
        "N08", "normal", "Perlin 0.20 m/s", scene="perlin",
        vx=0.20, step_freq=1.20, duty_factor=0.76,
    ),
    Scenario(
        "N09", "normal", "Flat low-friction floor", vx=0.30,
        ground_mu=0.45, mpc_mu=0.35, duty_factor=0.76,
    ),
    Scenario(
        "N10", "normal", "Perlin 0.30 m/s optimized footholds", scene="perlin",
        vx=0.30, step_freq=1.20, duty_factor=0.76,
    ),
    # Hard: speed, combined commands, severe terrain, or slope.
    Scenario("H01", "hard", "Flat forward 0.80 m/s", duration_s=6.0, vx=0.80),
    Scenario("H02", "hard", "Flat forward 1.00 m/s", duration_s=6.0, vx=1.00),
    Scenario(
        "H03", "hard", "Flat 0.60 m/s plus 0.50 rad/s yaw",
        duration_s=6.0, vx=0.60, yaw_rate=0.50,
    ),
    Scenario("H04", "hard", "Flat lateral 0.50 m/s", duration_s=6.0, vy=0.50),
    Scenario(
        "H05", "hard", "Perlin 0.50 m/s", scene="perlin", duration_s=6.0,
        vx=0.50, step_freq=1.20, duty_factor=0.76,
    ),
    Scenario(
        "H06", "hard", "Random boxes 0.50 m/s", scene="random_boxes",
        duration_s=6.0, vx=0.50, step_freq=1.20, duty_factor=0.76,
    ),
    Scenario(
        "H07", "hard", "Perlin low friction 0.45 m/s", scene="perlin",
        duration_s=6.0, vx=0.45, ground_mu=0.45, mpc_mu=0.35,
        step_freq=1.15, duty_factor=0.78,
    ),
    Scenario(
        "H08", "hard", "Bumpy flat 0.60 m/s", scene="bumpy_flat",
        duration_s=6.0, vx=0.60, step_freq=1.20, duty_factor=0.76,
    ),
    Scenario(
        "H09", "hard", "Bumpy uphill 0.40 m/s", scene="bumpy_uphill",
        duration_s=6.0, vx=0.40, mpc_mu=0.38, step_freq=1.10,
        duty_factor=0.78, ref_z_scale=1.08,
    ),
    Scenario(
        "H10", "hard", "Bumpy downhill 0.40 m/s", scene="bumpy_downhill",
        duration_s=6.0, vx=0.40, mpc_mu=0.35, step_freq=1.05,
        duty_factor=0.82, ref_z_scale=1.10,
    ),
]

SCENARIO_BY_ID = {scenario.id: scenario for scenario in SCENARIOS}


def _percentile(values: list[float], q: float) -> float:
    return float(np.percentile(values, q)) if values else 0.0


def run_scenario(scenario: Scenario) -> dict[str, Any]:
    """Execute one scenario through the actual PyMPC and MuJoCo control loop."""
    os.environ.setdefault("MUJOCO_GL", "egl")
    os.environ.setdefault(
        "ACADOS_SOURCE_DIR",
        str(PYMPC_ROOT / "quadruped_pympc" / "acados"),
    )
    if str(PYMPC_ROOT) not in sys.path:
        sys.path.insert(0, str(PYMPC_ROOT))
    if str(ROOT / "scripts") not in sys.path:
        sys.path.insert(0, str(ROOT / "scripts"))

    if scenario.scene.startswith("bumpy_"):
        from workshop_terrain import install_custom_terrains

        install_custom_terrains()

    from gym_quadruped.quadruped_env import QuadrupedEnv
    from gym_quadruped.utils.quadruped_utils import LegsAttr
    from quadruped_pympc import config as cfg

    # This teaching image ships the acados shared objects but not GNU make.
    # Reuse those generated solvers instead of attempting code generation.
    # The dynamics, OCP, solver calls, WB controller, and plant remain the real
    # repository implementation; only the already-built binary is loaded.
    import acados_template

    if not any(Path(path).joinpath("make").is_file() for path in os.environ.get("PATH", "").split(os.pathsep)):
        original_solver_class = acados_template.AcadosOcpSolver

        class ReusedAcadosOcpSolver(original_solver_class):
            def __init__(self, *args, **kwargs):
                kwargs.update(
                    generate=False,
                    build=False,
                    check_reuse_possible=False,
                )
                super().__init__(*args, **kwargs)

        acados_template.AcadosOcpSolver = ReusedAcadosOcpSolver

    from quadruped_pympc.quadruped_pympc_wrapper import QuadrupedPyMPC_Wrapper

    # Explicit baseline: every subprocess starts from the same nominal controller.
    cfg.mpc_params.update(
        {
            "type": "nominal",
            "horizon": 12,
            "dt": 0.02,
            "mu": scenario.mpc_mu,
            "grf_max": cfg.mass * cfg.gravity_constant,
            "grf_min": 0,
            "optimize_step_freq": False,
            "use_foothold_optimization": scenario.foothold_optimization,
            "use_foothold_constraints": False,
            "use_integrators": False,
            "use_RTI": False,
            "use_DDP": False,
            "num_qp_iterations": 1,
            "solver_mode": "balance",
        }
    )
    cfg.simulation_params.update(
        {
            "scene": scenario.scene,
            "gait": "trot",
            "dt": 0.002,
            "mpc_frequency": 100,
            "visual_foothold_adaptation": "blind",
            "ref_z": cfg.hip_height * scenario.ref_z_scale,
            "use_inertia_recomputation": True,
        }
    )
    cfg.simulation_params["gait_params"]["trot"].update(
        {"step_freq": scenario.step_freq, "duty_factor": scenario.duty_factor}
    )

    sim_dt = cfg.simulation_params["dt"]
    env = QuadrupedEnv(
        robot=cfg.robot,
        scene=scenario.scene,
        sim_dt=sim_dt,
        ref_base_lin_vel=0.0,
        ref_base_ang_vel=0.0,
        ground_friction_coeff=scenario.ground_mu,
        base_vel_command_type="forward",
        state_obs_names=(),
    )
    env.mjModel.opt.gravity[2] = -cfg.gravity_constant
    if cfg.qpos0_js is not None:
        env.mjModel.qpos0 = np.concatenate((env.mjModel.qpos0[:7], cfg.qpos0_js))
    env.reset(random=False)

    legs = ("FL", "FR", "RL", "RR")
    wrapper = QuadrupedPyMPC_Wrapper(
        initial_feet_pos=env.feet_pos,
        legs_order=legs,
        feet_geom_id=env._feet_geom_id,
    )
    tau = LegsAttr(*[np.zeros((3, 1)) for _ in legs])
    tau_limits = LegsAttr(
        **{
            leg: env.mjModel.actuator_ctrlrange[getattr(env.legs_tau_idx, leg)] * 0.9
            for leg in legs
        }
    )

    position0 = np.asarray(env.base_pos[:2], dtype=float).copy()
    command_h = np.array([scenario.vx, scenario.vy, 0.0])
    control_ms: list[float] = []
    solve_ms: list[float] = []
    velocity_error_sq: list[float] = []
    yaw_error_sq: list[float] = []
    heights: list[float] = []
    rolls: list[float] = []
    pitches: list[float] = []
    raw_torque_abs: list[float] = []
    solver_failures = 0
    solver_statuses: Counter[int] = Counter()
    saturated = 0
    torque_count = 0
    terminated = False
    termination_step: int | None = None
    max_steps = int(round(scenario.duration_s / sim_dt))
    mpc_stride = round(1 / (cfg.simulation_params["mpc_frequency"] * sim_dt))

    wall_start = time.perf_counter()
    try:
        for step in range(max_steps):
            # Hold a deterministic heading-frame command for the whole trial.
            env._ref_base_lin_vel_H = command_h.copy()
            env._ref_base_ang_yaw_dot = scenario.yaw_rate
            ref_lin_w, ref_ang_w = env.target_base_vel(frame="world")

            base_lin_w = env.base_lin_vel(frame="world")
            base_ang_b = env.base_ang_vel(frame="base")
            started = time.perf_counter()
            tau = wrapper.compute_actions(
                copy.deepcopy(env.com),
                copy.deepcopy(env.base_pos),
                base_lin_w,
                env.base_ori_euler_xyz,
                base_ang_b,
                env.feet_pos(frame="world"),
                env.hip_positions(frame="world"),
                LegsAttr(
                    FL=env.legs_qvel_idx.FL,
                    FR=env.legs_qvel_idx.FR,
                    RL=env.legs_qvel_idx.RL,
                    RR=env.legs_qvel_idx.RR,
                ),
                None,
                legs,
                sim_dt,
                ref_lin_w,
                ref_ang_w,
                env.step_num,
                env.mjData.qpos,
                env.mjData.qvel,
                env.feet_jacobians(frame="world", return_rot_jac=False),
                env.feet_jacobians_dot(frame="world", return_rot_jac=False),
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
            control_ms.append((time.perf_counter() - started) * 1000.0)

            if step % mpc_stride == 0:
                controller = wrapper.srbd_controller_interface.controller
                status = int(controller.previous_status)
                solver_statuses[status] += 1
                # Match the upstream fallback condition in
                # centroidal_nmpc_nominal.py rather than treating SQP
                # max-iteration status 2 as a hard failure.
                solver_failures += int(status in (1, 4))
                try:
                    solve_ms.append(float(controller.acados_ocp_solver.get_stats("time_tot")) * 1000.0)
                except Exception:
                    pass

            action = np.zeros(env.mjModel.nu)
            for leg in legs:
                raw = np.asarray(tau[leg], dtype=float).reshape(3)
                limits = np.asarray(tau_limits[leg], dtype=float)
                low, high = limits[:, 0], limits[:, 1]
                saturated += int(np.count_nonzero((raw < low) | (raw > high)))
                torque_count += raw.size
                raw_torque_abs.extend(np.abs(raw).tolist())
                clipped = np.clip(raw, low, high)
                action[getattr(env.legs_tau_idx, leg)] = clipped

            _, _, term, trunc, _ = env.step(action=action)
            measured_v = env.base_lin_vel(frame="world")
            measured_w = env.base_ang_vel(frame="base")
            velocity_error_sq.append(float(np.sum((measured_v[:2] - ref_lin_w[:2]) ** 2)))
            yaw_error_sq.append(float((measured_w[2] - scenario.yaw_rate) ** 2))
            heights.append(float(env.base_pos[2]))
            rolls.append(float(env.base_ori_euler_xyz[0]))
            pitches.append(float(env.base_ori_euler_xyz[1]))
            if term or trunc:
                terminated = True
                termination_step = step + 1
                break
    finally:
        wall_s = time.perf_counter() - wall_start
        final_position = np.asarray(env.base_pos[:2], dtype=float).copy()
        env.close()

    steps = len(control_ms)
    simulated_s = steps * sim_dt
    speed_rmse = float(np.sqrt(np.mean(velocity_error_sq))) if velocity_error_sq else float("nan")
    yaw_rmse = float(np.sqrt(np.mean(yaw_error_sq))) if yaw_error_sq else float("nan")
    command_speed = float(np.linalg.norm(command_h[:2]))
    tracking_limit = max(0.18, 0.55 * command_speed)
    stable = (
        not terminated
        and simulated_s >= scenario.duration_s - sim_dt
        and (min(heights) if heights else 0.0) >= 0.16
        and np.degrees(max(map(abs, rolls), default=0.0)) <= 35.0
        and np.degrees(max(map(abs, pitches), default=0.0)) <= 35.0
    )
    tracking_pass = speed_rmse <= tracking_limit and yaw_rmse <= max(0.20, abs(scenario.yaw_rate) * 0.75)

    return {
        "scenario": asdict(scenario),
        "metrics": {
            "success": bool(stable and tracking_pass),
            "stable": bool(stable),
            "tracking_pass": bool(tracking_pass),
            "terminated": terminated,
            "termination_step": termination_step,
            "simulated_s": simulated_s,
            "wall_s": wall_s,
            "realtime_factor": simulated_s / wall_s if wall_s > 0 else 0.0,
            "planar_displacement_m": float(np.linalg.norm(final_position - position0)),
            "speed_rmse_mps": speed_rmse,
            "speed_tracking_limit_mps": tracking_limit,
            "yaw_rate_rmse_radps": yaw_rmse,
            "min_height_m": min(heights) if heights else float("nan"),
            "max_height_m": max(heights) if heights else float("nan"),
            "max_abs_roll_deg": float(np.degrees(max(map(abs, rolls), default=0.0))),
            "max_abs_pitch_deg": float(np.degrees(max(map(abs, pitches), default=0.0))),
            "torque_saturation_rate": saturated / torque_count if torque_count else 0.0,
            "max_abs_raw_torque_nm": max(raw_torque_abs, default=0.0),
            "control_mean_ms": float(np.mean(control_ms)) if control_ms else 0.0,
            "control_p95_ms": _percentile(control_ms, 95),
            "control_max_ms": max(control_ms, default=0.0),
            "mpc_solve_mean_ms": float(np.mean(solve_ms)) if solve_ms else 0.0,
            "mpc_solve_p95_ms": _percentile(solve_ms, 95),
            "mpc_solve_max_ms": max(solve_ms, default=0.0),
            "mpc_updates": len(solve_ms),
            "solver_failures": solver_failures,
            "solver_status_histogram": {str(key): value for key, value in sorted(solver_statuses.items())},
        },
    }


def _run_child(scenario: Scenario, result_path: Path) -> int:
    command = [
        sys.executable,
        str(Path(__file__).resolve()),
        "--scenario",
        scenario.id,
        "--result-path",
        str(result_path),
    ]
    completed = subprocess.run(command, cwd=ROOT, check=False)
    return completed.returncode


def run_all() -> dict[str, Any]:
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    partial_dir = OUTPUT_DIR / "partial"
    partial_dir.mkdir(exist_ok=True)
    results = []
    started = time.perf_counter()
    for index, scenario in enumerate(SCENARIOS, 1):
        print(
            f"[{index:02d}/{len(SCENARIOS)}] {scenario.id} "
            f"{scenario.difficulty}: {scenario.description}",
            flush=True,
        )
        partial_path = partial_dir / f"{scenario.id}.json"
        code = _run_child(scenario, partial_path)
        if code != 0:
            results.append(
                {
                    "scenario": asdict(scenario),
                    "error": f"subprocess exit code {code}",
                }
            )
            continue
        results.append(json.loads(partial_path.read_text(encoding="utf-8")))
        metrics = results[-1]["metrics"]
        print(
            f"  success={metrics['success']} stable={metrics['stable']} "
            f"RMSE={metrics['speed_rmse_mps']:.3f} m/s "
            f"MPC p95={metrics['mpc_solve_p95_ms']:.2f} ms",
            flush=True,
        )

    payload = {
        "benchmark": {
            "name": "Quadruped-PyMPC 30-scenario curriculum benchmark",
            "controller": "nominal acados NMPC",
            "scenario_count": len(SCENARIOS),
            "wall_s": time.perf_counter() - started,
            "success_definition": (
                "no termination; complete duration; z>=0.16 m; "
                "|roll|,|pitch|<=35 deg; speed/yaw RMSE within scenario thresholds"
            ),
        },
        "results": results,
    }
    RESULTS_PATH.write_text(json.dumps(payload, indent=2, ensure_ascii=False), encoding="utf-8")
    return payload


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--scenario", choices=sorted(SCENARIO_BY_ID))
    parser.add_argument("--result-path", type=Path)
    args = parser.parse_args()
    if args.scenario:
        if args.result_path is None:
            parser.error("--scenario requires --result-path")
        scenario = SCENARIO_BY_ID[args.scenario]
        try:
            result = run_scenario(scenario)
        except Exception as exc:
            print(f"{scenario.id} failed: {type(exc).__name__}: {exc}", file=sys.stderr)
            raise
        args.result_path.parent.mkdir(parents=True, exist_ok=True)
        args.result_path.write_text(
            json.dumps(result, indent=2, ensure_ascii=False),
            encoding="utf-8",
        )
        return 0

    payload = run_all()
    valid = [row for row in payload["results"] if "metrics" in row]
    print(f"wrote {RESULTS_PATH}")
    print(f"completed {len(valid)}/{len(SCENARIOS)} scenarios")
    return 0 if len(valid) == len(SCENARIOS) else 1


if __name__ == "__main__":
    raise SystemExit(main())
