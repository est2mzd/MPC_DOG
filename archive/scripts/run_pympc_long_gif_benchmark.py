#!/usr/bin/env python3
"""Run 30 long PyMPC walks and save one >=20 s GIF per scenario.

Each trial uses the real nominal acados MPC, WBInterface, torque conversion,
soft torque clipping, and MuJoCo Go2 plant. A trial continues until both
20 simulated seconds and 10 m cumulative planar travel are reached. If MuJoCo
terminates, the fall is counted and the robot is reset; therefore the report
separates strict no-fall success from resilient task completion.
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
OUTPUT_DIR = ROOT / "notebook_pympc" / "benchmark_results_long"
GIF_DIR = OUTPUT_DIR / "gifs"
PARTIAL_DIR = OUTPUT_DIR / "partial"
RESULTS_PATH = OUTPUT_DIR / "scenario_results.json"


@dataclass(frozen=True)
class Scenario:
    id: str
    difficulty: str
    description: str
    scene: str = "flat"
    vx: float = 0.6
    yaw_rate: float = 0.0
    ground_mu: float = 0.8
    mpc_mu: float = 0.42
    step_freq: float = 1.25
    duty_factor: float = 0.76
    ref_z_scale: float = 1.08
    foothold_optimization: bool = True
    speed_ramp_s: float = 4.0
    min_duration_s: float = 20.0
    min_distance_m: float = 10.0
    max_duration_s: float = 60.0
    max_falls: int = 20


SCENARIOS = [
    # Easy: flat ground with conservative variations.
    Scenario("E01", "easy", "Flat 0.50 m/s baseline", vx=0.50),
    Scenario("E02", "easy", "Flat 0.55 m/s, slow gait", vx=0.55, step_freq=1.15),
    Scenario("E03", "easy", "Flat 0.60 m/s, long stance", vx=0.60, duty_factor=0.78),
    Scenario("E04", "easy", "Flat 0.60 m/s, conservative MPC friction", vx=0.60, mpc_mu=0.38),
    Scenario("E05", "easy", "Flat 0.60 m/s, fixed footholds", vx=0.60, foothold_optimization=False),
    Scenario("E06", "easy", "Flat 0.55 m/s, physical friction 0.60", vx=0.55, ground_mu=0.60),
    Scenario("E07", "easy", "Flat 0.60 m/s plus gentle yaw", vx=0.60, yaw_rate=0.10),
    Scenario("E08", "easy", "Flat 0.65 m/s, gait 1.20 Hz", vx=0.65, step_freq=1.20),
    Scenario("E09", "easy", "Flat 0.60 m/s, lower body reference", vx=0.60, ref_z_scale=1.06),
    Scenario("E10", "easy", "Flat 0.70 m/s, high physical friction", vx=0.70, ground_mu=1.0),
    # Normal: moderate speed, terrain, turning, or friction.
    Scenario("N01", "normal", "Flat 0.80 m/s", vx=0.80),
    Scenario("N02", "normal", "Flat 0.90 m/s", vx=0.90),
    Scenario("N03", "normal", "Flat 0.70 m/s plus yaw 0.25", vx=0.70, yaw_rate=0.25),
    Scenario("N04", "normal", "Random boxes 0.50 m/s", scene="random_boxes", vx=0.50),
    Scenario(
        "N05", "normal", "Random boxes 0.60 m/s conservative gait",
        scene="random_boxes", vx=0.60, step_freq=1.15, duty_factor=0.78,
    ),
    Scenario("N06", "normal", "Perlin 0.50 m/s", scene="perlin", vx=0.50),
    Scenario("N07", "normal", "Perlin 0.60 m/s", scene="perlin", vx=0.60),
    Scenario("N08", "normal", "Bumpy flat 0.50 m/s", scene="bumpy_flat", vx=0.50),
    Scenario(
        "N09", "normal", "Flat 0.60 m/s, low friction",
        vx=0.60, ground_mu=0.45, mpc_mu=0.35,
    ),
    Scenario(
        "N10", "normal", "Perlin 0.50 m/s plus yaw 0.15",
        scene="perlin", vx=0.50, yaw_rate=0.15,
    ),
    # Hard: speed plus terrain/friction combinations.
    Scenario("H01", "hard", "Flat 1.00 m/s", vx=1.00, speed_ramp_s=6.0),
    Scenario("H02", "hard", "Flat 1.20 m/s", vx=1.20, speed_ramp_s=8.0),
    Scenario(
        "H03", "hard", "Flat 0.90 m/s plus yaw 0.40",
        vx=0.90, yaw_rate=0.40, speed_ramp_s=6.0,
    ),
    Scenario(
        "H04", "hard", "Random boxes 0.70 m/s", scene="random_boxes",
        vx=0.70, step_freq=1.15, duty_factor=0.78, speed_ramp_s=6.0,
    ),
    Scenario(
        "H05", "hard", "Perlin 0.80 m/s", scene="perlin",
        vx=0.80, step_freq=1.20, duty_factor=0.76, speed_ramp_s=6.0,
    ),
    Scenario(
        "H06", "hard", "Perlin 0.70 m/s, low friction", scene="perlin",
        vx=0.70, ground_mu=0.45, mpc_mu=0.35, step_freq=1.15,
        duty_factor=0.78, speed_ramp_s=6.0,
    ),
    Scenario(
        "H07", "hard", "Bumpy flat 0.80 m/s", scene="bumpy_flat",
        vx=0.80, step_freq=1.15, duty_factor=0.78, speed_ramp_s=8.0,
    ),
    Scenario(
        "H08", "hard", "Bumpy uphill 0.60 m/s", scene="bumpy_uphill",
        vx=0.60, mpc_mu=0.38, step_freq=1.10, duty_factor=0.78,
        speed_ramp_s=8.0,
    ),
    Scenario(
        "H09", "hard", "Bumpy downhill 0.60 m/s", scene="bumpy_downhill",
        vx=0.60, mpc_mu=0.35, step_freq=1.05, duty_factor=0.82,
        ref_z_scale=1.10, speed_ramp_s=10.0,
    ),
    Scenario(
        "H10", "hard", "Random boxes 0.60 m/s, low friction",
        scene="random_boxes", vx=0.60, ground_mu=0.45, mpc_mu=0.35,
        step_freq=1.10, duty_factor=0.80, speed_ramp_s=8.0,
    ),
]
SCENARIO_BY_ID = {scenario.id: scenario for scenario in SCENARIOS}


def _install_and_import(scenario: Scenario):
    os.environ.setdefault("MUJOCO_GL", "egl")
    os.environ.setdefault(
        "ACADOS_SOURCE_DIR",
        str(PYMPC_ROOT / "quadruped_pympc" / "acados"),
    )
    for path in (PYMPC_ROOT, ROOT / "scripts"):
        if str(path) not in sys.path:
            sys.path.insert(0, str(path))
    if scenario.scene.startswith("bumpy_"):
        from workshop_terrain import install_custom_terrains

        install_custom_terrains()

    # The image contains precompiled acados solvers but no GNU make.
    # Force the standard AcadosOcpSolver class to load the shipped binary.
    import acados_template

    original = acados_template.AcadosOcpSolver

    class ReusedAcadosOcpSolver(original):
        def __init__(self, *args, **kwargs):
            kwargs.update(generate=False, build=False, check_reuse_possible=False)
            super().__init__(*args, **kwargs)

    acados_template.AcadosOcpSolver = ReusedAcadosOcpSolver


def _draw_overlay(image: np.ndarray, lines: list[str]):
    from PIL import Image, ImageDraw, ImageFont

    frame = Image.fromarray(image).convert("RGBA")
    overlay = Image.new("RGBA", frame.size, (0, 0, 0, 0))
    draw = ImageDraw.Draw(overlay)
    try:
        font = ImageFont.truetype(
            "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", 15
        )
    except OSError:
        font = ImageFont.load_default()
    y = 7
    for line in lines:
        box = draw.textbbox((0, 0), line, font=font)
        width, height = box[2] - box[0], box[3] - box[1]
        draw.rectangle((5, y - 2, width + 15, y + height + 4), fill=(0, 0, 0, 165))
        draw.text((10, y), line, fill=(255, 255, 255, 255), font=font)
        y += height + 7
    return Image.alpha_composite(frame, overlay).convert("RGB")


def _save_gif(frames, path: Path, frame_ms: int) -> float:
    path.parent.mkdir(parents=True, exist_ok=True)
    # Adaptive palettes keep 30 long GIFs substantially smaller than RGB frames.
    paletted = [frame.convert("P", palette=0, colors=128) for frame in frames]
    paletted[0].save(
        path,
        save_all=True,
        append_images=paletted[1:],
        duration=frame_ms,
        loop=0,
        optimize=False,
        disposal=2,
    )
    return len(paletted) * frame_ms / 1000.0


def run_scenario(scenario: Scenario) -> dict[str, Any]:
    _install_and_import(scenario)
    import mujoco
    from gym_quadruped.quadruped_env import QuadrupedEnv
    from gym_quadruped.utils.quadruped_utils import LegsAttr
    from quadruped_pympc import config as cfg
    from quadruped_pympc.quadruped_pympc_wrapper import QuadrupedPyMPC_Wrapper

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
    legs = ("FL", "FR", "RL", "RR")

    def create_env():
        environment = QuadrupedEnv(
            robot=cfg.robot,
            scene=scenario.scene,
            sim_dt=sim_dt,
            ref_base_lin_vel=0.0,
            ref_base_ang_vel=0.0,
            ground_friction_coeff=scenario.ground_mu,
            base_vel_command_type="forward",
            state_obs_names=(),
        )
        environment.mjModel.opt.gravity[2] = -cfg.gravity_constant
        if cfg.qpos0_js is not None:
            environment.mjModel.qpos0 = np.concatenate(
                (environment.mjModel.qpos0[:7], cfg.qpos0_js)
            )
        environment.reset(random=False)
        return environment

    env = create_env()
    renderer = mujoco.Renderer(env.mjModel, height=360, width=480)
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

    camera = mujoco.MjvCamera()
    camera.type = mujoco.mjtCamera.mjCAMERA_FREE
    camera.distance = 3.5
    camera.elevation = -25
    camera.azimuth = 120
    # GIF stores duration in 10 ms units. 130 ms guarantees that a 20 s
    # simulated run remains >=20 s after GIF timing quantization.
    frame_ms = 130
    capture_stride = round((frame_ms / 1000.0) / sim_dt)
    frames = []
    control_ms: list[float] = []
    solve_ms: list[float] = []
    velocity_error_sq: list[float] = []
    heights: list[float] = []
    rolls: list[float] = []
    pitches: list[float] = []
    raw_torque_abs: list[float] = []
    status_histogram: Counter[int] = Counter()
    saturated = torque_count = 0
    falls = 0
    cumulative_distance = 0.0
    previous_xy = np.asarray(env.base_pos[:2], dtype=float).copy()
    max_steps = round(scenario.max_duration_s / sim_dt)
    mpc_stride = round(1 / (cfg.simulation_params["mpc_frequency"] * sim_dt))
    wall_start = time.perf_counter()

    def reset_after_fall():
        nonlocal previous_xy, tau
        env.reset(random=False)
        wrapper.reset(initial_feet_pos=env.feet_pos(frame="world"))
        previous_xy = np.asarray(env.base_pos[:2], dtype=float).copy()
        tau = LegsAttr(*[np.zeros((3, 1)) for _ in legs])

    try:
        for step in range(max_steps):
            simulated_s = step * sim_dt
            ramp = min(1.0, simulated_s / scenario.speed_ramp_s)
            command_vx = scenario.vx * ramp
            command_yaw = scenario.yaw_rate * ramp
            env._ref_base_lin_vel_H = np.array([command_vx, 0.0, 0.0])
            env._ref_base_ang_yaw_dot = command_yaw
            ref_lin_w, ref_ang_w = env.target_base_vel(frame="world")

            started = time.perf_counter()
            tau = wrapper.compute_actions(
                copy.deepcopy(env.com),
                copy.deepcopy(env.base_pos),
                env.base_lin_vel(frame="world"),
                env.base_ori_euler_xyz,
                env.base_ang_vel(frame="base"),
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
                status_histogram[int(controller.previous_status)] += 1
                solve_ms.append(
                    float(controller.acados_ocp_solver.get_stats("time_tot")) * 1000.0
                )

            action = np.zeros(env.mjModel.nu)
            for leg in legs:
                raw = np.asarray(tau[leg], dtype=float).reshape(3)
                limits = np.asarray(tau_limits[leg], dtype=float)
                low, high = limits[:, 0], limits[:, 1]
                saturated += int(np.count_nonzero((raw < low) | (raw > high)))
                torque_count += 3
                raw_torque_abs.extend(np.abs(raw).tolist())
                action[getattr(env.legs_tau_idx, leg)] = np.clip(raw, low, high)

            _, _, terminated, truncated, _ = env.step(action=action)
            current_xy = np.asarray(env.base_pos[:2], dtype=float).copy()
            cumulative_distance += float(np.linalg.norm(current_xy - previous_xy))
            previous_xy = current_xy
            measured_v = env.base_lin_vel(frame="world")
            velocity_error_sq.append(float(np.sum((measured_v[:2] - ref_lin_w[:2]) ** 2)))
            heights.append(float(env.base_pos[2]))
            rolls.append(float(env.base_ori_euler_xyz[0]))
            pitches.append(float(env.base_ori_euler_xyz[1]))

            if step % capture_stride == 0:
                camera.lookat[:] = env.base_pos
                renderer.update_scene(env.mjData, camera=camera)
                result_now = (
                    "GOAL"
                    if cumulative_distance >= scenario.min_distance_m
                    else "RUNNING"
                )
                frames.append(
                    _draw_overlay(
                        renderer.render(),
                        [
                            f"{scenario.id} [{scenario.difficulty}] {scenario.scene}",
                            f"cmd {scenario.vx:.2f} m/s | t {simulated_s:05.1f} s",
                            f"distance {cumulative_distance:05.2f} m | falls {falls}",
                            result_now,
                        ],
                    )
                )

            if terminated or truncated:
                falls += 1
                elapsed = (step + 1) * sim_dt
                # Keep recording until 20 s even when the scenario has
                # already exceeded its fall budget, so every failure also
                # has a diagnostically useful >=20 s GIF.
                if falls > scenario.max_falls and elapsed >= scenario.min_duration_s:
                    break
                reset_after_fall()

            elapsed = (step + 1) * sim_dt
            if (
                elapsed >= scenario.min_duration_s
                and cumulative_distance >= scenario.min_distance_m
            ):
                break
    finally:
        wall_s = time.perf_counter() - wall_start
        renderer.close()
        env.close()

    simulated_s = len(control_ms) * sim_dt
    # Ensure a frame at the final state and exact >=20 s playback for completed trials.
    if not frames:
        raise RuntimeError("No GIF frames captured")
    gif_path = GIF_DIR / f"{scenario.id}.gif"
    gif_playback_s = _save_gif(frames, gif_path, frame_ms)
    speed_rmse = float(np.sqrt(np.mean(velocity_error_sq)))
    task_completed = (
        simulated_s >= scenario.min_duration_s
        and cumulative_distance >= scenario.min_distance_m
    )
    strict_success = (
        task_completed
        and falls == 0
        and np.degrees(max(map(abs, rolls), default=0.0)) <= 35.0
        and np.degrees(max(map(abs, pitches), default=0.0)) <= 35.0
    )
    return {
        "scenario": asdict(scenario),
        "metrics": {
            "task_completed": bool(task_completed),
            "strict_success": bool(strict_success),
            "simulated_s": simulated_s,
            "gif_playback_s": gif_playback_s,
            "cumulative_distance_m": cumulative_distance,
            "falls": falls,
            "speed_rmse_mps": speed_rmse,
            "min_height_m": min(heights, default=float("nan")),
            "max_abs_roll_deg": float(np.degrees(max(map(abs, rolls), default=0.0))),
            "max_abs_pitch_deg": float(np.degrees(max(map(abs, pitches), default=0.0))),
            "torque_saturation_rate": saturated / torque_count if torque_count else 0.0,
            "max_abs_raw_torque_nm": max(raw_torque_abs, default=0.0),
            "mpc_solve_mean_ms": float(np.mean(solve_ms)),
            "mpc_solve_p95_ms": float(np.percentile(solve_ms, 95)),
            "control_mean_ms": float(np.mean(control_ms)),
            "wall_s": wall_s,
            "realtime_factor": simulated_s / wall_s,
            "solver_status_histogram": {
                str(key): value for key, value in sorted(status_histogram.items())
            },
            "gif_path": str(gif_path.relative_to(ROOT / "notebook_pympc")),
        },
    }


def run_child(scenario: Scenario, path: Path) -> int:
    command = [
        sys.executable,
        str(Path(__file__).resolve()),
        "--scenario",
        scenario.id,
        "--result-path",
        str(path),
    ]
    return subprocess.run(command, cwd=ROOT, check=False).returncode


def run_all() -> dict[str, Any]:
    for directory in (OUTPUT_DIR, GIF_DIR, PARTIAL_DIR):
        directory.mkdir(parents=True, exist_ok=True)
    results = []
    started = time.perf_counter()
    for index, scenario in enumerate(SCENARIOS, 1):
        print(f"[{index:02d}/30] {scenario.id}: {scenario.description}", flush=True)
        path = PARTIAL_DIR / f"{scenario.id}.json"
        gif_path = GIF_DIR / f"{scenario.id}.gif"
        if path.is_file() and gif_path.is_file():
            cached = json.loads(path.read_text(encoding="utf-8"))
            metrics = cached.get("metrics", {})
            if (
                metrics.get("gif_playback_s", 0) >= 20.0
                and metrics.get("cumulative_distance_m", 0) >= 10.0
            ):
                results.append(cached)
                print("  reused completed cached result", flush=True)
                continue
        return_code = run_child(scenario, path)
        if return_code != 0:
            results.append(
                {"scenario": asdict(scenario), "error": f"exit code {return_code}"}
            )
            continue
        result = json.loads(path.read_text(encoding="utf-8"))
        results.append(result)
        metrics = result["metrics"]
        print(
            f"  task={metrics['task_completed']} strict={metrics['strict_success']} "
            f"t={metrics['simulated_s']:.1f}s d={metrics['cumulative_distance_m']:.1f}m "
            f"falls={metrics['falls']} gif={metrics['gif_playback_s']:.1f}s",
            flush=True,
        )
        # Checkpoint the aggregate so an interrupted long run remains usable.
        RESULTS_PATH.write_text(
            json.dumps({"results": results}, indent=2, ensure_ascii=False) + "\n",
            encoding="utf-8",
        )
    payload = {
        "benchmark": {
            "name": "Quadruped-PyMPC 30-scenario long GIF benchmark",
            "requirements": "each GIF >=20 s and cumulative walk >=10 m",
            "scenario_count": 30,
            "wall_s": time.perf_counter() - started,
        },
        "results": results,
    }
    RESULTS_PATH.write_text(
        json.dumps(payload, indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    return payload


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--scenario", choices=sorted(SCENARIO_BY_ID))
    parser.add_argument("--result-path", type=Path)
    args = parser.parse_args()
    if args.scenario:
        if args.result_path is None:
            parser.error("--result-path is required with --scenario")
        result = run_scenario(SCENARIO_BY_ID[args.scenario])
        args.result_path.parent.mkdir(parents=True, exist_ok=True)
        args.result_path.write_text(
            json.dumps(result, indent=2, ensure_ascii=False) + "\n",
            encoding="utf-8",
        )
        return 0
    payload = run_all()
    completed = sum("metrics" in row for row in payload["results"])
    print(f"completed {completed}/30; wrote {RESULTS_PATH}")
    return 0 if completed == 30 else 1


if __name__ == "__main__":
    raise SystemExit(main())
