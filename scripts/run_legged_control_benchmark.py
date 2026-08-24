#!/usr/bin/env python3
"""Reproducible 30-scenario benchmark for the legged_control MuJoCo A1 adapter.

This intentionally benchmarks ``src/legged_control_mujoco``.  It does not use
Quadruped-PyMPC, and it never resets or truncates the simulation after a fall.
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import os
import sys
import time
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any, Sequence

os.environ.setdefault("MUJOCO_GL", "egl")

import mujoco
import numpy as np
from PIL import Image, ImageDraw, ImageFont

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT / "src") not in sys.path:
    sys.path.insert(0, str(ROOT / "src"))

from legged_control_mujoco import (  # noqa: E402
    A1HeadlessAdapter,
    MotionCommand,
    ScenarioConfig,
    TORQUE_LIMIT,
)
from legged_control_mujoco.adapter import UPSTREAM_COMMIT  # noqa: E402

DEFAULT_OUTPUT_ROOT = ROOT / "notebook_legged" / "assets" / "scenarios"
IMPLEMENTATION_REVISION = "c51ef9e"
IMPLEMENTATION_BOUNDARY = (
    "qiayuanl/legged_control gait/state/input/WBC/hybrid-command interfaces; "
    "deterministic instantaneous friction-constrained force planner and MuJoCo "
    "acceleration-level WBC replace ROS/OCS2 SQP; not Quadruped-PyMPC"
)
THRESHOLD_DEFINITIONS = {
    "fall": "base height <0.18 m or |roll|/|pitch| >0.9 rad at any control sample",
    "height": "base z relative to the scenario's commanded base height",
    "attitude": "maximum absolute roll or pitch at any control sample",
    "velocity": "RMSE of world-frame vx/vy and yaw-rate tracking",
    "torque": "applied hybrid-command torque; A1 effort limit is 33.5 N m",
    "dynamics_residual": (
        "maximum L2 norm of post-torque-saturation generalized equations-of-motion "
        "residual; translational rows are N and rotational rows are N m"
    ),
    "contact_agreement": (
        "fraction of foot/control-sample booleans where planned stance equals "
        "measured MuJoCo foot contact"
    ),
}


@dataclass(frozen=True)
class BenchmarkScenario:
    name: str
    difficulty: str
    description: str
    gait: str
    vx: float = 0.0
    vy: float = 0.0
    yaw_rate: float = 0.0
    base_height: float = 0.32
    friction: float = 0.6
    payload_mass: float = 0.0
    push_force: tuple[float, float, float] = (0.0, 0.0, 0.0)
    push_start: float = 0.0
    push_duration: float = 0.0
    seed: int = 0


SCENARIOS: tuple[BenchmarkScenario, ...] = (
    # Easy: static balance and conservative static-walk variations.
    BenchmarkScenario("E01_stance_baseline", "easy", "Nominal stance", "stance", seed=101),
    BenchmarkScenario(
        "E02_stance_low", "easy", "Lower 0.30 m stance", "stance",
        base_height=0.30, seed=102,
    ),
    BenchmarkScenario(
        "E03_stance_high", "easy", "Higher 0.34 m stance", "stance",
        base_height=0.34, seed=103,
    ),
    BenchmarkScenario(
        "E04_walk_005", "easy", "Very slow forward static walk", "static_walk",
        vx=0.05, seed=104,
    ),
    BenchmarkScenario(
        "E05_walk_008", "easy", "Slow forward static walk", "static_walk",
        vx=0.08, seed=105,
    ),
    BenchmarkScenario(
        "E06_walk_lateral", "easy", "Slow lateral static walk", "static_walk",
        vy=0.04, seed=106,
    ),
    BenchmarkScenario(
        "E07_stance_payload", "easy", "Stance with 1 kg payload", "stance",
        payload_mass=1.0, seed=107,
    ),
    BenchmarkScenario(
        "E08_stance_gentle_push", "easy", "Stance with gentle timed lateral push",
        "stance", push_force=(0.0, 15.0, 0.0), push_start=8.0,
        push_duration=0.15, seed=108,
    ),
    BenchmarkScenario(
        "E09_walk_turn", "easy", "Slow static walk with gentle turn", "static_walk",
        vx=0.06, yaw_rate=0.08, seed=109,
    ),
    BenchmarkScenario(
        "E10_walk_012", "easy", "Verified 0.12 m/s static-walk command",
        "static_walk", vx=0.12, seed=110,
    ),
    # Normal: gait changes, combined motion, payload, and disturbances.
    BenchmarkScenario(
        "N01_walk_016", "normal", "Moderate forward static walk", "static_walk",
        vx=0.16, seed=201,
    ),
    BenchmarkScenario(
        "N02_walk_diagonal", "normal", "Diagonal static walk", "static_walk",
        vx=0.12, vy=0.08, seed=202,
    ),
    BenchmarkScenario(
        "N03_walk_turn", "normal", "Forward static walk with turn", "static_walk",
        vx=0.12, yaw_rate=0.20, seed=203,
    ),
    BenchmarkScenario(
        "N04_dynamic_walk", "normal", "Dynamic-walk gait", "dynamic_walk",
        vx=0.10, seed=204,
    ),
    BenchmarkScenario(
        "N05_standing_trot", "normal", "Standing-trot gait", "standing_trot",
        vx=0.08, seed=205,
    ),
    BenchmarkScenario(
        "N06_trot", "normal", "Conservative trot", "trot", vx=0.08, seed=206,
    ),
    BenchmarkScenario(
        "N07_walk_payload", "normal", "Static walk with 2 kg payload", "static_walk",
        vx=0.10, payload_mass=2.0, seed=207,
    ),
    BenchmarkScenario(
        "N08_walk_push", "normal", "Static walk with timed lateral push",
        "static_walk", vx=0.10, push_force=(0.0, 30.0, 0.0),
        push_start=8.0, push_duration=0.20, seed=208,
    ),
    BenchmarkScenario(
        "N09_walk_mu045", "normal", "Static walk on reduced friction",
        "static_walk", vx=0.10, friction=0.45, seed=209,
    ),
    BenchmarkScenario(
        "N10_walk_low_turn_push", "normal",
        "Lower body walk, turn, and forward push", "static_walk",
        vx=0.10, yaw_rate=0.15, base_height=0.30,
        push_force=(25.0, 0.0, 0.0), push_start=12.0,
        push_duration=0.20, seed=210,
    ),
    # Hard: aggressive commands and compounded physical disturbances.
    BenchmarkScenario(
        "H01_walk_025", "hard", "Aggressive forward static walk", "static_walk",
        vx=0.25, seed=301,
    ),
    BenchmarkScenario(
        "H02_walk_strafe", "hard", "Aggressive diagonal static walk",
        "static_walk", vx=0.20, vy=0.15, seed=302,
    ),
    BenchmarkScenario(
        "H03_walk_fast_turn", "hard", "Fast walk and turn", "static_walk",
        vx=0.20, yaw_rate=0.45, seed=303,
    ),
    BenchmarkScenario(
        "H04_trot_fast", "hard", "Aggressive trot command", "trot",
        vx=0.20, seed=304,
    ),
    BenchmarkScenario(
        "H05_flying_trot", "hard", "Flying-trot command", "flying_trot",
        vx=0.15, seed=305,
    ),
    BenchmarkScenario(
        "H06_pace", "hard", "Pace with lateral command", "pace",
        vx=0.12, vy=0.10, seed=306,
    ),
    BenchmarkScenario(
        "H07_low_friction", "hard", "Walk on low-friction floor", "static_walk",
        vx=0.15, friction=0.25, seed=307,
    ),
    BenchmarkScenario(
        "H08_heavy_payload", "hard", "Walk with 4 kg payload", "static_walk",
        vx=0.15, payload_mass=4.0, seed=308,
    ),
    BenchmarkScenario(
        "H09_strong_push", "hard", "Walk with strong lateral impulse",
        "static_walk", vx=0.15, push_force=(0.0, 70.0, 0.0),
        push_start=8.0, push_duration=0.30, seed=309,
    ),
    BenchmarkScenario(
        "H10_compound", "hard",
        "Low friction, payload, turn, and strong diagonal push", "static_walk",
        vx=0.20, vy=0.08, yaw_rate=0.35, base_height=0.30,
        friction=0.25, payload_mass=3.0, push_force=(55.0, 55.0, 0.0),
        push_start=8.0, push_duration=0.30, seed=310,
    ),
)
SCENARIO_BY_NAME = {scenario.name: scenario for scenario in SCENARIOS}


@dataclass(frozen=True)
class Thresholds:
    minimum_simulated_duration_s: float
    minimum_gif_playback_duration_s: float
    fall_allowed: bool = False
    minimum_base_height_m: float = 0.18
    maximum_height_rmse_m: float = 0.10
    maximum_roll_pitch_rad: float = 0.60
    maximum_planar_velocity_rmse_mps: float = 0.35
    maximum_yaw_rate_rmse_radps: float = 0.60
    maximum_abs_torque_nm: float = TORQUE_LIMIT + 1e-6
    maximum_torque_saturation_fraction: float = 0.10
    maximum_dynamics_residual: float = 5.0
    minimum_contact_agreement: float = 0.55


def validate_duration(duration: float, smoke: bool = False) -> None:
    if not math.isfinite(duration) or duration <= 0.0:
        raise ValueError("duration must be finite and positive")
    if duration < 20.0 and not smoke:
        raise ValueError("final benchmark duration must be >=20.0 s; use --smoke for shorter runs")


def gif_metadata(path: Path) -> tuple[int, float]:
    """Return decoded frame count and playback duration from GIF frame timing."""
    with Image.open(path) as image:
        frame_count = image.n_frames
        duration_ms = 0
        for index in range(frame_count):
            image.seek(index)
            duration_ms += int(image.info.get("duration", 0))
    return frame_count, duration_ms / 1000.0


def _finite_float(value: float) -> float | None:
    value = float(value)
    return value if math.isfinite(value) else None


def _max_abs(values: np.ndarray) -> float:
    return float(np.max(np.abs(values))) if values.size else 0.0


def compute_metrics(result: Any, duration: float) -> dict[str, Any]:
    xyz = np.asarray(result.metrics.base_xyz, dtype=float)
    rpy = np.asarray(result.metrics.base_rpy, dtype=float)
    velocity = np.asarray(result.metrics.base_velocity, dtype=float)
    commanded = np.asarray(result.metrics.commanded_velocity, dtype=float)
    torques = np.asarray(result.torques, dtype=float)
    planned = np.asarray(result.metrics.planned_contacts, dtype=bool)
    measured = np.asarray(result.metrics.measured_contacts, dtype=bool)
    velocity_error = velocity - commanded
    velocity_rmse = (
        np.sqrt(np.mean(velocity_error * velocity_error, axis=0))
        if len(velocity_error) else np.zeros(3)
    )
    height_error = xyz[:, 2] - float(commanded.shape[0] and 0.0)
    # commanded does not contain height; caller replaces this intermediate field.
    increments = np.diff(xyz[:, :2], axis=0)
    path_distance = float(np.sum(np.linalg.norm(increments, axis=1))) if len(increments) else 0.0
    net_xy = xyz[-1, :2] - xyz[0, :2] if len(xyz) else np.zeros(2)
    agreement = float(np.mean(planned == measured)) if planned.size else 0.0
    return {
        "simulated_duration_s": float(result.times[-1] + 0.01) if len(result.times) else 0.0,
        "requested_duration_s": float(duration),
        "control_samples": int(len(result.times)),
        "final_base_xyz_m": xyz[-1].tolist() if len(xyz) else [None, None, None],
        "net_displacement_xy_m": net_xy.tolist(),
        "net_distance_m": float(np.linalg.norm(net_xy)),
        "path_distance_m": path_distance,
        "_base_heights": xyz[:, 2].tolist() if len(xyz) else [],
        "_height_error": height_error.tolist(),
        "minimum_base_height_m": float(np.min(xyz[:, 2])) if len(xyz) else None,
        "maximum_base_height_m": float(np.max(xyz[:, 2])) if len(xyz) else None,
        "roll_pitch_rmse_rad": (
            np.sqrt(np.mean(rpy[:, :2] ** 2, axis=0)).tolist()
            if len(rpy) else [0.0, 0.0]
        ),
        "maximum_abs_roll_pitch_rad": _max_abs(rpy[:, :2]),
        "velocity_tracking_rmse": {
            "vx_mps": float(velocity_rmse[0]),
            "vy_mps": float(velocity_rmse[1]),
            "yaw_rate_radps": float(velocity_rmse[2]),
            "planar_mps": float(np.linalg.norm(velocity_rmse[:2])),
        },
        "maximum_abs_torque_nm": _max_abs(torques),
        "torque_saturation_fraction": float(result.torque_saturation_fraction),
        "maximum_dynamics_residual": float(result.max_dynamics_residual),
        "planned_vs_measured_contact_agreement": agreement,
        "fallen": bool(result.fallen),
        "fall_time_s": _finite_float(result.fall_time) if result.fall_time is not None else None,
    }


def assess_metrics(
    metrics: dict[str, Any],
    thresholds: Thresholds,
    require_gif: bool = True,
) -> tuple[bool, list[str]]:
    checks = (
        ("simulation shorter than required", metrics["simulated_duration_s"] + 1e-9
         < thresholds.minimum_simulated_duration_s),
        ("fall detected", metrics["fallen"] and not thresholds.fall_allowed),
        ("base height below threshold", metrics["minimum_base_height_m"] is None
         or metrics["minimum_base_height_m"] < thresholds.minimum_base_height_m),
        ("height RMSE exceeds threshold",
         metrics["height_error_rmse_m"] > thresholds.maximum_height_rmse_m),
        ("roll/pitch exceeds threshold",
         metrics["maximum_abs_roll_pitch_rad"] > thresholds.maximum_roll_pitch_rad),
        ("planar velocity RMSE exceeds threshold",
         metrics["velocity_tracking_rmse"]["planar_mps"]
         > thresholds.maximum_planar_velocity_rmse_mps),
        ("yaw-rate RMSE exceeds threshold",
         metrics["velocity_tracking_rmse"]["yaw_rate_radps"]
         > thresholds.maximum_yaw_rate_rmse_radps),
        ("torque exceeds physical limit",
         metrics["maximum_abs_torque_nm"] > thresholds.maximum_abs_torque_nm),
        ("torque saturation fraction exceeds threshold",
         metrics["torque_saturation_fraction"]
         > thresholds.maximum_torque_saturation_fraction),
        ("dynamics residual exceeds threshold",
         metrics["maximum_dynamics_residual"] > thresholds.maximum_dynamics_residual),
        ("contact agreement below threshold",
         metrics["planned_vs_measured_contact_agreement"]
         < thresholds.minimum_contact_agreement),
    )
    reasons = [reason for reason, failed in checks if failed]
    if require_gif:
        if metrics.get("gif_frame_count", 0) <= 0:
            reasons.append("GIF is missing")
        if (
            metrics.get("gif_playback_duration_s", 0.0) + 1e-9
            < thresholds.minimum_gif_playback_duration_s
        ):
            reasons.append("GIF playback shorter than required")
    return not reasons, reasons


def _font(size: int = 15) -> ImageFont.ImageFont:
    try:
        return ImageFont.truetype(
            "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", size
        )
    except OSError:
        return ImageFont.load_default()


def _overlay(frame: np.ndarray, lines: Sequence[str], fallen: bool) -> Image.Image:
    image = Image.fromarray(frame).convert("RGBA")
    layer = Image.new("RGBA", image.size, (0, 0, 0, 0))
    draw = ImageDraw.Draw(layer)
    font = _font()
    y = 7
    for line_index, line in enumerate(lines):
        box = draw.textbbox((0, 0), line, font=font)
        text_width = box[2] - box[0]
        text_height = box[3] - box[1]
        color = (160, 20, 20, 205) if fallen and line_index == len(lines) - 1 else (0, 0, 0, 170)
        draw.rectangle((5, y - 2, text_width + 15, y + text_height + 4), fill=color)
        draw.text((10, y), line, fill=(255, 255, 255, 255), font=font)
        y += text_height + 7
    return Image.alpha_composite(image, layer).convert("RGB")


def _render_gif(
    adapter: A1HeadlessAdapter,
    result: Any,
    scenario: BenchmarkScenario,
    duration: float,
    fps: int,
    width: int,
    height: int,
    path: Path,
) -> tuple[int, float]:
    if fps <= 0 or width < 64 or height < 64:
        raise ValueError("fps must be positive and render dimensions must be >=64")
    # GIF timing is quantized to 10 ms. Ceiling ensures playback never becomes
    # shorter than simulated time, including non-divisor frame rates.
    frame_ms = max(10, int(math.ceil((1000.0 / fps) / 10.0) * 10))
    frame_count = max(1, int(math.ceil(duration * 1000.0 / frame_ms)))
    renderer = mujoco.Renderer(adapter.model, height=height, width=width)
    camera = mujoco.MjvCamera()
    camera.type = mujoco.mjtCamera.mjCAMERA_FREE
    camera.distance, camera.azimuth, camera.elevation = 1.35, 135, -22
    replay = mujoco.MjData(adapter.model)
    qposes = np.asarray(result.qpos)
    finite_rows = np.all(np.isfinite(qposes), axis=1)
    frames: list[Image.Image] = []
    try:
        last_finite = qposes[0].copy()
        for frame_index in range(frame_count):
            sim_time = min(frame_index * frame_ms / 1000.0, duration)
            state_index = min(int(round(sim_time / adapter.control_dt)), len(qposes) - 1)
            if finite_rows[state_index]:
                last_finite = qposes[state_index]
            replay.qpos[:] = last_finite
            replay.qvel[:] = 0.0
            mujoco.mj_forward(adapter.model, replay)
            camera.lookat[:] = replay.qpos[:3]
            renderer.update_scene(replay, camera=camera)
            fallen_now = result.fallen and result.fall_time is not None and sim_time >= result.fall_time
            status = f"FALL at {result.fall_time:.2f} s" if fallen_now else "RUNNING"
            displacement = replay.qpos[:2] - qposes[0, :2]
            lines = (
                f"{scenario.name} [{scenario.difficulty}]",
                f"t {sim_time:05.1f} s | {scenario.gait} | "
                f"cmd ({scenario.vx:+.2f}, {scenario.vy:+.2f}, {scenario.yaw_rate:+.2f})",
                f"displacement ({displacement[0]:+.2f}, {displacement[1]:+.2f}) m | "
                f"height {replay.qpos[2]:.3f} m",
                status,
            )
            frames.append(_overlay(renderer.render().copy(), lines, fallen_now))
    finally:
        renderer.close()
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.stem}.tmp.gif")
    paletted = [frame.convert("P", palette=Image.Palette.ADAPTIVE, colors=128) for frame in frames]
    paletted[0].save(
        temporary,
        save_all=True,
        append_images=paletted[1:],
        duration=frame_ms,
        loop=0,
        optimize=False,
        disposal=2,
    )
    os.replace(temporary, path)
    return gif_metadata(path)


def _atomic_json(path: Path, value: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.tmp")
    temporary.write_text(
        json.dumps(value, indent=2, sort_keys=True, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    os.replace(temporary, path)


def _scenario_paths(output_root: Path, scenario: BenchmarkScenario) -> tuple[Path, Path]:
    return output_root / "partial" / f"{scenario.name}.json", output_root / "gifs" / f"{scenario.name}.gif"


def _cache_matches(
    record: dict[str, Any],
    scenario: BenchmarkScenario,
    duration: float,
    render: bool,
) -> bool:
    metrics = record.get("metrics", {})
    serialized_config = json.loads(json.dumps(asdict(scenario)))
    return (
        record.get("config") == serialized_config
        and metrics.get("requested_duration_s") == duration
        and metrics.get("simulated_duration_s", 0.0) + 1e-9 >= duration
        and (not render or metrics.get("gif_playback_duration_s", 0.0) + 1e-9 >= duration)
    )


def run_scenario(
    scenario: BenchmarkScenario,
    duration: float,
    fps: int,
    width: int,
    height: int,
    output_root: Path,
    render: bool = True,
    overwrite: bool = False,
) -> dict[str, Any]:
    metadata_path, gif_path = _scenario_paths(output_root, scenario)
    if not overwrite and metadata_path.is_file():
        cached = json.loads(metadata_path.read_text(encoding="utf-8"))
        if _cache_matches(cached, scenario, duration, render) and (
            not render or gif_path.is_file()
        ):
            return cached

    adapter = A1HeadlessAdapter(
        gait=scenario.gait,
        seed=scenario.seed,
        command=MotionCommand(
            scenario.vx, scenario.vy, scenario.yaw_rate, scenario.base_height
        ),
        scenario=ScenarioConfig(
            friction=scenario.friction,
            payload_mass=scenario.payload_mass,
            push_force=scenario.push_force,
            push_start=scenario.push_start,
            push_duration=scenario.push_duration,
        ),
    )
    started = time.perf_counter()
    # Simulation is deliberately uninterrupted after result.fallen becomes true.
    result = adapter.run(duration=duration, render=False)
    simulation_wall_s = time.perf_counter() - started
    metrics = compute_metrics(result, duration)
    metrics["simulated_duration_s"] = float(adapter.data.time)
    heights = np.asarray(metrics.pop("_base_heights"), dtype=float)
    height_errors = heights - scenario.base_height
    metrics.pop("_height_error")
    metrics["height_error_rmse_m"] = (
        float(np.sqrt(np.mean(height_errors * height_errors))) if len(height_errors) else None
    )
    metrics["maximum_abs_height_error_m"] = _max_abs(height_errors)
    metrics["gif_path"] = str(gif_path.relative_to(output_root)) if render else None
    metrics["gif_frame_count"] = 0
    metrics["gif_playback_duration_s"] = 0.0
    if render:
        frame_count, playback_s = _render_gif(
            adapter, result, scenario, duration, fps, width, height, gif_path
        )
        metrics["gif_frame_count"] = frame_count
        metrics["gif_playback_duration_s"] = playback_s
    metrics["simulation_wall_time_s"] = simulation_wall_s
    metrics["realtime_factor"] = (
        metrics["simulated_duration_s"] / simulation_wall_s if simulation_wall_s else None
    )
    thresholds = Thresholds(duration, duration)
    passed, reasons = assess_metrics(metrics, thresholds, require_gif=render)
    metrics["passed"] = passed
    metrics["failure_reasons"] = reasons
    record = {
        "schema_version": 1,
        "config": asdict(scenario),
        "upstream_commit": UPSTREAM_COMMIT,
        "adapter_implementation_revision": IMPLEMENTATION_REVISION,
        "implementation_boundary": IMPLEMENTATION_BOUNDARY,
        "thresholds": asdict(thresholds),
        "threshold_definitions": THRESHOLD_DEFINITIONS,
        "render": {
            "enabled": render,
            "fps_requested": fps,
            "width": width,
            "height": height,
            "camera": "free camera following A1 base; fixed world floor reference",
            "overlay": "scenario, difficulty, time, command, displacement, height, fall status",
        },
        "metrics": metrics,
    }
    _atomic_json(metadata_path, record)
    return record


def _csv_row(record: dict[str, Any]) -> dict[str, Any]:
    config, metrics = record["config"], record["metrics"]
    return {
        "name": config["name"],
        "difficulty": config["difficulty"],
        "gait": config["gait"],
        "seed": config["seed"],
        "simulated_duration_s": metrics["simulated_duration_s"],
        "gif_playback_duration_s": metrics["gif_playback_duration_s"],
        "gif_frame_count": metrics["gif_frame_count"],
        "net_distance_m": metrics["net_distance_m"],
        "path_distance_m": metrics["path_distance_m"],
        "fallen": metrics["fallen"],
        "fall_time_s": metrics["fall_time_s"],
        "height_error_rmse_m": metrics["height_error_rmse_m"],
        "maximum_abs_roll_pitch_rad": metrics["maximum_abs_roll_pitch_rad"],
        "planar_velocity_rmse_mps": metrics["velocity_tracking_rmse"]["planar_mps"],
        "maximum_abs_torque_nm": metrics["maximum_abs_torque_nm"],
        "torque_saturation_fraction": metrics["torque_saturation_fraction"],
        "maximum_dynamics_residual": metrics["maximum_dynamics_residual"],
        "contact_agreement": metrics["planned_vs_measured_contact_agreement"],
        "passed": metrics["passed"],
        "failure_reasons": "; ".join(metrics["failure_reasons"]),
        "gif_path": metrics["gif_path"],
    }


def write_aggregates(output_root: Path, records: Sequence[dict[str, Any]]) -> None:
    ordered = sorted(records, key=lambda item: item["config"]["name"])
    aggregate = {
        "schema_version": 1,
        "benchmark": "legged_control MuJoCo A1 adapter (not Quadruped-PyMPC)",
        "upstream_commit": UPSTREAM_COMMIT,
        "adapter_implementation_revision": IMPLEMENTATION_REVISION,
        "implementation_boundary": IMPLEMENTATION_BOUNDARY,
        "scenario_count": len(ordered),
        "passed_count": sum(record["metrics"]["passed"] for record in ordered),
        "failed_count": sum(not record["metrics"]["passed"] for record in ordered),
        "results": ordered,
    }
    _atomic_json(output_root / "scenario_results.json", aggregate)
    rows = [_csv_row(record) for record in ordered]
    csv_path = output_root / "scenario_results.csv"
    csv_path.parent.mkdir(parents=True, exist_ok=True)
    temporary = csv_path.with_name(f".{csv_path.name}.tmp")
    with temporary.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]) if rows else ["name"])
        writer.writeheader()
        writer.writerows(rows)
    os.replace(temporary, csv_path)
    _write_summary_png(output_root / "summary.png", ordered)


def _write_summary_png(path: Path, records: Sequence[dict[str, Any]]) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    levels = ("easy", "normal", "hard")
    passed = [
        sum(r["config"]["difficulty"] == level and r["metrics"]["passed"] for r in records)
        for level in levels
    ]
    failed = [
        sum(r["config"]["difficulty"] == level and not r["metrics"]["passed"] for r in records)
        for level in levels
    ]
    figure, axes = plt.subplots(1, 2, figsize=(10, 4.5))
    axes[0].bar(levels, passed, label="pass", color="#2a9d5b")
    axes[0].bar(levels, failed, bottom=passed, label="fail", color="#d1495b")
    axes[0].set_ylabel("scenario count")
    axes[0].set_title("Physically thresholded result")
    axes[0].legend()
    names = [r["config"]["name"].split("_", 1)[0] for r in records]
    rmse = [r["metrics"]["velocity_tracking_rmse"]["planar_mps"] for r in records]
    colors = ["#2a9d5b" if r["metrics"]["passed"] else "#d1495b" for r in records]
    axes[1].bar(names, rmse, color=colors)
    axes[1].axhline(
        Thresholds(20.0, 20.0).maximum_planar_velocity_rmse_mps,
        color="black", linestyle="--", linewidth=1, label="threshold",
    )
    axes[1].tick_params(axis="x", rotation=90, labelsize=7)
    axes[1].set_ylabel("planar velocity RMSE [m/s]")
    axes[1].set_title("Tracking error")
    axes[1].legend()
    figure.suptitle("legged_control A1 MuJoCo benchmark (not Quadruped-PyMPC)")
    figure.tight_layout()
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.stem}.tmp.png")
    figure.savefig(temporary, dpi=150)
    plt.close(figure)
    os.replace(temporary, path)


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    selection = parser.add_mutually_exclusive_group(required=True)
    selection.add_argument("--all", action="store_true", help="run all 30 scenarios")
    selection.add_argument("--scenario", choices=tuple(SCENARIO_BY_NAME))
    parser.add_argument("--duration", type=float, default=20.0)
    parser.add_argument("--smoke", action="store_true", help="allow duration below 20 s")
    parser.add_argument("--no-render", action="store_true", help="skip GIF rendering")
    parser.add_argument("--fps", type=int, default=10)
    parser.add_argument("--width", type=int, default=480)
    parser.add_argument("--height", type=int, default=360)
    parser.add_argument("--output-root", type=Path, default=DEFAULT_OUTPUT_ROOT)
    parser.add_argument(
        "--overwrite", action="store_true",
        help="replace matching per-scenario metadata and GIFs; default resumes valid results",
    )
    args = parser.parse_args(argv)
    try:
        validate_duration(args.duration, args.smoke)
    except ValueError as exc:
        parser.error(str(exc))
    if args.fps <= 0 or args.width < 64 or args.height < 64:
        parser.error("fps must be positive and width/height must be >=64")
    return args


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv)
    selected = SCENARIOS if args.all else (SCENARIO_BY_NAME[args.scenario],)
    records: list[dict[str, Any]] = []
    for index, scenario in enumerate(selected, 1):
        print(f"[{index:02d}/{len(selected):02d}] {scenario.name}: {scenario.description}", flush=True)
        record = run_scenario(
            scenario=scenario,
            duration=args.duration,
            fps=args.fps,
            width=args.width,
            height=args.height,
            output_root=args.output_root.resolve(),
            render=not args.no_render,
            overwrite=args.overwrite,
        )
        records.append(record)
        result = "PASS" if record["metrics"]["passed"] else "FAIL"
        print(f"  {result}: {', '.join(record['metrics']['failure_reasons']) or 'all thresholds met'}")
    write_aggregates(args.output_root.resolve(), records)
    print(json.dumps({
        "output_root": str(args.output_root.resolve()),
        "scenarios": len(records),
        "passed": sum(r["metrics"]["passed"] for r in records),
        "failed": sum(not r["metrics"]["passed"] for r in records),
    }, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
