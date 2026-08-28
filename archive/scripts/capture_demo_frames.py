#!/usr/bin/env python3
"""Capture offscreen MuJoCo frames + GIF for workshop assets."""
from __future__ import annotations

import copy
import json
import os
import subprocess
import sys
from dataclasses import dataclass, field
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
PYMPC = ROOT / "external" / "Quadruped-PyMPC"
ASSETS = ROOT / "docs" / "pympc_2day" / "assets"
sys.path.insert(0, str(ROOT / "scripts"))
sys.path.insert(0, str(PYMPC))

from demo_capture_common import GIF_FRAME_MS, GIF_N_FRAMES, GIF_TARGET_DURATION_S, save_gif_and_mp4


@dataclass
class CaptureProfile:
    preset: str
    tag: str
    n_steps: int = 1000
    step_freq: float | None = None
    overlay_lines: list[str] = field(default_factory=list)
    distance: float = 3.2
    elevation: float = -22.0
    azimuth: float = 130.0
    intro_frames: int = 0  # fixed wide shot before follow-cam
    intro_lookat: tuple[float, float, float] = (2.0, 0.0, 0.15)
    intro_distance: float = 7.0
    intro_elevation: float = -28.0
    intro_azimuth: float = 110.0
    capture_stride: int = 15  # sim steps between PNG saves
    vel_lin_scale: float = 1.0  # multiply ref forward speed for demo capture
    min_final_x_m: float = 0.0  # warn/fail if robot does not reach terrain


PROFILES: list[CaptureProfile] = [
    CaptureProfile(
        preset="session01_flat_smoke",
        tag="s01_flat",
        n_steps=2500,
        step_freq=1.4,
        vel_lin_scale=1.4,
        overlay_lines=["Session 1 | scene=flat", "foothold OFF | step_freq=1.4 Hz"],
        distance=3.0,
        elevation=-20.0,
        azimuth=135.0,
    ),
    CaptureProfile(
        preset="session02_flat_tune",
        tag="s02_tune",
        n_steps=2500,
        step_freq=1.75,  # faster trot — visually distinct from S1
        vel_lin_scale=1.4,
        overlay_lines=["Session 2 | scene=flat", "foothold OFF | step_freq=1.75 Hz (fast trot)"],
        distance=3.0,
        elevation=-18.0,
        azimuth=120.0,
    ),
    CaptureProfile(
        preset="session03_rough_boxes",
        tag="s03_boxes",
        n_steps=10000,
        vel_lin_scale=2.4,
        min_final_x_m=4.5,
        capture_stride=12,
        overlay_lines=["Session 3a | scene=random_boxes", "foothold ON | discrete box obstacles"],
        distance=4.5,
        elevation=-32.0,
        azimuth=95.0,
        intro_frames=4,
        intro_lookat=(2.5, -1.5, 0.2),
        intro_distance=9.0,
        intro_elevation=-35.0,
        intro_azimuth=115.0,
    ),
    CaptureProfile(
        preset="session03_rough_perlin",
        tag="s03_perlin",
        n_steps=10000,
        step_freq=1.15,
        vel_lin_scale=2.4,
        min_final_x_m=4.5,
        capture_stride=12,
        overlay_lines=["Session 3b | scene=perlin", "foothold ON | continuous height field"],
        distance=5.5,
        elevation=-38.0,
        azimuth=88.0,
        intro_frames=4,
        intro_lookat=(1.5, 0.0, 0.25),
        intro_distance=10.0,
        intro_elevation=-40.0,
        intro_azimuth=100.0,
    ),
]


def _try_mujoco_gl() -> str:
    for backend in ("egl", "osmesa", "glfw"):
        os.environ["MUJOCO_GL"] = backend
        try:
            import mujoco  # noqa: F401

            return backend
        except Exception as exc:
            print(f"MUJOCO_GL={backend} failed: {exc}")
    raise RuntimeError("No working MUJOCO_GL backend")


def _draw_overlay(img, lines: list[str]):
    from PIL import Image, ImageDraw, ImageFont

    im = Image.fromarray(img).convert("RGBA")
    overlay = Image.new("RGBA", im.size, (0, 0, 0, 0))
    draw = ImageDraw.Draw(overlay)
    try:
        font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", 18)
    except OSError:
        font = ImageFont.load_default()
    y = 8
    for line in lines:
        bbox = draw.textbbox((0, 0), line, font=font)
        tw, th = bbox[2] - bbox[0], bbox[3] - bbox[1]
        draw.rectangle([6, y - 2, 14 + tw, y + th + 4], fill=(0, 0, 0, 160))
        draw.text((10, y), line, fill=(255, 255, 255, 255), font=font)
        y += th + 8
    return Image.alpha_composite(im, overlay).convert("RGB")


def _pick_frames(saved: list[Path], n: int = 50) -> list[Path]:
    """Deprecated — use demo_capture_common.pick_frames."""
    from demo_capture_common import pick_frames

    return pick_frames(saved, n=n)


def capture(profile: CaptureProfile) -> Path:
    subprocess.run(
        [sys.executable, str(ROOT / "scripts" / "apply_pympc_preset.py"), profile.preset],
        check=True,
    )
    backend = _try_mujoco_gl()
    print(f"Using MUJOCO_GL={backend} for {profile.tag}")

    import importlib

    import mujoco
    import numpy as np
    from gym_quadruped.quadruped_env import QuadrupedEnv
    from gym_quadruped.utils.quadruped_utils import LegsAttr
    from PIL import Image
    import quadruped_pympc.config as cfg
    from quadruped_pympc.quadruped_pympc_wrapper import QuadrupedPyMPC_Wrapper

    importlib.reload(cfg)

    out = ASSETS / f"frames_{profile.tag}"
    out.mkdir(parents=True, exist_ok=True)
    for p in out.glob("*.png"):
        p.unlink()

    if profile.step_freq is not None:
        gait = cfg.simulation_params["gait"]
        cfg.simulation_params["gait_params"][gait]["step_freq"] = profile.step_freq

    sim_dt = cfg.simulation_params["dt"]
    hip = cfg.hip_height
    scene = cfg.simulation_params["scene"]
    print(f"  scene={scene} steps={profile.n_steps} step_freq={profile.step_freq}")

    env = QuadrupedEnv(
        robot=cfg.robot,
        scene=scene,
        sim_dt=sim_dt,
        ref_base_lin_vel=np.array([0.5, 0.8]) * hip * profile.vel_lin_scale,
        ref_base_ang_vel=(-0.2, 0.2),
        ground_friction_coeff=(0.5, 1.0),
        base_vel_command_type="forward",
        state_obs_names=(),
    )
    env.reset(random=False)
    renderer = mujoco.Renderer(env.mjModel, height=480, width=640)
    wrapper = QuadrupedPyMPC_Wrapper(
        initial_feet_pos=env.feet_pos,
        legs_order=("FL", "FR", "RL", "RR"),
        feet_geom_id=env._feet_geom_id,
    )
    tau = LegsAttr(FL=np.zeros((3, 1)), FR=np.zeros((3, 1)), RL=np.zeros((3, 1)), RR=np.zeros((3, 1)))

    saved: list[Path] = []
    cam = mujoco.MjvCamera()
    cam.type = mujoco.mjtCamera.mjCAMERA_FREE

    def render_frame(i: int, *, intro: bool) -> None:
        if intro:
            cam.lookat[:] = profile.intro_lookat
            cam.distance = profile.intro_distance
            cam.elevation = profile.intro_elevation
            cam.azimuth = profile.intro_azimuth
        else:
            cam.lookat[:] = env.base_pos
            cam.distance = profile.distance
            cam.elevation = profile.elevation
            cam.azimuth = profile.azimuth
        renderer.update_scene(env.mjData, camera=cam)
        img = _draw_overlay(renderer.render(), profile.overlay_lines)
        path = out / f"frame_{i:04d}.png"
        img.save(path)
        saved.append(path)

    for i in range(profile.n_steps):
        feet_pos = env.feet_pos(frame="world")
        ref_lin, ref_ang = env.target_base_vel()
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
            env.get_base_inertia().flatten(),
            env.mjData.contact,
        )
        action = np.zeros(env.mjModel.nu)
        for leg in ["FL", "FR", "RL", "RR"]:
            action[getattr(env.legs_tau_idx, leg)] = tau[leg].flatten()
        env.step(action=action)

        if i % profile.capture_stride != 0:
            continue
        intro = (i // profile.capture_stride) < profile.intro_frames
        render_frame(i, intro=intro)

    final_x = float(env.base_pos[0])
    env.close()
    if not saved:
        raise RuntimeError(f"No frames captured for {profile.tag}")
    if profile.min_final_x_m > 0 and final_x < profile.min_final_x_m:
        raise RuntimeError(
            f"{profile.tag}: final_x={final_x:.2f}m < min {profile.min_final_x_m}m — "
            "obstacle area not reached; increase n_steps or vel_lin_scale"
        )

    gif_path = ASSETS / f"demo_{profile.tag}.gif"
    png_path = ASSETS / f"demo_{profile.tag}.png"
    meta_path = ASSETS / f"demo_{profile.tag}.meta.json"
    gif_frames, playback_s = save_gif_and_mp4(saved, gif_path)
    print(f"mp4: {gif_path.with_suffix('.mp4')}")

    # PNG + metadata from **last** captured frame (end of run)
    last_frame = saved[-1]
    Image.open(last_frame).convert("RGB").save(png_path)
    meta = {
        "tag": profile.tag,
        "scene": scene,
        "final_x_m": round(final_x, 3),
        "n_saved_frames": len(saved),
        "n_gif_frames": len(gif_frames),
        "n_steps": profile.n_steps,
        "gif_playback_s": round(playback_s, 2),
        "gif_frame_ms": GIF_FRAME_MS,
    }
    meta_path.write_text(json.dumps(meta, indent=2), encoding="utf-8")
    print(
        f"saved {len(saved)} frames ({len(gif_frames)} in GIF, {playback_s:.1f}s playback), "
        f"final x={final_x:.2f} -> {gif_path}"
    )
    return gif_path


def main() -> int:
    import argparse

    parser = argparse.ArgumentParser(description="Capture workshop demo GIFs")
    parser.add_argument("--tag", help="capture single tag e.g. s03_boxes")
    args = parser.parse_args()

    ASSETS.mkdir(parents=True, exist_ok=True)
    profiles = PROFILES
    if args.tag:
        profiles = [p for p in PROFILES if p.tag == args.tag or p.tag == args.tag.removeprefix("demo_")]
        if not profiles:
            raise SystemExit(f"Unknown tag {args.tag!r}")
    for profile in profiles:
        capture(profile)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
