#!/usr/bin/env python3
"""Capture offscreen MuJoCo frames + GIF for workshop assets."""
from __future__ import annotations

import copy
import os
import subprocess
import sys
from dataclasses import dataclass, field
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
PYMPC = ROOT / "external" / "Quadruped-PyMPC"
ASSETS = ROOT / "docs" / "pympc_2day" / "assets"
sys.path.insert(0, str(PYMPC))


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


PROFILES: list[CaptureProfile] = [
    CaptureProfile(
        preset="session01_flat_smoke",
        tag="s01_flat",
        n_steps=1200,
        step_freq=1.4,
        overlay_lines=["Session 1 | scene=flat", "foothold OFF | step_freq=1.4 Hz"],
        distance=3.0,
        elevation=-20.0,
        azimuth=135.0,
    ),
    CaptureProfile(
        preset="session02_flat_tune",
        tag="s02_tune",
        n_steps=1200,
        step_freq=1.75,  # faster trot — visually distinct from S1
        overlay_lines=["Session 2 | scene=flat", "foothold OFF | step_freq=1.75 Hz (fast trot)"],
        distance=3.0,
        elevation=-18.0,
        azimuth=120.0,
    ),
    CaptureProfile(
        preset="session03_rough_boxes",
        tag="s03_boxes",
        n_steps=4500,  # ~9 s — walk into box field (boxes start at x≈1 m)
        overlay_lines=["Session 3a | scene=random_boxes", "foothold ON | discrete box obstacles"],
        distance=4.5,
        elevation=-32.0,
        azimuth=95.0,
        intro_frames=3,
        intro_lookat=(2.5, -1.5, 0.2),
        intro_distance=9.0,
        intro_elevation=-35.0,
        intro_azimuth=115.0,
    ),
    CaptureProfile(
        preset="session03_rough_perlin",
        tag="s03_perlin",
        n_steps=4500,
        step_freq=1.15,
        overlay_lines=["Session 3b | scene=perlin", "foothold ON | continuous height field"],
        distance=5.5,
        elevation=-38.0,
        azimuth=88.0,
        intro_frames=3,
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
        ref_base_lin_vel=np.array([0.5, 0.8]) * hip,
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

        if i % 15 != 0:
            continue
        intro = i // 15 < profile.intro_frames
        render_frame(i, intro=intro)

    final_x = float(env.base_pos[0])
    env.close()
    if not saved:
        raise RuntimeError(f"No frames captured for {profile.tag}")

    gif_path = ASSETS / f"demo_{profile.tag}.gif"
    png_path = ASSETS / f"demo_{profile.tag}.png"
    frames = [Image.open(p).convert("RGB") for p in saved[:50]]
    frames[0].save(
        gif_path,
        save_all=True,
        append_images=frames[1:],
        duration=120,
        loop=0,
        optimize=True,
    )
    try:
        import imageio.v3 as iio

        mp4_path = ASSETS / f"demo_{profile.tag}.mp4"
        stack = np.stack([np.array(Image.open(p).convert("RGB")) for p in saved[:50]])
        iio.imwrite(mp4_path, stack, fps=8, codec="libx264")
        print(f"mp4: {mp4_path}")
    except Exception as exc:
        print(f"mp4 skip: {exc}")

    frames[-1].save(png_path)
    print(f"saved {len(saved)} frames, final x={final_x:.2f} -> {gif_path}")
    return gif_path


def main() -> int:
    ASSETS.mkdir(parents=True, exist_ok=True)
    for profile in PROFILES:
        capture(profile)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
