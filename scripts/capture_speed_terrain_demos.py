#!/usr/bin/env python3
"""Capture Session 4 GIFs: 5 kph target on bumpy flat / uphill / downhill (resilient 20 m)."""
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
RESULTS = ASSETS / "speed_terrain_results.json"
sys.path.insert(0, str(ROOT / "scripts"))
sys.path.insert(0, str(PYMPC))


@dataclass
class SpeedCaptureProfile:
    scene: str
    tag: str
    mu: float
    step_freq: float
    duty_factor: float
    ref_z_scale: float
    speed_ramp_s: float
    target_speed_kph: float = 5.0
    min_distance_m: float = 20.0
    max_seconds: float = 180.0
    max_falls: int = 30
    distance_milestones: tuple[float, ...] = (0, 2, 4, 6, 8, 10, 12, 14, 16, 18, 20)
    max_gif_frames: int = 50
    distance: float = 6.0
    elevation: float = -38.0
    azimuth: float = 92.0
    overlay_lines: list[str] = field(default_factory=list)


def _load_profiles() -> list[SpeedCaptureProfile]:
    data = json.loads(RESULTS.read_text(encoding="utf-8"))
    labels = {
        "bumpy_flat": "Bumpy flat",
        "bumpy_uphill": "Bumpy uphill",
        "bumpy_downhill": "Bumpy downhill",
    }
    profiles: list[SpeedCaptureProfile] = []
    for scene, spec in data.items():
        mf = int(spec.get("max_falls", spec.get("result", {}).get("falls", 25)) + 8)
        profiles.append(
            SpeedCaptureProfile(
                scene=scene,
                tag=f"s04_{scene.replace('bumpy_', '')}",
                mu=spec["mu"],
                step_freq=spec["step_freq"],
                duty_factor=spec["duty_factor"],
                ref_z_scale=spec["ref_z_scale"],
                speed_ramp_s=spec["speed_ramp_s"],
                target_speed_kph=float(spec.get("target_speed_kph", spec.get("target", 5.0))),
                max_falls=mf,
                overlay_lines=[
                    f"Session 4 | {labels.get(scene, scene)}",
                    f"target 5 kph | resilient 20 m | foothold ON",
                ],
            )
        )
    return profiles


def _try_mujoco_gl() -> str:
    for backend in ("egl", "osmesa", "glfw"):
        os.environ["MUJOCO_GL"] = backend
        try:
            import mujoco  # noqa: F401

            return backend
        except Exception as exc:
            print(f"MUJOCO_GL={backend} failed: {exc}")
    raise RuntimeError("No working MUJOCO_GL backend")


def _pick_frames(saved: list[Path], n: int = 50) -> list[Path]:
    if len(saved) <= n:
        return saved
    import numpy as np

    idx = np.linspace(0, len(saved) - 1, n, dtype=int)
    return [saved[i] for i in idx]


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


def capture(profile: SpeedCaptureProfile) -> Path:
    subprocess.run(
        [sys.executable, str(ROOT / "scripts" / "apply_pympc_preset.py"), "session04_speed_bumpy_base"],
        check=True,
    )
    backend = _try_mujoco_gl()
    print(f"Using MUJOCO_GL={backend} for {profile.tag}")

    import importlib

    import mujoco
    import numpy as np
    from PIL import Image
    import quadruped_pympc.config as cfg
    from quadruped_pympc.quadruped_pympc_wrapper import QuadrupedPyMPC_Wrapper

    from workshop_terrain import install_custom_terrains

    install_custom_terrains()
    importlib.reload(cfg)

    from gym_quadruped.quadruped_env import QuadrupedEnv
    from gym_quadruped.utils.quadruped_utils import LegsAttr

    target_mps = profile.target_speed_kph * (1000.0 / 3600.0)
    cfg.mpc_params["mu"] = profile.mu
    gait = cfg.simulation_params["gait"]
    cfg.simulation_params["gait_params"][gait]["step_freq"] = profile.step_freq
    cfg.simulation_params["gait_params"][gait]["duty_factor"] = profile.duty_factor
    cfg.simulation_params["ref_z"] = cfg.hip_height * profile.ref_z_scale

    sim_dt = cfg.simulation_params["dt"]
    out = ASSETS / f"frames_{profile.tag}"
    out.mkdir(parents=True, exist_ok=True)
    for p in out.glob("*.png"):
        p.unlink()

    env = QuadrupedEnv(
        robot=cfg.robot,
        scene=profile.scene,
        sim_dt=sim_dt,
        ref_base_lin_vel=np.array([target_mps / cfg.hip_height, target_mps / cfg.hip_height]) * cfg.hip_height,
        ref_base_ang_vel=(-0.15, 0.15),
        ground_friction_coeff=(0.5, 1.0),
        base_vel_command_type="forward",
        state_obs_names=(),
    )
    env.mjModel.opt.gravity[2] = -cfg.gravity_constant
    env.reset(random=False)

    renderer = mujoco.Renderer(env.mjModel, height=480, width=640)
    wrapper = QuadrupedPyMPC_Wrapper(
        initial_feet_pos=env.feet_pos,
        legs_order=("FL", "FR", "RL", "RR"),
        feet_geom_id=env._feet_geom_id,
    )
    tau = LegsAttr(FL=np.zeros((3, 1)), FR=np.zeros((3, 1)), RL=np.zeros((3, 1)), RR=np.zeros((3, 1)))
    tau_soft = 0.9
    tau_limits = LegsAttr(
        FL=env.mjModel.actuator_ctrlrange[env.legs_tau_idx.FL] * tau_soft,
        FR=env.mjModel.actuator_ctrlrange[env.legs_tau_idx.FR] * tau_soft,
        RL=env.mjModel.actuator_ctrlrange[env.legs_tau_idx.RL] * tau_soft,
        RR=env.mjModel.actuator_ctrlrange[env.legs_tau_idx.RR] * tau_soft,
    )
    legs_order = ["FL", "FR", "RL", "RR"]
    heightmaps = None

    cam = mujoco.MjvCamera()
    cam.type = mujoco.mjtCamera.mjCAMERA_FREE
    saved: list[Path] = []
    cumulative_m = 0.0
    falls = 0
    session_start = float(env.base_pos[0])
    next_milestone = 0
    last_capture_dist = -1.0

    def reset_segment() -> None:
        nonlocal session_start
        env.reset(random=False)
        wrapper.reset(initial_feet_pos=env.feet_pos(frame="world"))
        session_start = float(env.base_pos[0])

    def render_frame(i: int, dist_m: float, *, reason: str) -> None:
        cam.lookat[:] = env.base_pos
        cam.distance = profile.distance
        cam.elevation = profile.elevation
        cam.azimuth = profile.azimuth
        renderer.update_scene(env.mjData, camera=cam)
        lines = profile.overlay_lines + [f"dist {dist_m:.1f} m | falls {falls}"]
        img = _draw_overlay(renderer.render(), lines)
        path = out / f"frame_{len(saved):04d}_{reason}.png"
        img.save(path)
        saved.append(path)

    def maybe_capture(step_i: int, dist_m: float, *, reason: str = "dist") -> None:
        nonlocal next_milestone, last_capture_dist
        while (
            next_milestone < len(profile.distance_milestones)
            and dist_m >= profile.distance_milestones[next_milestone]
        ):
            render_frame(step_i, dist_m, reason=f"m{profile.distance_milestones[next_milestone]:.0f}")
            next_milestone += 1
            last_capture_dist = dist_m
        # Also capture on large fall events (reset storytelling)
        if reason == "fall" and dist_m - last_capture_dist >= 0.5:
            render_frame(step_i, dist_m, reason="fall")
            last_capture_dist = dist_m

    maybe_capture(0, 0.0)

    n_max = int(profile.max_seconds / sim_dt)
    for step_i in range(n_max):
        t_global = step_i * sim_dt
        if profile.speed_ramp_s > 0 and t_global < profile.speed_ramp_s:
            v_cmd = target_mps * (t_global / profile.speed_ramp_s)
        else:
            v_cmd = target_mps
        env._ref_base_lin_vel_H = np.array([v_cmd, 0.0, 0.0])

        feet_pos = env.feet_pos(frame="world")
        base_lin_vel = env.base_lin_vel(frame="world")
        base_ang_vel = env.base_ang_vel(frame="base")
        base_ori_euler_xyz = env.base_ori_euler_xyz
        base_pos = copy.deepcopy(env.base_pos)
        com_pos = copy.deepcopy(env.com)
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
            feet_pos, env.hip_positions(frame="world"), joints_pos, heightmaps, legs_order, sim_dt,
            ref_base_lin_vel, ref_base_ang_vel, env.step_num, qpos, qvel,
            env.feet_jacobians(frame="world", return_rot_jac=False),
            env.feet_jacobians_dot(frame="world", return_rot_jac=False),
            env.feet_vel(frame="world"), env.legs_qfrc_passive, env.legs_qfrc_bias,
            env.legs_mass_matrix, legs_qpos_idx, legs_qvel_idx, tau, inertia, env.mjData.contact,
        )
        for leg in legs_order:
            tau_min, tau_max = tau_limits[leg][:, 0], tau_limits[leg][:, 1]
            tau[leg] = np.clip(tau[leg], tau_min, tau_max)

        action = np.zeros(env.mjModel.nu)
        for leg in legs_order:
            action[getattr(env.legs_tau_idx, leg)] = tau[leg].flatten()
        _, _, term, trunc, _ = env.step(action=action)

        seg_x = float(env.base_pos[0]) - session_start
        dist_m = cumulative_m + max(seg_x, 0.0)

        maybe_capture(step_i, dist_m)

        if term or trunc:
            cumulative_m += max(seg_x, 0.0)
            falls += 1
            maybe_capture(step_i, cumulative_m, reason="fall")
            if falls > profile.max_falls or cumulative_m >= profile.min_distance_m:
                break
            reset_segment()
            continue

        if dist_m >= profile.min_distance_m:
            cumulative_m = dist_m
            render_frame(step_i, dist_m, reason="goal20")
            break

    env.close()
    if not saved:
        raise RuntimeError(f"No frames captured for {profile.tag}")

    gif_path = ASSETS / f"demo_{profile.tag}.gif"
    png_path = ASSETS / f"demo_{profile.tag}.png"
    meta_path = ASSETS / f"demo_{profile.tag}.meta.json"
    gif_frames = _pick_frames(saved, n=profile.max_gif_frames)
    frames = [Image.open(p).convert("RGB") for p in gif_frames]
    frames[0].save(
        gif_path,
        save_all=True,
        append_images=frames[1:],
        duration=130,
        loop=0,
        optimize=True,
    )
    try:
        import imageio.v3 as iio

        mp4_path = ASSETS / f"demo_{profile.tag}.mp4"
        stack = np.stack([np.array(Image.open(p).convert("RGB")) for p in gif_frames])
        iio.imwrite(mp4_path, stack, fps=8, codec="libx264")
        print(f"mp4: {mp4_path}")
    except Exception as exc:
        print(f"mp4 skip: {exc}")

    last_frame = saved[-1]
    Image.open(last_frame).convert("RGB").save(png_path)
    meta = {
        "tag": profile.tag,
        "scene": profile.scene,
        "target_speed_kph": profile.target_speed_kph,
        "final_dist_m": round(cumulative_m, 3),
        "final_x_m": round(cumulative_m, 3),
        "falls": falls,
        "n_saved_frames": len(saved),
        "n_gif_frames": len(gif_frames),
    }
    meta_path.write_text(json.dumps(meta, indent=2), encoding="utf-8")
    print(f"saved {len(saved)} frames ({len(gif_frames)} in GIF), dist={cumulative_m:.2f} m falls={falls} -> {gif_path}")
    return gif_path


def main() -> int:
    ASSETS.mkdir(parents=True, exist_ok=True)
    for profile in _load_profiles():
        capture(profile)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
