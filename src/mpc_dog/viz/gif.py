"""Render a plant rollout to a GIF with force arrows. Used by notebooks via import."""

from __future__ import annotations

from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont

from mpc_dog.plant.mujoco_go2 import MujocoGo2
from mpc_dog.viz.gl import ensure_mujoco_gl
from mpc_dog.viz.overlay import overlay_contact_and_net

DEFAULT_WIDTH = 640
DEFAULT_HEIGHT = 480
GIF_FRAME_MS = 80


def _font() -> ImageFont.ImageFont:
    try:
        return ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 16)
    except OSError:
        return ImageFont.load_default()


def _caption(img: np.ndarray, lines: list[str]) -> Image.Image:
    im = Image.fromarray(img).convert("RGBA")
    layer = Image.new("RGBA", im.size, (0, 0, 0, 0))
    draw = ImageDraw.Draw(layer)
    font = _font()
    y = 8
    for line in lines:
        bbox = draw.textbbox((0, 0), line, font=font)
        tw, th = bbox[2] - bbox[0], bbox[3] - bbox[1]
        draw.rectangle([6, y - 2, 14 + tw, y + th + 4], fill=(0, 0, 0, 160))
        draw.text((10, y), line, fill=(255, 255, 255, 255), font=font)
        y += th + 8
    return Image.alpha_composite(im, layer).convert("RGB")


def render_rollout_gif(
    plant: MujocoGo2,
    out_path: Path,
    *,
    n_steps: int = 1500,
    capture_every: int = 30,
    tau: np.ndarray | None = None,
    tau_fn=None,
    command_grf: np.ndarray | None = None,
    title: str = "",
    width: int = DEFAULT_WIDTH,
    height: int = DEFAULT_HEIGHT,
) -> Path:
    """Step torques, overlay actual GRF + net contact + weight, write GIF.

    ``tau_fn(plant) -> (12,)`` is used when given. ``command_grf`` is drawn white.
    """
    ensure_mujoco_gl()
    import mujoco

    from mpc_dog.joint.clip import clip_torque

    out_path = Path(out_path)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    u_const = np.zeros(12, dtype=np.float64) if tau is None else np.asarray(tau, dtype=np.float64)

    renderer = mujoco.Renderer(plant.model, height=height, width=width)
    cam = mujoco.MjvCamera()
    cam.type = mujoco.mjtCamera.mjCAMERA_FREE
    mujoco.mjv_defaultFreeCamera(plant.model, cam)
    cam.lookat[:] = [0.15, 0.0, 0.25]
    cam.distance = 1.8
    cam.azimuth = 90.0
    cam.elevation = -20.0

    frames: list[Image.Image] = []
    try:
        for k in range(n_steps):
            u = tau_fn(plant) if tau_fn is not None else u_const
            u = clip_torque(u, plant.model.actuator_ctrlrange)
            plant.step(u)
            if k % capture_every != 0:
                continue
            feet = plant.feet_pos_world()
            grf = plant.contact_forces_world()
            net = plant.net_contact_force_world()
            weight = plant.gravity_force_world()
            com = plant.com_world()
            renderer.update_scene(plant.data, camera=cam)
            overlay_contact_and_net(
                renderer.scene,
                feet,
                grf,
                com,
                net,
                weight,
                command_forces=command_grf,
            )
            rgb = renderer.render()
            fz = grf[:, 2]
            lines = [
                title or "MuJoCo Go2",
                f"t={plant.data.time:.2f}s  z_base={plant.base_pos()[2]:.3f} m",
                f"actual GRF Fz [N] FL={fz[0]:.0f} FR={fz[1]:.0f} RL={fz[2]:.0f} RR={fz[3]:.0f}",
                f"net contact={net[2]:.0f} N  mg={weight[2]:.0f} N  white=command GRF",
            ]
            frames.append(_caption(rgb, lines))
    finally:
        renderer.close()

    if not frames:
        raise RuntimeError("no frames captured")
    frames[0].save(
        out_path,
        save_all=True,
        append_images=frames[1:],
        duration=GIF_FRAME_MS,
        loop=0,
        optimize=False,
    )
    return out_path
