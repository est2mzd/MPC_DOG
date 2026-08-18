"""Custom bumpy slope terrains for mpc_dog workshop (monkey-patch gym_quadruped)."""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

# 5 kph in m/s
KPH5_MPS = 5000.0 / 3600.0  # ≈ 1.389 m/s


@dataclass(frozen=True)
class BumpySceneSpec:
    scene: str
    pitch_rad: float
    label: str
    description: str


BUMPY_SCENES: dict[str, BumpySceneSpec] = {
    "bumpy_flat": BumpySceneSpec(
        scene="bumpy_flat",
        pitch_rad=0.0,
        label="Bumpy flat",
        description="Perlin heightfield, no global slope",
    ),
    "bumpy_uphill": BumpySceneSpec(
        scene="bumpy_uphill",
        pitch_rad=0.08,
        label="Bumpy uphill",
        description="Perlin + positive pitch (walk uphill in +x)",
    ),
    "bumpy_downhill": BumpySceneSpec(
        scene="bumpy_downhill",
        pitch_rad=-0.08,
        label="Bumpy downhill",
        description="Perlin + negative pitch (walk downhill in +x)",
    ),
}

_INSTALLED = False


def install_custom_terrains() -> None:
    """Register bumpy_flat / bumpy_uphill / bumpy_downhill with gym_quadruped.generate_terrain."""
    global _INSTALLED
    if _INSTALLED:
        return

    import gym_quadruped.utils.mujoco.terrain as terrain_mod

    orig = terrain_mod.generate_terrain

    def patched_generate_terrain(
        base_scene_env_path,
        procedural_assets_path,
        hip_height: float,
        terrain_name: str = "perlin",
        seed=10,
    ):
        if terrain_name not in BUMPY_SCENES:
            return orig(
                base_scene_env_path,
                procedural_assets_path,
                hip_height,
                terrain_name,
                seed,
            )

        spec = BUMPY_SCENES[terrain_name]
        with terrain_mod.local_seed(seed):
            base = procedural_assets_path / "scene_flat.xml"
            scene_env, terrain_limits = terrain_mod.add_perlin_heightfield(
                base,
                euler_xyz=[0.0, spec.pitch_rad, 0.0],
                size=(hip_height * 100, hip_height * 100),
                max_height=0.9 * hip_height,
                min_height=0.005,
                image_width=128,
                img_height=128,
                smooth=60,
                perlin_octaves=4,
                perlin_lacunarity=3.0,
                output_hfield_image=f"height_field_{terrain_name}",
            )
        return scene_env, terrain_limits

    terrain_mod.generate_terrain = patched_generate_terrain
    try:
        import gym_quadruped.quadruped_env as qe

        qe.generate_terrain = patched_generate_terrain
    except ImportError:
        pass
    _INSTALLED = True


def scene_label(scene: str) -> str:
    if scene in BUMPY_SCENES:
        return BUMPY_SCENES[scene].label
    return scene
