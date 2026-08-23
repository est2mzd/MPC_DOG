"""Stand on flat ground: EqualShare GRF mapped with -J^T F."""

from __future__ import annotations

from pathlib import Path

from mpc_dog.force.equal_share import equal_share
from mpc_dog.joint.map_jt import map_jt
from mpc_dog.plant.mujoco_go2 import MujocoGo2
from mpc_dog.viz.gif import render_rollout_gif


def run_stand_demo(out_gif: Path) -> Path:
    plant = MujocoGo2(scene="flat", seed=0)
    cmd = equal_share(plant.mass_kg, n_stance=4)
    geoms = plant.foot_geom_ids()

    def tau_fn(p: MujocoGo2):
        return map_jt(p.model, p.data, geoms, cmd)

    return render_rollout_gif(
        plant,
        Path(out_gif),
        n_steps=2000,
        capture_every=40,
        tau_fn=tau_fn,
        command_grf=cmd,
        title="02 stand  EqualShare + map_jt",
    )
