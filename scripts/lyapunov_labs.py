#!/usr/bin/env python3
"""Workshop labs for RAL 2025 Lyapunov stable centroidal MPC (parallel to Sessions 1–4)."""
from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parents[1]
ASSETS = ROOT / "docs" / "pympc_2day" / "assets"
RESULTS_PATH = ASSETS / "lyapunov_lab_results.json"

NOMINAL_PRESET = {
    1: "session01_flat_smoke",
    2: "session02_flat_tune",
    "3a": "session03_rough_boxes",
    "3b": "session03_rough_perlin",
    4: "session04_speed_bumpy_base",
}

LYAPUNOV_PRESET = {
    1: "session01_flat_smoke_lyapunov",
    2: "session02_flat_tune_lyapunov",
    "3a": "session03_rough_boxes_lyapunov",
    "3b": "session03_rough_perlin_lyapunov",
    4: "session04_speed_bumpy_base_lyapunov",
}


@dataclass(frozen=True)
class LyapunovLab:
    id: str
    session: str | int
    title: str
    preset: str
    nominal_preset: str
    scene: str
    seconds: float
    run_fn: str
    kwargs: dict[str, Any] = field(default_factory=dict)


LYAPUNOV_LABS: list[LyapunovLab] = [
    LyapunovLab(
        id="lya_s1_flat",
        session=1,
        title="S1 Lyapunov — flat smoke",
        preset=LYAPUNOV_PRESET[1],
        nominal_preset=NOMINAL_PRESET[1],
        scene="flat",
        seconds=4.0,
        run_fn="flat",
    ),
    LyapunovLab(
        id="lya_s2_flat",
        session=2,
        title="S2 Lyapunov — flat tune",
        preset=LYAPUNOV_PRESET[2],
        nominal_preset=NOMINAL_PRESET[2],
        scene="flat",
        seconds=4.0,
        run_fn="flat",
    ),
    LyapunovLab(
        id="lya_s3a_boxes",
        session="3a",
        title="S3a Lyapunov — random_boxes",
        preset=LYAPUNOV_PRESET["3a"],
        nominal_preset=NOMINAL_PRESET["3a"],
        scene="random_boxes",
        seconds=4.0,
        run_fn="flat",
    ),
    LyapunovLab(
        id="lya_s3b_perlin",
        session="3b",
        title="S3b Lyapunov — perlin",
        preset=LYAPUNOV_PRESET["3b"],
        nominal_preset=NOMINAL_PRESET["3b"],
        scene="perlin",
        seconds=4.0,
        run_fn="flat",
    ),
    LyapunovLab(
        id="lya_s4_bumpy_flat",
        session=4,
        title="S4 Lyapunov — bumpy_flat resilient 10m",
        preset=LYAPUNOV_PRESET[4],
        nominal_preset=NOMINAL_PRESET[4],
        scene="bumpy_flat",
        seconds=0.0,
        run_fn="speed_resilient",
        kwargs={
            "target_speed_kph": 4.0,
            "min_distance_m": 10.0,
            "max_seconds": 120.0,
            "speed_ramp_s": 12.0,
            "mu": 0.42,
            "step_freq": 1.30,
            "duty_factor": 0.75,
            "ref_z_scale": 1.08,
            "max_falls": 12,
        },
    ),
    LyapunovLab(
        id="lya_s4_bumpy_uphill",
        session=4,
        title="S4 Lyapunov — bumpy_uphill resilient 10m",
        preset=LYAPUNOV_PRESET[4],
        nominal_preset=NOMINAL_PRESET[4],
        scene="bumpy_uphill",
        seconds=0.0,
        run_fn="speed_resilient",
        kwargs={
            "target_speed_kph": 3.5,
            "min_distance_m": 10.0,
            "max_seconds": 120.0,
            "speed_ramp_s": 14.0,
            "mu": 0.38,
            "step_freq": 1.15,
            "duty_factor": 0.77,
            "ref_z_scale": 1.08,
            "max_falls": 12,
        },
    ),
]

LAB_BY_ID = {lab.id: lab for lab in LYAPUNOV_LABS}


def _serialize(result: dict) -> dict:
    skip = {"t", "vx", "vz", "x"}
    out = {}
    for k, v in result.items():
        if k in skip:
            continue
        if hasattr(v, "tolist"):
            out[k] = v.tolist()
        elif hasattr(v, "item"):
            out[k] = v.item()
        else:
            out[k] = v
    return out


def run_lab(lab_id: str) -> dict:
    if lab_id not in LAB_BY_ID:
        raise KeyError(f"Unknown lab {lab_id!r}")
    lab = LAB_BY_ID[lab_id]
    import pympc_lab as labmod

    labmod.apply_preset(lab.preset)
    if lab.run_fn == "flat":
        result = labmod.run_flat_sim(seconds=lab.seconds, scene=lab.scene)
    elif lab.run_fn == "speed_resilient":
        result = labmod.run_speed_terrain_sim_resilient(scene=lab.scene, **lab.kwargs)
    else:
        raise ValueError(lab.run_fn)
    return {
        "lab_id": lab.id,
        "title": lab.title,
        "session": lab.session,
        "preset": lab.preset,
        "nominal_preset": lab.nominal_preset,
        "scene": lab.scene,
        "controller": "lyapunov_centroidal_nmpc",
        "paper": "Elobaid RAL 2025",
        "result": _serialize(result),
    }


def save_results(entries: list[dict]) -> Path:
    ASSETS.mkdir(parents=True, exist_ok=True)
    cache = {}
    if RESULTS_PATH.is_file():
        cache = json.loads(RESULTS_PATH.read_text(encoding="utf-8"))
    for e in entries:
        cache[e["lab_id"]] = e
    RESULTS_PATH.write_text(json.dumps(cache, indent=2, ensure_ascii=False), encoding="utf-8")
    return RESULTS_PATH


def load_results() -> dict:
    if not RESULTS_PATH.is_file():
        return {}
    return json.loads(RESULTS_PATH.read_text(encoding="utf-8"))


def compare_table() -> list[dict]:
    data = load_results()
    rows = []
    for lab in LYAPUNOV_LABS:
        if lab.id not in data:
            continue
        r = data[lab.id]["result"]
        rows.append(
            {
                "lab": lab.id,
                "scene": lab.scene,
                "dist_or_vx": round(r.get("distance_m", r.get("mean_vx", 0) * lab.seconds), 3),
                "falls": r.get("falls", "-"),
                "terminated": r.get("terminated"),
                "success": r.get("success", not r.get("terminated", False)),
            }
        )
    return rows
