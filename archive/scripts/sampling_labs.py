#!/usr/bin/env python3
"""Workshop labs for IROS 2024 sampling MPC (parallel to Sessions 1–4)."""
from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parents[1]
ASSETS = ROOT / "docs" / "pympc_2day" / "assets"
RESULTS_PATH = ASSETS / "sampling_lab_results.json"

NOMINAL_PRESET = {
    1: "session01_flat_smoke",
    2: "session02_flat_tune",
    "3a": "session03_rough_boxes",
    "3b": "session03_rough_perlin",
    4: "session04_speed_bumpy_base",
}

SAMPLING_PRESET = {
    1: "session01_flat_smoke_sampling",
    2: "session02_flat_tune_sampling",
    "3a": "session03_rough_boxes_sampling",
    "3b": "session03_rough_perlin_sampling",
    4: "session04_speed_bumpy_base_sampling",
}


@dataclass(frozen=True)
class SamplingLab:
    id: str
    session: str | int
    title: str
    preset: str
    nominal_preset: str
    scene: str
    seconds: float
    run_fn: str  # flat | speed_resilient
    kwargs: dict[str, Any] = field(default_factory=dict)


SAMPLING_LABS: list[SamplingLab] = [
    SamplingLab(
        id="smp_s1_flat",
        session=1,
        title="S1 sampling — flat smoke",
        preset=SAMPLING_PRESET[1],
        nominal_preset=NOMINAL_PRESET[1],
        scene="flat",
        seconds=4.0,
        run_fn="flat",
    ),
    SamplingLab(
        id="smp_s2_flat",
        session=2,
        title="S2 sampling — flat tune baseline",
        preset=SAMPLING_PRESET[2],
        nominal_preset=NOMINAL_PRESET[2],
        scene="flat",
        seconds=4.0,
        run_fn="flat",
    ),
    SamplingLab(
        id="smp_s3a_boxes",
        session="3a",
        title="S3a sampling — random_boxes",
        preset=SAMPLING_PRESET["3a"],
        nominal_preset=NOMINAL_PRESET["3a"],
        scene="random_boxes",
        seconds=4.0,
        run_fn="flat",
    ),
    SamplingLab(
        id="smp_s3b_perlin",
        session="3b",
        title="S3b sampling — perlin",
        preset=SAMPLING_PRESET["3b"],
        nominal_preset=NOMINAL_PRESET["3b"],
        scene="perlin",
        seconds=4.0,
        run_fn="flat",
    ),
    SamplingLab(
        id="smp_s4_bumpy_flat",
        session=4,
        title="S4 sampling — bumpy_flat resilient 12m",
        preset=SAMPLING_PRESET[4],
        nominal_preset=NOMINAL_PRESET[4],
        scene="bumpy_flat",
        seconds=0.0,
        run_fn="speed_resilient",
        kwargs={
            "target_speed_kph": 3.0,
            "min_distance_m": 8.0,
            "max_seconds": 120.0,
            "speed_ramp_s": 14.0,
            "mu": 0.45,
            "step_freq": 1.1,
            "duty_factor": 0.78,
            "ref_z_scale": 1.08,
            "max_falls": 15,
        },
    ),
    SamplingLab(
        id="smp_s4_bumpy_uphill",
        session=4,
        title="S4 sampling — bumpy_uphill resilient 10m",
        preset=SAMPLING_PRESET[4],
        nominal_preset=NOMINAL_PRESET[4],
        scene="bumpy_uphill",
        seconds=0.0,
        run_fn="speed_resilient",
        kwargs={
            "target_speed_kph": 2.5,
            "min_distance_m": 8.0,
            "max_seconds": 120.0,
            "speed_ramp_s": 16.0,
            "mu": 0.40,
            "step_freq": 1.05,
            "duty_factor": 0.80,
            "ref_z_scale": 1.08,
            "max_falls": 15,
        },
    ),
]

LAB_BY_ID = {lab.id: lab for lab in SAMPLING_LABS}


def list_labs() -> list[SamplingLab]:
    return list(SAMPLING_LABS)


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


def run_lab(lab_id: str, *, compare_nominal: bool = False) -> dict:
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

    entry = {
        "lab_id": lab.id,
        "title": lab.title,
        "session": lab.session,
        "preset": lab.preset,
        "nominal_preset": lab.nominal_preset,
        "scene": lab.scene,
        "controller": "sampling_mppi",
        "paper": "Turrisi IROS 2024",
        "result": _serialize(result),
        "metrics": result,
    }
    if compare_nominal:
        labmod.apply_preset(lab.nominal_preset)
        if lab.run_fn == "flat":
            nominal = labmod.run_flat_sim(seconds=lab.seconds, scene=lab.scene)
        else:
            nominal = labmod.run_speed_terrain_sim_resilient(scene=lab.scene, **lab.kwargs)
        entry["nominal_result"] = _serialize(nominal)
    return entry


def save_results(entries: list[dict]) -> Path:
    ASSETS.mkdir(parents=True, exist_ok=True)
    cache = {}
    if RESULTS_PATH.is_file():
        cache = json.loads(RESULTS_PATH.read_text(encoding="utf-8"))
    for e in entries:
        cache[e["lab_id"]] = {k: e[k] for k in e if k != "metrics"}
    RESULTS_PATH.write_text(json.dumps(cache, indent=2, ensure_ascii=False), encoding="utf-8")
    return RESULTS_PATH


def load_results() -> dict:
    if not RESULTS_PATH.is_file():
        return {}
    return json.loads(RESULTS_PATH.read_text(encoding="utf-8"))


def compare_table() -> list[dict]:
    data = load_results()
    rows = []
    for lab in SAMPLING_LABS:
        if lab.id not in data:
            continue
        e = data[lab.id]
        r = e.get("result", {})
        nr = e.get("nominal_result", {})
        rows.append(
            {
                "lab": lab.id,
                "scene": lab.scene,
                "sampling_dist_m": round(r.get("distance_m", r.get("mean_vx", 0) * lab.seconds), 2)
                if lab.run_fn == "flat"
                else round(r.get("distance_m", 0), 2),
                "sampling_falls": r.get("falls", "-"),
                "sampling_terminated": r.get("terminated"),
                "nominal_dist_m": round(nr.get("distance_m", nr.get("mean_vx", 0) * lab.seconds), 2)
                if nr and lab.run_fn == "flat"
                else round(nr.get("distance_m", 0), 2) if nr else None,
            }
        )
    return rows
