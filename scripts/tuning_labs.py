#!/usr/bin/env python3
"""Hands-on MPC tuning labs — reproduce documented failures and successes."""
from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable

ROOT = Path(__file__).resolve().parents[1]
ASSETS = ROOT / "docs" / "pympc_2day" / "assets"
RESULTS_PATH = ASSETS / "tuning_lab_results.json"


@dataclass(frozen=True)
class TuningLab:
    id: str
    title: str
    session: str
    phase: str  # fail | compare | success
    narrative: str
    mpc_lesson: str
    preset: str
    run_fn: str  # flat | speed_no_fall | speed_resilient
    kwargs: dict[str, Any]
    doc_ref: str  # anchor in MPC_TUNING_JOURNEY.md


# --- Lab catalog (order = recommended learning path) ---
TUNING_LABS: list[TuningLab] = [
    TuningLab(
        id="s1_ref_z_fail",
        title="S1 ❌ ref_z 低すぎ → 即転倒",
        session="1",
        phase="fail",
        narrative="平坦・足場opt OFF の最小構成でも、胴体高さ参照が低いと数 step で転倒。",
        mpc_lesson="MPC 以前に ref_z = hip_height × 1.05 以上を確認。",
        preset="session01_flat_smoke",
        run_fn="flat",
        kwargs={"seconds": 3.0, "ref_z_scale": 0.95, "scene": "flat"},
        doc_ref="phase-1-flat-smoke",
    ),
    TuningLab(
        id="s1_ref_z_ok",
        title="S1 ✅ ref_z 正常 → スモーク成功",
        session="1",
        phase="success",
        narrative="同じ平坦 scene で ref_z を標準に戻すと trot が安定。",
        mpc_lesson="デバッグは 1 パラメータずつ。baseline を先に固定。",
        preset="session01_flat_smoke",
        run_fn="flat",
        kwargs={"seconds": 4.0, "ref_z_scale": 1.05, "scene": "flat"},
        doc_ref="phase-1-flat-smoke",
    ),
    TuningLab(
        id="s2_mu_aggressive",
        title="S2 ❌ mu=0.55 積極的 → 不安定",
        session="2",
        phase="fail",
        narrative="μ を上げると摩擦円錐が開き、水平 GRF を取りやすいが転倒リスク↑。",
        mpc_lesson="操舵 MPC のタイヤ μ と同型。不整地前に μ の体感を平坦で。",
        preset="session02_flat_tune",
        run_fn="flat",
        kwargs={"seconds": 4.0, "mu": 0.55, "step_freq": 1.4, "scene": "flat"},
        doc_ref="phase-2-flat-tune",
    ),
    TuningLab(
        id="s2_mu_conservative",
        title="S2 ✅ mu=0.35 保守的 → 安定",
        session="2",
        phase="success",
        narrative="μ↓で水平力を抑え、vx は下がるが姿勢は安定。",
        mpc_lesson="失敗時の第一候補: μ↓ / step_freq↓ / duty↑。",
        preset="session02_flat_tune",
        run_fn="flat",
        kwargs={"seconds": 4.0, "mu": 0.35, "step_freq": 1.4, "scene": "flat"},
        doc_ref="phase-2-flat-tune",
    ),
    TuningLab(
        id="s2_step_freq_fast",
        title="S2 ❌ step_freq=1.6 → 足が追いつかない",
        session="2",
        phase="fail",
        narrative="歩調を上げすぎると MPC 予測ホライゾン内で支持周期が短くなり、追従が崩れる。",
        mpc_lesson="step_freq は「MPC が解ける歩調」に合わせる。",
        preset="session02_flat_tune",
        run_fn="flat",
        kwargs={"seconds": 4.0, "mu": 0.5, "step_freq": 1.6, "scene": "flat"},
        doc_ref="phase-2-flat-tune",
    ),
    TuningLab(
        id="s3_boxes_freq_fail",
        title="S3a ❌ boxes + freq=1.6 → 転倒",
        session="3a",
        phase="fail",
        narrative="不整地で Session 2 の「速い trot」をそのまま使うと失敗。",
        mpc_lesson="地形が変わったら step_freq↓ duty↑ が定石。",
        preset="session03_rough_boxes",
        run_fn="flat",
        kwargs={
            "seconds": 4.0,
            "scene": "random_boxes",
            "step_freq": 1.6,
            "use_foothold_optimization": True,
        },
        doc_ref="phase-3-rough-terrain",
    ),
    TuningLab(
        id="s3_boxes_freq_ok",
        title="S3a ✅ boxes + freq=1.1 duty=0.75",
        session="3a",
        phase="success",
        narrative="保守 gait + 足場 opt ON で箱地形を通過。",
        mpc_lesson="足場 opt ON は「どこに足を置くか」も MPC が計画。",
        preset="session03_rough_boxes",
        run_fn="flat",
        kwargs={
            "seconds": 4.0,
            "scene": "random_boxes",
            "step_freq": 1.1,
            "duty_factor": 0.75,
            "use_foothold_optimization": True,
        },
        doc_ref="phase-3-rough-terrain",
    ),
    TuningLab(
        id="s3_perlin_mu_fail",
        title="S3b ❌ perlin + mu=0.55 → 転倒",
        session="3b",
        phase="fail",
        narrative="連続起伏では積極 μ が特に危険。",
        mpc_lesson="perlin は boxes より μ を下げる（0.45 前後）。",
        preset="session03_rough_perlin",
        run_fn="flat",
        kwargs={"seconds": 4.0, "scene": "perlin", "mu": 0.55, "use_foothold_optimization": True},
        doc_ref="phase-3-rough-terrain",
    ),
    TuningLab(
        id="s4_no_fall_fail",
        title="S4 ❌ bumpy_flat 5kph no-fall → ~4m",
        session="4",
        phase="fail",
        narrative="指令 5 kph + 短い ramp では凸凹で数 m 以内に転倒。",
        mpc_lesson="speed_ramp_s 不足と gait 攻めすぎが複合。",
        preset="session04_speed_bumpy_base",
        run_fn="speed_no_fall",
        kwargs={
            "scene": "bumpy_flat",
            "target_speed_kph": 5.0,
            "min_distance_m": 20.0,
            "max_seconds": 25.0,
            "mu": 0.42,
            "step_freq": 1.35,
            "duty_factor": 0.74,
            "ref_z_scale": 1.08,
            "speed_ramp_s": 12.0,
        },
        doc_ref="phase-4-speed-bumpy",
    ),
    TuningLab(
        id="s4_resilient_flat_win",
        title="S4 ✅ bumpy_flat resilient 20m",
        session="4",
        phase="success",
        narrative="mu↓ freq↓ duty↑ ramp↑ の組み合わせ + resilient で累積 20m。",
        mpc_lesson="no-fall 不可でも指令設計・gait の学習には resilient 評価が有効。",
        preset="session04_speed_bumpy_base",
        run_fn="speed_resilient",
        kwargs={
            "scene": "bumpy_flat",
            "target_speed_kph": 5.0,
            "min_distance_m": 20.0,
            "max_seconds": 120.0,
            "max_falls": 22,
            "mu": 0.42,
            "step_freq": 1.20,
            "duty_factor": 0.76,
            "ref_z_scale": 1.07,
            "speed_ramp_s": 18.0,
        },
        doc_ref="phase-4-speed-bumpy",
    ),
    TuningLab(
        id="s4_resilient_downhill_win",
        title="S4 ✅ bumpy_downhill resilient 20m（最難）",
        session="4",
        phase="success",
        narrative="下り坂は duty=0.82, mu=0.35, ramp=22s まで保守化が必要。",
        mpc_lesson="地形ごとに勝ちパラメータが異なる — YAML を分離保存。",
        preset="session04_speed_bumpy_base",
        run_fn="speed_resilient",
        kwargs={
            "scene": "bumpy_downhill",
            "target_speed_kph": 5.0,
            "min_distance_m": 20.0,
            "max_seconds": 120.0,
            "max_falls": 30,
            "mu": 0.35,
            "step_freq": 1.05,
            "duty_factor": 0.82,
            "ref_z_scale": 1.10,
            "speed_ramp_s": 22.0,
        },
        doc_ref="phase-4-speed-bumpy",
    ),
]

LAB_BY_ID: dict[str, TuningLab] = {lab.id: lab for lab in TUNING_LABS}


def list_labs(*, session: str | None = None) -> list[TuningLab]:
    if session is None:
        return list(TUNING_LABS)
    return [lab for lab in TUNING_LABS if lab.session == session]


def _serialize_result(result: dict) -> dict:
    import numpy as np

    skip = {"t", "vx", "vz", "x"}
    out = {}
    for k, v in result.items():
        if k in skip:
            continue
        if hasattr(v, "tolist"):
            out[k] = v.tolist()
        elif isinstance(v, (np.bool_, np.integer, np.floating)):
            out[k] = v.item()
        else:
            out[k] = v
    return out


def run_lab(lab_id: str) -> dict:
    """Execute one tuning lab; returns metrics + lab metadata."""
    if lab_id not in LAB_BY_ID:
        raise KeyError(f"Unknown lab {lab_id!r}. Available: {list(LAB_BY_ID)}")
    lab = LAB_BY_ID[lab_id]

    import pympc_lab as labmod

    labmod.apply_preset(lab.preset)
    kw = dict(lab.kwargs)
    if lab.run_fn == "flat":
        result = labmod.run_flat_sim(**kw)
    elif lab.run_fn == "speed_no_fall":
        result = labmod.run_speed_terrain_sim(**kw)
    elif lab.run_fn == "speed_resilient":
        result = labmod.run_speed_terrain_sim_resilient(**kw)
    else:
        raise ValueError(lab.run_fn)

    return {
        "lab_id": lab.id,
        "title": lab.title,
        "phase": lab.phase,
        "preset": lab.preset,
        "kwargs": kw,
        "result": _serialize_result(result),
        "metrics": result,
    }


def run_lab_pair(fail_id: str, success_id: str) -> list[tuple[str, dict]]:
    """Run fail→success pair for compare_runs."""
    out = []
    for lid in (fail_id, success_id):
        r = run_lab(lid)
        out.append((LAB_BY_ID[lid].title, r["metrics"]))
    return out


def load_cached_lab_results() -> dict[str, dict]:
    if not RESULTS_PATH.is_file():
        return {}
    return json.loads(RESULTS_PATH.read_text(encoding="utf-8"))


def save_lab_result(entry: dict) -> None:
    ASSETS.mkdir(parents=True, exist_ok=True)
    cache = load_cached_lab_results()
    cache[entry["lab_id"]] = {
        k: entry[k]
        for k in ("lab_id", "title", "phase", "preset", "kwargs", "result")
    }
    RESULTS_PATH.write_text(json.dumps(cache, indent=2, ensure_ascii=False), encoding="utf-8")


def run_all_labs(*, skip_resilient: bool = False) -> dict[str, dict]:
    """Run full lab matrix and write tuning_lab_results.json."""
    cache: dict[str, dict] = {}
    for lab in TUNING_LABS:
        if skip_resilient and lab.run_fn == "speed_resilient":
            continue
        print(f"lab {lab.id} ...", flush=True)
        entry = run_lab(lab.id)
        cache[lab.id] = {k: entry[k] for k in ("lab_id", "title", "phase", "preset", "kwargs", "result")}
    RESULTS_PATH.write_text(json.dumps(cache, indent=2, ensure_ascii=False), encoding="utf-8")
    return cache


def plot_lab_comparison(entries: list[tuple[str, dict]], *, title: str = "Tuning lab comparison"):
    import matplotlib.pyplot as plt

    import pympc_lab as labmod

    return labmod.compare_runs(entries)


def plot_speed_trial_journey():
    """Bar chart of S4 trial log: distance by scene/mode/kph."""
    import matplotlib.pyplot as plt
    import pandas as pd

    log_path = ASSETS / "speed_terrain_trial_log.json"
    if not log_path.is_file():
        raise FileNotFoundError(log_path)
    df = pd.DataFrame(json.loads(log_path.read_text()))
    df["label"] = df["scene"] + " " + df["mode"] + " " + df["kph"].astype(str) + "kph"
    fig, ax = plt.subplots(figsize=(10, 4))
    colors = df["success"].map({True: "#16a34a", False: "#dc2626"})
    ax.barh(df["label"], df["distance_m"], color=colors)
    ax.axvline(20.0, color="k", ls="--", lw=1, label="20 m goal")
    ax.set_xlabel("distance [m]")
    ax.set_title("Session 4 trial journey (green=success)")
    ax.legend()
    fig.tight_layout()
    return fig


def plot_param_study_mu():
    import matplotlib.pyplot as plt
    import pandas as pd

    import pympc_lab as labmod

    rows = labmod.load_param_study()
    df = pd.DataFrame(rows)
    sub = df[df["step_freq"] == 1.4].sort_values("mu")
    fig, ax = plt.subplots(figsize=(6, 3))
    ax.plot(sub["mu"], sub["mean_vx"], "o-")
    ax.set_xlabel("mu")
    ax.set_ylabel("mean vx [m/s]")
    ax.set_title("Param study: mu vs forward speed (flat)")
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    return fig


def main() -> int:
    import argparse

    parser = argparse.ArgumentParser(description="Run MPC tuning labs")
    parser.add_argument("--lab", help="single lab id")
    parser.add_argument("--all", action="store_true", help="run all labs (skip long resilient)")
    parser.add_argument("--include-resilient", action="store_true")
    parser.add_argument("--list", action="store_true")
    args = parser.parse_args()

    if args.list:
        for lab in TUNING_LABS:
            print(f"{lab.id:28} [{lab.phase:7}] {lab.title}")
        return 0
    if args.lab:
        entry = run_lab(args.lab)
        save_lab_result(entry)
        print(json.dumps(entry["result"], indent=2))
        return 0
    if args.all:
        run_all_labs(skip_resilient=not args.include_resilient)
        return 0
    parser.print_help()
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
