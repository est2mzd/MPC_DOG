#!/usr/bin/env python3
"""Trial-and-error benchmark: 5 kph, 20 m on bumpy flat/uphill/downhill."""
from __future__ import annotations

import json
import sys
from datetime import datetime, timezone
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))

from pympc_lab import run_speed_terrain_sim  # noqa: E402

ASSETS = ROOT / "docs" / "pympc_2day" / "assets"
TRIAL_LOG = ASSETS / "speed_terrain_trial_log.json"
RESULTS = ASSETS / "speed_terrain_results.json"

SCENES = ["bumpy_flat", "bumpy_uphill", "bumpy_downhill"]

# Trial matrix — conservative → aggressive (documented iteration)
TRIALS = [
    {"id": "t01_baseline", "mu": 0.42, "step_freq": 1.35, "duty_factor": 0.74, "ref_z_scale": 1.05},
    {"id": "t02_slower", "mu": 0.40, "step_freq": 1.25, "duty_factor": 0.76, "ref_z_scale": 1.06},
    {"id": "t03_uphill_safe", "mu": 0.38, "step_freq": 1.15, "duty_factor": 0.78, "ref_z_scale": 1.08},
    {"id": "t04_downhill_safe", "mu": 0.38, "step_freq": 1.15, "duty_factor": 0.78, "ref_z_scale": 1.07},
    {"id": "t05_flat_fast", "mu": 0.44, "step_freq": 1.38, "duty_factor": 0.72, "ref_z_scale": 1.05},
    {"id": "t06_ultra_safe", "mu": 0.36, "step_freq": 1.10, "duty_factor": 0.80, "ref_z_scale": 1.08},
    {"id": "t07_mid", "mu": 0.40, "step_freq": 1.28, "duty_factor": 0.75, "ref_z_scale": 1.06},
]


def best_trial_for_scene(scene: str, log: list[dict]) -> dict | None:
    ok = [e for e in log if e["scene"] == scene and e["result"]["success"]]
    if not ok:
        return None
    return max(ok, key=lambda e: (e["result"]["mean_kph"], e["result"]["distance_m"]))


def run_all() -> tuple[list[dict], dict[str, dict]]:
    log: list[dict] = []
    for scene in SCENES:
        for trial in TRIALS:
            # Pick terrain-specific trial subset
            if scene == "bumpy_uphill" and trial["id"] not in (
                "t01_baseline", "t02_slower", "t03_uphill_safe", "t06_ultra_safe", "t07_mid"
            ):
                continue
            if scene == "bumpy_downhill" and trial["id"] not in (
                "t01_baseline", "t02_slower", "t04_downhill_safe", "t06_ultra_safe", "t07_mid"
            ):
                continue
            if scene == "bumpy_flat" and trial["id"] not in (
                "t01_baseline", "t02_slower", "t05_flat_fast", "t06_ultra_safe", "t07_mid"
            ):
                continue

            print(f"\n=== {scene} / {trial['id']} ===")
            result = run_speed_terrain_sim(
                scene=scene,
                target_speed_kph=5.0,
                min_distance_m=20.0,
                max_seconds=25.0,
                mu=trial["mu"],
                step_freq=trial["step_freq"],
                duty_factor=trial["duty_factor"],
                ref_z_scale=trial["ref_z_scale"],
            )
            entry = {
                "timestamp": datetime.now(timezone.utc).isoformat(),
                "scene": scene,
                "trial_id": trial["id"],
                "params": trial,
                "result": {k: v for k, v in result.items() if k not in ("t", "vx", "vz", "x")},
            }
            log.append(entry)
            r = entry["result"]
            print(
                f"  dist={r['distance_m']:.2f}m mean={r['mean_kph']:.2f}kph "
                f"term={r['terminated']} success={r['success']}"
            )

    winners: dict[str, dict] = {}
    for scene in SCENES:
        best = best_trial_for_scene(scene, log)
        if best:
            winners[scene] = {
                "trial_id": best["trial_id"],
                "params": best["params"],
                "result": best["result"],
            }
    return log, winners


def main() -> int:
    ASSETS.mkdir(parents=True, exist_ok=True)
    log, winners = run_all()
    TRIAL_LOG.write_text(json.dumps(log, indent=2, ensure_ascii=False), encoding="utf-8")
    RESULTS.write_text(json.dumps(winners, indent=2, ensure_ascii=False), encoding="utf-8")
    print("\n=== SUMMARY ===")
    for scene in SCENES:
        w = winners.get(scene)
        if w:
            r = w["result"]
            print(f"  {scene}: OK {w['trial_id']} dist={r['distance_m']:.1f}m {r['mean_kph']:.2f}kph")
        else:
            print(f"  {scene}: FAILED (no successful trial)")
    print(f"log: {TRIAL_LOG}")
    print(f"winners: {RESULTS}")
    return 0 if len(winners) == len(SCENES) else 1


if __name__ == "__main__":
    raise SystemExit(main())
