#!/usr/bin/env python3
"""Verify workshop demo assets meet terrain / distance requirements."""
from __future__ import annotations

import json
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
ASSETS = ROOT / "docs" / "pympc_2day" / "assets"


def _require(path: Path, errors: list[str]) -> bool:
    if not path.is_file():
        errors.append(f"MISSING: {path.relative_to(ROOT)}")
        return False
    return True


def _parse_overlay_png(path: Path) -> dict[str, str | float]:
    """Parse dist/falls/scene lines from capture overlay (PIL pixel scan fallback)."""
    from PIL import Image

    im = Image.open(path).convert("RGB")
    # Read bottom overlay line area - white text on dark box at y~60-85
    meta: dict[str, str | float] = {}
    crop = im.crop((0, 0, min(640, im.width), min(100, im.height)))
    # Save temp and use simple regex on known capture format via reading all saved metadata if exists
    sidecar = path.with_suffix(path.suffix + ".meta.json")
    if sidecar.is_file():
        return json.loads(sidecar.read_text(encoding="utf-8"))
    return meta


def _ground_std(path: Path) -> float:
    import numpy as np
    from PIL import Image

    arr = np.array(Image.open(path).convert("RGB"))
    h, w = arr.shape[:2]
    ground = arr[h // 2 :, w // 4 : 3 * w // 4]
    return float(ground.mean(axis=2).std())


def _read_sidecar(png: Path) -> dict | None:
    sc = png.with_name(png.stem + ".meta.json")
    if sc.is_file():
        return json.loads(sc.read_text(encoding="utf-8"))
    return None


def verify() -> int:
    errors: list[str] = []
    warnings: list[str] = []

    # --- Required files ---
    required_gifs = [
        "demo_s01_flat.gif",
        "demo_s02_tune.gif",
        "demo_s03_boxes.gif",
        "demo_s03_perlin.gif",
        "demo_s04_flat.gif",
        "demo_s04_uphill.gif",
        "demo_s04_downhill.gif",
    ]
    for name in required_gifs:
        _require(ASSETS / name, errors)

    for name in ["demo_s01_flat.mp4", "demo_s03_boxes.mp4", "demo_s03_perlin.mp4"]:
        if not (ASSETS / name).is_file():
            warnings.append(f"MP4 missing (optional): {name}")

    # --- JSON benchmarks ---
    results = json.loads((ASSETS / "speed_terrain_results.json").read_text())
    for scene, spec in results.items():
        r = spec["result"]
        if r.get("distance_m", 0) < 19.99:
            errors.append(f"S4 {scene}: distance_m={r.get('distance_m')} < 20")
        if not r.get("success", False):
            errors.append(f"S4 {scene}: success=false in speed_terrain_results.json")

    headless = json.loads((ASSETS / "headless_results.json").read_text())
    for preset, v in headless.items():
        if not v.get("ok"):
            errors.append(f"headless failed: {preset}")

    # --- Capture metadata sidecars ---
    checks = [
        ("s03_boxes", "demo_s03_boxes.png", "random_boxes", 1.0, None),
        ("s03_perlin", "demo_s03_perlin.png", "perlin", 1.0, None),
        ("s04_flat", "demo_s04_flat.png", "bumpy_flat", None, 19.5),
        ("s04_uphill", "demo_s04_uphill.png", "bumpy_uphill", None, 19.5),
        ("s04_downhill", "demo_s04_downhill.png", "bumpy_downhill", None, 19.5),
    ]
    s01_std = _ground_std(ASSETS / "demo_s01_flat.png") if (ASSETS / "demo_s01_flat.png").is_file() else 0.0

    for tag, png_name, scene, min_x, min_dist in checks:
        png = ASSETS / png_name
        if not png.is_file():
            continue
        meta = _read_sidecar(png)
        if meta is None:
            errors.append(f"{tag}: no sidecar meta JSON — re-run capture script")
            continue
        if meta.get("scene") != scene:
            errors.append(f"{tag}: scene={meta.get('scene')} expected {scene}")
        if min_x is not None and float(meta.get("final_x_m", 0)) < min_x:
            errors.append(f"{tag}: final_x_m={meta.get('final_x_m')} < {min_x} (never reached terrain?)")
        if min_dist is not None and float(meta.get("final_dist_m", 0)) < min_dist:
            errors.append(
                f"{tag}: final_dist_m={meta.get('final_dist_m')} < {min_dist} (GIF shows start only?)"
            )
        gstd = _ground_std(png)
        if tag.startswith("s03") and gstd < s01_std * 0.8 and float(meta.get("final_x_m", 0)) < 2.0:
            warnings.append(f"{tag}: low ground variance at low x — may still be on flat approach")

    # --- S4 scenes must differ ---
    s4_scenes = {k: _read_sidecar(ASSETS / f"demo_{k}.png") for k in ["s04_flat", "s04_uphill", "s04_downhill"]}
    if all(v for v in s4_scenes.values()):
        if len({m["scene"] for m in s4_scenes.values()}) < 3:
            errors.append("S4: flat/uphill/downhill not all distinct scenes")

    print("=== Workshop asset verification ===")
    if warnings:
        print("\nWarnings:")
        for w in warnings:
            print(f"  ⚠ {w}")
    if errors:
        print("\nFAILED:")
        for e in errors:
            print(f"  ✗ {e}")
        return 1
    print("\nOK — all checks passed")
    for tag, png_name, scene, _, min_dist in checks:
        meta = _read_sidecar(ASSETS / png_name)
        if meta:
            dist = meta.get("final_dist_m", meta.get("final_x_m"))
            print(
                f"  {tag}: scene={meta.get('scene')} "
                f"dist/x={dist}m falls={meta.get('falls', '-')}"
            )
    return 0


if __name__ == "__main__":
    raise SystemExit(verify())
