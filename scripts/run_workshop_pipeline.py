#!/usr/bin/env python3
"""Run all workshop computations: param study, GIFs, headless logs, execute notebooks."""
from __future__ import annotations

import json
import os
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
PYMPC = ROOT / "external" / "Quadruped-PyMPC"
ACADOS = PYMPC / "quadruped_pympc" / "acados"
ASSETS = ROOT / "docs" / "pympc_2day" / "assets"
NOTEBOOKS = ROOT / "docs" / "pympc_2day" / "notebooks"
PYTHON = ROOT / ".venv" / "bin" / "python"


def env() -> dict[str, str]:
    e = os.environ.copy()
    e["ACADOS_SOURCE_DIR"] = str(ACADOS)
    e["LD_LIBRARY_PATH"] = f"{ACADOS}/lib:" + e.get("LD_LIBRARY_PATH", "")
    e.setdefault("MUJOCO_GL", "egl")
    return e


def run(cmd: list[str], *, cwd: Path | None = None, timeout: int | None = None) -> None:
    print("+", " ".join(cmd), flush=True)
    subprocess.run(cmd, check=True, cwd=cwd or ROOT, env=env(), timeout=timeout)


def step_param_study() -> None:
    print("\n=== Parameter study ===")
    run([str(PYTHON), str(ROOT / "scripts" / "run_parameter_study.py")], timeout=900)


def step_headless_sessions() -> None:
    print("\n=== Headless session validation ===")
    results = {}
    for preset in [
        "session01_flat_smoke",
        "session02_flat_tune",
        "session03_rough_boxes",
        "session03_rough_perlin",
    ]:
        run(
            [str(PYTHON), str(ROOT / "scripts" / "run_pympc_headless.py"), preset, "--seconds", "8"],
            timeout=600,
        )
        log = ROOT / "logs" / "pympc_sessions" / preset / "headless_sim.log"
        ok = log.is_file() and "HEADLESS_SIM_OK" in log.read_text(encoding="utf-8", errors="replace")
        results[preset] = {"log": str(log), "ok": ok}
    ASSETS.mkdir(parents=True, exist_ok=True)
    (ASSETS / "headless_results.json").write_text(json.dumps(results, indent=2), encoding="utf-8")


def step_capture_gifs() -> None:
    print("\n=== Demo frame / GIF capture ===")
    run([str(PYTHON), str(ROOT / "scripts" / "capture_demo_frames.py")], timeout=900)
    run([str(PYTHON), str(ROOT / "scripts" / "capture_speed_terrain_demos.py")], timeout=3600)


def step_generate_notebooks() -> None:
    print("\n=== Generate notebooks ===")
    run([str(PYTHON), str(ROOT / "scripts" / "generate_workshop_notebooks.py")], timeout=120)


def step_execute_notebooks() -> None:
    print("\n=== Execute notebooks (nbconvert) ===")
    for nb in sorted(NOTEBOOKS.glob("*.ipynb")):
        if nb.name.startswith("_"):
            continue
        print(f"executing {nb.name}...")
        run(
            [
                str(PYTHON),
                "-m",
                "jupyter",
                "nbconvert",
                "--to",
                "notebook",
                "--execute",
                "--inplace",
                f"--ExecutePreprocessor.timeout=600",
                str(nb),
            ],
            timeout=3600,
        )


def main() -> int:
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument("--from-step", choices=("all", "capture", "notebooks"), default="all")
    args = parser.parse_args()

    if not PYTHON.is_file():
        print("ERROR: run ./scripts/setup_uv_workshop.sh first", file=sys.stderr)
        return 1
    ASSETS.mkdir(parents=True, exist_ok=True)
    if args.from_step == "all":
        step_param_study()
        step_headless_sessions()
        step_generate_notebooks()
        step_capture_gifs()
        step_execute_notebooks()
    elif args.from_step == "capture":
        step_generate_notebooks()
        step_capture_gifs()
        step_execute_notebooks()
    else:
        step_execute_notebooks()
    print("\n=== WORKSHOP PIPELINE OK ===")
    print(f"assets: {ASSETS}")
    print(f"notebooks: {NOTEBOOKS}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
