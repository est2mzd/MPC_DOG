#!/usr/bin/env python3
"""Apply mpc_dog preset YAML to Quadruped-PyMPC quadruped_pympc/config.py."""
from __future__ import annotations

import argparse
import re
import shutil
import sys
from datetime import datetime, timezone
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]
PYMPC_ROOT = ROOT / "external" / "Quadruped-PyMPC"
CONFIG_PATH = PYMPC_ROOT / "quadruped_pympc" / "config.py"
PRESET_DIR = ROOT / "configs" / "pympc_presets"


def load_preset(name: str) -> dict:
    path = PRESET_DIR / f"{name}.yaml"
    if not path.is_file():
        path = Path(name)
    if not path.is_file():
        raise FileNotFoundError(f"Preset not found: {name}")
    with path.open(encoding="utf-8") as f:
        return yaml.safe_load(f)


def _python_repr(value) -> str:
    if isinstance(value, bool):
        return "True" if value else "False"
    if isinstance(value, str):
        return repr(value)
    if value is None:
        raise ValueError("null values should be handled before repr")
    return repr(value)


def _set_robot(text: str, robot: str) -> str:
    return re.sub(r"^robot = .*$", f"robot = {repr(robot)}", text, count=1, flags=re.M)


def _find_block(text: str, marker: str) -> tuple[int, int]:
    start = text.index(marker)
    brace = text.index("{", start)
    depth = 0
    for i in range(brace, len(text)):
        if text[i] == "{":
            depth += 1
        elif text[i] == "}":
            depth -= 1
            if depth == 0:
                return brace, i + 1
    raise ValueError(f"Unclosed block: {marker}")


def _set_in_block(text: str, block_marker: str, key: str, value, *, optional: bool = False) -> str:
    b0, b1 = _find_block(text, block_marker)
    block = text[b0:b1]
    pat = rf"(\s+['\"]{re.escape(key)}['\"]\s*:\s*)([^,\n]+)(,)"
    if not re.search(pat, block):
        if optional:
            print(f"WARN: skip missing {block_marker}['{key}']")
            return text
        raise KeyError(f"{block_marker}['{key}'] not found")
    new_block = re.sub(pat, rf"\g<1>{_python_repr(value)}\3", block, count=1)
    return text[:b0] + new_block + text[b1:]


def _set_mpc_key(text: str, key: str, value, optional: bool = False) -> str:
    return _set_in_block(text, "mpc_params =", key, value, optional=optional)


def _set_sim_key(text: str, key: str, value, optional: bool = False) -> str:
    return _set_in_block(text, "simulation_params =", key, value, optional=optional)


def _set_gait_param(text: str, gait: str, param: str, value) -> str:
    """Patch step_freq / duty_factor inside gait_params['trot'] etc."""
    pat = (
        rf"('{re.escape(gait)}':\s*\{{[^}}]*?"
        rf"'{re.escape(param)}':\s*)([^,\n]+)(,)"
    )
    if not re.search(pat, text, flags=re.S):
        raise KeyError(f"gait_params {gait}.{param} not found")
    return re.sub(pat, rf"\g<1>{_python_repr(value)}\3", text, count=1, flags=re.S)


def _set_ref_z_scale(text: str, scale: float) -> str:
    """Replace ref_z with hip_height * scale (never stack on previous value)."""
    return re.sub(
        r"('ref_z':\s*)([^,\n]+)(,)",
        rf"\g<1>hip_height * {scale}\3",
        text,
        count=1,
    )


def _set_step_height_default(text: str) -> str:
    if re.search(r"'step_height':\s*0\.2 \* hip_height", text):
        return text
    return _set_sim_key(text, "step_height", "PLACEHOLDER")


def apply_preset(preset: dict, dry_run: bool = False) -> Path:
    if not CONFIG_PATH.is_file():
        raise FileNotFoundError(
            f"{CONFIG_PATH} missing. Run: ./scripts/setup_references.sh"
        )

    text = CONFIG_PATH.read_text(encoding="utf-8")
    log_dir = ROOT / "logs" / "pympc_sessions" / preset.get("session_id", "unknown")
    log_dir.mkdir(parents=True, exist_ok=True)
    ts = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    backup = log_dir / f"config.py.bak.{ts}"

    patches = preset.get("patches") or {}
    if "robot" in patches:
        text = _set_robot(text, patches.pop("robot"))

    for key, value in patches.items():
        if value is None:
            continue
        optional = key in preset.get("optional_patches", [])
        if key.startswith("mpc_params."):
            text = _set_mpc_key(text, key.split(".", 1)[1], value, optional=optional)
        elif key.startswith("simulation_params."):
            k = key.split(".", 1)[1]
            if k == "step_height" and value is None:
                continue
            text = _set_sim_key(text, k, value, optional=optional)
        else:
            raise ValueError(f"Unknown patch key: {key}")

    for gait, params in (preset.get("gait_patches") or {}).items():
        for param, value in params.items():
            text = _set_gait_param(text, gait, param, value)

    if preset.get("ref_z_scale"):
        text = _set_ref_z_scale(text, float(preset["ref_z_scale"]))

    if dry_run:
        print(text[:2000], "...")
        return CONFIG_PATH

    if CONFIG_PATH.exists():
        shutil.copy2(CONFIG_PATH, backup)
        print(f"backup: {backup}")

    CONFIG_PATH.write_text(text, encoding="utf-8")
    (log_dir / "preset_applied.yaml").write_text(
        yaml.dump(preset, allow_unicode=True), encoding="utf-8"
    )
    print(f"applied: {preset.get('session_id')} -> {CONFIG_PATH}")
    return CONFIG_PATH


def main() -> int:
    parser = argparse.ArgumentParser(description="Apply PyMPC session preset")
    parser.add_argument("preset", help="Preset name (without .yaml) or path")
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()

    try:
        preset = load_preset(args.preset)
        apply_preset(preset, dry_run=args.dry_run)
        if preset.get("notes"):
            print(f"notes: {preset['notes'].strip()}")
        return 0
    except (FileNotFoundError, KeyError, ValueError) as e:
        print(f"ERROR: {e}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
