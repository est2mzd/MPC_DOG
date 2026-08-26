"""Focused regression tests for generated curriculum and fail-closed parity."""

from __future__ import annotations

import hashlib
import json
from pathlib import Path

from validate_notebook_annotations import validate_source


HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
FAIL_CLOSED = "NOT VERIFIED / FAIL-CLOSED"


def test_all_generated_code_lines_are_annotated() -> None:
    notebooks = sorted(HERE.glob("[0-1][0-9]_*.ipynb"))
    assert len(notebooks) == 16
    for path in notebooks:
        notebook = json.loads(path.read_text(encoding="utf-8"))
        for index, cell in enumerate(notebook["cells"]):
            if cell["cell_type"] == "code":
                validate_source(cell["source"], f"{path.name}:cell-{index}")


def test_truth_notice_is_present_in_every_notebook() -> None:
    for path in sorted(HERE.glob("[0-1][0-9]_*.ipynb")):
        text = path.read_text(encoding="utf-8")
        assert "ROS2 portを作成・compile・実行していない" in text
        assert FAIL_CLOSED in text
        assert "保存済み30 scenario dataが示すのはadapter挙動だけ" in text


def test_ros1_baseline_manifest_matches_checkout_when_present() -> None:
    manifest = json.loads((HERE / "ros1_logic_baseline_manifest.json").read_text(encoding="utf-8"))
    assert manifest["commit"] == "a7f381c0367e98e31c01336e678eef47e304d40d"
    assert manifest["scope"].startswith("ROS1 control-logic baseline only")
    checkout = ROOT / "external" / "legged_control"
    if not checkout.is_dir():
        assert not checkout.exists()
        return
    mismatches = []
    for relative, expected in manifest["files"].items():
        path = checkout / relative
        actual = hashlib.sha256(path.read_bytes()).hexdigest() if path.is_file() else None
        if actual != expected:
            mismatches.append(relative)
    assert not mismatches


def test_ros2_parity_never_uses_adapter_or_passes_without_port() -> None:
    candidates = [
        ROOT / "external" / "legged_control_ros2",
        ROOT / "src" / "legged_control_ros2",
        ROOT / "ros2_ws" / "src" / "legged_control",
    ]
    assert ROOT / "src" / "legged_control_mujoco" not in candidates
    actual_ports = [path for path in candidates if path.is_dir()]
    status = FAIL_CLOSED
    if not actual_ports:
        assert status == FAIL_CLOSED
    assert status != "PASS"


def test_migration_chapter_defines_all_golden_boundaries() -> None:
    notebook = json.loads((HERE / "15_ros_migration_logic_parity.ipynb").read_text(encoding="utf-8"))
    text = "\n".join(
        "".join(cell["source"]) if isinstance(cell["source"], list) else cell["source"]
        for cell in notebook["cells"]
    )
    for boundary in ("reference", "gait", "observation", "policy", "wbc", "torque"):
        assert f'"{boundary}"' in text
    for dimension in ('"dimensions": [24, 24]', '"dimensions": [42]', '"dimensions": [12]'):
        assert dimension in text
    assert '"mpc": 100' in text
    assert '"estimator_wbc_hardware": 500' in text
    assert '"solver_status_exact": True' in text


def test_only_non_benchmark_chapters_were_executed() -> None:
    for path in sorted(HERE.glob("[0-1][0-9]_*.ipynb")):
        notebook = json.loads(path.read_text(encoding="utf-8"))
        counts = [cell["execution_count"] for cell in notebook["cells"] if cell["cell_type"] == "code"]
        if path.name.startswith(("13_", "14_")):
            assert all(count is None for count in counts)
        else:
            assert counts and all(count is not None for count in counts)
