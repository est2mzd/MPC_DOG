import json
import sys
from collections import Counter
from pathlib import Path

import pytest
from PIL import Image

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from scripts.run_legged_control_benchmark import (
    SCENARIOS,
    Thresholds,
    assess_metrics,
    gif_metadata,
    run_scenario,
    validate_duration,
)


def test_benchmark_has_exactly_ten_unique_scenarios_per_level() -> None:
    assert len(SCENARIOS) == 30
    assert len({scenario.name for scenario in SCENARIOS}) == 30
    assert Counter(scenario.difficulty for scenario in SCENARIOS) == {
        "easy": 10,
        "normal": 10,
        "hard": 10,
    }
    assert all(scenario.seed > 0 for scenario in SCENARIOS)


def test_final_duration_guard_requires_explicit_smoke() -> None:
    with pytest.raises(ValueError, match=">=20.0"):
        validate_duration(19.99)
    validate_duration(20.0)
    validate_duration(0.03, smoke=True)
    with pytest.raises(ValueError, match="positive"):
        validate_duration(0.0, smoke=True)


def _passing_metrics() -> dict:
    return {
        "simulated_duration_s": 20.0,
        "fallen": False,
        "minimum_base_height_m": 0.30,
        "height_error_rmse_m": 0.02,
        "maximum_abs_roll_pitch_rad": 0.10,
        "velocity_tracking_rmse": {
            "vx_mps": 0.05,
            "vy_mps": 0.02,
            "yaw_rate_radps": 0.03,
            "planar_mps": 0.054,
        },
        "maximum_abs_torque_nm": 20.0,
        "torque_saturation_fraction": 0.0,
        "maximum_dynamics_residual": 0.2,
        "planned_vs_measured_contact_agreement": 0.90,
        "gif_frame_count": 200,
        "gif_playback_duration_s": 20.0,
    }


def test_metric_pass_logic_reports_physical_failures_and_serializes() -> None:
    metrics = _passing_metrics()
    passed, reasons = assess_metrics(metrics, Thresholds(20.0, 20.0))
    assert passed and reasons == []
    metrics["fallen"] = True
    metrics["fall_time_s"] = 12.79
    metrics["torque_saturation_fraction"] = 0.25
    passed, reasons = assess_metrics(metrics, Thresholds(20.0, 20.0))
    assert not passed
    assert "fall detected" in reasons
    assert "torque saturation fraction exceeds threshold" in reasons
    json.dumps({"metrics": metrics, "passed": passed, "failure_reasons": reasons}, allow_nan=False)


def test_smoke_benchmark_writes_gif_and_complete_metadata(tmp_path) -> None:
    scenario = SCENARIOS[0]
    record = run_scenario(
        scenario,
        duration=0.03,
        fps=10,
        width=160,
        height=120,
        output_root=tmp_path,
        render=True,
        overwrite=True,
    )
    metadata_path = tmp_path / "partial" / f"{scenario.name}.json"
    gif_path = tmp_path / "gifs" / f"{scenario.name}.gif"
    assert metadata_path.is_file() and gif_path.is_file()
    decoded = json.loads(metadata_path.read_text(encoding="utf-8"))
    assert decoded["metrics"] == record["metrics"]
    assert decoded["config"]["name"] == record["config"]["name"]
    assert decoded["upstream_commit"]
    assert "not Quadruped-PyMPC" in decoded["implementation_boundary"]
    metrics = decoded["metrics"]
    required = {
        "simulated_duration_s",
        "gif_path",
        "gif_playback_duration_s",
        "gif_frame_count",
        "final_base_xyz_m",
        "net_distance_m",
        "path_distance_m",
        "fallen",
        "fall_time_s",
        "height_error_rmse_m",
        "maximum_abs_roll_pitch_rad",
        "velocity_tracking_rmse",
        "maximum_abs_torque_nm",
        "torque_saturation_fraction",
        "maximum_dynamics_residual",
        "planned_vs_measured_contact_agreement",
        "passed",
        "failure_reasons",
    }
    assert required <= metrics.keys()
    with Image.open(gif_path) as image:
        assert image.format == "GIF"
        assert image.n_frames == metrics["gif_frame_count"]
    frame_count, playback_s = gif_metadata(gif_path)
    assert frame_count == metrics["gif_frame_count"]
    assert playback_s == metrics["gif_playback_duration_s"]
    assert playback_s >= 0.03
    resumed = run_scenario(
        scenario,
        duration=0.03,
        fps=10,
        width=160,
        height=120,
        output_root=tmp_path,
        render=True,
        overwrite=False,
    )
    assert resumed == decoded
