"""Tests for apply_pympc_preset (uses fixture config, no PyMPC clone required)."""
from __future__ import annotations

import shutil
import sys
from pathlib import Path

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))

FIXTURE = ROOT / "tests" / "fixtures" / "pympc_config_sample.py"
PRESET = ROOT / "configs" / "pympc_presets" / "session01_flat_smoke.yaml"


@pytest.fixture
def mock_pympc_config(tmp_path, monkeypatch):
    pympc = tmp_path / "Quadruped-PyMPC" / "quadruped_pympc"
    pympc.mkdir(parents=True)
    cfg = pympc / "config.py"
    shutil.copy(FIXTURE, cfg)
    import apply_pympc_preset as app

    monkeypatch.setattr(app, "PYMPC_ROOT", tmp_path / "Quadruped-PyMPC")
    monkeypatch.setattr(app, "CONFIG_PATH", cfg)
    monkeypatch.setattr(app, "ROOT", tmp_path)
    return app, cfg


def test_apply_session01_flat_smoke(mock_pympc_config):
    app, cfg = mock_pympc_config
    preset = yaml.safe_load(PRESET.read_text())
    app.apply_preset(preset)
    text = cfg.read_text()
    assert "'use_foothold_optimization': False" in text
    assert "'scene': 'flat'" in text
    assert "robot = 'go2'" in text
    assert "'dt': 0.02" in text  # mpc_params dt unchanged scope
    assert "'dt': 0.002" in text  # simulation dt unchanged
