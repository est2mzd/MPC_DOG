from pathlib import Path

import mujoco
import numpy as np
from PIL import Image

from legged_control_mujoco import (
    A1HeadlessAdapter,
    CONTACT_NAMES,
    JOINT_NAMES,
    ModeSchedule,
    MotionCommand,
    ScenarioConfig,
    TORQUE_LIMIT,
    hybrid_command,
    load_a1_model,
    mode_contacts,
    plan_contact_forces,
    save_gif,
)


def test_a1_model_has_floating_base_and_upstream_joints() -> None:
    model = load_a1_model()
    assert model.nq == 19
    assert model.nv == 18
    assert model.nu == 12
    names = tuple(
        mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, index)
        for index in range(1, model.njnt)
    )
    assert names == JOINT_NAMES
    for foot in CONTACT_NAMES:
        assert mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, f"{foot}_FOOT") >= 0


def test_scenario_config_applies_contact_friction_payload_and_tangent_feet() -> None:
    baseline = A1HeadlessAdapter()
    loaded = A1HeadlessAdapter(
        scenario=ScenarioConfig(
            friction=0.42,
            payload_mass=2.0,
            push_force=(10.0, 0.0, 0.0),
            push_start=0.1,
            push_duration=0.2,
        )
    )
    assert np.isclose(loaded.mass, baseline.mass + 2.0)
    np.testing.assert_allclose(
        loaded.model.geom_friction[loaded.foot_geom_ids, 0],
        0.42,
    )
    assert np.min(loaded.data.site_xpos[loaded.site_ids, 2]) >= 0.019


def test_gait_contacts_match_gait_info_trot() -> None:
    schedule = ModeSchedule.from_gait("trot")
    np.testing.assert_array_equal(schedule.contacts_at(0.10), [True, False, False, True])
    np.testing.assert_array_equal(schedule.contacts_at(0.31), [False, True, True, False])
    np.testing.assert_array_equal(schedule.contacts_at(0.61), [True, False, False, True])
    np.testing.assert_array_equal(mode_contacts("STANCE"), np.ones(4, dtype=bool))
    np.testing.assert_array_equal(mode_contacts("FLY"), np.zeros(4, dtype=bool))


def test_force_plan_obeys_zero_force_and_friction_pyramid() -> None:
    contacts = np.array([True, False, True, False])
    forces = plan_contact_forces(contacts, mass=12.0, desired_accel=(20.0, -20.0, 0.0))
    np.testing.assert_array_equal(forces[~contacts], 0.0)
    assert np.all(forces[contacts, 2] >= 0.0)
    assert np.all(np.abs(forces[contacts, 0]) <= 0.3 * forces[contacts, 2] + 1e-12)
    assert np.all(np.abs(forces[contacts, 1]) <= 0.3 * forces[contacts, 2] + 1e-12)


def test_wbc_uses_mujoco_dynamics_and_returns_constrained_forces() -> None:
    adapter = A1HeadlessAdapter(gait="trot")
    contacts = adapter.schedule.contacts_at(0.0)
    result = adapter.solve_wbc(contacts)
    np.testing.assert_allclose(result.contact_forces[~contacts], 0.0, atol=1e-9)
    active = result.contact_forces[contacts]
    assert np.all(active[:, 2] >= -1e-8)
    assert np.all(np.abs(active[:, :2]) <= 0.3 * active[:, 2, None] + 1e-7)
    assert np.max(np.abs(result.torque)) <= TORQUE_LIMIT
    assert np.isfinite(result.dynamics_residual)


def test_hybrid_command_saturates_at_upstream_effort_limit() -> None:
    zeros = np.zeros(12)
    command = hybrid_command(
        tau_ff=np.full(12, 100.0),
        q_des=zeros,
        q=zeros,
        dq_des=zeros,
        dq=np.full(12, -10.0),
    )
    np.testing.assert_array_equal(command, np.full(12, TORQUE_LIMIT))


def test_short_run_is_deterministic() -> None:
    first = A1HeadlessAdapter(seed=7).run(duration=0.03)
    second = A1HeadlessAdapter(seed=7).run(duration=0.03)
    np.testing.assert_array_equal(first.times, second.times)
    np.testing.assert_allclose(first.qpos, second.qpos, rtol=0.0, atol=1e-13)
    np.testing.assert_allclose(first.torques, second.torques, rtol=0.0, atol=1e-13)


def test_stance_remains_upright_for_twenty_seconds() -> None:
    result = A1HeadlessAdapter(gait="stance", seed=11).run(duration=20.0)
    assert not result.fallen, f"stance fell at {result.fall_time}"
    assert 0.28 <= result.metrics.base_xyz[-1, 2] <= 0.36
    assert np.max(np.abs(result.metrics.base_rpy[:, :2])) < 0.15
    assert result.torque_saturation_fraction < 0.01
    assert result.max_dynamics_residual < 0.1


def test_commanded_static_walk_moves_forward_without_early_fall() -> None:
    adapter = A1HeadlessAdapter(
        gait="static_walk",
        seed=13,
        command=MotionCommand(vx=0.12),
        scenario=ScenarioConfig(friction=0.6),
    )
    result = adapter.run(duration=5.0)
    forward = result.metrics.base_xyz[-1, 0] - result.metrics.base_xyz[0, 0]
    assert not result.fallen, f"walking fell at {result.fall_time}"
    assert forward > 0.30, f"forward displacement was only {forward:.3f} m"
    assert result.distance_xy > 0.30
    assert result.metrics.base_xyz[-1, 2] > 0.25
    assert result.metrics.planned_contacts.shape == result.metrics.measured_contacts.shape


def test_gif_creation(tmp_path: Path) -> None:
    frames = [
        np.full((24, 32, 3), [240, 80, 40], dtype=np.uint8),
        np.full((24, 32, 3), [40, 80, 240], dtype=np.uint8),
    ]
    output = save_gif(frames, tmp_path / "a1.gif", fps=20)
    assert output.exists() and output.stat().st_size > 0
    with Image.open(output) as image:
        assert image.format == "GIF"
        assert image.n_frames == 2
