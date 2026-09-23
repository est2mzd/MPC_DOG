import math
from pathlib import Path

from recovery_controller_node import (
    body_up_value,
    classify_pose,
    interpolate,
    load_config,
    mirror_left_right,
    recovery_frame_action,
    sequence_name_for_pose,
    should_switch_sequence,
    supine_chain_start,
)


DETECTION = {
    "upright_up_min": 0.82,
    "upright_height_min": 0.22,
    "supine_up_max": -0.55,
    "side_abs_up_max": 0.65,
}


def test_body_up_is_quaternion_sign_invariant():
    angle = math.pi
    qx = math.sin(angle / 2.0)
    qy = 0.0
    assert body_up_value(qx, qy) == body_up_value(-qx, -qy)
    assert math.isclose(body_up_value(qx, qy), -1.0)


def test_pose_classes_cover_required_initial_conditions():
    root = math.sqrt(0.5)
    assert classify_pose(1.0, 0.0, 0.0, 0.0, 0.14, DETECTION) == "supine"
    assert classify_pose(root, 0.0, 0.0, root, 0.14, DETECTION) == "right_side"
    assert classify_pose(-root, 0.0, 0.0, root, 0.14, DETECTION) == "left_side"
    assert classify_pose(0.0, 0.0, 0.0, 1.0, 0.27, DETECTION) == "upright"
    assert classify_pose(0.0, 0.0, 0.0, 1.0, 0.12, DETECTION) == "prone"


def test_interpolation_has_exact_endpoints():
    start = [0.0, 1.0, -2.0]
    goal = [1.0, 2.0, -1.0]
    assert interpolate(start, goal, 0.0) == start
    assert interpolate(start, goal, 1.0) == goal
    assert interpolate(start, goal, 0.25)[0] > 0.5


def test_supine_push_uses_one_side_and_can_reverse():
    config = load_config(Path(__file__).parents[1] / "config" / "go2_recovery.yaml")
    names = [frame["name"] for frame in config["sequences"]["supine"]]
    assert names[:2] == ["open_gap", "side_push"]
    gap = config["sequences"]["supine"][0]["joints"]
    push = config["sequences"]["supine"][1]["joints"]
    assert gap[0] > 0.8 and gap[6] < -0.8
    assert push[1] > 2.8
    assert push[1] > push[7]
    assert push[7] > 2.0
    assert push[0] > 0.0 and push[6] < 0.0
    reversed_push = mirror_left_right(push)
    assert reversed_push[7] > reversed_push[1]
    assert reversed_push[6] < 0.0 and reversed_push[0] > 0.0


def test_supine_does_not_stand_until_the_back_leaves_the_ground():
    assert recovery_frame_action("open_gap", "supine", False) == "advance"
    assert recovery_frame_action("side_push", "supine", False) == "reverse_push"
    assert recovery_frame_action("side_push", "supine", False, -0.60) == "hold"
    assert recovery_frame_action("side_push", "supine", True) == "retry"
    assert recovery_frame_action("low_support", "supine", True) == "retry"
    assert recovery_frame_action("stand", "supine", True) == "retry"
    assert recovery_frame_action("side_push", "right_side", False) == "hold"
    assert recovery_frame_action("side_push", "left_side", True) == "hold"
    assert recovery_frame_action("side_push", "prone", False) == "advance"
    assert recovery_frame_action("low_support", "right_side", False) == "advance"
    assert should_switch_sequence("supine", "left_side") is False
    assert should_switch_sequence("supine", "prone") is False
    assert should_switch_sequence("left_side", "prone") is False
    assert should_switch_sequence("left_side", "supine") is True
    assert should_switch_sequence("right_side", "supine") is True
    assert sequence_name_for_pose("left_side", 0.62) == "prone"
    assert sequence_name_for_pose("left_side", -0.05) == "left_side"
    assert supine_chain_start("left_side") == ("side_push", True)
    assert supine_chain_start("right_side") == ("side_push", False)
    assert supine_chain_start("supine") == (None, False)
    assert sequence_name_for_pose("supine", -1.0) == "supine"


def test_go2_config_has_four_valid_sequences():
    config_path = (
        Path(__file__).parents[1] / "config" / "go2_recovery.yaml"
    )
    config = load_config(config_path)
    assert set(config["sequences"]) == {
        "supine",
        "left_side",
        "right_side",
        "prone",
    }
