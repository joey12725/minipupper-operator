import math

import pytest

from minipupper_operator import messages as msg


def test_presets_cover_all_joints():
    for name, values in msg.JOINT_POSE_PRESETS_DEG.items():
        assert len(values) == len(msg.JOINT_NAMES), name


def test_sit_tucks_rear_legs_more_than_front():
    sit = msg.JOINT_POSE_PRESETS_DEG["sit"]
    front_upper, rear_upper = sit[1], sit[7]
    assert rear_upper > front_upper


def test_twist_message_format():
    assert msg.build_twist_message(0.15, 0.0, -0.6) == ("{linear: {x: 0.1500, y: 0.0000, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -0.6000}}")


def test_stop_twist_is_all_zero():
    assert "0.0000" in msg.STOP_TWIST
    assert "x: 0.1" not in msg.STOP_TWIST


@pytest.mark.parametrize(
    ("rpy", "expected"),
    [
        ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)),
        ((0.0, 0.0, math.pi), (0.0, 0.0, 1.0, 0.0)),
        ((math.pi / 2, 0.0, 0.0), (math.sqrt(0.5), 0.0, 0.0, math.sqrt(0.5))),
    ],
)
def test_quaternion_from_rpy(rpy, expected):
    assert msg.quaternion_from_rpy(*rpy) == pytest.approx(expected, abs=1e-9)


def test_quaternion_is_unit_length():
    q = msg.quaternion_from_rpy(0.3, -0.2, 1.1)
    assert math.fsum(c * c for c in q) == pytest.approx(1.0)


def test_pose_message_includes_offset_and_orientation():
    text = msg.build_pose_message(0.01, -0.02, -0.03, 0.0, 0.0, 0.0)
    assert "position: {x: 0.0100, y: -0.0200, z: -0.0300}" in text
    assert "w: 1.000000" in text


def test_joint_trajectory_converts_degrees_and_orders_joints():
    text = msg.build_joint_trajectory_message(msg.JOINT_POSE_PRESETS_DEG["neutral"])
    assert "joint_names: [base_lf1, lf1_lf2, lf2_lf3, base_rf1" in text
    assert f"{math.radians(15.0):.6f}" in text
    assert "time_from_start: {sec: 0, nanosec: 200000000}" in text


def test_joint_trajectory_splits_long_move_time():
    text = msg.build_joint_trajectory_message([0.0] * 12, move_time_ns=1_500_000_000)
    assert "{sec: 1, nanosec: 500000000}" in text


def test_joint_trajectory_rejects_wrong_length():
    with pytest.raises(ValueError):
        msg.build_joint_trajectory_message([0.0] * 11)


def test_choose_topics_prefers_first_available_per_category():
    topics = ["/cmd_vel", "/imu", "/imu/data_raw", "/joint_states", "/odometry/filtered", "/rosout"]
    assert msg.choose_topics(topics) == ["/joint_states", "/imu", "/odometry/filtered"]


def test_choose_topics_explicit_topic_wins():
    assert msg.choose_topics(["/joint_states"], "/battery") == ["/battery"]


def test_missing_topics_keeps_required_order():
    assert msg.missing_topics(["/odom"], ["/scan", "/odom", "/map"]) == ["/scan", "/map"]


@pytest.mark.parametrize(
    "args",
    [
        (0.0, 0.0, 0.0, 0.0),
        (0.0, 0.0, 0.0, -1.0),
        (0.0, 0.0, 0.0, msg.MAX_MOTION_DURATION + 1),
        (msg.MAX_LINEAR_SPEED + 0.01, 0.0, 0.0, 1.0),
        (0.0, -msg.MAX_LINEAR_SPEED - 0.01, 0.0, 1.0),
        (0.0, 0.0, msg.MAX_ANGULAR_SPEED + 0.1, 1.0),
        (0.0, 0.0, 0.0, float("nan")),
    ],
)
def test_validate_motion_rejects_out_of_envelope(args):
    with pytest.raises(msg.MotionLimitError):
        msg.validate_motion(*args)


def test_validate_motion_accepts_bench_test():
    msg.validate_motion(0.15, 0.0, 0.6, 1.5)


@pytest.mark.parametrize(
    ("keys", "expected"),
    [
        (set(), (0.0, 0.0)),
        ({"w"}, (0.1, 0.0)),
        ({"s", "d"}, (-0.1, -0.9)),
        ({"w", "s"}, (0.0, 0.0)),
        ({"a", "x"}, (0.0, 0.9)),
    ],
)
def test_teleop_twist(keys, expected):
    assert msg.teleop_twist(keys, 0.1, 0.9) == pytest.approx(expected)


def test_occupancy_colors():
    assert msg.occupancy_color(-1) == (185, 185, 185)
    assert msg.occupancy_color(0) == (245, 245, 245)
    assert msg.occupancy_color(64) == (245, 245, 245)
    assert msg.occupancy_color(65) == (25, 25, 25)
    assert msg.occupancy_color(100) == (25, 25, 25)


def test_occupancy_rows_flip_vertically():
    # 2x2 grid, row-major from the bottom-left: bottom row free, top row occupied.
    rows = msg.occupancy_rows([0, 0, 100, 100], width=2, height=2)
    assert rows[0] == [(25, 25, 25)] * 2
    assert rows[1] == [(245, 245, 245)] * 2


def test_occupancy_rows_rejects_bad_size():
    with pytest.raises(ValueError):
        msg.occupancy_rows([0, 0, 0], width=2, height=2)


def test_summarize_preset():
    assert msg.summarize_preset("stand").startswith("stand: LF(0,8,-12) RF(0,8,-12)")
