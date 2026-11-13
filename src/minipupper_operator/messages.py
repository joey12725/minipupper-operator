"""Pure helpers that build ROS 2 message payloads and pick topics.

Nothing in here touches the network or ROS, so it is safe to import and
test on any machine. The strings produced are the YAML-ish literals that
``ros2 topic pub`` accepts on the command line.
"""

from __future__ import annotations

import math
from collections.abc import Iterable, Mapping, Sequence

# Joint order expected by the CHAMP-based Mini Pupper controller
# (`/joint_group_effort_controller/joint_trajectory`).
JOINT_NAMES: tuple[str, ...] = (
    "base_lf1",
    "lf1_lf2",
    "lf2_lf3",
    "base_rf1",
    "rf1_rf2",
    "rf2_rf3",
    "base_lb1",
    "lb1_lb2",
    "lb2_lb3",
    "base_rb1",
    "rb1_rb2",
    "rb2_rb3",
)

# Hand-tuned on a Mini Pupper 1 after the calibration fix described in
# docs/field-notes.md. Values are degrees, one (hip, upper, lower) triple per leg
# in JOINT_NAMES order: LF, RF, LB, RB.
JOINT_POSE_PRESETS_DEG: dict[str, tuple[float, ...]] = {
    "neutral": (0.0, 15.0, -20.0) * 4,
    "stand": (0.0, 8.0, -12.0) * 4,
    "sit": (0.0, 25.0, -35.0) * 2 + (0.0, 40.0, -55.0) * 2,
    "lay": (0.0, 40.0, -55.0) * 4,
}

# For each state category, the first topic that exists on the robot wins.
STATE_TOPIC_PRIORITIES: dict[str, tuple[str, ...]] = {
    "joint_states": ("/joint_states",),
    "imu": ("/imu/data", "/imu", "/imu/data_raw"),
    "odom": ("/odom", "/odometry/filtered"),
    "scan": ("/scan",),
}

# Conservative envelope for bench testing. The stock gait can go faster, but
# a short, slow test is what this tool is for.
MAX_LINEAR_SPEED = 0.5  # m/s
MAX_ANGULAR_SPEED = 2.0  # rad/s
MAX_MOTION_DURATION = 30.0  # s


class MotionLimitError(ValueError):
    """Raised when a requested motion falls outside the bench-test envelope."""


def validate_motion(linear_x: float, linear_y: float, angular_z: float, duration_sec: float) -> None:
    if not duration_sec > 0:
        raise MotionLimitError("--duration-sec must be greater than 0")
    if duration_sec > MAX_MOTION_DURATION:
        raise MotionLimitError(f"--duration-sec must be <= {MAX_MOTION_DURATION:g}s")
    for name, value in (("linear-x", linear_x), ("linear-y", linear_y)):
        if abs(value) > MAX_LINEAR_SPEED:
            raise MotionLimitError(f"--{name} must be within +/-{MAX_LINEAR_SPEED:g} m/s")
    if abs(angular_z) > MAX_ANGULAR_SPEED:
        raise MotionLimitError(f"--angular-z must be within +/-{MAX_ANGULAR_SPEED:g} rad/s")


def choose_topics(topics: Iterable[str], explicit_topic: str = "auto") -> list[str]:
    """Pick one topic per state category, or just the explicit one."""
    if explicit_topic != "auto":
        return [explicit_topic]
    available = set(topics)
    selected: list[str] = []
    for candidates in STATE_TOPIC_PRIORITIES.values():
        for candidate in candidates:
            if candidate in available:
                selected.append(candidate)
                break
    return selected


def missing_topics(topics: Iterable[str], required: Sequence[str]) -> list[str]:
    available = set(topics)
    return [topic for topic in required if topic not in available]


def build_twist_message(linear_x: float, linear_y: float, angular_z: float) -> str:
    return f"{{linear: {{x: {linear_x:.4f}, y: {linear_y:.4f}, z: 0.0}}, angular: {{x: 0.0, y: 0.0, z: {angular_z:.4f}}}}}"


STOP_TWIST = build_twist_message(0.0, 0.0, 0.0)


def quaternion_from_rpy(roll: float, pitch: float, yaw: float) -> tuple[float, float, float, float]:
    """Convert roll/pitch/yaw (radians) to an (x, y, z, w) quaternion."""
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return x, y, z, w


def build_pose_message(x: float, y: float, z_offset: float, roll: float, pitch: float, yaw: float) -> str:
    qx, qy, qz, qw = quaternion_from_rpy(roll, pitch, yaw)
    return (
        f"{{position: {{x: {x:.4f}, y: {y:.4f}, z: {z_offset:.4f}}}, orientation: {{x: {qx:.6f}, y: {qy:.6f}, z: {qz:.6f}, w: {qw:.6f}}}}}"
    )


def build_joint_trajectory_message(positions_deg: Sequence[float], move_time_ns: int = 200_000_000) -> str:
    if len(positions_deg) != len(JOINT_NAMES):
        raise ValueError(f"expected {len(JOINT_NAMES)} joint positions, got {len(positions_deg)}")
    positions_text = ", ".join(f"{math.radians(value):.6f}" for value in positions_deg)
    names_text = ", ".join(JOINT_NAMES)
    sec, nanosec = divmod(move_time_ns, 1_000_000_000)
    return (
        "{"
        "header: {stamp: {sec: 0, nanosec: 0}, frame_id: ''}, "
        f"joint_names: [{names_text}], "
        f"points: [{{positions: [{positions_text}], time_from_start: {{sec: {sec}, nanosec: {nanosec}}}}}]"
        "}"
    )


def teleop_twist(
    keys_down: Iterable[str],
    linear_speed: float,
    angular_speed: float,
) -> tuple[float, float]:
    """Map held W/A/S/D keys to (linear_x, angular_z). Opposite keys cancel."""
    keys = set(keys_down)
    linear = (linear_speed if "w" in keys else 0.0) - (linear_speed if "s" in keys else 0.0)
    angular = (angular_speed if "a" in keys else 0.0) - (angular_speed if "d" in keys else 0.0)
    return linear, angular


def occupancy_color(value: int) -> tuple[int, int, int]:
    """Grey for unknown (-1), near-black for occupied (>= 65), off-white for free."""
    if value < 0:
        return (185, 185, 185)
    if value >= 65:
        return (25, 25, 25)
    return (245, 245, 245)


def occupancy_rows(data: Sequence[int], width: int, height: int) -> list[list[tuple[int, int, int]]]:
    """Turn a row-major OccupancyGrid into top-down image rows.

    ROS grids start at the bottom-left; images start at the top-left, so the
    row order is flipped.
    """
    if len(data) != width * height:
        raise ValueError(f"grid data has {len(data)} cells, expected {width * height}")
    return [[occupancy_color(data[y * width + x]) for x in range(width)] for y in range(height - 1, -1, -1)]


def summarize_preset(name: str, presets: Mapping[str, Sequence[float]] = JOINT_POSE_PRESETS_DEG) -> str:
    values = presets[name]
    legs = ("LF", "RF", "LB", "RB")
    parts = [f"{leg}({values[i * 3]:g},{values[i * 3 + 1]:g},{values[i * 3 + 2]:g})" for i, leg in enumerate(legs)]
    return f"{name}: " + " ".join(parts)
