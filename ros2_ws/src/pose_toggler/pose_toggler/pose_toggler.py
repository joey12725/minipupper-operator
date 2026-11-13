# Copyright (c) 2026 Joey Hersh
# SPDX-License-Identifier: MIT
"""
Cycle a Mini Pupper through joint-space pose presets.

On the stock image, CHAMP's /body_pose did not produce usable posture changes,
while JointTrajectory commands sent straight to the effort controller did.
This node uses that path: every `period_s` seconds it publishes the next
preset from `sequence` to /joint_group_effort_controller/joint_trajectory.
"""
import math

import rclpy
from builtin_interfaces.msg import Duration
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from pose_toggler.presets import JOINT_NAMES, PRESETS_DEG, next_index

TOPIC = '/joint_group_effort_controller/joint_trajectory'


class PoseToggler(Node):
    """Publish the next pose preset on a fixed timer."""

    def __init__(self):
        super().__init__('pose_toggler')
        self.declare_parameter('period_s', 4.0)
        self.declare_parameter('sequence', ['stand', 'sit'])
        self.declare_parameter('move_time_s', 0.4)

        self.sequence = [
            name for name in self.get_parameter('sequence').value
            if name in PRESETS_DEG
        ] or ['stand', 'sit']
        self.move_time_s = float(self.get_parameter('move_time_s').value)
        period_s = float(self.get_parameter('period_s').value)

        self.index = -1
        self.publisher = self.create_publisher(JointTrajectory, TOPIC, 10)
        self.create_timer(period_s, self._tick)
        self.get_logger().info(
            f"Cycling {' -> '.join(self.sequence)} every {period_s:.1f}s")

    def _tick(self):
        self.index = next_index(self.index, len(self.sequence))
        name = self.sequence[self.index]
        msg = JointTrajectory()
        msg.joint_names = list(JOINT_NAMES)
        point = JointTrajectoryPoint()
        point.positions = [math.radians(v) for v in PRESETS_DEG[name]]
        sec = int(self.move_time_s)
        point.time_from_start = Duration(
            sec=sec, nanosec=int((self.move_time_s - sec) * 1e9))
        msg.points = [point]
        self.publisher.publish(msg)
        self.get_logger().info(f'-> {name}')


def main():
    rclpy.init()
    node = PoseToggler()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
