#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool


class PoseToggler(Node):
    """Cycles Mini Pupper between SIT and STAND every 4 s."""
    def __init__(self, period_s: float = 4.0):
        super().__init__("pose_toggler")
        self.cmd_pub = self.create_publisher(String, "/robot_pose/change", 10)
        self.state_sub = self.create_subscription(
            Bool, "/robot_pose/is_standing", self._state_cb, 10)
        self.is_standing = False
        self.create_timer(period_s, self._tick)
        self.get_logger().info(f"Toggling every {period_s}s")

    def _state_cb(self, msg: Bool):
        self.is_standing = msg.data

    def _tick(self):
        target = "sit" if self.is_standing else "stand"
        self.cmd_pub.publish(String(data=target))
        self.get_logger().info(f"→ {target.upper()}")


def main():
    rclpy.init()
    node = PoseToggler()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()



if __name__ == "__main__":
    main()
