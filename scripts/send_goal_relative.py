#!/usr/bin/env python3
"""按机体当前航向，把目标点设在前方/侧方（ENU），用于避障验证。

用法：
    python3 scripts/send_goal_relative.py --ros-args -p distance_m:=4.0 -p bearing_deg:=0.0
    - bearing_deg=0 表示正前方；+90 左、-90 右（相对机体航向）
    - altitude_delta_m 默认 0（保持当前高度）
"""

import math
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from px4_interface.frames import ned_to_enu, ned_yaw_to_enu_yaw
from px4_interface.state import Px4StateMonitor
from rclpy.node import Node


class GoalSender(Node):
    def __init__(self) -> None:
        super().__init__('send_goal_relative')
        self.declare_parameter('distance_m', 4.0)
        self.declare_parameter('bearing_deg', 0.0)
        self.declare_parameter('altitude_delta_m', 0.0)
        self.declare_parameter('goal_topic', '/local_planner/goal')
        self.pub = self.create_publisher(
            PoseStamped, str(self.get_parameter('goal_topic').value), 10)
        self.monitor = Px4StateMonitor(self)

    def run(self) -> int:
        deadline = time.monotonic() + 15.0
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.2)
            position_ned = self.monitor.snapshot().position_ned
            if position_ned is None:
                continue

            east, north, up = ned_to_enu(*position_ned)
            heading = ned_yaw_to_enu_yaw(self.monitor.snapshot().heading)
            bearing = heading + math.radians(float(self.get_parameter('bearing_deg').value))
            distance = float(self.get_parameter('distance_m').value)
            goal = (east + distance * math.cos(bearing),
                    north + distance * math.sin(bearing),
                    up + float(self.get_parameter('altitude_delta_m').value))

            msg = PoseStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'map'
            msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = goal
            msg.pose.orientation.w = 1.0
            self.pub.publish(msg)
            print(f'当前位置 ENU=({east:+.2f}, {north:+.2f}, {up:+.2f})，'
                  f'航向 {math.degrees(heading):+.0f}°，'
                  f'目标 ENU=({goal[0]:+.2f}, {goal[1]:+.2f}, {goal[2]:+.2f})')
            return 0
        print('未能取得有效位置样本', file=sys.stderr)
        return 1


def main() -> int:
    rclpy.init()
    node = GoalSender()
    try:
        return node.run()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
