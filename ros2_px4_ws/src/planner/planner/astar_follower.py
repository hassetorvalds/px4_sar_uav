"""A* 路径跟随节点（默认关闭，只输出速度候选，不接 offboard）。

输入：/astar_planner/path、PX4 位姿
输出：~/velocity_candidate（geometry_msgs/Twist）、~/status

enabled=false 时绝不发布速度候选；即使 enabled=true，也只发布候选，
不会直接写 /offboard_bridge/velocity_setpoint。
"""

import time

import rclpy
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
import re

from px4_interface.frames import ned_to_enu, ned_yaw_to_enu_yaw
from px4_interface.state import Px4StateMonitor
from rclpy.node import Node
from std_msgs.msg import String

from .candidate_safety import gate_candidate
from .path_follow import follow_velocity


class AstarFollower(Node):
    def __init__(self) -> None:
        super().__init__('astar_follower')
        self.declare_parameter('path_topic', '/astar_planner/path')
        self.declare_parameter('velocity_topic', '~/velocity_candidate')
        self.declare_parameter('enabled', False)
        self.declare_parameter('rate_hz', 10.0)
        self.declare_parameter('max_speed_mps', 0.4)
        self.declare_parameter('gain', 0.8)
        self.declare_parameter('tolerance_m', 0.4)
        self.declare_parameter('lookahead_m', 0.5)
        self.declare_parameter('path_timeout_s', 2.0)
        self.declare_parameter('map_status_topic', '/local_map/status')
        self.declare_parameter('stop_m', 0.6)
        self._enabled = bool(self.get_parameter('enabled').value)
        self._max_speed = float(self.get_parameter('max_speed_mps').value)
        self._gain = float(self.get_parameter('gain').value)
        self._tolerance = float(self.get_parameter('tolerance_m').value)
        self._lookahead = float(self.get_parameter('lookahead_m').value)
        self._timeout = float(self.get_parameter('path_timeout_s').value)
        self._stop = float(self.get_parameter('stop_m').value)
        self._forward = None
        self._left = None
        self._right = None
        self._left_known = False
        self._right_known = False
        self._path = []
        self._path_at = 0.0
        self._monitor = Px4StateMonitor(self)
        self.create_subscription(String, str(self.get_parameter('map_status_topic').value),
                                 self._on_map, 5)
        self.create_subscription(Path, str(self.get_parameter('path_topic').value),
                                 self._on_path, 5)
        self._pub = self.create_publisher(
            Twist, str(self.get_parameter('velocity_topic').value), 5)
        self._status_pub = self.create_publisher(String, '~/status', 5)
        rate = max(float(self.get_parameter('rate_hz').value), 2.0)
        self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info(f'astar_follower enabled={self._enabled}（默认只发候选）')

    def _on_path(self, msg):
        self._path = [(float(p.pose.position.x), float(p.pose.position.y))
                      for p in msg.poses]
        self._path_at = time.monotonic()

    def _on_map(self, msg):
        def value(name):
            found = re.search(rf'{name}=([0-9.]+|inf)', msg.data)
            if not found:
                return None
            return float('inf') if found.group(1) == 'inf' else float(found.group(1))
        self._forward = value('nearest_forward_m')
        self._left = value('left_m')
        self._right = value('right_m')
        valid = re.search(r'left_valid=([01])\s+right_valid=([01])', msg.data)
        if valid:
            self._left_known = valid.group(1) == '1'
            self._right_known = valid.group(2) == '1'

    def _tick(self):
        if not self._enabled:
            self._status_pub.publish(String(data='disabled'))
            return
        if not self._path or time.monotonic() - self._path_at > self._timeout:
            self._status_pub.publish(String(data='no_path'))
            return
        state = self._monitor.snapshot()
        if state.position_ned is None:
            return
        position = ned_to_enu(*state.position_ned)
        command = follow_velocity(position, self._path,
                                  max_speed_mps=self._max_speed,
                                  gain=self._gain,
                                  tolerance_m=self._tolerance,
                                  lookahead_m=self._lookahead)
        heading = ned_yaw_to_enu_yaw(state.heading) if state.heading is not None else 0.0
        safe = gate_candidate(command.east, command.north, heading_rad=heading,
                              forward_m=self._forward, left_m=self._left,
                              right_m=self._right, stop_m=self._stop,
                              left_known=self._left_known,
                              right_known=self._right_known)
        twist = Twist()
        twist.linear.x = safe.east if not safe.blocked else 0.0
        twist.linear.y = safe.north if not safe.blocked else 0.0
        self._pub.publish(twist)
        self._status_pub.publish(String(
            data=f'candidate east={twist.linear.x:.2f} north={twist.linear.y:.2f} '
                 f'index={command.index} done={command.done} '
                 f'blocked={safe.blocked} reason={safe.reason}'))


def main(args=None):
    rclpy.init(args=args)
    node = AstarFollower()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
