"""A* 局部路径规划节点：真实 occupied 地图 → Path，不发布速度。

输入：`/local_map/occupied`、`/local_planner/goal`
输出：`~/path`（nav_msgs/Path，ENU/map）、`~/status`

安全边界：只规划，不控制飞控；速度/仲裁仍由现有执行器负责。
"""

import math
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from px4_interface.frames import ned_to_enu
from px4_interface.state import Px4StateMonitor
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String

from .astar import plan_path


def height_layer(points, altitude_m, min_above_m, max_above_m):
    if points.size == 0:
        return points
    keep = ((points[:, 2] >= altitude_m + min_above_m)
            & (points[:, 2] <= altitude_m + max_above_m))
    return points[keep]


class AstarPlanner(Node):
    def __init__(self) -> None:
        super().__init__('astar_planner')
        self.declare_parameter('occupied_topic', '/local_map/occupied')
        self.declare_parameter('goal_topic', '/local_planner/goal')
        self.declare_parameter('path_topic', '~/path')
        self.declare_parameter('resolution_m', 0.25)
        self.declare_parameter('inflate_m', 0.4)
        self.declare_parameter('max_radius_m', 12.0)
        self.declare_parameter('height_min_above_m', 0.3)
        self.declare_parameter('height_max_above_m', 1.5)
        self.declare_parameter('plan_rate_hz', 1.0)
        self.declare_parameter('map_timeout_s', 2.0)

        self._resolution = float(self.get_parameter('resolution_m').value)
        self._inflate = float(self.get_parameter('inflate_m').value)
        self._max_radius = float(self.get_parameter('max_radius_m').value)
        self._hmin = float(self.get_parameter('height_min_above_m').value)
        self._hmax = float(self.get_parameter('height_max_above_m').value)
        self._map_timeout = float(self.get_parameter('map_timeout_s').value)
        self._points = None
        self._map_at = 0.0
        self._goal = None
        self._monitor = Px4StateMonitor(self)

        self.create_subscription(PointCloud2, str(self.get_parameter('occupied_topic').value),
                                 self._on_cloud, 5)
        self.create_subscription(PoseStamped, str(self.get_parameter('goal_topic').value),
                                 self._on_goal, 5)
        self._path_pub = self.create_publisher(Path, str(self.get_parameter('path_topic').value), 5)
        self._status_pub = self.create_publisher(String, '~/status', 5)
        rate = max(float(self.get_parameter('plan_rate_hz').value), 0.2)
        self.create_timer(1.0 / rate, self._tick)

    def _on_cloud(self, msg):
        if msg.point_step != 12 or msg.width == 0:
            return
        self._points = np.frombuffer(msg.data, dtype=np.float32).reshape(-1, 3).astype(float)
        self._map_at = time.monotonic()

    def _on_goal(self, msg):
        p = msg.pose.position
        self._goal = (float(p.x), float(p.y))

    def _tick(self):
        if self._goal is None or self._points is None:
            return
        if time.monotonic() - self._map_at > self._map_timeout:
            self._status_pub.publish(String(data='map_stale'))
            return
        state = self._monitor.snapshot()
        if state.position_ned is None:
            return
        position = ned_to_enu(*state.position_ned)
        obstacles = height_layer(self._points, position[2], self._hmin, self._hmax)
        path = plan_path(obstacles, position[:2], self._goal,
                         resolution=self._resolution, inflate_m=self._inflate,
                         max_radius_m=self._max_radius)
        if path is None:
            self._status_pub.publish(String(
                data=f'no_path occupied={len(obstacles)} goal={self._goal}'))
            return
        msg = Path()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        for east, north in path:
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position.x = float(east)
            pose.pose.position.y = float(north)
            pose.pose.position.z = float(position[2])
            pose.pose.orientation.w = 1.0
            msg.poses.append(pose)
        self._path_pub.publish(msg)
        length = sum(math.hypot(path[i + 1][0] - path[i][0], path[i + 1][1] - path[i][1])
                     for i in range(len(path) - 1))
        self._status_pub.publish(String(
            data=f'path points={len(path)} length={length:.2f} occupied={len(obstacles)}'))


def main(args=None):
    rclpy.init(args=args)
    node = AstarPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
