"""自主探索节点：在未知区域中选择 frontier 作为下一个观察点。

输入：`/local_map/occupied`（PointCloud2，ENU 占据体素）+ PX4 位姿
输出：`/local_planner/goal`（PoseStamped，ENU）与 `~/status`（String）

行为：

- 周期性把占据点云与机体位置喂给 FrontierMap；
- 选下一个 frontier 作为目标发布给局部规划器；
- 当前目标在 `goal_timeout_s` 内未到达则重新选点；
- 连续 `complete_after_s` 没有可用 frontier → 判定探索完成并停止发目标。

安全边界：仅在 PX4 已解锁且处于 OFFBOARD 时发布目标。
"""

import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from px4_interface.frames import ned_to_enu
from px4_interface.state import Px4StateMonitor
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String

from .frontier import FrontierMap


class FrontierExplorer(Node):
    def __init__(self) -> None:
        super().__init__('frontier_explorer')
        self.declare_parameter('occupied_topic', '/local_map/occupied')
        self.declare_parameter('goal_topic', '/local_planner/goal')
        self.declare_parameter('resolution', 0.5)
        self.declare_parameter('free_radius', 3.0)
        self.declare_parameter('max_goal_distance', 12.0)
        self.declare_parameter('goal_timeout_s', 40.0)
        self.declare_parameter('complete_after_s', 12.0)
        self.declare_parameter('visited_radius_m', 1.5)
        self.declare_parameter('plan_rate_hz', 1.0)
        self.declare_parameter('explore_altitude_m', 2.0)

        self._grid = FrontierMap(
            resolution=float(self.get_parameter('resolution').value),
            free_radius=float(self.get_parameter('free_radius').value))
        self._max_goal = float(self.get_parameter('max_goal_distance').value)
        self._goal_timeout = float(self.get_parameter('goal_timeout_s').value)
        self._complete_after = float(self.get_parameter('complete_after_s').value)
        self._altitude = float(self.get_parameter('explore_altitude_m').value)

        self._goal = None
        self._goal_set_at = 0.0
        self._no_frontier_since = None
        self._complete = False
        self._visited = []          # 已选过的探索目标（避免反复挑同一处）
        self._visited_limit = 20

        self.create_subscription(
            PointCloud2, str(self.get_parameter('occupied_topic').value),
            self._on_occupied, 5)
        self._goal_pub = self.create_publisher(
            PoseStamped, str(self.get_parameter('goal_topic').value), 10)
        self._status_pub = self.create_publisher(String, '~/status', 5)
        self._monitor = Px4StateMonitor(self)

        rate = max(float(self.get_parameter('plan_rate_hz').value), 0.2)
        self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info(
            f'探索器启动：分辨率 {self._grid.resolution:.2f} m，'
            f'已知半径 {self._grid.free_radius:.1f} m，'
            f'最大目标距离 {self._max_goal:.0f} m，'
            f'目标超时 {self._goal_timeout:.0f} s，'
            f'无 frontier {self._complete_after:.0f} s 判定完成')

    # ------------------------------------------------------------------ 输入

    def _on_occupied(self, msg: PointCloud2) -> None:
        if msg.width == 0 or msg.point_step != 12:
            return
        points = np.frombuffer(msg.data, dtype=np.float32).reshape(-1, 3)
        self._grid.mark_occupied(points)

    # ------------------------------------------------------------------ 主循环

    def _tick(self) -> None:
        state = self._monitor.snapshot()
        position_ned = state.position_ned
        if position_ned is None:
            return
        position_enu = ned_to_enu(*position_ned)
        if not (state.armed and state.offboard and not state.failsafe):
            self.get_logger().info('未处于可控制状态，暂停探索目标发布',
                                   throttle_duration_sec=5.0)
            return

        self._grid.mark_free_around(position_enu)
        now = time.monotonic()

        if self._complete:
            self._publish_status('complete')
            return

        need_new_goal = self._goal is None or (now - self._goal_set_at) > self._goal_timeout
        if self._goal is not None and math_close(self._goal, position_enu, 1.0):
            need_new_goal = True

        if need_new_goal:
            candidate = self._grid.best_frontier(
                position_enu, self._max_goal, visited=self._visited,
                visited_radius=float(self.get_parameter('visited_radius_m').value))
            if candidate is None:
                if self._no_frontier_since is None:
                    self._no_frontier_since = now
                elif now - self._no_frontier_since > self._complete_after:
                    self._complete = True
                    self.get_logger().info(
                        f'探索完成：无可用 frontier（已知 {self._grid.free_count} 格，'
                        f'障碍 {self._grid.occupied_count} 格）')
                self._goal = None
                self._publish_status('无可用 frontier')
                return
            self._no_frontier_since = None
            self._goal = candidate
            self._goal_set_at = now
            self._visited.append(candidate)
            if len(self._visited) > self._visited_limit:
                self._visited.pop(0)
            self.get_logger().info(
                f'新探索目标 ENU=({candidate[0]:+.2f}, {candidate[1]:+.2f}, '
                f'{self._altitude:+.2f})，候选区域数 {len(self._grid.clusters())}，'
                f'已访问目标 {len(self._visited)} 处')

        self._publish_goal()
        self._publish_status('exploring')

    # ------------------------------------------------------------------ 输出

    def _publish_goal(self) -> None:
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position.x = float(self._goal[0])
        msg.pose.position.y = float(self._goal[1])
        msg.pose.position.z = self._altitude
        msg.pose.orientation.w = 1.0
        self._goal_pub.publish(msg)

    def _publish_status(self, stage: str) -> None:
        msg = String()
        goal = 'none' if self._goal is None else \
            f'({self._goal[0]:.1f},{self._goal[1]:.1f})'
        msg.data = (f'stage={stage} goal={goal} '
                    f'clusters={len(self._grid.clusters())} '
                    f'free={self._grid.free_count} occupied={self._grid.occupied_count}')
        self._status_pub.publish(msg)


def math_close(a, b, tolerance: float) -> bool:
    return ((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2) ** 0.5 <= tolerance


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FrontierExplorer()
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
