"""局部规划节点：目标点 + 前方净空 → 速度指令（含避让）。

输入：`~/goal`（geometry_msgs/PoseStamped，ENU/map 系）
      `/local_map/status`（String，含 `nearest_forward_m=`，来自 mapping/local_map）
      PX4 位姿（px4_interface 的 Px4StateMonitor）
输出：`/offboard_bridge/velocity_setpoint`（Twist，ENU 速度；偏航暂不控制）
      `~/status`（String：模式、净空、距目标距离）

安全边界：仅在 PX4 已解锁且处于 OFFBOARD 时发布速度指令；
地图状态超时（默认 1 s）时按“无净空信息”处理并降速（保守）。
"""

import re
import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from px4_interface.frames import ned_to_enu, ned_yaw_to_enu_yaw
from px4_interface.state import Px4StateMonitor
from rclpy.node import Node
from std_msgs.msg import String

from .avoidance import plan_velocity


class LocalPlanner(Node):
    def __init__(self) -> None:
        super().__init__('local_planner')
        self.declare_parameter('goal_topic', '~/goal')
        self.declare_parameter('map_status_topic', '/local_map/status')
        self.declare_parameter('velocity_topic', '/offboard_bridge/velocity_setpoint')
        self.declare_parameter('rate_hz', 10.0)
        self.declare_parameter('cruise_speed', 1.0)
        self.declare_parameter('slow_distance', 2.0)
        self.declare_parameter('stop_distance', 0.8)
        self.declare_parameter('goal_radius', 0.5)
        self.declare_parameter('avoid_side', -1)      # -1 右、+1 左
        self.declare_parameter('map_timeout_s', 1.0)
        self.declare_parameter('yaw_gain', 0.8)
        self.declare_parameter('max_yaw_rate', 0.6)

        self._map_timeout = float(self.get_parameter('map_timeout_s').value)
        self._yaw_gain = float(self.get_parameter('yaw_gain').value)
        self._max_yaw_rate = float(self.get_parameter('max_yaw_rate').value)
        self._params = {
            'cruise_speed': float(self.get_parameter('cruise_speed').value),
            'slow_distance': float(self.get_parameter('slow_distance').value),
            'stop_distance': float(self.get_parameter('stop_distance').value),
            'goal_radius': float(self.get_parameter('goal_radius').value),
            'avoid_side': int(self.get_parameter('avoid_side').value),
        }

        self._goal = None
        self._clearance = float('inf')
        self._left_clearance = float('inf')
        self._right_clearance = float('inf')
        self._clearance_at = 0.0
        self._last_avoid_side = 0

        self.create_subscription(PoseStamped, str(self.get_parameter('goal_topic').value),
                                 self._on_goal, 10)
        self.create_subscription(String, str(self.get_parameter('map_status_topic').value),
                                 self._on_map_status, 5)
        self._velocity_pub = self.create_publisher(
            Twist, str(self.get_parameter('velocity_topic').value), 10)
        self._status_pub = self.create_publisher(String, '~/status', 5)
        self._monitor = Px4StateMonitor(self)

        rate = max(float(self.get_parameter('rate_hz').value), 2.0)
        self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info(
            f'局部规划启动：巡航 {self._params["cruise_speed"]:.2f} m/s，'
            f'减速距离 {self._params["slow_distance"]:.1f} m，'
            f'停止距离 {self._params["stop_distance"]:.1f} m，'
            f'避让方向 {"左" if self._params["avoid_side"] > 0 else "右"}')

    # ------------------------------------------------------------------ 输入

    def _on_goal(self, msg: PoseStamped) -> None:
        p = msg.pose.position
        self._goal = (float(p.x), float(p.y), float(p.z))
        self.get_logger().info(f'新目标 ENU=({p.x:.2f}, {p.y:.2f}, {p.z:.2f})')

    def _on_map_status(self, msg: String) -> None:
        def parse(name: str) -> float:
            found = re.search(rf'{name}=([0-9.]+|inf)', msg.data)
            if not found:
                return float('inf')
            value = found.group(1)
            return float('inf') if value == 'inf' else float(value)

        self._clearance = parse('nearest_forward_m')
        self._left_clearance = parse('left_m')
        self._right_clearance = parse('right_m')
        self._clearance_at = time.monotonic()

    # ------------------------------------------------------------------ 主循环

    def _tick(self) -> None:
        state = self._monitor.snapshot()
        position_ned = state.position_ned
        if self._goal is None or position_ned is None:
            return
        if not (state.armed and state.offboard and not state.failsafe):
            self.get_logger().info(
                f'未处于可控制状态（armed={state.armed}, offboard={state.offboard}, '
                f'failsafe={state.failsafe}），暂不发布速度',
                throttle_duration_sec=5.0)
            return

        stale = (time.monotonic() - self._clearance_at) > self._map_timeout
        clearance = self._clearance
        if stale:
            # 地图信息过期：保守处理——按减速行为对待（不盲目巡航）
            clearance = min(self._params['slow_distance'],
                            self._params['stop_distance'] + 0.1)

        command = plan_velocity(
            self._goal, ned_to_enu(*position_ned), clearance_m=clearance,
            left_clearance=self._left_clearance,
            right_clearance=self._right_clearance,
            previous_avoid_side=self._last_avoid_side,
            **self._params)
        if command.mode == 'avoid':
            self._last_avoid_side = command.avoid_side_used

        twist = Twist()
        twist.linear.x, twist.linear.y, twist.linear.z = command.velocity_enu
        # 机头对准目标方向：净空锥（相机）与行进方向一致，避障判断才有意义
        heading_enu = ned_yaw_to_enu_yaw(state.heading)
        bearing = math.atan2(self._goal[1] - ned_to_enu(*position_ned)[1],
                             self._goal[0] - ned_to_enu(*position_ned)[0])
        error = math.atan2(math.sin(bearing - heading_enu), math.cos(bearing - heading_enu))
        twist.angular.z = max(-self._max_yaw_rate,
                              min(self._max_yaw_rate, self._yaw_gain * error))
        self._velocity_pub.publish(twist)

        status = String()
        status.data = (f'mode={command.mode} clearance_m={clearance:.2f} '
                       f'distance_m={command.distance_to_goal:.2f} stale={stale} '
                       f'reason={command.reason}')
        self._status_pub.publish(status)
        # 记录模式切换序列（而非仅打印某个模式），便于回放避让→巡航的完整过程
        if command.mode != getattr(self, '_last_mode', None):
            self.get_logger().warn(
                f'模式切换 → {command.mode}（净空 {clearance:.2f} m，'
                f'距目标 {command.distance_to_goal:.2f} m）{command.reason}')
            self._last_mode = command.mode


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LocalPlanner()
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
