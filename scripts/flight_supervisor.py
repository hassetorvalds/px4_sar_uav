#!/usr/bin/env python3
"""独立落地监督进程（服务器侧 2026-10-13 的安全建议 ②）。

背景：我们把 `land_fast` 写在驱动脚本的收尾里，结果**脚本自己崩溃时保护也一起没了**
（2026-10-13 实测：判别脚本汇总时抛异常，崩在自动降落之前，飞机没有自动落地）。
服务器侧建议把"落地保护"与"飞控/分析脚本"**解耦**，本进程就是那一层：

```text
谁在飞         驱动脚本（vlm_closed_loop.py / map_selfcheck.py）按 2 Hz 发心跳 /flight_heartbeat
监督什么       只要"已解锁"且心跳超过 timeout_s 没更新 → 立即下发 land_fast（并可重复）
独立性         与驱动脚本不同进程；驱动崩了、被 kill、卡死都能兜住
边界           `kill -9` 本进程 / 整机断电这类情况兜不住——那一层交给 PX4 的 failsafe
```

用法（起飞前启动，随栈常驻）：
    python3 scripts/flight_supervisor.py --ros-args -p timeout_s:=5.0
"""

import sys
import time

import rclpy
from px4_msgs.msg import VehicleStatus
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import String

QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)

sys.stdout.reconfigure(line_buffering=True)


class FlightSupervisor(Node):
    def __init__(self) -> None:
        super().__init__('flight_supervisor')
        self.declare_parameter('heartbeat_topic', '/flight_heartbeat')
        self.declare_parameter('command_topic', '/offboard_bridge/command')
        self.declare_parameter('status_topic', '/fmu/out/vehicle_status_v4')
        self.declare_parameter('timeout_s', 5.0)
        self.declare_parameter('repeat_s', 3.0)

        self.timeout_s = float(self.get_parameter('timeout_s').value)
        self.repeat_s = float(self.get_parameter('repeat_s').value)
        self.last_heartbeat = None
        self.armed = False
        self.last_land_sent = 0.0

        self.command_pub = self.create_publisher(
            String, str(self.get_parameter('command_topic').value), 10)
        self.create_subscription(
            String, str(self.get_parameter('heartbeat_topic').value),
            self._on_heartbeat, 10)
        self.create_subscription(
            VehicleStatus, str(self.get_parameter('status_topic').value),
            self._on_status, QOS)
        self.create_timer(1.0, self._tick)
        self.get_logger().info(
            f'落地监督启动：心跳 {self.get_parameter("heartbeat_topic").value}，'
            f'超过 {self.timeout_s:.0f} s 未更新且已解锁 → 下发 land_fast')

    def _on_heartbeat(self, msg: String) -> None:
        self.last_heartbeat = time.monotonic()

    def _on_status(self, msg: VehicleStatus) -> None:
        self.armed = msg.arming_state == VehicleStatus.ARMING_STATE_ARMED

    def _tick(self) -> None:
        if not self.armed:
            return
        age = (float('inf') if self.last_heartbeat is None
               else time.monotonic() - self.last_heartbeat)
        if age <= self.timeout_s:
            return
        now = time.monotonic()
        if now - self.last_land_sent < self.repeat_s:
            return
        self.last_land_sent = now
        msg = String()
        msg.data = 'land_fast'
        self.command_pub.publish(msg)
        self.get_logger().error(
            f'心跳已停 {age:.1f} s（> {self.timeout_s:.0f} s）且飞机已解锁 → '
            '监督层下发 land_fast')


def main() -> int:
    rclpy.init()
    node = FlightSupervisor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
