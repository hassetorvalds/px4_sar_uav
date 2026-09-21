#!/usr/bin/env python3
"""打印一次 PX4 状态快照（等到位置有效再输出）。

`ros2 topic echo --once` 取到的是“最后一条”消息，EKF 未收敛或刚订阅时可能是
无效样本（曾连续读到 x=y=z=0.00），使飞行测试的位移测量不可信。
本脚本等待 xy_valid/z_valid 与状态消息齐备后输出一行结果。

用法：python3 scripts/px4_state_snapshot.py [超时秒数]
"""

import sys

import rclpy
from px4_msgs.msg import VehicleLocalPosition, VehicleStatus
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)

QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)


class Snapshot(Node):
    def __init__(self) -> None:
        super().__init__('px4_state_snapshot')
        self.position = None
        self.status = None
        self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position_v1',
            self._on_position, QOS)
        self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status_v4', self._on_status, QOS)

    def _on_position(self, msg: VehicleLocalPosition) -> None:
        if msg.xy_valid and msg.z_valid:
            self.position = msg

    def _on_status(self, msg: VehicleStatus) -> None:
        self.status = msg

    def ready(self) -> bool:
        return self.position is not None and self.status is not None


def main() -> int:
    timeout_s = float(sys.argv[1]) if len(sys.argv) > 1 else 15.0
    rclpy.init()
    node = Snapshot()
    start = node.get_clock().now().nanoseconds / 1e9
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.2)
            if node.ready():
                break
            if node.get_clock().now().nanoseconds / 1e9 - start > timeout_s:
                print('超时：未取得有效位置样本', file=sys.stderr)
                return 1
    finally:
        position, status = node.position, node.status
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    armed = status.arming_state == VehicleStatus.ARMING_STATE_ARMED
    print(f'x(N)={position.x:.2f} y(E)={position.y:.2f} z(D)={position.z:.2f} '
          f'vz={position.vz:.2f} armed={armed} nav={status.nav_state} '
          f'failsafe={status.failsafe}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
