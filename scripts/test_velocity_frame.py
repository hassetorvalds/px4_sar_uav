#!/usr/bin/env python3
"""判定 PX4 速度设定点的坐标系解释：世界系（ENU→NED）还是机体系。

方法：读当前航向，选择一个与机体朝向**夹角最大**的世界轴（东或北）作为指令方向，
以固定速度飞数秒，比较实际位移方向与两种假设：

- 世界系假设：位移应沿被指令的世界轴；
- 机体系假设：位移应沿机体前方（= 当前航向）。

两者夹角至少 45°，因此结果可判别。前置：已起飞并处于 OFFBOARD。
"""

import math
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from px4_interface.frames import ned_to_enu, ned_yaw_to_enu_yaw
from px4_interface.state import Px4StateMonitor
from rclpy.node import Node


class VelocityFrameTest(Node):
    def __init__(self) -> None:
        super().__init__('test_velocity_frame')
        self.declare_parameter('speed_mps', 0.6)
        self.declare_parameter('duration_s', 5.0)
        self.declare_parameter('velocity_topic', '/offboard_bridge/velocity_setpoint')
        self.pub = self.create_publisher(
            Twist, str(self.get_parameter('velocity_topic').value), 10)
        self.monitor = Px4StateMonitor(self)

    def _spin(self, seconds: float, command=None) -> None:
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            if command is not None:
                twist = Twist()
                twist.linear.x, twist.linear.y, twist.linear.z = command
                self.pub.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.05)

    def _state_enu(self):
        state = self.monitor.snapshot()
        position = state.position_ned
        if position is None:
            return None
        return ned_to_enu(*position), ned_yaw_to_enu_yaw(state.heading)

    def run(self) -> int:
        deadline = time.monotonic() + 20.0
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.2)
            snapshot = self._state_enu()
            state = self.monitor.snapshot()
            if snapshot and state.armed and state.offboard:
                break
        else:
            print('前置条件不满足：需要已解锁且处于 OFFBOARD', file=sys.stderr)
            return 1

        (start, heading) = snapshot
        # 选择与航向夹角最大的世界轴：航向更接近北(90°)则指令"东"，否则指令"北"
        use_east = abs(math.sin(heading)) > abs(math.cos(heading))
        command = (1.0, 0.0, 0.0) if use_east else (0.0, 1.0, 0.0)
        axis_name = '东(+E)' if use_east else '北(+N)'
        print(f'起始 ENU=({start[0]:+.2f}, {start[1]:+.2f}, {start[2]:+.2f})，'
              f'航向 {math.degrees(heading):+.0f}°，指令方向 {axis_name} '
              f'{self.get_parameter("speed_mps").value} m/s')

        speed = float(self.get_parameter('speed_mps').value)
        self._spin(float(self.get_parameter('duration_s').value),
                   (command[0] * speed, command[1] * speed, 0.0))
        self._spin(1.0, (0.0, 0.0, 0.0))          # 停住

        end_state = self._state_enu()
        if end_state is None:
            print('结束时无法取得位置', file=sys.stderr)
            return 1
        (end, heading_end) = end_state
        de = end[0] - start[0]
        dn = end[1] - start[1]
        distance = math.hypot(de, dn)
        print(f'结束 ENU=({end[0]:+.2f}, {end[1]:+.2f}, {end[2]:+.2f})，'
              f'位移 Δe={de:+.2f} Δn={dn:+.2f}（{distance:.2f} m），'
              f'航向变为 {math.degrees(heading_end):+.0f}°')
        if distance < 0.3:
            print('位移过小，无法判定（检查是否真的在 OFFBOARD 速度控制下）')
            return 2

        move_dir = math.atan2(dn, de)
        axis_dir = 0.0 if use_east else math.pi / 2
        body_dir = heading

        def angle_diff(a, b):
            return abs(math.degrees(math.atan2(math.sin(a - b), math.cos(a - b))))

        world_err = angle_diff(move_dir, axis_dir)
        body_err = angle_diff(move_dir, body_dir)
        print(f'位移方向 {math.degrees(move_dir):+.0f}°；'
              f'与"世界轴"假设相差 {world_err:.0f}°，与"机体系"假设相差 {body_err:.0f}°')
        if world_err < body_err:
            print('判定：速度指令按**世界系(ENU→NED)**解释 ✅')
        else:
            print('判定：速度指令按**机体系(FRD)**解释 ⚠️（与当前实现假设不符）')
        return 0


def main() -> int:
    rclpy.init()
    node = VelocityFrameTest()
    try:
        return node.run()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
