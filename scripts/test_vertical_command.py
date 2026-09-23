#!/usr/bin/env python3
"""纯竖直速度指令测试：确认 ENU 上向速度与高度变化的符号一致。

背景：VLM 闭环中出现“机体上向指令 +0.11 m/s，PX4 高度却下降 0.63 m”。
高度真值对比已证明正常飞行段 PX4 估计可信（差 5 mm），因此需要单独检查
速度指令链路的符号方向：ENU 速度 → offboard_bridge → PX4 NED 设定点。

流程：起飞到指定高度 → 只发上向速度 → 只发下向速度 → 比较 PX4 与真值的高度变化。
用法（需先启动 Agent、PX4、offboard_bridge）：
    python3 scripts/test_vertical_command.py --ros-args -p command_speed_mps:=0.5
"""

import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from px4_msgs.msg import VehicleLocalPosition, VehicleStatus
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


class VerticalCommandTest(Node):
    def __init__(self) -> None:
        super().__init__('test_vertical_command')
        self.declare_parameter('start_altitude_m', 2.0)
        self.declare_parameter('command_speed_mps', 0.5)
        self.declare_parameter('phase_s', 6.0)
        self.declare_parameter('airsim_ip', '172.21.192.1')
        self.declare_parameter('airsim_port', 45000)

        self.start_altitude = float(self.get_parameter('start_altitude_m').value)
        self.speed = float(self.get_parameter('command_speed_mps').value)
        self.phase_s = float(self.get_parameter('phase_s').value)

        self.command_pub = self.create_publisher(String, '/offboard_bridge/command', 10)
        self.target_pub = self.create_publisher(PoseStamped, '/offboard_bridge/target_pose', 10)
        self.velocity_pub = self.create_publisher(
            Twist, '/offboard_bridge/velocity_setpoint', 10)
        self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position_v1',
            self._on_position, QOS)
        self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status_v4', self._on_status, QOS)

        self.px4_z = None
        self.px4_vz = None
        self.armed = False
        self.nav_state = -1
        self.phase = 'WAIT'
        self.phase_started = time.monotonic()
        self.marks = {}

        try:
            import airsim

            self.client = airsim.MultirotorClient(
                ip=str(self.get_parameter('airsim_ip').value),
                port=int(self.get_parameter('airsim_port').value),
                timeout_value=15)
            self.client.confirmConnection()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f'AirSim 连接失败：{exc}')
            raise SystemExit(1)

        self.create_timer(0.1, self._tick)
        self.get_logger().info(
            f'竖直指令测试：起始高度 {self.start_altitude:.1f} m，'
            f'速度 ±{self.speed:.2f} m/s，每段 {self.phase_s:.0f} s')

    def _on_position(self, msg: VehicleLocalPosition) -> None:
        if msg.xy_valid and msg.z_valid:
            self.px4_z = msg.z
            self.px4_vz = msg.vz

    def _on_status(self, msg: VehicleStatus) -> None:
        self.armed = msg.arming_state == VehicleStatus.ARMING_STATE_ARMED
        self.nav_state = msg.nav_state

    def _send(self, command: str) -> None:
        msg = String()
        msg.data = command
        self.command_pub.publish(msg)

    def _publish_target(self) -> None:
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position.z = self.start_altitude
        msg.pose.orientation.w = 1.0
        self.target_pub.publish(msg)

    def _publish_up_velocity(self, up_mps: float) -> None:
        msg = Twist()
        msg.linear.z = up_mps          # ENU：+z 向上
        self.velocity_pub.publish(msg)

    def _mark(self, name: str) -> None:
        pose = self.client.simGetVehiclePose().position
        self.marks[name] = (self.px4_z, self.px4_vz, pose.z_val)

    def _tick(self) -> None:
        elapsed = time.monotonic() - self.phase_started
        if self.phase == 'WAIT':
            if self.px4_z is not None:
                self._send('arm')
                self._enter('ARM')
        elif self.phase == 'ARM':
            self._send('arm')
            if self.armed:
                self._send('offboard')
                self._enter('OFFBOARD')
            elif elapsed > 15:
                self._fail('解锁超时')
        elif self.phase == 'OFFBOARD':
            self._send('offboard')
            if self.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
                self._enter('CLIMB')
            elif elapsed > 15:
                self._fail('进入 OFFBOARD 超时')
        elif self.phase == 'CLIMB':
            self._publish_target()
            if abs(self.px4_z + self.start_altitude) < 0.25:
                self.get_logger().info(f'到达起始高度 z={self.px4_z:.2f} m')
                time.sleep(1.0)
                self._mark('before_up')
                self._enter('UP')
            elif elapsed > 40:
                self._fail('爬升超时')
        elif self.phase == 'UP':
            self._publish_up_velocity(self.speed)
            if elapsed > self.phase_s:
                self._mark('after_up')
                # 回到原高度再测下向，避免高度一直漂
                self._publish_target()
                self._enter('RECENTER')
        elif self.phase == 'RECENTER':
            self._publish_target()
            if abs(self.px4_z + self.start_altitude) < 0.25:
                time.sleep(1.0)
                self._mark('before_down')
                self._enter('DOWN')
            elif elapsed > 30:
                self.get_logger().warn('回中高度未达成，仍继续下向测试')
                self._mark('before_down')
                self._enter('DOWN')
        elif self.phase == 'DOWN':
            self._publish_up_velocity(-self.speed)
            if elapsed > self.phase_s:
                self._mark('after_down')
                self._finish()
        elif self.phase == 'DONE':
            pass

    def _enter(self, phase: str) -> None:
        self.phase = phase
        self.phase_started = time.monotonic()
        self.get_logger().info(f'→ 阶段 {phase}')

    def _fail(self, reason: str) -> None:
        self.get_logger().error(f'中止：{reason}')
        self._finish()

    def _finish(self) -> None:
        def delta(a, b, idx):
            if a not in self.marks or b not in self.marks:
                return float('nan')
            return self.marks[b][idx] - self.marks[a][idx]

        up_px4 = delta('before_up', 'after_up', 0)
        up_sim = delta('before_up', 'after_up', 2)
        down_px4 = delta('before_down', 'after_down', 0)
        down_sim = delta('before_down', 'after_down', 2)

        print('\n========== 竖直速度指令方向测试 ==========')
        print(f'指令：上向 {self.speed:+.2f} m/s，持续 {self.phase_s:.0f} s')
        print(f'  高度变化：PX4 Δz={up_px4:+.3f} m，真值 Δz={up_sim:+.3f} m')
        print(f'  期望：Δz 为负（NED 向下为正，爬升即 z 减小）')
        print(f'指令：下向 {-self.speed:+.2f} m/s，持续 {self.phase_s:.0f} s')
        print(f'  高度变化：PX4 Δz={down_px4:+.3f} m，真值 Δz={down_sim:+.3f} m')
        print(f'  期望：Δz 为正（下降）')
        ok_up = up_px4 < -0.2 and up_sim < -0.2
        ok_down = down_px4 > 0.2 and down_sim > 0.2
        print(f'\n结论：上向 {"一致" if ok_up else "不一致"}，'
              f'下向 {"一致" if ok_down else "不一致"}')
        print('=========================================')
        raise SystemExit(0)


def main() -> None:
    rclpy.init()
    node = VerticalCommandTest()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
