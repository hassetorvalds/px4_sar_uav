#!/usr/bin/env python3
"""同时采样 PX4 高度估计与 AirSim 真值，量化 EKF 高度漂移。

背景：降落阶段曾观察到“机体上向速度指令为正、PX4 的 z 却朝地面方向变化”，
以及 AUTO_LAND 下降速率仅约 0.04 m/s。两者都指向同一种可能：
**PX4 的高度估计与仿真真值不一致**。本脚本用 AirSim RPC 读取真值位姿，
与 /fmu/out/vehicle_local_position_v1 的 z 同步记录，给出两者的增量对比。

用法（需先启动 Agent、PX4，并让 offboard_bridge 运行）：
    source scripts/env.sh
    python3 scripts/compare_altitude.py --ros-args -p target_altitude_m:=3.0
"""

import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped
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


class AltitudeCompare(Node):
    def __init__(self) -> None:
        super().__init__('compare_altitude')
        self.declare_parameter('target_altitude_m', 3.0)
        self.declare_parameter('hold_s', 20.0)
        self.declare_parameter('sample_period_s', 0.5)
        self.declare_parameter('airsim_ip', '172.21.192.1')
        self.declare_parameter('airsim_port', 45000)

        self.altitude = float(self.get_parameter('target_altitude_m').value)
        self.hold_s = float(self.get_parameter('hold_s').value)
        self.period = float(self.get_parameter('sample_period_s').value)

        self.command_pub = self.create_publisher(String, '/offboard_bridge/command', 10)
        self.target_pub = self.create_publisher(PoseStamped, '/offboard_bridge/target_pose', 10)
        self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position_v1',
            self._on_position, QOS)
        self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status_v4', self._on_status, QOS)

        self.px4_z = None
        self.px4_vz = None
        self.armed = False
        self.nav_state = -1
        self.samples = []
        self.phase = 'WAIT'
        self.phase_started = time.monotonic()
        self.last_sample = 0.0

        try:
            import airsim

            self.client = airsim.MultirotorClient(
                ip=str(self.get_parameter('airsim_ip').value),
                port=int(self.get_parameter('airsim_port').value),
                timeout_value=15)
            self.client.confirmConnection()
            self.get_logger().info('AirSim RPC 已连接，将同时记录真值高度')
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f'AirSim 连接失败：{exc}')
            raise SystemExit(1)

        self.create_timer(0.2, self._tick)

    def _on_position(self, msg: VehicleLocalPosition) -> None:
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
        msg.pose.position.z = self.altitude
        msg.pose.orientation.w = 1.0
        self.target_pub.publish(msg)

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
        elif self.phase in ('CLIMB', 'HOLD'):
            self._publish_target()
            self._sample()
            if self.phase == 'CLIMB' and abs(self.px4_z + self.altitude) < 0.3:
                self.get_logger().info(f'到达 {self.altitude:.1f} m，开始对比采样')
                self._enter('HOLD')
            elif self.phase == 'HOLD' and elapsed > self.hold_s:
                self._finish()
            elif elapsed > 60:
                self._fail('等待高度超时')
        elif self.phase == 'DONE':
            pass

    def _enter(self, phase: str) -> None:
        self.phase = phase
        self.phase_started = time.monotonic()

    def _sample(self) -> None:
        now = time.monotonic()
        if now - self.last_sample < self.period:
            return
        self.last_sample = now
        pose = self.client.simGetVehiclePose().position
        self.samples.append((now, self.px4_z, self.px4_vz, pose.z_val))

    def _fail(self, reason: str) -> None:
        self.get_logger().error(f'中止：{reason}')
        self._finish()

    def _finish(self) -> None:
        print('\n============ 高度对比（PX4 估计 vs AirSim 真值）============')
        if not self.samples:
            print('无样本')
        else:
            t0 = self.samples[0][0]
            p0_px4 = self.samples[0][1]
            p0_sim = self.samples[0][3]
            print(f'{"t(s)":>6} {"PX4 z":>9} {"PX4 vz":>8} {"Sim z":>9} '
                  f'{"ΔPX4":>8} {"ΔSim":>8}')
            for t, z, vz, sim_z in self.samples:
                print(f'{t - t0:6.1f} {z:9.3f} {vz:8.3f} {sim_z:9.3f} '
                      f'{z - p0_px4:8.3f} {sim_z - p0_sim:8.3f}')
            px4_delta = self.samples[-1][1] - p0_px4
            sim_delta = self.samples[-1][3] - p0_sim
            print(f'\n期间变化量：PX4 Δz={px4_delta:+.3f} m，AirSim Δz={sim_delta:+.3f} m，'
                  f'差值={px4_delta - sim_delta:+.3f} m')
        raise SystemExit(0)


def main() -> None:
    rclpy.init()
    node = AltitudeCompare()
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
