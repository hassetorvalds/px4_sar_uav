#!/usr/bin/env python3
"""降落诊断：起飞到指定高度后命令降落，逐周期记录落地检测标志与高度。

同时计算**仿真时间倍率**（PX4 时间戳推进 / 墙钟时间），
用于判断 LockStep + SteppableClock 是否在实时运行。

用法（需先启动 Agent 与 PX4 SITL）：
    source scripts/env.sh
    python3 scripts/landing_diagnose.py --ros-args -p target_altitude_m:=3.0
"""

import math
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from px4_msgs.msg import (
    VehicleLandDetected,
    VehicleLocalPosition,
    VehicleStatus,
)
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


class LandingDiagnose(Node):
    def __init__(self) -> None:
        super().__init__('landing_diagnose')
        self.declare_parameter('target_altitude_m', 3.0)
        self.declare_parameter('arm_timeout_s', 20.0)
        self.declare_parameter('takeoff_timeout_s', 40.0)
        self.declare_parameter('land_timeout_s', 30.0)
        self.declare_parameter('land_command', 'land_fast')
        self.declare_parameter('sample_period_s', 0.5)
        self.declare_parameter('csv_path', '/tmp/landing_trace.csv')
        self.declare_parameter('command_topic', '/offboard_bridge/command')
        self.declare_parameter('target_topic', '/offboard_bridge/target_pose')

        self.altitude = float(self.get_parameter('target_altitude_m').value)
        self.arm_timeout = float(self.get_parameter('arm_timeout_s').value)
        self.takeoff_timeout = float(self.get_parameter('takeoff_timeout_s').value)
        self.land_timeout = float(self.get_parameter('land_timeout_s').value)
        self.land_command = str(self.get_parameter('land_command').value)
        self.sample_period = float(self.get_parameter('sample_period_s').value)
        self.csv_path = str(self.get_parameter('csv_path').value)

        self.command_pub = self.create_publisher(
            String, str(self.get_parameter('command_topic').value), 10)
        self.target_pub = self.create_publisher(
            PoseStamped, str(self.get_parameter('target_topic').value), 10)
        self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position_v1',
            self._on_position, QOS)
        self.create_subscription(
            VehicleLandDetected, '/fmu/out/vehicle_land_detected',
            self._on_land, QOS)
        self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status_v4', self._on_status, QOS)

        self.phase = 'WAIT'
        self.phase_started = time.monotonic()
        self.z = float('nan')
        self.vz = float('nan')
        self.position_stamp_us = 0
        self.landed = False
        self.armed = False
        self.nav_state = -1
        self.failsafe = False
        self.preflight_ok = False
        self.flags = {}

        self.samples = []
        self.last_sample = 0.0
        self.takeoff_z_min = float('inf')
        self.rtf_pairs = []
        self.last_rt = None
        self.land_sent_at = 0.0
        self.csv_written = False

        self.create_timer(0.2, self._tick)
        self.get_logger().info(
            f'降落诊断启动：目标高度 {self.altitude:.1f} m，'
            f'跟踪文件 {self.csv_path}')

    # ---------------------------------------------------------------- 回调

    def _on_position(self, msg: VehicleLocalPosition) -> None:
        now = time.monotonic()
        if self.last_rt is not None and msg.timestamp > self.position_stamp_us:
            sim_delta = (msg.timestamp - self.position_stamp_us) / 1e6
            wall_delta = now - self.last_rt
            if wall_delta > 1e-3 and sim_delta < 5.0:
                self.rtf_pairs.append(sim_delta / wall_delta)
                self.rtf_pairs = self.rtf_pairs[-200:]
        self.last_rt = now
        self.position_stamp_us = msg.timestamp
        self.z = msg.z
        self.vz = msg.vz

    def _on_land(self, msg: VehicleLandDetected) -> None:
        self.landed = msg.landed
        self.flags = {
            'landed': msg.landed,
            'maybe_landed': msg.maybe_landed,
            'ground_contact': msg.ground_contact,
            'at_rest': msg.at_rest,
            'has_low_throttle': msg.has_low_throttle,
            'in_ground_effect': msg.in_ground_effect,
            'in_descend': msg.in_descend,
            'close_to_ground': msg.close_to_ground_or_skipped_check,
        }

    def _on_status(self, msg: VehicleStatus) -> None:
        self.armed = msg.arming_state == VehicleStatus.ARMING_STATE_ARMED
        self.nav_state = msg.nav_state
        self.failsafe = msg.failsafe
        self.preflight_ok = msg.pre_flight_checks_pass

    # ---------------------------------------------------------------- 主循环

    def _tick(self) -> None:
        elapsed = time.monotonic() - self.phase_started
        if self.phase == 'WAIT':
            if self.preflight_ok and not math.isnan(self.z):
                self.get_logger().info('自检通过，开始解锁')
                self._enter('ARM')
        elif self.phase == 'ARM':
            self._send('arm')
            if self.armed:
                self.get_logger().info('已解锁，进入 OFFBOARD')
                self._enter('OFFBOARD')
            elif elapsed > self.arm_timeout:
                self._fail('解锁超时')
        elif self.phase == 'OFFBOARD':
            self._send('offboard')
            if self.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
                self.get_logger().info('已进入 OFFBOARD，开始爬升')
                self._enter('TAKEOFF')
            elif elapsed > self.arm_timeout:
                self._fail('进入 OFFBOARD 超时')
        elif self.phase == 'TAKEOFF':
            self._publish_target(self.altitude)
            self.takeoff_z_min = min(self.takeoff_z_min, self.z)
            if abs(self.z + self.altitude) < 0.4:
                self.get_logger().info(
                    f'到达目标高度 z={self.z:.2f} m，命令降落（{self.land_command}）')
                self._send(self.land_command)
                self.land_sent_at = time.monotonic()
                self._enter('LAND')
            elif elapsed > self.takeoff_timeout:
                self._fail('爬升超时')
        elif self.phase == 'LAND':
            if abs(self.nav_state - VehicleStatus.NAVIGATION_STATE_AUTO_LAND) < 3 \
                    and elapsed < 2.0 and int(elapsed * 5) % 5 == 0:
                self._send('land')  # 保持降落意图
            self._sample()
            if self.landed or not self.armed:
                self._enter('DONE')
            elif time.monotonic() - self.land_sent_at > self.land_timeout:
                self.get_logger().error('降落超时，仍未落地')
                self._enter('DONE')
        elif self.phase == 'DONE':
            self._finish()

    # ---------------------------------------------------------------- 工具

    def _enter(self, phase: str) -> None:
        self.phase = phase
        self.phase_started = time.monotonic()

    def _send(self, command: str) -> None:
        msg = String()
        msg.data = command
        self.command_pub.publish(msg)

    def _publish_target(self, altitude: float) -> None:
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position.z = float(altitude)
        msg.pose.orientation.w = 1.0
        self.target_pub.publish(msg)

    def _sample(self) -> None:
        now = time.monotonic()
        if now - self.last_sample < self.sample_period:
            return
        self.last_sample = now
        self.samples.append((now - self.land_sent_at, self.z, self.vz, dict(self.flags)))

    def _fail(self, reason: str) -> None:
        self.get_logger().error(f'诊断中止：{reason}')
        self._enter('DONE')

    def _finish(self) -> None:
        if not self.csv_written:
            self.csv_written = True
            try:
                with open(self.csv_path, 'w', encoding='utf-8') as handle:
                    handle.write('t_s,z_m,vz_mps,' + ','.join(
                        ['landed', 'maybe_landed', 'ground_contact', 'at_rest',
                         'has_low_throttle', 'in_ground_effect', 'in_descend',
                         'close_to_ground']) + '\n')
                    for t_s, z, vz, flags in self.samples:
                        row = [f'{t_s:.2f}', f'{z:.3f}', f'{vz:.3f}'] + [
                            str(flags.get(key, '')).lower() for key in
                            ('landed', 'maybe_landed', 'ground_contact', 'at_rest',
                             'has_low_throttle', 'in_ground_effect', 'in_descend',
                             'close_to_ground')]
                        handle.write(','.join(row) + '\n')
            except OSError as exc:
                self.get_logger().warn(f'CSV 写入失败：{exc}')

        rtf = (sum(self.rtf_pairs) / len(self.rtf_pairs)) if self.rtf_pairs else float('nan')
        lowest = min((s[1] for s in self.samples), default=float('nan'))
        print('\n================ 降落诊断结果 ================')
        print(f'仿真时间倍率(RTF)        : {rtf:.3f}  (>1 表示仿真快于墙钟)')
        print(f'降落阶段采样点           : {len(self.samples)}')
        print(f'降落阶段最低高度 z       : {lowest:.3f} m')
        print(f'最终 landed/armed        : {self.landed} / {self.armed}')
        print(f'最终 nav_state           : {self.nav_state}')
        print(f'最终落地检测标志         : {self.flags}')
        print(f'轨迹 CSV                 : {self.csv_path}')
        print('=============================================')
        raise SystemExit(0)


def main() -> None:
    rclpy.init()
    node = LandingDiagnose()
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
