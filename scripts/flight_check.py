#!/usr/bin/env python3
"""飞控诊断合集（原 compare_altitude.py 与 test_vertical_command.py 合并）。

用法：
    python3 scripts/flight_check.py altitude --ros-args -p hold_s:=20.0
        → 对比 PX4 高度估计与 AirSim 真值，量化估计器偏差
    python3 scripts/flight_check.py vertical --ros-args -p command_speed_mps:=0.5
        → 只发上/下速度指令，检查指令链路的符号方向

前置：Agent、PX4 SITL、offboard_bridge 已启动。
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


class FlightCheckBase(Node):
    """公共部分：状态订阅、指令发布、AirSim 真值读取、阶段机。"""

    def __init__(self, name: str) -> None:
        super().__init__(name)
        self.declare_parameter('airsim_ip', '172.21.192.1')
        self.declare_parameter('airsim_port', 45000)
        self.declare_parameter('phase_timeout_s', 40.0)

        self.phase_timeout = float(self.get_parameter('phase_timeout_s').value)
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

        try:
            import airsim

            self.client = airsim.MultirotorClient(
                ip=str(self.get_parameter('airsim_ip').value),
                port=int(self.get_parameter('airsim_port').value),
                timeout_value=15)
            self.client.confirmConnection()
            self.get_logger().info('AirSim RPC 已连接')
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f'AirSim 连接失败：{exc}')
            raise SystemExit(1)

        self.create_timer(0.1, self._tick)

    # ------------------------------------------------------------- 回调

    def _on_position(self, msg: VehicleLocalPosition) -> None:
        if msg.xy_valid and msg.z_valid:
            self.px4_z = msg.z
            self.px4_vz = msg.vz

    def _on_status(self, msg: VehicleStatus) -> None:
        self.armed = msg.arming_state == VehicleStatus.ARMING_STATE_ARMED
        self.nav_state = msg.nav_state

    # ------------------------------------------------------------- 工具

    def elapsed(self) -> float:
        return time.monotonic() - self.phase_started

    def enter(self, phase: str) -> None:
        self.phase = phase
        self.phase_started = time.monotonic()
        self.get_logger().info(f'→ 阶段 {phase}')

    def send(self, command: str) -> None:
        msg = String()
        msg.data = command
        self.command_pub.publish(msg)

    def publish_target(self, altitude: float) -> None:
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position.z = float(altitude)
        msg.pose.orientation.w = 1.0
        self.target_pub.publish(msg)

    def publish_up_velocity(self, up_mps: float) -> None:
        msg = Twist()
        msg.linear.z = float(up_mps)
        self.velocity_pub.publish(msg)

    def sim_z(self) -> float:
        return self.client.simGetVehiclePose().position.z_val

    def climb_to(self, altitude: float) -> bool:
        """位置模式下爬到指定高度；返回是否到达。"""
        self.publish_target(altitude)
        if self.px4_z is not None and abs(self.px4_z + altitude) < 0.25:
            time.sleep(1.0)
            return True
        return False

    def _tick(self) -> None:
        """公共启动流程：WAIT → ARM → OFFBOARD → 交给子类。"""
        if self.phase == 'WAIT':
            if self.px4_z is not None:
                self.send('arm')
                self.enter('ARM')
        elif self.phase == 'ARM':
            self.send('arm')
            if self.armed:
                self.send('offboard')
                self.enter('OFFBOARD')
            elif self.elapsed() > 15:
                self.fail('解锁超时')
        elif self.phase == 'OFFBOARD':
            self.send('offboard')
            if self.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
                self.enter(self.first_phase())
            elif self.elapsed() > 15:
                self.fail('进入 OFFBOARD 超时')
        else:
            self.on_phase()

    def first_phase(self) -> str:
        raise NotImplementedError

    def on_phase(self) -> None:
        raise NotImplementedError

    def fail(self, reason: str) -> None:
        self.get_logger().error(f'中止：{reason}')
        self.finish()

    def finish(self) -> None:
        raise NotImplementedError


class AltitudeCheck(FlightCheckBase):
    """对比 PX4 高度估计与仿真真值。"""

    def __init__(self) -> None:
        super().__init__('flight_check_altitude')
        self.declare_parameter('target_altitude_m', 3.0)
        self.declare_parameter('hold_s', 20.0)
        self.declare_parameter('sample_period_s', 0.5)
        self.altitude = float(self.get_parameter('target_altitude_m').value)
        self.hold_s = float(self.get_parameter('hold_s').value)
        self.period = float(self.get_parameter('sample_period_s').value)
        self.samples = []
        self.last_sample = 0.0

    def first_phase(self) -> str:
        return 'CLIMB'

    def on_phase(self) -> None:
        if self.phase == 'CLIMB':
            if self.climb_to(self.altitude):
                self.enter('HOLD')
            elif self.elapsed() > self.phase_timeout:
                self.fail('爬升超时')
        elif self.phase == 'HOLD':
            self.publish_target(self.altitude)
            now = time.monotonic()
            if now - self.last_sample >= self.period:
                self.last_sample = now
                self.samples.append((now, self.px4_z, self.px4_vz, self.sim_z()))
            if self.elapsed() > self.hold_s:
                self.finish()

    def finish(self) -> None:
        print('\n============ 高度对比（PX4 估计 vs AirSim 真值）============')
        if self.samples:
            t0 = self.samples[0][0]
            z0, sim0 = self.samples[0][1], self.samples[0][3]
            for t, z, vz, sim_z in self.samples:
                print(f'{t - t0:6.1f} {z:9.3f} {vz:8.3f} {sim_z:9.3f} '
                      f'{z - z0:8.3f} {sim_z - sim0:8.3f}')
            print(f'\n期间变化量：PX4 Δz={self.samples[-1][1] - z0:+.3f} m，'
                  f'真值 Δz={self.samples[-1][3] - sim0:+.3f} m')
        raise SystemExit(0)


class VerticalCheck(FlightCheckBase):
    """检查竖直速度指令的符号方向。"""

    def __init__(self) -> None:
        super().__init__('flight_check_vertical')
        self.declare_parameter('start_altitude_m', 2.0)
        self.declare_parameter('command_speed_mps', 0.5)
        self.declare_parameter('segment_s', 6.0)
        self.altitude = float(self.get_parameter('start_altitude_m').value)
        self.speed = float(self.get_parameter('command_speed_mps').value)
        self.segment = float(self.get_parameter('segment_s').value)
        self.marks = {}

    def first_phase(self) -> str:
        return 'CLIMB'

    def mark(self, name: str) -> None:
        self.marks[name] = (self.px4_z, self.sim_z())

    def on_phase(self) -> None:
        if self.phase == 'CLIMB':
            if self.climb_to(self.altitude):
                self.mark('before_up')
                self.enter('UP')
            elif self.elapsed() > self.phase_timeout:
                self.fail('爬升超时')
        elif self.phase == 'UP':
            self.publish_up_velocity(self.speed)
            if self.elapsed() > self.segment:
                self.mark('after_up')
                self.enter('RECENTER')
        elif self.phase == 'RECENTER':
            if self.climb_to(self.altitude) or self.elapsed() > 25:
                self.mark('before_down')
                self.enter('DOWN')
        elif self.phase == 'DOWN':
            self.publish_up_velocity(-self.speed)
            if self.elapsed() > self.segment:
                self.mark('after_down')
                self.finish()

    def finish(self) -> None:
        def d(a, b, idx):
            return self.marks[b][idx] - self.marks[a][idx]

        up_px4, up_sim = d('before_up', 'after_up', 0), d('before_up', 'after_up', 1)
        dn_px4, dn_sim = d('before_down', 'after_down', 0), d('before_down', 'after_down', 1)
        print('\n========== 竖直速度指令方向测试 ==========')
        print(f'上向 {self.speed:+.2f} m/s × {self.segment:.0f} s：'
              f'PX4 Δz={up_px4:+.3f} m，真值 Δz={up_sim:+.3f} m（期望为负）')
        print(f'下向 {-self.speed:+.2f} m/s × {self.segment:.0f} s：'
              f'PX4 Δz={dn_px4:+.3f} m，真值 Δz={dn_sim:+.3f} m（期望为正）')
        print(f'\n结论：上向 {"一致" if up_px4 < -0.2 and up_sim < -0.2 else "不一致"}，'
              f'下向 {"一致" if dn_px4 > 0.2 and dn_sim > 0.2 else "不一致"}')
        raise SystemExit(0)


MODES = {'altitude': AltitudeCheck, 'vertical': VerticalCheck}


def main() -> None:
    if len(sys.argv) < 2 or sys.argv[1] not in MODES:
        print(f'用法：{sys.argv[0]} {{{"|".join(MODES)}}} [--ros-args ...]')
        raise SystemExit(2)
    mode = sys.argv[1]
    rclpy.init(args=[sys.argv[0]] + sys.argv[2:])
    node = MODES[mode]()
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
