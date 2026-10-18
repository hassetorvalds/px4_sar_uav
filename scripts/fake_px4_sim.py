#!/usr/bin/env python3
"""最小假 PX4：没有 AirSim/PX4 时，用 ROS 话题驱动 task_executor 做集成自检。

它发布 PX4 状态（vehicle_status / vehicle_local_position）与地图净空，
订阅执行器的 arm/offboard/land_fast 指令以及位置/速度目标，做一个**运动学近似**
（限速移动、land_fast 匀速下降、落地自动上锁）。

用途：在执行器上真机之前，验证
  ARM → OFFBOARD → takeoff → goto → hover → return_home → land → DONE
的状态迁移与指令接口；不验证 PX4 真实动力学。

用法（由 scripts/task_executor_simcheck.sh 调用）::

    python3 scripts/fake_px4_sim.py --duration 80 --report logs/tasks/xxx.jsonl
"""

import argparse
import json
import math
import os
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from px4_interface.frames import enu_to_ned, enu_yaw_to_ned_yaw
from px4_interface.qos import px4_qos
from px4_msgs.msg import VehicleLocalPosition, VehicleStatus
from rclpy.node import Node
from std_msgs.msg import String

MOVE_SPEED_MPS = 0.5
DESCEND_SPEED_MPS = 0.5


class FakePx4(Node):
    def __init__(self, report_path: str, duration_s: float, *,
                 block_after=None, block_for=60.0,
                 takeover_after=None, takeover_for=3.0, expect=('done',)):
        super().__init__('fake_px4_sim')
        self._report_path = report_path
        self._deadline = time.monotonic() + duration_s
        self._started = time.monotonic()
        self._block_after = block_after
        self._block_for = block_for
        self._takeover_after = takeover_after
        self._takeover_for = takeover_for
        self._expect = tuple(expect)

        self._armed = False
        self._offboard = False
        self._landing = False
        self._ever_armed = False
        self._east = 0.0
        self._north = 0.0
        self._up = 0.0
        self._target = None
        self._target_at = 0.0
        self._velocity = None
        self._velocity_at = 0.0
        self._heading_ned = 0.0
        self._finish_at = None

        qos = px4_qos()
        self._status_pub = self.create_publisher(
            VehicleStatus, '/fmu/out/vehicle_status_v4', qos)
        self._position_pub = self.create_publisher(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position_v1', qos)
        self._map_pub = self.create_publisher(String, '/local_map/status', 5)

        self.create_subscription(
            String, '/offboard_bridge/command', self._on_command, 10)
        self.create_subscription(
            PoseStamped, '/offboard_bridge/target_pose', self._on_target, 10)
        self.create_subscription(
            Twist, '/offboard_bridge/velocity_setpoint', self._on_velocity, 10)

        self.create_timer(0.02, self._tick)
        self.create_timer(0.2, self._publish_map)
        self.get_logger().info(
            f'假 PX4 启动：report={report_path} duration={duration_s:.0f}s')

    # ------------------------------------------------------------------

    def _on_command(self, msg: String) -> None:
        command = msg.data.strip().lower()
        if command == 'arm':
            self._armed = True
            self._ever_armed = True
            self.get_logger().info('假 PX4：已解锁（受令）')
        elif command == 'offboard':
            self._offboard = True
        elif command in ('land_fast', 'land'):
            self._landing = True
            self.get_logger().info('假 PX4：开始降落')
        elif command in ('disarm', 'disarm_force'):
            self._armed = False
            self._landing = False
            self._offboard = False

    def _on_target(self, msg: PoseStamped) -> None:
        self._target = (float(msg.pose.position.x), float(msg.pose.position.y),
                        float(msg.pose.position.z))
        self._target_at = time.monotonic()

    def _on_velocity(self, msg: Twist) -> None:
        self._velocity = (float(msg.linear.x), float(msg.linear.y),
                          float(msg.linear.z))
        self._velocity_at = time.monotonic()

    def _publish_map(self) -> None:
        clearance = 'inf'
        elapsed = time.monotonic() - self._started
        if (self._block_after is not None
                and self._block_after <= elapsed <= self._block_after + self._block_for):
            clearance = '0.50'
        self._map_pub.publish(String(
            data=f'voxels=100 frames=10 nearest_forward_m={clearance} '
                 f'left_m=inf right_m=inf '
                 f'nearest_centroid=inf fwd_floor_m=0.45'))

    def _tick(self) -> None:
        now = time.monotonic()
        dt = 0.02
        # 人工接管窗口：armed 但 nav_state 切到 POSCTL，执行器应暂停并停发速度
        if self._takeover_after is not None and self._ever_armed:
            elapsed = now - self._started
            in_window = (self._takeover_after
                         <= elapsed <= self._takeover_after + self._takeover_for)
            self._offboard = not in_window
        if self._landing:
            self._up = max(0.0, self._up - DESCEND_SPEED_MPS * dt)
            if self._up <= 0.005:
                self._up = 0.0
                self._armed = False
                self._landing = False
                self._offboard = False
                self.get_logger().info('假 PX4：已落地并上锁')
        elif self._armed and self._offboard:
            if self._velocity is not None and now - self._velocity_at < 0.5:
                vx, vy, vz = self._velocity
                self._east += vx * dt
                self._north += vy * dt
                self._up = max(0.0, self._up + vz * dt)
            elif self._target is not None and now - self._target_at < 2.0:
                self._move_toward_target(dt)

        self._publish_state()
        if self._ever_armed and not self._armed and not self._landing:
            if self._finish_at is None:
                # 等执行器把 done 写进报告（避免读到 land_fast 就判 FAIL）
                self._finish_at = now + 2.0
                self.get_logger().info('假 PX4：任务收尾（已上锁），2 s 后判定')
            elif now >= self._finish_at:
                raise SystemExit(self._check_report())

    def _move_toward_target(self, dt: float) -> None:
        target_east, target_north, target_up = self._target
        dx = target_east - self._east
        dy = target_north - self._north
        dz = target_up - self._up
        distance = math.sqrt(dx * dx + dy * dy + dz * dz)
        if distance < 1e-3:
            return
        step = min(MOVE_SPEED_MPS * dt, distance)
        self._east += dx / distance * step
        self._north += dy / distance * step
        self._up += dz / distance * step

    def _publish_state(self) -> None:
        x_ned, y_ned, z_ned = enu_to_ned(self._east, self._north, self._up)
        status = VehicleStatus()
        status.timestamp = int((time.monotonic() - self._started) * 1e6)
        status.arming_state = (VehicleStatus.ARMING_STATE_ARMED if self._armed
                               else VehicleStatus.ARMING_STATE_DISARMED)
        status.nav_state = (VehicleStatus.NAVIGATION_STATE_OFFBOARD if self._offboard
                            else VehicleStatus.NAVIGATION_STATE_POSCTL)
        status.failsafe = False
        status.pre_flight_checks_pass = True
        status.accepts_offboard_setpoints = self._offboard
        self._status_pub.publish(status)

        position = VehicleLocalPosition()
        position.timestamp = status.timestamp
        position.x = x_ned
        position.y = y_ned
        position.z = z_ned
        position.vx = 0.0
        position.vy = 0.0
        position.vz = 0.0
        position.heading = self._heading_ned
        position.xy_valid = True
        position.z_valid = True
        self._position_pub.publish(position)

    # ------------------------------------------------------------------

    def _check_report(self) -> int:
        if not os.path.isfile(self._report_path):
            print(f'[simcheck] 报告不存在：{self._report_path}')
            return 2
        events = []
        with open(self._report_path, encoding='utf-8') as handle:
            for line in handle:
                line = line.strip()
                if line:
                    events.append(json.loads(line))
        names = [event['event'] for event in events]
        aborts = [event for event in events if event['event'] == 'abort']
        missing = [name for name in self._expect if name not in names]
        print('[simcheck] 事件序列：' + ' → '.join(names))
        print(f'[simcheck] 期望事件：{list(self._expect)}；缺失={missing}')
        ok = ('done' in names and not missing
              and (not aborts or 'abort' in self._expect))
        if aborts and 'abort' not in self._expect:
            print(f'[simcheck] 非期望 abort：{aborts}')
        print(f'[simcheck] {"PASS" if ok else "FAIL"}'
              f'（done={"done" in names}, abort={len(aborts)}）')
        return 0 if ok else 1


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('--duration', type=float, default=90.0)
    parser.add_argument('--report', required=True)
    parser.add_argument('--block-after', type=float, default=None,
                        help='多少秒后模拟前向净空 0.5 m（触发净空门）')
    parser.add_argument('--block-for', type=float, default=60.0)
    parser.add_argument('--takeover-after', type=float, default=None,
                        help='多少秒后模拟人工接管（nav_state 切 POSCTL）')
    parser.add_argument('--takeover-for', type=float, default=3.0)
    parser.add_argument('--expect', default='done',
                        help='期望出现在报告里的事件名，逗号分隔')
    args = parser.parse_args(argv)
    expect = tuple(name.strip() for name in args.expect.split(',') if name.strip())
    rclpy.init()
    node = FakePx4(args.report, args.duration,
                   block_after=args.block_after, block_for=args.block_for,
                   takeover_after=args.takeover_after,
                   takeover_for=args.takeover_for, expect=expect)
    try:
        rclpy.spin(node)
    except SystemExit as exc:
        rclpy.try_shutdown()
        return int(exc.code or 0)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.try_shutdown()
    return 1


if __name__ == '__main__':
    raise SystemExit(main())
