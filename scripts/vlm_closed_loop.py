#!/usr/bin/env python3
"""Step C 闭环驱动（C1 只偏航 / C2 加前进 共用）：
验证 `waypoint yaw`→偏航、`waypoint forward`→前向速度这条链路的方向、幅值与收敛性。

做四件事：
  1) 起飞到 2 m 并悬停（位置模式）；
  2) 交给 `vlm_navigator`（非 dry_run）用速度设定点驱动：本脚本**不再发目标点**，
     让 offboard_bridge 切到速度模式，避免两个节点互相打架；
  3) 全程记录航向（解卷绕）、高度、**水平位移**、VLM 的 `apos_state`/`waypoint`/我方指令；
  4) 结束时给出**航向变化与状态动作的对应关系**——这是判符号的硬证据：
     PX4 航向是 NED（顺时针为正），所以"左转"= 航向**减小**。

用法（前置：Agent + PX4 + bringup 已启动；vlm_navigator 以
     `dry_run:=false forward_scale:=0.0 lateral_scale:=0.0 max_yaw_rate:=0.5
      run_duration_s:=30 geofence_max_distance_m:=5` 在跑）：

    python3 scripts/vlm_closed_loop.py --ros-args \
        -p start_altitude_m:=2.0 -p run_s:=40.0 \
        -p output_csv:=logs/2026-10-12/c2_closed_loop.csv
"""

import csv
import json
import math
import os
import sys

import rclpy
from flight_check import FlightCheckBase
from geometry_msgs.msg import Twist
from px4_interface.frames import ned_to_enu, ned_yaw_to_enu_yaw
from std_msgs.msg import String
from vlm.safety import parse_clearance_status

# 日志行缓冲：外部脚本常靠 grep 日志来判断阶段（例如"等进入 RUN 再开始注入"），
# 块缓冲会让判断滞后甚至永远不命中（2026-10-12 踩到两次）。
sys.stdout.reconfigure(line_buffering=True)


class ClosedLoopBase(FlightCheckBase):
    """CLIMB → RUN（交出控制权给 VLM）→ 汇总 → 降落。"""

    def __init__(self) -> None:
        super().__init__('vlm_closed_loop')
        self.declare_parameter('start_altitude_m', 2.0)
        self.declare_parameter('run_s', 40.0)
        self.declare_parameter('decision_topic', '/vlm_navigator/decision')
        self.declare_parameter('output_csv', 'logs/c1_closed_loop.csv')
        self.declare_parameter('land_at_end', True)
        # 摆位阶段：起飞后先朝当前机头方向慢速前进，直到"前向净空"降到阈值（把飞机送到障碍前）
        self.declare_parameter('pre_position', False)
        self.declare_parameter('pre_speed_mps', 0.2)
        self.declare_parameter('pre_stop_clearance_m', 2.0)
        self.declare_parameter('pre_timeout_s', 30.0)
        # 本地图偶发单帧近读数（实测见过 1.91 m 一闪而过），摆位判定必须要求**连续稳定**
        self.declare_parameter('pre_stable_samples', 3)
        # 扫描变体（服务器侧 2026-10-13 确认）：慢速偏航 360° 找障碍 → 对准 → 逼近 → 交权
        self.declare_parameter('pre_scan', False)
        self.declare_parameter('scan_yaw_rate_deg_s', 15.0)
        self.declare_parameter('scan_found_m', 3.0)
        self.declare_parameter('scan_side_open_m', 1.5)
        self.declare_parameter('scan_max_s', 32.0)
        self.declare_parameter('verify_s', 2.5)
        # 对准（服务器侧建议）：用扫描时确认的方位作为目标航向，转到位再逼近
        self.declare_parameter('align_tolerance_deg', 8.0)
        self.declare_parameter('align_timeout_s', 10.0)
        self.declare_parameter('align_yaw_rate', 0.4)
        self.declare_parameter('realign_max', 3)
        # 多点扫描（服务器侧 2026-10-13 建议 ①）：形如 "0,15;12,0;0,-15;-12,0"（东,北）
        self.declare_parameter('scan_points', '')
        self.declare_parameter('verify_rounds', 2)
        # 转场（服务器侧 2026-10-13 建议 ①）：高一点飞过去，再降到扫描高度
        self.declare_parameter('transit_enabled', False)
        self.declare_parameter('transit_east_m', 0.0)
        self.declare_parameter('transit_north_m', 0.0)
        self.declare_parameter('transit_altitude_m', 3.5)
        self.declare_parameter('transit_timeout_s', 45.0)
        # 交权瞬间通知 VLM 节点解除 dry_run（节点提前以 dry_run 跑，消除 ~30 s 启动时间差）
        self.declare_parameter('vlm_enable_topic', '/vlm_navigator/enable')
        self.declare_parameter('map_status_topic', '/local_map/status')

        self.altitude = float(self.get_parameter('start_altitude_m').value)
        self.run_s = float(self.get_parameter('run_s').value)
        self.output_csv = str(self.get_parameter('output_csv').value)
        self.land_at_end = bool(self.get_parameter('land_at_end').value)
        self.pre_position = bool(self.get_parameter('pre_position').value)
        self.pre_speed = float(self.get_parameter('pre_speed_mps').value)
        self.pre_stop = float(self.get_parameter('pre_stop_clearance_m').value)
        self.pre_timeout = float(self.get_parameter('pre_timeout_s').value)
        self.pre_stable = max(int(self.get_parameter('pre_stable_samples').value), 1)
        self._clearance_history = []
        self.pre_scan = bool(self.get_parameter('pre_scan').value)
        self.scan_yaw_rate = math.radians(
            float(self.get_parameter('scan_yaw_rate_deg_s').value))
        self.scan_found = float(self.get_parameter('scan_found_m').value)
        self.scan_side_open = float(self.get_parameter('scan_side_open_m').value)
        self.scan_max_s = float(self.get_parameter('scan_max_s').value)
        self.verify_s = float(self.get_parameter('verify_s').value)
        self.transit_enabled = bool(self.get_parameter('transit_enabled').value)
        self.transit_east = float(self.get_parameter('transit_east_m').value)
        self.transit_north = float(self.get_parameter('transit_north_m').value)
        self.transit_altitude = float(self.get_parameter('transit_altitude_m').value)
        self.transit_timeout = float(self.get_parameter('transit_timeout_s').value)
        self.enable_pub = self.create_publisher(
            String, str(self.get_parameter('vlm_enable_topic').value), 10)
        self._scan_hits = 0
        self._scan_best = None
        self._verify_history = []
        self._align_heading = None       # 目标航向（ENU）
        self._realign_count = 0
        self._lost_counter = 0
        self._points = self._parse_points(str(self.get_parameter('scan_points').value))
        self._point_index = 0
        self._verify_rounds = max(int(self.get_parameter('verify_rounds').value), 1)
        self._verify_passed = 0
        self.clearance = None

        self.rows = []
        self.latest_decision = None
        self._last_seq = None
        self._heading_prev = None
        self._heading_unwrapped_deg = 0.0
        self.heading_rad = float('nan')
        self.px4_x = float('nan')
        self.px4_y = float('nan')
        self._origin_xy = None
        self.create_subscription(
            String, str(self.get_parameter('decision_topic').value),
            self._on_decision, 10)
        self.create_subscription(
            String, str(self.get_parameter('map_status_topic').value),
            self._on_map_status, 10)
        # 心跳：独立监督进程（flight_supervisor.py）据此判断"驱动是否还活着"
        self.declare_parameter('heartbeat_topic', '/flight_heartbeat')
        self.heartbeat_pub = self.create_publisher(
            String, str(self.get_parameter('heartbeat_topic').value), 10)
        self.create_timer(0.5, self._beat)

    # ------------------------------------------------------------- 回调

    def _on_decision(self, msg: String) -> None:
        try:
            self.latest_decision = json.loads(msg.data)
        except ValueError:
            self.latest_decision = None

    def _on_map_status(self, msg: String) -> None:
        parsed = parse_clearance_status(msg.data)
        if parsed is not None:
            self.clearance = parsed

    def _beat(self) -> None:
        msg = String()
        msg.data = f'{self.phase} t={self.elapsed():.1f}'
        self.heartbeat_pub.publish(msg)

    def _on_position(self, msg) -> None:
        super()._on_position(msg)
        if getattr(msg, 'xy_valid', True):
            self.px4_x = float(msg.x)
            self.px4_y = float(msg.y)
        heading = getattr(msg, 'heading', None)
        if heading is None or math.isnan(heading):
            return
        self.heading_rad = float(heading)
        # 解卷绕：NED 航向是 [-π, π]，跨 ±π 处要按最短方向累加，否则算不出总转角
        if self._heading_prev is not None:
            delta = math.remainder(self.heading_rad - self._heading_prev, 2 * math.pi)
            self._heading_unwrapped_deg += math.degrees(delta)
        self._heading_prev = self.heading_rad

    # ------------------------------------------------------------- 记录

    def record(self, phase: str) -> None:
        decision = self.latest_decision or {}
        if decision.get('frame_seq') == self._last_seq:
            return
        self._last_seq = decision.get('frame_seq')
        if phase == 'RUN' and self._origin_xy is None and not math.isnan(self.px4_x):
            self._origin_xy = (self.px4_x, self.px4_y)
        drift = (math.hypot(self.px4_x - self._origin_xy[0], self.px4_y - self._origin_xy[1])
                 if self._origin_xy and not math.isnan(self.px4_x) else None)
        waypoints = decision.get('waypoints') or []
        first = waypoints[0] if waypoints else {}
        velocity = decision.get('body_velocity') or {}
        self.rows.append({
            't': round(self.elapsed(), 2),
            'phase': phase,
            'heading_unwrapped_deg': round(self._heading_unwrapped_deg, 2),
            'heading_deg': (round(math.degrees(self.heading_rad), 2)
                            if not math.isnan(self.heading_rad) else None),
            'px4_z': (round(self.px4_z, 3) if self.px4_z is not None else None),
            'px4_x': None if math.isnan(self.px4_x) else round(self.px4_x, 3),
            'px4_y': None if math.isnan(self.px4_y) else round(self.px4_y, 3),
            'drift_m': None if drift is None else round(drift, 3),
            'frame_seq': decision.get('frame_seq'),
            'apos_state': decision.get('apos_state'),
            'forward_clear_m': (self.clearance[0] if self.clearance else None),
            'wp_yaw_deg': (math.degrees(first['yaw_rad'])
                           if isinstance(first.get('yaw_rad'), (int, float)) else None),
            'command_source': decision.get('command_source'),
            'guard': decision.get('guard'),
            'cmd_yaw_rate': velocity.get('yaw_rate'),
            'cmd_forward': velocity.get('forward'),
        })

    def record_approach(self) -> None:
        """摆位阶段没有 VLM 决策，单独记一行（保留同一套列名，便于统一分析）。"""
        now = round(self.elapsed(), 2)
        if self.rows and self.rows[-1]['phase'] == 'APPROACH' \
                and now - self.rows[-1]['t'] < 0.4:
            return
        row = {
            'frame_seq': None, 'apos_state': None, 'wp_yaw_deg': None, 'guard': None,
            'drift_m': None,
        }
        row.update({
            't': now,
            'phase': 'APPROACH',
            'heading_unwrapped_deg': round(self._heading_unwrapped_deg, 2),
            'heading_deg': (round(math.degrees(self.heading_rad), 2)
                            if not math.isnan(self.heading_rad) else None),
            'px4_z': (round(self.px4_z, 3) if self.px4_z is not None else None),
            'px4_x': None if math.isnan(self.px4_x) else round(self.px4_x, 3),
            'px4_y': None if math.isnan(self.px4_y) else round(self.px4_y, 3),
            'forward_clear_m': (self.clearance[0] if self.clearance else None),
            'command_source': 'approach',
            'cmd_forward': self.pre_speed,
            'cmd_yaw_rate': 0.0,
        })
        self.rows.append(row)

    def publish_enu_velocity(self, forward_mps: float) -> None:
        """按当前航向把"机头方向的前进速度"转成 ENU 速度发给桥接节点。

        摆位阶段我们用最简单的开环前进（不依赖 VLM），把飞机送到障碍前再交权。
        """
        if math.isnan(self.heading_rad):
            return
        heading_enu = ned_yaw_to_enu_yaw(self.heading_rad)
        east = forward_mps * math.cos(heading_enu)
        north = forward_mps * math.sin(heading_enu)
        msg = Twist()
        msg.linear.x = east
        msg.linear.y = north
        msg.linear.z = 0.0
        msg.angular.z = 0.0
        self.velocity_pub.publish(msg)

    def publish_yaw_rate(self, rate: float) -> None:
        """只给偏航角速度（速度模式），用于扫描阶段边转边看净空。"""
        msg = Twist()
        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.z = float(rate)
        self.velocity_pub.publish(msg)

    def publish_target_enu(self, east: float, north: float, up: float) -> None:
        """位置模式目标（ENU，绝对坐标）：转场与降高用。

        基类的 ``publish_target`` 只写 z（x,y 默认 0），会把飞机拉回原点——转场必须给全 x,y。
        """
        from geometry_msgs.msg import PoseStamped

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position.x = float(east)
        msg.pose.position.y = float(north)
        msg.pose.position.z = float(up)
        msg.pose.orientation.w = 1.0
        self.target_pub.publish(msg)

    def position_enu(self):
        """当前 ENU 位置（拿不到返回 None）。"""
        if self.px4_z is None or math.isnan(self.px4_x) or math.isnan(self.px4_y):
            return None
        return ned_to_enu(self.px4_x, self.px4_y, self.px4_z)

    # ------------------------------------------------------------- 阶段机

    def first_phase(self) -> str:
        return 'CLIMB'

    @staticmethod
    def _parse_points(text: str):
        """解析 "东,北;东,北" → [(east, north), ...]（空串返回 []）。"""
        points = []
        for chunk in (text or '').split(';'):
            chunk = chunk.strip()
            if not chunk:
                continue
            try:
                east, north = (float(v) for v in chunk.split(','))
            except ValueError:
                continue
            points.append((east, north))
        return points

    def _current_point(self):
        if self._points:
            return self._points[min(self._point_index, len(self._points) - 1)]
        return (self.transit_east, self.transit_north)

    def _next_point(self) -> bool:
        """切到下一个候选点；用尽返回 False（调用方应降落收工）。"""
        self._point_index += 1
        if self._point_index >= len(self._points):
            return False
        self._scan_hits = 0
        self._verify_passed = 0
        self._realign_count = 0
        return True

    def enter(self, phase: str) -> None:
        """进入 RUN（交权）时通知提前跑着的 VLM 节点解除 dry_run。"""
        super().enter(phase)
        if phase == 'RUN':
            msg = String()
            msg.data = 'enable'
            self.enable_pub.publish(msg)
            self.get_logger().info('已向 VLM 节点发送 enable（解除 dry_run）')

    def on_phase(self) -> None:
        if self.phase == 'CLIMB':
            if self.climb_to(self.altitude):
                if self.pre_position:
                    if self.transit_enabled or self._points:
                        self.get_logger().info(
                            f'到达 {self.altitude:.1f} m，先转场到 ENU（东 {self.transit_east:+.1f}, '
                            f'北 {self.transit_north:+.1f}），转场高度 {self.transit_altitude:.1f} m')
                        self.enter('TRANSIT')
                    else:
                        self.get_logger().info(
                            f'到达 {self.altitude:.1f} m，开始摆位：以 {self.pre_speed:.2f} m/s '
                            f'前进直到前向净空 ≤ {self.pre_stop:.1f} m')
                        self.enter('SCAN' if self.pre_scan else 'APPROACH')
                else:
                    self.get_logger().info(
                        f'到达 {self.altitude:.1f} m，交出控制权给 VLM（本脚本不再发目标点）')
                    self.enter('RUN')
            elif self.elapsed() > self.phase_timeout:
                self.fail('爬升超时')
            return

        if self.phase in ('TRANSIT', 'DESCEND'):
            altitude = self.transit_altitude if self.phase == 'TRANSIT' else self.altitude
            target_east, target_north = self._current_point()
            self.publish_target_enu(target_east, target_north, altitude)
            self.record_approach()
            current = self.position_enu()
            close = (current is not None
                     and abs(current[0] - target_east) < 0.5
                     and abs(current[1] - target_north) < 0.5
                     and abs(current[2] - altitude) < 0.4)
            if close:
                if self.phase == 'TRANSIT':
                    self.get_logger().info(
                        f'转场到位（ENU 东 {current[0]:+.1f} 北 {current[1]:+.1f} '
                        f'上 {current[2]:.1f}），降到扫描高度 {self.altitude:.1f} m')
                    self.enter('DESCEND')
                else:
                    self.get_logger().info(
                        f'已降到 {self.altitude:.1f} m，开始扫描/摆位')
                    self.enter('SCAN' if self.pre_scan else 'APPROACH')
            elif self.elapsed() > self.transit_timeout:
                self.get_logger().warn(
                    f'{self.phase} 超时（{self.transit_timeout:.0f} s，'
                    f'当前 {current}）→ 就地继续')
                self.enter('SCAN' if self.pre_scan else 'APPROACH')
            return

        if self.phase == 'ALIGN':
            # 转到位再逼近（服务器侧建议）：用扫描确认方位作为目标航向
            if self._align_heading is None or math.isnan(self.heading_rad):
                self.enter('APPROACH')
                return
            current = ned_yaw_to_enu_yaw(self.heading_rad)
            error = math.remainder(self._align_heading - current, 2.0 * math.pi)
            tolerance = math.radians(float(self.get_parameter('align_tolerance_deg').value))
            if abs(error) <= tolerance:
                self.publish_yaw_rate(0.0)
                self.get_logger().info(
                    f'对准完成（误差 {math.degrees(error):+.1f}°）→ 开始以 '
                    f'{self.pre_speed:.2f} m/s 逼近')
                self.enter('APPROACH')
                return
            if self.elapsed() > float(self.get_parameter('align_timeout_s').value):
                self.publish_yaw_rate(0.0)
                self.get_logger().warn(
                    f'对准超时（残留误差 {math.degrees(error):+.1f}°）→ 仍转入逼近')
                self.enter('APPROACH')
                return
            limit = float(self.get_parameter('align_yaw_rate').value)
            self.publish_yaw_rate(max(-limit, min(limit, 1.2 * error)))
            return

        if self.phase == 'APPROACH':
            forward_clear = self.clearance[0] if self.clearance else None
            # 逼近中**持续复验**：净空回升/消失（障碍脱离视野）→ 停下重新对准
            if forward_clear is not None and forward_clear >= self.scan_found:
                self._lost_counter += 1
            else:
                self._lost_counter = 0
            if (self._lost_counter >= self.pre_stable
                    and self._realign_count < int(self.get_parameter('realign_max').value)):
                self.publish_enu_velocity(0.0)
                self._realign_count += 1
                self._lost_counter = 0
                self._clearance_history = []
                self.get_logger().warn(
                    f'逼近中净空回升到 {forward_clear:.2f} m（障碍脱离视野）→ 停下重新对准'
                    f'（第 {self._realign_count} 次）')
                self._align_heading = (ned_yaw_to_enu_yaw(self.heading_rad)
                                       if self._align_heading is None else self._align_heading)
                self.enter('ALIGN')
                return
            if forward_clear is not None:
                self._clearance_history.append(forward_clear)
                self._clearance_history = self._clearance_history[-self.pre_stable:]
            stable = (len(self._clearance_history) >= self.pre_stable
                      and all(value <= self.pre_stop for value in self._clearance_history))
            if stable:
                self.publish_enu_velocity(0.0)
                self.get_logger().info(
                    f'摆位完成：前向净空连续 {self.pre_stable} 次 ≤ {self.pre_stop:.1f} m'
                    f'（最近 {[round(v, 2) for v in self._clearance_history]}），'
                    '停下并交出控制权给 VLM')
                self.enter('RUN')
                return
            if self.elapsed() > self.pre_timeout:
                self.publish_enu_velocity(0.0)
                self.get_logger().warn(
                    f'摆位超时（{self.pre_timeout:.0f} s，前向净空 {forward_clear}），'
                    '仍在当前位置交出控制权')
                self.enter('RUN')
                return
            self.publish_enu_velocity(self.pre_speed)
            self.record_approach()
            return

        if self.phase == 'SCAN':
            # 慢速偏航扫一圈，边转边看本机地图净空（服务器侧 2026-10-13 确认的做法）
            self.publish_yaw_rate(self.scan_yaw_rate)
            self.record_approach()          # 与摆位共用记录（同一套列）
            forward_clear = self.clearance[0] if self.clearance else None
            sides_open = (self.clearance is not None
                          and max(self.clearance[1], self.clearance[2]) >= self.scan_side_open)
            if forward_clear is not None and forward_clear <= self.scan_found:
                self._scan_hits += 1
                self._scan_best = forward_clear
                if sides_open and self._scan_hits >= self.pre_stable:
                    self.publish_yaw_rate(0.0)
                    self.get_logger().info(
                        f'扫描发现可用障碍：前向净空连续 {self._scan_hits} 次 ≤ '
                        f'{self.scan_found:.1f} m（最近 {forward_clear:.2f} m），'
                        f'且一侧开阔（左 {self.clearance[1]:.2f} / 右 {self.clearance[2]:.2f}）'
                        '→ 停转，先**原地静止复验**（排除自运动拖影）')
                    self._verify_history = []
                    self.enter('VERIFY')
                    return
                if not sides_open:
                    self.get_logger().info(
                        f'该方位净空 {forward_clear:.2f} m 但两侧都紧'
                        f'（左 {self.clearance[1]:.2f} / 右 {self.clearance[2]:.2f}）'
                        '→ 不适合绕行，继续扫', throttle_duration_sec=2.0)
                    self._scan_hits = 0
            else:
                self._scan_hits = 0
            if self.elapsed() > self.scan_max_s:
                self.publish_yaw_rate(0.0)
                if self._next_point():
                    target_east, target_north = self._current_point()
                    self.get_logger().warn(
                        f'本点扫描 {self.scan_max_s:.0f} s 未找到可复验障碍 → 转场到下一个候选点 '
                        f'(东 {target_east:+.0f}, 北 {target_north:+.0f})')
                    self.enter('TRANSIT')
                else:
                    self.get_logger().warn(
                        f'全部候选点扫描完毕仍未找到可复验障碍（{self.scan_max_s:.0f} s/点）'
                        '→ 按多点预算收工降落')
                    self.send('land_fast')
                    self.finish()
            return

        if self.phase == 'VERIFY':
            # 原地静止复验：自运动拖影在停下 2–3 s 后会消失，真障碍会稳定留在净空里
            self.publish_yaw_rate(0.0)
            self.record_approach()
            if self.clearance is not None:
                self._verify_history.append(self.clearance[0])
                self._verify_history = self._verify_history[-self.pre_stable:]
            if self.elapsed() > self.verify_s:
                stable = (len(self._verify_history) >= self.pre_stable
                          and all(v <= self.scan_found for v in self._verify_history))
                values = [round(v, 2) for v in self._verify_history]
                if stable:
                    self._verify_passed += 1
                    if self._verify_passed >= self._verify_rounds:
                        self.get_logger().info(
                            f'静止复验通过 {self._verify_passed}/{self._verify_rounds} 轮：'
                            f'净空 {values} 仍 ≤ {self.scan_found:.1f} m → 真障碍，先对准再逼近')
                        self._clearance_history = []
                        self._align_heading = ned_yaw_to_enu_yaw(self.heading_rad)
                        self.enter('ALIGN')
                    else:
                        self.get_logger().info(
                            f'静止复验第 {self._verify_passed}/{self._verify_rounds} 轮通过'
                            f'（净空 {values}）→ 再做一轮确认（服务器侧建议的"二次通过"）')
                        self._verify_history = []
                        self.enter('VERIFY')
                else:
                    self.get_logger().info(
                        f'静止复验未通过：净空 {values}（疑似自运动拖影）→ 继续扫描')
                    self._scan_hits = 0
                    self._verify_passed = 0
                    self.enter('SCAN')
            return

        if self.phase == 'RUN':
            self.record('RUN')
            if self.elapsed() > self.run_s:
                self.finish()
            return

    # ------------------------------------------------------------- 汇总

    def finish(self) -> None:
        rows = [r for r in self.rows if r['phase'] == 'RUN']
        states = {}
        for row in rows:
            states[row['apos_state'] or '-'] = states.get(row['apos_state'] or '-', 0) + 1
        yaw_cmds = [r['cmd_yaw_rate'] for r in rows if isinstance(r['cmd_yaw_rate'], (int, float))]
        positives = sum(1 for v in yaw_cmds if v > 0.05)
        negatives = sum(1 for v in yaw_cmds if v < -0.05)
        delta = self._heading_unwrapped_deg

        if self.output_csv:
            directory = os.path.dirname(self.output_csv)
            if directory:
                os.makedirs(directory, exist_ok=True)
            if self.rows:
                with open(self.output_csv, 'w', newline='', encoding='utf-8') as handle:
                    writer = csv.DictWriter(handle, fieldnames=list(self.rows[0].keys()))
                    writer.writeheader()
                    writer.writerows(self.rows)

        print('\n=========== 闭环小结（C1 只偏航 / C2 加前进）===========')
        print(f'运行 {len(rows)} 帧；apos_state 分布 {states}')
        print(f'我方偏航指令：正（=左）{positives} 帧 / 负（=右）{negatives} 帧')
        print(f'航向总变化：{delta:+.1f}°（PX4 NED：**负 = 左转**）')
        drifts = [r['drift_m'] for r in rows if isinstance(r['drift_m'], (int, float))]
        if drifts:
            print(f'水平位移：末值 {drifts[-1]:.2f} m / 最大 {max(drifts):.2f} m（围栏 5 m 判据）')
        speeds = [r['cmd_forward'] for r in rows if isinstance(r['cmd_forward'], (int, float))]
        if speeds:
            print(f'前进指令：最大 {max(abs(v) for v in speeds):.2f} m/s')
        print('\n判据：若"正（左）指令占多数"且航向变化为**负**，则方向的符号链正确；'
              '若航向变化为正，说明反号，必须停机改映射。')
        if self.land_at_end:
            self.send('land_fast')
            print('已下发 land_fast。')
        print(f'CSV：{self.output_csv}')
        raise SystemExit(0)


def main() -> int:
    rclpy.init()
    node = ClosedLoopBase()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.send('land_fast')
    except Exception as exc:  # noqa: BLE001
        # 顶层兜底：脚本自己崩了也必须先落地（今天踩过——汇总崩溃时保护一起没了）。
        # 注意只拦 Exception：正常收尾走 SystemExit(0)，不该被记成异常。
        node.get_logger().error(f'驱动脚本异常退出（{type(exc).__name__}: {exc}），先下发 land_fast')
        node.send('land_fast')
        raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
