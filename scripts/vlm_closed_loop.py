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
from std_msgs.msg import String

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

        self.altitude = float(self.get_parameter('start_altitude_m').value)
        self.run_s = float(self.get_parameter('run_s').value)
        self.output_csv = str(self.get_parameter('output_csv').value)
        self.land_at_end = bool(self.get_parameter('land_at_end').value)

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

    # ------------------------------------------------------------- 回调

    def _on_decision(self, msg: String) -> None:
        try:
            self.latest_decision = json.loads(msg.data)
        except ValueError:
            self.latest_decision = None

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
            'wp_yaw_deg': (math.degrees(first['yaw_rad'])
                           if isinstance(first.get('yaw_rad'), (int, float)) else None),
            'command_source': decision.get('command_source'),
            'guard': decision.get('guard'),
            'cmd_yaw_rate': velocity.get('yaw_rate'),
            'cmd_forward': velocity.get('forward'),
        })

    # ------------------------------------------------------------- 阶段机

    def first_phase(self) -> str:
        return 'CLIMB'

    def on_phase(self) -> None:
        if self.phase == 'CLIMB':
            if self.climb_to(self.altitude):
                self.get_logger().info(
                    f'到达 {self.altitude:.1f} m，交出控制权给 VLM（本脚本不再发目标点）')
                self.enter('RUN')
            elif self.elapsed() > self.phase_timeout:
                self.fail('爬升超时')
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
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
