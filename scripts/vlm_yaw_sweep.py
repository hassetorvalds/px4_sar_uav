#!/usr/bin/env python3
"""实验 B：悬停缓慢偏航扫场，判断远端 VLM 的指向/路点是否跟随目标方位。

背景（与服务器侧确认后）：LightNav-0 的 ``apos`` 是**可通行落点**（"往哪走"），
悬停静止时它恒为画面底边中心（x_norm 0.5104 / y_norm 0.9815）是**预期行为**。
所以"148 s 内指向不动"本身不能证明模型好坏——必须**改变视角**再看它是否跟随。

做法：飞机悬停在固定高度，先按 ``yaw_rate_deg_s`` 匀速转 ``sweep_s`` 秒（一路），
停 ``pause_s`` 秒，再反向转同样时长；全程 dry_run 记录
``/vlm_navigator/decision`` 里的指向、状态、路点与航向，输出 CSV 与小结。

判据：
- 若 ``apos.x`` / ``waypoint yaw`` 随航向单调变化 → 模型在跟踪方位，只是控制映射问题；
- 若扫完一圈仍恒为同一值 → 才需要回头查 checkpoint / 输入（罕见，但一锤定音）。

用法：
    source scripts/env.sh
    # 另开终端：run_agent.sh / run_px4_sitl.sh / bringup.sh 已启动，
    #           并且 vlm_navigator 以 dry_run=true、provider=remote_http 在跑
    python3 scripts/vlm_yaw_sweep.py --ros-args \
        -p instruction:="go to the collapsed building ahead" \
        -p output_csv:=logs/2026-10-11/yaw_sweep.csv
"""

import csv
import json
import math
import os
import statistics
import sys
import time

import rclpy
from flight_check import FlightCheckBase
from geometry_msgs.msg import Twist
from std_msgs.msg import String


class YawSweepCheck(FlightCheckBase):
    """悬停 → 匀速偏航扫场 → 反向扫回 → 汇总。"""

    def __init__(self) -> None:
        super().__init__('vlm_yaw_sweep')
        self.declare_parameter('start_altitude_m', 2.0)
        self.declare_parameter('yaw_rate_deg_s', 15.0)
        self.declare_parameter('sweep_s', 24.0)
        self.declare_parameter('pause_s', 5.0)
        self.declare_parameter('decision_topic', '/vlm_navigator/decision')
        self.declare_parameter('output_csv', 'logs/yaw_sweep.csv')
        self.declare_parameter('land_at_end', True)

        self.altitude = float(self.get_parameter('start_altitude_m').value)
        self.yaw_rate = math.radians(float(self.get_parameter('yaw_rate_deg_s').value))
        self.sweep_s = float(self.get_parameter('sweep_s').value)
        self.pause_s = float(self.get_parameter('pause_s').value)
        self.land_at_end = bool(self.get_parameter('land_at_end').value)
        self.output_csv = str(self.get_parameter('output_csv').value)

        self.rows = []
        self.latest_decision = None
        self.heading_rad = float('nan')
        self._last_seq = None
        self._phase_marks = {}
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
        """基类只存高度，这里额外记录航向（判断指向是否跟随方位要用）。"""
        super()._on_position(msg)
        heading = getattr(msg, 'heading', None)
        if heading is not None and not math.isnan(heading):
            self.heading_rad = float(heading)

    # ------------------------------------------------------------- 工具

    def publish_yaw_rate(self, rate: float) -> None:
        """只给偏航角速度，线速度为 0（位置由速度环自己保持）。"""
        msg = Twist()
        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.z = float(rate)
        self.velocity_pub.publish(msg)

    def record(self, phase: str) -> None:
        decision = self.latest_decision or {}
        if decision.get('frame_seq') == self._last_seq:
            return                       # 同一帧只记一次（决策 2.2 Hz，tick 10 Hz）
        self._last_seq = decision.get('frame_seq')
        waypoints = decision.get('waypoints') or []
        first = waypoints[0] if waypoints else {}
        self.rows.append({
            't': round(self.elapsed(), 2),
            'phase': phase,
            'heading_deg': (round(math.degrees(self.heading_rad), 2)
                            if not math.isnan(self.heading_rad) else None),
            'frame_seq': decision.get('frame_seq'),
            'apos_state': decision.get('apos_state'),
            'opos_state': decision.get('opos_state'),
            'apos_x': decision.get('point_x_norm'),
            'apos_y': decision.get('point_y_norm'),
            'depth_m': decision.get('depth_m'),
            'wp_forward_m': first.get('forward_m'),
            'wp_lateral_m': first.get('lateral_m'),
            'wp_yaw_deg': (math.degrees(first['yaw_rad'])
                           if isinstance(first.get('yaw_rad'), (int, float)) else None),
            'command_source': decision.get('command_source'),
            'cmd_forward': (decision.get('body_velocity') or {}).get('forward'),
            'cmd_up': (decision.get('body_velocity') or {}).get('up'),
            'cmd_yaw_rate': (decision.get('body_velocity') or {}).get('yaw_rate'),
        })

    # ------------------------------------------------------------- 阶段机

    def first_phase(self) -> str:
        return 'CLIMB'

    def on_phase(self) -> None:
        if self.phase == 'CLIMB':
            if self.climb_to(self.altitude):
                self.enter('SWEEP_FORWARD')
            elif self.elapsed() > self.phase_timeout:
                self.fail('爬升超时')
            return

        if self.phase in ('SWEEP_FORWARD', 'SWEEP_BACKWARD'):
            rate = self.yaw_rate if self.phase == 'SWEEP_FORWARD' else -self.yaw_rate
            self.publish_yaw_rate(rate)
            self.record(self.phase)
            if self.elapsed() > self.sweep_s:
                self.publish_yaw_rate(0.0)
                self._phase_marks[self.phase] = self._summarise(self.phase)
                self.enter('PAUSE' if self.phase == 'SWEEP_FORWARD' else 'DONE')
            return

        if self.phase == 'PAUSE':
            self.publish_yaw_rate(0.0)
            self.record('PAUSE')
            if self.elapsed() > self.pause_s:
                self.enter('SWEEP_BACKWARD')
            return

        if self.phase == 'DONE':
            if self.land_at_end:
                self.send('land_fast')
            self.finish()

    # ------------------------------------------------------------- 汇总

    def _summarise(self, phase: str) -> dict:
        rows = [r for r in self.rows if r['phase'] == phase]
        xs = [r['apos_x'] for r in rows if isinstance(r['apos_x'], (int, float))]
        yaws = [r['wp_yaw_deg'] for r in rows if isinstance(r['wp_yaw_deg'], (int, float))]
        headings = [r['heading_deg'] for r in rows if isinstance(r['heading_deg'], (int, float))]
        out = {'frames': len(rows), 'apos_x': {}, 'wp_yaw_deg': {}}
        for name, values in (('apos_x', xs), ('wp_yaw_deg', yaws)):
            if values:
                out[name] = {
                    'min': round(min(values), 4), 'max': round(max(values), 4),
                    'median': round(statistics.median(values), 4),
                    'span': round(max(values) - min(values), 4),
                }
        if len(xs) >= 3 and len(headings) >= 3 and len(xs) == len(headings):
            out['corr_apos_x_vs_heading'] = round(_corr(headings, xs), 3)
        return out

    def finish(self) -> None:
        self._phase_marks.setdefault('SWEEP_FORWARD', self._summarise('SWEEP_FORWARD'))
        self._phase_marks.setdefault('SWEEP_BACKWARD', self._summarise('SWEEP_BACKWARD'))
        if self.output_csv:
            directory = os.path.dirname(self.output_csv)
            if directory:
                os.makedirs(directory, exist_ok=True)
            with open(self.output_csv, 'w', newline='', encoding='utf-8') as handle:
                writer = csv.DictWriter(handle, fieldnames=list(self.rows[0].keys()))
                writer.writeheader()
                writer.writerows(self.rows)

        print('\n=========== 偏航扫场小结 ===========')
        for phase in ('SWEEP_FORWARD', 'SWEEP_BACKWARD'):
            mark = self._phase_marks.get(phase, {})
            print(f'{phase}: {mark.get("frames", 0)} 帧  '
                  f'apos_x {mark.get("apos_x")}  wp_yaw_deg {mark.get("wp_yaw_deg")}  '
                  f'corr(航向, apos_x)={mark.get("corr_apos_x_vs_heading")}')
        span = (self._phase_marks.get('SWEEP_FORWARD', {}).get('apos_x', {}) or {}).get('span')
        print('\n判据：apos_x 或 wp_yaw 随航向变化 → 模型在跟踪方位（改控制映射即可）；'
              '两者 span≈0 → 疑似退化，需查 checkpoint/输入。')
        if span is not None:
            print(f'本次 apos_x 变化幅度 = {span}')
        print(f'CSV：{self.output_csv}')
        raise SystemExit(0)


def _corr(a, b) -> float:
    n = len(a)
    mean_a, mean_b = sum(a) / n, sum(b) / n
    cov = sum((x - mean_a) * (y - mean_b) for x, y in zip(a, b))
    var_a = math.sqrt(sum((x - mean_a) ** 2 for x in a))
    var_b = math.sqrt(sum((y - mean_b) ** 2 for y in b))
    if var_a == 0 or var_b == 0:
        return float('nan')
    return cov / (var_a * var_b)


def main() -> int:
    rclpy.init()
    node = YawSweepCheck()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.publish_yaw_rate(0.0)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
