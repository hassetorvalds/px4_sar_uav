#!/usr/bin/env python3
"""地图自身回波判别：**慢速偏航 90°，看近场读数相对机体还是相对世界**。

服务器侧 2026-10-13 建议的判据（最稳、一次见效）：

```text
自身点（相机看到自己的结构） → 相对**机体**几乎不动（机体系偏移恒定）
真实障碍                     → 相对**世界**固定（ENU 偏移恒定，机体系里会随航向扫动）
```

本脚本：起飞到 2 m → 以 `yaw_rate_deg_s` 慢速偏航 `yaw_s` 秒 → 记录
`/local_map/status` 的最近点（ENU 偏移）与航向/位置 → 打印两种偏移的离散度并落盘 CSV。

用法：
    python3 scripts/map_selfcheck.py --ros-args -p yaw_rate_deg_s:=7.5 -p yaw_s:=12.0 \
        -p output_csv:=logs/2026-10-13/map_selfcheck.csv
"""

import csv
import math
import os
import statistics
import sys

import rclpy
from flight_check import FlightCheckBase
from geometry_msgs.msg import Twist
from px4_interface.frames import ned_yaw_to_enu_yaw
from std_msgs.msg import String
from vlm.safety import parse_clearance_status

sys.stdout.reconfigure(line_buffering=True)


class MapSelfCheck(FlightCheckBase):
    def __init__(self) -> None:
        super().__init__('map_selfcheck')
        self.declare_parameter('start_altitude_m', 2.0)
        self.declare_parameter('yaw_rate_deg_s', 7.5)
        self.declare_parameter('yaw_s', 12.0)
        self.declare_parameter('map_status_topic', '/local_map/status')
        self.declare_parameter('output_csv', 'logs/map_selfcheck.csv')
        self.declare_parameter('land_at_end', True)

        self.altitude = float(self.get_parameter('start_altitude_m').value)
        self.yaw_rate = math.radians(float(self.get_parameter('yaw_rate_deg_s').value))
        self.yaw_s = float(self.get_parameter('yaw_s').value)
        self.output_csv = str(self.get_parameter('output_csv').value)
        self.land_at_end = bool(self.get_parameter('land_at_end').value)

        self.rows = []
        self.status_text = ''
        self.nearest = None          # (距离, (dx, dy, dz)) ENU 相对机体原点
        self.heading_rad = float('nan')
        self.px4_x = float('nan')
        self.px4_y = float('nan')
        self.create_subscription(
            String, str(self.get_parameter('map_status_topic').value),
            self._on_map_status, 10)

    def _on_map_status(self, msg: String) -> None:
        """注意：**不能**叫 `_on_status`——基类用它订阅 VehicleStatus（今天踩过）。"""
        self.status_text = msg.data
        # 形如 nearest_fwd=(-0.18,0.12,0.02)@0.23m
        if 'nearest_fwd=(' not in msg.data:
            self.nearest = None
            return
        try:
            body = msg.data.split('nearest_fwd=(')[1].split(')')[0]
            dx, dy, dz = (float(v) for v in body.split(','))
            distance = float(msg.data.split('@')[1].split('m')[0])
        except (IndexError, ValueError):
            self.nearest = None
            return
        self.nearest = (distance, (dx, dy, dz))

    def _on_position(self, msg) -> None:
        super()._on_position(msg)
        if getattr(msg, 'xy_valid', True):
            self.px4_x, self.px4_y = float(msg.x), float(msg.y)
        heading = getattr(msg, 'heading', None)
        if heading is not None and not math.isnan(heading):
            self.heading_rad = float(heading)

    def publish_yaw_rate(self, rate: float) -> None:
        msg = Twist()
        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.z = float(rate)
        self.velocity_pub.publish(msg)

    def first_phase(self) -> str:
        return 'CLIMB'

    def on_phase(self) -> None:
        if self.phase == 'CLIMB':
            if self.climb_to(self.altitude):
                self.get_logger().info(
                    f'到达 {self.altitude:.1f} m，开始慢速偏航 '
                    f'{math.degrees(self.yaw_rate):.1f}°/s × {self.yaw_s:.0f} s')
                self.enter('YAW')
            elif self.elapsed() > self.phase_timeout:
                self.fail('爬升超时')
            return

        if self.phase == 'YAW':
            self.publish_yaw_rate(self.yaw_rate)
            self.record()
            if self.elapsed() > self.yaw_s:
                self.publish_yaw_rate(0.0)
                self.enter('SETTLE')
            return

        if self.phase == 'SETTLE':
            self.publish_yaw_rate(0.0)
            self.record()
            if self.elapsed() > 2.0:
                self.finish()

    def record(self) -> None:
        # 即使没有命中点也要记一行（否则无法区分"没数据"和"没障碍"）——今天踩过
        if self.nearest is None:
            if self.status_text:
                self.rows.append({
                    't': round(self.elapsed(), 2),
                    'heading_deg': round(math.degrees(self.heading_rad), 1),
                    'distance_m': None, 'world_dx': None, 'world_dy': None, 'world_dz': None,
                    'body_forward': None, 'body_left': None,
                    'status': self.status_text.strip(),
                })
            return
        distance, (dx, dy, dz) = self.nearest
        heading_enu = ned_yaw_to_enu_yaw(self.heading_rad)
        # ENU 偏移 → 机体系（逆旋转）
        body_x = dx * math.cos(heading_enu) + dy * math.sin(heading_enu)
        body_y = -dx * math.sin(heading_enu) + dy * math.cos(heading_enu)
        self.rows.append({
            't': round(self.elapsed(), 2),
            'heading_deg': round(math.degrees(self.heading_rad), 1),
            'distance_m': round(distance, 3),
            'world_dx': round(dx, 3), 'world_dy': round(dy, 3), 'world_dz': round(dz, 3),
            'body_forward': round(body_x, 3), 'body_left': round(body_y, 3),
            'status': self.status_text.strip(),
        })

    def finish(self) -> None:
        if self.output_csv and self.rows:
            directory = os.path.dirname(self.output_csv)
            if directory:
                os.makedirs(directory, exist_ok=True)
            with open(self.output_csv, 'w', newline='', encoding='utf-8') as handle:
                writer = csv.DictWriter(handle, fieldnames=list(self.rows[0].keys()))
                writer.writeheader()
                writer.writerows(self.rows)

        print('\n=========== 地图自身回波判别 ===========')
        print(f'样本 {len(self.rows)}（慢速偏航 {self.yaw_s:.0f} s）')
        valid = [r for r in self.rows if r.get('distance_m') is not None]
        hits = len(valid)
        print(f'其中"命中前向锥"的样本 {hits}/{len(self.rows)}')
        if hits >= 3:
            def spread(key):
                values = [r[key] for r in valid]
                return statistics.pstdev(values)
            print(f'世界系(ENU)偏移离散度  dx σ={spread("world_dx"):.3f}  dy σ={spread("world_dy"):.3f}')
            print(f'机体系偏移离散度       fwd σ={spread("body_forward"):.3f}  '
                  f'left σ={spread("body_left"):.3f}')
            print('\n判据：机体系离散度明显更小 → 自身回波；世界系离散度更小 → 真实障碍。')
        elif hits == 0:
            print('本轮前向锥内**没有任何点**（nearest=inf）→ 干净环境下不存在近场伪点。')
        if self.land_at_end:
            self.send('land_fast')
            print('已下发 land_fast。')
        print(f'CSV：{self.output_csv}')
        raise SystemExit(0)


def main() -> int:
    rclpy.init()
    node = MapSelfCheck()
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
