#!/usr/bin/env python3
"""深度图找障碍：悬停慢速偏航 360°，**直接看深度图**判断哪一侧有结构、多远。

为什么绕过体素地图（服务器侧 2026-10-14 建议 ②）：
体素地图会累积运动/姿态拖影，而深度图是**当帧**数据；配合"排除光轴以下地面"与
"前向锥内"，判据更干净。用它对"附近到底有没有建筑、在哪个方位"做一次确定性判断，
再决定是"原地找"还是"朝建筑飞过去找"。

判据（每帧）：

```text
视野上 60% 行、中心 ±15% 列（避开地面与边缘）里的有效深度：
  depth_median_m   中位深度（"正前方这堵墙/建筑有多远"）
  depth_p10_m      10 分位（最近的结构面）
  frac_near_5m     深度 < 5 m 的像素占比（"前方有东西挡住"的比例）
```

跑完按 30° 扇区汇总 `depth_median_m`，打印**由近到远**的扇区排行——最小的扇区就是"最可能有建筑"的方位。

用法：
    source scripts/env.sh
    python3 scripts/depth_seek.py --ros-args -p yaw_rate_deg_s:=15.0 -p output_csv:=logs/depth_seek.csv
"""

import csv
import math
import os
import statistics
import sys

import numpy as np
import rclpy
from flight_check import FlightCheckBase
from geometry_msgs.msg import Twist
from px4_interface.frames import ned_yaw_to_enu_yaw
from sensor_msgs.msg import Image
from std_msgs.msg import String

sys.stdout.reconfigure(line_buffering=True)


class DepthSeek(FlightCheckBase):
    def __init__(self) -> None:
        super().__init__('depth_seek')
        self.declare_parameter('start_altitude_m', 2.0)
        self.declare_parameter('yaw_rate_deg_s', 15.0)
        self.declare_parameter('sweep_s', 26.0)
        self.declare_parameter('depth_topic', '/airsim_camera/depth')
        self.declare_parameter('output_csv', 'logs/depth_seek.csv')
        self.declare_parameter('land_at_end', True)

        self.altitude = float(self.get_parameter('start_altitude_m').value)
        self.yaw_rate = math.radians(float(self.get_parameter('yaw_rate_deg_s').value))
        self.sweep_s = float(self.get_parameter('sweep_s').value)
        self.output_csv = str(self.get_parameter('output_csv').value)
        self.land_at_end = bool(self.get_parameter('land_at_end').value)

        self.rows = []
        self.heading_rad = float('nan')
        self._last_heading = None
        self._turned_deg = 0.0
        self.create_subscription(
            Image, str(self.get_parameter('depth_topic').value), self._on_depth, 2)
        # 心跳：否则落地监督层会在 11 s 后判超时并下发 land_fast（今天踩到）
        self.declare_parameter('heartbeat_topic', '/flight_heartbeat')
        self.heartbeat_pub = self.create_publisher(
            String, str(self.get_parameter('heartbeat_topic').value), 10)
        self.create_timer(0.5, self._beat)

    def _beat(self) -> None:
        msg = String()
        msg.data = f'depth_seek {self.phase} t={self.elapsed():.1f}'
        self.heartbeat_pub.publish(msg)

    def _on_depth(self, msg: Image) -> None:
        if msg.encoding != '32FC1' or self.phase != 'SWEEP':
            return
        depth = np.frombuffer(msg.data, dtype=np.float32)
        if depth.size < int(msg.width) * int(msg.height):
            return
        image = depth.reshape(int(msg.height), int(msg.width))
        height, width = image.shape
        patch = image[int(height * 0.05):int(height * 0.60),
                      int(width * 0.35):int(width * 0.65)]
        valid = patch[np.isfinite(patch)]
        if valid.size == 0:
            return
        self.rows.append({
            't': round(self.elapsed(), 2),
            'heading_enu_deg': round(math.degrees(ned_yaw_to_enu_yaw(self.heading_rad)), 1),
            'depth_median_m': round(float(np.median(valid)), 2),
            'depth_p10_m': round(float(np.percentile(valid, 10)), 2),
            'frac_near_5m': round(float(np.mean(valid < 5.0)), 3),
        })

    def _on_position(self, msg) -> None:
        super()._on_position(msg)
        heading = getattr(msg, 'heading', None)
        if heading is None or math.isnan(heading):
            return
        self.heading_rad = float(heading)
        if self._last_heading is not None:
            self._turned_deg += math.degrees(
                math.remainder(self.heading_rad - self._last_heading, 2 * math.pi))
        self._last_heading = self.heading_rad

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
                    f'到达 {self.altitude:.1f} m，开始深度扫测 '
                    f'{math.degrees(self.yaw_rate):.1f}°/s × {self.sweep_s:.0f} s')
                self.enter('SWEEP')
            elif self.elapsed() > self.phase_timeout:
                self.fail('爬升超时')
            return

        if self.phase == 'SWEEP':
            self.publish_yaw_rate(self.yaw_rate)
            if self.elapsed() > self.sweep_s:
                self.publish_yaw_rate(0.0)
                self.finish()

    def finish(self) -> None:
        if self.output_csv and self.rows:
            directory = os.path.dirname(self.output_csv)
            if directory:
                os.makedirs(directory, exist_ok=True)
            with open(self.output_csv, 'w', newline='', encoding='utf-8') as handle:
                writer = csv.DictWriter(handle, fieldnames=list(self.rows[0].keys()))
                writer.writeheader()
                writer.writerows(self.rows)

        print('\n=========== 深度扫测汇总（30° 扇区）===========')
        print(f'样本 {len(self.rows)}，累计转过 {self._turned_deg:.0f}°')
        sectors = {}
        for row in self.rows:
            sector = int(row['heading_enu_deg'] // 30) * 30
            sectors.setdefault(sector, []).append(row['depth_median_m'])
        ranking = sorted(((statistics.median(v), k, len(v)) for k, v in sectors.items()))
        print(f'{"扇区(ENU)":>10s} {"中位深度 m":>10s} {"样本":>5s}')
        for depth, sector, count in ranking:
            print(f'{sector:>10d} {depth:>10.2f} {count:>5d}')
        if ranking:
            print(f'\n最近扇区：{ranking[0][1]}°（中位 {ranking[0][0]:.2f} m）——'
                  '若明显小于其他扇区，说明该方位有结构（建筑/墙），可朝它转场。')
        if self.land_at_end:
            self.send('land_fast')
            print('已下发 land_fast。')
        raise SystemExit(0)


def main() -> int:
    rclpy.init()
    node = DepthSeek()
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
