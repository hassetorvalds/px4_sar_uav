"""观测覆盖节点：相机点云 → ENU 地面网格 → 覆盖率/漏扫。"""

import json
import time

import numpy as np
import rclpy
from px4_interface.frames import ned_to_enu, ned_yaw_to_enu_yaw
from px4_interface.state import Px4StateMonitor
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String

from .coverage import CoverageGrid
from .voxel_grid import filter_by_min_range, yaw_rotate_flu_to_enu


class CoverageMonitor(Node):
    def __init__(self) -> None:
        super().__init__('coverage_monitor')
        self.declare_parameter('points_topic', '/airsim_camera/points')
        self.declare_parameter('polygon_topic', '/coverage/polygon')
        self.declare_parameter('status_topic', '/coverage/status')
        self.declare_parameter('cell_size_m', 1.0)
        self.declare_parameter('ground_tol_m', 0.6)
        self.declare_parameter('min_range_m', 0.45)
        self.declare_parameter('publish_rate_hz', 2.0)

        self._cell_size = float(self.get_parameter('cell_size_m').value)
        self._ground_tol = float(self.get_parameter('ground_tol_m').value)
        self._min_range = float(self.get_parameter('min_range_m').value)
        self._grid = None
        self._frames = 0
        self._last_publish = 0.0
        self._monitor = Px4StateMonitor(self)
        self.create_subscription(
            PointCloud2, str(self.get_parameter('points_topic').value),
            self._on_points, 5)
        self.create_subscription(
            String, str(self.get_parameter('polygon_topic').value),
            self._on_polygon, 5)
        self._status_pub = self.create_publisher(
            String, str(self.get_parameter('status_topic').value), 5)
        rate = max(float(self.get_parameter('publish_rate_hz').value), 0.2)
        self.create_timer(1.0 / rate, self._publish)
        self.get_logger().info(
            f'覆盖监视启动：cell={self._cell_size:.2f} m，'
            f'ground_tol={self._ground_tol:.2f} m，等待 {self.get_parameter("polygon_topic").value}')

    def _on_polygon(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
            polygon = payload.get('polygon') if isinstance(payload, dict) else payload
            if not polygon or len(polygon) < 3:
                raise ValueError('polygon 顶点不足')
            self._grid = CoverageGrid(polygon, cell_size_m=self._cell_size,
                                      ground_tol_m=self._ground_tol)
        except (json.JSONDecodeError, ValueError, TypeError) as exc:
            self.get_logger().warn(f'忽略 polyon：{exc}')
            return
        status = self._grid.status()
        self.get_logger().info(
            f'覆盖区域：{status["total_cells"]} 格 / '
            f'{status["polygon_area_m2"]:.1f} m²')
        self._publish(force=True)

    def _on_points(self, msg: PointCloud2) -> None:
        if self._grid is None or msg.width == 0 or msg.point_step != 12:
            return
        state = self._monitor.snapshot()
        if state.position_ned is None:
            return
        body = np.frombuffer(msg.data, dtype=np.float32).reshape(-1, 3)
        body = filter_by_min_range(body, self._min_range)
        if body.size == 0:
            return
        heading = ned_yaw_to_enu_yaw(state.heading)
        origin = ned_to_enu(*state.position_ned)
        world = yaw_rotate_flu_to_enu(body, heading) + np.asarray(origin)
        self._grid.observe(world)
        self._frames += 1

    def _publish(self, force: bool = False) -> None:
        if self._grid is None:
            return
        now = time.monotonic()
        if not force and now - self._last_publish < 0.4:
            return
        self._last_publish = now
        payload = self._grid.status()
        payload['frames'] = self._frames
        payload['stamp'] = self.get_clock().now().nanoseconds / 1e9
        self._status_pub.publish(String(data=json.dumps(payload, ensure_ascii=False)))


def main(args=None):
    rclpy.init(args=args)
    node = CoverageMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
