"""局部障碍地图节点：深度点云（机体 FLU）→ ENU 体素地图 + 前方最近障碍。

输入：`/airsim_camera/points`（sensor_msgs/PointCloud2，机体 FLU）
      PX4 状态（位置/航向，来自 px4_interface 的 Px4StateMonitor）
输出：`~/occupied`（PointCloud2，占据体素中心，ENU/map 系）
      `~/status`（String：体素数、前方最近障碍距离）

注意（v1 简化）：假设相机与机体同向、无安装偏移；地图为机体附近的局部地图，
超出 extent 的点直接丢弃。
"""

import numpy as np
import rclpy
from px4_interface.frames import ned_to_enu, ned_yaw_to_enu_yaw
from px4_interface.state import Px4StateMonitor
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import String

from .voxel_grid import VoxelGrid, filter_by_min_range, yaw_rotate_flu_to_enu


class LocalMap(Node):
    def __init__(self) -> None:
        super().__init__('local_map')
        self.declare_parameter('points_topic', '/airsim_camera/points')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('resolution', 0.15)
        self.declare_parameter('extent', 8.0)
        self.declare_parameter('height_min', -2.0)
        self.declare_parameter('height_max', 6.0)
        self.declare_parameter('decay_s', 5.0)
        self.declare_parameter('publish_rate_hz', 1.0)
        self.declare_parameter('cone_deg', 30.0)
        self.declare_parameter('cone_max_distance', 6.0)
        # 诊断用宽锥（服务器侧 2026-10-14 建议 A）：只多发布一个 nearest_wide，不影响原有判定
        self.declare_parameter('cone_wide_deg', 35.0)
        # 锥内连通簇质心的最小体素数（服务器侧"限定 ②"）：太小会锁到零散噪声
        self.declare_parameter('cluster_min_points', 3)
        self.declare_parameter('cone_vertical_tolerance_m', 0.6)
        # 向下的竖直窗要**更紧**，否则贴地/低空时地面点会被算成"正前方障碍"
        self.declare_parameter('cone_down_tolerance_m', 0.25)
        # 近场过滤：丢掉相机看到的"自身/起飞残点"（详见 voxel_grid.filter_by_min_range）。
        # **必须远低于保持阈值**（当前 0.6 m），否则会把触发"保持"所需的数据也删掉——
        # 服务器侧 2026-10-13 指出过这个死结，故取 0.3 m。
        self.declare_parameter('min_range_m', 0.45)

        self._frame_id = str(self.get_parameter('frame_id').value)
        self._cone_deg = float(self.get_parameter('cone_deg').value)
        self._cone_wide_deg = float(self.get_parameter('cone_wide_deg').value)
        self._cluster_min_points = int(self.get_parameter('cluster_min_points').value)
        self._cone_max = float(self.get_parameter('cone_max_distance').value)
        self._cone_vertical = float(
            self.get_parameter('cone_vertical_tolerance_m').value)
        self._cone_down = float(self.get_parameter('cone_down_tolerance_m').value)
        self._min_range = float(self.get_parameter('min_range_m').value)

        self._grid = VoxelGrid(
            resolution=float(self.get_parameter('resolution').value),
            extent=float(self.get_parameter('extent').value),
            height_min=float(self.get_parameter('height_min').value),
            height_max=float(self.get_parameter('height_max').value),
            decay_s=float(self.get_parameter('decay_s').value))

        self._monitor = Px4StateMonitor(self)
        self.create_subscription(
            PointCloud2, str(self.get_parameter('points_topic').value),
            self._on_points, 5)
        self._cloud_pub = self.create_publisher(PointCloud2, '~/occupied', 5)
        self._status_pub = self.create_publisher(String, '~/status', 5)

        rate = max(float(self.get_parameter('publish_rate_hz').value), 0.2)
        self.create_timer(1.0 / rate, self._tick)
        self._frames = 0
        self._dropped_near = 0          # 上一帧被近场过滤丢弃的点数
        self._dropped_min_m = float('inf')  # 被丢弃点中的最小距离（诊断用）
        self.get_logger().info(
            f'局部地图启动：分辨率 {self._grid.resolution:.2f} m，'
            f'范围 ±{self._grid.extent:.1f} m，衰减 {self._grid.decay_s:.0f} s，'
            f'前方锥 {self._cone_deg:.0f}° / {self._cone_max:.0f} m')

    # ------------------------------------------------------------------ 输入

    def _on_points(self, msg: PointCloud2) -> None:
        """把点云（机体 FLU）转换到 ENU 世界并插入体素地图。"""
        if msg.width == 0:
            return
        state = self._monitor.snapshot()
        position_ned = state.position_ned
        if position_ned is None:
            return
        # 本项目自产点云固定为 3×float32（x,y,z），据此直接解析
        if msg.point_step != 12:
            self.get_logger().warn('点云格式非 3×float32，已忽略', throttle_duration_sec=10.0)
            return
        body = np.frombuffer(msg.data, dtype=np.float32).reshape(-1, 3)
        if body.size == 0:
            return
        raw_count = int(body.shape[0])
        if self._min_range > 0.0 and raw_count:
            ranges = np.linalg.norm(body, axis=1)
            dropped = ranges < self._min_range
            self._dropped_near = int(np.count_nonzero(dropped))
            self._dropped_min_m = (float(ranges[dropped].min())
                                   if np.any(dropped) else float('inf'))
        body = filter_by_min_range(body, self._min_range)
        if body.size == 0:
            return
        if raw_count != int(body.shape[0]):
            self.get_logger().info(
                f'近场过滤：丢弃 {raw_count - int(body.shape[0])} 个 '
                f'<{self._min_range:.2f} m 的点',
                throttle_duration_sec=10.0)
        heading_enu = ned_yaw_to_enu_yaw(state.heading)
        origin_enu = ned_to_enu(*position_ned)
        world_local = yaw_rotate_flu_to_enu(body, heading_enu) + np.asarray(origin_enu)
        now = self.get_clock().now().nanoseconds / 1e9
        self._grid.insert(world_local, origin_enu=origin_enu, now=now,
                          min_range_m=self._min_range)
        self._frames += 1

    # ------------------------------------------------------------------ 输出

    def _tick(self) -> None:
        now = self.get_clock().now().nanoseconds / 1e9
        self._grid.decay(now)
        centers = self._grid.centers()

        cloud = PointCloud2()
        cloud.header.stamp = self.get_clock().now().to_msg()
        cloud.header.frame_id = self._frame_id
        cloud.height = 1
        cloud.width = int(centers.shape[0])
        cloud.is_bigendian = False
        cloud.is_dense = False
        cloud.point_step = 12
        cloud.row_step = 12 * cloud.width
        cloud.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        cloud.data = centers.astype(np.float32).tobytes()
        self._cloud_pub.publish(cloud)

        state = self._monitor.snapshot()
        position_ned = state.position_ned
        if position_ned is None:
            forward = left = right = float('inf')
            nearest_text = 'nearest_fwd=inf'      # 还没收到 PX4 位置时也要有值（否则报错）
            wide_text = 'nearest_wide=inf'
            centroid_text = 'nearest_centroid=inf'
        else:
            forward, left, right = self._grid.clearance_profile(
                ned_to_enu(*position_ned), ned_yaw_to_enu_yaw(state.heading),
                half_angle_deg=self._cone_deg, max_distance=self._cone_max,
                vertical_tolerance_m=self._cone_vertical,
                vertical_down_tolerance_m=self._cone_down,
                min_distance_m=self._min_range)
            nearest_d, nearest_delta = self._grid.nearest_point_in_cone(
                ned_to_enu(*position_ned), ned_yaw_to_enu_yaw(state.heading),
                half_angle_deg=self._cone_deg, max_distance=self._cone_max,
                vertical_tolerance_m=self._cone_vertical,
                vertical_down_tolerance_m=self._cone_down,
                min_distance_m=self._min_range)
            if nearest_delta is None:
                nearest_text = 'nearest_fwd=inf'
            else:
                nearest_text = ('nearest_fwd=(%.2f,%.2f,%.2f)@%.2fm'
                                % (nearest_delta[0], nearest_delta[1], nearest_delta[2],
                                   nearest_d))
            # 宽锥最近点（诊断用，服务器侧 2026-10-14 建议的 A）：
            # 微转复验"窄锥看不到"时，用宽锥区分"真实但被转出窄锥"与"本来就是假的"。
            wide_d, wide_delta = self._grid.nearest_point_in_cone(
                ned_to_enu(*position_ned), ned_yaw_to_enu_yaw(state.heading),
                half_angle_deg=self._cone_wide_deg, max_distance=self._cone_max,
                vertical_tolerance_m=self._cone_vertical,
                vertical_down_tolerance_m=self._cone_down,
                min_distance_m=self._min_range)
            wide_text = ('nearest_wide=inf' if wide_delta is None else
                         'nearest_wide=(%.2f,%.2f,%.2f)@%.2fm'
                         % (wide_delta[0], wide_delta[1], wide_delta[2], wide_d))
            # 锥内**连通簇质心**（服务器侧 2026-10-14 建议的"限定 ②"）：
            # 用"物体的质心"代替"锥内最近点"，避免最近点在大平面上随姿态滑移。
            (cluster_delta, cluster_size, cluster_dist,
             cluster_zmin, cluster_zmax, cluster_span) = self._grid.cluster_centroid_in_cone(
                ned_to_enu(*position_ned), ned_yaw_to_enu_yaw(state.heading),
                half_angle_deg=self._cone_deg, max_distance=self._cone_max,
                vertical_tolerance_m=self._cone_vertical,
                vertical_down_tolerance_m=self._cone_down,
                min_distance_m=self._min_range,
                min_cluster_points=self._cluster_min_points)
            centroid_text = ('nearest_centroid=inf' if cluster_delta is None else
                             'nearest_centroid=(%.2f,%.2f,%.2f)@%.2fm@%d'
                             % (cluster_delta[0], cluster_delta[1], cluster_delta[2],
                                cluster_dist, cluster_size)
                             + ('' if cluster_delta is None else
                                ' cluster_z=%.2f,%.2f span_h=%.2f'
                                % (cluster_zmin, cluster_zmax, cluster_span)))
        status = String()
        status.data = (f'voxels={self._grid.voxel_count} frames={self._frames} '
                       f'nearest_forward_m={forward:.2f} '
                       f'left_m={left:.2f} right_m={right:.2f} '
                       f'{nearest_text} {wide_text} {centroid_text} '
                       # 净空读数的**地板值**：上报值等于它时表示"触底（≤该值，未知）"，
                       # 不是"还有这么多余量"（服务器侧 2026-10-15 要求落盘区分）
                       f'fwd_floor_m={self._min_range:.2f} '
                       f'dropped_near={self._dropped_near} dropped_min_m={self._dropped_min_m:.2f}')
        self._status_pub.publish(status)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LocalMap()
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
