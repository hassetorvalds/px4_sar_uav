#!/usr/bin/env python3
"""列出 AirSim 场景物体及其与无人机起点的相对位置（用于挑选"有限跨度"的避障目标）。

服务器侧 2026-10-13 的建议：不要向服务端要场景坐标——**方向信息在 AirSim 真值里**。
本工具把场景物体按"与无人机当前/起点位置的距离"排序打印，并给出 ENU 相对坐标
（本项目 ROS 侧用 ENU：x 东、y 北、z 上；AirSim 物体位姿是 NED：x 北、y 东、z 下）。

用法：
    source scripts/env.sh
    python3 scripts/list_scene_objects.py --ros-args -p top_n:=25 -p max_distance_m:=40
"""

import math
import sys

import rclpy
import airsim
from rclpy.node import Node
from px4_msgs.msg import VehicleLocalPosition
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)


class SceneLister(Node):
    def __init__(self) -> None:
        super().__init__('scene_lister')
        self.declare_parameter('airsim_ip', '172.21.192.1')
        self.declare_parameter('airsim_port', 45000)
        self.declare_parameter('top_n', 25)
        self.declare_parameter('max_distance_m', 60.0)
        self.declare_parameter('min_size_hint_m', 0.0)
        self.local_position = None
        self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position_v1',
            self._on_position, QOS)

    def _on_position(self, msg: VehicleLocalPosition) -> None:
        if msg.xy_valid and msg.z_valid:
            self.local_position = (msg.x, msg.y, msg.z)

    def run(self) -> int:
        client = airsim.MultirotorClient(
            ip=str(self.get_parameter('airsim_ip').value),
            port=int(self.get_parameter('airsim_port').value),
            timeout_value=15)
        client.confirmConnection()

        # 等到拿到 PX4 本地位置（作为"相对原点"）
        for _ in range(100):
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.local_position is not None:
                break
        if self.local_position is None:
            print('未收到 PX4 本地位置，改用 AirSim 无人机位姿作原点')
            pose = client.simGetVehiclePose()
            origin_airsim = (pose.position.x_val, pose.position.y_val, pose.position.z_val)
        else:
            # PX4 NED → AirSim NED 近似同一原点（PX4 home 位于无人机起点）
            origin_airsim = (self.local_position[0], self.local_position[1],
                             self.local_position[2])

        names = sorted(client.simListSceneObjects())
        top_n = int(self.get_parameter('top_n').value)
        max_distance = float(self.get_parameter('max_distance_m').value)
        rows = []
        for name in names:
            try:
                pose = client.simGetObjectPose(name)
            except Exception:  # noqa: BLE001
                continue
            p = pose.position
            if not all(math.isfinite(v) for v in (p.x_val, p.y_val, p.z_val)):
                continue
            dx_ned = p.x_val - origin_airsim[0]
            dy_ned = p.y_val - origin_airsim[1]
            dz_ned = p.z_val - origin_airsim[2]
            # NED（x 北、y 东、z 下）→ ENU（x 东、y 北、z 上）
            enu_east, enu_north, enu_up = dy_ned, dx_ned, -dz_ned
            horizontal = math.hypot(enu_east, enu_north)
            if horizontal > max_distance:
                continue
            rows.append((horizontal, name, enu_east, enu_north, enu_up))

        rows.sort()
        print(f'场景物体 {len(names)} 个；距原点 ≤{max_distance:.0f} m 的 {len(rows)} 个'
              f'（原点 AirSim NED=({origin_airsim[0]:.2f}, {origin_airsim[1]:.2f}, '
              f'{origin_airsim[2]:.2f})）')
        print(f'{"距离m":>7s} {"物体名":<38s} {"ENU 东":>8s} {"ENU 北":>8s} {"ENU 上":>8s}')
        for horizontal, name, east, north, up in rows[:top_n]:
            print(f'{horizontal:7.2f} {name[:38]:<38s} {east:8.2f} {north:8.2f} {up:8.2f}')
        return 0


def main() -> int:
    rclpy.init()
    node = SceneLister()
    try:
        return node.run()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
