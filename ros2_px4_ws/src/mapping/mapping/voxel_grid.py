"""局部体素障碍地图（纯逻辑，便于单元测试）。

设计要点：

- 以**机体附近**的一个立方体范围为地图边界，超出范围的点直接丢弃（局部地图）；
- 每个体素记录“最后被击中时间”，超时自动清除（应对动态障碍与传感器噪声）；
- 支持按机体位置整体平移（重定心），避免地图随飞行无限增长；
- 提供“前方锥形区域内最近障碍距离”，供安全层使用。

坐标约定：**ENU 世界系**（x 东、y 北、z 上），与本项目 ROS 侧约定一致。
"""

import math

import numpy as np


def yaw_rotate_flu_to_enu(points_flu: np.ndarray, heading_enu: float) -> np.ndarray:
    """机体 FLU 点 → ENU 世界（绕 z 轴旋转，忽略相机安装偏移）。

    ``heading_enu`` 为 ENU 偏航（自东逆时针）。
    """
    if points_flu.size == 0:
        return points_flu.reshape(0, 3)
    cos_h = math.cos(heading_enu)
    sin_h = math.sin(heading_enu)
    forward = points_flu[:, 0]
    left = points_flu[:, 1]
    horizontal = np.stack([forward * cos_h - left * sin_h,
                           forward * sin_h + left * cos_h], axis=1)
    return np.concatenate([horizontal, points_flu[:, 2:3]], axis=1)


class VoxelGrid:
    """带时间衰减的局部体素栅格。"""

    def __init__(self, resolution: float = 0.15, extent: float = 8.0,
                 height_min: float = -2.0, height_max: float = 6.0,
                 decay_s: float = 5.0):
        self.resolution = float(resolution)
        self.extent = float(extent)
        self.height_min = float(height_min)
        self.height_max = float(height_max)
        self.decay_s = float(decay_s)
        self._voxels = {}          # (i, j, k) → 最后命中时间

    # ------------------------------------------------------------------ 坐标

    def _key(self, point) -> tuple:
        r = self.resolution
        return (int(math.floor(point[0] / r)),
                int(math.floor(point[1] / r)),
                int(math.floor(point[2] / r)))

    def _center(self, key: tuple) -> tuple:
        r = self.resolution
        return ((key[0] + 0.5) * r, (key[1] + 0.5) * r, (key[2] + 0.5) * r)

    # ------------------------------------------------------------------ 更新

    def insert(self, points_enu: np.ndarray, origin_enu, now: float) -> int:
        """插入一帧点云（ENU），返回新增/更新的体素数。"""
        if points_enu.size == 0:
            return 0
        origin = np.asarray(origin_enu, dtype=float)
        local = points_enu - origin
        inside = ((np.abs(local[:, 0]) <= self.extent)
                  & (np.abs(local[:, 1]) <= self.extent)
                  & (local[:, 2] >= self.height_min)
                  & (local[:, 2] <= self.height_max))
        updated = 0
        for point in points_enu[inside]:
            self._voxels[self._key(point)] = now
            updated += 1
        return updated

    def decay(self, now: float) -> int:
        """清除超时体素，返回清除数量。"""
        expired = [key for key, stamp in self._voxels.items()
                   if now - stamp > self.decay_s]
        for key in expired:
            del self._voxels[key]
        return len(expired)

    # ------------------------------------------------------------------ 查询

    @property
    def voxel_count(self) -> int:
        return len(self._voxels)

    def centers(self) -> np.ndarray:
        """已占体素的中心点（ENU），形状 (N, 3)。"""
        if not self._voxels:
            return np.zeros((0, 3), dtype=float)
        return np.array([self._center(key) for key in self._voxels], dtype=float)

    def nearest_in_cone(self, origin_enu, heading_enu: float,
                        half_angle_deg: float = 30.0,
                        max_distance: float = 6.0,
                        vertical_tolerance_m: float = 0.6) -> float:
        """前方锥形区域内最近障碍距离；无遮挡时返回 ``inf``。

        ``vertical_tolerance_m`` 限制只统计与机体**同高度薄层**内的点：
        否则低空飞行时下方地面会一直被判为"正前方障碍"（实测 0.8 m 高度时
        地面恒定贡献约 0.9 m 的净空读数，导致飞机无法正常前进）。
        """
        centers = self.centers()
        if centers.shape[0] == 0:
            return float('inf')
        delta = centers - np.asarray(origin_enu, dtype=float)
        distance = np.linalg.norm(delta, axis=1)
        keep = (distance > 1e-6) & (distance <= max_distance)
        if not np.any(keep):
            return float('inf')
        delta, distance = delta[keep], distance[keep]
        cos_limit = math.cos(math.radians(half_angle_deg))
        forward = np.array([math.cos(heading_enu), math.sin(heading_enu), 0.0])
        # 水平方向夹角（忽略高度差）与垂直夹角都要落在锥内
        horizontal_norm = np.linalg.norm(delta[:, :2], axis=1)
        cos_angle = np.divide(delta[:, :2] @ forward[:2], horizontal_norm,
                              out=np.zeros_like(horizontal_norm),
                              where=horizontal_norm > 1e-6)
        vertical_ok = np.abs(delta[:, 2]) <= float(vertical_tolerance_m)
        in_cone = (cos_angle >= cos_limit) & vertical_ok
        if not np.any(in_cone):
            return float('inf')
        return float(distance[in_cone].min())

    def clearance_profile(self, origin_enu, heading_enu: float,
                          half_angle_deg: float = 30.0,
                          max_distance: float = 6.0,
                          vertical_tolerance_m: float = 0.6):
        """返回 (前方, 左方, 右方) 最近障碍距离；左右按航向 ±90° 锥统计。

        用于决定绕障往哪一侧：更宽的一侧更安全。
        """
        forward = self.nearest_in_cone(
            origin_enu, heading_enu, half_angle_deg, max_distance,
            vertical_tolerance_m)
        left = self.nearest_in_cone(
            origin_enu, heading_enu + math.pi / 2, half_angle_deg,
            max_distance, vertical_tolerance_m)
        right = self.nearest_in_cone(
            origin_enu, heading_enu - math.pi / 2, half_angle_deg,
            max_distance, vertical_tolerance_m)
        return forward, left, right
