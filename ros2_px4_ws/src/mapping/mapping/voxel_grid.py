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


def filter_by_min_range(points_flu: np.ndarray, min_range_m: float) -> np.ndarray:
    """丢弃机体坐标系下距离原点小于 ``min_range_m`` 的点。

    为什么要这道过滤（2026-10-13 实测）：
    深度相机会把**无人机自身**（起落架/机臂）以及起飞阶段的近场残点投进地图，
    地面静止时 `/local_map/status` 持续读到 `nearest_forward_m=0.17`；
    在 2 m 高度交接瞬间也读到四周 0.40–0.53 m，随后又变 `inf`。
    这些近场点会被安全层当成"障碍"，造成**无谓的归零保持**。

    取值建议：**与本机避障的停止距离一致（0.6 m）**——比停止距离更近的信息
    对决策没有价值（那时本来就该保持），却最容易被自身点污染。
    """
    if points_flu.size == 0:
        return points_flu.reshape(0, 3)
    if min_range_m <= 0.0:
        return points_flu
    ranges = np.linalg.norm(points_flu, axis=1)
    return points_flu[ranges >= float(min_range_m)]


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

    def insert(self, points_enu: np.ndarray, origin_enu, now: float,
               min_range_m: float = 0.0) -> int:
        """插入一帧点云（ENU），返回新增/更新的体素数。

        ``min_range_m`` 以**机体原点**为球心再做一次过滤，判据落在**体素中心**上：
        点级过滤挡不住量化——离原点 0.31 m 的点可能被归到中心距 0.22 m 的体素里
        （2026-10-13 实测 `nearest_forward=0.23 m` 却设了 0.3 m 过滤，就是这个原因）。
        """
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
            key = self._key(point)
            if min_range_m > 0.0:
                center = np.asarray(self._center(key), dtype=float)
                if np.linalg.norm(center - origin) < float(min_range_m):
                    continue
            self._voxels[key] = now
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
                        vertical_tolerance_m: float = 0.6,
                        vertical_down_tolerance_m: float = 0.25,
                        min_distance_m: float = 0.0) -> float:
        """前方锥形区域内最近障碍距离；无遮挡时返回 ``inf``。

        只统计与机体**同高度薄层**内的点，且**上宽下窄**：

        - ``vertical_tolerance_m``（向上，默认 0.6 m）保留障碍在机体上方的部分；
        - ``vertical_down_tolerance_m``（向下，默认 0.25 m）**收紧**，用于排除地面：
          低空/贴地时光轴下方的地面点会被算成"正前方 0.5 m 障碍"
          （2026-10-13 地面静止实测 `nearest_forward_m=0.17~0.55`，就是这个原因）。
        """
        centers = self.centers()
        if centers.shape[0] == 0:
            return float('inf')
        delta = centers - np.asarray(origin_enu, dtype=float)
        distance = np.linalg.norm(delta, axis=1)
        keep = (distance >= max(float(min_distance_m), 1e-6)) & (distance <= max_distance)
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
        vertical_ok = ((delta[:, 2] <= float(vertical_tolerance_m))
                       & (delta[:, 2] >= -float(vertical_down_tolerance_m)))
        in_cone = (cos_angle >= cos_limit) & vertical_ok
        if not np.any(in_cone):
            return float('inf')
        return float(distance[in_cone].min())

    def nearest_point_in_cone(self, origin_enu, heading_enu: float,
                              half_angle_deg: float = 30.0,
                              max_distance: float = 6.0,
                              vertical_tolerance_m: float = 0.6,
                              vertical_down_tolerance_m: float = 0.25,
                              min_distance_m: float = 0.0):
        """与 ``nearest_in_cone`` 同样的判据，但返回最近点本身（用于诊断"障碍在哪"）。

        返回 ``(distance_m, (dx, dy, dz))``；无命中返回 ``(inf, None)``。
        诊断价值（2026-10-13）：只看距离无法区分"正前方 0.7 m 的真墙"、
        "机体自身近场回波"和"光轴下方的地面"，看相对坐标就一目了然。
        """
        centers = self.centers()
        if centers.shape[0] == 0:
            return float('inf'), None
        delta = centers - np.asarray(origin_enu, dtype=float)
        distance = np.linalg.norm(delta, axis=1)
        keep = (distance >= max(float(min_distance_m), 1e-6)) & (distance <= max_distance)
        if not np.any(keep):
            return float('inf'), None
        delta, distance = delta[keep], distance[keep]
        cos_limit = math.cos(math.radians(half_angle_deg))
        forward = np.array([math.cos(heading_enu), math.sin(heading_enu), 0.0])
        horizontal_norm = np.linalg.norm(delta[:, :2], axis=1)
        cos_angle = np.divide(delta[:, :2] @ forward[:2], horizontal_norm,
                              out=np.zeros_like(horizontal_norm),
                              where=horizontal_norm > 1e-6)
        vertical_ok = ((delta[:, 2] <= float(vertical_tolerance_m))
                       & (delta[:, 2] >= -float(vertical_down_tolerance_m)))
        in_cone = (cos_angle >= cos_limit) & vertical_ok
        if not np.any(in_cone):
            return float('inf'), None
        index = int(np.argmin(np.where(in_cone, distance, np.inf)))
        return float(distance[index]), tuple(float(v) for v in delta[index])

    def clearance_profile(self, origin_enu, heading_enu: float,
                          half_angle_deg: float = 30.0,
                          max_distance: float = 6.0,
                          vertical_tolerance_m: float = 0.6,
                          vertical_down_tolerance_m: float = 0.25,
                          min_distance_m: float = 0.0):
        """返回 (前方, 左方, 右方) 最近障碍距离；左右按航向 ±90° 锥统计。

        用于决定绕障往哪一侧：更宽的一侧更安全。
        """
        forward = self.nearest_in_cone(
            origin_enu, heading_enu, half_angle_deg, max_distance,
            vertical_tolerance_m, vertical_down_tolerance_m, min_distance_m)
        left = self.nearest_in_cone(
            origin_enu, heading_enu + math.pi / 2, half_angle_deg,
            max_distance, vertical_tolerance_m, vertical_down_tolerance_m, min_distance_m)
        right = self.nearest_in_cone(
            origin_enu, heading_enu - math.pi / 2, half_angle_deg,
            max_distance, vertical_tolerance_m, vertical_down_tolerance_m, min_distance_m)
        return forward, left, right
