"""Frontier 探索的纯逻辑（不依赖 ROS，可单测）。

思路（v1，够用为先）：

- 维护一张**机体附近的 2D 栅格**：`occupied` 来自障碍点云投影，`free` 来自机体轨迹附近
  的“已确认可通过”区域（简化：机体走过的位置周围一圈视为已知可通行，不做射线投射）；
- **frontier = 既属于 free、又紧邻 unknown 的格子**（unknown = 既非 free 也非 occupied）；
- 把互相连通的 frontier 格子聚类，取质心作为候选观察点；
- 在候选中选**距离最近且视线未被 occupied 阻断**的一个作为下一个目标；
- 没有任何候选时，说明探索完成。
"""

import math

import numpy as np

#: 8 邻域
_NEIGHBORS = [(-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)]


class FrontierMap:
    def __init__(self, resolution: float = 0.5, free_radius: float = 3.0):
        self.resolution = float(resolution)
        self.free_radius = float(free_radius)
        self._occupied = set()
        self._free = set()

    # ------------------------------------------------------------------ 坐标

    def key(self, x: float, y: float):
        r = self.resolution
        return (int(math.floor(x / r)), int(math.floor(y / r)))

    def center(self, key) -> tuple:
        r = self.resolution
        return ((key[0] + 0.5) * r, (key[1] + 0.5) * r, 0.0)

    # ------------------------------------------------------------------ 更新

    def mark_occupied(self, points_enu: np.ndarray) -> None:
        for point in points_enu:
            self._occupied.add(self.key(float(point[0]), float(point[1])))

    def mark_free_around(self, position_enu) -> None:
        """把机体当前位置周围一圈标记为“已确认可通过”。"""
        radius_cells = int(math.ceil(self.free_radius / self.resolution))
        cx, cy = self.key(float(position_enu[0]), float(position_enu[1]))
        for di in range(-radius_cells, radius_cells + 1):
            for dj in range(-radius_cells, radius_cells + 1):
                if math.hypot(di, dj) * self.resolution > self.free_radius:
                    continue
                cell = (cx + di, cy + dj)
                if cell not in self._occupied:
                    self._free.add(cell)

    # ------------------------------------------------------------------ 查询

    @property
    def free_count(self) -> int:
        return len(self._free)

    @property
    def occupied_count(self) -> int:
        return len(self._occupied)

    def frontier_cells(self) -> set:
        cells = set()
        for cell in self._free:
            for di, dj in _NEIGHBORS:
                neighbour = (cell[0] + di, cell[1] + dj)
                if neighbour not in self._free and neighbour not in self._occupied:
                    cells.add(cell)
                    break
        return cells

    def cluster_cells(self):
        """把 frontier 格子按连通性聚类，返回 [[格子, ...], ...]。"""
        remaining = set(self.frontier_cells())
        result = []
        while remaining:
            start = remaining.pop()
            stack = [start]
            group = [start]
            while stack:
                cell = stack.pop()
                for di, dj in _NEIGHBORS:
                    neighbour = (cell[0] + di, cell[1] + dj)
                    if neighbour in remaining:
                        remaining.discard(neighbour)
                        stack.append(neighbour)
                        group.append(neighbour)
            result.append(group)
        return result

    def clusters(self):
        """返回 [(质心 ENU, 格子数), ...]，供状态展示与统计使用。

        注意：环形 frontier 的质心可能落在环内（甚至与机体位置重合），
        因此**不能用质心当探索目标**——目标取簇内离机体最近的格子（见 best_frontier）。
        """
        result = []
        for group in self.cluster_cells():
            xs = [self.center(c)[0] for c in group]
            ys = [self.center(c)[1] for c in group]
            result.append(((float(np.mean(xs)), float(np.mean(ys)), 0.0), len(group)))
        return result

    def best_frontier(self, position_enu, max_distance: float = 15.0,
                      visited=(), visited_radius: float = 1.5,
                      size_weight: float = 1.0, distance_weight: float = 0.5):
        """按**信息增益**选择 frontier，而不是只看最近。

        评分 = ``size_weight × 簇规模 / (1 + distance_weight × 距离)``：
        簇规模 ≈ 未知边界长度（越大越可能通向未探索区域），距离做惩罚；
        ``visited`` 中已访问目标周围 ``visited_radius`` 内的候选直接跳过，
        避免反复挑同一处 frontier 卡住。

        目标点取**簇内离机体最近的格子**——质心对环形 frontier 会落在圆心，不可用。
        """
        candidates = []
        for group in self.cluster_cells():
            representative = min(
                group,
                key=lambda cell: (self.center(cell)[0] - position_enu[0]) ** 2
                + (self.center(cell)[1] - position_enu[1]) ** 2)
            point = self.center(representative)
            distance = math.hypot(point[0] - position_enu[0],
                                  point[1] - position_enu[1])
            if distance > max_distance or self._blocked(position_enu, point):
                continue
            if any(math.hypot(point[0] - v[0], point[1] - v[1]) <= visited_radius
                   for v in visited):
                continue
            score = size_weight * len(group) / (1.0 + distance_weight * distance)
            candidates.append((score, point))
        if not candidates:
            return None
        return max(candidates, key=lambda item: item[0])[1]

    def _blocked(self, start_enu, end_enu) -> bool:
        """栅格直线检查：起点到终点的连线上是否有 occupied 格子。"""
        x0, y0 = self.key(float(start_enu[0]), float(start_enu[1]))
        x1, y1 = self.key(float(end_enu[0]), float(end_enu[1]))
        steps = max(abs(x1 - x0), abs(y1 - y0))
        if steps == 0:
            return False
        for step in range(steps + 1):
            t = step / steps
            xi = int(round(x0 + (x1 - x0) * t))
            yi = int(round(y0 + (y1 - y0) * t))
            if (xi, yi) in self._occupied:
                return True
        return False
