"""2D A* 局部路径规划（纯逻辑，不依赖 ROS）。

输入 occupied 世界点（ENU 水平投影）、起点/终点、分辨率与安全膨胀；
输出从起点到终点的世界坐标路径，失败返回 None。
"""

import heapq
import math

_NEIGHBORS = [(-1, -1, math.sqrt(2.0)), (-1, 0, 1.0), (-1, 1, math.sqrt(2.0)),
              (0, -1, 1.0), (0, 1, 1.0),
              (1, -1, math.sqrt(2.0)), (1, 0, 1.0), (1, 1, math.sqrt(2.0))]


def _key(x, y, resolution):
    return (int(math.floor(float(x) / resolution)),
            int(math.floor(float(y) / resolution)))


def _center(key, resolution):
    return ((key[0] + 0.5) * resolution, (key[1] + 0.5) * resolution)


def _inflate(occupied, cells):
    if cells <= 0:
        return set(occupied)
    result = set()
    for cx, cy in occupied:
        for dx in range(-cells, cells + 1):
            for dy in range(-cells, cells + 1):
                if dx * dx + dy * dy <= cells * cells:
                    result.add((cx + dx, cy + dy))
    return result


def _line_clear(a, b, blocked):
    ax, ay = a
    bx, by = b
    steps = max(abs(bx - ax), abs(by - ay))
    for step in range(1, steps):
        t = step / steps
        cell = (int(round(ax + (bx - ax) * t)),
                int(round(ay + (by - ay) * t)))
        if cell in blocked:
            return False
    return True


def plan_path(occupied, start_enu, goal_enu, *, resolution: float = 0.25,
              inflate_m: float = 0.4, max_expansions: int = 200000,
              max_radius_m: float = 12.0, smooth: bool = True):
    """在 2D 占据栅格上做 A*；返回 [(east,north), ...] 或 None。"""
    resolution = max(float(resolution), 0.05)
    cells = int(math.ceil(max(float(inflate_m), 0.0) / resolution))
    blocked = _inflate({_key(x, y, resolution) for x, y in occupied}, cells)
    start = _key(start_enu[0], start_enu[1], resolution)
    goal = _key(goal_enu[0], goal_enu[1], resolution)
    radius_cells = int(math.ceil(max(float(max_radius_m), 0.0) / resolution))
    min_x = min(start[0], goal[0]) - radius_cells
    max_x = max(start[0], goal[0]) + radius_cells
    min_y = min(start[1], goal[1]) - radius_cells
    max_y = max(start[1], goal[1]) + radius_cells
    if start in blocked or goal in blocked:
        return None
    open_heap = [(0.0, 0.0, start)]
    came_from = {}
    cost = {start: 0.0}
    expansions = 0
    while open_heap:
        _, g, current = heapq.heappop(open_heap)
        if current == goal:
            path = [current]
            while path[-1] in came_from:
                path.append(came_from[path[-1]])
            path.reverse()
            points = [_center(cell, resolution) for cell in path]
            if smooth:
                points = _smooth(points, blocked, resolution)
            return points
        if g > cost.get(current, float('inf')):
            continue
        expansions += 1
        if expansions > max_expansions:
            return None
        for dx, dy, step in _NEIGHBORS:
            neighbour = (current[0] + dx, current[1] + dy)
            if not (min_x <= neighbour[0] <= max_x and min_y <= neighbour[1] <= max_y):
                continue
            if neighbour in blocked:
                continue
            new_cost = g + step * resolution
            if new_cost >= cost.get(neighbour, float('inf')):
                continue
            cost[neighbour] = new_cost
            h = math.hypot(goal[0] - neighbour[0], goal[1] - neighbour[1]) * resolution
            came_from[neighbour] = current
            heapq.heappush(open_heap, (new_cost + h, new_cost, neighbour))
    return None


def _smooth(points, blocked, resolution):
    if len(points) <= 2:
        return points
    result = [points[0]]
    i = 0
    while i < len(points) - 1:
        j = len(points) - 1
        while j > i + 1:
            a = _key(points[i][0], points[i][1], resolution)
            b = _key(points[j][0], points[j][1], resolution)
            if _line_clear(a, b, blocked):
                break
            j -= 1
        result.append(points[j])
        i = j
    return result
