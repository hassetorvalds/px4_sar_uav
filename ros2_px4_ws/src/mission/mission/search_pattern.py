"""搜索航迹生成（纯函数）：把"区域"变成一串可飞的航点。

v1 用**弓字形（lawnmower）**：在区域外接正方形内按固定间距来回扫。
这是覆盖搜索里最稳、最容易验证的一种；后续可替换成螺旋/前沿探索，
只要仍输出 ``[(east_m, north_m), ...]``，执行层不用改。
"""

import math


def lawnmower(center_east: float, center_north: float, radius_m: float,
              spacing_m: float, *, include_edges: bool = True):
    """返回覆盖外接正方形的弓字形航点（相对起飞点的 ENU 米）。

    - 航向沿东/西，换行沿北；
    - 相邻航线的间距 ≤ ``spacing_m``（最后一行为覆盖北边界可能更近）；
    - ``include_edges=True`` 时首尾行落在南/北边界上。
    """
    radius = float(radius_m)
    spacing = float(spacing_m)
    if radius <= 0:
        raise ValueError('radius_m 必须 > 0')
    if spacing <= 0:
        raise ValueError('spacing_m 必须 > 0')

    south = center_north - radius
    north = center_north + radius
    west = center_east - radius
    east = center_east + radius

    lanes = []
    y = south
    index = 0
    while y < north - 1e-9:
        lanes.append(y)
        y += spacing
        index += 1
    if include_edges:
        lanes.append(north)
    else:
        lanes.append(min(north, south + max(spacing, north - south)))

    waypoints = []
    for lane_index, lane_north in enumerate(lanes):
        if lane_index % 2 == 0:
            waypoints.append((west, lane_north))
            waypoints.append((east, lane_north))
        else:
            waypoints.append((east, lane_north))
            waypoints.append((west, lane_north))
    return waypoints


def path_length(waypoints):
    """折线总长度（米），用于估算搜索时长。"""
    total = 0.0
    for (x1, y1), (x2, y2) in zip(waypoints, waypoints[1:]):
        total += math.hypot(x2 - x1, y2 - y1)
    return total


def estimate_duration_s(waypoints, speed_mps: float, *, turn_overhead_s: float = 2.0):
    """按速度粗略估计飞行时长（每段加转向开销）。"""
    if speed_mps <= 0:
        raise ValueError('speed_mps 必须 > 0')
    return path_length(waypoints) / speed_mps + turn_overhead_s * max(len(waypoints) - 1, 0)
