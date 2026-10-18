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


def lawnmower_rectangle(center_east: float, center_north: float, width_m: float,
                       height_m: float, spacing_m: float,
                       angle_deg: float = 0.0, *, include_edges: bool = True):
    """矩形区域弓字搜索（可绕中心旋转）。

    - 本地坐标：x 沿宽（东）、y 沿长（北），中心在原点；
    - 航线沿 x，换行沿 y，间距 ≤ spacing；
    - 绕中心逆时针旋转 angle_deg，再平移到 center。
    """
    width = float(width_m)
    height = float(height_m)
    spacing = float(spacing_m)
    if width <= 0 or height <= 0:
        raise ValueError('width_m / height_m 必须 > 0')
    if spacing <= 0:
        raise ValueError('spacing_m 必须 > 0')

    half_w = width / 2.0
    half_h = height / 2.0
    lanes = []
    y = -half_h
    while y < half_h - 1e-9:
        lanes.append(y)
        y += spacing
    if include_edges:
        lanes.append(half_h)
    else:
        lanes.append(min(half_h, -half_h + max(spacing, height)))

    local = []
    for lane_index, lane_y in enumerate(lanes):
        if lane_index % 2 == 0:
            local.append((-half_w, lane_y))
            local.append((half_w, lane_y))
        else:
            local.append((half_w, lane_y))
            local.append((-half_w, lane_y))

    theta = math.radians(float(angle_deg))
    cos_t, sin_t = math.cos(theta), math.sin(theta)
    waypoints = []
    for x, y in local:
        east = center_east + x * cos_t - y * sin_t
        north = center_north + x * sin_t + y * cos_t
        waypoints.append((east, north))
    return waypoints


def search_metrics(step_params, waypoints, speed_mps: float):
    """覆盖指标：面积/折线长度/预计时间（与航点数量）。"""
    shape = str(step_params.get('shape', 'square')).lower()
    if shape == 'rectangle':
        area = float(step_params.get('width_m', 0.0)) * float(
            step_params.get('height_m', 0.0))
    else:
        radius = float(step_params.get('radius_m', 0.0))
        area = (2.0 * radius) ** 2
    return {
        'area_m2': round(area, 1),
        'path_length_m': round(path_length(waypoints), 1),
        'estimated_s': round(estimate_duration_s(waypoints, speed_mps), 1),
    }


def path_length(waypoints):
    """折线总长度（米），用于估算搜索时长。"""
    total = 0.0
    for first, second in zip(waypoints, waypoints[1:]):
        total += math.hypot(second[0] - first[0], second[1] - first[1])
    return total


def estimate_duration_s(waypoints, speed_mps: float, *, turn_overhead_s: float = 2.0):
    """按速度粗略估计飞行时长（每段加转向开销）。"""
    if speed_mps <= 0:
        raise ValueError('speed_mps 必须 > 0')
    return path_length(waypoints) / speed_mps + turn_overhead_s * max(len(waypoints) - 1, 0)
