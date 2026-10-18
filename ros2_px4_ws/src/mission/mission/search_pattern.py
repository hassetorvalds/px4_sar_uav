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




def lawnmower_polygon(polygon, spacing_m: float, angle_deg: float = 0.0,
                      *, include_edges: bool = True):
    """任意多边形（含凹形）的弓字搜索。

    思路：把多边形旋转到"航线沿 x"的局部系，按 y 扫描线求与多边形边的交点，
    排序后两两配对成段；再旋转回 ENU。凹多边形一条扫描线可能有多段。
    """
    points = [(float(e), float(n)) for e, n in polygon]
    if len(points) < 3:
        raise ValueError('polygon 至少 3 个顶点')
    spacing = float(spacing_m)
    if spacing <= 0:
        raise ValueError('spacing_m 必须 > 0')
    theta = math.radians(float(angle_deg))
    cos_t, sin_t = math.cos(theta), math.sin(theta)

    # 世界 → 局部（旋转 -theta）
    local = [(e * cos_t + n * sin_t, -e * sin_t + n * cos_t)
             for e, n in points]
    ys = [p[1] for p in local]
    y_min, y_max = min(ys), max(ys)

    lanes = []
    y = y_min
    while y < y_max - 1e-9:
        lanes.append(y)
        y += spacing
    if include_edges:
        lanes.append(y_max)

    def intersections(lane_y):
        xs = []
        count = len(local)
        for index in range(count):
            x1, y1 = local[index]
            x2, y2 = local[(index + 1) % count]
            if abs(y2 - y1) < 1e-12:
                continue
            low, high = (y1, y2) if y1 < y2 else (y2, y1)
            if low <= lane_y < high or (include_edges and lane_y == high):
                x = x1 + (lane_y - y1) * (x2 - x1) / (y2 - y1)
                xs.append(x)
        xs.sort()
        unique = []
        for x in xs:
            if not unique or abs(x - unique[-1]) > 1e-6:
                unique.append(x)
        return unique

    waypoints = []
    for lane_index, lane_y in enumerate(lanes):
        xs = intersections(lane_y)
        for start in range(0, len(xs) - 1, 2):
            x0, x1 = xs[start], xs[start + 1]
            if lane_index % 2 == 1:
                x0, x1 = x1, x0
            for x in (x0, x1):
                east = x * cos_t - lane_y * sin_t
                north = x * sin_t + lane_y * cos_t
                waypoints.append((east, north))
    return waypoints


def _polygon_area(points) -> float:
    if len(points) < 3:
        return 0.0
    total = 0.0
    for index, (east, north) in enumerate(points):
        next_east, next_north = points[(index + 1) % len(points)]
        total += east * next_north - next_east * north
    return abs(total) / 2.0


def search_metrics(step_params, waypoints, speed_mps: float,
                   altitude_m=None, hfov_deg: float = 90.0):
    """覆盖指标：面积/折线/预计时间；有高度时给下视足印与重叠率。"""
    shape = str(step_params.get('shape', 'square')).lower()
    if shape == 'polygon':
        vertices = (step_params.get('polygon')
                    or step_params.get('vertices') or [])
        area = _polygon_area([(float(e), float(n)) for e, n in vertices])
    elif shape == 'rectangle':
        area = float(step_params.get('width_m', 0.0)) * float(
            step_params.get('height_m', 0.0))
    else:
        radius = float(step_params.get('radius_m', 0.0))
        area = (2.0 * radius) ** 2
    metrics = {
        'area_m2': round(area, 1),
        'path_length_m': round(path_length(waypoints), 1),
        'estimated_s': round(estimate_duration_s(waypoints, speed_mps), 1),
        'spacing_m': float(step_params.get('spacing_m', 0.0)),
    }
    if altitude_m is not None:
        footprint = 2.0 * float(altitude_m) * math.tan(
            math.radians(float(hfov_deg)) / 2.0)
        spacing = float(step_params.get('spacing_m', 0.0))
        metrics['footprint_width_m'] = round(footprint, 2)
        metrics['lane_overlap_ratio'] = (
            round(max(0.0, 1.0 - spacing / footprint), 3)
            if footprint > 0 else 0.0)
    return metrics


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
