"""视觉目标 → 相对 ENU 目标（含 standoff），纯几何，便于单测。"""

import math
from statistics import median


def apply_standoff(east_m: float, north_m: float, standoff_m: float):
    """把目标从物体中心沿视线回退 standoff_m；不读取仿真物体坐标。"""
    east = float(east_m)
    north = float(north_m)
    standoff = max(float(standoff_m), 0.0)
    distance = math.hypot(east, north)
    if distance <= standoff + 1e-6:
        raise ValueError(f'target distance {distance:.2f} m <= standoff {standoff:.2f} m')
    scale = (distance - standoff) / distance
    return east * scale, north * scale, distance


def median_enu_target(samples, max_spread_m: float = 1.5):
    """对多次 (east,north) 取中位数；偏离中位数超限则拒绝。"""
    if not samples:
        raise ValueError('no visual target samples')
    east = median(float(item[0]) for item in samples)
    north = median(float(item[1]) for item in samples)
    spread = max(math.hypot(float(e) - east, float(n) - north)
                 for e, n in samples)
    if spread > float(max_spread_m):
        raise ValueError(f'target spread {spread:.2f} m > {max_spread_m:.2f} m')
    return east, north, spread
