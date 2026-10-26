"""视觉目标 → 相对 ENU 目标（含 standoff），纯几何，便于单测。"""

import math


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
