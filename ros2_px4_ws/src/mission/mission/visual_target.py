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


def select_verified_target(primary, verify, max_diff_m: float = 3.0):
    """主/第二意见一致性门。

    - 两者都有：分歧 ≤ max_diff 才接受 primary；否则拒绝；
    - 仅一个成功：采用成功的那个；
    - 都失败：拒绝。
    返回 (east, north, diff, source)。
    """
    primary_ok = primary is not None
    verify_ok = verify is not None
    if primary_ok and verify_ok:
        diff = math.hypot(float(primary[0]) - float(verify[0]),
                          float(primary[1]) - float(verify[1]))
        if diff > float(max_diff_m):
            raise ValueError(f'primary/verify disagreement {diff:.2f} m > {max_diff_m:.2f} m')
        return float(primary[0]), float(primary[1]), diff, 'primary'
    if primary_ok:
        return float(primary[0]), float(primary[1]), None, 'primary'
    if verify_ok:
        return float(verify[0]), float(verify[1]), None, 'verify'
    raise ValueError('both primary and verify failed')


def bbox_touches_edge(bbox, width: int, height: int, margin_px: float = 5.0) -> bool:
    """bbox 是否贴图像边缘；贴边目标不生成 goto。"""
    x1, y1, x2, y2 = (float(v) for v in bbox)
    m = max(float(margin_px), 0.0)
    return (x1 <= m or y1 <= m
            or x2 >= float(width) - m or y2 >= float(height) - m)
