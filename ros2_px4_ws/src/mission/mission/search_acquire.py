"""搜索→目标接近的纯几何逻辑。"""

import math


def acquire_goal(position_enu, target_enu, *, standoff_m=2.0, shrink=True):
    """目标中心 → 保持 standoff 的接近点；不读取 AirSim 物体坐标。"""
    dx = float(target_enu[0]) - float(position_enu[0])
    dy = float(target_enu[1]) - float(position_enu[1])
    distance = math.hypot(dx, dy)
    if distance <= 1e-6:
        raise ValueError('target coincides with position')
    standoff = (min(float(standoff_m), max(0.3, 0.5 * distance))
                if shrink else float(standoff_m))
    return (float(target_enu[0]) - dx / distance * standoff,
            float(target_enu[1]) - dy / distance * standoff,
            standoff)
