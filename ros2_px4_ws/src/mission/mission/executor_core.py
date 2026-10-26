"""任务执行的**纯逻辑核心**（不依赖 ROS，便于单测）。

ROS 节点只负责"读状态 → 调这里 → 发指令"，所有会出错的算术/判据都放在这里：

- 朝目标点的限速速度；
- 围栏判定；
- 前向/左侧/右侧净空门（对应方向速度分量归零并标记 blocked）；
- 搜索区域展开成弓字形航点；
- ``forward``（机体前向）在世界系里的目标点。
"""

import math
from dataclasses import dataclass

from .search_pattern import lawnmower, lawnmower_polygon, lawnmower_rectangle


@dataclass(frozen=True)
class VelocityDecision:
    east: float
    north: float
    done: bool
    blocked: bool = False
    reason: str = ''


def velocity_toward(current_enu, target_enu, *, max_speed_mps: float,
                    gain: float = 0.6, tolerance_m: float = 0.4) -> VelocityDecision:
    """朝目标点的 P 控制速度（ENU，水平），带限速与到点判定。"""
    dx = float(target_enu[0]) - float(current_enu[0])
    dy = float(target_enu[1]) - float(current_enu[1])
    distance = math.hypot(dx, dy)
    if distance <= tolerance_m:
        return VelocityDecision(0.0, 0.0, done=True)
    speed = min(float(max_speed_mps), gain * distance)
    return VelocityDecision(dx / distance * speed, dy / distance * speed, done=False)


def apply_clearance_gate(decision: VelocityDecision, current_enu, heading_enu: float,
                         nearest_forward_m, *, stop_m: float = 0.6,
                         nearest_left_m=None, nearest_right_m=None,
                         left_known: bool = True, right_known: bool = True):
    """按机体系前/左/右净空限制速度分量；未知净空不拦。"""
    if decision.done:
        return decision
    cos_h = math.cos(heading_enu)
    sin_h = math.sin(heading_enu)
    forward = decision.east * cos_h + decision.north * sin_h
    left = -decision.east * sin_h + decision.north * cos_h
    blocked = False
    reasons = []
    if nearest_forward_m is not None and float(nearest_forward_m) <= float(stop_m) and forward > 1e-6:
        forward = 0.0
        blocked = True
        reasons.append(f'前向净空 {float(nearest_forward_m):.2f} m ≤ {stop_m:.2f} m')
    if not left_known and left > 1e-6:
        left = 0.0
        blocked = True
        reasons.append('左侧净空未知')
    elif nearest_left_m is not None and float(nearest_left_m) <= float(stop_m) and left > 1e-6:
        left = 0.0
        blocked = True
        reasons.append(f'左侧净空 {float(nearest_left_m):.2f} m ≤ {stop_m:.2f} m')
    if not right_known and left < -1e-6:
        left = 0.0
        blocked = True
        reasons.append('右侧净空未知')
    elif nearest_right_m is not None and float(nearest_right_m) <= float(stop_m) and left < -1e-6:
        left = 0.0
        blocked = True
        reasons.append(f'右侧净空 {float(nearest_right_m):.2f} m ≤ {stop_m:.2f} m')
    east = forward * cos_h - left * sin_h
    north = forward * sin_h + left * cos_h
    return VelocityDecision(east, north, done=False, blocked=blocked,
                            reason='; '.join(reasons))


def inside_geofence(current_enu, origin_enu, radius_m: float, *, margin_m: float = 0.0):
    distance = math.hypot(float(current_enu[0]) - float(origin_enu[0]),
                          float(current_enu[1]) - float(origin_enu[1]))
    return distance <= float(radius_m) + float(margin_m)


def yaw_rate_to_target(current_enu: float, target_enu: float, *,
                       max_rate_rad_s: float, tolerance_rad: float = 0.05,
                       gain: float = 1.5):
    """朝目标航向的角速度（ENU，正=左/逆时针）；返回 (rate, done)。"""
    error = math.remainder(float(target_enu) - float(current_enu), 2.0 * math.pi)
    if abs(error) <= tolerance_rad:
        return 0.0, True
    rate = max(-float(max_rate_rad_s), min(float(max_rate_rad_s), gain * error))
    return rate, False


def altitude_hold_vz(target_up, current_up, *, gain: float = 0.6,
                     max_rate_mps: float = 0.3) -> float:
    """速度模式的高度保持：vz = clamp(gain * (target - current), ±max_rate)。

    2026-10-18 实测：vz 恒 0 时高度会缓慢下沉；低到地面回波进入前向锥后
    会触发净空门。显式 P 控制把高度拉回任务值。
    """
    if target_up is None or current_up is None:
        return 0.0
    return max(-float(max_rate_mps),
               min(float(max_rate_mps),
                   float(gain) * (float(target_up) - float(current_up))))


def forward_target(current_enu, heading_enu: float, meters: float):
    """机体前向 ``meters`` 米对应的世界 ENU 目标点（高度保持不变）。"""
    return (float(current_enu[0]) + float(meters) * math.cos(heading_enu),
            float(current_enu[1]) + float(meters) * math.sin(heading_enu),
            float(current_enu[2]))


def offset_waypoints(waypoints, origin):
    """把相对起飞点的 (e,n,up) 平移到世界 ENU。"""
    return [(float(e) + float(origin[0]), float(n) + float(origin[1]), float(up))
            for e, n, up in waypoints]


def search_waypoints(step_params, *, altitude_m: float):
    """把 ``search`` step 展开成世界系航点 ``[(east, north, up), ...]``。"""
    center_east = float(step_params.get('center_east_m', 0.0))
    center_north = float(step_params.get('center_north_m', 0.0))
    radius = float(step_params.get('radius_m', 10.0))
    spacing = float(step_params.get('spacing_m', 4.0))
    up = float(step_params.get('altitude_m', altitude_m))
    shape = str(step_params.get('shape', 'square')).lower()
    if shape == 'polygon':
        vertices = (step_params.get('polygon')
                    or step_params.get('vertices') or [])
        absolute = [(center_east + float(e), center_north + float(n))
                    for e, n in vertices]
        flat = lawnmower_polygon(absolute, spacing,
                                 float(step_params.get('angle_deg', 0.0)))
    elif shape == 'rectangle':
        flat = lawnmower_rectangle(
            center_east, center_north,
            float(step_params.get('width_m', 10.0)),
            float(step_params.get('height_m', 10.0)),
            spacing, float(step_params.get('angle_deg', 0.0)))
    else:
        flat = lawnmower(center_east, center_north, radius, spacing)
    return [(east, north, up) for east, north in flat]


def step_timeout_s(spec_step, *, speed_mps: float, min_s: float = 15.0,
                   extra_s: float = 10.0):
    """给单个 step 一个宽松但有界的超时（防止卡死）。"""
    kind = spec_step.type
    if kind == 'takeoff':
        return max(min_s, 20.0)
    if kind == 'hover':
        return float(spec_step.params.get('seconds', 5.0)) + extra_s
    if kind == 'goto':
        distance = math.hypot(float(spec_step.params.get('east_m', 0.0)),
                              float(spec_step.params.get('north_m', 0.0)))
        return max(min_s, distance / max(speed_mps, 0.1) * 2.0 + extra_s)
    if kind == 'forward':
        meters = float(spec_step.params.get('meters', 0.0))
        return max(min_s, meters / max(speed_mps, 0.1) * 2.0 + extra_s)
    if kind == 'yaw':
        degrees = abs(float(spec_step.params.get('degrees', 90.0)))
        rate = max(float(spec_step.params.get('rate_deg_s', 20.0)), 1.0)
        return max(min_s, degrees / rate * 2.0 + extra_s)
    if kind == 'scan':
        return float(spec_step.params.get('seconds', 20.0)) + extra_s
    if kind == 'search':
        # 搜索整体由任务级 max_duration 兜底，这里给单点足够时间
        return max(min_s, 60.0)
    return min_s
