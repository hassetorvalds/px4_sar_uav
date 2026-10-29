"""A* 候选速度的安全复核（纯逻辑，不依赖 ROS）。

与执行器的净空门保持同一物理含义：按机头前/左/右投影速度，
对应方向净空不足或未知时清零该分量；前向未知仍放行（保持旧行为）。
"""

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class SafeVelocity:
    east: float
    north: float
    blocked: bool = False
    reason: str = ''


def gate_candidate(east, north, *, heading_rad: float,
                   forward_m=None, left_m=None, right_m=None,
                   stop_m: float = 0.6,
                   left_known: bool = True, right_known: bool = True):
    e = float(east)
    n = float(north)
    c = math.cos(heading_rad)
    s = math.sin(heading_rad)
    forward = e * c + n * s
    left = -e * s + n * c
    blocked = False
    reasons = []
    if forward_m is not None and float(forward_m) <= float(stop_m) and forward > 1e-6:
        forward = 0.0
        blocked = True
        reasons.append('forward')
    if not left_known and left > 1e-6:
        left = 0.0
        blocked = True
        reasons.append('left_unknown')
    elif left_m is not None and float(left_m) <= float(stop_m) and left > 1e-6:
        left = 0.0
        blocked = True
        reasons.append('left')
    if not right_known and left < -1e-6:
        left = 0.0
        blocked = True
        reasons.append('right_unknown')
    elif right_m is not None and float(right_m) <= float(stop_m) and left < -1e-6:
        left = 0.0
        blocked = True
        reasons.append('right')
    return SafeVelocity(forward * c - left * s, forward * s + left * c,
                        blocked, ','.join(reasons))
