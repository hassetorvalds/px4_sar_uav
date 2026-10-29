"""A* Path 跟随纯逻辑：位置 + 路径 → 下一 waypoint / 速度指令。"""

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class FollowCommand:
    east: float
    north: float
    done: bool
    index: int


def next_waypoint(position, path, *, tolerance_m: float = 0.4,
                  lookahead_m: float = 0.5):
    """跳过已到达点，返回 (waypoint, index)；路径空返回 (None, -1)。"""
    if not path:
        return None, -1
    px, py = float(position[0]), float(position[1])
    for index, point in enumerate(path):
        distance = math.hypot(float(point[0]) - px, float(point[1]) - py)
        if distance <= float(tolerance_m):
            continue
        if distance < float(lookahead_m) and index + 1 < len(path):
            return (float(path[index + 1][0]), float(path[index + 1][1])), index + 1
        return (float(point[0]), float(point[1])), index
    return None, len(path)


def follow_velocity(position, path, *, max_speed_mps: float = 0.5,
                    gain: float = 0.8, tolerance_m: float = 0.4,
                    lookahead_m: float = 0.5):
    """朝路径下一 waypoint 的 P 速度；路径走完 done=True。"""
    waypoint, index = next_waypoint(
        position, path, tolerance_m=tolerance_m, lookahead_m=lookahead_m)
    if waypoint is None:
        return FollowCommand(0.0, 0.0, True, index)
    dx = waypoint[0] - float(position[0])
    dy = waypoint[1] - float(position[1])
    distance = math.hypot(dx, dy)
    if distance <= float(tolerance_m):
        return FollowCommand(0.0, 0.0, True, index)
    speed = min(float(max_speed_mps), float(gain) * distance)
    return FollowCommand(dx / distance * speed, dy / distance * speed, False, index)
