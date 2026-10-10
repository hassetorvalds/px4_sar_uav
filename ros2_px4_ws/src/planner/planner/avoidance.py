"""避障决策的纯逻辑（不依赖 ROS，便于单元测试）。

策略（v1，刻意保持简单与可预测）：

- 目标在到达半径内 → 停止（`arrived`）并保持高度；
- 前方锥形区域净空 ≤ stop_distance → **停止前进并侧移**（`avoid`），
  侧移方向默认向右（顺时针），使机体沿障碍物滑动；
- stop_distance < 净空 ≤ slow_distance → 按比例减速（`slow`）；
- 净空充足 → 以巡航速度朝目标飞（`cruise`）。

局限（已知）：侧移方向固定，不做左右净空比较；绕障后可能再次朝障碍物前进，
产生"贴边滑动"而非最优绕行。后续可接入更完整的局部规划器。
"""

import math
from dataclasses import dataclass


@dataclass
class AvoidanceCommand:
    velocity_enu: tuple      # (vx, vy, vz) m/s
    mode: str                # cruise | slow | avoid | arrived
    distance_to_goal: float
    reason: str = ''
    avoid_side_used: int = 0      # +1 左、-1 右、0 未避让


def plan_velocity(goal_enu, position_enu, *, clearance_m: float,
                  cruise_speed: float = 1.0, slow_distance: float = 2.0,
                  stop_distance: float = 0.8, goal_radius: float = 0.5,
                  avoid_side: int = -1, altitude_gain: float = 0.5,
                  left_clearance: float = float('inf'),
                  right_clearance: float = float('inf'),
                  previous_avoid_side: int = 0,
                  side_switch_margin: float = 0.5) -> AvoidanceCommand:
    """根据目标方向与前方净空给出速度指令。

    ``avoid_side``：+1 向左、-1 向右；仅在**没有左右净空信息**时作为兜底。
    ``left_clearance`` / ``right_clearance``：左右净空，提供时选择更宽的一侧绕障；
    两侧差距小于 ``side_switch_margin`` 时沿用 ``previous_avoid_side``（迟滞，避免左右摇摆）。
    ``altitude_gain``：高度误差到垂直速度的比例系数。
    """
    dx = goal_enu[0] - position_enu[0]
    dy = goal_enu[1] - position_enu[1]
    dz = goal_enu[2] - position_enu[2]
    distance = math.hypot(dx, dy)
    vertical = max(-1.0, min(altitude_gain * dz, 1.0))

    if distance <= goal_radius:
        return AvoidanceCommand((0.0, 0.0, vertical), 'arrived', distance,
                                '已到达目标')

    unit_x, unit_y = dx / distance, dy / distance
    if clearance_m <= stop_distance:
        # 停止前进，向更宽的一侧侧移（垂直目标方向）
        side_choice = _choose_avoid_side(
            left_clearance, right_clearance, previous_avoid_side,
            side_switch_margin, default=avoid_side)
        side = 1.0 if side_choice >= 0 else -1.0
        vel = (-unit_y * side * cruise_speed, unit_x * side * cruise_speed, vertical)
        return AvoidanceCommand(vel, 'avoid', distance,
                                f'前方净空 {clearance_m:.2f} m ≤ {stop_distance:.2f} m，'
                                f'向{"左" if side_choice > 0 else "右"}侧移'
                                f'（左净空 {left_clearance:.2f} m / 右净空 {right_clearance:.2f} m）',
                                avoid_side_used=side_choice)
    if clearance_m <= slow_distance:
        scale = (clearance_m - stop_distance) / max(slow_distance - stop_distance, 1e-6)
        speed = cruise_speed * max(0.2, min(scale, 1.0))
        return AvoidanceCommand((unit_x * speed, unit_y * speed, vertical), 'slow',
                                distance, f'前方净空 {clearance_m:.2f} m，减速至 {speed:.2f} m/s')
    return AvoidanceCommand((unit_x * cruise_speed, unit_y * cruise_speed, vertical),
                            'cruise', distance)


def _choose_avoid_side(left_clearance: float, right_clearance: float,
                       previous_side: int, margin: float, default: int) -> int:
    """选择绕障方向：更宽的一侧优先；差距不大时沿用上次方向（迟滞）。"""
    left_known = left_clearance is not None and not math.isinf(left_clearance)
    right_known = right_clearance is not None and not math.isinf(right_clearance)
    if not left_known and not right_known:
        return 1 if default >= 0 else -1
    if not right_known:
        return 1
    if not left_known:
        return -1
    if abs(left_clearance - right_clearance) < margin and previous_side != 0:
        return previous_side
    return 1 if left_clearance > right_clearance else -1
