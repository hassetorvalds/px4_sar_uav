"""地图安全层（Step C3）：把本机局部地图的三向净空插进 VLM 命令通道。

设计要点（2026-10-12 与服务器侧确认后定稿）：

1. **fail-safe 而不是 fail-open**：地图数据陈旧（>``stale_s``）时视为"前向净空未知"，
   把前向速度压到 ``stale_forward_limit_mps``，而不是"没有数据就原样飞"；
2. **左右符号与 LightNav 一致**：`local_map` 的 ``left_m`` 是机体左方（航向 +90° 锥），
   与路点 ``lateral_m``（+ = 左）同一套；因此"左更宽 → 左转（yaw_rate > 0）"，
   这里用单元测试把方向钉死；
3. **迟滞**：进入保持用 ``stop_m``、退出门槛用更高的 ``release_m``，避免在阈值附近抖振；
4. **障碍保持时偏航覆盖 VLM**（限幅），前向与横向归零；
5. 压速作用在**最终下发速度**上（不是路点层）：从 ``slow_start_m`` 到 ``stop_m`` 线性压到 0。

模块只依赖 ``math``/``dataclasses``/``re``，便于离线单元测试。
"""

import math
import re
from dataclasses import dataclass

from .pointing import BodyVelocity

_CLEARANCE_RE = re.compile(
    r'nearest_forward_m=(-?[\d.]+|inf|nan)\s+left_m=(-?[\d.]+|inf|nan)'
    r'\s+right_m=(-?[\d.]+|inf|nan)')


@dataclass
class MapSafetyLimits:
    """安全层阈值（全部可在 ROS 参数里覆盖）。"""

    enabled: bool = True
    stale_s: float = 0.5                 # 净空数据超过这个时间视为"未知"
    stale_forward_limit_mps: float = 0.1  # 未知时前向速度上限（fail-safe）
    slow_start_m: float = 1.2            # 从该净空开始压速
    stop_m: float = 0.6                  # 低于该净空进入"保持"
    release_m: float = 0.9               # 高于该净空才解除"保持"（迟滞）
    avoid_yaw_rate: float = 0.3          # 保持时朝较宽一侧的偏航角速度（覆盖 VLM）


def parse_clearance_status(text: str):
    """解析 ``/local_map/status``：``... nearest_forward_m=0.42 left_m=3.00 right_m=2.10``。

    返回 ``(forward, left, right)``；解析不出来返回 ``None``（上层按"未知"处理）。
    """
    if not text:
        return None
    match = _CLEARANCE_RE.search(text.replace(',', ' '))
    if not match:
        return None
    values = []
    for group in match.groups():
        try:
            values.append(float(group))
        except ValueError:
            return None
    return tuple(values)


def apply_map_safety(command: BodyVelocity, clearance, age_s: float, holding: bool,
                     limits: MapSafetyLimits):
    """对命令施加地图安全层。

    返回 ``(新命令, guard, 新 holding, 细节dict)``：

    - ``guard`` ∈ ``map_off / ok / stale / obstacle_slow / obstacle``；
    - ``holding`` 需要在上层保存（迟滞状态），下次调用传回来。
    """
    if not limits.enabled:
        return command, 'map_off', holding, {}

    def _with(forward=None, left=None, up=None, yaw_rate=None):
        return BodyVelocity(
            command.forward if forward is None else forward,
            command.left if left is None else left,
            command.up if up is None else up,
            command.yaw_rate if yaw_rate is None else yaw_rate,
            command.duration_s)

    if clearance is None or (age_s is not None and age_s > limits.stale_s):
        limited = math.copysign(
            min(abs(command.forward), limits.stale_forward_limit_mps), command.forward)
        return (_with(forward=limited), 'stale', holding,
                {'reason': 'clearance_unknown', 'age_s': None if age_s is None else round(age_s, 2),
                 'forward_limit_mps': limits.stale_forward_limit_mps})

    forward_c, left_c, right_c = clearance

    # 迟滞：已在保持状态时用更高的释放门槛
    holding = (forward_c < limits.release_m) if holding else (forward_c < limits.stop_m)
    if holding:
        # 障碍保持：偏航**覆盖** VLM（朝更宽一侧），前向/横向归零
        yaw = limits.avoid_yaw_rate if left_c >= right_c else -limits.avoid_yaw_rate
        side = 'left' if left_c >= right_c else 'right'
        return (_with(forward=0.0, left=0.0, yaw_rate=yaw), 'obstacle', True,
                {'reason': 'obstacle_hold', 'forward_m': round(forward_c, 2),
                 'left_m': round(left_c, 2), 'right_m': round(right_c, 2),
                 'avoid_side': side, 'yaw_rate': round(yaw, 2)})

    if forward_c < limits.slow_start_m:
        span = max(limits.slow_start_m - limits.stop_m, 1e-6)
        scale = min(max((forward_c - limits.stop_m) / span, 0.0), 1.0)
        return (_with(forward=command.forward * scale), 'obstacle_slow', False,
                {'reason': 'obstacle_slow', 'forward_m': round(forward_c, 2),
                 'scale': round(scale, 3)})

    return command, 'ok', False, {'forward_m': round(forward_c, 2)}
