"""“指向即飞行”的解析与几何核心。

思路借鉴自 CoRL 2025 的 See, Point, Fly（本地参考实现：``~/SeePointFly``）：
让 VLM 在图像上**指一个点**并给出粗略深度，再把该点反投影成机体坐标系下的运动指令，
从而避免让模型直接输出控制量。

与本项目落地方式的两点有意差异：

1. 竖直视场按图像宽高比推导（原实现水平/竖直都用同一个 FOV）；
2. 原实现先转向、再前进（两段式，依赖 AirSim 阻塞式 API），
   这里同时给出前进速度与偏航角速度，因为 PX4 速度控制可同时接受速度与 yawspeed。

模块只依赖 ``math`` 与 ``json``，便于离线单元测试。
"""

import json
import math
from dataclasses import dataclass

#: VLM 深度评分（1 最近 ~ 10 最远）映射到的实际距离区间，单位米
DEPTH_NEAR_M = 0.2
DEPTH_FAR_M = 2.0

#: 归一化坐标上限（See, Point, Fly 使用 0-1000）
NORMALIZED_MAX = 1000.0


class PointingParseError(ValueError):
    """VLM 返回内容无法解析为指向结果。"""


@dataclass
class Pointing:
    """VLM 给出的指向结果。"""

    x_norm: float          # 图像横向，0=最左，1=最右
    y_norm: float          # 图像纵向，0=最上（天空），1=最下
    depth_m: float         # 由深度评分换算出的距离，单位米
    depth_raw: float = 0.0  # 原始深度评分（1-10）
    label: str = ''


@dataclass
class BodyVelocity:
    """机体 FLU 速度指令（前/左/上，m/s）与偏航角速度（rad/s，逆时针为正）。"""

    forward: float
    left: float
    up: float
    yaw_rate: float
    duration_s: float


def _extract_json_payload(text: str):
    """从模型输出里取出第一段可解析的 JSON（容忍 ``` 包裹与前后解释文字）。"""
    if text is None:
        raise PointingParseError('VLM 返回为空')

    cleaned = text.strip()
    if '```' in cleaned:
        parts = cleaned.split('```')
        for part in parts:
            candidate = part.strip()
            if candidate.startswith('json'):
                candidate = candidate[4:].strip()
            if candidate.startswith('[') or candidate.startswith('{'):
                cleaned = candidate
                break

    # 括号配平扫描，取第一段完整 JSON
    start = min(
        (idx for idx in (cleaned.find('['), cleaned.find('{')) if idx >= 0),
        default=-1,
    )
    if start < 0:
        raise PointingParseError(f'未找到 JSON 内容：{text[:120]!r}')

    opening = cleaned[start]
    closing = ']' if opening == '[' else '}'
    depth = 0
    for idx in range(start, len(cleaned)):
        char = cleaned[idx]
        if char == opening:
            depth += 1
        elif char == closing:
            depth -= 1
            if depth == 0:
                try:
                    return json.loads(cleaned[start:idx + 1])
                except json.JSONDecodeError as exc:
                    raise PointingParseError(f'JSON 解析失败：{exc}') from exc
    raise PointingParseError('JSON 括号不配平')


def _to_unit(value: float) -> float:
    """把归一化坐标统一到 0-1（兼容 0-1000 与 0-1 两种约定）。"""
    number = float(value)
    if abs(number) > 1.5:
        number = number / NORMALIZED_MAX
    return min(max(number, 0.0), 1.0)


def depth_score_to_meters(score: float, depth_far_m: float = DEPTH_FAR_M) -> float:
    """深度评分（1-10）→ 米。沿用 See, Point, Fly 的线性映射（1→0.2 m，10→上限）。"""
    clamped = min(max(float(score), 1.0), 10.0)
    return DEPTH_NEAR_M + (clamped - 1.0) / 9.0 * (depth_far_m - DEPTH_NEAR_M)


def parse_pointing(text: str, depth_far_m: float = DEPTH_FAR_M) -> Pointing:
    """解析 VLM 指向结果，兼容 See, Point, Fly 的 ``[{"point": [y, x], ...}]`` 格式。"""
    payload = _extract_json_payload(text)
    if isinstance(payload, list):
        if not payload:
            raise PointingParseError('VLM 返回了空列表')
        payload = payload[0]
    if not isinstance(payload, dict):
        raise PointingParseError('顶层 JSON 既不是对象也不是对象数组')

    if 'point' in payload:
        point = payload['point']
        if not isinstance(point, (list, tuple)) or len(point) < 2:
            raise PointingParseError('point 字段格式不正确，应为 [y, x]')
        y_raw, x_raw = point[0], point[1]
    elif 'x' in payload and 'y' in payload:
        x_raw, y_raw = payload['x'], payload['y']
    else:
        raise PointingParseError(f'缺少 point / x,y 字段：{payload}')

    depth_raw = payload.get('depth', payload.get('depth_score', 5.0))
    try:
        x_value = float(x_raw)
        y_value = float(y_raw)
        depth_value = float(depth_raw)
    except (TypeError, ValueError) as exc:
        raise PointingParseError(f'坐标或深度不是数字：{payload}') from exc

    return Pointing(
        x_norm=_to_unit(x_value),
        y_norm=_to_unit(y_value),
        depth_m=depth_score_to_meters(depth_value, depth_far_m),
        depth_raw=depth_value,
        label=str(payload.get('label', '')),
    )


class PointingGeometry:
    """相机指向几何：图像归一化坐标 + 深度 → 机体 FLU 向量。"""

    def __init__(self, image_width: int, image_height: int, hfov_deg: float = 90.0):
        if image_width <= 0 or image_height <= 0:
            raise ValueError('图像尺寸必须为正')
        self.image_width = int(image_width)
        self.image_height = int(image_height)
        self.hfov_deg = float(hfov_deg)
        self.aspect = self.image_width / self.image_height

    @property
    def vfov_deg(self) -> float:
        """竖直视场：按宽高比从水平视场推导（原实现两者相同，此处为有意差异）。"""
        half_h_fov = math.radians(self.hfov_deg) / 2.0
        return math.degrees(2.0 * math.atan(math.tan(half_h_fov) / self.aspect))

    def reverse_project(self, x_norm: float, y_norm: float, depth_m: float):
        """(x_norm, y_norm, 深度) → 机体系 (forward, left, up)，单位米。"""
        if depth_m <= 0.0:
            raise ValueError('深度必须为正')
        x_offset = (x_norm - 0.5) * 2.0        # -1(左) .. +1(右)
        y_offset = (0.5 - y_norm) * 2.0        # -1(下) .. +1(上)
        tan_h = math.tan(math.radians(self.hfov_deg) / 2.0)
        tan_v = math.tan(math.radians(self.vfov_deg) / 2.0)
        forward = depth_m
        left = -x_offset * depth_m * tan_h     # 图像右侧 → 机体右 → 左为负
        up = y_offset * depth_m * tan_v
        return (forward, left, up)

    def pixel_from_norm(self, x_norm: float, y_norm: float):
        """归一化坐标 → 像素坐标（用于可视化与日志）。"""
        return (
            int(x_norm * self.image_width),
            int(y_norm * self.image_height),
        )


def pointing_to_body_velocity(
    pointing: Pointing,
    geometry: PointingGeometry,
    base_velocity: float = 1.0,
    max_speed: float = 1.5,
    max_yaw_rate: float = 0.6,
    yaw_gain: float = 1.0,
    yaw_deadband_rad: float = 0.0,
    min_duration_s: float = 1.0,
) -> BodyVelocity:
    """指向结果 → 机体速度指令（带安全限幅）。

    - 水平方向：沿指向点的水平投影方向飞行，速度取 ``base_velocity``；
    - 竖直方向：保持指向点的俯仰比例，因此“指高处”会同时上升；
    - 偏航：按指向点的横向偏角做比例控制，使机头转向目标（同时飞，见模块说明）。
    """
    forward_m, left_m, up_m = geometry.reverse_project(
        pointing.x_norm, pointing.y_norm, pointing.depth_m)

    horizontal = math.hypot(forward_m, left_m)
    speed = min(float(base_velocity), float(max_speed))
    if speed <= 0.0:
        raise ValueError('base_velocity 必须为正')

    if horizontal < 1e-6:
        # 目标在正上/正下方：只做竖直运动，不偏航
        vertical = math.copysign(speed, up_m)
        return BodyVelocity(0.0, 0.0, vertical, 0.0, float(min_duration_s))

    unit_forward = forward_m / horizontal
    unit_left = left_m / horizontal
    velocity_forward = unit_forward * speed
    velocity_left = unit_left * speed
    velocity_up = (up_m / horizontal) * speed

    scale = math.hypot(velocity_forward, velocity_left, velocity_up) / max_speed
    if scale > 1.0:
        velocity_forward /= scale
        velocity_left /= scale
        velocity_up /= scale

    # 偏航：比例控制 + 死区。死区抑制“目标已在中心附近仍来回修正”的震荡
    # （SeePointFly 用约 10° 阈值决定是否转向，这里做成可配参数）。
    bearing = math.atan2(left_m, forward_m)
    if abs(bearing) < float(yaw_deadband_rad):
        yaw_rate = 0.0
    else:
        yaw_rate = max(-max_yaw_rate, min(max_yaw_rate, float(yaw_gain) * bearing))

    distance_m = math.sqrt(forward_m ** 2 + left_m ** 2 + up_m ** 2)
    duration = max(distance_m / speed, float(min_duration_s))
    return BodyVelocity(velocity_forward, velocity_left, velocity_up, yaw_rate, duration)
