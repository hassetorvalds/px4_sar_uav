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
    #: 远端 LightNav-0 风格的状态字段（不是坐标，必须单独处理）
    apos_state: str = ''    # 例如 rot_left / rot_right / stop / not_visible
    opos_state: str = ''

    @property
    def has_position(self) -> bool:
        """是否存在有效像素坐标（rot_* / not_visible 等状态动作没有坐标）。"""
        return not (math.isnan(self.x_norm) or math.isnan(self.y_norm))


@dataclass
class BodyVelocity:
    """机体 FLU 速度指令（前/左/上，m/s）与偏航角速度（rad/s，逆时针为正）。"""

    forward: float
    left: float
    up: float
    yaw_rate: float
    duration_s: float


@dataclass
class Waypoint:
    """远端 LightNav-0 的 ``waypoints`` 元素：机体坐标系位移（米）与偏航（弧度）。

    服务器侧说明：``apos`` 是**可通行落点**（"往哪走"），``waypoints`` 才是动作，
    其中 ``yaw`` 是给无人机用的**转向信号**——比把画面底边的落点做三维反投影稳健得多。
    """

    forward_m: float
    lateral_m: float
    yaw_rad: float

    @property
    def has_motion(self) -> bool:
        return any(abs(value) > 1e-6
                   for value in (self.forward_m, self.lateral_m, self.yaw_rad))


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


def _clip_unit(value: float) -> float:
    """裁剪到 [-1, 1]（路点归一化用）。"""
    return min(max(float(value), -1.0), 1.0)


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

    # 远端服务格式（LightNav-0 风格）：
    # {"pointing": {"x_norm":.., "y_norm":.., "apos_state":.., "opos_state":..}}
    if 'pointing' in payload and isinstance(payload['pointing'], dict):
        remote = payload['pointing']
        raw_x, raw_y = remote.get('x_norm'), remote.get('y_norm')
        # 状态类动作（rot_left / rot_right / stop / not_visible）没有坐标，
        # 此时 x_norm/y_norm 为 null——必须保留该项而不是报错，由调用方按状态处理。
        if raw_x is None or raw_y is None:
            x_value = y_value = float('nan')
        else:
            try:
                x_value = float(raw_x)
                y_value = float(raw_y)
            except (TypeError, ValueError) as exc:
                raise PointingParseError(
                    f'远端 pointing 的 x_norm/y_norm 非法：{remote}') from exc
        return Pointing(
            x_norm=min(max(x_value, 0.0), 1.0) if not math.isnan(x_value) else x_value,
            y_norm=min(max(y_value, 0.0), 1.0) if not math.isnan(y_value) else y_value,
            depth_m=depth_score_to_meters(5.0, depth_far_m),   # 占位；深度由本机补
            depth_raw=0.0,
            label=str(remote.get('label', 'lightnav remote pointing')),
            apos_state=str(remote.get('apos_state', '')),
            opos_state=str(remote.get('opos_state', '')),
        )

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
        apos_state=str(payload.get('apos_state', '')),
        opos_state=str(payload.get('opos_state', '')),
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
    stop_distance_m: float = 0.0,
    min_duration_s: float = 1.0,
    vertical_mode: str = 'hold',
    max_vertical_mps: float = 0.4,
) -> BodyVelocity:
    """指向结果 → 机体速度指令（带安全限幅）。

    - 水平方向：沿指向点的水平投影方向飞行，速度取 ``base_velocity``；
    - 竖直方向：``vertical_mode='hold'``（默认）**恒为 0**，高度交给飞控保持；
      ``'ray'`` 才按指向俯仰比例给竖直速度（**对无人机不安全**，仅作对照）；
    - 偏航：按指向点的横向偏角做比例控制，使机头转向目标（同时飞，见模块说明）。

    为什么默认 ``hold``（2026-10-11 与服务器侧确认后改）：
    LightNav-0 的 ``apos`` 是**可通行落点**而非目标物体，户外场景里它就落在画面底边中心
    （"向前走"）。把它按像素反投影会得到一条朝下的射线，于是算出 −0.5 m/s 的下降速度——
    地面机器人这么算等于"前进"，无人机这么算等于"往下扎"。所以水平分量照用，竖直分量归零。
    """
    forward_m, left_m, up_m = geometry.reverse_project(
        pointing.x_norm, pointing.y_norm, pointing.depth_m)
    use_vertical = str(vertical_mode).lower() != 'hold'

    # 接近到阈值内就停止平移，只保留可能的偏航修正。
    # 没有这道保护时会一路飞向目标（实测飞到目标 0.2 m 处仍以 1 m/s 前飞，
    # 位移约 12 m、高度冲到 4.4 m）；SeePointFly 的 adaptive 模式也是这么做的。
    bearing = math.atan2(left_m, forward_m)
    if stop_distance_m > 0.0 and pointing.depth_m <= float(stop_distance_m):
        if abs(bearing) < float(yaw_deadband_rad):
            return BodyVelocity(0.0, 0.0, 0.0, 0.0, float(min_duration_s))
        yaw_only = max(-max_yaw_rate, min(max_yaw_rate, float(yaw_gain) * bearing))
        return BodyVelocity(0.0, 0.0, 0.0, yaw_only, float(min_duration_s))

    horizontal = math.hypot(forward_m, left_m)
    speed = min(float(base_velocity), float(max_speed))
    if speed <= 0.0:
        raise ValueError('base_velocity 必须为正')

    if horizontal < 1e-6:
        # 目标在正上/正下方
        if not use_vertical:
            return BodyVelocity(0.0, 0.0, 0.0, 0.0, float(min_duration_s))
        vertical = math.copysign(min(speed, max_vertical_mps), up_m)
        return BodyVelocity(0.0, 0.0, vertical, 0.0, float(min_duration_s))

    unit_forward = forward_m / horizontal
    unit_left = left_m / horizontal
    velocity_forward = unit_forward * speed
    velocity_left = unit_left * speed
    if use_vertical:
        velocity_up = (up_m / horizontal) * speed
        velocity_up = max(-max_vertical_mps, min(max_vertical_mps, velocity_up))
    else:
        velocity_up = 0.0

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


def parse_waypoints(text: str) -> list:
    """从远端响应里取出 ``waypoints``（没有或为 null 时返回空列表）。

    兼容三种写法（服务器侧接口 v2 的 ``waypoints`` 字段）：

    ```text
    "waypoints": null                                  → []
    "waypoints": [[1.2, 0.0, 0.3], ...]                → 依次 [forward_m, lateral_m, yaw_rad]
    "waypoints": [{"forward_m":1.2,"lateral_m":0,"yaw_rad":0.3}, ...]
    ```

    ``yaw`` 单位约定为**弧度**；若某元素 ``|yaw| > 4π``（≈12.6），按角度处理并换算，
    以免服务器侧误用度数时把飞机甩出去。
    """
    try:
        payload = _extract_json_payload(text)
    except PointingParseError:
        return []

    raw = payload.get('waypoints') if isinstance(payload, dict) else payload
    if not raw:
        return []
    if isinstance(raw, (dict, str, int, float)):
        raw = [raw]

    waypoints = []
    for item in raw:
        try:
            if isinstance(item, dict):
                forward = item.get('forward_m', item.get('forward', item.get('x')))
                lateral = item.get('lateral_m', item.get('lateral', item.get('y')))
                yaw = item.get('yaw_rad', item.get('yaw', item.get('yaw_rate', 0.0)))
                if forward is None or lateral is None:
                    continue
            elif isinstance(item, (list, tuple)) and len(item) >= 3:
                forward, lateral, yaw = item[0], item[1], item[2]
            else:
                continue
            forward = float(forward)
            lateral = float(lateral)
            yaw = float(yaw)
            if abs(yaw) > 4.0 * math.pi:
                yaw = math.radians(yaw)
            if not all(math.isfinite(value) for value in (forward, lateral, yaw)):
                continue
        except (TypeError, ValueError):
            continue
        waypoints.append(Waypoint(forward, lateral, yaw))
    return waypoints


def waypoint_to_body_velocity(
    waypoint: Waypoint,
    base_velocity: float = 1.0,
    max_speed: float = 1.5,
    max_yaw_rate: float = 0.6,
    yaw_gain: float = 1.0,
    yaw_deadband_rad: float = 0.0,
    min_duration_s: float = 1.0,
    forward_norm_m: float = 0.375,
    lateral_norm_m: float = 0.25,
    yaw_norm_rad: float = math.pi / 20.0,
    max_lateral_speed: float = 1.0,
) -> BodyVelocity:
    """路点 → 机体速度指令：**水平速度 + 偏航角速度，竖直恒为 0**（高度交给飞控）。

    归一化方式照服务器侧给的参考客户端（2026-10-12 采纳）：

    ```text
    vx   = clip(forward_m / 0.375) * 巡航速度
    vy   = clip(lateral_m / 0.25)  * 横向上限
    vyaw = clip(yaw_rad / (π/20))  * 偏航上限 * yaw_gain
    ```

    为什么不用"0.5 m 才算全速"的写法：LightNav-0 每步平移上限约 **0.25 m**、
    偏航约 **30°**，`waypoints[0]` 的 `forward` 通常在 0.15 m 量级——
    用 0.5 m 当阈值几乎永远触发不了巡航速度（实测我们真实帧的 `forward`≈0.000，
    即"原地转向"）。所以按参考客户端的**相对量程**归一化。

    注意：`waypoints` 是**累积位姿**序列（chunk 起点 ego 系），只取 `[0]`（第一步），
    其余是前瞻，本函数只接受单个 ``Waypoint``。
    """
    speed = min(float(base_velocity), float(max_speed))
    if speed <= 0.0:
        raise ValueError('base_velocity 必须为正')

    forward_span = max(float(forward_norm_m), 1e-6)
    lateral_span = max(float(lateral_norm_m), 1e-6)
    yaw_span = max(float(yaw_norm_rad), 1e-6)

    forward = _clip_unit(waypoint.forward_m / forward_span) * speed
    left = _clip_unit(waypoint.lateral_m / lateral_span) * min(
        float(max_lateral_speed), speed)

    if abs(waypoint.yaw_rad) < float(yaw_deadband_rad):
        yaw_rate = 0.0
    else:
        yaw_rate = (_clip_unit(float(yaw_gain) * waypoint.yaw_rad / yaw_span)
                    * float(max_yaw_rate))

    return BodyVelocity(forward, left, 0.0, yaw_rate, float(min_duration_s))
