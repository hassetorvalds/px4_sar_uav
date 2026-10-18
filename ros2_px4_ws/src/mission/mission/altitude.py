"""高度帧约定：**任何高度值都必须带帧标签**。

背景（2026-10-18 实飞）：PX4 的 local z 在触地后仍报 -0.5 m 左右（EKF 漂移），
而 AirSim 真值已经在地面。把两种高度混在一个判据里（高度门/报告）会直接判错。
参考 Simurgh/MDS 的做法：显示与记录都标 `REL / LCL / BARO / MSL`，缺失就是 `Unknown`。

本模块只做标签与比较守卫，不做物理换算（换算需要明确的参考点）。
"""

import math
from enum import Enum


class AltitudeFrame(str, Enum):
    REL = 'REL'      # 相对起飞点/home（任务高度、相对爬升用这个）
    LCL = 'LCL'      # PX4 local NED/ENU 的 z（可能漂移，不能当绝对高度）
    BARO = 'BARO'    # 气压高度
    MSL = 'MSL'      # 海拔
    TRUTH = 'TRUTH'  # AirSim 真值；**仅诊断/校验**，不参与控制


FRAMES = tuple(frame.value for frame in AltitudeFrame)


def parse_frame(value, default=AltitudeFrame.REL) -> AltitudeFrame:
    """把字符串/枚举规范成 :class:`AltitudeFrame`；未知值抛 ValueError。"""
    if isinstance(value, AltitudeFrame):
        return value
    text = str(value or '').strip().upper()
    if not text:
        return default
    try:
        return AltitudeFrame(text)
    except ValueError as exc:
        raise ValueError(f'未知高度帧 {value!r}；可选 {FRAMES}') from exc


def format_altitude(value, frame) -> str:
    """``format_altitude(2.0, 'REL') -> '2.00 m REL'``；None → ``'Unknown'``。"""
    if value is None or not math.isfinite(float(value)):
        return 'Unknown'
    label = parse_frame(frame).value
    return f'{float(value):.2f} m {label}'


def require_same_frame(frame_a, frame_b, *, what: str = '高度'):
    """比较两个高度前必须先确认同帧；不同帧直接抛错。"""
    a, b = parse_frame(frame_a), parse_frame(frame_b)
    if a != b:
        raise ValueError(f'{what}帧不一致：{a.value} vs {b.value}（先换算到同一帧再比较）')
    return a


def to_dict(value, frame, source: str = ''):
    """报告/CSV 用的统一结构：值、帧、来源三件套。"""
    return {
        'value_m': (None if value is None or not math.isfinite(float(value))
                    else round(float(value), 3)),
        'frame': parse_frame(frame).value,
        'source': source or 'unknown',
    }


def is_control_safe(frame) -> bool:
    """``TRUTH`` 只用于诊断，不能作为控制/判定输入。"""
    return parse_frame(frame) != AltitudeFrame.TRUTH
