"""PX4(NED/FRD) 与 ROS(ENU/FLU) 之间的坐标转换。

全项目统一约定：

- **PX4 侧**（uORB / ``/fmu/*`` 话题）：NED 坐标系，x 指北、y 指东、z 指下；
  姿态为 FRD。``vehicle_local_position.z`` 在 3 m 高度上是 -3。
- **ROS 2 侧**（mission / planner / perception 之间的内部话题）：ENU 坐标系，
  x 指东、y 指北、z 指上；姿态为 FLU。
- 转换只在 ``px4_interface`` 边界发生一次，其它模块一律使用 ROS 约定。

已实测确认：PX4 的 DDS 桥不做 NED→ENU 转换，
``/fmu/out/vehicle_local_position_v1`` 与 uORB 数值一致（见 CHANGELOG 2026-09-14 条目）。
"""

import math

#: 根据当前 X 轴方向定义，NED→ENU 的偏航换算为 yaw_enu = pi/2 - yaw_ned
_YAW_OFFSET_RAD = math.pi / 2.0


def normalize_angle(angle_rad: float) -> float:
    """把角度归一化到 [-pi, pi]（atan2 语义，±pi 视为同一方向的等价值）。"""
    return math.atan2(math.sin(angle_rad), math.cos(angle_rad))


def ned_to_enu(x_ned: float, y_ned: float, z_ned: float):
    """位置：NED(北,东,下) → ENU(东,北,上)。"""
    return (y_ned, x_ned, -z_ned)


def enu_to_ned(x_enu: float, y_enu: float, z_enu: float):
    """位置：ENU(东,北,上) → NED(北,东,下)。"""
    return (y_enu, x_enu, -z_enu)


def ned_yaw_to_enu_yaw(yaw_ned: float) -> float:
    """偏航：NED（自北顺时针）→ ENU（自东逆时针）。"""
    return normalize_angle(_YAW_OFFSET_RAD - yaw_ned)


def enu_yaw_to_ned_yaw(yaw_enu: float) -> float:
    """偏航：ENU（自东逆时针）→ NED（自北顺时针）。"""
    return normalize_angle(_YAW_OFFSET_RAD - yaw_enu)


def distance(a, b) -> float:
    """两个三维点的欧氏距离。"""
    return math.sqrt(sum((float(ai) - float(bi)) ** 2 for ai, bi in zip(a, b)))


def body_flu_to_enu(forward: float, left: float, up: float, heading_enu: float):
    """机体速度（FLU：前/左/上）→ ENU 世界速度。

    ``heading_enu`` 为 ENU 偏航角（自东轴逆时针，弧度）。
    前向单位向量为 (cosψ, sinψ)，左向为其逆时针旋转 90°，即 (-sinψ, cosψ)。
    """
    cos_h = math.cos(heading_enu)
    sin_h = math.sin(heading_enu)
    east = forward * cos_h - left * sin_h
    north = forward * sin_h + left * cos_h
    return (east, north, up)


def flip_yaw_rate_sign(rate: float) -> float:
    """偏航角速度在 ENU(逆时针为正) 与 NED(顺时针为正) 之间转换。"""
    return -rate
