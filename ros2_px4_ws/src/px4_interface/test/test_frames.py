"""坐标系转换的单元测试。

这些测试锁死全项目的坐标约定：ROS 侧 ENU、PX4 侧 NED，
转换只在 px4_interface 边界发生。
"""

import math

import pytest

from px4_interface.frames import (
    distance,
    enu_to_ned,
    enu_yaw_to_ned_yaw,
    ned_to_enu,
    ned_yaw_to_enu_yaw,
    normalize_angle,
)


def test_takeoff_altitude_maps_to_negative_ned_z():
    # 3 m 高度：ENU z=+3 对应 NED z=-3（这正是实机验证时看到的 z ≈ -3）
    assert enu_to_ned(0.0, 0.0, 3.0) == (0.0, 0.0, -3.0)
    assert ned_to_enu(0.0, 0.0, -3.0) == (0.0, 0.0, 3.0)


def test_east_north_axes_are_swapped():
    assert enu_to_ned(1.0, 2.0, 0.0) == (2.0, 1.0, 0.0)
    assert ned_to_enu(2.0, 1.0, 0.0) == (1.0, 2.0, 0.0)


@pytest.mark.parametrize('point', [(0.0, 0.0, 0.0), (1.5, -2.5, 3.0), (-4.0, 0.5, -1.0)])
def test_conversion_is_involutive(point):
    # NED→ENU 与 ENU→NED 使用同一组公式，互为逆变换
    assert ned_to_enu(*enu_to_ned(*point)) == pytest.approx(point)


def test_yaw_conventions():
    # NED yaw=0 指北 → ENU yaw=+pi/2（自东逆时针）
    assert ned_yaw_to_enu_yaw(0.0) == pytest.approx(math.pi / 2.0)
    # NED yaw=+pi/2 指东 → ENU yaw=0
    assert ned_yaw_to_enu_yaw(math.pi / 2.0) == pytest.approx(0.0)
    # 互为逆变换
    assert enu_yaw_to_ned_yaw(ned_yaw_to_enu_yaw(0.3)) == pytest.approx(0.3)


def test_normalize_angle_wraps_into_pi_range():
    # 归一化后落在 [-pi, pi]；3π 与 -3π 都等价于“朝向反方向”，取绝对值应为 pi
    assert normalize_angle(3.0 * math.pi) == pytest.approx(math.pi, abs=1e-9)
    assert abs(normalize_angle(-3.0 * math.pi)) == pytest.approx(math.pi, abs=1e-9)
    # 1.5π 等价于 -0.5π
    assert normalize_angle(1.5 * math.pi) == pytest.approx(-0.5 * math.pi)
    assert normalize_angle(-1.5 * math.pi) == pytest.approx(0.5 * math.pi)


def test_distance():
    assert distance((0.0, 0.0, 0.0), (3.0, 4.0, 0.0)) == pytest.approx(5.0)
