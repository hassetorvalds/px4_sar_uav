"""指向解析与几何反投影的单元测试（离线，不需要任何 API key）。"""

import math

import pytest

from vlm.pointing import (
    PointingGeometry,
    PointingParseError,
    depth_score_to_meters,
    parse_pointing,
    parse_waypoints,
    pointing_to_body_velocity,
    waypoint_to_body_velocity,
)


def test_parse_see_point_fly_format():
    # 归一化 0-1000，point 为 [y, x]
    text = '[{"point": [250, 750], "depth": 4, "label": "building entrance"}]'
    pointing = parse_pointing(text)
    assert pointing.x_norm == pytest.approx(0.75)
    assert pointing.y_norm == pytest.approx(0.25)
    assert pointing.depth_raw == pytest.approx(4.0)
    assert pointing.label == 'building entrance'


def test_parse_tolerates_markdown_and_extra_text():
    text = 'Here is the answer:\n```json\n[{"point": [500, 500], "depth": 6}]\n```\nhope it helps'
    pointing = parse_pointing(text)
    assert (pointing.x_norm, pointing.y_norm) == pytest.approx((0.5, 0.5))


def test_parse_accepts_x_y_object_and_unit_range():
    pointing = parse_pointing('{"x": 0.25, "y": 0.75, "depth": 2}')
    assert pointing.x_norm == pytest.approx(0.25)
    assert pointing.y_norm == pytest.approx(0.75)


def test_parse_rejects_garbage():
    with pytest.raises(PointingParseError):
        parse_pointing('there is nothing here')
    with pytest.raises(PointingParseError):
        parse_pointing('[{"point": "center of the building"}]')
    with pytest.raises(PointingParseError):
        parse_pointing('[{"point": [5]}]')
    with pytest.raises(PointingParseError):
        parse_pointing('[{"point": [500, 500], "depth": "far"}]')


def test_parse_accepts_extra_point_components():
    # 有些模型会多给一个置信度，允许忽略多余元素
    pointing = parse_pointing('[{"point": [250, 750, 0.9], "depth": 4}]')
    assert (pointing.x_norm, pointing.y_norm) == pytest.approx((0.75, 0.25))


def test_depth_mapping_matches_reference():
    # See, Point, Fly: score/10*2.0 → 1→0.2 m，10→2.0 m
    assert depth_score_to_meters(1) == pytest.approx(0.2)
    assert depth_score_to_meters(10) == pytest.approx(2.0)
    assert depth_score_to_meters(100) == pytest.approx(2.0)  # 越界裁剪


def test_vfov_derived_from_aspect_ratio():
    geometry = PointingGeometry(1920, 1080, hfov_deg=90.0)
    # 16:9 竖直视场应小于水平视场
    assert geometry.vfov_deg < geometry.hfov_deg
    assert geometry.vfov_deg == pytest.approx(
        math.degrees(2.0 * math.atan(math.tan(math.radians(45.0)) / (1920 / 1080))), abs=1e-6)


def test_center_point_projects_straight_ahead():
    geometry = PointingGeometry(640, 480, hfov_deg=90.0)
    forward, left, up = geometry.reverse_project(0.5, 0.5, 1.5)
    assert (forward, left, up) == pytest.approx((1.5, 0.0, 0.0), abs=1e-9)


def test_right_point_projects_to_body_right():
    geometry = PointingGeometry(640, 480, hfov_deg=90.0)
    _, left, _ = geometry.reverse_project(1.0, 0.5, 1.0)
    assert left < 0.0  # 图像右侧 = 机体右侧 = 左向为负


def test_up_point_projects_to_positive_up():
    geometry = PointingGeometry(640, 480, hfov_deg=90.0)
    _, _, up = geometry.reverse_project(0.5, 0.0, 1.0)
    assert up > 0.0


def test_body_velocity_respects_speed_limits():
    geometry = PointingGeometry(640, 480, hfov_deg=90.0)
    pointing = parse_pointing('{"point": [500, 1000], "depth": 10}')
    command = pointing_to_body_velocity(
        pointing, geometry, base_velocity=1.0, max_speed=1.5, max_yaw_rate=0.6)
    speed = math.sqrt(command.forward ** 2 + command.left ** 2 + command.up ** 2)
    assert speed <= 1.5 + 1e-9
    assert abs(command.yaw_rate) <= 0.6 + 1e-9
    assert command.duration_s >= 1.0


def test_yaw_rate_sign_follows_target_side():
    geometry = PointingGeometry(640, 480, hfov_deg=90.0)
    left_target = parse_pointing('{"point": [500, 200], "depth": 5}')
    right_target = parse_pointing('{"point": [500, 800], "depth": 5}')
    assert pointing_to_body_velocity(left_target, geometry).yaw_rate > 0.0
    assert pointing_to_body_velocity(right_target, geometry).yaw_rate < 0.0


def test_yaw_deadband_zeros_small_bearing():
    geometry = PointingGeometry(640, 480, hfov_deg=90.0)
    centered = parse_pointing('{"point": [500, 505], "depth": 5}')
    with_deadband = pointing_to_body_velocity(
        centered, geometry, yaw_gain=1.0, yaw_deadband_rad=0.17)
    assert with_deadband.yaw_rate == 0.0
    # 不设死区时同一输入会产生非零偏航，说明死区确实起作用
    assert pointing_to_body_velocity(centered, geometry, yaw_gain=1.0).yaw_rate != 0.0


def test_yaw_deadband_keeps_large_bearing():
    geometry = PointingGeometry(640, 480, hfov_deg=90.0)
    far_right = parse_pointing('{"point": [500, 900], "depth": 5}')
    command = pointing_to_body_velocity(
        far_right, geometry, yaw_gain=0.5, max_yaw_rate=0.4, yaw_deadband_rad=0.17)
    assert command.yaw_rate < 0.0
    assert abs(command.yaw_rate) <= 0.4 + 1e-9


def test_lower_gain_reduces_yaw_command():
    geometry = PointingGeometry(640, 480, hfov_deg=90.0)
    pointing = parse_pointing('{"point": [500, 600], "depth": 5}')
    strong = pointing_to_body_velocity(pointing, geometry, yaw_gain=1.0)
    weak = pointing_to_body_velocity(pointing, geometry, yaw_gain=0.5)
    assert abs(weak.yaw_rate) < abs(strong.yaw_rate)


def test_stop_distance_zeroes_translation():
    geometry = PointingGeometry(640, 480, hfov_deg=90.0)
    # depth=1 → 0.2 m，小于停止距离 0.35 m：应完全停止平移
    close = parse_pointing('{"point": [500, 520], "depth": 1}')
    command = pointing_to_body_velocity(
        close, geometry, base_velocity=1.0,
        yaw_deadband_rad=0.17, stop_distance_m=0.35)
    assert (command.forward, command.left, command.up) == pytest.approx((0.0, 0.0, 0.0))
    assert command.yaw_rate == 0.0          # 目标在死区内：平移与偏航都为零


def test_stop_distance_keeps_yaw_when_target_off_center():
    geometry = PointingGeometry(640, 480, hfov_deg=90.0)
    close_but_left = parse_pointing('{"point": [500, 100], "depth": 1}')
    command = pointing_to_body_velocity(
        close_but_left, geometry, base_velocity=1.0,
        stop_distance_m=0.35, yaw_gain=0.5, max_yaw_rate=0.4)
    assert (command.forward, command.left, command.up) == pytest.approx((0.0, 0.0, 0.0))
    assert command.yaw_rate > 0.0


def test_far_target_still_translates_with_stop_distance():
    geometry = PointingGeometry(640, 480, hfov_deg=90.0)
    far = parse_pointing('{"point": [500, 520], "depth": 8}')
    command = pointing_to_body_velocity(
        far, geometry, base_velocity=1.0, stop_distance_m=0.35)
    assert command.forward > 0.5


# --- 2026-10-11 与服务器侧确认后新增：竖直分量默认不交给视觉 ---

def _remote_pointing(x_norm: float, y_norm: float, depth_m: float):
    """构造一条"远端 LightNav-0 风格"的指向（深度由本机补，这里直接给实际距离）。"""
    pointing = parse_pointing(
        '{"pointing": {"x_norm": %r, "y_norm": %r, "apos_state": "point",'
        ' "opos_state": "not_visible"}}' % (x_norm, y_norm))
    pointing.depth_m = depth_m       # 本机深度采样结果
    return pointing


def test_vertical_mode_hold_zeroes_descent_for_bottom_edge_apos():
    """LightNav-0 的 apos 在画面底边（y_norm≈0.98）代表"向前"的落点，
    反投影会得到朝下的射线；默认 vertical_mode='hold' 必须把竖直分量压成 0，
    否则无人机会一边前飞一边下降（Step B 实测 −0.51 m/s）。"""
    geometry = PointingGeometry(1280, 720, hfov_deg=90.0)
    bottom = _remote_pointing(0.5104, 0.9815, depth_m=3.0)

    holding = pointing_to_body_velocity(
        bottom, geometry, base_velocity=1.0, vertical_mode='hold')
    assert holding.forward > 0.5
    assert holding.up == pytest.approx(0.0)

    legacy = pointing_to_body_velocity(
        bottom, geometry, base_velocity=1.0, vertical_mode='ray')
    assert legacy.up <= -0.3           # 旧行为：明显下降，正是要避免的


def test_vertical_mode_ray_respects_clamp():
    geometry = PointingGeometry(1280, 720, hfov_deg=90.0)
    bottom = _remote_pointing(0.5, 1.0, depth_m=3.0)
    command = pointing_to_body_velocity(
        bottom, geometry, base_velocity=1.5, max_speed=1.5,
        vertical_mode='ray', max_vertical_mps=0.2)
    assert command.up >= -0.2


def test_parse_waypoints_accepts_list_and_dict_and_null():
    assert parse_waypoints('{"pointing": {}, "waypoints": null}') == []
    lists = parse_waypoints('{"waypoints": [[1.2, -0.3, 0.25]]}')
    assert len(lists) == 1
    assert lists[0].forward_m == pytest.approx(1.2)
    assert lists[0].lateral_m == pytest.approx(-0.3)
    assert lists[0].yaw_rad == pytest.approx(0.25)
    dicts = parse_waypoints('{"waypoints": [{"forward_m": 0.8, "lateral_m": 0.1, "yaw_rad": -0.4}]}')
    assert dicts[0].yaw_rad == pytest.approx(-0.4)
    degrees = parse_waypoints('{"waypoints": [[1.0, 0.0, 30.0]]}')
    assert degrees[0].yaw_rad == pytest.approx(math.radians(30.0))


def test_waypoint_to_body_velocity_keeps_altitude_and_maps_yaw():
    waypoint = parse_waypoints('{"waypoints": [[0.4, -0.2, 0.5]]}')[0]
    command = waypoint_to_body_velocity(
        waypoint, base_velocity=1.0, max_speed=1.5, max_yaw_rate=0.6,
        yaw_gain=1.0, min_duration_s=1.0, full_speed_distance_m=0.5,
        max_lateral_speed=1.0)
    assert command.forward == pytest.approx(0.8)     # 0.4/0.5 → 80% 速度
    assert command.left == pytest.approx(-0.2)
    assert command.up == pytest.approx(0.0)          # 高度交飞控
    assert command.yaw_rate == pytest.approx(0.5)
