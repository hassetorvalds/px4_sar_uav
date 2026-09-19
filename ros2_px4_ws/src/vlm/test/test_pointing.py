"""指向解析与几何反投影的单元测试（离线，不需要任何 API key）。"""

import math

import pytest

from vlm.pointing import (
    PointingGeometry,
    PointingParseError,
    depth_score_to_meters,
    parse_pointing,
    pointing_to_body_velocity,
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
