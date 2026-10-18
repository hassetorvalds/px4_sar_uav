"""自然语言解析与搜索航迹的单测（纯逻辑，不需要 ROS）。"""

import math

import pytest

from mission.search_pattern import estimate_duration_s, lawnmower, path_length
from mission.task_parser import parse_task
from mission.task_spec import TaskError


def test_takeoff_goto_hover_land():
    spec = parse_task('起飞到 2 米，向北飞 10 米，悬停 5 秒，降落')
    types = [s.type for s in spec.steps]
    assert types == ['takeoff', 'goto', 'hover', 'land']
    assert spec.steps[0].params['altitude_m'] == pytest.approx(2.0)
    assert spec.steps[1].params['north_m'] == pytest.approx(10.0)
    assert spec.steps[1].params['east_m'] == pytest.approx(0.0)
    assert spec.steps[2].params['seconds'] == pytest.approx(5.0)


def test_search_area_with_target_after():
    spec = parse_task('搜索北侧 20 米、半径 10 米的区域，寻找橙色标记，高度 2 米')
    search = [s for s in spec.steps if s.type == 'search']
    assert len(search) == 1
    params = search[0].params
    assert params['center_north_m'] == pytest.approx(20.0)
    assert params['radius_m'] == pytest.approx(10.0)
    assert params['target']['kind'] == 'color'
    assert params['target']['label'] == 'orange'
    assert spec.altitude_m == pytest.approx(2.0)


def test_search_target_before_area():
    spec = parse_task('寻找人员，搜索半径 6 米的区域，降落')
    search = [s for s in spec.steps if s.type == 'search']
    assert len(search) == 1
    assert search[0].params['target']['kind'] == 'person'
    assert search[0].params['radius_m'] == pytest.approx(6.0)


def test_return_home_counts_as_landing():
    spec = parse_task('起飞到 2 米，向北飞 5 米，返航降落')
    assert spec.steps[-1].type == 'return_home'
    assert all(s.type != 'land' for s in spec.steps)


def test_forward_is_body_frame_step():
    spec = parse_task('起飞到 2 米，向前飞 10 米，降落')
    types = [s.type for s in spec.steps]
    assert types == ['takeoff', 'forward', 'land']
    assert spec.steps[1].params['meters'] == pytest.approx(10.0)


def test_multi_direction_goto():
    spec = parse_task('去东边 15 米、北边 5 米的位置，然后降落')
    goto = [s for s in spec.steps if s.type == 'goto'][0]
    assert goto.params['east_m'] == pytest.approx(15.0)
    assert goto.params['north_m'] == pytest.approx(5.0)


def test_missing_takeoff_and_land_are_added():
    spec = parse_task('向北飞 5 米')
    assert spec.steps[0].type == 'takeoff'
    assert spec.steps[-1].type == 'land'
    assert any('自动补' in note for note in spec.notes)


def test_nonsense_is_rejected():
    with pytest.raises(TaskError, match='没有识别出'):
        parse_task('今天天气不错')


def test_out_of_range_altitude_is_rejected():
    with pytest.raises(TaskError, match='altitude_m'):
        parse_task('起飞到 50 米，降落')


def test_fullwidth_digits_are_normalized():
    spec = parse_task('起飞到 ２ 米，向北飞 ５ 米，降落')
    assert spec.steps[0].params['altitude_m'] == pytest.approx(2.0)
    assert spec.steps[1].params['north_m'] == pytest.approx(5.0)


def test_chinese_numerals():
    spec = parse_task('起飞到 两 米，向北飞 十 米，降落')
    assert spec.steps[0].params['altitude_m'] == pytest.approx(2.0)
    assert spec.steps[1].params['north_m'] == pytest.approx(10.0)


def test_english_command():
    spec = parse_task('take off to 2 m, search radius 5 m for a person, land')
    types = [s.type for s in spec.steps]
    assert types == ['takeoff', 'search', 'land']
    assert spec.steps[1].params['target']['kind'] == 'person'


def test_goto_beyond_geofence_rejected():
    with pytest.raises(TaskError, match='围栏'):
        parse_task('向北飞 100 米，降落')


def test_lawnmower_covers_square():
    waypoints = lawnmower(0.0, 0.0, 10.0, 5.0)
    xs = [p[0] for p in waypoints]
    ys = [p[1] for p in waypoints]
    assert min(xs) == pytest.approx(-10.0)
    assert max(xs) == pytest.approx(10.0)
    assert min(ys) == pytest.approx(-10.0)
    assert max(ys) == pytest.approx(10.0)
    # 相邻行的北向间距不超过 spacing
    lanes = sorted(set(round(y, 6) for y in ys))
    assert all(b - a <= 5.0 + 1e-6 for a, b in zip(lanes, lanes[1:]))


def test_lawnmower_alternates_direction():
    waypoints = lawnmower(0.0, 0.0, 4.0, 4.0)
    assert waypoints[0][0] == pytest.approx(-4.0)
    assert waypoints[1][0] == pytest.approx(4.0)
    assert waypoints[2][0] == pytest.approx(4.0)
    assert waypoints[3][0] == pytest.approx(-4.0)


def test_path_length_and_duration():
    waypoints = lawnmower(0.0, 0.0, 10.0, 5.0)
    assert path_length(waypoints) > 0
    duration = estimate_duration_s(waypoints, 0.5)
    assert duration > path_length(waypoints) / 0.5


def test_lawnmower_rejects_bad_input():
    with pytest.raises(ValueError):
        lawnmower(0, 0, 0, 1)
    with pytest.raises(ValueError):
        lawnmower(0, 0, 1, 0)
