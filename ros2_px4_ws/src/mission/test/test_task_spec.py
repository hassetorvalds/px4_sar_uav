"""TaskSpec schema / 顺序 / 范围校验的单测。"""

import pytest

from mission.task_spec import Step, TaskError, TaskSpec


def _spec(steps, **kwargs):
    return TaskSpec(steps=steps, **kwargs)


def test_valid_minimal_task():
    spec = _spec([
        Step('takeoff', {'altitude_m': 2.0}),
        Step('goto', {'east_m': 5.0, 'north_m': 0.0}),
        Step('land', {}),
    ]).validate()
    assert spec.steps[1].type == 'goto'


def test_first_step_must_be_takeoff():
    with pytest.raises(TaskError, match='第一个 step'):
        _spec([Step('land', {})]).validate()


def test_must_end_with_land_or_return():
    with pytest.raises(TaskError, match='缺少收尾'):
        _spec([Step('takeoff', {}), Step('goto', {'east_m': 5.0})]).validate()


def test_no_step_after_land():
    with pytest.raises(TaskError, match='最后'):
        _spec([Step('takeoff', {}), Step('land', {}),
               Step('goto', {'east_m': 1.0})]).validate()


def test_altitude_out_of_range():
    with pytest.raises(TaskError, match='altitude_m'):
        _spec([Step('takeoff', {}), Step('land', {})], altitude_m=20.0).validate()


def test_goto_beyond_geofence():
    with pytest.raises(TaskError, match='围栏'):
        _spec([Step('takeoff', {}),
               Step('goto', {'east_m': 100.0, 'north_m': 0.0}),
               Step('land', {})], geofence_radius_m=20.0).validate()


def test_unknown_step_type():
    with pytest.raises(TaskError, match='类型未知'):
        _spec([Step('takeoff', {}), Step('teleport', {}), Step('land', {})]).validate()


def test_hover_seconds_range():
    with pytest.raises(TaskError, match='hover.seconds'):
        _spec([Step('takeoff', {}), Step('hover', {'seconds': 999.0}),
               Step('land', {})]).validate()


def test_search_target_kind_required():
    with pytest.raises(TaskError, match='target'):
        _spec([Step('takeoff', {}),
               Step('search', {'radius_m': 5.0, 'spacing_m': 2.0,
                               'target': {'label': 'x'}}),
               Step('land', {})]).validate()


def test_json_roundtrip():
    spec = _spec([Step('takeoff', {'altitude_m': 2.0}),
                  Step('goto', {'east_m': 3.0, 'north_m': 4.0,
                                'altitude_m': 2.0}),
                  Step('land', {})], source_text='测试').validate()
    restored = TaskSpec.from_json(spec.to_json())
    assert restored.to_dict() == spec.to_dict()
    assert restored.summary().startswith('起飞')


def test_from_json_rejects_bad_version():
    with pytest.raises(TaskError, match='schema 版本'):
        TaskSpec.from_json('{"version": 99, "steps": []}')


def test_altitude_frame_default_and_roundtrip():
    spec = _spec([Step('takeoff', {}), Step('land', {})]).validate()
    assert spec.altitude_frame == 'REL'
    assert spec.to_dict()['altitude_frame'] == 'REL'
    restored = TaskSpec.from_dict({**spec.to_dict(), 'altitude_frame': 'LCL'})
    assert restored.altitude_frame == 'LCL'


def test_altitude_frame_rejects_unknown():
    with pytest.raises(TaskError, match='高度帧'):
        _spec([Step('takeoff', {}), Step('land', {})],
              altitude_frame='AGL').validate()


def test_yaw_and_scan_validation():
    spec = _spec([Step('takeoff', {}),
                  Step('yaw', {'degrees': 90.0, 'rate_deg_s': 20.0}),
                  Step('scan', {'seconds': 10.0, 'rate_deg_s': 15.0}),
                  Step('land', {})]).validate()
    assert [step.type for step in spec.steps] == ['takeoff', 'yaw', 'scan', 'land']


def test_yaw_zero_degrees_rejected():
    with pytest.raises(TaskError, match='yaw.degrees'):
        _spec([Step('takeoff', {}), Step('yaw', {'degrees': 0.0}),
               Step('land', {})]).validate()


def test_scan_seconds_range():
    with pytest.raises(TaskError, match='scan.seconds'):
        _spec([Step('takeoff', {}), Step('scan', {'seconds': 0.0}),
               Step('land', {})]).validate()


def test_rectangle_search_validation_and_geofence():
    spec = _spec([Step('takeoff', {}),
                  Step('search', {'shape': 'rectangle',
                                  'center_east_m': 5.0, 'center_north_m': -6.0,
                                  'width_m': 8.0, 'height_m': 6.0,
                                  'spacing_m': 3.0,
                                  'target': {'kind': 'any'}}),
                  Step('land', {})]).validate()
    assert spec.steps[1].params['shape'] == 'rectangle'

    with pytest.raises(TaskError, match='search.shape'):
        _spec([Step('takeoff', {}),
               Step('search', {'shape': 'triangle',
                               'target': {'kind': 'any'}}),
               Step('land', {})]).validate()

    with pytest.raises(TaskError, match='围栏'):
        _spec([Step('takeoff', {}),
               Step('search', {'shape': 'rectangle',
                               'center_east_m': 35.0, 'center_north_m': 0.0,
                               'width_m': 20.0, 'height_m': 20.0,
                               'spacing_m': 3.0,
                               'target': {'kind': 'any'}}),
               Step('land', {})]).validate()


def test_polygon_search_validation():
    spec = _spec([Step('takeoff', {}),
                  Step('search', {'shape': 'polygon',
                                  'center_east_m': 5.0, 'center_north_m': -6.0,
                                  'polygon': [[-3, -3], [3, -3], [3, -1],
                                              [-1, -1], [-1, 3], [-3, 3]],
                                  'spacing_m': 2.0,
                                  'target': {'kind': 'any'}}),
                  Step('land', {})]).validate()
    assert '多边形 6 顶点' in spec.summary()

    with pytest.raises(TaskError, match='至少 3 个顶点'):
        _spec([Step('takeoff', {}),
               Step('search', {'shape': 'polygon', 'polygon': [[0, 0], [1, 1]],
                               'target': {'kind': 'any'}}),
               Step('land', {})]).validate()

    with pytest.raises(TaskError, match='围栏'):
        _spec([Step('takeoff', {}),
               Step('search', {'shape': 'polygon',
                               'center_east_m': 5.0, 'center_north_m': 0.0,
                               'polygon': [[40, 0], [42, 0], [42, 2], [40, 2]],
                               'target': {'kind': 'any'}}),
               Step('land', {})]).validate()
