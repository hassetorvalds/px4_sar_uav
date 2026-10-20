"""多区域最近邻排序单测。"""

from mission.regions import order_search_steps, region_center
from mission.task_spec import Step, TaskSpec


def _search(center):
    return Step('search', {'center_east_m': center[0], 'center_north_m': center[1],
                           'radius_m': 3.0, 'spacing_m': 2.0,
                           'target': {'kind': 'any'}})


def test_region_center():
    assert region_center(_search((3.0, -4.0))) == (3.0, -4.0)


def test_nearest_neighbor_order_and_preserves_other_steps():
    spec = TaskSpec(steps=[
        Step('takeoff', {}),
        _search((10.0, 0.0)),
        _search((0.0, 10.0)),
        _search((5.0, 5.0)),
        Step('land', {}),
    ])
    order_search_steps(spec, start=(0.0, 0.0))
    centers = [(step.params['center_east_m'], step.params['center_north_m'])
               for step in spec.steps if step.type == 'search']
    assert centers == [(5.0, 5.0), (10.0, 0.0), (0.0, 10.0)]
    assert spec.steps[0].type == 'takeoff'
    assert spec.steps[-1].type == 'land'
    assert any('最近邻排序' in note for note in spec.notes)


def test_single_search_is_unchanged():
    spec = TaskSpec(steps=[Step('takeoff', {}), _search((5.0, -6.0)), Step('land', {})])
    notes_before = list(spec.notes)
    order_search_steps(spec)
    assert spec.steps[1].params['center_east_m'] == 5.0
    assert spec.notes == notes_before
