"""VLM TaskSpec→A*目标测试。"""

import pytest

from planner.visual_goal import goal_from_task


def test_goal_from_relative_goto():
    task = {'coordinate_frame': 'relative', 'steps': [
        {'type': 'takeoff', 'params': {}},
        {'type': 'goto', 'params': {'east_m': 3.0, 'north_m': -2.0}},
    ]}
    assert goal_from_task(task, (10.0, 20.0)) == (13.0, 18.0)


def test_goal_from_map_goto():
    task = {'coordinate_frame': 'map', 'steps': [
        {'type': 'goto', 'params': {'east_m': 3.0, 'north_m': 4.0}},
    ]}
    assert goal_from_task(task, (10.0, 20.0)) == (3.0, 4.0)


def test_goal_missing_goto():
    with pytest.raises(ValueError):
        goal_from_task({'steps': []}, (0.0, 0.0))
