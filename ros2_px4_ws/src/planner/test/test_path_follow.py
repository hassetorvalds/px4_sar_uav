"""Path 跟随纯逻辑测试。"""

import pytest

from planner.path_follow import follow_velocity, next_waypoint


def test_next_waypoint_skips_reached():
    path = [(1.0, 0.0), (2.0, 0.0), (3.0, 0.0)]
    point, index = next_waypoint((1.1, 0.0), path, tolerance_m=0.4,
                                 lookahead_m=0.5)
    assert point == (2.0, 0.0)
    assert index == 1


def test_follow_velocity_clamps_speed():
    command = follow_velocity((0.0, 0.0), [(2.0, 0.0)],
                              max_speed_mps=0.5, gain=1.0)
    assert command.east == pytest.approx(0.5)
    assert command.north == pytest.approx(0.0)
    assert command.done is False


def test_follow_done_at_path_end():
    command = follow_velocity((2.0, 0.0), [(2.0, 0.0)], tolerance_m=0.4)
    assert command.done is True
    assert command.east == 0.0 and command.north == 0.0
