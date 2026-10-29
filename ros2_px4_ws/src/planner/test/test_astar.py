"""A* 局部路径规划纯逻辑测试。"""

import pytest

from planner.astar import plan_path


def test_astar_routes_through_gap():
    wall = [(2.0, y) for y in [i * 0.2 for i in range(-30, 31)] if abs(y) > 1.0]
    path = plan_path(wall, (0.0, 0.0), (4.0, 0.0),
                     resolution=0.25, inflate_m=0.4, smooth=False)
    assert path is not None
    assert path[0] != path[-1]
    assert any(abs(p[0] - 2.0) < 0.3 for p in path)


def test_astar_rejects_full_wall():
    wall = [(2.0, y) for y in [i * 0.2 for i in range(-100, 101)]]
    assert plan_path(wall, (0.0, 0.0), (4.0, 0.0),
                     resolution=0.25, inflate_m=0.4, max_radius_m=5.0) is None


def test_astar_rejects_blocked_goal():
    assert plan_path([(4.0, 0.0)], (0.0, 0.0), (4.0, 0.0),
                     resolution=0.25, inflate_m=0.4) is None
