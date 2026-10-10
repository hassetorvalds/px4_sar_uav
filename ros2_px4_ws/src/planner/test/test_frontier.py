"""Frontier 探索纯逻辑的单元测试。"""

import numpy as np

from planner.frontier import FrontierMap


def _map_with_free_around(origin=(0.0, 0.0), free_radius=3.0, resolution=0.5):
    grid = FrontierMap(resolution=resolution, free_radius=free_radius)
    grid.mark_free_around(origin)
    return grid


def test_no_frontier_before_any_free_cells():
    grid = FrontierMap(resolution=0.5, free_radius=3.0)
    assert grid.best_frontier((0.0, 0.0, 0.0)) is None


def test_free_region_creates_frontier_ring():
    grid = _map_with_free_around()
    clusters = grid.clusters()
    assert clusters, '已知可通行区域边界应形成 frontier'
    # 目标取簇内离机体最近的格子，应落在 free 半径（3 m）附近，而不是原点
    chosen = grid.best_frontier((0.0, 0.0, 0.0), max_distance=10.0)
    assert chosen is not None
    assert 2.0 < math_dist(chosen, (0.0, 0.0)) <= 3.5


def math_dist(a, b):
    return ((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2) ** 0.5


def test_occupied_cells_are_not_frontiers():
    grid = _map_with_free_around()
    # 在正东 2 m 处放一圈障碍，将其所在的 free 格子转为 occupied
    obstacle = np.array([[2.0, 0.0, 0.0], [2.0, 0.5, 0.0], [2.0, -0.5, 0.0]])
    grid.mark_occupied(obstacle)
    cells = grid.frontier_cells()
    assert grid.key(2.0, 0.0) not in cells


def test_best_frontier_prefers_nearest_visible():
    grid = _map_with_free_around(origin=(0.0, 0.0), free_radius=3.0)
    grid.mark_free_around((8.0, 0.0))       # 远处也有一片已知区域
    chosen = grid.best_frontier((0.0, 0.0, 0.0), max_distance=20.0)
    assert chosen is not None
    # 应偏向近处那片区域
    assert math_dist(chosen, (0.0, 0.0)) < math_dist(chosen, (8.0, 0.0))


def test_blocked_frontier_is_skipped():
    grid = _map_with_free_around(origin=(0.0, 0.0), free_radius=3.0)
    grid.mark_free_around((8.0, 0.0))
    # 在两片区域之间竖起一道栅格墙
    wall = np.array([[4.0, y * 0.5, 0.0] for y in range(-6, 7)])
    grid.mark_occupied(wall)
    chosen = grid.best_frontier((0.0, 0.0, 0.0), max_distance=20.0)
    assert chosen is not None
    # 被墙挡住的方向（正东远处）不应被选中
    assert not (chosen[0] > 5.0 and abs(chosen[1]) < 1.0)


def test_max_distance_filter():
    grid = _map_with_free_around(origin=(0.0, 0.0), free_radius=3.0)
    assert grid.best_frontier((0.0, 0.0, 0.0), max_distance=1.0) is None


def test_visited_goals_are_not_reselected():
    grid = _map_with_free_around(origin=(0.0, 0.0), free_radius=3.0)
    first = grid.best_frontier((0.0, 0.0, 0.0), max_distance=10.0)
    assert first is not None
    second = grid.best_frontier((0.0, 0.0, 0.0), max_distance=10.0,
                                visited=[first], visited_radius=1.0)
    assert second is None or math_dist(second, first) > 1.0
