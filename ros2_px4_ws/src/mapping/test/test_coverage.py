"""观测覆盖网格单测。"""

import numpy as np
import pytest

from mapping.coverage import (CoverageGrid, point_in_polygon, polygon_area,
                              polygon_bounds)


SQUARE = [(-5, -5), (5, -5), (5, 5), (-5, 5)]
L_SHAPE = [(0, 0), (10, 0), (10, 4), (4, 4), (4, 10), (0, 10)]


def test_point_in_polygon_square_and_l_shape():
    assert point_in_polygon(0, 0, SQUARE)
    assert not point_in_polygon(6, 0, SQUARE)
    assert point_in_polygon(2, 2, L_SHAPE)
    assert point_in_polygon(8, 2, L_SHAPE)
    assert not point_in_polygon(8, 8, L_SHAPE)   # L 的缺口


def test_polygon_area_and_bounds():
    assert polygon_area(SQUARE) == pytest.approx(100.0)
    assert polygon_area(L_SHAPE) == pytest.approx(64.0)
    assert polygon_bounds(SQUARE) == (-5, -5, 5, 5)


def test_coverage_marks_only_ground_cells_inside_polygon():
    grid = CoverageGrid(SQUARE, cell_size_m=1.0)
    assert grid.status()['total_cells'] == 100
    grid.observe(np.array([[0.5, 0.5, 0.0],
                           [1.5, 0.5, 0.0],
                           [0.5, 0.5, 1.0],     # 非地面，忽略
                           [50.0, 50.0, 0.0]]))  # 多边形外，忽略
    status = grid.status()
    assert status['observed_cells'] == 2
    assert status['coverage_ratio'] == pytest.approx(0.02)
    assert status['missed_cells'] == 98
    assert status['points_total'] == 4
    assert status['ground_points'] == 3
    assert status['points_in_polygon'] == 2


def test_coverage_reset():
    grid = CoverageGrid(SQUARE, cell_size_m=1.0)
    grid.observe(np.array([[0.5, 0.5, 0.0]]))
    assert grid.status()['observed_cells'] == 1
    grid.reset(L_SHAPE)
    assert grid.status()['observed_cells'] == 0
    assert grid.status()['total_cells'] < 100


def test_invalid_polygon_rejected():
    with pytest.raises(ValueError):
        CoverageGrid([(0, 0), (1, 1)])
