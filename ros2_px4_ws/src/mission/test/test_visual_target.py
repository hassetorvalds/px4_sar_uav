"""通用视觉目标 standoff 单测。"""

import pytest

from mission.visual_target import (apply_standoff, bbox_touches_edge,
                                      median_enu_target, select_verified_target)


def test_standoff_reduces_along_line():
    east, north, dist = apply_standoff(30.0, 40.0, 5.0)
    assert dist == pytest.approx(50.0)
    assert east == pytest.approx(27.0)
    assert north == pytest.approx(36.0)


def test_standoff_too_close():
    with pytest.raises(ValueError):
        apply_standoff(1.0, 1.0, 2.0)


def test_median_target_accepts_small_spread():
    east, north, spread = median_enu_target([(10.0, 20.0), (10.4, 20.2),
                                            (9.8, 20.1)], max_spread_m=1.0)
    assert east == pytest.approx(10.0)
    assert north == pytest.approx(20.1)
    assert spread <= 1.0


def test_median_target_rejects_bad_spread():
    with pytest.raises(ValueError):
        median_enu_target([(10.0, 20.0), (20.0, 20.0)], max_spread_m=1.5)


def test_select_consistent_primary():
    east, north, diff, source = select_verified_target((10, 5), (10.5, 5.2), 3.0)
    assert (east, north) == (10.0, 5.0)
    assert diff == pytest.approx(0.5385, abs=1e-3)
    assert source == 'primary'


def test_select_rejects_disagreement():
    with pytest.raises(ValueError):
        select_verified_target((10, 5), (20, 5), 3.0)


def test_select_falls_back_to_verify():
    east, north, diff, source = select_verified_target(None, (21.2, 5.5), 3.0)
    assert (east, north) == (21.2, 5.5)
    assert diff is None and source == 'verify'


def test_bbox_touches_edge_detects_edges():
    assert bbox_touches_edge([0, 100, 200, 300], 1280, 720, 5)
    assert bbox_touches_edge([1200, 100, 1280, 300], 1280, 720, 5)
    assert not bbox_touches_edge([600, 290, 670, 360], 1280, 720, 5)
