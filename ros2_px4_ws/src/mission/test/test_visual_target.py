"""通用视觉目标 standoff 单测。"""

import pytest

from mission.visual_target import apply_standoff, median_enu_target


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
