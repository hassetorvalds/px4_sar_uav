"""搜索目标接近几何测试。"""

import pytest

from mission.search_acquire import acquire_goal


def test_acquire_goal_applies_standoff():
    east, north, standoff = acquire_goal((0.0, 0.0), (10.0, 0.0), standoff_m=3.0)
    assert east == pytest.approx(7.0)
    assert north == pytest.approx(0.0)
    assert standoff == pytest.approx(3.0)


def test_acquire_goal_shrinks_near_target():
    east, north, standoff = acquire_goal((0.0, 0.0), (1.0, 0.0), standoff_m=3.0)
    assert east == pytest.approx(0.5)
    assert standoff == pytest.approx(0.5)


def test_acquire_goal_rejects_coincident():
    with pytest.raises(ValueError):
        acquire_goal((1.0, 1.0), (1.0, 1.0))
