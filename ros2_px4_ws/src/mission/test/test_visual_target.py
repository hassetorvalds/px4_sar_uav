"""通用视觉目标 standoff 单测。"""

import pytest

from mission.visual_target import apply_standoff


def test_standoff_reduces_along_line():
    east, north, dist = apply_standoff(30.0, 40.0, 5.0)
    assert dist == pytest.approx(50.0)
    assert east == pytest.approx(27.0)
    assert north == pytest.approx(36.0)


def test_standoff_too_close():
    with pytest.raises(ValueError):
        apply_standoff(1.0, 1.0, 2.0)
