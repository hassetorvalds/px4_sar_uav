"""速度仲裁的单元测试（纯函数，不依赖 ROS 运行时）。"""

import pytest

from px4_interface.velocity_arbiter import (
    DEFAULT_TIMEOUTS,
    ZERO_COMMAND,
    select_command,
)


def _fresh(velocity, yaw=0.0, now=100.0):
    return (velocity, yaw, now)


def test_avoid_wins_when_both_fresh():
    """两条回路都新鲜时，几何避障（avoid）优先于语义导航（vlm）。"""
    now = 100.0
    sources = {
        'avoid': _fresh((0.0, -0.5, 0.0), now=now),
        'vlm': _fresh((1.0, 0.0, 0.0), now=now),
    }
    name, velocity, yaw = select_command(
        sources, now, priority=('avoid', 'vlm'))
    assert name == 'avoid'
    assert velocity == (0.0, -0.5, 0.0)


def test_falls_through_to_vlm_when_avoid_stale():
    """几何回路过期时，退到语义回路，而不是输出过期指令。"""
    now = 100.0
    sources = {
        'avoid': _fresh((0.0, -0.5, 0.0), now=now - 5.0),
        'vlm': _fresh((1.0, 0.0, 0.0), now=now),
    }
    name, velocity, _ = select_command(sources, now, priority=('avoid', 'vlm'))
    assert name == 'vlm'
    assert velocity == (1.0, 0.0, 0.0)


def test_all_stale_selects_nothing():
    """全部过期 → 不选任何来源（交由 bridge 的速度超时保持位置）。"""
    now = 100.0
    sources = {
        'avoid': _fresh((0.0, -0.5, 0.0), now=now - 5.0),
        'vlm': _fresh((1.0, 0.0, 0.0), now=now - 10.0),
    }
    name, velocity, yaw = select_command(sources, now, priority=('avoid', 'vlm'))
    assert name is None
    assert velocity == ZERO_COMMAND[:3]
    assert yaw == 0.0


def test_priority_order_is_honored_on_tie():
    """优先级顺序由 priority 参数决定：反转顺序时 vlm 优先。"""
    now = 100.0
    sources = {
        'avoid': _fresh((0.0, -0.5, 0.0), now=now),
        'vlm': _fresh((1.0, 0.0, 0.0), now=now),
    }
    name, velocity, _ = select_command(sources, now, priority=('vlm', 'avoid'))
    assert name == 'vlm'
    assert velocity == (1.0, 0.0, 0.0)


def test_missing_source_is_skipped():
    """从未发布过的来源（None）不阻塞后面的来源。"""
    now = 100.0
    sources = {'vlm': _fresh((1.0, 0.0, 0.0), now=now)}
    name, velocity, _ = select_command(sources, now, priority=('avoid', 'vlm'))
    assert name == 'vlm'
    assert velocity == (1.0, 0.0, 0.0)


@pytest.mark.parametrize('age,expected', [
    (DEFAULT_TIMEOUTS['avoid'] - 0.01, 'avoid'),   # 刚好新鲜
    (DEFAULT_TIMEOUTS['avoid'] + 0.01, None),      # 刚好过期
])
def test_timeout_boundary(age, expected):
    """超时边界：``now - last <= timeout`` 视为新鲜。"""
    now = 100.0
    sources = {'avoid': _fresh((0.0, 0.0, 0.0), now=now - age)}
    name, _, _ = select_command(sources, now, priority=('avoid',))
    assert name == expected


def test_custom_timeout_overrides_default():
    """可对单个来源覆盖超时。"""
    now = 100.0
    sources = {'avoid': _fresh((0.0, 0.0, 0.0), now=now - 2.0)}
    assert select_command(sources, now, priority=('avoid',))[0] is None
    assert select_command(
        sources, now, priority=('avoid',),
        timeouts={'avoid': 3.0})[0] == 'avoid'
