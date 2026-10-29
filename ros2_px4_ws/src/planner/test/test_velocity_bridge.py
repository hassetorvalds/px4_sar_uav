"""A* candidate→offboard 桥的安全门测试。"""

from planner.astar_velocity_bridge import bridge_allowed


def _ok(**overrides):
    values = dict(enabled=True, armed=True, offboard=True, failsafe=False,
                  age_s=0.1, timeout_s=0.5, speed_mps=0.2,
                  max_speed_mps=0.3, active_s=1.0, max_active_s=10.0)
    values.update(overrides)
    return bridge_allowed(**values)


def test_bridge_allows_safe_candidate():
    assert _ok() == (True, 'ok')


def test_bridge_refuses_disabled_or_stale():
    assert _ok(enabled=False)[1] == 'disabled'
    assert _ok(age_s=1.0)[1] == 'candidate_stale'


def test_bridge_refuses_speed_and_duration():
    assert _ok(speed_mps=0.5)[1] == 'speed_limit'
    assert _ok(active_s=11.0)[1] == 'max_duration'
