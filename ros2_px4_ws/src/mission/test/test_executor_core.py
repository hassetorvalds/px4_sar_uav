"""执行核心纯逻辑单测。"""

import math

import pytest

from mission.executor_core import (VelocityDecision, apply_clearance_gate,
                                   forward_target, inside_geofence,
                                   search_waypoints, step_timeout_s,
                                   velocity_toward)
from mission.task_spec import Step


def test_velocity_toward_reaches_target():
    decision = velocity_toward((0, 0, 2), (0, 5, 2), max_speed_mps=0.5,
                               tolerance_m=0.4)
    assert decision.done is False
    assert decision.east == pytest.approx(0.0)
    assert decision.north == pytest.approx(0.5)


def test_velocity_toward_done_within_tolerance():
    decision = velocity_toward((0, 0, 2), (0.1, 0.1, 2), max_speed_mps=0.5)
    assert decision.done is True
    assert decision.east == 0 and decision.north == 0


def test_velocity_toward_caps_speed():
    decision = velocity_toward((0, 0, 2), (20, 0, 2), max_speed_mps=0.5, gain=1.0)
    assert math.hypot(decision.east, decision.north) == pytest.approx(0.5)


def test_clearance_gate_zeroes_forward_component():
    decision = VelocityDecision(east=0.5, north=0.0, done=False)
    gated = apply_clearance_gate(decision, (0, 0, 2), heading_enu=0.0,
                                 nearest_forward_m=0.5, stop_m=0.6)
    assert gated.blocked is True
    assert gated.east == pytest.approx(0.0)
    assert gated.north == pytest.approx(0.0)


def test_clearance_gate_keeps_lateral():
    decision = VelocityDecision(east=0.3, north=0.4, done=False)
    gated = apply_clearance_gate(decision, (0, 0, 2), heading_enu=0.0,
                                 nearest_forward_m=0.5, stop_m=0.6)
    # 前向分量 = east = 0.3，被去掉；north 作为侧向保留
    assert gated.east == pytest.approx(0.0)
    assert gated.north == pytest.approx(0.4)


def test_clearance_gate_ignores_retreat():
    decision = VelocityDecision(east=-0.3, north=0.0, done=False)
    gated = apply_clearance_gate(decision, (0, 0, 2), heading_enu=0.0,
                                 nearest_forward_m=0.5)
    assert gated.blocked is False
    assert gated.east == pytest.approx(-0.3)


def test_clearance_gate_ignores_unknown_clearance():
    decision = VelocityDecision(east=0.5, north=0.0, done=False)
    gated = apply_clearance_gate(decision, (0, 0, 2), heading_enu=0.0,
                                 nearest_forward_m=None)
    assert gated.blocked is False
    assert gated.east == pytest.approx(0.5)


def test_geofence():
    assert inside_geofence((10, 0, 2), (0, 0, 0), 10.0)
    assert not inside_geofence((11, 0, 2), (0, 0, 0), 10.0)
    assert inside_geofence((11, 0, 2), (0, 0, 0), 10.0, margin_m=1.0)


def test_forward_target_east_and_north():
    east = forward_target((0, 0, 2), 0.0, 5.0)
    assert east == pytest.approx((5.0, 0.0, 2.0))
    north = forward_target((0, 0, 2), math.pi / 2, 5.0)
    assert north == pytest.approx((0.0, 5.0, 2.0))


def test_search_waypoints_altitude_and_count():
    waypoints = search_waypoints({'center_east_m': 10.0, 'center_north_m': 20.0,
                                  'radius_m': 5.0, 'spacing_m': 5.0,
                                  'altitude_m': 2.5}, altitude_m=2.0)
    assert all(p[2] == pytest.approx(2.5) for p in waypoints)
    assert min(p[0] for p in waypoints) == pytest.approx(5.0)
    assert max(p[0] for p in waypoints) == pytest.approx(15.0)


def test_step_timeout_is_positive_and_scales():
    assert step_timeout_s(Step('takeoff', {}), speed_mps=0.5) >= 15.0
    short = step_timeout_s(Step('goto', {'east_m': 5.0}), speed_mps=0.5)
    far = step_timeout_s(Step('goto', {'east_m': 30.0}), speed_mps=0.5)
    assert far > short
    assert step_timeout_s(Step('hover', {'seconds': 5.0}), speed_mps=0.5) >= 15.0


def test_yaw_rate_sign_and_done():
    from mission.executor_core import yaw_rate_to_target

    rate, done = yaw_rate_to_target(0.0, math.pi / 2, max_rate_rad_s=1.0)
    assert rate > 0 and not done
    rate, done = yaw_rate_to_target(0.0, -math.pi / 2, max_rate_rad_s=1.0)
    assert rate < 0 and not done
    rate, done = yaw_rate_to_target(0.0, 0.01, max_rate_rad_s=1.0,
                                    tolerance_rad=0.05)
    assert done and rate == 0.0
    rate, done = yaw_rate_to_target(math.radians(350), math.radians(10),
                                    max_rate_rad_s=1.0)
    assert rate > 0 and not done
