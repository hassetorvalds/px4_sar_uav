"""避障决策纯逻辑的单元测试。"""

import math

from planner.avoidance import plan_velocity


def test_arrived_holds_position():
    cmd = plan_velocity((0.0, 0.0, 3.0), (0.1, 0.1, 3.0), clearance_m=5.0)
    assert cmd.mode == 'arrived'
    assert cmd.velocity_enu[0] == 0.0 and cmd.velocity_enu[1] == 0.0


def test_cruise_toward_goal_when_clear():
    cmd = plan_velocity((5.0, 0.0, 3.0), (0.0, 0.0, 3.0), clearance_m=6.0,
                        cruise_speed=1.0)
    assert cmd.mode == 'cruise'
    assert cmd.velocity_enu[0] == 1.0
    assert abs(cmd.velocity_enu[1]) < 1e-9


def test_slow_reduces_speed_near_obstacle():
    near = plan_velocity((5.0, 0.0, 3.0), (0.0, 0.0, 3.0), clearance_m=1.4,
                         slow_distance=2.0, stop_distance=0.8, cruise_speed=1.0)
    assert near.mode == 'slow'
    assert 0.0 < near.velocity_enu[0] < 1.0


def test_avoid_strafes_when_blocked():
    cmd = plan_velocity((5.0, 0.0, 3.0), (0.0, 0.0, 3.0), clearance_m=0.5,
                        stop_distance=0.8, cruise_speed=1.0, avoid_side=-1)
    assert cmd.mode == 'avoid'
    # 目标在正东，向右（顺时针）侧移应为正南
    assert abs(cmd.velocity_enu[0]) < 1e-9
    assert cmd.velocity_enu[1] < 0.0


def test_avoid_side_switches_direction():
    left = plan_velocity((5.0, 0.0, 3.0), (0.0, 0.0, 3.0), clearance_m=0.5,
                         avoid_side=+1)
    right = plan_velocity((5.0, 0.0, 3.0), (0.0, 0.0, 3.0), clearance_m=0.5,
                          avoid_side=-1)
    assert left.velocity_enu[1] > 0.0 > right.velocity_enu[1]


def test_altitude_correction_is_proportional_and_clamped():
    cmd = plan_velocity((0.0, 0.0, 10.0), (0.0, 0.0, 0.0), clearance_m=5.0,
                        altitude_gain=0.5)
    assert cmd.velocity_enu[2] == 1.0      # 限制在 ±1 m/s


def test_goal_distance_reported():
    cmd = plan_velocity((3.0, 4.0, 3.0), (0.0, 0.0, 3.0), clearance_m=9.0)
    assert math.isclose(cmd.distance_to_goal, 5.0, rel_tol=1e-9)
