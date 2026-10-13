"""地图安全层（C3）的单元测试：fail-safe、左右方向、迟滞、压速曲线。"""

import math

import pytest

from vlm.pointing import BodyVelocity
from vlm.safety import MapSafetyLimits, apply_map_safety, parse_clearance_status

LIMITS = MapSafetyLimits()


def _command(forward=0.12, left=0.0, yaw_rate=0.0):
    return BodyVelocity(forward, left, 0.0, yaw_rate, 1.0)


def test_parse_clearance_status_from_local_map_text():
    text = ('voxels=1956 frames=12 nearest_forward_m=0.42 left_m=3.00 right_m=2.10')
    assert parse_clearance_status(text) == pytest.approx((0.42, 3.00, 2.10))
    assert parse_clearance_status('garbage') is None
    assert parse_clearance_status('') is None


def test_clear_forward_passes_through():
    command, guard, holding, _ = apply_map_safety(
        _command(), (3.0, 3.0, 3.0), 0.1, False, LIMITS)
    assert guard == 'ok'
    assert command.forward == pytest.approx(0.12)
    assert holding is False


def test_slow_zone_scales_final_forward_speed():
    """0.9 m 处于 1.2→0.6 的压速段：scale=(0.9-0.6)/0.6=0.5。"""
    command, guard, _, detail = apply_map_safety(
        _command(forward=0.12), (0.9, 2.0, 2.0), 0.1, False, LIMITS)
    assert guard == 'obstacle_slow'
    assert command.forward == pytest.approx(0.06)
    assert detail['scale'] == pytest.approx(0.5)


def test_obstacle_hold_zeroes_translation_and_overrides_yaw_left():
    """左更宽且明显开阔 → 左转（yaw>0）+ 小前向绕行（不原地打转）。"""
    command, guard, holding, detail = apply_map_safety(
        _command(yaw_rate=0.5), (0.4, 3.0, 1.0), 0.1, False, LIMITS,
        stop_streak=LIMITS.stop_confirm_frames - 1)
    assert guard == 'obstacle'
    assert holding is True
    assert command.left == pytest.approx(0.0)
    assert command.forward == pytest.approx(LIMITS.creep_mps)         # 宽侧绕行
    assert command.yaw_rate == pytest.approx(LIMITS.avoid_yaw_rate)   # 覆盖 VLM 的 0.5
    assert detail['avoid_side'] == 'left'
    assert detail['yaw_capped'] is False


def test_obstacle_hold_zeroes_forward_when_both_sides_tight():
    """两侧都紧 → 完全归零保持（不前进）。"""
    command, _, _, _ = apply_map_safety(
        _command(), (0.4, 0.5, 0.5), 0.1, False, LIMITS,
        stop_streak=LIMITS.stop_confirm_frames - 1)
    assert command.forward == pytest.approx(0.0)


def test_turn_limit_stops_rotation_to_avoid_dead_end_circling():
    """累积转角到上限后停止继续转（服务器侧 2026-10-12 建议）。"""
    command, guard, _, detail = apply_map_safety(
        _command(), (0.4, 3.0, 1.0), 0.1, False, LIMITS, turn_deg=61.0,
        stop_streak=LIMITS.stop_confirm_frames - 1)
    assert guard == 'obstacle'
    assert command.yaw_rate == pytest.approx(0.0)
    assert detail['yaw_capped'] is True


def test_obstacle_hold_turns_right_when_right_is_wider():
    command, _, _, detail = apply_map_safety(
        _command(), (0.4, 1.0, 3.0), 0.1, False, LIMITS,
        stop_streak=LIMITS.stop_confirm_frames - 1)
    assert command.yaw_rate == pytest.approx(-LIMITS.avoid_yaw_rate)
    assert detail['avoid_side'] == 'right'


def test_single_frame_dip_does_not_enter_hold():
    """单帧近读数（伪点）不该立刻进"保持+转向"——先按慢速区把前向压到 0。

    这是服务器侧 2026-10-13 的建议：用**时间维连续确认**代替"按距离硬切"。
    """
    command, guard, holding, detail = apply_map_safety(
        _command(forward=0.12), (0.4, 3.0, 3.0), 0.1, False, LIMITS, stop_streak=0)
    assert guard == 'obstacle_slow'
    assert holding is False
    assert command.forward == pytest.approx(0.0)      # 压速把前向压到 0
    assert command.yaw_rate == pytest.approx(0.0)     # 但不转向
    assert detail['stop_streak'] == 1

    # 连续第 3 帧才进入保持
    _, guard3, holding3, _ = apply_map_safety(
        _command(forward=0.12), (0.4, 3.0, 3.0), 0.1, False, LIMITS, stop_streak=2)
    assert guard3 == 'obstacle'
    assert holding3 is True


def test_hysteresis_keeps_holding_until_release_threshold():
    # 已保持中：前向 0.8 m < 0.9 m → 继续保持
    _, guard, holding, _ = apply_map_safety(
        _command(), (0.8, 2.0, 2.0), 0.1, True, LIMITS)
    assert guard == 'obstacle'
    assert holding is True
    # 前向 0.95 m 但**所选侧只有 1.0 m < side_release 1.2 m** → 仍保持（迟滞更严）
    _, guard, holding, _ = apply_map_safety(
        _command(), (0.95, 1.0, 1.0), 0.1, True, LIMITS)
    assert holding is True
    # 前向 0.95 m 且侧向 2.0 m ≥ 1.2 m → 解除保持
    command, guard, holding, _ = apply_map_safety(
        _command(), (0.95, 2.0, 2.0), 0.1, True, LIMITS)
    assert holding is False
    assert guard == 'obstacle_slow'          # 0.95 仍在压速段
    assert command.forward < 0.12


def test_stale_clearance_is_fail_safe_not_fail_open():
    """数据陈旧 → 视为未知，前向压到很低（服务器侧 2026-10-12 明确要求 fail-safe）。"""
    command, guard, _, detail = apply_map_safety(
        _command(forward=0.5), (3.0, 3.0, 3.0), 2.0, False, LIMITS)
    assert guard == 'stale'
    assert command.forward == pytest.approx(LIMITS.stale_forward_limit_mps)
    assert detail['reason'] == 'clearance_unknown'


def test_not_stale_within_three_map_periods():
    """地图 2.25 Hz（0.44 s）时，1.0 s 的年龄不该判陈旧——否则会 ok/stale 逐帧闪烁。"""
    command, guard, _, _ = apply_map_safety(
        _command(forward=0.12), (3.0, 3.0, 3.0), 1.0, False, LIMITS)
    assert guard == 'ok'
    assert command.forward == pytest.approx(0.12)


def test_missing_clearance_is_treated_as_stale():
    command, guard, _, _ = apply_map_safety(_command(forward=0.5), None, None, False, LIMITS)
    assert guard == 'stale'
    assert command.forward == pytest.approx(LIMITS.stale_forward_limit_mps)


def test_disabled_map_layer_does_not_touch_command():
    limits = MapSafetyLimits(enabled=False)
    command, guard, _, _ = apply_map_safety(
        _command(forward=0.5), (0.1, 0.1, 0.1), 99.0, False, limits)
    assert guard == 'map_off'
    assert command.forward == pytest.approx(0.5)
