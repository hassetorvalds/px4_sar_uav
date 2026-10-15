"""地图安全层（C3）的单元测试：fail-safe、左右方向、迟滞、压速曲线。"""

import math

import pytest

from vlm.pointing import BodyVelocity
from vlm.safety import (
    MapSafetyLimits,
    ObstacleTrack,
    apply_map_safety,
    parse_clearance_status,
)

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
    # 注意：净空取 0.55 m（> blind_zone_m 0.5）——盲区内前向会被强制归零，那是另一条规则
    # （见 test_blind_zone_zeroes_forward_but_keeps_yaw）
    command, guard, holding, detail = apply_map_safety(
        _command(yaw_rate=0.5), (0.55, 3.0, 1.0), 0.1, False, LIMITS,
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


def test_world_inconsistent_reading_does_not_accumulate_streak():
    """世界系里逐帧跳变的近场点（自运动拖影）不该累积成"保持"。

    服务器侧 2026-10-13 建议：真障碍在世界系稳定，拖影会跳——用世界系一致性做判据。
    """
    _, guard, holding, detail = apply_map_safety(
        _command(forward=0.12), (0.4, 3.0, 3.0), 0.1, False, LIMITS,
        stop_streak=2, world_consistent=False)
    assert holding is False
    assert guard == 'obstacle_slow'
    assert detail['stop_streak'] == 1        # 不累计

    _, guard2, holding2, detail2 = apply_map_safety(
        _command(forward=0.12), (0.4, 3.0, 3.0), 0.1, False, LIMITS,
        stop_streak=2, world_consistent=True)
    assert holding2 is True
    assert guard2 == 'obstacle'
    assert detail2['stop_streak'] == 3


def test_obstacle_track_rejects_jumping_points_and_keeps_stable_ones():
    """世界系 track：稳定的真障碍连续通过；逐帧跳变的拖影被拒。"""
    track = ObstacleTrack(size=3, tolerance_m=0.4)
    assert track.update((0.0, 0.0, 0.0)) is True        # 第一个点建 track
    assert track.update((0.05, 0.02, 0.0)) is True      # 与质心一致
    assert track.update((0.10, 0.03, 0.0)) is True      # 仍在容差内（质心随动）
    assert track.update((1.50, 0.0, 0.0)) is False      # 跳变 → 拒绝并重建
    assert track.update((1.55, 0.0, 0.0)) is True       # 新 track 建立后继续一致


def test_obstacle_track_centroid_tracks_recent_points_only():
    track = ObstacleTrack(size=2, tolerance_m=0.4)
    track.update((0.0, 0.0, 0.0))
    track.update((0.2, 0.0, 0.0))
    # 只保留最近 2 个点 → 质心为 (0.1, 0, 0)
    assert track.centroid[0] == pytest.approx(0.1)
    track.update((0.4, 0.0, 0.0))
    assert track.centroid[0] == pytest.approx(0.3)


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
    # 前向 0.95 m 且侧向 2.0 m ≥ 1.2 m → 满足释放条件，但**要连续 release_confirm_frames 帧**
    # （"进出双门限"，2026-10-15 加：d4 实测瞬释会造成"保持↔释放"来回切换）
    streak = 0
    for _ in range(LIMITS.release_confirm_frames - 1):
        _, guard, holding, detail = apply_map_safety(
            _command(), (0.95, 2.0, 2.0), 0.1, True, LIMITS, stop_streak=streak)
        assert holding is True
        assert guard == 'obstacle'
        streak = detail['stop_streak']
    command, guard, holding, _ = apply_map_safety(
        _command(), (0.95, 2.0, 2.0), 0.1, True, LIMITS, stop_streak=streak)
    assert holding is False
    assert guard == 'obstacle_slow'          # 0.95 仍在压速段
    assert command.forward < 0.12


def test_release_confirm_resets_on_interruption():
    """释放确认必须是**连续**的：中途回到保持条件就清零，不许"累计"。"""
    streak = 0
    for _ in range(LIMITS.release_confirm_frames - 1):
        _, _, holding, detail = apply_map_safety(
            _command(), (0.95, 2.0, 2.0), 0.1, True, LIMITS, stop_streak=streak)
        assert holding is True
        streak = detail['stop_streak']
    # 中断一帧（回到保持条件）
    _, guard, holding, detail = apply_map_safety(
        _command(), (0.7, 2.0, 2.0), 0.1, True, LIMITS, stop_streak=streak)
    assert holding is True
    streak = detail['stop_streak']
    # 再满足释放条件一次也不够（计数已清零）
    _, _, holding, detail = apply_map_safety(
        _command(), (0.95, 2.0, 2.0), 0.1, True, LIMITS, stop_streak=streak)
    assert holding is True
    assert detail['stop_streak'] == 1


def test_slow_forward_floor_only_amplifies_intended_forward():
    """压速带前向下限只放大"本来就想前进"的帧（服务器侧 2026-10-15 约束 1）。

    - VLM 显式停止（forward ≤ 0）→ **不加下限**；
    - 纯转向（forward=0 且 yaw≠0）→ **不加下限**；
    - forward > 0 且被压速压到很小 → 抬到下限，并落盘 creep_floor=True。
    """
    limits = MapSafetyLimits(slow_min_forward_mps=0.05)
    # 显式停止：前向 0 → 保持 0，且标记未施加
    cmd, guard, holding, detail = apply_map_safety(
        _command(forward=0.0), (0.7, 3.0, 3.0), 0.1, False, limits, stop_streak=0)
    assert guard == 'obstacle_slow'
    assert cmd.forward == pytest.approx(0.0)
    assert detail['creep_floor'] is False
    # 纯转向：前向 0、yaw 非 0 → 仍不加下限
    cmd, _, _, detail = apply_map_safety(
        _command(forward=0.0, yaw_rate=0.2), (0.7, 3.0, 3.0), 0.1, False, limits,
        stop_streak=0)
    assert cmd.forward == pytest.approx(0.0)
    assert detail['creep_floor'] is False
    # 想前进但被压速压小 → 抬到 0.05
    cmd, _, _, detail = apply_map_safety(
        _command(forward=0.12), (0.7, 3.0, 3.0), 0.1, False, limits, stop_streak=0)
    assert cmd.forward == pytest.approx(0.05)
    assert detail['creep_floor'] is True


def test_blind_zone_zeroes_forward_but_keeps_yaw():
    """盲区底线（2026-10-15）：净空 ≤ blind_zone_m 时前向一律归零，但**保留偏航**。

    动机：地图 min_range 过滤（0.45 m）会让净空读数触底——"看不见"不等于"没有障碍"，
    所以贴到盲区就绝不再前推（保持态的 creep 也要压掉），只继续朝宽侧转把视野打开。
    """
    limits = MapSafetyLimits(blind_zone_m=0.5, creep_mps=0.05, slow_min_forward_mps=0.05)
    # 保持态、左侧开阔（本来会给 creep=0.05）→ 盲区内必须归零，但偏航保留
    cmd, guard, holding, detail = apply_map_safety(
        BodyVelocity(0.05, 0.0, 0.0, 0.0, 1.0), (0.45, 3.0, 0.6), 0.1, True, limits,
        stop_streak=0)
    assert guard == 'obstacle' and holding is True
    assert cmd.forward == pytest.approx(0.0)
    assert abs(cmd.yaw_rate) > 0.0            # 仍在朝更宽一侧转
    assert detail['blind_zone'] is True
    # 压速带、本来想前进且会触发下限 → 盲区内同样归零
    cmd2, guard2, _, detail2 = apply_map_safety(
        BodyVelocity(0.12, 0.0, 0.0, 0.0, 1.0), (0.48, 3.0, 3.0), 0.1, False, limits,
        stop_streak=0)
    assert guard2 == 'obstacle_slow'
    assert cmd2.forward == pytest.approx(0.0)
    assert detail2['blind_zone'] is True
    assert detail2['creep_floor'] is False
    # 盲区外（0.7 m）→ 下限照常生效
    cmd3, _, _, detail3 = apply_map_safety(
        BodyVelocity(0.12, 0.0, 0.0, 0.0, 1.0), (0.7, 3.0, 3.0), 0.1, False, limits,
        stop_streak=0)
    assert cmd3.forward == pytest.approx(0.05)
    assert detail3['blind_zone'] is False


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
