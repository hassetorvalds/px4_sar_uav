"""体素地图纯逻辑的单元测试（不需要 ROS 运行）。"""

import math

import numpy as np
import pytest

from mapping.voxel_grid import VoxelGrid, filter_by_min_range, yaw_rotate_flu_to_enu


def test_min_range_filter_drops_self_returns():
    """近场伪点（自身/起飞残点）必须被丢掉：地面静止实测读到 0.17 m。"""
    points = np.array([
        [0.17, 0.0, 0.0],      # 自身/近场伪点 → 丢
        [0.59, 0.1, 0.0],      # 仍小于阈值 → 丢
        [0.60, 0.0, 0.0],      # 恰好阈值 → 保留
        [3.0, 0.5, 0.2],       # 真实障碍 → 保留
    ])
    kept = filter_by_min_range(points, 0.6)
    assert kept.shape == (2, 3)
    assert np.allclose(kept[0], [0.60, 0.0, 0.0])
    assert np.allclose(kept[1], [3.0, 0.5, 0.2])


def test_min_range_filter_disabled_when_zero():
    points = np.array([[0.1, 0.0, 0.0], [2.0, 0.0, 0.0]])
    assert filter_by_min_range(points, 0.0).shape == (2, 3)


def test_ground_points_below_drone_are_excluded():
    """飞行高度下，机体下方 >0.25 m 的地面点不得算成"前方障碍"。

    2026-10-13 实测：地面静止/低空时 nearest_forward_m 持续 0.17~0.55 m，
    就是"光轴下方的地面"被当成障碍。竖直窗改成**下窄（0.25 m）上宽（0.6 m）**后，
    2 m 高度飞行时地面（低 2 m）被排除，只看到同高度的真实障碍。
    """
    grid = VoxelGrid(resolution=0.15, extent=8.0, height_min=-2.0, height_max=6.0,
                     decay_s=5.0)
    origin = (0.0, 0.0, 2.0)           # 2 m 高度飞行
    # 正前方 0.5 / 1.0 m 处的地面点（比机体低 2 m，旧的对称 ±0.6 m 窗也会排除，
    # 但低空时会漏进来；这里锁住"下窗更窄"的语义）
    ground = np.array([[0.5, 0.0, 0.0], [1.0, 0.0, 0.0]])
    # 正前方 2 m、与机体同高的真实障碍
    wall = np.array([[2.0, 0.0, 2.0]])
    grid.insert(np.vstack([ground, wall]), origin_enu=origin, now=0.0)

    forward, _, _ = grid.clearance_profile(
        origin, 0.0, half_angle_deg=30.0, max_distance=6.0,
        vertical_tolerance_m=0.6, vertical_down_tolerance_m=0.25)
    assert forward == pytest.approx(2.0, abs=0.2)     # 只看到墙，不是地面


def test_low_altitude_ground_within_down_window_is_still_seen():
    """诚实的边界：机体离地 <0.25 m 时，地面点仍落在下窗内，会被算作近障碍。

    这是"下窗 0.25 m"的已知代价；飞行中（≥0.3 m）不受影响，起飞阶段由爬升快速越过。
    """
    grid = VoxelGrid(resolution=0.15, extent=8.0, height_min=-2.0, height_max=6.0,
                     decay_s=5.0)
    origin = (0.0, 0.0, 0.15)
    grid.insert(np.array([[0.5, 0.0, 0.0]]), origin_enu=origin, now=0.0)
    forward, _, _ = grid.clearance_profile(
        origin, 0.0, half_angle_deg=30.0, max_distance=6.0,
        vertical_tolerance_m=0.6, vertical_down_tolerance_m=0.25)
    assert forward < 1.0


def test_rotate_identity_when_heading_east():
    points = np.array([[1.0, 0.0, 0.0], [0.0, 2.0, -1.0]])
    rotated = yaw_rotate_flu_to_enu(points, 0.0)
    assert np.allclose(rotated, points)


def test_rotate_ninety_degrees_turns_forward_to_north():
    # 机头朝北（ENU 偏航 +pi/2）时，机体前方 (1,0,0) 应指向北 (0,1,0)
    rotated = yaw_rotate_flu_to_enu(np.array([[1.0, 0.0, 0.5]]), math.pi / 2)
    assert np.allclose(rotated, [[0.0, 1.0, 0.5]], atol=1e-9)


def test_insert_quantizes_points_into_voxels():
    grid = VoxelGrid(resolution=0.5, extent=5.0)
    points = np.array([[0.1, 0.1, 0.1], [0.2, 0.2, 0.2], [1.1, 0.1, 0.1]])
    grid.insert(points, origin_enu=(0, 0, 0), now=10.0)
    # 前两个点落在同一个体素
    assert grid.voxel_count == 2


def test_insert_drops_points_outside_extent():
    grid = VoxelGrid(resolution=0.5, extent=2.0)
    grid.insert(np.array([[0.1, 0.1, 0.1], [5.0, 5.0, 5.0]]),
                origin_enu=(0, 0, 0), now=1.0)
    assert grid.voxel_count == 1


def test_decay_removes_stale_voxels():
    grid = VoxelGrid(resolution=0.5, extent=5.0, decay_s=2.0)
    grid.insert(np.array([[0.1, 0.1, 0.1]]), origin_enu=(0, 0, 0), now=0.0)
    assert grid.decay(now=1.0) == 0
    assert grid.decay(now=3.0) == 1
    assert grid.voxel_count == 0


def test_nearest_in_cone_finds_forward_obstacle():
    grid = VoxelGrid(resolution=0.5, extent=10.0)
    # 正前方 3 m 一处、正后方 2 m 一处
    grid.insert(np.array([[3.0, 0.0, 0.0], [-2.0, 0.0, 0.0]]),
                origin_enu=(0, 0, 0), now=0.0)
    distance = grid.nearest_in_cone((0, 0, 0), heading_enu=0.0, half_angle_deg=30.0)
    assert 2.5 <= distance <= 3.5      # 只应看到前方那个


def test_nearest_in_cone_ignores_side_obstacle():
    grid = VoxelGrid(resolution=0.5, extent=10.0)
    grid.insert(np.array([[0.0, 3.0, 0.0]]), origin_enu=(0, 0, 0), now=0.0)  # 正左方
    assert math.isinf(grid.nearest_in_cone((0, 0, 0), heading_enu=0.0, half_angle_deg=30.0))


def test_nearest_in_cone_respects_max_distance():
    grid = VoxelGrid(resolution=0.5, extent=20.0)
    grid.insert(np.array([[9.0, 0.0, 0.0]]), origin_enu=(0, 0, 0), now=0.0)
    assert math.isinf(grid.nearest_in_cone(
        (0, 0, 0), heading_enu=0.0, half_angle_deg=30.0, max_distance=5.0))


def test_ground_below_is_not_treated_as_forward_obstacle():
    """低空飞行时下方地面不应被判为前方障碍（历史 bug 的回归测试）。"""
    grid = VoxelGrid(resolution=0.15, extent=10.0)
    ground = np.array([[1.0, 0.0, -0.8], [2.0, 0.0, -0.8]])   # 机体在 z=0，地面在下方
    grid.insert(ground, origin_enu=(0, 0, 0), now=0.0)
    assert math.isinf(grid.nearest_in_cone(
        (0, 0, 0), heading_enu=0.0, half_angle_deg=30.0, vertical_tolerance_m=0.6))
    # 同高度的真实障碍仍能被检出
    grid.insert(np.array([[2.5, 0.0, 0.1]]), origin_enu=(0, 0, 0), now=0.0)
    assert 2.0 <= grid.nearest_in_cone(
        (0, 0, 0), heading_enu=0.0, half_angle_deg=30.0, vertical_tolerance_m=0.6) <= 3.0


def test_cluster_centroid_empty_map_is_six_tuple():
    """回归：空地图时必须返回 6 元组。

    2026-10-18 实测：启动瞬间地图为空，旧实现提前返回 3 元组，
    local_map 解包 6 个变量直接崩溃（`not enough values to unpack`）。
    """
    grid = VoxelGrid(resolution=0.5, extent=5.0)
    result = grid.cluster_centroid_in_cone((0, 0, 0), heading_enu=0.0)
    assert len(result) == 6
    assert result[0] is None
    assert result[1] == 0
    assert math.isinf(result[2])
    assert result[3:] == (None, None, None)


def test_cluster_centroid_returns_height_range():
    """高度门需要簇的 z 范围/水平跨度；返回元组的后三项必须有效。"""
    grid = VoxelGrid(resolution=0.5, extent=10.0)
    points = np.array([[3.0, y, z] for z in (1.0, 1.5, 2.0)
                       for y in (-0.3, 0.0, 0.3)])
    grid.insert(points, origin_enu=(0, 0, 0), now=0.0)
    result = grid.cluster_centroid_in_cone(
        (0, 0, 0), heading_enu=0.0, half_angle_deg=30.0, max_distance=6.0,
        vertical_tolerance_m=3.0, vertical_down_tolerance_m=3.0,
        min_cluster_points=3)
    delta, size, distance, z_min, z_max, span_h = result
    assert delta is not None
    assert size >= 3
    assert 2.5 <= distance <= 4.5
    assert z_min == pytest.approx(1.0, abs=0.3)
    assert z_max == pytest.approx(2.0, abs=0.3)
    assert span_h >= 0.5
