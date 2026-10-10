"""体素地图纯逻辑的单元测试（不需要 ROS 运行）。"""

import math

import numpy as np

from mapping.voxel_grid import VoxelGrid, yaw_rotate_flu_to_enu


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
