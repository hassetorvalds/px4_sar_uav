"""像素/深度 → 世界坐标的几何单测。"""

import math

import numpy as np
import pytest

from perception.geometry import (CameraIntrinsics, depth_patch_median,
                                 optical_to_flu, pixel_to_body_flu,
                                 pixel_to_optical, pixel_to_world_enu,
                                 scale_pixel)


@pytest.fixture()
def intrinsics():
    return CameraIntrinsics.from_hfov(640, 480, 90.0)


def test_intrinsics_from_hfov():
    intr = CameraIntrinsics.from_hfov(640, 480, 90.0)
    assert intr.fx == pytest.approx(320.0)
    assert intr.fy == pytest.approx(320.0)
    assert intr.cx == pytest.approx(320.0)
    assert intr.cy == pytest.approx(240.0)


def test_center_pixel_is_forward(intrinsics):
    flu = pixel_to_body_flu(320, 240, 10.0, intrinsics)
    assert flu == pytest.approx([10.0, 0.0, 0.0])


def test_right_pixel_is_right_negative_left(intrinsics):
    # 像素在中心右侧 → 光学 x 为正（右）→ FLU 的 y（左）为负
    flu = pixel_to_body_flu(640, 240, 10.0, intrinsics)
    assert flu[0] == pytest.approx(10.0)
    assert flu[1] == pytest.approx(-10.0)
    assert flu[2] == pytest.approx(0.0)


def test_lower_pixel_is_down(intrinsics):
    # 像素在中心下方 → 光学 y 为正（下）→ FLU 的 z（上）为负
    # v=480、cy=240、fy=320、d=10 → y_opt = 240/320*10 = 7.5
    flu = pixel_to_body_flu(320, 480, 10.0, intrinsics)
    assert flu[2] == pytest.approx(-7.5)


def test_optical_to_flu_axis_mapping():
    assert optical_to_flu(np.array([0.0, 0.0, 5.0])) == pytest.approx([5.0, 0.0, 0.0])
    assert optical_to_flu(np.array([2.0, 0.0, 0.0])) == pytest.approx([0.0, -2.0, 0.0])
    assert optical_to_flu(np.array([0.0, 3.0, 0.0])) == pytest.approx([0.0, 0.0, -3.0])


def test_world_facing_east(intrinsics):
    world = pixel_to_world_enu(320, 240, 10.0, intrinsics, 0.0, (0.0, 0.0, 2.0))
    assert world == pytest.approx([10.0, 0.0, 2.0])


def test_world_facing_north(intrinsics):
    world = pixel_to_world_enu(320, 240, 10.0, intrinsics, math.pi / 2, (0.0, 0.0, 2.0))
    assert world == pytest.approx([0.0, 10.0, 2.0])


def test_world_offset_adds_position(intrinsics):
    world = pixel_to_world_enu(320, 240, 10.0, intrinsics, 0.0, (100.0, -50.0, 3.0))
    assert world == pytest.approx([110.0, -50.0, 3.0])


def test_depth_patch_median_ignores_invalid():
    depth = np.full((10, 10), np.nan, np.float32)
    depth[5, 5] = 4.0
    depth[5, 6] = 4.2
    depth[6, 5] = 3.8
    depth[0, 0] = 99.0                            # 窗口外
    assert depth_patch_median(depth, 5, 5, radius=1) == pytest.approx(4.0)


def test_depth_patch_median_all_invalid():
    depth = np.full((10, 10), np.nan, np.float32)
    assert depth_patch_median(depth, 5, 5, radius=2) is None


def test_depth_patch_median_out_of_bounds():
    depth = np.ones((10, 10), np.float32)
    assert depth_patch_median(depth, -1, 5, radius=1) is None


def test_scale_pixel():
    assert scale_pixel(320, 240, (640, 480), (1280, 720)) == (640.0, 360.0)
