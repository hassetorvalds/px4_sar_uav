"""像素 + 深度 → 世界坐标（纯函数，便于单测）。

链路与 ``sensor_bridge`` 的点云保持一致：

    像素(u,v) + 深度 d  --针孔-->  光学系 (x 右, y 下, z 前)
    光学系 --固定变换-->            机体 FLU (前, 左, 上)
    机体 FLU + 航向 --旋转-->        ENU 偏移
    ENU 偏移 + 飞机位置 -->         世界 ENU 点

约定：``heading_enu`` 为 ENU 偏航（自东轴逆时针），位置也是 ENU。
"""

import math
from dataclasses import dataclass

import numpy as np

from px4_interface.frames import body_flu_to_enu


@dataclass(frozen=True)
class CameraIntrinsics:
    fx: float
    fy: float
    cx: float
    cy: float

    @classmethod
    def from_k(cls, k, width=None, height=None):
        """从 ``sensor_msgs/CameraInfo.k`` 构造（k = [fx,0,cx, 0,fy,cy, 0,0,1]）。"""
        values = list(k)
        if len(values) < 9:
            raise ValueError('CameraInfo.k 长度不足 9')
        fx, cx, fy, cy = float(values[0]), float(values[2]), float(values[4]), float(values[5])
        if width is not None and height is not None:
            if not (0 < cx <= width and 0 < cy <= height):
                # 有些驱动把主点写成 0；此时退回图像中心
                cx, cy = width / 2.0, height / 2.0
        return cls(fx=fx, fy=fy, cx=cx, cy=cy)

    @classmethod
    def from_hfov(cls, width: int, height: int, hfov_deg: float):
        """与 ``airsim_camera`` 的构造方式一致：由水平视场推焦距，主点取中心。"""
        focal = width / (2.0 * math.tan(math.radians(hfov_deg) / 2.0))
        return cls(fx=focal, fy=focal, cx=width / 2.0, cy=height / 2.0)


def pixel_to_optical(u: float, v: float, depth_m: float,
                     intrinsics: CameraIntrinsics) -> np.ndarray:
    """像素 + 深度 → 光学系 (x 右, y 下, z 前)。"""
    z = float(depth_m)
    x = (float(u) - intrinsics.cx) / intrinsics.fx * z
    y = (float(v) - intrinsics.cy) / intrinsics.fy * z
    return np.array([x, y, z], dtype=float)


def optical_to_flu(point: np.ndarray) -> np.ndarray:
    """光学系 (右, 下, 前) → 机体 FLU (前, 左, 上)。"""
    x_right, y_down, z_forward = float(point[0]), float(point[1]), float(point[2])
    return np.array([z_forward, -x_right, -y_down], dtype=float)


def pixel_to_body_flu(u: float, v: float, depth_m: float,
                      intrinsics: CameraIntrinsics) -> np.ndarray:
    return optical_to_flu(pixel_to_optical(u, v, depth_m, intrinsics))


def pixel_to_world_enu(u: float, v: float, depth_m: float,
                       intrinsics: CameraIntrinsics,
                       heading_enu: float,
                       position_enu) -> np.ndarray:
    """像素 + 深度 → 世界 ENU 点。"""
    forward, left, up = pixel_to_body_flu(u, v, depth_m, intrinsics)
    east, north, up_world = body_flu_to_enu(forward, left, up, heading_enu)
    return np.array([position_enu[0] + east,
                     position_enu[1] + north,
                     position_enu[2] + up_world], dtype=float)


def depth_patch_median(depth: np.ndarray, u: float, v: float, radius: int = 2):
    """取 ``(u,v)`` 周围方形窗口内有效深度的中位数；无效返回 None。

    中位数比单点更稳（边缘/空洞/反光）。
    """
    if depth is None:
        return None
    height, width = depth.shape[:2]
    x = int(round(u))
    y = int(round(v))
    if not (0 <= x < width and 0 <= y < height):
        return None
    r = max(int(radius), 0)
    patch = depth[max(0, y - r):min(height, y + r + 1),
                  max(0, x - r):min(width, x + r + 1)].astype(float)
    valid = patch[np.isfinite(patch) & (patch > 0.0)]
    if valid.size == 0:
        return None
    return float(np.median(valid))


def scale_pixel(u: float, v: float, source_size, target_size):
    """把 ``source_size`` 下的像素映射到 ``target_size``（RGB 与深度分辨率不同时）。"""
    sw, sh = float(source_size[0]), float(source_size[1])
    tw, th = float(target_size[0]), float(target_size[1])
    if sw <= 0 or sh <= 0:
        return u, v
    return u * tw / sw, v * th / sh
