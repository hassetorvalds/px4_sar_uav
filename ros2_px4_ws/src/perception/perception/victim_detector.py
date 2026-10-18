"""人员/目标检测节点：RGB(+深度) → 检测框 + 世界系位置（ENU）。

输入：
  ``image_topic``         sensor_msgs/Image（bgr8/rgb8）
  ``depth_topic``         sensor_msgs/Image（32FC1 米 / 16UC1 毫米，可缺省）
  ``camera_info_topic``   sensor_msgs/CameraInfo
  PX4 状态（位置/航向，来自 px4_interface 的 Px4StateMonitor）

输出：
  ``~/detections``（std_msgs/String，JSON）：每帧的检测列表（bbox / 分数 / 深度 / 世界点）
  ``~/best``（geometry_msgs/PoseStamped，map）：分数最高的目标，供任务层悬停/记录

安全边界：本节点**只发布观测**，不发任何飞控指令；世界点只在 PX4 位置新鲜时给出。
"""

import json
import math
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from px4_interface.frames import ned_to_enu, ned_yaw_to_enu_yaw
from px4_interface.state import Px4StateMonitor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String

from .detectors import make_detector
from .geometry import (CameraIntrinsics, depth_patch_median, pixel_to_body_flu,
                       pixel_to_world_enu, scale_pixel)


def _decode_rgb(msg: Image):
    """sensor_msgs/Image → HxWx3 BGR uint8。"""
    channels = 3
    array = np.frombuffer(msg.data, dtype=np.uint8)
    expected = int(msg.height) * int(msg.width) * channels
    if array.size < expected:
        raise ValueError(f'图像数据不足：{array.size} < {expected}')
    image = array[:expected].reshape(int(msg.height), int(msg.width), channels)
    if msg.encoding == 'rgb8':
        image = image[..., ::-1]
    elif msg.encoding != 'bgr8':
        raise ValueError(f'不支持的图像编码：{msg.encoding}')
    return np.ascontiguousarray(image)


def _decode_depth(msg: Image):
    """sensor_msgs/Image → HxW float32（米）。"""
    if msg.encoding == '32FC1':
        array = np.frombuffer(msg.data, dtype=np.float32)
        expected = int(msg.height) * int(msg.width)
        if array.size < expected:
            raise ValueError(f'深度数据不足：{array.size} < {expected}')
        return array[:expected].reshape(int(msg.height), int(msg.width))
    if msg.encoding == '16UC1':
        array = np.frombuffer(msg.data, dtype=np.uint16)
        expected = int(msg.height) * int(msg.width)
        if array.size < expected:
            raise ValueError(f'深度数据不足：{array.size} < {expected}')
        return array[:expected].reshape(int(msg.height), int(msg.width)).astype(np.float32) / 1000.0
    raise ValueError(f'不支持的深度编码：{msg.encoding}')


class VictimDetector(Node):
    def __init__(self) -> None:
        super().__init__('victim_detector')
        self.declare_parameter('image_topic', '/airsim_camera/image')
        self.declare_parameter('depth_topic', '/airsim_camera/depth')
        self.declare_parameter('camera_info_topic', '/airsim_camera/camera_info')
        self.declare_parameter('detector', 'color')
        self.declare_parameter('hsv_lower', [15.0, 0.5, 0.4])
        self.declare_parameter('hsv_upper', [40.0, 1.0, 1.0])
        self.declare_parameter('min_area_px', 200.0)
        self.declare_parameter('max_area_px', 1.0e9)
        self.declare_parameter('min_score', 0.4)
        self.declare_parameter('label', 'target')
        self.declare_parameter('max_rate_hz', 5.0)
        self.declare_parameter('depth_patch_px', 2)
        self.declare_parameter('min_depth_m', 0.3)
        self.declare_parameter('max_depth_m', 30.0)
        self.declare_parameter('detections_topic', '~/detections')
        self.declare_parameter('best_topic', '~/best')
        self.declare_parameter('h_min_area', 120.0)   # 像素→世界点所需的最小 bbox 面积

        self._min_score = float(self.get_parameter('min_score').value)
        self._max_rate_s = 1.0 / max(float(self.get_parameter('max_rate_hz').value), 0.1)
        self._depth_patch = int(self.get_parameter('depth_patch_px').value)
        self._min_depth = float(self.get_parameter('min_depth_m').value)
        self._max_depth = float(self.get_parameter('max_depth_m').value)
        self._h_min_area = float(self.get_parameter('h_min_area').value)
        self._label = str(self.get_parameter('label').value)
        self._detector = self._build_detector()
        self._intrinsics = None
        self._depth = None
        self._depth_size = None
        self._last_image_at = 0.0
        self._sequence = 0
        self._monitor = Px4StateMonitor(self)

        self._detections_pub = self.create_publisher(
            String, str(self.get_parameter('detections_topic').value), 10)
        self._best_pub = self.create_publisher(
            PoseStamped, str(self.get_parameter('best_topic').value), 10)

        self.create_subscription(
            Image, str(self.get_parameter('image_topic').value),
            self._on_image, qos_profile_sensor_data)
        self.create_subscription(
            Image, str(self.get_parameter('depth_topic').value),
            self._on_depth, qos_profile_sensor_data)
        self.create_subscription(
            CameraInfo, str(self.get_parameter('camera_info_topic').value),
            self._on_info, 10)
        self.get_logger().info(
            f'检测器就绪：{self._detector.__class__.__name__}；'
            f'输出 {self.get_parameter("detections_topic").value}')

    def _build_detector(self):
        kind = str(self.get_parameter('detector').value).strip().lower()
        kwargs = {
            'lower': [float(v) for v in self.get_parameter('hsv_lower').value],
            'upper': [float(v) for v in self.get_parameter('hsv_upper').value],
            'min_area': float(self.get_parameter('min_area_px').value),
            'max_area': float(self.get_parameter('max_area_px').value),
            'label': self._label,
        }
        try:
            return make_detector(kind, **kwargs)
        except ImportError as exc:
            raise SystemExit(
                f'检测器 {kind!r} 需要 OpenCV，但 import 失败：{exc}\n'
                '本机 numpy 2.x 与系统 cv2 不兼容时，请用 PYTHONNOUSERSITE=1 启动。') from exc

    # ------------------------------------------------------------------
    # 订阅
    # ------------------------------------------------------------------

    def _on_info(self, msg: CameraInfo) -> None:
        self._intrinsics = CameraIntrinsics.from_k(msg.k, msg.width, msg.height)

    def _on_depth(self, msg: Image) -> None:
        try:
            self._depth = _decode_depth(msg)
            self._depth_size = (msg.width, msg.height)
        except ValueError as exc:
            self.get_logger().warn(f'深度解码失败：{exc}', throttle_duration_sec=10.0)

    def _on_image(self, msg: Image) -> None:
        now = time.monotonic()
        if now - self._last_image_at < self._max_rate_s:
            return
        self._last_image_at = now
        try:
            image = _decode_rgb(msg)
        except ValueError as exc:
            self.get_logger().warn(f'图像解码失败：{exc}', throttle_duration_sec=10.0)
            return
        if self._intrinsics is None:
            # 没有 CameraInfo 时按 90° 水平视场兜底（与 airsim_camera 默认一致）
            self._intrinsics = CameraIntrinsics.from_hfov(
                int(msg.width), int(msg.height), 90.0)

        raw = self._detector.detect(image)
        state = self._monitor.snapshot()
        position_enu = None
        heading_enu = None
        if state.position_ned() is not None and not math.isnan(state.heading):
            position_enu = np.array(ned_to_enu(*state.position_ned()))
            heading_enu = ned_yaw_to_enu_yaw(state.heading)
        pose_fresh = self._monitor.position_fresh()

        detections = []
        best = None
        for det in raw:
            if det.score < self._min_score:
                continue
            item = det.to_dict()
            item['depth_m'] = None
            item['range_m'] = None
            item['world_enu'] = None
            u, v = det.center
            if (self._depth is not None and self._depth_size is not None
                    and det.area >= self._h_min_area):
                du, dv = scale_pixel(u, v, (msg.width, msg.height), self._depth_size)
                depth = depth_patch_median(self._depth, du, dv, self._depth_patch)
                if depth is not None and self._min_depth <= depth <= self._max_depth:
                    item['depth_m'] = round(float(depth), 3)
                    flu = pixel_to_body_flu(u, v, depth, self._intrinsics)
                    item['range_m'] = round(float(np.linalg.norm(flu)), 3)
                    if position_enu is not None and heading_enu is not None and pose_fresh:
                        world = pixel_to_world_enu(
                            u, v, depth, self._intrinsics, heading_enu, position_enu)
                        item['world_enu'] = [round(float(c), 2) for c in world]
            detections.append(item)
            if best is None or det.score * max(det.area, 1.0) > best[0]:
                best = (det.score * max(det.area, 1.0), det, item)

        self._sequence += 1
        payload = {
            'seq': self._sequence,
            'stamp': round(float(msg.header.stamp.sec)
                           + float(msg.header.stamp.nanosec) * 1e-9, 3),
            'pose_age_s': (None if not pose_fresh
                           else round(self._monitor.position_age_s(), 3)),
            'count': len(detections),
            'detections': detections,
        }
        self._detections_pub.publish(String(data=json.dumps(payload, ensure_ascii=False)))

        if best is not None and best[2].get('world_enu') is not None:
            self._publish_best(best[2])
            self.get_logger().info(
                f'检测 {best[1].label} score={best[1].score:.2f} '
                f'depth={best[2]["depth_m"]} m world={best[2]["world_enu"]}',
                throttle_duration_sec=1.0)
        elif detections:
            self.get_logger().info(
                f'检测 {len(detections)} 个目标（无可用深度/位姿，未给世界点）',
                throttle_duration_sec=2.0)

    def _publish_best(self, item) -> None:
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = 'map'
        pose.pose.position.x = float(item['world_enu'][0])
        pose.pose.position.y = float(item['world_enu'][1])
        pose.pose.position.z = float(item['world_enu'][2])
        pose.pose.orientation.w = 1.0
        self._best_pub.publish(pose)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VictimDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
