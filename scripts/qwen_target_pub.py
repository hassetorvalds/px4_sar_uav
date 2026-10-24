#!/usr/bin/env python3
"""Qwen 目标异步发布器：标称 2 Hz，实际受 Qwen/AirSim 延迟限制。

发布 /qwen/target（map ENU），执行器在 goto step 设 dynamic_target:=true 时使用。
不是 4 Hz 视觉伺服；过期/跳变由执行器回退到初始目标。
"""

import io
import math
import statistics
import time

import airsim
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from PIL import Image
from rclpy.node import Node

from perception.geometry import (CameraIntrinsics, depth_patch_median,
                                 pixel_to_world_enu)
from px4_interface.frames import ned_to_enu, ned_yaw_to_enu_yaw
from qwen_target import orange_center, qwen_grounding


class QwenTargetPub(Node):
    def __init__(self):
        super().__init__('qwen_target_pub')
        self.declare_parameter('instruction', '飞到前方橙色目标附近')
        self.declare_parameter('qwen_url',
                               'http://127.0.0.1:18081/v1/chat/completions')
        self.declare_parameter('topic', '/qwen/target')
        self.declare_parameter('rate_hz', 2.0)
        self.declare_parameter('airsim_ip', '172.21.192.1')
        self.declare_parameter('airsim_port', 45000)
        self.declare_parameter('hfov_deg', 90.0)
        self.declare_parameter('geofence_m', 50.0)
        self.declare_parameter('max_spread_m', 2.0)
        self.declare_parameter('max_history', 5)
        self._instruction = str(self.get_parameter('instruction').value)
        self._url = str(self.get_parameter('qwen_url').value)
        self._geofence = float(self.get_parameter('geofence_m').value)
        self._hfov = float(self.get_parameter('hfov_deg').value)
        self._max_spread = float(self.get_parameter('max_spread_m').value)
        self._max_history = max(int(self.get_parameter('max_history').value), 1)
        self._pub = self.create_publisher(
            PoseStamped, str(self.get_parameter('topic').value), 10)
        self._client = airsim.MultirotorClient(
            ip=str(self.get_parameter('airsim_ip').value),
            port=int(self.get_parameter('airsim_port').value), timeout_value=10)
        self._client.confirmConnection()
        self._history = []
        self._last_published = None
        self._busy = False
        self._last_publish = 0.0
        rate = max(float(self.get_parameter('rate_hz').value), 0.5)
        self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info(f'Qwen target publisher：{rate:.1f} Hz（实际受推理限制）')

    def _capture(self):
        state = self._client.getMultirotorState()
        pose = state.kinematics_estimated.position
        camera = self._client.simGetCameraInfo('0', vehicle_name='Drone1')
        _, _, yaw_ned = airsim.to_eularian_angles(camera.pose.orientation)
        position_enu = ned_to_enu(pose.x_val, pose.y_val, pose.z_val)
        heading_enu = ned_yaw_to_enu_yaw(yaw_ned)
        responses = self._client.simGetImages([
            airsim.ImageRequest('0', airsim.ImageType.Scene, False, False),
            airsim.ImageRequest('0', airsim.ImageType.DepthPlanar, True, False),
        ], vehicle_name='Drone1')
        scene, depth_response = responses[0], responses[1]
        raw = np.frombuffer(scene.image_data_uint8, dtype=np.uint8)
        height, width = int(scene.height), int(scene.width)
        image = Image.fromarray(
            raw[:height * width * 3].reshape(height, width, 3)[:, :, ::-1], 'RGB')
        jpeg = io.BytesIO()
        image.save(jpeg, format='JPEG', quality=85)
        dw, dh = int(depth_response.width), int(depth_response.height)
        depth = np.asarray(depth_response.image_data_float, dtype=float).reshape(dh, dw)
        u, v, confidence, latency = qwen_grounding(
            self._url, jpeg.getvalue(), self._instruction, (width, height))
        expected = orange_center(image)
        if expected is None:
            raise ValueError('orange target not visible')
        proxy_error = math.hypot(u / width - expected[0], v / height - expected[1])
        if proxy_error > 0.15:
            raise ValueError(f'Qwen proxy error {proxy_error:.3f} > 0.15')
        if confidence is not None and float(confidence) < 0.5:
            raise ValueError(f'Qwen confidence too low: {confidence}')
        depth_m = depth_patch_median(depth, u * dw / width, v * dh / height, radius=2)
        if depth_m is None or not math.isfinite(depth_m) or depth_m <= 0.0:
            raise ValueError('no valid depth')
        world = pixel_to_world_enu(
            u, v, depth_m, CameraIntrinsics.from_hfov(width, height, self._hfov),
            heading_enu, position_enu)
        return world, latency, confidence, proxy_error

    def _tick(self):
        if self._busy:
            return
        self._busy = True
        try:
            world, latency, confidence, proxy_error = self._capture()
            if (self._last_published is not None
                    and math.hypot(world[0] - self._last_published[0],
                                   world[1] - self._last_published[1]) > 3.0):
                raise ValueError('target jump > 3 m')
            self._history.append(tuple(float(v) for v in world))
            self._history = self._history[-self._max_history:]
            xs = [p[0] for p in self._history]
            ys = [p[1] for p in self._history]
            mx, my = statistics.median(xs), statistics.median(ys)
            spread = max(math.hypot(p[0] - mx, p[1] - my)
                         for p in self._history)
            if spread > self._max_spread:
                self.get_logger().warn(
                    f'目标 spread {spread:.2f} m > {self._max_spread:.2f}；暂不发布',
                    throttle_duration_sec=2.0)
                return
            pose = PoseStamped()
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.header.frame_id = 'map'
            pose.pose.position.x = float(mx)
            pose.pose.position.y = float(my)
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            self._pub.publish(pose)
            self._last_published = (mx, my)
            self._last_publish = time.monotonic()
            self.get_logger().info(
                f'Qwen target=({mx:.2f},{my:.2f}) spread={spread:.2f} '
                f'lat={latency:.0f}ms conf={confidence}',
                throttle_duration_sec=1.0)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f'目标采样失败：{type(exc).__name__}: {exc}',
                                   throttle_duration_sec=2.0)
            # 保持最后有效目标，避免执行器因短暂丢失而回退/停住。
            if self._last_published is not None:
                pose = PoseStamped()
                pose.header.stamp = self.get_clock().now().to_msg()
                pose.header.frame_id = 'map'
                pose.pose.position.x = float(self._last_published[0])
                pose.pose.position.y = float(self._last_published[1])
                pose.pose.orientation.w = 1.0
                self._pub.publish(pose)
        finally:
            self._busy = False


def main():
    rclpy.init()
    node = QwenTargetPub()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
