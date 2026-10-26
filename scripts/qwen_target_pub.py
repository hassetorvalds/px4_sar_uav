#!/usr/bin/env python3
"""Qwen 目标异步发布器：标称 2 Hz，实际受 Qwen/AirSim 延迟限制。

发布 /qwen/target（map ENU），执行器在 goto step 设 dynamic_target:=true 时使用。
不是 4 Hz 视觉伺服；过期/跳变由执行器回退到初始目标。
"""

import base64
import io
import json
import math
import statistics
import subprocess
import time
import urllib.request

import airsim
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from PIL import Image
from rclpy.node import Node

from perception.geometry import (CameraIntrinsics, depth_patch_median,
                                 pixel_to_world_enu)
from px4_interface.frames import ned_to_enu, ned_yaw_to_enu_yaw
from qwen_target import orange_center, qwen_bbox, qwen_grounding


class QwenTargetPub(Node):
    def __init__(self):
        super().__init__('qwen_target_pub')
        self.declare_parameter('instruction', '飞到前方橙色目标附近')
        self.declare_parameter('target_prompt', '')
        self.declare_parameter('require_orange', True)
        self.declare_parameter('verify_roi', False)
        self.declare_parameter('use_bbox', False)
        self.declare_parameter('backend', 'qwen')
        self.declare_parameter('dino_url', 'http://127.0.0.1:18082/detect')
        self.declare_parameter('bbox_area_tolerance', 0.6)
        self.declare_parameter('bbox_edge_margin_px', 5.0)
        self.declare_parameter('roi_margin_ratio', 0.2)
        self.declare_parameter('roi_tolerance_ratio', 0.12)
        self.declare_parameter('qwen_url',
                               'http://127.0.0.1:18081/v1/chat/completions')
        self.declare_parameter('topic', '/qwen/target')
        self.declare_parameter('rate_hz', 2.0)
        self.declare_parameter('airsim_ip', '172.21.192.1')
        self.declare_parameter('airsim_port', 45000)
        self.declare_parameter('hfov_deg', 90.0)
        self.declare_parameter('geofence_m', 50.0)
        self.declare_parameter('max_spread_m', 2.0)
        self.declare_parameter('lock_radius_m', 2.0)
        self.declare_parameter('stable_hits', 3)
        self.declare_parameter('depth_gate_m', 3.0)
        self.declare_parameter('max_history', 5)
        self._instruction = str(self.get_parameter('instruction').value)
        self._target_prompt = str(self.get_parameter('target_prompt').value).strip()
        self._require_orange = bool(self.get_parameter('require_orange').value)
        self._verify_roi = bool(self.get_parameter('verify_roi').value)
        self._use_bbox = bool(self.get_parameter('use_bbox').value)
        self._backend = str(self.get_parameter('backend').value).strip().lower()
        self._dino_url = str(self.get_parameter('dino_url').value)
        self._bbox_area_tol = float(self.get_parameter('bbox_area_tolerance').value)
        self._bbox_edge_margin = float(self.get_parameter('bbox_edge_margin_px').value)
        self._bbox_area_history = []
        self._last_bbox = None
        self._roi_margin = float(self.get_parameter('roi_margin_ratio').value)
        self._roi_tol = float(self.get_parameter('roi_tolerance_ratio').value)
        self._url = str(self.get_parameter('qwen_url').value)
        self._geofence = float(self.get_parameter('geofence_m').value)
        self._hfov = float(self.get_parameter('hfov_deg').value)
        self._max_spread = float(self.get_parameter('max_spread_m').value)
        self._lock_radius = float(self.get_parameter('lock_radius_m').value)
        self._stable_hits = max(int(self.get_parameter('stable_hits').value), 1)
        self._depth_gate = float(self.get_parameter('depth_gate_m').value)
        self._max_history = max(int(self.get_parameter('max_history').value), 1)
        self._pub = self.create_publisher(
            PoseStamped, str(self.get_parameter('topic').value), 10)
        self._client = airsim.MultirotorClient(
            ip=str(self.get_parameter('airsim_ip').value),
            port=int(self.get_parameter('airsim_port').value), timeout_value=10)
        self._client.confirmConnection()
        self._history = []
        self._depth_history = []
        self._bbox_area_history = []
        self._bbox_size_history = []
        self._last_published = None
        self._locked_target = None
        self._busy = False
        self._last_publish = 0.0
        rate = max(float(self.get_parameter('rate_hz').value), 0.5)
        self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info(f'Qwen target publisher：{rate:.1f} Hz（实际受推理限制）')
        self.get_logger().info(f'DINO endpoint={self._dino_url} backend={self._backend}')

    def _bbox_center(self, bbox, width, height, clipped):
        """Edge-aware bbox 中心：用历史中位宽高补截断边。"""
        x1, y1, x2, y2 = [float(v) for v in bbox]
        if not clipped:
            w, h = x2 - x1, y2 - y1
            area = w * h
            if self._bbox_area_history:
                previous = self._bbox_area_history[-1]
                if abs(area - previous) / max(previous, 1.0) > self._bbox_area_tol:
                    raise ValueError('bbox area changed too much')
            self._bbox_area_history.append(area)
            self._bbox_area_history = self._bbox_area_history[-5:]
            self._bbox_size_history.append((w, h))
            self._bbox_size_history = self._bbox_size_history[-5:]
            return x1, y1, x2, y2
        if not self._bbox_size_history:
            return x1, y1, x2, y2
        mw = statistics.median(w for w, _ in self._bbox_size_history)
        mh = statistics.median(h for _, h in self._bbox_size_history)
        margin = self._bbox_edge_margin
        if x1 <= margin and x2 < width - margin:
            x1 = max(0.0, x2 - mw)
        elif x2 >= width - margin and x1 > margin:
            x2 = min(float(width), x1 + mw)
        if y1 <= margin and y2 < height - margin:
            y1 = max(0.0, y2 - mh)
        elif y2 >= height - margin and y1 > margin:
            y2 = min(float(height), y1 + mh)
        return x1, y1, x2, y2

    def _dino_detect(self, jpeg, width, height):
        payload = json.dumps({
            'image_b64': base64.b64encode(jpeg).decode('ascii'),
            'prompt': self._target_prompt,
            'box_threshold': 0.5,
            'text_threshold': 0.25,
            'meta': {'frame_seq': 0},
        }).encode('utf-8')
        started = time.monotonic()
        # 本机 rclpy 进程里 urllib 对 18082 会间歇性 Connection refused，
        # curl 子进程稳定；仅 DINO 调用使用 curl。
        completed = subprocess.run(
            ['curl', '-sS', '-m', '20', '-X', 'POST', self._dino_url,
             '-H', 'Content-Type: application/json', '--data-binary', '@-'],
            input=payload, capture_output=True, timeout=25)
        if completed.returncode != 0:
            raise ValueError(completed.stderr.decode('utf-8', errors='replace')[:200])
        data = json.loads(completed.stdout.decode('utf-8'))
        latency = (time.monotonic() - started) * 1000.0
        boxes = data.get('boxes') or []
        if not boxes:
            raise ValueError('DINO returned no boxes')
        best = max(boxes, key=lambda box: float(box.get('score', 0.0)))
        bbox = [float(best['x1']), float(best['y1']),
                float(best['x2']), float(best['y2'])]
        if bbox[2] <= bbox[0] or bbox[3] <= bbox[1]:
            raise ValueError('DINO invalid bbox')
        return bbox, float(best.get('score', 0.0)), latency

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
        prompt = None
        if self._target_prompt:
            prompt = ('请定位图像中的“%s”，只输出 JSON：x、y 为 [0,1000] '
                      '归一化坐标（左上原点），以及 confidence（0-1）。'
                      '不要 markdown。' % self._target_prompt)
        if self._backend == 'dino':
            bbox, confidence, latency = self._dino_detect(
                jpeg.getvalue(), width, height)
            clipped = (bbox[0] <= self._bbox_edge_margin
                       or bbox[1] <= self._bbox_edge_margin
                       or bbox[2] >= width - self._bbox_edge_margin
                       or bbox[3] >= height - self._bbox_edge_margin)
            adjusted = self._bbox_center(bbox, width, height, clipped)
            if clipped:
                self.get_logger().info(
                    f'DINO bbox clipped; edge-aware {adjusted}',
                    throttle_duration_sec=2.0)
            self._last_bbox = [round(float(v), 1) for v in adjusted]
            u = (adjusted[0] + adjusted[2]) / 2.0
            v = (adjusted[1] + adjusted[3]) / 2.0
        elif self._use_bbox:
            try:
                bbox_prompt = None
                if self._target_prompt:
                    bbox_prompt = ('请定位图像中的“%s”，只输出 JSON：'
                                   'bbox=[x1,y1,x2,y2]，坐标归一化到 [0,1000]，'
                                   '以及 confidence。不要 markdown。'
                                   % self._target_prompt)
                bbox, confidence, latency = qwen_bbox(
                    self._url, jpeg.getvalue(), self._instruction,
                    (width, height), prompt=bbox_prompt)
                clipped = (bbox[0] <= 1.0 or bbox[1] <= 1.0
                           or bbox[2] >= width - 1.0 or bbox[3] >= height - 1.0)
                if clipped:
                    # 截断 bbox 中心有偏；退回 Qwen 点，不拿裁边中心当目标。
                    self._last_bbox = None
                    u, v, confidence, latency = qwen_grounding(
                        self._url, jpeg.getvalue(), self._instruction,
                        (width, height), prompt=prompt)
                else:
                    area = (bbox[2] - bbox[0]) * (bbox[3] - bbox[1])
                    if self._bbox_area_history:
                        if abs(area - self._bbox_area_history[-1]) / max(
                                self._bbox_area_history[-1], 1.0) > self._bbox_area_tol:
                            raise ValueError('bbox area changed too much')
                    self._bbox_area_history.append(area)
                    self._bbox_area_history = self._bbox_area_history[-5:]
                    self._last_bbox = [round(float(v), 1) for v in bbox]
                    u = (bbox[0] + bbox[2]) / 2.0
                    v = (bbox[1] + bbox[3]) / 2.0
            except (KeyError, ValueError):
                # 点预测 fallback：bbox 缺失时仍保留可用目标。
                self._last_bbox = None
                u, v, confidence, latency = qwen_grounding(
                    self._url, jpeg.getvalue(), self._instruction,
                    (width, height), prompt=prompt)
        else:
            u, v, confidence, latency = qwen_grounding(
                self._url, jpeg.getvalue(), self._instruction, (width, height),
                prompt=prompt)
        if not self._use_bbox and self._verify_roi and not (
                0.15 * width < u < 0.85 * width
                and 0.15 * height < v < 0.85 * height):
            self.get_logger().info('target near image border; skip ROI verify',
                                   throttle_duration_sec=2.0)
        elif self._verify_roi:
            margin_x = max(width * self._roi_margin, 16.0)
            margin_y = max(height * self._roi_margin, 16.0)
            x1 = max(0, int(u - margin_x))
            y1 = max(0, int(v - margin_y))
            x2 = min(width, int(u + margin_x))
            y2 = min(height, int(v + margin_y))
            crop = image.crop((x1, y1, x2, y2))
            crop_buf = io.BytesIO()
            crop.save(crop_buf, format='JPEG', quality=85)
            cu, cv, crop_conf, crop_latency = qwen_grounding(
                self._url, crop_buf.getvalue(), self._instruction,
                (crop.width, crop.height), prompt=prompt)
            latency += crop_latency
            full_cu, full_cv = x1 + cu, y1 + cv
            offset = math.hypot(full_cu - u, full_cv - v)
            tolerance = max(width, height) * self._roi_tol
            if crop_conf is not None and float(crop_conf) < 0.5:
                raise ValueError(f'ROI confidence too low: {crop_conf}')
            if offset > tolerance:
                raise ValueError(f'ROI point offset {offset:.1f}px > {tolerance:.1f}px')
        proxy_error = None
        if self._require_orange:
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
        distance = math.hypot(world[0] - position_enu[0],
                              world[1] - position_enu[1])
        if distance > self._geofence:
            raise ValueError(f'target distance {distance:.1f} m > geofence')
        return world, latency, confidence, proxy_error, float(depth_m)

    def _tick(self):
        if self._busy:
            return
        self._busy = True
        try:
            world, latency, confidence, proxy_error, depth_m = self._capture()
            if (self._lock_radius > 0.0 and self._locked_target is not None
                    and math.hypot(world[0] - self._locked_target[0],
                                   world[1] - self._locked_target[1]) > self._lock_radius):
                raise ValueError('target jumped outside fixed lock radius')
            if self._depth_history:
                depth_median = statistics.median(self._depth_history)
                if abs(depth_m - depth_median) > self._depth_gate:
                    raise ValueError(
                        f'depth jump {abs(depth_m - depth_median):.2f} m '
                        f'> {self._depth_gate:.2f} m')
            self._history.append(tuple(float(v) for v in world))
            self._history = self._history[-self._max_history:]
            self._depth_history.append(depth_m)
            self._depth_history = self._depth_history[-self._max_history:]
            if len(self._history) < self._stable_hits:
                self.get_logger().info(
                    f'稳定样本 {len(self._history)}/{self._stable_hits}，暂不发布',
                    throttle_duration_sec=1.0)
                return
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
            if self._locked_target is None:
                self._locked_target = (mx, my)
            self._last_publish = time.monotonic()
            self.get_logger().info(
                f'Qwen target=({mx:.2f},{my:.2f}) spread={spread:.2f} '
                f'lat={latency:.0f}ms conf={confidence} bbox={self._last_bbox}',
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
