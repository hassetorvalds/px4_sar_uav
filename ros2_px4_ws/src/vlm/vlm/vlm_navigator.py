"""VLM 导航节点：图像 → 指向点 → 机体速度 → （经接口层）PX4。

安全边界：

- 本节点**不直接**接触 ``/fmu/*``，只向 offboard_bridge 发布 ENU 速度指令；
- 只有 PX4 已解锁、处于 OFFBOARD、且未进入 failsafe 时才发布指令；
- VLM 调用失败或超时即停止发布，桥接节点会在速度超时后回到位置保持；
- 所有指令都经过 ``pointing_to_body_velocity`` 的速度/偏航限幅。
"""

import json
import math
import os
import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from px4_interface.frames import body_flu_to_enu, ned_yaw_to_enu_yaw
from px4_interface.state import Px4StateMonitor
from rclpy.node import Node
from sensor_msgs.msg import Image

from .pointing import (
    PointingGeometry,
    PointingParseError,
    parse_pointing,
    pointing_to_body_velocity,
)
from .vlm_client import VlmError, create_client, image_file_to_jpeg, image_message_to_jpeg

# 提示词为本项目自写（接口约定与本地参考实现 SeePointFly 一致，但文字不复制其原文；
# SeePointFly 为 Proprietary 许可，见 docs/reference-see-point-fly.md）。
PROMPT_TEMPLATE = """你是一架无人机的视觉导航模块，需要根据机载相机图像决定下一个前往位置。

任务描述：{instruction}

请完成两件事：
1. 在图中找出所有符合该任务描述的目标物体；
2. 从中选出最值得前往的一个，并在该物体上标出一个点（尽量落在物体中心）。

只输出 JSON 数组，不要输出任何解释文字：
[{{"point": [y, x], "depth": depth_value, "label": "简短说明"}}]

取值约定：
- x：横向位置，0-1000，500 为画面中心，大于 500 表示偏右；
- y：纵向位置，0-1000，数值越小表示越靠画面上方；
- depth：目测距离等级 1-10，1 表示非常近（物体占画面很大），10 表示很远（物体很小）；
- label：一句话说明你指向的物体。

如果图中没有符合描述的目标，输出空数组 []。"""


class VlmNavigator(Node):
    def __init__(self) -> None:
        super().__init__('vlm_navigator')

        self.declare_parameter('instruction', 'fly toward the center of the open area')
        self.declare_parameter('provider', 'mock')
        self.declare_parameter('model', '')
        self.declare_parameter('api_key_env', '')
        self.declare_parameter('image_topic', '/sensor_bridge/camera/image')
        self.declare_parameter('image_file', '')
        self.declare_parameter('velocity_topic', '/offboard_bridge/velocity_setpoint')
        self.declare_parameter('command_rate_hz', 5.0)
        self.declare_parameter('vlm_rate_hz', 0.5)
        self.declare_parameter('hfov_deg', 90.0)
        self.declare_parameter('base_velocity', 1.0)
        self.declare_parameter('max_speed', 1.5)
        self.declare_parameter('max_yaw_rate', 0.6)
        self.declare_parameter('yaw_gain', 1.0)
        self.declare_parameter('yaw_deadband_deg', 10.0)
        self.declare_parameter('stop_distance_m', 0.35)
        self.declare_parameter('min_command_duration_s', 1.0)
        self.declare_parameter('command_timeout_s', 3.0)
        self.declare_parameter('dry_run', False)
        self.declare_parameter('output_dir', 'vlm_decisions')
        self.declare_parameter('record_decisions', True)

        self._instruction = str(self.get_parameter('instruction').value)
        self._image_file = str(self.get_parameter('image_file').value)
        self._dry_run = bool(self.get_parameter('dry_run').value)
        self._max_speed = float(self.get_parameter('max_speed').value)
        self._max_yaw_rate = float(self.get_parameter('max_yaw_rate').value)
        self._base_velocity = float(self.get_parameter('base_velocity').value)
        self._yaw_gain = float(self.get_parameter('yaw_gain').value)
        self._yaw_deadband_rad = math.radians(
            float(self.get_parameter('yaw_deadband_deg').value))
        self._stop_distance = float(self.get_parameter('stop_distance_m').value)
        self._min_duration = float(self.get_parameter('min_command_duration_s').value)
        self._command_timeout = float(self.get_parameter('command_timeout_s').value)
        self._record = bool(self.get_parameter('record_decisions').value)
        self._output_dir = str(self.get_parameter('output_dir').value)
        vlm_rate = max(float(self.get_parameter('vlm_rate_hz').value), 0.05)
        command_rate = max(float(self.get_parameter('command_rate_hz').value), 1.0)

        self._geometry = PointingGeometry(1, 1, float(self.get_parameter('hfov_deg').value))
        self._client = create_client(
            str(self.get_parameter('provider').value),
            str(self.get_parameter('model').value),
            str(self.get_parameter('api_key_env').value))

        self._velocity_pub = self.create_publisher(
            Twist, str(self.get_parameter('velocity_topic').value), 10)
        self._monitor = Px4StateMonitor(self)

        self._lock = threading.Lock()
        self._latest_jpeg = None
        self._latest_image_size = None
        self._pending_command = None      # (BodyVelocity, Pointing, timestamp)
        self._vlm_busy = False
        self._last_error = ''
        self._running = True

        if self._record:
            os.makedirs(self._output_dir, exist_ok=True)

        if self._image_file:
            self._load_image_file()
        else:
            topic = str(self.get_parameter('image_topic').value)
            self.create_subscription(Image, topic, self._on_image, 1)
            self.get_logger().info(f'等待图像话题：{topic}')

        self._worker = threading.Thread(target=self._vlm_loop, daemon=True)
        self._worker.start()
        self.create_timer(1.0 / command_rate, self._publish_loop)

        self.get_logger().info(
            f'vlm_navigator 启动：provider={self._client.__class__.__name__}，'
            f'指令「{self._instruction}」，速度上限 {self._max_speed:.2f} m/s，'
            f'偏航上限 {self._max_yaw_rate:.2f} rad/s，dry_run={self._dry_run}')

    # ------------------------------------------------------------------
    # 图像来源
    # ------------------------------------------------------------------

    def _load_image_file(self) -> None:
        from PIL import Image as PilImage

        with PilImage.open(self._image_file) as image:
            width, height = image.size
        jpeg = image_file_to_jpeg(self._image_file)
        with self._lock:
            self._latest_jpeg = jpeg
            self._latest_image_size = (width, height)
        self._geometry = PointingGeometry(
            width, height, self._geometry.hfov_deg)
        self.get_logger().info(
            f'离线模式：使用静态图像 {self._image_file}（{width}x{height}）')

    def _on_image(self, msg: Image) -> None:
        try:
            jpeg = image_message_to_jpeg(msg)
        except (VlmError, ImportError) as exc:
            self.get_logger().warn(f'图像编码失败：{exc}', throttle_duration_sec=10.0)
            return
        with self._lock:
            self._latest_jpeg = jpeg
            self._latest_image_size = (int(msg.width), int(msg.height))

    # ------------------------------------------------------------------
    # VLM 工作线程
    # ------------------------------------------------------------------

    def _vlm_loop(self) -> None:
        period = 1.0 / max(float(self.get_parameter('vlm_rate_hz').value), 0.05)
        while self._running:
            started = time.monotonic()
            self._run_single_decision()
            time.sleep(max(0.0, period - (time.monotonic() - started)))

    def _run_single_decision(self) -> None:
        with self._lock:
            jpeg = self._latest_jpeg
            size = self._latest_image_size
        if jpeg is None or size is None:
            return
        if size != (self._geometry.image_width, self._geometry.image_height):
            self._geometry = PointingGeometry(
                size[0], size[1], self._geometry.hfov_deg)

        prompt = PROMPT_TEMPLATE.format(instruction=self._instruction)
        try:
            response = self._client.complete(prompt, jpeg)
            pointing = parse_pointing(response)
            command = pointing_to_body_velocity(
                pointing, self._geometry,
                base_velocity=self._base_velocity,
                max_speed=self._max_speed,
                max_yaw_rate=self._max_yaw_rate,
                yaw_gain=self._yaw_gain,
                yaw_deadband_rad=self._yaw_deadband_rad,
                stop_distance_m=self._stop_distance,
                min_duration_s=self._min_duration)
        except (VlmError, PointingParseError, ValueError) as exc:
            if str(exc) != self._last_error:
                self.get_logger().warn(f'本周期未产生有效指令：{exc}')
                self._last_error = str(exc)
            return

        self._last_error = ''
        with self._lock:
            self._pending_command = (command, pointing, time.monotonic(), response)
        self.get_logger().info(
            f'VLM 指向：像素 {self._geometry.pixel_from_norm(pointing.x_norm, pointing.y_norm)}'
            f' 深度 {pointing.depth_m:.2f} m「{pointing.label}」→ '
            f'机体速度 (前 {command.forward:.2f}, 左 {command.left:.2f}, '
            f'上 {command.up:.2f}) m/s，偏航 {command.yaw_rate:.2f} rad/s，'
            f'时长 {command.duration_s:.1f} s')
        if self._record:
            self._record_decision(pointing, command, response)

    # ------------------------------------------------------------------
    # 指令发布
    # ------------------------------------------------------------------

    def _publish_loop(self) -> None:
        with self._lock:
            pending = self._pending_command
        if pending is None:
            return
        command, pointing, stamp, _ = pending
        if time.monotonic() - stamp > self._command_timeout:
            self.get_logger().warn(
                'VLM 指令已过期，停止发布（桥接节点将回到位置保持）',
                throttle_duration_sec=5.0)
            return

        state = self._monitor.snapshot()
        if not (state.armed and state.offboard and not state.failsafe):
            if not self._monitor.status_fresh():
                self.get_logger().warn(
                    '未收到 PX4 状态，暂不发送速度指令', throttle_duration_sec=5.0)
            else:
                self.get_logger().info(
                    f'未处于可控制状态（armed={state.armed}, offboard={state.offboard}, '
                    f'failsafe={state.failsafe}），暂不发送速度指令',
                    throttle_duration_sec=5.0)
            return

        heading_enu = ned_yaw_to_enu_yaw(state.heading)
        east, north, up = body_flu_to_enu(
            command.forward, command.left, command.up, heading_enu)
        twist = Twist()
        twist.linear.x = east
        twist.linear.y = north
        twist.linear.z = up
        twist.angular.z = command.yaw_rate

        if self._dry_run:
            self.get_logger().info(
                f'[dry_run] ENU 速度 ({east:.2f}, {north:.2f}, {up:.2f}) m/s，'
                f'偏航 {command.yaw_rate:.2f} rad/s',
                throttle_duration_sec=1.0)
            return
        self._velocity_pub.publish(twist)

    # ------------------------------------------------------------------
    # 决策记录
    # ------------------------------------------------------------------

    def _record_decision(self, pointing, command, response: str) -> None:
        stamp = time.strftime('%Y%m%d_%H%M%S')
        record = {
            'timestamp': stamp,
            'instruction': self._instruction,
            'provider': self._client.__class__.__name__,
            'image_size': [self._geometry.image_width, self._geometry.image_height],
            'point_pixel': list(self._geometry.pixel_from_norm(
                pointing.x_norm, pointing.y_norm)),
            'point_norm': [pointing.x_norm, pointing.y_norm],
            'depth_m': pointing.depth_m,
            'depth_score': pointing.depth_raw,
            'label': pointing.label,
            'body_velocity': {
                'forward': command.forward,
                'left': command.left,
                'up': command.up,
                'yaw_rate': command.yaw_rate,
                'duration_s': command.duration_s,
            },
            'raw_response': response,
        }
        path = os.path.join(self._output_dir, f'decision_{stamp}.json')
        try:
            with open(path, 'w', encoding='utf-8') as handle:
                json.dump(record, handle, ensure_ascii=False, indent=2)
        except OSError as exc:
            self.get_logger().warn(f'决策记录写入失败：{exc}')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VlmNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._running = False
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
