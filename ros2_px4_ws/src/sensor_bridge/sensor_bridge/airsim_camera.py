"""AirSim 相机 → ROS 2 话题。

发布：
  ~/image        sensor_msgs/Image   彩色图（bgr8），来自 AirSim 的 Scene
  ~/depth        sensor_msgs/Image   深度图（32FC1，单位米），来自 DepthPlanar（可关）
  ~/camera_info  sensor_msgs/CameraInfo

注意：
- AirSim 的 RPC 端口由 settings.json 的 ApiServerPort 决定（本机为 45000，非默认 41451）；
- 相机分辨率与图像类型同样来自 settings.json，改完必须重启 AirSim 才能生效；
- 图像解码用 numpy（AirSim 客户端返回 uint8 缓冲），刻意不使用 cv2
  （本机 cv2 与 numpy 2.x 存在 ABI 冲突）。
"""

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image

try:  # 允许在没有安装 airsim 客户端时仍能导入本模块做单元测试
    import airsim
except ImportError:  # pragma: no cover
    airsim = None

_IMAGE_TYPES = {
    'scene': 0,
    'depth_planar': 1,
    'depth_vis': 3,
}


class AirSimCamera(Node):
    """周期性抓取 AirSim 相机图像并发布为 ROS 2 话题。"""

    def __init__(self) -> None:
        super().__init__('airsim_camera')
        self.declare_parameter('host', '127.0.0.1')
        self.declare_parameter('port', 45000)
        self.declare_parameter('vehicle_name', 'Drone1')
        self.declare_parameter('camera_name', '0')
        self.declare_parameter('frame_id', 'camera_link')
        self.declare_parameter('rate_hz', 2.0)
        self.declare_parameter('publish_depth', True)
        self.declare_parameter('depth_interval', 5)
        self.declare_parameter('hfov_deg', 90.0)

        self._host = str(self.get_parameter('host').value)
        self._port = int(self.get_parameter('port').value)
        self._vehicle = str(self.get_parameter('vehicle_name').value)
        self._camera = str(self.get_parameter('camera_name').value)
        self._frame_id = str(self.get_parameter('frame_id').value)
        self._publish_depth = bool(self.get_parameter('publish_depth').value)
        self._depth_interval = max(int(self.get_parameter('depth_interval').value), 1)
        self._hfov_deg = float(self.get_parameter('hfov_deg').value)
        rate_hz = max(float(self.get_parameter('rate_hz').value), 0.1)

        self._image_pub = self.create_publisher(Image, '~/image', 10)
        self._depth_pub = self.create_publisher(Image, '~/depth', 10)
        self._info_pub = self.create_publisher(CameraInfo, '~/camera_info', 10)

        self._client = None
        self._frame_index = 0
        self._warned = False
        self._connect()
        self.create_timer(1.0 / rate_hz, self._tick)
        self.get_logger().info(
            f'airsim_camera 启动：{self._host}:{self._port}，'
            f'vehicle={self._vehicle} camera={self._camera}，{rate_hz:.1f} Hz，'
            f'深度发布={self._publish_depth}')

    # ------------------------------------------------------------------
    # 连接与抓帧
    # ------------------------------------------------------------------

    def _connect(self) -> None:
        if airsim is None:
            self.get_logger().error(
                '未安装 AirSim Python 客户端：pip3 install --user airsim '
                '（依赖 msgpack、msgpack-rpc-python）')
            return
        try:
            client = airsim.MultirotorClient(
                ip=self._host, port=self._port, timeout_value=10)
            client.confirmConnection()
            self._client = client
            self.get_logger().info('已连接 AirSim RPC')
        except Exception as exc:  # noqa: BLE001 - AirSim 抛出的异常类型不稳定
            self._client = None
            self.get_logger().warn(
                f'连接 AirSim 失败：{exc}（将每秒重试）', throttle_duration_sec=5.0)

    def _tick(self) -> None:
        if self._client is None:
            self._connect()
            if self._client is None:
                return

        try:
            requests = [airsim.ImageRequest(
                self._camera, airsim.ImageType.Scene, False, False)]
            # 深度图分辨率低但 RPC 开销大（LockStep 下会拖慢整帧），按间隔抽取
            want_depth = (self._publish_depth
                          and self._frame_index % self._depth_interval == 0)
            if want_depth:
                requests.append(airsim.ImageRequest(
                    self._camera, airsim.ImageType.DepthPlanar, True, False))
            responses = self._client.simGetImages(
                requests, vehicle_name=self._vehicle)
            self._frame_index += 1
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f'抓帧失败：{exc}（将重连）', throttle_duration_sec=5.0)
            self._client = None
            return

        if not responses or responses[0].height == 0:
            self.get_logger().warn('AirSim 返回空图像', throttle_duration_sec=5.0)
            return

        stamp = self.get_clock().now().to_msg()
        scene = responses[0]
        self._publish_scene(scene, stamp)
        if want_depth and len(responses) > 1:
            self._publish_depth_image(responses[1], stamp)
        self._publish_camera_info(scene.width, scene.height, stamp)

        if not self._warned and scene.width < 640:
            self._warned = True
            self.get_logger().warn(
                f'相机分辨率仅 {scene.width}x{scene.height}：请在 AirSim settings.json 的 '
                'Cameras 段提高分辨率后重启 AirSim')

    # ------------------------------------------------------------------
    # 发布
    # ------------------------------------------------------------------

    def _publish_scene(self, response, stamp) -> None:
        height, width = int(response.height), int(response.width)
        raw = np.frombuffer(response.image_data_uint8, dtype=np.uint8)
        expected = height * width * 3
        if raw.size < expected:
            self.get_logger().warn(
                f'彩色图像数据不完整：{raw.size} < {expected}', throttle_duration_sec=5.0)
            return
        image = Image()
        image.header.stamp = stamp
        image.header.frame_id = self._frame_id
        image.height = height
        image.width = width
        image.encoding = 'bgr8'
        image.is_bigendian = 0
        image.step = width * 3
        image.data = raw[:expected].tobytes()
        self._image_pub.publish(image)

    def _publish_depth_image(self, response, stamp) -> None:
        height, width = int(response.height), int(response.width)
        depth = np.array(response.image_data_float, dtype=np.float32)
        if depth.size < height * width:
            return
        image = Image()
        image.header.stamp = stamp
        image.header.frame_id = self._frame_id
        image.height = height
        image.width = width
        image.encoding = '32FC1'
        image.is_bigendian = 0
        image.step = width * 4
        image.data = depth[:height * width].tobytes()
        self._depth_pub.publish(image)

    def _publish_camera_info(self, width: int, height: int, stamp) -> None:
        info = CameraInfo()
        info.header.stamp = stamp
        info.header.frame_id = self._frame_id
        info.width = int(width)
        info.height = int(height)
        # 由水平视场推算焦距（像素），主点取图像中心；与 vlm/pointing 的几何保持一致
        focal = width / (2.0 * np.tan(np.radians(self._hfov_deg) / 2.0))
        info.k = [focal, 0.0, width / 2.0,
                  0.0, focal, height / 2.0,
                  0.0, 0.0, 1.0]
        info.p = [focal, 0.0, width / 2.0, 0.0,
                  0.0, focal, height / 2.0, 0.0,
                  0.0, 0.0, 1.0, 0.0]
        self._info_pub.publish(info)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AirSimCamera()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
