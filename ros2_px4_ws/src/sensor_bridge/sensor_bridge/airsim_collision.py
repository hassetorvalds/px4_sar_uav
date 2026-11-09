"""AirSim 碰撞监视：只发布碰撞状态，不控制飞控。"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String

from .collision_filter import collision_label, significant_collision

try:
    import airsim
except ImportError:  # pragma: no cover
    airsim = None


class AirSimCollision(Node):
    def __init__(self):
        super().__init__('airsim_collision')
        self.declare_parameter('host', '172.21.192.1')
        self.declare_parameter('port', 45000)
        self.declare_parameter('vehicle_name', 'Drone1')
        self.declare_parameter('rate_hz', 2.0)
        self._client = None
        self._last = None
        self._pub = self.create_publisher(Bool, '/sensor/collision', 5)
        self._info_pub = self.create_publisher(String, '/sensor/collision_info', 5)
        if airsim is not None:
            try:
                self._client = airsim.MultirotorClient(
                    ip=str(self.get_parameter('host').value),
                    port=int(self.get_parameter('port').value), timeout_value=5)
                self._client.confirmConnection()
            except Exception as exc:  # pragma: no cover
                self.get_logger().error(f'AirSim 连接失败：{exc}')
        else:
            self.get_logger().error('未安装 airsim 客户端')
        rate = max(float(self.get_parameter('rate_hz').value), 0.5)
        self.create_timer(1.0 / rate, self._tick)

    def _tick(self):
        if self._client is None:
            return
        try:
            info = self._client.simGetCollisionInfo(
                vehicle_name=str(self.get_parameter('vehicle_name').value))
        except Exception as exc:  # pragma: no cover
            self.get_logger().warn(f'碰撞查询失败：{exc}', throttle_duration_sec=10.0)
            return
        collided = significant_collision(
            has_collided=bool(info.has_collided),
            object_name=str(info.object_name),
            penetration_m=float(info.penetration_depth))
        self._pub.publish(Bool(data=collided))
        if info.has_collided:
            text = collision_label(str(info.object_name),
                                   float(info.penetration_depth))
            self._info_pub.publish(String(data=text))
            key = (info.object_name, round(info.penetration_depth, 3))
            if self._last != key:
                if collided:
                    self.get_logger().warn(text)
                else:
                    self.get_logger().info(text)
                self._last = key
        else:
            self._last = None


def main(args=None):
    rclpy.init(args=args)
    node = AirSimCollision()
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
