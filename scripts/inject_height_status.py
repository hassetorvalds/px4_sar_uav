#!/usr/bin/env python3
"""阶段 1：向旧驱动注入可控 voxel 簇顶（不依赖真实结构）。

用法：
    python3 scripts/inject_height_status.py --ros-args \
      -p topic:=/inject/height_status -p top_m:=1.0
旧驱动配：-p map_status_topic:=/inject/height_status -p height_gate_mode:=c_map_rel
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class HeightStatusInjector(Node):
    def __init__(self):
        super().__init__('height_status_injector')
        self.declare_parameter('topic', '/inject/height_status')
        self.declare_parameter('top_m', 1.0)
        self.declare_parameter('rate_hz', 5.0)
        self._top = float(self.get_parameter('top_m').value)
        self._pub = self.create_publisher(
            String, str(self.get_parameter('topic').value), 10)
        self.create_timer(1.0 / max(float(self.get_parameter('rate_hz').value), 1.0),
                          self._tick)
        self.get_logger().info(
            f'注入高度状态：cluster_z=0.0,{self._top:.2f} span_h=2.0')

    def _tick(self):
        self._pub.publish(String(data=(
            f'voxels=1 frames=1 cluster_z=0.0,{self._top:.2f} span_h=2.0 '
            f'fwd_floor_m=0.45')))


def main():
    rclpy.init()
    node = HeightStatusInjector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
