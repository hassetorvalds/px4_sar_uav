"""A* 候选速度 → offboard 桥（默认关闭，feature flag）。

安全门：enabled + armed + offboard + not failsafe + candidate fresh + speed≤max
+ active duration≤max_active_s。任何条件不满足都不发布。
"""

import math
import time

import rclpy
from geometry_msgs.msg import Twist
from px4_interface.state import Px4StateMonitor
from rclpy.node import Node
from std_msgs.msg import String


def bridge_allowed(*, enabled: bool, allow_flight: bool, armed: bool, offboard: bool, failsafe: bool,
                   age_s: float, timeout_s: float, speed_mps: float,
                   max_speed_mps: float, active_s: float,
                   max_active_s: float):
    if not enabled:
        return False, 'disabled'
    if not allow_flight:
        return False, 'flight_not_authorized'
    if not armed or not offboard:
        return False, 'not_armed_offboard'
    if failsafe:
        return False, 'failsafe'
    if age_s > timeout_s:
        return False, 'candidate_stale'
    if speed_mps > max_speed_mps + 1e-6:
        return False, 'speed_limit'
    if active_s > max_active_s:
        return False, 'max_duration'
    return True, 'ok'


class AstarVelocityBridge(Node):
    def __init__(self):
        super().__init__('astar_velocity_bridge')
        self.declare_parameter('candidate_topic', '/astar_follower/velocity_candidate')
        self.declare_parameter('output_topic', '/offboard_bridge/velocity_setpoint')
        self.declare_parameter('enabled', False)
        self.declare_parameter('allow_flight', False)
        self.declare_parameter('max_speed_mps', 0.3)
        self.declare_parameter('candidate_timeout_s', 0.5)
        self.declare_parameter('max_active_s', 10.0)
        self.declare_parameter('rate_hz', 20.0)
        self._enabled = bool(self.get_parameter('enabled').value)
        self._allow_flight = bool(self.get_parameter('allow_flight').value)
        self._max_speed = float(self.get_parameter('max_speed_mps').value)
        self._timeout = float(self.get_parameter('candidate_timeout_s').value)
        self._max_active = float(self.get_parameter('max_active_s').value)
        self._candidate = None
        self._candidate_at = 0.0
        self._active_since = None
        self._monitor = Px4StateMonitor(self)
        self.create_subscription(
            Twist, str(self.get_parameter('candidate_topic').value),
            self._on_candidate, 5)
        self._pub = self.create_publisher(
            Twist, str(self.get_parameter('output_topic').value), 5)
        self._status_pub = self.create_publisher(String, '~/status', 5)
        rate = max(float(self.get_parameter('rate_hz').value), 2.0)
        self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info(f'astar_velocity_bridge enabled={self._enabled}')

    def _on_candidate(self, msg):
        self._candidate = msg
        self._candidate_at = time.monotonic()

    def _tick(self):
        state = self._monitor.snapshot()
        if not self._enabled:
            self._status_pub.publish(String(data='disabled'))
            return
        if self._candidate is None:
            self._status_pub.publish(String(data='no_candidate'))
            return
        now = time.monotonic()
        speed = math.hypot(self._candidate.linear.x, self._candidate.linear.y)
        if speed > 1e-6 and self._active_since is None:
            self._active_since = now
        active = 0.0 if self._active_since is None else now - self._active_since
        allowed, reason = bridge_allowed(
            enabled=self._enabled, allow_flight=self._allow_flight,
            armed=bool(state.armed),
            offboard=bool(state.offboard), failsafe=bool(state.failsafe),
            age_s=now - self._candidate_at, timeout_s=self._timeout,
            speed_mps=speed, max_speed_mps=self._max_speed,
            active_s=active, max_active_s=self._max_active)
        if allowed:
            self._pub.publish(self._candidate)
        self._status_pub.publish(String(
            data=f'allowed={allowed} reason={reason} speed={speed:.2f}'))


def main(args=None):
    rclpy.init(args=args)
    node = AstarVelocityBridge()
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
