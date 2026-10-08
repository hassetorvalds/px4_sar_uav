"""接近监督节点：按任务策略决定何时停止接近，并向接口层下发 hold。

输入：`/vlm_navigator/decision`（VLM 节点的指向与深度决策）
输出：`/offboard_bridge/command`（hold）与 `~/status`（可观测）

安全边界：只在 PX4 已解锁且处于 OFFBOARD 时干预；不直接接触 /fmu/*。
"""

import json
import time

import rclpy
from px4_interface.state import Px4StateMonitor
from rclpy.node import Node
from std_msgs.msg import String

from .approach_policy import evaluate_approach


class ApproachSupervisor(Node):
    def __init__(self) -> None:
        super().__init__('approach_supervisor')
        self.declare_parameter('decision_topic', '/vlm_navigator/decision')
        self.declare_parameter('command_topic', '/offboard_bridge/command')
        self.declare_parameter('standoff_m', 0.8)
        self.declare_parameter('timeout_s', 60.0)
        self.declare_parameter('hold_duration_s', 5.0)

        self._standoff = float(self.get_parameter('standoff_m').value)
        self._timeout = float(self.get_parameter('timeout_s').value)
        self._hold_duration = float(self.get_parameter('hold_duration_s').value)

        self._command_pub = self.create_publisher(
            String, str(self.get_parameter('command_topic').value), 10)
        self._status_pub = self.create_publisher(String, '~/status', 10)
        self.create_subscription(
            String, str(self.get_parameter('decision_topic').value),
            self._on_decision, 10)
        self._monitor = Px4StateMonitor(self)

        self._started_at = time.monotonic()
        self._hold_since = None
        self._holding = False
        self._last_depth = None
        self._last_label = ''
        self._decisions_seen = 0
        self._outcome = 'running'

        self.create_timer(1.0, self._tick)
        self.get_logger().info(
            f'接近监督启动：站定距离 {self._standoff:.2f} m，'
            f'接近超时 {self._timeout:.0f} s，到位后保持 {self._hold_duration:.0f} s')

    def _on_decision(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn('决策消息解析失败', throttle_duration_sec=10.0)
            return
        self._decisions_seen += 1
        self._last_depth = payload.get('depth_m')
        self._last_label = str(payload.get('label', ''))

    def _tick(self) -> None:
        state = self._monitor.snapshot()
        if not (state.armed and state.offboard):
            self._publish_status('等待可控制状态')
            return

        decision = evaluate_approach(
            self._last_depth, time.monotonic() - self._started_at,
            standoff_m=self._standoff, timeout_s=self._timeout,
            have_target=self._last_depth is not None,
            already_holding=self._holding)

        if decision.action in ('hold', 'timeout'):
            if not self._holding:
                self._holding = True
                self._hold_since = time.monotonic()
                self._outcome = decision.action
                command = String()
                command.data = 'hold'
                self._command_pub.publish(command)
                self.get_logger().info(f'停止接近并保持位置：{decision.reason}')
        elif self._holding and time.monotonic() - self._hold_since > self._hold_duration:
            self.get_logger().info(
                f'接近阶段结束（{self._outcome}），'
                f'共收到 {self._decisions_seen} 次决策，最近目标「{self._last_label}」')

        self._publish_status(decision.reason)

    def _publish_status(self, detail: str) -> None:
        msg = String()
        msg.data = (f'stage={self._outcome} holding={self._holding} '
                    f'depth_m={self._last_depth} decisions={self._decisions_seen} '
                    f'detail={detail}')
        self._status_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ApproachSupervisor()
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
