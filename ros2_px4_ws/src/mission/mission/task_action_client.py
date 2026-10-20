"""ExecuteTask Action 客户端（测试/BT/VLM 调用的最小入口）。"""

import argparse
import json
import sys
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from mission_interfaces.action import ExecuteTask


class TaskActionClient(Node):
    def __init__(self):
        super().__init__('task_action_client')
        self._client = ActionClient(self, ExecuteTask, '/mission/execute_task')
        self._last = None

    def send(self, task_json: str, timeout_s: float,
             cancel_after: float = 0.0, cancel_twice: bool = False) -> int:
        if not self._client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('找不到 /mission/execute_task')
            return 2
        goal = ExecuteTask.Goal()
        goal.task_json = task_json

        def on_feedback(msg):
            feedback = msg.feedback
            key = (feedback.phase, feedback.step_index)
            if key != self._last:
                self._last = key
                self.get_logger().info(
                    f'{feedback.phase} step {feedback.step_index + 1}/'
                    f'{feedback.step_total} {feedback.step_type} '
                    f'progress={feedback.progress:.0%} '
                    f'alt={feedback.altitude_lcl_m:.2f}m')

        send_future = self._client.send_goal_async(goal, feedback_callback=on_feedback)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=5.0)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error('goal 被拒绝')
            return 3
        self.get_logger().info('goal 已接受，等待结果…')
        result_future = goal_handle.get_result_async()
        start = time.monotonic()
        cancel_sent = False
        second_sent = False
        second_at = None
        while rclpy.ok() and not result_future.done():
            rclpy.spin_once(self, timeout_sec=0.1)
            elapsed = time.monotonic() - start
            if (cancel_after > 0 and not cancel_sent
                    and elapsed >= cancel_after):
                self.get_logger().warn(f'发送 cancel（{cancel_after:.1f}s）')
                goal_handle.cancel_goal_async()
                cancel_sent = True
                second_at = elapsed + 0.3
            if (cancel_twice and cancel_sent and not second_sent
                    and second_at is not None and elapsed >= second_at):
                self.get_logger().warn('重复发送 cancel（测试幂等）')
                goal_handle.cancel_goal_async()
                second_sent = True
            if elapsed > timeout_s:
                self.get_logger().error(f'等待结果超时（{timeout_s:.0f}s）')
                goal_handle.cancel_goal_async()
                return 4
        result = result_future.result().result
        self.get_logger().info(
            f'结果：success={result.success} message={result.message!r} '
            f'report={result.report_path}')
        try:
            findings = json.loads(result.findings_json or '[]')
        except json.JSONDecodeError:
            findings = []
        if findings:
            self.get_logger().info(f'发现物：{findings}')
        return 0 if result.success else 5


def main(argv=None):
    parser = argparse.ArgumentParser(description='通过 ExecuteTask Action 提交任务')
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument('--task-file', help='task_*.json')
    group.add_argument('--task-json', help='TaskSpec JSON 字符串')
    parser.add_argument('--timeout', type=float, default=180.0)
    parser.add_argument('--cancel-after', type=float, default=0.0,
                        help='N 秒后发 cancel（0=不取消）')
    parser.add_argument('--cancel-twice', action='store_true',
                        help='第一次 cancel 后 0.3 s 再发一次（测试幂等）')
    args = parser.parse_args(argv)
    if args.task_json:
        task_json = args.task_json
    else:
        with open(args.task_file, encoding='utf-8') as handle:
            task_json = handle.read()

    rclpy.init()
    node = TaskActionClient()
    try:
        return node.send(task_json, args.timeout, args.cancel_after,
                         args.cancel_twice)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
