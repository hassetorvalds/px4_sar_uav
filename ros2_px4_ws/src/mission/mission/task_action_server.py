"""ExecuteTask Action 适配层。

不重写控制逻辑：goal 是一份 TaskSpec JSON，动作服务器把它发布到现有的
`/mission/task`，再从 `/mission/status` 读阶段/高度/结果。这样：
- 现有 CLI/话题接口不变；
- Action 只增加 feedback/cancel/result 语义，供 BT/VLM 调用；
- 取消走 `/mission/cancel`，执行器会落地上锁。
"""

import json
import time

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from mission_interfaces.action import ExecuteTask
from std_msgs.msg import Empty, String

from .task_spec import TaskError, TaskSpec


def status_to_feedback(status: dict) -> ExecuteTask.Feedback:
    """把执行器状态 JSON 映射成 Action feedback（纯逻辑，可单测）。"""
    feedback = ExecuteTask.Feedback()
    feedback.phase = str(status.get('phase', ''))
    feedback.step_index = int(status.get('step', 0))
    feedback.step_total = int(status.get('step_total', 0))
    feedback.step_type = str(status.get('step_type') or '')
    total = max(feedback.step_total, 1)
    feedback.progress = float(min(max(feedback.step_index / total, 0.0), 1.0))
    altitude = (status.get('altitude_lcl') or {}).get('value_m')
    feedback.altitude_lcl_m = (float(altitude) if altitude is not None
                               else float('nan'))
    task_altitude = (status.get('task_altitude') or {}).get('value_m')
    feedback.target_altitude_m = (float(task_altitude)
                                  if task_altitude is not None else float('nan'))
    feedback.status = (f'{feedback.phase} {feedback.step_index + 1}/'
                       f'{total} {feedback.step_type}').strip()
    return feedback


class TaskActionServer(Node):
    def __init__(self):
        super().__init__('task_action_server')
        self._status = None
        self._status_at = 0.0
        self._active_goal = None
        self._cancel_sent = False
        self._group = ReentrantCallbackGroup()
        self._task_pub = self.create_publisher(String, '/mission/task', 10)
        self._cancel_pub = self.create_publisher(Empty, '/mission/cancel', 10)
        self.create_subscription(
            String, '/mission/status', self._on_status, 10,
            callback_group=self._group)
        self._server = ActionServer(
            self, ExecuteTask, '/mission/execute_task',
            execute_callback=self._execute_callback,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=self._group)

    def _on_status(self, msg: String) -> None:
        try:
            self._status = json.loads(msg.data)
            self._status_at = time.monotonic()
        except json.JSONDecodeError:
            pass

    def _executor_ready(self) -> bool:
        if self._status is None or time.monotonic() - self._status_at > 3.0:
            return False
        return self._status.get('phase') in ('IDLE', 'DONE')

    def _goal_callback(self, goal_request):
        try:
            TaskSpec.from_json(goal_request.task_json).validate()
        except (TaskError, ValueError, TypeError) as exc:
            self.get_logger().warn(f'拒绝 goal：TaskSpec 不合法（{exc}）')
            return GoalResponse.REJECT
        if self._active_goal is not None:
            self.get_logger().warn('拒绝 goal：已有任务在跑')
            return GoalResponse.REJECT
        if not self._executor_ready():
            self.get_logger().warn('拒绝 goal：执行器不在 IDLE/DONE（或状态过期）')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _cancel_callback(self, _goal_handle):
        return CancelResponse.ACCEPT

    def _execute_callback(self, goal_handle):
        result = ExecuteTask.Result()
        try:
            spec = TaskSpec.from_json(goal_handle.request.task_json)
            spec.validate()
        except (TaskError, ValueError, TypeError) as exc:
            result.success = False
            result.message = f'TaskSpec 不合法：{exc}'
            goal_handle.abort()
            return result

        self._active_goal = goal_handle
        self._cancel_sent = False
        self._task_pub.publish(String(data=goal_handle.request.task_json))
        deadline = time.monotonic() + max(60.0, float(spec.max_duration_s) + 30.0)
        last_feedback = 0.0
        self.get_logger().info(
            f'Action goal 已提交：{spec.summary()}（时限 {spec.max_duration_s:.0f}s）')

        while rclpy.ok() and time.monotonic() < deadline:
            if goal_handle.is_cancel_requested and not self._cancel_sent:
                self._cancel_sent = True
                self.get_logger().warn('收到 cancel → 通知执行器落地上锁')
                self._cancel_pub.publish(Empty())
            status = self._status
            now = time.monotonic()
            if status is not None and now - last_feedback >= 0.2:
                goal_handle.publish_feedback(status_to_feedback(status))
                last_feedback = now
            if status is not None and status.get('phase') == 'DONE':
                result.report_path = str(status.get('report_path') or '')
                result.findings_json = json.dumps(
                    status.get('findings') or [], ensure_ascii=False)
                if status.get('aborted'):
                    result.success = False
                    result.message = str(status.get('abort_reason') or 'aborted')
                    if goal_handle.is_cancel_requested:
                        goal_handle.canceled()
                    else:
                        goal_handle.abort()
                else:
                    result.success = True
                    result.message = 'done'
                    goal_handle.succeed()
                self._active_goal = None
                return result
            time.sleep(0.1)

        result.success = False
        result.message = 'timeout waiting for executor'
        goal_handle.abort()
        self._active_goal = None
        return result


def main(argv=None):
    rclpy.init(args=argv)
    node = TaskActionServer()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
