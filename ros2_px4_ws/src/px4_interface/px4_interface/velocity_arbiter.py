"""速度指令仲裁节点：把多个来源的速度指令按优先级合并成一路，喂给 offboard_bridge。

背景（服务器侧 2026-10-14 建议的 ②）：
`vlm_navigator`（慢速语义回路）与 `local_planner`（快速几何回路）**都**发布
`/offboard_bridge/velocity_setpoint`。两者若同时运行，会在同一个话题上互相覆盖
——"谁最后发谁生效"，既不可预测，也会让 offboard_bridge 的速度超时逻辑失效。

仲裁规则（本项目安全原则的落地：**实时避障不经过 VLM、安全层优先**）：

1. 每个来源都有各自的**新鲜度超时**；过期（`now - last > timeout`）视为"没有指令"。
2. 在**未过期**的来源里，按 `priority` 顺序取第一个有指令的来源作为输出。
3. 全部过期 → 不发布，交由 `offboard_bridge` 自身的速度超时保持位置
   （保守：宁可站住，也不把过期指令当成新指令）。

默认优先级 `avoid,vlm`：几何避障（`local_planner`）优先于语义导航（`vlm_navigator`）。
需要让 VLM 主控时，把 `priority` 设为 `vlm,avoid` 即可（例如只跑 VLM 闭环时）。

用法（不改动两个既有节点的默认话题，靠 remap 把它们的输出接到仲裁器输入）：

```text
ros2 run px4_interface velocity_arbiter --ros-args \
  -p priority:=avoid,vlm \
  -r /local_planner/velocity_setpoint:=/arbiter/avoid_cmd \
  ...
```

更省事的做法：把两个节点原本的 `velocity_topic` 参数分别指向仲裁器的两个输入，
再把仲裁器的 `output_topic` 指向 `/offboard_bridge/velocity_setpoint`。
"""

from __future__ import annotations

import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

# 每个来源的默认新鲜度超时（s）。取值参考各回路自身的发布周期：
# 几何回路 ~10 Hz（周期 0.1 s），语义回路 ~5 Hz（周期 0.2 s），留足丢帧余量。
DEFAULT_TIMEOUTS = {
    'avoid': 0.5,
    'vlm': 1.0,
}

ZERO_COMMAND = (0.0, 0.0, 0.0, 0.0)


def select_command(sources, now, *, priority, timeouts=None,
                   default_timeout_s: float = 0.5):
    """从多个来源里按优先级选一路未过期的速度指令（纯函数，便于单测）。

    参数
    ----
    sources : dict[str, tuple[tuple[float, float, float], float, float] | None]
        来源名 → ``((vx, vy, vz), yaw_rate, last_monotonic)``；``None`` 表示该来源从未发布过。
    now : float
        当前单调时钟（s）。
    priority : iterable[str]
        优先级从高到低排列的来源名。
    timeouts : dict[str, float] | None
        每个来源的新鲜度超时；缺省用 ``DEFAULT_TIMEOUTS``，仍未命中则该来源用 ``default_timeout_s``。

    返回
    ----
    tuple[str | None, tuple[float, float, float], float]
        ``(来源名, (vx, vy, vz), yaw_rate)``；全部过期时返回 ``(None, (0,0,0), 0.0)``。
    """
    resolved_timeouts = dict(DEFAULT_TIMEOUTS)
    if timeouts:
        resolved_timeouts.update(timeouts)

    for name in priority:
        entry = sources.get(name)
        if entry is None:
            continue
        velocity, yaw_rate, last = entry
        timeout = resolved_timeouts.get(name, default_timeout_s)
        if now - last <= timeout:
            return name, tuple(velocity), float(yaw_rate)

    return None, ZERO_COMMAND[:3], 0.0


class VelocityArbiter(Node):
    """订阅多个速度来源，按优先级把选中的一路以固定频率发到输出话题。"""

    def __init__(self) -> None:
        super().__init__('velocity_arbiter')
        self.declare_parameter('priority', 'avoid,vlm')
        self.declare_parameter('avoid_topic', '/local_planner/velocity_cmd')
        self.declare_parameter('vlm_topic', '/vlm_navigator/velocity_cmd')
        self.declare_parameter('output_topic', '/offboard_bridge/velocity_setpoint')
        self.declare_parameter('avoid_timeout_s', DEFAULT_TIMEOUTS['avoid'])
        self.declare_parameter('vlm_timeout_s', DEFAULT_TIMEOUTS['vlm'])
        self.declare_parameter('publish_rate_hz', 10.0)

        self._priority = [
            name.strip() for name in
            str(self.get_parameter('priority').value).split(',') if name.strip()
        ]
        self._timeouts = {
            'avoid': float(self.get_parameter('avoid_timeout_s').value),
            'vlm': float(self.get_parameter('vlm_timeout_s').value),
        }
        output_topic = str(self.get_parameter('output_topic').value)
        rate_hz = max(float(self.get_parameter('publish_rate_hz').value), 1.0)

        self._sources: dict = {}
        self.create_subscription(
            Twist, str(self.get_parameter('avoid_topic').value),
            lambda msg: self._on_cmd('avoid', msg), 10)
        self.create_subscription(
            Twist, str(self.get_parameter('vlm_topic').value),
            lambda msg: self._on_cmd('vlm', msg), 10)

        self._cmd_pub = self.create_publisher(Twist, output_topic, 10)
        self._active = None
        self.create_timer(1.0 / rate_hz, self._on_timer)

        self.get_logger().info(
            f'velocity_arbiter 启动：优先级 {self._priority}，'
            f'超时 {self._timeouts}，输出 {output_topic} @ {rate_hz:.1f} Hz')

    def _on_cmd(self, name: str, msg: Twist) -> None:
        velocity = (float(msg.linear.x), float(msg.linear.y), float(msg.linear.z))
        self._sources[name] = (velocity, float(msg.angular.z), time.monotonic())

    def _on_timer(self) -> None:
        now = time.monotonic()
        name, velocity, yaw_rate = select_command(
            self._sources, now, priority=self._priority, timeouts=self._timeouts)

        if name is None:
            # 全部过期：不发布，交由 offboard_bridge 的速度超时保持位置。
            if self._active is not None:
                self.get_logger().warn(
                    '所有速度来源均已过期 → 停止发布，交由 offboard_bridge 保持位置',
                    throttle_duration_sec=5.0)
            self._active = None
            return

        if name != self._active:
            self.get_logger().info(f'速度控制权 → {name}')
            self._active = name

        msg = Twist()
        msg.linear.x, msg.linear.y, msg.linear.z = (float(v) for v in velocity)
        msg.angular.z = yaw_rate
        self._cmd_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VelocityArbiter()
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
