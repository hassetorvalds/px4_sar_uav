#!/usr/bin/env python3
"""C3 注入式验证：向 VLM 的地图安全层喂可控的三向净空（不依赖真实场景）。

为什么用独立话题：真实的 `local_map` 一直在发布 `/local_map/status`，
直接往那个话题注入会被真实值覆盖。所以注入测试时把 VLM 节点指向
`-p map_status_topic:=/inject/status`，本脚本只发这一路。

场景（服务器侧 2026-10-12 要求覆盖的三个确定性用例都在里面）：

```text
0–8 s     不发布任何净空           → 期望 guard='stale'（fail-safe：前向压到 0.1 m/s）
8–18 s    f=3.0 / l=3.0 / r=3.0   → 期望 guard='ok'（恢复放行）
18–26 s   f=0.9 / l=2.0 / r=2.0   → 期望 guard='obstacle_slow'（压速 50%）
26–36 s   f=0.4 / l=3.0 / r=1.0   → 期望 guard='obstacle' 且**左转**（左更宽）
36–46 s   f=0.4 / l=1.0 / r=3.0   → 期望 guard='obstacle' 且**右转**（右更宽）
46–52 s  不再发布                  → 期望回到 guard='stale'
```

用法：
    python3 scripts/inject_clearance.py --ros-args -p topic:=/inject/status
"""

import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# (时长 s, (前, 左, 右) 或 None=不发布)
# demo：五态逐段验证（陈旧 / 通行 / 压速 / 左绕 / 右绕）
SCENARIO_DEMO = [
    (8.0, None),
    (10.0, (3.0, 3.0, 3.0)),
    (8.0, (0.9, 2.0, 2.0)),
    (10.0, (0.4, 3.0, 1.0)),
    (10.0, (0.4, 1.0, 3.0)),
    (6.0, None),
]

# turn：验证服务器侧 2026-10-12 追加的三条防死角规则
SCENARIO_TURN = [
    (6.0, None),                 # 陈旧 → stale
    (10.0, (3.0, 3.0, 3.0)),     # 放行 → ok
    (22.0, (0.4, 3.0, 1.0)),     # 障碍：先左转+小前向；累积到 60° 后应停转（yaw=0）
    (10.0, (0.95, 1.0, 1.0)),    # 前向恢复但侧向仍紧 → 迟滞：继续保持
    (10.0, (0.95, 2.0, 2.0)),    # 前向+侧向都恢复 → 释放（转 obstacle_slow）
    (6.0, None),                 # 陈旧 → stale
]

SCENARIOS = {'demo': SCENARIO_DEMO, 'turn': SCENARIO_TURN}


class ClearanceInjector(Node):
    def __init__(self) -> None:
        super().__init__('clearance_injector')
        self.declare_parameter('topic', '/inject/status')
        self.declare_parameter('rate_hz', 5.0)
        self.declare_parameter('scenario', 'demo')
        self.scenario_name = str(self.get_parameter('scenario').value)
        if self.scenario_name not in SCENARIOS:
            raise ValueError(f'未知场景 {self.scenario_name}，可选 {list(SCENARIOS)}')
        self.scenario = SCENARIOS[self.scenario_name]
        self.publisher = self.create_publisher(
            String, str(self.get_parameter('topic').value), 10)
        self.index = 0
        self.step_started = time.monotonic()
        self._last_log = 0.0
        self.timer = self.create_timer(
            1.0 / max(float(self.get_parameter('rate_hz').value), 0.5), self._tick)
        self.get_logger().info(
            f'注入场景 {self.scenario_name} 开始：共 {len(self.scenario)} 段、'
            f'约 {sum(s for s, _ in self.scenario):.0f} s'
            f'（话题 {self.get_parameter("topic").value}）')

    def _tick(self) -> None:
        if self.index >= len(self.scenario):
            self.get_logger().info('注入场景结束（此后不再发布 → 应回到 guard=stale）')
            raise SystemExit(0)
        duration, values = self.scenario[self.index]
        elapsed = time.monotonic() - self.step_started
        if elapsed > duration:
            self.index += 1
            self.step_started = time.monotonic()
            return
        label = '不发布（陈旧）' if values is None else f'f={values[0]} l={values[1]} r={values[2]}'
        if elapsed - self._last_log >= 2.0:
            self._last_log = elapsed
            self.get_logger().info(
                f'第 {self.index + 1} 段 {elapsed:.0f}/{duration:.0f} s：{label}')
        if values is None:
            return
        msg = String()
        msg.data = (f'voxels=0 frames=0 nearest_forward_m={values[0]:.2f} '
                    f'left_m={values[1]:.2f} right_m={values[2]:.2f}')
        self.publisher.publish(msg)


def main() -> int:
    rclpy.init()
    node = ClearanceInjector()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
