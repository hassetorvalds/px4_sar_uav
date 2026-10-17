#!/usr/bin/env python3
"""独立落地监督进程（服务器侧 2026-10-13 的安全建议 ②）。

背景：我们把 `land_fast` 写在驱动脚本的收尾里，结果**脚本自己崩溃时保护也一起没了**
（2026-10-13 实测：判别脚本汇总时抛异常，崩在自动降落之前，飞机没有自动落地）。
服务器侧建议把"落地保护"与"飞控/分析脚本"**解耦**，本进程就是那一层：

```text
谁在飞         驱动脚本（vlm_closed_loop.py / map_selfcheck.py）按 2 Hz 发心跳 /flight_heartbeat
监督什么       只要"已解锁"且心跳超过 timeout_s 没更新 → 立即下发 land_fast（并可重复）
独立性         与驱动脚本不同进程；驱动崩了、被 kill、卡死都能兜住
边界           `kill -9` 本进程 / 整机断电这类情况兜不住——那一层交给 PX4 的 failsafe
```

用法（起飞前启动，随栈常驻）：
    python3 scripts/flight_supervisor.py --ros-args -p timeout_s:=5.0
"""

import sys
import time

import rclpy
from px4_msgs.msg import VehicleStatus
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import String

QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)

sys.stdout.reconfigure(line_buffering=True)


def parse_phase_budgets(text: str) -> dict:
    """解析 "ALIGN:15|SEEK:45" → {"ALIGN": 15.0, "SEEK": 45.0}（纯函数，便于单测）。"""
    budgets = {}
    for chunk in (text or '').split('|'):
        chunk = chunk.strip()
        if not chunk or ':' not in chunk:
            continue
        name, _, value = chunk.partition(':')
        name = name.strip().upper()
        try:
            budgets[name] = float(value)
        except ValueError:
            continue
    return budgets


class FlightSupervisor(Node):
    def __init__(self) -> None:
        super().__init__('flight_supervisor')
        self.declare_parameter('heartbeat_topic', '/flight_heartbeat')
        self.declare_parameter('command_topic', '/offboard_bridge/command')
        self.declare_parameter('status_topic', '/fmu/out/vehicle_status_v4')
        self.declare_parameter('timeout_s', 5.0)
        self.declare_parameter('repeat_s', 3.0)
        # 解锁后的宽限期：驱动脚本要过约 1 s 才发布第一条心跳，
        # 若不给宽限，会拿"上一轮遗留的旧心跳时间"直接判超时并在起飞瞬间发 land_fast
        # （2026-10-13 实测：c16 起飞时心跳被判定"已停 712.9 s"→ 桥接切速度模式下降 → 爬升超时）
        self.declare_parameter('arm_grace_s', 6.0)
        # 卡阶段兜底：心跳里带阶段名（"ALIGN t=658.4"），同一阶段持续过久即降落。
        # 背景（2026-10-14）：驱动因缩进 bug 卡在 ALIGN 658 s，但心跳一直在发 →
        # "心跳停"判据兜不住；阶段不再变化才是那种情形的信号。
        self.declare_parameter('phase_stuck_s', 180.0)
        # 按阶段给"卡死预算"（服务器侧 2026-10-14 建议）：统一 180 s 对 ALIGN 太松、
        # 对长扫描又可能偏紧。格式 "PHASE:秒|PHASE:秒"，未列出的阶段用 phase_stuck_s。
        self.declare_parameter(
            'phase_budgets',
            # SCAN 预算 35→120 s：驱动"每点 25 s × 3 个候选点"本身就要 ~75 s（还有复验往返），
            # 35 s 会把**正常扫描**误判成卡死（2026-10-15 d18 实测）。120 s 仍是"卡死兜底"
            # （驱动侧 scan_max_s 保证单点 25 s 必结束，不会无限扫）。
            # TRANSIT 90→150 s：驱动"每段 45 s"、受阻时会换点继续，而监督层是**按同名阶段累计**的
            # （换点后名字仍是 TRANSIT）→ 3 段就可能到 135 s，90 s 会误杀（2026-10-16 k5 实测）。
            # 仍是卡死兜底：驱动单段 45 s < 监督层 150 s < max_flight_s 600 s（嵌套成立）。
            'CLIMB:30|SEEK:45|TRANSIT:150|DESCEND:30|SCAN:120|VERIFY:15'
            '|ALIGN:15|APPROACH:45|RUN:200')

        self.timeout_s = float(self.get_parameter('timeout_s').value)
        self.repeat_s = float(self.get_parameter('repeat_s').value)
        self.arm_grace_s = float(self.get_parameter('arm_grace_s').value)
        self.phase_stuck_s = float(self.get_parameter('phase_stuck_s').value)
        self.phase_budgets = parse_phase_budgets(
            str(self.get_parameter('phase_budgets').value))
        self.current_phase = None
        self.phase_since = None
        self.last_heartbeat = None
        self.armed = False
        self.armed_since = None
        self.last_land_sent = 0.0

        self.command_pub = self.create_publisher(
            String, str(self.get_parameter('command_topic').value), 10)
        self.create_subscription(
            String, str(self.get_parameter('heartbeat_topic').value),
            self._on_heartbeat, 10)
        self.create_subscription(
            VehicleStatus, str(self.get_parameter('status_topic').value),
            self._on_status, QOS)
        self.create_timer(1.0, self._tick)
        self.get_logger().info(
            f'落地监督启动：心跳 {self.get_parameter("heartbeat_topic").value}，'
            f'超过 {self.timeout_s:.0f} s 未更新且已解锁 → 下发 land_fast')

    def _on_heartbeat(self, msg: String) -> None:
        self.last_heartbeat = time.monotonic()
        phase = str(msg.data).split(' ')[0].strip() or None
        if phase != self.current_phase:
            self.current_phase = phase
            self.phase_since = self.last_heartbeat

    def _on_status(self, msg: VehicleStatus) -> None:
        self.armed = msg.arming_state == VehicleStatus.ARMING_STATE_ARMED

    def _tick(self) -> None:
        if not self.armed:
            self.armed_since = None
            return
        now = time.monotonic()
        if self.armed_since is None:
            self.armed_since = now
            self.get_logger().info(
                f'检测到解锁：给驱动 {self.arm_grace_s:.0f} s 宽限期再开始判心跳')
        if now - self.armed_since < self.arm_grace_s:
            return                      # 宽限期内不判（避免拿旧心跳时间误触发）
        # 卡阶段兜底
        budget = self.phase_budgets.get(self.current_phase, self.phase_stuck_s)
        if (self.phase_since is not None and now - self.phase_since > budget):
            self._send_land('阶段 {0} 已持续 {1:.0f} s（> {2:.0f} s）'.format(
                self.current_phase, now - self.phase_since, budget))
            return
        # 宽限期结束时若还没有任何心跳，就从"本进程启动"算起，而不是沿用上一轮的旧时间
        baseline = self.last_heartbeat if self.last_heartbeat is not None \
            else self.armed_since
        age = now - max(baseline, self.armed_since - self.arm_grace_s)
        if age <= self.timeout_s:
            return
        if now - self.last_land_sent < self.repeat_s:
            return
        self.last_land_sent = now
        self._send_land(f'心跳已停 {age:.1f} s（> {self.timeout_s:.0f} s）且飞机已解锁')

    def _send_land(self, reason: str) -> None:
        now = time.monotonic()
        if now - self.last_land_sent < self.repeat_s:
            return
        self.last_land_sent = now
        msg = String()
        msg.data = 'land_fast'
        self.command_pub.publish(msg)
        self.get_logger().error(f'监督层下发 land_fast：{reason}')


def main() -> int:
    rclpy.init()
    node = FlightSupervisor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
