"""任务执行节点：把 :class:`TaskSpec` 变成安全的飞行动作。

设计边界（与 README 的分层一致）：

- **只认识 TaskSpec/Step**，不认识自然语言；解析在 ``mission.task_parser``。
- **不直接碰 PX4**：所有指令经 ``offboard_bridge``（arm/offboard/land_fast/target/velocity）。
- **安全优先**：围栏、前向净空门、任务总时限、状态陈旧、卡阶段超时都会收工降落；
  默认 ``dry_run=true``，必须显式 ``dry_run:=false`` 才会解锁。
- **只做低速任务**：速度上限来自 TaskSpec（默认 0.5 m/s），高度保持交给位置模式。

用法::

    ros2 run mission task_executor --ros-args \
        -p task_file:=/tmp/task.json -p dry_run:=false

    # 或直接给 JSON
    ros2 run mission task_executor --ros-args \
        -p task_json:='{"version":1,...}' -p dry_run:=false
"""

import json
import math
import os
import re
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from px4_interface.frames import ned_to_enu, ned_yaw_to_enu_yaw
from px4_interface.qos import px4_qos
from px4_interface.state import Px4StateMonitor
from px4_msgs.msg import BatteryStatus, VehicleCommand, VehicleCommandAck
from rclpy.node import Node
from std_msgs.msg import Bool, Empty, String

from .altitude import format_altitude
from .altitude import to_dict as altitude_dict
from .command_tracker import CommandState, CommandTracker
from .executor_core import (altitude_hold_vz, apply_clearance_gate, forward_target,
                            inside_geofence,
                            search_waypoints, step_timeout_s, velocity_toward,
                            yaw_rate_to_target)
from .findings import FindingsBook, priority_for_kind
from .findings_store import persist_findings
from .health import check_prearm
from .search_pattern import estimate_duration_s
from .task_spec import TaskError, TaskSpec

_CLEARANCE_RE = re.compile(r'nearest_forward_m=([0-9.]+|inf)')

#: 命令名 → (PX4 vehicle_command, 是否期望 ACK)
#: ``land_fast`` 由 bridge 用速度指令实现，没有单条 MAVLink ACK，靠"是否上锁"判完成。
_COMMAND_IDS = {
    'arm': (VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, True),
    'disarm': (VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, True),
    'disarm_force': (VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, True),
    'offboard': (VehicleCommand.VEHICLE_CMD_DO_SET_MODE, True),
    'land': (VehicleCommand.VEHICLE_CMD_NAV_LAND, True),
    'land_fast': (None, False),
    'hold': (None, False),
}

PHASE_IDLE = 'IDLE'
PHASE_ARM = 'ARM'
PHASE_OFFBOARD = 'OFFBOARD'
PHASE_RUN = 'RUN'
PHASE_PAUSED = 'PAUSED'
PHASE_LAND = 'LAND'
PHASE_DONE = 'DONE'


class TaskExecutor(Node):
    def __init__(self) -> None:
        super().__init__('task_executor')
        self.declare_parameter('task_file', '')
        self.declare_parameter('task_json', '')
        self.declare_parameter('intent_path', '')
        self.declare_parameter('task_topic', '/mission/task')
        self.declare_parameter('status_topic', '/mission/status')
        self.declare_parameter('abort_topic', '~/abort')
        self.declare_parameter('command_topic', '/offboard_bridge/command')
        self.declare_parameter('target_topic', '/offboard_bridge/target_pose')
        self.declare_parameter('velocity_topic', '/offboard_bridge/velocity_setpoint')
        self.declare_parameter('heartbeat_topic', '/flight_heartbeat')
        self.declare_parameter('map_status_topic', '/local_map/status')
        self.declare_parameter('best_topic', '/perception/best')
        self.declare_parameter('report_path', '')
        self.declare_parameter('dry_run', True)
        self.declare_parameter('hold_on_target', True)
        self.declare_parameter('hold_once_per_step', True)
        self.declare_parameter('center_on_target', True)
        self.declare_parameter('center_tolerance_deg', 5.0)
        self.declare_parameter('center_yaw_rate_deg_s', 15.0)
        self.declare_parameter('findings_merge_radius_m', 3.0)
        self.declare_parameter('findings_dir', 'logs/findings')
        self.declare_parameter('persist_findings', True)
        self.declare_parameter('target_hold_s', 5.0)
        self.declare_parameter('land_on_target', False)
        self.declare_parameter('clearance_stop_m', 0.6)
        self.declare_parameter('arrive_tol_m', 0.4)
        self.declare_parameter('takeoff_tol_m', 0.25)
        self.declare_parameter('arm_timeout_s', 15.0)
        self.declare_parameter('arm_settle_s', 0.5)
        self.declare_parameter('offboard_timeout_s', 15.0)
        self.declare_parameter('land_timeout_s', 25.0)
        self.declare_parameter('blocked_abort_s', 10.0)
        self.declare_parameter('status_timeout_s', 2.0)
        self.declare_parameter('heartbeat_s', 0.5)
        self.declare_parameter('exit_on_done', True)
        self.declare_parameter('ack_topic', '/fmu/out/vehicle_command_ack_v1')
        self.declare_parameter('ack_timeout_s', 1.5)
        self.declare_parameter('ack_max_attempts', 3)
        self.declare_parameter('min_battery_ratio', 0.3)
        self.declare_parameter('require_battery', False)
        self.declare_parameter('max_position_age_s', 0.5)

        self._dry_run = bool(self.get_parameter('dry_run').value)
        self._hold_on_target = bool(self.get_parameter('hold_on_target').value)
        self._hold_once_per_step = bool(self.get_parameter('hold_once_per_step').value)
        self._center_on_target = bool(self.get_parameter('center_on_target').value)
        self._center_tolerance = math.radians(float(
            self.get_parameter('center_tolerance_deg').value))
        self._center_yaw_rate = math.radians(float(
            self.get_parameter('center_yaw_rate_deg_s').value))
        self._findings_radius = float(
            self.get_parameter('findings_merge_radius_m').value)
        self._findings_dir = str(self.get_parameter('findings_dir').value)
        self._persist_findings = bool(
            self.get_parameter('persist_findings').value)
        self._findings = FindingsBook(self._findings_radius)
        self._land_on_target = bool(self.get_parameter('land_on_target').value)
        self._target_hold_s = float(self.get_parameter('target_hold_s').value)
        self._clearance_stop = float(self.get_parameter('clearance_stop_m').value)
        self._arrive_tol = float(self.get_parameter('arrive_tol_m').value)
        self._takeoff_tol = float(self.get_parameter('takeoff_tol_m').value)
        self._arm_timeout = float(self.get_parameter('arm_timeout_s').value)
        self._arm_settle = float(self.get_parameter('arm_settle_s').value)
        self._offboard_timeout = float(self.get_parameter('offboard_timeout_s').value)
        self._land_timeout = float(self.get_parameter('land_timeout_s').value)
        self._blocked_abort = float(self.get_parameter('blocked_abort_s').value)
        self._status_timeout = float(self.get_parameter('status_timeout_s').value)
        self._heartbeat_s = float(self.get_parameter('heartbeat_s').value)
        self._exit_on_done = bool(self.get_parameter('exit_on_done').value)
        self._ack_timeout = float(self.get_parameter('ack_timeout_s').value)
        self._ack_max_attempts = max(int(self.get_parameter('ack_max_attempts').value), 1)
        self._min_battery = float(self.get_parameter('min_battery_ratio').value)
        self._require_battery = bool(self.get_parameter('require_battery').value)
        self._max_position_age = float(self.get_parameter('max_position_age_s').value)
        self._battery_ratio = None
        self._battery_warning = None
        self._battery_at = 0.0
        self._last_health = None
        self._last_health_at = 0.0
        self._tracker = CommandTracker(ack_timeout_s=self._ack_timeout,
                                       max_attempts=self._ack_max_attempts)
        self._ground_up_lcl = None       # 起飞点在地面时的 PX4 local up（LCL 帧）

        self._monitor = Px4StateMonitor(self)
        self._clock_started = time.monotonic()
        self._phase = PHASE_IDLE
        self._phase_started = time.monotonic()
        self._spec = None
        self._task_started = None
        self._step_index = 0
        self._step_started = time.monotonic()
        self._step_target = None
        self._yaw_target_enu = None
        self._scan_phase = 'dwell'
        self._scan_target_enu = None
        self._scan_dwell_until = 0.0
        self._search_wps = []
        self._search_index = 0
        self._takeoff_stable_at = None
        self._blocked_since = None
        self._target_hold_until = 0.0
        self._last_target_report_at = 0.0
        self._target_hold_count = 0
        self._target_world = None
        self._target_centered_reported = False
        self._findings = FindingsBook(self._findings_radius)
        self._manual_override = False
        self._last_target_pose = None
        self._origin_enu = (0.0, 0.0, 0.0)
        self._ever_armed = False
        self._land_command_sent = False
        self._abort_reason = None
        self._clearance_forward = None
        self._heading_enu = None
        self._last_heartbeat = 0.0
        self._last_status = 0.0
        self._report_handle = None
        self._report_path = str(self.get_parameter('report_path').value)

        self._command_pub = self.create_publisher(
            String, str(self.get_parameter('command_topic').value), 10)
        self._target_pub = self.create_publisher(
            PoseStamped, str(self.get_parameter('target_topic').value), 10)
        self._velocity_pub = self.create_publisher(
            Twist, str(self.get_parameter('velocity_topic').value), 10)
        self._heartbeat_pub = self.create_publisher(
            String, str(self.get_parameter('heartbeat_topic').value), 10)
        self._status_pub = self.create_publisher(
            String, str(self.get_parameter('status_topic').value), 10)
        self.create_subscription(
            String, str(self.get_parameter('task_topic').value), self._on_task, 10)
        self.create_subscription(
            Bool, str(self.get_parameter('abort_topic').value), self._on_abort, 10)
        self.create_subscription(Empty, '/mission/cancel', self._on_cancel, 10)
        self.create_subscription(
            String, str(self.get_parameter('map_status_topic').value),
            self._on_map_status, 10)
        self.create_subscription(
            PoseStamped, str(self.get_parameter('best_topic').value),
            self._on_best, 10)
        # PX4 命令 ACK：命令生命周期跟踪（发送 ≠ 完成）
        self.create_subscription(
            VehicleCommandAck, str(self.get_parameter('ack_topic').value),
            self._on_command_ack, px4_qos())
        self.create_subscription(
            BatteryStatus, '/fmu/out/battery_status_v1', self._on_battery, px4_qos())

        self._open_report()
        self._load_task()
        self.create_timer(0.1, self._tick)
        self.get_logger().info(
            f'任务执行器就绪：dry_run={self._dry_run}，'
            f'任务={"有" if self._spec else "无（等待 ~/task）"}')

    # ------------------------------------------------------------------
    # 任务装载 / 报告
    # ------------------------------------------------------------------

    def _open_report(self) -> None:
        path = self._report_path
        if not path:
            stamp = time.strftime('%Y%m%d-%H%M%S')
            path = os.path.join('logs', 'tasks', f'task_{stamp}.jsonl')
        directory = os.path.dirname(path)
        if directory:
            os.makedirs(directory, exist_ok=True)
        self._report_path = path
        self._report_handle = open(path, 'a', encoding='utf-8')

    def _report(self, event: str, **fields) -> None:
        if self._report_handle is None:
            return
        state = self._monitor.snapshot()
        position = (None if state.position_ned is None
                    else ned_to_enu(*state.position_ned))
        entry = {
            't': round(time.monotonic() - self._clock_started, 2),
            'event': event,
            'phase': self._phase,
            'step': self._step_index,
            'armed': bool(state.armed),
            'nav_state': int(state.nav_state),
            'position_enu': (None if position is None
                             else [round(float(v), 2) for v in position]),
            # 高度必须带帧：LCL 是 PX4 local，可能漂移；任务高度是 REL
            'altitude_lcl': altitude_dict(None if position is None else position[2],
                                          'LCL', 'px4_local'),
            **{key: value for key, value in fields.items()},
        }
        if self._spec is not None:
            entry['task_altitude'] = altitude_dict(
                self._spec.altitude_m, self._spec.altitude_frame, 'task')
        self._report_handle.write(json.dumps(entry, ensure_ascii=False) + '\n')
        self._report_handle.flush()

    def _now_t(self) -> float:
        return round(time.monotonic() - self._clock_started, 2)

    def _target_kind(self) -> str:
        if self._spec is not None:
            for step in self._spec.steps:
                if step.type == 'search':
                    target = step.params.get('target') or {}
                    return str(target.get('kind') or 'target')
        return 'target'

    def _on_battery(self, msg: BatteryStatus) -> None:
        remaining = float(msg.remaining)
        self._battery_ratio = remaining if remaining >= 0.0 else None
        self._battery_warning = int(msg.warning)
        self._battery_at = time.monotonic()

    def _prearm_report(self):
        state = self._monitor.snapshot()
        battery_age = (None if self._battery_at <= 0.0
                       else time.monotonic() - self._battery_at)
        return check_prearm(
            status_fresh=self._monitor.status_fresh(),
            status_age_s=self._monitor.status_age_s(),
            position_valid=state.position_ned is not None,
            position_age_s=self._monitor.position_age_s(),
            preflight_ok=state.pre_flight_checks_pass,
            failsafe=state.failsafe,
            battery_ratio=self._battery_ratio,
            battery_warning=self._battery_warning,
            battery_age_s=battery_age,
            min_battery_ratio=self._min_battery,
            require_battery=self._require_battery,
            max_position_age_s=self._max_position_age)

    def _publish_health(self, report, now: float) -> None:
        payload = report.to_dict()
        if payload != self._last_health or now - self._last_health_at > 1.0:
            self._last_health = payload
            self._last_health_at = now
            self._report('health_check', **payload)
            if not report.ok:
                self.get_logger().warn(
                    '起飞前检查未通过：' + '；'.join(report.reasons()),
                    throttle_duration_sec=1.0)

    def _on_command_ack(self, msg: VehicleCommandAck) -> None:
        record = self._tracker.on_ack(msg.command, msg.result)
        if record is None:
            return
        self._report('command_ack', name=record.name,
                     vehicle_command=int(msg.command), result=int(msg.result),
                     result_code=int(msg.result), state=record.state,
                     latency_s=record.latency_s, attempts=record.attempts)
        self.get_logger().info(
            f'命令 ACK：{record.name} result={int(msg.result)} → {record.state}'
            f'（{record.latency_s} s）')
        if (record.name in ('arm', 'offboard')
                and record.state == CommandState.REJECTED.value):
            self._abort(f'{record.name} 被拒绝（result={msg.result}）')

    def _track_commands(self, now: float) -> None:
        """推进命令生命周期：超时重试、状态确认完成、失败收工。"""
        for record in self._tracker.tick(now):
            self._report('command_timeout', name=record.name, state=record.state,
                         attempts=record.attempts, reason=record.reason)
            self.get_logger().warn(
                f'命令 {record.name} → {record.state}（attempts={record.attempts}）')
            if (record.name in ('arm', 'offboard')
                    and record.state == CommandState.DELIVERY_UNKNOWN.value):
                self._abort(f'{record.name} ACK 超时（重试 {record.attempts} 次）')
                return

        state = self._monitor.snapshot()
        for name, active in (('arm', state.armed), ('offboard', state.offboard)):
            record = self._tracker.get(name)
            if (active and record is not None
                    and record.state != CommandState.COMPLETED.value):
                self._tracker.mark_completed(name, 'state_confirmed')
                self._report('command_completed', name=name,
                             attempts=record.attempts, latency_s=record.latency_s)

        if self._ever_armed and not state.armed and self._phase == PHASE_LAND:
            record = self._tracker.get('land_fast')
            if record is not None and record.state != CommandState.COMPLETED.value:
                self._tracker.mark_completed('land_fast', 'disarmed')
                self._report('command_completed', name='land_fast',
                             state=CommandState.COMPLETED.value)

    def _load_task(self) -> None:
        task_json = str(self.get_parameter('task_json').value).strip()
        task_file = str(self.get_parameter('task_file').value).strip()
        if not task_json and task_file:
            try:
                with open(task_file, encoding='utf-8') as handle:
                    task_json = handle.read()
            except OSError as exc:
                self.get_logger().error(f'读取任务文件失败：{exc}')
                return
        if not task_json:
            return
        self._accept_task_json(task_json)

    def _accept_task_json(self, text: str) -> None:
        try:
            spec = TaskSpec.from_json(text)
        except TaskError as exc:
            self.get_logger().error(f'任务被拒绝：{exc}')
            self._report('task_rejected', reason=str(exc))
            return
        self._spec = spec
        self._reset_task_state()
        self._report('task_accepted', summary=spec.summary(),
                     spec=spec.to_dict(),
                     intent_path=str(self.get_parameter('intent_path').value))
        self.get_logger().info(f'接受任务：{spec.summary()}')
        if self._dry_run:
            self.get_logger().warn(
                'dry_run=true：只校验/打印任务，不解锁。要执行请设 dry_run:=false')
        elif self._phase == PHASE_IDLE:
            self._enter(PHASE_ARM)

    def _reset_task_state(self) -> None:
        self._phase = PHASE_IDLE
        self._phase_started = time.monotonic()
        self._task_started = None
        self._step_index = 0
        self._step_target = None
        self._yaw_target_enu = None
        self._scan_phase = 'dwell'
        self._scan_target_enu = None
        self._scan_dwell_until = 0.0
        self._search_wps = []
        self._search_index = 0
        self._takeoff_stable_at = None
        self._blocked_since = None
        self._target_hold_until = 0.0
        self._target_hold_count = 0
        self._target_world = None
        self._target_centered_reported = False
        self._findings = FindingsBook(self._findings_radius)
        self._land_command_sent = False
        self._abort_reason = None
        self._manual_override = False
        self._ground_up_lcl = None
        self._tracker = CommandTracker(ack_timeout_s=self._ack_timeout,
                                       max_attempts=self._ack_max_attempts)

    def _on_cancel(self, _msg: Empty) -> None:
        if self._phase in (PHASE_ARM, PHASE_OFFBOARD, PHASE_RUN, PHASE_PAUSED):
            self._abort('cancel_requested')

    def _on_task(self, msg: String) -> None:
        if self._phase not in (PHASE_IDLE, PHASE_DONE):
            self.get_logger().warn('当前任务未结束，忽略新任务（先 abort/降落）')
            return
        self._accept_task_json(msg.data)

    def _on_abort(self, msg: Bool) -> None:
        if msg.data:
            self.get_logger().error('收到 abort：立即收工降落')
            self._report('abort')
            self._begin_land()

    def _on_map_status(self, msg: String) -> None:
        match = _CLEARANCE_RE.search(msg.data)
        if not match:
            return
        value = match.group(1)
        self._clearance_forward = float('inf') if value == 'inf' else float(value)

    def _on_best(self, msg: PoseStamped) -> None:
        if not self._hold_on_target or self._phase != PHASE_RUN:
            return
        now = time.monotonic()
        if self._hold_once_per_step and self._target_hold_count >= 1:
            return
        # 去抖：同一目标持续可见时不要把搜索无限期拖住
        if now - self._last_target_report_at < 5.0:
            return
        self._target_hold_count += 1
        self._last_target_report_at = now
        world = [round(float(msg.pose.position.x), 2),
                 round(float(msg.pose.position.y), 2),
                 round(float(msg.pose.position.z), 2)]
        self.get_logger().info(f'确认目标：world={world} → 悬停 {self._target_hold_s:.0f} s')
        self._target_world = [float(v) for v in world]
        self._target_centered_reported = False
        kind = self._target_kind()
        label = 'target' if kind in ('', 'any', 'target') else kind
        finding = self._findings.add_or_update(
            label=label, world_enu=world, confidence=1.0,
            priority=priority_for_kind(kind), source='perception',
            t=self._now_t(), step_type=self._current_step_type(),
            status='confirmed')
        self._report('target_confirmed', world_enu=world,
                     step_type=self._current_step_type(),
                     centering=bool(self._center_on_target),
                     finding_id=finding.finding_id)
        self._report('finding', action='confirmed', **finding.to_dict())
        self._target_hold_until = now + self._target_hold_s
        if self._land_on_target:
            self._begin_land()

    # ------------------------------------------------------------------
    # 状态机
    # ------------------------------------------------------------------

    def _enter(self, phase: str) -> None:
        self._phase = phase
        self._phase_started = time.monotonic()
        self.get_logger().info(f'→ 阶段 {phase}')
        self._report('phase', to=phase)
        if phase == PHASE_DONE and self._exit_on_done:
            # 一次性任务：上锁后直接退出，避免 CLI/脚本一直挂着等超时
            self.get_logger().info('任务完成，执行器退出（exit_on_done=true）')
            if self._report_handle is not None:
                self._report_handle.close()
                self._report_handle = None
            raise SystemExit(0)

    def _phase_elapsed(self) -> float:
        return time.monotonic() - self._phase_started

    def _position_enu(self):
        state = self._monitor.snapshot()
        if state.position_ned is None:
            return None
        return ned_to_enu(*state.position_ned)

    def _tick(self) -> None:
        now = time.monotonic()
        self._update_heading()
        self._update_heartbeat(now)
        self._publish_status()
        self._track_commands(now)
        state = self._monitor.snapshot()
        if state.armed:
            self._ever_armed = True

        if self._phase == PHASE_IDLE:
            return
        if self._phase == PHASE_DONE:
            return

        # 安全闸：状态陈旧 / 越界 / 总时限
        # 启动阶段还没收到过 PX4 状态（age=inf）不算"陈旧"：ARM/OFFBOARD 阶段各有超时兜底，
        # 这里不要写 abort（否则报告里会出现假的 abort，2026-10-18 实飞踩到）。
        status_age = self._monitor.status_age_s()
        waiting_first_status = (math.isinf(status_age)
                                and self._phase in (PHASE_ARM, PHASE_OFFBOARD))
        if not waiting_first_status and status_age > self._status_timeout:
            self.get_logger().error('PX4 状态陈旧 → 停止发指令')
            self._report('abort', reason='stale_px4')
            self._stop_velocity()
            return

        # 人工接管：非 Offboard 时**停止自主输出**，交还后自动恢复
        if self._phase == PHASE_RUN and state.armed and not state.offboard:
            if not self._manual_override:
                self._manual_override = True
                self._stop_velocity()
                self._report('manual_override')
                self.get_logger().warn('检测到人工接管（非 Offboard）→ 暂停自主输出')
                self._enter(PHASE_PAUSED)
            return
        if self._phase == PHASE_PAUSED:
            if not state.armed:
                self._enter(PHASE_DONE)
            elif state.offboard:
                self._manual_override = False
                self._report('manual_resume')
                self._enter(PHASE_RUN)
            return

        if self._phase in (PHASE_RUN, PHASE_LAND) and self._position_enu() is not None:
            if not inside_geofence(self._position_enu(), self._origin_enu,
                                   self._spec.geofence_radius_m, margin_m=2.0):
                self.get_logger().error('越界 → 立即收工降落')
                self._report('abort', reason='geofence')
                self._begin_land()
                return
        if (self._phase == PHASE_RUN and self._task_started is not None
                and now - self._task_started > self._spec.max_duration_s):
            self.get_logger().warn('任务总时限到 → 收工降落')
            self._report('abort', reason='max_duration')
            self._begin_land()
            return

        if self._phase == PHASE_ARM:
            # 启动静置：等 ACK 订阅/命令链路稳定后再发第一条 arm，
            # 避免第一条命令的 ACK 在 DDS 匹配完成前丢失（2026-10-18 实测）。
            if self._phase_elapsed() < self._arm_settle:
                return
            report = self._prearm_report()
            self._publish_health(report, now)
            if not report.ok:
                if self._phase_elapsed() > self._arm_timeout:
                    self._abort('起飞前检查未通过：' + '；'.join(report.reasons()))
                return
            self._send_command('arm')
            if state.armed:
                self._enter(PHASE_OFFBOARD)
            elif self._phase_elapsed() > self._arm_timeout:
                self._abort('解锁超时')
            return

        if self._phase == PHASE_OFFBOARD:
            self._send_command('offboard')
            if state.offboard:
                self._task_started = now
                self._enter(PHASE_RUN)
                self._enter_step(0)
            elif self._phase_elapsed() > self._offboard_timeout:
                self._abort('进入 Offboard 超时')
            return

        if self._phase == PHASE_RUN:
            self._run_step(now)
            return

        if self._phase == PHASE_LAND:
            if not self._land_command_sent:
                self._send_command('land_fast')
                self._land_command_sent = True
                self._report('land_fast')
            if self._ever_armed and not state.armed:
                self._finish_done()
            elif self._phase_elapsed() > self._land_timeout:
                self.get_logger().warn('降落超时 → 强制上锁')
                self._send_command('disarm_force')
                self._finish_done(forced=True)
            return

    def _finish_done(self, *, forced: bool = False) -> None:
        # 先落 findings、再写 done（_enter(PHASE_DONE) 在 exit_on_done 时会退出）
        findings_path = ''
        findings_task_id = ''
        if self._persist_findings:
            try:
                info = persist_findings(
                    findings=self._findings.to_dicts(),
                    task_summary=(self._spec.summary() if self._spec else ''),
                    report_path=self._report_path,
                    intent_path=str(self.get_parameter('intent_path').value),
                    findings_dir=self._findings_dir,
                    started_at=self._task_started,
                    finished_at=time.time())
                findings_path = info['path']
                findings_task_id = info['task_id']
                self._report('findings_saved', path=findings_path,
                             task_id=findings_task_id,
                             count=len(self._findings))
            except OSError as exc:
                self.get_logger().warn(f'findings 持久化失败：{exc}')
        self._report('commands', commands=self._tracker.to_dicts())
        self._report('done', report_path=self._report_path, forced=forced,
                     commands=self._tracker.to_dicts(),
                     findings=self._findings.to_dicts(),
                     findings_path=findings_path,
                     findings_task_id=findings_task_id)
        self.get_logger().info('已上锁，任务结束' + ('（强制上锁）' if forced else ''))
        # 先发一条 DONE 状态再退出：Action server 依赖它收结果；
        # _enter(PHASE_DONE) 在 exit_on_done 时会直接 SystemExit。
        self._phase = PHASE_DONE
        self._last_status = 0.0
        self._publish_status()
        self._enter(PHASE_DONE)

    # ------------------------------------------------------------------
    # step 执行
    # ------------------------------------------------------------------

    def _current_step(self):
        if self._spec is None or self._step_index >= len(self._spec.steps):
            return None
        return self._spec.steps[self._step_index]

    def _current_step_type(self):
        step = self._current_step()
        return None if step is None else step.type

    def _enter_step(self, index: int) -> None:
        self._step_index = index
        self._step_started = time.monotonic()
        self._step_target = None
        self._yaw_target_enu = None
        self._search_wps = []
        self._search_index = 0
        self._takeoff_stable_at = None
        self._blocked_since = None
        self._target_hold_until = 0.0
        self._target_hold_count = 0
        self._target_world = None
        self._target_centered_reported = False
        step = self._current_step()
        self._scan_phase = 'dwell'
        self._scan_target_enu = None
        self._scan_dwell_until = (time.monotonic()
                                  + float(step.params.get('dwell_s', 2.5))
                                  if step is not None else 0.0)
        if step is None:
            self._begin_land()
            return
        self.get_logger().info(f'执行 step {index + 1}/{len(self._spec.steps)}：{step.type}')
        self._report('step', type=step.type, params=step.params)
        if step.type == 'search':
            self._search_wps = search_waypoints(
                step.params, altitude_m=self._spec.altitude_m)
            self._report('search_plan', waypoints=len(self._search_wps))
        if step.type == 'return_home':
            self._step_target = (self._origin_enu[0], self._origin_enu[1],
                                 self._origin_enu[2])

    def _advance_step(self, *, ignore_hold: bool = False) -> None:
        # takeoff 必须无视 hold 前进；否则起飞阶段看到目标会把 takeoff_complete
        # 重复写到 hold 结束（2026-10-18 实测 8 次）。
        if not ignore_hold and self._target_hold_until > time.monotonic():
            return
        self._enter_step(self._step_index + 1)

    def _run_step(self, now: float) -> None:
        step = self._current_step()
        if step is None:
            self._begin_land()
            return
        if now - self._step_started > self._step_timeout(step):
            self.get_logger().warn(f'step {step.type} 超时 → 收工降落')
            self._report('abort', reason=f'step_timeout:{step.type}')
            self._begin_land()
            return

        position = self._position_enu()
        if position is None:
            return

        if step.type == 'takeoff':
            self._run_takeoff(step, position)
        elif step.type == 'goto':
            self._run_goto(position, (float(step.params.get('east_m', 0.0)),
                                      float(step.params.get('north_m', 0.0)),
                                      float(step.params.get('altitude_m',
                                                            self._spec.altitude_m))))
        elif step.type == 'forward':
            if self._step_target is None:
                if self._heading_enu is None:
                    return
                self._step_target = forward_target(
                    position, self._heading_enu, float(step.params.get('meters', 0.0)))
            self._run_goto(position, self._step_target)
        elif step.type == 'yaw':
            self._run_yaw(step, position)
        elif step.type == 'scan':
            self._run_scan(step, position, now)
        elif step.type == 'hover':
            self._run_hover(step, position, now)
        elif step.type == 'search':
            self._run_search(position, now)
        elif step.type == 'return_home':
            self._run_goto(position, self._step_target)
        elif step.type == 'land':
            self._begin_land()
        else:
            self._abort(f'未知 step {step.type}')

    def _run_takeoff(self, step, position) -> None:
        altitude = float(step.params.get('altitude_m', self._spec.altitude_m))
        # 任务高度默认是 REL（相对起飞点）；PX4 的位置目标用 LCL。
        # 起飞前记下地面 LCL，之后 target_lcl = ground + altitude_rel，
        # **比较始终在 LCL 帧内**，避免 REL/LCL 混算（EKF 触地漂移的根因之一）。
        if self._spec.altitude_frame == 'LCL':
            target_lcl = altitude
        else:
            if self._ground_up_lcl is None:
                self._ground_up_lcl = float(position[2])
            target_lcl = self._ground_up_lcl + altitude
        self._publish_target(position[0], position[1], target_lcl)
        if abs(float(position[2]) - target_lcl) <= self._takeoff_tol:
            if self._takeoff_stable_at is None:
                self._takeoff_stable_at = time.monotonic()
            elif time.monotonic() - self._takeoff_stable_at > 1.0:
                self._origin_enu = (position[0], position[1], target_lcl)
                self._report(
                    'takeoff_complete',
                    altitude_lcl=altitude_dict(target_lcl, 'LCL', 'px4_local'),
                    target=altitude_dict(altitude, self._spec.altitude_frame, 'task'),
                    ground_lcl=self._ground_up_lcl)
                self._advance_step(ignore_hold=True)
        else:
            self._takeoff_stable_at = None

    def _target_up_lcl(self):
        """任务高度换算到 PX4 LCL 帧（REL → ground_lcl + altitude）。"""
        if self._spec is None:
            return None
        if self._spec.altitude_frame == 'LCL':
            return float(self._spec.altitude_m)
        ground = self._ground_up_lcl if self._ground_up_lcl is not None else 0.0
        return ground + float(self._spec.altitude_m)

    def _altitude_hold_vz(self, position) -> float:
        """速度模式下的高度保持：vz = clamp(0.6 * (target_lcl - current_lcl), ±0.3)。

        2026-10-18 实测：速度指令 vz 恒 0 时高度会缓慢下沉（1.77→0.24 m），
        低到地面回波进入前向锥后触发净空门。必须显式做高度保持。
        """
        return altitude_hold_vz(self._target_up_lcl(),
                                None if position is None else position[2])

    def _run_goto(self, position, target) -> None:
        decision = velocity_toward(position, target,
                                   max_speed_mps=self._spec.max_speed_mps,
                                   tolerance_m=self._arrive_tol)
        decision = apply_clearance_gate(
            decision, position, self._heading_enu or 0.0,
            self._clearance_forward, stop_m=self._clearance_stop)
        if decision.blocked:
            if self._blocked_since is None:
                self._blocked_since = time.monotonic()
                self._report('clearance_blocked', distance=self._clearance_forward)
            elif time.monotonic() - self._blocked_since > self._blocked_abort:
                self.get_logger().warn('持续被净空挡住 → 收工降落')
                self._report('abort', reason='clearance_blocked')
                self._begin_land()
                return
        else:
            self._blocked_since = None
        if decision.done:
            self._stop_velocity()
            self._advance_step()
            return
        self._publish_velocity(decision.east, decision.north,
                               self._altitude_hold_vz(position))

    def _run_yaw(self, step, position) -> None:
        if self._heading_enu is None:
            return
        if self._yaw_target_enu is None:
            self._yaw_target_enu = (self._heading_enu
                                    + math.radians(float(step.params.get('degrees', 90.0))))
            self._report('yaw_start',
                         target_enu_deg=round(math.degrees(self._yaw_target_enu), 1))
        rate, done = yaw_rate_to_target(
            self._heading_enu, self._yaw_target_enu,
            max_rate_rad_s=math.radians(float(step.params.get('rate_deg_s', 20.0))),
            tolerance_rad=math.radians(3.0))
        if done:
            self._stop_velocity()
            self._report('yaw_complete',
                         heading_enu_deg=round(math.degrees(self._heading_enu), 1))
            self._advance_step()
            return
        self._publish_velocity(0.0, 0.0, self._altitude_hold_vz(position),
                               yaw_rate=rate)

    def _run_scan(self, step, position, now) -> None:
        """步进+驻留扫描：转 step_deg，停 dwell_s，重复，直到总时长用完。

        连续匀速扫（15°/s）时，远处小目标在视野里只停留不到一帧；驻留 1.5 s
        约等于相机 2.5 Hz 的 3–4 帧，刚好够检测器做 3 帧确认。
        """
        if self._target_hold_until > now:
            self._run_target_hold(position, now)
            return
        seconds = float(step.params.get('seconds', 20.0))
        if now - self._step_started >= seconds:
            self._stop_velocity()
            self._report('scan_complete', seconds=seconds,
                         heading_enu_deg=(None if self._heading_enu is None
                                          else round(math.degrees(self._heading_enu), 1)))
            self._advance_step()
            return
        if self._heading_enu is None:
            return
        dwell = float(step.params.get('dwell_s', 2.5))
        step_deg = float(step.params.get('step_deg', 45.0))
        rate = math.radians(float(step.params.get('rate_deg_s', 20.0)))
        if self._scan_phase == 'dwell':
            self._publish_velocity(0.0, 0.0, self._altitude_hold_vz(position))
            if now >= self._scan_dwell_until:
                self._scan_target_enu = self._heading_enu + math.radians(step_deg)
                self._scan_phase = 'turn'
            return
        yaw_rate, done = yaw_rate_to_target(
            self._heading_enu, self._scan_target_enu,
            max_rate_rad_s=rate, tolerance_rad=math.radians(3.0))
        if done:
            self._stop_velocity()
            self._scan_phase = 'dwell'
            self._scan_dwell_until = now + dwell
            self._report('scan_dwell',
                         heading_enu_deg=round(math.degrees(self._heading_enu), 1))
            return
        self._publish_velocity(0.0, 0.0, self._altitude_hold_vz(position),
                               yaw_rate=yaw_rate)

    def _run_target_hold(self, position, now) -> None:
        """目标悬停：保持位置/高度，并转向目标（scan/hover 阶段）。

        只在 scan/hover 里用；search 阶段的 hold 不做偏航，避免改变航向
        后影响后续净空门的方向判断。
        """
        vz = self._altitude_hold_vz(position)
        if (not self._center_on_target or self._target_world is None
                or self._heading_enu is None):
            self._publish_velocity(0.0, 0.0, vz)
            return
        de = float(self._target_world[0]) - float(position[0])
        dn = float(self._target_world[1]) - float(position[1])
        if math.hypot(de, dn) < 0.5:
            self._publish_velocity(0.0, 0.0, vz)
            return
        desired = math.atan2(dn, de)
        rate, done = yaw_rate_to_target(
            self._heading_enu, desired,
            max_rate_rad_s=self._center_yaw_rate,
            tolerance_rad=self._center_tolerance)
        error_deg = math.degrees(math.remainder(desired - self._heading_enu, 2.0 * math.pi))
        if done:
            self._publish_velocity(0.0, 0.0, vz)
            if not self._target_centered_reported:
                self._target_centered_reported = True
                self._report('target_centered', target_world=self._target_world,
                             heading_enu_deg=round(math.degrees(self._heading_enu), 1),
                             error_deg=round(error_deg, 1))
                finding = self._findings.mark_inspected(
                    self._target_world, t=self._now_t(),
                    step_type=self._current_step_type(),
                    notes='target centered')
                if finding is not None:
                    self._report('finding', action='inspected', **finding.to_dict())
                self.get_logger().info(
                    f'目标已对准：方位误差 {error_deg:+.1f}°')
            return
        self._publish_velocity(0.0, 0.0, vz, yaw_rate=rate)

    def _run_hover(self, step, position, now) -> None:
        if self._target_hold_until > now:
            self._run_target_hold(position, now)
            return
        self._publish_target(position[0], position[1], self._target_up_lcl())
        self._stop_velocity()
        seconds = float(step.params.get('seconds', 5.0))
        if now - self._step_started >= seconds:
            self._advance_step()

    def _run_search(self, position, now) -> None:
        # 目标悬停优先：期间保持当前位置，不再推进航点
        if self._target_hold_until > now:
            self._publish_target(position[0], position[1], self._target_up_lcl())
            self._stop_velocity()
            return
        if self._search_index >= len(self._search_wps):
            self._advance_step()
            return
        target = self._search_wps[self._search_index]
        decision = velocity_toward(position, target,
                                   max_speed_mps=self._spec.max_speed_mps,
                                   tolerance_m=self._arrive_tol)
        decision = apply_clearance_gate(
            decision, position, self._heading_enu or 0.0,
            self._clearance_forward, stop_m=self._clearance_stop)
        if decision.blocked:
            if self._blocked_since is None:
                self._blocked_since = time.monotonic()
            elif time.monotonic() - self._blocked_since > self._blocked_abort:
                self._report('abort', reason='clearance_blocked')
                self._begin_land()
                return
        else:
            self._blocked_since = None
        if decision.done:
            self._search_index += 1
            self.get_logger().info(
                f'搜索航点 {self._search_index}/{len(self._search_wps)} 到位')
            self._report('search_waypoint', index=self._search_index,
                         total=len(self._search_wps),
                         target=list(target))
            return
        self._publish_velocity(decision.east, decision.north, 0.0)

    # ------------------------------------------------------------------
    # 指令 / 遥测
    # ------------------------------------------------------------------

    def _send_command(self, command: str) -> None:
        if self._dry_run:
            return
        vehicle_command, expect_ack = _COMMAND_IDS.get(command, (None, False))
        record = self._tracker.get(command)
        retry_states = (CommandState.TIMEOUT.value, CommandState.REJECTED.value,
                        CommandState.DELIVERY_UNKNOWN.value, CommandState.FAILED.value)
        if record is None or record.state in retry_states:
            record = self._tracker.submit(command, vehicle_command,
                                          expect_ack=expect_ack)
            self._report('command_sent', name=command,
                         vehicle_command=vehicle_command,
                         expect_ack=expect_ack, attempts=record.attempts,
                         state=record.state)
        elif record.state in (CommandState.EXECUTING.value,
                              CommandState.COMPLETED.value) and not expect_ack:
            return      # land_fast/hold 已发出，不重复
        self._command_pub.publish(String(data=command))

    def _publish_target(self, east: float, north: float, up: float) -> None:
        if self._dry_run:
            return
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = 'map'
        pose.pose.position.x = float(east)
        pose.pose.position.y = float(north)
        pose.pose.position.z = float(up)
        pose.pose.orientation.w = 1.0
        self._target_pub.publish(pose)

    def _publish_velocity(self, east: float, north: float, up: float,
                          yaw_rate: float = 0.0) -> None:
        if self._dry_run:
            return
        msg = Twist()
        msg.linear.x = float(east)
        msg.linear.y = float(north)
        msg.linear.z = float(up)
        msg.angular.z = float(yaw_rate)
        self._velocity_pub.publish(msg)

    def _stop_velocity(self) -> None:
        self._publish_velocity(0.0, 0.0, 0.0)

    def _update_heading(self) -> None:
        state = self._monitor.snapshot()
        if math.isnan(state.heading):
            return
        self._heading_enu = ned_yaw_to_enu_yaw(state.heading)

    def _update_heartbeat(self, now: float) -> None:
        if now - self._last_heartbeat < self._heartbeat_s:
            return
        self._last_heartbeat = now
        # 带 step 与 30 s 桶：监督层按"字段变化"重置卡阶段计时，
        # 避免一个长 goto/搜索被误判为"阶段卡死"而触发 land_fast。
        bucket = int(max(now - self._step_started, 0.0) // 30)
        self._heartbeat_pub.publish(
            String(data=f'{self._phase}:{self._step_index}:{bucket}'))

    def _publish_status(self) -> None:
        now = time.monotonic()
        if now - self._last_status < self._heartbeat_s:
            return
        self._last_status = now
        position = self._position_enu()
        payload = {
            'phase': self._phase,
            'step': self._step_index,
            'step_type': self._current_step_type(),
            'step_total': (len(self._spec.steps) if self._spec is not None else 0),
            'report_path': self._report_path,
            'aborted': self._abort_reason is not None,
            'abort_reason': self._abort_reason,
            'dry_run': self._dry_run,
            'search_waypoint': self._search_index,
            'clearance_forward_m': (None if self._clearance_forward is None
                                    else round(float(self._clearance_forward), 2)),
            'position_enu': (None if position is None
                             else [round(float(v), 2) for v in position]),
            # 高度永远带帧：LCL=px4 local，task=任务高度（默认 REL）
            'altitude_lcl': altitude_dict(None if position is None else position[2],
                                          'LCL', 'px4_local'),
            'commands': self._tracker.to_dicts(),
            'health': self._last_health,
            'findings': self._findings.to_dicts(),
        }
        if self._spec is not None:
            payload['task_altitude'] = altitude_dict(
                self._spec.altitude_m, self._spec.altitude_frame, 'task')
        self._status_pub.publish(String(data=json.dumps(payload, ensure_ascii=False)))

    def _step_timeout(self, step) -> float:
        if step.type == 'search' and self._search_wps:
            flat = [(point[0], point[1]) for point in self._search_wps]
            budget = estimate_duration_s(flat, self._spec.max_speed_mps)
            return max(60.0, 2.0 * budget + 30.0)
        return step_timeout_s(step, speed_mps=self._spec.max_speed_mps)

    def _abort(self, reason: str) -> None:
        self.get_logger().error(f'任务中止：{reason}')
        self._abort_reason = str(reason)
        self._report('abort', reason=reason)
        self._begin_land()

    def _begin_land(self) -> None:
        if self._phase in (PHASE_LAND, PHASE_DONE):
            return
        self._stop_velocity()
        self._land_command_sent = False
        self._enter(PHASE_LAND)

    def destroy_node(self):
        try:
            if self._report_handle is not None:
                self._report_handle.close()
        finally:
            super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TaskExecutor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
