"""航点任务状态机。

流程：WAIT_PX4 → ARM → OFFBOARD → TAKEOFF → CRUISE(逐个航点) → LAND → WAIT_DISARM → DONE，
任何环节失败或超时都会进入 ABORT 并给出原因。

本节点不直接收发 PX4 消息：

- 目标点以 ENU 坐标发布到 offboard_bridge 的 ``~/target_pose``；
- arm / offboard / land 指令以文本发布到 offboard_bridge 的 ``~/command``；
- 状态通过 ``px4_interface.state.Px4StateMonitor`` 读取（库复用，不重复订阅代码）。
"""

import time
from enum import Enum

import rclpy
from geometry_msgs.msg import PoseStamped
from px4_interface.frames import distance, ned_to_enu
from px4_interface.state import Px4StateMonitor
from rclpy.node import Node
from std_msgs.msg import String


class Phase(Enum):
    WAIT_PX4 = 'WAIT_PX4'
    ARM = 'ARM'
    OFFBOARD = 'OFFBOARD'
    TAKEOFF = 'TAKEOFF'
    CRUISE = 'CRUISE'
    PAUSED = 'PAUSED'
    LAND = 'LAND'
    WAIT_DISARM = 'WAIT_DISARM'
    DONE = 'DONE'
    ABORT = 'ABORT'


class WaypointMission(Node):
    """执行一串 ENU 航点的任务节点。"""

    def __init__(self) -> None:
        super().__init__('waypoint_mission')

        # 默认任务：以起飞点为参照的 5 m 方形航迹，高度 3 m（ENU，单位米）
        self.declare_parameter('waypoints', [
            0.0, 0.0, 3.0,
            5.0, 0.0, 3.0,
            5.0, 5.0, 3.0,
            0.0, 5.0, 3.0,
            0.0, 0.0, 3.0,
        ])
        self.declare_parameter('takeoff_altitude_m', 3.0)
        self.declare_parameter('arrival_radius_m', 0.5)
        self.declare_parameter('arrival_hold_s', 1.0)
        self.declare_parameter('waypoint_timeout_s', 30.0)
        self.declare_parameter('takeoff_timeout_s', 25.0)
        self.declare_parameter('arm_timeout_s', 15.0)
        self.declare_parameter('offboard_timeout_s', 15.0)
        self.declare_parameter('land_timeout_s', 40.0)
        self.declare_parameter('disarm_timeout_s', 15.0)
        self.declare_parameter('pause_timeout_s', 120.0)
        self.declare_parameter('loop_rate_hz', 5.0)
        self.declare_parameter('auto_land_on_abort', True)
        self.declare_parameter('command_topic', '/offboard_bridge/command')
        self.declare_parameter('target_topic', '/offboard_bridge/target_pose')

        self._waypoints = self._parse_waypoints(
            self.get_parameter('waypoints').value)
        self._takeoff_alt = float(self.get_parameter('takeoff_altitude_m').value)
        self._arrival_radius = float(self.get_parameter('arrival_radius_m').value)
        self._arrival_hold_s = float(self.get_parameter('arrival_hold_s').value)
        self._timeouts = {
            Phase.ARM: float(self.get_parameter('arm_timeout_s').value),
            Phase.OFFBOARD: float(self.get_parameter('offboard_timeout_s').value),
            Phase.TAKEOFF: float(self.get_parameter('takeoff_timeout_s').value),
            Phase.CRUISE: float(self.get_parameter('waypoint_timeout_s').value),
            Phase.LAND: float(self.get_parameter('land_timeout_s').value),
            Phase.WAIT_DISARM: float(self.get_parameter('disarm_timeout_s').value),
        }
        self._pause_timeout = float(self.get_parameter('pause_timeout_s').value)
        self._auto_land_on_abort = bool(
            self.get_parameter('auto_land_on_abort').value)

        self._command_pub = self.create_publisher(
            String, self.get_parameter('command_topic').value, 10)
        self._target_pub = self.create_publisher(
            PoseStamped, self.get_parameter('target_topic').value, 10)
        self._status_pub = self.create_publisher(String, '~/status', 10)

        self._monitor = Px4StateMonitor(self)

        self._phase = Phase.WAIT_PX4
        self._phase_entered = time.monotonic()
        self._target = None
        self._waypoint_index = 0
        self._inside_radius_since = None
        self._abort_reason = ''
        self._waypoint_errors = []
        self._paused_from = None

        rate_hz = float(self.get_parameter('loop_rate_hz').value)
        self.create_timer(1.0 / rate_hz, self._tick)

        if not self._waypoints:
            self._abort('waypoints 参数为空或长度不是 3 的倍数')
        else:
            self.get_logger().info(
                f'任务启动：{len(self._waypoints)} 个航点，起飞高度 {self._takeoff_alt:.1f} m，'
                f'到达判定 {self._arrival_radius:.2f} m / 保持 {self._arrival_hold_s:.1f} s')
            for i, wp in enumerate(self._waypoints):
                self.get_logger().info(
                    f'  航点 {i}: ENU=({wp[0]:.2f}, {wp[1]:.2f}, {wp[2]:.2f})')

    # ------------------------------------------------------------------
    # 参数解析
    # ------------------------------------------------------------------

    @staticmethod
    def _parse_waypoints(flat):
        values = [float(v) for v in flat]
        if len(values) == 0 or len(values) % 3 != 0:
            return []
        return [tuple(values[i:i + 3]) for i in range(0, len(values), 3)]

    # ------------------------------------------------------------------
    # 主循环
    # ------------------------------------------------------------------

    def _tick(self) -> None:
        if self._phase in (Phase.DONE, Phase.ABORT):
            return

        if not self._monitor.status_fresh():
            self.get_logger().warn(
                '等待 PX4 状态（检查 agent 与 PX4 是否已启动）',
                throttle_duration_sec=5.0)
            return

        state = self._monitor.snapshot()

        # 人工接管中：等待交还，回到 OFFBOARD 后从原阶段/原航点继续
        if self._phase is Phase.PAUSED:
            if state.failsafe:
                self._abort('暂停期间 PX4 进入 failsafe')
                return
            if state.offboard:
                resume_phase = self._paused_from or Phase.CRUISE
                self.get_logger().info(
                    f'检测到已交还控制，恢复自主：回到 {resume_phase.value}')
                self._enter(resume_phase)
                return
            if self._phase_elapsed() > self._pause_timeout:
                self._abort(f'人工接管超过 {self._pause_timeout:.0f} s 未交还')
            self._publish_status(state)
            return

        # 飞行阶段的安全监控：failsafe 或离开 OFFBOARD（例如人工接管）立即交还控制权
        if self._phase in (Phase.TAKEOFF, Phase.CRUISE):
            if state.failsafe:
                self._abort('PX4 进入 failsafe')
                return
            if not state.offboard:
                self.get_logger().warn(
                    f'检测到离开 OFFBOARD（nav_state={state.nav_state}），判定为人工接管，暂停任务')
                self._paused_from = self._phase
                self._enter(Phase.PAUSED)
                return

        # 周期性刷新目标：桥接节点带有目标超时看门狗，只发一次会被判定为过期
        if self._phase in (Phase.TAKEOFF, Phase.CRUISE) and self._target is not None:
            self._publish_target(self._target)

        self._check_arrival(state)
        self._check_timeout(state)
        self._advance(state)
        self._publish_status(state)

    # ------------------------------------------------------------------
    # 阶段进入动作（只在进入时执行一次）
    # ------------------------------------------------------------------

    def _enter(self, phase: Phase) -> None:
        self._phase = phase
        self._phase_entered = time.monotonic()
        self._inside_radius_since = None

        if phase is Phase.ARM:
            self._send_command('arm')
        elif phase is Phase.OFFBOARD:
            self._send_command('offboard')
        elif phase is Phase.TAKEOFF:
            current = self._current_enu()
            if current is None:
                self._abort('起飞前无法获取有效位置')
                return
            self._target = (current[0], current[1], self._takeoff_alt)
            self._publish_target(self._target)
            self.get_logger().info(
                f'起飞：目标 ENU=({self._target[0]:.2f}, {self._target[1]:.2f}, '
                f'{self._target[2]:.2f})')
        elif phase is Phase.CRUISE:
            if self._waypoint_index >= len(self._waypoints):
                self._enter(Phase.LAND)
                return
            self._target = self._waypoints[self._waypoint_index]
            self._publish_target(self._target)
            self.get_logger().info(
                f'飞往航点 {self._waypoint_index}: '
                f'({self._target[0]:.2f}, {self._target[1]:.2f}, {self._target[2]:.2f})')
        elif phase is Phase.LAND:
            self._target = None
            self._send_command('land')
            self.get_logger().info('开始自动降落')
        elif phase is Phase.DONE:
            self._target = None
            self._report_summary()

    # ------------------------------------------------------------------
    # 到达 / 超时 / 推进
    # ------------------------------------------------------------------

    def _check_arrival(self, state) -> None:
        if self._phase not in (Phase.TAKEOFF, Phase.CRUISE) or self._target is None:
            self._inside_radius_since = None
            return

        if self._distance_to(state, self._target) > self._arrival_radius:
            self._inside_radius_since = None
            return

        if self._inside_radius_since is None:
            self._inside_radius_since = time.monotonic()
            return

        if time.monotonic() - self._inside_radius_since < self._arrival_hold_s:
            return

        error = self._distance_to(state, self._target)
        if self._phase is Phase.TAKEOFF:
            self.get_logger().info(f'起飞完成，高度误差 {error:.2f} m')
            self._enter(Phase.CRUISE)
        else:
            self._waypoint_errors.append(error)
            self.get_logger().info(
                f'到达航点 {self._waypoint_index}，位置误差 {error:.2f} m')
            self._waypoint_index += 1
            self._enter(Phase.CRUISE)

    def _check_timeout(self, state) -> None:
        timeout = self._timeouts.get(self._phase)
        if timeout is None or self._phase_elapsed() <= timeout:
            return
        detail = ''
        if self._target is not None:
            detail = f'，距目标 {self._distance_to(state, self._target):.2f} m'
        if self._phase is Phase.CRUISE:
            detail = f'（航点 {self._waypoint_index}）' + detail
        self._abort(f'{self._phase.value} 阶段超时 {timeout:.0f} s{detail}')

    def _advance(self, state) -> None:
        if self._phase is Phase.WAIT_PX4:
            if state.pre_flight_checks_pass:
                self._enter(Phase.ARM)
            else:
                self.get_logger().info('等待 PX4 自检通过', throttle_duration_sec=5.0)
        elif self._phase is Phase.ARM:
            if state.armed:
                self.get_logger().info('已解锁')
                self._enter(Phase.OFFBOARD)
        elif self._phase is Phase.OFFBOARD:
            if state.offboard and state.accepts_offboard_setpoints:
                self.get_logger().info('已进入 OFFBOARD')
                self._enter(Phase.TAKEOFF)
        elif self._phase is Phase.LAND:
            if state.landed or not state.armed:
                self.get_logger().info('已落地')
                self._enter(Phase.WAIT_DISARM)
        elif self._phase is Phase.WAIT_DISARM:
            if not state.armed:
                self._enter(Phase.DONE)

    # ------------------------------------------------------------------
    # 辅助
    # ------------------------------------------------------------------

    def _phase_elapsed(self) -> float:
        return time.monotonic() - self._phase_entered

    def _send_command(self, command: str) -> None:
        msg = String()
        msg.data = command
        self._command_pub.publish(msg)

    def _publish_target(self, target_enu) -> None:
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position.x = float(target_enu[0])
        msg.pose.position.y = float(target_enu[1])
        msg.pose.position.z = float(target_enu[2])
        # 位置任务不控制朝向：四元数保持单位值，PX4 侧 yaw 置 NaN 维持当前偏航
        msg.pose.orientation.w = 1.0
        self._target_pub.publish(msg)

    def _current_enu(self):
        position_ned = self._monitor.snapshot().position_ned
        if position_ned is None:
            return None
        return ned_to_enu(*position_ned)

    def _distance_to(self, state, target_enu) -> float:
        current = self._current_enu()
        if current is None:
            return float('inf')
        return distance(current, target_enu)

    def _abort(self, reason: str) -> None:
        self._abort_reason = reason
        self.get_logger().error(f'任务中止：{reason}')
        self._phase = Phase.ABORT
        self._inside_radius_since = None
        if self._auto_land_on_abort and self._monitor.snapshot().armed:
            self._send_command('land')

    def _report_summary(self) -> None:
        if not self._waypoint_errors:
            self.get_logger().info('任务完成（无航点误差记录）')
            return
        average = sum(self._waypoint_errors) / len(self._waypoint_errors)
        worst = max(self._waypoint_errors)
        self.get_logger().info(
            f'任务完成：{len(self._waypoint_errors)} 个航点，'
            f'平均误差 {average:.2f} m，最大误差 {worst:.2f} m')

    def _publish_status(self, state) -> None:
        if self._target is None:
            distance_txt = 'n/a'
        else:
            distance_txt = f'{self._distance_to(state, self._target):.2f}'
        msg = String()
        msg.data = (
            f'phase={self._phase.value} waypoint={self._waypoint_index}/'
            f'{len(self._waypoints)} distance_m={distance_txt} '
            f'armed={state.armed} nav_state={state.nav_state} '
            f'landed={state.landed} abort_reason={self._abort_reason}')
        self._status_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = WaypointMission()
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
