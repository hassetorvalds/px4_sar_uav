"""Offboard 桥接节点：项目中唯一直接与 PX4 ``/fmu/*`` 通信的节点。

职责：

1. 以固定频率发送 ``OffboardControlMode`` 与 ``TrajectorySetpoint``（默认 20 Hz）；
2. 接收 ROS 侧的 ENU 目标点（``geometry_msgs/PoseStamped``）并转换为 PX4 的 NED；
3. 接收文本指令（``std_msgs/String``）执行 arm / disarm / offboard / land / hold；
4. 看门狗：PX4 状态超时即**停止发送设定点**，把控制权交回 PX4 的 offboard 丢失 failsafe。

指令用文本话题而不是 service/action，是为了避免单线程 executor 下
“在回调里等待 ack”造成的自死锁；任务层通过 PX4 状态判断执行结果。
"""

import time

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleCommandAck,
)
from rclpy.node import Node
from std_msgs.msg import String

from .frames import distance, enu_to_ned, flip_yaw_rate_sign, ned_to_enu
from .qos import px4_qos
from .state import Px4StateMonitor

# 文本指令字（发布到 /offboard_bridge/command）
CMD_ARM = 'arm'
CMD_DISARM = 'disarm'
CMD_DISARM_FORCE = 'disarm_force'
CMD_OFFBOARD = 'offboard'
CMD_LAND = 'land'
CMD_LAND_FAST = 'land_fast'
CMD_HOLD = 'hold'

SUPPORTED_COMMANDS = (
    CMD_ARM,
    CMD_DISARM,
    CMD_DISARM_FORCE,
    CMD_OFFBOARD,
    CMD_LAND,
    CMD_LAND_FAST,
    CMD_HOLD,
)

# MAVLink / PX4 常量
_MAV_MODE_FLAG_CUSTOM_MODE_ENABLED = 1
_PX4_CUSTOM_MAIN_MODE_OFFBOARD = 6
_FORCE_DISARM_MAGIC = 21196.0

_NAN = float('nan')


class OffboardBridge(Node):
    """把 ROS 侧的目标点与指令翻译成 PX4 Offboard 消息的桥接节点。"""

    def __init__(self) -> None:
        super().__init__('offboard_bridge')

        self.declare_parameter('setpoint_rate_hz', 20.0)
        self.declare_parameter('status_timeout_s', 1.0)
        self.declare_parameter('target_timeout_s', 2.0)
        self.declare_parameter('velocity_timeout_s', 1.0)
        self.declare_parameter('land_descend_speed_mps', 0.5)
        self.declare_parameter('land_duration_margin_s', 2.0)
        self.declare_parameter('status_publish_period_s', 1.0)

        rate_hz = float(self.get_parameter('setpoint_rate_hz').value)
        status_timeout_s = float(self.get_parameter('status_timeout_s').value)
        self._target_timeout_s = float(self.get_parameter('target_timeout_s').value)
        self._velocity_timeout_s = float(
            self.get_parameter('velocity_timeout_s').value)
        self._land_descend_speed = float(
            self.get_parameter('land_descend_speed_mps').value)
        self._land_margin_s = float(
            self.get_parameter('land_duration_margin_s').value)
        self._status_period_s = float(self.get_parameter('status_publish_period_s').value)

        qos = px4_qos()
        self._offboard_mode_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos)
        self._setpoint_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos)
        self._vehicle_command_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos)

        self.create_subscription(
            VehicleCommandAck, '/fmu/out/vehicle_command_ack_v1',
            self._on_command_ack, qos)
        self.create_subscription(PoseStamped, '~/target_pose', self._on_target_pose, 10)
        self.create_subscription(
            Twist, '~/velocity_setpoint', self._on_velocity_setpoint, 10)
        self.create_subscription(String, '~/command', self._on_command, 10)
        self._status_pub = self.create_publisher(String, '~/status', 10)

        self._monitor = Px4StateMonitor(self, status_timeout_s=status_timeout_s)

        self._target_enu = None
        self._last_target_monotonic = 0.0
        self._velocity_enu = None
        self._yaw_rate_enu = 0.0
        self._last_velocity_monotonic = 0.0
        self._mode = 'position'
        self._landing_started_at = None
        self._landing_duration_s = 0.0
        self._streaming = True
        self._had_status = False
        self._last_status_publish = 0.0
        self._stale_target_warned = False

        self.create_timer(1.0 / rate_hz, self._on_timer)
        self.get_logger().info(
            f'offboard_bridge 启动：{rate_hz:.1f} Hz 设定点流，'
            f'状态超时 {status_timeout_s:.1f} s，目标超时 {self._target_timeout_s:.1f} s，'
            f'速度超时 {self._velocity_timeout_s:.1f} s')
        self.get_logger().info(
            'ROS 侧一律使用 ENU 坐标；与 PX4 的 NED 转换只在本节点内发生')

    # ------------------------------------------------------------------
    # 订阅回调
    # ------------------------------------------------------------------

    def _on_target_pose(self, msg: PoseStamped) -> None:
        if msg.header.frame_id and msg.header.frame_id not in ('map', 'odom', 'world'):
            self.get_logger().warn(
                f'目标点 frame_id="{msg.header.frame_id}" 不是 map/odom/world，'
                '仍按 ENU 处理，请确认坐标约定')
        p = msg.pose.position
        new_target = (float(p.x), float(p.y), float(p.z))
        # 任务层会以 5 Hz 刷新目标，只在目标真正变化时打印，避免刷屏
        changed = self._target_enu is None or distance(new_target, self._target_enu) > 1e-6
        self._target_enu = new_target
        self._last_target_monotonic = time.monotonic()
        self._stale_target_warned = False
        if changed:
            self.get_logger().info(
                f'新目标点 ENU=({new_target[0]:.2f}, {new_target[1]:.2f}, '
                f'{new_target[2]:.2f})')

    def _on_command(self, msg: String) -> None:
        command = msg.data.strip().lower()
        if command == CMD_ARM:
            self._send_vehicle_command(
                VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        elif command == CMD_DISARM:
            self._send_vehicle_command(
                VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
        elif command == CMD_DISARM_FORCE:
            self._send_vehicle_command(
                VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
                param1=0.0, param2=_FORCE_DISARM_MAGIC)
        elif command == CMD_OFFBOARD:
            self._send_vehicle_command(
                VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
                param1=float(_MAV_MODE_FLAG_CUSTOM_MODE_ENABLED),
                param2=float(_PX4_CUSTOM_MAIN_MODE_OFFBOARD))
        elif command == CMD_LAND:
            self._send_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        elif command == CMD_LAND_FAST:
            self._start_fast_land()
        elif command == CMD_HOLD:
            self._target_enu = None
            self._velocity_enu = None
            self.get_logger().info('清空目标点，改为保持当前位置')
        else:
            self.get_logger().warn(
                f'未知指令 "{msg.data}"，支持：{", ".join(SUPPORTED_COMMANDS)}')
            return
        self.get_logger().info(f'已下发指令：{command}')

    def _on_command_ack(self, msg: VehicleCommandAck) -> None:
        if msg.command in (
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
            VehicleCommand.VEHICLE_CMD_NAV_LAND,
        ):
            level = 'OK' if msg.result == VehicleCommandAck.VEHICLE_CMD_RESULT_ACCEPTED \
                else f'失败(result={msg.result})'
            self.get_logger().info(f'指令 ACK：cmd={msg.command} {level}')

    def _start_fast_land(self) -> None:
        """按已知高度用速度模式定时下降，落定后强制上锁。

        背景（2026-09-19 实测）：AirSim + PX4 的 `AUTO_LAND` 在高度估计漂移时会
        以约 0.04 m/s 龟速下降（3 m 用约 80 s），期间 z 估计在 -2.8 到 +2.0 之间摆动。
        本通道只依赖“起飞时的高度估计 + 恒定下降速度 + 时间”，因此不受漂移影响。
        """
        state = self._monitor.snapshot()
        altitude = abs(state.z) if state.z_valid else 0.0
        if altitude < 0.2:
            # 已经在地面附近，直接上锁
            self._landing_duration_s = 0.5
        else:
            self._landing_duration_s = (altitude / max(self._land_descend_speed, 0.05)
                                        + self._land_margin_s)
        self._landing_started_at = time.monotonic()
        self.get_logger().info(
            f'快速降落：起始高度 {altitude:.2f} m，'
            f'下降速度 {self._land_descend_speed:.2f} m/s，'
            f'预计 {self._landing_duration_s:.1f} s 后上锁')

    def _update_fast_land(self) -> bool:
        """维护快速降落流程；返回 True 表示正在降落。"""
        if self._landing_started_at is None:
            return False
        elapsed = time.monotonic() - self._landing_started_at
        if elapsed >= self._landing_duration_s:
            self.get_logger().info('快速降落完成，强制上锁')
            self._send_vehicle_command(
                VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
                param1=0.0, param2=_FORCE_DISARM_MAGIC)
            self._landing_started_at = None
            self._velocity_enu = None
            return False
        # 持续刷新速度指令，使速度模式保持激活
        self._velocity_enu = (0.0, 0.0, -abs(self._land_descend_speed))
        self._yaw_rate_enu = 0.0
        self._last_velocity_monotonic = time.monotonic()
        return True

    def _on_velocity_setpoint(self, msg: Twist) -> None:
        """接收 ENU 世界速度（m/s）与 ENU 偏航角速度（rad/s）。

        这是 "See, Point, Fly" 一类“指向即飞行”闭环的落地接口：
        上层把图像里的目标点翻译成机体速度，再按当前航向旋转到 ENU 世界系后发布到这里。
        """
        new_velocity = (float(msg.linear.x), float(msg.linear.y), float(msg.linear.z))
        new_yaw_rate = float(msg.angular.z)
        previous = self._velocity_enu
        changed = (
            previous is None
            or distance(new_velocity, previous) > 1e-3
            or abs(new_yaw_rate - self._yaw_rate_enu) > 1e-3
        )
        self._velocity_enu = new_velocity
        self._yaw_rate_enu = new_yaw_rate
        self._last_velocity_monotonic = time.monotonic()
        if changed:
            self.get_logger().info(
                f'新速度指令 ENU=({new_velocity[0]:.2f}, {new_velocity[1]:.2f}, '
                f'{new_velocity[2]:.2f}) m/s，偏航角速度 {new_yaw_rate:.2f} rad/s')

    # ------------------------------------------------------------------
    # 定时器：设定点流 + 看门狗
    # ------------------------------------------------------------------

    def _on_timer(self) -> None:
        if not self._monitor.status_fresh():
            if self._streaming:
                if self._had_status:
                    self.get_logger().error(
                        f'PX4 状态超过 {self._monitor.status_age_s():.1f} s 未更新：'
                        '停止发送设定点，交由 PX4 的 offboard 丢失 failsafe 处理')
                else:
                    # 启动阶段还没收到过状态：等待而不是报错
                    self.get_logger().info(
                        '等待 PX4 状态（检查 agent 与 PX4 是否已启动）',
                        throttle_duration_sec=5.0)
                self._streaming = False
            return

        self._had_status = True
        self._update_fast_land()
        if not self._streaming:
            self.get_logger().info('PX4 状态恢复，继续发送设定点')
            self._streaming = True

        self._update_mode()
        self._publish_offboard_control_mode()
        self._publish_trajectory_setpoint()
        self._maybe_publish_status()

    def _publish_offboard_control_mode(self) -> None:
        msg = OffboardControlMode()
        msg.timestamp = self._px4_timestamp_us()
        msg.position = self._mode == 'position'
        msg.velocity = self._mode == 'velocity'
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.thrust_and_torque = False
        msg.direct_actuator = False
        self._offboard_mode_pub.publish(msg)

    def _publish_trajectory_setpoint(self) -> None:
        if self._mode == 'velocity':
            self._publish_velocity_setpoint()
            return

        target_enu = self._effective_target_enu()
        x_ned, y_ned, z_ned = enu_to_ned(*target_enu)

        msg = TrajectorySetpoint()
        msg.timestamp = self._px4_timestamp_us()
        msg.position = [float(x_ned), float(y_ned), float(z_ned)]
        # 只做位置控制：其余维度显式置 NaN，表示“不控制”
        msg.velocity = [_NAN, _NAN, _NAN]
        msg.acceleration = [_NAN, _NAN, _NAN]
        msg.jerk = [_NAN, _NAN, _NAN]
        msg.yaw = _NAN
        msg.yawspeed = _NAN
        self._setpoint_pub.publish(msg)

    def _publish_velocity_setpoint(self) -> None:
        """把 ENU 世界速度转换为 PX4 的 NED 速度设定点。"""
        velocity_enu = self._velocity_enu or (0.0, 0.0, 0.0)
        x_ned, y_ned, z_ned = enu_to_ned(*velocity_enu)

        msg = TrajectorySetpoint()
        msg.timestamp = self._px4_timestamp_us()
        msg.position = [_NAN, _NAN, _NAN]
        msg.velocity = [float(x_ned), float(y_ned), float(z_ned)]
        msg.acceleration = [_NAN, _NAN, _NAN]
        msg.jerk = [_NAN, _NAN, _NAN]
        msg.yaw = _NAN
        msg.yawspeed = float(flip_yaw_rate_sign(self._yaw_rate_enu))
        self._setpoint_pub.publish(msg)

    def _update_mode(self) -> None:
        """位置控制与速度控制之间切换；速度指令超时即回到位置保持。"""
        velocity_active = (
            self._velocity_enu is not None
            and (time.monotonic() - self._last_velocity_monotonic)
            <= self._velocity_timeout_s
        )
        new_mode = 'velocity' if velocity_active else 'position'
        if new_mode != self._mode:
            suffix = '（速度指令超时，回到位置保持）' if new_mode == 'position' else ''
            self.get_logger().info(f'控制模式切换：{self._mode} → {new_mode}{suffix}')
            self._mode = new_mode

    def _effective_target_enu(self):
        """当前生效的目标点（ENU）。

        优先级：未超时的外部目标 > 当前位置（保持）> 原点。
        """
        if self._target_enu is not None:
            age = time.monotonic() - self._last_target_monotonic
            if age <= self._target_timeout_s:
                return self._target_enu
            if not self._stale_target_warned:
                self.get_logger().warn(
                    f'目标点已 {age:.1f} s 未更新，改为保持当前位置')
                self._stale_target_warned = True
            self._target_enu = None

        position_ned = self._monitor.snapshot().position_ned
        if position_ned is not None:
            return ned_to_enu(*position_ned)
        return (0.0, 0.0, 0.0)

    def _maybe_publish_status(self) -> None:
        now = time.monotonic()
        if now - self._last_status_publish < self._status_period_s:
            return
        self._last_status_publish = now

        state = self._monitor.snapshot()
        target = self._target_enu
        target_txt = ('none' if target is None
                      else f'({target[0]:.2f},{target[1]:.2f},{target[2]:.2f})')
        msg = String()
        msg.data = (
            f'armed={state.armed} nav_state={state.nav_state} '
            f'failsafe={state.failsafe} landed={state.landed} '
            f'mode={self._mode} target_enu={target_txt} streaming={self._streaming}')
        self._status_pub.publish(msg)

    # ------------------------------------------------------------------
    # 工具函数
    # ------------------------------------------------------------------

    def _px4_timestamp_us(self) -> int:
        return self.get_clock().now().nanoseconds // 1000

    def _send_vehicle_command(self, command: int,
                              param1: float = 0.0, param2: float = 0.0) -> None:
        msg = VehicleCommand()
        msg.timestamp = self._px4_timestamp_us()
        msg.command = command
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self._vehicle_command_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OffboardBridge()
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
