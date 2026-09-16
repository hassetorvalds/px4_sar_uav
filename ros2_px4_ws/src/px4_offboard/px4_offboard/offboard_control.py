#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
    HistoryPolicy,
)

from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleCommandAck,
    VehicleStatus,
    VehicleLocalPosition,
)


class OffboardControl(Node):
    def __init__(self):
        super().__init__('offboard_control')

        # ==========================================================
        # QoS
        #
        # PX4 uXRCE-DDS output topics use BEST_EFFORT.
        # Using the default RELIABLE QoS causes:
        #   "offering incompatible QoS"
        # ==========================================================

        px4_out_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        px4_in_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # ==========================================================
        # ROS 2 -> PX4 publishers
        # ==========================================================

        self.offboard_control_mode_pub = self.create_publisher(
            OffboardControlMode,
            '/fmu/in/offboard_control_mode',
            px4_in_qos,
        )

        self.trajectory_setpoint_pub = self.create_publisher(
            TrajectorySetpoint,
            '/fmu/in/trajectory_setpoint',
            px4_in_qos,
        )

        self.vehicle_command_pub = self.create_publisher(
            VehicleCommand,
            '/fmu/in/vehicle_command',
            px4_in_qos,
        )

        # ==========================================================
        # PX4 -> ROS 2 subscriptions
        # ==========================================================

        self.vehicle_status_sub = self.create_subscription(
            VehicleStatus,
            '/fmu/out/vehicle_status_v4',
            self.vehicle_status_callback,
            px4_out_qos,
        )

        self.vehicle_local_position_sub = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position_v1',
            self.vehicle_local_position_callback,
            px4_out_qos,
        )

        self.command_ack_sub = self.create_subscription(
            VehicleCommandAck,
            '/fmu/out/vehicle_command_ack_v1',
            self.command_ack_callback,
            px4_out_qos,
        )

        # ==========================================================
        # State
        # ==========================================================

        self.arming_state = VehicleStatus.ARMING_STATE_DISARMED
        self.nav_state = VehicleStatus.NAVIGATION_STATE_MANUAL

        self.current_x = float('nan')
        self.current_y = float('nan')
        self.current_z = float('nan')

        self.offboard_requested = False
        self.arm_requested = False

        self.counter = 0

        # 20 Hz
        self.timer = self.create_timer(
            0.05,
            self.timer_callback,
        )

        self.get_logger().info(
            'PX4 Offboard + ARM test started.'
        )

        self.get_logger().info(
            'Target: x=0.0, y=0.0, z=-3.0 (NED)'
        )

        self.get_logger().info(
            'Sequence: setpoints -> OFFBOARD -> ARM'
        )

    # ==============================================================
    # PX4 callbacks
    # ==============================================================

    def vehicle_status_callback(self, msg: VehicleStatus):
        self.arming_state = msg.arming_state
        self.nav_state = msg.nav_state

    def vehicle_local_position_callback(
        self,
        msg: VehicleLocalPosition,
    ):
        self.current_x = msg.x
        self.current_y = msg.y
        self.current_z = msg.z

    def command_ack_callback(self, msg: VehicleCommandAck):
        if msg.command == VehicleCommand.VEHICLE_CMD_DO_SET_MODE:
            self.get_logger().info(
                f'OFFBOARD command ACK result={msg.result}'
            )

        elif msg.command == VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM:
            self.get_logger().info(
                f'ARM command ACK result={msg.result}'
            )

    # ==============================================================
    # OffboardControlMode
    # ==============================================================

    def publish_offboard_control_mode(self):

        msg = OffboardControlMode()

        msg.timestamp = (
            self.get_clock().now().nanoseconds // 1000
        )

        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.thrust_and_torque = False
        msg.direct_actuator = False

        self.offboard_control_mode_pub.publish(msg)

    # ==============================================================
    # TrajectorySetpoint
    # ==============================================================

    def publish_trajectory_setpoint(self):

        msg = TrajectorySetpoint()

        msg.timestamp = (
            self.get_clock().now().nanoseconds // 1000
        )

        # PX4 NED:
        #
        # x = North
        # y = East
        # z = Down
        #
        # z = -3.0 => 3 meters above home
        msg.position = [
            0.0,
            0.0,
            -3.0,
        ]

        msg.yaw = 0.0

        self.trajectory_setpoint_pub.publish(msg)

    # ==============================================================
    # VehicleCommand
    # ==============================================================

    def publish_vehicle_command(
        self,
        command: int,
        param1: float = 0.0,
        param2: float = 0.0,
    ):

        msg = VehicleCommand()

        msg.timestamp = (
            self.get_clock().now().nanoseconds // 1000
        )

        msg.command = command

        msg.param1 = param1
        msg.param2 = param2

        msg.target_system = 1
        msg.target_component = 1

        msg.source_system = 1
        msg.source_component = 1

        msg.from_external = True

        self.vehicle_command_pub.publish(msg)

    # ==============================================================
    # Request OFFBOARD
    # ==============================================================

    def request_offboard(self):

        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
            1.0,
            6.0,
        )

        self.offboard_requested = True

        self.get_logger().info(
            'Requested OFFBOARD mode.'
        )

    # ==============================================================
    # Request ARM
    # ==============================================================

    def request_arm(self):

        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            1.0,
            0.0,
        )

        self.arm_requested = True

        self.get_logger().info(
            'Requested ARM.'
        )

    # ==============================================================
    # Main loop
    # ==============================================================

    def timer_callback(self):

        # ----------------------------------------------------------
        # Always publish Offboard heartbeat and setpoint.
        # ----------------------------------------------------------

        self.publish_offboard_control_mode()
        self.publish_trajectory_setpoint()

        self.counter += 1

        # ----------------------------------------------------------
        # First 2 seconds:
        # build up Offboard setpoint stream
        # ----------------------------------------------------------

        if self.counter < 40:
            return

        # ----------------------------------------------------------
        # Request OFFBOARD
        # ----------------------------------------------------------

        if not self.offboard_requested:
            self.request_offboard()
            return

        # ----------------------------------------------------------
        # Wait for PX4 to actually report OFFBOARD.
        # ----------------------------------------------------------

        if self.nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            return

        # ----------------------------------------------------------
        # Wait another 0.5 sec after OFFBOARD.
        # ----------------------------------------------------------

        if self.counter < 50:
            return

        # ----------------------------------------------------------
        # ARM
        # ----------------------------------------------------------

        if not self.arm_requested:
            self.request_arm()
            return

        # ----------------------------------------------------------
        # Periodic state report.
        # ----------------------------------------------------------

        if self.counter % 100 == 0:

            self.get_logger().info(
                f'PX4 state: '
                f'nav_state={self.nav_state}, '
                f'arming_state={self.arming_state}, '
                f'position='
                f'({self.current_x:.2f}, '
                f'{self.current_y:.2f}, '
                f'{self.current_z:.2f})'
            )


def main(args=None):

    rclpy.init(args=args)

    node = OffboardControl()

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
