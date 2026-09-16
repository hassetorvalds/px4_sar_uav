"""PX4 状态跟踪：把 /fmu/out 的话题聚合成一个可查询的状态对象。

这是一个**库**而不是节点：任何需要 PX4 状态的节点实例化 :class:`Px4StateMonitor`，
它负责订阅、时间戳与超时判定，调用方只读快照，避免每个模块重复写一遍订阅代码。
"""

import time
from dataclasses import dataclass

from px4_msgs.msg import (
    VehicleLandDetected,
    VehicleLocalPosition,
    VehicleStatus,
)

from .qos import px4_qos


@dataclass
class Px4State:
    """PX4 状态快照。位置为 NED（与 PX4 一致，转换请在调用方使用 frames.py）。"""

    arming_state: int = VehicleStatus.ARMING_STATE_DISARMED
    nav_state: int = VehicleStatus.NAVIGATION_STATE_MANUAL
    failsafe: bool = False
    pre_flight_checks_pass: bool = False
    accepts_offboard_setpoints: bool = False
    x: float = float('nan')
    y: float = float('nan')
    z: float = float('nan')
    heading: float = float('nan')
    xy_valid: bool = False
    z_valid: bool = False
    landed: bool = False

    @property
    def armed(self) -> bool:
        return self.arming_state == VehicleStatus.ARMING_STATE_ARMED

    @property
    def offboard(self) -> bool:
        return self.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD

    @property
    def position_ned(self):
        """当前位置 (x, y, z)，NED；无效时返回 None。"""
        if not (self.xy_valid and self.z_valid):
            return None
        return (self.x, self.y, self.z)

    def is_flying(self) -> bool:
        return self.armed and not self.landed


class Px4StateMonitor:
    """订阅 /fmu/out 话题并维护最新的 :class:`Px4State`。"""

    def __init__(self, node, status_timeout_s: float = 1.0):
        self._node = node
        self._status_timeout_s = status_timeout_s
        self._state = Px4State()
        self._last_status_monotonic = 0.0
        self._last_position_monotonic = 0.0

        qos = px4_qos()
        node.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status_v4', self._on_status, qos)
        node.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position_v1',
            self._on_local_position, qos)
        node.create_subscription(
            VehicleLandDetected, '/fmu/out/vehicle_land_detected',
            self._on_land_detected, qos)

    # ------------------------------------------------------------------
    # 订阅回调
    # ------------------------------------------------------------------

    def _on_status(self, msg: VehicleStatus) -> None:
        s = self._state
        s.arming_state = msg.arming_state
        s.nav_state = msg.nav_state
        s.failsafe = msg.failsafe
        s.pre_flight_checks_pass = msg.pre_flight_checks_pass
        s.accepts_offboard_setpoints = msg.accepts_offboard_setpoints
        self._last_status_monotonic = time.monotonic()

    def _on_local_position(self, msg: VehicleLocalPosition) -> None:
        s = self._state
        s.x = msg.x
        s.y = msg.y
        s.z = msg.z
        s.heading = msg.heading
        s.xy_valid = msg.xy_valid
        s.z_valid = msg.z_valid
        self._last_position_monotonic = time.monotonic()

    def _on_land_detected(self, msg: VehicleLandDetected) -> None:
        self._state.landed = msg.landed

    # ------------------------------------------------------------------
    # 查询接口
    # ------------------------------------------------------------------

    def snapshot(self) -> Px4State:
        return self._state

    def status_age_s(self) -> float:
        """距离最近一次收到 vehicle_status 的秒数；从未收到时返回 inf。"""
        if self._last_status_monotonic == 0.0:
            return float('inf')
        return time.monotonic() - self._last_status_monotonic

    def status_fresh(self) -> bool:
        return self.status_age_s() <= self._status_timeout_s

    def position_fresh(self) -> bool:
        if self._last_position_monotonic == 0.0:
            return False
        return (time.monotonic() - self._last_position_monotonic) <= self._status_timeout_s
