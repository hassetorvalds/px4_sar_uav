"""命令生命周期跟踪：**发送 ≠ 完成**。

状态机（参考 QuickScout / MDS 的 command lifecycle，裁剪到当前需要）：

    QUEUED → AWAITING_ACK → ACCEPTED → EXECUTING → COMPLETED
                          ├→ REJECTED
                          └→ TIMEOUT（可重试）→ … → DELIVERY_UNKNOWN

``land_fast`` 这类"由 bridge 内部用速度指令实现、没有单条 MAVLink 命令 ACK"的动作，
用 ``expect_ack=False`` 直接进入 EXECUTING，由"是否上锁"来判定完成。

纯逻辑、不依赖 ROS，可单测。
"""

import time
from dataclasses import dataclass
from enum import Enum

RESULT_ACCEPTED = 0
RESULT_TEMPORARILY_REJECTED = 1
RESULT_IN_PROGRESS = 5


class CommandState(str, Enum):
    QUEUED = 'queued'
    AWAITING_ACK = 'awaiting_ack'
    ACCEPTED = 'accepted'
    REJECTED = 'rejected'
    TIMEOUT = 'timeout'
    EXECUTING = 'executing'
    COMPLETED = 'completed'
    FAILED = 'failed'
    DELIVERY_UNKNOWN = 'delivery_unknown'


@dataclass
class CommandRecord:
    name: str
    vehicle_command: int = None
    state: str = CommandState.QUEUED.value
    sent_at: float = 0.0
    ack_at: float = None
    attempts: int = 0
    result_code: int = None
    expect_ack: bool = True
    reason: str = ''

    @property
    def latency_s(self):
        if self.ack_at is None or self.sent_at is None:
            return None
        return round(self.ack_at - self.sent_at, 3)

    def to_dict(self):
        return {
            'name': self.name,
            'vehicle_command': self.vehicle_command,
            'state': self.state,
            'attempts': self.attempts,
            'expect_ack': self.expect_ack,
            'result_code': self.result_code,
            'latency_s': self.latency_s,
            'reason': self.reason,
        }


class CommandTracker:
    def __init__(self, *, ack_timeout_s: float = 1.5, max_attempts: int = 3):
        self.ack_timeout_s = float(ack_timeout_s)
        self.max_attempts = max(int(max_attempts), 1)
        self._records = {}

    def submit(self, name: str, vehicle_command: int = None, *,
               expect_ack: bool = True, now: float = None) -> CommandRecord:
        """登记一次发送。重复 submit 视为重试（attempts += 1）。"""
        now = time.monotonic() if now is None else float(now)
        record = self._records.get(name)
        if record is None:
            record = CommandRecord(name=name, vehicle_command=vehicle_command)
            self._records[name] = record
        else:
            record.attempts = max(record.attempts, 0)
            record.vehicle_command = vehicle_command or record.vehicle_command
            record.ack_at = None
            record.result_code = None
            record.reason = ''
        record.attempts += 1
        record.sent_at = now
        record.expect_ack = bool(expect_ack)
        record.state = (CommandState.AWAITING_ACK.value if expect_ack
                        else CommandState.EXECUTING.value)
        return record

    def on_ack(self, vehicle_command: int, result_code: int, *,
               now: float = None):
        """收到 PX4 ACK：只认**最近一次**同 command id 的 AWAITING_ACK。"""
        now = time.monotonic() if now is None else float(now)
        candidates = [record for record in self._records.values()
                      if record.vehicle_command == vehicle_command
                      and record.state == CommandState.AWAITING_ACK.value]
        if not candidates:
            return None
        record = max(candidates, key=lambda item: item.sent_at)
        record.ack_at = now
        record.result_code = int(result_code)
        if int(result_code) == RESULT_ACCEPTED:
            record.state = CommandState.ACCEPTED.value
        elif int(result_code) == RESULT_IN_PROGRESS:
            record.reason = 'in_progress'
        elif int(result_code) == RESULT_TEMPORARILY_REJECTED:
            record.state = CommandState.TIMEOUT.value
            record.reason = 'temporarily_rejected'
        else:
            record.state = CommandState.REJECTED.value
            record.reason = f'result={result_code}'
        return record

    def tick(self, now: float = None):
        """把超过 ACK 超时的记录推到 TIMEOUT / DELIVERY_UNKNOWN。"""
        now = time.monotonic() if now is None else float(now)
        changed = []
        for record in self._records.values():
            if record.state != CommandState.AWAITING_ACK.value:
                continue
            if now - record.sent_at <= self.ack_timeout_s:
                continue
            if record.attempts < self.max_attempts:
                record.state = CommandState.TIMEOUT.value
                record.reason = 'ack_timeout'
            else:
                record.state = CommandState.DELIVERY_UNKNOWN.value
                record.reason = 'ack_timeout_max_attempts'
            changed.append(record)
        return changed

    def mark(self, name: str, state: CommandState, reason: str = ''):
        record = self._records.get(name)
        if record is None:
            return None
        record.state = state.value
        if reason:
            record.reason = reason
        return record

    def mark_executing(self, name: str, reason: str = ''):
        return self.mark(name, CommandState.EXECUTING, reason)

    def mark_completed(self, name: str, reason: str = ''):
        return self.mark(name, CommandState.COMPLETED, reason)

    def mark_failed(self, name: str, reason: str = ''):
        return self.mark(name, CommandState.FAILED, reason)

    def get(self, name: str):
        return self._records.get(name)

    def pending(self):
        return [record for record in self._records.values()
                if record.state in (CommandState.QUEUED.value,
                                    CommandState.AWAITING_ACK.value,
                                    CommandState.ACCEPTED.value,
                                    CommandState.EXECUTING.value)]

    def to_dicts(self):
        return [record.to_dict() for record in self._records.values()]
