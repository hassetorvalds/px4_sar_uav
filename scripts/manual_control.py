#!/usr/bin/env python3
"""键盘手动接管工具（方案 A：键盘 → MAVLink → PX4，**不经过 ROS 2**）。

按键映射（可改，见 KEY_MAP）：
    W / S        油门：上升 / 下降
    A / D        偏航：向左 / 向右
    ↑ / ↓        俯仰：前进 / 后退
    ← / →        横滚：向左 / 向右
    F9           进入人工接管（切 POSCTL）
    F10          交还控制（切 OFFBOARD）
    q 或 Ctrl-C  退出（摇杆回中）

用法：
    交互式（手动飞）:   python3 scripts/manual_control.py
    脚本化（自动化验证）: python3 scripts/manual_control.py --script "takeover;forward:0.6:2;hover:1;handback"

为什么用 MAVLink 而不是 ROS 2：人工接管必须不依赖 ROS 2——
本脚本只与 PX4 的 MAVLink 实例通信，杀掉全部 ROS 2 节点后依然可用。

前置（PX4 每次重启后需在 pxh 里执行一次）：
    mavlink start -u 14700 -o 14701 -r 1000000 -m onboard
"""

import argparse
import os
import select
import sys
import termios
import time
import tty

from pymavlink import mavutil

MAVLINK_LISTEN_PORT = 14701
MAVLINK_TARGET = ('127.0.0.1', 14700)
SYSTEM_ID = 245
COMPONENT_ID = 190

#: PX4 custom_mode 编码：(主模式 << 16) | (子模式 << 24)
MODE_POSCTL = 3
MODE_OFFBOARD = 6

#: 按键 → (pitch, roll, throttle, yaw) 的归一化增量（-1..1，油门 0 表示保持）
KEY_MAP = {
    'w': ('throttle', +1.0),
    's': ('throttle', -1.0),
    'a': ('yaw', -1.0),
    'd': ('yaw', +1.0),
    '\x1b[A': ('pitch', +1.0),   # ↑ 前进
    '\x1b[B': ('pitch', -1.0),   # ↓ 后退
    '\x1b[D': ('roll', -1.0),    # ← 左
    '\x1b[C': ('roll', +1.0),    # → 右
}
TAKEOVER_KEYS = ('\x1b[20~',)    # F9
HANDBACK_KEYS = ('\x1b[21~',)    # F10
QUIT_KEYS = ('q', '\x03')


class ManualControl:
    def __init__(self, scale: float = 0.5, rate_hz: float = 20.0, verbose: bool = True):
        self.scale = max(0.05, min(scale, 1.0))
        self.period = 1.0 / max(rate_hz, 1.0)
        self.verbose = verbose
        self.link = mavutil.mavlink_connection(
            f'udpin:0.0.0.0:{MAVLINK_LISTEN_PORT}',
            source_system=SYSTEM_ID, source_component=COMPONENT_ID)
        self.sock = self.link.port
        self.mode = None
        self.manual_active = False
        self.sticks = {'pitch': 0.0, 'roll': 0.0, 'throttle': 0.0, 'yaw': 0.0}
        self.last_heartbeat = 0.0
        self.position = None
        self.position_start = None

    # ------------------------------------------------------------------ 通信

    def _send(self, msg) -> None:
        self.sock.sendto(msg.pack(self.link.mav), MAVLINK_TARGET)

    def send_heartbeat(self) -> None:
        self._send(self.link.mav.heartbeat_encode(
            mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0))
        self.last_heartbeat = time.monotonic()

    def pump(self) -> None:
        """收发一轮：维持心跳、上报摇杆、跟踪模式变化。"""
        now = time.monotonic()
        if now - self.last_heartbeat > 1.0:
            self.send_heartbeat()
        # 始终推送手动输入流（未接管时摇杆为中位）：
        # PX4 需要先看到有效的手动输入源，才会接受进入 POSCTL 的模式切换。
        self._send(self.link.mav.manual_control_encode(
            1,
            int(self.sticks['pitch'] * 1000 * self.scale),
            int(self.sticks['roll'] * 1000 * self.scale),
            int(500 + self.sticks['throttle'] * 400 * self.scale),
            int(self.sticks['yaw'] * 1000 * self.scale),
            0))
        msg = self.link.recv_match(type=['HEARTBEAT', 'LOCAL_POSITION_NED'],
                                   blocking=True, timeout=0.02)
        if msg is None:
            return
        if msg.get_type() == 'LOCAL_POSITION_NED':
            self.position = (msg.x, msg.y, msg.z)
            if self.position_start is None:
                self.position_start = self.position
            return
        if msg.get_srcSystem() == 1:
            main_mode = (msg.custom_mode >> 16) & 0xFF
            if main_mode != self.mode:
                self.mode = main_mode
                if self.verbose:
                    name = {0: 'MANUAL', 1: 'ALTCTL', 2: 'POSCTL', 6: 'OFFBOARD'}.get(
                        main_mode, str(main_mode))
                    print(f'[模式] 当前 = {name} (custom_mode={msg.custom_mode})')

    def set_mode(self, main_mode: int) -> bool:
        """请求模式切换；失败会重试（PX4 需要先收到有效手动输入源）。"""
        deadline = time.monotonic() + 8.0
        next_try = 0.0
        while time.monotonic() < deadline:
            self.pump()
            if time.monotonic() >= next_try:
                self._send(self.link.mav.command_long_encode(
                    1, 1, mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0, 1, main_mode,
                    0, 0, 0, 0, 0))
                next_try = time.monotonic() + 1.0
            if self.mode == main_mode:
                return True
        return False

    # ------------------------------------------------------------ 接管/交还

    def takeover(self) -> bool:
        print('→ 请求人工接管（POSCTL）')
        ok = self.set_mode(MODE_POSCTL)
        self.manual_active = True
        print('  接管' + ('成功' if ok else '未确认（仍继续发送摇杆）'))
        return ok

    def handback(self) -> bool:
        print('→ 交还控制（OFFBOARD）')
        self.manual_active = False
        self.release_all()
        ok = self.set_mode(MODE_OFFBOARD)
        print('  交还' + ('成功' if ok else '未确认'))
        return ok

    def release_all(self) -> None:
        for key in self.sticks:
            self.sticks[key] = 0.0

    # ------------------------------------------------------------------ 输入

    def _read_key(self, timeout: float) -> str:
        ready, _, _ = select.select([sys.stdin], [], [], timeout)
        if not ready:
            return ''
        char = os.read(sys.stdin.fileno(), 1).decode('utf-8', errors='ignore')
        if char == '\x1b':      # 可能是功能键/方向键序列
            seq = char
            for _ in range(4):
                ready, _, _ = select.select([sys.stdin], [], [], 0.02)
                if not ready:
                    break
                seq += os.read(sys.stdin.fileno(), 1).decode('utf-8', errors='ignore')
                if seq.endswith('~') or (len(seq) == 3 and seq[1] == '['):
                    break
            return seq
        return char

    def interactive(self) -> None:
        fd = sys.stdin.fileno()
        saved = termios.tcgetattr(fd)
        print(__doc__)
        print('等待按键（F9 接管 / F10 交还 / q 退出）…')
        try:
            tty.setcbreak(fd)
            while True:
                key = self._read_key(0.05)
                if key:
                    if key in QUIT_KEYS:
                        break
                    if key in TAKEOVER_KEYS:
                        self.takeover()
                        continue
                    if key in HANDBACK_KEYS:
                        self.handback()
                        continue
                    action = KEY_MAP.get(key)
                    if action:
                        channel, value = action
                        self.sticks[channel] = value
                        if self.verbose:
                            print(f'[摇杆] {channel} = {value:+.1f}')
                else:
                    # 松键即回中
                    if any(self.sticks.values()):
                        self.release_all()
                        if self.verbose:
                            print('[摇杆] 回中')
                self.pump()
        except KeyboardInterrupt:
            pass
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, saved)
            if self.manual_active:
                self.handback()

    # ------------------------------------------------------------------ 脚本

    def run_script(self, script: str) -> None:
        """脚本化输入序列，用于自动化验证。

        指令：takeover | handback | hover:秒 | <通道>:<值>:<秒> | wait:秒
        例如：takeover;forward:0.6:2;hover:1;handback（forward = pitch 前进）
        """
        print(f'脚本模式：{script}')
        for step in [s.strip() for s in script.split(';') if s.strip()]:
            parts = step.split(':')
            name = parts[0]
            if name == 'takeover':
                self.takeover()
            elif name == 'handback':
                self.handback()
            elif name == 'wait':
                self._hold(float(parts[1]))
            elif name == 'hover':
                self.release_all()
                self._hold(float(parts[1]))
            else:
                channel = NAME_TO_CHANNEL.get(name)
                if channel is None:
                    print(f'  未知指令：{step}')
                    continue
                self.sticks[channel] = float(parts[1])
                print(f'  {name} = {parts[1]}（{channel}）持续 {parts[2]} s')
                self._hold(float(parts[2]))
                self.release_all()
        print('脚本结束')
        if self.position and self.position_start:
            d = [self.position[i] - self.position_start[i] for i in range(3)]
            print(f'位移（NED）：Δn={d[0]:+.2f} Δe={d[1]:+.2f} Δd={d[2]:+.2f} m')

    def _hold(self, seconds: float) -> None:
        end = time.monotonic() + seconds
        next_report = time.monotonic() + 2.0
        while time.monotonic() < end:
            self.pump()
            time.sleep(self.period)
            if time.monotonic() >= next_report:
                next_report = time.monotonic() + 2.0
                if self.position:
                    print(f'  [{time.strftime("%H:%M:%S")}] pos(NED)='
                          f'({self.position[0]:+.2f}, {self.position[1]:+.2f}, '
                          f'{self.position[2]:+.2f}) mode={self.mode}')


NAME_TO_CHANNEL = {
    'forward': 'pitch', 'backward': 'pitch',
    'left': 'roll', 'right': 'roll',
    'up': 'throttle', 'down': 'throttle',
    'yaw_left': 'yaw', 'yaw_right': 'yaw',
}


def main() -> int:
    parser = argparse.ArgumentParser(description='键盘手动接管（MAVLink 直连 PX4）')
    parser.add_argument('--script', help='脚本化输入序列（见 run_script 文档）')
    parser.add_argument('--scale', type=float, default=0.5, help='摇杆幅度系数（0.05~1.0）')
    parser.add_argument('--rate', type=float, default=20.0, help='上报频率 Hz')
    args = parser.parse_args()

    controller = ManualControl(scale=args.scale, rate_hz=args.rate)
    controller.send_heartbeat()
    time.sleep(0.3)
    if args.script:
        controller.run_script(args.script)
    else:
        controller.interactive()
    return 0


if __name__ == '__main__':
    sys.exit(main())
