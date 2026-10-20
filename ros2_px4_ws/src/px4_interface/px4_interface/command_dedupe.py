"""落地命令去重：避免 executor/supervisor 在 2 s 内双发 land_fast。"""

import time


class CommandDeduper:
    def __init__(self, window_s: float = 2.0):
        self._window = max(float(window_s), 0.0)
        self._last = {}

    def accept(self, command: str, now: float = None) -> bool:
        command = (command or '').strip().lower()
        if command not in ('land', 'land_fast'):
            return True
        now = time.monotonic() if now is None else float(now)
        last = self._last.get(command)
        if last is not None and now - last < self._window:
            return False
        self._last[command] = now
        return True
