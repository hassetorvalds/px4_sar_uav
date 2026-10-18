"""起飞前健康检查（纯逻辑，可单测）。

目的：执行器在发 `arm` 之前，必须显式确认
- PX4 状态新鲜；
- 本地位置有效且新鲜；
- 飞控自检通过（`pre_flight_checks_pass`）；
- 没有 failsafe；
- 电池（若提供）不低于阈值、无告警。

任何一项失败都不发 arm；由阶段超时兜底收工（**fail-closed**）。
"""

from dataclasses import dataclass, field


@dataclass
class HealthCheck:
    name: str
    ok: bool
    detail: str = ''
    severity: str = 'fail'      # fail | warn | info

    def to_dict(self):
        return {'name': self.name, 'ok': self.ok, 'detail': self.detail,
                'severity': self.severity}


@dataclass
class PreArmReport:
    checks: list = field(default_factory=list)

    @property
    def ok(self):
        return all(check.ok for check in self.checks if check.severity == 'fail')

    def reasons(self):
        return [f'{check.name}: {check.detail}'
                for check in self.checks if not check.ok and check.severity == 'fail']

    def to_dict(self):
        return {'ok': self.ok, 'checks': [check.to_dict() for check in self.checks],
                'reasons': self.reasons()}


def check_prearm(*, status_fresh: bool, status_age_s: float,
                 position_valid: bool, position_age_s: float,
                 preflight_ok: bool, failsafe: bool,
                 battery_ratio=None, battery_warning=None,
                 battery_age_s=None, min_battery_ratio: float = 0.3,
                 require_battery: bool = False,
                 max_position_age_s: float = 0.5,
                 max_status_age_s: float = 1.0,
                 max_battery_age_s: float = 5.0) -> PreArmReport:
    checks = []

    checks.append(HealthCheck(
        'px4_status', bool(status_fresh) and status_age_s <= max_status_age_s,
        f'age={status_age_s:.2f}s（上限 {max_status_age_s:.1f}s）'
        if status_age_s is not None else '从未收到状态'))

    checks.append(HealthCheck(
        'local_position', bool(position_valid) and position_age_s <= max_position_age_s,
        f'valid={position_valid}, age={position_age_s:.2f}s'
        f'（上限 {max_position_age_s:.1f}s）'))

    checks.append(HealthCheck('preflight_checks', bool(preflight_ok),
                              'pre_flight_checks_pass=%s' % bool(preflight_ok)))

    checks.append(HealthCheck('failsafe', not bool(failsafe),
                              'failsafe=%s' % bool(failsafe)))

    if battery_ratio is None or (battery_age_s is not None
                                 and battery_age_s > max_battery_age_s):
        checks.append(HealthCheck(
            'battery', not require_battery,
            '电池读数缺失/过期' if require_battery else '电池读数缺失（按未知处理）',
            severity='fail' if require_battery else 'warn'))
    else:
        low = float(battery_ratio) < float(min_battery_ratio)
        warned = battery_warning is not None and int(battery_warning) > 0
        checks.append(HealthCheck(
            'battery', not low and not warned,
            f'remaining={float(battery_ratio):.2f}（阈值 {min_battery_ratio:.2f}）, '
            f'warning={battery_warning}'))

    return PreArmReport(checks)
