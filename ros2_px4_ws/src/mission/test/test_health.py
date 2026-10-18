"""起飞前健康检查的单测。"""

import pytest

from mission.health import check_prearm


def _kwargs(**overrides):
    values = dict(status_fresh=True, status_age_s=0.1,
                  position_valid=True, position_age_s=0.1,
                  preflight_ok=True, failsafe=False,
                  battery_ratio=0.9, battery_warning=0, battery_age_s=0.2)
    values.update(overrides)
    return values


def test_all_good():
    report = check_prearm(**_kwargs())
    assert report.ok
    assert report.reasons() == []


def test_stale_status_fails():
    report = check_prearm(**_kwargs(status_fresh=False, status_age_s=3.0))
    assert not report.ok
    assert any('px4_status' in reason for reason in report.reasons())


def test_position_stale_or_invalid_fails():
    assert not check_prearm(**_kwargs(position_valid=False)).ok
    assert not check_prearm(**_kwargs(position_age_s=2.0)).ok


def test_preflight_and_failsafe_fail():
    assert not check_prearm(**_kwargs(preflight_ok=False)).ok
    assert not check_prearm(**_kwargs(failsafe=True)).ok


def test_low_battery_fails():
    report = check_prearm(**_kwargs(battery_ratio=0.1))
    assert not report.ok
    assert any('battery' in reason for reason in report.reasons())


def test_battery_warning_fails():
    report = check_prearm(**_kwargs(battery_warning=1))
    assert not report.ok


def test_missing_battery_is_warn_by_default():
    report = check_prearm(**_kwargs(battery_ratio=None, battery_age_s=None))
    assert report.ok
    battery = [check for check in report.checks if check.name == 'battery'][0]
    assert battery.severity == 'warn'


def test_missing_battery_fails_when_required():
    report = check_prearm(**_kwargs(battery_ratio=None, battery_age_s=None,
                                    require_battery=True))
    assert not report.ok


def test_stale_battery_reading_fails_when_required():
    report = check_prearm(**_kwargs(battery_age_s=99.0, require_battery=True))
    assert not report.ok


def test_report_to_dict_contains_checks_and_reasons():
    report = check_prearm(**_kwargs(preflight_ok=False))
    payload = report.to_dict()
    assert payload['ok'] is False
    assert payload['reasons']
    assert any(check['name'] == 'preflight_checks' for check in payload['checks'])
