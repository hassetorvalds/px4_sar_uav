"""命令生命周期跟踪的单测。"""

import pytest

from mission.command_tracker import (CommandState, CommandTracker,
                                     RESULT_ACCEPTED, RESULT_IN_PROGRESS)


def test_submit_and_accept():
    tracker = CommandTracker(ack_timeout_s=1.0)
    record = tracker.submit('arm', 400, now=0.0)
    assert record.state == CommandState.AWAITING_ACK.value
    assert record.attempts == 1
    acked = tracker.on_ack(400, RESULT_ACCEPTED, now=0.2)
    assert acked is record
    assert record.state == CommandState.ACCEPTED.value
    assert record.latency_s == pytest.approx(0.2)


def test_rejected_ack():
    tracker = CommandTracker()
    tracker.submit('offboard', 176, now=0.0)
    record = tracker.on_ack(176, 2, now=0.1)          # DENIED
    assert record.state == CommandState.REJECTED.value
    assert 'result=2' in record.reason


def test_in_progress_keeps_awaiting():
    tracker = CommandTracker()
    tracker.submit('arm', 400, now=0.0)
    record = tracker.on_ack(400, RESULT_IN_PROGRESS, now=0.1)
    assert record.state == CommandState.AWAITING_ACK.value
    assert record.reason == 'in_progress'


def test_timeout_retry_then_delivery_unknown():
    tracker = CommandTracker(ack_timeout_s=1.0, max_attempts=3)
    tracker.submit('arm', 400, now=0.0)
    first = tracker.tick(now=2.0)
    assert [record.state for record in first] == [CommandState.TIMEOUT.value]
    assert tracker.get('arm').attempts == 1

    tracker.submit('arm', 400, now=2.0)
    second = tracker.tick(now=4.0)
    assert [record.state for record in second] == [CommandState.TIMEOUT.value]
    assert tracker.get('arm').attempts == 2

    tracker.submit('arm', 400, now=4.0)
    third = tracker.tick(now=6.0)
    assert [record.state for record in third] == [CommandState.DELIVERY_UNKNOWN.value]
    assert tracker.get('arm').reason == 'ack_timeout_max_attempts'


def test_expect_ack_false_goes_straight_to_executing():
    tracker = CommandTracker()
    record = tracker.submit('land_fast', None, expect_ack=False, now=0.0)
    assert record.state == CommandState.EXECUTING.value
    assert tracker.tick(now=99.0) == []


def test_mark_executing_completed():
    tracker = CommandTracker()
    tracker.submit('land_fast', None, expect_ack=False, now=0.0)
    tracker.mark_executing('land_fast')
    tracker.mark_completed('land_fast')
    assert tracker.get('land_fast').state == CommandState.COMPLETED.value


def test_ack_matches_latest_pending_for_same_command_id():
    tracker = CommandTracker()
    tracker.submit('arm', 400, now=0.0)
    tracker.submit('disarm_force', 400, now=0.5)
    tracker.on_ack(400, RESULT_ACCEPTED, now=0.6)
    assert tracker.get('disarm_force').state == CommandState.ACCEPTED.value
    assert tracker.get('arm').state == CommandState.AWAITING_ACK.value


def test_unknown_ack_is_ignored():
    tracker = CommandTracker()
    assert tracker.on_ack(999, RESULT_ACCEPTED) is None


def test_to_dicts_contains_lifecycle_fields():
    tracker = CommandTracker()
    tracker.submit('offboard', 176, now=0.0)
    payload = tracker.to_dicts()[0]
    assert payload['name'] == 'offboard'
    assert payload['vehicle_command'] == 176
    assert payload['state'] == CommandState.AWAITING_ACK.value
    assert payload['attempts'] == 1
