"""findings 模型单测。"""

import pytest

from mission.findings import FindingsBook, priority_for_kind


def test_priority_mapping():
    assert priority_for_kind('person') == 'high'
    assert priority_for_kind('vehicle') == 'medium'
    assert priority_for_kind('any') == 'unknown'
    assert priority_for_kind('') == 'unknown'


def test_add_new_finding():
    book = FindingsBook(merge_radius_m=3.0)
    finding = book.add_or_update(label='target', world_enu=(10, 20, 5),
                                 confidence=0.9, t=1.0, step_type='scan')
    assert finding.finding_id == 1
    assert finding.updates == 1
    assert finding.world_enu == (10, 20, 5)
    assert finding.status == 'confirmed'
    assert len(book) == 1


def test_merge_within_radius_averages_and_updates():
    book = FindingsBook(merge_radius_m=3.0)
    book.add_or_update(label='target', world_enu=(10, 20, 5), t=1.0)
    second = book.add_or_update(label='target', world_enu=(12, 20, 5), t=4.0)
    assert second.updates == 2
    assert second.world_enu == pytest.approx((11, 20, 5))
    assert second.last_seen == 4.0
    assert len(book) == 1


def test_separate_outside_radius():
    book = FindingsBook(merge_radius_m=3.0)
    book.add_or_update(label='target', world_enu=(0, 0, 0))
    book.add_or_update(label='target', world_enu=(10, 0, 0))
    assert len(book) == 2


def test_status_upgrades_but_never_downgrades():
    book = FindingsBook(merge_radius_m=3.0)
    book.add_or_update(label='target', world_enu=(0, 0, 0), status='confirmed')
    book.add_or_update(label='target', world_enu=(0.2, 0, 0), status='candidate')
    assert book.to_dicts()[0]['status'] == 'confirmed'
    book.mark_inspected((0.1, 0, 0), t=5.0)
    assert book.to_dicts()[0]['status'] == 'inspected'
    book.add_or_update(label='target', world_enu=(0.3, 0, 0), status='confirmed')
    assert book.to_dicts()[0]['status'] == 'inspected'


def test_mark_inspected_unknown_returns_none():
    book = FindingsBook(merge_radius_m=1.0)
    assert book.mark_inspected((10, 10, 0)) is None


def test_to_dicts_rounding():
    book = FindingsBook(merge_radius_m=3.0)
    book.add_or_update(label='target', world_enu=(1.23456, 2.34567, 3.45678),
                       confidence=0.98765, t=1.234)
    payload = book.to_dicts()[0]
    assert payload['world_enu'] == [1.23, 2.35, 3.46]
    assert payload['confidence'] == 0.988
    assert payload['first_seen'] == 1.23


def test_merge_window_and_revisit_of():
    book = FindingsBook(merge_radius_m=3.0, merge_window_s=30.0,
                        task_id='task-1', epoch_id='task-1')
    first = book.add_or_update(label='target', world_enu=(10, 20, 0), t=1.0)
    merged = book.add_or_update(label='target', world_enu=(10.5, 20, 0), t=10.0)
    assert merged.finding_id == first.finding_id
    assert merged.updates == 2
    revisited = book.add_or_update(label='target', world_enu=(10.5, 20, 0), t=50.0)
    assert revisited.finding_id != first.finding_id
    assert revisited.revisit_of == first.finding_id
    assert len(book) == 2


def test_per_label_radius():
    book = FindingsBook(merge_radius_m=3.0,
                        radius_by_label={'target': 1.0})
    book.add_or_update(label='target', world_enu=(0, 0, 0), t=1.0)
    book.add_or_update(label='target', world_enu=(1.5, 0, 0), t=2.0)
    assert len(book) == 2


def test_finding_1_fields_and_history():
    book = FindingsBook(task_id='task-1', epoch_id='task-1')
    finding = book.add_or_update(
        label='target', world_enu=(1, 2, 3), t=5.0, frame_seq=7,
        frame_size=(640, 480), ros_stamp_ns=123456,
        pose_at_first_seen={'world_enu': [0, 0, 2], 'heading_deg': 90.0})
    payload = finding.to_dict()
    assert payload['schema_version'] == '1.0'
    assert payload['task_id'] == 'task-1'
    assert payload['epoch_id'] == 'task-1'
    assert payload['frame_seq'] == 7
    assert payload['frame_size'] == [640, 480]
    assert payload['first_seen_ros_ns'] == 123456
    assert payload['pose_at_first_seen']['heading_deg'] == 90.0
    assert payload['history']
    assert payload['discarded'] is False


def test_discarded_is_terminal_flag_not_status():
    book = FindingsBook()
    finding = book.add_or_update(label='target', world_enu=(0, 0, 0), t=1.0)
    book.add_or_update(label='target', world_enu=(0.1, 0, 0), t=2.0,
                       discarded=True, discard_reason='false_positive')
    payload = book.to_dicts()[0]
    assert payload['discarded'] is True
    assert payload['discard_reason'] == 'false_positive'
    assert payload['status'] == finding.status


def test_entry_status_and_detector_hits():
    book = FindingsBook()
    finding = book.add_or_update(
        label='target', world_enu=(0, 0, 0), t=1.0,
        entry_status='confirmed', detector_hits=3)
    payload = finding.to_dict()
    assert payload['entry_status'] == 'confirmed'
    assert payload['detector_hits'] == 3
    book.add_or_update(label='target', world_enu=(0, 0, 0), t=2.0,
                       detector_hits=5)
    assert book.to_dicts()[0]['detector_hits'] == 5
