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
