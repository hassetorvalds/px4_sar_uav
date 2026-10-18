"""多帧关联单测（合成检测，不需要 ROS）。"""

import numpy as np
import pytest

from perception.detectors import Detection
from perception.tracking import TargetTracker, bbox_iou


def _det(x1=0.0, y1=0.0, x2=10.0, y2=20.0, score=0.5, label='target'):
    return Detection(x1, y1, x2, y2, score=score, label=label)


def test_iou_identical_and_disjoint():
    assert bbox_iou((0, 0, 10, 10), (0, 0, 10, 10)) == pytest.approx(1.0)
    assert bbox_iou((0, 0, 10, 10), (20, 20, 30, 30)) == 0.0


def test_same_world_merges_and_confirms():
    tracker = TargetTracker(gate_m=2.0, confirm_hits=3)
    for index in range(3):
        tracks = tracker.update([_det()], stamp=index * 0.5,
                                world_points=[[1.0, 2.0, 0.0]])
        assert len(tracks) == 1
    assert len(tracker.confirmed()) == 1
    assert tracker.confirmed()[0].hits == 3
    assert tracker.confirmed()[0].world == pytest.approx([1.0, 2.0, 0.0])


def test_not_confirmed_before_hits():
    tracker = TargetTracker(confirm_hits=3)
    tracker.update([_det()], 0.0, [[1.0, 2.0, 0.0]])
    tracker.update([_det()], 0.5, [[1.0, 2.0, 0.0]])
    assert tracker.confirmed() == []


def test_far_target_creates_second_track():
    tracker = TargetTracker(gate_m=1.0)
    tracker.update([_det()], 0.0, [[0.0, 0.0, 0.0]])
    tracks = tracker.update([_det()], 0.5, [[5.0, 0.0, 0.0]])
    assert len(tracks) == 2


def test_ema_smooths_world_point():
    tracker = TargetTracker(gate_m=5.0, ema_alpha=0.5)
    tracker.update([_det()], 0.0, [[0.0, 0.0, 0.0]])
    tracks = tracker.update([_det()], 0.5, [[2.0, 0.0, 0.0]])
    assert tracks[0].world == pytest.approx([1.0, 0.0, 0.0])


def test_ttl_expires_track():
    tracker = TargetTracker(ttl_s=1.0)
    tracker.update([_det()], 0.0, [[0.0, 0.0, 0.0]])
    assert len(tracker.update([], 2.0)) == 0
    tracks = tracker.update([_det()], 2.1, [[0.0, 0.0, 0.0]])
    assert len(tracks) == 1
    assert tracks[0].track_id == 2


def test_bbox_only_association_without_world():
    tracker = TargetTracker(iou_gate=0.3)
    tracker.update([_det(0, 0, 10, 20)], 0.0, [None])
    tracks = tracker.update([_det(1, 1, 11, 21)], 0.5, [None])
    assert len(tracks) == 1
    assert tracks[0].world is None


def test_to_dict_reports_confirmed():
    tracker = TargetTracker(confirm_hits=2)
    tracker.update([_det()], 0.0, [[1.0, 2.0, 3.0]])
    tracks = tracker.update([_det()], 0.5, [[1.0, 2.0, 3.0]])
    payload = tracks[0].to_dict(2)
    assert payload['confirmed'] is True
    assert payload['world_enu'] == [1.0, 2.0, 3.0]
    assert payload['hits'] == 2
