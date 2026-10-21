"""高度门 C：两档 top + 共同漂移不变性。"""

from mission.height_gate import evaluate_height_gate


def _gate(top, mode='c_map_rel', offset=0.0):
    return evaluate_height_gate(
        mode=mode, phase='DESCEND', px4_z=2.0 + offset,
        airsim_z=2.0 + offset, map_z=2.0 + offset,
        cluster_top_z=top + offset, clearance_m=0.5)


def test_c_two_top_levels():
    assert _gate(1.0).overflyable is True    # gap 1.0 >= 0.5
    assert _gate(5.0).overflyable is False   # gap -3.0


def test_c_common_drift_does_not_change_decision():
    for top, expected in ((1.0, True), (5.0, False)):
        for offset in (0.5, -0.5):
            assert _gate(top, offset=offset).overflyable is expected


def test_legacy_never_overflies():
    assert _gate(1.0, mode='legacy').overflyable is False
    assert _gate(5.0, mode='legacy').overflyable is False


def test_gate_records_frame_anchor_transform():
    decision = _gate(1.0)
    assert decision.frame_id == 'map'
    assert decision.anchor_id == 'local_map'
    assert decision.transform_id == 'static_arm'
    assert decision.reason == 'c_map_rel'
