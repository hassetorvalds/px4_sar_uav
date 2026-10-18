"""IntentFrame / 合并规则的单元测试。"""

import pytest

from mission.intent import IntentFrame, merge_intent, parse_intent


def test_parse_intent_simple():
    frame = parse_intent('起飞到 2 米，向北飞 10 米，降落')
    assert frame.source == 'rules'
    assert frame.confidence == pytest.approx(1.0)
    assert frame.missing == []
    facts = frame.fact_map()
    assert facts['goto[1].north_m'].value == pytest.approx(10.0)
    assert facts['takeoff[0].altitude_m'].value == pytest.approx(2.0)


def test_missing_takeoff_land_altitude_reduces_confidence():
    frame = parse_intent('向北飞 5 米')
    joined = ' '.join(frame.missing)
    assert 'takeoff' in joined
    assert 'land' in joined
    assert 'altitude' in joined
    assert frame.confidence < 1.0


def test_merge_rejects_vlm_override_of_local_distance():
    local = parse_intent('起飞到 2 米，向北飞 10 米，降落')
    remote = parse_intent('起飞到 2 米，向北飞 20 米，降落', source='vlm')
    merged = merge_intent(local, remote)
    assert merged.source == 'merged'
    assert merged.task.steps[1].params['north_m'] == pytest.approx(10.0)
    assert any('拒绝' in warning for warning in merged.warnings)


def test_merge_accepts_target_only_when_local_did_not_specify():
    local = parse_intent('起飞到 2 米，搜索半径 5 米，降落')
    remote = parse_intent('起飞到 2 米，搜索半径 5 米，寻找人员，降落', source='vlm')
    merged = merge_intent(local, remote)
    search = [step for step in merged.task.steps if step.type == 'search'][0]
    assert search.params['target']['kind'] == 'person'
    assert any('接受远端补全' in warning for warning in merged.warnings)


def test_merge_same_values_has_no_rejection_warning():
    local = parse_intent('起飞到 2 米，向北飞 10 米，降落')
    remote = parse_intent('起飞到 2 米，向北飞 10 米，降落', source='vlm')
    merged = merge_intent(local, remote)
    assert not any('拒绝' in warning for warning in merged.warnings)


def test_merge_with_none_returns_local():
    local = parse_intent('起飞到 2 米，降落')
    assert merge_intent(local, None) is local


def test_intent_json_roundtrip():
    frame = parse_intent('起飞到 2 米，向北飞 10 米，降落')
    restored = IntentFrame.from_dict(frame.to_dict())
    assert restored.task.to_dict() == frame.task.to_dict()
    assert restored.source == frame.source
    assert restored.confidence == frame.confidence
    assert len(restored.facts) == len(frame.facts)
