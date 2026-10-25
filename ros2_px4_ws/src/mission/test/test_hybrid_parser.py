"""Hybrid NL 解析器单测。"""

from mission.hybrid_parser import parse_hybrid_instruction


def test_visual_target_after_takeoff():
    parsed = parse_hybrid_instruction(
        '起飞到 2 米，飞到前方橙色目标附近，悬停 3 秒，降落')
    types = [step['type'] for step in parsed['steps']]
    assert types == ['takeoff', 'visual_goto', 'hover', 'land']
    assert parsed['visual_index'] == 1
    assert parsed['steps'][2]['params']['seconds'] == 3.0
    assert parsed['altitude_m'] == 2.0


def test_visual_target_auto_takeoff_land():
    parsed = parse_hybrid_instruction('飞到前方橙色目标附近')
    types = [step['type'] for step in parsed['steps']]
    assert types == ['takeoff', 'visual_goto', 'land']
    assert any('自动补起飞' in note for note in parsed['notes'])
    assert any('自动补降落' in note for note in parsed['notes'])


def test_visual_target_after_direction():
    parsed = parse_hybrid_instruction(
        '起飞到 2 米，向北飞 5 米，飞到前方橙色目标附近，降落')
    types = [step['type'] for step in parsed['steps']]
    assert types == ['takeoff', 'goto', 'visual_goto', 'land']
    assert parsed['visual_index'] == 2


def test_search_scan_then_visual_target():
    parsed = parse_hybrid_instruction(
        '起飞到 2 米，搜索东侧 5 米、南侧 6 米、半径 3 米的区域，扫描 10 秒，'
        '飞到前方橙色目标附近，降落')
    types = [step['type'] for step in parsed['steps']]
    assert types == ['takeoff', 'search', 'scan', 'visual_goto', 'land']
    assert parsed['visual_index'] == 3
    assert parsed['steps'][2]['params']['seconds'] == 10.0
