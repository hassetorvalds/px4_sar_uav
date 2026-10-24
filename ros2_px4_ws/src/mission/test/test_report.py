"""任务报告汇总（JSONL → Markdown）的单测。"""

import json

from mission.report import summarize


def _write_report(path):
    events = [
        {'t': 0.0, 'event': 'task_accepted', 'summary': '起飞 2.0 m（REL） → 扫描 → 降落',
         'spec': {'altitude_m': 2.0, 'altitude_frame': 'REL', 'max_speed_mps': 0.5,
                  'geofence_radius_m': 40.0}},
        {'t': 0.5, 'event': 'health_check', 'ok': True,
         'checks': [{'name': 'px4_status', 'ok': True, 'detail': 'age=0.05s'}]},
        {'t': 6.0, 'event': 'takeoff_complete',
         'altitude_lcl': {'value_m': 2.0}, 'target': {'value_m': 2.0, 'frame': 'REL'}},
        {'t': 8.0, 'event': 'target_confirmed', 'world_enu': [1.0, 2.0, 3.0],
         'step_type': 'scan', 'target_demo_kind': 'proxy'},
        {'t': 9.0, 'event': 'target_centered', 'error_deg': 2.0},
        {'t': 10.0, 'event': 'scan_complete'},
        {'t': 12.0, 'event': 'done', 'commands': [
            {'name': 'arm', 'state': 'completed', 'attempts': 1,
             'result_code': 0, 'latency_s': 0.01}],
         'findings': [{'finding_id': 1, 'label': 'target',
                       'world_enu': [1.0, 2.0, 3.0], 'confidence': 1.0,
                       'priority': 'unknown', 'status': 'inspected',
                       'first_seen': 8.0, 'last_seen': 9.0}],
         'findings_path': 'logs/findings/findings_demo.json',
         'coverage': {'observed_cells': 6, 'total_cells': 24,
                      'coverage_ratio': 0.25, 'missed_cells': 18,
                      'observed_area_m2': 6.0}},
    ]
    with open(path, 'w', encoding='utf-8') as handle:
        for event in events:
            handle.write(json.dumps(event) + '\n')


def test_summarize_contains_target_and_commands(tmp_path):
    report = tmp_path / 'report.jsonl'
    _write_report(report)
    output = summarize(str(report))
    text = open(output, encoding='utf-8').read()
    assert '# 任务报告' in text
    assert '成功完成' in text
    assert 'world=(1.00, 2.00, 3.00)' in text
    assert '目标类型：**代理目标（非人员检测）**' in text
    assert '已对准' in text
    assert '| arm | completed | 1 | 0 |' in text
    assert '## 发现物' in text
    assert '| 1 | target | (1.00, 2.00, 3.00)' in text
    assert 'findings_demo.json' in text
    assert '观测覆盖：6/24 格（25.0%）' in text


def test_summarize_abort(tmp_path):
    report = tmp_path / 'report.jsonl'
    _write_report(report)
    with open(report, 'a', encoding='utf-8') as handle:
        handle.write(json.dumps({'t': 13.0, 'event': 'abort',
                                 'reason': 'clearance_blocked'}) + '\n')
    output = summarize(str(report))
    text = open(output, encoding='utf-8').read()
    assert 'clearance_blocked' in text


def test_summarize_multi_region_and_transit(tmp_path):
    report = tmp_path / 'multi.jsonl'
    events = [
        {'t': 0.0, 'event': 'task_accepted', 'summary': 'two regions',
         'spec': {'altitude_m': 2.0, 'altitude_frame': 'REL'}},
        {'t': 5.0, 'event': 'search_plan', 'step': 1, 'waypoints': 8,
         'area_m2': 20.0, 'path_length_m': 20.0, 'estimated_s': 60.0},
        {'t': 6.0, 'event': 'search_coverage', 'step': 1,
         'observed_cells': 80, 'total_cells': 80, 'coverage_ratio': 1.0,
         'missed_cells': 0},
        {'t': 7.0, 'event': 'transit', 'step': 2, 'distance_m': 11.0},
        {'t': 8.0, 'event': 'search_plan', 'step': 2, 'waypoints': 6,
         'area_m2': 16.0, 'path_length_m': 24.0, 'estimated_s': 50.0},
        {'t': 9.0, 'event': 'search_coverage', 'step': 2,
         'observed_cells': 60, 'total_cells': 64, 'coverage_ratio': 0.9375,
         'missed_cells': 4},
        {'t': 10.0, 'event': 'done', 'commands': [], 'findings': []},
    ]
    with open(report, 'w', encoding='utf-8') as handle:
        for event in events:
            handle.write(json.dumps(event) + '\n')
    text = open(summarize(str(report)), encoding='utf-8').read()
    assert '## 搜索区域' in text
    assert '2 个区域，合计 36.0 m²' in text
    assert '观测覆盖（合计）：140/144 格（97.2%）' in text
    assert '转场：1 段，合计 11.0 m' in text
    assert '| 1 | 20.0 | 8 | 80/80（100.0%） | 0 |' in text
    assert '| 2 | 16.0 | 6 | 60/64（93.8%） | 4 |' in text
