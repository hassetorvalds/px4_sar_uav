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
         'step_type': 'scan'},
        {'t': 9.0, 'event': 'target_centered', 'error_deg': 2.0},
        {'t': 10.0, 'event': 'scan_complete'},
        {'t': 12.0, 'event': 'done', 'commands': [
            {'name': 'arm', 'state': 'completed', 'attempts': 1,
             'result_code': 0, 'latency_s': 0.01}],
         'findings': [{'finding_id': 1, 'label': 'target',
                       'world_enu': [1.0, 2.0, 3.0], 'confidence': 1.0,
                       'priority': 'unknown', 'status': 'inspected',
                       'first_seen': 8.0, 'last_seen': 9.0}]},
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
    assert '已对准' in text
    assert '| arm | completed | 1 | 0 |' in text
    assert '## 发现物' in text
    assert '| 1 | target | (1.00, 2.00, 3.00)' in text


def test_summarize_abort(tmp_path):
    report = tmp_path / 'report.jsonl'
    _write_report(report)
    with open(report, 'a', encoding='utf-8') as handle:
        handle.write(json.dumps({'t': 13.0, 'event': 'abort',
                                 'reason': 'clearance_blocked'}) + '\n')
    output = summarize(str(report))
    text = open(output, encoding='utf-8').read()
    assert 'clearance_blocked' in text
