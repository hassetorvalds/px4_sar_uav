"""findings 持久化/汇总单测。"""

import json

from mission.findings_store import (load_records, persist_findings,
                                    render_summary, summarize_records)


def _finding(world=(1, 2, 3), status='inspected'):
    return {'finding_id': 1, 'label': 'target', 'world_enu': list(world),
            'confidence': 1.0, 'priority': 'unknown', 'status': status,
            'first_seen': 1.0, 'last_seen': 2.0}


def test_persist_writes_json_and_index(tmp_path):
    info = persist_findings(
        findings=[_finding()], task_summary='scan',
        report_path='logs/tasks/report_demo.jsonl',
        findings_dir=str(tmp_path), finished_at=100.0)
    assert info['task_id'] == 'demo'
    assert (tmp_path / 'findings_demo.json').is_file()
    assert (tmp_path / 'index.jsonl').is_file()
    record = json.loads((tmp_path / 'findings_demo.json').read_text())
    assert record['task_summary'] == 'scan'
    assert record['findings'][0]['status'] == 'inspected'


def test_load_records_dedups_by_task_id(tmp_path):
    persist_findings(findings=[_finding()], task_summary='a',
                     report_path='report_a.jsonl', findings_dir=str(tmp_path),
                     finished_at=1.0)
    persist_findings(findings=[_finding()], task_summary='a2',
                     report_path='report_a.jsonl', findings_dir=str(tmp_path),
                     finished_at=2.0)
    persist_findings(findings=[_finding()], task_summary='b',
                     report_path='report_b.jsonl', findings_dir=str(tmp_path),
                     finished_at=3.0)
    records = load_records(str(tmp_path))
    assert len(records) == 2
    assert records[0]['task_summary'] == 'b'


def test_summarize_counts():
    records = [
        {'task_id': 'a', 'finished_at': 1.0, 'findings': [_finding()]},
        {'task_id': 'b', 'finished_at': 2.0, 'findings': []},
        {'task_id': 'c', 'finished_at': 3.0,
         'findings': [_finding(status='confirmed'), _finding(world=(9, 9, 9))]},
    ]
    summary = summarize_records(records)
    assert summary['tasks'] == 3
    assert summary['tasks_with_findings'] == 2
    assert summary['findings_total'] == 3
    assert summary['by_status'] == {'confirmed': 1, 'inspected': 2}


def test_render_summary_contains_rows():
    summary = summarize_records([
        {'task_id': 'a', 'finished_at': 1.0, 'findings': [_finding()]}])
    text = render_summary(summary)
    assert '# 发现物汇总' in text
    assert '| a |' in text
    assert '(1.00, 2.00, 3.00)' in text


def test_render_empty_summary():
    text = render_summary(summarize_records([]))
    assert '任务数：0' in text
    assert '| — |' in text
