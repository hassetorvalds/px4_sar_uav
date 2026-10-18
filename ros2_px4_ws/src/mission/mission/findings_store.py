"""findings 持久化与跨任务汇总。

每个任务结束写一份 `logs/findings/findings_<task_id>.json`，并在
`logs/findings/index.jsonl` 追加一行。`task_findings` CLI 读取 index 做汇总。
纯逻辑，不依赖 ROS。
"""

import argparse
import glob
import json
import os
import time


def _task_id_from_report(report_path: str) -> str:
    base = os.path.basename(report_path or '')
    stem = os.path.splitext(base)[0]
    if stem.startswith('report_'):
        stem = stem[len('report_'):]
    if stem.endswith('.report'):
        stem = stem[:-len('.report')]
    if stem:
        return stem
    return time.strftime('%Y%m%d-%H%M%S') + f'-{os.getpid()}'


def persist_findings(*, findings, task_summary: str, report_path: str,
                     intent_path: str = '', findings_dir: str = 'logs/findings',
                     started_at=None, finished_at=None,
                     task_id: str = '') -> dict:
    """原子写单任务 findings，并追加 index；返回路径与 task_id。"""
    os.makedirs(findings_dir, exist_ok=True)
    task_id = task_id or _task_id_from_report(report_path)
    record = {
        'task_id': task_id,
        'task_summary': task_summary,
        'report_path': report_path,
        'intent_path': intent_path,
        'started_at': started_at,
        'finished_at': (time.time() if finished_at is None else float(finished_at)),
        'findings': list(findings or []),
    }
    path = os.path.join(findings_dir, f'findings_{task_id}.json')
    tmp = path + '.tmp'
    with open(tmp, 'w', encoding='utf-8') as handle:
        json.dump(record, handle, ensure_ascii=False, indent=2)
        handle.write('\n')
    os.replace(tmp, path)

    index = os.path.join(findings_dir, 'index.jsonl')
    with open(index, 'a', encoding='utf-8') as handle:
        handle.write(json.dumps(record, ensure_ascii=False) + '\n')
    return {'path': path, 'index': index, 'task_id': task_id}


def load_records(findings_dir: str = 'logs/findings') -> list:
    """优先读 index.jsonl；缺失时退回 glob findings_*.json。按 task_id 去重。"""
    records = {}
    index = os.path.join(findings_dir, 'index.jsonl')
    if os.path.isfile(index):
        with open(index, encoding='utf-8') as handle:
            for line in handle:
                line = line.strip()
                if not line:
                    continue
                try:
                    record = json.loads(line)
                except json.JSONDecodeError:
                    continue
                records[record.get('task_id', '')] = record
    if not records:
        for path in sorted(glob.glob(os.path.join(findings_dir, 'findings_*.json'))):
            try:
                with open(path, encoding='utf-8') as handle:
                    record = json.load(handle)
            except (OSError, json.JSONDecodeError):
                continue
            records[record.get('task_id', os.path.basename(path))] = record
    return sorted(records.values(),
                  key=lambda item: float(item.get('finished_at') or 0.0),
                  reverse=True)


def summarize_records(records: list) -> dict:
    by_label = {}
    by_status = {}
    findings_total = 0
    tasks_with_findings = 0
    for record in records:
        findings = record.get('findings') or []
        if findings:
            tasks_with_findings += 1
        findings_total += len(findings)
        for finding in findings:
            label = finding.get('label', 'unknown')
            status = finding.get('status', 'unknown')
            by_label[label] = by_label.get(label, 0) + 1
            by_status[status] = by_status.get(status, 0) + 1
    return {
        'tasks': len(records),
        'tasks_with_findings': tasks_with_findings,
        'findings_total': findings_total,
        'by_label': dict(sorted(by_label.items())),
        'by_status': dict(sorted(by_status.items())),
        'records': records,
    }


def render_summary(summary: dict, limit: int = 20) -> str:
    lines = ['# 发现物汇总', '']
    lines.append(f'- 任务数：{summary["tasks"]}（有发现 {summary["tasks_with_findings"]}）')
    lines.append(f'- 发现物总数：{summary["findings_total"]}')
    lines.append(f'- 按类别：{summary["by_label"] or "无"}')
    lines.append(f'- 按状态：{summary["by_status"] or "无"}')
    lines.append('')
    lines.append('| 任务 | 时间 | 类别 | 位置 ENU | 状态 | 置信度 |')
    lines.append('|---|---|---|---|---|---|')
    rows = 0
    for record in summary['records']:
        if rows >= limit:
            break
        finished = time.strftime('%Y-%m-%d %H:%M:%S',
                                 time.localtime(float(record.get('finished_at') or 0)))
        for finding in record.get('findings') or []:
            world = finding.get('world_enu') or [0, 0, 0]
            lines.append(
                f'| {record.get("task_id", "")} | {finished} | '
                f'{finding.get("label", "")} | '
                f'({float(world[0]):.2f}, {float(world[1]):.2f}, {float(world[2]):.2f}) | '
                f'{finding.get("status", "")} | '
                f'{float(finding.get("confidence", 0)):.2f} |')
            rows += 1
    if rows == 0:
        lines.append('| — | — | — | — | — | — |')
    return '\n'.join(lines) + '\n'


def main(argv=None):
    parser = argparse.ArgumentParser(description='汇总 logs/findings 下的发现物')
    parser.add_argument('--dir', default='logs/findings')
    parser.add_argument('--limit', type=int, default=20)
    parser.add_argument('--json', action='store_true', help='输出 JSON 而不是 Markdown')
    parser.add_argument('--md', default='', help='同时写一份 Markdown 到指定路径')
    args = parser.parse_args(argv)
    records = load_records(args.dir)
    summary = summarize_records(records)
    text = render_summary(summary, limit=args.limit)
    if args.md:
        os.makedirs(os.path.dirname(args.md) or '.', exist_ok=True)
        with open(args.md, 'w', encoding='utf-8') as handle:
            handle.write(text)
    if args.json:
        print(json.dumps({key: value for key, value in summary.items()
                          if key != 'records'}, ensure_ascii=False, indent=2))
        print(f'任务数 {summary["tasks"]}，发现物 {summary["findings_total"]}')
    else:
        print(text)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
