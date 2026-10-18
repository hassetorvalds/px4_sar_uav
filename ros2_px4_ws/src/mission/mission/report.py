"""把任务执行报告（JSONL）汇总成一份可读的 Markdown 证据页。

输入：`fly_task.py` 写出的 `report_*.jsonl`（可选 `intent_*.json`）。
输出：同目录 `report_*.md`，包含结果、目标、时间线、命令生命周期、产物路径。

纯逻辑，不依赖 ROS。
"""

import argparse
import json
import os
import time


def _load_events(path):
    events = []
    with open(path, encoding='utf-8') as handle:
        for line in handle:
            line = line.strip()
            if not line:
                continue
            try:
                events.append(json.loads(line))
            except json.JSONDecodeError:
                continue
    return events


def _load_intent(path):
    if not path or not os.path.isfile(path):
        return None
    try:
        with open(path, encoding='utf-8') as handle:
            return json.load(handle)
    except (OSError, json.JSONDecodeError):
        return None


def _fmt(value, digits=2):
    if value is None:
        return '—'
    if isinstance(value, float):
        return f'{value:.{digits}f}'
    return str(value)


def _world(value):
    if not value:
        return '—'
    return '(' + ', '.join(f'{float(v):.2f}' for v in value) + ')'


def summarize(report_path: str, intent_path: str = '', output_path: str = '') -> str:
    events = _load_events(report_path)
    intent = _load_intent(intent_path)
    accepted = next((e for e in events if e.get('event') == 'task_accepted'), {})
    spec = accepted.get('spec') or {}
    summary = accepted.get('summary') or (intent or {}).get('text') or '（未记录任务摘要）'
    done = next((e for e in events if e.get('event') == 'done'), None)
    aborts = [e for e in events if e.get('event') == 'abort']
    targets = [e for e in events if e.get('event') == 'target_confirmed']
    centered = [e for e in events if e.get('event') == 'target_centered']
    health = next((e for e in reversed(events) if e.get('event') == 'health_check'), None)
    takeoff = next((e for e in events if e.get('event') == 'takeoff_complete'), None)
    commands = done.get('commands') if done else []
    findings = (done or {}).get('findings') or []
    if not findings:
        by_id = {}
        for event in events:
            if event.get('event') == 'finding' and event.get('finding_id') is not None:
                by_id[event['finding_id']] = event
        findings = list(by_id.values())
    scan_complete = any(e.get('event') == 'scan_complete' for e in events)
    waypoints = [e for e in events if e.get('event') == 'search_waypoint']
    plan = next((e for e in events if e.get('event') == 'search_plan'), None)
    mtime = time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(os.path.getmtime(report_path)))

    lines = [f'# 任务报告：{summary}', '']
    lines.append(f'- 生成时间：{mtime}')
    lines.append(f'- 报告文件：`{report_path}`')
    if intent_path:
        lines.append(f'- 意图文件：`{intent_path}`')
    if intent:
        lines.append(f'- 意图来源：{intent.get("source", "?")}，'
                     f'置信度 {float(intent.get("confidence", 0)):.2f}')
    if spec:
        lines.append(f'- 任务高度：{_fmt(spec.get("altitude_m"))} m '
                     f'{spec.get("altitude_frame", "REL")}；'
                     f'速度 ≤{_fmt(spec.get("max_speed_mps"))} m/s；'
                     f'围栏 {_fmt(spec.get("geofence_radius_m"))} m')
    lines.append('')
    lines.append('## 结果')
    lines.append('')
    lines.append(f'- **{"成功完成" if done and not aborts else "未完成"}**'
                 f'（{"done" if done else "无 done"}，abort {len(aborts)} 次）')
    if aborts:
        for abort in aborts:
            lines.append(f'- 中止：{abort.get("reason", "?")}（t={abort.get("t")} s）')
    if health:
        lines.append(f'- 起飞前检查：{"通过" if health.get("ok") else "未通过"}')
        for check in health.get('checks', []):
            mark = '✓' if check.get('ok') else '✗'
            lines.append(f'  - {mark} {check.get("name")}: {check.get("detail")}')
    if takeoff:
        altitude = takeoff.get('altitude_lcl') or {}
        target = takeoff.get('target') or {}
        lines.append(f'- 起飞：LCL {_fmt(altitude.get("value_m"))} m / '
                     f'任务 {_fmt(target.get("value_m"))} m {target.get("frame", "")}')
    if plan:
        lines.append(f'- 搜索：{plan.get("waypoints", "?")} 航点，'
                     f'到位 {len(waypoints)} 个')
    if scan_complete:
        lines.append('- 扫描：完成')
    if targets:
        lines.append(f'- 目标：{len(targets)} 次确认')
        for target in targets:
            lines.append(f'  - t={target.get("t")} s，world={_world(target.get("world_enu"))}，'
                         f'阶段 {target.get("step_type", "?")}')
    if centered:
        for item in centered:
            lines.append(f'  - 已对准：方位误差 {_fmt(item.get("error_deg"), 1)}°'
                         f'（t={item.get("t")} s）')
    lines.append('')
    lines.append('## 发现物')
    lines.append('')
    if findings:
        lines.append('| ID | 类型 | 位置 ENU | 置信度 | 优先级 | 状态 | 首次/最后 (s) |')
        lines.append('|---|---|---|---|---|---|---|')
        for finding in findings:
            lines.append(
                f'| {finding.get("finding_id")} | {finding.get("label")} | '
                f'{_world(finding.get("world_enu"))} | '
                f'{_fmt(finding.get("confidence"), 2)} | '
                f'{finding.get("priority")} | {finding.get("status")} | '
                f'{_fmt(finding.get("first_seen"), 1)} / '
                f'{_fmt(finding.get("last_seen"), 1)} |')
    else:
        lines.append('未记录发现物。')
    if commands:
        lines.append('')
        lines.append('## 命令生命周期')
        lines.append('')
        lines.append('| 命令 | 状态 | 尝试 | ACK 结果 | 延迟 s |')
        lines.append('|---|---|---|---|---|')
        for command in commands:
            lines.append(f'| {command.get("name")} | {command.get("state")} | '
                         f'{command.get("attempts")} | {command.get("result_code")} | '
                         f'{_fmt(command.get("latency_s"), 3)} |')

    lines.append('')
    lines.append('## 时间线')
    lines.append('')
    lines.append('| t (s) | 事件 | 详情 |')
    lines.append('|---|---|---|')
    for event in events:
        name = event.get('event', '?')
        detail = ''
        if name.startswith('command'):
            detail = f'{event.get("name")} → {event.get("state")}'
            if event.get('result_code') is not None:
                detail += f'（result={event.get("result_code")}）'
        elif name == 'health_check':
            detail = 'ok' if event.get('ok') else '；'.join(event.get('reasons') or [])
        elif name == 'target_confirmed':
            detail = f'world={_world(event.get("world_enu"))}'
        elif name == 'target_centered':
            detail = f'error={_fmt(event.get("error_deg"), 1)}°'
        elif name == 'takeoff_complete':
            detail = f'LCL={_fmt((event.get("altitude_lcl") or {}).get("value_m"))} m'
        elif name == 'search_waypoint':
            detail = f'{event.get("index")}/{event.get("total")}'
        elif name == 'scan_dwell':
            detail = f'heading={event.get("heading_enu_deg")}°'
        elif name == 'abort':
            detail = str(event.get('reason', ''))
        lines.append(f'| {event.get("t")} | {name} | {detail} |')

    if not output_path:
        output_path = os.path.splitext(report_path)[0] + '.md'
    with open(output_path, 'w', encoding='utf-8') as handle:
        handle.write('\n'.join(lines) + '\n')
    return output_path


def main(argv=None):
    parser = argparse.ArgumentParser(description='把任务 JSONL 报告汇总成 Markdown')
    parser.add_argument('report', help='report_*.jsonl')
    parser.add_argument('--intent', default='', help='intent_*.json（可选）')
    parser.add_argument('--out', default='', help='输出 .md（默认同目录同名）')
    args = parser.parse_args(argv)
    output = summarize(args.report, args.intent, args.out)
    print(output)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
