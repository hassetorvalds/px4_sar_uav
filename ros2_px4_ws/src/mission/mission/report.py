"""把任务执行报告（JSONL）汇总成一份可读的 Markdown 证据页。

输入：`fly_task.py` 写出的 `report_*.jsonl`（可选 `intent_*.json`）。
输出：同目录 `report_*.md`，包含结果、目标、时间线、命令生命周期、产物路径。

纯逻辑，不依赖 ROS。
"""

import argparse
import json
import math
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

def _note(notes, key):
    for note in notes or []:
        if isinstance(note, str) and note.startswith(key + '='):
            return note.split('=', 1)[1]
    return None


def _trajectory_section(findings):
    """把 finding.history 写成可读轨迹；无 history 时返回空。"""
    tracked = [item for item in findings if item.get('history')]
    if not tracked:
        return []
    lines = ['', '## 目标轨迹', '']
    for finding in tracked:
        lines.append(f"### Finding #{finding.get('finding_id')} "
                     f"({finding.get('label')}, {finding.get('status')})")
        lines.append('')
        lines.append(f"- 位置：{_world(finding.get('world_enu'))}；"
                     f"updates={finding.get('updates', 1)}；"
                     f"first/last={_fmt(finding.get('first_seen'), 1)}/"
                     f"{_fmt(finding.get('last_seen'), 1)} s")
        if finding.get('revisit_of'):
            lines.append(f"- revisit_of=#{finding.get('revisit_of')}")
        history = finding.get('history') or []
        lines.append('')
        lines.append('| t (s) | 状态 | 位置 ENU | 重访 |')
        lines.append('|---|---|---|---|')
        for entry in history[-30:]:
            lines.append(f"| {_fmt(entry.get('t'), 1)} | "
                         f"{entry.get('status', '?')} | "
                         f"{_world(entry.get('world_enu'))} | "
                         f"{'是' if entry.get('revisit') else ''} |")
        lines.append('')
    return lines

def _visual_target_section(events, spec, done, aborts):
    """Qwen/动态目标的可解释结果；没有视觉目标时返回空。"""
    notes = spec.get('notes') or []
    dynamic = [event for event in events if event.get('event') == 'dynamic_target']
    steps = spec.get('steps') or []
    goto = next((step.get('params') or {} for step in steps
                 if step.get('type') == 'goto'), {})
    dynamic_requested = bool(goto.get('dynamic_target'))
    abort_reason = aborts[-1].get('reason') if aborts else None
    if (not dynamic and not dynamic_requested
            and not str(abort_reason or '').startswith('dynamic_target')
            and not any(str(note).startswith('qwen_') for note in notes)):
        return []
    lines = ['', '## 视觉目标', '']
    source = spec.get('intent_source') or (
        'qwen_dynamic' if (dynamic or dynamic_requested) else 'qwen_grounding')
    lines.append(f'- 来源：{source}')
    if str(source).startswith('qwen'):
        lines.append('- VLM：Qwen3-VL-8B，只做图像 grounding，不直接输出飞控速度')
    lines.append('- 控制方式：VLM 目标 → ENU 目标 → TaskSpec goto → PX4 offboard')
    if goto:
        lines.append(f'- 初始 goto：东 {_fmt(goto.get("east_m"))} m，'
                     f'北 {_fmt(goto.get("north_m"))} m')
    for key, label in (('qwen_samples', 'Qwen 采样'), ('qwen_spread_m', '目标 spread'),
                       ('qwen_latency_ms', 'Qwen 延迟'), ('pixel', '目标像素'),
                       ('depth_m', '目标深度'), ('proxy_error_norm', 'bbox 偏差')):
        value = _note(notes, key)
        if value is not None:
            suffix = {'qwen_spread_m': ' m', 'qwen_latency_ms': ' ms',
                      'depth_m': ' m'}.get(key, '')
            lines.append(f'- {label}：{value}{suffix}')

    target = None
    if dynamic:
        last = dynamic[-1]
        target = (float(last.get('east_m')), float(last.get('north_m')))
        ages = [event.get('age_s') for event in dynamic if event.get('age_s') is not None]
        lines.append(f'- 动态更新：{len(dynamic)} 次；最后目标 {_world(target)}'
                     + (f'，age {_fmt(min(ages), 2)} s' if ages else ''))
    else:
        median = _note(notes, 'qwen_median_enu')
        if median:
            try:
                east, north = (float(item) for item in median.split(','))
                target = (east, north)
            except (TypeError, ValueError):
                target = None
        if target:
            lines.append(f'- Qwen 目标：{_world(target)}')
        elif (not dynamic_requested and goto
              and goto.get('east_m') is not None and goto.get('north_m') is not None):
            east = float(goto.get('east_m'))
            north = float(goto.get('north_m'))
            takeoff = next((event for event in events
                            if event.get('event') == 'takeoff_complete'), None)
            origin = (takeoff or {}).get('position_enu') or [0.0, 0.0]
            if spec.get('coordinate_frame') == 'relative':
                east += float(origin[0])
                north += float(origin[1])
            target = (east, north)
            lines.append(f'- Qwen 目标（初始 goto）：{_world(target)}')

    if dynamic_requested or dynamic:
        if abort_reason == 'dynamic_target_unavailable':
            status = '未获得动态目标'
        elif abort_reason == 'dynamic_target_stale':
            status = '动态目标已过期'
        elif abort_reason == 'dynamic_target_out_of_geofence':
            status = '动态目标出围栏'
        elif dynamic:
            status = '已获得并持续更新'
        else:
            status = '等待动态目标'
        lines.append(f'- 目标状态：{status}')

    if aborts:
        lines.append(f'- 失败分类：{abort_reason or "?"}')
    if target and not aborts and done and done.get('position_enu'):
        position = done['position_enu']
        error = math.hypot(float(position[0]) - target[0],
                           float(position[1]) - target[1])
        lines.append(f'- 最终位置：{_world(position)}')
        lines.append(f'- 到点误差（水平）：{error:.2f} m')
    return lines


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
    coverage = (done or {}).get('coverage')
    if not coverage:
        coverage = next((e for e in reversed(events)
                         if e.get('event') == 'search_coverage'), None)
    waypoints = [e for e in events if e.get('event') == 'search_waypoint']
    plans = [e for e in events if e.get('event') == 'search_plan']
    coverages = {e.get('step'): e for e in events
                 if e.get('event') == 'search_coverage'}
    transits = [e for e in events
                if e.get('event') in ('transit', 'transit_climb')]
    plan = plans[0] if plans else None
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
    if plans and len(plans) == 1:
        plan = plans[0]
        detail = f'- 搜索：{plan.get("waypoints", "?")} 航点，到位 {len(waypoints)} 个'
        if plan.get('area_m2') is not None:
            detail += (f'；面积 {_fmt(plan.get("area_m2"), 1)} m²'
                       f'，折线 {_fmt(plan.get("path_length_m"), 1)} m'
                       f'，预计 {_fmt(plan.get("estimated_s"), 0)} s')
            if plan.get('lane_overlap_ratio') is not None:
                detail += (f'，下视足印 {_fmt(plan.get("footprint_width_m"), 1)} m'
                           f'、重叠 {100.0 * float(plan.get("lane_overlap_ratio")):.0f}%')
        lines.append(detail)
    elif len(plans) > 1:
        total_area = sum(float(item.get('area_m2') or 0.0) for item in plans)
        lines.append(f'- 搜索：{len(plans)} 个区域，合计 {_fmt(total_area, 1)} m²')
    if scan_complete:
        lines.append('- 扫描：完成')
    if len(plans) > 1:
        region_coverages = [coverages.get(item.get('step')) for item in plans]
        region_coverages = [item for item in region_coverages if item]
        if region_coverages:
            observed = sum(int(item.get('observed_cells') or 0)
                          for item in region_coverages)
            total = sum(int(item.get('total_cells') or 0)
                        for item in region_coverages)
            missed = sum(int(item.get('missed_cells') or 0)
                         for item in region_coverages)
            ratio = (observed / total) if total else 0.0
            lines.append(
                f'- 观测覆盖（合计）：{observed}/{total} 格'
                f'（{100.0 * ratio:.1f}%），漏扫 {missed} 格')
    elif coverage:
        lines.append(
            f'- 观测覆盖：{coverage.get("observed_cells")}/'
            f'{coverage.get("total_cells")} 格'
            f'（{100.0 * float(coverage.get("coverage_ratio", 0.0)):.1f}%），'
            f'漏扫 {coverage.get("missed_cells")} 格，'
            f'已观测 {_fmt(coverage.get("observed_area_m2"), 1)} m²'
            f'（地面点 {coverage.get("ground_points", "?")}，'
            f'区域内 {coverage.get("points_in_polygon", "?")}）')
        if coverage.get('pose_age_s') is not None:
            lines.append(
                f'- 位姿新鲜度：{_fmt(coverage.get("pose_age_s"), 2)} s，'
                f'eph={_fmt(coverage.get("eph_m"), 2)} m、'
                f'epv={_fmt(coverage.get("epv_m"), 2)} m，'
                f'heading={_fmt(coverage.get("heading_deg"), 1)}°')
        if coverage.get('heatmap_path'):
            lines.append(f'- 覆盖热力图：`{coverage.get("heatmap_path")}`')
    if transits:
        total_transit = sum(float(item.get('distance_m') or 0.0)
                            for item in transits)
        lines.append(f'- 转场：{len(transits)} 段，合计 {_fmt(total_transit, 1)} m')
    if targets:
        lines.append(f'- 目标：{len(targets)} 次确认')
        demo_kinds = {item.get('target_demo_kind') for item in targets
                      if item.get('target_demo_kind')}
        if 'proxy' in demo_kinds:
            lines.append('- 目标类型：**代理目标（非人员检测）**')
        elif demo_kinds:
            lines.append(f'- 目标类型：{" / ".join(sorted(demo_kinds))}')
        for target in targets:
            lines.append(f'  - t={target.get("t")} s，world={_world(target.get("world_enu"))}，'
                         f'阶段 {target.get("step_type", "?")}')
    if centered:
        for item in centered:
            lines.append(f'  - 已对准：方位误差 {_fmt(item.get("error_deg"), 1)}°'
                         f'（t={item.get("t")} s）')
    if done and done.get('findings_path'):
        lines.append(f'- 发现物记录：`{done.get("findings_path")}`')
    lines.extend(_visual_target_section(events, spec, done, aborts))
    if len(plans) > 1:
        lines.append('')
        lines.append('## 搜索区域')
        lines.append('')
        lines.append('| # | 面积 m² | 航点 | 覆盖 | 漏扫 |')
        lines.append('|---|---|---|---|---|')
        for index, item in enumerate(plans, start=1):
            cov = coverages.get(item.get('step'))
            if cov:
                ratio = 100.0 * float(cov.get('coverage_ratio', 0.0))
                cover = (f'{cov.get("observed_cells")}/{cov.get("total_cells")}'
                         f'（{ratio:.1f}%）')
                missed = cov.get('missed_cells', '?')
            else:
                cover, missed = '—', '—'
            lines.append(f'| {index} | {_fmt(item.get("area_m2"), 1)} | '
                         f'{item.get("waypoints", "?")} | {cover} | {missed} |')
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
    lines.extend(_trajectory_section(findings))
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
        elif name == 'transit':
            detail = f'{event.get("distance_m", "?")} m'
        elif name == 'transit_climb':
            detail = (f'{event.get("distance_m", "?")} m → '
                      f'{event.get("climb_to_m", "?")} m')
        elif name == 'transit_move':
            detail = '到达转场高度，开始巡航'
        elif name == 'transit_descend':
            detail = f'下降到 {event.get("to_m", "?")} m'
        elif name == 'transit_complete':
            detail = '转场完成'
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
