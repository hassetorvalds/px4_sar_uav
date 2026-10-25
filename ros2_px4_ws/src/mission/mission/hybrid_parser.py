"""文本动作 + 视觉目标槽的混合解析（Hybrid NL）。

暂不交给 VLM 产 TaskSpec：规则解析动作，视觉目标只留一个 ``visual_goto`` 槽，
由 Qwen grounding 填成 goto。视觉目标必须紧接起飞，避免前面有位移时坐标错位。
"""

import re

from .task_parser import (_DIRECTIONS, _RE_DIR_DIST, _RE_FORWARD, _RE_HOVER,
                          _RE_LAND, _RE_SCAN, _RE_SEARCH, _RE_TAKEOFF,
                          _RE_TAKEOFF_ALT, _clauses, _num, parse_task)
from .task_spec import TaskError

_RE_VISUAL = re.compile(
    r'(?:飞到|飞向|前往|接近|靠近|去)\s*.*?(?:目标|物体|建筑|橙色|红色|画面|前方|那里|它)',
    re.IGNORECASE)


def parse_hybrid_instruction(text: str, default_altitude: float = 2.0) -> dict:
    """返回 steps/visual_index/altitude/notes；不构造 TaskSpec。"""
    text = str(text).strip()
    if not text:
        raise TaskError('指令为空')
    altitude = float(default_altitude)
    steps = []
    notes = []
    visual_index = None
    for clause in _clauses(text):
        takeoff = _RE_TAKEOFF_ALT.search(clause)
        if takeoff:
            altitude = _num(takeoff.group('num'))
            steps.append({'type': 'takeoff',
                          'params': {'altitude_m': altitude}})
            continue
        if _RE_LAND.search(clause):
            steps.append({'type': 'land', 'params': {}})
            continue
        hover = _RE_HOVER.search(clause)
        if hover and ('悬停' in clause or '等待' in clause or 'hover' in clause.lower()):
            seconds = _num(hover.group('num')) if hover.group('num') else 5.0
            steps.append({'type': 'hover', 'params': {'seconds': seconds}})
            continue
        if _RE_SEARCH.search(clause) or _RE_SCAN.search(clause):
            # 复用现有搜索几何/扫描参数，不复制 lawnmower 逻辑。
            try:
                parsed_clause = parse_task(clause)
            except TaskError as exc:
                raise TaskError(f'搜索/扫描子句解析失败：{clause!r}：{exc}') from exc
            here = [step for step in parsed_clause.steps
                    if step.type in ('search', 'scan')]
            if not here:
                raise TaskError(f'搜索/扫描子句没有产生动作：{clause!r}')
            for step in here:
                params = dict(step.params)
                params['altitude_m'] = altitude
                steps.append({'type': step.type, 'params': params})
            continue
        if _RE_VISUAL.search(clause):
            if visual_index is not None:
                raise TaskError('只支持一个视觉目标')
            visual_index = len(steps)
            steps.append({'type': 'visual_goto', 'params': {}})
            continue
        forward = _RE_FORWARD.search(clause)
        if forward:
            steps.append({'type': 'forward',
                          'params': {'meters': _num(forward.group('num'))}})
            continue
        east = north = 0.0
        matched = False
        for match in _RE_DIR_DIST.finditer(clause):
            axis, sign = _DIRECTIONS[match.group('dir').lower()]
            distance = _num(match.group('num'))
            if axis == 'east':
                east += sign * distance
            else:
                north += sign * distance
            matched = True
        if matched:
            steps.append({'type': 'goto', 'params': {
                'east_m': round(east, 2), 'north_m': round(north, 2),
                'altitude_m': altitude}})
            continue
        if _RE_TAKEOFF.search(clause):
            steps.append({'type': 'takeoff', 'params': {'altitude_m': altitude}})
            continue
        raise TaskError(
            f'未识别的子句：{clause!r}；支持：起飞/飞到前方目标/方向移动/悬停/降落')

    if not steps:
        raise TaskError('没有识别出可执行动作')
    if steps[0]['type'] != 'takeoff':
        steps.insert(0, {'type': 'takeoff',
                         'params': {'altitude_m': altitude}})
        notes.append('未显式说起飞：已自动补起飞')
        if visual_index is not None:
            visual_index += 1
    if steps[-1]['type'] != 'land':
        steps.append({'type': 'land', 'params': {}})
        notes.append('未显式说降落：已自动补降落')
    # 视觉目标可以在搜索/扫描后；Qwen 目标在起飞前从原点拍摄，
    # 因此 TaskSpec 仍按相对起飞原点解释，不做“移动后重拍”。
    return {'steps': steps, 'visual_index': visual_index,
            'altitude_m': altitude, 'notes': notes, 'source_text': text}
