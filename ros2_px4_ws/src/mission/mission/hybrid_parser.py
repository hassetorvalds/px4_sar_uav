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
    r'(?:飞到|飞向|前往|接近|靠近|去)\s*(?P<target>[^，,。；;]+)',
    re.IGNORECASE)
_RE_VISUAL_WORDS = re.compile(
    r'目标|物体|建筑|橙色|红色|屋顶|锥|车|门|窗|桥|树|人|标志|标记|'
    r'附近|旁边|前面|后面|那里|它|cone|building|roof|vehicle|person|marker',
    re.IGNORECASE)
_RE_TARGET_PROMPT = re.compile(
    r'(?:寻找|查找|找一下|搜寻|find|look\s+for|search\s+for)\s*'
    r'(?P<prompt>[^，,。；;]+)', re.IGNORECASE)


def _target_prompt(clause: str):
    match = _RE_TARGET_PROMPT.search(clause)
    if not match:
        return None
    prompt = match.group('prompt').strip(' ，,。；;')
    prompt = re.sub(r'^(一下|一个|那个)', '', prompt).strip()
    if not prompt or prompt in ('区域', '范围'):
        return None
    return prompt


def parse_hybrid_instruction(text: str, default_altitude: float = 2.0) -> dict:
    """返回 steps/visual_index/altitude/notes；不构造 TaskSpec。"""
    text = str(text).strip()
    if not text:
        raise TaskError('指令为空')
    altitude = float(default_altitude)
    steps = []
    notes = []
    visual_index = None
    visual_target_text = None
    pending_prompt = None
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
            prompt = _target_prompt(clause)
            try:
                parsed_clause = parse_task(clause)
            except TaskError:
                # “寻找橙色球”只有目标、没有区域；作为 prompt 挂到 search。
                if prompt and steps and steps[-1]['type'] == 'search':
                    target = dict(steps[-1]['params'].get('target') or {})
                    target['prompt'] = prompt
                    steps[-1]['params']['target'] = target
                    continue
                if prompt:
                    pending_prompt = prompt
                    continue
                raise
            here = [step for step in parsed_clause.steps
                    if step.type in ('search', 'scan')]
            if not here:
                # “寻找橙色球”这类只给目标、不带区域的子句：挂到上一条 search
                if prompt and steps and steps[-1]['type'] == 'search':
                    target = dict(steps[-1]['params'].get('target') or {})
                    target['prompt'] = prompt
                    steps[-1]['params']['target'] = target
                    continue
                if prompt:
                    pending_prompt = prompt
                    continue
                raise TaskError(f'搜索/扫描子句没有产生动作：{clause!r}')
            for step in here:
                params = dict(step.params)
                params['altitude_m'] = altitude
                if step.type == 'search':
                    target = dict(params.get('target') or {})
                    used_prompt = prompt or pending_prompt
                    if used_prompt:
                        target['prompt'] = used_prompt
                        pending_prompt = None
                    params['target'] = target
                steps.append({'type': step.type, 'params': params})
            continue
        visual_match = _RE_VISUAL.search(clause)
        if visual_match:
            # 有方向+距离、又没有物体/附近语义的，仍按方向 goto 处理。
            if (_RE_DIR_DIST.search(clause)
                    and not _RE_VISUAL_WORDS.search(clause)):
                visual_match = None
        if visual_match:
            if visual_index is not None:
                raise TaskError('只支持一个视觉目标')
            target_text = visual_match.group('target').strip(' ，,。；;')
            target_text = re.sub(r'(附近|旁边|前面|后面|处|那里)$', '', target_text).strip()
            target_text = re.sub(r'^(前方|前面|画面中|一个|的)', '', target_text).strip()
            if not target_text:
                raise TaskError('视觉目标描述为空')
            visual_index = len(steps)
            visual_target_text = target_text
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

    if pending_prompt is not None:
        raise TaskError(f'目标描述“{pending_prompt}”没有挂到搜索区域')
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
            'visual_target_text': visual_target_text,
            'altitude_m': altitude, 'notes': notes, 'source_text': text}
