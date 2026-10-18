"""意图帧（IntentFrame）：把"一句话"变成**可审计、可确认、可拒绝**的任务草案。

参考 Simurgh/MDS 的分层：

    语义解释（规则 / VLM） → IntentFrame（typed facts + provenance + confidence + missing）
      → 确定性校验（TaskSpec） → 人工确认 → 执行

关键不变量：**VLM 只能补全本地没解析出来的事实，不能改写本地已解析的事实**
（例如本地解析出"向北 10 米"，VLM 不能把它改成 100 米）。

本模块不依赖 ROS，可离线单测。
"""

import re
import time
from dataclasses import dataclass, field

from .task_parser import parse_task
from .task_spec import TaskSpec

LOCAL_SOURCE = 'local'
VLM_SOURCE = 'vlm'


@dataclass
class Fact:
    key: str
    value: object
    source: str = LOCAL_SOURCE
    span: str = ''

    def to_dict(self):
        return {'key': self.key, 'value': self.value,
                'source': self.source, 'span': self.span}


@dataclass
class IntentFrame:
    text: str
    task: TaskSpec
    source: str = 'rules'           # rules | vlm | merged | manual
    confidence: float = 1.0
    missing: list = field(default_factory=list)
    warnings: list = field(default_factory=list)
    facts: list = field(default_factory=list)
    created_at: float = field(default_factory=time.time)

    def fact_map(self):
        return {fact.key: fact for fact in self.facts}

    def to_dict(self):
        return {
            'text': self.text,
            'source': self.source,
            'confidence': round(float(self.confidence), 3),
            'missing': list(self.missing),
            'warnings': list(self.warnings),
            'facts': [fact.to_dict() for fact in self.facts],
            'created_at': round(float(self.created_at), 3),
            'task': self.task.to_dict(),
        }

    @classmethod
    def from_dict(cls, data):
        return cls(
            text=str(data.get('text', '')),
            task=TaskSpec.from_dict(data['task']),
            source=str(data.get('source', 'rules')),
            confidence=float(data.get('confidence', 1.0)),
            missing=list(data.get('missing') or []),
            warnings=list(data.get('warnings') or []),
            facts=[Fact(key=f['key'], value=f.get('value'), source=f.get('source', LOCAL_SOURCE),
                        span=f.get('span', '')) for f in data.get('facts') or []],
            created_at=float(data.get('created_at', time.time())),
        )

    def summary(self):
        parts = [f'来源 {self.source}，置信度 {self.confidence:.2f}']
        if self.missing:
            parts.append('缺失/默认：' + '、'.join(self.missing))
        if self.warnings:
            parts.append('提示：' + '；'.join(self.warnings))
        return f'{self.task.summary()}（{"；".join(parts)}）'


def _facts_from_task(task: TaskSpec):
    facts = [
        Fact('altitude_m', float(task.altitude_m)),
        Fact('max_duration_s', float(task.max_duration_s)),
        Fact('geofence_radius_m', float(task.geofence_radius_m)),
    ]
    for index, step in enumerate(task.steps):
        prefix = f'{step.type}[{index}]'
        params = step.params
        if step.type == 'takeoff':
            facts.append(Fact(f'{prefix}.altitude_m', float(params.get('altitude_m', task.altitude_m))))
        elif step.type == 'goto':
            facts.append(Fact(f'{prefix}.east_m', float(params.get('east_m', 0.0))))
            facts.append(Fact(f'{prefix}.north_m', float(params.get('north_m', 0.0))))
        elif step.type == 'forward':
            facts.append(Fact(f'{prefix}.meters', float(params.get('meters', 0.0))))
        elif step.type == 'search':
            facts.append(Fact(f'{prefix}.center_east_m', float(params.get('center_east_m', 0.0))))
            facts.append(Fact(f'{prefix}.center_north_m', float(params.get('center_north_m', 0.0))))
            facts.append(Fact(f'{prefix}.radius_m', float(params.get('radius_m', 0.0))))
            facts.append(Fact(f'{prefix}.spacing_m', float(params.get('spacing_m', 0.0))))
            target = params.get('target') or {}
            facts.append(Fact(f'{prefix}.target.kind', target.get('kind', 'any')))
        elif step.type == 'hover':
            facts.append(Fact(f'{prefix}.seconds', float(params.get('seconds', 0.0))))
    return facts


def _infer_missing(text: str, task: TaskSpec):
    missing = []
    notes = ' '.join(task.notes)
    if '自动补"起飞' in notes:
        missing.append('takeoff（自动补）')
    if '自动补"降落' in notes:
        missing.append('land（自动补）')
    has_altitude = bool(re.search(r'高度|altitude|起飞到|升到|爬到|take\s*-?\s*off\s*to',
                                  text, re.IGNORECASE))
    if not has_altitude:
        missing.append('altitude（默认值）')
    for step in task.steps:
        if step.type == 'search':
            target = step.params.get('target') or {}
            if target.get('kind') == 'any':
                missing.append('target（未指定目标类别）')
    # 去重且保持顺序
    return list(dict.fromkeys(missing))


def _confidence(missing, task: TaskSpec):
    score = 1.0
    score -= 0.10 * len(missing)
    score -= 0.05 * max(len(task.notes) - len(missing), 0)
    return max(0.40, min(1.0, round(score, 3)))


def parse_intent(text: str, source: str = 'rules') -> IntentFrame:
    """规则解析 → IntentFrame（本地事实全部标 ``source=local``）。"""
    task = parse_task(text)
    missing = _infer_missing(text, task)
    warnings = list(task.notes)
    return IntentFrame(
        text=text,
        task=task,
        source=source,
        confidence=_confidence(missing, task),
        missing=missing,
        warnings=warnings,
        facts=_facts_from_task(task),
    )


def _same_value(a, b, tol: float = 1e-6):
    if isinstance(a, (int, float)) and isinstance(b, (int, float)):
        return abs(float(a) - float(b)) <= tol
    return a == b


def merge_intent(local: IntentFrame, remote: IntentFrame) -> IntentFrame:
    """把 VLM 的解析结果并入本地解析结果，**本地事实优先**。

    - 本地有、远端不同：保留本地，记 warning（明确写出被拒绝的键与两个值）；
    - 本地是 ``any``/缺失、远端有：接受远端（补全）；
    - 远端没有的：保留本地。
    """
    if remote is None:
        return local
    local_facts = local.fact_map()
    remote_facts = remote.fact_map()
    merged_facts = []
    accepted = []
    warnings = list(local.warnings)

    for key, fact in local_facts.items():
        remote_fact = remote_facts.get(key)
        if remote_fact is None:
            merged_facts.append(fact)
            continue
        if _same_value(fact.value, remote_fact.value):
            merged_facts.append(fact)
            continue
        # 本地未指定（any/空）时允许远端补全
        if fact.value in ('any', None, '') and remote_fact.value not in ('any', None, ''):
            merged_facts.append(Fact(key, remote_fact.value, VLM_SOURCE))
            accepted.append(key)
            continue
        warnings.append(
            f'VLM 试图改写 {key}：本地 {fact.value!r} → 远端 {remote_fact.value!r}，已拒绝')
        merged_facts.append(fact)

    for key, fact in remote_facts.items():
        if key not in local_facts:
            merged_facts.append(Fact(key, fact.value, VLM_SOURCE))
            accepted.append(key)

    task = TaskSpec.from_dict(local.task.to_dict())
    # 只接受"目标类别"的补全，回写到对应 search step；其余新事实仅记录、不改任务
    for fact in merged_facts:
        if fact.source != VLM_SOURCE or not fact.key.endswith('.target.kind'):
            continue
        index = int(fact.key.split('[')[1].split(']')[0])
        if 0 <= index < len(task.steps) and task.steps[index].type == 'search':
            target = dict(task.steps[index].params.get('target') or {})
            target['kind'] = fact.value
            task.steps[index].params['target'] = target

    merged = IntentFrame(
        text=local.text,
        task=task,
        source='merged',
        confidence=min(local.confidence, remote.confidence),
        missing=_infer_missing(local.text, task),
        warnings=warnings,
        facts=merged_facts,
        created_at=time.time(),
    )
    if accepted:
        merged.warnings.append('已接受远端补全：' + '、'.join(sorted(set(accepted))))
    return merged
