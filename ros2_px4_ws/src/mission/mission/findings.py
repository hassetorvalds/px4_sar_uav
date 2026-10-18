"""结构化发现物（findings）模型。

搜索/检测确认的目标不再只是报告里的一条 `target_confirmed`，而是可被上层
（报告、Action result、后续任务）消费的 finding：

    finding_id / label / world_enu / confidence / priority /
    status / source / first_seen / last_seen / updates / step_type

同一目标在 takeoff/search/scan 多次确认时按世界距离合并，位置取增量平均，
减少单帧深度误差；`target_centered` 后状态升级为 `inspected`。
"""

import math
from dataclasses import dataclass, field


_STATUS_ORDER = {'candidate': 0, 'confirmed': 1, 'inspected': 2, 'resolved': 3}


def priority_for_kind(kind: str) -> str:
    """目标类别 → 优先级；未知类别不猜，返回 unknown。"""
    kind = (kind or '').lower()
    if kind in ('person', 'victim', 'human', 'people'):
        return 'high'
    if kind in ('vehicle', 'car', 'truck'):
        return 'medium'
    return 'unknown'


@dataclass
class Finding:
    finding_id: int
    label: str = 'target'
    world_enu: tuple = (0.0, 0.0, 0.0)
    confidence: float = 1.0
    priority: str = 'unknown'
    status: str = 'confirmed'
    source: str = 'perception'
    first_seen: float = 0.0
    last_seen: float = 0.0
    updates: int = 1
    step_type: str = ''
    notes: str = ''

    def to_dict(self):
        return {
            'finding_id': self.finding_id,
            'label': self.label,
            'world_enu': [round(float(v), 2) for v in self.world_enu],
            'confidence': round(float(self.confidence), 3),
            'priority': self.priority,
            'status': self.status,
            'source': self.source,
            'first_seen': round(float(self.first_seen), 2),
            'last_seen': round(float(self.last_seen), 2),
            'updates': int(self.updates),
            'step_type': self.step_type,
            'notes': self.notes,
        }


class FindingsBook:
    def __init__(self, merge_radius_m: float = 3.0):
        self._radius = max(float(merge_radius_m), 0.1)
        self._items = []
        self._next_id = 1

    def __len__(self):
        return len(self._items)

    def clear(self):
        self._items = []
        self._next_id = 1

    def _nearest(self, world_enu):
        best, best_dist = None, None
        for item in self._items:
            distance = math.dist(item.world_enu, world_enu)
            if best_dist is None or distance < best_dist:
                best, best_dist = item, distance
        return best, best_dist

    def add_or_update(self, *, label: str, world_enu, confidence: float = 1.0,
                      priority: str = 'unknown', source: str = 'perception',
                      t: float = 0.0, step_type: str = '',
                      status: str = 'confirmed') -> Finding:
        world = tuple(float(v) for v in world_enu)
        item, distance = self._nearest(world)
        if item is not None and distance <= self._radius:
            count = item.updates
            item.world_enu = tuple(
                (item.world_enu[i] * count + world[i]) / (count + 1)
                for i in range(3))
            item.updates = count + 1
            item.last_seen = max(item.last_seen, float(t))
            item.confidence = max(item.confidence, float(confidence))
            if _STATUS_ORDER.get(status, 0) > _STATUS_ORDER.get(item.status, 0):
                item.status = status
            item.step_type = step_type or item.step_type
            return item
        item = Finding(
            finding_id=self._next_id, label=label, world_enu=world,
            confidence=float(confidence), priority=priority, status=status,
            source=source, first_seen=float(t), last_seen=float(t),
            updates=1, step_type=step_type)
        self._next_id += 1
        self._items.append(item)
        return item

    def mark_inspected(self, world_enu, t: float = 0.0,
                       step_type: str = '', notes: str = '') -> Finding:
        item, distance = self._nearest(tuple(float(v) for v in world_enu))
        if item is None or distance > self._radius:
            return None
        if _STATUS_ORDER.get('inspected', 0) > _STATUS_ORDER.get(item.status, 0):
            item.status = 'inspected'
        item.last_seen = max(item.last_seen, float(t))
        if step_type:
            item.step_type = step_type
        if notes:
            item.notes = notes
        return item

    def to_dicts(self):
        return [item.to_dict() for item in self._items]

    def confirmed(self):
        return [item for item in self._items
                if _STATUS_ORDER.get(item.status, 0) >= 1]
