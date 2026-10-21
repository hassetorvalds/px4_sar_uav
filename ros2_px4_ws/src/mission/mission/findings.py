"""结构化发现物（Finding 1.0）。

字段：schema_version/task_id/frame_id/frame_seq/frame_size/epoch_id/
world_enu/confidence/priority/status/source/first_seen(_ros)/last_seen(_ros)/
pose_at_first_seen/revisit_of/discarded/history/step_type/notes。

合并：label + 半径（可按 label 配）+ 30 s 时间窗；窗口外同位置新建并记
`revisit_of`。状态只升级；`discarded` 是独立终态标记，不改状态。
"""

import math
from dataclasses import dataclass, field

SCHEMA_VERSION = '1.0'
_STATUS_ORDER = {'candidate': 0, 'confirmed': 1, 'inspected': 2, 'resolved': 3}


def priority_for_kind(kind: str) -> str:
    kind = (kind or '').lower()
    if kind in ('person', 'victim', 'human', 'people'):
        return 'high'
    if kind in ('vehicle', 'car', 'truck'):
        return 'medium'
    return 'unknown'


@dataclass
class Finding:
    finding_id: int
    schema_version: str = SCHEMA_VERSION
    task_id: str = ''
    epoch_id: str = ''
    label: str = 'target'
    world_enu: tuple = (0.0, 0.0, 0.0)
    frame_id: str = 'map'
    frame_seq: int = 0
    frame_size: tuple = (0, 0)
    confidence: float = 1.0
    priority: str = 'unknown'
    status: str = 'confirmed'
    entry_status: str = 'confirmed'
    detector_hits: int = 0
    source: str = 'perception'
    first_seen: float = 0.0
    last_seen: float = 0.0
    first_seen_ros_ns: int = 0
    last_seen_ros_ns: int = 0
    clock_source: str = 'ros'
    pose_at_first_seen: dict = field(default_factory=dict)
    revisit_of: int = 0
    discarded: bool = False
    discard_reason: str = ''
    updates: int = 1
    step_type: str = ''
    notes: str = ''
    history: list = field(default_factory=list)

    def to_dict(self):
        return {
            'schema_version': self.schema_version,
            'task_id': self.task_id,
            'epoch_id': self.epoch_id,
            'finding_id': self.finding_id,
            'label': self.label,
            'world_enu': [round(float(v), 2) for v in self.world_enu],
            'frame_id': self.frame_id,
            'frame_seq': int(self.frame_seq),
            'frame_size': [int(self.frame_size[0]), int(self.frame_size[1])],
            'confidence': round(float(self.confidence), 3),
            'priority': self.priority,
            'status': self.status,
            'entry_status': self.entry_status,
            'detector_hits': int(self.detector_hits),
            'source': self.source,
            'first_seen': round(float(self.first_seen), 2),
            'last_seen': round(float(self.last_seen), 2),
            'first_seen_ros_ns': int(self.first_seen_ros_ns),
            'last_seen_ros_ns': int(self.last_seen_ros_ns),
            'clock_source': self.clock_source,
            'pose_at_first_seen': self.pose_at_first_seen,
            'revisit_of': int(self.revisit_of),
            'discarded': bool(self.discarded),
            'discard_reason': self.discard_reason,
            'updates': int(self.updates),
            'step_type': self.step_type,
            'notes': self.notes,
            'history': self.history[-50:],
        }


class FindingsBook:
    def __init__(self, merge_radius_m: float = 3.0,
                 merge_window_s: float = 30.0, radius_by_label=None,
                 task_id: str = '', epoch_id: str = ''):
        self._radius = max(float(merge_radius_m), 0.1)
        self._window = max(float(merge_window_s), 0.0)
        self._radius_by_label = {str(k): max(float(v), 0.1)
                                 for k, v in (radius_by_label or {}).items()}
        self._task_id = task_id
        self._epoch_id = epoch_id
        self._items = []
        self._next_id = 1

    def __len__(self):
        return len(self._items)

    def clear(self):
        self._items = []
        self._next_id = 1

    def _radius_for(self, label):
        return self._radius_by_label.get(label, self._radius)

    def _nearest(self, world_enu, label=None):
        best, best_dist = None, None
        for item in self._items:
            if label is not None and item.label != label:
                continue
            distance = math.dist(item.world_enu, world_enu)
            if best_dist is None or distance < best_dist:
                best, best_dist = item, distance
        return best, best_dist

    def add_or_update(self, *, label: str, world_enu, confidence: float = 1.0,
                      priority: str = 'unknown', source: str = 'perception',
                      t: float = 0.0, step_type: str = '',
                      status: str = 'confirmed', frame_seq: int = 0,
                      frame_size=(0, 0), pose_at_first_seen=None,
                      ros_stamp_ns: int = 0, clock_source: str = 'ros',
                      entry_status: str = 'confirmed', detector_hits: int = 0,
                      discarded: bool = False,
                      discard_reason: str = '') -> Finding:
        world = tuple(float(v) for v in world_enu)
        item, distance = self._nearest(world, label=label)
        same_place = item is not None and distance <= self._radius_for(label)
        in_window = (same_place
                     and float(t) - item.last_seen <= self._window)
        if same_place and in_window:
            count = item.updates
            item.world_enu = tuple(
                (item.world_enu[i] * count + world[i]) / (count + 1)
                for i in range(3))
            item.updates = count + 1
            item.last_seen = max(item.last_seen, float(t))
            item.last_seen_ros_ns = max(item.last_seen_ros_ns, int(ros_stamp_ns))
            item.confidence = max(item.confidence, float(confidence))
            if _STATUS_ORDER.get(status, 0) > _STATUS_ORDER.get(item.status, 0):
                item.status = status
            item.frame_seq = int(frame_seq) or item.frame_seq
            if tuple(frame_size) != (0, 0):
                item.frame_size = tuple(frame_size)
            item.step_type = step_type or item.step_type
            item.detector_hits = max(item.detector_hits, int(detector_hits))
            item.discarded = bool(item.discarded or discarded)
            if discard_reason:
                item.discard_reason = discard_reason
            item.history.append({
                't': round(float(t), 2), 'status': item.status,
                'world_enu': [round(float(v), 2) for v in item.world_enu]})
            return item

        revisit_of = item.finding_id if same_place else 0
        item = Finding(
            finding_id=self._next_id, task_id=self._task_id,
            epoch_id=self._epoch_id, label=label, world_enu=world,
            frame_seq=int(frame_seq), frame_size=tuple(frame_size),
            confidence=float(confidence), priority=priority, status=status,
            entry_status=entry_status, detector_hits=int(detector_hits),
            source=source, first_seen=float(t), last_seen=float(t),
            first_seen_ros_ns=int(ros_stamp_ns),
            last_seen_ros_ns=int(ros_stamp_ns), clock_source=clock_source,
            pose_at_first_seen=dict(pose_at_first_seen or {}),
            revisit_of=revisit_of, discarded=bool(discarded),
            discard_reason=discard_reason, updates=1,
            step_type=step_type,
            history=[{'t': round(float(t), 2), 'status': status,
                      'world_enu': [round(float(v), 2) for v in world]}])
        self._next_id += 1
        self._items.append(item)
        return item

    def mark_inspected(self, world_enu, t: float = 0.0,
                       step_type: str = '', notes: str = '',
                       ros_stamp_ns: int = 0) -> Finding:
        item, distance = self._nearest(tuple(float(v) for v in world_enu))
        if item is None or distance > self._radius_for(item.label):
            return None
        if _STATUS_ORDER.get('inspected', 0) > _STATUS_ORDER.get(item.status, 0):
            item.status = 'inspected'
        item.last_seen = max(item.last_seen, float(t))
        item.last_seen_ros_ns = max(item.last_seen_ros_ns, int(ros_stamp_ns))
        if step_type:
            item.step_type = step_type
        if notes:
            item.notes = notes
        item.history.append({
            't': round(float(t), 2), 'status': item.status,
            'world_enu': [round(float(v), 2) for v in item.world_enu]})
        return item

    def to_dicts(self):
        return [item.to_dict() for item in self._items]

    def confirmed(self):
        return [item for item in self._items
                if _STATUS_ORDER.get(item.status, 0) >= 1]
