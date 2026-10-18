"""多帧目标关联：把逐帧检测变成**确认目标**（track）。

为什么需要：单帧检测会抖、会误报；任务层需要的是"这个目标连续出现了 N 帧、
世界位置稳定"才值得悬停/报告。纯函数、不依赖 ROS，可用合成数据单测。

关联顺序（由强到弱）：
1. 两边都有世界坐标 → 世界距离 ≤ ``gate_m``；
2. 否则用 bbox IoU ≥ ``iou_gate``（没有深度时的退化路径）。
"""

from dataclasses import dataclass, field

import numpy as np


def bbox_iou(a, b) -> float:
    ax1, ay1, ax2, ay2 = a
    bx1, by1, bx2, by2 = b
    ix1, iy1 = max(ax1, bx1), max(ay1, by1)
    ix2, iy2 = min(ax2, bx2), min(ay2, by2)
    iw, ih = max(0.0, ix2 - ix1), max(0.0, iy2 - iy1)
    inter = iw * ih
    if inter <= 0:
        return 0.0
    area_a = max(0.0, ax2 - ax1) * max(0.0, ay2 - ay1)
    area_b = max(0.0, bx2 - bx1) * max(0.0, by2 - by1)
    union = area_a + area_b - inter
    return inter / union if union > 0 else 0.0


@dataclass
class Track:
    track_id: int
    world: np.ndarray = None
    last_seen: float = 0.0
    first_seen: float = 0.0
    hits: int = 0
    label: str = 'target'
    score: float = 0.0
    bbox: tuple = (0.0, 0.0, 0.0, 0.0)
    history: list = field(default_factory=list)

    def to_dict(self, confirm_hits: int):
        return {
            'track_id': self.track_id,
            'label': self.label,
            'confirmed': self.hits >= confirm_hits,
            'hits': self.hits,
            'score': round(float(self.score), 3),
            'world_enu': (None if self.world is None
                          else [round(float(c), 2) for c in self.world]),
            'bbox': [round(float(v), 1) for v in self.bbox],
            'first_seen': round(float(self.first_seen), 3),
            'last_seen': round(float(self.last_seen), 3),
        }


class TargetTracker:
    def __init__(self, *, gate_m: float = 2.0, iou_gate: float = 0.2,
                 confirm_hits: int = 3, ttl_s: float = 2.0,
                 ema_alpha: float = 0.4, max_history: int = 100):
        self.gate_m = float(gate_m)
        self.iou_gate = float(iou_gate)
        self.confirm_hits = max(int(confirm_hits), 1)
        self.ttl_s = float(ttl_s)
        self.ema_alpha = float(ema_alpha)
        self.max_history = max(int(max_history), 1)
        self._tracks = []
        self._next_id = 1

    @property
    def tracks(self):
        return list(self._tracks)

    def confirmed(self):
        return [t for t in self._tracks if t.hits >= self.confirm_hits]

    def update(self, detections, stamp: float, world_points=None):
        """喂一帧检测，返回当前轨道列表（含未确认）。

        ``detections`` 需有 ``bbox``/``score``/``label`` 属性（``Detection`` 满足）；
        ``world_points`` 与 detections 等长，元素可为 None。
        """
        detections = list(detections)
        worlds = list(world_points) if world_points is not None else [None] * len(detections)
        used = set()
        for detection, world in zip(detections, worlds):
            match = self._match(detection, world, used)
            if match is None:
                track = Track(track_id=self._next_id,
                              world=(None if world is None
                                     else np.array(world, dtype=float)),
                              last_seen=stamp,
                              first_seen=stamp, hits=1, label=detection.label,
                              score=float(detection.score),
                              bbox=tuple(float(v) for v in
                                         (detection.x1, detection.y1,
                                          detection.x2, detection.y2)))
                track.history.append((stamp, None if world is None else np.array(world)))
                self._tracks.append(track)
                self._next_id += 1
                used.add(track.track_id)
                continue

            used.add(match.track_id)
            match.hits += 1
            match.last_seen = stamp
            match.score = max(match.score * 0.8, float(detection.score))
            match.label = detection.label
            match.bbox = tuple(float(v) for v in
                               (detection.x1, detection.y1, detection.x2, detection.y2))
            if world is not None:
                if match.world is None:
                    match.world = np.array(world, dtype=float)
                else:
                    alpha = self.ema_alpha
                    match.world = (1.0 - alpha) * match.world + alpha * np.array(world)
            match.history.append((stamp, None if match.world is None else match.world.copy()))
            if len(match.history) > self.max_history:
                match.history = match.history[-self.max_history:]

        self._tracks = [t for t in self._tracks if stamp - t.last_seen <= self.ttl_s]
        return self.tracks

    def _match(self, detection, world, used):
        best = None
        best_metric = None
        for track in self._tracks:
            if track.track_id in used:
                continue
            if track.world is not None and world is not None:
                distance = float(np.linalg.norm(np.asarray(world, dtype=float) - track.world))
                if distance > self.gate_m:
                    continue
                metric = distance / self.gate_m
            else:
                iou = bbox_iou(track.bbox,
                               (detection.x1, detection.y1, detection.x2, detection.y2))
                if iou < self.iou_gate:
                    continue
                metric = 1.0 - iou
            if best_metric is None or metric < best_metric:
                best, best_metric = track, metric
        return best
