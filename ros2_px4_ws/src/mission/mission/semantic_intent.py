"""SemanticIntent 1.0：语义意图 → 确定性 TaskSpec。

服务器只给语义（做什么/去哪/约束/置信度）；本模块做 schema 校验、稳定拒绝码、
本地事实检查和 TaskSpec 编译。模型不直接产出 TaskSpec。
"""

import json
import math
from dataclasses import dataclass, field

from .task_spec import Step, TaskError, TaskSpec

CONTRACT_VERSION = '1.0'
TASK_TYPES = ('area_cover', 'go_to', 'inspect', 'patrol', 'return')
ERROR_CODES = (
    'SCHEMA_INVALID', 'UNSUPPORTED_VERSION', 'UNKNOWN_TASK_TYPE',
    'UNSUPPORTED_ALTITUDE_FRAME', 'TARGET_UNREACHABLE', 'TARGET_IN_KEEPOUT',
    'ALTITUDE_OUT_OF_BAND', 'CLEARANCE_TOO_LOW', 'LOW_CONFIDENCE',
    'FACTS_CONFLICT', 'LOCAL_FACTS_STALE', 'OPERATOR_REQUIRED',
    'ERROR_TARGET_OUTSIDE_MAP', 'ERROR_TARGET_IN_KEEPOUT')


@dataclass
class RejectedReason:
    code: str
    field: str = ''
    detail: str = ''

    def to_dict(self):
        return {'code': self.code, 'field': self.field, 'detail': self.detail}


def _polygon_area(points):
    area = 0.0
    for i, (x1, y1) in enumerate(points):
        x2, y2 = points[(i + 1) % len(points)]
        area += x1 * y2 - x2 * y1
    return area / 2.0


def _segments_intersect(a, b, c, d):
    def cross(o, p, q):
        return (p[0] - o[0]) * (q[1] - o[1]) - (p[1] - o[1]) * (q[0] - o[0])

    def on_segment(p, q, r):
        return (min(p[0], r[0]) <= q[0] <= max(p[0], r[0])
                and min(p[1], r[1]) <= q[1] <= max(p[1], r[1])
                and abs(cross(p, q, r)) < 1e-9)

    o1, o2, o3, o4 = cross(a, b, c), cross(a, b, d), cross(c, d, a), cross(c, d, b)
    if (o1 * o2 < 0) and (o3 * o4 < 0):
        return True
    return (on_segment(a, c, b) or on_segment(a, d, b)
            or on_segment(c, a, d) or on_segment(c, b, d))


def _polygon_is_simple(points):
    n = len(points)
    for i in range(n):
        a, b = points[i], points[(i + 1) % n]
        for j in range(i + 1, n):
            if j == i or (j + 1) % n == i or (i + 1) % n == j:
                continue
            c, d = points[j], points[(j + 1) % n]
            if _segments_intersect(a, b, c, d):
                return False
    return True


def _area_spacing(area, altitude):
    line = area.get('line_spacing_m')
    overlap = area.get('overlap')
    if line is not None:
        return max(1.0, min(10.0, float(line)))
    if overlap is not None:
        footprint = 2.0 * float(altitude) * math.tan(math.radians(45.0))
        return max(1.0, min(10.0, footprint * (1.0 - float(overlap))))
    if area.get('spacing_m') is not None:
        return max(1.0, min(10.0, float(area['spacing_m'])))
    # 1.1 默认 overlap=0.25；footprint=2*alt（90° FOV）
    return max(1.0, min(10.0, 1.5 * float(altitude)))


def _point_in_polygon(east, north, polygon):
    inside = False
    j = len(polygon) - 1
    for i in range(len(polygon)):
        xi, yi = polygon[i]
        xj, yj = polygon[j]
        if ((yi > north) != (yj > north)) and (
                east < (xj - xi) * (north - yi) / (yj - yi) + xi):
            inside = not inside
        j = i
    return inside


@dataclass
class SemanticIntent:
    contract_version: str = CONTRACT_VERSION
    intent_id: str = ''
    task_type: str = 'area_cover'
    target: dict = field(default_factory=dict)
    constraints: dict = field(default_factory=dict)
    priority: str = 'normal'
    confidence: float = 1.0
    source: str = 'operator'
    facts_used: list = field(default_factory=list)
    notes: str = ''

    @classmethod
    def from_dict(cls, data):
        return cls(
            contract_version=str(data.get('contract_version', '')),
            intent_id=str(data.get('intent_id', '')),
            task_type=str(data.get('task_type', '')),
            target=dict(data.get('target') or {}),
            constraints=dict(data.get('constraints') or {}),
            priority=str(data.get('priority', 'normal')),
            confidence=float(data.get('confidence', 1.0)),
            source=str(data.get('source', 'operator')),
            facts_used=list(data.get('facts_used') or []),
            notes=str(data.get('notes', '')),
        )

    @classmethod
    def from_json(cls, text):
        return cls.from_dict(json.loads(text))

    def to_dict(self):
        return {
            'contract_version': self.contract_version,
            'intent_id': self.intent_id,
            'task_type': self.task_type,
            'target': self.target,
            'constraints': self.constraints,
            'priority': self.priority,
            'confidence': self.confidence,
            'source': self.source,
            'facts_used': self.facts_used,
            'notes': self.notes,
        }

    def validate(self):
        reasons = []
        if self.contract_version != CONTRACT_VERSION:
            reasons.append(RejectedReason(
                'UNSUPPORTED_VERSION', 'contract_version',
                f'只支持 {CONTRACT_VERSION}，得到 {self.contract_version!r}'))
        if self.task_type not in TASK_TYPES:
            reasons.append(RejectedReason(
                'UNKNOWN_TASK_TYPE', 'task_type',
                f'未知任务类型 {self.task_type!r}'))
        altitude_frame = self.constraints.get('altitude_frame', 'REL')
        if altitude_frame != 'REL':
            reasons.append(RejectedReason(
                'UNSUPPORTED_ALTITUDE_FRAME', 'constraints.altitude_frame',
                'v1.0 固定 REL'))
        if self.source == 'vlm' and float(self.confidence) < 0.5:
            reasons.append(RejectedReason(
                'LOW_CONFIDENCE', 'confidence', 'VLM 置信度 < 0.5'))
        target = self.target
        if target and target.get('frame_id', 'map') != 'map':
            reasons.append(RejectedReason(
                'SCHEMA_INVALID', 'target.frame_id', '只支持 map 系'))
        band = self.constraints.get('altitude_band_m')
        if band is not None:
            if (not isinstance(band, (list, tuple)) or len(band) != 2
                    or float(band[0]) > float(band[1])
                    or float(band[0]) < 0.5 or float(band[1]) > 30.0):
                reasons.append(RejectedReason(
                    'ALTITUDE_OUT_OF_BAND', 'constraints.altitude_band_m',
                    '需要 [low, high]，0.5–30 m，low ≤ high'))
        if self.task_type in ('go_to', 'inspect') and not target.get('world_enu'):
            reasons.append(RejectedReason(
                'SCHEMA_INVALID', 'target.world_enu', 'go_to/inspect 需要目标点'))
        if self.task_type in ('area_cover', 'patrol') and not self.constraints.get('area'):
            reasons.append(RejectedReason(
                'SCHEMA_INVALID', 'constraints.area',
                'area_cover/patrol 需要区域（area）'))
        if self.source == 'vlm' and self.task_type == 'area_cover':
            reasons.append(RejectedReason(
                'OPERATOR_REQUIRED', 'source',
                'source=vlm 的 area_cover 需要操作者确认（模型不产可靠多边形）'))
        area = self.constraints.get('area') or {}
        if area:
            if area.get('frame_id', 'map') != 'map':
                reasons.append(RejectedReason(
                    'SCHEMA_INVALID', 'constraints.area.frame_id', '只支持 map 系'))
            polygon = area.get('polygon')
            if polygon is not None:
                if not isinstance(polygon, list) or len(polygon) < 3:
                    reasons.append(RejectedReason(
                        'SCHEMA_INVALID', 'constraints.area.polygon',
                        'polygon 至少 3 个点'))
                else:
                    try:
                        points = [(float(p[0]), float(p[1])) for p in polygon]
                        if abs(_polygon_area(points)) <= 1e-6:
                            raise ValueError('面积为 0')
                        if not _polygon_is_simple(points):
                            raise ValueError('多边形自交')
                    except (TypeError, ValueError, IndexError) as exc:
                        reasons.append(RejectedReason(
                            'SCHEMA_INVALID', 'constraints.area.polygon', str(exc)))
                # 编译期 Geofence 语义：inclusion=map_boundary，exclusion=keepout_polygons。
                # 未配置 map_boundary 时跳过边界检查（保持旧行为）。
                try:
                    points = [(float(p[0]), float(p[1])) for p in polygon]
                except (TypeError, ValueError, IndexError):
                    points = []
                boundary = self.constraints.get('map_boundary')
                if points and boundary is not None:
                    try:
                        boundary_points = [(float(p[0]), float(p[1]))
                                           for p in boundary]
                    except (TypeError, ValueError, IndexError):
                        reasons.append(RejectedReason(
                            'SCHEMA_INVALID', 'constraints.map_boundary',
                            'map_boundary 必须是 [[e,n], ...]'))
                        boundary_points = []
                    for index, point in enumerate(points):
                        if boundary_points and not _point_in_polygon(
                                point[0], point[1], boundary_points):
                            reasons.append(RejectedReason(
                                'ERROR_TARGET_OUTSIDE_MAP',
                                'constraints.area.polygon',
                                f'area.polygon[{index}]=[{point[0]:.2f},{point[1]:.2f}] '
                                '在 map_boundary 外'))
                            break
                keepouts = self.constraints.get('keepout_polygons')
                if keepouts is None:
                    keepouts = self.constraints.get('keepout')
                if points and keepouts:
                    for keepout_index, keepout in enumerate(keepouts):
                        try:
                            keepout_points = [(float(p[0]), float(p[1]))
                                              for p in keepout]
                        except (TypeError, ValueError, IndexError):
                            continue
                        for index, point in enumerate(points):
                            if _point_in_polygon(point[0], point[1], keepout_points):
                                reasons.append(RejectedReason(
                                    'ERROR_TARGET_IN_KEEPOUT',
                                    'constraints.area.polygon',
                                    f'area.polygon[{index}]=[{point[0]:.2f},{point[1]:.2f}] '
                                    f'在 keepout_polygons[{keepout_index}] 内'))
                                break
                        else:
                            continue
                        break
            if area.get('line_spacing_m') is not None and float(area['line_spacing_m']) <= 0:
                reasons.append(RejectedReason(
                    'SCHEMA_INVALID', 'constraints.area.line_spacing_m', '必须 > 0'))
            if area.get('overlap') is not None:
                try:
                    overlap = float(area['overlap'])
                    if not (0.0 <= overlap < 1.0):
                        raise ValueError
                except (TypeError, ValueError):
                    reasons.append(RejectedReason(
                        'SCHEMA_INVALID', 'constraints.area.overlap', '需要 [0,1)'))
            if area.get('overlap_override') is not None:
                override = area['overlap_override']
                if (isinstance(override, bool)
                        or not isinstance(override, (int, float))):
                    reasons.append(RejectedReason(
                        'SCHEMA_INVALID', 'constraints.area.overlap_override',
                        '必须为数值或 null'))
                elif not (0.0 <= float(override) < 1.0):
                    reasons.append(RejectedReason(
                        'SCHEMA_INVALID', 'constraints.area.overlap_override',
                        '需要 [0,1)'))
        keepout = self.constraints.get('keepout') or []
        world = target.get('world_enu')
        if world and isinstance(world, (list, tuple)) and len(world) >= 2:
            for polygon in keepout:
                if _point_in_polygon(float(world[0]), float(world[1]), polygon):
                    reasons.append(RejectedReason(
                        'TARGET_IN_KEEPOUT', 'target.world_enu',
                        '目标落在 keepout 内'))
                    break
        return reasons

    def _area_notes(self, constraints, altitude):
        notes = [f'SemanticIntent {self.intent_id}（{self.source}）']
        area = constraints.get('area') or {}
        if area:
            if area.get('line_spacing_m') is not None:
                notes.append(f'area.line_spacing_m={area["line_spacing_m"]}')
            if area.get('overlap') is not None:
                notes.append(f'area.overlap={area["overlap"]}')
            if area.get('overlap_override') is not None:
                notes.append(f'area.overlap_override={area["overlap_override"]}')
            notes.append(f'area.effective_spacing_m={_area_spacing(area, altitude):.3f}')
        return notes

    def compile(self):
        reasons = self.validate()
        if reasons:
            return None, reasons
        target = self.target
        constraints = self.constraints
        band = constraints.get('altitude_band_m') or [2.0, 2.0]
        altitude = float(band[0])
        speed = max(0.1, min(1.0, float(constraints.get('speed_cap_mps', 0.5))))
        target_label = str(target.get('label', 'target'))
        target_kind = ('person' if target_label in ('person', 'victim', 'human')
                       else 'marker' if target_label in ('marker', 'color')
                       else 'any')
        steps = [Step('takeoff', {'altitude_m': altitude})]
        try:
            coordinate_frame = 'relative'
            if self.task_type in ('area_cover', 'patrol'):
                area = dict(constraints.get('area') or {})
                if area.get('polygon') is not None:
                    shape = 'polygon'
                    coordinate_frame = 'map'
                    params = {
                        'shape': 'polygon',
                        'center_east_m': 0.0,
                        'center_north_m': 0.0,
                        'polygon': area['polygon'],
                        'spacing_m': _area_spacing(area, altitude),
                        'target': {'kind': target_kind, 'label': target_label},
                        'altitude_m': altitude,
                    }
                else:
                    shape = str(area.get('shape', 'rectangle'))
                    params = {
                        'shape': shape,
                        'center_east_m': float(area.get('center_east_m', 0.0)),
                        'center_north_m': float(area.get('center_north_m', 0.0)),
                        'spacing_m': _area_spacing(area, altitude),
                        'target': {'kind': target_kind, 'label': target_label},
                        'altitude_m': altitude,
                    }
                    if shape == 'polygon':
                        params['polygon'] = area.get('polygon') or area.get('vertices')
                    elif shape == 'rectangle':
                        params['width_m'] = float(area.get('width_m', 10.0))
                        params['height_m'] = float(area.get('height_m', 10.0))
                        params['angle_deg'] = float(area.get('angle_deg', 0.0))
                    elif shape == 'square':
                        params['radius_m'] = float(area.get('radius_m', 5.0))
                    else:
                        raise ValueError(f'未知 area.shape {shape!r}')
                steps.append(Step('search', params))
            elif self.task_type in ('go_to', 'inspect'):
                world = target.get('world_enu') or [0.0, 0.0]
                east, north = float(world[0]), float(world[1])
                if math.hypot(east, north) > 40.0:
                    raise ValueError('目标超出围栏 40 m')
                steps.append(Step('goto', {'east_m': east, 'north_m': north,
                                           'altitude_m': altitude}))
                if self.task_type == 'inspect':
                    steps.append(Step('hover', {'seconds': 3.0}))
            elif self.task_type == 'return':
                steps.append(Step('return_home', {}))
            if self.task_type != 'return':
                steps.append(Step('land', {}))
            spec = TaskSpec(
                steps=steps, altitude_m=altitude, max_speed_mps=speed,
                max_duration_s=180.0, geofence_radius_m=40.0,
                source_text=self.notes or f'SemanticIntent {self.task_type}',
                notes=self._area_notes(constraints, altitude),
                coordinate_frame=coordinate_frame,
                intent_id=self.intent_id,
                contract_version=self.contract_version,
                intent_source=self.source,
                intent_confidence=float(self.confidence))
            spec.validate()
            return spec, []
        except (TaskError, ValueError, TypeError) as exc:
            return None, [RejectedReason(
                'TARGET_UNREACHABLE', 'target', str(exc))]


def main(argv=None):
    import argparse
    import sys
    parser = argparse.ArgumentParser(description='SemanticIntent → TaskSpec')
    parser.add_argument('--file', required=True)
    parser.add_argument('--json', action='store_true')
    args = parser.parse_args(argv)
    with open(args.file, encoding='utf-8') as handle:
        intent = SemanticIntent.from_json(handle.read())
    spec, reasons = intent.compile()
    if reasons:
        payload = {'accepted': False, 'rejected_reasons':
                   [reason.to_dict() for reason in reasons]}
        print(json.dumps(payload, ensure_ascii=False, indent=2))
        return 1
    if args.json:
        print(spec.to_json())
    else:
        print(spec.summary())
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
