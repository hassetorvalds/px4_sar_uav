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
    'FACTS_CONFLICT', 'LOCAL_FACTS_STALE', 'OPERATOR_REQUIRED')


@dataclass
class RejectedReason:
    code: str
    field: str = ''
    detail: str = ''

    def to_dict(self):
        return {'code': self.code, 'field': self.field, 'detail': self.detail}


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
                'area_cover/patrol 需要区域（area 扩展）'))
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
            if self.task_type in ('area_cover', 'patrol'):
                area = dict(constraints.get('area') or {})
                shape = str(area.get('shape', 'rectangle'))
                params = {
                    'shape': shape,
                    'center_east_m': float(area.get('center_east_m', 0.0)),
                    'center_north_m': float(area.get('center_north_m', 0.0)),
                    'spacing_m': float(area.get('spacing_m', 2.0)),
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
                notes=[f'SemanticIntent {self.intent_id}（{self.source}）'])
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
