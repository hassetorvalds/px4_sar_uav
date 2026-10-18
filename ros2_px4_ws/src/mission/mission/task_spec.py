"""任务描述（TaskSpec）：自然语言解析后的**唯一可控中间层**。

设计原则（README 第 7 节）：

- VLM/语言层只输出**任务级语义**，绝不输出姿态/电机量；
- 任务在执行前必须先过这一层的 schema 校验，非法/超范围直接拒绝；
- 执行层只认识 ``Step``，不认识自然语言。

本模块**不依赖 ROS**，便于单测与离线校验。
"""

import json
import math
from dataclasses import dataclass, field

SCHEMA_VERSION = 1

#: 安全包络（执行层还会再夹一次；这里先拒绝明显越界的指令）
ALTITUDE_MIN_M = 0.5
ALTITUDE_MAX_M = 10.0
SPEED_MIN_MPS = 0.1
SPEED_MAX_MPS = 1.0
DURATION_MIN_S = 10.0
DURATION_MAX_S = 600.0
GEOFENCE_MIN_M = 2.0
GEOFENCE_MAX_M = 50.0
SEARCH_RADIUS_MIN_M = 1.0
SEARCH_RADIUS_MAX_M = 30.0
SEARCH_SPACING_MIN_M = 1.0
SEARCH_SPACING_MAX_M = 10.0
HOVER_MIN_S = 1.0
HOVER_MAX_S = 120.0

STEP_TYPES = ('takeoff', 'goto', 'forward', 'search', 'hover', 'land', 'return_home')


class TaskError(ValueError):
    """任务描述非法（schema/范围/顺序）。"""


@dataclass
class Step:
    type: str
    params: dict = field(default_factory=dict)

    def to_dict(self):
        return {'type': self.type, 'params': dict(self.params)}

    @classmethod
    def from_dict(cls, data):
        if not isinstance(data, dict):
            raise TaskError(f'step 必须是对象：{data!r}')
        step_type = data.get('type')
        if step_type not in STEP_TYPES:
            raise TaskError(f'未知 step 类型 {step_type!r}；可选 {STEP_TYPES}')
        params = data.get('params') or {}
        if not isinstance(params, dict):
            raise TaskError('step.params 必须是对象')
        return cls(step_type, params)


@dataclass
class TaskSpec:
    steps: list
    altitude_m: float = 2.0
    max_speed_mps: float = 0.5
    max_duration_s: float = 180.0
    geofence_radius_m: float = 40.0
    source_text: str = ''
    notes: list = field(default_factory=list)
    version: int = SCHEMA_VERSION

    # ------------------------------------------------------------------
    # 校验
    # ------------------------------------------------------------------

    def validate(self):
        _check_range('altitude_m', self.altitude_m, ALTITUDE_MIN_M, ALTITUDE_MAX_M)
        _check_range('max_speed_mps', self.max_speed_mps, SPEED_MIN_MPS, SPEED_MAX_MPS)
        _check_range('max_duration_s', self.max_duration_s, DURATION_MIN_S, DURATION_MAX_S)
        _check_range('geofence_radius_m', self.geofence_radius_m,
                     GEOFENCE_MIN_M, GEOFENCE_MAX_M)
        if not self.steps:
            raise TaskError('任务没有任何 step')
        if self.steps[0].type != 'takeoff':
            raise TaskError('第一个 step 必须是 takeoff（安全：不允许在未起飞时发运动指令）')

        landed = False
        for index, step in enumerate(self.steps):
            if step.type not in STEP_TYPES:
                raise TaskError(f'第 {index + 1} 个 step 类型未知：{step.type!r}'
                                f'；可选 {STEP_TYPES}')
            if landed:
                raise TaskError(f'第 {index + 1} 个 step 出现在 land/return_home 之后')
            self._validate_step(step)
            if step.type in ('land', 'return_home'):
                landed = True
                if index != len(self.steps) - 1:
                    raise TaskError('land/return_home 必须是最后一个 step')
        if not landed:
            raise TaskError('任务缺少收尾（land/return_home）——安全默认要求在末尾降落')
        return self

    def _validate_step(self, step):
        p = step.params
        if step.type == 'takeoff':
            _check_range('takeoff.altitude_m', float(p.get('altitude_m', self.altitude_m)),
                         ALTITUDE_MIN_M, ALTITUDE_MAX_M)
        elif step.type == 'goto':
            east = float(p.get('east_m', 0.0))
            north = float(p.get('north_m', 0.0))
            distance = math.hypot(east, north)
            if distance > self.geofence_radius_m:
                raise TaskError(
                    f'goto 目标距起飞点 {distance:.1f} m，超出围栏 {self.geofence_radius_m:.1f} m')
            if distance < 0.1:
                raise TaskError('goto 目标与起飞点重合（无位移）')
        elif step.type == 'forward':
            _check_range('forward.meters', float(p.get('meters', 0.0)), 0.1,
                         self.geofence_radius_m)
        elif step.type == 'search':
            _check_range('search.radius_m', float(p.get('radius_m', 10.0)),
                         SEARCH_RADIUS_MIN_M, SEARCH_RADIUS_MAX_M)
            _check_range('search.spacing_m', float(p.get('spacing_m', 4.0)),
                         SEARCH_SPACING_MIN_M, SEARCH_SPACING_MAX_M)
            center_east = float(p.get('center_east_m', 0.0))
            center_north = float(p.get('center_north_m', 0.0))
            if math.hypot(center_east, center_north) + float(p.get('radius_m', 10.0)) \
                    > self.geofence_radius_m:
                raise TaskError('search 区域超出围栏（中心距 + 半径 > geofence）')
            target = p.get('target') or {}
            if not isinstance(target, dict) or 'kind' not in target:
                raise TaskError('search.target 必须是含 kind 的对象')
            if target['kind'] not in ('color', 'person', 'marker', 'any'):
                raise TaskError(f'未知 target.kind {target["kind"]!r}')
        elif step.type == 'hover':
            _check_range('hover.seconds', float(p.get('seconds', 5.0)),
                         HOVER_MIN_S, HOVER_MAX_S)
        return

    # ------------------------------------------------------------------
    # 序列化
    # ------------------------------------------------------------------

    def to_dict(self):
        return {
            'version': self.version,
            'source_text': self.source_text,
            'altitude_m': round(float(self.altitude_m), 2),
            'max_speed_mps': round(float(self.max_speed_mps), 2),
            'max_duration_s': round(float(self.max_duration_s), 1),
            'geofence_radius_m': round(float(self.geofence_radius_m), 1),
            'steps': [s.to_dict() for s in self.steps],
            'notes': list(self.notes),
        }

    def to_json(self):
        return json.dumps(self.to_dict(), ensure_ascii=False)

    @classmethod
    def from_dict(cls, data):
        if not isinstance(data, dict):
            raise TaskError('TaskSpec 必须是对象')
        version = int(data.get('version', SCHEMA_VERSION))
        if version != SCHEMA_VERSION:
            raise TaskError(f'不支持的 schema 版本 {version}（当前 {SCHEMA_VERSION}）')
        steps = [Step.from_dict(item) for item in (data.get('steps') or [])]
        spec = cls(
            steps=steps,
            altitude_m=float(data.get('altitude_m', 2.0)),
            max_speed_mps=float(data.get('max_speed_mps', 0.5)),
            max_duration_s=float(data.get('max_duration_s', 180.0)),
            geofence_radius_m=float(data.get('geofence_radius_m', 40.0)),
            source_text=str(data.get('source_text', '')),
            notes=list(data.get('notes') or []),
            version=version,
        )
        return spec.validate()

    @classmethod
    def from_json(cls, text):
        try:
            data = json.loads(text)
        except json.JSONDecodeError as exc:
            raise TaskError(f'JSON 解析失败：{exc}') from exc
        return cls.from_dict(data)

    def summary(self):
        """给人看的一行摘要（CLI/日志用）。"""
        parts = []
        for step in self.steps:
            if step.type == 'takeoff':
                parts.append(f'起飞 {step.params.get("altitude_m", self.altitude_m):.1f} m')
            elif step.type == 'goto':
                parts.append(f'飞到 (东 {step.params.get("east_m", 0):+.1f}, '
                             f'北 {step.params.get("north_m", 0):+.1f})')
            elif step.type == 'forward':
                parts.append(f'向前 {step.params.get("meters", 0):.1f} m')
            elif step.type == 'search':
                parts.append(f'搜索 半径 {step.params.get("radius_m", 10):.1f} m'
                             f'（{step.params.get("target", {}).get("kind", "target")}）')
            elif step.type == 'hover':
                parts.append(f'悬停 {step.params.get("seconds", 5):.0f} s')
            elif step.type == 'land':
                parts.append('降落')
            elif step.type == 'return_home':
                parts.append('返航降落')
        return ' → '.join(parts)


def _check_range(name, value, low, high):
    try:
        value = float(value)
    except (TypeError, ValueError) as exc:
        raise TaskError(f'{name} 不是数字：{value!r}') from exc
    if not math.isfinite(value):
        raise TaskError(f'{name} 不是有限数：{value!r}')
    if not (low <= value <= high):
        raise TaskError(f'{name}={value} 超出允许范围 [{low}, {high}]')
    return value
