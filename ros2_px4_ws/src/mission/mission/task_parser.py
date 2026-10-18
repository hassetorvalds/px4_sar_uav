"""自然语言 → :class:`TaskSpec`（**规则解析**，可离线单测）。

定位：这是"语言指挥飞行"的**前端接口**。v1 用确定性规则，好处是可校验、可回归、
出错能给出具体原因；后续换成 VLM 解析时，只要仍产出同一个 ``TaskSpec``，
执行层与安全校验完全不用改。

支持的说法（中英混合，按子句顺序执行）：

    起飞到 2 米，向北飞 10 米，悬停 5 秒，降落
    搜索北侧 20 米、半径 10 米的区域，寻找橙色标记，高度 2 米
    去东边 15 米、北边 5 米的位置，然后返航
    向前飞 10 米，悬停 3 秒，降落
    take off to 2 m, search radius 5 m for a person, land

解析失败抛 :class:`TaskError`，并给出可操作的原因。
"""

import re
import unicodedata

from .search_pattern import estimate_duration_s, lawnmower, lawnmower_rectangle
from .task_spec import DURATION_MAX_S, Step, TaskError, TaskSpec

_CN_DIGITS = {
    '零': 0, '〇': 0, '一': 1, '二': 2, '两': 2, '三': 3, '四': 4,
    '五': 5, '六': 6, '七': 7, '八': 8, '九': 9,
}

_DIRECTIONS = {
    '东': ('east', 1.0), '东边': ('east', 1.0), '东侧': ('east', 1.0),
    'east': ('east', 1.0),
    '西': ('east', -1.0), '西边': ('east', -1.0), '西侧': ('east', -1.0),
    'west': ('east', -1.0),
    '北': ('north', 1.0), '北边': ('north', 1.0), '北侧': ('north', 1.0),
    'north': ('north', 1.0),
    '南': ('north', -1.0), '南边': ('north', -1.0), '南侧': ('north', -1.0),
    'south': ('north', -1.0),
}

_NUM = r'(?:\d+(?:\.\d+)?|[零〇一二两三四五六七八九十]+)'
_DIR = r'(?:东|西|南|北)(?:边|侧|方向)?|east|west|south|north'
_UNIT = r'(?:米|m|meters?|metres?)'

_RE_DIR_DIST = re.compile(
    rf'(?P<dir>{_DIR})\s*(?:方向|一侧)?\s*[约]?\s*'
    rf'(?:飞|走|移动|前进|fly|go|move)?\s*(?P<num>{_NUM})\s*{_UNIT}',
    re.IGNORECASE)
_RE_FORWARD = re.compile(
    rf'(?:向前|往前|前进|forward)\s*(?:飞|走|移动|fly|go|move)?\s*'
    rf'(?P<num>{_NUM})\s*{_UNIT}', re.IGNORECASE)
_RE_HOVER = re.compile(
    rf'(?:悬停|等待|hover|wait)\s*(?P<num>{_NUM})?\s*(?:秒|s|seconds?)?', re.IGNORECASE)
_RE_WIDTH = re.compile(
    rf'(?:宽|宽度|width)\s*(?P<num>{_NUM})?\s*(?:米|m)?', re.IGNORECASE)
_RE_HEIGHT = re.compile(
    rf'(?:长|长度|高|高度|height|length)\s*(?P<num>{_NUM})?\s*(?:米|m)?',
    re.IGNORECASE)
_RE_ANGLE = re.compile(
    rf'(?:角度|斜向|旋转|angle)\s*(?P<num>{_NUM})?\s*(?:度|°|deg)?',
    re.IGNORECASE)
_RE_YAW_LEFT = re.compile(
    rf'(?:向左|左转|left)\s*(?P<num>{_NUM})?\s*(?:度|deg|°)', re.IGNORECASE)
_RE_YAW_RIGHT = re.compile(
    rf'(?:向右|右转|right)\s*(?P<num>{_NUM})?\s*(?:度|deg|°)', re.IGNORECASE)
_RE_SCAN = re.compile(
    rf'(?:原地)?(?:扫描|扫视|scan|sweep)\s*(?P<num>{_NUM})?\s*(?:秒|s|seconds?)?',
    re.IGNORECASE)
_RE_TAKEOFF = re.compile(r'起飞|升空|take\s*-?\s*off', re.IGNORECASE)
_RE_LAND = re.compile(r'降落|着陆|落地|land(?:ing)?\b', re.IGNORECASE)
_RE_RETURN = re.compile(r'返航|返回|回来|回到起飞点|return|rtl', re.IGNORECASE)
_RE_SEARCH = re.compile(r'搜索|搜寻|寻找|找一下|search|find|look\s+for', re.IGNORECASE)
_RE_RADIUS = re.compile(
    rf'(?:半径|radius)\s*(?:为|是|:)?\s*(?P<num>{_NUM})\s*{_UNIT}?', re.IGNORECASE)
_RE_RADIUS_ALT = re.compile(
    rf'(?P<num>{_NUM})\s*{_UNIT}\s*(?:半径|范围)', re.IGNORECASE)
_RE_SPACING = re.compile(
    rf'(?:间距|间隔|spacing)\s*(?:为|是|:)?\s*(?P<num>{_NUM})\s*{_UNIT}?', re.IGNORECASE)
_RE_ALTITUDE = re.compile(
    rf'(?:高度|飞行高度|altitude|alt)\s*(?:为|到|是|:)?\s*(?P<num>{_NUM})\s*{_UNIT}?',
    re.IGNORECASE)
_RE_TAKEOFF_ALT = re.compile(
    rf'(?:起飞到|升到|爬到|take\s*-?\s*off\s+to|climb\s+to)\s*(?P<num>{_NUM})\s*{_UNIT}',
    re.IGNORECASE)
_RE_DURATION = re.compile(
    rf'(?:最多|不超过|时限|超时|timeout|max(?:imum)?)\s*(?P<num>{_NUM})\s*(?:秒|s|seconds?)',
    re.IGNORECASE)

#: 颜色目标的 HSV 预设（H 度，S/V 0–1）；与 perception 的 backend 一致。
COLOR_PRESETS = {
    'orange': ((15.0, 0.5, 0.4), (40.0, 1.0, 1.0)),
    'red': ((350.0, 0.5, 0.4), (10.0, 1.0, 1.0)),
    'yellow': ((40.0, 0.5, 0.5), (65.0, 1.0, 1.0)),
    'blue': ((200.0, 0.5, 0.4), (260.0, 1.0, 1.0)),
}


def _normalize(text: str) -> str:
    return unicodedata.normalize('NFKC', str(text)).strip()


def _num(token) -> float:
    """阿拉伯数字或 0–99 的中文数字（'' → 1）。"""
    if token is None or token == '':
        return 1.0
    token = str(token)
    try:
        return float(token)
    except ValueError:
        pass
    if '十' in token:
        left, _, right = token.partition('十')
        tens = _CN_DIGITS.get(left, 1) if left else 1
        ones = _CN_DIGITS.get(right, 0) if right else 0
        return float(tens * 10 + ones)
    if len(token) == 1 and token in _CN_DIGITS:
        return float(_CN_DIGITS[token])
    raise TaskError(f'无法识别的数字：{token!r}')


def _clauses(text: str):
    parts = re.split(r'[，,。；;！!？?]|然后|接着|之后', text)
    return [p.strip() for p in parts if p.strip()]


def _direction_vector(clause: str):
    east = north = 0.0
    consumed = []
    for match in _RE_DIR_DIST.finditer(clause):
        key = match.group('dir').lower()
        axis, sign = _DIRECTIONS[key]
        distance = _num(match.group('num'))
        if axis == 'east':
            east += sign * distance
        else:
            north += sign * distance
        consumed.append(match.span())
    return east, north, consumed


def _target_from_clause(clause: str):
    lowered = clause.lower()
    if re.search(r'人员|人|person|victim|survivor', lowered):
        return {'kind': 'person', 'label': 'person'}
    for name, (lower, upper) in COLOR_PRESETS.items():
        if re.search(name, lowered):
            return {'kind': 'color', 'label': name,
                    'hsv_lower': list(lower), 'hsv_upper': list(upper)}
    if re.search(r'橙色|橘色|orange', lowered):
        lower, upper = COLOR_PRESETS['orange']
        return {'kind': 'color', 'label': 'orange',
                'hsv_lower': list(lower), 'hsv_upper': list(upper)}
    if re.search(r'红色|red', lowered):
        lower, upper = COLOR_PRESETS['red']
        return {'kind': 'color', 'label': 'red',
                'hsv_lower': list(lower), 'hsv_upper': list(upper)}
    if re.search(r'标记|信标|目标|marker|beacon|target', lowered):
        return {'kind': 'marker', 'label': 'marker'}
    return {'kind': 'any', 'label': 'target'}


def _intent(clause: str):
    lowered = clause.lower()
    if _RE_RETURN.search(clause):
        return 'return_home'
    if _RE_LAND.search(clause):
        return 'land'
    if _RE_SEARCH.search(clause):
        return 'search'
    if _RE_SCAN.search(clause):
        return 'scan'
    if _RE_YAW_LEFT.search(clause) or _RE_YAW_RIGHT.search(clause):
        return 'yaw'
    if _RE_TAKEOFF.search(clause):
        return 'takeoff'
    if _RE_HOVER.search(clause):
        return 'hover'
    if _RE_FORWARD.search(clause):
        return 'forward'
    if _RE_DIR_DIST.search(clause):
        return 'goto'
    return None


def parse_task(text: str) -> TaskSpec:
    """把一句（或几句）中文/英文指令解析成经过校验的 TaskSpec。"""
    normalized = _normalize(text)
    if not normalized:
        raise TaskError('指令为空')

    altitude = 2.0
    max_duration = 180.0
    notes = []
    steps = []
    pending_target = None

    # 全局高度/时限（先扫一遍；子句里更具体的会覆盖）
    alt_match = _RE_ALTITUDE.search(normalized) or _RE_TAKEOFF_ALT.search(normalized)
    if alt_match:
        altitude = _num(alt_match.group('num'))
    dur_match = _RE_DURATION.search(normalized)
    if dur_match:
        max_duration = _num(dur_match.group('num'))

    for clause in _clauses(normalized):
        intent = _intent(clause)
        if intent is None:
            continue
        east, north, consumed = _direction_vector(clause)

        if intent == 'return_home':
            steps.append(Step('return_home', {}))
            continue
        if intent == 'land':
            steps.append(Step('land', {}))
            continue
        if intent == 'takeoff':
            takeoff_alt = altitude
            match = _RE_TAKEOFF_ALT.search(clause)
            if match:
                takeoff_alt = _num(match.group('num'))
            steps.append(Step('takeoff', {'altitude_m': takeoff_alt}))
            continue
        if intent == 'search':
            radius_match = _RE_RADIUS.search(clause) or _RE_RADIUS_ALT.search(clause)
            width_match = _RE_WIDTH.search(clause)
            height_match = _RE_HEIGHT.search(clause)
            is_rect = width_match is not None or height_match is not None
            has_area = (radius_match is not None or is_rect
                        or abs(east) > 1e-6 or abs(north) > 1e-6)
            target = _target_from_clause(clause)
            if not has_area:
                # "寻找橙色标记" 这类只给目标、不给区域的子句：
                # 挂到上一条 search 上，或留给下一条带区域的 search。
                if target['kind'] != 'any':
                    pending_target = target
                    if steps and steps[-1].type == 'search':
                        steps[-1].params['target'] = target
                continue
            radius = _num(radius_match.group('num')) if radius_match else 10.0
            spacing_match = _RE_SPACING.search(clause)
            target = pending_target if (target['kind'] == 'any'
                                        and pending_target is not None) else target
            params = {
                'center_east_m': east,
                'center_north_m': north,
                'target': target,
                'altitude_m': altitude,
            }
            if is_rect:
                width = (_num(width_match.group('num'))
                         if width_match and width_match.group('num') else 10.0)
                if height_match and height_match.group('num'):
                    height = _num(height_match.group('num'))
                elif width_match and width_match.group('num'):
                    height = width
                else:
                    height = 10.0
                if not (width_match and width_match.group('num')):
                    width = height
                angle = (_num(_RE_ANGLE.search(clause).group('num'))
                         if _RE_ANGLE.search(clause)
                         and _RE_ANGLE.search(clause).group('num') else 0.0)
                spacing = (_num(spacing_match.group('num')) if spacing_match
                           else max(min(width, height) / 2.0, 2.0))
                params.update({'shape': 'rectangle', 'width_m': width,
                               'height_m': height, 'angle_deg': angle,
                               'spacing_m': spacing})
            else:
                spacing = (_num(spacing_match.group('num')) if spacing_match
                           else max(radius / 2.0, 2.0))
                params.update({'shape': 'square', 'radius_m': radius,
                               'spacing_m': spacing})
            steps.append(Step('search', params))
            pending_target = None
            continue
        if intent == 'scan':
            match = _RE_SCAN.search(clause)
            seconds = _num(match.group('num')) if match and match.group('num') else 20.0
            steps.append(Step('scan', {'seconds': seconds, 'rate_deg_s': 15.0}))
            continue
        if intent == 'yaw':
            left = _RE_YAW_LEFT.search(clause)
            right = _RE_YAW_RIGHT.search(clause)
            match = left or right
            degrees = _num(match.group('num')) if match and match.group('num') else 90.0
            if right is not None and left is None:
                degrees = -degrees
            steps.append(Step('yaw', {'degrees': degrees, 'rate_deg_s': 20.0}))
            continue
        if intent == 'hover':
            match = _RE_HOVER.search(clause)
            seconds = _num(match.group('num')) if match and match.group('num') else 5.0
            steps.append(Step('hover', {'seconds': seconds}))
            continue
        if intent == 'forward':
            match = _RE_FORWARD.search(clause)
            if not match:
                continue
            steps.append(Step('forward', {'meters': _num(match.group('num'))}))
            continue
        if intent == 'goto':
            if abs(east) < 1e-6 and abs(north) < 1e-6:
                continue
            steps.append(Step('goto', {'east_m': east, 'north_m': north,
                                       'altitude_m': altitude}))
            continue

    if not steps:
        raise TaskError(
            '没有识别出可执行动作。支持：起飞/飞到某方向 N 米/向前 N 米/搜索半径 N 米/'
            '悬停 N 秒/返航/降落。示例："起飞到 2 米，向北飞 10 米，悬停 5 秒，降落"')

    # 安全默认：补起飞、补降落
    if steps[0].type != 'takeoff':
        steps.insert(0, Step('takeoff', {'altitude_m': altitude}))
        notes.append('指令未显式说起飞：已自动补"起飞到任务高度"')
    if steps[-1].type not in ('land', 'return_home'):
        steps.append(Step('land', {}))
        notes.append('指令未显式说降落：已自动补"降落"')

    # 搜索任务：按 0.5 m/s 粗估时长；若超过当前时限，自动放宽（有上限）并留痕
    for step in steps:
        if step.type != 'search':
            continue
        params = step.params
        if str(params.get('shape', 'square')).lower() == 'rectangle':
            waypoints = lawnmower_rectangle(
                params['center_east_m'], params['center_north_m'],
                params['width_m'], params['height_m'], params['spacing_m'],
                params.get('angle_deg', 0.0))
        else:
            waypoints = lawnmower(params['center_east_m'], params['center_north_m'],
                                  params['radius_m'], params['spacing_m'])
        estimate = estimate_duration_s(waypoints, 0.5)
        if estimate * 1.5 > max_duration:
            max_duration = min(DURATION_MAX_S, estimate * 1.5)
            notes.append(f'搜索预计约 {estimate:.0f} s，任务时限自动放宽到 {max_duration:.0f} s')

    spec = TaskSpec(steps=steps, altitude_m=altitude, max_duration_s=max_duration,
                    source_text=text, notes=notes)
    return spec.validate()


def main(argv=None):
    import sys

    if argv is None:
        argv = sys.argv[1:]
    if not argv:
        print('用法：python3 -m mission.task_parser "起飞到 2 米，向北飞 10 米，降落"')
        return 2
    try:
        spec = parse_task(' '.join(argv))
    except TaskError as exc:
        print(f'解析失败：{exc}')
        return 1
    print(spec.to_json())
    print(f'\n摘要：{spec.summary()}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
