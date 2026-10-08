#!/usr/bin/env python3
"""分析 .ulg 中的 AUTO_LAND 段，定位降落异常。

用法：python3 scripts/analyze_landing.py logs/2026-09-19/08_27_49.ulg

输出每个 AUTO_LAND 段的高度变化、实际下降速率、垂速分布、姿态倾角，
以及 vehicle_land_detected 各标志在该段内的翻转情况。
"""

import math
import sys

import numpy as np
from pyulog import ULog

NAV_AUTO_LAND = 18
WANTED = [
    'vehicle_status',
    'vehicle_local_position',
    'vehicle_land_detected',
    'vehicle_attitude',
    'actuator_controls_0',
    'vehicle_thrust_setpoint',
    'hover_thrust_estimate',
]
FLAGS = ['landed', 'maybe_landed', 'ground_contact', 'at_rest',
         'has_low_throttle', 'in_ground_effect', 'in_descend',
         'close_to_ground_or_skipped_check']


def load(path: str) -> dict:
    ulog = ULog(path, WANTED)
    return {d.name: d for d in ulog.data_list if d.multi_id == 0}


def tilt_deg(q: np.ndarray) -> np.ndarray:
    """由姿态四元数计算倾角（相对水平面，度）。q 顺序为 [w, x, y, z]。"""
    w, x, y, z = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    sin_pitch = np.clip(2.0 * (w * y - z * x), -1.0, 1.0)
    pitch = np.arcsin(sin_pitch)
    roll = np.arctan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    return np.degrees(np.sqrt(roll ** 2 + pitch ** 2))


def analyze(path: str) -> None:
    topics = load(path)
    status = topics.get('vehicle_status')
    position = topics.get('vehicle_local_position')
    if status is None or position is None:
        print(f'{path}: 缺少必要话题')
        return

    t_status = status.data['timestamp']
    nav = status.data['nav_state']
    indices = np.where(nav == NAV_AUTO_LAND)[0]
    print(f'\n{path}')
    print(f'  总时长 {t_status[-1] / 1e6 - t_status[0] / 1e6:.1f} s，'
          f'AUTO_LAND 状态样本 {indices.size} 个')
    if indices.size == 0:
        print('  没有 AUTO_LAND 段')
        return

    groups = np.split(indices, np.where(np.diff(indices) > 1)[0] + 1)
    for gi, group in enumerate(groups):
        t0 = t_status[group[0]]
        t1 = t_status[group[-1]]
        duration = (t1 - t0) / 1e6
        mask = (position.data['timestamp'] >= t0) & (position.data['timestamp'] <= t1)
        ts = position.data['timestamp'][mask]
        z = position.data['z'][mask]
        vz = position.data['vz'][mask]
        print(f'\n  ── AUTO_LAND 段 {gi}：时长 {duration:.1f} s，{ts.size} 个位置样本')
        if ts.size > 1:
            rate = (z[-1] - z[0]) / ((ts[-1] - ts[0]) / 1e6)
            print(f'     高度 z：{z[0]:+.3f} → {z[-1]:+.3f} m（净变化 {z[-1] - z[0]:+.3f} m）')
            print(f'     实际下沉速率：{rate:+.3f} m/s'
                  f'（正=下降；AUTO_LAND 期望约 +0.7 m/s）')
            print(f'     垂速 vz：平均 {vz.mean():+.3f}，范围 [{vz.min():+.3f}, {vz.max():+.3f}]')
            print(f'     高度波动：最低 {z.max():+.3f} / 最高 {z.min():+.3f} m'
                  f'（跨度 {z.max() - z.min():.3f} m）')

        for name, extractor in (
            ('姿态倾角', lambda t0, t1: _window(topics.get('vehicle_attitude'), t0, t1,
                                              lambda d: (tilt_deg(np.column_stack(
                                                  [d['q[0]'], d['q[1]'], d['q[2]'], d['q[3]']])),
                                                         '°'))),
            ('油门输出', lambda t0, t1: _window(topics.get('actuator_controls_0'), t0, t1,
                                              lambda d: (d['control[2]'], ''))),
            ('推力设定', lambda t0, t1: _window(topics.get('vehicle_thrust_setpoint'), t0, t1,
                                              lambda d: (d['xyz[2]'], ''))),
            ('悬停推力估计', lambda t0, t1: _window(topics.get('hover_thrust_estimate'), t0, t1,
                                                lambda d: (d['hover_thrust'], ''))),
        ):
            summary = extractor(t0, t1)
            if summary:
                print(f'     {name}：{summary}')

        land = topics.get('vehicle_land_detected')
        if land is not None:
            inside = (land.data['timestamp'] >= t0) & (land.data['timestamp'] <= t1)
            flags_text = []
            for flag in FLAGS:
                values = land.data.get(flag)
                if values is None:
                    continue
                window = values[inside]
                if window.size and window.any():
                    flags_text.append(f'{flag}={int(window.sum())}/{window.size}')
                elif window.size:
                    flags_text.append(f'{flag}=从未置位')
            print('     落地检测：' + '，'.join(flags_text) if flags_text else '     落地检测：无数据')


def _window(topic, t0: float, t1: float, extractor):
    if topic is None:
        return ''
    mask = (topic.data['timestamp'] >= t0) & (topic.data['timestamp'] <= t1)
    if not mask.any():
        return ''
    values, unit = extractor(topic.data)
    values = values[mask]
    if values.size == 0:
        return ''
    return (f'平均 {values.mean():.3f}{unit}，范围 '
            f'[{values.min():.3f}, {values.max():.3f}]{unit}')


def main() -> None:
    if len(sys.argv) < 2:
        print(__doc__)
        raise SystemExit(2)
    for path in sys.argv[1:]:
        try:
            analyze(path)
        except Exception as exc:  # noqa: BLE001
            print(f'{path}: 分析失败 {type(exc).__name__}: {exc}')


if __name__ == '__main__':
    main()
