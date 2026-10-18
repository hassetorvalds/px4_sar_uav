#!/usr/bin/env python3
"""三路高度对比：只用 CSV 里已有的 px4_up / truth_up / up_offset_m 做离线诊断。

背景（2026-10-17，c1）：EKF 的 z 估计在飞行/接地段会漂（真值触地后仍报 +1.8 m），
现在还会污染高度门（"结构是障碍还是可飞越"）。飞行时 CSV 同时记录两路高度，
本脚本回答三件事：

1. 漂移有多大：offset = px4_up − truth_up 的均值/标准差/极值；
2. 漂移是否随时间单调：对 offset 做最小二乘斜率（m/min）；
3. 哪个阶段最脏：按 phase 分组统计。

用法：
    python3 scripts/analyze_height_sources.py logs/2026-10-17/c1_*.csv
    python3 scripts/analyze_height_sources.py --out /tmp/h.csv logs/2026-10-17/*.csv
"""

import argparse
import csv
import glob
import math
import os
import statistics
import sys


def _load(path):
    with open(path, newline='', encoding='utf-8') as handle:
        return list(csv.DictReader(handle))


def _num(value):
    if value is None or value == '':
        return None
    try:
        return float(value)
    except ValueError:
        return None


def _slope_per_min(points):
    """最小二乘斜率（y 单位 m，x 单位 s → 换算成 m/min）。"""
    n = len(points)
    if n < 3:
        return None
    mean_x = sum(x for x, _ in points) / n
    mean_y = sum(y for _, y in points) / n
    denom = sum((x - mean_x) ** 2 for x, _ in points)
    if denom <= 1e-9:
        return None
    numer = sum((x - mean_x) * (y - mean_y) for x, y in points)
    return numer / denom * 60.0


def summarize(path):
    rows = _load(path)
    samples = []
    per_phase = {}
    for row in rows:
        px4_up = _num(row.get('px4_up'))
        truth_up = _num(row.get('truth_up'))
        if px4_up is None or truth_up is None:
            continue
        t = _num(row.get('t'))
        if t is None:
            continue
        offset = px4_up - truth_up
        samples.append((t, offset, px4_up, truth_up, row.get('phase', '?')))
        per_phase.setdefault(row.get('phase', '?'), []).append(offset)
    if not samples:
        return {'path': path, 'rows': len(rows), 'samples': 0}

    offsets = [s[1] for s in samples]
    ts = [s[0] for s in samples]
    drift = _slope_per_min([(s[0], s[1]) for s in samples])
    return {
        'path': path,
        'rows': len(rows),
        'samples': len(samples),
        'duration': max(ts) - min(ts),
        'mean': statistics.fmean(offsets),
        'std': statistics.pstdev(offsets) if len(offsets) > 1 else 0.0,
        'min': min(offsets),
        'max': max(offsets),
        'p95_abs': sorted(abs(o) for o in offsets)[max(int(0.95 * len(offsets)) - 1, 0)],
        'drift_per_min': drift,
        'per_phase': {k: (len(v), statistics.fmean(v), max(v) - min(v))
                      for k, v in sorted(per_phase.items())},
    }


def verdict(stats):
    if not stats.get('samples'):
        return '无三路高度数据（旧版 CSV 或 truth_sample_s=0）'
    spread = stats['max'] - stats['min']
    if stats['std'] < 0.15 and spread < 0.5:
        return '两路一致性好（高度门用 PX4 也不会被漂移污染）'
    if stats['std'] < 0.35 and spread < 1.0:
        return '有中等漂移：建议高度门改用 airsim 真值或只用相对量'
    return '漂移显著：高度门必须改用 airsim 真值/相对量，PX4 绝对 z 不可用于判定'


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('csv', nargs='+', help='一个或多个 detour CSV（支持通配符）')
    parser.add_argument('--out', help='把汇总写成 CSV')
    args = parser.parse_args(argv)

    paths = []
    for pattern in args.csv:
        matched = sorted(glob.glob(pattern)) or [pattern]
        paths.extend(p for p in matched if os.path.isfile(p))
    if not paths:
        print('没有找到 CSV', file=sys.stderr)
        return 2

    results = [summarize(p) for p in paths]
    print(f'{"文件":42s} {"样本":>5s} {"时长s":>7s} {"offset均值":>10s} '
          f'{"std":>6s} {"min":>7s} {"max":>7s} {"p95|off|":>8s} {"斜率m/min":>9s}')
    for r in results:
        if not r.get('samples'):
            print(f'{os.path.basename(r["path"])[:42]:42s} {0:>5d}  —— 无三路高度数据')
            continue
        drift = '—' if r['drift_per_min'] is None else f'{r["drift_per_min"]:+.3f}'
        print(f'{os.path.basename(r["path"])[:42]:42s} {r["samples"]:5d} {r["duration"]:7.1f} '
              f'{r["mean"]:+10.3f} {r["std"]:6.3f} {r["min"]:+7.3f} {r["max"]:+7.3f} '
              f'{r["p95_abs"]:8.3f} {drift:>9s}')
        print(f'    → {verdict(r)}')
        if r['per_phase']:
            parts = [f'{k}: n={n} mean={m:+.2f} range={span:.2f}'
                     for k, (n, m, span) in r['per_phase'].items()]
            print('    阶段：' + ' | '.join(parts))

    if args.out:
        with open(args.out, 'w', newline='', encoding='utf-8') as handle:
            fields = ['file', 'samples', 'duration_s', 'offset_mean_m', 'offset_std_m',
                      'offset_min_m', 'offset_max_m', 'offset_p95_abs_m', 'drift_m_per_min',
                      'verdict']
            writer = csv.DictWriter(handle, fieldnames=fields)
            writer.writeheader()
            for r in results:
                writer.writerow({
                    'file': os.path.basename(r['path']),
                    'samples': r.get('samples', 0),
                    'duration_s': round(r.get('duration', 0.0), 1),
                    'offset_mean_m': round(r['mean'], 3) if r.get('samples') else '',
                    'offset_std_m': round(r['std'], 3) if r.get('samples') else '',
                    'offset_min_m': round(r['min'], 3) if r.get('samples') else '',
                    'offset_max_m': round(r['max'], 3) if r.get('samples') else '',
                    'offset_p95_abs_m': round(r['p95_abs'], 3) if r.get('samples') else '',
                    'drift_m_per_min': (round(r['drift_per_min'], 3)
                                        if r.get('drift_per_min') is not None else ''),
                    'verdict': verdict(r),
                })
        print(f'\n汇总已写入 {args.out}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
