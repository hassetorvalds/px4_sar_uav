#!/usr/bin/env python3
"""汇总实验 B（偏航扫场）的 CSV：判断远端输出是否随视角/指令变化。

用法：
    python3 scripts/analyze_yaw_sweep.py logs/2026-10-12/yaw_sweep.csv

为什么不能只看 corr(航向, apos_x)（服务器侧的提醒）：
`apos` 是**机体坐标系**的可通行落点，若模型始终选"前方自由空间"，
偏航时它在图像里也确实会一直贴底边中心——所以判据要看三样：
① `apos_x` 是否**曾偏离默认值** 0.5104；② `waypoint yaw` 是否随目标方位变化；
③ 相关系数（仅作旁证）。本脚本把三者一起打印。
"""

import csv
import math
import statistics
import sys

DEFAULT_APOS_X = 0.5104          # 户外场景的"向前"默认落点（画面底边中心）


def _finite(values):
    return [v for v in values if isinstance(v, float) and math.isfinite(v)]


def _corr(a, b):
    if len(a) < 3 or len(a) != len(b):
        return float('nan')
    mean_a, mean_b = sum(a) / len(a), sum(b) / len(b)
    cov = sum((x - mean_a) * (y - mean_b) for x, y in zip(a, b))
    var_a = math.sqrt(sum((x - mean_a) ** 2 for x in a))
    var_b = math.sqrt(sum((y - mean_b) ** 2 for y in b))
    if var_a == 0 or var_b == 0:
        return float('nan')
    return cov / (var_a * var_b)


def _number(text):
    try:
        return float(text)
    except (TypeError, ValueError):
        return float('nan')


def main(argv):
    path = argv[1] if len(argv) > 1 else 'logs/yaw_sweep.csv'
    with open(path, newline='', encoding='utf-8') as handle:
        rows = list(csv.DictReader(handle))
    if not rows:
        print(f'{path} 没有数据行')
        return 1

    phases = []
    for row in rows:
        if row['phase'] not in phases:
            phases.append(row['phase'])

    print(f'文件 {path}：{len(rows)} 帧\n')
    header = (f'{"阶段":16s} {"帧数":>4s} {"航向范围":>16s} {"apos_x(min/中位/max/幅度)":>34s} '
              f'{"wp_yaw(度) 范围":>18s} {"corr(航向,x)":>12s} {"corr(航向,yaw)":>13s}')
    print(header)
    for phase in phases:
        subset = [r for r in rows if r['phase'] == phase]
        headings = _finite([_number(r['heading_deg']) for r in subset])
        xs = _finite([_number(r['apos_x']) for r in subset])
        yaws = _finite([_number(r['wp_yaw_deg']) for r in subset])
        # 相关系数要求成对（同一行都有效）
        pairs_x = [(_number(r['heading_deg']), _number(r['apos_x'])) for r in subset]
        pairs_x = [(h, x) for h, x in pairs_x if math.isfinite(h) and math.isfinite(x)]
        pairs_y = [(_number(r['heading_deg']), _number(r['wp_yaw_deg'])) for r in subset]
        pairs_y = [(h, y) for h, y in pairs_y if math.isfinite(h) and math.isfinite(y)]
        heading_range = (f'{min(headings):7.1f}→{max(headings):7.1f}' if headings else 'n/a')
        x_text = (f'{min(xs):.4f}/{statistics.median(xs):.4f}/{max(xs):.4f}/'
                  f'{max(xs) - min(xs):.4f}' if xs else 'n/a')
        yaw_text = (f'{min(yaws):6.1f}→{max(yaws):6.1f}' if yaws else 'n/a')
        print(f'{phase:16s} {len(subset):4d} {heading_range:>16s} {x_text:>34s} '
              f'{yaw_text:>18s} {_corr(*zip(*pairs_x)) if pairs_x else float("nan"):12.2f} '
              f'{_corr(*zip(*pairs_y)) if pairs_y else float("nan"):13.2f}')

    print('\n各阶段 apos_state 分布：')
    for phase in phases:
        counts = {}
        for r in rows:
            if r['phase'] == phase:
                counts[r['apos_state'] or '-'] = counts.get(r['apos_state'] or '-', 0) + 1
        print(f'  {phase:16s} {counts}')

    deviated = 0
    for row in rows:
        value = _number(row['apos_x'])
        if math.isfinite(value) and abs(value - DEFAULT_APOS_X) > 0.02:
            deviated += 1
    finite_x = len(_finite([_number(r['apos_x']) for r in rows]))
    print(f'\n偏离默认落点(0.5104)超过 0.02 的帧：{deviated}/{len(rows)}'
          f'（有效坐标帧 {finite_x}；其余多为 rot_* 状态动作，无坐标）')
    print('判读：① 有偏离 + ② wp_yaw 随视角变化 → 模型对场景有响应；'
          '两者都恒定才需要查 checkpoint/输入。')
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))
