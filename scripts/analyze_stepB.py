#!/usr/bin/env python3
"""汇总 Step B（悬停 + dry_run 记录）产出的决策 JSON，打印验收指标。

用法：
    python3 scripts/analyze_stepB.py [目录]        # 默认 vlm_decisions_stepB

统计口径：
- 往返耗时来自客户端实测（含隧道与 base64 编解码），字段 latency_ms；
- depth_m 是**本机**在指向像素处的采样值（远端不返回深度），
  有效指 `isfinite 且 > 0`；
- point_pixel 为 null 表示该帧是状态动作（rot_left/rot_right/stop/not_visible）。
"""

import glob
import json
import math
import os
import statistics
import sys


def _pct(values, q):
    if not values:
        return float('nan')
    ordered = sorted(values)
    index = min(len(ordered) - 1, int(round(q * (len(ordered) - 1))))
    return ordered[index]


def main(argv):
    directory = argv[1] if len(argv) > 1 else 'vlm_decisions_stepB'
    paths = sorted(glob.glob(os.path.join(directory, 'decision_*.json')))
    if not paths:
        print(f'{directory} 下没有 decision_*.json')
        return 1

    records = []
    for path in paths:
        with open(path, encoding='utf-8') as handle:
            records.append(json.load(handle))

    latencies = [r['latency_ms'] for r in records if r.get('latency_ms')]
    seqs = [r.get('frame_seq') for r in records if r.get('frame_seq') is not None]
    depths = [r['depth_m'] for r in records
              if isinstance(r.get('depth_m'), (int, float)) and math.isfinite(r['depth_m'])
              and r['depth_m'] > 0]
    points = [r['point_pixel'] for r in records if r.get('point_pixel')]

    apos = {}
    opos = {}
    for record in records:
        apos[record.get('apos_state') or '-'] = apos.get(record.get('apos_state') or '-', 0) + 1
        opos[record.get('opos_state') or '-'] = opos.get(record.get('opos_state') or '-', 0) + 1

    stamps = [r['timestamp'] for r in records if r.get('timestamp')]
    print(f'目录            {directory}')
    print(f'决策条数        {len(records)}')
    if stamps:
        print(f'时间跨度        {stamps[0]} → {stamps[-1]}（文件名时间戳，同一秒会多个）')
    if latencies:
        print(f'往返耗时 ms     最小 {min(latencies):.0f} / 均值 {statistics.mean(latencies):.0f} '
              f'/ p95 {_pct(latencies, 0.95):.0f} / 最大 {max(latencies):.0f}')
    gaps = []
    if seqs:
        gaps = [b - a for a, b in zip(seqs, seqs[1:])]
        monotonic = all(g > 0 for g in gaps)
        print(f'frame_seq       {seqs[0]} → {seqs[-1]}（严格递增：{monotonic}）')
    print(f'apos_state      {apos}')
    print(f'opos_state      {opos}')
    print(f'有像素坐标的帧   {len(points)}/{len(records)}')
    if points:
        xs = [p[0] for p in points]
        ys = [p[1] for p in points]
        print(f'指向像素 x      最小 {min(xs)} / 中位 {statistics.median(xs):.0f} / 最大 {max(xs)}')
        print(f'指向像素 y      最小 {min(ys)} / 中位 {statistics.median(ys):.0f} / 最大 {max(ys)}')
    print(f'本机深度有效帧   {len(depths)}/{len(records)}'
          + (f'（{len(depths) / len(records) * 100:.0f}%）' if records else ''))
    if depths:
        print(f'深度 m          最小 {min(depths):.2f} / 中位 {statistics.median(depths):.2f} '
              f'/ 最大 {max(depths):.2f}')
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))
