#!/usr/bin/env python3
"""时序稳定化：多次采样 Qwen 目标，取中位数，spread 过大则拒绝。"""

import argparse
import json
import math
import os
import statistics
import subprocess
import sys
import time


def run_sample(index, args):
    stamp = time.strftime('%Y%m%d-%H%M%S')
    out = f'logs/tasks/qwen_avg_{stamp}_s{index}.json'
    cmd = [sys.executable, 'scripts/qwen_target.py',
           '--instruction', args.instruction,
           '--qwen-url', args.qwen_url,
           '--geofence-m', str(args.geofence_m),
           '--altitude-m', str(args.altitude_m),
           '--out', out]
    completed = subprocess.run(cmd, cwd=args.root, capture_output=True,
                               text=True, timeout=args.timeout)
    if completed.returncode != 0:
        raise RuntimeError(f'sample {index} failed: {completed.stderr[-300:]}')
    task = json.load(open(out, encoding='utf-8'))
    goto = next(step['params'] for step in task['steps'] if step['type'] == 'goto')
    notes = task.get('notes') or []
    latency = next((float(n.split('=', 1)[1]) for n in notes
                    if n.startswith('qwen_latency_ms=')), None)
    return {'east': float(goto['east_m']), 'north': float(goto['north_m']),
            'task': out, 'latency_ms': latency,
            'confidence': task.get('intent_confidence')}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--samples', type=int, default=3)
    parser.add_argument('--max-spread-m', type=float, default=2.0)
    parser.add_argument('--instruction', default='飞到前方橙色目标附近')
    parser.add_argument('--qwen-url', default='http://127.0.0.1:18081/v1/chat/completions')
    parser.add_argument('--geofence-m', type=float, default=50.0)
    parser.add_argument('--altitude-m', type=float, default=2.0)
    parser.add_argument('--timeout', type=float, default=45.0)
    parser.add_argument('--root', default=os.getcwd())
    parser.add_argument('--out', default='')
    args = parser.parse_args()

    samples = [run_sample(i, args) for i in range(max(args.samples, 1))]
    east = statistics.median(s['east'] for s in samples)
    north = statistics.median(s['north'] for s in samples)
    spread = max(math.hypot(s['east'] - east, s['north'] - north) for s in samples)
    print(json.dumps({'samples': samples, 'median': [round(east, 2), round(north, 2)],
                      'spread_m': round(spread, 2)}, ensure_ascii=False, indent=2))
    if spread > args.max_spread_m:
        print(f'拒绝：目标 spread {spread:.2f} m > {args.max_spread_m:.2f} m',
              file=sys.stderr)
        return 2

    template = json.load(open(samples[0]['task'], encoding='utf-8'))
    for step in template['steps']:
        if step['type'] == 'goto':
            step['params']['east_m'] = round(east, 2)
            step['params']['north_m'] = round(north, 2)
    template['source_text'] = f'Qwen stabilized ({len(samples)} samples): {args.instruction}'
    template['notes'] = list(template.get('notes') or []) + [
        f'qwen_samples={len(samples)}',
        f'qwen_spread_m={spread:.2f}',
        f'qwen_median_enu={east:.2f},{north:.2f}',
    ]
    out = args.out or f'logs/tasks/qwen_stable_{time.strftime("%Y%m%d-%H%M%S")}.json'
    os.makedirs(os.path.dirname(out), exist_ok=True)
    json.dump(template, open(out, 'w', encoding='utf-8'), ensure_ascii=False, indent=2)
    print('stable task:', out)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
