#!/usr/bin/env python3
"""多帧稳定视觉目标：多次 visual_target → 中位数 + spread 拒绝。

仍然是 image/depth/standoff；不读取 AirSim 物体坐标。
"""

import argparse
import json
import math
import os
import subprocess
import sys
import tempfile
import time

from mission.visual_target import median_enu_target


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--instruction', default='fly to the object ahead')
    parser.add_argument('--target-prompt', required=True)
    parser.add_argument('--backend', choices=['dino', 'qwen'], default='dino')
    parser.add_argument('--dino-url', default='http://127.0.0.1:18082/detect')
    parser.add_argument('--qwen-url', default='http://127.0.0.1:18081/v1/chat/completions')
    parser.add_argument('--standoff-m', type=float, default=3.0)
    parser.add_argument('--altitude-m', type=float, default=2.0)
    parser.add_argument('--geofence-m', type=float, default=50.0)
    parser.add_argument('--samples', type=int, default=3)
    parser.add_argument('--max-spread-m', type=float, default=1.5)
    parser.add_argument('--edge-margin-px', type=float, default=5.0)
    parser.add_argument('--out', default='')
    args = parser.parse_args()
    if args.samples < 1:
        raise SystemExit('--samples must be >= 1')

    samples = []
    notes = []
    task = None
    with tempfile.TemporaryDirectory(prefix='visual_avg_') as tmp:
        for index in range(args.samples):
            path = os.path.join(tmp, f'sample_{index}.json')
            cmd = [sys.executable, 'scripts/visual_target.py',
                   '--instruction', args.instruction,
                   '--target-prompt', args.target_prompt,
                   '--backend', args.backend,
                   '--dino-url', args.dino_url,
                   '--qwen-url', args.qwen_url,
                   '--standoff-m', str(args.standoff_m),
                   '--altitude-m', str(args.altitude_m),
                   '--geofence-m', str(args.geofence_m),
                   '--edge-margin-px', str(args.edge_margin_px),
                   '--out', path]
            completed = subprocess.run(cmd, capture_output=True, text=True, timeout=90)
            if completed.returncode != 0:
                raise SystemExit(completed.stderr.strip()[-500:] or
                                 f'visual_target sample {index} failed')
            task = json.load(open(path, encoding='utf-8'))
            goto = next(step['params'] for step in task['steps']
                        if step['type'] == 'goto')
            samples.append((float(goto['east_m']), float(goto['north_m'])))
            notes.append(f'sample_{index}={goto["east_m"]},{goto["north_m"]}')
            if index + 1 < args.samples:
                time.sleep(0.2)

    east, north, spread = median_enu_target(samples, args.max_spread_m)
    goto = next(step['params'] for step in task['steps'] if step['type'] == 'goto')
    goto['east_m'] = round(east, 2)
    goto['north_m'] = round(north, 2)
    task['source_text'] = (f'visual target avg: {args.target_prompt} '
                           f'({args.samples} samples, standoff {args.standoff_m:g} m)')
    task['intent_source'] = 'visual_target_avg'
    task.setdefault('notes', []).extend(notes)
    task['notes'].append(f'median_target={east:.2f},{north:.2f}')
    task['notes'].append(f'target_spread_m={spread:.2f}')
    out = args.out or f'logs/tasks/visual_target_avg_{time.strftime("%Y%m%d-%H%M%S")}.json'
    os.makedirs(os.path.dirname(out), exist_ok=True)
    with open(out, 'w', encoding='utf-8') as handle:
        json.dump(task, handle, ensure_ascii=False, indent=2)
    print(json.dumps({
        'backend': args.backend, 'target_prompt': args.target_prompt,
        'samples': samples, 'median_target': [round(east, 2), round(north, 2)],
        'spread_m': round(spread, 2), 'task': out,
    }, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    raise SystemExit(main())
