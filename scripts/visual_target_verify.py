#!/usr/bin/env python3
"""主视觉 backend + Qwen 第二意见；主结果失败/分歧时采用 Qwen。

仍是 image/depth/standoff；不读取 AirSim 物体坐标。
"""

import argparse
import json
import math
import os
import subprocess
import sys
import tempfile
import time


def run_target(script, target_prompt, backend, out, args):
    cmd = [sys.executable, script,
           '--instruction', args.instruction,
           '--target-prompt', target_prompt,
           '--backend', backend,
           '--qwen-url', args.qwen_url,
           '--dino-url', args.dino_url,
           '--standoff-m', str(args.standoff_m),
           '--altitude-m', str(args.altitude_m),
           '--geofence-m', str(args.geofence_m),
           '--samples', str(args.samples),
           '--max-spread-m', str(args.max_spread_m),
           '--out', out]
    if script.endswith('visual_target.py'):
        cmd = [part for i, part in enumerate(cmd)
               if not (part == '--samples' or part == '--max-spread-m'
                       or (i > 0 and cmd[i - 1] in ('--samples', '--max-spread-m')))]
    completed = subprocess.run(cmd, capture_output=True, text=True, timeout=120)
    if completed.returncode != 0:
        return None, completed.stderr.strip()[-300:]
    task = json.load(open(out, encoding='utf-8'))
    goto = next(step['params'] for step in task['steps'] if step['type'] == 'goto')
    return {'task': task, 'east': float(goto['east_m']),
            'north': float(goto['north_m']),
            'notes': task.get('notes') or []}, ''


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--instruction', default='fly to the object ahead')
    parser.add_argument('--target-prompt', required=True)
    parser.add_argument('--primary', choices=['dino', 'qwen'], default='dino')
    parser.add_argument('--verify', choices=['qwen', 'dino', 'none'], default='qwen')
    parser.add_argument('--qwen-url', default='http://127.0.0.1:18081/v1/chat/completions')
    parser.add_argument('--dino-url', default='http://127.0.0.1:18082/detect')
    parser.add_argument('--standoff-m', type=float, default=3.0)
    parser.add_argument('--altitude-m', type=float, default=2.0)
    parser.add_argument('--geofence-m', type=float, default=50.0)
    parser.add_argument('--samples', type=int, default=3)
    parser.add_argument('--max-spread-m', type=float, default=1.5)
    parser.add_argument('--max-diff-m', type=float, default=3.0)
    parser.add_argument('--out', default='')
    args = parser.parse_args()
    stamp = time.strftime('%Y%m%d-%H%M%S')
    out = args.out or f'logs/tasks/visual_target_verify_{stamp}.json'
    os.makedirs(os.path.dirname(out), exist_ok=True)
    with tempfile.TemporaryDirectory(prefix='visual_verify_') as tmp:
        primary, primary_err = run_target(
            'scripts/visual_target_avg.py', args.target_prompt, args.primary,
            os.path.join(tmp, 'primary.json'), args)
        if args.verify == 'none':
            verify, verify_err = None, 'disabled'
        else:
            verify, verify_err = run_target(
                'scripts/visual_target_avg.py', args.target_prompt, args.verify,
                os.path.join(tmp, 'verify.json'), args)
    notes = [f'primary={args.primary}', f'verify={args.verify}']
    if primary is None:
        notes.append(f'primary_failed={primary_err}')
    if verify is None:
        notes.append(f'verify_failed={verify_err}')
    if primary and verify:
        diff = math.hypot(primary['east'] - verify['east'],
                          primary['north'] - verify['north'])
        notes.append(f'primary_target={primary["east"]:.2f},{primary["north"]:.2f}')
        notes.append(f'verify_target={verify["east"]:.2f},{verify["north"]:.2f}')
        notes.append(f'disagreement_m={diff:.2f}')
        chosen = verify if diff > args.max_diff_m else primary
        notes.append(f'selected={"verify" if diff > args.max_diff_m else "primary"}')
    elif primary:
        chosen = primary
        notes.append('selected=primary')
    elif verify:
        chosen = verify
        notes.append('selected=verify')
    else:
        raise SystemExit('both primary and verify failed')
    task = chosen['task']
    task['source_text'] = (f'visual target verify: {args.target_prompt} '
                           f'(primary {args.primary}, verify {args.verify})')
    task['intent_source'] = 'visual_target_verify'
    task['notes'] = (task.get('notes') or []) + notes
    with open(out, 'w', encoding='utf-8') as handle:
        json.dump(task, handle, ensure_ascii=False, indent=2)
    print(json.dumps({'target_prompt': args.target_prompt,
                      'selected': chosen is verify and args.verify != 'none',
                      'task': out, 'notes': notes}, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    raise SystemExit(main())
