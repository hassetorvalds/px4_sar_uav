#!/usr/bin/env python3
"""Hybrid NL：文本动作 + Qwen 视觉目标槽 → TaskSpec →（可选）执行。

例：
  source scripts/env.sh
  python3 scripts/fly_nl.py "起飞到 2 米，飞到前方橙色目标附近，悬停 3 秒，降落"
  python3 scripts/fly_nl.py "..." --execute
"""

import argparse
import json
import os
import subprocess
import sys
import time

from mission.hybrid_parser import parse_hybrid_instruction


def qwen_target(instruction, out, qwen_url):
    cmd = [sys.executable, 'scripts/qwen_target.py',
           '--instruction', instruction, '--qwen-url', qwen_url, '--out', out]
    completed = subprocess.run(cmd, capture_output=True, text=True, timeout=60)
    if completed.returncode != 0:
        raise RuntimeError(completed.stderr.strip()[-500:] or
                           f'qwen_target 退出码 {completed.returncode}')
    task = json.load(open(out, encoding='utf-8'))
    goto = next(step['params'] for step in task['steps'] if step['type'] == 'goto')
    return goto, (task.get('notes') or [])


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('instruction')
    parser.add_argument('--execute', action='store_true')
    parser.add_argument('--target-only', action='store_true')
    parser.add_argument('--qwen-url',
                        default='http://127.0.0.1:18081/v1/chat/completions')
    parser.add_argument('--out', default='')
    args = parser.parse_args()

    parsed = parse_hybrid_instruction(args.instruction)
    steps = parsed['steps']
    notes = list(parsed['notes'])
    qwen_notes = []
    if parsed['visual_index'] is not None:
        stamp = time.strftime('%Y%m%d-%H%M%S')
        qwen_task = f'logs/tasks/nl_qwen_{stamp}.json'
        os.makedirs('logs/tasks', exist_ok=True)
        goto, qwen_notes = qwen_target(args.instruction, qwen_task, args.qwen_url)
        index = parsed['visual_index']
        steps[index] = {'type': 'goto', 'params': {
            'east_m': goto['east_m'], 'north_m': goto['north_m'],
            'altitude_m': parsed['altitude_m']}}
        notes.append(f'visual_target=qwen:{goto["east_m"]},{goto["north_m"]}')
        notes.extend(qwen_notes)
    else:
        notes.append('hybrid_text_only')

    task = {
        'version': 1,
        'source_text': args.instruction,
        'altitude_m': parsed['altitude_m'],
        'altitude_frame': 'REL',
        'max_speed_mps': 0.5,
        'max_duration_s': 180.0,
        'geofence_radius_m': 50.0,
        'coordinate_frame': 'relative',
        'intent_source': ('qwen_grounding' if parsed['visual_index'] is not None
                          else 'hybrid_text'),
        'steps': steps,
        'notes': notes,
    }
    out = args.out or f'logs/tasks/nl_hybrid_{time.strftime("%Y%m%d-%H%M%S")}.json'
    os.makedirs(os.path.dirname(out), exist_ok=True)
    with open(out, 'w', encoding='utf-8') as handle:
        json.dump(task, handle, ensure_ascii=False, indent=2)
    print(json.dumps({'instruction': args.instruction, 'steps': steps,
                      'notes': notes, 'task': out},
                     ensure_ascii=False, indent=2))
    if args.target_only or not args.execute:
        print('未执行；确认后加 --execute')
        return 0

    env = dict(os.environ)
    env['FLY_TASK_CONFIRM'] = '1'
    completed = subprocess.run(
        [sys.executable, 'scripts/fly_task.py', '--task-json', out, '--execute'],
        env=env)
    return completed.returncode


if __name__ == '__main__':
    raise SystemExit(main())
