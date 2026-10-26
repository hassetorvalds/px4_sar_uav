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
import urllib.request

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


def _dino_ready(url):
    health = url.rsplit('/detect', 1)[0] + '/health'
    try:
        with urllib.request.urlopen(health, timeout=5) as response:
            return response.status == 200
    except Exception:  # noqa: BLE001
        return False


def _qwen_ready(url):
    health = url.rsplit('/v1/', 1)[0] + '/v1/models'
    try:
        with urllib.request.urlopen(health, timeout=5) as response:
            return response.status == 200
    except Exception:  # noqa: BLE001
        return False


_PROMPT_EN = {
    '橙色球': 'orange ball', '橙色目标': 'orange target',
    '红色屋顶': 'red roof', '灰色建筑': 'grey building',
    '建筑': 'building', '车辆': 'vehicle', '车': 'vehicle',
    '人': 'person', '人员': 'person',
}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('instruction')
    parser.add_argument('--execute', action='store_true')
    parser.add_argument('--detector', choices=['qwen', 'dino'], default='qwen')
    parser.add_argument('--target-only', action='store_true')
    parser.add_argument('--qwen-url',
                        default='http://127.0.0.1:18081/v1/chat/completions')
    parser.add_argument('--dino-url', default='http://127.0.0.1:18082/detect')
    parser.add_argument('--out', default='')
    args = parser.parse_args()

    parsed = parse_hybrid_instruction(args.instruction)
    steps = parsed['steps']
    notes = list(parsed['notes'])
    qwen_notes = []
    search_prompt = None
    for step in steps:
        if step['type'] == 'search':
            prompt = (step['params'].get('target') or {}).get('prompt')
            if prompt:
                search_prompt = prompt
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
    detector = None
    detector_log = None
    if search_prompt:
        if args.detector == 'dino':
            search_prompt = _PROMPT_EN.get(search_prompt, search_prompt)
            if not _dino_ready(args.dino_url):
                print('[nl] DINO 18082 不可达；拒绝执行', file=sys.stderr)
                return 4
        elif not _qwen_ready(args.qwen_url):
            print('[nl] 搜索目标 prompt 存在，但 Qwen 18081 不可达；拒绝执行', file=sys.stderr)
            return 4
        detector_log = open(f'logs/tasks/nl_detector_{time.strftime("%Y%m%d-%H%M%S")}.log',
                            'w', encoding='utf-8')
        detector = subprocess.Popen(
            [sys.executable, '-u', 'scripts/qwen_target_pub.py', '--ros-args',
             '-p', 'topic:=/perception/best',
             '-p', f'target_prompt:={search_prompt}',
             '-p', 'require_orange:=false',
             '-p', f'backend:={args.detector}',
             '-p', f'dino_url:={args.dino_url}',
             '-p', 'verify_roi:=true',
             '-p', 'use_bbox:=false',
             '-p', 'lock_radius_m:=2.0',
             '-p', f'instruction:={args.instruction}'],
            env=env, stdout=detector_log, stderr=subprocess.STDOUT)
        print(f'[nl] detector pid={detector.pid} log={detector_log.name}')
        ready = False
        deadline = time.time() + 12.0
        while time.time() < deadline:
            if detector.poll() is not None:
                break
            try:
                with open(detector_log.name, encoding='utf-8') as handle:
                    if 'Qwen target=' in handle.read():
                        ready = True
                        break
            except OSError:
                pass
            time.sleep(0.5)
        if not ready:
            detector.terminate()
            print(f'[nl] Qwen 检测器未在 12 s 内确认目标；'
                  f'log={detector_log.name}', file=sys.stderr)
            return 5
        print(f'[nl] 通用检测器已启动，prompt={search_prompt!r}')
    try:
        completed = subprocess.run(
            [sys.executable, 'scripts/fly_task.py', '--task-json', out, '--execute',
             '--merge-revisits'],
            env=env)
        return completed.returncode
    finally:
        if detector is not None:
            detector.terminate()
            try:
                detector.wait(timeout=5)
            except subprocess.TimeoutExpired:
                detector.kill()
        if detector_log is not None:
            detector_log.close()


if __name__ == '__main__':
    raise SystemExit(main())
