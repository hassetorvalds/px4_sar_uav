#!/usr/bin/env python3
"""自然语言指挥飞行（CLI）：解析 → 展示计划 →（可选）执行。

默认**只解析、不飞**；加 ``--execute`` 才会起执行节点并解锁。

用法::

    # 1) 看看这句话会被解析成什么（安全）
    scripts/fly_task.py "起飞到 2 米，向北飞 10 米，悬停 5 秒，降落"

    # 2) 真飞（要求 Windows AirSim + PX4 + Agent + ROS 栈已就绪）
    scripts/fly_task.py "搜索北侧 20 米、半径 10 米的区域，寻找橙色标记，高度 2 米，降落" --execute

    # 3) 走速度仲裁（栈用默认 with_arbiter:=true 时）
    scripts/fly_task.py "..." --execute --via-arbiter
"""

import argparse
import json
import os
import shlex
import shutil
import subprocess
import sys
import time

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(_ROOT, 'ros2_px4_ws', 'src', 'mission'))

from mission.executor_core import search_waypoints  # noqa: E402
from mission.intent import IntentFrame, merge_intent, parse_intent  # noqa: E402
from mission.report import summarize  # noqa: E402
from mission.search_pattern import estimate_duration_s, lawnmower  # noqa: E402
from mission.task_spec import TaskError  # noqa: E402


def _print_intent(frame):
    print('-' * 64)
    print(f'意图：来源 {frame.source}，置信度 {frame.confidence:.2f}')
    if frame.missing:
        print('缺失/默认：' + '、'.join(frame.missing))
    if frame.warnings:
        for warning in frame.warnings:
            print(f'提示：{warning}')
    facts = [fact for fact in frame.facts if not fact.key.startswith(
        ('altitude_m', 'max_duration_s', 'geofence_radius_m'))]
    if facts:
        print('事实（本地优先）：')
        for fact in facts:
            print(f'  · {fact.key} = {fact.value!r} [{fact.source}]')


def _print_plan(spec, json_only=False):
    if json_only:
        print(spec.to_json())
        return
    print('=' * 64)
    print(f'指令：{spec.source_text}')
    print(f'摘要：{spec.summary()}')
    print(f'高度 {spec.altitude_m:.1f} m {spec.altitude_frame} | '
          f'速度 ≤{spec.max_speed_mps:.2f} m/s | '
          f'时限 {spec.max_duration_s:.0f} s | 围栏 {spec.geofence_radius_m:.0f} m')
    print('-' * 64)
    for index, step in enumerate(spec.steps, start=1):
        print(f'  {index}. {step.type:11s} {json.dumps(step.params, ensure_ascii=False)}')
    for index, step in enumerate(spec.steps, start=1):
        if step.type == 'search':
            params = step.params
            waypoints = lawnmower(params['center_east_m'], params['center_north_m'],
                                  params['radius_m'], params['spacing_m'])
            estimate = estimate_duration_s(waypoints, spec.max_speed_mps)
            print(f'  搜索展开：{len(waypoints)} 个航点，'
                  f'折线 {estimate * spec.max_speed_mps:.0f} m，预计 {estimate:.0f} s')
    if spec.notes:
        print('-' * 64)
        for note in spec.notes:
            print(f'  · {note}')
    print('=' * 64)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('text', nargs='+', help='自然语言指令')
    parser.add_argument('--execute', action='store_true', help='真正执行（默认只解析）')
    parser.add_argument('--json', action='store_true', help='只输出 TaskSpec JSON')
    parser.add_argument('--intent-json', action='store_true',
                        help='只输出 IntentFrame JSON（含 provenance/confidence）')
    parser.add_argument('--remote-intent', default='',
                        help='可选：VLM 解析的 IntentFrame JSON；本地事实优先合并')
    parser.add_argument('--via-arbiter', action='store_true',
                        help='速度经 /velocity_arbiter/vlm_cmd（栈开着仲裁时用）')
    parser.add_argument('--speed', type=float, default=None,
                        help='覆盖最大速度（0.1–1.0 m/s）')
    parser.add_argument('--timeout', type=float, default=None,
                        help='执行节点硬超时（秒，默认任务时限 + 60）')
    parser.add_argument('--report', default='', help='执行报告路径（默认 logs/tasks/）')
    parser.add_argument('--no-hold-on-target', action='store_true',
                        help='检测到目标时不悬停（只记录）')
    parser.add_argument('--no-summary', action='store_true',
                        help='不生成 Markdown 证据页')
    args = parser.parse_args(argv)

    text = ' '.join(args.text)
    try:
        frame = parse_intent(text)
        if args.remote_intent:
            with open(args.remote_intent, encoding='utf-8') as handle:
                remote = IntentFrame.from_dict(json.load(handle))
            frame = merge_intent(frame, remote)
    except (TaskError, ValueError, OSError, json.JSONDecodeError) as exc:
        print(f'解析失败：{exc}', file=sys.stderr)
        return 2

    spec = frame.task
    if args.speed is not None:
        speed = max(0.1, min(1.0, float(args.speed)))
        spec.max_speed_mps = speed

    if args.intent_json:
        print(json.dumps(frame.to_dict(), ensure_ascii=False))
        return 0
    _print_plan(spec, json_only=args.json)
    if args.json:
        return 0
    _print_intent(frame)

    stamp = (time.strftime('%Y%m%d-%H%M%S')
             + f'-{int(time.time() * 1000) % 1000:03d}')
    task_dir = os.path.join(_ROOT, 'logs', 'tasks')
    os.makedirs(task_dir, exist_ok=True)
    task_file = os.path.join(task_dir, f'task_{stamp}.json')
    with open(task_file, 'w', encoding='utf-8') as handle:
        handle.write(spec.to_json())
    intent_file = os.path.join(task_dir, f'intent_{stamp}.json')
    with open(intent_file, 'w', encoding='utf-8') as handle:
        json.dump(frame.to_dict(), handle, ensure_ascii=False, indent=2)
    print(f'任务文件：{task_file}')
    print(f'意图文件：{intent_file}')

    if not args.execute:
        print('\n（默认只解析，未起飞。确认计划后加 --execute 真正执行。）')
        return 0

    if os.environ.get('FLY_TASK_CONFIRM', '') != '1':
        print('\n⚠️ 即将真机/仿真飞行。确认安全后设置 FLY_TASK_CONFIRM=1 再执行，'
              '例如：\n    FLY_TASK_CONFIRM=1 scripts/fly_task.py "..." --execute')
        return 3

    report = args.report or os.path.join(task_dir, f'report_{stamp}.jsonl')
    cmd = [
        'ros2', 'run', 'mission', 'task_executor', '--ros-args',
        '-p', f'task_file:={task_file}',
        '-p', 'dry_run:=false',
        '-p', f'report_path:={report}',
        '-p', f'intent_path:={intent_file}',
    ]
    if args.via_arbiter:
        cmd += ['-p', 'velocity_topic:=/velocity_arbiter/vlm_cmd']
    if args.no_hold_on_target:
        cmd += ['-p', 'hold_on_target:=false']
    timeout = args.timeout or (spec.max_duration_s + 60.0)
    # 子进程不会自动带 ROS 环境：没找到 ros2 时用 `bash -c 'source env.sh && exec ...'`。
    if shutil.which('ros2') is None:
        env_script = os.path.join(_ROOT, 'scripts', 'env.sh')
        inner = ' '.join(shlex.quote(part) for part in cmd)
        cmd = ['bash', '-c',
               f'source {shlex.quote(env_script)} >/dev/null 2>&1 && exec {inner}']
    print(f'\n启动执行器（timeout {timeout:.0f} s）：\n  ' + ' '.join(cmd))
    try:
        completed = subprocess.run(cmd, timeout=timeout, check=False)
    except subprocess.TimeoutExpired:
        print(f'\n执行超时（{timeout:.0f} s）——请确认飞机已落地/上锁，'
              f'必要时 scripts/stop_stack.sh 收尾', file=sys.stderr)
        return 4
    except FileNotFoundError:
        print('\n找不到 ros2：请先 `source scripts/env.sh` 再运行本脚本', file=sys.stderr)
        return 5
    print(f'\n执行结束（退出码 {completed.returncode}）')
    print(f'报告：{report}')
    if not args.no_summary and os.path.isfile(report):
        try:
            summary_path = summarize(report, intent_file)
            print(f'证据页：{summary_path}')
        except OSError as exc:
            print(f'证据页生成失败：{exc}', file=sys.stderr)
    return completed.returncode


if __name__ == '__main__':
    raise SystemExit(main())
