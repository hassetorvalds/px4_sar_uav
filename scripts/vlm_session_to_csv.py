#!/usr/bin/env python3
"""把 `vlm_decisions_*/decision_*.json` 导出成一份逐帧 CSV（便于与服务器侧日志对账）。

用法：
    python3 scripts/vlm_session_to_csv.py vlm_decisions_sweep logs/2026-10-12/drone1_session.csv

为什么需要它：VLM 节点**不会在落地时停止调用远端**——飞行结束后它仍在记录，
所以服务端看到的请求数会比"扫场窗口"多（2026-10-12 就出现过 746 vs 244 的口径差）。
这个脚本把整个会话（含地面段）导出，两个口径就能对上。
"""

import csv
import glob
import json
import math
import os
import sys

FIELDS = [
    'timestamp', 'frame_seq', 'command_source', 'instruction', 'apos_state', 'opos_state',
    'point_x_norm', 'point_y_norm', 'depth_m', 'latency_ms',
    'waypoint_count', 'wp0_forward_m', 'wp0_lateral_m', 'wp0_yaw_deg',
    'cmd_forward', 'cmd_left', 'cmd_up', 'cmd_yaw_rate',
]


def main(argv):
    source = argv[1] if len(argv) > 1 else 'vlm_decisions'
    target = argv[2] if len(argv) > 2 else 'logs/vlm_session.csv'
    paths = sorted(glob.glob(os.path.join(source, 'decision_*.json')))
    if not paths:
        print(f'{source} 下没有 decision_*.json')
        return 1

    rows = []
    for path in paths:
        with open(path, encoding='utf-8') as handle:
            record = json.load(handle)
        waypoints = record.get('waypoints') or []
        first = waypoints[0] if waypoints else {}
        velocity = record.get('body_velocity') or {}
        yaw_deg = (math.degrees(first['yaw_rad'])
                   if isinstance(first.get('yaw_rad'), (int, float)) else None)
        rows.append({
            'timestamp': record.get('timestamp'),
            'frame_seq': record.get('frame_seq'),
            'command_source': record.get('command_source'),
            'instruction': record.get('instruction'),
            'apos_state': record.get('apos_state'),
            'opos_state': record.get('opos_state'),
            'point_x_norm': record.get('point_x_norm'),
            'point_y_norm': record.get('point_y_norm'),
            'depth_m': record.get('depth_m'),
            'latency_ms': round(record['latency_ms'], 1)
            if isinstance(record.get('latency_ms'), (int, float)) else None,
            'waypoint_count': len(waypoints),
            'wp0_forward_m': first.get('forward_m'),
            'wp0_lateral_m': first.get('lateral_m'),
            'wp0_yaw_deg': yaw_deg,
            'cmd_forward': velocity.get('forward'),
            'cmd_left': velocity.get('left'),
            'cmd_up': velocity.get('up'),
            'cmd_yaw_rate': velocity.get('yaw_rate'),
        })

    rows.sort(key=lambda r: r['frame_seq'] if r['frame_seq'] is not None else 0)
    directory = os.path.dirname(target)
    if directory:
        os.makedirs(directory, exist_ok=True)
    with open(target, 'w', newline='', encoding='utf-8') as handle:
        writer = csv.DictWriter(handle, fieldnames=FIELDS)
        writer.writeheader()
        writer.writerows(rows)
    seqs = [r['frame_seq'] for r in rows if r['frame_seq'] is not None]
    print(f'{len(rows)} 帧 → {target}')
    if seqs:
        gaps = [b - a for a, b in zip(sorted(seqs), sorted(seqs)[1:]) if b - a != 1]
        print(f'frame_seq {min(seqs)} → {max(seqs)}；非 +1 处 {len(gaps)} 个'
              f'（文件名毫秒字段有墙钟微调时会出现，按 seq 排序后应为 0）')
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))
