#!/usr/bin/env python3
"""离线 grounding 评测：LightNav /navigate ↔ 橙色目标 bbox 中心。

不飞、不依赖 ROS；用 manifest 固定帧/指令，输出 JSON+CSV。
Qwen/Molmo 的 provider 入口留待服务器部署后接入，本轮只跑 LightNav baseline。
"""

import argparse
import base64
import csv
import io
import json
import math
import os
import time
import urllib.error
import urllib.request

import numpy as np
from PIL import Image


def orange_center(path):
    image = Image.open(path).convert('RGB')
    width, height = image.size
    array = np.asarray(image)
    r, g, b = (array[:, :, i].astype(int) for i in range(3))
    mask = (r > 150) & (g > 70) & (g < 190) & (b < 100) & (r > g + 40)
    ys, xs = np.where(mask)
    if len(xs) == 0:
        return None
    return [float(xs.mean() / width), float(ys.mean() / height)]


def jpeg_bytes(path):
    image = Image.open(path).convert('RGB')
    buffer = io.BytesIO()
    image.save(buffer, format='JPEG', quality=85)
    return buffer.getvalue()


def post_navigate(url, session, instruction, jpeg, frame_seq, timeout=15.0):
    payload = json.dumps({
        'session_id': session,
        'instruction': instruction,
        'image_b64': base64.b64encode(jpeg).decode('ascii'),
        'meta': {'frame_seq': frame_seq},
    }).encode('utf-8')
    request = urllib.request.Request(
        url, data=payload, method='POST',
        headers={'Content-Type': 'application/json'})
    started = time.monotonic()
    with urllib.request.urlopen(request, timeout=timeout) as response:
        body = response.read().decode('utf-8')
    return json.loads(body), (time.monotonic() - started) * 1000.0


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--manifest', required=True)
    parser.add_argument('--url', default='http://127.0.0.1:18080/navigate')
    parser.add_argument('--provider', default='lightnav', choices=['lightnav'])
    parser.add_argument('--repeat', type=int, default=3)
    parser.add_argument('--session', default='grounding-baseline')
    parser.add_argument('--out', default='')
    args = parser.parse_args()

    with open(args.manifest, encoding='utf-8') as handle:
        entries = json.load(handle)
    stamp = time.strftime('%Y%m%d-%H%M%S')
    out = args.out or f'logs/grounding/{args.provider}_{stamp}.json'
    os.makedirs(os.path.dirname(out), exist_ok=True)

    rows = []
    for entry in entries:
        image_path = entry['image']
        instruction = entry['instruction']
        expected = orange_center(image_path)
        jpeg = jpeg_bytes(image_path)
        calls = []
        for index in range(args.repeat):
            try:
                data, latency = post_navigate(
                    args.url, f'{args.session}-{entry["id"]}-{stamp}',
                    instruction, jpeg, index)
            except (urllib.error.URLError, TimeoutError, OSError) as exc:
                calls.append({'index': index, 'error': f'{type(exc).__name__}: {exc}'})
                continue
            pointing = data.get('pointing') or {}
            x = pointing.get('x_norm')
            y = pointing.get('y_norm')
            error = (None if x is None or y is None or expected is None
                     else math.hypot(float(x) - expected[0], float(y) - expected[1]))
            calls.append({
                'index': index, 'latency_ms': round(latency, 1),
                'x_norm': x, 'y_norm': y,
                'apos_state': pointing.get('apos_state'),
                'opos_state': pointing.get('opos_state'),
                'error_norm': None if error is None else round(error, 4),
                'waypoints': data.get('waypoints') or [],
            })
        valid = [c for c in calls if c.get('latency_ms') is not None]
        point_calls = [c for c in valid if c.get('error_norm') is not None]
        rows.append({
            'id': entry['id'], 'image': image_path,
            'instruction': instruction, 'expected_center': expected,
            'calls': calls,
            'steady_latency_ms': (round(sum(c['latency_ms'] for c in valid[1:]) /
                                        len(valid[1:]), 1) if len(valid) > 1 else None),
            'point_hits': len(point_calls),
            'mean_point_error': (round(sum(c['error_norm'] for c in point_calls) /
                                        len(point_calls), 4) if point_calls else None),
        })
        print(f'{entry["id"]}: expected={expected} '
              f'latency={[c.get("latency_ms") for c in calls]} '
              f'errors={[c.get("error_norm") for c in calls]} '
              f'actions={[c.get("apos_state") for c in calls]}')

    summary = {
        'provider': args.provider, 'url': args.url,
        'repeat': args.repeat, 'rows': rows,
    }
    with open(out, 'w', encoding='utf-8') as handle:
        json.dump(summary, handle, ensure_ascii=False, indent=2)
    csv_path = out.replace('.json', '.csv')
    with open(csv_path, 'w', newline='', encoding='utf-8') as handle:
        writer = csv.writer(handle)
        writer.writerow(['id', 'instruction', 'call', 'latency_ms',
                         'x_norm', 'y_norm', 'error_norm', 'apos_state', 'opos_state'])
        for row in rows:
            for call in row['calls']:
                writer.writerow([row['id'], row['instruction'], call.get('index'),
                                 call.get('latency_ms'), call.get('x_norm'),
                                 call.get('y_norm'), call.get('error_norm'),
                                 call.get('apos_state'), call.get('opos_state')])
    print('json:', out)
    print('csv :', csv_path)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
