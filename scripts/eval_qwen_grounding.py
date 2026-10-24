#!/usr/bin/env python3
"""Qwen3-VL-8B 离线 grounding 评测（OpenAI-compatible，归一化 [0,1000]）。"""

import argparse
import base64
import csv
import io
import json
import math
import os
import re
import time
import urllib.error
import urllib.request

import numpy as np
from PIL import Image

EN_PROMPT = ('Locate the orange ball in the image. Return ONLY a JSON object '
             'with keys x, y (normalized [0,1000], top-left origin) and '
             'confidence (0-1). Do not use markdown.')
ZH_PROMPT = ('请定位图像中的橙色球，只输出 JSON：x、y 为 [0,1000] 归一化坐标'
             '（左上原点），以及 confidence（0-1）。不要 markdown。')


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


def parse_json_content(text):
    text = (text or '').strip()
    text = re.sub(r'^```(?:json)?\s*|```$', '', text, flags=re.IGNORECASE).strip()
    start, end = text.find('{'), text.rfind('}')
    if start < 0 or end <= start:
        raise ValueError(f'no JSON object: {text[:120]!r}')
    data = json.loads(text[start:end + 1])
    x, y = float(data['x']), float(data['y'])
    if not (0.0 <= x <= 1000.0 and 0.0 <= y <= 1000.0):
        raise ValueError(f'coordinates out of [0,1000]: {x}, {y}')
    return x / 1000.0, y / 1000.0, data.get('confidence')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--manifest', required=True)
    parser.add_argument('--url', default='http://127.0.0.1:18081/v1/chat/completions')
    parser.add_argument('--repeat', type=int, default=3)
    parser.add_argument('--out', default='')
    args = parser.parse_args()

    entries = json.load(open(args.manifest, encoding='utf-8'))
    stamp = time.strftime('%Y%m%d-%H%M%S')
    out = args.out or f'logs/grounding/qwen_{stamp}.json'
    os.makedirs(os.path.dirname(out), exist_ok=True)
    rows = []
    for entry in entries:
        lang = entry.get('lang', 'en')
        prompt = ZH_PROMPT if lang == 'zh' else EN_PROMPT
        expected = orange_center(entry['image'])
        jpeg = jpeg_bytes(entry['image'])
        calls = []
        for index in range(args.repeat):
            payload = json.dumps({
                'model': 'Qwen3-VL-8B-Instruct',
                'messages': [{'role': 'user', 'content': [
                    {'type': 'text', 'text': prompt},
                    {'type': 'image_url', 'image_url': {
                        'url': 'data:image/jpeg;base64,' +
                               base64.b64encode(jpeg).decode('ascii')}},
                ]}],
                'temperature': 0.0,
                'max_tokens': 128,
            }).encode('utf-8')
            request = urllib.request.Request(
                args.url, data=payload, method='POST',
                headers={'Content-Type': 'application/json'})
            started = time.monotonic()
            try:
                with urllib.request.urlopen(request, timeout=20) as response:
                    body = response.read().decode('utf-8')
                latency = (time.monotonic() - started) * 1000.0
                data = json.loads(body)
                content = data['choices'][0]['message']['content']
                x, y, confidence = parse_json_content(content)
                error = (None if expected is None
                         else math.hypot(x - expected[0], y - expected[1]))
                calls.append({
                    'index': index, 'latency_ms': round(latency, 1),
                    'x_norm': round(x, 4), 'y_norm': round(y, 4),
                    'confidence': confidence,
                    'error_norm': None if error is None else round(error, 4),
                    'raw': content,
                })
            except Exception as exc:  # noqa: BLE001
                calls.append({'index': index,
                              'error': f'{type(exc).__name__}: {exc}'})
        valid = [c for c in calls if c.get('latency_ms') is not None]
        point = [c for c in valid if c.get('error_norm') is not None]
        rows.append({
            'id': entry['id'], 'lang': lang, 'instruction': entry['instruction'],
            'expected_center': expected, 'calls': calls,
            'parse_ok': len(point), 'point_hits': len(point),
            'mean_error': (round(sum(c['error_norm'] for c in point) / len(point), 4)
                           if point else None),
        })
        print(entry['id'], lang, 'expected', expected,
              'latency', [c.get('latency_ms') for c in calls],
              'err', [c.get('error_norm') for c in calls])
    json.dump({'provider': 'qwen', 'url': args.url, 'rows': rows},
              open(out, 'w', encoding='utf-8'), ensure_ascii=False, indent=2)
    csv_path = out.replace('.json', '.csv')
    with open(csv_path, 'w', newline='', encoding='utf-8') as handle:
        writer = csv.writer(handle)
        writer.writerow(['id', 'lang', 'call', 'latency_ms', 'x_norm', 'y_norm',
                         'confidence', 'error_norm', 'raw'])
        for row in rows:
            for call in row['calls']:
                writer.writerow([row['id'], row['lang'], call.get('index'),
                                 call.get('latency_ms'), call.get('x_norm'),
                                 call.get('y_norm'), call.get('confidence'),
                                 call.get('error_norm'), call.get('raw')])
    print('json:', out)
    print('csv :', csv_path)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
