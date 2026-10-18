#!/usr/bin/env python3
"""离线评估检测器：对目录里的图像跑一遍，报检出率/耗时（不依赖 ROS）。

用途：换检测器/调阈值前，先用已存帧做回归，避免"上机才发现漏检/误报"。

用法：
    source scripts/env.sh
    PYTHONNOUSERSITE=1 python3 scripts/eval_detector.py \
        --frames 'archive/vlm_decisions/**/*.jpg' --detector hog --limit 100
    python3 scripts/eval_detector.py --frames '...' --detector color \
        --hsv-lower 15,0.5,0.4 --hsv-upper 40,1,1
"""

import argparse
import glob
import json
import os
import statistics
import sys
import time

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..',
                                'ros2_px4_ws', 'src', 'perception'))

from perception.detectors import make_detector  # noqa: E402


def _read_bgr(path):
    try:
        import cv2
    except ImportError:
        return None
    return cv2.imread(path)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--frames', required=True, help='图像 glob（可递归 **）')
    parser.add_argument('--detector', default='color', choices=['color', 'hog', 'haar'])
    parser.add_argument('--hsv-lower', default='15,0.5,0.4')
    parser.add_argument('--hsv-upper', default='40,1,1')
    parser.add_argument('--min-area', type=float, default=200.0)
    parser.add_argument('--limit', type=int, default=0, help='最多评估多少帧（0=全部）')
    parser.add_argument('--min-score', type=float, default=0.0)
    parser.add_argument('--json-out', help='把逐帧结果写成 JSON')
    parser.add_argument('--quiet', action='store_true', help='只打印汇总')
    args = parser.parse_args(argv)

    paths = sorted(glob.glob(args.frames, recursive=True))
    if args.limit > 0:
        paths = paths[:args.limit]
    if not paths:
        print(f'没有匹配到图像：{args.frames}', file=sys.stderr)
        return 2

    lower = [float(v) for v in args.hsv_lower.split(',')]
    upper = [float(v) for v in args.hsv_upper.split(',')]
    detector = make_detector(
        args.detector, lower=lower, upper=upper,
        min_area=args.min_area, label='target')

    latencies = []
    hit_frames = 0
    total_dets = 0
    rows = []
    for path in paths:
        image = _read_bgr(path)
        if image is None:
            print('读取失败（需要 cv2）：', path, file=sys.stderr)
            return 3
        started = time.perf_counter()
        detections = [d for d in detector.detect(image) if d.score >= args.min_score]
        elapsed_ms = (time.perf_counter() - started) * 1000.0
        latencies.append(elapsed_ms)
        if detections:
            hit_frames += 1
            total_dets += len(detections)
        rows.append({
            'file': os.path.relpath(path),
            'ms': round(elapsed_ms, 2),
            'detections': [d.to_dict() for d in detections],
        })
        if detections and not args.quiet:
            best = detections[0]
            print(f'{os.path.basename(path):40s} {len(detections):2d} 个  '
                  f'best={best.label} score={best.score:.2f} '
                  f'bbox=({best.x1:.0f},{best.y1:.0f},{best.x2:.0f},{best.y2:.0f}) '
                  f'{elapsed_ms:.1f} ms')

    print('\n== 汇总 ==')
    print(f'检测器      {args.detector}')
    print(f'帧数        {len(paths)}')
    print(f'有检出帧    {hit_frames}（{100.0 * hit_frames / len(paths):.1f}%）')
    print(f'检出总数    {total_dets}')
    print(f'单帧耗时    mean={statistics.fmean(latencies):.1f} ms  '
          f'p95={sorted(latencies)[max(int(0.95 * len(latencies)) - 1, 0)]:.1f} ms  '
          f'max={max(latencies):.1f} ms')
    if args.json_out:
        with open(args.json_out, 'w', encoding='utf-8') as handle:
            json.dump({'summary': {'frames': len(paths), 'hit_frames': hit_frames,
                                   'total_detections': total_dets,
                                   'mean_ms': round(statistics.fmean(latencies), 2)},
                       'rows': rows}, handle, ensure_ascii=False, indent=2)
        print(f'逐帧结果    {args.json_out}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
