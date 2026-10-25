#!/usr/bin/env python3
"""Qwen3-VL-8B 视觉目标 → 相对 ENU 目标 → go_to TaskSpec。

不直接控制飞机：只生成经过几何/围栏校验的 TaskSpec，由 fly_task/Action 执行。
Qwen 只负责图像 grounding；深度/航向/坐标由本地 AirSim/几何负责。
"""

import argparse
import base64
import io
import json
import math
import os
import re
import time
import urllib.request

import airsim
import numpy as np
from PIL import Image

from perception.geometry import (CameraIntrinsics, depth_patch_median,
                                 pixel_to_world_enu)
from px4_interface.frames import ned_to_enu, ned_yaw_to_enu_yaw

EN_PROMPT = ('Locate the orange ball in the image. Return ONLY a JSON object '
             'with keys x, y (normalized [0,1000], top-left origin) and '
             'confidence (0-1). Do not use markdown.')
ZH_PROMPT = ('请定位图像中的橙色球，只输出 JSON：x、y 为 [0,1000] 归一化坐标'
             '（左上原点），以及 confidence（0-1）。不要 markdown。')


def orange_center(image):
    array = np.asarray(image)
    r, g, b = (array[:, :, i].astype(int) for i in range(3))
    mask = (r > 150) & (g > 70) & (g < 190) & (b < 100) & (r > g + 40)
    ys, xs = np.where(mask)
    if len(xs) == 0:
        return None
    return float(xs.mean() / image.width), float(ys.mean() / image.height)


def qwen_grounding(url, image_bytes, instruction, size, prompt=None):
    prompt = prompt or (ZH_PROMPT if any('\u4e00' <= ch <= '\u9fff' for ch in instruction) else EN_PROMPT)
    payload = json.dumps({
        'model': 'Qwen3-VL-8B-Instruct',
        'messages': [{'role': 'user', 'content': [
            {'type': 'text', 'text': prompt + '\n' + instruction},
            {'type': 'image_url', 'image_url': {
                'url': 'data:image/jpeg;base64,' +
                       base64.b64encode(image_bytes).decode('ascii')}},
        ]}],
        'temperature': 0.0, 'max_tokens': 128,
    }).encode('utf-8')
    request = urllib.request.Request(
        url, data=payload, method='POST',
        headers={'Content-Type': 'application/json'})
    started = time.monotonic()
    with urllib.request.urlopen(request, timeout=20) as response:
        body = response.read().decode('utf-8')
    latency = (time.monotonic() - started) * 1000.0
    content = json.loads(body)['choices'][0]['message']['content']
    content = re.sub(r'^```(?:json)?\s*|```$', '', content.strip(),
                     flags=re.IGNORECASE).strip()
    start, end = content.find('{'), content.rfind('}')
    if start < 0 or end <= start:
        raise ValueError(f'Qwen 没有返回 JSON：{content[:160]!r}')
    data = json.loads(content[start:end + 1])
    x, y = float(data['x']), float(data['y'])
    if not (0.0 <= x <= 1000.0 and 0.0 <= y <= 1000.0):
        raise ValueError(f'Qwen 坐标越界 [0,1000]：{x}, {y}')
    return x / 1000.0 * size[0], y / 1000.0 * size[1], data.get('confidence'), latency


BBOX_ZH_PROMPT = ('请定位图像中的“%s”，输出 JSON：bbox=[x1,y1,x2,y2]，'
                  '坐标归一化到 [0,1000]，以及 confidence。只输出 JSON。')


def qwen_bbox(url, image_bytes, instruction, size, prompt=None):
    prompt = prompt or ('请定位图像中的目标，输出 JSON：bbox=[x1,y1,x2,y2]，'
                        '坐标归一化到 [0,1000]，以及 confidence。只输出 JSON。')
    payload = json.dumps({
        'model': 'Qwen3-VL-8B-Instruct',
        'messages': [{'role': 'user', 'content': [
            {'type': 'text', 'text': prompt + '\n' + instruction},
            {'type': 'image_url', 'image_url': {
                'url': 'data:image/jpeg;base64,' +
                       base64.b64encode(image_bytes).decode('ascii')}},
        ]}],
        'temperature': 0.0, 'max_tokens': 160,
    }).encode('utf-8')
    request = urllib.request.Request(
        url, data=payload, method='POST',
        headers={'Content-Type': 'application/json'})
    started = time.monotonic()
    with urllib.request.urlopen(request, timeout=20) as response:
        body = response.read().decode('utf-8')
    latency = (time.monotonic() - started) * 1000.0
    content = json.loads(body)['choices'][0]['message']['content']
    content = re.sub(r'^```(?:json)?\s*|```$', '', content.strip(),
                     flags=re.IGNORECASE).strip()
    start, end = content.find('{'), content.rfind('}')
    if start < 0 or end <= start:
        raise ValueError(f'Qwen 没有返回 bbox JSON：{content[:160]!r}')
    data = json.loads(content[start:end + 1])
    raw = [float(v) for v in data['bbox']]
    if len(raw) != 4 or not all(0.0 <= v <= 1000.0 for v in raw):
        raise ValueError(f'bbox 越界/格式错误：{raw}')
    x1, y1, x2, y2 = raw
    if x2 <= x1 or y2 <= y1:
        raise ValueError(f'bbox 宽高无效：{raw}')
    width, height = size
    bbox = [x1 / 1000.0 * width, y1 / 1000.0 * height,
            x2 / 1000.0 * width, y2 / 1000.0 * height]
    return bbox, data.get('confidence'), latency


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--instruction', default='飞到前方橙色目标附近')
    parser.add_argument('--qwen-url', default='http://127.0.0.1:18081/v1/chat/completions')
    parser.add_argument('--airsim-ip', default='172.21.192.1')
    parser.add_argument('--airsim-port', type=int, default=45000)
    parser.add_argument('--hfov-deg', type=float, default=90.0)
    parser.add_argument('--geofence-m', type=float, default=50.0)
    parser.add_argument('--altitude-m', type=float, default=2.0)
    parser.add_argument('--out', default='')
    args = parser.parse_args()

    client = airsim.MultirotorClient(
        ip=args.airsim_ip, port=args.airsim_port, timeout_value=10)
    client.confirmConnection()
    state = client.getMultirotorState()
    pose = state.kinematics_estimated.position
    # 以相机实际姿态为准：teleport 后 vehicle state 的 orientation 可能滞后，
    # 而图像已经按新相机姿态渲染；用错航向会把目标 ENU 旋转到错误方向。
    camera_info = client.simGetCameraInfo('0', vehicle_name='Drone1')
    _, _, yaw_ned = airsim.to_eularian_angles(camera_info.pose.orientation)
    position_enu = ned_to_enu(pose.x_val, pose.y_val, pose.z_val)
    heading_enu = ned_yaw_to_enu_yaw(yaw_ned)

    responses = client.simGetImages([
        airsim.ImageRequest('0', airsim.ImageType.Scene, False, False),
        airsim.ImageRequest('0', airsim.ImageType.DepthPlanar, True, False),
    ], vehicle_name='Drone1')
    if len(responses) < 2 or not responses[0].image_data_uint8:
        raise SystemExit('AirSim 没有返回 scene/depth')
    scene = responses[0]
    depth_response = responses[1]
    # AirSim ImageRequest 返回原始 BGR uint8，不是 PNG
    raw = np.frombuffer(scene.image_data_uint8, dtype=np.uint8)
    height, width = int(scene.height), int(scene.width)
    if raw.size < height * width * 3:
        raise SystemExit('AirSim scene 数据不完整')
    image = Image.fromarray(
        raw[:height * width * 3].reshape(height, width, 3)[:, :, ::-1], 'RGB')
    image_bytes = io.BytesIO()
    image.save(image_bytes, format='JPEG', quality=85)
    depth_width = int(depth_response.width)
    depth_height = int(depth_response.height)
    depth = np.asarray(depth_response.image_data_float, dtype=float).reshape(
        depth_height, depth_width)

    u, v, confidence, latency = qwen_grounding(
        args.qwen_url, image_bytes.getvalue(), args.instruction, (width, height))
    expected = orange_center(image)
    proxy_error = (None if expected is None
                   else math.hypot(u / width - expected[0], v / height - expected[1]))
    if proxy_error is not None and proxy_error > 0.15:
        raise SystemExit(
            f'Qwen 指向与橙色代理目标 bbox 偏差 {proxy_error:.3f} > 0.15；拒绝生成目标')
    # 深度分辨率可能与彩色不同（本机 640×480 vs 1280×720）
    depth_m = depth_patch_median(
        depth, u * depth_width / width, v * depth_height / height, radius=2)
    if depth_m is None or not math.isfinite(depth_m) or depth_m <= 0.0:
        raise SystemExit(f'目标像素 ({u:.0f},{v:.0f}) 无有效深度；拒绝猜坐标')
    intrinsics = CameraIntrinsics.from_hfov(width, height, args.hfov_deg)
    world = pixel_to_world_enu(u, v, depth_m, intrinsics, heading_enu, position_enu)
    east = float(world[0] - position_enu[0])
    north = float(world[1] - position_enu[1])
    distance = math.hypot(east, north)
    if distance > args.geofence_m:
        raise SystemExit(f'目标距当前 {distance:.1f} m > 围栏 {args.geofence_m:.1f} m；拒绝')
    if distance < 0.5:
        raise SystemExit(f'目标过近/重合：{distance:.2f} m；拒绝')

    stamp = time.strftime('%Y%m%d-%H%M%S')
    out = args.out or f'logs/tasks/qwen_go_to_{stamp}.json'
    os.makedirs(os.path.dirname(out), exist_ok=True)
    task = {
        'version': 1,
        'source_text': f'Qwen grounding: {args.instruction}',
        'altitude_m': args.altitude_m,
        'altitude_frame': 'REL',
        'max_speed_mps': 0.5,
        'max_duration_s': 180.0,
        'geofence_radius_m': args.geofence_m,
        'coordinate_frame': 'relative',
        'intent_source': 'qwen_grounding',
        'intent_confidence': confidence,
        'steps': [
            {'type': 'takeoff', 'params': {'altitude_m': args.altitude_m}},
            {'type': 'goto', 'params': {'east_m': round(east, 2),
                                        'north_m': round(north, 2),
                                        'altitude_m': args.altitude_m}},
            {'type': 'hover', 'params': {'seconds': 2.0}},
            {'type': 'land', 'params': {}},
        ],
        'notes': [f'qwen_latency_ms={latency:.1f}',
                  f'pixel=({u:.1f},{v:.1f})', f'depth_m={depth_m:.2f}',
                  f'proxy_error_norm={proxy_error}',
                  f'heading_enu_deg={math.degrees(heading_enu):.1f}'],
    }
    with open(out, 'w', encoding='utf-8') as handle:
        json.dump(task, handle, ensure_ascii=False, indent=2)
    print(json.dumps({
        'instruction': args.instruction, 'pixel': [round(u, 1), round(v, 1)],
        'depth_m': round(depth_m, 2), 'target_enu_rel': [round(east, 2), round(north, 2)],
        'distance_m': round(distance, 2), 'confidence': confidence,
        'proxy_error_norm': None if proxy_error is None else round(proxy_error, 3),
        'qwen_latency_ms': round(latency, 1), 'task': out,
    }, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    raise SystemExit(main())
