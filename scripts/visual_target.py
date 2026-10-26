#!/usr/bin/env python3
"""通用 VLM 视觉导航目标：图像 grounding + 深度 + standoff → TaskSpec。

不读取 AirSim 物体坐标；只用相机图像/深度、无人机位姿/航向。
支持 backend=dino（英文 prompt）或 qwen（中文/英文 prompt）。
"""

import argparse
import base64
import io
import json
import math
import os
import time
import urllib.request

import airsim
import numpy as np
from PIL import Image

from mission.visual_target import apply_standoff
from perception.geometry import (CameraIntrinsics, depth_patch_median,
                                 pixel_to_world_enu)
from px4_interface.frames import ned_to_enu, ned_yaw_to_enu_yaw
from qwen_target import qwen_bbox, qwen_grounding


def dino_detect(url, jpeg, prompt):
    payload = json.dumps({
        'image_b64': base64.b64encode(jpeg).decode('ascii'),
        'prompt': prompt, 'box_threshold': 0.4, 'text_threshold': 0.2,
    }).encode('utf-8')
    request = urllib.request.Request(
        url, data=payload, method='POST',
        headers={'Content-Type': 'application/json'})
    started = time.monotonic()
    with urllib.request.urlopen(request, timeout=20) as response:
        data = json.loads(response.read().decode('utf-8'))
    boxes = data.get('boxes') or []
    if not boxes:
        raise ValueError('DINO no boxes')
    best = max(boxes, key=lambda box: float(box.get('score', 0.0)))
    bbox = [float(best['x1']), float(best['y1']),
            float(best['x2']), float(best['y2'])]
    return bbox, float(best.get('score', 0.0)), (time.monotonic() - started) * 1000.0


def qwen_detect(url, jpeg, prompt, size):
    bbox_prompt = ('请定位图像中的“%s”，只输出 JSON：bbox=[x1,y1,x2,y2]，'
                   '坐标归一化到 [0,1000]，以及 confidence。不要 markdown。' % prompt)
    try:
        bbox, confidence, latency = qwen_bbox(
            url, jpeg, prompt, size, prompt=bbox_prompt)
        return bbox, confidence, latency
    except (KeyError, ValueError):
        u, v, confidence, latency = qwen_grounding(
            url, jpeg, prompt, size)
        return [u - 1, v - 1, u + 1, v + 1], confidence, latency


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
    parser.add_argument('--out', default='')
    args = parser.parse_args()

    client = airsim.MultirotorClient(ip='172.21.192.1', port=45000, timeout_value=10)
    client.confirmConnection()
    state = client.getMultirotorState()
    pos = state.kinematics_estimated.position
    camera = client.simGetCameraInfo('0', vehicle_name='Drone1')
    _, _, yaw_ned = airsim.to_eularian_angles(camera.pose.orientation)
    position_enu = ned_to_enu(pos.x_val, pos.y_val, pos.z_val)
    heading_enu = ned_yaw_to_enu_yaw(yaw_ned)
    responses = client.simGetImages([
        airsim.ImageRequest('0', airsim.ImageType.Scene, False, False),
        airsim.ImageRequest('0', airsim.ImageType.DepthPlanar, True, False),
    ], vehicle_name='Drone1')
    scene, depth_response = responses
    width, height = int(scene.width), int(scene.height)
    raw = np.frombuffer(scene.image_data_uint8, dtype=np.uint8)
    image = Image.fromarray(
        raw[:width * height * 3].reshape(height, width, 3)[:, :, ::-1], 'RGB')
    jpeg_buf = io.BytesIO()
    image.save(jpeg_buf, format='JPEG', quality=85)
    jpeg = jpeg_buf.getvalue()

    if args.backend == 'dino':
        bbox, confidence, latency = dino_detect(args.dino_url, jpeg, args.target_prompt)
    else:
        bbox, confidence, latency = qwen_detect(
            args.qwen_url, jpeg, args.target_prompt, (width, height))
    u = (bbox[0] + bbox[2]) / 2.0
    v = (bbox[1] + bbox[3]) / 2.0
    dw, dh = int(depth_response.width), int(depth_response.height)
    depth = np.asarray(depth_response.image_data_float, dtype=float).reshape(dh, dw)
    depth_m = depth_patch_median(depth, u * dw / width, v * dh / height, radius=3)
    if depth_m is None or not math.isfinite(depth_m) or depth_m <= 0.0:
        raise SystemExit('no valid depth at target')
    world = pixel_to_world_enu(
        u, v, depth_m, CameraIntrinsics.from_hfov(width, height, 90.0),
        heading_enu, position_enu)
    east = float(world[0] - position_enu[0])
    north = float(world[1] - position_enu[1])
    original_distance = math.hypot(east, north)
    if original_distance > args.geofence_m:
        raise SystemExit(f'object {original_distance:.1f} m > geofence {args.geofence_m:.1f} m')
    try:
        target_east, target_north, _ = apply_standoff(
            east, north, args.standoff_m)
    except ValueError as exc:
        raise SystemExit(str(exc)) from exc
    stamp = time.strftime('%Y%m%d-%H%M%S')
    out = args.out or f'logs/tasks/visual_target_{stamp}.json'
    os.makedirs(os.path.dirname(out), exist_ok=True)
    task = {
        'version': 1,
        'source_text': f'visual target: {args.target_prompt} (standoff {args.standoff_m:g} m)',
        'altitude_m': args.altitude_m, 'altitude_frame': 'REL',
        'max_speed_mps': 0.5, 'max_duration_s': 180.0,
        'geofence_radius_m': args.geofence_m,
        'coordinate_frame': 'relative',
        'intent_source': 'visual_target',
        'steps': [
            {'type': 'takeoff', 'params': {'altitude_m': args.altitude_m}},
            {'type': 'goto', 'params': {'east_m': round(target_east, 2),
                                        'north_m': round(target_north, 2),
                                        'altitude_m': args.altitude_m}},
            {'type': 'hover', 'params': {'seconds': 2.0}},
            {'type': 'land', 'params': {}},
        ],
        'notes': [
            f'backend={args.backend}', f'target_prompt={args.target_prompt}',
            f'bbox_px={[round(v, 1) for v in bbox]}',
            f'confidence={confidence}', f'depth_m={depth_m:.2f}',
            f'object_distance_m={original_distance:.2f}',
            f'standoff_m={args.standoff_m:g}',
            f'object_rel_enu={east:.2f},{north:.2f}',
            f'target_rel_enu={target_east:.2f},{target_north:.2f}',
            f'latency_ms={latency:.0f}',
        ],
    }
    with open(out, 'w', encoding='utf-8') as handle:
        json.dump(task, handle, ensure_ascii=False, indent=2)
    print(json.dumps({
        'backend': args.backend, 'target_prompt': args.target_prompt,
        'bbox': [round(v, 1) for v in bbox], 'depth_m': round(depth_m, 2),
        'object_rel_enu': [round(east, 2), round(north, 2)],
        'target_rel_enu': [round(target_east, 2), round(target_north, 2)],
        'standoff_m': args.standoff_m, 'task': out,
    }, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    raise SystemExit(main())
