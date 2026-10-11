#!/usr/bin/env python3
"""远端 VLM 服务连通性检查（Step A 验收用，不涉及飞行）。

做三件事：
  1) GET /health 看服务是否就绪；
  2) 用一张本地图片 + 英文指令 POST /navigate；
  3) 打印原始响应、解析出的指向（含 state）与端到端往返耗时。

用法（先建隧道：ssh -N -L 18080:127.0.0.1:18080 <user>@10.255.248.205）：
    source scripts/env.sh
    python3 scripts/vlm_remote_check.py --ros-args -p instruction:="go to the collapsed building ahead"
"""

import base64
import json
import sys
import time
import urllib.error
import urllib.request

import rclpy
from rclpy.node import Node
from vlm.pointing import PointingParseError, parse_pointing
from vlm.vlm_client import image_file_to_jpeg

DEFAULT_IMAGE = 'ros2_px4_ws/src/vlm/test/data/test_scene.jpg'


class RemoteCheck(Node):
    def __init__(self) -> None:
        super().__init__('vlm_remote_check')
        self.declare_parameter('health_url', 'http://127.0.0.1:18080/health')
        self.declare_parameter('navigate_url', 'http://127.0.0.1:18080/navigate')
        self.declare_parameter('instruction', 'go to the collapsed building ahead')
        self.declare_parameter('image_path', '')
        self.declare_parameter('session_id', 'drone1')
        self.declare_parameter('repeat', 5)

    def run(self) -> int:
        health_url = str(self.get_parameter('health_url').value)
        try:
            with urllib.request.urlopen(health_url, timeout=5) as response:
                print(f'[health] {response.status} {response.read().decode()[:200]}')
        except Exception as exc:  # noqa: BLE001
            print(f'[health] 失败：{type(exc).__name__} {exc}')
            print('提示：先建立隧道 ssh -N -L 18080:127.0.0.1:18080 <user>@10.255.248.205')
            return 1

        image_path = str(self.get_parameter('image_path').value) or DEFAULT_IMAGE
        try:
            jpeg = image_file_to_jpeg(image_path)
        except Exception as exc:  # noqa: BLE001
            print(f'读取图片失败 {image_path}：{exc}')
            return 1
        instruction = str(self.get_parameter('instruction').value)
        url = str(self.get_parameter('navigate_url').value)
        session = str(self.get_parameter('session_id').value)
        print(f'[navigate] 图片 {image_path}（{len(jpeg)/1024:.0f} KB），指令「{instruction}」')

        latencies = []
        for index in range(int(self.get_parameter('repeat').value)):
            payload = json.dumps({
                'session_id': session,
                'instruction': instruction,
                'image_b64': base64.b64encode(jpeg).decode('ascii'),
                'meta': {'frame_seq': index},
            }).encode('utf-8')
            request = urllib.request.Request(
                url, data=payload, method='POST',
                headers={'Content-Type': 'application/json'})
            started = time.monotonic()
            try:
                with urllib.request.urlopen(request, timeout=15) as response:
                    text = response.read().decode('utf-8')
            except urllib.error.HTTPError as exc:
                print(f'  第 {index} 次 HTTP {exc.code}: '
                      f'{exc.read().decode("utf-8", errors="replace")[:200]}')
                return 1
            except Exception as exc:  # noqa: BLE001
                print(f'  第 {index} 次失败：{type(exc).__name__} {exc}')
                return 1
            elapsed_ms = (time.monotonic() - started) * 1000.0
            latencies.append(elapsed_ms)
            try:
                pointing = parse_pointing(text)
                summary = (f'x_norm={pointing.x_norm:.3f} y_norm={pointing.y_norm:.3f} '
                           f'apos={pointing.apos_state or "-"} opos={pointing.opos_state or "-"}')
            except PointingParseError as exc:
                summary = f'解析失败：{exc}'
            print(f'  第 {index} 次：往返 {elapsed_ms:.0f} ms | {summary}')
            if index == 0:
                print(f'  原始响应：{text[:300]}')

        if latencies:
            print(f'\n往返耗时：最小 {min(latencies):.0f} ms，'
                  f'平均 {sum(latencies)/len(latencies):.0f} ms，'
                  f'最大 {max(latencies):.0f} ms（{len(latencies)} 次）')
        return 0


def main() -> int:
    rclpy.init()
    node = RemoteCheck()
    try:
        return node.run()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
