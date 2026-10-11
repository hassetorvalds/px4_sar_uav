#!/usr/bin/env python3
"""远端 VLM 服务的本地桩（测试替身，用于在服务器就绪前验证客户端链路）。

实现与 agent_communicate_reply.md 中「接口定稿 v2」一致的 /health 与 /navigate，
返回 LightNav-0 风格的 pointing。仅用于本地联调，**不是真实推理**。

用法：python3 scripts/vlm_remote_stub.py [端口，默认 18080]
"""

import base64
import json
import sys
import time
from http.server import BaseHTTPRequestHandler, HTTPServer

# 依次返回的指向（模拟 apos_state 变化，便于验证状态处理）
SEQUENCE = [
    {'x_norm': 0.62, 'y_norm': 0.48, 'apos_state': 'rot_left',
     'opos_state': 'not_visible'},
    {'x_norm': 0.55, 'y_norm': 0.51, 'apos_state': 'rot_right',
     'opos_state': 'not_visible'},
    {'x_norm': 0.50, 'y_norm': 0.50, 'apos_state': 'stop',
     'opos_state': 'visible'},
]


class Handler(BaseHTTPRequestHandler):
    counter = 0

    def _json(self, payload: dict, status: int = 200) -> None:
        body = json.dumps(payload).encode('utf-8')
        self.send_response(status)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self) -> None:  # noqa: N802
        if self.path.startswith('/health'):
            self._json({'ready': True, 'model': 'LightNav-0-stub', 'rate_hz': 4.0})
        else:
            self._json({'error': 'not found'}, 404)

    def do_POST(self) -> None:  # noqa: N802
        if not self.path.startswith('/navigate'):
            self._json({'error': 'not found'}, 404)
            return
        length = int(self.headers.get('Content-Length', '0'))
        started = time.monotonic()
        request = json.loads(self.rfile.read(length).decode('utf-8'))
        time.sleep(0.15)      # 模拟模型推理耗时
        index = Handler.counter % len(SEQUENCE)
        Handler.counter += 1
        pointing = dict(SEQUENCE[index])
        pointing.update({'mode': 'grid', 'frame_size': [480, 270]})
        image_bytes = len(base64.b64decode(request.get('image_b64', '') or ''))
        self._json({
            'pointing': pointing,
            'waypoints': None,
            'latency_ms': int((time.monotonic() - started) * 1000),
            'seq': request.get('meta', {}).get('frame_seq', index),
            'echo': {'session': request.get('session_id'),
                     'instruction': request.get('instruction'),
                     'image_bytes': image_bytes},
        })

    def log_message(self, *args) -> None:
        pass


def main() -> int:
    port = int(sys.argv[1]) if len(sys.argv) > 1 else 18080
    print(f'[stub] 远端服务替身监听 127.0.0.1:{port}')
    HTTPServer(('127.0.0.1', port), Handler).serve_forever()
    return 0


if __name__ == '__main__':
    sys.exit(main())
