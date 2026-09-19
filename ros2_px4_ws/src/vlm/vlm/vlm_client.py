"""VLM 客户端：mock / gemini / openai。

只用标准库（urllib）发起 HTTPS 请求，避免引入重型 SDK，也避开系统里
``cv2`` 与 numpy 2.x 的 ABI 冲突（图像编码统一走 PIL）。
密钥一律从环境变量读取，不写入仓库。
"""

import base64
import io
import json
import os
import urllib.error
import urllib.request

GEMINI_ENDPOINT = 'https://generativelanguage.googleapis.com/v1beta/models'
DEFAULT_OPENAI_BASE_URL = 'https://openrouter.ai/api/v1'


class VlmError(RuntimeError):
    """VLM 调用失败（网络、鉴权、返回格式等）。"""


def image_message_to_jpeg(image_msg) -> bytes:
    """``sensor_msgs/Image`` → JPEG 字节流（依赖 PIL，按需导入）。"""
    from PIL import Image

    encoding = getattr(image_msg, 'encoding', 'rgb8')
    height = int(image_msg.height)
    width = int(image_msg.width)
    step = int(getattr(image_msg, 'step', width * 3))
    raw = bytes(image_msg.data)

    channels = {'rgb8': 3, 'bgr8': 3, 'rgba8': 4, 'bgra8': 4, 'mono8': 1}.get(encoding)
    if channels is None:
        raise VlmError(f'暂不支持的图像编码：{encoding}')

    packed = bytearray()
    for row in range(height):
        start = row * step
        packed.extend(raw[start:start + width * channels])

    if channels == 1:
        image = Image.frombytes('L', (width, height), bytes(packed)).convert('RGB')
    elif channels == 4:
        image = Image.frombytes('RGBA', (width, height), bytes(packed)).convert('RGB')
    else:
        image = Image.frombytes('RGB', (width, height), bytes(packed))
        if encoding == 'bgr8':
            # PIL 不支持 BGR 模式，按 RGB 读入后交换红蓝通道
            red, green, blue = image.split()
            image = Image.merge('RGB', (blue, green, red))

    buffer = io.BytesIO()
    image.save(buffer, format='JPEG', quality=85)
    return buffer.getvalue()


def image_file_to_jpeg(path: str) -> bytes:
    """本地图片文件 → JPEG 字节流（离线测试用）。"""
    from PIL import Image

    with Image.open(path) as image:
        buffer = io.BytesIO()
        image.convert('RGB').save(buffer, format='JPEG', quality=85)
        return buffer.getvalue()


class BaseVlmClient:
    def complete(self, prompt: str, jpeg_bytes: bytes) -> str:
        raise NotImplementedError


class MockVlmClient(BaseVlmClient):
    """离线桩：返回固定指向结果，用于在没有 API key 的情况下验证整条链路。"""

    def __init__(self, response_text: str = ''):
        self.response_text = response_text or (
            '[{"point": [500, 620], "depth": 5, "label": "mock target"}]')
        self.calls = 0

    def complete(self, prompt: str, jpeg_bytes: bytes) -> str:
        self.calls += 1
        return self.response_text


class GeminiVlmClient(BaseVlmClient):
    def __init__(self, model: str = 'gemini-2.5-flash', api_key_env: str = 'GEMINI_API_KEY',
                 timeout_s: float = 30.0):
        self.model = model
        self.api_key_env = api_key_env
        self.timeout_s = timeout_s

    def complete(self, prompt: str, jpeg_bytes: bytes) -> str:
        api_key = os.getenv(self.api_key_env)
        if not api_key:
            raise VlmError(f'环境变量 {self.api_key_env} 未设置')
        url = f'{GEMINI_ENDPOINT}/{self.model}:generateContent?key={api_key}'
        payload = {
            'contents': [{
                'parts': [
                    {'text': prompt},
                    {'inline_data': {
                        'mime_type': 'image/jpeg',
                        'data': base64.b64encode(jpeg_bytes).decode('ascii'),
                    }},
                ],
            }],
            'generationConfig': {'temperature': 0.4, 'topP': 0.95, 'maxOutputTokens': 1024},
        }
        response = _post_json(url, payload, self.timeout_s, {})
        try:
            return response['candidates'][0]['content']['parts'][0]['text']
        except (KeyError, IndexError) as exc:
            raise VlmError(f'Gemini 返回格式不符合预期：{str(response)[:200]}') from exc


class OpenAiVlmClient(BaseVlmClient):
    def __init__(self, model: str = 'google/gemini-2.5-flash',
                 api_key_env: str = 'OPENAI_API_KEY',
                 base_url: str = '', timeout_s: float = 30.0):
        self.model = model
        self.api_key_env = api_key_env
        self.base_url = (base_url or os.getenv('OPENAI_BASE_URL')
                         or DEFAULT_OPENAI_BASE_URL).rstrip('/')
        self.timeout_s = timeout_s

    def complete(self, prompt: str, jpeg_bytes: bytes) -> str:
        api_key = os.getenv(self.api_key_env)
        if not api_key:
            raise VlmError(f'环境变量 {self.api_key_env} 未设置')
        payload = {
            'model': self.model,
            'temperature': 0.4,
            'messages': [{
                'role': 'user',
                'content': [
                    {'type': 'text', 'text': prompt},
                    {'type': 'image_url', 'image_url': {
                        'url': 'data:image/jpeg;base64,'
                               + base64.b64encode(jpeg_bytes).decode('ascii'),
                    }},
                ],
            }],
        }
        response = _post_json(
            f'{self.base_url}/chat/completions', payload, self.timeout_s,
            {'Authorization': f'Bearer {api_key}'})
        try:
            return response['choices'][0]['message']['content']
        except (KeyError, IndexError) as exc:
            raise VlmError(f'OpenAI 兼容接口返回格式不符合预期：{str(response)[:200]}') from exc


def _post_json(url: str, payload: dict, timeout_s: float, headers: dict) -> dict:
    body = json.dumps(payload).encode('utf-8')
    request = urllib.request.Request(
        url, data=body, method='POST',
        headers={'Content-Type': 'application/json', **headers})
    try:
        with urllib.request.urlopen(request, timeout=timeout_s) as response:
            return json.loads(response.read().decode('utf-8'))
    except urllib.error.HTTPError as exc:
        detail = exc.read().decode('utf-8', errors='replace')[:300]
        raise VlmError(f'HTTP {exc.code}：{detail}') from exc
    except urllib.error.URLError as exc:
        raise VlmError(f'网络错误：{exc.reason}') from exc


def create_client(provider: str, model: str = '', api_key_env: str = '',
                  timeout_s: float = 30.0) -> BaseVlmClient:
    """按 provider 创建客户端：``mock`` / ``gemini`` / ``openai``。"""
    name = (provider or 'mock').strip().lower()
    if name == 'mock':
        return MockVlmClient()
    if name == 'gemini':
        return GeminiVlmClient(
            model=model or 'gemini-2.5-flash',
            api_key_env=api_key_env or 'GEMINI_API_KEY',
            timeout_s=timeout_s)
    if name == 'openai':
        return OpenAiVlmClient(
            model=model or 'google/gemini-2.5-flash',
            api_key_env=api_key_env or 'OPENAI_API_KEY',
            timeout_s=timeout_s)
    raise VlmError(f'不支持的 provider：{provider}')
