"""确定性视觉桩：用颜色检测替代 VLM，用于在没有 API key 时验证**闭环动力学**。

它与真实 VLM 走完全相同的接口（返回同样的 JSON 指向结果），
区别只在于"看图像"这一步是规则而非模型，因此可复现、可解释。
注意：它是测试替身，不是感知模块；正式的人员/目标识别仍在 perception 阶段实现。
"""

import io
import json
from typing import Optional, Tuple

from .vlm_client import BaseVlmClient

#: 目标颜色阈值（RGB）——AirSim 场景里的橙色球体
DEFAULT_COLOR_RANGES = {
    'orange': {'r': (140, 255), 'g': (60, 180), 'b': (0, 110)},
    'red': {'r': (120, 255), 'g': (0, 90), 'b': (0, 90)},
}


def locate_color(jpeg_bytes: bytes, color: str = 'orange'
                 ) -> Optional[Tuple[float, float, float]]:
    """在图像中定位指定颜色区域，返回 (x_norm, y_norm, area_ratio)。"""
    import numpy as np
    from PIL import Image

    limits = DEFAULT_COLOR_RANGES.get(color)
    if limits is None:
        raise ValueError(f'未知颜色：{color}')

    with Image.open(io.BytesIO(jpeg_bytes)) as image:
        arr = np.asarray(image.convert('RGB'), dtype=np.uint8)

    mask = ((arr[:, :, 0] >= limits['r'][0]) & (arr[:, :, 0] <= limits['r'][1])
            & (arr[:, :, 1] >= limits['g'][0]) & (arr[:, :, 1] <= limits['g'][1])
            & (arr[:, :, 2] >= limits['b'][0]) & (arr[:, :, 2] <= limits['b'][1]))
    count = int(mask.sum())
    if count == 0:
        return None
    height, width = mask.shape
    ys, xs = np.nonzero(mask)
    return (float(xs.mean()) / width, float(ys.mean()) / height,
            count / float(width * height))


def depth_from_area(area_ratio: float, min_m: float = 0.2, max_m: float = 2.0) -> float:
    """按目标占画面比例粗估距离：占得越大越近。"""
    if area_ratio <= 0.0:
        return max_m
    estimate = 0.05 / (area_ratio ** 0.5)
    return min(max(estimate, min_m), max_m)


class VisionStubClient(BaseVlmClient):
    """把颜色检测包装成与 VLM 相同的返回格式（0-1000 坐标 + 1-10 深度）。"""

    def __init__(self, color: str = 'orange'):
        self.color = color
        self.last_area_ratio = 0.0

    def complete(self, prompt: str, jpeg_bytes: bytes) -> str:
        found = locate_color(jpeg_bytes, self.color)
        if found is None:
            # 与真实 VLM 的约定一致：看不到目标就返回空数组
            return '[]'
        x_norm, y_norm, area = found
        self.last_area_ratio = area
        depth_m = depth_from_area(area)
        # 反推回 1-10 的深度评分（与 pointing.depth_score_to_meters 互逆）
        score = 1.0 + (depth_m - 0.2) / (2.0 - 0.2) * 9.0
        payload = [{
            'point': [int(round(y_norm * 1000)), int(round(x_norm * 1000))],
            'depth': round(max(1.0, min(score, 10.0)), 2),
            'label': f'{self.color} target (visual stub)',
        }]
        return json.dumps(payload)
