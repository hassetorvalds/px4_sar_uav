"""目标检测内核：**纯函数 + 可插拔 backend**，不依赖 ROS。

这样做的目的：先用已存的 1245 张 AirSim 帧离线评估（召回/误报/耗时），
再接进 ROS 节点；换检测器不影响任务层接口。

backend：
- ``color``：颜色连通域（默认；不需要 OpenCV，scipy 可选）；用于信标/标记物或
  明显着装的假人；
- ``hog``：OpenCV HOG + 行人 SVM（需要 cv2）；
- ``haar``：OpenCV 全身级联（需要 cv2）。

坐标约定：bbox 用像素坐标 ``(x1, y1, x2, y2)``，左上角为原点。
"""

from dataclasses import dataclass

import numpy as np

DEFAULT_HAAR_PATH = '/usr/share/opencv4/haarcascades/haarcascade_fullbody.xml'


@dataclass(frozen=True)
class Detection:
    """一个检测框。``score`` 约定在 [0, 1]。"""

    x1: float
    y1: float
    x2: float
    y2: float
    score: float = 1.0
    label: str = 'target'

    @property
    def center(self):
        return ((self.x1 + self.x2) / 2.0, (self.y1 + self.y2) / 2.0)

    @property
    def width(self):
        return max(0.0, self.x2 - self.x1)

    @property
    def height(self):
        return max(0.0, self.y2 - self.y1)

    @property
    def area(self):
        return self.width * self.height

    def to_dict(self):
        return {
            'label': self.label,
            'score': round(float(self.score), 3),
            'bbox': [round(float(v), 1) for v in (self.x1, self.y1, self.x2, self.y2)],
            'center': [round(float(v), 1) for v in self.center],
            'area': round(float(self.area), 1),
        }


def bgr_to_hsv(bgr: np.ndarray) -> np.ndarray:
    """BGR uint8/float → HSV，H∈[0,360)，S/V∈[0,1]。

    不依赖 cv2，保证颜色 backend 在最简环境也能跑。
    """
    arr = bgr.astype(np.float32)
    if arr.max(initial=0.0) > 1.5:          # uint8 量级 → 归一化
        arr = arr / 255.0
    blue, green, red = arr[..., 0], arr[..., 1], arr[..., 2]
    maxc = np.max(arr, axis=-1)
    minc = np.min(arr, axis=-1)
    delta = maxc - minc

    hue = np.zeros_like(maxc)
    nonzero = delta > 1e-6
    # 红色主导
    mask = nonzero & (maxc == red)
    hue[mask] = (60.0 * ((green[mask] - blue[mask]) / delta[mask])) % 360.0
    # 绿色主导
    mask = nonzero & (maxc == green)
    hue[mask] = 60.0 * ((blue[mask] - red[mask]) / delta[mask]) + 120.0
    # 蓝色主导
    mask = nonzero & (maxc == blue)
    hue[mask] = 60.0 * ((red[mask] - green[mask]) / delta[mask]) + 240.0

    saturation = np.where(maxc > 1e-6, delta / np.maximum(maxc, 1e-6), 0.0)
    return np.stack([hue, saturation, maxc], axis=-1)


def _label_components(mask: np.ndarray):
    """返回 [(label, area, (x1,y1,x2,y2)), ...]；scipy 缺失时退化为整体 bbox。"""
    try:
        from scipy import ndimage  # noqa: PLC0415 - 可选依赖

        labels, count = ndimage.label(mask)
        if count == 0:
            return []
        slices = ndimage.find_objects(labels)
        out = []
        for index, slc in enumerate(slices, start=1):
            if slc is None:
                continue
            area = int(np.count_nonzero(labels[slc] == index))
            y_slice, x_slice = slc
            out.append((index, area,
                        (x_slice.start, y_slice.start, x_slice.stop, y_slice.stop)))
        return out
    except Exception:                        # noqa: BLE001 - 退化路径
        ys, xs = np.nonzero(mask)
        if xs.size == 0:
            return []
        return [(1, int(xs.size), (int(xs.min()), int(ys.min()),
                                   int(xs.max()) + 1, int(ys.max()) + 1))]


def color_blob_detect(bgr: np.ndarray, lower, upper, *, min_area: float = 200.0,
                      max_area: float = 1e9, min_aspect: float = 0.15,
                      max_aspect: float = 6.0, label: str = 'target'):
    """颜色连通域检测。

    ``lower``/``upper`` 为 ``(H, S, V)``，H 用度（0–360），S/V 用 0–1。
    bbox 面积与长宽比用来滤掉小块噪声与横跨画面的天空/地面。
    """
    hsv = bgr_to_hsv(bgr)
    low = np.asarray(lower, dtype=np.float32)
    high = np.asarray(upper, dtype=np.float32)
    if low[0] > high[0]:                     # 跨 0° 的红色区间
        hue_ok = (hsv[..., 0] >= low[0]) | (hsv[..., 0] <= high[0])
    else:
        hue_ok = (hsv[..., 0] >= low[0]) & (hsv[..., 0] <= high[0])
    mask = (hue_ok
            & (hsv[..., 1] >= low[1]) & (hsv[..., 1] <= high[1])
            & (hsv[..., 2] >= low[2]) & (hsv[..., 2] <= high[2]))

    detections = []
    for _index, area, (x1, y1, x2, y2) in _label_components(mask):
        if area < min_area or area > max_area:
            continue
        width, height = max(x2 - x1, 1), max(y2 - y1, 1)
        aspect = width / height
        if aspect < min_aspect or aspect > max_aspect:
            continue
        # 置信度：面积越大越可信（对数压缩，封顶 1.0）
        score = float(min(1.0, 0.4 + 0.15 * np.log10(max(area, 1.0))))
        detections.append(Detection(float(x1), float(y1), float(x2), float(y2),
                                    score=score, label=label))
    detections.sort(key=lambda d: d.area, reverse=True)
    return detections


class HogPersonDetector:
    """OpenCV HOG + 行人 SVM。CPU 上约 20 ms/帧（640 宽）。"""

    def __init__(self, *, threshold: float = 0.5, resize_width: int = 640,
                 win_stride: int = 8, scale: float = 1.05, label: str = 'person'):
        import cv2  # noqa: PLC0415 - 可选依赖

        self._cv2 = cv2
        self._hog = cv2.HOGDescriptor()
        self._hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())
        self._threshold = float(threshold)
        self._resize_width = int(resize_width)
        self._win_stride = (int(win_stride), int(win_stride))
        self._scale = float(scale)
        self._label = label

    def detect(self, bgr: np.ndarray):
        height, width = bgr.shape[:2]
        ratio = 1.0
        image = bgr
        if self._resize_width > 0 and width > self._resize_width:
            ratio = self._resize_width / float(width)
            image = self._cv2.resize(
                bgr, (self._resize_width, max(int(height * ratio), 1)),
                interpolation=self._cv2.INTER_AREA)
        rects, weights = self._hog.detectMultiScale(
            image, winStride=self._win_stride, padding=(8, 8), scale=self._scale)
        out = []
        for (x, y, w, h), weight in zip(rects, weights):
            score = float(weight)
            if score < self._threshold:
                continue
            out.append(Detection(x / ratio, y / ratio,
                                 (x + w) / ratio, (y + h) / ratio,
                                 score=min(score, 1.0), label=self._label))
        out.sort(key=lambda d: d.score, reverse=True)
        return out


class HaarFullBodyDetector:
    """OpenCV 全身级联；比 HOG 快，但漏检率更高。"""

    def __init__(self, cascade_path: str = DEFAULT_HAAR_PATH, *,
                 resize_width: int = 640, min_neighbors: int = 2,
                 min_size=(40, 80), label: str = 'person'):
        import cv2  # noqa: PLC0415 - 可选依赖

        self._cv2 = cv2
        self._cascade = cv2.CascadeClassifier(cascade_path)
        if self._cascade.empty():
            raise ValueError(f'级联文件不可用：{cascade_path}')
        self._resize_width = int(resize_width)
        self._min_neighbors = int(min_neighbors)
        self._min_size = tuple(int(v) for v in min_size)
        self._label = label

    def detect(self, bgr: np.ndarray):
        gray = self._cv2.cvtColor(bgr, self._cv2.COLOR_BGR2GRAY)
        height, width = gray.shape[:2]
        ratio = 1.0
        if self._resize_width > 0 and width > self._resize_width:
            ratio = self._resize_width / float(width)
            gray = self._cv2.resize(
                gray, (self._resize_width, max(int(height * ratio), 1)),
                interpolation=self._cv2.INTER_AREA)
        rects = self._cascade.detectMultiScale(
            gray, scaleFactor=1.05, minNeighbors=self._min_neighbors,
            minSize=self._min_size)
        out = [Detection(x / ratio, y / ratio, (x + w) / ratio, (y + h) / ratio,
                         score=1.0, label=self._label)
               for (x, y, w, h) in rects]
        out.sort(key=lambda d: d.area, reverse=True)
        return out


def make_detector(kind: str, **kwargs):
    """工厂：``color`` / ``hog`` / ``haar``。未知类型抛 ValueError。"""
    kind = (kind or 'color').strip().lower()
    if kind == 'color':
        allowed = {k: v for k, v in kwargs.items()
                   if k in ('lower', 'upper', 'min_area', 'max_area',
                            'min_aspect', 'max_aspect', 'label')}
        required = {'lower', 'upper'} - set(allowed)
        if required:
            raise ValueError(f'color 检测器缺少参数：{sorted(required)}')
        return _ColorAdapter(**allowed)
    if kind == 'hog':
        return HogPersonDetector(**{k: v for k, v in kwargs.items()
                                    if k in ('threshold', 'resize_width',
                                             'win_stride', 'scale', 'label')})
    if kind == 'haar':
        return HaarFullBodyDetector(**{k: v for k, v in kwargs.items()
                                       if k in ('cascade_path', 'resize_width',
                                                'min_neighbors', 'min_size', 'label')})
    raise ValueError(f'未知检测器类型：{kind!r}（可选 color/hog/haar）')


class _ColorAdapter:
    def __init__(self, lower, upper, **kwargs):
        self._lower = lower
        self._upper = upper
        self._kwargs = kwargs

    def detect(self, bgr):
        return color_blob_detect(bgr, self._lower, self._upper, **self._kwargs)
