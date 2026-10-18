"""检测内核单测（不需要 ROS 运行）。"""

import numpy as np
import pytest

from perception.detectors import (Detection, bgr_to_hsv, color_blob_detect,
                                  make_detector)


def _image_with_rect(color_bgr, rect, size=(640, 480)):
    width, height = size
    image = np.zeros((height, width, 3), np.uint8)
    x1, y1, x2, y2 = rect
    image[y1:y2, x1:x2] = color_bgr
    return image


def test_hsv_pure_red():
    image = np.zeros((1, 1, 3), np.uint8)
    image[0, 0] = (0, 0, 255)                    # BGR 纯红
    hsv = bgr_to_hsv(image)
    hue = float(hsv[0, 0, 0])
    assert hue < 1e-3 or hue > 359.0
    assert hsv[0, 0, 1] == pytest.approx(1.0, abs=1e-3)
    assert hsv[0, 0, 2] == pytest.approx(1.0, abs=1e-3)


def test_hsv_orange():
    image = np.zeros((1, 1, 3), np.uint8)
    image[0, 0] = (0, 128, 255)                  # BGR 橙
    hsv = bgr_to_hsv(image)
    assert 20.0 <= float(hsv[0, 0, 0]) <= 40.0


def test_color_blob_finds_orange_rect():
    image = _image_with_rect((0, 128, 255), (100, 100, 200, 220))
    detections = color_blob_detect(image, (15.0, 0.5, 0.4), (40.0, 1.0, 1.0),
                                   min_area=200.0)
    assert len(detections) == 1
    box = detections[0]
    assert box.x1 == pytest.approx(100, abs=1)
    assert box.y1 == pytest.approx(100, abs=1)
    assert box.x2 == pytest.approx(200, abs=1)
    assert box.y2 == pytest.approx(220, abs=1)
    assert 0.0 < box.score <= 1.0


def test_color_blob_ignores_small_noise():
    image = _image_with_rect((0, 128, 255), (100, 100, 105, 105))   # 5x5 = 25 px
    detections = color_blob_detect(image, (15.0, 0.5, 0.4), (40.0, 1.0, 1.0),
                                   min_area=200.0)
    assert detections == []


def test_color_blob_ignores_wrong_color():
    image = _image_with_rect((255, 0, 0), (100, 100, 300, 300))     # 纯蓝
    detections = color_blob_detect(image, (15.0, 0.5, 0.4), (40.0, 1.0, 1.0),
                                   min_area=200.0)
    assert detections == []


def test_detection_properties():
    box = Detection(10, 20, 30, 60, score=0.5, label='person')
    assert box.center == (20, 40)
    assert box.width == 20 and box.height == 40
    assert box.area == 800
    payload = box.to_dict()
    assert payload['label'] == 'person'
    assert payload['bbox'] == [10.0, 20.0, 30.0, 60.0]


def test_make_detector_unknown():
    with pytest.raises(ValueError):
        make_detector('nope')


def test_make_detector_color_requires_bounds():
    with pytest.raises(ValueError):
        make_detector('color')


def test_hog_on_blank_image():
    pytest.importorskip('cv2')
    from perception.detectors import HogPersonDetector

    detector = HogPersonDetector()
    blank = np.zeros((480, 640, 3), np.uint8)
    assert detector.detect(blank) == []
