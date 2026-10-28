import numpy as np
import pytest

from vision.hsv_block_detector import HsvBlockDetector


def test_detects_small_colored_blocks_without_model_weights():
    rgb = np.full((120, 160, 3), 150, dtype=np.uint8)
    rgb[30:48, 20:42] = [255, 0, 0]
    rgb[60:78, 65:87] = [0, 255, 0]
    rgb[25:43, 110:132] = [0, 0, 255]

    detections = {item.name: item for item in HsvBlockDetector().detect(rgb)}

    assert set(detections) == {"red_block", "green_block", "blue_block"}
    assert detections["red_block"].bbox == (20, 30, 42, 48)
    assert detections["red_block"].mask.sum() == 18 * 22
    assert detections["red_block"].confidence >= 0.6


def test_rejects_small_color_speckle_and_unsaturated_background():
    rgb = np.full((100, 100, 3), 180, dtype=np.uint8)
    rgb[10:13, 10:13] = [255, 0, 0]

    assert HsvBlockDetector().detect(rgb) == []


def test_rejects_invalid_rgb_shape():
    with pytest.raises(ValueError, match="shape"):
        HsvBlockDetector().detect(np.zeros((10, 10), dtype=np.uint8))
