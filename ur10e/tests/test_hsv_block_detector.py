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


def test_rejects_large_same_color_workcell_surface():
    rgb = np.full((100, 100, 3), 180, dtype=np.uint8)
    rgb[40:55, 40:55] = [0, 255, 0]
    rgb[70:100, 0:100] = [0, 255, 0]

    detections = HsvBlockDetector(max_area_fraction=0.05).detect(rgb)

    assert len(detections) == 1
    assert detections[0].name == "green_block"
    assert detections[0].bbox == (40, 40, 55, 55)


def test_rejects_invalid_rgb_shape():
    with pytest.raises(ValueError, match="shape"):
        HsvBlockDetector().detect(np.zeros((10, 10), dtype=np.uint8))


def test_opening_default_preserved_and_optional_single_pixel_corner_retained():
    rgb = np.zeros((100, 100, 3), dtype=np.uint8)
    rgb[40:50, 40:50] = [255, 0, 0]
    rgb[39, 39] = [255, 0, 0]
    default = HsvBlockDetector().detect(rgb)[0].mask
    explicit = HsvBlockDetector(opening_kernel_size=3).detect(rgb)[0].mask
    raw = HsvBlockDetector(opening_kernel_size=1).detect(rgb)[0].mask
    np.testing.assert_array_equal(default, explicit)
    assert not default[39, 39] and raw[39, 39]
    assert raw.sum() == default.sum()+1


@pytest.mark.parametrize("kernel", [0, 2, 10, True, 3.0])
def test_invalid_opening_kernel_rejected(kernel):
    with pytest.raises(ValueError, match="opening_kernel_size"):
        HsvBlockDetector(opening_kernel_size=kernel)
