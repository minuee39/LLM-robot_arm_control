import numpy as np

from vision.color_detector import detect_block_name, detect_color


def solid_bgr(color):
    image = np.zeros((40, 40, 3), dtype=np.uint8)
    image[:, :] = color
    return image


def test_detects_primary_block_colors_from_bgr_crops():
    assert detect_color(solid_bgr((0, 0, 255))) == "red"
    assert detect_color(solid_bgr((255, 0, 0))) == "blue"
    assert detect_color(solid_bgr((0, 255, 0))) == "green"


def test_detects_primary_block_colors_from_rgb_crops():
    blue_rgb = np.zeros((40, 40, 3), dtype=np.uint8)
    blue_rgb[:, :] = (0, 0, 255)

    assert detect_color(blue_rgb, color_space="RGB") == "blue"
    assert detect_block_name(blue_rgb, color_space="RGB") == "blue_block"


def test_ignores_background_around_crop():
    image = np.full((80, 80, 3), 190, dtype=np.uint8)
    image[20:60, 20:60] = (255, 0, 0)

    assert detect_color(image) == "blue"


def test_unknown_for_empty_or_low_saturation_crop():
    assert detect_color(np.empty((0, 0, 3), dtype=np.uint8)) == "unknown"
    assert detect_color(np.full((40, 40, 3), 128, dtype=np.uint8)) == "unknown"
