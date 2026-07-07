from dataclasses import dataclass

import cv2
import numpy as np


COLOR_TO_BLOCK_NAME = {
    "red": "red_block",
    "blue": "blue_block",
    "green": "green_block",
}


@dataclass(frozen=True)
class ColorDetection:
    color: str
    confidence: float
    scores: dict[str, float]


def detect_dominant_color(
    image: np.ndarray,
    *,
    color_space: str = "BGR",
    min_ratio: float = 0.08,
    dominance_margin: float = 1.35,
) -> ColorDetection:
    """Detect the dominant block color in an RGB/BGR crop.

    The score is based on the fraction of saturated, bright pixels whose hue
    falls into each block color range. This is more stable than a raw pixel
    count because YOLO crops often include table/background around the block.
    """

    if image is None or image.size == 0:
        return ColorDetection("unknown", 0.0, {})

    color_space = color_space.upper()
    if color_space == "BGR":
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    elif color_space == "RGB":
        hsv = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)
    else:
        raise ValueError("color_space must be 'BGR' or 'RGB'")

    hue = hsv[:, :, 0]
    saturation = hsv[:, :, 1]
    value = hsv[:, :, 2]
    chromatic_mask = (saturation >= 50) & (value >= 45)
    chromatic_pixels = int(np.count_nonzero(chromatic_mask))
    if chromatic_pixels == 0:
        return ColorDetection("unknown", 0.0, {"red": 0.0, "blue": 0.0, "green": 0.0})

    masks = {
        "red": ((hue <= 10) | (hue >= 170)) & chromatic_mask,
        "blue": (hue >= 95) & (hue <= 135) & chromatic_mask,
        "green": (hue >= 35) & (hue <= 90) & chromatic_mask,
    }
    scores = {
        color: float(np.count_nonzero(mask)) / chromatic_pixels
        for color, mask in masks.items()
    }

    ranked = sorted(scores.items(), key=lambda item: item[1], reverse=True)
    best_color, best_score = ranked[0]
    second_score = ranked[1][1] if len(ranked) > 1 else 0.0

    if best_score < min_ratio:
        return ColorDetection("unknown", best_score, scores)
    if second_score > 0 and best_score < second_score * dominance_margin:
        return ColorDetection("unknown", best_score, scores)

    return ColorDetection(best_color, best_score, scores)


def detect_color(image: np.ndarray, *, color_space: str = "BGR") -> str:
    return detect_dominant_color(image, color_space=color_space).color


def color_to_block_name(color: str) -> str:
    return COLOR_TO_BLOCK_NAME.get(color, "unknown_block")


def detect_block_name(image: np.ndarray, *, color_space: str = "BGR") -> str:
    return color_to_block_name(detect_color(image, color_space=color_space))
