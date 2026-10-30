"""Model-free RGB detector for the calibrated red, green, and blue blocks."""

import cv2
import numpy as np

from .yolo_detector import YoloDetection


HSV_RANGES = {
    "red": ((0, 10), (170, 179)),
    "green": ((35, 90),),
    "blue": ((95, 135),),
}


class HsvBlockDetector:
    def __init__(self, *, min_area_pixels=30, max_area_fraction=0.05,
                 min_saturation=70, min_value=45, opening_kernel_size=3):
        if min_area_pixels <= 0:
            raise ValueError("min_area_pixels must be positive")
        self.min_area_pixels = int(min_area_pixels)
        if not 0.0 < max_area_fraction <= 1.0:
            raise ValueError("max_area_fraction must be within (0, 1]")
        self.max_area_fraction = float(max_area_fraction)
        self.min_saturation = int(min_saturation)
        self.min_value = int(min_value)
        if not 0 <= self.min_saturation <= 255 or not 0 <= self.min_value <= 255:
            raise ValueError("HSV saturation and value thresholds must be between 0 and 255")
        self.label_mode = "color"
        if (isinstance(opening_kernel_size, bool) or not isinstance(opening_kernel_size, int)
                or opening_kernel_size not in (1, 3, 5, 7, 9)):
            raise ValueError("opening_kernel_size must be an odd integer in [1,9]")
        self.opening_kernel_size = opening_kernel_size

    def detect(self, rgb_image):
        if rgb_image is None or rgb_image.ndim != 3 or rgb_image.shape[2] != 3:
            raise ValueError("rgb_image must have shape (height, width, 3)")
        hsv = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2HSV)
        detections = []
        max_area_pixels = int(rgb_image.shape[0] * rgb_image.shape[1] * self.max_area_fraction)
        kernel = np.ones((self.opening_kernel_size, self.opening_kernel_size), dtype=np.uint8)
        for color, ranges in HSV_RANGES.items():
            color_mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
            for lower_hue, upper_hue in ranges:
                color_mask |= cv2.inRange(
                    hsv,
                    (lower_hue, self.min_saturation, self.min_value),
                    (upper_hue, 255, 255),
                )
            color_mask = cv2.morphologyEx(color_mask, cv2.MORPH_OPEN, kernel)
            component_count, labels, stats, centroids = cv2.connectedComponentsWithStats(
                color_mask, connectivity=8
            )
            for component in range(1, component_count):
                x, y, width, height, area = [int(v) for v in stats[component]]
                if area < self.min_area_pixels or area > max_area_pixels:
                    continue
                mask = labels == component
                center_u, center_v = [int(round(v)) for v in centroids[component]]
                confidence = min(1.0, area / (area + self.min_area_pixels / 2.0))
                detections.append(
                    YoloDetection(
                        name=f"{color}_block",
                        confidence=confidence,
                        bbox=(x, y, x + width, y + height),
                        center_pixel=(center_u, center_v),
                        mask=mask,
                        class_name="block",
                        color=color,
                    )
                )
        return detections
