from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np

from .color_detector import detect_block_name


@dataclass(frozen=True)
class YoloDetection:
    name: str
    confidence: float
    bbox: tuple[int, int, int, int]
    center_pixel: tuple[int, int]


def _to_numpy(value: Any) -> np.ndarray:
    if hasattr(value, "detach"):
        value = value.detach()
    if hasattr(value, "cpu"):
        value = value.cpu()
    if hasattr(value, "numpy"):
        value = value.numpy()
    return np.asarray(value)


class YoloDetector:
    """Detect colored blocks in RGB images using YOLO boxes plus HSV labels."""

    def __init__(
        self,
        model_path: str | Path | None = None,
        *,
        confidence_threshold: float = 0.4,
        model: Any | None = None,
    ) -> None:
        if not 0.0 <= confidence_threshold <= 1.0:
            raise ValueError("confidence_threshold must be between 0 and 1")
        if model is None:
            if model_path is None:
                raise ValueError("model_path is required when model is not provided")
            model_path = Path(model_path).expanduser().resolve()
            if not model_path.is_file():
                raise FileNotFoundError(f"YOLO model was not found: {model_path}")
            from ultralytics import YOLO

            model = YOLO(str(model_path))

        self.model = model
        self.confidence_threshold = float(confidence_threshold)

    def detect(self, rgb_image: np.ndarray) -> list[YoloDetection]:
        if rgb_image is None or rgb_image.ndim != 3 or rgb_image.shape[2] != 3:
            raise ValueError("rgb_image must have shape (height, width, 3)")

        height, width = rgb_image.shape[:2]
        results = self.model(rgb_image, conf=self.confidence_threshold, verbose=False)
        detections: list[YoloDetection] = []

        for result in results:
            for box in result.boxes:
                confidence = float(_to_numpy(box.conf[0]).reshape(-1)[0])
                coordinates = _to_numpy(box.xyxy[0]).reshape(-1)
                if coordinates.size != 4:
                    continue

                x1, y1, x2, y2 = coordinates.astype(float)
                crop_x1 = int(np.clip(np.floor(x1), 0, width - 1))
                crop_y1 = int(np.clip(np.floor(y1), 0, height - 1))
                crop_x2 = int(np.clip(np.ceil(x2), crop_x1 + 1, width))
                crop_y2 = int(np.clip(np.ceil(y2), crop_y1 + 1, height))
                center_u = int(np.clip(round((x1 + x2) / 2.0), 0, width - 1))
                center_v = int(np.clip(round((y1 + y2) / 2.0), 0, height - 1))

                name = detect_block_name(
                    rgb_image[crop_y1:crop_y2, crop_x1:crop_x2],
                    color_space="RGB",
                )
                detections.append(
                    YoloDetection(
                        name=name,
                        confidence=confidence,
                        bbox=(crop_x1, crop_y1, crop_x2, crop_y2),
                        center_pixel=(center_u, center_v),
                    )
                )

        return detections
