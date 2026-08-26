from dataclasses import dataclass
from pathlib import Path
import re
from typing import Any

import numpy as np

from .color_detector import color_to_block_name, detect_color


LABEL_MODES = ("color", "model", "hybrid")


@dataclass(frozen=True)
class YoloDetection:
    name: str
    confidence: float
    bbox: tuple[int, int, int, int]
    center_pixel: tuple[int, int]
    mask: np.ndarray | None = None
    class_name: str = "unknown"
    color: str = "unknown"


def _to_numpy(value: Any) -> np.ndarray:
    if hasattr(value, "detach"):
        value = value.detach()
    if hasattr(value, "cpu"):
        value = value.cpu()
    if hasattr(value, "numpy"):
        value = value.numpy()
    return np.asarray(value)


def _normalize_label(value: Any) -> str:
    label = re.sub(r"[^a-z0-9]+", "_", str(value).strip().lower()).strip("_")
    return label or "unknown"


def _class_name(box: Any, names: Any) -> str:
    if not hasattr(box, "cls"):
        return "unknown"
    class_values = _to_numpy(box.cls).reshape(-1)
    if class_values.size == 0:
        return "unknown"
    class_id = int(class_values[0])
    if isinstance(names, dict):
        return _normalize_label(names.get(class_id, class_id))
    if isinstance(names, (list, tuple)) and 0 <= class_id < len(names):
        return _normalize_label(names[class_id])
    return _normalize_label(class_id)


def _hybrid_name(color: str, class_name: str) -> str:
    if class_name == "unknown":
        return color_to_block_name(color)
    if class_name in {"red_block", "green_block", "blue_block"}:
        return class_name
    if color == "unknown":
        return class_name
    if class_name == "block":
        return color_to_block_name(color)
    return f"{color}_{class_name}"


def _mask_center(mask: np.ndarray) -> tuple[int, int]:
    """Choose the foreground pixel nearest the mask's median location."""
    mask_v, mask_u = np.nonzero(mask)
    median_u = float(np.median(mask_u))
    median_v = float(np.median(mask_v))
    nearest_index = int(np.argmin((mask_u - median_u) ** 2 + (mask_v - median_v) ** 2))
    return int(mask_u[nearest_index]), int(mask_v[nearest_index])


class YoloDetector:
    """Detect objects and label them by HSV color, YOLO class, or both."""

    def __init__(
        self,
        model_path: str | Path | None = None,
        *,
        confidence_threshold: float = 0.4,
        label_mode: str = "color",
        classes: tuple[str, ...] | list[str] | set[str] | None = None,
        require_segmentation: bool = False,
        model: Any | None = None,
    ) -> None:
        if not 0.0 <= confidence_threshold <= 1.0:
            raise ValueError("confidence_threshold must be between 0 and 1")
        label_mode = label_mode.strip().lower()
        if label_mode not in LABEL_MODES:
            raise ValueError(f"label_mode must be one of: {', '.join(LABEL_MODES)}")
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
        self.label_mode = label_mode
        self.require_segmentation = bool(require_segmentation)
        model_task = getattr(model, "task", None)
        if self.require_segmentation and model_task not in {None, "segment"}:
            raise ValueError(
                f"the loaded YOLO model task is {model_task!r}, not 'segment'; "
                "use segmentation weights such as yolo26n-seg.pt"
            )
        self.classes = (
            {_normalize_label(class_name) for class_name in classes}
            if classes
            else None
        )

    def detect(self, rgb_image: np.ndarray) -> list[YoloDetection]:
        if rgb_image is None or rgb_image.ndim != 3 or rgb_image.shape[2] != 3:
            raise ValueError("rgb_image must have shape (height, width, 3)")

        height, width = rgb_image.shape[:2]
        results = self.model(rgb_image, conf=self.confidence_threshold, verbose=False)
        detections: list[YoloDetection] = []

        for result in results:
            names = getattr(result, "names", None)
            if names is None:
                names = getattr(self.model, "names", None)
            result_masks = getattr(getattr(result, "masks", None), "data", None)
            if self.require_segmentation and result_masks is None and len(result.boxes) > 0:
                raise ValueError(
                    "the loaded YOLO model does not provide segmentation masks; "
                    "use segmentation weights such as yolo26n-seg.pt"
                )
            masks = _to_numpy(result_masks) if result_masks is not None else None
            for detection_index, box in enumerate(result.boxes):
                confidence = float(_to_numpy(box.conf[0]).reshape(-1)[0])
                class_name = _class_name(box, names)
                if self.classes is not None and class_name not in self.classes:
                    continue
                coordinates = _to_numpy(box.xyxy[0]).reshape(-1)
                if coordinates.size != 4:
                    continue

                x1, y1, x2, y2 = coordinates.astype(float)
                crop_x1 = int(np.clip(np.floor(x1), 0, width - 1))
                crop_y1 = int(np.clip(np.floor(y1), 0, height - 1))
                crop_x2 = int(np.clip(np.ceil(x2), crop_x1 + 1, width))
                crop_y2 = int(np.clip(np.ceil(y2), crop_y1 + 1, height))
                mask = None
                if masks is not None and detection_index < len(masks):
                    import cv2

                    mask_values = np.asarray(masks[detection_index], dtype=np.float32)
                    if mask_values.shape != (height, width):
                        mask_values = cv2.resize(
                            mask_values,
                            (width, height),
                            interpolation=cv2.INTER_LINEAR,
                        )
                    mask = mask_values >= 0.5
                    if not np.any(mask):
                        if self.require_segmentation:
                            continue
                        mask = None

                if mask is not None:
                    center_u, center_v = _mask_center(mask)
                    color_crop = rgb_image[crop_y1:crop_y2, crop_x1:crop_x2].copy()
                    crop_mask = mask[crop_y1:crop_y2, crop_x1:crop_x2]
                    color_crop[~crop_mask] = 0
                else:
                    center_u = int(np.clip(round((x1 + x2) / 2.0), 0, width - 1))
                    center_v = int(np.clip(round((y1 + y2) / 2.0), 0, height - 1))
                    color_crop = rgb_image[crop_y1:crop_y2, crop_x1:crop_x2]

                color = detect_color(
                    color_crop,
                    color_space="RGB",
                )
                if self.label_mode == "color":
                    name = color_to_block_name(color)
                elif self.label_mode == "model":
                    if class_name == "unknown":
                        continue
                    name = class_name
                else:
                    name = _hybrid_name(color, class_name)
                    if name == "unknown_block":
                        continue
                detections.append(
                    YoloDetection(
                        name=name,
                        confidence=confidence,
                        bbox=(crop_x1, crop_y1, crop_x2, crop_y2),
                        center_pixel=(center_u, center_v),
                        mask=mask,
                        class_name=class_name,
                        color=color,
                    )
                )

        return detections
