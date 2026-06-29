import numpy as np

from .base import Detection


def detections_to_scene_objects(detections: dict[str, Detection]) -> dict:
    return {
        name: {
            "position": np.array(detection.position, dtype=float).copy(),
        }
        for name, detection in detections.items()
    }
