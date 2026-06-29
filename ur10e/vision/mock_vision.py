import numpy as np
from .base import VisionProvider, Detection


DEFAULT_OBJECT_POSITIONS = {
    "red_block": np.array([-0.30, 0.30, 0.02575]),
    "blue_block": np.array([0.30, 0.30, 0.02575]),
    "green_block": np.array([0.00, 0.45, 0.02575]),
}


class MockVision(VisionProvider):
    def __init__(self, object_positions: dict | None = None, confidence: float = 1.0):
        self.confidence = confidence
        initial_positions = object_positions or DEFAULT_OBJECT_POSITIONS
        self._object_positions = {
            name: np.array(position, dtype=float).copy()
            for name, position in initial_positions.items()
        }

    def detect_objects(self) -> dict[str, Detection]:
        return {
            name: Detection(name, position.copy(), self.confidence)
            for name, position in self._object_positions.items()
        }

    def update_object_position(self, object_name: str, position: np.ndarray) -> None:
        self._object_positions[object_name] = np.array(position, dtype=float).copy()

    def update_object_positions(self, object_positions: dict) -> None:
        for object_name, position in object_positions.items():
            self.update_object_position(object_name, position)
