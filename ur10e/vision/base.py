from abc import ABC, abstractmethod
from dataclasses import dataclass
import numpy as np


@dataclass
class Detection:
    name: str
    position: np.ndarray
    confidence: float


class VisionProvider(ABC):
    @abstractmethod
    def detect_objects(self) -> dict[str, Detection]:
        pass