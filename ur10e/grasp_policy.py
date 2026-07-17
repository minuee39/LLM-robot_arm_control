from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import Any, Mapping

import numpy as np


@dataclass
class GraspRequest:
    object_name: str
    object_pose: np.ndarray
    target_position: np.ndarray
    object_size: np.ndarray | None = None
    object_type: str | None = None
    nearby_obstacles: tuple[Any, ...] = ()
    robot_state: Mapping[str, Any] | None = None


@dataclass
class GraspAction:
    end_effector_offset: np.ndarray
    gripper_command: float | None = None
    confidence: float = 1.0
    metadata: dict = field(default_factory=dict)


class GraspPolicy(ABC):
    @abstractmethod
    def predict(self, request: GraspRequest) -> GraspAction:
        pass


class FixedGraspPolicy(GraspPolicy):
    def __init__(
        self,
        end_effector_offset=None,
        gripper_command: float | None = None,
        confidence: float = 1.0,
    ) -> None:
        if end_effector_offset is None:
            end_effector_offset = [0.0, 0.0, 0.20]
        self.end_effector_offset = np.array(end_effector_offset, dtype=float)
        self.gripper_command = gripper_command
        self.confidence = float(confidence)

    def predict(self, request: GraspRequest) -> GraspAction:
        return GraspAction(
            end_effector_offset=self.end_effector_offset.copy(),
            gripper_command=self.gripper_command,
            confidence=self.confidence,
            metadata={
                "policy": self.__class__.__name__,
                "object_name": request.object_name,
            },
        )
