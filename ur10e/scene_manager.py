from dataclasses import dataclass, field
from typing import Mapping

import numpy as np

from command_parser import RELATION_OFFSETS
from scene_config import BLOCK_SIZE, MTC_OBJECT_POSITION, MTC_OBJECT_SIZE, OBJECT_POSITIONS


@dataclass
class CommandMemory:
    last_picked_object: str | None = None
    last_moved_object: str | None = None
    last_target_object: str | None = None
    last_relation: str | None = None


@dataclass
class SceneObject:
    name: str
    position: np.ndarray
    orientation: np.ndarray | None = None
    size: np.ndarray | None = None
    confidence: float = 1.0
    status: str = "on_table"
    metadata: dict = field(default_factory=dict)

    def copy(self) -> "SceneObject":
        return SceneObject(
            name=self.name,
            position=np.array(self.position, dtype=float).copy(),
            orientation=None if self.orientation is None else np.array(self.orientation, dtype=float).copy(),
            size=None if self.size is None else np.array(self.size, dtype=float).copy(),
            confidence=float(self.confidence),
            status=self.status,
            metadata=dict(self.metadata),
        )


class SceneManager:
    def __init__(self, objects: Mapping[str, SceneObject] | None = None) -> None:
        self._objects: dict[str, SceneObject] = {}
        self.memory = CommandMemory()
        if objects:
            for name, scene_object in objects.items():
                self._objects[name] = scene_object.copy()

    @classmethod
    def from_defaults(cls) -> "SceneManager":
        objects = {
            name: SceneObject(
                name=name,
                position=np.array(position, dtype=float).copy(),
                size=np.array(BLOCK_SIZE, dtype=float).copy(),
            )
            for name, position in OBJECT_POSITIONS.items()
        }
        objects["object"] = SceneObject(
            name="object",
            position=np.array(MTC_OBJECT_POSITION, dtype=float).copy(),
            size=np.array(MTC_OBJECT_SIZE, dtype=float).copy(),
        )
        return cls(objects)

    @classmethod
    def from_command_scene(cls, scene_objects: Mapping[str, dict]) -> "SceneManager":
        manager = cls()
        for name, info in scene_objects.items():
            manager.update_object(
                name,
                position=info["position"],
                orientation=info.get("orientation"),
                size=info.get("size"),
                confidence=info.get("confidence", 1.0),
                status=info.get("status", "on_table"),
            )
        return manager

    def update_object(
        self,
        name: str,
        position,
        orientation=None,
        size=None,
        confidence: float = 1.0,
        status: str = "on_table",
        metadata: dict | None = None,
    ) -> None:
        existing = self._objects.get(name)
        if existing is not None:
            if orientation is None:
                orientation = existing.orientation
            if size is None:
                size = existing.size
            if metadata is None:
                metadata = existing.metadata

        self._objects[name] = SceneObject(
            name=name,
            position=np.array(position, dtype=float).copy(),
            orientation=None if orientation is None else np.array(orientation, dtype=float).copy(),
            size=None if size is None else np.array(size, dtype=float).copy(),
            confidence=float(confidence),
            status=status,
            metadata=dict(metadata or {}),
        )

    def update_from_detections(self, detections: Mapping) -> None:
        for name, detection in detections.items():
            self.update_object(
                name,
                position=detection.position,
                confidence=getattr(detection, "confidence", 1.0),
            )

    def get_object(self, name: str) -> SceneObject:
        if name not in self._objects:
            raise KeyError(f"scene에 없는 object입니다: {name}")
        return self._objects[name].copy()

    def as_command_scene(self) -> dict:
        scene = {}
        for name, scene_object in self._objects.items():
            info = {
                "position": np.array(scene_object.position, dtype=float).copy(),
                "confidence": scene_object.confidence,
                "status": scene_object.status,
            }
            if scene_object.orientation is not None:
                info["orientation"] = np.array(scene_object.orientation, dtype=float).copy()
            if scene_object.size is not None:
                info["size"] = np.array(scene_object.size, dtype=float).copy()
            scene[name] = info
        return scene

    def names(self) -> set[str]:
        return set(self._objects)

    def resolve_target_position(self, command: Mapping) -> np.ndarray:
        target_object = command.get("target_object")
        relation = command.get("relation")

        if target_object not in self._objects:
            raise ValueError(f"scene에 없는 target_object입니다: {target_object}")

        if relation not in RELATION_OFFSETS:
            raise ValueError(f"지원하지 않는 relation입니다: {relation}")

        target_object_position = np.array(self._objects[target_object].position, dtype=float)
        return target_object_position + RELATION_OFFSETS[relation]

    def apply_pick_place_result(self, command: Mapping, final_position) -> None:
        pick_object = command.get("pick_object")
        target_object = command.get("target_object")
        relation = command.get("relation")

        if pick_object not in self._objects:
            raise ValueError(f"scene에 없는 pick_object입니다: {pick_object}")

        self.update_object(pick_object, final_position, status="on_table")
        self.memory.last_picked_object = pick_object
        self.memory.last_moved_object = pick_object
        self.memory.last_target_object = target_object
        self.memory.last_relation = relation
