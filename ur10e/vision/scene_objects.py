import numpy as np
import json
import os
from pathlib import Path
import time
from collections import deque

from .base import Detection


EXPECTED_BLOCK_NAMES = ("red_block", "green_block", "blue_block")


class TimedDetectionStore:
    def __init__(self, ttl_seconds=3.0, expected_names=EXPECTED_BLOCK_NAMES):
        if ttl_seconds <= 0.0:
            raise ValueError("ttl_seconds must be positive")
        self.ttl_seconds = float(ttl_seconds)
        self.expected_names = tuple(expected_names)
        self._detections = {}

    def update(self, name, confidence, position, timestamp=None):
        if name not in self.expected_names:
            return
        position = np.asarray(position, dtype=float)
        if position.shape != (3,) or not np.all(np.isfinite(position)):
            raise ValueError("position must be a finite three-dimensional vector")
        if timestamp is None:
            timestamp = time.monotonic()
        self._detections[name] = (
            float(timestamp),
            float(confidence),
            position.copy(),
        )

    def snapshot(self, timestamp=None):
        if timestamp is None:
            timestamp = time.monotonic()
        expired = [
            name
            for name, (seen_at, _confidence, _position) in self._detections.items()
            if timestamp - seen_at > self.ttl_seconds
        ]
        for name in expired:
            del self._detections[name]

        return {
            name: {
                "confidence": round(self._detections[name][1], 3),
                "position": [round(float(value), 4) for value in self._detections[name][2]],
            }
            for name in self.expected_names
            if name in self._detections
        }

    def missing_names(self, timestamp=None):
        snapshot = self.snapshot(timestamp)
        return [name for name in self.expected_names if name not in snapshot]


class StableDetectionStore:
    def __init__(
        self,
        *,
        window_size=10,
        min_samples=5,
        ttl_seconds=1.0,
        min_confidence=0.6,
        max_position_std=0.02,
        outlier_distance=0.05,
        expected_names=EXPECTED_BLOCK_NAMES,
    ):
        if window_size <= 0:
            raise ValueError("window_size must be positive")
        if not 1 <= min_samples <= window_size:
            raise ValueError("min_samples must be between 1 and window_size")
        if ttl_seconds <= 0.0 or max_position_std <= 0.0 or outlier_distance <= 0.0:
            raise ValueError("ttl and position thresholds must be positive")
        if not 0.0 <= min_confidence <= 1.0:
            raise ValueError("min_confidence must be between 0 and 1")

        self.window_size = int(window_size)
        self.min_samples = int(min_samples)
        self.ttl_seconds = float(ttl_seconds)
        self.min_confidence = float(min_confidence)
        self.max_position_std = float(max_position_std)
        self.outlier_distance = float(outlier_distance)
        self.expected_names = tuple(expected_names)
        self._samples = {
            name: deque(maxlen=self.window_size)
            for name in self.expected_names
        }

    def _prune(self, timestamp):
        for samples in self._samples.values():
            while samples and timestamp - samples[0][0] > self.ttl_seconds:
                samples.popleft()

    def update(self, name, confidence, position, timestamp=None):
        if name not in self._samples or confidence < self.min_confidence:
            return False
        position = np.asarray(position, dtype=float)
        if position.shape != (3,) or not np.all(np.isfinite(position)):
            raise ValueError("position must be a finite three-dimensional vector")
        if timestamp is None:
            timestamp = time.monotonic()
        timestamp = float(timestamp)
        self._prune(timestamp)

        samples = self._samples[name]
        if len(samples) >= self.min_samples:
            center = np.median(
                np.asarray([sample_position for _seen, _confidence, sample_position in samples]),
                axis=0,
            )
            if np.linalg.norm(position - center) > self.outlier_distance:
                return False

        samples.append((timestamp, float(confidence), position.copy()))
        return True

    def snapshot(self, timestamp=None):
        if timestamp is None:
            timestamp = time.monotonic()
        timestamp = float(timestamp)
        self._prune(timestamp)
        result = {}

        for name in self.expected_names:
            samples = self._samples[name]
            if len(samples) < self.min_samples:
                continue
            positions = np.asarray(
                [sample_position for _seen, _confidence, sample_position in samples]
            )
            position_std = np.std(positions, axis=0)
            if np.any(position_std > self.max_position_std):
                continue
            confidences = [confidence for _seen, confidence, _position in samples]
            result[name] = {
                "confidence": round(float(np.mean(confidences)), 3),
                "position": [
                    round(float(value), 4)
                    for value in np.median(positions, axis=0)
                ],
                "sample_count": len(samples),
                "position_std": [round(float(value), 5) for value in position_std],
            }
        return result

    def compact_snapshot(self, timestamp=None):
        return {
            name: {
                "confidence": info["confidence"],
                "position": info["position"],
            }
            for name, info in self.snapshot(timestamp).items()
        }

    def missing_names(self, timestamp=None):
        snapshot = self.snapshot(timestamp)
        return [name for name in self.expected_names if name not in snapshot]


def write_vision_scene(path, objects, *, updated_at=None):
    path = Path(path)
    if not objects:
        raise ValueError("vision scene requires at least one detected block")
    unknown_names = sorted(set(objects) - set(EXPECTED_BLOCK_NAMES))
    if unknown_names:
        raise ValueError(f"vision scene contains unknown blocks: {', '.join(unknown_names)}")
    if updated_at is None:
        updated_at = time.time()

    payload = {
        "updated_at": float(updated_at),
        "frame": "world",
        "objects": objects,
    }
    tmp_path = Path(f"{path}.tmp")
    with tmp_path.open("w", encoding="utf-8") as file:
        json.dump(payload, file, ensure_ascii=False, indent=2)
    os.replace(tmp_path, path)


def detections_to_scene_objects(detections: dict[str, Detection]) -> dict:
    return {
        name: {
            "position": np.array(detection.position, dtype=float).copy(),
        }
        for name, detection in detections.items()
    }
