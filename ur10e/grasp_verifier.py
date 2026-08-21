import json
import threading
import time
from dataclasses import dataclass
from pathlib import Path

import numpy as np


@dataclass(frozen=True)
class GraspSample:
    object_position: np.ndarray
    gripper_position: np.ndarray


@dataclass(frozen=True)
class GraspVerification:
    success: bool
    reason: str
    sample_count: int
    lifted_sample_count: int
    max_lift: float
    max_relative_deviation: float | None


def read_grasp_sample(path: Path, object_name: str) -> GraspSample:
    data = json.loads(path.read_text(encoding="utf-8"))
    objects = data.get("objects")
    if not isinstance(objects, list):
        raise ValueError("Isaac scene state must contain an object list")

    object_item = next(
        (item for item in objects if isinstance(item, dict) and item.get("name") == object_name),
        None,
    )
    if object_item is None:
        raise ValueError(f"Isaac scene state does not contain {object_name}")

    gripper = data.get("gripper")
    if not isinstance(gripper, dict):
        raise ValueError("Isaac scene state does not contain gripper pose")

    object_position = np.asarray(object_item.get("position"), dtype=float)
    gripper_position = np.asarray(gripper.get("position"), dtype=float)
    if object_position.shape != (3,) or not np.all(np.isfinite(object_position)):
        raise ValueError(f"invalid object position for {object_name}")
    if gripper_position.shape != (3,) or not np.all(np.isfinite(gripper_position)):
        raise ValueError("invalid gripper position")

    return GraspSample(object_position=object_position, gripper_position=gripper_position)


def evaluate_grasp_samples(
    samples: list[GraspSample],
    initial_object_position,
    *,
    min_lift: float = 0.03,
    relative_tolerance: float = 0.03,
    min_lifted_samples: int = 2,
) -> GraspVerification:
    if min_lift <= 0.0:
        raise ValueError("min_lift must be positive")
    if relative_tolerance <= 0.0:
        raise ValueError("relative_tolerance must be positive")
    if min_lifted_samples < 2:
        raise ValueError("min_lifted_samples must be at least 2")

    initial_position = np.asarray(initial_object_position, dtype=float)
    if initial_position.shape != (3,) or not np.all(np.isfinite(initial_position)):
        raise ValueError("initial_object_position must be a finite XYZ vector")

    if not samples:
        return GraspVerification(False, "no Isaac pose samples were received", 0, 0, 0.0, None)

    lifts = [float(sample.object_position[2] - initial_position[2]) for sample in samples]
    max_lift = max(lifts)
    lifted_samples = [sample for sample, lift in zip(samples, lifts) if lift >= min_lift]
    if len(lifted_samples) < min_lifted_samples:
        return GraspVerification(
            False,
            f"object did not remain lifted by at least {min_lift:.3f} m",
            len(samples),
            len(lifted_samples),
            max_lift,
            None,
        )

    # Only validate the contiguous interval in which the object is being
    # carried.  Samples after release must not invalidate a successful grasp:
    # when placing on another block the object remains elevated while the
    # gripper retreats, so their relative pose intentionally changes.
    stable_runs: list[list[GraspSample]] = []
    current_run: list[GraspSample] = []
    for sample, lift in zip(samples, lifts):
        if lift < min_lift:
            current_run = []
            continue

        candidate_run = [*current_run, sample]
        candidate_relative_positions = np.asarray(
            [item.object_position - item.gripper_position for item in candidate_run],
            dtype=float,
        )
        candidate_center = np.mean(candidate_relative_positions, axis=0)
        candidate_deviation = float(
            np.max(np.linalg.norm(candidate_relative_positions - candidate_center, axis=1))
        )
        if current_run and candidate_deviation > relative_tolerance:
            current_run = [sample]
        else:
            current_run = candidate_run
        stable_runs.append(current_run.copy())

    minimum_carry_motion = min_lift * 0.5
    carried_runs = []
    for run in stable_runs:
        if len(run) < min_lifted_samples:
            continue
        object_positions = np.asarray([item.object_position for item in run], dtype=float)
        object_motion = float(
            np.max(np.linalg.norm(object_positions - object_positions[0], axis=1))
        )
        if object_motion >= minimum_carry_motion:
            carried_runs.append(run)

    if not carried_runs:
        relative_positions = np.asarray(
            [sample.object_position - sample.gripper_position for sample in lifted_samples],
            dtype=float,
        )
        relative_center = np.mean(relative_positions, axis=0)
        max_relative_deviation = float(
            np.max(np.linalg.norm(relative_positions - relative_center, axis=1))
        )
        return GraspVerification(
            False,
            "object did not move through a stable carried interval with the gripper",
            len(samples),
            len(lifted_samples),
            max_lift,
            max_relative_deviation,
        )

    carried_run = max(carried_runs, key=len)
    carried_relative_positions = np.asarray(
        [sample.object_position - sample.gripper_position for sample in carried_run],
        dtype=float,
    )
    carried_center = np.mean(carried_relative_positions, axis=0)
    max_relative_deviation = float(
        np.max(np.linalg.norm(carried_relative_positions - carried_center, axis=1))
    )

    return GraspVerification(
        True,
        "object lifted and moved through a stable carried interval with the gripper",
        len(samples),
        len(carried_run),
        max_lift,
        max_relative_deviation,
    )


class GraspMonitor:
    def __init__(
        self,
        path: str | Path,
        object_name: str,
        initial_object_position,
        *,
        poll_period: float = 0.05,
        min_lift: float = 0.03,
        relative_tolerance: float = 0.03,
        min_lifted_samples: int = 2,
    ) -> None:
        self.path = Path(path)
        self.object_name = object_name
        self.initial_object_position = np.asarray(initial_object_position, dtype=float).copy()
        self.poll_period = poll_period
        self.min_lift = min_lift
        self.relative_tolerance = relative_tolerance
        self.min_lifted_samples = min_lifted_samples
        self.samples: list[GraspSample] = []
        self.last_error: str | None = None
        self._last_mtime_ns: int | None = None
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None

    def start(self) -> None:
        if self._thread is not None:
            raise RuntimeError("grasp monitor is already running")
        self._capture_if_updated()
        self._thread = threading.Thread(target=self._run, name="isaac-grasp-monitor", daemon=True)
        self._thread.start()

    def stop(self) -> GraspVerification:
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=max(1.0, self.poll_period * 4.0))
        initial_object_position = (
            self.samples[0].object_position
            if self.samples
            else self.initial_object_position
        )
        result = evaluate_grasp_samples(
            self.samples,
            initial_object_position,
            min_lift=self.min_lift,
            relative_tolerance=self.relative_tolerance,
            min_lifted_samples=self.min_lifted_samples,
        )
        if not result.success and self.last_error and not self.samples:
            return GraspVerification(
                False,
                self.last_error,
                result.sample_count,
                result.lifted_sample_count,
                result.max_lift,
                result.max_relative_deviation,
            )
        return result

    def _run(self) -> None:
        while not self._stop_event.is_set():
            self._capture_if_updated()
            self._stop_event.wait(self.poll_period)

    def _capture_if_updated(self) -> None:
        try:
            stat = self.path.stat()
            if stat.st_mtime_ns != self._last_mtime_ns:
                self.samples.append(read_grasp_sample(self.path, self.object_name))
                self._last_mtime_ns = stat.st_mtime_ns
                self.last_error = None
        except (OSError, ValueError, json.JSONDecodeError) as error:
            self.last_error = str(error)
