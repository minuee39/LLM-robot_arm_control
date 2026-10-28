#!/usr/bin/env python3
"""Pure configuration, validation, and RobStride protocol helpers."""

from __future__ import annotations

from dataclasses import dataclass
import math
import struct
import time
from pathlib import Path
from typing import Any, Iterable, Mapping

import yaml


TYPE_ENABLE = 0x03
TYPE_STOP = 0x04
TYPE_READ_PARAM = 0x11
TYPE_WRITE_PARAM = 0x12
PARAM_RUN_MODE = 0x7005
PARAM_LOC_REF = 0x7016
PARAM_LIMIT_CUR = 0x7018
PARAM_MECH_POS = 0x7019
PARAM_VEL_MAX = 0x7024
PARAM_ACC_SET = 0x7025
RUN_MODE_POSITION_PP = 1


class ConfigurationError(ValueError):
    """Raised when a hardware configuration is unsafe or incomplete."""


class TrajectoryError(ValueError):
    """Raised when a trajectory violates the configured execution envelope."""


@dataclass(frozen=True)
class JointConfig:
    name: str
    motor_id: int
    direction: int
    motor_zero_rad: float
    transmission_ratio: float
    lower_rad: float
    upper_rad: float
    max_velocity_rad_s: float
    max_acceleration_rad_s2: float
    current_limit_a: float

    def motor_from_joint(self, joint_rad: float) -> float:
        return self.motor_zero_rad + self.direction * joint_rad * self.transmission_ratio

    def joint_from_motor(self, motor_rad: float) -> float:
        return self.direction * (motor_rad - self.motor_zero_rad) / self.transmission_ratio


@dataclass(frozen=True)
class HardwareConfig:
    commissioned: bool
    interface: str
    host_id: int
    command_period_s: float
    feedback_period_s: float
    feedback_timeout_s: float
    max_start_error_rad: float
    max_tracking_error_rad: float
    tracking_error_grace_s: float
    goal_tolerance_rad: float
    settle_timeout_s: float
    joints: tuple[JointConfig, ...]


def _finite_positive(value: Any, field: str) -> float:
    number = float(value)
    if not math.isfinite(number) or number <= 0.0:
        raise ConfigurationError(f"{field} must be finite and greater than zero")
    return number


def load_hardware_config(path: str | Path, *, require_commissioned: bool) -> HardwareConfig:
    with Path(path).open("r", encoding="utf-8") as stream:
        document = yaml.safe_load(stream)
    if not isinstance(document, Mapping):
        raise ConfigurationError("configuration root must be a mapping")
    commissioned = document.get("commissioned") is True
    if require_commissioned and not commissioned:
        raise ConfigurationError(
            "hardware execution is locked: verify motor IDs, directions, zero offsets, "
            "and limits, then set commissioned: true"
        )
    raw_joints = document.get("joints")
    if not isinstance(raw_joints, Mapping) or not raw_joints:
        raise ConfigurationError("joints must be a non-empty mapping")

    joints: list[JointConfig] = []
    motor_ids: set[int] = set()
    for name, raw in raw_joints.items():
        if not isinstance(name, str) or not isinstance(raw, Mapping):
            raise ConfigurationError("each joint entry must be a named mapping")
        motor_id = int(raw.get("motor_id", 0))
        if not 1 <= motor_id <= 255:
            raise ConfigurationError(f"{name}.motor_id must be in 1..255")
        if motor_id in motor_ids:
            raise ConfigurationError(f"duplicate motor_id: {motor_id}")
        motor_ids.add(motor_id)
        direction = int(raw.get("direction", 0))
        if direction not in (-1, 1):
            raise ConfigurationError(f"{name}.direction must be -1 or 1")
        lower = float(raw["lower_rad"])
        upper = float(raw["upper_rad"])
        if not math.isfinite(lower) or not math.isfinite(upper) or lower >= upper:
            raise ConfigurationError(f"{name} has invalid joint limits")
        joints.append(
            JointConfig(
                name=name,
                motor_id=motor_id,
                direction=direction,
                motor_zero_rad=float(raw["motor_zero_rad"]),
                transmission_ratio=_finite_positive(raw.get("transmission_ratio", 1.0), f"{name}.transmission_ratio"),
                lower_rad=lower,
                upper_rad=upper,
                max_velocity_rad_s=_finite_positive(raw["max_velocity_rad_s"], f"{name}.max_velocity_rad_s"),
                max_acceleration_rad_s2=_finite_positive(raw["max_acceleration_rad_s2"], f"{name}.max_acceleration_rad_s2"),
                current_limit_a=_finite_positive(raw["current_limit_a"], f"{name}.current_limit_a"),
            )
        )
    return HardwareConfig(
        commissioned=commissioned,
        interface=str(document.get("interface", "can0")),
        host_id=int(document.get("host_id", 0xFD)),
        command_period_s=_finite_positive(document.get("command_period_s", 0.02), "command_period_s"),
        feedback_period_s=_finite_positive(document.get("feedback_period_s", 0.05), "feedback_period_s"),
        feedback_timeout_s=_finite_positive(document.get("feedback_timeout_s", 0.1), "feedback_timeout_s"),
        max_start_error_rad=_finite_positive(document.get("max_start_error_rad", 0.15), "max_start_error_rad"),
        max_tracking_error_rad=_finite_positive(document.get("max_tracking_error_rad", 0.35), "max_tracking_error_rad"),
        tracking_error_grace_s=_finite_positive(document.get("tracking_error_grace_s", 0.5), "tracking_error_grace_s"),
        goal_tolerance_rad=_finite_positive(document.get("goal_tolerance_rad", 0.05), "goal_tolerance_rad"),
        settle_timeout_s=_finite_positive(document.get("settle_timeout_s", 2.0), "settle_timeout_s"),
        joints=tuple(joints),
    )


def duration_seconds(duration: Any) -> float:
    return float(duration.sec) + float(duration.nanosec) * 1e-9


def validate_trajectory(joint_names: Iterable[str], points: Iterable[Any], config: HardwareConfig) -> list[tuple[float, list[float]]]:
    names = list(joint_names)
    expected = [joint.name for joint in config.joints]
    if len(names) != len(expected) or set(names) != set(expected):
        raise TrajectoryError(f"expected exactly {tuple(expected)}; received {tuple(names)}")
    by_name = {joint.name: joint for joint in config.joints}
    ordered = [by_name[name] for name in names]
    validated: list[tuple[float, list[float]]] = []
    previous_time = -1.0
    previous_positions: list[float] | None = None
    for index, point in enumerate(points):
        target_time = duration_seconds(point.time_from_start)
        positions = [float(value) for value in point.positions]
        if len(positions) != len(names):
            raise TrajectoryError(f"point {index} has {len(positions)} positions; expected {len(names)}")
        if not math.isfinite(target_time) or target_time < 0.0 or target_time <= previous_time:
            raise TrajectoryError("time_from_start must be finite and strictly increasing")
        for joint, position in zip(ordered, positions):
            if not math.isfinite(position):
                raise TrajectoryError(f"{joint.name} position is not finite")
            if position < joint.lower_rad or position > joint.upper_rad:
                raise TrajectoryError(f"{joint.name} target {position:.6f} is outside [{joint.lower_rad:.6f}, {joint.upper_rad:.6f}] rad")
        if previous_positions is not None:
            delta_t = target_time - previous_time
            for joint, before, after in zip(ordered, previous_positions, positions):
                velocity = abs(after - before) / delta_t
                if velocity > joint.max_velocity_rad_s * 1.01:
                    raise TrajectoryError(f"{joint.name} segment velocity {velocity:.6f} exceeds {joint.max_velocity_rad_s:.6f} rad/s")
        validated.append((target_time, positions))
        previous_time = target_time
        previous_positions = positions
    if not validated:
        raise TrajectoryError("trajectory is empty")
    return validated


def arbitration_id(comm_type: int, host_id: int, motor_id: int) -> int:
    return ((comm_type & 0x1F) << 24) | ((host_id & 0xFFFF) << 8) | (motor_id & 0xFF)


def write_u8_payload(index: int, value: int) -> bytes:
    return index.to_bytes(2, "little") + b"\x00\x00" + bytes((value & 0xFF, 0, 0, 0))


def write_float_payload(index: int, value: float) -> bytes:
    return index.to_bytes(2, "little") + b"\x00\x00" + struct.pack("<f", float(value))


def read_payload(index: int) -> bytes:
    return index.to_bytes(2, "little") + bytes(6)


class RobStrideBus:
    """Synchronous SocketCAN transport for the RobStride private protocol."""

    def __init__(self, interface: str, host_id: int):
        import can
        self._can = can
        self._bus = can.interface.Bus(channel=interface, interface="socketcan")
        self.host_id = host_id

    def _send(self, comm_type: int, motor_id: int, payload: bytes = bytes(8)) -> None:
        if len(payload) != 8:
            raise ValueError("RobStride payload must contain exactly 8 bytes")
        self._bus.send(self._can.Message(arbitration_id=arbitration_id(comm_type, self.host_id, motor_id), data=payload, is_extended_id=True), timeout=0.2)

    def stop(self, motor_id: int) -> None:
        self._send(TYPE_STOP, motor_id)

    def enable(self, motor_id: int) -> None:
        self._send(TYPE_ENABLE, motor_id)

    def write_u8(self, motor_id: int, index: int, value: int) -> None:
        self._send(TYPE_WRITE_PARAM, motor_id, write_u8_payload(index, value))

    def write_float(self, motor_id: int, index: int, value: float) -> None:
        self._send(TYPE_WRITE_PARAM, motor_id, write_float_payload(index, value))

    def configure_position_mode(self, joint: JointConfig, hold_joint_position: float) -> None:
        self.stop(joint.motor_id)
        time.sleep(0.03)
        self.write_u8(joint.motor_id, PARAM_RUN_MODE, RUN_MODE_POSITION_PP)
        self.write_float(joint.motor_id, PARAM_LIMIT_CUR, joint.current_limit_a)
        self.write_float(joint.motor_id, PARAM_VEL_MAX, joint.max_velocity_rad_s * joint.transmission_ratio)
        self.write_float(joint.motor_id, PARAM_ACC_SET, joint.max_acceleration_rad_s2 * joint.transmission_ratio)
        # Load the measured position before enabling.  Reusing a stale LOC_REF
        # here could make an axis jump as soon as PP mode becomes active.
        self.write_float(joint.motor_id, PARAM_LOC_REF, joint.motor_from_joint(hold_joint_position))
        self.enable(joint.motor_id)

    def command_joint_positions(self, joints: Iterable[JointConfig], joint_positions: Iterable[float]) -> None:
        for joint, position in zip(joints, joint_positions):
            self.write_float(joint.motor_id, PARAM_LOC_REF, joint.motor_from_joint(position))

    def read_motor_positions(self, motor_ids: Iterable[int], timeout: float) -> dict[int, float]:
        requested = set(motor_ids)
        for motor_id in requested:
            self._send(TYPE_READ_PARAM, motor_id, read_payload(PARAM_MECH_POS))
        positions: dict[int, float] = {}
        deadline = time.monotonic() + timeout
        expected_index = PARAM_MECH_POS.to_bytes(2, "little")
        while requested - positions.keys() and time.monotonic() < deadline:
            message = self._bus.recv(timeout=max(0.0, deadline - time.monotonic()))
            if message is None or not message.is_extended_id:
                continue
            comm_type = (message.arbitration_id >> 24) & 0x1F
            motor_id = (message.arbitration_id >> 8) & 0xFF
            data = bytes(message.data)
            if comm_type == TYPE_READ_PARAM and motor_id in requested and len(data) == 8 and data[:2] == expected_index:
                positions[motor_id] = struct.unpack("<f", data[4:8])[0]
        return positions

    def close(self) -> None:
        self._bus.shutdown()
