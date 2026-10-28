from pathlib import Path

import pytest
import yaml


PACKAGE_ROOT = Path(__file__).parents[1]
REPOSITORY_ROOT = Path(__file__).parents[4]
JOINT_LIMITS = PACKAGE_ROOT / "config" / "joint_limits.yaml"
PHYSICS_CONFIG = (
    REPOSITORY_ROOT
    / "ros2_ur_ws"
    / "src"
    / "pallet"
    / "config"
    / "isaac_gripperless_physics_test.json"
)

# Conservative gravity envelopes from deterministic full-range URDF sampling.
# The test reserves at least 20% of continuous torque for acceleration and
# position-error correction after gravity and velocity damping are accounted for.
GRAVITY_ENVELOPE_NM = {
    "j1": 0.257,
    "j2": 5.505,
    "j3": 1.989,
    "j4": 0.068,
    "j5": 0.068,
}
CONTINUOUS_EFFORT_NM = {"j1": 5.0, "j2": 11.0, "j3": 5.0, "j4": 1.8, "j5": 1.8}


def test_operational_limits_preserve_continuous_torque_margin():
    limits = yaml.safe_load(JOINT_LIMITS.read_text(encoding="utf-8"))
    physics = yaml.safe_load(PHYSICS_CONFIG.read_text(encoding="utf-8"))

    assert limits["default_velocity_scaling_factor"] == pytest.approx(1.0)
    assert limits["default_acceleration_scaling_factor"] == pytest.approx(1.0)

    for joint, joint_limit in limits["joint_limits"].items():
        assert joint_limit["has_velocity_limits"] is True
        assert joint_limit["has_acceleration_limits"] is True
        damping = physics["joints"][joint]["damping"]
        continuous_demand = (
            GRAVITY_ENVELOPE_NM[joint]
            + damping * joint_limit["max_velocity"]
        )
        assert continuous_demand <= 0.8 * CONTINUOUS_EFFORT_NM[joint]


def test_physics_test_profiles_stay_inside_operational_motion_limits():
    limits = yaml.safe_load(JOINT_LIMITS.read_text(encoding="utf-8"))["joint_limits"]
    physics = yaml.safe_load(PHYSICS_CONFIG.read_text(encoding="utf-8"))
    previous = {joint: 0.0 for joint in limits}

    for phase in physics["phases"]:
        duration = float(phase["duration_seconds"])
        target = phase["target_rad"]
        for joint, joint_limit in limits.items():
            distance = abs(float(target[joint]) - previous[joint])
            smoothstep_peak_velocity = 1.5 * distance / duration
            smoothstep_peak_acceleration = 6.0 * distance / (duration * duration)
            assert smoothstep_peak_velocity <= joint_limit["max_velocity"]
            assert smoothstep_peak_acceleration <= joint_limit["max_acceleration"]
        previous = {joint: float(target[joint]) for joint in limits}
