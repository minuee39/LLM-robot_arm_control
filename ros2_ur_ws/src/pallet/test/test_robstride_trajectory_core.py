#!/usr/bin/env python3

import importlib.util
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest


MODULE_PATH = Path(__file__).parents[1] / "scripts" / "robstride_trajectory_core.py"
SPEC = importlib.util.spec_from_file_location("robstride_trajectory_core", MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


def duration(seconds: float):
    whole = int(seconds)
    return SimpleNamespace(sec=whole, nanosec=int((seconds - whole) * 1e9))


class RobStrideTrajectoryCoreTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.config_path = Path(__file__).parents[1] / "config" / "robstride_hardware.yaml"

    def test_live_mode_is_locked_before_commissioning(self):
        with self.assertRaisesRegex(MODULE.ConfigurationError, "commissioned"):
            MODULE.load_hardware_config(self.config_path, require_commissioned=True)

    def test_joint_motor_transform_round_trip(self):
        config = MODULE.load_hardware_config(self.config_path, require_commissioned=False)
        self.assertEqual([joint.motor_id for joint in config.joints], [1, 2, 3, 4, 5])
        joint = config.joints[0]
        self.assertAlmostEqual(joint.joint_from_motor(joint.motor_from_joint(0.42)), 0.42)

    def test_protocol_payload_matches_existing_controller(self):
        self.assertEqual(MODULE.arbitration_id(MODULE.TYPE_WRITE_PARAM, 0xFD, 2), 0x1200FD02)
        self.assertEqual(MODULE.write_float_payload(MODULE.PARAM_LOC_REF, 1.0).hex(), "167000000000803f")

    def test_valid_trajectory_is_accepted_in_any_joint_order(self):
        config = MODULE.load_hardware_config(self.config_path, require_commissioned=False)
        names = ["j3", "j1", "j5", "j2", "j4"]
        points = [
            SimpleNamespace(time_from_start=duration(0.0), positions=[0.0] * 5),
            SimpleNamespace(time_from_start=duration(2.0), positions=[0.1] * 5),
        ]
        self.assertEqual(len(MODULE.validate_trajectory(names, points, config)), 2)

    def test_limit_and_segment_velocity_are_rejected(self):
        config = MODULE.load_hardware_config(self.config_path, require_commissioned=False)
        names = [joint.name for joint in config.joints]
        bad_limit = [
            SimpleNamespace(time_from_start=duration(0.0), positions=[0.0] * 5),
            SimpleNamespace(time_from_start=duration(1.0), positions=[0.0, 2.0, 0.0, 0.0, 0.0]),
        ]
        with self.assertRaisesRegex(MODULE.TrajectoryError, "outside"):
            MODULE.validate_trajectory(names, bad_limit, config)
        too_fast = [
            SimpleNamespace(time_from_start=duration(0.0), positions=[0.0] * 5),
            SimpleNamespace(time_from_start=duration(0.1), positions=[0.1, 0.0, 0.0, 0.0, 0.0]),
        ]
        with self.assertRaisesRegex(MODULE.TrajectoryError, "velocity"):
            MODULE.validate_trajectory(names, too_fast, config)


if __name__ == "__main__":
    unittest.main()
