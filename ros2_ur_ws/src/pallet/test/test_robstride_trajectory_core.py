#!/usr/bin/env python3

import importlib.util
from dataclasses import replace
from pathlib import Path
import struct
import sys
import tempfile
from types import SimpleNamespace
import unittest
import xml.etree.ElementTree as ET

import yaml


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
        cls.config_path = Path(__file__).parents[1] / "config" / "robstride_hardware_legacy_5axis.yaml"

    def test_live_mode_is_locked_before_commissioning(self):
        with self.assertRaisesRegex(MODULE.ConfigurationError, "commissioned"):
            MODULE.load_hardware_config(self.config_path, require_commissioned=True)

    def test_invalid_calibration_values_are_rejected_before_can(self):
        document = yaml.safe_load(self.config_path.read_text(encoding="utf-8"))
        for field, value, message in (
            ("motor_zero_rad", float("nan"), "motor_zero_rad must be finite"),
            ("motor_id", 1.5, "motor_id must be an integer"),
            ("direction", 1.5, "direction must be -1 or 1"),
        ):
            with self.subTest(field=field), tempfile.TemporaryDirectory() as directory:
                modified = yaml.safe_load(self.config_path.read_text(encoding="utf-8"))
                modified["joints"]["j1"][field] = value
                path = Path(directory) / "hardware.yaml"
                path.write_text(yaml.safe_dump(modified), encoding="utf-8")
                with self.assertRaisesRegex(MODULE.ConfigurationError, message):
                    MODULE.load_hardware_config(path, require_commissioned=False)
        document["host_id"] = 256
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "hardware.yaml"
            path.write_text(yaml.safe_dump(document), encoding="utf-8")
            with self.assertRaisesRegex(MODULE.ConfigurationError, "host_id must be an integer"):
                MODULE.load_hardware_config(path, require_commissioned=False)

    def test_joint_motor_transform_round_trip(self):
        config = MODULE.load_hardware_config(self.config_path, require_commissioned=False)
        self.assertEqual([joint.motor_id for joint in config.joints], [1, 2, 3, 4, 5])
        joint = config.joints[0]
        self.assertAlmostEqual(joint.joint_from_motor(joint.motor_from_joint(0.42)), 0.42)
        with self.assertRaisesRegex(RuntimeError, "motor command is not finite"):
            replace(joint, transmission_ratio=1e300).motor_from_joint(1e300)
        with self.assertRaisesRegex(RuntimeError, "measured joint position is not finite"):
            replace(joint, transmission_ratio=1e-300).joint_from_motor(1e300)

    def test_hardware_joint_envelope_matches_planning_model(self):
        package_root = Path(__file__).parents[1]
        workspace_src = package_root.parent
        config = MODULE.load_hardware_config(
            self.config_path, require_commissioned=False)
        model = ET.parse(package_root / "urdf" / "pallet_gripperless.urdf").getroot()
        model_limits = {
            joint.attrib["name"]: joint.find("limit").attrib
            for joint in model.findall("joint") if joint.find("limit") is not None
        }
        planning = yaml.safe_load((
            workspace_src / "pallet_moveit_config" / "config" / "joint_limits.yaml"
        ).read_text(encoding="utf-8"))["joint_limits"]
        self.assertEqual(set(planning), {joint.name for joint in config.joints})
        for joint in config.joints:
            with self.subTest(joint=joint.name):
                self.assertEqual(joint.lower_rad, float(model_limits[joint.name]["lower"]))
                self.assertEqual(joint.upper_rad, float(model_limits[joint.name]["upper"]))
                self.assertTrue(planning[joint.name]["has_velocity_limits"])
                self.assertTrue(planning[joint.name]["has_acceleration_limits"])
                self.assertEqual(
                    joint.max_velocity_rad_s, planning[joint.name]["max_velocity"])
                self.assertEqual(
                    joint.max_acceleration_rad_s2, planning[joint.name]["max_acceleration"])

    def test_protocol_payload_matches_existing_controller(self):
        self.assertEqual(MODULE.arbitration_id(MODULE.TYPE_WRITE_PARAM, 0xFD, 2), 0x1200FD02)
        self.assertEqual(MODULE.write_float_payload(MODULE.PARAM_LOC_REF, 1.0).hex(), "167000000000803f")

    def test_nonfinite_motor_feedback_is_rejected(self):
        motor_id = 2
        response = SimpleNamespace(
            is_extended_id=True,
            arbitration_id=(MODULE.TYPE_READ_PARAM << 24) | (motor_id << 8),
            data=MODULE.PARAM_MECH_POS.to_bytes(2, "little") + bytes(2)
            + struct.pack("<f", float("nan")),
        )
        bus = MODULE.RobStrideBus.__new__(MODULE.RobStrideBus)
        bus._bus = SimpleNamespace(recv=lambda timeout: response)
        bus._send = lambda *_args: None
        with self.assertRaisesRegex(RuntimeError, "non-finite position feedback"):
            bus.read_motor_positions([motor_id], timeout=0.1)

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
