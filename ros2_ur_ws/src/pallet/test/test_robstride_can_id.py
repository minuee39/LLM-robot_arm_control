#!/usr/bin/env python3

import importlib.util
import sys
import unittest
from pathlib import Path


SCRIPT = Path(__file__).parents[1] / "scripts" / "robstride_can_id.py"
SPEC = importlib.util.spec_from_file_location("robstride_can_id", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


class RobStrideFrameTest(unittest.TestCase):
    def test_get_device_id_frame(self):
        self.assertEqual(MODULE.get_device_id_arbitration_id(0xFD, 0x7F), 0x0000FD7F)

    def test_set_can_id_frame_matches_protocol_example(self):
        self.assertEqual(MODULE.set_can_id_arbitration_id(0xFD, 1, 11), 0x070BFD01)

    def test_decode_device_identity(self):
        uid = bytes.fromhex("0102030405060708")
        raw = MODULE.CAN_FRAME.pack(
            MODULE.CAN_EFF_FLAG | 0x00007FFE,
            8,
            uid,
        )
        identity = MODULE.decode_device_identity(raw)
        self.assertEqual(identity, MODULE.DeviceIdentity(motor_id=0x7F, uid=uid))

    def test_ignore_standard_and_unrelated_extended_frames(self):
        standard = MODULE.CAN_FRAME.pack(0x7F, 8, bytes(8))
        feedback = MODULE.CAN_FRAME.pack(
            MODULE.CAN_EFF_FLAG | 0x02007FFD, 8, bytes(8)
        )
        self.assertIsNone(MODULE.decode_device_identity(standard))
        self.assertIsNone(MODULE.decode_device_identity(feedback))

    def test_encode_marks_frame_as_extended(self):
        encoded = MODULE.encode_can_frame(0x070BFD01, b"\x01\x02")
        can_id, dlc, payload = MODULE.CAN_FRAME.unpack(encoded)
        self.assertEqual(can_id, MODULE.CAN_EFF_FLAG | 0x070BFD01)
        self.assertEqual(dlc, 2)
        self.assertEqual(payload, b"\x01\x02" + bytes(6))


if __name__ == "__main__":
    unittest.main()
