"""Contract tests for the symmetric 67 mm parallel-jaw proxy."""

import unittest
import xml.etree.ElementTree as ET

from gripper_calibration import GripperGeometry, URDF


class GripperCalibrationTest(unittest.TestCase):
    def test_prismatic_mimic_and_clearance(self):
        root = ET.parse(URDF).getroot()
        right = root.find("joint[@name='J8']")
        left = root.find("joint[@name='J9']")
        self.assertEqual('prismatic', right.get('type'))
        self.assertEqual('prismatic', left.get('type'))
        self.assertEqual('J8', left.find('mimic').get('joint'))
        self.assertEqual('1', left.find('mimic').get('multiplier'))

        geometry = GripperGeometry()
        self.assertAlmostEqual(0.0, geometry.lower)
        self.assertAlmostEqual(0.0335, geometry.upper)
        self.assertAlmostEqual(0.0, geometry.clearance(geometry.lower, 0.01), places=5)
        self.assertAlmostEqual(0.067, geometry.clearance(geometry.upper, 0.01), places=5)
        self.assertAlmostEqual(0.015, geometry.position_for_clearance(0.03, 0.01), places=5)

    def test_measured_opening_endpoints(self):
        geometry = GripperGeometry()
        self.assertEqual(0.0, geometry.position_for_clearance(0.0, 0.01))
        self.assertEqual(0.0335, geometry.position_for_clearance(0.067, 0.01))

    def test_out_of_range_and_nonfinite_openings(self):
        geometry = GripperGeometry()
        for target in (-0.001, 0.068, float('nan'), float('-inf'), float('inf')):
            with self.subTest(target=target), self.assertRaises(ValueError):
                geometry.position_for_clearance(target, 0.01)


if __name__ == '__main__':
    unittest.main()
