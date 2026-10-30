from pathlib import Path
import math
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / 'scripts'))
from analyze_pallet_transfer import measured_kinematics, norm, rotation_vector


def samples(gap=False):
    rows = []
    for i in range(21):
        t = i*0.1 + (1 if gap and i >= 10 else 0)
        angle = t*0.2
        rows.append({'seconds': t, 'tcp_measurement': {
            'position_m': [t*0.3, 0, 0],
            'orientation_xyzw': [0, 0, math.sin(angle/2), math.cos(angle/2)],
        }})
    return rows


class TransferAnalysisTest(unittest.TestCase):
    def test_constant_velocity_has_zero_acceleration(self):
        velocity, acceleration = measured_kinematics(samples())
        self.assertGreater(len(acceleration), 0)
        for item in velocity:
            self.assertAlmostEqual(norm(item['linear_m_s']), 0.3)
            self.assertAlmostEqual(norm(item['angular_rad_s']), 0.2)
        for item in acceleration:
            self.assertLess(norm(item['linear_m_s2']), 1e-10)
            self.assertLess(norm(item['angular_rad_s2']), 1e-10)

    def test_quaternion_sign_does_not_create_rotation(self):
        self.assertEqual(rotation_vector([0, 0, 0, -1]), [0, 0, 0])
        with self.assertRaises(ValueError):
            rotation_vector([0, 0, 0, 0])

    def test_gap_not_bridged(self):
        velocity, _ = measured_kinematics(samples(gap=True))
        self.assertTrue(all(not 0.91 < x['seconds'] < 2.2 for x in velocity))


if __name__ == '__main__':
    unittest.main()
