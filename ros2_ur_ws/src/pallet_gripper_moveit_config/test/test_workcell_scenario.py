from pathlib import Path
import sys
import unittest
import xml.etree.ElementTree as ET

import yaml


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'scripts'))
from workcell_scenario import (
    block_scenario, gripper_effort_limit, gripper_symmetry_config,
    symmetric_jaw_targets,
)


class BlockScenarioTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.workcell = yaml.safe_load((ROOT / 'config/workcell.yaml').read_text())
        cls.physics = yaml.safe_load((ROOT / 'config/isaac_physics.yaml').read_text())

    def test_defaults_and_overrides_do_not_mutate_source(self):
        original = self.workcell['block'].copy()
        self.assertEqual(original, block_scenario(self.workcell))
        result = block_scenario(self.workcell, (-0.27, 0.28), (0.03, 0.025, 0.05), 0.08)
        self.assertEqual([-0.27, 0.28], result['center_xy_base'])
        self.assertEqual([0.03, 0.025, 0.05], result['size'])
        self.assertEqual(0.08, result['mass_kg'])
        self.assertEqual(original, self.workcell['block'])

    def test_rejects_unreachable_physical_scenarios(self):
        for kwargs in (
            {'center_xy': (float('nan'), 0.3)},
            {'center_xy': (-0.3, 0.37)},
            {'size': (0.03, 0.067, 0.05)},
            {'size': (0.03, 0.03, 0)},
            {'mass_kg': -0.01},
        ):
            with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):
                block_scenario(self.workcell, **kwargs)

    def test_isaac_gripper_effort_candidate_is_sixty_newtons_per_finger(self):
        self.assertEqual(60.0, gripper_effort_limit(self.physics))

    def test_rejects_invalid_gripper_effort_limits(self):
        for value in (None, True, 0, -1, float('nan'), float('inf'), '60'):
            with self.subTest(value=value), self.assertRaises(ValueError):
                gripper_effort_limit({'gripper_effort_limit_n_per_finger': value})

    def test_gripper_symmetry_recenters_asymmetric_contact(self):
        gain, maximum = gripper_symmetry_config(self.physics)
        self.assertEqual((1.0, 0.005), (gain, maximum))
        targets = symmetric_jaw_targets(
            0.002, 0.017, 0.013, 0.0, 0.0335, gain, maximum)
        self.assertAlmostEqual(0.0, targets[0])
        self.assertAlmostEqual(0.004, targets[1])
        self.assertAlmostEqual(0.004, targets[1] - targets[0])

    def test_gripper_symmetry_clamps_targets_and_rejects_invalid_values(self):
        self.assertEqual(
            (0.0, 0.007),
            symmetric_jaw_targets(0.002, 0.030, 0.0, 0.0, 0.0335, 1.0, 0.005),
        )
        for values in (
            (0.04, 0.01, 0.01, 0.0, 0.0335, 0.5, 0.005),
            (0.01, float('nan'), 0.01, 0.0, 0.0335, 0.5, 0.005),
            (0.01, 0.01, 0.01, 0.0, 0.0335, 0.0, 0.005),
        ):
            with self.subTest(values=values), self.assertRaises(ValueError):
                symmetric_jaw_targets(*values)

    def test_public_urdf_effort_contract_remains_ten_newtons(self):
        source = ROOT.parent / 'pallet_gripper/urdf/pallet_gripper.urdf'
        tree = ET.parse(source)
        for name in ('J8', 'J9'):
            joint = tree.getroot().find(f"joint[@name='{name}']")
            self.assertIsNotNone(joint)
            self.assertEqual(10.0, float(joint.find('limit').get('effort')))


if __name__ == '__main__':
    unittest.main()
