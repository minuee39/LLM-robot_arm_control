from pathlib import Path
import sys
import unittest
import xml.etree.ElementTree as ET

import yaml


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'scripts'))
from workcell_scenario import (
    block_scenario, diagnostic_block_poses, gripper_effort_limit, gripper_symmetry_config, joint_state_snapshot,
    symmetric_jaw_targets,
)


class BlockScenarioTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.workcell = yaml.safe_load((ROOT / 'config/workcell.yaml').read_text())
        cls.physics = yaml.safe_load((ROOT / 'config/isaac_physics.yaml').read_text())

    def test_diagnostic_fixture_defaults_and_target_poses(self):
        self.assertEqual({}, diagnostic_block_poses(None, self.workcell))
        poses = diagnostic_block_poses([
            ['red_block', '.24', '-.28', '0'],
            ['green_block', '.28', '-.24', '1.5707963267948966'],
            ['blue_block', '.28', '-.28', '0'],
        ], self.workcell)
        self.assertEqual((.24, -.28, 0), poses['red_block'])
        self.assertAlmostEqual(1.5707963267948966, poses['green_block'][2])

    def test_diagnostic_fixture_rejects_invalid_objects_numbers_and_edges(self):
        for entries in (
            [['unknown', 0, 0, 0]], [['red_block', 0, 0, 0]]*2,
            [['red_block', 0, 0]], [['red_block', True, 0, 0]],
            [['red_block', 'nan', 0, 0]], [['red_block', 0, 'bad', 0]],
            [['red_block', 0, 0, 4]], [['red_block', .9, 0, 0]],
            [['red_block', 0, .36, .7853981633974483]],
        ):
            with self.subTest(entries=entries), self.assertRaises(ValueError):
                diagnostic_block_poses(entries, self.workcell)

    def test_defaults_and_overrides_do_not_mutate_source(self):
        original = self.workcell['block'].copy()
        self.assertEqual(original, block_scenario(self.workcell))
        result = block_scenario(self.workcell, (-0.27, 0.28), (0.03, 0.025, 0.05), 0.08)
        self.assertEqual([-0.27, 0.28], result['center_xy_base'])
        self.assertEqual([0.03, 0.025, 0.05], result['size'])
        self.assertEqual(0.08, result['mass_kg'])
        self.assertEqual(original, self.workcell['block'])

    def test_joint_state_snapshot_requires_complete_feedback(self):
        position = [0.1, 0.2]
        velocity = [0.0, 0.0]
        effort = [1.0, 2.0]
        self.assertEqual(
            (position, velocity, effort),
            joint_state_snapshot(position, velocity, effort, 2),
        )
        for readings in (
            (None, velocity, effort),
            (position, None, effort),
            (position, velocity, None),
            (position[:1], velocity, effort),
            (position, velocity[:1], effort),
            (position, velocity, effort[:1]),
        ):
            with self.subTest(readings=readings):
                self.assertIsNone(joint_state_snapshot(*readings, expected_count=2))

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
