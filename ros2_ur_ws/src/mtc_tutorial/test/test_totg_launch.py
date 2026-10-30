"""Test the pure launch configuration helper without starting ROS."""
import ast
from copy import deepcopy
import json
import math
from pathlib import Path
import unittest

import yaml


ROOT = Path(__file__).resolve().parents[1]
tree = ast.parse((ROOT / 'launch/pick_place_demo.launch.py').read_text())
helper = next(node for node in tree.body if isinstance(node, ast.FunctionDef)
              and node.name == '_configure_totg')
namespace = {'json': json, 'math': math}
exec(compile(ast.Module(body=[helper], type_ignores=[]), str(ROOT), 'exec'), namespace)
configure = namespace['_configure_totg']


class TotgLaunchTest(unittest.TestCase):
    def config(self):
        path = ROOT.parent / 'pallet_gripper_moveit_config/config/ompl_planning.yaml'
        return {'ompl': yaml.safe_load(path.read_text())}

    def test_default_preserves_existing_configuration(self):
        config = self.config()
        original = deepcopy(config)
        configure(config, 'default', 'pallet_gripper')
        self.assertEqual(config, original)

    def test_override_changes_only_path_tolerance(self):
        config = self.config()
        original = deepcopy(config)
        configure(config, '0.01', 'pallet_gripper')
        self.assertEqual(config['ompl'].pop('totg'), {'path_tolerance': 0.01})
        self.assertEqual(config, original)

    def test_invalid_values_profiles_and_missing_validation_are_rejected(self):
        for value in ('0', '-0.01', '0.2', 'nan', 'inf', 'bad'):
            with self.subTest(value=value), self.assertRaises(ValueError):
                configure(self.config(), value, 'pallet_gripper')
        with self.assertRaises(ValueError):
            configure(self.config(), '0.01', 'ur10e_robotiq')
        config = self.config()
        config['ompl']['response_adapters'] = []
        with self.assertRaises(ValueError):
            configure(config, '0.01', 'pallet_gripper')


if __name__ == '__main__':
    unittest.main()
