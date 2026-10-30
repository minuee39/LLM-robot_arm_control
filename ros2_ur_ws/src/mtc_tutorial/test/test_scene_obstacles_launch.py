"""Exercise the launch's exact helper and numeric override loop without ROS."""
import ast
import json
import math
from pathlib import Path
import unittest
import yaml

ROOT = Path(__file__).resolve().parents[1]
TREE = ast.parse((ROOT / 'launch/pick_place_demo.launch.py').read_text())
HELPER = next(n for n in TREE.body if isinstance(n, ast.FunctionDef)
              and n.name == '_pallet_obstacle_parameters')
NS = {'json': json, 'math': math}
exec(compile(ast.Module(body=[HELPER], type_ignores=[]), str(ROOT), 'exec'), NS)
configure = NS['_pallet_obstacle_parameters']
DEFAULTS = next(ast.literal_eval(n.value) for n in TREE.body if isinstance(n, ast.Assign)
                and any(isinstance(a, ast.Name) and a.id == 'FLOAT_DEFAULTS' for a in n.targets))
SETUP = next(n for n in TREE.body if isinstance(n, ast.FunctionDef) and n.name == '_launch_setup')
LOOP = next(n for n in SETUP.body if isinstance(n, ast.For))


class SceneLaunchTest(unittest.TestCase):
    def obstacle(self):
        return {'id': 'green_block', 'position_m': [-0.32, 0.02, 0.025],
                'radius_m': math.sqrt(0.03**2 + 0.03**2 + 0.05**2) / 2}

    def test_disabled_preserves_other_profiles(self):
        self.assertEqual(configure('[]', 'ur10e_robotiq', 'red_block'), {})

    def test_explicit_nonpicked_sphere(self):
        b = self.obstacle()
        self.assertEqual(configure(json.dumps([b]), 'pallet_gripper', 'red_block'),
                         {'pallet_obstacle_ids': ['green_block'],
                          'pallet_obstacle_spheres': [*b['position_m'], b['radius_m']]})

    def test_invalid_obstacles_rejected(self):
        bad = [None, {}, [self.obstacle()] * 3, [self.obstacle()] * 2,
               [dict(self.obstacle(), id='red_block')],
               [dict(self.obstacle(), radius_m=float('nan'))],
               [dict(self.obstacle(), radius_m=True)],
               [dict(self.obstacle(), position_m=[0, 0, float('inf')])],
               [dict(self.obstacle(), position_m=[0, 0, -1])]]
        for value in bad:
            with self.subTest(value=value), self.assertRaises(ValueError):
                configure(json.dumps(value), 'pallet_gripper', 'red_block')
        with self.assertRaises(ValueError):
            configure(json.dumps([self.obstacle()]), 'ur10e_robotiq', 'red_block')

    def test_numeric_loop_preserves_profile_and_forwards_height_override(self):
        profiles = yaml.safe_load((ROOT / 'config/robot_profiles.yaml').read_text())['profiles']
        fields = ('lift_min_distance', 'lift_max_distance', 'lower_min_distance', 'lower_max_distance')
        class Configuration:
            def __init__(self, name):
                self.name = name
            def perform(self, context):
                return context.get(self.name, 'profile')
        for profile in profiles.values():
            original = profile['mtc_parameters']
            for raw in ({}, {k: '0.10' for k in fields}):
                ns = {'FLOAT_DEFAULTS': DEFAULTS, 'LaunchConfiguration': Configuration,
                      'parameters': dict(original), 'context': raw}
                exec(compile(ast.Module(body=[LOOP], type_ignores=[]), str(ROOT), 'exec'), ns)
                for k in fields:
                    self.assertEqual(ns['parameters'][k], 0.1 if raw else original[k])

    def test_release_override_defaults_and_only_final_stage_use(self):
        class Configuration:
            def __init__(self, name):
                self.name = name
            def perform(self, context):
                return context.get(self.name, 'profile')
        for raw, expected in (({}, -1.0), ({'release_open_position': '0.020'}, 0.020)):
            ns = {'FLOAT_DEFAULTS': DEFAULTS, 'LaunchConfiguration': Configuration,
                  'parameters': {}, 'context': raw}
            exec(compile(ast.Module(body=[LOOP], type_ignores=[]), str(ROOT), 'exec'), ns)
            self.assertEqual(ns['parameters']['release_open_position'], expected)
        source = (ROOT/'src/mtc_tutorial.cpp').read_text()
        final = source.index('const double release_open_position = param("release_open_position")')
        self.assertLess(source.index('stage_open_hand->setGoal(open_pose_name)'), final)
        self.assertLess(source.index('stage->setPreGraspPose(open_pose_name)'), final)


if __name__ == '__main__':
    unittest.main()
