from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
WORKSPACE_SRC = PACKAGE_ROOT.parent


class RobotProfilesTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        data = yaml.safe_load(
            (PACKAGE_ROOT / 'config/robot_profiles.yaml').read_text(encoding='utf-8')
        )
        cls.profiles = data['profiles']

    def test_expected_profiles_exist(self):
        self.assertEqual(
            {'ur10e_robotiq', 'pallet_gripper'}, set(self.profiles)
        )

    def test_profile_semantics_match_srdf(self):
        for name, profile in self.profiles.items():
            with self.subTest(profile=name):
                package = WORKSPACE_SRC / profile['moveit_config_package']
                semantic = package / profile['semantic_file'].removeprefix('config/')
                if not semantic.exists():
                    semantic = package / profile['semantic_file']
                root = ET.parse(semantic).getroot()
                params = profile['mtc_parameters']

                self.assertEqual(profile['robot_name'], root.attrib['name'])
                groups = {node.attrib['name'] for node in root.findall('group')}
                self.assertIn(params['arm_group_name'], groups)
                self.assertIn(params['hand_group_name'], groups)

                end_effectors = {
                    node.attrib['name']: node for node in root.findall('end_effector')
                }
                self.assertIn(params['eef_name'], end_effectors)
                self.assertEqual(
                    params['hand_group_name'],
                    end_effectors[params['eef_name']].attrib['group'],
                )

                states = {
                    (node.attrib['group'], node.attrib['name'])
                    for node in root.findall('group_state')
                }
                self.assertIn(
                    (params['hand_group_name'], params['open_pose_name']), states
                )
                self.assertIn(
                    (params['arm_group_name'], params['home_pose_name']), states
                )

                hand_group = next(
                    node
                    for node in root.findall('group')
                    if node.attrib['name'] == params['hand_group_name']
                )
                hand_joints = {node.attrib['name'] for node in hand_group.findall('joint')}
                self.assertIn(params['gripper_joint_name'], hand_joints)

    def test_arm_groups_have_the_selected_ompl_planner(self):
        for name, profile in self.profiles.items():
            with self.subTest(profile=name):
                package = WORKSPACE_SRC / profile['moveit_config_package']
                planning = yaml.safe_load(
                    (package / 'config/ompl_planning.yaml').read_text(
                        encoding='utf-8'
                    )
                )
                arm_group = profile['mtc_parameters']['arm_group_name']
                planner_id = 'RRTConnectkConfigDefault'
                self.assertIn(planner_id, planning['planner_configs'])
                self.assertIn(arm_group, planning)
                self.assertIn(planner_id, planning[arm_group]['planner_configs'])

    def test_pallet_workspace_matches_centered_table(self):
        params = self.profiles['pallet_gripper']['mtc_parameters']
        workcell = yaml.safe_load(
            (WORKSPACE_SRC / 'pallet_gripper_moveit_config/config/workcell.yaml').read_text()
        )
        self.assertEqual('base_footprint', params['planning_frame'])
        self.assertEqual(workcell['table']['size'][0], params['table_size_x'])
        self.assertEqual(workcell['table']['size'][1], params['table_size_y'])
        self.assertEqual(workcell['table']['size'][2], params['table_height'])
        self.assertEqual(0.0, params['table_center_x'])
        self.assertEqual(0.0, params['table_center_y'])
        self.assertEqual(0.0, params['table_top_z'])
        self.assertEqual(workcell['block']['center_xy_base'][0], params['object_x'])
        self.assertEqual(workcell['block']['center_xy_base'][1], params['object_y'])
        self.assertEqual(workcell['block']['size'], [
            params['object_size_x'], params['object_size_y'], params['object_size_z'],
        ])
        self.assertEqual(workcell['block']['size'][2] / 2.0, params['object_z'])

    def test_pallet_calibration_block_and_gripper_target(self):
        params = self.profiles['pallet_gripper']['mtc_parameters']
        self.assertEqual(0.030, params['object_size_x'])
        self.assertEqual(0.030, params['object_size_y'])
        self.assertEqual(0.050, params['object_size_z'])
        self.assertEqual(0.025, params['object_z'])
        self.assertEqual(0.025, params['place_z'])
        self.assertAlmostEqual(0.010, params['gripper_close_min'], places=6)
        self.assertEqual(params['gripper_close_min'], params['gripper_close_max'])
        self.assertAlmostEqual(-0.015, params['grasp_offset_z'], places=6)
        self.assertEqual(0.04, params['lift_min_distance'])
        self.assertEqual(params['lift_min_distance'], params['lift_max_distance'])
        self.assertEqual(0.02, params['transfer_min_clearance'])
        self.assertGreater(params['transfer_clearance_check_step'], 0.0)
        self.assertEqual(5.0, params['move_to_place_timeout'])
        self.assertGreaterEqual(params['task_timeout'], 180.0)
        self.assertEqual(2.0, params['place_max_ik_solutions'])
        self.assertEqual(0.04, params['lower_min_distance'])
        self.assertEqual(params['lower_min_distance'], params['lower_max_distance'])
        self.assertEqual(0.04, params['retreat_min_distance'])
        self.assertEqual(params['retreat_min_distance'], params['retreat_max_distance'])
        self.assertEqual(0.04, params['recovery_retreat_distance'])
        self.assertEqual(5.0, params['recovery_retreat_timeout'])
        self.assertEqual(0.50, params['arm_velocity_scaling'])
        self.assertEqual(0.50, params['arm_acceleration_scaling'])
        self.assertEqual(0.25, params['cartesian_velocity_scaling'])
        self.assertEqual(0.25, params['cartesian_acceleration_scaling'])
        self.assertEqual(0.50, params['gripper_velocity_scaling'])
        self.assertEqual(0.50, params['gripper_acceleration_scaling'])
        self.assertEqual('pallet_arm_controller', params['arm_controller_name'])
        self.assertEqual('gripper_controller', params['gripper_controller_name'])


if __name__ == '__main__':
    unittest.main()
