from pathlib import Path
import sys
import tempfile
import unittest


PROJECT_ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(PROJECT_ROOT / 'ros2_ur_ws/scripts'))
from run_pallet_scenario_matrix import (
    classify_failure, execution_seconds, parse_recovery_diagnostics,
    recovery_candidates, scenario_commands, summarize_matrix, summarize_runs,
    task_completed, validate_scenario)


class PalletScenarioMatrixTest(unittest.TestCase):
    def test_commands_keep_isaac_monitor_and_mtc_geometry_consistent(self):
        scenario = validate_scenario('narrow', {
            'center_xy': [-0.27, 0.28],
            'size': [0.03, 0.025, 0.05],
            'mass_kg': 0.065,
            'gripper_close_position': 0.002,
            'arm_velocity_scaling': 0.2,
            'arm_acceleration_scaling': 0.1,
        })
        with tempfile.TemporaryDirectory() as directory:
            simulation, monitor, task = scenario_commands(
                scenario, Path(directory) / 'motion.json', 180)
        self.assertIn('block_y:=0.280000', simulation)
        self.assertIn('block_size_y:=0.025000', simulation)
        self.assertIn('block_mass_kg:=0.065000', simulation)
        self.assertIn('object_y:=0.280000', task)
        self.assertIn('object_size_y:=0.025000', task)
        self.assertIn('object_z:=0.025000', task)
        self.assertIn('gripper_close_min:=0.002000', task)
        self.assertIn('gripper_close_max:=0.002000', task)
        self.assertIn('arm_velocity_scaling:=0.200000', task)
        self.assertIn('arm_acceleration_scaling:=0.100000', task)
        self.assertIn('0.025000', monitor)
        self.assertIn('--slip-threshold', monitor)
        self.assertIn('0.005000', monitor)
        self.assertIn('--slip-confirm-samples', monitor)
        self.assertIn('3', monitor)

        with tempfile.TemporaryDirectory() as directory:
            gui_simulation, _, _ = scenario_commands(
                scenario, Path(directory) / 'motion.json', 180, headless=False)
        self.assertIn('headless:=false', gui_simulation)

        with tempfile.TemporaryDirectory() as directory:
            event_file = Path(directory) / 'event.json'
            safe_simulation, injected_monitor, _ = scenario_commands(
                scenario, Path(directory) / 'motion.json', 180,
                safety_cancel_enabled=True,
                inject_safety_event_after_contact=0.5,
                event_file=event_file,
                inject_safety_code='TILT_DETECTED')
        self.assertIn('safety_cancel_enabled:=true', safe_simulation)
        self.assertIn('--inject-safety-event-after-contact', injected_monitor)
        self.assertIn('0.500000', injected_monitor)
        self.assertIn('--event-output', injected_monitor)
        self.assertIn(str(event_file), injected_monitor)
        self.assertIn('--inject-safety-code', injected_monitor)
        self.assertIn('TILT_DETECTED', injected_monitor)
        self.assertIn('--drop-clearance-threshold', injected_monitor)
        self.assertIn('--tilt-threshold-deg', injected_monitor)
        self.assertIn('--fault-confirm-samples', injected_monitor)

        with tempfile.TemporaryDirectory() as directory:
            fault_simulation, fault_monitor, _ = scenario_commands(
                scenario, Path(directory) / 'motion.json', 180,
                safety_cancel_enabled=True,
                event_file=Path(directory) / 'event.json',
                inject_sim_fault_phase='transfer',
                inject_sim_fault_code='TELEMETRY_STALE',
                inject_sim_fault_delay=0.25)
        self.assertIn('fault_injection_enabled:=true', fault_simulation)
        self.assertIn('--inject-sim-fault-phase', fault_monitor)
        self.assertIn('transfer', fault_monitor)
        self.assertIn('--inject-sim-fault-code', fault_monitor)
        self.assertIn('TELEMETRY_STALE', fault_monitor)
        self.assertIn('--pose-stream-timeout', fault_monitor)

    def test_invalid_scenarios_are_rejected(self):
        valid = {'center_xy': [-0.3, 0.3], 'size': [0.03, 0.03, 0.05], 'mass_kg': 0.05}
        self.assertEqual('base', validate_scenario('base', valid)['name'])
        heavy = dict(valid, mass_kg=0.5)
        self.assertEqual(0.5, validate_scenario('mass_500', heavy)['mass_kg'])
        for close in (-0.001, 0.034, float('nan')):
            with self.subTest(close=close), self.assertRaises(ValueError):
                validate_scenario(
                    'bad_close', dict(valid, gripper_close_position=close))
        for scaling in (0.0, 1.01, float('inf')):
            with self.subTest(scaling=scaling), self.assertRaises(ValueError):
                validate_scenario(
                    'bad_scaling', dict(valid, arm_velocity_scaling=scaling))
        for change in (
            {'center_xy': [-0.3]}, {'center_xy': [-0.3, 0.37]},
            {'size': [0.03, 0.067, 0.05]}, {'mass_kg': 0},
        ):
            candidate = dict(valid)
            candidate.update(change)
            with self.subTest(change=change), self.assertRaises(ValueError):
                validate_scenario('bad', candidate)

    def test_task_completion_requires_child_success_message(self):
        with tempfile.TemporaryDirectory() as directory:
            log = Path(directory) / 'task.log'
            self.assertFalse(task_completed(log))
            log.write_text(
                '[ERROR] [mtc_tutorial-1]: process has died [exit code 1]\n',
                encoding='utf-8')
            self.assertFalse(task_completed(log))
            log.write_text(
                '[mtc_tutorial-1] [INFO] Task execution completed in 2.3 s\n',
                encoding='utf-8')
            self.assertTrue(task_completed(log))

    def test_run_statistics_include_success_rate_and_metric_spread(self):
        runs = [
            {
                'scenario': {'name': 'base'}, 'passed': True,
                'elapsed_seconds': 10.0,
                'task_planning_seconds': 2.0,
                'task_execution_seconds': 5.0,
                'process_cleanup_ok': True,
                'summary': {
                    'transfer_min_bottom_clearance': 0.02,
                    'final_xy_error': 0.004,
                },
            },
            {
                'scenario': {'name': 'base'}, 'passed': True,
                'elapsed_seconds': 14.0,
                'task_planning_seconds': 4.0,
                'task_execution_seconds': 7.0,
                'process_cleanup_ok': True,
                'summary': {
                    'transfer_min_bottom_clearance': 0.03,
                    'final_xy_error': 0.006,
                },
            },
            {
                'scenario': {'name': 'base'}, 'passed': False,
                'elapsed_seconds': 12.0,
                'task_planning_seconds': 6.0,
                'task_execution_seconds': 9.0,
                'task_execution_completed': True,
                'process_cleanup_ok': True,
                'summary': {
                    'transfer_min_bottom_clearance': -1.0,
                    'final_xy_error': 1.0,
                },
            },
        ]
        summary = summarize_runs(runs)['base']
        self.assertEqual(3, summary['run_count'])
        self.assertEqual(2, summary['success_count'])
        self.assertAlmostEqual(2 / 3, summary['success_rate'])
        self.assertEqual(2, summary['transfer_min_bottom_clearance']['count'])
        self.assertAlmostEqual(0.025, summary['transfer_min_bottom_clearance']['mean'])
        self.assertAlmostEqual(0.005, summary['transfer_min_bottom_clearance']['stddev'])
        self.assertEqual(12.0, summary['elapsed_seconds']['mean'])
        self.assertEqual(4.0, summary['task_planning_seconds']['mean'])
        self.assertEqual(7.0, summary['task_execution_seconds']['mean'])
        overall = summarize_matrix(runs)
        self.assertEqual(3, overall['run_count'])
        self.assertAlmostEqual(2 / 3, overall['success_rate'])

    def test_execution_timing_and_failure_classification(self):
        with tempfile.TemporaryDirectory() as directory:
            log = Path(directory) / 'task.log'
            log.write_text(
                '[INFO] Task execution completed in 4.125 s\n', encoding='utf-8')
            self.assertEqual(4.125, execution_seconds(log))
        self.assertEqual('task_timeout', classify_failure({
            'passed': False, 'task_returncode': 124, 'error': 'MTC task timed out',
        }))
        self.assertEqual('safety_false_positive', classify_failure({
            'passed': False, 'safety_stop_expected': False,
            'process_cleanup_ok': True, 'summary': {'safety_event_published': True},
        }))
        self.assertEqual('process_cleanup', classify_failure({
            'passed': False, 'process_cleanup_ok': False,
        }))

    def test_recovery_candidates_are_bounded_and_deterministic(self):
        scenario = {'center_xy': [-0.27, 0.28]}
        candidates = recovery_candidates(scenario)
        self.assertEqual(
            ['vertical', 'reverse_transfer', 'lateral_left', 'lateral_right'],
            [name for name, _ in candidates])
        self.assertEqual((0.0, 0.0, 1.0), candidates[0][1])
        for _, direction in candidates:
            self.assertGreater(sum(value * value for value in direction), 0.0)
        self.assertEqual(
            ['reverse_transfer'],
            [name for name, _ in recovery_candidates(
                scenario, ['reverse_transfer'])])

    def test_recovery_diagnostics_classify_safe_cartesian_refusal(self):
        diagnostics = parse_recovery_diagnostics(
            'vertical',
            '[ERROR] Task planning failed after 1.137 s\n'
            'recovery retreat vertical (0/1): CartesianPath: '
            'min_fraction not met. Achieved: 0.000000\n',
            1)
        self.assertEqual('cartesian_path_incomplete',
                         diagnostics['failure_classification'])
        self.assertEqual(1.137, diagnostics['planning_seconds'])
        self.assertTrue(diagnostics['safe_refusal'])
        self.assertFalse(diagnostics['execution_started'])


if __name__ == '__main__':
    unittest.main()
