"""Invalid contact overrides must fail before importing or starting Isaac."""
from pathlib import Path
import ast
import subprocess
import sys
import unittest


SCRIPT = Path(__file__).resolve().parents[1] / 'scripts/isaac_sim.py'


class ContactOverrideCliTest(unittest.TestCase):
    def test_fixture_pose_is_persisted_across_world_reset(self):
        tree = ast.parse(SCRIPT.read_text())
        loops = [n for n in ast.walk(tree) if isinstance(n, ast.For)
                 and ast.unparse(n.iter) == 'fixture_poses.items()']
        self.assertEqual(len(loops), 1)
        calls = [n.value for n in loops[0].body if isinstance(n, ast.Expr)
                 and isinstance(n.value, ast.Call)]
        default = next(c for c in calls if c.func.attr == 'set_default_state')
        current = next(c for c in calls if c.func.attr == 'set_world_pose')
        self.assertEqual(ast.unparse(default.keywords[0].value), ast.unparse(current.keywords[0].value))
        self.assertEqual(ast.unparse(default.keywords[1].value), ast.unparse(current.keywords[1].value))

    def test_diagnostic_fixture_cannot_subscribe_to_motion_commands(self):
        tree = ast.parse(SCRIPT.read_text())
        guards = [n for n in ast.walk(tree) if isinstance(n, ast.If)
                  and isinstance(n.test, ast.UnaryOp) and isinstance(n.test.op, ast.Not)
                  and isinstance(n.test.operand, ast.Name) and n.test.operand.id == 'fixture_poses']
        self.assertEqual(len(guards), 1)
        self.assertIn('/isaac_joint_commands', ast.unparse(guards[0].body[0]))
        self.assertNotIn('create_subscription', ast.unparse(guards[0].orelse[0]))

    def test_diagnostic_fixture_requires_bounded_camera_only_contract(self):
        for extra in ([], ['--seconds', '0'], ['--seconds', '121'],
                      ['--seconds', 'nan'], ['--seconds', '60', '--fault-injection-enabled', 'true']):
            result = subprocess.run([
                sys.executable, str(SCRIPT), '--diagnostic-block-pose', 'red_block', '.24', '-.28', '0',
                '--calibration-block', '--block-layout', 'three', '--rgbd-camera', *extra,
            ], capture_output=True, text=True, timeout=10)
            self.assertEqual(result.returncode, 2, result.stderr)
            self.assertIn('--diagnostic-block-pose requires', result.stderr)
            self.assertNotIn('Traceback', result.stderr)

    def test_geometry_logging_requires_block_before_startup(self):
        result = subprocess.run(
            [sys.executable, str(SCRIPT), '--contact-geometry-log', 'true'],
            capture_output=True, text=True, timeout=10,
        )
        self.assertEqual(result.returncode, 2, result.stderr)
        self.assertIn('requires --calibration-block', result.stderr)
        self.assertNotIn('Traceback', result.stderr)

    def test_rejects_invalid_overrides_before_simulator_start(self):
        cases = {
            '--solver-velocity-iterations': ('-2', '0', '65'),
            '--jaw-friction': ('nan', 'inf', '-0.1', '2.01'),
            '--block-static-friction': ('nan', '-0.1', '2.01'),
            '--block-contact-offset-m': ('nan', '0', '-0.1', '0.0101'),
        }
        for option, values in cases.items():
            for value in values:
                with self.subTest(option=option, value=value):
                    result = subprocess.run(
                        [sys.executable, str(SCRIPT), option, value],
                        capture_output=True, text=True, timeout=10,
                    )
                    self.assertEqual(result.returncode, 2, result.stderr)
                    self.assertIn(f'{option} must be', result.stderr)
                    self.assertNotIn('Traceback', result.stderr)

    def test_help_exposes_experimental_options_without_startup(self):
        result = subprocess.run(
            [sys.executable, str(SCRIPT), '--help'],
            capture_output=True, text=True, timeout=10,
        )
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn('--jaw-friction', result.stdout)
        self.assertIn('--contact-geometry-log', result.stdout)

    def test_geometry_logging_rejects_invalid_switch_before_startup(self):
        result = subprocess.run(
            [sys.executable, str(SCRIPT), '--contact-geometry-log', 'yes'],
            capture_output=True, text=True, timeout=10,
        )
        self.assertEqual(result.returncode, 2, result.stderr)
        self.assertIn('invalid choice', result.stderr)
        self.assertNotIn('Traceback', result.stderr)

    def test_external_force_mode_rejects_unknown_value_before_startup(self):
        result = subprocess.run(
            [sys.executable, str(SCRIPT), '--external-forces-every-iteration', 'yes'],
            capture_output=True, text=True, timeout=10,
        )
        self.assertEqual(result.returncode, 2, result.stderr)
        self.assertIn('invalid choice', result.stderr)

    def test_camera_view_rejects_unknown_before_startup(self):
        result = subprocess.run(
            [sys.executable, str(SCRIPT), '--camera-view', 'unknown'],
            capture_output=True, text=True, timeout=10,
        )
        self.assertEqual(result.returncode, 2, result.stderr)
        self.assertIn('invalid choice', result.stderr)


if __name__ == '__main__':
    unittest.main()
