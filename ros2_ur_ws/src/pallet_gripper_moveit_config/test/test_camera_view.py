"""Camera offsets/argument wiring; actual CameraInfo/image still requires runtime QA."""
import ast
import math
from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'scripts'))
from workcell_scenario import rgbd_camera_offsets, rgbd_camera_resolution


class CameraViewTest(unittest.TestCase):
    def test_clock_default_and_all_camera_outputs_forwarded(self):
        tree = ast.parse((ROOT.parent/'mtc_tutorial/launch/pallet_mtc_isaac_demo.launch.py').read_text())
        calls = [n for n in ast.walk(tree) if isinstance(n, ast.Call)
                 and isinstance(n.func, ast.Name) and n.func.id == 'DeclareLaunchArgument'
                 and n.args and isinstance(n.args[0], ast.Constant) and n.args[0].value == 'rgbd_clock']
        kwargs = {k.arg: ast.literal_eval(k.value) for k in calls[0].keywords}
        self.assertEqual(kwargs, {'default_value': 'simulation', 'choices': ['simulation', 'system']})
        simulator = ast.parse((ROOT/'scripts/isaac_sim.py').read_text())
        entries = [n for n in ast.walk(simulator) if isinstance(n, ast.Tuple) and len(n.elts) == 2
                   and isinstance(n.elts[0], ast.Constant) and str(n.elts[0].value).endswith('inputs:useSystemTime')]
        self.assertEqual({n.elts[0].value for n in entries},
                         {'Rgb.inputs:useSystemTime', 'Depth.inputs:useSystemTime', 'Info.inputs:useSystemTime'})
        for entry in entries:
            expression = ast.Expression(entry.elts[1])
            from types import SimpleNamespace
            for mode in ('simulation', 'system'):
                self.assertEqual(eval(compile(expression, 'clock', 'eval'),
                                      {'args': SimpleNamespace(rgbd_clock=mode)}), mode == 'system')

    def test_resolution_default_and_choice_validation(self):
        self.assertEqual(rgbd_camera_resolution(), (640, 480))
        self.assertEqual(rgbd_camera_resolution('1280x960'), (1280, 960))
        with self.assertRaises(ValueError):
            rgbd_camera_resolution('invalid')

    def test_resolution_launch_forwarding_and_default(self):
        tree = ast.parse((ROOT.parent/'mtc_tutorial/launch/pallet_mtc_isaac_demo.launch.py').read_text())
        calls = [n for n in ast.walk(tree) if isinstance(n, ast.Call)
                 and isinstance(n.func, ast.Name) and n.func.id == 'DeclareLaunchArgument'
                 and n.args and isinstance(n.args[0], ast.Constant)
                 and n.args[0].value == 'rgbd_resolution']
        self.assertEqual(len(calls), 1)
        kwargs = {k.arg: ast.literal_eval(k.value) for k in calls[0].keywords}
        self.assertEqual(kwargs, {'default_value': '640x480', 'choices': ['640x480', '1280x960']})
        args = next(n.value for n in ast.walk(tree) if isinstance(n, ast.Assign)
                    and any(isinstance(t, ast.Name) and t.id == 'block_args' for t in n.targets))
        index = next(i for i, n in enumerate(args.elts) if isinstance(n, ast.Constant)
                     and n.value == '--rgbd-resolution')
        self.assertEqual(args.elts[index+1].args[0].value, 'rgbd_resolution')

    def test_default_pose_preserved_and_unknown_rejected(self):
        self.assertEqual(rgbd_camera_offsets(), ((-.55, .65, .85), (-.32, .18, .025)))
        with self.assertRaises(ValueError):
            rgbd_camera_offsets('unknown')

    def test_candidate_workspace_directions_fit_conservative_window(self):
        eye, target = rgbd_camera_offsets('workcell')
        def normalize(v):
            size = math.sqrt(sum(x*x for x in v))
            return [x/size for x in v]
        forward = normalize([target[i]-eye[i] for i in range(3)])
        right = normalize([forward[1], -forward[0], 0])
        down = [forward[1]*right[2]-forward[2]*right[1],
                forward[2]*right[0]-forward[0]*right[2],
                forward[0]*right[1]-forward[1]*right[0]]
        # Candidate geometry only; do not substitute this for measured intrinsics/FOV.
        for x, y in [(-.32, .10), (-.32, .02), (-.32, -.06),
                     (.24, -.28), (.28, -.24), (.28, -.28)]:
            for dx in (-.015, .015):
                for dy in (-.015, .015):
                    for height in (0, .05):
                        delta = [x+dx-eye[0], y+dy-eye[1], height-eye[2]]
                        point = [sum(delta[i]*axis[i] for i in range(3))
                                 for axis in (right, down, forward)]
                        self.assertGreater(point[2], 0)
                        self.assertLess(abs(point[0]/point[2]), .30)
                        self.assertLess(abs(point[1]/point[2]), .25)

    def test_launch_default_declaration_and_simulator_forwarding(self):
        launch = ROOT.parent/'mtc_tutorial/launch/pallet_mtc_isaac_demo.launch.py'
        tree = ast.parse(launch.read_text())
        declarations = [n for n in ast.walk(tree) if isinstance(n, ast.Call)
                        and isinstance(n.func, ast.Name) and n.func.id == 'DeclareLaunchArgument'
                        and n.args and isinstance(n.args[0], ast.Constant)
                        and n.args[0].value == 'camera_view']
        self.assertEqual(len(declarations), 1)
        kwargs = {k.arg: ast.literal_eval(k.value) for k in declarations[0].keywords}
        self.assertEqual(kwargs['default_value'], 'source')
        self.assertEqual(kwargs['choices'], ['source', 'workcell'])
        args = next(n.value for n in ast.walk(tree) if isinstance(n, ast.Assign)
                    and any(isinstance(t, ast.Name) and t.id == 'block_args' for t in n.targets))
        index = next(i for i, n in enumerate(args.elts) if isinstance(n, ast.Constant)
                     and n.value == '--camera-view')
        self.assertEqual(args.elts[index+1].args[0].value, 'camera_view')


if __name__ == '__main__':
    unittest.main()
