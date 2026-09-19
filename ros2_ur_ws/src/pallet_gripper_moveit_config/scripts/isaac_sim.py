"""Run with isaac_env Python. Imports the source URDF and uses measured feedback."""
import argparse
import math
import tempfile
import shutil
import time
import xml.etree.ElementTree as ET
from pathlib import Path

parser = argparse.ArgumentParser()
parser.add_argument('--headless', action='store_true')
parser.add_argument('--seconds', type=float, default=0)
parser.add_argument(
    '--disable-gravity', action='store_true',
    help='Diagnostic mode only: disable gravity on the articulation',
)
args = parser.parse_args()
root = Path(__file__).resolve().parents[1]
source = root.parent / 'pallet_gripper/urdf/pallet_gripper.urdf'
import yaml
home = yaml.safe_load((root / 'config/initial_positions.yaml').read_text())['initial_positions']
physics = yaml.safe_load((root / 'config/isaac_physics.yaml').read_text())
tree = ET.parse(source)
limits = {j.get('name'): {k: float(v) for k, v in j.find('limit').attrib.items()}
          for j in tree.getroot().findall('joint') if j.get('type') == 'revolute'}
# Keep the URDF mimic relationship; PhysX imports J9 as a follower of J8.
for mesh in tree.iter('mesh'):
    mesh.set('filename', mesh.get('filename').replace('package://pallet_gripper/',
             str(source.parent.parent) + '/'))

from isaacsim import SimulationApp
app = SimulationApp({'headless': args.headless})
try:
    import numpy as np
    import omni.kit.commands
    import omni.usd
    from pxr import Gf, Sdf, UsdGeom, UsdLux, UsdPhysics
    from isaacsim.core.api import World
    from isaacsim.core.prims import SingleArticulation
    from isaacsim.core.utils.extensions import enable_extension
    from isaacsim.core.utils.types import ArticulationAction
    from isaacsim.core.utils.viewports import set_camera_view
    enable_extension('isaacsim.ros2.bridge')
    enable_extension('isaacsim.asset.importer.urdf')
    app.update()
    import rclpy
    from sensor_msgs.msg import JointState
    world = World(
        stage_units_in_meters=1,
        physics_dt=float(physics['physics_dt']),
        rendering_dt=1/60,
    )
    world.get_physics_context().set_gravity(float(physics['gravity_m_s2']))
    world.scene.add_default_ground_plane(z_position=-0.005)
    ok, cfg = omni.kit.commands.execute('URDFCreateImportConfig')
    if not ok:
        raise RuntimeError('URDF importer unavailable')
    cfg.merge_fixed_joints = True
    cfg.fix_base = True
    cfg.import_inertia_tensor = True
    cfg.set_up_vector(0.0, 0.0, 1.0)
    with tempfile.TemporaryDirectory(prefix='pallet_gripper_isaac_') as tmp:
        path = Path(tmp) / 'pallet_gripper.urdf'
        for index, mesh in enumerate(tree.iter("mesh")):
            original = Path(mesh.get("filename"))
            renamed = Path(tmp) / f"mesh_{index}{original.suffix}"
            shutil.copyfile(original, renamed)
            mesh.set("filename", str(renamed))
        tree.write(path)
        ok, prim = omni.kit.commands.execute('URDFParseAndImportFile', urdf_path=str(path),
                                            import_config=cfg, get_articulation_root=True)
        if not ok:
            raise RuntimeError('URDF import failed')
        stage = omni.usd.get_context().get_stage()

        # SRDF collision exclusions are MoveIt-only. Mirror the requested
        # J6-to-finger exclusions explicitly in PhysX for consistent behavior.
        rigid_links = {
            p.GetName(): p for p in stage.Traverse()
            if p.GetName() in {'J6', 'J8', 'J9'} and p.HasAPI(UsdPhysics.RigidBodyAPI)
        }
        if set(rigid_links) != {'J6', 'J8', 'J9'}:
            raise RuntimeError(f'Could not find collision-filter links: {sorted(rigid_links)}')
        filtered_pairs = UsdPhysics.FilteredPairsAPI.Apply(rigid_links['J6']).CreateFilteredPairsRel()
        filtered_pairs.AddTarget(Sdf.Path(rigid_links['J8'].GetPath()))
        filtered_pairs.AddTarget(Sdf.Path(rigid_links['J9'].GetPath()))
        print(
            'PALLET_FILTERED_COLLISIONS '
            f"J6-J8={rigid_links['J8'].GetPath()} J6-J9={rigid_links['J9'].GetPath()}",
            flush=True,
        )

        # "Stage Lights" only uses lights authored in USD.  Add both ambient
        # and directional illumination so that mode is usable by default.
        dome = UsdLux.DomeLight.Define(stage, '/PalletEnvironmentLight')
        dome.CreateIntensityAttr(250.0)
        dome.CreateColorAttr(Gf.Vec3f(0.85, 0.90, 1.0))
        sun = UsdLux.DistantLight.Define(stage, '/PalletKeyLight')
        sun.CreateIntensityAttr(700.0)
        sun.CreateAngleAttr(4.0)
        UsdGeom.Xformable(sun).AddRotateXYZOp().Set(Gf.Vec3f(-35.0, 25.0, -35.0))

        # Use an upright, repeatable view of the complete robot rather than the
        # viewport's previous session camera.
        set_camera_view(
            eye=np.array([0.68, 0.58, 0.58]),
            target=np.array([-0.10, 0.00, 0.27]),
        )
        for p in stage.Traverse():
            name = p.GetName()
            profile_name = 'J8' if name == 'J9' else name
            if profile_name in home and p.IsA(UsdPhysics.RevoluteJoint):
                drive = UsdPhysics.DriveAPI.Apply(p, 'angular')
                drive.CreateTypeAttr('force')
                drive.CreateMaxForceAttr(limits[name]['effort'])
                drive.CreateStiffnessAttr(float(physics['joints'][profile_name]['stiffness']))
                drive.CreateDampingAttr(float(physics['joints'][profile_name]['damping']))
        robot = world.scene.add(SingleArticulation(prim_path=prim, name='pallet_gripper'))
        world.reset()
        if args.disable_gravity:
            robot.disable_gravity()
        else:
            robot.enable_gravity()
        names = list(robot.dof_names)
        if not set(home).issubset(names) or set(names) - set(home) - {'J9'}:
            raise RuntimeError(f'Unexpected DOFs: {names}')
        target = np.array([float(home.get(n, -home['J8'])) for n in names])
        rated_effort = np.array([float(limits[n]['effort']) for n in names])
        max_measured_effort = np.zeros(len(names))
        robot.set_joint_positions(target)
        robot.set_joint_velocities(np.zeros(len(names)))
        controller = robot.get_articulation_controller()
        rclpy.init()
        node = rclpy.create_node('pallet_gripper_isaac_endpoint')
        pub = node.create_publisher(JointState, '/isaac_joint_states', 10)
        def command(msg):
            if len(msg.name) != len(msg.position) or len(set(msg.name)) != len(msg.name):
                return
            if any(n not in home or not math.isfinite(v) or not limits[n]['lower'] <= v <= limits[n]['upper']
                   for n, v in zip(msg.name, msg.position)):
                node.get_logger().error('Rejected invalid joint command')
                return
            for n, v in zip(msg.name, msg.position):
                target[names.index(n)] = v
            if 'J9' in names:
                target[names.index('J9')] = -target[names.index('J8')]
        node.create_subscription(JointState, '/isaac_joint_commands', command, 10)
        gravity = 0.0 if args.disable_gravity else float(physics['gravity_m_s2'])
        print(f'PALLET_ISAAC_READY joints={names} gravity_m_s2={gravity} base=fixed', flush=True)
        started = time.monotonic()
        while app.is_running() and (not args.seconds or time.monotonic() - started < args.seconds):
            tick = time.monotonic()
            rclpy.spin_once(node, timeout_sec=0)
            controller.apply_action(ArticulationAction(joint_positions=target))
            world.step(render=not args.headless)
            measured_effort = np.abs(np.asarray(robot.get_measured_joint_efforts(), dtype=float))
            max_measured_effort = np.maximum(max_measured_effort, measured_effort)
            msg = JointState()
            msg.header.stamp = node.get_clock().now().to_msg()
            msg.name = names
            msg.position = [float(v) for v in robot.get_joint_positions()]
            msg.velocity = [float(v) for v in robot.get_joint_velocities()]
            pub.publish(msg)
            time.sleep(max(0, 1/240 - (time.monotonic() - tick)))
        final_position = np.asarray(robot.get_joint_positions(), dtype=float)
        final_velocity = np.asarray(robot.get_joint_velocities(), dtype=float)
        print('PALLET_FINAL ' + str(dict(zip(names, final_position.tolist()))), flush=True)
        print('PALLET_FINAL_ERROR_RAD ' + str(dict(zip(names, (final_position - target).tolist()))), flush=True)
        print('PALLET_FINAL_VELOCITY_RAD_S ' + str(dict(zip(names, final_velocity.tolist()))), flush=True)
        print('PALLET_MAX_MEASURED_EFFORT_NM ' + str(dict(zip(names, max_measured_effort.tolist()))), flush=True)
        print('PALLET_MAX_MEASURED_TO_RATED ' + str(dict(zip(names, (max_measured_effort / rated_effort).tolist()))), flush=True)
        print('PALLET_RATED_EFFORT_NM ' + str(dict(zip(names, rated_effort.tolist()))), flush=True)
        node.destroy_node()
        rclpy.shutdown()
finally:
    app.close()
