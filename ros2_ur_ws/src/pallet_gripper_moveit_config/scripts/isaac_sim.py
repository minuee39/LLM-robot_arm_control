"""Run with isaac_env Python. Imports the source URDF and uses measured feedback."""
import argparse
import json
import math
import os
import tempfile
import shutil
import time
import traceback
import xml.etree.ElementTree as ET
from pathlib import Path

parser = argparse.ArgumentParser()
parser.add_argument('--headless', action='store_true')
parser.add_argument('--calibration-block', action='store_true')
parser.add_argument('--block-center-xy', nargs=2, type=float, metavar=('X', 'Y'))
parser.add_argument('--block-size', nargs=3, type=float, metavar=('X', 'Y', 'Z'))
parser.add_argument('--block-mass-kg', type=float)
parser.add_argument(
    '--fault-injection-enabled', choices=('true', 'false'), default='false',
    help='Enable the test-only /pallet/sim_fault_commands endpoint',
)
parser.add_argument('--rgbd-camera', action='store_true', help='Publish simulated RGB, depth, and CameraInfo topics')
parser.add_argument('--camera-calibration-file', default='/tmp/pallet_camera_calibration.json')
parser.add_argument('--no-cup', action='store_true')
parser.add_argument('--seconds', type=float, default=0)
parser.add_argument(
    '--disable-gravity', action='store_true',
    help='Diagnostic mode only: disable gravity on the articulation',
)
args = parser.parse_args()
fault_injection_enabled = args.fault_injection_enabled == 'true'
root = Path(__file__).resolve().parents[1]
source = root.parent / 'pallet_gripper/urdf/pallet_gripper.urdf'
import yaml
home = yaml.safe_load((root / 'config/initial_positions.yaml').read_text())['initial_positions']
physics = yaml.safe_load((root / 'config/isaac_physics.yaml').read_text())
workcell = yaml.safe_load((root / 'config/workcell.yaml').read_text())
from workcell_scenario import (
    block_scenario, gripper_effort_limit, gripper_symmetry_config,
    symmetric_jaw_targets,
)
try:
    workcell['block'] = block_scenario(
        workcell, args.block_center_xy, args.block_size, args.block_mass_kg,
    )
except ValueError as error:
    parser.error(str(error))
tree = ET.parse(source)
limits = {j.get('name'): {k: float(v) for k, v in j.find('limit').attrib.items()}
          for j in tree.getroot().findall('joint')
          if j.get('type') in {'revolute', 'prismatic'}}
# J8/J9 are linear simulation proxies, so the Notion calibration candidate is
# expressed as N/finger rather than motor torque in N*m. Keep the source URDF
# limits untouched because they are also the public MoveIt/hardware contract.
sim_gripper_effort_limit = gripper_effort_limit(physics)
symmetry_gain, symmetry_max_correction = gripper_symmetry_config(physics)
for jaw_name in ('J8', 'J9'):
    limits[jaw_name]['effort'] = sim_gripper_effort_limit
print(
    'PALLET_SIM_GRIPPER_EFFORT_LIMIT_N_PER_FINGER '
    f'{sim_gripper_effort_limit}',
    flush=True,
)
# Keep the public URDF mimic relationship for MoveIt and TF. In the simulator,
# drive both jaws explicitly and publish both measured positions so asymmetric
# contact is observable instead of inferred from the commanded mimic position.
j9 = tree.getroot().find("joint[@name='J9']")
if j9 is None or j9.find('mimic') is None:
    raise RuntimeError('Expected J9 to mimic J8 in the source URDF')
j9.remove(j9.find('mimic'))
# The imported CAD collision hulls include finger linkages and touch the block
# before the measured fingertip gap closes. Use a pair of flat fingertip pads
# for this *simulation* contact study. Visual meshes and the public MoveIt URDF
# remain CAD-based; this proxy must not be treated as measured hardware geometry.
for name, local_xyz, local_pitch in (
    ('J8', '-0.07212641 0.0082 -0.03350539', '1.39629265359'),
    ('J9', '-0.07212666 0.0082 0.03350486', '-1.3963'),
):
    collision = tree.getroot().find(f"link[@name='{name}']/collision")
    if collision is None:
        raise RuntimeError(f'Expected collision geometry on {name}')
    collision.find('origin').set('xyz', local_xyz)
    collision.find('origin').set('rpy', f'0 {local_pitch} 0')
    geometry = collision.find('geometry')
    for element in list(geometry):
        geometry.remove(element)
    ET.SubElement(geometry, 'box', size='0.012 0.022 0.030')
for mesh in tree.iter('mesh'):
    mesh.set('filename', mesh.get('filename').replace('package://pallet_gripper/',
             str(source.parent.parent) + '/'))

from isaacsim import SimulationApp
app = SimulationApp({'headless': args.headless})
try:
    import numpy as np
    import omni.kit.commands
    import omni.usd
    import omni.graph.core as og
    import usdrt.Sdf
    from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux, UsdPhysics
    from isaacsim.core.api import World
    from isaacsim.core.api.objects import DynamicCuboid, DynamicCylinder, FixedCuboid
    from isaacsim.core.prims import SingleArticulation
    from isaacsim.core.utils.extensions import enable_extension
    from isaacsim.core.utils.types import ArticulationAction
    from isaacsim.core.utils.viewports import set_camera_view
    enable_extension('isaacsim.ros2.bridge')
    enable_extension('isaacsim.asset.importer.urdf')
    app.update()

    def add_rgbd_camera(stage, base_position):
        """Author a fixed USD camera and return its measured optical-to-base pose."""
        camera_path = '/World/PalletRGBDCamera'
        camera = UsdGeom.Camera.Define(stage, camera_path)
        camera.GetFocalLengthAttr().Set(24.0)
        camera.GetClippingRangeAttr().Set(Gf.Vec2f(0.01, 10.0))
        eye = np.asarray(base_position) + np.array([-0.55, 0.65, 0.85])
        target = np.asarray(base_position) + np.array([-0.30, 0.30, 0.025])
        camera_to_world = Gf.Matrix4d().SetLookAt(
            Gf.Vec3d(*eye), Gf.Vec3d(*target), Gf.Vec3d(0.0, 0.0, 1.0)
        ).GetInverse()
        camera_xform = UsdGeom.Xformable(camera.GetPrim())
        camera_xform.AddTransformOp().Set(camera_to_world)
        actual = camera_xform.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        # USD cameras look along -Z with +Y up; ROS optical cameras use +Z
        # forward and +Y down. The robot's mounted base has identity rotation.
        optical_to_base = np.eye(4)
        optical_to_base[:3, 0] = np.asarray(actual.TransformDir(Gf.Vec3d(1, 0, 0)))
        optical_to_base[:3, 1] = np.asarray(actual.TransformDir(Gf.Vec3d(0, -1, 0)))
        optical_to_base[:3, 2] = np.asarray(actual.TransformDir(Gf.Vec3d(0, 0, -1)))
        optical_to_base[:3, 3] = np.asarray(actual.ExtractTranslation()) - base_position
        return camera_path, optical_to_base

    def create_rgbd_graph(camera_path):
        og.Controller.edit(
            {'graph_path': '/PalletCameraGraph', 'evaluator_name': 'execution'},
            {
                og.Controller.Keys.CREATE_NODES: [
                    ('Tick', 'omni.graph.action.OnPlaybackTick'),
                    ('RenderProduct', 'isaacsim.core.nodes.IsaacCreateRenderProduct'),
                    ('Rgb', 'isaacsim.ros2.bridge.ROS2CameraHelper'),
                    ('Depth', 'isaacsim.ros2.bridge.ROS2CameraHelper'),
                    ('Info', 'isaacsim.ros2.bridge.ROS2CameraInfoHelper'),
                ],
                og.Controller.Keys.SET_VALUES: [
                    ('RenderProduct.inputs:cameraPrim', [usdrt.Sdf.Path(camera_path)]),
                    ('RenderProduct.inputs:width', 640),
                    ('RenderProduct.inputs:height', 480),
                    ('Rgb.inputs:type', 'rgb'),
                    ('Rgb.inputs:topicName', '/sim_camera/rgb'),
                    ('Rgb.inputs:frameId', 'pallet_camera_optical_frame'),
                    ('Depth.inputs:type', 'depth'),
                    ('Depth.inputs:topicName', '/sim_camera/depth'),
                    ('Depth.inputs:frameId', 'pallet_camera_optical_frame'),
                    ('Info.inputs:topicName', '/sim_camera/camera_info'),
                    ('Info.inputs:frameId', 'pallet_camera_optical_frame'),
                ],
                og.Controller.Keys.CONNECT: [
                    ('Tick.outputs:tick', 'RenderProduct.inputs:execIn'),
                    ('RenderProduct.outputs:execOut', 'Rgb.inputs:execIn'),
                    ('RenderProduct.outputs:execOut', 'Depth.inputs:execIn'),
                    ('RenderProduct.outputs:execOut', 'Info.inputs:execIn'),
                    ('RenderProduct.outputs:renderProductPath', 'Rgb.inputs:renderProductPath'),
                    ('RenderProduct.outputs:renderProductPath', 'Depth.inputs:renderProductPath'),
                    ('RenderProduct.outputs:renderProductPath', 'Info.inputs:renderProductPath'),
                ],
            },
        )

    def write_camera_calibration(path, transform):
        calibration_path = Path(path).expanduser()
        calibration_path.parent.mkdir(parents=True, exist_ok=True)
        payload = {
            'camera': {'optical_to_base_footprint': transform.tolist()},
        }
        with tempfile.NamedTemporaryFile(
            mode='w', encoding='utf-8', dir=calibration_path.parent,
            prefix=f'.{calibration_path.name}.', delete=False,
        ) as stream:
            json.dump(payload, stream, indent=2)
            temporary_path = stream.name
        os.replace(temporary_path, calibration_path)
    import rclpy
    from geometry_msgs.msg import PoseStamped
    from sensor_msgs.msg import JointState
    from std_msgs.msg import String
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

        # SRDF collision exclusions are MoveIt-only. Mirror its fixed
        # arm-to-finger and finger-to-finger exclusions in PhysX.
        rigid_links = {
            p.GetName(): p for p in stage.Traverse()
            if p.GetName() in {'J4', 'J5', 'J6', 'J8', 'J9'}
            and p.HasAPI(UsdPhysics.RigidBodyAPI)
        }
        if set(rigid_links) != {'J4', 'J5', 'J6', 'J8', 'J9'}:
            raise RuntimeError(f'Could not find collision-filter links: {sorted(rigid_links)}')
        for arm_link in ('J4', 'J5', 'J6'):
            filtered_pairs = UsdPhysics.FilteredPairsAPI.Apply(
                rigid_links[arm_link]
            ).CreateFilteredPairsRel()
            for finger_link in ('J8', 'J9'):
                filtered_pairs.AddTarget(Sdf.Path(rigid_links[finger_link].GetPath()))
        UsdPhysics.FilteredPairsAPI.Apply(rigid_links['J8']).CreateFilteredPairsRel().AddTarget(
            Sdf.Path(rigid_links['J9'].GetPath())
        )
        print('PALLET_FILTERED_COLLISIONS J4/J5/J6-to-J8/J9 and J8-to-J9', flush=True)

        # "Stage Lights" only uses lights authored in USD.  Add both ambient
        # and directional illumination so that mode is usable by default.
        dome = UsdLux.DomeLight.Define(stage, '/PalletEnvironmentLight')
        dome.CreateIntensityAttr(250.0)
        dome.CreateColorAttr(Gf.Vec3f(0.85, 0.90, 1.0))
        sun = UsdLux.DistantLight.Define(stage, '/PalletKeyLight')
        sun.CreateIntensityAttr(700.0)
        sun.CreateAngleAttr(4.0)
        UsdGeom.Xformable(sun).AddRotateXYZOp().Set(Gf.Vec3f(-35.0, 25.0, -35.0))

        # Robot workcell. Dimensions are in meters in the Z-up world frame.
        # The table is fixed; the calibration block and cup are dynamic.
        table_spec = workcell['table']
        table_size = np.array(table_spec['size'], dtype=float)
        table_center = np.array([
            *table_spec['center_xy_world'], table_size[2] / 2.0,
        ], dtype=float)
        table_top_thickness = float(table_spec['top_thickness'])
        table_leg_size = float(table_spec['leg_size'])
        table_color = np.array([0.42, 0.24, 0.10])
        table_top_z = table_center[2] + table_size[2] / 2.0
        world.scene.add(FixedCuboid(
            prim_path='/World/Table/Top', name='table_top',
            position=np.array([
                table_center[0], table_center[1],
                table_top_z - table_top_thickness / 2.0,
            ]),
            scale=np.array([table_size[0], table_size[1], table_top_thickness]),
            color=table_color,
        ))
        leg_height = table_size[2] - table_top_thickness
        for index, (sx, sy) in enumerate(((-1, -1), (-1, 1), (1, -1), (1, 1))):
            world.scene.add(FixedCuboid(
                prim_path=f'/World/Table/Leg_{index}', name=f'table_leg_{index}',
                position=np.array([
                    table_center[0] + sx * (table_size[0] / 2.0 - table_leg_size / 2.0),
                    table_center[1] + sy * (table_size[1] / 2.0 - table_leg_size / 2.0),
                    leg_height / 2.0,
                ]),
                scale=np.array([table_leg_size, table_leg_size, leg_height]),
                color=table_color,
            ))

        block_spec = workcell['block']
        block_size = np.array(block_spec['size'], dtype=float)
        block_position = np.array([
            table_center[0] + float(block_spec['center_xy_base'][0]),
            table_center[1] + float(block_spec['center_xy_base'][1]),
            table_top_z + block_size[2] / 2.0 + 0.003,
        ])
        calibration_block = None
        if args.calibration_block:
            calibration_block = world.scene.add(DynamicCuboid(
                prim_path='/World/CalibrationBlock', name='calibration_block',
                position=block_position, scale=block_size,
                mass=float(block_spec['mass_kg']),
                color=np.array([0.85, 0.12, 0.12]),
            ))
        cup_spec = workcell['cup']
        cup_height = float(cup_spec['height'])
        cup_radius = float(cup_spec['radius'])
        if not args.no_cup:
            world.scene.add(DynamicCylinder(
                prim_path='/World/Cup', name='cup',
                position=np.array([
                    *cup_spec['center_xy_world'], table_top_z + cup_height / 2.0 + 0.003,
                ]),
                radius=cup_radius, height=cup_height, mass=float(cup_spec['mass_kg']),
                color=np.array([0.12, 0.45, 0.85]),
            ))
        # Visual-only U-shaped handle. UsdGeom has no Torus schema in Isaac
        # Sim 5.1, so use three supported Cube prims. The cup cylinder remains
        # the conservative collider until a convex handle collider is added.
        handle_color = [Gf.Vec3f(0.12, 0.45, 0.85)]
        handle_thickness = cup_radius * 0.24
        handle_reach = cup_radius * 0.90
        for name, translation, scale in (
            ('Top', (cup_radius, 0.0, cup_height * 0.26),
             (handle_reach, handle_thickness, handle_thickness)),
            ('Bottom', (cup_radius, 0.0, -cup_height * 0.26),
             (handle_reach, handle_thickness, handle_thickness)),
            ('Outer', (cup_radius + handle_reach, 0.0, 0.0),
             (handle_thickness, handle_thickness, cup_height * 0.52)),
        ):
            if not args.no_cup:
                handle = UsdGeom.Cube.Define(stage, f'/World/Cup/Handle{name}')
                handle.CreateSizeAttr(1.0)
                handle.AddTranslateOp().Set(Gf.Vec3d(*translation))
                handle.AddScaleOp().Set(Gf.Vec3f(*scale))
                handle.CreateDisplayColorAttr(handle_color)

        # Use an upright, repeatable view of the complete robot rather than the
        # viewport's previous session camera.
        set_camera_view(
            eye=np.array([1.05, 1.15, 1.45]),
            target=np.array([-0.43, 0.00, 0.95]),
        )
        for p in stage.Traverse():
            name = p.GetName()
            profile_name = 'J8' if name == 'J9' else name
            if profile_name in home and (
                p.IsA(UsdPhysics.RevoluteJoint) or p.IsA(UsdPhysics.PrismaticJoint)
            ):
                drive = UsdPhysics.DriveAPI.Apply(
                    p, 'linear' if p.IsA(UsdPhysics.PrismaticJoint) else 'angular'
                )
                drive.CreateTypeAttr('force')
                drive.CreateMaxForceAttr(limits[name]['effort'])
                drive.CreateStiffnessAttr(float(physics['joints'][profile_name]['stiffness']))
                drive.CreateDampingAttr(float(physics['joints'][profile_name]['damping']))
        robot = world.scene.add(SingleArticulation(prim_path=prim, name='pallet_gripper'))
        world.reset()
        # Mount the fixed robot base at the exact center of the tabletop.
        table_mount_position = np.array([
            table_center[0], table_center[1], table_top_z,
        ])
        robot.set_world_pose(position=table_mount_position)
        mounted_position, _ = robot.get_world_pose()
        if not np.allclose(mounted_position, table_mount_position, atol=1e-4):
            raise RuntimeError(
                'Robot table mount pose was not applied: '
                f'expected={table_mount_position.tolist()} actual={mounted_position.tolist()}'
            )
        print(f'PALLET_TABLE_MOUNT position_m={mounted_position.tolist()}', flush=True)
        if args.rgbd_camera:
            camera_path, optical_to_base = add_rgbd_camera(stage, table_mount_position)
            create_rgbd_graph(camera_path)
            write_camera_calibration(args.camera_calibration_file, optical_to_base)
            print(
                'PALLET_RGBD_READY topics=/sim_camera/rgb,/sim_camera/depth,'
                '/sim_camera/camera_info '
                f'calibration={args.camera_calibration_file}', flush=True,
            )
        if args.disable_gravity:
            robot.disable_gravity()
        else:
            robot.enable_gravity()
        dof_names = list(robot.dof_names)
        controlled_names = list(home)
        if len(controlled_names) != 7 or set(controlled_names) != {
            'J1', 'J2', 'J3', 'J4', 'J5', 'J6', 'J8',
        }:
            raise RuntimeError(f'Expected exactly seven controlled joints, got {controlled_names}')
        if not set(controlled_names).issubset(dof_names) or set(dof_names) - set(controlled_names) != {'J9'}:
            raise RuntimeError(f'Unexpected PhysX DOFs: {dof_names}')
        controlled_indices = [dof_names.index(name) for name in controlled_names]
        target = np.array([float(home.get(n, home['J8'])) for n in dof_names])
        jaw_control = {'symmetric': True}
        rated_effort = np.array([float(limits[n]['effort']) for n in dof_names])
        max_measured_effort = np.zeros(len(dof_names))
        robot.set_joint_positions(target)
        robot.set_joint_velocities(np.zeros(len(dof_names)))
        controller = robot.get_articulation_controller()
        # Do not expose the URDF import pose to all_in_one. Hold the configured
        # MoveIt home state until physics has settled, then start ROS feedback.
        settle_steps = max(
            1,
            math.ceil(float(physics['startup_settle_seconds']) / float(physics['physics_dt'])),
        )
        for _ in range(settle_steps):
            controller.apply_action(ArticulationAction(joint_positions=target))
            world.step(render=not args.headless or args.rgbd_camera)
        settled_position = np.asarray(robot.get_joint_positions(), dtype=float)
        home_error = np.abs(settled_position - target)
        tolerances = np.array([
            float(physics['home_tolerance_gripper_m'] if n in {'J8', 'J9'}
                  else physics['home_tolerance_arm_rad'])
            for n in dof_names
        ])
        if np.any(home_error > tolerances):
            raise RuntimeError(
                'Robot did not settle at the configured home position: '
                f'target={target.tolist()} actual={settled_position.tolist()} '
                f'error={home_error.tolist()} tolerance={tolerances.tolist()}'
            )
        print(
            'PALLET_HOME_READY '
            f'positions={dict(zip(controlled_names, settled_position[controlled_indices].tolist()))} '
            f'error={home_error.tolist()} units=arm_rad,J8_J9_m',
            flush=True,
        )
        rclpy.init()
        node = rclpy.create_node('pallet_gripper_isaac_endpoint')
        pub = node.create_publisher(JointState, '/isaac_joint_states', 10)
        block_pose_pub = (
            node.create_publisher(PoseStamped, '/isaac_calibration_block_pose', 10)
            if calibration_block is not None else None
        )
        sim_fault_state = {
            'handled_ids': set(),
            'suppress_block_pose_until': 0.0,
        }

        def sim_fault_command(msg):
            if not fault_injection_enabled or calibration_block is None:
                return
            try:
                payload = json.loads(msg.data)
                fault_id = payload['fault_id']
                code = payload['code']
                published_at = float(payload['published_at_unix'])
            except (KeyError, TypeError, ValueError, json.JSONDecodeError):
                node.get_logger().error('Rejected malformed simulation fault command')
                return
            if (not isinstance(fault_id, str) or not fault_id or
                    fault_id in sim_fault_state['handled_ids'] or
                    abs(time.time() - published_at) > 2.0):
                return
            sim_fault_state['handled_ids'].add(fault_id)
            block_world_position, _ = calibration_block.get_world_pose()
            if code == 'BLOCK_DROP_DETECTED':
                dropped_position = np.asarray(block_world_position, dtype=float)
                dropped_position[2] = table_top_z + block_size[2] / 2.0 + 0.002
                calibration_block.set_world_pose(position=dropped_position)
                calibration_block.set_linear_velocity(np.array([0.0, 0.0, -0.4]))
            elif code == 'TILT_DETECTED':
                half_angle = math.radians(30.0) / 2.0
                calibration_block.set_world_pose(
                    position=np.asarray(block_world_position, dtype=float),
                    orientation=np.array([
                        math.cos(half_angle), math.sin(half_angle), 0.0, 0.0,
                    ]),
                )
                calibration_block.set_angular_velocity(np.array([1.5, 0.0, 0.0]))
            elif code == 'TELEMETRY_STALE':
                sim_fault_state['suppress_block_pose_until'] = time.monotonic() + 2.0
            else:
                node.get_logger().error(f'Rejected unsupported simulation fault code {code!r}')
                return
            print(
                f'PALLET_SIM_FAULT_APPLIED id={fault_id} code={code} '
                f'phase={payload.get("phase", "unknown")}',
                flush=True,
            )

        node.create_subscription(String, '/pallet/sim_fault_commands', sim_fault_command, 10)
        def command(msg):
            if len(msg.name) != len(msg.position) or len(set(msg.name)) != len(msg.name):
                return
            if any(n not in dof_names or not math.isfinite(v) or not limits[n]['lower'] <= v <= limits[n]['upper']
                   for n, v in zip(msg.name, msg.position)):
                node.get_logger().error('Rejected invalid joint command')
                return
            for n, v in zip(msg.name, msg.position):
                target[dof_names.index(n)] = v
            # Normal goals send J8 only and couple the jaws. Cancellation may
            # explicitly hold both measured positions after asymmetric contact.
            if 'J8' in msg.name and 'J9' not in msg.name:
                target[dof_names.index('J9')] = target[dof_names.index('J8')]
                jaw_control['symmetric'] = True
            elif 'J9' in msg.name:
                jaw_control['symmetric'] = False
        node.create_subscription(JointState, '/isaac_joint_commands', command, 10)
        gravity = 0.0 if args.disable_gravity else float(physics['gravity_m_s2'])
        print(
            f'PALLET_ISAAC_READY joints={controlled_names} '
            f'physics_dofs={dof_names} gravity_m_s2={gravity} base=fixed '
            f'fault_injection_enabled={fault_injection_enabled}',
            flush=True,
        )
        started = time.monotonic()
        last_gripper_diagnostic = 0.0
        last_block_pose_publish = 0.0
        while app.is_running() and (not args.seconds or time.monotonic() - started < args.seconds):
            tick = time.monotonic()
            rclpy.spin_once(node, timeout_sec=0)
            applied_target = target.copy()
            observed_before_step = np.asarray(robot.get_joint_positions(), dtype=float)
            if jaw_control['symmetric']:
                j8_index = dof_names.index('J8')
                j9_index = dof_names.index('J9')
                applied_target[j8_index], applied_target[j9_index] = symmetric_jaw_targets(
                    float(target[j8_index]),
                    float(observed_before_step[j8_index]),
                    float(observed_before_step[j9_index]),
                    float(limits['J8']['lower']), float(limits['J8']['upper']),
                    symmetry_gain, symmetry_max_correction,
                )
            controller.apply_action(ArticulationAction(joint_positions=applied_target))
            world.step(render=not args.headless or args.rgbd_camera)
            if (block_pose_pub is not None and
                    time.monotonic() >= sim_fault_state['suppress_block_pose_until'] and
                    time.monotonic() - last_block_pose_publish >= 0.1):
                block_world_position, block_wxyz = calibration_block.get_world_pose()
                block_pose = PoseStamped()
                block_pose.header.stamp = node.get_clock().now().to_msg()
                block_pose.header.frame_id = 'base_footprint'
                block_base_position = np.asarray(block_world_position) - table_mount_position
                block_pose.pose.position.x = float(block_base_position[0])
                block_pose.pose.position.y = float(block_base_position[1])
                block_pose.pose.position.z = float(block_base_position[2])
                block_pose.pose.orientation.x = float(block_wxyz[1])
                block_pose.pose.orientation.y = float(block_wxyz[2])
                block_pose.pose.orientation.z = float(block_wxyz[3])
                block_pose.pose.orientation.w = float(block_wxyz[0])
                block_pose_pub.publish(block_pose)
                last_block_pose_publish = time.monotonic()
            measured_effort = np.abs(np.asarray(robot.get_measured_joint_efforts(), dtype=float))
            max_measured_effort = np.maximum(max_measured_effort, measured_effort)
            if (abs(float(target[dof_names.index('J8')]) -
                    float(robot.get_joint_positions()[dof_names.index('J8')])) > 0.002
                    and time.monotonic() - last_gripper_diagnostic > 1.0):
                observed = robot.get_joint_positions()
                print(
                    'PALLET_GRIPPER_TRACKING '
                    f"target_m={[float(target[dof_names.index(n)]) for n in ('J8', 'J9')]} "
                    f"applied_target_m={[float(applied_target[dof_names.index(n)]) for n in ('J8', 'J9')]} "
                    f"actual_m={[float(observed[dof_names.index(n)]) for n in ('J8', 'J9')]} "
                    f"effort={[float(measured_effort[dof_names.index(n)]) for n in ('J8', 'J9')]}",
                    flush=True,
                )
                last_gripper_diagnostic = time.monotonic()
            msg = JointState()
            msg.header.stamp = node.get_clock().now().to_msg()
            msg.name = dof_names
            msg.position = [float(v) for v in robot.get_joint_positions()]
            msg.velocity = [float(v) for v in robot.get_joint_velocities()]
            msg.effort = [float(v) for v in np.asarray(
                robot.get_measured_joint_efforts(), dtype=float
            )]
            pub.publish(msg)
            time.sleep(max(0, 1/240 - (time.monotonic() - tick)))
        final_position = np.asarray(robot.get_joint_positions(), dtype=float)
        final_velocity = np.asarray(robot.get_joint_velocities(), dtype=float)
        final_position = final_position[controlled_indices]
        final_velocity = final_velocity[controlled_indices]
        controlled_target = target[controlled_indices]
        controlled_effort = max_measured_effort[controlled_indices]
        controlled_rated_effort = rated_effort[controlled_indices]
        print('PALLET_FINAL ' + str(dict(zip(controlled_names, final_position.tolist()))), flush=True)
        print('PALLET_FINAL_ERROR_ARM_RAD_GRIPPER_M ' + str(dict(zip(controlled_names, (final_position - controlled_target).tolist()))), flush=True)
        print('PALLET_FINAL_VELOCITY_ARM_RAD_S_GRIPPER_M_S ' + str(dict(zip(controlled_names, final_velocity.tolist()))), flush=True)
        print('PALLET_MAX_MEASURED_EFFORT_ARM_NM_GRIPPER_N ' + str(dict(zip(controlled_names, controlled_effort.tolist()))), flush=True)
        print('PALLET_MAX_MEASURED_TO_RATED ' + str(dict(zip(controlled_names, (controlled_effort / controlled_rated_effort).tolist()))), flush=True)
        print('PALLET_RATED_EFFORT_ARM_NM_GRIPPER_N ' + str(dict(zip(controlled_names, controlled_rated_effort.tolist()))), flush=True)
        node.destroy_node()
        rclpy.shutdown()
except Exception:
    traceback.print_exc()
    raise
finally:
    app.close()
