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
parser.add_argument('--block-contact-offset-m', type=float, default=-1.0,
                    help='Experimental block contact offset; -1 keeps Isaac default')
parser.add_argument('--block-static-friction', type=float, default=-1.0,
                    help='Experimental static friction; -1 keeps Isaac default')
parser.add_argument('--jaw-friction', type=float, default=-1.0,
                    help='Experimental explicit jaw static/dynamic friction; -1 preserves binding')
parser.add_argument('--contact-geometry-log', choices=('true', 'false'), default='false',
                    help='Log actual jaw collider poses and red block geometry at 1 Hz')
parser.add_argument('--solver-velocity-iterations', type=int, default=-1,
                    help='Experimental articulation velocity iterations; -1 preserves default')
parser.add_argument('--external-forces-every-iteration',
                    choices=('default', 'true', 'false'), default='default',
                    help='Experimental TGS force application; default preserves scene setting')
parser.add_argument(
    '--block-layout', choices=('single', 'three'), default='single',
    help='Spawn the primary red block only, or red/green/blue blocks.',
)
parser.add_argument(
    '--fault-injection-enabled', choices=('true', 'false'), default='false',
    help='Enable the test-only /pallet/sim_fault_commands endpoint',
)
parser.add_argument('--rgbd-camera', action='store_true', help='Publish simulated RGB, depth, and CameraInfo topics')
parser.add_argument('--rgbd-clock', choices=('simulation', 'system'), default='simulation',
                    help='RGB/depth/CameraInfo header clock; legacy simulation time by default')
parser.add_argument('--rgbd-resolution', choices=('640x480', '1280x960'), default='640x480',
                    help='Optional RGB-D render resolution; default preserves legacy images')
parser.add_argument('--camera-view', choices=('source', 'workcell'), default='source',
                    help='Fixed RGB-D view: legacy source row or source plus placement workspace')
parser.add_argument('--camera-calibration-file', default='/tmp/pallet_camera_calibration.json')
parser.add_argument('--no-cup', action='store_true')
parser.add_argument('--seconds', type=float, default=0)
parser.add_argument('--diagnostic-block-pose', nargs=4, action='append',
                    metavar=('OBJECT', 'X_M', 'Y_M', 'YAW_RAD'),
                    help='Bounded simulation-only static fixture; never forwarded to vision.')
parser.add_argument(
    '--disable-gravity', action='store_true',
    help='Diagnostic mode only: disable gravity on the articulation',
)
args = parser.parse_args()
if args.contact_geometry_log == 'true' and not args.calibration_block:
    parser.error('--contact-geometry-log true requires --calibration-block')
if args.diagnostic_block_pose and not (
        args.calibration_block and args.block_layout == 'three' and args.rgbd_camera
        and math.isfinite(args.seconds) and 0 < args.seconds <= 120
        and args.fault_injection_enabled == 'false'):
    parser.error('--diagnostic-block-pose requires three calibration blocks, RGB-D, seconds in (0,120] and no fault injection')
if args.solver_velocity_iterations != -1 and not 1 <= args.solver_velocity_iterations <= 64:
    parser.error('--solver-velocity-iterations must be -1 or within [1, 64]')
if (not math.isfinite(args.block_contact_offset_m) or
        (args.block_contact_offset_m != -1.0 and
         not 0.0 < args.block_contact_offset_m <= 0.01)):
    parser.error('--block-contact-offset-m must be -1 or within (0, 0.01]')
if (not math.isfinite(args.block_static_friction) or
        (args.block_static_friction != -1.0 and
         not 0.0 <= args.block_static_friction <= 2.0)):
    parser.error('--block-static-friction must be -1 or within [0, 2]')
if (not math.isfinite(args.jaw_friction) or
        (args.jaw_friction != -1.0 and not 0.0 <= args.jaw_friction <= 2.0)):
    parser.error('--jaw-friction must be -1 or within [0, 2]')
fault_injection_enabled = args.fault_injection_enabled == 'true'
root = Path(__file__).resolve().parents[1]
source = root.parent / 'pallet_gripper/urdf/pallet_gripper.urdf'
import yaml
home = yaml.safe_load((root / 'config/initial_positions.yaml').read_text())['initial_positions']
physics = yaml.safe_load((root / 'config/isaac_physics.yaml').read_text())
workcell = yaml.safe_load((root / 'config/workcell.yaml').read_text())
from workcell_scenario import (
    block_scenario, diagnostic_block_poses, gripper_effort_limit, gripper_symmetry_config,
    joint_state_snapshot, rgbd_camera_offsets, rgbd_camera_resolution, symmetric_jaw_targets,
)
try:
    workcell['block'] = block_scenario(
        workcell, args.block_center_xy, args.block_size, args.block_mass_kg,
    )
    fixture_poses = diagnostic_block_poses(args.diagnostic_block_pose, workcell)
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
    from pxr import Gf, PhysxSchema, Sdf, Usd, UsdGeom, UsdLux, UsdPhysics, UsdShade
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
        eye_offset, target_offset = rgbd_camera_offsets(args.camera_view)
        eye = np.asarray(base_position) + np.asarray(eye_offset)
        target = np.asarray(base_position) + np.asarray(target_offset)
        print(f'PALLET_RGBD_VIEW view={args.camera_view} eye_base_m={eye_offset} '
              f'target_base_m={target_offset}', flush=True)
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
        width, height = rgbd_camera_resolution(args.rgbd_resolution)
        print(f'PALLET_RGBD_RESOLUTION width={width} height={height}', flush=True)
        print(f'PALLET_RGBD_CLOCK mode={args.rgbd_clock}', flush=True)
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
                    ('RenderProduct.inputs:width', width),
                    ('RenderProduct.inputs:height', height),
                    ('Rgb.inputs:type', 'rgb'),
                    ('Rgb.inputs:topicName', '/sim_camera/rgb'),
                    ('Rgb.inputs:frameId', 'pallet_camera_optical_frame'),
                    ('Depth.inputs:type', 'depth'),
                    ('Depth.inputs:topicName', '/sim_camera/depth'),
                    ('Depth.inputs:frameId', 'pallet_camera_optical_frame'),
                    ('Info.inputs:topicName', '/sim_camera/camera_info'),
                    ('Info.inputs:frameId', 'pallet_camera_optical_frame'),
                    ('Rgb.inputs:useSystemTime', args.rgbd_clock == 'system'),
                    ('Depth.inputs:useSystemTime', args.rgbd_clock == 'system'),
                    ('Info.inputs:useSystemTime', args.rgbd_clock == 'system'),
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
    scene_prim = omni.usd.get_context().get_stage().GetPrimAtPath(
        world.get_physics_context().prim_path)
    scene_api = PhysxSchema.PhysxSceneAPI.Apply(scene_prim)
    original_external_forces = scene_api.GetEnableExternalForcesEveryIterationAttr().Get()
    if args.external_forces_every_iteration != 'default':
        requested_external_forces = args.external_forces_every_iteration == 'true'
        scene_api.CreateEnableExternalForcesEveryIterationAttr().Set(requested_external_forces)
        if scene_api.GetEnableExternalForcesEveryIterationAttr().Get() != requested_external_forces:
            raise RuntimeError('External forces every iteration override not applied')
    print('PALLET_SCENE_SOLVER_AUDIT ' + json.dumps({
        'scene': str(scene_prim.GetPath()),
        'solver_type': scene_api.GetSolverTypeAttr().Get(),
        'external_forces_every_iteration': scene_api.GetEnableExternalForcesEveryIterationAttr().Get(),
        'original_external_forces_every_iteration': original_external_forces,
    }), flush=True)
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
        block_objects = {}
        if args.calibration_block:
            calibration_block = world.scene.add(DynamicCuboid(
                prim_path='/World/CalibrationBlock', name='calibration_block',
                position=block_position, scale=block_size,
                mass=float(block_spec['mass_kg']),
                color=np.array([0.85, 0.12, 0.12]),
            ))
            block_objects['red_block'] = calibration_block
            if args.block_layout == 'three':
                additional_blocks = {
                    'green_block': (-0.08, np.array([0.12, 0.75, 0.18])),
                    'blue_block': (-0.16, np.array([0.05, 0.05, 0.95])),
                }
                for block_name, (y_offset, color) in additional_blocks.items():
                    position = np.array([
                        block_position[0], block_position[1] + y_offset,
                        block_position[2],
                    ])
                    block_objects[block_name] = world.scene.add(DynamicCuboid(
                        prim_path=f'/World/{block_name}', name=block_name,
                        position=position, scale=block_size,
                        mass=float(block_spec['mass_kg']), color=color,
                    ))
            for name, (x, y, yaw) in fixture_poses.items():
                block_objects[name].set_default_state(
                    position=np.array([table_center[0]+x, table_center[1]+y, block_position[2]]),
                    orientation=np.array([math.cos(yaw/2), 0., 0., math.sin(yaw/2)]),
                )
                block_objects[name].set_world_pose(
                    position=np.array([table_center[0]+x, table_center[1]+y, block_position[2]]),
                    orientation=np.array([math.cos(yaw/2), 0., 0., math.sin(yaw/2)]),
                )
            if fixture_poses:
                print('PALLET_DIAGNOSTIC_BLOCK_POSES '+json.dumps(fixture_poses, sort_keys=True), flush=True)
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
        if args.block_contact_offset_m != -1.0:
            for block_object in block_objects.values():
                block_object.set_contact_offset(args.block_contact_offset_m)
        if args.block_static_friction != -1.0:
            for block_object in block_objects.values():
                block_prim = stage.GetPrimAtPath(block_object.prim_path)
                material, _ = UsdShade.MaterialBindingAPI(block_prim).ComputeBoundMaterial('physics')
                if not material:
                    raise RuntimeError(f'Block physics material missing: {block_object.prim_path}')
                UsdPhysics.MaterialAPI(material.GetPrim()).CreateStaticFrictionAttr(
                    args.block_static_friction)
        jaw_material = None
        # The importer returns the articulation root joint, not the link parent.
        robot_link_root = str(stage.GetPrimAtPath(prim).GetParent().GetPath())
        if args.jaw_friction != -1.0:
            jaw_material = UsdShade.Material.Define(stage, '/World/Physics_Materials/jaw_experiment')
            jaw_physics = UsdPhysics.MaterialAPI.Apply(jaw_material.GetPrim())
            jaw_physics.CreateStaticFrictionAttr(args.jaw_friction)
            jaw_physics.CreateDynamicFrictionAttr(args.jaw_friction)
            jaw_physics.CreateRestitutionAttr(0.0)
            for jaw in ('J8', 'J9'):
                jaw_prim = stage.GetPrimAtPath(f'{robot_link_root}/{jaw}')
                if not jaw_prim.IsValid():
                    raise RuntimeError(f'Jaw prim missing: {robot_link_root}/{jaw}')
                UsdShade.MaterialBindingAPI.Apply(jaw_prim).Bind(
                    jaw_material, UsdShade.Tokens.strongerThanDescendants, 'physics')
        audited_jaws = set()
        jaw_colliders = []
        # Read the composed physics binding at each collider, including inherited
        # materials. Missing bindings remain explicit rather than guessed values.
        for collider in stage.Traverse(Usd.TraverseInstanceProxies()):
            collider_path = str(collider.GetPath())
            if not collider.HasAPI(UsdPhysics.CollisionAPI):
                continue
            if not (any(f'/{jaw}/' in collider_path for jaw in ('J8', 'J9')) or
                    collider_path in {'/World/CalibrationBlock', '/World/red_block',
                                      '/World/green_block', '/World/blue_block'}):
                continue
            material, _ = UsdShade.MaterialBindingAPI(collider).ComputeBoundMaterial('physics')
            if collider_path.startswith(f'{robot_link_root}/') and any(
                    f'/{jaw}/' in collider_path for jaw in ('J8', 'J9')):
                jaw_colliders.append(collider)
            if jaw_material is not None and collider_path.startswith(f'{robot_link_root}/'):
                for jaw in ('J8', 'J9'):
                    if f'/{jaw}/' in collider_path:
                        if not material or material.GetPath() != jaw_material.GetPath():
                            raise RuntimeError(f'Jaw material binding not applied: {collider_path}')
                        audited_jaws.add(jaw)
            material_prim = material.GetPrim() if material else None
            def material_value(name):
                if material_prim is None:
                    return None
                return material_prim.GetAttribute(name).Get()
            print('PALLET_CONTACT_AUDIT ' + json.dumps({
                'collider': collider_path,
                'material': str(material.GetPath()) if material else None,
                'static_friction': material_value('physics:staticFriction'),
                'dynamic_friction': material_value('physics:dynamicFriction'),
                'restitution': material_value('physics:restitution'),
                'contact_offset_m': collider.GetAttribute('physxCollision:contactOffset').Get(),
                'rest_offset_m': collider.GetAttribute('physxCollision:restOffset').Get(),
            }), flush=True)
        if jaw_material is not None and audited_jaws != {'J8', 'J9'}:
            raise RuntimeError('Both jaw colliders must be audited before motion')
        if args.contact_geometry_log == 'true' and len(jaw_colliders) != 2:
            raise RuntimeError('Geometry logging requires exactly two jaw colliders')
        robot = world.scene.add(SingleArticulation(prim_path=prim, name='pallet_gripper'))
        world.reset()
        original_velocity_iterations = int(robot.get_solver_velocity_iteration_count())
        if args.solver_velocity_iterations != -1:
            robot.set_solver_velocity_iteration_count(args.solver_velocity_iterations)
            if int(robot.get_solver_velocity_iteration_count()) != args.solver_velocity_iterations:
                raise RuntimeError('Articulation velocity iteration override not applied')
        print('PALLET_SOLVER_AUDIT ' + json.dumps({
            'articulation': prim,
            'position_iterations': int(robot.get_solver_position_iteration_count()),
            'velocity_iterations': int(robot.get_solver_velocity_iteration_count()),
            'original_velocity_iterations': original_velocity_iterations,
        }), flush=True)
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
        if block_objects:
            settled_blocks = {}
            for block_name, block_object in block_objects.items():
                block_world_position, block_world_orientation = block_object.get_world_pose()
                settled_blocks[block_name] = (
                    np.asarray(block_world_position, dtype=float) - table_mount_position
                ).round(6).tolist()
                if block_name in fixture_poses:
                    x, y, yaw = fixture_poses[block_name]
                    expected_position = np.array([x, y, block_size[2]/2])
                    observed_position = np.asarray(block_world_position)-table_mount_position
                    expected_orientation = np.array([math.cos(yaw/2), 0., 0., math.sin(yaw/2)])
                    observed_orientation = np.asarray(block_world_orientation)
                    dot = abs(float(expected_orientation @ observed_orientation/np.linalg.norm(observed_orientation)))
                    if (np.linalg.norm(observed_position-expected_position) > .0001
                            or 2*math.acos(min(1., dot)) > .001):
                        raise RuntimeError(f'Diagnostic fixture changed during reset/settling: {block_name}')
            print(
                'PALLET_BLOCK_LAYOUT_READY base_positions_m='
                f'{json.dumps(settled_blocks, sort_keys=True)}',
                flush=True,
            )
        settled_snapshot = joint_state_snapshot(
            robot.get_joint_positions(),
            robot.get_joint_velocities(),
            robot.get_measured_joint_efforts(),
            len(dof_names),
        )
        if settled_snapshot is None:
            raise RuntimeError('Articulation feedback unavailable after startup settling')
        settled_position = np.asarray(settled_snapshot[0], dtype=float)
        last_joint_position = settled_position.copy()
        last_joint_velocity = np.asarray(settled_snapshot[1], dtype=float)
        last_joint_effort = np.abs(np.asarray(settled_snapshot[2], dtype=float))
        max_measured_effort = np.maximum(max_measured_effort, last_joint_effort)
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
        block_pose_pubs = {
            name: node.create_publisher(PoseStamped, f'/isaac_{name}_pose', 10)
            for name in block_objects
        }
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
        if not fixture_poses:
            node.create_subscription(JointState, '/isaac_joint_commands', command, 10)
        else:
            print('PALLET_DIAGNOSTIC_COMMANDS_DISABLED', flush=True)
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
        last_geometry_log = 0.0
        while app.is_running() and (not args.seconds or time.monotonic() - started < args.seconds):
            tick = time.monotonic()
            rclpy.spin_once(node, timeout_sec=0)
            applied_target = target.copy()
            if jaw_control['symmetric']:
                j8_index = dof_names.index('J8')
                j9_index = dof_names.index('J9')
                applied_target[j8_index], applied_target[j9_index] = symmetric_jaw_targets(
                    float(target[j8_index]),
                    float(last_joint_position[j8_index]),
                    float(last_joint_position[j9_index]),
                    float(limits['J8']['lower']), float(limits['J8']['upper']),
                    symmetry_gain, symmetry_max_correction,
                )
            controller.apply_action(ArticulationAction(joint_positions=applied_target))
            world.step(render=not args.headless or args.rgbd_camera)
            snapshot = joint_state_snapshot(
                robot.get_joint_positions(),
                robot.get_joint_velocities(),
                robot.get_measured_joint_efforts(),
                len(dof_names),
            )
            if snapshot is None:
                print(
                    'PALLET_ISAAC_FEEDBACK_STOPPED reason=articulation_unavailable',
                    flush=True,
                )
                break
            last_joint_position = np.asarray(snapshot[0], dtype=float)
            last_joint_velocity = np.asarray(snapshot[1], dtype=float)
            last_joint_effort = np.abs(np.asarray(snapshot[2], dtype=float))
            if args.contact_geometry_log == 'true' and time.monotonic() - last_geometry_log >= 1.0:
                block = block_objects.get('red_block')
                if block is None:
                    raise RuntimeError('Contact geometry logging requires red block')
                block_position, block_orientation = block.get_world_pose()
                pads = []
                for collider in jaw_colliders:
                    transform = UsdGeom.Xformable(collider).ComputeLocalToWorldTransform(
                        Usd.TimeCode.Default())
                    center = np.asarray(transform.Transform(Gf.Vec3d(0, 0, 0)))
                    vectors = [np.asarray(transform.TransformDir(Gf.Vec3d(*axis)))
                               for axis in ((1, 0, 0), (0, 1, 0), (0, 0, 1))]
                    scales = np.asarray([np.linalg.norm(v) for v in vectors])
                    if not np.all(np.isfinite(scales)) or np.any(scales <= 0):
                        raise RuntimeError('Invalid pad geometry transform')
                    axes = [v / scale for v, scale in zip(vectors, scales)]
                    cube = UsdGeom.Cube(collider)
                    if not cube:
                        raise RuntimeError('Contact geometry logger expects box pads')
                    pads.append({
                        'collider': str(collider.GetPath()),
                        'center_base_m': (center - table_mount_position).tolist(),
                        'axes_base': [v.tolist() for v in axes],
                        'size_m': (scales * float(cube.GetSizeAttr().Get())).tolist(),
                        'block_center_pad_m': [float(np.dot(block_position - center, v))
                                               for v in axes],
                    })
                print('PALLET_CONTACT_GEOMETRY ' + json.dumps({
                    'seconds': time.monotonic() - started,
                    'unix_seconds': time.time(), 'frame': 'base_footprint', 'unit': 'm',
                    'simulation_time_seconds': float(world.current_time),
                    'physics_step_index': int(world.current_time_step_index),
                    'physics_dt_seconds': float(world.get_physics_dt()),
                    'rendering_dt_seconds': float(world.get_rendering_dt()),
                    'block_position_m': (block_position - table_mount_position).tolist(),
                    'block_orientation_wxyz': np.asarray(block_orientation).tolist(),
                    'blocks': {
                        name: {
                            'position_m': (np.asarray(pose[0]) - table_mount_position).tolist(),
                            'orientation_wxyz': np.asarray(pose[1]).tolist(),
                        }
                        for name, obj in block_objects.items()
                        for pose in (obj.get_world_pose(),)
                    },
                    'jaw_position_m': {name: float(last_joint_position[dof_names.index(name)])
                                       for name in ('J8', 'J9')},
                    'pads': pads,
                }), flush=True)
                last_geometry_log = time.monotonic()
            if (block_pose_pub is not None and
                    time.monotonic() >= sim_fault_state['suppress_block_pose_until'] and
                    time.monotonic() - last_block_pose_publish >= 0.1):
                stamp = node.get_clock().now().to_msg()
                for block_name, block_object in block_objects.items():
                    block_world_position, block_wxyz = block_object.get_world_pose()
                    block_pose = PoseStamped()
                    block_pose.header.stamp = stamp
                    block_pose.header.frame_id = 'base_footprint'
                    block_base_position = np.asarray(block_world_position) - table_mount_position
                    block_pose.pose.position.x = float(block_base_position[0])
                    block_pose.pose.position.y = float(block_base_position[1])
                    block_pose.pose.position.z = float(block_base_position[2])
                    block_pose.pose.orientation.x = float(block_wxyz[1])
                    block_pose.pose.orientation.y = float(block_wxyz[2])
                    block_pose.pose.orientation.z = float(block_wxyz[3])
                    block_pose.pose.orientation.w = float(block_wxyz[0])
                    block_pose_pubs[block_name].publish(block_pose)
                    if block_name == 'red_block':
                        block_pose_pub.publish(block_pose)
                last_block_pose_publish = time.monotonic()
            max_measured_effort = np.maximum(max_measured_effort, last_joint_effort)
            if (abs(float(target[dof_names.index('J8')]) -
                    float(last_joint_position[dof_names.index('J8')])) > 0.002
                    and time.monotonic() - last_gripper_diagnostic > 1.0):
                print(
                    'PALLET_GRIPPER_TRACKING '
                    f"target_m={[float(target[dof_names.index(n)]) for n in ('J8', 'J9')]} "
                    f"applied_target_m={[float(applied_target[dof_names.index(n)]) for n in ('J8', 'J9')]} "
                    f"actual_m={[float(last_joint_position[dof_names.index(n)]) for n in ('J8', 'J9')]} "
                    f"effort={[float(last_joint_effort[dof_names.index(n)]) for n in ('J8', 'J9')]}",
                    flush=True,
                )
                last_gripper_diagnostic = time.monotonic()
            msg = JointState()
            msg.header.stamp = node.get_clock().now().to_msg()
            msg.name = dof_names
            msg.position = [float(v) for v in last_joint_position]
            msg.velocity = [float(v) for v in last_joint_velocity]
            msg.effort = [float(v) for v in last_joint_effort]
            pub.publish(msg)
            time.sleep(max(0, 1/240 - (time.monotonic() - tick)))
        final_position = last_joint_position[controlled_indices]
        final_velocity = last_joint_velocity[controlled_indices]
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
