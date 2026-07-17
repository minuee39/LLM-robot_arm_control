import argparse

from isaacsim import SimulationApp

parser = argparse.ArgumentParser()
parser.add_argument(
    "--command",
    default="red block on green block",
    help="Natural-language command understood by command_parser.py",
)
parser.add_argument("--keep-running", action="store_true", help="Keep Isaac Sim open after pick-place completes")
args, _ = parser.parse_known_args()

simulation_app = SimulationApp(
    {
        "headless": False,
        "renderer": "RayTracedLighting",
    }
)

import omni.graph.core as og
import omni.timeline
import omni.usd
import usdrt.Sdf
from controller.pick_place import PickPlaceController
from isaacsim.core.api import World
from isaacsim.core.api.objects import DynamicCuboid
from isaacsim.core.utils.extensions import enable_extension
from isaacsim.core.utils.viewports import set_camera_view
from pxr import Gf, UsdGeom
from scene_config import BLOCK_SIZE, OBJECT_COLORS, OBJECT_POSITIONS
from tasks.pick_place import PickPlace

from command_parser import parse_user_command_with_memory, validate_command
from grasp_policy import FixedGraspPolicy, GraspRequest
from scene_manager import SceneManager


TASK_NAME = "ur10e_pick_place_camera"
CAMERA_PRIM_PATH = "/World/RGBD_Camera"
FRAME_ID = "sim_camera"
WIDTH = 640
HEIGHT = 480


def create_camera():
    stage = omni.usd.get_context().get_stage()
    camera_prim = UsdGeom.Camera.Define(stage, CAMERA_PRIM_PATH)
    camera_prim.GetFocalLengthAttr().Set(24.0)
    camera_prim.GetClippingRangeAttr().Set(Gf.Vec2f(0.01, 10.0))

    set_camera_view(
        eye=[0.0, 0.95, 2.9],
        target=[0.0, 0.32, 0.08],
        camera_prim_path=CAMERA_PRIM_PATH,
    )


def create_ros_camera_graph():
    og.Controller.edit(
        {"graph_path": "/ActionGraph", "evaluator_name": "execution"},
        {
            og.Controller.Keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("CreateRenderProduct", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
                ("RgbPublisher", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                ("DepthPublisher", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                ("PointCloudPublisher", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                ("CameraInfoPublisher", "isaacsim.ros2.bridge.ROS2CameraInfoHelper"),
            ],
            og.Controller.Keys.SET_VALUES: [
                ("CreateRenderProduct.inputs:cameraPrim", [usdrt.Sdf.Path(CAMERA_PRIM_PATH)]),
                ("CreateRenderProduct.inputs:width", WIDTH),
                ("CreateRenderProduct.inputs:height", HEIGHT),
                ("RgbPublisher.inputs:type", "rgb"),
                ("RgbPublisher.inputs:topicName", "/sim_camera/rgb"),
                ("RgbPublisher.inputs:frameId", FRAME_ID),
                ("DepthPublisher.inputs:type", "depth"),
                ("DepthPublisher.inputs:topicName", "/sim_camera/depth"),
                ("DepthPublisher.inputs:frameId", FRAME_ID),
                ("PointCloudPublisher.inputs:type", "depth_pcl"),
                ("PointCloudPublisher.inputs:topicName", "/sim_camera/points"),
                ("PointCloudPublisher.inputs:frameId", FRAME_ID),
                ("CameraInfoPublisher.inputs:topicName", "/sim_camera/camera_info"),
                ("CameraInfoPublisher.inputs:frameId", FRAME_ID),
            ],
            og.Controller.Keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", "CreateRenderProduct.inputs:execIn"),
                ("CreateRenderProduct.outputs:execOut", "RgbPublisher.inputs:execIn"),
                ("CreateRenderProduct.outputs:execOut", "DepthPublisher.inputs:execIn"),
                ("CreateRenderProduct.outputs:execOut", "PointCloudPublisher.inputs:execIn"),
                ("CreateRenderProduct.outputs:execOut", "CameraInfoPublisher.inputs:execIn"),
                ("CreateRenderProduct.outputs:renderProductPath", "RgbPublisher.inputs:renderProductPath"),
                ("CreateRenderProduct.outputs:renderProductPath", "DepthPublisher.inputs:renderProductPath"),
                ("CreateRenderProduct.outputs:renderProductPath", "PointCloudPublisher.inputs:renderProductPath"),
                ("CreateRenderProduct.outputs:renderProductPath", "CameraInfoPublisher.inputs:renderProductPath"),
            ],
        },
    )


def add_blocks(world):
    for object_name, object_pos in OBJECT_POSITIONS.items():
        world.scene.add(
            DynamicCuboid(
                prim_path=f"/World/{object_name}",
                name=object_name,
                position=object_pos,
                scale=BLOCK_SIZE,
                color=OBJECT_COLORS[object_name],
            )
        )


def main():
    enable_extension("isaacsim.ros2.bridge")
    simulation_app.update()

    scene_manager = SceneManager.from_defaults()
    command = parse_user_command_with_memory(args.command, scene_manager.memory)
    scene_objects = scene_manager.as_command_scene()
    validate_command(command, scene_objects)
    target_position = scene_manager.resolve_target_position(command)
    grasp_policy = FixedGraspPolicy()

    print("[INFO] Pick-place command:", command)
    print("[INFO] Target position:", target_position)

    world = World(stage_units_in_meters=1.0, physics_dt=1 / 200, rendering_dt=20 / 200)
    world.add_task(PickPlace(name=TASK_NAME))
    add_blocks(world)
    create_camera()

    world.reset()
    create_ros_camera_graph()
    omni.timeline.get_timeline_interface().play()

    task_params = world.get_task(TASK_NAME).get_params()
    robot_name = task_params["robot_name"]["value"]
    robot = world.scene.get_object(robot_name)
    controller = PickPlaceController(
        name="pick_place_controller",
        robot_articulation=robot,
        gripper=robot.gripper,
    )
    articulation_controller = robot.get_articulation_controller()

    print("[INFO] Publishing camera topics:")
    print("  /sim_camera/rgb")
    print("  /sim_camera/depth")
    print("  /sim_camera/camera_info")
    print("  /sim_camera/points")
    print("[INFO] Open RViz with: /home/minwoo/Desktop/LLM/ur10e/scripts/run_sim_camera_rviz.sh")

    reset_needed = False
    done_printed = False

    while simulation_app.is_running():
        world.step(render=True)

        if world.is_playing():
            if reset_needed:
                world.reset()
                controller.reset()
                reset_needed = False
                done_printed = False

            if world.current_time_step_index == 0:
                controller.reset()
                done_printed = False

            pick_object = world.scene.get_object(command["pick_object"])
            pick_position, _ = pick_object.get_world_pose()
            scene_manager.update_object(command["pick_object"], pick_position)
            grasp_action = grasp_policy.predict(
                GraspRequest(
                    object_name=command["pick_object"],
                    object_pose=pick_position,
                    target_position=target_position,
                    object_size=scene_manager.get_object(command["pick_object"]).size,
                    object_type=command["pick_object"],
                    robot_state={"joint_positions": robot.get_joint_positions()},
                )
            )

            actions = controller.forward(
                picking_position=pick_position,
                placing_position=target_position,
                current_joint_positions=robot.get_joint_positions(),
                end_effector_offset=grasp_action.end_effector_offset,
            )

            if controller.is_done():
                if not done_printed:
                    scene_manager.apply_pick_place_result(command, target_position)
                    print("[DONE] Pick-place completed")
                    done_printed = True
                    if not args.keep_running:
                        break
                continue

            articulation_controller.apply_action(actions)

        if world.is_stopped():
            reset_needed = True

    simulation_app.close()


if __name__ == "__main__":
    main()
