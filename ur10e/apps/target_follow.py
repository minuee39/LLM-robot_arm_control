import argparse
import math
import sys
import time
from pathlib import Path

from isaacsim import SimulationApp


PROJECT_DIR = Path(__file__).resolve().parents[1]
if str(PROJECT_DIR) not in sys.path:
    sys.path.insert(0, str(PROJECT_DIR))

parser = argparse.ArgumentParser(
    description="Isaac Sim scene for MoveIt2 target following. Isaac publishes target pose and executes MoveIt trajectories."
)
parser.add_argument("--headless", action="store_true")
parser.add_argument("--auto-move", action="store_true", help="Animate the target so MoveIt2 can follow without dragging.")
parser.add_argument("--smoke-test-seconds", type=float, default=0.0)
parser.add_argument("--target-x", type=float, default=0.45)
parser.add_argument("--target-y", type=float, default=0.0)
parser.add_argument("--target-z", type=float, default=0.45)
parser.add_argument("--target-size", type=float, default=0.06)
parser.add_argument("--physics-dt", type=float, default=1.0 / 200.0)
parser.add_argument("--rendering-dt", type=float, default=1.0 / 60.0)
args, _ = parser.parse_known_args()

simulation_app = SimulationApp(
    {
        "headless": args.headless,
        "renderer": "RayTracedLighting",
    }
)

import numpy as np
import omni.graph.core as og
import omni.timeline
import omni.usd
import usdrt.Sdf
from isaacsim.core.api import World
from isaacsim.core.utils.extensions import enable_extension
from isaacsim.core.utils.viewports import set_camera_view
from pxr import UsdPhysics
from tasks.follow_target import FollowTarget


TASK_NAME = "ur10e_moveit_target_follow"
TARGET_NAME = "target"


def find_articulation_root_path() -> str:
    stage = omni.usd.get_context().get_stage()
    articulation_roots = [
        str(prim.GetPath())
        for prim in stage.Traverse()
        if prim.HasAPI(UsdPhysics.ArticulationRootAPI)
    ]
    if not articulation_roots:
        raise RuntimeError("No USD prim with UsdPhysics.ArticulationRootAPI was found")
    return articulation_roots[0]


def create_moveit_bridge_graph(robot_prim_path: str, target_prim_path: str) -> None:
    og.Controller.edit(
        {"graph_path": "/MoveItBridgeGraph", "evaluator_name": "execution"},
        {
            og.Controller.Keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                ("SubscribeJointCommand", "isaacsim.ros2.bridge.ROS2SubscribeJointState"),
                ("ArticulationController", "isaacsim.core.nodes.IsaacArticulationController"),
                ("PublishJointState", "isaacsim.ros2.bridge.ROS2PublishJointState"),
                ("PublishTargetTf", "isaacsim.ros2.bridge.ROS2PublishTransformTree"),
            ],
            og.Controller.Keys.SET_VALUES: [
                ("SubscribeJointCommand.inputs:topicName", "/isaac_joint_commands"),
                ("ArticulationController.inputs:robotPath", robot_prim_path),
                ("PublishJointState.inputs:topicName", "/isaac_joint_states"),
                ("PublishJointState.inputs:targetPrim", [usdrt.Sdf.Path(robot_prim_path)]),
                ("PublishTargetTf.inputs:topicName", "/tf"),
                ("PublishTargetTf.inputs:targetPrims", [usdrt.Sdf.Path(target_prim_path)]),
            ],
            og.Controller.Keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", "SubscribeJointCommand.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "PublishJointState.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "PublishTargetTf.inputs:execIn"),
                ("ReadSimTime.outputs:simulationTime", "PublishJointState.inputs:timeStamp"),
                ("ReadSimTime.outputs:simulationTime", "PublishTargetTf.inputs:timeStamp"),
                ("SubscribeJointCommand.outputs:execOut", "ArticulationController.inputs:execIn"),
                ("SubscribeJointCommand.outputs:jointNames", "ArticulationController.inputs:jointNames"),
                ("SubscribeJointCommand.outputs:positionCommand", "ArticulationController.inputs:positionCommand"),
                ("SubscribeJointCommand.outputs:velocityCommand", "ArticulationController.inputs:velocityCommand"),
                ("SubscribeJointCommand.outputs:effortCommand", "ArticulationController.inputs:effortCommand"),
            ],
        },
    )


def get_robot_joint_names(robot) -> list[str]:
    if hasattr(robot, "dof_names"):
        return list(robot.dof_names)
    if hasattr(robot, "get_dof_names"):
        return list(robot.get_dof_names())
    if hasattr(robot, "joint_names"):
        return list(robot.joint_names)
    raise RuntimeError("Could not read joint names from Isaac articulation")


def get_scene_object_prim_path(scene_object, object_name: str) -> str:
    for attr_name in ("prim_path", "_prim_path"):
        prim_path = getattr(scene_object, attr_name, None)
        if prim_path:
            return str(prim_path)

    prim = getattr(scene_object, "prim", None)
    if prim is not None and hasattr(prim, "GetPath"):
        return str(prim.GetPath())

    stage = omni.usd.get_context().get_stage()
    candidates = [
        str(prim.GetPath())
        for prim in stage.Traverse()
        if prim.GetName() == object_name or prim.GetName().lower() == object_name.lower()
    ]
    if candidates:
        return candidates[0]

    raise RuntimeError(f"Could not find prim path for scene object: {object_name}")


def animate_target(target, step_index: int, base_position: np.ndarray) -> None:
    time_s = step_index * args.physics_dt
    position = base_position.copy()
    position[0] += 0.12 * math.sin(0.45 * time_s)
    position[1] += 0.12 * math.cos(0.35 * time_s)
    position[2] += 0.06 * math.sin(0.60 * time_s)
    target.set_world_pose(position=position)


def main() -> None:
    enable_extension("isaacsim.ros2.bridge")
    simulation_app.update()

    world = World(
        stage_units_in_meters=1.0,
        physics_dt=args.physics_dt,
        rendering_dt=args.rendering_dt,
    )
    world.add_task(
        FollowTarget(
            name=TASK_NAME,
            target_name=TARGET_NAME,
            target_position=np.array([args.target_x, args.target_y, args.target_z], dtype=float),
        )
    )
    set_camera_view(
        eye=[1.35, 1.35, 1.15],
        target=[0.25, 0.0, 0.35],
        camera_prim_path="/OmniverseKit_Persp",
    )
    world.reset()

    task_params = world.get_task(TASK_NAME).get_params()
    robot = world.scene.get_object(task_params["robot_name"]["value"])
    target = world.scene.get_object(task_params["target_name"]["value"])
    target.set_local_scale(np.array([args.target_size] * 3, dtype=float))
    target_base_position, _ = target.get_world_pose()

    robot_prim_path = find_articulation_root_path()
    target_prim_path = get_scene_object_prim_path(target, task_params["target_name"]["value"])
    create_moveit_bridge_graph(robot_prim_path, target_prim_path)
    omni.timeline.get_timeline_interface().play()

    print("[INFO] Isaac target-follow scene is running", flush=True)
    print(f"[INFO] Robot prim: {robot_prim_path}", flush=True)
    print(f"[INFO] Target prim: {target_prim_path}", flush=True)
    print("[INFO] Isaac executes only /isaac_joint_commands from MoveIt2", flush=True)
    print("[INFO] Publishing /isaac_joint_states and /tf with the target frame", flush=True)
    print("[INFO] Drag the target object, or run this script with --auto-move", flush=True)

    started_at = time.monotonic()
    while simulation_app.is_running():
        world.step(render=not args.headless)

        if args.auto_move and world.is_playing():
            animate_target(target, world.current_time_step_index, target_base_position)

        if args.smoke_test_seconds > 0.0 and time.monotonic() - started_at >= args.smoke_test_seconds:
            print("[INFO] Smoke test completed", flush=True)
            break

    simulation_app.close()


if __name__ == "__main__":
    main()
