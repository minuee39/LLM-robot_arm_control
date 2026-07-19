import argparse
import json
import os
import sys
import rclpy

import numpy as np
from sensor_msgs.msg import JointState
from isaacsim import SimulationApp


parser = argparse.ArgumentParser()
parser.add_argument("--headless", action="store_true", help="Run Isaac Sim without a visible window")
parser.add_argument(
    "--smoke-test-seconds",
    type=float,
    default=0.0,
    help="Exit after this many seconds once the bridge graph has started. Zero runs forever.",
)
parser.add_argument(
    "--scene-state-file",
    default="/tmp/ur10e_isaac_scene_objects.json",
    help="JSON file used by the ROS 2 scene collision publisher.",
)
parser.add_argument(
    "--scene-state-period",
    type=float,
    default=0.2,
    help="Seconds between scene object pose exports.",
)
args, _ = parser.parse_known_args()

simulation_app = SimulationApp(
    {
        "headless": args.headless,
        "renderer": "RayTracedLighting",
    }
)

CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.dirname(CURRENT_DIR)
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

import omni.graph.core as og
import omni.timeline
import omni.usd
import time
from isaacsim.core.api import World
from isaacsim.core.api.objects import DynamicCuboid
from isaacsim.core.utils.extensions import enable_extension
from pxr import UsdPhysics
from scene_config import (
    BLOCK_SIZE,
    OBJECT_COLORS,
    OBJECT_POSITIONS,
    UR10E_INITIAL_JOINT_POSITIONS,
)
from tasks.pick_place import PickPlace


TASK_NAME = "ur10e_moveit_bridge"


def find_articulation_root_path() -> str:
    stage = omni.usd.get_context().get_stage()
    articulation_roots = [
        str(prim.GetPath())
        for prim in stage.Traverse()
        if prim.HasAPI(UsdPhysics.ArticulationRootAPI)
    ]
    if not articulation_roots:
        raise RuntimeError("No USD prim with UsdPhysics.ArticulationRootAPI was found")

    print("[INFO] Isaac articulation roots:", ", ".join(articulation_roots), flush=True)
    return articulation_roots[0]


def create_moveit_bridge_graph(robot_prim_path: str) -> None:
    graph_path = "/MoveItBridgeGraph"
    og.Controller.edit(
        {"graph_path": graph_path, "evaluator_name": "execution"},
        {
            og.Controller.Keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("SubscribeJointCommand", "isaacsim.ros2.bridge.ROS2SubscribeJointState"),
                ("ArticulationController", "isaacsim.core.nodes.IsaacArticulationController"),
            ],
            og.Controller.Keys.SET_VALUES: [
                ("SubscribeJointCommand.inputs:topicName", "/isaac_joint_commands"),
                ("ArticulationController.inputs:robotPath", robot_prim_path),
            ],
            og.Controller.Keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", "SubscribeJointCommand.inputs:execIn"),
                ("SubscribeJointCommand.outputs:execOut", "ArticulationController.inputs:execIn"),
                ("SubscribeJointCommand.outputs:jointNames", "ArticulationController.inputs:jointNames"),
                ("SubscribeJointCommand.outputs:positionCommand", "ArticulationController.inputs:positionCommand"),
                ("SubscribeJointCommand.outputs:velocityCommand", "ArticulationController.inputs:velocityCommand"),
                ("SubscribeJointCommand.outputs:effortCommand", "ArticulationController.inputs:effortCommand"),
            ],
        },
    )

def get_robot_joint_names(robot):
    if hasattr(robot, "dof_names"):
          return list(robot.dof_names)
    if hasattr(robot, "get_dof_names"):
          return list(robot.get_dof_names())
    if hasattr(robot, "joint_names"):
          return list(robot.joint_names)
    raise RuntimeError("Could not read joint names from Isaac articulation")


def set_named_joint_positions(robot, target_positions: dict[str, float]) -> None:
    joint_names = get_robot_joint_names(robot)
    joint_indices = []
    positions = []

    for joint_name, position in target_positions.items():
        if joint_name not in joint_names:
            print(f"[WARN] Initial joint '{joint_name}' was not found in Isaac joints", flush=True)
            continue

        joint_indices.append(joint_names.index(joint_name))
        positions.append(float(position))

    if not positions:
        raise RuntimeError("None of the configured initial joints exist on the Isaac robot")

    robot.set_joint_positions(np.array(positions, dtype=float), joint_indices=np.array(joint_indices, dtype=np.int32))
    print("[INFO] Applied initial joints:", target_positions, flush=True)


def add_blocks(world: World) -> None:
    for object_name, object_position in OBJECT_POSITIONS.items():
        world.scene.add(
            DynamicCuboid(
                prim_path=f"/World/{object_name}",
                name=object_name,
                position=object_position,
                scale=BLOCK_SIZE,
                color=OBJECT_COLORS[object_name],
            )
        )


def write_scene_state(world: World, path: str) -> None:
    object_specs = {
        object_name: (BLOCK_SIZE, OBJECT_COLORS[object_name])
        for object_name in OBJECT_POSITIONS
    }
    objects = []
    for object_name, (object_size, object_color) in object_specs.items():
        scene_object = world.scene.get_object(object_name)
        position, _ = scene_object.get_world_pose()
        objects.append(
            {
                "name": object_name,
                "position": [float(value) for value in position],
                "size": [float(value) for value in object_size],
                "color": [float(value) for value in object_color] + [1.0],
            }
        )

    tmp_path = f"{path}.tmp"
    with open(tmp_path, "w", encoding="utf-8") as file:
        json.dump({"frame": "world", "objects": objects}, file)
    os.replace(tmp_path, path)


def main() -> None:
    enable_extension("isaacsim.ros2.bridge")
    simulation_app.update()

    world = World(stage_units_in_meters=1.0, physics_dt=1 / 200, rendering_dt=1 / 60)
    world.add_task(PickPlace(name=TASK_NAME))
    add_blocks(world)
    world.reset()

    task_params = world.get_task(TASK_NAME).get_params()
    robot_name = task_params["robot_name"]["value"]
    robot = world.scene.get_object(robot_name)
    set_named_joint_positions(robot, UR10E_INITIAL_JOINT_POSITIONS)
    
    rclpy.init(args=None)
    state_node =rclpy.create_node("isaac_joint_state_feedback")
    state_pub = state_node.create_publisher(JointState,"/isaac_joint_states", 10)
    joint_names = get_robot_joint_names(robot)
    print("[INFO] Isaac joint names:", joint_names,flush=True)
    print("[INFO] Publishing: /isaac_joint_states",flush=True)
  
    robot_prim_path = find_articulation_root_path()
    create_moveit_bridge_graph(robot_prim_path)
    omni.timeline.get_timeline_interface().play()
    

    print("[INFO] Isaac MoveIt bridge graph is running", flush=True)
    print(f"[INFO] Robot prim:   {robot_prim_path}", flush=True)
    print("[INFO] Blocks:", ", ".join(OBJECT_POSITIONS.keys()), flush=True)
    print("[INFO] Scene state file:", args.scene_state_file, flush=True)
    

    started_at = time.monotonic()
    last_scene_state_write = 0.0
    while simulation_app.is_running():
        world.step(render=not args.headless)
        rclpy.spin_once(state_node, timeout_sec=0.0)
        
        msg = JointState()
        msg.header.stamp = state_node.get_clock().now().to_msg()
        msg.name = joint_names
        msg.position = [float(v) for v in robot.get_joint_positions()]
        msg.velocity = [float(v) for v in robot.get_joint_velocities()]
        state_pub.publish(msg)

        now = time.monotonic()
        if now - last_scene_state_write >= args.scene_state_period:
            write_scene_state(world, args.scene_state_file)
            last_scene_state_write = now
        
        
        if args.smoke_test_seconds > 0.0 and now - started_at >= args.smoke_test_seconds:
            print("[INFO] Smoke test completed", flush=True)
            break
        
        
    state_node.destroy_node()
    rclpy.shutdown()
    simulation_app.close()


if __name__ == "__main__":
    main()
