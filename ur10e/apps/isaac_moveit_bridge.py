import argparse
import os
import sys

from isaacsim import SimulationApp


parser = argparse.ArgumentParser()
parser.add_argument("--headless", action="store_true", help="Run Isaac Sim without a visible window")
parser.add_argument(
    "--smoke-test-seconds",
    type=float,
    default=0.0,
    help="Exit after this many seconds once the bridge graph has started. Zero runs forever.",
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
from isaacsim.core.utils.extensions import enable_extension
from pxr import UsdPhysics
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


def main() -> None:
    enable_extension("isaacsim.ros2.bridge")
    simulation_app.update()

    world = World(stage_units_in_meters=1.0, physics_dt=1 / 200, rendering_dt=1 / 60)
    world.add_task(PickPlace(name=TASK_NAME))
    world.reset()

    robot_prim_path = find_articulation_root_path()
    create_moveit_bridge_graph(robot_prim_path)
    omni.timeline.get_timeline_interface().play()

    print("[INFO] Isaac MoveIt bridge graph is running", flush=True)
    print(f"[INFO] Robot prim:   {robot_prim_path}", flush=True)
    print("[INFO] Subscribing: /isaac_joint_commands", flush=True)

    started_at = time.monotonic()
    while simulation_app.is_running():
        world.step(render=not args.headless)
        if args.smoke_test_seconds > 0.0 and time.monotonic() - started_at >= args.smoke_test_seconds:
            print("[INFO] Smoke test completed", flush=True)
            break

    simulation_app.close()


if __name__ == "__main__":
    main()
