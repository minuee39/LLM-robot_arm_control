from isaacsim import SimulationApp

simulation_app = SimulationApp(
    {
        "headless": False,
        "renderer": "RayTracedLighting",
    }
)

import numpy as np
import omni.graph.core as og
import omni.usd
import usdrt.Sdf
from isaacsim.core.api import World
from isaacsim.core.api.objects import DynamicCuboid
from isaacsim.core.utils.extensions import enable_extension
from isaacsim.core.utils.viewports import set_camera_view
from pxr import Gf, UsdGeom


CAMERA_PRIM_PATH = "/World/RGBD_Camera"
FRAME_ID = "sim_camera"
WIDTH = 640
HEIGHT = 480


def create_block(world, name, position, color):
    block = DynamicCuboid(
        prim_path=f"/World/{name}",
        name=name,
        position=np.array(position, dtype=np.float32),
        scale=np.array([0.05, 0.05, 0.05], dtype=np.float32),
        color=np.array(color, dtype=np.float32),
    )
    world.scene.add(block)
    return block


def create_camera():
    stage = omni.usd.get_context().get_stage()
    camera_prim = UsdGeom.Camera.Define(stage, CAMERA_PRIM_PATH)
    camera_prim.GetFocalLengthAttr().Set(24.0)
    camera_prim.GetClippingRangeAttr().Set(Gf.Vec2f(0.01, 10.0))

    set_camera_view(
        eye=[0.0, 0.85, 1.0],
        target=[0.0, 0.35, 0.0],
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


def main():
    enable_extension("isaacsim.ros2.bridge")
    simulation_app.update()

    world = World(stage_units_in_meters=1.0)
    world.scene.add_default_ground_plane()

    create_block(world, "red_block", [-0.30, 0.30, 0.025], [1.0, 0.0, 0.0])
    create_block(world, "blue_block", [0.30, 0.30, 0.025], [0.0, 0.0, 1.0])
    create_block(world, "green_block", [0.00, 0.45, 0.025], [0.0, 1.0, 0.0])
    create_camera()

    world.reset()
    create_ros_camera_graph()

    print("[INFO] Publishing Isaac Sim camera topics:")
    print("  /sim_camera/rgb")
    print("  /sim_camera/depth")
    print("  /sim_camera/camera_info")
    print("  /sim_camera/points")
    print("[INFO] Keep this process running while the YOLO node is running.")

    while simulation_app.is_running():
        world.step(render=True)

    simulation_app.close()


if __name__ == "__main__":
    main()
