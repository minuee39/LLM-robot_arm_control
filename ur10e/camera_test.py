# camera_rgb_test.py
# Isaac Sim USD Camera -> RGB image extraction test
# OpenCV imshow 사용하지 않는 안전 버전

from omni.isaac.kit import SimulationApp

simulation_app = SimulationApp({
    "headless": False,
    "renderer": "RayTracedLighting",
})

import time
import numpy as np
import cv2

from omni.isaac.core import World
from omni.isaac.core.objects import DynamicCuboid

from pxr import UsdGeom, Gf
import omni.usd
import omni.kit.viewport.utility as vp_utils

import omni.replicator.core as rep


OUTPUT_PATH = "/home/minwoo/Desktop/LLM/ur10e/camera_rgb_output.png"


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


def create_usd_camera(camera_path="/World/RGBD_Camera"):
    stage = omni.usd.get_context().get_stage()

    camera_prim = UsdGeom.Camera.Define(stage, camera_path)

    # 이전 단계에서 물체가 잘 보였던 위치/회전 사용
    camera_prim.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.35, 1.2))
    camera_prim.AddRotateXYZOp().Set(Gf.Vec3f(0.0, 0.0, 0.0))

    camera_prim.GetFocalLengthAttr().Set(24.0)
    camera_prim.GetClippingRangeAttr().Set(Gf.Vec2f(0.01, 10.0))

    return camera_path


def set_viewport_camera(camera_path):
    viewport = vp_utils.get_active_viewport()
    if viewport is not None:
        viewport.camera_path = camera_path
        print(f"[INFO] Viewport camera set to: {camera_path}")
    else:
        print("[WARN] Active viewport not found.")


def main():
    world = World(stage_units_in_meters=1.0)
    world.scene.add_default_ground_plane()

    create_block(world, "red_block", [-0.30, 0.30, 0.025], [1.0, 0.0, 0.0])
    create_block(world, "blue_block", [0.30, 0.30, 0.025], [0.0, 0.0, 1.0])
    create_block(world, "green_block", [0.00, 0.45, 0.025], [0.0, 1.0, 0.0])

    camera_path = create_usd_camera("/World/RGBD_Camera")

    world.reset()
    set_viewport_camera(camera_path)

    render_product = rep.create.render_product(camera_path, resolution=(640, 480))

    rgb_annotator = rep.AnnotatorRegistry.get_annotator("rgb")
    rgb_annotator.attach([render_product])

    print("[INFO] RGB extraction test started.")
    print("[INFO] Will save one RGB image to:")
    print(OUTPUT_PATH)

    frame_count = 0
    saved = False

    try:
        while simulation_app.is_running():
            world.step(render=True)

            frame_count += 1
            if frame_count < 60:
                continue

            rgb_data = rgb_annotator.get_data()

            if rgb_data is None:
                print("[WARN] RGB data is None")
                continue

            rgb_array = np.array(rgb_data)

            print("[INFO] RGB raw shape:", rgb_array.shape)
            print("[INFO] RGB raw dtype:", rgb_array.dtype)

            if rgb_array.ndim != 3:
                print("[ERROR] Unexpected RGB shape:", rgb_array.shape)
                break

            if rgb_array.shape[2] == 4:
                rgb = rgb_array[:, :, :3]
            else:
                rgb = rgb_array[:, :, :3]

            rgb = rgb.astype(np.uint8)

            # 저장용으로 RGB -> BGR 변환
            bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)

            success = cv2.imwrite(OUTPUT_PATH, bgr)

            if success:
                print("[SUCCESS] RGB image saved.")
                print("[SUCCESS] Check this file:")
                print(OUTPUT_PATH)
            else:
                print("[ERROR] Failed to save image.")

            saved = True
            break

    finally:
        # cv2.destroyAllWindows() 사용하지 않음
        simulation_app.close()

    if saved:
        print("[DONE] RGB extraction test finished.")


if __name__ == "__main__":
    main()
