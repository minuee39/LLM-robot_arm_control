import argparse
import math
import random
from pathlib import Path

from isaacsim import SimulationApp


parser = argparse.ArgumentParser()
parser.add_argument("--frames", type=int, default=20, help="Number of synthetic frames to write")
parser.add_argument(
    "--output-dir",
    default=str(Path(__file__).resolve().parents[1] / "datasets" / "block_synthetic" / "raw_replicator"),
    help="Replicator BasicWriter output directory",
)
parser.add_argument("--headless", action="store_true", help="Run Isaac Sim without opening the viewport")
parser.add_argument("--width", type=int, default=640)
parser.add_argument("--height", type=int, default=480)
parser.add_argument("--seed", type=int, default=7)
args, _ = parser.parse_known_args()

simulation_app = SimulationApp(
    {
        "headless": args.headless,
        "renderer": "RayTracedLighting",
    }
)

import numpy as np
import omni.replicator.core as rep
import omni.usd
from isaacsim.core.api import World
from isaacsim.core.api.objects import DynamicCuboid
from isaacsim.core.utils.viewports import set_camera_view
from pxr import Gf, UsdGeom

from scene_config import BLOCK_SIZE, OBJECT_COLORS


CAMERA_PRIM_PATH = "/World/RGBD_Camera"
BLOCK_NAMES = ("red_block", "blue_block", "green_block")
TABLE_X_LIMITS = (-0.42, 0.42)
TABLE_Y_LIMITS = (0.22, 0.58)
MIN_BLOCK_DISTANCE = 0.16


def add_semantics(prim_path, label):
    prim = omni.usd.get_context().get_stage().GetPrimAtPath(prim_path)
    if not prim.IsValid():
        raise RuntimeError(f"Cannot add semantics to missing prim: {prim_path}")

    try:
        from isaacsim.core.utils.semantics import add_update_semantics
    except Exception:
        from omni.isaac.core.utils.semantics import add_update_semantics

    add_update_semantics(prim, label, "class")


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
    return CAMERA_PRIM_PATH


def create_blocks(world):
    blocks = {}
    for name in BLOCK_NAMES:
        block = world.scene.add(
            DynamicCuboid(
                prim_path=f"/World/{name}",
                name=name,
                position=np.array([0.0, 0.0, BLOCK_SIZE[2] / 2.0], dtype=np.float32),
                scale=BLOCK_SIZE,
                color=OBJECT_COLORS[name],
            )
        )
        add_semantics(f"/World/{name}", name)
        blocks[name] = block
    return blocks


def random_positions():
    positions = []
    for _ in BLOCK_NAMES:
        for _attempt in range(200):
            candidate = np.array(
                [
                    random.uniform(*TABLE_X_LIMITS),
                    random.uniform(*TABLE_Y_LIMITS),
                    float(BLOCK_SIZE[2] / 2.0),
                ],
                dtype=np.float32,
            )
            if all(np.linalg.norm(candidate[:2] - pos[:2]) >= MIN_BLOCK_DISTANCE for pos in positions):
                positions.append(candidate)
                break
        else:
            raise RuntimeError("Could not sample non-overlapping block positions")
    return positions


def randomize_blocks(blocks):
    for name, position in zip(BLOCK_NAMES, random_positions()):
        block = blocks[name]
        yaw = random.uniform(-math.pi, math.pi)
        color_noise = np.array([random.uniform(-0.08, 0.08) for _ in range(3)], dtype=np.float32)
        color = np.clip(OBJECT_COLORS[name] + color_noise, 0.0, 1.0)

        block.set_world_pose(position=position, orientation=euler_to_quat(0.0, 0.0, yaw))
        material = block.get_applied_visual_material()
        if material is not None and hasattr(material, "set_color"):
            material.set_color(color)


def euler_to_quat(roll, pitch, yaw):
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    return np.array(
        [
            cr * cp * cy + sr * sp * sy,
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
        ],
        dtype=np.float32,
    )


def create_randomizers():
    light = rep.create.light(light_type="Distant", intensity=1800, rotation=(315, 0, 0))

    with rep.trigger.on_frame():
        with light:
            rep.modify.pose(rotation=rep.distribution.uniform((285, -20, -10), (345, 20, 10)))
            rep.modify.attribute("intensity", rep.distribution.uniform(900, 2800))


def main():
    random.seed(args.seed)
    np.random.seed(args.seed)

    output_dir = Path(args.output_dir).expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)

    world = World(stage_units_in_meters=1.0)
    world.scene.add_default_ground_plane()
    blocks = create_blocks(world)
    camera_path = create_camera()

    render_product = rep.create.render_product(camera_path, resolution=(args.width, args.height))
    writer = rep.WriterRegistry.get("BasicWriter")
    writer.initialize(
        output_dir=str(output_dir),
        rgb=True,
        bounding_box_2d_tight=True,
        semantic_segmentation=True,
    )
    create_randomizers()

    world.reset()
    writer.attach([render_product])
    print(f"[INFO] Writing {args.frames} synthetic frames to {output_dir}")

    for frame_index in range(args.frames):
        randomize_blocks(blocks)
        world.step(render=False)
        rep.orchestrator.step()
        print(f"[INFO] Wrote frame {frame_index + 1}/{args.frames}")

    rep.orchestrator.wait_until_complete()
    simulation_app.close()
    print("[DONE] Replicator dataset generation complete")


if __name__ == "__main__":
    main()
