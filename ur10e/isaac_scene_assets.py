import numpy as np

from isaacsim.core.prims import SingleXFormPrim
from isaacsim.core.utils.semantics import add_labels
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.storage.native import get_assets_root_path


CUP_NAME = "cup"
CUP_PRIM_PATH = "/World/cup"
CUP_ASSET_RELATIVE_PATH = "/Isaac/Props/Mugs/SM_Mug_A2.usd"
CUP_POSITION = np.array([0.0, 0.60, 0.075], dtype=float)
CUP_SCALE = np.array([0.01, 0.01, 0.01], dtype=float)
CUP_APPROX_SIZE = np.array([0.12, 0.12, 0.15], dtype=float)


def add_cup(world):
    """Add a stationary, camera-visible mug asset to an Isaac Sim world."""

    assets_root_path = get_assets_root_path()
    if assets_root_path is None:
        raise RuntimeError(
            "Isaac Sim assets root is unavailable; configure the default asset root first"
        )

    usd_path = assets_root_path + CUP_ASSET_RELATIVE_PATH
    add_reference_to_stage(usd_path=usd_path, prim_path=CUP_PRIM_PATH)
    cup = world.scene.add(
        SingleXFormPrim(
            prim_path=CUP_PRIM_PATH,
            name=CUP_NAME,
            position=CUP_POSITION,
            scale=CUP_SCALE,
        )
    )
    add_labels(cup.prim, labels=[CUP_NAME], instance_name="class")
    return cup, usd_path
