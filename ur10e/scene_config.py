import numpy as np


BLOCK_SIZE = np.array([0.1, 0.0515, 0.1])

OBJECT_POSITIONS = {
    "red_block": np.array([-0.45, 0.55, 0.02575]),
    "blue_block": np.array([0.45, 0.55, 0.02575]),
    "green_block": np.array([0.0, 0.75, 0.02575]),
}

OBJECT_COLORS = {
    "red_block": np.array([1.0, 0.0, 0.0]),
    "blue_block": np.array([0.0, 0.0, 1.0]),
    "green_block": np.array([0.0, 1.0, 0.0]),
}


def build_scene_objects() -> dict:
    return {
        object_name: {
            "position": object_position
        }
        for object_name, object_position in OBJECT_POSITIONS.items()
    }