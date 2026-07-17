import numpy as np


BLOCK_SIZE = np.array([0.1, 0.0515, 0.1])

OBJECT_POSITIONS = {
    "red_block": np.array([-0.30, 0.30, 0.02575]),
    "blue_block": np.array([0.30, 0.30, 0.02575]),
    "green_block": np.array([0.0, 0.45, 0.02575]),
}

OBJECT_COLORS = {
    "red_block": np.array([1.0, 0.0, 0.0]),
    "blue_block": np.array([0.0, 0.0, 1.0]),
    "green_block": np.array([0.0, 1.0, 0.0]),
}

MTC_OBJECT_NAME = "object"
MTC_OBJECT_POSITION = np.array([0.70, 0.40, 0.05])
MTC_OBJECT_SIZE = np.array([0.04, 0.04, 0.10])
MTC_OBJECT_COLOR = np.array([0.9, 0.8, 0.1])


def build_scene_objects() -> dict:
    return {
        object_name: {
            "position": object_position
        }
        for object_name, object_position in OBJECT_POSITIONS.items()
    }


UR10E_INITIAL_JOINT_POSITIONS = {
    "shoulder_pan_joint": 0.0,
    "shoulder_lift_joint": -1.57,
    "elbow_joint": 2,
    "wrist_1_joint": -1.57,
    "wrist_2_joint": 1.57,
    "wrist_3_joint": 3.14,
    "finger_joint": 0.01,
}
