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

SIM_CAMERA_EYE = np.array([0.0, 0.95, 2.9])
SIM_CAMERA_TARGET = np.array([0.0, 0.32, 0.08])
SIM_CAMERA_UP = np.array([0.0, 0.0, 1.0])

def build_scene_objects() -> dict:
    return {
        object_name: {
            "position": object_position
        }
        for object_name, object_position in OBJECT_POSITIONS.items()
    }


UR10E_INITIAL_JOINT_POSITIONS = {
    "shoulder_pan_joint": 0.56,
    "shoulder_lift_joint": -1.78,
    "elbow_joint": 1.41,
    "wrist_1_joint": 1.94,
    "wrist_2_joint": 1.57,
    "wrist_3_joint": -2.58,
    "finger_joint": 0.01,
}
