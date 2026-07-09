from dataclasses import dataclass
from math import sqrt
from typing import Tuple

from geometry_msgs.msg import Pose


Vector3 = Tuple[float, float, float]
Quaternion = Tuple[float, float, float, float]


@dataclass(frozen=True)
class Transform:
    translation: Vector3
    rotation: Quaternion


TOOL0_TO_ROBOTIQ_GRASPING_FRAME = Transform(
    translation=(0.0, 0.0, 0.172),
    rotation=(-0.5, 0.5, -0.5, -0.5),
)


def normalize_quaternion(q: Quaternion) -> Quaternion:
    x, y, z, w = q
    norm = sqrt(x * x + y * y + z * z + w * w)
    if norm == 0.0:
        raise ValueError("Quaternion norm must be non-zero")
    return (x / norm, y / norm, z / norm, w / norm)


def multiply_quaternions(left: Quaternion, right: Quaternion) -> Quaternion:
    return normalize_quaternion(multiply_quaternions_raw(left, right))


def multiply_quaternions_raw(left: Quaternion, right: Quaternion) -> Quaternion:
    lx, ly, lz, lw = left
    rx, ry, rz, rw = right
    return (
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
        lw * rw - lx * rx - ly * ry - lz * rz,
    )


def conjugate_quaternion(q: Quaternion) -> Quaternion:
    x, y, z, w = q
    return (-x, -y, -z, w)


def rotate_vector(q: Quaternion, vector: Vector3) -> Vector3:
    q = normalize_quaternion(q)
    vector_quat = (vector[0], vector[1], vector[2], 0.0)
    rotated = multiply_quaternions_raw(
        multiply_quaternions_raw(q, vector_quat),
        conjugate_quaternion(q),
    )
    return (rotated[0], rotated[1], rotated[2])


def invert_transform(transform: Transform) -> Transform:
    inverse_rotation = conjugate_quaternion(normalize_quaternion(transform.rotation))
    inverse_translation = rotate_vector(
        inverse_rotation,
        (
            -transform.translation[0],
            -transform.translation[1],
            -transform.translation[2],
        ),
    )
    return Transform(translation=inverse_translation, rotation=inverse_rotation)


def compose_transforms(left: Transform, right: Transform) -> Transform:
    rotated_translation = rotate_vector(left.rotation, right.translation)
    translation = (
        left.translation[0] + rotated_translation[0],
        left.translation[1] + rotated_translation[1],
        left.translation[2] + rotated_translation[2],
    )
    rotation = multiply_quaternions(left.rotation, right.rotation)
    return Transform(translation=translation, rotation=rotation)


def pose_to_transform(pose: Pose) -> Transform:
    return Transform(
        translation=(pose.position.x, pose.position.y, pose.position.z),
        rotation=(
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        ),
    )


def transform_to_pose(transform: Transform) -> Pose:
    pose = Pose()
    pose.position.x = transform.translation[0]
    pose.position.y = transform.translation[1]
    pose.position.z = transform.translation[2]
    pose.orientation.x = transform.rotation[0]
    pose.orientation.y = transform.rotation[1]
    pose.orientation.z = transform.rotation[2]
    pose.orientation.w = transform.rotation[3]
    return pose


def tcp_target_to_tool0_target(
    tcp_target_pose: Pose,
    tool0_to_tcp: Transform = TOOL0_TO_ROBOTIQ_GRASPING_FRAME,
) -> Pose:
    base_to_tcp_target = pose_to_transform(tcp_target_pose)
    tcp_to_tool0 = invert_transform(tool0_to_tcp)
    base_to_tool0_target = compose_transforms(base_to_tcp_target, tcp_to_tool0)
    return transform_to_pose(base_to_tool0_target)
