import numpy as np


def validate_rigid_transform(transform_matrix):
    transform = np.asarray(transform_matrix, dtype=float)
    if transform.shape != (4, 4):
        raise ValueError("transform must have shape (4, 4)")
    if not np.all(np.isfinite(transform)):
        raise ValueError("transform must contain only finite values")
    if not np.allclose(transform[3], np.array([0.0, 0.0, 0.0, 1.0]), atol=1e-6):
        raise ValueError("transform must have a homogeneous final row")

    rotation = transform[:3, :3]
    if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-5):
        raise ValueError("transform rotation must be orthonormal")
    if not np.isclose(np.linalg.det(rotation), 1.0, atol=1e-5):
        raise ValueError("transform rotation determinant must be 1")
    return transform


def optical_camera_to_world_transform(eye, target, up=(0.0, 0.0, 1.0)):
    """Build a transform from ROS optical coordinates to world coordinates.

    Optical coordinates use +X right, +Y down, and +Z forward. The returned
    matrix maps homogeneous camera points into the world frame.
    """

    eye = np.asarray(eye, dtype=float)
    target = np.asarray(target, dtype=float)
    up = np.asarray(up, dtype=float)
    if eye.shape != (3,) or target.shape != (3,) or up.shape != (3,):
        raise ValueError("eye, target, and up must be three-dimensional vectors")

    forward = target - eye
    forward_norm = np.linalg.norm(forward)
    up_norm = np.linalg.norm(up)
    if forward_norm <= 1e-9:
        raise ValueError("eye and target must be different")
    if up_norm <= 1e-9:
        raise ValueError("up vector must be non-zero")
    forward /= forward_norm
    up /= up_norm

    right = np.cross(forward, up)
    right_norm = np.linalg.norm(right)
    if right_norm <= 1e-9:
        raise ValueError("view direction and up vector must not be parallel")
    right /= right_norm
    down = np.cross(forward, right)
    down /= np.linalg.norm(down)

    transform = np.eye(4, dtype=float)
    transform[:3, 0] = right
    transform[:3, 1] = down
    transform[:3, 2] = forward
    transform[:3, 3] = eye
    return validate_rigid_transform(transform)
