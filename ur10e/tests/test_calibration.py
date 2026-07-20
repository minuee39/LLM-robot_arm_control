import numpy as np
import pytest

from vision.calibration import optical_camera_to_world_transform, validate_rigid_transform
from vision.depth_utils import transform_point


def test_optical_camera_center_ray_points_at_target():
    eye = np.array([0.0, 0.0, 1.0])
    target = np.array([0.0, 1.0, 1.0])
    transform = optical_camera_to_world_transform(eye, target)

    world_point = transform_point(np.array([0.0, 0.0, 1.0]), transform)

    assert np.allclose(world_point, target)


def test_optical_camera_right_axis_matches_view_right():
    transform = optical_camera_to_world_transform(
        eye=[0.0, 0.0, 1.0],
        target=[0.0, 1.0, 1.0],
    )

    world_point = transform_point(np.array([1.0, 0.0, 0.0]), transform)

    assert np.allclose(world_point, np.array([1.0, 0.0, 1.0]))


def test_optical_camera_transform_rejects_parallel_up_vector():
    with pytest.raises(ValueError, match="parallel"):
        optical_camera_to_world_transform(
            eye=[0.0, 0.0, 0.0],
            target=[0.0, 0.0, 1.0],
            up=[0.0, 0.0, 1.0],
        )


def test_validate_rigid_transform_rejects_scaled_rotation():
    transform = np.eye(4)
    transform[0, 0] = 2.0

    with pytest.raises(ValueError, match="orthonormal"):
        validate_rigid_transform(transform)
