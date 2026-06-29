import numpy as np
from vision.depth_utils import pixel_to_camera_point, transform_point


def test_pixel_to_camera_point_center():
    point = pixel_to_camera_point(
        u=640,
        v=360,
        depth=1.0,
        fx=600,
        fy=600,
        cx=640,
        cy=360,
    )

    assert np.allclose(point, np.array([0.0, 0.0, 1.0]))


def test_transform_point_identity():
    point = np.array([0.1, 0.2, 0.3])
    T = np.eye(4)

    result = transform_point(point, T)

    assert np.allclose(result, point)