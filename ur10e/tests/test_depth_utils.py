import numpy as np
from vision.depth_utils import (
    median_depth_in_bbox,
    pixel_to_camera_point,
    surface_point_to_box_center,
    transform_point,
)


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


def test_median_depth_in_bbox_ignores_invalid_samples():
    depth = np.full((10, 10), np.nan)
    depth[4:6, 4:6] = [[1.0, 0.0], [1.2, 1.1]]

    result = median_depth_in_bbox(depth, (2, 2, 8, 8), center_fraction=0.4)

    assert result == 1.1


def test_median_depth_in_bbox_returns_none_without_valid_depth():
    depth = np.zeros((10, 10), dtype=float)

    assert median_depth_in_bbox(depth, (2, 2, 8, 8)) is None


def test_transform_point_identity():
    point = np.array([0.1, 0.2, 0.3])
    T = np.eye(4)

    result = transform_point(point, T)

    assert np.allclose(result, point)


def test_surface_point_to_box_center_from_top_face():
    center = surface_point_to_box_center(
        surface_point=[0.0, 0.0, 0.10],
        camera_origin=[0.0, 0.0, 2.9],
        box_size=[0.1, 0.0515, 0.1],
    )

    assert np.allclose(center, [0.0, 0.0, 0.05])


def test_surface_point_to_box_center_follows_oblique_camera_ray():
    camera_origin = np.array([0.0, 0.95, 2.9])
    expected_center = np.array([0.0, 0.45, 0.05])
    direction = expected_center - camera_origin
    direction /= np.linalg.norm(direction)
    half_size = np.array([0.05, 0.02575, 0.05])
    surface_distance = np.min(half_size[np.abs(direction) > 1e-9] / np.abs(direction[np.abs(direction) > 1e-9]))
    surface_point = expected_center - direction * surface_distance

    center = surface_point_to_box_center(
        surface_point,
        camera_origin,
        box_size=[0.1, 0.0515, 0.1],
    )

    assert np.allclose(center, expected_center)
