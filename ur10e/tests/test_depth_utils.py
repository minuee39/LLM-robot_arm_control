import numpy as np
from vision.depth_utils import (
    depth_value_to_millimeters,
    depth_value_to_meters,
    median_depth_in_bbox,
    median_depth_in_mask,
    pixel_to_camera_point,
    surface_point_to_box_center,
    transform_point,
    validated_world_position,
)


def test_depth_value_to_meters_converts_astra_millimetres():
    assert depth_value_to_meters(593, "16UC1") == 0.593


def test_depth_value_to_meters_keeps_float_depth_metres():
    assert depth_value_to_meters(0.593, "32FC1") == 0.593


def test_depth_value_to_meters_rejects_unknown_encoding():
    with np.testing.assert_raises_regex(ValueError, "unsupported depth encoding"):
        depth_value_to_meters(593, "8UC1")


def test_depth_value_to_millimeters_keeps_astra_depth():
    assert depth_value_to_millimeters(593, "16UC1") == 593.0


def test_depth_value_to_millimeters_converts_float_depth():
    assert depth_value_to_millimeters(0.593, "32FC1") == 593.0


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


def test_median_depth_in_mask_uses_only_segmented_shape():
    depth = np.full((4, 5), 900.0)
    mask = np.zeros_like(depth, dtype=bool)
    mask[1:3, 1:4] = True
    depth[mask] = [500.0, 0.0, 520.0, np.nan, 510.0, 530.0]

    assert median_depth_in_mask(depth, mask) == 515.0


def test_median_depth_in_mask_rejects_mismatched_shape():
    with np.testing.assert_raises_regex(ValueError, "same shape"):
        median_depth_in_mask(np.ones((4, 5)), np.ones((2, 2), dtype=bool))


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


def test_validated_world_position_uses_reference_above_grasp_tolerance():
    position, used_reference, error = validated_world_position(
        [-0.303, 0.289, 0.05], [-0.3, 0.3, 0.05], max_error=0.003
    )

    assert used_reference is True
    assert error > 0.01
    assert np.allclose(position, [-0.3, 0.3, 0.05])


def test_validated_world_position_keeps_measurement_without_simulator_reference():
    position, used_reference, error = validated_world_position(
        [0.1, 0.2, 0.05], None, max_error=0.003
    )

    assert used_reference is False
    assert error == 0.0
    assert np.allclose(position, [0.1, 0.2, 0.05])
