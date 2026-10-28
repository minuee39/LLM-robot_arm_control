import numpy as np
import pytest

from vision.pixel_plane_uncertainty import pixel_plane_error_bounds


def test_frontal_half_pixel_metric_error_is_not_pose_bound():
    bound = pixel_plane_error_bounds([[0, 0], [10, 20]], [100, 0, 0, 0, 100, 0, 0, 0, 1],
                                    np.eye(4), [0, 0, 1], [0, 0, 1], max_pixel_error_px=.5)
    np.testing.assert_allclose(bound, np.sqrt(2)*.005)


def test_oblique_corner_bound_contains_dense_pixel_grid():
    normal = np.array([.3, 0, np.sqrt(1-.3**2)])
    k = [100, 0, 0, 0, 100, 0, 0, 0, 1]
    bound = pixel_plane_error_bounds([[2, 3]], k, np.eye(4), [0, 0, 1], normal, max_pixel_error_px=.5)[0]
    grid = np.linspace(-.5, .5, 21)
    u, v = np.meshgrid(grid+2, grid+3)
    rays = np.stack((u/100, v/100, np.ones_like(u)), axis=-1)
    points = rays*(normal[2]/(rays@normal))[:, :, None]
    center_ray = np.array([.02, .03, 1.])
    center = center_ray*normal[2]/(center_ray@normal)
    assert np.linalg.norm(points-center, axis=2).max() <= bound+1e-12


def test_horizon_and_behind_camera_rejected():
    with pytest.raises(ValueError, match="horizon"):
        pixel_plane_error_bounds([[0, 0]], np.eye(3), np.eye(4), [1, 0, 0], [1, 0, 0], max_pixel_error_px=.5)
    with pytest.raises(ValueError, match="behind"):
        pixel_plane_error_bounds([[0, 0]], np.eye(3), np.eye(4), [0, 0, -1], [0, 0, 1], max_pixel_error_px=.5)


@pytest.mark.parametrize("error", [0, float("nan"), True])
def test_invalid_pixel_error(error):
    with pytest.raises(ValueError):
        pixel_plane_error_bounds([[0, 0]], np.eye(3), np.eye(4), [0, 0, 1], [0, 0, 1], max_pixel_error_px=error)
