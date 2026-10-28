import numpy as np
import pytest

from vision.box_boundary import SquareBoundaryRejected, fit_square_boundary_candidate


def square(angle=0., tilt=0., omit_edge=None):
    t = np.linspace(-.015, .015, 31)
    edges = [np.column_stack((np.full_like(t, -.015), t)),
             np.column_stack((np.full_like(t, .015), t)),
             np.column_stack((t, np.full_like(t, -.015))),
             np.column_stack((t, np.full_like(t, .015)))]
    xy = np.concatenate([e for i, e in enumerate(edges) if i != omit_edge])
    n = np.array([np.sin(tilt), 0, np.cos(tilt)])
    x = np.array([np.cos(tilt), 0, -np.sin(tilt)])
    basis = np.column_stack((x, [0, 1, 0]))
    axes = np.array([[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]])
    center = np.array([.24, -.28, .05])
    return xy @ axes.T @ basis.T + center, n, center


def fit(args, **overrides):
    limits = dict(expected_side_m=.03, max_size_error_m=.001,
                  max_plane_residual_m=.0001, edge_band_m=.0001,
                  min_edge_support=10, min_edge_span_fraction=.8,
                  max_hull_residual_m=.0001, image_edge_clipped=False)
    limits.update(overrides)
    return fit_square_boundary_candidate(*args, **limits)


@pytest.mark.parametrize("angle", [0., np.pi/4, np.radians(89), np.pi/2, np.radians(135)])
@pytest.mark.parametrize("tilt", [0., .15])
def test_measured_square_direction_and_center(angle, tilt):
    args = square(angle, tilt)
    result = fit(args)
    delta = (result.yaw_mod_quarter_turn_rad-angle+np.pi/4) % (np.pi/2)-np.pi/4
    assert abs(delta) < 1e-6
    np.testing.assert_allclose(result.top_center_base_m, args[2], atol=1e-8)
    np.testing.assert_allclose(result.rotation_base[:, 2], args[1], atol=1e-8)
    np.testing.assert_allclose(result.rotation_base.T @ result.rotation_base, np.eye(3), atol=1e-8)
    assert np.linalg.det(result.rotation_base) == pytest.approx(1.)
    np.testing.assert_allclose(result.observed_size_xy_m, [.03, .03], atol=1e-8)


@pytest.mark.parametrize("edge", range(4))
def test_missing_edge_not_completed_from_expected_size(edge):
    with pytest.raises(ValueError, match="edge support"):
        fit(square(.3, omit_edge=edge))


def test_partial_square_wrong_size_and_off_plane_rejected():
    points, n, center = square()
    with pytest.raises(ValueError, match="size"):
        fit((center + .5*(points-center), n, center))
    points[0, 2] += .001
    with pytest.raises(ValueError, match="plane residual"):
        fit((points, n, center))


def test_cut_corner_rejected_with_diagnostic_not_accepted_pose():
    points, normal, center = square()
    local = points-center
    kept = local[:, 0]+local[:, 1] <= .025
    with pytest.raises(SquareBoundaryRejected) as failure:
        fit((points[kept], normal, center))
    metrics = failure.value.diagnostics
    assert metrics["accepted"] is False
    assert metrics["hull_residual_max_m"] > metrics["hull_residual_limit_m"]
    assert "rotation_base" not in metrics


@pytest.mark.parametrize("override", [
    {"image_edge_clipped": True}, {"image_edge_clipped": None},
    {"expected_side_m": float("nan")}, {"edge_band_m": .02},
    {"min_edge_support": True}, {"min_edge_span_fraction": 0},
])
def test_unverified_or_invalid_limits(override):
    with pytest.raises(ValueError):
        fit(square(), **override)
