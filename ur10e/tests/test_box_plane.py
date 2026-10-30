import numpy as np
import pytest

from vision.box_plane import fit_top_plane_candidate
from vision.box_pose_samples import mask_depth_samples


def cloud(normal=(0., 0., 1.), corrupt=False):
    normal = np.asarray(normal)
    v, u = np.mgrid[:21, :21]
    rays = np.stack(((u-10)/500, (v-10)/500, np.ones_like(u)), axis=-1)
    depth = 1/(rays @ normal)
    if corrupt:
        depth[5:8, 5:8] += .02
    return mask_depth_samples(
        depth, np.ones_like(depth, dtype=bool), encoding="32FC1",
        intrinsics=[500, 0, 10, 0, 500, 10, 0, 0, 1],
        optical_to_base_footprint=np.eye(4), rgb_stamp=1., depth_stamp=1., now=1.,
        max_age_s=.1, max_sync_delta_s=0., stamp_clock="synthetic",
        calibration_revision="test", mask_source="hsv_color_mask",
        registered_rectified=True, depth_semantics="optical_z")


def fit(samples, **overrides):
    limits = dict(distance_limit_m=.0001, max_tilt_rad=.3, min_inliers=100,
                  min_inlier_fraction=.8, min_minor_span_m=.02)
    limits.update(overrides)
    return fit_top_plane_candidate(samples, **limits)


def test_actual_tilt_not_assumed_identity_and_outlier_support():
    normal = np.array([np.sin(.15), 0., np.cos(.15)])
    result = fit(cloud(normal, corrupt=True))
    np.testing.assert_allclose(result.normal_base, normal, atol=1e-10)
    assert result.tilt_rad == pytest.approx(.15)
    assert result.residual_rms_m < 1e-12
    assert result.inlier_fraction == pytest.approx(352/361)
    assert result.minor_span_m > .03
    assert not hasattr(result, "yaw")


def test_tilted_or_side_plane_is_not_upright():
    with pytest.raises(ValueError, match="no supported"):
        fit(cloud((np.sin(.5), 0, np.cos(.5))))


@pytest.mark.parametrize("overrides", [
    {"distance_limit_m": 0}, {"distance_limit_m": float("nan")},
    {"max_tilt_rad": np.pi/2}, {"min_inlier_fraction": 0},
    {"min_minor_span_m": 0}, {"min_inliers": True}, {"trials": 10001},
])
def test_invalid_limits(overrides):
    with pytest.raises(ValueError):
        fit(cloud(), **overrides)


def test_insufficient_support_is_rejected():
    with pytest.raises(ValueError, match="insufficient"):
        fit(cloud(), min_inliers=400)
    with pytest.raises(ValueError, match="too narrow"):
        fit(cloud(), min_minor_span_m=.1)
