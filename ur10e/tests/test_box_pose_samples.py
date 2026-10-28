import numpy as np
import pytest

from vision.box_pose_samples import mask_depth_samples


def sample(**overrides):
    mask = np.zeros((5, 5), dtype=bool)
    mask[1:4, 1:4] = True
    args = dict(depth_image=np.ones((5, 5)), mask=mask, encoding="32FC1",
                intrinsics=[100, 0, 2, 0, 100, 2, 0, 0, 1],
                optical_to_base_footprint=np.eye(4), rgb_stamp=10., depth_stamp=10.,
                now=10.1, max_age_s=.2, max_sync_delta_s=.01, stamp_clock="unix",
                calibration_revision="capture-1", mask_source="hsv_color_mask",
                registered_rectified=True, depth_semantics="optical_z")
    args.update(overrides)
    return mask_depth_samples(**args)


def test_individual_depth_not_median_and_valid_fraction():
    depth = np.ones((5, 5))
    depth[1, 1], depth[2, 2], depth[3, 3] = 2., 0., np.nan
    result = sample(depth_image=depth)
    np.testing.assert_allclose(result.points_base_m[0], [-.02, -.02, 2.])
    assert result.mask_pixel_count == 9
    assert result.valid_depth_fraction == 7/9
    assert result.boundary_uv.shape == (8, 2)
    assert result.interior.sum() == 0
    assert not result.image_edge_clipped
    assert result.frame == "base_footprint" and result.unit == "m"
    assert not hasattr(result, "orientation")


@pytest.mark.parametrize("encoding", ["16UC1", "MONO16"])
def test_millimetres_and_nonidentity_transform(encoding):
    transform = np.array([[0, -1, 0, .1], [1, 0, 0, .2], [0, 0, 1, .3], [0, 0, 0, 1]])
    result = sample(depth_image=np.full((5, 5), 1000, dtype=np.uint16),
                    encoding=encoding, optical_to_base_footprint=transform)
    np.testing.assert_allclose(result.points_base_m[0], [.11, .19, 1.3])
    assert result.interior.sum() == 1
    with pytest.raises(ValueError):
        result.points_base_m[0] = 0


def test_boundary_retained_without_depth_and_clipping_is_not_hidden():
    result = sample(mask=np.ones((5, 5), dtype=bool))
    assert result.image_edge_clipped
    assert result.boundary_uv.shape == (16, 2)


def test_verified_half_pixel_center_is_opt_in_and_default_unchanged():
    old = sample()
    centered = sample(pixel_center_offset_px=.5)
    np.testing.assert_allclose(centered.points_base_m-old.points_base_m,
                               np.tile([.005, .005, 0], (9, 1)))
    np.testing.assert_array_equal(old.pixel_uv, centered.pixel_uv)
    assert old.pixel_center_offset_px == 0.
    assert centered.pixel_center_offset_px == .5
    with pytest.raises(ValueError): sample(pixel_center_offset_px=.25)


@pytest.mark.parametrize("overrides", [
    {"rgb_stamp": 9.8}, {"depth_stamp": 10.2}, {"rgb_stamp": 10.02},
    {"rgb_stamp": float("nan")}, {"now": True}, {"max_age_s": 0},
    {"max_sync_delta_s": -1}, {"stamp_clock": ""}, {"calibration_revision": ""},
    {"mask_source": "bbox"}, {"registered_rectified": False},
    {"depth_semantics": "range"}, {"encoding": "8UC1"},
    {"intrinsics": [100, 1, 2, 0, 100, 2, 0, 0, 1]},
    {"intrinsics": [0, 0, 2, 0, 100, 2, 0, 0, 1]},
    {"optical_to_base_footprint": np.zeros((4, 4))},
    {"mask": np.ones((5, 5))}, {"mask": np.ones((4, 4), dtype=bool)},
    {"mask": np.zeros((5, 5), dtype=bool)}, {"depth_image": np.zeros((5, 5))},
])
def test_reject_unproven_or_invalid_inputs(overrides):
    with pytest.raises(ValueError):
        sample(**overrides)
