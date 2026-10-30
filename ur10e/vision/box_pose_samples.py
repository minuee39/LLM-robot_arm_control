"""Measured RGB-D samples for a future box-pose estimator (not a pose).

No simulator object pose, box size, table height or assumed normal is used.
Registration, optical-Z depth and rectified intrinsics must be established by
the caller. Quality limits are caller-supplied; this module does not certify
that the resulting cloud is accurate enough for collision-model reduction.
"""

from dataclasses import dataclass

import numpy as np

from .calibration import validate_rigid_transform


@dataclass(frozen=True)
class MaskDepthSamples:
    points_base_m: np.ndarray
    pixel_uv: np.ndarray
    boundary_uv: np.ndarray
    interior: np.ndarray
    valid_depth_fraction: float
    mask_pixel_count: int
    image_edge_clipped: bool
    rgb_stamp: float
    depth_stamp: float
    stamp_clock: str
    calibration_revision: str
    mask_source: str
    frame: str = "base_footprint"
    unit: str = "m"
    pixel_center_offset_px: float = 0.


def _finite_number(value, name):
    if isinstance(value, (bool, np.bool_)):
        raise ValueError(f"{name} must be a finite number")
    try:
        result = float(value)
    except (TypeError, ValueError) as error:
        raise ValueError(f"{name} must be a finite number") from error
    if not np.isfinite(result):
        raise ValueError(f"{name} must be a finite number")
    return result


def mask_depth_samples(
    depth_image, mask, *, encoding, intrinsics, optical_to_base_footprint,
    rgb_stamp, depth_stamp, now, max_age_s, max_sync_delta_s, stamp_clock,
    calibration_revision, mask_source, registered_rectified, depth_semantics,
    pixel_center_offset_px=0.,
):
    """Back-project each valid mask pixel with its own depth into base metres.

    Timestamps and ``now`` must share the explicitly named clock. A file-write
    timestamp is not an exposure timestamp. A bounding rectangle is not a mask.
    Original boundary rays are kept separately from the eroded plane-fitting
    interior, including boundary pixels whose depth is invalid.
    """
    if registered_rectified is not True or depth_semantics != "optical_z":
        raise ValueError("registered rectified RGB-D with optical_z depth is required")
    offset = _finite_number(pixel_center_offset_px, "pixel center offset")
    if offset not in (0., .5):
        raise ValueError("pixel center offset must be established as 0 or 0.5 pixels")
    if mask_source not in {"hsv_color_mask", "yolo_segmentation_mask"}:
        raise ValueError("an actual segmentation mask source is required")
    for name, value in (("stamp_clock", stamp_clock),
                        ("calibration_revision", calibration_revision)):
        if not isinstance(value, str) or not value.strip():
            raise ValueError(f"{name} must be a nonempty string")
    stamps = [_finite_number(value, name) for name, value in (
        ("rgb_stamp", rgb_stamp), ("depth_stamp", depth_stamp), ("now", now))]
    age_limit = _finite_number(max_age_s, "max_age_s")
    sync_limit = _finite_number(max_sync_delta_s, "max_sync_delta_s")
    if age_limit <= 0 or sync_limit < 0:
        raise ValueError("age must be positive and sync limit nonnegative")
    if any(stamp < 0 or stamp > stamps[2] or stamps[2] - stamp > age_limit
           for stamp in stamps[:2]):
        raise ValueError("exposure timestamp is stale, negative or in the future")
    if abs(stamps[0] - stamps[1]) > sync_limit:
        raise ValueError("RGB-depth exposure timestamps exceed sync limit")
    depth = np.asarray(depth_image)
    mask = np.asarray(mask)
    if depth.ndim != 2 or not depth.size or mask.shape != depth.shape:
        raise ValueError("depth and mask must be nonempty matching 2D images")
    if mask.dtype != np.bool_:
        raise ValueError("mask must be boolean; convert a validated binary mask explicitly")
    if not np.issubdtype(depth.dtype, np.number) or np.iscomplexobj(depth):
        raise ValueError("depth image must be real numeric")
    mask_count = int(np.count_nonzero(mask))
    if not mask_count:
        raise ValueError("mask must contain foreground pixels")
    scale = {"16UC1": .001, "MONO16": .001, "32FC1": 1.0}.get(
        str(encoding).strip().upper())
    if scale is None:
        raise ValueError("unsupported depth encoding")
    k = np.asarray(intrinsics, dtype=float)
    if k.shape == (9,):
        k = k.reshape(3, 3)
    if (k.shape != (3, 3) or not np.all(np.isfinite(k))
            or k[0, 0] <= 0 or k[1, 1] <= 0
            or not np.allclose(k[2], [0, 0, 1], atol=1e-9, rtol=0)
            or abs(k[0, 1]) > 1e-9 or abs(k[1, 0]) > 1e-9):
        raise ValueError("intrinsics must be a finite rectified zero-skew camera matrix")
    transform = validate_rigid_transform(optical_to_base_footprint)
    depth_m = depth.astype(float) * scale
    valid = mask & np.isfinite(depth_m) & (depth_m > 0)
    v, u = np.nonzero(valid)
    if not len(u):
        raise ValueError("mask contains no valid positive depth")
    z = depth_m[v, u]
    optical = np.column_stack(((u+offset-k[0, 2])*z/k[0, 0],
                               (v+offset-k[1, 2])*z/k[1, 1], z))
    points = optical @ transform[:3, :3].T + transform[:3, 3]
    padded = np.pad(mask, 1, constant_values=False)
    interior_mask = np.ones_like(mask)
    h, w = mask.shape
    for dy in range(3):
        for dx in range(3):
            interior_mask &= padded[dy:dy+h, dx:dx+w]
    bv, bu = np.nonzero(mask & ~interior_mask)
    arrays = (points, np.column_stack((u, v)), np.column_stack((bu, bv)),
              interior_mask[v, u].copy())
    for array in arrays:
        array.setflags(write=False)
    return MaskDepthSamples(
        *arrays, len(u)/mask_count, mask_count,
        bool(mask[0].any() or mask[-1].any() or mask[:, 0].any() or mask[:, -1].any()),
        stamps[0], stamps[1], stamp_clock, calibration_revision, mask_source,
        pixel_center_offset_px=offset,
    )
