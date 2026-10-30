"""Conditional pixel footprint only; not a full pose uncertainty certificate."""
import numpy as np

from .box_pose_samples import _finite_number
from .calibration import validate_rigid_transform


def pixel_plane_error_bounds(
    pixel_uv, intrinsics, optical_to_base, plane_origin_m, plane_normal,
    *, max_pixel_error_px,
):
    """Worst displacement for a pixel's +/- error rectangle on a fixed plane.

    The transform, intrinsics and measured plane are held exact. This excludes
    their errors, depth noise, mask errors and unobserved edges. For each pixel,
    ray-plane projection maps its error rectangle to a convex quadrilateral
    while the denominator keeps its sign; distance is maximal at a vertex.
    Reject rectangles crossing the plane horizon or projecting behind camera.
    """
    uv = np.asarray(pixel_uv, dtype=float)
    k = np.asarray(intrinsics, dtype=float)
    if k.shape == (9,):
        k = k.reshape(3, 3)
    normal = np.asarray(plane_normal, dtype=float)
    origin = np.asarray(plane_origin_m, dtype=float)
    error = _finite_number(max_pixel_error_px, "max_pixel_error_px")
    if (uv.ndim != 2 or uv.shape[1:] != (2,) or not len(uv)
            or not np.all(np.isfinite(uv)) or error <= 0
            or k.shape != (3, 3) or not np.all(np.isfinite(k))
            or k[0, 0] <= 0 or k[1, 1] <= 0
            or not np.allclose(k[2], [0, 0, 1], atol=1e-9, rtol=0)
            or abs(k[0, 1]) > 1e-9 or abs(k[1, 0]) > 1e-9
            or normal.shape != (3,) or not np.all(np.isfinite(normal))
            or abs(np.linalg.norm(normal)-1) > 1e-6
            or origin.shape != (3,) or not np.all(np.isfinite(origin))):
        raise ValueError("invalid pixels, rectified intrinsics, plane or pixel error")
    transform = validate_rigid_transform(optical_to_base)
    offsets = np.array([[0, 0], [-error, -error], [-error, error],
                        [error, -error], [error, error]])
    pixels = uv[:, None, :] + offsets
    optical = np.concatenate(((pixels-k[:2, 2])/np.array([k[0, 0], k[1, 1]]),
                              np.ones((*pixels.shape[:2], 1))), axis=2)
    rays = optical @ transform[:3, :3].T
    denominator = rays @ normal
    numerator = float((origin-transform[:3, 3]) @ normal)
    if (np.any(np.abs(denominator) < 1e-9)
            or np.any(np.sign(denominator) != np.sign(denominator[:, :1]))):
        raise ValueError("pixel error rectangle crosses plane horizon")
    distance = numerator/denominator
    if np.any(distance <= 0) or not np.all(np.isfinite(distance)):
        raise ValueError("plane projection is behind camera or nonfinite")
    intersections = rays*distance[:, :, None] + transform[:3, 3]
    bounds = np.linalg.norm(intersections[:, 1:]-intersections[:, :1], axis=2).max(axis=1)
    bounds.setflags(write=False)
    return bounds
