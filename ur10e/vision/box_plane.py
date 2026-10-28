"""Diagnostic top-plane candidate fitting, not an accepted box orientation."""

from dataclasses import dataclass

import numpy as np

from .box_pose_samples import MaskDepthSamples, _finite_number


@dataclass(frozen=True)
class PlaneCandidate:
    normal_base: np.ndarray
    centroid_base_m: np.ndarray
    inlier_indices: np.ndarray
    residual_rms_m: float
    inlier_fraction: float
    tilt_rad: float
    minor_span_m: float


def fit_top_plane_candidate(
    samples, *, distance_limit_m, max_tilt_rad, min_inliers,
    min_inlier_fraction, min_minor_span_m, trials=96,
):
    """Find a measured +Z-facing plane using only eroded mask depth samples.

    Caller supplies every acceptance limit; no hardware noise/clearance limit
    is inferred here. Normal sign is canonicalized, not replaced with +Z.
    This cannot establish visible box edges, yaw, shape or pose uncertainty.
    RANSAC is bounded/deterministic and may reject a visible plane it misses.
    """
    if not isinstance(samples, MaskDepthSamples):
        raise ValueError("validated MaskDepthSamples are required")
    distance = _finite_number(distance_limit_m, "distance_limit_m")
    tilt_limit = _finite_number(max_tilt_rad, "max_tilt_rad")
    fraction = _finite_number(min_inlier_fraction, "min_inlier_fraction")
    span = _finite_number(min_minor_span_m, "min_minor_span_m")
    if (distance <= 0 or not 0 <= tilt_limit < np.pi/2
            or not 0 < fraction <= 1 or span <= 0):
        raise ValueError("invalid plane quality limits")
    if (isinstance(min_inliers, bool) or not isinstance(min_inliers, int)
            or min_inliers < 3 or isinstance(trials, bool)
            or not isinstance(trials, int) or not 1 <= trials <= 10000):
        raise ValueError("inliers/trials must be bounded positive integers")
    indices = np.flatnonzero(samples.interior)
    points = samples.points_base_m[indices]
    if len(points) < min_inliers:
        raise ValueError("insufficient interior depth points")
    rng = np.random.default_rng(0)
    best = None
    cosine_limit = np.cos(tilt_limit)
    for _ in range(trials):
        p = points[rng.choice(len(points), 3, replace=False)]
        normal = np.cross(p[1]-p[0], p[2]-p[0])
        norm = np.linalg.norm(normal)
        if norm <= np.finfo(float).eps:
            continue
        normal /= norm
        if normal[2] < 0:
            normal = -normal
        if normal[2] < cosine_limit:
            continue
        residual = np.abs((points-p[0]) @ normal)
        inliers = residual <= distance
        count = int(inliers.sum())
        if count < min_inliers or count/len(points) < fraction:
            continue
        score = (count, -float(np.mean(residual[inliers]**2)))
        if best is None or score > best[0]:
            best = (score, inliers)
    if best is None:
        raise ValueError("no supported top-plane candidate")
    inliers = best[1]
    # Refit twice and revalidate rather than reporting the seed's quality.
    for _ in range(2):
        centroid = points[inliers].mean(axis=0)
        _u, singular, vh = np.linalg.svd(points[inliers]-centroid, full_matrices=False)
        normal = vh[-1]
        if normal[2] < 0:
            normal = -normal
        residual = np.abs((points-centroid) @ normal)
        new_inliers = residual <= distance
        if new_inliers.sum() < min_inliers or new_inliers.sum()/len(points) < fraction:
            raise ValueError("refitted plane lost support")
        if np.array_equal(inliers, new_inliers):
            break
        inliers = new_inliers
    centroid = points[inliers].mean(axis=0)
    _u, singular, vh = np.linalg.svd(points[inliers]-centroid, full_matrices=False)
    normal = vh[-1]
    if normal[2] < 0:
        normal = -normal
    if normal[2] < cosine_limit:
        raise ValueError("refitted normal exceeds tilt limit")
    # Support in both plane axes; a line cannot establish a plane normal.
    minor_span = float(np.ptp((points[inliers]-centroid) @ vh[1]))
    if singular[1] <= np.finfo(float).eps or minor_span < span:
        raise ValueError("plane support is degenerate or too narrow")
    residual = np.abs((points[inliers]-centroid) @ normal)
    if np.any(residual > distance):
        raise ValueError("final plane residual exceeds limit")
    selected = indices[inliers].copy()
    for array in (normal, centroid, selected):
        array.setflags(write=False)
    return PlaneCandidate(normal, centroid, selected,
                          float(np.sqrt(np.mean(residual**2))),
                          float(inliers.mean()), float(np.arccos(np.clip(normal[2], -1, 1))),
                          minor_span)
