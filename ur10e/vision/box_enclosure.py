"""Conditional conservative OBB geometry; no bound certification or planner writes."""
from dataclasses import dataclass

import numpy as np

from .box_pose_samples import _finite_number


@dataclass(frozen=True)
class BoxEnclosure:
    center_m: np.ndarray
    rotation_base: np.ndarray
    size_m: np.ndarray
    margin_local_m: np.ndarray
    bound_source: str


def conservative_box_enclosure(center_m, rotation_base, nominal_size_m, *,
                               position_error_xyz_m, rotation_error_rad,
                               size_error_xyz_m, bound_source):
    """Enclose boxes within caller-established centre/SO(3)/size error bounds.

    The nominal body's corner radius is R. Rotation error theta moves each
    nominal point by <= 2*R*sin(theta/2). Dimension error adds <= ||ds||/2.
    World-axis centre intervals are projected into measured box axes using
    abs(rotation.T). Bounds cannot be inferred from sample standard deviation.
    This computes an envelope conditional on bounds, not their validity.
    """
    center = np.asarray(center_m, dtype=float)
    rotation = np.asarray(rotation_base, dtype=float)
    size = np.asarray(nominal_size_m, dtype=float)
    position_error = np.asarray(position_error_xyz_m, dtype=float)
    size_error = np.asarray(size_error_xyz_m, dtype=float)
    angle = _finite_number(rotation_error_rad, "rotation error bound")
    if (any(v.shape != (3,) or not np.all(np.isfinite(v))
            for v in (center, size, position_error, size_error))
            or np.any(size <= 0) or np.any(position_error < 0) or np.any(size_error < 0)
            or not 0 <= angle <= np.pi or rotation.shape != (3, 3)
            or not np.all(np.isfinite(rotation))
            or not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-6, rtol=0)
            or abs(np.linalg.det(rotation)-1) > 1e-6
            or not isinstance(bound_source, str) or not bound_source.strip()):
        raise ValueError("finite geometry, rigid rotation and explicit nonnegative error bounds required")
    radius = np.linalg.norm(size)/2
    scalar_margin = 2*radius*np.sin(angle/2)+np.linalg.norm(size_error)/2
    margin = np.abs(rotation.T) @ position_error + scalar_margin
    arrays = (center.copy(), rotation.copy(), size+2*margin, margin)
    for array in arrays:
        array.setflags(write=False)
    return BoxEnclosure(*arrays, bound_source)


def enclosures_separated(first, second, *, numerical_margin_m):
    """Separating-axis test; touching or numerical ambiguity is not clear.

    Checks 3+3 face normals and nondegenerate 9 cross axes. Passing means only
    these two conditional boxes are disjoint, not gripper/path collision-free.
    """
    margin = _finite_number(numerical_margin_m, "numerical margin")
    if margin < 0:
        raise ValueError("numerical margin must be nonnegative")
    return enclosure_axis_gap(first, second) > margin


def enclosure_axis_gap(first, second):
    """Largest signed projected SAT gap, not shortest Euclidean distance.

    Positive means a separating axis exists; zero/negative is not clear.
    Valid only for the supplied conditional enclosures, not an error certificate.
    """
    if not isinstance(first, BoxEnclosure) or not isinstance(second, BoxEnclosure):
        raise ValueError("conditional BoxEnclosure inputs required")
    axes = [*first.rotation_base.T, *second.rotation_base.T]
    axes.extend(np.cross(x, y) for x in first.rotation_base.T for y in second.rotation_base.T)
    delta = second.center_m-first.center_m
    gaps = []
    for axis in axes:
        norm = np.linalg.norm(axis)
        if norm <= 1e-12:
            continue
        axis = axis/norm
        extent = (np.abs(first.rotation_base.T @ axis) @ (first.size_m/2)
                  + np.abs(second.rotation_base.T @ axis) @ (second.size_m/2))
        gaps.append(float(abs(delta @ axis)-extent))
    return max(gaps)
