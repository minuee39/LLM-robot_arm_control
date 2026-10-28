"""Measured square top-boundary diagnostics, not collision-model approval."""
from dataclasses import dataclass

import cv2
import numpy as np

from .box_pose_samples import _finite_number


@dataclass(frozen=True)
class SquareBoundaryCandidate:
    top_center_base_m: np.ndarray
    rotation_base: np.ndarray
    observed_size_xy_m: np.ndarray
    yaw_mod_quarter_turn_rad: float
    edge_support_counts: tuple
    edge_support_spans_m: tuple
    hull_residual_max_m: float


class SquareBoundaryRejected(ValueError):
    """Rejected fit metrics for diagnostics; never an accepted orientation."""

    def __init__(self, reason, diagnostics):
        super().__init__(reason)
        self.diagnostics = diagnostics


def fit_square_boundary_candidate(
    points_base_m, normal_base, plane_origin_base_m, *, expected_side_m,
    max_size_error_m, max_plane_residual_m, edge_band_m,
    min_edge_support, min_edge_span_fraction, max_hull_residual_m,
    image_edge_clipped,
):
    """Fit in measured plane coordinates, never in perspective image pixels.

    Input must contain observed top-surface points, including its boundary,
    not points inferred by completing a partially visible box. All four edge
    support counts/spans and nominal size are checked. These tests can reject
    incomplete views but cannot prove visibility or bound calibration errors;
    the result remains a diagnostic candidate, not a trusted full box pose.
    Square yaw is observable only modulo pi/2. No PCA yaw is used.
    """
    if image_edge_clipped is not False:
        raise ValueError("unclipped image boundary must be established")
    points = np.asarray(points_base_m, dtype=float)
    normal = np.asarray(normal_base, dtype=float)
    origin = np.asarray(plane_origin_base_m, dtype=float)
    if (points.ndim != 2 or points.shape[1:] != (3,) or len(points) < 4
            or not np.all(np.isfinite(points)) or normal.shape != (3,)
            or origin.shape != (3,) or not np.all(np.isfinite(origin))
            or not np.all(np.isfinite(normal))
            or abs(np.linalg.norm(normal)-1) > 1e-6 or normal[2] <= 0):
        raise ValueError("finite points/origin and measured upward unit normal required")
    side, size_error, plane_error, band, span_fraction, hull_error = [
        _finite_number(value, name) for name, value in (
            ("side", expected_side_m), ("size error", max_size_error_m),
            ("plane residual", max_plane_residual_m), ("edge band", edge_band_m),
            ("edge span fraction", min_edge_span_fraction),
            ("hull residual", max_hull_residual_m))]
    if (side <= 0 or not 0 < size_error < side/2 or plane_error <= 0
            or not 0 < band < side/4 or not 0 < span_fraction <= 1 or hull_error <= 0
            or isinstance(min_edge_support, bool) or not isinstance(min_edge_support, int)
            or min_edge_support < 3):
        raise ValueError("invalid square boundary quality limits")
    delta = points-origin
    if np.max(np.abs(delta @ normal)) > plane_error:
        raise ValueError("top points exceed measured plane residual")
    # Project base X into the measured plane; do not force a horizontal plane.
    basis_x = np.array([1., 0., 0.]) - normal[0]*normal
    if np.linalg.norm(basis_x) < 1e-6:
        raise ValueError("normal too close to horizontal")
    basis_x /= np.linalg.norm(basis_x)
    basis_y = np.cross(normal, basis_x)
    basis = np.column_stack((basis_x, basis_y))
    xy = delta @ basis
    # Recentered metre coordinates keep float32 OpenCV fitting well conditioned.
    hull = cv2.convexHull(xy.astype(np.float32)).reshape(-1, 2)
    if len(hull) < 3:
        raise ValueError("degenerate top boundary")
    rectangle = cv2.minAreaRect(hull)
    corners = cv2.boxPoints(rectangle).astype(float)
    edge = corners[1]-corners[0]
    if np.linalg.norm(edge) < 1e-9:
        raise ValueError("degenerate rectangle")
    angle = float(np.arctan2(edge[1], edge[0]) % (np.pi/2))
    # Snap only the representational branch cut, not the measured angle.
    if np.pi/2-angle < 1e-7:
        angle = 0.
    axes = np.array([[np.cos(angle), -np.sin(angle)],
                     [np.sin(angle), np.cos(angle)]])
    local = xy @ axes
    low, high = local.min(axis=0), local.max(axis=0)
    size = high-low
    if np.any(np.abs(size-side) > size_error):
        raise ValueError("observed square size exceeds residual limit")
    counts, spans = [], []
    for axis in (0, 1):
        for limit in (low[axis], high[axis]):
            supported = np.abs(local[:, axis]-limit) <= band
            count = int(supported.sum())
            edge_span = float(np.ptp(local[supported, 1-axis])) if count else 0.
            if count < min_edge_support or edge_span < span_fraction*side:
                raise ValueError("missing or insufficient measured edge support")
            counts.append(count)
            spans.append(edge_span)
    hull_local = hull @ axes
    # Inspect entire hull segments, not vertices alone: a cut corner joins
    # two well-supported edges with a diagonal whose endpoints look valid.
    # Inside the rectangle, distance to its nearest edge is the minimum of
    # four affine functions. Its segment maximum occurs at an endpoint or a
    # pairwise crossing. Check these exactly rather than sampling the chord.
    residual = 0.
    for start, end in zip(hull_local, np.roll(hull_local, -1, axis=0)):
        a = np.array([start[0]-low[0], high[0]-start[0],
                      start[1]-low[1], high[1]-start[1]])
        b = np.array([end[0]-low[0], high[0]-end[0],
                      end[1]-low[1], high[1]-end[1]]) - a
        candidates = [0., 1.]
        for i in range(4):
            for j in range(i+1, 4):
                denominator = b[i]-b[j]
                if abs(denominator) > 1e-15:
                    crossing = (a[j]-a[i])/denominator
                    if 0 < crossing < 1:
                        candidates.append(crossing)
        residual = max(residual, max(float(np.min(a+t*b)) for t in candidates))
    if residual > hull_error:
        raise SquareBoundaryRejected("top hull deviates from a square boundary", {
            "accepted": False,
            "yaw_mod_quarter_turn_rad": angle,
            "observed_size_xy_m": size.tolist(),
            "edge_support_counts": counts,
            "edge_support_spans_m": spans,
            "hull_residual_max_m": residual,
            "hull_residual_limit_m": hull_error,
        })
    center = origin + basis @ (axes @ ((low+high)/2))
    rotation = np.column_stack((basis @ axes, normal))
    for array in (center, rotation, size):
        array.setflags(write=False)
    return SquareBoundaryCandidate(center, rotation, size, angle,
                                   tuple(counts), tuple(spans), residual)
