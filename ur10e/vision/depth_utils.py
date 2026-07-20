import numpy as np


def median_depth_in_bbox(depth_image, bbox, center_fraction=0.3):
    if depth_image is None or np.asarray(depth_image).ndim != 2:
        raise ValueError("depth_image must be a 2D array")
    if not 0.0 < center_fraction <= 1.0:
        raise ValueError("center_fraction must be in the range (0, 1]")

    depth_image = np.asarray(depth_image)
    height, width = depth_image.shape
    x1, y1, x2, y2 = [int(value) for value in bbox]
    x1 = int(np.clip(x1, 0, width))
    x2 = int(np.clip(x2, 0, width))
    y1 = int(np.clip(y1, 0, height))
    y2 = int(np.clip(y2, 0, height))
    if x2 <= x1 or y2 <= y1:
        return None

    center_u = (x1 + x2) / 2.0
    center_v = (y1 + y2) / 2.0
    sample_width = max(1, int(round((x2 - x1) * center_fraction)))
    sample_height = max(1, int(round((y2 - y1) * center_fraction)))
    sample_x1 = max(x1, int(round(center_u - sample_width / 2.0)))
    sample_x2 = min(x2, sample_x1 + sample_width)
    sample_y1 = max(y1, int(round(center_v - sample_height / 2.0)))
    sample_y2 = min(y2, sample_y1 + sample_height)

    samples = depth_image[sample_y1:sample_y2, sample_x1:sample_x2].astype(float)
    valid_samples = samples[np.isfinite(samples) & (samples > 0.0)]
    if valid_samples.size == 0:
        return None
    return float(np.median(valid_samples))


def pixel_to_camera_point(u, v, depth, fx, fy, cx, cy):
    if not np.isfinite(depth) or depth <= 0.0:
        raise ValueError("depth must be finite and positive")
    if fx <= 0.0 or fy <= 0.0:
        raise ValueError("fx and fy must be positive")
    x = (u - cx) * depth / fx
    y = (v - cy) * depth / fy
    z = depth
    return np.array([x, y, z])


def transform_point(point, transform_matrix):
    point_h = np.array([point[0], point[1], point[2], 1.0])
    result = transform_matrix @ point_h
    return result[:3]


def surface_point_to_box_center(surface_point, camera_origin, box_size):
    """Estimate an axis-aligned box center from the ray's visible surface point."""
    surface_point = np.asarray(surface_point, dtype=float)
    camera_origin = np.asarray(camera_origin, dtype=float)
    box_size = np.asarray(box_size, dtype=float)
    if surface_point.shape != (3,) or camera_origin.shape != (3,) or box_size.shape != (3,):
        raise ValueError("surface point, camera origin, and box size must be three-dimensional")
    if not np.all(np.isfinite(surface_point)) or not np.all(np.isfinite(camera_origin)):
        raise ValueError("surface point and camera origin must be finite")
    if not np.all(np.isfinite(box_size)) or np.any(box_size <= 0.0):
        raise ValueError("box size must be finite and positive")

    ray = surface_point - camera_origin
    ray_norm = np.linalg.norm(ray)
    if ray_norm <= 1e-9:
        raise ValueError("surface point must differ from camera origin")
    direction = ray / ray_norm

    half_size = box_size / 2.0
    valid_axes = np.abs(direction) > 1e-9
    center_distance = np.min(half_size[valid_axes] / np.abs(direction[valid_axes]))
    return surface_point + direction * center_distance
