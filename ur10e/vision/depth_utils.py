import numpy as np


def pixel_to_camera_point(u, v, depth, fx, fy, cx, cy):
    x = (u - cx) * depth / fx
    y = (v - cy) * depth / fy
    z = depth
    return np.array([x, y, z])


def transform_point(point, transform_matrix):
    point_h = np.array([point[0], point[1], point[2], 1.0])
    result = transform_matrix @ point_h
    return result[:3]