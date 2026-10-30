from itertools import product

import numpy as np
import pytest

from vision.box_enclosure import conservative_box_enclosure, enclosures_separated


def rotation(angle):
    return np.array([[np.cos(angle), -np.sin(angle), 0],
                     [np.sin(angle), np.cos(angle), 0], [0, 0, 1]])


def box(center=(0., 0., 0.), yaw=0., **overrides):
    bounds = dict(position_error_xyz_m=[0., 0., 0.], rotation_error_rad=0.,
                  size_error_xyz_m=[0., 0., 0.], bound_source="synthetic_test_only")
    bounds.update(overrides)
    return conservative_box_enclosure(center, rotation(yaw), [.03, .03, .05], **bounds)


def test_uncertain_corner_samples_contained_in_measured_axes():
    measured = rotation(.3)
    position_error = np.array([.001, .002, .0005])
    size_error = np.array([.0001, .0003, .0002])
    envelope = box(yaw=.3, position_error_xyz_m=position_error,
                   size_error_xyz_m=size_error, rotation_error_rad=.1)
    rng = np.random.default_rng(2)
    signs = np.array(list(product((-1, 1), repeat=3)))
    for _ in range(300):
        axis = rng.normal(size=3); axis /= np.linalg.norm(axis)
        angle = rng.uniform(-.1, .1)
        k = np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]])
        perturbation = np.eye(3)+np.sin(angle)*k+(1-np.cos(angle))*(k@k)
        size = np.array([.03, .03, .05])+rng.uniform(-1, 1, 3)*size_error
        center = rng.uniform(-1, 1, 3)*position_error
        corners = (signs*size/2) @ (perturbation @ measured).T + center
        assert np.all(np.abs(corners @ measured) <= envelope.size_m/2+1e-12)


def test_original_40mm_cells_not_arbitrarily_declared_clear():
    assert enclosures_separated(box(), box((.04, 0, 0)), numerical_margin_m=1e-9)
    assert not enclosures_separated(box(yaw=np.pi/4), box((.04, 0, 0), yaw=np.pi/4), numerical_margin_m=1e-9)
    assert not enclosures_separated(box(position_error_xyz_m=[.006]*3),
                                    box((.04, 0, 0), position_error_xyz_m=[.006]*3), numerical_margin_m=1e-9)


def test_touching_is_not_separated():
    assert not enclosures_separated(box(), box((.03, 0, 0)), numerical_margin_m=0.)


@pytest.mark.parametrize("override", [
    {"position_error_xyz_m": None}, {"rotation_error_rad": float("nan")},
    {"size_error_xyz_m": [-.001, 0, 0]}, {"bound_source": ""},
])
def test_missing_invalid_bounds_rejected(override):
    with pytest.raises(ValueError): box(**override)
