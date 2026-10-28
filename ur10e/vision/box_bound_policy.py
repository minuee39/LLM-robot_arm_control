"""Diagnostic-only applicability checks for explicit conditional error bounds.

Numbers and prose supplied here are assumptions, not an error certificate.
No policy is inferred from observed variance or marked execution-approved.
"""
from dataclasses import dataclass

import numpy as np

from .box_enclosure import BoxEnclosure, conservative_box_enclosure
from .box_pose_reader import DiagnosticBoxPose, _array, _number, validate_square_pose_geometry


@dataclass(frozen=True)
class ConditionalBoxPolicy:
    measurement_revision: str
    nominal_size_m: tuple
    position_error_xyz_m: tuple
    rotation_error_rad: float
    size_error_xyz_m: tuple
    valid_from_unix: float
    valid_until_unix: float
    position_min_m: tuple
    position_max_m: tuple
    bound_source: str
    assumptions: tuple

    def __post_init__(self):
        for name in ('measurement_revision', 'bound_source'):
            value = getattr(self, name)
            if not isinstance(value, str) or not value.strip():
                raise ValueError(f'explicit {name} required')
        if (not isinstance(self.assumptions, (tuple, list)) or not self.assumptions
                or any(not isinstance(a, str) or not a.strip() for a in self.assumptions)):
            raise ValueError('explicit conditional assumptions required; not certification')
        object.__setattr__(self, 'assumptions', tuple(self.assumptions))
        for name in ('nominal_size_m', 'position_error_xyz_m', 'size_error_xyz_m',
                     'position_min_m', 'position_max_m'):
            object.__setattr__(self, name, tuple(map(float, _array(getattr(self, name), (3,), name))))
        for name in ('rotation_error_rad', 'valid_from_unix', 'valid_until_unix'):
            object.__setattr__(self, name, _number(getattr(self, name), name))
        if (any(s <= 0 for s in self.nominal_size_m)
                or any(e < 0 for e in (*self.position_error_xyz_m, *self.size_error_xyz_m))
                or not 0 <= self.rotation_error_rad <= np.pi
                or not 0 <= self.valid_from_unix < self.valid_until_unix
                or np.any(np.asarray(self.position_min_m) >= self.position_max_m)):
            raise ValueError('invalid dimensions, error bounds, validity interval or position scope')


@dataclass(frozen=True)
class ConditionalDiagnosticEnvelope:
    object_id: str
    geometry: BoxEnclosure
    policy: ConditionalBoxPolicy
    diagnostic_only: bool = True
    collision_model_approved: bool = False
    uncertainty: str = 'conditional_assumptions_not_certified'


def diagnostic_box_envelope(pose, policy, *, now_unix, max_age_s):
    """Compute a conditional envelope only inside explicitly supplied scope.

    This cannot certify policy assumptions or gripper/path collision clearance.
    Recompute exposure age: retaining a previously validated pose renews nothing.
    """
    now = _number(now_unix, 'Unix now')
    max_age = _number(max_age_s, 'max age')
    if not 0 < max_age <= 1:
        raise ValueError('diagnostic max age must be within (0,1] seconds')
    if (not isinstance(pose, DiagnosticBoxPose) or not isinstance(policy, ConditionalBoxPolicy)
            or pose.diagnostic_only is not True or pose.collision_model_approved is not False
            or pose.uncertainty != 'not_certified' or pose.frame != 'base_footprint'
            or pose.unit != 'm' or pose.stamp_clock != 'unix_system'
            or pose.source != 'rgbd_top_plane_square_boundary'
            or pose.calibration_revision != policy.measurement_revision):
        raise ValueError('matching diagnostic pose and conditional policy required')
    stamp = _number(pose.exposure_stamp, 'exposure time')
    if (not policy.valid_from_unix <= stamp <= now <= policy.valid_until_unix
            or not 0 <= now-stamp <= max_age):
        raise ValueError('pose or conditional policy is expired/future/outside validity')
    position, _normal, rotation, _yaw = validate_square_pose_geometry(
        pose.position_m, pose.normal_base, pose.rotation_base, pose.yaw_mod_quarter_turn_rad,
    )
    error = np.asarray(policy.position_error_xyz_m)
    if (np.any(position-error < policy.position_min_m)
            or np.any(position+error > policy.position_max_m)):
        raise ValueError('possible position interval exceeds conditional policy scope')
    geometry = conservative_box_enclosure(
        position, rotation, policy.nominal_size_m,
        position_error_xyz_m=policy.position_error_xyz_m,
        rotation_error_rad=policy.rotation_error_rad,
        size_error_xyz_m=policy.size_error_xyz_m, bound_source=policy.bound_source,
    )
    return ConditionalDiagnosticEnvelope(pose.object_id, geometry, policy)
