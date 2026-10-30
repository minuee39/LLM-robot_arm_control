"""Strict diagnostic snapshot consumer; never certifies collision geometry.

This separate reader cannot populate legacy SceneManager or execution obstacles.
It recomputes Unix exposure age rather than trusting serialized age/variance.
"""
from dataclasses import dataclass
import json
from pathlib import Path

import numpy as np

from .scene_objects import EXPECTED_BLOCK_NAMES


def _number(value, name):
    if type(value) not in (int, float) or not np.isfinite(value):
        raise ValueError(f'{name} requires a finite JSON number')
    return float(value)


def _unique_json_object(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError('duplicate JSON object key')
        result[key] = value
    return result


def _array(value, shape, name):
    raw = np.asarray(value, dtype=object)
    if raw.shape != shape:
        raise ValueError(f'invalid {name} shape')
    return np.array([_number(v, name) for v in raw.flat]).reshape(shape)


def validate_square_pose_geometry(position, normal, rotation, yaw):
    """Revalidate one measured bundle, including square-periodic yaw agreement."""
    position = _array(position, (3,), 'position')
    normal = _array(normal, (3,), 'normal')
    rotation = _array(rotation, (3, 3), 'rotation')
    yaw = _number(yaw, 'square yaw')
    if (abs(np.linalg.norm(normal)-1) > 1e-6 or normal[2] <= 0
            or not np.allclose(rotation.T@rotation, np.eye(3), atol=1e-6, rtol=0)
            or abs(np.linalg.det(rotation)-1) > 1e-6
            or not np.allclose(rotation[:, 2], normal, atol=1e-6, rtol=0)):
        raise ValueError('invalid measured normal/rotation')
    x = np.array([1., 0., 0.])-normal[0]*normal
    x /= np.linalg.norm(x)
    y = np.cross(normal, x)
    observed_yaw = np.arctan2(rotation[:, 0]@y, rotation[:, 0]@x)
    delta = (observed_yaw-yaw+np.pi/4) % (np.pi/2)-np.pi/4
    if abs(delta) > 1e-6:
        raise ValueError('rotation and square yaw disagree')
    return position, normal, rotation, yaw


@dataclass(frozen=True)
class DiagnosticBoxPose:
    object_id: str
    position_m: tuple
    rotation_base: tuple
    normal_base: tuple
    yaw_mod_quarter_turn_rad: float
    exposure_stamp: float
    exposure_age_s: float
    calibration_revision: str
    sample_count: int
    position_std_m: tuple
    max_yaw_spread_rad: float
    max_normal_spread_rad: float
    source: str = 'rgbd_top_plane_square_boundary'
    frame: str = 'base_footprint'
    unit: str = 'm'
    stamp_clock: str = 'unix_system'
    diagnostic_only: bool = True
    collision_model_approved: bool = False
    uncertainty: str = 'not_certified'


@dataclass(frozen=True)
class DiagnosticPoseSnapshot:
    updated_at_unix: float
    validated_at_unix: float
    objects: tuple
    diagnostic_only: bool = True
    collision_model_approved: bool = False


class DiagnosticPoseReader:
    """Consume complete, increasing snapshots for one explicit revision.

    Duplicates/reverse exposures are rejected, never used to renew freshness.
    Failed reads return no cached pose. A new reader is an explicit epoch reset,
    not implicit acceptance of an unexpected calibration revision.
    """
    def __init__(self, *, expected_revision, max_age_s):
        if not isinstance(expected_revision, str) or not expected_revision.strip():
            raise ValueError('expected measurement revision required')
        age = _number(max_age_s, 'max age')
        if not 0 < age <= 1:
            raise ValueError('diagnostic max age must be within (0,1] seconds')
        self.expected_revision = expected_revision
        self.max_age_s = age
        self._last_root = None
        self._last_exposures = {}

    def read(self, path, *, now_unix):
        return self.parse(json.loads(Path(path).read_text(), object_pairs_hook=_unique_json_object),
                          now_unix=now_unix)

    def parse(self, payload, *, now_unix):
        now = _number(now_unix, 'Unix now')
        if now < 0 or not isinstance(payload, dict):
            raise ValueError('nonnegative Unix now and object snapshot required')
        if payload.get('diagnostic_only') is not True or payload.get('collision_model_approved') is not False:
            raise ValueError('explicit diagnostic-only snapshot required')
        updated = _number(payload.get('updated_at_unix'), 'snapshot update time')
        if not 0 <= now-updated <= self.max_age_s:
            raise ValueError('stale or future snapshot update')
        objects = payload.get('objects')
        if not isinstance(objects, dict) or set(objects) != set(EXPECTED_BLOCK_NAMES):
            raise ValueError('complete known three-block snapshot required')
        poses = []
        for name in EXPECTED_BLOCK_NAMES:
            item = objects[name]
            if (not isinstance(item, dict) or item.get('diagnostic_only') is not True
                    or item.get('collision_model_approved') is not False
                    or item.get('uncertainty') != 'not_certified'):
                raise ValueError('explicit uncertified diagnostic object required')
            if (item.get('frame') != 'base_footprint' or item.get('unit') != 'm'
                    or item.get('stamp_clock') != 'unix_system'
                    or item.get('source') != 'rgbd_top_plane_square_boundary'
                    or item.get('calibration_revision') != self.expected_revision):
                raise ValueError('object clock/frame/unit/source/revision mismatch')
            stamp = _number(item.get('exposure_stamp'), 'exposure time')
            # File writes cannot make an older exposure fresh or precede exposure.
            age = now-stamp
            if stamp < 0 or stamp > updated or not 0 <= age <= self.max_age_s:
                raise ValueError('stale or future object exposure')
            position, normal, rotation, yaw = validate_square_pose_geometry(
                item.get('position'), item.get('normal_base'), item.get('rotation_base'),
                item.get('yaw_mod_quarter_turn_rad'),
            )
            count = item.get('sample_count')
            std = _array(item.get('position_std_m'), (3,), 'position std')
            yaw_spread = _number(item.get('max_yaw_spread_rad'), 'yaw spread')
            normal_spread = _number(item.get('max_normal_spread_rad'), 'normal spread')
            if (type(count) is not int or count < 3 or np.any(std < 0)
                    or not 0 <= yaw_spread < np.pi/4 or not 0 <= normal_spread < np.pi/2):
                raise ValueError('invalid stability diagnostics; not error bounds')
            poses.append(DiagnosticBoxPose(
                name, tuple(map(float, position)), tuple(tuple(map(float, row)) for row in rotation),
                tuple(map(float, normal)), yaw, stamp, age, self.expected_revision, count,
                tuple(map(float, std)), yaw_spread, normal_spread,
            ))
        if self._last_root is not None and updated < self._last_root:
            raise ValueError('snapshot write time went backwards')
        if any(p.exposure_stamp <= self._last_exposures.get(p.object_id, -1) for p in poses):
            raise ValueError('duplicate or reverse object exposure')
        # Commit complete bundles atomically; failure never combines old direction.
        self._last_root = updated
        self._last_exposures = {p.object_id: p.exposure_stamp for p in poses}
        return DiagnosticPoseSnapshot(updated, now, tuple(poses))
