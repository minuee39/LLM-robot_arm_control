"""Joint position/direction stability for diagnostic square-box observations.

This store never certifies collision geometry. Caller must establish exposure
age in the named sensor clock; receipt time is a separate monotonic clock.
"""
from collections import deque

import numpy as np

from .box_pose_samples import _finite_number
from .scene_objects import EXPECTED_BLOCK_NAMES


class StableBoxPoseStore:
    def __init__(self, *, window_size, min_samples, ttl_s,
                 max_position_std_m, max_yaw_spread_rad, max_normal_spread_rad):
        if (isinstance(window_size, bool) or not isinstance(window_size, int)
                or isinstance(min_samples, bool) or not isinstance(min_samples, int)
                or not 2 <= min_samples <= window_size):
            raise ValueError("window/min_samples must be integers with 2 <= min <= window")
        self.ttl = _finite_number(ttl_s, "ttl_s")
        self.position_limit = _finite_number(max_position_std_m, "position std")
        self.yaw_limit = _finite_number(max_yaw_spread_rad, "yaw spread")
        self.normal_limit = _finite_number(max_normal_spread_rad, "normal spread")
        if (self.ttl <= 0 or self.position_limit <= 0
                or not 0 < self.yaw_limit < np.pi/4 or not 0 < self.normal_limit < np.pi/2):
            raise ValueError("invalid joint pose stability limits")
        self.minimum = min_samples
        self.samples = {name: deque(maxlen=window_size) for name in EXPECTED_BLOCK_NAMES}
        self.last_exposure = {}
        self.identity = None

    def invalidate(self, name, *, exposure_stamp=None):
        """Any rejected/missing direction clears position and direction jointly."""
        if name in self.samples:
            self.samples[name].clear()
            if exposure_stamp is not None:
                stamp = _finite_number(exposure_stamp, "invalidated exposure stamp")
                if stamp < 0:
                    raise ValueError("invalidated exposure stamp must be nonnegative")
                self.last_exposure[name] = max(stamp, self.last_exposure.get(name, stamp))

    def _prune(self, name, now):
        samples = self.samples[name]
        # Receipt age alone must not grant a delayed exposure a new full TTL.
        self.samples[name] = deque((s for s in samples if now-s[0]+s[5] <= self.ttl),
                                   maxlen=samples.maxlen)
        return self.samples[name]

    def reset_epoch(self):
        """Explicit camera restart: no old direction survives a clock reset."""
        for samples in self.samples.values():
            samples.clear()
        self.last_exposure.clear()
        self.identity = None

    def update(self, name, candidate, *, exposure_stamp, clock,
               calibration_revision, received_at_monotonic, exposure_age_s, frame, unit):
        if name not in self.samples:
            return False
        try:
            if frame != "base_footprint" or unit != "m":
                raise ValueError("joint pose observations require base_footprint/metres")
            stamp = _finite_number(exposure_stamp, "exposure stamp")
            received = _finite_number(received_at_monotonic, "receipt time")
            age = _finite_number(exposure_age_s, "exposure age")
            if stamp < 0 or received < 0 or not 0 <= age <= self.ttl:
                raise ValueError("invalid or stale observation timestamp")
            if any(not isinstance(s, str) or not s.strip() for s in (clock, calibration_revision)):
                raise ValueError("clock and calibration revision are required")
            identity = (clock, calibration_revision)
            if identity != self.identity:
                self.reset_epoch()
                self.identity = identity
            previous = self.last_exposure.get(name)
            if previous is not None and stamp <= previous:
                # Duplicates do not extend TTL. Reverse order invalidates rather
                # than mixing epochs; restart requires explicit reset_epoch.
                if stamp < previous:
                    self.invalidate(name)
                return False
            # Even a rejected newer direction advances the exposure watermark.
            self.last_exposure[name] = stamp
            if (not isinstance(candidate, dict) or candidate.get("diagnostic_candidate") is not True
                    or candidate.get("collision_model_approved") is not False):
                raise ValueError("explicit diagnostic candidate required")
            position = np.asarray(candidate.get("box_center_candidate_m"), dtype=float)
            normal = np.asarray(candidate.get("normal_base"), dtype=float)
            rotation = np.asarray(candidate.get("rotation_base"), dtype=float)
            yaw = _finite_number(candidate.get("yaw_mod_quarter_turn_rad"), "square yaw")
            if (position.shape != (3,) or not np.all(np.isfinite(position))
                    or normal.shape != (3,) or not np.all(np.isfinite(normal))
                    or abs(np.linalg.norm(normal)-1) > 1e-6 or normal[2] <= 0
                    or rotation.shape != (3, 3) or not np.all(np.isfinite(rotation))
                    or not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-6, rtol=0)
                    or abs(np.linalg.det(rotation)-1) > 1e-6
                    or not np.allclose(rotation[:, 2], normal, atol=1e-6, rtol=0)):
                raise ValueError("invalid measured position/normal/rotation")
            basis_x = np.array([1., 0, 0])-normal[0]*normal
            basis_x /= np.linalg.norm(basis_x)
            basis_y = np.cross(normal, basis_x)
            observed_yaw = np.arctan2(rotation[:, 0] @ basis_y, rotation[:, 0] @ basis_x)
            delta = (observed_yaw-yaw+np.pi/4) % (np.pi/2)-np.pi/4
            if abs(delta) > 1e-6:
                raise ValueError("rotation and square yaw disagree")
            samples = self.samples[name]
            if samples and received < samples[-1][0]:
                raise ValueError("receipt clock went backwards")
            samples = self._prune(name, received)
            samples.append((received, stamp, position.copy(), normal.copy(), yaw, age))
            return True
        except (ValueError, TypeError):
            self.invalidate(name)
            raise

    def snapshot(self, *, now_monotonic):
        now = _finite_number(now_monotonic, "snapshot time")
        result = {}
        for name in self.samples:
            samples = self.samples[name]
            if samples and now < samples[-1][0]:
                raise ValueError("snapshot receipt clock went backwards")
            samples = self._prune(name, now)
            if len(samples) < self.minimum:
                continue
            positions = np.array([s[2] for s in samples])
            normals = np.array([s[3] for s in samples])
            angles = np.array([s[4] for s in samples])
            phase = np.mean(np.exp(4j*angles))
            if abs(phase) < 1e-9:
                continue
            yaw = float((np.angle(phase)/4) % (np.pi/2))
            spread = np.abs((angles-yaw+np.pi/4) % (np.pi/2)-np.pi/4)
            normal = normals.mean(axis=0)
            normal /= np.linalg.norm(normal)
            normal_spread = np.arccos(np.clip(normals @ normal, -1, 1))
            std = positions.std(axis=0)
            if (np.any(std > self.position_limit) or np.any(spread > self.yaw_limit)
                    or np.any(normal_spread > self.normal_limit)):
                continue
            x = np.array([1., 0, 0])-normal[0]*normal
            x /= np.linalg.norm(x)
            y = np.cross(normal, x)
            rotation = np.column_stack((np.cos(yaw)*x+np.sin(yaw)*y,
                                        -np.sin(yaw)*x+np.cos(yaw)*y, normal))
            result[name] = dict(
                position=np.median(positions, axis=0).tolist(), rotation_base=rotation.tolist(),
                yaw_mod_quarter_turn_rad=yaw, normal_base=normal.tolist(),
                source="rgbd_top_plane_square_boundary", frame="base_footprint", unit="m",
                exposure_stamp=samples[-1][1], stamp_clock=self.identity[0],
                calibration_revision=self.identity[1], receipt_age_s=now-samples[-1][0],
                exposure_age_s=samples[-1][5]+now-samples[-1][0],
                sample_count=len(samples), position_std_m=std.tolist(),
                max_yaw_spread_rad=float(spread.max()), max_normal_spread_rad=float(normal_spread.max()),
                diagnostic_only=True, collision_model_approved=False, uncertainty="not_certified")
        return result
