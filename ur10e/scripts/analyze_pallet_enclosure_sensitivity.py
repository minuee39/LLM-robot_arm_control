#!/usr/bin/env python3
"""Diagnostic sensitivity only: assumed error bounds are never certified."""
import argparse
from itertools import combinations, product
import json
from pathlib import Path
import sys
import time

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from vision.box_bound_policy import ConditionalBoxPolicy, diagnostic_box_envelope
from vision.box_enclosure import enclosure_axis_gap, enclosures_separated
from vision.box_pose_reader import DiagnosticPoseReader, _array, _number


def analyze(snapshot, *, nominal_size_m, position_errors_m, rotation_errors_rad,
            size_error_m, numerical_margin_m, position_min_m, position_max_m):
    size = tuple(map(float, _array(nominal_size_m, (3,), 'nominal size')))
    dimension_error = _number(size_error_m, 'dimension error')
    margin = _number(numerical_margin_m, 'numerical margin')
    positions = [_number(v, 'position error') for v in position_errors_m]
    rotations = [_number(v, 'rotation error') for v in rotation_errors_rad]
    if (dimension_error < 0 or margin < 0 or not positions or not rotations
            or len(positions)*len(rotations) > 400 or any(v < 0 for v in positions)
            or any(not 0 <= v <= np.pi for v in rotations)):
        raise ValueError('nonnegative explicit error grid of at most 400 combinations required')
    pairs = list(combinations(snapshot.objects, 2))
    sphere_radius = float(np.linalg.norm(size)/2)
    sphere_pairs = [{
        'objects': [a.object_id, b.object_id],
        'nominal_sphere_signed_gap_m': float(np.linalg.norm(np.array(a.position_m)-b.position_m)-2*sphere_radius),
    } for a, b in pairs]
    rows = []
    for error, angle in product(positions, rotations):
        policy = ConditionalBoxPolicy(
            measurement_revision=snapshot.objects[0].calibration_revision,
            nominal_size_m=size, position_error_xyz_m=(error,)*3,
            rotation_error_rad=angle, size_error_xyz_m=(dimension_error,)*3,
            valid_from_unix=min(p.exposure_stamp for p in snapshot.objects),
            valid_until_unix=snapshot.validated_at_unix+1.,
            position_min_m=position_min_m, position_max_m=position_max_m,
            bound_source='sensitivity_only_not_certification',
            assumptions=('User-specified hypothetical error intervals, not observed/certified bounds.',
                         'No gripper/path collision or current motion authorization is implied.'),
        )
        envelopes = {p.object_id: diagnostic_box_envelope(
            p, policy, now_unix=snapshot.validated_at_unix, max_age_s=1.).geometry
            for p in snapshot.objects}
        pair_results = []
        for a, b in pairs:
            first, second = envelopes[a.object_id], envelopes[b.object_id]
            pair_results.append(dict(objects=[a.object_id, b.object_id],
                                     sat_axis_gap_m=enclosure_axis_gap(first, second),
                                     separated=enclosures_separated(first, second, numerical_margin_m=margin)))
        rows.append(dict(position_error_each_xyz_m=error, full_rotation_error_rad=angle,
                         size_error_each_dimension_m=dimension_error,
                         all_pairs_separated=all(p['separated'] for p in pair_results),
                         minimum_pair_sat_axis_gap_m=min(p['sat_axis_gap_m'] for p in pair_results),
                         pairs=pair_results))
    return dict(diagnostic_only=True, collision_model_approved=False,
                uncertainty='hypothetical_sensitivity_not_certified',
                measurement_revision=snapshot.objects[0].calibration_revision,
                evaluated_at_unix=snapshot.validated_at_unix,
                nominal_size_m=list(size), numerical_margin_m=margin,
                nominal_unknown_direction_spheres=sphere_pairs, rows=rows,
                caution='SAT gap is projected separation, not Euclidean gripper/path clearance. No bounds are adopted.')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('snapshot', type=Path)
    parser.add_argument('--expected-revision', required=True)
    parser.add_argument('--nominal-size-m', type=float, nargs=3, required=True)
    parser.add_argument('--position-error-m', type=float, action='append', required=True)
    parser.add_argument('--rotation-error-rad', type=float, action='append', required=True)
    parser.add_argument('--size-error-m', type=float, required=True)
    parser.add_argument('--numerical-margin-m', type=float, required=True)
    parser.add_argument('--position-min-m', type=float, nargs=3, required=True)
    parser.add_argument('--position-max-m', type=float, nargs=3, required=True)
    parser.add_argument('--replay-recorded-time', action='store_true')
    args = parser.parse_args()
    # Default current-time reader rejects historical files; replay is explicit.
    payload = json.loads(args.snapshot.read_text())
    now = payload.get('updated_at_unix') if args.replay_recorded_time else time.time()
    snapshot = DiagnosticPoseReader(expected_revision=args.expected_revision, max_age_s=1.).read(
        args.snapshot, now_unix=now)
    result = analyze(snapshot, nominal_size_m=args.nominal_size_m,
                     position_errors_m=args.position_error_m, rotation_errors_rad=args.rotation_error_rad,
                     size_error_m=args.size_error_m, numerical_margin_m=args.numerical_margin_m,
                     position_min_m=args.position_min_m, position_max_m=args.position_max_m)
    result['time_validation'] = ('recorded_timestamp_replay_not_current_freshness'
                                 if args.replay_recorded_time else 'current_unix_exposure_validation')
    print(json.dumps(result, indent=2, allow_nan=False))


if __name__ == '__main__':
    main()
