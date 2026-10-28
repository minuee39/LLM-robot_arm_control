from dataclasses import replace

import numpy as np
import pytest

from vision.box_bound_policy import ConditionalBoxPolicy, diagnostic_box_envelope
from vision.box_pose_reader import DiagnosticPoseReader
from test_box_pose_reader import payload


def policy(**changes):
    values = dict(measurement_revision='revision', nominal_size_m=(.03, .03, .05),
                  position_error_xyz_m=(.001, .002, .0005), rotation_error_rad=.01,
                  size_error_xyz_m=(.0001, .0001, .0001), valid_from_unix=99.,
                  valid_until_unix=101., position_min_m=(-1., -1., 0.),
                  position_max_m=(1., 1., 1.), bound_source='synthetic test only',
                  assumptions=('synthetic bounded errors; not adopted for camera',))
    values.update(changes)
    return ConditionalBoxPolicy(**values)


def pose():
    return DiagnosticPoseReader(expected_revision='revision', max_age_s=1.).parse(
        payload(), now_unix=100.).objects[0]


def test_explicit_policy_stays_conditional_and_does_not_use_variance():
    p = pose()
    out = diagnostic_box_envelope(p, policy(), now_unix=100., max_age_s=1.)
    assert out.diagnostic_only and not out.collision_model_approved
    assert out.uncertainty == 'conditional_assumptions_not_certified'
    assert np.all(out.geometry.size_m > policy().nominal_size_m)
    zero_std = replace(p, position_std_m=(0., 0., 0.), max_yaw_spread_rad=0.)
    high_std = replace(p, position_std_m=(.1, .1, .1), max_yaw_spread_rad=.1)
    first = diagnostic_box_envelope(zero_std, policy(), now_unix=100., max_age_s=1.)
    second = diagnostic_box_envelope(high_std, policy(), now_unix=100., max_age_s=1.)
    assert np.array_equal(first.geometry.size_m, second.geometry.size_m)


@pytest.mark.parametrize('changes', [
    dict(bound_source=''), dict(assumptions=()), dict(assumptions='guaranteed'),
    dict(measurement_revision=''), dict(position_error_xyz_m=(True, 0, 0)),
    dict(position_error_xyz_m=(-.001, 0, 0)), dict(rotation_error_rad=True),
    dict(rotation_error_rad=float('nan')), dict(rotation_error_rad=4),
    dict(nominal_size_m=(.03, 0, .05)), dict(size_error_xyz_m=(0, 0, float('inf'))),
    dict(valid_until_unix=98.), dict(position_max_m=(-1, -1, 0)),
])
def test_invalid_or_implicit_policy_is_rejected(changes):
    with pytest.raises(ValueError):
        policy(**changes)


@pytest.mark.parametrize('changes,now', [
    (dict(measurement_revision='other'), 100.),
    (dict(valid_from_unix=99.9), 100.),
    (dict(valid_until_unix=100.1), 100.2),
    (dict(position_min_m=(.2395, -1, 0)), 100.),
    ({}, 101.01), ({}, 99.7),
])
def test_scope_revision_and_freshness_are_checked_at_use(changes, now):
    with pytest.raises(ValueError):
        diagnostic_box_envelope(pose(), policy(**changes), now_unix=now, max_age_s=1.)


def test_fake_approval_and_missing_policy_are_rejected():
    with pytest.raises(ValueError):
        diagnostic_box_envelope(replace(pose(), collision_model_approved=True), policy(),
                                now_unix=100., max_age_s=1.)
    with pytest.raises(ValueError):
        diagnostic_box_envelope(pose(), None, now_unix=100., max_age_s=1.)


@pytest.mark.parametrize('changes', [dict(normal_base=(0., 0., -1.)),
                                     dict(yaw_mod_quarter_turn_rad=.1),
                                     dict(source='planned/remembered')])
def test_modified_bundle_is_revalidated_at_envelope_use(changes):
    with pytest.raises(ValueError):
        diagnostic_box_envelope(replace(pose(), **changes), policy(), now_unix=100., max_age_s=1.)
