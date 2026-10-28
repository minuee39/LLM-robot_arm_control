from copy import deepcopy
from dataclasses import FrozenInstanceError
import json

import pytest

from vision.box_pose_reader import DiagnosticPoseReader
from vision.scene_objects import EXPECTED_BLOCK_NAMES


def payload():
    item = dict(position=[.24, -.28, .025], normal_base=[0., 0., 1.],
                rotation_base=[[1., 0., 0.], [0., 1., 0.], [0., 0., 1.]],
                yaw_mod_quarter_turn_rad=0., exposure_stamp=99.8,
                stamp_clock='unix_system', calibration_revision='revision',
                frame='base_footprint', unit='m', source='rgbd_top_plane_square_boundary',
                diagnostic_only=True, collision_model_approved=False, uncertainty='not_certified',
                sample_count=3, position_std_m=[0., 0., 0.],
                max_yaw_spread_rad=0., max_normal_spread_rad=0.,
                exposure_age_s=0., receipt_age_s=0.)
    return dict(diagnostic_only=True, collision_model_approved=False,
                updated_at_unix=99.9, objects={name: deepcopy(item) for name in EXPECTED_BLOCK_NAMES})


def reader():
    return DiagnosticPoseReader(expected_revision='revision', max_age_s=1.)


def test_fresh_snapshot_is_immutable_and_not_an_approved_obstacle(tmp_path):
    p = payload()
    p['objects']['red_block']['bound_source'] = 'fake approval'
    path = tmp_path/'latest.json'
    path.write_text(json.dumps(p))
    result = reader().read(path, now_unix=100.)
    assert result.diagnostic_only and result.collision_model_approved is False
    assert len(result.objects) == 3
    pose = result.objects[0]
    assert pose.exposure_age_s == pytest.approx(.2)  # ignores fake serialized age=0
    assert pose.uncertainty == 'not_certified' and pose.collision_model_approved is False
    assert not hasattr(pose, 'bound_source')
    p['objects'][pose.object_id]['position'][0] = 100
    assert pose.position_m[0] == .24
    with pytest.raises(FrozenInstanceError):
        pose.collision_model_approved = True
    with pytest.raises(TypeError):
        pose.rotation_base[0][0] = 5


def test_duplicate_json_fields_cannot_silently_replace_metadata(tmp_path):
    path = tmp_path/'duplicate.json'
    raw = json.dumps(payload()).replace('"updated_at_unix": 99.9',
                                        '"updated_at_unix": 0, "updated_at_unix": 99.9')
    path.write_text(raw)
    with pytest.raises(ValueError, match='duplicate JSON'):
        reader().read(path, now_unix=100.)


@pytest.mark.parametrize('field,value', [
    ('exposure_stamp', 98.), ('exposure_stamp', 100.1), ('exposure_stamp', True),
    ('exposure_stamp', float('nan')), ('exposure_stamp', '99.8'),
    ('stamp_clock', 'simulation'), ('frame', 'world'), ('unit', 'mm'),
    ('calibration_revision', 'old'), ('source', 'planned/remembered'),
    ('diagnostic_only', False), ('diagnostic_only', 1),
    ('collision_model_approved', True), ('collision_model_approved', 0),
    ('uncertainty', 'certified'), ('sample_count', True), ('sample_count', 2),
    ('position', [True, 0., 0.]), ('position', [float('inf'), 0., 0.]),
    ('position', [0., 0.]), ('normal_base', [0., 0., -1.]),
    ('normal_base', [0., 0., .5]), ('rotation_base', [[1., 0., 0.], [0., 2., 0.], [0., 0., 1.]]),
    ('rotation_base', [[1., 0., 0.], [0., -1., 0.], [0., 0., 1.]]),
    ('normal_base', [.01, 0., .99994999875]), ('yaw_mod_quarter_turn_rad', .1),
    ('max_yaw_spread_rad', -.01), ('position_std_m', [0., -.01, 0.]),
])
def test_invalid_item_rejects_entire_bundle(field, value):
    p = payload()
    p['objects']['green_block'][field] = value
    with pytest.raises(ValueError):
        reader().parse(p, now_unix=100.)


@pytest.mark.parametrize('change', [
    lambda p: p.update(updated_at_unix=98.),
    lambda p: p.update(updated_at_unix=100.1),
    lambda p: p.update(updated_at_unix=True),
    lambda p: p.update(diagnostic_only=1),
    lambda p: p.update(collision_model_approved=0),
    lambda p: p['objects'].pop('green_block'),
    lambda p: p['objects'].update(unknown={}),
    lambda p: p.update(objects=[]),
])
def test_invalid_root_and_incomplete_scene_are_rejected(change):
    p = payload()
    change(p)
    with pytest.raises(ValueError):
        reader().parse(p, now_unix=100.)


def test_repeated_write_does_not_refresh_old_exposure_or_watermark():
    r = reader()
    r.parse(payload(), now_unix=100.)
    p = payload()
    p['updated_at_unix'] = 100.2
    with pytest.raises(ValueError, match='duplicate'):
        r.parse(p, now_unix=100.3)
    for item in p['objects'].values():
        item['exposure_stamp'] = 100.1
    assert r.parse(p, now_unix=100.3).objects[0].exposure_stamp == 100.1
    p['objects']['red_block']['exposure_stamp'] = 100.
    p['updated_at_unix'] = 100.4
    with pytest.raises(ValueError, match='reverse'):
        r.parse(p, now_unix=100.5)


def test_failed_read_cannot_return_or_mix_cached_pose():
    r = reader()
    r.parse(payload(), now_unix=100.)
    p = payload()
    p['updated_at_unix'] = 100.2
    for item in p['objects'].values():
        item['exposure_stamp'] = 100.1
    p['objects']['green_block']['yaw_mod_quarter_turn_rad'] = .1
    with pytest.raises(ValueError):
        r.parse(p, now_unix=100.3)
    p['objects']['green_block']['yaw_mod_quarter_turn_rad'] = 0.
    assert r.parse(p, now_unix=100.3).objects[0].exposure_stamp == 100.1


def test_write_reversal_rejected_even_with_new_exposures():
    r = reader()
    r.parse(payload(), now_unix=100.)
    p = payload()
    p['updated_at_unix'] = 99.85
    for item in p['objects'].values():
        item['exposure_stamp'] = 99.81
    with pytest.raises(ValueError, match='write time'):
        r.parse(p, now_unix=100.)


@pytest.mark.parametrize('revision,age', [('', 1.), ('revision', 0), ('revision', 1.1),
                                        ('revision', True), ('revision', float('nan'))])
def test_invalid_consumer_contract_is_rejected(revision, age):
    with pytest.raises(ValueError):
        DiagnosticPoseReader(expected_revision=revision, max_age_s=age)
