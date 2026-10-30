import numpy as np
import pytest

from vision.box_pose_store import StableBoxPoseStore


def store():
    return StableBoxPoseStore(window_size=5, min_samples=3, ttl_s=1.,
                              max_position_std_m=.001, max_yaw_spread_rad=.05,
                              max_normal_spread_rad=.05)


def candidate(yaw=0., x=.24):
    return dict(diagnostic_candidate=True, collision_model_approved=False,
                box_center_candidate_m=[x, -.28, .025], normal_base=[0, 0, 1],
                rotation_base=[[np.cos(yaw), -np.sin(yaw), 0],
                               [np.sin(yaw), np.cos(yaw), 0], [0, 0, 1]],
                yaw_mod_quarter_turn_rad=yaw)


def add(s, i, c=None, **overrides):
    args = dict(exposure_stamp=10+i*.1, clock="sim", calibration_revision="a",
                received_at_monotonic=1+i*.1, exposure_age_s=.01, frame="base_footprint", unit="m")
    args.update(overrides)
    return s.update("red_block", candidate() if c is None else c, **args)


def test_joint_square_symmetry_and_clock_contract():
    s = store()
    for i, yaw in enumerate([.01, np.pi/2-.01, np.pi+.005]):
        add(s, i, candidate(yaw))
    info = s.snapshot(now_monotonic=1.3)["red_block"]
    assert info["sample_count"] == 3
    assert info["max_yaw_spread_rad"] < .02
    assert info["stamp_clock"] == "sim" and info["exposure_stamp"] == 10.2
    assert info["collision_model_approved"] is False


def test_duplicate_does_not_extend_receipt_ttl():
    s = store()
    for i in range(3): add(s, i)
    assert not add(s, 2, received_at_monotonic=1.9)
    assert s.snapshot(now_monotonic=2.3) == {}


def test_failure_reverse_exposure_and_calibration_clear_joint_pose():
    for mode in ("invalid", "reverse", "calibration", "explicit"):
        s = store()
        for i in range(3): add(s, i)
        if mode == "invalid":
            with pytest.raises(ValueError): add(s, 3, {"diagnostic_candidate": False})
        elif mode == "reverse": assert not add(s, 1)
        elif mode == "calibration": add(s, 3, calibration_revision="b")
        else: s.invalidate("red_block")
        assert s.snapshot(now_monotonic=1.4) == {}


def test_unstable_joint_samples_and_stale_exposure_rejected():
    s = store()
    for i in range(3): add(s, i, candidate(x=.24+i*.01))
    assert s.snapshot(now_monotonic=1.3) == {}
    with pytest.raises(ValueError, match="stale"):
        add(s, 3, exposure_age_s=2)


def test_rotation_must_match_measured_direction_and_not_be_approved():
    s = store()
    c = candidate()
    c["yaw_mod_quarter_turn_rad"] = .1
    with pytest.raises(ValueError, match="disagree"): add(s, 0, c)
    c = candidate()
    c["collision_model_approved"] = True
    with pytest.raises(ValueError): add(s, 1, c)


def test_delayed_exposures_do_not_get_new_full_ttl():
    s = store()
    for i in range(3): add(s, i, exposure_age_s=.8)
    assert s.snapshot(now_monotonic=1.21) == {}


def test_invalid_newer_direction_does_not_allow_older_exposure_resurrection():
    s = store()
    for i in range(3): add(s, i)
    with pytest.raises(ValueError): add(s, 3, {"diagnostic_candidate": False})
    assert not add(s, 2.5)
    assert s.snapshot(now_monotonic=1.4) == {}


def test_unstable_yaw_is_not_hidden_by_position_stability():
    s = store()
    for i, yaw in enumerate([-.2, 0, .2]): add(s, i, candidate(yaw))
    assert s.snapshot(now_monotonic=1.3) == {}


def test_clock_change_resets_all_objects_and_wrong_units_clear_pose():
    s = store()
    for i in range(3): add(s, i)
    add(s, 3, clock="new_epoch")
    assert s.snapshot(now_monotonic=1.4) == {}
    with pytest.raises(ValueError, match="metres"): add(s, 4, unit="mm")
    assert s.snapshot(now_monotonic=1.5) == {}
