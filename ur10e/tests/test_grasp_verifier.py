import json

import numpy as np

from grasp_verifier import GraspSample, evaluate_grasp_samples, read_grasp_sample


def sample(object_position, gripper_position):
    return GraspSample(
        object_position=np.asarray(object_position, dtype=float),
        gripper_position=np.asarray(gripper_position, dtype=float),
    )


def test_read_grasp_sample_reads_object_and_gripper_pose(tmp_path):
    scene_state = tmp_path / "scene.json"
    scene_state.write_text(
        json.dumps(
            {
                "objects": [
                    {"name": "red_block", "position": [0.1, 0.2, 0.05]},
                    {"name": "blue_block", "position": [0.3, 0.2, 0.05]},
                ],
                "gripper": {"position": [0.1, 0.2, 0.20]},
            }
        ),
        encoding="utf-8",
    )

    grasp_sample = read_grasp_sample(scene_state, "red_block")

    np.testing.assert_allclose(grasp_sample.object_position, [0.1, 0.2, 0.05])
    np.testing.assert_allclose(grasp_sample.gripper_position, [0.1, 0.2, 0.20])


def test_evaluate_grasp_samples_accepts_stable_lift():
    samples = [
        sample([0.0, 0.0, 0.05], [0.0, 0.0, 0.20]),
        sample([0.0, 0.0, 0.09], [0.0, 0.0, 0.24]),
        sample([0.0, 0.0, 0.14], [0.0, 0.0, 0.29]),
    ]

    result = evaluate_grasp_samples(samples, [0.0, 0.0, 0.05])

    assert result.success is True
    assert result.lifted_sample_count == 2
    assert np.isclose(result.max_lift, 0.09)
    assert np.isclose(result.max_relative_deviation, 0.0)


def test_evaluate_grasp_samples_rejects_object_that_never_lifts():
    samples = [
        sample([0.0, 0.0, 0.05], [0.0, 0.0, 0.20]),
        sample([0.0, 0.0, 0.051], [0.0, 0.0, 0.30]),
        sample([0.0, 0.0, 0.049], [0.0, 0.0, 0.40]),
    ]

    result = evaluate_grasp_samples(samples, [0.0, 0.0, 0.05])

    assert result.success is False
    assert "did not remain lifted" in result.reason


def test_evaluate_grasp_samples_rejects_unstable_relative_pose():
    samples = [
        sample([0.0, 0.0, 0.09], [0.0, 0.0, 0.24]),
        sample([0.0, 0.0, 0.14], [0.15, 0.0, 0.29]),
    ]

    result = evaluate_grasp_samples(
        samples,
        [0.0, 0.0, 0.05],
        relative_tolerance=0.03,
    )

    assert result.success is False
    assert "stable carried interval" in result.reason


def test_evaluate_grasp_samples_accepts_carry_then_elevated_release():
    samples = [
        sample([0.0, 0.0, 0.05], [0.0, 0.0, 0.20]),
        sample([0.0, 0.0, 0.09], [0.0, 0.0, 0.24]),
        sample([0.0, 0.0, 0.14], [0.0, 0.0, 0.29]),
        sample([0.2, 0.0, 0.15], [0.2, 0.0, 0.30]),
        # The object remains on top of another block while the gripper
        # retreats.  These release samples must not invalidate the carry.
        sample([0.2, 0.0, 0.15], [0.2, 0.0, 0.45]),
        sample([0.2, 0.0, 0.15], [0.4, 0.0, 0.60]),
    ]

    result = evaluate_grasp_samples(samples, [0.0, 0.0, 0.05])

    assert result.success is True
    assert result.lifted_sample_count >= 2
    assert result.max_relative_deviation <= 0.03


def test_evaluate_grasp_samples_rejects_stationary_elevated_object_after_release():
    samples = [
        sample([0.0, 0.0, 0.05], [0.0, 0.0, 0.20]),
        sample([0.2, 0.0, 0.15], [0.2, 0.0, 0.45]),
        sample([0.2, 0.0, 0.15], [0.2, 0.0, 0.45]),
        sample([0.2, 0.0, 0.15], [0.2, 0.0, 0.45]),
    ]

    result = evaluate_grasp_samples(samples, [0.0, 0.0, 0.05])

    assert result.success is False
