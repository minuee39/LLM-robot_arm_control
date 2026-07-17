import numpy as np

from grasp_policy import FixedGraspPolicy, GraspRequest


def test_fixed_grasp_policy_returns_existing_offset():
    request = GraspRequest(
        object_name="red_block",
        object_pose=np.array([-0.3, 0.3, 0.02575]),
        target_position=np.array([0.0, 0.45, 0.2]),
    )
    policy = FixedGraspPolicy()

    action = policy.predict(request)

    np.testing.assert_allclose(action.end_effector_offset, np.array([0.0, 0.0, 0.20]))
    assert action.confidence == 1.0
    assert action.metadata["policy"] == "FixedGraspPolicy"
