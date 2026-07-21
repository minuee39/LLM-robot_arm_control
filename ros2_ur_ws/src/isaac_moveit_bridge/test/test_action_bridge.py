from isaac_moveit_bridge.action_bridge import (
    isaac_gripper_position_to_moveit,
    isaac_gripper_velocity_to_moveit,
    gripper_motion_complete,
    moveit_gripper_position_to_isaac,
)


def test_moveit_gripper_command_stays_in_radians():
    assert moveit_gripper_position_to_isaac(0.0) == 0.0
    assert moveit_gripper_position_to_isaac(0.7) == 0.7
    assert moveit_gripper_position_to_isaac(0.45) == 0.45


def test_gripper_position_mapping_clamps_out_of_range_values():
    assert moveit_gripper_position_to_isaac(-1.0) == 0.0
    assert moveit_gripper_position_to_isaac(1.0) == 0.7
    assert isaac_gripper_position_to_moveit(-10.0) == 0.0
    assert isaac_gripper_position_to_moveit(1.0) == 0.7


def test_isaac_gripper_feedback_is_already_in_radians():
    assert isaac_gripper_position_to_moveit(0.0) == 0.0
    assert isaac_gripper_position_to_moveit(0.7) == 0.7
    assert isaac_gripper_position_to_moveit(0.45) == 0.45
    assert isaac_gripper_velocity_to_moveit(0.4) == 0.4


def test_gripper_motion_complete_when_measured_position_reaches_target():
    complete, stalled = gripper_motion_complete(0.01, 0.495, 0.5, 0.02, 0.0)

    assert complete is True
    assert stalled is False


def test_gripper_close_accepts_near_target_contact_position():
    complete, stalled = gripper_motion_complete(0.01, 0.482, 0.5, 0.04, 0.0)

    assert complete is True
    assert stalled is False


def test_gripper_close_accepts_lower_edge_of_block_contact_range():
    complete, stalled = gripper_motion_complete(0.01, 0.4777, 0.5, 0.08, 0.0)

    assert complete is True
    assert stalled is False


def test_gripper_close_rejects_position_before_block_contact_range():
    complete, stalled = gripper_motion_complete(0.01, 0.465, 0.5, 0.08, 0.0)

    assert complete is False
    assert stalled is False


def test_gripper_motion_complete_on_stable_block_contact():
    complete, stalled = gripper_motion_complete(0.01, 0.475, 0.5, 0.0, 0.3)

    assert complete is True
    assert stalled is False


def test_gripper_motion_does_not_complete_before_stall_hold_time():
    complete, stalled = gripper_motion_complete(0.01, 0.2, 0.5, 0.0, 0.1)

    assert complete is False
    assert stalled is False
