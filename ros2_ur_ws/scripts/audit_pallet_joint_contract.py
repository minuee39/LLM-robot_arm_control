#!/usr/bin/env python3
"""Read-only consistency audit for the active Pallet simulation joint contract."""

from __future__ import annotations

import math
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import yaml


SRC = Path(__file__).resolve().parents[1] / "src"
DESCRIPTION = SRC / "pallet_gripper" / "urdf" / "pallet_gripper.urdf"
MOVEIT = SRC / "pallet_gripper_moveit_config" / "config"
PROFILES = SRC / "mtc_tutorial" / "config" / "robot_profiles.yaml"
LEGACY_HARDWARE = SRC / "pallet" / "config" / "robstride_hardware.yaml"
JAW_TRAVEL_M = 0.0335


def read_yaml(path: Path) -> dict:
    with path.open(encoding="utf-8") as stream:
        return yaml.safe_load(stream)


def main() -> int:
    errors: list[str] = []
    robot = ET.parse(DESCRIPTION).getroot()
    joints = {joint.attrib["name"]: joint for joint in robot.findall("joint")}
    arm = [f"J{number}" for number in range(1, 7)]
    gripper = ["J8"]
    actuated = arm + gripper

    for name in actuated:
        joint = joints.get(name)
        if joint is None or joint.get("type") not in ("revolute", "continuous", "prismatic"):
            errors.append(f"URDF: {name} is missing or not actuated")
    for name in ("J8", "J9"):
        joint = joints.get(name)
        if joint is None or joint.get("type") != "prismatic":
            errors.append(f"URDF: {name} must be prismatic")
        limit = joint.find("limit") if joint is not None else None
        if limit is None or any(
            not math.isclose(float(limit.get(key, "nan")), expected, rel_tol=0.0, abs_tol=1e-9)
            for key, expected in (("lower", 0.0), ("upper", JAW_TRAVEL_M))
        ):
            errors.append(f"URDF: {name} must travel 0..{JAW_TRAVEL_M} m (67 mm total opening)")
    mimic = joints.get("J9")
    mimic_tag = mimic.find("mimic") if mimic is not None else None
    if (mimic_tag is None or mimic_tag.get("joint") != "J8" or
            float(mimic_tag.get("multiplier", "1")) != 1.0 or
            float(mimic_tag.get("offset", "0")) != 0.0):
        errors.append("URDF: J9 must mimic J8 with multiplier 1 and offset 0")

    srdf = ET.parse(MOVEIT / "pallet_gripper.srdf").getroot()
    groups = {group.get("name"): group for group in srdf.findall("group")}
    arm_group = groups.get("pallet_arm")
    chain = arm_group.find("chain") if arm_group is not None else None
    if chain is None or chain.get("base_link") != "base_link" or chain.get("tip_link") != "tcp":
        errors.append("SRDF: pallet_arm must use base_link -> tcp")
    hand_group = groups.get("gripper")
    hand_joint = hand_group.find("joint") if hand_group is not None else None
    if hand_joint is None or hand_joint.get("name") != "J8":
        errors.append("SRDF: gripper must use J8")
    open_joint = srdf.find("group_state[@name='open'][@group='gripper']/joint[@name='J8']")
    if open_joint is None or not math.isclose(
        float(open_joint.get("value", "nan")), JAW_TRAVEL_M, rel_tol=0.0, abs_tol=1e-9
    ):
        errors.append(f"SRDF: gripper open state must use J8={JAW_TRAVEL_M} m")
    if not any(
        {pair.get("link1"), pair.get("link2")} == {"J8", "J9"}
        for pair in srdf.findall("disable_collisions")
    ):
        errors.append("SRDF: finger-to-finger collision J8/J9 must be disabled")

    ros_controllers = read_yaml(MOVEIT / "ros2_controllers.yaml")
    moveit_controllers = read_yaml(MOVEIT / "moveit_controllers.yaml")
    moveit_entries = moveit_controllers["moveit_simple_controller_manager"]
    actual_sets = {
        "ros2_control arm": ros_controllers["pallet_arm_controller"]["ros__parameters"]["joints"],
        "MoveIt arm": moveit_entries["pallet_arm_controller"]["joints"],
        "MoveIt gripper": moveit_entries["gripper_controller"]["joints"],
    }
    for label in ("ros2_control arm", "MoveIt arm"):
        if actual_sets[label] != arm:
            errors.append(f"{label}: expected {arm}, got {actual_sets[label]}")
    control_description = ET.parse(MOVEIT / "pallet_gripper.ros2_control.xacro").getroot()
    control_joints = [joint.get("name") for joint in control_description.findall(".//ros2_control/joint")]
    if control_joints != actuated:
        errors.append(f"ros2_control description: expected {actuated}, got {control_joints}")
    if ros_controllers["gripper_controller"]["ros__parameters"]["joint"] != "J8":
        errors.append("ros2_control gripper: expected J8")
    if actual_sets["MoveIt gripper"] != gripper:
        errors.append(f"MoveIt gripper: expected {gripper}")

    initial = read_yaml(MOVEIT / "initial_positions.yaml")["initial_positions"]
    if set(initial) != set(actuated):
        errors.append(f"initial positions: expected {actuated}, got {list(initial)}")
    if not math.isclose(float(initial.get("J8", "nan")), JAW_TRAVEL_M, rel_tol=0.0, abs_tol=1e-9):
        errors.append(f"initial positions: J8 must start open at {JAW_TRAVEL_M} m")
    limits = read_yaml(MOVEIT / "joint_limits.yaml")["joint_limits"]
    if set(limits) != set(actuated):
        errors.append(f"MoveIt limits: expected {actuated}, got {list(limits)}")
    else:
        urdf_hand_limit = joints["J8"].find("limit") if "J8" in joints else None
        if urdf_hand_limit is None or any(
            abs(float(urdf_hand_limit.get(key)) - float(limits["J8"][moveit_key])) > 1e-9
            for key, moveit_key in (("lower", "min_position"), ("upper", "max_position"))
        ):
            errors.append("J8 position limits differ between URDF and MoveIt")

    profile = read_yaml(PROFILES)["profiles"]["pallet_gripper"]
    params = profile["mtc_parameters"]
    expected_profile = {
        "arm_group_name": "pallet_arm",
        "hand_group_name": "gripper",
        "hand_frame": "tcp",
        "gripper_joint_name": "J8",
        "arm_controller_name": "pallet_arm_controller",
        "gripper_controller_name": "gripper_controller",
    }
    for key, expected in expected_profile.items():
        if params.get(key) != expected:
            errors.append(f"MTC {key}: expected {expected}, got {params.get(key)}")

    hardware = read_yaml(LEGACY_HARDWARE)
    legacy_joints = list(hardware["joints"])
    print("Active Pallet model: J1 J2 J3 J4 J5 J6; gripper J8; J9 mimics J8")
    print(f"Legacy CAN bridge: {', '.join(legacy_joints)}; commissioned={hardware.get('commissioned')}")
    if legacy_joints != actuated:
        print("HARDWARE GAP: legacy five-axis CAN configuration does not match the active seven-command-joint model")
    if errors:
        for error in errors:
            print(f"ERROR: {error}", file=sys.stderr)
        return 1
    print("PASS: active URDF, SRDF, controllers, initial state, limits, and MTC profile agree")
    return 0


if __name__ == "__main__":
    sys.exit(main())
