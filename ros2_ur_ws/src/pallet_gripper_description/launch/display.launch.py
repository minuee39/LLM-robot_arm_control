"""Display the final CAD export in the MoveIt SRDF home configuration."""

from pathlib import Path
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import LogInfo
from launch_ros.actions import Node


def load_home_positions(robot):
    moveit_package = Path(
        get_package_share_directory("pallet_gripper_moveit_config")
    )
    srdf = ET.parse(
        moveit_package / "config/pallet_gripper.srdf"
    ).getroot()
    home = next(
        (state for state in srdf.findall("group_state")
         if state.get("name") == "home"),
        None,
    )
    if home is None:
        raise RuntimeError("SRDF group state 'home' was not found")

    urdf_joints = {joint.get("name"): joint for joint in robot.findall("joint")}
    positions = {}
    for joint_value in home.findall("joint"):
        name = joint_value.get("name")
        if name not in urdf_joints:
            raise RuntimeError(f"SRDF home references unknown URDF joint: {name}")
        value = float(joint_value.get("value"))
        limit = urdf_joints[name].find("limit")
        if limit is not None:
            lower = float(limit.get("lower"))
            upper = float(limit.get("upper"))
            if not lower <= value <= upper:
                raise RuntimeError(
                    f"SRDF home value for {name} ({value}) is outside "
                    f"the URDF limits [{lower}, {upper}]"
                )
        positions[f"zeros.{name}"] = value
    return positions


def generate_launch_description():
    package = Path(get_package_share_directory("pallet_gripper_description"))
    robot = ET.parse(package / "urdf/pallet_gripper.urdf").getroot()
    home_positions = load_home_positions(robot)

    # Apply only a world mounting transform; retain all CAD link frames.
    # Base STL has Y-up and minimum Y = -0.004224755335599184 metres.
    ET.SubElement(robot, "link", name="world")
    mount = ET.SubElement(robot, "joint", name="world_mount", type="fixed")
    ET.SubElement(mount, "parent", link="world")
    ET.SubElement(mount, "child", link="base_link")
    ET.SubElement(mount, "origin", xyz="0 0 0.004224755335599184",
                  rpy="1.57079633  0 0")

    unresolved = []
    for joint in robot.findall("joint"):
        if joint.get("type") == "fixed":
            continue
        axis = joint.find("axis")
        if axis is not None and all(float(v) == 0 for v in axis.get("xyz").split()):
            unresolved.append(joint.get("name"))
            joint.set("type", "fixed")
            for tag in ("axis", "limit"):
                child = joint.find(tag)
                if child is not None:
                    joint.remove(child)

    description = ET.tostring(robot, encoding="unicode")
    actions = []
    if unresolved:
        actions.append(LogInfo(msg=(
            "PREVIEW ONLY: CAD joints with undefined zero axes are held fixed: "
            + ", ".join(unresolved)
            + ". Source URDF is unchanged; gripper motion is not configured."
        )))
    actions.extend([
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             parameters=[{"robot_description": description}], output="screen"),
        Node(package="joint_state_publisher_gui", executable="joint_state_publisher_gui",
             parameters=[{"robot_description": description, **home_positions}],
             output="screen"),
        Node(package="rviz2", executable="rviz2",
             arguments=["-d", str(package / "config/display.rviz")], output="screen"),
    ])
    return LaunchDescription(actions)
