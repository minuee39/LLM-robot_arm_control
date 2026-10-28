"""Static checks for the canonical Pallet 6-axis + one-actuator gripper config."""
from pathlib import Path
import xml.etree.ElementTree as ET

import yaml

ROOT = Path(__file__).resolve().parents[1]
PALLET = ROOT.parent / "pallet"
ARM = [f"J{i}" for i in range(1, 7)]


def test_moveit_and_can_joint_contract():
    limits = yaml.safe_load((ROOT / "config/joint_limits.yaml").read_text())
    hw = yaml.safe_load((PALLET / "config/robstride_hardware.yaml").read_text())
    controllers = yaml.safe_load((ROOT / "config/moveit_controllers.yaml").read_text())
    joints = controllers["moveit_simple_controller_manager"]["pallet_arm_controller"]["joints"]
    assert joints == ARM
    assert list(hw["joints"]) == ARM
    assert [hw["joints"][name]["motor_id"] for name in ARM] == list(range(1, 7))
    assert set(limits["joint_limits"]) == set(ARM + ["g_r"])
    assert 0 < limits["default_velocity_scaling_factor"] <= 1
    assert 0 < limits["default_acceleration_scaling_factor"] <= 1


def test_canonical_urdf_gripper_mimic_and_srdf():
    urdf = ET.parse(PALLET / "urdf/pallet_with_gripper.urdf").getroot()
    joints = {j.get("name"): j for j in urdf.findall("joint")}
    for name in ARM:
        assert joints[name].get("type") == "revolute"
    assert joints["g_r"].get("type") == "revolute"
    assert joints["g_l"].find("mimic").get("joint") == "g_r"
    srdf = ET.parse(ROOT / "config/pallet.srdf").getroot()
    assert srdf.find("group[@name='gripper']/joint").get("name") == "g_r"
    assert srdf.find("group[@name='pallet_arm']/chain").get("tip_link") == "gripper"
