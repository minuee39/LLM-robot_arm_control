"""Check semantic references and execution joint sets against the actual CAD model."""
from pathlib import Path
import xml.etree.ElementTree as ET
import yaml

ROOT = Path(__file__).resolve().parents[1]
MODEL = ROOT.parent / 'pallet/urdf/pallet_with_gripper.urdf'
ARM = [f'J{i}' for i in range(1, 7)]


def config(name):
    return yaml.safe_load((ROOT / 'config' / name).read_text())


def test_semantic_references_and_six_axis_chain():
    urdf = ET.parse(MODEL).getroot()
    srdf = ET.parse(ROOT / 'config/pallet.srdf').getroot()
    links = {x.get('name') for x in urdf.findall('link')}
    joints = {x.get('name'): x for x in urdf.findall('joint')}
    chain = srdf.find("group[@name='pallet_arm']/chain")
    child_map = {j.find('child').get('link'): j for j in joints.values()}
    tip = chain.get('tip_link')
    found = []
    while tip != chain.get('base_link'):
        joint = child_map[tip]
        if joint.get('type') != 'fixed':
            found.append(joint.get('name'))
        tip = joint.find('parent').get('link')
    assert found[::-1] == ARM
    for pair in srdf.findall('disable_collisions'):
        assert pair.get('link1') in links and pair.get('link2') in links
    assert joints['g_l'].find('mimic').get('joint') == 'g_r'
    assert not srdf.findall("group_state[@name='home']")


def test_controller_interfaces_match_model():
    limits = config('joint_limits.yaml')['joint_limits']
    initial = config('initial_positions.yaml')['initial_positions']
    moveit = config('moveit_controllers.yaml')['moveit_simple_controller_manager']
    ros = config('ros2_controllers.yaml')
    assert set(limits) == set(initial) == set(ARM + ['g_r'])
    for controller, expected in [('pallet_arm_controller', ARM), ('pallet_gripper_controller', ['g_r'])]:
        assert moveit[controller]['joints'] == expected
        assert ros[controller]['ros__parameters']['joints'] == expected
    urdf = ET.parse(MODEL).getroot()
    for name, value in initial.items():
        bound = urdf.find(f"joint[@name='{name}']/limit")
        assert float(bound.get('lower')) <= value <= float(bound.get('upper'))
        assert 0 < limits[name]['max_velocity'] <= float(bound.get('velocity'))
