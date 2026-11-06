import importlib.util
import json
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np

ROOT=Path(__file__).parents[1]
spec=importlib.util.spec_from_file_location('cad',ROOT/'scripts/cad_tcp_reference.py')
cad=importlib.util.module_from_spec(spec);spec.loader.exec_module(cad)


def test_reference_matches_current_meshes_and_generated_frame():
    result=cad.extract(ROOT.parent/'pallet')
    stored=json.loads((ROOT/'config/cad_tcp_reference.json').read_text())
    assert result['source_sha256']==stored['source_sha256']
    np.testing.assert_allclose(result['position_m'],stored['position_m'],atol=1e-12)
    robot=ET.parse(ROOT/'config/cad_tcp_reference.xacro').getroot()
    joint=robot.find('joint');r,t=cad.origin(joint.find('origin'))
    np.testing.assert_allclose(t,result['position_m'],atol=1e-12)
    # Approach comes from CAD geometry +Y, independent of the old legacy TCP axes.
    np.testing.assert_allclose(r[:,2],[0,1,0],atol=1e-12)


def test_reference_does_not_claim_measured_opening_or_replace_planning_tip():
    data=json.loads((ROOT/'config/cad_tcp_reference.json').read_text())
    assert data['physical_opening_mm'] is None
    assert data['status']=='CAD_REFERENCE_ONLY_NOT_PHYSICAL_TCP'
    srdf=ET.parse(ROOT/'config/pallet.srdf').getroot()
    assert srdf.find("group[@name='pallet_arm']/chain").get('tip_link')=='gripper'
