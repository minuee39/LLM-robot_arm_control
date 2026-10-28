import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts'))
from hand_guiding_core import origin,rotation,GravityModel


def transforms(path,q):
    robot=ET.parse(path).getroot()
    pending=robot.findall('joint')
    children={j.find('child').get('link') for j in pending}
    base=(set(l.get('name') for l in robot.findall('link'))-children).pop()
    frames={base:np.eye(4)}
    while pending:
        for j in pending[:]:
            p,c=j.find('parent').get('link'),j.find('child').get('link')
            if p not in frames:continue
            t=frames[p]@origin(j.find('origin'))
            if j.get('type')!='fixed':
                mimic=j.find('mimic')
                value=q[j.get('name')] if mimic is None else q[mimic.get('joint')]*float(mimic.get('multiplier',1))+float(mimic.get('offset',0))
                motion=np.eye(4);motion[:3,:3]=rotation(np.fromstring(j.find('axis').get('xyz'),sep=' '),value)
                t=t@motion
            frames[c]=t;pending.remove(j)
    return frames


def test_rebased_geometry_and_gravity():
    offsets=json.loads((ROOT/'config/hand_guiding_zero_reference.json').read_text())['offset_rad']
    old=ROOT/'urdf/before_zero_rebase_20261022/pallet_with_gripper.urdf'
    new=ROOT/'urdf/pallet_with_gripper.urdf'
    for seed in range(5):
        q=dict.fromkeys(offsets,0.) if seed==0 else dict(zip(offsets,np.random.default_rng(seed).uniform(-.04,.04,7)))
        oldq={n:(-q[n] if n=='J4' else q[n])+v for n,v in offsets.items()}
        a,b=transforms(old,oldq),transforms(new,q)
        correction=np.array(json.loads((ROOT/'config/base_level_alignment.json').read_text())['world_correction'])
        for link in a:
            expected=a[link] if link=='base_footprint' else correction@a[link]
            np.testing.assert_allclose(expected,b[link],atol=1e-12)
        g=np.array([0.,0.,-9.80665])
        ta,ua=GravityModel(old).evaluate(oldq,correction[:3,:3].T@g)
        tb,ub=GravityModel(new).evaluate(q)
        ta['J4'] = -ta['J4']
        np.testing.assert_allclose(list(ta.values()),list(tb.values()),atol=1e-12)
        mass=sum(float(e.get('value')) for e in ET.parse(new).getroot().findall('link/inertial/mass'))
        assert abs(ub-(ua-mass*(g@correction[:3,3])))<1e-12


def test_requested_limits_and_zero_inside():
    offsets=json.loads((ROOT/'config/hand_guiding_zero_reference.json').read_text())['offset_rad']
    old=ET.parse(ROOT/'urdf/before_zero_rebase_20261022/pallet_with_gripper.urdf').getroot()
    new=ET.parse(ROOT/'urdf/pallet_with_gripper.urdf').getroot()
    for n,d in offsets.items():
        a=old.find(f"joint[@name='{n}']/limit")
        b=new.find(f"joint[@name='{n}']/limit")
        if n.startswith('J'):
            assert float(b.get('lower')) == -np.pi
            assert float(b.get('upper')) == np.pi
        else:
            for k in ['lower','upper']:
                assert abs(float(b.get(k))-(float(a.get(k))-d))<1e-12
        assert float(b.get('lower'))<=0<=float(b.get('upper'))
