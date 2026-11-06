"""Read-only validation; --strict-dynamics fails on unresolved CAD properties."""
import argparse
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np


def rotation(rpy):
    x,y,z = map(float,rpy.split())
    cx,cy,cz = np.cos([x,y,z]); sx,sy,sz = np.sin([x,y,z])
    return np.array([[cz*cy,cz*sy*sx-sz*cx,cz*sy*cx+sz*sx],
                     [sz*cy,sz*sy*sx+cz*cx,sz*sy*cx-cz*sx],
                     [-sy,cy*sx,cy*cx]])


def validate(root):
    robot=ET.parse(root/'urdf/pallet.urdf').getroot()
    audit=json.loads((root/'config/mass_revised_audit.json').read_text())
    raw=root/'urdf/mass_revised_source'
    source=ET.parse(raw/'urdf/어셈블리1.SLDASM.urdf').getroot()
    raw_links={link.get('name'):link for link in source.findall('link')}
    for name,digest in audit['source_sha256'].items():
        assert hashlib.sha256((raw/name).read_bytes()).hexdigest()==digest, name
    links={link.get('name'):link for link in robot.findall('link')}
    joints={j.get('name'):j for j in robot.findall('joint')}
    assert len(links)==len(robot.findall('link'))==11
    assert len(joints)==len(robot.findall('joint'))==10
    children=set()
    for joint in joints.values():
        parent=joint.find('parent').get('link'); child=joint.find('child').get('link')
        assert parent in links and child in links and child not in children
        children.add(child)
        if joint.get('type')=='revolute':
            axis=np.fromstring(joint.find('axis').get('xyz'),sep=' ')
            assert abs(np.linalg.norm(axis)-1)<1e-6
            limit=joint.find('limit')
            assert float(limit.get('lower'))<float(limit.get('upper'))
            assert float(limit.get('effort'))>0 and float(limit.get('velocity'))>0
    assert set(links)-children=={'base_footprint'}
    seen={'base_footprint'}
    for _ in range(len(joints)):
        for j in joints.values():
            if j.find('parent').get('link') in seen: seen.add(j.find('child').get('link'))
    assert seen==set(links)
    assert joints['J6'].get('type')=='revolute' and joints['g2'].get('type')=='fixed'
    assert joints['g_r'].get('type')==joints['g_l'].get('type')=='revolute'
    assert joints['g_l'].find('mimic').attrib=={'joint':'g_r','multiplier':'-1','offset':'0'}
    assert joints['g_r'].find('mimic') is None
    invalid=[]
    base_vertices=None
    for name,link in links.items():
        if name=='base_footprint': continue
        item=link.find('visual'); filename=item.find('geometry/mesh').get('filename')
        assert filename.startswith('package://pallet/')
        path=root/filename.removeprefix('package://pallet/')
        assert hashlib.sha256(path.read_bytes()).hexdigest()==audit['source_sha256']['meshes/'+name+'.STL']
        assert link.find('collision/geometry/mesh').get('filename')==filename
        assert link.find('collision/origin').attrib==item.find('origin').attrib
        data=path.read_bytes(); n=int.from_bytes(data[80:84],'little')
        assert len(data)==84+50*n
        dtype=np.dtype([('normal','<f4',3),('v','<f4',(3,3)),('attr','<u2')])
        v=np.frombuffer(data,dtype=dtype,offset=84)['v'].reshape(-1,3)
        origin=item.find('origin')
        v=v@rotation(origin.get('rpy')).T+np.fromstring(origin.get('xyz'),sep=' ')
        assert np.isfinite(v).all()
        inertial=link.find('inertial')
        assert float(inertial.find('mass').get('value'))==audit['links'][name]['mass']>0
        com=np.fromstring(inertial.find('origin').get('xyz'),sep=' ')
        transform=audit['links'][name]
        r=np.array(transform['rotation']); t=np.array(transform['translation'])
        raw_inertial=raw_links[name].find('inertial')
        raw_origin=raw_inertial.find('origin')
        raw_com=np.fromstring(raw_origin.get('xyz'),sep=' ')
        raw_r=rotation(raw_origin.get('rpy'))
        if name=='base_link':
            provisional=rotation('1.5707963267948966 0 0')
            raw_com=provisional@raw_com; raw_r=provisional@raw_r
        assert np.allclose(com,r@raw_com+t,atol=1e-12,rtol=0)
        assert np.allclose(rotation(inertial.find('origin').get('rpy')),r@raw_r,atol=1e-12,rtol=0)
        assert np.allclose(rotation(origin.get('rpy')),r,atol=1e-12,rtol=0)
        assert np.allclose(np.fromstring(origin.get('xyz'),sep=' '),t,atol=1e-12,rtol=0)
        assert inertial.find('inertia').attrib==raw_inertial.find('inertia').attrib
        assert np.all(com>=v.min(0)-1e-6) and np.all(com<=v.max(0)+1e-6), name
        values=inertial.find('inertia').attrib
        xx,xy,xz,yy,yz,zz=[float(values[k]) for k in ('ixx','ixy','ixz','iyy','iyz','izz')]
        eig=np.linalg.eigvalsh([[xx,xy,xz],[xy,yy,yz],[xz,yz,zz]])
        assert np.isfinite(eig).all() and np.min(eig)>0
        if eig[-1]>eig[0]+eig[1]+1e-10: invalid.append(name)
        if name=='base_link': base_vertices=v
    mount=joints['base_footprint_joint'].find('origin')
    grounded=base_vertices@rotation(mount.get('rpy')).T+np.fromstring(mount.get('xyz'),sep=' ')
    assert abs(grounded[:,2].min())<1e-9
    home=ET.parse(root/'config/pallet.srdf').find("group_state[@name='home']")
    assert {j.get('name') for j in home}=={f'J{i}' for i in range(1,7)}
    for j in home:
        limit=joints[j.get('name')].find('limit')
        assert float(limit.get('lower'))<=float(j.get('value'))<=float(limit.get('upper'))
    assert (root/'urdf/pallet.urdf').read_bytes()==(root/'urdf/pallet_with_gripper.urdf').read_bytes()
    return invalid


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--package-root',type=Path,default=Path(__file__).resolve().parents[1])
    parser.add_argument('--strict-dynamics',action='store_true')
    args=parser.parse_args()
    invalid=validate(args.package_root)
    print('PASS: source hashes, new mesh hashes, tree, joint limits/mimic, COM bounds, ground contact, SRDF home')
    print('UNRESOLVED: provisional base mass frame; invalid source inertia tensors: '+', '.join(invalid))
    if args.strict_dynamics: raise SystemExit(2)
