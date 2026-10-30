#!/usr/bin/env python3
"""Extract a CAD distal-edge reference, not a calibrated physical grasp TCP."""
import argparse
import hashlib
import json
from pathlib import Path
import struct
import xml.etree.ElementTree as ET
import numpy as np


def rpy(text):
    x,y,z=map(float,text.split())
    cx,cy,cz=np.cos([x,y,z]);sx,sy,sz=np.sin([x,y,z])
    return np.array([[cz*cy,cz*sy*sx-sz*cx,cz*sy*cx+sz*sx],
                     [sz*cy,sz*sy*sx+cz*cx,sz*sy*cx-cz*sx],[-sy,cy*sx,cy*cx]])


def origin(element):
    return rpy(element.get('rpy','0 0 0')),np.fromstring(element.get('xyz','0 0 0'),sep=' ')


def vertices(path):
    data=path.read_bytes()
    if len(data)<84 or len(data)!=84+50*struct.unpack_from('<I',data,80)[0]:
        raise ValueError('Expected binary STL with complete triangles')
    dtype=np.dtype([('normal','<f4',3),('vertices','<f4',(3,3)),('attribute','<u2')])
    v=np.frombuffer(data,dtype=dtype,offset=84)['vertices'].reshape(-1,3)
    if not np.isfinite(v).all():raise ValueError('Nonfinite mesh')
    return v


def extract(root):
    model=root/'urdf/pallet_with_gripper.urdf'
    robot=ET.parse(model).getroot()
    tips=[];sources={str(model.relative_to(root)):hashlib.sha256(model.read_bytes()).hexdigest()}
    for name,joint in [('gripper_r','g_r'),('gripper_l','g_l')]:
        visual=robot.find(f"link[@name='{name}']/visual")
        path=root/visual.find('geometry/mesh').get('filename').removeprefix('package://pallet/')
        sources[str(path.relative_to(root))]=hashlib.sha256(path.read_bytes()).hexdigest()
        rv,tv=origin(visual.find('origin'));rj,tj=origin(robot.find(f"joint[@name='{joint}']/origin"))
        points=(vertices(path)@rv.T+tv)@rj.T+tj
        # Center of terminal edge bounds: independent of triangulation density.
        edge=points[points[:,1]>=points[:,1].max()-1e-7]
        tips.append((edge.min(0)+edge.max(0))/2)
    middle=np.mean(tips,axis=0)
    rg,tg=origin(robot.find("joint[@name='g2']/origin"))
    reference=rg@middle+tg
    return {'status':'CAD_REFERENCE_ONLY_NOT_PHYSICAL_TCP',
            'parent_frame':'gripper','child_frame':'cad_tcp_reference_unverified',
            'model_joint_rad':{'g_r':0.0,'g_l':0.0},
            'physical_opening_mm':None,
            'position_m':reference.tolist(),
            'rpy_rad':[-np.pi/2,1.17809724509618,0.0],
            'approach_definition':'reference +Z is CAD move_gripper +Y; +X follows move_gripper +X',
            'distal_edge_centers_in_move_gripper_m':[p.tolist() for p in tips],
            'source_sha256':sources,
            'method':'Transform both CAD STLs through visual and joint origins at model zero, midpoint of terminal +Y edge bounding centers.',
            'limitations':['Not a contact-pad center measurement.',
                           'Actual motor 0rad=20mm does not establish model g_r=0.',
                           'Adaptive contact and screw motion are not modeled here.',
                           'Do not use as an automatically commissioned physical motion target.']}


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model-root',type=Path)
    parser.add_argument('--output',type=Path)
    args=parser.parse_args()
    if args.model_root is None:
        from ament_index_python.packages import get_package_share_directory
        args.model_root=Path(get_package_share_directory('pallet'))
    result=extract(args.model_root)
    if args.output:
        with args.output.open('x') as f:json.dump(result,f,indent=2);f.write('\n')
    else:print(json.dumps(result,indent=2))


if __name__=='__main__':main()
