#!/usr/bin/env python3
"""Change joint coordinates q_new=q_old-offset, preserving link geometry."""
import copy
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation


def rebase(robot, offsets):
    robot = copy.deepcopy(robot)
    joints = robot.findall('joint')
    shifts = dict(offsets)
    for joint in joints:
        mimic = joint.find('mimic')
        if mimic is not None:
            shifts[joint.get('name')] = float(mimic.get('multiplier',1))*shifts.get(mimic.get('joint'),0)
    for joint in joints:
        name = joint.get('name')
        delta = shifts.get(name,0)
        if joint.get('type') == 'fixed':
            continue
        origin = joint.find('origin')
        if origin is None:
            origin = ET.SubElement(joint,'origin',xyz='0 0 0',rpy='0 0 0')
        rpy = np.fromstring(origin.get('rpy','0 0 0'),sep=' ')
        xyz = np.fromstring(origin.get('xyz','0 0 0'),sep=' ')
        axis = np.fromstring(joint.find('axis').get('xyz'),sep=' ')
        axis /= np.linalg.norm(axis)
        r = Rotation.from_euler('xyz',rpy)
        if joint.get('type') in ('revolute','continuous'):
            origin.set('rpy',' '.join(f'{v:.16g}' for v in (r*Rotation.from_rotvec(axis*delta)).as_euler('xyz')))
        elif joint.get('type') == 'prismatic':
            origin.set('xyz',' '.join(f'{v:.16g}' for v in xyz+r.apply(axis*delta)))
        else:
            raise ValueError('Unsupported joint type')
        limit = joint.find('limit')
        if limit is not None:
            for key in ('lower','upper'):
                if key in limit.attrib:
                    limit.set(key,f'{float(limit.get(key))-delta:.16g}')
        mimic = joint.find('mimic')
        if mimic is not None:
            shift = float(mimic.get('multiplier',1))*shifts.get(mimic.get('joint'),0)
            mimic.set('offset',f'{float(mimic.get("offset",0))+shift-delta:.16g}')
    return robot
