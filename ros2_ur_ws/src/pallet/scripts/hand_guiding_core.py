"""URDF gravity compensation. SI units; gravity expressed in URDF root frame."""
import math
import xml.etree.ElementTree as ET
import numpy as np


def vec(element, key, default='0 0 0'):
    return np.fromstring(element.get(key, default) if element is not None else default, sep=' ')


def rotation(axis, angle):
    axis = np.asarray(axis, dtype=float)
    axis = axis / np.linalg.norm(axis)
    x, y, z = axis
    k = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    return np.eye(3) + math.sin(angle)*k + (1-math.cos(angle))*(k@k)


def origin(element):
    t = np.eye(4)
    r, p, y = vec(element, 'rpy')
    t[:3, :3] = rotation([0,0,1], y) @ rotation([0,1,0], p) @ rotation([1,0,0], r)
    t[:3, 3] = vec(element, 'xyz')
    return t


class GravityModel:
    def __init__(self, path):
        root = ET.parse(path).getroot()
        self.links = {e.get('name'): e for e in root.findall('link')}
        self.joints = root.findall('joint')
        children = {j.find('child').get('link') for j in self.joints}
        roots = set(self.links)-children
        if len(roots) != 1:
            raise ValueError('URDF must have one root')
        self.root = roots.pop()
        self.names = [j.get('name') for j in self.joints if j.get('type') != 'fixed' and j.find('mimic') is None]

    def evaluate(self, positions, gravity=(0,0,-9.80665)):
        if set(positions) != set(self.names) or not np.isfinite(list(positions.values())).all():
            raise ValueError('Provide finite positions for every independent joint')
        gravity = np.asarray(gravity, dtype=float)
        if gravity.shape != (3,) or not np.isfinite(gravity).all():
            raise ValueError('Invalid gravity vector')
        transforms = {self.root: np.eye(4)}
        paths = {self.root: []}
        axes = {}
        pending = list(self.joints)
        while pending:
            progress = False
            for j in pending[:]:
                parent, child = j.find('parent').get('link'), j.find('child').get('link')
                if parent not in transforms:
                    continue
                name, kind = j.get('name'), j.get('type')
                t = transforms[parent] @ origin(j.find('origin'))
                path = list(paths[parent])
                motion = np.eye(4)
                if kind != 'fixed':
                    if kind not in ('revolute', 'continuous', 'prismatic'):
                        raise ValueError('Unsupported joint type: '+kind)
                    a = vec(j.find('axis'), 'xyz', '1 0 0')
                    a = a / np.linalg.norm(a)
                    mimic = j.find('mimic')
                    source = name if mimic is None else mimic.get('joint')
                    factor = 1.0 if mimic is None else float(mimic.get('multiplier', 1))
                    q = positions[source]*factor + (0 if mimic is None else float(mimic.get('offset', 0)))
                    axes[name] = (t[:3,3].copy(), t[:3,:3]@a, kind, source, factor)
                    path.append(name)
                    if kind == 'prismatic':
                        motion[:3,3] = a*q
                    else:
                        motion[:3,:3] = rotation(a, q)
                transforms[child], paths[child] = t@motion, path
                pending.remove(j)
                progress = True
            if not progress:
                raise ValueError('Disconnected or cyclic URDF')
        torque = dict.fromkeys(self.names, 0.0)
        potential = 0.0
        for name, link in self.links.items():
            inertial = link.find('inertial')
            if inertial is None:
                continue
            mass = float(inertial.find('mass').get('value'))
            if not math.isfinite(mass) or mass < 0:
                raise ValueError('Invalid link mass')
            com = (transforms[name] @ origin(inertial.find('origin')))[:3,3]
            force = mass*gravity
            potential -= float(force@com)
            for joint in paths[name]:
                point, axis, kind, source, factor = axes[joint]
                jac = axis if kind == 'prismatic' else np.cross(axis, com-point)
                torque[source] -= factor*float(jac@force)
        return torque, potential


def command(model, config, motor_positions, joint_velocities, gravity_vector=(0,0,-9.80665)):
    """q = direction*(motor-zero)/ratio; torque obeys virtual work."""
    q = {}
    for name, c in config.items():
        values = [c[k] for k in ('direction','zero_rad','ratio','damping','torque_limit_nm','current_limit_a','output_nm_per_a','lower_rad','upper_rad','max_speed_rad_s')]
        if not np.isfinite(values).all() or c['direction'] not in (-1,1):
            raise ValueError('Invalid calibration: '+name)
        if any(c[k] <= 0 for k in ('ratio','torque_limit_nm','current_limit_a','output_nm_per_a','max_speed_rad_s')) or c['damping'] < 0 or c['lower_rad'] >= c['upper_rad']:
            raise ValueError('Invalid limits: '+name)
        q[name] = c['direction']*(motor_positions[c['motor_id']]-c['zero_rad'])/c['ratio']
        if not c['lower_rad'] <= q[name] <= c['upper_rad']:
            raise RuntimeError('Position outside commissioned limits: '+name)
        if not math.isfinite(joint_velocities[name]) or abs(joint_velocities[name]) > c['max_speed_rad_s']:
            raise RuntimeError('Overspeed: '+name)
    gravity, _ = model.evaluate(q, gravity_vector)
    currents = {}
    for name, c in config.items():
        tau = gravity[name]-c['damping']*joint_velocities[name]
        # Refuse insufficient torque capacity instead of silently undercompensating.
        if abs(tau) > c['torque_limit_nm']:
            raise RuntimeError('Required torque exceeds limit: '+name)
        current = c['direction']*tau/(c['ratio']*c['output_nm_per_a'])
        if abs(current) > c['current_limit_a']:
            raise RuntimeError('Required current exceeds limit: '+name)
        currents[c['motor_id']] = current
    return q, gravity, currents
