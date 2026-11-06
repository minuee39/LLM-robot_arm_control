#!/usr/bin/env python3
"""Render CAD rigid bodies using the same solved preview joints."""
import argparse
from pathlib import Path
import xml.etree.ElementTree as ET

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.collections import PolyCollection
import numpy as np

from adaptive_gripper_core import AdaptiveGripper, rotation
from generate_adaptive_gripper import load_triangles
from validate_mass_revised import rotation as rpy_rotation


def transforms(robot, positions):
    world = {'gripper': np.eye(4)}
    pending = list(robot.findall('joint'))
    while pending:
        progress = False
        for joint in pending[:]:
            parent = joint.find('parent').get('link')
            if parent not in world:
                continue
            origin = joint.find('origin')
            local = np.eye(4)
            local[:3, :3] = rpy_rotation(origin.get('rpy', '0 0 0'))
            local[:3, 3] = np.fromstring(origin.get('xyz', '0 0 0'), sep=' ')
            motion = np.eye(4)
            q = positions.get(joint.get('name'), 0.0)
            if joint.get('type') == 'prismatic':
                motion[:3, 3] = np.fromstring(joint.find('axis').get('xyz'), sep=' ')*q
            elif joint.get('type') == 'revolute':
                motion[:2, :2] = rotation(q)  # generated joints all use +Z
            world[joint.find('child').get('link')] = world[parent]@local@motion
            pending.remove(joint)
            progress = True
        if not progress:
            raise ValueError('URDF tree disconnected')
    return world


def render(package, output):
    model = AdaptiveGripper.load(package/'config/adaptive_gripper/geometry.json')
    robot = ET.parse(package/'urdf/adaptive_gripper_preview.urdf').getroot()
    fig, axes = plt.subplots(1, 3, figsize=(12, 5), sharex=True, sharey=True)
    angle = np.pi/8
    u = np.array([-np.sin(angle), 0., np.cos(angle)])
    normal = np.array([np.cos(angle), 0., np.sin(angle)])
    for ax, motor in zip(axes, [12, 0, -12]):
        pose = model.pose(motor)
        tf = transforms(robot, pose['joints'])
        faces = []
        for link in robot.findall('link'):
            if link.get('name') == 'gripper':
                continue
            for visual in link.findall('visual'):
                mesh = visual.find('geometry/mesh')
                if mesh is None:
                    continue
                tri = load_triangles(package/mesh.get('filename').split('package://pallet/')[1])
                vo = visual.find('origin')
                if vo is not None:
                    tri = tri@rpy_rotation(vo.get('rpy', '0 0 0')).T+np.fromstring(vo.get('xyz', '0 0 0'), sep=' ')
                t = tf[link.get('name')]
                tri = (tri@t[:3, :3].T+t[:3, 3])*1000
                color = np.fromstring(visual.find('material/color').get('rgba'), sep=' ')[:3]
                for triangle in tri:
                    projected = np.column_stack((triangle@u, triangle[:, 1]))
                    faces.append((float(triangle.mean(axis=0)@normal), projected, color))
        faces.sort(key=lambda item: item[0])
        ax.add_collection(PolyCollection([f[1] for f in faces], facecolors=[f[2] for f in faces], edgecolors='none'))
        contacts = []
        for side, data in model.geometry['sides'].items():
            pin = np.array(data['pins']['pad_inner']['move_xy_mm'])
            local = np.r_[(np.array(data['contact_datum_xy_mm'])-pin)/1000, 0, 1]
            point = tf[f'{side}_wrist_link']@local*1000
            contacts.append([point[:3]@u, point[1]])
        y = max(p[1] for p in contacts)+6
        xs = sorted(p[0] for p in contacts)
        ax.annotate('', xy=(xs[0], y), xytext=(xs[1], y), arrowprops={'arrowstyle': '<->', 'color': 'black'})
        ax.text(np.mean(xs), y+2, f"{pose['opening_mm']/10:.1f} cm", ha='center')
        ax.set_title(f"{motor:+g} rad | nut depth {pose['nut_depth_mm']/10:.1f} cm")
        ax.set_aspect('equal')
        ax.set_xlim(-85, 85)
        ax.set_ylim(0, 145)
        ax.set_xlabel('Front horizontal (mm)')
        ax.grid(alpha=.2)
    axes[0].set_ylabel('Height (mm)')
    fig.suptitle('Straight inner links + parallel pads | proposed virtual wrist | base hidden')
    fig.tight_layout()
    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output, dpi=170)
    plt.close(fig)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--package-root', type=Path, default=Path(__file__).resolve().parents[1])
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    render(args.package_root, args.output)
