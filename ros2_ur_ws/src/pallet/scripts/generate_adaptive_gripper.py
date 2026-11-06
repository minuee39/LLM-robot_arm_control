#!/usr/bin/env python3
"""Generate a separate articulated preview from unchanged CAD meshes.

No CAD/mass reference files are overwritten. Mesh components and pin rings are
CAD-derived. The nut upper face is assumed to be the user measurement datum.
"""
import argparse
import copy
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components

from adaptive_gripper_core import AdaptiveGripper
from adaptive_gripper_mesh import split_pad
from validate_mass_revised import rotation

DTYPE = np.dtype([('normal', '<f4', 3), ('v', '<f4', (3, 3)), ('attr', '<u2')])
SOURCE_YAW = {'gripper_r': -1.74528160479734, 'gripper_l': -1.39631104879178}
ANCHOR = {0: 'pad_outer', 1: 'inner_elbow', 2: 'rocker_outer', 3: 'inner_ground',
          4: 'nut_pivot', 5: 'nut_pivot', 6: 'rocker_ground'}
ROLE = {0: 'pad', 1: 'inner_upper', 2: 'outer', 3: 'inner_lower',
        4: 'helper', 5: 'helper', 6: 'rocker'}


def origin(element):
    return rotation(element.get('rpy', '0 0 0')), np.fromstring(element.get('xyz', '0 0 0'), sep=' ')


def load_triangles(path):
    data = path.read_bytes()
    count = int.from_bytes(data[80:84], 'little')
    if len(data) != 84+50*count or not count:
        raise ValueError(f'Invalid binary STL: {path}')
    tri = np.frombuffer(data, DTYPE, count=count, offset=84)['v'].astype(float)
    if not np.isfinite(tri).all():
        raise ValueError(f'Nonfinite mesh: {path}')
    return tri


def split_components(tri):
    # Vertex equality is tested in raw CAD coordinates at 0.1 um tolerance.
    vertices, inverse = np.unique(np.round(tri.reshape(-1, 3), 7), axis=0, return_inverse=True)
    faces = inverse.reshape(-1, 3)
    rows, cols = faces[:, [0, 1, 2]].ravel(), faces[:, [1, 2, 0]].ravel()
    graph = coo_matrix((np.ones(len(rows)), (rows, cols)), shape=(len(vertices), len(vertices)))
    count, labels = connected_components(graph.tocsr(), directed=False)
    return [tri[labels[faces[:, 0]] == i] for i in range(count)]


def write_stl(path, tri):
    records = np.zeros(len(tri), dtype=DTYPE)
    normal = np.cross(tri[:, 1]-tri[:, 0], tri[:, 2]-tri[:, 0])
    normal /= np.maximum(np.linalg.norm(normal, axis=1, keepdims=True), 1e-30)
    records['normal'], records['v'] = normal, tri
    path.write_bytes(b'CAD component; articulated preview'.ljust(80, b' ')+len(tri).to_bytes(4, 'little')+records.tobytes())


def vector_text(values):
    return ' '.join(f'{float(x):.16g}' for x in values)


def add_joint(robot, name, parent, child, xyz, kind='revolute', rpy=None, axis='0 0 1', limits=(-2.4, 2.4)):
    joint = ET.SubElement(robot, 'joint', name=name, type=kind)
    ET.SubElement(joint, 'parent', link=parent)
    ET.SubElement(joint, 'child', link=child)
    ET.SubElement(joint, 'origin', xyz=vector_text(xyz), rpy=rpy or '0 0 0')
    if kind != 'fixed':
        ET.SubElement(joint, 'axis', xyz=axis)
        # Positive parser bounds, preview only; NOT commissioned force/speed limits.
        ET.SubElement(joint, 'limit', lower=str(limits[0]), upper=str(limits[1]), effort='1', velocity='0.1')
    return joint


def save_xml(robot, path):
    ET.indent(robot, space='  ')
    ET.ElementTree(robot).write(path, encoding='utf-8', xml_declaration=True)


def generate(root, calibration_path):
    robot = ET.parse(root/'urdf/pallet_with_gripper.urdf').getroot()
    pins = json.loads((root/'config/adaptive_gripper/cad_pins.json').read_text())
    calibration = json.loads(calibration_path.read_text())
    # Refuse different input geometry instead of applying component IDs blindly.
    source = root/'urdf/mass_revised_source'
    for name in SOURCE_YAW:
        if (root/f'meshes/mass_revised/{name}.STL').read_bytes() != (source/f'meshes/{name}.STL').read_bytes():
            raise ValueError(f'{name} is not the audited source STL')
    raw_robot = ET.parse(source/'urdf/어셈블리1.SLDASM.urdf').getroot()
    output = root/'meshes/adaptive_gripper'
    output.mkdir(exist_ok=True)
    geometry = {'schema_version': 1, 'status': 'MEASURED_FREE_SPACE_PREVIEW',
                'calibration': calibration, 'screw_reference': pins['screw_reference'], 'source_yaw_rad': SOURCE_YAW, 'sides': {}, 'source_sha256': {},
                'limitations': ['CAD screw top to nut upper face is used; the physical nut measurement face must be confirmed.',
                               'Both nut depth and opening are calibration inputs; opening is not predicted from nut travel.',
                               'Inner links stay straight and pads stay parallel through an added virtual wrist. The source pad is monolithic; this proposed articulation is not verified on hardware. No measured springs/contact response.',
                               'Tree loop cuts are checked numerically; no closed-loop physics or hardware actuation.']}
    for name, yaw in SOURCE_YAW.items():
        side = name[-1]
        joint_name = 'g_'+side
        raw_yaw = float(raw_robot.find(f"joint[@name='{joint_name}']/origin").get('rpy').split()[2])
        if abs(raw_yaw-yaw) > 1e-12:
            raise ValueError('Source CAD joint orientation changed')
        visual = robot.find(f"link[@name='{name}']/visual")
        rv, tv = origin(visual.find('origin'))
        rj = rotation(f'0 0 {yaw}')
        tj = np.fromstring(raw_robot.find(f"joint[@name='{joint_name}']/origin").get('xyz'), sep=' ')
        path = root/f'meshes/mass_revised/{name}.STL'
        geometry['source_sha256'][str(path.relative_to(root))] = hashlib.sha256(path.read_bytes()).hexdigest()
        raw = load_triangles(path)
        components = split_components(raw)
        if len(components) != 7 or sorted(map(len, components)) != [324, 442, 502, 714, 720, 794, 1018]:
            raise ValueError('CAD components changed; pin mapping requires a new audit')
        components = [(tri@rv.T+tv)@rj.T+tj for tri in components]
        p = pins['sides'][name]
        # Distal inner contact datum: innermost CAD vertex within last 3 mm.
        # Same point is used in every pose, rather than a moving image pixel.
        pad = components[0].reshape(-1, 3)*1000
        distal = pad[pad[:, 1] >= pad[:, 1].max()-3.0]
        tip = distal[np.argmin(np.abs(distal[:, 0])), :2]
        # Long straight pad edges determine the face direction in source CAD.
        pad_tri = components[0]*1000
        edges = np.concatenate([pad_tri[:, 1]-pad_tri[:, 0],
                                pad_tri[:, 2]-pad_tri[:, 1],
                                pad_tri[:, 0]-pad_tri[:, 2]])
        edges = edges[(np.linalg.norm(edges[:, :2], axis=1)>30)&(abs(edges[:, 2])<1e-4)]
        edges *= np.sign(edges[:, 1, None])
        angles = np.arctan2(edges[:, 0],edges[:, 1])
        directions = np.unique(np.round(angles,4))
        if len(directions) != 3:
            raise ValueError('Pad edge directions changed; face direction needs audit')
        parallel_angle = float(np.mean(angles[abs(angles-np.median(directions))<1e-4]))
        geometry['sides'][side] = {'parallel_pad_angle_rad': parallel_angle, 'pins': p, 'contact_datum_xy_mm': tip.tolist(),
                                    'contact_datum_method': 'Innermost vertex in distal 3 mm of source CAD pad; fixed material point.'}
        # Proposed wrist: split the monolithic pad 10 mm above its inner pin.
        # This is a new visual articulation, not an audited physical CAD joint.
        normal = np.array([np.sin(parallel_angle),np.cos(parallel_angle),0.])
        wrist_anchor = np.r_[p['pad_inner']['move_xy_mm'],0]/1000
        cut_point = wrist_anchor+normal*.010
        yoke, contact_pad = split_pad(components[0],cut_point,normal)
        write_stl(output/f'{side}_yoke.stl',yoke-np.r_[p['pad_outer']['move_xy_mm'],0]/1000)
        write_stl(output/f'{side}_contact_pad.stl',contact_pad-wrist_anchor)
        geometry['sides'][side]['proposed_wrist'] = {
            'pivot': 'pad_inner', 'cut_offset_mm': 10.0,
            'physical_joint_verified': False,
            'source_triangles': len(components[0]),
            'split_triangles': [len(yoke),len(contact_pad)]}
        for i, tri in enumerate(components):
            anchor = np.r_[p[ANCHOR[i]]['move_xy_mm'], 0]/1000
            write_stl(output/f'{side}_{i}.stl', tri-anchor)
    source_gap = sum(abs(side['contact_datum_xy_mm'][0]) for side in geometry['sides'].values())
    widths = [row['opening_mm'] for row in calibration['samples']]
    depths = [row['nut_depth_mm'] for row in calibration['samples']]
    if not widths[0] <= source_gap <= widths[-1]:
        raise ValueError('CAD reference opening lies outside measured calibration')
    reference_depth = pins['screw_reference']['source_nut_top_drop_mm']
    geometry['reference'] = {'source_contact_opening_mm': source_gap,
                             'cad_nut_upper_face_depth_mm': reference_depth,
                             'datum_is_physically_verified': False,
                             'nut_top_reference': 'Assumes user measures screw top to nut upper face; physical face not verified'}
    geometry['source_sha256']['urdf/mass_revised_source/urdf/어셈블리1.SLDASM.urdf'] = hashlib.sha256((source/'urdf/어셈블리1.SLDASM.urdf').read_bytes()).hexdigest()
    # Validate the whole measured interval before replacing URDF/metadata.
    model = AdaptiveGripper(geometry, calibration)
    for angle in np.linspace(calibration['samples'][0]['motor_angle_rad'], calibration['samples'][-1]['motor_angle_rad'], 97):
        model.pose(float(angle))
    geometry_path = root/'config/adaptive_gripper/geometry.json'
    geometry_path.write_text(json.dumps(geometry, ensure_ascii=False, indent=2)+'\n')
    additions = ET.Element('robot', name='adaptive_gripper_preview')
    additions.append(ET.Comment('Visual kinematic reconstruction only. No measured split inertias, physical effort/velocity limits, contact law or actuator interface.'))
    # A display frame with +Z up along the fingers and +X toward the viewer.
    additions.append(ET.Element('link', name='preview_view'))
    add_joint(additions, 'preview_view_joint', 'gripper', 'preview_view', [0, 0, 0],
              kind='fixed', rpy='-1.5707963267948966 -0.39269908169872414 0')
    additions.append(ET.Element('link', name='adaptive_frame'))
    nut = copy.deepcopy(robot.find("link[@name='move_gripper']"))
    # Original nut visual frame is retained; no arbitrary mass split.
    additions.append(nut)
    old_origin = robot.find("joint[@name='g2']/origin")
    xyz = np.fromstring(old_origin.get('xyz'), sep=' ')
    add_joint(additions, 'adaptive_frame_joint', 'gripper', 'adaptive_frame', xyz,
              kind='fixed', rpy=old_origin.get('rpy'))
    dlow, dhigh = min(depths), max(depths)
    add_joint(additions, 'g2', 'gripper', 'move_gripper', xyz, kind='prismatic',
              rpy=old_origin.get('rpy'), axis='0 1 0',
              limits=((reference_depth-dhigh)/1000, (reference_depth-dlow)/1000))
    for side, data in geometry['sides'].items():
        p = {k: np.r_[v['move_xy_mm'], 0]/1000 for k, v in data['pins'].items()}
        specs = [('rocker', 'adaptive_frame', 'rocker_ground', None),
                 ('helper', 'move_gripper', 'nut_pivot', None),
                 ('outer', f'{side}_rocker_link', 'rocker_outer', 'rocker_ground'),
                 ('pad', f'{side}_outer_link', 'pad_outer', 'rocker_outer'),
                 ('wrist', f'{side}_pad_link', 'pad_inner', 'pad_outer'),
                 ('inner_lower', 'adaptive_frame', 'inner_ground', None),
                 ('inner_upper', f'{side}_inner_lower_link', 'inner_elbow', 'inner_ground')]
        color = '0.91 0.43 0.29 1' if side == 'r' else '0.18 0.68 0.81 1'
        for role, parent, anchor, parent_anchor in specs:
            linkname = f'{side}_{role}_link'
            link = ET.SubElement(additions, 'link', name=linkname)
            mesh_names = ([f'{side}_yoke.stl'] if role == 'pad' else
                          [f'{side}_contact_pad.stl'] if role == 'wrist' else
                          [f'{side}_{i}.stl' for i,name in ROLE.items() if name == role])
            for mesh_name in mesh_names:
                for element in ('visual', 'collision'):
                    item = ET.SubElement(link, element)
                    shape = ET.SubElement(item, 'geometry')
                    ET.SubElement(shape, 'mesh', filename=f'package://pallet/meshes/adaptive_gripper/{mesh_name}')
                    if element == 'visual':
                        material = ET.SubElement(item, 'material', name=f'adaptive_{side}')
                        ET.SubElement(material, 'color', rgba=color)
            offset = p[anchor]-(p[parent_anchor] if parent_anchor else 0)
            add_joint(additions, f'{side}_{role}', parent, linkname, offset)
    # Give the nut an unambiguous display color.
    material = additions.find("link[@name='move_gripper']/visual/material")
    material.set('name', 'adaptive_nut')
    material.find('color').set('rgba', '0.96 0.71 0.20 1')
    standalone = ET.Element('robot', name='adaptive_gripper_preview')
    standalone.append(copy.deepcopy(robot.find("link[@name='gripper']")))
    standalone.extend(copy.deepcopy(list(additions)))
    save_xml(standalone, root/'urdf/adaptive_gripper_preview.urdf')
    full = copy.deepcopy(robot)
    for element in list(full):
        if element.tag == 'link' and element.get('name') in {'move_gripper', 'gripper_l', 'gripper_r'}:
            full.remove(element)
        elif element.tag == 'joint' and element.get('name') in {'g2', 'g_l', 'g_r'}:
            full.remove(element)
    full.extend(copy.deepcopy(list(additions)))
    save_xml(full, root/'urdf/pallet_adaptive_preview.urdf')
    # Reusable descriptions omit the RViz-only camera reference frame.
    for tree, name in [(standalone, 'adaptive_gripper'), (full, 'pallet_adaptive')]:
        exported = copy.deepcopy(tree)
        exported.set('name', name)
        for element in list(exported):
            if element.get('name') in {'preview_view', 'preview_view_joint'}:
                exported.remove(element)
        exported.insert(0, ET.Comment(
            'Straight inner links + parallel pads. The wrist joints are proposed '
            'virtual articulations. Coupled motion requires adaptive_gripper_core; '
            'URDF alone does not enforce the closed loops. Visual kinematics only.'))
        save_xml(exported, root/f'urdf/{name}.urdf')
    model = AdaptiveGripper(geometry, calibration)
    for row in calibration['samples']:
        pose = model.pose(row['motor_angle_rad'])
        print(f"{pose['motor_rad']:+g} rad: opening={pose['reconstructed_opening_mm']:.6f} mm, nut depth={pose['nut_depth_mm']:.3f} mm, closure error={pose['closure_error_mm']:.3g} mm")
    print(f'Source CAD distal contact gap: {source_gap:.6f} mm; CAD upper-face reference nut depth: {reference_depth:.6f} mm (not physically verified)')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--package-root', type=Path, default=Path(__file__).resolve().parents[1])
    parser.add_argument('--calibration', type=Path)
    args = parser.parse_args()
    calibration = args.calibration or args.package_root.parent/'pallet_physical_moveit_config/config/gripper_calibration.json'
    generate(args.package_root, calibration)


if __name__ == '__main__':
    main()
