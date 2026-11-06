import copy
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'scripts'))
from adaptive_gripper_core import AdaptiveGripper
from generate_adaptive_gripper import load_triangles
from render_adaptive_gripper import transforms


@pytest.fixture(scope='module')
def model():
    return AdaptiveGripper.load(ROOT/'config/adaptive_gripper/geometry.json')


@pytest.mark.parametrize('angle,opening,depth', [(-12,19,10),(0,38,6),(12,53,2)])
def test_measured_states(model, angle, opening, depth):
    pose = model.pose(angle)
    assert pose['reconstructed_opening_mm'] == pytest.approx(opening, abs=1e-6)
    assert pose['nut_depth_mm'] == depth
    assert pose['actuation'] is False


@pytest.mark.parametrize('constraint', ['parallel_pads','straight_inner','straight_parallel'])
def test_sweep_and_urdf_loop_closure(model,constraint):
    robot = ET.parse(ROOT/'urdf/adaptive_gripper_preview.urdf').getroot()
    previous = None
    for angle in np.linspace(-12,12,97):
        pose = model.pose(float(angle),constraint=constraint)
        assert pose['closure_error_mm'] < 1e-6
        tf = transforms(robot, pose['joints'])
        # Check omitted loop cuts via URDF forward kinematics, independently
        # from the solver's endpoint coordinates, for both sides.
        frame_inverse = np.linalg.inv(tf['adaptive_frame'])
        def point(link, xy):
            return (frame_inverse@tf[link]@np.r_[xy/1000,0,1])[:2]*1000
        tips = {}
        for side, data in model.geometry['sides'].items():
            p = {k:np.array(v['move_xy_mm']) for k,v in data['pins'].items()}
            a = point(f'{side}_helper_link',p['rocker_nut']-p['nut_pivot'])
            b = point(f'{side}_rocker_link',p['rocker_nut']-p['rocker_ground'])
            np.testing.assert_allclose(a,b,atol=1e-6)
            a = point(f'{side}_inner_upper_link',p['pad_inner']-p['inner_elbow'])
            b = point(f'{side}_pad_link',p['pad_inner']-p['pad_outer'])
            np.testing.assert_allclose(a,b,atol=1e-6)
            if constraint in ('straight_inner','straight_parallel'):
                d = point(f'{side}_inner_lower_link',np.zeros(2))
                f = point(f'{side}_inner_upper_link',np.zeros(2))
                e = point(f'{side}_inner_upper_link',p['pad_inner']-p['inner_elbow'])
                first, second = f-d, e-f
                cross = first[0]*second[1]-first[1]*second[0]
                assert abs(cross)/(np.linalg.norm(first)*np.linalg.norm(second)) < 1e-8
                assert np.dot(first, second) > 0
            if constraint in ('parallel_pads','straight_parallel'):
                # A source pad face edge, transformed through the full URDF,
                # must remain vertical in the frontal plane.
                pad_angle = data['parallel_pad_angle_rad']
                edge = np.array([np.sin(pad_angle),np.cos(pad_angle)])
                lower = point(f'{side}_wrist_link',np.zeros(2))
                upper = point(f'{side}_wrist_link',edge)
                assert abs(upper[0]-lower[0]) < 1e-8
            tips[side] = point(f'{side}_wrist_link',np.array(data['contact_datum_xy_mm'])-p['pad_inner'])
        assert tips['l'][0]-tips['r'][0] == pytest.approx(pose['opening_mm'],abs=1e-6)
        # Reject assembly branch flips between adjacent motor commands.
        current = np.array(list(pose['joints'].values()))
        if previous is not None:
            assert np.max(abs(current-previous)) < .1
        previous = current


@pytest.mark.parametrize('angle',[-12.01,12.01,float('nan'),float('inf'),True])
def test_invalid_commands(model,angle):
    with pytest.raises(ValueError):
        model.pose(angle)


def test_all_cad_triangles_preserved():
    for side in ['l','r']:
        original = load_triangles(ROOT/f'meshes/mass_revised/gripper_{side}.STL')
        assert sum(len(load_triangles(ROOT/f'meshes/adaptive_gripper/{side}_{i}.stl')) for i in range(7)) == len(original)


def test_bad_calibration_rejected(model):
    calibration = copy.deepcopy(model.geometry['calibration'])
    calibration['samples'][1]['nut_depth_mm'] = 20
    with pytest.raises(ValueError):
        AdaptiveGripper(model.geometry,calibration)


@pytest.mark.parametrize('side',['l','r'])
def test_virtual_wrist_mesh_cut_preserves_volume_and_bounds(model, side):
    pins = model.geometry['sides'][side]['pins']
    def mesh(name, pin):
        return load_triangles(ROOT/f'meshes/adaptive_gripper/{side}_{name}.stl')+np.r_[pins[pin]['move_xy_mm'],0]/1000
    original = mesh('0','pad_outer')
    pieces = [mesh('yoke','pad_outer'), mesh('contact_pad','pad_inner')]
    def volume(tri):
        return np.einsum('ij,ij->',tri[:,0],np.cross(tri[:,1],tri[:,2]))/6
    assert sum(volume(tri) for tri in pieces) == pytest.approx(volume(original),abs=2e-11)
    joined = np.concatenate(pieces)
    np.testing.assert_allclose(joined.min(axis=(0,1)), original.min(axis=(0,1)), atol=1e-8)
    np.testing.assert_allclose(joined.max(axis=(0,1)), original.max(axis=(0,1)), atol=1e-8)
