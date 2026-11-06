import sys
from pathlib import Path
import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'scripts'))
from hand_guiding_core import GravityModel, command


def test_gravity_matches_potential_gradient():
    model = GravityModel(ROOT/'urdf/pallet_with_gripper.urdf')
    for seed in range(4):
        q = dict(zip(model.names, np.random.default_rng(seed).uniform(-0.07, 0.07, len(model.names))))
        tau, _ = model.evaluate(q)
        for name in model.names:
            plus, minus = dict(q), dict(q)
            plus[name] += 1e-6
            minus[name] -= 1e-6
            derivative = (model.evaluate(plus)[1]-model.evaluate(minus)[1])/2e-6
            assert tau[name] == pytest.approx(derivative, abs=1e-7)


def test_bad_positions_rejected():
    model = GravityModel(ROOT/'urdf/pallet_with_gripper.urdf')
    with pytest.raises(ValueError):
        model.evaluate({})
    with pytest.raises(ValueError):
        model.evaluate(dict.fromkeys(model.names, float('nan')))


def test_current_mapping_and_limits():
    class Model:
        def evaluate(self, q, gravity):
            return {'j': 4.0}, 0
    c = {'j': dict(motor_id=1, direction=-1, zero_rad=1., ratio=2., damping=0.5, torque_limit_nm=5., current_limit_a=3., output_nm_per_a=1., lower_rad=-2., upper_rad=2., max_speed_rad_s=1.)}
    q, _, currents = command(Model(), c, {1:3.}, {'j':0.2})
    assert q['j'] == -1
    assert currents[1] == pytest.approx(-1.95)
    with pytest.raises(RuntimeError, match='Overspeed'):
        command(Model(), c, {1:3.}, {'j':2.})
    c['j']['current_limit_a'] = 0.1
    with pytest.raises(RuntimeError, match='current'):
        command(Model(), c, {1:3.}, {'j':0.})


def test_queued_fault_is_not_discarded():
    from types import SimpleNamespace
    from pallet_hand_guiding import FeedbackBus
    bus = object.__new__(FeedbackBus)
    frame = SimpleNamespace(is_error_frame=False, is_extended_id=True, arbitration_id=(2<<24)|(1<<16)|(1<<8))
    bus._bus = SimpleNamespace(recv=lambda timeout: frame)
    with pytest.raises(RuntimeError, match='Motor fault'):
        bus.positions([1], 0.01)
