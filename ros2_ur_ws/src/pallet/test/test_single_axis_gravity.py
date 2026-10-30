import sys,math
from pathlib import Path
import pytest
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts'))
from single_axis_gravity_core import GravityLimits,output,rs05_reported_torque
from pallet_single_axis_gravity import live,calculate,J5Bus
from hand_guiding_core import GravityModel
from motor_joint_state_core import MotorJointMapping

def test_rs05_manual_scale():
    assert rs05_reported_torque(0)==-5.5
    assert rs05_reported_torque(65535)==5.5
    assert abs(rs05_reported_torque(32767))<.0001
    with pytest.raises(ValueError):rs05_reported_torque(-1)

def test_current_slew_and_clipping():
    l=GravityLimits();prev=0.
    for k in range(20):
        r=output(10.,0.,-1,1.,k*.05,prev,.05,l)
        assert abs(r['command_iq_a'])<=.03
        assert abs(r['command_iq_a']-prev)<=.003+1e-12
        prev=r['command_iq_a']
    assert r['current_clipped'] and prev<0

def test_gravity_zero_ramp_and_damping_sign():
    l=GravityLimits()
    assert output(10.,0.,1,1.,0.,0.,.05,l)['command_iq_a']==0
    assert output(0.,1.,1,1.,1.,0.,.05,l)['command_iq_a']<0
    assert output(0.,1.,-1,1.,1.,0.,.05,l)['command_iq_a']>0

@pytest.mark.parametrize('kw',[{'current_a':.1},{'gain':1.},{'travel_rad':1.},{'duration_s':10.},{'timeout_s':.05},{'gain':float('nan')}])
def test_bad_limits(kw):
    with pytest.raises(ValueError):GravityLimits(**kw).validate()

def test_fixed_axes_periodic_geometry_without_rezero():
    model=GravityModel(ROOT/'urdf/pallet_with_gripper.urdf')
    mapping=MotorJointMapping(ROOT/'config/hand_guiding.yaml',ROOT/'urdf/pallet_with_gripper.urdf',False)
    raw={1:0,2:-6.15,3:0,4:0,5:0,6:3.28}
    before=dict(raw)
    q,tau=calculate(model,mapping,raw,[0,0,-9.80665])
    raw2=dict(raw);raw2[2]+=2*math.pi;raw2[6]-=2*math.pi
    q2,tau2=calculate(model,mapping,raw2,[0,0,-9.80665])
    assert tau==pytest.approx(tau2)
    assert q['J2']==pytest.approx(q2['J2'])
    assert raw==before


def test_invalid_j5_pose_never_enabled_and_cleanup_independent():
    import yaml
    cfg=yaml.safe_load((ROOT/'config/hand_guiding.yaml').read_text())
    model=GravityModel(ROOT/'urdf/pallet_with_gripper.urdf')
    mapping=MotorJointMapping(ROOT/'config/hand_guiding.yaml',ROOT/'urdf/pallet_with_gripper.urdf',False)
    class Bus:
        def __init__(self):self.calls=[]
        def sample(self):return {1:0,2:0,3:0,4:0,5:6.15,6:0},0.,dict.fromkeys(range(1,7),1.)
        def send(self,*a):self.calls.append(a)
        def current(self,x):self.calls.append(('current',x));raise RuntimeError('zero write failed')
        def stop_and_confirm(self):self.calls.append(('stop',))
        def close(self):self.calls.append(('close',))
    b=Bus()
    with pytest.raises(RuntimeError,match='Cleanup incomplete'):
        live(b,model,mapping,cfg,GravityLimits(),lambda _:None)
    assert ('stop',) in b.calls and ('close',) in b.calls
    assert not any(c[0]==3 for c in b.calls)

def test_other_id_drive_prohibited():
    b=object.__new__(J5Bus);b.mid=4
    for kind in (3,4,18,24):
        with pytest.raises(ValueError,match='ID5'):b.send(kind)

@pytest.mark.parametrize('continuous,fail_sample',[(False,None),(True,None),(True,26)])
def test_runtime_and_continuous_cleanup(monkeypatch,continuous,fail_sample):
    import yaml
    import pallet_single_axis_gravity as controller
    cfg=yaml.safe_load((ROOT/'config/hand_guiding.yaml').read_text())
    mapping=MotorJointMapping(ROOT/'config/hand_guiding.yaml',ROOT/'urdf/pallet_with_gripper.urdf',False)
    model=GravityModel(ROOT/'urdf/pallet_with_gripper.urdf')
    now=[0.];rows=[]
    monkeypatch.setattr(controller,'prepare_current_mode',lambda *a:None)
    class Bus:
        report={}
        def __init__(self):self.calls=[];self.samples=0
        def sample(self):
            self.samples+=1;now[0]+=.001
            if self.samples==fail_sample:raise RuntimeError('Injected feedback timeout')
            return dict.fromkeys(range(1,7),0.),0.,dict.fromkeys(range(1,7),now[0])
        def send(self,*args):self.calls.append(('send',)+args)
        def current(self,x):self.calls.append(('current',x))
        def stop_and_confirm(self):self.calls.append(('stop',))
        def read(self,*args):return 0.
        def close(self):self.calls.append(('close',))
    b=Bus()
    def log(row):
        rows.append(row)
        if continuous and fail_sample is None and row.get('elapsed_s',0)>1.2:raise KeyboardInterrupt
    kwargs=dict(clock=lambda:now[0],sleep=lambda dt:now.__setitem__(0,now[0]+dt),continuous=continuous)
    if continuous:
        expected=RuntimeError if fail_sample else KeyboardInterrupt
        with pytest.raises(expected):live(b,model,mapping,cfg,GravityLimits(),log,**kwargs)
        assert max(r.get('elapsed_s',0) for r in rows)>1.
    else:
        live(b,model,mapping,cfg,GravityLimits(),log,**kwargs)
        assert max(r.get('elapsed_s',0) for r in rows)<1.05
    assert b.calls[-3:]==[('current',0.),('stop',),('close',)]
    assert rows[-1]['after']['fresh_reset_fault0']
    assert max(abs(c[1]) for c in b.calls if c[0]=='current')<=.03


def test_continuous_cli_requires_explicit_live_and_excludes_duration():
    import subprocess
    script=ROOT/'scripts/pallet_single_axis_gravity.py'
    for args in (['--continuous'],['--live','--continuous','--duration-s','1']):
        r=subprocess.run([sys.executable,str(script),*args],capture_output=True,text=True)
        assert r.returncode==2
