from pathlib import Path
import sys
import pytest
sys.path.insert(0,str(Path(__file__).parents[1]/'scripts'))
from pallet_j5_response_compare import compare

class Clock:
    t=0.
    def now(self):return self.t
    def sleep(self,t):self.t+=t

@pytest.mark.parametrize('failure',[None,'feedback','stop'])
def test_fixed_trial_writes_target_once_and_all_exits_stop(failure):
    events=[];clock=Clock()
    class Backend:
        q=[.1,.2,.3,.4,.5,.6];stopped=False;commands=[]
        def sample(self):
            if failure=='feedback' and self.commands:raise RuntimeError('Timeout')
            return list(self.q)
        def prepare(self,start):assert start==self.q
        def command(self,q):self.commands.append(list(q));self.q=list(q)
        def stop(self):
            self.stopped=True
            if failure=='stop':raise RuntimeError('STOP missing')
    b=Backend()
    def log(row):events.append((dict(row),len(b.commands)))
    if failure:
        with pytest.raises(RuntimeError):compare(b,{'period_s':.1},log,clock.now,clock.sleep)
    else:
        result=compare(b,{'period_s':.1},log,clock.now,clock.sleep)
        assert all(r['goal_hold_passed'] for r in result)
        fixed=[count for row,count in events if row.get('mode')=='fixed' and row['event']=='trial_sample']
        assert set(fixed)=={1}
        assert all(q[:4]==[.1,.2,.3,.4] and q[5]==.6 for q in b.commands)
    assert b.stopped
    if failure=='stop':assert not any(row['event']=='stop_confirmed_all_6' for row,count in events)

@pytest.mark.parametrize('failure',[None,'bounds','tracking','hold','feedback'])
def test_visual_20deg_ramp_hold_and_failure_stop(failure):
    import math
    from types import SimpleNamespace as NS
    from pallet_j5_response_compare import visual_move
    clock=Clock();events=[]
    class Backend:
        q=[0.]*6
        commands=[]
        stopped=False
        def prepare(self,start):pass
        def sample(self):
            if failure=='feedback' and self.commands:raise RuntimeError('Timeout')
            q=list(self.q)
            if failure=='tracking' and clock.t>1:q[4]+=.02
            if failure=='hold' and clock.t>12:q[4]+=.002
            return q
        def command(self,q):self.q=list(q);self.commands.append((clock.t,list(q)))
        def stop(self):self.stopped=True
    b=Backend();joints=[NS(lower=-math.pi,upper=.1 if failure=='bounds' else math.pi) for _ in range(6)]
    config={'period_s':.1,'settle_timeout_s':3.,'max_path_error_rad':.01,'goal_tolerance_rad':.001}
    if failure:
        with pytest.raises((ValueError,RuntimeError)):
            visual_move(b,config,joints,events.append,clock.now,clock.sleep)
    else:
        visual_move(b,config,joints,events.append,clock.now,clock.sleep)
        assert clock.t>=15
        assert b.q[4]==pytest.approx(math.radians(20))
        assert all(q[:4]==[0.]*4 and q[5]==0 for _,q in b.commands)
        assert max(abs(b.commands[i][1][4]-b.commands[i-1][1][4]) for i in range(1,len(b.commands)))<.006
        assert any(e['event']=='visual_result' and e['held_s']>=5 for e in events)
    assert b.stopped
    if failure=='bounds':assert not b.commands

def test_initial_feedback_failure_logged_even_when_stop_also_fails():
    from pallet_j5_response_compare import visual_move
    events=[]
    class Backend:
        def sample(self):raise RuntimeError('J1: feedback timeout')
        def stop(self):raise RuntimeError('STOP not confirmed: J1')
    with pytest.raises(RuntimeError,match='STOP not confirmed'):
        visual_move(Backend(),{},[],events.append)
    assert events==[{'event':'operation_failure','reason':'J1: feedback timeout'}]


@pytest.mark.parametrize('cleanup_fails',[False,True])
def test_main_cleanup_logs_before_file_closes(monkeypatch,tmp_path,cleanup_fails):
    import json
    import pallet_j5_response_compare as tool
    path=tmp_path/'run.jsonl';closed=[]
    monkeypatch.setattr(sys,'argv',['test','--visual-20deg','--j5-pp-slow','--live','--arm-supported','--log',str(path)])
    monkeypatch.setattr(tool,'get_package_share_directory',lambda _:str(tmp_path))
    monkeypatch.setattr(tool,'load_config',lambda *a,**k:({},[]))
    class Backend:
        profile_changed=True
        def __init__(self,config,joints,log):self.log=log
        def stop(self):
            if cleanup_fails:raise RuntimeError('cleanup STOP missing')
            self.profile_changed=False
            self.log({'event':'J5_profile_restore_transmitted'})
        def close(self):closed.append(True)
    monkeypatch.setattr(tool,'SlowJ5ProfileBackend',Backend)
    def fail(*args):raise RuntimeError('initial feedback timeout')
    monkeypatch.setattr(tool,'visual_move',fail)
    with pytest.raises(SystemExit) as error:tool.main()
    assert error.value.code==1
    events=[json.loads(line) for line in path.read_text().splitlines()]
    assert any(e['event']=='failure' and e['reason']=='initial feedback timeout' for e in events)
    expected='cleanup_failure' if cleanup_fails else 'J5_profile_restore_transmitted'
    assert any(e['event']==expected for e in events)
    assert closed==[True]


@pytest.mark.parametrize('failure',[None,'partial_write','stop'])
def test_slow_profile_restore_order_and_partial_failure(monkeypatch,failure):
    from types import SimpleNamespace as NS
    from single_axis_position_core import Limits
    from pallet_j5_response_compare import SlowJ5ProfileBackend,SerialBackend
    events=[];logs=[]
    class Axis:
        joint=NS(name='J5',ratio=1.)
        limits=Limits(None,None,1.,1.,.37,.001)
        failed=False
        def write(self,index,value,fmt):
            events.append(('write',index,value))
            if failure=='partial_write' and value==.1 and not self.failed:
                self.failed=True;raise RuntimeError('partial')
    axis=Axis()
    def init(self,*args):self.axes=[None]*4+[axis]
    def stop(self):
        events.append(('stop',))
        if failure=='stop':raise RuntimeError('no STOP ACK')
    monkeypatch.setattr(SerialBackend,'__init__',init)
    monkeypatch.setattr(SerialBackend,'stop',stop)
    monkeypatch.setattr(SerialBackend,'prepare',lambda *a:events.append(('enable',)))
    b=SlowJ5ProfileBackend({'motor_profile_policy':'preserve_existing'},[],logs.append)
    if failure:
        with pytest.raises(RuntimeError):b.prepare([0.]*6)
    else:b.prepare([0.]*6)
    if failure=='stop':
        assert events==[('stop',)]
        assert not b.profile_changed
    else:
        b.stop()
        assert events[-3:]==[('stop',),('write',0x7024,10.),('write',0x7025,10.)]
        assert not b.profile_changed
        assert logs[-1]['readback_verified'] is False
        if failure is None:assert events.index(('enable',))>events.index(('write',0x7025,.1))
