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
