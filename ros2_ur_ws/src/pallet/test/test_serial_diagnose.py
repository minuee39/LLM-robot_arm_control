from pathlib import Path
import sys
from types import SimpleNamespace as NS
import pytest
sys.path.insert(0,str(Path(__file__).parents[1]/'scripts'))
import pallet_serial_diagnose as diagnostic
from single_axis_current_core import FeedbackTimeout


JOINTS=[NS(name=f'J{mid}',motor_id=mid) for mid in range(1,7)]


class Clock:
    def __init__(self):self.now=0.
    def __call__(self):return self.now
    def sleep(self,seconds):self.now+=seconds


class Reader:
    # No stop, prepare, write, or enable methods exist on this fake.
    def __init__(self,clock,fail=None,trace=True):
        self.clock=clock;self.requests=[];self.fail=fail
        if trace:self.transport=NS(trace=None)
    def read(self,index,fmt,timeout):
        self.requests.append((self.mid,index,fmt,timeout,self._read_interval_s,self.clock()))
        if hasattr(self,'transport') and self.transport.trace:
            self.transport.trace({'event':'serial_tx','time_s':self.clock(),'hex':'4154'})
        self.clock.sleep(.001)
        if self.fail:self.fail(self.mid,index)
        return self.mid*.1


def test_isolated_axis_order_and_start_to_start_spacing():
    clock=Clock();reader=Reader(clock);rows=[]
    result=diagnostic.run_phase(reader,'isolated_position',JOINTS,3,rows.append,clock,clock.sleep)
    assert [(mid,index) for mid,index,*_ in reader.requests]==[(mid,0x7019) for mid in range(1,7) for _ in range(3)]
    assert [row[-1] for row in reader.requests]==pytest.approx([i*.1 for i in range(18)])
    assert all(row[2:5]==('<f',.05,.003) for row in reader.requests)
    assert result['complete_cycles']==18 and result['total_reads']==18 and result['timeouts']==0


@pytest.mark.parametrize('phase,ids,interval',[
    ('full_forward_3ms',range(1,7),.003),('full_reverse_3ms',range(6,0,-1),.003),
    ('full_forward_6ms',range(1,7),.006),
])
def test_full_phase_order_interval_and_cycle_spacing(phase,ids,interval):
    clock=Clock();reader=Reader(clock);rows=[]
    result=diagnostic.run_phase(reader,phase,JOINTS,3,rows.append,clock,clock.sleep)
    assert [(mid,index) for mid,index,*_ in reader.requests]==[(mid,index) for mid in ids for index in diagnostic.TELEMETRY]*3
    assert all(row[2:5]==('<f',.05,interval) for row in reader.requests)
    assert [reader.requests[i][-1] for i in (0,18,36)]==pytest.approx([0.,.25,.5])
    assert result['cycles']==3 and result['complete_cycles']==3 and result['total_reads']==54


def test_timeout_is_reported_once_without_retry_and_other_axes_continue():
    def fail(mid,index):
        if mid==4 and index==0x7019:raise FeedbackTimeout('Feedback timeout: 0x7019')
    clock=Clock();reader=Reader(clock,fail);rows=[]
    result=diagnostic.run_phase(reader,'full_forward_3ms',JOINTS,3,rows.append,clock,clock.sleep)
    assert len(reader.requests)==54 and result['complete_cycles']==0
    assert result['timeouts']==3
    assert result['timeouts_by_register']==[{'joint':'J4','register':'0x7019','count':3}]
    assert len([row for row in rows if row['event']=='diagnostic_timeout'])==3
    assert reader.requests[-1][:2]==(6,0x701b)


@pytest.mark.parametrize('error',[RuntimeError('J2: motor fault frame'),RuntimeError('Parameter read rejected'),
                                RuntimeError('Feedback timeout: not a typed timeout'),OSError('Serial disconnected')])
def test_critical_failures_propagate_without_continuing(error):
    def fail(mid,index):raise error
    clock=Clock();reader=Reader(clock,fail);rows=[]
    with pytest.raises(type(error),match=str(error)):
        diagnostic.diagnose(reader,JOINTS,3,rows.append,clock,clock.sleep)
    assert len(reader.requests)==1 and reader.transport.trace is None
    assert not any(row['event']=='diagnostic_summary' for row in rows)


def test_nonfinite_feedback_aborts():
    clock=Clock();reader=Reader(clock);reader.read=lambda *args:float('nan')
    with pytest.raises(RuntimeError,match='non-finite'):
        diagnostic.diagnose(reader,JOINTS,3,lambda row:None,clock,clock.sleep)


@pytest.mark.parametrize('trace',[True,False])
def test_complete_read_only_diagnostic_never_qualifies_motion_and_trace_has_request_context(trace):
    clock=Clock();reader=Reader(clock,trace=trace);rows=[]
    result=diagnostic.diagnose(reader,JOINTS,3,rows.append,clock,clock.sleep)
    assert result['all_reads_received'] and not result['qualified_for_motion']
    assert not result['actuation'] and not result['settings_modified']
    assert len(reader.requests)==18+54*3
    assert all(row[1] in diagnostic.TELEMETRY for row in reader.requests)
    assert [row['phase'] for row in result['phases']]==list(diagnostic.PHASES)
    raw=[row for row in rows if row['event']=='serial_tx']
    if trace:
        assert raw[0]['joint']=='J1' and raw[0]['register']=='0x7019' and raw[0]['phase']=='isolated_position'
        assert reader.transport.trace is None
    else:assert not raw


def test_offline_and_missing_output_never_open_serial(monkeypatch,capsys):
    monkeypatch.setattr(diagnostic,'get_package_share_directory',lambda package:str(Path(__file__).parents[1]))
    def forbidden(*args):raise AssertionError('Serial opened')
    monkeypatch.setattr(diagnostic,'ReadOnlyParameters',forbidden)
    diagnostic.main([])
    assert 'OFFLINE_NO_DEVICE_ACCESS' in capsys.readouterr().out
    with pytest.raises(SystemExit) as error:diagnostic.main(['--read'])
    assert error.value.code==2


@pytest.mark.parametrize('samples',[2,51,True])
def test_invalid_sample_count_rejected_without_reads(samples):
    clock=Clock();reader=Reader(clock)
    with pytest.raises(ValueError,match='3..50'):
        diagnostic.diagnose(reader,JOINTS,samples,lambda row:None,clock,clock.sleep)
    assert reader.requests==[]


@pytest.mark.parametrize('ids',[[1,2,3,4,5,7],[1,2,3,4,5,5],[1,2,3,4,5]])
def test_only_six_unique_arm_ids_are_allowed(ids):
    clock=Clock();reader=Reader(clock)
    joints=[NS(name=f'J{mid}',motor_id=mid) for mid in ids]
    with pytest.raises(ValueError,match='exactly arm motor IDs 1..6'):
        diagnostic.diagnose(reader,joints,3,lambda row:None,clock,clock.sleep)
    assert reader.requests==[]


def test_existing_output_file_is_preserved_and_device_stays_closed(tmp_path,monkeypatch):
    monkeypatch.setattr(diagnostic,'get_package_share_directory',lambda package:str(Path(__file__).parents[1]))
    path=tmp_path/'existing.jsonl';path.write_text('existing evidence\n')
    def forbidden(*args):raise AssertionError('Serial opened')
    monkeypatch.setattr(diagnostic,'ReadOnlyParameters',forbidden)
    with pytest.raises(SystemExit) as error:diagnostic.main(['--read','--output',str(path)])
    assert error.value.code==1 and path.read_text()=='existing evidence\n'
