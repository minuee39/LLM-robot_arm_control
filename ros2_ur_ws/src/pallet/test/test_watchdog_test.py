from pathlib import Path
from types import SimpleNamespace
import sys
import pytest
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts'))
from pallet_watchdog_test import read_state_reply


@pytest.mark.parametrize('state,fault',[(2,0),(0,1)])
def test_status_reply_decoding(state,fault):
    wrong=SimpleNamespace(arbitration_id=(2<<24)|(state<<22)|(fault<<16)|(5<<8)|253)
    correct=SimpleNamespace(arbitration_id=(2<<24)|(state<<22)|(fault<<16)|(6<<8)|253)
    queue=[wrong,correct]
    bus=SimpleNamespace(mid=6,host=253,transport=SimpleNamespace(recv=lambda timeout:queue.pop(0) if queue else None))
    assert read_state_reply(bus)=={'state':state,'fault_bits':fault}


def test_missing_reply_never_passes():
    bus=SimpleNamespace(mid=6,host=253,transport=SimpleNamespace(recv=lambda timeout:None))
    with pytest.raises(RuntimeError,match='UNVERIFIED'):read_state_reply(bus,.001)


def test_passive_watch_never_transmits_and_detects_reset():
    from pallet_watchdog_test import passive_watch
    clock=[0.]
    def recv(timeout):
        clock[0]+=.01
        state=2 if clock[0]<.25 else 0
        return SimpleNamespace(arbitration_id=(2<<24)|(state<<22)|(6<<8)|253)
    bus=SimpleNamespace(mid=6,host=253,transport=SimpleNamespace(recv=recv))
    result=passive_watch(bus,clock=lambda:clock[0])
    assert .24<=result['first_reset_s']<=.27
    assert result['last_status']['state']==0


def test_passive_running_state_is_failure():
    from pallet_watchdog_test import passive_watch
    clock=[0.]
    def recv(timeout):
        clock[0]+=.01
        return SimpleNamespace(arbitration_id=(2<<24)|(2<<22)|(6<<8)|253)
    bus=SimpleNamespace(mid=6,host=253,transport=SimpleNamespace(recv=recv))
    with pytest.raises(RuntimeError,match='UNVERIFIED'):
        passive_watch(bus,clock=lambda:clock[0])


def test_real_type24_capture_decodes_stopped_state():
    # Observed on user's ID6: arbitration ID 0x180006fd, 8-byte status data.
    message=SimpleNamespace(arbitration_id=0x180006fd,data=bytes.fromhex('841e7ff77fff0168'))
    bus=SimpleNamespace(mid=6,host=253,transport=SimpleNamespace(recv=lambda timeout:message))
    assert read_state_reply(bus)=={'state':0,'fault_bits':0}


def test_type24_passive_reset_transition():
    from pallet_watchdog_test import passive_watch
    clock=[0.]
    def recv(timeout):
        clock[0]+=.01
        state=2 if clock[0]<.25 else 0
        return SimpleNamespace(arbitration_id=(24<<24)|(state<<22)|(6<<8)|253)
    bus=SimpleNamespace(mid=6,host=253,transport=SimpleNamespace(recv=recv))
    assert passive_watch(bus,clock=lambda:clock[0])['last_status']['state']==0


def test_old_stopped_report_does_not_mask_enable_reply():
    queue=[SimpleNamespace(arbitration_id=0x180006fd),SimpleNamespace(arbitration_id=(2<<24)|(2<<22)|(6<<8)|253)]
    bus=SimpleNamespace(mid=6,host=253,transport=SimpleNamespace(recv=lambda timeout:queue.pop(0) if queue else None))
    assert read_state_reply(bus,expected_state=2)['state']==2
