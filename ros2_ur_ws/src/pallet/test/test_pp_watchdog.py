import sys
from pathlib import Path
from types import SimpleNamespace
import pytest
sys.path.insert(0,str(Path(__file__).parents[1]/'scripts'))
from pallet_pp_watchdog_test import observe_silence


def bus_for(state,fault=0):
    tick=[0.]
    def recv(timeout):
        tick[0]+=.01
        value=state(tick[0])
        return SimpleNamespace(arbitration_id=(24<<24)|(value<<22)|(fault<<16)|(1<<8)|253,data=bytes(8))
    bus=SimpleNamespace(mid=1,host=253,transport=SimpleNamespace(recv=recv))
    return bus,lambda:tick[0]


def test_receive_only_transition():
    bus,clock=bus_for(lambda t:2 if t<.25 else 0)
    result=observe_silence(bus,clock=clock)
    assert .24<=result['first_reset_s']<=.27
    # bus has no send method: observation cannot transmit.


@pytest.mark.parametrize('state',[lambda t:2,lambda t:0,lambda t:2 if t<.5 else 0,lambda t:0 if .25<t<.4 else 2])
def test_no_transition_or_wrong_timing_fails(state):
    bus,clock=bus_for(state)
    with pytest.raises(RuntimeError):observe_silence(bus,clock=clock)


def test_fault_not_accepted_as_success():
    bus,clock=bus_for(lambda t:2 if t<.25 else 0,1)
    with pytest.raises(RuntimeError,match='Fault'):observe_silence(bus,clock=clock)
