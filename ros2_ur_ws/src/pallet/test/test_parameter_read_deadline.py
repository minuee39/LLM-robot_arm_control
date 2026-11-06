from pathlib import Path
import struct
import sys
from types import SimpleNamespace as NS
import pytest
sys.path.insert(0,str(Path(__file__).parents[1]/'scripts'))
import single_axis_current_core as m


def bus(clock,send_s,reply_s,waits):
    def send(*args):clock[0]+=send_s
    def recv(timeout):
        if timeout==0:return None
        waits.append(timeout);clock[0]+=reply_s
        return NS(arbitration_id=(17<<24)|(1<<8)|253,
                  data=b'\x19\x70\0\0'+struct.pack('<f',.2))
    return NS(mid=1,host=253,send=send,check_fault=lambda _:None,transport=NS(recv=recv))


def test_request_pacing_consumes_reply_timeout(monkeypatch):
    clock=[0.];waits=[]
    monkeypatch.setattr(m,'time',NS(monotonic=lambda:clock[0]))
    value=m.CurrentTestBus.read(bus(clock,.003,.001,waits),0x7019,'<f',.015)
    assert value==pytest.approx(.2)
    assert waits==pytest.approx([.012])


def test_send_cannot_start_a_fresh_timeout_after_cycle_budget(monkeypatch):
    clock=[0.];waits=[]
    monkeypatch.setattr(m,'time',NS(monotonic=lambda:clock[0]))
    with pytest.raises(m.FeedbackTimeout):
        m.CurrentTestBus.read(bus(clock,.003,.001,waits),0x7019,'<f',.002)
    assert waits==[]


def test_reply_received_after_deadline_is_not_accepted(monkeypatch):
    clock=[0.];waits=[]
    monkeypatch.setattr(m,'time',NS(monotonic=lambda:clock[0]))
    with pytest.raises(m.FeedbackTimeout):
        m.CurrentTestBus.read(bus(clock,.003,.020,waits),0x7019,'<f',.015)
