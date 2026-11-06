import sys
from pathlib import Path
import pytest
sys.path.insert(0,str(Path(__file__).parents[1]/'scripts'))
from pallet_position_preflight import ReadOnlyParameters,collect,REGISTERS


def test_read_only_rejects_all_actuation_and_write_only_registers():
    r=ReadOnlyParameters.__new__(ReadOnlyParameters)
    for kind in [3,4,6,18,22,24]:
        with pytest.raises(ValueError):r.send(kind,bytes(8))
    for index in [0x7024,0x7025,0x7006]:
        with pytest.raises(ValueError):r.send(17,index.to_bytes(2,'little')+bytes(6))


def test_collect_does_not_certify_observed_settings():
    class Reader:
        def read(self,index,fmt,timeout):return 5000 if index==0x7028 else 1
    result=collect(Reader(),1)
    assert result['complete'] and result['values']['watchdog_configured_s']==.25
    assert result['pp_watchdog_verified'] is False
    assert result['settings_modified'] is False


def test_incomplete_read_and_fault_are_not_hidden():
    class Reader:
        def read(self,index,fmt,timeout):raise RuntimeError(f'Feedback timeout: {hex(index)}')
    result=collect(Reader(),1)
    assert not result['complete'] and len(result['read_errors'])==len(REGISTERS)
    class Fault:
        def read(self,*args):raise RuntimeError('Motor fault frame received')
    with pytest.raises(RuntimeError,match='fault'):collect(Fault(),1)
