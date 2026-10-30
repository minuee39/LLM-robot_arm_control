import sys
from pathlib import Path
import pytest
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts'))
from robstride_serial_readonly import ATParser,encode_read


def test_known_read_frame():
    frame=encode_read(0x1100fd01,b'\x19\x70'+bytes(6))
    assert frame.hex()=='41548807e80c0819700000000000000d0a'
    parsed=ATParser().feed(frame)
    assert len(parsed)==1
    assert parsed[0].arbitration_id==0x1100fd01
    assert parsed[0].data==b'\x19\x70'+bytes(6)


def test_fragmentation_noise_and_embedded_delimiter():
    payload=b'\r\nAT1234'
    frame=encode_read(0x110001fd,payload)
    parser=ATParser();result=[]
    for byte in b'junkAT'+bytes([255])*7+frame+frame:
        result.extend(parser.feed(bytes([byte])))
    assert len(result)==2
    assert all(m.data==payload for m in result)


def test_control_commands_rejected():
    for command in (1,3,4,6,18):
        with pytest.raises(ValueError,match='Read-only'):
            encode_read((command<<24)|1,bytes(8))


def test_bad_tail_resynchronizes():
    frame=encode_read(0x110001fd,bytes(8))
    parser=ATParser()
    result=parser.feed(frame[:-2]+b'xx'+frame)
    assert len(result)==1


def test_serial_sequential_requests_share_deadline(monkeypatch):
    from robstride_serial_readonly import ReadOnlySerialMotorBus
    from motor_joint_state_core import ReadOnlyMotorBus
    calls=[]
    def fake_read(self,ids,timeout):
        calls.append((list(ids),timeout))
        return {ids[0]:ids[0]*.1}
    monkeypatch.setattr(ReadOnlyMotorBus,'read',fake_read)
    reader=object.__new__(ReadOnlySerialMotorBus)
    result=reader.read([1,2,3],.1)
    assert list(result)==[1,2,3]
    assert [ids for ids,_ in calls]==[[1],[2],[3]]
    assert 0<calls[-1][1]<=calls[0][1]<=.1


def test_serial_missing_reply_preserves_prior_results(monkeypatch):
    from robstride_serial_readonly import ReadOnlySerialMotorBus
    from motor_joint_state_core import ReadOnlyMotorBus,MissingFeedbackError
    def fake_read(self,ids,timeout):
        if ids==[2]:raise MissingFeedbackError(ids,{})
        return {1:.5}
    monkeypatch.setattr(ReadOnlyMotorBus,'read',fake_read)
    reader=object.__new__(ReadOnlySerialMotorBus)
    with pytest.raises(MissingFeedbackError) as exc:reader.read([1,2,3],.1)
    assert exc.value.received=={1:.5}
    assert exc.value.missing==[2,3]
