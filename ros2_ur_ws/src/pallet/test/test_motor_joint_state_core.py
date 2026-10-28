import sys
import struct
from pathlib import Path
from types import SimpleNamespace
import yaml
import pytest
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts'))
from motor_joint_state_core import MotorJointMapping,ReadOnlyMotorBus,NAMES


def calibrated(tmp_path):
    cfg=yaml.safe_load((ROOT/'config/hand_guiding.yaml').read_text())
    for name,c in cfg['joints'].items():
        c.update(direction=-1 if name=='J4' else 1,zero_rad=1.,ratio=2.)
    p=tmp_path/'calibration.yaml';p.write_text(yaml.safe_dump(cfg))
    return p


def test_motor_to_urdf_and_mimic_source(tmp_path):
    m=MotorJointMapping(calibrated(tmp_path),ROOT/'urdf/pallet_with_gripper.urdf',True)
    q=m.convert({i:0.9 for i in m.ids})
    assert q[3]==pytest.approx(0.05)
    assert q[6]==pytest.approx(-0.05)
    assert q[0]==pytest.approx(-0.05)
    with pytest.raises(ValueError,match='complete'):m.convert({1:0.9})
    with pytest.raises(ValueError,match='non-finite'):m.convert(dict.fromkeys(m.ids,float('nan')))
    with pytest.raises(ValueError,match='outside'):m.convert(dict.fromkeys(m.ids,100.))


def test_uncalibrated_rejected(tmp_path):
    cfg=yaml.safe_load((ROOT/'config/hand_guiding.yaml').read_text())
    cfg['joints']['J1']['direction']=None
    path=tmp_path/'uncalibrated.yaml';path.write_text(yaml.safe_dump(cfg))
    with pytest.raises(ValueError,match='calibration required'):
        MotorJointMapping(path,ROOT/'urdf/pallet_with_gripper.urdf')


def test_arm_only_gripper_display_zero(tmp_path):
    m=MotorJointMapping(calibrated(tmp_path),ROOT/'urdf/pallet_with_gripper.urdf')
    assert m.ids==[1,2,3,4,5,6]
    assert m.convert(dict.fromkeys(m.ids,1.))==[0.]*7


def test_readonly_requests_and_host_filter():
    reader=object.__new__(ReadOnlyMotorBus);reader.host_id=253
    reader.can=SimpleNamespace(Message=lambda **kw:SimpleNamespace(**kw))
    sent=[];queue=[]
    def response(host,value):
        return SimpleNamespace(is_error_frame=False,is_extended_id=True,is_remote_frame=False,arbitration_id=(17<<24)|(2<<8)|host,data=b'\x19\x70'+bytes(2)+struct.pack('<f',value))
    def send(msg,timeout):
        sent.append(msg);queue.extend([response(252,99.),response(253,0.5)])
    reader.bus=SimpleNamespace(send=send,recv=lambda timeout:queue.pop(0) if queue else None)
    assert reader.read([2],.01)=={2:0.5}
    assert len(sent)==1
    assert sent[0].arbitration_id==(17<<24)|(253<<8)|2
    assert sent[0].data==b'\x19\x70'+bytes(6)
    assert sent[0].is_extended_id


def test_partial_feedback_retained():
    from motor_joint_state_core import MissingFeedbackError
    reader=object.__new__(ReadOnlyMotorBus);reader.host_id=253
    reader.can=SimpleNamespace(Message=lambda **kw:SimpleNamespace(**kw))
    queue=[]
    def send(msg,timeout):
        if msg.arbitration_id&255 == 1:
            queue.append(SimpleNamespace(is_error_frame=False,is_extended_id=True,is_remote_frame=False,arbitration_id=(17<<24)|(1<<8)|253,data=b'\x19\x70'+bytes(2)+struct.pack('<f',.25)))
    reader.bus=SimpleNamespace(send=send,recv=lambda timeout:queue.pop(0) if queue else None)
    with pytest.raises(MissingFeedbackError) as exc:
        reader.read([1,2],.002)
    assert exc.value.received=={1:.25}
    assert exc.value.missing==[2]


def test_position_error_frame_is_rejected():
    reader=object.__new__(ReadOnlyMotorBus);reader.host_id=253
    reader.can=SimpleNamespace(Message=lambda **kw:SimpleNamespace(**kw))
    queue=[]
    def send(msg,timeout):
        queue.append(SimpleNamespace(is_error_frame=False,is_extended_id=True,is_remote_frame=False,arbitration_id=0x110106fd,data=bytes.fromhex('1970000000000000')))
    reader.bus=SimpleNamespace(send=send,recv=lambda timeout:queue.pop(0) if queue else None)
    with pytest.raises(RuntimeError,match='Parameter read rejected'):reader.read([6],.1)
