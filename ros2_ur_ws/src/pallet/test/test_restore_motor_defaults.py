from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).parents[1]/'scripts'))
from pallet_restore_motor_defaults import restore


def test_restore_stops_all_before_writes_and_preserves_gripper():
    class Bus:
        mid=1;targets={str(i):float(i+10) for i in range(1,7)}
        values={str(i):5.5 for i in range(1,8)};calls=[]
        def stop_and_confirm(self):self.calls.append(('stop',self.mid))
        def read(self,index,*args):return self.values[str(self.mid)]
        def write(self,index,value):
            assert self.calls[:7]==[('stop',i) for i in range(1,8)]
            self.calls.append(('write',self.mid,index,value))
            if index==0x7018:self.values[str(self.mid)]=value
    b=Bus();report={};restore(b,report,lambda:None)
    assert report['complete'] and report['all_stop_confirmed']
    assert report['verified_current_a']['7']==5.5
    writes=[c for c in b.calls if c[0]=='write']
    assert len(writes)==21
    assert {c[2] for c in writes}=={0x7018,0x7024,0x7025}
    assert all(c[3]==10. for c in writes if c[2] in (0x7024,0x7025))
