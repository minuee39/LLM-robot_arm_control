"""RobStride CH340 AT transport, restricted to CAN parameter-read frames."""
import time
from types import SimpleNamespace
from motor_joint_state_core import ReadOnlyMotorBus, MissingFeedbackError


def encode_read(arbitration_id, data):
    if not 0 <= arbitration_id <= 0x1fffffff or (arbitration_id>>24)&31 != 17:
        raise ValueError('Read-only transport only permits parameter-read type 17')
    if len(data)!=8:
        raise ValueError('Expected eight payload bytes')
    return b'AT'+((arbitration_id<<3)|4).to_bytes(4,'big')+bytes([len(data)])+data+b'\r\n'


class ATParser:
    def __init__(self):
        self.buffer=bytearray()

    def feed(self,data):
        self.buffer.extend(data)
        frames=[]
        while True:
            start=self.buffer.find(b'AT')
            if start<0:
                self.buffer[:]=self.buffer[-1:] if self.buffer[-1:]==b'A' else b''
                break
            if start:del self.buffer[:start]
            if len(self.buffer)<7:break
            dlc=self.buffer[6]
            if dlc>8:
                del self.buffer[0];continue
            size=9+dlc
            if len(self.buffer)<size:break
            if self.buffer[size-2:size]!=b'\r\n':
                del self.buffer[0];continue
            encoded=int.from_bytes(self.buffer[2:6],'big')
            if encoded&7==4:
                frames.append(SimpleNamespace(arbitration_id=encoded>>3,data=bytes(self.buffer[7:7+dlc]),is_extended_id=True,is_remote_frame=False,is_error_frame=False))
            del self.buffer[:size]
        return frames


class SerialTransport:
    def __init__(self,port,baud):
        import serial
        # exclusive prevents another cooperating serial client from sharing the port.
        self.serial=serial.Serial(port,baudrate=baud,timeout=0,write_timeout=0.2,exclusive=True)
        self.parser=ATParser()
        self.queue=[]

    def send(self,msg,timeout):
        frame=encode_read(msg.arbitration_id,bytes(msg.data))
        if self.serial.write(frame)!=len(frame):
            raise RuntimeError('Incomplete serial write')

    def recv(self,timeout):
        deadline=time.monotonic()+timeout
        while True:
            if self.queue:return self.queue.pop(0)
            data=self.serial.read(min(4096,max(1,self.serial.in_waiting)))
            if data:self.queue.extend(self.parser.feed(data))
            if self.queue:return self.queue.pop(0)
            if time.monotonic()>=deadline:return None
            time.sleep(min(0.001,max(0,deadline-time.monotonic())))

    def shutdown(self):
        self.serial.close()


class ReadOnlySerialMotorBus(ReadOnlyMotorBus):
    def __init__(self,port='/dev/ttyUSB0',host_id=253,baud=921600):
        self.host_id=host_id
        self.can=SimpleNamespace(Message=lambda **kwargs:SimpleNamespace(**kwargs))
        self.bus=SerialTransport(port,baud)

    def read(self,ids,timeout):
        # CH340 adapter may drop a burst of requests. Wait for each reply before
        # sending the next request, within one overall cycle deadline.
        deadline=time.monotonic()+timeout
        result={}
        for mid in ids:
            remaining=deadline-time.monotonic()
            if remaining<=0:
                raise MissingFeedbackError(ids,result)
            try:
                result.update(super().read([mid],remaining))
            except MissingFeedbackError as exc:
                result.update(exc.received)
                raise MissingFeedbackError(ids,result) from exc
        return result
