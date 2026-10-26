import math
import pytest
from ebimu_driver.protocol import AsciiParser

def test_fragmented_protocol_and_unit_conversion():
    p=AsciiParser();assert p.feed(b'noise*10,20,')==[]
    s=p.feed(b'30,180,-90,45\r\n')[0]
    assert s.euler_rad==(math.radians(10),math.radians(20),math.radians(30))
    assert s.gyro_rad_s==(math.pi,-math.pi/2,math.pi/4)

def test_resync_corruption_and_channel_count():
    p=AsciiParser();out=p.feed(b'*cut*1,2,3,4,5,6\r\n*nan,2,3,4,5,6\r\n*1,2,3\r\n*1,2,3,4,5,6,7\r\n*1,91,3,4,5,6\r\n')
    assert len(out)==1 and p.rejected==5
    assert p.feed(b'*'+b'1'*300)==[] and len(p.buffer)==0

def test_exact_framing():
    p=AsciiParser();assert not p.feed(b'*1,2,3,4,5,6\n');assert len(p.feed(b'*1,2,3,4,5,6\r\n'))==1

def test_quaternion_timestamp_wire_order_and_fragmentation():
    p=AsciiParser(orientation_format='quaternion',include_acceleration=True,include_timestamp=True)
    assert p.feed(b'noise*0.5,-0.5,')==[]
    s=p.feed(b'0.5,0.5,180,-90,45,0,0,1,42982\r\n')[0]
    assert s.quaternion_xyzw==(0.5,-0.5,0.5,0.5)
    assert s.gyro_rad_s==(math.pi,-math.pi/2,math.pi/4)
    assert (s.ax,s.ay,s.az)==(0,0,1) and s.sensor_time_ms==42982

@pytest.mark.parametrize('frame',[
    b'*0,0,0,0,0,0,0,0,0,1,10\r\n',
    b'*0,0,0,2,0,0,0,0,0,1,10\r\n',
    b'*0,0,0,1,0,0,0,0,0,1,-10\r\n',
    b'*0,0,0,1,0,0,0,0,0,1,10.5\r\n',
    b'*0,0,0,1,0,0,nan,0,0,1,10\r\n',
    b'*0,0,0,1,0,0,0,0,0,1\r\n',
])
def test_quaternion_rejects_invalid_norm_timestamp_and_channels(frame):
    p=AsciiParser(orientation_format='quaternion',include_acceleration=True,include_timestamp=True)
    assert p.feed(frame)==[] and p.rejected==1
