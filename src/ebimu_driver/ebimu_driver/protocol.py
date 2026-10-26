"""EBIMU-9DOFV6 rev10: ASCII Euler or Z,Y,X,W quaternion, gyro DPS, accel g."""
from dataclasses import dataclass
import math

@dataclass(frozen=True)
class Sample:
    roll:float;pitch:float;yaw:float;gx:float;gy:float;gz:float
    ax:float=None;ay:float=None;az:float=None
    quaternion_xyzw:tuple=None
    sensor_time_ms:float=None
    @property
    def euler_rad(self):return tuple(math.radians(x) for x in (self.roll,self.pitch,self.yaw))
    @property
    def gyro_rad_s(self):return tuple(math.radians(x) for x in (self.gx,self.gy,self.gz))

class AsciiParser:
    def __init__(self,max_frame=256,include_acceleration=False,orientation_format='euler',include_timestamp=False):
        if orientation_format not in ('euler','quaternion'):raise ValueError('orientation_format must be euler or quaternion')
        self.orientation_format=orientation_format;self.include_timestamp=include_timestamp
        self.include_acceleration=include_acceleration;self.buffer=bytearray();self.max_frame=max_frame;self.accepted=0;self.rejected=0;self.discarded=0
    def feed(self,data):
        self.buffer.extend(data);out=[]
        while self.buffer:
            start=self.buffer.find(b'*')
            if start<0:self.discarded+=len(self.buffer);self.buffer.clear();break
            if start:self.discarded+=start;del self.buffer[:start]
            end=self.buffer.find(b'\n');restart=self.buffer.find(b'*',1)
            if restart>=0 and (end<0 or restart<end):self.rejected+=1;del self.buffer[:restart];continue
            if end<0:
                if len(self.buffer)>self.max_frame:self.rejected+=1;self.buffer.clear()
                break
            frame=bytes(self.buffer[:end+1]);del self.buffer[:end+1]
            try:
                if len(frame)>self.max_frame or not frame.endswith(b'\r\n'):raise ValueError('framing')
                parts=frame[1:-2].decode('ascii').split(',')
                orientation_count=4 if self.orientation_format=='quaternion' else 3
                expected=orientation_count+3+(3 if self.include_acceleration else 0)+int(self.include_timestamp)
                if len(parts)!=expected:raise ValueError('configured channel count mismatch')
                if any(not p.strip() for p in parts):raise ValueError('empty value')
                vals=tuple(float(p) for p in parts)
                if not all(math.isfinite(v) for v in vals):raise ValueError('nonfinite')
                quaternion=None
                if self.orientation_format=='quaternion':
                    z,y,x,w=vals[:4];norm=math.sqrt(x*x+y*y+z*z+w*w)
                    if abs(norm-1.)>0.05:raise ValueError('quaternion norm')
                    quaternion=tuple(v/norm for v in (x,y,z,w));euler=(0.,0.,0.)
                else:
                    if abs(vals[0])>180 or abs(vals[1])>90 or abs(vals[2])>180:raise ValueError('Euler range')
                    euler=vals[:3]
                gyro=vals[orientation_count:orientation_count+3]
                accel=vals[orientation_count+3:orientation_count+6] if self.include_acceleration else (None,None,None)
                sensor_time=vals[-1] if self.include_timestamp else None
                if sensor_time is not None and (sensor_time<0 or not sensor_time.is_integer()):raise ValueError('timestamp must be nonnegative integer milliseconds')
                out.append(Sample(*euler,*gyro,*accel,quaternion_xyzw=quaternion,sensor_time_ms=sensor_time));self.accepted+=1
            except (ValueError,UnicodeDecodeError):self.rejected+=1
        return out

QUERY_COMMANDS={'configuration':'<cfg>','version':'<ver>'}
# Explicit only. Extra channels OFF are necessary to guarantee six-field framing.
CONFIG_COMMANDS=('<sem0>','<soc1>','<sof1>','<sog1>','<soa0>','<som0>','<sod0>','<sot0>','<sots0>','<sor10>')
GYRO_CALIBRATION='<cg>'
