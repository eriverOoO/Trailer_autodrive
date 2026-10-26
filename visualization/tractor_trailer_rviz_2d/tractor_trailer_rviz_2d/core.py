"""Pure geometry and timestamped relative yaw; no ROS, serial, or control I/O."""
from collections import deque
import math


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def quat_mul(a, b):
    x,y,z,w = a
    u,v,t,s = b
    return (w*u+x*s+y*t-z*v, w*v-x*t+y*s+z*u,
            w*t+x*v-y*u+z*s, w*s-x*u-y*v-z*t)


def quat_inverse(q):
    return (-q[0],-q[1],-q[2],q[3])


def rpy_quat(r,p,y):
    cr,sr,cp,sp,cy,sy = math.cos(r/2),math.sin(r/2),math.cos(p/2),math.sin(p/2),math.cos(y/2),math.sin(y/2)
    return (sr*cp*cy-cr*sp*sy,cr*sp*cy+sr*cp*sy,cr*cp*sy-sr*sp*cy,cr*cp*cy+sr*sp*sy)


def yaw_quat(q):
    if len(q)!=4 or not all(math.isfinite(x) for x in q):
        raise ValueError('nonfinite quaternion')
    n=math.sqrt(sum(x*x for x in q))
    if abs(n-1)>.01:
        raise ValueError('quaternion not normalized')
    x,y,z,w=(x/n for x in q)
    return math.atan2(2*(w*z+x*y),1-2*(y*y+z*z))


def rotate(q,v):
    out=quat_mul(quat_mul(q,(*v,0.)),quat_inverse(q))
    return out[:3]


def rot2(yaw,v):
    c,s=math.cos(yaw),math.sin(yaw)
    return (c*v[0]-s*v[1],s*v[0]+c*v[1])


def add(a,b):
    return (a[0]+b[0],a[1]+b[1])


class YawTrack:
    def __init__(self, source='orientation', sign=1., mounting=(0.,0.,0.), bias=0., timeout=.5,
                 max_yaw_rate=None):
        if source not in ('orientation','gyro_z') or sign not in (-1,1):
            raise ValueError('explicit yaw source and sign required')
        if len(mounting)!=3 or not all(math.isfinite(x) for x in (*mounting,bias,timeout)) or timeout<=0:
            raise ValueError('invalid mounting/bias/timeout')
        self.source,self.sign,self.mount,self.bias,self.timeout=source,sign,rpy_quat(*mounting),bias,timeout
        self.history=deque(maxlen=4000)
        self.epoch=0
        self.last_stamp=self.last_wall=self.raw=self.value=self.rate=None
        self.error=''
        self.accepted=self.rejected=0
        if max_yaw_rate is not None and (not math.isfinite(max_yaw_rate) or max_yaw_rate<=0):
            raise ValueError('invalid maximum yaw rate')
        self.max_yaw_rate=max_yaw_rate

    def reject(self, reason):
        self.epoch+=1
        self.history.clear()
        self.raw=self.value=self.rate=None
        self.error=reason
        self.rejected+=1
        return False

    def push(self, stamp, wall, now_ns, q=None, gyro=None):
        if stamp<=0 or not -.05<=(now_ns-stamp)/1e9<=self.timeout:
            return self.reject('invalid or stale timestamp')
        if self.last_stamp is not None and stamp<=self.last_stamp:
            return self.reject('timestamp repeated or reversed')
        gap=self.last_stamp is not None and ((stamp-self.last_stamp)/1e9>self.timeout or wall-self.last_wall>self.timeout)
        if gap:
            self.epoch+=1
            self.history.clear()
            self.raw=self.value=self.rate=None
        try:
            if self.source=='orientation':
                if q is None:
                    raise ValueError('orientation unavailable; no gyro fallback')
                # mount = R_body_sensor; q = R_reference_sensor.
                raw=self.sign*yaw_quat(quat_mul(q,quat_inverse(self.mount)))
                if self.raw is not None and self.max_yaw_rate is not None:
                    dt=(stamp-self.last_stamp)/1e9
                    if abs(wrap(raw-self.raw))>self.max_yaw_rate*dt:
                        return self.reject('orientation jump exceeds maximum yaw rate')
                value=raw if self.raw is None else self.value+wrap(raw-self.raw)
                self.raw=raw
            else:
                if gyro is None or len(gyro)!=3 or not all(math.isfinite(v) for v in gyro):
                    raise ValueError('gyro unavailable')
                rate=self.sign*(rotate(self.mount,gyro)[2]-self.bias)
                value=0. if self.value is None else self.value+.5*(self.rate+rate)*(stamp-self.last_stamp)/1e9
                self.rate=rate
            self.value=value
            self.history.append((stamp,value))
            self.last_stamp,self.last_wall=stamp,wall
            self.error=''
            self.accepted+=1
            return True
        except (ValueError,TypeError) as exc:
            return self.reject(str(exc))

    def state(self, wall, now_ns):
        if self.error:
            return 'INVALID'
        if self.last_wall is None:
            return 'NO DATA'
        if wall-self.last_wall>self.timeout or (now_ns-self.last_stamp)/1e9>self.timeout:
            return 'STALE'
        return 'LIVE'

    def at(self, stamp):
        if not self.history or stamp<self.history[0][0] or stamp>self.history[-1][0]:
            raise ValueError('timestamp not bracketed; no extrapolation')
        prev=self.history[0]
        for nxt in self.history:
            if stamp==nxt[0]:return nxt[1]
            if nxt[0]>stamp:
                return prev[1]+(nxt[1]-prev[1])*(stamp-prev[0])/(nxt[0]-prev[0])
            prev=nxt
        raise ValueError('timestamp unavailable')


class Alignment:
    def __init__(self):
        self.anchor=None

    def align(self, a,b,stamp,map_yaw):
        self.anchor=(a.at(stamp),b.at(stamp),map_yaw,a.epoch,b.epoch,stamp)

    def evaluate(self,a,b,stamp):
        if self.anchor is None:
            raise ValueError('NOT ALIGNED: confirm stationary straight alignment')
        a0,b0,heading,ae,be,_=self.anchor
        if (ae,be)!=(a.epoch,b.epoch):
            raise ValueError('ALIGNMENT LOST: discontinuity; realign both sensors')
        da,db=a.at(stamp)-a0,b.at(stamp)-b0
        return dict(tractor_yaw=heading+da,trailer_yaw=heading+db,
                    tractor_delta=da,trailer_delta=db,phi=wrap(db-da))


def vehicle_poses(slam_xy,slam_yaw,tractor_yaw,trailer_yaw,geometry):
    # SLAM yaw is used only for the physical lever arm from the tracked point
    # to the tractor rear axle. Neither displayed body yaw comes from SLAM.
    axle=add(slam_xy,rot2(slam_yaw,geometry['slam_to_rear_axle_xy']))
    hitch=add(axle,rot2(tractor_yaw,geometry['rear_axle_to_hitch_xy']))
    th=rot2(trailer_yaw,geometry['trailer_origin_to_hitch_xy'])
    origin=(hitch[0]-th[0],hitch[1]-th[1])
    trailer_axle=add(origin,rot2(trailer_yaw,geometry['trailer_origin_to_axle_xy']))
    return dict(tractor=axle,hitch=hitch,trailer=origin,trailer_axle=trailer_axle)


def validate_geometry(g):
    for key in ('slam_to_rear_axle_xy','rear_axle_to_hitch_xy',
                'trailer_origin_to_hitch_xy','trailer_origin_to_axle_xy'):
        v=g.get(key)
        if not isinstance(v,list) or len(v)!=2 or not all(type(x) in (int,float) and math.isfinite(x) for x in v):
            raise ValueError('measured finite geometry required: '+key)
    if math.hypot(*g['rear_axle_to_hitch_xy'])<=0 or math.dist(g['trailer_origin_to_hitch_xy'],g['trailer_origin_to_axle_xy'])<=0:
        raise ValueError('nonzero hitch distances required')
    offset=g.get('slam_to_body_yaw_rad')
    if type(offset) not in (int,float) or not math.isfinite(offset):
        raise ValueError('measured finite slam_to_body_yaw_rad required')


class StationaryReference:
    """Averaged startup yaw reference; moving/late inputs restart the window."""
    def __init__(self, duration=5., min_samples=40, max_gap=.25,
                 position_span=.02, yaw_span=math.radians(1.)):
        if not all(math.isfinite(x) and x>0 for x in
                   (duration,min_samples,max_gap,position_span,yaw_span)):
            raise ValueError('invalid startup stability limits')
        self.duration,self.min_samples,self.max_gap=duration,min_samples,max_gap
        self.position_span,self.yaw_span=position_span,yaw_span
        self.window=deque()
        self.reason='waiting for stable inputs'
        self.progress=0.

    def clear(self, reason='waiting for stable inputs'):
        self.window.clear();self.progress=0.;self.reason=reason

    def push(self, stamp, wall, a, b, x, y, heading, stationary):
        if not all(math.isfinite(v) for v in (stamp,wall,a,b,x,y,heading)):
            self.clear('nonfinite startup sample');return None
        if not stationary:
            self.clear('hold both IMUs stationary');return None
        if self.window:
            prev=self.window[-1]
            if stamp==prev[0]:
                return None
            if stamp<prev[0] or wall<=prev[1] or max((stamp-prev[0])/1e9,wall-prev[1])>self.max_gap:
                self.clear('startup input gap or clock discontinuity')
        self.window.append((stamp,wall,a,b,x,y,heading))
        while len(self.window)>1 and self.window[1][0]<=stamp-self.duration*1e9:
            self.window.popleft()
        first=self.window[0]
        if (any(math.hypot(v[4]-first[4],v[5]-first[5])>self.position_span for v in self.window)
                or any(max(abs(v[2]-first[2]),abs(v[3]-first[3]),abs(wrap(v[6]-first[6])))>self.yaw_span
                       for v in self.window)):
            self.clear('vehicle/SLAM heading or position moving');return None
        span=min((stamp-first[0])/1e9,wall-first[1])
        self.progress=min(1.,span/self.duration)
        self.reason=f'hold still: {span:.1f}/{self.duration:.1f}s'
        if span+1e-9<self.duration or len(self.window)<self.min_samples:
            return None
        n=len(self.window)
        self.reason='READY'
        return (sum(v[2] for v in self.window)/n,
                sum(v[3] for v in self.window)/n,
                first[6]+sum(wrap(v[6]-first[6]) for v in self.window)/n)
