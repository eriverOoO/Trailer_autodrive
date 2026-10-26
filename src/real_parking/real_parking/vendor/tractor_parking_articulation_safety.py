"""Final-command reverse articulation constraint, independent of tracking costs.

No localization correction: the measured state remains authoritative. Command
history is used only to bound delayed-state safety prediction. All resulting
motion still requires the runtime RAW-map swept-footprint check.
"""
from collections import deque
import math
import numpy as np
from tractor_trailer_model import step,wrap


class ArticulationSafety:
    def __init__(self,geometry):
        self.geometry=geometry
        self.history=deque(maxlen=100)
        self.bound=math.radians(29.)
        self.horizon=1.5
        #028 measured outward residual max .7713deg/s (p95 .5319).
        self.rate_uncertainty=math.radians(.8)

    def record(self,now,v,delta):
        if self.history and now<self.history[-1][0]:self.history.clear()
        self.history.append((float(now),float(v),float(delta)))

    def predicted_now(self,state,stamp,now):
        p=np.asarray(state,dtype=float).copy();cursor=float(stamp);v=delta=0.
        for t,nv,nd in self.history:
            if t<=stamp:v,delta=nv,nd;continue
            if t>now:break
            p=step(p,v,delta,t-cursor,self.geometry);cursor=t;v,delta=nv,nd
        return step(p,v,delta,max(0.,now-cursor),self.geometry)

    def project(self,state,v,requested,stamp,now,dt=.1):
        info={'requested_delta_deg':math.degrees(requested),'intervened':False,'stop':False}
        if v>=0:return v,requested,info
        p=self.predicted_now(state,stamp,now)
        info['predicted_now_phi_deg']=math.degrees(wrap(p[2]-p[3]))
        last=self.history[-1][2] if self.history else 0.
        lo=max(-self.geometry.max_steer,last-.9*dt)
        hi=min(self.geometry.max_steer,last+.9*dt)
        # beta(t) under constant steering is scalar and order-preserving with
        # steering in this envelope. Test the entire prediction, not its end.
        def safe(delta):
            beta=float(wrap(p[2]-p[3]))
            uncertainty=self.rate_uncertainty*max(0.,now-stamp)
            bounds=np.array([beta-uncertainty,beta+uncertainty])
            peak=float(np.max(abs(bounds)))
            disturbance=np.array([-self.rate_uncertainty,self.rate_uncertainty])
            yaw_rate=v*math.tan(delta)/self.geometry.wheelbase
            def rates(b):
                return (yaw_rate*(1+self.geometry.hitch_offset*np.cos(b)/self.geometry.trailer_length)
                        -v*np.sin(b)/self.geometry.trailer_length+disturbance)
            # Scalar interval dynamics is order-preserving. The +/- disturbance
            # endpoints bound either sign of observed model-rate error.
            for _ in range(15):
                k1=rates(bounds);k2=rates(bounds+.05*k1)
                k3=rates(bounds+.05*k2);k4=rates(bounds+.1*k3)
                bounds+=.1*(k1+2*k2+2*k3+k4)/6
                peak=max(peak,float(np.max(abs(bounds))))
            return peak<=self.bound,peak
        requested=float(np.clip(requested,lo,hi))
        ok,peak=safe(requested)
        if ok:result=requested
        else:
            # Positive tractor-minus-trailer beta needs positive reverse steer.
            direction=1 if wrap(p[2]-p[3])>=0 else -1
            edge=hi if direction>0 else lo
            edge_ok,edge_peak=safe(edge)
            if not edge_ok:
                info.update(stop=True,intervened=True,predicted_peak_deg=math.degrees(edge_peak),reason='no rate-reachable reverse steering within recovery envelope')
                return 0.,0.,info
            unsafe=requested;result=edge
            for _ in range(16):
                middle=(unsafe+result)/2
                if safe(middle)[0]:result=middle
                else:unsafe=middle
            peak=safe(result)[1]
        info.update(intervened=abs(result-requested)>1e-7,predicted_peak_deg=math.degrees(peak),selected_delta_deg=math.degrees(result))
        return v,float(result),info
