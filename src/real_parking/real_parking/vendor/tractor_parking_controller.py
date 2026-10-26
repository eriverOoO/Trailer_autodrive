"""Forward finite-horizon and stabilized reverse feedback for the 4D rig."""
import math
import time
import numpy as np
from tractor_trailer_model import step,wrap,derivative
from scipy.linalg import solve_discrete_are


class Controller:
    def __init__(self,path,geometry,speed_factor=1.0,completion_hinge_deg=6.25):
        self.completion_hinge_deg=float(completion_hinge_deg)
        self.speed_factor=float(speed_factor)
        if not 1.0<=self.speed_factor<=2.0:raise ValueError("Unvalidated speed factor outside1..2")
        self.path=path;self.geometry=geometry;self.index=0;self.last_delta=0.;self.hold_until=0.
        changes=np.where(np.diff(path.gears)!=0)[0]
        self.ends=[int(i) for i in changes]+[len(path.states)-1];self.segment=0
        self.cumulative=np.r_[0,np.cumsum(np.linalg.norm(np.diff(path.states[:,:2],axis=0),axis=1))]
        self.last_reference=path.states[0];self.deviation=0.

    def reference(self,arc,end):
        j=min(end,max(0,int(np.searchsorted(self.cumulative,arc))))
        if j==0:return self.path.states[0].copy(),float(self.path.steering[0])
        low=j-1;den=self.cumulative[j]-self.cumulative[low]
        t=np.clip((arc-self.cumulative[low])/max(den,1e-9),0,1)
        dif=self.path.states[j]-self.path.states[low];dif[2:]=wrap(dif[2:])
        return self.path.states[low]+t*dif,float(self.path.steering[j])

    def command(self,state,dt=.1):
        self.terms={}
        end=self.ends[self.segment];lower=self.ends[self.segment-1]+1 if self.segment else 0
        lo=max(lower,self.index-3);hi=min(end+1,self.index+35)
        candidates=self.path.states[lo:hi]
        errors=candidates-state;errors[:,2:]=wrap(errors[:,2:])
        scores=np.linalg.norm(errors[:,:2],axis=1)+.20*np.linalg.norm(errors[:,2:],axis=1)
        self.index=max(self.index,lo+int(np.argmin(scores)))
        self.deviation=float(np.linalg.norm(self.path.states[self.index,:2]-state[:2]))
        remaining=max(0.,self.cumulative[end]-self.cumulative[self.index])
        endpoint=self.path.states[end];end_distance=float(np.linalg.norm(endpoint[:2]-state[:2]))
        # Short shunts need most of their distance to realize both headings.
        # A bounded endpoint-plane crossing handles delayed pose samples.
        final=self.segment==len(self.ends)-1
        tangent=endpoint[:2]-self.path.states[max(0,end-1),:2]
        tangent=tangent/max(np.linalg.norm(tangent),1e-9)
        offset=state[:2]-endpoint[:2]
        lateral=abs(tangent[0]*offset[1]-tangent[1]*offset[0])
        passed=0.<=float(offset@tangent)<.12 and lateral<.08
        segment_distance=self.cumulative[end]-self.cumulative[lower]
        tolerance=.35 if final else (.03 if segment_distance<.4 else .05)
        at_end=end_distance<tolerance or (not final and passed)
        heading_error=float(np.max(abs(wrap(state[2:]-endpoint[2:]))))
        aligned=heading_error<math.radians(5) and abs(wrap(state[3]-state[2]))<math.radians(self.completion_hinge_deg)
        if at_end and remaining<(.35 if final else .18) and (not final or aligned):
            if self.segment<len(self.ends)-1:
                self.segment+=1;self.index=end+1;self.hold_until=time.monotonic()+1.;return 0.,0.,False
            return 0.,0.,True
        if time.monotonic()<self.hold_until:return 0.,0.,False
        gear=int(self.path.gears[self.index]);v=.05*self.speed_factor*gear
        if remaining<.65:v=gear*max(.05,min(abs(v),remaining*.5))
        if abs(wrap(state[3]-state[2]))>math.radians(30):v*=.6
        # Project continuously on the current reference edge.
        arc=self.cumulative[self.index]
        if self.index<end:
            tangent=self.path.states[self.index+1,:2]-self.path.states[self.index,:2]
            length=np.linalg.norm(tangent)
            if length>1e-8:arc+=np.clip(np.dot(state[:2]-self.path.states[self.index,:2],tangent)/length,0,length)
        ref,feed=self.reference(arc,end);self.last_reference=ref
        if gear<0:
            # Fixed-speed reverse motion regulates cross-track and both headings.
            # A stabilizing Riccati solution avoids the short-horizon common
            # heading drift while leaving longitudinal progress to projection.
            h=.2;t0,t1=ref[2:];beta=t0-t1;w=v*math.tan(feed)/self.geometry.wheelbase
            db=(v*math.cos(beta)+self.geometry.hitch_offset*w*math.sin(beta))/self.geometry.trailer_length
            dw=v/(self.geometry.wheelbase*math.cos(feed)**2)
            A=np.array([[1.,h*v,0.],[0.,1.,0.],[0.,h*db,1.-h*db]])
            B=np.array([0.,h*dw,-h*self.geometry.hitch_offset*dw*math.cos(beta)/self.geometry.trailer_length])[:,None]
            Q=np.diag([7.,4.,10.]);R=np.array([[.7]])
            P=solve_discrete_are(A,B,Q,R)
            K=np.linalg.solve(R+B.T@P@B,B.T@P@A)
            error=state-ref;error[2:]=wrap(error[2:])
            cross=-math.sin(ref[2])*error[0]+math.cos(ref[2])*error[1]
            delta=float(feed-(K@np.r_[cross,error[2:]])[0])
        else:
            h=.2;N=25;Q=np.diag([7.,7.,4.,10.]);P=Q*3;R=.7
            gains=[]
            refs=[]
            for k in range(N):refs.append(self.reference(min(self.cumulative[end],arc+abs(v)*h*k),end))
            for r,d in refs[::-1]:
                t0,t1=r[2:];beta=t0-t1;w=v*math.tan(d)/self.geometry.wheelbase
                A=np.eye(4);A[0,2]=-h*v*math.sin(t0);A[1,2]=h*v*math.cos(t0)
                db=(v*math.cos(beta)+self.geometry.hitch_offset*w*math.sin(beta))/self.geometry.trailer_length
                A[3,2]=h*db;A[3,3]-=h*db
                dw=v/(self.geometry.wheelbase*math.cos(d)**2)
                B=np.array([0.,0.,h*dw,-h*self.geometry.hitch_offset*dw*math.cos(beta)/self.geometry.trailer_length])[:,None]
                K=(B.T@P@A)/(R+(B.T@P@B)[0,0]);P=Q+A.T@P@(A-B@K);gains.append(K)
            error=state-ref;error[2:]=wrap(error[2:])
            delta=float(feed-(gains[-1]@error)[0])
        self.terms=dict(feed_deg=math.degrees(feed),lqr_delta_deg=math.degrees(delta))
        before_articulation=delta
        # Passive-trailer yaw can depart from the kinematic prediction at very
        # low speed. Stabilize excessive reverse articulation directly, while
        # retaining the independent 30/35-degree planning/emergency limits.
        phi=wrap(state[3]-state[2])
        desired_phi=float(np.clip(wrap(ref[3]-ref[2]),-math.radians(27),math.radians(27)))
        phi_error=wrap(phi-desired_phi)
        if gear<0 and abs(phi)>math.radians(27) and phi*phi_error>0:
            delta-=1.5*phi_error
        self.terms.update(articulation_term_deg=math.degrees(delta-before_articulation),preclip_deg=math.degrees(delta))
        delta=float(np.clip(delta,-self.geometry.max_steer,self.geometry.max_steer))
        delta=float(np.clip(delta,self.last_delta-.9*dt,self.last_delta+.9*dt));self.last_delta=delta
        return v,delta,False
