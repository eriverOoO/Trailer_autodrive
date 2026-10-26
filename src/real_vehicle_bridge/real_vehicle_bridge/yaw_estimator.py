"""Gyro-only trailer direction: no orientation, GT, joint or command integration."""
import math,collections,numpy as np
class GyroYaw:
 def __init__(self):
  self.samples=[];self.bias=None;self.bias_sigma=None;self.last_ns=None;self.last_g=None;self.integral=0.;self.history=collections.deque(maxlen=30000);self.anchor=None;self.anchor_integral=None;self.anchor_ns=None;self.fault=None;self.count=0;self.orientation_used=False
 def push(self,ns,gyro,orientation_covariance):
  if orientation_covariance!=-1:self.fault='IMU orientation unexpectedly provided';return
  if not math.isfinite(gyro):self.fault='Nonfinite gyro';return
  if self.last_ns is not None:
   dt=(ns-self.last_ns)/1e9
   if dt<=0:self.fault='IMU timestamp duplicate/backward';return
   if dt>.030:self.fault='IMU sample gap';return
  else:dt=0
  if self.bias is None:
   if ns>10_000_000_000:self.fault='Bias initialization timeout';return
   if abs(gyro)>.003:self.fault='Initial stationary gyro bias calibration not stationary';return
   self.samples.append((ns,gyro))
   if len(self.samples)>=800 and ns-self.samples[0][0]>=8_000_000_000:
    vals=np.array([v for _,v in self.samples]);self.bias=float(vals.mean());self.bias_sigma=float(vals.std()/math.sqrt(len(vals)))
  else:
   # Samples are interval averages over the preceding physical steps.
   self.integral+=(gyro-self.bias)*dt
   self.history.append((ns,self.integral))
  self.last_ns=ns;self.last_g=gyro;self.count+=1
 def integral_at(self,ns):
  if not self.history:return None
  xs=np.array([v[0] for v in self.history],dtype=np.int64)
  if ns<xs[0] or ns>xs[-1]+15_000_000:return None
  return float(np.interp(ns,xs,[v[1] for v in self.history]))
 def initialize(self,ns,tractor_yaw):
  if self.anchor is not None:return True
  val=self.integral_at(ns)
  if self.bias is None or val is None:return False
  self.anchor=tractor_yaw;self.anchor_integral=val;self.anchor_ns=ns;return True
 def at(self,ns):
  val=self.integral_at(ns)
  if self.anchor is None or val is None or self.fault:return None
  return math.atan2(math.sin(self.anchor+val-self.anchor_integral),math.cos(self.anchor+val-self.anchor_integral))
