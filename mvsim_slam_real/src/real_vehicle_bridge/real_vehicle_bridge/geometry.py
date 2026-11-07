import math
import numpy as np
import yaml

class VehicleGeometry:
    def __init__(self,filename):
        with open(filename,encoding='utf8') as f:self.values=yaml.safe_load(f)
        g=self.values
        for name in ['tractor_length','tractor_width','trailer_length','trailer_width','tractor_wheelbase','hitch_to_axle_distance','maximum_steering_angle','steering_rate_limit','forward_velocity','reverse_velocity']:
            if not math.isfinite(g[name]) or g[name]<=0:raise ValueError('invalid '+name)
        if g['maximum_steering_angle']>=math.pi/2:raise ValueError('maximum steering must be below pi/2')
        for name in ['tractor_rear_axle_position','tractor_steering_axis','trailer_axle_position','hitch_position']:
            if len(g[name])!=3 or not np.isfinite(g[name]).all():raise ValueError('invalid '+name)
            if abs(g[name][1])>1e-8:raise ValueError('Final B model requires centered rear axle/hitch/steering/trailer axle')
        for name in ['lidar_mounting_offset','tractor_imu_offset','trailer_imu_offset']:
            if len(g[name])!=6 or not np.isfinite(g[name]).all():raise ValueError('invalid '+name)
        if abs(g['tractor_steering_axis'][0]-g['tractor_rear_axle_position'][0]-g['tractor_wheelbase'])>1e-6:raise ValueError('wheelbase and axle locations disagree')
        self.hitch_offset=g['tractor_rear_axle_position'][0]-g['hitch_position'][0]
        if self.hitch_offset<0:raise ValueError('Final B requires hitch aft of rear axle')
        # Connection is the straight body-to-body gap, split independently at
        # the hinge. Legacy files without either clearance retain equal halves.
        gap = g['connection_length']
        keys = ('tractor_rear_to_hitch_distance', 'trailer_front_to_hitch_distance')
        if any(k in g for k in keys) and not all(k in g for k in keys):
            raise ValueError('both hitch clearances are required')
        rear, front = (g.get(k, gap / 2) for k in keys)
        if not all(math.isfinite(v) and v >= 0 for v in (gap, rear, front)):
            raise ValueError('invalid hitch clearance/connection length')
        if abs(rear + front - gap) > 1e-6:
            raise ValueError('hitch clearances and connection length inconsistent')
        self.trailer_hitch_x=g['trailer_length']/2+front
        if abs(g['hitch_position'][0]+g['tractor_length']/2+rear)>1e-6:raise ValueError('hitch and tractor rear clearance inconsistent')
        if abs(self.trailer_hitch_x-g['trailer_axle_position'][0]-g['hitch_to_axle_distance'])>1e-6:raise ValueError('trailer hitch-to-axle distance inconsistent')
    def model(self):
        from tractor_trailer_model import Geometry
        g=self.values
        return Geometry(wheelbase=g['tractor_wheelbase'],hitch_offset=self.hitch_offset,trailer_length=g['hitch_to_axle_distance'],max_steer=g['maximum_steering_angle'])
    def polygons(self):
        g=self.values
        def rect(length,width,center):return np.array([[-length/2,-width/2],[length/2,-width/2],[length/2,width/2],[-length/2,width/2]])+center
        return rect(g['tractor_length'],g['tractor_width'],[-g['tractor_rear_axle_position'][0],0.]),rect(g['trailer_length'],g['trailer_width'],[-self.trailer_hitch_x,0.])
    def tractor_axle(self,center,yaw):
        p=self.values['tractor_rear_axle_position'];c,s=math.cos(yaw),math.sin(yaw)
        return np.asarray(center)+[c*p[0]-s*p[1],s*p[0]+c*p[1]]
    def trailer_center(self,state):
        x,y,t0,t1=state
        hitch=np.array([x,y])-self.hitch_offset*np.array([math.cos(t0),math.sin(t0)])
        return hitch-self.trailer_hitch_x*np.array([math.cos(t1),math.sin(t1)])
