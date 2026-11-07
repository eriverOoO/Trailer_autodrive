import math
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped
from scipy.spatial.transform import Rotation
from .geometry import VehicleGeometry
from .common import spin

class SensorTf(Node):
    def __init__(self,**kwargs):
        super().__init__('sensor_tf',**kwargs);self.declare_parameter('geometry_file','');g=VehicleGeometry(self.get_parameter('geometry_file').value).values
        self.broadcaster=StaticTransformBroadcaster(self);out=[]
        for parent,child,values in [('base_footprint','base_link',[0.]*6),('base_footprint','laser_frame',g['lidar_mounting_offset']),('base_footprint','tractor_imu_link',g['tractor_imu_offset']),('trailer_link','trailer_imu_link',g['trailer_imu_offset']),('map','local_map',[0.]*6)]:
            t=TransformStamped();t.header.stamp=self.get_clock().now().to_msg();t.header.frame_id=parent;t.child_frame_id=child;t.transform.translation.x=float(values[0]);t.transform.translation.y=float(values[1]);t.transform.translation.z=float(values[2]);q=Rotation.from_euler('xyz',values[3:]).as_quat();t.transform.rotation.x=float(q[0]);t.transform.rotation.y=float(q[1]);t.transform.rotation.z=float(q[2]);t.transform.rotation.w=float(q[3]);out.append(t)
        self.broadcaster.sendTransform(out)

def main(args=None):spin(SensorTf,args)
