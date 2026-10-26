from pathlib import Path
from unittest.mock import patch,MagicMock
import math,json,time
import numpy as np
import pytest,yaml,rclpy
from rclpy.parameter import Parameter
from geometry_msgs.msg import Twist,TransformStamped
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import Imu,LaserScan,PointCloud2
from std_msgs.msg import String,Bool
from interfaces_pkg.msg import MotionCommand
from ebimu_driver.node import EbimuNode
from ebimu_driver.protocol import AsciiParser
from real_vehicle_bridge.adapter import Conversion,AdapterConfig,ControlAdapter
from real_vehicle_bridge.estimator import StateEstimator
from real_vehicle_bridge.lidar import normalize_scan,LidarGateway
from serial_communication_pkg.lib.protocol_convert_func_lib import convert_serial_message
W=Path(__file__).resolve().parents[1];GEOMETRY=W/'src/mvsim_slam_real_bringup/config/vehicle_geometry.yaml'

@pytest.fixture
def ros():
    rclpy.init();yield
    if rclpy.ok():rclpy.shutdown()

def params(**kwargs):return [Parameter(k,value=v) for k,v in kwargs.items()]
def jmsg(j):m=String();m.data=json.dumps(j);return m

def test_motion_message_protocol_and_adapter_units():
    assert MotionCommand.get_fields_and_field_types()=={'steering':'int32','left_speed':'int32','right_speed':'int32'}
    c=Conversion(AdapterConfig(.58,.45,.9,.14,.08775,1000.,-1.,1.,1.,255))
    s,l,r=c.convert(.5,.5*math.tan(.45)/.58,1.)
    assert (s,l,r)==(-7,140,140)
    s,l,r=c.convert(-.5,-.5*math.tan(.45)/.58,1.)
    assert (s,l,r)==(-7,-88,-88)
    assert c.convert(0.,0.,.1)==(-7,0,0)
    for v,w in [(0.,1.),(float('nan'),0.),(.1,float('inf'))]:
        with pytest.raises(ValueError):c.convert(v,w,.1)
    for s in [-7,0,7]:
        for speed in [-255,0,255]:assert len(convert_serial_message(s,speed,speed).rstrip('\n'))<=14
    assert convert_serial_message(-7,-255,-255)=='s-7l-255r-255\n'

def test_two_imu_ports_offline_and_axis_units_covariance(ros):
    with patch('serial.Serial',side_effect=AssertionError('hardware access forbidden')):
        a=EbimuNode(parameter_overrides=params(port='/dev/NOT_OPEN_TRACTOR',frame_id='tractor_imu_link',output_topic='/tractor/imu/data',mounting_rpy=[0.,0.,math.pi/2]))
        b=EbimuNode(parameter_overrides=params(port='/dev/NOT_OPEN_TRAILER',frame_id='trailer_imu_link',output_topic='/trailer/imu/data',publish_orientation=False))
        try:
            s=AsciiParser().feed(b'*0,0,0,180,0,0\r\n')[0];ma=a.message(s,123);mb=b.message(s,124)
            assert ma.header.frame_id=='tractor_imu_link' and mb.header.frame_id=='trailer_imu_link'
            assert a.pub.topic_name=='/tractor/imu/data' and b.pub.topic_name=='/trailer/imu/data'
            assert abs(ma.angular_velocity.y-math.pi)<1e-8 and abs(ma.angular_velocity.x)<1e-8
            assert ma.linear_acceleration_covariance[0]==-1 and mb.orientation_covariance[0]==-1
            assert abs(sum(getattr(ma.orientation,k)**2 for k in ['x','y','z','w'])-1)<1e-8
        finally:a.destroy_node();b.destroy_node()

def test_optional_real_acceleration_schema(ros):
    p=AsciiParser(include_acceleration=True);s=p.feed(b'*0,0,0,0,0,90,0,0,1\r\n')[0]
    a=EbimuNode(parameter_overrides=params(include_acceleration=True))
    try:
        m=a.message(s,1);assert m.linear_acceleration.z==9.80665 and m.linear_acceleration_covariance[0]>0
        assert AsciiParser().feed(b'*0,0,0,0,0,90,0,0,1\r\n')==[]
    finally:a.destroy_node()

def test_quaternion_message_units_mount_and_host_timestamp(ros):
    p=AsciiParser(orientation_format='quaternion',include_acceleration=True,include_timestamp=True)
    s=p.feed(b'*0.5,-0.5,0.5,0.5,180,0,0,0,0,1,42982\r\n')[0]
    a=EbimuNode(parameter_overrides=params(orientation_format='quaternion',include_acceleration=True,include_timestamp=True,mounting_rpy=[0.,0.,math.pi/2]))
    try:
        m=a.message(s,1234567890)
        from scipy.spatial.transform import Rotation
        q=(Rotation.from_quat([.5,-.5,.5,.5])*Rotation.from_euler('z',-math.pi/2)).as_quat()
        # q and -q represent the same attitude.
        assert abs(np.dot([m.orientation.x,m.orientation.y,m.orientation.z,m.orientation.w],q))==pytest.approx(1.)
        assert np.allclose([m.angular_velocity.x,m.angular_velocity.y,m.angular_velocity.z],[0,math.pi,0])
        assert m.linear_acceleration.z==9.80665 and m.linear_acceleration_covariance[0]>0
        assert (m.header.stamp.sec,m.header.stamp.nanosec)==(1,234567890)
    finally:a.destroy_node()

def test_scan_geometry_direction_timestamp_and_invalid_returns(ros):
    m=LaserScan();m.header.frame_id='raw';m.header.stamp=rclpy.clock.Clock().now().to_msg();m.angle_min=-1.;m.angle_max=1.;m.angle_increment=1.;m.range_min=.1;m.range_max=10.;m.ranges=[1.,2.,float('inf')]
    out=normalize_scan(m,'laser_frame',.2,True)
    assert list(out.ranges)==[float('inf'),2.,1.] and abs(out.angle_min+.8)<1e-6 and out.header.stamp==m.header.stamp
    m.time_increment=.01
    with pytest.raises(ValueError):normalize_scan(m,'laser_frame',reverse=True)
    m.time_increment=0.;gateway=LidarGateway();gateway.pub=MagicMock()
    try:
        gateway.receive(m);assert gateway.pub.publish.call_count==1
        gateway.receive(m);assert gateway.error=='scan timestamp stale/backward' and gateway.pub.publish.call_count==1
    finally:gateway.destroy_node()

def test_adapter_preview_timeout_estop_and_hardware_disabled(ros):
    a=ControlAdapter(parameter_overrides=params(geometry_file=str(GEOMETRY)));a.pub=MagicMock();a.preview=MagicMock()
    try:
        now=a.get_clock().now().nanoseconds;a.state(jmsg({'valid':True,'state':[0.,0.,0.,0.],'stamp_ns':now}))
        imu=Imu();imu.header.stamp=a.get_clock().now().to_msg();imu.angular_velocity_covariance[0]=.001;a.imu(imu)
        cmd=Twist();cmd.linear.x=.1;a.receive(cmd);a.tick();m=a.preview.publish.call_args.args[0]
        assert m.left_speed==100 and m.right_speed==100 and a.pub.publish.call_count==0
        a.last_cmd-=1.;a.tick();assert a.preview.publish.call_args.args[0].left_speed==0
        a.receive(cmd);a.emergency(Bool(data=True));a.tick();assert a.estop and a.preview.publish.call_args.args[0].left_speed==0
        a.emergency(Bool(data=False));assert a.estop
        cmd.linear.x=float('nan');a.receive(cmd);assert a.command is None
    finally:a.destroy_node()

def test_slam_rear_axle_articulation_dynamic_state_and_loss(ros,tmp_path):
    g=yaml.safe_load(GEOMETRY.read_text());g['initial_alignment_confirmed']=True;g['trailer_bias_confirmed']=True;g['initial_articulation_rad']=.1;f=tmp_path/'geometry.yaml';f.write_text(yaml.safe_dump(g))
    node=StateEstimator(parameter_overrides=params(geometry_file=str(f)));node.pub=MagicMock()
    try:
        now=node.get_clock().now().nanoseconds
        def stamp(m,ns):m.header.stamp.sec=ns//10**9;m.header.stamp.nanosec=ns%10**9
        for i in range(13):
            m=Imu();m.header.frame_id='trailer_imu_link';stamp(m,now-120_000_000+i*10_000_000);m.angular_velocity_covariance[0]=.001;node.trailer_imu(m)
        t=TransformStamped();t.header.frame_id='map';t.child_frame_id='base_footprint';stamp(t,now-20_000_000);t.transform.rotation.w=1.;node.buffer.set_transform(t,'offline fixture')
        m=LaserScan();m.header.frame_id='laser_frame';stamp(m,now);node.on_scan(m)
        p=PointCloud2();stamp(p,now);node.matched(p)
        grid=OccupancyGrid();grid.header.frame_id='map';stamp(grid,now);grid.info.width=10;grid.info.height=10;grid.info.resolution=.05;grid.info.origin.orientation.w=1.;grid.data=[100]+[0]*99;node.on_map(grid)
        with pytest.raises(ValueError,match='tractor IMU'):node.evaluate()
        front=Imu();front.header.frame_id='tractor_imu_link';stamp(front,now);front.angular_velocity_covariance[0]=.001;node.tractor_imu(front)
        state,ts=node.evaluate();assert np.allclose(state,[-.305,0.,0.,.1]) and ts==now-20_000_000
        node.tick();assert json.loads(node.pub.publish.call_args.args[0].data)['valid']
        node.imu_wall-=1.;node.tick();assert not json.loads(node.pub.publish.call_args.args[0].data)['valid']
    finally:node.destroy_node()

def test_tf_buffer_preserves_publication_rate_for_coherent_pairing(ros):
    from tf2_msgs.msg import TFMessage
    n=StateEstimator(parameter_overrides=params(geometry_file=str(GEOMETRY)))
    try:
        base=n.get_clock().now().nanoseconds
        for offset in [0,5_000_000,10_000_000]:
            a=TransformStamped();a.header.frame_id='map';a.child_frame_id='odom';a.header.stamp.sec=(base+offset)//10**9;a.header.stamp.nanosec=(base+offset)%10**9;a.transform.rotation.w=1.;a.transform.translation.x=1.
            b=TransformStamped();b.header=a.header;b.header.frame_id='odom';b.child_frame_id='base_footprint';b.transform.rotation.w=1.;b.transform.translation.x=2.
            # Avoid sharing/mutating the parent header object between TF edges.
            a.header.frame_id='map';b.header.frame_id='odom'
            import copy
            a.header=copy.deepcopy(a.header);a.header.frame_id='map'
            n.capture_tf(TFMessage(transforms=[a,b]))
        assert len(n.pose_history)==3
        assert all(np.allclose(p,[2.695,0.,0.]) for _,p in n.pose_history)
        gyro_ns=base+7_000_000
        available=[t for t,_ in n.pose_history if t<=gyro_ns and base+10_000_000-t<=30_000_000]
        assert available[-1]==base+5_000_000
    finally:n.destroy_node()

def test_partial_imu_timeout_drops_old_fragment_without_device(ros):
    n=EbimuNode();n.serial=MagicMock();n.pub=MagicMock();n.serial.in_waiting=20
    try:
        n.serial.read.return_value=b'*1,2,';n.poll();assert n.parser.buffer
        n.last_byte-=1.;n.serial.read.return_value=b'3,4,5,6\r\n';n.poll();assert not n.parser.buffer and n.pub.publish.call_count==0
        n.serial.read.return_value=b'*1,2,3,4,5,6\r\n';n.poll();assert n.pub.publish.call_count==1
    finally:n.destroy_node()

def test_rplidar_missing_bins_remain_unknown(ros):
    from real_vehicle_bridge.rplidar_source import RplidarSource
    n=RplidarSource();n.pub=MagicMock()
    try:
        n.q.put((n.get_clock().now().to_msg(),.1,[(10,0.,1000.)]));n.tick();scan=n.pub.publish.call_args.args[0]
        assert sum(math.isfinite(x) for x in scan.ranges)==1
        assert sum(math.isnan(x) for x in scan.ranges)==719 and not any(math.isinf(x) for x in scan.ranges)
    finally:n.destroy_node()
