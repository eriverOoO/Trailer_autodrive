"""Explicit synthetic ROS integration check; NEVER connects physical devices."""
from pathlib import Path
import os,sys,subprocess,signal,time,json,math,yaml
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data,QoSProfile,DurabilityPolicy
from sensor_msgs.msg import LaserScan,Imu
from std_msgs.msg import String
from interfaces_pkg.msg import MotionCommand
from nav_msgs.msg import OccupancyGrid,Path as RosPath
W=Path(__file__).resolve().parents[1]

class Probe(Node):
    def __init__(self,synthetic):
        super().__init__('offline_verification_probe');self.states=[];self.status=[];self.preview=[];self.hardware=[];self.maps=0;self.paths=0;self.synthetic=synthetic;self.started=time.monotonic();self.step=0
        self.create_subscription(String,'/parking/state',lambda m:self.states.append(json.loads(m.data)),100)
        self.create_subscription(String,'/parking/status',lambda m:self.status.append(json.loads(m.data)),QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(MotionCommand,'/parking/motion_preview',lambda m:self.preview.append([m.steering,m.left_speed,m.right_speed]),10)
        self.create_subscription(MotionCommand,'/topic_control_signal',lambda m:self.hardware.append([m.steering,m.left_speed,m.right_speed]),10)
        self.create_subscription(OccupancyGrid,'/map',self.on_map,QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(RosPath,'/parking/tracking_path',lambda m:setattr(self,'paths',self.paths+1),10)
        if synthetic:
            self.scan_pub=self.create_publisher(LaserScan,'/lidar_raw',qos_profile_sensor_data);self.imus=[self.create_publisher(Imu,f'/{n}/imu/data',qos_profile_sensor_data) for n in ['tractor','trailer']];self.create_timer(.01,self.publish_fixture)
            self.ranges=[]
            for i in range(720):
                a=-math.pi+i*2*math.pi/720;c,s=math.cos(a),math.sin(a);ds=[]
                for x in [-4.,10.]:
                    if abs(c)>1e-8:
                        d=x/c;y=d*s
                        if d>0 and -6.<=y<=4.:ds.append(d)
                for y in [-6.,4.]:
                    if abs(s)>1e-8:
                        d=y/s;x=d*c
                        if d>0 and -4.<=x<=10.:ds.append(d)
                self.ranges.append(min(ds) if ds else float('inf'))
    def on_map(self,m):
        if any(x>=50 for x in m.data):self.maps+=1
    def publish_fixture(self):
        elapsed=time.monotonic()-self.started
        if elapsed<2. or elapsed>17.:return
        now=self.get_clock().now().to_msg()
        for name,pub in zip(['tractor','trailer'],self.imus):
            m=Imu();m.header.stamp=now;m.header.frame_id=name+'_imu_link';m.orientation_covariance[0]=-1.;m.linear_acceleration_covariance[0]=-1.;m.angular_velocity_covariance=[.0001,0.,0.,0.,.0001,0.,0.,0.,.0001];pub.publish(m)
        self.step+=1
        if self.step%10==0:
            m=LaserScan();m.header.stamp=now;m.header.frame_id='laser_frame';m.angle_min=-math.pi;m.angle_increment=2*math.pi/720;m.angle_max=m.angle_min+719*m.angle_increment;m.range_min=.02;m.range_max=12.;m.scan_time=.1;m.ranges=self.ranges;self.scan_pub.publish(m)

def run(synthetic=False):
    name='synthetic_integration' if synthetic else ('optional_imu_config' if '--optional-config' in sys.argv else 'default_no_hardware');log=W/'verification'/f'{name}.log';out=W/'verification'/name;out.mkdir(exist_ok=True)
    result_file=W/'verification'/f'{name}.json';result_file.write_text(json.dumps({'status':'RUNNING','hardware_test':False}))
    command=['ros2','launch','mvsim_slam_real_bringup','real_parking.launch.py',f'output_directory:={out}']
    if '--optional-config' in sys.argv:command.append('enable_imu_fusion:=true')
    if synthetic:
        geometry=yaml.safe_load((W/'src/mvsim_slam_real_bringup/config/vehicle_geometry.yaml').read_text());geometry['initial_alignment_confirmed']=True;geometry['trailer_bias_confirmed']=True;fixture=out/'SYNTHETIC_ONLY_geometry.yaml';fixture.write_text(yaml.safe_dump(geometry));command.append('geometry_file:='+str(fixture))
    with log.open('w') as stream:
        proc=subprocess.Popen(command,stdout=stream,stderr=subprocess.STDOUT,start_new_session=True,env=os.environ.copy());node=Probe(synthetic);start=time.monotonic();graph=[]
        try:
            while time.monotonic()-start<(21. if synthetic else 5.):
                rclpy.spin_once(node,timeout_sec=.02)
                if proc.poll() is not None:raise AssertionError(f'launch exited: {proc.returncode}')
                if time.monotonic()-start>3.:graph=node.get_node_names()
            assert 'serial_sender_node' not in graph and 'motion_planner_node' not in graph
            assert len(node.states)>5 and node.status and node.preview and not node.hardware
            if synthetic:
                valid=[s for s in node.states if s['valid']]
                assert valid,'no synthetic SLAM/gyro state became valid'
                assert node.maps and node.paths,'Cartographer map/tracking outputs absent'
                assert any(s['state']=='SEARCH_FORWARD' for s in node.status),'original AutoParking never entered search'
                assert any(abs(m[1])>0 for m in node.preview),'no synthetic control preview observed'
                assert not node.states[-1]['valid'] and node.preview[-1][1:]==[0,0],'lost sensors failed to stop preview'
            else:
                assert all(not s['valid'] for s in node.states) and all(m[1:]==[0,0] for m in node.preview)
                assert node.status[-1]['state']=='WAIT_FOR_LOCALIZATION'
            result={'status':'PASS','synthetic_fixture':synthetic,'hardware_test':False,'nodes':sorted(set(graph)),'states':len(node.states),'valid_states':sum(bool(s['valid']) for s in node.states),'map_messages':node.maps,'tracking_path_messages':node.paths,'hardware_commands':len(node.hardware),'last_status':node.status[-1]['state'],'last_preview':node.preview[-1]}

        finally:
            (W/'verification'/f'{name}_trace.json').write_text(json.dumps({'states':node.states,'parking_status':node.status,'preview':node.preview},indent=2))
            node.destroy_node();proc.send_signal(signal.SIGINT)
            try:proc.wait(timeout=12.)
            except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGKILL);proc.wait(timeout=3.)
        text=log.read_text()
        # Shutdown KeyboardInterrupt traces are distinguished from runtime process failures.
        runtime=text
        if 'process has died' in runtime or 'Traceback' in runtime:
            result['status']='FAIL';result['reason']='process failure including shutdown';result_file.write_text(json.dumps(result,indent=2));raise AssertionError('runtime process failure; inspect '+str(log))
        result_file.write_text(json.dumps(result,indent=2));print(json.dumps(result))

def main():
    rclpy.init()
    try:
        if '--synthetic-only' not in sys.argv:run(False)
        if '--optional-config' not in sys.argv:run(True)
    except BaseException as error:
        for p in (W/'verification').glob('*.json'):
            try:record=json.loads(p.read_text())
            except ValueError:continue
            if isinstance(record,dict) and record.get('status')=='RUNNING':record.update(status='FAIL',reason=str(error));p.write_text(json.dumps(record,indent=2))
        raise
    finally:rclpy.shutdown()
if __name__=='__main__':main()
