"""Receive-only EBIMU bridge, opt-in explicit role/port; isolated visualization topics."""
import argparse
import json
import os
from pathlib import Path
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from diagnostic_msgs.msg import DiagnosticArray,DiagnosticStatus,KeyValue
from .serial_protocol import Parser,open_serial


class Bridge(Node):
    def __init__(self):
        super().__init__('vehicle_viz_serial_bridge')
        defaults=dict(role='',port='',other_imu_port='',lidar_port='',ports_confirmed=False,
                      baud=115200,orientation='quaternion',gyro=True,accel=True,timestamp=True,
                      output_directory='')
        for k,v in defaults.items():self.declare_parameter(k,v)
        v={k:self.get_parameter(k).value for k in defaults}
        if v['role'] not in ('tractor','trailer') or not v['port'] or not v['ports_confirmed']:
            raise ValueError('explicit role, port and physical port confirmation required')
        for other in (v['other_imu_port'],v['lidar_port']):
            if other and os.path.realpath(other)==os.path.realpath(v['port']):
                raise ValueError('IMUs and LiDAR must use distinct physical devices')
        if not v['output_directory']:raise ValueError('new output directory required for raw evidence')
        p=Path(v['output_directory']);p.mkdir(parents=True,exist_ok=False)
        self.raw=(p/'rx.bin').open('xb')
        self.log=(p/'samples.jsonl').open('x')
        self.parser=Parser(v['orientation'],v['gyro'],v['accel'],v['timestamp'])
        self.ser=open_serial(argparse.Namespace(**v))
        self.role=v['role'];self.frame='vehicle_viz_2d_'+self.role+'_imu'
        self.pub=self.create_publisher(Imu,'/vehicle_viz_2d/'+self.role+'/imu/data',qos_profile_sensor_data)
        self.diag=self.create_publisher(DiagnosticArray,'/vehicle_viz_2d/'+self.role+'/diagnostics',10)
        self.count=self.bytes=0
        self.started=time.monotonic();self.last=None
        self.error='';self.previous_sensor_ms=None;self.parser_fault=False
        self.create_timer(.002,self.poll);self.create_timer(.2,self.health)
        self.get_logger().info('REAL_UART_ONLY; TX=0; role='+self.role+'; port='+v['port'])

    def poll(self):
        if self.error:return
        try:
            data=self.ser.read(min(8192,self.ser.in_waiting))
            if not data:return
            self.raw.write(data);self.bytes+=len(data)
            before=self.parser.rejected
            samples=self.parser.feed(data)
            if self.parser.rejected>before:self.parser_fault=True
            for sample in samples:
                self.parser_fault=False
                sensor_ms=sample['sensor_ms']
                if sensor_ms is not None and self.previous_sensor_ms is not None and sensor_ms<=self.previous_sensor_ms:
                    raise ValueError('sensor timestamp reset/repeated; restart bridge and realign')
                self.previous_sensor_ms=sensor_ms
                stamp=self.get_clock().now().nanoseconds
                m=Imu();m.header.frame_id=self.frame
                m.header.stamp.sec,m.header.stamp.nanosec=divmod(stamp,10**9)
                m.orientation.x,m.orientation.y,m.orientation.z,m.orientation.w=sample['q']
                if sample['gyro'] is None:m.angular_velocity_covariance[0]=-1.
                else:m.angular_velocity.x,m.angular_velocity.y,m.angular_velocity.z=sample['gyro']
                if sample['accel'] is None:m.linear_acceleration_covariance[0]=-1.
                else:m.linear_acceleration.x,m.linear_acceleration.y,m.linear_acceleration.z=sample['accel']
                self.pub.publish(m);self.count+=1;self.last=time.monotonic()
                self.log.write(json.dumps(dict(host_stamp_ns=stamp,**sample))+'\n')
        except (OSError,ValueError) as exc:
            self.error=str(exc);self.ser.close();self.get_logger().error(self.error)

    def health(self):
        state='INVALID' if self.error or self.parser_fault else ('NO DATA' if self.last is None else ('STALE' if time.monotonic()-self.last>.5 else 'LIVE'))
        d=DiagnosticStatus();d.name='/vehicle_viz_2d/'+self.role+'/UART'
        d.hardware_id=self.ser.port;d.message=state+':'+self.error
        d.level=DiagnosticStatus.OK if state=='LIVE' else DiagnosticStatus.ERROR
        values=dict(rx_bytes=self.bytes,tx_bytes=0,parsed=self.parser.accepted,
                    rejected=self.parser.rejected,published=self.count,
                    timestamp='host receipt, not hardware synchronized',
                    covariance='received fields unknown=0; unavailable=-1',
                    reference='manufacturer native; mounting and sign require validation')
        d.values=[KeyValue(key=k,value=str(v)) for k,v in values.items()]
        msg=DiagnosticArray();msg.header.stamp=self.get_clock().now().to_msg();msg.status=[d]
        self.diag.publish(msg);self.raw.flush();self.log.flush()

    def destroy_node(self):
        self.ser.close();self.raw.close();self.log.close()
        return super().destroy_node()


def main(args=None):
    from .runtime import run
    run(Bridge,args)
