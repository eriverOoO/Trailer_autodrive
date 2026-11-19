#!/usr/bin/env python3
"""Replay recorded perception data at original cadence in a separate ROS domain."""
import argparse
import json
from pathlib import Path
import time

import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.qos import QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import String

W=Path(__file__).resolve().parents[1]


def read_records(directory):
    records=[json.loads(line) for line in (directory/'measurements.jsonl').read_text().splitlines() if line.strip()]
    if not records or not any(row['type']=='image' for row in records) or not any(row['type']=='scan' for row in records):
        raise ValueError('Capture needs both image and scan records')
    for row in records:
        if type(row['stamp_ns']) is not int or row['stamp_ns']<=0:
            raise ValueError('Every replay record needs a real header timestamp')
        if row['type']=='image':
            file=(directory/row['file']).resolve()
            if not file.is_relative_to(directory.resolve()) or not file.is_file():
                raise ValueError('Capture image missing or outside capture directory')
        elif row['type']!='scan':
            raise ValueError('Unknown capture record type')
    return sorted(records,key=lambda row:row['stamp_ns'])


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--capture',type=Path,default=W/'artifacts/sensor_compare_push/20261118_155446')
    parser.add_argument('--domain-id',type=int,default=230)
    args=parser.parse_args()
    if not 1<=args.domain_id<=232:
        parser.error('Use a dedicated nonzero ROS domain in 1..232 for replay')
    records=read_records(args.capture)
    rclpy.init(domain_id=args.domain_id)
    node=rclpy.create_node('recorded_sensor_replay')
    image_pub=node.create_publisher(Image,'/sensor_replay/image_raw',QoSProfile(depth=1))
    scan_pub=node.create_publisher(LaserScan,'/sensor_replay/scan',qos_profile_sensor_data)
    status_pub=node.create_publisher(String,'/sensor_replay/status',10)
    bridge=CvBridge()
    counts={'images':0,'scans':0}
    try:
        # Let the diagnostic's subscriptions discover these replay-only publishers.
        until=time.monotonic()+1.5
        while time.monotonic()<until:
            rclpy.spin_once(node,timeout_sec=.02)
        start=time.monotonic()
        start_ns=node.get_clock().now().nanoseconds
        first_ns=records[0]['stamp_ns']
        for row in records:
            offset=(row['stamp_ns']-first_ns)/1e9
            while rclpy.ok() and time.monotonic()-start<offset:
                rclpy.spin_once(node,timeout_sec=min(.01,max(0.,offset-(time.monotonic()-start))))
            if not rclpy.ok():
                break
            if row['type']=='image':
                frame=cv2.imread(str(args.capture/row['file']))
                if frame is None:
                    raise ValueError('Cannot decode capture image: '+row['file'])
                message=bridge.cv2_to_imgmsg(frame,encoding='bgr8')
                counts['images']+=1
                publisher=image_pub
            else:
                message=LaserScan()
                for field in ('angle_min','angle_increment','range_min','range_max','scan_time','time_increment'):
                    setattr(message,field,float(row[field]))
                message.ranges=[float(value) if value is not None else float('nan') for value in row['ranges']]
                message.angle_max=message.angle_min+max(0,len(message.ranges)-1)*message.angle_increment
                counts['scans']+=1
                publisher=scan_pub
            stamp=start_ns+row['stamp_ns']-first_ns
            message.header.stamp.sec=stamp//10**9
            message.header.stamp.nanosec=stamp%10**9
            message.header.frame_id=row['frame_id']
            publisher.publish(message)
            status=String(data=json.dumps({'replay_only':True,'domain_id':args.domain_id,
                                          'original_stamp_ns':row['stamp_ns'],**counts}))
            status_pub.publish(status)
        until=time.monotonic()+.5
        while rclpy.ok() and time.monotonic()<until:
            rclpy.spin_once(node,timeout_sec=.02)
        print(json.dumps({'replay_only':True,'capture':str(args.capture),**counts}),flush=True)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__=='__main__':
    main()
