#!/usr/bin/env python3
"""Synthetic-only ROS callback smoke; no device access or actuation."""
import json
import tempfile
import time
from pathlib import Path
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from real_parking.wallless_node import WalllessValidation


def main():
    with tempfile.TemporaryDirectory(prefix='wallless-smoke-') as directory:
        output = Path(directory)/'events.jsonl'
        rclpy.init(args=['--ros-args', '-p', f'output_file:={output}'])
        node = WalllessValidation()
        try:
            scan = LaserScan(); scan.range_min = .15; scan.range_max = 12.
            scan.angle_increment = .01; scan.ranges = [2.]*20; scan.header.frame_id = 'laser_frame'
            for _ in range(3):
                stamp = node.get_clock().now().to_msg()
                tf = TransformStamped(); tf.header.frame_id = 'map'; tf.child_frame_id = 'base_footprint'
                tf.header.stamp = stamp; tf.transform.rotation.w = 1.
                node.buffer.set_transform(tf, 'SYNTHETIC_ONLY')
                scan.header.stamp = stamp; node.on_scan(scan)
                grid = OccupancyGrid(); grid.header.stamp = stamp; grid.header.frame_id = 'map'
                grid.info.resolution = .1; grid.info.width = 180; grid.info.height = 80
                grid.info.origin.position.x = -2.; grid.info.origin.position.y = -5.
                grid.info.origin.orientation.w = 1.
                data = [0]*(180*80)
                for y in range(30, 36):
                    for x in list(range(40, 50))+list(range(90, 100)):
                        data[y*180+x] = 100
                grid.data = data; node.on_map(grid); node.tick(); time.sleep(1.05)
            assert node.result['ready'], node.error
            node.on_reference(String(data='synthetic_start'))
            node.scan_wall = 0.; node.tick()
        finally:
            node.destroy_node(); rclpy.shutdown()
        events = [json.loads(line) for line in output.read_text().splitlines()]
        assert any(e.get('perception', {}).get('ready') for e in events)
        assert events[-1]['inputs_fresh'] is False and not events[-1]['perception']
        assert any(e.get('event') == 'reference' for e in events)
        print('SYNTHETIC_ONLY: TF/map/scan, gap, reference logging, stale suppression passed')


if __name__ == '__main__':
    main()
