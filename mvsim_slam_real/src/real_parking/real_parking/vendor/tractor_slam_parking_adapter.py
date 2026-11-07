#!/usr/bin/env python3
"""Cartographer-only SE(2) pose/map interface for the existing parking stack.

No simulator TF, odometry, commanded motion or ground truth is consumed.
Native map resolution/cell geometry is retained; unknown cells remain unknown.
"""
import copy
import json
import math
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, Imu
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener, TransformException, StaticTransformBroadcaster
from tractor_trailer_state_estimator import yaw, stamp_seconds


def compose(a, b):
    c, s = math.cos(a[2]), math.sin(a[2])
    return np.array([a[0]+c*b[0]-s*b[1], a[1]+s*b[0]+c*b[1],
                     math.atan2(math.sin(a[2]+b[2]), math.cos(a[2]+b[2]))])


def inverse(a):
    c, s = math.cos(a[2]), math.sin(a[2])
    return np.array([-c*a[0]-s*a[1], s*a[0]-c*a[1], -a[2]])


def planar(transform):
    p = transform.translation
    return np.array([p.x, p.y, yaw(transform.rotation)])


def valid_quaternion(q):
    v = np.array([q.x, q.y, q.z, q.w])
    return np.isfinite(v).all() and abs(float(v@v)-1.) < .01


def convert_grid(msg, local_from_map):
    """Rigidly transform the grid origin; leave every cell footprint intact.

    Binarization uses the existing planner's threshold 50. This also keeps its
    gap detector (threshold 52) consistent with collision checking.
    """
    if (msg.header.frame_id != 'map' or msg.info.resolution <= 0 or
        not math.isfinite(msg.info.resolution) or not valid_quaternion(msg.info.origin.orientation) or
        len(msg.data) != msg.info.width*msg.info.height or not msg.data):
        raise ValueError('invalid Cartographer OccupancyGrid geometry/frame')
    o = msg.info.origin
    origin = np.array([o.position.x, o.position.y, yaw(o.orientation)])
    if not np.isfinite(origin).all():
        raise ValueError('nonfinite map origin')
    values = np.asarray(msg.data)
    if np.any((values < -1) | (values > 100)):
        raise ValueError('invalid occupancy probability')
    out = copy.deepcopy(msg)
    xyh = compose(local_from_map, origin)
    out.header.frame_id = 'local_map'
    out.info.origin.position.x = float(xyh[0])
    out.info.origin.position.y = float(xyh[1])
    out.info.origin.orientation.x = out.info.origin.orientation.y = 0.
    out.info.origin.orientation.z = math.sin(xyh[2]/2)
    out.info.origin.orientation.w = math.cos(xyh[2]/2)
    out.data = np.where(values < 0, -1, np.where(values >= 50, 100, 0)).tolist()
    return out


class SlamParkingAdapter(Node):
    def __init__(self):
        super().__init__('tractor_slam_parking_adapter')
        self.buffer = Buffer(cache_time=rclpy.duration.Duration(seconds=15))
        self.listener = TransformListener(self.buffer, self)
        self.static = StaticTransformBroadcaster(self)
        self.anchor = None
        self.initial_correction = None
        self.map_msg = None
        self.map_wall = 0.
        self.map_valid = False
        self.sensor = {}
        self.last_pose_key = None
        self.last_error = 'waiting for SLAM pose/map'
        self.pose_pub = self.create_publisher(PoseStamped, '/tractor/localization/pose', 20)
        self.correction_pub = self.create_publisher(PoseStamped, '/tractor/localization/map_correction', 20)
        self.status_pub = self.create_publisher(String, '/tractor/localization/status', 20)
        q = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.map_pub = self.create_publisher(OccupancyGrid, '/tractor/localization/local_map', q)
        self.create_subscription(OccupancyGrid, '/tractor/slam/map', self.on_map, q)
        for name, kind, topic in [('scan', LaserScan, '/tractor/laser1'),
                                 ('imu', Imu, '/tractor/imu_tractor')]:
            self.create_subscription(kind, topic, lambda m, k=name: self.on_sensor(k, m),
                                     qos_profile_sensor_data)
        self.timer = self.create_timer(.02, self.tick)

    def on_sensor(self, kind, msg):
        if kind == 'scan':
            good = (msg.header.frame_id == 'laser1' and len(msg.ranges) > 20 and
                    np.count_nonzero(np.isfinite(msg.ranges)) > 20)
        else:
            good = (msg.header.frame_id == 'imu_tractor' and
                    np.isfinite([msg.angular_velocity.z, msg.linear_acceleration.x,
                                 msg.linear_acceleration.y, msg.linear_acceleration.z]).all() and
                    msg.angular_velocity_covariance[0] >= 0 and msg.linear_acceleration_covariance[0] >= 0)
        if good:
            self.sensor[kind] = (stamp_seconds(msg.header.stamp), time.monotonic())

    def on_map(self, msg):
        self.map_msg = msg
        self.map_wall = time.monotonic()
        self.map_valid = False
        if self.anchor is not None:
            try:
                out = convert_grid(msg, self.anchor)
                self.map_valid = True
                now = self.get_clock().now().nanoseconds*1e-9
                if -.1 <= now-stamp_seconds(msg.header.stamp) <= 1.5:
                    self.map_pub.publish(out)
            except ValueError as exc:
                self.last_error = str(exc)

    def tick(self):
        now = self.get_clock().now().nanoseconds*1e-9
        wall = time.monotonic()
        status = dict(initialized=False, source='cartographer', error=self.last_error)
        try:
            for kind in ('scan', 'imu'):
                st, wt = self.sensor.get(kind, (-math.inf, -math.inf))
                if not (-.1 <= now-st <= .8 and wall-wt <= 1.):
                    raise ValueError(f'{kind} missing/stale')
            tf = self.buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            st = stamp_seconds(tf.header.stamp)
            if not (-.1 <= now-st <= .8) or not valid_quaternion(tf.transform.rotation):
                raise ValueError('SLAM measured pose stale/invalid')
            mo = self.buffer.lookup_transform('map', 'odom', rclpy.time.Time.from_msg(tf.header.stamp))
            base = planar(tf.transform)
            if not np.isfinite(base).all() or not valid_quaternion(mo.transform.rotation):
                raise ValueError('SLAM TF invalid')
            if self.anchor is None:
                self.anchor = inverse(base)
                self.initial_correction = yaw(mo.transform.rotation)
                # map -> local_map is the inverse of the numerical pose adapter.
                anchor_tf = TransformStamped()
                anchor_tf.header = tf.header
                anchor_tf.child_frame_id = 'local_map'
                anchor_tf.transform.translation.x = float(base[0])
                anchor_tf.transform.translation.y = float(base[1])
                anchor_tf.transform.rotation.z = math.sin(base[2]/2)
                anchor_tf.transform.rotation.w = math.cos(base[2]/2)
                self.static.sendTransform(anchor_tf)
                if self.map_msg is not None:
                    self.on_map(self.map_msg)
            if self.map_msg is None or wall-self.map_wall > 1.5:
                raise ValueError('SLAM map missing/stale reception')
            map_age = now-stamp_seconds(self.map_msg.header.stamp)
            if not (-.1 <= map_age <= 1.5):
                raise ValueError('SLAM map timestamp stale')
            if not self.map_valid:
                raise ValueError('SLAM map geometry invalid')
            pose = compose(self.anchor, base)
            correction = yaw(mo.transform.rotation)-self.initial_correction
            key = (st, *base, correction)
            if key != self.last_pose_key:
                out = PoseStamped()
                out.header.stamp = tf.header.stamp
                out.header.frame_id = 'local_map'
                out.pose.position.x = float(pose[0])
                out.pose.position.y = float(pose[1])
                out.pose.orientation.z = math.sin(pose[2]/2)
                out.pose.orientation.w = math.cos(pose[2]/2)
                corr = PoseStamped()
                corr.header = copy.deepcopy(out.header)
                corr.pose.orientation.z = math.sin(correction/2)
                corr.pose.orientation.w = math.cos(correction/2)
                self.correction_pub.publish(corr)
                self.pose_pub.publish(out)
                self.last_pose_key = key
            status.update(initialized=True, error='', pose_stamp_s=st,
                          pose_age_s=now-st, map_age_s=map_age,
                          map_correction_yaw=correction,
                          local_from_map=self.anchor.tolist(),
                          resolution=self.map_msg.info.resolution)
        except (ValueError, TransformException) as exc:
            self.last_error = str(exc)
            status['error'] = self.last_error
        msg = String()
        msg.data = json.dumps(status)
        self.status_pub.publish(msg)


def main():
    rclpy.init()
    node = SlamParkingAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
