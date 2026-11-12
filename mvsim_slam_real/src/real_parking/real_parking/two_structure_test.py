"""Separate automatic-search / manual-survey test using the preserved B controller."""

from collections import deque
from dataclasses import replace
import json
import math
import time

import numpy as np
from rcl_interfaces.msg import SetParametersResult
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from scipy.spatial.transform import Rotation
from std_msgs.msg import Header, String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

from .bootstrap import load
from .two_structure_gap import PairConfig, PairObservation, observe_two_structure_gap
load()
from tractor_parallel_parking_auto import AutoParking
from tractor_trailer_model import wrap


def transform_points(points, transform):
    """Apply a real 3D sensor TF before discarding height in the 2D map."""
    points = np.asarray(points, dtype=float)
    if points.shape[1] == 2:
        points = np.c_[points, np.zeros(len(points))]
    q, t = transform.rotation, transform.translation
    quaternion = np.array([q.x, q.y, q.z, q.w])
    if not np.isfinite(quaternion).all() or abs(float(quaternion@quaternion)-1) > .01:
        raise ValueError('Invalid association transform quaternion')
    transformed = Rotation.from_quat(quaternion).apply(points) + [t.x, t.y, t.z]
    if not np.isfinite(transformed).all():
        raise ValueError('Nonfinite association transform')
    return transformed


class TwoStructureParkingTest(AutoParking):
    """Change perception and survey entry only; reuse planning/execution guards."""

    def __init__(self):
        self.test_motion_enabled = False
        super().__init__()
        self.target_source = 'two observed LiDAR structures + calibrated camera; configured depth'
        self.declare_parameter('manual_survey', False)
        self.declare_parameter('require_camera', True)
        self.declare_parameter('camera_evidence_lifetime_s', 180.0)
        self.declare_parameter('camera_stream_timeout_s', 1.0)
        for name, value in vars(PairConfig()).items():
            if name not in ('slot_width', 'min_lateral_m'):
                self.declare_parameter('pair_'+name, value)
        self.pair_config()  # Reject invalid test dimensions at startup.
        if self.get_parameter('parking_side').value != 'left':
            raise ValueError('This isolated test expects parking_side=left')
        for name in ('camera_evidence_lifetime_s', 'camera_stream_timeout_s'):
            value = self.get_parameter(name).value
            if not math.isfinite(value) or value <= 0:
                raise ValueError(name+' must be finite and positive')
        self.operation = 'manual' if self.get_parameter('manual_survey').value else 'idle'
        self.parking_started = False
        self.camera_wall = 0.0
        self.camera_valid = False
        self.camera_stamp_ns = 0
        self.camera_scan_stamp_ns = 0
        self.camera_reason = 'waiting for calibrated camera / LiDAR associations'
        self.camera_evidence = []
        self.last_association_key = None
        self.stationary_samples = deque()
        self.association_buffer = Buffer()
        self.association_listener = TransformListener(self.association_buffer, self)
        self.test_observation = self.create_publisher(String, '/parking_test/observation', 10)
        self.test_markers = self.create_publisher(MarkerArray, '/parking_test/markers', 10)
        self.create_subscription(String, '/parking_vision/associations', self.on_associations,
                                 qos_profile_sensor_data)
        self.create_service(Trigger, '/parking_test/start_search', self.start_search)
        self.create_service(Trigger, '/parking_test/start_parking', self.start_parking)
        self.create_service(Trigger, '/parking_test/stop', self.stop_test)
        self.add_on_set_parameters_callback(self.validate_parameters)
        self.get_logger().info('Two-structure test: waiting for start_search; '
                               'manual_survey is an intermediate test option only')

    def pair_config(self, overrides=None):
        overrides = overrides or {}
        values = {}
        for name in vars(PairConfig()):
            parameter = ('slot_width' if name == 'slot_width' else
                         'gap_min_lateral_m' if name == 'min_lateral_m' else 'pair_'+name)
            values[name] = overrides.get(parameter, self.get_parameter(parameter).value)
        return PairConfig(**values)

    def validate_parameters(self, parameters):
        overrides = {p.name: p.value for p in parameters}
        protected = {'slot_width', 'gap_min_lateral_m', 'parking_side', 'manual_survey',
                     'require_camera', 'camera_evidence_lifetime_s', 'camera_stream_timeout_s'}
        affected = any(name in protected or name.startswith('pair_') for name in overrides)
        try:
            if affected and self.test_motion_enabled:
                raise ValueError('Restart the test before changing perception during automatic operation')
            if overrides.get('parking_side', 'left') != 'left':
                raise ValueError('This test expects parking_side=left')
            if 'manual_survey' in overrides:
                raise ValueError('Select manual_survey at launch, then restart to change it')
            self.pair_config(overrides)
            for name in ('camera_evidence_lifetime_s', 'camera_stream_timeout_s'):
                if name in overrides and (not math.isfinite(overrides[name]) or overrides[name] <= 0):
                    raise ValueError(name+' must be finite and positive')
            if affected:
                self.observation_window = []
                self.observation_hits = 0
            return SetParametersResult(successful=True)
        except (ValueError, TypeError) as exc:
            return SetParametersResult(successful=False, reason=str(exc))

    def send(self, velocity, steering):
        if not self.test_motion_enabled:
            velocity = steering = 0.0
        super().send(velocity, steering)

    def camera_ready(self):
        if not self.get_parameter('require_camera').value:
            return True
        timeout = self.get_parameter('camera_stream_timeout_s').value
        now_ns = self.get_clock().now().nanoseconds
        return (self.camera_valid and time.monotonic()-self.camera_wall <= timeout
                and -.03 <= (now_ns-self.camera_stamp_ns)/1e9 <= timeout
                and -.03 <= (now_ns-self.camera_scan_stamp_ns)/1e9 <= timeout)

    def on_associations(self, message):
        try:
            data = json.loads(message.data)
            self.camera_reason = data.get('reason', 'invalid camera correspondence')
            if data.get('calibrated') is not True or data.get('valid') is not True:
                self.camera_valid = False
                return
            image_stamp, scan_stamp = data['image_stamp_ns'], data['scan_stamp_ns']
            now_ns = self.get_clock().now().nanoseconds
            timeout = self.get_parameter('camera_stream_timeout_s').value
            if not all(type(stamp) is int and stamp > 0
                       and -.03 <= (now_ns-stamp)/1e9 <= timeout for stamp in (image_stamp, scan_stamp)):
                raise ValueError('stale/future image or scan association')
            if abs(image_stamp-scan_stamp) > 100_000_000:
                raise ValueError('image and scan timestamps differ by more than 100 ms')
            if not isinstance(data['scan_frame'], str) or not data['scan_frame']:
                raise ValueError('association scan frame missing')
            key = (image_stamp, scan_stamp)
            if key == self.last_association_key:
                return
            # Store in odom so old landmarks follow the latest map->odom correction.
            groups = data['objects']
            if not isinstance(groups, list) or len(groups) > 64:
                raise ValueError('invalid association object list')
            if groups:
                tf = self.association_buffer.lookup_transform(
                    'odom', data['scan_frame'], Time(nanoseconds=scan_stamp)).transform
            received = time.monotonic()
            lifetime = self.get_parameter('camera_evidence_lifetime_s').value
            evidence = [entry for entry in self.camera_evidence if received-entry[0] <= lifetime]
            for group in groups:
                points = np.asarray(group['scan_xy'], dtype=float)
                if (points.ndim != 2 or points.shape[1] != 2 or not 3 <= len(points) <= 10000
                        or not np.isfinite(points).all()):
                    raise ValueError('invalid measured scan XY group')
                points = transform_points(points, tf)
                # Coalesce repeat frames instead of evicting the first structure
                # merely because the second one produces many image frames.
                center = np.mean(points, axis=0)
                evidence = [entry for entry in evidence
                            if np.linalg.norm(np.mean(entry[1], axis=0)-center) > .15]
                evidence.append((received, points))
            self.camera_evidence = evidence[-64:]
            self.last_association_key = key
            self.camera_wall = received
            self.camera_stamp_ns, self.camera_scan_stamp_ns = image_stamp, scan_stamp
            self.camera_valid = True
        except (KeyError, TypeError, ValueError, TransformException) as exc:
            self.camera_valid = False
            self.camera_reason = str(exc)

    def evidence_in_map(self, frame):
        now = time.monotonic()
        lifetime = self.get_parameter('camera_evidence_lifetime_s').value
        self.camera_evidence = [entry for entry in self.camera_evidence if now-entry[0] <= lifetime]
        if not self.camera_evidence:
            return []
        tf = self.association_buffer.lookup_transform(frame, 'odom', Time()).transform
        return [transform_points(points, tf)[:, :2] for _, points in self.camera_evidence]

    def detect_gap(self, road_pose, length, width):
        from .nodes import grid_from_message
        message = self.detection_msg
        if message is None or message.header.frame_id != 'local_map':
            observation = PairObservation(reason='waiting for measured local_map detection grid')
            self.publish_observation(observation, Header(frame_id='local_map', stamp=self.get_clock().now().to_msg()))
            return observation
        stamp_ns = message.header.stamp.sec*10**9+message.header.stamp.nanosec
        if not -.03 <= (self.get_clock().now().nanoseconds-stamp_ns)/1e9 <= 2.5:
            observation = PairObservation(reason='detection map timestamp stale/future')
            self.publish_observation(observation, message.header)
            return observation
        config = self.pair_config()
        if self.footprints is not None:
            config = replace(config, min_gap_length=max(config.min_gap_length,
                                                      float(self.footprints.size[0])+.36))
            if config.slot_width < float(self.footprints.size[1])+.36:
                raise RuntimeError('configured depth is too narrow for rig plus existing clearance')
        try:
            evidence = self.evidence_in_map(message.header.frame_id)
        except (ValueError, TransformException) as exc:
            self.camera_reason = str(exc)
            evidence = []
        observation = observe_two_structure_gap(
            grid_from_message(message), road_pose, config,
            camera_evidence=evidence, require_camera=self.get_parameter('require_camera').value)
        if self.get_parameter('require_camera').value and not self.camera_ready():
            observation.target = None
            observation.reason = 'camera unavailable: '+self.camera_reason
        self.publish_observation(observation, message.header)
        return observation

    def publish_observation(self, observation, header):
        target = observation.lidar_target
        message = String()
        message.data = json.dumps({
            'stamp_ns': self.get_clock().now().nanoseconds,
            'map_stamp_ns': header.stamp.sec*10**9+header.stamp.nanosec,
            'frame_id': header.frame_id, 'mode': self.operation,
            'valid': observation.target is not None,
            'lidar_pair_seen': target is not None,
            'camera_confirmed': observation.camera_confirmed,
            'require_camera': self.get_parameter('require_camera').value,
            'camera_stream_ready': self.camera_ready(),
            'reason': observation.reason, 'free_fraction': observation.free_fraction,
            'center': target.center.tolist() if target else None,
            'heading': target.heading if target else None,
            'length': target.length if target else None,
            'width': self.get_parameter('slot_width').value,
            'depth_source': 'configured; no rear wall inferred',
        }, allow_nan=False)
        self.test_observation.publish(message)
        markers = [Marker(action=Marker.DELETEALL)]
        if target:
            bay = Marker(header=header, ns='two_structure_bay', id=0, type=Marker.LINE_STRIP,
                         action=Marker.ADD)
            bay.pose.orientation.w = 1.0
            bay.scale.x = .035
            bay.color.r = 1.0 if observation.target is None else 0.0
            bay.color.g = 1.0
            bay.color.a = 1.0
            bay.points = [Point(x=float(x), y=float(y), z=.05)
                          for x, y in np.vstack([target.polygon(), target.polygon()[0]])]
            markers.append(bay)
            for i, component in enumerate(observation.components):
                marker = Marker(header=header, ns='measured_structures', id=i, type=Marker.POINTS,
                                action=Marker.ADD)
                marker.pose.orientation.w = 1.0
                marker.scale.x = marker.scale.y = .06
                marker.color.b = marker.color.a = 1.0
                marker.points = [Point(x=float(x), y=float(y), z=.05) for x, y in component]
                markers.append(marker)
        self.test_markers.publish(MarkerArray(markers=markers))

    def stable_observation(self, target):
        if target is None:
            self.observation_window = []
            self.observation_hits = 0
            self.legacy_observation_stamps = []
            return False
        return super().stable_observation(target)

    def stop_test(self, request, response):
        self.send(0., 0.)
        self.test_motion_enabled = False
        self.transition('EMERGENCY_STOP', 'explicit test stop command; restart to run again')
        response.success = True
        response.message = 'Zero command sent; test stopped'
        return response

    def streams_ready(self):
        now = time.monotonic()
        return (self.map_msg is not None and self.map_msg.header.frame_id == 'local_map'
                and self.footprints is not None and self.est.pose() is not None
                and now-self.map_wall <= 1.5 and now-self.scan_wall <= 1.0
                and now-self.local_status_wall <= 1.5)

    def stationary(self):
        samples = self.stationary_samples
        if len(samples) < 2 or time.monotonic()-samples[-1][0] > .3:
            return False
        if samples[-1][0]-samples[0][0] < .8:
            return False
        states = np.array([s for _, s in samples])
        return (np.max(np.linalg.norm(states[:, :2]-states[0, :2], axis=1)) < .025
                and np.max(np.abs(wrap(states[:, 2:]-states[0, 2:]))) < math.radians(2))

    def start_search(self, request, response):
        if self.test_motion_enabled or self.stage in ('FAILED', 'EMERGENCY_STOP', 'PARKED'):
            response.message = 'Restart the test before starting another search'
        elif not self.streams_ready() or not self.camera_ready() or not self.stationary():
            response.message = 'Need current SLAM/IMU/scan/camera and a stationary rig for one second'
        elif not self.get_parameter('require_camera').value:
            response.message = 'LiDAR-only diagnostics cannot start automatic motion'
        else:
            self.operation = 'automatic_search'
            self.test_motion_enabled = True
            self.auto = True
            self.transition('WAIT_FOR_LOCALIZATION', 'explicit automatic search command')
            response.success = True
            response.message = 'Automatic forward search -> bay confirmation -> existing B parking'
        return response

    def start_parking(self, request, response):
        if self.operation != 'manual' or self.test_motion_enabled:
            response.message = 'start_parking is for manual_survey only; use start_search for final autonomy'
            return response
        if (not self.get_parameter('require_camera').value or not self.streams_ready()
                or not self.camera_ready() or not self.stationary()):
            response.message = 'Need calibrated camera, current sensors and a stationary rig for one second'
            return response
        try:
            observation = self.detect_gap(self.road_pose, 0., self.get_parameter('slot_width').value)
            if observation.target is None or self.observation_hits < 3:
                raise RuntimeError('Need three stable confirmed bay maps: '+observation.reason)
            self.test_motion_enabled = True
            self.parking_started = True
            self.auto = True
            self.operation = 'parking_after_manual_survey'
            self.send(0., 0.)
            now = time.monotonic()
            self.settle_until = now+10.0
            self.stop_started = {'mono_ns': time.monotonic_ns(),
                                 'sim_ns': self.get_clock().now().nanoseconds,
                                 'state': self.current.tolist()}
            self.observation_window = []
            self.observation_hits = 0
            self.observation_stamp = None
            self.transition('STOP_AND_STABILIZE', 'manual survey complete; explicit parking command')
            response.success = True
            response.message = 'Revalidate bay after settling, then use existing B planner/controller'
        except (RuntimeError, ValueError) as exc:
            response.message = str(exc)
        return response

    def tick_checked(self):
        if self.stage in ('FAILED', 'EMERGENCY_STOP', 'PARKED'):
            return super().tick_checked()
        if self.test_motion_enabled:
            if not self.camera_ready():
                self.send(0., 0.)
                self.transition('EMERGENCY_STOP', 'calibrated camera/scan stream lost')
                return
            return super().tick_checked()
        self.send(0., 0.)
        if not self.streams_ready():
            self.stationary_samples.clear()
            self.publish_status()
            return
        state = self.est.update(calibrate=self.road_pose is None)
        if state is None:
            self.stationary_samples.clear()
            return
        self.current = state
        self.observe_pose_age()
        if self.road_pose is None:
            self.road_pose = self.est.initial_pose.copy()
        self.stage = 'MANUAL_SURVEY' if self.operation == 'manual' else 'WAIT_FOR_START_COMMAND'
        now = time.monotonic()
        self.stationary_samples.append((now, state.copy()))
        while self.stationary_samples and now-self.stationary_samples[0][0] > 1.1:
            self.stationary_samples.popleft()
        if now-self.last_detect > .5:
            self.last_detect = now
            observation = self.detect_gap(self.road_pose, 0., self.get_parameter('slot_width').value)
            self.last_gap_observation = observation
            self.first_seen = observation.first_obstacle
            self.gap_seen = observation.gap_seen
            self.second_seen = observation.second_obstacle
            self.stable_observation(observation.target)
        self.visualize()
        self.publish_status()


def main(args=None):
    from .nodes import auto_parking_main
    auto_parking_main(args, node_class=TwoStructureParkingTest)
