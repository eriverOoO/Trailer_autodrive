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
from tractor_parking_clearance import PLANNER_HARD_CLEARANCE_M
from tractor_parking_perception import planning_grid
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


def block_behind_bay(data, resolution, origin, target, margin=0.0):
    """Return a copy of data with every cell deeper than the bay's inner edge occupied.

    Bay frame: target.center/heading, left normal = heading+90 deg. Cells whose centre lies
    beyond target.width/2+margin along the normal become 100 over the whole map length.
    Bay interior, road side and already occupied cells are left as they are.
    """
    margin = float(margin)
    if not math.isfinite(margin) or margin < 0:
        raise ValueError('rear block margin must be finite and >= 0')
    data = np.array(data, dtype=np.int16, copy=True)
    _, depth = bay_coordinates(data.shape, resolution, origin, target)
    data[(depth > target.width/2+margin) & (data < 50)] = 100
    return data


def bay_coordinates(shape, resolution, origin, target):
    """Cell-centre coordinates in the bay frame: (along heading, along left normal)."""
    origin = np.asarray(origin, dtype=float)
    rows, cols = np.indices(shape)
    c, s = math.cos(origin[2]), math.sin(origin[2])
    u, v = (cols+.5)*resolution, (rows+.5)*resolution
    dx = origin[0]+c*u-s*v-target.center[0]
    dy = origin[1]+s*u+c*v-target.center[1]
    h = target.heading
    return dx*math.cos(h)+dy*math.sin(h), -dx*math.sin(h)+dy*math.cos(h)


def open_bay_unknown(data, resolution, origin, target):
    """Return a copy of data with unobserved cells inside the confirmed bay rectangle free.

    Matches the detection tolerance (a few unseen interior cells). Occupied cells and
    everything outside the bay stay as measured.
    """
    data = np.array(data, dtype=np.int16, copy=True)
    along, depth = bay_coordinates(data.shape, resolution, origin, target)
    inside = (np.abs(along) <= target.length/2) & (np.abs(depth) <= target.width/2)
    data[inside & (data < 0)] = 0
    return data


def sample_points(points, limit=300):
    """Evenly thinned [[x, y], ...] list for JSON/marker output."""
    points = np.asarray(points, dtype=float)
    step = max(1, int(math.ceil(len(points)/limit)))
    return np.round(points[::step], 3).tolist()


class TwoStructureParkingTest(AutoParking):
    """Detect a two-structure bay and choose a staging pose on the measured road."""

    parking_goal_offsets = ((.4, 0.), *AutoParking.parking_goal_offsets)
    stop_timeout_reason = 'rig did not settle with fresh localization'

    def __init__(self):
        self.test_motion_enabled = False
        super().__init__()
        self.staging_reference = None
        self.target_source = ('two observed LiDAR structures (+ camera unless LiDAR-only test); '
                              'inner edge on measured rear wall or configured depth')
        self.declare_parameter('manual_survey', False)
        self.declare_parameter('require_camera', True)
        # Test stage only: with require_camera:=false, also allow automatic search/parking
        # on LiDAR alone. Camera confirmation is still computed and reported.
        self.declare_parameter('allow_lidar_only_motion', False)
        self.declare_parameter('camera_evidence_lifetime_s', 180.0)
        self.declare_parameter('camera_stream_timeout_s', 1.0)
        self.declare_parameter('camera_object_labels', ['cone', 'white body', 'ride_on_car'])
        self.declare_parameter('camera_min_confidence', 0.)
        for name, value in vars(PairConfig()).items():
            if name not in ('slot_width', 'min_lateral_m'):
                self.declare_parameter('pair_'+name, value)
        # Virtual wall behind the confirmed bay: planning/execution never route around
        # the far side of the structures. Detection and forward search are unaffected.
        self.declare_parameter('pair_block_behind_bay', True)
        self.declare_parameter('pair_rear_block_margin_m', 0.0)
        # Planning relaxation: unobserved cells inside the confirmed bay count as free.
        self.declare_parameter('pair_bay_unknown_as_free', True)
        self.rear_block_cache = {}
        self.pair_config()  # Reject invalid test dimensions at startup.
        self.rear_block_margin()
        self.camera_filter()
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
        self.manual_survey_ready = False
        self.manual_stop_reason = None
        self.association_buffer = Buffer()
        self.association_listener = TransformListener(self.association_buffer, self)
        self.test_observation = self.create_publisher(String, '/parking_test/observation', 10)
        self.manual_guidance_pub = self.create_publisher(String, '/parking_test/manual_guidance', 1)
        self.test_markers = self.create_publisher(MarkerArray, '/parking_test/markers', 10)
        self.create_subscription(String, '/parking_vision/associations', self.on_associations,
                                 qos_profile_sensor_data)
        self.create_service(Trigger, '/parking_test/start_search', self.start_search)
        self.create_service(Trigger, '/parking_test/start_parking', self.start_parking)
        self.create_service(Trigger, '/parking_test/stop', self.stop_test)
        # Perception stays live independently of a blocked/failed parking plan.
        # Search/manual stages already publish observations in tick_checked.
        self.create_timer(.5, self.refresh_recognition)
        self.create_timer(.1, self.publish_manual_guidance)
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

    def rear_block_margin(self, overrides=None):
        value = (overrides or {}).get('pair_rear_block_margin_m',
                                      self.get_parameter('pair_rear_block_margin_m').value)
        if not math.isfinite(value) or value < 0:
            raise ValueError('pair_rear_block_margin_m must be finite and >= 0')
        return float(value)

    def grid(self, fill=False):
        target = getattr(self, 'target', None)
        depth_observed = getattr(target, 'depth_observed', True)
        grid = super().grid(fill and depth_observed)
        block = bool(self.get_parameter('pair_block_behind_bay').value)
        open_bay = depth_observed and bool(self.get_parameter('pair_bay_unknown_as_free').value)
        if target is None or not (block or open_bay):
            return grid
        margin = self.rear_block_margin()
        key = (grid.map_sequence, tuple(np.asarray(target.center, dtype=float)),
               float(target.heading), float(target.width), float(target.length), margin, block, open_bay,
               depth_observed)
        cached = self.rear_block_cache.get(fill)
        if cached is not None and cached[0] == key:
            return cached[1]
        data = grid.data
        if open_bay:
            data = open_bay_unknown(data, grid.resolution, grid.origin, target)
        if block:
            data = block_behind_bay(data, grid.resolution, grid.origin, target, margin)
        # Rebuild instead of editing grid.data so clearance (distance transform) follows the wall.
        blocked = planning_grid(data, grid.resolution, grid.origin, simulation_clearance=True)
        blocked.map_stamp_ns = grid.map_stamp_ns
        blocked.map_sequence = grid.map_sequence
        blocked.frame_id = grid.frame_id
        self.rear_block_cache[fill] = (key, blocked)
        return blocked

    def search_grid(self):
        # Forward search keeps the original grid, without the virtual rear wall.
        if bool(self.get_parameter('search_stop_on_unknown').value):
            return super().grid(False)
        return super().search_grid()

    def validate_parameters(self, parameters):
        overrides = {p.name: p.value for p in parameters}
        protected = {'slot_width', 'gap_min_lateral_m', 'parking_side', 'manual_survey',
                     'camera_object_labels', 'camera_min_confidence',
                     'require_camera', 'allow_lidar_only_motion', 'camera_evidence_lifetime_s', 'camera_stream_timeout_s'}
        affected = any(name in protected or name.startswith('pair_') for name in overrides)
        try:
            if affected and self.test_motion_enabled:
                raise ValueError('Restart the test before changing perception during automatic operation')
            if overrides.get('parking_side', 'left') != 'left':
                raise ValueError('This test expects parking_side=left')
            if 'manual_survey' in overrides:
                raise ValueError('Select manual_survey at launch, then restart to change it')
            if 'allow_lidar_only_motion' in overrides:
                raise ValueError('Select allow_lidar_only_motion at launch, then restart to change it')
            self.pair_config(overrides)
            self.rear_block_margin(overrides)
            self.camera_filter(overrides)
            for name in ('camera_evidence_lifetime_s', 'camera_stream_timeout_s'):
                if name in overrides and (not math.isfinite(overrides[name]) or overrides[name] <= 0):
                    raise ValueError(name+' must be finite and positive')
            if affected:
                self.observation_window = []
                self.observation_hits = 0
            if {'camera_object_labels', 'camera_min_confidence'} & overrides.keys():
                self.camera_evidence = []
                self.camera_valid = False
                self.last_association_key = None
            return SetParametersResult(successful=True)
        except (ValueError, TypeError) as exc:
            return SetParametersResult(successful=False, reason=str(exc))

    def camera_filter(self, overrides=None):
        values = overrides or {}
        labels = values.get('camera_object_labels', self.get_parameter('camera_object_labels').value)
        minimum = values.get('camera_min_confidence', self.get_parameter('camera_min_confidence').value)
        if (not isinstance(labels, (list, tuple)) or not labels
                or not all(isinstance(label, str) and label.strip() for label in labels)):
            raise ValueError('camera_object_labels must be a nonempty list of labels')
        if type(minimum) not in (float, int) or not math.isfinite(minimum) or not 0 <= minimum <= 1:
            raise ValueError('camera_min_confidence must be finite and within 0..1')
        return labels, minimum

    def lidar_only_motion(self):
        return (not self.get_parameter('require_camera').value
                and bool(self.get_parameter('allow_lidar_only_motion').value))

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
            labels = self.get_parameter('camera_object_labels').value
            minimum = self.get_parameter('camera_min_confidence').value
            selected = []
            for group in groups:
                if not isinstance(group, dict):
                    raise ValueError('invalid association object')
                if group.get('label') not in labels:
                    continue
                if minimum > 0:
                    confidence = group.get('confidence')
                    if (type(confidence) not in (float, int) or not np.isfinite(confidence)
                            or not minimum <= confidence <= 1):
                        continue
                selected.append(group)
            groups = selected
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
        minimum_goal_width = 0.
        if self.footprints is not None:
            config = replace(config, min_gap_length=max(config.min_gap_length,
                                                      float(self.footprints.size[0])+.36))
            minimum_goal_width = float(self.footprints.size[1])+.36
        try:
            evidence = self.evidence_in_map(message.header.frame_id)
        except (ValueError, TransformException) as exc:
            self.camera_reason = str(exc)
            evidence = []
        observation = observe_two_structure_gap(
            grid_from_message(message), road_pose, config,
            camera_evidence=evidence, require_camera=self.get_parameter('require_camera').value,
            minimum_goal_width=minimum_goal_width)
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
            'lidar_only_motion': self.lidar_only_motion(),
            'camera_stream_ready': self.camera_ready(),
            'reason': observation.reason, 'free_fraction': observation.free_fraction,
            'center': target.center.tolist() if target else None,
            'heading': target.heading if target else None,
            'length': target.length if target else None,
            'width': target.width if target else self.get_parameter('slot_width').value,
            'configured_depth_m': self.get_parameter('slot_width').value,
            'depth_source': 'measured rear wall' if observation.rear_wall else 'configured; no rear wall within depth',
            'rear_depth_m': observation.rear_depth_m,
            'structures_found': observation.structures,
            'rejected_pairs': [reason for _, reason in observation.rejected_pairs][:5],
            'bay_polygon': target.polygon().tolist() if target else None,
            'structure_points': [sample_points(c) for c in observation.components],
            'wall_points': [sample_points(w) for w in observation.walls][:6],
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
        # Structure candidates are shown even when no bay could be formed from them.
        for i, component in enumerate(observation.components):
            marker = Marker(header=header, ns='measured_structures', id=i, type=Marker.POINTS,
                            action=Marker.ADD)
            marker.pose.orientation.w = 1.0
            marker.scale.x = marker.scale.y = .06
            marker.color.b = marker.color.a = 1.0
            marker.points = [Point(x=float(x), y=float(y), z=.05) for x, y in sample_points(component)]
            markers.append(marker)
        for i, wall in enumerate(observation.walls):
            marker = Marker(header=header, ns='measured_walls', id=i, type=Marker.POINTS, action=Marker.ADD)
            marker.pose.orientation.w = 1.0
            marker.scale.x = marker.scale.y = .04
            marker.color.r = marker.color.g = marker.color.b = .6
            marker.color.a = 1.0
            marker.points = [Point(x=float(x), y=float(y), z=.03) for x, y in sample_points(wall)]
            markers.append(marker)
        self.test_markers.publish(MarkerArray(markers=markers))

    def stable_observation(self, target):
        if target is None:
            self.observation_window = []
            self.observation_hits = 0
            self.legacy_observation_stamps = []
            return False
        return super().stable_observation(target)

    def legacy_search_stop(self, observation, road_forward):
        if observation.target is None or self.observation_hits < 3 or len(observation.components) != 2:
            return math.inf, False
        along = np.array([math.cos(self.road_pose[2]), math.sin(self.road_pose[2])])
        far_edge = float(np.max((observation.components[1]-self.road_pose[:2])@along))
        rear_extent = float(np.min((np.vstack(self.footprints.world(self.current))-self.current[:2])@along))
        stop_u = far_edge - rear_extent + PLANNER_HARD_CLEARANCE_M
        if road_forward >= stop_u:
            return stop_u, True
        # If forward room runs out, stop with the confirmed target and plan from
        # here. Search still uses its original occupied-only policy by default.
        from observed_stop import safe_stop_sweep
        sweep = safe_stop_sweep(self.search_grid(), self.current, self.footprints, self.geometry,
                                max(abs(self.v), abs(self.measured_speed)), self.delta)
        return stop_u, not sweep['safe']

    def staging_road_coordinate(self, normal):
        if self.stop_policy != 'legacy':
            return super().staging_road_coordinate(normal)
        reference = self.current if self.staging_reference is None else self.staging_reference
        return float(reference[:2]@normal)

    def staging_distances(self, grid, goal, normal):
        if self.stop_policy != 'legacy':
            return super().staging_distances(grid, goal, normal)
        along = np.array([normal[1], -normal[0]])
        reference = self.current if self.staging_reference is None else self.staging_reference
        past_opening = float((self.target.center-goal[:2])@along) + self.target.length/2
        ahead = max(float((reference[:2]-goal[:2])@along), past_opening)
        road_offset = self.staging_road_coordinate(normal) - float(goal[:2]@normal)
        distances = []
        for distance in dict.fromkeys((ahead, ahead+.5, ahead+1., ahead-.5, 7.)):
            if distance < past_opening:
                continue
            endpoint = goal.copy()
            endpoint[:2] += distance*along + road_offset*normal
            if grid.check(endpoint, self.footprints):
                distances.append(distance)
        return tuple(distances)

    def select_goal(self):
        if self.stop_policy != 'legacy':
            return super().select_goal()
        grid = self.grid(False)
        heading = self.target.heading
        normal = np.array([-math.sin(heading), math.cos(heading)])
        for goal, shift, lateral in self.goal_candidates(grid, self.parking_goal_offsets):
            if self.staging_distances(grid, goal, normal):
                self.goal_shift, self.goal_lateral_shift = shift, lateral
                return goal
        raise RuntimeError('no bay-contained goal has an observed-safe staging pose')

    def record_stationary_sample(self, now, state):
        self.stationary_samples.append((now, state.copy()))
        while self.stationary_samples and now-self.stationary_samples[0][0] > 1.1:
            self.stationary_samples.popleft()

    def begin_stabilizing(self, state, now, reason, evidence=None):
        self.stationary_samples.clear()
        self.stop_pose_stamp = self.est.pose_stamp
        super().begin_stabilizing(state, now, reason, evidence)

    def stop_and_stabilize(self, state, now):
        if self.stop_policy != 'legacy':
            return super().stop_and_stabilize(state, now)
        self.send(0., 0.)
        pose_stamp = self.est.pose_stamp
        if not self.streams_ready() or pose_stamp is None:
            self.stationary_samples.clear()
        elif self.stop_pose_stamp is None or pose_stamp > self.stop_pose_stamp:
            if self.stationary_samples and now-self.stationary_samples[-1][0] > .3:
                self.stationary_samples.clear()
            self.stop_pose_stamp = pose_stamp
            self.record_stationary_sample(now, state)
        if self.stationary() and abs(self.measured_speed) < .06:
            self.staging_reference = state.copy()
            self.settle_until = now
            if self.detection_msg is not None:
                stamp = self.detection_msg.header.stamp
                self.observation_stamp = (stamp.sec, stamp.nanosec)
            self.legacy_stop_completed = {'sim_ns': self.get_clock().now().nanoseconds,
                                          'state': state.tolist(),
                                          'measured_speed_m_s': self.measured_speed,
                                          'zero_command': [self.v, self.delta]}
            self.transition('DETECT_TARGET', 'stationary rig; revalidate with new maps')
        elif now >= self.settle_until:
            self.transition('FAILED', self.stop_timeout_reason)
        else:
            self.publish_status()

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
        return bool(np.max(np.linalg.norm(states[:, :2]-states[0, :2], axis=1)) < .025
                    and np.max(np.abs(wrap(states[:, 2:]-states[0, 2:]))) < math.radians(2))

    def manual_guidance(self):
        """Operator advice only: never enables motion or starts parking."""
        result = dict(state='inactive', stop_requested=False, stationary=False,
                      distance_remaining_m=None, reason='손밀기 탐색 노드 대기')
        if self.operation != 'manual' or self.test_motion_enabled:
            return result
        ready = self.streams_ready() and self.camera_ready()
        if self.stage in ('FAILED', 'EMERGENCY_STOP', 'PARKED'):
            self.manual_stop_reason = '테스트 종료 · 밀지 마세요'
        if not ready:
            if self.manual_survey_ready:
                self.manual_stop_reason = self.manual_stop_reason or '센서·위치 추정 끊김 · 멈추고 확인하세요'
            elif self.manual_stop_reason is None:
                return dict(result, state='waiting', reason='준비 안 됨 · 밀지 마세요 (센서·위치 추정·카메라 확인)')
        observation = getattr(self, 'last_gap_observation', None)
        fresh = time.monotonic()-self.last_detect <= 1.0
        if ready and not fresh and self.manual_survey_ready:
            self.manual_stop_reason = self.manual_stop_reason or '공간 탐지 갱신 끊김 · 멈추고 확인하세요'
        if self.manual_stop_reason is not None:
            return dict(result, state='stop', stop_requested=True,
                        stationary=ready and fresh and self.stationary(), reason=self.manual_stop_reason)
        if not ready or not fresh or self.road_pose is None:
            return dict(result, state='waiting', reason='준비 안 됨 · 밀지 마세요 (탐색 데이터 대기)')
        self.manual_survey_ready = True
        if observation is None or observation.target is None:
            return dict(result, state='searching', reason='공간 탐색 중 · 주변을 보며 천천히 미세요')
        if self.observation_hits < 3:
            return dict(result, state='confirming',
                        reason=f'공간 후보 확인 중 · 새 지도 {self.observation_hits}/3장')
        along = np.array([math.cos(self.road_pose[2]), math.sin(self.road_pose[2])])
        forward = float((self.current[:2]-self.road_pose[:2])@along)
        stop_u, stop = self.legacy_search_stop(observation, forward)
        if stop:
            self.manual_stop_reason = ('공간 확인 · 정지 위치 도달' if forward >= stop_u else
                                       '전방 정지 여유 부족 · 멈추고 확인하세요')
            return dict(result, state='stop', stop_requested=True,
                        stationary=self.stationary(), reason=self.manual_stop_reason)
        remaining = max(0., stop_u-forward) if math.isfinite(stop_u) else None
        return dict(result, state='approaching', distance_remaining_m=remaining,
                    reason='공간 확인 · 트레일러 후단이 두 번째 차를 지날 때까지 천천히 미세요')

    def publish_manual_guidance(self):
        advice = self.manual_guidance()
        advice['stamp_ns'] = self.get_clock().now().nanoseconds
        self.manual_guidance_pub.publish(String(data=json.dumps(advice, allow_nan=False)))

    def start_search(self, request, response):
        if self.test_motion_enabled or self.stage in ('FAILED', 'EMERGENCY_STOP', 'PARKED'):
            response.message = 'Restart the test before starting another search'
        elif not self.streams_ready() or not self.camera_ready() or not self.stationary():
            response.message = 'Need current SLAM/IMU/scan/camera and a stationary rig for one second'
        elif not self.get_parameter('require_camera').value and not self.lidar_only_motion():
            response.message = ('LiDAR-only diagnostics cannot start automatic motion; '
                                'launch with allow_lidar_only_motion:=true for the LiDAR-only test stage')
        else:
            self.operation = 'automatic_search'
            self.test_motion_enabled = True
            self.auto = True
            self.transition('WAIT_FOR_LOCALIZATION', 'explicit automatic search command')
            response.success = True
            response.message = ('Automatic forward search -> bay confirmation -> existing B parking'
                                + (' (LiDAR-only test stage; camera reported only)' if self.lidar_only_motion() else ''))
        return response

    def start_parking(self, request, response):
        if self.operation != 'manual' or self.test_motion_enabled:
            response.message = 'start_parking is for manual_survey only; use start_search for final autonomy'
            return response
        if ((not self.get_parameter('require_camera').value and not self.lidar_only_motion())
                or not self.streams_ready()
                or not self.camera_ready() or not self.stationary()):
            response.message = ('Need calibrated camera (or allow_lidar_only_motion), current sensors '
                                'and a stationary rig for one second')
            return response
        try:
            observation = self.detect_gap(self.road_pose, 0., self.get_parameter('slot_width').value)
            if observation.target is None or self.observation_hits < 3:
                raise RuntimeError('Need three stable confirmed bay maps: '+observation.reason)
            self.test_motion_enabled = True
            self.parking_started = True
            self.auto = True
            self.operation = 'parking_after_manual_survey'
            self.begin_stabilizing(self.current, time.monotonic(),
                                   'manual survey complete; explicit parking command')
            response.success = True
            response.message = 'Revalidate bay after settling, then use existing B planner/controller'
        except (RuntimeError, ValueError) as exc:
            response.message = str(exc)
        return response

    def refresh_recognition(self):
        """Publish sensor-based bay recognition without changing motion/plan state."""
        if self.stage not in ('PLAN', 'REPLAN', 'READY', 'EXECUTE',
                              'FAILED', 'EMERGENCY_STOP', 'PARKED') or self.road_pose is None:
            return
        try:
            # detect_gap uses the measured detection_msg, not self.grid(), which
            # may include a virtual wall for parking-path collision checks.
            self.detect_gap(self.road_pose, 0., self.get_parameter('slot_width').value)
        except (RuntimeError, ValueError) as exc:
            self.publish_observation(PairObservation(reason=f'perception update failed: {exc}'),
                Header(frame_id='local_map', stamp=self.get_clock().now().to_msg()))

    def tick_checked(self):
        if self.stage in ('FAILED', 'EMERGENCY_STOP', 'PARKED'):
            return super().tick_checked()
        if self.test_motion_enabled:
            if not self.camera_ready():
                self.send(0., 0.)
                self.transition('EMERGENCY_STOP', 'calibrated camera/scan stream lost')
                return
            super().tick_checked()
            # The base tick returns before the stop handler when localization
            # is missing. That cannot keep a stopped test waiting indefinitely.
            if (self.stop_policy == 'legacy' and self.stage == 'STOP_AND_STABILIZE'
                    and time.monotonic() >= self.settle_until):
                self.transition('FAILED', self.stop_timeout_reason)
            return
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
        self.record_stationary_sample(now, state)
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
