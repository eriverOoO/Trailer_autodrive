"""LiDAR-only hand-push observations and conditional plans; no control publisher."""
from collections import deque
from concurrent.futures import ThreadPoolExecutor
import copy
import json
import math
from pathlib import Path
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from scipy.spatial.transform import Rotation
from nav_msgs.msg import OccupancyGrid, Path as RosPath
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan, PointCloud2
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener, TransformException
import yaml

from real_vehicle_bridge.geometry import VehicleGeometry
from .two_structure_gap import PairConfig, PairObservation, observe_two_structure_gap
from .nodes import grid_from_message
from .bootstrap import install_b
from .two_structure_test import block_behind_bay, sample_points
from tractor_trailer_model import Footprints, wrap


def stamp_ns(stamp):
    return stamp.sec*10**9+stamp.nanosec


class HandPushSurvey(Node):
    def __init__(self):
        super().__init__('hand_push_survey')
        for key, value in dict(geometry_file='', test_params_file='', slot_width=0.,
                               planner_timeout=30.).items():
            self.declare_parameter(key, value)
        self.vehicle = VehicleGeometry(self.get_parameter('geometry_file').value)
        self.geometry = self.vehicle.model()
        self.footprints = Footprints(*self.vehicle.polygons(), self.geometry)
        values = {}
        filename = self.get_parameter('test_params_file').value
        if filename:
            values = yaml.safe_load(Path(filename).read_text())['tractor_parallel_parking_auto']['ros__parameters']
        config = {key: values.get('slot_width' if key == 'slot_width' else
                                 'gap_min_lateral_m' if key == 'min_lateral_m' else 'pair_'+key, value)
                  for key, value in vars(PairConfig()).items()}
        if self.get_parameter('slot_width').value:
            config['slot_width'] = self.get_parameter('slot_width').value
        config['min_gap_length'] = max(config['min_gap_length'], float(self.footprints.size[0])+.36)
        self.config = PairConfig(**config)
        g = self.vehicle.values
        # Whole-rig diagonal is a deliberately generous distance from the LiDAR.
        # This is a survey cue, not a measured trailer-rear position.
        self.pass_margin = math.hypot(g['tractor_length']+g['connection_length']+g['trailer_length'],
                                      max(g['tractor_width'], g['trailer_width']))+.18
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        q = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.map = None
        self.receipts = {}
        self.scan = None
        self.sensor_group = ReentrantCallbackGroup()
        self.create_subscription(OccupancyGrid, '/map', self.on_map, q, callback_group=self.sensor_group)
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data, callback_group=self.sensor_group)
        self.create_subscription(PointCloud2, '/scan_matched_points2',
                                 lambda m: self.remember('match', m), qos_profile_sensor_data,
                                 callback_group=self.sensor_group)
        self.pubs = {key: self.create_publisher(String, topic, 1) for key, topic in {
            'guidance': '/parking_test/manual_guidance', 'observation': '/parking_test/observation',
            'status': '/parking/status'}.items()}
        self.path_pub = self.create_publisher(RosPath, '/parking/survey_planned_path', q)
        self.create_service(Trigger, '/parking_test/start_search', self.reject_motion)
        self.create_service(Trigger, '/parking_test/start_parking', self.request_plan)
        self.create_service(Trigger, '/parking_test/stop', self.stop)
        self.road = self.pose = self.previous_pose = None
        self.samples = deque()
        self.pose_key = None
        self.last_detect = 0.
        self.map_key = None
        self.targets = []
        self.observation = PairObservation(reason='waiting for live LiDAR SLAM')
        self.stop_reason = None
        self.armed = False
        self.stage, self.failure = 'WAIT_FOR_SLAM', ''
        self.pool = ThreadPoolExecutor(max_workers=1)
        self.future = None
        self.plan_start = None
        self.plan_invalid = False
        self.plan_attempted = False
        self.create_timer(.1, self.tick)

    def emit(self, key, value):
        self.pubs[key].publish(String(data=json.dumps(value, allow_nan=False)))

    def remember(self, key, message):
        self.receipts[key] = (time.monotonic(), stamp_ns(message.header.stamp))

    def on_map(self, message):
        self.map = message
        self.remember('map', message)

    def on_scan(self, message):
        self.scan = message
        self.remember('scan', message)

    def fresh_pose(self):
        clock = self.get_clock().now().nanoseconds
        for key, limit in (('scan', .6), ('match', .6), ('map', 2.5)):
            wall, stamp = self.receipts.get(key, (0., 0))
            if time.monotonic()-wall > limit or not -.03 <= (clock-stamp)/1e9 <= limit:
                raise ValueError(f'{key} 수신 대기/끊김')
        if self.map is None or self.map.header.frame_id != 'map':
            raise ValueError('map 좌표의 SLAM 지도 대기')
        tf = self.tf.lookup_transform('map', self.scan.header.frame_id, Time())
        if not -.3 <= (clock-stamp_ns(tf.header.stamp))/1e9 <= .6:
            raise ValueError('SLAM 위치 갱신 끊김')
        t, q = tf.transform.translation, tf.transform.rotation
        rotation = Rotation.from_quat([q.x, q.y, q.z, q.w])
        if not np.isfinite([t.x, t.y, t.z]).all():
            raise ValueError('SLAM 위치 형식 오류')
        sensor = np.array([t.x, t.y, rotation.as_euler('xyz')[2]])
        if self.previous_pose is not None and (np.linalg.norm(sensor[:2]-self.previous_pose[:2]) > .5
                or abs(wrap(sensor[2]-self.previous_pose[2])) > math.radians(15)):
            raise ValueError('SLAM 위치 급변 · 멈추고 탐색을 다시 시작하세요')
        self.previous_pose = sensor.copy()
        key = stamp_ns(tf.header.stamp)
        if key != self.pose_key:
            self.pose_key = key
            self.samples.append((time.monotonic(), sensor.copy()))
        while self.samples and time.monotonic()-self.samples[0][0] > 1.2:
            self.samples.popleft()
        mount = self.vehicle.values['lidar_mounting_offset']
        body_rotation = rotation*Rotation.from_euler('xyz', mount[3:]).inv()
        body = np.array([t.x, t.y, t.z])-body_rotation.apply(mount[:3])
        axle = body+body_rotation.apply(self.vehicle.values['tractor_rear_axle_position'])
        heading = body_rotation.as_euler('xyz')[2]
        # Conditional planning input only. Never publish this as /parking/state.
        assumed_state = np.array([axle[0], axle[1], heading, heading])
        return sensor, assumed_state

    def stationary(self):
        if len(self.samples) < 3 or time.monotonic()-self.samples[-1][0] > .3:
            return False
        if self.samples[-1][0]-self.samples[0][0] < .8:
            return False
        poses = np.array([p for _, p in self.samples])
        return bool(np.max(np.linalg.norm(poses[:, :2]-poses[0, :2], axis=1)) < .025
                    and np.max(np.abs(wrap(poses[:, 2]-poses[0, 2]))) < math.radians(2))

    def observe(self):
        m = self.map
        self.observation = observe_two_structure_gap(grid_from_message(m), self.road, self.config,
            require_camera=False, minimum_goal_width=float(self.footprints.size[1])+.36)
        target = self.observation.target
        key = stamp_ns(m.header.stamp)
        if target is None:
            self.targets.clear()
        elif key != self.map_key:
            if any(np.linalg.norm(target.center-old.center) >= .15 or
                   abs(wrap(target.heading-old.heading)) >= math.radians(2) or
                   abs(target.length-old.length) >= .20 or abs(target.width-old.width) >= .15
                   for old in self.targets):
                self.targets.clear()
            self.targets = (self.targets+[target])[-3:]
        self.map_key = key
        obs = self.observation
        target = obs.lidar_target
        self.emit('observation', dict(stamp_ns=self.get_clock().now().nanoseconds,
            map_stamp_ns=key, frame_id=m.header.frame_id, mode='manual', survey_only=True,
            valid=obs.target is not None, lidar_pair_seen=target is not None,
            camera_confirmed=False, require_camera=False, lidar_only_motion=False,
            camera_stream_ready=False, reason=obs.reason, free_fraction=obs.free_fraction,
            center=target.center.tolist() if target else None, heading=target.heading if target else None,
            length=target.length if target else None, width=target.width if target else self.config.slot_width,
            rear_depth_m=obs.rear_depth_m, structures_found=obs.structures,
            rejected_pairs=[reason for _, reason in obs.rejected_pairs][:5],
            bay_polygon=target.polygon().tolist() if target else None,
            structure_points=[sample_points(p) for p in obs.components],
            wall_points=[sample_points(p) for p in obs.walls]))

    def stop_distance(self, pose):
        if self.observation.target is None or len(self.targets) < 3 or len(self.observation.components) != 2:
            return None
        along = np.array([math.cos(self.road[2]), math.sin(self.road[2])])
        far = max(float(np.max((p-self.road[:2])@along)) for p in self.observation.components)
        return far+self.pass_margin-float((pose[:2]-self.road[:2])@along)

    def reject_motion(self, request, response):
        response.message = '이 모드는 손밀기 관찰·참고 경로 계획 전용입니다. 자동 주행은 꺼져 있습니다.'
        return response

    def stop(self, request, response):
        self.stop_reason = '사용자 정지 요청'
        self.invalidate_plan()
        response.success, response.message = True, '정지 안내 유지 · 차량 구동 없음'
        return response

    def invalidate_plan(self):
        if self.plan_start is not None and not self.plan_invalid:
            self.plan_invalid = True
            m = RosPath(); m.header.frame_id = 'map'
            self.path_pub.publish(m)

    def request_plan(self, request, response):
        try:
            _, start = self.fresh_pose()
            if not self.stationary():
                raise ValueError('약 1초간 멈춰야 참고 경로를 계획합니다')
            if self.road is None:
                raise ValueError('탐색 기준 위치 대기')
            self.observe()
            if self.observation.target is None or len(self.targets) < 3:
                raise ValueError('새 지도 3장에서 안정적으로 확인한 공간이 필요합니다')
            if self.future is not None:
                raise ValueError('이미 경로 계획 중입니다')
            snapshot = copy.deepcopy(self.map)
            grid = grid_from_message(snapshot)
            target = self.observation.target
            blocked = snapshot
            blocked.data = block_behind_bay(grid.data, grid.resolution, grid.origin, target).ravel().astype(int).tolist()
            grid = grid_from_message(blocked)
            self.plan_start, self.plan_invalid, self.plan_attempted = start.copy(), False, True
            self.future = self.pool.submit(self.plan, grid, target, start)
            self.stage, self.failure = 'PLAN_PREVIEW', ''
            response.success, response.message = True, '트레일러 직선 가정으로 참고 경로 계획 중 · 구동 없음'
        except (ValueError, RuntimeError, TransformException) as exc:
            response.message = str(exc)
        return response

    def plan(self, grid, target, start):
        module = install_b()
        from tractor_parking_clearance import footprint_clearances, PLANNER_HARD_CLEARANCE_M
        along = np.array([math.cos(target.heading), math.sin(target.heading)])
        goals = [self.footprints.goal(target.center-shift*along, target.heading) for shift in (.4, 0., .2, .6)]
        goals = [g for g in goals if target.contains(self.footprints.world(g)) and grid.check(g, self.footprints)]
        if not goals:
            raise RuntimeError('관측 지도에서 차체 전체가 들어갈 목표 자세가 없습니다')
        # One bounded attempt; failures remain visible, never relax occupied/unknown cells.
        planner = module.HybridAStar(self.geometry, self.footprints, grid,
                                     timeout=self.get_parameter('planner_timeout').value)
        result = planner.plan(start, goals[0], (1, -1))
        if not grid.path_free(result.states, self.footprints) or np.any(
                footprint_clearances(grid, result.states, self.footprints) <= PLANNER_HARD_CLEARANCE_M):
            raise RuntimeError('참고 경로의 전체 차체 충돌 여유 검증 실패')
        return result

    def tick(self):
        ready, reason, sensor, assumed = False, '', None, None
        try:
            sensor, assumed = self.fresh_pose()
            ready = True
            if self.road is None:
                if not self.stationary():
                    raise ValueError('출발 위치에서 약 1초간 멈춰 주세요')
                self.road = sensor.copy()
            self.armed = True
            if time.monotonic()-self.last_detect >= .5:
                self.observe()
                self.last_detect = time.monotonic()
            if self.stop_reason is None:
                remaining = self.stop_distance(sensor)
                if remaining is not None and remaining <= 0:
                    self.stop_reason = '라이다 공간 확인 · 보수적 통과 위치 도달 (관절각 미측정)'
            if self.plan_start is not None and (np.linalg.norm(assumed[:2]-self.plan_start[:2]) > .05
                    or abs(wrap(assumed[2]-self.plan_start[2])) > math.radians(3)):
                self.invalidate_plan()
                self.failure = '계획 시작 위치에서 이동 · 참고 경로 무효'
            if (self.stop_reason and '통과 위치' in self.stop_reason and self.stationary()
                    and not self.plan_attempted):
                self.request_plan(Trigger.Request(), Trigger.Response())
        except (ValueError, RuntimeError, TransformException) as exc:
            ready, reason = False, str(exc)
            self.invalidate_plan()
            if self.armed:
                self.stop_reason = self.stop_reason or reason+' · 멈추고 확인하세요'
        if self.future is not None and self.future.done():
            future, self.future = self.future, None
            try:
                result = future.result()
                if self.plan_invalid:
                    raise RuntimeError('계획 중 이동/센서 끊김 · 참고 경로 무효')
                m = RosPath(); m.header.frame_id = 'map'; m.header.stamp = self.get_clock().now().to_msg()
                for state in result.states:
                    p = PoseStamped(); p.header = m.header
                    p.pose.position.x, p.pose.position.y = map(float, state[:2])
                    p.pose.orientation.z, p.pose.orientation.w = math.sin(state[2]/2), math.cos(state[2]/2)
                    m.poses.append(p)
                self.path_pub.publish(m)
                self.stage = 'PREVIEW_READY'
            except Exception as exc:
                self.stage, self.failure = 'PREVIEW_FAILED', str(exc)
        remaining = self.stop_distance(sensor) if ready and sensor is not None else None
        state = ('stop' if self.stop_reason else 'waiting' if not ready else
                 'approaching' if remaining is not None else 'confirming' if self.targets else 'searching')
        text = self.stop_reason or reason or ('공간 확인 · 보수적 통과 위치까지 천천히 미세요' if remaining is not None else
               f'라이다 공간 탐색 중 · 확인 {len(self.targets)}/3장 · 카메라는 참고 영상')
        self.emit('guidance', dict(stamp_ns=self.get_clock().now().nanoseconds, state=state,
            reason=text, stop_requested=bool(self.stop_reason), stationary=ready and self.stationary(),
            distance_remaining_m=max(0., remaining) if remaining is not None else None, survey_only=True))
        self.emit('status', dict(state=self.stage if self.plan_attempted else '손밀기 탐색' if ready else '준비 대기',
            failure=self.failure or reason, survey_only=True, localization_ready=ready,
            articulation_measured=False, camera_association=False))

    def close(self):
        self.pool.shutdown(wait=False, cancel_futures=True)


def main(args=None):
    rclpy.init(args=args)
    node = HandPushSurvey()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.close(); node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
