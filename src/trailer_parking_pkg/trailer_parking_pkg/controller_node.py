"""Two entry points share one articulated controller and differ in required sensors."""
from concurrent.futures import ThreadPoolExecutor
import json
import math
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import String
from interfaces_pkg.msg import MotionCommand
from .core import (Geometry, State, Scene, Slot, Tracker, PlanningError, plan,
                   goal_reached, trailer_pose, wrap, advance)
from .actuation import StrollerActuation, motion_values
from .sensors import fresh, scan_obstacles


class ParkingController(Node):
    def __init__(self, use_lidar):
        super().__init__('parallel_parking')
        self.use_lidar = use_lidar
        self.timeout = self.declare_parameter('sensor_timeout', 0.8).value
        self.speed = self.declare_parameter('parking_speed', 0.30).value
        self.motor_pwm = int(self.declare_parameter('parking_pwm', 90).value)
        self.steer_steps = int(self.declare_parameter('steer_steps', 7).value)
        self.planning_timeout = self.declare_parameter('planning_timeout', 50.0).value
        self.scene_bounds = tuple(self.declare_parameter('bounds', [-19.5, 19.5, -26.0, 26.0]).value)
        self.safety_margin = self.declare_parameter('collision_margin', 0.18).value
        self.observations, self.scans = {}, {}
        self.g = Geometry()
        self.actuation = StrollerActuation(max_steer_rad=self.g.max_steer,
            max_steer_step=self.steer_steps, reference_speed_mps=self.speed,
            reference_pwm=self.motor_pwm)
        self.slot, self.tracker, self.future = None, None, None
        self.slot_candidate, self.slot_count, self.last_slot_stamp = None, 0, None
        self.obstacle_memory = {}
        self.phase = ''
        self.fault = None
        self.executor_pool = ThreadPoolExecutor(max_workers=1)
        self.publisher = self.create_publisher(MotionCommand, '/parking/raw_motion_command', 1)
        self.status_pub = self.create_publisher(String, '/parking/status', 5)
        self.path_pub = self.create_publisher(Path, '/parking/path', 1)
        for camera in ('front', 'rear'):
            self.create_subscription(String, f'/parking/{camera}/observation',
                lambda msg, name=camera: self.observe(name, msg), 5)
        if self.use_lidar:
            for name in ('tractor', 'trailer'):
                self.create_subscription(LaserScan, f'/parking/{name}/scan',
                    lambda msg, name=name: self.scan(name, msg), qos_profile_sensor_data)
        self.last_change, self.previous_direction = 0.0, 0
        self.last_position, self.progress_time = None, time.monotonic()
        self.started = time.monotonic()
        self.done_count = 0
        self.search_origin = None
        self.create_timer(0.05, self.tick)

    def observe(self, name, msg):
        try:
            data = json.loads(msg.data)
            if data.get('camera') != name or not isinstance(data['stamp'], (int, float)):
                raise ValueError('Invalid observation header')
            if data.get('valid'):
                if len(data['pose']) != 3 or not all(math.isfinite(v) for v in data['pose']):
                    raise ValueError('Invalid camera pose')
                for poly in data['obstacles']:
                    if len(poly) < 3 or not all(len(p) == 2 and all(math.isfinite(v) for v in p) for p in poly):
                        raise ValueError('Invalid obstacle')
                for slot in data['slots']:
                    if not all(math.isfinite(v) for v in vars(Slot(**slot)).values()):
                        raise ValueError('Invalid slot')
                Geometry(**data['geometry'])
            self.observations[name] = data
        except (ValueError, KeyError, TypeError):
            self.observations.pop(name, None)

    def scan(self, name, msg):
        self.scans[name] = msg

    def status(self, value):
        if self.phase != value:
            self.phase = value
            self.get_logger().info(value)
        msg = String()
        msg.data = value
        self.status_pub.publish(msg)

    def stop(self, why):
        self.publish_motion(0.0, 0.0)
        self.status(why)

    def publish_motion(self, speed, steering):
        values = motion_values(speed, steering, self.actuation)
        msg = MotionCommand()
        msg.steering, msg.left_speed, msg.right_speed = values
        self.publisher.publish(msg)

    def inputs(self, now):
        if any(name not in self.observations for name in ('front', 'rear')):
            raise ValueError('WAIT_CAMERAS')
        front, rear = self.observations['front'], self.observations['rear']
        for data in (front, rear):
            if not data.get('valid') or not fresh(data['stamp'], now, self.timeout):
                raise ValueError('STOP_CAMERA_STALE_OR_UNLOCALIZED')
        if abs(front['stamp']-rear['stamp']) > 0.20:
            raise ValueError('STOP_CAMERA_TIME_SKEW')
        self.g = Geometry(**front['geometry'])
        if front['geometry'] != rear['geometry']:
            raise ValueError('STOP_GEOMETRY_MISMATCH')
        x, y, yaw = front['pose']
        tx, ty, tyaw = rear['pose']
        state = State(x, y, yaw, wrap(tyaw-yaw))
        predicted = trailer_pose(state, self.g)
        if math.hypot(tx-predicted[0], ty-predicted[1]) > 0.35:
            raise ValueError('STOP_VISUAL_HITCH_INCONSISTENT')
        # Preserve parked-car footprints when they leave the camera field of view.
        for data in (front, rear):
            for poly in data['obstacles']:
                poly = tuple(tuple(p) for p in poly)
                center = tuple(round(sum(p[i] for p in poly)/len(poly)/0.75) for i in (0, 1))
                self.obstacle_memory[center] = poly
        obstacles = list(self.obstacle_memory.values())
        if self.use_lidar:
            for name, pose, offset in (('tractor', (x, y, yaw), (3.90, 0.0, 0.0)),
                                       ('trailer', (tx, ty, tyaw), (-1.0, 0.0, 0.0))):
                scan = self.scans.get(name)
                if scan is None:
                    raise ValueError('WAIT_LIDAR')
                stamp = scan.header.stamp.sec+scan.header.stamp.nanosec/1e9
                if not fresh(stamp, now, self.timeout) or abs(stamp-front['stamp']) > 0.25:
                    raise ValueError('STOP_LIDAR_STALE_OR_TIME_SKEW')
                obstacles.extend(scan_obstacles(scan.ranges, scan.angle_min, scan.angle_increment,
                    scan.range_min, scan.range_max, pose, state, self.g, offset))
        return state, Scene(tuple(obstacles), self.scene_bounds, self.safety_margin)

    def acquire_slot(self, state, scene):
        candidates = []
        for data in self.observations.values():
            for item in data['slots']:
                slot = Slot(**item)
                if (slot.length >= self.g.rig_length+0.6 and slot.width >= self.g.width+0.5 and
                        scene.free(slot.goal(self.g), self.g)):
                    candidates.append((slot, data['stamp']))
        if not candidates:
            self.slot_count = 0
            return
        slot, stamp = min(candidates, key=lambda item: math.hypot(item[0].x-state.x, item[0].y-state.y))
        if self.last_slot_stamp == stamp:
            return
        self.last_slot_stamp = stamp
        old = self.slot_candidate
        if old and math.hypot(slot.x-old.x, slot.y-old.y) < 0.25 and abs(wrap(slot.yaw-old.yaw)) < 0.07 and abs(slot.length-old.length) < 0.4:
            self.slot_count += 1
        else:
            self.slot_count = 1
        self.slot_candidate = slot
        if self.slot_count >= 5:
            self.slot = slot

    def tick(self):
        try:
            self.control_tick()
        except Exception as exc:
            self.fault = f'STOP_CONTROLLER_ERROR: {exc}'
            self.stop(self.fault)

    def control_tick(self):
        if self.fault:
            self.stop(self.fault)
            return
        now = self.get_clock().now().nanoseconds/1e9
        try:
            state, scene = self.inputs(now)
        except ValueError as exc:
            self.stop(str(exc))
            return
        if not scene.free(state, self.g):
            self.stop('STOP_COLLISION_OR_ARTICULATION_LIMIT')
            return
        if self.slot is None:
            self.acquire_slot(state, scene)
            if self.slot_count > 0 or self.slot is not None:
                self.stop('ACQUIRE_EMPTY_SLOT_5_FRAMES')
                return
            # Approach only after YOLO has observed a (possibly single-car)
            # parking-space mask, with valid visual motion from both cameras.
            if not self.observations['front']['slots']:
                self.stop('WAIT_VISIBLE_PARKING_SPACE')
                return
            if self.search_origin is None:
                self.search_origin = (state.x, state.y)
            if math.dist((state.x, state.y), self.search_origin) > 10:
                self.fault = 'STOP_SLOT_SEARCH_DISTANCE_LIMIT'
                self.stop(self.fault)
                return
            q = state
            for _ in range(20):
                q = advance(q, 0.05, 0.0, self.g)
                if not scene.free(q, self.g):
                    self.stop('STOP_SEARCH_OBSTACLE')
                    return
            self.publish_motion(0.12, 0.0)
            self.status('APPROACH_VISIBLE_SLOT')
            return
        if goal_reached(state, self.slot, self.g):
            self.done_count += 1
            self.stop('PARKED' if self.done_count >= 10 else 'VERIFY_PARKED')
            if self.done_count >= 10:
                self.fault = 'PARKED'
            return
        self.done_count = 0
        if self.future is not None:
            self.stop('PLANNING')
            if self.future.done():
                try:
                    path = self.future.result()
                except PlanningError as exc:
                    self.fault = f'STOP_NO_PATH: {exc}'
                    return
                self.future = None
                if math.hypot(state.x-path[0].state.x, state.y-path[0].state.y) > 0.20:
                    self.fault = 'STOP_MOVED_DURING_PLANNING'
                    return
                if not all(scene.free(p.state, self.g) for p in path):
                    self.fault = 'STOP_SCENE_CHANGED_DURING_PLANNING'
                    return
                self.tracker = Tracker(path, self.g, self.speed)
                self.publish_path(path)
                self.progress_time = time.monotonic()
            return
        if self.tracker is None:
            self.stop('PLANNING')
            self.future = self.executor_pool.submit(plan, state, self.slot, scene, self.g,
                                                   timeout=self.planning_timeout)
            return
        if time.monotonic()-self.started > 600:
            self.fault = 'STOP_MISSION_TIMEOUT'
            self.stop(self.fault)
            return
        if self.last_position is None or math.dist((state.x, state.y), self.last_position) > 0.08:
            self.last_position, self.progress_time = (state.x, state.y), time.monotonic()
        elif time.monotonic()-self.progress_time > 10:
            self.fault = 'STOP_NO_PROGRESS'
            self.stop(self.fault)
            return
        v, steer = self.tracker.command(state, scene)
        # Check live obstacles over stopping distance, beyond the tracker horizon.
        q = state
        for _ in range(15):
            q = advance(q, v*0.1, steer, self.g)
            if not scene.free(q, self.g):
                self.stop('STOP_LIVE_OBSTACLE')
                return
        direction = 1 if v > 0 else -1 if v < 0 else 0
        if direction and direction != self.previous_direction:
            self.previous_direction, self.last_change = direction, time.monotonic()
        if time.monotonic()-self.last_change < 0.6:
            self.stop('GEAR_CHANGE_SETTLE')
            return
        self.publish_motion(v, steer)
        self.status('TRACK_FORWARD' if v > 0 else 'TRACK_REVERSE' if v < 0 else 'TRACK_HOLD')

    def publish_path(self, path):
        message = Path()
        message.header.frame_id = 'world'
        message.header.stamp = self.get_clock().now().to_msg()
        for step in path:
            pose = PoseStamped()
            pose.header = message.header
            pose.pose.position.x, pose.pose.position.y = step.state.x, step.state.y
            pose.pose.orientation.z = math.sin(step.state.yaw/2)
            pose.pose.orientation.w = math.cos(step.state.yaw/2)
            message.poses.append(pose)
        self.path_pub.publish(message)


def run(use_lidar, args=None):
    rclpy.init(args=args)
    node = ParkingController(use_lidar)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_motion(0.0, 0.0)
        node.executor_pool.shutdown(wait=False, cancel_futures=True)
        node.destroy_node()
        rclpy.shutdown()


def lidar_yolo_main(args=None):
    run(True, args)


def camera_yolo_main(args=None):
    run(False, args)
