import math

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSReliabilityPolicy

from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from visualization_msgs.msg import Marker

from interfaces_pkg.msg import (
    DetectionArray,
    LaneInfo,
    MotionCommand,
    ParkingSpaceDetectionInfo,
)
from .lib import decision_making_func_lib as DMFL


#---------------Variable Setting---------------
SUB_PARKING_SPACE_BBOX_TOPIC_NAME = "yolov8_parking_space_detection_info"
SUB_PARKING_SPACE_LANE_TOPIC_NAME = "yolov8_parking_space_info"
SUB_PARKING_SPACE_ROI_TOPIC_NAME = "roi_image_parking_space"
SUB_REAR_DETECTION_TOPIC_NAME = "/rear/detections"
PUB_TOPIC_NAME = "topic_control_signal"
PUB_PARKING_START_MARKER_TOPIC_NAME = "parking_start_marker"
PUB_PARKING_START_STATUS_TOPIC_NAME = "parking_start_status"
PUB_END_LINE_LOST_DEBUG_TOPIC_NAME = "debug_end_line_lost"
#----------------------------------------------

TIMER = 0.05

# 뒤 카메라 parking_space target point가 이 시간 이상 새로 안 들어오면 끊긴 것으로 판단
TARGET_TIMEOUT = 0.5

# 후진 Stanley 중 end_line을 한 번 본 뒤 이 시간 이상 안 보이면 정지한다.
END_LINE_CLASS_NAME = "end_line"
END_LINE_TIMEOUT = 1.0

# ============================================================
# 주차공간 탐색 전 초기 하드코딩 시퀀스 설정
# ============================================================
INITIAL_RIGHT_FORWARD_STEERING = 7
INITIAL_RIGHT_FORWARD_SPEED = 70
INITIAL_RIGHT_FORWARD_DURATION = 6.0

INITIAL_LEFT_FORWARD_STEERING = -7
INITIAL_LEFT_FORWARD_SPEED = 70
INITIAL_LEFT_FORWARD_DURATION = 6.5

INITIAL_STRAIGHT_REVERSE_STEERING = 0
INITIAL_STRAIGHT_REVERSE_SPEED = -70
INITIAL_STRAIGHT_REVERSE_DURATION = 8.0

# 초기 시퀀스가 끝난 뒤 직진하며 parking_space를 찾는다.
SEARCH_FORWARD_STEERING = 0
SEARCH_FORWARD_SPEED = 70

LEFT_FORWARD_STEERING = -7
LEFT_FORWARD_SPEED = 100
LEFT_FORWARD_DURATION = 9.0 # 9.0

REVERSE_SEARCH_STEERING = 7
REVERSE_SEARCH_SPEED = -80
RIGHT_REVERSE_DURATION = 4.5
REVERSE_WAIT_STEERING = 0
REVERSE_WAIT_SPEED = -80
REVERSE_LOST_TARGET_SPEED = -50

# ============================================================
# end_line 소실 이후 출차 시퀀스 설정
# ============================================================
STOP_DURATION = 4.0

FIRST_STRAIGHT_FORWARD_STEERING = 0
FIRST_STRAIGHT_FORWARD_SPEED = 255
FIRST_STRAIGHT_FORWARD_DURATION = 1.0

RIGHT_FORWARD_STEERING = 7
RIGHT_FORWARD_SPEED = 255
RIGHT_FORWARD_DURATION = 7.0

FINAL_STRAIGHT_FORWARD_STEERING = 0
FINAL_STRAIGHT_FORWARD_SPEED = 255
FINAL_STRAIGHT_FORWARD_DURATION = 7.0

# ROI 흰색 픽셀 상태는 디버그용으로 유지한다.
WHITE_LANE_TIMEOUT = 1.0
WHITE_LANE_LOST_REVERSE_DURATION = 3.0
WHITE_LANE_LOST_REVERSE_STEERING = 0
WHITE_LANE_LOST_REVERSE_SPEED = -50

# ROI 이미지의 위쪽 150픽셀에서 흰색 픽셀을 검사한다.
# 화면 아래쪽의 흰색 가로 테두리는 검사 범위에서 제외된다.
WHITE_LANE_CHECK_Y_MAX = 150
WHITE_LANE_BINARY_THRESHOLD = 127
WHITE_LANE_MIN_PIXEL_COUNT = 30

# ============================================================
# 앞 카메라 parking_space bbox 감지 설정
# ============================================================
# 현재 bbox 맨 위 선이 y=330보다 아래에 있으면 주차 시작 조건으로 판단한다.
# bbox_y_max = bbox_center_y - bbox_height / 2
# ============================================================
PARKING_SPACE_BOTTOM_Y_MIN = 320 # 330

PARKING_SPACE_DETECTION_CONFIRM_COUNT = 3

PARKING_START_MARKER_FRAME_ID = "map"
PARKING_START_MARKER_TEXT = "parking start"
PARKING_START_MARKER_LIFETIME = 5.0

# ============================================================
# 후진용 유사 Stanley 제어 설정
# ============================================================
REVERSE_CAR_CENTER_X = 320.0 # 320.0
REVERSE_STANLEY_HEADING_GAIN = 0.50 # 0.35
REVERSE_STANLEY_CROSS_TRACK_GAIN = 0.55 # 0.70
REVERSE_STANLEY_SOFTENING = 70.0 # 40.0
REVERSE_TARGET_PATH_INDEX_OFFSET = 5 # 5
REVERSE_PATH_HEADING_LOOKAHEAD_OFFSET = 5 # 5
REVERSE_MAX_STEER_RAD = 0.50  # 0.50
REVERSE_MAX_STEERING_COMMAND = 7 # 7
REVERSE_STANLEY_SPEED = -80

# 후진에서 실제 조향 반응이 반대로 나오면 이 값을 False로 바꿔서 테스트
REVERSE_STEERING_SIGN_REVERSE = True


class MotionPlanningNode(Node):
    def __init__(self):
        super().__init__('motion_planner_node')

        self.sub_parking_space_bbox_topic = self.declare_parameter(
            'sub_parking_space_bbox_topic',
            SUB_PARKING_SPACE_BBOX_TOPIC_NAME
        ).value

        self.sub_parking_space_lane_topic = self.declare_parameter(
            'sub_parking_space_lane_topic',
            SUB_PARKING_SPACE_LANE_TOPIC_NAME
        ).value

        self.sub_parking_space_roi_topic = self.declare_parameter(
            'sub_parking_space_roi_topic',
            SUB_PARKING_SPACE_ROI_TOPIC_NAME
        ).value

        self.sub_rear_detection_topic = self.declare_parameter(
            'sub_rear_detection_topic',
            SUB_REAR_DETECTION_TOPIC_NAME
        ).value

        self.pub_topic = self.declare_parameter(
            'pub_topic',
            PUB_TOPIC_NAME
        ).value

        self.pub_parking_start_marker_topic = self.declare_parameter(
            'pub_parking_start_marker_topic',
            PUB_PARKING_START_MARKER_TOPIC_NAME
        ).value

        self.pub_parking_start_status_topic = self.declare_parameter(
            'pub_parking_start_status_topic',
            PUB_PARKING_START_STATUS_TOPIC_NAME
        ).value

        self.pub_end_line_lost_debug_topic = self.declare_parameter(
            'pub_end_line_lost_debug_topic',
            PUB_END_LINE_LOST_DEBUG_TOPIC_NAME
        ).value

        self.parking_start_marker_frame_id = self.declare_parameter(
            'parking_start_marker_frame_id',
            PARKING_START_MARKER_FRAME_ID
        ).value

        self.timer_period = self.declare_parameter(
            'timer',
            TIMER
        ).value

        self.qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1
        )

        self.parking_start_status_qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            depth=1
        )

        self.cv_bridge = CvBridge()

        # ============================================================
        # Input Data
        # ============================================================
        self.parking_space_bbox_data = None
        self.path_data = None
        self.path_last_time = None

        self.white_lane_visible = False
        self.white_lane_last_seen_time = None
        self.white_lane_pixel_count = 0

        self.end_line_seen_once = False
        self.end_line_visible = False
        self.end_line_last_seen_time = None
        self.end_line_last_detection_time = None

        self.parking_space_y_max = None
        self.parking_space_detection_confirm_count = 0

        # ============================================================
        # State
        # ============================================================
        self.current_mode = "initial_right_forward"
        self.initial_right_forward_start_time = self.get_clock().now()
        self.initial_left_forward_start_time = None
        self.initial_straight_reverse_start_time = None
        self.left_forward_start_time = None
        self.right_reverse_start_time = None
        self.reverse_follow_start_time = None
        self.white_lane_lost_reverse_start_time = None
        self.stop_start_time = None
        self.first_straight_forward_start_time = None
        self.right_forward_start_time = None
        self.final_straight_forward_start_time = None

        # ============================================================
        # Output / Debug
        # ============================================================
        self.steering_command = 0
        self.left_speed_command = 0
        self.right_speed_command = 0

        self.debug_parking_space_right_raw = False
        self.debug_parking_space_right_stable = False
        self.debug_parking_space_y_max = 0.0

        self.debug_target_slope = 0.0
        self.debug_heading_error = 0.0
        self.debug_cross_track_error = 0.0
        self.debug_end_line_lost = False

        # ============================================================
        # Subscriber
        # ============================================================
        self.parking_space_bbox_sub = self.create_subscription(
            ParkingSpaceDetectionInfo,
            self.sub_parking_space_bbox_topic,
            self.parking_space_bbox_callback,
            self.qos_profile
        )

        self.parking_space_lane_sub = self.create_subscription(
            LaneInfo,
            self.sub_parking_space_lane_topic,
            self.parking_space_lane_callback,
            self.qos_profile
        )

        self.parking_space_roi_sub = self.create_subscription(
            Image,
            self.sub_parking_space_roi_topic,
            self.parking_space_roi_callback,
            self.qos_profile
        )

        self.rear_detection_sub = self.create_subscription(
            DetectionArray,
            self.sub_rear_detection_topic,
            self.rear_detection_callback,
            self.qos_profile
        )

        # ============================================================
        # Publisher
        # ============================================================
        self.publisher = self.create_publisher(
            MotionCommand,
            self.pub_topic,
            self.qos_profile
        )

        self.parking_start_marker_publisher = self.create_publisher(
            Marker,
            self.pub_parking_start_marker_topic,
            self.qos_profile
        )

        self.parking_start_status_publisher = self.create_publisher(
            Bool,
            self.pub_parking_start_status_topic,
            self.parking_start_status_qos_profile
        )

        self.end_line_lost_debug_publisher = self.create_publisher(
            Bool,
            self.pub_end_line_lost_debug_topic,
            self.parking_start_status_qos_profile
        )

        self.timer = self.create_timer(self.timer_period, self.timer_callback)

    # ============================================================
    # Callback Functions
    # ============================================================
    def parking_space_bbox_callback(self, msg: ParkingSpaceDetectionInfo):
        self.parking_space_bbox_data = msg

        if msg.parking_space_detected:
            center_y = float(msg.bbox_center_y)
            bbox_height = float(msg.bbox_height)
            y_max = center_y - bbox_height / 2.0

            self.parking_space_y_max = y_max
        else:
            self.clear_parking_space_bbox_history()

    def parking_space_lane_callback(self, msg: LaneInfo):
        if len(msg.target_points) == 0:
            self.path_data = None
            self.path_last_time = None
            return

        self.path_data = [
            (float(point.target_x), float(point.target_y))
            for point in msg.target_points
        ]
        self.path_last_time = self.get_clock().now()

    def parking_space_roi_callback(self, msg: Image):
        try:
            roi_image = self.cv_bridge.imgmsg_to_cv2(
                msg,
                desired_encoding='mono8'
            )
        except Exception as e:
            self.get_logger().error(
                f"Failed to convert parking space ROI image: {e}"
            )
            return

        if roi_image is None or roi_image.size == 0:
            self.white_lane_visible = False
            self.white_lane_pixel_count = 0
            return

        if len(roi_image.shape) == 3:
            roi_image = cv2.cvtColor(roi_image, cv2.COLOR_BGR2GRAY)

        image_height = roi_image.shape[0]
        check_y_max = min(WHITE_LANE_CHECK_Y_MAX, image_height)
        check_image = roi_image[0:check_y_max, :]

        self.white_lane_pixel_count = int(
            np.count_nonzero(
                check_image >= WHITE_LANE_BINARY_THRESHOLD
            )
        )

        self.white_lane_visible = (
            self.white_lane_pixel_count >= WHITE_LANE_MIN_PIXEL_COUNT
        )

        if self.white_lane_visible:
            self.white_lane_last_seen_time = self.get_clock().now()

    def rear_detection_callback(self, msg: DetectionArray):
        now = self.get_clock().now()
        self.end_line_last_detection_time = now

        end_line_detected = any(
            detection.class_name == END_LINE_CLASS_NAME
            for detection in msg.detections
        )

        self.end_line_visible = end_line_detected

        if (
            self.current_mode == "reverse_follow"
            and end_line_detected
        ):
            self.end_line_seen_once = True
            self.end_line_last_seen_time = now

    def clear_parking_space_bbox_history(self):
        self.parking_space_y_max = None
        self.parking_space_detection_confirm_count = 0
        self.debug_parking_space_y_max = 0.0

    # ============================================================
    # Detection Helpers
    # ============================================================
    def is_parking_space_in_right_area(self):
        if self.parking_space_y_max is None:
            self.debug_parking_space_right_raw = False
            self.debug_parking_space_y_max = 0.0
            return False

        self.debug_parking_space_y_max = self.parking_space_y_max

        detected_raw = (
            self.parking_space_y_max > PARKING_SPACE_BOTTOM_Y_MIN
        )

        self.debug_parking_space_right_raw = detected_raw
        return detected_raw

    def is_parking_space_confirmed_for_left_turn(self):
        if self.is_parking_space_in_right_area():
            self.parking_space_detection_confirm_count += 1

            if (
                self.parking_space_detection_confirm_count
                >= PARKING_SPACE_DETECTION_CONFIRM_COUNT
            ):
                self.debug_parking_space_right_stable = True
                return True

            self.debug_parking_space_right_stable = False
            return False

        self.parking_space_detection_confirm_count = 0
        self.debug_parking_space_right_stable = False
        return False

    def is_target_point_detected(self):
        if self.path_data is None:
            return False

        if len(self.path_data) < 5:
            return False

        if self.path_last_time is None:
            return False

        now = self.get_clock().now()
        path_age = (now - self.path_last_time).nanoseconds / 1e9

        return path_age <= TARGET_TIMEOUT

    def is_white_lane_lost_for_timeout(self, now):
        if self.reverse_follow_start_time is None:
            return False

        reference_time_ns = self.reverse_follow_start_time.nanoseconds

        if self.white_lane_last_seen_time is not None:
            reference_time_ns = max(
                reference_time_ns,
                self.white_lane_last_seen_time.nanoseconds
            )

        white_lane_missing_time = (
            now.nanoseconds - reference_time_ns
        ) / 1e9

        return white_lane_missing_time >= WHITE_LANE_TIMEOUT

    def reset_end_line_state(self):
        self.end_line_seen_once = False
        self.end_line_visible = False
        self.end_line_last_seen_time = None
        self.debug_end_line_lost = False

    def is_end_line_lost_for_timeout(self, now):
        if not self.end_line_seen_once:
            self.debug_end_line_lost = False
            return False

        if self.end_line_last_seen_time is None:
            self.debug_end_line_lost = False
            return False

        end_line_missing_time = (
            now.nanoseconds - self.end_line_last_seen_time.nanoseconds
        ) / 1e9

        self.debug_end_line_lost = end_line_missing_time >= END_LINE_TIMEOUT
        return self.debug_end_line_lost

    # ============================================================
    # Control Helpers
    # ============================================================
    def clamp(self, value, lower, upper):
        return max(lower, min(upper, value))

    def get_current_time_sec(self):
        return self.get_clock().now().nanoseconds / 1e9

    def set_command(self, steering, speed):
        self.steering_command = int(steering)
        self.left_speed_command = int(speed)
        self.right_speed_command = int(speed)

    def publish_parking_start_marker(self, now):
        marker = Marker()
        marker.header.frame_id = self.parking_start_marker_frame_id
        marker.header.stamp = now.to_msg()

        marker.ns = "parking_status"
        marker.id = 0
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD

        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 2.0
        marker.pose.orientation.w = 1.0

        marker.scale.z = 0.8

        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        marker.lifetime.sec = int(PARKING_START_MARKER_LIFETIME)
        marker.lifetime.nanosec = int(
            (PARKING_START_MARKER_LIFETIME % 1.0) * 1e9
        )
        marker.text = PARKING_START_MARKER_TEXT

        self.parking_start_marker_publisher.publish(marker)

    def publish_parking_start_status(self):
        msg = Bool()
        msg.data = True
        self.parking_start_status_publisher.publish(msg)

    def publish_end_line_lost_debug_status(self):
        msg = Bool()
        msg.data = self.debug_end_line_lost
        self.end_line_lost_debug_publisher.publish(msg)

    def calculate_reverse_pseudo_stanley_command(self):
        if self.path_data is None or len(self.path_data) < 10:
            self.debug_target_slope = 0.0
            self.debug_heading_error = 0.0
            self.debug_cross_track_error = 0.0
            return 0, 0, 0

        target_index = min(
            len(self.path_data) - 1,
            REVERSE_TARGET_PATH_INDEX_OFFSET
        )
        target_point = self.path_data[target_index]

        next_index = min(
            len(self.path_data) - 1,
            target_index + REVERSE_PATH_HEADING_LOOKAHEAD_OFFSET
        )
        next_point = self.path_data[next_index]

        path_dx = float(next_point[0] - target_point[0])
        path_dy = float(next_point[1] - target_point[1])

        target_slope = DMFL.calculate_slope_between_points(
            target_point,
            next_point
        )
        self.debug_target_slope = target_slope

        if abs(path_dy) < 1e-6 and abs(path_dx) < 1e-6:
            self.debug_heading_error = 0.0
            self.debug_cross_track_error = 0.0
            return 0, REVERSE_STANLEY_SPEED, REVERSE_STANLEY_SPEED

        # ROI 좌표계 기준 path heading 계산
        path_heading = math.atan2(path_dx, path_dy)
        vehicle_heading = 0.0

        heading_error = path_heading - vehicle_heading
        heading_error = math.atan2(
            math.sin(heading_error),
            math.cos(heading_error)
        )

        cross_track_error = float(target_point[0] - REVERSE_CAR_CENTER_X)
        lateral_term = math.atan2(
            REVERSE_STANLEY_CROSS_TRACK_GAIN * cross_track_error,
            REVERSE_STANLEY_SOFTENING + abs(REVERSE_STANLEY_SPEED)
        )

        steering_rad = (
            REVERSE_STANLEY_HEADING_GAIN * heading_error + lateral_term
        )

        if REVERSE_STEERING_SIGN_REVERSE:
            steering_rad = -steering_rad

        steering_rad = self.clamp(
            steering_rad,
            -REVERSE_MAX_STEER_RAD,
            REVERSE_MAX_STEER_RAD
        )

        steering_command = int(
            round(
                (steering_rad / REVERSE_MAX_STEER_RAD)
                * REVERSE_MAX_STEERING_COMMAND
            )
        )
        steering_command = int(
            self.clamp(
                steering_command,
                -REVERSE_MAX_STEERING_COMMAND,
                REVERSE_MAX_STEERING_COMMAND
            )
        )

        self.debug_heading_error = math.degrees(heading_error)
        self.debug_cross_track_error = cross_track_error

        return steering_command, REVERSE_STANLEY_SPEED, REVERSE_STANLEY_SPEED

    # ============================================================
    # Main Timer Callback
    # ============================================================
    def timer_callback(self):
        now = self.get_clock().now()

        if self.current_mode == "initial_right_forward":
            elapsed = (
                now - self.initial_right_forward_start_time
            ).nanoseconds / 1e9

            if elapsed < INITIAL_RIGHT_FORWARD_DURATION:
                self.set_command(
                    INITIAL_RIGHT_FORWARD_STEERING,
                    INITIAL_RIGHT_FORWARD_SPEED
                )
            else:
                self.current_mode = "initial_left_forward"
                self.initial_right_forward_start_time = None
                self.initial_left_forward_start_time = now
                self.set_command(
                    INITIAL_LEFT_FORWARD_STEERING,
                    INITIAL_LEFT_FORWARD_SPEED
                )

        elif self.current_mode == "initial_left_forward":
            elapsed = (
                now - self.initial_left_forward_start_time
            ).nanoseconds / 1e9

            if elapsed < INITIAL_LEFT_FORWARD_DURATION:
                self.set_command(
                    INITIAL_LEFT_FORWARD_STEERING,
                    INITIAL_LEFT_FORWARD_SPEED
                )
            else:
                self.current_mode = "initial_straight_reverse"
                self.initial_left_forward_start_time = None
                self.initial_straight_reverse_start_time = now
                self.set_command(
                    INITIAL_STRAIGHT_REVERSE_STEERING,
                    INITIAL_STRAIGHT_REVERSE_SPEED
                )

        elif self.current_mode == "initial_straight_reverse":
            elapsed = (
                now - self.initial_straight_reverse_start_time
            ).nanoseconds / 1e9

            if elapsed < INITIAL_STRAIGHT_REVERSE_DURATION:
                self.set_command(
                    INITIAL_STRAIGHT_REVERSE_STEERING,
                    INITIAL_STRAIGHT_REVERSE_SPEED
                )
            else:
                self.current_mode = "search_forward"
                self.initial_straight_reverse_start_time = None
                self.set_command(
                    SEARCH_FORWARD_STEERING,
                    SEARCH_FORWARD_SPEED
                )

        elif self.current_mode == "search_forward":
            if self.is_parking_space_confirmed_for_left_turn():
                self.current_mode = "left_forward"
                self.left_forward_start_time = now
                self.publish_parking_start_marker(now)
                self.publish_parking_start_status()
                self.set_command(LEFT_FORWARD_STEERING, LEFT_FORWARD_SPEED)
            else:
                self.set_command(SEARCH_FORWARD_STEERING, SEARCH_FORWARD_SPEED)

        elif self.current_mode == "left_forward":
            elapsed = (
                now - self.left_forward_start_time
            ).nanoseconds / 1e9

            if elapsed < LEFT_FORWARD_DURATION:
                self.set_command(LEFT_FORWARD_STEERING, LEFT_FORWARD_SPEED)
            else:
                self.current_mode = "right_reverse"
                self.left_forward_start_time = None
                self.right_reverse_start_time = now
                self.reset_end_line_state()
                self.set_command(REVERSE_SEARCH_STEERING, REVERSE_SEARCH_SPEED)

        elif self.current_mode == "right_reverse":
            elapsed = (
                now - self.right_reverse_start_time
            ).nanoseconds / 1e9

            if elapsed < RIGHT_REVERSE_DURATION:
                self.set_command(REVERSE_SEARCH_STEERING, REVERSE_SEARCH_SPEED)
            else:
                self.current_mode = "reverse_follow"
                self.right_reverse_start_time = None
                self.reverse_follow_start_time = now
                self.reset_end_line_state()

                if self.is_target_point_detected():
                    (
                        self.steering_command,
                        self.left_speed_command,
                        self.right_speed_command
                    ) = self.calculate_reverse_pseudo_stanley_command()
                else:
                    self.set_command(REVERSE_WAIT_STEERING, REVERSE_WAIT_SPEED)

        elif self.current_mode == "reverse_follow":
            if self.is_end_line_lost_for_timeout(now):
                self.current_mode = "white_lane_lost_reverse"
                self.reverse_follow_start_time = None
                self.white_lane_lost_reverse_start_time = now
                self.set_command(
                    WHITE_LANE_LOST_REVERSE_STEERING,
                    WHITE_LANE_LOST_REVERSE_SPEED
                )
            elif self.is_target_point_detected():
                (
                    self.steering_command,
                    self.left_speed_command,
                    self.right_speed_command
                ) = self.calculate_reverse_pseudo_stanley_command()
            else:
                # 경로가 잠시 끊겨도 오른쪽 조향 7을 다시 넣지 않고
                # 조향 0으로 후진하면서 뒤 카메라 경로 재수신을 기다린다.
                self.set_command(
                    REVERSE_WAIT_STEERING,
                    REVERSE_LOST_TARGET_SPEED
                )

        elif self.current_mode == "white_lane_lost_reverse":
            elapsed = (
                now - self.white_lane_lost_reverse_start_time
            ).nanoseconds / 1e9

            if elapsed < WHITE_LANE_LOST_REVERSE_DURATION:
                self.set_command(
                    WHITE_LANE_LOST_REVERSE_STEERING,
                    WHITE_LANE_LOST_REVERSE_SPEED
                )
            else:
                self.current_mode = "stop"
                self.white_lane_lost_reverse_start_time = None
                self.stop_start_time = now
                self.set_command(0, 0)

        elif self.current_mode == "stop":
            elapsed = (
                now - self.stop_start_time
            ).nanoseconds / 1e9

            if elapsed < STOP_DURATION:
                self.set_command(0, 0)
            else:
                self.current_mode = "first_straight_forward"
                self.stop_start_time = None
                self.first_straight_forward_start_time = now
                self.set_command(
                    FIRST_STRAIGHT_FORWARD_STEERING,
                    FIRST_STRAIGHT_FORWARD_SPEED
                )

        elif self.current_mode == "first_straight_forward":
            elapsed = (
                now - self.first_straight_forward_start_time
            ).nanoseconds / 1e9

            if elapsed < FIRST_STRAIGHT_FORWARD_DURATION:
                self.set_command(
                    FIRST_STRAIGHT_FORWARD_STEERING,
                    FIRST_STRAIGHT_FORWARD_SPEED
                )
            else:
                self.current_mode = "right_forward"
                self.first_straight_forward_start_time = None
                self.right_forward_start_time = now
                self.set_command(
                    RIGHT_FORWARD_STEERING,
                    RIGHT_FORWARD_SPEED
                )

        elif self.current_mode == "right_forward":
            elapsed = (
                now - self.right_forward_start_time
            ).nanoseconds / 1e9

            if elapsed < RIGHT_FORWARD_DURATION:
                self.set_command(
                    RIGHT_FORWARD_STEERING,
                    RIGHT_FORWARD_SPEED
                )
            else:
                self.current_mode = "final_straight_forward"
                self.right_forward_start_time = None
                self.final_straight_forward_start_time = now
                self.set_command(
                    FINAL_STRAIGHT_FORWARD_STEERING,
                    FINAL_STRAIGHT_FORWARD_SPEED
                )

        elif self.current_mode == "final_straight_forward":
            elapsed = (
                now - self.final_straight_forward_start_time
            ).nanoseconds / 1e9

            if elapsed < FINAL_STRAIGHT_FORWARD_DURATION:
                self.set_command(
                    FINAL_STRAIGHT_FORWARD_STEERING,
                    FINAL_STRAIGHT_FORWARD_SPEED
                )
            else:
                self.current_mode = "final_stop"
                self.final_straight_forward_start_time = None
                self.set_command(0, 0)

        elif self.current_mode == "final_stop":
            self.set_command(0, 0)

        else:
            self.current_mode = "initial_right_forward"
            self.initial_right_forward_start_time = now
            self.initial_left_forward_start_time = None
            self.initial_straight_reverse_start_time = None
            self.set_command(
                INITIAL_RIGHT_FORWARD_STEERING,
                INITIAL_RIGHT_FORWARD_SPEED
            )

        self.get_logger().info(
            f"mode: {self.current_mode}, "
            f"parking_space_right_raw: {self.debug_parking_space_right_raw}, "
            f"parking_space_right_stable: "
            f"{self.debug_parking_space_right_stable}, "
            f"parking_space_y_max: "
            f"{self.debug_parking_space_y_max:.1f}, "
            f"target_received: {self.is_target_point_detected()}, "
            f"target_count: {len(self.path_data) if self.path_data else 0}, "
            f"white_lane_visible: {self.white_lane_visible}, "
            f"white_lane_pixels: {self.white_lane_pixel_count}, "
            f"end_line_visible: {self.end_line_visible}, "
            f"end_line_seen_once: {self.end_line_seen_once}, "
            f"end_line_lost: {self.debug_end_line_lost}, "
            f"target_slope: {self.debug_target_slope:.2f}, "
            f"heading_error_deg: {self.debug_heading_error:.2f}, "
            f"cross_track_error: {self.debug_cross_track_error:.2f}, "
            f"steering: {self.steering_command}, "
            f"left_speed: {self.left_speed_command}, "
            f"right_speed: {self.right_speed_command}"
        )

        motion_command_msg = MotionCommand()
        motion_command_msg.steering = self.steering_command
        motion_command_msg.left_speed = self.left_speed_command
        motion_command_msg.right_speed = self.right_speed_command

        self.publisher.publish(motion_command_msg)
        self.publish_end_line_lost_debug_status()


def main(args=None):
    rclpy.init(args=args)
    node = MotionPlanningNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n\nshutdown\n\n")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
