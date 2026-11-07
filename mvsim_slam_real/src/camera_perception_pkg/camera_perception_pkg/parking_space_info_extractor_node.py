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
from interfaces_pkg.msg import TargetPoint, LaneInfo, DetectionArray

#---------------Variable Setting---------------
# Subscribe할 토픽 이름
SUB_TOPIC_NAME = "detections"

# YOLO에서 학습한 주차칸 클래스 이름
PARKING_SPACE_CLASS_NAME = "parking_space"

# Publish할 ParkingSpaceInfo 토픽 이름
PARKING_SPACE_INFO_TOPIC_NAME = "yolov8_parking_space_info"

# Publish할 ROI 이미지 토픽 이름
ROI_IMAGE_PARKING_SPACE_TOPIC_NAME = "roi_image_parking_space"

# 화면에 이미지를 처리하는 과정을 띄울것인지 여부: True, 또는 False 중 택1하여 입력
SHOW_IMAGE = True

PROCESS_IMAGE_WIDTH = 640
PROCESS_IMAGE_HEIGHT = 480

BIRD_SRC_MAT = [
    [51, 316],
    [557, 316],
    [639, 476],
    [0, 476],
]

PARKING_SPACE_CENTER_X = 320
PARKING_SPACE_WIDTH = 330

BIRD_DST_LEFT_X = PARKING_SPACE_CENTER_X - PARKING_SPACE_WIDTH // 2
BIRD_DST_RIGHT_X = PARKING_SPACE_CENTER_X + PARKING_SPACE_WIDTH // 2

BIRD_DST_MAT = [
    [BIRD_DST_LEFT_X, 0],
    [BIRD_DST_RIGHT_X, 0],
    [BIRD_DST_RIGHT_X, PROCESS_IMAGE_HEIGHT],
    [BIRD_DST_LEFT_X, PROCESS_IMAGE_HEIGHT],
]

ROI_CUTTING_INDEX = 300

TARGET_POINT_Y_VALUES = list(range(5, 155, 15))
TARGET_DETECTION_THICKNESS = 10

MIN_MASK_PIXEL_COUNT = 300
MIN_VALID_TARGET_COUNT = 3
MIN_ROW_MASK_WIDTH = 20
MIN_VALID_BIRD_WIDTH = 100
MAX_VALID_BIRD_WIDTH = 620
BORDER_MARGIN = 2
SIDE_EDGE_THICKNESS = 2
#----------------------------------------------


class ParkingSpaceInfoExtractor(Node):
    def __init__(self):
        super().__init__('parking_space_info_extractor_node')

        self.sub_topic = self.declare_parameter(
            'sub_detection_topic',
            SUB_TOPIC_NAME
        ).value

        self.parking_space_class_name = self.declare_parameter(
            'parking_space_class_name',
            PARKING_SPACE_CLASS_NAME
        ).value

        self.parking_space_info_topic = self.declare_parameter(
            'parking_space_info_topic',
            PARKING_SPACE_INFO_TOPIC_NAME
        ).value

        self.roi_image_parking_space_topic = self.declare_parameter(
            'roi_image_parking_space_topic',
            ROI_IMAGE_PARKING_SPACE_TOPIC_NAME
        ).value

        self.show_image = self.declare_parameter(
            'show_image',
            SHOW_IMAGE
        ).value

        self.cv_bridge = CvBridge()
        self.transform_matrix = cv2.getPerspectiveTransform(
            np.float32(BIRD_SRC_MAT),
            np.float32(BIRD_DST_MAT)
        )

        # QoS settings
        self.qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1
        )

        self.subscriber = self.create_subscription(
            DetectionArray,
            self.sub_topic,
            self.yolov8_detections_callback,
            self.qos_profile
        )

        # parking_space LaneInfo 퍼블리셔
        self.parking_space_publisher = self.create_publisher(
            LaneInfo,
            self.parking_space_info_topic,
            self.qos_profile
        )

        # parking_space ROI 이미지 퍼블리셔
        self.roi_image_parking_space_publisher = self.create_publisher(
            Image,
            self.roi_image_parking_space_topic,
            self.qos_profile
        )

    def make_empty_result(self):
        lane = LaneInfo()
        lane.slope = 0.0
        lane.target_points = []
        roi_image = np.zeros(
            (PROCESS_IMAGE_HEIGHT - ROI_CUTTING_INDEX, PROCESS_IMAGE_WIDTH),
            dtype=np.uint8
        )
        return lane, roi_image, []

    def select_parking_space_mask(self, detection_msg: DetectionArray):
        best_contour = None
        best_area = 0.0
        best_width = 0
        best_height = 0

        for detection in detection_msg.detections:
            if detection.class_name != self.parking_space_class_name:
                continue

            mask_msg = detection.mask
            if not mask_msg.data:
                continue

            if mask_msg.width <= 0 or mask_msg.height <= 0:
                continue

            contour = np.array(
                [[point.x, point.y] for point in mask_msg.data],
                dtype=np.float32
            )
            if contour.shape[0] < 3:
                continue

            contour[:, 0] = np.clip(contour[:, 0], 0, mask_msg.width - 1)
            contour[:, 1] = np.clip(contour[:, 1], 0, mask_msg.height - 1)
            contour = np.round(contour).astype(np.int32)

            area = cv2.contourArea(contour)
            if area <= 0:
                continue

            if area > best_area:
                best_area = area
                best_contour = contour
                best_width = mask_msg.width
                best_height = mask_msg.height

        if best_contour is None:
            return None

        filled_mask = np.zeros((best_height, best_width), dtype=np.uint8)
        cv2.fillPoly(filled_mask, [best_contour], 255)

        if (best_width, best_height) != (PROCESS_IMAGE_WIDTH, PROCESS_IMAGE_HEIGHT):
            filled_mask = cv2.resize(
                filled_mask,
                (PROCESS_IMAGE_WIDTH, PROCESS_IMAGE_HEIGHT),
                interpolation=cv2.INTER_NEAREST
            )

        _, filled_mask = cv2.threshold(filled_mask, 127, 255, cv2.THRESH_BINARY)
        return filled_mask.astype(np.uint8)

    def make_bird_mask(self, filled_mask):
        bird_mask = cv2.warpPerspective(
            filled_mask,
            self.transform_matrix,
            (PROCESS_IMAGE_WIDTH, PROCESS_IMAGE_HEIGHT),
            flags=cv2.INTER_NEAREST
        )
        _, bird_mask = cv2.threshold(bird_mask, 127, 255, cv2.THRESH_BINARY)
        return bird_mask.astype(np.uint8)

    def calculate_row_center(self, row):
        detected_x = np.flatnonzero(row > 0)
        if detected_x.size < MIN_ROW_MASK_WIDTH:
            return None

        left_x = int(detected_x[0])
        right_x = int(detected_x[-1])
        mask_width = right_x - left_x + 1

        left_on_border = left_x <= BORDER_MARGIN
        right_on_border = right_x >= PROCESS_IMAGE_WIDTH - 1 - BORDER_MARGIN

        if left_on_border and right_on_border:
            return None

        if not left_on_border and not right_on_border:
            if mask_width < MIN_VALID_BIRD_WIDTH or mask_width > MAX_VALID_BIRD_WIDTH:
                return None
            center_x = (left_x + right_x) / 2.0
        elif not left_on_border and right_on_border:
            center_x = left_x + PARKING_SPACE_WIDTH / 2.0
        elif left_on_border and not right_on_border:
            center_x = right_x - PARKING_SPACE_WIDTH / 2.0
        else:
            return None

        if center_x < 0 or center_x > PROCESS_IMAGE_WIDTH - 1:
            return None

        return center_x

    def calculate_target_center(self, roi_mask, target_y):
        half_thickness = TARGET_DETECTION_THICKNESS // 2
        start_y = max(0, target_y - half_thickness)
        end_y = min(roi_mask.shape[0], target_y + half_thickness)

        row_centers = []
        for row_y in range(start_y, end_y):
            row_center = self.calculate_row_center(roi_mask[row_y, :])
            if row_center is not None:
                row_centers.append(row_center)

        if len(row_centers) < MIN_VALID_TARGET_COUNT:
            return None

        return float(np.median(row_centers))

    def make_side_edge_image(self, roi_mask):
        side_edge_image = np.zeros_like(roi_mask, dtype=np.uint8)

        for row_y in range(roi_mask.shape[0]):
            detected_x = np.flatnonzero(roi_mask[row_y, :] > 0)
            if detected_x.size < MIN_ROW_MASK_WIDTH:
                continue

            left_x = int(detected_x[0])
            right_x = int(detected_x[-1])

            if left_x > BORDER_MARGIN:
                cv2.line(
                    side_edge_image,
                    (left_x, row_y),
                    (left_x, row_y),
                    255,
                    thickness=SIDE_EDGE_THICKNESS
                )

            if right_x < PROCESS_IMAGE_WIDTH - 1 - BORDER_MARGIN:
                cv2.line(
                    side_edge_image,
                    (right_x, row_y),
                    (right_x, row_y),
                    255,
                    thickness=SIDE_EDGE_THICKNESS
                )

        return side_edge_image

    def interpolate_target_centers(self, target_centers):
        valid_indices = [
            idx for idx, center in enumerate(target_centers)
            if center is not None
        ]

        if len(valid_indices) < MIN_VALID_TARGET_COUNT:
            return None

        x_values = np.array(
            [target_centers[idx] for idx in valid_indices],
            dtype=np.float32
        )
        y_values = np.array(
            [TARGET_POINT_Y_VALUES[idx] for idx in valid_indices],
            dtype=np.float32
        )
        target_y_values = np.array(TARGET_POINT_Y_VALUES, dtype=np.float32)

        interpolated = np.interp(target_y_values, y_values, x_values)
        interpolated = np.clip(interpolated, 0, PROCESS_IMAGE_WIDTH - 1)
        return interpolated

    def extract_lane_info(self, detection_msg: DetectionArray):
        filled_mask = self.select_parking_space_mask(detection_msg)
        if filled_mask is None:
            return self.make_empty_result()

        if cv2.countNonZero(filled_mask) < MIN_MASK_PIXEL_COUNT:
            return self.make_empty_result()

        bird_mask = self.make_bird_mask(filled_mask)
        roi_mask = bird_mask[ROI_CUTTING_INDEX:, :]
        roi_side_edge_image = self.make_side_edge_image(roi_mask)

        target_centers = [
            self.calculate_target_center(roi_mask, target_y)
            for target_y in TARGET_POINT_Y_VALUES
        ]
        target_x_values = self.interpolate_target_centers(target_centers)
        if target_x_values is None:
            lane, _, target_points = self.make_empty_result()
            return lane, roi_side_edge_image, target_points

        target_points = []
        for target_x, target_y in zip(target_x_values, TARGET_POINT_Y_VALUES):
            target_point = TargetPoint()
            target_point.target_x = int(round(float(target_x)))
            target_point.target_y = int(target_y)
            target_points.append(target_point)

        x_values = np.array(
            [target_point.target_x for target_point in target_points],
            dtype=np.float32
        )
        y_values = np.array(TARGET_POINT_Y_VALUES, dtype=np.float32)
        coefficient = np.polyfit(y_values, x_values, 1)[0]
        gradient_degree = np.degrees(np.arctan(coefficient))

        lane = LaneInfo()
        lane.slope = float(gradient_degree)
        lane.target_points = target_points

        return lane, roi_side_edge_image, target_points

    def publish_roi_image(self, roi_image, publisher, lane_name: str):
        try:
            roi_image_msg = self.cv_bridge.cv2_to_imgmsg(
                roi_image,
                encoding="mono8"
            )
            publisher.publish(roi_image_msg)

        except Exception as e:
            self.get_logger().error(
                f"Failed to convert and publish {lane_name} ROI image: {e}"
            )

    def show_roi_image(self, roi_image, target_points, window_name: str):
        # 빨간색 점을 찍기 위해 흑백 ROI 이미지를 BGR 컬러 이미지로 변환
        roi_image_color = cv2.cvtColor(roi_image, cv2.COLOR_GRAY2BGR)

        # 타겟 포인트를 빨간색 점으로 표시
        for target_point in target_points:
            x = target_point.target_x
            y = target_point.target_y
            cv2.circle(roi_image_color, (x, y), 5, (0, 0, 255), -1)

        # ROI 이미지 자체를 화면 표시용으로 변형
        # 가로는 덜 키우고, 세로는 더 크게 키움
        # 계산용 roi_image는 건드리지 않음
        roi_image_display = cv2.resize(
            roi_image_color,
            None,
            fx=0.35,
            fy=0.875,
            interpolation=cv2.INTER_NEAREST
        )

        cv2.imshow(window_name, roi_image_display)

    def yolov8_detections_callback(self, detection_msg: DetectionArray):
        # parking_space 추출
        parking_space_info, roi_image_parking_space, parking_space_target_points = self.extract_lane_info(
            detection_msg
        )

        # parking_space LaneInfo 퍼블리시
        self.parking_space_publisher.publish(parking_space_info)

        # parking_space ROI 이미지 퍼블리시
        self.publish_roi_image(
            roi_image_parking_space,
            self.roi_image_parking_space_publisher,
            lane_name='parking_space'
        )

        if self.show_image:
            self.show_roi_image(
                roi_image_parking_space,
                parking_space_target_points,
                window_name='roi_parking_space_img'
            )

            cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = ParkingSpaceInfoExtractor()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        print("\n\nshutdown\n\n")

    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
