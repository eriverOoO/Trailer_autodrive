import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSReliabilityPolicy

from interfaces_pkg.msg import DetectionArray, ParkingSpaceDetectionInfo

# ---------------Variable Setting---------------
# Subscribe할 토픽 이름
SUB_DETECTION_TOPIC_NAME = "detections"

# Publish할 토픽 이름
PUB_TOPIC_NAME = "yolov8_parking_space_detection_info"

# YOLO에서 학습한 parking_space 클래스 이름
PARKING_SPACE_CLASS_NAME = "parking_space"
NORMALIZE_BBOX_TO_BASE_RESOLUTION = False
SOURCE_WIDTH = 640.0
SOURCE_HEIGHT = 480.0
BASE_WIDTH = 640.0
BASE_HEIGHT = 480.0
# ----------------------------------------------


class ParkingSpaceDetector(Node):
    def __init__(self):
        super().__init__('parking_space_detector_node')

        self.sub_detection_topic = self.declare_parameter(
            'sub_detection_topic',
            SUB_DETECTION_TOPIC_NAME
        ).value

        self.pub_topic = self.declare_parameter(
            'pub_topic',
            PUB_TOPIC_NAME
        ).value

        self.parking_space_class_name = self.declare_parameter(
            'parking_space_class_name',
            PARKING_SPACE_CLASS_NAME
        ).value

        self.normalize_bbox_to_base_resolution = self.declare_parameter(
            'normalize_bbox_to_base_resolution',
            NORMALIZE_BBOX_TO_BASE_RESOLUTION
        ).value

        self.source_width = float(self.declare_parameter(
            'source_width',
            SOURCE_WIDTH
        ).value)

        self.source_height = float(self.declare_parameter(
            'source_height',
            SOURCE_HEIGHT
        ).value)

        self.qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1
        )

        # /detections 구독
        self.detection_sub = self.create_subscription(
            DetectionArray,
            self.sub_detection_topic,
            self.detection_callback,
            self.qos_profile
        )

        # /yolov8_parking_space_detection_info 퍼블리시
        self.publisher = self.create_publisher(
            ParkingSpaceDetectionInfo,
            self.pub_topic,
            self.qos_profile
        )

    def detection_callback(self, detection_msg: DetectionArray):
        parking_space_info_msg = ParkingSpaceDetectionInfo()
        best_parking_space_detection = None

        for detection in detection_msg.detections:
            if detection.class_name == self.parking_space_class_name:
                if (
                    best_parking_space_detection is None
                    or detection.score > best_parking_space_detection.score
                ):
                    best_parking_space_detection = detection

        if best_parking_space_detection is not None:
            bbox = best_parking_space_detection.bbox
            bbox_width = float(bbox.size.x)
            bbox_height = float(bbox.size.y)
            bbox_center_x = float(bbox.center.position.x)
            bbox_center_y = float(bbox.center.position.y)

            if self.normalize_bbox_to_base_resolution:
                source_width = self.source_width
                source_height = self.source_height
                if best_parking_space_detection.mask.width > 0:
                    source_width = float(best_parking_space_detection.mask.width)
                if best_parking_space_detection.mask.height > 0:
                    source_height = float(best_parking_space_detection.mask.height)

                x_ratio = source_width / BASE_WIDTH if source_width > 0.0 else 1.0
                y_ratio = source_height / BASE_HEIGHT if source_height > 0.0 else 1.0

                bbox_width = bbox_width / x_ratio
                bbox_height = bbox_height / y_ratio
                bbox_center_x = bbox_center_x / x_ratio
                bbox_center_y = bbox_center_y / y_ratio

            parking_space_info_msg.parking_space_detected = True
            parking_space_info_msg.bbox_width = bbox_width
            parking_space_info_msg.bbox_height = bbox_height
            parking_space_info_msg.bbox_area = bbox_width * bbox_height
            parking_space_info_msg.bbox_center_x = bbox_center_x
            parking_space_info_msg.bbox_center_y = bbox_center_y
            parking_space_info_msg.score = float(best_parking_space_detection.score)

        print(
            f'parking_space detected: '
            f'{parking_space_info_msg.parking_space_detected}, '
            f'bbox_width: {parking_space_info_msg.bbox_width:.2f}, '
            f'bbox_height: {parking_space_info_msg.bbox_height:.2f}, '
            f'bbox_area: {parking_space_info_msg.bbox_area:.2f}, '
            f'center_x: {parking_space_info_msg.bbox_center_x:.2f}, '
            f'center_y: {parking_space_info_msg.bbox_center_y:.2f}, '
            f'score: {parking_space_info_msg.score:.2f}'
        )
        self.publisher.publish(parking_space_info_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ParkingSpaceDetector()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        print("\n\nshutdown\n\n")

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
