import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSReliabilityPolicy

from interfaces_pkg.msg import CarDetectionInfo, DetectionArray

# ---------------Variable Setting---------------
# Subscribe할 토픽 이름
SUB_DETECTION_TOPIC_NAME = "detections"

# Publish할 토픽 이름
PUB_TOPIC_NAME = "yolov8_car_info"

# YOLO에서 학습한 car 클래스 이름
CAR_CLASS_NAME = "car"
# ----------------------------------------------


class CarDetector(Node):
    def __init__(self):
        super().__init__('car_detector_node')

        self.sub_detection_topic = self.declare_parameter(
            'sub_detection_topic',
            SUB_DETECTION_TOPIC_NAME
        ).value

        self.pub_topic = self.declare_parameter(
            'pub_topic',
            PUB_TOPIC_NAME
        ).value

        self.car_class_name = self.declare_parameter(
            'car_class_name',
            CAR_CLASS_NAME
        ).value

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

        # /yolov8_car_info 퍼블리시
        self.publisher = self.create_publisher(
            CarDetectionInfo,
            self.pub_topic,
            self.qos_profile
        )

    def detection_callback(self, detection_msg: DetectionArray):
        car_info_msg = CarDetectionInfo()
        best_car_detection = None

        for detection in detection_msg.detections:
            if detection.class_name == self.car_class_name:
                if (
                    best_car_detection is None
                    or detection.score > best_car_detection.score
                ):
                    best_car_detection = detection

        if best_car_detection is not None:
            bbox = best_car_detection.bbox
            bbox_width = float(bbox.size.x)
            bbox_height = float(bbox.size.y)

            car_info_msg.car_detected = True
            car_info_msg.bbox_width = bbox_width
            car_info_msg.bbox_height = bbox_height
            car_info_msg.bbox_area = bbox_width * bbox_height
            car_info_msg.bbox_center_x = float(bbox.center.position.x)
            car_info_msg.bbox_center_y = float(bbox.center.position.y)
            car_info_msg.score = float(best_car_detection.score)

        print(
            f'car detected: {car_info_msg.car_detected}, '
            f'bbox_width: {car_info_msg.bbox_width:.2f}, '
            f'bbox_height: {car_info_msg.bbox_height:.2f}, '
            f'bbox_area: {car_info_msg.bbox_area:.2f}, '
            f'center_x: {car_info_msg.bbox_center_x:.2f}, '
            f'center_y: {car_info_msg.bbox_center_y:.2f}, '
            f'score: {car_info_msg.score:.2f}'
        )
        self.publisher.publish(car_info_msg)


def main(args=None):
    rclpy.init(args=args)
    node = CarDetector()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        print("\n\nshutdown\n\n")

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
