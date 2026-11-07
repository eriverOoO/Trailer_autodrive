import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSReliabilityPolicy

from cv_bridge import CvBridge

from sensor_msgs.msg import Image
from interfaces_pkg.msg import TargetPoint, LaneInfo, DetectionArray
from .lib import camera_perception_func_lib as CPFL

#---------------Variable Setting---------------
# Subscribe할 토픽 이름
SUB_TOPIC_NAME = "detections"

# Publish할 parking_space LaneInfo 토픽 이름
PARKING_SPACE_INFO_TOPIC_NAME = "yolov8_parking_space_info"

# Publish할 ROI 이미지 토픽 이름
ROI_IMAGE_PARKING_SPACE_TOPIC_NAME = "roi_image_parking_space"

# 화면에 이미지를 처리하는 과정을 띄울것인지 여부: True, 또는 False 중 택1하여 입력
SHOW_IMAGE = True
#----------------------------------------------


class Yolov8InfoExtractor(Node):
    def __init__(self):
        super().__init__('parking_space_info_extractor_node')

        self.sub_topic = self.declare_parameter(
            'sub_detection_topic',
            SUB_TOPIC_NAME
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

    def extract_lane_info(self, detection_msg: DetectionArray, cls_name: str):
        lane_edge_image = CPFL.draw_edges(
            detection_msg,
            cls_name=cls_name,
            color=255
        )

        (h, w) = (lane_edge_image.shape[0], lane_edge_image.shape[1])  # (480, 640)

        dst_mat = [
            [round(w * 0.3), round(h * 0.0)],
            [round(w * 0.7), round(h * 0.0)],
            [round(w * 0.7), h],
            [round(w * 0.3), h]
        ]

        src_mat = [[238, 316], [402, 313], [501, 476], [155, 476]]

        lane_bird_image = CPFL.bird_convert(
            lane_edge_image,
            srcmat=src_mat,
            dstmat=dst_mat
        )

        roi_image = CPFL.roi_rectangle_below(
            lane_bird_image,
            cutting_idx=300
        )

        # roi_image를 uint8 형식으로 변환
        roi_image = cv2.convertScaleAbs(roi_image)  # 64FC1 -> uint8로 변환

        grad = CPFL.dominant_gradient(roi_image, theta_limit=70)

        target_points = []

        # target point 10개 생성
        # y = 5, 20, 35, 50, 65, 80, 95, 110, 125, 140
        for target_point_y in range(5, 155, 15):
            target_point_x = CPFL.get_lane_center(
                roi_image,
                detection_height=target_point_y,
                detection_thickness=10,
                road_gradient=grad,
                lane_width=300
            )

            target_point = TargetPoint()
            target_point.target_x = round(target_point_x)
            target_point.target_y = round(target_point_y)
            target_points.append(target_point)

        lane = LaneInfo()
        lane.slope = grad
        lane.target_points = target_points

        return lane, roi_image, target_points

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
        if len(detection_msg.detections) == 0:
            return

        # parking_space 추출
        parking_space_info, roi_image_parking_space, parking_space_target_points = self.extract_lane_info(
            detection_msg,
            cls_name='parking_space'
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
    node = Yolov8InfoExtractor()

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
