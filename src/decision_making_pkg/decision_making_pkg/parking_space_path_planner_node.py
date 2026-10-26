import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSDurabilityPolicy, QoSReliabilityPolicy
from interfaces_pkg.msg import LaneInfo, PathPlanningResult
import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import CubicSpline

#---------------Variable Setting---------------
SUB_PARKING_SPACE_TOPIC_NAME = "yolov8_parking_space_info"

PUB_PARKING_SPACE_TOPIC_NAME = "path_planning_result_parking_space"

CAR_CENTER_POINT = (340, 179) # 이미지 상에서 차량 앞 범퍼의 중심이 위치한 픽셀 좌표
#----------------------------------------------


class ParkingSpacePathPlannerNode(Node):
    def __init__(self):
        super().__init__('parking_space_path_planner_node')

        # 파라미터 선언
        self.sub_parking_space_topic = self.declare_parameter(
            'sub_parking_space_topic',
            SUB_PARKING_SPACE_TOPIC_NAME
        ).value

        self.pub_parking_space_topic = self.declare_parameter(
            'pub_parking_space_topic',
            PUB_PARKING_SPACE_TOPIC_NAME
        ).value

        self.car_center_point = self.declare_parameter(
            'car_center_point',
            CAR_CENTER_POINT
        ).value

        # QoS 설정
        self.qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1
        )

        # parking_space LaneInfo 서브스크라이버
        self.parking_space_sub = self.create_subscription(
            LaneInfo,
            self.sub_parking_space_topic,
            self.parking_space_callback,
            self.qos_profile
        )

        # parking_space PathPlanningResult 퍼블리셔
        self.parking_space_publisher = self.create_publisher(
            PathPlanningResult,
            self.pub_parking_space_topic,
            self.qos_profile
        )

    def parking_space_callback(self, msg: LaneInfo):
        # parking_space 타겟 포인트 받아서 경로 계획
        target_points = msg.target_points

        if len(target_points) >= 5:
            self.plan_path(
                target_points=target_points,
                publisher=self.parking_space_publisher,
                lane_name="parking_space"
            )

    def plan_path(self, target_points, publisher, lane_name):
        # target_points가 TargetPoint 객체들의 리스트라고 가정
        if not target_points:
            self.get_logger().warn(f"No target points available for {lane_name}")
            return

        # TargetPoint 객체에서 x, y 값 추출
        x_points, y_points = zip(*[(tp.target_x, tp.target_y) for tp in target_points])

        # 차량 앞 범퍼의 중심이 위치한 픽셀 좌표 추가
        y_points_list = list(y_points)
        x_points_list = list(x_points)

        y_points_list.append(self.car_center_point[1])
        x_points_list.append(self.car_center_point[0])

        y_points = tuple(y_points_list)
        x_points = tuple(x_points_list)

        # y 값을 기준으로 정렬
        sorted_points = sorted(zip(y_points, x_points), key=lambda point: point[0])

        # 정렬된 y, x 값을 다시 분리
        y_points, x_points = zip(*sorted_points)

        # 몇 개의 점으로 경로 계획을 하는지 확인
        self.get_logger().info(
            f"Planning {lane_name} path with {len(y_points)} points"
        )

        # 스플라인 보간법을 사용하여 경로 생성
        try:
            cs = CubicSpline(y_points, x_points, bc_type='natural')
        except Exception as e:
            self.get_logger().error(
                f"Failed to create CubicSpline for {lane_name}: {e}"
            )
            return

        # 생성된 경로 점들
        y_new = np.linspace(min(y_points), max(y_points), 100)
        x_new = cs(y_new)

        # PathPlanningResult 메시지로 발행
        path_msg = PathPlanningResult()
        path_msg.x_points = list(x_new)
        path_msg.y_points = list(y_new)

        # 경로 퍼블리시
        publisher.publish(path_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ParkingSpacePathPlannerNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        print("\n\nshutdown\n\n")

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
