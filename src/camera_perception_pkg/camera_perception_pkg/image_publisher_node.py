import rclpy 
from rclpy.node import Node 
from sensor_msgs.msg import Image 
from std_msgs.msg import Header
from cv_bridge import CvBridge, CvBridgeError

from rclpy.qos import QoSProfile
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSReliabilityPolicy

import sys
import cv2
import os
import shutil
import subprocess
import threading
import time

#---------------Variable Setting---------------
# Publish할 토픽 이름
PUB_TOPIC_NAME = 'image_raw'

# 데이터 입력 소스: 'camera', 'image', 또는 'video' 중 택1하여 입력
DATA_SOURCE = 'camera'

# 카메라(웹캠) 장치 번호 (ls /dev/video* 명령을 터미널 창에 입력하여 확인)
CAM_NUM = 2

# 고정 카메라 경로 (/dev/v4l/by-id/... 사용 시 포트 번호 변경 영향을 받지 않음)
CAMERA_PATH = ''

# 이미지 데이터가 들어있는 디렉토리의 경로를 입력
IMAGE_DIRECTORY_PATH = 'src/camera_perception_pkg/camera_perception_pkg/lib/Collected_Datasets/sample_dataset'

# 비디오 데이터 파일의 경로를 입력
VIDEO_FILE_PATH = 'src/camera_perception_pkg/camera_perception_pkg/lib/Collected_Datasets/driving_simulation.mp4'

# 화면에 publish하는 이미지를 띄울것인지 여부: True, 또는 False 중 택1하여 입력
SHOW_IMAGE = True

# 이미지 발행 주기 (초) - 소수점 필요 (int형은 반영되지 않음)
TIMER = 0.03

FRAME_ID = 'image_frame'

BASE_WIDTH = 640
BASE_HEIGHT = 480
FRONT_CAMERA_WIDTH = 1280
FRONT_CAMERA_HEIGHT = 720
REAR_CAMERA_WIDTH = 1280
REAR_CAMERA_HEIGHT = 720
CAMERA_FPS = 30
METRICS_LOG_INTERVAL = 5.0
EXPECTED_CAMERA_CONFIG = {
    'front': (2, '/dev/video2', 'front_camera_frame'),
    'rear': (4, '/dev/video4', 'rear_camera_frame'),
}
#----------------------------------------------

class ImagePublisherNode(Node):
    def __init__(self, data_source=DATA_SOURCE, cam_num=CAM_NUM, camera_path=CAMERA_PATH, img_dir=IMAGE_DIRECTORY_PATH, video_path=VIDEO_FILE_PATH, pub_topic=PUB_TOPIC_NAME, logger=SHOW_IMAGE, timer=TIMER, frame_id=FRAME_ID):
        super().__init__('image_publisher_node')
        self.declare_parameter('data_source', data_source)
        self.declare_parameter('cam_num', cam_num)
        self.declare_parameter('camera_path', camera_path)
        self.declare_parameter('img_dir', img_dir)
        self.declare_parameter('video_path', video_path)
        self.declare_parameter('pub_topic', pub_topic)
        self.declare_parameter('logger', logger)
        self.declare_parameter('timer', timer)
        self.declare_parameter('frame_id', frame_id)
        
        self.data_source = self.get_parameter('data_source').get_parameter_value().string_value
        self.cam_num = self.get_parameter('cam_num').value
        self.camera_path = self.get_parameter('camera_path').value
        self.img_dir = self.get_parameter('img_dir').get_parameter_value().string_value
        self.video_path = self.get_parameter('video_path').get_parameter_value().string_value
        self.pub_topic = self.get_parameter('pub_topic').get_parameter_value().string_value
        self.logger = self.get_parameter('logger').get_parameter_value().bool_value
        self.timer_period = self.get_parameter('timer').get_parameter_value().double_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.camera_role = self._camera_role_label()
        self.camera_num_label = self._camera_num_label(self.cam_num)
        self.request_width, self.request_height = self._requested_camera_size()
        self.latest_frame = None
        self.latest_frame_id = 0
        self.last_published_frame_id = 0
        self.latest_frame_lock = threading.Lock()
        self.cap_lock = threading.Lock()
        self.capture_stop_event = threading.Event()
        self.capture_thread = None
        self.failed_read_count = 0
        self.capture_interval_sum = 0.0
        self.capture_interval_count = 0
        self.process_time_sum = 0.0
        self.process_time_count = 0
        self.capture_last_time = None
        self.metrics_last_log_time = time.monotonic()

        self.qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1
        )
        
        self.br = CvBridge()
        
        if self.data_source == 'camera':
            if not self._validate_camera_mapping():
                rclpy.shutdown()
                sys.exit(1)
            self.camera_sources = self._resolve_camera_sources(self.camera_path, self.cam_num)
            self.get_logger().info('%s Camera device: /dev/video%s' % (self.camera_role, self.camera_num_label))
            self.cap = self._open_camera(self.camera_sources)
            if self.cap is None:
                self.get_logger().error(
                    'Failed to open %s Camera: Camera Number %s from candidates: %s' %
                    (self.camera_role, self.camera_num_label, self.camera_sources)
                )
            else:
                self._start_capture_thread()
        elif self.data_source == 'video':
            self.cap = cv2.VideoCapture(self.video_path)
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            if not self.cap.isOpened():
                self.get_logger().error('Cannot open video file: %s' % self.video_path)
                rclpy.shutdown()
                sys.exit(1)
        elif self.data_source == 'image':
            if os.path.isdir(self.img_dir):
                self.img_list = sorted(os.listdir(self.img_dir))
                self.img_num = 0
            else:
                self.get_logger().error('Not a directory file: %s' % self.img_dir)
                rclpy.shutdown()
                sys.exit(1)
        else:
            self.get_logger().error("Wrong data source: %s \nCheck that the DATA_SOURCE variable is either 'camera', 'image', or 'video'." % self.data_source)
            rclpy.shutdown()
            sys.exit(1)
        self.publisher = self.create_publisher(Image, self.pub_topic, self.qos_profile)
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

    def _camera_role_label(self):
        namespace = self.get_namespace().strip('/')
        frame_id = self.frame_id.lower()
        if namespace == 'front' or 'front' in frame_id:
            return 'Front'
        if namespace == 'rear' or 'rear' in frame_id:
            return 'Rear'
        return 'Unknown'

    def _normalize_camera_num(self, cam_num):
        if isinstance(cam_num, str):
            cam_num = cam_num.strip()
            if cam_num.startswith('/dev/video'):
                cam_num = cam_num.replace('/dev/video', '', 1)
            if cam_num.isdigit():
                return int(cam_num)
            return None
        if isinstance(cam_num, int):
            return cam_num
        return None

    def _validate_camera_mapping(self):
        namespace = self.get_namespace().strip('/')
        if namespace not in EXPECTED_CAMERA_CONFIG:
            return True

        expected_num, expected_path, expected_frame_id = EXPECTED_CAMERA_CONFIG[namespace]
        actual_num = self._normalize_camera_num(self.cam_num)
        actual_path = str(self.camera_path).strip() if self.camera_path is not None else ''

        if actual_num != expected_num:
            self.get_logger().error(
                '%s namespace must use camera %d (%s), but cam_num is %s.' %
                (namespace, expected_num, expected_path, self.cam_num)
            )
            return False

        if actual_path and actual_path != expected_path:
            self.get_logger().error(
                '%s namespace must use %s, but camera_path is %s.' %
                (namespace, expected_path, actual_path)
            )
            return False

        if self.frame_id != expected_frame_id:
            self.get_logger().error(
                '%s namespace must use frame_id %s, but frame_id is %s.' %
                (namespace, expected_frame_id, self.frame_id)
            )
            return False

        return True

    def _camera_num_label(self, cam_num):
        if isinstance(cam_num, str):
            cam_num = cam_num.strip()
            if cam_num.startswith('/dev/video'):
                return cam_num.replace('/dev/video', '', 1)
        return str(cam_num)

    def _requested_camera_size(self):
        if self.camera_role == 'Front':
            return FRONT_CAMERA_WIDTH, FRONT_CAMERA_HEIGHT
        if self.camera_role == 'Rear':
            return REAR_CAMERA_WIDTH, REAR_CAMERA_HEIGHT
        return BASE_WIDTH, BASE_HEIGHT

    def _resolve_camera_sources(self, camera_path, cam_num):
        camera_path = str(camera_path).strip() if camera_path is not None else ''
        if camera_path:
            return [int(camera_path) if camera_path.isdigit() else camera_path]

        if isinstance(cam_num, str):
            cam_num = cam_num.strip()
            if not cam_num:
                return []
            cam_num = int(cam_num) if cam_num.isdigit() else cam_num

        if isinstance(cam_num, int):
            return [f'/dev/video{cam_num}', cam_num]

        return [cam_num]

    def _source_to_device_path(self, camera_source):
        if isinstance(camera_source, str):
            if camera_source.startswith('/dev/video'):
                return camera_source
            return ''
        if isinstance(camera_source, int):
            return f'/dev/video{camera_source}'
        return ''

    def _fourcc_to_string(self, fourcc_value):
        fourcc_int = int(fourcc_value)
        chars = [
            chr((fourcc_int >> 8 * i) & 0xFF)
            for i in range(4)
        ]
        return ''.join(chars).strip()

    def _is_mjpg_supported(self, camera_source):
        device_path = self._source_to_device_path(camera_source)
        if not device_path or shutil.which('v4l2-ctl') is None:
            return False

        try:
            result = subprocess.run(
                ['v4l2-ctl', '--device', device_path, '--list-formats-ext'],
                capture_output=True,
                text=True,
                timeout=2.0,
                check=False
            )
        except Exception as e:
            self.get_logger().warn(
                '%s Camera format check failed for %s: %s' %
                (self.camera_role, device_path, e)
            )
            return False

        if result.returncode != 0:
            self.get_logger().warn(
                '%s Camera format check failed for %s: %s' %
                (self.camera_role, device_path, result.stderr.strip())
            )
            return False

        return 'MJPG' in result.stdout or 'MJPEG' in result.stdout

    def _try_apply_mjpg(self, cap, camera_source):
        current_format = self._fourcc_to_string(cap.get(cv2.CAP_PROP_FOURCC))
        self.get_logger().info(
            '%s Camera current format before MJPG check: %s' %
            (self.camera_role, current_format or 'unknown')
        )

        if not self._is_mjpg_supported(camera_source):
            self.get_logger().info(
                '%s Camera MJPG support not confirmed; keeping existing format: %s' %
                (self.camera_role, current_format or 'unknown')
            )
            return

        mjpg = cv2.VideoWriter_fourcc(*'MJPG')
        if not cap.set(cv2.CAP_PROP_FOURCC, mjpg):
            self.get_logger().warn(
                '%s Camera MJPG format request failed; keeping existing format.' %
                self.camera_role
            )
            return

        applied_format = self._fourcc_to_string(cap.get(cv2.CAP_PROP_FOURCC))
        if applied_format == 'MJPG':
            self.get_logger().info('%s Camera MJPG format applied.' % self.camera_role)
        else:
            self.get_logger().warn(
                '%s Camera MJPG requested but actual format is %s.' %
                (self.camera_role, applied_format or 'unknown')
            )

    def _log_actual_camera_settings(self, cap):
        actual_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        actual_fps = cap.get(cv2.CAP_PROP_FPS)
        actual_format = self._fourcc_to_string(cap.get(cv2.CAP_PROP_FOURCC))

        self.get_logger().info(
            'cam%s (%s) requested resolution: %dx%d' %
            (
                self.camera_num_label,
                self.camera_role.lower(),
                self.request_width,
                self.request_height
            )
        )
        self.get_logger().info(
            'cam%s (%s) actual resolution: %dx%d, fps: %.2f, format: %s' %
            (
                self.camera_num_label,
                self.camera_role,
                actual_width,
                actual_height,
                actual_fps,
                actual_format or 'unknown'
            )
        )

        if (
            actual_width != self.request_width
            or actual_height != self.request_height
        ):
            self.get_logger().warn(
                'cam%s (%s) requested resolution was not fully applied. Using actual %dx%d.' %
                (self.camera_num_label, self.camera_role.lower(), actual_width, actual_height)
            )

    def _open_camera(self, camera_sources):
        for camera_source in camera_sources:
            for api_preference in (cv2.CAP_V4L2, cv2.CAP_ANY):
                cap = cv2.VideoCapture(camera_source, api_preference)
                cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

                if not cap.isOpened():
                    self.get_logger().warn(
                        '%s Camera open failed: %s' %
                        (self.camera_role, camera_source)
                    )
                    cap.release()
                    continue

                self._try_apply_mjpg(cap, camera_source)
                width_set = cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.request_width)
                height_set = cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.request_height)
                cap.set(cv2.CAP_PROP_FPS, CAMERA_FPS)

                if not width_set or not height_set:
                    self.get_logger().warn(
                        '%s Camera resolution set request failed: requested %dx%d' %
                        (self.camera_role, self.request_width, self.request_height)
                    )

                for _ in range(5):
                    ret, _ = cap.read()
                    if ret:
                        self.get_logger().info('Opened %s Camera: %s' % (self.camera_role, camera_source))
                        self._log_actual_camera_settings(cap)
                        return cap
                    time.sleep(0.05)

                self.get_logger().warn('%s Camera opened but did not return frames: %s' % (self.camera_role, camera_source))
                cap.release()

        return None

    def _start_capture_thread(self):
        if self.capture_thread is not None and self.capture_thread.is_alive():
            return
        self.capture_stop_event.clear()
        self.capture_thread = threading.Thread(
            target=self._capture_loop,
            name=f'{self.camera_role.lower()}_camera_capture',
            daemon=True
        )
        self.capture_thread.start()

    def _capture_loop(self):
        while not self.capture_stop_event.is_set():
            with self.cap_lock:
                cap = self.cap

            if cap is None:
                time.sleep(0.1)
                continue

            ret, frame = cap.read()
            now = time.monotonic()

            if ret:
                if self.capture_last_time is not None:
                    self.capture_interval_sum += now - self.capture_last_time
                    self.capture_interval_count += 1
                self.capture_last_time = now
                self.failed_read_count = 0
                with self.latest_frame_lock:
                    self.latest_frame = frame
                    self.latest_frame_id += 1
            else:
                self.failed_read_count += 1
                if self.failed_read_count >= 5:
                    self.get_logger().warn(
                        '%s Camera read failed repeatedly. Reopening camera.' %
                        self.camera_role
                    )
                    self._release_capture()
                    new_cap = self._open_camera(self.camera_sources)
                    with self.cap_lock:
                        self.cap = new_cap
                    self.failed_read_count = 0
                    self.capture_last_time = None
                    if new_cap is None:
                        self.get_logger().error(
                            'Failed to reopen %s Camera: Camera Number %s from candidates: %s' %
                            (self.camera_role, self.camera_num_label, self.camera_sources)
                        )
                        time.sleep(0.5)
                else:
                    self.get_logger().warn(
                        '%s Camera frame read failed. count=%d' %
                        (self.camera_role, self.failed_read_count)
                    )
                    time.sleep(0.03)

    def _release_capture(self):
        with self.cap_lock:
            cap = self.cap
            self.cap = None

        if cap is not None:
            try:
                cap.release()
            except Exception as e:
                self.get_logger().warn(
                    '%s Camera release failed: %s' %
                    (self.camera_role, e)
                )

    def _log_metrics_if_due(self):
        now = time.monotonic()
        if now - self.metrics_last_log_time < METRICS_LOG_INTERVAL:
            return

        avg_capture_interval = (
            self.capture_interval_sum / self.capture_interval_count
            if self.capture_interval_count > 0 else 0.0
        )
        avg_process_time = (
            self.process_time_sum / self.process_time_count
            if self.process_time_count > 0 else 0.0
        )

        with self.cap_lock:
            cap = self.cap
            actual_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)) if cap is not None else 0
            actual_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)) if cap is not None else 0
            actual_fps = cap.get(cv2.CAP_PROP_FPS) if cap is not None else 0.0

        self.get_logger().info(
            'cam%s (%s) metrics: actual=%dx%d, fps=%.2f, '
            'avg_capture_interval=%.4fs, avg_processing_time=%.4fs, '
            'latest_frame_only=True, latest_frame_id=%d' %
            (
                self.camera_num_label,
                self.camera_role,
                actual_width,
                actual_height,
                actual_fps,
                avg_capture_interval,
                avg_process_time,
                self.latest_frame_id
            )
        )

        self.capture_interval_sum = 0.0
        self.capture_interval_count = 0
        self.process_time_sum = 0.0
        self.process_time_count = 0
        self.metrics_last_log_time = now

    def destroy_node(self):
        self.capture_stop_event.set()
        if self.capture_thread is not None:
            self.capture_thread.join(timeout=1.0)
            self.capture_thread = None
        self._release_capture()
        super().destroy_node()
        
    def timer_callback(self):
        if self.data_source == 'camera':
            with self.cap_lock:
                cap = self.cap

            if cap is None:
                if self.capture_thread is not None and self.capture_thread.is_alive():
                    self._log_metrics_if_due()
                    return

                new_cap = self._open_camera(self.camera_sources)
                with self.cap_lock:
                    self.cap = new_cap
                if new_cap is None:
                    self._log_metrics_if_due()
                    return
                self._start_capture_thread()

            with self.latest_frame_lock:
                if (
                    self.latest_frame is None
                    or self.latest_frame_id == self.last_published_frame_id
                ):
                    self._log_metrics_if_due()
                    return
                frame = self.latest_frame.copy()
                frame_id = self.latest_frame_id

            process_start = time.monotonic()
            try:
                image_msg = self.br.cv2_to_imgmsg(frame)
                image_msg.header = Header()
                image_msg.header.stamp = self.get_clock().now().to_msg()
                image_msg.header.frame_id = self.frame_id
                self.publisher.publish(image_msg)
                self.last_published_frame_id = frame_id
                if self.logger:
                    cv2.imshow('Camera Image', frame)
                    cv2.waitKey(1)
            except CvBridgeError as e:
                self.get_logger().error(
                    '%s Camera failed to convert frame: %s' %
                    (self.camera_role, e)
                )
            finally:
                self.process_time_sum += time.monotonic() - process_start
                self.process_time_count += 1
                self._log_metrics_if_due()
        elif self.data_source == 'image':
            while self.img_num < len(self.img_list):
                img_file = self.img_list[self.img_num]
                img_path = os.path.join(self.img_dir, img_file)
                img = cv2.imread(img_path)
                if img is None:
                    self.get_logger().warn('Skipping non-image file: %s' % img_file)
                else:
                    img = cv2.resize(img, (640, 480))
                    image_msg = self.br.cv2_to_imgmsg(img)
                    image_msg.header = Header()
                    image_msg.header.stamp = self.get_clock().now().to_msg()
                    image_msg.header.frame_id = self.frame_id
                    self.publisher.publish(image_msg)
                    if self.logger:
                        self.get_logger().info('Published image: %s' % img_file)
                        cv2.imshow('Saved Image', img)
                        cv2.waitKey(1)
                
                self.img_num += 1
                break
            else:
                self.img_num = 0
        elif self.data_source == 'video':
            ret, img = self.cap.read()
            if ret:
                img = cv2.resize(img, (640, 480))
                image_msg = self.br.cv2_to_imgmsg(img)
                image_msg.header = Header()
                image_msg.header.stamp = self.get_clock().now().to_msg()
                image_msg.header.frame_id = self.frame_id
                self.publisher.publish(image_msg)
                print(image_msg.header)
                if self.logger:
                    cv2.imshow('Video Frame', img)
                    cv2.waitKey(1)
            else:
                self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)  # Reset video to the first frame
    
def main(args=None):
    rclpy.init(args=args)
    node = ImagePublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n\nshutdown\n\n")
        pass
    node.destroy_node()
    if hasattr(node, 'cap') and node.cap is not None and node.cap.isOpened():
        node.cap.release()
    cv2.destroyAllWindows()
    rclpy.shutdown()
  
if __name__ == '__main__':
    main()
