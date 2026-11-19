"""Actual ONNX image inference + synthetic projection/TF/map; no devices or actuation."""
from pathlib import Path
import json
import math
import sys
from unittest.mock import Mock

import cv2
import numpy as np
import pytest

pytest.importorskip('rclpy')
pytest.importorskip('onnxruntime')
import rclpy
from rclpy.parameter import Parameter
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan

W=Path(__file__).resolve().parents[1]
ROOT=W.parent
MODEL=ROOT/'artifacts/ride_on_detector/ride_on_yolo11n.onnx'
CAPTURE=W/'artifacts/sensor_compare_push/20261118_155446/image_0012.jpg'
if not MODEL.is_file() or not CAPTURE.is_file():
    pytest.skip('Prepare the real model and capture for the runtime integration test',allow_module_level=True)
sys.path.insert(0,str(W/'src/real_parking'))
from camera_perception_pkg.parking_object_diagnostic_node import ParkingObjectDiagnostic
from real_parking.two_structure_test import TwoStructureParkingTest
from real_vehicle_bridge.geometry import VehicleGeometry
from tractor_trailer_model import Geometry


@pytest.fixture
def factory(tmp_path, monkeypatch):
    geometry=W/'src/mvsim_slam_real_bringup/config/vehicle_geometry.yaml'
    monkeypatch.setattr(Geometry,'from_world',classmethod(lambda cls,name:VehicleGeometry(name).model()))
    rclpy.init(args=['--ros-args','-p',f'world_file:={geometry}','-p',f'blocked_debug_directory:={tmp_path}'])
    calibration=tmp_path/'SYNTHETIC_ONLY.json'
    calibration.write_text(json.dumps({'image_size':[640,480], 'camera_frame':'synthetic_camera',
        'scan_frame':'laser_frame','camera_matrix':[[400,0,320],[0,400,240],[0,0,1]],
        'distortion_coefficients':[0,0,0,0,0],
        'lidar_to_camera_rotation':[[0,-1,0],[0,0,-1],[1,0,0]],
        'lidar_to_camera_translation_m':[0,0,0]}))
    nodes=[]
    def make(kind='vision', calibrated=True, wait=0.):
        if kind=='vision':
            node=ParkingObjectDiagnostic(parameter_overrides=[
                Parameter('detector_backend',value='ride_on_onnx'),
                Parameter('detector_model',value=str(MODEL)),
                Parameter('calibration_file',value=str(calibration) if calibrated else ''),
                Parameter('scan_wait_sec',value=wait),
                Parameter('detector_confidence',value=.65)])
            node.associations=Mock()
            node.overlay=Mock()
        else:
            node=TwoStructureParkingTest()
            result=node.set_parameters([Parameter('camera_object_labels',value=['ride_on_car']),
                                        Parameter('camera_min_confidence',value=.65)])
            assert all(p.successful for p in result)
        nodes.append(node)
        return node
    yield make
    for node in reversed(nodes):
        if hasattr(node,'planner_pool'):
            node.planner_pool.shutdown(wait=True,cancel_futures=True)
        node.destroy_node()
    rclpy.try_shutdown()


def stamped(message, stamp):
    message.header.stamp.sec=stamp//10**9
    message.header.stamp.nanosec=stamp%10**9
    return message


def observe(vision, offset=0, scan_delta=0, frame='synthetic_camera'):
    now=vision.get_clock().now().nanoseconds+offset
    image=stamped(CvBridge().cv2_to_imgmsg(cv2.imread(str(CAPTURE)),encoding='bgr8'),now)
    image.header.frame_id=frame
    scan=stamped(LaserScan(),now+scan_delta)
    scan.header.frame_id='laser_frame'
    scan.angle_min=0.;scan.angle_increment=.02;scan.range_min=.1;scan.range_max=12.
    scan.ranges=[2.]*5
    vision.recent_scans.clear();vision.recent_scans.append(scan)
    vision.on_image(image)
    return vision.associations.publish.call_args.args[0]


def test_real_ride_on_image_without_calibration_cannot_confirm(factory):
    vision=factory(calibrated=False)
    data=json.loads(observe(vision).data)
    assert data['image_candidates'][0]['label']=='ride_on_car'
    assert not data['calibrated'] and not data['valid'] and data['objects']==[]


def test_real_model_publishes_confident_measured_scan_group(factory):
    data=json.loads(observe(factory()).data)
    assert data['calibrated'] and data['valid']
    assert len(data['objects'])==1 and len(data['objects'][0]['scan_xy'])==5
    assert data['objects'][0]['label']=='ride_on_car' and data['objects'][0]['confidence']>=.65


@pytest.mark.parametrize('options,reason', [({'offset':-2_000_000_000},'stale'),
                                           ({'scan_delta':-150_000_000},'No scan close'),
                                           ({'frame':'wrong_frame'},'does not match')])
def test_bad_time_or_frame_cannot_produce_correspondence(factory,options,reason):
    data=json.loads(observe(factory(),**options).data)
    assert not data['valid'] and data['objects']==[] and reason in data['reason']


def test_detector_failure_clears_associations_immediately(factory):
    vision=factory()
    vision.detector.detect=Mock(side_effect=RuntimeError('detector failed'))
    data=json.loads(observe(vision).data)
    assert not data['valid'] and data['objects']==[] and data['reason']=='detector failed'


def test_camera_waits_for_later_arriving_scan_with_matching_acquisition_time(factory):
    vision=factory(wait=.15)
    now=vision.get_clock().now().nanoseconds
    image=stamped(CvBridge().cv2_to_imgmsg(cv2.imread(str(CAPTURE)),encoding='bgr8'),now)
    image.header.frame_id='synthetic_camera'
    scan=stamped(LaserScan(),now-150_000_000)
    scan.header.frame_id='laser_frame';scan.angle_min=0.;scan.angle_increment=.02
    scan.range_min=.1;scan.range_max=12.;scan.ranges=[2.]*5
    vision.recent_scans.append(scan)
    vision.on_image(image)
    assert vision.associations.publish.call_count==0
    stamped(scan,now)
    vision.recent_scans.append(scan)
    vision.process_pending_image()
    result=json.loads(vision.associations.publish.call_args.args[0].data)
    assert result['valid'] and result['scan_stamp_ns']==now and result['objects']


def test_separate_frames_with_ego_motion_confirm_one_left_gap(factory):
    vision,parking=factory(),factory('parking')
    identity=TransformStamped();identity.header.frame_id='local_map';identity.child_frame_id='odom'
    identity.transform.rotation.w=1.
    parking.association_buffer.set_transform_static(identity,'SYNTHETIC_ONLY')
    data=np.zeros((160,220),dtype=np.int16)
    data[60:76,40:60]=100     # A: x=1..2, y=2..2.8
    data[60:76,126:146]=100   # B: x=5.3..6.3, y=2..2.8; gap=3.3
    grid=OccupancyGrid();grid.header.frame_id='local_map';grid.info.resolution=.05
    grid.info.width=220;grid.info.height=160;grid.info.origin.position.x=-1.;grid.info.origin.position.y=-1.
    grid.info.origin.orientation.w=1.;grid.data=data.ravel().tolist()
    parking.detection_msg=grid
    for index,x in enumerate([1.5,5.8]):
        association=observe(vision)
        payload=json.loads(association.data)
        tf=stamped(TransformStamped(),payload['scan_stamp_ns'])
        tf.header.frame_id='odom';tf.child_frame_id='laser_frame'
        tf.transform.translation.x=x
        tf.transform.rotation.z=math.sin(math.pi/4);tf.transform.rotation.w=math.cos(math.pi/4)
        parking.association_buffer.set_transform(tf,'SYNTHETIC_ONLY')
        parking.on_associations(association)
        stamped(grid,parking.get_clock().now().nanoseconds)
        result=parking.detect_gap(np.zeros(3),2.19,.55)
        assert len(parking.evidence_in_map('local_map'))==index+1
        if index==0:
            assert result.lidar_target is not None and result.target is None
        else:
            assert result.camera_confirmed and result.target is not None
            assert result.target.length==pytest.approx(3.3,abs=.08)
    assert not parking.test_motion_enabled


@pytest.mark.parametrize('label,confidence', [('chair',.99),('white body',.99),('ride_on_car',.1),('ride_on_car',None)])
def test_wrong_or_unconfident_class_does_not_accumulate_vehicle_evidence(factory,label,confidence):
    vision,parking=factory(),factory('parking')
    message=observe(vision)
    payload=json.loads(message.data)
    payload['objects'][0].update(label=label,confidence=confidence)
    message.data=json.dumps(payload)
    parking.on_associations(message)
    assert parking.camera_evidence==[]
