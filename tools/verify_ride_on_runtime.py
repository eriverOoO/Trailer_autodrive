"""Score the exact ROS ONNX backend on the recorded side-view sequence."""
import argparse
import hashlib
import json
from pathlib import Path
import statistics
import sys
import time

import cv2
import numpy as np
import onnxruntime

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'mvsim_slam_real/src/camera_perception_pkg'))
from camera_perception_pkg.ride_on_detector import RideOnOnnxDetector


def overlap(one,two):
    if one is None:
        return 0.
    intersection=max(0,min(one[2],two[2])-max(one[0],two[0]))*max(0,min(one[3],two[3])-max(one[1],two[1]))
    union=(one[2]-one[0])*(one[3]-one[1])+(two[2]-two[0])*(two[3]-two[1])-intersection
    return intersection/union if union>0 else 0.


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model',type=Path,default=ROOT/'artifacts/ride_on_detector/ride_on_yolo11n.onnx')
    parser.add_argument('--capture',type=Path,default=ROOT/'mvsim_slam_real/artifacts/sensor_compare_push/20261118_155446')
    parser.add_argument('--annotations',type=Path,default=ROOT/'docs/evaluations/side_vehicle_20261119/annotations.json')
    parser.add_argument('--confidence',type=float,default=.65)
    parser.add_argument('--output',type=Path,default=ROOT/'docs/evaluations/side_vehicle_20261119/ros_runtime_onnx.json')
    args=parser.parse_args()
    annotations=json.loads(args.annotations.read_text())['frames']
    detector=RideOnOnnxDetector(args.model,confidence=args.confidence)
    rows=[];durations=[];tp=fp=fn=tn=positives=0
    for index,truth in sorted(annotations.items(),key=lambda item:int(item[0])):
        file=args.capture/f'image_{int(index):04d}.jpg'
        image=cv2.imread(str(file))
        if image is None:
            raise FileNotFoundError(file)
        started=time.monotonic();candidates=detector.detect(image)
        durations.append((time.monotonic()-started)*1000)
        predictions=[{'box':[int(c.contour[:,0].min()),int(c.contour[:,1].min()),
                              int(c.contour[:,0].max()),int(c.contour[:,1].max())],
                      'confidence':c.confidence} for c in candidates]
        expected=truth['box']
        best=max((overlap(expected,p['box']) for p in predictions),default=0.)
        hit=expected is not None and best>=.5
        tp+=int(hit);fp+=len(predictions)-int(hit);fn+=int(expected is not None and not hit)
        tn+=int(expected is None and not predictions);positives+=int(expected is not None)
        rows.append(dict(index=int(index),ground_truth=expected,predictions=predictions,best_iou=best,hit=hit))
    result=dict(backend='ROS RideOnOnnxDetector',input_shape=[1,3,detector.size,detector.size],
                confidence=args.confidence,onnx_sha256=hashlib.sha256(args.model.read_bytes()).hexdigest(),
                versions={'onnxruntime':onnxruntime.__version__,'opencv':cv2.__version__,'numpy':np.__version__},
                tp=tp,fp=fp,fn=fn,tn=tn,positive_frames=positives,
                median_inference_ms=statistics.median(durations),frames=rows)
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(result,indent=2,allow_nan=False))
    print(json.dumps({k:v for k,v in result.items() if k!='frames'}),flush=True)


if __name__=='__main__':
    main()
