"""Evaluate side-body boxes on one recorded C920 session; never train on it."""
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path
import sys
import time

import cv2
import numpy as np
import torch
import ultralytics
from ultralytics import YOLO

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'mvsim_slam_real/tools'))
from sensor_compare_core import Detector


def overlap(one, two):
    if one is None:
        return 0.
    w = max(0., min(one[2], two[2])-max(one[0], two[0]))
    h = max(0., min(one[3], two[3])-max(one[1], two[1]))
    intersection = w*h
    union = (one[2]-one[0])*(one[3]-one[1])+(two[2]-two[0])*(two[3]-two[1])-intersection
    return intersection/union if union>0 else 0.


def metrics(rows, threshold, indices=None):
    tp = fp = fn = tn = positives = 0
    ious = []
    failures = []
    for row in rows:
        if indices is not None and row['index'] not in indices:
            continue
        expected = row['ground_truth']
        predictions = [p for p in row['predictions'] if p['confidence']>=threshold]
        best = max((overlap(expected,p['box']) for p in predictions),default=0.)
        hit = expected is not None and best>=.5
        tp += int(hit)
        fp += len(predictions)-int(hit)
        fn += int(expected is not None and not hit)
        tn += int(expected is None and not predictions)
        if expected is not None:
            positives+=1
            ious.append(best)
        if (expected is not None and not hit) or len(predictions)>int(hit):
            failures.append(row['index'])
    return dict(positives=positives,tp=tp,fn=fn,fp=fp,tn=tn,
                precision=tp/(tp+fp) if tp+fp else None,
                recall=tp/positives if positives else None,
                mean_best_iou=float(np.mean(ious)) if ious else None,
                failure_frames=failures)


def draw_sheet(rows, files, path, threshold=.25):
    tiles=[]
    for row,file in zip(rows,files):
        frame=cv2.imread(str(file))
        expected=row['ground_truth']
        if expected is not None:
            x1,y1,x2,y2=map(int,expected)
            cv2.rectangle(frame,(x1,y1),(min(x2,639),min(y2,479)),(50,220,50),2)
        kept=[p for p in row['predictions'] if p['confidence']>=threshold]
        for p in kept:
            x1,y1,x2,y2=map(int,p['box'])
            cv2.rectangle(frame,(x1,y1),(min(x2,639),min(y2,479)),(255,160,30),2)
            cv2.putText(frame,f"{p['confidence']:.2f}",(max(x1,0),max(y1,20)),
                        cv2.FONT_HERSHEY_SIMPLEX,.5,(255,160,30),1)
        best=max((overlap(expected,p['box']) for p in kept),default=0.)
        cv2.rectangle(frame,(0,448),(640,480),(0,0,0),-1)
        cv2.putText(frame,f"{row['index']:04d} IoU={best:.2f} boxes={len(kept)}",(8,471),
                    cv2.FONT_HERSHEY_SIMPLEX,.6,(0,220,255),1)
        tiles.append(cv2.resize(frame,(320,240)))
    while len(tiles)%6:
        tiles.append(np.zeros_like(tiles[0]))
    sheet=np.vstack([np.hstack(tiles[i:i+6]) for i in range(0,len(tiles),6)])
    if not cv2.imwrite(str(path),sheet):
        raise RuntimeError(f'Cannot write {path}')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--capture',type=Path,default=ROOT/'mvsim_slam_real/artifacts/sensor_compare_push/20261118_155446')
    parser.add_argument('--output',type=Path,default=ROOT/'mvsim_slam_real/artifacts/side_vehicle_eval_20261119')
    parser.add_argument('--annotations',type=Path,default=ROOT/'docs/evaluations/side_vehicle_20261119/annotations.json')
    args=parser.parse_args()
    args.output.mkdir(parents=True,exist_ok=True)
    annotations=json.loads(args.annotations.read_text())['frames']
    indices=sorted(map(int,annotations))
    files=[args.capture/f'image_{i:04d}.jpg' for i in indices]
    if any(not p.is_file() for p in files):
        raise FileNotFoundError('Missing annotated capture image')
    cv2.setNumThreads(2)
    torch.set_num_threads(4)
    specs=[('fine_tuned',ROOT/'artifacts/ride_on_detector/ride_on_yolo11n.pt','ride_on_car'),
           ('coco_car',ROOT/'artifacts/ride_on_detector/yolo11n.pt','car'),
           ('mobilenet_car',ROOT/'mvsim_slam_real/models/mobilenet_ssd','car')]
    output={'capture':str(args.capture.relative_to(ROOT)),
            'annotation_note':'12 pre-existing manual boxes plus 31 approximate contact-sheet annotations; one session, not an independent generalization test.',
            'versions':dict(torch=torch.__version__,ultralytics=ultralytics.__version__,opencv=cv2.__version__),
            'iou_threshold':.5,'imgsz':416,'minimum_inference_confidence':.05,'models':{}}
    original_indices={0,2,5,12,17,18,19,20,26,35,38,42}
    for name,weights,label in specs:
        started=time.monotonic()
        rows=[]
        other=Counter()
        if name=='mobilenet_car':
            model=Detector(weights)
            predictions=[]
            for file in files:
                detections=model.detect(cv2.imread(str(file)),confidence=.05)
                other.update(p['label'] for p in detections if p['label']!='car' and p['confidence']>=.65)
                predictions.append([p for p in detections if p['label']=='car'])
            weight_file=weights/'mobilenet_iter_73000.caffemodel'
        else:
            model=YOLO(str(weights))
            target=next(k for k,v in model.names.items() if v==label)
            predictions=[]
            for result in model.predict([str(p) for p in files],device='cpu',imgsz=416,
                                        conf=.05,classes=[target],batch=1,verbose=False,stream=True):
                predictions.append([dict(box=box,confidence=conf,label=label)
                                    for box,conf in zip(result.boxes.xyxy.tolist(),result.boxes.conf.tolist())])
            weight_file=weights
        for index,predicted in zip(indices,predictions,strict=True):
            annotation=annotations[str(index)]
            rows.append(dict(index=index,ground_truth=annotation['box'],category=annotation['category'],predictions=predicted))
        info={'weights':str(weight_file.relative_to(ROOT)),
              'sha256':hashlib.sha256(weight_file.read_bytes()).hexdigest(),
              'elapsed_s':round(time.monotonic()-started,3),
              'thresholds':{str(t):metrics(rows,t) for t in [.25,.5,.65,.8]},
              'original_12_at_0_25':metrics(rows,.25,original_indices),
              'unoccluded_41_at_0_25':metrics(rows,.25,set(indices)-{18,19}),
              'other_labels_at_0_65':dict(other),'frames':rows}
        output['models'][name]=info
        draw_sheet(rows,files,args.output/f'{name}_all43.jpg')
        print(name,json.dumps({k:v for k,v in info.items() if k not in ('frames','weights','sha256')}),flush=True)
    (args.output/'evaluation.json').write_text(json.dumps(output,indent=2,allow_nan=False))


if __name__=='__main__':
    main()
