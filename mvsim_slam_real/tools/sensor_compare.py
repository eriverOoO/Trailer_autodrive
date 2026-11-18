#!/usr/bin/env python3
"""Live object candidates next to measured 2D LiDAR groups; no vehicle commands."""
import argparse
from collections import deque
from datetime import datetime
import json
import math
from pathlib import Path
import threading
import time
import tkinter as tk
from tkinter import ttk

import cv2
from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.qos import QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import Image, LaserScan

from sensor_compare_core import Detector, message_stamp, nearest_scan, scan_groups

WORKSPACE = Path(__file__).resolve().parents[1]


class CompareWindow:
    def __init__(self, root, detector, output):
        self.root, self.detector, self.output = root, detector, output
        self.node = rclpy.create_node('sensor_compare')
        self.bridge = CvBridge()
        self.lock = threading.Lock()
        self.stop = threading.Event()
        self.scans = deque(maxlen=64)
        self.image = self.result = None
        self.confidence = .65
        self.times = deque(maxlen=60)
        self.last_report = 0.
        self.node.create_subscription(Image, '/parking_vision/image_raw', self.on_image, QoSProfile(depth=1))
        self.node.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        root.title('카메라 물체 후보 / LiDAR 실시간 비교')
        ttk.Label(root, text='왼쪽: 학습된 물체 후보와 신뢰도  |  오른쪽: 라이다 점무리와 거리 (전방 ↑, 왼쪽 ←)',
                  font=('', 12)).pack(padx=10, pady=5)
        self.picture = tk.Label(root)
        self.picture.pack()
        bar = ttk.Frame(root)
        bar.pack(fill='x', padx=10, pady=5)
        ttk.Label(bar, text='검출 신뢰도 최소값:').pack(side='left')
        self.threshold = tk.StringVar(value='0.65')
        ttk.Spinbox(bar, from_=.30, to=.95, increment=.05, textvariable=self.threshold, width=5).pack(side='left')
        ttk.Button(bar, text='현재 영상·스캔 저장', command=self.save).pack(side='left', padx=15)
        ttk.Label(root, text='보정 없음: 두 화면의 같은 물체 여부는 사람이 확인합니다. L 번호는 매 스캔 바뀔 수 있습니다.',
                  foreground='#9b5400').pack(padx=10)
        self.status = ttk.Label(root, text='실제 센서 데이터를 기다리는 중…')
        self.status.pack(padx=10, pady=5)
        self.threads = [threading.Thread(target=self.spin, daemon=True),
                        threading.Thread(target=self.infer, daemon=True)]
        for thread in self.threads:
            thread.start()
        root.protocol('WM_DELETE_WINDOW', self.close)
        root.bind('<Escape>', lambda event: self.close())
        root.after(0, self.update)

    def on_image(self, message):
        frame = self.bridge.imgmsg_to_cv2(message, 'passthrough').copy()
        if message.encoding == 'rgb8':
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        with self.lock:
            self.image = (frame, message_stamp(message), message.header.frame_id)

    def on_scan(self, message):
        with self.lock:
            self.scans.append(message)

    def spin(self):
        while not self.stop.is_set() and rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=.05)

    def infer(self):
        previous = None
        while not self.stop.is_set():
            with self.lock:
                source = self.image
            if source is None or source[1] == previous:
                self.stop.wait(.005)
                continue
            raw, stamp, frame_id = source
            started = time.monotonic()
            try:
                detections = self.detector.detect(raw, self.confidence)
            except Exception as exc:
                with self.lock:
                    self.error = str(exc)
                return
            frame = raw.copy()
            for i, item in enumerate(detections, 1):
                x1, y1, x2, y2 = item['box']
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 220, 60), 2)
                label = f"C{i} {item['label']} {item['confidence']:.0%}"
                cv2.putText(frame, label, (x1, max(20, y1-6)), cv2.FONT_HERSHEY_SIMPLEX, .55, (0, 220, 60), 2)
            now = time.monotonic()
            with self.lock:
                self.times.append(now)
                self.result = dict(raw=raw, image=frame, stamp=stamp, frame_id=frame_id,
                                   detections=detections, inference_ms=(now-started)*1000)
            previous = stamp

    def update(self):
        if self.stop.is_set():
            return
        try:
            self.confidence = float(np.clip(float(self.threshold.get()), .30, .95))
        except ValueError:
            pass
        with self.lock:
            result, scans, times = self.result, list(self.scans), list(self.times)
            error = getattr(self, 'error', None)
        canvas = np.full((520, 1160, 3), (25, 25, 25), dtype=np.uint8)
        now_ns = self.node.get_clock().now().nanoseconds
        if result:
            canvas[20:500, :640] = cv2.resize(result['image'], (640, 480))
            age = (now_ns-result['stamp'])/1e9
            fps = (len(times)-1)/(times[-1]-times[0]) if len(times)>1 else 0.
            label = f"{'LIVE' if age<.5 else 'STALE'}  {fps:.1f} FPS  age {age*1000:.0f} ms"
            cv2.putText(canvas, label, (8, 18), cv2.FONT_HERSHEY_SIMPLEX, .5,
                        (0, 230, 80) if age<.5 else (0, 0, 255), 1)
            if not result['detections']:
                cv2.putText(canvas, 'No trained class detected', (8, 517), cv2.FONT_HERSHEY_SIMPLEX, .5, (0, 210, 255), 1)
        scan = None
        if result:
            scan, delta = nearest_scan(scans, result['stamp'])
        else:
            delta = None
        # If timestamps cannot be paired, show the latest scan with an explicit warning.
        display_scan = scan if scan is not None else (scans[-1] if scans else None)
        lidar = canvas[:, 640:]
        center, scale = 260, 52.
        for radius in (1, 2, 3, 4, 5):
            cv2.circle(lidar, (center, center), int(radius*scale), (65, 65, 65), 1)
            cv2.putText(lidar, f'{radius}m', (center+3, max(12, center-int(radius*scale))),
                        cv2.FONT_HERSHEY_SIMPLEX, .35, (150, 150, 150), 1)
        cv2.arrowedLine(lidar, (260, 260), (260, 225), (0, 255, 255), 2)
        if display_scan is not None:
            labels = []
            for i, group in enumerate(scan_groups(display_scan), 1):
                color = ((73*i)%175+80, (97*i)%175+80, (131*i)%175+80)
                xy = np.column_stack((center-group[:, 1]*scale, center-group[:, 0]*scale)).astype(int)
                inside = (xy>=0).all(axis=1) & (xy<520).all(axis=1)
                for p in xy[inside]:
                    cv2.circle(lidar, tuple(p), 1, color, -1)
                if inside.any():
                    p = xy[inside][len(xy[inside])//2]
                    distance = float(np.linalg.norm(group, axis=1).min())
                    labels.append((distance, i, p, color))
            placed = []
            for distance, i, p, color in sorted(labels, key=lambda item:item[0]):
                if len(placed)>=8 or any(np.linalg.norm(p-other)<70 for other in placed):
                    continue
                p = np.clip(p, (5, 38), (420, 505))
                text = f'L{i} {distance:.2f}m'
                cv2.rectangle(lidar, (int(p[0])-2, int(p[1])-13), (int(p[0])+95, int(p[1])+3), (25,25,25), -1)
                cv2.putText(lidar, text, tuple(p), cv2.FONT_HERSHEY_SIMPLEX, .4, color, 1)
                placed.append(p)
            scan_age = (now_ns-message_stamp(display_scan))/1e9
            text = f"{'LIVE' if scan_age<.5 else 'STALE'} LiDAR age {scan_age*1000:.0f} ms"
            cv2.putText(lidar, text, (8, 18), cv2.FONT_HERSHEY_SIMPLEX, .5,
                        (0, 230, 80) if scan_age<.5 else (0, 0, 255), 1)
        else:
            cv2.putText(lidar, 'WAITING FOR REAL SCAN', (50, 200), cv2.FONT_HERSHEY_SIMPLEX, .7, (0, 0, 255), 2)
        if error:
            self.status.config(text='인식 모델 오류: '+error)
        elif result:
            matched = f'영상–스캔 시각 차이 {abs(delta)*1000:.0f}ms' if scan is not None else '영상과 100ms 이내 스캔 없음 — 시간 대응 불가'
            self.status.config(text=f"물체 후보 {len(result['detections'])}개 · {matched} · 자동 공간 대응은 미보정 상태")
            if time.monotonic()-self.last_report>5:
                print(json.dumps({'fps':round(fps,1),'image_age_ms':round(age*1000),
                                  'inference_ms':round(result['inference_ms'],1),
                                  'detections':result['detections'],'scan_delta_s':delta,
                                  'scan_available':display_scan is not None}), flush=True)
                cv2.imwrite('/tmp/trailer_sensor_compare_preview.jpg', canvas)
                self.last_report = time.monotonic()
        rgb = cv2.cvtColor(canvas, cv2.COLOR_BGR2RGB)
        photo = tk.PhotoImage(data=b'P6\n1160 520\n255\n'+rgb.tobytes(), format='PPM')
        self.picture.config(image=photo)
        self.picture.image = photo
        self.root.after(25, self.update)

    def save(self):
        with self.lock:
            result, scans = self.result, list(self.scans)
        if result is None or (self.node.get_clock().now().nanoseconds-result['stamp'])/1e9>.5:
            self.status.config(text='저장 실패: 최신 영상이 없습니다.')
            return
        scan, delta = nearest_scan(scans, result['stamp'])
        if scan is None:
            self.status.config(text='저장 실패: 영상과 100ms 이내의 실제 스캔이 없습니다.')
            return
        directory = self.output/datetime.now().strftime('%Y%m%d_%H%M%S_%f')
        directory.mkdir(parents=True)
        cv2.imwrite(str(directory/'camera.png'), result['raw'])
        cv2.imwrite(str(directory/'detections.png'), result['image'])
        record = {key:result[key] for key in ('stamp', 'frame_id', 'detections', 'inference_ms')}
        record.update(calibrated=False, automatic_object_association=False, scan_delta_s=delta,
                      scan={'stamp_ns':message_stamp(scan), 'frame_id':scan.header.frame_id,
                            **{key:float(getattr(scan,key)) for key in ('angle_min','angle_increment','range_min','range_max','scan_time','time_increment')},
                            'ranges':[float(v) if math.isfinite(v) else None for v in scan.ranges]})
        (directory/'observation.json').write_text(json.dumps(record,indent=2,allow_nan=False))
        print('SAVED', directory, flush=True)
        # Keep the path visible independently of the periodic live status.
        self.root.title('저장 완료: '+str(directory))

    def close(self):
        self.stop.set()
        for thread in self.threads:
            thread.join(timeout=2)
        self.node.destroy_node()
        rclpy.try_shutdown()
        self.root.destroy()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model-dir', type=Path, default=WORKSPACE/'models/mobilenet_ssd')
    parser.add_argument('--output', type=Path, default=WORKSPACE/'artifacts/sensor_compare')
    args = parser.parse_args()
    cv2.setNumThreads(2)
    detector = Detector(args.model_dir)
    rclpy.init()
    root = tk.Tk()
    window = CompareWindow(root, detector, args.output)
    try:
        root.mainloop()
    except KeyboardInterrupt:
        window.close()


if __name__ == '__main__':
    main()
