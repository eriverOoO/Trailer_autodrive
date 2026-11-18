#!/usr/bin/env python3
"""Show the latest ROS camera image with measured FPS and message age."""
import argparse
from collections import deque
import time
import tkinter as tk

import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.qos import QoSProfile
from sensor_msgs.msg import Image


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--topic', default='/parking_vision/objects_overlay')
    args = parser.parse_args()
    rclpy.init()
    node = rclpy.create_node('live_camera_view')
    bridge = CvBridge()
    latest = {}
    times = deque(maxlen=60)

    def receive(message):
        latest['image'] = bridge.imgmsg_to_cv2(message, 'passthrough').copy()
        latest['stamp'] = message.header.stamp.sec * 10**9 + message.header.stamp.nanosec
        latest['received'] = time.monotonic()
        times.append(latest['received'])

    # Both the camera and diagnostic publishers offer reliable delivery.
    subscription = node.create_subscription(Image, args.topic, receive, QoSProfile(depth=1))
    root = tk.Tk()
    root.title('Live camera - ' + args.topic)
    picture = tk.Label(root, text='Waiting for camera frames...', width=960, height=720)
    picture.pack()
    root.bind('<Escape>', lambda event: root.destroy())
    root.bind('q', lambda event: root.destroy())
    last_report = 0.0

    def update():
        nonlocal last_report
        if not rclpy.ok():
            root.destroy()
            return
        rclpy.spin_once(node, timeout_sec=0)
        if latest:
            now = time.monotonic()
            age = (node.get_clock().now().nanoseconds - latest['stamp']) / 1e9
            fps = (len(times)-1)/(times[-1]-times[0]) if len(times)>1 else 0.0
            stale = now-latest['received'] > 0.5 or age > 0.5
            text = f"{'STALE' if stale else 'LIVE'} | {fps:.1f} FPS | age {age*1000:.0f} ms"
            frame = cv2.resize(latest['image'], (960, 720))
            cv2.rectangle(frame, (0, 680), (960, 720), (0, 0, 0), -1)
            cv2.putText(frame, text, (8, 708), cv2.FONT_HERSHEY_SIMPLEX,
                        0.7, (0, 0, 255) if stale else (0, 255, 0), 2)
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            photo = tk.PhotoImage(data=b'P6\n960 720\n255\n'+rgb.tobytes(), format='PPM')
            picture.configure(image=photo, text='', width=960, height=720)
            picture.image = photo
            if now-last_report > 5:
                print(text, flush=True)
                last_report = now
        root.after(10, update)

    try:
        root.after(0, update)
        root.mainloop()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_subscription(subscription)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
