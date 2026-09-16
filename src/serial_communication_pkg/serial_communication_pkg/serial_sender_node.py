"""Send the hardware MotionCommand topic to the stroller Arduino."""
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSDurabilityPolicy, QoSReliabilityPolicy
from interfaces_pkg.msg import MotionCommand
import serial

from .protocol import encode_command


class SerialSender(Node):
    def __init__(self):
        super().__init__('serial_sender_node')
        port = self.declare_parameter('port', '/dev/ttyACM0').value
        baud = int(self.declare_parameter('baud', 115200).value)
        topic = self.declare_parameter('sub_topic', 'topic_control_signal').value
        startup_delay = float(self.declare_parameter('startup_delay', 4.5).value)
        if not port or baud <= 0 or startup_delay < 0:
            raise ValueError('Invalid serial parameters')
        self.serial = serial.Serial(port, baud, timeout=1, write_timeout=0.2)
        self.lock = threading.Lock()
        if startup_delay:
            time.sleep(startup_delay)
        self.write_stop()
        qos = QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST, durability=QoSDurabilityPolicy.VOLATILE,
            depth=1)
        self.subscription = self.create_subscription(MotionCommand, topic, self.receive, qos)
        self.get_logger().info(f'Arduino connected: {port} @ {baud} bps')

    def write(self, steering, left_speed, right_speed):
        packet = encode_command(steering, left_speed, right_speed)
        with self.lock:
            self.serial.write(packet)
            self.serial.flush()

    def write_stop(self):
        self.write(0, 0, 0)

    def receive(self, msg):
        try:
            self.write(msg.steering, msg.left_speed, msg.right_speed)
        except (ValueError, serial.SerialException, serial.SerialTimeoutException) as exc:
            self.get_logger().error(f'Hardware command rejected: {exc}')
            try:
                self.write_stop()
            except serial.SerialException:
                pass

    def close(self):
        try:
            self.write_stop()
        finally:
            self.serial.close()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = SerialSender()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        rclpy.shutdown()
