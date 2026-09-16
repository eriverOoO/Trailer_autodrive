"""Independent wall-clock watchdog for the real stroller command contract."""
import time
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from interfaces_pkg.msg import MotionCommand


class Guard(Node):
    def __init__(self):
        super().__init__('parking_command_guard')
        self.command, self.received = MotionCommand(), 0.0
        self.publisher = self.create_publisher(MotionCommand, 'topic_control_signal', 1)
        self.create_subscription(MotionCommand, '/parking/raw_motion_command', self.receive, 1)
        self.create_timer(0.05, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def receive(self, msg):
        if (abs(msg.steering) > 7 or abs(msg.left_speed) > 255 or
                abs(msg.right_speed) > 255):
            self.command, self.received = MotionCommand(), 0.0
            return
        self.command, self.received = msg, time.monotonic()

    def tick(self):
        command = self.command if time.monotonic()-self.received <= 0.25 else MotionCommand()
        self.publisher.publish(command)


def main(args=None):
    rclpy.init(args=args)
    node = Guard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publisher.publish(MotionCommand())
        node.destroy_node()
        rclpy.shutdown()
