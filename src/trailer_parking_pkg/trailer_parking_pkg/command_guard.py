"""Independent wall-clock watchdog: stop even if planner stalls/sim time pauses."""
import math
import time
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from geometry_msgs.msg import Twist


class Guard(Node):
    def __init__(self):
        super().__init__('parking_command_guard')
        self.command, self.received = Twist(), 0.0
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 1)
        self.create_subscription(Twist, '/parking/raw_cmd', self.receive, 1)
        self.create_timer(0.05, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def receive(self, msg):
        if (not math.isfinite(msg.linear.x) or not math.isfinite(msg.angular.z) or
                abs(msg.linear.x) > 0.5 or abs(msg.angular.z) > 0.60):
            self.command, self.received = Twist(), 0.0
            return
        self.command, self.received = msg, time.monotonic()

    def tick(self):
        self.publisher.publish(self.command if time.monotonic()-self.received <= 0.25 else Twist())


def main(args=None):
    rclpy.init(args=args)
    node = Guard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publisher.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()
