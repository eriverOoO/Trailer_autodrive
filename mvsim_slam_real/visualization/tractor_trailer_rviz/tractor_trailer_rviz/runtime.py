"""Process-local shutdown handling; repeated launch SIGINT must not interrupt cleanup."""
import signal
import rclpy
from rclpy.signals import SignalHandlerOptions


def run(factory,args=None):
    rclpy.init(args=args,signal_handler_options=SignalHandlerOptions.NO)
    stop=False
    def request_stop(*_):
        nonlocal stop
        stop=True
    signal.signal(signal.SIGINT,request_stop)
    signal.signal(signal.SIGTERM,request_stop)
    node=None
    try:
        node=factory()
        while rclpy.ok() and not stop:rclpy.spin_once(node,timeout_sec=.05)
    finally:
        if node:node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
