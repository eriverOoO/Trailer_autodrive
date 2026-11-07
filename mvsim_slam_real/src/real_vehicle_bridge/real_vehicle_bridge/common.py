import json,signal
import rclpy
from std_msgs.msg import String
from rclpy.signals import SignalHandlerOptions
from rclpy.executors import ExternalShutdownException

def init_ros(args=None):
    # Keep context alive for a final zero command before shutdown.
    rclpy.init(args=args,signal_handler_options=SignalHandlerOptions.NO)
    def interrupted(signum,frame):raise KeyboardInterrupt
    signal.signal(signal.SIGTERM,interrupted)

def spin(cls,args=None):
    init_ros(args); node=None
    try:
        node=cls()
        while rclpy.ok():rclpy.spin_once(node,timeout_sec=.1)
    except (KeyboardInterrupt,ExternalShutdownException):
        pass
    finally:
        if node is not None:
            if hasattr(node,'close') and rclpy.ok(): node.close()
            node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()

def publish_json(pub,data):
    m=String();m.data=json.dumps(data,allow_nan=False);pub.publish(m)
