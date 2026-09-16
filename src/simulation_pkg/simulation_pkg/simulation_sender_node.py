import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSReliabilityPolicy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from interfaces_pkg.msg import MotionCommand
from .command_adapter import SendSignal


#------------- config.py에서 수정 권장 ------------
from simulation_pkg.config import SimulationSenderSettings as set
#from simulation_pkg.config import control_motor as CONTROL
#from simulation_pkg.config import convert_arduino_msg as PROTOCOL

SUB_TOPIC_NAME = set.MOTION_PLANNER_TOPIC 
PUB_TOPIC_NAME = set.GAZEBO_CONTROL_TOPIC

STEER = set.STEERING
DIRECT = set.DIRECTION

MAX_SPEED = set.MAX_SPEED
# ----------------------------------------------


class MotorControlNode(Node):
  def __init__(self, sub_topic=SUB_TOPIC_NAME, pub_topic=PUB_TOPIC_NAME):
    super().__init__('simulation_sender_node')
    
    self.declare_parameter('sub_topic', sub_topic)
    self.declare_parameter('pub_topic', pub_topic)
    self.declare_parameter('max_speed', float(MAX_SPEED))
    self.declare_parameter('max_steer', 0.6458)
    
    self.sub_topic = self.get_parameter('sub_topic').get_parameter_value().string_value
    self.pub_topic = self.get_parameter('pub_topic').get_parameter_value().string_value
    qos_profile = QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE, history=QoSHistoryPolicy.KEEP_LAST, durability=QoSDurabilityPolicy.VOLATILE, depth=1)
    
    self.simul = SendSignal(self.get_parameter('max_speed').value,
                            self.get_parameter('max_steer').value,
                            STEER, DIRECT)
    self.subscription = self.create_subscription(MotionCommand, self.sub_topic, self.data_callback, qos_profile)
    
    self.publisher = self.create_publisher(Twist, self.pub_topic, qos_profile)
    self.timer = self.create_timer(0.1, self.send_cmd_vel)
    self.velocity = Twist()
    
  def send_cmd_vel(self):
    self.publisher.publish(self.velocity)

  def data_callback(self, motor):
    angle, left, right = self.simul.process(motor)

    # MotionCommand is the hardware-facing contract.  Only this simulation
    # adapter compensates the Gazebo Classic Ackermann plugin, which applies
    # the velocity sign to its steering target internally while reversing.
    speed = (left + right) / 2.0
    self.velocity.linear.x = float(speed)
    self.velocity.angular.z = float(angle if speed >= 0.0 else -angle)
  
    self.publisher.publish(self.velocity)
    
  def stop_cmd(self):
    self.velocity.linear.x = 0.0
    
    self.publisher.publish(self.velocity)
    self.get_logger().error("\n\nRobot stopped\n\n")
    
def main(args=None):
  rclpy.init(args=args)
  node = MotorControlNode()
  try:
    rclpy.spin(node)
  except KeyboardInterrupt:
    node.stop_cmd()
    node.get_logger().fatal("\n\nsimulation_sender_node shutdown!!!\n\n")
  finally:
    node.destroy_node()
    rclpy.shutdown()
  
if __name__ == '__main__':
  main()
