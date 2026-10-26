"""SYNTHETIC_ONLY: isolated ROS domain + pseudo UART; never opens hardware."""
import importlib.util
import os
from pathlib import Path
import pty
import time
from types import SimpleNamespace

import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool

spec = importlib.util.spec_from_file_location('manual_motor_bridge', Path(__file__).parents[1] / 'tools/manual_motor_bridge.py')
m = importlib.util.module_from_spec(spec)
spec.loader.exec_module(m)


def test_conversion():
    assert m.motor_command(.08, 0., 50) == (0, 50)
    assert m.motor_command(-.06, 0., 50) == (0, -50)
    assert m.motor_command(0., 0., 50) == (None, 0)
    assert m.motor_command(.08, .03, 50)[0] < 0
    with pytest.raises(ValueError):
        m.motor_command(float('nan'), 0., 50)


def test_native_pwm_input_and_invalid_commands():
    from interfaces_pkg.msg import MotionCommand
    from types import SimpleNamespace
    bridge = SimpleNamespace(args=SimpleNamespace(pwm=50), estop=False)
    m.ManualMotorBridge.receive_pwm(bridge, MotionCommand(steering=-3, left_speed=50, right_speed=50))
    assert (bridge.requested_steering, bridge.requested_pwm) == (-3, 50)
    assert bridge.command == (50., 0.)
    for values in [(8,50,50),(0,51,51),(0,50,-50)]:
        bridge.estop = False
        m.ManualMotorBridge.receive_pwm(bridge, MotionCommand(steering=values[0], left_speed=values[1], right_speed=values[2]))
        assert bridge.estop


@pytest.mark.parametrize('input_mode', ['twist', 'pwm'])
def test_uart_timeout_readiness_clearance_and_estop(input_mode):
    assert os.environ.get('ROS_DOMAIN_ID') == '223', 'isolated domain required'
    master, slave = pty.openpty()
    os.set_blocking(master, False)
    rclpy.init()
    bridge = m.ManualMotorBridge(SimpleNamespace(port=os.ttyname(slave), pwm=50, input_mode=input_mode))
    source = rclpy.create_node('synthetic_only_manual_motor_fixture')
    ready = source.create_publisher(Bool, '/vehicle_viz_2d/ready', 1)
    scan = source.create_publisher(LaserScan, '/scan', 1)
    from interfaces_pkg.msg import MotionCommand
    command_type = MotionCommand if input_mode=='pwm' else Twist
    command = source.create_publisher(command_type, bridge.input_topic, 1)
    estop = source.create_publisher(Bool, '/parking/emergency_stop', 1)
    executor = SingleThreadedExecutor()
    executor.add_node(bridge)
    executor.add_node(source)

    def run(seconds, enabled=True, send_command=True, obstacle=False):
        packets = b''
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            ready.publish(Bool(data=enabled))
            msg = LaserScan()
            msg.header.stamp = source.get_clock().now().to_msg()
            msg.angle_min, msg.angle_increment = -3.14, .01
            msg.range_min, msg.range_max = .15, 12.
            msg.ranges = [0.5 if obstacle else 3.] * 629
            scan.publish(msg)
            if send_command:
                cmd = command_type()
                if input_mode=='pwm':
                    cmd.left_speed = cmd.right_speed = 50
                else:
                    cmd.linear.x = .08
                command.publish(cmd)
            executor.spin_once(timeout_sec=.01)
            try:
                packets += os.read(master, 65536)
            except BlockingIOError:
                pass
        return packets

    try:
        assert b's0l50r50\n' in run(1.5)
        assert run(.6, send_command=False).splitlines()[-1] == b's0l0r0'
        assert run(.4, enabled=False).splitlines()[-1] == b's0l0r0'
        assert run(.4, obstacle=True).splitlines()[-1] == b's0l0r0'
        estop.publish(Bool(data=True))
        assert run(.4).splitlines()[-1] == b's0l0r0'
    finally:
        bridge.stop()
        bridge.serial.close()
        executor.shutdown()
        bridge.destroy_node()
        source.destroy_node()
        rclpy.shutdown()
        os.close(master)
        os.close(slave)
